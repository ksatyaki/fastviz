//! Mesh pass: renders all `Mesh` entities. Each entity owns its own GPU vertex
//! and index buffers, allocated lazily on first sight of a new entity ID and
//! re-uploaded only when the entity's `revision` advances past the cached
//! `uploaded_revision` — so URDF link geometry (which is static after load) is
//! streamed to the GPU exactly once, even though `entity.transform` may change
//! every frame as joint state / TF updates flow in.

use std::collections::{HashMap, HashSet};
use std::mem::size_of;

use bytemuck::{Pod, Zeroable};
use scene::{EntityId, Mesh, SceneGraph, ScenePrimitive, Vertex};

use crate::gpu::{GpuContext, DEPTH_FORMAT};

#[repr(C)]
#[derive(Copy, Clone, PartialEq, Pod, Zeroable)]
struct InstanceUniform {
    model: [[f32; 4]; 4],
    color: [f32; 4],
}

struct SharedGeometry {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    
    instance_buffer: wgpu::Buffer,
    instance_capacity: usize,
    
    instances: Vec<InstanceUniform>,
}

pub struct MeshPass {
    fill_pipeline: wgpu::RenderPipeline,
    geometries: HashMap<u64, SharedGeometry>,
    
    /// Maps an EntityId to its geometry hash so we know which SharedGeometry to use
    entity_to_hash: HashMap<EntityId, u64>,
    /// We also track the revision of the entity when we computed its hash,
    /// so we can re-hash if the primitive changes.
    entity_revisions: HashMap<EntityId, u64>,

    last_revision: Option<u64>,
}

impl MeshPass {
    pub fn new(gpu: &GpuContext, camera_bgl: &wgpu::BindGroupLayout) -> Self {
        let device = &gpu.device;
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("lit-shader-mesh"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../../shaders/lit.wgsl").into()),
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("mesh-pl"),
            bind_group_layouts: &[camera_bgl],
            push_constant_ranges: &[],
        });

        let vbuffers = [
            wgpu::VertexBufferLayout {
                array_stride: size_of::<Vertex>() as u64,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &wgpu::vertex_attr_array![
                    0 => Float32x3,
                    1 => Float32x3,
                    2 => Float32x2,
                ],
            },
            wgpu::VertexBufferLayout {
                array_stride: size_of::<InstanceUniform>() as u64,
                step_mode: wgpu::VertexStepMode::Instance,
                attributes: &wgpu::vertex_attr_array![
                    3 => Float32x4,
                    4 => Float32x4,
                    5 => Float32x4,
                    6 => Float32x4,
                    7 => Float32x4,
                ],
            },
        ];

        let fill_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("mesh-fill-pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                compilation_options: Default::default(),
                buffers: &vbuffers,
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_main",
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: gpu.surface_config.format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                // Backface culling disabled: URDF meshes have unreliable
                // winding (Collada exporters and many ROS meshes are not
                // consistent about it), so culling tends to hide geometry.
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::LessEqual,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState {
                count: gpu.sample_count,
                ..Default::default()
            },
            multiview: None,
            cache: None,
        });

        MeshPass {
            fill_pipeline,
            geometries: HashMap::new(),
            entity_to_hash: HashMap::new(),
            entity_revisions: HashMap::new(),
            last_revision: None,
        }
    }

    pub fn prepare(&mut self, gpu: &GpuContext, scene: &SceneGraph) {
        let revision = scene.revision();
        if self.last_revision == Some(revision) {
            return;
        }
        self.last_revision = Some(revision);

        for geo in self.geometries.values_mut() {
            geo.instances.clear();
        }

        let mut seen: HashSet<EntityId> = HashSet::new();

        for entity in scene.entities.values() {
            let ScenePrimitive::Mesh(mesh) = &entity.primitive else { continue };
            seen.insert(entity.id);
            
            let mut current_hash = *self.entity_to_hash.get(&entity.id).unwrap_or(&0);
            let last_rev = *self.entity_revisions.get(&entity.id).unwrap_or(&0);
            
            if last_rev != entity.revision || !self.geometries.contains_key(&current_hash) {
                current_hash = mesh.geometry_hash();
                self.entity_to_hash.insert(entity.id, current_hash);
                self.entity_revisions.insert(entity.id, entity.revision);
                
                if !self.geometries.contains_key(&current_hash) {
                    let mut geo = create_shared_geometry(gpu, &mesh);
                    self.geometries.insert(current_hash, geo);
                }
            }
            
            if !entity.visible {
                continue;
            }

            let inst = InstanceUniform {
                model: entity.transform.to_cols_array_2d(),
                color: mesh.material.base_color.to_array(),
            };
            self.geometries.get_mut(&current_hash).unwrap().instances.push(inst);
        }

        self.entity_to_hash.retain(|id, _| seen.contains(id));
        self.entity_revisions.retain(|id, _| seen.contains(id));
        
        // Remove unused geometries
        self.geometries.retain(|_, geo| !geo.instances.is_empty());

        for geo in self.geometries.values_mut() {
            if geo.instances.is_empty() {
                continue;
            }
            if geo.instances.len() > geo.instance_capacity {
                geo.instance_capacity = (geo.instances.len() * 2).next_power_of_two();
                geo.instance_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("mesh-instance-vb"),
                    size: (geo.instance_capacity * size_of::<InstanceUniform>()) as u64,
                    usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
            }
            gpu.queue.write_buffer(&geo.instance_buffer, 0, bytemuck::cast_slice(&geo.instances));
        }
    }

    pub fn draw<'a>(
        &'a self,
        rpass: &mut wgpu::RenderPass<'a>,
        camera_bg: &'a wgpu::BindGroup,
        _scene: &SceneGraph,
    ) {
        if self.geometries.is_empty() {
            return;
        }
        rpass.set_pipeline(&self.fill_pipeline);
        rpass.set_bind_group(0, camera_bg, &[]);
        
        for geo in self.geometries.values() {
            if geo.instances.is_empty() {
                continue;
            }
            rpass.set_vertex_buffer(0, geo.vertex_buffer.slice(..));
            rpass.set_vertex_buffer(1, geo.instance_buffer.slice(..));
            rpass.set_index_buffer(geo.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
            rpass.draw_indexed(0..geo.index_count, 0, 0..geo.instances.len() as u32);
        }
    }
}

fn create_shared_geometry(
    gpu: &GpuContext,
    mesh: &Mesh,
) -> SharedGeometry {
    let vertex_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("mesh-vb"),
        size: ((mesh.vertices.len().max(1)) * size_of::<Vertex>()) as u64,
        usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let index_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("mesh-ib"),
        size: ((mesh.indices.len().max(1)) * size_of::<u32>()) as u64,
        usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    gpu.queue.write_buffer(&vertex_buffer, 0, bytemuck::cast_slice(&mesh.vertices));
    gpu.queue.write_buffer(&index_buffer, 0, bytemuck::cast_slice(&mesh.indices));

    let instance_capacity = 64usize;
    let instance_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("mesh-instance-vb"),
        size: (instance_capacity * size_of::<InstanceUniform>()) as u64,
        usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    SharedGeometry {
        vertex_buffer,
        index_buffer,
        index_count: mesh.indices.len() as u32,
        instance_buffer,
        instance_capacity,
        instances: Vec::new(),
    }
}
