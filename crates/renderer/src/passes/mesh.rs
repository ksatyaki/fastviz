//! Mesh pass: renders all `Mesh` entities. Each entity owns its own GPU vertex
//! and index buffers, allocated lazily on first sight of a new entity ID and
//! re-uploaded only when the entity's `dirty` flag is set.

use std::collections::{HashMap, HashSet};
use std::mem::size_of;

use bytemuck::{Pod, Zeroable};
use scene::{EntityId, Mesh, SceneGraph, ScenePrimitive, Vertex};

use crate::gpu::{GpuContext, DEPTH_FORMAT};

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct InstanceUniform {
    model: [[f32; 4]; 4],
    color: [f32; 4],
}

struct GpuMesh {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    instance_buffer: wgpu::Buffer,
    instance_bind_group: wgpu::BindGroup,
    wireframe: bool,
}

pub struct MeshPass {
    fill_pipeline: wgpu::RenderPipeline,
    instance_bgl: wgpu::BindGroupLayout,
    meshes: HashMap<EntityId, GpuMesh>,
}

impl MeshPass {
    pub fn new(gpu: &GpuContext, camera_bgl: &wgpu::BindGroupLayout) -> Self {
        let device = &gpu.device;
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("lit-shader-mesh"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../../shaders/lit.wgsl").into()),
        });

        let instance_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("mesh-instance-bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("mesh-pl"),
            bind_group_layouts: &[camera_bgl, &instance_bgl],
            push_constant_ranges: &[],
        });

        let vbuffers = [wgpu::VertexBufferLayout {
            array_stride: size_of::<Vertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &wgpu::vertex_attr_array![
                0 => Float32x3,
                1 => Float32x3,
                2 => Float32x2,
            ],
        }];

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
                // Backface culling disabled: URDF meshes go through
                // `ROS_TO_WORLD` (a reflection, det = -1), which flips winding
                // and would otherwise show only the inside of every mesh.
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
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        MeshPass {
            fill_pipeline,
            instance_bgl,
            meshes: HashMap::new(),
        }
    }

    pub fn prepare(&mut self, gpu: &GpuContext, scene: &SceneGraph) {
        let mut seen: HashSet<EntityId> = HashSet::new();

        for entity in scene.entities.values() {
            let ScenePrimitive::Mesh(mesh) = &entity.primitive else { continue };
            seen.insert(entity.id);
            let inst = InstanceUniform {
                model: entity.transform.to_cols_array_2d(),
                color: mesh.material.base_color.to_array(),
            };

            if !self.meshes.contains_key(&entity.id) {
                self.meshes
                    .insert(entity.id, create_gpu_mesh(gpu, &self.instance_bgl, mesh));
            }
            let gm = self.meshes.get_mut(&entity.id).unwrap();
            if entity.visible && entity.dirty {
                upload_mesh_geometry(gpu, gm, mesh);
            }
            gpu.queue
                .write_buffer(&gm.instance_buffer, 0, bytemuck::bytes_of(&inst));
            gm.wireframe = mesh.material.wireframe;
        }

        // Drop GPU resources for entities that disappeared.
        self.meshes.retain(|id, _| seen.contains(id));
    }

    pub fn draw<'a>(
        &'a self,
        rpass: &mut wgpu::RenderPass<'a>,
        camera_bg: &'a wgpu::BindGroup,
        scene: &SceneGraph,
    ) {
        if self.meshes.is_empty() {
            return;
        }
        rpass.set_pipeline(&self.fill_pipeline);
        rpass.set_bind_group(0, camera_bg, &[]);
        for (id, gm) in &self.meshes {
            // Honor visibility from the scene graph.
            if let Some(e) = scene.entities.get(id) {
                if !e.visible {
                    continue;
                }
            }
            rpass.set_bind_group(1, &gm.instance_bind_group, &[]);
            rpass.set_vertex_buffer(0, gm.vertex_buffer.slice(..));
            rpass.set_index_buffer(gm.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
            rpass.draw_indexed(0..gm.index_count, 0, 0..1);
        }
    }
}

fn create_gpu_mesh(
    gpu: &GpuContext,
    instance_bgl: &wgpu::BindGroupLayout,
    mesh: &Mesh,
) -> GpuMesh {
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
    let instance_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("mesh-instance-ub"),
        size: size_of::<InstanceUniform>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let instance_bind_group = gpu.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("mesh-instance-bg"),
        layout: instance_bgl,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: instance_buffer.as_entire_binding(),
        }],
    });

    let mut gm = GpuMesh {
        vertex_buffer,
        index_buffer,
        index_count: 0,
        instance_buffer,
        instance_bind_group,
        wireframe: mesh.material.wireframe,
    };
    upload_mesh_geometry(gpu, &mut gm, mesh);
    gm
}

fn upload_mesh_geometry(gpu: &GpuContext, gm: &mut GpuMesh, mesh: &Mesh) {
    let vbytes = bytemuck::cast_slice(&mesh.vertices);
    let ibytes = bytemuck::cast_slice(&mesh.indices);
    if (gm.vertex_buffer.size() as usize) < vbytes.len() {
        gm.vertex_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("mesh-vb"),
            size: vbytes.len() as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
    }
    if (gm.index_buffer.size() as usize) < ibytes.len() {
        gm.index_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("mesh-ib"),
            size: ibytes.len() as u64,
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
    }
    gpu.queue.write_buffer(&gm.vertex_buffer, 0, vbytes);
    gpu.queue.write_buffer(&gm.index_buffer, 0, ibytes);
    gm.index_count = mesh.indices.len() as u32;
}
