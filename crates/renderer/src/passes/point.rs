//! Point pass: instanced quads in screen space. Per-instance attributes are
//! local-space position, color, and pixel size; a per-entity transform uniform
//! takes the local position to world space in the vertex shader.
//!
//! The pass caches its prepared state and only rebuilds when `SceneGraph::revision`
//! advances, so steady-state redraw against an unchanging scene costs only the
//! draw call itself (no CPU repack, no GPU upload).

use std::mem::size_of;

use bytemuck::{Pod, Zeroable};
use scene::{SceneGraph, ScenePrimitive};

use crate::gpu::{GpuContext, DEPTH_FORMAT};

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct ScreenUniform {
    viewport: [f32; 2],
    _pad: [f32; 2],
}

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct CornerVertex {
    corner: [f32; 2],
}

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct PointInstance {
    pos: [f32; 3],
    color: [f32; 4],
    size: f32,
}

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct TransformUniform {
    matrix: [[f32; 4]; 4],
}

struct DrawSlice {
    instance_offset: u32,
    instance_count: u32,
    /// Dynamic offset (in bytes) into the transform UBO for this entity.
    transform_dyn_offset: u32,
}

pub struct PointPass {
    pipeline: wgpu::RenderPipeline,
    quad_vb: wgpu::Buffer,

    instance_buffer: wgpu::Buffer,
    instance_capacity: usize,
    instances: Vec<PointInstance>,

    transform_buffer: wgpu::Buffer,
    transform_bgl: wgpu::BindGroupLayout,
    transform_bg: wgpu::BindGroup,
    transform_capacity: usize,
    /// Padded slot size = align_up(sizeof::<TransformUniform>(),
    /// device.min_uniform_buffer_offset_alignment).
    transform_slot_size: u64,
    transform_staging: Vec<u8>,

    /// Per-entity draw ranges, refreshed on rebuild.
    draws: Vec<DrawSlice>,
    /// `Some(rev)` when the buffers reflect that scene revision; `None` until
    /// the first prepare. Only a revision change triggers a rebuild + upload.
    last_revision: Option<u64>,

    screen_buffer: wgpu::Buffer,
    screen_bg: wgpu::BindGroup,
    last_viewport: [u32; 2],
}

impl PointPass {
    pub fn new(gpu: &GpuContext, camera_bgl: &wgpu::BindGroupLayout) -> Self {
        let device = &gpu.device;
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("point-shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../../shaders/point.wgsl").into()),
        });

        let screen_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("screen-bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });

        let transform_min_binding =
            std::num::NonZeroU64::new(size_of::<TransformUniform>() as u64);
        let transform_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("point-transform-bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: true,
                    min_binding_size: transform_min_binding,
                },
                count: None,
            }],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("point-pl"),
            bind_group_layouts: &[camera_bgl, &screen_bgl, &transform_bgl],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("point-pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                compilation_options: Default::default(),
                buffers: &[
                    wgpu::VertexBufferLayout {
                        array_stride: size_of::<CornerVertex>() as u64,
                        step_mode: wgpu::VertexStepMode::Vertex,
                        attributes: &wgpu::vertex_attr_array![0 => Float32x2],
                    },
                    wgpu::VertexBufferLayout {
                        array_stride: size_of::<PointInstance>() as u64,
                        step_mode: wgpu::VertexStepMode::Instance,
                        attributes: &wgpu::vertex_attr_array![
                            1 => Float32x3,
                            2 => Float32x4,
                            3 => Float32,
                        ],
                    },
                ],
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
                topology: wgpu::PrimitiveTopology::TriangleStrip,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::LessEqual,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let corners = [
            CornerVertex {
                corner: [-1.0, -1.0],
            },
            CornerVertex {
                corner: [1.0, -1.0],
            },
            CornerVertex {
                corner: [-1.0, 1.0],
            },
            CornerVertex {
                corner: [1.0, 1.0],
            },
        ];
        let quad_vb = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("point-corner-vb"),
            size: (corners.len() * size_of::<CornerVertex>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        gpu.queue
            .write_buffer(&quad_vb, 0, bytemuck::cast_slice(&corners));

        let instance_capacity = 4096usize;
        let instance_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("point-instance-vb"),
            size: (instance_capacity * size_of::<PointInstance>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let alignment = device.limits().min_uniform_buffer_offset_alignment as u64;
        let transform_slot_size = align_up(size_of::<TransformUniform>() as u64, alignment);
        let transform_capacity = 8usize;
        let transform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("point-transform-ub"),
            size: transform_slot_size * transform_capacity as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let transform_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("point-transform-bg"),
            layout: &transform_bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                    buffer: &transform_buffer,
                    offset: 0,
                    size: transform_min_binding,
                }),
            }],
        });

        let screen_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("screen-ub"),
            size: size_of::<ScreenUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let screen_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("screen-bg"),
            layout: &screen_bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: screen_buffer.as_entire_binding(),
            }],
        });

        PointPass {
            pipeline,
            quad_vb,
            instance_buffer,
            instance_capacity,
            instances: Vec::new(),
            transform_buffer,
            transform_bgl,
            transform_bg,
            transform_capacity,
            transform_slot_size,
            transform_staging: Vec::new(),
            draws: Vec::new(),
            last_revision: None,
            screen_buffer,
            screen_bg,
            last_viewport: [0, 0],
        }
    }

    pub fn prepare(&mut self, gpu: &GpuContext, scene: &SceneGraph) {
        // Screen UBO: only re-upload on viewport change. Cheap either way.
        let viewport = [gpu.surface_config.width, gpu.surface_config.height];
        if viewport != self.last_viewport {
            self.last_viewport = viewport;
            let screen = ScreenUniform {
                viewport: [viewport[0] as f32, viewport[1] as f32],
                _pad: [0.0, 0.0],
            };
            gpu.queue
                .write_buffer(&self.screen_buffer, 0, bytemuck::bytes_of(&screen));
        }

        let revision = scene.revision();
        if self.last_revision == Some(revision) {
            return; // scene unchanged — keep cached buffers + draw list
        }
        self.last_revision = Some(revision);

        self.instances.clear();
        self.draws.clear();

        // Walk visible Points entities, packing per-instance data and recording
        // the transform slot for each. Position stays in entity-local coords;
        // the vertex shader applies entity.transform via a uniform.
        let mut transform_count: usize = 0;
        for entity in scene.entities.values() {
            if !entity.visible {
                continue;
            }
            let ScenePrimitive::Points(points) = &entity.primitive else {
                continue;
            };
            if points.is_empty() {
                continue;
            }

            let instance_offset = self.instances.len() as u32;
            self.instances.extend(points.iter().map(|p| PointInstance {
                pos: [p.position.x, p.position.y, p.position.z],
                color: p.color.to_array(),
                size: p.size.max(1.0),
            }));
            let instance_count = self.instances.len() as u32 - instance_offset;
            let transform_dyn_offset = (transform_count as u64 * self.transform_slot_size) as u32;
            self.draws.push(DrawSlice {
                instance_offset,
                instance_count,
                transform_dyn_offset,
            });
            transform_count += 1;
        }

        // Resize instance buffer if needed.
        if self.instances.len() > self.instance_capacity {
            self.instance_capacity = (self.instances.len() * 2).next_power_of_two();
            self.instance_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("point-instance-vb"),
                size: (self.instance_capacity * size_of::<PointInstance>()) as u64,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
        }
        if !self.instances.is_empty() {
            gpu.queue.write_buffer(
                &self.instance_buffer,
                0,
                bytemuck::cast_slice(&self.instances),
            );
        }

        // Resize transform UBO + rebuild bind group if needed.
        if transform_count > self.transform_capacity {
            self.transform_capacity = (transform_count * 2).next_power_of_two().max(8);
            self.transform_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("point-transform-ub"),
                size: self.transform_slot_size * self.transform_capacity as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            self.transform_bg = gpu.device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("point-transform-bg"),
                layout: &self.transform_bgl,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                        buffer: &self.transform_buffer,
                        offset: 0,
                        size: std::num::NonZeroU64::new(size_of::<TransformUniform>() as u64),
                    }),
                }],
            });
        }

        // Pack transforms into a contiguous staging buffer with each Mat4 at a
        // dyn-offset-aligned slot.
        if transform_count > 0 {
            let total_bytes = transform_count * self.transform_slot_size as usize;
            self.transform_staging.clear();
            self.transform_staging.resize(total_bytes, 0);
            let mut idx = 0;
            for entity in scene.entities.values() {
                if !entity.visible {
                    continue;
                }
                let ScenePrimitive::Points(points) = &entity.primitive else {
                    continue;
                };
                if points.is_empty() {
                    continue;
                }
                let t = TransformUniform {
                    matrix: entity.transform.to_cols_array_2d(),
                };
                let off = idx * self.transform_slot_size as usize;
                self.transform_staging[off..off + size_of::<TransformUniform>()]
                    .copy_from_slice(bytemuck::bytes_of(&t));
                idx += 1;
            }
            gpu.queue
                .write_buffer(&self.transform_buffer, 0, &self.transform_staging);
        }
    }

    pub fn draw<'a>(
        &'a self,
        rpass: &mut wgpu::RenderPass<'a>,
        camera_bg: &'a wgpu::BindGroup,
    ) {
        if self.draws.is_empty() {
            return;
        }
        rpass.set_pipeline(&self.pipeline);
        rpass.set_bind_group(0, camera_bg, &[]);
        rpass.set_bind_group(1, &self.screen_bg, &[]);
        rpass.set_vertex_buffer(0, self.quad_vb.slice(..));
        rpass.set_vertex_buffer(1, self.instance_buffer.slice(..));
        for d in &self.draws {
            rpass.set_bind_group(2, &self.transform_bg, &[d.transform_dyn_offset]);
            rpass.draw(0..4, d.instance_offset..(d.instance_offset + d.instance_count));
        }
    }
}

fn align_up(n: u64, align: u64) -> u64 {
    n.div_ceil(align) * align
}
