//! Point pass: instanced quads in screen space. Per-instance attributes are
//! position, color, and pixel size. Used for laser scans and any future point
//! cloud rendering.

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

pub struct PointPass {
    pipeline: wgpu::RenderPipeline,
    quad_vb: wgpu::Buffer,
    instance_buffer: wgpu::Buffer,
    capacity: usize,
    live: u32,

    screen_buffer: wgpu::Buffer,
    screen_bg: wgpu::BindGroup,
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

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("point-pl"),
            bind_group_layouts: &[camera_bgl, &screen_bgl],
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
            CornerVertex { corner: [-1.0, -1.0] },
            CornerVertex { corner: [ 1.0, -1.0] },
            CornerVertex { corner: [-1.0,  1.0] },
            CornerVertex { corner: [ 1.0,  1.0] },
        ];
        let quad_vb = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("point-corner-vb"),
            size: (corners.len() * size_of::<CornerVertex>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        gpu.queue
            .write_buffer(&quad_vb, 0, bytemuck::cast_slice(&corners));

        let initial_cap = 4096usize;
        let instance_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("point-instance-vb"),
            size: (initial_cap * size_of::<PointInstance>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
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
            capacity: initial_cap,
            live: 0,
            screen_buffer,
            screen_bg,
        }
    }

    pub fn prepare(&mut self, gpu: &GpuContext, scene: &SceneGraph) {
        let mut instances: Vec<PointInstance> = Vec::new();
        for entity in scene.entities.values() {
            if !entity.visible {
                continue;
            }
            if let ScenePrimitive::Points(points) = &entity.primitive {
                let m = entity.transform;
                for p in points {
                    let v = m * glam::Vec4::new(p.position.x, p.position.y, p.position.z, 1.0);
                    instances.push(PointInstance {
                        pos: [v.x, v.y, v.z],
                        color: p.color.to_array(),
                        size: p.size.max(1.0),
                    });
                }
            }
        }

        if instances.len() > self.capacity {
            self.capacity = (instances.len() * 2).next_power_of_two();
            self.instance_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("point-instance-vb"),
                size: (self.capacity * size_of::<PointInstance>()) as u64,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
        }

        if !instances.is_empty() {
            gpu.queue
                .write_buffer(&self.instance_buffer, 0, bytemuck::cast_slice(&instances));
        }
        self.live = instances.len() as u32;

        let screen = ScreenUniform {
            viewport: [
                gpu.surface_config.width as f32,
                gpu.surface_config.height as f32,
            ],
            _pad: [0.0, 0.0],
        };
        gpu.queue
            .write_buffer(&self.screen_buffer, 0, bytemuck::bytes_of(&screen));
    }

    pub fn draw<'a>(
        &'a self,
        rpass: &mut wgpu::RenderPass<'a>,
        camera_bg: &'a wgpu::BindGroup,
    ) {
        if self.live == 0 {
            return;
        }
        rpass.set_pipeline(&self.pipeline);
        rpass.set_bind_group(0, camera_bg, &[]);
        rpass.set_bind_group(1, &self.screen_bg, &[]);
        rpass.set_vertex_buffer(0, self.quad_vb.slice(..));
        rpass.set_vertex_buffer(1, self.instance_buffer.slice(..));
        rpass.draw(0..4, 0..self.live);
    }
}
