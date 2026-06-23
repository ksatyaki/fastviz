//! Line pass: renders all `Polyline` and `Frame` (3-axis) entities from the
//! scene graph as a single line-list draw.
//!
//! For now we rebuild the vertex buffer every frame. We can move to per-entity
//! buffers + dirty tracking once the workload demands it.

use glam::{Vec3, Vec4};
use scene::{Color, SceneGraph, ScenePrimitive};

use super::LineVertex;
use crate::gpu::{GpuContext, DEPTH_FORMAT};

pub struct LinePass {
    pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    capacity: usize,
    vertex_count: u32,
}

impl LinePass {
    pub fn new(gpu: &GpuContext, camera_bgl: &wgpu::BindGroupLayout) -> Self {
        let device = &gpu.device;
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("line-shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../../shaders/line.wgsl").into()),
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("line-pl"),
            bind_group_layouts: &[camera_bgl],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("line-pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                compilation_options: Default::default(),
                buffers: &[LineVertex::LAYOUT],
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
                topology: wgpu::PrimitiveTopology::LineList,
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

        let initial_capacity = 4096;
        let vertex_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("line-vb"),
            size: (initial_capacity * std::mem::size_of::<LineVertex>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        LinePass {
            pipeline,
            vertex_buffer,
            capacity: initial_capacity,
            vertex_count: 0,
        }
    }

    pub fn prepare(&mut self, gpu: &GpuContext, scene: &SceneGraph) {
        let mut verts: Vec<LineVertex> = Vec::with_capacity(self.capacity);

        for entity in scene.entities.values() {
            if !entity.visible {
                continue;
            }
            match &entity.primitive {
                ScenePrimitive::Polyline(p) => {
                    if p.points.len() < 2 {
                        continue;
                    }
                    let color = p.color.to_array();
                    for pair in p.points.windows(2) {
                        let a = transform_point(entity.transform, pair[0]);
                        let b = transform_point(entity.transform, pair[1]);
                        verts.push(LineVertex { position: a.into(), color });
                        verts.push(LineVertex { position: b.into(), color });
                    }
                }
                ScenePrimitive::Frame(f) => {
                    push_frame_axes(&mut verts, entity.transform * f.transform, f.axis_length);
                }
                _ => {}
            }
        }

        self.vertex_count = verts.len() as u32;
        if verts.is_empty() {
            return;
        }

        if verts.len() > self.capacity {
            self.capacity = (verts.len() * 2).next_power_of_two();
            self.vertex_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("line-vb"),
                size: (self.capacity * std::mem::size_of::<LineVertex>()) as u64,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
        }

        gpu.queue
            .write_buffer(&self.vertex_buffer, 0, bytemuck::cast_slice(&verts));
    }

    pub fn draw<'a>(
        &'a self,
        rpass: &mut wgpu::RenderPass<'a>,
        camera_bg: &'a wgpu::BindGroup,
    ) {
        if self.vertex_count == 0 {
            return;
        }
        rpass.set_pipeline(&self.pipeline);
        rpass.set_bind_group(0, camera_bg, &[]);
        rpass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        rpass.draw(0..self.vertex_count, 0..1);
    }
}

fn transform_point(m: glam::Mat4, p: Vec3) -> Vec3 {
    let v = m * Vec4::new(p.x, p.y, p.z, 1.0);
    Vec3::new(v.x, v.y, v.z)
}

fn push_frame_axes(out: &mut Vec<LineVertex>, transform: glam::Mat4, length: f32) {
    let origin = transform_point(transform, Vec3::ZERO);
    let x = transform_point(transform, Vec3::new(length, 0.0, 0.0));
    let y = transform_point(transform, Vec3::new(0.0, length, 0.0));
    let z = transform_point(transform, Vec3::new(0.0, 0.0, length));

    let red = Color::RED.to_array();
    let green = Color::GREEN.to_array();
    let blue = Color::BLUE.to_array();

    out.push(LineVertex { position: origin.into(), color: red });
    out.push(LineVertex { position: x.into(), color: red });
    out.push(LineVertex { position: origin.into(), color: green });
    out.push(LineVertex { position: y.into(), color: green });
    out.push(LineVertex { position: origin.into(), color: blue });
    out.push(LineVertex { position: z.into(), color: blue });
}
