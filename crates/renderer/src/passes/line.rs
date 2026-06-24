//! Line pass: renders all `Polyline` and `Frame` (3-axis) entities from the
//! scene graph as instanced quads to support configurable line thickness.

use glam::{Vec3, Vec4};
use scene::{Color, SceneGraph, ScenePrimitive};
use std::mem::size_of;

use crate::gpu::{GpuContext, DEPTH_FORMAT};

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct QuadVertex {
    position: [f32; 2],
}

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct LineInstance {
    start: [f32; 3],
    end: [f32; 3],
    color: [f32; 4],
    thickness: f32,
}

pub struct LinePass {
    pipeline: wgpu::RenderPipeline,
    quad_vertex_buffer: wgpu::Buffer,
    quad_index_buffer: wgpu::Buffer,
    instance_buffer: wgpu::Buffer,
    capacity: usize,
    instance_count: u32,
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
                buffers: &[
                    wgpu::VertexBufferLayout {
                        array_stride: size_of::<QuadVertex>() as u64,
                        step_mode: wgpu::VertexStepMode::Vertex,
                        attributes: &wgpu::vertex_attr_array![
                            0 => Float32x2,
                        ],
                    },
                    wgpu::VertexBufferLayout {
                        array_stride: size_of::<LineInstance>() as u64,
                        step_mode: wgpu::VertexStepMode::Instance,
                        attributes: &wgpu::vertex_attr_array![
                            1 => Float32x3,
                            2 => Float32x3,
                            3 => Float32x4,
                            4 => Float32,
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
                topology: wgpu::PrimitiveTopology::TriangleList,
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

        let quad_verts = [
            QuadVertex { position: [0.0, -0.5] },
            QuadVertex { position: [1.0, -0.5] },
            QuadVertex { position: [0.0,  0.5] },
            QuadVertex { position: [1.0,  0.5] },
        ];
        let quad_indices: [u32; 6] = [0, 1, 2, 2, 1, 3];

        let quad_vertex_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("line-quad-vb"),
            size: (quad_verts.len() * size_of::<QuadVertex>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        gpu.queue.write_buffer(&quad_vertex_buffer, 0, bytemuck::cast_slice(&quad_verts));

        let quad_index_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("line-quad-ib"),
            size: (quad_indices.len() * size_of::<u32>()) as u64,
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        gpu.queue.write_buffer(&quad_index_buffer, 0, bytemuck::cast_slice(&quad_indices));

        let initial_capacity = 4096;
        let instance_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("line-instance-vb"),
            size: (initial_capacity * size_of::<LineInstance>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        LinePass {
            pipeline,
            quad_vertex_buffer,
            quad_index_buffer,
            instance_buffer,
            capacity: initial_capacity,
            instance_count: 0,
        }
    }

    pub fn prepare(&mut self, gpu: &GpuContext, scene: &SceneGraph) {
        let mut instances: Vec<LineInstance> = Vec::with_capacity(self.capacity);

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
                    let thickness = p.width;
                    for pair in p.points.windows(2) {
                        let a = transform_point(entity.transform, pair[0]);
                        let b = transform_point(entity.transform, pair[1]);
                        instances.push(LineInstance {
                            start: a.into(),
                            end: b.into(),
                            color,
                            thickness,
                        });
                    }
                }
                ScenePrimitive::Frame(f) => {
                    push_frame_axes(&mut instances, entity.transform * f.transform, f.axis_length);
                }
                _ => {}
            }
        }

        self.instance_count = instances.len() as u32;
        if instances.is_empty() {
            return;
        }

        if instances.len() > self.capacity {
            self.capacity = (instances.len() * 2).next_power_of_two();
            self.instance_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("line-instance-vb"),
                size: (self.capacity * size_of::<LineInstance>()) as u64,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
        }

        gpu.queue
            .write_buffer(&self.instance_buffer, 0, bytemuck::cast_slice(&instances));
    }

    pub fn draw<'a>(
        &'a self,
        rpass: &mut wgpu::RenderPass<'a>,
        camera_bg: &'a wgpu::BindGroup,
    ) {
        if self.instance_count == 0 {
            return;
        }
        rpass.set_pipeline(&self.pipeline);
        rpass.set_bind_group(0, camera_bg, &[]);
        rpass.set_vertex_buffer(0, self.quad_vertex_buffer.slice(..));
        rpass.set_vertex_buffer(1, self.instance_buffer.slice(..));
        rpass.set_index_buffer(self.quad_index_buffer.slice(..), wgpu::IndexFormat::Uint32);
        rpass.draw_indexed(0..6, 0, 0..self.instance_count);
    }
}

fn transform_point(m: glam::Mat4, p: Vec3) -> Vec3 {
    let v = m * Vec4::new(p.x, p.y, p.z, 1.0);
    Vec3::new(v.x, v.y, v.z)
}

fn push_frame_axes(out: &mut Vec<LineInstance>, transform: glam::Mat4, length: f32) {
    let origin = transform_point(transform, Vec3::ZERO);
    let x = transform_point(transform, Vec3::new(length, 0.0, 0.0));
    let y = transform_point(transform, Vec3::new(0.0, length, 0.0));
    let z = transform_point(transform, Vec3::new(0.0, 0.0, length));

    let red = Color::RED.to_array();
    let green = Color::GREEN.to_array();
    let blue = Color::BLUE.to_array();

    // Default thickness for frames could be something like 0.01 * length or a fixed value.
    let thickness = (length * 0.05).clamp(0.005, 0.05);

    out.push(LineInstance { start: origin.into(), end: x.into(), color: red, thickness });
    out.push(LineInstance { start: origin.into(), end: y.into(), color: green, thickness });
    out.push(LineInstance { start: origin.into(), end: z.into(), color: blue, thickness });
}
