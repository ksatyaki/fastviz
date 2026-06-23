//! Arrow pass: renders all `Arrow` and `Vec<Arrow>` entities. Geometry is a
//! shared unit-arrow mesh (length 1, oriented along +X), drawn with hardware
//! instancing: every arrow is one element in a single per-instance vertex
//! buffer (model matrix + color), and the whole scene's arrows go out in a
//! single instanced draw call.
//!
//! Like [`PointPass`](super::point::PointPass), the prepared buffer is cached
//! and only rebuilt + re-uploaded when `SceneGraph::revision` advances — so
//! redrawing an unchanged scene (renderer running faster than producers
//! publish) costs only the one draw call, with no CPU repack and no GPU upload.

use std::mem::size_of;

use bytemuck::{Pod, Zeroable};
use glam::{Mat4, Quat, Vec3};
use scene::{Arrow, SceneGraph, ScenePrimitive, Vertex};

use crate::gpu::{GpuContext, DEPTH_FORMAT};

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct ArrowInstance {
    model: [[f32; 4]; 4],
    color: [f32; 4],
}

pub struct ArrowPass {
    pipeline: wgpu::RenderPipeline,

    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,

    instance_buffer: wgpu::Buffer,
    instance_capacity: usize,
    instances: Vec<ArrowInstance>,
    /// Number of instances in `instance_buffer` to draw.
    live: u32,
    /// `Some(rev)` when the buffer reflects that scene revision; `None` until
    /// the first prepare. Only a revision change triggers a rebuild + upload.
    last_revision: Option<u64>,
}

impl ArrowPass {
    pub fn new(gpu: &GpuContext, camera_bgl: &wgpu::BindGroupLayout) -> Self {
        let device = &gpu.device;
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("arrow-shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../../shaders/arrow.wgsl").into()),
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("arrow-pl"),
            bind_group_layouts: &[camera_bgl],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("arrow-pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                compilation_options: Default::default(),
                buffers: &[
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
                        array_stride: size_of::<ArrowInstance>() as u64,
                        step_mode: wgpu::VertexStepMode::Instance,
                        // model matrix columns (3..=6) + color (7).
                        attributes: &wgpu::vertex_attr_array![
                            3 => Float32x4,
                            4 => Float32x4,
                            5 => Float32x4,
                            6 => Float32x4,
                            7 => Float32x4,
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
                cull_mode: Some(wgpu::Face::Back),
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

        let (verts, idx) = build_unit_arrow();
        let vertex_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("arrow-vb"),
            size: (verts.len() * size_of::<Vertex>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let index_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("arrow-ib"),
            size: (idx.len() * size_of::<u32>()) as u64,
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        gpu.queue
            .write_buffer(&vertex_buffer, 0, bytemuck::cast_slice(&verts));
        gpu.queue
            .write_buffer(&index_buffer, 0, bytemuck::cast_slice(&idx));

        let instance_capacity = 256usize;
        let instance_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("arrow-instance-vb"),
            size: (instance_capacity * size_of::<ArrowInstance>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        ArrowPass {
            pipeline,
            vertex_buffer,
            index_buffer,
            index_count: idx.len() as u32,
            instance_buffer,
            instance_capacity,
            instances: Vec::new(),
            live: 0,
            last_revision: None,
        }
    }

    pub fn prepare(&mut self, gpu: &GpuContext, scene: &SceneGraph) {
        let revision = scene.revision();
        if self.last_revision == Some(revision) {
            return; // scene unchanged — keep cached instance buffer + draw count
        }
        self.last_revision = Some(revision);

        self.instances.clear();
        for entity in scene.entities.values() {
            if !entity.visible {
                continue;
            }
            if let ScenePrimitive::Arrows(arrows) = &entity.primitive {
                for a in arrows {
                    self.instances.push(arrow_instance(a, entity.transform));
                }
            }
        }
        self.live = self.instances.len() as u32;

        // Grow the instance buffer if needed (power-of-two, like the point pass).
        if self.instances.len() > self.instance_capacity {
            self.instance_capacity = (self.instances.len() * 2).next_power_of_two();
            self.instance_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("arrow-instance-vb"),
                size: (self.instance_capacity * size_of::<ArrowInstance>()) as u64,
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
        rpass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        rpass.set_vertex_buffer(1, self.instance_buffer.slice(..));
        rpass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
        rpass.draw_indexed(0..self.index_count, 0, 0..self.live);
    }
}

fn arrow_instance(a: &Arrow, parent: Mat4) -> ArrowInstance {
    let dir = if a.direction.length_squared() < 1e-8 {
        Vec3::X
    } else {
        a.direction.normalize()
    };
    let rot = Quat::from_rotation_arc(Vec3::X, dir);
    // Local mesh shaft radius is 0.5; head_radius_factor is baked at 1.6.
    // We scale isotropically in cross-section by `shaft_radius * 2`.
    let scale = Vec3::new(a.length, a.shaft_radius * 2.0, a.shaft_radius * 2.0);
    let local = Mat4::from_scale_rotation_translation(scale, rot, a.origin);
    ArrowInstance {
        model: (parent * local).to_cols_array_2d(),
        color: a.color.to_array(),
    }
}

/// A unit arrow lying along +X, total length 1.0. Shaft radius 0.5 (local Y/Z),
/// head radius 1.6× shaft radius. Caller scales.
fn build_unit_arrow() -> (Vec<Vertex>, Vec<u32>) {
    let segments = 12u32;
    let shaft_end = 0.7f32;
    let radius = 0.5f32;
    let head_radius = radius * 1.6;

    let mut verts: Vec<Vertex> = Vec::new();
    let mut idx: Vec<u32> = Vec::new();

    // Shaft side.
    let ring_start = verts.len() as u32;
    for i in 0..segments {
        let theta = (i as f32) / (segments as f32) * std::f32::consts::TAU;
        let (s, c) = theta.sin_cos();
        let n = [0.0, c, s];
        verts.push(Vertex::new([0.0, c * radius, s * radius], n, [0.0, 0.0]));
        verts.push(Vertex::new([shaft_end, c * radius, s * radius], n, [1.0, 0.0]));
    }
    for i in 0..segments {
        let next = (i + 1) % segments;
        let a = ring_start + i * 2;
        let b = ring_start + next * 2;
        idx.extend_from_slice(&[a, a + 1, b]);
        idx.extend_from_slice(&[b, a + 1, b + 1]);
    }

    // Shaft back cap (faces -X).
    let cap_center = verts.len() as u32;
    verts.push(Vertex::new([0.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.5, 0.5]));
    let cap_ring = verts.len() as u32;
    for i in 0..segments {
        let theta = (i as f32) / (segments as f32) * std::f32::consts::TAU;
        let (s, c) = theta.sin_cos();
        verts.push(Vertex::new(
            [0.0, c * radius, s * radius],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0],
        ));
    }
    for i in 0..segments {
        let next = (i + 1) % segments;
        idx.extend_from_slice(&[cap_center, cap_ring + next, cap_ring + i]);
    }

    // Head base disk (closes the cone, facing -X).
    let head_base_center = verts.len() as u32;
    verts.push(Vertex::new(
        [shaft_end, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.5, 0.5],
    ));
    let head_base = verts.len() as u32;
    for i in 0..segments {
        let theta = (i as f32) / (segments as f32) * std::f32::consts::TAU;
        let (s, c) = theta.sin_cos();
        verts.push(Vertex::new(
            [shaft_end, c * head_radius, s * head_radius],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0],
        ));
    }
    for i in 0..segments {
        let next = (i + 1) % segments;
        idx.extend_from_slice(&[head_base_center, head_base + i, head_base + next]);
    }

    // Cone slant.
    let slant_len = (1.0 - shaft_end).hypot(head_radius);
    let nx = head_radius / slant_len;
    let nr = (1.0 - shaft_end) / slant_len;
    let head_tip = verts.len() as u32;
    verts.push(Vertex::new([1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, 1.0]));
    let head_side = verts.len() as u32;
    for i in 0..segments {
        let theta = (i as f32) / (segments as f32) * std::f32::consts::TAU;
        let (s, c) = theta.sin_cos();
        let n = [nx, c * nr, s * nr];
        verts.push(Vertex::new(
            [shaft_end, c * head_radius, s * head_radius],
            n,
            [0.0, 0.0],
        ));
    }
    for i in 0..segments {
        let next = (i + 1) % segments;
        idx.extend_from_slice(&[head_side + i, head_tip, head_side + next]);
    }

    (verts, idx)
}
