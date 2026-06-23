//! A static reference grid on the XZ plane. Lines every 1m (major) and every
//! 0.1m (minor). Drawn as a single line-list.
//!
//! Independent of the scene graph — always rendered, but visibility and extent
//! are configurable.

use wgpu::util::DeviceExt;

use super::LineVertex;
use crate::gpu::{GpuContext, DEPTH_FORMAT};

pub struct ReferenceGridPass {
    pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    vertex_count: u32,
    pub visible: bool,
}

impl ReferenceGridPass {
    pub fn new(gpu: &GpuContext, camera_bgl: &wgpu::BindGroupLayout) -> Self {
        let device = &gpu.device;
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("line-shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../../shaders/line.wgsl").into()),
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("reference-grid-pl"),
            bind_group_layouts: &[camera_bgl],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("reference-grid-pipeline"),
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
                depth_write_enabled: false,
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

        let verts = build_grid_vertices(10.0, 1.0, 0.1);
        let vertex_count = verts.len() as u32;
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("reference-grid-vb"),
            contents: bytemuck::cast_slice(&verts),
            usage: wgpu::BufferUsages::VERTEX,
        });

        ReferenceGridPass {
            pipeline,
            vertex_buffer,
            vertex_count,
            visible: true,
        }
    }

    pub fn draw<'a>(
        &'a self,
        rpass: &mut wgpu::RenderPass<'a>,
        camera_bg: &'a wgpu::BindGroup,
    ) {
        if !self.visible {
            return;
        }
        rpass.set_pipeline(&self.pipeline);
        rpass.set_bind_group(0, camera_bg, &[]);
        rpass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        rpass.draw(0..self.vertex_count, 0..1);
    }
}

fn build_grid_vertices(extent: f32, major: f32, minor: f32) -> Vec<LineVertex> {
    let mut out = Vec::new();
    let major_color = [0.45, 0.45, 0.50, 1.0];
    let minor_color = [0.25, 0.25, 0.28, 1.0];
    let axis_x = [0.85, 0.20, 0.20, 1.0];
    let axis_z = [0.20, 0.40, 0.85, 1.0];

    let n_minor = (extent / minor) as i32;
    for i in -n_minor..=n_minor {
        let t = i as f32 * minor;
        // Skip lines that will be drawn as major or axes (avoid Z-fighting).
        if (t.abs() < 1e-4) || ((t / major).round() * major - t).abs() < 1e-4 {
            continue;
        }
        out.push(LineVertex { position: [t, 0.0, -extent], color: minor_color });
        out.push(LineVertex { position: [t, 0.0,  extent], color: minor_color });
        out.push(LineVertex { position: [-extent, 0.0, t], color: minor_color });
        out.push(LineVertex { position: [ extent, 0.0, t], color: minor_color });
    }

    let n_major = (extent / major) as i32;
    for i in -n_major..=n_major {
        let t = i as f32 * major;
        if t.abs() < 1e-4 {
            continue;
        }
        out.push(LineVertex { position: [t, 0.0, -extent], color: major_color });
        out.push(LineVertex { position: [t, 0.0,  extent], color: major_color });
        out.push(LineVertex { position: [-extent, 0.0, t], color: major_color });
        out.push(LineVertex { position: [ extent, 0.0, t], color: major_color });
    }

    // Origin axes (red = +X, blue = +Z).
    out.push(LineVertex { position: [-extent, 0.0, 0.0], color: axis_x });
    out.push(LineVertex { position: [ extent, 0.0, 0.0], color: axis_x });
    out.push(LineVertex { position: [0.0, 0.0, -extent], color: axis_z });
    out.push(LineVertex { position: [0.0, 0.0,  extent], color: axis_z });

    out
}
