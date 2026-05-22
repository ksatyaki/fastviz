//! Occupancy / costmap pass: renders `Grid` entities (Cells variant) as a
//! textured quad on the XZ plane. The colormap is baked CPU-side into an RGBA
//! texture so the fragment shader is trivial.

use std::collections::{HashMap, HashSet};
use std::mem::size_of;

use bytemuck::{Pod, Zeroable};
use scene::{Color, Colormap, EntityId, Grid, GridData, SceneGraph, ScenePrimitive};

use crate::gpu::{GpuContext, DEPTH_FORMAT};

#[repr(C)]
#[derive(Copy, Clone, PartialEq, Pod, Zeroable)]
struct GridUniform {
    model: [[f32; 4]; 4],
    tint: [f32; 4],
}

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct QuadVertex {
    pos: [f32; 3],
    uv: [f32; 2],
}

struct GpuGrid {
    uniform_buffer: wgpu::Buffer,
    texture: wgpu::Texture,
    bind_group: wgpu::BindGroup,
    cols: u32,
    rows: u32,
    /// `entity.revision` of the texels currently sitting on the GPU. The
    /// prepare loop only re-bakes + re-uploads when this falls behind, so a
    /// static map (one publish, many frames) costs zero per-frame upload.
    uploaded_revision: u64,
    /// Last uniform we wrote to the UBO. Maps rarely move once placed, so
    /// skipping unchanged uniform writes keeps the queue quiet too.
    last_uniform: Option<GridUniform>,
}

pub struct OccupancyPass {
    pipeline: wgpu::RenderPipeline,
    bgl: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    quad_vb: wgpu::Buffer,
    quad_ib: wgpu::Buffer,
    grids: HashMap<EntityId, GpuGrid>,
}

impl OccupancyPass {
    pub fn new(gpu: &GpuContext, camera_bgl: &wgpu::BindGroupLayout) -> Self {
        let device = &gpu.device;
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("occupancy-shader"),
            source: wgpu::ShaderSource::Wgsl(
                include_str!("../../shaders/occupancy.wgsl").into(),
            ),
        });

        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("occupancy-bgl"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: false },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::NonFiltering),
                    count: None,
                },
            ],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("occupancy-pl"),
            bind_group_layouts: &[camera_bgl, &bgl],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("occupancy-pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                compilation_options: Default::default(),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: size_of::<QuadVertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![
                        0 => Float32x3,
                        1 => Float32x2,
                    ],
                }],
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
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::LessEqual,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        // A unit quad in XZ plane, centered at origin, side length 1.
        let verts = [
            QuadVertex { pos: [-0.5, 0.0, -0.5], uv: [0.0, 0.0] },
            QuadVertex { pos: [ 0.5, 0.0, -0.5], uv: [1.0, 0.0] },
            QuadVertex { pos: [ 0.5, 0.0,  0.5], uv: [1.0, 1.0] },
            QuadVertex { pos: [-0.5, 0.0,  0.5], uv: [0.0, 1.0] },
        ];
        let idx: [u32; 6] = [0, 1, 2, 0, 2, 3];

        let quad_vb = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("occupancy-quad-vb"),
            size: (verts.len() * size_of::<QuadVertex>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let quad_ib = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("occupancy-quad-ib"),
            size: (idx.len() * size_of::<u32>()) as u64,
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        gpu.queue.write_buffer(&quad_vb, 0, bytemuck::cast_slice(&verts));
        gpu.queue.write_buffer(&quad_ib, 0, bytemuck::cast_slice(&idx));

        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("occupancy-sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Nearest,
            min_filter: wgpu::FilterMode::Nearest,
            ..Default::default()
        });

        OccupancyPass {
            pipeline,
            bgl,
            sampler,
            quad_vb,
            quad_ib,
            grids: HashMap::new(),
        }
    }

    pub fn prepare(&mut self, gpu: &GpuContext, scene: &SceneGraph) {
        let mut seen: HashSet<EntityId> = HashSet::new();

        for entity in scene.entities.values() {
            let ScenePrimitive::Grid(grid) = &entity.primitive else { continue };
            seen.insert(entity.id);

            // Re-create the GPU grid when dimensions change OR when no entry
            // exists yet. A dimension change forces a fresh texture; we mark
            // the new entry as "stale" (uploaded_revision = revision - 1) so
            // the upload branch below runs once.
            let needs_new = match self.grids.get(&entity.id) {
                Some(g) => g.cols != grid.cols || g.rows != grid.rows,
                None => true,
            };
            if needs_new {
                let mut gg = create_gpu_grid(gpu, &self.bgl, &self.sampler, grid);
                gg.uploaded_revision = entity.revision.wrapping_sub(1);
                self.grids.insert(entity.id, gg);
            }

            let gg = self.grids.get_mut(&entity.id).unwrap();

            // Re-bake + re-upload only when the payload has actually changed
            // since the last frame we saw it. For a typical static map (one
            // publish, latched), this branch runs exactly once.
            if gg.uploaded_revision != entity.revision {
                let pixels = bake_pixels(grid);
                gpu.queue.write_texture(
                    wgpu::ImageCopyTexture {
                        texture: &gg.texture,
                        mip_level: 0,
                        origin: wgpu::Origin3d::ZERO,
                        aspect: wgpu::TextureAspect::All,
                    },
                    &pixels,
                    wgpu::ImageDataLayout {
                        offset: 0,
                        bytes_per_row: Some(grid.cols * 4),
                        rows_per_image: Some(grid.rows),
                    },
                    wgpu::Extent3d {
                        width: grid.cols,
                        height: grid.rows,
                        depth_or_array_layers: 1,
                    },
                );
                gg.uploaded_revision = entity.revision;
            }

            // Update uniform: model matrix that places the unit quad on the XY-aligned
            // grid in world space. Grid.origin is the *center* of the grid in XZ.
            let width = grid.cols as f32 * grid.cell_size;
            let height = grid.rows as f32 * grid.cell_size;
            let translate = glam::Mat4::from_translation(glam::Vec3::new(
                grid.origin.x,
                0.0,
                grid.origin.y,
            ));
            let scale = glam::Mat4::from_scale(glam::Vec3::new(width, 1.0, height));
            let local = translate * scale;
            let model = entity.transform * local;
            let uniform = GridUniform {
                model: model.to_cols_array_2d(),
                tint: Color::rgba(1.0, 1.0, 1.0, 0.85).to_array(),
            };
            if gg.last_uniform != Some(uniform) {
                gpu.queue
                    .write_buffer(&gg.uniform_buffer, 0, bytemuck::bytes_of(&uniform));
                gg.last_uniform = Some(uniform);
            }
        }

        self.grids.retain(|id, _| seen.contains(id));
    }

    pub fn draw<'a>(
        &'a self,
        rpass: &mut wgpu::RenderPass<'a>,
        camera_bg: &'a wgpu::BindGroup,
        scene: &SceneGraph,
    ) {
        if self.grids.is_empty() {
            return;
        }
        rpass.set_pipeline(&self.pipeline);
        rpass.set_bind_group(0, camera_bg, &[]);
        rpass.set_vertex_buffer(0, self.quad_vb.slice(..));
        rpass.set_index_buffer(self.quad_ib.slice(..), wgpu::IndexFormat::Uint32);
        for (id, gg) in &self.grids {
            if let Some(e) = scene.entities.get(id) {
                if !e.visible {
                    continue;
                }
            }
            rpass.set_bind_group(1, &gg.bind_group, &[]);
            rpass.draw_indexed(0..6, 0, 0..1);
        }
    }
}

fn create_gpu_grid(
    gpu: &GpuContext,
    bgl: &wgpu::BindGroupLayout,
    sampler: &wgpu::Sampler,
    grid: &Grid,
) -> GpuGrid {
    let texture = gpu.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("occupancy-tex"),
        size: wgpu::Extent3d {
            width: grid.cols,
            height: grid.rows,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    let uniform_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("occupancy-ub"),
        size: size_of::<GridUniform>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let bind_group = gpu.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("occupancy-bg"),
        layout: bgl,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(&view),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::Sampler(sampler),
            },
        ],
    });
    GpuGrid {
        uniform_buffer,
        texture,
        bind_group,
        cols: grid.cols,
        rows: grid.rows,
        uploaded_revision: 0,
        last_uniform: None,
    }
}

fn bake_pixels(grid: &Grid) -> Vec<u8> {
    let n = (grid.cols * grid.rows) as usize;
    let mut out = Vec::with_capacity(n * 4);
    match &grid.data {
        GridData::Uniform(c) => {
            for _ in 0..n {
                out.extend_from_slice(&[
                    (c.r * 255.0) as u8,
                    (c.g * 255.0) as u8,
                    (c.b * 255.0) as u8,
                    (c.a * 255.0) as u8,
                ]);
            }
        }
        GridData::Cells(cells, cmap) => {
            let take = cells.len().min(n);
            for &v in cells.iter().take(take) {
                let c = cmap.sample(v);
                out.extend_from_slice(&[
                    (c.r * 255.0) as u8,
                    (c.g * 255.0) as u8,
                    (c.b * 255.0) as u8,
                    (c.a * 255.0) as u8,
                ]);
            }
            // Pad if the cells array is shorter than declared.
            for _ in take..n {
                out.extend_from_slice(&[128, 128, 128, 128]);
            }
        }
    }
    out
}

fn _ensure_colormap_used(_: Colormap) {}
