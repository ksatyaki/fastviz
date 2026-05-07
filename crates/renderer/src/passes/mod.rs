//! Render passes. Each pass owns its pipeline, vertex buffers, and per-frame
//! upload logic. They all share the camera bind group at group 0.

pub mod arrow;
pub mod line;
pub mod mesh;
pub mod occupancy;
pub mod point;
pub mod reference_grid;

pub use arrow::ArrowPass;
pub use line::LinePass;
pub use mesh::MeshPass;
pub use occupancy::OccupancyPass;
pub use point::PointPass;
pub use reference_grid::ReferenceGridPass;

/// A vertex used by the line and reference-grid passes: position + color.
#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct LineVertex {
    pub position: [f32; 3],
    pub color: [f32; 4],
}

impl LineVertex {
    pub const LAYOUT: wgpu::VertexBufferLayout<'static> = wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<LineVertex>() as u64,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &wgpu::vertex_attr_array![
            0 => Float32x3,
            1 => Float32x4,
        ],
    };
}
