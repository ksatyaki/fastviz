use bytemuck::{Pod, Zeroable};
use glam::{Mat4, Vec2, Vec3};

use crate::color::{Color, Colormap};

/// A single colored point in 3D space.
#[derive(Clone, Debug)]
pub struct Point {
    pub position: Vec3,
    pub color: Color,
    pub size: f32,
}

/// A polyline — ordered sequence of points connected by line segments.
#[derive(Clone, Debug)]
pub struct Polyline {
    pub points: Vec<Vec3>,
    pub color: Color,
    pub width: f32,
}

/// An arrow (pose indicator).
///
/// `direction` is normalized; `length` encodes scale separately.
#[derive(Clone, Debug)]
pub struct Arrow {
    pub origin: Vec3,
    pub direction: Vec3,
    pub length: f32,
    pub shaft_radius: f32,
    pub head_radius: f32,
    pub color: Color,
}

/// A flat grid aligned to the XY plane.
#[derive(Clone, Debug)]
pub struct Grid {
    pub origin: Vec2,
    pub cell_size: f32,
    pub cols: u32,
    pub rows: u32,
    pub data: GridData,
}

#[derive(Clone, Debug)]
pub enum GridData {
    Uniform(Color),
    Cells(Vec<u8>, Colormap),
}

/// Opaque GPU texture handle. The renderer maps these to wgpu textures.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct TextureHandle(pub u64);

/// Per-vertex attributes used by the mesh pass.
#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub uv: [f32; 2],
}

impl Vertex {
    pub const fn new(position: [f32; 3], normal: [f32; 3], uv: [f32; 2]) -> Self {
        Vertex { position, normal, uv }
    }
}

#[derive(Clone, Debug)]
pub struct Material {
    pub base_color: Color,
    pub texture: Option<TextureHandle>,
    pub wireframe: bool,
}

impl Default for Material {
    fn default() -> Self {
        Material {
            base_color: Color::WHITE,
            texture: None,
            wireframe: false,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub material: Material,
}

/// A screen-space or world-space text label.
#[derive(Clone, Debug)]
pub struct Label {
    pub position: Vec3,
    pub text: String,
    pub color: Color,
    pub scale: f32,
}

/// A coordinate frame indicator (3 colored axes).
#[derive(Clone, Debug)]
pub struct Frame {
    pub transform: Mat4,
    pub axis_length: f32,
    pub label: Option<String>,
}
