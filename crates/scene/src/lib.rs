//! Scene graph and ROS-agnostic primitives.
//!
//! The render pipeline consumes only the types defined here. ROS message types
//! are mapped onto these primitives by the ingestion layer; they never reach
//! the renderer directly.

pub mod color;
pub mod graph;
pub mod primitives;

pub use color::{Color, Colormap};
pub use graph::{
    apply_color, apply_scale, primitive_color, primitive_scale, EntityId, SceneEntity, SceneGraph,
    SceneHandle, ScenePrimitive, StyleOverride,
};
pub use primitives::{
    Arrow, Frame, Grid, GridData, Label, Material, Mesh, Point, Polyline, TextureHandle, Vertex,
};
