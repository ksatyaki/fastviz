//! GPU rendering foundation for fastviz.
//!
//! The renderer is the only crate that knows about wgpu. It consumes the
//! `scene::SceneGraph` and turns it into draw calls. It knows nothing about
//! ROS2 or any specific data source.

pub mod camera;
pub mod gpu;
pub mod passes;
pub mod renderer;

pub use camera::{CameraUniform, OrbitCamera};
pub use gpu::GpuContext;
pub use renderer::{FrameStats, OverlayContext, Renderer};
