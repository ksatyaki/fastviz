//! ROS2 ingestion crate for fastviz.
//!
//! Owns a dedicated thread that runs the r2r executor. Subscribers convert
//! ROS messages into `scene::ScenePrimitive`s and write them through a shared
//! `SceneHandle`. The render thread is never blocked beyond a short write
//! lock per message.

pub mod config;
pub mod coords;
pub mod ids;
pub mod node;
pub mod stats;
pub mod subscribers;
pub mod tf;
pub mod tf_refresh;
pub mod urdf;

pub use config::RosConfig;
pub use node::RosNode;
pub use stats::RosStats;
