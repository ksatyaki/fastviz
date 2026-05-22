//! ROS2 ingestion crate for fastviz.
//!
//! Owns a dedicated thread that runs the r2r executor. Subscribers convert
//! ROS messages into `scene::ScenePrimitive`s and write them through a shared
//! `SceneHandle`. The render thread is never blocked beyond a short write
//! lock per message.

pub mod config;
pub mod config_writer;
pub mod coords;
pub mod ids;
pub mod node;
pub mod stats;
pub mod subscribers;
pub mod tf;
pub mod tf_axes;
pub mod tf_refresh;
pub mod urdf;

pub use config::{RosConfig, UiGroup};
pub use config_writer::{
    to_toml as config_to_toml, to_toml_full as config_to_toml_full, TopicKind, UiGroupSave,
};
pub use ids::{TF_FRAME_BASE, TF_FRAME_CAPACITY, URDF_LINK_BASE};
pub use node::{RosNode, TopicsSnapshot};
pub use stats::RosStats;
