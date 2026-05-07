//! Convention for mapping ROS REP-103 coordinates onto fastviz world space.
//!
//! ROS frames are right-handed Z-up (X forward, Y left, Z up). The fastviz
//! renderer is Y-up with the ground plane on XZ. M0.5's pragmatic mapping:
//!
//! ```text
//!     ROS x → world x
//!     ROS y → world z
//!     ROS z → world y   (up stays up)
//! ```
//!
//! `ROS_TO_WORLD` is a reflection (det = -1). Acceptable here because we
//! consume orientation only as visual indicators; nothing in M0.5 depends on
//! handedness. Subscribers compose:
//!
//! ```text
//!     entity.transform = ROS_TO_WORLD * tf_in_ros * primitive_pose_in_ros * ROS_TO_WORLD
//! ```
//!
//! so the renderer's local-XZ-plane primitives end up correctly placed in
//! world after the lookup.

use glam::{Mat4, Vec4};

pub const ROS_TO_WORLD: Mat4 = Mat4::from_cols(
    Vec4::new(1.0, 0.0, 0.0, 0.0),
    Vec4::new(0.0, 0.0, 1.0, 0.0),
    Vec4::new(0.0, 1.0, 0.0, 0.0),
    Vec4::new(0.0, 0.0, 0.0, 1.0),
);
