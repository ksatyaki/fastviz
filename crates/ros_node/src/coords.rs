//! Convention for mapping ROS REP-103 coordinates onto fastviz world space.
//!
//! ROS frames are right-handed Z-up (X forward, Y left, Z up). The fastviz
//! renderer is Y-up. We use a proper rotation (det = +1) so handedness is
//! preserved and a top-down camera shows ROS +Y as screen up (matching RViz):
//!
//! ```text
//!     ROS x →  world x
//!     ROS y →  world -z   (so +Y is "up the screen" in top-down view)
//!     ROS z →  world  y   (ROS up = world up)
//! ```
//!
//! Subscribers that transform ROS points to world compose:
//!
//! ```text
//!     world_point = ROS_TO_WORLD * tf_in_ros * point_in_ros
//! ```
//!
//! The occupancy quad lives in the renderer's local XZ plane and needs an
//! extra `QUAD_SWAP` step to reinterpret those vertices as lying in the ROS
//! XY plane before the ROS-side transforms run.

use glam::{Mat4, Vec4};

/// Proper rotation. Applied as `M * v`, sends ROS basis vectors to
/// e_x → (1, 0, 0), e_y → (0, 0, -1), e_z → (0, 1, 0).
pub const ROS_TO_WORLD: Mat4 = Mat4::from_cols(
    Vec4::new(1.0, 0.0, 0.0, 0.0),
    Vec4::new(0.0, 0.0, -1.0, 0.0),
    Vec4::new(0.0, 1.0, 0.0, 0.0),
    Vec4::new(0.0, 0.0, 0.0, 1.0),
);

/// Reflection that takes a renderer-local point in the XZ ground plane
/// `(qx, 0, qz)` to the same point reinterpreted as lying in the ROS XY
/// ground plane `(qx, qz, 0)`. Used by the occupancy pass to bridge the
/// quad-builder convention into ROS space before `ROS_TO_WORLD` runs.
pub const QUAD_SWAP: Mat4 = Mat4::from_cols(
    Vec4::new(1.0, 0.0, 0.0, 0.0),
    Vec4::new(0.0, 0.0, 1.0, 0.0),
    Vec4::new(0.0, 1.0, 0.0, 0.0),
    Vec4::new(0.0, 0.0, 0.0, 1.0),
);
