//! Centralised EntityId allocation for the ROS layer (see plan §6.1).

use scene::EntityId;

/// Mock injector reserves 1..=999, so ROS starts at 1000.
pub const ROS_ID_BASE: u64 = 1000;

/// OccupancyGrid (`/map`) singleton.
pub const ROS_ID_MAP: EntityId = EntityId(1000);

// Per-topic ranges within 2000..=2999 (plan §6.1).
pub const ROS_ID_POSE_BASE: u64 = 2000; // PoseStamped topics
pub const ROS_ID_POSE_ARRAY_BASE: u64 = 2100; // PoseArray topics
pub const ROS_ID_PATH_BASE: u64 = 2200; // Path topics
pub const ROS_ID_SCAN_BASE: u64 = 2300; // LaserScan topics
pub const ROS_ID_POINTCLOUD_BASE: u64 = 2400; // PointCloud2 topics

/// URDF link entities live at 3000.. (plan §3.1).
pub const URDF_LINK_BASE: u64 = 3000;

/// `visualization_msgs/Marker` entities. Each topic gets a 100_000-wide slab;
/// within a slab, every (ns, id) pair from a publisher is mapped to a unique
/// EntityId allocated sequentially as it first appears.
pub const ROS_ID_MARKER_BASE: u64 = 4_000_000;
pub const ROS_ID_MARKER_PER_TOPIC: u64 = 100_000;

/// Compute the base EntityId for a marker topic (the first slot in its slab).
pub fn marker_topic_base(topic_index: usize) -> u64 {
    ROS_ID_MARKER_BASE + (topic_index as u64) * ROS_ID_MARKER_PER_TOPIC
}

/// TF-frame axis indicators live above the URDF range. Each frame in the TF
/// tree is materialised as a `ScenePrimitive::Frame` entity with a stable id
/// allocated in encounter order from this base.
pub const TF_FRAME_BASE: u64 = 2_000_000;
/// Upper bound on simultaneously tracked TF frames. Used by the UI to identify
/// TF entities by id range (mirrors how `URDF_LINK_BASE` works in ui.rs).
pub const TF_FRAME_CAPACITY: u64 = 1_000_000;

pub fn pose_id(topic_index: usize) -> EntityId {
    EntityId(ROS_ID_POSE_BASE + topic_index as u64)
}

pub fn pose_array_id(topic_index: usize) -> EntityId {
    EntityId(ROS_ID_POSE_ARRAY_BASE + topic_index as u64)
}

pub fn path_id(topic_index: usize) -> EntityId {
    EntityId(ROS_ID_PATH_BASE + topic_index as u64)
}

pub fn scan_id(topic_index: usize) -> EntityId {
    EntityId(ROS_ID_SCAN_BASE + topic_index as u64)
}

pub fn pointcloud_id(topic_index: usize) -> EntityId {
    EntityId(ROS_ID_POINTCLOUD_BASE + topic_index as u64)
}
