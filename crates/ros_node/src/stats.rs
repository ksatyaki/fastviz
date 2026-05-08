//! Lightweight cross-thread counters for diagnosing producer/consumer drop
//! between subscribers and the render loop.
//!
//! Subscribers bump the relevant `*_received` counter after each successful
//! `SceneGraph::upsert`. The app reads the counter on each draw and compares
//! it to the previous read; any positive delta means "at least one frame was
//! overwritten since the last display."

use std::sync::atomic::AtomicU64;

#[derive(Default, Debug)]
pub struct RosStats {
    /// Total `sensor_msgs/PointCloud2` messages successfully decoded and
    /// written to the SceneGraph (summed across all PC2 topics).
    pub pc2_received: AtomicU64,
}
