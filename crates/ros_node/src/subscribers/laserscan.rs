//! `sensor_msgs/LaserScan` → `scene::ScenePrimitive::Points`.
//!
//! Polar→Cartesian on CPU into the laser's own frame (Z=0). The TF lookup
//! `reference_frame ← header.frame_id` is folded into `entity.transform`
//! together with `ROS_TO_WORLD`, so the per-point math stays cheap (one
//! `cos`/`sin` per range, no per-point matrix multiply). The Vec<Point>
//! buffer is reserved with capacity = ranges.len() so we don't grow.

use std::sync::Arc;

use anyhow::{Context, Result};
use futures::executor::LocalSpawner;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use glam::{Mat4, Vec3};
use r2r::sensor_msgs::msg::LaserScan;
use r2r::QosProfile;
use scene::{Point, SceneEntity, SceneHandle, ScenePrimitive};

use crate::config::{QosOverride, ScanStyle};
use crate::coords::ROS_TO_WORLD;
use crate::ids::scan_id;
use crate::subscribers::discovery::CancelSet;
use crate::tf::TfTree;
use crate::tf_refresh::TfRegistry;

pub const MSG_TYPE: &str = "sensor_msgs/msg/LaserScan";

#[allow(clippy::too_many_arguments)]
pub fn spawn_topic(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: SceneHandle,
    tf: Arc<TfTree>,
    tf_refresh: Arc<TfRegistry>,
    reference_frame: String,
    style: ScanStyle,
    topic: String,
    topic_idx: usize,
    qos_override: Option<QosOverride>,
    cancelled: CancelSet,
) -> Result<()> {
    // sensor_data QoS: best_effort, depth 5 — matches typical laser drivers.
    let mut qos = QosProfile::sensor_data();
    if let Some(o) = &qos_override {
        qos = o.apply(qos);
        log::debug!("qos override applied to {topic}: {o:?}");
    }
    let mut sub = node
        .subscribe::<LaserScan>(&topic, qos)
        .with_context(|| format!("subscribing to {topic}"))?;
    log::info!("subscribed: {topic} (sensor_msgs/LaserScan)");
    let id = scan_id(topic_idx);

    spawner
        .spawn_local(async move {
            let mut first = true;
            // Re-warn only on success→failure transitions so a continuously
            // broken TF chain doesn't spam at the scan publish rate.
            let mut warned_missing_tf = false;
            let mut points: Vec<Point> = Vec::new();
            while let Some(msg) = sub.next().await {
                if cancelled.read().contains(&topic) {
                    break;
                }
                points.clear();
                if points.capacity() < msg.ranges.len() {
                    points.reserve(msg.ranges.len() - points.capacity());
                }

                for (i, &r) in msg.ranges.iter().enumerate() {
                    if !r.is_finite() || r < msg.range_min || r > msg.range_max {
                        continue;
                    }
                    let theta = msg.angle_min + (i as f32) * msg.angle_increment;
                    points.push(Point {
                        position: Vec3::new(r * theta.cos(), r * theta.sin(), 0.0),
                        color: style.color,
                        size: style.size,
                    });
                }

                let frame = &msg.header.frame_id;
                let stamp_ns = (msg.header.stamp.sec as i64) * 1_000_000_000
                    + (msg.header.stamp.nanosec as i64);
                let tf_ref_from_frame = if frame == &reference_frame {
                    warned_missing_tf = false;
                    Mat4::IDENTITY
                } else {
                    match tf.lookup_at(&reference_frame, frame, stamp_ns) {
                        Some(m) => {
                            warned_missing_tf = false;
                            m
                        }
                        None => {
                            if !warned_missing_tf {
                                log::warn!(
                                    "{topic}: tf lookup {frame} -> {reference_frame} unavailable — {}. \
                                     Rendering scan at world origin; it will be hidden inside any URDF mesh.",
                                    tf.diagnose_lookup(&reference_frame, frame)
                                );
                                warned_missing_tf = true;
                            }
                            Mat4::IDENTITY
                        }
                    }
                };
                // Points are in ROS coords (laser frame); take them to world.
                let transform = ROS_TO_WORLD * tf_ref_from_frame;

                if first {
                    log::info!(
                        "{topic}: first message ({}/{} ranges valid, frame={frame})",
                        points.len(),
                        msg.ranges.len()
                    );
                    first = false;
                }

                let entity = SceneEntity::new(id, ScenePrimitive::Points(std::mem::take(&mut points)))
                    .with_transform(transform)
                    .with_label(topic.clone());
                scene.write().upsert(entity);
                // (Re-)bind to the registry so late /tf arrivals get applied
                // by the main loop's refresh pass even if no fresh scan
                // message comes in. Points are already in `frame` coords, so
                // the per-frame local transform is just IDENTITY.
                tf_refresh.register_at(id, frame.as_str(), Mat4::IDENTITY, Some(stamp_ns));
            }
        })
        .context("spawning laserscan subscriber task")?;
    Ok(())
}
