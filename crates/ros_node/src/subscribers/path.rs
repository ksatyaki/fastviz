//! `nav_msgs/Path` → `scene::ScenePrimitive::Polyline`.
//!
//! One TF lookup per message — uses the parent `header.frame_id`, ignoring the
//! per-pose `header.frame_id` (typical case is identical anyway). Each pose's
//! position is mapped through `ROS_TO_WORLD * tf * t_pose`.

use std::sync::Arc;

use anyhow::{Context, Result};
use futures::executor::LocalSpawner;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use glam::{Mat4, Vec3};
use r2r::nav_msgs::msg::Path;
use r2r::QosProfile;
use scene::{Polyline, SceneEntity, SceneHandle, ScenePrimitive};

use crate::config::{PathStyle, QosOverride};
use crate::coords::ROS_TO_WORLD;
use crate::ids::path_id;
use crate::subscribers::discovery::CancelSet;
use crate::tf::TfTree;

pub const MSG_TYPE: &str = "nav_msgs/msg/Path";

#[allow(clippy::too_many_arguments)]
pub fn spawn_topic(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: SceneHandle,
    tf: Arc<TfTree>,
    reference_frame: String,
    style: PathStyle,
    topic: String,
    topic_idx: usize,
    qos_override: Option<QosOverride>,
    cancelled: CancelSet,
) -> Result<()> {
    let mut qos = QosProfile::default().keep_last(10);
    if let Some(o) = &qos_override {
        qos = o.apply(qos);
        log::debug!("qos override applied to {topic}: {o:?}");
    }
    let mut sub = node
        .subscribe::<Path>(&topic, qos)
        .with_context(|| format!("subscribing to {topic}"))?;
    log::info!("subscribed: {topic} (nav_msgs/Path)");
    let id = path_id(topic_idx);

    spawner
        .spawn_local(async move {
            let mut first = true;
            while let Some(msg) = sub.next().await {
                if cancelled.read().contains(&topic) {
                    break;
                }
                let frame = &msg.header.frame_id;
                let tf_ref_from_frame = if frame == &reference_frame {
                    Mat4::IDENTITY
                } else {
                    match tf.lookup(&reference_frame, frame) {
                        Some(m) => m,
                        None => {
                            log::warn!(
                                "tf lookup {frame} -> {reference_frame} not yet available; rendering path in its own frame"
                            );
                            Mat4::IDENTITY
                        }
                    }
                };
                let to_world = ROS_TO_WORLD * tf_ref_from_frame;

                let points: Vec<Vec3> = msg
                    .poses
                    .iter()
                    .map(|ps| {
                        let p = &ps.pose.position;
                        to_world.transform_point3(Vec3::new(p.x as f32, p.y as f32, p.z as f32))
                    })
                    .collect();

                if first {
                    log::info!(
                        "{topic}: first message ({} poses, frame={frame})",
                        points.len()
                    );
                    first = false;
                }

                let polyline = Polyline {
                    points,
                    color: style.color,
                    width: style.width,
                    strip: true,
                };
                let entity = SceneEntity::new(id, ScenePrimitive::Polyline(polyline))
                    .with_label(topic.clone());
                scene.write().upsert(entity);
            }
        })
        .context("spawning path subscriber task")?;
    Ok(())
}
