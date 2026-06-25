//! `geometry_msgs/PoseStamped` and `geometry_msgs/PoseArray` → `Arrows`.
//!
//! Each pose's frame is resolved through `TfTree::lookup(reference_frame, ...)`.
//! Translation and orientation are applied in ROS space, then mapped through
//! `ROS_TO_WORLD`. The resulting world translation is the arrow origin and the
//! world-rotated ROS +X axis is its direction.

use std::sync::Arc;

use anyhow::{Context, Result};
use futures::executor::LocalSpawner;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use glam::{Mat4, Quat, Vec3};
use r2r::geometry_msgs::msg::{Pose, PoseArray, PoseStamped};
use r2r::QosProfile;
use scene::{Arrow, SceneEntity, SceneHandle, ScenePrimitive};

use crate::config::{ArrowStyle, QosOverride};
use crate::coords::ROS_TO_WORLD;
use crate::ids::{pose_array_id, pose_id};
use crate::subscribers::discovery::CancelSet;
use crate::tf::TfTree;

pub const STAMPED_TYPE: &str = "geometry_msgs/msg/PoseStamped";
pub const ARRAY_TYPE: &str = "geometry_msgs/msg/PoseArray";

#[allow(clippy::too_many_arguments)]
pub fn spawn_pose_stamped_topic(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: SceneHandle,
    tf: Arc<TfTree>,
    reference_frame: String,
    style: ArrowStyle,
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
        .subscribe::<PoseStamped>(&topic, qos)
        .with_context(|| format!("subscribing to {topic}"))?;
    log::info!("subscribed: {topic} (geometry_msgs/PoseStamped)");
    let id = pose_id(topic_idx);

    spawner
        .spawn_local(async move {
            let mut first = true;
            while let Some(msg) = sub.next().await {
                if cancelled.read().contains(&topic) {
                    break;
                }
                let arrow = pose_to_arrow(
                    &msg.pose,
                    &msg.header.frame_id,
                    &reference_frame,
                    &tf,
                    &style,
                );
                if first {
                    log::info!("{topic}: first message (frame={})", msg.header.frame_id);
                    first = false;
                }
                let entity = SceneEntity::new(id, ScenePrimitive::Arrows(vec![arrow]))
                    .with_label(topic.clone());
                scene.write().upsert(entity);
            }
        })
        .context("spawning pose subscriber task")?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub fn spawn_pose_array_topic(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: SceneHandle,
    tf: Arc<TfTree>,
    reference_frame: String,
    style: ArrowStyle,
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
        .subscribe::<PoseArray>(&topic, qos)
        .with_context(|| format!("subscribing to {topic}"))?;
    log::info!("subscribed: {topic} (geometry_msgs/PoseArray)");
    let id = pose_array_id(topic_idx);

    spawner
        .spawn_local(async move {
            let mut first = true;
            while let Some(msg) = sub.next().await {
                if cancelled.read().contains(&topic) {
                    break;
                }
                let arrows: Vec<Arrow> = msg
                    .poses
                    .iter()
                    .map(|p| pose_to_arrow(p, &msg.header.frame_id, &reference_frame, &tf, &style))
                    .collect();
                if first {
                    log::info!(
                        "{topic}: first message ({} poses, frame={})",
                        arrows.len(),
                        msg.header.frame_id
                    );
                    first = false;
                }
                let entity = SceneEntity::new(id, ScenePrimitive::Arrows(arrows))
                    .with_label(topic.clone());
                scene.write().upsert(entity);
            }
        })
        .context("spawning pose subscriber task")?;
    Ok(())
}

fn pose_to_arrow(
    pose: &Pose,
    pose_frame: &str,
    reference_frame: &str,
    tf: &TfTree,
    style: &ArrowStyle,
) -> Arrow {
    let p = &pose.position;
    let q = &pose.orientation;
    let pose_in_frame = Mat4::from_rotation_translation(
        Quat::from_xyzw(q.x as f32, q.y as f32, q.z as f32, q.w as f32),
        Vec3::new(p.x as f32, p.y as f32, p.z as f32),
    );

    let tf_ref_from_frame = if pose_frame == reference_frame {
        Mat4::IDENTITY
    } else {
        match tf.lookup(reference_frame, pose_frame) {
            Some(m) => m,
            None => {
                log::warn!(
                    "tf lookup {pose_frame} -> {reference_frame} not yet available; rendering pose in its own frame"
                );
                Mat4::IDENTITY
            }
        }
    };

    // Pose-in-world (Mat4): the arrow's origin is the world-translation,
    // and direction is the world-rotated ROS +X axis (REP-103: arrow points forward).
    let pose_in_world = ROS_TO_WORLD * tf_ref_from_frame * pose_in_frame;
    let origin = pose_in_world.transform_point3(Vec3::ZERO);
    let direction = pose_in_world
        .transform_vector3(Vec3::X)
        .try_normalize()
        .unwrap_or(Vec3::X);

    Arrow {
        origin,
        direction,
        length: style.length,
        shaft_radius: style.shaft_radius,
        head_radius: style.head_radius,
        color: style.color,
    }
}
