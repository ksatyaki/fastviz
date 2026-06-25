//! `nav_msgs/OccupancyGrid` → `scene::ScenePrimitive::Grid`.
//!
//! The grid is encoded in its own local frame (lower-left at origin, cells
//! extending +X/+Y). All placement — `info.origin` pose, TF lookup into the
//! scene's reference frame, and the ROS↔fastviz coord swap — is composed into
//! `entity.transform`.

use std::sync::Arc;

use anyhow::{Context, Result};
use futures::executor::LocalSpawner;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use glam::{Mat4, Quat, Vec2, Vec3};
use r2r::qos::DurabilityPolicy;
use r2r::{nav_msgs, QosProfile};
use scene::{Colormap, Grid, GridData, SceneEntity, SceneHandle, ScenePrimitive};

use crate::config::{QosOverride, RosConfig};
use crate::coords::{QUAD_SWAP, ROS_TO_WORLD};
use crate::ids::{costmap_id, ROS_ID_MAP};
use crate::subscribers::discovery::CancelSet;
use crate::tf::TfTree;

pub const MSG_TYPE: &str = "nav_msgs/msg/OccupancyGrid";

/// `/map` subscriber: single topic, rendered opaque with the occupancy colormap.
pub fn spawn(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: SceneHandle,
    tf: Arc<TfTree>,
    cfg: &RosConfig,
    cancelled: CancelSet,
) -> Result<()> {
    spawn_grid(
        node,
        spawner,
        scene,
        tf,
        cfg.reference_frame.clone(),
        cfg.map_topic.clone(),
        ROS_ID_MAP,
        Colormap::OccupancyDefault,
        "map",
        cfg.map_qos.get(&cfg.map_topic).cloned(),
        cancelled,
    )
}

/// Map subscriber for a runtime-chosen topic (the "Add" dialog). Uses the
/// opaque occupancy colormap and the singleton `ROS_ID_MAP` entity, so it
/// replaces whatever map slot was previously empty.
#[allow(clippy::too_many_arguments)]
pub fn spawn_map_topic(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: SceneHandle,
    tf: Arc<TfTree>,
    reference_frame: String,
    topic: String,
    qos_override: Option<QosOverride>,
    cancelled: CancelSet,
) -> Result<()> {
    spawn_grid(
        node,
        spawner,
        scene,
        tf,
        reference_frame,
        topic,
        ROS_ID_MAP,
        Colormap::OccupancyDefault,
        "map",
        qos_override,
        cancelled,
    )
}

/// Costmap-overlay subscriber: per-topic entity, rendered with the cost
/// colormap (free/unknown transparent) so it layers on top of `/map`.
#[allow(clippy::too_many_arguments)]
pub fn spawn_costmap_topic(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: SceneHandle,
    tf: Arc<TfTree>,
    reference_frame: String,
    topic: String,
    topic_index: usize,
    qos_override: Option<QosOverride>,
    cancelled: CancelSet,
) -> Result<()> {
    spawn_grid(
        node,
        spawner,
        scene,
        tf,
        reference_frame,
        topic,
        costmap_id(topic_index),
        Colormap::Costmap,
        "costmap",
        qos_override,
        cancelled,
    )
}

#[allow(clippy::too_many_arguments)]
fn spawn_grid(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: SceneHandle,
    tf: Arc<TfTree>,
    reference_frame: String,
    topic: String,
    entity_id: scene::EntityId,
    colormap: Colormap,
    label_kind: &'static str,
    qos_override: Option<QosOverride>,
    cancelled: CancelSet,
) -> Result<()> {
    // Grid publishers commonly use TRANSIENT_LOCAL (latched) durability, but ad-hoc
    // pubs (and most costmaps) use VOLATILE. BestAvailable matches either.
    #[cfg(not(ros_humble))]
    let durability = DurabilityPolicy::BestAvailable;
    #[cfg(ros_humble)]
    let durability = DurabilityPolicy::TransientLocal;

    let mut qos = QosProfile::default().keep_last(1).durability(durability);
    if let Some(o) = &qos_override {
        qos = o.apply(qos);
        log::debug!("qos override applied to {topic}: {o:?}");
    }
    let mut sub = node
        .subscribe::<nav_msgs::msg::OccupancyGrid>(&topic, qos)
        .with_context(|| format!("subscribing to {topic}"))?;
    log::info!("subscribed: {topic} (nav_msgs/OccupancyGrid, {label_kind})");

    let label = format!("{topic} [{label_kind}]");
    spawner
        .spawn_local(async move {
            let mut last_stamp_ns: Option<i64> = None;
            let mut first = true;
            while let Some(msg) = sub.next().await {
                if cancelled.read().contains(&topic) {
                    break;
                }
                let stamp_ns = stamp_ns(&msg.header.stamp);
                if last_stamp_ns == Some(stamp_ns) {
                    continue;
                }
                last_stamp_ns = Some(stamp_ns);

                if first {
                    log::info!(
                        "{topic}: first message ({}x{} cells, {:.3} m/cell, frame={})",
                        msg.info.width,
                        msg.info.height,
                        msg.info.resolution,
                        msg.header.frame_id
                    );
                    first = false;
                }

                if let Some(entity) =
                    build_entity(&msg, &tf, &reference_frame, entity_id, colormap.clone(), &label)
                {
                    scene.write().upsert(entity);
                }
            }
        })
        .context("spawning occupancy subscriber task")?;

    Ok(())
}

fn stamp_ns(t: &r2r::builtin_interfaces::msg::Time) -> i64 {
    (t.sec as i64) * 1_000_000_000 + (t.nanosec as i64)
}

fn build_entity(
    msg: &nav_msgs::msg::OccupancyGrid,
    tf: &TfTree,
    reference_frame: &str,
    entity_id: scene::EntityId,
    colormap: Colormap,
    label: &str,
) -> Option<SceneEntity> {
    let cols = msg.info.width;
    let rows = msg.info.height;
    let cell_size = msg.info.resolution;

    // Convert i8 → u8 with -1 → 255 so it samples through Colormap::OccupancyDefault.
    let cells: Vec<u8> = msg
        .data
        .iter()
        .map(|&v| if v < 0 { 255u8 } else { v as u8 })
        .collect();

    // Grid is encoded in its own local frame: lower-left at origin, extending +X/+Y.
    // Renderer wants the *center* of the grid; in this frame the center is (W/2, H/2).
    let half_w = (cols as f32 * cell_size) * 0.5;
    let half_h = (rows as f32 * cell_size) * 0.5;
    let grid = Grid {
        origin: Vec2::new(half_w, half_h),
        cell_size,
        cols,
        rows,
        data: GridData::Cells(cells, colormap),
    };

    // Pose of the grid's local frame inside header.frame_id (info.origin).
    let p = &msg.info.origin.position;
    let q = &msg.info.origin.orientation;
    let pose_in_frame = Mat4::from_rotation_translation(
        Quat::from_xyzw(q.x as f32, q.y as f32, q.z as f32, q.w as f32),
        Vec3::new(p.x as f32, p.y as f32, p.z as f32),
    );

    // header.frame_id → reference_frame, in ROS space.
    let tf_ref_from_frame = if msg.header.frame_id == reference_frame {
        Mat4::IDENTITY
    } else {
        match tf.lookup(reference_frame, &msg.header.frame_id) {
            Some(m) => m,
            None => {
                log::warn!(
                    "tf lookup {} -> {} not yet available; rendering map in its own frame",
                    msg.header.frame_id,
                    reference_frame
                );
                Mat4::IDENTITY
            }
        }
    };

    // entity.transform takes the renderer's local XZ-plane quad → world.
    // QUAD_SWAP reinterprets the quad's (qx, 0, qz) as a ROS XY-plane point
    // before the ROS-side transforms run, and ROS_TO_WORLD finishes the trip
    // into renderer world space.
    let transform = ROS_TO_WORLD * tf_ref_from_frame * pose_in_frame * QUAD_SWAP;

    Some(
        SceneEntity::new(entity_id, ScenePrimitive::Grid(grid))
            .with_transform(transform)
            .with_label(label.to_string()),
    )
}
