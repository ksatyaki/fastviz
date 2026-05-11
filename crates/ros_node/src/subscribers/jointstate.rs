//! `sensor_msgs/JointState` → per-link `entity.transform` updates.
//!
//! The URDF is loaded once at startup; this subscriber owns the resulting
//! [`UrdfModel`] (wrapped in a `parking_lot::Mutex` so we can share it with
//! future subscribers). Each incoming JointState message overwrites the named
//! joint positions, then we recompute `entity.transform` for every link with a
//! visual.
//!
//! Robot placement in the world: the URDF root frame is looked up in the TF
//! tree against the scene's reference frame. If it's not there yet (no
//! `robot_state_publisher` or `static_transform_publisher` providing
//! `reference_frame ← root_link`), we render the robot in its own root frame.

use std::sync::Arc;

use anyhow::{Context, Result};
use futures::executor::LocalSpawner;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use glam::Mat4;
use parking_lot::Mutex;
use r2r::sensor_msgs::msg::JointState;
use r2r::QosProfile;
use scene::SceneHandle;

use crate::config::QosOverride;
use crate::coords::ROS_TO_WORLD;
use crate::tf::TfTree;
use crate::tf_refresh::TfRegistry;
use crate::urdf::UrdfModel;

pub const MSG_TYPE: &str = "sensor_msgs/msg/JointState";

#[allow(clippy::too_many_arguments)]
pub fn spawn_topic(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: SceneHandle,
    tf: Arc<TfTree>,
    tf_refresh: Arc<TfRegistry>,
    reference_frame: String,
    model: Arc<Mutex<UrdfModel>>,
    topic: String,
    qos_override: Option<QosOverride>,
) -> Result<()> {
    let mut qos = QosProfile::default().keep_last(10);
    if let Some(o) = &qos_override {
        qos = o.apply(qos);
        log::debug!("qos override applied to {topic}: {o:?}");
    }
    let mut sub = node
        .subscribe::<JointState>(&topic, qos)
        .with_context(|| format!("subscribing to {topic}"))?;
    log::info!("subscribed: {topic} (sensor_msgs/JointState)");

    spawner
        .spawn_local(async move {
            let mut first = true;
            while let Some(msg) = sub.next().await {
                if msg.name.len() != msg.position.len() {
                    log::warn!(
                        "{topic}: name.len({}) != position.len({}); dropping",
                        msg.name.len(),
                        msg.position.len()
                    );
                    continue;
                }
                {
                    let mut m = model.lock();
                    m.apply_joint_positions(msg.name.iter().map(String::as_str), &msg.position);
                    push_link_transforms(&scene, &tf, &reference_frame, &m);
                    // Re-bind links so the registry can refresh world
                    // transforms when /tf changes between joint-state messages
                    // (or never publishes a new one).
                    register_link_transforms(&tf_refresh, &m);
                }
                if first {
                    log::info!("{topic}: first message ({} joints)", msg.name.len());
                    first = false;
                }
            }
        })
        .context("spawning jointstate subscriber task")?;
    Ok(())
}

/// Register every URDF link with the TF registry so the main loop's refresh
/// can recompute world transforms whenever the TF tree changes — even between
/// JointState messages, or with no JointState messages at all (URDFs whose
/// joints never move).
pub fn register_link_transforms(tf_refresh: &TfRegistry, model: &UrdfModel) {
    for link in &model.links {
        // `local` is the visual pose expressed in the URDF root link's ROS
        // frame; the registry composes it with `ROS_TO_WORLD * ref_T_root`.
        tf_refresh.register(link.entity_id, model.root_link.clone(), model.visual_in_root(link));
    }
}

/// Compute `entity.transform` for every link in the URDF and push it into the
/// scene graph. Called once at startup (after parsing the URDF, with all joint
/// positions at zero) and again on every JointState message.
pub fn push_link_transforms(
    scene: &SceneHandle,
    tf: &TfTree,
    reference_frame: &str,
    model: &UrdfModel,
) {
    let tf_ref_from_root = if reference_frame == model.root_link {
        Mat4::IDENTITY
    } else {
        match tf.lookup(reference_frame, &model.root_link) {
            Some(m) => m,
            None => {
                log::trace!(
                    "tf lookup {} -> {reference_frame} not yet available; rendering urdf in its own root frame",
                    model.root_link
                );
                Mat4::IDENTITY
            }
        }
    };
    let to_world = ROS_TO_WORLD * tf_ref_from_root;
    let mut scene = scene.write();
    for link in &model.links {
        let transform = to_world * model.visual_in_root(link);
        scene.update_transform(link.entity_id, transform);
    }
}
