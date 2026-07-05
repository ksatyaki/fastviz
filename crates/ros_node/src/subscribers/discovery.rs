//! Subscriber registry and polled topic discovery.
//!
//! Concrete topics from `RosConfig` are spawned at startup via `bootstrap`.
//! If any kind's topic list contains the bare wildcard `"*"`, [`tick`] runs
//! periodically against `node.get_topic_names_and_types()` and spawns a
//! subscriber for each newly-seen topic of the matching message type.
//!
//! Subscriber teardown on topic-disappear is **not** implemented in M0.5.

use std::collections::{HashMap, HashSet};
use std::sync::Arc;

use anyhow::Result;
use futures::executor::LocalSpawner;
use parking_lot::RwLock;
use scene::SceneHandle;

use crate::config::RosConfig;
use crate::stats::RosStats;
use crate::subscribers::{laserscan, marker, occupancy, path, pointcloud, pose};
use crate::tf::TfTree;
use crate::tf_refresh::TfRegistry;

const WILDCARD: &str = "*";

/// Shared set of topics the user has removed from the view. Subscriber tasks
/// can't be dropped (no join handle), so each cooperatively checks this set at
/// the top of its message loop and `break`s when its topic is present.
pub type CancelSet = Arc<RwLock<HashSet<String>>>;

/// Tracks already-spawned per-topic subscribers so we never double-subscribe,
/// and assigns stable indices for `EntityId` allocation.
#[derive(Default, Debug)]
pub struct Registry {
    map_spawned: bool,
    /// Topic name occupying the singleton map slot, so removal can free it.
    map_topic: Option<String>,
    costmaps: HashMap<String, usize>,
    poses: HashMap<String, usize>,
    pose_arrays: HashMap<String, usize>,
    paths: HashMap<String, usize>,
    scans: HashMap<String, usize>,
    points: HashMap<String, usize>,
    markers: HashMap<String, usize>,
    marker_arrays: HashMap<String, usize>,
    costmap_next: usize,
    pose_next: usize,
    pose_array_next: usize,
    path_next: usize,
    scan_next: usize,
    point_next: usize,
    marker_next: usize,
    /// Topics the user has removed; shared with every subscriber task.
    cancelled: CancelSet,
}

impl Registry {
    pub fn new() -> Self {
        Self::default()
    }

    /// Shared cancel set handed to subscriber tasks.
    pub fn cancel_set(&self) -> CancelSet {
        self.cancelled.clone()
    }

    /// Forget a removed topic so a later "Add" re-spawns it cleanly: drop its
    /// registry slot from whichever map owns it, and free the singleton map
    /// slot if this was the map topic.
    pub fn forget(&mut self, topic: &str) {
        self.costmaps.remove(topic);
        self.poses.remove(topic);
        self.pose_arrays.remove(topic);
        self.paths.remove(topic);
        self.scans.remove(topic);
        self.points.remove(topic);
        self.markers.remove(topic);
        self.marker_arrays.remove(topic);
        if self.map_topic.as_deref() == Some(topic) {
            self.map_spawned = false;
            self.map_topic = None;
        }
    }
}

/// Spawn subscribers for every concrete topic listed in `cfg`. Wildcard
/// entries (`"*"`) are deferred to [`tick`].
#[allow(clippy::too_many_arguments)]
pub fn bootstrap(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: SceneHandle,
    tf: Arc<TfTree>,
    tf_refresh: Arc<TfRegistry>,
    cfg: &RosConfig,
    stats: Arc<RosStats>,
) -> Result<Registry> {
    let mut reg = Registry::new();

    // Map: single topic, no wildcards in M0.5.
    if cfg.map_topic == WILDCARD {
        log::warn!("wildcard map topics aren't supported in M0.5; skipping");
    } else {
        occupancy::spawn(node, spawner, scene.clone(), tf.clone(), cfg, reg.cancel_set())?;
        reg.map_spawned = true;
        reg.map_topic = Some(cfg.map_topic.clone());
    }

    for topic in cfg.costmap_topics.iter().filter(|t| t.as_str() != WILDCARD) {
        spawn_costmap(node, spawner, &scene, &tf, cfg, &mut reg, topic)?;
    }

    for topic in cfg.pose_topics.iter().filter(|t| t.as_str() != WILDCARD) {
        spawn_pose(node, spawner, &scene, &tf, cfg, &mut reg, topic)?;
    }
    for topic in cfg
        .pose_array_topics
        .iter()
        .filter(|t| t.as_str() != WILDCARD)
    {
        spawn_pose_array(node, spawner, &scene, &tf, cfg, &mut reg, topic)?;
    }
    for topic in cfg.path_topics.iter().filter(|t| t.as_str() != WILDCARD) {
        spawn_path(node, spawner, &scene, &tf, cfg, &mut reg, topic)?;
    }
    for topic in cfg.scan_topics.iter().filter(|t| t.as_str() != WILDCARD) {
        spawn_scan(node, spawner, &scene, &tf, &tf_refresh, cfg, &mut reg, topic)?;
    }
    for topic in cfg.point_topics.iter().filter(|t| t.as_str() != WILDCARD) {
        spawn_pointcloud(
            node,
            spawner,
            &scene,
            &tf,
            &tf_refresh,
            cfg,
            &mut reg,
            &stats,
            topic,
        )?;
    }
    for topic in cfg.marker_topics.iter().filter(|t| t.as_str() != WILDCARD) {
        spawn_marker(node, spawner, &scene, &tf, &tf_refresh, cfg, &mut reg, topic)?;
    }
    for topic in cfg
        .marker_array_topics
        .iter()
        .filter(|t| t.as_str() != WILDCARD)
    {
        spawn_marker_array(node, spawner, &scene, &tf, &tf_refresh, cfg, &mut reg, topic)?;
    }

    if has_wildcard(cfg) {
        log::info!("topic discovery enabled (wildcard \"*\" present in config)");
    }
    Ok(reg)
}

/// Run one discovery cycle. Cheap when no wildcards are configured; otherwise
/// asks rcl for the topic snapshot and spawns subscribers for new matches.
#[allow(clippy::too_many_arguments)]
pub fn tick(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: &SceneHandle,
    tf: &Arc<TfTree>,
    tf_refresh: &Arc<TfRegistry>,
    cfg: &RosConfig,
    reg: &mut Registry,
    stats: &Arc<RosStats>,
) -> Result<()> {
    if !has_wildcard(cfg) {
        return Ok(());
    }
    let snapshot = match node.get_topic_names_and_types() {
        Ok(s) => s,
        Err(e) => {
            log::warn!("topic discovery snapshot failed: {e}");
            return Ok(());
        }
    };

    let costmap_wild = cfg.costmap_topics.iter().any(|t| t == WILDCARD);
    let pose_wild = cfg.pose_topics.iter().any(|t| t == WILDCARD);
    let pose_array_wild = cfg.pose_array_topics.iter().any(|t| t == WILDCARD);
    let path_wild = cfg.path_topics.iter().any(|t| t == WILDCARD);
    let scan_wild = cfg.scan_topics.iter().any(|t| t == WILDCARD);
    let point_wild = cfg.point_topics.iter().any(|t| t == WILDCARD);
    let marker_wild = cfg.marker_topics.iter().any(|t| t == WILDCARD);
    let marker_array_wild = cfg.marker_array_topics.iter().any(|t| t == WILDCARD);

    for (topic, types) in &snapshot {
        for ty in types {
            if costmap_wild
                && ty == occupancy::MSG_TYPE
                && topic != &cfg.map_topic
                && !reg.costmaps.contains_key(topic)
            {
                // A wildcard costmap catches every OccupancyGrid except the one
                // already shown as the static map.
                spawn_costmap(node, spawner, scene, tf, cfg, reg, topic)?;
            } else if pose_wild && ty == pose::STAMPED_TYPE && !reg.poses.contains_key(topic) {
                spawn_pose(node, spawner, scene, tf, cfg, reg, topic)?;
            } else if pose_array_wild
                && ty == pose::ARRAY_TYPE
                && !reg.pose_arrays.contains_key(topic)
            {
                spawn_pose_array(node, spawner, scene, tf, cfg, reg, topic)?;
            } else if path_wild && ty == path::MSG_TYPE && !reg.paths.contains_key(topic) {
                spawn_path(node, spawner, scene, tf, cfg, reg, topic)?;
            } else if scan_wild && ty == laserscan::MSG_TYPE && !reg.scans.contains_key(topic) {
                spawn_scan(node, spawner, scene, tf, tf_refresh, cfg, reg, topic)?;
            } else if point_wild
                && ty == pointcloud::MSG_TYPE
                && !reg.points.contains_key(topic)
            {
                spawn_pointcloud(node, spawner, scene, tf, tf_refresh, cfg, reg, stats, topic)?;
            } else if marker_wild
                && ty == marker::MARKER_TYPE
                && !reg.markers.contains_key(topic)
            {
                spawn_marker(node, spawner, scene, tf, tf_refresh, cfg, reg, topic)?;
            } else if marker_array_wild
                && ty == marker::ARRAY_TYPE
                && !reg.marker_arrays.contains_key(topic)
            {
                spawn_marker_array(node, spawner, scene, tf, tf_refresh, cfg, reg, topic)?;
            }
        }
    }
    Ok(())
}

/// Spawn a subscriber for a single `topic` of a known [`TopicKind`], on demand
/// (the "Add" dialog). Idempotent per topic: the registry guards against a
/// double-subscribe. An OccupancyGrid lands in the singleton map slot if it's
/// still free, otherwise as a costmap overlay so it doesn't clobber an existing
/// map.
#[allow(clippy::too_many_arguments)]
pub fn spawn_for_kind(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: &SceneHandle,
    tf: &Arc<TfTree>,
    tf_refresh: &Arc<TfRegistry>,
    cfg: &RosConfig,
    reg: &mut Registry,
    stats: &Arc<RosStats>,
    topic: &str,
    kind: crate::config_writer::TopicKind,
) -> Result<()> {
    use crate::config_writer::TopicKind;
    // A fresh add for a previously-removed topic must clear its cancel flag so
    // the new subscriber task isn't immediately torn down on its first message.
    reg.cancelled.write().remove(topic);
    match kind {
        TopicKind::Map => {
            if reg.map_spawned {
                spawn_costmap(node, spawner, scene, tf, cfg, reg, topic)?;
            } else {
                occupancy::spawn_map_topic(
                    node,
                    spawner,
                    scene.clone(),
                    tf.clone(),
                    cfg.reference_frame.clone(),
                    topic.to_string(),
                    cfg.map_qos.get(topic).cloned(),
                    reg.cancel_set(),
                )?;
                reg.map_spawned = true;
                reg.map_topic = Some(topic.to_string());
                log::info!("added map topic at runtime: {topic}");
            }
        }
        TopicKind::Pose => spawn_pose(node, spawner, scene, tf, cfg, reg, topic)?,
        TopicKind::PoseArray => spawn_pose_array(node, spawner, scene, tf, cfg, reg, topic)?,
        TopicKind::Path => spawn_path(node, spawner, scene, tf, cfg, reg, topic)?,
        TopicKind::Scan => spawn_scan(node, spawner, scene, tf, tf_refresh, cfg, reg, topic)?,
        TopicKind::PointCloud => {
            spawn_pointcloud(node, spawner, scene, tf, tf_refresh, cfg, reg, stats, topic)?
        }
        TopicKind::Marker => spawn_marker(node, spawner, scene, tf, tf_refresh, cfg, reg, topic)?,
        TopicKind::MarkerArray => {
            spawn_marker_array(node, spawner, scene, tf, tf_refresh, cfg, reg, topic)?
        }
    }
    Ok(())
}

fn has_wildcard(cfg: &RosConfig) -> bool {
    [
        &cfg.costmap_topics,
        &cfg.pose_topics,
        &cfg.pose_array_topics,
        &cfg.path_topics,
        &cfg.scan_topics,
        &cfg.point_topics,
        &cfg.marker_topics,
        &cfg.marker_array_topics,
    ]
    .into_iter()
    .any(|v| v.iter().any(|t| t == WILDCARD))
}

fn spawn_costmap(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: &SceneHandle,
    tf: &Arc<TfTree>,
    cfg: &RosConfig,
    reg: &mut Registry,
    topic: &str,
) -> Result<()> {
    if reg.costmaps.contains_key(topic) {
        return Ok(());
    }
    let idx = reg.costmap_next;
    reg.costmap_next += 1;
    occupancy::spawn_costmap_topic(
        node,
        spawner,
        scene.clone(),
        tf.clone(),
        cfg.reference_frame.clone(),
        topic.to_string(),
        idx,
        cfg.costmap_qos.get(topic).cloned(),
        reg.cancel_set(),
    )?;
    reg.costmaps.insert(topic.to_string(), idx);
    Ok(())
}

fn spawn_pose(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: &SceneHandle,
    tf: &Arc<TfTree>,
    cfg: &RosConfig,
    reg: &mut Registry,
    topic: &str,
) -> Result<()> {
    if reg.poses.contains_key(topic) {
        return Ok(());
    }
    let idx = reg.pose_next;
    reg.pose_next += 1;
    pose::spawn_pose_stamped_topic(
        node,
        spawner,
        scene.clone(),
        tf.clone(),
        cfg.reference_frame.clone(),
        cfg.arrow_style_for(topic, false),
        topic.to_string(),
        idx,
        cfg.pose_qos.get(topic).cloned(),
        reg.cancel_set(),
    )?;
    reg.poses.insert(topic.to_string(), idx);
    Ok(())
}

fn spawn_pose_array(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: &SceneHandle,
    tf: &Arc<TfTree>,
    cfg: &RosConfig,
    reg: &mut Registry,
    topic: &str,
) -> Result<()> {
    if reg.pose_arrays.contains_key(topic) {
        return Ok(());
    }
    let idx = reg.pose_array_next;
    reg.pose_array_next += 1;
    pose::spawn_pose_array_topic(
        node,
        spawner,
        scene.clone(),
        tf.clone(),
        cfg.reference_frame.clone(),
        cfg.arrow_style_for(topic, true),
        topic.to_string(),
        idx,
        cfg.pose_array_qos.get(topic).cloned(),
        reg.cancel_set(),
    )?;
    reg.pose_arrays.insert(topic.to_string(), idx);
    Ok(())
}

fn spawn_path(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: &SceneHandle,
    tf: &Arc<TfTree>,
    cfg: &RosConfig,
    reg: &mut Registry,
    topic: &str,
) -> Result<()> {
    if reg.paths.contains_key(topic) {
        return Ok(());
    }
    let idx = reg.path_next;
    reg.path_next += 1;
    path::spawn_topic(
        node,
        spawner,
        scene.clone(),
        tf.clone(),
        cfg.reference_frame.clone(),
        cfg.path_style_for(topic),
        topic.to_string(),
        idx,
        cfg.path_qos.get(topic).cloned(),
        reg.cancel_set(),
    )?;
    reg.paths.insert(topic.to_string(), idx);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn spawn_scan(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: &SceneHandle,
    tf: &Arc<TfTree>,
    tf_refresh: &Arc<TfRegistry>,
    cfg: &RosConfig,
    reg: &mut Registry,
    topic: &str,
) -> Result<()> {
    if reg.scans.contains_key(topic) {
        return Ok(());
    }
    let idx = reg.scan_next;
    reg.scan_next += 1;
    laserscan::spawn_topic(
        node,
        spawner,
        scene.clone(),
        tf.clone(),
        tf_refresh.clone(),
        cfg.reference_frame.clone(),
        cfg.scan_style_for(topic),
        topic.to_string(),
        idx,
        cfg.scan_qos.get(topic).cloned(),
        reg.cancel_set(),
    )?;
    reg.scans.insert(topic.to_string(), idx);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn spawn_pointcloud(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: &SceneHandle,
    tf: &Arc<TfTree>,
    tf_refresh: &Arc<TfRegistry>,
    cfg: &RosConfig,
    reg: &mut Registry,
    stats: &Arc<RosStats>,
    topic: &str,
) -> Result<()> {
    if reg.points.contains_key(topic) {
        return Ok(());
    }
    let idx = reg.point_next;
    reg.point_next += 1;
    pointcloud::spawn_topic(
        node,
        spawner,
        scene.clone(),
        tf.clone(),
        tf_refresh.clone(),
        cfg.reference_frame.clone(),
        cfg.point_style_for(topic),
        topic.to_string(),
        idx,
        cfg.point_qos.get(topic).cloned(),
        stats.clone(),
        reg.cancel_set(),
    )?;
    reg.points.insert(topic.to_string(), idx);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn spawn_marker(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: &SceneHandle,
    tf: &Arc<TfTree>,
    tf_refresh: &Arc<TfRegistry>,
    cfg: &RosConfig,
    reg: &mut Registry,
    topic: &str,
) -> Result<()> {
    if reg.markers.contains_key(topic) {
        return Ok(());
    }
    let idx = reg.marker_next;
    reg.marker_next += 1;
    marker::spawn_marker_topic(
        node,
        spawner,
        scene.clone(),
        tf.clone(),
        tf_refresh.clone(),
        cfg.reference_frame.clone(),
        topic.to_string(),
        idx,
        cfg.marker_qos.get(topic).cloned(),
        reg.cancel_set(),
    )?;
    reg.markers.insert(topic.to_string(), idx);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn spawn_marker_array(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: &SceneHandle,
    tf: &Arc<TfTree>,
    tf_refresh: &Arc<TfRegistry>,
    cfg: &RosConfig,
    reg: &mut Registry,
    topic: &str,
) -> Result<()> {
    if reg.marker_arrays.contains_key(topic) {
        return Ok(());
    }
    let idx = reg.marker_next;
    reg.marker_next += 1;
    marker::spawn_marker_array_topic(
        node,
        spawner,
        scene.clone(),
        tf.clone(),
        tf_refresh.clone(),
        cfg.reference_frame.clone(),
        topic.to_string(),
        idx,
        cfg.marker_array_qos.get(topic).cloned(),
        reg.cancel_set(),
    )?;
    reg.marker_arrays.insert(topic.to_string(), idx);
    Ok(())
}

