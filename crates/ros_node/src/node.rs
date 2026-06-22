//! Node lifecycle: spawn a dedicated thread for the r2r executor and shut it
//! down cleanly when the app exits.

use std::sync::atomic::AtomicU32;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result};
use crossbeam_channel::{bounded, unbounded, Receiver, Sender};
use futures::executor::{LocalPool, LocalSpawner};
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use parking_lot::{Mutex, RwLock};
use r2r::qos::DurabilityPolicy;
use r2r::QosProfile;
use scene::SceneHandle;

use crate::config::RosConfig;
use crate::config_writer::TopicKind;
use crate::stats::RosStats;
use crate::subscribers;
use crate::tf::TfTree;
use crate::tf_axes::{TfAxesRegistry, DEFAULT_AXIS_LENGTH_M};
use crate::tf_refresh::TfRegistry;
use crate::urdf::UrdfModel;

/// Snapshot of every topic currently visible on the ROS graph, refreshed by
/// the executor thread. Each entry is `(topic, [msg_type, ...])`.
pub type TopicsSnapshot = Arc<RwLock<Vec<(String, Vec<String>)>>>;

pub struct RosNode {
    handle: Option<thread::JoinHandle<()>>,
    shutdown: Sender<()>,
    stats: Arc<RosStats>,
    /// Shared TF axis length (meters) — UI mutates, executor thread reads.
    /// Stored as `f32::to_bits` in an atomic.
    tf_axis_length: Arc<AtomicU32>,
    /// Latest topic-graph snapshot, refreshed roughly once per second by the
    /// executor. Used by the UI's topic discoverer.
    topics: TopicsSnapshot,
    /// Runtime "Add topic" requests from the UI. The executor drains this each
    /// spin and spawns the matching subscriber (RViz-style live add).
    add_topic: Sender<(String, TopicKind)>,
}

impl RosNode {
    pub fn spawn(scene: SceneHandle, cfg: RosConfig) -> Result<Self> {
        let (tx, rx) = bounded::<()>(1);
        let stats = Arc::new(RosStats::default());
        let stats_thread = stats.clone();
        let tf_axis_length =
            Arc::new(AtomicU32::new(DEFAULT_AXIS_LENGTH_M.to_bits()));
        let tf_axis_length_thread = tf_axis_length.clone();
        let topics: TopicsSnapshot = Arc::new(RwLock::new(Vec::new()));
        let topics_thread = topics.clone();
        let (add_tx, add_rx) = unbounded::<(String, TopicKind)>();
        let handle = thread::Builder::new()
            .name("ros2-executor".into())
            .spawn(move || {
                if let Err(e) = run(
                    scene,
                    cfg,
                    rx,
                    stats_thread,
                    tf_axis_length_thread,
                    topics_thread,
                    add_rx,
                ) {
                    log::error!("ros2 executor exited with error: {e:#}");
                }
            })
            .context("spawning ros2-executor thread")?;
        Ok(RosNode {
            handle: Some(handle),
            shutdown: tx,
            stats,
            tf_axis_length,
            topics,
            add_topic: add_tx,
        })
    }

    /// Request a live subscription to `topic` as `kind`. Non-blocking; the
    /// executor picks it up on its next spin. A double-add is harmless — the
    /// subscriber registry ignores topics it already owns.
    pub fn request_add_topic(&self, topic: String, kind: TopicKind) {
        if let Err(e) = self.add_topic.send((topic, kind)) {
            log::warn!("add-topic request dropped (executor gone): {e}");
        }
    }

    /// Cross-thread counters (e.g. PC2 messages received). Cheap atomic loads.
    pub fn stats(&self) -> &Arc<RosStats> {
        &self.stats
    }

    /// Shared handle to the live TF axis length (meters). Stored as
    /// `f32::to_bits` in an atomic; use `f32::from_bits` / `f32::to_bits`
    /// when reading or writing.
    pub fn tf_axis_length_handle(&self) -> &Arc<AtomicU32> {
        &self.tf_axis_length
    }

    /// Latest topic-graph snapshot from the executor thread.
    pub fn topics(&self) -> &TopicsSnapshot {
        &self.topics
    }

    pub fn shutdown(mut self) {
        let _ = self.shutdown.send(());
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

impl Drop for RosNode {
    fn drop(&mut self) {
        let _ = self.shutdown.send(());
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn run(
    scene: SceneHandle,
    cfg: RosConfig,
    shutdown: Receiver<()>,
    stats: Arc<RosStats>,
    tf_axis_length: Arc<AtomicU32>,
    topics: TopicsSnapshot,
    add_topic: Receiver<(String, TopicKind)>,
) -> Result<()> {
    let ctx = r2r::Context::create().context("r2r::Context::create")?;
    let mut node =
        r2r::Node::create(ctx, &cfg.node_name, &cfg.namespace).context("r2r::Node::create")?;
    log::info!(
        "ros2 node up: name={} namespace={:?}",
        cfg.node_name,
        cfg.namespace
    );

    let pool = LocalPool::new();
    let spawner = pool.spawner();
    let tf_tree = Arc::new(TfTree::new());
    let tf_refresh = TfRegistry::new();
    let tf_axes = TfAxesRegistry::with_scale(tf_axis_length);

    subscribers::tf::spawn(&mut node, &spawner, tf_tree.clone(), &cfg).context("TF subscribers")?;
    let mut registry = subscribers::discovery::bootstrap(
        &mut node,
        &spawner,
        scene.clone(),
        tf_tree.clone(),
        tf_refresh.clone(),
        &cfg,
        stats.clone(),
    )
    .context("subscriber bootstrap")?;

    if let Some(urdf_path) = cfg.urdf_path.clone() {
        match UrdfModel::load(&urdf_path) {
            Ok((model, entities)) => init_urdf(
                &mut node,
                &spawner,
                &scene,
                &tf_tree,
                &tf_refresh,
                &cfg,
                model,
                entities,
            )
            .context("init urdf from file")?,
            Err(e) => log::error!("urdf load failed: {e:#}"),
        }
    }
    // URDF over topic: subscribe once, latch the first message into shared
    // state, and let the main loop drain it (needs `&mut node` to spawn the
    // JointState subscriber, which we don't have inside the async task).
    let pending_urdf_text: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
    let mut urdf_topic_seen = cfg.urdf_path.is_some();
    if !urdf_topic_seen {
        if let Some(topic) = cfg.urdf_topic.clone() {
            spawn_urdf_topic_subscriber(
                &mut node,
                &spawner,
                pending_urdf_text.clone(),
                topic,
                cfg.urdf_topic_qos.clone(),
            )
            .context("URDF topic subscriber")?;
        } else {
            urdf_topic_seen = true; // nothing to wait for
        }
    }

    // Discovery cadence: 20 ms ticks → poll every 50 iterations ≈ 1 Hz.
    const DISCOVERY_EVERY_N_TICKS: u32 = 50;
    let mut tick_count: u32 = 0;

    let mut pool = pool;
    while shutdown.try_recv().is_err() {
        // Block on the rcl wait set for up to 20ms for new work.
        node.spin_once(Duration::from_millis(20));
        // r2r's spin_once takes exactly one rcl message per ready subscriber
        // per call. Latched (TRANSIENT_LOCAL) topics like /tf_static can land
        // several messages back-to-back at startup as each publisher's latched
        // message is delivered — drain them in the same iteration so downstream
        // subscribers see a fully-populated TF tree before their first lookup.
        for _ in 0..32 {
            node.spin_once(Duration::ZERO);
        }
        pool.run_until_stalled();

        // Drain runtime "Add topic" requests from the UI and spawn each one.
        while let Ok((topic, kind)) = add_topic.try_recv() {
            if let Err(e) = subscribers::discovery::spawn_for_kind(
                &mut node,
                &spawner,
                &scene,
                &tf_tree,
                &tf_refresh,
                &cfg,
                &mut registry,
                &stats,
                &topic,
                kind,
            ) {
                log::warn!("failed to add topic {topic}: {e:#}");
            }
        }

        // Late URDF arrival (from the /robot_description-style topic): consume
        // the first message, parse, spawn entities + JointState subscriber.
        if !urdf_topic_seen {
            let text = pending_urdf_text.lock().take();
            if let Some(text) = text {
                match UrdfModel::load_from_text(&text) {
                    Ok((model, entities)) => {
                        if let Err(e) = init_urdf(
                            &mut node,
                            &spawner,
                            &scene,
                            &tf_tree,
                            &tf_refresh,
                            &cfg,
                            model,
                            entities,
                        ) {
                            log::error!("urdf init (from topic) failed: {e:#}");
                        }
                    }
                    Err(e) => log::error!("urdf parse (from topic) failed: {e:#}"),
                }
                urdf_topic_seen = true;
            }
        }
        // Re-evaluate world transforms for every TF-bound entity using the
        // current TF tree. This is what makes a late /tf message *retroactively*
        // fix a scan/cloud/URDF that was rendered with `IDENTITY` because TF
        // wasn't ready when its message arrived.
        tf_refresh.refresh(&tf_tree, &cfg.reference_frame, &scene);
        // Re-publish per-frame axis entities so the user gets an RViz-style TF
        // display they can toggle per-frame in the side panel.
        tf_axes.refresh(&tf_tree, &cfg.reference_frame, &scene);
        tick_count = tick_count.wrapping_add(1);
        if tick_count.is_multiple_of(DISCOVERY_EVERY_N_TICKS) {
            if let Err(e) = subscribers::discovery::tick(
                &mut node,
                &spawner,
                &scene,
                &tf_tree,
                &tf_refresh,
                &cfg,
                &mut registry,
                &stats,
            ) {
                log::warn!("discovery tick failed: {e:#}");
            }
            // Refresh the shared topic-graph snapshot for the UI's topic
            // discoverer. Cheap (single rcl call) and runs unconditionally —
            // wildcard config is no longer the only consumer.
            match node.get_topic_names_and_types() {
                Ok(snapshot) => {
                    let mut list: Vec<(String, Vec<String>)> =
                        snapshot.into_iter().collect();
                    list.sort_by(|a, b| a.0.cmp(&b.0));
                    *topics.write() = list;
                }
                Err(e) => log::warn!("topic snapshot failed: {e}"),
            }
        }
    }

    log::info!("ros2 node shutting down");
    Ok(())
}

/// Push a freshly-parsed URDF into the scene graph + register every link with
/// the TF registry + spawn the JointState subscriber. Shared between the
/// "load from file at startup" and "first message on the URDF topic" paths.
#[allow(clippy::too_many_arguments)]
fn init_urdf(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: &SceneHandle,
    tf_tree: &Arc<TfTree>,
    tf_refresh: &Arc<TfRegistry>,
    cfg: &RosConfig,
    model: UrdfModel,
    entities: Vec<scene::SceneEntity>,
) -> Result<()> {
    {
        let mut scene_w = scene.write();
        for entity in entities {
            scene_w.upsert(entity);
        }
    }
    let model = Arc::new(Mutex::new(model));
    // Push transforms with all joints at zero so the robot is visible before
    // the first JointState message arrives, and bind every link to the TF
    // registry so late /tf hops repair its pose.
    {
        let m = model.lock();
        subscribers::jointstate::push_link_transforms(
            scene,
            tf_tree,
            &cfg.reference_frame,
            &m,
        );
        subscribers::jointstate::register_link_transforms(tf_refresh, &m);
    }
    subscribers::jointstate::spawn_topic(
        node,
        spawner,
        scene.clone(),
        tf_tree.clone(),
        cfg.reference_frame.clone(),
        model,
        cfg.joint_states_topic.clone(),
        cfg.joint_states_qos.clone(),
    )
    .context("JointState subscriber")?;
    Ok(())
}

/// Subscribe to a `std_msgs/String` URDF topic (typically
/// `/robot_description`, latched by `robot_state_publisher`). The first
/// message is stored in `pending`; the main spin loop drains it and runs the
/// real URDF init. Defaults to TRANSIENT_LOCAL QoS so latched publishers are
/// picked up; can be overridden via `[urdf].topic_qos` in the TOML config.
fn spawn_urdf_topic_subscriber(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    pending: Arc<Mutex<Option<String>>>,
    topic: String,
    qos_override: Option<crate::config::QosOverride>,
) -> Result<()> {
    let mut qos = QosProfile::default()
        .keep_last(1)
        .durability(DurabilityPolicy::TransientLocal);
    if let Some(o) = &qos_override {
        qos = o.apply(qos);
    }
    let mut sub = node
        .subscribe::<r2r::std_msgs::msg::String>(&topic, qos)
        .with_context(|| format!("subscribing to {topic}"))?;
    log::info!("subscribed: {topic} (std_msgs/String — URDF)");
    let topic_owned = topic;
    spawner
        .spawn_local(async move {
            while let Some(msg) = sub.next().await {
                let mut slot = pending.lock();
                if slot.is_none() {
                    log::info!(
                        "{topic_owned}: received URDF ({} bytes)",
                        msg.data.len()
                    );
                }
                *slot = Some(msg.data);
            }
        })
        .context("spawning URDF topic task")?;
    Ok(())
}
