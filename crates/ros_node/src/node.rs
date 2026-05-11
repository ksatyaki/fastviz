//! Node lifecycle: spawn a dedicated thread for the r2r executor and shut it
//! down cleanly when the app exits.

use std::sync::Arc;
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result};
use crossbeam_channel::{bounded, Receiver, Sender};
use futures::executor::LocalPool;
use parking_lot::Mutex;
use scene::SceneHandle;

use crate::config::RosConfig;
use crate::stats::RosStats;
use crate::subscribers;
use crate::tf::TfTree;
use crate::tf_refresh::TfRegistry;
use crate::urdf::UrdfModel;

pub struct RosNode {
    handle: Option<thread::JoinHandle<()>>,
    shutdown: Sender<()>,
    stats: Arc<RosStats>,
}

impl RosNode {
    pub fn spawn(scene: SceneHandle, cfg: RosConfig) -> Result<Self> {
        let (tx, rx) = bounded::<()>(1);
        let stats = Arc::new(RosStats::default());
        let stats_thread = stats.clone();
        let handle = thread::Builder::new()
            .name("ros2-executor".into())
            .spawn(move || {
                if let Err(e) = run(scene, cfg, rx, stats_thread) {
                    log::error!("ros2 executor exited with error: {e:#}");
                }
            })
            .context("spawning ros2-executor thread")?;
        Ok(RosNode {
            handle: Some(handle),
            shutdown: tx,
            stats,
        })
    }

    /// Cross-thread counters (e.g. PC2 messages received). Cheap atomic loads.
    pub fn stats(&self) -> &Arc<RosStats> {
        &self.stats
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

fn run(
    scene: SceneHandle,
    cfg: RosConfig,
    shutdown: Receiver<()>,
    stats: Arc<RosStats>,
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

    subscribers::tf::spawn(&mut node, &spawner, tf_tree.clone()).context("TF subscribers")?;
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
            Ok((model, entities)) => {
                {
                    let mut scene_w = scene.write();
                    for entity in entities {
                        scene_w.upsert(entity);
                    }
                }
                let model = Arc::new(Mutex::new(model));
                // Push transforms with all joints at zero so the robot is visible
                // before the first JointState message arrives, and bind every
                // link to the TF registry so late /tf hops repair its pose.
                {
                    let m = model.lock();
                    subscribers::jointstate::push_link_transforms(
                        &scene,
                        &tf_tree,
                        &cfg.reference_frame,
                        &m,
                    );
                    subscribers::jointstate::register_link_transforms(&tf_refresh, &m);
                }
                subscribers::jointstate::spawn_topic(
                    &mut node,
                    &spawner,
                    scene.clone(),
                    tf_tree.clone(),
                    tf_refresh.clone(),
                    cfg.reference_frame.clone(),
                    model,
                    cfg.joint_states_topic.clone(),
                    cfg.joint_states_qos.clone(),
                )
                .context("JointState subscriber")?;
            }
            Err(e) => log::error!("urdf load failed: {e:#}"),
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
        // Re-evaluate world transforms for every TF-bound entity using the
        // current TF tree. This is what makes a late /tf message *retroactively*
        // fix a scan/cloud/URDF that was rendered with `IDENTITY` because TF
        // wasn't ready when its message arrived.
        tf_refresh.refresh(&tf_tree, &cfg.reference_frame, &scene);
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
        }
    }

    log::info!("ros2 node shutting down");
    Ok(())
}
