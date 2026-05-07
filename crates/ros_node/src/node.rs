//! Node lifecycle: spawn a dedicated thread for the r2r executor and shut it
//! down cleanly when the app exits.

use std::sync::Arc;
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result};
use crossbeam_channel::{bounded, Receiver, Sender};
use futures::executor::LocalPool;
use scene::SceneHandle;

use crate::config::RosConfig;
use crate::subscribers;
use crate::tf::TfTree;

pub struct RosNode {
    handle: Option<thread::JoinHandle<()>>,
    shutdown: Sender<()>,
}

impl RosNode {
    pub fn spawn(scene: SceneHandle, cfg: RosConfig) -> Result<Self> {
        let (tx, rx) = bounded::<()>(1);
        let handle = thread::Builder::new()
            .name("ros2-executor".into())
            .spawn(move || {
                if let Err(e) = run(scene, cfg, rx) {
                    log::error!("ros2 executor exited with error: {e:#}");
                }
            })
            .context("spawning ros2-executor thread")?;
        Ok(RosNode {
            handle: Some(handle),
            shutdown: tx,
        })
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

fn run(scene: SceneHandle, cfg: RosConfig, shutdown: Receiver<()>) -> Result<()> {
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

    subscribers::tf::spawn(&mut node, &spawner, tf_tree.clone()).context("TF subscribers")?;
    let mut registry = subscribers::discovery::bootstrap(
        &mut node,
        &spawner,
        scene.clone(),
        tf_tree.clone(),
        &cfg,
    )
    .context("subscriber bootstrap")?;

    // Discovery cadence: 20 ms ticks → poll every 50 iterations ≈ 1 Hz.
    const DISCOVERY_EVERY_N_TICKS: u32 = 50;
    let mut tick_count: u32 = 0;

    let mut pool = pool;
    while shutdown.try_recv().is_err() {
        node.spin_once(Duration::from_millis(20));
        pool.run_until_stalled();
        tick_count = tick_count.wrapping_add(1);
        if tick_count.is_multiple_of(DISCOVERY_EVERY_N_TICKS) {
            if let Err(e) = subscribers::discovery::tick(
                &mut node,
                &spawner,
                &scene,
                &tf_tree,
                &cfg,
                &mut registry,
            ) {
                log::warn!("discovery tick failed: {e:#}");
            }
        }
    }

    log::info!("ros2 node shutting down");
    Ok(())
}
