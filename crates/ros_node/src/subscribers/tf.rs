//! Subscribers for the dynamic-frame TF topic (default `/tf`, volatile) and
//! the static-frame TF topic (default `/tf_static`, transient_local). Both
//! topic names come from `RosConfig` and can be remapped in TOML.
//!
//! Both feed the same `TfTree`; static messages just have latched durability
//! so late subscribers get the latest set.

use std::sync::Arc;

use anyhow::{Context, Result};
use futures::executor::LocalSpawner;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use r2r::qos::DurabilityPolicy;
use r2r::{tf2_msgs, QosProfile};

use crate::config::{QosOverride, RosConfig};
use crate::tf::TfTree;

pub fn spawn(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    tree: Arc<TfTree>,
    cfg: &RosConfig,
) -> Result<()> {
    spawn_one(
        node,
        spawner,
        tree.clone(),
        &cfg.tf_topic,
        apply_qos(QosProfile::default().keep_last(100), cfg.tf_qos.as_ref()),
        false,
    )?;
    // The static TF topic is low-rate (latched messages, one per publisher)
    // but load-bearing for every downstream lookup. Log every message at INFO
    // so misconfigurations show up immediately rather than disappearing into
    // TRACE.
    spawn_one(
        node,
        spawner,
        tree,
        &cfg.tf_static_topic,
        apply_qos(
            QosProfile::default()
                .keep_last(100)
                .durability(DurabilityPolicy::TransientLocal),
            cfg.tf_static_qos.as_ref(),
        ),
        true,
    )?;
    Ok(())
}

fn apply_qos(base: QosProfile, o: Option<&QosOverride>) -> QosProfile {
    match o {
        Some(o) => o.apply(base),
        None => base,
    }
}

fn spawn_one(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    tree: Arc<TfTree>,
    topic: &str,
    qos: QosProfile,
    log_every: bool,
) -> Result<()> {
    let mut sub = node
        .subscribe::<tf2_msgs::msg::TFMessage>(topic, qos)
        .with_context(|| format!("subscribing to {topic}"))?;
    log::info!("subscribed: {topic} (tf2_msgs/TFMessage)");

    let topic_owned = topic.to_string();
    spawner
        .spawn_local(async move {
            let mut total: u64 = 0;
            let mut logged_first = false;
            while let Some(msg) = sub.next().await {
                let n = msg.transforms.len();
                tree.update(&msg);
                total += n as u64;
                if log_every {
                    log::info!(
                        "{topic_owned}: +{n} transforms (cumulative {total}; tree size = {})",
                        tree.frame_count()
                    );
                } else if !logged_first && n > 0 {
                    log::info!(
                        "{topic_owned}: first message ({n} transforms; tree size = {})",
                        tree.frame_count()
                    );
                    logged_first = true;
                } else {
                    log::trace!("{topic_owned}: +{n} (cumulative {total})");
                }
            }
        })
        .with_context(|| format!("spawning {topic} task"))?;
    Ok(())
}
