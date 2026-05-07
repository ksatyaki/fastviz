//! Subscribers for `/tf` (volatile) and `/tf_static` (transient_local).
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

use crate::tf::TfTree;

pub fn spawn(node: &mut r2r::Node, spawner: &LocalSpawner, tree: Arc<TfTree>) -> Result<()> {
    spawn_one(
        node,
        spawner,
        tree.clone(),
        "/tf",
        QosProfile::default().keep_last(100),
    )?;
    spawn_one(
        node,
        spawner,
        tree,
        "/tf_static",
        QosProfile::default()
            .keep_last(100)
            .durability(DurabilityPolicy::TransientLocal),
    )?;
    Ok(())
}

fn spawn_one(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    tree: Arc<TfTree>,
    topic: &str,
    qos: QosProfile,
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
                if !logged_first && n > 0 {
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
