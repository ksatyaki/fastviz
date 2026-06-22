//! `sensor_msgs/PointCloud2` → `scene::ScenePrimitive::Points`.
//!
//! Walks `fields[]` once per message to locate x/y/z floats, then strides the
//! flat byte buffer at `point_step`. Non-finite values are skipped. The
//! decoded `Vec<Point>` buffer is reused across messages, and `style.stride`
//! lets users decimate dense LiDAR clouds (`stride = 4` keeps every 4th point).
//!
//! Endianness: only little-endian is supported in M0.5. Intensity / RGB
//! coloring is not yet wired up — every point gets the uniform `style.color`.

use std::sync::atomic::Ordering;
use std::sync::Arc;

use anyhow::{Context, Result};
use futures::executor::LocalSpawner;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use glam::{Mat4, Vec3};
use r2r::sensor_msgs::msg::{PointCloud2, PointField};
use r2r::QosProfile;
use scene::{Point, SceneEntity, SceneHandle, ScenePrimitive};

use crate::config::{PointCloudStyle, QosOverride};
use crate::coords::ROS_TO_WORLD;
use crate::ids::pointcloud_id;
use crate::stats::RosStats;
use crate::tf::TfTree;
use crate::tf_refresh::TfRegistry;

pub const MSG_TYPE: &str = "sensor_msgs/msg/PointCloud2";

// PointField datatype enum (see sensor_msgs/msg/PointField).
const PF_FLOAT32: u8 = 7;
const PF_FLOAT64: u8 = 8;

#[derive(Copy, Clone, Debug)]
struct XyzLayout {
    x_off: usize,
    y_off: usize,
    z_off: usize,
    /// True if x,y,z are FLOAT32; false if FLOAT64. Mixed types are rejected.
    f32: bool,
}

fn locate_xyz(fields: &[PointField]) -> Option<XyzLayout> {
    let mut x = None;
    let mut y = None;
    let mut z = None;
    for f in fields {
        match f.name.as_str() {
            "x" => x = Some((f.offset as usize, f.datatype)),
            "y" => y = Some((f.offset as usize, f.datatype)),
            "z" => z = Some((f.offset as usize, f.datatype)),
            _ => {}
        }
    }
    let (x_off, x_dt) = x?;
    let (y_off, y_dt) = y?;
    let (z_off, z_dt) = z?;
    let f32 = match (x_dt, y_dt, z_dt) {
        (PF_FLOAT32, PF_FLOAT32, PF_FLOAT32) => true,
        (PF_FLOAT64, PF_FLOAT64, PF_FLOAT64) => false,
        _ => return None,
    };
    Some(XyzLayout {
        x_off,
        y_off,
        z_off,
        f32,
    })
}

#[allow(clippy::too_many_arguments)]
pub fn spawn_topic(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: SceneHandle,
    tf: Arc<TfTree>,
    tf_refresh: Arc<TfRegistry>,
    reference_frame: String,
    style: PointCloudStyle,
    topic: String,
    topic_idx: usize,
    qos_override: Option<QosOverride>,
    stats: Arc<RosStats>,
) -> Result<()> {
    // sensor_data: best_effort, depth 5 — matches typical lidar/depth publishers.
    let mut qos = QosProfile::sensor_data();
    if let Some(o) = &qos_override {
        qos = o.apply(qos);
        log::debug!("qos override applied to {topic}: {o:?}");
    }
    let mut sub = node
        .subscribe::<PointCloud2>(&topic, qos)
        .with_context(|| format!("subscribing to {topic}"))?;
    log::info!("subscribed: {topic} (sensor_msgs/PointCloud2)");
    let id = pointcloud_id(topic_idx);
    let stride = style.stride.max(1);

    spawner
        .spawn_local(async move {
            let mut first = true;
            // Re-warn only on success→failure transitions so a continuously
            // broken TF chain doesn't spam at the publish rate.
            let mut warned_missing_tf = false;
            let mut points: Vec<Point> = Vec::new();
            while let Some(msg) = sub.next().await {
                if msg.is_bigendian {
                    log::warn!("{topic}: big-endian PointCloud2 not supported, dropping");
                    continue;
                }
                let Some(layout) = locate_xyz(&msg.fields) else {
                    log::warn!(
                        "{topic}: missing/incompatible x,y,z fields, dropping (fields: {})",
                        msg.fields
                            .iter()
                            .map(|f| f.name.as_str())
                            .collect::<Vec<_>>()
                            .join(",")
                    );
                    continue;
                };

                let n_total = (msg.width as usize).saturating_mul(msg.height as usize);
                let point_step = msg.point_step as usize;
                let scalar_size = if layout.f32 { 4 } else { 8 };
                if layout.x_off + scalar_size > point_step
                    || layout.y_off + scalar_size > point_step
                    || layout.z_off + scalar_size > point_step
                {
                    log::warn!("{topic}: xyz offsets exceed point_step, dropping");
                    continue;
                }
                if msg.data.len() < n_total.saturating_mul(point_step) {
                    log::warn!(
                        "{topic}: data buffer too small ({} < {}*{}), dropping",
                        msg.data.len(),
                        n_total,
                        point_step
                    );
                    continue;
                }

                let est = n_total / stride + 1;
                points.clear();
                if points.capacity() < est {
                    points.reserve(est - points.capacity());
                }
                for i in (0..n_total).step_by(stride) {
                    let base = i * point_step;
                    let x = read_scalar(&msg.data, base + layout.x_off, layout.f32);
                    let y = read_scalar(&msg.data, base + layout.y_off, layout.f32);
                    let z = read_scalar(&msg.data, base + layout.z_off, layout.f32);
                    if !x.is_finite() || !y.is_finite() || !z.is_finite() {
                        continue;
                    }
                    points.push(Point {
                        position: Vec3::new(x, y, z),
                        color: style.color,
                        size: style.size,
                    });
                }

                let frame = &msg.header.frame_id;
                let stamp_ns = (msg.header.stamp.sec as i64) * 1_000_000_000
                    + (msg.header.stamp.nanosec as i64);
                let tf_ref_from_frame = if frame == &reference_frame {
                    warned_missing_tf = false;
                    Mat4::IDENTITY
                } else {
                    match tf.lookup_at(&reference_frame, frame, stamp_ns) {
                        Some(m) => {
                            warned_missing_tf = false;
                            m
                        }
                        None => {
                            if !warned_missing_tf {
                                log::warn!(
                                    "{topic}: tf lookup {frame} -> {reference_frame} unavailable — {}. \
                                     Rendering cloud at world origin.",
                                    tf.diagnose_lookup(&reference_frame, frame)
                                );
                                warned_missing_tf = true;
                            }
                            Mat4::IDENTITY
                        }
                    }
                };
                let transform = ROS_TO_WORLD * tf_ref_from_frame;

                if first {
                    log::info!(
                        "{topic}: first message ({} of {n_total} points kept, stride={stride}, frame={frame})",
                        points.len()
                    );
                    first = false;
                }

                // Move the buffer into the entity (no 5 MB memcpy). The
                // local `points` is replaced with an empty Vec; the next
                // iteration's `points.reserve` re-allocates from scratch.
                let entity = SceneEntity::new(id, ScenePrimitive::Points(std::mem::take(&mut points)))
                    .with_transform(transform)
                    .with_label(topic.clone());
                scene.write().upsert(entity);
                // (Re-)bind to the registry so late /tf arrivals get applied
                // by the main loop's refresh pass even if no fresh cloud
                // message comes in. Points are already in `frame` coords.
                tf_refresh.register_at(id, frame.as_str(), Mat4::IDENTITY, Some(stamp_ns));
                stats.pc2_received.fetch_add(1, Ordering::Relaxed);
            }
        })
        .context("spawning pointcloud subscriber task")?;
    Ok(())
}

fn read_scalar(buf: &[u8], offset: usize, is_f32: bool) -> f32 {
    if is_f32 {
        let b = &buf[offset..offset + 4];
        f32::from_le_bytes([b[0], b[1], b[2], b[3]])
    } else {
        let b = &buf[offset..offset + 8];
        f64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]) as f32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pf(name: &str, offset: u32, datatype: u8) -> PointField {
        PointField {
            name: name.into(),
            offset,
            datatype,
            count: 1,
        }
    }

    #[test]
    fn locate_xyz_f32_typical() {
        let fields = vec![
            pf("x", 0, PF_FLOAT32),
            pf("y", 4, PF_FLOAT32),
            pf("z", 8, PF_FLOAT32),
            pf("intensity", 12, PF_FLOAT32),
        ];
        let l = locate_xyz(&fields).unwrap();
        assert_eq!((l.x_off, l.y_off, l.z_off), (0, 4, 8));
        assert!(l.f32);
    }

    #[test]
    fn locate_xyz_missing_returns_none() {
        let fields = vec![pf("x", 0, PF_FLOAT32), pf("y", 4, PF_FLOAT32)];
        assert!(locate_xyz(&fields).is_none());
    }

    #[test]
    fn locate_xyz_mixed_types_rejected() {
        let fields = vec![
            pf("x", 0, PF_FLOAT32),
            pf("y", 4, PF_FLOAT64),
            pf("z", 12, PF_FLOAT32),
        ];
        assert!(locate_xyz(&fields).is_none());
    }

    #[test]
    fn read_scalar_le_f32() {
        let mut buf = [0u8; 4];
        buf.copy_from_slice(&1.5f32.to_le_bytes());
        assert_eq!(read_scalar(&buf, 0, true), 1.5);
    }
}
