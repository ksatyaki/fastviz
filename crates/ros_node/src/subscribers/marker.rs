//! `visualization_msgs/Marker` and `visualization_msgs/MarkerArray` →
//! `ScenePrimitive`.
//!
//! One subscriber per topic. Each `(ns, id)` pair from the publisher maps to a
//! unique `EntityId` allocated sequentially within the topic's slab in
//! [`crate::ids::ROS_ID_MARKER_BASE`]. `DELETE` removes one entity, `DELETEALL`
//! drops every entity owned by this topic. Per-message `header.frame_id` drives
//! the TF lookup; the entity is also registered with [`TfRegistry`] so late
//! `/tf` arrivals retroactively fix its world pose.
//!
//! Supported marker types (REP-105 / visualization_msgs::msg::Marker):
//! `ARROW`, `CUBE`, `SPHERE`, `CYLINDER`, `LINE_STRIP`, `CUBE_LIST`,
//! `SPHERE_LIST`, `POINTS`, `TEXT_VIEW_FACING`, `TRIANGLE_LIST`. `LINE_LIST`,
//! `ARROW_STRIP`, `MESH_RESOURCE` log a one-time warning per topic.

use std::collections::HashMap;
use std::hash::{Hash, Hasher};
use std::sync::Arc;

use anyhow::{Context, Result};
use futures::executor::LocalSpawner;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use glam::{Mat4, Quat, Vec3};
use parking_lot::Mutex;
use r2r::visualization_msgs::msg::{Marker, MarkerArray};
use r2r::QosProfile;
use scene::{
    Arrow, Color, EntityId, Label, Material, Mesh, Point, Polyline, SceneEntity, SceneHandle,
    ScenePrimitive, Vertex,
};

use crate::config::QosOverride;
use crate::coords::ROS_TO_WORLD;
use crate::ids::{marker_topic_base, ROS_ID_MARKER_PER_TOPIC};
use crate::tf::TfTree;
use crate::tf_refresh::TfRegistry;
use crate::urdf::{box_mesh, cylinder_mesh, sphere_mesh};

pub const MARKER_TYPE: &str = "visualization_msgs/msg/Marker";
pub const ARRAY_TYPE: &str = "visualization_msgs/msg/MarkerArray";

// Marker.type_ constants (mirrors visualization_msgs::msg::Marker::*).
const T_ARROW: i32 = 0;
const T_CUBE: i32 = 1;
const T_SPHERE: i32 = 2;
const T_CYLINDER: i32 = 3;
const T_LINE_STRIP: i32 = 4;
const T_LINE_LIST: i32 = 5;
const T_CUBE_LIST: i32 = 6;
const T_SPHERE_LIST: i32 = 7;
const T_POINTS: i32 = 8;
const T_TEXT_VIEW_FACING: i32 = 9;
const T_MESH_RESOURCE: i32 = 10;
const T_TRIANGLE_LIST: i32 = 11;

// Marker.action constants. ADD (0) and MODIFY (0) share a value; we treat them
// identically, so we only define one.
const A_ADD: i32 = 0;
const A_DELETE: i32 = 2;
const A_DELETEALL: i32 = 3;

/// Per-topic mutable state shared between the marker stream task and any
/// future helpers. Holds the (ns, id) → EntityId allocation map and a small
/// "unsupported type warned" set so the log doesn't spam.
#[derive(Default)]
struct TopicState {
    slab_base: u64,
    next_slot: u64,
    entities: HashMap<(String, i32), EntityId>,
    /// Hash of the last *payload* we accepted for each (ns, id). Bumped only
    /// when the marker's render-relevant fields change; world transform is
    /// deliberately excluded (TF motion is handled by `tf_refresh`, not by
    /// re-upserting the entity).
    last_payload_hash: HashMap<(String, i32), u64>,
    warned_unsupported: std::collections::HashSet<i32>,
}

impl TopicState {
    fn new(slab_base: u64) -> Self {
        TopicState {
            slab_base,
            next_slot: 0,
            entities: HashMap::new(),
            last_payload_hash: HashMap::new(),
            warned_unsupported: std::collections::HashSet::new(),
        }
    }

    fn allocate(&mut self, ns: &str, id: i32) -> Option<EntityId> {
        let key = (ns.to_string(), id);
        if let Some(&eid) = self.entities.get(&key) {
            return Some(eid);
        }
        if self.next_slot >= ROS_ID_MARKER_PER_TOPIC {
            return None;
        }
        let eid = EntityId(self.slab_base + self.next_slot);
        self.next_slot += 1;
        self.entities.insert(key, eid);
        Some(eid)
    }

    fn remove(&mut self, ns: &str, id: i32) -> Option<EntityId> {
        self.last_payload_hash.remove(&(ns.to_string(), id));
        self.entities.remove(&(ns.to_string(), id))
    }
}

/// Hash of the render-relevant marker fields. Excludes `header.stamp` (often
/// `now()` even on identical content) and the looked-up world transform (TF
/// changes are dispatched via `tf_refresh`, not by re-upserting the entity).
fn payload_hash(m: &Marker) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    m.type_.hash(&mut h);
    m.action.hash(&mut h);
    m.header.frame_id.hash(&mut h);
    let p = &m.pose;
    p.position.x.to_bits().hash(&mut h);
    p.position.y.to_bits().hash(&mut h);
    p.position.z.to_bits().hash(&mut h);
    p.orientation.x.to_bits().hash(&mut h);
    p.orientation.y.to_bits().hash(&mut h);
    p.orientation.z.to_bits().hash(&mut h);
    p.orientation.w.to_bits().hash(&mut h);
    m.scale.x.to_bits().hash(&mut h);
    m.scale.y.to_bits().hash(&mut h);
    m.scale.z.to_bits().hash(&mut h);
    m.color.r.to_bits().hash(&mut h);
    m.color.g.to_bits().hash(&mut h);
    m.color.b.to_bits().hash(&mut h);
    m.color.a.to_bits().hash(&mut h);
    m.text.hash(&mut h);
    (m.points.len() as u64).hash(&mut h);
    for pt in &m.points {
        pt.x.to_bits().hash(&mut h);
        pt.y.to_bits().hash(&mut h);
        pt.z.to_bits().hash(&mut h);
    }
    (m.colors.len() as u64).hash(&mut h);
    for c in &m.colors {
        c.r.to_bits().hash(&mut h);
        c.g.to_bits().hash(&mut h);
        c.b.to_bits().hash(&mut h);
        c.a.to_bits().hash(&mut h);
    }
    h.finish()
}

#[allow(clippy::too_many_arguments)]
pub fn spawn_marker_topic(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: SceneHandle,
    tf: Arc<TfTree>,
    tf_refresh: Arc<TfRegistry>,
    reference_frame: String,
    topic: String,
    topic_idx: usize,
    qos_override: Option<QosOverride>,
) -> Result<()> {
    let mut qos = QosProfile::default().keep_last(100);
    if let Some(o) = &qos_override {
        qos = o.apply(qos);
        log::debug!("qos override applied to {topic}: {o:?}");
    }
    let mut sub = node
        .subscribe::<Marker>(&topic, qos)
        .with_context(|| format!("subscribing to {topic}"))?;
    log::info!("subscribed: {topic} (visualization_msgs/Marker)");
    let state = Arc::new(Mutex::new(TopicState::new(marker_topic_base(topic_idx))));

    spawner
        .spawn_local(async move {
            let mut first = true;
            while let Some(msg) = sub.next().await {
                if first {
                    log::info!(
                        "{topic}: first marker (ns={} id={} type={} action={})",
                        msg.ns,
                        msg.id,
                        msg.type_,
                        msg.action
                    );
                    first = false;
                }
                process_marker_batch(
                    std::slice::from_ref(&msg),
                    &topic,
                    &reference_frame,
                    &tf,
                    &tf_refresh,
                    &scene,
                    &state,
                );
            }
        })
        .context("spawning marker subscriber task")?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub fn spawn_marker_array_topic(
    node: &mut r2r::Node,
    spawner: &LocalSpawner,
    scene: SceneHandle,
    tf: Arc<TfTree>,
    tf_refresh: Arc<TfRegistry>,
    reference_frame: String,
    topic: String,
    topic_idx: usize,
    qos_override: Option<QosOverride>,
) -> Result<()> {
    let mut qos = QosProfile::default().keep_last(100);
    if let Some(o) = &qos_override {
        qos = o.apply(qos);
        log::debug!("qos override applied to {topic}: {o:?}");
    }
    let mut sub = node
        .subscribe::<MarkerArray>(&topic, qos)
        .with_context(|| format!("subscribing to {topic}"))?;
    log::info!("subscribed: {topic} (visualization_msgs/MarkerArray)");
    let state = Arc::new(Mutex::new(TopicState::new(marker_topic_base(topic_idx))));

    spawner
        .spawn_local(async move {
            let mut first = true;
            while let Some(msg) = sub.next().await {
                if first {
                    log::info!("{topic}: first MarkerArray ({} markers)", msg.markers.len());
                    first = false;
                }
                process_marker_batch(
                    &msg.markers,
                    &topic,
                    &reference_frame,
                    &tf,
                    &tf_refresh,
                    &scene,
                    &state,
                );
            }
        })
        .context("spawning MarkerArray subscriber task")?;
    Ok(())
}

/// Pending TF refresh registration accumulated while building entities; flushed
/// after the scene write lock is released so we don't hold both locks longer
/// than necessary.
struct PendingTfReg {
    eid: EntityId,
    frame: String,
    pose_local: Mat4,
}

/// Process a batch of markers under a single scene write lock and a single
/// topic-state lock. Markers may arrive at high rate (e.g. nav plans at 30 Hz
/// with hundreds of markers); per-message locking would thrash against the
/// renderer's read lock.
fn process_marker_batch(
    markers: &[Marker],
    topic: &str,
    reference_frame: &str,
    tf: &TfTree,
    tf_refresh: &TfRegistry,
    scene: &SceneHandle,
    state: &Arc<Mutex<TopicState>>,
) {
    if markers.is_empty() {
        return;
    }

    // Build everything we need under the state lock first (allocations, hash
    // checks, primitive construction). Then acquire the scene write lock once
    // to apply removes + upserts. TF registration is deferred to after both
    // locks release.
    let mut to_remove: Vec<EntityId> = Vec::new();
    let mut to_upsert: Vec<SceneEntity> = Vec::new();
    let mut to_register: Vec<PendingTfReg> = Vec::new();
    let mut to_unregister: Vec<EntityId> = Vec::new();

    {
        let mut s = state.lock();
        for m in markers {
            match m.action {
                A_DELETE => {
                    if let Some(eid) = s.remove(&m.ns, m.id) {
                        to_remove.push(eid);
                        to_unregister.push(eid);
                    }
                    continue;
                }
                A_DELETEALL => {
                    let ids: Vec<EntityId> = s.entities.values().copied().collect();
                    s.entities.clear();
                    s.last_payload_hash.clear();
                    for eid in &ids {
                        to_remove.push(*eid);
                        to_unregister.push(*eid);
                    }
                    continue;
                }
                A_ADD => {} // == A_MODIFY
                other => {
                    log::warn!("{topic}: unknown marker action {other}, treating as ADD");
                }
            }

            // Skip identical re-publishes: avoids bumping entity revision and
            // re-uploading GPU buffers for content we already have. TF motion
            // is handled separately by `tf_refresh`.
            let hash = payload_hash(m);
            let key = (m.ns.clone(), m.id);
            if s.last_payload_hash.get(&key) == Some(&hash) {
                continue;
            }

            let primitive = match build_primitive_locked(m, topic, &mut s) {
                Some(p) => p,
                None => continue,
            };

            let eid = match s.allocate(&m.ns, m.id) {
                Some(e) => e,
                None => {
                    log::warn!(
                        "{topic}: marker slab full (>{} unique (ns,id) keys); dropping ns={} id={}",
                        ROS_ID_MARKER_PER_TOPIC,
                        m.ns,
                        m.id
                    );
                    continue;
                }
            };

            let frame = &m.header.frame_id;
            let pose_local = pose_to_mat4(&m.pose);
            let tf_ref_from_frame = if frame == reference_frame {
                Mat4::IDENTITY
            } else {
                match tf.lookup(reference_frame, frame) {
                    Some(x) => x,
                    None => {
                        log::warn!(
                            "{topic}: tf lookup {frame} -> {reference_frame} unavailable; rendering marker ns={} id={} at frame origin",
                            m.ns,
                            m.id
                        );
                        Mat4::IDENTITY
                    }
                }
            };
            let transform = ROS_TO_WORLD * tf_ref_from_frame * pose_local;

            let label = if m.ns.is_empty() {
                format!("{topic} [{}]", m.id)
            } else {
                format!("{topic} [{}/{}]", m.ns, m.id)
            };
            let entity = SceneEntity::new(eid, primitive)
                .with_transform(transform)
                .with_label(label);
            to_upsert.push(entity);
            to_register.push(PendingTfReg {
                eid,
                frame: frame.clone(),
                pose_local,
            });
            s.last_payload_hash.insert(key, hash);
        }
    }

    if !to_remove.is_empty() || !to_upsert.is_empty() {
        let mut sw = scene.write();
        for eid in &to_remove {
            sw.remove(*eid);
        }
        for entity in to_upsert {
            sw.upsert(entity);
        }
    }

    for eid in to_unregister {
        tf_refresh.unregister(eid);
    }
    for reg in to_register {
        tf_refresh.register(reg.eid, reg.frame.as_str(), reg.pose_local);
    }
}

/// Construct the `ScenePrimitive` for a marker, returning `None` if the type
/// is unsupported (with a one-time warning per topic+type). Takes the already-
/// held topic-state lock so we don't drop and re-acquire it mid-batch.
fn build_primitive_locked(m: &Marker, topic: &str, state: &mut TopicState) -> Option<ScenePrimitive> {
    let color = ros_color(&m.color);
    let sx = m.scale.x as f32;
    let sy = m.scale.y as f32;
    let sz = m.scale.z as f32;

    match m.type_ {
        T_ARROW => Some(build_arrow(m, color, sx, sy, sz)),
        T_CUBE => Some(mesh_primitive(box_mesh(sx.max(1e-4), sy.max(1e-4), sz.max(1e-4)), color)),
        T_SPHERE => Some(mesh_primitive(
            sphere_uniform(sx, sy, sz, 16, 16),
            color,
        )),
        T_CYLINDER => {
            // RViz: x = diameter (x and y are equal), z = height.
            let radius = sx.max(sy).max(1e-4) * 0.5;
            let height = sz.max(1e-4);
            Some(mesh_primitive(cylinder_mesh(radius, height, 24), color))
        }
        T_LINE_STRIP => {
            let pts: Vec<Vec3> = m
                .points
                .iter()
                .map(|p| Vec3::new(p.x as f32, p.y as f32, p.z as f32))
                .collect();
            Some(ScenePrimitive::Polyline(Polyline {
                points: pts,
                color,
                width: sx.max(0.005),
            }))
        }
        T_POINTS => {
            let size = (sx.max(sy)).max(1.0);
            let pts: Vec<Point> = m
                .points
                .iter()
                .enumerate()
                .map(|(i, p)| Point {
                    position: Vec3::new(p.x as f32, p.y as f32, p.z as f32),
                    color: per_point_color(m, i, color),
                    size,
                })
                .collect();
            Some(ScenePrimitive::Points(pts))
        }
        T_SPHERE_LIST | T_CUBE_LIST => {
            // Render as points (each entry's per-point color preserved). Faithful
            // shape rendering would need many sub-meshes; that's M1+ work.
            let size = (sx.max(sy)).max(0.05) * 40.0;
            let pts: Vec<Point> = m
                .points
                .iter()
                .enumerate()
                .map(|(i, p)| Point {
                    position: Vec3::new(p.x as f32, p.y as f32, p.z as f32),
                    color: per_point_color(m, i, color),
                    size,
                })
                .collect();
            Some(ScenePrimitive::Points(pts))
        }
        T_TEXT_VIEW_FACING => {
            // Text is placed at the marker pose; scale.z is character height (m).
            Some(ScenePrimitive::Labels(vec![Label {
                position: Vec3::ZERO,
                text: m.text.clone(),
                color,
                scale: sz.max(0.05),
            }]))
        }
        T_TRIANGLE_LIST => {
            // Build a Mesh straight from `points` taken in triples.
            if m.points.len() < 3 {
                return None;
            }
            let mut verts: Vec<Vertex> = Vec::with_capacity(m.points.len());
            let mut idx: Vec<u32> = Vec::with_capacity(m.points.len());
            for tri in m.points.chunks_exact(3) {
                let p0 = Vec3::new(tri[0].x as f32, tri[0].y as f32, tri[0].z as f32);
                let p1 = Vec3::new(tri[1].x as f32, tri[1].y as f32, tri[1].z as f32);
                let p2 = Vec3::new(tri[2].x as f32, tri[2].y as f32, tri[2].z as f32);
                let n = (p1 - p0).cross(p2 - p0).normalize_or_zero();
                let base = verts.len() as u32;
                for p in &[p0, p1, p2] {
                    verts.push(Vertex {
                        position: [p.x, p.y, p.z],
                        normal: [n.x, n.y, n.z],
                        uv: [0.0, 0.0],
                    });
                }
                idx.extend_from_slice(&[base, base + 1, base + 2]);
            }
            Some(ScenePrimitive::Mesh(Mesh {
                vertices: verts,
                indices: idx,
                material: Material {
                    base_color: color,
                    texture: None,
                    wireframe: false,
                },
            }))
        }
        other => {
            if state.warned_unsupported.insert(other) {
                let name = match other {
                    T_LINE_LIST => "LINE_LIST",
                    T_MESH_RESOURCE => "MESH_RESOURCE",
                    12 => "ARROW_STRIP",
                    _ => "UNKNOWN",
                };
                log::warn!(
                    "{topic}: marker type {other} ({name}) not supported in M0.5 — skipping subsequent {name} markers on this topic"
                );
            }
            None
        }
    }
}

fn build_arrow(m: &Marker, color: Color, sx: f32, sy: f32, sz: f32) -> ScenePrimitive {
    // Two-point form: m.points[0] = start, m.points[1] = end.
    // (Both points are expressed in the marker's frame; `pose` is then applied
    // on top via the entity transform — same as RViz.)
    if m.points.len() >= 2 {
        let a = Vec3::new(m.points[0].x as f32, m.points[0].y as f32, m.points[0].z as f32);
        let b = Vec3::new(m.points[1].x as f32, m.points[1].y as f32, m.points[1].z as f32);
        let dir = (b - a).try_normalize().unwrap_or(Vec3::X);
        let length = (b - a).length().max(1e-4);
        return ScenePrimitive::Arrows(vec![Arrow {
            origin: a,
            direction: dir,
            length,
            shaft_radius: (sx * 0.5).max(0.005),
            head_radius: (sy * 0.5).max(0.01),
            color,
        }]);
    }
    // Position+scale form: arrow lies along the marker's local +X by default;
    // m.pose orients it and we transform the whole entity through that pose.
    ScenePrimitive::Arrows(vec![Arrow {
        origin: Vec3::ZERO,
        direction: Vec3::X,
        length: sx.max(1e-4),
        shaft_radius: (sy * 0.5).max(0.005),
        head_radius: (sz * 0.5).max(0.01),
        color,
    }])
}

fn mesh_primitive(geom: (Vec<Vertex>, Vec<u32>), color: Color) -> ScenePrimitive {
    let (vertices, indices) = geom;
    ScenePrimitive::Mesh(Mesh {
        vertices,
        indices,
        material: Material {
            base_color: color,
            texture: None,
            wireframe: false,
        },
    })
}

/// Sphere with possibly non-uniform diameters — we generate a unit sphere and
/// pre-scale vertices in place. Cheaper than a one-off matrix on every draw
/// and avoids leaking a non-rigid transform into the TF-refresh path.
fn sphere_uniform(sx: f32, sy: f32, sz: f32, rings: u32, segments: u32) -> (Vec<Vertex>, Vec<u32>) {
    let (mut verts, idx) = sphere_mesh(1.0, rings, segments);
    let rx = (sx * 0.5).max(1e-4);
    let ry = (sy * 0.5).max(1e-4);
    let rz = (sz * 0.5).max(1e-4);
    for v in verts.iter_mut() {
        v.position[0] *= rx;
        v.position[1] *= ry;
        v.position[2] *= rz;
        // Re-normalize for axis stretch (good enough — meshpass tolerates non-unit normals).
        let inv = [1.0 / rx, 1.0 / ry, 1.0 / rz];
        v.normal[0] *= inv[0];
        v.normal[1] *= inv[1];
        v.normal[2] *= inv[2];
        let len = (v.normal[0] * v.normal[0] + v.normal[1] * v.normal[1] + v.normal[2] * v.normal[2]).sqrt();
        if len > 1e-6 {
            v.normal[0] /= len;
            v.normal[1] /= len;
            v.normal[2] /= len;
        }
    }
    (verts, idx)
}

fn per_point_color(m: &Marker, i: usize, fallback: Color) -> Color {
    if i < m.colors.len() {
        ros_color(&m.colors[i])
    } else {
        fallback
    }
}

fn ros_color(c: &r2r::std_msgs::msg::ColorRGBA) -> Color {
    Color::rgba(c.r, c.g, c.b, if c.a <= 0.0 { 1.0 } else { c.a })
}

fn pose_to_mat4(p: &r2r::geometry_msgs::msg::Pose) -> Mat4 {
    let q = &p.orientation;
    // ROS pose default has all zeros, including quaternion. Treat zero-quat as identity.
    let qx = q.x as f32;
    let qy = q.y as f32;
    let qz = q.z as f32;
    let qw = q.w as f32;
    let norm2 = qx * qx + qy * qy + qz * qz + qw * qw;
    let rot = if norm2 < 1e-6 {
        Quat::IDENTITY
    } else {
        Quat::from_xyzw(qx, qy, qz, qw).normalize()
    };
    let t = Vec3::new(p.position.x as f32, p.position.y as f32, p.position.z as f32);
    Mat4::from_rotation_translation(rot, t)
}
