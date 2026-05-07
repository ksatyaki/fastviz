//! TF tree.
//!
//! Stores the latest `parent_T_child` transform per frame and looks up
//! `target_T_source` by walking each chain to its root. No interpolation
//! (plan §5.2 — added only if visible jitter forces the issue).

use std::collections::HashMap;

use glam::{Mat4, Quat, Vec3};
use parking_lot::RwLock;
use r2r::geometry_msgs::msg::TransformStamped;
use r2r::tf2_msgs::msg::TFMessage;

#[derive(Clone, Debug)]
pub struct TransformEntry {
    pub parent: String,
    /// `parent_T_child` — takes a point in the child frame to the parent frame.
    pub xform: Mat4,
    pub stamp_ns: i64,
}

#[derive(Default)]
pub struct TfTree {
    pub frames: RwLock<HashMap<String, TransformEntry>>,
}

impl TfTree {
    pub fn new() -> Self {
        TfTree::default()
    }

    /// Apply every transform in a `tf2_msgs/TFMessage`.
    pub fn update(&self, msg: &TFMessage) {
        let mut frames = self.frames.write();
        for ts in &msg.transforms {
            apply(&mut frames, ts);
        }
    }

    /// Returns `target_T_source` — multiply this with a point expressed in
    /// `source` to get the point in `target`.
    pub fn lookup(&self, target: &str, source: &str) -> Option<Mat4> {
        if target == source {
            return Some(Mat4::IDENTITY);
        }
        let frames = self.frames.read();
        let (root_t_source, root_s) = walk_to_root(&frames, source)?;
        let (root_t_target, root_t) = walk_to_root(&frames, target)?;
        if root_s != root_t {
            return None;
        }
        Some(root_t_target.inverse() * root_t_source)
    }

    pub fn frame_count(&self) -> usize {
        self.frames.read().len()
    }
}

fn apply(frames: &mut HashMap<String, TransformEntry>, ts: &TransformStamped) {
    let stamp_ns =
        (ts.header.stamp.sec as i64) * 1_000_000_000 + (ts.header.stamp.nanosec as i64);
    let t = &ts.transform.translation;
    let r = &ts.transform.rotation;
    let xform = Mat4::from_rotation_translation(
        Quat::from_xyzw(r.x as f32, r.y as f32, r.z as f32, r.w as f32),
        Vec3::new(t.x as f32, t.y as f32, t.z as f32),
    );
    frames.insert(
        ts.child_frame_id.clone(),
        TransformEntry {
            parent: ts.header.frame_id.clone(),
            xform,
            stamp_ns,
        },
    );
}

/// Walks `frame` up the parent chain. Returns `(root_T_frame, root_name)`.
/// Returns `None` if a parent cycle is detected (malformed tree).
fn walk_to_root(
    frames: &HashMap<String, TransformEntry>,
    frame: &str,
) -> Option<(Mat4, String)> {
    let mut cur = frame.to_string();
    let mut accum = Mat4::IDENTITY;
    let mut visited = std::collections::HashSet::new();
    while let Some(entry) = frames.get(&cur) {
        if !visited.insert(cur.clone()) {
            log::warn!("tf: cycle detected at frame {cur}");
            return None;
        }
        accum = entry.xform * accum;
        cur = entry.parent.clone();
    }
    Some((accum, cur))
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn translation_entry(parent: &str, t: Vec3) -> TransformEntry {
        TransformEntry {
            parent: parent.into(),
            xform: Mat4::from_translation(t),
            stamp_ns: 0,
        }
    }

    #[test]
    fn lookup_identity_when_same_frame() {
        let tf = TfTree::new();
        assert_eq!(tf.lookup("a", "a"), Some(Mat4::IDENTITY));
    }

    #[test]
    fn lookup_translation_chain() {
        let tf = TfTree::new();
        // map → odom (offset +x by 1)
        // odom → base (offset +y by 2)
        // Expect: lookup(map, base) translates by (1, 2, 0).
        {
            let mut f = tf.frames.write();
            f.insert("odom".into(), translation_entry("map", Vec3::new(1.0, 0.0, 0.0)));
            f.insert("base".into(), translation_entry("odom", Vec3::new(0.0, 2.0, 0.0)));
        }
        let m = tf.lookup("map", "base").expect("path exists");
        let p = m.transform_point3(Vec3::ZERO);
        assert!((p - Vec3::new(1.0, 2.0, 0.0)).length() < 1e-5);
    }

    #[test]
    fn lookup_inverse_direction() {
        let tf = TfTree::new();
        {
            let mut f = tf.frames.write();
            f.insert("odom".into(), translation_entry("map", Vec3::new(1.0, 0.0, 0.0)));
        }
        // base point (0,0,0) in odom → (1,0,0) in map; map_T_odom = +x.
        // odom_T_map = -x.
        let m = tf.lookup("odom", "map").unwrap();
        let p = m.transform_point3(Vec3::ZERO);
        assert!((p - Vec3::new(-1.0, 0.0, 0.0)).length() < 1e-5);
    }

    #[test]
    fn lookup_disconnected_returns_none() {
        let tf = TfTree::new();
        {
            let mut f = tf.frames.write();
            f.insert("a".into(), translation_entry("root_a", Vec3::ZERO));
            f.insert("b".into(), translation_entry("root_b", Vec3::ZERO));
        }
        assert!(tf.lookup("a", "b").is_none());
    }

    #[test]
    fn update_overwrites() {
        let tf = TfTree::new();
        {
            let mut f = tf.frames.write();
            f.insert("c".into(), translation_entry("p", Vec3::new(1.0, 0.0, 0.0)));
        }
        // Pretend a TFMessage updated c to (5, 0, 0).
        {
            let mut f = tf.frames.write();
            f.insert("c".into(), translation_entry("p", Vec3::new(5.0, 0.0, 0.0)));
        }
        let m = tf.lookup("p", "c").unwrap();
        let p = m.transform_point3(Vec3::ZERO);
        assert!((p - Vec3::new(5.0, 0.0, 0.0)).length() < 1e-5);
    }
}
