//! TF tree.
//!
//! Stores `parent_T_child` per frame and looks up `target_T_source` by walking
//! each chain to its root. Each frame keeps a short bounded history of recent
//! samples so [`TfTree::lookup_at`] can interpolate the transform at an
//! arbitrary stamp (slerp on rotation, lerp on translation) — matching RViz,
//! which renders each message at *its own* timestamp rather than snapping to
//! the latest TF. The stamp-free [`TfTree::lookup`] still returns the latest.

use std::collections::{HashMap, VecDeque};

use glam::{Mat4, Quat, Vec3};
use parking_lot::RwLock;
use r2r::geometry_msgs::msg::TransformStamped;
use r2r::tf2_msgs::msg::TFMessage;

/// Upper bound on per-frame samples kept for interpolation. At a typical 10–50
/// Hz /tf rate this covers 2–10 s of history — plenty to bracket any message
/// stamp the subscribers hand us, while keeping memory per frame trivial.
const MAX_SAMPLES: usize = 100;

/// One timestamped `parent_T_child` sample, stored decomposed so interpolation
/// is a slerp + lerp rather than a matrix decomposition per lookup.
#[derive(Clone, Copy, Debug)]
pub struct TfSample {
    pub stamp_ns: i64,
    pub rot: Quat,
    pub trans: Vec3,
}

impl TfSample {
    fn to_mat4(self) -> Mat4 {
        Mat4::from_rotation_translation(self.rot, self.trans)
    }
}

#[derive(Clone, Debug)]
pub struct TransformEntry {
    pub parent: String,
    /// `parent_T_child` for the most recent sample — takes a point in the child
    /// frame to the parent frame.
    pub xform: Mat4,
    pub stamp_ns: i64,
    /// Recent samples in ascending stamp order, bounded to [`MAX_SAMPLES`].
    /// Static transforms keep a single entry; the latest sample mirrors
    /// `xform`/`stamp_ns`.
    pub history: VecDeque<TfSample>,
}

impl TransformEntry {
    /// Build an entry from a single sample (used by tests and the first insert
    /// for a frame).
    pub fn from_sample(parent: String, sample: TfSample) -> Self {
        let mut history = VecDeque::with_capacity(1);
        history.push_back(sample);
        TransformEntry {
            parent,
            xform: sample.to_mat4(),
            stamp_ns: sample.stamp_ns,
            history,
        }
    }
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
    /// `source` to get the point in `target`. Uses the latest sample per hop.
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

    /// Like [`lookup`](Self::lookup), but evaluates every hop at `stamp_ns` by
    /// interpolating between the two bracketing samples (slerp on rotation,
    /// lerp on translation). Stamps before a frame's oldest sample or after its
    /// newest clamp to that endpoint, so a stale-but-static hop still resolves.
    /// Falls back to the latest sample for hops with a single sample.
    pub fn lookup_at(&self, target: &str, source: &str, stamp_ns: i64) -> Option<Mat4> {
        if target == source {
            return Some(Mat4::IDENTITY);
        }
        let frames = self.frames.read();
        let (root_t_source, root_s) = walk_to_root_at(&frames, source, stamp_ns)?;
        let (root_t_target, root_t) = walk_to_root_at(&frames, target, stamp_ns)?;
        if root_s != root_t {
            return None;
        }
        Some(root_t_target.inverse() * root_t_source)
    }

    pub fn frame_count(&self) -> usize {
        self.frames.read().len()
    }

    /// Walks `frame` up the parent chain and returns the name of the topmost
    /// reachable ancestor. `None` if the frame itself isn't in the tree.
    /// Used by subscribers to explain *why* a lookup failed: if the source and
    /// target reach different roots, the chain is broken there.
    pub fn root_of(&self, frame: &str) -> Option<String> {
        let frames = self.frames.read();
        if !frames.contains_key(frame) && !frames.values().any(|e| e.parent == frame) {
            return None;
        }
        if let Some((_, root)) = walk_to_root(&frames, frame) {
            Some(root)
        } else {
            Some(frame.to_string())
        }
    }

    /// One-line human-readable explanation of why `target_T_source` is unavailable.
    /// Used by laserscan/pointcloud warnings so users see *what's wrong*, not
    /// just *that* something's wrong.
    pub fn diagnose_lookup(&self, target: &str, source: &str) -> String {
        let frames = self.frames.read();
        let src_present = frames.contains_key(source) || frames.values().any(|e| e.parent == source);
        let tgt_present = frames.contains_key(target) || frames.values().any(|e| e.parent == target);
        if !src_present && !tgt_present {
            return format!(
                "neither '{source}' nor '{target}' appears in the TF tree ({} frames known)",
                frames.len()
            );
        }
        if !src_present {
            return format!("source frame '{source}' not in TF tree");
        }
        if !tgt_present {
            return format!("target frame '{target}' not in TF tree");
        }
        let src_root = walk_to_root(&frames, source)
            .map(|(_, r)| r)
            .unwrap_or_else(|| source.to_string());
        let tgt_root = walk_to_root(&frames, target)
            .map(|(_, r)| r)
            .unwrap_or_else(|| target.to_string());
        if src_root == tgt_root {
            // Shouldn't happen if lookup returned None, but handle gracefully.
            format!("chain reaches a common root '{src_root}' but lookup still failed")
        } else {
            format!(
                "chain break: '{source}' reaches root '{src_root}', '{target}' reaches root '{tgt_root}' — \
                 typically a missing dynamic /tf link or a frame-name mismatch (namespaced vs not)"
            )
        }
    }
}

fn apply(frames: &mut HashMap<String, TransformEntry>, ts: &TransformStamped) {
    let stamp_ns =
        (ts.header.stamp.sec as i64) * 1_000_000_000 + (ts.header.stamp.nanosec as i64);
    let t = &ts.transform.translation;
    let r = &ts.transform.rotation;
    let sample = TfSample {
        stamp_ns,
        rot: Quat::from_xyzw(r.x as f32, r.y as f32, r.z as f32, r.w as f32),
        trans: Vec3::new(t.x as f32, t.y as f32, t.z as f32),
    };
    match frames.get_mut(&ts.child_frame_id) {
        // Same parent: append to the rolling history.
        Some(entry) if entry.parent == ts.header.frame_id => {
            push_sample(&mut entry.history, sample);
            if let Some(latest) = entry.history.back() {
                entry.stamp_ns = latest.stamp_ns;
                entry.xform = latest.to_mat4();
            }
        }
        // New frame, or the child was re-parented — drop any stale history.
        _ => {
            frames.insert(
                ts.child_frame_id.clone(),
                TransformEntry::from_sample(ts.header.frame_id.clone(), sample),
            );
        }
    }
}

/// Insert `sample` keeping `history` ascending by stamp and bounded to
/// [`MAX_SAMPLES`]. The common case (monotonic stamps) is a `push_back`; mild
/// reordering is handled by a linear back-scan.
fn push_sample(history: &mut VecDeque<TfSample>, sample: TfSample) {
    match history.back() {
        Some(last) if sample.stamp_ns >= last.stamp_ns => history.push_back(sample),
        None => history.push_back(sample),
        Some(_) => {
            // Out-of-order arrival: find the insertion point from the back.
            let pos = history
                .iter()
                .rposition(|s| s.stamp_ns <= sample.stamp_ns)
                .map(|i| i + 1)
                .unwrap_or(0);
            history.insert(pos, sample);
        }
    }
    while history.len() > MAX_SAMPLES {
        history.pop_front();
    }
}

/// `parent_T_child` for this hop at `stamp_ns`, interpolating between the
/// bracketing samples. Clamps to the nearest endpoint outside the recorded
/// range. Returns `None` only for an empty history (never happens for a frame
/// present in the map).
fn interp_at(history: &VecDeque<TfSample>, stamp_ns: i64) -> Option<Mat4> {
    let first = *history.front()?;
    if stamp_ns <= first.stamp_ns {
        return Some(first.to_mat4());
    }
    let last = *history.back()?;
    if stamp_ns >= last.stamp_ns {
        return Some(last.to_mat4());
    }
    // Bracket: a.stamp <= stamp < b.stamp for adjacent samples a, b.
    for i in 0..history.len() - 1 {
        let a = history[i];
        let b = history[i + 1];
        if a.stamp_ns <= stamp_ns && stamp_ns < b.stamp_ns {
            let span = (b.stamp_ns - a.stamp_ns) as f32;
            let t = if span > 0.0 {
                (stamp_ns - a.stamp_ns) as f32 / span
            } else {
                0.0
            };
            let rot = a.rot.slerp(b.rot, t);
            let trans = a.trans.lerp(b.trans, t);
            return Some(Mat4::from_rotation_translation(rot, trans));
        }
    }
    Some(last.to_mat4())
}

/// Walks `frame` up the parent chain using each hop's latest sample. Returns
/// `(root_T_frame, root_name)`. `None` on a parent cycle (malformed tree).
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

/// Like [`walk_to_root`], but evaluates each hop at `stamp_ns` via [`interp_at`].
fn walk_to_root_at(
    frames: &HashMap<String, TransformEntry>,
    frame: &str,
    stamp_ns: i64,
) -> Option<(Mat4, String)> {
    let mut cur = frame.to_string();
    let mut accum = Mat4::IDENTITY;
    let mut visited = std::collections::HashSet::new();
    while let Some(entry) = frames.get(&cur) {
        if !visited.insert(cur.clone()) {
            log::warn!("tf: cycle detected at frame {cur}");
            return None;
        }
        let hop = interp_at(&entry.history, stamp_ns).unwrap_or(entry.xform);
        accum = hop * accum;
        cur = entry.parent.clone();
    }
    Some((accum, cur))
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    fn translation_entry(parent: &str, t: Vec3) -> TransformEntry {
        TransformEntry::from_sample(
            parent.into(),
            TfSample {
                stamp_ns: 0,
                rot: Quat::IDENTITY,
                trans: t,
            },
        )
    }

    fn sample(stamp_ns: i64, rot: Quat, trans: Vec3) -> TfSample {
        TfSample {
            stamp_ns,
            rot,
            trans,
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

    #[test]
    fn lookup_at_interpolates_translation_midpoint() {
        let tf = TfTree::new();
        {
            let mut f = tf.frames.write();
            let mut e = TransformEntry::from_sample(
                "map".into(),
                sample(1_000, Quat::IDENTITY, Vec3::new(0.0, 0.0, 0.0)),
            );
            push_sample(
                &mut e.history,
                sample(3_000, Quat::IDENTITY, Vec3::new(2.0, 0.0, 0.0)),
            );
            f.insert("base".into(), e);
        }
        // Halfway in time → halfway in space.
        let m = tf.lookup_at("map", "base", 2_000).unwrap();
        let p = m.transform_point3(Vec3::ZERO);
        assert!((p - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-5, "got {p:?}");
    }

    #[test]
    fn lookup_at_clamps_outside_range() {
        let tf = TfTree::new();
        {
            let mut f = tf.frames.write();
            let mut e = TransformEntry::from_sample(
                "map".into(),
                sample(1_000, Quat::IDENTITY, Vec3::new(0.0, 0.0, 0.0)),
            );
            push_sample(
                &mut e.history,
                sample(3_000, Quat::IDENTITY, Vec3::new(2.0, 0.0, 0.0)),
            );
            f.insert("base".into(), e);
        }
        // Before the first sample clamps to it; after the last clamps to it.
        let before = tf.lookup_at("map", "base", 0).unwrap();
        assert!(before.transform_point3(Vec3::ZERO).length() < 1e-5);
        let after = tf.lookup_at("map", "base", 9_999).unwrap();
        let p = after.transform_point3(Vec3::ZERO);
        assert!((p - Vec3::new(2.0, 0.0, 0.0)).length() < 1e-5);
    }

    #[test]
    fn lookup_at_slerps_rotation_midpoint() {
        let tf = TfTree::new();
        let q0 = Quat::IDENTITY;
        let q1 = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2); // 90° about Z
        {
            let mut f = tf.frames.write();
            let mut e =
                TransformEntry::from_sample("map".into(), sample(0, q0, Vec3::ZERO));
            push_sample(&mut e.history, sample(2_000, q1, Vec3::ZERO));
            f.insert("base".into(), e);
        }
        // Midpoint of slerp(0°, 90°) is 45° about Z.
        let m = tf.lookup_at("map", "base", 1_000).unwrap();
        let p = m.transform_point3(Vec3::new(1.0, 0.0, 0.0));
        let expected = Vec3::new(
            std::f32::consts::FRAC_1_SQRT_2,
            std::f32::consts::FRAC_1_SQRT_2,
            0.0,
        );
        assert!((p - expected).length() < 1e-4, "got {p:?}");
    }

    #[test]
    fn push_sample_bounds_history() {
        let mut h = VecDeque::new();
        for i in 0..(MAX_SAMPLES as i64 + 50) {
            push_sample(&mut h, sample(i, Quat::IDENTITY, Vec3::ZERO));
        }
        assert_eq!(h.len(), MAX_SAMPLES);
        // Oldest retained sample is the most recent MAX_SAMPLES.
        assert_eq!(h.front().unwrap().stamp_ns, 50);
        assert_eq!(h.back().unwrap().stamp_ns, MAX_SAMPLES as i64 + 49);
    }

    #[test]
    fn push_sample_orders_out_of_order_arrival() {
        let mut h = VecDeque::new();
        push_sample(&mut h, sample(10, Quat::IDENTITY, Vec3::ZERO));
        push_sample(&mut h, sample(30, Quat::IDENTITY, Vec3::ZERO));
        push_sample(&mut h, sample(20, Quat::IDENTITY, Vec3::ZERO));
        let stamps: Vec<i64> = h.iter().map(|s| s.stamp_ns).collect();
        assert_eq!(stamps, vec![10, 20, 30]);
    }
}
