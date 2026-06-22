//! TF-frame visualization registry.
//!
//! Snapshots the live [`TfTree`] on every tick and upserts one
//! `ScenePrimitive::Frame` entity per known frame, positioned at
//! `reference_frame -> frame`. This is what gives the user the RViz-style "TF"
//! display: per-frame R/G/B axes, listed in the side panel with a checkbox per
//! frame so individual frames can be toggled on or off.
//!
//! Frames are visible by default. The `SceneGraph::upsert` contract preserves
//! visibility across re-publishes, so once the user hides a frame it stays
//! hidden as long as the entity lives.

use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;

use glam::Mat4;
use parking_lot::Mutex;
use scene::{EntityId, Frame, SceneEntity, SceneHandle, ScenePrimitive};

use crate::coords::ROS_TO_WORLD;
use crate::ids::TF_FRAME_BASE;
use crate::tf::TfTree;

/// Default axis arm length (meters, ROS scale). Visible at typical indoor zoom
/// levels; the UI can scale this up at runtime via the shared scale handle.
pub const DEFAULT_AXIS_LENGTH_M: f32 = 0.3;

pub struct TfAxesRegistry {
    inner: Mutex<Inner>,
    /// Shared, runtime-mutable axis length (meters). Stored as `f32::to_bits`
    /// in an `AtomicU32` so the UI thread can update it lock-free.
    axis_length: Arc<AtomicU32>,
}

#[derive(Default)]
struct Inner {
    /// Stable EntityId per frame name, allocated in encounter order. Persisted
    /// across ticks so toggle state and revisions remain coherent.
    ids: HashMap<String, EntityId>,
    next_offset: u64,
    /// `axis_length` value applied on the previous refresh — used to decide
    /// when we need to rewrite the Frame primitive (vs. only its transform).
    last_applied: f32,
}

impl TfAxesRegistry {
    pub fn new() -> Arc<Self> {
        Self::with_scale(Arc::new(AtomicU32::new(
            DEFAULT_AXIS_LENGTH_M.to_bits(),
        )))
    }

    pub fn with_scale(axis_length: Arc<AtomicU32>) -> Arc<Self> {
        Arc::new(Self {
            inner: Mutex::new(Inner::default()),
            axis_length,
        })
    }

    /// Shared handle to the axis length so the UI thread can mutate it.
    pub fn axis_length_handle(&self) -> Arc<AtomicU32> {
        self.axis_length.clone()
    }

    fn current_length(&self) -> f32 {
        f32::from_bits(self.axis_length.load(Ordering::Relaxed)).max(1e-4)
    }

    /// Walk the current TF tree and materialise one Frame entity per frame.
    /// Cheap on steady-state: only the transform changes after the first tick
    /// for an existing frame, so this is just a `update_transform` per frame.
    pub fn refresh(&self, tf: &TfTree, reference_frame: &str, scene: &SceneHandle) {
        // Collect every name that appears anywhere in the tree, including
        // parents that are themselves not children of anything (e.g. a root
        // frame published only as the parent in /tf_static).
        let names: Vec<String> = {
            let frames = tf.frames.read();
            let mut set: HashSet<String> = HashSet::with_capacity(frames.len() * 2);
            for (child, entry) in frames.iter() {
                set.insert(child.clone());
                set.insert(entry.parent.clone());
            }
            // The reference frame may not appear in /tf at all (nothing
            // published *relative to it*), but the user still expects to see
            // its axes at world origin.
            set.insert(reference_frame.to_string());
            set.into_iter().collect()
        };

        let axis_length = self.current_length();
        let mut inner = self.inner.lock();
        // When the user changes the scale, every existing Frame's primitive
        // needs to be rewritten so the line pass picks up the new length.
        let scale_changed = (inner.last_applied - axis_length).abs() > 1e-6;
        inner.last_applied = axis_length;
        let mut scene_w = scene.write();
        for name in names {
            let world = if name == reference_frame {
                ROS_TO_WORLD
            } else {
                match tf.lookup(reference_frame, &name) {
                    Some(m) => ROS_TO_WORLD * m,
                    None => continue,
                }
            };

            let (id, is_new) = match inner.ids.get(&name) {
                Some(&id) => (id, false),
                None => {
                    let id = EntityId(TF_FRAME_BASE + inner.next_offset);
                    inner.next_offset += 1;
                    inner.ids.insert(name.clone(), id);
                    (id, true)
                }
            };

            if !is_new && scene_w.entities.contains_key(&id) {
                scene_w.update_transform(id, world);
                if scale_changed {
                    scene_w.update_primitive(
                        id,
                        ScenePrimitive::Frame(Frame {
                            transform: Mat4::IDENTITY,
                            axis_length,
                            label: Some(name.clone()),
                        }),
                    );
                }
                continue;
            }

            let mut entity = SceneEntity::new(
                id,
                ScenePrimitive::Frame(Frame {
                    transform: Mat4::IDENTITY,
                    axis_length,
                    label: Some(name.clone()),
                }),
            );
            entity.label = Some(format!("tf: {name}"));
            entity.transform = world;
            scene_w.upsert(entity);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tf::TransformEntry;
    use glam::Vec3;
    use parking_lot::RwLock;
    use scene::SceneGraph;

    fn translation_entry(parent: &str, t: Vec3) -> TransformEntry {
        TransformEntry::from_sample(
            parent.into(),
            crate::tf::TfSample {
                stamp_ns: 0,
                rot: glam::Quat::IDENTITY,
                trans: t,
            },
        )
    }

    #[test]
    fn refresh_materialises_one_frame_entity_per_known_frame() {
        let scene: SceneHandle = Arc::new(RwLock::new(SceneGraph::new("map")));
        let tf = TfTree::new();
        {
            let mut f = tf.frames.write();
            f.insert("odom".into(), translation_entry("map", Vec3::new(1.0, 0.0, 0.0)));
            f.insert("base".into(), translation_entry("odom", Vec3::new(0.0, 2.0, 0.0)));
        }
        let reg = TfAxesRegistry::new();
        reg.refresh(&tf, "map", &scene);

        // map (reference) + odom + base = 3 frame entities, all in TF range.
        let s = scene.read();
        let frame_entities: Vec<_> = s
            .entities
            .values()
            .filter(|e| matches!(e.primitive, ScenePrimitive::Frame(_)))
            .collect();
        assert_eq!(frame_entities.len(), 3);
        for e in frame_entities {
            assert!(e.id.0 >= TF_FRAME_BASE);
            assert!(e.visible);
            assert!(e.label.as_deref().unwrap().starts_with("tf: "));
        }
    }

    #[test]
    fn refresh_preserves_user_visibility_toggle() {
        let scene: SceneHandle = Arc::new(RwLock::new(SceneGraph::new("map")));
        let tf = TfTree::new();
        {
            let mut f = tf.frames.write();
            f.insert("odom".into(), translation_entry("map", Vec3::new(1.0, 0.0, 0.0)));
        }
        let reg = TfAxesRegistry::new();
        reg.refresh(&tf, "map", &scene);

        // Find the odom frame entity and hide it.
        let odom_id = {
            let s = scene.read();
            s.entities
                .values()
                .find(|e| e.label.as_deref() == Some("tf: odom"))
                .map(|e| e.id)
                .unwrap()
        };
        scene.write().set_visible(odom_id, false);

        // Move odom and refresh again; visibility must NOT flip back to true.
        {
            let mut f = tf.frames.write();
            f.insert("odom".into(), translation_entry("map", Vec3::new(5.0, 0.0, 0.0)));
        }
        reg.refresh(&tf, "map", &scene);
        assert!(!scene.read().entities[&odom_id].visible);
    }

    #[test]
    fn refresh_reuses_entity_id_for_known_frame() {
        let scene: SceneHandle = Arc::new(RwLock::new(SceneGraph::new("map")));
        let tf = TfTree::new();
        {
            let mut f = tf.frames.write();
            f.insert("odom".into(), translation_entry("map", Vec3::new(1.0, 0.0, 0.0)));
        }
        let reg = TfAxesRegistry::new();
        reg.refresh(&tf, "map", &scene);
        let count_after_first = scene.read().entities.len();
        reg.refresh(&tf, "map", &scene);
        assert_eq!(scene.read().entities.len(), count_after_first);
    }

    #[test]
    fn updating_scale_rewrites_frame_axis_length() {
        let scene: SceneHandle = Arc::new(RwLock::new(SceneGraph::new("map")));
        let tf = TfTree::new();
        {
            let mut f = tf.frames.write();
            f.insert("odom".into(), translation_entry("map", Vec3::new(1.0, 0.0, 0.0)));
        }
        let reg = TfAxesRegistry::new();
        reg.refresh(&tf, "map", &scene);
        let handle = reg.axis_length_handle();
        handle.store(1.5_f32.to_bits(), Ordering::Relaxed);
        reg.refresh(&tf, "map", &scene);
        let s = scene.read();
        for e in s.entities.values() {
            if let ScenePrimitive::Frame(f) = &e.primitive {
                assert!((f.axis_length - 1.5).abs() < 1e-5);
            }
        }
    }
}
