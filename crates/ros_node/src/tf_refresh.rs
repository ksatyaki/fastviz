//! TF-aware entity registry.
//!
//! Subscribers compute their entity's world transform at message-receive time
//! by looking up the current TF tree. If TF is incomplete at that moment they
//! fall back to `Mat4::IDENTITY` — and without this registry the entity would
//! stay misplaced until the next message on that topic ever arrives (a 100 ms
//! glitch for a 10 Hz laser, *forever* for a one-shot rosbag pose).
//!
//! Each TF-dependent subscriber registers `(entity_id, frame_id, local)` here,
//! where `local` is the entity's pose **inside its own ROS frame**. The main
//! spin loop calls [`TfRegistry::refresh`] every tick; it recomputes
//! `world = ROS_TO_WORLD * ref_T_frame * local` and pushes it into the scene.
//! So as soon as the missing TF link arrives, the next refresh fixes every
//! stale entity without needing a fresh sensor message.

use std::collections::HashMap;
use std::sync::Arc;

use glam::Mat4;
use parking_lot::Mutex;
use scene::{EntityId, SceneHandle};

use crate::coords::ROS_TO_WORLD;
use crate::tf::TfTree;

#[derive(Clone, Debug)]
struct Entry {
    frame: String,
    /// Entity pose in `frame`'s coordinates (ROS, right-handed Z-up).
    /// For sensor points already expressed in `frame`, this is `IDENTITY`.
    /// For URDF visuals expressed in the root link's frame, this is the
    /// link's `root_T_link * visual_origin`.
    local: Mat4,
}

#[derive(Default)]
pub struct TfRegistry {
    entries: Mutex<HashMap<EntityId, Entry>>,
}

impl TfRegistry {
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    /// Insert or replace the TF binding for `id`. Subsequent [`refresh`] calls
    /// will recompute the world transform from `frame` + `local` and the
    /// current TF tree.
    pub fn register(&self, id: EntityId, frame: impl Into<String>, local: Mat4) {
        self.entries.lock().insert(
            id,
            Entry {
                frame: frame.into(),
                local,
            },
        );
    }

    /// Forget an entity (e.g., when its publisher disappears). Idempotent.
    pub fn unregister(&self, id: EntityId) {
        self.entries.lock().remove(&id);
    }

    /// Recompute world transforms for every registered entity and push them
    /// into the scene graph. Entities whose TF lookup still fails keep their
    /// previous transform — we don't want to flicker back to IDENTITY just
    /// because a transient /tf message hasn't arrived this tick.
    pub fn refresh(&self, tf: &TfTree, reference_frame: &str, scene: &SceneHandle) {
        let entries = self.entries.lock();
        if entries.is_empty() {
            return;
        }
        let mut scene_w = scene.write();
        for (id, entry) in entries.iter() {
            let tf_ref_from_frame = if entry.frame == reference_frame {
                Mat4::IDENTITY
            } else {
                match tf.lookup(reference_frame, &entry.frame) {
                    Some(m) => m,
                    None => continue,
                }
            };
            let world = ROS_TO_WORLD * tf_ref_from_frame * entry.local;
            scene_w.update_transform(*id, world);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tf::TransformEntry;
    use glam::Vec3;
    use parking_lot::RwLock;
    use scene::{Point, SceneEntity, SceneGraph, ScenePrimitive};
    use std::sync::Arc;

    fn entity_with_id(id: u64) -> SceneEntity {
        SceneEntity::new(
            EntityId(id),
            ScenePrimitive::Points(vec![Point {
                position: Vec3::ZERO,
                color: scene::Color::WHITE,
                size: 1.0,
            }]),
        )
    }

    fn translation_entry(parent: &str, t: Vec3) -> TransformEntry {
        TransformEntry {
            parent: parent.into(),
            xform: Mat4::from_translation(t),
            stamp_ns: 0,
        }
    }

    #[test]
    fn refresh_updates_transform_once_tf_arrives() {
        let scene: SceneHandle = Arc::new(RwLock::new(SceneGraph::new("odom")));
        scene.write().upsert(entity_with_id(42));

        let tf = TfTree::new();
        let reg = TfRegistry::default();
        reg.register(EntityId(42), "laser", Mat4::IDENTITY);

        // No TF yet → refresh leaves the entity's initial IDENTITY in place.
        reg.refresh(&tf, "odom", &scene);
        assert_eq!(scene.read().entities[&EntityId(42)].transform, Mat4::IDENTITY);

        // TF arrives: odom → laser with +1m on ROS x.
        tf.frames
            .write()
            .insert("laser".into(), translation_entry("odom", Vec3::new(1.0, 0.0, 0.0)));

        reg.refresh(&tf, "odom", &scene);
        let t = scene.read().entities[&EntityId(42)].transform;
        // ROS_TO_WORLD maps ROS x→world x, so origin in laser frame ends up at (1,0,0) in world.
        let p = t.transform_point3(Vec3::ZERO);
        assert!((p - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-5);
    }

    #[test]
    fn refresh_keeps_prior_transform_when_lookup_fails() {
        let scene: SceneHandle = Arc::new(RwLock::new(SceneGraph::new("odom")));
        scene.write().upsert(entity_with_id(7));
        let placeholder = Mat4::from_translation(Vec3::new(5.0, 0.0, 0.0));
        scene.write().update_transform(EntityId(7), placeholder);

        let tf = TfTree::new();
        let reg = TfRegistry::default();
        reg.register(EntityId(7), "disconnected_frame", Mat4::IDENTITY);

        // Lookup fails — refresh must NOT clobber the existing transform with IDENTITY.
        reg.refresh(&tf, "odom", &scene);
        assert_eq!(scene.read().entities[&EntityId(7)].transform, placeholder);
    }
}
