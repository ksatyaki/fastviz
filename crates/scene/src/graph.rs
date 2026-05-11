use std::collections::HashMap;
use std::sync::Arc;

use glam::Mat4;
use parking_lot::RwLock;

use crate::primitives::{Arrow, Frame, Grid, Label, Mesh, Point, Polyline};

/// Stable identifier for a scene entity. Owned by the ingestion-layer caller
/// (a ROS subscriber, mock injector, etc.) so it can update its entity in
/// place rather than re-inserting on every tick.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct EntityId(pub u64);

/// Discriminated union of every primitive payload a scene entity can hold.
#[derive(Clone, Debug)]
pub enum ScenePrimitive {
    Points(Vec<Point>),
    Polyline(Polyline),
    Arrows(Vec<Arrow>),
    Grid(Grid),
    Mesh(Mesh),
    Labels(Vec<Label>),
    Frame(Frame),
}

#[derive(Clone, Debug)]
pub struct SceneEntity {
    pub id: EntityId,
    pub label: Option<String>,
    /// World-space transform applied to the primitive's local coordinates.
    pub transform: Mat4,
    pub primitive: ScenePrimitive,
    pub visible: bool,
    /// Set by the ingestion layer when the primitive payload changes.
    /// Cleared by the renderer after the GPU buffer is updated.
    pub dirty: bool,
    /// Bumped on every payload change (upsert / update_primitive). The renderer
    /// caches `last_uploaded_revision` per entity and only re-uploads vertex /
    /// index buffers when this counter advances — so transform-only updates
    /// (e.g. URDF joint motion, TF refreshes) don't re-stream geometry that
    /// hasn't actually changed.
    pub revision: u64,
}

impl SceneEntity {
    pub fn new(id: EntityId, primitive: ScenePrimitive) -> Self {
        SceneEntity {
            id,
            label: None,
            transform: Mat4::IDENTITY,
            primitive,
            visible: true,
            dirty: true,
            revision: 1,
        }
    }

    pub fn with_label(mut self, label: impl Into<String>) -> Self {
        self.label = Some(label.into());
        self
    }

    pub fn with_transform(mut self, transform: Mat4) -> Self {
        self.transform = transform;
        self
    }
}

#[derive(Debug)]
pub struct SceneGraph {
    pub entities: HashMap<EntityId, SceneEntity>,
    pub reference_frame: String,
    /// Monotonic counter bumped on every mutation. Render passes cache their
    /// last-seen value so they can skip rebuild + upload when the scene is
    /// unchanged (i.e. the renderer is redrawing faster than producers are
    /// publishing).
    revision: u64,
}

impl Default for SceneGraph {
    fn default() -> Self {
        SceneGraph::new("map")
    }
}

impl SceneGraph {
    pub fn new(reference_frame: impl Into<String>) -> Self {
        SceneGraph {
            entities: HashMap::new(),
            reference_frame: reference_frame.into(),
            revision: 0,
        }
    }

    /// Latest mutation counter — see [`SceneGraph::revision`] doc on the field.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    /// Insert or replace an entity, marking it dirty. Visibility is preserved
    /// across replacement so a user toggle in the UI isn't clobbered by the
    /// next publish from the ingestion layer.
    pub fn upsert(&mut self, mut entity: SceneEntity) {
        entity.dirty = true;
        if let Some(existing) = self.entities.get(&entity.id) {
            entity.visible = existing.visible;
            // Monotonic per-entity revision across re-publishes, so consumers
            // (e.g. MeshPass) can detect "this is new payload data".
            entity.revision = existing.revision.wrapping_add(1);
        }
        self.entities.insert(entity.id, entity);
        self.revision += 1;
    }

    /// Update an entity's primitive payload in place. Marks dirty.
    pub fn update_primitive(&mut self, id: EntityId, primitive: ScenePrimitive) {
        if let Some(e) = self.entities.get_mut(&id) {
            e.primitive = primitive;
            e.dirty = true;
            e.revision = e.revision.wrapping_add(1);
            self.revision += 1;
        }
    }

    /// Update the transform. Bumps revision so the renderer picks up the new
    /// value; the per-entity primitive payload doesn't change so the data
    /// upload itself is unaffected.
    pub fn update_transform(&mut self, id: EntityId, transform: Mat4) {
        if let Some(e) = self.entities.get_mut(&id) {
            e.transform = transform;
            self.revision += 1;
        }
    }

    pub fn set_visible(&mut self, id: EntityId, visible: bool) {
        if let Some(e) = self.entities.get_mut(&id) {
            e.visible = visible;
            self.revision += 1;
        }
    }

    pub fn remove(&mut self, id: EntityId) -> Option<SceneEntity> {
        let removed = self.entities.remove(&id);
        if removed.is_some() {
            self.revision += 1;
        }
        removed
    }
}

/// Shared, lock-protected handle to the scene graph.
///
/// Writers (ingestion / mock injector) take a write lock briefly to mutate.
/// The render thread takes a read lock once per frame to walk entities.
pub type SceneHandle = Arc<RwLock<SceneGraph>>;

pub fn new_handle() -> SceneHandle {
    Arc::new(RwLock::new(SceneGraph::default()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::color::Color;
    use crate::primitives::{Point, Polyline};
    use glam::Vec3;

    #[test]
    fn upsert_marks_dirty() {
        let mut g = SceneGraph::default();
        g.upsert(SceneEntity::new(
            EntityId(1),
            ScenePrimitive::Points(vec![Point {
                position: Vec3::ZERO,
                color: Color::WHITE,
                size: 1.0,
            }]),
        ));
        assert!(g.entities[&EntityId(1)].dirty);
    }

    #[test]
    fn update_primitive_in_place_keeps_id() {
        let mut g = SceneGraph::default();
        g.upsert(SceneEntity::new(
            EntityId(7),
            ScenePrimitive::Polyline(Polyline {
                points: vec![Vec3::ZERO, Vec3::X],
                color: Color::RED,
                width: 1.0,
            }),
        ));
        g.entities.get_mut(&EntityId(7)).unwrap().dirty = false;

        g.update_primitive(
            EntityId(7),
            ScenePrimitive::Polyline(Polyline {
                points: vec![Vec3::Y, Vec3::Z],
                color: Color::GREEN,
                width: 2.0,
            }),
        );
        let e = &g.entities[&EntityId(7)];
        assert!(e.dirty);
        assert_eq!(g.entities.len(), 1);
    }

    #[test]
    fn update_transform_does_not_mark_dirty() {
        let mut g = SceneGraph::default();
        g.upsert(SceneEntity::new(
            EntityId(2),
            ScenePrimitive::Points(vec![]),
        ));
        g.entities.get_mut(&EntityId(2)).unwrap().dirty = false;
        g.update_transform(EntityId(2), Mat4::from_translation(Vec3::X));
        assert!(!g.entities[&EntityId(2)].dirty);
    }

    #[test]
    fn upsert_preserves_visibility_across_republish() {
        let mut g = SceneGraph::default();
        g.upsert(SceneEntity::new(
            EntityId(9),
            ScenePrimitive::Points(vec![]),
        ));
        g.set_visible(EntityId(9), false);

        // Simulate the ingestion layer publishing again with a fresh entity.
        g.upsert(SceneEntity::new(
            EntityId(9),
            ScenePrimitive::Points(vec![]),
        ));
        assert!(!g.entities[&EntityId(9)].visible);
    }

    #[test]
    fn revision_advances_only_on_payload_change() {
        // The MeshPass relies on this: transform-only updates (joint motion /
        // TF refresh) must NOT bump entity.revision, otherwise URDF geometry
        // gets re-uploaded to the GPU every frame.
        let mut g = SceneGraph::default();
        g.upsert(SceneEntity::new(EntityId(11), ScenePrimitive::Points(vec![])));
        let rev0 = g.entities[&EntityId(11)].revision;

        g.update_transform(EntityId(11), Mat4::from_translation(Vec3::X));
        assert_eq!(g.entities[&EntityId(11)].revision, rev0, "transform update bumped revision");

        g.update_primitive(EntityId(11), ScenePrimitive::Points(vec![]));
        assert!(g.entities[&EntityId(11)].revision > rev0, "update_primitive should bump");

        let rev1 = g.entities[&EntityId(11)].revision;
        g.upsert(SceneEntity::new(EntityId(11), ScenePrimitive::Points(vec![])));
        assert!(g.entities[&EntityId(11)].revision > rev1, "re-upsert should bump");
    }

    #[test]
    fn remove_returns_entity() {
        let mut g = SceneGraph::default();
        g.upsert(SceneEntity::new(
            EntityId(3),
            ScenePrimitive::Points(vec![]),
        ));
        assert!(g.remove(EntityId(3)).is_some());
        assert!(g.entities.is_empty());
    }
}
