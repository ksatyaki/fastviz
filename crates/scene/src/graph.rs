use std::collections::HashMap;
use std::sync::Arc;

use glam::Mat4;
use parking_lot::RwLock;

use crate::color::Color;
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

/// Per-entity UI-driven style override. Survives subscriber re-publishes so a
/// user's color/scale edit isn't clobbered the next time a sensor message
/// arrives — `SceneGraph::upsert` re-applies any stored override after the
/// fresh primitive is inserted.
#[derive(Copy, Clone, Debug, Default)]
pub struct StyleOverride {
    pub color: Option<Color>,
    pub scale: Option<f32>,
    /// Arrow head radius (world-space, absolute) — independent of `scale`,
    /// which scales the whole arrow proportionally. Only affects Arrows.
    pub head_scale: Option<f32>,
}

impl StyleOverride {
    /// True when nothing is overridden — used to drop the map entry entirely.
    fn is_empty(&self) -> bool {
        self.color.is_none() && self.scale.is_none() && self.head_scale.is_none()
    }
}

#[derive(Debug)]
pub struct SceneGraph {
    pub entities: HashMap<EntityId, SceneEntity>,
    pub reference_frame: String,
    /// Per-entity style overrides applied on top of subscriber-published
    /// primitives. Indexed by `EntityId`; missing entry = no override.
    pub style_overrides: HashMap<EntityId, StyleOverride>,
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
            style_overrides: HashMap::new(),
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
        if let Some(ov) = self.style_overrides.get(&entity.id).copied() {
            apply_override(&mut entity.primitive, ov);
        }
        self.entities.insert(entity.id, entity);
        self.revision += 1;
    }

    /// Update an entity's primitive payload in place. Marks dirty.
    pub fn update_primitive(&mut self, id: EntityId, primitive: ScenePrimitive) {
        if let Some(e) = self.entities.get_mut(&id) {
            e.primitive = primitive;
            if let Some(ov) = self.style_overrides.get(&id).copied() {
                apply_override(&mut e.primitive, ov);
            }
            e.dirty = true;
            e.revision = e.revision.wrapping_add(1);
            self.revision += 1;
        }
    }

    /// Set (or clear) the per-entity color override. Re-applies to the live
    /// entity so the change is visible on the next render.
    pub fn set_color_override(&mut self, id: EntityId, color: Option<Color>) {
        let ov = self.style_overrides.entry(id).or_default();
        ov.color = color;
        if ov.is_empty() {
            self.style_overrides.remove(&id);
        }
        if let Some(e) = self.entities.get_mut(&id) {
            if let Some(c) = color {
                apply_color(&mut e.primitive, c);
            }
            e.dirty = true;
            e.revision = e.revision.wrapping_add(1);
        }
        self.revision += 1;
    }

    /// Set (or clear) the per-entity scale override. Semantics depend on the
    /// primitive: point size, polyline width, arrow length (proportional),
    /// label scale. Frames/grids/meshes ignore it.
    pub fn set_scale_override(&mut self, id: EntityId, scale: Option<f32>) {
        let ov = self.style_overrides.entry(id).or_default();
        ov.scale = scale;
        if ov.is_empty() {
            self.style_overrides.remove(&id);
        }
        if let Some(e) = self.entities.get_mut(&id) {
            if let Some(s) = scale {
                apply_scale(&mut e.primitive, s);
            }
            e.dirty = true;
            e.revision = e.revision.wrapping_add(1);
        }
        self.revision += 1;
    }

    /// Set (or clear) the per-entity arrow head-size override. Absolute
    /// world-space head radius, independent of the proportional `scale`
    /// override. No-op for non-Arrow primitives.
    pub fn set_head_scale_override(&mut self, id: EntityId, head_scale: Option<f32>) {
        let ov = self.style_overrides.entry(id).or_default();
        ov.head_scale = head_scale;
        if ov.is_empty() {
            self.style_overrides.remove(&id);
        }
        if let Some(e) = self.entities.get_mut(&id) {
            if let Some(h) = head_scale {
                apply_head_scale(&mut e.primitive, h);
            }
            e.dirty = true;
            e.revision = e.revision.wrapping_add(1);
        }
        self.revision += 1;
    }

    /// Update the transform. Bumps revision so the renderer picks up the new
    /// value; the per-entity primitive payload doesn't change so the data
    /// upload itself is unaffected. No-op (and no revision bump) when the new
    /// matrix is identical to the current one — `tf_refresh` calls this every
    /// tick for every TF-bound entity, so an unconditional bump would defeat
    /// every revision-keyed cache downstream (point pass especially).
    pub fn update_transform(&mut self, id: EntityId, transform: Mat4) {
        if let Some(e) = self.entities.get_mut(&id) {
            if e.transform == transform {
                return;
            }
            e.transform = transform;
            self.revision += 1;
        }
    }

    pub fn set_visible(&mut self, id: EntityId, visible: bool) {
        if let Some(e) = self.entities.get_mut(&id) {
            if e.visible == visible {
                return;
            }
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

fn apply_override(p: &mut ScenePrimitive, ov: StyleOverride) {
    if let Some(c) = ov.color {
        apply_color(p, c);
    }
    if let Some(s) = ov.scale {
        apply_scale(p, s);
    }
    if let Some(h) = ov.head_scale {
        apply_head_scale(p, h);
    }
}

/// Apply an absolute arrow head radius. No-op for non-Arrow primitives.
pub fn apply_head_scale(p: &mut ScenePrimitive, head_radius: f32) {
    let head_radius = head_radius.max(1e-4);
    if let ScenePrimitive::Arrows(arrs) = p {
        for a in arrs.iter_mut() {
            a.head_radius = head_radius;
        }
    }
}

/// Read the arrow head radius from a primitive, if it is an Arrows primitive.
pub fn primitive_head_radius(p: &ScenePrimitive) -> Option<f32> {
    match p {
        ScenePrimitive::Arrows(arrs) => arrs.first().map(|a| a.head_radius),
        _ => None,
    }
}

/// Read the arrow shaft radius from a primitive, if it is an Arrows primitive.
pub fn primitive_shaft_radius(p: &ScenePrimitive) -> Option<f32> {
    match p {
        ScenePrimitive::Arrows(arrs) => arrs.first().map(|a| a.shaft_radius),
        _ => None,
    }
}

/// Apply a single RGB color to every part of `p` that has one. No-op for
/// primitives without a single canonical color (Frame, Grid::Cells).
pub fn apply_color(p: &mut ScenePrimitive, c: Color) {
    match p {
        ScenePrimitive::Points(pts) => {
            for pt in pts.iter_mut() {
                pt.color = c;
            }
        }
        ScenePrimitive::Polyline(pl) => pl.color = c,
        ScenePrimitive::Arrows(arrs) => {
            for a in arrs.iter_mut() {
                a.color = c;
            }
        }
        ScenePrimitive::Mesh(m) => m.material.base_color = c,
        ScenePrimitive::Labels(ls) => {
            for l in ls.iter_mut() {
                l.color = c;
            }
        }
        ScenePrimitive::Grid(g) => {
            if let crate::primitives::GridData::Uniform(_) = g.data {
                g.data = crate::primitives::GridData::Uniform(c);
            }
        }
        ScenePrimitive::Frame(_) => {
            // R/G/B axes are intentional; per-frame color isn't user-editable.
        }
    }
}

/// Apply a "scale" override. Per-primitive semantics:
/// - Points: per-point size
/// - Polyline: width
/// - Arrows: proportional scaling of length / shaft / head
/// - Frame: axis_length (only honored when no global TF size is in effect)
/// - Labels: scale
/// - Mesh / Grid: no-op
pub fn apply_scale(p: &mut ScenePrimitive, s: f32) {
    let s = s.max(1e-4);
    match p {
        ScenePrimitive::Points(pts) => {
            for pt in pts.iter_mut() {
                pt.size = s;
            }
        }
        ScenePrimitive::Polyline(pl) => pl.width = s,
        ScenePrimitive::Arrows(arrs) => {
            for a in arrs.iter_mut() {
                // Scale length and radii proportionally relative to current length.
                if a.length > 1e-6 {
                    let k = s / a.length;
                    a.length = s;
                    a.shaft_radius *= k;
                    a.head_radius *= k;
                } else {
                    a.length = s;
                }
            }
        }
        ScenePrimitive::Frame(f) => f.axis_length = s,
        ScenePrimitive::Labels(ls) => {
            for l in ls.iter_mut() {
                l.scale = s;
            }
        }
        ScenePrimitive::Mesh(_) | ScenePrimitive::Grid(_) => {}
    }
}

/// Read the canonical single color from a primitive, if it has one.
pub fn primitive_color(p: &ScenePrimitive) -> Option<Color> {
    match p {
        ScenePrimitive::Points(pts) => pts.first().map(|pt| pt.color),
        ScenePrimitive::Polyline(pl) => Some(pl.color),
        ScenePrimitive::Arrows(arrs) => arrs.first().map(|a| a.color),
        ScenePrimitive::Mesh(m) => Some(m.material.base_color),
        ScenePrimitive::Labels(ls) => ls.first().map(|l| l.color),
        ScenePrimitive::Grid(g) => match g.data {
            crate::primitives::GridData::Uniform(c) => Some(c),
            crate::primitives::GridData::Cells(..) => None,
        },
        ScenePrimitive::Frame(_) => None,
    }
}

/// Read the canonical "scale" from a primitive, if one applies.
pub fn primitive_scale(p: &ScenePrimitive) -> Option<f32> {
    match p {
        ScenePrimitive::Points(pts) => pts.first().map(|pt| pt.size),
        ScenePrimitive::Polyline(pl) => Some(pl.width),
        ScenePrimitive::Arrows(arrs) => arrs.first().map(|a| a.length),
        ScenePrimitive::Frame(f) => Some(f.axis_length),
        ScenePrimitive::Labels(ls) => ls.first().map(|l| l.scale),
        ScenePrimitive::Mesh(_) | ScenePrimitive::Grid(_) => None,
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
                strip: true,
            }),
        ));
        g.entities.get_mut(&EntityId(7)).unwrap().dirty = false;

        g.update_primitive(
            EntityId(7),
            ScenePrimitive::Polyline(Polyline {
                points: vec![Vec3::Y, Vec3::Z],
                color: Color::GREEN,
                width: 2.0,
                strip: true,
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
    fn color_override_survives_republish() {
        let mut g = SceneGraph::default();
        g.upsert(SceneEntity::new(
            EntityId(50),
            ScenePrimitive::Points(vec![Point {
                position: Vec3::ZERO,
                color: Color::WHITE,
                size: 1.0,
            }]),
        ));
        g.set_color_override(EntityId(50), Some(Color::RED));
        // Subscriber pushes a fresh primitive with the old style color.
        g.upsert(SceneEntity::new(
            EntityId(50),
            ScenePrimitive::Points(vec![Point {
                position: Vec3::X,
                color: Color::GREEN,
                size: 2.0,
            }]),
        ));
        let pts = match &g.entities[&EntityId(50)].primitive {
            ScenePrimitive::Points(p) => p.clone(),
            _ => unreachable!(),
        };
        assert_eq!(pts.first().unwrap().color, Color::RED);
    }

    #[test]
    fn scale_override_survives_republish() {
        let mut g = SceneGraph::default();
        g.upsert(SceneEntity::new(
            EntityId(51),
            ScenePrimitive::Polyline(Polyline {
                points: vec![Vec3::ZERO, Vec3::X],
                color: Color::RED,
                width: 1.0,
                strip: true,
            }),
        ));
        g.set_scale_override(EntityId(51), Some(5.0));
        g.upsert(SceneEntity::new(
            EntityId(51),
            ScenePrimitive::Polyline(Polyline {
                points: vec![Vec3::Y, Vec3::Z],
                color: Color::GREEN,
                width: 2.0,
                strip: true,
            }),
        ));
        match &g.entities[&EntityId(51)].primitive {
            ScenePrimitive::Polyline(p) => assert!((p.width - 5.0).abs() < 1e-6),
            _ => unreachable!(),
        }
    }

    #[test]
    fn head_scale_override_survives_republish() {
        use crate::primitives::Arrow;
        let mut g = SceneGraph::default();
        let arrow = || Arrow {
            origin: Vec3::ZERO,
            direction: Vec3::X,
            length: 1.0,
            shaft_radius: 0.05,
            head_radius: 0.1,
            color: Color::RED,
        };
        g.upsert(SceneEntity::new(
            EntityId(60),
            ScenePrimitive::Arrows(vec![arrow()]),
        ));
        g.set_head_scale_override(EntityId(60), Some(0.5));
        // Subscriber pushes a fresh primitive with the old head radius.
        g.upsert(SceneEntity::new(
            EntityId(60),
            ScenePrimitive::Arrows(vec![arrow()]),
        ));
        match &g.entities[&EntityId(60)].primitive {
            ScenePrimitive::Arrows(a) => {
                assert!((a[0].head_radius - 0.5).abs() < 1e-6, "head_radius={}", a[0].head_radius)
            }
            _ => unreachable!(),
        }
        // Clearing it drops the map entry.
        g.set_head_scale_override(EntityId(60), None);
        assert!(!g.style_overrides.contains_key(&EntityId(60)));
    }

    #[test]
    fn clearing_override_drops_map_entry() {
        let mut g = SceneGraph::default();
        g.upsert(SceneEntity::new(
            EntityId(52),
            ScenePrimitive::Points(vec![]),
        ));
        g.set_color_override(EntityId(52), Some(Color::BLUE));
        assert!(g.style_overrides.contains_key(&EntityId(52)));
        g.set_color_override(EntityId(52), None);
        assert!(!g.style_overrides.contains_key(&EntityId(52)));
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
