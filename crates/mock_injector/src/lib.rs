//! Mock data injector for milestone 0. Populates the scene graph with one of
//! every primitive type so each render pass has something to draw without any
//! ROS2 dependency.

use std::f32::consts::TAU;
use std::time::Instant;

use glam::{Mat4, Quat, Vec2, Vec3};
use scene::{
    Arrow, Color, Colormap, EntityId, Frame, Grid, GridData, Material, Mesh, Point, Polyline,
    SceneEntity, SceneGraph, ScenePrimitive, Vertex,
};

const ID_REFERENCE_FRAME: EntityId = EntityId(1);
const ID_CHILD_FRAME: EntityId = EntityId(2);
const ID_OCCUPANCY: EntityId = EntityId(3);
const ID_FIGURE_EIGHT: EntityId = EntityId(4);
const ID_ANIMATED_ARROW: EntityId = EntityId(5);
const ID_CUBE_MESH: EntityId = EntityId(6);
const ID_HEMISPHERE: EntityId = EntityId(7);
const ID_POSE_ARRAY: EntityId = EntityId(8);

pub struct MockInjector {
    started: Instant,
    seeded: bool,
}

impl Default for MockInjector {
    fn default() -> Self {
        Self::new()
    }
}

impl MockInjector {
    pub fn new() -> Self {
        MockInjector {
            started: Instant::now(),
            seeded: false,
        }
    }

    /// Update the scene. Call once per frame from the app's main loop.
    pub fn update(&mut self, scene: &mut SceneGraph) {
        let t = self.started.elapsed().as_secs_f32();

        if !self.seeded {
            self.seed_static(scene);
            self.seeded = true;
        }

        // Animated arrow circling the origin in the XZ plane.
        let radius = 2.0;
        let pos = Vec3::new(radius * t.cos(), 0.25, radius * t.sin());
        let dir = Vec3::new(-t.sin(), 0.0, t.cos()).normalize();
        scene.upsert(SceneEntity::new(
            ID_ANIMATED_ARROW,
            ScenePrimitive::Arrows(vec![Arrow {
                origin: pos,
                direction: dir,
                length: 0.8,
                shaft_radius: 0.04,
                head_radius: 0.08,
                color: Color::rgb(1.0, 0.55, 0.10),
            }]),
        ));

        // Spinning child frame so the line pass is exercised every frame.
        let rot = Quat::from_rotation_y(t * 0.3);
        let xform = Mat4::from_rotation_translation(rot, Vec3::new(1.5, 0.0, 0.0));
        scene.upsert(
            SceneEntity::new(
                ID_CHILD_FRAME,
                ScenePrimitive::Frame(Frame {
                    transform: Mat4::IDENTITY,
                    axis_length: 0.4,
                    label: Some("child".into()),
                }),
            )
            .with_label("child_frame")
            .with_transform(xform),
        );
    }

    /// One-shot static content: reference frame, occupancy grid, figure-8
    /// path, cube mesh, hemisphere of points, dense pose array.
    fn seed_static(&self, scene: &mut SceneGraph) {
        // Reference frame at origin.
        scene.upsert(
            SceneEntity::new(
                ID_REFERENCE_FRAME,
                ScenePrimitive::Frame(Frame {
                    transform: Mat4::IDENTITY,
                    axis_length: 1.0,
                    label: Some("map".into()),
                }),
            )
            .with_label("map_frame"),
        );

        // 10x10 occupancy grid at the origin (centered on -X side).
        let cols = 10u32;
        let rows = 10u32;
        let cell = 0.5;
        let cells = pseudo_random_cells(cols, rows, 0xC0FFEE);
        scene.upsert(
            SceneEntity::new(
                ID_OCCUPANCY,
                ScenePrimitive::Grid(Grid {
                    origin: Vec2::new(-3.5, -1.0),
                    cell_size: cell,
                    cols,
                    rows,
                    data: GridData::Cells(cells, Colormap::OccupancyDefault),
                }),
            )
            .with_label("mock_occupancy"),
        );

        // Figure-8 polyline lying on the ground plane (XZ).
        let pts = (0..256)
            .map(|i| {
                let s = (i as f32 / 256.0) * TAU;
                let x = (s).sin() * 1.6;
                let z = (s * 2.0).sin() * 0.8;
                Vec3::new(x, 0.02, z)
            })
            .collect::<Vec<_>>();
        scene.upsert(
            SceneEntity::new(
                ID_FIGURE_EIGHT,
                ScenePrimitive::Polyline(Polyline {
                    points: pts,
                    color: Color::rgb(0.30, 0.85, 0.55),
                    width: 1.0,
                }),
            )
            .with_label("figure_eight"),
        );

        // Hardcoded cube mesh.
        let (verts, idx) = cube_mesh(0.6);
        scene.upsert(
            SceneEntity::new(
                ID_CUBE_MESH,
                ScenePrimitive::Mesh(Mesh {
                    vertices: verts,
                    indices: idx,
                    material: Material {
                        base_color: Color::rgb(0.40, 0.60, 0.95),
                        texture: None,
                        wireframe: false,
                    },
                }),
            )
            .with_label("cube")
            .with_transform(Mat4::from_translation(Vec3::new(0.0, 0.6, -2.0))),
        );

        // 500 points scattered in a hemisphere above origin.
        let mut points = Vec::with_capacity(500);
        for i in 0..500 {
            let (x, y, z) = halton_hemisphere(i);
            points.push(Point {
                position: Vec3::new(x * 1.8, y * 1.8 + 0.05, z * 1.8 - 1.5),
                color: Color::rgba(0.95, 0.30, 0.55, 0.95),
                size: 4.0,
            });
        }
        scene.upsert(
            SceneEntity::new(ID_HEMISPHERE, ScenePrimitive::Points(points))
                .with_label("hemisphere_points"),
        );

        // A dense PoseArray-style cluster of arrows.
        let mut arrows = Vec::with_capacity(36);
        for i in 0..36 {
            let theta = (i as f32 / 36.0) * TAU;
            let r = 1.2;
            let origin = Vec3::new(r * theta.cos() + 3.0, 0.0, r * theta.sin() - 1.5);
            let dir = Vec3::new(-theta.sin(), 0.0, theta.cos());
            arrows.push(Arrow {
                origin,
                direction: dir,
                length: 0.35,
                shaft_radius: 0.025,
                head_radius: 0.05,
                color: Color::rgba(0.85, 0.85, 0.30, 0.95),
            });
        }
        scene.upsert(
            SceneEntity::new(ID_POSE_ARRAY, ScenePrimitive::Arrows(arrows))
                .with_label("pose_array"),
        );
    }
}

/// Tiny xorshift just so the grid pattern is reproducible.
fn pseudo_random_cells(cols: u32, rows: u32, seed: u32) -> Vec<u8> {
    let mut s = seed | 1;
    let n = (cols * rows) as usize;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        let r = s % 16;
        out.push(match r {
            0 => 100,           // occupied
            1..=3 => 255,       // unknown (-1 in OccupancyGrid -> 255 byte)
            _ => 0,             // free
        });
    }
    out
}

fn halton_hemisphere(i: usize) -> (f32, f32, f32) {
    fn halton(i: usize, b: u32) -> f32 {
        let mut f = 1.0;
        let mut r = 0.0;
        let mut idx = i + 1;
        let bf = b as f32;
        while idx > 0 {
            f /= bf;
            r += f * (idx as u32 % b) as f32;
            idx /= b as usize;
        }
        r
    }
    let u = halton(i, 2);
    let v = halton(i, 3);
    let phi = u * TAU;
    let costheta = v;
    let sintheta = (1.0 - costheta * costheta).sqrt();
    (sintheta * phi.cos(), costheta, sintheta * phi.sin())
}

fn cube_mesh(size: f32) -> (Vec<Vertex>, Vec<u32>) {
    let s = size * 0.5;
    let faces = [
        // (normal, four corner positions)
        ([0.0, 0.0, 1.0], [[-s, -s, s], [s, -s, s], [s, s, s], [-s, s, s]]),
        ([0.0, 0.0, -1.0], [[s, -s, -s], [-s, -s, -s], [-s, s, -s], [s, s, -s]]),
        ([1.0, 0.0, 0.0], [[s, -s, s], [s, -s, -s], [s, s, -s], [s, s, s]]),
        ([-1.0, 0.0, 0.0], [[-s, -s, -s], [-s, -s, s], [-s, s, s], [-s, s, -s]]),
        ([0.0, 1.0, 0.0], [[-s, s, s], [s, s, s], [s, s, -s], [-s, s, -s]]),
        ([0.0, -1.0, 0.0], [[-s, -s, -s], [s, -s, -s], [s, -s, s], [-s, -s, s]]),
    ];
    let mut verts: Vec<Vertex> = Vec::with_capacity(24);
    let mut idx: Vec<u32> = Vec::with_capacity(36);
    for (n, corners) in faces {
        let base = verts.len() as u32;
        let uvs = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        for i in 0..4 {
            verts.push(Vertex::new(corners[i], n, uvs[i]));
        }
        idx.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    (verts, idx)
}
