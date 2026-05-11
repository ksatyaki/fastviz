//! URDF loader and forward kinematics.
//!
//! Parses a URDF file once at startup, resolves each link's visual geometry to
//! a `scene::Mesh` (loading STL/OBJ files for `Geometry::Mesh`, tessellating
//! built-in primitives in place), and exposes [`UrdfModel::link_world`] so the
//! JointState subscriber can update per-link transforms after each message.
//!
//! Kinematics are computed in ROS coordinates (right-handed, Z up). The
//! callsite composes [`crate::coords::ROS_TO_WORLD`] and the TF lookup from
//! `reference_frame ← urdf_root_frame` on top.
//!
//! Joint support: `Revolute`, `Continuous`, `Prismatic`, and `Fixed` move under
//! `JointState`. `Floating`, `Planar`, and `Spherical` are treated as `Fixed`
//! (warning logged at parse time).
//!
//! `package://` URIs in mesh filenames are resolved through urdf-rs's existing
//! helper, which calls out to `ros2 pkg prefix` / `AMENT_PREFIX_PATH`.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use anyhow::{anyhow, Context, Result};
use glam::{Mat4, Quat, Vec3};
use scene::{Color, Material, Mesh, SceneEntity, ScenePrimitive, Vertex};
use urdf_rs::{Geometry, Joint, JointType, Robot, Visual};

use crate::ids::URDF_LINK_BASE;

/// Per-link visual entity together with the kinematic info needed to recompute
/// its transform every JointState tick.
#[derive(Debug)]
pub struct UrdfLink {
    pub name: String,
    pub entity_id: scene::EntityId,
    /// Pose of the visual relative to the link frame (URDF `<visual><origin/>`).
    pub visual_origin: Mat4,
}

#[derive(Debug)]
struct JointInfo {
    name: String,
    joint_type: JointType,
    /// Parent link.
    parent: String,
    /// Child link.
    child: String,
    /// Fixed offset from parent → child at zero joint position.
    origin: Mat4,
    /// Joint axis, expressed in the joint (≡ child) frame.
    axis: Vec3,
}

pub struct UrdfModel {
    /// Root link name — corresponds to the URDF's coordinate frame, looked up
    /// in TF as a child of `reference_frame`.
    pub root_link: String,
    pub links: Vec<UrdfLink>,
    /// Joints keyed by child-link name. URDF guarantees each link has at most
    /// one parent joint, so the chain from any link to root is unambiguous.
    joints_by_child: HashMap<String, JointInfo>,
    /// Parent link of each non-root link (derived from joints).
    parent_of: HashMap<String, String>,
    /// Joint positions; defaults to zero, updated by JointState messages.
    joint_positions: HashMap<String, f64>,
}

impl UrdfModel {
    /// Parse the URDF, load every visual mesh, and produce a `SceneEntity` per
    /// link for the caller to upsert into the scene graph.
    pub fn load(path: &Path) -> Result<(Self, Vec<SceneEntity>)> {
        let robot: Robot = urdf_rs::utils::read_urdf_or_xacro(path)
            .with_context(|| format!("reading URDF {}", path.display()))?;
        let base_dir = path.parent().map(PathBuf::from);

        let mut joints_by_child: HashMap<String, JointInfo> = HashMap::new();
        let mut parent_of: HashMap<String, String> = HashMap::new();
        let mut children: HashMap<String, Vec<String>> = HashMap::new();
        for joint in &robot.joints {
            let info = JointInfo::from(joint);
            children
                .entry(info.parent.clone())
                .or_default()
                .push(info.child.clone());
            parent_of.insert(info.child.clone(), info.parent.clone());
            joints_by_child.insert(info.child.clone(), info);
        }

        let root_link = find_root_link(&robot, &parent_of)?;
        log::info!("urdf: {} link(s), root = {root_link}", robot.links.len());

        let mut links: Vec<UrdfLink> = Vec::new();
        let mut entities: Vec<SceneEntity> = Vec::new();
        for (idx, link) in robot.links.iter().enumerate() {
            let Some(visual) = link.visual.first() else {
                log::debug!("urdf: link {} has no <visual>; skipping", link.name);
                continue;
            };
            let mesh = match build_mesh(visual, base_dir.as_deref()) {
                Ok(m) => m,
                Err(e) => {
                    log::warn!("urdf: link {}: {e:#}", link.name);
                    continue;
                }
            };
            let entity_id = scene::EntityId(URDF_LINK_BASE + idx as u64);
            let visual_origin = pose_to_mat4(&visual.origin);
            entities.push(
                SceneEntity::new(entity_id, ScenePrimitive::Mesh(mesh))
                    .with_label(link.name.clone()),
            );
            links.push(UrdfLink {
                name: link.name.clone(),
                entity_id,
                visual_origin,
            });
        }

        Ok((
            UrdfModel {
                root_link,
                links,
                joints_by_child,
                parent_of,
                joint_positions: HashMap::new(),
            },
            entities,
        ))
    }

    /// Apply a `sensor_msgs/JointState`-style update: every named joint's
    /// position is overwritten. Unknown joints are ignored (they may belong to
    /// a different controller).
    pub fn apply_joint_positions<I, S>(&mut self, names: I, positions: &[f64])
    where
        I: IntoIterator<Item = S>,
        S: AsRef<str>,
    {
        for (name, pos) in names.into_iter().zip(positions.iter().copied()) {
            self.joint_positions.insert(name.as_ref().to_string(), pos);
        }
    }

    /// Walk the chain from `link` to root and accumulate joint transforms. The
    /// result is `root_T_link` — multiply on the left to lift a link-local point
    /// into the URDF root frame.
    pub fn link_in_root(&self, link: &str) -> Mat4 {
        let mut accum = Mat4::IDENTITY;
        let mut cur = link;
        let mut steps = 0;
        while let Some(parent) = self.parent_of.get(cur) {
            let joint = match self.joints_by_child.get(cur) {
                Some(j) => j,
                None => break,
            };
            let q = self
                .joint_positions
                .get(&joint.name)
                .copied()
                .unwrap_or(0.0);
            accum = joint.origin * joint_motion(joint, q) * accum;
            cur = parent.as_str();
            steps += 1;
            if steps > 1024 {
                log::warn!("urdf: kinematic chain depth > 1024 starting at {link}; aborting");
                break;
            }
        }
        accum
    }

    /// `root_T_visual` for the given link: `link_in_root * visual_origin`.
    pub fn visual_in_root(&self, link: &UrdfLink) -> Mat4 {
        self.link_in_root(&link.name) * link.visual_origin
    }
}

fn find_root_link(robot: &Robot, parent_of: &HashMap<String, String>) -> Result<String> {
    let candidates: Vec<&str> = robot
        .links
        .iter()
        .map(|l| l.name.as_str())
        .filter(|n| !parent_of.contains_key(*n))
        .collect();
    match candidates.as_slice() {
        [single] => Ok((*single).to_string()),
        [] => Err(anyhow!("urdf has no root link (every link is a joint child)")),
        many => {
            log::warn!(
                "urdf has {} root-like links {:?}; using the first",
                many.len(),
                many
            );
            Ok(many[0].to_string())
        }
    }
}

fn pose_to_mat4(pose: &urdf_rs::Pose) -> Mat4 {
    let t = Vec3::new(pose.xyz.0[0] as f32, pose.xyz.0[1] as f32, pose.xyz.0[2] as f32);
    let r = pose.rpy.0;
    let rot = Quat::from_euler(
        glam::EulerRot::XYZ,
        r[0] as f32,
        r[1] as f32,
        r[2] as f32,
    );
    Mat4::from_rotation_translation(rot, t)
}

fn joint_motion(joint: &JointInfo, q: f64) -> Mat4 {
    match joint.joint_type {
        JointType::Revolute | JointType::Continuous => {
            Mat4::from_quat(Quat::from_axis_angle(joint.axis, q as f32))
        }
        JointType::Prismatic => Mat4::from_translation(joint.axis * q as f32),
        JointType::Fixed => Mat4::IDENTITY,
        // Multi-DoF joints are not in M0.5 scope; treat as fixed.
        _ => Mat4::IDENTITY,
    }
}

impl JointInfo {
    fn from(j: &Joint) -> Self {
        if matches!(
            j.joint_type,
            JointType::Floating | JointType::Planar | JointType::Spherical
        ) {
            log::warn!(
                "urdf: joint {} is {:?}; treating as Fixed (multi-DoF not supported in M0.5)",
                j.name,
                j.joint_type
            );
        }
        let axis = Vec3::new(
            j.axis.xyz.0[0] as f32,
            j.axis.xyz.0[1] as f32,
            j.axis.xyz.0[2] as f32,
        )
        .try_normalize()
        .unwrap_or(Vec3::X);
        JointInfo {
            name: j.name.clone(),
            joint_type: j.joint_type.clone(),
            parent: j.parent.link.clone(),
            child: j.child.link.clone(),
            origin: pose_to_mat4(&j.origin),
            axis,
        }
    }
}

// ---------- mesh construction ----------

fn build_mesh(visual: &Visual, base_dir: Option<&Path>) -> Result<Mesh> {
    let material = visual
        .material
        .as_ref()
        .and_then(|m| m.color.as_ref())
        .map(|c| Material {
            base_color: Color::rgba(
                c.rgba.0[0] as f32,
                c.rgba.0[1] as f32,
                c.rgba.0[2] as f32,
                c.rgba.0[3] as f32,
            ),
            ..Material::default()
        })
        .unwrap_or(Material {
            base_color: Color::rgb(0.75, 0.75, 0.78),
            ..Material::default()
        });

    let (vertices, indices) = match &visual.geometry {
        Geometry::Box { size } => {
            box_mesh(size.0[0] as f32, size.0[1] as f32, size.0[2] as f32)
        }
        Geometry::Cylinder { radius, length } => {
            cylinder_mesh(*radius as f32, *length as f32, 24)
        }
        Geometry::Capsule { radius, length } => {
            // Capsule rendered as cylinder for M0.5.
            cylinder_mesh(*radius as f32, *length as f32, 24)
        }
        Geometry::Sphere { radius } => sphere_mesh(*radius as f32, 16, 16),
        Geometry::Mesh { filename, scale } => {
            let scale = scale
                .as_ref()
                .map(|s| Vec3::new(s.0[0] as f32, s.0[1] as f32, s.0[2] as f32))
                .unwrap_or(Vec3::ONE);
            load_mesh_file(filename, base_dir, scale)?
        }
    };

    Ok(Mesh {
        vertices,
        indices,
        material,
    })
}

fn load_mesh_file(
    filename: &str,
    base_dir: Option<&Path>,
    scale: Vec3,
) -> Result<(Vec<Vertex>, Vec<u32>)> {
    let resolved = urdf_rs::utils::expand_package_path(filename, base_dir)
        .with_context(|| format!("resolving mesh path {filename}"))?;
    let path = Path::new(resolved.as_ref());
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .map(|s| s.to_ascii_lowercase())
        .unwrap_or_default();

    match ext.as_str() {
        "stl" => load_stl(path, scale),
        "obj" => load_obj(path, scale),
        other => Err(anyhow!(
            "unsupported mesh extension {other:?} (only .stl and .obj in M0.5): {}",
            path.display()
        )),
    }
}

fn load_stl(path: &Path, scale: Vec3) -> Result<(Vec<Vertex>, Vec<u32>)> {
    let mut file = std::fs::File::open(path)
        .with_context(|| format!("opening STL {}", path.display()))?;
    let mesh = stl_io::read_stl(&mut file)
        .with_context(|| format!("parsing STL {}", path.display()))?;
    let vertices: Vec<Vertex> = mesh
        .vertices
        .iter()
        .map(|v| Vertex {
            position: [v[0] * scale.x, v[1] * scale.y, v[2] * scale.z],
            normal: [0.0, 0.0, 1.0],
            uv: [0.0, 0.0],
        })
        .collect();
    let mut indices: Vec<u32> = Vec::with_capacity(mesh.faces.len() * 3);
    let mut normals_accum: Vec<[f32; 3]> = vec![[0.0; 3]; vertices.len()];
    for tri in &mesh.faces {
        let i0 = tri.vertices[0] as u32;
        let i1 = tri.vertices[1] as u32;
        let i2 = tri.vertices[2] as u32;
        indices.extend_from_slice(&[i0, i1, i2]);
        let n = tri.normal;
        for &idx in &[i0, i1, i2] {
            let a = &mut normals_accum[idx as usize];
            a[0] += n[0];
            a[1] += n[1];
            a[2] += n[2];
        }
    }
    let mut out = vertices;
    for (v, n) in out.iter_mut().zip(normals_accum.iter()) {
        let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        v.normal = if l > 1e-8 {
            [n[0] / l, n[1] / l, n[2] / l]
        } else {
            [0.0, 0.0, 1.0]
        };
    }
    Ok((out, indices))
}

fn load_obj(path: &Path, scale: Vec3) -> Result<(Vec<Vertex>, Vec<u32>)> {
    let (models, _materials) = tobj::load_obj(path, &tobj::GPU_LOAD_OPTIONS)
        .with_context(|| format!("parsing OBJ {}", path.display()))?;
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut offset: u32 = 0;
    for model in &models {
        let m = &model.mesh;
        let n_verts = m.positions.len() / 3;
        let has_normals = m.normals.len() == m.positions.len();
        let has_uvs = m.texcoords.len() / 2 == n_verts;
        for i in 0..n_verts {
            let p = [
                m.positions[3 * i] * scale.x,
                m.positions[3 * i + 1] * scale.y,
                m.positions[3 * i + 2] * scale.z,
            ];
            let normal = if has_normals {
                [m.normals[3 * i], m.normals[3 * i + 1], m.normals[3 * i + 2]]
            } else {
                [0.0, 0.0, 1.0]
            };
            let uv = if has_uvs {
                [m.texcoords[2 * i], m.texcoords[2 * i + 1]]
            } else {
                [0.0, 0.0]
            };
            vertices.push(Vertex {
                position: p,
                normal,
                uv,
            });
        }
        indices.extend(m.indices.iter().map(|&i| i + offset));
        offset += n_verts as u32;
    }
    Ok((vertices, indices))
}

fn box_mesh(sx: f32, sy: f32, sz: f32) -> (Vec<Vertex>, Vec<u32>) {
    let (hx, hy, hz) = (sx * 0.5, sy * 0.5, sz * 0.5);
    let faces: [[[f32; 3]; 5]; 6] = [
        // Each row: normal, then four corners.
        [[1.0, 0.0, 0.0], [hx, -hy, -hz], [hx, hy, -hz], [hx, hy, hz], [hx, -hy, hz]],
        [[-1.0, 0.0, 0.0], [-hx, hy, -hz], [-hx, -hy, -hz], [-hx, -hy, hz], [-hx, hy, hz]],
        [[0.0, 1.0, 0.0], [hx, hy, -hz], [-hx, hy, -hz], [-hx, hy, hz], [hx, hy, hz]],
        [[0.0, -1.0, 0.0], [-hx, -hy, -hz], [hx, -hy, -hz], [hx, -hy, hz], [-hx, -hy, hz]],
        [[0.0, 0.0, 1.0], [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz]],
        [[0.0, 0.0, -1.0], [hx, -hy, -hz], [-hx, -hy, -hz], [-hx, hy, -hz], [hx, hy, -hz]],
    ];
    let mut verts = Vec::with_capacity(24);
    let mut idx = Vec::with_capacity(36);
    for face in &faces {
        let n = face[0];
        let base = verts.len() as u32;
        for (i, uv) in [(1, [0.0, 0.0]), (2, [1.0, 0.0]), (3, [1.0, 1.0]), (4, [0.0, 1.0])] {
            verts.push(Vertex {
                position: face[i],
                normal: n,
                uv,
            });
        }
        idx.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    (verts, idx)
}

fn cylinder_mesh(radius: f32, length: f32, segments: u32) -> (Vec<Vertex>, Vec<u32>) {
    let half = length * 0.5;
    let segments = segments.max(3);
    let mut verts = Vec::new();
    let mut idx = Vec::new();
    // Side wall.
    for i in 0..segments {
        let t0 = (i as f32) / segments as f32 * std::f32::consts::TAU;
        let t1 = ((i + 1) as f32) / segments as f32 * std::f32::consts::TAU;
        let (c0, s0) = (t0.cos(), t0.sin());
        let (c1, s1) = (t1.cos(), t1.sin());
        let base = verts.len() as u32;
        verts.push(Vertex {
            position: [radius * c0, radius * s0, -half],
            normal: [c0, s0, 0.0],
            uv: [0.0, 0.0],
        });
        verts.push(Vertex {
            position: [radius * c1, radius * s1, -half],
            normal: [c1, s1, 0.0],
            uv: [0.0, 0.0],
        });
        verts.push(Vertex {
            position: [radius * c1, radius * s1, half],
            normal: [c1, s1, 0.0],
            uv: [0.0, 1.0],
        });
        verts.push(Vertex {
            position: [radius * c0, radius * s0, half],
            normal: [c0, s0, 0.0],
            uv: [0.0, 1.0],
        });
        idx.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    // Caps.
    let top_center = verts.len() as u32;
    verts.push(Vertex {
        position: [0.0, 0.0, half],
        normal: [0.0, 0.0, 1.0],
        uv: [0.5, 0.5],
    });
    let bot_center = verts.len() as u32;
    verts.push(Vertex {
        position: [0.0, 0.0, -half],
        normal: [0.0, 0.0, -1.0],
        uv: [0.5, 0.5],
    });
    for i in 0..segments {
        let t0 = (i as f32) / segments as f32 * std::f32::consts::TAU;
        let t1 = ((i + 1) as f32) / segments as f32 * std::f32::consts::TAU;
        let (c0, s0) = (t0.cos(), t0.sin());
        let (c1, s1) = (t1.cos(), t1.sin());
        let top0 = verts.len() as u32;
        verts.push(Vertex {
            position: [radius * c0, radius * s0, half],
            normal: [0.0, 0.0, 1.0],
            uv: [0.0, 0.0],
        });
        let top1 = verts.len() as u32;
        verts.push(Vertex {
            position: [radius * c1, radius * s1, half],
            normal: [0.0, 0.0, 1.0],
            uv: [0.0, 0.0],
        });
        idx.extend_from_slice(&[top_center, top0, top1]);
        let bot0 = verts.len() as u32;
        verts.push(Vertex {
            position: [radius * c0, radius * s0, -half],
            normal: [0.0, 0.0, -1.0],
            uv: [0.0, 0.0],
        });
        let bot1 = verts.len() as u32;
        verts.push(Vertex {
            position: [radius * c1, radius * s1, -half],
            normal: [0.0, 0.0, -1.0],
            uv: [0.0, 0.0],
        });
        idx.extend_from_slice(&[bot_center, bot1, bot0]);
    }
    (verts, idx)
}

fn sphere_mesh(radius: f32, rings: u32, segments: u32) -> (Vec<Vertex>, Vec<u32>) {
    let rings = rings.max(2);
    let segments = segments.max(3);
    let mut verts = Vec::new();
    let mut idx = Vec::new();
    for r in 0..=rings {
        let phi = std::f32::consts::PI * (r as f32) / rings as f32;
        let (sp, cp) = (phi.sin(), phi.cos());
        for s in 0..=segments {
            let theta = std::f32::consts::TAU * (s as f32) / segments as f32;
            let (st, ct) = (theta.sin(), theta.cos());
            let nx = sp * ct;
            let ny = sp * st;
            let nz = cp;
            verts.push(Vertex {
                position: [radius * nx, radius * ny, radius * nz],
                normal: [nx, ny, nz],
                uv: [s as f32 / segments as f32, r as f32 / rings as f32],
            });
        }
    }
    let stride = segments + 1;
    for r in 0..rings {
        for s in 0..segments {
            let a = r * stride + s;
            let b = a + stride;
            idx.extend_from_slice(&[a, b, a + 1, a + 1, b, b + 1]);
        }
    }
    (verts, idx)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(urdf: &str) -> Robot {
        urdf_rs::read_from_string(urdf).unwrap()
    }

    #[test]
    fn revolute_z_rotates_axis() {
        let info = JointInfo {
            name: "j".into(),
            joint_type: JointType::Revolute,
            parent: "p".into(),
            child: "c".into(),
            origin: Mat4::IDENTITY,
            axis: Vec3::Z,
        };
        let m = joint_motion(&info, std::f32::consts::FRAC_PI_2 as f64);
        let p = m.transform_point3(Vec3::X);
        assert!((p - Vec3::Y).length() < 1e-5, "rot(z, 90°) X → Y, got {p}");
    }

    #[test]
    fn prismatic_translates_along_axis() {
        let info = JointInfo {
            name: "j".into(),
            joint_type: JointType::Prismatic,
            parent: "p".into(),
            child: "c".into(),
            origin: Mat4::IDENTITY,
            axis: Vec3::X,
        };
        let m = joint_motion(&info, 1.5);
        let p = m.transform_point3(Vec3::ZERO);
        assert!((p - Vec3::new(1.5, 0.0, 0.0)).length() < 1e-5);
    }

    #[test]
    fn fk_composes_two_revolute() {
        // base → l1 (1m along +x) → l2 (1m along +x), both revolute about Z.
        let urdf = r#"
            <robot name='t'>
              <link name='base'/>
              <link name='l1'/>
              <link name='l2'/>
              <joint name='j1' type='revolute'>
                <parent link='base'/>
                <child link='l1'/>
                <origin xyz='0 0 0' rpy='0 0 0'/>
                <axis xyz='0 0 1'/>
                <limit lower='-3' upper='3' effort='0' velocity='0'/>
              </joint>
              <joint name='j2' type='revolute'>
                <parent link='l1'/>
                <child link='l2'/>
                <origin xyz='1 0 0' rpy='0 0 0'/>
                <axis xyz='0 0 1'/>
                <limit lower='-3' upper='3' effort='0' velocity='0'/>
              </joint>
            </robot>
        "#;
        let robot = parse(urdf);
        let mut parent_of = HashMap::new();
        let mut joints_by_child = HashMap::new();
        for j in &robot.joints {
            let i = JointInfo::from(j);
            parent_of.insert(i.child.clone(), i.parent.clone());
            joints_by_child.insert(i.child.clone(), i);
        }
        let mut model = UrdfModel {
            root_link: "base".into(),
            links: Vec::new(),
            joints_by_child,
            parent_of,
            joint_positions: HashMap::new(),
        };
        model.apply_joint_positions(["j1"], &[std::f32::consts::FRAC_PI_2 as f64]);
        // l1 origin (no per-joint offset → 0,0,0) under j1 = 90° about Z stays (0,0,0).
        let p1 = model.link_in_root("l1").transform_point3(Vec3::ZERO);
        assert!(p1.length() < 1e-4, "l1 at {p1:?}");
        // l2 origin: j1 rotates 90° about z → j2 origin (1,0,0) becomes (0,1,0).
        let p2 = model.link_in_root("l2").transform_point3(Vec3::ZERO);
        assert!((p2 - Vec3::Y).length() < 1e-4, "l2 expected (0,1,0), got {p2:?}");
    }

    #[test]
    fn box_mesh_has_36_indices() {
        let (v, i) = box_mesh(1.0, 1.0, 1.0);
        assert_eq!(v.len(), 24);
        assert_eq!(i.len(), 36);
    }
}
