//! URDF loader and forward kinematics.
//!
//! Parses a URDF (from a file or an XML string published on
//! `/robot_description`) once at startup, resolves each link's visual geometry
//! to a `scene::Mesh` (loading STL/OBJ/Collada (.dae) files via
//! [`mesh_loader`] for `Geometry::Mesh`, tessellating built-in primitives in
//! place), and exposes [`UrdfModel::link_in_root`] so the JointState
//! subscriber can update per-link transforms after each message.
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
    /// `<mimic>` relationship, if any. Mimic joints don't appear in
    /// `/joint_states` directly — their position is derived as
    /// `multiplier * target.position + offset`.
    mimic: Option<MimicInfo>,
}

#[derive(Debug, Clone)]
struct MimicInfo {
    target: String,
    multiplier: f64,
    offset: f64,
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
    /// Parse the URDF from a file (supports xacro expansion), load every visual
    /// mesh, and produce a `SceneEntity` per link for the caller to upsert into
    /// the scene graph.
    pub fn load(path: &Path) -> Result<(Self, Vec<SceneEntity>)> {
        let robot: Robot = urdf_rs::utils::read_urdf_or_xacro(path)
            .with_context(|| format!("reading URDF {}", path.display()))?;
        let base_dir = path.parent().map(PathBuf::from);
        Self::build(robot, base_dir.as_deref())
    }

    /// Parse the URDF from an XML string (e.g. the payload of a
    /// `std_msgs/String` published on `/robot_description`). Mesh `package://`
    /// URIs are resolved through `AMENT_PREFIX_PATH` since there is no base
    /// directory.
    pub fn load_from_text(xml: &str) -> Result<(Self, Vec<SceneEntity>)> {
        let robot: Robot = urdf_rs::read_from_string(xml).context("parsing URDF text")?;
        Self::build(robot, None)
    }

    fn build(robot: Robot, base_dir: Option<&Path>) -> Result<(Self, Vec<SceneEntity>)> {
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

        // Log every non-Fixed joint and every mimic relationship. These are the
        // joints that need either /joint_states or /tf to render correctly —
        // and the most likely culprits when a single link looks misplaced
        // ("my bumper is hanging" almost always = mimic / multi-DoF joint we
        // can't FK on our own, no /tf publisher to fall back on).
        for j in joints_by_child.values() {
            let non_fixed = !matches!(j.joint_type, JointType::Fixed);
            if non_fixed || j.mimic.is_some() {
                let kind = format!("{:?}", j.joint_type).to_lowercase();
                match &j.mimic {
                    Some(m) => log::info!(
                        "urdf: joint {} ({kind}, parent={} child={}) mimics {} (×{} +{})",
                        j.name,
                        j.parent,
                        j.child,
                        m.target,
                        m.multiplier,
                        m.offset,
                    ),
                    None => log::info!(
                        "urdf: joint {} ({kind}, parent={} child={})",
                        j.name,
                        j.parent,
                        j.child,
                    ),
                }
            }
        }

        let mut links: Vec<UrdfLink> = Vec::new();
        let mut entities: Vec<SceneEntity> = Vec::new();
        for (idx, link) in robot.links.iter().enumerate() {
            let Some(visual) = link.visual.first() else {
                log::debug!("urdf: link {} has no <visual>; skipping", link.name);
                continue;
            };
            let mesh = match build_mesh(visual, base_dir) {
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
            let q = self.joint_position(joint);
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

    /// Resolve the position for a joint. Honours `<mimic joint="…" multiplier=
    /// "…" offset="…"/>` by chasing the target joint up to 16 hops (cycles or
    /// pathological chains collapse to 0). Joints with no mimic and no entry
    /// in `joint_positions` default to 0.
    fn joint_position(&self, joint: &JointInfo) -> f64 {
        if let Some(direct) = self.joint_positions.get(&joint.name).copied() {
            return direct;
        }
        let mut hops = 0;
        let mut cur = joint;
        let mut scale = 1.0_f64;
        let mut bias = 0.0_f64;
        while let Some(m) = &cur.mimic {
            scale *= m.multiplier;
            bias = m.multiplier * bias + m.offset;
            if let Some(q) = self.joint_positions.get(&m.target).copied() {
                return scale * q + bias;
            }
            // Target may itself be a mimic — follow it.
            let next = self
                .joints_by_child
                .values()
                .find(|j| j.name == m.target);
            cur = match next {
                Some(j) => j,
                None => break,
            };
            hops += 1;
            if hops > 16 {
                log::warn!("urdf: mimic chain >16 hops at {}; aborting", joint.name);
                break;
            }
        }
        0.0
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
    // URDF rpy is rotation about FIXED axes — extrinsic X then Y then Z,
    // i.e. the resulting matrix is Rz(yaw) * Ry(pitch) * Rx(roll). glam's
    // `EulerRot::XYZ` is the *intrinsic* variant (Rx*Ry*Rz), which is what
    // we used before — and wrong: visuals with non-trivial rpy rendered with
    // the rotation order flipped, producing garbled URDFs the moment any
    // link wasn't axis-aligned.
    let rot = Quat::from_euler(
        glam::EulerRot::XYZEx,
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
        let mimic = j.mimic.as_ref().map(|m| MimicInfo {
            target: m.joint.clone(),
            // URDF spec: missing multiplier defaults to 1.0, missing offset to 0.0.
            multiplier: m.multiplier.unwrap_or(1.0),
            offset: m.offset.unwrap_or(0.0),
        });
        JointInfo {
            name: j.name.clone(),
            joint_type: j.joint_type.clone(),
            parent: j.parent.link.clone(),
            child: j.child.link.clone(),
            origin: pose_to_mat4(&j.origin),
            axis,
            mimic,
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
    if !matches!(ext.as_str(), "stl" | "obj" | "dae") {
        return Err(anyhow!(
            "unsupported mesh extension {ext:?} (supported: .stl, .obj, .dae): {}",
            path.display()
        ));
    }
    let scene = mesh_loader::Loader::default()
        .load(path)
        .with_context(|| format!("loading mesh {}", path.display()))?;
    if scene.meshes.is_empty() {
        return Err(anyhow!("mesh file contained no geometry: {}", path.display()));
    }
    let merged = mesh_loader::Mesh::merge(scene.meshes);
    Ok(convert_mesh(merged, scale))
}

fn convert_mesh(mesh: mesh_loader::Mesh, scale: Vec3) -> (Vec<Vertex>, Vec<u32>) {
    let mesh_loader::Mesh {
        vertices: positions,
        normals,
        texcoords,
        faces,
        ..
    } = mesh;
    let has_normals = normals.len() == positions.len();
    let uv0 = &texcoords[0];
    let has_uvs = uv0.len() == positions.len();
    let mut vertices: Vec<Vertex> = Vec::with_capacity(positions.len());
    for (i, p) in positions.iter().enumerate() {
        let normal = if has_normals { normals[i] } else { [0.0, 0.0, 1.0] };
        let uv = if has_uvs { uv0[i] } else { [0.0, 0.0] };
        vertices.push(Vertex {
            position: [p[0] * scale.x, p[1] * scale.y, p[2] * scale.z],
            normal,
            uv,
        });
    }
    let mut indices: Vec<u32> = Vec::with_capacity(faces.len() * 3);
    for f in &faces {
        indices.extend_from_slice(&[f[0], f[1], f[2]]);
    }
    if !has_normals {
        recompute_normals(&mut vertices, &indices);
    }
    (vertices, indices)
}

fn recompute_normals(vertices: &mut [Vertex], indices: &[u32]) {
    let mut accum: Vec<[f32; 3]> = vec![[0.0; 3]; vertices.len()];
    for tri in indices.chunks_exact(3) {
        let (i0, i1, i2) = (tri[0] as usize, tri[1] as usize, tri[2] as usize);
        let p0 = vertices[i0].position;
        let p1 = vertices[i1].position;
        let p2 = vertices[i2].position;
        let e1 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
        let e2 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
        let n = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        for &idx in &[i0, i1, i2] {
            accum[idx][0] += n[0];
            accum[idx][1] += n[1];
            accum[idx][2] += n[2];
        }
    }
    for (v, n) in vertices.iter_mut().zip(accum.iter()) {
        let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        v.normal = if l > 1e-8 {
            [n[0] / l, n[1] / l, n[2] / l]
        } else {
            [0.0, 0.0, 1.0]
        };
    }
}

pub(crate) fn box_mesh(sx: f32, sy: f32, sz: f32) -> (Vec<Vertex>, Vec<u32>) {
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

pub(crate) fn cylinder_mesh(radius: f32, length: f32, segments: u32) -> (Vec<Vertex>, Vec<u32>) {
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

pub(crate) fn sphere_mesh(radius: f32, rings: u32, segments: u32) -> (Vec<Vertex>, Vec<u32>) {
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
            mimic: None,
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
            mimic: None,
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
    fn mimic_joint_follows_target_position() {
        // l1 is driven by j1 (revolute about Z). l2 sits 1m along +x of l1
        // and its joint j2 mimics j1 with multiplier=0.5, offset=π/4.
        // After setting j1 = π/2, j2's effective q = 0.5*π/2 + π/4 = π/2.
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
                <mimic joint='j1' multiplier='0.5' offset='0.7853981633974483'/>
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
        // j2 is NOT in /joint_states — only j1 is. Mimic must resolve it.
        model.apply_joint_positions(["j1"], &[std::f32::consts::FRAC_PI_2 as f64]);
        let q2 = model.joint_position(model.joints_by_child.get("l2").unwrap());
        assert!(
            (q2 - std::f32::consts::FRAC_PI_2 as f64).abs() < 1e-6,
            "j2 mimic should evaluate to π/2, got {q2}",
        );
    }

    #[test]
    fn rpy_follows_urdf_fixed_axis_convention() {
        // URDF rpy is rotation about FIXED axes (extrinsic XYZ), so the
        // matrix is Rz(yaw) * Ry(pitch) * Rx(roll). A point on +X rotated by
        // (roll=π/2, pitch=0, yaw=π/2) should end up on +Z:
        //   Rx(π/2): X → X
        //   Ry(0):   X → X
        //   Rz(π/2): X → Y     ❌ wrong
        // …no wait. Extrinsic XYZ applies the rotations in the *original*
        // frame in order X, Y, Z. So the composed matrix M = Rz·Ry·Rx
        // acts on a column vector. For v = (1,0,0):
        //   Rx(π/2)·v = (1,0,0)              (X axis fixed)
        //   Ry(0)·…  = (1,0,0)
        //   Rz(π/2)·… = (0,1,0)
        // So +X → +Y for roll=π/2, pitch=0, yaw=π/2. Pick a non-trivial case
        // that distinguishes intrinsic from extrinsic: r=π/2, p=π/2, y=0.
        //   Intrinsic XYZ (Rx*Ry*Rz):
        //     Rz(0)·(1,0,0) = (1,0,0)
        //     Ry(π/2)·(1,0,0) = (0,0,-1)
        //     Rx(π/2)·(0,0,-1) = (0,1,0)
        //   Extrinsic XYZ (Rz*Ry*Rx):
        //     Rx(π/2)·(1,0,0) = (1,0,0)
        //     Ry(π/2)·(1,0,0) = (0,0,-1)
        //     Rz(0)·(0,0,-1)  = (0,0,-1)
        // The two give different answers for (1,0,0) → distinguishes them.
        let pose = urdf_rs::Pose {
            xyz: urdf_rs::Vec3([0.0, 0.0, 0.0]),
            rpy: urdf_rs::Vec3([
                std::f32::consts::FRAC_PI_2 as f64,
                std::f32::consts::FRAC_PI_2 as f64,
                0.0,
            ]),
        };
        let m = pose_to_mat4(&pose);
        let p = m.transform_point3(Vec3::X);
        // Extrinsic XYZ result: (0, 0, -1).
        assert!(
            (p - Vec3::new(0.0, 0.0, -1.0)).length() < 1e-5,
            "extrinsic XYZ expected (0,0,-1), got {p:?}",
        );
    }

    #[test]
    fn box_mesh_has_36_indices() {
        let (v, i) = box_mesh(1.0, 1.0, 1.0);
        assert_eq!(v.len(), 24);
        assert_eq!(i.len(), 36);
    }
}
