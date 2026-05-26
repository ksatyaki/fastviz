# Milestone 0.5 — Babysteps Plan

This is a self-contained execution plan for a fresh Claude Code session
running **inside the dev container**. It captures the decisions made in
the M0→M0.5 hand-off conversation so we don't have to re-debate them.

---

## 0. Progress (as of 2026-05-26)

| Step | What | Status |
|---|---|---|
| A (0.5.1) | Crate skeleton + `RosNode::spawn` thread + clean shutdown | ✅ done |
| B (0.5.3) | OccupancyGrid (`/map`) → `Grid` (no TF) | ✅ done |
| C (0.5.2) | TF tree + reframe to `map` | ✅ done |
| D (0.5.4) | PoseStamped + PoseArray → Arrow(s) | ✅ done |
| E (0.5.5) | Path → Polyline | ✅ done |
| F (0.5.6) | LaserScan → Points | ✅ done |
| F.5 | Config file + polled topic discovery + per-topic QoS | ✅ done |
| G | CLI flip: ROS always on, `--mock` opt-in only | ✅ done |
| H | PointCloud2 → Points (M3 pulled into M0.5) | ✅ done |
| H.1 | PC2 received-vs-displayed counter | ✅ done |
| H.2 | PointPass perf: revision-cached, GPU-side transform, buffer reuse | ✅ done |
| I (0.5.7) | URDF + JointState → meshes + FK | ✅ done |
| J | `visualization_msgs/Marker` + `MarkerArray` → primitives | ✅ done (2026-05-26) |

---

## 1. Decisions log (do not relitigate)

| Decision | Choice | Reason |
|---|---|---|
| Rust→ROS2 binding | **`r2r`** (only) | Pure-cargo build flow, mature for sub/pub, sufficient for the six M0.5 message types. `rclrs` is deferred until we hit dynamic-message needs. |
| ROS2 distro | **Jazzy** (24.04) | Matches the existing `Dockerfile`'s base; long-term support; `nav_msgs`/`sensor_msgs` packaged. |
| Dev environment | **devcontainer** (`.devcontainer/Dockerfile`) | Fedora host has no `/opt/ros`. Container gives us Jazzy + apt-installed message packages. NVIDIA Vulkan is enabled via `nvidia-container-toolkit` + `NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute`. |
| Workspace path inside container | `/workspaces/fastviz` | devcontainer convention; `~/.claude` is mounted so this conversation history survives. |
| Threading model | Render loop on main thread; `r2r` executor on a dedicated thread; communication via `Arc<RwLock<SceneGraph>>`. | Same model the project plan §0.5.1 prescribes. |
| ROS env at build time | `LIBCLANG_PATH=/usr/lib/x86_64-linux-gnu` exported in `~/.bashrc` (Dockerfile already does this). | r2r's `bindgen` step needs libclang; without this it fails on first build. |

---

## 2. Step I — URDF + JointState (next)

Workspace deps to add when starting:

```toml
urdf-rs = "0.8"
stl_io = "0.8"          # binary STL loader
tobj = "4"              # OBJ loader (no MTL needed for M0.5)
```

New modules under `crates/ros_node/src/`:
- `urdf.rs` — parse URDF, load STL/OBJ, FK
- `subscribers/jointstate.rs` — `sensor_msgs/JointState` → mesh transforms

Implementation outline:

`urdf-rs::read_from_file(...)` → `Robot`. For each `Link` with a `Visual.geometry::Mesh`, resolve the mesh path (`package://foo/bar.stl` lookup uses `AMENT_PREFIX_PATH` env). Load with `stl_io` for STL or `tobj` for OBJ, convert to `scene::primitives::Mesh`. Register each link as a `SceneEntity` whose `transform` will be updated by FK.

`JointState` callback:
1. Update internal joint angle map.
2. Walk the URDF kinematic chain → compute world transforms per link (root identity, accumulate joint transforms).
3. For each link, `scene.update_transform(entity_id, world_xform)` — no GPU re-upload (mesh data unchanged).

CLI arg: `--urdf /path/to/robot.urdf`. If provided, load at startup; otherwise skip.

Smoke test:
```bash
cargo run -p app -- --urdf /opt/ros/jazzy/share/turtlebot3_description/urdf/turtlebot3_burger.urdf
# wiggle joints with:
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

---

## 3. Conventions

### 3.1 EntityId allocation

| Range | Owner |
|---|---|
| `1..=999` | mock injector (M0) |
| `1000..=1999` | ROS singletons (TF frames hash into this, occupancy=1000, etc.) |
| `2000..=2999` | per-topic poses/paths/scans/pointclouds (allocate sequentially as topics arrive) |
| `3000..=3999` | URDF link entities |
| `4_000_000..` | `visualization_msgs/Marker` entities — 100k-wide slab per marker topic; each `(ns, id)` from the publisher gets one slot, allocated on first sight (see `ids::marker_topic_base`). |

### 3.2 Reference frame

`SceneGraph::reference_frame` (currently `"map"` from M0) is the target for all TF lookups. User-configurable via `--ref-frame` CLI flag and the config file.

### 3.3 Visibility and labels

Every entity should be created with `with_label(topic_name_or_link_name)` so the entity list panel is meaningful.

---

## 4. Things explicitly out of scope for M0.5

- `Image`, `Imu`, `Odometry` → milestones 1–4.
- MCAP record/playback → M5.
- Plugin system → M4.
- Any rclrs work.
- Wayland-native windowing.
- TF interpolation (added only if jitter is visible).

---

## 5. Step J — `visualization_msgs/Marker` (done 2026-05-26)

Pulled forward from M1 because it's a single-message subscriber and the
existing primitive set already covers most marker shapes. Added:

- `crates/ros_node/src/subscribers/marker.rs` — subscribes to either
  `visualization_msgs/Marker` or `visualization_msgs/MarkerArray` per
  configured topic.
- Per-topic state: `(ns, id) → EntityId` allocator inside a 100k-wide slab
  starting at `ids::ROS_ID_MARKER_BASE = 4_000_000`. `DELETE` removes one
  entity; `DELETEALL` clears every entity owned by that topic.
- TF integration: each marker is registered with `TfRegistry`, so late
  `/tf` arrivals retroactively reposition it (same mechanism scans and
  point clouds already use).
- Config schema additions (TOML):
  - `[markers]` — `topics = [...]`, optional `[markers.qos."/foo"]` per-topic QoS.
  - `[marker_arrays]` — same shape, for `MarkerArray`-typed topics.
- Wildcard `"*"` works in both sections, going through the existing
  `subscribers::discovery::tick` poll path.
- `config_writer` gained `TopicKind::Marker` / `MarkerArray` so the
  Save-config window can emit `[markers]` / `[marker_arrays]` sections
  from selected topics.

**Marker-type coverage:**

| `Marker.type_` | Mapping |
|---|---|
| `ARROW` (0) | `ScenePrimitive::Arrows`. Both forms supported: position-and-scale, and the two-point `points[0..2]` form. |
| `CUBE` (1) | `Mesh` via `urdf::box_mesh` with `scale.xyz` as side lengths. |
| `SPHERE` (2) | `Mesh` via a unit sphere whose vertices are scaled to `scale/2` per axis (ellipsoid). |
| `CYLINDER` (3) | `Mesh` via `urdf::cylinder_mesh` (radius = `max(sx, sy)/2`, height = `sz`). |
| `LINE_STRIP` (4) | `Polyline` with width = `scale.x`. |
| `CUBE_LIST` (6), `SPHERE_LIST` (7), `POINTS` (8) | `Points`; per-point colors preserved when `colors[]` is populated. |
| `TEXT_VIEW_FACING` (9) | `Labels` at the marker pose; height = `scale.z`. |
| `TRIANGLE_LIST` (11) | `Mesh` built from `points[]` in triples with per-triangle flat normals. |
| `LINE_LIST` (5), `MESH_RESOURCE` (10), `ARROW_STRIP` (12) | Logged once per topic; skipped. (M1+.) |

**Out of scope inside Step J:**

- Per-vertex colors for `LINE_STRIP` (`colors[]` array on a polyline) — only
  the marker's flat color is used. Renderer's `LinePass` doesn't take
  per-vertex color attributes yet.
- `MESH_RESOURCE` mesh loading — would re-share URDF's `mesh-loader` code
  path but needs a different `package://` resolution context. Punted.
- Lifetime expiry. Markers stay until DELETE/DELETEALL or the topic
  re-publishes the same `(ns, id)`.
- Frame-locked vs. message-time semantics: every marker is treated as
  frame-locked (re-resolved through TF on each refresh).

**Smoke test from the devcontainer:**

```bash
# Publisher side:
ros2 topic pub /visualization_marker visualization_msgs/msg/Marker \
  "{header: {frame_id: 'map'}, ns: 'demo', id: 0, type: 1, action: 0, \
    pose: {position: {x: 1.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}, \
    scale: {x: 0.4, y: 0.4, z: 0.4}, color: {r: 1.0, g: 0.5, b: 0.0, a: 1.0}}" -r 1

# Viz side (fastviz config snippet):
# [markers]
# topics = ["/visualization_marker"]
```

