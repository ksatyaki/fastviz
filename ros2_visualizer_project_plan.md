# ROS2 Visualizer — Project Plan
*A ROS2-native, GPU-accelerated visualization tool built in Rust*

*Plan version 2.0 — May 2026.* Folds the prior `ros2_visualizer_milestone0_detailed.md`
into a single document. Milestone 0 is complete; Milestone 0.5 is in progress
(only Step I — URDF + JointState — remaining). PointCloud2 was pulled forward
from M3 into M0.5. For the running execution log and per-step notes, see
[MBABYSTEPS_1.md](MBABYSTEPS_1.md).

---

## Vision

A standalone, open source ROS2 visualizer that:
- Requires **no bridge, no middleware, no separate process** — it is a ROS2 node
- Renders with **wgpu** (Vulkan/Metal/DX12/WebGPU) for GPU-accelerated performance
- Works **natively inside Docker containers** with X11/Wayland passthrough
- Supports the full breadth of common ROS2 data types
- Is **extensible** — custom message types and plugins from day one
- Targets **Humble + Jazzy** (the two active LTS releases); Jazzy is the dev default

---

## Name

Working name: **`fastviz`**. To be revisited at community launch.

---

## Technology Stack

| Layer | Choice | Rationale |
|---|---|---|
| Language | Rust | Performance, safety, wgpu ecosystem |
| Rendering | wgpu 22 + naga | GPU-native, cross-backend (Vulkan primary on Linux) |
| UI framework | egui 0.28 | Immediate-mode, embeds cleanly with wgpu, used by Rerun |
| Windowing | winit 0.30 | ApplicationHandler model, X11 + Wayland |
| ROS2 bindings | **r2r 0.9** (only) | Pure-cargo build flow, mature for sub/pub. `rclrs` is deferred until dynamic-message needs appear. |
| Message types | r2r generates from `AMENT_PREFIX_PATH` at build time | No `ros_idl` codegen step in our tree |
| TF | Reimplemented in Rust (`crates/ros_node/src/tf.rs`) | Latest-only lookup; interpolation deferred unless jitter is observed |
| Occupancy grid | wgpu texture | Grid as R8Unorm texture; colormap in fragment shader |
| Point cloud | Custom wgpu pipeline | GPU-side per-entity transform; `revision()`-cached `prepare()` |
| Mesh/URDF | `urdf-rs` + `stl_io` (binary STL) + `tobj` (OBJ) | DAE deferred |
| Image display | wgpu texture upload (M1+) | Direct GPU texture, no CPU copy per frame |
| Config/layout | TOML (`fastviz.toml`) | Schema deserialised via serde; `#[serde(deny_unknown_fields)]` at the top level |
| Build system | Cargo workspace | Plays well with ROS2 workspaces but is independent of `colcon` |
| CI | GitHub Actions | Build, test, clippy, format |
| Container | Devcontainer (Ubuntu 24.04 + Jazzy) + a release `Dockerfile` | NVIDIA Vulkan via `nvidia-container-toolkit`; X11/Wayland socket passthrough documented |

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────┐
│                   ROS2 DDS Layer                    │
└────────────────────────┬────────────────────────────┘
                         │ r2r subscriptions (one stream per topic)
┌────────────────────────▼────────────────────────────┐
│              Message Ingestion Layer                │
│  - Polled topic discovery (rcl, ~1 Hz)              │
│  - Type-string filtering, per-kind subscriber spawn │
│  - TF tree maintenance                              │
│  - Per-message decode → SceneEntity write           │
└────────────────────────┬────────────────────────────┘
                         │ Arc<RwLock<SceneGraph>> (brief write locks)
┌────────────────────────▼────────────────────────────┐
│                  Scene Graph                        │
│  - Frame-relative entity positions                  │
│  - Retained CPU-side primitive data                 │
│  - revision() counter drives pass-level caching     │
└────────────────────────┬────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────┐
│              wgpu Render Pipeline                   │
│  - Reference grid pass                              │
│  - Mesh / occupancy passes                          │
│  - Line / arrow passes                              │
│  - Point pass (instanced, GPU-side entity xform)    │
│  - egui pass (panels, topic list, config)           │
└─────────────────────────────────────────────────────┘
```

Key design principle: **the render thread never touches ROS2.** The r2r executor
runs on a dedicated thread; communication with the renderer is via
`Arc<RwLock<SceneGraph>>` with brief, write-only locks. The UI never blocks on
DDS, and rendering is always smooth.

---

## Milestones (high level)

| # | Focus | Status |
|---|---|---|
| 0 | Core renderer (no ROS2) | ✅ done |
| 0.5 | 2D nav suite + PointCloud2 | 🟡 in progress — Step I (URDF) remaining |
| 1 | Camera feeds & MarkerArray | ⏳ |
| 2 | Time-series & IMU | ⏳ |
| 3 | PointCloud2 enhancements (color modes, benchmarking) | ⏳ — basic ingestion was pulled forward into M0.5 |
| 4 | Plugin system & custom messages | ⏳ |
| 5 | Polish, MCAP record/playback, community launch | ⏳ |

---

## Guiding Principles

1. **Render thread is pure** — it only reads from the scene graph. It never touches ROS2, never blocks on I/O.
2. **Scene primitives are ROS-agnostic** — the renderer knows nothing about ROS message types. It knows about points, lines, polygons, meshes, textures.
3. **Brief lock handoff** — ROS ingestion holds `RwLock` write only long enough to swap an entity's primitive in. The render loop holds it read-only.
4. **Incremental updates** — `SceneGraph::revision()` bumps on every mutation; render passes cache by revision so an idle scene costs ~zero CPU.
5. **Immediate-mode UI** — egui renders on top of the 3D scene every frame.

---

## Milestone 0 — Core Renderer ✅

**Status:** complete. `cargo run -p app -- --mock` renders an animated scene
exercising every primitive type. This section documents the breakdown for
historical reference and onboarding.

### 0.1 — Cargo workspace skeleton ✅

- Workspace crates: `app`, `renderer`, `scene`, `mock_injector` (and now `ros_node` from M0.5).
- Pinned versions: `wgpu = "22"`, `winit = "0.30"`, `egui = "0.28"`, `egui-wgpu = "0.28"`, `glam`, `bytemuck`, `crossbeam-channel`.
- `clippy`, `rustfmt`, `.github/workflows/ci.yml` in place.
- Top-level `Dockerfile` (release image) + `.devcontainer/` (M0.5+ workflow).

### 0.2 — Scene primitives ✅

The `scene` crate defines the canonical set of primitives. ROS message types
map *onto* these; they are never passed to the renderer directly.

```rust
pub struct Point     { pub position: Vec3, pub color: Color, pub size: f32 }
pub struct Polyline  { pub points: Vec<Vec3>, pub color: Color, pub width: f32 }
pub struct Arrow     { pub origin: Vec3, pub direction: Vec3, pub length: f32,
                       pub shaft_radius: f32, pub head_radius: f32, pub color: Color }
pub struct Grid      { pub origin: Vec2, pub cell_size: f32, pub cols: u32,
                       pub rows: u32, pub data: GridData }
pub enum   GridData  { Uniform(Color), Cells(Vec<u8>, Colormap) }
pub enum   Colormap  { OccupancyDefault, Grayscale, Inferno, Custom(Vec<Color>) }
pub struct Mesh      { pub vertices: Vec<Vertex>, pub indices: Vec<u32>, pub material: Material }
pub struct Material  { pub base_color: Color, pub texture: Option<TextureHandle>, pub wireframe: bool }
pub struct Label     { pub position: Vec3, pub text: String, pub color: Color, pub scale: f32 }
pub struct Frame     { pub transform: Mat4, pub axis_length: f32, pub label: Option<String> }
```

```rust
pub struct SceneGraph {
    pub entities: HashMap<EntityId, SceneEntity>,
    pub reference_frame: String,
    // bumps on every upsert / update_primitive / update_transform / set_visible / remove
    revision: AtomicU64,
}

pub struct SceneEntity {
    pub id: EntityId,
    pub label: Option<String>,
    pub transform: Mat4,           // world transform at time of last update
    pub primitive: ScenePrimitive,
    pub visible: bool,
    pub dirty: bool,
}

pub enum ScenePrimitive {
    Points(Vec<Point>), Polyline(Polyline), Arrows(Vec<Arrow>),
    Grid(Grid), Mesh(Mesh), Labels(Vec<Label>), Frame(Frame),
}
```

Design notes:
- `EntityId` is a `u64`; callers (ROS subscribers) own the ID so they can update in place rather than re-inserting.
- `dirty` is set by the writer, cleared by the renderer after GPU upload.
- Behind `Arc<RwLock<SceneGraph>>` — write-locked only briefly during ingestion, read-locked during rendering.

### 0.3 — wgpu init + window ✅

- `winit` 0.30 `ApplicationHandler`; wgpu instance/adapter (prefer Vulkan on Linux) → device/queue.
- Surface + depth buffer (f32). Resize-on-`WindowEvent::Resized`.
- Frame timing recorded for the FPS display.

### 0.4 — Orbit camera ✅

```rust
pub struct OrbitCamera {
    pub target: Vec3, pub yaw: f32, pub pitch: f32, pub distance: f32,
    pub fov_y: f32, pub near: f32, pub far: f32,
}
```

Input: LMB drag → orbit, RMB drag → pan, scroll → zoom, `F`/`T`/`S` → reset/top/side, `Esc` → quit.
Camera bind group is shared across all 3D pipelines (group 0).

### 0.5 — Render passes ✅

Per-frame pass order:
```
reference_grid → mesh → occupancy → line → arrow → point → egui
```

All 3D passes share the camera bind group:
```wgsl
@group(0) @binding(0) var<uniform> camera: CameraUniform;
```

Each pass is a struct with `prepare()` and `draw()` methods. The empty scene
must not panic.

### 0.6 — egui integration ✅

```
┌──────────────────────────────────────────────┐
│  [Toolbar: FPS, camera reset, view presets]  │
├────────────────┬─────────────────────────────┤
│  Entity list   │                             │
│  ☑ visible     │    3D Viewport              │
│  ☑ visible     │                             │
└────────────────┴─────────────────────────────┘
```

- `egui-wgpu` renders into the swapchain after 3D passes.
- Mouse routing: egui captures when over a panel; 3D viewport otherwise.
- Per-entity visibility goes through `SceneGraph::set_visible` so the
  revision counter bumps (direct field assignment would silently bypass the
  pass cache — see Step H.2).

### 0.7 — Mock data injector ✅

`mock_injector` populates the scene without ROS2: animated arrow, static
occupancy grid, figure-8 polyline, two reference frames, a small mesh, 500
hemisphere points. Layered on top of ROS subscribers when both are active
(see Step G — `--mock` is opt-in but composable).

---

## Milestone 0.5 — First ROS2 Node: 2D Navigation Suite (in progress)

**Goal:** subscribe to the six core 2D-nav data types plus PointCloud2,
transform everything via TF, write into the scene graph from a dedicated
ROS2 thread.

**Crate added:** `ros_node` — depends on `r2r 0.9` only.

**Exit criterion:** all seven data types visualized correctly in a real
Nav2 stack (simulated or physical). TF transforms applied correctly. UI
remains at 60fps under all listed topic rates.

### Status (2026-05-09)

| Step | What | Status |
|---|---|---|
| A (0.5.1) | Crate skeleton + `RosNode::spawn` thread + clean shutdown | ✅ |
| B (0.5.3) | OccupancyGrid (`/map`) → `Grid` (no TF) | ✅ |
| C (0.5.2) | TF tree + reframe to `map` | ✅ |
| D (0.5.4) | PoseStamped + PoseArray → Arrow(s) | ✅ |
| E (0.5.5) | Path → Polyline | ✅ |
| F (0.5.6) | LaserScan → Points | ✅ |
| F.5 (added) | TOML config + polled topic discovery + per-topic QoS | ✅ |
| G (added) | CLI flip: ROS always on, `--mock` opt-in | ✅ |
| H (pulled from M3) | PointCloud2 → Points | ✅ |
| H.1 (added) | PC2 received-vs-displayed counter | ✅ |
| H.2 (added) | PointPass perf: revision cache, GPU-side transform, buffer reuse | ✅ |
| I (0.5.7) | URDF + JointState → meshes + FK | ⏳ next |

For per-step notes (smoke tests, measured numbers, decisions) see
[MBABYSTEPS_1.md](MBABYSTEPS_1.md). The headline summary follows.

### Decisions log (do not relitigate)

| Decision | Choice | Reason |
|---|---|---|
| Rust→ROS2 binding | **`r2r`** (only) | Pure-cargo build flow; mature for sub/pub; sufficient for the seven M0.5 message types. `rclrs` is deferred until dynamic-message needs surface. |
| ROS2 distro | **Jazzy** (24.04) | Devcontainer base; long-term support; `nav_msgs`/`sensor_msgs` packaged. |
| Dev environment | **devcontainer** | Fedora host has no `/opt/ros`; the container gives Jazzy + apt-installed message packages. NVIDIA Vulkan via `nvidia-container-toolkit` + `NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute`. |
| Workspace path inside container | `/workspaces/fastviz` | devcontainer convention; `~/.claude` is mounted so conversation history survives. |
| Threading | Render loop on main thread; r2r executor on a dedicated thread; shared scene via `Arc<RwLock<SceneGraph>>`. | Consistent with the architecture above. |
| ROS env at build time | `LIBCLANG_PATH=/usr/lib/x86_64-linux-gnu` exported in `~/.bashrc`. | r2r's `bindgen` step needs libclang; without this it fails on first build. |
| Discovery model | **Polled** at ~1 Hz via `node.get_topic_names_and_types()`. | r2r 0.9 doesn't expose the rcl graph guard condition. Going around r2r to DDS (`rustdds`/`cyclonedds-rs`) gives reactive discovery but doubles the DDS participant + serde pipeline. Polling rcl costs ~nothing. |
| Coordinate convention | `ROS_TO_WORLD: Mat4` swaps ROS (x,y,z) → world (x,z,y). It's a reflection (det = −1) — accepted because M0.5 only consumes orientation as visual indicators. | Renderer is Y-up; ROS is Z-up. |
| TF interpolation | Skipped — latest transform per frame only. | Add only if jitter is visible. None observed so far. |

### 0.5.1 — Node infrastructure (Step A) ✅

```rust
// lib.rs
pub use config::RosConfig;
pub use node::{RosNode, RosNodeHandle};

// node.rs
pub struct RosNode {
    handle: std::thread::JoinHandle<()>,
    shutdown: crossbeam_channel::Sender<()>,
}

impl RosNode {
    pub fn spawn(scene: scene::SceneHandle, cfg: RosConfig) -> anyhow::Result<Self> { ... }
    pub fn shutdown(self) { ... }
}
```

- Crate `crates/ros_node/` with `lib.rs`, `node.rs`, `config.rs`, `ids.rs`.
- Workspace deps: `r2r = "0.9"`, `futures = "0.3"`.
- `app` has a `ros` cargo feature (default-on); on hosts without ROS sourced, `cargo build --no-default-features --features mock` still works.
- `RosNode` owns the executor thread; its `Drop` impl signals shutdown and joins. r2r consumes subscriptions as async streams, so we spin a `futures::executor::LocalPool` between `node.spin_once` calls.
- **Acceptance:** `cargo run -p app -- --no-mock` → `/fastviz` visible in `ros2 node list` within ~1 s; `kill -INT` shuts down cleanly.

### 0.5.2 — TF tree (Step C) ✅

```rust
pub struct TransformEntry { pub parent: String, pub xform: glam::Mat4, pub stamp_ns: i64 }
pub struct TfTree { pub frames: parking_lot::RwLock<HashMap<String, TransformEntry>> }

impl TfTree {
    pub fn update(&self, msg: &r2r::tf2_msgs::msg::TFMessage);
    pub fn lookup(&self, target: &str, source: &str) -> Option<glam::Mat4>;
}
```

- `lookup` walks each frame to its root and returns `(root_T_target)^-1 * root_T_source`, returning `None` for disconnected trees / cycles. 5 unit tests pass: identity, translation chain, inverse direction, disconnected, overwrite.
- `crates/ros_node/src/subscribers/tf.rs` — subscribes `/tf` (volatile, depth 100) and `/tf_static` (transient_local, depth 100). Both feed the same `TfTree`.
- `crates/ros_node/src/coords.rs` — `ROS_TO_WORLD` conversion. Subscribers compose `entity.transform = ROS_TO_WORLD * tf_in_ros * primitive_pose_in_ros * ROS_TO_WORLD` so renderer-local-XZ-plane primitives end up correctly placed.
- Subscribers fall back to identity + a one-time warn if the TF lookup fails.

### 0.5.3 — OccupancyGrid (Step B) ✅

- Subscriber: `crates/ros_node/src/subscribers/occupancy.rs`.
- Per message: `data: Vec<i8>` → `Vec<u8>` mapping `-1 → 255, 0 → 0, 100 → 100` (matches `Colormap::OccupancyDefault`).
- Renderer-quirk: ROS sends `info.origin.position` as the **lower-left corner**; fastviz `Grid.origin` is the **center** — shift by half the map extent.
- Stamp-based dedup: skip re-uploads when `header.stamp` is unchanged.
- After Step C the grid is encoded in its own local frame (lower-left at origin, cells extending +X/+Y); all placement (`info.origin` pose + TF + coord swap) lives in `entity.transform`.
- QoS uses `DurabilityPolicy::BestAvailable` so the subscriber connects to either a latched (`map_server`) or volatile (`ros2 topic pub`) publisher.

### 0.5.4 — Poses (Step D) ✅

- Subscriber: `crates/ros_node/src/subscribers/pose.rs`. Handles both `geometry_msgs/PoseStamped` (one Arrow per topic) and `geometry_msgs/PoseArray` (one entity holding many Arrows).
- Each pose is composed `pose_in_world = ROS_TO_WORLD * tf_lookup(ref, frame_id) * pose_local`.
- Arrow's `origin = pose_in_world.translation`, `direction = pose_in_world * X̂` (REP-103: arrow points along the body's +X / forward axis).

### 0.5.5 — Path (Step E) ✅

- Subscriber: `crates/ros_node/src/subscribers/path.rs`. Single TF lookup per message (uses parent `header.frame_id`; per-pose headers ignored).

### 0.5.6 — LaserScan (Step F) ✅

- Subscriber: `crates/ros_node/src/subscribers/laserscan.rs`.
- QoS: `QosProfile::sensor_data()` (best_effort, depth 5).
- Polar→Cartesian on CPU into the laser's own frame (Z=0). The `tf_lookup` and `ROS_TO_WORLD` are folded into `entity.transform`; per-point math is one `cos`/`sin` per range — no per-point matrix multiply. No conjugation here (cf. occupancy, which conjugates because its primitive lives in renderer-XZ-plane local space).
- `Vec<Point>` buffer is reused across iterations.
- Range filter: skip non-finite values, skip below `range_min`, skip above `range_max`.

### 0.5.F.5 — Config + polled discovery + per-topic QoS ✅ *(added during M0.5)*

**a. TOML config file.** `RosConfig::from_path(p)` loads via a `RawConfig`
schema (TOML 1:1) that converts to runtime `RosConfig`. Missing fields fall
back to defaults; top-level unknown keys are rejected
(`#[serde(deny_unknown_fields)]`). New CLI flag `--config <path>`. Repo-root
[fastviz.toml](fastviz.toml) mirrors `RosConfig::default()` and is the docs
source of truth. CLI `--ref-frame` wins over the config value.

**b. Per-topic QoS overrides.** `QosOverride { reliability, durability, depth }`
with optional fields and `apply(base) -> r2r::QosProfile`. Recognised values:
`"reliable"` / `"best_effort"`; `"volatile"` / `"transient_local"` / `"best_available"`;
any positive depth. Each kind's section may contain `[<kind>.qos."<topic>"]` tables.

**c. Polled discovery + bare `"*"` wildcard.**
[discovery.rs](crates/ros_node/src/subscribers/discovery.rs) owns a `Registry`
tracking already-spawned topics and per-kind index counters. `bootstrap` spawns
concrete topics from config at startup; `tick` runs every 50 spin_once cycles
(~1 Hz) calling `node.get_topic_names_and_types()`, filtering by expected
message-type strings, spawning subscribers for new matches. Each subscriber
module exposes `MSG_TYPE: &str` + `pub fn spawn_topic`. Wildcards work for
poses/pose_arrays/paths/scans/points; the map subscriber is single-topic
(wildcard `"*"` rejected with one warn at bootstrap).

**Out of scope** (deferred): regex/glob beyond bare `"*"`; egui sidebar to add
or remove at runtime; subscriber teardown on topic-disappear; wildcards on the
map subscriber.

### 0.5.G — CLI flip ✅ *(added during M0.5)*

- `--ros` and `--no-ros` are gone. The ROS2 node always starts (this *is* a
  ROS2 visualizer). The `ros` cargo feature is still default-on so the gate
  exists for hosts without ROS sourced, but no CLI flag toggles it.
- `--mock` is opt-in (default = off). `--no-mock` is kept as an explicit
  override but is the default behaviour. The mock injector is layered on top
  of ROS subscribers when present, so `--mock` means "test the rendering
  pipeline without a live ROS graph or in addition to one."

### 0.5.H — PointCloud2 → Points ✅ *(pulled from M3)*

- Subscriber: `crates/ros_node/src/subscribers/pointcloud.rs`.
- Parses `fields[]` once per message to locate `x`, `y`, `z` (FLOAT32 or FLOAT64; mixed types rejected). Walks `data` at `point_step` stride; non-finite values dropped. Big-endian buffers rejected with a one-line warn.
- Decimation via `style.stride` (config: `[points] style = { stride = N }`); `stride = 1` is the default.
- TF lookup folded into `entity.transform` — per-point math is three scalar reads.
- QoS = `sensor_data` (best_effort, depth 5).
- Entity IDs from `pointcloud_id(idx)` (2400+).
- **Intentional non-goals (deferred to M3):** intensity- or RGB-packed coloring; `is_dense=false` density-stats reporting; big-endian; struct-of-arrays format support; per-cloud aggregation history.

### 0.5.H.1 — PC2 throughput counter ✅

- New module [stats.rs](crates/ros_node/src/stats.rs): `RosStats { pc2_received: AtomicU64 }`. Owned by `RosNode` as `Arc<RosStats>`, exposed via `RosNode::stats()`.
- `App` keeps `pc2_last_seen_received: u64` and `pc2_displayed: u64`. Each `draw()` loads the atomic; if it advanced, exactly one of the new messages reached the screen — bump `pc2_displayed` by 1 regardless of how many were received in the interval (the rest were overwritten).
- egui toolbar shows `PC2 D/R drop X%`. On `App::drop` and every 5 s during steady state, `log::info!("PC2 throughput: received=…, displayed=…, dropped=… (X%)")`.

### 0.5.H.2 — PointPass performance ✅

Five layered wins:

- **Revision-cached `prepare`**: [`SceneGraph`](crates/scene/src/graph.rs) gained a monotonic `revision()` counter that bumps on every mutation. [`PointPass::prepare`](crates/renderer/src/passes/point.rs) caches `last_revision`; if unchanged, only the screen UBO is refreshed on viewport change. Steady-state redraws against an unchanging scene skip the per-point repack and the GPU upload entirely.
- **Per-entity transform via uniform**: per-point CPU `Mat4 * Vec4` is gone. Each visible Points entity gets a slot in a dynamic-offset uniform buffer; the vertex shader applies `entity.transform` before `view_proj`. Per-instance position is now in entity-local coords, so the same instance bytes survive across redraws even if the entity moves.
- **Reused `Vec<PointInstance>`**: `instances` is a `PointPass` field, `clear()`-ed before refill instead of allocated each frame.
- **`mem::take` in subscribers**: pointcloud and laserscan subscribers move the local `Vec<Point>` into the new `SceneEntity` instead of cloning. The 5 MB memcpy per PC2 message is gone.
- **UI visibility goes through `set_visible`**: the entity-list checkbox now routes through `SceneGraph::set_visible` so the revision bumps. Direct `entity.visible = …` would silently bypass the cache.

**Measured impact** (indoor_easy_01 1:1 loop, container-headless, Intel UHD
RPL-S iGPU + Vulkan, 161k–171k pts/cloud):

| | Before H.2 | After H.2 |
|---|---|---|
| 24 s sample | received=580, dropped=15 (**2.6%**) | (instantaneous) |
| 150 s sample | — | received=4018, dropped=14 (**0.3%**) |

Drops only appear under occasional renderer stalls (Wayland surface
reconfigure, etc.); steady-state drop rate is ~0.3% on integrated graphics.
Discrete-GPU runs are expected to drop to 0%.

### 0.5.7 / 0.5.I — URDF + JointState ⏳ *(next)*

**Input:** URDF file path (not a topic — loaded from disk, or eventually from
the `robot_description` parameter).

**Subscription:** `sensor_msgs/JointState` (for animation).

**Loading pipeline:**
```
URDF file
  → parse with urdf-rs
  → for each link: load mesh (STL via stl_io / OBJ via tobj; DAE deferred)
  → resolve `package://` paths via AMENT_PREFIX_PATH
  → build link tree
  → register each link as a SceneEntity whose transform is updated by FK
```

**Animation:**
- `JointState` callback updates joint angles in the URDF link tree.
- Recompute forward kinematics → new transforms per link.
- `scene.update_transform(entity_id, world_xform)` — no GPU re-upload (mesh data unchanged).

**Tasks:**
- [ ] `urdf-rs` integration in `ros_node/src/urdf.rs`.
- [ ] STL loader (binary STL → `Vec<Vertex>`).
- [ ] OBJ loader (simple wavefront, no MTL required initially).
- [ ] `package://` → filesystem path resolution via `AMENT_PREFIX_PATH`.
- [ ] Forward kinematics from joint angles.
- [ ] `JointState` subscriber → FK → scene entity transform updates.
- [ ] CLI arg `--urdf /path/to/robot.urdf`. (Eventually load from `robot_description` param.)
- [ ] egui panel: show/hide per link; toggle visual vs. collision meshes.

**Smoke test plan:**
```bash
cargo run -p app -- --urdf /opt/ros/jazzy/share/turtlebot3_description/urdf/turtlebot3_burger.urdf
# wiggle joints with: ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

---

## Conventions

### EntityId allocation

| Range | Owner |
|---|---|
| `1..=999` | mock injector (M0) |
| `1000..=1999` | ROS singletons (TF frames hash into this; occupancy = 1000) |
| `2000..=2999` | per-topic poses/paths/scans/pointclouds (allocated sequentially as topics arrive) |
| `3000..=3999` | URDF link entities (M0.5 Step I) |

### Reference frame

`SceneGraph::reference_frame` (default `"map"`) is the target for all TF lookups.
Configurable via CLI (`--ref-frame`) and the `reference_frame` key in
`fastviz.toml`. CLI wins.

### Visibility and labels

Every entity is created with `with_label(topic_name_or_link_name)` so the entity
list panel is meaningful. Visibility toggles route through
`SceneGraph::set_visible` (revision-bumping); direct field assignment is a bug.

---

## Things explicitly out of scope for M0.5

- `MarkerArray`, `Image`, `Imu`, `Odometry` → milestones 1–2.
- MCAP record/playback → M5.
- Plugin system → M4.
- `rclrs` work.
- Wayland-native windowing.
- TF interpolation (added only if jitter is visible).

---

## Future Milestones

### Milestone 1 — Camera Feeds & MarkerArray

- [ ] `sensor_msgs/Image` and `sensor_msgs/CompressedImage` — 2D panel, GPU texture upload.
- [ ] Camera info overlay (topic, hz, resolution).
- [ ] `visualization_msgs/MarkerArray` — arrow, cube, sphere, line strip, text label, mesh resource.
- [ ] Costmap2D overlay (second `OccupancyGrid` layer with separate colormap).
- [ ] Topic discovery UI: browse and subscribe to any topic from within the app.

**Deliverable:** full sensor suite visible; Nav2 costmaps renderable; arbitrary debug markers supported.

### Milestone 2 — Time Series & IMU

- [ ] Collapsible time-series panel (egui plot).
- [ ] `sensor_msgs/Imu` — orientation display + angular velocity / acceleration graphs.
- [ ] `nav_msgs/Odometry` — pose arrow + covariance ellipse.
- [ ] `sensor_msgs/BatteryState` and arbitrary `std_msgs/Float64` topics.
- [ ] Configurable history window, pause/resume, export to CSV.

**Deliverable:** IMU, odometry, and scalar topics plotted live alongside the 3D view.

### Milestone 3 — PointCloud2 enhancements

*Basic PC2 ingestion + GPU pipeline + revision-cached point pass were
completed in M0.5 (Step H / H.1 / H.2). This milestone is the deferred work.*

- [ ] Color modes: flat, intensity, height, ring, time.
- [ ] Intensity- and RGB-packed coloring (deferred from H).
- [ ] `is_dense=false` density-stats reporting.
- [ ] Big-endian buffer support.
- [ ] Struct-of-arrays format support.
- [ ] Per-cloud aggregation history.
- [ ] GPU memory usage indicator.
- [ ] Benchmark vs RViz2 published in README — target 60fps for 100k+ points on a mid-range GPU.

### Milestone 4 — Plugin System & Custom Messages

- [ ] `Visualizer` trait: `subscribe()`, `update_scene()`, `draw_egui_panel()`.
- [ ] Dynamic library plugin loading (`.so`, source-compatible ABI).
- [ ] Plugin registry and discovery via config file.
- [ ] Example plugin: custom message type visualizer.
- [ ] Plugin authoring guide in `docs/plugin_guide.md`.

**Deliverable:** third parties can add visualizers without forking core. Tutorial completable in under 1 hour.

### Milestone 5 — Polish & Community Launch

- [ ] Layout save/load — panel configuration persisted to TOML.
- [ ] Topic search and filter in entity list.
- [ ] Recording to MCAP (rosbag2-compatible).
- [ ] Playback of MCAP files — offline mode, no ROS2 needed.
- [ ] Dark/light theme toggle.
- [ ] User-facing error messages (topic type mismatch, TF lookup failure, etc.).
- [ ] Installation docs: `cargo install`, apt PPA, Docker.
- [ ] Demo video / GIF for README.
- [ ] License: Apache 2.0.

---

## Docker Strategy

Three container scenarios should work:

1. **Tool runs on host, ROS2 in container** — standard DDS discovery, no special config.
2. **Tool runs in container alongside ROS2** — compose file with shared network.
3. **Tool runs in container, display on host** — X11 socket mount (`-v /tmp/.X11-unix:/tmp/.X11-unix`) or Wayland (`WAYLAND_DISPLAY` passthrough).

The repo today ships:
- A top-level release `Dockerfile` (M0 path; `--mock`).
- `.devcontainer/` (Ubuntu 24.04 + Jazzy, with r2r build deps, Vulkan loader, and the host's `~/.claude` mounted).

NVIDIA Vulkan acceleration on Fedora hosts requires
`nvidia-container-toolkit` + `nvidia-ctk runtime configure --runtime=docker`.
Without it, remove `--gpus=all` from `.devcontainer/devcontainer.json` —
Mesa llvmpipe is the fallback.

---

## Repository Structure (current)

```
fastviz/
├── Cargo.toml                  # workspace
├── crates/
│   ├── app/                    # main binary, window, event loop, egui shell, CLI
│   ├── renderer/               # wgpu pipelines, render passes, camera
│   ├── scene/                  # scene graph, scene primitives, dirty tracking, revision
│   ├── mock_injector/          # animates 5 primitive types; layered on top of ROS when --mock
│   └── ros_node/               # r2r executor on a dedicated thread; TF tree; per-message subscribers; polled discovery
├── .devcontainer/              # Ubuntu 24.04 + ROS2 Jazzy
├── Dockerfile                  # release image (M0 path)
├── fastviz.toml                # mirror of RosConfig::default()
├── MBABYSTEPS_1.md             # running execution log
├── ros2_visualizer_project_plan.md   # this file
└── README.md
```

Future crates (per-milestone): `plugin_api/`, `builtin_plugins/`, `mcap_io/`.

---

## Open Questions / Decisions to Revisit

1. **Name** — `fastviz` is the working name; revisit at community launch.
2. ~~r2r vs rclrs~~ — settled: r2r only for now. Revisit if dynamic message support is needed (M4 plugins).
3. **Wayland vs X11** — X11 first via winit; native Wayland later.
4. **MCAP vs rosbag2 SQLite** — MCAP is the future, prioritise it (M5).
5. **Plugin ABI stability** — decide early whether plugins are source-compatible or binary-compatible (source is much easier).

---

## Reference Projects (study, not copy)

| Project | What to study |
|---|---|
| Rerun | Point cloud rendering, egui panel layout, wgpu pipeline structure |
| Foxglove Studio | UX patterns, panel system, topic subscription model |
| RViz2 | What to avoid; also marker and display plugin API |
| Lichtblick | Open source Foxglove fork — good for UX reference |
| ROSboard | Minimal ROS2 web viz — study what "no bridge" means in practice |

---

## Success Metrics

- Point cloud rendering at **60fps** for 100k+ points on a mid-range GPU. *(M0.5 H.2 hits 0.3% drop on Intel iGPU at 161k pts/cloud × 14 Hz; M3 should formalise the bench vs RViz2.)*
- **Cold start to first frame** under 2 seconds.
- Works in Docker with **zero extra configuration** beyond X11 socket mount.
- Plugin tutorial completable in **under 1 hour**.
- GitHub stars / Discourse engagement as community signal.

---

*Plan v2.0 — May 2026.*
*Combines the previous `ros2_visualizer_project_plan.md` (vision, architecture,
milestone overview) with `ros2_visualizer_milestone0_detailed.md` (M0 / M0.5
sub-task detail). MBABYSTEPS_1.md remains the running execution log.*
