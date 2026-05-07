# Milestone 0.5 — Babysteps Plan

This is a self-contained execution plan for a fresh Claude Code session
running **inside the dev container**. It captures the decisions made in
the M0→M0.5 hand-off conversation so we don't have to re-debate them.

---

## 0. Progress (as of 2026-05-07)

| Step | What | Status |
|---|---|---|
| A (0.5.1) | Crate skeleton + `RosNode::spawn` thread + clean shutdown | ✅ done |
| B (0.5.3) | OccupancyGrid (`/map`) → `Grid` (no TF) | ✅ done |
| C (0.5.2) | TF tree + reframe to `map` | ✅ done |
| D (0.5.4) | PoseStamped + PoseArray → Arrow(s) | ✅ done |
| E (0.5.5) | Path → Polyline | ✅ done |
| F (0.5.6) | LaserScan → Points | ✅ done |
| F.5 (new) | Config file + polled topic discovery + per-topic QoS | ✅ done |
| G (0.5.7) | URDF + JointState → meshes + FK | ⏳ next |

### Step A notes
- New crate [crates/ros_node/](crates/ros_node/) with `lib.rs`, `node.rs`, `config.rs`, `ids.rs`.
- Workspace-level `r2r = "0.9"` and `futures = "0.3"` added.
- `app` gets a `ros` cargo feature (in default), `--ros` / `--no-ros` / `--ref-frame FRAME` CLI flags.
- `RosNode` owns the executor thread; its `Drop` impl signals shutdown and joins.
- Smoke test: `cargo run -p app -- --ros --no-mock` → `/fastviz` visible in `ros2 node list` within ~1 s; `kill -INT` shuts down cleanly.

### Step F.5 notes (implemented)

**a. TOML config file.**
- New deps: `serde = { version = "1", features = ["derive"] }`, `toml = "0.8"` (workspace-level).
- `RosConfig::from_path(p)` loads via a `RawConfig` schema (TOML 1:1) that converts to runtime `RosConfig`. Missing fields fall back to `RosConfig::default()`. Top-level unknown keys are rejected (`#[serde(deny_unknown_fields)]`).
- New CLI flag `--config <path>` in [app/cli.rs](crates/app/src/cli.rs); main.rs loads it, falls back to defaults on error. CLI `--ref-frame` still wins over the config value.
- New file [fastviz.toml](fastviz.toml) at repo root mirrors `RosConfig::default()` and is now the source of truth for documentation purposes.
- 4 deserialiser unit tests added.

**b. Per-topic QoS overrides.**
- New `QosOverride { reliability, durability, depth }` struct with optional fields and an `apply(base) -> r2r::QosProfile` method (recognises `"reliable"`/`"best_effort"`, `"volatile"`/`"transient_local"`/`"best_available"`, and any positive depth).
- Each kind's section may include `[<kind>.qos."<topic>"]` tables. RosConfig now exposes `map_qos`, `pose_qos`, `pose_array_qos`, `path_qos`, `scan_qos` HashMaps.
- Each subscriber's per-topic spawn function takes an `Option<QosOverride>` and applies it on top of the kind's default profile.
- Smoke test confirmed via `ros2 topic info -v`: `/map` switched to `RELIABLE` + `TRANSIENT_LOCAL`, `/scan` switched to `RELIABLE`.
- 2 QoS unit tests added (parse + apply).

**c. Polled discovery + bare `"*"` wildcard.**
- New module [crates/ros_node/src/subscribers/discovery.rs](crates/ros_node/src/subscribers/discovery.rs) — owns a `Registry` tracking already-spawned topics + per-kind index counters.
- `bootstrap` spawns concrete topics from config at startup. `tick` runs every 50 spin_once cycles (≈ 1 Hz) — calls `node.get_topic_names_and_types()`, filters by expected message-type strings, spawns subscribers for new matches.
- Each subscriber module now exposes a `MSG_TYPE: &str` constant + a `pub fn spawn_topic` (or `spawn_pose_stamped_topic` / `spawn_pose_array_topic` for the dual case). `spawn_all` removed in favour of the registry.
- Map subscriber is single-topic; wildcard `"*"` is rejected with a warning if used in `[map]` (logged once at bootstrap). Wildcards work for poses, pose_arrays, paths, scans.
- Smoke test: started with `[scans] topics = ["*"]` and `[paths] topics = ["*"]`. Then published to `/robot1/scan`, `/robot2/scan`, `/robot1/plan` ad-hoc — all three got auto-subscribed within ~3 s and produced `first message` log lines.

**Why polled, not event-driven.** r2r 0.9 only exposes the rcl snapshot API (`get_topic_names_and_types`); the rcl graph guard condition isn't bound. Going around r2r to the DDS layer (`rustdds`/`cyclonedds-rs`) gives reactive discovery via DDS built-in topics but means a second DDS participant in the process and a separate type-mangling / deserialisation pipeline. Polling rcl at 1 Hz costs ~nothing and stays ROS-correct.

**Out of scope** (deferred): regex/glob beyond bare `"*"`; egui sidebar to add/remove at runtime; subscriber teardown on topic-disappear; wildcards on the map subscriber.

### Step F notes
- New module [crates/ros_node/src/subscribers/laserscan.rs](crates/ros_node/src/subscribers/laserscan.rs).
- QoS = `QosProfile::sensor_data()` (best_effort, depth 5) — matches typical laser drivers.
- Polar→Cartesian on CPU into the laser's own frame (Z=0). The `tf_lookup` and `ROS_TO_WORLD` are folded into `entity.transform`, so the per-point math stays cheap (one `cos`/`sin` per range, no per-point matrix multiply). No conjugation here — points are emitted directly in ROS coords (laser frame), so just `entity.transform = ROS_TO_WORLD * tf` (cf. occupancy which conjugates because its primitive lives in renderer-XZ-plane local space).
- `Vec<Point>` buffer is reused across iterations; `points.reserve` ensures we don't re-grow once it reaches the working size.
- Range filter: skip non-finite values, skip below `range_min`, skip above `range_max`.
- `RosConfig` gained `scan_topics: Vec<String>` (default `["/scan"]`) and `scan_style: ScanStyle` (size, color).
- Entity IDs from `scan_id(idx)` (2300+).
- Smoke test: published 10 ranges including `inf`, `nan`, and `0.05` (below `range_min=0.1`) → log shows `"first message (7/10 ranges valid, frame=base_laser)"` and the expected TF warning fired (since no `base_laser → map` transform was published).

### Step E notes
- New module [crates/ros_node/src/subscribers/path.rs](crates/ros_node/src/subscribers/path.rs) — single TF lookup per message (uses parent `header.frame_id`, ignores per-pose headers — typical case is identical anyway). Each pose's position is mapped through `ROS_TO_WORLD * tf_lookup`.
- `RosConfig` gained `path_topics: Vec<String>` (default `["/plan"]`) and `path_style: PathStyle` (width, color).
- Entity IDs from `path_id(idx)` (2200+).
- Smoke test: published a 4-point `nav_msgs/Path` on `/plan` (frame=map), got `"first message (4 poses, frame=map)"`.

### Step D notes
- New module [crates/ros_node/src/subscribers/pose.rs](crates/ros_node/src/subscribers/pose.rs) — handles both `geometry_msgs/PoseStamped` (one Arrow per topic) and `geometry_msgs/PoseArray` (one entity holding many Arrows).
- Each pose is composed `pose_in_world = ROS_TO_WORLD * tf_lookup(ref, frame_id) * pose_local`. Arrow's `origin = pose_in_world.translation`, `direction = pose_in_world * X̂` (REP-103: arrow points along the body's +X / forward axis).
- `RosConfig` gained `pose_topics: Vec<String>` (default `["/goal_pose"]`), `pose_array_topics: Vec<String>` (default `["/particle_cloud"]`), and `arrow: ArrowStyle` (length, shaft/head radius, color).
- Entity IDs allocated via `pose_id(idx)` (2000+) and `pose_array_id(idx)` (2100+) — see [ids.rs](crates/ros_node/src/ids.rs).
- Smoke test: published a single PoseStamped on `/goal_pose` and a 3-pose PoseArray on `/particle_cloud` (with rotated quaternions). Both subscribers showed `"first message"` log with correct counts and frame.

### Step C notes
- New module [crates/ros_node/src/tf.rs](crates/ros_node/src/tf.rs) — `TfTree { frames: RwLock<HashMap<String, TransformEntry>> }`, `update(&TFMessage)` overwrites entries, `lookup(target, source)` walks each chain to its root and returns `(root_T_target)^-1 * root_T_source` (or `None` for disconnected trees / cycles). 5 unit tests pass: identity, translation chain, inverse direction, disconnected, overwrite.
- New module [crates/ros_node/src/subscribers/tf.rs](crates/ros_node/src/subscribers/tf.rs) — subscribes `/tf` (volatile, depth 100) and `/tf_static` (transient_local, depth 100). Both feed the same `TfTree`.
- New module [crates/ros_node/src/coords.rs](crates/ros_node/src/coords.rs) — `ROS_TO_WORLD: Mat4` swaps ROS (x,y,z) → world (x,z,y). It's a reflection (det = -1); accepted because M0.5 only consumes orientation as visual indicators. Subscribers compose `entity.transform = ROS_TO_WORLD * tf_in_ros * primitive_pose_in_ros * ROS_TO_WORLD` so the renderer's local-XZ-plane primitives end up correctly placed.
- [occupancy.rs](crates/ros_node/src/subscribers/occupancy.rs) refactored: grid is now encoded in its **own** local frame (lower-left at origin, cells extending +X/+Y); all placement (`info.origin` pose + TF + coord swap) lives in `entity.transform`. If the TF lookup fails, falls back to identity and warns once.
- Smoke test: publish `/tf {map → odom translation (1,2,0)}` then `/map` in frame `odom` → log shows `"first message (1 transforms; tree size = 1)"` and `"first message (...frame=odom)"` with no missing-TF warning. `ros2 topic info` confirms all three subs (`/tf`, `/tf_static`, `/map`).

### Step B notes
- `r2r` consumes subscriptions as async streams; ros_node now spins a `futures::executor::LocalPool` between `node.spin_once` calls.
- New module [crates/ros_node/src/subscribers/occupancy.rs](crates/ros_node/src/subscribers/occupancy.rs).
- QoS uses `DurabilityPolicy::BestAvailable` so the subscriber connects to either a latched (`map_server`) or volatile (`ros2 topic pub`) publisher.
- Renderer-quirk reconciled: ROS sends `info.origin.position` as the **lower-left corner**, fastviz `Grid.origin` is the **center** — we shift by half the map extent. ROS XY plane → fastviz XZ plane (Y-up renderer).
- Stamp-based dedup: subscriber skips re-uploads when `header.stamp` is unchanged.
- Orientation handling deferred to Step C; entity transform is identity for now.
- Smoke test: ad-hoc `ros2 topic pub --once /map nav_msgs/msg/OccupancyGrid '{...4x3 grid...}'` → log line `"/map: first message (4x3 cells, 0.100 m/cell, frame=map)"`; `ros2 topic info -v /map` shows `Subscription count: 1, Node name: fastviz`.

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

## 2. Inherited state (what M0 already gave us)

Already committed and verified working in `cargo build --workspace`,
`cargo test --workspace`, `cargo clippy --workspace --all-targets`:

```
crates/
├── app/                # binary; winit 0.30 ApplicationHandler; egui shell
├── renderer/           # wgpu 22; OrbitCamera; 6 render passes
│   └── passes/         # reference_grid, line, arrow, mesh, point, occupancy
├── scene/              # SceneGraph behind Arc<RwLock<>>; 7 ScenePrimitive variants
└── mock_injector/      # animates 5 primitive types; will be retained for --mock
```

Public types the ROS node will lean on (no API changes expected):

- `scene::SceneGraph::{upsert, update_primitive, update_transform, set_visible, remove}`
- `scene::ScenePrimitive::{Points, Polyline, Arrows, Grid, Mesh, Labels, Frame}`
- `scene::EntityId(u64)`
- `scene::primitives::{Arrow, Polyline, Grid, GridData, Frame, Mesh, Point}`
- `scene::Color`, `scene::Colormap`

The mock injector reserves IDs 1..=8. The ROS node should namespace its
IDs above 1000 (see §6.1 ID allocation table).

---

## 3. New crate: `ros_node`

### 3.1 Workspace changes

Add to `Cargo.toml` workspace members and dependencies:

```toml
# Cargo.toml (top level)
[workspace]
members = ["crates/app", "crates/renderer", "crates/scene", "crates/mock_injector", "crates/ros_node"]

[workspace.dependencies]
# ...existing entries...
r2r = "0.9"
crossbeam-channel = "0.5"
toml = "0.8"
serde = { version = "1", features = ["derive"] }
urdf-rs = "0.8"
stl_io = "0.8"          # binary STL loader
tobj = "4"              # OBJ loader (no MTL needed for M0.5)
```

### 3.2 Crate skeleton

```
crates/ros_node/
├── Cargo.toml
└── src/
    ├── lib.rs              # pub struct RosNode + spawn(SceneHandle, Config)
    ├── config.rs           # serde struct: topics, frames, colors, sizes
    ├── node.rs             # r2r::Context/Node, dedicated executor thread, shutdown
    ├── tf.rs               # TfTree { frames: HashMap<String, TransformEntry> }
    ├── ids.rs              # const ROS_ID_BASE: u64 = 1000; helpers per topic
    └── subscribers/
        ├── mod.rs
        ├── occupancy.rs    # nav_msgs/OccupancyGrid    → Grid
        ├── pose.rs         # geometry_msgs/PoseStamped & PoseArray → Arrow(s)
        ├── path.rs         # nav_msgs/Path             → Polyline
        ├── laserscan.rs    # sensor_msgs/LaserScan     → Points
        └── jointstate.rs   # sensor_msgs/JointState    → mesh transforms (URDF FK)
    └── urdf.rs             # parse URDF, load STL/OBJ, FK
```

`crates/ros_node/Cargo.toml`:

```toml
[package]
name = "ros_node"
version = "0.1.0"
edition = "2021"

[dependencies]
scene = { path = "../scene" }
r2r.workspace = true
crossbeam-channel.workspace = true
parking_lot.workspace = true
glam.workspace = true
anyhow.workspace = true
log.workspace = true
serde.workspace = true
toml.workspace = true
urdf-rs.workspace = true
stl_io.workspace = true
tobj.workspace = true
```

App wiring (`crates/app/Cargo.toml`): add `ros_node = { path = "../ros_node", optional = true }`. Gate behind a `ros` cargo feature so M0's pure-cargo build still works on hosts without ROS2 sourced. Default features = `["ros"]` inside the container.

---

## 4. Suggested attack order

The project plan lists 0.5.1 → 0.5.7. The smartest *implementation* order is slightly different — it lets us see something on screen by the end of step 2:

| Step | What | Why this order |
|---|---|---|
| **A. 0.5.1** | Crate skeleton + `RosNode::spawn` thread + clean shutdown | Foundation for everything else. End state: `cargo run -p app -- --ros` opens window, ROS2 thread is spinning, no subscribers yet. |
| **B. 0.5.3** | OccupancyGrid (`/map`) — render in *its own frame* (no TF yet) | Simplest message → primitive mapping, biggest visual reward. Validates the SceneGraph write path from another thread. Use `ros2 launch nav2_bringup tb3_simulation_launch.py` or just `ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=...`. |
| **C. 0.5.2** | TF tree + reframe everything to `map` | Now we have a real reason for TF (the next data types need it). Build `TfTree` with insert/lookup; subscribe `/tf`+`/tf_static`; transform the OccupancyGrid into the reference frame. |
| **D. 0.5.4** | PoseStamped + PoseArray → Arrow(s) | Easy primitive mapping, exercises TF lookup per pose. |
| **E. 0.5.5** | Path → Polyline | Trivial after Pose. |
| **F. 0.5.6** | LaserScan → Points | Polar→Cartesian on CPU, then a TF apply. Pre-allocate the GPU buffer. |
| **G. 0.5.7** | URDF + JointState | Biggest sub-task. Load STL/OBJ once, then per-`JointState` recompute FK and update transforms only (no mesh re-upload). |

After each step run `cargo test --workspace && cargo clippy --workspace --all-targets` and a manual smoke run.

---

## 5. Sub-task detail

### 5.1 — Node infrastructure (`ros_node/src/node.rs`, `lib.rs`)

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
    pub fn spawn(scene: scene::SceneHandle, cfg: RosConfig) -> anyhow::Result<Self> {
        let (tx, rx) = crossbeam_channel::bounded(1);
        let handle = std::thread::Builder::new()
            .name("ros2-executor".into())
            .spawn(move || run_node(scene, cfg, rx))?;
        Ok(Self { handle, shutdown: tx })
    }
    pub fn shutdown(self) { let _ = self.shutdown.send(()); let _ = self.handle.join(); }
}

fn run_node(scene: scene::SceneHandle, cfg: RosConfig, shutdown: crossbeam_channel::Receiver<()>) {
    let ctx = r2r::Context::create().unwrap();
    let mut node = r2r::Node::create(ctx, "fastviz", "").unwrap();

    // create subscribers (each of which captures `scene.clone()`)
    let _occ = subscribers::occupancy::spawn(&mut node, scene.clone(), &cfg);
    // ... others ...

    while shutdown.try_recv().is_err() {
        node.spin_once(std::time::Duration::from_millis(20));
    }
}
```

**Acceptance:** with no subscribers wired, `cargo run -p app -- --ros` should result in `ros2 node list` showing `/fastviz`.

### 5.2 — TF tree (`ros_node/src/tf.rs`)

Data shape:
```rust
pub struct TransformEntry { pub parent: String, pub xform: glam::Mat4, pub stamp_ns: i64 }
pub struct TfTree { pub frames: parking_lot::RwLock<HashMap<String, TransformEntry>> }

impl TfTree {
    pub fn update(&self, msg: &r2r::tf2_msgs::msg::TFMessage);
    pub fn lookup(&self, target: &str, source: &str) -> Option<glam::Mat4>;
}
```

`lookup` walks each frame to its root (or until they meet) — straight matrix chain composition. **Skip interpolation in M0.5.1**; just use the latest transform per frame. Add interpolation only if visible jitter is observed.

Subscribers for `/tf` and `/tf_static` both call `TfTree::update`. `/tf_static` uses transient_local QoS (latched).

### 5.3 — OccupancyGrid (`ros_node/src/subscribers/occupancy.rs`)

Per-message:
1. Convert `data: Vec<i8>` → `Vec<u8>` mapping `-1 → 255, 0 → 0, 100 → 100` (matches `Colormap::OccupancyDefault` in scene).
2. Build `scene::Grid { origin: Vec2::new(info.origin.position.x, info.origin.position.y), cell_size: info.resolution, cols: info.width, rows: info.height, data: GridData::Cells(bytes, Colormap::OccupancyDefault) }`.
3. The pose orientation in `info.origin.orientation` becomes the entity's transform.
4. `scene.write().upsert(ROS_ID_MAP, ScenePrimitive::Grid(grid)).with_label("/map")`.

Dirty tracking: only re-upload if `header.stamp` is new — store last_stamp in the subscriber's closure state.

### 5.4 — Poses

`PoseStamped` → single `Arrow` at `ROS_ID_POSE_BASE + topic_index`.
`PoseArray` → `Vec<Arrow>` written as `ScenePrimitive::Arrows` (one entity per topic).

Each pose's frame goes through `TfTree::lookup(reference_frame, pose.header.frame_id)` → multiply the pose by the resulting Mat4 before stuffing into the Arrow.

### 5.5 — Path

`Vec<PoseStamped>` → `Polyline` of points connecting positions in order. Single TF lookup per `header.frame_id` per message.

### 5.6 — LaserScan

```rust
let n = msg.ranges.len();
let mut points = Vec::with_capacity(n);
for (i, r) in msg.ranges.iter().enumerate() {
    if !r.is_finite() || *r < msg.range_min || *r > msg.range_max { continue; }
    let theta = msg.angle_min + (i as f32) * msg.angle_increment;
    let p = Vec3::new(r * theta.cos(), r * theta.sin(), 0.0);
    points.push(scene::Point { position: p, color: cfg.scan_color, size: cfg.scan_size });
}
```

Then transform with TF lookup of `header.frame_id`. Use `scene.update_primitive` to avoid re-allocating the entity. The `point` render pass already pre-allocates capacity; we just need to make sure we don't grow the per-frame `Vec` beyond `max_ranges` more than once per session.

### 5.7 — URDF + JointState

`urdf-rs::read_from_file(...)` → `Robot`. For each `Link` with a `Visual.geometry::Mesh`, resolve the mesh path (`package://foo/bar.stl` lookup uses `AMENT_PREFIX_PATH` env). Load with `stl_io` for STL or `tobj` for OBJ, convert to `scene::primitives::Mesh`. Register each link as a `SceneEntity` whose `transform` will be updated by FK.

`JointState` callback:
1. Update internal joint angle map.
2. Walk the URDF kinematic chain → compute world transforms per link (root identity, accumulate joint transforms).
3. For each link, `scene.update_transform(entity_id, world_xform)` — no GPU re-upload (mesh data unchanged).

CLI arg: `--urdf /path/to/robot.urdf`. If provided, load at startup; otherwise skip.

---

## 6. Conventions

### 6.1 EntityId allocation

| Range | Owner |
|---|---|
| `1..=999` | mock injector (M0) |
| `1000..=1999` | ROS singletons (TF frames hash into this, occupancy=1000, etc.) |
| `2000..=2999` | per-topic poses/paths/scans (allocate sequentially as topics arrive) |
| `3000..=3999` | URDF link entities |

### 6.2 Reference frame

`SceneGraph::reference_frame` (currently `"map"` from M0) is the target for all TF lookups. Make it user-configurable from CLI (`--ref-frame`) and the egui sidebar.

### 6.3 Visibility and labels

Every entity should be created with `with_label(topic_name_or_link_name)` so the entity list panel is meaningful.

---

## 7. Verification

Run each as a smoke test inside the container after the matching step:

```bash
# Step A: node up
cargo run -p app -- --ros &
ros2 node list           # expects /fastviz

# Step B: occupancy
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/path/to/empty_world.yaml

# Step D-F: tb3 sim has it all
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
# expect: map quad, robot pose arrow, laser scan dots, planned path polyline

# Step G: URDF
cargo run -p app -- --ros --urdf /opt/ros/jazzy/share/turtlebot3_description/urdf/turtlebot3_burger.urdf
# wiggle joints with: ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

Performance target (project plan): 60fps with all of the above active.

---

## 8. Things explicitly out of scope for M0.5

- `MarkerArray`, `Image`, `PointCloud2`, `Imu`, `Odometry` → milestones 1–4.
- MCAP record/playback → M5.
- Plugin system → M4.
- Any rclrs work.
- Wayland-native windowing.
- TF interpolation (added only if jitter is visible).

---

## 9. First commands inside the container

```bash
# Verify the toolchain
source /opt/ros/jazzy/setup.bash       # bashrc already does this; no-op for sanity
echo $ROS_DISTRO                       # → jazzy
vulkaninfo --summary | head            # → NVIDIA driver + Vulkan 1.3 if --gpus=all worked

# Verify M0 still builds inside the container
cd /workspaces/fastviz
cargo build --workspace
cargo run -p app -- --mock             # window opens via X11

# Then start step A: create the ros_node crate.
```
