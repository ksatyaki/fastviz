# ROS2 Visualizer — Fine-Grained Implementation Plan
*Milestones 0 and 0.5: Renderer foundation + First ROS2 data types*

---

## Scope of This Document

This plan covers the two foundational milestones in detail:

- **Milestone 0** — Core renderer: wgpu pipeline, scene graph, scene primitives, egui shell. No ROS2. Driven by a mock data injector.
- **Milestone 0.5** — First ROS2 node: OccupancyGrid, TF, Poses, Paths, LaserScan 2D, URDF. The point where the tool becomes genuinely useful for 2D robotics.

Later milestones (PointCloud2, images, IMU, plugin system) follow the high-level plan and are not expanded here.

---

## Guiding Principles

1. **Render thread is pure** — it only reads from the scene graph. It never touches ROS2, never blocks on I/O.
2. **Scene primitives are ROS-agnostic** — the renderer knows nothing about ROS message types. It knows about points, lines, polygons, meshes, textures.
3. **Lock-free handoff** — ROS ingestion writes to the scene graph via lock-free channels or double-buffered structures. The render loop reads without locking.
4. **Incremental updates** — scene objects track dirty state. GPU buffers are only re-uploaded when data changes.
5. **Immediate-mode UI** — egui renders on top of the 3D scene every frame. Panel layout is simple and hardcoded initially; made configurable later.

---

## Milestone 0 — Core Renderer (No ROS2)

**Goal:** A window with a functional wgpu renderer, a working scene graph, all primitive types renderable, camera control, and an egui panel. Data comes from a mock injector, not ROS2.

**Exit criterion:** All scene primitives render correctly. Camera works. egui panels are functional. Mock data injector can drive all primitive types. No ROS2 dependency anywhere in this milestone.

---

### 0.1 — Cargo Workspace & Project Skeleton

**Tasks:**
- Create Cargo workspace with the following crates:
  - `app` — binary crate, main loop, window, event handling
  - `renderer` — wgpu pipelines, render passes, camera
  - `scene` — scene graph, scene primitives, dirty tracking
  - `mock_injector` — test harness that populates the scene without ROS2
- Set up `clippy`, `rustfmt`, `.github/workflows/ci.yml`
- Add `Dockerfile` with Ubuntu 24.04 base, X11 passthrough documented in README
- Choose and pin dependency versions:
  - `wgpu = "22"` (or latest stable)
  - `winit = "0.30"`
  - `egui = "0.28"`
  - `egui-wgpu = "0.28"`
  - `glam` for math (vec2/vec3/mat4)
  - `bytemuck` for GPU buffer casting
  - `crossbeam-channel` for ingestion→scene handoff

**Outputs:** `cargo build` succeeds. `cargo run` opens a black window and exits cleanly.

---

### 0.2 — Scene Primitives Definition

The `scene` crate defines the canonical set of primitives the renderer understands. These are the **only** types the render pipeline knows about. ROS message types map *onto* these; they are never passed to the renderer directly.

#### Primitive types

```rust
/// A single colored point in 3D space
pub struct Point {
    pub position: Vec3,
    pub color: Color,
    pub size: f32,
}

/// A polyline — ordered sequence of points connected by line segments
pub struct Polyline {
    pub points: Vec<Vec3>,
    pub color: Color,
    pub width: f32,
}

/// An axis-aligned or arbitrarily oriented arrow (pose indicator)
pub struct Arrow {
    pub origin: Vec3,
    pub direction: Vec3,  // normalized; length encodes scale separately
    pub length: f32,
    pub shaft_radius: f32,
    pub head_radius: f32,
    pub color: Color,
}

/// A flat grid aligned to XY or XZ plane — for ground plane and occupancy cells
pub struct Grid {
    pub origin: Vec2,         // world-space center
    pub cell_size: f32,
    pub cols: u32,
    pub rows: u32,
    pub data: GridData,       // see below
}

pub enum GridData {
    /// Uniform color — used for reference grid
    Uniform(Color),
    /// Per-cell u8 values + colormap — used for OccupancyGrid and costmaps
    Cells(Vec<u8>, Colormap),
}

pub enum Colormap {
    OccupancyDefault,   // white=free, black=occupied, grey=unknown
    Grayscale,
    Inferno,
    Custom(Vec<Color>), // 256-entry LUT
}

/// A triangle mesh — used for URDF links, map overlays
pub struct Mesh {
    pub vertices: Vec<Vertex>,   // position + normal + uv
    pub indices: Vec<u32>,
    pub material: Material,
}

pub struct Material {
    pub base_color: Color,
    pub texture: Option<TextureHandle>,
    pub wireframe: bool,
}

/// A screen-space or world-space text label
pub struct Label {
    pub position: Vec3,
    pub text: String,
    pub color: Color,
    pub scale: f32,
}

/// A coordinate frame indicator (3 colored axes)
pub struct Frame {
    pub transform: Mat4,
    pub axis_length: f32,
    pub label: Option<String>,
}
```

#### Scene graph structure

```rust
pub struct SceneGraph {
    pub entities: HashMap<EntityId, SceneEntity>,
    pub reference_frame: String,
}

pub struct SceneEntity {
    pub id: EntityId,
    pub label: Option<String>,
    pub transform: Mat4,           // world transform at time of last update
    pub primitive: ScenePrimitive,
    pub visible: bool,
    pub dirty: bool,               // true = GPU buffer needs re-upload
}

pub enum ScenePrimitive {
    Points(Vec<Point>),
    Polyline(Polyline),
    Arrows(Vec<Arrow>),
    Grid(Grid),
    Mesh(Mesh),
    Labels(Vec<Label>),
    Frame(Frame),
}
```

**Key design notes:**
- `EntityId` is a `u64` — callers (ROS subscribers) own the ID, so they can update their entity in-place rather than re-inserting.
- `dirty` flag is set by the writer (ingestion layer), cleared by the renderer after GPU upload.
- The scene graph lives behind an `Arc<RwLock<SceneGraph>>` — write-locked only during ingestion updates (brief), read-locked during rendering.

**Tasks:**
- [ ] Define all primitive structs in `scene/src/primitives.rs`
- [ ] Define `SceneGraph`, `SceneEntity`, `ScenePrimitive` in `scene/src/graph.rs`
- [ ] Define `EntityId`, `Color`, `Vertex` types
- [ ] Write unit tests: insert entity, mark dirty, update in-place
- [ ] Document the design contract in `scene/README.md`

---

### 0.3 — wgpu Initialization & Window

**Tasks:**
- Initialize `winit` event loop and window
- Request wgpu instance, adapter (prefer Vulkan on Linux), device, queue
- Create swapchain / surface with correct format
- Set up resize handling — recreate surface on `WindowEvent::Resized`
- Depth buffer texture (f32 depth, standard Z-testing)
- Frame timing: measure and store last frame duration

**Outputs:** Window clears to a dark background each frame at vsync.

---

### 0.4 — Camera System

A standard orbit camera. The camera is a first-class object in the renderer, not embedded in the scene graph.

```rust
pub struct OrbitCamera {
    pub target: Vec3,       // point being orbited
    pub yaw: f32,           // horizontal angle (radians)
    pub pitch: f32,         // vertical angle (radians), clamped ±89°
    pub distance: f32,      // distance from target
    pub fov_y: f32,
    pub near: f32,
    pub far: f32,
}

impl OrbitCamera {
    pub fn view_matrix(&self) -> Mat4 { ... }
    pub fn proj_matrix(&self, aspect: f32) -> Mat4 { ... }
    pub fn view_proj(&self, aspect: f32) -> Mat4 { ... }
}
```

**Input handling:**
- Left mouse drag → orbit (yaw + pitch)
- Right mouse drag → pan (translate target in camera-relative XZ plane)
- Scroll wheel → zoom (change distance)
- `F` key → reset to default view
- `T` key → top-down view (pitch = −90°)
- `S` key → side view

**Uniform buffer:**
```rust
#[repr(C)]
#[derive(bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniform {
    pub view_proj: [[f32; 4]; 4],
    pub camera_pos: [f32; 3],
    pub _pad: f32,
}
```
Uploaded to GPU once per frame if camera moved.

**Tasks:**
- [ ] `OrbitCamera` struct and methods
- [ ] `CameraUniform` and wgpu buffer
- [ ] Input event → camera mutation (in `app` crate)
- [ ] Camera bind group shared across all pipelines

---

### 0.5 — Render Passes & Pipeline Architecture

The renderer executes passes in order each frame:

```
Frame start
  │
  ├─ [Pass 1] Grid pass          — reference ground grid
  ├─ [Pass 2] Mesh pass          — opaque geometry (URDF, map overlays)
  ├─ [Pass 3] Line pass          — polylines, frame axes, path lines
  ├─ [Pass 4] Point pass         — point clouds, pose particles (future)
  ├─ [Pass 5] Arrow pass         — pose arrows, TF axis arrows
  ├─ [Pass 6] egui pass          — UI overlay (transparent, always on top)
  │
Frame present
```

All 3D passes share the camera bind group (group 0). Each pass has its own pipeline, vertex layout, and shader.

**Shared bind group layout (group 0):**
```wgsl
@group(0) @binding(0) var<uniform> camera: CameraUniform;
```

#### Pass specifications

**Grid pass**
- Generates a flat grid of lines on the XY plane
- Configurable cell size and extent
- Two line widths: major (every 1m) and minor (every 0.1m)
- Implemented as a line list (pairs of vertices)
- Color: subtle grey, alpha-blended

**Line pass**
- Input: `Vec<(Vec3, Vec3, Color)>` — line segments
- Geometry: each segment → two vertices
- Shader: simple color passthrough, no lighting
- Used for: polylines, path visualizations, TF tree connectors

**Arrow pass**
- Input: `Vec<Arrow>`
- Geometry: generated on CPU — shaft as a thin box, head as a cone approximated by a triangle fan
- Alternative: billboard-style 2D arrows for flat overhead views
- Used for: pose arrows, velocity vectors

**Mesh pass**
- Input: `Vec<(Mesh, Mat4)>` — mesh + world transform
- Per-mesh push constants or uniform: model matrix
- Simple diffuse lighting with a fixed sun direction
- Supports both filled and wireframe (toggle per mesh)

**Point pass** *(stub in M0, used properly in M3 for LaserScan)*
- Input: `Vec<Point>`
- Geometry: instanced quads (two triangles per point, sized in screen space)
- Used for: LaserScan returns, future point clouds

**Tasks:**
- [ ] Abstract `RenderPass` trait with `prepare()` and `draw()` methods
- [ ] Implement each pass as a struct implementing the trait
- [ ] WGSL shaders for each pass (in `renderer/shaders/`)
- [ ] Frame loop: acquire surface texture → run passes → present
- [ ] Validation: render without any scene entities (empty scene must not panic)

---

### 0.6 — egui Integration

**Panel layout (hardcoded for M0):**
```
┌──────────────────────────────────────────────┐
│  [Toolbar: camera reset, view presets]       │
├────────────────┬─────────────────────────────┤
│  Entity list   │                             │
│  (left panel)  │    3D Viewport              │
│                │                             │
│  - entity      │                             │
│    ☑ visible   │                             │
│  - entity      │                             │
│    ☑ visible   │                             │
│                │                             │
└────────────────┴─────────────────────────────┘
```

**Tasks:**
- [ ] `egui-wgpu` integration — egui renders into final swapchain texture after 3D passes
- [ ] Left panel: list of scene entities, visibility toggle per entity
- [ ] Top toolbar: FPS counter, camera reset button, top/side view buttons
- [ ] Mouse input routing: egui captures mouse when cursor is over a panel; 3D viewport captures otherwise
- [ ] Resize: viewport fills remaining space after panels

---

### 0.7 — Mock Data Injector

A test harness in the `mock_injector` crate that populates the scene graph without any ROS2 dependency. Used to validate all rendering paths before ROS2 is wired up.

**Scenarios the injector must cover:**
- Animated arrow moving in a circle (simulates a moving pose)
- Static occupancy grid (10x10, random free/occupied/unknown cells)
- A set of polylines forming a figure-8 path
- A reference frame at origin + one child frame offset by 1m
- A small triangle mesh (hardcoded cube or pyramid)
- 500 points scattered in a hemisphere (stub for future point cloud)

**Tasks:**
- [ ] `MockInjector` struct with `update(dt: f32, scene: &mut SceneGraph)` method
- [ ] All scenarios listed above implemented
- [ ] Injector runs in the main loop (same thread as app for M0; moved to separate thread in M1)
- [ ] CLI flag `--mock` to enable injector mode (later default when no ROS2 found)

---

### Milestone 0 Checklist Summary

| # | Task | Crate |
|---|------|-------|
| 0.1 | Workspace + CI + Docker skeleton | all |
| 0.2 | Scene primitives + scene graph | `scene` |
| 0.3 | wgpu init + window + resize | `renderer` |
| 0.4 | Orbit camera + input | `renderer`, `app` |
| 0.5 | Render passes (grid, line, arrow, mesh, point) | `renderer` |
| 0.6 | egui integration + panel layout | `app` |
| 0.7 | Mock data injector | `mock_injector` |

**Done when:** `cargo run --features mock` shows an animated scene with all primitive types, camera is fully controllable, egui panels work, runs inside Docker with X11.

---

## Milestone 0.5 — First ROS2 Node: 2D Navigation Suite

**Goal:** The tool subscribes to the six most important 2D robotics data types and visualizes them correctly relative to a chosen reference frame. This is the milestone where the tool first becomes useful in a real robot lab.

**ROS2 crate added:** `ros_node` — depends on `rclrs` and `r2r`. Ingests ROS2 messages and writes into the scene graph.

**Supported distros:** Humble (22.04) and Jazzy (24.04).

**Exit criterion:** All six data types visualized correctly in a real Nav2 stack (simulated or physical). TF transforms applied correctly. Performance: UI remains at 60fps under all listed topic rates.

---

### 0.5.1 — ROS2 Node Infrastructure

**Tasks:**
- [ ] Add `ros_node` crate to workspace
- [ ] `rclrs` node initialization: context, node, executor
- [ ] Executor runs on a **dedicated thread** — never blocks the render loop
- [ ] Node writes to scene graph via `Arc<RwLock<SceneGraph>>`; write lock held for minimum time
- [ ] Topic configuration: loaded from a TOML config file or CLI args
- [ ] Connection: node announces itself as `/ros2_viz` node visible in `ros2 node list`
- [ ] Graceful shutdown: Ctrl-C signals both the ROS2 thread and render loop

**Thread model:**
```
Main thread:       winit event loop → render loop → egui
ROS2 thread:       rclrs executor spin → message callbacks → scene write
```

---

### 0.5.2 — TF / Transform Tree

TF is the foundation — all other data types need transforms to be placed correctly in the world.

**Subscriptions:**
- `/tf` (`tf2_msgs/TFMessage`) — dynamic transforms
- `/tf_static` (`tf2_msgs/TFMessage`) — static transforms (latched)

**Internal TF tree:**
```rust
pub struct TfTree {
    // frame_id → (parent_frame_id, transform, timestamp)
    frames: HashMap<String, TransformEntry>,
}

impl TfTree {
    pub fn update(&mut self, msg: &TFMessage) { ... }
    pub fn lookup(&self, target: &str, source: &str, time: Time) -> Option<Mat4> { ... }
}
```

**Rendering (optional, togglable):**
- Each known frame → `Frame` primitive (3 colored axes, XYZ = RGB)
- Parent→child connector → `Polyline` primitive
- Frame label → `Label` primitive

**Tasks:**
- [ ] `TfTree` struct in `ros_node/src/tf.rs`
- [ ] Subscribe to `/tf` and `/tf_static`
- [ ] `lookup_transform()` with interpolation between closest timestamps
- [ ] Update scene with frame entities when TF visualization is enabled
- [ ] egui panel: list of known frames, toggle frame display per frame

---

### 0.5.3 — OccupancyGrid

**Subscription:** `/map` (`nav_msgs/OccupancyGrid`)

**Mapping to scene primitive:**
- Message → `Grid { origin, cell_size, cols, rows, data: GridData::Cells(...) }`
- Apply `map.info.origin` pose (position + orientation) as the grid's world transform
- Cell values: `−1` = unknown (grey), `0` = free (white), `100` = occupied (black)
- On update: only re-upload GPU texture if `map.header.stamp` changed (dirty flag)

**GPU representation:**
- Grid rendered as a single quad with a GPU texture (R8Unorm)
- Colormap applied in the fragment shader (no CPU-side colormap baking)
- Alpha: configurable (default 0.7 so the robot model shows through)

**Tasks:**
- [ ] Subscriber + message callback
- [ ] Grid → `ScenePrimitive::Grid` conversion
- [ ] wgpu texture upload (R8Unorm, 1 byte per cell)
- [ ] Fragment shader with OccupancyDefault colormap
- [ ] egui panel: topic name, opacity slider, colormap selector
- [ ] Handle map updates (map may be re-published at low rate as it grows)

---

### 0.5.4 — Poses

Two message types, same primitive output:

**Subscriptions:**
- `geometry_msgs/PoseStamped` — single pose
- `geometry_msgs/PoseArray` — array of poses (particle filter output)

**Mapping:**
- Each pose → `Arrow` primitive
- Arrow placed at pose position, pointing in pose heading direction
- For `PoseArray`: all poses → `Vec<Arrow>` under a single entity (one GPU upload)
- Transform pose from message frame to reference frame via TfTree

**Tasks:**
- [ ] `PoseStamped` subscriber → single `Arrow` entity
- [ ] `PoseArray` subscriber → `Vec<Arrow>` entity
- [ ] TF lookup for each pose's `header.frame_id`
- [ ] egui panel: color picker, arrow size slider, topic selector

---

### 0.5.5 — Paths

**Subscription:** `nav_msgs/Path`

**Mapping:**
- `Path.poses` → `Polyline` (connect pose positions in order)
- Optionally also render small arrows at each waypoint (configurable)
- Transform all points from path's `header.frame_id` to reference frame

**Tasks:**
- [ ] `Path` subscriber → `Polyline` entity
- [ ] TF lookup for path frame
- [ ] egui panel: color picker, line width, show/hide waypoint arrows
- [ ] Handle path updates efficiently (replace entity in-place, set dirty)

---

### 0.5.6 — LaserScan

**Subscription:** `sensor_msgs/LaserScan`

**Mapping:**
- Each valid range reading → a `Point` in polar→Cartesian conversion
- Points placed in the sensor frame, then transformed to reference frame via TF
- Color modes:
  - Flat (single color)
  - Intensity (if intensity field populated)
  - Range (distance colormap)

**Performance notes:**
- LaserScan at 10–40Hz with 360–1800 points each: moderate load
- Convert polar to Cartesian on CPU; upload as `Vec<Point>` → GPU buffer
- Buffer is pre-allocated to `max_ranges` capacity; only length varies

**Tasks:**
- [ ] `LaserScan` subscriber + polar→Cartesian conversion
- [ ] TF lookup for `header.frame_id` (typically `laser` or `lidar`)
- [ ] `Vec<Point>` → `ScenePrimitive::Points` entity
- [ ] Pre-allocated GPU vertex buffer (avoid realloc per scan)
- [ ] egui panel: color mode selector, point size, min/max range filter

---

### 0.5.7 — URDF Robot Model

**Input:** URDF file path (not a ROS2 topic — loaded from disk or `robot_description` parameter)

**Subscription:** `sensor_msgs/JointState` (for animation)

**Loading pipeline:**
```
URDF file
  → parse with urdf-rs
  → for each link: load mesh (STL/OBJ/DAE)
  → build link tree (parent/child relationships)
  → for each mesh: convert to ScenePrimitive::Mesh
  → register each link as a SceneEntity with link's frame_id
```

**Animation:**
- `JointState` message → update joint angles in the URDF link tree
- Recompute forward kinematics → new transforms for each link
- Update link entities' transforms in scene graph (no GPU re-upload needed — mesh data unchanged, only transform changes)

**Tasks:**
- [ ] `urdf-rs` integration in `ros_node/src/urdf.rs`
- [ ] STL loader (binary STL → `Vec<Vertex>`)
- [ ] OBJ loader (simple wavefront, no MTL required initially)
- [ ] Forward kinematics from joint angles
- [ ] `JointState` subscriber → FK → scene entity transform updates
- [ ] CLI arg: `--urdf /path/to/robot.urdf` or load from `robot_description` param
- [ ] egui panel: show/hide per link, toggle visual vs. collision meshes

---

### Milestone 0.5 Checklist Summary

| # | Component | Topics | Primitive |
|---|-----------|--------|-----------|
| 0.5.1 | ROS2 node infrastructure | — | — |
| 0.5.2 | TF tree | `/tf`, `/tf_static` | `Frame`, `Polyline` |
| 0.5.3 | OccupancyGrid | `/map` | `Grid` |
| 0.5.4 | Poses | `/pose`, `/particles` | `Arrow` |
| 0.5.5 | Path | `/path`, `/plan` | `Polyline` |
| 0.5.6 | LaserScan | `/scan` | `Points` |
| 0.5.7 | URDF + JointState | `/joint_states` | `Mesh` |

**Done when:** A full Nav2 stack (Gazebo or real robot) is visually debuggable. Robot model animates, laser scan updates live, map renders correctly, TF frames are visible, planned path is drawn.

---

## What Comes After

The subsequent milestones follow the high-level plan, now with a solid foundation:

| Milestone | Focus |
|-----------|-------|
| 1 | `sensor_msgs/Image`, `CompressedImage` — camera feeds in a 2D panel |
| 2 | `sensor_msgs/PointCloud2` — LiDAR, full GPU instanced pipeline |
| 3 | `visualization_msgs/MarkerArray` — full marker type support |
| 4 | IMU, Odometry, time-series plots |
| 5 | Plugin system, custom message types |
| 6 | MCAP recording + playback, community launch |

Point clouds are intentionally last — they require the most GPU pipeline work and are less critical for the 2D nav use case that most users will validate first.

---

## Dependency Summary

```toml
# Cargo.toml (workspace)
[workspace.dependencies]
wgpu = "22"
winit = "0.30"
egui = "0.28"
egui-wgpu = "0.28"
egui-winit = "0.28"
glam = "0.28"
bytemuck = { version = "1.16", features = ["derive"] }
crossbeam-channel = "0.5"
parking_lot = "0.12"
urdf-rs = "0.8"
# ROS2 (added in Milestone 0.5)
rclrs = "0.4"
r2r = "0.9"
```

---

*Fine-grained plan version 1.0 — May 2026*
*Milestone 0 is self-contained and has zero ROS2 dependencies.*
*Milestone 0.5 builds directly on top of it.*
