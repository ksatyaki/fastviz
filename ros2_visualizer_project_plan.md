# ROS2 Visualizer — Project Plan
*A ROS2-native, GPU-accelerated visualization tool built in Rust*

---

## Vision

A standalone, open source ROS2 visualizer that:
- Requires **no bridge, no middleware, no separate process** — it is a ROS2 node
- Renders with **wgpu** (Vulkan/Metal/DX12/WebGPU) for GPU-accelerated performance
- Works **natively inside Docker containers** with X11/Wayland passthrough
- Supports the full breadth of common ROS2 data types
- Is **extensible** — custom message types and plugins from day one
- Targets **Humble + Jazzy** (the two active LTS releases)

---

## Name suggestion

**`rosv`** or **`vista`** or **`veil`** — short, memorable, CLI-friendly. To be decided by community vote at launch.

---

## Technology Stack

| Layer | Choice | Rationale |
|---|---|---|
| Language | Rust | Performance, safety, wgpu ecosystem |
| Rendering | wgpu + naga | GPU-native, cross-backend (Vulkan primary) |
| UI framework | egui | Immediate-mode, embeds cleanly with wgpu, used by Rerun |
| ROS2 bindings | rclrs (ros2-rust) | Official Rust ROS2 client library |
| Message types | rosidl generated + r2r | r2r for dynamic message handling |
| Point cloud | Custom wgpu pipeline | GPU instancing, no per-frame CPU upload |
| Mesh/URDF | urdf-rs + custom loader | Parse URDF, load meshes via gltf/obj |
| Image display | wgpu texture upload | Direct GPU texture, no CPU copy per frame |
| TF | tf2 reimplemented in Rust | Transform tree, interpolation |
| Occupancy grid | wgpu texture | Grid as texture, updated incrementally |
| Config/layout | RON or TOML | Rerun-inspired "panel" layout saved to file |
| Build system | Cargo + colcon overlay | Plays well with ROS2 workspace |
| CI | GitHub Actions | Build, test, clippy, format |
| Container | Docker + docker-compose examples | X11/Wayland socket passthrough documented |

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────┐
│                   ROS2 DDS Layer                    │
└────────────────────────┬────────────────────────────┘
                         │ rclrs subscriptions
┌────────────────────────▼────────────────────────────┐
│              Message Ingestion Layer                │
│  - Topic discovery & type introspection             │
│  - Deserialize into internal scene primitives       │
│  - TF tree maintenance                              │
└────────────────────────┬────────────────────────────┘
                         │ lock-free ring buffers
┌────────────────────────▼────────────────────────────┐
│                  Scene Graph                        │
│  - Frame-relative entity positions                  │
│  - Retained GPU buffers (point clouds, meshes)      │
│  - Dirty-flag driven updates                        │
└────────────────────────┬────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────┐
│              wgpu Render Pipeline                   │
│  - Point cloud pass (instanced, depth-sorted)       │
│  - Mesh pass (PBR or flat shading)                  │
│  - Image/overlay pass                               │
│  - Grid / axes / decorators pass                    │
│  - egui UI pass (panels, topic list, config)        │
└─────────────────────────────────────────────────────┘
```

Key design principle: **the render thread never touches ROS2**. Ingestion and rendering communicate via lock-free channels. This means the UI never blocks on DDS and rendering is always smooth.

---

## Milestones

### Milestone 0 — Core Renderer (No ROS2)
*Goal: wgpu pipeline, scene graph, all primitives renderable, camera control, egui shell. Data driven by a mock injector — zero ROS2 dependency.*

- [ ] Cargo workspace: `app`, `renderer`, `scene`, `mock_injector` crates
- [ ] Scene primitives defined: `Points`, `Polyline`, `Arrow`, `Grid`, `Mesh`, `Frame`, `Label`
- [ ] Scene graph with dirty-flag driven GPU updates
- [ ] wgpu init + winit window + resize handling + depth buffer
- [ ] Orbit camera: mouse orbit/pan/zoom, keyboard view presets
- [ ] Render passes: grid, line, arrow, mesh, point (stub)
- [ ] egui integration: entity list panel + visibility toggles + FPS counter
- [ ] Mock data injector: animated arrow, occupancy grid, path, TF frames, mesh
- [ ] CI pipeline: Ubuntu 22.04 + 24.04, clippy, rustfmt
- [ ] Docker: X11 passthrough documented and working
- [ ] README and CONTRIBUTING docs

**Deliverable:** `cargo run --features mock` shows an animated scene with all primitive types. No ROS2 required.

*See fine-grained plan document for full sub-task breakdown.*

---

### Milestone 0.5 — First ROS2 Node: 2D Navigation Suite
*Goal: The tool becomes genuinely useful for 2D robotics. Subscribes to the core Nav2 data types, transforms everything via TF.*

- [ ] ROS2 node infrastructure: dedicated thread, graceful shutdown, TOML config
- [ ] TF tree (`/tf`, `/tf_static`) — frame display, parent/child connectors
- [ ] `nav_msgs/OccupancyGrid` — GPU texture, opacity control, colormap
- [ ] `geometry_msgs/PoseStamped` + `PoseArray` — arrow visualization
- [ ] `nav_msgs/Path` — polyline with optional waypoint arrows
- [ ] `sensor_msgs/LaserScan` — polar→Cartesian, pre-allocated GPU buffer, color modes
- [ ] URDF loading + `sensor_msgs/JointState` — forward kinematics, mesh animation
- [ ] egui panels: per-topic color, size, visibility; frame selector; topic name input

**Deliverable:** Full Nav2 stack (map, robot model, laser scan, poses, path, TF) visually debuggable. Runs natively in Docker with no bridge.

*See fine-grained plan document for full sub-task breakdown.*

---

### Milestone 1 — Camera Feeds & MarkerArray
*Goal: Sensor cameras and arbitrary marker overlays supported*

- [ ] `sensor_msgs/Image` and `sensor_msgs/CompressedImage` — 2D panel, GPU texture upload
- [ ] Camera info overlay (topic, hz, resolution)
- [ ] `visualization_msgs/MarkerArray` — arrow, cube, sphere, line strip, text label, mesh resource
- [ ] Costmap2D overlay (second `OccupancyGrid` layer with separate colormap)
- [ ] Topic discovery UI: browse and subscribe to any topic from within the app

**Deliverable:** Full sensor suite visible; Nav2 costmaps renderable; arbitrary debug markers supported.

---

### Milestone 2 — Time Series & IMU
*Goal: Non-spatial data has a home*

- [ ] Collapsible time-series panel (egui plot)
- [ ] `sensor_msgs/Imu` — orientation display + angular velocity / acceleration graphs
- [ ] `nav_msgs/Odometry` — pose arrow + covariance ellipse
- [ ] `sensor_msgs/BatteryState` and arbitrary `std_msgs/Float64` topics
- [ ] Configurable history window, pause/resume, export to CSV

**Deliverable:** IMU, odometry, and scalar topics plotted live alongside the 3D view.

---

### Milestone 3 — Point Clouds
*Goal: LiDAR visualization faster than RViz2*

- [ ] `sensor_msgs/PointCloud2` — parse field descriptors (XYZ, intensity, ring, time)
- [ ] Persistent GPU vertex buffer — no per-frame CPU re-upload
- [ ] Instanced point quad rendering — configurable point size
- [ ] Color modes: flat, intensity, height, ring, time
- [ ] GPU memory usage indicator
- [ ] Benchmark vs RViz2 published in README

**Deliverable:** Velodyne / Ouster / Livox clouds at 60fps on mid-range GPU.

---

### Milestone 4 — Plugin System & Custom Messages
*Goal: Extensibility for community contributions*

- [ ] `Visualizer` trait: `subscribe()`, `update_scene()`, `draw_egui_panel()`
- [ ] Dynamic library plugin loading (`.so` files, source-compatible ABI)
- [ ] Plugin registry and discovery via config file
- [ ] Example plugin: custom message type visualizer
- [ ] Plugin authoring guide in `docs/plugin_guide.md`

**Deliverable:** Third parties can add visualizers without forking core. Tutorial completable in under 1 hour.

---

### Milestone 5 — Polish & Community Launch
*Goal: Ready for public announcement on ROS Discourse and GitHub*

- [ ] Layout save/load — panel configuration persisted to TOML
- [ ] Topic search and filter in entity list
- [ ] Recording to MCAP (rosbag2-compatible)
- [ ] Playback of MCAP files — offline mode, no ROS2 needed
- [ ] Dark/light theme toggle
- [ ] Proper user-facing error messages (topic type mismatch, TF lookup failure, etc.)
- [ ] Installation docs: `cargo install`, apt PPA, Docker
- [ ] Demo video / GIF for README
- [ ] License: Apache 2.0

---

## Docker Strategy

The tool should work in three container scenarios:

1. **Tool runs on host, ROS2 in container** — standard DDS discovery, no special config
2. **Tool runs in container alongside ROS2** — compose file with shared network
3. **Tool runs in container, display on host** — X11 socket mount (`-v /tmp/.X11-unix:/tmp/.X11-unix`) or Wayland (`WAYLAND_DISPLAY` passthrough)

Document all three with compose examples. No bridge required in any scenario.

---

## Repository Structure

```
ros2_viz/
├── Cargo.toml                  # workspace
├── crates/
│   ├── app/                    # main binary, window, event loop
│   ├── renderer/               # wgpu pipelines, scene graph
│   ├── ros_bridge/             # rclrs node, ingestion, TF
│   ├── plugin_api/             # Visualizer trait, public API
│   ├── builtin_plugins/        # all milestone visualizers
│   └── mcap_io/                # recording and playback
├── docker/
│   ├── Dockerfile
│   └── compose examples/
├── docs/
│   ├── architecture.md
│   ├── plugin_guide.md
│   └── docker_guide.md
└── .github/
    └── workflows/
        ├── ci.yml
        └── release.yml
```

---

## Open Questions / Decisions to Revisit

1. **Name** — needs community input early
2. **r2r vs rclrs** — r2r has better dynamic message support (important for custom messages and topic introspection); rclrs is more official. May need both.
3. **Wayland vs X11** — target X11 first via winit, add native Wayland later
4. **MCAP vs rosbag2 SQLite** — MCAP is the future, prioritize it
5. **Plugin ABI stability** — decide early whether plugins are source-compatible or binary-compatible (source is much easier)

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

- Point cloud rendering at **60fps** for 100k+ points on mid-range GPU
- **Cold start to first frame** under 2 seconds
- Works in Docker with **zero extra configuration** beyond X11 socket mount
- Plugin tutorial completable in **under 1 hour**
- GitHub stars / Discourse engagement as community signal

---

*Plan version 1.1 — revised May 2026*
*Milestones restructured: M0 is renderer-only (no ROS2), M0.5 is 2D nav suite, point clouds moved to M3.*
*See fine-grained plan document for M0 and M0.5 sub-task detail.*
