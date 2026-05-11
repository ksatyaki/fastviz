# fastviz

A Rust-based ROS2 visualizer built on `wgpu` + `egui`. RViz alternative.

## Installation

### Prerequisites

- ROS2 **Jazzy** (24.04). Other distros are untested.
- A Rust toolchain — see [rust-toolchain.toml](rust-toolchain.toml). [rustup](https://rustup.rs) picks this up automatically.
- `r2r`'s `bindgen` step needs libclang. On Debian/Ubuntu:

  ```sh
  sudo apt-get install -y libclang-dev clang
  export LIBCLANG_PATH=/usr/lib/x86_64-linux-gnu
  ```
- Vulkan loader (`libvulkan1` on Ubuntu) for `wgpu`. NVIDIA users also want `nvidia-container-toolkit` if running inside Docker.

### Build

Source ROS2, then build the workspace:

```sh
source /opt/ros/jazzy/setup.bash
cargo build --release
```

The binary lands at `target/release/app`.

### Dev container

`.devcontainer/` ships an Ubuntu 24.04 + ROS2 Jazzy image with `r2r` build deps, the Vulkan loader, and your host's `~/.claude` mounted in. Open the folder in VS Code / Cursor and pick "Reopen in Container".

## Use

The node always starts as a ROS2 visualizer — no flag needed:

```sh
cargo run -p app -- --config configs/default.toml
```

Common invocations:

```sh
# TurtleBot 4 Gazebo sim — picks the URDF up off /robot_description
cargo run -p app -- --config configs/turtlebot4.toml

# Override the reference frame from the CLI (wins over the config file)
cargo run -p app -- --config configs/default.toml --ref-frame odom

# Load a URDF from a file directly (skips /robot_description)
cargo run -p app -- --config configs/default.toml --urdf /path/to/robot.urdf

# No live ROS graph? Layer the M0 mock injector on top for a quick sanity check
cargo run -p app -- --mock
```

### Supported message types

| Topic kind                          | Message                          | Scene primitive  |
| ----------------------------------- | -------------------------------- | ---------------- |
| `[map]`                             | `nav_msgs/OccupancyGrid`         | `Grid`           |
| `[poses]`                           | `geometry_msgs/PoseStamped`      | `Arrow`          |
| `[pose_arrays]`                     | `geometry_msgs/PoseArray`        | `Arrows`         |
| `[paths]`                           | `nav_msgs/Path`                  | `Polyline`       |
| `[scans]`                           | `sensor_msgs/LaserScan`          | `Points`         |
| `[points]`                          | `sensor_msgs/PointCloud2`        | `Points`         |
| `[tf]` (`/tf`, `/tf_static`)        | `tf2_msgs/TFMessage`             | TF tree          |
| `[urdf]` (`/robot_description`)     | `std_msgs/String` + `JointState` | meshes + FK      |

Mesh files referenced from a URDF can be `.stl`, `.obj`, or `.dae` (Collada). `package://` URIs are resolved through `AMENT_PREFIX_PATH`.

### Controls

- Left mouse drag — orbit
- Right mouse drag — pan
- Scroll — zoom
- `F` — reset view
- `T` — top-down view
- `S` — side view
- `Esc` — quit

### CLI

```
app [--mock] [--ref-frame FRAME] [--config PATH] [--urdf PATH] [--width N] [--height N]
```

`--ref-frame` and `--urdf` win over the matching config-file values when both are set.

## Configuration

[configs/default.toml](configs/default.toml) mirrors `RosConfig::default()` and is the source of truth. Other presets live alongside it (e.g. [configs/turtlebot4.toml](configs/turtlebot4.toml)). Key features:

- Each per-message kind (`[map]`, `[poses]`, `[pose_arrays]`, `[paths]`, `[scans]`, `[points]`) takes a `topics = [...]` list.
- A bare `"*"` element enables polled discovery: anything in the ROS graph with the matching message type is auto-subscribed within ~1 s. Works for the per-message kinds above (not `[map]`, which is single-topic).
- Per-topic QoS overrides via `[<kind>.qos."<topic>"]`: `reliability`, `durability`, `depth`.
- Visual style per kind (`arrow`, `paths.style`, `scans.style`, `points.style`).
- `[tf]` block lets you remap `/tf` / `/tf_static` topic names + override their QoS — handy for robots that publish under a namespace.
- `[urdf]` block: set either `path` (URDF/xacro file on disk) or `topic` (`std_msgs/String` topic carrying the URDF XML, typically `/robot_description`). `joint_states_topic` defaults to `/joint_states`.

Example with wildcard scans, a per-topic QoS override on `/map`, and a robot loaded off `/robot_description`:

```toml
reference_frame = "map"

[tf]
topic        = "/tf"
static_topic = "/tf_static"

[urdf]
topic              = "/robot_description"
joint_states_topic = "/joint_states"

[map]
topics = ["/map"]
[map.qos."/map"]
durability  = "transient_local"
reliability = "reliable"

[scans]
topics = ["*"]                # auto-discover every sensor_msgs/LaserScan
style  = { size = 4.0, color = [1.0, 0.95, 0.20] }
```

## Workspace layout

| crate           | role                                                       |
| --------------- | ---------------------------------------------------------- |
| `app`           | binary: window, event loop, egui shell, CLI                |
| `renderer`      | wgpu pipelines, render passes, camera                      |
| `scene`         | scene graph, scene primitives, dirty tracking              |
| `mock_injector` | test harness that populates the scene without ROS2         |
| `ros_node`      | r2r executor on a dedicated thread; per-message subscribers, TF tree, URDF loader, polled topic discovery |

## Docker (release image)

The root `Dockerfile` builds the binary and runs it.

```sh
docker build -t fastviz .
xhost +SI:localuser:$(whoami)
docker run --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix fastviz --mock
```

## Status

Milestone 0.5 features (TF, OccupancyGrid, PoseStamped, PoseArray, Path, LaserScan, PointCloud2, URDF + JointState with STL/OBJ/DAE meshes, TOML config with polled wildcard discovery and per-topic QoS) are complete. See [MBABYSTEPS_1.md](MBABYSTEPS_1.md) for the running plan and what's next.

`--mock`, the cargo `mock` feature, and `--no-default-features --features mock` let you run/build without a live ROS2 install — useful for testing the rendering pipeline on hosts that don't have `/opt/ros`.
