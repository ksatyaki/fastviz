<img src="LOGO.png" alt="Logo" width="180"/>

A Rust-based ROS2 visualizer built on `wgpu` + `egui`. RViz alternative.

<img src="Screenshot.png" alt="Screenshot" width="720"/>

## Install

### Prebuilt release (Linux x86_64)

Each tagged release publishes Ubuntu 24.04 / ROS Jazzy artifacts on the [Releases page](../../releases) — both as a tarball and as a `.deb`.

#### Option A — .deb (recommended)

The `.deb` declares all runtime dependencies, including the ROS message packages and Vulkan loader, so `apt` pulls them in for you:

| OS                   | Asset                                   |
| -------------------- | --------------------------------------- |
| Ubuntu 24.04 / Jazzy | `fastviz-jazzy_<version>-1_amd64.deb`   |

```sh
curl -LO "https://github.com/OWNER/REPO/releases/latest/download/fastviz-jazzy_<version>-1_amd64.deb"
sudo apt install ./fastviz-jazzy_<version>-1_amd64.deb
fastviz --config /usr/share/fastviz/configs/default.toml
```

The installed binary has the ROS lib directory baked into its rpath, so you do *not* need to `source /opt/ros/<distro>/setup.bash` before launching it.

#### Option B — tarball

```sh
tag=$(curl -s https://api.github.com/repos/OWNER/REPO/releases/latest | grep -oE '"tag_name": *"[^"]+"' | cut -d'"' -f4)
asset="app-${tag}-x86_64-unknown-linux-gnu-noble-jazzy.tar.gz"
curl -L -o fastviz.tar.gz "https://github.com/OWNER/REPO/releases/download/${tag}/${asset}"
tar -xzf fastviz.tar.gz
sudo install -m 0755 "${asset%.tar.gz}" /usr/local/bin/fastviz
```

Replace `OWNER/REPO` with the repo slug (or just download the asset from the Releases page in a browser).

### Runtime dependencies (tarball only — the .deb pulls these in)

The release binary is dynamically linked. On the host you'll need:

- **ROS2 Jazzy** (Ubuntu 24.04). Other distros are untested.
- ROS2 message packages used by the subscribers:
  ```sh
  sudo apt-get install -y \
    ros-jazzy-tf2-msgs ros-jazzy-nav-msgs ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs ros-jazzy-visualization-msgs
  ```
- **Vulkan loader + an ICD** for `wgpu`:
  ```sh
  sudo apt-get install -y libvulkan1 vulkan-tools mesa-vulkan-drivers
  ```
  On NVIDIA hosts, the proprietary driver provides its own ICD — install `nvidia-driver-*` from the Ubuntu archive (or the upstream `.run` installer). Verify with `vulkaninfo --summary`.
- Windowing/input libraries (usually already present on a desktop install): `libxkbcommon0`, `libwayland-client0`, `libxcb1`.

### Build from source

```sh
sudo apt-get install -y build-essential pkg-config clang libclang-dev \
  libx11-dev libxcb1-dev libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev \
  libxkbcommon-dev libwayland-dev libssl-dev \
  libvulkan1 vulkan-tools mesa-vulkan-drivers
export LIBCLANG_PATH=/usr/lib/x86_64-linux-gnu   # r2r's bindgen step needs this
source /opt/ros/jazzy/setup.bash
cargo build --release
```

The binary lands at `target/release/app`. Toolchain version is pinned in [rust-toolchain.toml](rust-toolchain.toml); [rustup](https://rustup.rs) picks it up automatically.

### Dev container

If your host doesn't have ROS2 Jazzy (e.g. you're on Fedora, Arch, or a non-24.04 Ubuntu), use the bundled dev container — it ships Ubuntu 24.04 + Jazzy + the `r2r` build deps + the Vulkan loader, all pre-installed.

```sh
# VS Code / Cursor:
#   1. Install the "Dev Containers" extension.
#   2. Open the repo folder, then "Reopen in Container".
```

NVIDIA GPU passthrough requires [`nvidia-container-toolkit`](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) on the host. The container ships an NVIDIA Vulkan ICD manifest that activates automatically when the toolkit bind-mounts the driver libs; on non-NVIDIA hosts it falls back to Mesa.

See [.devcontainer/Dockerfile](.devcontainer/Dockerfile) for the exact image — it's also a good reference if you want to build your own container.

## Use

Source ROS2 first, then launch with a config file:

```sh
source /opt/ros/jazzy/setup.bash
fastviz --config configs/default.toml
```

Common invocations:

```sh
# TurtleBot 4 Gazebo sim — picks the URDF up off /robot_description
fastviz --config configs/turtlebot4.toml

# Override the reference frame from the CLI (wins over the config file)
fastviz --config configs/default.toml --ref-frame odom

# Load a URDF from a file directly (skips /robot_description)
fastviz --config configs/default.toml --urdf /path/to/robot.urdf
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
| `[markers]`                         | `visualization_msgs/Marker`      | mixed primitives |
| `[marker_arrays]`                   | `visualization_msgs/MarkerArray` | mixed primitives |
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
fastviz [--ref-frame FRAME] [--config PATH] [--urdf PATH] [--width N] [--height N]
```

`--ref-frame` and `--urdf` win over the matching config-file values when both are set.

## Configuration

[configs/default.toml](configs/default.toml) mirrors `RosConfig::default()` and is the source of truth. Other presets live alongside it (e.g. [configs/turtlebot4.toml](configs/turtlebot4.toml)). Key features:

- Each per-message kind (`[map]`, `[poses]`, `[pose_arrays]`, `[paths]`, `[scans]`, `[points]`, `[markers]`, `[marker_arrays]`) takes a `topics = [...]` list.
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
| `ros_node`     | r2r executor on a dedicated thread; per-message subscribers, TF tree, URDF loader, polled topic discovery |
| `mock_injector` | dev-only test harness that populates the scene without ROS2 |

## Status

Milestone 0.5 is complete: TF, OccupancyGrid, PoseStamped, PoseArray, Path, LaserScan, PointCloud2, URDF + JointState with STL/OBJ/DAE meshes, `visualization_msgs/Marker(Array)`, and TOML config with polled wildcard discovery + per-topic QoS. See [MBABYSTEPS_1.md](MBABYSTEPS_1.md) for the running plan.
