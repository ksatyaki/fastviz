# fastviz

A Rust-based ROS2 visualizer built on `wgpu` + `egui`. RViz alternative.

## Status

**Milestone 0.5 — in progress.** Steps A–F + F.5 complete (six message types
ingest into the scene graph; TF tree + reframing; TOML config with per-topic
QoS overrides and polled wildcard discovery). **Step G** (URDF + JointState
→ meshes + FK) is the remaining piece. See [MBABYSTEPS_1.md](MBABYSTEPS_1.md)
for the running plan.

Supported ROS2 message types today:

| Topic kind                        | Message                          | Scene primitive  |
| --------------------------------- | -------------------------------- | ---------------- |
| `[map]`                           | `nav_msgs/OccupancyGrid`         | `Grid`           |
| `[poses]`                         | `geometry_msgs/PoseStamped`      | `Arrow`          |
| `[pose_arrays]`                   | `geometry_msgs/PoseArray`        | `Arrows`         |
| `[paths]`                         | `nav_msgs/Path`                  | `Polyline`       |
| `[scans]`                         | `sensor_msgs/LaserScan`          | `Points`         |
| `/tf`, `/tf_static`               | `tf2_msgs/TFMessage`             | (TF tree)        |

## Build

Workspace builds with no ROS2 sourced (`mock` feature, default):

```sh
cargo build
cargo run -p app -- --mock
```

## ROS2 mode

Inside the dev container (ROS2 Jazzy already sourced):

```sh
cargo run -p app -- --ros                            # defaults; subscribes to /map, /scan, /plan, …
cargo run -p app -- --ros --config fastviz.toml      # explicit config
cargo run -p app -- --ros --no-mock                  # ROS only, no mock injector
```

`--ros` lives behind a cargo feature (`ros`) that's on by default. `cargo
build --no-default-features --features mock` drops the `r2r`/`bindgen`
toolchain requirement for hosts without ROS2.

### Config file

[fastviz.toml](fastviz.toml) at the repo root mirrors `RosConfig::default()`
and is the source of truth for documentation. Key features:

- Each kind (`[map]`, `[poses]`, `[pose_arrays]`, `[paths]`, `[scans]`)
  takes a `topics = [...]` list.
- A bare `"*"` element enables polled discovery: anything in the ROS graph
  with the matching message type is auto-subscribed within ~1 s.
- Per-topic QoS overrides via `[<kind>.qos."<topic>"]`:
  `reliability`, `durability`, `depth`.
- Visual style per kind (`arrow`, `paths.style`, `scans.style`).

Example with wildcard scans + a per-topic QoS override on `/map`:

```toml
reference_frame = "map"

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
| `ros_node`      | r2r executor on a dedicated thread; per-message subscribers, TF tree, polled topic discovery |

## Controls

- Left mouse drag — orbit
- Right mouse drag — pan
- Scroll — zoom
- `F` — reset view
- `T` — top-down view
- `S` — side view
- `Esc` — quit

## CLI

```
app [--mock] [--ros] [--ref-frame FRAME] [--config PATH] [--width N] [--height N]
```

`--ref-frame` (CLI) wins over `reference_frame` in the config file when both
are set.

## Docker (release image)

The root `Dockerfile` is the M0 release image — builds the binary and runs it.

```sh
docker build -t fastviz .
xhost +SI:localuser:$(whoami)
docker run --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix fastviz --mock
```

## Dev container (M0.5+ workflow)

`.devcontainer/` holds an Ubuntu 24.04 + ROS2 Jazzy image with `r2r` build
deps, Vulkan loader, and the host's `~/.claude` mounted in. Open the
folder in VS Code / Cursor and choose "Reopen in Container".

Host prereqs for NVIDIA Vulkan acceleration (Fedora):

```sh
sudo dnf install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
xhost +SI:localuser:$(whoami)
```

Inside the container:

```sh
echo $ROS_DISTRO                  # → jazzy
vulkaninfo --summary | head       # → NVIDIA driver if --gpus=all worked
cargo run -p app -- --mock        # M0 path still works
cargo run -p app -- --ros --no-mock --config fastviz.toml
```

If you don't have `nvidia-container-toolkit`, remove `--gpus=all` from
`.devcontainer/devcontainer.json`'s `runArgs` — Mesa llvmpipe will be
used instead.
