//! Runtime configuration for the ROS node.
//!
//! Two related types live here:
//! - [`RawConfig`]: the TOML-deserialisable schema (uses `[f32; 3]` for colors,
//!   plain `Vec<String>` for topics, etc.). This is what `configs/*.toml` maps to.
//! - [`RosConfig`]: the runtime form consumed by subscribers (uses `scene::Color`,
//!   already-validated values). Built either via `RosConfig::default()` or
//!   `RawConfig::into_runtime()`.
//!
//! `RosConfig::from_path(p)` is the convenience loader.

use std::collections::HashMap;
use std::fs;
use std::path::Path;

use anyhow::{Context, Result};
use serde::Deserialize;

// ---------- Runtime form ----------

#[derive(Clone, Debug)]
pub struct RosConfig {
    pub node_name: String,
    pub namespace: String,
    /// Frame all incoming data is transformed into before insertion.
    pub reference_frame: String,
    /// Initial TF axis-arm length (meters). UI can mutate at runtime.
    pub tf_axis_length: Option<f32>,
    pub map_topic: String,
    /// Topics carrying `nav_msgs/OccupancyGrid` to render as cost overlays
    /// (cost colormap, free/unknown transparent). Each gets its own entity, so
    /// these layer on top of `map_topic`. Supports the `"*"` wildcard.
    pub costmap_topics: Vec<String>,
    /// Topics carrying `geometry_msgs/PoseStamped`. Each gets one Arrow entity.
    pub pose_topics: Vec<String>,
    /// Topics carrying `geometry_msgs/PoseArray`. Each gets one entity holding many Arrows.
    pub pose_array_topics: Vec<String>,
    pub arrow: ArrowStyle,
    /// Topics carrying `nav_msgs/Path`. Each gets one Polyline entity.
    pub path_topics: Vec<String>,
    pub path_style: PathStyle,
    /// Topics carrying `sensor_msgs/LaserScan`. Each gets one Points entity.
    pub scan_topics: Vec<String>,
    pub scan_style: ScanStyle,
    /// Topics carrying `sensor_msgs/PointCloud2`. Each gets one Points entity.
    pub point_topics: Vec<String>,
    pub point_style: PointCloudStyle,
    /// Topics carrying `visualization_msgs/Marker`.
    pub marker_topics: Vec<String>,
    /// Topics carrying `visualization_msgs/MarkerArray`.
    pub marker_array_topics: Vec<String>,
    /// Optional URDF file. When `Some`, the node loads it at startup and
    /// updates link transforms from `joint_states_topic`.
    pub urdf_path: Option<std::path::PathBuf>,
    /// Optional `std_msgs/String` topic carrying the URDF XML
    /// (e.g. `/robot_description`). If `Some` and `urdf_path` is `None`, the
    /// node subscribes (TRANSIENT_LOCAL by default) and parses the URDF as
    /// soon as the first message arrives.
    pub urdf_topic: Option<String>,
    pub urdf_topic_qos: Option<QosOverride>,
    pub joint_states_topic: String,
    /// `tf2_msgs/TFMessage` dynamic-frame topic. Defaults to `/tf`.
    pub tf_topic: String,
    /// `tf2_msgs/TFMessage` latched static-frame topic. Defaults to
    /// `/tf_static`.
    pub tf_static_topic: String,
    pub tf_qos: Option<QosOverride>,
    pub tf_static_qos: Option<QosOverride>,
    /// Per-topic QoS overrides (applied on top of each subscriber's default profile).
    pub map_qos: HashMap<String, QosOverride>,
    pub costmap_qos: HashMap<String, QosOverride>,
    pub pose_qos: HashMap<String, QosOverride>,
    pub pose_array_qos: HashMap<String, QosOverride>,
    pub path_qos: HashMap<String, QosOverride>,
    pub scan_qos: HashMap<String, QosOverride>,
    pub point_qos: HashMap<String, QosOverride>,
    pub marker_qos: HashMap<String, QosOverride>,
    pub marker_array_qos: HashMap<String, QosOverride>,
    pub joint_states_qos: Option<QosOverride>,
    /// Side-panel grouping. Each group renders as a collapsible egui section
    /// in the order it appears here. Empty = current flat list behavior.
    pub ui_groups: Vec<UiGroup>,
    /// Saved camera framing (`[view]`). When `Some`, the app restores the orbit
    /// camera to this pose at startup, the way RViz reopens a saved view.
    pub view: Option<ViewConfig>,
}

/// Persisted orbit-camera pose. Written by the "Save config" button and read
/// back at startup. Angles are radians; `target` is in renderer-world coords.
#[derive(Clone, Copy, Debug)]
pub struct ViewConfig {
    pub target: [f32; 3],
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
}

/// One side-panel group. Entities are matched by exact label (topic name) and
/// rendered in the order topics appear in `topics`. With `urdf = true`, every
/// URDF link is appended to the group (in link order). With `tf = true`, every
/// TF-frame axis entity is appended (in encounter order, which is roughly the
/// order frames first arrive on /tf or /tf_static).
#[derive(Clone, Debug)]
pub struct UiGroup {
    pub name: String,
    pub topics: Vec<String>,
    pub urdf: bool,
    pub tf: bool,
    /// Initial fold state. Toggleable in the UI; this is just the default.
    pub collapsed: bool,
}

/// Optional per-topic QoS override. Missing fields fall back to the
/// subscriber's per-kind default profile.
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct QosOverride {
    /// `"reliable"` or `"best_effort"`.
    pub reliability: Option<String>,
    /// `"volatile"`, `"transient_local"`, or `"best_available"`.
    pub durability: Option<String>,
    /// History depth (KEEP_LAST).
    pub depth: Option<usize>,
}

impl QosOverride {
    /// Apply this override on top of a base `r2r::QosProfile`. Unknown enum
    /// values are logged and ignored.
    pub fn apply(&self, mut base: r2r::QosProfile) -> r2r::QosProfile {
        use r2r::qos::{DurabilityPolicy, ReliabilityPolicy};
        if let Some(s) = &self.reliability {
            base = match s.as_str() {
                "reliable" => base.reliability(ReliabilityPolicy::Reliable),
                "best_effort" => base.reliability(ReliabilityPolicy::BestEffort),
                other => {
                    log::warn!("unknown qos.reliability {other:?}; ignoring");
                    base
                }
            };
        }
        if let Some(s) = &self.durability {
            base = match s.as_str() {
                "volatile" => base.durability(DurabilityPolicy::Volatile),
                "transient_local" => base.durability(DurabilityPolicy::TransientLocal),
                #[cfg(not(ros_humble))]
                "best_available" => base.durability(DurabilityPolicy::BestAvailable),
                #[cfg(ros_humble)]
                "best_available" => {
                    log::warn!("'best_available' durability is not supported on ROS Humble; using 'transient_local' fallback.");
                    base.durability(DurabilityPolicy::TransientLocal)
                }
                other => {
                    log::warn!("unknown qos.durability {other:?}; ignoring");
                    base
                }
            };
        }
        if let Some(d) = self.depth {
            base = base.keep_last(d);
        }
        base
    }
}

#[derive(Clone, Debug)]
pub struct ScanStyle {
    pub size: f32,
    pub color: scene::Color,
}

impl Default for ScanStyle {
    fn default() -> Self {
        ScanStyle {
            size: 4.0,
            color: scene::Color::rgb(1.0, 0.95, 0.2),
        }
    }
}

#[derive(Clone, Debug)]
pub struct PointCloudStyle {
    pub size: f32,
    pub color: scene::Color,
    /// Decimation: keep 1 of every N points (1 = all). Useful for dense LiDAR clouds.
    pub stride: usize,
}

impl Default for PointCloudStyle {
    fn default() -> Self {
        PointCloudStyle {
            size: 2.0,
            color: scene::Color::rgb(0.30, 0.85, 1.0),
            stride: 1,
        }
    }
}

#[derive(Clone, Debug)]
pub struct PathStyle {
    pub width: f32,
    pub color: scene::Color,
}

impl Default for PathStyle {
    fn default() -> Self {
        PathStyle {
            // World-space thickness in meters: the line pass renders Polylines
            // as instanced quads extruded by `width`, so this is a physical
            // width (RViz-style ~few cm), not the old pixel-ish value.
            width: 0.05,
            color: scene::Color::rgb(0.20, 0.85, 0.30),
        }
    }
}

#[derive(Clone, Debug)]
pub struct ArrowStyle {
    pub length: f32,
    pub shaft_radius: f32,
    pub head_radius: f32,
    pub color: scene::Color,
}

impl Default for ArrowStyle {
    fn default() -> Self {
        ArrowStyle {
            length: 0.5,
            shaft_radius: 0.025,
            head_radius: 0.06,
            color: scene::Color::rgb(0.95, 0.55, 0.10),
        }
    }
}

impl Default for RosConfig {
    fn default() -> Self {
        RosConfig {
            node_name: "fastviz".into(),
            namespace: String::new(),
            reference_frame: "map".into(),
            tf_axis_length: None,
            map_topic: "/map".into(),
            costmap_topics: Vec::new(),
            pose_topics: vec!["/goal_pose".into()],
            pose_array_topics: vec!["/particle_cloud".into()],
            arrow: ArrowStyle::default(),
            path_topics: vec!["/plan".into()],
            path_style: PathStyle::default(),
            scan_topics: vec!["/scan".into()],
            scan_style: ScanStyle::default(),
            point_topics: Vec::new(),
            point_style: PointCloudStyle::default(),
            marker_topics: Vec::new(),
            marker_array_topics: Vec::new(),
            urdf_path: None,
            urdf_topic: None,
            urdf_topic_qos: None,
            joint_states_topic: "/joint_states".into(),
            tf_topic: "/tf".into(),
            tf_static_topic: "/tf_static".into(),
            tf_qos: None,
            tf_static_qos: None,
            map_qos: HashMap::new(),
            costmap_qos: HashMap::new(),
            pose_qos: HashMap::new(),
            pose_array_qos: HashMap::new(),
            path_qos: HashMap::new(),
            scan_qos: HashMap::new(),
            point_qos: HashMap::new(),
            marker_qos: HashMap::new(),
            marker_array_qos: HashMap::new(),
            joint_states_qos: None,
            ui_groups: Vec::new(),
            view: None,
        }
    }
}

impl RosConfig {
    pub fn from_path(p: &Path) -> Result<Self> {
        let text = fs::read_to_string(p)
            .with_context(|| format!("reading config file {}", p.display()))?;
        let raw: RawConfig = toml::from_str(&text)
            .with_context(|| format!("parsing TOML config {}", p.display()))?;
        Ok(raw.into_runtime())
    }
}

// ---------- TOML schema ----------

fn rgb_to_color(arr: [f32; 3]) -> scene::Color {
    scene::Color::rgb(arr[0], arr[1], arr[2])
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RawConfig {
    pub node_name: Option<String>,
    pub namespace: Option<String>,
    pub reference_frame: Option<String>,
    pub map: Option<RawMap>,
    pub costmaps: Option<RawTopics>,
    pub poses: Option<RawTopics>,
    pub pose_arrays: Option<RawTopics>,
    pub arrow: Option<RawArrow>,
    pub paths: Option<RawPaths>,
    pub scans: Option<RawScans>,
    pub points: Option<RawPoints>,
    pub markers: Option<RawTopics>,
    pub marker_arrays: Option<RawTopics>,
    pub urdf: Option<RawUrdf>,
    pub tf: Option<RawTf>,
    pub ui: Option<RawUi>,
    pub view: Option<RawView>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RawView {
    /// Orbit target in renderer-world coords. Defaults to the origin.
    pub target: [f32; 3],
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RawUi {
    /// `[[ui.group]]` array of tables. Each one becomes a collapsible section
    /// in the side panel, rendered in TOML order.
    pub group: Vec<RawUiGroup>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RawUiGroup {
    pub name: String,
    /// Entity labels (topic names) that belong to this group. Matched against
    /// the first whitespace-separated token of `entity.label`, so
    /// `/map [map]` matches `topics = ["/map"]`.
    pub topics: Vec<String>,
    /// When true, every URDF link is appended to the group.
    pub urdf: bool,
    /// When true, every TF-frame axis entity is appended to the group.
    pub tf: bool,
    /// Default fold state — toggleable at runtime.
    pub collapsed: bool,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RawUrdf {
    /// Path to a URDF or xacro file on disk.
    pub path: Option<String>,
    /// `std_msgs/String` topic carrying the URDF XML (typically
    /// `/robot_description`, published by `robot_state_publisher`).
    pub topic: Option<String>,
    pub joint_states_topic: Option<String>,
    /// QoS override for `joint_states_topic`.
    pub qos: Option<QosOverride>,
    /// QoS override for the URDF topic (`topic`). Defaults to TRANSIENT_LOCAL
    /// so latched publishers are picked up.
    pub topic_qos: Option<QosOverride>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RawTf {
    /// Dynamic-frame topic, default `/tf`.
    pub topic: Option<String>,
    /// Static (latched) frame topic, default `/tf_static`.
    pub static_topic: Option<String>,
    pub qos: Option<QosOverride>,
    pub static_qos: Option<QosOverride>,
    /// Initial axis-arm length in meters; the UI can override at runtime.
    pub axis_length: Option<f32>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RawMap {
    pub topics: Vec<String>,
    pub qos: HashMap<String, QosOverride>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RawTopics {
    pub topics: Vec<String>,
    pub qos: HashMap<String, QosOverride>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RawArrow {
    pub length: f32,
    pub shaft_radius: f32,
    pub head_radius: f32,
    pub color: [f32; 3],
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RawPaths {
    pub topics: Vec<String>,
    pub style: Option<RawPathStyle>,
    pub qos: HashMap<String, QosOverride>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RawPathStyle {
    pub width: f32,
    pub color: [f32; 3],
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RawScans {
    pub topics: Vec<String>,
    pub style: Option<RawScanStyle>,
    pub qos: HashMap<String, QosOverride>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RawScanStyle {
    pub size: f32,
    pub color: [f32; 3],
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RawPoints {
    pub topics: Vec<String>,
    pub style: Option<RawPointStyle>,
    pub qos: HashMap<String, QosOverride>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RawPointStyle {
    pub size: Option<f32>,
    pub color: Option<[f32; 3]>,
    pub stride: Option<usize>,
}

impl RawConfig {
    /// Converts the parsed TOML into the runtime form. Anything not specified
    /// in the file falls back to `RosConfig::default()`.
    pub fn into_runtime(self) -> RosConfig {
        let d = RosConfig::default();

        // Take ownership of each section once so we can pull both topics and qos out.
        let RawConfig {
            node_name,
            namespace,
            reference_frame,
            map,
            costmaps,
            poses,
            pose_arrays,
            arrow,
            paths,
            scans,
            points,
            markers,
            marker_arrays,
            urdf,
            tf,
            ui,
            view,
        } = self;

        let (map_topic, map_qos) = match map {
            Some(m) => (m.topics.into_iter().next().unwrap_or(d.map_topic), m.qos),
            None => (d.map_topic, d.map_qos),
        };
        let (costmap_topics, costmap_qos) = match costmaps {
            Some(c) => (c.topics, c.qos),
            None => (d.costmap_topics, d.costmap_qos),
        };
        let (pose_topics, pose_qos) = match poses {
            Some(p) => (p.topics, p.qos),
            None => (d.pose_topics, d.pose_qos),
        };
        let (pose_array_topics, pose_array_qos) = match pose_arrays {
            Some(p) => (p.topics, p.qos),
            None => (d.pose_array_topics, d.pose_array_qos),
        };
        let arrow_style = arrow
            .map(|a| ArrowStyle {
                length: a.length,
                shaft_radius: a.shaft_radius,
                head_radius: a.head_radius,
                color: rgb_to_color(a.color),
            })
            .unwrap_or(d.arrow);
        let (path_topics, path_style, path_qos) = match paths {
            Some(p) => (
                p.topics,
                p.style
                    .map(|s| PathStyle {
                        width: s.width,
                        color: rgb_to_color(s.color),
                    })
                    .unwrap_or(d.path_style),
                p.qos,
            ),
            None => (d.path_topics, d.path_style, d.path_qos),
        };
        let (scan_topics, scan_style, scan_qos) = match scans {
            Some(s) => (
                s.topics,
                s.style
                    .map(|s| ScanStyle {
                        size: s.size,
                        color: rgb_to_color(s.color),
                    })
                    .unwrap_or(d.scan_style),
                s.qos,
            ),
            None => (d.scan_topics, d.scan_style, d.scan_qos),
        };
        let (point_topics, point_style, point_qos) = match points {
            Some(p) => {
                let dp = d.point_style;
                let style = p
                    .style
                    .map(|s| PointCloudStyle {
                        size: s.size.unwrap_or(dp.size),
                        color: s.color.map(rgb_to_color).unwrap_or(dp.color),
                        stride: s.stride.unwrap_or(dp.stride),
                    })
                    .unwrap_or(dp);
                (p.topics, style, p.qos)
            }
            None => (d.point_topics, d.point_style, d.point_qos),
        };
        let (marker_topics, marker_qos) = match markers {
            Some(m) => (m.topics, m.qos),
            None => (d.marker_topics, d.marker_qos),
        };
        let (marker_array_topics, marker_array_qos) = match marker_arrays {
            Some(m) => (m.topics, m.qos),
            None => (d.marker_array_topics, d.marker_array_qos),
        };

        let (urdf_path, urdf_topic, urdf_topic_qos, joint_states_topic, joint_states_qos) =
            match urdf {
                Some(u) => (
                    u.path.map(std::path::PathBuf::from),
                    u.topic,
                    u.topic_qos,
                    u.joint_states_topic.unwrap_or(d.joint_states_topic),
                    u.qos,
                ),
                None => (
                    d.urdf_path,
                    d.urdf_topic,
                    d.urdf_topic_qos,
                    d.joint_states_topic,
                    d.joint_states_qos,
                ),
            };

        let (tf_topic, tf_static_topic, tf_qos, tf_static_qos, tf_axis_length) = match tf {
            Some(t) => (
                t.topic.unwrap_or(d.tf_topic),
                t.static_topic.unwrap_or(d.tf_static_topic),
                t.qos,
                t.static_qos,
                t.axis_length,
            ),
            None => (
                d.tf_topic,
                d.tf_static_topic,
                d.tf_qos,
                d.tf_static_qos,
                d.tf_axis_length,
            ),
        };

        let ui_groups = ui
            .map(|u| {
                u.group
                    .into_iter()
                    .map(|g| UiGroup {
                        name: g.name,
                        topics: g.topics,
                        urdf: g.urdf,
                        tf: g.tf,
                        collapsed: g.collapsed,
                    })
                    .collect()
            })
            .unwrap_or_default();

        let view = view.map(|v| ViewConfig {
            target: v.target,
            yaw: v.yaw,
            pitch: v.pitch,
            distance: v.distance,
        });

        RosConfig {
            node_name: node_name.unwrap_or(d.node_name),
            namespace: namespace.unwrap_or(d.namespace),
            reference_frame: reference_frame.unwrap_or(d.reference_frame),
            tf_axis_length,
            map_topic,
            costmap_topics,
            pose_topics,
            pose_array_topics,
            arrow: arrow_style,
            path_topics,
            path_style,
            scan_topics,
            scan_style,
            point_topics,
            point_style,
            marker_topics,
            marker_array_topics,
            urdf_path,
            urdf_topic,
            urdf_topic_qos,
            joint_states_topic,
            tf_topic,
            tf_static_topic,
            tf_qos,
            tf_static_qos,
            map_qos,
            costmap_qos,
            pose_qos,
            pose_array_qos,
            path_qos,
            scan_qos,
            point_qos,
            marker_qos,
            marker_array_qos,
            joint_states_qos,
            ui_groups,
            view,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_toml_yields_defaults() {
        let raw: RawConfig = toml::from_str("").unwrap();
        let cfg = raw.into_runtime();
        let d = RosConfig::default();
        assert_eq!(cfg.map_topic, d.map_topic);
        assert_eq!(cfg.pose_topics, d.pose_topics);
        assert_eq!(cfg.scan_topics, d.scan_topics);
    }

    #[test]
    fn partial_overrides_apply_only_to_given_fields() {
        let toml = r#"
            reference_frame = "odom"
            [scans]
            topics = ["/scan_front", "/scan_back"]
        "#;
        let raw: RawConfig = toml::from_str(toml).unwrap();
        let cfg = raw.into_runtime();
        assert_eq!(cfg.reference_frame, "odom");
        assert_eq!(cfg.scan_topics, vec!["/scan_front", "/scan_back"]);
        // unspecified field still default
        assert_eq!(cfg.map_topic, "/map");
    }

    #[test]
    fn full_schema_round_trips() {
        let toml = r#"
            node_name = "viz"
            namespace = "/ns"
            reference_frame = "map"

            [map]
            topics = ["/cartographer_map"]

            [poses]
            topics = ["/goal_pose", "/extra_pose"]

            [pose_arrays]
            topics = ["/particle_cloud"]

            [arrow]
            length = 0.7
            shaft_radius = 0.04
            head_radius = 0.10
            color = [1.0, 0.0, 0.0]

            [paths]
            topics = ["/plan", "/global_plan"]
            style = { width = 3.0, color = [0.0, 1.0, 0.0] }

            [scans]
            topics = ["/scan"]
            style = { size = 5.0, color = [0.0, 0.0, 1.0] }
        "#;
        let raw: RawConfig = toml::from_str(toml).unwrap();
        let cfg = raw.into_runtime();
        assert_eq!(cfg.node_name, "viz");
        assert_eq!(cfg.namespace, "/ns");
        assert_eq!(cfg.map_topic, "/cartographer_map");
        assert_eq!(cfg.pose_topics.len(), 2);
        assert_eq!(cfg.path_topics, vec!["/plan", "/global_plan"]);
        assert!((cfg.arrow.length - 0.7).abs() < 1e-6);
        assert!((cfg.path_style.width - 3.0).abs() < 1e-6);
        assert!((cfg.scan_style.size - 5.0).abs() < 1e-6);
    }

    #[test]
    fn costmaps_section_parses_topics_and_qos() {
        let toml = r#"
            [costmaps]
            topics = ["/global_costmap/costmap", "/local_costmap/costmap"]

            [costmaps.qos."/local_costmap/costmap"]
            reliability = "best_effort"
        "#;
        let raw: RawConfig = toml::from_str(toml).unwrap();
        let cfg = raw.into_runtime();
        assert_eq!(
            cfg.costmap_topics,
            vec!["/global_costmap/costmap", "/local_costmap/costmap"]
        );
        assert_eq!(
            cfg.costmap_qos
                .get("/local_costmap/costmap")
                .unwrap()
                .reliability
                .as_deref(),
            Some("best_effort")
        );
        // Map is independent of costmaps.
        assert_eq!(cfg.map_topic, "/map");
    }

    #[test]
    fn unknown_field_is_rejected() {
        let bad = "totally_made_up_field = 7\n";
        let r = toml::from_str::<RawConfig>(bad);
        assert!(r.is_err(), "unknown root keys should fail");
    }

    #[test]
    fn points_section_parses_with_partial_style() {
        let toml = r#"
            [points]
            topics = ["/points2/decompressed"]
            style = { stride = 4 }
        "#;
        let raw: RawConfig = toml::from_str(toml).unwrap();
        let cfg = raw.into_runtime();
        assert_eq!(cfg.point_topics, vec!["/points2/decompressed"]);
        assert_eq!(cfg.point_style.stride, 4);
        // Unspecified fields fall back to defaults.
        let d = RosConfig::default();
        assert!((cfg.point_style.size - d.point_style.size).abs() < 1e-6);
    }

    #[test]
    fn qos_overrides_parse_per_topic() {
        let toml = r#"
            [scans]
            topics = ["/scan_front", "/scan_back"]

            [scans.qos."/scan_back"]
            reliability = "reliable"
            depth = 20
        "#;
        let raw: RawConfig = toml::from_str(toml).unwrap();
        let cfg = raw.into_runtime();
        assert_eq!(cfg.scan_qos.len(), 1);
        let q = cfg.scan_qos.get("/scan_back").unwrap();
        assert_eq!(q.reliability.as_deref(), Some("reliable"));
        assert_eq!(q.depth, Some(20));
        assert!(q.durability.is_none());
        // Other topic has no override
        assert!(!cfg.scan_qos.contains_key("/scan_front"));
    }

    #[test]
    fn urdf_topic_section_parses() {
        let toml = r#"
            [urdf]
            topic = "/robot_description"
            joint_states_topic = "/jstates"
            [urdf.topic_qos]
            durability = "transient_local"
        "#;
        let raw: RawConfig = toml::from_str(toml).unwrap();
        let cfg = raw.into_runtime();
        assert_eq!(cfg.urdf_topic.as_deref(), Some("/robot_description"));
        assert_eq!(cfg.urdf_path, None);
        assert_eq!(cfg.joint_states_topic, "/jstates");
        assert_eq!(
            cfg.urdf_topic_qos.as_ref().unwrap().durability.as_deref(),
            Some("transient_local")
        );
    }

    #[test]
    fn tf_section_overrides_topics() {
        let toml = r#"
            [tf]
            topic        = "/robot/tf"
            static_topic = "/robot/tf_static"
            [tf.qos]
            reliability = "best_effort"
        "#;
        let raw: RawConfig = toml::from_str(toml).unwrap();
        let cfg = raw.into_runtime();
        assert_eq!(cfg.tf_topic, "/robot/tf");
        assert_eq!(cfg.tf_static_topic, "/robot/tf_static");
        assert_eq!(
            cfg.tf_qos.as_ref().unwrap().reliability.as_deref(),
            Some("best_effort")
        );
        assert!(cfg.tf_static_qos.is_none());
    }

    #[test]
    fn ui_groups_parse_in_order() {
        let toml = r#"
            [[ui.group]]
            name   = "Sensors"
            topics = ["/scan", "/points"]

            [[ui.group]]
            name      = "Robot Description"
            urdf      = true
            collapsed = true
        "#;
        let raw: RawConfig = toml::from_str(toml).unwrap();
        let cfg = raw.into_runtime();
        assert_eq!(cfg.ui_groups.len(), 2);
        assert_eq!(cfg.ui_groups[0].name, "Sensors");
        assert_eq!(cfg.ui_groups[0].topics, vec!["/scan", "/points"]);
        assert!(!cfg.ui_groups[0].urdf);
        assert!(!cfg.ui_groups[0].collapsed);
        assert_eq!(cfg.ui_groups[1].name, "Robot Description");
        assert!(cfg.ui_groups[1].urdf);
        assert!(cfg.ui_groups[1].collapsed);
    }

    #[test]
    fn view_section_parses() {
        let toml = r#"
            [view]
            target = [1.0, 2.0, 3.0]
            yaw = 0.5
            pitch = 0.7
            distance = 12.0
        "#;
        let raw: RawConfig = toml::from_str(toml).unwrap();
        let cfg = raw.into_runtime();
        let v = cfg.view.expect("view present");
        assert_eq!(v.target, [1.0, 2.0, 3.0]);
        assert!((v.yaw - 0.5).abs() < 1e-6);
        assert!((v.pitch - 0.7).abs() < 1e-6);
        assert!((v.distance - 12.0).abs() < 1e-6);
    }

    #[test]
    fn view_absent_when_omitted() {
        let raw: RawConfig = toml::from_str("").unwrap();
        assert!(raw.into_runtime().view.is_none());
    }

    #[test]
    fn tf_defaults_when_omitted() {
        let raw: RawConfig = toml::from_str("").unwrap();
        let cfg = raw.into_runtime();
        assert_eq!(cfg.tf_topic, "/tf");
        assert_eq!(cfg.tf_static_topic, "/tf_static");
    }

    #[test]
    fn qos_apply_modifies_base_profile() {
        let q = QosOverride {
            reliability: Some("best_effort".into()),
            durability: Some("transient_local".into()),
            depth: Some(7),
        };
        let p = q.apply(r2r::QosProfile::default());
        assert_eq!(p.reliability, r2r::qos::ReliabilityPolicy::BestEffort);
        assert_eq!(p.durability, r2r::qos::DurabilityPolicy::TransientLocal);
        assert_eq!(p.depth, 7);
    }
}
