//! Runtime configuration for the ROS node.
//!
//! Two related types live here:
//! - [`RawConfig`]: the TOML-deserialisable schema (uses `[f32; 3]` for colors,
//!   plain `Vec<String>` for topics, etc.). This is what `fastviz.toml` maps to.
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
    pub map_topic: String,
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
    /// Per-topic QoS overrides (applied on top of each subscriber's default profile).
    pub map_qos: HashMap<String, QosOverride>,
    pub pose_qos: HashMap<String, QosOverride>,
    pub pose_array_qos: HashMap<String, QosOverride>,
    pub path_qos: HashMap<String, QosOverride>,
    pub scan_qos: HashMap<String, QosOverride>,
    pub point_qos: HashMap<String, QosOverride>,
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
                "best_available" => base.durability(DurabilityPolicy::BestAvailable),
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
            width: 2.0,
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
            map_topic: "/map".into(),
            pose_topics: vec!["/goal_pose".into()],
            pose_array_topics: vec!["/particle_cloud".into()],
            arrow: ArrowStyle::default(),
            path_topics: vec!["/plan".into()],
            path_style: PathStyle::default(),
            scan_topics: vec!["/scan".into()],
            scan_style: ScanStyle::default(),
            point_topics: Vec::new(),
            point_style: PointCloudStyle::default(),
            map_qos: HashMap::new(),
            pose_qos: HashMap::new(),
            pose_array_qos: HashMap::new(),
            path_qos: HashMap::new(),
            scan_qos: HashMap::new(),
            point_qos: HashMap::new(),
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
    pub poses: Option<RawTopics>,
    pub pose_arrays: Option<RawTopics>,
    pub arrow: Option<RawArrow>,
    pub paths: Option<RawPaths>,
    pub scans: Option<RawScans>,
    pub points: Option<RawPoints>,
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
            poses,
            pose_arrays,
            arrow,
            paths,
            scans,
            points,
        } = self;

        let (map_topic, map_qos) = match map {
            Some(m) => (m.topics.into_iter().next().unwrap_or(d.map_topic), m.qos),
            None => (d.map_topic, d.map_qos),
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

        RosConfig {
            node_name: node_name.unwrap_or(d.node_name),
            namespace: namespace.unwrap_or(d.namespace),
            reference_frame: reference_frame.unwrap_or(d.reference_frame),
            map_topic,
            pose_topics,
            pose_array_topics,
            arrow: arrow_style,
            path_topics,
            path_style,
            scan_topics,
            scan_style,
            point_topics,
            point_style,
            map_qos,
            pose_qos,
            pose_array_qos,
            path_qos,
            scan_qos,
            point_qos,
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
