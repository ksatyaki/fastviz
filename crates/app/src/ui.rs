//! Egui panel layout: top toolbar + left entity list.

use std::collections::HashMap;
#[cfg(feature = "ros")]
use std::collections::HashSet;

use egui::{Align, Color32, FontFamily, FontId, Layout, RichText};
use renderer::{FrameStats, OrbitCamera};
use scene::{EntityId, ScenePrimitive, SceneGraph};

use crate::theme;

#[cfg(feature = "ros")]
const URDF_LINK_BASE: u64 = ros_node::URDF_LINK_BASE;
#[cfg(not(feature = "ros"))]
const URDF_LINK_BASE: u64 = u64::MAX;

#[cfg(feature = "ros")]
const TF_FRAME_BASE: u64 = ros_node::TF_FRAME_BASE;
#[cfg(feature = "ros")]
const TF_FRAME_CAPACITY: u64 = ros_node::TF_FRAME_CAPACITY;
#[cfg(not(feature = "ros"))]
const TF_FRAME_BASE: u64 = u64::MAX;
#[cfg(not(feature = "ros"))]
const TF_FRAME_CAPACITY: u64 = 0;

#[cfg(feature = "ros")]
const MARKER_BASE: u64 = ros_node::ROS_ID_MARKER_BASE;
#[cfg(not(feature = "ros"))]
const MARKER_BASE: u64 = u64::MAX;

fn is_marker_entity(id: EntityId) -> bool {
    id.0 >= MARKER_BASE
}

/// Best-effort topic name for an entity, from its label. Returns `None` for
/// entities whose label doesn't encode a topic (TF axes, URDF links, untitled).
fn topic_for_entity(id: EntityId, label: Option<&str>) -> Option<&str> {
    let label = label?;
    // TF / URDF use non-topic labels; they're grouped separately.
    let in_tf_range =
        id.0 >= TF_FRAME_BASE && id.0 < TF_FRAME_BASE.saturating_add(TF_FRAME_CAPACITY);
    let in_urdf_range =
        id.0 >= URDF_LINK_BASE && id.0 < URDF_LINK_BASE.saturating_add(1_000_000);
    if in_tf_range || in_urdf_range {
        return None;
    }
    // Marker labels look like "topic [id]" or "topic [ns/id]". Other subscribers
    // set the label to the topic name directly; occupancy uses "/map [frame]"
    // which still leaves "/map" as the first whitespace token.
    let head = label.split_whitespace().next().unwrap_or(label);
    if head.is_empty() {
        None
    } else {
        Some(head)
    }
}

/// Parse a marker label of the form `"<topic> [<ns>/<id>]"` or
/// `"<topic> [<id>]"`. Returns `(topic, ns_or_none, id_str)`.
fn parse_marker_label(label: &str) -> Option<(&str, Option<&str>, &str)> {
    let bracket_open = label.find(" [")?;
    let topic = &label[..bracket_open];
    let inside_start = bracket_open + 2;
    let close_rel = label[inside_start..].rfind(']')?;
    let inside = &label[inside_start..inside_start + close_rel];
    if let Some(slash) = inside.find('/') {
        Some((topic, Some(&inside[..slash]), &inside[slash + 1..]))
    } else {
        Some((topic, None, inside))
    }
}

/// Tri-state for a bulk visibility checkbox over N entities: all visible, none
/// visible, or mixed. The eye toggle uses this to drive a single click that
/// flips everything to the opposite of the dominant state.
#[derive(Copy, Clone, PartialEq)]
enum BulkVis {
    All,
    None,
    Mixed,
}

fn bulk_state<'a, I: IntoIterator<Item = &'a EntityId>>(
    scene: &SceneGraph,
    ids: I,
) -> BulkVis {
    let (mut all_on, mut all_off, mut empty) = (true, true, true);
    for id in ids {
        if let Some(e) = scene.entities.get(id) {
            empty = false;
            if e.visible {
                all_off = false;
            } else {
                all_on = false;
            }
        }
    }
    if empty {
        BulkVis::None
    } else if all_on {
        BulkVis::All
    } else if all_off {
        BulkVis::None
    } else {
        BulkVis::Mixed
    }
}

fn set_bulk_visible(scene: &mut SceneGraph, ids: &[EntityId], visible: bool) {
    for id in ids {
        scene.set_visible(*id, visible);
    }
}

/// Render the eye visibility toggle for a bucket of entities. Click flips
/// to the opposite of the dominant state (All -> None, anything-else -> All).
/// The indicator is painted directly so it doesn't depend on font glyphs —
/// previously we used ●/○/◐, but IBM Plex Sans renders U+25CF as a notdef
/// square and egui's fallback chain doesn't override an existing glyph.
fn eye_toggle(
    ui: &mut egui::Ui,
    scene: &mut SceneGraph,
    ids: &[EntityId],
    salt: impl std::hash::Hash,
) {
    let _ = salt;
    let state = bulk_state(scene, ids);
    let hover = match state {
        BulkVis::All => "Visible \u{2014} click to hide",
        BulkVis::None => "Hidden \u{2014} click to show",
        BulkVis::Mixed => "Mixed \u{2014} click to show all",
    };

    let size = egui::vec2(22.0, ui.spacing().interact_size.y.max(18.0));
    let (rect, resp) = ui.allocate_exact_size(size, egui::Sense::click());
    let resp = resp.on_hover_text(hover);

    let visuals = ui.style().interact(&resp);
    let center = rect.center();
    let painter = ui.painter();
    let stroke_color = if resp.hovered() {
        theme::accent()
    } else {
        visuals.fg_stroke.color
    };
    let stroke = egui::Stroke::new(1.4, stroke_color);

    // Almond outline (eye shape): ellipse approximated by a closed polyline.
    let hw = 8.5; // half-width
    let hh = 4.5; // half-height
    let segs = 24;
    let outline: Vec<egui::Pos2> = (0..segs)
        .map(|i| {
            let t = (i as f32) / (segs as f32) * std::f32::consts::TAU;
            egui::pos2(center.x + hw * t.cos(), center.y + hh * t.sin())
        })
        .collect();
    painter.add(egui::Shape::closed_line(outline, stroke));

    match state {
        BulkVis::All => {
            // Iris + pupil — solid accent so "visible" reads at a glance.
            painter.circle_filled(center, 3.0, theme::accent());
            painter.circle_filled(center, 1.2, Color32::BLACK);
        }
        BulkVis::Mixed => {
            painter.circle_stroke(center, 3.0, stroke);
            painter.circle_filled(center, 1.2, stroke_color);
        }
        BulkVis::None => {
            // Diagonal slash across the eye, classic "hidden" affordance.
            let off = hw + 1.5;
            let off_y = hh + 2.5;
            painter.line_segment(
                [
                    egui::pos2(center.x - off, center.y + off_y),
                    egui::pos2(center.x + off, center.y - off_y),
                ],
                stroke,
            );
        }
    }

    if resp.clicked() {
        let target = !matches!(state, BulkVis::All);
        set_bulk_visible(scene, ids, target);
    }
}

#[derive(Copy, Clone, Default)]
pub struct Pc2Stats {
    pub received: u64,
    pub displayed: u64,
}

/// One side-panel group, materialised from the TOML config plus the live fold
/// state the panel tracks across frames.
#[derive(Clone, Debug)]
pub struct UiGroupView {
    pub name: String,
    pub topics: Vec<String>,
    pub urdf: bool,
    pub tf: bool,
    /// Current fold state — starts at `collapsed` from TOML, then user-driven.
    pub open: bool,
}

impl UiGroupView {
    /// Convert the parsed TOML groups into UI views. If no group claims TF
    /// frames, a default "TF Frames" group is appended (collapsed) so every
    /// frame entity has a home.
    #[cfg(feature = "ros")]
    pub fn from_config(groups: &[ros_node::UiGroup]) -> Vec<Self> {
        let mut out: Vec<UiGroupView> = groups
            .iter()
            .map(|g| UiGroupView {
                name: g.name.clone(),
                topics: g.topics.clone(),
                urdf: g.urdf,
                tf: g.tf,
                open: !g.collapsed,
            })
            .collect();
        if !out.iter().any(|g| g.tf) {
            out.push(UiGroupView {
                name: "TF Frames".to_string(),
                topics: Vec::new(),
                urdf: false,
                tf: true,
                open: false,
            });
        }
        out
    }
}

/// Persistent state for the "Add topics" window (RViz-style live subscribe).
/// Owned by the app so the window's fold state survives across frames.
#[cfg_attr(not(feature = "ros"), allow(dead_code))]
#[derive(Default)]
pub struct TopicDiscovererState {
    pub open: bool,
    /// One-line status message rendered under the topic tree.
    pub status: Option<String>,
}

/// Persistent state for the "Save config" window. Owned by the app so the
/// typed filename and last status survive across frames.
#[cfg_attr(not(feature = "ros"), allow(dead_code))]
#[derive(Default)]
pub struct SaveConfigState {
    pub open: bool,
    /// Config name the user types (without extension); saved to the CWD.
    pub name: String,
    /// One-line status (resolved save path or error) under the Save button.
    pub status: Option<String>,
}

/// Persistent state for the "Load config" window. Owned by the app so the
/// typed path and last status survive across frames.
#[cfg_attr(not(feature = "ros"), allow(dead_code))]
#[derive(Default)]
pub struct LoadConfigState {
    pub open: bool,
    /// Path the user typed (relative paths resolve against the CWD).
    pub path: String,
    /// One-line status (result of the last load attempt), set by `main.rs`
    /// after it actually applies the config.
    pub status: Option<String>,
}

/// Persistent state for the "Publish" window — pick a topic + type, edit a JSON
/// body, publish once or at a fixed rate. Owned by the app so it survives across
/// frames.
#[cfg_attr(not(feature = "ros"), allow(dead_code))]
pub struct PublishState {
    pub open: bool,
    pub topic: String,
    pub type_name: String,
    pub json: String,
    pub rate_hz: f32,
    /// When true, publish every `1/rate_hz` seconds (driven by the redraw loop).
    pub repeat: bool,
    /// One-line status (last publish result / JSON parse error).
    pub status: Option<String>,
    /// Wall-clock of the last rate-driven publish; `None` until the first tick.
    pub last_publish: Option<std::time::Instant>,
}

impl Default for PublishState {
    fn default() -> Self {
        PublishState {
            open: false,
            topic: String::new(),
            type_name: String::new(),
            json: String::new(),
            rate_hz: 10.0,
            repeat: false,
            status: None,
            last_publish: None,
        }
    }
}

/// State for the interactive `/goal_pose` tool. When `armed`, the next viewport
/// left-drag sets a ground-plane pose that's published on release.
#[cfg_attr(not(feature = "ros"), allow(dead_code))]
pub struct GoalToolState {
    pub topic: String,
    pub armed: bool,
}

impl Default for GoalToolState {
    fn default() -> Self {
        GoalToolState {
            topic: "/goal_pose".to_string(),
            armed: false,
        }
    }
}

/// State for the interactive pose-estimate tool (`/initialpose`). When
/// `armed`, the next viewport left-drag sets a ground-plane pose that's
/// published as a `PoseWithCovarianceStamped` on release — the message type
/// both AMCL and slam_toolbox subscribe to for initial-pose seeding.
#[cfg_attr(not(feature = "ros"), allow(dead_code))]
pub struct PoseEstimateToolState {
    pub topic: String,
    pub armed: bool,
}

impl Default for PoseEstimateToolState {
    fn default() -> Self {
        PoseEstimateToolState {
            topic: "/initialpose".to_string(),
            armed: false,
        }
    }
}

/// Built-in JSON starter template for a message type. Returns `"{}"` for types
/// without a bundled template — the user edits the body by hand. Schema-driven
/// form generation is intentionally out of scope (see the plan's Open Questions).
#[cfg_attr(not(feature = "ros"), allow(dead_code))]
pub fn template_for(type_name: &str) -> &'static str {
    match type_name {
        "geometry_msgs/msg/PoseStamped" => {
            "{\n  \"header\": { \"frame_id\": \"map\" },\n  \"pose\": {\n    \"position\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\n    \"orientation\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0, \"w\": 1.0 }\n  }\n}"
        }
        "geometry_msgs/msg/PoseWithCovarianceStamped" => {
            "{\n  \"header\": { \"frame_id\": \"map\" },\n  \"pose\": {\n    \"pose\": {\n      \"position\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\n      \"orientation\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0, \"w\": 1.0 }\n    },\n    \"covariance\": [0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0]\n  }\n}"
        }
        "geometry_msgs/msg/PointStamped" => {
            "{\n  \"header\": { \"frame_id\": \"map\" },\n  \"point\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 }\n}"
        }
        "geometry_msgs/msg/Twist" => {
            "{\n  \"linear\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\n  \"angular\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 }\n}"
        }
        "geometry_msgs/msg/TwistStamped" => {
            "{\n  \"header\": { \"frame_id\": \"base_link\" },\n  \"twist\": {\n    \"linear\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 },\n    \"angular\": { \"x\": 0.0, \"y\": 0.0, \"z\": 0.0 }\n  }\n}"
        }
        "std_msgs/msg/String" => "{\n  \"data\": \"\"\n}",
        "std_msgs/msg/Bool" => "{\n  \"data\": false\n}",
        "std_msgs/msg/Int32" => "{\n  \"data\": 0\n}",
        "std_msgs/msg/Float64" => "{\n  \"data\": 0.0\n}",
        _ => "{}",
    }
}

/// Inputs the topic discoverer needs from outside.
#[cfg(feature = "ros")]
#[derive(Copy, Clone)]
pub struct TopicDiscovererCtx<'a> {
    pub topics: &'a ros_node::TopicsSnapshot,
    pub reference_frame: &'a str,
    /// Topics currently subscribed (from the running config). Used to
    /// pre-check the matching rows in the discoverer.
    pub active_topics: &'a [String],
}

/// In-progress hex strings for per-entity color edits. Keyed by entity id so
/// the textbox state survives across frames without persistent egui memory.
#[derive(Default)]
pub struct EntityEditState {
    hex: HashMap<EntityId, String>,
}

/// Single-selection model for the entities panel. UI-only — never persisted
/// to config.
#[derive(Default)]
pub struct SelectionState {
    pub selected: Option<EntityId>,
}

/// Floating "Edit style" popup: which entity it's editing and whether it's
/// currently open. Opened via the row context menu or the pinned Edit button.
#[derive(Default)]
pub struct EditPopupState {
    pub open: bool,
    pub target: Option<EntityId>,
}

/// Whether an entity can be edited via the style popup: it must expose a
/// color or scale control. `Frame` uses the global TF-size control instead of
/// a per-entity one, and `Grid(Cells)` has no single color/scale.
fn is_editable(p: &ScenePrimitive) -> bool {
    scene::primitive_color(p).is_some()
        || (scene::primitive_scale(p).is_some() && !matches!(p, ScenePrimitive::Frame(_)))
}

#[allow(clippy::too_many_arguments)]
pub fn draw(
    ctx: &egui::Context,
    scene: &mut SceneGraph,
    camera: &mut OrbitCamera,
    stats: FrameStats,
    show_reference_grid: &mut bool,
    pc2: Pc2Stats,
    groups: &mut [UiGroupView],
    tf_axis_length: &mut f32,
    discoverer: &mut TopicDiscovererState,
    save_state: &mut SaveConfigState,
    load_state: &mut LoadConfigState,
    edit_state: &mut EntityEditState,
    selection: &mut SelectionState,
    edit_popup: &mut EditPopupState,
    follow_frame: &mut Option<String>,
    theme_mode: &mut theme::Mode,
    // `remove_requests`: topics the user clicked ✕ on this frame. Always
    // collected (the side-panel tree is feature-agnostic); only forwarded to the
    // ROS node in ROS builds.
    remove_requests: &mut Vec<String>,
    #[cfg(feature = "ros")] topic_ctx: Option<TopicDiscovererCtx<'_>>,
    // `add_requests`: live "Add topic" requests collected this frame; the app
    // forwards them to the ROS node and appends them to the active-topic set.
    #[cfg(feature = "ros")] add_requests: &mut Vec<(String, ros_node::TopicKind)>,
    #[cfg(feature = "ros")] publish_state: &mut PublishState,
    #[cfg(feature = "ros")] goal_state: &mut GoalToolState,
    #[cfg(feature = "ros")] pose_estimate_state: &mut PoseEstimateToolState,
    // `publish_requests`: one-shot publishes collected this frame (publish pane's
    // "Publish" button). Rate publishing is driven from `main.rs`.
    #[cfg(feature = "ros")] publish_requests: &mut Vec<ros_node::PublishRequest>,
    #[cfg(feature = "ros")] load_request: &mut Option<std::path::PathBuf>,
) {
    // Build the list of TF frames currently in the scene for the follow-frame
    // dropdown. Cheap — there are typically tens of frames.
    let mut available_frames: Vec<String> = scene
        .entities
        .values()
        .filter_map(|e| {
            e.label
                .as_deref()
                .and_then(|l| l.strip_prefix("tf: "))
                .map(|s| s.to_string())
        })
        .collect();
    available_frames.sort();
    available_frames.dedup();
    egui::TopBottomPanel::top("toolbar")
        .exact_height(36.0)
        .show(ctx, |ui| {
            ui.add_space(2.0);
            ui.horizontal(|ui| {
                // Brand mark — Plex Medium in accent. Logo art will replace
                // this later; the typeset wordmark is the placeholder.
                ui.label(
                    RichText::new("FastViz")
                        .font(FontId::new(18.0, FontFamily::Proportional))
                        .color(theme::accent()),
                );
                #[cfg(feature = "ros")]
                {
                    ui.add_space(6.0);
                    ui.separator();
                    if ui
                        .button("Publish…")
                        .on_hover_text("Publish a JSON message to a ROS topic")
                        .clicked()
                    {
                        publish_state.open = !publish_state.open;
                    }
                    ui.separator();
                    ui.label("Goal");
                    ui.add(
                        egui::TextEdit::singleline(&mut goal_state.topic)
                            .desired_width(110.0)
                            .hint_text("/goal_pose"),
                    )
                    .on_hover_text("Topic the goal-pose tool publishes to");
                    let btn = egui::Button::new("Set Goal").fill(if goal_state.armed {
                        theme::accent()
                    } else {
                        ui.visuals().widgets.inactive.bg_fill
                    });
                    if ui
                        .add(btn)
                        .on_hover_text(
                            "Click, then click-drag on the ground to set position + heading",
                        )
                        .clicked()
                    {
                        goal_state.armed = !goal_state.armed;
                        if goal_state.armed {
                            pose_estimate_state.armed = false;
                        }
                    }
                    ui.separator();
                    ui.label("Pose Est.");
                    ui.add(
                        egui::TextEdit::singleline(&mut pose_estimate_state.topic)
                            .desired_width(110.0)
                            .hint_text("/initialpose"),
                    )
                    .on_hover_text("Topic the pose-estimate tool publishes to");
                    let btn = egui::Button::new("Set Pose").fill(if pose_estimate_state.armed {
                        theme::accent()
                    } else {
                        ui.visuals().widgets.inactive.bg_fill
                    });
                    if ui
                        .add(btn)
                        .on_hover_text(
                            "Click, then click-drag on the ground to set the localization pose estimate",
                        )
                        .clicked()
                    {
                        pose_estimate_state.armed = !pose_estimate_state.armed;
                        if pose_estimate_state.armed {
                            goal_state.armed = false;
                        }
                    }
                }
            });
        });

    egui::TopBottomPanel::bottom("viewbar")
        .exact_height(32.0)
        .show(ctx, |ui| {
            ui.horizontal_centered(|ui| {
                if ui.button("Reset view").on_hover_text("F").clicked() {
                    camera.reset_default();
                }
                if ui.button("Top").on_hover_text("T").clicked() {
                    camera.top_down();
                }
                if ui.button("Side").on_hover_text("S").clicked() {
                    camera.side();
                }
                ui.separator();
                ui.checkbox(show_reference_grid, "Grid");
                ui.separator();
                ui.label("Follow");
                let current = follow_frame.clone().unwrap_or_else(|| "—".to_string());
                egui::ComboBox::from_id_salt("follow_frame_combo")
                    .selected_text(current)
                    .show_ui(ui, |ui| {
                        if ui
                            .selectable_label(follow_frame.is_none(), "—")
                            .clicked()
                        {
                            *follow_frame = None;
                        }
                        for name in &available_frames {
                            let selected = follow_frame.as_deref() == Some(name.as_str());
                            if ui.selectable_label(selected, name).clicked() {
                                *follow_frame = Some(name.clone());
                            }
                        }
                    });

                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    // Theme toggle — sun in dark mode (click to go light),
                    // moon in light mode. Unicode glyphs render via egui's
                    // default emoji fallback fonts.
                    let (icon, hover) = match *theme_mode {
                        theme::Mode::Dark => ("\u{2600}", "Switch to light theme"),
                        theme::Mode::Light => ("\u{263E}", "Switch to dark theme"),
                    };
                    if ui
                        .add(egui::Button::new(RichText::new(icon).size(15.0)).frame(false))
                        .on_hover_text(hover)
                        .clicked()
                    {
                        *theme_mode = theme_mode.toggled();
                    }
                    ui.separator();
                    let dt = stats.smoothed_frame_seconds;
                    let fps = if dt > 1e-6 { 1.0 / dt } else { 0.0 };
                    ui.label(
                        RichText::new(format!("{fps:5.1} fps  {:.1} ms", dt * 1000.0))
                            .monospace(),
                    );
                    if pc2.received > 0 {
                        ui.separator();
                        let dropped = pc2.received.saturating_sub(pc2.displayed);
                        let pct = 100.0 * (dropped as f64) / (pc2.received as f64);
                        ui.label(
                            RichText::new(format!(
                                "PC2 {}/{}  drop {pct:.1}%",
                                pc2.displayed, pc2.received
                            ))
                            .monospace(),
                        );
                    }
                });
            });
        });

    egui::SidePanel::left("entities")
        .resizable(true)
        .default_width(260.0)
        .show(ctx, |ui| {
            ui.add_space(4.0);
            #[cfg(feature = "ros")]
            {
                ui.horizontal(|ui| {
                    if ui
                        .button("Add")
                        .on_hover_text("Subscribe to a topic on the live ROS graph")
                        .clicked()
                    {
                        discoverer.open = !discoverer.open;
                    }
                    if ui
                        .button("Save config…")
                        .on_hover_text("Save displayed topics + current view to a config file")
                        .clicked()
                    {
                        save_state.open = !save_state.open;
                    }
                    if ui
                        .button("Load config…")
                        .on_hover_text(
                            "Load topics + styles + view from a config file (replaces the current session)",
                        )
                        .clicked()
                    {
                        load_state.open = !load_state.open;
                    }
                });
                ui.add_space(4.0);
            }
            #[cfg(not(feature = "ros"))]
            {
                let _ = &discoverer;
                let _ = &save_state;
                let _ = &load_state;
            }
            ui.heading("Entities");
            ui.label(
                RichText::new(format!("reference frame · {}", scene.reference_frame))
                    .small()
                    .weak(),
            );
            ui.add_space(2.0);
            ui.separator();

            // Build a stable, sorted view of entity IDs once per frame.
            let mut all_ids: Vec<EntityId> = scene.entities.keys().copied().collect();
            all_ids.sort();

            // Assign each entity to at most one group (first match wins).
            let assigned: Vec<Option<usize>> = all_ids
                .iter()
                .map(|id| {
                    let label = scene.entities[id].label.as_deref().unwrap_or("");
                    let label_token = label.split_whitespace().next().unwrap_or("");
                    let is_urdf =
                        id.0 >= URDF_LINK_BASE && id.0 < URDF_LINK_BASE.saturating_add(1_000_000);
                    let is_tf = id.0 >= TF_FRAME_BASE
                        && id.0 < TF_FRAME_BASE.saturating_add(TF_FRAME_CAPACITY);
                    groups.iter().position(|g| {
                        (g.urdf && is_urdf)
                            || (g.tf && is_tf)
                            || g.topics.iter().any(|t| t == label_token)
                    })
                })
                .collect();

            // Bottom-up so the Edit row is pinned below the scroll area
            // (added first => placed at the very bottom) while the
            // ScrollArea (added last) claims the remaining space above it.
            ui.with_layout(Layout::bottom_up(Align::Min), |ui| {
                let enabled = selection
                    .selected
                    .and_then(|id| scene.entities.get(&id))
                    .map(|e| is_editable(&e.primitive))
                    .unwrap_or(false);
                if ui
                    .add_enabled(enabled, egui::Button::new("Edit size/shape…"))
                    .clicked()
                {
                    edit_popup.target = selection.selected;
                    edit_popup.open = true;
                }
                ui.separator();

                // The scroll area's inner content inherits whatever layout
                // direction is active on `ui` when `.show()` is called (it
                // doesn't force top-down itself). Without this explicit
                // override it would inherit the bottom-up layout above and
                // stack entities upward, off the top of the viewport.
                ui.with_layout(Layout::top_down(Align::Min), |ui| {
                    egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                        for (gi, group) in groups.iter_mut().enumerate() {
                            let members: Vec<EntityId> = all_ids
                                .iter()
                                .zip(assigned.iter())
                                .filter_map(|(id, a)| (*a == Some(gi)).then_some(*id))
                                .collect();
                            let show_tf_scale = group.tf;
                            draw_group(
                                ui,
                                scene,
                                group,
                                &members,
                                selection,
                                edit_popup,
                                if show_tf_scale { Some(tf_axis_length) } else { None },
                                remove_requests,
                            );
                        }

                        let ungrouped: Vec<EntityId> = all_ids
                            .iter()
                            .zip(assigned.iter())
                            .filter_map(|(id, a)| a.is_none().then_some(*id))
                            .collect();
                        if !ungrouped.is_empty() {
                            if !groups.is_empty() {
                                ui.separator();
                            }
                            // Synthetic group for orphan entities. Collapsed by default
                            // and topic-bucketed inside, same as the configured groups.
                            let mut synth = UiGroupView {
                                name: "Ungrouped".to_string(),
                                topics: Vec::new(),
                                urdf: false,
                                tf: false,
                                open: false,
                            };
                            draw_group(
                                ui,
                                scene,
                                &mut synth,
                                &ungrouped,
                                selection,
                                edit_popup,
                                None,
                                remove_requests,
                            );
                        }
                    });
                });
            });
        });

    draw_edit_popup(ctx, scene, edit_popup, edit_state);

    #[cfg(feature = "ros")]
    if let Some(tctx) = topic_ctx {
        draw_add_topics(ctx, discoverer, tctx, add_requests);
        draw_save_config(ctx, save_state, tctx, scene, groups, *tf_axis_length, camera);
        draw_load_config(ctx, load_state, load_request);
        draw_publish(ctx, publish_state, tctx, publish_requests);
    }
}

/// A namespace tree built from supported topic names, split on `/`. Each node
/// holds child namespaces (sorted) and the leaf topics that live directly at
/// this level. Lets the Add window collapse `/robot1/...` and add a whole
/// subtree or a single topic.
#[cfg(feature = "ros")]
#[derive(Default)]
struct NsNode {
    children: std::collections::BTreeMap<String, NsNode>,
    /// `(full_topic, kind)` for leaves directly at this level.
    leaves: Vec<(String, ros_node::TopicKind)>,
}

#[cfg(feature = "ros")]
impl NsNode {
    fn insert(&mut self, topic: &str, kind: ros_node::TopicKind) {
        let segments: Vec<&str> = topic.trim_start_matches('/').split('/').collect();
        let mut node = self;
        // Descend through every segment except the last (the leaf name).
        for seg in &segments[..segments.len().saturating_sub(1)] {
            node = node.children.entry((*seg).to_string()).or_default();
        }
        node.leaves.push((topic.to_string(), kind));
    }

    /// Total leaf count in this subtree.
    fn leaf_count(&self) -> usize {
        self.leaves.len() + self.children.values().map(NsNode::leaf_count).sum::<usize>()
    }

    /// Collect every leaf in this subtree not already in `active` into `out`.
    fn collect_leaves(&self, active: &HashSet<&str>, out: &mut Vec<(String, ros_node::TopicKind)>) {
        for (topic, kind) in &self.leaves {
            if !active.contains(topic.as_str()) {
                out.push((topic.clone(), *kind));
            }
        }
        for child in self.children.values() {
            child.collect_leaves(active, out);
        }
    }
}

/// "Add topics" window — RViz-style live subscribe. Supported topics are shown
/// first as a collapsible namespace tree (collapsed by default); each namespace
/// can add its whole subtree, each leaf can be added individually. Unsupported
/// topics are tucked into a collapsed section at the bottom.
#[cfg(feature = "ros")]
fn draw_add_topics(
    ctx: &egui::Context,
    state: &mut TopicDiscovererState,
    tctx: TopicDiscovererCtx<'_>,
    add_requests: &mut Vec<(String, ros_node::TopicKind)>,
) {
    let mut open = state.open;
    egui::Window::new("Add topics")
        .resizable(true)
        .default_width(460.0)
        .default_height(440.0)
        .open(&mut open)
        .show(ctx, |ui| {
            let snapshot: Vec<(String, Vec<String>)> = tctx.topics.read().clone();
            let active: HashSet<&str> =
                tctx.active_topics.iter().map(String::as_str).collect();

            // Partition into supported (build a namespace tree) and unsupported.
            let mut root = NsNode::default();
            let mut supported_count = 0usize;
            let mut unsupported: Vec<(&str, String)> = Vec::new();
            for (topic, types) in &snapshot {
                match ros_node::TopicKind::from_types(types) {
                    Some(kind) => {
                        root.insert(topic, kind);
                        supported_count += 1;
                    }
                    None => {
                        let ty = types.first().cloned().unwrap_or_default();
                        unsupported.push((topic.as_str(), ty));
                    }
                }
            }

            ui.label(format!(
                "{} topic(s) on the ROS graph — {supported_count} supported, {} unsupported.",
                snapshot.len(),
                unsupported.len()
            ));
            ui.label("Add subscribes live; topics already shown are marked “added”.");
            ui.separator();
            if ui
                .button("Add all supported")
                .on_hover_text("Subscribe to every supported topic not already shown")
                .clicked()
            {
                root.collect_leaves(&active, add_requests);
            }
            ui.separator();

            egui::ScrollArea::vertical()
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    draw_ns_node(ui, &root, "", &active, add_requests);
                    if !unsupported.is_empty() {
                        ui.add_space(4.0);
                        egui::CollapsingHeader::new(format!(
                            "Unsupported  ({})",
                            unsupported.len()
                        ))
                        .id_salt("add_unsupported")
                        .default_open(false)
                        .show(ui, |ui| {
                            unsupported.sort_by(|a, b| a.0.cmp(b.0));
                            for (topic, ty) in &unsupported {
                                ui.label(
                                    RichText::new(format!("{topic}  ·  {ty}"))
                                        .weak(),
                                );
                            }
                        });
                    }
                });

            if let Some(msg) = &state.status {
                ui.separator();
                ui.label(msg);
            }
        });
    state.open = open;
    if !add_requests.is_empty() {
        state.status = Some(format!("added {} topic(s)", add_requests.len()));
    }
}

/// Recursively render one namespace level: child namespaces (each collapsible,
/// default-closed, with an "add all" affordance) then the leaf topics.
#[cfg(feature = "ros")]
fn draw_ns_node(
    ui: &mut egui::Ui,
    node: &NsNode,
    prefix: &str,
    active: &HashSet<&str>,
    add_requests: &mut Vec<(String, ros_node::TopicKind)>,
) {
    for (seg, child) in &node.children {
        let ns_path = format!("{prefix}/{seg}");
        ui.horizontal(|ui| {
            if ui
                .small_button("+ all")
                .on_hover_text(format!("Add every supported topic under {ns_path}/"))
                .clicked()
            {
                child.collect_leaves(active, add_requests);
            }
            egui::CollapsingHeader::new(format!("{seg}/  ({})", child.leaf_count()))
                .id_salt(("add_ns", ns_path.as_str()))
                .default_open(false)
                .show(ui, |ui| {
                    draw_ns_node(ui, child, &ns_path, active, add_requests);
                });
        });
    }
    for (topic, kind) in &node.leaves {
        let leaf = topic.rsplit('/').next().unwrap_or(topic.as_str());
        ui.horizontal(|ui| {
            if active.contains(topic.as_str()) {
                ui.add_enabled(false, egui::Button::new("added").small());
            } else if ui.small_button("Add").clicked() {
                add_requests.push((topic.clone(), *kind));
            }
            ui.label(format!("{leaf}  ·  {}", kind.label()));
        });
    }
}

/// "Save config" window — prompt for a name, then write all displayed topics
/// plus the current view transform to `<name>.toml` in the working directory.
#[cfg(feature = "ros")]
fn draw_save_config(
    ctx: &egui::Context,
    state: &mut SaveConfigState,
    tctx: TopicDiscovererCtx<'_>,
    scene: &SceneGraph,
    groups: &[UiGroupView],
    tf_axis_length: f32,
    camera: &OrbitCamera,
) {
    let mut open = state.open;
    egui::Window::new("Save config")
        .resizable(false)
        .default_width(420.0)
        .open(&mut open)
        .show(ctx, |ui| {
            ui.label(
                "Saves every displayed topic and the current camera view to a \
                 config file in the working directory.",
            );
            ui.separator();
            if state.name.is_empty() {
                state.name = "fastviz".to_string();
            }
            let mut save_clicked = false;
            ui.horizontal(|ui| {
                ui.label("Name:");
                let resp = ui.add(
                    egui::TextEdit::singleline(&mut state.name)
                        .desired_width(220.0)
                        .hint_text("fastviz"),
                );
                save_clicked = (resp.lost_focus()
                    && ui.input(|i| i.key_pressed(egui::Key::Enter)))
                    || ui.button("Save").clicked();
                ui.label(".toml");
            });
            if save_clicked {
                let selected: Vec<String> = tctx.active_topics.to_vec();
                let ui_groups_for_save = groups_for_save(groups);
                let snapshot: Vec<(String, Vec<String>)> = tctx.topics.read().clone();
                let cam = ros_node::CameraSave {
                    target: camera.target.to_array(),
                    yaw: camera.yaw,
                    pitch: camera.pitch,
                    distance: camera.distance,
                };
                let toml = ros_node::config_to_toml_full(
                    &snapshot,
                    &selected,
                    tctx.reference_frame,
                    scene,
                    tf_axis_length,
                    &ui_groups_for_save,
                    Some(cam),
                );
                state.status = Some(save_config_file(&state.name, &toml, selected.len()));
            }
            if let Some(msg) = &state.status {
                ui.separator();
                ui.label(msg);
            }
        });
    state.open = open;
}

/// "Load config" window — parse a config file at the given path and rebuild
/// the session from it, mirroring what `--config` does at process start.
#[cfg(feature = "ros")]
fn draw_load_config(
    ctx: &egui::Context,
    state: &mut LoadConfigState,
    load_request: &mut Option<std::path::PathBuf>,
) {
    let mut open = state.open;
    egui::Window::new("Load config")
        .resizable(false)
        .default_width(420.0)
        .open(&mut open)
        .show(ctx, |ui| {
            ui.label(
                "Loads topics, styles, TF axis length and camera view from a \
                 config file, replacing the current session (like restarting \
                 with --config).",
            );
            ui.separator();
            if state.path.is_empty() {
                state.path = "fastviz.toml".to_string();
            }
            let mut load_clicked = false;
            ui.horizontal(|ui| {
                ui.label("Path:");
                let resp = ui.add(
                    egui::TextEdit::singleline(&mut state.path)
                        .desired_width(280.0)
                        .hint_text("fastviz.toml"),
                );
                load_clicked = (resp.lost_focus()
                    && ui.input(|i| i.key_pressed(egui::Key::Enter)))
                    || ui.button("Load").clicked();
            });
            if load_clicked {
                let trimmed = state.path.trim();
                if trimmed.is_empty() {
                    state.status = Some("enter a path".to_string());
                } else {
                    *load_request = Some(std::path::PathBuf::from(trimmed));
                }
            }
            if let Some(msg) = &state.status {
                ui.separator();
                ui.label(msg);
            }
        });
    state.open = open;
}

/// Write `toml` to `<name>.toml` in the current working directory and return a
/// status line quoting the fully-resolved path (or the error).
#[cfg(feature = "ros")]
fn save_config_file(name: &str, toml: &str, topic_count: usize) -> String {
    let trimmed = name.trim();
    let stem = trimmed.strip_suffix(".toml").unwrap_or(trimmed);
    let stem = if stem.is_empty() { "fastviz" } else { stem };
    let filename = format!("{stem}.toml");
    let cwd = std::env::current_dir().unwrap_or_else(|_| std::path::PathBuf::from("."));
    let path = cwd.join(&filename);
    match std::fs::write(&path, toml) {
        Ok(()) => {
            // Resolve symlinks/.. for an unambiguous message; fall back to the
            // joined path if canonicalize fails (it shouldn't, post-write).
            let resolved = std::fs::canonicalize(&path).unwrap_or(path);
            log::info!(
                "saved {topic_count} displayed topic(s) + view to {}",
                resolved.display()
            );
            format!("Saved to {}", resolved.display())
        }
        Err(e) => {
            log::error!("save config failed: {e}");
            format!("Save failed: {e}")
        }
    }
}

/// "Publish" window — pick a topic + type (or type them manually), edit a JSON
/// body, and publish once or at a fixed rate. JSON is validated locally so the
/// status line can report parse errors before anything hits the wire.
#[cfg(feature = "ros")]
fn draw_publish(
    ctx: &egui::Context,
    state: &mut PublishState,
    tctx: TopicDiscovererCtx<'_>,
    publish_requests: &mut Vec<ros_node::PublishRequest>,
) {
    let mut open = state.open;
    egui::Window::new("Publish")
        .resizable(true)
        .default_width(440.0)
        .default_height(420.0)
        .open(&mut open)
        .show(ctx, |ui| {
            let snapshot: Vec<(String, Vec<String>)> = tctx.topics.read().clone();

            ui.horizontal(|ui| {
                ui.label("Topic on graph:");
                egui::ComboBox::from_id_salt("publish_topic_combo")
                    .selected_text(if state.topic.is_empty() {
                        "—".to_string()
                    } else {
                        state.topic.clone()
                    })
                    .width(260.0)
                    .show_ui(ui, |ui| {
                        for (topic, types) in &snapshot {
                            let ty = types.first().cloned().unwrap_or_default();
                            if ui
                                .selectable_label(
                                    state.topic == *topic,
                                    format!("{topic}  ·  {ty}"),
                                )
                                .clicked()
                            {
                                state.topic = topic.clone();
                                state.type_name = ty.clone();
                                // Only seed the body when it's empty so we don't
                                // clobber a hand-edited message on re-select.
                                if state.json.trim().is_empty() {
                                    state.json = template_for(&ty).to_string();
                                }
                            }
                        }
                    });
            });
            ui.horizontal(|ui| {
                ui.label("Topic");
                ui.add(
                    egui::TextEdit::singleline(&mut state.topic)
                        .desired_width(260.0)
                        .hint_text("/cmd_vel"),
                );
            });
            ui.horizontal(|ui| {
                ui.label("Type ");
                ui.add(
                    egui::TextEdit::singleline(&mut state.type_name)
                        .desired_width(260.0)
                        .hint_text("geometry_msgs/msg/Twist"),
                );
                if ui
                    .small_button("template")
                    .on_hover_text("Reset the body to the built-in template for this type")
                    .clicked()
                {
                    state.json = template_for(&state.type_name).to_string();
                }
            });
            ui.separator();
            ui.label("Message (JSON):");
            egui::ScrollArea::vertical()
                .max_height(200.0)
                .show(ui, |ui| {
                    ui.add(
                        egui::TextEdit::multiline(&mut state.json)
                            .code_editor()
                            .desired_rows(8)
                            .desired_width(f32::INFINITY),
                    );
                });
            ui.separator();
            ui.horizontal(|ui| {
                let can_publish = !state.topic.trim().is_empty()
                    && !state.type_name.trim().is_empty();
                if ui
                    .add_enabled(can_publish, egui::Button::new("Publish"))
                    .clicked()
                {
                    match serde_json::from_str::<serde_json::Value>(&state.json) {
                        Ok(_) => {
                            publish_requests.push(ros_node::PublishRequest {
                                topic: state.topic.trim().to_string(),
                                type_name: state.type_name.trim().to_string(),
                                json: state.json.clone(),
                            });
                            state.status = Some(format!("published to {}", state.topic.trim()));
                        }
                        Err(e) => state.status = Some(format!("invalid JSON: {e}")),
                    }
                }
                ui.checkbox(&mut state.repeat, "Publish @");
                ui.add(
                    egui::DragValue::new(&mut state.rate_hz)
                        .speed(0.5)
                        .range(0.1..=1000.0)
                        .suffix(" Hz"),
                );
            });
            if let Some(msg) = &state.status {
                ui.separator();
                ui.label(msg);
            }
        });
    state.open = open;
}

/// Build the lightweight `UiGroupSave` list the config writer wants from the
/// in-memory UI groups. Decoupled so the writer crate doesn't depend on egui.
#[cfg(feature = "ros")]
fn groups_for_save(groups: &[UiGroupView]) -> Vec<ros_node::UiGroupSave> {
    groups
        .iter()
        .map(|g| ros_node::UiGroupSave {
            name: g.name.clone(),
            topics: g.topics.clone(),
            urdf: g.urdf,
            tf: g.tf,
            collapsed: !g.open,
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn draw_group(
    ui: &mut egui::Ui,
    scene: &mut SceneGraph,
    group: &mut UiGroupView,
    members: &[EntityId],
    selection: &mut SelectionState,
    edit_popup: &mut EditPopupState,
    tf_axis_length: Option<&mut f32>,
    remove_requests: &mut Vec<String>,
) {
    ui.horizontal(|ui| {
        eye_toggle(ui, scene, members, ("group_eye", group.name.as_str()));
        let header_label = format!("{}  ({})", group.name, members.len());
        let resp = egui::CollapsingHeader::new(header_label)
            .id_salt(("ui_group", group.name.as_str()))
            .default_open(group.open)
            .show(ui, |ui| {
                if let Some(len) = tf_axis_length {
                    ui.horizontal(|ui| {
                        ui.label("TF size");
                        ui.add(
                            egui::DragValue::new(len)
                                .speed(0.02)
                                .range(0.01..=20.0)
                                .suffix(" m")
                                .max_decimals(3),
                        )
                        .on_hover_text(
                            "Length of every TF-frame axis arm in meters. \
                             Proportions are fixed; only the overall size scales.",
                        );
                    });
                    ui.separator();
                }

                // TF frames don't have a parseable "topic" in their label
                // (just "tf: <frame>"), so render them as a flat list under
                // the group header without an extra collapser per frame.
                if group.tf {
                    for id in members {
                        draw_entity_leaf(ui, scene, *id, selection, edit_popup);
                    }
                    return;
                }

                // Bucket members by topic. Entities without a topic label
                // (URDF links, anything without a label) fall into `topicless`.
                let mut by_topic: std::collections::BTreeMap<String, Vec<EntityId>> =
                    std::collections::BTreeMap::new();
                let mut topicless: Vec<EntityId> = Vec::new();
                for id in members {
                    let label = scene.entities.get(id).and_then(|e| e.label.as_deref());
                    match topic_for_entity(*id, label) {
                        Some(t) => by_topic.entry(t.to_string()).or_default().push(*id),
                        None => topicless.push(*id),
                    }
                }
                for (topic, ids) in &by_topic {
                    draw_topic_bucket(ui, scene, topic, ids, selection, edit_popup, remove_requests);
                }
                for id in &topicless {
                    draw_entity_leaf(ui, scene, *id, selection, edit_popup);
                }
            });
        group.open = resp.openness > 0.5;
    });
}

/// One bucket per topic. Header is collapsed by default. If the bucket
/// contains marker entities, sub-buckets by namespace (also collapsed).
#[allow(clippy::too_many_arguments)]
fn draw_topic_bucket(
    ui: &mut egui::Ui,
    scene: &mut SceneGraph,
    topic: &str,
    ids: &[EntityId],
    selection: &mut SelectionState,
    edit_popup: &mut EditPopupState,
    remove_requests: &mut Vec<String>,
) {
    // A topic with exactly one entity has nothing to disclose under a
    // "topic (1)" header identical to the single leaf inside it — render it
    // as a flat row instead (eye, ✕, label — no expander).
    if ids.len() == 1 {
        ui.horizontal(|ui| {
            eye_toggle(ui, scene, ids, ("topic_eye", topic));
            if cfg!(feature = "ros")
                && ui
                    .small_button("\u{2715}")
                    .on_hover_text("Remove this topic from the view (stops its subscription)")
                    .clicked()
            {
                remove_requests.push(topic.to_string());
            }
            draw_entity_row(ui, scene, ids[0], selection, edit_popup, false);
        });
        return;
    }

    ui.horizontal(|ui| {
        eye_toggle(ui, scene, ids, ("topic_eye", topic));
        // Permanent removal — only meaningful with a live ROS node behind it.
        if cfg!(feature = "ros")
            && ui
                .small_button("\u{2715}")
                .on_hover_text("Remove this topic from the view (stops its subscription)")
                .clicked()
        {
            remove_requests.push(topic.to_string());
        }
        egui::CollapsingHeader::new(format!("{topic}  ({})", ids.len()))
            .id_salt(("topic", topic))
            .default_open(false)
            .show(ui, |ui| {
                let any_marker = ids.iter().any(|id| is_marker_entity(*id));
                if any_marker {
                    let mut by_ns: std::collections::BTreeMap<String, Vec<EntityId>> =
                        std::collections::BTreeMap::new();
                    for id in ids {
                        let label = scene
                            .entities
                            .get(id)
                            .and_then(|e| e.label.as_deref())
                            .unwrap_or("");
                        let ns = parse_marker_label(label)
                            .and_then(|(_, ns, _)| ns)
                            .unwrap_or("");
                        by_ns.entry(ns.to_string()).or_default().push(*id);
                    }
                    // If the only ns is the empty one, skip the ns layer and
                    // render markers directly under the topic so the user
                    // doesn't have to click through a single "(no ns)" header.
                    if by_ns.len() == 1 && by_ns.contains_key("") {
                        for id in ids {
                            draw_entity_leaf(ui, scene, *id, selection, edit_popup);
                        }
                    } else {
                        for (ns, ns_ids) in &by_ns {
                            draw_ns_bucket(ui, scene, topic, ns, ns_ids, selection, edit_popup);
                        }
                    }
                } else {
                    for id in ids {
                        draw_entity_leaf(ui, scene, *id, selection, edit_popup);
                    }
                }
            });
    });
}

fn draw_ns_bucket(
    ui: &mut egui::Ui,
    scene: &mut SceneGraph,
    topic: &str,
    ns: &str,
    ids: &[EntityId],
    selection: &mut SelectionState,
    edit_popup: &mut EditPopupState,
) {
    let header = if ns.is_empty() {
        format!("(no ns)  ({})", ids.len())
    } else {
        format!("{ns}  ({})", ids.len())
    };
    ui.horizontal(|ui| {
        eye_toggle(ui, scene, ids, ("ns_eye", topic, ns));
        egui::CollapsingHeader::new(header)
            .id_salt(("ns", topic, ns))
            .default_open(false)
            .show(ui, |ui| {
                for id in ids {
                    draw_entity_leaf(ui, scene, *id, selection, edit_popup);
                }
            });
    });
}

/// Leaf row for a single entity: eye toggle + a selectable label. Left-click
/// selects the entity; right-click opens a context menu that also selects it
/// and (when the entity is editable) offers "Edit size/shape…", which opens
/// the shared floating style-editor popup.
fn draw_entity_leaf(
    ui: &mut egui::Ui,
    scene: &mut SceneGraph,
    id: EntityId,
    selection: &mut SelectionState,
    edit_popup: &mut EditPopupState,
) {
    ui.horizontal(|ui| {
        draw_entity_row(ui, scene, id, selection, edit_popup, true);
    });
}

/// Row body for a single entity: an optional eye toggle (`with_eye` is false
/// when the caller — e.g. a single-entity topic bucket — already drew its
/// own), then a selectable label with the selection + context-menu behavior
/// shared by every entity row.
fn draw_entity_row(
    ui: &mut egui::Ui,
    scene: &mut SceneGraph,
    id: EntityId,
    selection: &mut SelectionState,
    edit_popup: &mut EditPopupState,
    with_eye: bool,
) {
    let Some(e) = scene.entities.get(&id) else {
        return;
    };
    let display = e
        .label
        .clone()
        .unwrap_or_else(|| format!("entity #{}", id.0));
    let kind = primitive_label(&e.primitive);
    let editable = is_editable(&e.primitive);

    if with_eye {
        eye_toggle(ui, scene, std::slice::from_ref(&id), ("entity_eye", id.0));
    }
    let selected = selection.selected == Some(id);
    let resp = ui.selectable_label(selected, format!("{display}  ({kind})"));
    if resp.clicked() {
        selection.selected = Some(id);
    }
    resp.context_menu(|ui| {
        selection.selected = Some(id);
        if editable && ui.button("Edit size/shape…").clicked() {
            edit_popup.target = Some(id);
            edit_popup.open = true;
            ui.close_menu();
        }
    });
}

/// Floating "Edit — <label>" window driven by either the row context menu or
/// the pinned Edit button. Renders `draw_entity_style_editor` for whichever
/// entity `edit_popup.target` names; closes itself if that entity vanishes.
fn draw_edit_popup(
    ctx: &egui::Context,
    scene: &mut SceneGraph,
    edit_popup: &mut EditPopupState,
    edit_state: &mut EntityEditState,
) {
    let Some(id) = edit_popup.target else { return };
    if !edit_popup.open {
        return;
    }
    let (display, kind, current_color, current_scale, is_frame, prim_kind, current_head) =
        match scene.entities.get(&id) {
            Some(e) => (
                e.label
                    .clone()
                    .unwrap_or_else(|| format!("entity #{}", id.0)),
                primitive_label(&e.primitive),
                scene::primitive_color(&e.primitive),
                scene::primitive_scale(&e.primitive),
                matches!(e.primitive, ScenePrimitive::Frame(_)),
                StyleEditKind::of(&e.primitive),
                scene::primitive_head_radius(&e.primitive),
            ),
            None => {
                edit_popup.open = false;
                edit_popup.target = None;
                return;
            }
        };

    let mut open = edit_popup.open;
    egui::Window::new(format!("Edit — {display}  ({kind})"))
        .id(egui::Id::new("style_edit_popup"))
        .resizable(false)
        .open(&mut open)
        .show(ctx, |ui| {
            draw_entity_style_editor(
                ui,
                scene,
                id,
                current_color,
                current_scale,
                is_frame,
                prim_kind,
                current_head,
                edit_state,
            );
        });
    edit_popup.open = open;
}

/// Which per-entity scale control to present in the style editor. Most
/// primitives just get a generic "scale"; Polylines get a world-space
/// "thickness" and Arrows additionally get an independent "head size".
#[derive(Copy, Clone, PartialEq)]
enum StyleEditKind {
    Generic,
    Polyline,
    Arrows,
}

impl StyleEditKind {
    fn of(p: &ScenePrimitive) -> Self {
        match p {
            ScenePrimitive::Polyline(_) => StyleEditKind::Polyline,
            ScenePrimitive::Arrows(_) => StyleEditKind::Arrows,
            _ => StyleEditKind::Generic,
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn draw_entity_style_editor(
    ui: &mut egui::Ui,
    scene: &mut SceneGraph,
    id: EntityId,
    current_color: Option<scene::Color>,
    current_scale: Option<f32>,
    is_frame: bool,
    prim_kind: StyleEditKind,
    current_head: Option<f32>,
    edit_state: &mut EntityEditState,
) {
    if let Some(c) = current_color {
        let canonical = c.to_hex();
        // Initialize / re-sync the text buffer from the live primitive when the
        // user isn't actively typing into it.
        let entry = edit_state.hex.entry(id).or_insert_with(|| canonical.clone());
        ui.horizontal(|ui| {
            ui.label("color");
            let resp = ui.add(
                egui::TextEdit::singleline(entry)
                    .desired_width(80.0)
                    .hint_text("#RRGGBB"),
            );
            let commit = resp.lost_focus()
                || (resp.changed()
                    && ui.input(|i| i.key_pressed(egui::Key::Enter)));
            if commit {
                if let Some(parsed) = scene::Color::from_hex(entry) {
                    scene.set_color_override(id, Some(parsed));
                    *entry = parsed.to_hex();
                } else {
                    // Reject malformed input: snap back to the live color.
                    *entry = canonical.clone();
                }
            } else if !resp.has_focus() && *entry != canonical {
                // External change (e.g. subscriber pushed a new style) — sync.
                *entry = canonical.clone();
            }
            if ui.small_button("reset").on_hover_text("Drop color override").clicked() {
                scene.set_color_override(id, None);
                *entry = canonical.clone();
            }
        });
    }

    if let Some(s) = current_scale {
        // Frame primitives use the global TF size slider in the group header;
        // a per-entity scale editor here would just fight that update loop.
        if !is_frame {
            let (label, hover) = match prim_kind {
                // Polyline.width renders as a world-space line thickness now
                // (instanced quads), so present it in meters.
                StyleEditKind::Polyline => (
                    "thickness (m)",
                    "World-space line thickness in meters.",
                ),
                StyleEditKind::Arrows => (
                    "length (m)",
                    "Arrow length in meters (shaft + head scale proportionally).",
                ),
                StyleEditKind::Generic => ("scale", "Per-entity size override."),
            };
            let mut val = s;
            ui.horizontal(|ui| {
                ui.label(label);
                let resp = ui.add(
                    egui::DragValue::new(&mut val)
                        .speed(0.01)
                        .range(0.001..=100.0)
                        .max_decimals(3),
                )
                .on_hover_text(hover);
                if resp.changed() {
                    scene.set_scale_override(id, Some(val));
                }
                if ui.small_button("reset").on_hover_text("Drop scale override").clicked() {
                    scene.set_scale_override(id, None);
                }
            });
        }
    }

    // Arrows get an independent head-size control (absolute world radius),
    // separate from the proportional length scale above.
    if prim_kind == StyleEditKind::Arrows {
        if let Some(h) = current_head {
            let mut val = h;
            ui.horizontal(|ui| {
                ui.label("head size (m)");
                let resp = ui.add(
                    egui::DragValue::new(&mut val)
                        .speed(0.005)
                        .range(0.001..=10.0)
                        .max_decimals(3),
                )
                .on_hover_text("Arrow head radius in meters, independent of length.");
                if resp.changed() {
                    scene.set_head_scale_override(id, Some(val));
                }
                if ui
                    .small_button("reset")
                    .on_hover_text("Drop head-size override")
                    .clicked()
                {
                    scene.set_head_scale_override(id, None);
                }
            });
        }
    }
}

fn primitive_label(p: &ScenePrimitive) -> &'static str {
    match p {
        ScenePrimitive::Points(_) => "points",
        ScenePrimitive::Polyline(_) => "polyline",
        ScenePrimitive::Arrows(_) => "arrows",
        ScenePrimitive::Grid(_) => "grid",
        ScenePrimitive::Mesh(_) => "mesh",
        ScenePrimitive::Labels(_) => "labels",
        ScenePrimitive::Frame(_) => "frame",
    }
}
