//! Egui panel layout: top toolbar + left entity list.

use std::collections::{HashMap, HashSet};

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

/// Persistent state for the "Topics" / "Save current config" window. Owned by
/// the app so the user's selection survives across frames.
#[cfg_attr(not(feature = "ros"), allow(dead_code))]
#[derive(Default)]
pub struct TopicDiscovererState {
    pub open: bool,
    pub filename: String,
    pub selected: HashSet<String>,
    /// One-line status message rendered under the save button.
    pub status: Option<String>,
    /// True once we've pre-populated `selected` from the live config; gated so
    /// re-opening the window doesn't clobber user-driven edits.
    pub primed: bool,
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
    edit_state: &mut EntityEditState,
    follow_frame: &mut Option<String>,
    theme_mode: &mut theme::Mode,
    #[cfg(feature = "ros")] topic_ctx: Option<TopicDiscovererCtx<'_>>,
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
                ui.add_space(6.0);
                ui.separator();
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
                #[cfg(feature = "ros")]
                {
                    ui.separator();
                    if ui.button("Save config…").clicked() {
                        discoverer.open = !discoverer.open;
                    }
                }
                #[cfg(not(feature = "ros"))]
                {
                    let _ = &discoverer;
                }
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
                        edit_state,
                        if show_tf_scale { Some(tf_axis_length) } else { None },
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
                    draw_group(ui, scene, &mut synth, &ungrouped, edit_state, None);
                }
            });
        });

    #[cfg(feature = "ros")]
    if let Some(tctx) = topic_ctx {
        draw_topic_discoverer(ctx, discoverer, tctx, scene, groups, *tf_axis_length);
    }
}

#[cfg(feature = "ros")]
fn draw_topic_discoverer(
    ctx: &egui::Context,
    state: &mut TopicDiscovererState,
    tctx: TopicDiscovererCtx<'_>,
    scene: &SceneGraph,
    groups: &[UiGroupView],
    tf_axis_length: f32,
) {
    // Pre-populate `selected` with the topics currently being subscribed to,
    // the first time the window is opened. Subsequent re-opens keep the user's
    // edits so they aren't lost on a stray toggle.
    if state.open && !state.primed {
        state.selected.clear();
        for t in tctx.active_topics {
            state.selected.insert(t.clone());
        }
        state.primed = true;
    }
    if !state.open {
        state.primed = false;
    }

    let mut open = state.open;
    egui::Window::new("Save current config")
        .resizable(true)
        .default_width(460.0)
        .default_height(420.0)
        .open(&mut open)
        .show(ctx, |ui| {
            let snapshot: Vec<(String, Vec<String>)> = tctx.topics.read().clone();
            ui.label(format!(
                "Discovered {} topic(s) on the ROS graph. Pre-checked rows are already in the current view.",
                snapshot.len()
            ));
            ui.label(
                "Tick the topics you want in the saved config. Per-entity color and scale tweaks \
                 from the sidebar are baked into the per-kind style on save.",
            );
            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Select all supported").clicked() {
                    for (t, types) in &snapshot {
                        if ros_node::TopicKind::from_types(types).is_some() {
                            state.selected.insert(t.clone());
                        }
                    }
                }
                if ui.button("Reset to current view").clicked() {
                    state.selected.clear();
                    for t in tctx.active_topics {
                        state.selected.insert(t.clone());
                    }
                }
                if ui.button("Clear").clicked() {
                    state.selected.clear();
                }
            });
            ui.separator();
            egui::ScrollArea::vertical()
                .max_height(220.0)
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    egui::Grid::new("topic_grid")
                        .striped(true)
                        .num_columns(3)
                        .spacing([8.0, 2.0])
                        .show(ui, |ui| {
                            for (topic, types) in &snapshot {
                                let kind = ros_node::TopicKind::from_types(types);
                                let supported = kind.is_some();
                                let mut checked = state.selected.contains(topic);
                                let resp = ui.add_enabled(
                                    supported,
                                    egui::Checkbox::new(&mut checked, ""),
                                );
                                if resp.changed() {
                                    if checked {
                                        state.selected.insert(topic.clone());
                                    } else {
                                        state.selected.remove(topic);
                                    }
                                }
                                ui.label(topic);
                                let kind_str = match kind {
                                    Some(k) => k.label().to_string(),
                                    None => {
                                        let head =
                                            types.first().cloned().unwrap_or_default();
                                        format!("{head} (unsupported)")
                                    }
                                };
                                ui.label(kind_str);
                                ui.end_row();
                            }
                        });
                });
            ui.separator();
            ui.horizontal(|ui| {
                ui.label("Save as:");
                if state.filename.is_empty() {
                    state.filename = "configs/discovered.toml".to_string();
                }
                ui.add(
                    egui::TextEdit::singleline(&mut state.filename)
                        .desired_width(260.0)
                        .hint_text("configs/discovered.toml"),
                );
                let save_clicked = ui.button("Save to TOML").clicked();
                if save_clicked {
                    let selected: Vec<String> =
                        state.selected.iter().cloned().collect();
                    let ui_groups_for_save = groups_for_save(groups);
                    let toml = ros_node::config_to_toml_full(
                        &snapshot,
                        &selected,
                        tctx.reference_frame,
                        scene,
                        tf_axis_length,
                        &ui_groups_for_save,
                    );
                    let path = std::path::PathBuf::from(state.filename.trim());
                    if let Some(parent) = path.parent() {
                        if !parent.as_os_str().is_empty() {
                            let _ = std::fs::create_dir_all(parent);
                        }
                    }
                    match std::fs::write(&path, toml) {
                        Ok(()) => {
                            state.status = Some(format!(
                                "saved {} topic(s) to {}",
                                selected.len(),
                                path.display()
                            ));
                            log::info!(
                                "wrote {} topic selection(s) to {}",
                                selected.len(),
                                path.display()
                            );
                        }
                        Err(e) => {
                            state.status =
                                Some(format!("save failed: {e}"));
                            log::error!("topic discoverer save failed: {e}");
                        }
                    }
                }
            });
            if let Some(msg) = &state.status {
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

fn draw_group(
    ui: &mut egui::Ui,
    scene: &mut SceneGraph,
    group: &mut UiGroupView,
    members: &[EntityId],
    edit_state: &mut EntityEditState,
    tf_axis_length: Option<&mut f32>,
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
                        draw_entity_leaf(ui, scene, *id, edit_state);
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
                    draw_topic_bucket(ui, scene, topic, ids, edit_state);
                }
                for id in &topicless {
                    draw_entity_leaf(ui, scene, *id, edit_state);
                }
            });
        group.open = resp.openness > 0.5;
    });
}

/// One bucket per topic. Header is collapsed by default. If the bucket
/// contains marker entities, sub-buckets by namespace (also collapsed).
fn draw_topic_bucket(
    ui: &mut egui::Ui,
    scene: &mut SceneGraph,
    topic: &str,
    ids: &[EntityId],
    edit_state: &mut EntityEditState,
) {
    ui.horizontal(|ui| {
        eye_toggle(ui, scene, ids, ("topic_eye", topic));
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
                            draw_entity_leaf(ui, scene, *id, edit_state);
                        }
                    } else {
                        for (ns, ns_ids) in &by_ns {
                            draw_ns_bucket(ui, scene, topic, ns, ns_ids, edit_state);
                        }
                    }
                } else {
                    for id in ids {
                        draw_entity_leaf(ui, scene, *id, edit_state);
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
    edit_state: &mut EntityEditState,
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
                    draw_entity_leaf(ui, scene, *id, edit_state);
                }
            });
    });
}

/// Leaf row for a single entity. The expander reveals the color/scale editor.
fn draw_entity_leaf(
    ui: &mut egui::Ui,
    scene: &mut SceneGraph,
    id: EntityId,
    edit_state: &mut EntityEditState,
) {
    let (display, kind, current_color, current_scale, is_frame) =
        match scene.entities.get(&id) {
            Some(e) => (
                e.label
                    .clone()
                    .unwrap_or_else(|| format!("entity #{}", id.0)),
                primitive_label(&e.primitive),
                scene::primitive_color(&e.primitive),
                scene::primitive_scale(&e.primitive),
                matches!(e.primitive, ScenePrimitive::Frame(_)),
            ),
            None => return,
        };

    ui.horizontal(|ui| {
        eye_toggle(ui, scene, std::slice::from_ref(&id), ("entity_eye", id.0));
        let header_resp = egui::CollapsingHeader::new(format!("{display}  ({kind})"))
            .id_salt(("entity", id.0))
            .default_open(false)
            .show(ui, |ui| {
                draw_entity_style_editor(
                    ui,
                    scene,
                    id,
                    current_color,
                    current_scale,
                    is_frame,
                    edit_state,
                );
            });
        let _ = header_resp;
    });
}

fn draw_entity_style_editor(
    ui: &mut egui::Ui,
    scene: &mut SceneGraph,
    id: EntityId,
    current_color: Option<scene::Color>,
    current_scale: Option<f32>,
    is_frame: bool,
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
            let mut val = s;
            ui.horizontal(|ui| {
                ui.label("scale");
                let resp = ui.add(
                    egui::DragValue::new(&mut val)
                        .speed(0.01)
                        .range(0.001..=100.0)
                        .max_decimals(3),
                );
                if resp.changed() {
                    scene.set_scale_override(id, Some(val));
                }
                if ui.small_button("reset").on_hover_text("Drop scale override").clicked() {
                    scene.set_scale_override(id, None);
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
