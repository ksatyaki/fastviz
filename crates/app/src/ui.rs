//! Egui panel layout: top toolbar + left entity list.

use std::collections::{HashMap, HashSet};

use egui::{Align, Layout};
use renderer::{FrameStats, OrbitCamera};
use scene::{EntityId, ScenePrimitive, SceneGraph};

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
    egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
        ui.horizontal(|ui| {
            ui.heading("fastviz");
            ui.separator();
            if ui.button("reset view (F)").clicked() {
                camera.reset_default();
            }
            if ui.button("top (T)").clicked() {
                camera.top_down();
            }
            if ui.button("side (S)").clicked() {
                camera.side();
            }
            ui.separator();
            ui.checkbox(show_reference_grid, "ref grid");
            ui.separator();
            ui.label("follow:");
            let current = follow_frame.clone().unwrap_or_else(|| "(none)".to_string());
            egui::ComboBox::from_id_salt("follow_frame_combo")
                .selected_text(current)
                .show_ui(ui, |ui| {
                    if ui
                        .selectable_label(follow_frame.is_none(), "(none)")
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
                let dt = stats.smoothed_frame_seconds;
                let fps = if dt > 1e-6 { 1.0 / dt } else { 0.0 };
                ui.label(format!("{fps:6.1} fps  ({:.1} ms)", dt * 1000.0));
                if pc2.received > 0 {
                    ui.separator();
                    let dropped = pc2.received.saturating_sub(pc2.displayed);
                    let pct = 100.0 * (dropped as f64) / (pc2.received as f64);
                    ui.label(format!(
                        "PC2 {}/{}  drop {pct:.1}%",
                        pc2.displayed, pc2.received
                    ));
                }
            });
        });
    });

    egui::SidePanel::left("entities")
        .resizable(true)
        .default_width(260.0)
        .show(ctx, |ui| {
            ui.heading("Entities");
            ui.label(format!("reference frame: {}", scene.reference_frame));
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
                    for id in ungrouped {
                        draw_entity_row(ui, scene, id, edit_state);
                    }
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
        // Group-level visibility: checked when every member is visible.
        let all_visible = !members.is_empty() && members.iter().all(|id| scene.entities[id].visible);
        let mut new_all = all_visible;
        if ui
            .add_enabled(!members.is_empty(), egui::Checkbox::new(&mut new_all, ""))
            .changed()
        {
            for id in members {
                scene.set_visible(*id, new_all);
            }
        }
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
                for id in members {
                    draw_entity_row(ui, scene, *id, edit_state);
                }
            });
        group.open = resp.openness > 0.5;
    });
}

fn draw_entity_row(
    ui: &mut egui::Ui,
    scene: &mut SceneGraph,
    id: EntityId,
    edit_state: &mut EntityEditState,
) {
    // Clone the bits we need so we can keep `scene` mutable below.
    let (display, kind, mut visible, current_color, current_scale, is_frame) =
        match scene.entities.get(&id) {
            Some(e) => (
                e.label
                    .clone()
                    .unwrap_or_else(|| format!("entity #{}", id.0)),
                primitive_label(&e.primitive),
                e.visible,
                scene::primitive_color(&e.primitive),
                scene::primitive_scale(&e.primitive),
                matches!(e.primitive, ScenePrimitive::Frame(_)),
            ),
            None => return,
        };

    ui.horizontal(|ui| {
        if ui.checkbox(&mut visible, "").changed() {
            scene.set_visible(id, visible);
        }
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
