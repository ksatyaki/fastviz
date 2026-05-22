//! Egui panel layout: top toolbar + left entity list.

use std::collections::HashSet;

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
    /// Convert the parsed TOML groups into UI views.
    #[cfg(feature = "ros")]
    pub fn from_config(groups: &[ros_node::UiGroup]) -> Vec<Self> {
        groups
            .iter()
            .map(|g| UiGroupView {
                name: g.name.clone(),
                topics: g.topics.clone(),
                urdf: g.urdf,
                tf: g.tf,
                open: !g.collapsed,
            })
            .collect()
    }
}

/// Persistent state for the floating "Topics" window. Owned by the app so the
/// user's selection survives across frames.
#[cfg_attr(not(feature = "ros"), allow(dead_code))]
#[derive(Default)]
pub struct TopicDiscovererState {
    pub open: bool,
    pub filename: String,
    pub selected: HashSet<String>,
    /// One-line status message rendered under the save button.
    pub status: Option<String>,
}

/// Inputs the topic discoverer needs from outside.
#[cfg(feature = "ros")]
#[derive(Copy, Clone)]
pub struct TopicDiscovererCtx<'a> {
    pub topics: &'a ros_node::TopicsSnapshot,
    pub reference_frame: &'a str,
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
    #[cfg(feature = "ros")] topic_ctx: Option<TopicDiscovererCtx<'_>>,
) {
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
            ui.label("TF size");
            ui.add(
                egui::DragValue::new(tf_axis_length)
                    .speed(0.02)
                    .range(0.01..=20.0)
                    .suffix(" m")
                    .max_decimals(3),
            )
            .on_hover_text("Length of every TF-frame axis arm in meters. Proportions are fixed; only the overall size scales.");
            #[cfg(feature = "ros")]
            {
                ui.separator();
                if ui.button("Topics…").clicked() {
                    discoverer.open = !discoverer.open;
                }
            }
            #[cfg(not(feature = "ros"))]
            {
                let _ = &discoverer;
            }
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                // Use the smoothed frame time so the readout doesn't flicker.
                // Raw `last_frame_seconds` is still on FrameStats for callers
                // that want the instantaneous value.
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
        .default_width(220.0)
        .show(ctx, |ui| {
            ui.heading("Entities");
            ui.label(format!("reference frame: {}", scene.reference_frame));
            ui.separator();

            // Build a stable, sorted view of entity IDs once per frame.
            let mut all_ids: Vec<EntityId> = scene.entities.keys().copied().collect();
            all_ids.sort();

            // Assign each entity to at most one group (first match wins).
            // `assigned[i] = Some(group_idx)` or None for ungrouped.
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

            for (gi, group) in groups.iter_mut().enumerate() {
                let members: Vec<EntityId> = all_ids
                    .iter()
                    .zip(assigned.iter())
                    .filter_map(|(id, a)| (*a == Some(gi)).then_some(*id))
                    .collect();
                draw_group(ui, scene, group, &members);
            }

            // Ungrouped entities render flat at the bottom — they show up even
            // if the user forgets to claim a topic in a group.
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
                    draw_entity_row(ui, scene, id);
                }
            }
        });

    #[cfg(feature = "ros")]
    if let Some(tctx) = topic_ctx {
        draw_topic_discoverer(ctx, discoverer, tctx);
    }
}

#[cfg(feature = "ros")]
fn draw_topic_discoverer(
    ctx: &egui::Context,
    state: &mut TopicDiscovererState,
    tctx: TopicDiscovererCtx<'_>,
) {
    let mut open = state.open;
    egui::Window::new("Topic discoverer")
        .resizable(true)
        .default_width(420.0)
        .default_height(360.0)
        .open(&mut open)
        .show(ctx, |ui| {
            let snapshot: Vec<(String, Vec<String>)> = tctx.topics.read().clone();
            ui.label(format!(
                "Discovered {} topic(s) on the ROS graph.",
                snapshot.len()
            ));
            ui.label(
                "Tick the ones you want to visualise, then save a config file. \
                 Topic types fastviz doesn't understand are listed but skipped on save.",
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
                    let toml = ros_node::config_to_toml(
                        &snapshot,
                        &selected,
                        tctx.reference_frame,
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

fn draw_group(
    ui: &mut egui::Ui,
    scene: &mut SceneGraph,
    group: &mut UiGroupView,
    members: &[EntityId],
) {
    ui.horizontal(|ui| {
        // Group-level visibility: checked when every member is visible. A
        // click toggles every member at once (RViz folder behaviour).
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
                for id in members {
                    draw_entity_row(ui, scene, *id);
                }
            });
        // Remember the user's fold toggle so it persists across frames.
        group.open = resp.openness > 0.5;
    });
}

fn draw_entity_row(ui: &mut egui::Ui, scene: &mut SceneGraph, id: EntityId) {
    let entity = match scene.entities.get(&id) {
        Some(e) => e,
        None => return,
    };
    let display = entity
        .label
        .clone()
        .unwrap_or_else(|| format!("entity #{}", id.0));
    let kind = primitive_label(&entity.primitive);
    let mut visible = entity.visible;
    ui.horizontal(|ui| {
        if ui.checkbox(&mut visible, "").changed() {
            scene.set_visible(id, visible);
        }
        ui.label(format!("{display}  ({kind})"));
    });
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
