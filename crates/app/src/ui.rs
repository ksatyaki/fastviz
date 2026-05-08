//! Egui panel layout: top toolbar + left entity list.

use egui::{Align, Layout};
use renderer::{FrameStats, OrbitCamera};
use scene::{ScenePrimitive, SceneGraph};

#[derive(Copy, Clone, Default)]
pub struct Pc2Stats {
    pub received: u64,
    pub displayed: u64,
}

pub fn draw(
    ctx: &egui::Context,
    scene: &mut SceneGraph,
    camera: &mut OrbitCamera,
    stats: FrameStats,
    show_reference_grid: &mut bool,
    pc2: Pc2Stats,
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
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                let fps = if stats.last_frame_seconds > 1e-6 {
                    1.0 / stats.last_frame_seconds
                } else {
                    0.0
                };
                ui.label(format!("{fps:6.1} fps  ({:.1} ms)", stats.last_frame_seconds * 1000.0));
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

            // Stable iteration order for the panel.
            let mut ids: Vec<_> = scene.entities.keys().copied().collect();
            ids.sort();

            for id in ids {
                let entity = scene.entities.get(&id).unwrap();
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
