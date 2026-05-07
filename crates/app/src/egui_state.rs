//! egui glue: holds the egui context, the winit-side input bridge, and the
//! wgpu-side renderer.
//!
//! The drawing flow is split in two so the app can keep its mutable borrows
//! tidy:
//!
//! 1. `run_ui` — calls `egui::Context::run`, which is where buttons /
//!    checkboxes mutate the app state. Produces a `FullOutput` for the GPU.
//! 2. `render` — invoked from inside `Renderer::render`'s overlay closure;
//!    uploads textures and records draw calls into the shared encoder.

use egui::{FullOutput, ViewportId};
use renderer::OverlayContext;
use winit::window::Window;

pub struct EguiState {
    pub ctx: egui::Context,
    pub winit: egui_winit::State,
    pub renderer: egui_wgpu::Renderer,
}

impl EguiState {
    pub fn new(window: &Window, device: &wgpu::Device, color_format: wgpu::TextureFormat) -> Self {
        let ctx = egui::Context::default();
        ctx.set_pixels_per_point(window.scale_factor() as f32);

        let winit = egui_winit::State::new(
            ctx.clone(),
            ViewportId::ROOT,
            window,
            Some(window.scale_factor() as f32),
            None,
            None,
        );

        let renderer = egui_wgpu::Renderer::new(device, color_format, None, 1, false);

        EguiState { ctx, winit, renderer }
    }

    pub fn handle_event(
        &mut self,
        window: &Window,
        event: &winit::event::WindowEvent,
    ) -> egui_winit::EventResponse {
        self.winit.on_window_event(window, event)
    }

    /// Run the UI closure and produce a `FullOutput`. The ctx is cloned so
    /// the closure body can mutate state borrowed from elsewhere on `self`.
    pub fn run_ui<F>(&mut self, window: &Window, mut ui: F) -> FullOutput
    where
        F: FnMut(&egui::Context),
    {
        let raw_input = self.winit.take_egui_input(window);
        let ctx = self.ctx.clone();
        let full_output = ctx.run(raw_input, |c| ui(c));
        self.winit
            .handle_platform_output(window, full_output.platform_output.clone());
        full_output
    }

    /// Push the egui output to the GPU and record its render pass on top of
    /// the 3D color attachment.
    pub fn render(&mut self, overlay: OverlayContext<'_>, full_output: FullOutput) {
        let primitives = self
            .ctx
            .tessellate(full_output.shapes, full_output.pixels_per_point);

        for (id, image_delta) in &full_output.textures_delta.set {
            self.renderer
                .update_texture(overlay.device, overlay.queue, *id, image_delta);
        }

        let screen = egui_wgpu::ScreenDescriptor {
            size_in_pixels: [overlay.surface_config.width, overlay.surface_config.height],
            pixels_per_point: full_output.pixels_per_point,
        };

        self.renderer.update_buffers(
            overlay.device,
            overlay.queue,
            overlay.encoder,
            &primitives,
            &screen,
        );

        {
            let rpass =
                overlay
                    .encoder
                    .begin_render_pass(&wgpu::RenderPassDescriptor {
                        label: Some("egui-overlay"),
                        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                            view: overlay.view,
                            resolve_target: None,
                            ops: wgpu::Operations {
                                load: wgpu::LoadOp::Load,
                                store: wgpu::StoreOp::Store,
                            },
                        })],
                        depth_stencil_attachment: None,
                        timestamp_writes: None,
                        occlusion_query_set: None,
                    });
            self.renderer
                .render(&mut rpass.forget_lifetime(), &primitives, &screen);
        }

        for id in &full_output.textures_delta.free {
            self.renderer.free_texture(id);
        }
    }
}
