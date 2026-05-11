//! Frame-level orchestration: acquires the swapchain texture, runs every
//! render pass in order, and hands the encoder back to the caller (so that
//! egui can render after the 3D passes).

use std::sync::Arc;
use std::time::Instant;

use scene::SceneGraph;

use crate::camera::{CameraGpu, OrbitCamera};
use crate::gpu::GpuContext;
use crate::passes::{
    ArrowPass, LinePass, MeshPass, OccupancyPass, PointPass, ReferenceGridPass,
};

/// Per-frame timing for HUD display.
#[derive(Copy, Clone, Default, Debug)]
pub struct FrameStats {
    /// Raw wall-clock between the previous and current frame.
    pub last_frame_seconds: f32,
    /// Same value smoothed with a ~0.25 s exponential moving average so the HUD
    /// FPS readout doesn't flicker every frame. UI code should prefer this.
    pub smoothed_frame_seconds: f32,
    pub frame_index: u64,
}

/// Time constant for the FPS lowpass — values within `~TAU` seconds dominate.
/// Tuned so the readout settles in well under a second but still rejects the
/// jitter you'd get from any single frame stalling on swapchain acquisition.
const FRAME_TIME_LOWPASS_TAU: f32 = 0.25;

pub struct Renderer {
    pub gpu: GpuContext,
    pub camera: OrbitCamera,
    pub camera_gpu: CameraGpu,

    pub reference_grid: ReferenceGridPass,
    pub line: LinePass,
    pub arrow: ArrowPass,
    pub mesh: MeshPass,
    pub point: PointPass,
    pub occupancy: OccupancyPass,

    last_frame_at: Instant,
    pub stats: FrameStats,
}

impl Renderer {
    pub async fn new<W>(window: W, width: u32, height: u32) -> anyhow::Result<Self>
    where
        W: Into<wgpu::SurfaceTarget<'static>>,
    {
        let gpu = GpuContext::new(window, width, height).await?;
        let camera = OrbitCamera::default();
        let camera_gpu = CameraGpu::new(&gpu.device);

        let reference_grid = ReferenceGridPass::new(&gpu, &camera_gpu.bind_group_layout);
        let line = LinePass::new(&gpu, &camera_gpu.bind_group_layout);
        let arrow = ArrowPass::new(&gpu, &camera_gpu.bind_group_layout);
        let mesh = MeshPass::new(&gpu, &camera_gpu.bind_group_layout);
        let point = PointPass::new(&gpu, &camera_gpu.bind_group_layout);
        let occupancy = OccupancyPass::new(&gpu, &camera_gpu.bind_group_layout);

        Ok(Renderer {
            gpu,
            camera,
            camera_gpu,
            reference_grid,
            line,
            arrow,
            mesh,
            point,
            occupancy,
            last_frame_at: Instant::now(),
            stats: FrameStats::default(),
        })
    }

    pub fn resize(&mut self, width: u32, height: u32) {
        self.gpu.resize(width, height);
    }

    /// Drive a single frame end-to-end. The closure is invoked after the 3D
    /// passes have written their colors but before `present`, so egui (or any
    /// other 2D overlay) can render into the same surface texture.
    pub fn render<F>(&mut self, scene: &SceneGraph, mut overlay: F) -> Result<(), wgpu::SurfaceError>
    where
        F: FnMut(OverlayContext<'_>),
    {
        let now = Instant::now();
        let dt = (now - self.last_frame_at).as_secs_f32();
        self.last_frame_at = now;
        self.stats.last_frame_seconds = dt;
        // First frame: seed the EMA with the raw value so we don't display 0 fps.
        // After that: alpha = dt / (tau + dt) gives a time-domain EMA whose
        // response is independent of the actual frame rate.
        if self.stats.smoothed_frame_seconds <= 0.0 {
            self.stats.smoothed_frame_seconds = dt;
        } else {
            let alpha = dt / (FRAME_TIME_LOWPASS_TAU + dt);
            self.stats.smoothed_frame_seconds =
                self.stats.smoothed_frame_seconds + alpha * (dt - self.stats.smoothed_frame_seconds);
        }
        self.stats.frame_index = self.stats.frame_index.wrapping_add(1);

        // Camera uniform.
        let aspect = self.gpu.aspect();
        self.camera_gpu.upload(&self.gpu.queue, &self.camera, aspect);

        // Pass prepare.
        self.line.prepare(&self.gpu, scene);
        self.arrow.prepare(&self.gpu, scene);
        self.mesh.prepare(&self.gpu, scene);
        self.point.prepare(&self.gpu, scene);
        self.occupancy.prepare(&self.gpu, scene);

        let frame = self.gpu.surface.get_current_texture()?;
        let view = frame
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder =
            self.gpu
                .device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("frame-encoder"),
                });

        {
            let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("3d-pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.07,
                            g: 0.07,
                            b: 0.10,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.gpu.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            self.occupancy.draw(&mut rpass, &self.camera_gpu.bind_group, scene);
            self.mesh.draw(&mut rpass, &self.camera_gpu.bind_group, scene);
            self.reference_grid.draw(&mut rpass, &self.camera_gpu.bind_group);
            self.line.draw(&mut rpass, &self.camera_gpu.bind_group);
            self.arrow.draw(&mut rpass, &self.camera_gpu.bind_group);
            self.point.draw(&mut rpass, &self.camera_gpu.bind_group);
        }

        overlay(OverlayContext {
            encoder: &mut encoder,
            view: &view,
            device: &self.gpu.device,
            queue: &self.gpu.queue,
            surface_config: &self.gpu.surface_config,
        });

        self.gpu.queue.submit(std::iter::once(encoder.finish()));
        frame.present();
        Ok(())
    }
}

/// Hand-off to a 2D overlay (e.g. egui) so it can render into the same surface
/// texture immediately after the 3D passes.
pub struct OverlayContext<'a> {
    pub encoder: &'a mut wgpu::CommandEncoder,
    pub view: &'a wgpu::TextureView,
    pub device: &'a wgpu::Device,
    pub queue: &'a wgpu::Queue,
    pub surface_config: &'a wgpu::SurfaceConfiguration,
}

/// Re-export so consumers don't need to depend on `Arc<wgpu::Device>` directly.
pub type SharedDevice = Arc<wgpu::Device>;
