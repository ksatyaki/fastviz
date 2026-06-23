//! wgpu instance / device / surface bring-up and per-frame surface acquisition.

use std::sync::Arc;

use anyhow::{Context, Result};

/// All long-lived GPU state shared by render passes.
pub struct GpuContext {
    pub instance: wgpu::Instance,
    pub surface: wgpu::Surface<'static>,
    pub adapter: wgpu::Adapter,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub surface_config: wgpu::SurfaceConfiguration,
    pub depth_view: wgpu::TextureView,
    pub depth_format: wgpu::TextureFormat,
    /// MSAA sample count used by every 3D pipeline and the depth buffer. Either
    /// [`MSAA_SAMPLES`] (when the adapter supports it for the surface format) or
    /// `1` (no anti-aliasing). All render pipelines must build with this value.
    pub sample_count: u32,
    /// Multisampled color render target. `Some` when `sample_count > 1`: the 3D
    /// pass renders into this and resolves into the swapchain texture. `None`
    /// when MSAA is unavailable, in which case passes render straight to the
    /// swapchain.
    pub msaa_view: Option<wgpu::TextureView>,
}

pub const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// Preferred MSAA level. 4× is broadly supported and a good quality/cost
/// trade-off; we fall back to no MSAA if the adapter can't do it.
pub const MSAA_SAMPLES: u32 = 4;

impl GpuContext {
    /// Build a `GpuContext` against the given surface target. The target must
    /// be `'static` (typically `Arc<winit::window::Window>`).
    pub async fn new<W>(window: W, width: u32, height: u32) -> Result<Self>
    where
        W: Into<wgpu::SurfaceTarget<'static>>,
    {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::util::backend_bits_from_env().unwrap_or(wgpu::Backends::PRIMARY),
            ..Default::default()
        });

        let surface = instance
            .create_surface(window)
            .context("create wgpu surface")?;

        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .context("no compatible wgpu adapter")?;

        let info = adapter.get_info();
        log::info!(
            "wgpu adapter: {} ({:?}) backend={:?}",
            info.name,
            info.device_type,
            info.backend
        );

        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: Some("fastviz-device"),
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::downlevel_defaults()
                        .using_resolution(adapter.limits()),
                    memory_hints: wgpu::MemoryHints::Performance,
                },
                None,
            )
            .await
            .context("request_device")?;

        let surface_caps = surface.get_capabilities(&adapter);
        // Prefer a linear surface format. egui-wgpu writes its UI in linear
        // space and warns when given an sRGB target. Our 3D passes also write
        // colors in linear space, so a linear framebuffer is the right
        // default.
        let surface_format = surface_caps
            .formats
            .iter()
            .copied()
            .find(|f| !f.is_srgb())
            .unwrap_or(surface_caps.formats[0]);

        let surface_config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: width.max(1),
            height: height.max(1),
            present_mode: surface_caps
                .present_modes
                .iter()
                .copied()
                .find(|m| *m == wgpu::PresentMode::Mailbox)
                .unwrap_or(wgpu::PresentMode::Fifo),
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![],
            desired_maximum_frame_latency: 2,
        };
        surface.configure(&device, &surface_config);

        // Pick an MSAA level the adapter can actually back for this format.
        let sample_count = if surface_format_supports_msaa(&adapter, surface_format, MSAA_SAMPLES) {
            MSAA_SAMPLES
        } else {
            log::info!("MSAA x{MSAA_SAMPLES} unsupported for {surface_format:?}; disabling AA");
            1
        };

        let depth_view =
            create_depth_view(&device, width.max(1), height.max(1), sample_count);
        let msaa_view = create_msaa_view(
            &device,
            width.max(1),
            height.max(1),
            surface_format,
            sample_count,
        );

        Ok(GpuContext {
            instance,
            surface,
            adapter,
            device,
            queue,
            surface_config,
            depth_view,
            depth_format: DEPTH_FORMAT,
            sample_count,
            msaa_view,
        })
    }

    /// Reconfigure the swapchain, depth buffer, and MSAA target after a resize.
    pub fn resize(&mut self, width: u32, height: u32) {
        let w = width.max(1);
        let h = height.max(1);
        self.surface_config.width = w;
        self.surface_config.height = h;
        self.surface.configure(&self.device, &self.surface_config);
        self.depth_view = create_depth_view(&self.device, w, h, self.sample_count);
        self.msaa_view = create_msaa_view(
            &self.device,
            w,
            h,
            self.surface_config.format,
            self.sample_count,
        );
    }

    pub fn aspect(&self) -> f32 {
        self.surface_config.width as f32 / self.surface_config.height.max(1) as f32
    }

    /// Wrap device + queue handles for callers that want to spawn other GPU work.
    pub fn device(&self) -> &wgpu::Device {
        &self.device
    }

    pub fn queue(&self) -> &wgpu::Queue {
        &self.queue
    }
}

fn create_depth_view(
    device: &wgpu::Device,
    width: u32,
    height: u32,
    sample_count: u32,
) -> wgpu::TextureView {
    let depth = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("fastviz-depth"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count,
        dimension: wgpu::TextureDimension::D2,
        format: DEPTH_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    depth.create_view(&wgpu::TextureViewDescriptor::default())
}

/// Multisampled color target the 3D pass resolves into the swapchain. Returns
/// `None` when `sample_count == 1` (rendering goes straight to the swapchain).
fn create_msaa_view(
    device: &wgpu::Device,
    width: u32,
    height: u32,
    format: wgpu::TextureFormat,
    sample_count: u32,
) -> Option<wgpu::TextureView> {
    if sample_count <= 1 {
        return None;
    }
    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("fastviz-msaa-color"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    Some(tex.create_view(&wgpu::TextureViewDescriptor::default()))
}

/// Whether the adapter can multisample-render the given format at `samples`.
fn surface_format_supports_msaa(
    adapter: &wgpu::Adapter,
    format: wgpu::TextureFormat,
    samples: u32,
) -> bool {
    let flags = adapter.get_texture_format_features(format).flags;
    flags.sample_count_supported(samples)
}

/// Type alias used by passes that want shared, cheap-clone handles.
pub type SharedDevice = Arc<wgpu::Device>;
pub type SharedQueue = Arc<wgpu::Queue>;
