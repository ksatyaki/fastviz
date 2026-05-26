//! fastviz application entry point. Owns the winit event loop, the renderer,
//! the scene graph, the egui shell, and (in M0) the mock data injector.

mod cli;
mod egui_state;
mod ui;

use std::sync::Arc;

use cli::Args;
use egui_state::EguiState;
use mock_injector::MockInjector;
use parking_lot::RwLock;
use renderer::Renderer;
use scene::SceneGraph;
use winit::application::ApplicationHandler;
use winit::dpi::LogicalSize;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};

fn main() -> anyhow::Result<()> {
    // Force-clamp the noisy graphics modules even when RUST_LOG is set, so
    // wgpu's per-frame Device::maintain INFO lines don't spam the terminal.
    let rust_log = std::env::var("RUST_LOG").unwrap_or_else(|_| "info".into());
    let filter = format!("{rust_log},wgpu_core=warn,wgpu_hal=warn,wgpu=warn,naga=warn");
    env_logger::Builder::new().parse_filters(&filter).init();

    let args = Args::parse();
    let event_loop = EventLoop::new()?;
    event_loop.set_control_flow(ControlFlow::Poll);

    let mut app = App::new(args);
    event_loop.run_app(&mut app)?;
    Ok(())
}

struct App {
    args: Args,
    ctx: Option<AppContext>,
}

struct AppContext {
    window: Arc<Window>,
    renderer: Renderer,
    egui: EguiState,
    scene: Arc<RwLock<SceneGraph>>,
    mock: Option<MockInjector>,
    #[cfg(feature = "ros")]
    ros: Option<ros_node::RosNode>,
    /// Side-panel grouping config. Empty = flat list.
    ui_groups: Vec<ui::UiGroupView>,
    /// Current TF axis length in meters (UI-driven). Mirrored into the ROS
    /// node's shared atomic each frame so the executor thread picks it up.
    tf_axis_length: f32,
    /// Reference frame the topic discoverer writes into newly-generated TOML.
    /// Cloned from the loaded `RosConfig` at startup so the value survives
    /// without holding a borrow on the ROS node.
    #[cfg_attr(not(feature = "ros"), allow(dead_code))]
    reference_frame: String,
    #[cfg_attr(not(feature = "ros"), allow(dead_code))]
    discoverer: ui::TopicDiscovererState,
    /// Per-entity color/scale text-edit buffers. Lives outside egui memory so
    /// it survives across panel rebuilds without keying on internal egui ids.
    edit_state: ui::EntityEditState,
    /// Concrete topic list (one entry per subscribed topic) used to pre-check
    /// rows in the Save-config window. Built once from the loaded RosConfig.
    #[cfg_attr(not(feature = "ros"), allow(dead_code))]
    active_topics: Vec<String>,
    input: InputState,
    show_reference_grid: bool,
    /// Optional TF frame for the camera to follow. When set, the camera target
    /// is snapped each frame to the world-space position of that frame.
    /// Distinct from `reference_frame` (which is the fixed rendering frame);
    /// follow_frame only affects the view, not what's drawn or in which frame.
    follow_frame: Option<String>,
    /// PC2 messages we've witnessed in `RosStats` so far. Compared to the
    /// current value on each draw to count how many distinct frames actually
    /// reached the screen.
    pc2_last_seen_received: u64,
    /// Number of draws where a fresh PC2 frame was on screen (i.e.
    /// `pc2_received` advanced since the previous draw).
    pc2_displayed: u64,
    /// Wall-clock origin for periodic stats logging.
    pc2_stats_started_at: std::time::Instant,
    /// Most recent periodic-log emission (every N seconds).
    pc2_last_log_at: std::time::Instant,
}

#[derive(Default)]
struct InputState {
    cursor: glam::Vec2,
    last_cursor: glam::Vec2,
    left_down: bool,
    right_down: bool,
}

impl App {
    fn new(args: Args) -> Self {
        App { args, ctx: None }
    }
}

impl Drop for App {
    fn drop(&mut self) {
        let Some(ctx) = self.ctx.as_ref() else {
            return;
        };
        #[cfg(feature = "ros")]
        let received = ctx
            .ros
            .as_ref()
            .map(|n| {
                n.stats()
                    .pc2_received
                    .load(std::sync::atomic::Ordering::Relaxed)
            })
            .unwrap_or(ctx.pc2_last_seen_received);
        #[cfg(not(feature = "ros"))]
        let received = ctx.pc2_last_seen_received;

        let displayed = ctx.pc2_displayed;
        if received > 0 {
            let dropped = received.saturating_sub(displayed);
            let pct = 100.0 * (dropped as f64) / (received as f64);
            log::info!(
                "PC2 throughput: received={received}, displayed={displayed}, dropped={dropped} ({pct:.1}%)"
            );
        }
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.ctx.is_some() {
            return;
        }

        let window_attrs = Window::default_attributes()
            .with_title("fastviz")
            .with_inner_size(LogicalSize::new(self.args.width, self.args.height));

        let window = match event_loop.create_window(window_attrs) {
            Ok(w) => Arc::new(w),
            Err(e) => {
                log::error!("create_window failed: {e}");
                event_loop.exit();
                return;
            }
        };

        let size = window.inner_size();
        let renderer = match pollster::block_on(Renderer::new(
            window.clone(),
            size.width,
            size.height,
        )) {
            Ok(r) => r,
            Err(e) => {
                log::error!("renderer init failed: {e}");
                event_loop.exit();
                return;
            }
        };

        let egui = EguiState::new(
            &window,
            &renderer.gpu.device,
            renderer.gpu.surface_config.format,
        );

        let scene = Arc::new(RwLock::new(SceneGraph::new(self.args.ref_frame.clone())));
        let mock = if self.args.mock {
            Some(MockInjector::new())
        } else {
            None
        };

        #[cfg(feature = "ros")]
        let (ros, ui_groups, reference_frame, active_topics, cfg_tf_axis_length) = {
            let mut cfg = match self.args.config.as_deref() {
                Some(p) => match ros_node::RosConfig::from_path(p) {
                    Ok(c) => {
                        log::info!("loaded ros config from {}", p.display());
                        c
                    }
                    Err(e) => {
                        log::error!("failed to load ros config: {e:#}");
                        ros_node::RosConfig::default()
                    }
                },
                None => ros_node::RosConfig::default(),
            };
            // CLI --ref-frame takes precedence over the config file value.
            if self.args.ref_frame != "map" {
                cfg.reference_frame = self.args.ref_frame.clone();
            }
            // CLI --urdf takes precedence over the config file value.
            if let Some(p) = self.args.urdf.as_ref() {
                cfg.urdf_path = Some(p.clone());
            }
            let groups = ui::UiGroupView::from_config(&cfg.ui_groups);
            let reference_frame = cfg.reference_frame.clone();
            // Snapshot every concrete topic the node will subscribe to so the
            // Save-config window can pre-check them. Wildcards ("*") are skipped
            // since they don't name a real topic.
            let mut active: Vec<String> = Vec::new();
            if cfg.map_topic != "*" {
                active.push(cfg.map_topic.clone());
            }
            for v in [
                &cfg.pose_topics,
                &cfg.pose_array_topics,
                &cfg.path_topics,
                &cfg.scan_topics,
                &cfg.point_topics,
            ] {
                for t in v.iter().filter(|t| t.as_str() != "*") {
                    active.push(t.clone());
                }
            }
            let tf_axis_len = cfg.tf_axis_length;
            let n = match ros_node::RosNode::spawn(scene.clone(), cfg) {
                Ok(n) => Some(n),
                Err(e) => {
                    log::error!("failed to start ros2 node: {e:#}");
                    None
                }
            };
            (n, groups, reference_frame, active, tf_axis_len)
        };
        #[cfg(not(feature = "ros"))]
        let ui_groups: Vec<ui::UiGroupView> = Vec::new();
        #[cfg(not(feature = "ros"))]
        let reference_frame: String = self.args.ref_frame.clone();
        #[cfg(not(feature = "ros"))]
        let active_topics: Vec<String> = Vec::new();

        window.request_redraw();

        #[cfg(feature = "ros")]
        let initial_tf_scale = {
            // If the config file pinned [tf].axis_length, push it into the
            // shared atomic so the executor and the UI start in sync.
            if let (Some(v), Some(n)) = (cfg_tf_axis_length, ros.as_ref()) {
                if v > 0.0 {
                    n.tf_axis_length_handle()
                        .store(v.to_bits(), std::sync::atomic::Ordering::Relaxed);
                }
            }
            ros.as_ref()
                .map(|n| {
                    f32::from_bits(
                        n.tf_axis_length_handle()
                            .load(std::sync::atomic::Ordering::Relaxed),
                    )
                })
                .unwrap_or(0.3)
        };
        #[cfg(not(feature = "ros"))]
        let initial_tf_scale = 0.3;

        let mut discoverer = ui::TopicDiscovererState::default();
        discoverer.filename = "configs/discovered.toml".to_string();

        self.ctx = Some(AppContext {
            window,
            renderer,
            egui,
            scene,
            mock,
            #[cfg(feature = "ros")]
            ros,
            ui_groups,
            tf_axis_length: initial_tf_scale,
            reference_frame,
            discoverer,
            edit_state: ui::EntityEditState::default(),
            active_topics,
            input: InputState::default(),
            show_reference_grid: true,
            follow_frame: None,
            pc2_last_seen_received: 0,
            pc2_displayed: 0,
            pc2_stats_started_at: std::time::Instant::now(),
            pc2_last_log_at: std::time::Instant::now(),
        });
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _window_id: WindowId,
        event: WindowEvent,
    ) {
        let Some(ctx) = self.ctx.as_mut() else {
            return;
        };

        let response = ctx.egui.handle_event(&ctx.window, &event);
        let egui_consumed = response.consumed;
        if response.repaint {
            ctx.window.request_redraw();
        }

        match event {
            WindowEvent::CloseRequested => event_loop.exit(),

            WindowEvent::Resized(size) => {
                ctx.renderer.resize(size.width, size.height);
                ctx.window.request_redraw();
            }

            WindowEvent::ScaleFactorChanged { .. } => {
                let s = ctx.window.inner_size();
                ctx.renderer.resize(s.width, s.height);
                ctx.window.request_redraw();
            }

            WindowEvent::CursorMoved { position, .. } => {
                let p = glam::Vec2::new(position.x as f32, position.y as f32);
                ctx.input.last_cursor = ctx.input.cursor;
                ctx.input.cursor = p;
                if !egui_consumed {
                    let dx = p.x - ctx.input.last_cursor.x;
                    let dy = p.y - ctx.input.last_cursor.y;
                    if ctx.input.left_down {
                        ctx.renderer.camera.orbit(dx, dy);
                        ctx.window.request_redraw();
                    } else if ctx.input.right_down {
                        let h = ctx.renderer.gpu.surface_config.height as f32;
                        ctx.renderer.camera.pan(dx, dy, h);
                        ctx.window.request_redraw();
                    }
                }
            }

            WindowEvent::MouseInput { state, button, .. } => {
                let pressed = state == ElementState::Pressed;
                match button {
                    MouseButton::Left => ctx.input.left_down = pressed && !egui_consumed,
                    MouseButton::Right => ctx.input.right_down = pressed && !egui_consumed,
                    _ => {}
                }
            }

            WindowEvent::MouseWheel { delta, .. } if !egui_consumed => {
                let d = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y,
                    MouseScrollDelta::PixelDelta(p) => p.y as f32 / 50.0,
                };
                ctx.renderer.camera.zoom(d);
                ctx.window.request_redraw();
            }

            WindowEvent::KeyboardInput { event: kev, .. } => {
                if egui_consumed {
                    return;
                }
                if kev.state == ElementState::Pressed {
                    match kev.physical_key {
                        PhysicalKey::Code(KeyCode::Escape) => event_loop.exit(),
                        PhysicalKey::Code(KeyCode::KeyF) => {
                            ctx.renderer.camera.reset_default();
                            ctx.window.request_redraw();
                        }
                        PhysicalKey::Code(KeyCode::KeyT) => {
                            ctx.renderer.camera.top_down();
                            ctx.window.request_redraw();
                        }
                        PhysicalKey::Code(KeyCode::KeyS) => {
                            ctx.renderer.camera.side();
                            ctx.window.request_redraw();
                        }
                        _ => {}
                    }
                }
            }

            WindowEvent::RedrawRequested => {
                ctx.draw();
                // Continuous redraw — mock injector animates the scene.
                ctx.window.request_redraw();
            }

            _ => {}
        }
    }
}

impl AppContext {
    fn draw(&mut self) {
        if let Some(mock) = self.mock.as_mut() {
            let mut scene = self.scene.write();
            mock.update(&mut scene);
        }

        let stats = self.renderer.stats;

        // Producer/consumer accounting for PC2 frames. If `pc2_received`
        // advanced since the last draw, exactly ONE of those messages is on
        // screen this frame (the others were overwritten before we got here).
        #[cfg(feature = "ros")]
        let pc2_received: u64 = self
            .ros
            .as_ref()
            .map(|n| {
                n.stats()
                    .pc2_received
                    .load(std::sync::atomic::Ordering::Relaxed)
            })
            .unwrap_or(0);
        #[cfg(not(feature = "ros"))]
        let pc2_received: u64 = 0;
        if pc2_received > self.pc2_last_seen_received {
            self.pc2_displayed += 1;
            self.pc2_last_seen_received = pc2_received;
        }
        let pc2_stats = ui::Pc2Stats {
            received: pc2_received,
            displayed: self.pc2_displayed,
        };

        // Periodic throughput log (every 5 s) so we get measurements even when
        // the app is killed without a graceful exit.
        let now = std::time::Instant::now();
        if pc2_received > 0 && now.duration_since(self.pc2_last_log_at).as_secs_f32() >= 5.0 {
            let elapsed = now.duration_since(self.pc2_stats_started_at).as_secs_f32();
            let dropped = pc2_received.saturating_sub(self.pc2_displayed);
            let pct = 100.0 * (dropped as f64) / (pc2_received as f64);
            log::info!(
                "PC2 throughput @ {elapsed:.1}s: received={pc2_received}, displayed={}, dropped={dropped} ({pct:.1}%)",
                self.pc2_displayed
            );
            self.pc2_last_log_at = now;
        }

        // 1. Run the UI logic (button clicks etc. mutate camera + scene now).
        #[cfg(feature = "ros")]
        let topic_ctx = self.ros.as_ref().map(|n| ui::TopicDiscovererCtx {
            topics: n.topics(),
            reference_frame: self.reference_frame.as_str(),
            active_topics: self.active_topics.as_slice(),
        });
        let full_output = {
            let window = self.window.clone();
            let scene = self.scene.clone();
            let camera = &mut self.renderer.camera;
            let show_grid = &mut self.show_reference_grid;
            let groups = &mut self.ui_groups;
            let tf_len = &mut self.tf_axis_length;
            let discoverer = &mut self.discoverer;
            let edit_state = &mut self.edit_state;
            let follow = &mut self.follow_frame;
            self.egui.run_ui(&window, |egui_ctx| {
                let mut scene = scene.write();
                ui::draw(
                    egui_ctx,
                    &mut scene,
                    camera,
                    stats,
                    show_grid,
                    pc2_stats,
                    groups,
                    tf_len,
                    discoverer,
                    edit_state,
                    follow,
                    #[cfg(feature = "ros")]
                    topic_ctx,
                );
            })
        };

        // Apply follow-frame: snap camera target to the TF frame's current
        // world position. The frame's entity transform already takes the frame
        // origin to renderer-world coords.
        if let Some(name) = self.follow_frame.as_deref() {
            let scene = self.scene.read();
            let target_label = format!("tf: {name}");
            if let Some(e) = scene
                .entities
                .values()
                .find(|e| e.label.as_deref() == Some(target_label.as_str()))
            {
                let p = e.transform.transform_point3(glam::Vec3::ZERO);
                self.renderer.camera.target = p;
            }
        }

        // Mirror UI-driven TF scale into the ROS executor's shared atomic.
        #[cfg(feature = "ros")]
        if let Some(n) = self.ros.as_ref() {
            n.tf_axis_length_handle().store(
                self.tf_axis_length.to_bits(),
                std::sync::atomic::Ordering::Relaxed,
            );
        }

        // 2. Apply UI-mutated state to the renderer.
        self.renderer.reference_grid.visible = self.show_reference_grid;

        // 3. Render the 3D scene + egui overlay.
        let scene = self.scene.read();
        let render_result = self.renderer.render(&scene, |overlay| {
            self.egui.render(overlay, full_output.clone());
        });

        if let Err(e) = render_result {
            match e {
                wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated => {
                    let s = self.window.inner_size();
                    self.renderer.resize(s.width, s.height);
                }
                wgpu::SurfaceError::OutOfMemory => log::error!("GPU OOM, exiting frame"),
                wgpu::SurfaceError::Timeout => log::warn!("surface timeout"),
            }
        }
    }
}
