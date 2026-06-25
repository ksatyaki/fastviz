//! fastviz application entry point. Owns the winit event loop, the renderer,
//! the scene graph, the egui shell, and (in M0) the mock data injector.

mod cli;
mod egui_state;
mod theme;
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
    /// Current theme (light/dark). Persisted across runs.
    theme: theme::Mode,
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
    #[cfg_attr(not(feature = "ros"), allow(dead_code))]
    save_config: ui::SaveConfigState,
    #[cfg_attr(not(feature = "ros"), allow(dead_code))]
    publish: ui::PublishState,
    #[cfg_attr(not(feature = "ros"), allow(dead_code))]
    goal: ui::GoalToolState,
    /// Transient goal-tool drag state (armed → pressed → dragging).
    #[cfg(feature = "ros")]
    goal_drag: Option<GoalDrag>,
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

/// Reserved entity for the goal-pose tool's live preview arrow. Lives only
/// during a drag; well above every ROS id range so it never collides.
#[cfg(feature = "ros")]
const GOAL_PREVIEW_ID: scene::EntityId = scene::EntityId(u64::MAX - 7);

/// Transient state while the goal tool is mid-drag: the ground-plane press
/// point and the current cursor's ground hit, both in renderer-world space.
#[cfg(feature = "ros")]
#[derive(Copy, Clone)]
struct GoalDrag {
    start: glam::Vec3,
    current: glam::Vec3,
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

        let icon_bytes = include_bytes!("../../../icons/fastviz_round_f_64x64.png");
        let icon_image = image::load_from_memory(icon_bytes).expect("Failed to load icon").into_rgba8();
        let (width, height) = icon_image.dimensions();
        let icon = winit::window::Icon::from_rgba(icon_image.into_raw(), width, height).unwrap();

        let window_attrs = Window::default_attributes()
            .with_title("fastviz")
            // `with_window_icon` covers X11/XWayland and Windows. On Wayland it
            // is a no-op: the compositor draws no client-supplied icon and
            // instead resolves the icon from a `.desktop` file whose basename
            // matches the surface app_id. We set app_id = "fastviz" below so it
            // matches fastviz.desktop (installed system-wide by the .deb, or
            // into ~/.local/share by icons/install-desktop.sh during dev).
            .with_window_icon(Some(icon))
            .with_inner_size(LogicalSize::new(self.args.width, self.args.height));

        #[cfg(target_os = "linux")]
        let window_attrs = {
            use winit::platform::wayland::WindowAttributesExtWayland;
            window_attrs.with_name("fastviz", "")
        };

        let window = match event_loop.create_window(window_attrs) {
            Ok(w) => Arc::new(w),
            Err(e) => {
                log::error!("create_window failed: {e}");
                event_loop.exit();
                return;
            }
        };

        let size = window.inner_size();
        #[cfg_attr(not(feature = "ros"), allow(unused_mut))]
        let mut renderer = match pollster::block_on(Renderer::new(
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

        // Bundled IBM Plex fonts + initial theme. Fonts are installed once;
        // the theme toggle later only re-pushes a new Style.
        theme::install_fonts(&egui.ctx);
        let theme_mode = theme::load();
        theme::apply(&egui.ctx, theme_mode);

        let scene = Arc::new(RwLock::new(SceneGraph::new(self.args.ref_frame.clone())));
        let mock = if self.args.mock {
            Some(MockInjector::new())
        } else {
            None
        };

        #[cfg(feature = "ros")]
        let (ros, ui_groups, reference_frame, active_topics, cfg_tf_axis_length, cfg_view) = {
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
                &cfg.marker_topics,
                &cfg.marker_array_topics,
            ] {
                for t in v.iter().filter(|t| t.as_str() != "*") {
                    active.push(t.clone());
                }
            }
            let tf_axis_len = cfg.tf_axis_length;
            let view = cfg.view;
            let n = match ros_node::RosNode::spawn(scene.clone(), cfg) {
                Ok(n) => Some(n),
                Err(e) => {
                    log::error!("failed to start ros2 node: {e:#}");
                    None
                }
            };
            (n, groups, reference_frame, active, tf_axis_len, view)
        };
        #[cfg(not(feature = "ros"))]
        let ui_groups: Vec<ui::UiGroupView> = Vec::new();
        #[cfg(not(feature = "ros"))]
        let reference_frame: String = self.args.ref_frame.clone();
        #[cfg(not(feature = "ros"))]
        let active_topics: Vec<String> = Vec::new();

        // Restore a saved camera view (`[view]` in the config), the way RViz
        // reopens a saved framing.
        #[cfg(feature = "ros")]
        if let Some(v) = cfg_view {
            let cam = &mut renderer.camera;
            cam.target = glam::Vec3::from_array(v.target);
            cam.yaw = v.yaw;
            cam.pitch = v.pitch;
            cam.distance = v.distance;
        }

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

        let discoverer = ui::TopicDiscovererState::default();

        self.ctx = Some(AppContext {
            window,
            renderer,
            egui,
            scene,
            mock,
            #[cfg(feature = "ros")]
            ros,
            theme: theme_mode,
            ui_groups,
            tf_axis_length: initial_tf_scale,
            reference_frame,
            discoverer,
            save_config: ui::SaveConfigState::default(),
            publish: ui::PublishState::default(),
            goal: ui::GoalToolState::default(),
            #[cfg(feature = "ros")]
            goal_drag: None,
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
                #[cfg(feature = "ros")]
                if ctx.goal_drag.is_some() {
                    if let Some(hit) = ctx.cursor_ground_hit() {
                        if let Some(d) = ctx.goal_drag.as_mut() {
                            d.current = hit;
                        }
                        ctx.update_goal_preview();
                        ctx.window.request_redraw();
                    }
                    return;
                }
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
                #[cfg(feature = "ros")]
                if button == MouseButton::Left && ctx.goal.armed {
                    if pressed && !egui_consumed {
                        if let Some(hit) = ctx.cursor_ground_hit() {
                            ctx.goal_drag = Some(GoalDrag { start: hit, current: hit });
                            ctx.update_goal_preview();
                        }
                    } else if !pressed && ctx.goal_drag.is_some() {
                        // Only a press that landed on the ground publishes; a
                        // missed (horizon) press leaves the tool armed to retry.
                        ctx.publish_goal();
                    }
                    // The goal click must not also orbit the camera.
                    ctx.input.left_down = false;
                    ctx.window.request_redraw();
                    return;
                }
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
    /// Ground-plane (y = 0) world point under the cursor, or `None` when the
    /// ray misses the ground (e.g. a horizon pick).
    #[cfg(feature = "ros")]
    fn cursor_ground_hit(&self) -> Option<glam::Vec3> {
        let w = self.renderer.gpu.surface_config.width as f32;
        let h = self.renderer.gpu.surface_config.height as f32;
        if w < 1.0 || h < 1.0 {
            return None;
        }
        // winit cursor is y-down in physical pixels; flip to GL-style NDC.
        let ndc = glam::Vec2::new(
            2.0 * self.input.cursor.x / w - 1.0,
            1.0 - 2.0 * self.input.cursor.y / h,
        );
        self.renderer.camera.ground_ray_hit(ndc, w / h)
    }

    /// Upsert the live preview arrow for the current goal drag.
    #[cfg(feature = "ros")]
    fn update_goal_preview(&mut self) {
        let Some(d) = self.goal_drag else {
            return;
        };
        let delta = d.current - d.start;
        let len = delta.length();
        let direction = if len > 1e-4 {
            delta / len
        } else {
            glam::Vec3::X
        };
        let arrow = scene::Arrow {
            origin: d.start,
            direction,
            length: len.max(0.3),
            shaft_radius: 0.05,
            head_radius: 0.12,
            color: scene::Color::rgb(1.0, 0.85, 0.1),
        };
        let entity =
            scene::SceneEntity::new(GOAL_PREVIEW_ID, scene::ScenePrimitive::Arrows(vec![arrow]))
                .with_label("goal preview".to_string());
        self.scene.write().upsert(entity);
    }

    /// Build a `PoseStamped` from the finished drag, publish it to the goal
    /// topic, clear the preview, and disarm the tool.
    #[cfg(feature = "ros")]
    fn publish_goal(&mut self) {
        if let Some(d) = self.goal_drag.take() {
            // World ground point → ROS position: (x, -z, 0).
            let px = d.start.x as f64;
            let py = -d.start.z as f64;
            // Drag direction in world → ROS (dx, -dz); yaw about ROS +Z.
            let dx = (d.current.x - d.start.x) as f64;
            let dz = (d.current.z - d.start.z) as f64;
            let yaw = if dx.abs() < 1e-4 && dz.abs() < 1e-4 {
                0.0
            } else {
                (-dz).atan2(dx)
            };
            let qz = (yaw * 0.5).sin();
            let qw = (yaw * 0.5).cos();
            let json = serde_json::json!({
                "header": { "frame_id": self.reference_frame },
                "pose": {
                    "position": { "x": px, "y": py, "z": 0.0 },
                    "orientation": { "x": 0.0, "y": 0.0, "z": qz, "w": qw }
                }
            })
            .to_string();
            if let Some(n) = self.ros.as_ref() {
                n.publish(ros_node::PublishRequest {
                    topic: self.goal.topic.trim().to_string(),
                    type_name: "geometry_msgs/msg/PoseStamped".to_string(),
                    json,
                });
                log::info!(
                    "goal published to {} ({}, {}) yaw={:.3}",
                    self.goal.topic.trim(),
                    px,
                    py,
                    yaw
                );
            }
            self.scene.write().remove(GOAL_PREVIEW_ID);
        }
        self.goal.armed = false;
    }

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
        // Live "Add topic" requests the UI collects this frame.
        #[cfg(feature = "ros")]
        let mut add_requests: Vec<(String, ros_node::TopicKind)> = Vec::new();
        // Topics the user clicked ✕ on, and one-shot publishes, this frame.
        let mut remove_requests: Vec<String> = Vec::new();
        #[cfg(feature = "ros")]
        let mut publish_requests: Vec<ros_node::PublishRequest> = Vec::new();
        let full_output = {
            let window = self.window.clone();
            let scene = self.scene.clone();
            let camera = &mut self.renderer.camera;
            let show_grid = &mut self.show_reference_grid;
            let groups = &mut self.ui_groups;
            let tf_len = &mut self.tf_axis_length;
            let discoverer = &mut self.discoverer;
            let save_config = &mut self.save_config;
            let edit_state = &mut self.edit_state;
            let follow = &mut self.follow_frame;
            let theme_mode = &mut self.theme;
            let remove_requests = &mut remove_requests;
            #[cfg(feature = "ros")]
            let add_requests = &mut add_requests;
            #[cfg(feature = "ros")]
            let publish_state = &mut self.publish;
            #[cfg(feature = "ros")]
            let goal_state = &mut self.goal;
            #[cfg(feature = "ros")]
            let publish_requests = &mut publish_requests;
            self.egui.run_ui(&window, |egui_ctx| {
                let before = *theme_mode;
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
                    save_config,
                    edit_state,
                    follow,
                    theme_mode,
                    remove_requests,
                    #[cfg(feature = "ros")]
                    topic_ctx,
                    #[cfg(feature = "ros")]
                    add_requests,
                    #[cfg(feature = "ros")]
                    publish_state,
                    #[cfg(feature = "ros")]
                    goal_state,
                    #[cfg(feature = "ros")]
                    publish_requests,
                );
                if *theme_mode != before {
                    theme::apply(egui_ctx, *theme_mode);
                    theme::save(*theme_mode);
                }
            })
        };

        // Forward any live-add requests to the ROS node and record them as
        // active so they appear "added" and get included in a saved config.
        #[cfg(feature = "ros")]
        if !add_requests.is_empty() {
            if let Some(n) = self.ros.as_ref() {
                for (topic, kind) in &add_requests {
                    n.request_add_topic(topic.clone(), *kind);
                    if !self.active_topics.iter().any(|t| t == topic) {
                        self.active_topics.push(topic.clone());
                    }
                }
            }
        }

        // Forward topic removals: stop the subscriber + purge entities on the
        // node side, and drop the topic from `active_topics` so it isn't written
        // to a saved config.
        #[cfg(feature = "ros")]
        if !remove_requests.is_empty() {
            if let Some(n) = self.ros.as_ref() {
                for topic in &remove_requests {
                    n.request_remove_topic(topic.clone());
                }
            }
            self.active_topics
                .retain(|t| !remove_requests.iter().any(|r| r == t));
        }
        #[cfg(not(feature = "ros"))]
        let _ = &remove_requests;

        // Forward one-shot publishes, then drive optional rate publishing.
        #[cfg(feature = "ros")]
        if let Some(n) = self.ros.as_ref() {
            for req in &publish_requests {
                n.publish(req.clone());
            }
            if self.publish.repeat
                && !self.publish.topic.trim().is_empty()
                && !self.publish.type_name.trim().is_empty()
            {
                let period = std::time::Duration::from_secs_f32(
                    (1.0 / self.publish.rate_hz.max(0.1)).max(0.0),
                );
                let now = std::time::Instant::now();
                let due = self
                    .publish
                    .last_publish
                    .map(|t| now.duration_since(t) >= period)
                    .unwrap_or(true);
                if due {
                    // Validate JSON each tick; surface parse errors without spamming.
                    match serde_json::from_str::<serde_json::Value>(&self.publish.json) {
                        Ok(_) => {
                            n.publish(ros_node::PublishRequest {
                                topic: self.publish.topic.trim().to_string(),
                                type_name: self.publish.type_name.trim().to_string(),
                                json: self.publish.json.clone(),
                            });
                            self.publish.last_publish = Some(now);
                        }
                        Err(e) => {
                            self.publish.status = Some(format!("invalid JSON: {e}"));
                            self.publish.repeat = false;
                        }
                    }
                }
            }
        }

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
