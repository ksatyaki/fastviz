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
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("info,wgpu_core=warn,wgpu_hal=warn,naga=warn"),
    )
    .init();

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
    #[allow(dead_code)] // kept alive for its Drop impl (signals shutdown + joins thread)
    ros: Option<ros_node::RosNode>,
    input: InputState,
    show_reference_grid: bool,
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
        let ros = if self.args.ros {
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
            match ros_node::RosNode::spawn(scene.clone(), cfg) {
                Ok(n) => Some(n),
                Err(e) => {
                    log::error!("failed to start ros2 node: {e:#}");
                    None
                }
            }
        } else {
            None
        };

        window.request_redraw();

        self.ctx = Some(AppContext {
            window,
            renderer,
            egui,
            scene,
            mock,
            #[cfg(feature = "ros")]
            ros,
            input: InputState::default(),
            show_reference_grid: true,
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
                        ctx.renderer.camera.pan(dx, dy);
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

        // 1. Run the UI logic (button clicks etc. mutate camera + scene now).
        let full_output = {
            let window = self.window.clone();
            let scene = self.scene.clone();
            let camera = &mut self.renderer.camera;
            let show_grid = &mut self.show_reference_grid;
            self.egui.run_ui(&window, |egui_ctx| {
                let mut scene = scene.write();
                ui::draw(egui_ctx, &mut scene, camera, stats, show_grid);
            })
        };

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
