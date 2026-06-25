//! Orbit camera + GPU-side uniform buffer.

use bytemuck::{Pod, Zeroable};
use glam::{Mat4, Quat, Vec3};

/// Standard orbit-around-a-target camera.
#[derive(Clone, Debug)]
pub struct OrbitCamera {
    pub target: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
    pub fov_y: f32,
    pub near: f32,
    pub far: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        OrbitCamera {
            target: Vec3::ZERO,
            yaw: 0.6,
            pitch: 0.7,
            distance: 12.0,
            fov_y: 60_f32.to_radians(),
            near: 0.05,
            far: 500.0,
        }
    }
}

impl OrbitCamera {
    /// World-space position derived from yaw/pitch/distance.
    pub fn eye(&self) -> Vec3 {
        let cp = self.pitch.cos();
        let offset = Vec3::new(
            self.distance * cp * self.yaw.sin(),
            self.distance * self.pitch.sin(),
            self.distance * cp * self.yaw.cos(),
        );
        self.target + offset
    }

    pub fn view_matrix(&self) -> Mat4 {
        Mat4::look_at_rh(self.eye(), self.target, Vec3::Y)
    }

    pub fn proj_matrix(&self, aspect: f32) -> Mat4 {
        Mat4::perspective_rh(self.fov_y, aspect.max(1e-3), self.near, self.far)
    }

    pub fn view_proj(&self, aspect: f32) -> Mat4 {
        self.proj_matrix(aspect) * self.view_matrix()
    }

    /// Mouse drag → orbit. Inputs are pixel deltas.
    pub fn orbit(&mut self, dx: f32, dy: f32) {
        let sensitivity = 0.005;
        self.yaw -= dx * sensitivity;
        self.pitch = (self.pitch + dy * sensitivity).clamp(
            -89_f32.to_radians(),
            89_f32.to_radians(),
        );
    }

    /// Pan the target in the plane perpendicular to the view direction.
    /// `dx`/`dy` are pixel deltas; `viewport_height` is the framebuffer height
    /// in pixels. The pan amount is the exact world-distance corresponding to
    /// the cursor motion at the target plane, so geometry under the cursor
    /// moves with the mouse 1:1 regardless of zoom level.
    pub fn pan(&mut self, dx: f32, dy: f32, viewport_height: f32) {
        let view = self.view_matrix();
        let right = Vec3::new(view.x_axis.x, view.y_axis.x, view.z_axis.x);
        let up = Vec3::new(view.x_axis.y, view.y_axis.y, view.z_axis.y);
        // World units per pixel at the target plane (perspective).
        let world_per_pixel =
            2.0 * self.distance * (self.fov_y * 0.5).tan() / viewport_height.max(1.0);
        self.target += right * (-dx * world_per_pixel) + up * (dy * world_per_pixel);
    }

    /// Scroll → zoom. Positive `delta` zooms in.
    pub fn zoom(&mut self, delta: f32) {
        let factor = (1.0 - delta * 0.1).clamp(0.1, 10.0);
        self.distance = (self.distance * factor).clamp(0.01, 5000.0);
    }

    pub fn reset_default(&mut self) {
        *self = OrbitCamera::default();
    }

    pub fn top_down(&mut self) {
        self.yaw = 0.0;
        self.pitch = 89_f32.to_radians();
    }

    pub fn side(&mut self) {
        self.yaw = 90_f32.to_radians();
        self.pitch = 0.0;
    }

    /// Unproject a normalized-device-coordinate point and intersect the camera
    /// ray with the ground plane `y = 0`. `ndc` uses the GL convention: x and y
    /// in `[-1, 1]` with **+y up** (callers must flip winit's y-down cursor).
    /// Returns `None` when the ray is parallel to the ground or points away
    /// from it (e.g. a horizon/sky pick).
    pub fn ground_ray_hit(&self, ndc: glam::Vec2, aspect: f32) -> Option<Vec3> {
        let inv = self.view_proj(aspect).inverse();
        let unproject = |z: f32| -> Vec3 {
            let p = inv * glam::Vec4::new(ndc.x, ndc.y, z, 1.0);
            p.truncate() / p.w
        };
        // glam's perspective_rh maps near→0, far→1 (wgpu depth convention).
        let near = unproject(0.0);
        let far = unproject(1.0);
        let dir = (far - near).normalize_or_zero();
        if dir.y.abs() < 1e-6 {
            return None; // parallel to the ground plane
        }
        let t = -near.y / dir.y;
        if t < 0.0 {
            return None; // intersection is behind the camera
        }
        Some(near + dir * t)
    }

    /// Transform-only orientation; useful for a few passes that want to face
    /// the camera (e.g. screen-aligned arrows).
    pub fn rotation(&self) -> Quat {
        Quat::from_euler(glam::EulerRot::YXZ, self.yaw, self.pitch, 0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec2;

    #[test]
    fn center_ray_hits_target_on_ground() {
        // Default camera targets the origin, which lies on y = 0; the screen
        // center ray must hit there.
        let cam = OrbitCamera::default();
        let hit = cam.ground_ray_hit(Vec2::ZERO, 16.0 / 9.0).expect("center hit");
        assert!(hit.distance(Vec3::ZERO) < 1e-3, "hit = {hit:?}");
    }

    #[test]
    fn top_down_center_hits_below_camera() {
        let mut cam = OrbitCamera::default();
        cam.target = Vec3::new(2.0, 0.0, -3.0);
        cam.top_down();
        let hit = cam.ground_ray_hit(Vec2::ZERO, 1.0).expect("center hit");
        assert!(hit.distance(cam.target) < 1e-2, "hit = {hit:?}");
    }

    #[test]
    fn horizon_ray_misses() {
        // Side-on camera (pitch ≈ 0): the top of the screen looks toward the
        // sky and should not intersect the ground.
        let mut cam = OrbitCamera::default();
        cam.side();
        assert!(cam.ground_ray_hit(Vec2::new(0.0, 1.0), 1.0).is_none());
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct CameraUniform {
    pub view_proj: [[f32; 4]; 4],
    pub camera_pos: [f32; 3],
    pub _pad: f32,
}

impl CameraUniform {
    pub fn from_camera(cam: &OrbitCamera, aspect: f32) -> Self {
        CameraUniform {
            view_proj: cam.view_proj(aspect).to_cols_array_2d(),
            camera_pos: cam.eye().to_array(),
            _pad: 0.0,
        }
    }
}

/// GPU-resident camera buffer + bind group, shared across passes.
pub struct CameraGpu {
    pub buffer: wgpu::Buffer,
    pub bind_group_layout: wgpu::BindGroupLayout,
    pub bind_group: wgpu::BindGroup,
}

impl CameraGpu {
    pub fn new(device: &wgpu::Device) -> Self {
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("camera-uniform"),
            size: std::mem::size_of::<CameraUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("camera-bgl"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("camera-bg"),
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: buffer.as_entire_binding(),
            }],
        });

        CameraGpu {
            buffer,
            bind_group_layout,
            bind_group,
        }
    }

    pub fn upload(&self, queue: &wgpu::Queue, cam: &OrbitCamera, aspect: f32) {
        let uniform = CameraUniform::from_camera(cam, aspect);
        queue.write_buffer(&self.buffer, 0, bytemuck::bytes_of(&uniform));
    }
}
