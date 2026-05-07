// Instanced screen-space point. The vertex stream is a single quad; per-instance
// data carries the world-space center, color, and pixel size.

struct CameraUniform {
    view_proj: mat4x4<f32>,
    camera_pos: vec3<f32>,
    _pad: f32,
};
@group(0) @binding(0) var<uniform> camera: CameraUniform;

struct ScreenUniform {
    viewport: vec2<f32>,  // pixels
    _pad: vec2<f32>,
};
@group(1) @binding(0) var<uniform> screen: ScreenUniform;

struct VsIn {
    @location(0) corner: vec2<f32>,            // (-1,-1)..(1,1)
    @location(1) inst_pos: vec3<f32>,
    @location(2) inst_color: vec4<f32>,
    @location(3) inst_size: f32,               // pixels
};

struct VsOut {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) corner: vec2<f32>,
};

@vertex
fn vs_main(in: VsIn) -> VsOut {
    var out: VsOut;
    let center_clip = camera.view_proj * vec4<f32>(in.inst_pos, 1.0);
    let px_to_ndc = 2.0 / screen.viewport;
    let offset = in.corner * in.inst_size * 0.5 * px_to_ndc * center_clip.w;
    out.clip_position = vec4<f32>(
        center_clip.x + offset.x,
        center_clip.y + offset.y,
        center_clip.z,
        center_clip.w,
    );
    out.color = in.inst_color;
    out.corner = in.corner;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    // Round point: discard outside unit disk.
    let r = length(in.corner);
    if (r > 1.0) {
        discard;
    }
    let edge = smoothstep(1.0, 0.85, r);
    return vec4<f32>(in.color.rgb, in.color.a * edge);
}
