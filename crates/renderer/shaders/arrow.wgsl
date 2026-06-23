// Instanced lit shader for the arrow (Pose) pass.
//
// The unit-arrow mesh is drawn once per instance via hardware instancing. Each
// instance supplies its own model matrix (4 columns) and color through instance
// vertex attributes, so there is a single draw call and a single instance
// buffer for every arrow in the scene — no per-arrow uniform buffers, bind
// groups, or draw calls.
//
// group(0) — camera uniform.

struct CameraUniform {
    view_proj: mat4x4<f32>,
    camera_pos: vec3<f32>,
    _pad: f32,
};
@group(0) @binding(0) var<uniform> camera: CameraUniform;

struct VsIn {
    // Per-vertex (the shared unit-arrow mesh).
    @location(0) position: vec3<f32>,
    @location(1) normal:   vec3<f32>,
    @location(2) uv:       vec2<f32>,
    // Per-instance: model matrix columns + color.
    @location(3) model_0: vec4<f32>,
    @location(4) model_1: vec4<f32>,
    @location(5) model_2: vec4<f32>,
    @location(6) model_3: vec4<f32>,
    @location(7) color:   vec4<f32>,
};

struct VsOut {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_pos: vec3<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(2) base_color: vec4<f32>,
};

@vertex
fn vs_main(in: VsIn) -> VsOut {
    let model = mat4x4<f32>(in.model_0, in.model_1, in.model_2, in.model_3);
    var out: VsOut;
    let world = model * vec4<f32>(in.position, 1.0);
    out.clip_position = camera.view_proj * world;
    out.world_pos = world.xyz;
    // Naive: assume model has uniform scale and rotation only.
    out.world_normal = normalize((model * vec4<f32>(in.normal, 0.0)).xyz);
    out.base_color = in.color;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    let n = normalize(in.world_normal);
    let sun_dir = normalize(vec3<f32>(0.4, 0.9, 0.3));
    let lambert = max(dot(n, sun_dir), 0.0);
    let ambient = 0.25;
    let lit = in.base_color.rgb * (ambient + 0.85 * lambert);
    return vec4<f32>(lit, in.base_color.a);
}
