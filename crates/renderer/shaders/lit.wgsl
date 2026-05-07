// Shared lit-shading shader for the arrow and mesh passes.
//
// group(0) — camera uniform.
// group(1) — per-instance: model matrix + base color (uniform).

struct CameraUniform {
    view_proj: mat4x4<f32>,
    camera_pos: vec3<f32>,
    _pad: f32,
};
@group(0) @binding(0) var<uniform> camera: CameraUniform;

struct InstanceUniform {
    model: mat4x4<f32>,
    color: vec4<f32>,
};
@group(1) @binding(0) var<uniform> inst: InstanceUniform;

struct VsIn {
    @location(0) position: vec3<f32>,
    @location(1) normal:   vec3<f32>,
    @location(2) uv:       vec2<f32>,
};

struct VsOut {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_pos: vec3<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(2) base_color: vec4<f32>,
};

@vertex
fn vs_main(in: VsIn) -> VsOut {
    var out: VsOut;
    let world = inst.model * vec4<f32>(in.position, 1.0);
    out.clip_position = camera.view_proj * world;
    out.world_pos = world.xyz;
    // Naive: assume model has uniform scale and rotation only.
    out.world_normal = normalize((inst.model * vec4<f32>(in.normal, 0.0)).xyz);
    out.base_color = inst.color;
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
