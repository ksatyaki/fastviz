// Occupancy grid pass: textured quad in the XZ plane. Cell values come from a
// single-channel R8Unorm texture (255 = unknown, 0 = free, 100/255 ≈ 0.39 =
// occupied) and are mapped through the colormap chosen at the uniform.
//
// We don't actually generate the colormap LUT GPU-side here; we rely on the
// CPU mapping done in `OccupancyPass::prepare` to bake the colormap into an
// RGBA texture. This keeps the WGSL trivial.

struct CameraUniform {
    view_proj: mat4x4<f32>,
    camera_pos: vec3<f32>,
    _pad: f32,
};
@group(0) @binding(0) var<uniform> camera: CameraUniform;

struct GridUniform {
    model: mat4x4<f32>,
    tint: vec4<f32>,
};
@group(1) @binding(0) var<uniform> grid: GridUniform;
@group(1) @binding(1) var grid_tex: texture_2d<f32>;
@group(1) @binding(2) var grid_samp: sampler;

struct VsIn {
    @location(0) position: vec3<f32>,
    @location(1) uv:       vec2<f32>,
};

struct VsOut {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex
fn vs_main(in: VsIn) -> VsOut {
    var out: VsOut;
    let world = grid.model * vec4<f32>(in.position, 1.0);
    out.clip_position = camera.view_proj * world;
    out.uv = in.uv;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    let c = textureSample(grid_tex, grid_samp, in.uv);
    return c * grid.tint;
}
