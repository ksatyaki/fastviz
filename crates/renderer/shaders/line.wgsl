struct CameraUniform {
    view_proj: mat4x4<f32>,
    camera_pos: vec3<f32>,
    _pad: f32,
};

@group(0) @binding(0) var<uniform> camera: CameraUniform;

struct VsIn {
    @location(0) quad_pos: vec2<f32>,
    @location(1) start: vec3<f32>,
    @location(2) end: vec3<f32>,
    @location(3) color: vec4<f32>,
    @location(4) thickness: f32,
};

struct VsOut {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_main(in: VsIn) -> VsOut {
    var out: VsOut;
    
    let base_pos = mix(in.start, in.end, in.quad_pos.x);
    let line_dir = in.end - in.start;
    
    // Fallback if segment is zero-length
    if length(line_dir) < 1e-6 {
        out.clip_position = camera.view_proj * vec4<f32>(base_pos, 1.0);
        out.color = in.color;
        return out;
    }
    
    let dir = normalize(line_dir);
    let to_camera = normalize(camera.camera_pos - base_pos);
    
    var side = cross(dir, to_camera);
    if length(side) < 1e-4 {
        // Fallback if line is pointing directly at camera
        side = vec3<f32>(1.0, 0.0, 0.0);
    } else {
        side = normalize(side);
    }
    
    // Extrude based on y in [-0.5, 0.5]
    let world_pos = base_pos + side * (in.quad_pos.y * in.thickness);
    
    out.clip_position = camera.view_proj * vec4<f32>(world_pos, 1.0);
    out.color = in.color;
    return out;
}

@fragment
fn fs_main(in: VsOut) -> @location(0) vec4<f32> {
    return in.color;
}
