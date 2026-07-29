use crate::scene;

fn vec3_literal(v: glam::Vec3) -> String {
    format!("vec3<f32>({:.6}, {:.6}, {:.6})", v.x, v.y, v.z)
}

fn constants() -> String {
    let mut out = String::new();
    out.push_str("const PI: f32 = 3.141592653589793;\n");
    out.push_str(&format!("const ENV_SKY: vec3<f32> = {};\n", vec3_literal(scene::ENV_SKY)));
    out.push_str(&format!(
        "const ENV_HORIZON: vec3<f32> = {};\n",
        vec3_literal(scene::ENV_HORIZON)
    ));
    out.push_str(&format!("const ENV_GROUND: vec3<f32> = {};\n", vec3_literal(scene::ENV_GROUND)));
    out.push_str(&format!("const ENV_SWEEP: vec3<f32> = {};\n", vec3_literal(scene::ENV_SWEEP)));
    out.push_str(&format!("const KEY_COLOUR: vec3<f32> = {};\n", vec3_literal(scene::KEY_COLOUR)));
    out.push_str(&format!(
        "const FILL_COLOUR: vec3<f32> = {};\n",
        vec3_literal(scene::FILL_COLOUR)
    ));
    out.push_str(&format!(
        "const BACKDROP_CENTRE: vec3<f32> = {};\n",
        vec3_literal(scene::BACKDROP_CENTRE)
    ));
    out.push_str(&format!(
        "const BACKDROP_EDGE: vec3<f32> = {};\n",
        vec3_literal(scene::BACKDROP_EDGE)
    ));
    out.push_str(&format!(
        "const BACKDROP_FOCUS: vec2<f32> = vec2<f32>({:.4}, {:.4});\n",
        scene::BACKDROP_FOCUS.0,
        scene::BACKDROP_FOCUS.1
    ));
    out.push_str(&format!(
        "const FLOOR_ALBEDO: vec3<f32> = {};\n",
        vec3_literal(scene::FLOOR_ALBEDO)
    ));
    out.push_str(&format!("const FLOOR_ROUGHNESS: f32 = {:.4};\n", scene::FLOOR_ROUGHNESS));
    out.push_str(&format!(
        "const FLOOR_FADE_FRACTION: f32 = {:.4};\n",
        scene::FLOOR_FADE_FRACTION
    ));
    out.push_str(&format!("const MATERIAL_ROUGHNESS: f32 = {:.4};\n", scene::MATERIAL_ROUGHNESS));
    out.push_str(&format!("const MATERIAL_F0: f32 = {:.4};\n", scene::MATERIAL_F0));
    out.push_str(&format!(
        "const REFLECTION_STRENGTH: f32 = {:.4};\n",
        scene::REFLECTION_STRENGTH
    ));
    out.push_str(&format!(
        "const CONTACT_SHADOW_STRENGTH: f32 = {:.4};\n",
        scene::CONTACT_SHADOW_STRENGTH
    ));
    out.push_str(&format!(
        "const OCCLUSION_STRENGTH: f32 = {:.4};\n",
        scene::AMBIENT_OCCLUSION_STRENGTH
    ));
    out.push_str(&format!("const EXPOSURE: f32 = {:.4};\n", scene::EXPOSURE));
    out.push_str(&format!(
        "const SHADOW_NORMAL_OFFSET_TEXELS: f32 = {:.4};\n",
        scene::SHADOW_NORMAL_OFFSET_TEXELS
    ));
    out.push_str(&format!("const SHADOW_SLOPE_OFFSET: f32 = {:.4};\n", scene::SHADOW_SLOPE_OFFSET));
    out.push_str(&format!("const BLOOM_THRESHOLD: f32 = {:.4};\n", scene::BLOOM_THRESHOLD));
    out.push_str(&format!("const BLOOM_KNEE: f32 = {:.4};\n", scene::BLOOM_KNEE));
    out.push_str(&format!("const BLOOM_INTENSITY: f32 = {:.4};\n", scene::BLOOM_INTENSITY));
    out.push_str(&format!("const VIGNETTE_STRENGTH: f32 = {:.4};\n", scene::VIGNETTE_STRENGTH));
    out.push_str(&format!("const VIGNETTE_RADIUS: f32 = {:.4};\n", scene::VIGNETTE_RADIUS));
    out.push_str(&format!("const LINE_HDR_GAIN: f32 = {:.4};\n", scene::LINE_HDR_GAIN));
    out.push_str(&format!("const LAYER_HEIGHT: f32 = {:.4};\n", scene::LAYER_HEIGHT));
    out.push_str(&format!("const LAYER_RELIEF: f32 = {:.4};\n", scene::LAYER_RELIEF));
    out.push_str(&format!("const LAYER_FACING_FADE: f32 = {:.4};\n", scene::LAYER_FACING_FADE));
    out.push_str(&format!("const LAYER_SELF_SHADOW: f32 = {:.4};\n", scene::LAYER_SELF_SHADOW));
    out.push_str(&format!(
        "const LAYER_SPECULAR_SPREAD: f32 = {:.4};\n",
        scene::LAYER_SPECULAR_SPREAD
    ));
    out.push_str(&format!("const GI_BOUNCE_STRENGTH: f32 = {:.4};\n", scene::GI_BOUNCE_STRENGTH));
    out.push_str(&format!("const DOF_MAX_RADIUS: f32 = {:.4};\n", scene::DOF_MAX_RADIUS));
    out
}

const COLOUR_PRELUDE: &str = r#"
fn tonemap(colour: vec3<f32>) -> vec3<f32> {
    let x = colour * EXPOSURE;
    let a = 2.51;
    let b = 0.03;
    let c = 2.43;
    let d = 0.59;
    let e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), vec3<f32>(0.0), vec3<f32>(1.0));
}

fn encode_srgb(c: vec3<f32>) -> vec3<f32> {
    return pow(max(c, vec3<f32>(0.0)), vec3<f32>(1.0 / 2.2));
}

fn decode_srgb(c: vec3<f32>) -> vec3<f32> {
    return pow(max(c, vec3<f32>(0.0)), vec3<f32>(2.2));
}

fn present(linear_colour: vec3<f32>) -> vec3<f32> {
    return encode_srgb(tonemap(linear_colour));
}

fn scene_rgb(linear_colour: vec3<f32>, ldr: f32) -> vec3<f32> {
    return select(linear_colour, present(linear_colour), ldr > 0.5);
}

fn env_irradiance(n: vec3<f32>) -> vec3<f32> {
    let t = n.z * 0.5 + 0.5;
    let low = mix(ENV_GROUND, ENV_HORIZON, smoothstep(0.0, 0.55, t));
    return mix(low, ENV_SKY, smoothstep(0.45, 1.0, t));
}
"#;

const BRDF_PRELUDE: &str = r#"
fn env_radiance(r: vec3<f32>, roughness: f32) -> vec3<f32> {
    let base = env_irradiance(r);
    let horizon = pow(1.0 - abs(r.z), 5.0);
    return base + ENV_SWEEP * horizon * (1.0 - roughness);
}

fn distribution_ggx(nh: f32, a: f32) -> f32 {
    let a2 = a * a;
    let d = nh * nh * (a2 - 1.0) + 1.0;
    return a2 / max(PI * d * d, 1e-7);
}

fn visibility_smith(nv: f32, nl: f32, a: f32) -> f32 {
    let a2 = a * a;
    let gv = nl * sqrt(nv * nv * (1.0 - a2) + a2);
    let gl = nv * sqrt(nl * nl * (1.0 - a2) + a2);
    return 0.5 / max(gv + gl, 1e-5);
}

fn fresnel_schlick(f0: vec3<f32>, u: f32) -> vec3<f32> {
    return f0 + (vec3<f32>(1.0) - f0) * pow(clamp(1.0 - u, 0.0, 1.0), 5.0);
}

fn env_dfg(roughness: f32, nv: f32) -> vec2<f32> {
    let c0 = vec4<f32>(-1.0, -0.0275, -0.572, 0.022);
    let c1 = vec4<f32>(1.0, 0.0425, 1.04, -0.04);
    let r = roughness * c0 + c1;
    let a004 = min(r.x * r.x, exp2(-9.28 * nv)) * r.x + r.y;
    return vec2<f32>(-1.04, 1.04) * a004 + r.zw;
}

fn env_brdf(f0: vec3<f32>, dfg: vec2<f32>) -> vec3<f32> {
    return f0 * dfg.x + dfg.y;
}

fn energy_compensation(f0: vec3<f32>, dfg: vec2<f32>) -> vec3<f32> {
    return vec3<f32>(1.0) + f0 * (1.0 / max(dfg.y, 1e-4) - 1.0);
}

fn specular_occlusion(nv: f32, ao: f32, roughness: f32) -> f32 {
    return clamp(pow(nv + ao, exp2(-16.0 * roughness - 1.0)) - 1.0 + ao, 0.0, 1.0);
}

fn ibl_specular(radiance: vec3<f32>, f0: vec3<f32>, roughness: f32, nv: f32, ao: f32) -> vec3<f32> {
    let dfg = env_dfg(roughness, nv);
    return radiance
         * env_brdf(f0, dfg)
         * energy_compensation(f0, dfg)
         * specular_occlusion(nv, ao, roughness);
}

fn direct_lobe(
    n: vec3<f32>,
    v: vec3<f32>,
    l: vec3<f32>,
    albedo: vec3<f32>,
    f0: vec3<f32>,
    roughness: f32,
) -> vec3<f32> {
    let nl = max(dot(n, l), 0.0);
    if (nl <= 0.0) {
        return vec3<f32>(0.0);
    }
    let h = normalize(l + v);
    let nv = max(dot(n, v), 1e-4);
    let a = max(roughness * roughness, 1e-3);
    let d = distribution_ggx(max(dot(n, h), 0.0), a);
    let vis = visibility_smith(nv, nl, a);
    let f = fresnel_schlick(f0, max(dot(v, h), 0.0));
    let specular = d * vis * f;
    let diffuse = (vec3<f32>(1.0) - f) * albedo / PI;
    return (diffuse + specular) * nl;
}
"#;

const LAYER_PRELUDE: &str = r#"
const LAYER_STEPS: i32 = 12;
const LAYER_SHADOW_STEPS: i32 = 6;

fn layer_footprint(z: f32) -> f32 {
    return fwidth(z / LAYER_HEIGHT);
}

fn layer_attenuation(footprint: f32) -> f32 {
    let x = PI * footprint;
    return select(clamp(sin(x) / x, 0.0, 1.0), 1.0, x < 1e-4);
}

fn layer_facing(n: vec3<f32>) -> f32 {
    let facing = 1.0 - abs(n.z);
    return select(smoothstep(LAYER_FACING_FADE, 1.0, facing), 0.0, facing < LAYER_FACING_FADE);
}

fn layer_depth(z: f32, facing: f32, atten: f32) -> f32 {
    let phase = z * (2.0 * PI) / LAYER_HEIGHT;
    return LAYER_RELIEF * facing * 0.5 * (1.0 - atten * cos(phase));
}

fn layer_slope(z: f32, facing: f32, atten: f32) -> f32 {
    let phase = z * (2.0 * PI) / LAYER_HEIGHT;
    return LAYER_RELIEF * facing * atten * PI / LAYER_HEIGHT * sin(phase);
}

fn layer_march(wpos: vec3<f32>, n: vec3<f32>, v: vec3<f32>, facing: f32, atten: f32) -> vec3<f32> {
    let march = -v / max(dot(n, v), 0.08);
    let dt = LAYER_RELIEF / f32(LAYER_STEPS);
    var prev_t = 0.0;
    var prev_h = layer_depth(wpos.z, facing, atten);
    var t = 0.0;
    for (var i = 0; i < LAYER_STEPS; i = i + 1) {
        t = t + dt;
        let h = layer_depth((wpos + march * t).z, facing, atten);
        if (h <= t) {
            let before = prev_h - prev_t;
            let after = h - t;
            t = mix(prev_t, t, before / max(before - after, 1e-6));
            break;
        }
        prev_t = t;
        prev_h = h;
    }
    return wpos + march * t;
}

fn layer_normal(n: vec3<f32>, slope: f32) -> vec3<f32> {
    let along = vec3<f32>(0.0, 0.0, 1.0) - n * n.z;
    let len = length(along);
    return select(normalize(n + (along / len) * slope * len), n, len < 1e-4);
}

fn layer_self_shadow(hit: vec3<f32>, n: vec3<f32>, l: vec3<f32>, facing: f32, atten: f32) -> f32 {
    let nl = dot(n, l);
    if (nl <= 0.0 || facing <= 0.0) {
        return 1.0;
    }
    let march = l / max(nl, 0.08);
    let here = layer_depth(hit.z, facing, atten);
    var blocked = 0.0;
    for (var i = 1; i <= LAYER_SHADOW_STEPS; i = i + 1) {
        let t = LAYER_RELIEF * f32(i) / f32(LAYER_SHADOW_STEPS);
        let h = layer_depth((hit + march * t).z, facing, atten);
        blocked = max(blocked, (here - t) - h);
    }
    return 1.0 - clamp(blocked / LAYER_RELIEF, 0.0, 1.0) * LAYER_SELF_SHADOW;
}
"#;

const SCENE_BINDINGS: &str = r#"
struct Scene {
    view_proj: mat4x4<f32>,
    light_vp: mat4x4<f32>,
    eye_time: vec4<f32>,
    fill_lines: vec4<f32>,
    key_ldr: vec4<f32>,
    viewport: vec4<f32>,
    shadow: vec4<f32>,
    floor_plane: vec4<f32>,
    toggles: vec4<f32>,
}

@group(0) @binding(0) var<uniform> scene: Scene;
@group(0) @binding(1) var t_shadow: texture_depth_2d;
@group(0) @binding(2) var s_shadow: sampler_comparison;
@group(0) @binding(3) var t_screen: texture_2d<f32>;
@group(0) @binding(4) var s_screen: sampler;

fn screen_uv(frag: vec2<f32>) -> vec2<f32> {
    return (frag - scene.viewport.xy) / max(scene.viewport.zw, vec2<f32>(1.0));
}

fn scene_out(linear_colour: vec3<f32>) -> vec4<f32> {
    return vec4<f32>(scene_rgb(linear_colour, scene.key_ldr.w), 1.0);
}

fn backdrop_linear(uv: vec2<f32>) -> vec3<f32> {
    let from_bottom = vec2<f32>(uv.x, 1.0 - uv.y);
    var d = from_bottom - BACKDROP_FOCUS;
    d.x = d.x * max(scene.viewport.z, 1.0) / max(scene.viewport.w, 1.0);
    let sweep = 1.0 - smoothstep(0.04, 0.92, length(d));
    return mix(BACKDROP_EDGE, BACKDROP_CENTRE, sweep * sweep);
}

fn shadow_factor(wpos: vec3<f32>, n: vec3<f32>) -> f32 {
    if (scene.shadow.x < 0.5) {
        return 1.0;
    }
    let key_dir = scene.key_ldr.xyz;
    let slope = clamp(1.0 - dot(n, key_dir), 0.0, 1.0);
    let normal_offset = scene.shadow.z
                      * (SHADOW_NORMAL_OFFSET_TEXELS + slope * SHADOW_SLOPE_OFFSET);
    let clip = scene.light_vp * vec4<f32>(wpos + n * normal_offset, 1.0);
    let ndc = clip.xyz / clip.w;
    let coord = vec3<f32>(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5, ndc.z);
    if (coord.x < 0.0 || coord.x > 1.0 || coord.y < 0.0 || coord.y > 1.0
     || coord.z < 0.0 || coord.z > 1.0) {
        return 1.0;
    }
    let reference = coord.z - (0.0015 + slope * 0.004);
    let taps = i32(scene.shadow.w);
    var sum = 0.0;
    var count = 0.0;
    for (var y = -2; y <= 2; y = y + 1) {
        for (var x = -2; x <= 2; x = x + 1) {
            if (abs(x) > taps || abs(y) > taps) {
                continue;
            }
            let offset = vec2<f32>(f32(x), f32(y)) * scene.shadow.y;
            sum = sum + textureSampleCompareLevel(
                t_shadow,
                s_shadow,
                coord.xy + offset,
                reference
            );
            count = count + 1.0;
        }
    }
    return mix(1.0, sum / max(count, 1.0), CONTACT_SHADOW_STRENGTH);
}
"#;

const SCENE_ENTRY_POINTS: &str = r#"
struct MeshIn {
    @location(0) pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec3<f32>,
    @location(3) bad: f32,
}

struct MeshOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) wpos: vec3<f32>,
    @location(2) color: vec3<f32>,
    @location(3) bad: f32,
}

@vertex
fn vs_mesh(v: MeshIn) -> MeshOut {
    var out: MeshOut;
    out.clip = scene.view_proj * vec4<f32>(v.pos, 1.0);
    out.normal = v.normal;
    out.wpos = v.pos;
    out.color = v.color;
    out.bad = v.bad;
    return out;
}

@fragment
fn fs_depth_normal(v: MeshOut) -> @location(0) vec4<f32> {
    return vec4<f32>(normalize(v.normal) * 0.5 + 0.5, 1.0);
}

@fragment
fn fs_mesh(v: MeshOut) -> @location(0) vec4<f32> {
    let atten = layer_attenuation(layer_footprint(v.wpos.z));
    let eye = scene.eye_time.xyz;
    let key_dir = scene.key_ldr.xyz;
    var n = normalize(v.normal);
    let view = normalize(eye - v.wpos);
    if (dot(n, view) < 0.0) {
        n = -n;
    }
    var nv = max(dot(n, view), 1e-4);

    var gi = vec4<f32>(0.0, 0.0, 0.0, 1.0);
    if (scene.toggles.x > 0.5) {
        gi = textureSampleLevel(t_screen, s_screen, screen_uv(v.clip.xy), 0.0);
    }
    let ao = mix(1.0, gi.a, OCCLUSION_STRENGTH);

    if (v.bad > 0.5) {
        let rim = pow(1.0 - nv, 2.5);
        let pulse = 0.70 + 0.30 * sin(scene.eye_time.w * 3.2);
        let key = max(dot(n, key_dir), 0.0);
        let flagged = vec3<f32>(0.32, 0.015, 0.02) * (0.35 + 0.9 * key) * ao
                    + vec3<f32>(2.60, 0.30, 0.14) * rim * pulse;
        return scene_out(flagged);
    }

    var wpos = v.wpos;
    var grooves = 1.0;
    let lines = select(0.0, layer_facing(n), scene.fill_lines.w > 0.5);
    if (lines > 0.0) {
        wpos = layer_march(wpos, n, view, lines, atten);
        grooves = layer_self_shadow(wpos, n, key_dir, lines, atten);
        n = layer_normal(n, layer_slope(wpos.z, lines, atten));
        nv = max(dot(n, view), 1e-4);
    }

    let albedo = decode_srgb(v.color);
    let f0 = vec3<f32>(MATERIAL_F0);
    let roughness = min(
        1.0,
        MATERIAL_ROUGHNESS + LAYER_SPECULAR_SPREAD * lines * (1.0 - atten)
    );
    let shadow = shadow_factor(wpos, n) * grooves;

    var lit = direct_lobe(n, view, key_dir, albedo, f0, roughness) * KEY_COLOUR * shadow;
    lit = lit + albedo / PI * FILL_COLOUR * max(dot(n, scene.fill_lines.xyz), 0.0);
    lit = lit + albedo * select(env_irradiance(n), gi.rgb, scene.toggles.x > 0.5);
    lit = lit + ibl_specular(env_radiance(reflect(-view, n), roughness), f0, roughness, nv, ao);

    return scene_out(lit);
}

struct FullscreenOut {
    @builtin(position) clip: vec4<f32>,
}

@vertex
fn vs_backdrop(@builtin(vertex_index) index: u32) -> FullscreenOut {
    var out: FullscreenOut;
    let p = vec2<f32>(f32((index << 1u) & 2u), f32(index & 2u));
    out.clip = vec4<f32>(p * 2.0 - 1.0, 0.0, 1.0);
    return out;
}

@fragment
fn fs_backdrop(v: FullscreenOut) -> @location(0) vec4<f32> {
    return scene_out(backdrop_linear(screen_uv(v.clip.xy)));
}

struct FloorOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) wpos: vec3<f32>,
}

@vertex
fn vs_floor(@builtin(vertex_index) index: u32) -> FloorOut {
    var out: FloorOut;
    let corner = vec2<f32>(f32(index & 1u), f32((index >> 1u) & 1u)) * 2.0 - 1.0;
    let world = scene.floor_plane.xyz + vec3<f32>(corner * scene.floor_plane.w, 0.0);
    out.wpos = world;
    out.clip = scene.view_proj * vec4<f32>(world, 1.0);
    return out;
}

@fragment
fn fs_floor(v: FloorOut) -> @location(0) vec4<f32> {
    let uv = screen_uv(v.clip.xy);
    let backdrop = backdrop_linear(uv);
    let radial = length(v.wpos.xy - scene.floor_plane.xy) / max(scene.floor_plane.w, 1e-3);
    let presence = (1.0 - smoothstep(1.0 - FLOOR_FADE_FRACTION, 1.0, radial)) * scene.toggles.y;
    if (presence <= 0.0) {
        return scene_out(backdrop);
    }

    let n = vec3<f32>(0.0, 0.0, 1.0);
    let view = normalize(scene.eye_time.xyz - v.wpos);
    let nv = max(dot(n, view), 1e-4);
    let f0 = vec3<f32>(MATERIAL_F0);
    let shadow = shadow_factor(v.wpos, n);

    var lit = direct_lobe(n, view, scene.key_ldr.xyz, FLOOR_ALBEDO, f0, FLOOR_ROUGHNESS)
            * KEY_COLOUR * shadow;
    lit = lit + FLOOR_ALBEDO / PI * FILL_COLOUR * max(dot(n, scene.fill_lines.xyz), 0.0);
    lit = lit + FLOOR_ALBEDO * env_irradiance(n);

    var radiance = env_radiance(reflect(-view, n), FLOOR_ROUGHNESS);
    if (scene.toggles.z > 0.5) {
        let mirror = textureSampleLevel(t_screen, s_screen, uv, 0.0);
        let reflected = mirror.rgb / max(mirror.a, 1e-4);
        let coverage = clamp(mirror.a * REFLECTION_STRENGTH, 0.0, 1.0);
        radiance = mix(radiance, reflected, coverage);
    }
    lit = lit + ibl_specular(radiance, f0, FLOOR_ROUGHNESS, nv, 1.0);

    return scene_out(mix(backdrop, lit, presence));
}
"#;

const POST_BINDINGS: &str = r#"
struct Post {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    eye: vec4<f32>,
    target_size: vec2<f32>,
    direction: vec2<f32>,
    origin: vec2<f32>,
    near_far: vec2<f32>,
    params: vec4<f32>,
    flags: vec4<f32>,
}

@group(0) @binding(0) var<uniform> post: Post;
@group(0) @binding(1) var t_source: texture_2d<f32>;
@group(0) @binding(2) var s_linear: sampler;
@group(0) @binding(3) var t_aux: texture_2d<f32>;
@group(0) @binding(4) var t_previous: texture_2d<f32>;
@group(0) @binding(5) var t_depth: texture_depth_2d;
@group(0) @binding(6) var s_depth: sampler;

fn post_texel() -> vec2<f32> {
    return 1.0 / max(post.target_size, vec2<f32>(1.0));
}

fn post_uv(frag: vec2<f32>) -> vec2<f32> {
    return (frag - post.origin) * post_texel();
}

fn depth_at(uv: vec2<f32>) -> f32 {
    return textureSampleLevel(t_depth, s_depth, uv, 0);
}

fn view_depth(uv: vec2<f32>) -> f32 {
    let near = post.near_far.x;
    let far = post.near_far.y;
    return near * far / max(far - depth_at(uv) * (far - near), 1e-6);
}

fn source_at(uv: vec2<f32>) -> vec4<f32> {
    return textureSampleLevel(t_source, s_linear, uv, 0.0);
}

struct FullscreenOut {
    @builtin(position) clip: vec4<f32>,
}

@vertex
fn vs_fullscreen(@builtin(vertex_index) index: u32) -> FullscreenOut {
    var out: FullscreenOut;
    let p = vec2<f32>(f32((index << 1u) & 2u), f32(index & 2u));
    out.clip = vec4<f32>(p * 2.0 - 1.0, 0.0, 1.0);
    return out;
}
"#;

const OCCLUSION_ENTRY_POINT: &str = r#"
const OCCLUSION_GOLDEN_ANGLE: f32 = 2.39996323;

struct Reconstructed {
    world: vec3<f32>,
    depth: f32,
}

fn world_at(uv: vec2<f32>) -> Reconstructed {
    var out: Reconstructed;
    out.depth = depth_at(uv);
    let clip = vec4<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, out.depth, 1.0);
    let world = post.inv_view_proj * clip;
    out.world = world.xyz / world.w;
    return out;
}

fn ndc_to_uv(ndc: vec2<f32>) -> vec2<f32> {
    return vec2<f32>(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
}

fn gradient_noise(p: vec2<f32>) -> f32 {
    return fract(52.9829189 * fract(dot(p, vec2<f32>(0.06711056, 0.00583715))));
}

struct Basis {
    tangent: vec3<f32>,
    bitangent: vec3<f32>,
}

fn tangent_basis(n: vec3<f32>) -> Basis {
    var out: Basis;
    let up = select(vec3<f32>(1.0, 0.0, 0.0), vec3<f32>(0.0, 0.0, 1.0), abs(n.z) < 0.999);
    out.tangent = normalize(cross(up, n));
    out.bitangent = cross(n, out.tangent);
    return out;
}

@fragment
fn fs_occlusion(v: FullscreenOut) -> @location(0) vec4<f32> {
    let uv = v.clip.xy * post_texel();
    let centre = world_at(uv);
    if (centre.depth >= 1.0) {
        return vec4<f32>(1.0);
    }
    let origin = centre.world;
    let eye = post.eye.xyz;
    let radius = post.params.x;
    let samples = i32(post.flags.x);
    let bounce_enabled = post.flags.z;

    let n = normalize(textureSampleLevel(t_aux, s_linear, uv, 0.0).rgb * 2.0 - 1.0);
    let origin_dist = distance(origin, eye);
    var occlusion = 0.0;
    var weight = 0.0;
    var bent = vec3<f32>(0.0);
    var bounce = vec3<f32>(0.0);
    let basis = tangent_basis(n);
    let rotation = gradient_noise(v.clip.xy + vec2<f32>(post.flags.y * 5.588238));
    let lifted = origin + n * radius * 0.02;
    for (var i = 0; i < 32; i = i + 1) {
        if (i >= samples) {
            break;
        }
        let slice = (f32(i) + 0.5) / f32(samples);
        let angle = (f32(i) + rotation) * OCCLUSION_GOLDEN_ANGLE;
        let ring = sqrt(slice);
        let dir = basis.tangent * (cos(angle) * ring)
                + basis.bitangent * (sin(angle) * ring)
                + n * sqrt(max(1.0 - slice, 0.0));
        let span = mix(0.25, 1.0, fract(slice + rotation));
        let sample_pos = lifted + dir * radius * span;
        let clip = post.view_proj * vec4<f32>(sample_pos, 1.0);
        if (clip.w <= 1e-5) {
            continue;
        }
        let sample_uv = ndc_to_uv(clip.xy / clip.w);
        if (sample_uv.x < 0.0 || sample_uv.x > 1.0
         || sample_uv.y < 0.0 || sample_uv.y > 1.0) {
            continue;
        }
        let surface = world_at(sample_uv);
        weight = weight + 1.0;
        if (surface.depth >= 1.0) {
            bent = bent + dir;
            continue;
        }
        let surface_dist = distance(surface.world, eye);
        let sample_dist = distance(sample_pos, eye);
        if (surface_dist < sample_dist - radius * 0.03) {
            let range = smoothstep(
                0.0,
                1.0,
                radius / max(abs(origin_dist - surface_dist), 1e-4)
            );
            occlusion = occlusion + range;
            if (bounce_enabled > 0.5) {
                let toward = surface.world - origin;
                let reach = length(toward);
                let facing = select(0.0, max(dot(n, toward / reach), 0.0), reach > 1e-5);
                bounce = bounce
                       + textureSampleLevel(t_previous, s_linear, sample_uv, 0.0).rgb
                       * range * facing;
            }
        } else {
            bent = bent + dir;
        }
    }
    let ao = 1.0 - occlusion / max(weight, 1.0);
    let bent_normal = select(n, normalize(bent), dot(bent, bent) > 1e-8);
    let indirect = env_irradiance(bent_normal) * ao
                 + bounce * GI_BOUNCE_STRENGTH / max(weight, 1.0);
    return vec4<f32>(indirect, ao);
}
"#;

const POST_ENTRY_POINTS: &str = r#"
@fragment
fn fs_bilateral_blur(v: FullscreenOut) -> @location(0) vec4<f32> {
    let texel = post_texel();
    let uv = v.clip.xy * texel;
    let centre = view_depth(uv);
    let tolerance = post.params.y;
    var sum = source_at(uv);
    var weight = 1.0;
    for (var i = 1; i <= 4; i = i + 1) {
        let falloff = exp(-f32(i * i) * 0.14);
        for (var side = 0; side < 2; side = side + 1) {
            let side_sign = select(-1.0, 1.0, side == 0);
            let at = uv + post.direction * texel * f32(i) * side_sign;
            let w = falloff * exp(-abs(view_depth(at) - centre) / max(tolerance, 1e-4));
            sum = sum + source_at(at) * w;
            weight = weight + w;
        }
    }
    return sum / max(weight, 1e-4);
}

@fragment
fn fs_gaussian_blur(v: FullscreenOut) -> @location(0) vec4<f32> {
    var weights = array<f32, 3>(0.2270270270, 0.3162162162, 0.0702702703);
    var offsets = array<f32, 3>(0.0, 1.3846153846, 3.2307692308);
    let texel = post_texel();
    let uv = v.clip.xy * texel;
    let stride = post.direction * texel * post.params.x;
    var sum = source_at(uv) * weights[0];
    for (var i = 1; i < 3; i = i + 1) {
        let offset = stride * offsets[i];
        sum = sum + source_at(uv + offset) * weights[i];
        sum = sum + source_at(uv - offset) * weights[i];
    }
    return sum;
}

@fragment
fn fs_bloom_bright(v: FullscreenOut) -> @location(0) vec4<f32> {
    let uv = v.clip.xy * post_texel();
    let c = source_at(uv).rgb;
    let brightness = max(c.r, max(c.g, c.b));
    let knee = max(BLOOM_THRESHOLD * BLOOM_KNEE, 1e-4);
    var soft = clamp(brightness - BLOOM_THRESHOLD + knee, 0.0, 2.0 * knee);
    soft = soft * soft / (4.0 * knee);
    let contribution = max(soft, brightness - BLOOM_THRESHOLD) / max(brightness, 1e-4);
    return vec4<f32>(c * contribution, 1.0);
}

const DOF_TAPS: i32 = 16;
const GOLDEN_ANGLE: f32 = 2.39996323;

fn circle_of_confusion(uv: vec2<f32>) -> f32 {
    let focus = post.params.z;
    let aperture = post.params.w;
    return clamp(
        abs(view_depth(uv) - focus) / max(focus, 1e-3) * aperture,
        0.0,
        DOF_MAX_RADIUS
    );
}

fn gather_defocus(uv: vec2<f32>, texel: vec2<f32>, coc: f32) -> vec3<f32> {
    let centre_depth = view_depth(uv);
    var sum = source_at(uv).rgb;
    var weight = 1.0;
    for (var i = 1; i <= DOF_TAPS; i = i + 1) {
        let t = f32(i) / f32(DOF_TAPS);
        let angle = f32(i) * GOLDEN_ANGLE;
        let reach = sqrt(t) * coc;
        let at = uv + vec2<f32>(cos(angle), sin(angle)) * reach * texel;
        let in_front = view_depth(at) < centre_depth;
        let spreads_this_far = circle_of_confusion(at) >= reach;
        let w = select(1.0, select(0.0, 1.0, spreads_this_far), in_front);
        sum = sum + source_at(at).rgb * w;
        weight = weight + w;
    }
    return sum / max(weight, 1e-4);
}

@fragment
fn fs_resolve(v: FullscreenOut) -> @location(0) vec4<f32> {
    let texel = post_texel();
    let uv = v.clip.xy * texel;
    var c = source_at(uv).rgb;
    if (post.params.w > 0.0 && depth_at(uv) < 1.0) {
        let coc = circle_of_confusion(uv);
        if (coc > 1.0) {
            c = gather_defocus(uv, texel, coc);
        }
    }
    if (post.flags.z > 0.5) {
        c = c + textureSampleLevel(t_aux, s_linear, uv, 0.0).rgb * BLOOM_INTENSITY;
    }
    let radial = length((uv - 0.5) * 2.0);
    let vignette = mix(
        1.0 - VIGNETTE_STRENGTH,
        1.0,
        smoothstep(1.0, VIGNETTE_RADIUS, radial)
    );
    c = c * vignette;
    return vec4<f32>(scene_rgb(c, post.flags.w), 1.0);
}

fn blit_out(rgb: vec3<f32>) -> vec4<f32> {
    return vec4<f32>(select(rgb, decode_srgb(rgb), post.flags.x > 0.5), 1.0);
}

@fragment
fn fs_copy(v: FullscreenOut) -> @location(0) vec4<f32> {
    return blit_out(source_at(post_uv(v.clip.xy)).rgb);
}

const EDGE_THRESHOLD: f32 = 0.125;
const EDGE_THRESHOLD_MIN: f32 = 0.0312;
const DIRECTION_REDUCE: f32 = 0.125;
const DIRECTION_REDUCE_MIN: f32 = 0.0078125;
const SPAN_MAX: f32 = 8.0;

fn luma(c: vec3<f32>) -> f32 {
    return dot(c, vec3<f32>(0.299, 0.587, 0.114));
}

@fragment
fn fs_fxaa(v: FullscreenOut) -> @location(0) vec4<f32> {
    let texel = post_texel();
    let uv = post_uv(v.clip.xy);
    let rgb_m = source_at(uv).rgb;
    let luma_m = luma(rgb_m);
    let luma_n = luma(source_at(uv + vec2<f32>(0.0, texel.y)).rgb);
    let luma_s = luma(source_at(uv - vec2<f32>(0.0, texel.y)).rgb);
    let luma_e = luma(source_at(uv + vec2<f32>(texel.x, 0.0)).rgb);
    let luma_w = luma(source_at(uv - vec2<f32>(texel.x, 0.0)).rgb);
    let luma_min = min(luma_m, min(min(luma_n, luma_s), min(luma_e, luma_w)));
    let luma_max = max(luma_m, max(max(luma_n, luma_s), max(luma_e, luma_w)));
    let range = luma_max - luma_min;
    if (range < max(EDGE_THRESHOLD_MIN, luma_max * EDGE_THRESHOLD)) {
        return blit_out(rgb_m);
    }

    let luma_nw = luma(source_at(uv + vec2<f32>(-texel.x, texel.y)).rgb);
    let luma_ne = luma(source_at(uv + vec2<f32>(texel.x, texel.y)).rgb);
    let luma_sw = luma(source_at(uv + vec2<f32>(-texel.x, -texel.y)).rgb);
    let luma_se = luma(source_at(uv + vec2<f32>(texel.x, -texel.y)).rgb);

    var dir = vec2<f32>(
        -((luma_nw + luma_ne) - (luma_sw + luma_se)),
        ((luma_nw + luma_sw) - (luma_ne + luma_se))
    );
    let reduction = max(
        (luma_nw + luma_ne + luma_sw + luma_se) * 0.25 * DIRECTION_REDUCE,
        DIRECTION_REDUCE_MIN
    );
    let rcp = 1.0 / (min(abs(dir.x), abs(dir.y)) + reduction);
    dir = clamp(dir * rcp, vec2<f32>(-SPAN_MAX), vec2<f32>(SPAN_MAX)) * texel;

    let inner = 0.5 * (
        source_at(uv + dir * (1.0 / 3.0 - 0.5)).rgb
      + source_at(uv + dir * (2.0 / 3.0 - 0.5)).rgb
    );
    let outer = inner * 0.5 + 0.25 * (
        source_at(uv - dir * 0.5).rgb
      + source_at(uv + dir * 0.5).rgb
    );
    let luma_outer = luma(outer);
    return blit_out(select(outer, inner, luma_outer < luma_min || luma_outer > luma_max));
}
"#;

const LINE_MODULE: &str = r#"
struct Line {
    view_proj: mat4x4<f32>,
    half_vp: vec2<f32>,
    width: f32,
    alpha: f32,
    settings: vec4<f32>,
}

@group(0) @binding(0) var<uniform> line: Line;

struct LineIn {
    @location(0) p0: vec3<f32>,
    @location(1) p1: vec3<f32>,
    @location(2) color: vec3<f32>,
    @location(3) end: f32,
    @location(4) side: f32,
}

struct LineOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec3<f32>,
    @location(1) across: f32,
    @location(2) half_width: f32,
}

@vertex
fn vs_line(v: LineIn) -> LineOut {
    let c0 = line.view_proj * vec4<f32>(v.p0, 1.0);
    let c1 = line.view_proj * vec4<f32>(v.p1, 1.0);
    let c = select(c1, c0, v.end < 0.5);
    let s0 = c0.xy / max(abs(c0.w), 1e-4) * line.half_vp;
    let s1 = c1.xy / max(abs(c1.w), 1e-4) * line.half_vp;
    let d = s1 - s0;
    let len = length(d);
    let dir = select(vec2<f32>(1.0, 0.0), d / len, len > 1e-6);
    let nrm = vec2<f32>(-dir.y, dir.x);
    let half_w = 0.5 * line.width;
    let ext = half_w + 1.0;

    var out: LineOut;
    out.clip = vec4<f32>(
        c.xy + nrm * v.side * ext / line.half_vp * c.w,
        c.z,
        c.w
    );
    out.color = v.color;
    out.across = v.side * ext;
    out.half_width = half_w;
    return out;
}

@fragment
fn fs_line(v: LineOut) -> @location(0) vec4<f32> {
    let d = abs(v.across);
    let aa = fwidth(d);
    let a = 1.0 - smoothstep(v.half_width - aa, v.half_width + aa, d);
    if (a <= 0.0) {
        discard;
    }
    let rgb = select(decode_srgb(v.color) * LINE_HDR_GAIN, v.color, line.settings.x > 0.5);
    return vec4<f32>(rgb, a * line.alpha);
}
"#;

pub fn scene_module() -> String {
    format!(
        "{}{}{}{}{}{}",
        constants(),
        COLOUR_PRELUDE,
        BRDF_PRELUDE,
        LAYER_PRELUDE,
        SCENE_BINDINGS,
        SCENE_ENTRY_POINTS
    )
}

pub fn post_module() -> String {
    format!(
        "{}{}{}{}{}",
        constants(),
        COLOUR_PRELUDE,
        POST_BINDINGS,
        OCCLUSION_ENTRY_POINT,
        POST_ENTRY_POINTS
    )
}

pub fn line_module() -> String {
    format!("{}{}{}", constants(), COLOUR_PRELUDE, LINE_MODULE)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn every_module() -> [(&'static str, String); 3] {
        [
            ("scene", scene_module()),
            ("post", post_module()),
            ("line", line_module()),
        ]
    }

    fn validate(source: &str) -> naga::valid::ModuleInfo {
        let module = naga::front::wgsl::parse_str(source)
            .unwrap_or_else(|e| panic!("{}", e.emit_to_string(source)));
        naga::valid::Validator::new(
            naga::valid::ValidationFlags::all(),
            naga::valid::Capabilities::empty(),
        )
        .validate(&module)
        .unwrap_or_else(|e| panic!("{e:?}"))
    }

    #[test]
    fn every_module_parses_and_validates_as_wgsl() {
        for (name, source) in every_module() {
            let module = naga::front::wgsl::parse_str(&source)
                .unwrap_or_else(|e| panic!("{name}: {}", e.emit_to_string(&source)));
            naga::valid::Validator::new(
                naga::valid::ValidationFlags::all(),
                naga::valid::Capabilities::empty(),
            )
            .validate(&module)
            .unwrap_or_else(|e| panic!("{name}: {e:?}"));
        }
    }

    #[test]
    fn the_modules_declare_exactly_the_entry_points_the_pipelines_ask_for() {
        let expected: [(&str, &[&str]); 3] = [
            ("scene", &["vs_mesh", "fs_depth_normal", "fs_mesh", "vs_backdrop", "fs_backdrop", "vs_floor", "fs_floor"]),
            (
                "post",
                &[
                    "vs_fullscreen",
                    "fs_occlusion",
                    "fs_bilateral_blur",
                    "fs_gaussian_blur",
                    "fs_bloom_bright",
                    "fs_resolve",
                    "fs_copy",
                    "fs_fxaa",
                ],
            ),
            ("line", &["vs_line", "fs_line"]),
        ];
        for ((name, source), (_, entries)) in every_module().into_iter().zip(expected) {
            let module = naga::front::wgsl::parse_str(&source).expect("valid wgsl");
            let found: Vec<&str> =
                module.entry_points.iter().map(|e| e.name.as_str()).collect();
            for entry in entries {
                assert!(found.contains(entry), "{name} is missing {entry}, has {found:?}");
            }
        }
    }

    #[test]
    fn the_shaded_surfaces_carry_the_scene_constants_and_the_shared_helpers() {
        let source = scene_module();
        assert!(source.contains("const PI: f32"));
        assert!(source.contains("fn present(linear_colour: vec3<f32>)"));
        assert!(source.contains("fn shadow_factor(wpos: vec3<f32>"));
        assert!(source.contains("fn backdrop_linear(uv: vec2<f32>)"));
        validate(&source);
    }

    #[test]
    fn the_post_chain_does_not_drag_in_the_lighting_helpers_it_never_calls() {
        let source = post_module();
        assert!(!source.contains("sampler_comparison"));
        assert!(!source.contains("fn direct_lobe("));
        assert!(!source.contains("fn shadow_factor("));
    }

    #[test]
    fn scene_constants_reach_the_shader_as_literals_rather_than_uniforms() {
        let source = scene_module();
        assert!(source.contains(&format!("{:.4}", scene::MATERIAL_ROUGHNESS)));
        assert!(source.contains(&format!("{:.6}", scene::ENV_SKY.x)));
        assert!(post_module().contains(&format!("{:.4}", scene::VIGNETTE_STRENGTH)));
        assert!(post_module().contains(&format!("{:.4}", scene::BLOOM_THRESHOLD)));
        assert!(line_module().contains(&format!("{:.4}", scene::LINE_HDR_GAIN)));
    }

    #[test]
    fn the_shadow_offset_is_taken_in_world_units_not_texture_coordinates() {
        let source = scene_module();
        assert!(source.contains("let normal_offset = scene.shadow.z"));
        assert!(source.contains("wpos + n * normal_offset"));
    }

    #[test]
    fn the_shadow_lookup_survives_the_non_uniform_branch_that_precedes_it() {
        let source = scene_module();
        assert!(
            source.contains("textureSampleCompareLevel("),
            "the plain compare sample requires uniform control flow, which the \
             out-of-frustum early return breaks",
        );
        assert!(!source.contains("textureSampleCompare("));
    }

    #[test]
    fn every_texture_read_takes_an_explicit_level_so_loops_may_branch() {
        for (name, source) in every_module() {
            let plain = source.matches("textureSample(").count();
            assert_eq!(
                plain, 0,
                "{name} samples without a level, which is undefined in non-uniform control flow",
            );
        }
    }

    #[test]
    fn the_floor_undoes_the_reflections_premultiplied_coverage_before_mixing() {
        assert!(scene_module().contains("mirror.rgb / max(mirror.a"));
    }

    #[test]
    fn the_planar_reflection_enters_through_the_specular_lobe_not_as_an_overlay() {
        let source = scene_module();
        assert!(
            source.contains("radiance = mix(radiance, reflected, coverage)"),
            "the mirror image must replace the environment radiance the specular lobe samples",
        );
        assert!(
            !source.contains("lit = mix(lit, reflected"),
            "lerping the whole radiance toward the mirror image paints it on as a decal",
        );
        assert!(source.contains("ibl_specular(radiance,"));
    }

    #[test]
    fn ambient_specular_is_never_darkened_by_the_directional_shadow() {
        let source = scene_module();
        assert!(!source.contains("ibl_specular(radiance, f0, FLOOR_ROUGHNESS, nv, shadow)"));
        assert!(source.contains("ibl_specular(radiance, f0, FLOOR_ROUGHNESS, nv, 1.0)"));
    }

    #[test]
    fn the_image_based_specular_carries_occlusion_and_multiple_scattering() {
        let source = scene_module();
        assert!(source.contains("fn specular_occlusion(nv: f32, ao: f32, roughness: f32)"));
        assert!(source.contains("fn energy_compensation(f0: vec3<f32>, dfg: vec2<f32>)"));
        assert!(source.contains("* energy_compensation(f0, dfg)"));
        assert!(source.contains("* specular_occlusion(nv, ao, roughness)"));
    }

    #[test]
    fn layer_lines_march_the_relief_rather_than_only_tinting_the_normal() {
        let source = scene_module();
        assert!(source.contains("fn layer_march("));
        assert!(source.contains("wpos = layer_march(wpos, n, view, lines, atten)"));
        assert!(
            source.contains("shadow_factor(wpos, n)"),
            "shading must use the parallax-corrected position, not the rasterised one",
        );
        assert!(source.contains("layer_self_shadow(wpos, n, key_dir, lines, atten)"));
        assert!(source.contains(&format!("{:.4}", scene::LAYER_HEIGHT)));
    }

    #[test]
    fn layer_lines_are_prefiltered_and_roll_into_roughness_as_they_stop_resolving() {
        let source = scene_module();
        assert!(source.contains("let atten = layer_attenuation(layer_footprint(v.wpos.z));"));
        assert!(
            source.contains("atten * cos(phase)"),
            "the profile must be prefiltered in place, converging to its own mean",
        );
        assert!(
            source.contains("LAYER_SPECULAR_SPREAD * lines * (1.0 - atten)"),
            "detail below the pixel footprint must become roughness, not aliasing",
        );
    }

    #[test]
    fn the_only_derivative_in_the_mesh_shader_sits_in_uniform_control_flow() {
        let source = scene_module();
        let derivative = source.find("fwidth").expect("the band limit takes a derivative");
        let branch = source.find("if (lines > 0.0)").expect("the relief branch exists");
        assert!(derivative < branch);
    }

    #[test]
    fn the_final_blit_is_told_where_its_viewport_starts() {
        let source = post_module();
        assert!(source.contains("fn post_uv(frag: vec2<f32>)"));
        assert!(
            source.contains("(frag - post.origin)"),
            "the builtin position is a framebuffer coordinate; a panel-offset viewport \
             samples past 1.0 and clamps the edge texel across the image",
        );
        assert!(source.contains("post_uv(v.clip.xy)"));
    }

    #[test]
    fn the_blit_hands_a_linear_target_the_encoding_the_hardware_expects() {
        assert!(
            post_module().contains("select(rgb, decode_srgb(rgb), post.flags.x > 0.5)"),
            "an srgb destination re-encodes on write, so the already-encoded resolve \
             output has to be decoded first or it gamma-shifts",
        );
    }

    #[test]
    fn layer_lines_fade_out_on_surfaces_the_nozzle_never_walls() {
        let source = scene_module();
        assert!(source.contains("let facing = 1.0 - abs(n.z);"));
        assert!(source.contains("smoothstep(LAYER_FACING_FADE, 1.0, facing)"));
    }

    #[test]
    fn both_surfaces_share_one_image_based_specular_path() {
        let source = scene_module();
        assert_eq!(source.matches("lit = lit + ibl_specular(").count(), 2);
    }

    #[test]
    fn the_backdrop_focus_is_still_measured_up_the_screen() {
        assert!(
            scene_module().contains("let from_bottom = vec2<f32>(uv.x, 1.0 - uv.y);"),
            "the builtin position runs down the screen where gl_FragCoord ran up it, \
             so the focus constant has to be read against a flipped axis",
        );
    }

    fn kernel_row(index: usize) -> Vec<f32> {
        post_module()
            .split("array<f32, 3>(")
            .nth(index + 1)
            .and_then(|rest| rest.split(')').next())
            .expect("the kernel lists this row")
            .split(',')
            .map(|value| value.trim().parse().expect("a numeric entry"))
            .collect()
    }

    #[test]
    fn the_shared_blur_is_separable_and_spreads_across_distinct_radii() {
        assert!(post_module().contains("post.direction * texel * post.params.x"));
        let offsets = kernel_row(1);
        assert_eq!(offsets.len(), 3);
        assert_eq!(offsets[0], 0.0, "the kernel must sample its own centre");
        for pair in offsets.windows(2) {
            assert!(
                pair[1] > pair[0],
                "taps sharing one radius make a ring, which reproduces a silhouette as \
                 displaced copies rather than blurring it",
            );
        }
    }

    #[test]
    fn the_shared_blur_preserves_energy_and_fades_at_its_outer_tap() {
        let weights = kernel_row(0);
        assert_eq!(weights.len(), 3);
        let total = weights[0] + 2.0 * (weights[1] + weights[2]);
        assert!((total - 1.0).abs() < 1e-4, "the kernel must preserve energy, got {total}");
        assert!(weights[2] < weights[0] && weights[2] < weights[1]);
    }

    #[test]
    fn the_blur_carries_coverage_so_a_premultiplied_reflection_survives_it() {
        let source = post_module();
        let body = source
            .split("fn fs_gaussian_blur")
            .nth(1)
            .and_then(|rest| rest.split("\n@fragment").next())
            .expect("the blur has a body");
        assert!(!body.contains(".rgb"));
        assert!(!body.contains("vec4<f32>(sum, 1.0)"));
    }

    #[test]
    fn the_depth_reconstruction_reads_the_zero_to_one_range_webgpu_hands_it() {
        let source = post_module();
        assert!(
            source.contains("near * far / max(far - depth_at(uv) * (far - near), 1e-6)"),
            "the gl remap of a minus-one-to-one depth would double the near plane",
        );
        assert!(!source.contains("depth * 2.0 - 1.0"));
        assert!(source.contains("out.depth, 1.0)"));
    }

    #[test]
    fn the_backdrop_is_never_defocused_so_it_cannot_gather_the_silhouette() {
        assert!(
            post_module().contains("post.params.w > 0.0 && depth_at(uv) < 1.0"),
            "background pixels hold cleared depth, which reads as the far plane and \
             saturates the circle of confusion into a halo around the model",
        );
    }

    #[test]
    fn a_nearer_defocus_tap_contributes_only_where_its_own_blur_reaches() {
        let source = post_module();
        assert!(source.contains("let in_front = view_depth(at) < centre_depth;"));
        assert!(source.contains("let spreads_this_far = circle_of_confusion(at) >= reach;"));
        assert!(
            !source.contains("step(coc * 0.35, circle_of_confusion(at))"),
            "comparing circle-of-confusion magnitudes passes a near foreground tap and a \
             far background tap alike, which is what bled the model outward",
        );
    }

    #[test]
    fn no_shader_source_carries_a_comment() {
        for (name, source) in every_module() {
            assert!(
                !source.contains("//"),
                "{name}: shader sources in this repository carry no comments",
            );
            assert!(!source.contains("/*"), "{name}");
        }
    }
}
