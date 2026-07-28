use crate::scene;

fn vec3_literal(v: glam::Vec3) -> String {
    format!("vec3({:.6}, {:.6}, {:.6})", v.x, v.y, v.z)
}

fn constants() -> String {
    let mut out = String::new();
    out.push_str("const float PI = 3.141592653589793;\n");
    out.push_str(&format!("const vec3 ENV_SKY = {};\n", vec3_literal(scene::ENV_SKY)));
    out.push_str(&format!("const vec3 ENV_HORIZON = {};\n", vec3_literal(scene::ENV_HORIZON)));
    out.push_str(&format!("const vec3 ENV_GROUND = {};\n", vec3_literal(scene::ENV_GROUND)));
    out.push_str(&format!("const vec3 ENV_SWEEP = {};\n", vec3_literal(scene::ENV_SWEEP)));
    out.push_str(&format!("const vec3 KEY_COLOUR = {};\n", vec3_literal(scene::KEY_COLOUR)));
    out.push_str(&format!("const vec3 FILL_COLOUR = {};\n", vec3_literal(scene::FILL_COLOUR)));
    out.push_str(&format!(
        "const vec3 BACKDROP_CENTRE = {};\n",
        vec3_literal(scene::BACKDROP_CENTRE)
    ));
    out.push_str(&format!("const vec3 BACKDROP_EDGE = {};\n", vec3_literal(scene::BACKDROP_EDGE)));
    out.push_str(&format!(
        "const vec2 BACKDROP_FOCUS = vec2({:.4}, {:.4});\n",
        scene::BACKDROP_FOCUS.0,
        scene::BACKDROP_FOCUS.1
    ));
    out.push_str(&format!("const vec3 FLOOR_ALBEDO = {};\n", vec3_literal(scene::FLOOR_ALBEDO)));
    out.push_str(&format!("const float FLOOR_ROUGHNESS = {:.4};\n", scene::FLOOR_ROUGHNESS));
    out.push_str(&format!(
        "const float FLOOR_FADE_FRACTION = {:.4};\n",
        scene::FLOOR_FADE_FRACTION
    ));
    out.push_str(&format!(
        "const float MATERIAL_ROUGHNESS = {:.4};\n",
        scene::MATERIAL_ROUGHNESS
    ));
    out.push_str(&format!("const float MATERIAL_F0 = {:.4};\n", scene::MATERIAL_F0));
    out.push_str(&format!(
        "const float REFLECTION_STRENGTH = {:.4};\n",
        scene::REFLECTION_STRENGTH
    ));
    out.push_str(&format!(
        "const float CONTACT_SHADOW_STRENGTH = {:.4};\n",
        scene::CONTACT_SHADOW_STRENGTH
    ));
    out.push_str(&format!(
        "const float OCCLUSION_STRENGTH = {:.4};\n",
        scene::AMBIENT_OCCLUSION_STRENGTH
    ));
    out.push_str(&format!("const float EXPOSURE = {:.4};\n", scene::EXPOSURE));
    out.push_str(&format!(
        "const float SHADOW_NORMAL_OFFSET_TEXELS = {:.4};\n",
        scene::SHADOW_NORMAL_OFFSET_TEXELS
    ));
    out.push_str(&format!(
        "const float SHADOW_SLOPE_OFFSET = {:.4};\n",
        scene::SHADOW_SLOPE_OFFSET
    ));
    out.push_str(&format!("const float BLOOM_THRESHOLD = {:.4};\n", scene::BLOOM_THRESHOLD));
    out.push_str(&format!("const float BLOOM_KNEE = {:.4};\n", scene::BLOOM_KNEE));
    out.push_str(&format!("const float BLOOM_INTENSITY = {:.4};\n", scene::BLOOM_INTENSITY));
    out.push_str(&format!("const float VIGNETTE_STRENGTH = {:.4};\n", scene::VIGNETTE_STRENGTH));
    out.push_str(&format!("const float VIGNETTE_RADIUS = {:.4};\n", scene::VIGNETTE_RADIUS));
    out.push_str(&format!("const float LINE_HDR_GAIN = {:.4};\n", scene::LINE_HDR_GAIN));
    out.push_str(&format!("const float LAYER_HEIGHT = {:.4};\n", scene::LAYER_HEIGHT));
    out.push_str(&format!("const float LAYER_RELIEF = {:.4};\n", scene::LAYER_RELIEF));
    out.push_str(&format!(
        "const float LAYER_FACING_FADE = {:.4};\n",
        scene::LAYER_FACING_FADE
    ));
    out.push_str(&format!(
        "const float LAYER_SELF_SHADOW = {:.4};\n",
        scene::LAYER_SELF_SHADOW
    ));
    out.push_str(&format!(
        "const float LAYER_SPECULAR_SPREAD = {:.4};\n",
        scene::LAYER_SPECULAR_SPREAD
    ));
    out.push_str(&format!(
        "const float GI_BOUNCE_STRENGTH = {:.4};\n",
        scene::GI_BOUNCE_STRENGTH
    ));
    out.push_str(&format!("const float DOF_MAX_RADIUS = {:.4};\n", scene::DOF_MAX_RADIUS));
    out
}

const PRECISION: &str = "precision highp float;\nprecision highp int;\n";

const SHADOW_SAMPLER_PRECISION: &str = "precision highp sampler2DShadow;\n";

const COLOUR_PRELUDE: &str = r#"
uniform vec2 u_vp_origin;
uniform vec2 u_vp_size;
uniform float u_scene_ldr;

vec2 screen_uv() {
    return (gl_FragCoord.xy - u_vp_origin) / max(u_vp_size, vec2(1.0));
}

vec3 tonemap(vec3 x) {
    x *= EXPOSURE;
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

vec3 encode_srgb(vec3 c) {
    return pow(max(c, vec3(0.0)), vec3(1.0 / 2.2));
}

vec3 decode_srgb(vec3 c) {
    return pow(max(c, vec3(0.0)), vec3(2.2));
}

vec3 present(vec3 linear_colour) {
    return encode_srgb(tonemap(linear_colour));
}

vec3 scene_rgb(vec3 linear_colour) {
    return u_scene_ldr > 0.5 ? present(linear_colour) : linear_colour;
}

vec4 scene_out(vec3 linear_colour) {
    return vec4(scene_rgb(linear_colour), 1.0);
}

vec3 backdrop_linear(vec2 uv) {
    vec2 d = uv - BACKDROP_FOCUS;
    d.x *= max(u_vp_size.x, 1.0) / max(u_vp_size.y, 1.0);
    float sweep = 1.0 - smoothstep(0.04, 0.92, length(d));
    return mix(BACKDROP_EDGE, BACKDROP_CENTRE, sweep * sweep);
}
"#;

const SHADING_PRELUDE: &str = r#"
vec3 env_irradiance(vec3 n) {
    float t = n.z * 0.5 + 0.5;
    vec3 low = mix(ENV_GROUND, ENV_HORIZON, smoothstep(0.0, 0.55, t));
    return mix(low, ENV_SKY, smoothstep(0.45, 1.0, t));
}

vec3 env_radiance(vec3 r, float roughness) {
    vec3 base = env_irradiance(r);
    float horizon = pow(1.0 - abs(r.z), 5.0);
    return base + ENV_SWEEP * horizon * (1.0 - roughness);
}

float distribution_ggx(float nh, float a) {
    float a2 = a * a;
    float d = nh * nh * (a2 - 1.0) + 1.0;
    return a2 / max(PI * d * d, 1e-7);
}

float visibility_smith(float nv, float nl, float a) {
    float a2 = a * a;
    float gv = nl * sqrt(nv * nv * (1.0 - a2) + a2);
    float gl = nv * sqrt(nl * nl * (1.0 - a2) + a2);
    return 0.5 / max(gv + gl, 1e-5);
}

vec3 fresnel_schlick(vec3 f0, float u) {
    return f0 + (1.0 - f0) * pow(clamp(1.0 - u, 0.0, 1.0), 5.0);
}

vec2 env_dfg(float roughness, float nv) {
    const vec4 c0 = vec4(-1.0, -0.0275, -0.572, 0.022);
    const vec4 c1 = vec4(1.0, 0.0425, 1.04, -0.04);
    vec4 r = roughness * c0 + c1;
    float a004 = min(r.x * r.x, exp2(-9.28 * nv)) * r.x + r.y;
    return vec2(-1.04, 1.04) * a004 + r.zw;
}

vec3 env_brdf(vec3 f0, vec2 dfg) {
    return f0 * dfg.x + dfg.y;
}

vec3 energy_compensation(vec3 f0, vec2 dfg) {
    return vec3(1.0) + f0 * (1.0 / max(dfg.y, 1e-4) - 1.0);
}

float specular_occlusion(float nv, float ao, float roughness) {
    return clamp(pow(nv + ao, exp2(-16.0 * roughness - 1.0)) - 1.0 + ao, 0.0, 1.0);
}

vec3 ibl_specular(vec3 radiance, vec3 f0, float roughness, float nv, float ao) {
    vec2 dfg = env_dfg(roughness, nv);
    return radiance
         * env_brdf(f0, dfg)
         * energy_compensation(f0, dfg)
         * specular_occlusion(nv, ao, roughness);
}

vec3 direct_lobe(vec3 n, vec3 v, vec3 l, vec3 albedo, vec3 f0, float roughness) {
    float nl = max(dot(n, l), 0.0);
    if (nl <= 0.0) {
        return vec3(0.0);
    }
    vec3 h = normalize(l + v);
    float nv = max(dot(n, v), 1e-4);
    float a = max(roughness * roughness, 1e-3);
    float d = distribution_ggx(max(dot(n, h), 0.0), a);
    float vis = visibility_smith(nv, nl, a);
    vec3 f = fresnel_schlick(f0, max(dot(v, h), 0.0));
    vec3 specular = d * vis * f;
    vec3 diffuse = (vec3(1.0) - f) * albedo / PI;
    return (diffuse + specular) * nl;
}

uniform float u_layer_lines;

const int LAYER_STEPS = 12;
const int LAYER_SHADOW_STEPS = 6;

float layer_footprint(float z) {
    return fwidth(z / LAYER_HEIGHT);
}

float layer_attenuation(float footprint) {
    float x = PI * footprint;
    return x < 1e-4 ? 1.0 : clamp(sin(x) / x, 0.0, 1.0);
}

float layer_facing(vec3 n) {
    float facing = 1.0 - abs(n.z);
    return facing < LAYER_FACING_FADE ? 0.0 : smoothstep(LAYER_FACING_FADE, 1.0, facing);
}

float layer_depth(float z, float facing, float atten) {
    float phase = z * (2.0 * PI) / LAYER_HEIGHT;
    return LAYER_RELIEF * facing * 0.5 * (1.0 - atten * cos(phase));
}

float layer_slope(float z, float facing, float atten) {
    float phase = z * (2.0 * PI) / LAYER_HEIGHT;
    return LAYER_RELIEF * facing * atten * PI / LAYER_HEIGHT * sin(phase);
}

vec3 layer_march(vec3 wpos, vec3 n, vec3 v, float facing, float atten) {
    vec3 march = -v / max(dot(n, v), 0.08);
    float dt = LAYER_RELIEF / float(LAYER_STEPS);
    float prev_t = 0.0;
    float prev_h = layer_depth(wpos.z, facing, atten);
    float t = 0.0;
    for (int i = 0; i < LAYER_STEPS; ++i) {
        t += dt;
        float h = layer_depth((wpos + march * t).z, facing, atten);
        if (h <= t) {
            float before = prev_h - prev_t;
            float after = h - t;
            t = mix(prev_t, t, before / max(before - after, 1e-6));
            break;
        }
        prev_t = t;
        prev_h = h;
    }
    return wpos + march * t;
}

vec3 layer_normal(vec3 n, float slope) {
    vec3 along = vec3(0.0, 0.0, 1.0) - n * n.z;
    float len = length(along);
    return len < 1e-4 ? n : normalize(n + (along / len) * slope * len);
}

float layer_self_shadow(vec3 hit, vec3 n, vec3 l, float facing, float atten) {
    float nl = dot(n, l);
    if (nl <= 0.0 || facing <= 0.0) {
        return 1.0;
    }
    vec3 march = l / max(nl, 0.08);
    float here = layer_depth(hit.z, facing, atten);
    float blocked = 0.0;
    for (int i = 1; i <= LAYER_SHADOW_STEPS; ++i) {
        float t = LAYER_RELIEF * float(i) / float(LAYER_SHADOW_STEPS);
        float h = layer_depth((hit + march * t).z, facing, atten);
        blocked = max(blocked, (here - t) - h);
    }
    return 1.0 - clamp(blocked / LAYER_RELIEF, 0.0, 1.0) * LAYER_SELF_SHADOW;
}

uniform mat4 u_light_vp;
uniform sampler2DShadow u_shadow;
uniform float u_shadow_enabled;
uniform float u_shadow_texel;
uniform float u_shadow_world_texel;
uniform int u_shadow_taps;
uniform vec3 u_key_dir;

float shadow_factor(vec3 wpos, vec3 n) {
    if (u_shadow_enabled < 0.5) {
        return 1.0;
    }
    float slope = clamp(1.0 - dot(n, u_key_dir), 0.0, 1.0);
    float normal_offset = u_shadow_world_texel
                        * (SHADOW_NORMAL_OFFSET_TEXELS + slope * SHADOW_SLOPE_OFFSET);
    vec4 clip = u_light_vp * vec4(wpos + n * normal_offset, 1.0);
    vec3 coord = clip.xyz / clip.w * 0.5 + 0.5;
    if (coord.x < 0.0 || coord.x > 1.0 || coord.y < 0.0 || coord.y > 1.0
     || coord.z < 0.0 || coord.z > 1.0) {
        return 1.0;
    }
    coord.z -= 0.0015 + slope * 0.004;
    float sum = 0.0;
    float count = 0.0;
    for (int y = -2; y <= 2; ++y) {
        for (int x = -2; x <= 2; ++x) {
            if (abs(x) > u_shadow_taps || abs(y) > u_shadow_taps) {
                continue;
            }
            vec2 offset = vec2(float(x), float(y)) * u_shadow_texel;
            sum += texture(u_shadow, vec3(coord.xy + offset, coord.z));
            count += 1.0;
        }
    }
    return mix(1.0, sum / max(count, 1.0), CONTACT_SHADOW_STRENGTH);
}
"#;

pub const FULLSCREEN_VS: &str = r#"
    void main() {
        vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
        gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
    }
"#;

pub const MESH_VS: &str = r#"
    layout (location = 0) in vec3 a_pos;
    layout (location = 1) in vec3 a_normal;
    layout (location = 2) in vec3 a_color;
    layout (location = 3) in float a_bad;
    uniform mat4 u_view_proj;
    uniform mat4 u_model;
    out vec3 v_normal;
    out vec3 v_wpos;
    out vec3 v_color;
    out float v_bad;
    void main() {
        v_normal = a_normal;
        v_wpos = a_pos;
        v_color = a_color;
        v_bad = a_bad;
        gl_Position = u_view_proj * u_model * vec4(a_pos, 1.0);
    }
"#;

const MESH_FS_BODY: &str = r#"
    in vec3 v_normal;
    in vec3 v_wpos;
    in vec3 v_color;
    in float v_bad;
    out vec4 frag;
    uniform vec3 u_eye;
    uniform vec3 u_fill_dir;
    uniform float u_time;
    uniform sampler2D u_occlusion;
    uniform float u_occlusion_enabled;

    void main() {
        float atten = layer_attenuation(layer_footprint(v_wpos.z));
        vec3 n = normalize(v_normal);
        vec3 v = normalize(u_eye - v_wpos);
        if (dot(n, v) < 0.0) {
            n = -n;
        }
        float nv = max(dot(n, v), 1e-4);
        vec4 gi = vec4(0.0, 0.0, 0.0, 1.0);
        if (u_occlusion_enabled > 0.5) {
            gi = texture(u_occlusion, screen_uv());
        }
        float ao = mix(1.0, gi.a, OCCLUSION_STRENGTH);

        if (v_bad > 0.5) {
            float rim = pow(1.0 - nv, 2.5);
            float pulse = 0.70 + 0.30 * sin(u_time * 3.2);
            float key = max(dot(n, u_key_dir), 0.0);
            vec3 lit = vec3(0.32, 0.015, 0.02) * (0.35 + 0.9 * key) * ao
                     + vec3(2.60, 0.30, 0.14) * rim * pulse;
            frag = scene_out(lit);
            return;
        }

        vec3 wpos = v_wpos;
        float grooves = 1.0;
        float lines = u_layer_lines > 0.5 ? layer_facing(n) : 0.0;
        if (lines > 0.0) {
            wpos = layer_march(wpos, n, v, lines, atten);
            grooves = layer_self_shadow(wpos, n, u_key_dir, lines, atten);
            n = layer_normal(n, layer_slope(wpos.z, lines, atten));
            nv = max(dot(n, v), 1e-4);
        }

        vec3 albedo = decode_srgb(v_color);
        vec3 f0 = vec3(MATERIAL_F0);
        float roughness = min(
            1.0,
            MATERIAL_ROUGHNESS + LAYER_SPECULAR_SPREAD * lines * (1.0 - atten)
        );
        float shadow = shadow_factor(wpos, n) * grooves;

        vec3 lit = direct_lobe(n, v, u_key_dir, albedo, f0, roughness) * KEY_COLOUR * shadow;
        lit += albedo / PI * FILL_COLOUR * max(dot(n, u_fill_dir), 0.0);
        lit += albedo * (u_occlusion_enabled > 0.5 ? gi.rgb : env_irradiance(n));
        lit += ibl_specular(env_radiance(reflect(-v, n), roughness), f0, roughness, nv, ao);

        frag = scene_out(lit);
    }
"#;

const BACKDROP_FS_BODY: &str = r#"
    out vec4 frag;
    void main() {
        frag = scene_out(backdrop_linear(screen_uv()));
    }
"#;

pub const FLOOR_VS: &str = r#"
    uniform mat4 u_view_proj;
    uniform vec3 u_floor_centre;
    uniform float u_floor_radius;
    out vec3 v_wpos;
    void main() {
        vec2 corner = vec2(float(gl_VertexID & 1), float((gl_VertexID >> 1) & 1)) * 2.0 - 1.0;
        vec3 world = u_floor_centre + vec3(corner * u_floor_radius, 0.0);
        v_wpos = world;
        gl_Position = u_view_proj * vec4(world, 1.0);
    }
"#;

const FLOOR_FS_BODY: &str = r#"
    in vec3 v_wpos;
    out vec4 frag;
    uniform vec3 u_eye;
    uniform vec3 u_fill_dir;
    uniform vec3 u_floor_centre;
    uniform float u_floor_radius;
    uniform sampler2D u_reflection;
    uniform float u_reflection_weight;

    void main() {
        vec2 uv = screen_uv();
        vec3 backdrop = backdrop_linear(uv);
        float radial = length(v_wpos.xy - u_floor_centre.xy) / max(u_floor_radius, 1e-3);
        float presence = 1.0 - smoothstep(1.0 - FLOOR_FADE_FRACTION, 1.0, radial);
        if (presence <= 0.0) {
            frag = scene_out(backdrop);
            return;
        }

        vec3 n = vec3(0.0, 0.0, 1.0);
        vec3 v = normalize(u_eye - v_wpos);
        float nv = max(dot(n, v), 1e-4);
        vec3 f0 = vec3(MATERIAL_F0);
        float shadow = shadow_factor(v_wpos, n);

        vec3 lit = direct_lobe(n, v, u_key_dir, FLOOR_ALBEDO, f0, FLOOR_ROUGHNESS)
                 * KEY_COLOUR * shadow;
        lit += FLOOR_ALBEDO / PI * FILL_COLOUR * max(dot(n, u_fill_dir), 0.0);
        lit += FLOOR_ALBEDO * env_irradiance(n);

        vec3 radiance = env_radiance(reflect(-v, n), FLOOR_ROUGHNESS);
        if (u_reflection_weight > 0.0) {
            vec4 mirror = texture(u_reflection, uv);
            vec3 reflected = mirror.rgb / max(mirror.a, 1e-4);
            float coverage = clamp(
                mirror.a * REFLECTION_STRENGTH * u_reflection_weight,
                0.0,
                1.0
            );
            radiance = mix(radiance, reflected, coverage);
        }
        lit += ibl_specular(radiance, f0, FLOOR_ROUGHNESS, nv, 1.0);

        frag = scene_out(mix(backdrop, lit, presence));
    }
"#;

pub const DEPTH_NORMAL_FS: &str = r#"
    precision highp float;
    in vec3 v_normal;
    in vec3 v_wpos;
    in vec3 v_color;
    in float v_bad;
    out vec4 frag;
    void main() {
        frag = vec4(normalize(v_normal) * 0.5 + 0.5, 1.0);
    }
"#;

const OCCLUSION_FS_BODY: &str = r#"
    out vec4 frag;
    uniform sampler2D u_depth;
    uniform sampler2D u_normal;
    uniform sampler2D u_previous;
    uniform float u_bounce;
    uniform mat4 u_view_proj;
    uniform mat4 u_inv_view_proj;
    uniform vec2 u_target_size;
    uniform vec3 u_eye;
    uniform float u_radius;
    uniform int u_samples;
    uniform float u_frame;

    vec3 world_at(vec2 uv, out float depth) {
        depth = texture(u_depth, uv).r;
        vec4 clip = vec4(uv * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
        vec4 world = u_inv_view_proj * clip;
        return world.xyz / world.w;
    }

    const float OCCLUSION_GOLDEN_ANGLE = 2.39996323;

    float gradient_noise(vec2 p) {
        return fract(52.9829189 * fract(dot(p, vec2(0.06711056, 0.00583715))));
    }

    void tangent_basis(vec3 n, out vec3 tangent, out vec3 bitangent) {
        vec3 up = abs(n.z) < 0.999 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
        tangent = normalize(cross(up, n));
        bitangent = cross(n, tangent);
    }

    void main() {
        vec2 uv = gl_FragCoord.xy / max(u_target_size, vec2(1.0));
        float depth;
        vec3 origin = world_at(uv, depth);
        if (depth >= 1.0) {
            frag = vec4(1.0);
            return;
        }
        vec3 n = normalize(texture(u_normal, uv).rgb * 2.0 - 1.0);
        float origin_dist = distance(origin, u_eye);
        float occlusion = 0.0;
        float weight = 0.0;
        vec3 bent = vec3(0.0);
        vec3 bounce = vec3(0.0);
        vec3 tangent;
        vec3 bitangent;
        tangent_basis(n, tangent, bitangent);
        float rotation = gradient_noise(gl_FragCoord.xy + vec2(u_frame * 5.588238));
        vec3 lifted = origin + n * u_radius * 0.02;
        for (int i = 0; i < 32; ++i) {
            if (i >= u_samples) {
                break;
            }
            float slice = (float(i) + 0.5) / float(u_samples);
            float angle = (float(i) + rotation) * OCCLUSION_GOLDEN_ANGLE;
            float ring = sqrt(slice);
            vec3 dir = tangent * (cos(angle) * ring)
                     + bitangent * (sin(angle) * ring)
                     + n * sqrt(max(1.0 - slice, 0.0));
            float span = mix(0.25, 1.0, fract(slice + rotation));
            vec3 sample_pos = lifted + dir * u_radius * span;
            vec4 clip = u_view_proj * vec4(sample_pos, 1.0);
            if (clip.w <= 1e-5) {
                continue;
            }
            vec2 sample_uv = clip.xy / clip.w * 0.5 + 0.5;
            if (sample_uv.x < 0.0 || sample_uv.x > 1.0
             || sample_uv.y < 0.0 || sample_uv.y > 1.0) {
                continue;
            }
            float surface_depth;
            vec3 surface = world_at(sample_uv, surface_depth);
            weight += 1.0;
            if (surface_depth >= 1.0) {
                bent += dir;
                continue;
            }
            float surface_dist = distance(surface, u_eye);
            float sample_dist = distance(sample_pos, u_eye);
            if (surface_dist < sample_dist - u_radius * 0.03) {
                float range = smoothstep(
                    0.0,
                    1.0,
                    u_radius / max(abs(origin_dist - surface_dist), 1e-4)
                );
                occlusion += range;
                if (u_bounce > 0.5) {
                    vec3 toward = surface - origin;
                    float span = length(toward);
                    float facing = span > 1e-5 ? max(dot(n, toward / span), 0.0) : 0.0;
                    bounce += texture(u_previous, sample_uv).rgb * range * facing;
                }
            } else {
                bent += dir;
            }
        }
        float ao = 1.0 - occlusion / max(weight, 1.0);
        vec3 bent_normal = dot(bent, bent) > 1e-8 ? normalize(bent) : n;
        vec3 indirect = env_irradiance(bent_normal) * ao
                      + bounce * GI_BOUNCE_STRENGTH / max(weight, 1.0);
        frag = vec4(indirect, ao);
    }
"#;

pub const BILATERAL_BLUR_FS: &str = r#"
    precision highp float;
    out vec4 frag;
    uniform sampler2D u_source;
    uniform sampler2D u_depth;
    uniform vec2 u_target_size;
    uniform vec2 u_direction;
    uniform float u_near;
    uniform float u_far;
    uniform float u_tolerance;

    float view_depth(vec2 at) {
        float z = texture(u_depth, at).r * 2.0 - 1.0;
        return (2.0 * u_near * u_far) / (u_far + u_near - z * (u_far - u_near));
    }

    void main() {
        vec2 texel = 1.0 / max(u_target_size, vec2(1.0));
        vec2 uv = gl_FragCoord.xy * texel;
        float centre = view_depth(uv);
        vec4 sum = texture(u_source, uv);
        float weight = 1.0;
        for (int i = 1; i <= 4; ++i) {
            float falloff = exp(-float(i * i) * 0.14);
            for (int side = 0; side < 2; ++side) {
                vec2 offset = u_direction * texel * float(i) * (side == 0 ? 1.0 : -1.0);
                vec2 at = uv + offset;
                float w = falloff
                        * exp(-abs(view_depth(at) - centre) / max(u_tolerance, 1e-4));
                sum += texture(u_source, at) * w;
                weight += w;
            }
        }
        frag = sum / max(weight, 1e-4);
    }
"#;

const BLOOM_BRIGHT_FS_BODY: &str = r#"
    out vec4 frag;
    uniform sampler2D u_source;
    uniform vec2 u_target_size;
    void main() {
        vec2 uv = gl_FragCoord.xy / max(u_target_size, vec2(1.0));
        vec3 c = texture(u_source, uv).rgb;
        float brightness = max(c.r, max(c.g, c.b));
        float knee = max(BLOOM_THRESHOLD * BLOOM_KNEE, 1e-4);
        float soft = clamp(brightness - BLOOM_THRESHOLD + knee, 0.0, 2.0 * knee);
        soft = soft * soft / (4.0 * knee);
        float contribution = max(soft, brightness - BLOOM_THRESHOLD) / max(brightness, 1e-4);
        frag = vec4(c * contribution, 1.0);
    }
"#;

pub const GAUSSIAN_BLUR_FS: &str = r#"
    precision highp float;
    out vec4 frag;
    uniform sampler2D u_source;
    uniform vec2 u_target_size;
    uniform vec2 u_direction;
    uniform float u_radius;
    const float WEIGHTS[3] = float[3](0.2270270270, 0.3162162162, 0.0702702703);
    const float OFFSETS[3] = float[3](0.0, 1.3846153846, 3.2307692308);
    void main() {
        vec2 texel = 1.0 / max(u_target_size, vec2(1.0));
        vec2 uv = gl_FragCoord.xy * texel;
        vec2 stride = u_direction * texel * u_radius;
        vec4 sum = texture(u_source, uv) * WEIGHTS[0];
        for (int i = 1; i < 3; ++i) {
            vec2 offset = stride * OFFSETS[i];
            sum += texture(u_source, uv + offset) * WEIGHTS[i];
            sum += texture(u_source, uv - offset) * WEIGHTS[i];
        }
        frag = sum;
    }
"#;

const RESOLVE_FS_BODY: &str = r#"
    out vec4 frag;
    uniform sampler2D u_scene;
    uniform sampler2D u_bloom;
    uniform sampler2D u_depth;
    uniform vec2 u_target_size;
    uniform float u_bloom_enabled;
    uniform float u_scene_linear;
    uniform float u_near;
    uniform float u_far;
    uniform float u_focus;
    uniform float u_aperture;

    const int DOF_TAPS = 16;
    const float GOLDEN_ANGLE = 2.39996323;

    float view_depth(vec2 at) {
        float z = texture(u_depth, at).r * 2.0 - 1.0;
        return (2.0 * u_near * u_far) / (u_far + u_near - z * (u_far - u_near));
    }

    float circle_of_confusion(vec2 at) {
        float distance_to = view_depth(at);
        return clamp(
            abs(distance_to - u_focus) / max(u_focus, 1e-3) * u_aperture,
            0.0,
            DOF_MAX_RADIUS
        );
    }

    vec3 gather_defocus(vec2 uv, vec2 texel, float coc) {
        vec3 sum = texture(u_scene, uv).rgb;
        float weight = 1.0;
        for (int i = 1; i <= DOF_TAPS; ++i) {
            float t = float(i) / float(DOF_TAPS);
            float angle = float(i) * GOLDEN_ANGLE;
            vec2 at = uv + vec2(cos(angle), sin(angle)) * sqrt(t) * coc * texel;
            float w = step(coc * 0.35, circle_of_confusion(at));
            sum += texture(u_scene, at).rgb * w;
            weight += w;
        }
        return sum / max(weight, 1e-4);
    }

    void main() {
        vec2 texel = 1.0 / max(u_target_size, vec2(1.0));
        vec2 uv = gl_FragCoord.xy * texel;
        vec3 c = texture(u_scene, uv).rgb;
        if (u_aperture > 0.0) {
            float coc = circle_of_confusion(uv);
            if (coc > 1.0) {
                c = gather_defocus(uv, texel, coc);
            }
        }
        if (u_bloom_enabled > 0.5) {
            c += texture(u_bloom, uv).rgb * BLOOM_INTENSITY;
        }
        float radial = length((uv - 0.5) * 2.0);
        float vignette = mix(
            1.0 - VIGNETTE_STRENGTH,
            1.0,
            smoothstep(1.0, VIGNETTE_RADIUS, radial)
        );
        c *= vignette;
        frag = vec4(u_scene_linear > 0.5 ? present(c) : c, 1.0);
    }
"#;

pub const COPY_FS: &str = r#"
    precision highp float;
    out vec4 frag;
    uniform sampler2D u_source;
    uniform vec2 u_target_size;
    uniform vec2 u_origin;
    void main() {
        frag = texture(u_source, (gl_FragCoord.xy - u_origin) / max(u_target_size, vec2(1.0)));
    }
"#;

pub const FXAA_FS: &str = r#"
    precision highp float;
    out vec4 frag;
    uniform sampler2D u_source;
    uniform vec2 u_target_size;
    uniform vec2 u_origin;

    const float EDGE_THRESHOLD = 0.125;
    const float EDGE_THRESHOLD_MIN = 0.0312;
    const float DIRECTION_REDUCE = 0.125;
    const float DIRECTION_REDUCE_MIN = 0.0078125;
    const float SPAN_MAX = 8.0;

    float luma(vec3 c) {
        return dot(c, vec3(0.299, 0.587, 0.114));
    }

    void main() {
        vec2 texel = 1.0 / max(u_target_size, vec2(1.0));
        vec2 uv = (gl_FragCoord.xy - u_origin) * texel;
        vec3 rgb_m = texture(u_source, uv).rgb;
        float luma_m = luma(rgb_m);
        float luma_n = luma(texture(u_source, uv + vec2(0.0, texel.y)).rgb);
        float luma_s = luma(texture(u_source, uv - vec2(0.0, texel.y)).rgb);
        float luma_e = luma(texture(u_source, uv + vec2(texel.x, 0.0)).rgb);
        float luma_w = luma(texture(u_source, uv - vec2(texel.x, 0.0)).rgb);
        float luma_min = min(luma_m, min(min(luma_n, luma_s), min(luma_e, luma_w)));
        float luma_max = max(luma_m, max(max(luma_n, luma_s), max(luma_e, luma_w)));
        float range = luma_max - luma_min;
        if (range < max(EDGE_THRESHOLD_MIN, luma_max * EDGE_THRESHOLD)) {
            frag = vec4(rgb_m, 1.0);
            return;
        }

        float luma_nw = luma(texture(u_source, uv + vec2(-texel.x, texel.y)).rgb);
        float luma_ne = luma(texture(u_source, uv + vec2(texel.x, texel.y)).rgb);
        float luma_sw = luma(texture(u_source, uv + vec2(-texel.x, -texel.y)).rgb);
        float luma_se = luma(texture(u_source, uv + vec2(texel.x, -texel.y)).rgb);

        vec2 dir = vec2(
            -((luma_nw + luma_ne) - (luma_sw + luma_se)),
            ((luma_nw + luma_sw) - (luma_ne + luma_se))
        );
        float reduction = max(
            (luma_nw + luma_ne + luma_sw + luma_se) * 0.25 * DIRECTION_REDUCE,
            DIRECTION_REDUCE_MIN
        );
        float rcp = 1.0 / (min(abs(dir.x), abs(dir.y)) + reduction);
        dir = clamp(dir * rcp, vec2(-SPAN_MAX), vec2(SPAN_MAX)) * texel;

        vec3 inner = 0.5 * (
            texture(u_source, uv + dir * (1.0 / 3.0 - 0.5)).rgb
          + texture(u_source, uv + dir * (2.0 / 3.0 - 0.5)).rgb
        );
        vec3 outer = inner * 0.5 + 0.25 * (
            texture(u_source, uv - dir * 0.5).rgb
          + texture(u_source, uv + dir * 0.5).rgb
        );
        float luma_outer = luma(outer);
        frag = vec4(luma_outer < luma_min || luma_outer > luma_max ? inner : outer, 1.0);
    }
"#;

pub const LINE_VS: &str = r#"
    layout (location = 0) in vec3 a_p0;
    layout (location = 1) in vec3 a_p1;
    layout (location = 2) in vec3 a_color;
    layout (location = 3) in float a_end;
    layout (location = 4) in float a_side;
    uniform mat4 u_view_proj;
    uniform vec2 u_half_vp;
    uniform float u_width;
    out vec3 v_color;
    out float v_across;
    out float v_half;
    void main() {
        vec4 c0 = u_view_proj * vec4(a_p0, 1.0);
        vec4 c1 = u_view_proj * vec4(a_p1, 1.0);
        vec4 c = (a_end < 0.5) ? c0 : c1;
        vec2 s0 = c0.xy / max(abs(c0.w), 1e-4) * u_half_vp;
        vec2 s1 = c1.xy / max(abs(c1.w), 1e-4) * u_half_vp;
        vec2 d = s1 - s0;
        float len = length(d);
        vec2 dir = len > 1e-6 ? d / len : vec2(1.0, 0.0);
        vec2 nrm = vec2(-dir.y, dir.x);
        float half_w = 0.5 * u_width;
        float ext = half_w + 1.0;
        gl_Position = c;
        gl_Position.xy += nrm * a_side * ext / u_half_vp * c.w;
        v_color = a_color;
        v_across = a_side * ext;
        v_half = half_w;
    }
"#;

const LINE_FS_BODY: &str = r#"
    in vec3 v_color;
    in float v_across;
    in float v_half;
    out vec4 frag;
    uniform float u_alpha;
    void main() {
        float d = abs(v_across);
        float aa = fwidth(d);
        float a = 1.0 - smoothstep(v_half - aa, v_half + aa, d);
        if (a <= 0.0) discard;
        vec3 rgb = u_scene_ldr > 0.5 ? v_color : decode_srgb(v_color) * LINE_HDR_GAIN;
        frag = vec4(rgb, a * u_alpha);
    }
"#;

fn presented(body: &str) -> String {
    format!("{}{}{}\n{}", PRECISION, constants(), COLOUR_PRELUDE, body)
}

fn shaded(body: &str) -> String {
    format!(
        "{}{}{}{}{}\n{}",
        PRECISION,
        SHADOW_SAMPLER_PRECISION,
        constants(),
        COLOUR_PRELUDE,
        SHADING_PRELUDE,
        body
    )
}

pub fn mesh_fs() -> String {
    shaded(MESH_FS_BODY)
}

pub fn backdrop_fs() -> String {
    shaded(BACKDROP_FS_BODY)
}

pub fn floor_fs() -> String {
    shaded(FLOOR_FS_BODY)
}

pub fn line_fs() -> String {
    presented(LINE_FS_BODY)
}

pub fn bloom_bright_fs() -> String {
    presented(BLOOM_BRIGHT_FS_BODY)
}

pub fn resolve_fs() -> String {
    presented(RESOLVE_FS_BODY)
}

pub fn occlusion_fs() -> String {
    shaded(OCCLUSION_FS_BODY)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn generated() -> Vec<String> {
        vec![
            mesh_fs(),
            backdrop_fs(),
            floor_fs(),
            line_fs(),
            bloom_bright_fs(),
            resolve_fs(),
            occlusion_fs(),
        ]
    }

    fn every_source() -> Vec<String> {
        let mut all = generated();
        for fixed in [
            MESH_VS,
            FLOOR_VS,
            FULLSCREEN_VS,
            LINE_VS,
            DEPTH_NORMAL_FS,
            BILATERAL_BLUR_FS,
            GAUSSIAN_BLUR_FS,
            FXAA_FS,
        ] {
            all.push(fixed.to_string());
        }
        all
    }

    #[test]
    fn every_shaded_program_carries_the_scene_constants_and_the_shared_helpers() {
        for source in [mesh_fs(), backdrop_fs(), floor_fs()] {
            assert!(source.contains("const float PI"));
            assert!(source.contains("vec3 present(vec3"));
            assert!(source.contains("float shadow_factor(vec3"));
            assert!(source.contains("vec3 backdrop_linear(vec2"));
        }
    }

    #[test]
    fn every_generated_program_can_choose_between_linear_and_presented_output() {
        for source in generated() {
            assert!(source.contains("uniform float u_scene_ldr"));
        }
    }

    #[test]
    fn the_post_chain_does_not_drag_in_the_lighting_helpers_it_never_calls() {
        for source in [line_fs(), bloom_bright_fs(), resolve_fs()] {
            assert!(!source.contains("sampler2DShadow"));
            assert!(!source.contains("vec3 direct_lobe("));
        }
    }

    #[test]
    fn scene_constants_reach_the_shader_as_literals_rather_than_uniforms() {
        let source = mesh_fs();
        assert!(source.contains(&format!("{:.4}", scene::MATERIAL_ROUGHNESS)));
        assert!(source.contains(&format!("{:.6}", scene::ENV_SKY.x)));
        assert!(resolve_fs().contains(&format!("{:.4}", scene::VIGNETTE_STRENGTH)));
        assert!(bloom_bright_fs().contains(&format!("{:.4}", scene::BLOOM_THRESHOLD)));
        assert!(line_fs().contains(&format!("{:.4}", scene::LINE_HDR_GAIN)));
    }

    #[test]
    fn the_shadow_offset_is_taken_in_world_units_not_texture_coordinates() {
        let source = mesh_fs();
        assert!(source.contains("uniform float u_shadow_world_texel"));
        assert!(source.contains("float normal_offset = u_shadow_world_texel"));
        assert!(source.contains("wpos + n * normal_offset"));
        assert!(!source.contains("u_shadow_texel * 2.5"));
    }

    #[test]
    fn the_floor_undoes_the_reflections_premultiplied_coverage_before_mixing() {
        assert!(floor_fs().contains("mirror.rgb / max(mirror.a"));
    }

    #[test]
    fn the_planar_reflection_enters_through_the_specular_lobe_not_as_an_overlay() {
        let source = floor_fs();
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
        for source in [mesh_fs(), floor_fs()] {
            assert!(!source.contains("env_brdf(f0, roughness, nv) * shadow"));
            assert!(!source.contains("ibl_specular(radiance, f0, FLOOR_ROUGHNESS, nv, shadow)"));
        }
    }

    #[test]
    fn the_image_based_specular_carries_occlusion_and_multiple_scattering() {
        let source = mesh_fs();
        assert!(source.contains("float specular_occlusion(float nv, float ao, float roughness)"));
        assert!(source.contains("vec3 energy_compensation(vec3 f0, vec2 dfg)"));
        assert!(source.contains("* energy_compensation(f0, dfg)"));
        assert!(source.contains("* specular_occlusion(nv, ao, roughness)"));
    }

    #[test]
    fn layer_lines_march_the_relief_rather_than_only_tinting_the_normal() {
        let source = mesh_fs();
        assert!(source.contains("vec3 layer_march(vec3 wpos, vec3 n, vec3 v, float facing, float atten)"));
        assert!(source.contains("wpos = layer_march(wpos, n, v, lines, atten)"));
        assert!(
            source.contains("shadow_factor(wpos, n)"),
            "shading must use the parallax-corrected position, not the rasterised one",
        );
        assert!(source.contains("layer_self_shadow(wpos, n, u_key_dir, lines, atten)"));
        assert!(source.contains(&format!("{:.4}", scene::LAYER_HEIGHT)));
    }

    #[test]
    fn layer_lines_are_prefiltered_and_roll_into_roughness_as_they_stop_resolving() {
        let source = mesh_fs();
        assert!(source.contains("float atten = layer_attenuation(layer_footprint(v_wpos.z));"));
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
        let source = mesh_fs();
        assert_eq!(
            source.matches("fwidth").count(),
            1,
            "a derivative reached from a non-uniform branch is undefined in GLSL ES",
        );
        let derivative = source.find("fwidth").expect("the band limit takes a derivative");
        let branch = source.find("if (lines > 0.0)").expect("the relief branch exists");
        assert!(derivative < branch);
    }

    #[test]
    fn the_final_blit_is_told_where_its_viewport_starts() {
        assert!(FXAA_FS.contains("uniform vec2 u_origin"));
        assert!(
            FXAA_FS.contains("(gl_FragCoord.xy - u_origin)"),
            "gl_FragCoord is a window coordinate; a panel-offset viewport samples past 1.0 \
             and clamps the edge texel across the image",
        );
    }

    #[test]
    fn layer_lines_fade_out_on_surfaces_the_nozzle_never_walls() {
        assert!(mesh_fs().contains("float facing = 1.0 - abs(n.z);"));
        assert!(mesh_fs().contains("smoothstep(LAYER_FACING_FADE, 1.0, facing)"));
    }

    #[test]
    fn both_surfaces_share_one_image_based_specular_path() {
        for source in [mesh_fs(), floor_fs()] {
            assert!(source.contains("lit += ibl_specular("));
        }
    }

    fn kernel_row(index: usize) -> Vec<f32> {
        GAUSSIAN_BLUR_FS
            .split("float[3](")
            .nth(index + 1)
            .and_then(|rest| rest.split(')').next())
            .expect("the kernel lists this row")
            .split(',')
            .map(|value| value.trim().parse().expect("a numeric entry"))
            .collect()
    }

    #[test]
    fn the_shared_blur_is_separable_and_spreads_across_distinct_radii() {
        assert!(GAUSSIAN_BLUR_FS.contains("uniform vec2 u_direction"));
        let offsets = kernel_row(1);
        assert_eq!(offsets.len(), 3);
        assert_eq!(offsets[0], 0.0, "the kernel must sample its own centre");
        for pair in offsets.windows(2) {
            assert!(
                pair[1] > pair[0],
                "taps sharing one radius make a ring, which reproduces a silhouette as displaced \
                 copies rather than blurring it",
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
        assert!(!GAUSSIAN_BLUR_FS.contains(".rgb"));
        assert!(!GAUSSIAN_BLUR_FS.contains("vec4(sum, 1.0)"));
    }

    #[test]
    fn every_fragment_source_sets_a_float_precision_before_it_declares_a_float() {
        let fragment = {
            let mut sources = generated();
            for fixed in [DEPTH_NORMAL_FS, BILATERAL_BLUR_FS, GAUSSIAN_BLUR_FS, FXAA_FS, COPY_FS] {
                sources.push(fixed.to_string());
            }
            sources
        };
        for source in fragment {
            let precision =
                source.find("precision highp float").expect("a fragment source sets a precision");
            let first_float = source.find("float").expect("a fragment source declares a float");
            assert!(
                precision <= first_float,
                "GLSL ES rejects a float declared before any precision qualifier, which desktop \
                 GL silently accepts",
            );
        }
    }

    #[test]
    fn no_shader_source_declares_its_own_version_directive() {
        for source in every_source() {
            assert!(!source.contains("#version"));
        }
    }

    #[test]
    fn no_shader_source_carries_a_comment() {
        for source in every_source() {
            assert!(!source.contains("//"), "shader sources in this repository carry no comments");
            assert!(!source.contains("/*"));
        }
    }
}
