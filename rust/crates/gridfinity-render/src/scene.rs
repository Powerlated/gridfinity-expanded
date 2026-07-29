use glam::{Mat4, Vec3};

pub const ENV_SKY: Vec3 = Vec3::new(0.44, 0.47, 0.54);
pub const ENV_HORIZON: Vec3 = Vec3::new(0.16, 0.17, 0.20);
pub const ENV_GROUND: Vec3 = Vec3::new(0.030, 0.030, 0.035);
pub const ENV_SWEEP: Vec3 = Vec3::new(0.11, 0.11, 0.12);

pub const KEY_COLOUR: Vec3 = Vec3::new(3.10, 3.04, 2.90);
pub const FILL_COLOUR: Vec3 = Vec3::new(0.34, 0.37, 0.46);

pub const BACKDROP_CENTRE: Vec3 = Vec3::new(0.0320, 0.0332, 0.0405);
pub const BACKDROP_EDGE: Vec3 = Vec3::new(0.0050, 0.0052, 0.0070);
pub const BACKDROP_FOCUS: (f32, f32) = (0.5, 0.60);

pub const MATERIAL_ROUGHNESS: f32 = 0.40;
pub const MATERIAL_F0: f32 = 0.04;

pub const FLOOR_ALBEDO: Vec3 = Vec3::new(0.0180, 0.0188, 0.0225);
pub const FLOOR_ROUGHNESS: f32 = 0.22;
pub const FLOOR_RADIUS_MULTIPLE: f32 = 7.0;
pub const FLOOR_FADE_FRACTION: f32 = 0.35;

pub const REFLECTION_STRENGTH: f32 = 0.85;
pub const REFLECTION_GLOSS_RADIUS: f32 = 2.0;

pub const FLOOR_GRAZING_FADE_START: f32 = 0.0;
pub const FLOOR_GRAZING_FADE_END: f32 = 0.13;

pub const CONTACT_SHADOW_STRENGTH: f32 = 0.82;
pub const AMBIENT_OCCLUSION_STRENGTH: f32 = 0.85;
pub const EXPOSURE: f32 = 1.0;

pub const SHADOW_FIT_MARGIN: f32 = 1.45;
pub const SHADOW_DEPTH_MARGIN: f32 = 2.0;
pub const SHADOW_NORMAL_OFFSET_TEXELS: f32 = 1.6;
pub const SHADOW_SLOPE_OFFSET: f32 = 0.35;

pub const BLOOM_THRESHOLD: f32 = 1.05;
pub const BLOOM_KNEE: f32 = 0.55;
pub const BLOOM_INTENSITY: f32 = 0.16;
pub const BLOOM_BLUR_RADIUS: f32 = 1.35;

pub const VIGNETTE_STRENGTH: f32 = 0.34;
pub const VIGNETTE_RADIUS: f32 = 0.78;

pub const LINE_HDR_GAIN: f32 = 1.6;

pub const LAYER_HEIGHT: f32 = 0.2;
pub const LAYER_RELIEF: f32 = 0.035;
pub const LAYER_FACING_FADE: f32 = 0.10;
pub const LAYER_SELF_SHADOW: f32 = 0.55;
pub const LAYER_SPECULAR_SPREAD: f32 = 0.30;

pub const GI_BOUNCE_STRENGTH: f32 = 0.65;

pub const DOF_MAX_RADIUS: f32 = 14.0;
pub const DOF_APERTURE: f32 = 5.5;

pub fn key_direction() -> Vec3 {
    Vec3::new(-0.35, -0.55, 0.76).normalize()
}

pub fn fill_direction() -> Vec3 {
    Vec3::new(0.62, 0.45, 0.20).normalize()
}

pub fn floor_height(min: Vec3) -> f32 {
    min.z
}

pub fn scene_radius(min: Vec3, max: Vec3) -> f32 {
    ((max - min).length() * 0.5).max(1.0)
}

pub fn floor_radius(min: Vec3, max: Vec3) -> f32 {
    scene_radius(min, max) * FLOOR_RADIUS_MULTIPLE
}

pub fn floor_centre(min: Vec3, max: Vec3) -> Vec3 {
    Vec3::new((min.x + max.x) * 0.5, (min.y + max.y) * 0.5, floor_height(min))
}

pub fn floor_presence(pitch: f32) -> f32 {
    let t = (pitch - FLOOR_GRAZING_FADE_START)
        / (FLOOR_GRAZING_FADE_END - FLOOR_GRAZING_FADE_START);
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

pub fn mirror_about_height(z: f32) -> Mat4 {
    Mat4::from_cols_array_2d(&[
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 0.0, 2.0 * z, 1.0],
    ])
}

pub fn shadow_radius(min: Vec3, max: Vec3) -> f32 {
    scene_radius(min, max) * SHADOW_FIT_MARGIN
}

pub fn shadow_world_texel(min: Vec3, max: Vec3, resolution: i32) -> f32 {
    shadow_radius(min, max) * 2.0 / resolution.max(1) as f32
}

pub fn shadow_view_proj(min: Vec3, max: Vec3) -> Mat4 {
    let centre = (min + max) * 0.5;
    let radius = shadow_radius(min, max);
    let distance = radius * SHADOW_DEPTH_MARGIN;
    let eye = centre + key_direction() * distance;
    let up = if key_direction().z.abs() > 0.99 { Vec3::Y } else { Vec3::Z };
    let proj =
        Mat4::orthographic_rh(-radius, radius, -radius, radius, 0.01, distance + radius * 2.0);
    proj * Mat4::look_at_rh(eye, centre, up)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_floor_sits_at_the_lowest_point_of_the_scene() {
        let min = Vec3::new(-10.0, -20.0, 3.5);
        assert_eq!(floor_height(min), 3.5);
    }

    #[test]
    fn the_floor_is_centred_under_the_scene_footprint() {
        let centre = floor_centre(Vec3::new(-10.0, 0.0, 2.0), Vec3::new(30.0, 10.0, 8.0));
        assert_eq!(centre, Vec3::new(10.0, 5.0, 2.0));
    }

    #[test]
    fn the_floor_is_gone_at_eye_level_and_whole_once_the_camera_lifts() {
        assert_eq!(floor_presence(0.0), 0.0);
        assert_eq!(floor_presence(-1.0), 0.0);
        assert_eq!(floor_presence(FLOOR_GRAZING_FADE_START), 0.0);
        assert_eq!(floor_presence(FLOOR_GRAZING_FADE_END), 1.0);
        assert_eq!(floor_presence(1.5), 1.0);
    }

    #[test]
    fn the_floor_fade_rises_monotonically_across_the_transition() {
        let mut previous = -1.0;
        for step in 0..=40 {
            let pitch = step as f32 * 0.02;
            let weight = floor_presence(pitch);
            assert!(weight >= previous, "fade must not dip at pitch {pitch}");
            previous = weight;
        }
    }

    #[test]
    fn the_fade_is_confined_to_the_camera_almost_touching_the_floor() {
        assert!(
            FLOOR_GRAZING_FADE_END < 0.2,
            "the floor must stay whole across the oblique angles where Fresnel makes \
             the reflection strongest, or the fade hides the effect it exists to show",
        );
        assert_eq!(floor_presence(0.25), 1.0);
    }

    #[test]
    fn mirroring_reflects_positions_through_the_floor_plane() {
        let mirror = mirror_about_height(5.0);
        let above = mirror * Vec3::new(1.0, 2.0, 8.0).extend(1.0);
        assert_eq!(above.truncate(), Vec3::new(1.0, 2.0, 2.0));
        let on_plane = mirror * Vec3::new(1.0, 2.0, 5.0).extend(1.0);
        assert_eq!(on_plane.truncate(), Vec3::new(1.0, 2.0, 5.0));
    }

    #[test]
    fn the_shadow_projection_covers_every_corner_of_the_scene_box() {
        let (min, max) = (Vec3::new(-40.0, -20.0, 0.0), Vec3::new(40.0, 20.0, 30.0));
        let view_proj = shadow_view_proj(min, max);
        for corner in [
            Vec3::new(min.x, min.y, min.z),
            Vec3::new(max.x, min.y, min.z),
            Vec3::new(min.x, max.y, min.z),
            Vec3::new(max.x, max.y, max.z),
            Vec3::new(min.x, max.y, max.z),
        ] {
            let clip = view_proj * corner.extend(1.0);
            let ndc = clip.truncate() / clip.w;
            assert!(
                ndc.x.abs() <= 1.0 && ndc.y.abs() <= 1.0 && (0.0..=1.0).contains(&ndc.z),
                "corner {corner:?} fell outside the shadow frustum at {ndc:?}",
            );
        }
    }

    #[test]
    fn a_shadow_texel_is_measured_in_millimetres_of_scene_not_texture_coordinates() {
        let (min, max) = (Vec3::new(-40.0, -20.0, 0.0), Vec3::new(40.0, 20.0, 30.0));
        let texel = shadow_world_texel(min, max, 1024);
        assert_eq!(texel, shadow_radius(min, max) * 2.0 / 1024.0);
        assert!(texel > 0.05, "a texel of a 90 mm scene must be a fraction of a millimetre, not {texel}");
        assert!(shadow_world_texel(min, max, 2048) < texel);
    }

    #[test]
    fn a_shadow_map_with_no_resolution_still_reports_a_finite_texel() {
        assert!(shadow_world_texel(Vec3::ZERO, Vec3::ONE, 0).is_finite());
    }

    #[test]
    fn the_shadow_frustum_is_fitted_to_the_radius_the_texel_size_is_derived_from() {
        let (min, max) = (Vec3::new(-5.0, -5.0, 0.0), Vec3::new(5.0, 5.0, 10.0));
        let radius = shadow_radius(min, max);
        let centre = (min + max) * 0.5;
        let right = (-key_direction()).cross(Vec3::Z).normalize();
        let view_proj = shadow_view_proj(min, max);
        let edge = view_proj * (centre + right * radius).extend(1.0);
        assert!((edge.x / edge.w).abs() - 1.0 < 1e-3);
        let inside = view_proj * (centre + right * radius * 0.5).extend(1.0);
        assert!((inside.x / inside.w).abs() < 0.51);
    }

    #[test]
    fn a_degenerate_scene_still_has_a_usable_radius() {
        assert!(scene_radius(Vec3::ZERO, Vec3::ZERO) > 0.0);
        assert!(floor_radius(Vec3::ZERO, Vec3::ZERO) > 0.0);
    }
}
