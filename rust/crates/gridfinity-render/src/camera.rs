use glam::{Mat4, Vec3};

pub const FOV_Y_DEGREES: f32 = 45.0;
pub const DEFAULT_YAW: f32 = 0.9;
pub const DEFAULT_PITCH: f32 = 0.5;
pub const DEFAULT_DISTANCE: f32 = 200.0;

const MIN_PITCH: f32 = -1.54;
const MAX_PITCH: f32 = 1.54;
const MIN_DISTANCE: f32 = 10.0;
const MAX_DISTANCE: f32 = 40000.0;
const FIT_MARGIN: f32 = 1.1;
const ORBIT_PER_PIXEL: f32 = 0.01;
const ZOOM_PER_UNIT: f32 = 0.001;
const NEAR_FRACTION_OF_DISTANCE: f32 = 0.002;
const MIN_NEAR: f32 = 0.1;
const FAR_MULTIPLE_OF_DISTANCE: f32 = 8.0;

#[derive(Clone, Copy)]
pub struct Camera {
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
    pub target: Vec3,
}

impl Default for Camera {
    fn default() -> Camera {
        Camera {
            yaw: DEFAULT_YAW,
            pitch: DEFAULT_PITCH,
            distance: DEFAULT_DISTANCE,
            target: Vec3::ZERO,
        }
    }
}

impl Camera {
    pub fn frame(&mut self, min: Vec3, max: Vec3) {
        self.target = (min + max) * 0.5;
        let radius = (max - min).length() * 0.5 * FIT_MARGIN;
        self.distance = (radius / (FOV_Y_DEGREES * 0.5).to_radians().tan())
            .clamp(MIN_DISTANCE, MAX_DISTANCE);
    }

    pub fn reset_angles(&mut self) {
        self.yaw = DEFAULT_YAW;
        self.pitch = DEFAULT_PITCH;
    }

    pub fn look_down(&mut self) {
        self.yaw = -std::f32::consts::FRAC_PI_2;
        self.pitch = MAX_PITCH;
    }

    pub fn eye(&self) -> Vec3 {
        let (sp, cp) = self.pitch.sin_cos();
        let (sy, cy) = self.yaw.sin_cos();
        self.target + self.distance * Vec3::new(cp * cy, cp * sy, sp)
    }

    pub fn near_far(&self) -> (f32, f32) {
        (
            (self.distance * NEAR_FRACTION_OF_DISTANCE).max(MIN_NEAR),
            self.distance * FAR_MULTIPLE_OF_DISTANCE,
        )
    }

    pub fn view_proj(&self, aspect: f32) -> Mat4 {
        let (near, far) = self.near_far();
        let proj =
            Mat4::perspective_rh_gl(FOV_Y_DEGREES.to_radians(), aspect.max(0.01), near, far);
        proj * Mat4::look_at_rh(self.eye(), self.target, Vec3::Z)
    }

    pub fn project_to_viewport(&self, p: Vec3, width: f32, height: f32) -> Option<(f32, f32)> {
        let clip = self.view_proj(width / height.max(1.0)) * p.extend(1.0);
        if clip.w <= 1e-6 {
            return None;
        }
        let ndc = clip.truncate() / clip.w;
        Some(((ndc.x * 0.5 + 0.5) * width, (0.5 - ndc.y * 0.5) * height))
    }

    pub fn orbit(&mut self, dx: f32, dy: f32) {
        self.yaw -= dx * ORBIT_PER_PIXEL;
        self.pitch = (self.pitch + dy * ORBIT_PER_PIXEL).clamp(MIN_PITCH, MAX_PITCH);
    }

    pub fn zoom(&mut self, delta: f32) {
        if delta != 0.0 {
            self.distance =
                (self.distance * (1.0 - delta * ZOOM_PER_UNIT)).clamp(MIN_DISTANCE, MAX_DISTANCE);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn framing_centres_the_box_and_backs_off_past_its_bounding_sphere() {
        let mut camera = Camera::default();
        let (min, max) = (Vec3::new(-10.0, -10.0, 0.0), Vec3::new(10.0, 10.0, 20.0));
        camera.frame(min, max);
        assert_eq!(camera.target, Vec3::new(0.0, 0.0, 10.0));
        assert!(camera.distance > (max - min).length() * 0.5);
    }

    #[test]
    fn framing_a_degenerate_box_still_yields_a_usable_distance() {
        let mut camera = Camera::default();
        camera.frame(Vec3::ZERO, Vec3::ZERO);
        assert_eq!(camera.distance, MIN_DISTANCE);
    }

    #[test]
    fn pitch_is_clamped_short_of_the_poles_so_the_up_vector_stays_valid() {
        let mut camera = Camera::default();
        camera.orbit(0.0, 100_000.0);
        assert_eq!(camera.pitch, MAX_PITCH);
        camera.orbit(0.0, -200_000.0);
        assert_eq!(camera.pitch, MIN_PITCH);
    }

    #[test]
    fn zoom_is_clamped_at_both_ends() {
        let mut camera = Camera::default();
        camera.zoom(100_000.0);
        assert_eq!(camera.distance, MIN_DISTANCE);
        for _ in 0..10 {
            camera.zoom(-100_000.0);
        }
        assert_eq!(camera.distance, MAX_DISTANCE);
    }

    #[test]
    fn the_near_plane_stays_ahead_of_the_eye_and_behind_the_far_plane() {
        let mut camera = Camera::default();
        for distance in [MIN_DISTANCE, 200.0, MAX_DISTANCE] {
            camera.distance = distance;
            let (near, far) = camera.near_far();
            assert!(near >= MIN_NEAR && near < far, "near {near} far {far}");
        }
    }

    #[test]
    fn the_eye_sits_at_the_orbit_distance_from_the_target() {
        let camera = Camera::default();
        let offset = camera.eye() - camera.target;
        assert!((offset.length() - camera.distance).abs() < 1e-3);
    }

    #[test]
    fn looking_down_puts_the_eye_above_the_target() {
        let mut camera = Camera::default();
        camera.target = Vec3::new(1.0, 2.0, 3.0);
        camera.look_down();
        let offset = camera.eye() - camera.target;
        assert!(offset.z > offset.x.abs() && offset.z > offset.y.abs());
    }

    #[test]
    fn the_target_projects_to_the_centre_of_the_viewport() {
        let camera = Camera::default();
        let (x, y) = camera.project_to_viewport(camera.target, 800.0, 600.0).expect("target is in front");
        assert!((x - 400.0).abs() < 1e-2 && (y - 300.0).abs() < 1e-2);
    }

    #[test]
    fn a_point_behind_the_eye_does_not_project() {
        let camera = Camera::default();
        let behind = camera.target + (camera.eye() - camera.target) * 2.0;
        assert!(camera.project_to_viewport(behind, 800.0, 600.0).is_none());
    }
}
