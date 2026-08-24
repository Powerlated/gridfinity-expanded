#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Quality {
    Low,
    Medium,
    High,
}

impl Default for Quality {
    fn default() -> Quality {
        Quality::High
    }
}

impl Quality {
    pub fn from_index(index: u32) -> Quality {
        let level = match index {
            0 => Quality::Low,
            1 => Quality::Medium,
            _ => Quality::High,
        };
        assert!(
            index > Quality::High.index() || level.index() == index,
            "Quality index {index} does not round-trip"
        );
        level
    }

    pub fn index(self) -> u32 {
        match self {
            Quality::Low => 0,
            Quality::Medium => 1,
            Quality::High => 2,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Quality::Low => "low",
            Quality::Medium => "medium",
            Quality::High => "high",
        }
    }

    pub fn ambient_occlusion(self) -> bool {
        self != Quality::Low
    }

    pub fn reflection(self) -> bool {
        self != Quality::Low
    }

    pub fn shadow(self) -> bool {
        self != Quality::Low
    }

    pub fn bloom(self) -> bool {
        self != Quality::Low
    }

    pub fn antialias(self) -> bool {
        self != Quality::Low
    }

    pub fn layer_lines(self) -> bool {
        self != Quality::Low
    }

    pub fn depth_of_field(self) -> bool {
        self == Quality::High
    }

    pub fn accumulation_samples(self) -> u32 {
        match self {
            Quality::Low => 1,
            Quality::Medium => 16,
            Quality::High => 48,
        }
    }

    pub fn shadow_resolution(self) -> i32 {
        match self {
            Quality::Low => 0,
            Quality::Medium => 1024,
            Quality::High => 2048,
        }
    }

    pub fn shadow_taps(self) -> i32 {
        match self {
            Quality::Low => 0,
            Quality::Medium => 1,
            Quality::High => 2,
        }
    }

    pub fn occlusion_samples(self) -> i32 {
        match self {
            Quality::Low => 0,
            Quality::Medium => 8,
            Quality::High => 16,
        }
    }

    pub fn occlusion_divisor(self) -> i32 {
        1
    }

    pub fn bloom_bright_divisor(self) -> i32 {
        2
    }

    pub fn bloom_blur_divisor(self) -> i32 {
        4
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_renderer_starts_at_the_richest_level() {
        assert_eq!(Quality::default(), Quality::High);
    }

    #[test]
    fn the_cheapest_level_skips_every_optional_pass() {
        assert!(!Quality::Low.ambient_occlusion());
        assert!(!Quality::Low.reflection());
        assert!(!Quality::Low.shadow());
        assert!(!Quality::Low.bloom());
        assert!(!Quality::Low.antialias());
        assert_eq!(Quality::Low.occlusion_samples(), 0);
        assert_eq!(Quality::Low.shadow_resolution(), 0);
    }

    #[test]
    fn every_level_above_the_cheapest_runs_the_full_pass_list() {
        for level in [Quality::Medium, Quality::High] {
            assert!(level.ambient_occlusion());
            assert!(level.reflection());
            assert!(level.shadow());
            assert!(level.bloom());
            assert!(level.antialias());
            assert!(level.shadow_resolution() > 0);
            assert!(level.shadow_taps() > 0);
            assert!(level.occlusion_samples() > 0);
        }
    }

    #[test]
    fn the_richest_level_never_asks_for_less_than_the_middle_one() {
        assert!(Quality::High.shadow_resolution() >= Quality::Medium.shadow_resolution());
        assert!(Quality::High.shadow_taps() >= Quality::Medium.shadow_taps());
        assert!(Quality::High.occlusion_samples() >= Quality::Medium.occlusion_samples());
    }

    #[test]
    fn every_divisor_is_a_usable_denominator() {
        for level in [Quality::Low, Quality::Medium, Quality::High] {
            assert!(level.occlusion_divisor() > 0);
            assert!(level.bloom_bright_divisor() > 0);
            assert!(level.bloom_blur_divisor() > 0);
        }
    }

    #[test]
    fn each_level_names_itself_distinctly() {
        let names: Vec<&str> =
            [Quality::Low, Quality::Medium, Quality::High].map(Quality::name).to_vec();
        assert_eq!(names, ["low", "medium", "high"]);
    }
}
