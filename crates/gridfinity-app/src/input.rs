//! The TOML an `optimize` run is described by, and the validated configuration
//! it resolves to.
//!
//! `InputFile` and the types under it are the file's own shape, deserialised
//! verbatim and rejecting unknown keys so a misspelling is an error rather than
//! a silently ignored setting. `Spec` is what the rest of the run works from:
//! every optional setting filled in, the printer resolved to a real profile, and
//! every object turned into an edge-connected part list in millimetres with the
//! quantity wanted. `parse` is the whole transformation, and every failure it
//! returns names the object and the key at fault.
//!
//! Every measurement in the file is a `Length`: a bare number of millimetres, or
//! a string carrying a unit, so a drawer measured with an imperial tape is
//! written as it was measured rather than converted by hand.

use gridfinity_cad::gridfinity::{
    BASE_TOTAL_HEIGHT, FLOOR_THICKNESS, GRID_PITCH, HEIGHT_PER_UNIT, MIN_FASTENER_GRID_PITCH,
    MIN_GRID_PITCH, buildable_floor_fillet,
};
use gridfinity_cad::printers::{DEFAULT_PRINTER, PRINTER_PROFILES, PrinterProfile};
use gridfinity_cad::project::pack::{PackEffort, PackObject};
use gridfinity_cad::project::rects::{Rect, parts_connected};

/// One of the units a measurement may name, as the millimetres it is worth. The
/// empty unit is millimetres, so `"400"` and `400` are the same length.
fn unit_in_mm(unit: &str) -> Option<f64> {
    match unit {
        "" | "mm" => Some(1.0),
        "cm" => Some(10.0),
        "m" => Some(1000.0),
        "in" | "inch" | "inches" | "\"" => Some(25.4),
        "ft" | "foot" | "feet" | "'" => Some(304.8),
        _ => None,
    }
}

/// A measurement written as text as the millimetres it is, or the reason it is
/// not a measurement. The number is the leading run of decimal characters and
/// the unit is the rest of the trimmed text, matched without case, so `"2.1 in"`
/// and `"2.1IN"` are both 53.34.
fn text_to_mm(text: &str) -> Result<f64, String> {
    let trimmed = text.trim();
    let end = trimmed
        .find(|c: char| !matches!(c, '0'..='9' | '.' | '+' | '-'))
        .unwrap_or(trimmed.len());
    let (number, unit) = trimmed.split_at(end);
    let value: f64 = number.parse().map_err(|_| {
        format!("{text:?} does not begin with a number, so it is not a measurement")
    })?;
    let unit = unit.trim();
    let scale = unit_in_mm(&unit.to_ascii_lowercase()).ok_or_else(|| {
        format!("{text:?} is measured in {unit:?}, which is not one of mm, cm, m, in, ft")
    })?;
    Ok(value * scale)
}

/// A length as the file states it, held in millimetres: a bare number is already
/// millimetres, and a string is its number scaled by the unit it names.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Length(f64);

impl Length {
    /// The length in millimetres, which is the only form it leaves this module
    /// in.
    fn mm(self) -> f64 {
        self.0
    }
}

impl<'de> serde::Deserialize<'de> for Length {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        #[derive(serde::Deserialize)]
        #[serde(untagged)]
        enum Stated {
            Millimetres(f64),
            Measured(String),
        }
        match Stated::deserialize(deserializer)? {
            Stated::Millimetres(mm) => Ok(Length(mm)),
            Stated::Measured(text) => {
                text_to_mm(&text).map(Length).map_err(serde::de::Error::custom)
            }
        }
    }
}

/// The drawer's inside measurements, in millimetres.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct DrawerSpec {
    width: Length,
    depth: Length,
}

/// A printer bed stated directly rather than by profile name, in millimetres.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct BedSpec {
    width: Length,
    depth: Length,
}

/// Every setting the file may state, all optional.
#[derive(Default, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct SettingsSpec {
    divider_thickness: Option<Length>,
    clearance: Option<Length>,
    grid_size: Option<Length>,
    effort: Option<String>,
    height_units: Option<u32>,
    wall_thickness: Option<Length>,
    fillet_radius: Option<Length>,
    tidy_absorb: Option<Length>,
    magnets: Option<bool>,
    screws: Option<bool>,
    printer: Option<String>,
    bed: Option<BedSpec>,
    baseplate: Option<bool>,
}

/// One box of an object, in the object's own millimetre frame.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct BoxSpec {
    #[serde(default)]
    x: Length,
    #[serde(default)]
    y: Length,
    width: Length,
    depth: Length,
    height: Option<Length>,
}

/// One thing to organise: a name, how many of it, and its footprint stated
/// either as a single `size` or as an edge-connected list of `boxes`.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct ObjectSpec {
    name: String,
    quantity: Option<u32>,
    size: Option<Vec<Length>>,
    boxes: Option<Vec<BoxSpec>>,
}

/// The file as written.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct InputFile {
    drawer: DrawerSpec,
    #[serde(default)]
    settings: SettingsSpec,
    #[serde(default)]
    objects: Vec<ObjectSpec>,
}

/// One validated object: what the packer needs, plus the tallest height the file
/// declared for it, which drives no geometry but is reported against the cavity.
#[derive(Debug)]
pub struct Object {
    pub pack: PackObject,
    pub height: Option<f64>,
}

/// A validated run: the drawer, the bin's parameters, the printer to fit, and
/// the objects to place.
#[derive(Debug)]
pub struct Spec {
    pub drawer_width: f64,
    pub drawer_depth: f64,
    /// The millimetres one grid cell spans. `GRID_PITCH` unless the file says
    /// otherwise, and every cell measurement of the run -- how many fit the
    /// drawer, the packing area, the bin, the baseplate and what fits the bed --
    /// is taken in it.
    pub pitch: f64,
    pub divider_thickness: f64,
    pub clearance: f64,
    pub wall_thickness: f64,
    pub fillet_radius: f64,
    /// The widest strip of leftover space worth absorbing into the compartments
    /// facing it, once a layout is packed. Half the run's own cell pitch unless
    /// the file says otherwise -- leftover that could not hold half a cell's
    /// worth of anything is not worth the material it costs -- and zero turns the
    /// growth off, leaving the pass only evening the slack out.
    pub tidy_absorb: f64,
    pub height_units: u32,
    pub magnets: bool,
    pub screws: bool,
    /// Whether to build the baseplate the fitted bin drops into, alongside it.
    /// On unless the file says otherwise: a bin carries a connector peg under
    /// every cell and has nothing to sit in without one.
    pub baseplate: bool,
    pub effort: PackEffort,
    pub printer: PrinterProfile,
    pub objects: Vec<Object>,
}

impl Spec {
    /// The bin's overall height in millimetres, base included.
    pub fn total_height(&self) -> f64 {
        f64::from(BASE_TOTAL_HEIGHT)
            + f64::from(HEIGHT_PER_UNIT) * f64::from(self.height_units.max(1))
    }

    /// How deep a compartment is: everything above the base and its floor.
    pub fn cavity_depth(&self) -> f64 {
        self.total_height() - f64::from(BASE_TOTAL_HEIGHT) - f64::from(FLOOR_THICKNESS)
    }

    /// The floor fillet the model will actually blend this bin's compartments
    /// with: the requested radius after the clamps the cavity's depth and corner
    /// impose on it.
    ///
    /// A drawer bin is never sloped and takes `fillet_radius` as its corner
    /// radius too, so this is `buildable_floor_fillet` asked exactly as
    /// `plan_cavities` will ask it. It is the fitter's number as much as the
    /// model's: the blend takes this much floor away from every compartment
    /// wall, so it is what an object standing on that floor has to be held clear
    /// of before its own clearance.
    pub fn built_floor_fillet(&self) -> f64 {
        buildable_floor_fillet(
            self.fillet_radius,
            self.cavity_depth(),
            self.fillet_radius.max(0.0),
            false,
        )
    }

    /// The objects the packer is asked to place.
    pub fn pack_objects(&self) -> Vec<PackObject> {
        self.objects.iter().map(|o| o.pack.clone()).collect()
    }
}

/// A measurement that must be positive, or an error naming the object and key it
/// came from.
fn positive(value: f64, what: &str, whose: &str) -> Result<f64, String> {
    if value > 0.0 {
        return Ok(value);
    }
    Err(format!("{whose}: {what} must be greater than zero, but is {value}"))
}

/// The `size` array as one box plus the height it declares, accepting `[width,
/// depth]` or `[width, depth, height]`.
fn size_to_parts(size: &[Length], whose: &str) -> Result<(Vec<Rect>, Option<f64>), String> {
    let (width, depth, height) = match size {
        [w, d] => (w.mm(), d.mm(), None),
        [w, d, h] => (w.mm(), d.mm(), Some(h.mm())),
        other => {
            return Err(format!(
                "{whose}: size is [width, depth] or [width, depth, height], but has {} entries",
                other.len()
            ));
        }
    };
    positive(width, "size width", whose)?;
    positive(depth, "size depth", whose)?;
    if let Some(h) = height {
        positive(h, "size height", whose)?;
    }
    Ok((vec![Rect::new(0.0, 0.0, width, depth)], height))
}

/// The `boxes` list as a part list plus the tallest height any of them declares.
fn boxes_to_parts(boxes: &[BoxSpec], whose: &str) -> Result<(Vec<Rect>, Option<f64>), String> {
    if boxes.is_empty() {
        return Err(format!("{whose}: boxes is empty, so the object has no footprint"));
    }
    let mut parts = Vec::with_capacity(boxes.len());
    let mut height: Option<f64> = None;
    for b in boxes {
        positive(b.width.mm(), "box width", whose)?;
        positive(b.depth.mm(), "box depth", whose)?;
        if let Some(h) = b.height.map(Length::mm) {
            positive(h, "box height", whose)?;
            height = Some(height.map_or(h, |t: f64| t.max(h)));
        }
        parts.push(Rect::new(b.x.mm(), b.y.mm(), b.width.mm(), b.depth.mm()));
    }
    Ok((parts, height))
}

/// The printer the settings name: a profile by name, a bed stated directly, or
/// the default. Naming both is an error, because the two would disagree.
fn resolve_printer(settings: &SettingsSpec) -> Result<PrinterProfile, String> {
    match (&settings.printer, &settings.bed) {
        (Some(_), Some(_)) => Err(
            "settings names both a printer profile and a bed; state one or the other".to_string(),
        ),
        (Some(name), None) => PrinterProfile::find(name).ok_or_else(|| {
            let names: Vec<&str> = PRINTER_PROFILES.iter().map(|p| p.name).collect();
            format!(
                "settings.printer is {name:?}, which is not a profile. The profiles are: {}",
                names.join(", ")
            )
        }),
        (None, Some(bed)) => {
            let (width, depth) = (bed.width.mm().round(), bed.depth.mm().round());
            if width <= 0.0 || depth <= 0.0 {
                return Err(format!(
                    "settings.bed is {} x {} mm, which is not a bed",
                    bed.width.mm(),
                    bed.depth.mm()
                ));
            }
            Ok(PrinterProfile {
                name: "Custom",
                bed_width: width as i32,
                bed_depth: depth as i32,
            })
        }
        (None, None) => Ok(DEFAULT_PRINTER),
    }
}

/// The TOML text as a validated run, or the first reason it is not one.
pub fn parse(text: &str) -> Result<Spec, String> {
    let file: InputFile = toml::from_str(text).map_err(|e| e.to_string())?;
    positive(file.drawer.width.mm(), "drawer.width", "drawer")?;
    positive(file.drawer.depth.mm(), "drawer.depth", "drawer")?;

    let settings = &file.settings;
    let effort_name = settings.effort.as_deref().unwrap_or("standard");
    let effort = PackEffort::from_name(effort_name).ok_or_else(|| {
        format!("settings.effort is {effort_name:?}, not one of quick, standard, thorough")
    })?;
    let divider_thickness = positive(
        settings.divider_thickness.map_or(1.2, Length::mm),
        "settings.divider_thickness",
        "settings",
    )?;
    let wall_thickness = positive(
        settings.wall_thickness.map_or(1.2, Length::mm),
        "settings.wall_thickness",
        "settings",
    )?;
    let clearance = settings.clearance.map_or(0.5, Length::mm);
    if clearance < 0.0 {
        return Err(format!("settings.clearance is {clearance}, which is less than none"));
    }
    let fillet_radius = settings.fillet_radius.map_or(2.5, Length::mm);
    if fillet_radius < 0.0 {
        return Err(format!(
            "settings.fillet_radius is {fillet_radius}, which is less than none"
        ));
    }
    let pitch = settings.grid_size.map_or(GRID_PITCH, Length::mm);
    if pitch < MIN_GRID_PITCH {
        return Err(format!(
            "settings.grid_size is {pitch} mm, and a Gridfinity cell cannot be built below              {MIN_GRID_PITCH} mm -- its peg profile does not close"
        ));
    }
    let magnets = settings.magnets.unwrap_or(false);
    let screws = settings.screws.unwrap_or(false);
    if (magnets || screws) && pitch <= MIN_FASTENER_GRID_PITCH {
        return Err(format!(
            "settings.grid_size is {pitch} mm, and a cell carries magnets or screws only above              {MIN_FASTENER_GRID_PITCH} mm -- the four bores of a smaller cell run into one another"
        ));
    }
    let height_units = settings.height_units.unwrap_or(3);
    if height_units == 0 {
        return Err("settings.height_units is 0, so the bin has no cavity".to_string());
    }

    let mut objects: Vec<Object> = Vec::with_capacity(file.objects.len());
    for spec in &file.objects {
        let whose = format!("object {:?}", spec.name);
        if objects.iter().any(|o| o.pack.name == spec.name) {
            return Err(format!(
                "{whose} is declared twice; every object needs its own name"
            ));
        }
        let quantity = spec.quantity.unwrap_or(1);
        if quantity == 0 {
            return Err(format!("{whose}: quantity is 0, so none of it is wanted"));
        }
        let (parts, height) = match (&spec.size, &spec.boxes) {
            (Some(_), Some(_)) => {
                return Err(format!("{whose}: states both size and boxes; state one or the other"));
            }
            (Some(size), None) => size_to_parts(size, &whose)?,
            (None, Some(boxes)) => boxes_to_parts(boxes, &whose)?,
            (None, None) => {
                return Err(format!("{whose}: has neither size nor boxes, so it has no footprint"));
            }
        };
        if !parts_connected(&parts) {
            return Err(format!(
                "{whose}: its boxes do not touch along an edge, so it is more than one object"
            ));
        }
        objects.push(Object {
            pack: PackObject {
                id: spec.name.clone(),
                name: spec.name.clone(),
                parts,
                quantity,
            },
            height,
        });
    }

    let tidy_absorb = settings.tidy_absorb.map_or(pitch / 2.0, Length::mm);
    if tidy_absorb < 0.0 {
        return Err(format!(
            "settings.tidy_absorb is {tidy_absorb}, which is less than none"
        ));
    }

    Ok(Spec {
        drawer_width: file.drawer.width.mm(),
        drawer_depth: file.drawer.depth.mm(),
        pitch,
        divider_thickness,
        clearance,
        wall_thickness,
        fillet_radius,
        tidy_absorb,
        height_units,
        magnets,
        screws,
        baseplate: settings.baseplate.unwrap_or(true),
        effort,
        printer: resolve_printer(settings)?,
        objects,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const MINIMAL: &str = "[drawer]\nwidth = 400\ndepth = 300\n";

    #[test]
    fn fills_every_setting_the_file_leaves_out() {
        let spec = parse(MINIMAL).expect("a drawer alone is a valid run");
        assert_eq!(spec.effort, PackEffort::Standard);
        assert_eq!(spec.height_units, 3);
        assert_eq!(spec.printer.name, DEFAULT_PRINTER.name);
        assert!(spec.baseplate, "a drawer bin gets the grid it sits in unless asked not to");
        assert!(spec.objects.is_empty());
    }

    #[test]
    fn measures_the_cavity_from_the_height_units() {
        let spec = parse(MINIMAL).expect("a drawer alone is a valid run");
        assert_eq!(spec.total_height(), 28.0);
        assert_eq!(
            spec.cavity_depth(),
            28.0 - f64::from(BASE_TOTAL_HEIGHT) - f64::from(FLOOR_THICKNESS)
        );
        assert!(
            (spec.cavity_depth() - 19.8).abs() < 1e-6,
            "a three-unit bin is 19.8 mm deep inside, not {}",
            spec.cavity_depth()
        );
    }

    #[test]
    fn takes_the_baseplate_away_when_the_file_says_so() {
        let spec = parse(&format!("{MINIMAL}
[settings]
baseplate = false
"))
            .expect("baseplate is a setting");
        assert!(!spec.baseplate);
    }

    /// The widest strip of leftover worth absorbing is half the run's own cell
    /// pitch unless the file names one, so a file on a finer grid absorbs
    /// proportionally less without having to say so.
    #[test]
    fn takes_the_widest_strip_worth_absorbing_from_the_pitch() {
        let spec = parse(MINIMAL).expect("the minimal file is a valid run");
        assert!((spec.tidy_absorb - spec.pitch / 2.0).abs() < 1e-9);
        let stated = parse(&format!("{MINIMAL}
[settings]
tidy_absorb = \"4 mm\"
"))
        .expect("tidy_absorb is a setting");
        assert!((stated.tidy_absorb - 4.0).abs() < 1e-9);
        let refused = parse(&format!("{MINIMAL}
[settings]
tidy_absorb = -1
"))
        .expect_err("a strip of negative width is not a strip");
        assert!(refused.contains("tidy_absorb"), "{refused}");
    }

    #[test]
    fn rejects_an_object_whose_boxes_do_not_touch() {
        let text = format!(
            "{MINIMAL}\n[[objects]]\nname = \"split\"\nboxes = [\
             {{ x = 0, y = 0, width = 10, depth = 10 }}, \
             {{ x = 40, y = 0, width = 10, depth = 10 }}]\n"
        );
        let err = parse(&text).expect_err("disconnected boxes are not one object");
        assert!(err.contains("do not touch"), "{err}");
    }

    #[test]
    fn rejects_a_misspelled_key_rather_than_ignoring_it() {
        let err = parse(&format!("{MINIMAL}\n[settings]\nclearence = 0.5\n"))
            .expect_err("an unknown key is a mistake, not a setting");
        assert!(err.contains("clearence"), "{err}");
    }

    #[test]
    fn rejects_a_printer_that_is_not_a_profile() {
        let err = parse(&format!("{MINIMAL}\n[settings]\nprinter = \"Nonesuch\"\n"))
            .expect_err("an unknown printer is a mistake");
        assert!(err.contains("Nonesuch"), "{err}");
    }

    #[test]
    fn reads_a_size_with_or_without_a_height() {
        let text = format!(
            "{MINIMAL}\n[[objects]]\nname = \"flat\"\nsize = [40, 30]\n\
             [[objects]]\nname = \"tall\"\nquantity = 2\nsize = [40, 30, 55]\n"
        );
        let spec = parse(&text).expect("both spellings of size are valid");
        assert_eq!(spec.objects[0].height, None);
        assert_eq!(spec.objects[1].height, Some(55.0));
        assert_eq!(spec.objects[1].pack.quantity, 2);
    }

    #[test]
    fn reads_a_measurement_in_every_unit_it_names() {
        for (text, mm) in [
            ("400", 400.0),
            ("400 mm", 400.0),
            ("40cm", 400.0),
            ("0.4 m", 400.0),
            ("2.1 in", 53.34),
            ("2.1IN", 53.34),
            ("1 ft", 304.8),
        ] {
            let got = text_to_mm(text).unwrap_or_else(|e| panic!("{text:?} is a measurement: {e}"));
            assert!((got - mm).abs() < 1e-9, "{text:?} is {mm} mm, not {got}");
        }
    }

    #[test]
    fn takes_a_size_stated_in_inches() {
        let text = format!(
            "{MINIMAL}\n[[objects]]\nname = \"level\"\nsize = [\"2.1 in\", \"9.2 in\", \"1 in\"]\n"
        );
        let spec = parse(&text).expect("a size may be measured in inches");
        let bounds = &spec.objects[0].pack.parts[0];
        assert!(
            (bounds.width - 53.34).abs() < 1e-9 && (bounds.depth - 233.68).abs() < 1e-9,
            "2.1 in x 9.2 in is 53.34 x 233.68 mm, not {} x {}",
            bounds.width,
            bounds.depth
        );
        assert_eq!(spec.objects[0].height, Some(25.4));
    }

    #[test]
    fn measures_the_drawer_and_the_settings_in_the_units_they_name() {
        let spec = parse(
            "[drawer]\nwidth = \"11.5 in\"\ndepth = \"20.6 in\"\n\
             [settings]\nclearance = \"1 mm\"\nbed = { width = \"25 cm\", depth = 210 }\n",
        )
        .expect("every measurement takes a unit");
        assert!((spec.drawer_width - 292.1).abs() < 1e-9, "{}", spec.drawer_width);
        assert!((spec.drawer_depth - 523.24).abs() < 1e-9, "{}", spec.drawer_depth);
        assert_eq!(spec.clearance, 1.0);
        assert_eq!(spec.printer.bed_width, 250);
        assert_eq!(spec.printer.bed_depth, 210);
    }

    #[test]
    fn takes_the_grid_size_the_file_states_and_the_standard_otherwise() {
        assert_eq!(parse(MINIMAL).expect("a drawer alone is a valid run").pitch, GRID_PITCH);
        let spec = parse(&format!("{MINIMAL}
[settings]
grid_size = \"21 mm\"
"))
            .expect("a grid size is a setting, and it is a measurement");
        assert_eq!(spec.pitch, 21.0);
    }

    #[test]
    fn rejects_a_grid_size_no_cell_can_be_built_at() {
        let err = parse(&format!("{MINIMAL}
[settings]
grid_size = 4
"))
            .expect_err("a 4 mm cell has no peg profile");
        assert!(err.contains("grid_size"), "{err}");
    }

    #[test]
    fn rejects_fasteners_a_cell_that_small_cannot_hold() {
        let text = format!("{MINIMAL}
[settings]
grid_size = 21
magnets = true
");
        let err = parse(&text).expect_err("a 21 mm cell cannot hold four magnet bores");
        assert!(err.contains("magnets or screws"), "{err}");
        parse(&format!("{MINIMAL}
[settings]
grid_size = 21
"))
            .expect("the same cell without fasteners is fine");
    }

    #[test]
    fn rejects_a_measurement_in_a_unit_it_does_not_know() {
        let err = parse(&format!("{MINIMAL}\n[[objects]]\nname = \"x\"\nsize = [\"3 furlongs\", 10]\n"))
            .expect_err("a furlong is not a unit");
        assert!(err.contains("furlong"), "{err}");
    }
}
