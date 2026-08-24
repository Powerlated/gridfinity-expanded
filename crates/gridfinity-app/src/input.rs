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

use gridfinity_cad::gridfinity::{BASE_TOTAL_HEIGHT, FLOOR_THICKNESS, HEIGHT_PER_UNIT};
use gridfinity_cad::printers::{DEFAULT_PRINTER, PRINTER_PROFILES, PrinterProfile};
use gridfinity_cad::project::pack::{PackEffort, PackObject};
use gridfinity_cad::project::rects::{Rect, parts_connected};

/// The drawer's inside measurements, in millimetres.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct DrawerSpec {
    width: f64,
    depth: f64,
}

/// A printer bed stated directly rather than by profile name, in millimetres.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct BedSpec {
    width: i32,
    depth: i32,
}

/// Every setting the file may state, all optional.
#[derive(Default, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct SettingsSpec {
    divider_thickness: Option<f64>,
    clearance: Option<f64>,
    effort: Option<String>,
    height_units: Option<u32>,
    wall_thickness: Option<f64>,
    fillet_radius: Option<f64>,
    magnets: Option<bool>,
    screws: Option<bool>,
    printer: Option<String>,
    bed: Option<BedSpec>,
}

/// One box of an object, in the object's own millimetre frame.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct BoxSpec {
    #[serde(default)]
    x: f64,
    #[serde(default)]
    y: f64,
    width: f64,
    depth: f64,
    height: Option<f64>,
}

/// One thing to organise: a name, how many of it, and its footprint stated
/// either as a single `size` or as an edge-connected list of `boxes`.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct ObjectSpec {
    name: String,
    quantity: Option<u32>,
    size: Option<Vec<f64>>,
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
    pub divider_thickness: f64,
    pub clearance: f64,
    pub wall_thickness: f64,
    pub fillet_radius: f64,
    pub height_units: u32,
    pub magnets: bool,
    pub screws: bool,
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
fn size_to_parts(size: &[f64], whose: &str) -> Result<(Vec<Rect>, Option<f64>), String> {
    let (width, depth, height) = match size {
        [w, d] => (*w, *d, None),
        [w, d, h] => (*w, *d, Some(*h)),
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
        positive(b.width, "box width", whose)?;
        positive(b.depth, "box depth", whose)?;
        if let Some(h) = b.height {
            positive(h, "box height", whose)?;
            height = Some(height.map_or(h, |t: f64| t.max(h)));
        }
        parts.push(Rect::new(b.x, b.y, b.width, b.depth));
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
            if bed.width <= 0 || bed.depth <= 0 {
                return Err(format!(
                    "settings.bed is {} x {} mm, which is not a bed",
                    bed.width, bed.depth
                ));
            }
            Ok(PrinterProfile {
                name: "Custom",
                bed_width: bed.width,
                bed_depth: bed.depth,
            })
        }
        (None, None) => Ok(DEFAULT_PRINTER),
    }
}

/// The TOML text as a validated run, or the first reason it is not one.
pub fn parse(text: &str) -> Result<Spec, String> {
    let file: InputFile = toml::from_str(text).map_err(|e| e.to_string())?;
    positive(file.drawer.width, "drawer.width", "drawer")?;
    positive(file.drawer.depth, "drawer.depth", "drawer")?;

    let settings = &file.settings;
    let effort_name = settings.effort.as_deref().unwrap_or("standard");
    let effort = PackEffort::from_name(effort_name).ok_or_else(|| {
        format!("settings.effort is {effort_name:?}, not one of quick, standard, thorough")
    })?;
    let divider_thickness = positive(
        settings.divider_thickness.unwrap_or(1.2),
        "settings.divider_thickness",
        "settings",
    )?;
    let wall_thickness = positive(
        settings.wall_thickness.unwrap_or(1.2),
        "settings.wall_thickness",
        "settings",
    )?;
    let clearance = settings.clearance.unwrap_or(0.5);
    if clearance < 0.0 {
        return Err(format!("settings.clearance is {clearance}, which is less than none"));
    }
    let fillet_radius = settings.fillet_radius.unwrap_or(2.5);
    if fillet_radius < 0.0 {
        return Err(format!(
            "settings.fillet_radius is {fillet_radius}, which is less than none"
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

    Ok(Spec {
        drawer_width: file.drawer.width,
        drawer_depth: file.drawer.depth,
        divider_thickness,
        clearance,
        wall_thickness,
        fillet_radius,
        height_units,
        magnets: settings.magnets.unwrap_or(false),
        screws: settings.screws.unwrap_or(false),
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
}
