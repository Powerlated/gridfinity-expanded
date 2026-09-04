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

use gridfinity_model::gridfinity::{
    BASE_TOTAL_HEIGHT, FLOOR_THICKNESS, GRID_PITCH, HALF_TOL, HEIGHT_PER_UNIT,
    MIN_FASTENER_GRID_PITCH, MIN_GRID_PITCH, buildable_floor_fillet,
};
use gridfinity_model::printers::{DEFAULT_PRINTER, PRINTER_PROFILES, PrinterProfile};
use gridfinity_project::pack::{PackEffort, PackObject};
use gridfinity_project::rects::{Rect, parts_bounds, parts_connected};

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
            Stated::Measured(text) => text_to_mm(&text)
                .map(Length)
                .map_err(serde::de::Error::custom),
        }
    }
}

/// A length the file may decline to state, as `max_size` does per axis: a number
/// or a measured string is a limit, and an empty string is none.
///
/// An empty string rather than a missing entry, because `max_size` names both
/// axes positionally -- `["4.5 cm", ""]` holds the width and lets the depth be
/// whatever the fit gives it, and there is no way to say that with a shorter
/// list.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Limit(Option<f64>);

impl<'de> serde::Deserialize<'de> for Limit {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        #[derive(serde::Deserialize)]
        #[serde(untagged)]
        enum Stated {
            Millimetres(f64),
            Measured(String),
        }
        match Stated::deserialize(deserializer)? {
            Stated::Millimetres(mm) => Ok(Limit(Some(mm))),
            Stated::Measured(text) if text.trim().is_empty() => Ok(Limit(None)),
            Stated::Measured(text) => text_to_mm(&text)
                .map(|mm| Limit(Some(mm)))
                .map_err(serde::de::Error::custom),
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
    subbin_wall_thickness: Option<Length>,
    subbin_clearance: Option<Length>,
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

/// One thing to organise: a name, how many of it, its footprint stated either
/// as a single `size` or as an edge-connected list of `boxes`, and optionally
/// the most its compartment may grow to.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct ObjectSpec {
    name: String,
    quantity: Option<u32>,
    size: Option<Vec<Length>>,
    boxes: Option<Vec<BoxSpec>>,
    max_size: Option<Vec<Limit>>,
    subbin: Option<Vec<Limit>>,
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

/// The insert an object asks to be given, as the file states it.
///
/// A subbin is a separately printed open-top box that drops into the object's
/// compartment: the object stands in the insert rather than on the compartment
/// floor, and the insert's interior is a strictly bounded square-cornered box,
/// so an axis the file pins holds the object at exactly that measurement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Subbin {
    /// The interior the file asks for, in the object's own frame: width, depth
    /// and height, each `None` on an axis it leaves to the fit. An unstated
    /// width or depth is the object's own, grown by whatever settling gives the
    /// compartment; an unstated height fills the compartment.
    pub interior: [Option<f64>; 3],
}

/// One validated object: what the packer needs, the object as declared, the
/// insert it asks for, the tallest height the file declared for it, which drives
/// no geometry but is reported against the cavity, and the most its compartment
/// may measure.
#[derive(Debug)]
pub struct Object {
    pub pack: PackObject,
    /// The object as the file declares it, in its own frame. `pack.parts` is
    /// what the packer is given, which is the same rectangles for an ordinary
    /// object and the **insert's** footprint for one that asks for a subbin --
    /// the object itself is what a report prints and what `--view` draws.
    pub footprint: Vec<Rect>,
    /// The insert this object is to be given, when it asks for one. Its outer
    /// box is already what `pack.parts` states, so nothing but the geometry and
    /// the report reads this.
    pub subbin: Option<Subbin>,
    pub height: Option<f64>,
    /// The most this object's compartment may measure on each axis of the
    /// object's **own** frame, in millimetres, `None` on an axis it is not held
    /// to. Stated as `max_size`; `settle` grows a compartment into whatever
    /// leftover faces it, and an object that must not turn in its compartment --
    /// a battery, a row of drill bits -- is one that wants that growth bounded.
    /// Never smaller than the object itself, which `parse` refuses.
    pub max_size: [Option<f64>; 2],
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
    /// How thick the wall and floor of an insert are. `wall_thickness` unless
    /// the file names another: an insert is a smaller, lighter part than the bin
    /// it drops into, so it is worth being able to give it thinner material
    /// without thinning the drawer.
    pub subbin_wall_thickness: f64,
    /// The gap between an insert and the compartment it drops into, per side.
    ///
    /// **Not `clearance`**, which is the room an *object* is given inside its
    /// compartment -- a hand tool wants millimetres of it, and a printed part
    /// dropping into a printed pocket does not. `HALF_TOL` unless the file names
    /// another, which is the same gap the Gridfinity standard leaves between a
    /// bin and the baseplate it sits in.
    pub subbin_clearance: f64,
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

/// The overall height in millimetres of a bin of `height_units`, base included.
fn total_height_of(height_units: u32) -> f64 {
    BASE_TOTAL_HEIGHT + HEIGHT_PER_UNIT * f64::from(height_units.max(1))
}

/// How deep a compartment of a bin of `height_units` is: everything above the
/// base and its floor.
fn cavity_depth_of(height_units: u32) -> f64 {
    total_height_of(height_units) - BASE_TOTAL_HEIGHT - FLOOR_THICKNESS
}

/// The floor fillet the model will actually blend a bin of these settings with:
/// the requested radius after the clamps its cavity's depth and corner impose.
///
/// Stated apart from `Spec` because `parse` needs it while it is still building
/// the object list -- an object asking for a subbin is packed against the
/// fillet the model will build, not the one the file asked for -- and two
/// derivations of it would disagree the moment either moved.
fn built_floor_fillet_of(fillet_radius: f64, height_units: u32) -> f64 {
    buildable_floor_fillet(
        fillet_radius,
        cavity_depth_of(height_units),
        fillet_radius.max(0.0),
        false,
    )
}

impl Spec {
    /// The bin's overall height in millimetres, base included.
    pub fn total_height(&self) -> f64 {
        total_height_of(self.height_units)
    }

    /// How deep a compartment is: everything above the base and its floor.
    pub fn cavity_depth(&self) -> f64 {
        cavity_depth_of(self.height_units)
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
        built_floor_fillet_of(self.fillet_radius, self.height_units)
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
    Err(format!(
        "{whose}: {what} must be greater than zero, but is {value}"
    ))
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

/// The `max_size` array as the two limits it states, against the object's own
/// footprint: `[width, depth]`, each a length or an empty string for no limit.
///
/// A limit is what the compartment may measure, so it is refused below the
/// object's own extent on that axis -- a compartment smaller than the thing in
/// it is not a smaller compartment, it is a fit that does not hold. Stating the
/// object's own size, as a battery tray does, is the tightest legal answer and
/// means "grow this one no further".
fn max_size_of(
    max_size: Option<&[Limit]>,
    parts: &[Rect],
    whose: &str,
) -> Result<[Option<f64>; 2], String> {
    let Some(stated) = max_size else {
        return Ok([None, None]);
    };
    let [width, depth] = match stated {
        [w, d] => [w, d],
        other => {
            return Err(format!(
                "{whose}: max_size is [width, depth], with an empty string for no limit, but has \
                 {} entries",
                other.len()
            ));
        }
    };
    let bounds = parts_bounds(parts);
    let mut out = [None, None];
    for (index, (limit, (axis, own))) in [width, depth]
        .into_iter()
        .zip([("width", bounds.width), ("depth", bounds.depth)])
        .enumerate()
    {
        let Some(most) = limit.0 else {
            continue;
        };
        positive(most, &format!("max_size {axis}"), whose)?;
        if most + 1e-9 < own {
            return Err(format!(
                "{whose}: max_size {axis} is {most} mm, but the object is {own} mm across, so no \
                 compartment that size holds it"
            ));
        }
        out[index] = Some(most);
    }
    Ok(out)
}

/// The `subbin` array as the interior it asks for: `[width, depth, height]`,
/// each a length or an empty string for an axis left to the fit.
///
/// `None` for an object that asks for no insert, which is every object written
/// before the key existed.
fn subbin_of(stated: Option<&[Limit]>, whose: &str) -> Result<Option<Subbin>, String> {
    let Some(stated) = stated else {
        return Ok(None);
    };
    let [width, depth, height] = match stated {
        [w, d, h] => [w, d, h],
        other => {
            return Err(format!(
                "{whose}: subbin is [width, depth, height] of the insert's interior, with an \
                 empty string for an axis the fit decides, but has {} entries",
                other.len()
            ));
        }
    };
    let mut interior = [None; 3];
    for (index, (limit, axis)) in [width, depth, height]
        .into_iter()
        .zip(["subbin width", "subbin depth", "subbin height"])
        .enumerate()
    {
        if let Some(measure) = limit.0 {
            positive(measure, axis, whose)?;
            interior[index] = Some(measure);
        }
    }
    Ok(Some(Subbin { interior }))
}

/// What the packer is given for an object that asks for a subbin, and the limit
/// its compartment is held to: the insert's outer box less twice the floor
/// fillet, and that same number on every axis the interior is pinned to.
///
/// **The deflation is the whole of why an insert fits snugly.** A claim reserves
/// `clearance + floor_fillet + divider / 2` because an object rests on the
/// compartment floor and the blend takes `floor_fillet` of that floor from every
/// wall. An object in an insert rests on neither: the insert stands on the floor
/// and its bottom chamfer clears the blend, and the gap around it is a part
/// fitting a part rather than a tool sitting in a compartment. So both are
/// handed back and the insert's own clearance put in their place -- `deflate` is
/// `clearance + floor_fillet - subbin_clearance`, and what is packed shrinks by
/// twice it:
///
/// ```text
/// footprint = interior + 2 * wall - 2 * deflate
/// pocket    = footprint + 2 * (clearance + fillet)
///           = interior + 2 * wall + 2 * subbin_clearance
/// outer     = pocket - 2 * subbin_clearance = interior + 2 * wall
/// ```
///
/// so the insert stands `subbin_clearance` inside the compartment at every height
/// and its walls come out exactly `wall` thick. On an axis the file leaves open
/// the same arithmetic runs backwards and the insert takes whatever settling
/// grew the compartment to.
///
/// For an object with an insert **every stated limit is about the interior**:
/// `max_size` on an axis the subbin leaves open caps that interior, and stating
/// both on one axis is refused rather than silently resolved.
fn subbin_footprint(
    subbin: &Subbin,
    parts: &[Rect],
    max_size: Option<&[Limit]>,
    wall: f64,
    deflate: f64,
    whose: &str,
) -> Result<(Vec<Rect>, [Option<f64>; 2]), String> {
    assert!(
        wall > 0.0,
        "an insert is built with {wall} mm walls, which is not a body"
    );
    if parts.len() != 1 {
        return Err(format!(
            "{whose}: an insert is one rectangular box, so an object of {} boxes cannot be given \
             one",
            parts.len()
        ));
    }
    let capped = match max_size {
        Some([w, d]) => [w.0, d.0],
        Some(other) => {
            return Err(format!(
                "{whose}: max_size is [width, depth], with an empty string for no limit, but has \
                 {} entries",
                other.len()
            ));
        }
        None => [None, None],
    };
    let own = parts_bounds(parts);
    let mut extent = [0.0; 2];
    let mut limits = [None; 2];
    for (index, (axis, own)) in [("width", own.width), ("depth", own.depth)]
        .into_iter()
        .enumerate()
    {
        let pinned = subbin.interior[index];
        if let Some(interior) = pinned
            && interior + 1e-9 < own
        {
            return Err(format!(
                "{whose}: subbin {axis} is {interior} mm, but the object is {own} mm across, so \
                 no insert that size holds it"
            ));
        }
        if pinned.is_some() && capped[index].is_some() {
            return Err(format!(
                "{whose}: subbin and max_size both state the {axis}, which is one measurement -- \
                 the insert's interior -- stated twice"
            ));
        }
        if let Some(cap) = capped[index]
            && cap + 1e-9 < own
        {
            return Err(format!(
                "{whose}: max_size {axis} is {cap} mm, but the object is {own} mm across, so no \
                 insert that size holds it"
            ));
        }
        let footprint = |interior: f64| interior + 2.0 * wall - 2.0 * deflate;
        extent[index] = footprint(pinned.unwrap_or(own));
        if extent[index] <= 0.0 {
            return Err(format!(
                "{whose}: an insert {} mm across inside {wall} mm walls gives back more than it \
                 claims, {deflate} mm of it a side, so there is nothing left to pack",
                pinned.unwrap_or(own)
            ));
        }
        limits[index] = pinned.or(capped[index]).map(footprint);
    }
    Ok((vec![Rect::new(0.0, 0.0, extent[0], extent[1])], limits))
}

/// The `boxes` list as a part list plus the tallest height any of them declares.
fn boxes_to_parts(boxes: &[BoxSpec], whose: &str) -> Result<(Vec<Rect>, Option<f64>), String> {
    if boxes.is_empty() {
        return Err(format!(
            "{whose}: boxes is empty, so the object has no footprint"
        ));
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
        return Err(format!(
            "settings.clearance is {clearance}, which is less than none"
        ));
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
            "settings.grid_size is {pitch} mm, and a Gridfinity cell cannot be built below \
             {MIN_GRID_PITCH} mm -- its peg profile does not close"
        ));
    }
    let magnets = settings.magnets.unwrap_or(false);
    let screws = settings.screws.unwrap_or(false);
    if (magnets || screws) && pitch <= MIN_FASTENER_GRID_PITCH {
        return Err(format!(
            "settings.grid_size is {pitch} mm, and a cell carries magnets or screws only above \
             {MIN_FASTENER_GRID_PITCH} mm -- the four bores of a smaller cell run into one another"
        ));
    }
    let height_units = settings.height_units.unwrap_or(3);
    if height_units == 0 {
        return Err("settings.height_units is 0, so the bin has no cavity".to_string());
    }

    let subbin_wall_thickness = positive(
        settings
            .subbin_wall_thickness
            .map_or(wall_thickness, Length::mm),
        "settings.subbin_wall_thickness",
        "settings",
    )?;
    let subbin_clearance = settings.subbin_clearance.map_or(HALF_TOL, Length::mm);
    if subbin_clearance < 0.0 {
        return Err(format!(
            "settings.subbin_clearance is {subbin_clearance}, which is less than none"
        ));
    }
    let floor_fillet = built_floor_fillet_of(fillet_radius, height_units);

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
                return Err(format!(
                    "{whose}: states both size and boxes; state one or the other"
                ));
            }
            (Some(size), None) => size_to_parts(size, &whose)?,
            (None, Some(boxes)) => boxes_to_parts(boxes, &whose)?,
            (None, None) => {
                return Err(format!(
                    "{whose}: has neither size nor boxes, so it has no footprint"
                ));
            }
        };
        if !parts_connected(&parts) {
            return Err(format!(
                "{whose}: its boxes do not touch along an edge, so it is more than one object"
            ));
        }
        let subbin = subbin_of(spec.subbin.as_deref(), &whose)?;
        let (packed, max_size) = match &subbin {
            Some(sub) => subbin_footprint(
                sub,
                &parts,
                spec.max_size.as_deref(),
                subbin_wall_thickness,
                clearance + floor_fillet - subbin_clearance,
                &whose,
            )?,
            None => (
                parts.clone(),
                max_size_of(spec.max_size.as_deref(), &parts, &whose)?,
            ),
        };
        objects.push(Object {
            pack: PackObject {
                id: spec.name.clone(),
                name: spec.name.clone(),
                parts: packed,
                quantity,
            },
            footprint: parts,
            subbin,
            height,
            max_size,
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
        subbin_wall_thickness,
        subbin_clearance,
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

    /// `max_size` states a limit per axis, and an empty string is how an axis
    /// says it has none. Both are lengths like every other measurement, so a
    /// unit may be named.
    #[test]
    fn reads_a_max_size_per_axis_with_an_empty_string_for_no_limit() {
        let spec = parse(
            "[drawer]
width = 400
depth = 300

[[objects]]
name = \"battery\"
             size = [\"4.5 cm\", \"124 mm\"]
max_size = [\"4.5 cm\", \"\"]
",
        )
        .expect("a stated max_size is a valid run");
        assert_eq!(spec.objects[0].max_size, [Some(45.0), None]);
    }

    /// `subbin` states the insert's interior on three axes, an empty string
    /// leaving one to the fit, and what the packer is given is that interior
    /// grown by two walls, handed back the floor fillet its chamfer replaces and
    /// the object clearance it does not want, and given its own gap instead.
    #[test]
    fn reads_a_subbin_as_the_interior_it_states() {
        let spec = parse(
            "[drawer]
width = 400
depth = 300

[[objects]]
name = \"battery\"
size =   [\"4.5 cm\", \"124 mm\"]
subbin = [\"4.5cm\", \"\", \"3cm\"]
",
        )
        .expect("a stated subbin is a valid run");
        let object = &spec.objects[0];
        assert_eq!(
            object.subbin,
            Some(Subbin {
                interior: [Some(45.0), None, Some(30.0)]
            })
        );
        assert_eq!(object.footprint, vec![Rect::new(0.0, 0.0, 45.0, 124.0)]);

        let wall = spec.subbin_wall_thickness;
        let deflate = spec.clearance + spec.built_floor_fillet() - spec.subbin_clearance;
        let packed = parts_bounds(&object.pack.parts);
        let want = |interior: f64| interior + 2.0 * wall - 2.0 * deflate;
        assert!(
            (packed.width - want(45.0)).abs() < 1e-9 && (packed.depth - want(124.0)).abs() < 1e-9,
            "the insert's outer box less what it gives back is {} x {}, but {} x {} was \
             packed",
            want(45.0),
            want(124.0),
            packed.width,
            packed.depth
        );
        assert_eq!(
            object.max_size,
            [Some(want(45.0)), None],
            "a pinned axis holds the compartment and an open one grows"
        );
    }

    /// Every way a `subbin` says nothing buildable is a named error against the
    /// object that asked, not a silently resolved reading.
    #[test]
    fn a_subbin_that_states_no_insert_is_refused() {
        let bad = |object: &str| {
            parse(&format!(
                "[drawer]
width = 400
depth = 300

{object}"
            ))
            .expect_err("the fixture is meant to be refused")
        };
        let err = bad("[[objects]]
name = \"a\"
size = [50, 20]
subbin = [40, \"\", 10]
");
        assert!(err.contains("subbin width"), "{err}");

        let err = bad("[[objects]]
name = \"a\"
size = [50, 20]
max_size = [60, \"\"]
             subbin = [55, \"\", 10]
");
        assert!(err.contains("stated twice"), "{err}");

        let err = bad(
            "[[objects]]
name = \"a\"
boxes = [{ x = 0, y = 0, width = 10, depth = 10 },              { x = 10, y = 0, width = 10, depth = 10 }]
subbin = [\"\", \"\", 10]
",
        );
        assert!(err.contains("one rectangular box"), "{err}");

        let err = bad("[[objects]]
name = \"a\"
size = [50, 20]
subbin = [50, 20]
");
        assert!(err.contains("[width, depth, height]"), "{err}");
    }

    /// An object that asks for no insert is packed as exactly what it declares,
    /// which is what every file written before the key existed says.
    #[test]
    fn an_object_that_asks_for_no_subbin_is_packed_as_itself() {
        let spec = parse(
            "[drawer]
width = 400
depth = 300

[[objects]]
name = \"a\"
size = [10, 20]
",
        )
        .expect("the fixture is a valid run");
        assert_eq!(spec.objects[0].subbin, None);
        assert_eq!(spec.objects[0].pack.parts, spec.objects[0].footprint);
    }

    /// An object with no `max_size` is held to nothing, which is what every file
    /// written before the key existed says.
    #[test]
    fn an_object_that_states_no_max_size_is_held_to_nothing() {
        let spec = parse(
            "[drawer]
width = 400
depth = 300

[[objects]]
name = \"a\"
size = [10, 10]
",
        )
        .expect("the fixture is a valid run");
        assert_eq!(spec.objects[0].max_size, [None, None]);
    }

    /// A limit under the object's own extent is refused rather than clamped up:
    /// a compartment smaller than the thing in it is not a tighter fit, it is a
    /// fit that does not hold, and the file is where that can still be said.
    #[test]
    fn a_max_size_smaller_than_the_object_is_refused() {
        let err = parse(
            "[drawer]
width = 400
depth = 300

[[objects]]
name = \"a\"
             size = [50, 20]
max_size = [40, \"\"]
",
        )
        .expect_err("40 mm does not hold a 50 mm object");
        assert!(
            err.contains("max_size width"),
            "the error names the axis: {err}"
        );

        let short = parse(
            "[drawer]
width = 400
depth = 300

[[objects]]
name = \"a\"
             size = [50, 20]
max_size = [60]
",
        )
        .expect_err("max_size names both axes");
        assert!(short.contains("max_size is [width, depth]"), "{short}");
    }

    #[test]
    fn fills_every_setting_the_file_leaves_out() {
        let spec = parse(MINIMAL).expect("a drawer alone is a valid run");
        assert_eq!(spec.effort, PackEffort::Standard);
        assert_eq!(spec.height_units, 3);
        assert_eq!(spec.printer.name, DEFAULT_PRINTER.name);
        assert!(
            spec.baseplate,
            "a drawer bin gets the grid it sits in unless asked not to"
        );
        assert_eq!(spec.subbin_wall_thickness, spec.wall_thickness);
        assert_eq!(
            spec.subbin_clearance, HALF_TOL,
            "an insert drops into its compartment on the gap the standard leaves between a bin \
             and its baseplate, not on the room an object is given"
        );
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
        let spec = parse(&format!(
            "{MINIMAL}
[settings]
baseplate = false
"
        ))
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
        let stated = parse(&format!(
            "{MINIMAL}
[settings]
tidy_absorb = \"4 mm\"
"
        ))
        .expect("tidy_absorb is a setting");
        assert!((stated.tidy_absorb - 4.0).abs() < 1e-9);
        let refused = parse(&format!(
            "{MINIMAL}
[settings]
tidy_absorb = -1
"
        ))
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
        assert!(
            (spec.drawer_width - 292.1).abs() < 1e-9,
            "{}",
            spec.drawer_width
        );
        assert!(
            (spec.drawer_depth - 523.24).abs() < 1e-9,
            "{}",
            spec.drawer_depth
        );
        assert_eq!(spec.clearance, 1.0);
        assert_eq!(spec.printer.bed_width, 250);
        assert_eq!(spec.printer.bed_depth, 210);
    }

    #[test]
    fn takes_the_grid_size_the_file_states_and_the_standard_otherwise() {
        assert_eq!(
            parse(MINIMAL).expect("a drawer alone is a valid run").pitch,
            GRID_PITCH
        );
        let spec = parse(&format!(
            "{MINIMAL}
[settings]
grid_size = \"21 mm\"
"
        ))
        .expect("a grid size is a setting, and it is a measurement");
        assert_eq!(spec.pitch, 21.0);
    }

    #[test]
    fn rejects_a_grid_size_no_cell_can_be_built_at() {
        let err = parse(&format!(
            "{MINIMAL}
[settings]
grid_size = 4
"
        ))
        .expect_err("a 4 mm cell has no peg profile");
        assert!(err.contains("grid_size"), "{err}");
    }

    #[test]
    fn rejects_fasteners_a_cell_that_small_cannot_hold() {
        let text = format!(
            "{MINIMAL}
[settings]
grid_size = 21
magnets = true
"
        );
        let err = parse(&text).expect_err("a 21 mm cell cannot hold four magnet bores");
        assert!(err.contains("magnets or screws"), "{err}");
        parse(&format!(
            "{MINIMAL}
[settings]
grid_size = 21
"
        ))
        .expect("the same cell without fasteners is fine");
    }

    #[test]
    fn rejects_a_measurement_in_a_unit_it_does_not_know() {
        let err = parse(&format!(
            "{MINIMAL}\n[[objects]]\nname = \"x\"\nsize = [\"3 furlongs\", 10]\n"
        ))
        .expect_err("a furlong is not a unit");
        assert!(err.contains("furlong"), "{err}");
    }
}
