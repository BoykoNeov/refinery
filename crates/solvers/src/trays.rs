//! Sieve-tray hydraulics (M73.1, docs/DESIGN.md §78): what a heat-balance
//! cascade's converged loads come to on the trays it declares — how near each
//! tray runs to flooding and to weeping, its pressure drop and its downcomer's
//! backup.
//!
//! **A reading, not a model of the column.** Nothing here feeds back into the
//! stage balances: the column still solves at its one pressure, and a tray past
//! flood still separates as an equilibrium stage (§78.5 defers how the products
//! worsen). Computed once per converged pass, on columns with trays only.
//!
//! Every correlation was checked against a published source before it was
//! written (§78.4.1), and the set reproduces a published worked example — NPTEL,
//! *Chemical Engineering Design II*, Module 7, a sieve tray by Sinnott's method —
//! to its rounding (`tests/reference/sieve_tray.rs`). The kernel is written in
//! Sinnott's units (heads in mm of liquid), as his equations are, and returns SI.
//!
//! Sources, cited by key below:
//! - **[P]** Perry's *Chemical Engineers' Handbook*, 8th ed. (2008), §14,
//!   pp. 14-37–14-38.
//! - **[S]** Sinnott, *Coulson & Richardson's Chemical Engineering* Vol. 6, 4th
//!   ed. (2005), §11.13.
//! - **[S22]** Souza et al., *Chem. Eng. Res. Des.* 180 (2022) 1–12.
//! - **[S25]** Souza et al., *Ind. Eng. Chem. Res.* 64 (2025) 2256–2273.

use crate::cascade::Flows;
use refinery_core::components::{Phase, Slate};
use refinery_core::graph::{CascadeSpec, TraySection};
use refinery_core::traits::TrayHydraulics;
use refinery_core::units::{
    KgPerM3, KgPerSec, Meter, MeterPerSec, NewtonPerMeter, Pascal, SquareMeter, G, R_GAS,
};
use refinery_core::SimError;

/// The tray spacings Fair's chart has curves for [m], 150–900 mm ([P]
/// Fig. 14-31). Outside them the fit extrapolates a chart that is not there.
pub const SPACING_RANGE_M: (f64, f64) = (0.15, 0.90);
/// The lowest weir admitted [m]. The weep constant's fit is a logarithm of the
/// liquid's depth, checked from 55 to 72 mm and running away at small depth;
/// weirs of 40–90 mm are the common ones above atmospheric pressure (NPTEL
/// §7.6.3), 6–12 mm under vacuum, which these columns are not.
pub const MIN_WEIR_M: f64 = 0.025;
/// Fair's chart holds for a weir under 15 % of the tray spacing ([P] p. 14-38,
/// restriction 2).
pub const MAX_WEIR_OF_SPACING: f64 = 0.15;
/// Fair's chart holds for perforations of 13 mm or less ([P] restriction 3).
pub const MAX_HOLE_M: f64 = 0.013;
/// The hole share the orifice coefficient is published for, and over which
/// Fair's hole-area correction is one ([P] restriction 4: 10 % or more).
pub const HOLE_SHARE_RANGE: (f64, f64) = (0.10, 0.15);
/// The most liquid paths a tray is admitted with.
pub const MAX_PASSES: u32 = 4;

/// Each cut's density falls this much per kelvin above its declared (15 °C)
/// one [kg/m³/K] — a game value: petroleum liquids expand about 0.0006–0.0011
/// per kelvin, and this is 0.0008 at 800 kg/m³ (§78.4.1).
const DENSITY_SLOPE: f64 = 0.65;
/// The lightest a cut is taken at the tray, hot [kg/m³] — a game value.
const DENSITY_FLOOR: f64 = 300.0;
/// The declared densities' reference temperature, 15 °C [K].
const DENSITY_REFERENCE_K: f64 = 288.15;
/// A cut's surface tension at 15 °C [N/m] — a game value (§78.4.1).
const SURFACE_TENSION_AT_REFERENCE: f64 = 0.027;
/// The least surface tension a tray's liquid is taken at [N/m].
const SURFACE_TENSION_FLOOR: f64 = 0.001;

/// A tray section's geometry, derived once from its declaration: the areas the
/// correlations read. Public, with public fields, so the reference test can
/// state a published tray's own numbers rather than re-derive them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrayGeometry {
    /// Tray spacing [m].
    pub spacing: Meter,
    /// Outlet weir height [m].
    pub weir_height: Meter,
    /// Perforation diameter [m].
    pub hole_diameter: Meter,
    /// The column's area less one downcomer's [m²] — what Fair's chart rates.
    pub net_area: SquareMeter,
    /// The holes' total area [m²].
    pub hole_area: SquareMeter,
    /// One downcomer's area [m²].
    pub downcomer_area: SquareMeter,
    /// The outlet weir's length [m], per pass.
    pub weir_length: Meter,
    /// The gap under the downcomer's apron [m].
    pub clearance: Meter,
    /// The orifice coefficient `C0` of the dry-plate drop [-].
    pub orifice_coefficient: f64,
    /// The liquid's paths across the tray.
    pub passes: u32,
}

impl TrayGeometry {
    /// A declared section's geometry. A multipass tray is the simplification
    /// §78.4.1 names: its areas a single-pass tray's (net `A_c − A_d`, active
    /// `A_c − 2·A_d`), each pass over a weir the downcomer's chord long.
    pub fn of(section: &TraySection) -> Self {
        let diameter = section.diameter.value();
        let column = std::f64::consts::PI * diameter * diameter / 4.0;
        let downcomer = section.downcomer_share * column;
        let active = column - 2.0 * downcomer;
        Self {
            spacing: section.spacing,
            weir_height: section.weir_height,
            hole_diameter: section.hole_diameter,
            net_area: SquareMeter(column - downcomer),
            hole_area: SquareMeter(section.hole_area_share * active),
            downcomer_area: SquareMeter(downcomer),
            weir_length: Meter(diameter * chord_of_segment(section.downcomer_share)),
            clearance: section.clearance,
            orifice_coefficient: orifice_coefficient(section.hole_area_share),
            passes: section.passes,
        }
    }
}

/// The chord of a circular segment holding `share` of the circle's area, over
/// the diameter [-] — the weir of a segmental downcomer. The segment's area
/// share is `(θ − sin θ)/(2π)` at a central angle `θ`, which rises with `θ`, and
/// its chord `D·sin(θ/2)`; `θ` found by bisection (a fixed count, deterministic).
/// 0.727 at 10 %, 0.764 at 12 % — Sinnott's Fig. 11.31 reads 0.73 and 0.76.
fn chord_of_segment(share: f64) -> f64 {
    let (mut low, mut high) = (0.0_f64, std::f64::consts::PI);
    for _ in 0..60 {
        let angle = 0.5 * (low + high);
        if (angle - angle.sin()) / std::f64::consts::TAU < share {
            low = angle;
        } else {
            high = angle;
        }
    }
    (0.25 * (low + high)).sin()
}

/// The dry-plate orifice coefficient at a hole share, for a plate as thick as
/// its holes (Sinnott Fig. 11.34): 0.84 at 10 % (his worked example) and 0.88
/// at 15 % (NPTEL p. 25), straight between them — the only two published
/// readings (§78.4.1). The loader admits nothing outside them.
fn orifice_coefficient(hole_area_share: f64) -> f64 {
    let (low, high) = HOLE_SHARE_RANGE;
    0.84 + (0.88 - 0.84) * (hole_area_share - low) / (high - low)
}

/// Fair's capacity factor at jet flood, `C_sb` [m/s], at a tray spacing and a
/// flow parameter — Fair's chart as fitted by Lygeros & Magoulas, [P] eq. 14-90:
/// `C_sb = 0.0105 + 8.127e-4·TS^0.755·exp(−1.463·F_LV^0.842)`, `TS` in mm. Its
/// error against the chart's own curves 3.1 % on average and 6.7 % at worst at
/// 0.61 m spacing ([S25] Table 5); it reads low where the wide spacings' curves
/// flatten at small flow parameters (§78.4.1).
pub fn flood_capacity(spacing: Meter, flow_parameter: f64) -> MeterPerSec {
    let spacing_mm = spacing.value() * 1000.0;
    MeterPerSec(
        0.0105 + 8.127e-4 * spacing_mm.powf(0.755) * (-1.463 * flow_parameter.powf(0.842)).exp(),
    )
}

/// One tray's loads: the vapour coming up through it and the liquid going over
/// its weir, each with its density, and the liquid's surface tension.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrayLoad {
    /// The vapour rising through the tray [kg/s], ≥ 0.
    pub vapour: KgPerSec,
    /// Its density [kg/m³], > 0.
    pub vapour_density: KgPerM3,
    /// The liquid leaving the tray over its weir [kg/s], ≥ 0.
    pub liquid: KgPerSec,
    /// Its density [kg/m³], above the vapour's.
    pub liquid_density: KgPerM3,
    /// Its surface tension [N/m], > 0.
    pub surface_tension: NewtonPerMeter,
}

/// One tray's hydraulics, every head the checks pass through — what the
/// reference test reads against the published example, and what
/// `TrayHydraulics` is made from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SieveTray {
    /// Fair's flow parameter `(L/G)/passes·√(ρV/ρL)` [-] ([P] eq. 14-89).
    pub flow_parameter: f64,
    /// The vapour's velocity through the net area at jet flood [m/s].
    pub flood_velocity: MeterPerSec,
    /// Its velocity through the net area [m/s].
    pub net_velocity: MeterPerSec,
    /// Its velocity through the holes [m/s].
    pub hole_velocity: MeterPerSec,
    /// The hole velocity at the weep point [m/s].
    pub weep_velocity: MeterPerSec,
    /// The liquid's crest over the weir [m of liquid].
    pub weir_crest: Meter,
    /// The dry plate's head [m of liquid].
    pub dry_head: Meter,
    /// The residual head [m of liquid].
    pub residual_head: Meter,
    /// The tray's whole head, dry plate, liquid on it and residual [m of
    /// liquid].
    pub total_head: Meter,
    /// The head the liquid loses under the downcomer's apron [m of liquid].
    pub downcomer_loss: Meter,
    /// The clear liquid backed up in the downcomer [m].
    pub backup: Meter,
    /// The most backup the downcomer holds, `(TS + h_w)/2` [m].
    pub backup_limit: Meter,
    /// The tray's pressure drop, `ρL·g·h_t` [Pa].
    pub pressure_drop: Pascal,
}

impl SieveTray {
    /// What the snapshot reports of it, at `stage` (one-based from the top).
    pub fn reading(&self, stage: u32) -> TrayHydraulics {
        TrayHydraulics {
            stage,
            flood_fraction: self.net_velocity.value() / self.flood_velocity.value(),
            weep_margin: self.hole_velocity.value() / self.weep_velocity.value(),
            pressure_drop: self.pressure_drop,
            downcomer_backup: self.backup.value() / self.backup_limit.value(),
        }
    }
}

/// One tray's hydraulics at its loads (§78.4.1). Every denominator is bounded
/// away from zero by the loads' own contract (densities, surface tension > 0,
/// liquid denser than vapour) and the loader's (every length > 0), so the
/// answer is finite; no vapour reads as no flood and full weeping.
pub fn sieve_tray(geometry: &TrayGeometry, load: &TrayLoad) -> SieveTray {
    let passes = f64::from(geometry.passes);
    let vapour = load.vapour.value();
    let liquid = load.liquid.value();
    let rho_v = load.vapour_density.value();
    let rho_l = load.liquid_density.value();
    let volume = vapour / rho_v;
    // Fair's flow parameter, the liquid divided between the passes ([P]
    // eq. 14-89 and p. 14-37: "for multipass trays, the ratio needs to be
    // divided by the number of passes"). No vapour: the chart's far right.
    let flow_parameter = if vapour > 0.0 {
        liquid / vapour / passes * (rho_v / rho_l).sqrt()
    } else {
        f64::INFINITY
    };
    // The flood velocity on the net area, the surface tension in mN/m ([P]
    // eq. 14-91); the hole share is 10 % or more, so Fair's correction for
    // smaller holes is one.
    let surface_tension_mn = load.surface_tension.value() * 1000.0;
    let flood_velocity = flood_capacity(geometry.spacing, flow_parameter).value()
        * (surface_tension_mn / 20.0).powf(0.2)
        * ((rho_l - rho_v) / rho_v).sqrt();
    let net_velocity = volume / geometry.net_area.value();
    let hole_velocity = volume / geometry.hole_area.value();

    // Sinnott's heads, in mm of liquid ([S] eqs. 11.84–11.92).
    let weir_mm = geometry.weir_height.value() * 1000.0;
    let hole_mm = geometry.hole_diameter.value() * 1000.0;
    let pass_liquid = liquid / passes;
    // Francis weir crest: h_ow = 750·(L_w/(ρL·l_w))^(2/3) mm ([S] eq. 11.85).
    let crest_mm = 750.0 * (pass_liquid / (rho_l * geometry.weir_length.value())).powf(2.0 / 3.0);
    // Weep point (Eduljee 1959): u_h,min = (K2 − 0.90·(25.4 − d_h))/√ρV
    // ([S] eq. 11.84), K2 Ogboja & Kuye's fit of [S] Fig. 11.30,
    // 23.48 + 1.66·ln(h_w + h_ow), mm ([S22] eq. 67).
    let k2 = 23.48 + 1.66 * (weir_mm + crest_mm).ln();
    let weep_velocity = (k2 - 0.90 * (25.4 - hole_mm)) / rho_v.sqrt();
    // Dry plate: h_d = 51·(u_h/C0)²·ρV/ρL mm ([S] eq. 11.88).
    let dry_mm = 51.0 * (hole_velocity / geometry.orifice_coefficient).powi(2) * rho_v / rho_l;
    // Residual head: h_r = 12.5e3/ρL mm (Hunt et al. 1955; [S] eq. 11.89).
    let residual_mm = 12.5e3 / rho_l;
    // The tray's whole head: h_t = h_d + (h_w + h_ow) + h_r ([S] eq. 11.90).
    let total_mm = dry_mm + weir_mm + crest_mm + residual_mm;
    // Under the apron: h_dc = 166·(L_wd/(ρL·A_m))² mm, A_m the smaller of the
    // clearance's area and the downcomer's (Cicalese et al. 1947; [S]
    // eq. 11.92).
    let apron = (geometry.clearance.value() * geometry.weir_length.value())
        .min(geometry.downcomer_area.value());
    let downcomer_mm = 166.0 * (pass_liquid / (rho_l * apron)).powi(2);
    // The downcomer's backup: h_b = (h_w + h_ow) + h_t + h_dc ([S] eq. 11.91),
    // flooded past (TS + h_w)/2 — the froth in it at half the liquid's density
    // ([P] eqs. 14-92–14-93).
    let backup_mm = weir_mm + crest_mm + total_mm + downcomer_mm;
    let mm = |head: f64| Meter(head / 1000.0);
    SieveTray {
        flow_parameter,
        flood_velocity: MeterPerSec(flood_velocity),
        net_velocity: MeterPerSec(net_velocity),
        hole_velocity: MeterPerSec(hole_velocity),
        weep_velocity: MeterPerSec(weep_velocity),
        weir_crest: mm(crest_mm),
        dry_head: mm(dry_mm),
        residual_head: mm(residual_mm),
        total_head: mm(total_mm),
        downcomer_loss: mm(downcomer_mm),
        backup: mm(backup_mm),
        backup_limit: Meter(0.5 * (geometry.spacing.value() + geometry.weir_height.value())),
        // ΔP = ρL·g·h_t.
        pressure_drop: Pascal(rho_l * G * total_mm / 1000.0),
    }
}

/// The liquid's density at the tray [kg/m³] — a game value (§78.4.1): each
/// condensed cut its declared (15 °C) density less `DENSITY_SLOPE` per kelvin
/// above, floored, mixed by volume (ideal mixing, as `Composition` mixes).
/// `liquid` is the stage's mole fractions; gas-phase members carry no
/// condensed density and are left out.
fn liquid_density(slate: &Slate, liquid: &[f64], temperature: f64) -> f64 {
    let (mut mass, mut volume) = (0.0, 0.0);
    for (c, x) in liquid.iter().enumerate() {
        let component = slate.get(c);
        let Some(density) = component
            .density
            .filter(|_| component.phase == Phase::Liquid)
        else {
            continue;
        };
        let m = x * component.molar_mass.value();
        let hot = (density.value() - DENSITY_SLOPE * (temperature - DENSITY_REFERENCE_K))
            .max(DENSITY_FLOOR);
        mass += m;
        volume += m / hot;
    }
    if volume > 0.0 {
        mass / volume
    } else {
        DENSITY_FLOOR
    }
}

/// The liquid's surface tension at the tray [N/m] — a game value (§78.4.1):
/// each cut `SURFACE_TENSION_AT_REFERENCE` at 15 °C, falling to zero at a
/// critical temperature `1.15·Tb + 120 K` (n-hexane 513 against 507.6 K,
/// n-decane 634 against 617.7 K) with the corresponding-states exponent 11/9
/// (Brock & Bird, AIChE J. 1 (1955) 174), mole-fraction averaged, floored.
fn surface_tension(slate: &Slate, liquid: &[f64], temperature: f64) -> f64 {
    let (mut sigma, mut moles) = (0.0, 0.0);
    for (c, x) in liquid.iter().enumerate() {
        let critical = 1.15 * slate.get(c).tb.value() + 120.0;
        let reduced = ((critical - temperature) / (critical - DENSITY_REFERENCE_K)).max(0.0);
        sigma += x * SURFACE_TENSION_AT_REFERENCE * reduced.powf(11.0 / 9.0);
        moles += x;
    }
    if moles > 0.0 {
        (sigma / moles).max(SURFACE_TENSION_FLOOR)
    } else {
        SURFACE_TENSION_FLOOR
    }
}

/// The section a stage's tray is in, by the loader's ordering (sections top
/// down, contiguous).
fn section_of(trays: &[TraySection], stage: u32) -> Option<&TraySection> {
    trays.iter().find(|s| s.from <= stage && stage <= s.to)
}

/// A converged column's loads on its trays, and each tray's hydraulics (§78.3).
/// Tray `j` (zero-based, every stage but the reboiler) carries the vapour
/// rising from stage `j + 1` — its oil at `y = K·x` and its water vapour, an
/// ideal gas at that stage's temperature and the column's pressure — and the
/// liquid leaving stage `j`: down, drawn and into a loop, its oil and its free
/// water.
#[allow(clippy::too_many_arguments)]
pub(crate) fn column_trays(
    slate: &Slate,
    spec: &CascadeSpec,
    pressure: Pascal,
    temperatures: &[f64],
    liquid: &[Vec<f64>],
    k: &[Vec<f64>],
    flows: &Flows,
    water_vapour: &[f64],
    free_water: &[f64],
) -> Result<Vec<TrayHydraulics>, SimError> {
    let water_molar_mass = slate
        .free_water()
        .map_or(0.0, |index| slate.get(index).molar_mass.value());
    let molar_mass = |c: usize| slate.get(c).molar_mass.value();
    let stages = temperatures.len();
    let mut out = Vec::with_capacity(stages.saturating_sub(1));
    for j in 0..stages.saturating_sub(1) {
        let stage = j as u32 + 1;
        let section = section_of(&spec.trays, stage).ok_or_else(|| {
            SimError::Scenario(format!(
                "tray hydraulics: stage {stage} is in no tray section (docs/DESIGN.md §78.2)."
            ))
        })?;
        let below = j + 1;
        // The vapour from below: its oil's mean molar mass from y = K·x.
        let (mut y_sum, mut y_mass) = (0.0, 0.0);
        for (c, x) in liquid[below].iter().enumerate() {
            let y = k[below][c] * x;
            y_sum += y;
            y_mass += y * molar_mass(c);
        }
        let oil_vapour = flows.vapour[below];
        let water = water_vapour.get(below).copied().unwrap_or(0.0);
        let oil_molar_mass = if y_sum > 0.0 { y_mass / y_sum } else { 0.0 };
        let vapour_mass = oil_vapour * oil_molar_mass + water * water_molar_mass;
        let vapour_moles = oil_vapour + water;
        let mean_molar_mass = if vapour_moles > 0.0 {
            vapour_mass / vapour_moles
        } else {
            oil_molar_mass
        };
        // Ideal gas: ρV = P·M̄/(R·T).
        let vapour_density = pressure.value() * mean_molar_mass / (R_GAS * temperatures[below]);
        // The liquid leaving the tray, its oil and its free water.
        let liquid_molar_mass: f64 = liquid[j]
            .iter()
            .enumerate()
            .map(|(c, x)| x * molar_mass(c))
            .sum();
        let leaving = flows.liquid_down[j] + flows.liquid_drawn[j] + flows.circulation_out[j];
        let liquid_mass = leaving * liquid_molar_mass
            + free_water.get(j).copied().unwrap_or(0.0) * water_molar_mass;
        let load = TrayLoad {
            vapour: KgPerSec(vapour_mass),
            vapour_density: KgPerM3(vapour_density),
            liquid: KgPerSec(liquid_mass),
            liquid_density: KgPerM3(liquid_density(slate, &liquid[j], temperatures[j])),
            surface_tension: NewtonPerMeter(surface_tension(slate, &liquid[j], temperatures[j])),
        };
        if !(vapour_density.is_finite() && vapour_density > 0.0) {
            return Err(SimError::Scenario(format!(
                "tray hydraulics: tray {stage}'s vapour has density {vapour_density} kg/m³."
            )));
        }
        let reading = sieve_tray(&TrayGeometry::of(section), &load).reading(stage);
        let finite = reading.flood_fraction.is_finite()
            && reading.weep_margin.is_finite()
            && reading.pressure_drop.is_finite()
            && reading.downcomer_backup.is_finite();
        if !finite {
            return Err(SimError::Scenario(format!(
                "tray hydraulics: tray {stage} read {reading:?} at loads {load:?}."
            )));
        }
        out.push(reading);
    }
    Ok(out)
}

/// A column's tray sections, checked against the stages and the ranges their
/// correlations hold over (§78.2, §78.4.1). Called from `CascadePlan::new`, so
/// the loader's `StageCascade::validate` refuses a bad file by name.
pub(crate) fn validate(spec: &CascadeSpec) -> Result<(), SimError> {
    if spec.trays.is_empty() {
        return Ok(());
    }
    let refuse = |message: String| Err(SimError::Scenario(format!("tray sections: {message}")));
    let last_tray = spec.stages.saturating_sub(1);
    if last_tray == 0 {
        return refuse(format!(
            "the column has {} stage(s) and the reboiler is no tray; there is nothing to \
             declare trays on.",
            spec.stages
        ));
    }
    // Top down, contiguous, every stage but the reboiler in exactly one.
    let mut next = 1;
    for section in &spec.trays {
        let (from, to) = (section.from, section.to);
        if from != next || to < from {
            return refuse(format!(
                "a section runs from stage {from} to {to}; the sections run top down, each \
                 from the stage after the last one's end (this one from stage {next}), every \
                 stage but the reboiler in exactly one (docs/DESIGN.md §78.2)."
            ));
        }
        if to > last_tray {
            return refuse(format!(
                "a section runs to stage {to}; the trays end at stage {last_tray}, above the \
                 reboiler ({}).",
                spec.stages
            ));
        }
        next = to + 1;
        check_geometry(section)
            .or_else(|reason| refuse(format!("the section from stage {from} to {to}: {reason}")))?;
    }
    if next != last_tray + 1 {
        return refuse(format!(
            "the sections end at stage {}; every stage down to {last_tray} (the last above \
             the reboiler) must be in one.",
            next - 1
        ));
    }
    for declared in &spec.loops {
        let draw = section_of(&spec.trays, declared.draw_stage).map(|s| s.from);
        let back = section_of(&spec.trays, declared.return_stage).map(|s| s.from);
        if draw != back {
            return refuse(format!(
                "the pumparound loop drawn from stage {} returns to stage {} in another \
                 section; a loop's two stages are one section (docs/DESIGN.md §78.2).",
                declared.draw_stage, declared.return_stage
            ));
        }
    }
    Ok(())
}

/// One section's numbers inside the ranges their correlations were checked over,
/// or why not.
fn check_geometry(section: &TraySection) -> Result<(), String> {
    let positive = |name: &str, value: f64| {
        if value.is_finite() && value > 0.0 {
            Ok(())
        } else {
            Err(format!(
                "{name} is {value}, which must be finite and positive."
            ))
        }
    };
    positive("diameter_m", section.diameter.value())?;
    positive("spacing_m", section.spacing.value())?;
    positive("weir_height_m", section.weir_height.value())?;
    positive("hole_diameter_m", section.hole_diameter.value())?;
    positive("clearance_m", section.clearance.value())?;
    let spacing = section.spacing.value();
    let (low, high) = SPACING_RANGE_M;
    if !(low..=high).contains(&spacing) {
        return Err(format!(
            "spacing_m = {spacing} is outside {low}–{high} m, the tray spacings Fair's \
             flooding chart has curves for (Perry 8th ed., Fig. 14-31)."
        ));
    }
    let weir = section.weir_height.value();
    if weir < MIN_WEIR_M {
        return Err(format!(
            "weir_height_m = {weir} is under {MIN_WEIR_M} m: the weep point's correlation \
             was checked on weirs of 40–90 mm, the common ones above atmospheric pressure \
             (docs/DESIGN.md §78.4.1)."
        ));
    }
    if weir > MAX_WEIR_OF_SPACING * spacing {
        return Err(format!(
            "weir_height_m = {weir} is over 15 % of the tray spacing ({spacing} m); Fair's \
             flooding chart holds only below (Perry 8th ed., p. 14-38)."
        ));
    }
    let hole = section.hole_diameter.value();
    if hole > MAX_HOLE_M {
        return Err(format!(
            "hole_diameter_m = {hole} is over {MAX_HOLE_M} m; Fair's flooding chart holds \
             for perforations of 13 mm or less (Perry 8th ed., p. 14-38)."
        ));
    }
    let share = section.hole_area_share;
    let (low, high) = HOLE_SHARE_RANGE;
    if !(share.is_finite() && (low..=high).contains(&share)) {
        return Err(format!(
            "hole_area_share = {share} is outside {low}–{high}, the hole shares the \
             dry plate's orifice coefficient is published for (Sinnott Fig. 11.34, \
             docs/DESIGN.md §78.4.1)."
        ));
    }
    let downcomer = section.downcomer_share;
    if !(downcomer.is_finite() && downcomer > 0.0 && downcomer < 0.5) {
        return Err(format!(
            "downcomer_share = {downcomer} must be above 0 and under 0.5: two downcomers \
             of half the column leave the tray no active area."
        ));
    }
    let clearance = section.clearance.value();
    if clearance >= weir {
        return Err(format!(
            "clearance_m = {clearance} is not under the weir ({weir} m): the downcomer \
             would be unsealed, vapour up it (Sinnott sets it 5–10 mm under)."
        ));
    }
    if !(1..=MAX_PASSES).contains(&section.passes) {
        return Err(format!(
            "passes = {} is outside 1–{MAX_PASSES}.",
            section.passes
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn section(passes: u32) -> TraySection {
        TraySection {
            from: 1,
            to: 1,
            diameter: Meter(6.0),
            spacing: Meter(0.61),
            weir_height: Meter(0.05),
            hole_diameter: Meter(0.005),
            hole_area_share: 0.10,
            downcomer_share: 0.12,
            clearance: Meter(0.04),
            passes,
        }
    }

    fn load(vapour: f64, liquid: f64) -> TrayLoad {
        TrayLoad {
            vapour: KgPerSec(vapour),
            vapour_density: KgPerM3(5.0),
            liquid: KgPerSec(liquid),
            liquid_density: KgPerM3(700.0),
            surface_tension: NewtonPerMeter(0.015),
        }
    }

    #[test]
    fn the_weir_is_the_downcomers_chord() {
        // Sinnott Fig. 11.31: 0.73 at a 10 % downcomer, 0.76–0.77 at 12 %.
        assert!((chord_of_segment(0.10) - 0.7266).abs() < 1e-3);
        assert!((chord_of_segment(0.12) - 0.7642).abs() < 1e-3);
        // A half-circle's chord is its diameter.
        assert!((chord_of_segment(0.5) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn the_orifice_coefficient_runs_between_its_two_published_readings() {
        assert!((orifice_coefficient(0.10) - 0.84).abs() < 1e-12);
        assert!((orifice_coefficient(0.15) - 0.88).abs() < 1e-12);
        assert!((orifice_coefficient(0.125) - 0.86).abs() < 1e-12);
    }

    #[test]
    fn a_multipass_tray_divides_its_liquid_and_its_flow_parameter() {
        // One pass is the single-pass tray; four quarter the weir's load and
        // the flow parameter, and leave the vapour's side alone.
        let one = sieve_tray(&TrayGeometry::of(&section(1)), &load(60.0, 120.0));
        let four = sieve_tray(&TrayGeometry::of(&section(4)), &load(60.0, 120.0));
        let quarter = sieve_tray(&TrayGeometry::of(&section(1)), &load(60.0, 30.0));
        assert!((four.flow_parameter - one.flow_parameter / 4.0).abs() < 1e-12);
        assert_eq!(four.weir_crest, quarter.weir_crest);
        assert_eq!(four.downcomer_loss, quarter.downcomer_loss);
        assert_eq!(four.hole_velocity, one.hole_velocity);
        assert_eq!(four.dry_head, one.dry_head);
        // A smaller flow parameter is a larger flood capacity.
        assert!(four.flood_velocity.value() > one.flood_velocity.value());
    }

    #[test]
    fn a_tray_with_no_vapour_reads_finite_numbers() {
        let geometry = TrayGeometry::of(&section(4));
        for (vapour, liquid) in [(0.0, 50.0), (0.0, 0.0), (40.0, 0.0)] {
            let reading = sieve_tray(&geometry, &load(vapour, liquid)).reading(3);
            assert!(reading.flood_fraction.is_finite(), "{reading:?}");
            assert!(reading.weep_margin.is_finite(), "{reading:?}");
            assert!(reading.pressure_drop.is_finite(), "{reading:?}");
            assert!(reading.downcomer_backup.is_finite(), "{reading:?}");
            if vapour == 0.0 {
                assert_eq!(reading.flood_fraction, 0.0);
                assert!(reading.weeping() && !reading.flooding());
            }
        }
    }
}
