//! Reference case for the sieve-tray hydraulics (M73.1, docs/DESIGN.md
//! §78.4.1), against published worked examples with every input stated.
//!
//! **The hydraulics: NPTEL's sieve tray**, *Chemical Engineering Design II*,
//! Module 7, Lectures 4–5 (IIT; archive.nptel.ac.in/content/storage2/courses/
//! 103103027/pdf/mod7.pdf, pp. 18–26): an acetone–water column designed by
//! Sinnott's method (*Coulson & Richardson* Vol. 6, 3rd ed., pp. 567–576) — the
//! set `refinery_solvers::trays` implements, step for step. Its tray is restated
//! below as the example states it (its own weir length, hole area and `C0`, not
//! re-derived), and every head it prints is compared at its own rounding: half a
//! unit of its last digit, plus 0.5 %.
//!
//! **The flooding fit: three chart readings**, each a worked example's own
//! reading off Fair's chart (Perry's *Chemical Engineers' Handbook*, 8th ed.,
//! Example 9; Dutta, *Principles of Mass Transfer and Separation Processes*,
//! 2007, Example 5.1, at its 18-inch spacing; NPTEL's bottom section), within
//! 2.5 %, and Perry's Example 9 carried to its percent of flood.
//!
//! What it pins and what it cannot, as `orifice.rs` says of itself: a wrong
//! constant, a unit slip, a dropped factor — not the correlations' physical
//! truth, which is their authors'. No number below is read back from the
//! workspace.

use refinery_core::graph::TraySection;
use refinery_core::units::*;
use refinery_solvers::trays::{flood_capacity, sieve_tray, TrayGeometry, TrayLoad};

/// NPTEL's tray (p. 23–24): a 2.4 m column, downcomers 10 % of its area, a
/// 1.752 m weir 40 mm high, 12 mm holes in a plate as thick, the hole area
/// 0.489 m², 600 mm spacing, the apron 10 mm under the weir.
fn nptel_tray() -> TrayGeometry {
    TrayGeometry {
        spacing: Meter(0.600),
        weir_height: Meter(0.040),
        hole_diameter: Meter(0.012),
        net_area: SquareMeter(4.068),
        hole_area: SquareMeter(0.489),
        downcomer_area: SquareMeter(0.452),
        weir_length: Meter(1.752),
        clearance: Meter(0.030),
        // "C0 = 0.88 at Ah/Ap = 15 % and plate thickness/hole diameter = 1".
        orifice_coefficient: 0.88,
        passes: 1,
    }
}

/// A section's loads as the example states them: the vapour by its volume
/// [m³/s] at its density, the liquid by its mass.
fn load(vapour_m3_s: f64, vapour_density: f64, liquid_kg_s: f64, liquid_density: f64) -> TrayLoad {
    TrayLoad {
        vapour: KgPerSec(vapour_m3_s * vapour_density),
        vapour_density: KgPerM3(vapour_density),
        liquid: KgPerSec(liquid_kg_s),
        liquid_density: KgPerM3(liquid_density),
        // 20 mN/m at the top, 58 at the bottom (p. 20–21); the heads do not
        // read it.
        surface_tension: NewtonPerMeter(0.020),
    }
}

/// `got` against a published figure printed to `unit` (1.0 for "122", 0.1 for
/// "12.6"): half a unit of its last digit, plus 0.5 %.
fn assert_published(name: &str, got: f64, published: f64, unit: f64) {
    let tolerance = 0.5 * unit + 0.005 * published.abs();
    assert!(
        (got - published).abs() <= tolerance,
        "{name}: computed {got:.4}, published {published} (±{tolerance:.3})"
    );
}

/// Metres of liquid as the millimetres the example prints.
fn mm(head: Meter) -> f64 {
    head.value() * 1000.0
}

#[test]
fn nptel_top_section() {
    // p. 24–25: the top section at its maximum rates — 6.98 m³/s of vapour at
    // 2.08 kg/m³, 9.02 kg/s of liquid at 744 kg/m³ — and its weep check at the
    // minimum liquid, 70 % of it (6.32 kg/s).
    let tray = nptel_tray();
    let max = sieve_tray(&tray, &load(6.98, 2.08, 9.02, 744.0));
    assert_published("hole velocity [m/s]", max.hole_velocity.value(), 14.3, 0.1);
    assert_published("weir crest [mm]", mm(max.weir_crest), 27.2, 0.1);
    assert_published("dry plate [mm]", mm(max.dry_head), 37.5, 0.1);
    assert_published("residual head [mm]", mm(max.residual_head), 16.8, 0.1);
    assert_published("total head [mm]", mm(max.total_head), 122.0, 1.0);
    assert_published("downcomer loss [mm]", mm(max.downcomer_loss), 8.8, 0.1);
    assert_published("downcomer backup [mm]", mm(max.backup), 198.0, 1.0);
    assert_published("backup limit [mm]", mm(max.backup_limit), 320.0, 1.0);
    let min = sieve_tray(&tray, &load(0.7 * 6.98, 2.08, 6.32, 744.0));
    assert_published("crest at the minimum [mm]", mm(min.weir_crest), 21.5, 0.1);
    // K2 = 30.3 at h_w + h_ow = 61.5 mm read off Sinnott's Fig. 11.30.
    assert_published("weep velocity [m/s]", min.weep_velocity.value(), 12.6, 0.1);
}

#[test]
fn nptel_bottom_section() {
    // p. 24–25: 15.84 m³/s of vapour at 0.693 kg/m³, 6.571 kg/s of liquid at
    // 955 kg/m³; the minimum liquid 4.6 kg/s.
    let tray = nptel_tray();
    let max = sieve_tray(&tray, &load(15.84, 0.693, 6.571, 955.0));
    assert_published("hole velocity [m/s]", max.hole_velocity.value(), 32.4, 0.1);
    assert_published("weir crest [mm]", mm(max.weir_crest), 18.7, 0.1);
    assert_published("dry plate [mm]", mm(max.dry_head), 50.2, 0.1);
    assert_published("residual head [mm]", mm(max.residual_head), 13.0, 1.0);
    assert_published("total head [mm]", mm(max.total_head), 122.0, 1.0);
    assert_published("downcomer loss [mm]", mm(max.downcomer_loss), 2.9, 0.1);
    assert_published("downcomer backup [mm]", mm(max.backup), 184.0, 1.0);
    let min = sieve_tray(&tray, &load(0.7 * 15.84, 0.693, 4.6, 955.0));
    assert_published("crest at the minimum [mm]", mm(min.weir_crest), 14.7, 0.1);
    // K2 = 30.2 at 54.7 mm.
    assert_published("weep velocity [m/s]", min.weep_velocity.value(), 21.8, 0.1);
    // The pressure drop, ρL·g·h_t: 122 mm of a 955 kg/m³ liquid.
    let expected = 955.0 * 9.806_65 * max.total_head.value();
    assert!((max.pressure_drop.value() - expected).abs() < 1e-9);
}

#[test]
fn the_declared_section_comes_to_nptels_tray() {
    // The same tray declared as a section: 2.4 m, 10 % downcomers, the hole
    // share NPTEL's 0.489 m² over its 3.616 m² active area. Its areas and weir
    // come out within 0.5 % of the example's (it took the column's area as
    // 4.52 m², and its weir off Sinnott's chart).
    let section = TraySection {
        from: 1,
        to: 1,
        diameter: Meter(2.4),
        spacing: Meter(0.6),
        weir_height: Meter(0.040),
        hole_diameter: Meter(0.012),
        hole_area_share: 0.489 / 3.616,
        downcomer_share: 0.10,
        clearance: Meter(0.030),
        passes: 1,
    };
    let got = TrayGeometry::of(&section);
    let close = |name: &str, got: f64, published: f64| {
        assert!(
            (got / published - 1.0).abs() < 0.005,
            "{name}: {got} against {published}"
        );
    };
    close("net area", got.net_area.value(), 4.068);
    close("hole area", got.hole_area.value(), 0.489);
    close("downcomer area", got.downcomer_area.value(), 0.452);
    close("weir length", got.weir_length.value(), 1.752);
}

#[test]
fn the_flooding_fit_reads_fairs_chart_as_the_examples_do() {
    // (spacing [m], flow parameter, C_sb read off Fair's chart [m/s]).
    let readings = [
        // Perry 8th ed., Example 9: 500 mm, F_LG = 0.021, C_sbf = 0.095.
        (0.500, 0.021, 0.095),
        // Dutta Example 5.1: 18 in, the top section's (14 100/23 300)·
        // √(2.65/810) = 0.0346, C_sb = 0.285 ft/s.
        (0.4572, 0.0346, 0.285 * 0.3048),
        // NPTEL's bottom section: 600 mm, 0.016, 0.11 m/s.
        (0.600, 0.016, 0.11),
    ];
    for (spacing, flow_parameter, chart) in readings {
        let fit = flood_capacity(Meter(spacing), flow_parameter).value();
        assert!(
            (fit / chart - 1.0).abs() < 0.025,
            "{spacing} m, F_LV {flow_parameter}: fit {fit:.4} against the chart's {chart}"
        );
    }
    // NPTEL's top section reads 0.12 m/s at 600 mm and 0.033, where the fit
    // gives 0.104: the fit falls with the flow parameter where the wide
    // spacings' curves flatten, so it is LOW there — toward flooding sooner
    // (§78.4.1). Pinned so a change of fit is seen.
    let fit = flood_capacity(Meter(0.600), 0.033).value();
    assert!((0.85..0.89).contains(&(fit / 0.12)), "{fit}");
}

#[test]
fn perrys_example_9_runs_at_its_percent_of_flood() {
    // Perry 8th ed., Example 9: an ethylbenzene/styrene tray, 25 500 kg/h of
    // vapour at 0.481 kg/m³, 22 000 kg/h of liquid at 841 kg/m³, 25 mN/m, on
    // 4.66 m² of net area at 500 mm spacing. Perry's own: C = 0.0756 m/s
    // against C_sb·(σ/20)^0.2 = 0.0993 off the chart — 76.1 % of flood.
    let tray = TrayGeometry {
        spacing: Meter(0.500),
        weir_height: Meter(0.038),
        hole_diameter: Meter(0.00476),
        net_area: SquareMeter(4.66),
        hole_area: SquareMeter(0.617),
        downcomer_area: SquareMeter(0.25),
        weir_length: Meter(1.50),
        clearance: Meter(0.028),
        orifice_coefficient: 0.88,
        passes: 1,
    };
    let reading = sieve_tray(
        &tray,
        &TrayLoad {
            vapour: KgPerSec(25_500.0 / 3600.0),
            vapour_density: KgPerM3(0.481),
            liquid: KgPerSec(22_000.0 / 3600.0),
            liquid_density: KgPerM3(841.0),
            surface_tension: NewtonPerMeter(0.025),
        },
    )
    .reading(1);
    let perry = 0.0756 / 0.0993;
    assert!(
        (reading.flood_fraction / perry - 1.0).abs() < 0.02,
        "{} against Perry's {perry}",
        reading.flood_fraction
    );
}
