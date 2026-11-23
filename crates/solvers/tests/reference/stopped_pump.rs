//! Reference case for a stopped centrifugal pump (C35, docs/DESIGN.md §84),
//! against the radial pump's complete characteristics.
//!
//! Source: Chaudhry, *Applied Hydraulic Transients*, 3rd ed. (2014), Eq. 4-5
//! (`θ = tan⁻¹(α/v)`) and Table A-1, after Brown & Rogers (1980), the radial
//! pump of specific speed `Ns = 0.46` (SI). At zero speed (`α = 0`) the head
//! is `h = WH(θ)·v²` of the rated head `H_R`, `v = Q/Q_R`:
//!
//! - `θ = 0°`, flow in the pumping direction: `WH = −0.55`, a LOSS of
//!   `0.55·H_R` at the rated flow;
//! - `θ = 180°`, flow against it: `WH = +0.71`, a loss of `0.71·H_R`;
//! - `θ = 90°`, shutoff at rated speed: `WH = 1.35`, so `h0 = 1.35·H_R`.
//!
//! The engine's pump has a curve `H = h0 − a·Q|Q|` and no rated point, so the
//! rated point is put where the table puts it, from `h0` alone: `H_R =
//! h0/1.35`, and `Q_R` where the curve gives `H_R`, `a·Q_R² = h0 − H_R`.
//! The numbers below are restated by hand, not read from the workspace's
//! constants, so a wrong table value, a swapped direction or a dropped `ρ·g`
//! fails here.

use refinery_solvers::elements::QuadraticBranch;

const RHO: f64 = 750.0; // kg/m³, a naphtha
const G: f64 = 9.806_65; // m/s²
const H0: f64 = 40.0; // m, shutoff head (the crude units' product pump)
const A: f64 = 2000.0; // s²/m⁵

/// The rated point the table implies: `H_R = h0/1.35`, `Q_R = √((h0 − H_R)/a)`.
fn rated() -> (f64, f64) {
    let head_rated = H0 / 1.35; // 29.630 m
    let flow_rated = ((H0 - head_rated) / A).sqrt(); // 0.072 008 m³/s
    (head_rated, flow_rated)
}

#[test]
fn a_locked_rotor_loses_055_of_rated_head_at_rated_flow_forward() {
    let (head_rated, flow_rated) = rated();
    // The drop across it that drives the rated flow forward: 0.55·H_R of head.
    let dp = RHO * G * 0.55 * head_rated; // 119 857 Pa
    let pump = QuadraticBranch::stopped_pump(A, RHO, G, true);
    let q = pump.flow(dp, 1e-9);
    assert!(
        (q - flow_rated).abs() < 1e-6 * flow_rated,
        "{q} m³/s, the table's {flow_rated}"
    );
    assert_eq!(pump.beta, 0.0, "a stopped pump adds no head");
}

#[test]
fn a_locked_rotor_loses_071_of_rated_head_at_rated_flow_reversed() {
    let (head_rated, flow_rated) = rated();
    let dp = RHO * G * 0.71 * head_rated;
    let pump = QuadraticBranch::stopped_pump(A, RHO, G, false);
    let q = pump.flow(dp, 1e-9);
    assert!(
        (q - flow_rated).abs() < 1e-6 * flow_rated,
        "{q} m³/s, the table's {flow_rated}"
    );
}

/// What the change buys, against the old model (the running curve with its
/// head taken away, `a·Q|Q|`): the same drop drives `√(0.35/0.55)` ≈ 0.80
/// of the flow forward and `√(0.35/0.71)` ≈ 0.70 reversed — a restriction,
/// not a shut valve.
#[test]
fn a_stopped_pump_passes_about_four_fifths_of_what_the_bare_curve_did() {
    let dp = 50_000.0; // Pa
    let old = QuadraticBranch::pump(0.0, A, RHO, G).flow(dp, 1e-9);
    let forward = QuadraticBranch::stopped_pump(A, RHO, G, true).flow(dp, 1e-9);
    let reverse = QuadraticBranch::stopped_pump(A, RHO, G, false).flow(dp, 1e-9);
    assert!((forward / old - (0.35_f64 / 0.55).sqrt()).abs() < 1e-9);
    assert!((reverse / old - (0.35_f64 / 0.71).sqrt()).abs() < 1e-9);
    assert!((forward / old - 0.7977).abs() < 1e-4);
}
