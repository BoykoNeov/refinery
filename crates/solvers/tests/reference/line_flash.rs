//! Reference cases for the M53 line flash (docs/DESIGN.md §58): a supply above
//! its bubble point flashed where it stands, and a stream re-flashed at a lower
//! pressure with its enthalpy held.
//!
//! The expected numbers come from an independent hand calculation — Rachford–
//! Rice by bisection on Trouton's vapour pressure, `Δh_vap = 88·tb` per mole,
//! constant `cp`, ideal-gas vapour — written in a few lines of Python outside the
//! repo (ROADMAP M53 records it), not from this code. The first case needs no
//! program at all: a PURE component at one atmosphere boils at its normal
//! boiling point exactly, so the energy balance is one division.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::traits::{EnthalpyModel, LineFlashModel};
use refinery_core::units::{JPerKgK, Kelvin, KgPerM3, KgPerMol, Pascal, P_ATM};
use refinery_solvers::{ConstantEnthalpy, EquilibriumLineFlash, NoLineFlash, TroutonThermo};

/// The pump demo's two naphtha cuts (`pump_cavitation_flow_limit.toml`).
fn naphtha() -> Slate {
    let cut = |name: &str, tb_c: f64, molar_mass: f64, density: f64, cp: f64| PseudoComponent {
        name: name.into(),
        tb: Kelvin(tb_c + 273.15),
        molar_mass: KgPerMol(molar_mass),
        density: Some(KgPerM3(density)),
        cp: JPerKgK(cp),
        cp_shape: None,
        phase: Phase::Liquid,
    };
    Slate::new(vec![
        cut("light_naphtha", 80.0, 0.100, 680.0, 2200.0),
        cut("heavy_naphtha", 150.0, 0.130, 750.0, 2100.0),
    ])
    .unwrap()
}

fn blend() -> Composition {
    Composition::from_weights(&[0.7, 0.3]).unwrap()
}

fn light_only() -> Composition {
    Composition::pure(2, 0)
}

/// Relative closeness, for numbers whose scale differs by orders.
fn close(actual: f64, expected: f64, rel: f64) -> bool {
    (actual - expected).abs() <= rel * expected.abs().max(1e-300)
}

/// **THE HAND CALCULATION, a pure component.** Light naphtha at one atmosphere
/// boils at `tb = 353.15 K` exactly (Trouton's vapour pressure is `P_ATM` there
/// by construction). Arriving with the enthalpy of liquid 20 K above that:
///
/// ```text
///   λ = 88·353.15 / 0.100      = 310 772 J/kg
///   q = cp·ΔT/λ = 2200·20/310 772 = 0.141 582 896 786 068
/// ```
///
/// and it sits at 353.15 K. This is also the case the bisection cannot answer
/// by itself — a pure component's flash jumps from all liquid to all vapour at
/// one temperature — which is why `q` comes from the energy (§58 fork 2).
#[test]
fn a_pure_cut_flashed_at_one_atmosphere_sits_at_its_boiling_point() {
    let slate = naphtha();
    let settled = EquilibriumLineFlash
        .settle(
            &slate,
            &light_only(),
            Kelvin(373.15),
            P_ATM,
            &TroutonThermo::new(),
            &ConstantEnthalpy,
        )
        .unwrap();
    let vapour = settled.vapour.expect("20 K of superheat boils some of it");
    assert!(
        (settled.temperature.value() - 353.15).abs() < 1e-9,
        "a pure cut boils at its boiling point: {} K",
        settled.temperature.value()
    );
    assert!(
        close(vapour.mass_fraction, 0.141_582_896_786_068, 1e-9),
        "q = cp·ΔT/λ: {}",
        vapour.mass_fraction
    );
    assert!(close(vapour.latent.value(), 2200.0 * 20.0, 1e-9));
    assert_eq!(vapour.liquid_equivalent, Kelvin(373.15));
}

/// **Below the bubble point nothing boils** — and the mix is handed back as it
/// came, not re-derived.
#[test]
fn a_liquid_below_its_bubble_point_is_handed_back_untouched() {
    let slate = naphtha();
    let settled = EquilibriumLineFlash
        .settle(
            &slate,
            &blend(),
            Kelvin(383.15),
            Pascal(2.4e5),
            &TroutonThermo::new(),
            &ConstantEnthalpy,
        )
        .unwrap();
    assert_eq!(settled.temperature, Kelvin(383.15));
    assert_eq!(settled.vapour, None);
}

/// **A supply above its bubble point, flashed where it stands** — isothermal, at
/// its declared 122 °C and 2.4 bar (the M53 measurement's second row). Hand
/// calculation:
///
/// ```text
///   q        = 0.107 835 961 205 522   (10.8% by mass)
///   λ_vapour = 309 078.508 J/kg
///   latent   = q·λ = 33 329.778 J/kg
///   T_le     = T + latent/cp̄ = 395.15 + 33 329.778/2170 = 410.509 345 K
/// ```
#[test]
fn a_supply_above_its_bubble_point_boils_where_it_stands() {
    let slate = naphtha();
    let share = EquilibriumLineFlash
        .supply(
            &slate,
            &blend(),
            Kelvin(395.15),
            Pascal(2.4e5),
            &TroutonThermo::new(),
            &ConstantEnthalpy,
        )
        .unwrap()
        .expect("122 °C at 2.4 bar is above the bubble point");
    assert!(close(share.mass_fraction, 0.107_835_961_205_522, 1e-9));
    assert!(close(share.latent.value(), 33_329.778_001_350_5, 1e-9));
    assert!(close(
        share.liquid_equivalent.value(),
        410.509_344_701_083,
        1e-12
    ));
    // At 120 °C the same supply is liquid (its bubble pressure 2.349 bar).
    let cold = EquilibriumLineFlash
        .supply(
            &slate,
            &blend(),
            Kelvin(393.15),
            Pascal(2.4e5),
            &TroutonThermo::new(),
            &ConstantEnthalpy,
        )
        .unwrap();
    assert_eq!(cold, None);
}

/// **A stream re-flashed at a lower pressure with its enthalpy held** — the
/// valve's case. Liquid-equivalent 125 °C, let down to 1.5 bar. Hand
/// calculation (bisection on `cp̄·(T − T_le) + q(T)·λ(T) = 0` over
/// `[T_bubble, T_le]`):
///
/// ```text
///   T_bubble(1.5 bar) = 375.592 K
///   T                 = 377.105 873 257 K   (it cools 21 K as it boils)
///   q                 = 0.147 723 059 625   (14.8% by mass)
///   ρ_v = 4.8597, ρ_l = 702.307 kg/m³ → ρ = 31.634 26 kg/m³
///   liquid share of the volume = 0.038 389 4
/// ```
#[test]
fn a_stream_let_down_with_its_enthalpy_held_cools_as_it_boils() {
    let slate = naphtha();
    let thermo = TroutonThermo::new();
    let settled = EquilibriumLineFlash
        .settle(
            &slate,
            &blend(),
            Kelvin(398.15),
            Pascal(1.5e5),
            &thermo,
            &ConstantEnthalpy,
        )
        .unwrap();
    let vapour = settled
        .vapour
        .expect("1.5 bar is below its bubble pressure");
    assert!(
        (settled.temperature.value() - 377.105_873_257_097).abs() < 1e-9,
        "{} K",
        settled.temperature.value()
    );
    assert!(close(vapour.mass_fraction, 0.147_723_059_624_648, 1e-9));
    let density = EquilibriumLineFlash
        .density(
            &slate,
            &blend(),
            Kelvin(398.15),
            Pascal(1.5e5),
            &thermo,
            &ConstantEnthalpy,
        )
        .unwrap()
        .expect("two-phase");
    assert!(close(density.mixture.value(), 31.634_258_265_096, 1e-9));
    assert!(close(
        density.liquid_volume_share,
        0.038_389_422_958_520,
        1e-8
    ));
}

/// **The first law, on every settle**: `h(T) + latent = h(T_le)`. Swept rather
/// than sampled, across a let-down from just under the bubble pressure to a
/// quarter of it, on the blend and on the pure cut.
#[test]
fn every_settle_keeps_the_enthalpy_it_was_handed() {
    let slate = naphtha();
    let thermo = TroutonThermo::new();
    for composition in [blend(), light_only()] {
        for step in 0..40 {
            let pressure = Pascal(2.4e5 * (1.0 - 0.75 * f64::from(step) / 39.0));
            let liquid_equivalent = Kelvin(398.15);
            let settled = EquilibriumLineFlash
                .settle(
                    &slate,
                    &composition,
                    liquid_equivalent,
                    pressure,
                    &thermo,
                    &ConstantEnthalpy,
                )
                .unwrap();
            let before = ConstantEnthalpy
                .specific_enthalpy(&slate, &composition, liquid_equivalent)
                .unwrap()
                .value();
            let after = ConstantEnthalpy
                .specific_enthalpy(&slate, &composition, settled.temperature)
                .unwrap()
                .value()
                + settled.vapour.map_or(0.0, |v| v.latent.value());
            assert!(
                (after - before).abs() <= 1e-9 * before.abs(),
                "at {} Pa: {after} J/kg against {before}",
                pressure.value()
            );
            if let Some(v) = settled.vapour {
                assert!(v.mass_fraction > 0.0 && v.mass_fraction <= 1.0);
                assert!(settled.temperature.value() < liquid_equivalent.value());
            }
        }
    }
}

/// **`NoLineFlash` is the pre-M53 engine**: no vapour at a supply, the mix back
/// untouched, no two-phase density.
#[test]
fn no_line_flash_changes_nothing() {
    let slate = naphtha();
    let thermo = TroutonThermo::new();
    let at = Kelvin(398.15);
    let low = Pascal(1.0e5);
    assert_eq!(
        NoLineFlash
            .supply(&slate, &blend(), at, low, &thermo, &ConstantEnthalpy)
            .unwrap(),
        None
    );
    let settled = NoLineFlash
        .settle(&slate, &blend(), at, low, &thermo, &ConstantEnthalpy)
        .unwrap();
    assert_eq!(settled.temperature, at);
    assert_eq!(settled.vapour, None);
    assert_eq!(
        NoLineFlash
            .density(&slate, &blend(), at, low, &thermo, &ConstantEnthalpy)
            .unwrap(),
        None
    );
    assert!(!NoLineFlash.carries_vapour());
    assert!(EquilibriumLineFlash.carries_vapour());
}
