//! The free-water flash against a hand calculation (M62.2, docs/DESIGN.md
//! §67.2).
//!
//! Water never dissolves in the oil, nor the oil in water, so a stream holding
//! free water is in one of four states: no vapour, three phases (oil liquid,
//! free water, vapour), the oil all vapour over free water, or every drop of
//! water vapour. The reference is `tools/steptime/prototypes/m62_2/
//! free_water_flash.py`, which reaches the same states by a different route —
//! a bisection on the vapour's moles with the oil liquid found by a nested
//! bisection, where the engine solves the oil's Rachford–Rice at its partial
//! pressure `P − P_w` — so the two share only the physics: Trouton oil cuts at
//! `C = 88`, water's vapour pressure by Clausius–Clapeyron on its declared
//! 40 650 J/mol (M62.1), Dalton for the two liquids.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::traits::{LineFlashModel, ThermoModel};
use refinery_core::units::{JPerKgK, JPerMol, Kelvin, KgPerM3, KgPerMol, Pascal};
use refinery_solvers::{
    bubble_temperature, ConstantEnthalpy, EquilibriumLineFlash, MoleFractions, TroutonThermo,
};

/// The script's pressure [Pa].
const PRESSURE: f64 = 1.8e5;

fn cut(name: &str, tb_c: f64, molar_mass: f64) -> PseudoComponent {
    PseudoComponent {
        name: name.into(),
        tb: Kelvin(tb_c + 273.15),
        molar_mass: KgPerMol(molar_mass),
        density: Some(KgPerM3(750.0)),
        cp: JPerKgK(2100.0),
        cp_shape: None,
        phase: Phase::Liquid,
        dh_vap: None,
        cp_vapour: None,
        free_water: false,
    }
}

fn water() -> PseudoComponent {
    PseudoComponent {
        name: "water".into(),
        tb: Kelvin(373.15),
        molar_mass: KgPerMol(0.018),
        density: Some(KgPerM3(998.0)),
        cp: JPerKgK(4184.0),
        cp_shape: None,
        phase: Phase::Liquid,
        dh_vap: Some(JPerMol(40_650.0)),
        cp_vapour: Some(JPerKgK(2000.0)),
        free_water: true,
    }
}

/// The script's heavy naphtha (one cut) over water.
fn heavy_naphtha() -> Slate {
    Slate::new(vec![cut("heavy_naphtha", 150.0, 0.130), water()]).unwrap()
}

/// The script's three-cut naphtha over water.
fn three_cut() -> Slate {
    Slate::new(vec![
        cut("light_naphtha", 70.0, 0.095),
        cut("heavy_naphtha", 150.0, 0.130),
        cut("kerosene", 220.0, 0.165),
        water(),
    ])
    .unwrap()
}

/// A molar feed as the mass fractions the engine carries.
fn mass_of(slate: &Slate, moles: &[f64]) -> Composition {
    MoleFractions::from_amounts(moles)
        .unwrap()
        .to_mass(slate)
        .unwrap()
}

/// The vapour's mass share of the isothermal flash at `(T, P)` — the supply's
/// flash, which is the line's `flash_on_the_line` read in mass.
fn vapour_mass_share(slate: &Slate, feed: &Composition, celsius: f64) -> f64 {
    EquilibriumLineFlash
        .supply(
            slate,
            feed,
            Kelvin(celsius + 273.15),
            Pascal(PRESSURE),
            &TroutonThermo::new(),
            &ConstantEnthalpy,
        )
        .unwrap()
        .map_or(0.0, |share| share.mass_fraction)
}

fn check(slate: &Slate, moles: &[f64], cases: &[(f64, f64, &str)]) {
    let feed = mass_of(slate, moles);
    for &(celsius, expected, state) in cases {
        let got = vapour_mass_share(slate, &feed, celsius);
        assert!(
            (got - expected).abs() <= 1e-9 + 1e-8 * expected,
            "{celsius} °C ({state}): vapour {got} of the mass against the script's {expected}"
        );
    }
}

/// Every state the script prints for a one-cut and a three-cut naphtha, each
/// with free water, and a water-rich stream whose oil is all vapour over the
/// free water — the state a condenser's water lives in (M62.5).
#[test]
fn the_free_water_flash_matches_the_hand_calculation_in_every_state() {
    check(
        &heavy_naphtha(),
        &[0.98, 0.02],
        &[
            (100.0, 0.0, "no vapour"),
            (112.0, 0.007_845_897_0, "water all vapour"),
            (120.0, 0.009_638_022_8, "water all vapour"),
            (150.0, 0.029_027_257_4, "water all vapour"),
            (170.0, 0.202_707_116_2, "water all vapour"),
        ],
    );
    check(
        &heavy_naphtha(),
        &[0.10, 0.90],
        &[
            (105.0, 0.0, "no vapour"),
            (112.0, 0.785_685_502_1, "oil all vapour"),
            (115.0, 1.0, "all vapour"),
            (125.0, 1.0, "all vapour"),
        ],
    );
    let third = 0.95 / 3.0;
    check(
        &three_cut(),
        &[third, third, third, 0.05],
        &[
            (97.0, 0.0, "no vapour"),
            (98.5, 0.007_531_258_6, "three phases"),
            (99.0, 0.028_483_158_4, "three phases"),
            (99.3, 0.040_143_738_6, "water all vapour"),
            (110.0, 0.061_121_610_9, "water all vapour"),
            (130.0, 0.145_841_621_1, "water all vapour"),
            (200.0, 0.906_926_116_2, "water all vapour"),
        ],
    );
}

/// Oil over free water boils where the two vapour pressures add up to the
/// pressure (Dalton): the heavy naphtha at 110.65 °C where alone it boils at
/// 174.3 °C (M62.0's finding 11), the three-cut naphtha at 98.32 °C, pure water
/// at 117.12 °C — and the bubble PRESSURE at each is the pressure, so the
/// boil-off's cheap test, the cavitation readout and the line flash agree on
/// what boiling is.
#[test]
fn the_wet_bubble_point_adds_the_two_liquids_vapour_pressures() {
    let thermo = TroutonThermo::new();
    let third = 0.95 / 3.0;
    let pure_water = {
        let s = heavy_naphtha();
        (s, vec![0.0, 1.0])
    };
    for ((slate, moles), expected_c) in [
        ((heavy_naphtha(), vec![0.98, 0.02]), 110.652_663),
        ((three_cut(), vec![third, third, third, 0.05]), 98.322_825),
        (pure_water, 117.115_960),
    ] {
        let x = MoleFractions::from_amounts(&moles).unwrap();
        let bubble = bubble_temperature(&slate, &thermo, &x, Pascal(PRESSURE), "a test")
            .unwrap()
            .value()
            - 273.15;
        assert!(
            (bubble - expected_c).abs() < 1e-5,
            "{moles:?}: bubbles at {bubble} °C against the script's {expected_c}"
        );
        let pressure = thermo
            .bubble_pressure(&slate, &mass_of(&slate, &moles), Kelvin(bubble + 273.15))
            .unwrap()
            .value();
        assert!(
            (pressure / PRESSURE - 1.0).abs() < 1e-9,
            "{moles:?}: bubble pressure {pressure} Pa at its own bubble point"
        );
    }
    // Dry, the same cut boils where it always did: the free-water path runs
    // only where water stands.
    let dry = MoleFractions::from_amounts(&[1.0, 0.0]).unwrap();
    let dry_bubble =
        bubble_temperature(&heavy_naphtha(), &thermo, &dry, Pascal(PRESSURE), "a test")
            .unwrap()
            .value()
            - 273.15;
    assert!(
        (dry_bubble - 174.3).abs() < 0.05,
        "the dry cut boils at {dry_bubble} °C"
    );
}

/// A trace of water at or below the books' rounding (`FREE_WATER_TRACE`) is
/// no free water: a holdup a boil-off has emptied of water to a few ULP boils
/// where its dry oil does, not 64 K lower where water's whole vapour pressure
/// would put it. Just above the trace, free water stands and acts in full.
#[test]
fn a_rounding_trace_of_water_is_no_free_water() {
    let thermo = TroutonThermo::new();
    let slate = heavy_naphtha();
    let bubble = |water: f64| {
        let x = MoleFractions::from_amounts(&[1.0 - water, water]).unwrap();
        bubble_temperature(&slate, &thermo, &x, Pascal(PRESSURE), "a test")
            .unwrap()
            .value()
            - 273.15
    };
    let (dry, trace, wet) = (bubble(0.0), bubble(1e-12), bubble(1e-6));
    assert!(
        (trace - dry).abs() < 1e-6,
        "a trace boils at {trace} °C, dry at {dry} °C"
    );
    assert!(
        dry - wet > 60.0,
        "free water boils at {wet} °C, dry at {dry} °C"
    );
}
