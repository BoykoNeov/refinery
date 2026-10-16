//! Water's own properties against steam tables (M62.1, docs/DESIGN.md §67.1).
//!
//! A slate member may declare its latent heat at the normal boiling point
//! (`dh_vap`) and its vapour's heat capacity (`cp_vapour`). Under
//! `TroutonThermo` the first replaces `C·tb` in the vapour pressure AND the
//! latent heat; the second moves the latent heat with temperature by
//! Kirchhoff's law, so a vapour's enthalpy rises along the vapour's capacity.
//!
//! Reference values: saturation pressures and the enthalpy of superheated steam
//! from the IAPWS-IF97 steam tables (recalled standard values, as tabulated in
//! e.g. Rogers & Mayhew, *Thermodynamic and Transport Properties of Fluids*):
//! P_sat 1.01418 bar at 100 °C, 4.7617 bar at 150 °C, 15.549 bar at 200 °C;
//! h = 3263.9 kJ/kg at 10 bar and 400 °C on the 0 °C saturated-liquid datum
//! (the engine's `T_REF`); λ = 2256.5 kJ/kg at 100 °C, 1940.7 kJ/kg at 200 °C.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::energy::T_REF;
use refinery_core::traits::{LineFlashModel, ThermoModel};
use refinery_core::units::{JPerKgK, JPerMol, Kelvin, KgPerM3, KgPerMol, Pascal, P_ATM, R_GAS};
use refinery_solvers::{ConstantEnthalpy, EquilibriumLineFlash, TroutonThermo};

/// Water's latent heat at 100 °C [J/mol]: 2 256.5 kJ/kg × 18.015 g/mol.
const WATER_DH_VAP: f64 = 40_650.0;
const WATER_MOLAR_MASS: f64 = 0.018015;

fn water(dh_vap: Option<f64>, cp_vapour: Option<f64>) -> PseudoComponent {
    PseudoComponent {
        name: "water".into(),
        tb: Kelvin(373.15),
        molar_mass: KgPerMol(WATER_MOLAR_MASS),
        density: Some(KgPerM3(998.0)),
        cp: JPerKgK(4184.0),
        cp_shape: None,
        phase: Phase::Liquid,
        dh_vap: dh_vap.map(JPerMol),
        cp_vapour: cp_vapour.map(JPerKgK),
        free_water: false,
    }
}

fn slate(component: PseudoComponent) -> Slate {
    Slate::new(vec![component]).unwrap()
}

fn psat(thermo: &TroutonThermo, slate: &Slate, celsius: f64) -> f64 {
    // `K = Psat/P` is exact, so a K at the anchor pressure times that pressure
    // is the vapour pressure (the envelope gate's route, `vapour_pressure.rs`).
    thermo
        .k_value(slate, 0, Kelvin(celsius + 273.15), P_ATM)
        .unwrap()
        * P_ATM.value()
}

#[test]
fn a_declared_latent_heat_gives_water_its_steam_table_vapour_pressure() {
    let thermo = TroutonThermo::new();
    let declared = slate(water(Some(WATER_DH_VAP), None));
    for (celsius, table_bar, tolerance) in [
        (100.0, 1.01418, 0.002),
        (150.0, 4.7617, 0.002),
        (200.0, 15.549, 0.05),
    ] {
        let got = psat(&thermo, &declared, celsius) / 1e5;
        assert!(
            (got / table_bar - 1.0).abs() < tolerance,
            "{celsius} °C: {got} bar against the table's {table_bar}"
        );
    }
    // And Trouton's rule, which the key replaces, is far off for water — the
    // reason the key exists (TroutonThermo's own note: ~60 % high at 50 °C).
    let trouton = slate(water(None, None));
    let at_150 = psat(&thermo, &trouton, 150.0) / 1e5;
    assert!(
        at_150 / 4.7617 - 1.0 < -0.15,
        "Trouton gives {at_150} bar at 150 °C"
    );
}

#[test]
fn the_declared_latent_heat_is_one_number_seen_twice() {
    // The exact identities, independent of the steam tables: K = 1 at tb and
    // P_atm, the latent heat at tb is the declared number, and the vapour
    // pressure's slope is Clausius–Clapeyron's on it, d ln K / d(1/T) = −λ/R.
    let thermo = TroutonThermo::new();
    let s = slate(water(Some(WATER_DH_VAP), None));
    assert_eq!(thermo.k_value(&s, 0, Kelvin(373.15), P_ATM).unwrap(), 1.0);
    assert_eq!(
        thermo.dh_vap(&s, 0, Kelvin(373.15)).unwrap().value(),
        WATER_DH_VAP
    );
    // Without a vapour heat capacity the latent heat is constant.
    assert_eq!(
        thermo.dh_vap(&s, 0, Kelvin(473.15)).unwrap().value(),
        WATER_DH_VAP
    );
    for t in [330.0, 420.0, 520.0] {
        let step = 1.0e-3;
        let ln_k = |t: f64| {
            thermo
                .k_value(&s, 0, Kelvin(t), Pascal(2.0e5))
                .unwrap()
                .ln()
        };
        let slope = (ln_k(t + step) - ln_k(t - step)) / (1.0 / (t + step) - 1.0 / (t - step));
        assert!(
            (slope / (-WATER_DH_VAP / R_GAS) - 1.0).abs() < 1e-6,
            "at {t} K the slope {slope} against −λ/R = {}",
            -WATER_DH_VAP / R_GAS
        );
    }
}

#[test]
fn a_vapour_heat_capacity_moves_the_latent_heat_by_kirchhoffs_law() {
    let thermo = TroutonThermo::new();
    let s = slate(water(Some(WATER_DH_VAP), Some(2000.0)));
    let per_kg = |celsius: f64| {
        thermo
            .dh_vap(&s, 0, Kelvin(celsius + 273.15))
            .unwrap()
            .value()
            / WATER_MOLAR_MASS
    };
    // At tb the declared number, exactly.
    assert_eq!(
        thermo.dh_vap(&s, 0, Kelvin(373.15)).unwrap().value(),
        WATER_DH_VAP
    );
    // dλ/dT = cp_vapour − cp, per kilogram here: (2000 − 4184) J/(kg·K).
    let slope = (per_kg(250.0) - per_kg(150.0)) / 100.0;
    assert!((slope - (2000.0 - 4184.0)).abs() < 1e-6, "slope {slope}");
    // Against the table at 200 °C: Kirchhoff on constant capacities gives
    // 2 038 kJ/kg where the table has 1 940.7 — the stated 5 % (§67.1).
    let at_200 = per_kg(200.0) / 1e3;
    assert!(
        (at_200 / 1940.7 - 1.0).abs() < 0.06,
        "λ at 200 °C is {at_200} kJ/kg"
    );
    // The vapour pressure does not move with the vapour heat capacity: the
    // K-value keeps the constant latent heat (the stated inconsistency).
    let without = slate(water(Some(WATER_DH_VAP), None));
    for celsius in [120.0, 180.0] {
        assert_eq!(
            psat(&thermo, &s, celsius).to_bits(),
            psat(&thermo, &without, celsius).to_bits()
        );
    }
}

#[test]
fn a_superheated_steam_supply_carries_the_steam_tables_heat() {
    // 10 bar and 400 °C: superheated, all vapour. Its enthalpy on the engine's
    // datum is h_liquid(T) + latent, latent = λ(T) per kilogram of vapour.
    let thermo = TroutonThermo::new();
    let flash = EquilibriumLineFlash;
    let enthalpy = ConstantEnthalpy;
    let table = 3263.9e3;
    let enthalpy_of = |component: PseudoComponent| {
        let s = slate(component);
        let steam = Composition::pure(1, 0);
        let t = Kelvin(400.0 + 273.15);
        let share = flash
            .supply(&s, &steam, t, Pascal(10.0e5), &thermo, &enthalpy)
            .unwrap()
            .expect("400 °C at 10 bar is all vapour");
        assert_eq!(share.mass_fraction, 1.0);
        4184.0 * (t.value() - T_REF.value()) + share.latent.value()
    };
    let with_vapour_cp = enthalpy_of(water(Some(WATER_DH_VAP), Some(2000.0)));
    assert!(
        (with_vapour_cp / table - 1.0).abs() < 0.005,
        "{with_vapour_cp} J/kg against the table's {table}"
    );
    // On the liquid's capacity alone the superheat is counted at 4 184 J/(kg·K)
    // and the supply carries about a fifth too much — what the key fixes.
    let liquid_cp_only = enthalpy_of(water(Some(WATER_DH_VAP), None));
    assert!(
        liquid_cp_only / table - 1.0 > 0.15,
        "{liquid_cp_only} J/kg against the table's {table}"
    );
}
