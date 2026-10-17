//! A column with its reboiler held (M64, docs/DESIGN.md §69.1): the bottom
//! stage's energy balance an equation, the reflux ratio the result.
//!
//! The reference is the column itself, solved the other way round. M62.6's
//! steamed test column (`steam_column.rs`: 8 stages, feed on 5, four cuts and
//! 1 % water at 1.8 bar, 0.15 kg/s of steam on the reboiler) is solved at its
//! declared reflux, its reboiler the closure; held at that closure, the same
//! column must come back at the same reflux, temperatures and draws — two
//! formulations of one set of equations meeting at one point. Then the held
//! duty walks down, warm, and the column answers with less reflux and a
//! cooler bottom, its published duty the held one exactly.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::graph::{CascadeSpec, ColumnDraw, NodeId};
use refinery_core::traits::{ColumnPass, ColumnSteam, Separation, SeparationModel};
use refinery_core::units::{JPerKgK, JPerMol, Kelvin, KgPerM3, KgPerMol, KgPerSec, Pascal, Watt};
use refinery_solvers::{HeatBalanceCascade, TroutonThermo};

const FEED_KG_PER_S: f64 = 10.0;
const STEAM_KG_PER_S: f64 = 0.15;
const PRESSURE: f64 = 1.8e5;

fn slate() -> Slate {
    let cut = |name: &str, tb_c: f64, molar_mass: f64, cp: f64| PseudoComponent {
        name: name.into(),
        tb: Kelvin(tb_c + 273.15),
        molar_mass: KgPerMol(molar_mass),
        density: Some(KgPerM3(800.0)),
        cp: JPerKgK(cp),
        cp_shape: None,
        phase: Phase::Liquid,
        dh_vap: None,
        cp_vapour: None,
        free_water: false,
    };
    Slate::new(vec![
        cut("light", 70.0, 0.095, 2200.0),
        cut("naphtha", 150.0, 0.13, 2100.0),
        cut("kerosene", 220.0, 0.17, 2050.0),
        cut("diesel", 300.0, 0.22, 2000.0),
        PseudoComponent {
            name: "water".into(),
            tb: Kelvin(373.15),
            molar_mass: KgPerMol(0.018015),
            density: Some(KgPerM3(998.0)),
            cp: JPerKgK(4184.0),
            cp_shape: None,
            phase: Phase::Liquid,
            dh_vap: Some(JPerMol(40_650.0)),
            cp_vapour: Some(JPerKgK(2000.0)),
            free_water: true,
        },
    ])
    .unwrap()
}

/// Steam at 400 °C as its liquid-equivalent temperature (`steam_column.rs`).
fn steam() -> ColumnSteam {
    let (cp, cp_vapour, molar_mass, temperature_c) = (4184.0, 2000.0, 0.018015, 400.0);
    let latent = 40_650.0 / molar_mass + (cp_vapour - cp) * (temperature_c - 100.0);
    ColumnSteam {
        stage: 8,
        flow: KgPerSec(STEAM_KG_PER_S),
        temperature: Kelvin(273.15 + temperature_c + latent / cp),
    }
}

fn feed() -> Composition {
    Composition::from_weights(&[0.198, 0.297, 0.297, 0.198, 0.01]).unwrap()
}

fn draws() -> Vec<ColumnDraw> {
    vec![
        ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
        ColumnDraw::water_draw(NodeId(2)),
        ColumnDraw::by_stage(NodeId(3), 3, Some(0.25)),
        ColumnDraw::by_stage(NodeId(4), 8, None),
    ]
}

fn spec(reflux_ratio: f64, reboiler_duty: Option<f64>) -> CascadeSpec {
    CascadeSpec {
        stages: 8,
        feed_stage: 5,
        reflux_ratio,
        pumparounds: Vec::new(),
        steam: None,
        reboiler_duty: reboiler_duty.map(Watt),
    }
}

/// One pass at a feed temperature [°C], warm from `seed` when given.
fn separate(
    spec: &CascadeSpec,
    feed_c: f64,
    seed: Option<&Separation>,
) -> Result<Separation, String> {
    let slate = slate();
    let draws = draws();
    HeatBalanceCascade::new()
        .separate(
            &ColumnPass {
                slate: &slate,
                draws: &draws,
                smearing: Kelvin(0.0),
                pressure: Pascal(PRESSURE),
                feed: &feed(),
                feed_flow: KgPerSec(FEED_KG_PER_S),
                temperature: Kelvin(feed_c + 273.15),
                cascade: Some(spec),
                seed: seed.and_then(|s| s.profile.as_ref()),
                steam: Some(steam()),
            },
            &TroutonThermo::new(),
        )
        .map_err(|e| e.to_string())
}

fn reboiler(separation: &Separation) -> f64 {
    separation.reboiler_duty.unwrap().value()
}

fn bottom_c(separation: &Separation) -> f64 {
    separation
        .profile
        .as_ref()
        .unwrap()
        .temperatures
        .last()
        .unwrap()
        .value()
        - 273.15
}

/// **Held at its own closure, the column is the declared one.** Declared at a
/// reflux of 2, the reboiler closes at some duty; held at that duty, with 3
/// declared (now only the seed's), the column comes back at a reflux of 2 —
/// every stage temperature and draw with it — and publishes the held duty.
#[test]
fn held_at_its_own_closure_the_column_is_the_declared_one() {
    let declared = separate(&spec(2.0, None), 230.0, None).unwrap();
    let duty = reboiler(&declared);
    let held = separate(&spec(3.0, Some(duty)), 230.0, None).unwrap();

    let reflux = held.reflux_ratio.unwrap();
    assert!((reflux - 2.0).abs() < 1.0e-6, "reflux {reflux}");
    assert_eq!(reboiler(&held), duty, "a held duty is published as held");
    let (a, b) = (
        declared.profile.as_ref().unwrap(),
        held.profile.as_ref().unwrap(),
    );
    for (j, (x, y)) in a.temperatures.iter().zip(&b.temperatures).enumerate() {
        assert!(
            (x.value() - y.value()).abs() < 1.0e-5,
            "stage {}: {x:?} against {y:?}",
            j + 1
        );
    }
    for (i, (x, y)) in declared.draws.iter().zip(&held.draws).enumerate() {
        assert!(
            (x.split - y.split).abs() < 1.0e-8,
            "draw {i}: {} against {}",
            x.split,
            y.split
        );
    }
    let condenser = |s: &Separation| s.condenser_duty.unwrap().value();
    assert!(
        (condenser(&declared) - condenser(&held)).abs() < 1.0e-6 * condenser(&declared),
        "condenser {} against {}",
        condenser(&declared),
        condenser(&held)
    );
}

/// **Less reboiler, less reflux and a cooler bottom.** Walked warm from the
/// declared column's closure toward zero on a feed hot enough to need none
/// (330 °C), each held duty is published exactly, and the reflux and the
/// bottom both fall as the reboiler does (2.40 → 2.10, 237 → 183 °C) — the
/// steam alone stripping at zero. The declared column there runs on its vapour
/// floor, above its declared reflux; held at its closure it comes back at the
/// reflux it ran at.
#[test]
fn a_held_reboiler_walks_to_zero_and_the_steam_strips_alone() {
    let feed_c = 330.0;
    let start = separate(&spec(2.0, None), feed_c, None).unwrap();
    let top = reboiler(&start);
    assert!(top > 0.0, "the declared column needs a reboiler: {top} W");
    let mut previous = start;
    let mut last = (f64::INFINITY, f64::INFINITY);
    for step in (0..=8).rev() {
        let duty = top * f64::from(step) / 8.0;
        let held = separate(&spec(2.0, Some(duty)), feed_c, Some(&previous))
            .unwrap_or_else(|e| panic!("held at {duty:.4e} W: {e}"));
        assert_eq!(reboiler(&held), duty);
        let (reflux, bottom) = (held.reflux_ratio.unwrap(), bottom_c(&held));
        if step == 8 {
            // At its own closure the held column is the declared one — here
            // above its declared 2, where the floor held it (§66.3).
            let ran = previous.reflux_ratio.unwrap();
            assert!(
                (reflux - ran).abs() < 1.0e-6,
                "reflux {reflux} against {ran}"
            );
            assert!(ran > 2.0 + 1.0e-3, "the declared column's floor: {ran}");
        }
        assert!(
            reflux < last.0 + 1.0e-9 && bottom < last.1 + 1.0e-9,
            "at {duty:.4e} W: reflux {reflux}, bottom {bottom} °C after {last:?}"
        );
        last = (reflux, bottom);
        previous = held;
    }
    assert_eq!(reboiler(&previous), 0.0);
}
