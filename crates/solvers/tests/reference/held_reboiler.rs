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
//!
//! M64.2 (§69.3): the same column held at zero with nothing to start from is
//! started by a continuation and lands where the warm walk does; handed a
//! tick's earlier pass it starts from that instead; a declared column ignores
//! one to the bit; a walk that stops is refused where it stopped.

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
    separate_by(&HeatBalanceCascade::new(), spec, feed_c, seed, None)
}

/// One pass by `model`, handed `seed` as the last tick's answer and `earlier`
/// as this tick's earlier pass (M64.2).
fn separate_by(
    model: &HeatBalanceCascade,
    spec: &CascadeSpec,
    feed_c: f64,
    seed: Option<&Separation>,
    earlier: Option<&Separation>,
) -> Result<Separation, String> {
    let slate = slate();
    let draws = draws();
    model
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
                running_reboiler: None,
                earlier_pass: earlier.and_then(|s| s.profile.as_ref()),
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

/// The 330 °C walk above, warm from the declared column's closure to zero:
/// the column held at zero that the walk lands on.
fn walked_to_zero(feed_c: f64) -> Separation {
    let start = separate(&spec(2.0, None), feed_c, None).unwrap();
    let top = reboiler(&start);
    let mut previous = start;
    for step in (0..=8).rev() {
        let duty = top * f64::from(step) / 8.0;
        previous = separate(&spec(2.0, Some(duty)), feed_c, Some(&previous))
            .unwrap_or_else(|e| panic!("held at {duty:.4e} W: {e}"));
    }
    previous
}

/// **Cold, a held column lands where the warm walk does** (M64.2,
/// docs/DESIGN.md §69.3). With nothing to start from, the column held at zero
/// on its 330 °C feed is started by the continuation — the same column with
/// its reboiler and 1.5 times the feed's latent heat flow of side cooling,
/// walked to zero in ten solves — and comes back as the column the warm walk
/// reached: its stages within 1e-8 K (measured 2e-11), its reflux and draws
/// with them.
#[test]
fn cold_a_held_column_lands_where_the_warm_walk_does() {
    let feed_c = 330.0;
    let warm = walked_to_zero(feed_c);
    let cold = separate(&spec(2.0, Some(0.0)), feed_c, None)
        .unwrap_or_else(|e| panic!("the cold start: {e}"));
    assert_eq!(reboiler(&cold), 0.0);
    let (a, b) = (
        warm.profile.as_ref().unwrap(),
        cold.profile.as_ref().unwrap(),
    );
    for (j, (x, y)) in a.temperatures.iter().zip(&b.temperatures).enumerate() {
        assert!(
            (x.value() - y.value()).abs() < 1.0e-8,
            "stage {}: {x:?} against {y:?}",
            j + 1
        );
    }
    let (x, y) = (warm.reflux_ratio.unwrap(), cold.reflux_ratio.unwrap());
    assert!((x - y).abs() < 1.0e-9, "reflux {x} against {y}");
    for (i, (x, y)) in warm.draws.iter().zip(&cold.draws).enumerate() {
        assert!(
            (x.split - y.split).abs() < 1.0e-10,
            "draw {i}: {} against {}",
            x.split,
            y.split
        );
    }
}

/// **A held column with an earlier pass starts from it** (M64.2, §69.3): a
/// tick's later pass, handed the first's answer, is a warm solve and not the
/// continuation again. Capped at four Newton steps, the continuation cannot
/// start the column (its base alone takes more) and the cascade refuses; the
/// same capped cascade handed the earlier pass answers, and answers the same
/// column.
#[test]
fn a_held_column_starts_from_its_earlier_pass() {
    let feed_c = 330.0;
    let answer = separate(&spec(2.0, Some(0.0)), feed_c, None).unwrap();
    let capped = HeatBalanceCascade::with_max_iterations(4);
    let e = separate_by(&capped, &spec(2.0, Some(0.0)), feed_c, None, None)
        .expect_err("four Newton steps cannot start a held column from nothing");
    assert!(
        e.contains("starts from the same column with its reboiler"),
        "{e}"
    );
    let again = separate_by(&capped, &spec(2.0, Some(0.0)), feed_c, None, Some(&answer))
        .unwrap_or_else(|e| panic!("from the earlier pass: {e}"));
    let (a, b) = (
        answer.profile.as_ref().unwrap(),
        again.profile.as_ref().unwrap(),
    );
    for (j, (x, y)) in a.temperatures.iter().zip(&b.temperatures).enumerate() {
        assert!(
            (x.value() - y.value()).abs() < 1.0e-8,
            "stage {}: {x:?} against {y:?}",
            j + 1
        );
    }
}

/// **A held column starts from its earlier pass before the last tick's**
/// (M64.3, docs/DESIGN.md §69.9): mid-run, a move the last tick's profile
/// cannot follow sends the first pass of the tick to the continuation; its
/// later passes, handed that answer, start from it rather than failing on the
/// last tick's profile again and walking the continuation once a pass. Capped
/// at four Newton steps, a column handed a profile it cannot follow — the
/// declared column on a 230 °C feed — and nothing else is refused; handed the
/// earlier pass too, it answers, and answers the same column.
#[test]
fn a_held_column_starts_from_its_earlier_pass_before_its_last_tick() {
    let feed_c = 330.0;
    let answer = separate(&spec(2.0, Some(0.0)), feed_c, None).unwrap();
    let stale = separate(&spec(2.0, None), 230.0, None).unwrap();
    let capped = HeatBalanceCascade::with_max_iterations(4);
    separate_by(&capped, &spec(2.0, Some(0.0)), feed_c, Some(&stale), None)
        .expect_err("four Newton steps cannot follow the stale profile");
    let again = separate_by(
        &capped,
        &spec(2.0, Some(0.0)),
        feed_c,
        Some(&stale),
        Some(&answer),
    )
    .unwrap_or_else(|e| panic!("from the earlier pass: {e}"));
    let (a, b) = (
        answer.profile.as_ref().unwrap(),
        again.profile.as_ref().unwrap(),
    );
    for (j, (x, y)) in a.temperatures.iter().zip(&b.temperatures).enumerate() {
        assert!(
            (x.value() - y.value()).abs() < 1.0e-8,
            "stage {}: {x:?} against {y:?}",
            j + 1
        );
    }
}

/// **A declared column ignores an earlier pass, to the bit** (M64.2, §69.3):
/// only a held column reads it — or, since M65.1 (§70.7), one whose cold
/// start refused, which this one's does not — which is what keeps every
/// shipped plant's re-solved ticks what they were. Handed a held column's answer as its
/// earlier pass, the declared column is the one it is handed nothing.
#[test]
fn a_declared_column_ignores_an_earlier_pass() {
    let feed_c = 230.0;
    let other = separate(&spec(2.0, Some(0.0)), 330.0, None).unwrap();
    let model = HeatBalanceCascade::new();
    let bare = separate_by(&model, &spec(2.0, None), feed_c, None, None).unwrap();
    let handed = separate_by(&model, &spec(2.0, None), feed_c, None, Some(&other)).unwrap();
    let (a, b) = (
        bare.profile.as_ref().unwrap(),
        handed.profile.as_ref().unwrap(),
    );
    assert_eq!(a.temperatures, b.temperatures);
    assert_eq!(a.liquid, b.liquid);
    assert_eq!(a.vapour, b.vapour);
    for (x, y) in bare.draws.iter().zip(&handed.draws) {
        assert_eq!(x.split.to_bits(), y.split.to_bits());
        assert_eq!(x.composition.fractions(), y.composition.fractions());
    }
    assert_eq!(reboiler(&bare).to_bits(), reboiler(&handed).to_bits());
    assert_eq!(
        bare.reflux_ratio.unwrap().to_bits(),
        handed.reflux_ratio.unwrap().to_bits()
    );
}

/// **A walk that stops is refused where it stopped** (M64.2, §69.3). Held at
/// zero on a 180 °C feed the column cannot run — the steam alone does not
/// keep its side-draw tray wet, even with the draw cut back (M67.1; on a
/// 240 °C one it runs so) — and the continuation stops short of zero:
/// the refusal names the start, the step and the duty it reached, the dry
/// tray, and the held duty as the setting (not the reflux, a result here).
#[test]
fn a_walk_that_stops_is_refused_where_it_stopped() {
    let e = separate(&spec(2.0, Some(0.0)), 180.0, None).expect_err("held at zero at 180 °C");
    for part in [
        "holding its reboiler at 0.000 MW starts from the same column with its reboiler",
        "the walk from its closure of",
        "stopped at step",
        "of 10, the reboiler at",
        "run dry",
        "could not settle with its reboiler held at 0.000 MW",
    ] {
        assert!(e.contains(part), "missing '{part}' in: {e}");
    }
}
