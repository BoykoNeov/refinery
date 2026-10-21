//! A held reboiler the column declines (M66.2, docs/DESIGN.md §71.2): asked to
//! hold a duty it cannot run at, a column runs at the duty it last ran at and
//! says so; asked one it can, it takes it; a later pass of a tick that
//! declined goes straight to the running duty; where the running duty fails
//! too, the refusal is the column's own first. A column holding the duty it
//! runs at, or handed no running duty, is the column it always was, to the
//! bit.
//!
//! The column is `held_reboiler.rs`'s (M62.6's steamed test column): held at
//! zero it runs on a 330 °C feed, on a 240 °C one with its side draw cut back
//! (M67.1, §72.1), and not on a 180 °C one (§69.8); its reboiler free, it runs
//! on all three.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::graph::{CascadeSpec, ColumnDraw, NodeId};
use refinery_core::traits::{
    ColumnPass, ColumnSteam, DeclinedHold, RunningReboiler, Separation, SeparationModel,
};
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
        loops: Vec::new(),
        trays: Vec::new(),
    }
}

/// One pass at a feed temperature [°C], warm from `seed`, handed `running` as
/// the duty the column last ran at.
fn separate(
    spec: &CascadeSpec,
    feed_c: f64,
    seed: Option<&Separation>,
    running: Option<RunningReboiler<'_>>,
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
                earlier_pass: None,
                running_reboiler: running,
                steam: Some(steam()),
            },
            &TroutonThermo::new(),
        )
        .map_err(|e| e.to_string())
}

fn reboiler(separation: &Separation) -> f64 {
    separation.reboiler_duty.unwrap().value()
}

fn running(duty: f64) -> Option<RunningReboiler<'static>> {
    Some(RunningReboiler {
        duty: Watt(duty),
        declined: None,
    })
}

fn temperatures(separation: &Separation) -> Vec<f64> {
    separation
        .profile
        .as_ref()
        .unwrap()
        .temperatures
        .iter()
        .map(|t| t.value())
        .collect()
}

/// The column on its reboiler at a reflux of 2: the start every gate below
/// hands over from, and the duty it ran at.
fn free(feed_c: f64) -> (Separation, f64) {
    let free = separate(&spec(2.0, None), feed_c, None, None).expect("the free column runs");
    let duty = reboiler(&free);
    assert!(duty > 1.0e5, "a reboiler of {duty} W");
    (free, duty)
}

/// **Held at zero where it cannot run, the column runs at its running duty and
/// says so.** On a 180 °C feed, from the free column: the answer is the
/// column held at the duty the free column ran at — to the bit — published at
/// that duty, the decline naming the zero asked for and the column's own
/// reason.
#[test]
fn a_hold_it_cannot_run_at_is_declined_for_its_running_duty() {
    let (start, duty) = free(180.0);
    let declined = separate(&spec(2.0, Some(0.0)), 180.0, Some(&start), running(duty)).unwrap();
    assert_eq!(reboiler(&declined), duty);
    let hold = declined
        .reboiler_declined
        .clone()
        .expect("the hold is declined");
    assert_eq!(hold.asked, Watt(0.0));
    assert!(
        hold.reason.starts_with("heat-balance cascade") && hold.reason.contains("0.000 MW"),
        "{}",
        hold.reason
    );
    // The held duty is tried warm alone: no continuation, whose refusal would
    // start "a column holding its reboiler at … starts from the same column".
    assert!(
        !hold.reason.contains("starts from the same column"),
        "{}",
        hold.reason
    );
    let kept = separate(&spec(2.0, Some(duty)), 180.0, Some(&start), running(duty)).unwrap();
    assert_eq!(kept.reboiler_declined, None);
    assert_eq!(temperatures(&declined), temperatures(&kept));
    assert_eq!(declined.reflux_ratio, kept.reflux_ratio);
}

/// **Held at zero where it can run, the column takes it.** On a 330 °C feed,
/// from the free column: the reboiler published as zero, no decline, and the
/// column the one handed no running duty (to 1e-8 K; that one may start by the
/// continuation, this one is warm).
#[test]
fn a_hold_it_can_run_at_is_taken() {
    let (start, duty) = free(330.0);
    let taken = separate(&spec(2.0, Some(0.0)), 330.0, Some(&start), running(duty)).unwrap();
    assert_eq!(reboiler(&taken), 0.0);
    assert_eq!(taken.reboiler_declined, None);
    let plain = separate(&spec(2.0, Some(0.0)), 330.0, Some(&start), None).unwrap();
    for (a, b) in temperatures(&taken).iter().zip(temperatures(&plain)) {
        assert!((a - b).abs() < 1.0e-8, "{a} K against {b} K");
    }
}

/// **A later pass of a tick that declined goes straight to its running duty.**
/// Handed the earlier pass's decline of the same duty, the column does not ask
/// again: the decline it reports is the earlier one, word for word. A decline
/// of another duty is not this one's, and the column asks.
#[test]
fn a_later_pass_of_a_declining_tick_does_not_ask_again() {
    let (start, duty) = free(180.0);
    let earlier = DeclinedHold {
        asked: Watt(0.0),
        reason: "the earlier pass's reason".into(),
    };
    let later = separate(
        &spec(2.0, Some(0.0)),
        180.0,
        Some(&start),
        Some(RunningReboiler {
            duty: Watt(duty),
            declined: Some(&earlier),
        }),
    )
    .unwrap();
    assert_eq!(reboiler(&later), duty);
    assert_eq!(later.reboiler_declined, Some(earlier.clone()));

    let other = DeclinedHold {
        asked: Watt(5.0e5),
        ..earlier
    };
    let asked = separate(
        &spec(2.0, Some(0.0)),
        180.0,
        Some(&start),
        Some(RunningReboiler {
            duty: Watt(duty),
            declined: Some(&other),
        }),
    )
    .unwrap();
    let reason = asked.reboiler_declined.unwrap().reason;
    assert!(reason.starts_with("heat-balance cascade"), "{reason}");
}

/// **Where the running duty fails too, the refusal is the column's own
/// first.** On a 180 °C feed, held at zero, handed a running duty of 1 kW it
/// cannot run at either: the column's refusal at zero, then that it could not
/// keep the 0.001 MW either, and why.
#[test]
fn where_the_running_duty_fails_too_the_column_is_refused_by_its_own_reason() {
    let (start, _) = free(180.0);
    let e = separate(&spec(2.0, Some(0.0)), 180.0, Some(&start), running(1.0e3))
        .expect_err("neither duty runs");
    assert!(
        e.starts_with("numerical error: heat-balance cascade"),
        "{e}"
    );
    let (own, kept) = e
        .split_once("It could not keep the 0.001 MW its reboiler ran at either")
        .unwrap_or_else(|| panic!("{e}"));
    assert!(own.contains("held at 0.000 MW"), "{own}");
    assert!(kept.contains("heat-balance cascade"), "{kept}");
}

/// **Held at zero where it runs only with a draw cut back, the column takes
/// it** (M67.1, §72.1). On a 240 °C feed — declined until M67 — the hold is
/// taken: the reboiler published as zero, no decline, the side draw (stage 3)
/// cut back and reported, the same answer handed no running duty.
#[test]
fn a_hold_it_can_run_at_with_a_draw_cut_back_is_taken() {
    let (start, duty) = free(240.0);
    let taken = separate(&spec(2.0, Some(0.0)), 240.0, Some(&start), running(duty)).unwrap();
    assert_eq!(reboiler(&taken), 0.0);
    assert_eq!(taken.reboiler_declined, None);
    assert_eq!(taken.cut_back.len(), 1, "{:?}", taken.cut_back);
    assert_eq!(taken.cut_back[0].draw, 2);
    let share = taken.cut_back[0].of_declared;
    assert!(share > 0.0 && share < 1.0, "{share}");
    let plain = separate(&spec(2.0, Some(0.0)), 240.0, Some(&start), None).unwrap();
    for (a, b) in temperatures(&taken).iter().zip(temperatures(&plain)) {
        assert!((a - b).abs() < 1.0e-8, "{a} K against {b} K");
    }
}

/// **A column holding the duty it runs at, or handed none, is untouched**, to
/// the bit: held at zero on 330 °C with a running duty of zero and with none;
/// the free column with a running duty and with none.
#[test]
fn a_hold_at_its_running_duty_or_none_is_the_column_it_was() {
    let (start, duty) = free(330.0);
    let held = spec(2.0, Some(0.0));
    let at_zero = separate(&held, 330.0, Some(&start), running(0.0)).unwrap();
    let without = separate(&held, 330.0, Some(&start), None).unwrap();
    assert_eq!(temperatures(&at_zero), temperatures(&without));
    assert_eq!(at_zero.reflux_ratio, without.reflux_ratio);
    let free_running = separate(&spec(2.0, None), 330.0, Some(&start), running(duty)).unwrap();
    let free_without = separate(&spec(2.0, None), 330.0, Some(&start), None).unwrap();
    assert_eq!(temperatures(&free_running), temperatures(&free_without));
    assert_eq!(reboiler(&free_running), reboiler(&free_without));
}
