//! M36: `scenarios/furnace_dry_fired.toml`, as shipped (docs/DESIGN.md §40
//! fork 6) — a furnace fired by hand on a charge that runs out, with no trip.
//!
//! The flame law's own gates run on fixtures in `furnace_reference.rs`. This
//! file pins what only the demo shows, on both fidelities: the feed reaches
//! exactly zero on a measured tick; from there the coil is the dry closed form
//! `T_f − (T_f − T_c)·e^(−t/τ)`; it never crosses its flame; it reads the numbers
//! the file quotes; and the two fidelities agree on it. The FLUID is bounded
//! separately, by what the file says it is.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::NodeKind;
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/furnace_dry_fired.toml");

/// The file's flame, `flame_temperature_c = 1951.1`, and the combustion air's
/// 20 °C, by hand.
const FLAME_K: f64 = 2224.25;
const AIR_K: f64 = 293.15;
/// The file's duty and coil, by hand: 0.6 MW into 0.6 MJ/K.
const DUTY_W: f64 = 0.6e6;
const COIL_C_J_PER_K: f64 = 0.6e6;
const DT_S: f64 = 1.0;
const TICKS: u64 = 20_000;
/// The feed is DRY below the Newton solve's absolute mass tolerance [kg/s]: a
/// flow under it is not resolved, only rounded. Until M37 the feed reached
/// exactly zero (tick 1 749 Newton, 1 819 game); since M37 splits the outlet for
/// its burn-out hole (docs/DESIGN.md §42), Newton has one more free node and
/// settles the dry line to residual noise, ±2.6e-12 kg/s, that never lands on
/// zero. A trickle this size carries ~4e-5 W/K out of the coil: nothing.
const DRY_KG_S: f64 = 1.0e-8;

fn build(solver: &str) -> Engine {
    let src = DEMO.replacen(r#"flow = "newton""#, &format!(r#"flow = "{solver}""#), 1);
    let file = refinery_scenarios::load_str(&src).expect("the demo must parse");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the demo must build: {e}"))
}

fn coil_c(engine: &Engine) -> f64 {
    let heater = engine.graph.find_node("heater").expect("a heater");
    match &engine.graph.node(heater).kind {
        NodeKind::Furnace { coil, .. } => coil.temperature.value() - 273.15,
        other => panic!("heater is a furnace, not {other:?}"),
    }
}

/// The coil every tick, the tick the feed first reads exactly zero, and the
/// worst gap between the fluid and the coil on the dry ticks [K].
fn run(solver: &str) -> (Vec<f64>, u64, f64) {
    let mut engine = build(solver);
    let mut coil = Vec::with_capacity(TICKS as usize);
    let mut dry_from = None;
    let mut worst_gap = 0.0_f64;
    for t in 1..=TICKS {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("{solver}, tick {t}: {e}"));
        let snapshot = engine.snapshot();
        let feed = snapshot
            .edges
            .iter()
            .find(|e| e.name == "feed_line")
            .expect("a feed line")
            .stream
            .mass_flow
            .value();
        if dry_from.is_none() && feed.abs() < DRY_KG_S {
            dry_from = Some(t);
        }
        let c = coil_c(&engine);
        if dry_from.is_some() {
            let fluid = snapshot
                .nodes
                .iter()
                .find(|n| n.name == "heater")
                .expect("a heater")
                .temperature_k
                - 273.15;
            worst_gap = worst_gap.max((fluid - c).abs());
        }
        coil.push(c);
    }
    (coil, dry_from.expect("the charge runs out"), worst_gap)
}

/// **The demo, on both fidelities.** Dry from tick 1 743 (Newton) and 1 744
/// (game), by `DRY_KG_S`; the coil at 538.82 °C at tick 1 749, 1 794.8 °C at 6 000 and
/// 1 950.99 °C at 20 000 — on the closed form from the dry tick on, and below
/// its 1 951.1 °C flame on every tick. The two fidelities differ only in when
/// the trickle ends, and the coil does not see that: they agree to 1e-3 K.
#[test]
fn the_dry_fired_coil_levels_off_under_its_flame_on_both_fidelities() {
    let flame_c = FLAME_K - 273.15;
    let tau = COIL_C_J_PER_K * (FLAME_K - AIR_K) / DUTY_W;
    let mut runs = Vec::new();
    for (solver, expected_dry, gap_bound) in [("newton", 1743, 0.5), ("simple", 1744, 25.0)] {
        let (coil, dry_from, worst_gap) = run(solver);
        assert_eq!(dry_from, expected_dry, "{solver}: the feed reaches zero");
        for (i, c) in coil.iter().enumerate() {
            assert!(
                *c < flame_c,
                "{solver}, tick {}: the coil passed its flame: {c}",
                i + 1
            );
        }
        // From the dry tick on, nothing carries heat away but the stack.
        let start = coil[dry_from as usize - 1] + 273.15;
        for t in dry_from..=TICKS {
            let expected =
                FLAME_K - (FLAME_K - start) * (-((t - dry_from) as f64) * DT_S / tau).exp();
            let c = coil[t as usize - 1] + 273.15;
            assert!(
                (c - expected).abs() < 1.0e-6 * FLAME_K,
                "{solver}, tick {t}: the dry coil must sit on the closed form {expected} K, \
                 got {c}"
            );
        }
        assert!(
            (coil[5_999] - 1794.8).abs() < 0.05,
            "{solver}: 1 794.8 °C at tick 6 000, got {}",
            coil[5_999]
        );
        assert!(
            (coil[TICKS as usize - 1] - 1950.99).abs() < 0.01,
            "{solver}: 1 950.99 °C at tick 20 000, got {}",
            coil[TICKS as usize - 1]
        );
        // The fluid on the dry ticks: within the file's stated bound of the coil.
        assert!(
            worst_gap < gap_bound,
            "{solver}: the fluid strays {worst_gap} K from the coil on the dry ticks, \
             past the file's {gap_bound} K"
        );
        runs.push(coil);
    }
    assert!(
        (runs[0][1_748] - 538.82).abs() < 0.01,
        "newton: 538.82 °C at tick 1 749 (539.0 before M37 split the outlet), got {}",
        runs[0][1_748]
    );
    for t in [3_000, 6_000, 20_000] {
        let (newton, simple) = (runs[0][t - 1], runs[1][t - 1]);
        assert!(
            (newton - simple).abs() < 1.0e-3,
            "tick {t}: the fidelities agree on the coil, {newton} against {simple} °C"
        );
    }
}
