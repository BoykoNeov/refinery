//! M55.0 — the step tick solved on its own states (docs/DESIGN.md §60.0; ledger
//! rows B50 and B53).
//!
//! The shipped gas-lock plant (`scenarios/pump_gas_lock.toml`) with its pump's
//! pocket filling in ONE tick — M54's instant lock — so that a single misread
//! tick is a permanent, visible lock. Until M55 the tick a supply moved read the
//! zero-volume valves' LAST-tick states, and a liquid line's composition from
//! its stored stream (on a cold start, `Stream::stagnant`'s placeholder): the
//! first tick of a cold start at 112 °C flowed 10.50 kg/s against a settled
//! 10.65, and a cold start or a move to 118 °C offered the pump 45% vapour for
//! one tick and locked it, where its settled suction offers 12.7%.

use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::units::Kelvin;
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/pump_gas_lock.toml");

const SOLVERS: [&str; 2] = ["newton", "simple"];

/// Supply temperatures [°C] at which the pump's settled suction offers less
/// than the 16.5% that locks it: liquid, and across the table's fall.
const UNLOCKED: [f64; 6] = [100.0, 110.0, 112.0, 116.0, 118.0, 119.0];

/// The demo plant on `solver` at `celsius`, its pocket filling in one tick.
fn plant(solver: &str, celsius: f64) -> String {
    DEMO.replace("flow = \"newton\"", &format!("flow = \"{solver}\""))
        .replacen(
            "temperature_c = 100.0",
            &format!("temperature_c = {celsius}"),
            1,
        )
        .replace(
            "a = 800.0\non = true\n",
            "a = 800.0\non = true\ngas_fill_time_s = 0.1\n",
        )
}

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the scenario must parse");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant must build: {e}"))
}

fn flow(s: &Snapshot) -> f64 {
    s.edges
        .iter()
        .find(|e| e.name == "suction_line")
        .unwrap()
        .stream
        .mass_flow
        .value()
}

fn locked(engine: &Engine) -> bool {
    let id = engine.graph.find_node("feed_pump").unwrap();
    match engine.graph.node(id).kind {
        NodeKind::Pump { gas_locked, .. } => gas_locked,
        _ => unreachable!(),
    }
}

/// Tick `ticks` times; the flow after the first tick and after the last, and
/// whether the pump locked on any of them.
fn first_and_settled(engine: &mut Engine, ticks: u32, label: &str) -> (f64, f64, bool) {
    let mut first = f64::NAN;
    let mut ever_locked = false;
    for t in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("{label}: tick {t} failed: {e}"));
        let s = engine.snapshot();
        assert!(s.solver.converged, "{label}: tick {t} did not converge");
        if t == 1 {
            first = flow(&s);
        }
        ever_locked |= locked(engine);
    }
    (first, flow(&engine.snapshot()), ever_locked)
}

/// **A cold start's first tick is its settled answer**, at every supply
/// temperature where the pump runs, on both fidelities, and the pump never locks.
#[test]
fn a_cold_starts_first_tick_is_its_settled_answer() {
    for solver in SOLVERS {
        for celsius in UNLOCKED {
            let label = format!("{solver} cold at {celsius} °C");
            let mut engine = build(&plant(solver, celsius));
            let (first, settled, ever_locked) = first_and_settled(&mut engine, 20, &label);
            assert!(!ever_locked, "{label}: the pump locked");
            assert!(
                (first - settled).abs() <= 1e-6 * settled,
                "{label}: first tick {first} kg/s, settled {settled} kg/s"
            );
        }
    }
}

/// **A move's first tick is its settled answer**: from 110 °C, settled, to each
/// temperature where the pump runs, on both fidelities, and the pump never
/// locks.
#[test]
fn a_moves_first_tick_is_its_settled_answer() {
    for solver in SOLVERS {
        for celsius in UNLOCKED {
            let label = format!("{solver} moved 110 -> {celsius} °C");
            let mut engine = build(&plant(solver, 110.0));
            first_and_settled(&mut engine, 20, &label);
            let supply = engine.graph.find_node("rundown_source").unwrap();
            engine
                .apply(Command::SetSourceTemperature {
                    node: supply,
                    temperature: Kelvin(celsius + 273.15),
                })
                .unwrap();
            let (first, settled, ever_locked) = first_and_settled(&mut engine, 20, &label);
            assert!(!ever_locked, "{label}: the pump locked");
            assert!(
                (first - settled).abs() <= 1e-6 * settled,
                "{label}: first tick {first} kg/s, settled {settled} kg/s"
            );
        }
    }
}

/// **The re-solves are reported, and only where a state moved**: a cold start
/// re-solves its first tick (the solve read no states), a settled tick none.
#[test]
fn a_settled_tick_needs_no_re_solve() {
    for solver in SOLVERS {
        let mut engine = build(&plant(solver, 112.0));
        engine.tick().unwrap();
        assert!(engine.snapshot().solver.re_solves >= 1, "{solver}");
        for _ in 0..10 {
            engine.tick().unwrap();
        }
        assert_eq!(engine.snapshot().solver.re_solves, 0, "{solver}");
    }
}
