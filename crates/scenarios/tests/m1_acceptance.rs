//! M1 acceptance gate — the engine-level enforcement of docs/ROADMAP.md §M1.
//!
//! The M1 acceptance criteria are only worth anything if a test fails when
//! they stop holding. The solver-level tests already cover a single solve in
//! isolation (`solvers/tests/invariants.rs`) and hand calculations
//! (`solvers/tests/newton_reference.rs`); what they cannot see is the *engine
//! tick loop* — command application, flow → stream transport, and inventory
//! integration compounding over a long run. That is what this file locks:
//!
//!   A1. Convergence — 1000 ticks of the reference plant, every tick
//!       converging inside the Newton iteration budget.
//!   A2. Mass balance — total inventory never drifts from its initial value
//!       by more than the mass budget.
//!   A3. Determinism — same scenario + same commands ⇒ bit-identical
//!       snapshots on a fresh rerun.
//!
//! The fourth criterion ("both solvers produce qualitatively matching steady
//! states") is covered by `solvers/tests/fidelity_agreement.rs`; it is not
//! duplicated here.

use refinery_core::engine::Engine;
use refinery_core::graph::NodeKind;
use refinery_core::snapshot::Command;

const SCENARIO: &str = include_str!("../../../scenarios/tank_pump_valve.toml");

/// Run length fixed by the roadmap's acceptance criteria.
const ACCEPTANCE_TICKS: u64 = 1000;

/// Per-tick Newton iteration budget (roadmap: "<50 Newton iterations").
/// Observed maximum on the reference plant is 9, so a regression has to be
/// severe to trip this — it guards against a solve degrading into a crawl,
/// not against ordinary tick-to-tick variation.
const MAX_ITERATIONS: u32 = 50;

/// Mass-balance budget [kg], absolute, from the roadmap's "<1e-8".
///
/// Deliberately absolute rather than relative. The tanks exchange *matched*
/// increments — the same solved flow leaves one tank and enters the other —
/// so there is no catastrophic cancellation, and the only error sources are
/// the ~1e-12 kg/s residual between per-edge flows and ulp rounding when
/// adding ~1.4 kg to a ~1.6e5 kg inventory. That random walk measures at
/// ~1.7e-10 kg over the full run, ~57x inside this budget. A *relative* 1e-8
/// would be ~1.8e-3 kg here: seven orders of magnitude above the real drift,
/// and loose enough to stay green while the plant leaked.
const MASS_TOLERANCE_KG: f64 = 1e-8;

fn build() -> Engine {
    let file = refinery_scenarios::load_str(SCENARIO).expect("reference scenario must parse");
    refinery_scenarios::build_engine(&file).expect("reference plant must build")
}

/// Total liquid inventory across every tank [kg].
///
/// `tank_pump_valve` is a *closed* plant — two tanks and no Source, Sink or
/// Atmosphere — so the pump can only move water from one tank to the other
/// and this sum is conserved exactly. Any drift is numerical, not physical.
fn total_tank_mass(engine: &Engine) -> f64 {
    engine
        .graph
        .node_ids()
        .filter_map(|id| match &engine.graph.node(id).kind {
            NodeKind::Tank(t) => Some(t.mass.value()),
            _ => None,
        })
        .sum()
}

/// A1 — every tick of the acceptance run converges inside the budget.
#[test]
fn every_tick_converges_within_the_newton_budget() {
    let mut engine = build();
    for tick in 1..=ACCEPTANCE_TICKS {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick} must converge, got {e:?}"));
        let solver = engine.snapshot().solver;
        assert!(
            solver.converged,
            "tick {tick}: solver reported non-convergence (residual {})",
            solver.residual
        );
        assert!(
            solver.iterations < MAX_ITERATIONS,
            "tick {tick}: {} Newton iterations exceeds the M1 budget of {MAX_ITERATIONS}",
            solver.iterations
        );
    }
}

/// A2 — the closed plant conserves mass across the acceptance run.
#[test]
fn closed_plant_conserves_mass_over_the_acceptance_run() {
    let mut engine = build();

    // Baseline BEFORE the first tick. The law is "total mass never leaves its
    // initial value", so anchoring to tick 1 instead would quietly tolerate a
    // plant that leaked on the very first tick.
    let initial = total_tank_mass(&engine);
    assert!(
        initial > 0.0,
        "reference plant must start with water in it, got {initial} kg"
    );

    for tick in 1..=ACCEPTANCE_TICKS {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick} failed: {e:?}"));
        let error = (total_tank_mass(&engine) - initial).abs();
        assert!(
            error < MASS_TOLERANCE_KG,
            "tick {tick}: total mass drifted {error} kg from the initial {initial} kg, \
             over the {MASS_TOLERANCE_KG} kg budget"
        );
    }
}

/// Fixed command schedule, replayed identically by every determinism run.
///
/// Determinism must hold *with commands applied*, not just for a plant left
/// alone — command application mutates the graph the solver reads, so it is
/// part of the reproducible path. Valve movements keep this inside the
/// envelope the reference tests already cover; the pump-off path (which can
/// reverse flow down the 5 m fill line) is deliberately left to its own test
/// rather than smuggled into the determinism gate.
fn apply_scheduled_commands(engine: &mut Engine, tick: u64) {
    let opening = match tick {
        300 => 0.25,
        600 => 0.75,
        _ => return,
    };
    let valve = engine
        .graph
        .find_node("discharge_valve")
        .expect("reference plant must have a discharge_valve");
    engine
        .apply(Command::SetValveOpening {
            node: valve,
            opening,
        })
        .expect("valve opening within [0,1] must be accepted");
}

/// One full run from a fresh engine, capturing the serialized snapshot after
/// every tick. Fresh engine *and* fresh solvers per call, so this models a
/// real rerun rather than two interleaved engines sharing a process.
fn run_capturing_snapshots() -> Vec<Vec<u8>> {
    let mut engine = build();
    let mut snapshots = Vec::with_capacity(ACCEPTANCE_TICKS as usize);
    for tick in 1..=ACCEPTANCE_TICKS {
        apply_scheduled_commands(&mut engine, tick);
        engine
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick} failed: {e:?}"));
        snapshots.push(serde_json::to_vec(&engine.snapshot()).expect("snapshot must serialize"));
    }
    snapshots
}

/// A3 — reruns are bit-identical (roadmap: golden determinism test).
///
/// Byte comparison of the serialized snapshots is the strict form of the
/// check: serde_json renders identical f64 bits to identical text, so any
/// difference at all — a reordered map, one perturbed low bit — fails here.
#[test]
fn reruns_are_bit_identical() {
    let first = run_capturing_snapshots();
    let second = run_capturing_snapshots();

    // Guards against the whole check passing vacuously: an empty run would
    // zip to zero comparisons and report success.
    assert_eq!(
        first.len(),
        ACCEPTANCE_TICKS as usize,
        "a run must capture one snapshot per tick"
    );
    assert_eq!(
        first.len(),
        second.len(),
        "reruns produced different snapshot counts"
    );
    for (index, (a, b)) in first.iter().zip(&second).enumerate() {
        assert!(
            a == b,
            "reruns diverged at tick {}:\n  first:  {}\n  second: {}",
            index + 1,
            String::from_utf8_lossy(a),
            String::from_utf8_lossy(b)
        );
    }
}
