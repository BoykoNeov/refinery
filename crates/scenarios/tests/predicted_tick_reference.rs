//! M69.2 — a boiling plant's tick predicted (docs/DESIGN.md §74.5): the first
//! solve of a tick reads each zero-volume node moved on by the last tick's move,
//! so the re-solves start closer; not on the tick after a command, and not on a
//! plant at rest. The coil's first law at 1e-12 of its duty, the gate the
//! predictor first broke, stays in `coil_cell_reference.rs`.

use refinery_core::snapshot::Command;
use refinery_core::Engine;

const BOILING: &str = include_str!("../../../scenarios/crude_unit_boiling.toml");
const GAS_LOCK: &str = include_str!("../../../scenarios/pump_gas_lock.toml");

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the plant parses");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant builds: {e}"))
}

/// One tick; its re-solves and its hardest solve's Newton iterations.
fn tick(engine: &mut Engine) -> (u32, u32) {
    engine.tick().expect("the plant runs");
    let s = engine.snapshot();
    assert!(s.solver.converged);
    (s.solver.re_solves, s.solver.iterations)
}

/// **A drifting boiling plant solves once a tick.** The boiling crude unit held
/// from its file drifts as its coil's metal settles: read a tick behind, each
/// tick's first solve misses the densities the tick ends at and is re-solved
/// once (the second tick, with no move yet to go on); read moved on by the last
/// tick's move, the first solve already lands within `SWEPT_DENSITY_TOL`, from
/// the third tick on. Measured: 1 re-solve on tick 2, 0 on ticks 3–12.
#[test]
fn a_drifting_boiling_plant_solves_once_a_tick() {
    let mut engine = build(BOILING);
    tick(&mut engine);
    assert_eq!(
        tick(&mut engine).0,
        1,
        "the second tick reads the first's states and re-solves once"
    );
    for t in 3..=12 {
        assert_eq!(
            tick(&mut engine).0,
            0,
            "tick {t}: the predicted first solve stands"
        );
    }
}

/// **The tick after a command is not guessed.** A command is a step, not a
/// drift, so the next tick reads the last tick's states as before and re-solves
/// as an unpredicted tick does; the one after is predicted again. The command
/// here sets the feed valve where it already stands, so the plant itself does
/// not move: what moves the count is the guard alone.
#[test]
fn the_tick_after_a_command_is_not_guessed() {
    let mut engine = build(BOILING);
    for _ in 1..=6 {
        tick(&mut engine);
    }
    let feed_valve = engine.graph.find_node("feed_valve").expect("a feed valve");
    engine
        .apply(Command::SetValveOpening {
            node: feed_valve,
            opening: 1.0,
        })
        .expect("the feed valve opens fully");
    assert_eq!(
        tick(&mut engine).0,
        1,
        "the tick after a command re-solves as an unpredicted tick"
    );
    for t in 1..=3 {
        assert_eq!(
            tick(&mut engine).0,
            0,
            "{t} ticks on: the guard is for one tick only"
        );
    }
}

/// **A plant at rest is solved as it was.** The gas-lock plant carries the line
/// flash and stands still from its second tick: its states predict themselves,
/// so no tick is guessed, and no solve takes the step a guessed one would. Its
/// answers are bit for bit those before M69.2 (the corpus); here, no Newton step
/// after the first tick.
#[test]
fn a_plant_at_rest_is_not_guessed() {
    let mut engine = build(GAS_LOCK);
    tick(&mut engine);
    for t in 2..=100 {
        assert_eq!(tick(&mut engine).1, 0, "tick {t} took a Newton step");
    }
}
