//! The relief valve as a pure element characteristic (docs/ROADMAP.md §M5.4c,
//! docs/DESIGN.md §3a fork 5), and the blowdown demo as wired.
//!
//! **What is pinned, and what is structural rather than pinned.**
//!
//! Fork 5's headline property is MEMORYLESSNESS: the same inlet pressure gives
//! the same opening however it was reached. Asserting that on `relief_opening`
//! directly would be a tautology — it is a pure function of one `f64`, so there
//! is nothing there to remember with. The gate that is not a tautology is at the
//! PLANT level: a receiver driven to its relieving state **from below** (building
//! from 12 bar) and **from above** (blowing down from 30 bar) must settle on the
//! same pressure. Hysteresis — the property a real PSV has and this one does not
//! — would give two different settle points, which is exactly what
//! `a_psv_settles_at_the_same_pressure_from_either_direction` refuses.
//!
//! The rest of the file separates cheaply-pinned shape (the opening curve, the
//! loader's refusals) from the demo's own claims (it builds, it lifts, it
//! relieves, it settles inside the accumulation band with a PARTIAL opening).
//! That last word matters: a PSV pinned at full lift is an undersized relief and
//! would make the "pressure-actuated area" claim untestable, because a saturated
//! valve behaves exactly like a fixed one.

use refinery_core::graph::NodeKind;
use refinery_scenarios::NodeDef;

const PLANT: &str = include_str!("../../../scenarios/relief_blowdown.toml");

/// The plant's declared relief spec, restated rather than read back.
const SET_BAR: f64 = 20.0;
const ACCUMULATION_BAR: f64 = 1.0;

fn load() -> refinery_scenarios::ScenarioFile {
    refinery_scenarios::load_str(PLANT).expect("relief_blowdown.toml parses")
}

/// Run the plant from a given starting receiver pressure and report
/// `(P_receiver [bar], relief [kg/s], make_up [kg/s])` at the end.
fn run_from(start_bar: f64, ticks: usize) -> (f64, f64, f64) {
    let mut file = load();
    match file.nodes.get_mut("receiver") {
        Some(NodeDef::Vessel { pressure_bar, .. }) => *pressure_bar = start_bar,
        other => panic!("relief_blowdown.toml must define receiver as a vessel, got {other:?}"),
    }
    let mut engine = refinery_scenarios::build_engine(&file).expect("builds");
    for i in 0..ticks {
        engine.tick().unwrap_or_else(|e| panic!("tick {i}: {e}"));
    }
    let receiver = engine.graph.find_node("receiver").expect("receiver");
    let pressure = match &engine.graph.node(receiver).kind {
        NodeKind::Vessel(v) => v.pressure(&engine.slate).value() / 1e5,
        other => panic!("receiver must be a vessel, got {other:?}"),
    };
    let flow = |name: &str| {
        let eid = engine
            .graph
            .edge_ids()
            .find(|e| engine.graph.pipe(*e).name == name)
            .unwrap_or_else(|| panic!("plant must have a '{name}' pipe"));
        engine.graph.pipe(eid).stream.mass_flow.value()
    };
    (pressure, flow("flare_line"), flow("make_up"))
}

// ---------------------------------------------------------------------------
// A — memorylessness, the property fork 5 trades hysteresis away for.
// ---------------------------------------------------------------------------

/// The receiver settles on the same pressure whether it arrives from below or
/// from above.
///
/// This is the non-tautological form of "the opening is a memoryless function of
/// its own inlet pressure". A spring-loaded PSV with real blowdown hysteresis
/// recloses BELOW its set pressure, so a vessel arriving from above would sit at
/// a *lower* equilibrium than one arriving from below — two settle points, one
/// plant. This model has one, and that is both the simplification fork 5 makes
/// and the thing it is honest about giving up.
///
/// The two runs are also asserted to genuinely approach from opposite sides, so
/// the gate cannot pass by both starting on the same side of the answer.
#[test]
fn a_psv_settles_at_the_same_pressure_from_either_direction() {
    let (rising, _, _) = run_from(12.0, 1500);
    let (falling, _, _) = run_from(30.0, 1500);

    assert!(
        12.0 < rising && 30.0 > falling,
        "premise: the two runs must approach the settle point from opposite \
         sides — rising started at 12 bar and reached {rising:.4}, falling \
         started at 30 bar and reached {falling:.4}"
    );
    approx::assert_relative_eq!(rising, falling, max_relative = 1e-3);
}

// ---------------------------------------------------------------------------
// B — the demo's own claims.
// ---------------------------------------------------------------------------

/// Pressure builds, the PSV lifts, the flare takes the relief, and the receiver
/// settles INSIDE the accumulation band at a partial opening.
///
/// Four separate claims, asserted separately because each can fail on its own:
/// a PSV that never lifts, one that lifts immediately, one that saturates at full
/// lift (an undersized relief, which would make the whole "pressure-actuated
/// area" idea untestable — a saturated valve is indistinguishable from a fixed
/// one), and one whose relief does not balance the make-up at steady state.
#[test]
fn the_receiver_builds_lifts_relieves_and_settles_in_band() {
    // Shut below the set pressure: the plant starts at 12 bar, and after a
    // single tick it is still far below 20, so nothing may reach the flare.
    let (early_p, early_relief, early_make_up) = run_from(12.0, 1);
    assert!(
        early_p < SET_BAR,
        "premise: one tick in, the receiver must still be below set pressure, \
         got {early_p:.3} bar"
    );
    assert_eq!(
        early_relief, 0.0,
        "a PSV below its set pressure must be shut, got {early_relief} kg/s"
    );
    assert!(
        early_make_up > 0.0,
        "and the make-up must be filling it, got {early_make_up} kg/s"
    );

    // Settled: inside the band, strictly — above the set pressure (it has lifted)
    // and below full lift (it is not saturated).
    let (settled, relief, make_up) = run_from(12.0, 1500);
    assert!(
        settled > SET_BAR,
        "the receiver must settle ABOVE the set pressure — a PSV that holds \
         exactly at set is passing nothing. Got {settled:.4} bar"
    );
    assert!(
        settled < SET_BAR + ACCUMULATION_BAR,
        "the receiver must settle BELOW full lift, or the relief is undersized \
         and the opening is saturated — which would make the partial-opening \
         claim untestable. Got {settled:.4} bar against a full-lift point of \
         {:.4}",
        SET_BAR + ACCUMULATION_BAR
    );

    // At steady state the flare takes exactly what the header supplies.
    approx::assert_relative_eq!(relief, make_up, max_relative = 5e-3);
    assert!(
        relief > 0.0,
        "the flare must be taking relief, got {relief}"
    );
}

/// The relieving rate RESPONDS to the set pressure, which is what says the
/// opening is actuated by pressure rather than fixed.
///
/// Raising the set point by half a bar must raise the settle pressure by
/// approximately the same half bar — the valve finds the same opening at a higher
/// pressure, because at steady state the opening is fixed by the make-up rate.
/// A PSV stuck fully open would settle in the same place regardless.
#[test]
fn the_settle_point_tracks_the_set_pressure() {
    let settle_at = |set: f64| {
        let mut file = load();
        match file.nodes.get_mut("psv") {
            Some(NodeDef::ReliefValve {
                set_pressure_bar, ..
            }) => *set_pressure_bar = set,
            other => {
                panic!("relief_blowdown.toml must define psv as a relief_valve, got {other:?}")
            }
        }
        let mut engine = refinery_scenarios::build_engine(&file).expect("builds");
        for _ in 0..1500 {
            engine.tick().expect("ticks");
        }
        let receiver = engine.graph.find_node("receiver").expect("receiver");
        match &engine.graph.node(receiver).kind {
            NodeKind::Vessel(v) => v.pressure(&engine.slate).value() / 1e5,
            other => panic!("receiver must be a vessel, got {other:?}"),
        }
    };
    let base = settle_at(SET_BAR);
    let raised = settle_at(SET_BAR + 0.5);
    let shift = raised - base;
    assert!(
        (0.35..0.65).contains(&shift),
        "raising the set pressure by 0.5 bar must move the settle point by about \
         the same amount; got {shift:.4} bar ({base:.4} → {raised:.4})"
    );
}

// ---------------------------------------------------------------------------
// B′ — the demo under BOTH fidelities.
// ---------------------------------------------------------------------------

/// Both fidelities run the demo to the same settle point, and the Simple sweep
/// count stays well inside its cap.
///
/// **This gate exists because its absence let a real failure through.** The first
/// demo geometry diverged under `simple` at tick 91 — 5000 Gauss–Seidel sweeps,
/// residual 5.9e-7 — while Newton took 8, and it was found by running the CLI, not
/// by running the suite: every other gate in this file builds from the scenario
/// file, whose `[fidelity] flow` is `newton`. So the geometry fix that cured it
/// was protected by nothing. This is M5.3's own finding restated — the drum "had
/// never been run under `simple` at all, and now is".
///
/// The mechanism is worth keeping with the gate, because it generalises past this
/// plant: a normally-shut PSV leaves its valve node a DEAD END (`flare_line` does
/// not conduct), so the receiver's Gauss–Seidel diagonal is dominated by a fat
/// inlet branch carrying no net flow, and each sweep moves the vessel by almost
/// nothing. Newton is immune — it solves the linear system exactly. Any
/// normally-shut branch on a low-resistance line will do the same.
///
/// The sweep budget is asserted at half the solver's 5000 cap. Measured: 868.
/// A margin gate rather than an exact count, because the number is a property of
/// the geometry and would move with any legitimate re-sizing — what must not move
/// is that it stays far from the cliff.
#[test]
fn both_fidelities_settle_the_relief_and_simple_stays_clear_of_its_cap() {
    let run = |fidelity: &str| {
        let mut file = load();
        file.fidelity.flow = fidelity.to_string();
        let mut engine = refinery_scenarios::build_engine(&file).expect("builds");
        let mut worst = 0u32;
        for i in 0..1500 {
            engine
                .tick()
                .unwrap_or_else(|e| panic!("{fidelity} tick {i}: {e}"));
            worst = worst.max(engine.snapshot().solver.iterations);
        }
        let receiver = engine.graph.find_node("receiver").expect("receiver");
        let pressure = match &engine.graph.node(receiver).kind {
            NodeKind::Vessel(v) => v.pressure(&engine.slate).value() / 1e5,
            other => panic!("receiver must be a vessel, got {other:?}"),
        };
        (pressure, worst)
    };

    let (newton_p, newton_iters) = run("newton");
    let (simple_p, simple_iters) = run("simple");

    approx::assert_relative_eq!(newton_p, simple_p, max_relative = 1e-4);
    assert!(
        newton_iters < 25,
        "Newton must crack this plant easily; took {newton_iters} iterations"
    );
    assert!(
        simple_iters < 2500,
        "the Simple sweep count must stay well inside its 5000 cap, or a \
         normally-shut PSV has put its vessel back into the dead-end stall this \
         plant's inlet line is sized to avoid. Took {simple_iters} sweeps"
    );
}

// ---------------------------------------------------------------------------
// C — refusals, at both doors.
// ---------------------------------------------------------------------------

/// A gas valve with no `x_T` is refused by the SOLVER as well as the loader.
///
/// `require_gas_valve_x_t` runs inside `build_engine`, and `build_engine` is not
/// the only way to a `PlantGraph` — the invariant proptests construct one
/// directly. Every generator uses `Slate::water_only()` today, so none can reach
/// the gas branch; that is a fact about the current generators rather than about
/// the type, and this second door is what stops a gas-valve arm added to one of
/// them later from silently running the incompressible law on a compressible
/// fluid.
///
/// Built here by hand for exactly that reason: going through the loader would
/// test the loader's guard again instead of this one.
#[test]
fn the_solver_refuses_a_gas_valve_with_no_x_t_even_bypassing_the_loader() {
    use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
    use refinery_core::graph::{LeakRole, Node, Pipe, PlantGraph};
    use refinery_core::traits::FlowSolver;
    use refinery_core::units::*;

    let slate = Slate::new(vec![PseudoComponent {
        name: "gas".into(),
        tb: Kelvin(111.0),
        molar_mass: KgPerMol(0.016_043),
        density: None,
        cp: JPerKgK(2220.0),
        cp_shape: None,
        phase: Phase::Gas,
    }])
    .expect("a single gas cut is a valid slate");
    let pure = Composition::pure(1, 0);
    let node = |name: &str, kind| Node {
        name: name.into(),
        kind,
        heat_input: Watt(0.0),
    };

    let mut graph = PlantGraph::new();
    let src = graph.add_node(node(
        "header",
        NodeKind::Source {
            pressure: Pascal(10.0e5),
            temperature: Kelvin(293.15),
            composition: pure.clone(),
        },
    ));
    // `x_t: None` on a gas stream — the pairing the loader refuses.
    let valve = graph.add_node(node(
        "v",
        NodeKind::Valve {
            cv_max: 2.0e-5,
            opening: 1.0,
            x_t: None,
        },
    ));
    let sink = graph.add_node(node(
        "flare",
        NodeKind::Sink {
            pressure: Pascal(1.0e5),
            temperature: Kelvin(293.15),
            composition: pure.clone(),
        },
    ));
    let mut pipe = |name: &str, a, b| {
        graph.add_pipe(
            a,
            b,
            Pipe {
                name: name.into(),
                length: Meter(10.0),
                diameter: Meter(0.1),
                friction_factor: 0.02,
                elevation_change: Meter(0.0),
                leak: LeakRole::None,
                ambient_ua: WattPerKelvin::ZERO,
                stream: refinery_core::stream::Stream::stagnant(1, Kelvin(293.15), P_ATM),
            },
        );
    };
    pipe("in", src, valve);
    pipe("out", valve, sink);

    let err = refinery_solvers::NewtonFlowSolver::default()
        .solve(&graph, &slate, &Default::default(), Seconds(0.1))
        .expect_err("a gas valve with no x_T must not compile");
    let msg = err.to_string();
    assert!(
        msg.contains("x_T") && msg.contains("gas"),
        "the refusal must name the missing factor and the phase: {msg}"
    );
}

/// `SetValveOpening` on a relief valve is refused **for its own reason**.
///
/// Not a silent no-op, and not the generic "is not a valve" — a relief valve IS a
/// valve, and the point is that its opening is not an operator setpoint at all.
/// A command that appeared to take effect would be overwritten by the very next
/// solve, which is worse than a refusal. The message is asserted rather than just
/// the `Err`, on `negative_furnace_duty_is_refused`'s precedent: a refusal for
/// the wrong reason passes an `is_err()` check just as well as the right one.
#[test]
fn setting_a_relief_valves_opening_by_command_is_refused_naming_the_kind() {
    use refinery_core::snapshot::Command;

    let mut engine = refinery_scenarios::build_engine(&load()).expect("builds");
    let psv = engine.graph.find_node("psv").expect("plant has a psv");
    let err = engine
        .apply(Command::SetValveOpening {
            node: psv,
            opening: 0.5,
        })
        .expect_err("a relief valve's opening must not be commandable");
    let msg = err.to_string();
    assert!(
        msg.contains("relief valve") && msg.contains("inlet pressure"),
        "the refusal must say WHY, not merely that it is not a valve: {msg}"
    );
}

// ---------------------------------------------------------------------------
// C′ — loader refusals.
// ---------------------------------------------------------------------------

fn expect_refusal(file: &refinery_scenarios::ScenarioFile, what: &str) -> String {
    match refinery_scenarios::build_engine(file) {
        Ok(_) => panic!("{what}"),
        Err(e) => e.to_string(),
    }
}

/// A zero accumulation band is refused: it makes the opening a STEP in pressure,
/// and a discontinuous characteristic is precisely what `elements.rs` promises
/// not to hand the Newton Jacobian.
#[test]
fn a_zero_accumulation_band_is_refused() {
    for bad in [0.0f64, -1.0] {
        let mut file = load();
        match file.nodes.get_mut("psv") {
            Some(NodeDef::ReliefValve {
                accumulation_bar, ..
            }) => *accumulation_bar = bad,
            other => panic!("expected a relief_valve, got {other:?}"),
        }
        let msg = expect_refusal(&file, &format!("accumulation_bar = {bad} must not load"));
        assert!(
            msg.contains("accumulation"),
            "the refusal must name the field, got: {msg}"
        );
    }
}

/// A PSV in gas service needs `x_t` for the same reason an ordinary valve does —
/// they share `compile_edge`'s arm and therefore the same compressible law, so a
/// separate requirement could let a PSV reach the gas branch with no choke point.
#[test]
fn a_gas_relief_valve_without_x_t_is_refused() {
    let mut file = load();
    match file.nodes.get_mut("psv") {
        Some(NodeDef::ReliefValve { x_t, .. }) => *x_t = None,
        other => panic!("expected a relief_valve, got {other:?}"),
    }
    let msg = expect_refusal(&file, "a gas PSV with no x_t must not load");
    assert!(
        msg.contains("x_t") && msg.contains("gas"),
        "the refusal must name the field and the service: {msg}"
    );
}

/// Measurement, not a gate: the trajectory the demo's prose describes.
/// `cargo test -p refinery-scenarios --test relief_valve_reference -- \
/// --ignored --nocapture`
#[test]
#[ignore = "measurement, not a gate"]
fn measure_the_relief_trajectory() {
    for start in [12.0f64, 30.0] {
        for ticks in [1usize, 100, 200, 400, 800, 1500] {
            let (p, relief, make_up) = run_from(start, ticks);
            println!(
                "start {start:>5.1} bar, {ticks:>5} ticks: P = {p:.4} bar, \
                 relief = {relief:.5}, make-up = {make_up:.5} kg/s"
            );
        }
    }
}
