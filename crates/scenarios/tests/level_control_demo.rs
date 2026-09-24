//! M8.4: `scenarios/tank_level_control.toml`, as shipped.
//!
//! The seam's own gates live in `control_reference.rs` and run on fixtures built
//! to expose one behaviour each. What none of them can see, and this file does,
//! is the **demo file**: the first plant in `scenarios/` that declares a
//! `[[controls]]` table, and therefore the first place the loader, the tick
//! order, the PI algorithm and a real hydraulic network are exercised together
//! by a file a reader is invited to run.
//!
//! Three claims the file's header comment makes, each pinned here so that a
//! change to the plant's sizing fails a test rather than quietly making the
//! comment wrong:
//!
//! 1. **It holds, and parked it does not.** `control_reference.rs` opens by
//!    refusing "the level sat at the setpoint" as a gate, because a tank
//!    draining through a *fixed* valve self-regulates. So the counterfactual is
//!    measured rather than assumed: the same file in `manual` — the loop
//!    present, measuring and reporting, writing nothing — runs to the roof.
//! 2. **It starts without stepping its own actuator**, which is what the file's
//!    equal `level_valve.opening` and `initial_output` are *for*. The bound is
//!    derived from the two roundings that stand between them, not chosen.
//! 3. **Its gain reaches the clamp later than its neighbour does**, and that
//!    neighbour is run here too — a margin claim with only the passing side
//!    measured is not a margin claim. M8.4 wrote this as a bound on a solver
//!    STALL; M9.0 fixed the stall (DESIGN §11), so what is left is a tuning
//!    bound, and the gate had to be re-premised rather than inverted.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{ControlledValue, LoopId, NodeKind};
use refinery_core::snapshot::Command;
use refinery_core::units::Meter;
use refinery_core::Engine;
use refinery_scenarios::{build_engine, load_str};

const DEMO: &str = include_str!("../../../scenarios/tank_level_control.toml");

/// The file's own declared numbers, named once so an assertion below cannot
/// drift away from the plant it is about.
const SETPOINT_M: f64 = 4.0;
const GAIN_PER_M: f64 = 0.25;
const DECLARED_OPENING: f64 = 0.2;
const TANK_HEIGHT_M: f64 = 10.0;

fn build(src: &str) -> Engine {
    build_engine(&load_str(src).expect("a shipped scenario must parse"))
        .unwrap_or_else(|e| panic!("a shipped scenario must build: {e}"))
}

/// The measurement the loop ACTED ON, which is what a snapshot reports and what
/// the file's header quotes.
fn level(engine: &Engine) -> f64 {
    match engine.snapshot().controls[0]
        .measurement
        .expect("a stored quantity is measured from load")
    {
        ControlledValue::Level { m } => m.value(),
        // M10 added a second variant. A level fixture reading a pressure means
        // the loop measures something this helper cannot report, which is a
        // broken test rather than a number to coerce.
        other => panic!("this fixture's loop measures a level, not {other:?}"),
    }
}

fn output(engine: &Engine) -> f64 {
    engine.snapshot().controls[0].output
}

fn valve_opening(engine: &Engine, name: &str) -> f64 {
    let id = engine
        .graph
        .find_node(name)
        .unwrap_or_else(|| panic!("the demo declares a node '{name}'"));
    match engine.graph.node(id).kind {
        NodeKind::Valve { opening, .. } => opening,
        _ => panic!("'{name}' is a valve in the demo"),
    }
}

fn run(engine: &mut Engine, ticks: u64) {
    for t in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the demo must run: tick {t}: {e}"));
    }
}

// ----------------------------------------------------------------- the pair

/// The demo against itself with the loop parked, which is the only form of this
/// assertion that means anything.
///
/// A single "it ended near the setpoint" would pass on a plant with no loop at
/// all: a tank's bottom pressure rises with level, so a fixed drain passes more
/// as the tank fills and the thing finds an equilibrium unaided. The demo is
/// therefore *sized* so that the unaided equilibrium is useless — the drain at
/// its declared 0.20 cannot pass the pump's flow below the tank's roof — and the
/// gate is the separation between the two runs, not the AUTO number alone.
///
/// **The last assertion is the one a proportional loop could not pass**, and it
/// is why the demo ships `algorithm = "pi"`. Over the last 2 000 ticks the load
/// is still falling (the supply tank is draining, so the pump delivers less),
/// and the loop tracks it by CLOSING the drain. `u = K·e` makes a P loop's
/// output and its level the same statement: a falling output forces a falling
/// level, `Δlevel = Δu / K`. Here the output falls by 6.778e-3 while the level
/// *rises* by 1.584e-3 — the wrong sign for that identity, not merely a smaller
/// number than it predicts, which is the discrimination M8.3's gate 3 had to be
/// rebuilt to get (docs/DESIGN.md §10).
#[test]
fn the_demo_holds_its_level_where_the_parked_loop_runs_to_the_roof() {
    let mut auto = build(DEMO);
    run(&mut auto, 4_000);
    let (level_at_4k, output_at_4k) = (level(&auto), output(&auto));
    run(&mut auto, 2_000);
    let (level_at_6k, output_at_6k) = (level(&auto), output(&auto));

    // Measured 3.991994 m. The band is a tenth of a metre against a setpoint of
    // four, which is a claim about regulation rather than about this run's last
    // digits — the tight numbers are the header's, and the identity below is
    // what carries the discrimination.
    assert!(
        (level_at_6k - SETPOINT_M).abs() < 0.1,
        "the demo in auto must hold its setpoint: {level_at_6k} m against {SETPOINT_M} m"
    );

    // The loop present, truthful on its faceplate, and writing nothing.
    let mut parked = build(&DEMO.replace("mode = \"auto\"", "mode = \"manual\""));
    run(&mut parked, 6_000);
    let parked_level = level(&parked);
    assert_eq!(
        valve_opening(&parked, "level_valve"),
        DECLARED_OPENING,
        "a parked loop writes nothing, so the drain stays where the file declared it"
    );
    // Measured 9.190452 m, still rising. Stated against the tank's own geometry
    // rather than a chosen number: the counterfactual is not "somewhat high", it
    // is against the roof.
    assert!(
        parked_level > 0.9 * TANK_HEIGHT_M,
        "the counterfactual must run away, or the demo is measuring the plant \
         instead of the loop: parked ended at {parked_level} m"
    );

    // The identity a proportional loop cannot escape, run on the demo's own
    // trajectory. Sign, not magnitude: `Δlevel = Δu / K` would need the level to
    // have FALLEN by 2.711e-2 m for this valve travel.
    let level_move = level_at_6k - level_at_4k;
    let output_move = output_at_6k - output_at_4k;
    let forced_by_a_p_loop = output_move / GAIN_PER_M;
    assert!(
        output_move < 0.0,
        "the load falls over the run, so the loop must close its drain: Δu = {output_move}"
    );
    assert!(
        level_move > 0.0 && level_move.abs() < forced_by_a_p_loop.abs(),
        "the loop moved its output by {output_move} while its level moved {level_move} m; \
         a proportional loop with gain {GAIN_PER_M} would have been forced to move the \
         level by {forced_by_a_p_loop} m to do that"
    );
}

// -------------------------------------------------------------- the startup

/// `level_valve.opening` and `initial_output` are equal in the file, and this is
/// what that buys.
///
/// The PI loop's memory is stored in OUTPUT units and seeded at load by
/// `b = u − K·e` against the error standing there; the first `update` then
/// returns `K·e + b`, which is `u` again. So a demo that declares the two equal
/// starts with the valve exactly where the file put it, and a demo that declared
/// them apart would open with a step it invented.
///
/// **The bound is derived, not chosen** (M8.3's
/// `a-tight-looking-bound-is-still-too-loose`). Two roundings stand between the
/// declared 0.2 and the value that comes back: the one that forms
/// `b = 0.2 − 0.25·(−2.0) = 0.7`, and the one that forms `0.25·(−2.0) + b`. Each
/// is at most half an ulp of 0.7, and 0.7 lies in `[0.5, 1)` where an ulp is
/// `f64::EPSILON / 2` — so the round trip cannot exceed `f64::EPSILON / 2`. It
/// comes back 2 ulp of 0.2 low, which is `f64::EPSILON / 4`.
#[test]
fn the_demo_starts_without_stepping_its_own_actuator() {
    let mut engine = build(DEMO);
    assert_eq!(
        valve_opening(&engine, "level_valve"),
        DECLARED_OPENING,
        "before the first tick the drain is where the file declared it"
    );

    run(&mut engine, 1);

    let bound = f64::EPSILON / 2.0;
    let first_output = output(&engine);
    let step = first_output - DECLARED_OPENING;
    assert!(
        step.abs() <= bound,
        "the loop's first output must be the position it was seeded with: \
         {first_output} against {DECLARED_OPENING}, a step of {step} outside the \
         derived {bound}"
    );
    assert_eq!(
        valve_opening(&engine, "level_valve"),
        first_output,
        "and what the loop reports is what it wrote"
    );
}

// ---------------------------------------------------------------- the bound

/// The gain bound the file's header quotes, with BOTH sides run — and the bound
/// is no longer the one M8.4 measured.
///
/// **M8.4 wrote this as a stability bound and M9.0 demoted it to a tuning one.**
/// At the settled operating point the drain sits at ~0.376, so a setpoint step of
/// +1 m subtracts `gain_per_m` from the output in a single tick. Below the bound
/// that is a step the plant absorbs; at it the output reaches exactly zero and the
/// drain branch is shut in one tick — which used to diverge the Newton solver and
/// now does not (DESIGN §11). So `gain_per_m = 0.4` is still the gain that clamps
/// and `0.25` is still the gain that ships, but the reason 0.4 is not shipped is
/// that a loop which slams its actuator against a limit on a one-metre setpoint
/// move is badly tuned, not that it breaks the engine.
///
/// **A gate whose failing half stops failing has to be re-premised, not
/// inverted.** The old third assertion was "and this one diverges"; deleting it
/// would leave a test that only runs the gains that work, which is the shape
/// M8.4's own docstring refuses ("a margin claim with only the passing side
/// measured is not a margin claim"). What is asserted instead is the three-way
/// separation that survives the fix — two gains that never touch the clamp, one
/// that reaches it EXACTLY, and the recovery that proves reaching it is now a
/// transient rather than an ending.
#[test]
fn the_shipped_gain_clamps_later_than_the_neighbour_that_reaches_zero() {
    /// Settle, then step the setpoint a metre up — the direction that SUBTRACTS
    /// from the output, because `error = measurement − setpoint`.
    ///
    /// Returns the output at the end, the lowest output seen after the step, the
    /// level at the end, and any failure.
    fn step_setpoint(gain: &str) -> (f64, f64, f64, Option<String>) {
        let src = DEMO.replace(
            &format!("gain_per_m = {GAIN_PER_M}"),
            &format!("gain_per_m = {gain}"),
        );
        let mut engine = build(&src);
        run(&mut engine, 3_000);
        engine
            .apply(Command::SetSetpoint {
                loop_id: LoopId(0),
                value: ControlledValue::Level {
                    m: Meter(SETPOINT_M + 1.0),
                },
            })
            .expect("a metre up is inside the tank");
        let mut floor = output(&engine);
        for _ in 0..2_000 {
            if let Err(e) = engine.tick() {
                return (output(&engine), floor, level(&engine), Some(e.to_string()));
            }
            floor = floor.min(output(&engine));
        }
        (output(&engine), floor, level(&engine), None)
    }

    // Two gains that absorb the step without ever reaching the clamp.
    for gain in ["0.25", "0.35"] {
        let (opening, floor, level, failure) = step_setpoint(gain);
        assert!(
            failure.is_none(),
            "gain {gain} must absorb the step: {}",
            failure.unwrap_or_default()
        );
        assert!(
            floor > 0.0,
            "gain {gain} must not shut the drain at any point: floor {floor}"
        );
        assert!(
            (level - (SETPOINT_M + 1.0)).abs() < 0.2,
            "gain {gain} must reach the stepped setpoint: {level} m against \
             {} m (output {opening})",
            SETPOINT_M + 1.0
        );
    }

    // And the neighbour that does reach it. `0.4` against an operating point of
    // ~0.376 lands on exactly zero, which is the arithmetic the file's header
    // quotes; the assertion is `== 0.0` rather than a band because it is a clamp
    // and not a near miss.
    let (opening, floor, level, failure) = step_setpoint("0.4");
    assert_eq!(
        floor, 0.0,
        "gain 0.4 against an operating point of ~0.376 must reach the clamp exactly"
    );
    assert!(
        failure.is_none(),
        "SOLVER REGRESSED: shutting the drain in one tick used to diverge the \
         Newton solver and must not any more (DESIGN §11): {}",
        failure.unwrap_or_default()
    );

    // The recovery is the half that makes the clamp a transient. A plant that
    // merely failed to error could sit at the clamp for ever with the tank
    // filling past its roof; this says the loop came off the limit and parked
    // the level on the stepped setpoint like its gentler neighbours did.
    assert!(
        opening > 0.0,
        "gain 0.4 must come back OFF the clamp once the level reaches its new \
         setpoint; it is still at {opening}"
    );
    assert!(
        (level - (SETPOINT_M + 1.0)).abs() < 0.2,
        "gain 0.4 must reach the stepped setpoint like the gains that never \
         clamped: {level} m against {} m",
        SETPOINT_M + 1.0
    );
}
