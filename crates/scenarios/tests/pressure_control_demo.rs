//! M10.1: `scenarios/vessel_pressure_control.toml`, as shipped.
//!
//! The seam's own gates live in `pressure_control_reference.rs` and run on
//! fixtures built to expose one behaviour each. What none of them can see, and
//! this file does, is the **demo file**: the first plant in `scenarios/` whose
//! `[[controls]]` table regulates something other than a level, and therefore the
//! first place the `setpoint_bar`/`gain_per_bar` pair, a vessel measurement, the
//! tick order and a real compressible network are exercised together by a file a
//! reader is invited to run.
//!
//! Three claims the file's header makes, each pinned here so that a change to the
//! plant's sizing fails a test rather than quietly making the comment wrong:
//!
//! 1. **It holds, and parked it holds the WRONG number.** The counterfactual is
//!    measured rather than assumed, and it comes out differently from M8.4's: a
//!    tank filling through a fixed drain ran to its roof, but a vessel venting
//!    through a fixed valve *does* find an equilibrium, because the vent's flow
//!    rises with pressure. So the manual run settles — at 25.20 bar, a quarter
//!    above the setpoint the auto run holds to six figures. A separation of five
//!    bar, not a divergence.
//! 2. **It starts without stepping its own actuator**, which is what the file's
//!    equal `vent_valve.opening` and `initial_output` are for.
//! 3. **Its gain has a bound on BOTH sides**, which the level demo's did not. The
//!    vent settles interior at 0.5582, so a one-bar setpoint step moves the output
//!    by `gain_per_bar` in either direction: 0.5582 reaches 0 upward and 0.4418
//!    reaches 1 downward. Both are run, because a margin claim with only the
//!    passing side measured is not a margin claim.
//!
//! Claim 3 is also where this file closes a gap M8.4 recorded as open. That demo
//! never reached either saturation arm from a wired run, and neither does this
//! one — but here the arms are reachable by a command from the shipped file
//! rather than only from a fixture, because the operating point was sized to sit
//! in the middle. The clamp is exercised below on the plant as shipped.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{ControlledValue, LoopId, NodeKind};
use refinery_core::snapshot::Command;
use refinery_core::units::Pascal;
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/vessel_pressure_control.toml");

/// The file's own declared numbers, named once so an assertion below cannot drift
/// away from the plant it is about.
const SETPOINT_BAR: f64 = 20.0;
const GAIN_PER_BAR: f64 = 0.10;
const DECLARED_OPENING: f64 = 0.30;
const DECLARED_PRESSURE_BAR: f64 = 12.0;

/// Where the shipped file settles, measured over 20 000 ticks before any
/// assertion here was written.
const SETTLED_OPENING: f64 = 0.558224;

fn build(src: &str) -> Engine {
    build_engine_or_panic(src)
}

fn build_engine_or_panic(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("a shipped scenario must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("a shipped scenario must build: {e}"))
}

/// The measurement the loop ACTED ON, which is what a snapshot reports and what
/// the file's header quotes. Bar, because that is the unit the file is written in
/// and every number in this test's messages should be comparable to it by eye.
fn pressure_bar(engine: &Engine) -> f64 {
    match engine.snapshot().controls[0]
        .measurement
        .expect("a stored quantity is measured from load")
    {
        ControlledValue::Pressure { pa } => pa.value() / 1.0e5,
        other => panic!("the demo's loop measures a pressure, not {other:?}"),
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

/// Long enough for the integral term to finish: the file's header records the
/// pressure inside 0.01 bar from tick 1993 and holding.
const SETTLE_TICKS: u64 = 20_000;

// ------------------------------------------------------- the two controls

/// **Written first, and they are the controls rather than the gates.**
///
/// M9.3b's habit: a run that sits still passes every assertion about where it
/// settled, and a loop that writes nothing passes every assertion about the
/// plant's own self-regulation. So before anything else, the pressure must
/// actually move and the vent must actually leave the position the file declares
/// for it.
#[test]
fn the_demo_moves_its_pressure_and_writes_its_actuator() {
    let mut engine = build(DEMO);
    let start = pressure_bar(&engine);
    assert!(
        (start - DECLARED_PRESSURE_BAR).abs() < 1.0e-9,
        "the loop's measurement before the first tick is the vessel's declared \
         {DECLARED_PRESSURE_BAR} bar, and it read {start}"
    );
    assert!(
        (valve_opening(&engine, "vent_valve") - DECLARED_OPENING).abs() < 1.0e-12,
        "the vent starts at the opening the file declares"
    );

    run(&mut engine, SETTLE_TICKS);

    let moved = pressure_bar(&engine) - start;
    assert!(
        moved > 7.0,
        "the receiver must actually build pressure over the run, and moved only \
         {moved} bar. Every assertion in this file is vacuous on a plant that sits \
         still"
    );
    let travel = (valve_opening(&engine, "vent_valve") - DECLARED_OPENING).abs();
    assert!(
        travel > 0.2,
        "the loop must actually write its actuator, and the vent moved only \
         {travel} from its declared opening. A loop that writes nothing is \
         indistinguishable from the plant regulating itself"
    );
}

/// **The wire form, and it is here because the mutation pass found nothing else
/// covering it.**
///
/// §12 predicted that giving the second `ControlledValue` variant the same serde
/// tag as the first would be "caught by the byte-identity prediction". **It is
/// not.** Measured both ways:
///
/// - tagging the EXISTING level variant `"pressure"` moves `tank_level_control`
///   and the corpus exits nonzero on both fidelities — that is the mutation the
///   note's prose describes;
/// - tagging the NEW pressure variant `"level"` moves **nothing**. The corpus
///   compares against a baseline recorded before this slice, so the only plant
///   whose bytes change is the one that is new in the same slice and has no
///   baseline row. Zero rows moved, whole suite green, and the demo then reports
///   `{"variable":"level","pa":2000000.0}` — a frontend would draw a pressure
///   faceplate labelled as a level.
///
/// The general rule, which is the reason this gate is worth its lines: **a
/// regression anchor protects the old files and has no power over the file the
/// slice adds.** Anything new needs an assertion of its own, and for a wire form
/// that assertion has to be on the SERIALIZED bytes rather than on the enum —
/// matching `ControlledValue::Pressure { .. }` in Rust passes under any tag.
#[test]
fn the_demo_reports_its_variable_on_the_wire_as_pressure() {
    let engine = build(DEMO);
    let json = serde_json::to_string(&engine.snapshot()).expect("a snapshot serializes");

    assert!(
        json.contains(r#""setpoint":{"variable":"pressure","pa":2000000.0}"#),
        "the loop's setpoint must reach a frontend tagged as a pressure, in Pascals: the snapshot \
         said {json}"
    );
    assert!(
        !json.contains(r#""variable":"level""#),
        "nothing on this plant is a level, so no control value may be tagged as one. A second \
         variant sharing the first's serde tag is invisible to the corpus baseline, because the only \
         plant it changes is this one"
    );
}

// ------------------------------------------------ claim 1: the counterfactual

/// **Claim 1, and it is the gate this file exists for.**
///
/// The same plant with the loop parked in MANUAL — the loop present, measuring
/// and reporting, writing nothing — against the same plant in AUTO. The two runs
/// differ in exactly one word of scenario text.
///
/// **The counterfactual came out differently from M8.4's, and the difference is
/// physical rather than a sizing accident.** A tank filling through a fixed drain
/// self-regulates only weakly (`ρgh` grows with level), so that demo's manual run
/// climbed to the tank's roof and the gate could assert divergence. A vessel
/// vents through a valve whose flow grows with the vessel's own pressure, which is
/// a much stiffer feedback, so this manual run genuinely **settles** — at 25.20
/// bar. So the assertion cannot be "it runs away"; it is that the equilibrium it
/// finds is the wrong one, five bar above the setpoint, while the loop holds the
/// right one to six figures.
#[test]
fn the_demo_holds_its_setpoint_where_the_parked_loop_settles_five_bar_high() {
    let mut auto = build(DEMO);
    run(&mut auto, SETTLE_TICKS);
    let held = pressure_bar(&auto);

    let manual_src = DEMO.replace("mode = \"auto\"", "mode = \"manual\"");
    assert_ne!(
        manual_src, DEMO,
        "the counterfactual must differ from the shipped file, and the swap \
         matched nothing"
    );
    let mut manual = build(&manual_src);
    run(&mut manual, SETTLE_TICKS);
    let parked = pressure_bar(&manual);

    assert!(
        (held - SETPOINT_BAR).abs() < 1.0e-3,
        "the loop must hold the setpoint it declares: it held {held} bar against \
         {SETPOINT_BAR}"
    );
    assert!(
        parked - SETPOINT_BAR > 4.0,
        "the parked loop must settle somewhere the auto loop plainly does not, and \
         settled at {parked} bar — only {} above setpoint. If a fixed vent lands \
         near the setpoint anyway, this whole gate is measuring the plant's own \
         self-regulation rather than the controller",
        parked - SETPOINT_BAR
    );

    // The parked loop still MEASURES and still reports — it is the same plant with
    // a truthful faceplate, not a plant with the loop deleted. That is what makes
    // the comparison above a comparison.
    assert!(
        (valve_opening(&manual, "vent_valve") - DECLARED_OPENING).abs() < 1.0e-12,
        "a parked loop writes nothing, so the vent must still sit at its declared \
         opening"
    );
    assert!(
        (output(&manual) - DECLARED_OPENING).abs() < 1.0e-12,
        "in MANUAL the faceplate TRACKS the real opening rather than reporting a \
         number the loop would have written"
    );
}

// -------------------------------------------- claim 2: the startup transient

/// **Claim 2.** The loop's memory is seeded by `b = u − K·e` at load, so the first
/// output is exactly the declared `initial_output` whatever the gain is — and
/// because the file declares `vent_valve.opening` equal to it, the loop starts
/// without stepping its own actuator.
///
/// The bound is derived rather than chosen: the two numbers are the same decimal
/// literal in the same file, so the only thing between them is the
/// back-calculation's own round trip through `b = u − K·e` and back. A few ULP on
/// a number of order 1 is 1e-15; the assertion allows 1e-12 and the measurement
/// sits at 0.
#[test]
fn the_demo_starts_without_stepping_its_own_actuator() {
    let mut engine = build(DEMO);
    engine.tick().expect("tick 1");

    let step = (output(&engine) - DECLARED_OPENING).abs();
    assert!(
        step < 1.0e-12,
        "the first output must be the declared `initial_output` of \
         {DECLARED_OPENING} whatever the gain is, and it was {} — off by {step}. \
         The integral term is back-calculated from `initial_output` and the error \
         standing at load, so this is an identity of the seeding rather than a \
         property of the tuning",
        output(&engine)
    );
    assert!(
        (valve_opening(&engine, "vent_valve") - DECLARED_OPENING).abs() < 1.0e-12,
        "and the valve it wrote must still be where the file declared it"
    );
}

// ------------------------------------------------ claim 3: the gain bound

/// The gain that reaches a clamp in one tick, per side, from the settled opening.
///
/// Raising the setpoint by one bar makes the error one bar more negative, which
/// subtracts `gain_per_bar` from the output — so a gain of `SETTLED_OPENING`
/// lands on 0. Lowering it by one bar adds `gain_per_bar` — so a gain of
/// `1 − SETTLED_OPENING` lands on 1. The level demo had only the first of these,
/// because its drain settled at 0.376 and its setpoint was only ever stepped up.
const STEP_BAR: f64 = 1.0;

fn step_setpoint_and_read(gain_per_bar: f64, to_bar: f64) -> f64 {
    let src = DEMO.replace(
        &format!("gain_per_bar = {GAIN_PER_BAR:.2}"),
        &format!("gain_per_bar = {gain_per_bar}"),
    );
    assert_ne!(src, DEMO, "the gain swap must match the shipped literal");
    let mut engine = build(&src);
    run(&mut engine, SETTLE_TICKS);

    engine
        .apply(Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Pressure {
                pa: Pascal(to_bar * 1.0e5),
            },
        })
        .expect("a pressure setpoint on a pressure loop is legal");
    engine.tick().expect("the tick after the step");
    output(&engine)
}

/// **Claim 3, and both sides are run.**
///
/// A margin claim with only the passing side measured is not a margin claim: if
/// the shipped gain sits comfortably inside the bound but the bound itself was
/// never reached, the file's header is quoting a number nobody has seen the plant
/// produce. So the neighbours that DO reach each clamp are run here beside it.
///
/// This also closes, on the shipped file, the gap M8.4 recorded: that demo's
/// wired run never left `[0.194, 0.384]`, so its anti-windup arm was reachable
/// only from a fixture. Sizing this plant's operating point to sit interior is
/// what makes both arms reachable from the demo itself, by a command.
#[test]
fn the_shipped_gain_clamps_on_neither_side_where_its_neighbours_clamp_on_both() {
    // First, the settled operating point the two bounds are computed from. If this
    // moves, both bounds below move with it and the file's header is wrong.
    let mut shipped = build(DEMO);
    run(&mut shipped, SETTLE_TICKS);
    let settled = output(&shipped);
    assert!(
        (settled - SETTLED_OPENING).abs() < 1.0e-5,
        "the vent settles at {SETTLED_OPENING}, and this run settled at {settled}. \
         Both gain bounds below are computed from that number, and the file's \
         header quotes it"
    );
    assert!(
        settled > 0.05 && settled < 0.95,
        "the operating point must sit INTERIOR — off both limits — or this plant \
         repeats M8.4's coverage gap and cannot reach its own saturation arms. It \
         settled at {settled}"
    );

    // The shipped gain reaches neither clamp, in either direction.
    let up = step_setpoint_and_read(GAIN_PER_BAR, SETPOINT_BAR + STEP_BAR);
    let down = step_setpoint_and_read(GAIN_PER_BAR, SETPOINT_BAR - STEP_BAR);
    assert!(
        up > 0.0 && down < 1.0,
        "the shipped gain of {GAIN_PER_BAR}/bar must absorb a one-bar step either \
         way without saturating, and produced {up} (up) and {down} (down)"
    );
    assert!(
        (up - (settled - GAIN_PER_BAR)).abs() < 1.0e-6
            && (down - (settled + GAIN_PER_BAR)).abs() < 1.0e-6,
        "a one-bar setpoint step moves the output by exactly `gain_per_bar`, which \
         is what makes the two bounds arithmetic rather than fitted: from {settled} \
         the step produced {up} and {down}"
    );

    // **The bound and the clamp are two claims, and the first draft of this gate
    // conflated them.** A gain of exactly `settled` lands the output ON zero, so
    // the clamp never engages — measured at 3.4e-10 rather than at 0, because the
    // loop is still that far from its own fixed point after 20 000 ticks. So the
    // arithmetic is asserted at the bound and the clamp is asserted just past it.
    let at_bound_low = step_setpoint_and_read(settled, SETPOINT_BAR + STEP_BAR);
    assert!(
        at_bound_low.abs() < 1.0e-6,
        "a gain of {settled}/bar is precisely the distance from the settled \
         opening to 0, so a one-bar step up must land the vent ON its lower limit \
         — it produced {at_bound_low}"
    );
    let clamps_low = step_setpoint_and_read(settled * 1.1, SETPOINT_BAR + STEP_BAR);
    assert_eq!(
        clamps_low, 0.0,
        "and ten percent past that bound the CLAMP must engage, producing exactly \
         0 rather than a small negative number reaching the valve"
    );

    // And the same pair on the side the level demo had no analogue of.
    let at_bound_high = step_setpoint_and_read(1.0 - settled, SETPOINT_BAR - STEP_BAR);
    assert!(
        (at_bound_high - 1.0).abs() < 1.0e-6,
        "a gain of {}/bar is the distance from the settled opening to 1, so a \
         one-bar step DOWN must land the vent ON its upper limit — it produced \
         {at_bound_high}. This is the side a level loop on a drain never has, \
         because that demo's setpoint was only ever stepped up",
        1.0 - settled
    );
    let clamps_high = step_setpoint_and_read((1.0 - settled) * 1.1, SETPOINT_BAR - STEP_BAR);
    assert_eq!(
        clamps_high, 1.0,
        "and ten percent past THAT bound the upper clamp must engage exactly"
    );
}

/// The clamped loop must RECOVER, not stay pinned.
///
/// M8.4's own gain gate was re-premised by M9.0 for exactly this reason: a gain
/// that drives the actuator onto a limit used to stall the solver, and now
/// clamps, recovers and parks on the stepped setpoint. The same claim has to be
/// re-measured here rather than inherited, because it is a claim about this
/// plant's hydraulics and not about the controller.
#[test]
fn a_clamped_vent_recovers_and_parks_on_the_stepped_setpoint() {
    let mut shipped = build(DEMO);
    run(&mut shipped, SETTLE_TICKS);
    let settled = output(&shipped);

    // Ten percent past the bound, so the clamp genuinely engages rather than the
    // output merely landing on the limit — see the gate above for why those are
    // two different things.
    let src = DEMO.replace(
        &format!("gain_per_bar = {GAIN_PER_BAR:.2}"),
        &format!("gain_per_bar = {}", settled * 1.1),
    );
    let mut engine = build(&src);
    run(&mut engine, SETTLE_TICKS);
    engine
        .apply(Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Pressure {
                pa: Pascal((SETPOINT_BAR + STEP_BAR) * 1.0e5),
            },
        })
        .expect("a pressure setpoint on a pressure loop is legal");

    engine.tick().expect("the tick after the step");
    assert_eq!(
        output(&engine),
        0.0,
        "the premise of this test is that the vent is driven onto its lower clamp"
    );

    run(&mut engine, SETTLE_TICKS);
    let held = pressure_bar(&engine);
    assert!(
        (held - (SETPOINT_BAR + STEP_BAR)).abs() < 1.0e-2,
        "a loop that clamps must come back off the limit and hold the new \
         setpoint of {} bar, and it held {held}",
        SETPOINT_BAR + STEP_BAR
    );
    assert!(
        output(&engine) > 0.0,
        "and it must actually leave the clamp, rather than holding the setpoint by \
         accident with a shut vent"
    );
}
