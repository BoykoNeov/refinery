//! M20.1: the fourth controlled variable — a valve holding the FLOW in its own
//! line (docs/DESIGN.md §24).
//!
//! **A flow is stored AND absent at load.** `Pipe::stream.mass_flow` is on the
//! graph, but the loader writes `Stream::stagnant`'s zero there: an initialiser,
//! not a declaration, and indistinguishable from a valve shut on tick 400. So the
//! loop measures the last hydraulic SOLUTION, which is `None` at load, and M19's
//! rule is reused whole — no measurement, no action; a PI loop's memory stays
//! PENDING until the first measurement seeds it.
//!
//! **The first reverse loop on a valve.** Opening a valve raises the flow in its
//! own pipe; a valve has exactly one inlet and one outlet, so the loader checks
//! that sign in ONE hop and the loop must declare `action = "reverse"`. (Since
//! M29 a level or pressure loop's valve is held to its side of the holdup the
//! same way: a fill must say reverse, a drain may not — docs/DESIGN.md §32.)
//!
//! **Zero flow is a measurement**, unlike M19's stagnant outlet: the solve
//! computes it. And a flow running BACKWARDS is published as measured, never
//! clipped — the loop then pins its valve open (E11).
//!
//! The demo is `scenarios/tank_flow_control.toml`, the M1 reference plant with
//! its discharge valve under a flow loop on the valve's OUTLET pipe, `fill_line`.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{ControlMode, ControlledValue, LoopId, NodeKind};
use refinery_core::snapshot::{Command, ControlSnapshot};
use refinery_core::units::{Kelvin, KgPerSec, Meter, Pascal};
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/tank_flow_control.toml");
const LEVEL_DEMO: &str = include_str!("../../../scenarios/tank_level_control.toml");

/// The file's declared numbers, named once so no assertion can drift away from
/// the plant it is about. `tuning_lands` asserts the file still declares them.
const SETPOINT: f64 = 12.0;
const DECLARED_OUTPUT: f64 = 0.4;
const GAIN: f64 = 0.02;
const INTEGRAL_TIME_S: f64 = 10.0;
const DT_S: f64 = 1.0;

// ------------------------------------------------------------------- helpers

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the fixture must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("the fixture must build: {e}"))
}

/// The text of the refusal a scenario earns, whichever stage produced it.
/// `what` names the case, so a plant that loads says WHICH refusal went
/// missing: the mutation pass reads that line to know a catch fired for its
/// own reason rather than for the first case in the sweep that happened to load.
fn refusal(what: &str, src: &str) -> String {
    match refinery_scenarios::load_str(src) {
        Err(e) => e.to_string(),
        Ok(file) => match refinery_scenarios::build_engine(&file) {
            Ok(_) => panic!("{what}: this plant should not have loaded"),
            Err(e) => e.to_string(),
        },
    }
}

fn tick(engine: &mut Engine, t: u64) {
    engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
}

fn run(engine: &mut Engine, ticks: u64) {
    for t in 1..=ticks {
        tick(engine, t);
    }
}

fn control(engine: &Engine) -> ControlSnapshot {
    engine.snapshot().controls[0].clone()
}

/// The measurement the loop ACTED ON, in kg/s, or `None` if it had none.
fn measured(engine: &Engine) -> Option<f64> {
    control(engine).measurement.map(|m| match m {
        ControlledValue::Flow { kg_per_s } => kg_per_s.value(),
        other => panic!("a flow loop measures a flow, not {other:?}"),
    })
}

fn output(engine: &Engine) -> f64 {
    control(engine).output
}

/// A pipe's PUBLISHED flow [kg/s], as a frontend reads it off the edge snapshot.
fn pipe_flow(engine: &Engine, name: &str) -> f64 {
    engine
        .snapshot()
        .edges
        .iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("the plant has a pipe '{name}'"))
        .stream
        .mass_flow
        .value()
}

fn valve_opening(engine: &Engine) -> f64 {
    let id = engine
        .graph
        .find_node("discharge_valve")
        .expect("the demo has a discharge valve");
    match engine.graph.node(id).kind {
        NodeKind::Valve { opening, .. } => opening,
        ref other => panic!("discharge_valve is a valve, not {other:?}"),
    }
}

fn set_setpoint(engine: &mut Engine, kg_per_s: f64) {
    engine
        .apply(Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Flow {
                kg_per_s: KgPerSec(kg_per_s),
            },
        })
        .unwrap_or_else(|e| panic!("the setpoint command must be accepted: {e}"));
}

/// Replace `from` with `to` in `src`, asserting that it landed: a substitution
/// that finds nothing returns the fixture unchanged, and a gate built on it
/// would then test the shipped plant while saying it tests another (M19.1).
fn swap(src: &str, from: &str, to: &str) -> String {
    assert!(
        src.contains(from),
        "the fixture's substitution must land: `{from}` is not in the plant"
    );
    src.replace(from, to)
}

fn manual(src: &str) -> String {
    swap(src, r#"mode = "auto""#, r#"mode = "manual""#)
}

/// The demo with its gain replaced — the plant is untouched.
fn with_gain(gain: f64) -> String {
    swap(
        DEMO,
        &format!("gain_per_kg_per_s = {GAIN}\n"),
        &format!("gain_per_kg_per_s = {gain:?}\n"),
    )
}

/// The demo with its valve starting at `opening`, and the loop's memory with it.
fn with_opening(opening: f64) -> String {
    let plant = swap(
        DEMO,
        &format!("opening = {DECLARED_OUTPUT}\n"),
        &format!("opening = {opening:?}\n"),
    );
    swap(
        &plant,
        &format!("initial_output = {DECLARED_OUTPUT}\n"),
        &format!("initial_output = {opening:?}\n"),
    )
}

/// `sp − m`: a flow loop is reverse acting, so a flow BELOW setpoint is a
/// positive error and opens the valve — the same subtraction
/// `ControlledValue::error` makes for `ControlAction::Reverse`.
fn error(measurement: f64) -> f64 {
    SETPOINT - measurement
}

/// One tick of a loop that is PINNED at `u = 1`, checked against the law.
///
/// "Pinned" is not "exactly 1 on every tick" (M18's finding (iv)). A tick on the
/// clamp back-calculates the memory to `1 − K·e`, so the next unclamped output is
/// `1 + K·(e₊ − e)`: when the error shrinks by a hair, the loop dips off the clamp
/// by exactly `K·(e − e₊)`, accumulates the integral step — which here, against a
/// large error, is far bigger than the dip — and is back on the clamp the tick
/// after. So a dip never lasts two ticks, and its depth is known to the bit.
fn assert_pinned(t: u64, (u, e): (f64, f64), (u_before, e_before): (f64, f64)) {
    if u == 1.0 {
        return;
    }
    assert_eq!(
        u_before, 1.0,
        "tick {t}: a pinned loop is off the clamp for one tick at most, and read {u}          after {u_before}"
    );
    let dip = 1.0 + GAIN * (e - e_before);
    assert!(
        (u - dip).abs() < 1.0e-15,
        "tick {t}: a dip off the clamp is exactly 1 + K·(e₊ − e) = {dip}, and read {u}"
    );
}

#[test]
fn tuning_lands() {
    for line in [
        format!("setpoint_kg_per_s = {SETPOINT:?}\n"),
        format!("gain_per_kg_per_s = {GAIN}\n"),
        format!("integral_time_s = {INTEGRAL_TIME_S:?}\n"),
        format!("initial_output = {DECLARED_OUTPUT}\n"),
        format!("opening = {DECLARED_OUTPUT}\n"),
        format!("dt = {DT_S:?}"),
        r#"measurement = { pipe = "fill_line", variable = "flow" }"#.to_owned(),
        r#"action = "reverse""#.to_owned(),
    ] {
        assert!(
            DEMO.contains(&line),
            "the demo no longer declares `{line}`, which this file's constants mirror"
        );
    }
}

// ------------------------------------------------ gate 1: the blind start

/// **Gate 1. At load and after tick 1 the loop has NO measurement, and says so on
/// the bytes.**
///
/// The pipe's stored flow at load is `Stream::stagnant`'s zero, and a loop that
/// read it (mutation 1) would publish `{"variable":"flow","kg_per_s":0.0}` here
/// — a number nothing solved, on a plant whose first tick flows 11.09 kg/s. So
/// the faceplate must carry no `measurement` key at all, and no `null` either,
/// and both early faceplates must deserialize back. Across the blind tick the loop
/// writes nothing: the valve keeps its declared 0.4 and the faceplate tracks it.
#[test]
fn the_loop_starts_blind_and_the_snapshot_says_nothing_rather_than_a_number() {
    let mut engine = build(DEMO);
    assert_eq!(
        engine
            .graph
            .pipe(
                engine
                    .graph
                    .edge_ids()
                    .find(|e| engine.graph.pipe(*e).name == "fill_line")
                    .expect("fill_line")
            )
            .stream
            .mass_flow
            .value(),
        0.0,
        "the counterfactual's premise: the graph's stored flow at load IS the \
         initialiser zero a stored-stream reader would have taken"
    );

    for label in ["at load", "after tick 1"] {
        if label == "after tick 1" {
            tick(&mut engine, 1);
            assert!(
                (pipe_flow(&engine, "fill_line") - 11.0867).abs() < 1.0e-3,
                "and the plant is NOT at zero flow: tick 1 solves 11.09 kg/s"
            );
        }
        let faceplate = control(&engine);
        let json = serde_json::to_string(&faceplate).expect("a faceplate serializes");
        assert!(
            !json.contains("measurement") && !json.contains("null"),
            "{label}: a flow does not exist before the first solve, so the faceplate \
             must carry no `measurement` key and no null — it published {json}"
        );
        let back: ControlSnapshot = serde_json::from_str(&json).expect("round trips");
        assert_eq!(back.measurement, None, "{label}: absent reads back as None");
        assert_eq!(back, faceplate, "{label}: the faceplate round-trips whole");
        assert_eq!(
            faceplate.output, DECLARED_OUTPUT,
            "{label}: with nothing to act on the faceplate tracks the valve's own 0.4"
        );
        assert_eq!(
            valve_opening(&engine),
            DECLARED_OUTPUT,
            "{label}: no measurement, no action — the valve keeps its declared opening"
        );
    }

    tick(&mut engine, 2);
    let json = serde_json::to_string(&control(&engine)).expect("serializes");
    assert!(
        json.contains(r#""measurement":{"variable":"flow","kg_per_s":"#),
        "after tick 2 the loop has measured tick 1's flow, and publishes it: {json}"
    );
    let back: ControlSnapshot = serde_json::from_str(&json).expect("round trips");
    assert!(back.measurement.is_some(), "and it reads back as Some");
}

// ------------------------------------------ gate 2: the one-tick identity

/// **Gate 2. The loop acts on the flow the PREVIOUS tick solved into its metered
/// pipe — the number that tick's edge snapshot published — bit for bit, over the
/// whole run.**
///
/// Asserted against the MEASURED pipe, `fill_line`, only. The valve's other
/// pipe, `discharge`, carries the same flow to within the valve node's solver
/// residual and no closer, and the control half of this gate shows it differs on
/// most ticks: an identity asserted against the wrong pipe of the two cannot pass
/// by accident.
#[test]
fn the_loop_acts_on_the_flow_the_previous_tick_solved_into_its_own_pipe() {
    let mut engine = build(DEMO);
    tick(&mut engine, 1);
    let mut previous = pipe_flow(&engine, "fill_line");
    let mut previous_inlet = pipe_flow(&engine, "discharge");
    let (mut moved, mut inlet_differs) = (0, 0);
    for t in 2..=6000 {
        tick(&mut engine, t);
        let acted_on = measured(&engine).unwrap_or_else(|| panic!("tick {t}: measured"));
        assert_eq!(
            acted_on.to_bits(),
            previous.to_bits(),
            "tick {t}: the loop must act on tick {}'s published fill_line flow, {previous} \
             kg/s, and it acted on {acted_on} kg/s",
            t - 1
        );
        if acted_on.to_bits() != previous_inlet.to_bits() {
            inlet_differs += 1;
        }
        let now = pipe_flow(&engine, "fill_line");
        if now != previous {
            moved += 1;
        }
        previous = now;
        previous_inlet = pipe_flow(&engine, "discharge");
    }
    assert!(
        moved > 5000,
        "the identity is only a gate if the flow moves: it moved on {moved} of 5 999 ticks"
    );
    assert!(
        inlet_differs > 0,
        "the control: the valve's INLET pipe must differ from the measurement on at \
         least one tick, or asserting against it could not be told apart"
    );
}

// ------------------------------------ gate 3: the seed at the first measurement

/// **Gate 3. The memory waits for the first measurement, is seeded against it with
/// the loop's own REVERSE sign, and the output then follows the PI law by hand.**
///
/// The first measurement is tick 1's 11.0867 kg/s, 0.91 kg/s BELOW setpoint — the
/// design input the demo's 0.4 opening was chosen for. After tick 2 the output must
/// be exactly `initial_output`: a memory seeded against the wrong sign steps it by
/// `2·K·e ≈ 0.037`, and one seeded against the stored zero (mutation 1) against a
/// 12 kg/s error. After tick 3 it must equal the PI law computed from the two
/// published measurements, in the controller's own order of operations, bit for
/// bit.
#[test]
fn the_memory_is_seeded_at_the_first_measurement_with_the_loops_own_sign() {
    let mut engine = build(DEMO);
    run(&mut engine, 2);
    let e2 = error(measured(&engine).expect("tick 2 measures tick 1's flow"));
    assert!(
        (0.85..0.95).contains(&e2),
        "the fixture's design input: the first measurement sits ~0.91 kg/s below \
         setpoint, and it sits {e2} below"
    );
    assert_eq!(
        output(&engine),
        DECLARED_OUTPUT,
        "the first output after the seed is the declared `initial_output`, exactly"
    );

    tick(&mut engine, 3);
    let e3 = error(measured(&engine).expect("measured"));
    // The memory the seed left, `u − K·e`, plus the one explicit-Euler step tick 2
    // took (its output was interior, so it accumulated).
    let mut b = DECLARED_OUTPUT - GAIN * e2;
    b += GAIN / INTEGRAL_TIME_S * e2 * DT_S;
    let expected = GAIN * e3 + b;
    assert_eq!(
        output(&engine).to_bits(),
        expected.to_bits(),
        "after tick 3 the output is K·(sp − m₃) + b = {expected}, by hand from the \
         published measurements, and the loop produced {}",
        output(&engine)
    );
    assert!(
        output(&engine) > DECLARED_OUTPUT,
        "and it OPENED the valve, a flow below setpoint being a positive error"
    );
}

// ------------------------------------------ gate 4: holds; the twin does not

/// **Gate 4. The loop holds 12 kg/s, off both clamps, and every published step is
/// the PI law; the MANUAL twin falls to 10.207 kg/s.**
///
/// The tanks drain and fill, so the valve must keep opening and the loop never
/// settles to the last bit (M19's finding (iv)). A PI loop following a ramp in its
/// required output carries a steady error, and the loop's own arithmetic says how
/// big: rearranging the update `u₊ − u = K·(e₊ − e) + (K/T_i)·e·dt` gives
/// `e = (Δu − K·Δe)·T_i/(K·dt)`, and on a slow ramp `Δe` is negligible, so
/// `e ≈ Δu·T_i/(K·dt)` — no `G`, no drift rate from a twin. The error itself is
/// then bounded at 1e-2 kg/s, a number the identity explains rather than one
/// chosen to pass.
#[test]
fn the_loop_holds_its_flow_and_the_parked_twin_does_not() {
    let mut engine = build(DEMO);
    run(&mut engine, 5000);
    let mut last = (error(measured(&engine).expect("measured")), output(&engine));
    let mut worst_law_residual: f64 = 0.0;
    let mut second_last_output = f64::NAN;
    for t in 5001..=6000 {
        tick(&mut engine, t);
        let (e, u) = (error(measured(&engine).expect("measured")), output(&engine));
        assert!(
            u > 0.0 && u < 1.0,
            "tick {t}: the shipped tuning stays off both clamps, and read {u}"
        );
        assert!(
            e.abs() < 1.0e-2,
            "tick {t}: the loop holds 12 kg/s to within 1e-2, and is off by {e}"
        );
        let law = GAIN * (e - last.0) + GAIN / INTEGRAL_TIME_S * last.0 * DT_S;
        worst_law_residual = worst_law_residual.max(((u - last.1) - law).abs());
        second_last_output = last.1;
        last = (e, u);
    }
    // Two outputs near 0.47 each carry half an ULP (2.8e-17), and the memory a few
    // more accumulated over the run; 1e-15 is ~20 ULPs of the output and ten orders
    // under the step a wrong law would make (K·e ≈ 1e-4 per tick).
    assert!(
        worst_law_residual < 1.0e-15,
        "every published step satisfies the PI update to the rounding of a difference \
         of two outputs, and the worst missed by {worst_law_residual}"
    );

    // The steady-ramp form, on the last pair of ticks.
    let (e_end, u_end) = last;
    let du = u_end - second_last_output;
    let ramp = du * INTEGRAL_TIME_S / (GAIN * DT_S);
    assert!(
        du > 0.0 && ((e_end - ramp) / e_end).abs() < 1.0e-2,
        "the steady error is the ramp's: Δu·T_i/(K·dt) = {ramp} kg/s against {e_end} \
         (Δu = {du} per tick, the valve still opening as the tanks move)"
    );

    let mut twin = build(&manual(DEMO));
    run(&mut twin, 6000);
    let parked = pipe_flow(&twin, "fill_line");
    assert!(
        (parked - 10.207091).abs() < 1.0e-6,
        "parked at 0.4 the flow falls to 10.207091 kg/s by tick 6 000, and reads {parked}"
    );
    assert_eq!(
        output(&twin),
        DECLARED_OUTPUT,
        "and the faceplate tracks 0.4"
    );
    assert!(
        measured(&twin).is_some(),
        "a MANUAL loop still measures, so its faceplate is truthful"
    );
}

// ---------------------------------------- gate 5: the stability bound, both sides

/// **Gate 5. `K·G` is measured LOCALLY and sits at half the bound; the bound is
/// `2/(2 − dt/T_i)`, computed here rather than typed as 1, and both sides of it
/// behave accordingly.**
///
/// A lag-free plant behind one sample of delay: `e(k+2) = (1 − L)·e(k+1) +
/// L·(1 − a)·e(k)`, `L = K·G`, `a = dt/T_i`. Jury's conditions put the edge at
/// `L = 2/(2 − a)`, 1.0526 here — M19's "`K·G < 1`" is the `T_i → ∞` limit
/// (docs/DESIGN.md §24 fork 6).
///
/// `G` is taken from two one-tick MANUAL runs at openings 0.43 and 0.44, which
/// bracket the settled opening near load. NOT from the AUTO/MANUAL pair: by the end
/// those two plants stand ~0.8 m of head apart and the valve's curve bends between
/// their openings, which reads ~23 against a local ~27 and would size the settling
/// fixture onto the edge (§24's own first draft).
#[test]
fn the_gain_bound_is_two_over_two_minus_dt_over_ti_and_both_sides_behave_so() {
    let one_tick_flow = |opening: f64| {
        let mut engine = build(&manual(&with_opening(opening)));
        tick(&mut engine, 1);
        pipe_flow(&engine, "fill_line")
    };
    let g = (one_tick_flow(0.44) - one_tick_flow(0.43)) / 0.01;
    let loop_gain = GAIN * g;
    assert!(
        (0.4..=0.6).contains(&loop_gain),
        "K·G at the operating point is {loop_gain} (G = {g} kg/s per unit opening); \
         the shipped tuning sits at half the bound"
    );
    let bound = 2.0 / (2.0 - DT_S / INTEGRAL_TIME_S);
    assert!(
        bound > 1.05,
        "the bound is computed from T_i and is above M19's 1: {bound}"
    );

    // Above the bound: a ring that alternates tick to tick and grows onto both clamps.
    let mut ringing = build(&with_gain(1.15 * bound / g));
    let (mut hit_low, mut hit_high) = (false, false);
    let mut errors = Vec::new();
    for t in 1..=400 {
        tick(&mut ringing, t);
        let u = output(&ringing);
        hit_low |= u == 0.0;
        hit_high |= u == 1.0;
        if t > 360 {
            errors.push(error(measured(&ringing).expect("measured")));
        }
    }
    assert!(
        hit_low && hit_high,
        "at K·G = 1.15 × {bound} the ring must reach BOTH clamps within 400 ticks"
    );
    assert!(
        errors.windows(2).all(|w| w[0] * w[1] < 0.0),
        "and the error alternates sign on every tick (the pole beyond −1): {errors:?}"
    );

    // Below the bound: the same plant settles, and never touches a clamp.
    let mut settling = build(&with_gain(0.85 * bound / g));
    let mut clamped = 0;
    for t in 1..=400 {
        tick(&mut settling, t);
        let u = output(&settling);
        if u <= 0.0 || u >= 1.0 {
            clamped += 1;
        }
    }
    let settled = error(measured(&settling).expect("measured"));
    assert_eq!(
        clamped, 0,
        "at K·G = 0.85 × the bound the loop never clamps"
    );
    assert!(
        settled.abs() < 1.0e-2,
        "and it has settled on 12 kg/s by tick 400, off by {settled} — the ring at \
         1.15 × the bound spans the valve's whole range"
    );
}

// ---------------------------------- gate 6: windup, a shut start, a reversed plant

/// **Gate 6a. An unreachable setpoint pins the valve open, and the release is where
/// the SIGNED back-calculated memory says.**
///
/// The fully open valve passes ~25.9 kg/s, so 30 kg/s pins `u = 1` and the loop
/// back-calculates its memory at the clamp every tick instead of integrating. On
/// release to 12 kg/s both a right and a wrong memory land interior on this plant
/// (by hand ~0.64 against ~0.80), so the released output is asserted against the
/// law computed from published numbers, which only the right sign reproduces.
#[test]
fn an_unreachable_setpoint_pins_the_valve_and_the_release_is_signed() {
    let mut engine = build(DEMO);
    run(&mut engine, 300);
    set_setpoint(&mut engine, 30.0);
    let mut pinned_at = None;
    let mut before = (output(&engine), 30.0 - measured(&engine).expect("measured"));
    for t in 301..=400 {
        tick(&mut engine, t);
        let now = (output(&engine), 30.0 - measured(&engine).expect("measured"));
        match pinned_at {
            None if now.0 == 1.0 => pinned_at = Some(t),
            Some(_) => assert_pinned(t, now, before),
            None => {}
        }
        before = now;
    }
    let pinned_at = pinned_at.expect("30 kg/s is above what the open valve passes");
    // §24 predicted "within ten ticks" and the engine takes 31: a PI loop at half
    // its bound climbs onto the clamp along its SLOW pole, the proportional kick
    // covering only `K·Δsp = 0.36` of the 0.53 of travel.
    assert!(
        pinned_at <= 340,
        "the loop reaches the clamp within forty ticks, and took until tick {pinned_at}"
    );
    let ceiling = pipe_flow(&engine, "fill_line");
    assert!(
        (24.0..28.0).contains(&ceiling),
        "fully open the valve passes ~25.9 kg/s, and passes {ceiling}"
    );
    let pinned_error = 30.0 - measured(&engine).expect("measured");
    assert_eq!(output(&engine), 1.0, "still pinned at tick 400");

    set_setpoint(&mut engine, SETPOINT);
    tick(&mut engine, 401);
    // The memory the clamp left on tick 400 — `1 − K·e`, with e against 30 — then
    // the new error against 12.
    let memory = 1.0 - GAIN * pinned_error;
    let released_error = error(measured(&engine).expect("measured"));
    let expected = GAIN * released_error + memory;
    assert_eq!(
        output(&engine).to_bits(),
        expected.to_bits(),
        "the release is K·(12 − m) + (1 − K·(30 − m′)) = {expected}, and read {}",
        output(&engine)
    );
    let unsigned = GAIN * released_error + (1.0 + GAIN * pinned_error);
    assert!(
        (expected - unsigned).abs() > 0.1 && unsigned < 1.0,
        "the control: an unsigned memory releases interior too ({unsigned}), so only \
         the exact value tells them apart"
    );
    // And no windup: the loop never rides the clamp again, comes back to 12 kg/s
    // from ABOVE without undershooting, and does so along its own slow pole — the
    // larger root of `z² − (1 − L)·z − L·(1 − a)`, which over gate 5's band
    // `L ∈ [0.4, 0.6]` lies in `[0.9616, 0.9708]` per tick (§24's "about 29 ticks"
    // is its time constant). A wound-up memory would hold the valve open past the
    // release and then overshoot below 12.
    let a = DT_S / INTEGRAL_TIME_S;
    let slow_pole = |l: f64| ((1.0 - l) + ((1.0 - l).powi(2) + 4.0 * l * (1.0 - a)).sqrt()) / 2.0;
    let (fastest, slowest) = (slow_pole(0.6), slow_pole(0.4));
    let mut previous = pipe_flow(&engine, "fill_line") - SETPOINT;
    for t in 402..=560 {
        tick(&mut engine, t);
        assert!(output(&engine) < 1.0, "tick {t}: released for good");
        let excess = pipe_flow(&engine, "fill_line") - SETPOINT;
        assert!(
            excess > 0.0,
            "tick {t}: the flow comes back from above and does not undershoot: {excess}"
        );
        if (420..=520).contains(&t) {
            let ratio = excess / previous;
            assert!(
                (fastest..=slowest).contains(&ratio),
                "tick {t}: the recovery decays along the slow pole, [{fastest},                  {slowest}] per tick, and decayed by {ratio}"
            );
        }
        previous = excess;
    }
}

/// **Gate 6b. Zero flow is a MEASUREMENT.** The demo with the valve declared shut
/// reads an exact `0.0` kg/s on its metered pipe, and after tick 2 that zero is
/// PRESENT, not absent.
///
/// The output after tick 2 is still exactly 0.0 — gate 3's seed returns
/// `initial_output` on the first measured tick. After tick 3 it is
/// `K·sp + (−K·sp + (K/T_i)·sp·dt)`, which §24 writes as 0.024, computed here in
/// the controller's own order and compared bit for bit. That also guards a
/// quieter rule: a raw output of exactly 0.0 is INSIDE `[0, 1]`, so tick 2's
/// memory accumulated rather than being back-calculated at the clamp. A zero
/// borrowed into M19's `held` rule (mutation 7) leaves the output at 0.0 forever.
///
/// Why the OUTLET: the valve's shut characteristic lives on the pipe leaving it
/// (a device folds into its outlet edge), so `fill_line` reads a real zero while
/// `discharge` is the pump's dead leg and reads a solver residual that no
/// exact-zero rule would touch.
#[test]
fn zero_flow_through_a_shut_valve_is_a_measurement_and_the_loop_opens_it() {
    let mut engine = build(&with_opening(0.0));
    for t in 1..=2 {
        tick(&mut engine, t);
        assert_eq!(
            pipe_flow(&engine, "fill_line"),
            0.0,
            "tick {t}: a shut valve's outlet pipe reads exactly zero"
        );
    }
    // The control: the valve's INLET is the pump's dead leg and reads the solve's
    // residual there, which is not an exact zero by construction. M20.1 showed it
    // on this plant alone (−1.547e-11 kg/s); M23.1 found that number was a
    // property of the solver's COLD SEED, not of the valve — one more atmosphere
    // node moved the seed and made it exactly 0.0 (docs/DESIGN.md §27). So the
    // control is taken over several seeds (the supply level moves the pinned
    // pressures the seed is the mean of): the outlet is exactly zero on every
    // one, the inlet is residual-sized on every one, and not zero on all of them.
    let mut inlet_residuals = Vec::new();
    for supply_level in ["8.0", "7.5", "7.0", "6.5", "6.0", "5.0"] {
        let plant = swap(
            &with_opening(0.0),
            "initial_level_m = 8.0\n",
            &format!("initial_level_m = {supply_level}\n"),
        );
        let mut seeded = build(&plant);
        tick(&mut seeded, 1);
        assert_eq!(
            pipe_flow(&seeded, "fill_line"),
            0.0,
            "supply at {supply_level} m"
        );
        let inlet = pipe_flow(&seeded, "discharge");
        // Newton's `tol_abs`: the node carries no flow, so its bar is that alone.
        assert!(inlet.abs() <= 1.0e-8, "supply at {supply_level} m: {inlet}");
        inlet_residuals.push(inlet);
    }
    assert!(
        inlet_residuals.iter().any(|r| *r != 0.0),
        "the control: on some seed the inlet reads a nonzero residual, {inlet_residuals:?}"
    );
    assert_eq!(
        measured(&engine),
        Some(0.0),
        "after tick 2 the loop has MEASURED tick 1's zero — present, not absent"
    );
    assert_eq!(
        output(&engine),
        0.0,
        "the seeded first output is initial_output"
    );

    tick(&mut engine, 3);
    let mut b = 0.0 - GAIN * SETPOINT;
    b += GAIN / INTEGRAL_TIME_S * SETPOINT * DT_S;
    let expected = GAIN * SETPOINT + b;
    assert_eq!(
        output(&engine).to_bits(),
        expected.to_bits(),
        "after tick 3 the loop opens the valve to K·sp·dt/T_i = {expected}, and read {}",
        output(&engine)
    );
    assert!(
        (expected - 0.024).abs() < 1.0e-15,
        "which is §24's 0.024 to the rounding of its three operations: {expected}"
    );
    assert_eq!(valve_opening(&engine), expected, "and the valve is there");
}

/// **Gate 6c. A plant that drives flow BACKWARDS through the valve flips the
/// loop's sign under it, and the loop pins the valve open** (`docs/DEFERRED.md`
/// E11). The pump is off and the receiving tank is the higher, so `fill_line`
/// runs backwards from tick 1. The measurement is published NEGATIVE — clipping it
/// to zero would be a fabricated number (mutation 6) — and the run stays `Ok` and
/// finite: a pinned actuator, not a divergence.
#[test]
fn a_reversed_plant_pins_the_valve_open_and_the_negative_flow_is_published() {
    let plant = swap(DEMO, "on = true", "on = false");
    let plant = swap(
        &plant,
        "initial_level_m = 8.0\ntemperature_c = 20.0\n\n[nodes.transfer_pump]",
        "initial_level_m = 1.0\ntemperature_c = 20.0\n\n[nodes.transfer_pump]",
    );
    let plant = swap(
        &plant,
        "initial_level_m = 1.0\ntemperature_c = 20.0\n\n[[pipes]]",
        "initial_level_m = 8.0\ntemperature_c = 20.0\n\n[[pipes]]",
    );
    let mut engine = build(&plant);
    tick(&mut engine, 1);
    assert!(
        pipe_flow(&engine, "fill_line") < -5.0,
        "the fixture's premise: flow runs backwards through the valve at 0.4 open, \
         and reads {}",
        pipe_flow(&engine, "fill_line")
    );
    let mut pinned_from = None;
    let mut before = (output(&engine), f64::NAN);
    for t in 2..=200 {
        tick(&mut engine, t);
        let m = measured(&engine).unwrap_or_else(|| panic!("tick {t}: measured"));
        assert!(
            m < 0.0 && m.is_finite(),
            "tick {t}: the backward flow is measured as it is, {m} kg/s — not clipped"
        );
        let now = (output(&engine), error(m));
        assert!(now.0.is_finite(), "tick {t}: no NaN reaches the valve");
        match pinned_from {
            None if now.0 == 1.0 => pinned_from = Some(t),
            Some(_) => assert_pinned(t, now, before),
            None => {}
        }
        before = now;
    }
    let pinned_from = pinned_from.expect("the loop opens the valve all the way");
    assert!(pinned_from < 50, "pinned from tick {pinned_from}");
    let backward = pipe_flow(&engine, "fill_line");
    assert!(
        backward < -13.0,
        "fully open, the backward flow is the larger for it (−13.86 kg/s at 1.0 on \
         §24's probe), and reads {backward}"
    );
}

// ------------------------------------ MANUAL→AUTO on a flow loop

/// **A MANUAL→AUTO transfer is refused before the first solve, and is bumpless
/// after it.** The engine site the transfer runs through was changed to take the
/// hydraulic solution, and its message names the missing quantity by variable.
#[test]
fn manual_to_auto_is_refused_before_the_first_solve_and_bumpless_after_it() {
    let to_auto = Command::SetControllerMode {
        loop_id: LoopId(0),
        mode: ControlMode::Auto,
    };
    let mut engine = build(&manual(DEMO));
    let refused = engine
        .apply(to_auto.clone())
        .err()
        .map(|e| e.to_string())
        .unwrap_or_else(|| panic!("a transfer before the first solve must be refused"));
    assert!(
        refused.contains("has no measurement to transfer against")
            && refused.contains("'fill_line' has no resolved flow yet")
            && !refused.contains("no flow through it"),
        "the refusal names the pipe and the missing flow, and not an outlet's \
         stagnation: {refused}"
    );
    assert_eq!(
        control(&engine).mode,
        ControlMode::Manual,
        "and it stays MANUAL"
    );

    tick(&mut engine, 1);
    engine
        .apply(to_auto)
        .unwrap_or_else(|e| panic!("with a flow solved, the transfer is legal: {e}"));
    tick(&mut engine, 2);
    assert!(
        (output(&engine) - DECLARED_OUTPUT).abs() < 1.0e-15,
        "a bumpless transfer's first output is the opening it took over from, and it \
         was {}",
        output(&engine)
    );
}

// ------------------------------------------------------ gate 7: the wire form

/// **Gate 7. A flow setpoint serializes as `{"variable":"flow","kg_per_s":12.0}`
/// and round-trips; the three existing tags are unchanged.**
///
/// Asserted on the BYTES: a Rust match on `ControlledValue::Flow { .. }` passes
/// under any tag, and a tag that collided with an existing variant (mutation 2)
/// moves no corpus row — the only plant whose bytes change is new in this slice
/// and has no baseline (M10.1's escape).
#[test]
fn the_flow_setpoint_has_its_own_tag_on_the_wire() {
    let cases = [
        (
            ControlledValue::Flow {
                kg_per_s: KgPerSec(12.0),
            },
            r#"{"variable":"flow","kg_per_s":12.0}"#,
        ),
        (
            ControlledValue::Level { m: Meter(6.0) },
            r#"{"variable":"level","m":6.0}"#,
        ),
        (
            ControlledValue::Pressure { pa: Pascal(2.0e6) },
            r#"{"variable":"pressure","pa":2000000.0}"#,
        ),
        (
            ControlledValue::Temperature { k: Kelvin(333.15) },
            r#"{"variable":"temperature","k":333.15}"#,
        ),
    ];
    for (value, wire) in cases {
        let json = serde_json::to_string(&value).expect("serializes");
        assert_eq!(json, wire, "the wire form of {value:?}");
        let back: ControlledValue = serde_json::from_str(wire).expect("deserializes");
        assert_eq!(back, value, "{wire} round-trips");
    }
    // And on the demo's own faceplate, as a frontend receives it.
    let json = serde_json::to_string(&control(&build(DEMO))).expect("serializes");
    assert!(
        json.contains(r#""setpoint":{"variable":"flow","kg_per_s":12.0}"#),
        "the demo publishes its setpoint under the flow tag: {json}"
    );
}

// ------------------------------------------------------ gate 8: the refusals

/// **Gate 8. Every way a file can declare a flow loop that would run and be wrong
/// is refused, each asserting a substring distinctive to its own message.**
///
/// The accepted plant is built first — the demo, and the demo metering the
/// valve's INLET, which the loader admits because the sign argument holds for
/// both — so no case can be passing because the fixture was broken some other way.
#[test]
fn every_malformed_flow_loop_is_refused_for_its_own_reason() {
    build(DEMO);
    build(&swap(
        DEMO,
        r#"pipe = "fill_line""#,
        r#"pipe = "discharge""#,
    ));

    let measuring = |replacement: &str| {
        swap(
            DEMO,
            r#"measurement = { pipe = "fill_line", variable = "flow" }"#,
            replacement,
        )
    };
    // `discharge` split for a leak: its UPSTREAM half keeps the name, and the
    // loader-made `discharge__downstream` half is the pipe that runs INTO the valve
    // — so a lookup among the graph's edges by name would admit it.
    let leaking = format!(
        "{}\n[nodes.outside]\ntype = \"atmosphere\"\n",
        swap(
            DEMO,
            "name = \"discharge\"\nfrom = \"transfer_pump\"\nto = \"discharge_valve\"\n",
            "name = \"discharge\"\nfrom = \"transfer_pump\"\nto = \"discharge_valve\"\n\
             leak_to = \"outside\"\n",
        )
    );
    let leaking_on = |pipe: &str| {
        swap(
            &leaking,
            r#"pipe = "fill_line""#,
            &format!(r#"pipe = "{pipe}""#),
        )
    };
    // A duty unit inline between the valve and the receiving tank, actuated.
    let duty_unit = |kind: &str| {
        let plant = swap(
            DEMO,
            "name = \"fill_line\"\nfrom = \"discharge_valve\"\nto = \"receiving_tank\"",
            "name = \"fill_line\"\nfrom = \"discharge_valve\"\nto = \"unit\"",
        );
        let plant = swap(
            &plant,
            r#"actuator = "discharge_valve""#,
            r#"actuator = "unit""#,
        );
        format!(
            "{plant}\n[nodes.unit]\ntype = \"{kind}\"\nduty_mw = 0.0\n\n[[pipes]]\n\
             name = \"unit_line\"\nfrom = \"unit\"\nto = \"receiving_tank\"\n\
             length_m = 1.0\ndiameter_m = 0.10\n"
        )
    };
    let setpoint = |value: &str| {
        swap(
            DEMO,
            "setpoint_kg_per_s = 12.0",
            &format!("setpoint_kg_per_s = {value}"),
        )
    };

    let cases: Vec<(&str, String, &str)> = vec![
        (
            "node and pipe both",
            measuring(
                r#"measurement = { node = "receiving_tank", pipe = "fill_line", variable = "flow" }"#,
            ),
            "names both `node",
        ),
        (
            "neither node nor pipe",
            measuring(r#"measurement = { variable = "flow" }"#),
            "names neither `node` nor `pipe`",
        ),
        (
            "a flow asked of a node",
            measuring(r#"measurement = { node = "discharge_valve", variable = "flow" }"#),
            "a flow belongs to a PIPE: write `pipe",
        ),
        (
            "a level asked of a pipe",
            measuring(r#"measurement = { pipe = "fill_line", variable = "level" }"#),
            "are measured at a NODE: write `node",
        ),
        (
            "a pipe the file does not declare",
            measuring(r#"measurement = { pipe = "overflow", variable = "flow" }"#),
            "which this file does not declare",
        ),
        (
            "a loader-made half that runs into the valve",
            leaking_on("discharge__downstream"),
            "which this file does not declare",
        ),
        (
            // Pipe names are not otherwise unique: a second `fill_line` from the
            // receiving tank to a spill sink loads as a plant, and a meter naming
            // it must not resolve to whichever came first.
            "a pipe name declared twice",
            format!(
                "{DEMO}
[nodes.spill]
type = \"sink\"
pressure_bar = 1.01325

                 [[pipes]]
name = \"fill_line\"
from = \"receiving_tank\"
to = \"spill\"
                 length_m = 1.0
diameter_m = 0.05
"
            ),
            "pipes by that name, so the meter is ambiguous",
        ),
        (
            "a pipe that declares leak_to",
            leaking_on("discharge"),
            "which declares `leak_to",
        ),
        (
            "a pipe that is not on the valve",
            measuring(r#"measurement = { pipe = "suction", variable = "flow" }"#),
            "is not one of valve 'discharge_valve''s own two pipes",
        ),
        (
            "a cooler",
            duty_unit("cooler"),
            "A duty moves heat and no mass",
        ),
        (
            "a furnace",
            duty_unit("furnace"),
            "A duty moves heat and no mass",
        ),
        (
            "a pump",
            swap(
                DEMO,
                r#"actuator = "discharge_valve""#,
                r#"actuator = "transfer_pump""#,
            ),
            "A pump's `on` is a switch",
        ),
        (
            "action absent",
            swap(DEMO, "action = \"reverse\"\n", ""),
            "declares no `action`. Opening a valve RAISES the flow",
        ),
        (
            "action direct",
            swap(DEMO, r#"action = "reverse""#, r#"action = "direct""#),
            "would shut the valve the moment the flow ran low",
        ),
        (
            "a zero setpoint",
            setpoint("0.0"),
            "is not a finite flow above zero",
        ),
        (
            "a negative setpoint",
            setpoint("-12.0"),
            "is not a finite flow above zero",
        ),
        (
            "a NaN setpoint",
            setpoint("nan"),
            "is not a finite flow above zero",
        ),
        (
            "a level key on a flow loop",
            swap(
                DEMO,
                "setpoint_kg_per_s = 12.0",
                "setpoint_kg_per_s = 12.0\nsetpoint_m = 4.0",
            ),
            "declares `setpoint_m`, which is the Level loop's key",
        ),
        (
            "a flow key on a level loop",
            swap(
                LEVEL_DEMO,
                "gain_per_m = 0.25\n",
                "gain_per_m = 0.25\nsetpoint_kg_per_s = 12.0\n",
            ),
            "declares `setpoint_kg_per_s`, which is the Flow loop's key",
        ),
        (
            "reverse on a level loop's drain (E8's input; M29 refuses it by side)",
            swap(
                LEVEL_DEMO,
                "gain_per_m = 0.25\n",
                "gain_per_m = 0.25\naction = \"reverse\"\n",
            ),
            "which DRAINS 'receiving_tank'",
        ),
    ];
    for (what, plant, expected) in cases {
        let message = refusal(what, &plant);
        assert!(
            message.contains(expected),
            "{what}: expected the refusal to say `{expected}`, and it said: {message}"
        );
    }

    // The same range at runtime, through the one `check_setpoint`.
    let mut engine = build(DEMO);
    for bad in [0.0, -1.0, f64::NAN] {
        let refused = engine
            .apply(Command::SetSetpoint {
                loop_id: LoopId(0),
                value: ControlledValue::Flow {
                    kg_per_s: KgPerSec(bad),
                },
            })
            .err()
            .map(|e| e.to_string())
            .unwrap_or_else(|| panic!("a setpoint of {bad} kg/s must be refused"));
        assert!(
            refused.contains("is not a finite flow above zero"),
            "{refused}"
        );
    }
}
