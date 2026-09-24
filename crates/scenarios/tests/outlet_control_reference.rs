//! M19.1: a furnace holding its own OUTLET — the zero-volume measurement
//! (docs/DESIGN.md §23).
//!
//! **The first loop in this engine whose measurement does not exist at load.** A
//! tank's or vessel's temperature is a field on the graph, exact from load; a
//! furnace or cooler outlet is resolved by each tick's sweep into `NodeStates` and
//! is absent before the first one. The rule §23 states is **no measurement, no
//! action**: the loop writes nothing, its faceplate tracks the actuator, and a PI
//! loop's memory waits — PENDING on its declared `initial_output` — until the
//! first measurement seeds it. A stagnant outlet, whose entry is a held
//! placeholder, is the same state.
//!
//! The demo is `scenarios/furnace_outlet_control.toml`, the M18 heating file with
//! the loop moved from the tank to the heater. Its gates are here beside the
//! fixtures because every one of them is about the same new thing — an absent
//! measurement, and a plant with no lag — rather than about the demo's numbers.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{ControlMode, ControlledValue, LoopId, NodeKind};
use refinery_core::snapshot::{Command, ControlSnapshot};
use refinery_core::units::Kelvin;
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/furnace_outlet_control.toml");

/// The file's declared numbers, named once so no assertion can drift away from
/// the plant it is about.
const SETPOINT_C: f64 = 60.0;
const DECLARED_OUTPUT: f64 = 0.25;
const GAIN_PER_K: f64 = 0.015;
const INTEGRAL_TIME_S: f64 = 10.0;
const DT_S: f64 = 1.0;
const MAX_DUTY_W: f64 = 2.0e6;

// ------------------------------------------------------------------- helpers

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the fixture must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("the fixture must build: {e}"))
}

/// The text of the refusal a scenario earns, whichever stage produced it.
fn refusal(src: &str) -> String {
    match refinery_scenarios::load_str(src) {
        Err(e) => e.to_string(),
        Ok(file) => match refinery_scenarios::build_engine(&file) {
            Ok(_) => panic!("this plant should not have loaded"),
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

/// The measurement the loop ACTED ON, in kelvin, or `None` if it had none.
fn measured_k(engine: &Engine) -> Option<f64> {
    control(engine).measurement.map(|m| match m {
        ControlledValue::Temperature { k } => k.value(),
        other => panic!("an outlet loop measures a temperature, not {other:?}"),
    })
}

fn output(engine: &Engine) -> f64 {
    control(engine).output
}

/// The heater's resolved (published) temperature [K] — NaN before the first tick.
fn outlet_k(engine: &Engine) -> f64 {
    engine
        .snapshot()
        .nodes
        .iter()
        .find(|n| n.name == "heater")
        .expect("the demo has a heater")
        .temperature_k
}

fn heater_duty_w(engine: &Engine) -> f64 {
    let id = engine
        .graph
        .find_node("heater")
        .expect("the demo has a heater");
    match engine.graph.node(id).kind {
        NodeKind::Furnace { duty } => duty.value(),
        ref other => panic!("heater is a furnace, not {other:?}"),
    }
}

fn set_setpoint_c(engine: &mut Engine, c: f64) {
    engine
        .apply(Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Temperature {
                k: Kelvin(c + 273.15),
            },
        })
        .unwrap_or_else(|e| panic!("the setpoint command must be accepted: {e}"));
}

/// The demo with its tuning replaced — the plant is untouched. Both substitutions
/// are asserted to land: a `replace` that finds nothing returns the demo as
/// shipped, and a gate built on it would then be testing the shipped tuning while
/// saying it tests another (the M19.1 mutation pass found gate 5 doing exactly
/// that).
fn tuned(gain_per_k: f64, integral_time_s: f64) -> String {
    assert!(
        DEMO.contains(&format!("gain_per_k = {GAIN_PER_K}\n"))
            && DEMO.contains("integral_time_s = 10.0\n"),
        "the demo no longer declares the tuning this file's constants mirror"
    );
    DEMO.replace(
        &format!("gain_per_k = {GAIN_PER_K}"),
        &format!("gain_per_k = {gain_per_k:?}"),
    )
    .replace(
        "integral_time_s = 10.0",
        &format!("integral_time_s = {integral_time_s:?}"),
    )
}

/// The demo with a FEED VALVE between the source and the heater, so the furnace
/// can be starved of flow. `opening` is declared; the loop's mode is substituted.
fn with_feed_valve(opening: &str, mode: &str) -> String {
    let plant = DEMO
        .replace(r#"mode = "auto""#, &format!(r#"mode = "{mode}""#))
        .replace(
            "[nodes.heater]\ntype = \"furnace\"",
            &format!(
                "[nodes.feed_valve]\ntype = \"valve\"\nkv = 150.0\nopening = {opening}\n\n\
                 [nodes.heater]\ntype = \"furnace\""
            ),
        )
        .replace(
            "name = \"feed_line\"\nfrom = \"cool_feed\"\nto = \"heater\"",
            "name = \"feed_line\"\nfrom = \"cool_feed\"\nto = \"feed_valve\"",
        );
    assert!(
        plant.contains("[nodes.feed_valve]") && plant.contains("to = \"feed_valve\""),
        "the fixture's substitutions must all land, or it is testing the demo"
    );
    format!(
        "{plant}\n[[pipes]]\nname = \"valve_line\"\nfrom = \"feed_valve\"\nto = \"heater\"\n\
         length_m = 2.0\ndiameter_m = 0.10\n"
    )
}

fn set_feed_valve(engine: &mut Engine, opening: f64) {
    let node = engine.graph.find_node("feed_valve").expect("a feed valve");
    engine
        .apply(Command::SetValveOpening { node, opening })
        .unwrap_or_else(|e| panic!("no loop owns the feed valve: {e}"));
}

// ------------------------------------------------ gate 1: the blind start

/// **Gate 1. At load and after tick 1 the loop has NO measurement, and says so on
/// the bytes.**
///
/// An `Option` holding a value serializes exactly as the value, so every loop
/// written before M19 publishes the bytes it always did whether or not the
/// absent case is skipped — the corpus cannot see this, and a new plant has no
/// baseline (M10.1's escape). So the absence is asserted HERE, on the serialized
/// faceplate: no `measurement` key, and no `null` either. Both then deserialize
/// back, as `None` — a NaN inside the value would have serialized as `null` and
/// failed to come back.
///
/// Across the blind tick the loop writes nothing: the heater still holds its
/// declared 0.5 MW, and the faceplate tracks it at 0.25.
#[test]
fn the_loop_starts_blind_and_the_snapshot_says_nothing_rather_than_a_number() {
    let mut engine = build(DEMO);

    for label in ["at load", "after tick 1"] {
        if label == "after tick 1" {
            tick(&mut engine, 1);
        }
        let faceplate = control(&engine);
        let json = serde_json::to_string(&faceplate).expect("a faceplate serializes");
        assert!(
            !json.contains("measurement") && !json.contains("null"),
            "{label}: an outlet does not exist yet, so the faceplate must carry no \
             `measurement` key and no null — it published {json}"
        );
        let back: ControlSnapshot = serde_json::from_str(&json).expect("round trips");
        assert_eq!(back.measurement, None, "{label}: absent reads back as None");
        assert_eq!(back, faceplate, "{label}: the faceplate round-trips whole");
        assert_eq!(
            faceplate.output, DECLARED_OUTPUT,
            "{label}: with nothing to act on the faceplate tracks the heater's own \
             position, 0.5 of 2 MW"
        );
        assert_eq!(
            heater_duty_w(&engine),
            DECLARED_OUTPUT * MAX_DUTY_W,
            "{label}: no measurement, no action — the heater keeps its declared duty"
        );
    }

    tick(&mut engine, 2);
    let json = serde_json::to_string(&control(&engine)).expect("serializes");
    assert!(
        json.contains(r#""measurement":{"variable":"temperature","k":"#),
        "after tick 2 the loop has measured tick 1's outlet, and publishes it: {json}"
    );
    let back: ControlSnapshot = serde_json::from_str(&json).expect("round trips");
    assert!(back.measurement.is_some(), "and it reads back as Some");
}

// ------------------------------------------ gate 2: the one-tick identity

/// **Gate 2. The loop acts on the outlet the PREVIOUS tick resolved, bit for bit,
/// on every tick.**
///
/// M17's identity for a holdup was "the snapshot's temperature at `n` is the
/// graph's at `n − 1`". For a zero-volume node there is no graph field: the
/// measurement at the top of tick `n + 1` IS tick `n`'s resolved outlet, the very
/// number tick `n`'s snapshot published. A faceplate showing a fresh re-read, or
/// a measurement taken from anywhere else, breaks the equality on the first tick
/// the outlet moves — and it moves on every tick of the approach.
#[test]
fn the_loop_acts_on_the_outlet_the_previous_tick_resolved() {
    let mut engine = build(DEMO);
    tick(&mut engine, 1);
    let mut previous = outlet_k(&engine);
    assert!(previous.is_finite(), "tick 1 resolves the outlet");
    let mut moved = 0;
    for t in 2..=600 {
        tick(&mut engine, t);
        let acted_on = measured_k(&engine).unwrap_or_else(|| panic!("tick {t}: measured"));
        assert_eq!(
            acted_on.to_bits(),
            previous.to_bits(),
            "tick {t}: the loop must act on tick {}'s published outlet, {previous} K, \
             and it acted on {acted_on} K",
            t - 1
        );
        let now = outlet_k(&engine);
        if now != previous {
            moved += 1;
        }
        previous = now;
    }
    assert!(
        moved > 500,
        "the identity is only a gate if the outlet moves: it moved on {moved} of 599 ticks"
    );
}

// ------------------------------------ gate 3: the seed at the first measurement

/// **Gate 3. The memory waits for the first measurement, is seeded against it
/// with the loop's own sign, and the output then follows `K·e + b` by hand.**
///
/// The first measurement is tick 1's outlet, 48.32 °C — 11.68 K BELOW setpoint,
/// the design input that makes the seed's sign visible. After tick 2 the output
/// must be exactly `initial_output`: a memory seeded against the wrong sign steps
/// it by `2·K·e ≈ 0.35`, one left at zero reads `K·e ≈ 0.175`, and one seeded
/// against a stand-in (the setpoint, `e = 0`) reads `0.25 + K·e`. After tick 3 it
/// must equal the PI law computed from the two published measurements.
#[test]
fn the_memory_is_seeded_at_the_first_measurement_with_the_loops_own_sign() {
    let mut engine = build(DEMO);
    run(&mut engine, 2);
    let m2 = measured_k(&engine).expect("tick 2 measures tick 1's outlet");
    let e2 = (SETPOINT_C + 273.15) - m2;
    assert!(
        (11.0..12.5).contains(&e2),
        "the fixture's design input: the first measurement sits ~11.7 K below \
         setpoint, and it sits {e2} K below"
    );
    assert_eq!(
        output(&engine),
        DECLARED_OUTPUT,
        "the first output after the seed is the declared `initial_output`, exactly"
    );

    tick(&mut engine, 3);
    let m3 = measured_k(&engine).expect("measured");
    let e3 = (SETPOINT_C + 273.15) - m3;
    // The memory the seed left, plus the one explicit-Euler step tick 2 took
    // (its output was interior, so it accumulated).
    let b = (DECLARED_OUTPUT - GAIN_PER_K * e2) + GAIN_PER_K / INTEGRAL_TIME_S * e2 * DT_S;
    let expected = GAIN_PER_K * e3 + b;
    assert!(
        (output(&engine) - expected).abs() < 1.0e-12,
        "after tick 3 the output is K·(sp − m₃) + b = {expected}, by hand from the \
         published measurements, and the loop produced {}",
        output(&engine)
    );
}

// ------------------------------------------ gate 4: holds; the twin does not

/// **Gate 4. The loop holds the outlet on 60 °C, off both clamps; the MANUAL twin
/// parks at its own duty's 48.32 °C.**
///
/// The tolerance is DERIVED, and the first draft of it was wrong. The loop's slow
/// pole is 0.966 per tick, so the 11.68 K it starts from has decayed to nothing
/// long before tick 1 000 — and the draft asserted 1e-6 K on that ground and read
/// 1.65e-5. What remains is not the transient: the flow drifts as the tank level
/// moves, so the outlet at a FIXED duty drifts too, and a PI loop follows a ramp
/// disturbance with a lag of `1/(K·G·dt/T_i)` ticks. The drift is measured on the
/// twin (same hydraulics, so the same flow) and scaled by duty — the outlet's rise
/// above the feed is `Q/(ṁ·c̄p)`, so its drift under a moving `ṁ` is proportional
/// to `Q` — and the bound is twice the lag error that predicts.
#[test]
fn the_loop_holds_its_outlet_and_the_parked_twin_does_not() {
    let mut engine = build(DEMO);
    let mut clamped = 0;
    for t in 1..=1000 {
        tick(&mut engine, t);
        let u = output(&engine);
        if u <= 0.0 || u >= 1.0 {
            clamped += 1;
        }
    }
    assert_eq!(clamped, 0, "the shipped tuning never reaches either clamp");
    let u = output(&engine);
    assert!(
        (0.55..0.65).contains(&u),
        "holding 60 °C from a 40 °C feed takes about 0.60 of 2 MW, and it took {u}"
    );

    let mut twin = build(&DEMO.replace(r#"mode = "auto""#, r#"mode = "manual""#));
    run(&mut twin, 999);
    let before = outlet_k(&twin);
    tick(&mut twin, 1000);
    let parked_c = outlet_k(&twin) - 273.15;
    assert!(
        (parked_c - 48.32).abs() < 0.01,
        "parked at 0.5 MW the outlet sits at 48.32 °C, and it sits at {parked_c}"
    );
    assert_eq!(
        output(&twin),
        DECLARED_OUTPUT,
        "and the twin's faceplate tracks 0.25"
    );

    // The ramp the loop is following, and the lag error it implies.
    let twin_drift_per_tick = outlet_k(&twin) - before;
    let g = (outlet_k(&engine) - outlet_k(&twin)) / (u - DECLARED_OUTPUT);
    let lag_ticks = INTEGRAL_TIME_S / (GAIN_PER_K * g * DT_S);
    let predicted = (twin_drift_per_tick * u / DECLARED_OUTPUT).abs() * lag_ticks;
    let error = (outlet_k(&engine) - 273.15 - SETPOINT_C).abs();
    assert!(
        twin_drift_per_tick != 0.0 && predicted < 1.0e-3,
        "the plant must be drifting slowly for this bound to mean anything: the twin \
         moved {twin_drift_per_tick} K in one tick"
    );
    assert!(
        error <= 2.0 * predicted,
        "by tick 1 000 the outlet is on 60 °C to within the lag a PI loop owes a slow \
         ramp: off by {error} K, against 2 × {predicted} K ({lag_ticks} ticks of a \
         {twin_drift_per_tick} K/tick drift at the loop's duty)"
    );
}

// ---------------------------------------- gate 5: the stability bound, both sides

/// **Gate 5. The loop gain `K·G` is computed from PUBLISHED numbers and sits at
/// half its bound; above the bound the loop rings between both clamps, below it
/// the loop settles.**
///
/// An outlet has no thermal mass, so the loop's only dynamics is the one tick it
/// waits to measure: poles at `1` and `−K·G`, stable only for `K·G < 1`
/// (docs/DESIGN.md §23 fork 6). `G` is taken from two operating points the demo
/// publishes — the settled AUTO pair and the MANUAL twin's — so the gate does not
/// trust the note's 33.28 K. Then the bound is exercised on both sides of itself,
/// because a bound only asserted in prose is a comment.
#[test]
fn the_gain_bound_is_a_stability_bound_and_both_sides_of_it_behave_accordingly() {
    let mut auto = build(DEMO);
    run(&mut auto, 1000);
    let (u_a, t_a) = (output(&auto), outlet_k(&auto));
    let mut twin = build(&DEMO.replace(r#"mode = "auto""#, r#"mode = "manual""#));
    run(&mut twin, 1000);
    let (u_m, t_m) = (output(&twin), outlet_k(&twin));
    let g = (t_a - t_m) / (u_a - u_m);
    // `K` is the FILE's, not only this test's constant: without this line the
    // product below is a number the gate declared itself, and it stays green
    // whatever tuning the demo ships.
    assert!(
        DEMO.contains(&format!("gain_per_k = {GAIN_PER_K}\n")),
        "the demo must declare the gain this gate multiplies by"
    );
    let loop_gain = GAIN_PER_K * g;
    assert!(
        (0.4..=0.6).contains(&loop_gain),
        "K·G from the published operating points is {loop_gain} (G = {g} K per unit \
         output); the shipped tuning sits at half the bound"
    );

    // Above the bound: the ring grows until both clamps catch it.
    let mut ringing = build(&tuned(1.2 / g, INTEGRAL_TIME_S));
    run(&mut ringing, 200);
    let mut outlets = Vec::new();
    let (mut hit_low, mut hit_high) = (false, false);
    for t in 201..=240 {
        tick(&mut ringing, t);
        outlets.push(outlet_k(&ringing));
        let u = output(&ringing);
        hit_low |= u == 0.0;
        hit_high |= u == 1.0;
    }
    let steps: Vec<f64> = outlets.windows(2).map(|w| w[1] - w[0]).collect();
    assert!(
        steps.windows(2).all(|w| w[0] * w[1] < 0.0),
        "at K·G = 1.2 the outlet must alternate direction on every tick (the −K·G \
         pole): {outlets:?}"
    );
    assert!(
        hit_low && hit_high,
        "and the ring must be caught by BOTH clamps, bang-bang between the feed and \
         full firing"
    );

    // Below the bound: the same plant settles.
    let mut settling = build(&tuned(0.8 / g, INTEGRAL_TIME_S));
    let mut clamped = 0;
    for t in 1..=2000 {
        tick(&mut settling, t);
        let u = output(&settling);
        if u <= 0.0 || u >= 1.0 {
            clamped += 1;
        }
    }
    let settled_c = outlet_k(&settling) - 273.15;
    assert!(
        // The ramp lag of gate 4 applies here too; 1e-3 K is what separates
        // "settled" from a ring between 40 and 73 °C, by four orders.
        (settled_c - SETPOINT_C).abs() < 1.0e-3,
        "at K·G = 0.8 the loop still settles on 60 °C, and it reads {settled_c}"
    );
    assert_eq!(clamped, 0, "and never touches a clamp");
}

// ------------------------------------------------ gate 6: the stagnant outlet

/// **Gate 6. A furnace with no flow through it has no measurement, and the loop
/// holds — including its PENDING memory — until flow comes.**
///
/// A zero-volume node with no inflow reports a held placeholder, not a computed
/// temperature (docs/DESIGN.md §23 fork 4). The case that makes the placeholder
/// dangerous is the one this gate builds: the feed valve shut from LOAD, so the
/// heater has never resolved anything and its placeholder is `T_AMBIENT`, 20 °C —
/// 40 K below setpoint.
///
/// **The counterfactual is asserted first, on the engine's own published
/// number**: the heater's `temperature_k` really does read 293.15 K throughout,
/// and a PI loop that took it as a measurement would seed against a 40 K error
/// and integrate `K/T_i · 40 K = 0.06` of output per tick — into the upper clamp
/// in about thirteen ticks, firing full duty into the stream the moment the valve
/// opens. What the loop does instead: no measurement on any stagnant tick, the
/// output and the heater's duty exactly as declared, and — because its memory was
/// never seeded — a first measured output after the valve opens of exactly
/// `initial_output`, the same seed a loop gets whose outlet existed from tick 1.
///
/// **§23 fork 4 said the MID-RUN stall winds up too, and it does not**, which is
/// the other half of this gate. A furnace that has been flowing holds its LAST
/// resolved outlet, and a loop at steady state last saw its own setpoint — so a
/// loop reading that placeholder would sit still, not wind. The rule still
/// removes the measurement there (asserted below), but the hazard it closes is
/// the startup placeholder, not the stall.
#[test]
fn a_stagnant_outlet_has_no_measurement_and_the_loop_holds_until_flow_returns() {
    let mut engine = build(&with_feed_valve("0.0", "auto"));
    let outage: u64 = 40;
    for t in 1..=outage {
        tick(&mut engine, t);
        let placeholder_c = outlet_k(&engine) - 273.15;
        assert_eq!(
            placeholder_c, 20.0,
            "tick {t}: a heater that has never had flow publishes the ambient \
             placeholder — the number a stand-in would have seeded against"
        );
        assert_eq!(
            measured_k(&engine),
            None,
            "tick {t}: a stagnant outlet is not a measurement"
        );
        assert_eq!(
            output(&engine),
            DECLARED_OUTPUT,
            "tick {t}: the output holds"
        );
        assert_eq!(
            heater_duty_w(&engine),
            DECLARED_OUTPUT * MAX_DUTY_W,
            "tick {t}: the heater holds its declared duty"
        );
    }

    // The counterfactual, from the loop's own law against the placeholder: seeded
    // at 0.25 on the first stagnant tick it read, and integrating from there.
    let placeholder_error = (SETPOINT_C + 273.15) - (20.0 + 273.15);
    let mut b = DECLARED_OUTPUT - GAIN_PER_K * placeholder_error;
    let mut wound_at = None;
    for n in 1..outage {
        let u = GAIN_PER_K * placeholder_error + b;
        if u >= 1.0 {
            wound_at = Some(n);
            break;
        }
        b += GAIN_PER_K / INTEGRAL_TIME_S * placeholder_error * DT_S;
    }
    let wound_at = wound_at.unwrap_or_else(|| {
        panic!("the counterfactual must reach the clamp inside the {outage}-tick outage")
    });
    assert!(
        wound_at < outage / 2,
        "a loop reading the placeholder would have been pinned at full firing from \
         tick {wound_at} of the outage"
    );

    // Flow returns. The tick that opens the valve is solved with it open, so the
    // loop — which ran at the top of that tick — is still blind; the next tick
    // measures, and seeds the memory that has been pending since load.
    set_feed_valve(&mut engine, 1.0);
    tick(&mut engine, outage + 1);
    assert_eq!(measured_k(&engine), None, "still blind on the opening tick");
    tick(&mut engine, outage + 2);
    let first = measured_k(&engine).expect("flow is back, so is the measurement");
    assert!(
        (first - (SETPOINT_C + 273.15)).abs() > 1.0,
        "the seed's first error must be visible for this to mean anything: {first} K"
    );
    assert_eq!(
        output(&engine),
        DECLARED_OUTPUT,
        "the pending memory is seeded now, from the declared initial_output — exactly"
    );

    // Mid-run: settle, starve, and the measurement goes away again.
    for t in outage + 3..=outage + 400 {
        tick(&mut engine, t);
    }
    set_feed_valve(&mut engine, 0.0);
    // The shutting tick's control pass still measured the last FLOWING outlet and
    // acted on it; that is the position the loop then holds.
    tick(&mut engine, outage + 401);
    assert!(
        measured_k(&engine).is_some(),
        "the shutting tick measured flow"
    );
    let settled_output = output(&engine);
    for t in outage + 402..=outage + 420 {
        tick(&mut engine, t);
        assert_eq!(measured_k(&engine), None, "tick {t}: stagnant mid-run");
        assert_eq!(
            output(&engine),
            settled_output,
            "tick {t}: holds where it stood"
        );
    }
}

// ------------------------------------ gate 7: MANUAL→AUTO needs a measurement

/// **Gate 7. A MANUAL→AUTO transfer is refused while there is nothing to
/// measure, and is bumpless once there is.**
///
/// The transfer back-calculates the loop's memory from the error standing NOW
/// (§10 fork 4). Before the first tick, and while the outlet is stagnant, there is
/// no error, and seeding against a stand-in is the fabricated number §23 refuses.
#[test]
fn manual_to_auto_is_refused_without_a_measurement_and_bumpless_with_one() {
    let to_auto = Command::SetControllerMode {
        loop_id: LoopId(0),
        mode: ControlMode::Auto,
    };

    let mut engine = build(&DEMO.replace(r#"mode = "auto""#, r#"mode = "manual""#));
    let before_first_tick = engine
        .apply(to_auto.clone())
        .err()
        .map(|e| e.to_string())
        .unwrap_or_else(|| panic!("a transfer before the first tick must be refused"));
    assert!(
        before_first_tick.contains("has no measurement to transfer against"),
        "{before_first_tick}"
    );
    assert_eq!(
        control(&engine).mode,
        ControlMode::Manual,
        "and the loop stays MANUAL"
    );

    // One tick later the outlet exists, and the transfer is bumpless: the next
    // output is the position the heater already holds.
    tick(&mut engine, 1);
    engine
        .apply(to_auto.clone())
        .unwrap_or_else(|e| panic!("with an outlet measured, the transfer is legal: {e}"));
    tick(&mut engine, 2);
    assert!(
        (output(&engine) - DECLARED_OUTPUT).abs() < 1.0e-12,
        "a bumpless transfer's first output is the position it took over from, and \
         it was {}",
        output(&engine)
    );

    // Stagnant: refused again.
    let mut starved = build(&with_feed_valve("0.0", "manual"));
    run(&mut starved, 3);
    let stagnant = starved
        .apply(to_auto)
        .err()
        .map(|e| e.to_string())
        .unwrap_or_else(|| panic!("a transfer against a stagnant outlet must be refused"));
    assert!(
        stagnant.contains("has no measurement to transfer against"),
        "{stagnant}"
    );
}

// ---------------------------------- the anti-windup arm, reached by a command

/// **A 75 °C setpoint is above what full firing reaches, so the loop pins at
/// `u = 1`; stepping back just below where the outlet sits releases it where the
/// SIGNED memory says.**
///
/// Full duty puts the outlet at 73.29 °C, so 75 °C is unreachable and the loop
/// must sit on its clamp, back-calculating its memory every tick rather than
/// integrating (§23 fork 7, M17's method). The release target is 72 °C — just
/// below the ceiling, because M18 found that stepping all the way back cannot see
/// a sign: from the clamp, a right and a wrong memory both fall to zero. At 72 °C
/// the signed memory `b ≈ 1 − K·(75 − 73.29)` releases to about 0.955; a memory
/// back-calculated with the UNSIGNED error, `1 + K·1.71`, stays pinned at 1.
///
/// On an algebraic plant the pinned outlet is constant, so "pinned" is steadier
/// here than on a tank — but M18's finding (iv) still applies at the last bit:
/// `K·e + (1 − K·e)` can land a hair under 1, so the window is bounded rather
/// than asserted equal.
#[test]
fn an_unreachable_setpoint_pins_the_furnace_and_the_release_is_signed() {
    let mut engine = build(DEMO);
    run(&mut engine, 300);
    set_setpoint_c(&mut engine, 75.0);
    for t in 301..=400 {
        tick(&mut engine, t);
    }
    for t in 401..=450 {
        tick(&mut engine, t);
        let u = output(&engine);
        assert!(
            u >= 1.0 - 1.0e-9,
            "tick {t}: 75 °C is above full firing's 73.29 °C, so the loop sits on its \
             clamp — it read {u}"
        );
    }
    let ceiling_c = outlet_k(&engine) - 273.15;
    assert!(
        (ceiling_c - 73.29).abs() < 0.01,
        "pinned at 2 MW the outlet sits at 73.29 °C, and reads {ceiling_c}"
    );

    set_setpoint_c(&mut engine, 72.0);
    tick(&mut engine, 451);
    let released = output(&engine);
    assert!(
        (0.9..0.99).contains(&released),
        "the signed memory releases to about 0.955 on the first tick below the \
         ceiling; an unsigned back-calculation would have stayed at 1, and it read \
         {released}"
    );
}

// ------------------------------------------------------ gate 8: the refusals

/// **Gate 8. Every zero-volume kind that is NOT a furnace or cooler outlet is
/// refused, each asserting a substring distinctive to its own message.**
///
/// The accepted plant is built first — the demo, measuring the heater — so no
/// case can be passing because the fixture was broken some other way. Each case
/// then points the SAME loop at a different node.
#[test]
fn every_other_zero_volume_measurement_is_refused_for_its_own_reason() {
    build(DEMO);
    // A cooler outlet is admitted too, on the cooler demo's plant.
    let cooler = include_str!("../../../scenarios/tank_temperature_control.toml").replace(
        r#"measurement = { node = "hold_tank", variable = "temperature" }"#,
        r#"measurement = { node = "chiller", variable = "temperature" }"#,
    );
    assert!(
        cooler.contains(r#"node = "chiller""#),
        "the substitution landed"
    );
    build(&cooler);

    let pointed_at = |node: &str, extra: &str| {
        let plant = DEMO.replace(
            r#"measurement = { node = "heater", variable = "temperature" }"#,
            &format!(r#"measurement = {{ node = "{node}", variable = "temperature" }}"#),
        );
        format!("{plant}{extra}")
    };
    let scope = "Other zero-volume nodes are not admitted, as a scope decision";
    let cases: Vec<(&str, String, &str)> = vec![
        ("a valve", pointed_at("drain_valve", ""), scope),
        (
            "a junction",
            pointed_at(
                "tee",
                "\n[nodes.tee]\ntype = \"junction\"\n\n[[pipes]]\nname = \"tee_in\"\n\
                 from = \"hold_tank\"\nto = \"tee\"\nlength_m = 1.0\ndiameter_m = 0.05\n\n\
                 [[pipes]]\nname = \"tee_out\"\nfrom = \"tee\"\nto = \"rundown\"\n\
                 length_m = 1.0\ndiameter_m = 0.05\n",
            ),
            scope,
        ),
        (
            "a pump",
            pointed_at(
                "booster",
                "\n[nodes.booster]\ntype = \"pump\"\nh0_m = 10.0\na = 800.0\non = true\n\n\
                 [[pipes]]\nname = \"pump_in\"\nfrom = \"hold_tank\"\nto = \"booster\"\n\
                 length_m = 1.0\ndiameter_m = 0.05\n\n[[pipes]]\nname = \"pump_out\"\n\
                 from = \"booster\"\nto = \"rundown\"\nlength_m = 1.0\ndiameter_m = 0.05\n",
            ),
            scope,
        ),
        (
            "a relief valve",
            pointed_at(
                "psv",
                "\n[nodes.psv]\ntype = \"relief_valve\"\nkv = 12.0\nset_pressure_bar = 5.0\n\
                 accumulation_bar = 1.0\n\n[[pipes]]\nname = \"psv_in\"\n\
                 from = \"hold_tank\"\nto = \"psv\"\nlength_m = 1.0\ndiameter_m = 0.05\n\n\
                 [[pipes]]\nname = \"psv_out\"\nfrom = \"psv\"\nto = \"rundown\"\n\
                 length_m = 1.0\ndiameter_m = 0.05\n",
            ),
            "is a relief valve, which is shut in normal operation",
        ),
        (
            "an exchanger side",
            format!(
                "{}\n[[controls]]\nname = \"side_temperature\"\n\
                 measurement = {{ node = \"hx_cold\", variable = \"temperature\" }}\n\
                 actuator = \"hx_hot\"\nalgorithm = \"p\"\nmode = \"auto\"\n\
                 setpoint_c = 60.0\ngain_per_k = 0.01\n",
                include_str!("../../../scenarios/heat_recovery.toml")
            ),
            scope,
        ),
    ];
    for (what, plant, expected) in cases {
        let message = refusal(&plant);
        assert!(
            message.contains(expected),
            "{what}: expected the refusal to say `{expected}`, and it said: {message}"
        );
        assert!(
            !message.contains("stated rule for what a loop measures at tick 0"),
            "{what}: the old reason — no tick-0 rule — is false since §23: {message}"
        );
    }
}
