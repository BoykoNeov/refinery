//! M19.1: a furnace holding its own OUTLET — the zero-volume measurement
//! (docs/DESIGN.md §23).
//!
//! **The first loop in this engine whose measurement does not exist at load.** A
//! tank's or vessel's temperature is a field on the graph, exact from load; a
//! furnace or cooler outlet is resolved by each tick's sweep into `NodeStates` and
//! is absent before the first one. The rule §23 states is **no measurement, no
//! action**: the loop writes nothing, its faceplate tracks the actuator, and a PI
//! loop's memory waits — PENDING on its declared `initial_output` — until the
//! first measurement seeds it. A stagnant COOLER outlet, whose entry is a held
//! placeholder, is the same state.
//!
//! The demo is `scenarios/furnace_outlet_control.toml`, the M18 heating file with
//! the loop moved from the tank to the heater. Its gates are here beside the
//! fixtures because every one of them is about the same new thing — an absent
//! measurement — rather than about the demo's numbers.
//!
//! **Since M34 the furnace has a coil** (docs/DESIGN.md §37): its duty heats tube
//! metal with a temperature of its own, so the outlet lags the duty by the coil's
//! `C/G` (38.5 s here) instead of answering on the same tick, and a furnace with
//! no flow is measured at its coil rather than held. Gate 5 and gate 6 were
//! rewritten for that; the stagnant-outlet rule they used to gate on the furnace
//! is gated on a COOLER in gate 6b, the unit it still applies to.
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
/// The coil's `coil_heat_capacity_mj_per_k = 2`, in SI (M34).
const COIL_C_J_PER_K: f64 = 2.0e6;
/// The demo's coil conductance, `coil_ua_kw_per_k = 120.2`, by hand.
const COIL_UA_W_PER_K: f64 = 120.2e3;
/// The demo's flame, `flame_temperature_c = 1951.1`, and the combustion air's
/// 20 °C, by hand (M36, docs/DESIGN.md §40).
const FLAME_K: f64 = 2224.25;
const AIR_K: f64 = 293.15;

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
        NodeKind::Furnace { duty, .. } => duty.value(),
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
/// parks at its own duty's 48.19 °C** (48.32 °C before M36 sent part of the duty
/// up the stack, docs/DESIGN.md §40).
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
        (parked_c - 48.19).abs() < 0.01,
        "parked at 0.5 MW the outlet sits at 48.19 °C, and it sits at {parked_c}"
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

// ------------------------------- gate 5: the coil removed the lag-free bound

/// **Gate 5. The loop gain `K·G` from PUBLISHED numbers still sits near 0.5, and
/// the gain that rang between both clamps before M34 now settles.**
///
/// Before M34 the outlet had no thermal mass, so the loop's only dynamics was the
/// one tick it waits to measure: poles at `1` and `−K·G`, stable only for
/// `K·G < 2/(2 − dt/T_i)`, 1.053 here (docs/DESIGN.md §23 fork 6, §24 fork 6),
/// and this gate drove `K·G = 1.2` into a period-two ring caught by both clamps
/// (ledger row E10). **The coil is the lag that row said a real heater has**
/// (docs/DESIGN.md §37): `C/G` is 38.5 s on this plant, so the outlet answers a
/// duty step over tens of ticks, and the same 1.2 now settles on 60 °C without
/// touching either clamp. The bound is not relocated here: a proportional loop on
/// a first-order lag behind one tick of delay rings at `K·G ≈ 1/(1 − e^(−G·dt/C))`,
/// about 39, and §37's sweep found the clamps catching every gain up to 60 before
/// any ring could grow.
///
/// `G` is still the STATIC gain, read off two published operating points (the
/// settled AUTO pair and the MANUAL twin's), and it is unchanged by the coil:
/// settled, the coil stores nothing and the outlet is `T_in + Q/(ṁ·cp)` again.
#[test]
fn the_coil_lets_a_gain_that_rang_without_it_settle() {
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
         output); the coil leaves the static gain where it was"
    );

    for target in [0.8, 1.2] {
        let mut engine = build(&tuned(target / g, INTEGRAL_TIME_S));
        let mut outlets = Vec::new();
        let mut clamped = 0;
        for t in 1..=2000 {
            tick(&mut engine, t);
            outlets.push(outlet_k(&engine));
            let u = output(&engine);
            if u <= 0.0 || u >= 1.0 {
                clamped += 1;
            }
        }
        assert_eq!(
            clamped, 0,
            "at K·G = {target} the loop never touches a clamp"
        );
        // The pre-M34 signature, gone: a ring alternates direction on EVERY tick.
        let late = &outlets[200..240];
        let steps: Vec<f64> = late.windows(2).map(|w| w[1] - w[0]).collect();
        assert!(
            !steps.windows(2).all(|w| w[0] * w[1] < 0.0),
            "at K·G = {target} the outlet must not alternate tick by tick: {late:?}"
        );
        let settled_c = outlet_k(&engine) - 273.15;
        assert!(
            (settled_c - SETPOINT_C).abs() < 1.0e-3,
            "at K·G = {target} the loop settles on 60 °C, and reads {settled_c}"
        );
    }
}

// ---------------------------------- gate 6: a furnace with no flow is measured

/// **Gate 6. A furnace with no flow through it IS measured: the fluid standing in
/// its tubes sits at its coil's temperature, and the loop acts on it.**
///
/// Before M34 a stagnant furnace held a placeholder (`T_AMBIENT` if it had never
/// flowed), which the loop was refused as a measurement, and the duty was dropped
/// (docs/DESIGN.md §23 fork 4, ledger row B39). Now the duty goes into the coil
/// (§37), which has a temperature whether or not anything flows, so there is
/// nothing to hold: from tick 2 the loop measures the coil, and the coil keeps
/// exactly what its flame lets it absorb under whatever duty the loop writes —
/// `T_f − (T_f − T_c)·exp(−Q·dt/(C·(T_f − T_a)))` over a tick, the dry coil's
/// exact step (M36, docs/DESIGN.md §40; `Q·dt/C` before, when the coil kept the
/// whole duty). The placeholder
/// rule survives for a COOLER, which has no coil (gate 6b).
///
/// What a loop does with that reading is what a real outlet controller on a
/// stagnant coil does: below setpoint it fires harder, and the coil heats until
/// the loop backs off. A low-flow trip is the protection against that, not the
/// loop (`flow_trip_reference.rs`).
#[test]
fn a_furnace_with_no_flow_is_measured_at_its_coil() {
    let mut engine = build(&with_feed_valve("0.0", "auto"));
    let heater = engine.graph.find_node("heater").expect("a heater");
    let coil_k = |engine: &Engine| match &engine.graph.node(heater).kind {
        NodeKind::Furnace { coil, .. } => coil.temperature.value(),
        other => panic!("heater is a furnace, not {other:?}"),
    };
    let mut previous_coil = coil_k(&engine);
    let mut previous_outlet = None;
    for t in 1..=40 {
        tick(&mut engine, t);
        assert!(
            !engine.node_states().held.contains(&heater),
            "tick {t}: a furnace is never held"
        );
        let coil = coil_k(&engine);
        assert_eq!(
            outlet_k(&engine),
            coil,
            "tick {t}: the fluid standing in the tubes is at the coil's temperature"
        );
        // Everything the flame let the metal absorb under the duty written at the
        // top of this tick stayed in it: the dry coil's exact step.
        let tau = COIL_C_J_PER_K * (FLAME_K - AIR_K) / heater_duty_w(&engine);
        let expected_rise = (FLAME_K - previous_coil) * -(-DT_S / tau).exp_m1();
        assert!(
            (coil - previous_coil - expected_rise).abs() < 1.0e-9,
            "tick {t}: the coil rises by exactly (T_f − T_c)·(1 − e^(−dt/τ)) = \
             {expected_rise} K, and rose {}",
            coil - previous_coil
        );
        // Blind on tick 1 only, as every outlet loop is; then it measures the
        // outlet the previous tick resolved — the coil.
        assert_eq!(
            measured_k(&engine),
            previous_outlet,
            "tick {t}: the loop acts on the outlet the previous tick resolved"
        );
        previous_coil = coil;
        previous_outlet = Some(outlet_k(&engine));
    }
    assert!(
        output(&engine) > DECLARED_OUTPUT,
        "a stagnant coil below setpoint draws more firing, and the loop wrote {}",
        output(&engine)
    );
}

// --------------------------------- gate 6b: a stagnant COOLER is still held

/// The demo with its furnace swapped for a COOLER holding a 30 °C outlet on the
/// 40 °C feed — direct action — so the stagnant-outlet rule, which a cooler still
/// needs (it has no coil), stays gated.
fn as_cooler(src: &str) -> String {
    let coil_lines: String = src
        .lines()
        .filter(|line| !line.starts_with("coil_") && !line.starts_with("# Tube coil"))
        .filter(|line| !line.starts_with("# the load flow's") && !line.starts_with("# at load,"))
        .map(|line| format!("{line}\n"))
        .collect();
    let plant = coil_lines
        .replace(
            "[nodes.heater]\ntype = \"furnace\"",
            "[nodes.heater]\ntype = \"cooler\"",
        )
        .replace(r#"action = "reverse""#, r#"action = "direct""#)
        .replace("setpoint_c = 60.0", "setpoint_c = 30.0");
    assert!(
        plant.contains("type = \"cooler\"")
            && plant.contains(r#"action = "direct""#)
            && plant.contains("setpoint_c = 30.0")
            && !plant.contains("coil_"),
        "the cooler fixture's substitutions must all land"
    );
    plant
}

fn cooler_duty_w(engine: &Engine) -> f64 {
    let id = engine.graph.find_node("heater").expect("the unit");
    match engine.graph.node(id).kind {
        NodeKind::Cooler { duty } => duty.value(),
        ref other => panic!("the fixture's unit is a cooler, not {other:?}"),
    }
}

/// **Gate 6b. A COOLER with no flow through it has no measurement, and the loop
/// holds — including its PENDING memory — until flow comes.**
///
/// The pre-M34 gate 6, moved to the unit it still applies to. A cooler has no
/// coil, so a zero-volume cooler with no inflow reports a held placeholder, not a
/// computed temperature (docs/DESIGN.md §23 fork 4). The dangerous case is the
/// one built here: the feed shut from LOAD, so the cooler has never resolved
/// anything and its placeholder is `T_AMBIENT`, 20 °C — 10 K below setpoint.
///
/// **The counterfactual is asserted first, on the engine's own published
/// number**: a direct-acting loop that took that placeholder as a measurement
/// would seed against a −10 K error and integrate its cooling down to zero inside
/// the outage, so the plant would get no cooling the moment flow returned. What
/// the loop does instead: no measurement on any stagnant tick, the output and the
/// duty exactly as declared, and a first measured output after the valve opens of
/// exactly `initial_output`.
#[test]
fn a_stagnant_cooler_outlet_has_no_measurement_and_the_loop_holds_until_flow_returns() {
    let setpoint_c = 30.0;
    let mut engine = build(&as_cooler(&with_feed_valve("0.0", "auto")));
    let outage: u64 = 40;
    for t in 1..=outage {
        tick(&mut engine, t);
        let placeholder_c = outlet_k(&engine) - 273.15;
        assert_eq!(
            placeholder_c, 20.0,
            "tick {t}: a cooler that has never had flow publishes the ambient \
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
            cooler_duty_w(&engine),
            DECLARED_OUTPUT * MAX_DUTY_W,
            "tick {t}: the cooler holds its declared duty"
        );
    }

    // The counterfactual, from the loop's own direct-acting law against the
    // placeholder: seeded at 0.25 on the first stagnant tick, integrating down.
    let placeholder_error = (20.0 + 273.15) - (setpoint_c + 273.15);
    let mut b = DECLARED_OUTPUT - GAIN_PER_K * placeholder_error;
    let mut wound_at = None;
    for n in 1..outage {
        let u = GAIN_PER_K * placeholder_error + b;
        if u <= 0.0 {
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
        "a loop reading the placeholder would have cut all cooling from tick \
         {wound_at} of the outage"
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
        (first - (setpoint_c + 273.15)).abs() > 1.0,
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
/// (§10 fork 4). Before the first tick there is no error, and seeding against a
/// stand-in is the fabricated number §23 refuses. A stagnant COOLER is the same
/// state; a stagnant FURNACE is not, since M34 — its coil is a measurement — and
/// the transfer onto it is admitted and bumpless.
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

    // A stagnant furnace: admitted, and bumpless, against its coil.
    let mut dry = build(&with_feed_valve("0.0", "manual"));
    run(&mut dry, 3);
    dry.apply(to_auto.clone())
        .unwrap_or_else(|e| panic!("a stagnant furnace is measured at its coil: {e}"));
    tick(&mut dry, 4);
    assert!(
        (output(&dry) - DECLARED_OUTPUT).abs() < 1.0e-12,
        "the transfer onto a stagnant furnace is bumpless too, and output {}",
        output(&dry)
    );

    // A stagnant cooler: refused, as every stagnant outlet was before M34.
    let mut starved = build(&as_cooler(&with_feed_valve("0.0", "manual")));
    run(&mut starved, 3);
    let stagnant = starved
        .apply(to_auto)
        .err()
        .map(|e| e.to_string())
        .unwrap_or_else(|| panic!("a transfer against a stagnant cooler must be refused"));
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
/// Full duty puts the outlet at 72.29 °C (73.29 before M36's stack loss,
/// docs/DESIGN.md §40), so 75 °C is unreachable and the loop
/// must sit on its clamp, back-calculating its memory every tick rather than
/// integrating (§23 fork 7, M17's method). The release target is 72 °C — just
/// below the ceiling, because M18 found that stepping all the way back cannot see
/// a sign: from the clamp, a right and a wrong memory both fall to zero. At 72 °C
/// the signed memory `b ≈ 1 − K·(75 − 72.29)` releases to about 0.955; a memory
/// back-calculated with the UNSIGNED error, `1 + K·2.71`, stays pinned at 1.
///
/// Since M34 the coil puts 38.5 s of lag between the duty and the outlet
/// (docs/DESIGN.md §37), so the loop takes longer to reach its clamp and the
/// outlet longer to settle under it: the step is held for 400 ticks, ten time
/// constants, before the pinned window is read. **And "pinned" is no longer
/// exact.** M18's finding (iv) applies at more than the last bit on a lagging
/// plant: the back-calculated memory is one tick behind an outlet still creeping
/// up toward its ceiling, so the output lands under 1 by `K` times that creep —
/// 1.3e-7 at tick 702, where the lag-free plant held it within 1e-9. The window
/// is bounded at 1e-6, which still separates "on the clamp" from the 0.045 the
/// release below moves by.
#[test]
fn an_unreachable_setpoint_pins_the_furnace_and_the_release_is_signed() {
    let mut engine = build(DEMO);
    run(&mut engine, 300);
    set_setpoint_c(&mut engine, 75.0);
    for t in 301..=700 {
        tick(&mut engine, t);
    }
    for t in 701..=750 {
        tick(&mut engine, t);
        let u = output(&engine);
        assert!(
            u >= 1.0 - 1.0e-6,
            "tick {t}: 75 °C is above full firing's 72.29 °C, so the loop sits on its \
             clamp — it read {u}"
        );
    }
    // The ceiling from this tick's own published feed: the outlet a settled coil
    // gives at full firing, `T_in + G·(T_c − T_in)/(ṁ·cp)` with `T_c` the root of
    // `Q_max·(T_f − T_c)/(T_f − T_a) = G·(T_c − T_in)` (M36, §40). It was
    // `T_in + Q_max/(ṁ·cp)`, 73.29 °C at tick 450, before M36; it is 72.29 °C now,
    // and drifts by hundredths because the tank's level moves the flow (gate 4).
    let feed = engine
        .snapshot()
        .edges
        .into_iter()
        .find(|e| e.name == "feed_line")
        .expect("the demo's feed line");
    let inlet_k = feed.stream.temperature.value();
    let capacity_rate =
        feed.stream.mass_flow.value() * feed.stream.composition.mixture_cp(&engine.slate).value();
    let conductance = capacity_rate * -(-COIL_UA_W_PER_K / capacity_rate).exp_m1();
    let span = FLAME_K - AIR_K;
    let coil_k =
        (MAX_DUTY_W * FLAME_K / span + conductance * inlet_k) / (MAX_DUTY_W / span + conductance);
    let ceiling_k = inlet_k + conductance * (coil_k - inlet_k) / capacity_rate;
    let pinned_c = outlet_k(&engine) - 273.15;
    assert!(
        (pinned_c - (ceiling_k - 273.15)).abs() < 1.0e-3 && (pinned_c - 72.29).abs() < 0.05,
        "pinned at 2 MW the outlet sits at its full-firing ceiling, {} °C, and reads          {pinned_c}",
        ceiling_k - 273.15
    );

    set_setpoint_c(&mut engine, 72.0);
    tick(&mut engine, 751);
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
