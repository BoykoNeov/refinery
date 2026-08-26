//! Gates for the control-loop seam (M8.2, DESIGN §10).
//!
//! **The gate written first is the counterfactual, and it is the control rather
//! than the test.** Fork 6 opens by refusing the obvious assertion: "the level sat
//! at the setpoint" is not a gate, because a tank draining through a *fixed* valve
//! self-regulates — `bottom_pressure` rises with level, so outflow rises with
//! level, and the thing finds an equilibrium with no controller anywhere in sight.
//! A single steady-state assertion therefore passes on the plant with the loop
//! removed, which is `a-control-can-be-implied-by-its-assertion` one milestone
//! later.
//!
//! So the plant below is **built** to fail that way if the loop is parked, and the
//! numbers in this file were measured on it before any assertion was written:
//!
//! | run (8 000 ticks, dt = 0.5 s) | level at the end |
//! |---|---|
//! | AUTO, setpoint 4.0 m | **4.7386 m**, valve 0.3693, settled |
//! | AUTO, setpoint stepped to 3.0 m | **3.8337 m**, settled |
//! | MANUAL, valve pinned at its declared 0.20 | **6.3449 m** and still rising |
//!
//! The AUTO run settles; the MANUAL run does not, and leaves any band around the
//! setpoint on its way to the tank's roof. That separation is the whole gate.
//!
//! **The offset is real and is not a defect.** A proportional loop with no
//! integral term cannot hold a setpoint against a load: it settles wherever the
//! error is large enough for `K·e` to open the drain by as much as the inflow
//! needs. `+0.74 m` at a setpoint of 4 m is that error, and it is exactly the half
//! of M8.3's disturbance-rejection pair this slice is meant to supply — which is
//! why `ProportionalController` ships with no bias term to hide it behind.
//!
//! The plants are declared here rather than shipped, and that is the same call
//! `leak_reference.rs` already makes for its vacuum and gas plants: a plant built
//! to expose one behaviour is a fixture, and the thirteen files in `scenarios/`
//! are the regression anchor this milestone is measured against. The wired demo
//! that regulates — diffable against `tank_pump_valve.toml` — is M8.4's.

use refinery_core::graph::{ControlMode, ControlledValue, LoopId, MeasuredVariable, NodeId};
use refinery_core::snapshot::Command;
use refinery_core::units::Meter;
use refinery_core::{Engine, SimError};

// ------------------------------------------------------------------ fixtures

/// Header → fixed feed valve → tank → controlled drain valve → rundown.
///
/// Two things about the sizing are load bearing rather than arbitrary:
///
/// - **The inflow barely depends on the level.** The header sits at 5 bar against
///   a tank whose bottom pressure spans ~1.0 to ~2.0 bar over its full height, so
///   the feed valve's own `ΔP` dominates and the fill rate is nearly constant.
///   A feed that fell off steeply with level would self-regulate on its own, and
///   the counterfactual would be measuring the plant instead of the loop.
/// - **The drain valve at its declared 0.20 cannot pass the inflow** anywhere
///   below the tank's roof, which is what makes the MANUAL run RUN AWAY instead of
///   settling somewhere else. Fork 6 asks for a plant BUILT this way, because a
///   counterfactual that happens to settle proves nothing.
const PLANT: &str = r#"
[meta]
name = "level_control_gate"
description = "Header through a fixed feed valve into a tank drained by a controlled valve."

[simulation]
dt = 0.5

[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"

[nodes.header]
type = "source"
pressure_bar = 5.0
temperature_c = 20.0

[nodes.feed_valve]
type = "valve"
kv = 20.0
opening = 0.4

[nodes.control_tank]
type = "tank"
area_m2 = 3.0
height_m = 10.0
initial_level_m = 4.0
temperature_c = 20.0

[nodes.drain_valve]
type = "valve"
kv = 60.0
opening = 0.2

[nodes.rundown]
type = "sink"
pressure_bar = 1.01325

[[pipes]]
name = "feed_line"
from = "header"
to = "feed_valve"
length_m = 10.0
diameter_m = 0.10

[[pipes]]
name = "fill_line"
from = "feed_valve"
to = "control_tank"
length_m = 10.0
diameter_m = 0.10

[[pipes]]
name = "drain_line"
from = "control_tank"
to = "drain_valve"
length_m = 10.0
diameter_m = 0.10

[[pipes]]
name = "rundown_line"
from = "drain_valve"
to = "rundown"
length_m = 10.0
diameter_m = 0.10

[[controls]]
name = "tank_level"
measurement = { node = "control_tank", variable = "level" }
actuator = "drain_valve"
algorithm = "p"
mode = "auto"
setpoint_m = 4.0
gain_per_m = 0.5
"#;

/// The setpoint every fixture below starts at, and the band the AUTO run must
/// hold. The band is `± BAND` around it and is deliberately *loose*: this gate is
/// about the separation between two runs, not about tuning quality, and a tight
/// band would turn a controller change into a failure for the wrong reason.
const SETPOINT_M: f64 = 4.0;
const BAND_M: f64 = 1.0;

/// How long the runs are. The AUTO plant is settled to 1e-4 m by ~6 000 ticks
/// (measured, see the module header); 8 000 leaves margin without making the
/// suite slow — the whole file is a couple of seconds in a debug build.
const TICKS: u64 = 8_000;

fn engine_from(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("fixture parses");
    refinery_scenarios::build_engine(&file).expect("fixture builds")
}

fn refusal(src: &str) -> String {
    let parsed = match refinery_scenarios::load_str(src) {
        Ok(file) => file,
        // A `deny_unknown_fields` rejection lands here rather than in
        // `build_engine`, and both are refusals of the same file — the caller
        // asserts on the text, not on which stage produced it.
        Err(e) => return e.to_string(),
    };
    match refinery_scenarios::build_engine(&parsed) {
        Ok(_) => panic!("this scenario was expected to be refused, and loaded"),
        Err(e) => e.to_string(),
    }
}

/// The plant's one tank, by name.
fn tank_id(engine: &Engine) -> NodeId {
    engine
        .graph
        .find_node("control_tank")
        .expect("the fixture has a control_tank")
}

/// The tank's level right now, read through the engine's own single owner of that
/// question (`PlantGraph::measure`) so a test cannot measure a level by a
/// different rule than the controller does.
fn level_now(engine: &Engine) -> f64 {
    match engine
        .graph
        .measure(&engine.slate, tank_id(engine), MeasuredVariable::Level)
        .expect("the control tank has a level")
    {
        ControlledValue::Level { m } => m.value(),
    }
}

/// Run `ticks` ticks, returning the level at the end and the highest level seen.
fn run(engine: &mut Engine, ticks: u64) -> (f64, f64) {
    let mut peak = level_now(engine);
    for t in 0..ticks {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        peak = peak.max(level_now(engine));
    }
    (level_now(engine), peak)
}

// ----------------------------------------------------- gate 1: the control

/// **Gate 1, and it is the control.** The same plant with the loop parked in
/// MANUAL must leave the band the AUTO run holds.
///
/// Written before the AUTO assertion and asserted in both directions in one test,
/// because the two halves are only meaningful together: if the MANUAL run also
/// stayed in band, the AUTO half would be measuring `ρgh` self-regulation and
/// would pass with the controller deleted.
///
/// The MANUAL run's fixed opening is the one the file declares, so the two runs
/// differ in exactly one character of scenario text.
#[test]
fn a_parked_loop_leaves_the_band_the_auto_loop_holds() {
    let (auto_final, auto_peak) = run(&mut engine_from(PLANT), TICKS);
    let manual_src = PLANT.replace(r#"mode = "auto""#, r#"mode = "manual""#);
    let (manual_final, _) = run(&mut engine_from(&manual_src), TICKS);

    // The AUTO run never leaves the band. `peak` rather than the final value:
    // a loop that overshot and came back would pass a final-value assertion.
    assert!(
        auto_peak <= SETPOINT_M + BAND_M,
        "the AUTO run left the band it is supposed to hold: peak {auto_peak:.4} m \
         against a setpoint of {SETPOINT_M} ± {BAND_M} m"
    );
    // Sitting ABOVE the setpoint is the proportional loop's defining behaviour,
    // not a defect: the drain valve is only open because the error is nonzero.
    // Measured +0.7386 m; asserted only as a sign, since its size is a tuning
    // consequence and M8.3's gate is what measures it.
    assert!(
        auto_final > SETPOINT_M,
        "a proportional loop on a draining tank must settle ABOVE its setpoint — \
         `u = K·e` needs a positive error to open the valve at all. Got \
         {auto_final:.4} m against {SETPOINT_M} m"
    );

    // The control: without the loop writing, the same plant runs away.
    assert!(
        manual_final > SETPOINT_M + BAND_M,
        "THE COUNTERFACTUAL FAILED, which invalidates the assertion above rather \
         than merely failing on its own: with the loop in MANUAL the level ended \
         at {manual_final:.4} m, still inside the band the AUTO run is credited \
         with holding. Either the plant self-regulates without the controller \
         (resize it: the drain valve at its declared opening must not be able to \
         pass the inflow) or the loop is being run in MANUAL too"
    );
    assert!(
        manual_final > auto_final,
        "the MANUAL run ended at {manual_final:.4} m and the AUTO run at \
         {auto_final:.4} m; the parked plant is supposed to fill"
    );
}

// --------------------------------------------- gate 2: setpoint tracking

/// **Gate 2.** Step the setpoint mid-run; the level must follow it.
///
/// This is what the steady-state assertion cannot prove: a self-regulating tank's
/// equilibrium is a function of the valve position alone, so only *moving the
/// setpoint and watching the level move with it* shows the output is a function
/// of the setpoint at all.
///
/// **The step is downward, and that is a measured choice rather than a stylistic
/// one.** An upward step of this size drives the drain valve fully shut in one
/// tick, which the hydraulic solver does not survive — for reasons that have
/// nothing to do with control and that a human `SetValveOpening` reproduces
/// exactly. That is pinned separately, by
/// `a_branch_shut_in_one_tick_stalls_the_solver_whoever_shuts_it`, so this gate
/// can measure tracking instead of measuring the solver.
///
/// The asserted quantity is the DIRECTION and the SIZE of the move, and both
/// bounds are derived rather than fitted:
///
/// - The level must FALL, because the setpoint fell. A gain applied to the
///   measurement instead of the error — one of the mutations this slice owes —
///   makes the output a function of the level alone, so the step moves nothing
///   and this assertion is the one that fires. **Verified rather than predicted**:
///   the edit was applied, checked to compile, and run, and it failed this gate
///   (and gate 1, and the solver-stall characterization). M8.4 owns the full
///   pass; this one was run early because a docstring saying "this is the
///   assertion it has to get past" is a claim, not a hope.
/// - The fall must be **smaller than the step**. At the lower level the tank's
///   own head is smaller, so more valve opening is needed to pass the same
///   inflow, so the proportional offset `e = u/K` is larger. The level moves by
///   `Δsp + (e₂ − e₁)` with `e₂ > e₁`, and both are negative-going. Measured:
///   0.9049 m for a 1.0 m step (offset 0.739 m at 4 m, 0.834 m at 3 m).
/// - A floor of half the step is slack, and says only that it tracked rather than
///   twitched.
#[test]
fn the_level_follows_a_setpoint_step() {
    const STEP_TO_M: f64 = 3.0;
    let mut engine = engine_from(PLANT);
    let (before, _) = run(&mut engine, TICKS);

    engine
        .apply(Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Level {
                m: Meter(STEP_TO_M),
            },
        })
        .expect("a reachable setpoint is accepted");

    let (after, _) = run(&mut engine, TICKS);
    let moved = before - after;
    let step = SETPOINT_M - STEP_TO_M;

    assert!(
        moved > 0.0,
        "the setpoint fell by {step} m and the level did not follow: {before:.4} m \
         -> {after:.4} m. A controller whose output does not depend on the \
         setpoint passes every steady-state assertion and fails this one"
    );
    assert!(
        moved < step,
        "the level moved {moved:.4} m for a {step} m setpoint step, which is not \
         less than the step. A proportional loop's offset GROWS as the tank's own \
         head shrinks, so a downward move is bounded above by the step itself"
    );
    assert!(
        moved > step / 2.0,
        "the level moved only {moved:.4} m for a {step} m setpoint step — it \
         twitched rather than tracked"
    );
    assert!(
        after > STEP_TO_M,
        "the loop must still sit above its setpoint after the step: {after:.4} m \
         against {STEP_TO_M} m"
    );
}

/// A branch driven to zero flow in ONE tick stalls the hydraulic solver — and a
/// human command does it exactly as a controller does.
///
/// **A characterization test, and the control is the point of it.** M8.2 makes
/// something newly reachable: before it, a valve opening only moved when a person
/// sent a command, and now a loop can slam one shut between two ticks. The
/// question a gate has to answer is whether the seam introduced a failure or
/// merely reached one, and the two halves below answer it — the same plant, the
/// same endpoint, once by `SetSetpoint` and once by `SetValveOpening`, both
/// refused by the solver in the same way. So this is not the control loop's
/// defect, and M8.2 does not fix it inside the control loop, where a rate limit
/// would hide it rather than mend it.
///
/// **What was measured**, on the plant above after 8 000 ticks:
///
/// - The residual falls MONOTONICALLY and by about 0.36% per iteration —
///   `4.158 → 3.344` over the 50-iteration cap. Newton is crawling, not
///   oscillating and not stuck: the pipe characteristic `ΔP = α·Q|Q|` has an
///   unbounded `dQ/dΔP` as `Q → 0`, so the Newton step from a warm start carrying
///   3.3 kg/s is enormous and the line search cuts it back to almost nothing.
/// - The same endpoint reached GRADUALLY converges (20 ticks of `0.20 → 0.00`),
///   and so does a cold start already at the shut state. It is the jump that
///   fails, not the state.
/// - `gain_per_m = 0.05`, which never fully shuts the valve, survives the same
///   step; so does the `simple` flow solver; so does any target opening at or
///   above 0.01.
///
/// This belongs to a solver slice, not to M8. It un-defers with a warm-start or
/// step-damping fix in `newton_flow`, at which point this test fails and should
/// then assert that both halves CONVERGE.
#[test]
fn a_branch_shut_in_one_tick_stalls_the_solver_whoever_shuts_it() {
    // By controller: a setpoint step big enough that `u = K·e` clamps to zero.
    let mut by_loop = engine_from(PLANT);
    run(&mut by_loop, TICKS);
    by_loop
        .apply(Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Level { m: Meter(6.0) },
        })
        .expect("a reachable setpoint is accepted");
    let loop_err = by_loop
        .tick()
        .expect_err("SOLVER FIXED: a controller can now shut a branch in one tick");

    // By hand, on the same plant with the loop parked: the identical endpoint,
    // written by a command that has existed since M1.
    let mut by_hand = engine_from(&PLANT.replace(r#"mode = "auto""#, r#"mode = "manual""#));
    run(&mut by_hand, TICKS);
    let valve = by_hand.graph.find_node("drain_valve").expect("drain valve");
    by_hand
        .apply(Command::SetValveOpening {
            node: valve,
            opening: 0.0,
        })
        .expect("in MANUAL a human drives the valve");
    let hand_err = by_hand
        .tick()
        .expect_err("SOLVER FIXED: a hand-shut branch now converges");

    for (who, err) in [("the control loop", loop_err), ("a hand command", hand_err)] {
        match err {
            SimError::SolverDiverged {
                residual_history, ..
            } => {
                // Monotone descent is the claim that makes this a stall rather
                // than a divergence, and it is what says the fix belongs in the
                // step control rather than in the model.
                let monotone = residual_history.windows(2).all(|w| w[1] <= w[0]);
                assert!(
                    monotone,
                    "{who}: the residual history is not monotone, so this is no \
                     longer the slow-crawl stall this test characterizes: \
                     {residual_history:?}"
                );
            }
            other => panic!(
                "{who} was expected to stall the solver and instead produced {other}. \
                 If the solver was fixed, both halves should now converge and this \
                 test should assert that"
            ),
        }
    }
}

// ------------------------------------------- what the snapshot reports

/// The reported measurement is the one the controller ACTED ON — one tick old.
///
/// An exact identity, not a tolerance: the loop captures the level at the top of
/// the tick, so the value a snapshot reports after tick *n* is bit-for-bit the
/// level that stood at the end of tick *n − 1*, and the level *now* has moved
/// away from it.
///
/// This is the gate that sees the tick ORDER. A loop moved to run after the solve
/// would report a measurement equal to the current level instead, which the second
/// assertion refuses.
#[test]
fn the_reported_measurement_is_the_one_the_controller_acted_on() {
    let mut engine = engine_from(PLANT);
    // Away from the start, where the level is still moving fast enough for the
    // two readings to be distinguishable at all.
    run(&mut engine, 100);

    let before_tick = level_now(&engine);
    engine.tick().expect("tick");
    let now = level_now(&engine);

    let reported = match engine.snapshot().controls[0].measurement {
        ControlledValue::Level { m } => m.value(),
    };
    assert_eq!(
        reported, before_tick,
        "the snapshot must report the measurement the controller acted on — the \
         level at the TOP of this tick — and not a fresh read. Reporting the fresh \
         one makes a lagging loop look instantaneous, which hides the lag from the \
         person debugging it"
    );
    assert_ne!(
        reported, now,
        "the reported measurement and the current level are identical, so this \
         gate cannot tell a one-tick lag from no lag at all. Either the plant is \
         at steady state here (move the sample earlier) or the loop is reading \
         this tick's state instead of the previous one"
    );
}

/// A plant with no loop emits no `controls` key at all — the byte-identity move.
///
/// `skip_serializing_if = "Vec::is_empty"` is the mechanism; this is the claim.
/// Every scenario in `scenarios/` was written before M8 and declares no loop, so
/// none of their snapshots may grow a field because this seam landed.
#[test]
fn a_plant_with_no_loop_reports_no_controls_field() {
    const SHIPPED: [(&str, &str); 13] = [
        (
            "cooler_chiller",
            include_str!("../../../scenarios/cooler_chiller.toml"),
        ),
        (
            "crude_column",
            include_str!("../../../scenarios/crude_column.toml"),
        ),
        (
            "crude_column_cascade",
            include_str!("../../../scenarios/crude_column_cascade.toml"),
        ),
        (
            "fcc_plant",
            include_str!("../../../scenarios/fcc_plant.toml"),
        ),
        (
            "fcc_reactor",
            include_str!("../../../scenarios/fcc_reactor.toml"),
        ),
        (
            "furnace_heater",
            include_str!("../../../scenarios/furnace_heater.toml"),
        ),
        ("gas_line", include_str!("../../../scenarios/gas_line.toml")),
        (
            "gas_valve",
            include_str!("../../../scenarios/gas_valve.toml"),
        ),
        (
            "heat_recovery",
            include_str!("../../../scenarios/heat_recovery.toml"),
        ),
        (
            "knockout_drum",
            include_str!("../../../scenarios/knockout_drum.toml"),
        ),
        (
            "leaking_line",
            include_str!("../../../scenarios/leaking_line.toml"),
        ),
        (
            "relief_blowdown",
            include_str!("../../../scenarios/relief_blowdown.toml"),
        ),
        (
            "tank_pump_valve",
            include_str!("../../../scenarios/tank_pump_valve.toml"),
        ),
    ];

    for (name, src) in SHIPPED {
        let mut engine = engine_from(src);
        engine
            .tick()
            .unwrap_or_else(|e| panic!("{name} tick 1: {e}"));
        let json = serde_json::to_string(&engine.snapshot()).expect("snapshot serializes");
        assert!(
            !json.contains("\"controls\""),
            "{name} grew a `controls` key. Every scenario in this repo was written \
             before M8 and declares no loop; a key appearing on one of them is the \
             seam leaking into plants that never asked for it"
        );
        assert!(
            engine.snapshot().controls.is_empty(),
            "{name} acquired a control loop it does not declare"
        );
    }
}

/// A loop reports a real measurement before the first tick.
///
/// A level is a STORED quantity — `TankState::mass` is real from load — so the
/// loop is seeded by the same read the tick pass uses, and a faceplate drawn
/// before anything is solved shows the tank's declared level rather than a NaN or
/// an absent field. (Fork 3's first version got this wrong and justified
/// `initial_output` with "tick 0 has no previous state", which is false for
/// exactly this variable.)
#[test]
fn a_loop_has_a_real_measurement_before_the_first_tick() {
    let engine = engine_from(PLANT);
    let snapshot = engine.snapshot();
    assert_eq!(snapshot.tick, 0);
    let reported = match snapshot.controls[0].measurement {
        ControlledValue::Level { m } => m.value(),
    };
    assert_eq!(
        reported,
        level_now(&engine),
        "a level loop's measurement is real from load and must not wait for a tick"
    );
    // Seeded from the valve's own declared opening, which is what MANUAL would
    // report and what AUTO overwrites on tick 1.
    assert_eq!(snapshot.controls[0].output, 0.2);
    assert_eq!(snapshot.controls[0].algorithm, "proportional");
    assert_eq!(snapshot.controls[0].mode, ControlMode::Auto);
}

// ------------------------------------------------- the command surface

/// A manual valve write is refused while a loop owns the valve in AUTO, and
/// accepted the moment the loop is put in MANUAL.
///
/// Both halves in one test, because the refusal alone would pass on an
/// implementation that refused `SetValveOpening` on that valve unconditionally —
/// which would break what MANUAL is *for*.
#[test]
fn a_manual_valve_write_is_refused_under_auto_and_allowed_under_manual() {
    let mut engine = engine_from(PLANT);
    let valve = engine.graph.find_node("drain_valve").expect("drain valve");
    let loop_id = LoopId(0);

    let refused = engine
        .apply(Command::SetValveOpening {
            node: valve,
            opening: 0.9,
        })
        .expect_err("a valve under a loop in AUTO must refuse a manual write");
    let text = refused.to_string();
    assert!(
        text.contains("tank_level") && text.contains("AUTO"),
        "the refusal must name the loop that owns the opening and say why: {text}"
    );
    assert!(
        matches!(refused, SimError::InvalidCommand(_)),
        "a contested write is an invalid command, not a numerical failure"
    );

    engine
        .apply(Command::SetControllerMode {
            loop_id,
            mode: ControlMode::Manual,
        })
        .expect("a loop can be put in manual");
    engine
        .apply(Command::SetValveOpening {
            node: valve,
            opening: 0.9,
        })
        .expect("in MANUAL the existing command drives the actuator unchanged");

    // And the write SURVIVES a tick, which is the thing the AUTO refusal exists
    // to protect: a command that is overwritten at the top of the next tick is a
    // command that appears to work and does not.
    engine.tick().expect("tick");
    let snapshot = engine.snapshot();
    assert_eq!(
        snapshot.controls[0].output, 0.9,
        "in MANUAL the faceplate tracks the actuator's real opening"
    );
}

/// A setpoint the plant cannot reach is refused, and by the same rule at load and
/// at runtime.
#[test]
fn an_unreachable_setpoint_is_refused() {
    let mut engine = engine_from(PLANT);
    let err = engine
        .apply(Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Level { m: Meter(12.0) },
        })
        .expect_err("a setpoint above the tank's height is unreachable");
    assert!(
        err.to_string().contains("[0, 10]"),
        "the refusal must name the range it is enforcing: {err}"
    );

    // The same number, declared in the file instead of commanded, is refused by
    // the same check — one owner, so a loop cannot load with a value this command
    // would then reject.
    let text = refusal(&PLANT.replace("setpoint_m = 4.0", "setpoint_m = 12.0"));
    assert!(
        text.contains("[0, 10]"),
        "a declared setpoint must meet the same bound as a commanded one: {text}"
    );
}

/// A `LoopId` from outside names a loop or is refused — never an index panic.
#[test]
fn an_unknown_loop_id_is_refused() {
    let mut engine = engine_from(PLANT);
    for cmd in [
        Command::SetControllerMode {
            loop_id: LoopId(7),
            mode: ControlMode::Manual,
        },
        Command::SetSetpoint {
            loop_id: LoopId(7),
            value: ControlledValue::Level { m: Meter(5.0) },
        },
    ] {
        let err = engine
            .apply(cmd)
            .expect_err("an out-of-range LoopId is an invalid command");
        assert!(
            matches!(err, SimError::InvalidCommand(_)),
            "got {err} — a LoopId arrives from outside the engine, so it is \
             refused rather than indexed"
        );
    }
}

// ------------------------------------------------------- load refusals

/// Fork 5's four refusals, plus the format ones, each with the thing its message
/// must say.
///
/// One table rather than a test each: they differ only in the edit and the
/// expected substring, and a table makes it obvious when one is missing.
#[test]
fn a_control_table_is_refused_where_fork_5_says_it_must_be() {
    // (what the edit does, the edit, a phrase the refusal must contain)
    let cases: Vec<(&str, String, &str)> = vec![
        (
            "tuning that belongs to the other algorithm",
            PLANT.replace(
                "gain_per_m = 0.5",
                "gain_per_m = 0.5\nintegral_time_s = 120.0",
            ),
            "integral term",
        ),
        (
            "two loops on one actuator",
            format!(
                "{PLANT}\n[[controls]]\nname = \"second\"\nmeasurement = {{ node = \
                 \"control_tank\", variable = \"level\" }}\nactuator = \"drain_valve\"\n\
                 algorithm = \"p\"\nmode = \"auto\"\nsetpoint_m = 3.0\ngain_per_m = 0.5\n"
            ),
            "both actuate",
        ),
        (
            "a level measured on a node that is not a tank",
            PLANT.replace(
                r#"measurement = { node = "control_tank", variable = "level" }"#,
                r#"measurement = { node = "feed_valve", variable = "level" }"#,
            ),
            "not a tank",
        ),
        (
            "an actuator that is not a valve",
            PLANT.replace(
                r#"actuator = "drain_valve""#,
                r#"actuator = "control_tank""#,
            ),
            "not a valve",
        ),
        (
            "an unknown algorithm",
            PLANT.replace(r#"algorithm = "p""#, r#"algorithm = "pi""#),
            "unknown algorithm",
        ),
        (
            "an unknown measured variable",
            PLANT.replace(r#"variable = "level""#, r#"variable = "pressure""#),
            "unknown variable",
        ),
        (
            "an unknown mode",
            PLANT.replace(r#"mode = "auto""#, r#"mode = "cascade""#),
            "unknown mode",
        ),
        (
            "no setpoint key",
            PLANT.replace("setpoint_m = 4.0\n", ""),
            "setpoint_m",
        ),
        (
            "no gain key",
            PLANT.replace("gain_per_m = 0.5\n", ""),
            "gain_per_m",
        ),
        (
            "a gain that does nothing",
            PLANT.replace("gain_per_m = 0.5", "gain_per_m = 0.0"),
            "finite and > 0",
        ),
        // Same branch as the zero gain, and gated separately because the type's
        // doc makes a stronger claim than the branch does: reverse action is
        // refused as a CONCEPT, not merely as an out-of-range number. A reader
        // relying on that sentence needs a test that checks it.
        (
            "a reverse-acting gain",
            PLANT.replace("gain_per_m = 0.5", "gain_per_m = -0.5"),
            "reverse action",
        ),
        (
            "two loops with one name",
            format!(
                "{PLANT}\n[[controls]]\nname = \"tank_level\"\nmeasurement = {{ node = \
                 \"control_tank\", variable = \"level\" }}\nactuator = \"feed_valve\"\n\
                 algorithm = \"p\"\nmode = \"auto\"\nsetpoint_m = 3.0\ngain_per_m = 0.5\n"
            ),
            "are called",
        ),
        (
            "a misspelled key, which `deny_unknown_fields` is here for",
            PLANT.replace("gain_per_m = 0.5", "gain_per_metre = 0.5"),
            "gain_per_metre",
        ),
    ];

    for (what, src, expected) in cases {
        let text = refusal(&src);
        assert!(
            text.contains(expected),
            "refusing {what}: the message must contain {expected:?}, and said: {text}"
        );
    }
}

/// A loop pointed at a relief valve is refused with its OWN reason, not with
/// "not a valve".
///
/// A relief valve IS a valve, and the point is that its opening is not a setpoint
/// at all — it is a memoryless function of its own inlet pressure, recomputed on
/// every solve (§3a fork 5). A loop writing one would be overwritten before the
/// tick ended, which is the same failure `Command::SetValveOpening` already
/// refuses it for.
#[test]
fn a_loop_on_a_relief_valve_is_refused_with_its_own_reason() {
    const RELIEF: &str = r#"
[meta]
name = "relief_actuator"
[simulation]
dt = 0.5
[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"

[nodes.control_tank]
type = "tank"
area_m2 = 3.0
height_m = 10.0
initial_level_m = 4.0
temperature_c = 20.0

[nodes.psv]
type = "relief_valve"
kv = 60.0
set_pressure_bar = 3.0
accumulation_bar = 0.3

[nodes.flare]
type = "sink"
pressure_bar = 1.01325

[[pipes]]
name = "riser"
from = "control_tank"
to = "psv"
length_m = 5.0
diameter_m = 0.10

[[pipes]]
name = "tail"
from = "psv"
to = "flare"
length_m = 5.0
diameter_m = 0.10

[[controls]]
name = "tank_level"
measurement = { node = "control_tank", variable = "level" }
actuator = "psv"
algorithm = "p"
mode = "auto"
setpoint_m = 4.0
gain_per_m = 0.5
"#;
    let text = refusal(RELIEF);
    assert!(
        text.contains("relief valve") && text.contains("inlet pressure"),
        "a relief valve must be refused for being pressure-actuated, not for \
         failing to be a valve: {text}"
    );
}
