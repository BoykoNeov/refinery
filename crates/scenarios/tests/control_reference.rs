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
//! **M8.3 adds the integral term, and its two gates are a PAIR and a PLANT.** The
//! disturbance-rejection gate below runs the SAME leak on the SAME plant twice,
//! once with `algorithm = "p"` and once with `"pi"`, because neither half proves
//! anything alone — and its discriminating assertion is not "the PI loop has no
//! offset", which a large enough proportional gain also produces. It is that the
//! P loop's level move is FORCED to be `Δu/K` (measured 0.477229 m against the
//! 0.477230 m its own valve travel forces) while the PI loop moved its valve
//! slightly FURTHER and its level by 0.0013 m. A proportional loop cannot hold a
//! level while its output changes; that is what the pair discriminates. The windup gate needs a plant
//! built to saturate, and gets its own, for the reason fork 6 states in advance.
//!
//! The plants are declared here rather than shipped, and that is the same call
//! `leak_reference.rs` already makes for its vacuum and gas plants: a plant built
//! to expose one behaviour is a fixture, and the thirteen files in `scenarios/`
//! are the regression anchor this milestone is measured against. The wired demo
//! that regulates — diffable against `tank_pump_valve.toml` — is M8.4's.

use refinery_core::graph::{ControlMode, ControlledValue, LoopId, MeasuredVariable, NodeId};
use refinery_core::snapshot::Command;
use refinery_core::units::{Meter, SquareMeter};
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

/// The actuator position the loop last put on its faceplate.
///
/// Read off the snapshot rather than off the valve, deliberately: it is the
/// number a frontend sees, and in MANUAL it is the number that has to TRACK a
/// valve the loop is not writing.
fn output_now(engine: &Engine) -> f64 {
    engine
        .snapshot()
        .controls
        .first()
        .expect("the fixture declares one loop")
        .output
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
    // The M8.2 plant with M8.3's algorithm and the two keys it requires. The
    // PI-only cases below edit THIS rather than `PLANT`, and it is asserted to
    // load first: a refusal case built on a file that was already invalid proves
    // nothing about the key it names.
    let pi_plant = PLANT
        .replace(r#"algorithm = "p""#, r#"algorithm = "pi""#)
        .replace(
            "gain_per_m = 0.5
",
            "gain_per_m = 0.5
integral_time_s = 300.0
initial_output = 0.2
",
        );
    engine_from(&pi_plant);

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
        // `"pi"` was this case's edit in M8.2 and is a real algorithm now, which
        // is the expiry that slice's comment named. The edit moved to `"pid"`
        // rather than the case being deleted: derivative action is the next thing
        // a file will reach for, and the refusal says why it is deferred.
        (
            "an unknown algorithm",
            PLANT.replace(r#"algorithm = "p""#, r#"algorithm = "pid""#),
            "unknown algorithm",
        ),
        // The four refusals M8.3 makes reachable. Two of them are the mirrors
        // M8.2 could only record in a comment, because until `"pi"` was
        // selectable the unknown-algorithm arm is what a file writing either key
        // already hit.
        (
            "the integral time missing on a PI loop",
            pi_plant.replace("integral_time_s = 300.0\n", ""),
            "integral_time_s",
        ),
        (
            "the loop's memory missing on a PI loop",
            pi_plant.replace("initial_output = 0.2\n", ""),
            "initial_output",
        ),
        (
            "a memory on a controller that has none",
            PLANT.replace("gain_per_m = 0.5", "gain_per_m = 0.5\ninitial_output = 0.2"),
            "no memory for it to be the initial condition of",
        ),
        (
            "a memory that is not an actuator position",
            pi_plant.replace("initial_output = 0.2", "initial_output = 1.4"),
            "not a finite fraction in [0, 1]",
        ),
        (
            "an integral time that divides by zero",
            pi_plant.replace("integral_time_s = 300.0", "integral_time_s = 0.0"),
            "finite and > 0 s",
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

// ============================================================ M8.3: the integral term

/// Gate 3's plant: gate 1's, plus an atmosphere and a declared leak path.
///
/// **The leak has to be declared at LOAD, and that is not a formality** — `apply`
/// refuses `PuncturePipe` on a pipe whose file says nothing about leaking (§3b
/// fork C), because writing an area onto a pipe with no orifice is the M6.0 defect
/// where a command stores a number nothing reads. So the gate's plant declares
/// `leak_to`, exactly as fork 6 says it must.
///
/// Nothing else differs from gate 1's plant, deliberately: the disturbance gate
/// and the counterfactual gate should be measuring the same hydraulics.
const LEAK_PLANT: &str = r#"
[meta]
name = "level_control_disturbance"
description = "Gate 1's plant with a declared leak on the fill line."

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

[nodes.outside]
type = "atmosphere"

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
leak_to = "outside"

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
"#;

/// The proportional half of gate 3's pair — gate 1's loop verbatim.
const P_LOOP: &str = r#"
[[controls]]
name = "tank_level"
measurement = { node = "control_tank", variable = "level" }
actuator = "drain_valve"
algorithm = "p"
mode = "auto"
setpoint_m = 4.0
gain_per_m = 0.5
"#;

/// The integral half of gate 3's pair.
///
/// **The same gain**, which is what makes the two halves comparable: the pair's
/// discriminating quantity is `Δu/K`, and a pair that also changed `K` could not
/// say whether the integral term or the tuning did the work.
///
/// `integral_time_s = 300.0` was measured rather than derived from a tuning rule.
/// The tank holds 3 m³ per metre against a feed of a few kg/s, so its own time
/// constant is minutes; 300 s settles to within 0.01 m of setpoint in 12 000
/// ticks (6 000 s) without the ringing a faster reset produces — 200 s reaches
/// ±0.2 m and stays there. `initial_output = 0.2` is the valve's declared opening,
/// so the loop starts holding the position the file already gave it.
const PI_LOOP: &str = r#"
[[controls]]
name = "tank_level"
measurement = { node = "control_tank", variable = "level" }
actuator = "drain_valve"
algorithm = "pi"
mode = "auto"
setpoint_m = 4.0
gain_per_m = 0.5
integral_time_s = 300.0
initial_output = 0.2
"#;

/// How long each half of gate 3 runs before and after the disturbance.
///
/// Measured, not chosen: the PI loop is within 0.01 m of setpoint by 12 000 ticks
/// (6 000 s) from a cold start and the P loop is settled to 1e-4 m well before
/// that. Shorter runs measure the transient rather than the steady state, and the
/// whole quantity this gate is about is a steady-state one.
const REJECT_TICKS: u64 = 12_000;

/// The hole. 5e-4 m² ≈ a 25 mm puncture, and the size is load-bearing in both
/// directions, which is why it is stated here rather than tuned into the test.
///
/// - Smaller (1e-4 m²) and the disturbance is too weak to separate the halves:
///   the P loop's level moves 0.10 m, which is within the noise of "did it settle".
/// - Larger (1e-3 m²) and it stops being a disturbance and becomes a different
///   plant: the leak takes essentially the whole feed, both loops drive the drain
///   valve to zero and sit there, and neither is regulating anything. Measured —
///   the PI loop ends at 2.57 m with its valve shut.
///
/// At 5e-4 m² both loops keep a working valve position and the pair discriminates.
const HOLE_M2: f64 = 5.0e-4;

/// Run one half of gate 3, returning `(level, output)` before and after the leak.
fn reject_disturbance(loop_table: &str) -> ((f64, f64), (f64, f64)) {
    let mut engine = engine_from(&format!("{LEAK_PLANT}{loop_table}"));
    run(&mut engine, REJECT_TICKS);
    let before = (level_now(&engine), output_now(&engine));

    let fill = engine
        .snapshot()
        .edges
        .iter()
        .find(|e| e.name == "fill_line")
        .expect("the plant declares a fill_line")
        .id;
    engine
        .apply(Command::PuncturePipe {
            edge: fill,
            area: SquareMeter(HOLE_M2),
        })
        .expect("the fill line declares `leak_to`, so it is punctureable");

    run(&mut engine, REJECT_TICKS);
    (before, (level_now(&engine), output_now(&engine)))
}

/// **Gate 3, and it is a PAIR.** The same plant, the same leak, twice: once with
/// `algorithm = "p"` and once with `"pi"`. Neither half proves the integral term
/// alone, which is the whole reason M8.2 shipped the proportional loop by itself.
///
/// **The discriminating assertion is not "the PI loop has no offset".** That is
/// the obvious one and it does not discriminate: a proportional loop's offset is
/// `e = u/K`, so a large enough gain shrinks it toward zero and passes a
/// no-offset test with no integral term anywhere. What a proportional loop
/// *cannot* do is move its output while holding its level, because `u = K·e` makes
/// the two the same statement. So the pair asserts:
///
/// - **P half:** the level moved by exactly `Δu/K`. This is an identity of the
///   algorithm rather than a fitted bound, and it is what makes the offset a
///   *consequence* rather than an observation — measured 0.477229 m against the
///   0.477230 m its own 0.238615 of valve travel forces, agreeing to 1.2e-6 m.
/// - **PI half:** MORE valve travel, and a level that did not move. The valve
///   moved 0.260950 (further than the P loop, since it is holding a lower level
///   against the same leak) while the level moved 0.001340 m — against the
///   0.521900 m that same travel would have forced on a proportional loop.
///
/// The two runs are also asserted to have used *different algorithms*, by name off
/// the faceplate. A pair whose halves silently built the same controller would
/// pass every numeric assertion in the P direction and prove nothing, and the
/// fixture strings differ by enough characters to make that a real mistake.
#[test]
fn the_integral_term_holds_a_level_a_proportional_loop_can_only_offset() {
    let ((p_level_0, p_out_0), (p_level_1, p_out_1)) = reject_disturbance(P_LOOP);
    let ((pi_level_0, pi_out_0), (pi_level_1, pi_out_1)) = reject_disturbance(PI_LOOP);

    let p_travel = p_out_0 - p_out_1;
    let pi_travel = pi_out_0 - pi_out_1;
    let p_drop = p_level_0 - p_level_1;
    let pi_drop = pi_level_0 - pi_level_1;

    // The disturbance has to have DONE something, or every assertion below is a
    // statement about a plant nothing happened to.
    assert!(
        p_travel > 0.15 && pi_travel > 0.15,
        "the leak must move both loops' valves by a measurable amount, and moved \
         {p_travel:.4} (P) and {pi_travel:.4} (PI). A disturbance nothing responds \
         to makes this whole gate vacuous"
    );

    // The P half: `u = K·e` forces `Δlevel = Δu/K`. The tolerance is the width of
    // the steady state rather than a fitted margin — both runs settle to ~1e-4 m,
    // so a millimetre is the honest bound, and the measured disagreement is
    // 1.2e-6 m, three orders inside it.
    let forced = p_travel / 0.5;
    assert!(
        (p_drop - forced).abs() < 1.0e-3,
        "a proportional loop's level move is its output move divided by the gain, \
         and this run moved the level {p_drop:.4} m while {forced:.4} m is what its \
         {p_travel:.4} of valve travel forces. If these disagree, either the error \
         term or the gain is not what `u = K·e` says"
    );
    assert!(
        p_drop > 0.3,
        "the proportional loop's offset must be large enough to be the visible half \
         of this pair, and moved only {p_drop:.4} m"
    );

    // The PI half: the same travel, and the level stayed where the setpoint is.
    // A tenth of a metre on a ten-metre tank: a stated 1% band, not the measured
    // 0.0013 m rounded up. Two orders of margin is what says this is the integral
    // term rather than a lucky fixture.
    assert!(
        pi_drop.abs() < 0.1,
        "the integral term must hold the level against the load, and the level \
         moved {pi_drop:.4} m. A proportional loop with this gain would have moved \
         {:.4} m for the same {pi_travel:.4} of valve travel",
        pi_travel / 0.5
    );
    assert!(
        (pi_level_1 - 4.0).abs() < 0.1,
        "the integral term must return the level TO SETPOINT, not merely hold it \
         somewhere: it ended at {pi_level_1:.4} m against a setpoint of 4.0 m"
    );
    assert!(
        pi_drop.abs() * 5.0 < p_drop,
        "the pair does not separate: the PI loop's level moved {pi_drop:.4} m and \
         the P loop's {p_drop:.4} m for comparable valve travel"
    );

    // The control on the pair itself: two fixtures, two algorithms.
    let p_name = engine_from(&format!("{LEAK_PLANT}{P_LOOP}"))
        .snapshot()
        .controls[0]
        .algorithm
        .clone();
    let pi_name = engine_from(&format!("{LEAK_PLANT}{PI_LOOP}"))
        .snapshot()
        .controls[0]
        .algorithm
        .clone();
    assert_ne!(
        p_name, pi_name,
        "both halves of the pair loaded the same algorithm ('{p_name}'), so the \
         comparison above is between a run and itself"
    );
}

/// Gate 4's plant: built to SATURATE, which fork 6 says in advance is the only
/// way an anti-windup gate can mean anything.
///
/// The drain valve at FULL opening cannot pass the feed — `kv` 20 against the
/// feed valve's 20 at 0.8 open — so the level rises while the controller has
/// already asked for everything the actuator has. That is the state in which an
/// unclamped integral accumulates against a plant that cannot answer, and no
/// plant that can meet its setpoint ever enters it.
///
/// `initial_level_m` is the setpoint, so the run starts with zero error and the
/// saturation is produced by the plant rather than by the initial condition.
const SATURATING_PLANT: &str = r#"
[meta]
name = "level_control_windup"
description = "A tank whose drain valve at full opening cannot pass its feed."

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
opening = 0.8

[nodes.control_tank]
type = "tank"
area_m2 = 3.0
height_m = 10.0
initial_level_m = 4.0
temperature_c = 20.0

[nodes.drain_valve]
type = "valve"
kv = 20.0
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
algorithm = "pi"
mode = "auto"
setpoint_m = 4.0
gain_per_m = 0.3
integral_time_s = 300.0
initial_output = 0.2
"#;

/// **Gate 4.** Saturate the actuator, then remove the load, and watch what the
/// loop does with the time it spent asking for something the plant could not give.
///
/// **The reachability half is asserted first and is not decoration.** This repo
/// has twice shipped a counter that proved nothing because the branch it counted
/// was never reached, so the gate counts the ticks on which the output was pinned
/// at exactly 1.0 while the level was still above setpoint — the state in which
/// the anti-windup branch is the code that runs. Measured 779 of the first 2 000
/// ticks. If that count is zero, every number below is about a plant that never
/// saturated and the gate has no power, whatever it asserts.
///
/// **The load is removed by a PARTIAL cut, and that is a measured constraint
/// rather than a stylistic one.** Shutting the feed valve outright drives a branch
/// to zero flow in one tick, which stalls the hydraulic solver for reasons that
/// have nothing to do with control and are pinned by
/// `a_branch_shut_in_one_tick_stalls_the_solver_whoever_shuts_it`. Quartering it
/// removes more than enough load to expose windup and leaves the solver a problem
/// it can solve.
///
/// **The signature is the undershoot, and both bounds were measured on this plant
/// with the clamp removed** — the one mutation this slice ran early, because a
/// docstring claiming a gate catches something is a claim and not a hope. The
/// mutation is `if (0.0..=1.0).contains(&unclamped)` → `if true`, i.e. accumulate
/// while saturated; it was checked to compile, run, and restored from a single
/// pre-mutation snapshot:
///
/// | | with the clamp | without it |
/// |---|---|---|
/// | deepest level after the cut | **3.7042 m** | **2.7379 m** |
/// | level after 20 000 ticks | 3.9304 m | 4.6213 m |
/// | valve while undershooting | leaves 1.0 at ~6 000 ticks | pinned at 1.0, then slammed to 0.0 |
///
/// So the loop without anti-windup spends the surplus it accumulated by holding
/// the drain wide open a metre below setpoint, and then overshoots the other way.
/// Both assertions below fire on it.
#[test]
fn an_unclamped_integral_would_hold_the_valve_open_past_setpoint() {
    let mut engine = engine_from(SATURATING_PLANT);

    // Phase 1 — the plant cannot meet its setpoint, so the loop saturates.
    let mut saturated_ticks = 0_u32;
    for t in 0..SATURATE_TICKS {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        if output_now(&engine) == 1.0 && level_now(&engine) > 4.0 {
            saturated_ticks += 1;
        }
    }
    assert!(
        saturated_ticks > 100,
        "the plant must actually saturate its actuator, and the output was pinned \
         at 1.0 above setpoint on only {saturated_ticks} of {SATURATE_TICKS} ticks. \
         An anti-windup gate on a plant that never saturates is the vacuous counter \
         this repo has shipped twice"
    );

    // Phase 2 — the load is removed. The feed valve is not this loop's actuator,
    // so a hand write on it is admissible with the loop still in AUTO.
    let feed = engine
        .graph
        .find_node("feed_valve")
        .expect("the plant has a feed valve");
    engine
        .apply(Command::SetValveOpening {
            node: feed,
            opening: 0.2,
        })
        .expect("the feed valve is nobody's actuator");

    let mut deepest = level_now(&engine);
    for t in 0..RECOVER_TICKS {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("recovery tick {t}: {e}"));
        deepest = deepest.min(level_now(&engine));
    }

    assert!(
        deepest > 3.5,
        "the level undershot to {deepest:.4} m. An integral term that kept \
         accumulating while the valve was pinned at 1.0 has a surplus to spend, and \
         spends it by holding the drain open well below setpoint — measured 2.7379 m \
         with the clamp removed, against 3.7042 m with it"
    );
    let settled = level_now(&engine);
    assert!(
        (settled - 4.0).abs() < 0.25,
        "the loop must come back to setpoint after the load is removed, and ended \
         at {settled:.4} m. With the clamp removed the same run ends at 4.6213 m, on \
         the far side of an overshoot the windup paid for"
    );
}

/// Ticks of saturation before the load is removed. Long enough that an unclamped
/// integral has something to accumulate (measured: 779 pinned ticks) and short
/// enough that the level stays well below the tank's 10 m roof — a mass clamp
/// would make this gate a test of the clamp instead.
const SATURATE_TICKS: u64 = 2_000;

/// Ticks after the cut. The undershoot bottoms out around 8 000 (clamped) and
/// 12 000 (unclamped), so the run has to outlast both to compare their depths.
const RECOVER_TICKS: u64 = 20_000;

/// **MANUAL→AUTO is bumpless, and the counterfactual is in the same test.**
///
/// A loop that has been sitting in MANUAL while a human moved the valve holds
/// memory from whenever it last ran. Taking over would step the actuator to
/// whatever that stale memory asks for — which is exactly the "a command that
/// appears to work and does not" failure fork 4 refuses in the other direction.
/// So `SetControllerMode` seeds the algorithm from the position the actuator is
/// actually at, by the same back-calculation the anti-windup clamp performs.
///
/// **The tolerance is derived, not fitted.** Seeding stores `b = u − K·e` and the
/// next update returns `K·e + b`; the error is bit-identical between the two,
/// because commands are applied between ticks and the seed reads the same state
/// the next tick's control pass will. So the difference from `u` is at most the
/// two roundings in that subtract-then-add, a few ULP of a quantity in `[0, 1]`.
/// `1e-15` is that bound with room; the measured deviation on this fixture is
/// exactly zero, which Sterbenz's lemma predicts for these magnitudes.
///
/// **The same transfer on a proportional loop steps the valve by 0.1832**, and
/// that half is asserted too — not because it is a defect but because it is what
/// makes the PI half mean something. `u = K·e` has no memory to seed, so it
/// returns what the error says and ignores where the human left the valve
/// entirely. A transfer test that passed for both controllers would be measuring
/// the plant.
#[test]
fn a_loop_taking_over_from_a_human_does_not_step_the_valve() {
    /// Where the human leaves the valve before handing it back. Deliberately not
    /// the file's declared 0.20 and not what either controller would ask for, so
    /// "the valve did not move" cannot be satisfied by a coincidence.
    const HANDOVER: f64 = 0.31;

    let mut deltas = Vec::new();
    for loop_table in [PI_LOOP, P_LOOP] {
        let parked =
            format!("{LEAK_PLANT}{loop_table}").replace(r#"mode = "auto""#, r#"mode = "manual""#);
        let mut engine = engine_from(&parked);
        run(&mut engine, 3_000);

        let valve = engine
            .graph
            .find_node("drain_valve")
            .expect("the plant has a drain valve");
        engine
            .apply(Command::SetValveOpening {
                node: valve,
                opening: HANDOVER,
            })
            .expect("in MANUAL a human drives the valve");
        engine.tick().expect("a tick in MANUAL");
        assert_eq!(
            output_now(&engine),
            HANDOVER,
            "in MANUAL the faceplate TRACKS the actuator, and reported something else"
        );

        engine
            .apply(Command::SetControllerMode {
                loop_id: LoopId(0),
                mode: ControlMode::Auto,
            })
            .expect("a declared loop can be put in AUTO");
        assert_eq!(
            output_now(&engine),
            HANDOVER,
            "the faceplate must report what the loop will hold at the moment of \
             transfer, not the output it last computed before it was parked"
        );

        engine.tick().expect("the first tick in AUTO");
        deltas.push(output_now(&engine) - HANDOVER);
    }

    let (pi_step, p_step) = (deltas[0], deltas[1]);
    assert!(
        pi_step.abs() < 1.0e-15,
        "MANUAL→AUTO moved the PI loop's valve by {pi_step:e}, and the bound is the \
         rounding in `b = u − K·e` followed by `K·e + b` — a few ULP. A larger step \
         means the loop was seeded against a different error than it spent, most \
         likely the one-tick-old `last_measurement` instead of a fresh read"
    );
    assert!(
        p_step.abs() > 0.05,
        "the same transfer on a memoryless controller stepped the valve by only \
         {p_step:e}, so this fixture does not discriminate: `u = K·e` cannot be \
         seeded, and if it too holds the handover position then the PI assertion \
         above is being satisfied by the plant rather than by the back-calculation"
    );
}

/// Rule 3 on the project's first stateful seam: the same PI scenario twice ⇒
/// bit-identical snapshots.
///
/// Every seam before this one is a pure function of its arguments, so determinism
/// followed from the graph and the solver alone. A `Box<dyn Controller>` carries a
/// number across ticks, and M8.0 has already paid once for the belief that a
/// carried-over number is "a path, not an answer". The comparison is on the
/// serialized bytes for `reruns_are_bit_identical`'s reason: identical f64 bits
/// render to identical text, so one perturbed low bit fails here.
#[test]
fn a_loop_with_memory_reruns_bit_identically() {
    fn capture() -> Vec<Vec<u8>> {
        let mut engine = engine_from(SATURATING_PLANT);
        (0..500)
            .map(|t| {
                engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
                serde_json::to_vec(&engine.snapshot()).expect("a snapshot serializes")
            })
            .collect()
    }

    let first = capture();
    let second = capture();
    assert_eq!(first.len(), 500, "a run must capture one snapshot per tick");
    for (index, (a, b)) in first.iter().zip(&second).enumerate() {
        assert!(
            a == b,
            "two runs of one PI scenario diverged at tick {}:\n  first:  {}\n  second: {}",
            index + 1,
            String::from_utf8_lossy(a),
            String::from_utf8_lossy(b)
        );
    }
}
