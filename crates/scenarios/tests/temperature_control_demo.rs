//! M17.1: `scenarios/tank_temperature_control.toml`, as shipped.
//!
//! The seam's own gates live in `temperature_control_reference.rs` and run on
//! fixtures built to expose one behaviour each. This file pins what only the demo
//! can show: the first plant in `scenarios/` whose loop regulates a temperature
//! and whose actuator is not a valve, run as a reader is invited to run it.
//!
//! Four claims the file's header makes, each pinned so that a change to the
//! plant's sizing fails a test rather than quietly making the comment wrong:
//!
//! 1. **The measurement is real at tick 0 and one Euler step behind the graph
//!    after it** (docs/DESIGN.md §21, gates 1 and 2) — the finding that licensed
//!    the milestone, on the plant rather than on a probe.
//! 2. **It holds, and parked it holds the wrong number** — 71.71 °C at the
//!    declared 0.5 MW against the loop's 60 °C.
//! 3. **It starts without stepping its own actuator** and never reaches either
//!    clamp on its own.
//! 4. **Its anti-windup arm is reachable by a command** from the shipped file: a
//!    step to 45 °C sits below the coldest inflow full duty can make.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{ControlledValue, LoopId, NodeKind};
use refinery_core::snapshot::Command;
use refinery_core::units::Kelvin;
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/tank_temperature_control.toml");

/// The file's declared numbers, named once so no assertion can drift away from
/// the plant it is about.
const SETPOINT_C: f64 = 60.0;
const DECLARED_TANK_C: f64 = 80.0;
const DECLARED_OUTPUT: f64 = 0.25;
const MAX_DUTY_W: f64 = 2.0e6;

/// Where the shipped file settles, measured over 20 000 ticks before any
/// assertion here was written: u = 0.601092, i.e. 1.202 MW.
const SETTLED_OUTPUT: f64 = 0.601092;

/// Where the MANUAL twin settles at the declared 0.5 MW, measured the same way.
const MANUAL_SETTLED_C: f64 = 71.7085;

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("a shipped scenario must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("a shipped scenario must build: {e}"))
}

fn run(engine: &mut Engine, ticks: u64) {
    for t in 1..=ticks {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
    }
}

/// The temperature the loop ACTED ON, in °C to match the file.
fn measured_c(engine: &Engine) -> f64 {
    match engine.snapshot().controls[0]
        .measurement
        .expect("a stored quantity is measured from load")
    {
        ControlledValue::Temperature { k } => k.value() - 273.15,
        other => panic!("the demo's loop measures a temperature, not {other:?}"),
    }
}

fn output(engine: &Engine) -> f64 {
    engine.snapshot().controls[0].output
}

fn chiller_duty_w(engine: &Engine) -> f64 {
    let id = engine
        .graph
        .find_node("chiller")
        .expect("the demo has a chiller");
    match engine.graph.node(id).kind {
        NodeKind::Cooler { duty } => duty.value(),
        ref other => panic!("chiller is a cooler, not {other:?}"),
    }
}

/// The tank's two temperatures, off one snapshot: the one STORED on the graph
/// (published through `NodeSnapshot::kind`) and the one the snapshot reports as
/// `temperature_k`, which is resolved by the tick.
fn tank_temperatures(engine: &Engine) -> (f64, f64) {
    let id = engine
        .graph
        .find_node("hold_tank")
        .expect("the demo has a tank");
    let snapshot = engine.snapshot();
    let node = snapshot
        .nodes
        .iter()
        .find(|n| n.id == id)
        .expect("the tank is in the snapshot");
    match &node.kind {
        NodeKind::Tank(t) => (t.temperature.value(), node.temperature_k),
        other => panic!("hold_tank is a tank, not {other:?}"),
    }
}

// --------------------------------------------- claim 1: tick 0 and the lag

/// **Gate 1 on the shipped plant. The loop has the declared temperature before
/// the first tick, while the snapshot's own temperature for that tank is NaN.**
///
/// The equality half is near a round trip (`c_to_k` in, `.temperature` out); the
/// NaN half is the discriminating one — the same tank's `temperature_k` comes
/// from `NodeStates`, which is empty before a tick has run. A loop that read the
/// snapshot's path instead of the graph's would have nothing to act on here.
#[test]
fn the_loop_has_the_declared_temperature_before_the_first_tick() {
    let engine = build(DEMO);
    let expected = DECLARED_TANK_C + 273.15;
    match engine.snapshot().controls[0]
        .measurement
        .expect("a stored quantity is measured from load")
    {
        ControlledValue::Temperature { k } => assert_eq!(
            k.value(),
            expected,
            "before any tick the loop must hold the tank's declared temperature, bit \
             for bit — it is the graph's stored value, set by the same `+ 273.15`"
        ),
        other => panic!("the demo's loop measures a temperature, not {other:?}"),
    }
    let (stored, published) = tank_temperatures(&engine);
    assert_eq!(stored, expected, "the graph holds the declaration at load");
    assert!(
        published.is_nan(),
        "the tank's snapshot `temperature_k` comes from the tick and no tick has run, \
         so it must be NaN — it read {published}. If it becomes a number, this gate \
         has stopped discriminating between the two paths"
    );
}

/// **Gate 2. The one-step identity, exact, over the whole demo run.**
///
/// A snapshot's `temperature_k` for the tank at tick `n` equals the graph's
/// stored temperature at tick `n − 1`, bit for bit — the tick resolves node
/// temperatures before the unit dynamics integrate the tank, so the published
/// number is the one the tank STARTED the tick with. And the loop's measurement
/// at tick `n` is the same number, because the loop runs at the top of the tick
/// on the state the previous tick left.
///
/// **The direction of the lag is asserted, not just its existence**: at the
/// settled end the reversed identity (`temperature_k` at `n` against the graph at
/// `n`) also holds to many digits, and fails only in the transient. So the
/// reversed form is shown to FAIL somewhere, or a gate stated backwards would
/// pass on the tail of the run.
#[test]
fn the_published_temperature_is_the_stored_one_one_tick_late_exactly() {
    let mut engine = build(DEMO);
    let (mut previous_stored, _) = tank_temperatures(&engine);
    let mut reversed_fails = 0_u32;
    for t in 1..=6_000_u64 {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        let (stored, published) = tank_temperatures(&engine);
        assert_eq!(
            published.to_bits(),
            previous_stored.to_bits(),
            "tick {t}: the snapshot's temperature must be the graph's from the \
             previous tick, bit for bit — read {published} against {previous_stored}"
        );
        let measured = measured_c(&engine) + 273.15;
        let acted_on = match engine.snapshot().controls[0]
            .measurement
            .expect("a stored quantity is measured from load")
        {
            ControlledValue::Temperature { k } => k.value(),
            _ => unreachable!("checked by measured_c"),
        };
        assert_eq!(
            acted_on.to_bits(),
            previous_stored.to_bits(),
            "tick {t}: the loop acts on the temperature the previous tick left, \
             exactly (read {measured} K)"
        );
        if published.to_bits() != stored.to_bits() {
            reversed_fails += 1;
        }
        previous_stored = stored;
    }
    assert!(
        reversed_fails > 100,
        "the reversed identity must fail during the transient, or this gate cannot \
         tell which way the lag runs — it failed on only {reversed_fails} ticks"
    );
}

// ----------------------------------------------- claim 2: holds versus parked

/// **Gate 3. The loop holds 60 °C; the same file in MANUAL settles at its own
/// duty's temperature, 11.7 K away.**
///
/// The tolerance is the run's own: at tick 6 000 the loop is 3.2e-5 K from its
/// setpoint on a still-closing integral (it reads 60.000000 at tick 20 000), so
/// 1e-3 K is thirty times that and eleven thousand times smaller than the
/// separation from the manual twin.
#[test]
fn the_demo_holds_sixty_degrees_where_the_parked_loop_settles_near_seventy_two() {
    let mut auto = build(DEMO);
    run(&mut auto, 6_000);
    let held = measured_c(&auto);
    assert!(
        (held - SETPOINT_C).abs() < 1.0e-3,
        "the loop must hold the tank at {SETPOINT_C} °C by tick 6000; it reads {held}"
    );
    let u = output(&auto);
    assert!(
        (u - SETTLED_OUTPUT).abs() < 1.0e-4 && u > 0.0 && u < 1.0,
        "the cooler must sit interior at the measured {SETTLED_OUTPUT}; it reads {u}"
    );
    assert!(
        (chiller_duty_w(&auto) - u * MAX_DUTY_W).abs() <= 1e-9 * MAX_DUTY_W,
        "the duty written is the output times the loop's 2 MW authority"
    );

    let mut manual = build(&DEMO.replace(r#"mode = "auto""#, r#"mode = "manual""#));
    run(&mut manual, 6_000);
    let parked = measured_c(&manual);
    assert!(
        (parked - MANUAL_SETTLED_C).abs() < 1.0e-3,
        "the manual twin must settle at the declared 0.5 MW's temperature, \
         {MANUAL_SETTLED_C} °C; it reads {parked}"
    );
    assert_eq!(
        output(&manual),
        DECLARED_OUTPUT,
        "in MANUAL the faceplate tracks the cooler's real duty as a fraction of the \
         loop's range: 0.5 MW / 2 MW"
    );
}

/// The wire form, on the demo's own bytes (gate 5's wired half; the fixture half
/// is in the reference file). A Rust match on `ControlledValue::Temperature`
/// passes under any serde tag, which is how M10.1's escape happened.
#[test]
fn the_demo_reports_its_variable_on_the_wire_as_temperature() {
    let json = serde_json::to_string(&build(DEMO).snapshot()).expect("a snapshot serializes");
    assert!(
        json.contains(r#""setpoint":{"variable":"temperature","k":333.15}"#),
        "the setpoint must travel as a tagged kelvin number: {}",
        &json[json.find("\"controls\"").unwrap_or(0)..]
    );
    assert!(
        json.contains(r#""measurement":{"variable":"temperature","k":353.15}"#),
        "the measurement must travel the same way"
    );
}

// ------------------------------------------------ claim 3: startup and range

/// The loop starts where the file says the cooler is, and its first move is the
/// controller's rather than a step to its own memory.
#[test]
fn the_demo_starts_without_stepping_its_own_actuator_and_never_clamps() {
    let mut engine = build(DEMO);
    assert_eq!(output(&engine), DECLARED_OUTPUT, "the faceplate at load");
    engine.tick().expect("tick 1");
    assert_eq!(
        output(&engine),
        DECLARED_OUTPUT,
        "the first update returns the seeded position exactly (the memory is \
         back-calculated from it), so the cooler does not step at startup"
    );
    let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
    for t in 2..=6_000_u64 {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        let u = output(&engine);
        low = low.min(u);
        high = high.max(u);
    }
    assert!(
        low > 0.0 && high < 1.0,
        "the shipped tuning is chosen to stay off both clamps — the pull-down from \
         80 °C peaks at ~0.809 — and ranged over [{low}, {high}]"
    );
}

// ------------------------------------------ claim 4: the anti-windup arm

/// **The anti-windup arm, reached by a command from the shipped file.**
///
/// 45 °C is below the 46.73 °C the inflow reaches at the full 2 MW, so the loop
/// can never get there and pins at `u = 1`. What anti-windup buys is the RETURN:
/// the memory is clamped while saturated, so stepping back to 60 °C unpins the
/// cooler on the first tick the error changes sign instead of after an
/// integral's worth of wound-up error has been paid back.
#[test]
fn a_saturated_cooler_pins_at_full_duty_and_releases_on_the_way_back() {
    let mut engine = build(DEMO);
    run(&mut engine, 6_000);
    engine
        .apply(Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Temperature {
                k: Kelvin(45.0 + 273.15),
            },
        })
        .expect("45 °C is a legal setpoint on a tank");
    // Eight of the tank's ~1 000 s time constants, so the approach to the floor
    // is spent and the band below measures the floor rather than the transient.
    run(&mut engine, 8_000);
    assert_eq!(
        output(&engine),
        1.0,
        "an unreachable target pins the cooler"
    );
    assert_eq!(
        chiller_duty_w(&engine),
        MAX_DUTY_W,
        "full duty is the loop's range"
    );
    let floor = measured_c(&engine);
    assert!(
        floor > 45.0 && (floor - 46.73).abs() < 0.05,
        "the tank settles on the coldest inflow full duty can make, ~46.73 °C; it \
         reads {floor}"
    );

    engine
        .apply(Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Temperature {
                k: Kelvin(SETPOINT_C + 273.15),
            },
        })
        .expect("back to 60 °C");
    engine.tick().expect("first tick back");
    let released = output(&engine);
    assert!(
        released < 1.0,
        "with the memory held at the clamp, the first tick with the error reversed \
         must come off full duty — it read {released}. A wound-up integral would \
         keep it at 1 for thousands of ticks"
    );
    run(&mut engine, 6_000);
    assert!(
        (measured_c(&engine) - SETPOINT_C).abs() < 1.0e-2,
        "and the loop comes back to 60 °C"
    );
}
