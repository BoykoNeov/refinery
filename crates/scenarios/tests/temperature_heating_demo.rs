//! M18.1: `scenarios/tank_temperature_heating.toml`, as shipped.
//!
//! The seam's own gates live in `reverse_action_reference.rs` and run on
//! fixtures. This file pins what only the demo can show: the first plant in
//! `scenarios/` whose loop is REVERSE acting — a furnace holding a tank — run as
//! a reader is invited to run it. It is the cooler demo mirrored, and three of
//! its claims are that mirror's numbers:
//!
//! 1. **It starts without stepping its furnace**, although the tank starts 20 K
//!    BELOW setpoint — the design input that makes a wrongly-signed seed visible
//!    (docs/DESIGN.md §22, gate 2).
//! 2. **It holds 60 °C, and parked it holds 48.30 °C** (gate 3).
//! 3. **Its anti-windup arm is reachable by a command**, and the release after it
//!    lands where the SIGNED back-calculation says (gate 4).
//! 4. **It says `reverse` on the wire** (gate 6's wired half).
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{ControlAction, ControlledValue, LoopId, NodeKind};
use refinery_core::snapshot::Command;
use refinery_core::units::Kelvin;
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/tank_temperature_heating.toml");

/// The file's declared numbers, named once so no assertion can drift away from
/// the plant it is about.
const SETPOINT_C: f64 = 60.0;
const DECLARED_TANK_C: f64 = 40.0;
const DECLARED_OUTPUT: f64 = 0.25;
const GAIN_PER_K: f64 = 0.1;
const MAX_DUTY_W: f64 = 2.0e6;

/// Where the shipped file settles, measured over 20 000 ticks before any
/// assertion here was written: u = 0.600948, i.e. 1.202 MW.
const SETTLED_OUTPUT: f64 = 0.600948;

/// Where the MANUAL twin settles at the declared 0.5 MW, measured the same way
/// (48.2963 °C at tick 6 000).
const MANUAL_SETTLED_C: f64 = 48.2963;

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
    match engine.snapshot().controls[0].measurement {
        ControlledValue::Temperature { k } => k.value() - 273.15,
        other => panic!("the demo's loop measures a temperature, not {other:?}"),
    }
}

fn output(engine: &Engine) -> f64 {
    engine.snapshot().controls[0].output
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
        .unwrap_or_else(|e| panic!("{c} °C is a legal setpoint on a tank: {e}"));
}

// ------------------------------------------ claim 1: startup, and the seed's sign

/// **Gate 2. The seed is signed.** The loop starts where the file says the
/// furnace is, with the tank 20 K below setpoint, and its first update returns
/// the seeded 0.25 exactly.
///
/// The memory is back-calculated at load as `b = u − K·e`. With the REVERSE error
/// `e = setpoint − measurement = +20 K` that is `0.25 − 2.0`, and the first update
/// `K·e + b` returns 0.25. Seeded with the direct error (`e = −20 K`) it would be
/// `b = 2.25`, and the first update `2.0 + 2.25` — a step to full firing on
/// tick 1. At zero starting error the two are indistinguishable, which is why the
/// file starts its tank at 40 °C.
#[test]
fn the_demo_starts_twenty_kelvin_cold_without_stepping_its_furnace() {
    let mut engine = build(DEMO);
    assert_eq!(output(&engine), DECLARED_OUTPUT, "the faceplate at load");
    assert!(
        (measured_c(&engine) - DECLARED_TANK_C).abs() < 1.0e-12,
        "the loop holds the tank's declared 40 °C before the first tick"
    );
    engine.tick().expect("tick 1");
    assert_eq!(
        output(&engine),
        DECLARED_OUTPUT,
        "the first update must return the seeded position exactly, 20 K below \
         setpoint — anything else is a seed taken against the other sign"
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
        "the shipped tuning stays off both clamps — the warm-up from 40 °C peaks at \
         ~0.809, the cooler demo's number — and ranged over [{low}, {high}]"
    );
}

// ----------------------------------------------- claim 2: holds versus parked

/// **Gate 3. The loop holds 60 °C; the same file in MANUAL settles at its own
/// duty's temperature, 11.7 K below.**
///
/// The tolerance is the cooler demo's, for the same reason: at tick 6 000 the
/// loop is 3.2e-5 K from its setpoint on a still-closing integral, so 1e-3 K is
/// thirty times that and eleven thousand times smaller than the separation from
/// the manual twin. The faceplate half is what catches a furnace mapped as
/// `(1 − u)·max`: its output would read 0.4 against 1.2 MW of firing.
#[test]
fn the_demo_holds_sixty_degrees_where_the_parked_loop_settles_near_forty_eight() {
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
        "the furnace must sit interior at the measured {SETTLED_OUTPUT}; it reads {u}"
    );
    assert!(
        (heater_duty_w(&auto) - u * MAX_DUTY_W).abs() <= 1e-9 * MAX_DUTY_W,
        "the faceplate reads the real firing fraction: the duty written is the output \
         times the loop's 2 MW authority, not (1 − u) of it"
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
        "in MANUAL the faceplate tracks the furnace's real duty as a fraction of the \
         loop's range: 0.5 MW / 2 MW"
    );
}

/// The wire form, on the demo's own bytes. A Rust match on
/// `ControlAction::Reverse` passes under any serde spelling, which is how M10.1's
/// escape happened; the reference file asserts the other half, that a DIRECT
/// loop publishes no `action` key at all.
#[test]
fn the_demo_reports_reverse_action_on_the_wire() {
    let engine = build(DEMO);
    assert_eq!(engine.snapshot().controls[0].action, ControlAction::Reverse);
    let json = serde_json::to_string(&engine.snapshot()).expect("a snapshot serializes");
    assert!(
        json.contains(
            r#""mode":"auto","action":"reverse","setpoint":{"variable":"temperature","k":333.15}"#
        ),
        "the loop must publish its action between its mode and its setpoint: {}",
        &json[json.find("\"controls\"").unwrap_or(0)..]
    );
}

// ------------------------------------------ claim 3: the anti-windup arm

/// **Gate 4. The anti-windup arm on the reverse side, reached by a command, and
/// released where the SIGNED back-calculation says.**
///
/// 75 °C is above the 73.28 °C the inflow reaches at the full 2 MW, so the loop
/// pins at `u = 1`. While pinned, the clamp branch sets its memory to
/// `b = 1 − K·e` with `e = setpoint − measurement` — the same signed error the
/// proportional term used. Stepping back to 72 °C, just below where the tank sits,
/// then gives `u = K·(72 − m₂) + 1 − K·(75 − m₁)`, about 0.70: interior.
///
/// **The release is asserted against that number, not merely below 1**, because
/// "releases" alone cannot see the sign. Stepping back all the way to 60 °C, as
/// the cooler demo does, drives both a right and a wrongly-signed memory to 0 on
/// the first tick. A clamp branch back-calculating with the UNSIGNED error would
/// hold `b = 1 + K·(75 − m₁)` ≈ 1.17 and stay pinned at 1 on the way back to 72.
#[test]
fn a_saturated_furnace_pins_at_full_firing_and_releases_where_the_signed_memory_says() {
    let mut engine = build(DEMO);
    run(&mut engine, 6_000);
    set_setpoint_c(&mut engine, 75.0);
    // Eight of the tank's ~1 000 s time constants, as in the cooler demo, so the
    // band below measures the ceiling rather than the approach to it.
    run(&mut engine, 8_000);
    // **Pinned is not the same as "exactly 1 on every tick", and the first
    // draft of this gate said it was.** As the tank creeps toward its ceiling
    // the error shrinks, `K·e + b` dips a hair below 1, the integral accumulates
    // for one tick, and the next update is on the clamp again — measured at tick
    // 14 000, 0.99999943. The cooler demo has the same mechanism and happens to
    // sample a clamped tick. So the gate tolerates that hair over a window, then
    // steps back on a tick that IS clamped, because the hand-computed memory
    // below is only valid right after one.
    let mut lowest = f64::INFINITY;
    for t in 1..=200_u64 {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("saturated tick {t}: {e}"));
        lowest = lowest.min(output(&engine));
    }
    assert!(
        lowest > 1.0 - 1.0e-5,
        "an unreachable target pins the furnace to within a hair of full firing; \
         over 200 ticks the output fell to {lowest}"
    );
    let mut waited = 0_u32;
    while output(&engine) != 1.0 {
        engine.tick().expect("waiting for a clamped tick");
        waited += 1;
        assert!(waited < 100, "no clamped tick in 100");
    }
    assert_eq!(
        heater_duty_w(&engine),
        MAX_DUTY_W,
        "on a clamped tick the furnace fires at the loop's full 2 MW"
    );
    let ceiling = measured_c(&engine);
    assert!(
        ceiling < 75.0 && (ceiling - 73.28).abs() < 0.05,
        "the tank settles on the hottest inflow full firing can make, ~73.28 °C; it \
         reads {ceiling}"
    );

    // The measurement the last clamped update acted on, from the published
    // snapshot — the reader-side reconstruction the single error owner promises.
    let pinned_k = ceiling + 273.15;
    set_setpoint_c(&mut engine, 72.0);
    engine.tick().expect("first tick back");
    let released_k = measured_c(&engine) + 273.15;
    let memory = 1.0 - GAIN_PER_K * ((75.0 + 273.15) - pinned_k);
    let expected = GAIN_PER_K * ((72.0 + 273.15) - released_k) + memory;
    assert!(
        expected > 0.5 && expected < 0.9,
        "the gate's own arithmetic must land interior, and gave {expected}"
    );
    let released = output(&engine);
    assert!(
        (released - expected).abs() < 1.0e-9,
        "the first tick back must leave the clamp at K·e + b with b = 1 − K·e from \
         the last clamped tick, signed as the loop is: expected {expected}, read \
         {released}. Pinned at 1 means the clamp branch back-calculated with the \
         unsigned error"
    );
    set_setpoint_c(&mut engine, SETPOINT_C);
    run(&mut engine, 6_000);
    assert!(
        (measured_c(&engine) - SETPOINT_C).abs() < 1.0e-2,
        "and the loop comes back to 60 °C"
    );
}
