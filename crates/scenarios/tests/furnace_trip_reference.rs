//! M32: a trip that cuts a furnace's fuel (docs/DESIGN.md §35, docs/DEFERRED.md
//! E14's furnace clause).
//!
//! - **gate 1**, the demo (`tank_overheat_trip.toml`) on both fidelities: armed
//!   through tick 1 250 and bit for bit its own untripped twin until then;
//!   `Tripped { at_tick: 1251 }` from 1 251 on; the furnace at zero duty from that
//!   tick's own snapshot; and its outlet EXACTLY its inlet stream's temperature.
//! - **gate 2**, the latch on the demo: the condition clears inside the tripping
//!   tick, the trip stays tripped to tick 6 000, the tank cools toward its feed,
//!   and the twin does not.
//! - **gate 3**, the tick order with a loop on the furnace: on the tripping tick
//!   the loop already reports MANUAL and a faceplate of exactly zero.
//! - **gate 4**, every writer refused while latched, the cut itself admitted, and
//!   the hand-back: reset relights nothing, a human fires the furnace, AUTO takes
//!   over without a bump.
//! - **gate 5**, the reset refused inside its condition, on a plant loaded there.
//! - **gate 6**, the trip opens a cascade: the inner loop yields, the primary
//!   writes nothing from the tripping tick, and the cascade closes again after a
//!   reset and a relight.
//!
//! The load-time refusals are cases in `trip_reference.rs`'s sweep, beside the
//! pump and valve ones, so the sweep stays the single owner of "every trip the
//! loader cannot honour".

use refinery_core::graph::{ControlMode, ControlledValue, NodeKind, TripId, TripState};
use refinery_core::snapshot::{Command, ControlSnapshot};
use refinery_core::units::{Kelvin, Watt};
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/tank_overheat_trip.toml");
const TANK_LOOP: &str = include_str!("../../../scenarios/tank_temperature_heating.toml");
const CASCADE: &str = include_str!("../../../scenarios/furnace_cascade_control.toml");

/// The demo's own trip block, verbatim with its comment, so removing it must land.
const DEMO_TRIP: &str = r#"# Fires AT OR ABOVE 75 °C (docs/DESIGN.md §26 fork 6) and cuts the furnace's
# fuel. A furnace action takes no `position`: its only safe state is zero duty.
[[trips]]
name = "tank_high_temperature"
measurement = { node = "hold_tank", variable = "temperature" }
direction = "high"
limit_c = 75.0
actions = [{ furnace = "heater" }]
"#;

/// The tick whose trip pass fires the demo's trip, measured on the engine on
/// both fidelities before it was written here: the tank ends tick 1 250 at
/// 75.0007 °C.
const DEMO_TRIP_TICK: u64 = 1251;
const DEMO_LIMIT_C: f64 = 75.0;

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the fixture must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("the fixture must build: {e}"))
}

fn with_solver(src: &str, solver: &str) -> String {
    swap(src, r#"flow = "newton""#, &format!(r#"flow = "{solver}""#))
}

fn swap(src: &str, from: &str, to: &str) -> String {
    assert!(
        src.contains(from),
        "the fixture's substitution must land: `{from}` is not in the plant"
    );
    src.replace(from, to)
}

fn tick(engine: &mut Engine) {
    let t = engine.snapshot().tick + 1;
    engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
}

fn duty_w(engine: &Engine) -> f64 {
    let id = engine.graph.find_node("heater").expect("a heater");
    match engine.graph.node(id).kind {
        NodeKind::Furnace { duty } => duty.value(),
        ref other => panic!("'heater' is a furnace, not {other:?}"),
    }
}

/// The tank's stored temperature, °C: what the trip compares.
fn tank_c(engine: &Engine) -> f64 {
    let id = engine.graph.find_node("hold_tank").expect("a tank");
    match &engine.graph.node(id).kind {
        NodeKind::Tank(t) => t.temperature.value() - 273.15,
        other => panic!("'hold_tank' is a tank, not {other:?}"),
    }
}

fn state(engine: &Engine) -> TripState {
    engine.snapshot().trips[0].state
}

fn faceplate(engine: &Engine, name: &str) -> ControlSnapshot {
    engine
        .snapshot()
        .controls
        .into_iter()
        .find(|c| c.name == name)
        .unwrap_or_else(|| panic!("no loop called '{name}'"))
}

fn set_duty(engine: &mut Engine, watts: f64) -> Result<(), String> {
    let node = engine.graph.find_node("heater").expect("a heater");
    engine
        .apply(Command::SetFurnaceDuty {
            node,
            duty: Watt(watts),
        })
        .map_err(|e| e.to_string())
}

fn set_mode(engine: &mut Engine, name: &str, mode: ControlMode) -> Result<(), String> {
    let loop_id = faceplate(engine, name).id;
    engine
        .apply(Command::SetControllerMode { loop_id, mode })
        .map_err(|e| e.to_string())
}

fn set_setpoint_c(engine: &mut Engine, name: &str, c: f64) {
    let loop_id = faceplate(engine, name).id;
    engine
        .apply(Command::SetSetpoint {
            loop_id,
            value: ControlledValue::Temperature {
                k: Kelvin(c + 273.15),
            },
        })
        .unwrap_or_else(|e| panic!("'{name}' to {c} °C: {e}"));
}

fn reset(engine: &mut Engine) -> Result<(), String> {
    engine
        .apply(Command::ResetTrip { trip_id: TripId(0) })
        .map_err(|e| e.to_string())
}

fn expect_refused(result: Result<(), String>, what: &str, says: &str) {
    match result {
        Ok(()) => panic!("{what} should have been refused"),
        Err(e) => assert!(
            e.contains(says),
            "{what} was refused for another reason: {e}"
        ),
    }
}

/// Tick until `done` holds, at most `budget` ticks; returns the tick it held at.
fn until(engine: &mut Engine, budget: u64, what: &str, done: impl Fn(&Engine) -> bool) -> u64 {
    for _ in 0..budget {
        tick(engine);
        if done(engine) {
            return engine.snapshot().tick;
        }
    }
    panic!("{what} did not happen within {budget} ticks");
}

// ------------------------------------------------------------- gates 1 and 2

/// **Gates 1 and 2, on both fidelities.** The fuel cut lands on the measured
/// tick, the furnace passes its stream through unheated from that tick's own
/// snapshot, the trip latches all the way down, and the untripped twin shows
/// what it prevented.
#[test]
fn the_demo_cuts_the_fuel_on_its_tick_and_the_latch_holds_it_out() {
    for solver in ["newton", "simple"] {
        let mut demo = build(&with_solver(DEMO, solver));
        let mut twin = build(&with_solver(&swap(DEMO, DEMO_TRIP, ""), solver));
        assert!(twin.snapshot().trips.is_empty(), "the twin has no trip");

        for t in 1..=6000u64 {
            tick(&mut demo);
            tick(&mut twin);
            let snapshot = demo.snapshot();
            let heater = snapshot
                .nodes
                .iter()
                .find(|n| n.name == "heater")
                .expect("a heater");
            let feed = snapshot
                .edges
                .iter()
                .find(|e| e.name == "feed_line")
                .expect("a feed line");
            if t < DEMO_TRIP_TICK {
                assert_eq!(
                    snapshot.trips[0].state,
                    TripState::Armed,
                    "{solver}, tick {t}"
                );
                // An armed trip is inert: the plant is its twin to the bit.
                let twin_snapshot = twin.snapshot();
                assert_eq!(
                    serde_json::to_string(&snapshot.nodes).unwrap(),
                    serde_json::to_string(&twin_snapshot.nodes).unwrap(),
                    "{solver}, tick {t}: an armed trip moved the plant"
                );
                assert_eq!(
                    serde_json::to_string(&snapshot.edges).unwrap(),
                    serde_json::to_string(&twin_snapshot.edges).unwrap(),
                    "{solver}, tick {t}: an armed trip moved the plant"
                );
            } else {
                assert_eq!(
                    snapshot.trips[0].state,
                    TripState::Tripped {
                        at_tick: DEMO_TRIP_TICK
                    },
                    "{solver}, tick {t}: tripped on its tick, and latched"
                );
                assert_eq!(duty_w(&demo), 0.0, "{solver}, tick {t}: the fuel is cut");
                // A furnace at zero duty passes its stream through: its outlet is
                // its inlet exactly. (`heated_line` reads 0.0008 K warmer, its own
                // friction, which is why the furnace NODE is compared.)
                assert_eq!(
                    heater.temperature_k,
                    feed.stream.temperature.value(),
                    "{solver}, tick {t}: a cut furnace adds no heat"
                );
            }
            if t == DEMO_TRIP_TICK - 1 {
                assert!(
                    tank_c(&demo) >= DEMO_LIMIT_C,
                    "{solver}: the tank crosses the limit at the end of tick {t}: {}",
                    tank_c(&demo)
                );
            }
            if t == DEMO_TRIP_TICK {
                // The trip compared 75.0007 °C at the top of this tick and the whole
                // tick then ran on cold feed: the condition clears INSIDE the tick
                // that trips (§26, M22.1), and only the latch holds the cut.
                assert!(
                    tank_c(&demo) < DEMO_LIMIT_C,
                    "{solver}: the condition clears within the tripping tick: {}",
                    tank_c(&demo)
                );
            }
        }
        // Cooled toward its 40 °C feed (40.36 °C measured), while the twin, still
        // fired, nears the 89.93 °C its inflow carries (89.77 °C measured).
        assert!(tank_c(&demo) < 41.0, "{solver}: {}", tank_c(&demo));
        assert!(tank_c(&twin) > 89.0, "{solver}: {}", tank_c(&twin));
    }
}

// ------------------------------------------------------------- gates 3 and 4

const LOOP_LIMIT_C: f64 = 65.0;

/// `tank_temperature_heating.toml` — a reverse PI loop holding 60 °C on the
/// furnace — with a 65 °C trip on the tank that cuts the furnace, and the
/// loop's setpoint raised to 72 °C (inside the 73.28 °C full firing reaches), so
/// the loop itself drives the tank into the trip with the furnace firing hard.
fn loop_fixture() -> Engine {
    let block = format!(
        r#"[[trips]]
name = "tank_high_temperature"
measurement = {{ node = "hold_tank", variable = "temperature" }}
direction = "high"
limit_c = {LOOP_LIMIT_C:?}
actions = [{{ furnace = "heater" }}]"#
    );
    let mut engine = build(&format!("{TANK_LOOP}\n{block}\n"));
    set_setpoint_c(&mut engine, "tank_temperature", 72.0);
    engine
}

/// **Gate 3. Trips run before the loops**, so on the tripping tick a loop on
/// the cut furnace already reports MANUAL and a faceplate of exactly zero — its
/// own output that tick would have been the furnace near full firing.
#[test]
fn the_tripping_tick_reports_the_furnace_loop_in_manual_at_zero() {
    let mut engine = loop_fixture();
    let mut last_auto_output = None;
    for _ in 0..5000 {
        tick(&mut engine);
        if state(&engine).is_tripped() {
            break;
        }
        let face = faceplate(&engine, "tank_temperature");
        assert_eq!(face.mode, ControlMode::Auto);
        last_auto_output = Some(face.output);
    }
    let snapshot = engine.snapshot();
    let TripState::Tripped { at_tick } = snapshot.trips[0].state else {
        panic!("the raised setpoint never drove the tank into the trip");
    };
    assert_eq!(at_tick, snapshot.tick, "the snapshot of the tripping tick");
    assert!(at_tick > 1, "the trip fires on a crossing, mid-run");
    // The loop was firing hard while it drove the tank up, so a faceplate of
    // 0.0 below cannot be the loop's own output.
    assert!(last_auto_output.unwrap() > 0.5, "{last_auto_output:?}");
    let face = faceplate(&engine, "tank_temperature");
    assert_eq!(face.mode, ControlMode::Manual, "the trip forced MANUAL");
    assert_eq!(face.output, 0.0, "MANUAL tracks the cut furnace");
    assert_eq!(duty_w(&engine), 0.0);
}

/// **Gate 4. Every writer of the cut furnace is refused while latched; the cut
/// itself is admitted; and the hand-back is M22's**: the reset relights nothing,
/// a human fires the furnace, and AUTO takes over without a bump.
#[test]
fn a_cut_furnace_is_refused_every_relight_until_the_reset_and_the_reset_relights_nothing() {
    let mut engine = loop_fixture();
    until(&mut engine, 5000, "the trip", |e| state(e).is_tripped());

    expect_refused(
        set_duty(&mut engine, 1.0e6),
        "relighting by hand",
        "fuel cut",
    );
    expect_refused(
        set_mode(&mut engine, "tank_temperature", ControlMode::Auto),
        "AUTO while latched",
        "latched",
    );
    // Zero on a cut furnace moves nothing, and a frontend's "cut" button must not
    // fail on a plant that is already safe.
    set_duty(&mut engine, 0.0).expect("writing the cut itself is admitted");
    // MANUAL is what the trip already put the loop in.
    set_mode(&mut engine, "tank_temperature", ControlMode::Manual)
        .expect("MANUAL is admitted while latched");

    // A tick later the latch still holds the cut — the hold check runs on every
    // tick a trip is latched, and finds the furnace where the trip put it.
    tick(&mut engine);
    assert_eq!(duty_w(&engine), 0.0);

    until(&mut engine, 5000, "the tank to cool below the limit", |e| {
        tank_c(e) < LOOP_LIMIT_C
    });
    reset(&mut engine).expect("the condition has cleared");
    assert_eq!(state(&engine), TripState::Armed);
    // The reset moved nothing: the furnace is still cut and the loop in MANUAL.
    assert_eq!(duty_w(&engine), 0.0);
    assert_eq!(
        faceplate(&engine, "tank_temperature").mode,
        ControlMode::Manual
    );

    // A human puts the loop back where it should take over, lights the furnace,
    // and hands it back.
    set_setpoint_c(&mut engine, "tank_temperature", 60.0);
    set_duty(&mut engine, 1.2e6).expect("the reset lifted the refusal");
    set_mode(&mut engine, "tank_temperature", ControlMode::Auto)
        .expect("the reset lifted the refusal");
    tick(&mut engine);
    let output = faceplate(&engine, "tank_temperature").output;
    // M8.3's transfer: the next update returns the position the loop was
    // seeded from (1.2 MW of a 2 MW range), up to the few ULP of its own
    // back-calculation.
    assert!(
        (output - 0.6).abs() < 1e-12,
        "AUTO took over at {output}, not at the 0.6 it was handed"
    );
}

// ------------------------------------------------------------------ gate 5

/// **Gate 5. A reset inside its condition is refused**, on the demo loaded at
/// 80 °C against its 75 °C limit: it trips on tick 1, before any solve, and cools
/// for many ticks before the condition clears.
#[test]
fn a_plant_loaded_inside_its_condition_cuts_on_tick_one_and_refuses_the_reset_until_it_cools() {
    let hot = swap(
        DEMO,
        "initial_level_m = 4.968\ntemperature_c = 40.0",
        "initial_level_m = 4.968\ntemperature_c = 80.0",
    );
    let mut engine = build(&hot);
    tick(&mut engine);
    assert_eq!(state(&engine), TripState::Tripped { at_tick: 1 });
    assert_eq!(
        duty_w(&engine),
        0.0,
        "the fuel is cut before tick 1's solve"
    );
    assert!(tank_c(&engine) >= DEMO_LIMIT_C, "{}", tank_c(&engine));
    expect_refused(
        reset(&mut engine),
        "a reset inside the condition",
        "still holds",
    );
    expect_refused(set_duty(&mut engine, 3.0e6), "relighting", "fuel cut");

    let cooled = until(&mut engine, 5000, "the tank to cool below 75 °C", |e| {
        tank_c(e) < DEMO_LIMIT_C
    });
    assert!(
        cooled > 100,
        "the action is slow here: cleared at tick {cooled}"
    );
    reset(&mut engine).expect("the condition has cleared");
    assert_eq!(duty_w(&engine), 0.0, "the reset relights nothing");
    set_duty(&mut engine, 3.0e6).expect("after the reset a human may relight");
}

// ------------------------------------------------------------------ gate 6

const CASCADE_LIMIT_C: f64 = 62.0;

/// **Gate 6. A fuel cut opens a cascade** (§29 fork 4's rule, now reached by a
/// furnace action): the trip forces the INNER loop, the furnace's own, to
/// MANUAL, and from that tick the primary writes nothing and tracks. Tripped
/// mid-run with the furnace firing, because tick 1 is open anyway and an
/// already-idle furnace would make the cut invisible.
#[test]
fn a_fuel_cut_opens_the_cascade_and_it_closes_again_after_a_reset_and_a_relight() {
    let block = format!(
        r#"[[trips]]
name = "tank_high_temperature"
measurement = {{ node = "hold_tank", variable = "temperature" }}
direction = "high"
limit_c = {CASCADE_LIMIT_C:?}
actions = [{{ furnace = "heater" }}]"#
    );
    let mut engine = build(&format!("{CASCADE}\n{block}\n"));
    for _ in 0..4000 {
        tick(&mut engine);
    }
    assert!(
        state(&engine) == TripState::Armed,
        "the settled cascade is armed"
    );
    // The outer target raised inside the 65 °C range: the cascade fires harder
    // and walks the tank up into the trip.
    set_setpoint_c(&mut engine, "tank_temperature", 64.0);

    let inner_setpoint = |e: &Engine| match faceplate(e, "outlet_temperature").setpoint {
        ControlledValue::Temperature { k } => k.value(),
        other => panic!("the inner loop holds a temperature, not {other:?}"),
    };
    let mut duty_before = 0.0;
    let mut tripped_at = None;
    let mut frozen = 0.0;
    for _ in 0..4000 {
        let setpoint_before = inner_setpoint(&engine);
        tick(&mut engine);
        let t = engine.snapshot().tick;
        if tripped_at.is_none() {
            if state(&engine).is_tripped() {
                tripped_at = Some(t);
                frozen = setpoint_before;
            } else {
                duty_before = duty_w(&engine);
                continue;
            }
        }
        assert_eq!(duty_w(&engine), 0.0, "tick {t}: the fuel is cut");
        let inner = faceplate(&engine, "outlet_temperature");
        assert_eq!(
            inner.mode,
            ControlMode::Manual,
            "tick {t}: the inner loop yielded"
        );
        assert_eq!(inner.output, 0.0, "tick {t}: and tracks the cut furnace");
        let primary = faceplate(&engine, "tank_temperature");
        assert_eq!(
            primary.mode,
            ControlMode::Auto,
            "tick {t}: the primary stays AUTO"
        );
        assert_eq!(
            inner_setpoint(&engine),
            frozen,
            "tick {t}: from the tripping tick the primary writes nothing"
        );
        if t == tripped_at.unwrap() {
            // The operator brings the outer target back at once, while the
            // cascade is OPEN: the open primary re-seeds its memory against this
            // target on every open tick, so the change costs no kick at the
            // close. (Moved in the same batch as the close instead, it lands as
            // a proportional kick of `K·Δe`, 13.2 K on the inner target — M25's
            // finding, not the trip's.)
            set_setpoint_c(&mut engine, "tank_temperature", 60.0);
        }
        if t > tripped_at.unwrap() + 200 && tank_c(&engine) < CASCADE_LIMIT_C {
            break;
        }
    }
    let at = tripped_at.expect("the raised target must walk the tank into the trip");
    assert!(
        duty_before > 0.2e6,
        "the furnace was firing when it was cut ({duty_before} W at tick {})",
        at - 1
    );

    // Hand it back: the reset, a relight at the inner loop's own range, the
    // inner loop to AUTO. The primary closes by itself on the next tick.
    reset(&mut engine).expect("the condition has cleared");
    set_duty(&mut engine, 0.8e6).expect("the reset lifted the refusal");
    set_mode(&mut engine, "outlet_temperature", ControlMode::Auto)
        .expect("the reset lifted the refusal");
    tick(&mut engine);
    // The primary closes with one tick of control: measured +0.0023 K, onto
    // the top of its 40–65 °C range, where the tank's 1.9 K shortfall drives it.
    // A target moved in the same batch as the close lands 13.2 K away instead.
    let step = inner_setpoint(&engine) - frozen;
    assert!(
        step != 0.0 && step.abs() <= 0.05,
        "with the inner loop back in AUTO the primary writes again, by one tick of          control: it moved {step:+e} K"
    );
    assert!(duty_w(&engine) > 0.0, "the furnace is regulated again");
}
