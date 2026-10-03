//! M22.1: trips on fixtures (docs/DESIGN.md §26).
//!
//! The demo (`trip_demo.rs`) shows a trip firing and latching, and that is all a
//! CLI run can show: it issues no commands. Everything a command reaches is here,
//! each on the smallest fixture that exposes it, built from a shipped plant with
//! a `[[trips]]` block appended or swapped in:
//!
//! - **gate 3**, at the limit, both directions, on a vessel's PRESSURE and a
//!   tank's TEMPERATURE — the two quantities whose tie at load was measured to be
//!   exact (fork 6). The arms that must NOT trip are the ones that catch a
//!   missing unit conversion: an unconverted `limit_bar` is 13 Pa against a
//!   1.2 MPa vessel, an unconverted `limit_c` is 81 K against 353 K, and both
//!   would fire.
//! - **gate 4**, the tick order: trips run BEFORE the loops, so on the tripping
//!   tick a loop on the tripped valve already reports MANUAL and a faceplate of
//!   exactly the trip's position.
//! - **gate 5**, the loop hand-back: AUTO refused while latched; after the reset
//!   and a human reopening the valve, AUTO takes over without stepping it.
//! - **gate 6**, every refusal in fork 4, with the writes that EQUAL the safe
//!   state admitted, and two trips on one pump.
//! - **gate 9**, the load-time refusal sweep, each case asserting a substring of
//!   its OWN message, so a case that is refused for the wrong reason fails.

use refinery_core::graph::{
    ControlMode, ControlledValue, LoopId, NodeId, NodeKind, TripId, TripState,
};
use refinery_core::snapshot::Command;
use refinery_core::units::Meter;
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/tank_overfill_trip.toml");
const VESSEL: &str = include_str!("../../../scenarios/vessel_pressure_control.toml");
const HOT_TANK: &str = include_str!("../../../scenarios/tank_temperature_control.toml");
const LEVEL_LOOP: &str = include_str!("../../../scenarios/tank_level_control.toml");
const FURNACE: &str = include_str!("../../../scenarios/furnace_outlet_control.toml");
const RELIEF: &str = include_str!("../../../scenarios/relief_blowdown.toml");
const HEATING: &str = include_str!("../../../scenarios/tank_temperature_heating.toml");
const LEAKING: &str = include_str!("../../../scenarios/leaking_line.toml");

/// The demo's own trip block, verbatim, so a swap into it must land.
const DEMO_TRIP: &str = r#"[[trips]]
name = "receiving_high_level"
measurement = { node = "receiving_tank", variable = "level" }
direction = "high"
limit_m = 6.0
actions = [
  { pump = "transfer_pump" },
  { valve = "discharge_valve", position = 0.0 },
]"#;

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the fixture must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("the fixture must build: {e}"))
}

/// The refusal a fixture earns, whichever stage produced it. `what` names the
/// case, so a plant that LOADS says which refusal went missing.
fn refusal(what: &str, src: &str) -> String {
    match refinery_scenarios::load_str(src) {
        Err(e) => e.to_string(),
        Ok(file) => match refinery_scenarios::build_engine(&file) {
            Ok(_) => panic!("{what}: this plant should not have loaded"),
            Err(e) => e.to_string(),
        },
    }
}

fn swap(src: &str, from: &str, to: &str) -> String {
    assert!(
        src.contains(from),
        "the fixture's substitution must land: `{from}` is not in the plant"
    );
    src.replace(from, to)
}

fn with_trip(src: &str, block: &str) -> String {
    format!("{src}\n{block}\n")
}

fn tick(engine: &mut Engine) {
    engine
        .tick()
        .unwrap_or_else(|e| panic!("tick {}: {e}", engine.snapshot().tick + 1));
}

fn node(engine: &Engine, name: &str) -> NodeId {
    engine
        .graph
        .find_node(name)
        .unwrap_or_else(|| panic!("the fixture has a node '{name}'"))
}

fn pump_on(engine: &Engine, name: &str) -> bool {
    match engine.graph.node(node(engine, name)).kind {
        NodeKind::Pump { on, .. } => on,
        ref other => panic!("{name} is a pump, not {other:?}"),
    }
}

fn opening(engine: &Engine, name: &str) -> f64 {
    match engine.graph.node(node(engine, name)).kind {
        NodeKind::Valve { opening, .. } => opening,
        ref other => panic!("{name} is a valve, not {other:?}"),
    }
}

fn level(engine: &Engine, tank: &str) -> f64 {
    match &engine.graph.node(node(engine, tank)).kind {
        NodeKind::Tank(t) => t.level(&engine.slate).value(),
        other => panic!("{tank} is a tank, not {other:?}"),
    }
}

fn state(engine: &Engine, trip: u32) -> TripState {
    engine.snapshot().trips[trip as usize].state
}

fn set_pump(engine: &mut Engine, name: &str, on: bool) -> Result<(), String> {
    let node = node(engine, name);
    engine
        .apply(Command::SetPumpOn { node, on })
        .map_err(|e| e.to_string())
}

fn set_valve(engine: &mut Engine, name: &str, opening: f64) -> Result<(), String> {
    let node = node(engine, name);
    engine
        .apply(Command::SetValveOpening { node, opening })
        .map_err(|e| e.to_string())
}

fn reset(engine: &mut Engine, trip: u32) -> Result<(), String> {
    engine
        .apply(Command::ResetTrip {
            trip_id: TripId(trip),
        })
        .map_err(|e| e.to_string())
}

fn set_mode(engine: &mut Engine, mode: ControlMode) -> Result<(), String> {
    engine
        .apply(Command::SetControllerMode {
            loop_id: LoopId(0),
            mode,
        })
        .map_err(|e| e.to_string())
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

fn expect_refused(result: Result<(), String>, what: &str, says: &str) {
    match result {
        Ok(()) => panic!("{what} should have been refused"),
        Err(e) => assert!(
            e.contains(says),
            "{what} was refused for another reason: {e}"
        ),
    }
}

// ------------------------------------------------------------------ gate 3

/// A trip block on `node`'s `variable` at `limit_key = limit`, acting on `valve`.
fn trip_block(
    node: &str,
    variable: &str,
    direction: &str,
    limit_key: &str,
    limit: f64,
    valve: &str,
) -> String {
    format!(
        r#"[[trips]]
name = "at_the_limit"
measurement = {{ node = "{node}", variable = "{variable}" }}
direction = "{direction}"
{limit_key} = {limit:?}
actions = [{{ valve = "{valve}", position = 1.0 }}]"#
    )
}

/// Whether the plant is tripped after its FIRST tick, which is the pass that
/// compares the declared state against the limit.
fn trips_on_tick_one(plant: &str, block: &str) -> bool {
    let mut engine = build(&with_trip(plant, block));
    tick(&mut engine);
    match state(&engine, 0) {
        TripState::Tripped { at_tick } => {
            assert_eq!(at_tick, 1, "a trip fired on tick 1 says so");
            true
        }
        TripState::Armed => false,
    }
}

#[test]
fn a_trip_fires_at_its_limit_on_a_vessels_pressure_and_not_one_bar_inside_it() {
    // `receiver` is declared at 12.0 bar; the tie is exact (fork 6).
    let at = |direction: &str, limit: f64| {
        trips_on_tick_one(
            VESSEL,
            &trip_block(
                "receiver",
                "pressure",
                direction,
                "limit_bar",
                limit,
                "vent_valve",
            ),
        )
    };
    assert!(at("high", 12.0), "a high trip AT its limit fires");
    assert!(!at("high", 13.0), "a high trip one bar above does not");
    assert!(at("low", 12.0), "a low trip AT its limit fires");
    assert!(!at("low", 11.0), "a low trip one bar below does not");
}

#[test]
fn a_trip_fires_at_its_limit_on_a_tanks_temperature_and_not_one_degree_inside_it() {
    // `hold_tank` is declared at 80.0 °C; the tie is exact (fork 6).
    let at = |direction: &str, limit: f64| {
        trips_on_tick_one(
            HOT_TANK,
            &trip_block(
                "hold_tank",
                "temperature",
                direction,
                "limit_c",
                limit,
                "drain_valve",
            ),
        )
    };
    assert!(at("high", 80.0), "a high trip AT its limit fires");
    assert!(!at("high", 81.0), "a high trip one degree above does not");
    assert!(at("low", 80.0), "a low trip AT its limit fires");
    assert!(!at("low", 79.0), "a low trip one degree below does not");
}

// ------------------------------------------------------------ gates 4 and 5

/// `tank_level_control.toml` with a LOW-level trip that shuts the PI loop's own
/// drain: a trip protecting what the drain feeds from running the tank dry.
/// The loop is lowered to a setpoint below the trip, so it opens the drain and
/// drives the level into the trip.
const LOW_LIMIT_M: f64 = 1.5;
fn loop_fixture() -> Engine {
    let block = format!(
        r#"[[trips]]
name = "receiving_low_level"
measurement = {{ node = "receiving_tank", variable = "level" }}
direction = "low"
limit_m = {LOW_LIMIT_M:?}
actions = [{{ valve = "level_valve", position = 0.0 }}]"#
    );
    let mut engine = build(&with_trip(LEVEL_LOOP, &block));
    engine
        .apply(Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Level { m: Meter(0.5) },
        })
        .expect("a setpoint inside the tank is accepted");
    engine
}

#[test]
fn trips_run_before_the_loops_so_the_tripping_tick_reports_manual_and_the_trip_position() {
    let mut engine = loop_fixture();
    let mut last_auto_output = None;
    for _ in 0..5000 {
        tick(&mut engine);
        let snapshot = engine.snapshot();
        if snapshot.trips[0].state.is_tripped() {
            break;
        }
        assert_eq!(snapshot.controls[0].mode, ControlMode::Auto);
        last_auto_output = Some(snapshot.controls[0].output);
    }
    let snapshot = engine.snapshot();
    let TripState::Tripped { at_tick } = snapshot.trips[0].state else {
        panic!("the lowered loop never drove the level into the trip");
    };
    assert_eq!(at_tick, snapshot.tick, "the snapshot of the tripping tick");
    // The loop had the drain well open while it was driving the level down, so
    // a faceplate of 0.0 below cannot be the loop's own output.
    assert!(last_auto_output.unwrap() > 0.1, "{last_auto_output:?}");
    assert_eq!(snapshot.controls[0].mode, ControlMode::Manual);
    assert_eq!(
        snapshot.controls[0].output, 0.0,
        "MANUAL tracks the tripped valve"
    );
    assert_eq!(opening(&engine, "level_valve"), 0.0);
}

#[test]
fn after_a_reset_and_a_human_reopening_the_valve_auto_takes_over_without_a_bump() {
    let mut engine = loop_fixture();
    until(&mut engine, 5000, "the trip", |e| state(e, 0).is_tripped());

    // Latched: AUTO would reopen the valve at the next tick, so it is refused.
    expect_refused(
        set_mode(&mut engine, ControlMode::Auto),
        "AUTO while latched",
        "latched",
    );
    // **The condition has already cleared, inside the tripping tick itself.** The
    // trip compared the START-of-tick level and shut the drain before that
    // tick's solve, so the pump spent the whole tick filling: the level standing
    // now is above the limit. Only the latch is holding the valve — which is the
    // whole of fork 4, and why AUTO was refused just above with the condition
    // gone. (A reset refused INSIDE its condition is gate 6's, on a plant that
    // stays inside it.)
    assert!(
        level(&engine, "receiving_tank") > LOW_LIMIT_M,
        "the level recovered within the tripping tick: {}",
        level(&engine, "receiving_tank")
    );

    reset(&mut engine, 0).expect("the condition has cleared");
    assert_eq!(state(&engine, 0), TripState::Armed);
    // The reset moved nothing.
    assert_eq!(opening(&engine, "level_valve"), 0.0);
    assert_eq!(engine.snapshot().controls[0].mode, ControlMode::Manual);

    // A human puts the loop back where it should take over, and hands it back.
    engine
        .apply(Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Level { m: Meter(4.0) },
        })
        .expect("a setpoint move in MANUAL moves nothing");
    set_valve(&mut engine, "level_valve", 0.3).expect("the reset lifted the refusal");
    set_mode(&mut engine, ControlMode::Auto).expect("the reset lifted the refusal");
    tick(&mut engine);
    let output = engine.snapshot().controls[0].output;
    // M8.3's transfer: the next update returns the position the loop was
    // seeded from, up to the few ULP of its own back-calculation.
    assert!(
        (output - 0.3).abs() < 1e-12,
        "AUTO took over at {output}, not at the 0.3 it was handed"
    );
}

// ------------------------------------------------------------------ gate 6

/// The demo with its trip lowered to 1.5 m, so it fires on tick 1 (the tank is
/// declared at 2.0 m), plus a second trip that is armed and stays armed.
fn refusal_fixture() -> Engine {
    let lowered = swap(DEMO, "limit_m = 6.0", "limit_m = 1.5");
    build(&with_trip(
        &lowered,
        r#"[[trips]]
name = "supply_low_level"
measurement = { node = "supply_tank", variable = "level" }
direction = "low"
limit_m = 0.5
actions = [{ pump = "transfer_pump" }]"#,
    ))
}

#[test]
fn every_writer_of_tripped_equipment_is_refused_until_the_reset_and_the_reset_restarts_nothing() {
    let mut engine = refusal_fixture();
    tick(&mut engine);
    assert_eq!(state(&engine, 0), TripState::Tripped { at_tick: 1 });
    assert_eq!(state(&engine, 1), TripState::Armed);

    expect_refused(
        set_pump(&mut engine, "transfer_pump", true),
        "starting the pump",
        "latched",
    );
    expect_refused(
        set_valve(&mut engine, "discharge_valve", 0.3),
        "reopening the valve",
        "latched",
    );
    // Writes that equal the safe state move nothing, and are admitted.
    set_pump(&mut engine, "transfer_pump", false).expect("stopping a stopped pump");
    set_valve(&mut engine, "discharge_valve", 0.0).expect("shutting a shut valve");

    expect_refused(
        reset(&mut engine, 0),
        "a reset inside the condition",
        "still holds",
    );
    expect_refused(reset(&mut engine, 1), "resetting an armed trip", "is armed");
    expect_refused(reset(&mut engine, 7), "an unknown trip", "names no trip");

    // Held while latched: the hold check passes on every tick.
    let cleared = until(
        &mut engine,
        5000,
        "the level falling under the limit",
        |e| level(e, "receiving_tank") < 1.5,
    );
    assert_eq!(
        state(&engine, 0),
        TripState::Tripped { at_tick: 1 },
        "tick {cleared}"
    );

    reset(&mut engine, 0).expect("the condition has cleared");
    assert_eq!(
        state(&engine, 0),
        TripState::Armed,
        "a reset clears the tick"
    );
    assert!(
        !pump_on(&engine, "transfer_pump"),
        "the reset restarted nothing"
    );
    assert_eq!(
        opening(&engine, "discharge_valve"),
        0.0,
        "the reset moved nothing"
    );

    set_valve(&mut engine, "discharge_valve", 0.3).expect("the refusal is lifted");
    set_pump(&mut engine, "transfer_pump", true).expect("the refusal is lifted");
    tick(&mut engine);
    assert!(pump_on(&engine, "transfer_pump"));
}

#[test]
fn with_two_trips_on_one_pump_resetting_one_leaves_the_pump_held_by_the_other() {
    let lowered = swap(DEMO, "limit_m = 6.0", "limit_m = 1.5");
    // The supply tank is declared at 8.0 m, so a low trip at 9.0 fires on tick 1
    // too, and — with the pump stopped and the valve shut — never clears.
    let mut engine = build(&with_trip(
        &lowered,
        r#"[[trips]]
name = "supply_guard"
measurement = { node = "supply_tank", variable = "level" }
direction = "low"
limit_m = 9.0
actions = [{ pump = "transfer_pump" }]"#,
    ));
    tick(&mut engine);
    assert!(state(&engine, 0).is_tripped() && state(&engine, 1).is_tripped());
    until(&mut engine, 5000, "the receiving level clearing", |e| {
        level(e, "receiving_tank") < 1.5
    });
    reset(&mut engine, 0).expect("the first trip's condition has cleared");
    expect_refused(
        set_pump(&mut engine, "transfer_pump", true),
        "starting a pump the second trip still holds",
        "supply_guard",
    );
    // The valve was only the first trip's, and is free again.
    set_valve(&mut engine, "discharge_valve", 0.3).expect("no latched trip holds the valve");
}

// ------------------------------------------------------------------ gate 9

/// A high-temperature trip on `tank_temperature_heating.toml`'s tank with one
/// `action`, for the furnace cases of the sweep (M32, docs/DESIGN.md §35).
fn furnace_trip(action: &str) -> String {
    with_trip(
        HEATING,
        &format!(
            r#"[[trips]]
name = "overheat"
measurement = {{ node = "hold_tank", variable = "temperature" }}
direction = "high"
limit_c = 75.0
actions = [{action}]"#
        ),
    )
}

#[test]
fn every_trip_the_loader_cannot_honour_is_refused_for_its_own_reason() {
    let demo_with = |block: &str| swap(DEMO, DEMO_TRIP, block);
    let action = |from: &str, to: &str| demo_with(&swap(DEMO_TRIP, from, to));
    let cases: Vec<(&str, String, &str)> = vec![
        // M33 (docs/DESIGN.md §36) admits a flow. What used to be refused here
        // as E13 is now refused for the key: `limit_m` is a level's limit.
        (
            "a flow with a level's limit key",
            action(
                r#"{ node = "receiving_tank", variable = "level" }"#,
                r#"{ pipe = "fill_line", variable = "flow" }"#,
            ),
            "write `limit_kg_per_s` instead",
        ),
        (
            "a flow with no limit",
            action(
                r#"measurement = { node = "receiving_tank", variable = "level" }
direction = "high"
limit_m = 6.0"#,
                r#"measurement = { pipe = "fill_line", variable = "flow" }
direction = "low""#,
            ),
            "declares no `limit_kg_per_s`",
        ),
        (
            "a flow limit that is not a number",
            action(
                r#"measurement = { node = "receiving_tank", variable = "level" }
direction = "high"
limit_m = 6.0"#,
                r#"measurement = { pipe = "fill_line", variable = "flow" }
direction = "low"
limit_kg_per_s = nan"#,
            ),
            "not a number a flow can be compared with",
        ),
        (
            "a flow on a pipe the file does not declare",
            action(
                r#"measurement = { node = "receiving_tank", variable = "level" }
direction = "high"
limit_m = 6.0"#,
                r#"measurement = { pipe = "receiving_tank__overflow", variable = "flow" }
direction = "low"
limit_kg_per_s = 1.0"#,
            ),
            "which this file does not declare",
        ),
        (
            "a flow on a leaking pipe",
            with_trip(
                LEAKING,
                r#"[[trips]]
name = "low_fill"
measurement = { pipe = "fill_line", variable = "flow" }
direction = "low"
limit_kg_per_s = 1.0
actions = [{ pump = "transfer_pump" }]"#,
            ),
            "A flow is metered on a pipe with no leak path",
        ),
        (
            "a flow's limit key on a level",
            action("limit_m = 6.0", "limit_kg_per_s = 6.0"),
            "write `limit_m` instead",
        ),
        (
            "a furnace outlet",
            with_trip(
                FURNACE,
                r#"[[trips]]
name = "outlet"
measurement = { node = "heater", variable = "temperature" }
direction = "high"
limit_c = 90.0
actions = [{ valve = "drain_valve", position = 0.0 }]"#,
            ),
            // Since M34 a furnace's outlet exists on every tick after the first
            // (its coil, docs/DESIGN.md §37); the refusal says so and stays.
            "a trip on it is not admitted yet",
        ),
        (
            // A cooler has no coil: its outlet still goes missing whenever it
            // stagnates, and that is still the reason it is refused.
            "a cooler outlet",
            with_trip(
                HOT_TANK,
                r#"[[trips]]
name = "outlet"
measurement = { node = "chiller", variable = "temperature" }
direction = "high"
limit_c = 90.0
actions = [{ valve = "drain_valve", position = 0.0 }]"#,
            ),
            "not while the unit is stagnant",
        ),
        (
            "a cooler as equipment",
            with_trip(
                HOT_TANK,
                r#"[[trips]]
name = "cooler"
measurement = { node = "hold_tank", variable = "temperature" }
direction = "high"
limit_c = 95.0
actions = [{ valve = "chiller", position = 0.0 }]"#,
            ),
            "losing cooling",
        ),
        (
            "a cooler under the furnace key",
            with_trip(
                HOT_TANK,
                r#"[[trips]]
name = "cooler"
measurement = { node = "hold_tank", variable = "temperature" }
direction = "high"
limit_c = 95.0
actions = [{ furnace = "chiller" }]"#,
            ),
            "losing cooling",
        ),
        (
            "a furnace with a position",
            furnace_trip(r#"{ furnace = "heater", position = 0.0 }"#),
            "is its fuel cut",
        ),
        (
            "a furnace under the valve key",
            furnace_trip(r#"{ valve = "heater", position = 0.0 }"#),
            "it is not a valve",
        ),
        (
            "a furnace under the pump key",
            furnace_trip(r#"{ pump = "heater" }"#),
            "it is not a pump",
        ),
        (
            "the furnace key on a valve",
            furnace_trip(r#"{ furnace = "drain_valve" }"#),
            "it is not a furnace",
        ),
        (
            "a furnace and a valve in one action",
            furnace_trip(r#"{ furnace = "heater", valve = "drain_valve", position = 0.0 }"#),
            "more than one of",
        ),
        (
            "a relief valve as equipment",
            with_trip(
                RELIEF,
                r#"[[trips]]
name = "psv"
measurement = { node = "receiver", variable = "pressure" }
direction = "high"
limit_bar = 20.0
actions = [{ valve = "psv", position = 1.0 }]"#,
            ),
            "a relief valve",
        ),
        (
            "a level on a vessel",
            with_trip(
                VESSEL,
                r#"[[trips]]
name = "wrong_kind"
measurement = { node = "receiver", variable = "level" }
direction = "high"
limit_m = 1.0
actions = [{ valve = "vent_valve", position = 1.0 }]"#,
            ),
            "cannot measure level",
        ),
        (
            "a pump action on a valve",
            action(
                r#"{ pump = "transfer_pump" }"#,
                r#"{ pump = "discharge_valve" }"#,
            ),
            "it is not a pump",
        ),
        (
            "a valve action on a pump",
            action(
                r#"{ valve = "discharge_valve", position = 0.0 }"#,
                r#"{ valve = "transfer_pump", position = 0.0 }"#,
            ),
            "it is not a valve",
        ),
        (
            "a valve with no position",
            action(", position = 0.0 }", " }"),
            "with no `position`",
        ),
        (
            "a position outside [0, 1]",
            action("position = 0.0", "position = 1.5"),
            "outside",
        ),
        (
            "a position on a pump",
            action(
                r#"{ pump = "transfer_pump" }"#,
                r#"{ pump = "transfer_pump", position = 0.0 }"#,
            ),
            "has no position to hold",
        ),
        (
            "an action naming both",
            action(
                r#"{ pump = "transfer_pump" }"#,
                r#"{ pump = "transfer_pump", valve = "discharge_valve" }"#,
            ),
            "more than one of a `pump`, a `valve` and",
        ),
        (
            "an action naming neither",
            action(r#"{ pump = "transfer_pump" }"#, "{ }"),
            "none of a `pump`, a `valve` or a",
        ),
        (
            "unknown equipment",
            action(
                r#"{ pump = "transfer_pump" }"#,
                r#"{ pump = "no_such_pump" }"#,
            ),
            "acts on unknown node",
        ),
        (
            "two trips giving one valve different positions",
            with_trip(
                DEMO,
                r#"[[trips]]
name = "opens_it"
measurement = { node = "supply_tank", variable = "level" }
direction = "low"
limit_m = 0.5
actions = [{ valve = "discharge_valve", position = 1.0 }]"#,
            ),
            "must demand the same safe state",
        ),
        (
            "an empty actions list",
            demo_with(&DEMO_TRIP[..DEMO_TRIP.find("actions").unwrap()]),
            "declares no `actions`",
        ),
        (
            "a missing direction",
            action("direction = \"high\"\n", ""),
            "declares no `direction`",
        ),
        (
            "an unknown direction",
            action("direction = \"high\"", "direction = \"up\""),
            "unknown direction",
        ),
        (
            "the pressure key on a level trip",
            action("limit_m = 6.0", "limit_bar = 6.0"),
            "write `limit_m` instead",
        ),
        (
            "no limit",
            action("limit_m = 6.0\n", ""),
            "declares no `limit_m`",
        ),
        (
            "a level limit above the tank",
            action("limit_m = 6.0", "limit_m = 10.5"),
            "the tank's height",
        ),
        (
            "an unknown variable",
            action("variable = \"level\"", "variable = \"density\""),
            "unknown variable",
        ),
        (
            "a duplicate name",
            with_trip(DEMO, DEMO_TRIP),
            "two trips are called",
        ),
        (
            "an unknown key",
            action("limit_m = 6.0", "limit_m = 6.0\ndelay_s = 5.0"),
            "unknown field",
        ),
    ];
    for (what, src, says) in cases {
        let error = refusal(what, &src);
        assert!(
            error.contains(says),
            "{what} was refused for another reason: {error}"
        );
    }
}

/// Fork 5: a trip and a control loop on one valve is the ordinary case — an
/// overfill trip on a valve a loop regulates — and loads.
#[test]
fn a_trip_and_a_loop_on_one_valve_load_together() {
    let engine = build(&with_trip(
        VESSEL,
        &trip_block(
            "receiver",
            "pressure",
            "high",
            "limit_bar",
            25.0,
            "vent_valve",
        ),
    ));
    assert_eq!(engine.snapshot().trips.len(), 1);
    assert_eq!(engine.snapshot().controls.len(), 1);
}
