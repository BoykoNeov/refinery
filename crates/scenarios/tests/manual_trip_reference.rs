//! M38: a trip pressed by hand — the emergency-stop button (docs/DESIGN.md §43).
//!
//! `Command::ManualTrip` fires one armed trip at the command: the trip's own
//! latch, safe states and loop hand-over, with `by_hand: true` on its state. A
//! scenario file cannot press a button, so every gate here is a shipped plant
//! with commands applied to it, and no shipped plant moves:
//!
//! - **gate 1**, the press writes every safe state AT the command, before any
//!   tick, and the ticks after it run in them.
//! - **gate 2**, the tick a press records is the next one, the first to run in
//!   the safe state — `at_tick` means what it means for a measured trip.
//! - **gate 3**, every writer of the pressed equipment is refused from the
//!   press, a second press is refused, and so is an unknown id.
//! - **gate 4**, a press on a healthy plant resets at once and restarts
//!   nothing; re-armed, the trip then fires by its measurement as it always did.
//! - **gate 5**, a press inside the condition cannot be reset, and the trip pass
//!   that would have fired it keeps the press's record.
//! - **gate 6**, the wire form: `"by_hand":true` on a press, the M22 form
//!   unchanged on a measured trip.
//! - **gate 7**, a furnace trip: the press cuts the fuel and hands the loop on
//!   the furnace to MANUAL, on a trip whose measurement does not exist yet.
//! - **gate 8**, that trip's reset before tick 1 is refused as a command — it has
//!   nothing to compare — and goes through one tick later.

use refinery_core::graph::{ControlMode, LoopId, NodeId, NodeKind, TripId, TripState};
use refinery_core::snapshot::Command;
use refinery_core::units::Watt;
use refinery_core::Engine;

/// The M22 demo: the receiving tank is declared at 2.0 m against a HIGH trip at
/// 6.0 m that stops the pump and shuts the discharge valve; it fires by its
/// measurement on tick 1 236.
const DEMO: &str = include_str!("../../../scenarios/tank_overfill_trip.toml");
const DEMO_TRIP_TICK: u64 = 1_236;
/// The M35 demo: an outlet loop on a fouled furnace, a coil trip and an outlet
/// trip, both cutting the furnace's fuel.
const FURNACE: &str = include_str!("../../../scenarios/furnace_coil_trip.toml");

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the fixture must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("the fixture must build: {e}"))
}

fn swap(src: &str, from: &str, to: &str) -> String {
    assert!(
        src.contains(from),
        "the fixture's substitution must land: `{from}` is not in the plant"
    );
    src.replace(from, to)
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

fn duty(engine: &Engine, name: &str) -> Watt {
    match engine.graph.node(node(engine, name)).kind {
        NodeKind::Furnace { duty, .. } => duty,
        ref other => panic!("{name} is a furnace, not {other:?}"),
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

fn pressed(at_tick: u64) -> TripState {
    TripState::Tripped {
        at_tick,
        by_hand: true,
    }
}

fn press(engine: &mut Engine, trip: u32) -> Result<(), String> {
    engine
        .apply(Command::ManualTrip {
            trip_id: TripId(trip),
        })
        .map_err(|e| e.to_string())
}

fn reset(engine: &mut Engine, trip: u32) -> Result<(), String> {
    engine
        .apply(Command::ResetTrip {
            trip_id: TripId(trip),
        })
        .map_err(|e| e.to_string())
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

fn expect_refused(result: Result<(), String>, what: &str, says: &str) {
    match result {
        Ok(()) => panic!("{what} should have been refused"),
        Err(e) => assert!(
            e.contains(says),
            "{what} was refused for another reason: {e}"
        ),
    }
}

// ------------------------------------------------------------------ gate 1

#[test]
fn a_press_writes_every_safe_state_at_the_command_and_the_ticks_after_run_in_them() {
    let mut engine = build(DEMO);
    assert!(pump_on(&engine, "transfer_pump"));
    assert!(opening(&engine, "discharge_valve") > 0.0);
    assert_eq!(state(&engine, 0), TripState::Armed);

    press(&mut engine, 0).expect("an armed trip on a healthy plant can be pressed");
    // No tick has run: the command itself wrote the safe states.
    assert_eq!(engine.snapshot().tick, 0);
    assert_eq!(state(&engine, 0), pressed(1));
    assert!(!pump_on(&engine, "transfer_pump"), "the pump stopped");
    assert_eq!(opening(&engine, "discharge_valve"), 0.0, "the valve shut");

    // The hold check passes on every tick, and nothing fills the tank: it only
    // drains through its own level valve.
    let start = level(&engine, "receiving_tank");
    for _ in 0..50 {
        tick(&mut engine);
        assert_eq!(state(&engine, 0), pressed(1));
    }
    assert!(
        level(&engine, "receiving_tank") < start,
        "the tank drained from {start} m to {} m with its feed cut",
        level(&engine, "receiving_tank")
    );
}

// ------------------------------------------------------------------ gate 2

#[test]
fn a_press_between_ticks_records_the_next_tick_the_first_in_the_safe_state() {
    let mut engine = build(DEMO);
    for _ in 0..100 {
        tick(&mut engine);
    }
    press(&mut engine, 0).expect("the trip is armed at tick 100");
    assert_eq!(engine.snapshot().tick, 100);
    assert_eq!(state(&engine, 0), pressed(101));
    tick(&mut engine);
    assert_eq!(
        engine.snapshot().tick,
        101,
        "the snapshot of the first tick in the safe state carries the press's tick"
    );
    assert_eq!(state(&engine, 0), pressed(101));
}

// ------------------------------------------------------------------ gate 3

#[test]
fn every_writer_of_pressed_equipment_is_refused_and_so_is_a_second_press() {
    let mut engine = build(DEMO);
    press(&mut engine, 0).expect("the first press");

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

    expect_refused(press(&mut engine, 0), "a second press", "already tripped");
    expect_refused(press(&mut engine, 7), "an unknown trip", "names no trip");
    assert_eq!(
        state(&engine, 0),
        pressed(1),
        "the refused press moved nothing"
    );

    // A trip its measurement fired cannot be pressed either.
    let mut fired = build(&swap(DEMO, "limit_m = 6.0", "limit_m = 1.5"));
    tick(&mut fired);
    assert!(state(&fired, 0).is_tripped());
    expect_refused(
        press(&mut fired, 0),
        "pressing a fired trip",
        "already tripped",
    );
}

// ------------------------------------------------------------------ gate 4

#[test]
fn a_press_on_a_healthy_plant_resets_at_once_and_the_trip_then_fires_as_before() {
    let mut engine = build(DEMO);
    press(&mut engine, 0).expect("the press");
    reset(&mut engine, 0).expect("the level is far under the limit");
    assert_eq!(state(&engine, 0), TripState::Armed);
    assert!(
        !pump_on(&engine, "transfer_pump"),
        "the reset restarted nothing"
    );
    assert_eq!(
        opening(&engine, "discharge_valve"),
        0.0,
        "the reset moved nothing"
    );

    // A human restarts the plant at the demo's own positions, which is the
    // demo's state at load, so its measured trip fires on its own tick.
    let declared = opening(&build(DEMO), "discharge_valve");
    set_valve(&mut engine, "discharge_valve", declared).expect("the refusal is lifted");
    set_pump(&mut engine, "transfer_pump", true).expect("the refusal is lifted");
    for _ in 0..DEMO_TRIP_TICK {
        tick(&mut engine);
    }
    assert_eq!(
        state(&engine, 0),
        TripState::Tripped {
            at_tick: DEMO_TRIP_TICK,
            by_hand: false
        },
        "re-armed, the trip fires by its measurement on the demo's tick"
    );
}

// ------------------------------------------------------------------ gate 5

#[test]
fn a_press_inside_the_condition_cannot_be_reset_and_the_trip_pass_keeps_the_press() {
    // Lowered under the tank's declared 2.0 m, so tick 1's pass would fire it.
    let mut engine = build(&swap(DEMO, "limit_m = 6.0", "limit_m = 1.5"));
    press(&mut engine, 0).expect("still armed before the first tick");
    expect_refused(
        reset(&mut engine, 0),
        "a reset inside the condition",
        "still holds",
    );
    tick(&mut engine);
    assert_eq!(
        state(&engine, 0),
        pressed(1),
        "a latched trip is not re-latched by the pass its measurement reaches"
    );
}

// ------------------------------------------------------------------ gate 6

#[test]
fn the_wire_form_adds_by_hand_to_a_press_only() {
    let measured = TripState::Tripped {
        at_tick: 1_236,
        by_hand: false,
    };
    assert_eq!(
        serde_json::to_string(&measured).unwrap(),
        r#"{"status":"tripped","at_tick":1236}"#,
        "a measured trip writes the M22 form unchanged"
    );
    assert_eq!(
        serde_json::to_string(&pressed(1_236)).unwrap(),
        r#"{"status":"tripped","at_tick":1236,"by_hand":true}"#
    );
    let parsed: TripState = serde_json::from_str(r#"{"status":"tripped","at_tick":1236}"#).unwrap();
    assert_eq!(parsed, measured, "the M22 form reads back as measured");

    // And through a real snapshot, which is what a frontend reads.
    let mut engine = build(DEMO);
    press(&mut engine, 0).expect("the press");
    let snapshot = serde_json::to_value(engine.snapshot()).unwrap();
    assert_eq!(
        snapshot["trips"][0]["state"],
        serde_json::json!({"status": "tripped", "at_tick": 1, "by_hand": true})
    );
}

// ------------------------------------------------------------------ gate 7

#[test]
fn a_press_on_a_furnace_trip_cuts_the_fuel_and_hands_its_loop_to_manual() {
    let mut engine = build(FURNACE);
    assert!(duty(&engine, "heater") > Watt::ZERO, "the furnace is lit");
    assert_eq!(engine.snapshot().controls[0].mode, ControlMode::Auto);

    // Trip 1 watches the furnace's OUTLET, which does not exist until tick 1's
    // sweep (docs/DESIGN.md §39): a button needs no measurement.
    press(&mut engine, 1).expect("the outlet trip is armed");
    assert_eq!(duty(&engine, "heater"), Watt::ZERO, "the fuel is cut");
    assert_eq!(engine.snapshot().controls[0].mode, ControlMode::Manual);

    let heater = node(&engine, "heater");
    expect_refused(
        engine
            .apply(Command::SetFurnaceDuty {
                node: heater,
                duty: Watt(100_000.0),
            })
            .map_err(|e| e.to_string()),
        "relighting the furnace",
        "latched",
    );
    expect_refused(
        engine
            .apply(Command::SetControllerMode {
                loop_id: LoopId(0),
                mode: ControlMode::Auto,
            })
            .map_err(|e| e.to_string()),
        "handing the loop back",
        "latched",
    );

    for _ in 0..20 {
        tick(&mut engine);
    }
    assert_eq!(state(&engine, 1), pressed(1));
    assert_eq!(duty(&engine, "heater"), Watt::ZERO);
    // MANUAL tracks: from the first tick after the press, the faceplate reads
    // the cut fuel (between the press and that tick it still reads the last
    // AUTO output, as after any write between ticks — DESIGN §43 fork 4).
    assert_eq!(engine.snapshot().controls[0].output, 0.0);
}

// ------------------------------------------------------------------ gate 8

/// A trip pressed before its measurement exists cannot be reset until it does:
/// the reset compares a fresh reading, and before tick 1 a furnace's outlet has
/// none. Refused as a command, not raised as an engine fault.
#[test]
fn a_press_before_the_first_tick_on_an_unmeasured_trip_resets_only_after_a_tick() {
    let mut engine = build(FURNACE);
    press(&mut engine, 1).expect("the outlet trip is armed");
    let refused = engine.apply(Command::ResetTrip { trip_id: TripId(1) });
    match refused {
        Err(refinery_core::SimError::InvalidCommand(message)) => assert!(
            message.contains("after the first tick"),
            "refused for another reason: {message}"
        ),
        other => panic!("a reset with nothing to compare must be refused: {other:?}"),
    }
    assert_eq!(
        state(&engine, 1),
        pressed(1),
        "the refused reset moved nothing"
    );

    // One tick later the outlet exists, sits far under its 70 °C limit with the
    // fuel cut, and the reset goes through.
    tick(&mut engine);
    reset(&mut engine, 1).expect("the outlet is measured and under its limit");
    assert_eq!(state(&engine, 1), TripState::Armed);
}
