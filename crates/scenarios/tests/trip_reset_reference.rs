//! M40, docs/DESIGN.md §45: who resets a trip, and whether the reset hands back
//! what the trip stopped (`docs/DEFERRED.md` E15's "a reset that restarts
//! equipment").
//!
//! Every fixture is a shipped plant with its `[[trips]]` lines edited in memory,
//! so the plant is the file's to the bit until the trip, and no shipped file
//! changes (CLAUDE.md: the existing files are the regression anchor).
//!
//! - `furnace_coil_trip.toml`: a furnace under an AUTO outlet loop, cut by a
//!   tube-skin trip at tick 75 — the restart through the loop.
//! - `tank_overheat_trip.toml`: a furnace fired by hand at 3 MW, cut by a tank
//!   temperature trip — the restart of a duty with no loop.
//! - `tank_overfill_trip.toml`: a pump and a valve, stopped and shut by a level
//!   trip — the restart of both.
//!
//! The two load-bearing gates are equivalences, not numbers: a `manual_restart`
//! reset is byte-for-byte a manual reset followed by a person's AUTO (gate 2),
//! and a self-reset on tick N is byte-for-byte that reset sent between N − 1
//! and N (gate 3).

use refinery_core::graph::{ControlMode, LoopId, NodeKind, TripId, TripState};
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::Engine;
use refinery_scenarios::{build_engine, load_str};

const COIL_TRIP: &str = include_str!("../../../scenarios/furnace_coil_trip.toml");
const OVERHEAT_TRIP: &str = include_str!("../../../scenarios/tank_overheat_trip.toml");
const OVERFILL_TRIP: &str = include_str!("../../../scenarios/tank_overfill_trip.toml");

const TUBE_SKIN: TripId = TripId(0);
const OUTLET_LOOP: LoopId = LoopId(0);

/// The coil plant with `tube_skin_high`'s reset declared (`extra` is the
/// `reset` lines, inserted after its `limit_c`).
fn coil_plant(extra: &str) -> String {
    edit(COIL_TRIP, "limit_c = 100.0\n", extra)
}

/// Insert `extra` after the one line `anchor` in `src`.
fn edit(src: &str, anchor: &str, extra: &str) -> String {
    assert_eq!(
        src.matches(anchor).count(),
        1,
        "anchor '{anchor}' is not unique"
    );
    src.replacen(anchor, &format!("{anchor}{extra}"), 1)
}

fn build(src: &str) -> Engine {
    build_engine(&load_str(src).expect("the plant must parse")).expect("the plant must build")
}

fn refusal(src: &str) -> String {
    match build_engine(&load_str(src).expect("the plant must parse")) {
        Ok(_) => panic!("the plant loaded and should have been refused"),
        Err(e) => e.to_string(),
    }
}

fn tick(engine: &mut Engine) -> Snapshot {
    engine.tick().expect("the tick must run");
    engine.snapshot()
}

fn run_to(engine: &mut Engine, tick_index: u64) -> Snapshot {
    while engine.snapshot().tick < tick_index {
        tick(engine);
    }
    engine.snapshot()
}

fn heater(snapshot: &Snapshot) -> &refinery_core::snapshot::NodeSnapshot {
    snapshot.nodes.iter().find(|n| n.name == "heater").unwrap()
}

fn duty_w(snapshot: &Snapshot) -> f64 {
    match &heater(snapshot).kind {
        NodeKind::Furnace { duty, .. } => duty.value(),
        other => panic!("heater is not a furnace: {other:?}"),
    }
}

fn coil_c(snapshot: &Snapshot) -> f64 {
    match &heater(snapshot).kind {
        NodeKind::Furnace { coil, .. } => coil.temperature.value() - 273.15,
        other => panic!("heater is not a furnace: {other:?}"),
    }
}

fn loop_mode(snapshot: &Snapshot) -> ControlMode {
    snapshot.controls[OUTLET_LOOP.0 as usize].mode
}

fn tripped(snapshot: &Snapshot, trip: TripId) -> bool {
    snapshot.trips[trip.0 as usize].state.is_tripped()
}

/// Everything a player sees of the plant, without the trips' own records
/// (whose `reset` field differs between the two plants an equivalence
/// compares) — and so without each node's `trip_stop` (M43), the trips'
/// account of what their resets will do, which differs for the same reason.
fn plant_bytes(snapshot: &Snapshot) -> String {
    let mut nodes = snapshot.nodes.clone();
    for node in &mut nodes {
        node.trip_stop = None;
    }
    format!(
        "{}{}{}",
        serde_json::to_string(&nodes).unwrap(),
        serde_json::to_string(&snapshot.edges).unwrap(),
        serde_json::to_string(&snapshot.controls).unwrap()
    )
}

/// The first tick on which `trip` reads armed again after having tripped.
fn rearm_tick(engine: &mut Engine, trip: TripId, limit: u64) -> u64 {
    let mut seen_trip = false;
    while engine.snapshot().tick < limit {
        let snapshot = tick(engine);
        let now = tripped(&snapshot, trip);
        if seen_trip && !now {
            return snapshot.tick;
        }
        seen_trip |= now;
    }
    panic!("trip {trip:?} did not re-arm by tick {limit}");
}

/// Gate 1 — the default is the real-plant rule, unchanged: a person resets the
/// trip and nothing restarts. The loop stays in MANUAL, the furnace dark, and
/// the snapshot writes no `reset` key.
#[test]
fn by_default_a_reset_restarts_nothing() {
    let mut engine = build(COIL_TRIP);
    let at_trip = run_to(&mut engine, 75);
    assert!(
        tripped(&at_trip, TUBE_SKIN),
        "the coil trip fires on tick 75"
    );
    assert!(
        !serde_json::to_string(&at_trip)
            .unwrap()
            .contains("\"reset\""),
        "a manual trip writes no reset key"
    );
    run_to(&mut engine, 150);
    engine
        .apply(Command::ResetTrip { trip_id: TUBE_SKIN })
        .unwrap();
    let after = tick(&mut engine);
    assert!(!tripped(&after, TUBE_SKIN));
    assert_eq!(loop_mode(&after), ControlMode::Manual);
    assert_eq!(duty_w(&after), 0.0);
}

/// Gate 2 — `manual_restart`: the reset a person presses also puts the outlet
/// loop back in AUTO, through the bumpless transfer, from the furnace's safe
/// state. Byte-for-byte what M39's screen does by hand at tick 150 (reset, then
/// AUTO), for 200 ticks — so the relight ramps from zero rather than jumping
/// back to the firing that tripped it.
#[test]
fn a_restarting_reset_is_a_reset_and_a_persons_auto() {
    let mut restarting = build(&coil_plant("reset = \"manual_restart\"\n"));
    let mut by_hand = build(COIL_TRIP);
    let before = run_to(&mut restarting, 74);
    let fired_before_trip = duty_w(&before);
    run_to(&mut restarting, 150);
    run_to(&mut by_hand, 150);
    assert!(tripped(&restarting.snapshot(), TUBE_SKIN));

    restarting
        .apply(Command::ResetTrip { trip_id: TUBE_SKIN })
        .unwrap();
    // At the command, as a press writes at the command: the loop is in AUTO
    // now and the furnace has not been written.
    assert_eq!(loop_mode(&restarting.snapshot()), ControlMode::Auto);
    assert_eq!(duty_w(&restarting.snapshot()), 0.0);

    by_hand
        .apply(Command::ResetTrip { trip_id: TUBE_SKIN })
        .unwrap();
    by_hand
        .apply(Command::SetControllerMode {
            loop_id: OUTLET_LOOP,
            mode: ControlMode::Auto,
        })
        .unwrap();
    for _ in 0..200 {
        let (a, b) = (tick(&mut restarting), tick(&mut by_hand));
        assert_eq!(plant_bytes(&a), plant_bytes(&b), "tick {}", a.tick);
    }
    let first = {
        let mut probe = build(&coil_plant("reset = \"manual_restart\"\n"));
        run_to(&mut probe, 150);
        probe
            .apply(Command::ResetTrip { trip_id: TUBE_SKIN })
            .unwrap();
        duty_w(&tick(&mut probe))
    };
    assert!(
        first < 0.1 * fired_before_trip,
        "the relight ramps: {first} W on its first tick against {fired_before_trip} W before \
         the trip"
    );
}

/// Gate 3 — `auto`: the trip re-arms itself on the first pass whose reading is
/// strictly below its 80 °C reset point, and not one pass sooner; and that
/// self-reset on tick N is byte-for-byte a `manual_restart` reset sent between
/// N − 1 and N.
#[test]
fn a_self_resetting_trip_rearms_past_its_reset_point_as_a_restarting_reset_would() {
    let mut selfish = build(&coil_plant("reset = \"auto\"\nreset_limit_c = 80.0\n"));
    let rearm = rearm_tick(&mut selfish, TUBE_SKIN, 1_000);
    let at_rearm = selfish.snapshot();
    // The pass compares the reading standing at the top of the tick: the coil
    // the previous snapshot published.
    let mut probe = build(&coil_plant("reset = \"auto\"\nreset_limit_c = 80.0\n"));
    let before = run_to(&mut probe, rearm - 1);
    assert!(
        tripped(&before, TUBE_SKIN),
        "still tripped on tick {}",
        rearm - 1
    );
    let compared = |s: &Snapshot| match s.trips[0].measurement {
        Some(refinery_core::graph::ControlledValue::Temperature { k }) => k.value() - 273.15,
        other => panic!("{other:?}"),
    };
    assert!(compared(&before) >= 80.0, "{}", compared(&before));
    assert!(compared(&at_rearm) < 80.0, "{}", compared(&at_rearm));
    assert_eq!(loop_mode(&at_rearm), ControlMode::Auto);
    assert_eq!(rearm, 128, "measured: the first re-arm is on tick 128");

    let mut restarting = build(&coil_plant("reset = \"manual_restart\"\n"));
    run_to(&mut restarting, rearm - 1);
    restarting
        .apply(Command::ResetTrip { trip_id: TUBE_SKIN })
        .unwrap();
    let pressed = tick(&mut restarting);
    assert_eq!(plant_bytes(&pressed), plant_bytes(&at_rearm));
    for _ in 0..40 {
        let (a, b) = (tick(&mut selfish), tick(&mut restarting));
        assert_eq!(plant_bytes(&a), plant_bytes(&b), "tick {}", a.tick);
    }
}

/// Gate 4 — a furnace with no loop gets its own pre-trip duty back, exactly:
/// the 3 MW `tank_overheat_trip.toml` fires by hand.
#[test]
fn a_furnace_fired_by_hand_gets_its_duty_back() {
    let src = edit(
        OVERHEAT_TRIP,
        "limit_c = 75.0\n",
        "reset = \"auto\"\nreset_limit_c = 60.0\n",
    );
    let mut engine = build(&src);
    let fired = duty_w(&engine.snapshot());
    assert_eq!(fired, 3.0e6, "the file fires 3 MW by hand");
    let rearm = rearm_tick(&mut engine, TripId(0), 20_000);
    let after = engine.snapshot();
    assert_eq!(duty_w(&after), fired, "re-armed on tick {rearm}");
}

/// Gate 5 — a pump comes back on and a valve back to its own opening.
#[test]
fn a_pump_and_a_valve_come_back_as_they_were() {
    let src = edit(
        OVERFILL_TRIP,
        "limit_m = 6.0\n",
        "reset = \"auto\"\nreset_limit_m = 5.0\n",
    );
    let mut engine = build(&src);
    let state = |s: &Snapshot| {
        let pump = s.nodes.iter().find(|n| n.name == "transfer_pump").unwrap();
        let valve = s
            .nodes
            .iter()
            .find(|n| n.name == "discharge_valve")
            .unwrap();
        let on = match &pump.kind {
            NodeKind::Pump { on, .. } => *on,
            other => panic!("{other:?}"),
        };
        let opening = match &valve.kind {
            NodeKind::Valve { opening, .. } => *opening,
            other => panic!("{other:?}"),
        };
        (on, opening)
    };
    let before = state(&tick(&mut engine));
    assert!(
        before.0 && before.1 > 0.0,
        "running before the trip: {before:?}"
    );
    let rearm = rearm_tick(&mut engine, TripId(0), 20_000);
    assert_eq!(
        state(&engine.snapshot()),
        before,
        "re-armed on tick {rearm}"
    );
}

/// Gate 6 — a trip pressed by hand never restarts anything, whatever its mode:
/// it does not reset itself, and a person's reset leaves the furnace dark.
#[test]
fn an_emergency_stop_is_never_restarted_by_the_trip() {
    let mut engine = build(&coil_plant("reset = \"auto\"\nreset_limit_c = 80.0\n"));
    run_to(&mut engine, 10);
    engine
        .apply(Command::ManualTrip { trip_id: TUBE_SKIN })
        .unwrap();
    let later = run_to(&mut engine, 600);
    assert!(
        coil_c(&later) < 80.0,
        "the coil is well under its reset point"
    );
    assert!(
        tripped(&later, TUBE_SKIN),
        "a pressed trip does not reset itself"
    );
    engine
        .apply(Command::ResetTrip { trip_id: TUBE_SKIN })
        .unwrap();
    let after = tick(&mut engine);
    assert_eq!(loop_mode(&after), ControlMode::Manual);
    assert_eq!(duty_w(&after), 0.0);
}

/// A third trip on the same furnace, watching the tank it heats: the tank warms
/// for ~260 ticks after the cut (measured: 40.77 °C at tick 75, 42.02 °C at
/// 334), so this one latches on a LATER pass than the tube trip, on a furnace
/// already dark, and clears near tick 1 500.
fn tank_warm(reset: &str) -> String {
    format!(
        "\n[[trips]]\nname = \"tank_warm\"\nmeasurement = {{ node = \"hold_tank\", variable = \
         \"temperature\" }}\ndirection = \"high\"\nlimit_c = 41.0\n{reset}actions = [{{ furnace \
         = \"heater\" }}]\n"
    )
}

/// Gate 7 — two trips on one furnace. The equipment comes back only when the
/// LAST holder lets go, and only as it stood before the FIRST trip: a record
/// taken by the second, on a furnace the first had cut, would hand back a dark
/// furnace with its loop in MANUAL.
#[test]
fn the_last_trip_to_let_go_restarts_the_furnace_as_it_was_before_the_first() {
    let src = coil_plant("reset = \"auto\"\nreset_limit_c = 80.0\n")
        + &tank_warm("reset = \"auto\"\nreset_limit_c = 40.9\n");
    let mut engine = build(&src);
    let tank = TripId(2);
    // The tube trip re-arms first, while the tank trip still holds the furnace.
    let tube_rearm = rearm_tick(&mut engine, TUBE_SKIN, 1_000);
    let at = engine.snapshot();
    assert!(
        tripped(&at, tank),
        "the tank trip latched after the tube trip"
    );
    assert_eq!(loop_mode(&at), ControlMode::Manual, "tick {tube_rearm}");
    assert_eq!(duty_w(&at), 0.0);
    // The tank trip lets go last, and the loop takes the furnace back.
    let tank_rearm = rearm_tick(&mut engine, tank, 6_000);
    let at = engine.snapshot();
    assert!(!tripped(&at, TUBE_SKIN));
    assert_eq!(loop_mode(&at), ControlMode::Auto, "tick {tank_rearm}");
}

/// Gate 8 — a `manual` trip that held the furnace keeps it for a person, even
/// after every trip has let go: the restart needs every holder's say.
#[test]
fn one_manual_holder_keeps_the_equipment_stopped_for_a_person() {
    let src = coil_plant("reset = \"auto\"\nreset_limit_c = 80.0\n") + &tank_warm("");
    let mut engine = build(&src);
    let tank = TripId(2);
    rearm_tick(&mut engine, TUBE_SKIN, 1_000);
    assert!(tripped(&engine.snapshot(), tank));
    // Wait for the tank to clear its limit, then a person resets the tank trip.
    while engine.snapshot().trips[2]
        .measurement
        .is_some_and(|m| m.magnitude() >= 273.15 + 41.0)
    {
        tick(&mut engine);
    }
    engine.apply(Command::ResetTrip { trip_id: tank }).unwrap();
    let after = tick(&mut engine);
    assert!(!tripped(&after, TUBE_SKIN) && !tripped(&after, tank));
    assert_eq!(loop_mode(&after), ControlMode::Manual);
    assert_eq!(duty_w(&after), 0.0);
}

/// Gate 9 — the loader's refusals, each in its own words.
#[test]
fn the_loader_refuses_a_reset_point_it_cannot_use() {
    let cases: [(&str, &str); 7] = [
        (
            "reset = \"auto\"\nreset_limit_c = 100.0\n",
            "not on the safe side",
        ),
        (
            "reset = \"auto\"\nreset_limit_c = 120.0\n",
            "strictly below its limit",
        ),
        ("reset = \"auto\"\n", "declares no `reset_limit_c`"),
        (
            "reset = \"auto\"\nreset_limit_m = 1.0\n",
            "Write `reset_limit_c` instead",
        ),
        (
            "reset = \"manual_restart\"\nreset_limit_c = 80.0\n",
            "read only by a trip that resets itself",
        ),
        ("reset = \"sometimes\"\n", "unknown reset 'sometimes'"),
        (
            "reset = \"auto\"\nreset_limit_c = -300.0\n",
            "outside its range",
        ),
    ];
    for (extra, expected) in cases {
        let message = refusal(&coil_plant(extra));
        assert!(
            message.contains(expected),
            "{extra:?}: expected '{expected}' in: {message}"
        );
    }
    // A low trip's reset point is ABOVE its limit.
    let low = coil_plant("").replacen(
        "direction = \"high\"\nlimit_c = 100.0\n",
        "direction = \"low\"\nlimit_c = 10.0\nreset = \"auto\"\nreset_limit_c = 5.0\n",
        1,
    );
    assert!(refusal(&low).contains("strictly above its limit"));
}

/// Gate 10 — the wire form a frontend branches on, and `TripState` untouched.
#[test]
fn the_snapshot_says_how_a_trip_resets() {
    let auto = build(&coil_plant("reset = \"auto\"\nreset_limit_c = 80.0\n")).snapshot();
    let bytes = serde_json::to_string(&auto).unwrap();
    assert!(
        bytes.contains(
            r#""reset":{"mode":"auto","reset_at":{"variable":"temperature","k":353.15}},"state":{"status":"armed"}"#
        ),
        "{bytes}"
    );
    let restart = build(&coil_plant("reset = \"manual_restart\"\n")).snapshot();
    let bytes = serde_json::to_string(&restart).unwrap();
    assert!(
        bytes.contains(r#""reset":{"mode":"manual_restart"}"#),
        "{bytes}"
    );
    assert_eq!(restart.trips[0].state, TripState::Armed);
}

/// Gate 11 — the demo, `furnace_coil_trip_autoreset.toml`, as its header says:
/// 43 trips in 6 000 ticks, one every 139 ticks from 75 to 5 913, a re-arm 53
/// ticks after each, the outlet never past 55.52 °C, `outlet_high` never fired.
/// Until tick 75 it is `furnace_coil_trip.toml` to the bit.
#[test]
fn the_self_resetting_demo_cuts_and_relights_every_139_ticks() {
    const DEMO: &str = include_str!("../../../scenarios/furnace_coil_trip_autoreset.toml");
    let mut demo = build(DEMO);
    let mut twin = build(COIL_TRIP);
    for _ in 0..75 {
        assert_eq!(plant_bytes(&tick(&mut demo)), plant_bytes(&tick(&mut twin)));
    }
    let mut demo = build(DEMO);
    let (mut trips, mut rearms) = (Vec::new(), Vec::new());
    let mut was = false;
    let mut hottest_outlet_k = f64::NEG_INFINITY;
    for _ in 0..6_000 {
        let s = tick(&mut demo);
        let now = tripped(&s, TUBE_SKIN);
        if now && !was {
            trips.push(s.tick);
        }
        if was && !now {
            rearms.push(s.tick);
        }
        was = now;
        assert!(!tripped(&s, TripId(1)), "outlet_high fired at {}", s.tick);
        // NaN before the first solve; `max` keeps the other operand.
        hottest_outlet_k = hottest_outlet_k.max(heater(&s).temperature_k);
    }
    assert_eq!(trips.len(), 43);
    assert_eq!(rearms.len(), 43);
    assert_eq!((trips[0], *trips.last().unwrap()), (75, 5_913));
    assert!(trips.windows(2).all(|w| w[1] - w[0] == 139), "{trips:?}");
    assert!(
        trips.iter().zip(&rearms).all(|(t, r)| r - t == 53),
        "{rearms:?}"
    );
    assert!(
        (hottest_outlet_k - 273.15 - 55.52).abs() < 0.005,
        "{}",
        hottest_outlet_k - 273.15
    );
}
