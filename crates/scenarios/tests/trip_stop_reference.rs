//! M43, docs/DESIGN.md §48: the snapshot says why a stop by the trips did not,
//! or will not, end in the trips handing the equipment back
//! (`NodeSnapshot::trip_stop`).
//!
//! M42 made a burst during a stop keep the furnace dark for a person, new tubes
//! or not. The engine knew it in a private record, dropped the moment the trip
//! let go — so a frontend saw new tubes, a cleared trip and a dark furnace, with
//! nothing saying why. `trip_stop` publishes the record while a trip holds the
//! equipment (`held`, with what would bar a restart now) and keeps the reason
//! after the last lets go without a restart (`not_restarted`), until a person
//! restarts the equipment or the next stop begins.
//!
//! Fixtures: the new `furnace_burst_during_stop.toml` and shipped plants, some
//! edited in memory (CLAUDE.md: the existing files are the regression anchor).

use refinery_core::graph::{ControlMode, ControlledValue, LoopId, NodeId, TripId};
use refinery_core::snapshot::{Command, RestartBar, Snapshot, TripStop};
use refinery_core::units::{Kelvin, SquareMeter, Watt};
use refinery_core::Engine;
use refinery_scenarios::{build_engine, load_str};

const BURST_DURING_STOP: &str = include_str!("../../../scenarios/furnace_burst_during_stop.toml");
const COIL_TRIP: &str = include_str!("../../../scenarios/furnace_coil_trip.toml");
const AUTORESET: &str = include_str!("../../../scenarios/furnace_coil_trip_autoreset.toml");
const OVERFILL: &str = include_str!("../../../scenarios/tank_overfill_trip.toml");

const TUBE_SKIN: TripId = TripId(0);
const OUTLET_LOOP: LoopId = LoopId(0);
const HIGH_LEVEL: TripId = TripId(0);

use RestartBar::{PressedByHand, ResetRestartsNothing, TubesBurst, TubesBurstDuringStop};

fn build(src: &str) -> Engine {
    build_engine(&load_str(src).expect("the plant must parse")).expect("the plant must build")
}

fn run_to(engine: &mut Engine, tick_index: u64) -> Snapshot {
    while engine.snapshot().tick < tick_index {
        engine.tick().expect("the tick must run");
    }
    engine.snapshot()
}

fn node_id(snapshot: &Snapshot, name: &str) -> NodeId {
    snapshot
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("no node '{name}'"))
        .id
}

fn stop_of(snapshot: &Snapshot, name: &str) -> Option<TripStop> {
    snapshot.nodes[node_id(snapshot, name).0 as usize]
        .trip_stop
        .clone()
}

fn held(barred_by: &[RestartBar]) -> Option<TripStop> {
    Some(TripStop::Held {
        barred_by: barred_by.to_vec(),
    })
}

fn not_restarted(at_tick: u64, barred_by: &[RestartBar]) -> Option<TripStop> {
    Some(TripStop::NotRestarted {
        at_tick,
        barred_by: barred_by.to_vec(),
    })
}

fn duty_w(snapshot: &Snapshot) -> f64 {
    match &snapshot.nodes[node_id(snapshot, "heater").0 as usize].kind {
        refinery_core::graph::NodeKind::Furnace { duty, .. } => duty.value(),
        other => panic!("heater is not a furnace: {other:?}"),
    }
}

fn apply(engine: &mut Engine, command: Command) {
    engine.apply(command).expect("the command must be accepted");
}

/// Gate 1 — the story the furnace screen tells (key 4). The tubes burst on the
/// trip's own tick 75; new tubes at 80 lift `tubes_burst` and leave
/// `tubes_burst_during_stop`; the trip re-arms by itself at 128 and the furnace
/// stays dark, its snapshot saying why on its own; a person's relight ends it
/// at the command.
#[test]
fn a_burst_during_the_stop_is_published_as_the_reason_the_furnace_stays_dark() {
    let mut engine = build(BURST_DURING_STOP);
    assert_eq!(stop_of(&run_to(&mut engine, 74), "heater"), None);

    let snap = run_to(&mut engine, 75);
    assert_eq!(
        stop_of(&snap, "heater"),
        held(&[TubesBurstDuringStop, TubesBurst]),
        "burst in the trip's own tick, so during the stop, and burst in place"
    );

    let snap = run_to(&mut engine, 80);
    let heated_line = snap
        .edges
        .iter()
        .find(|e| e.name == "heated_line")
        .unwrap()
        .id;
    let heater = node_id(&snap, "heater");
    apply(
        &mut engine,
        Command::PuncturePipe {
            edge: heated_line,
            area: SquareMeter(0.0),
        },
    );
    apply(&mut engine, Command::ReplaceTubes { node: heater });
    assert_eq!(
        stop_of(&engine.snapshot(), "heater"),
        held(&[TubesBurstDuringStop]),
        "new tubes lift the burst in place, never the burst during the stop"
    );

    let snap = run_to(&mut engine, 127);
    assert!(snap.trips[TUBE_SKIN.0 as usize].state.is_tripped());
    let snap = run_to(&mut engine, 128);
    assert!(!snap.trips[TUBE_SKIN.0 as usize].state.is_tripped());
    assert_eq!(duty_w(&snap), 0.0);
    assert_eq!(
        snap.controls[OUTLET_LOOP.0 as usize].mode,
        ControlMode::Manual
    );
    // The snapshot one tick after the release says why, on its own.
    assert_eq!(
        stop_of(&snap, "heater"),
        not_restarted(128, &[TubesBurstDuringStop])
    );
    assert_eq!(
        stop_of(&run_to(&mut engine, 199), "heater"),
        not_restarted(128, &[TubesBurstDuringStop])
    );

    // A person relights it on a 50 °C target: the reason is gone at once.
    apply(
        &mut engine,
        Command::SetSetpoint {
            loop_id: OUTLET_LOOP,
            value: ControlledValue::Temperature { k: Kelvin(323.15) },
        },
    );
    apply(
        &mut engine,
        Command::SetControllerMode {
            loop_id: OUTLET_LOOP,
            mode: ControlMode::Auto,
        },
    );
    assert_eq!(stop_of(&engine.snapshot(), "heater"), None);
    let snap = run_to(&mut engine, 300);
    assert_eq!(stop_of(&snap, "heater"), None);
    assert!(duty_w(&snap) > 0.0);
}

/// Gate 2 — the default trip, reset by a person: `reset_restarts_nothing` while
/// it holds and after, from the tick after the reset; the next stop replaces
/// the reason (a press), and a person's restart ends it — for good, so cutting
/// the furnace by hand later does not bring it back.
#[test]
fn a_manual_trip_says_a_person_restarts_and_the_next_stop_replaces_it() {
    let mut engine = build(COIL_TRIP);
    let snap = run_to(&mut engine, 75);
    assert_eq!(stop_of(&snap, "heater"), held(&[ResetRestartsNothing]));

    run_to(&mut engine, 150);
    apply(&mut engine, Command::ResetTrip { trip_id: TUBE_SKIN });
    assert_eq!(
        stop_of(&engine.snapshot(), "heater"),
        not_restarted(151, &[ResetRestartsNothing]),
        "the first tick to run let go is the next one"
    );
    assert_eq!(
        stop_of(&run_to(&mut engine, 160), "heater"),
        not_restarted(151, &[ResetRestartsNothing])
    );

    // The next stop is a press: `held` again, with the press's reason only.
    apply(&mut engine, Command::ManualTrip { trip_id: TUBE_SKIN });
    assert_eq!(
        stop_of(&engine.snapshot(), "heater"),
        held(&[PressedByHand])
    );
    run_to(&mut engine, 170);
    apply(&mut engine, Command::ResetTrip { trip_id: TUBE_SKIN });
    assert_eq!(
        stop_of(&run_to(&mut engine, 180), "heater"),
        not_restarted(171, &[PressedByHand])
    );

    // A person's AUTO restarts it; then a person cuts it by hand.
    apply(
        &mut engine,
        Command::SetControllerMode {
            loop_id: OUTLET_LOOP,
            mode: ControlMode::Auto,
        },
    );
    run_to(&mut engine, 185);
    let heater = node_id(&engine.snapshot(), "heater");
    apply(
        &mut engine,
        Command::SetControllerMode {
            loop_id: OUTLET_LOOP,
            mode: ControlMode::Manual,
        },
    );
    apply(
        &mut engine,
        Command::SetFurnaceDuty {
            node: heater,
            duty: Watt(0.0),
        },
    );
    assert_eq!(stop_of(&run_to(&mut engine, 190), "heater"), None);
}

/// Gate 3 — a self-resetting trip that hands back: `held` with nothing in the
/// way, and nothing at all once it has relit.
#[test]
fn a_stop_the_trips_will_hand_back_says_nothing_bars_it() {
    let mut engine = build(AUTORESET);
    for tick in [75, 127] {
        assert_eq!(stop_of(&run_to(&mut engine, tick), "heater"), held(&[]));
    }
    let snap = run_to(&mut engine, 128);
    assert_eq!(stop_of(&snap, "heater"), None);
    assert_eq!(
        snap.controls[OUTLET_LOOP.0 as usize].mode,
        ControlMode::Auto
    );
}

/// Gate 4 — what `held` says is what the release then does. On every tick
/// whose snapshot shows `held` and whose next shows the trips let go: nothing
/// in the way exactly when the equipment was handed back, and otherwise the
/// same reasons as `not_restarted`, at that tick. 43 restarts on the self-reset
/// plant, a refused one on the burst plant.
#[test]
fn the_published_verdict_is_what_the_release_does() {
    let mut releases = (0, 0);
    for (src, ticks) in [(AUTORESET, 6_000), (BURST_DURING_STOP, 400)] {
        let mut engine = build(src);
        let mut before = engine.snapshot();
        while before.tick < ticks {
            engine.tick().expect("the tick must run");
            let after = engine.snapshot();
            if let Some(TripStop::Held { barred_by }) = stop_of(&before, "heater") {
                match stop_of(&after, "heater") {
                    Some(TripStop::Held { .. }) => {}
                    None => {
                        assert!(barred_by.is_empty(), "tick {}", after.tick);
                        releases.0 += 1;
                    }
                    Some(TripStop::NotRestarted {
                        at_tick,
                        barred_by: why,
                    }) => {
                        assert!(!barred_by.is_empty(), "tick {}", after.tick);
                        assert_eq!((at_tick, why), (after.tick, barred_by));
                        releases.1 += 1;
                    }
                }
            }
            before = after;
        }
    }
    assert_eq!(releases, (43, 1));
}

/// Gate 5 — a pump and a valve: one trip stops both, a person's reset leaves
/// both `not_restarted`, and each ends when ITS equipment leaves the safe state
/// the trip wrote, whatever the other does.
#[test]
fn a_pump_and_a_valve_each_wait_for_their_own_restart() {
    let mut engine = build(OVERFILL);
    let snap = run_to(&mut engine, 1_236);
    assert!(snap.trips[HIGH_LEVEL.0 as usize].state.is_tripped());
    for name in ["transfer_pump", "discharge_valve"] {
        assert_eq!(
            stop_of(&snap, name),
            held(&[ResetRestartsNothing]),
            "{name}"
        );
    }
    assert_eq!(stop_of(&snap, "level_valve"), None, "no trip acts on it");

    run_to(&mut engine, 2_000);
    apply(
        &mut engine,
        Command::ResetTrip {
            trip_id: HIGH_LEVEL,
        },
    );
    let snap = run_to(&mut engine, 2_001);
    for name in ["transfer_pump", "discharge_valve"] {
        assert_eq!(
            stop_of(&snap, name),
            not_restarted(2_001, &[ResetRestartsNothing]),
            "{name}"
        );
    }

    let pump = node_id(&snap, "transfer_pump");
    let valve = node_id(&snap, "discharge_valve");
    apply(
        &mut engine,
        Command::SetPumpOn {
            node: pump,
            on: true,
        },
    );
    // Written again at the trip's own position: still where the trip left it.
    apply(
        &mut engine,
        Command::SetValveOpening {
            node: valve,
            opening: 0.0,
        },
    );
    let snap = run_to(&mut engine, 2_002);
    assert_eq!(stop_of(&snap, "transfer_pump"), None);
    assert_eq!(
        stop_of(&snap, "discharge_valve"),
        not_restarted(2_001, &[ResetRestartsNothing])
    );
    apply(
        &mut engine,
        Command::SetValveOpening {
            node: valve,
            opening: 0.5,
        },
    );
    assert_eq!(
        stop_of(&run_to(&mut engine, 2_003), "discharge_valve"),
        None
    );
}

/// Gate 6 — the wire: a plant whose trips have not fired writes no `trip_stop`
/// key at all, and one that has writes the tagged form.
#[test]
fn the_field_is_written_only_when_the_trips_have_something_to_say() {
    let mut engine = build(BURST_DURING_STOP);
    let quiet = serde_json::to_string(&run_to(&mut engine, 74)).unwrap();
    assert!(!quiet.contains("trip_stop"));
    let held = serde_json::to_value(run_to(&mut engine, 75)).unwrap();
    let heater = held["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .find(|n| n["name"] == "heater")
        .unwrap();
    assert_eq!(
        heater["trip_stop"],
        serde_json::json!({
            "status": "held",
            "barred_by": ["tubes_burst_during_stop", "tubes_burst"],
        })
    );
}
