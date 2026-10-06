//! M44, docs/DESIGN.md §49: a restart asks the trips that do not hold the
//! equipment (`docs/DEFERRED.md` E28's last clause).
//!
//! Every fixture is a shipped plant with its `[[trips]]` lines edited in memory,
//! so the plant is the file's to the bit until the edit matters, and no shipped
//! file changes (CLAUDE.md: the existing files are the regression anchor).
//!
//! - `tank_overheat_trip.toml`: a furnace fired by hand at 3 MW into a tank,
//!   cut at tick 1355 by the tank's temperature trip — given `manual_restart`
//!   here, and a second trip on the same furnace, on the tank's LEVEL. Shutting
//!   the drain after the cut fills the tank into that second trip.

use refinery_core::graph::{NodeId, NodeKind, TripId};
use refinery_core::snapshot::{Command, NodeSnapshot, RestartBar, Snapshot, TripStop};
use refinery_core::Engine;
use refinery_scenarios::{build_engine, load_str};

const OVERHEAT_TRIP: &str = include_str!("../../../scenarios/tank_overheat_trip.toml");

const TEMPERATURE_TRIP: TripId = TripId(0);
const LEVEL_TRIP: TripId = TripId(1);

/// The level the second trip cuts the furnace at: above the 4.97 m the tank
/// stands at while it drains, below the ~6 m the feed's 1.6 bar can lift it to.
const LEVEL_LIMIT_M: f64 = 5.2;

/// The overheat plant with its temperature trip resetting into a restart, and
/// a high-level trip on the same furnace.
fn two_trips_on_the_furnace() -> String {
    let anchor = "limit_c = 75.0\n";
    assert_eq!(OVERHEAT_TRIP.matches(anchor).count(), 1);
    let src = OVERHEAT_TRIP.replacen(anchor, &format!("{anchor}reset = \"manual_restart\"\n"), 1);
    format!(
        "{src}
[[trips]]
name = \"tank_high_level\"
measurement = {{ node = \"hold_tank\", variable = \"level\" }}
direction = \"high\"
limit_m = {LEVEL_LIMIT_M}
actions = [{{ furnace = \"heater\" }}]
"
    )
}

fn build(src: &str) -> Engine {
    build_engine(&load_str(src).expect("the plant must parse")).expect("the plant must build")
}

fn tick(engine: &mut Engine) -> Snapshot {
    engine.tick().expect("the tick must run");
    engine.snapshot()
}

fn node_id(engine: &Engine, name: &str) -> NodeId {
    engine
        .snapshot()
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("no node '{name}'"))
        .id
}

fn heater(snapshot: &Snapshot) -> &NodeSnapshot {
    snapshot.nodes.iter().find(|n| n.name == "heater").unwrap()
}

fn duty_w(snapshot: &Snapshot) -> f64 {
    match &heater(snapshot).kind {
        NodeKind::Furnace { duty, .. } => duty.value(),
        other => panic!("heater is not a furnace: {other:?}"),
    }
}

fn level_m(engine: &Engine) -> f64 {
    match &engine.graph.node(node_id(engine, "hold_tank")).kind {
        NodeKind::Tank(t) => t.level(&engine.slate).value(),
        other => panic!("hold_tank is not a tank: {other:?}"),
    }
}

fn tripped(snapshot: &Snapshot, trip: TripId) -> bool {
    snapshot.trips[trip.0 as usize].state.is_tripped()
}

/// The overheat plant run to its temperature trip, its drain then shut, and
/// run on to the end of the tick whose solve lifts the level to the second
/// trip's limit — `stop_short` ticks before it. That tick's trip pass read the
/// level below the limit, so the level trip is still armed at its end.
fn at_the_level_crossing(stop_short: u64) -> Engine {
    let mut engine = build(&two_trips_on_the_furnace());
    while !tripped(&engine.snapshot(), TEMPERATURE_TRIP) {
        tick(&mut engine);
    }
    assert_eq!(engine.snapshot().tick, 1355, "the temperature trip's tick");
    let drain = node_id(&engine, "drain_valve");
    engine
        .apply(Command::SetValveOpening {
            node: drain,
            opening: 0.0,
        })
        .unwrap();
    let mut probe = build(&two_trips_on_the_furnace());
    let crossing = {
        while !tripped(&probe.snapshot(), TEMPERATURE_TRIP) {
            tick(&mut probe);
        }
        probe
            .apply(Command::SetValveOpening {
                node: drain,
                opening: 0.0,
            })
            .unwrap();
        while level_m(&probe) < LEVEL_LIMIT_M {
            tick(&mut probe);
        }
        probe.snapshot().tick
    };
    while engine.snapshot().tick < crossing - stop_short {
        tick(&mut engine);
    }
    engine
}

/// Gate 1 — the hole the clause named, closed. A person resets the temperature
/// trip after the solve that filled the tank past the level trip's limit and
/// before that trip's pass: the reset re-arms, the furnace stays DARK at the
/// command, and the snapshot says why. Without the check the furnace read 3 MW
/// lit here, with no `trip_stop`, and the level trip cut it again at the top
/// of the next tick — a restart that appeared to work and did not.
#[test]
fn a_reset_does_not_relight_what_another_trip_is_about_to_cut() {
    let mut engine = at_the_level_crossing(0);
    let before = engine.snapshot();
    assert_eq!(before.tick, 1407, "the tick the level crosses");
    assert!(level_m(&engine) >= LEVEL_LIMIT_M);
    assert!(
        !tripped(&before, LEVEL_TRIP),
        "its pass read the level below"
    );
    // Published before anyone acts: a reset now would not relight it.
    assert_eq!(
        heater(&before).trip_stop,
        Some(TripStop::Held {
            barred_by: vec![RestartBar::TripAboutToFire]
        })
    );

    engine
        .apply(Command::ResetTrip {
            trip_id: TEMPERATURE_TRIP,
        })
        .expect("the reset itself is the temperature trip's, and its reading is clear");
    let after = engine.snapshot();
    assert!(!tripped(&after, TEMPERATURE_TRIP), "the reset re-armed it");
    assert_eq!(duty_w(&after), 0.0, "and did not relight the furnace");
    assert_eq!(
        heater(&after).trip_stop,
        Some(TripStop::NotRestarted {
            at_tick: 1408,
            barred_by: vec![RestartBar::TripAboutToFire]
        })
    );

    // The level trip's pass: it cuts what is already dark, and the stop is its.
    let next = tick(&mut engine);
    assert!(tripped(&next, LEVEL_TRIP));
    assert_eq!(duty_w(&next), 0.0);
    assert_eq!(
        heater(&next).trip_stop,
        Some(TripStop::Held {
            barred_by: vec![RestartBar::ResetRestartsNothing]
        })
    );
}

/// Gate 2 — the check is the reading's, not the trip's existence. The same
/// reset one tick earlier, with the level still below the limit, relights the
/// furnace at its 3 MW as M40's `manual_restart` does.
#[test]
fn a_reset_before_the_reading_crosses_relights_as_before() {
    let mut engine = at_the_level_crossing(1);
    assert!(level_m(&engine) < LEVEL_LIMIT_M);
    assert_eq!(
        heater(&engine.snapshot()).trip_stop,
        Some(TripStop::Held { barred_by: vec![] })
    );
    engine
        .apply(Command::ResetTrip {
            trip_id: TEMPERATURE_TRIP,
        })
        .unwrap();
    let after = engine.snapshot();
    assert_eq!(duty_w(&after), 3.0e6);
    assert_eq!(heater(&after).trip_stop, None);
}
