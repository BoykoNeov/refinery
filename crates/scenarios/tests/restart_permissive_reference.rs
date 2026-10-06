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
//! - `furnace_restart_permissive.toml` (the M44 demo): a feed pump stopped by
//!   the tank's overfill trip, and a furnace whose self-resetting tube trip
//!   names that overfill trip as its start permissive.

use refinery_core::graph::{NodeId, NodeKind, TripId};
use refinery_core::snapshot::{Command, NodeSnapshot, RestartBar, Snapshot, TripStop};
use refinery_core::units::Watt;
use refinery_core::Engine;
use refinery_scenarios::{build_engine, load_str};

const OVERHEAT_TRIP: &str = include_str!("../../../scenarios/tank_overheat_trip.toml");
const PERMISSIVE: &str = include_str!("../../../scenarios/furnace_restart_permissive.toml");
const AUTORESET: &str = include_str!("../../../scenarios/furnace_coil_trip_autoreset.toml");

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

// ---------------------------------------------------------------------------
// M44.1 — start permissives: trips on OTHER equipment a restart waits for.
// ---------------------------------------------------------------------------

/// The demo's two trips, in file order.
const OVERFILL: TripId = TripId(0);
const TUBE_SKIN: TripId = TripId(1);

const PERMISSIVE_LINE: &str = "restart_permissives = [\"hold_tank_high_level\"]\n";

/// The demo with one line edited — `anchor`, which must be in it once.
fn demo_with(anchor: &str, replacement: &str) -> String {
    assert_eq!(PERMISSIVE.matches(anchor).count(), 1, "anchor '{anchor}'");
    PERMISSIVE.replacen(anchor, replacement, 1)
}

fn refusal(src: &str) -> String {
    match build_engine(&load_str(src).expect("the plant must parse")) {
        Ok(_) => panic!("the plant loaded and should have been refused"),
        Err(e) => e.to_string(),
    }
}

fn run_to(engine: &mut Engine, tick_index: u64) -> Snapshot {
    while engine.snapshot().tick < tick_index {
        tick(engine);
    }
    engine.snapshot()
}

fn pump_on(snapshot: &Snapshot) -> bool {
    match &snapshot
        .nodes
        .iter()
        .find(|n| n.name == "feed_pump")
        .unwrap()
        .kind
    {
        NodeKind::Pump { on, .. } => *on,
        other => panic!("feed_pump is not a pump: {other:?}"),
    }
}

/// Gate 3 — the demo, as its file says. The overfill trip stops the pump on
/// 266, the tube trip cuts the starved furnace on 299 and re-arms itself on
/// 321, and the furnace stays dark because the pump's trip is latched. A person
/// resetting that trip restarts nothing — its reset is `manual` — and the
/// heater still waits; a person starting the pump and firing the heater ends it.
#[test]
fn the_demo_waits_for_the_pump_trip_and_then_for_a_person() {
    let mut engine = build(PERMISSIVE);
    assert!(!tripped(&run_to(&mut engine, 265), OVERFILL));
    let pump_stopped = run_to(&mut engine, 266);
    assert!(tripped(&pump_stopped, OVERFILL));
    assert!(!pump_on(&pump_stopped));
    assert_eq!(
        duty_w(&pump_stopped),
        3.0e6,
        "the pump's trip leaves the heater lit"
    );

    assert!(!tripped(&run_to(&mut engine, 298), TUBE_SKIN));
    let cut = run_to(&mut engine, 299);
    assert!(tripped(&cut, TUBE_SKIN));
    assert_eq!(duty_w(&cut), 0.0);
    assert_eq!(
        heater(&cut).trip_stop,
        Some(TripStop::Held {
            barred_by: vec![RestartBar::PermissiveNotClear]
        }),
        "published from the cut: its reset will not relight it while the pump's trip is latched"
    );

    assert!(tripped(&run_to(&mut engine, 320), TUBE_SKIN));
    let rearmed = run_to(&mut engine, 321);
    assert!(!tripped(&rearmed, TUBE_SKIN), "the tube trip resets itself");
    assert_eq!(duty_w(&rearmed), 0.0, "and leaves the furnace dark");
    let waiting = Some(TripStop::NotRestarted {
        at_tick: 321,
        barred_by: vec![RestartBar::PermissiveNotClear],
    });
    assert_eq!(heater(&rearmed).trip_stop, waiting);

    let later = run_to(&mut engine, 1000);
    assert_eq!(duty_w(&later), 0.0);
    assert_eq!(heater(&later).trip_stop, waiting);

    engine
        .apply(Command::ResetTrip { trip_id: OVERFILL })
        .unwrap();
    let pump_trip_reset = engine.snapshot();
    assert!(
        !pump_on(&pump_trip_reset),
        "a manual reset restarts nothing"
    );
    assert_eq!(duty_w(&pump_trip_reset), 0.0);
    assert_eq!(
        heater(&pump_trip_reset).trip_stop,
        waiting,
        "the permissive clearing later does not relight it: the stop ended at 321"
    );

    let (pump, furnace) = (node_id(&engine, "feed_pump"), node_id(&engine, "heater"));
    engine
        .apply(Command::SetPumpOn {
            node: pump,
            on: true,
        })
        .unwrap();
    engine
        .apply(Command::SetFurnaceDuty {
            node: furnace,
            duty: Watt(3.0e6),
        })
        .unwrap();
    assert_eq!(heater(&engine.snapshot()).trip_stop, None);
    let relit = run_to(&mut engine, 1010);
    assert_eq!(duty_w(&relit), 3.0e6);
    assert!(pump_on(&relit));
    assert_eq!(heater(&relit).trip_stop, None);
}

/// Gate 4 — what the permissive prevents: the same file without the line
/// relights at 321 into the starved flow and cuts again at 340.
#[test]
fn without_the_permissive_the_tube_trip_relights_into_the_starved_flow() {
    let mut engine = build(&demo_with(PERMISSIVE_LINE, ""));
    let relit = run_to(&mut engine, 321);
    assert!(tripped(&relit, OVERFILL), "the pump is still stopped");
    assert_eq!(duty_w(&relit), 3.0e6);
    assert_eq!(heater(&relit).trip_stop, None);
    assert!(!tripped(&run_to(&mut engine, 339), TUBE_SKIN));
    assert!(tripped(&run_to(&mut engine, 340), TUBE_SKIN));
}

/// Gate 5 — a permissive is asked when the stop ENDS, not when it began: a
/// person resets the pump's trip at 310, while the tube trip still holds the
/// furnace, and the tube trip's own reset at 321 relights it (the pump stays
/// off: that reset was a person's, and restarts nothing).
#[test]
fn a_permissive_cleared_before_the_stop_ends_lets_the_restart_through() {
    let mut engine = build(PERMISSIVE);
    run_to(&mut engine, 310);
    engine
        .apply(Command::ResetTrip { trip_id: OVERFILL })
        .expect("the level has fallen under the overfill limit");
    assert_eq!(
        heater(&engine.snapshot()).trip_stop,
        Some(TripStop::Held { barred_by: vec![] })
    );
    let relit = run_to(&mut engine, 321);
    assert!(!tripped(&relit, TUBE_SKIN));
    assert_eq!(duty_w(&relit), 3.0e6);
    assert!(!pump_on(&relit));
    assert_eq!(heater(&relit).trip_stop, None);
}

/// Gate 6 — "clear" is armed AND outside its condition, read fresh, and only
/// the trips named are asked. The tube trip is pressed by hand at 100 while the
/// pump still fills the tank: the stop is barred by the press alone until the
/// tick whose solve lifts the level to 7.0 m, when the overfill trip — still
/// armed, its pass having read the level below — is no longer clear. Nothing
/// else is added: the overfill trip acts on the pump, not on the furnace.
#[test]
fn an_armed_permissive_past_its_limit_is_not_clear() {
    let mut engine = build(PERMISSIVE);
    run_to(&mut engine, 100);
    engine
        .apply(Command::ManualTrip { trip_id: TUBE_SKIN })
        .unwrap();
    let pressed_only = Some(TripStop::Held {
        barred_by: vec![RestartBar::PressedByHand],
    });
    assert_eq!(heater(&engine.snapshot()).trip_stop, pressed_only);
    loop {
        let snapshot = tick(&mut engine);
        if level_m(&engine) >= 7.0 {
            assert!(
                !tripped(&snapshot, OVERFILL),
                "its pass read the level below"
            );
            break;
        }
        assert_eq!(
            heater(&snapshot).trip_stop,
            pressed_only,
            "tick {}",
            snapshot.tick
        );
        assert!(snapshot.tick < 400, "the pump never filled the tank to 7 m");
    }
    let both = Some(TripStop::Held {
        barred_by: vec![RestartBar::PressedByHand, RestartBar::PermissiveNotClear],
    });
    assert_eq!(heater(&engine.snapshot()).trip_stop, both);
    let latched = tick(&mut engine);
    assert!(tripped(&latched, OVERFILL));
    assert_eq!(heater(&latched).trip_stop, both);
}

/// Gate 7 — the loader. A name may point forward; the list is refused where it
/// would mean nothing or say nothing.
#[test]
fn the_loader_resolves_permissives_and_refuses_empty_ones() {
    // Forward: the overfill trip moved after the tube trip that names it.
    let split = PERMISSIVE.find("# The tank's overfill").unwrap();
    let (head, both_trips) = PERMISSIVE.split_at(split);
    let (overfill, tube) = both_trips.split_at(both_trips.find("# The tubes' protection").unwrap());
    let forward = build(&format!("{head}{tube}\n{overfill}")).snapshot();
    assert_eq!(forward.trips[0].name, "tube_skin_high");
    assert_eq!(forward.trips[0].restart_permissives, vec![TripId(1)]);

    let same_equipment = format!(
        "{}
[[trips]]
name = \"outlet_high\"
measurement = {{ node = \"heater\", variable = \"temperature\" }}
direction = \"high\"
limit_c = 95.0
actions = [{{ furnace = \"heater\" }}]
",
        demo_with(
            PERMISSIVE_LINE,
            "restart_permissives = [\"hold_tank_high_level\", \"outlet_high\"]\n"
        )
    );
    for (src, expected) in [
        (
            demo_with("reset = \"auto\"\nreset_limit_c = 80.0\n", ""),
            "its reset restarts nothing",
        ),
        (
            demo_with(
                PERMISSIVE_LINE,
                "restart_permissives = [\"no_such_trip\"]\n",
            ),
            "no trip on this plant is called that",
        ),
        (
            demo_with(
                PERMISSIVE_LINE,
                "restart_permissives = [\"tube_skin_high\"]\n",
            ),
            "names itself",
        ),
        (
            demo_with(
                PERMISSIVE_LINE,
                "restart_permissives = [\"hold_tank_high_level\", \"hold_tank_high_level\"]\n",
            ),
            "twice",
        ),
        (same_equipment, "both act on 'heater'"),
    ] {
        let message = refusal(&src);
        assert!(
            message.contains(expected),
            "expected '{expected}' in: {message}"
        );
    }
}

/// Gate 8 — the wire. The tube trip writes its permissives as ids into the
/// snapshot's own `trips`; a trip with none, and every plant before M44, writes
/// no key. The two new reasons have their snake-case names.
#[test]
fn the_snapshot_carries_the_permissives_and_the_new_reasons() {
    let mut engine = build(PERMISSIVE);
    let at_load = serde_json::to_string(&engine.snapshot()).unwrap();
    assert_eq!(at_load.matches("\"restart_permissives\"").count(), 1);
    assert!(at_load.contains("\"restart_permissives\":[0]"));
    assert!(!serde_json::to_string(&build(AUTORESET).snapshot())
        .unwrap()
        .contains("restart_permissives"));

    run_to(&mut engine, 321);
    let waiting = serde_json::to_string(&engine.snapshot()).unwrap();
    assert!(waiting.contains(
        "\"trip_stop\":{\"status\":\"not_restarted\",\"at_tick\":321,\"barred_by\":[\"permissive_not_clear\"]}"
    ));

    let crossing = at_the_level_crossing(0);
    let about_to_fire = serde_json::to_string(&crossing.snapshot()).unwrap();
    assert!(about_to_fire.contains("\"barred_by\":[\"trip_about_to_fire\"]"));
}
