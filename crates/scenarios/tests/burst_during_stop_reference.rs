//! M42, docs/DESIGN.md §47: a burst during a stop makes its restart a person's
//! (`docs/DEFERRED.md` E28's replaced-tubes clause).
//!
//! M41 refused a trip's restart onto tubes that ARE burst. It asked the tubes in
//! place, so tubes that burst while a trip held the furnace dark and were
//! replaced before it let go were relit by an `auto` or `manual_restart` trip.
//! Now the burst marks the stop itself: the trip still re-arms, and the furnace
//! stays dark, its loop in MANUAL, for a person — new tubes or not. The mark
//! is the stop's, so a burst on a LIT furnace marks nothing, and the next stop
//! restarts as M40 built it.
//!
//! Every fixture is a shipped plant edited in memory (CLAUDE.md: the existing
//! files are the regression anchor), the two of M41's file:
//!
//! - `furnace_coil_trip.toml` with its tubes' limit lowered from 550 °C to
//!   100.1 °C, so the coil that ends tick 74 at 100.18 °C bursts them on tick
//!   75, the tick the 100 °C tube-skin trip cuts the fuel. The dark coil is
//!   under 100.1 °C from that tick on, so new tubes fit while the trip holds.
//! - `tank_overheat_trip.toml` with its tubes' limit lowered to 100 °C (the
//!   coil runs at 95.48 °C), its trip restarting on a person's reset, and a
//!   fire put on the LIT furnace to burst its tubes between two stops.

use refinery_core::graph::{ControlMode, EdgeId, LoopId, NodeId, NodeKind, TripId, TubeState};
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::units::{SquareMeter, Watt};
use refinery_core::Engine;
use refinery_scenarios::{build_engine, load_str};

const COIL_TRIP: &str = include_str!("../../../scenarios/furnace_coil_trip.toml");
const OVERHEAT_TRIP: &str = include_str!("../../../scenarios/tank_overheat_trip.toml");

const TUBE_SKIN: TripId = TripId(0);
const OUTLET_LOOP: LoopId = LoopId(0);
const TANK_HIGH: TripId = TripId(0);

/// Replace the one line `from` in `src` with `to`.
fn replace(src: &str, from: &str, to: &str) -> String {
    assert_eq!(src.matches(from).count(), 1, "'{from}' is not unique");
    src.replacen(from, to, 1)
}

/// The coil plant with `tube_skin_high`'s reset declared and its tubes' limit
/// at `failure_c`.
fn coil_plant(reset: &str, failure_c: &str) -> String {
    let src = replace(
        COIL_TRIP,
        "limit_c = 100.0\n",
        &format!("limit_c = 100.0\n{reset}"),
    );
    replace(
        &src,
        "tube_failure_c = 550.0\n",
        &format!("tube_failure_c = {failure_c}\n"),
    )
}

/// The overheat plant with a restarting reset and tubes that fail at 100 °C.
fn overheat_plant() -> String {
    let src = replace(
        OVERHEAT_TRIP,
        "limit_c = 75.0\n",
        "limit_c = 75.0\nreset = \"manual_restart\"\n",
    );
    replace(&src, "tube_failure_c = 550.0\n", "tube_failure_c = 100.0\n")
}

fn build(src: &str) -> Engine {
    build_engine(&load_str(src).expect("the plant must parse")).expect("the plant must build")
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

fn heater_kind(snapshot: &Snapshot) -> &NodeKind {
    &snapshot
        .nodes
        .iter()
        .find(|n| n.name == "heater")
        .unwrap()
        .kind
}

fn heater_id(snapshot: &Snapshot) -> NodeId {
    snapshot
        .nodes
        .iter()
        .find(|n| n.name == "heater")
        .unwrap()
        .id
}

fn duty_w(snapshot: &Snapshot) -> f64 {
    match heater_kind(snapshot) {
        NodeKind::Furnace { duty, .. } => duty.value(),
        other => panic!("heater is not a furnace: {other:?}"),
    }
}

fn coil_c(snapshot: &Snapshot) -> f64 {
    match heater_kind(snapshot) {
        NodeKind::Furnace { coil, .. } => coil.temperature.value() - 273.15,
        other => panic!("heater is not a furnace: {other:?}"),
    }
}

fn tubes(snapshot: &Snapshot) -> TubeState {
    match heater_kind(snapshot) {
        NodeKind::Furnace { tubes, .. } => tubes.state,
        other => panic!("heater is not a furnace: {other:?}"),
    }
}

fn loop_mode(snapshot: &Snapshot) -> ControlMode {
    snapshot.controls[OUTLET_LOOP.0 as usize].mode
}

fn tripped(snapshot: &Snapshot, trip: TripId) -> bool {
    snapshot.trips[trip.0 as usize].state.is_tripped()
}

fn tank_c(snapshot: &Snapshot) -> f64 {
    snapshot.trips[TANK_HIGH.0 as usize]
        .measurement
        .expect("the tank is measured once the plant has solved")
        .magnitude()
        - 273.15
}

fn edge_id(snapshot: &Snapshot, name: &str) -> EdgeId {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("no pipe '{name}'"))
        .id
}

/// Patch the burn-out hole on `heated_line`, then fit new tubes: the player's
/// order (M37, §42).
fn patch_and_replace(engine: &mut Engine) {
    let now = engine.snapshot();
    engine
        .apply(Command::PuncturePipe {
            edge: edge_id(&now, "heated_line"),
            area: SquareMeter(0.0),
        })
        .expect("the hole is patched");
    engine
        .apply(Command::ReplaceTubes {
            node: heater_id(&now),
        })
        .expect("patched hole, coil under its limit: new tubes fit");
    assert_eq!(tubes(&engine.snapshot()), TubeState::Intact);
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

/// Run until `trip` has tripped and the tank and coil have cooled enough for
/// a reset that relights (tank 5 K under its 75 °C limit, coil under 90 °C).
fn run_until_clear(engine: &mut Engine, trip: TripId, limit: u64) -> u64 {
    let mut seen_trip = tripped(&engine.snapshot(), trip);
    loop {
        let s = tick(engine);
        seen_trip |= tripped(&s, trip);
        if seen_trip && tank_c(&s) < 70.0 && coil_c(&s) < 90.0 {
            return s.tick;
        }
        assert!(s.tick < limit, "the tank never cleared by tick {limit}");
    }
}

/// Gate 1 — a self-resetting trip whose furnace burst its tubes while it held
/// it, and got new ones before it let go, re-arms and relights nothing. The
/// tubes are INTACT at the release, so M41's rule alone would have relit them:
/// only the stop's mark keeps the furnace dark. The twin on 550 °C tubes, the
/// same plant with no burst, relights on the same tick.
#[test]
fn a_burst_during_the_stop_outlives_new_tubes_on_a_self_reset() {
    let reset = "reset = \"auto\"\nreset_limit_c = 80.0\n";
    let mut burst = build(&coil_plant(reset, "100.1"));
    let mut intact = build(&coil_plant(reset, "550.0"));

    let at_cut = run_to(&mut burst, 75);
    assert!(tripped(&at_cut, TUBE_SKIN), "the trip cuts on tick 75");
    assert_eq!(tubes(&at_cut), TubeState::Failed { at_tick: 75 });

    // New tubes while the trip still holds the furnace dark.
    let at_fit = run_to(&mut burst, 80);
    assert!(tripped(&at_fit, TUBE_SKIN));
    assert!(coil_c(&at_fit) < 100.1, "{}", coil_c(&at_fit));
    patch_and_replace(&mut burst);

    let rearm = rearm_tick(&mut burst, TUBE_SKIN, 1_000);
    let intact_rearm = rearm_tick(&mut intact, TUBE_SKIN, 1_000);
    assert_eq!(
        rearm, intact_rearm,
        "the tubes move no reading the trip compares"
    );
    assert_eq!(rearm, 128, "measured: the re-arm tick M40's gate 3 pins");

    let (dark, lit) = (burst.snapshot(), intact.snapshot());
    assert_eq!(
        tubes(&dark),
        TubeState::Intact,
        "M41's rule sees nothing to refuse"
    );
    assert_eq!(loop_mode(&lit), ControlMode::Auto, "no burst: relit");
    assert_eq!(
        loop_mode(&dark),
        ControlMode::Manual,
        "a burst in the stop: not"
    );
    assert_eq!(duty_w(&dark), 0.0);

    // And for good: the record is dropped, so nothing relights it later.
    let later = run_to(&mut burst, 600);
    assert!(!tripped(&later, TUBE_SKIN));
    assert_eq!(loop_mode(&later), ControlMode::Manual);
    assert_eq!(duty_w(&later), 0.0);
}

/// Gate 2 — the same through a person's restarting reset (`ResetTrip`, a
/// different road to the release than the trip pass). New tubes, patched hole,
/// the reset re-arms the trip and relights nothing. A person's AUTO then
/// relights it, so the furnace is left dark, not bricked.
#[test]
fn a_burst_during_the_stop_outlives_new_tubes_on_a_persons_reset() {
    let mut engine = build(&coil_plant("reset = \"manual_restart\"\n", "100.1"));
    let at = run_to(&mut engine, 150);
    assert!(tripped(&at, TUBE_SKIN) && tubes(&at).is_failed());
    patch_and_replace(&mut engine);
    assert!(
        tripped(&engine.snapshot(), TUBE_SKIN),
        "new tubes reset no trip"
    );

    engine
        .apply(Command::ResetTrip { trip_id: TUBE_SKIN })
        .expect("the reset itself is not refused");
    let now = engine.snapshot();
    assert!(!tripped(&now, TUBE_SKIN), "the trip re-armed");
    assert_eq!(tubes(&now), TubeState::Intact);
    assert_eq!(loop_mode(&now), ControlMode::Manual, "and relit nothing");
    let after = run_to(&mut engine, 200);
    assert_eq!(loop_mode(&after), ControlMode::Manual);
    assert_eq!(duty_w(&after), 0.0);

    engine
        .apply(Command::SetControllerMode {
            loop_id: OUTLET_LOOP,
            mode: ControlMode::Auto,
        })
        .expect("a person puts the loop back in AUTO");
    let relit = run_to(&mut engine, 260);
    assert!(duty_w(&relit) > 0.0, "the loop fires the new tubes");
}

/// Gate 3 — tubes that burst after a restart, on a lit furnace no trip holds,
/// mark no stop: the next one restarts as M40 built it. Stop one clears and a
/// person's reset hands back the 3 MW; a fire then bursts the lit tubes; the
/// hole is patched and new tubes fitted with the trip armed; the tank trips
/// again, clears, and the reset hands back the 3 MW again.
#[test]
fn a_burst_after_a_restart_does_not_mark_the_next_stop() {
    let mut engine = build(&overheat_plant());

    let clear = run_until_clear(&mut engine, TANK_HIGH, 20_000);
    engine
        .apply(Command::ResetTrip { trip_id: TANK_HIGH })
        .expect("the tank is under its limit");
    assert_eq!(duty_w(&engine.snapshot()), 3.0e6, "stop one: relit");

    // Burst the lit tubes with a fire, then put it out.
    run_to(&mut engine, clear + 20);
    let heater = heater_id(&engine.snapshot());
    engine
        .apply(Command::SetHeatInput {
            node: heater,
            power: Watt(50.0e6),
        })
        .unwrap();
    let burst = loop {
        let s = tick(&mut engine);
        assert!(!tripped(&s, TANK_HIGH), "the burst comes between the stops");
        if let TubeState::Failed { at_tick } = tubes(&s) {
            break at_tick;
        }
    };
    engine
        .apply(Command::SetHeatInput {
            node: heater,
            power: Watt(0.0),
        })
        .unwrap();
    // The lit coil cools under its limit, the trip still armed, and is refitted.
    loop {
        let s = tick(&mut engine);
        assert!(!tripped(&s, TANK_HIGH));
        if coil_c(&s) < 100.0 {
            break;
        }
    }
    let fitted = engine.snapshot().tick;
    patch_and_replace(&mut engine);

    let retrip = loop {
        let s = tick(&mut engine);
        if tripped(&s, TANK_HIGH) {
            break s.tick;
        }
        assert!(s.tick < 20_000, "the tank never tripped again");
    };
    let clear_again = run_until_clear(&mut engine, TANK_HIGH, 20_000);
    assert_eq!(
        (clear, burst, fitted, retrip, clear_again),
        (1_594, 1_618, 1_717, 1_907, 2_145),
        "measured: stop one clears, the fire bursts the lit tubes, new tubes, stop two"
    );

    engine
        .apply(Command::ResetTrip { trip_id: TANK_HIGH })
        .expect("the tank is under its limit");
    let now = engine.snapshot();
    assert_eq!(tubes(&now), TubeState::Intact);
    assert_eq!(
        duty_w(&now),
        3.0e6,
        "stop two: relit, the earlier burst marks nothing"
    );
}

/// Gate 4 — a person's restarting reset on a stop that saw no burst still
/// relights, on the coil plant's own 550 °C tubes: the loop takes the furnace
/// back in AUTO and fires it.
#[test]
fn a_stop_without_a_burst_still_relights() {
    let mut engine = build(&coil_plant("reset = \"manual_restart\"\n", "550.0"));
    let at = run_to(&mut engine, 150);
    assert!(tripped(&at, TUBE_SKIN));
    assert_eq!(tubes(&at), TubeState::Intact);
    engine
        .apply(Command::ResetTrip { trip_id: TUBE_SKIN })
        .expect("the coil is under its limit");
    let now = engine.snapshot();
    assert!(!tripped(&now, TUBE_SKIN));
    assert_eq!(
        loop_mode(&now),
        ControlMode::Auto,
        "relit through the transfer"
    );
    // Ten ticks on, before the fouled coil climbs back to its trip (M40's cycle).
    let relit = run_to(&mut engine, 160);
    assert!(!tripped(&relit, TUBE_SKIN));
    assert_eq!(loop_mode(&relit), ControlMode::Auto);
    assert!(duty_w(&relit) > 0.0, "and the loop fires");
}
