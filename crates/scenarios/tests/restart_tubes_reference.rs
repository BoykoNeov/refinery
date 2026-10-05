//! M41, docs/DESIGN.md §46: no restart onto burst tubes (`docs/DEFERRED.md`
//! E28's tube clause).
//!
//! A trip whose reset restarts (M40, §45) hands a furnace back when its OWN
//! reading clears. Before M41 it did so whatever had happened to the tubes in
//! the meantime, and relit a burst coil. Now the trip still re-arms — the
//! refusal is of the restart, not of the reset — and the furnace stays dark,
//! its loop in MANUAL, for a person.
//!
//! Every fixture is a shipped plant edited in memory (CLAUDE.md: the existing
//! files are the regression anchor):
//!
//! - `furnace_coil_trip.toml` with its tubes' limit lowered from 550 °C to
//!   100.1 °C, so the coil that ends tick 74 at 100.18 °C bursts them on tick
//!   75, the tick the 100 °C tube-skin trip cuts the fuel. Water does not burn,
//!   so the burst leaks and lights nothing, and the dark coil cools.
//! - `tank_overheat_trip.toml` with its tubes' limit lowered to 100 °C (the
//!   coil runs at 95.48 °C), and a fire put on the dark furnace: the coil
//!   crosses the limit BETWEEN ticks with the tubes still intact, the one case
//!   where "will burst on the next tick" is not "has burst".

use refinery_core::graph::{ControlMode, EdgeId, LoopId, NodeId, NodeKind, TripId, TubeState};
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::units::{SquareMeter, Watt};
use refinery_core::Engine;
use refinery_scenarios::{build_engine, load_str};

const COIL_TRIP: &str = include_str!("../../../scenarios/furnace_coil_trip.toml");
const OVERHEAT_TRIP: &str = include_str!("../../../scenarios/tank_overheat_trip.toml");

const TUBE_SKIN: TripId = TripId(0);
const OUTLET_LOOP: LoopId = LoopId(0);

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

fn edge_id(snapshot: &Snapshot, name: &str) -> EdgeId {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("no pipe '{name}'"))
        .id
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

/// Gate 1 — a self-resetting trip on burst tubes re-arms and relights nothing;
/// the same plant on intact tubes relights on the same tick. The pair is the
/// rule: the tubes are the only difference, and the burst leaves the coil's
/// cooling, the trip's reading and so its re-arm tick exactly where they were.
#[test]
fn a_self_reset_does_not_relight_burst_tubes() {
    let reset = "reset = \"auto\"\nreset_limit_c = 80.0\n";
    let mut burst = build(&coil_plant(reset, "100.1"));
    let mut intact = build(&coil_plant(reset, "550.0"));

    let at_cut = run_to(&mut burst, 75);
    assert!(tripped(&at_cut, TUBE_SKIN), "the trip cuts on tick 75");
    assert_eq!(
        tubes(&at_cut),
        TubeState::Failed { at_tick: 75 },
        "the tubes burst on the trip's own tick: protection does not un-burst a tube"
    );

    let rearm = rearm_tick(&mut burst, TUBE_SKIN, 1_000);
    let intact_rearm = rearm_tick(&mut intact, TUBE_SKIN, 1_000);
    assert_eq!(
        rearm, intact_rearm,
        "the burst moves no reading the trip compares"
    );
    assert_eq!(rearm, 128, "measured: the re-arm tick M40's gate 3 pins");

    let (dark, lit) = (burst.snapshot(), intact.snapshot());
    assert_eq!(loop_mode(&lit), ControlMode::Auto, "intact tubes relight");
    assert_eq!(loop_mode(&dark), ControlMode::Manual, "burst tubes do not");
    assert_eq!(duty_w(&dark), 0.0);

    // And for good: the record is dropped, not deferred, so nothing relights it
    // later either. The trip stays armed on a cold coil.
    let later = run_to(&mut burst, 600);
    assert!(!tripped(&later, TUBE_SKIN));
    assert_eq!(loop_mode(&later), ControlMode::Manual);
    assert_eq!(duty_w(&later), 0.0);
}

/// Gate 2 — a person's restarting reset on burst tubes re-arms the trip and
/// relights nothing, even with the hole patched: patching stops the leak, it
/// does not mend the tubes. After new tubes, a person's AUTO relights it, so
/// the rule leaves the furnace dark, not bricked.
#[test]
fn a_restarting_reset_waits_for_new_tubes_and_a_person() {
    let mut engine = build(&coil_plant("reset = \"manual_restart\"\n", "100.1"));
    let at = run_to(&mut engine, 150);
    assert!(tripped(&at, TUBE_SKIN) && tubes(&at).is_failed());
    engine
        .apply(Command::PuncturePipe {
            edge: edge_id(&at, "heated_line"),
            area: SquareMeter(0.0),
        })
        .expect("the hole is patched");
    engine
        .apply(Command::ResetTrip { trip_id: TUBE_SKIN })
        .expect("the reset itself is not refused");
    let now = engine.snapshot();
    assert!(!tripped(&now, TUBE_SKIN), "the trip re-armed");
    assert_eq!(loop_mode(&now), ControlMode::Manual);
    let after = run_to(&mut engine, 200);
    assert_eq!(loop_mode(&after), ControlMode::Manual);
    assert_eq!(duty_w(&after), 0.0);

    engine
        .apply(Command::ReplaceTubes {
            node: heater_id(&engine.snapshot()),
        })
        .expect("cold coil, patched hole: new tubes fit");
    engine
        .apply(Command::SetControllerMode {
            loop_id: OUTLET_LOOP,
            mode: ControlMode::Auto,
        })
        .expect("a person puts the loop back in AUTO");
    let relit = run_to(&mut engine, 260);
    assert_eq!(tubes(&relit), TubeState::Intact);
    assert!(duty_w(&relit) > 0.0, "the loop fires the new tubes");
}

/// Gate 3 — tubes that will burst on the next tick are not relit either. A
/// fire on the dark furnace carries its coil past 100 °C between two ticks,
/// tubes still intact; a restarting reset sent THEN re-arms the trip and leaves
/// the duty at zero, and the next tick bursts the tubes. The same reset sent one
/// tick earlier, the coil just under its limit, hands back the 3 MW.
#[test]
fn tubes_about_to_burst_are_not_relit() {
    let fire = Watt(50.0e6);
    // Run until the tank trip has cut the furnace and its tank has cooled
    // 5 K under the 75 °C limit, so the reset is accepted after the fire.
    let mut probe = build(&overheat_plant());
    let mut seen_trip = false;
    loop {
        let s = tick(&mut probe);
        seen_trip |= tripped(&s, TripId(0));
        let clear = s.trips[0]
            .measurement
            .is_some_and(|m| m.magnitude() < 273.15 + 70.0);
        if seen_trip && clear && coil_c(&s) < 90.0 {
            break;
        }
        assert!(s.tick < 20_000, "the tank never cleared");
    }
    let clear_at = probe.snapshot().tick;

    // Light the fire there and find the first tick the coil ends past 100 °C.
    let crossing = {
        let mut engine = build(&overheat_plant());
        run_to(&mut engine, clear_at);
        engine
            .apply(Command::SetHeatInput {
                node: heater_id(&engine.snapshot()),
                power: fire,
            })
            .unwrap();
        loop {
            let s = tick(&mut engine);
            assert_eq!(tubes(&s), TubeState::Intact, "tick {}", s.tick);
            if coil_c(&s) >= 100.0 {
                break s.tick;
            }
        }
    };

    let reset_after = |last_tick: u64| {
        let mut engine = build(&overheat_plant());
        run_to(&mut engine, clear_at);
        engine
            .apply(Command::SetHeatInput {
                node: heater_id(&engine.snapshot()),
                power: fire,
            })
            .unwrap();
        run_to(&mut engine, last_tick);
        engine
            .apply(Command::ResetTrip { trip_id: TripId(0) })
            .expect("the tank is under its limit");
        engine
    };

    assert_eq!(
        (clear_at, crossing),
        (1_594, 1_598),
        "measured: the fire lit on tick 1 594 carries the coil past 100 °C by the end of 1 598"
    );
    let mut early = reset_after(crossing - 1);
    let s = early.snapshot();
    assert!(coil_c(&s) < 100.0, "{}", coil_c(&s));
    assert_eq!(duty_w(&s), 3.0e6, "a coil under its limit is relit");

    let mut late = reset_after(crossing);
    let s = late.snapshot();
    assert!(coil_c(&s) >= 100.0 && tubes(&s) == TubeState::Intact);
    assert!(!tripped(&s, TripId(0)), "the trip re-armed");
    assert_eq!(duty_w(&s), 0.0, "tubes about to burst are not relit");
    let next = tick(&mut late);
    assert!(tubes(&next).is_failed(), "and they burst on the next tick");
    assert_eq!(duty_w(&next), 0.0);
    let _ = tick(&mut early);
}
