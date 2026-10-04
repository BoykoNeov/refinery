//! M22.1: `scenarios/tank_overfill_trip.toml`, as shipped (docs/DESIGN.md §26).
//!
//! The demo is a pump filling a tank faster than its fixed drain empties it,
//! with one latching high-level trip that stops the pump and shuts the fill
//! valve. Three of §26's gates run here, on both fidelities:
//!
//! 1. **It trips at the right tick and holds** (gate 1): armed through tick
//!    1 235, `Tripped { at_tick: 1236 }` from then on, the pump off and the valve
//!    shut, and `fill_line` carrying exactly zero from the tripping tick's own
//!    snapshot — because trips run at the TOP of the tick, before the solve.
//! 2. **The latch holds after the condition clears** (gate 2): the level falls
//!    back under the limit within a tick and keeps falling, and the trip stays
//!    tripped. A trip that did not latch would park the level on the limit.
//! 3. **The wire form** (gate 8): a plant with no trip emits no `trips` key, and
//!    the demo's `state` is tagged, both asserted on the serialized BYTES — a
//!    Rust match on the enum passes under any tag (M10.1's lesson).
//!
//! And the counterfactual the file's header quotes: without the trip, the tank
//! reaches its own 10 m brim and spills. Until M23.1 it passed the brim and read
//! 15.17 m in a 10 m shell, because the engine had no overflow
//! (`docs/DEFERRED.md` B28, now struck; docs/DESIGN.md §27 gate 10).
//!
//! The reset, every refusal and the loop hand-back need commands, which the CLI
//! never issues, so they live on fixtures in `trip_reference.rs`.

use refinery_core::graph::{ControlledValue, NodeKind, TripId, TripState};
use refinery_core::snapshot::Snapshot;
use refinery_core::Engine;
use refinery_scenarios::{build_engine, load_str};

const DEMO: &str = include_str!("../../../scenarios/tank_overfill_trip.toml");
/// The same plant with its `[[trips]]` table removed — the counterfactual, as a
/// shipped file rather than a slice of this one (docs/DESIGN.md §27 fork 7).
const UNTRIPPED: &str = include_str!("../../../scenarios/tank_overflow.toml");
const NO_TRIP_PLANT: &str = include_str!("../../../scenarios/tank_level_control.toml");

/// The tick whose trip pass fires, measured on the engine's own trip (and equal
/// to the M22.0 probe's): the level first reaches 6 m at the END of tick 1 235.
const TRIP_TICK: u64 = 1236;
const LIMIT_M: f64 = 6.0;
const TANK_HEIGHT_M: f64 = 10.0;
const RUN: u64 = 6000;

fn build(src: &str) -> Engine {
    build_engine(&load_str(src).expect("the plant must parse"))
        .unwrap_or_else(|e| panic!("the plant must build: {e}"))
}

fn on(flow: &str) -> String {
    let from = r#"flow = "newton""#;
    assert!(DEMO.contains(from), "the demo declares the newton fidelity");
    DEMO.replace(from, &format!(r#"flow = "{flow}""#))
}

fn tick(engine: &mut Engine) {
    engine
        .tick()
        .unwrap_or_else(|e| panic!("tick {}: {e}", engine.snapshot().tick + 1));
}

fn pipe_flow(snapshot: &Snapshot, name: &str) -> f64 {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("the demo has a pipe '{name}'"))
        .stream
        .mass_flow
        .value()
}

fn pump_on(engine: &Engine) -> bool {
    let id = engine.graph.find_node("transfer_pump").unwrap();
    match engine.graph.node(id).kind {
        NodeKind::Pump { on, .. } => on,
        ref other => panic!("transfer_pump is a pump, not {other:?}"),
    }
}

fn opening(engine: &Engine, name: &str) -> f64 {
    let id = engine.graph.find_node(name).unwrap();
    match engine.graph.node(id).kind {
        NodeKind::Valve { opening, .. } => opening,
        ref other => panic!("{name} is a valve, not {other:?}"),
    }
}

/// The receiving tank's level right now, off the graph — the fresh reading,
/// not the one the last trip pass compared.
fn level(engine: &Engine) -> f64 {
    let id = engine.graph.find_node("receiving_tank").unwrap();
    match &engine.graph.node(id).kind {
        NodeKind::Tank(t) => t.level(&engine.slate).value(),
        other => panic!("receiving_tank is a tank, not {other:?}"),
    }
}

fn measured_level(snapshot: &Snapshot) -> f64 {
    match snapshot.trips[0].measurement {
        Some(ControlledValue::Level { m }) => m.value(),
        other => panic!("the demo's trip measures a level, got {other:?}"),
    }
}

#[test]
fn the_demo_trips_at_its_tick_and_the_flow_stops_on_that_ticks_own_snapshot() {
    for flow in ["newton", "simple"] {
        let mut engine = build(&on(flow));

        // Before any tick: armed, and no measurement yet (fork 8).
        let start = engine.snapshot();
        assert_eq!(start.trips.len(), 1);
        assert_eq!(start.trips[0].id, TripId(0));
        assert_eq!(start.trips[0].state, TripState::Armed, "{flow}");
        assert!(
            start.trips[0].measurement.is_none(),
            "{flow}: no pass has run at load"
        );

        for t in 1..=RUN {
            tick(&mut engine);
            let snapshot = engine.snapshot();
            assert_eq!(snapshot.tick, t);
            let state = snapshot.trips[0].state;
            if t < TRIP_TICK {
                assert_eq!(
                    state,
                    TripState::Armed,
                    "{flow}: tick {t} is before the trip"
                );
                assert!(pump_on(&engine), "{flow}: tick {t}");
                assert!(
                    measured_level(&snapshot) < LIMIT_M,
                    "{flow}: an armed trip compared a level below its limit at tick {t}"
                );
            } else {
                assert_eq!(
                    state,
                    TripState::Tripped {
                        at_tick: TRIP_TICK,
                        by_hand: false
                    },
                    "{flow}: tick {t} is at or after the trip"
                );
                assert!(!pump_on(&engine), "{flow}: the pump is stopped at tick {t}");
                assert_eq!(opening(&engine, "discharge_valve"), 0.0, "{flow}: tick {t}");
                // `==`, which reads −0.0 as zero: the solve writes the sign
                // either way on a branch carrying nothing (§26 premise 2).
                assert!(
                    pipe_flow(&snapshot, "fill_line") == 0.0,
                    "{flow}: fill_line carries {} at tick {t}",
                    pipe_flow(&snapshot, "fill_line")
                );
            }
            if t == TRIP_TICK {
                // The trip compared the level standing at the TOP of this tick,
                // which the previous tick's transport left at or above the limit.
                assert!(measured_level(&snapshot) >= LIMIT_M, "{flow}");
            }
        }
    }
}

#[test]
fn the_trip_stays_latched_while_the_tank_drains_far_below_its_limit() {
    for flow in ["newton", "simple"] {
        let mut engine = build(&on(flow));
        for _ in 0..TRIP_TICK {
            tick(&mut engine);
        }
        // Within a tick of the trip the level is back under the limit — the
        // condition has cleared, which is what makes the latch the subject.
        tick(&mut engine);
        assert!(level(&engine) < LIMIT_M, "{flow}: {}", level(&engine));
        let mut previous = level(&engine);
        for _ in TRIP_TICK + 1..RUN {
            tick(&mut engine);
            let snapshot = engine.snapshot();
            assert!(
                snapshot.trips[0].state.is_tripped(),
                "{flow}: tick {}",
                snapshot.tick
            );
            assert!(level(&engine) <= previous, "{flow}: the tank only drains");
            previous = level(&engine);
        }
        // A trip that did not latch parks the level on 6 m (§26 premise 4).
        assert!(
            level(&engine) < 2.0,
            "{flow}: the latched plant drains to {} m",
            level(&engine)
        );
    }
}

/// The counterfactual the file's header states (docs/DESIGN.md §27 gate 10).
///
/// **The untripped twin is a shipped file, and the test first proves it IS the
/// twin**: `tank_overflow.toml`'s plant, from `[simulation]` on, is this file's
/// up to its `[[trips]]` table, byte for byte. Then it runs: the level passes
/// nothing. It reaches the 10 m brim at tick 2 859 — the tick at which, before
/// M23.1, it passed it on its way to 15.17 m — and from then on the tank holds
/// exactly its capacity at the end of every tick, spilling the rest.
#[test]
fn without_the_trip_the_tank_reaches_its_brim_and_spills() {
    fn plant(src: &str) -> &str {
        &src[src
            .find("[simulation]")
            .expect("a plant declares its simulation")..]
    }
    // Everything before the trip table, less the comment block that introduces
    // it (trailing comment and blank lines), is the plant both files declare.
    fn without_trailing_comments(src: &str) -> String {
        let mut lines: Vec<&str> = src.lines().collect();
        while lines
            .last()
            .is_some_and(|l| l.trim().is_empty() || l.starts_with('#'))
        {
            lines.pop();
        }
        lines.join("\n")
    }
    let at = DEMO.find("\n[[trips]]").expect("the demo declares a trip");
    assert!(
        !UNTRIPPED.contains("\n[[trips]]"),
        "the twin declares no trip"
    );
    assert_eq!(
        without_trailing_comments(plant(UNTRIPPED)),
        without_trailing_comments(plant(&DEMO[..at])),
        "tank_overflow.toml is tank_overfill_trip.toml without its trip"
    );

    for flow in ["newton", "simple"] {
        let from = r#"flow = "newton""#;
        let mut engine = build(&UNTRIPPED.replace(from, &format!(r#"flow = "{flow}""#)));
        assert!(engine.snapshot().trips.is_empty());
        let tank = engine.graph.find_node("receiving_tank").unwrap();
        let mut first_spill = None;
        let mut spilled = 0.0;
        for t in 1..=RUN {
            tick(&mut engine);
            let spill = pipe_flow(&engine.snapshot(), "receiving_tank__overflow");
            if spill > 0.0 {
                first_spill.get_or_insert(t);
                spilled += spill;
            }
            let NodeKind::Tank(state) = &engine.graph.node(tank).kind else {
                panic!("receiving_tank is a tank");
            };
            // Never above the brim, on MASS — the comparison the engine makes.
            assert!(
                state.mass.value() <= state.capacity(&engine.slate).value(),
                "{flow}, tick {t}: {} kg in a tank that holds {} kg",
                state.mass.value(),
                state.capacity(&engine.slate).value()
            );
            if first_spill.is_some() {
                assert_eq!(
                    state.mass.value(),
                    state.capacity(&engine.slate).value(),
                    "{flow}, tick {t}: a spilling tank ends the tick exactly full"
                );
            }
        }
        assert_eq!(first_spill, Some(2859), "{flow}");
        assert!(
            (level(&engine) - TANK_HEIGHT_M).abs() < 1e-12,
            "{flow}: the untripped tank reads {} m in a {TANK_HEIGHT_M} m shell",
            level(&engine)
        );
        // `dt = 1`, so the sum of the rates is the mass. Measured 19 454.4898 kg
        // (newton) and 19 454.4904 kg (simple).
        assert!(
            (spilled - 19_454.49).abs() < 0.01,
            "{flow}: spilled {spilled} kg by tick {RUN}"
        );
    }
}

/// Gate 8, on the bytes. A plant with no trip must not emit the key at all —
/// that is what keeps every pre-M22 plant byte-identical, and CI commits no
/// baseline, so nothing else would notice. And the state's tag is the contract a
/// frontend branches on.
#[test]
fn the_trips_key_is_absent_without_a_trip_and_its_state_is_tagged() {
    let mut plain = build(NO_TRIP_PLANT);
    tick(&mut plain);
    let bytes = serde_json::to_string(&plain.snapshot()).unwrap();
    assert!(
        !bytes.contains("\"trips\""),
        "a plant with no trip emits no trips key"
    );

    let mut engine = build(DEMO);
    let at_load = serde_json::to_string(&engine.snapshot()).unwrap();
    assert!(
        at_load.contains(
            r#""trips":[{"id":0,"name":"receiving_high_level","direction":"high","limit":{"variable":"level","m":6.0},"state":{"status":"armed"}}]"#
        ),
        "at load: {at_load}"
    );
    for _ in 0..TRIP_TICK {
        tick(&mut engine);
    }
    let tripped = serde_json::to_string(&engine.snapshot()).unwrap();
    assert!(
        tripped.contains(r#""state":{"status":"tripped","at_tick":1236}"#),
        "tripped: {tripped}"
    );
    // A round trip through the wire reproduces the same bytes.
    let back: Snapshot = serde_json::from_str(&tripped).unwrap();
    assert_eq!(serde_json::to_string(&back).unwrap(), tripped);
}
