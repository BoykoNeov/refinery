//! M35: a trip on a furnace's COIL and on its OUTLET (docs/DESIGN.md §39, ledger
//! row E13's outlet clause, for furnaces).
//!
//! M34 gave every furnace a coil (§37): tube metal with a temperature that is a
//! state, and the reason a furnace's outlet now exists on every tick but the
//! first. This file gates the two trips that builds on:
//!
//! - **a coil trip**, `{ coil = "heater", variable = "temperature" }` — the
//!   tube-skin temperature a real heater trips on. A state: compared from tick 1.
//! - **an outlet trip**, `{ node = "heater", variable = "temperature" }` — absent
//!   on tick 1's pass only, which compares nothing and leaves the trip armed:
//!   M33's rule for a flow (§36 fork 1), extended by exactly this one case.
//!
//! The demo is `scenarios/furnace_coil_trip.toml`: a FOULED furnace whose outlet
//! loop holds 60 °C while its tubes climb past 100 °C. A cooler's outlet stays
//! refused (it is absent again whenever the cooler stagnates); that case is in
//! `trip_reference.rs`'s sweep, beside the M22 ones.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{ControlledValue, NodeKind, TripId, TripState};
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/furnace_coil_trip.toml");

/// The demo's two trip blocks, verbatim with their comments, so removing them
/// must land.
const DEMO_TRIPS: &str = r#"
# Fires AT OR ABOVE 100 °C on the coil — the tube metal — and cuts the fuel. The
# coil is a state on the graph (docs/DESIGN.md §37), present from load.
[[trips]]
name = "tube_skin_high"
measurement = { coil = "heater", variable = "temperature" }
direction = "high"
limit_c = 100.0
actions = [{ furnace = "heater" }]

# Fires AT OR ABOVE 70 °C on the furnace's outlet. Absent on tick 1's pass only
# (docs/DESIGN.md §39). Never reached here: the outlet loop holds 60 °C.
[[trips]]
name = "outlet_high"
measurement = { node = "heater", variable = "temperature" }
direction = "high"
limit_c = 70.0
actions = [{ furnace = "heater" }]
"#;

/// The tick whose trip pass fires `tube_skin_high`, measured on both fidelities
/// before it was written here: the coil ends tick 74 at 100.18 °C (71 at 100.21 before M36, §40).
const DEMO_TRIP_TICK: u64 = 75;
const SKIN_LIMIT_C: f64 = 100.0;
const OUTLET_LIMIT_C: f64 = 70.0;

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the fixture must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("the fixture must build: {e}"))
}

fn refusal(src: &str) -> String {
    match refinery_scenarios::load_str(src) {
        Err(e) => e.to_string(),
        Ok(file) => match refinery_scenarios::build_engine(&file) {
            Ok(_) => panic!("this plant should not have loaded"),
            Err(e) => e.to_string(),
        },
    }
}

fn swap(src: &str, from: &str, to: &str) -> String {
    assert_eq!(
        src.matches(from).count(),
        1,
        "the fixture's substitution must land exactly once: {from:?}"
    );
    src.replacen(from, to, 1)
}

fn with_solver(src: &str, solver: &str) -> String {
    swap(src, r#"flow = "newton""#, &format!(r#"flow = "{solver}""#))
}

fn tick(engine: &mut Engine) {
    let t = engine.snapshot().tick + 1;
    engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
}

fn duty_w(engine: &Engine) -> f64 {
    let id = engine.graph.find_node("heater").expect("a heater");
    match engine.graph.node(id).kind {
        NodeKind::Furnace { duty, .. } => duty.value(),
        ref other => panic!("'heater' is a furnace, not {other:?}"),
    }
}

/// The coil's temperature, °C: a state, the END of the last tick.
fn coil_c(engine: &Engine) -> f64 {
    let id = engine.graph.find_node("heater").expect("a heater");
    match &engine.graph.node(id).kind {
        NodeKind::Furnace { coil, .. } => coil.temperature.value() - 273.15,
        other => panic!("'heater' is a furnace, not {other:?}"),
    }
}

/// The furnace's resolved outlet, °C.
fn outlet_c(snapshot: &Snapshot) -> f64 {
    snapshot
        .nodes
        .iter()
        .find(|n| n.name == "heater")
        .expect("a heater")
        .temperature_k
        - 273.15
}

fn measured_c(snapshot: &Snapshot, trip: usize) -> Option<f64> {
    snapshot.trips[trip].measurement.map(|m| match m {
        ControlledValue::Temperature { k } => k.value() - 273.15,
        other => panic!("a temperature trip measures a temperature, not {other:?}"),
    })
}

// ------------------------------------------------------------------- gate 1

/// **Gate 1, on both fidelities. The tube-skin trip cuts the fuel with the
/// outlet still under its own setpoint, and the outlet trip never fires.**
///
/// Bit-identical to its untripped twin until the cut; the coil crosses 100 °C at
/// the end of tick 74 and the pass at the top of 75 fires (72 before M36's flame,
/// docs/DESIGN.md §40); on that tick's own snapshot the furnace reads zero duty
/// and the outlet is 55.52 °C — below the loop's 60 °C setpoint and 14.48 K below
/// `outlet_high`. That trip compares
/// nothing on tick 1 and a measurement on every tick after, and stays armed to
/// tick 6 000. The twin, with no trips, settles its outlet on 60 °C with its coil
/// at 117.3 °C: the outlet never shows what the tubes are doing.
#[test]
fn the_tube_skin_trip_cuts_the_fuel_with_the_outlet_on_target() {
    for solver in ["newton", "simple"] {
        let mut demo = build(&with_solver(DEMO, solver));
        let mut twin = build(&with_solver(&swap(DEMO, DEMO_TRIPS, ""), solver));
        assert!(twin.snapshot().trips.is_empty(), "the twin has no trips");
        let mut outlet_peak = f64::MIN;
        for t in 1..=6000u64 {
            tick(&mut demo);
            tick(&mut twin);
            let snapshot = demo.snapshot();
            // `outlet_high` (trip 1): blind on tick 1 only.
            if t == 1 {
                assert_eq!(
                    measured_c(&snapshot, 1),
                    None,
                    "{solver}: no outlet on tick 1"
                );
            } else {
                assert!(measured_c(&snapshot, 1).is_some(), "{solver}, tick {t}");
            }
            assert_eq!(
                snapshot.trips[1].state,
                TripState::Armed,
                "{solver}, tick {t}"
            );
            // `tube_skin_high` (trip 0): a state, so measured from tick 1.
            assert!(measured_c(&snapshot, 0).is_some(), "{solver}, tick {t}");
            outlet_peak = outlet_peak.max(outlet_c(&snapshot));
            if t < DEMO_TRIP_TICK {
                assert_eq!(
                    snapshot.trips[0].state,
                    TripState::Armed,
                    "{solver}, tick {t}"
                );
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
                        at_tick: DEMO_TRIP_TICK,
                        by_hand: false
                    },
                    "{solver}, tick {t}: tripped on its tick, and latched"
                );
                assert_eq!(duty_w(&demo), 0.0, "{solver}, tick {t}: the fuel is cut");
            }
            if t == DEMO_TRIP_TICK - 2 {
                assert!(coil_c(&demo) < SKIN_LIMIT_C, "{solver}: {}", coil_c(&demo));
            }
            if t == DEMO_TRIP_TICK - 1 {
                assert!(coil_c(&demo) >= SKIN_LIMIT_C, "{solver}: {}", coil_c(&demo));
            }
            if t == DEMO_TRIP_TICK {
                let outlet = outlet_c(&snapshot);
                assert!(
                    outlet < 60.0 && OUTLET_LIMIT_C - outlet > 14.0,
                    "{solver}: the tubes trip with the outlet at {outlet} °C, under its \
                     setpoint and far under the outlet trip"
                );
            }
        }
        assert!(
            outlet_peak < OUTLET_LIMIT_C,
            "{solver}: the outlet never reaches the outlet trip: {outlet_peak} °C"
        );
        // What the tube-skin trip prevents: the twin's outlet on target, its tubes
        // at 117.3 °C.
        let twin_snapshot = twin.snapshot();
        assert!(
            (outlet_c(&twin_snapshot) - 60.0).abs() < 0.01 && coil_c(&twin) > 115.0,
            "{solver}: the twin holds 60 °C at the outlet ({}) with its tubes at {} °C",
            outlet_c(&twin_snapshot),
            coil_c(&twin)
        );
        // Cut, the coil gives its heat back and the plant cools to its feed.
        assert!(
            coil_c(&demo) < 40.01 && outlet_c(&demo.snapshot()) < 40.01,
            "{solver}"
        );
    }
}

// ------------------------------------------------------------------- gate 2

/// **Gate 2, the wire form.** On tick 1 the coil trip carries a measurement (a
/// state) and the outlet trip carries no `measurement` key at all (skipped, not
/// `null`); from tick 2 both do, tagged as temperatures. Asserted on the bytes.
#[test]
fn the_coil_trip_measures_on_tick_one_and_the_outlet_trip_from_tick_two() {
    let mut demo = build(DEMO);
    tick(&mut demo);
    let one = demo.snapshot().trips;
    let skin = serde_json::to_string(&one[0]).unwrap();
    let outlet = serde_json::to_string(&one[1]).unwrap();
    assert!(
        skin.contains(r#""measurement":{"variable":"temperature","k":"#),
        "{skin}"
    );
    assert!(!outlet.contains("measurement"), "{outlet}");
    assert!(
        outlet.contains(r#""limit":{"variable":"temperature","k":343.15}"#),
        "{outlet}"
    );
    tick(&mut demo);
    let two = serde_json::to_string(&demo.snapshot().trips[1]).unwrap();
    assert!(
        two.contains(r#""measurement":{"variable":"temperature","k":"#),
        "{two}"
    );
}

// ------------------------------------------------------------- gates 3 and 4

/// **Gate 3, the coil has no window.** A coil loaded above its limit is compared
/// on tick 1's pass, and the trip cuts the fuel on tick 1: the coil is a state on
/// the graph from load, so there is nothing to wait for.
#[test]
fn a_coil_loaded_over_its_limit_trips_on_tick_one() {
    // The coil loads at 71.29 °C; a 70 °C skin limit is already reached.
    let mut engine = build(&swap(DEMO, "limit_c = 100.0", "limit_c = 70.0"));
    tick(&mut engine);
    let one = engine.snapshot();
    assert_eq!(
        one.trips[0].state,
        TripState::Tripped {
            at_tick: 1,
            by_hand: false
        }
    );
    assert_eq!(duty_w(&engine), 0.0, "cut on tick 1");
    assert!(measured_c(&one, 0).is_some_and(|c| c > 71.0));
}

/// **Gate 4, the outlet's one-tick window.** A furnace whose outlet is already
/// above an outlet trip's limit runs tick 1 unprotected — the pass had no outlet
/// to compare — and trips on tick 2, as M33's flow trip does on a plant loaded
/// with too little feed (§36 fork 1). Here the window costs nothing visible: the
/// coil, not the outlet, is what one tick of fire heats.
#[test]
fn a_furnace_loaded_over_its_outlet_limit_trips_on_tick_two() {
    // Tick 1's outlet is 48.32 °C; a 45 °C outlet limit is reached from the start.
    let mut engine = build(&swap(DEMO, "limit_c = 70.0", "limit_c = 45.0"));
    tick(&mut engine);
    let one = engine.snapshot();
    assert_eq!(measured_c(&one, 1), None, "no outlet to compare on tick 1");
    assert_eq!(one.trips[1].state, TripState::Armed);
    assert!(duty_w(&engine) > 0.0, "lit through tick 1");
    assert!(outlet_c(&one) > 45.0, "{}", outlet_c(&one));
    tick(&mut engine);
    assert_eq!(
        engine.snapshot().trips[1].state,
        TripState::Tripped {
            at_tick: 2,
            by_hand: false
        }
    );
    assert_eq!(duty_w(&engine), 0.0);
}

// ------------------------------------------------------------------- gate 5

/// **Gate 5, a reset on an outlet trip reads the last resolved outlet, fresh.**
/// After gate 4's cut the coil gives its heat back and the outlet falls. A reset
/// is refused on every tick that ended with the outlet still at or over 45 °C, and
/// admitted on the first tick that ended under it — with no extra tick, because
/// the reset reads what the last tick resolved, which is what the next tick's pass
/// would compare (§26 fork 4). It relights nothing.
#[test]
fn a_reset_on_an_outlet_trip_reads_the_last_resolved_outlet() {
    let mut engine = build(&swap(DEMO, "limit_c = 70.0", "limit_c = 45.0"));
    tick(&mut engine);
    tick(&mut engine);
    assert!(engine.snapshot().trips[1].state.is_tripped());
    // The skin trip must not be what holds the furnace here.
    assert_eq!(engine.snapshot().trips[0].state, TripState::Armed);
    let reset = |engine: &mut Engine| {
        engine
            .apply(Command::ResetTrip { trip_id: TripId(1) })
            .map_err(|e| e.to_string())
    };
    let mut admitted = false;
    for _ in 0..2000 {
        let outlet = outlet_c(&engine.snapshot());
        if outlet >= 45.0 {
            let refused = reset(&mut engine).expect_err("over the limit, refused");
            assert!(refused.contains("condition still holds"), "{refused}");
        } else {
            reset(&mut engine).unwrap_or_else(|e| panic!("under the limit at {outlet}: {e}"));
            admitted = true;
            break;
        }
        tick(&mut engine);
    }
    assert!(
        admitted,
        "the outlet cools under its limit as the coil gives its heat back"
    );
    assert_eq!(engine.snapshot().trips[1].state, TripState::Armed);
    assert_eq!(duty_w(&engine), 0.0, "a reset relights nothing");
}

// ------------------------------------------------------------------- gate 6

/// **Gate 6, the refusals.** A coil is a furnace's, it has a temperature and
/// nothing else, it is named alone, and only a trip may watch one: a loop on a
/// coil would be a skin-temperature override, which no slice has admitted.
#[test]
fn every_malformed_coil_measurement_is_refused_for_its_own_reason() {
    // Anchored on the trip's name: the measurement line alone also appears in the
    // demo's header comment.
    let skin = r#"name = "tube_skin_high"
measurement = { coil = "heater", variable = "temperature" }"#;
    let skin_at = |measurement: &str| {
        format!(
            "name = \"tube_skin_high\"
{measurement}"
        )
    };
    let cases = [
        (
            "a coil on a node that is not a furnace",
            swap(
                DEMO,
                skin,
                &skin_at(r#"measurement = { coil = "drain_valve", variable = "temperature" }"#),
            ),
            "is not a furnace, so it has no coil",
        ),
        (
            "a coil on an unknown node",
            swap(
                DEMO,
                skin,
                &skin_at(r#"measurement = { coil = "nowhere", variable = "temperature" }"#),
            ),
            "the coil of unknown node 'nowhere'",
        ),
        (
            "a coil with a level",
            swap(
                &swap(
                    DEMO,
                    skin,
                    &skin_at(r#"measurement = { coil = "heater", variable = "level" }"#),
                ),
                "limit_c = 100.0",
                "limit_m = 5.0",
            ),
            "is measured for its temperature only",
        ),
        (
            "a coil and a node together",
            swap(
                DEMO,
                skin,
                &skin_at(
                    r#"measurement = { coil = "heater", node = "heater", variable = "temperature" }"#,
                ),
            ),
            "and `node` in `measurement`",
        ),
        (
            "a loop on a coil",
            swap(
                DEMO,
                r#"measurement = { node = "heater", variable = "temperature" }
actuator = "heater""#,
                r#"measurement = { coil = "heater", variable = "temperature" }
actuator = "heater""#,
            ),
            "A coil is watched by TRIPS",
        ),
    ];
    for (label, src, says) in cases {
        let message = refusal(&src);
        assert!(
            message.contains(says),
            "{label}: expected the refusal to say `{says}`, and it said: {message}"
        );
    }
}
