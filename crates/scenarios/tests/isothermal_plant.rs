//! The reference plant must stay exactly isothermal (M2 slice 1).
//!
//! Lives here rather than beside the other energy tests in
//! `solvers/tests/energy_invariants.rs` because it needs the reference TOML and
//! its loader, and `scenarios` depends on `solvers` — the reverse import would
//! be a dependency cycle.
//!
//! `tank_pump_valve` is 20 °C throughout: both tanks, and therefore every
//! stream they feed. No arrangement of flows can change that, because mixing a
//! fluid with itself is a no-op and there is no heat input anywhere in the
//! plant. So 293.15 K is not an approximation to hold within a tolerance — it
//! is a number that must not move at all.
//!
//! That makes this a cheap, very sharp trap for *spurious* energy terms. Every
//! M2 test that exercises a temperature DIFFERENCE would still pass if the
//! engine quietly added a constant offset, leaked pump work into the stream, or
//! mishandled T_REF symmetrically at both ends of a balance. Here, any of those
//! shows up immediately as drift off a flat line.

use refinery_core::engine::Engine;
use refinery_core::graph::NodeKind;

const SCENARIO: &str = include_str!("../../../scenarios/tank_pump_valve.toml");

/// Every node and pipe in the reference plant is at 20 °C.
const PLANT_TEMPERATURE_K: f64 = 293.15;

/// Long enough for the levels (and so the flow) to move substantially, which is
/// the point: the temperature must be invariant to all of it.
const TICKS: u64 = 200;

/// Pure float round-off on a value that is only ever copied, never computed
/// from a difference — measured drift is 0.0 exactly at the time of writing.
/// Kept as a tolerance rather than `assert_eq!` so a future T-dependent
/// property model can perturb the last bits without a spurious failure.
const TOLERANCE_K: f64 = 1e-9;

fn tank_temperature(engine: &Engine, name: &str) -> f64 {
    let id = engine
        .graph
        .find_node(name)
        .unwrap_or_else(|| panic!("the reference plant must have a '{name}'"));
    match &engine.graph.node(id).kind {
        NodeKind::Tank(tank) => tank.temperature.value(),
        other => panic!("'{name}' is not a tank: {other:?}"),
    }
}

#[test]
fn the_reference_plant_stays_isothermal() {
    let file = refinery_scenarios::load_str(SCENARIO).expect("reference scenario must parse");
    let mut engine = refinery_scenarios::build_engine(&file).expect("reference plant must build");

    for tick in 1..=TICKS {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick} failed: {e:?}"));

        for name in ["supply_tank", "receiving_tank"] {
            let actual = tank_temperature(&engine, name);
            assert!(
                (actual - PLANT_TEMPERATURE_K).abs() < TOLERANCE_K,
                "tick {tick}: {name} drifted to {actual} K in a plant that is \
                 {PLANT_TEMPERATURE_K} K throughout"
            );
        }

        let snapshot = engine.snapshot();
        assert_eq!(
            snapshot.edges.len(),
            3,
            "the reference plant has three pipes; a vacuous loop would prove nothing"
        );
        for edge in snapshot.edges {
            let actual = edge.stream.temperature.value();
            assert!(
                (actual - PLANT_TEMPERATURE_K).abs() < TOLERANCE_K,
                "tick {tick}: stream '{}' drifted to {actual} K",
                edge.name
            );
        }
    }
}
