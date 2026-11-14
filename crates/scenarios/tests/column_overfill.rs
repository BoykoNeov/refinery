//! M75.4 — the overfill plant shipped (`scenarios/crude_unit_overfill.toml`,
//! docs/DESIGN.md §80.7): the holdup plant with its steel, the M75.1
//! condenser limit, the M75.2 drain and the M75.3 relief valve on the map —
//! exactly the plant M75's inline tests build (`column_relief::with_relief`),
//! its valve the book's at relieving conditions, Kv 5 000 (the user's
//! decision, 2026-11-14). These gates keep the file and the fixtures from
//! drifting apart, so every M75 test speaks for the shipped plant.

use std::collections::BTreeMap;

use refinery_core::engine::Engine;
use refinery_core::snapshot::Snapshot;
use refinery_core::units::Pascal;

use crate::column_empty::{engine_of, holdup, reading, run, state, FLARE_PIPE};
use crate::column_relief::{close, vent, with_relief, DESIGN, INLET_PIPE, P_ATM, SET, X_T};

const OVERFILL_PLANT: &str = include_str!("../../../scenarios/crude_unit_overfill.toml");

/// The relief valve the plant declares: the book rule at relieving
/// conditions (DESIGN §80.4, §80.7).
const KV: f64 = 5000.0;

/// Every node's and pipe's readings by name, its id and ends taken off (the
/// file's nodes stand in another order than the fixture's, which shifts ids).
fn by_name(snapshot: &Snapshot) -> BTreeMap<String, serde_json::Value> {
    let mut named = BTreeMap::new();
    for node in &snapshot.nodes {
        let mut value = serde_json::to_value(node).unwrap();
        value.as_object_mut().unwrap().remove("id");
        named.insert(format!("node {}", node.name), value);
    }
    for edge in &snapshot.edges {
        let mut value = serde_json::to_value(edge).unwrap();
        let fields = value.as_object_mut().unwrap();
        for key in ["id", "from", "to"] {
            fields.remove(key);
        }
        named.insert(format!("edge {}", edge.name), value);
    }
    named
}

/// **The shipped plant is the fixture**: its condenser, its drain and its
/// relief declared as M75's inline tests declare them — the probe's
/// condenser (5 %, 35 °C, 100 m³), the book's valve (set at the design
/// pressure, full lift 10 % of it gauge above, `x_t` 0.7) of Kv 5 000 — and
/// over 150 ticks it runs as that fixture does, every node, pipe and the
/// column's state by name to `1e-6` (the valve's set and accumulation are
/// written to the bar's sixth digit in the file, the fixture's to the bit).
#[test]
fn the_shipped_plant_is_the_fixture() {
    let mut shipped = engine_of(OVERFILL_PLANT).unwrap();
    let mut fixture = engine_of(&with_relief(KV)).unwrap();
    let (ours, theirs) = (holdup(&shipped), holdup(&fixture));
    let (ours, theirs) = (ours.condenser.unwrap(), theirs.condenser.unwrap());
    assert_eq!(ours.margin, theirs.margin);
    assert_eq!(ours.coolant, theirs.coolant);
    assert_eq!(ours.overhead_volume, theirs.overhead_volume);
    assert_eq!(ours.set_pressure, Pascal(SET));
    assert_eq!(ours.design_pressure, theirs.design_pressure);
    let (ours, theirs) = (ours.relief.unwrap(), theirs.relief.unwrap());
    assert_eq!(ours.cv_max, KV / (3600.0 * 1e5_f64.sqrt()));
    assert_eq!(ours.cv_max, theirs.cv_max);
    assert_eq!(ours.x_t, X_T);
    assert!((ours.set_pressure.value() - DESIGN).abs() < 1e-6);
    assert!((ours.set_pressure.value() - theirs.set_pressure.value()).abs() < 1e-6);
    assert!((ours.accumulation.value() - 0.10 * (DESIGN - P_ATM)).abs() < 1e-6);
    assert_eq!(ours.ceiling, theirs.ceiling);
    assert!((ours.flare_pressure.value() - P_ATM).abs() < 1e-6);
    for tick in 0..150 {
        shipped
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        fixture
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        let as_value = |e: &Engine| serde_json::to_value(state(e)).unwrap();
        assert!(
            close(&as_value(&shipped), &as_value(&fixture)),
            "tick {tick}: the column's state"
        );
        let (ours, theirs) = (by_name(&shipped.snapshot()), by_name(&fixture.snapshot()));
        assert_eq!(
            ours.keys().collect::<Vec<_>>(),
            theirs.keys().collect::<Vec<_>>()
        );
        for (name, value) in &ours {
            assert!(
                close(value, &theirs[name]),
                "tick {tick}: {name}\n{value}\n{}",
                theirs[name]
            );
        }
    }
}

/// **At rest the shipped plant holds its set pressure, its relief shut**:
/// over 600 ticks the column stands at 1.8 bar exactly, the condenser within
/// its capacity, nothing backed up, the relief shut and its line to the
/// flare carrying exactly nothing, no alarm.
#[test]
fn at_rest_the_shipped_plant_holds_its_set_pressure() {
    let mut engine = engine_of(OVERFILL_PLANT).unwrap();
    for tick in 1..=600 {
        let snapshot = run(&mut engine, 1);
        let condenser = reading(&snapshot).condenser.unwrap();
        assert_eq!(condenser.pressure_pa, SET, "tick {tick}");
        assert!(!condenser.at_capacity, "tick {tick}");
        assert!(!condenser.at_design, "tick {tick}");
        assert!(!condenser.at_ceiling, "tick {tick}");
        assert_eq!(condenser.flooded_share, 0.0, "tick {tick}");
        for name in [INLET_PIPE, FLARE_PIPE] {
            let line = snapshot.edges.iter().find(|e| e.name == name).unwrap();
            assert_eq!(line.stream.mass_flow.value(), 0.0, "tick {tick}: {name}");
        }
        if tick > 1 {
            let shut = vent(&snapshot);
            assert_eq!(shut.opening, 0.0, "tick {tick}");
            assert_eq!(shut.flow_kg_s, 0.0, "tick {tick}");
        }
    }
}
