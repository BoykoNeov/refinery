//! M41, docs/DESIGN.md §46 (`docs/DEFERRED.md` F3): a loop's snapshot says
//! WHERE it measures — what M40 gave the trips (`trip_watches.rs`), on the
//! faceplate.
//!
//! The two points a loop may watch, each on a shipped plant: a node's state
//! (both loops of `furnace_cascade_control.toml`, a tank and the furnace
//! outlet — the screen with more than one loop to place) and a pipe's flow
//! (`tank_flow_control.toml`). A loop on a coil is refused at load (§39), so
//! there is no third. The ids are resolved here by name from the snapshot's own
//! `nodes` and `edges`, so the test asserts the pairing, not a numbering, and
//! then pins the wire form a frontend branches on.

use refinery_core::graph::MeasurementPoint;
use refinery_core::snapshot::Snapshot;
use refinery_scenarios::{build_engine, load_str};

const CASCADE: &str = include_str!("../../../scenarios/furnace_cascade_control.toml");
const FLOW_CONTROL: &str = include_str!("../../../scenarios/tank_flow_control.toml");

fn snapshot_of(src: &str) -> Snapshot {
    build_engine(&load_str(src).expect("the plant must parse"))
        .expect("the plant must build")
        .snapshot()
}

fn node_named(snapshot: &Snapshot, name: &str) -> refinery_core::graph::NodeId {
    snapshot
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("no node '{name}'"))
        .id
}

#[test]
fn two_loops_of_a_cascade_say_which_gauge_each_belongs_on() {
    let snapshot = snapshot_of(CASCADE);
    let tank = node_named(&snapshot, "hold_tank");
    let heater = node_named(&snapshot, "heater");
    let watches: Vec<(&str, MeasurementPoint)> = snapshot
        .controls
        .iter()
        .map(|c| (c.name.as_str(), c.watches))
        .collect();
    assert_eq!(
        watches,
        vec![
            ("tank_temperature", MeasurementPoint::Node(tank)),
            ("outlet_temperature", MeasurementPoint::Node(heater)),
        ]
    );
    // The wire form: right after the name, as a trip's.
    let bytes = serde_json::to_string(&snapshot).unwrap();
    for (name, id) in [
        ("tank_temperature", tank.0),
        ("outlet_temperature", heater.0),
    ] {
        assert!(
            bytes.contains(&format!(r#""name":"{name}","watches":{{"node":{id}}}"#)),
            "{bytes}"
        );
    }
}

#[test]
fn a_flow_loop_names_its_pipe() {
    let snapshot = snapshot_of(FLOW_CONTROL);
    let [control] = snapshot.controls.as_slice() else {
        panic!("the flow-control demo has one loop");
    };
    let MeasurementPoint::Pipe(pipe) = control.watches else {
        panic!("a flow loop watches a pipe, got {:?}", control.watches);
    };
    let edge = snapshot
        .edges
        .iter()
        .find(|e| e.id == pipe)
        .expect("the loop's pipe is in the snapshot's edges");
    let bytes = serde_json::to_string(&snapshot).unwrap();
    assert!(
        bytes.contains(&format!(r#""watches":{{"pipe":{}}}"#, pipe.0)),
        "{bytes}"
    );
    // The pipe the file names, read back through the snapshot alone.
    assert!(
        FLOW_CONTROL.contains(&format!("pipe = \"{}\"", edge.name)),
        "the loop watches '{}', which the file does not name",
        edge.name
    );
}
