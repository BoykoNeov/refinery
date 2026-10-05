//! M40, docs/DESIGN.md §45 (`docs/DEFERRED.md` F2): a trip's snapshot says
//! WHERE it measures, so a frontend can put its limit on the right gauge.
//!
//! The three points the loader admits, each on a shipped plant: a furnace's
//! coil (`tube_skin_high`), a furnace node's outlet (`outlet_high`) and a pipe's
//! flow (`furnace_low_flow_trip.toml`). The ids are resolved here from the
//! snapshot's own `nodes` and `edges` by name, so the test asserts the pairing,
//! not a numbering, and then pins the wire form a frontend branches on.

use refinery_core::graph::MeasurementPoint;
use refinery_core::snapshot::Snapshot;
use refinery_scenarios::{build_engine, load_str};

const COIL_TRIP: &str = include_str!("../../../scenarios/furnace_coil_trip.toml");
const LOW_FLOW_TRIP: &str = include_str!("../../../scenarios/furnace_low_flow_trip.toml");

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
fn a_coil_trip_and_an_outlet_trip_on_one_furnace_say_which_is_which() {
    let snapshot = snapshot_of(COIL_TRIP);
    let heater = node_named(&snapshot, "heater");
    let watches: Vec<(&str, MeasurementPoint)> = snapshot
        .trips
        .iter()
        .map(|t| (t.name.as_str(), t.watches))
        .collect();
    assert_eq!(
        watches,
        vec![
            ("tube_skin_high", MeasurementPoint::Coil(heater)),
            ("outlet_high", MeasurementPoint::Node(heater)),
        ]
    );
    // The wire form: the file's key, the id for the name.
    let bytes = serde_json::to_string(&snapshot).unwrap();
    let heater = heater.0;
    assert!(
        bytes.contains(&format!(
            r#""name":"tube_skin_high","watches":{{"coil":{heater}}}"#
        )),
        "{bytes}"
    );
    assert!(
        bytes.contains(&format!(
            r#""name":"outlet_high","watches":{{"node":{heater}}}"#
        )),
        "{bytes}"
    );
}

#[test]
fn a_flow_trip_names_its_pipe() {
    let snapshot = snapshot_of(LOW_FLOW_TRIP);
    let [trip] = snapshot.trips.as_slice() else {
        panic!("the low-flow demo has one trip");
    };
    let MeasurementPoint::Pipe(pipe) = trip.watches else {
        panic!("a flow trip watches a pipe, got {:?}", trip.watches);
    };
    let edge = snapshot
        .edges
        .iter()
        .find(|e| e.id == pipe)
        .expect("the trip's pipe is in the snapshot's edges");
    let bytes = serde_json::to_string(&snapshot).unwrap();
    assert!(
        bytes.contains(&format!(r#""watches":{{"pipe":{}}}"#, pipe.0)),
        "{bytes}"
    );
    // The pipe the file names, read back through the snapshot alone.
    assert!(
        LOW_FLOW_TRIP.contains(&format!("pipe = \"{}\"", edge.name)),
        "the trip watches '{}', which the file does not name",
        edge.name
    );
}
