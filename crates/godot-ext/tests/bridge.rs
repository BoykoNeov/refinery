//! Gates for the translation layer (M6.2, DESIGN §8).
//!
//! What is being pinned here is a **contract**, not physics: the JSON text a
//! frontend writes, the ids it may send, the codes it branches on, and the
//! shape of the snapshot it reads. Physics is gated in `scenarios/tests`.

use refinery_core::graph::{ControlMode, ControlledValue, EdgeId, LoopId, NodeId, TripId};
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::units::{Meter, SquareMeter, Watt};
use refinery_core::SimError;
use refinery_godot_ext::bridge::{Bridge, BridgeError, ErrorReport, Session, MISSING_ID};
use serde_json::Value;

// ---------------------------------------------------------------- fixtures

fn scenario_src(name: &str) -> String {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("reading {}: {e}", path.display()))
}

fn bridge(name: &str) -> Bridge {
    Bridge::load(&scenario_src(name)).expect("scenario loads")
}

/// The plant M6.1 built: a pump, a control valve, tanks, and a pipe declared
/// punctureable — four of the six commands are exercisable on it alone.
const LEAKY: &str = "leaking_line.toml";

/// A plant with one control loop, declared here rather than shipped.
///
/// **Inline on purpose.** No scenario in `scenarios/` declares a `[[controls]]`
/// table: the thirteen were written before M8 and adding one to any of them would
/// move its snapshot, which is the regression anchor this milestone is measured
/// against. The wired demo that regulates is M8.4's, and a bridge contract test
/// should not be the thing that forces it early — so the two loop-addressed
/// commands get the smallest plant that can carry a loop.
const LEVEL: &str = r#"
[meta]
name = "bridge_level_control"
[simulation]
dt = 0.5
[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"

[nodes.header]
type = "source"
pressure_bar = 5.0
temperature_c = 20.0

[nodes.control_tank]
type = "tank"
area_m2 = 3.0
height_m = 10.0
initial_level_m = 4.0
temperature_c = 20.0

[nodes.drain_valve]
type = "valve"
kv = 60.0
opening = 0.2

[nodes.rundown]
type = "sink"
pressure_bar = 1.01325

[[pipes]]
name = "fill_line"
from = "header"
to = "control_tank"
length_m = 10.0
diameter_m = 0.10

[[pipes]]
name = "drain_line"
from = "control_tank"
to = "drain_valve"
length_m = 10.0
diameter_m = 0.10

[[pipes]]
name = "rundown_line"
from = "drain_valve"
to = "rundown"
length_m = 10.0
diameter_m = 0.10

[[controls]]
name = "tank_level"
measurement = { node = "control_tank", variable = "level" }
actuator = "drain_valve"
algorithm = "p"
mode = "auto"
setpoint_m = 4.0
gain_per_m = 0.5
"#;

/// A plant with one trip that fires on tick 1 and whose condition then clears,
/// so `reset_trip` has something legal to do (M22, docs/DESIGN.md §26).
///
/// The tank is declared at 5.0 m against a HIGH trip at 4.9 m, so the first trip
/// pass fires it; its action throws the drain wide OPEN (a dump valve trips
/// open, which is why a valve action says its position), and the tank falls
/// under the limit within a few ticks. `TRIP_TICKS` steps well past that.
const TRIP: &str = r#"
[meta]
name = "bridge_trip"
[simulation]
dt = 1.0
[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"

[nodes.dump_tank]
type = "tank"
area_m2 = 1.0
height_m = 10.0
initial_level_m = 5.0
temperature_c = 20.0

[nodes.dump_valve]
type = "valve"
kv = 50.0
opening = 0.1

[nodes.drain]
type = "sink"
pressure_bar = 1.01325

[[pipes]]
name = "dump_line"
from = "dump_tank"
to = "dump_valve"
length_m = 10.0
diameter_m = 0.10

[[pipes]]
name = "drain_line"
from = "dump_valve"
to = "drain"
length_m = 10.0
diameter_m = 0.10

[[trips]]
name = "dump_on_high_level"
measurement = { node = "dump_tank", variable = "level" }
direction = "high"
limit_m = 4.9
actions = [{ valve = "dump_valve", position = 1.0 }]
"#;
const TRIP_TICKS: u32 = 50;

// ------------------------------------------------ the wire format, pinned

/// The exact JSON text of every `Command` variant.
///
/// **Wildcard-free on purpose, and this is the point of the test.** The
/// bridge itself never matches on a `Command`'s *fields*, so nothing in
/// `src/` fails to build when a `#[serde]` tag is renamed or a field is
/// spelled differently. This match does: a new variant stops the test suite
/// compiling until its wire text is written down, and a renamed tag fails the
/// assertion below. DESIGN §7 calls these names a frontend contract; this is
/// where that claim is enforced rather than asserted.
fn wire_text(cmd: &Command) -> &'static str {
    match cmd {
        Command::SetValveOpening { .. } => r#"{"cmd":"set_valve_opening","node":2,"opening":0.25}"#,
        Command::SetPumpOn { .. } => r#"{"cmd":"set_pump_on","node":1,"on":false}"#,
        Command::PuncturePipe { .. } => r#"{"cmd":"puncture_pipe","edge":2,"area":0.0001}"#,
        Command::SetHeatInput { .. } => r#"{"cmd":"set_heat_input","node":0,"power":1000.0}"#,
        Command::SetFurnaceDuty { .. } => r#"{"cmd":"set_furnace_duty","node":1,"duty":500000.0}"#,
        Command::SetCoolerDuty { .. } => r#"{"cmd":"set_cooler_duty","node":1,"duty":500000.0}"#,
        Command::SetControllerMode { .. } => {
            r#"{"cmd":"set_controller_mode","loop_id":0,"mode":"manual"}"#
        }
        // The setpoint is a TAGGED value, not a bare number, and that is the part
        // of this line worth reading: a loop's setpoint is metres today and
        // Pascals once pressure control un-defers, so the unit travels with the
        // value rather than in a field name (docs/DESIGN.md §10 fork 4).
        Command::SetSetpoint { .. } => {
            r#"{"cmd":"set_setpoint","loop_id":0,"value":{"variable":"level","m":5.0}}"#
        }
        Command::ResetTrip { .. } => r#"{"cmd":"reset_trip","trip_id":0}"#,
    }
}

/// One value of each variant, matching `wire_text` field for field.
fn every_variant() -> Vec<Command> {
    vec![
        Command::SetValveOpening {
            node: NodeId(2),
            opening: 0.25,
        },
        Command::SetPumpOn {
            node: NodeId(1),
            on: false,
        },
        Command::PuncturePipe {
            edge: EdgeId(2),
            area: SquareMeter(1.0e-4),
        },
        Command::SetHeatInput {
            node: NodeId(0),
            power: Watt(1000.0),
        },
        Command::SetFurnaceDuty {
            node: NodeId(1),
            duty: Watt(500_000.0),
        },
        Command::SetCoolerDuty {
            node: NodeId(1),
            duty: Watt(500_000.0),
        },
        Command::SetControllerMode {
            loop_id: LoopId(0),
            mode: ControlMode::Manual,
        },
        Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Level { m: Meter(5.0) },
        },
        Command::ResetTrip { trip_id: TripId(0) },
    ]
}

#[test]
fn command_wire_format_is_pinned_in_both_directions() {
    for cmd in every_variant() {
        let expected = wire_text(&cmd);

        // Out: the engine's serde produces exactly this text.
        let written = serde_json::to_string(&cmd).expect("Command serializes");
        assert_eq!(
            written, expected,
            "the JSON a frontend must READ changed for {cmd:?}"
        );

        // In: that text parses back to the same variant with the same fields.
        // Compared through the text rather than by `PartialEq`, which
        // `Command` does not implement — and re-serializing is a stricter
        // check anyway, since it would catch a field silently defaulting.
        let parsed: Command = serde_json::from_str(expected).expect("Command parses");
        assert_eq!(
            serde_json::to_string(&parsed).unwrap(),
            expected,
            "the JSON a frontend must WRITE changed for {cmd:?}"
        );
    }

    // The count is part of the claim: it is what makes "every variant" true
    // rather than "every variant someone remembered".
    assert_eq!(every_variant().len(), 9, "a Command variant was added");
}

// -------------------------------------------- commands reach a real engine

/// A plant each variant is legal on, and the element it addresses there.
///
/// **Wildcard-free, and that is what makes the sweep below a mechanism rather
/// than a list someone maintains.** A new `Command` variant does not compile
/// until it names somewhere it can be applied.
fn fixture(cmd: &Command) -> Fixture {
    match cmd {
        Command::SetValveOpening { .. } => Fixture::shipped(LEAKY, "discharge_valve"),
        Command::SetPumpOn { .. } => Fixture::shipped(LEAKY, "transfer_pump"),
        Command::PuncturePipe { .. } => Fixture::shipped(LEAKY, "fill_line"),
        Command::SetHeatInput { .. } => Fixture::shipped(LEAKY, "receiving_tank"),
        Command::SetFurnaceDuty { .. } => Fixture::shipped("furnace_heater.toml", "heater"),
        Command::SetCoolerDuty { .. } => Fixture::shipped("cooler_chiller.toml", "chiller"),
        // A loop-addressed command needs a plant with a loop, and the one it gets
        // is written above rather than shipped — see `LEVEL`.
        Command::SetControllerMode { .. } => Fixture::inline(LEVEL, "tank_level"),
        Command::SetSetpoint { .. } => Fixture::inline(LEVEL, "tank_level"),
        // A reset is legal only on a trip that has fired and whose condition has
        // cleared, so its plant has to be run first — see `TRIP`.
        Command::ResetTrip { .. } => Fixture::Tripped {
            src: TRIP,
            trip: "dump_on_high_level",
            ticks: TRIP_TICKS,
        },
    }
}

/// Where a command can be applied, and what it addresses there.
///
/// Two constructors rather than a tuple, because a loop-addressed command names
/// something the phone book has no entry for: `Bridge` resolves node and edge
/// names, and a loop's id travels on the snapshot beside its name instead (a
/// name→id lookup for loops is a frontend affordance, and belongs with M8.5).
enum Fixture {
    Shipped {
        plant: &'static str,
        target: &'static str,
    },
    Inline {
        src: &'static str,
        control: &'static str,
    },
    /// A trip-addressed command: the plant is stepped `ticks` times first, and
    /// the trip's id is read off `snapshot.trips` beside its name.
    Tripped {
        src: &'static str,
        trip: &'static str,
        ticks: u32,
    },
}

impl Fixture {
    fn shipped(plant: &'static str, target: &'static str) -> Self {
        Fixture::Shipped { plant, target }
    }
    fn inline(src: &'static str, control: &'static str) -> Self {
        Fixture::Inline { src, control }
    }
}

/// Every variant is accepted by a real engine through the bridge.
///
/// Driven from `every_variant()`, whose completeness `wire_text`'s match
/// already forces — so "every" here is enforced by the compiler, not by the
/// author's memory. This gate covers ACCEPTANCE only; the per-command effects
/// are the hand-picked sweep below, which deliberately does not claim to be
/// exhaustive.
#[test]
fn every_command_variant_is_accepted_by_a_real_engine() {
    for cmd in every_variant() {
        let (mut sim, key, id, where_) = match fixture(&cmd) {
            Fixture::Shipped { plant, target } => {
                let sim = bridge(plant);
                let value = serde_json::to_value(&cmd).expect("Command to JSON");
                let key = if value.get("node").is_some() {
                    "node"
                } else if value.get("edge").is_some() {
                    "edge"
                } else {
                    panic!(
                        "{cmd:?} addresses neither a node nor an edge, and its fixture \
                            says it is shipped-plant addressed — `Referent` and this \
                            sweep both need a decision"
                    )
                };
                let id = if key == "node" {
                    sim.node_id(target)
                } else {
                    sim.edge_id(target)
                }
                .unwrap_or_else(|e| panic!("{plant} has no '{target}': {e}"));
                (sim, key, id, plant)
            }
            // A loop's id is read off the snapshot beside its name, which is how a
            // frontend addresses one: there is no `loop_id(name)` on the bridge.
            Fixture::Inline { src, control } => {
                let sim = Bridge::load(src).expect("inline control plant loads");
                let id = snapshot_value(&sim)["controls"]
                    .as_array()
                    .and_then(|loops| {
                        loops
                            .iter()
                            .find(|l| l["name"] == control)
                            .and_then(|l| l["id"].as_i64())
                    })
                    .unwrap_or_else(|| panic!("inline plant has no control loop '{control}'"));
                (sim, "loop_id", id, "the inline control plant")
            }
            Fixture::Tripped { src, trip, ticks } => {
                let mut sim = Bridge::load(src).expect("inline trip plant loads");
                for _ in 0..ticks {
                    sim.tick().expect("the inline trip plant ticks");
                }
                let id = snapshot_value(&sim)["trips"]
                    .as_array()
                    .and_then(|trips| {
                        trips
                            .iter()
                            .find(|t| t["name"] == trip)
                            .and_then(|t| t["id"].as_i64())
                    })
                    .unwrap_or_else(|| panic!("inline plant has no trip '{trip}'"));
                (sim, "trip_id", id, "the inline trip plant")
            }
        };

        // Rewrite the placeholder id in the canned variant with one this
        // plant actually has, leaving every other field as written.
        let mut value = serde_json::to_value(&cmd).expect("Command to JSON");
        let object = value.as_object_mut().expect("Command is a JSON object");
        object.insert(key.to_string(), Value::from(id));

        sim.apply_command_json(&value.to_string())
            .unwrap_or_else(|e| panic!("{cmd:?} refused on {where_}: {e}"));
    }
}

/// The effects themselves, hand-picked per command. **Not exhaustive**, by
/// name and by intent: what each command should physically do differs enough
/// that a generic assertion would say nothing. Completeness of the variant
/// set is `every_command_variant_is_accepted_by_a_real_engine`'s job.
#[test]
fn each_command_has_its_documented_effect() {
    // Valve, pump and puncture, on M6.1's leaky plant.
    let mut sim = bridge(LEAKY);
    let valve = sim.node_id("discharge_valve").unwrap();
    let pump = sim.node_id("transfer_pump").unwrap();
    let pipe = sim.edge_id("fill_line").unwrap();

    sim.apply_command_json(&format!(
        r#"{{"cmd":"set_valve_opening","node":{valve},"opening":0.25}}"#
    ))
    .expect("valve opening accepted");
    assert!(
        snapshot_value(&sim)["nodes"][valve as usize]["kind"]["opening"]
            .as_f64()
            .unwrap()
            == 0.25,
        "SetValveOpening did not reach the valve"
    );

    sim.apply_command_json(&format!(
        r#"{{"cmd":"set_pump_on","node":{pump},"on":false}}"#
    ))
    .expect("pump command accepted");
    assert_eq!(
        snapshot_value(&sim)["nodes"][pump as usize]["kind"]["on"],
        Value::Bool(false),
        "SetPumpOn did not reach the pump"
    );

    sim.apply_command_json(&format!(
        r#"{{"cmd":"set_pump_on","node":{pump},"on":true}}"#
    ))
    .unwrap();
    sim.apply_command_json(&format!(
        r#"{{"cmd":"puncture_pipe","edge":{pipe},"area":0.0005}}"#
    ))
    .expect("puncture accepted");
    sim.tick().expect("tick after puncture");
    let leaked = snapshot_value(&sim)["edges"][pipe as usize]["leak_mass_flow"]
        .as_f64()
        .unwrap();
    assert!(
        leaked > 0.0,
        "PuncturePipe reached the engine but no mass left through the hole \
         (leak_mass_flow = {leaked}) — this is M6.0's defect, at the bridge"
    );

    // SetHeatInput: a fire on the receiving tank raises its temperature.
    let mut sim = bridge(LEAKY);
    let sink_tank = sim.node_id("receiving_tank").unwrap();
    let before = {
        sim.tick().unwrap();
        snapshot_value(&sim)["nodes"][sink_tank as usize]["kind"]["temperature"]
            .as_f64()
            .unwrap()
    };
    sim.apply_command_json(&format!(
        r#"{{"cmd":"set_heat_input","node":{sink_tank},"power":5000000.0}}"#
    ))
    .expect("heat input accepted");
    for _ in 0..20 {
        sim.tick().unwrap();
    }
    let after = snapshot_value(&sim)["nodes"][sink_tank as usize]["kind"]["temperature"]
        .as_f64()
        .unwrap();
    assert!(
        after > before,
        "SetHeatInput did not heat the tank ({before} K -> {after} K)"
    );
    // The two duty commands need their own units.
    let mut furnace = bridge("furnace_heater.toml");
    let heater = furnace.node_id("heater").unwrap();
    furnace
        .apply_command_json(&format!(
            r#"{{"cmd":"set_furnace_duty","node":{heater},"duty":250000.0}}"#
        ))
        .expect("furnace duty accepted");
    assert_eq!(
        snapshot_value(&furnace)["nodes"][heater as usize]["kind"]["duty"]
            .as_f64()
            .unwrap(),
        250_000.0,
        "SetFurnaceDuty did not reach the furnace"
    );

    let mut cooler = bridge("cooler_chiller.toml");
    let chiller = cooler.node_id("chiller").unwrap();
    cooler
        .apply_command_json(&format!(
            r#"{{"cmd":"set_cooler_duty","node":{chiller},"duty":250000.0}}"#
        ))
        .expect("cooler duty accepted");
    assert_eq!(
        snapshot_value(&cooler)["nodes"][chiller as usize]["kind"]["duty"]
            .as_f64()
            .unwrap(),
        250_000.0,
        "SetCoolerDuty did not reach the cooler"
    );
}

// ------------------------------------------------------- the trust boundary

/// `Engine::apply` **refuses** an out-of-range node or edge id (M27).
///
/// Until M27 this was a characterization test asserting that it PANICKED, the
/// behaviour the bridge's guard existed to keep unreachable. `core` now checks
/// every node and edge id a command names before any arm touches the graph
/// (rule 5), so the engine half of the trust boundary is an ordinary
/// `InvalidCommand`. Both kinds of id are asserted because they are two checks:
/// deleting the edge half alone used to leave `PuncturePipe` panicking.
///
/// The bridge's own guard stays — it is what gives a stale id the `unknown_id`
/// code rather than `invalid_command` (see `src/bridge.rs`).
#[test]
fn core_refuses_an_out_of_range_id() {
    let file = refinery_scenarios::load_str(&scenario_src(LEAKY)).unwrap();
    let mut engine = refinery_scenarios::build_engine(&file).unwrap();

    let commands = [
        (
            Command::SetPumpOn {
                node: NodeId(9999),
                on: false,
            },
            "NodeId(9999) names no node",
        ),
        // The trip guard reads the node's NAME for its message before the arm's
        // own lookup, so this one reached the graph first.
        (
            Command::SetValveOpening {
                node: NodeId(9999),
                opening: 0.5,
            },
            "NodeId(9999) names no node",
        ),
        (
            Command::PuncturePipe {
                edge: EdgeId(9999),
                area: SquareMeter(1.0e-4),
            },
            "EdgeId(9999) names no pipe",
        ),
    ];
    for (cmd, expected) in commands {
        let what = format!("{cmd:?}");
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| engine.apply(cmd)));
        match outcome {
            Ok(Err(SimError::InvalidCommand(message))) => assert!(
                message.contains(expected),
                "{what}: refused, but not by the id check: {message}"
            ),
            Ok(other) => panic!("{what}: expected an InvalidCommand, got {other:?}"),
            Err(_) => panic!("{what}: core panicked on an out-of-range id (rule 5)"),
        }
    }
}

#[test]
fn an_out_of_range_id_is_refused_by_the_bridge_not_forwarded() {
    for (json, what) in [
        (r#"{"cmd":"set_pump_on","node":9999,"on":false}"#, "node"),
        (
            r#"{"cmd":"puncture_pipe","edge":9999,"area":0.0001}"#,
            "edge",
        ),
    ] {
        let mut sim = bridge(LEAKY);
        let before = sim.snapshot_json();
        let err = sim
            .apply_command_json(json)
            .expect_err("an out-of-range id must be refused");
        assert_eq!(
            ErrorReport::from(&err).code,
            "unknown_id",
            "wrong code for an out-of-range {what} id"
        );
        assert_eq!(
            sim.snapshot_json(),
            before,
            "a refused command changed the plant"
        );
    }
}

#[test]
fn malformed_and_unknown_commands_are_refused_without_touching_the_engine() {
    let cases = [
        ("not json at all", "bad_json"),
        (r#"{"cmd":"self_destruct","node":0}"#, "bad_json"),
        (r#"{"cmd":"set_pump_on","node":1}"#, "bad_json"), // missing field
        (
            r#"{"cmd":"set_valve_opening","node":1,"opening":2.0}"#,
            "invalid_command",
        ), // valid id, bad value
        (
            r#"{"cmd":"set_pump_on","node":0,"on":true}"#,
            "invalid_command",
        ), // valid id, not a pump
    ];
    for (json, expected_code) in cases {
        let mut sim = bridge(LEAKY);
        let before = sim.snapshot_json();
        let err = match sim.apply_command_json(json) {
            Err(err) => err,
            Ok(()) => panic!("expected a refusal for {json}, got Ok"),
        };
        assert_eq!(
            ErrorReport::from(&err).code,
            expected_code,
            "wrong code for {json}"
        );
        assert_eq!(
            sim.snapshot_json(),
            before,
            "a refused command changed the plant: {json}"
        );
    }
}

// ----------------------------------------------------------- names and ids

#[test]
fn names_resolve_to_the_ids_the_snapshot_reports() {
    let sim = bridge(LEAKY);
    let snapshot = snapshot_value(&sim);

    for node in snapshot["nodes"].as_array().unwrap() {
        let name = node["name"].as_str().unwrap();
        assert_eq!(
            sim.node_id(name).unwrap(),
            node["id"].as_i64().unwrap(),
            "node '{name}' resolves to the wrong id"
        );
    }
    for edge in snapshot["edges"].as_array().unwrap() {
        let name = edge["name"].as_str().unwrap();
        assert_eq!(
            sim.edge_id(name).unwrap(),
            edge["id"].as_i64().unwrap(),
            "edge '{name}' resolves to the wrong id"
        );
    }

    // Every name is listed, and nothing else is.
    assert_eq!(
        sim.node_names().len(),
        snapshot["nodes"].as_array().unwrap().len()
    );
    assert_eq!(
        sim.edge_names().len(),
        snapshot["edges"].as_array().unwrap().len()
    );

    // An unknown name is an error, not a panic and not a plausible id.
    let err = sim
        .node_id("no_such_node")
        .expect_err("unknown name refused");
    assert_eq!(ErrorReport::from(&err).code, "unknown_name");
    let err = sim
        .edge_id("no_such_edge")
        .expect_err("unknown name refused");
    assert_eq!(ErrorReport::from(&err).code, "unknown_name");
}

/// M6.1 splits a declared punctureable pipe into two halves plus an orifice.
/// The declared name must resolve to the UPSTREAM half — the edge
/// `PuncturePipe` addresses and the one carrying `leak_mass_flow` (DESIGN
/// §3b). Resolving to the downstream half would be silently wrong: both are
/// pipes, both carry flow, and only the leak number would give it away.
#[test]
fn the_declared_pipe_name_resolves_to_the_upstream_half() {
    let mut sim = bridge(LEAKY);
    let declared = sim.edge_id("fill_line").unwrap();
    let downstream = sim.edge_id("fill_line__downstream").unwrap();
    let orifice = sim.edge_id("fill_line__leak").unwrap();
    assert_ne!(declared, downstream);
    assert_ne!(declared, orifice);

    let snapshot = snapshot_value(&sim);
    let junction = snapshot["edges"][declared as usize]["to"].as_i64().unwrap();
    assert_eq!(
        snapshot["nodes"][junction as usize]["kind"]["type"], "junction",
        "the declared name is not the half that ends at the leak junction"
    );
    assert_eq!(
        snapshot["edges"][downstream as usize]["from"]
            .as_i64()
            .unwrap(),
        junction,
        "the downstream half does not start at the same junction"
    );

    // And it is the edge the leak flow is reported on.
    sim.apply_command_json(&format!(
        r#"{{"cmd":"puncture_pipe","edge":{declared},"area":0.0005}}"#
    ))
    .unwrap();
    sim.tick().unwrap();
    let snapshot = snapshot_value(&sim);
    assert!(
        snapshot["edges"][declared as usize]["leak_mass_flow"]
            .as_f64()
            .unwrap()
            > 0.0,
        "the declared name resolves to an edge that reports no leak"
    );
    assert_eq!(
        snapshot["edges"][downstream as usize]["leak_mass_flow"]
            .as_f64()
            .unwrap(),
        0.0
    );
}

/// Name uniqueness is a **requirement** of the lookup API — `Bridge::load`
/// refuses a plant that breaks it, because a duplicated name makes
/// `node_id`/`edge_id` ambiguous. The sweep over the repo's scenarios is
/// evidence that the requirement is not onerous in practice; it is not what
/// makes the requirement true.
#[test]
fn every_scenario_in_the_repo_loads_with_unique_names() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
    let mut checked = 0;
    let mut files: Vec<_> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "toml"))
        .collect();
    files.sort(); // deterministic order, and a deterministic failure message

    for path in files {
        let src = std::fs::read_to_string(&path).unwrap();
        let sim =
            Bridge::load(&src).unwrap_or_else(|e| panic!("{} failed to load: {e}", path.display()));
        let snapshot: Value = serde_json::from_str(&sim.snapshot_json()).unwrap();
        assert_eq!(
            sim.node_names().len(),
            snapshot["nodes"].as_array().unwrap().len(),
            "{} has duplicate node names",
            path.display()
        );
        assert_eq!(
            sim.edge_names().len(),
            snapshot["edges"].as_array().unwrap().len(),
            "{} has duplicate edge names",
            path.display()
        );
        checked += 1;
    }
    assert!(
        checked >= 10,
        "only {checked} scenarios swept — expected the whole folder"
    );
}

// ------------------------------------------------------------- the snapshot

/// Collect the JSON paths whose value is `null`, with array indices erased so
/// the result is a claim about FIELDS rather than about how many nodes the
/// fixture happens to have.
fn null_paths(value: &Value, path: &str, out: &mut Vec<String>) {
    match value {
        Value::Null => out.push(path.to_string()),
        Value::Array(items) => {
            for item in items {
                null_paths(item, &format!("{path}[]"), out);
            }
        }
        Value::Object(fields) => {
            for (key, item) in fields {
                let child = if path.is_empty() {
                    key.clone()
                } else {
                    format!("{path}.{key}")
                };
                null_paths(item, &child, out);
            }
        }
        _ => {}
    }
}

fn snapshot_value(sim: &Bridge) -> Value {
    serde_json::from_str(&sim.snapshot_json()).expect("snapshot JSON parses as generic JSON")
}

/// Before the first tick, exactly three fields are `null` — the three the
/// engine documents as NaN until a solve has happened.
///
/// The gate is the field SET, not "the round trip fails": a round-trip
/// assertion would keep passing if a different field started emitting null,
/// or if serde changed which error it raised. The failed round trip is a
/// consequence, checked below as one.
#[test]
fn pre_tick_json_is_null_in_exactly_the_documented_fields() {
    let sim = bridge(LEAKY);
    assert_eq!(sim.tick_index(), 0, "fixture has already ticked");

    let mut found = Vec::new();
    null_paths(&snapshot_value(&sim), "", &mut found);
    found.sort();
    found.dedup();

    assert_eq!(
        found,
        vec![
            "edges[].dissipation_w".to_string(),
            "nodes[].pressure_pa".to_string(),
            "nodes[].temperature_k".to_string(),
        ],
        "the set of not-yet-solved fields changed; a scene reading any new one \
         gets null where it expected a number"
    );

    // The consequence, stated once: this JSON is emit-only until a tick.
    assert!(
        serde_json::from_str::<Snapshot>(&sim.snapshot_json()).is_err(),
        "pre-tick JSON round-trips now — which would mean the nulls are gone"
    );

    // A stored number is NOT a solved one: the tank's own temperature is real
    // while the node's resolved temperature is null. Both are correct.
    let snapshot = snapshot_value(&sim);
    let tank = snapshot["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .find(|n| n["kind"]["type"] == "tank")
        .unwrap();
    assert!(tank["temperature_k"].is_null());
    assert!(tank["kind"]["temperature"].as_f64().unwrap() > 0.0);
}

/// The slate crosses the bridge, and it is real BEFORE the first tick (M8.5).
///
/// Both halves matter to a scene. That the slate crosses at all is what lets
/// `demo/plant.gd` draw a fill level instead of mass on a shared scale — the
/// M6.2 deferral. That it is real pre-tick is what lets a scene lay itself out
/// in `_ready()`, before `_physics_process` has run once: like a name or a
/// tank's own temperature, the slate is a STORED quantity and not a solved one,
/// so it is the wrong side of this file's `null` boundary to be NaN.
///
/// The arithmetic itself is gated on the engine side
/// (`scenarios/tests/snapshot_slate.rs`); what is bridge-specific is that the
/// keys survive the crossing, since this JSON is a scene's only channel.
#[test]
fn the_slate_crosses_the_bridge_and_is_real_before_the_first_tick() {
    let sim = bridge(LEAKY);
    assert_eq!(sim.tick_index(), 0, "fixture has already ticked");
    let snapshot = snapshot_value(&sim);

    let slate = snapshot["slate"]
        .as_array()
        .expect("a snapshot carries a slate");
    assert_eq!(slate.len(), 1, "leaking_line is water only");
    assert_eq!(slate[0]["name"], "water");
    assert_eq!(
        slate[0]["density_kg_per_m3"].as_f64().unwrap(),
        998.0,
        "water's density crosses unrounded"
    );

    // The scene's own computation, over the keys the scene reads: a tank
    // declared at 8.0 m in a 10 m shell draws 80% full, with no tick required.
    let tank = snapshot["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .find(|n| n["name"] == "supply_tank")
        .expect("leaking_line has a supply_tank")
        .clone();
    let fractions = tank["kind"]["composition"]["mass_fractions"]
        .as_array()
        .unwrap();
    let inverse: f64 = fractions
        .iter()
        .zip(slate)
        .filter(|(f, _)| f.as_f64().unwrap() > 0.0)
        .map(|(f, c)| f.as_f64().unwrap() / c["density_kg_per_m3"].as_f64().unwrap())
        .sum();
    let level = tank["kind"]["mass"].as_f64().unwrap()
        / ((1.0 / inverse) * tank["kind"]["area"].as_f64().unwrap());
    // The round trip is `(ρ·A·h)/(ρ·A)` in f64 — exact but for two roundings,
    // so the bound is a few ULP rather than a chosen tolerance.
    assert!(
        (level - 8.0).abs() < 1e-11,
        "supply_tank is declared at 8.0 m; the snapshot reconstructs {level}"
    );
    let fraction = level / tank["kind"]["height"].as_f64().unwrap();
    assert!(
        (fraction - 0.80).abs() < 1e-11,
        "8 m in a 10 m shell draws 80% full; got {fraction}"
    );
}

/// After a solve there are no nulls, and the JSON is **exactly** reversible.
///
/// The exactness half is load-bearing beyond this crate: it is what pins
/// `serde_json`'s `float_roundtrip` feature in the workspace manifest. Without
/// it the default parser lands 1–2 ULP off for some values, and a saved
/// snapshot would not reload as the state it was written from. Swept over
/// several plants and several ticks rather than one snapshot, because whether
/// a given float survives the default parser is luck — the first version of
/// this gate caught it on exactly two edge floats out of ~90.
#[test]
fn post_tick_json_has_no_nulls_and_round_trips_exactly() {
    for plant in [LEAKY, "tank_pump_valve.toml", "furnace_heater.toml"] {
        let mut sim = bridge(plant);
        for tick in 1..=25 {
            sim.tick()
                .unwrap_or_else(|e| panic!("{plant} tick {tick}: {e}"));

            let json = sim.snapshot_json();
            let mut found = Vec::new();
            null_paths(&snapshot_value(&sim), "", &mut found);
            assert!(found.is_empty(), "{plant} tick {tick}: null at {found:?}");

            let parsed: Snapshot = serde_json::from_str(&json)
                .unwrap_or_else(|e| panic!("{plant} tick {tick} does not parse: {e}"));
            assert_eq!(
                serde_json::to_string(&parsed).unwrap(),
                json,
                "{plant} tick {tick}: the JSON round trip is lossy — check that \
                 serde_json still has the `float_roundtrip` feature"
            );
        }
        assert_eq!(sim.tick_index(), 25);
    }
}

/// The bridge is a pass-through, not a second engine: driving it must give
/// byte-identical snapshots to driving `Engine` directly. This is what makes
/// every physics gate in `scenarios/tests` apply to the frontend too.
#[test]
fn the_bridge_reproduces_a_direct_engine_run_byte_for_byte() {
    let src = scenario_src(LEAKY);

    let file = refinery_scenarios::load_str(&src).unwrap();
    let mut direct = refinery_scenarios::build_engine(&file).unwrap();
    let mut sim = Bridge::load(&src).unwrap();

    let valve_direct = Command::SetValveOpening {
        node: NodeId(sim.node_id("discharge_valve").unwrap() as u32),
        opening: 0.3,
    };
    direct.apply(valve_direct).unwrap();
    sim.apply_command_json(&format!(
        r#"{{"cmd":"set_valve_opening","node":{},"opening":0.3}}"#,
        sim.node_id("discharge_valve").unwrap()
    ))
    .unwrap();

    for tick in 1..=50 {
        direct.tick().unwrap();
        sim.tick().unwrap();
        assert_eq!(
            serde_json::to_string(&direct.snapshot()).unwrap(),
            sim.snapshot_json(),
            "bridge and direct runs diverged at tick {tick}"
        );
    }
}

// ------------------------------------------------------------- error codes

/// Codes are a frontend contract: a scene branches on them. Every `SimError`
/// variant is constructed here, so a renamed code fails, and the mapping in
/// `src/bridge.rs` is itself a wildcard-free match, so a NEW variant fails to
/// compile rather than falling into a catch-all.
#[test]
fn error_codes_are_stable() {
    let cases: Vec<(BridgeError, &str)> = vec![
        (BridgeError::BadJson("x".into()), "bad_json"),
        (BridgeError::NotLoaded, "not_loaded"),
        (BridgeError::FileUnreadable("x".into()), "file_unreadable"),
        (BridgeError::UnknownId("x".into()), "unknown_id"),
        (BridgeError::UnknownName("x".into()), "unknown_name"),
        (BridgeError::DuplicateName("x".into()), "duplicate_name"),
        (
            BridgeError::Sim(SimError::SolverDiverged {
                iterations: 7,
                residual: 1.5,
                residual_history: vec![9.0, 3.0, 1.5],
            }),
            "solver_diverged",
        ),
        (
            BridgeError::Sim(SimError::NonFiniteState {
                location: "somewhere".into(),
            }),
            "non_finite_state",
        ),
        (
            BridgeError::Sim(SimError::InvalidCommand("x".into())),
            "invalid_command",
        ),
        (BridgeError::Sim(SimError::Scenario("x".into())), "scenario"),
        (
            BridgeError::Sim(SimError::Numerical("x".into())),
            "numerical",
        ),
    ];

    for (err, expected) in &cases {
        let report = ErrorReport::from(err);
        assert_eq!(&report.code, expected, "code changed for {err:?}");
        assert!(!report.message.is_empty(), "empty message for {err:?}");
    }

    // The diverged report carries the summary and NOT the history — the
    // decision recorded in `BridgeError`'s docs, gated so it stays true.
    // Found by code rather than by index: a variant added above must not
    // silently re-point this at a different error.
    let diverged = ErrorReport::from(
        &cases
            .iter()
            .find(|(_, code)| *code == "solver_diverged")
            .expect("the diverged case is still in the list")
            .0,
    );
    assert!(
        diverged.message.contains('7') && diverged.message.contains("1.500e0"),
        "the diverged summary lost its iterations/residual: {}",
        diverged.message
    );
    assert!(
        !diverged.message.contains("9"),
        "the residual history leaked into the game-facing payload: {}",
        diverged.message
    );
}

// ----------------------------------------------------------------- Session

// `Session` is what the gdext binding calls, method for method. Everything
// below therefore gates code that no test can otherwise reach: the binding's
// own lines are `GString` marshalling with the decisions removed, so gating
// `Session` is gating the binding's behaviour.

fn outcome_code(json: &str) -> Option<String> {
    let value: Value = serde_json::from_str(json).expect("an outcome is JSON");
    match value {
        Value::Null => None,
        other => Some(
            other["code"]
                .as_str()
                .expect("an outcome that is not null carries a code")
                .to_string(),
        ),
    }
}

/// A Godot node exists before it is told what to simulate, so every call has
/// to answer in that state. **All of them, not the ones someone remembered**:
/// the point is that a scene calling any method on a fresh node gets a
/// documented value instead of a panic or a plausible-looking lie.
#[test]
fn every_session_call_answers_before_a_scenario_is_loaded() {
    let mut sim = Session::new();

    assert!(!sim.is_loaded());
    assert_eq!(sim.tick_index(), MISSING_ID, "unloaded must not read as 0");
    assert_eq!(sim.node_id("supply_tank"), MISSING_ID);
    assert_eq!(sim.edge_id("fill_line"), MISSING_ID);
    assert_eq!(sim.snapshot_json(), "null");
    assert_eq!(sim.node_names_json(), "null");
    assert_eq!(sim.edge_names_json(), "null");

    assert_eq!(outcome_code(&sim.tick()).as_deref(), Some("not_loaded"));
    assert_eq!(
        outcome_code(&sim.apply_command_json(r#"{"cmd":"set_pump_on","node":1,"on":false}"#))
            .as_deref(),
        Some("not_loaded"),
        "a command must not be judged before there is a plant to judge it against"
    );

    // And loading turns all of that on.
    assert_eq!(outcome_code(&sim.load_scenario(&scenario_src(LEAKY))), None);
    assert!(sim.is_loaded());
    assert_eq!(sim.tick_index(), 0, "loaded but unsolved is 0, not -1");
    assert!(sim.node_id("supply_tank") >= 0);
    assert_ne!(sim.snapshot_json(), "null");
}

/// Success is the JSON `null`, failure is an object with a code. One shape,
/// so a scene parses once and branches on truthiness — the contract the
/// binding's return type rests on.
#[test]
fn a_session_outcome_is_null_or_a_coded_object() {
    let mut sim = Session::new();
    assert_eq!(sim.load_scenario(&scenario_src(LEAKY)), "null");
    assert_eq!(sim.tick(), "null");

    let refused = sim.apply_command_json("not json at all");
    let value: Value = serde_json::from_str(&refused).expect("a refusal is JSON");
    assert_eq!(value["code"], "bad_json");
    assert!(
        value["message"].as_str().is_some_and(|m| !m.is_empty()),
        "a refusal with no message tells a log nothing: {refused}"
    );
}

/// A typo in a scenario path must not destroy a running game. The previously
/// loaded plant survives a failed load **unchanged** — same rule a refused
/// `Command` follows, and the only reason this is testable at all is that the
/// decision lives here rather than in the binding.
#[test]
fn a_failed_load_leaves_the_running_plant_untouched() {
    let mut sim = Session::new();
    sim.load_scenario(&scenario_src(LEAKY));
    for _ in 0..10 {
        assert_eq!(sim.tick(), "null");
    }
    let before = sim.snapshot_json();

    assert_eq!(
        outcome_code(&sim.load_scenario("this is not toml")).as_deref(),
        Some("scenario")
    );
    assert!(sim.is_loaded(), "a failed load unloaded the running plant");
    assert_eq!(sim.tick_index(), 10);
    assert_eq!(
        sim.snapshot_json(),
        before,
        "a failed load disturbed the running plant"
    );

    // A successful load, by contrast, does replace it.
    assert_eq!(
        outcome_code(&sim.load_scenario(&scenario_src("tank_pump_valve.toml"))),
        None
    );
    assert_eq!(sim.tick_index(), 0);
}

/// The names lookups are the same phone book, in the shape that crosses.
#[test]
fn session_name_lists_match_the_bridges() {
    let mut sim = Session::new();
    sim.load_scenario(&scenario_src(LEAKY));
    let direct = bridge(LEAKY);

    let nodes: Vec<String> = serde_json::from_str(&sim.node_names_json()).unwrap();
    let edges: Vec<String> = serde_json::from_str(&sim.edge_names_json()).unwrap();
    assert_eq!(nodes, direct.node_names());
    assert_eq!(edges, direct.edge_names());
    assert!(edges.contains(&"fill_line__leak".to_string()));

    for name in &nodes {
        assert_eq!(sim.node_id(name), direct.node_id(name).unwrap());
    }
    assert_eq!(
        sim.node_id("no_such_node"),
        MISSING_ID,
        "an unknown name must not resolve to a plausible id"
    );
}

/// The binding's whole claim to correctness is that `Session` is a
/// pass-through. Driving one against a direct `Engine` for 50 ticks, with a
/// command applied, is that claim as a gate — the `Bridge`-level version of
/// this test does not cover the `Option` layer the binding actually talks to.
#[test]
fn a_session_reproduces_a_direct_engine_run_byte_for_byte() {
    let src = scenario_src(LEAKY);

    let file = refinery_scenarios::load_str(&src).unwrap();
    let mut direct = refinery_scenarios::build_engine(&file).unwrap();
    let mut sim = Session::new();
    assert_eq!(sim.load_scenario(&src), "null");

    let pipe = sim.edge_id("fill_line");
    direct
        .apply(Command::PuncturePipe {
            edge: EdgeId(pipe as u32),
            area: SquareMeter(5.0e-4),
        })
        .unwrap();
    assert_eq!(
        sim.apply_command_json(&format!(
            r#"{{"cmd":"puncture_pipe","edge":{pipe},"area":0.0005}}"#
        )),
        "null"
    );

    for tick in 1..=50 {
        direct.tick().unwrap();
        assert_eq!(sim.tick(), "null");
        assert_eq!(
            serde_json::to_string(&direct.snapshot()).unwrap(),
            sim.snapshot_json(),
            "session and direct runs diverged at tick {tick}"
        );
    }
    assert_eq!(sim.tick_index(), 50);
}

#[test]
fn a_scenario_that_does_not_load_reports_as_a_scenario_error() {
    let err = match Bridge::load("this is not toml") {
        Err(err) => err,
        Ok(_) => panic!("garbage TOML loaded as a plant"),
    };
    assert_eq!(ErrorReport::from(&err).code, "scenario");
}
