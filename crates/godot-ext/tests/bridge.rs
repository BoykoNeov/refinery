//! Gates for the translation layer (M6.2, DESIGN §8).
//!
//! What is being pinned here is a **contract**, not physics: the JSON text a
//! frontend writes, the ids it may send, the codes it branches on, and the
//! shape of the snapshot it reads. Physics is gated in `scenarios/tests`.

use refinery_core::graph::{EdgeId, NodeId};
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::units::{SquareMeter, Watt};
use refinery_core::SimError;
use refinery_godot_ext::bridge::{Bridge, BridgeError, ErrorReport};
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
    assert_eq!(every_variant().len(), 6, "a Command variant was added");
}

// -------------------------------------------- commands reach a real engine

#[test]
fn every_command_variant_reaches_the_engine_and_changes_it() {
    // Valve, pump and puncture, on M6.1's leaky plant.
    let mut sim = bridge(LEAKY);
    let valve = sim.node_id("discharge_valve").unwrap();
    let pump = sim.node_id("transfer_pump").unwrap();
    let tank = sim.node_id("supply_tank").unwrap();
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
    let _ = tank; // supply_tank resolves; its own heating is furnace_reference's gate

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

/// Characterization test: `Engine::apply` **panics** on an out-of-range id.
///
/// This is not a wish. It is the behaviour the bridge's id validation exists
/// to keep unreachable, recorded so the guard cannot quietly become
/// decorative — if `core` ever starts returning `Err` here, this test fails
/// and the guard's justification (and the deferral in `bridge.rs`'s module
/// docs) gets revisited on purpose rather than by accident.
#[test]
fn core_panics_on_an_out_of_range_id() {
    let file = refinery_scenarios::load_str(&scenario_src(LEAKY)).unwrap();
    let mut engine = refinery_scenarios::build_engine(&file).unwrap();

    let hushed = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        engine.apply(Command::SetPumpOn {
            node: NodeId(9999),
            on: false,
        })
    }));
    std::panic::set_hook(hushed);

    assert!(
        outcome.is_err(),
        "core no longer panics on an out-of-range node id — good news, but the \
         bridge's guard and its written deferral now need re-deciding \
         (see src/bridge.rs)"
    );
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
    let diverged = ErrorReport::from(&cases[4].0);
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

#[test]
fn a_scenario_that_does_not_load_reports_as_a_scenario_error() {
    let err = match Bridge::load("this is not toml") {
        Err(err) => err,
        Ok(_) => panic!("garbage TOML loaded as a plant"),
    };
    assert_eq!(ErrorReport::from(&err).code, "scenario");
}
