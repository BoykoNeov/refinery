//! The fire, as a frontend can SEE it (M6.2): `NodeSnapshot::heat_input_w`.
//!
//! **The gap this closes, and why it is a gap rather than a design.**
//! `Command::SetHeatInput` is the damage model's fire (DESIGN §7) and it works
//! — `energy::heat_load` sums it into every node's heat balance and the
//! temperature responds. Nothing *reported* it. A frontend could watch a tank
//! warm up and infer a fire, or remember having sent the command and assume
//! one, but it could not read the engine's own answer to "is this node on
//! fire?". `EdgeSnapshot::leak_mass_flow` exists for exactly the analogous
//! question about the other half of the damage model, which is what makes the
//! asymmetry unintended rather than a decision.
//!
//! It matters beyond tidiness because of what a scene does with the answer. A
//! scene drawing flames from its own memory of having sent the command keeps
//! drawing them after a reload, after a refused command, or after anything
//! clears the field — a picture of what the frontend did, not of what the
//! engine holds. That is M6.0's defect (a stored number nothing consumes) with
//! the direction reversed, and it would have shipped inside the milestone that
//! found it.
//!
//! **The trap these gates exist for.** `energy::heat_load(node)` returns the
//! fire PLUS the node's own unit term — a furnace's duty, a cooler's negative
//! duty, a tank's ambient exchange. Snapshotting that sum would be the obvious
//! implementation and would report every furnace in every scenario as being on
//! fire. `a_working_furnace_is_not_on_fire` is the gate that discriminates;
//! `engine.rs`'s own comment at the `heat_load` call site warns about the same
//! confusion from the other side.

use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::units::Watt;

const LEAKY: &str = include_str!("../../../scenarios/leaking_line.toml");
const FURNACE: &str = include_str!("../../../scenarios/furnace_heater.toml");
const COOLER: &str = include_str!("../../../scenarios/cooler_chiller.toml");

/// 5 MW onto the ~20 t receiving tank. Sized against the rise it has to
/// produce, not by feel: 5 MW for [`BURN_TICKS`] ticks is 150 MJ, which is
/// ~1.7 K on 20 t of water — comfortably past the 1 K the gate demands, and
/// nowhere near the absolute-zero or boiling guards.
const FIRE_W: f64 = 5.0e6;

/// 30 s at the scenario's 0.1 s timestep.
const BURN_TICKS: usize = 300;

fn build(src: &str) -> refinery_core::Engine {
    let file = refinery_scenarios::load_str(src).expect("scenario loads");
    refinery_scenarios::build_engine(&file).expect("engine builds")
}

fn node<'a>(snapshot: &'a Snapshot, name: &str) -> &'a refinery_core::snapshot::NodeSnapshot {
    snapshot
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("no node named '{name}'"))
}

/// A fire is reported by the engine, from the moment it is set until it is
/// put out — and it is a *stored* quantity, so it is a real number before the
/// first tick rather than the NaN the solved fields carry.
#[test]
fn a_fire_is_visible_in_the_snapshot_from_the_moment_it_is_set() {
    let mut engine = build(LEAKY);
    let target = node(&engine.snapshot(), "receiving_tank").id;

    // Pre-tick, and with no fire: a real zero, not NaN.
    let before = engine.snapshot();
    let tank = node(&before, "receiving_tank");
    assert_eq!(before.tick, 0);
    assert_eq!(
        tank.heat_input_w, 0.0,
        "an undamaged node reports a fire it does not have"
    );
    assert!(
        tank.pressure_pa.is_nan() && tank.temperature_k.is_nan(),
        "the fixture is no longer pre-tick, so the stored-vs-solved contrast \
         below proves nothing"
    );

    // Set the fire. Visible immediately — before any tick has processed it,
    // because it is state the command wrote, not a number a solve produced.
    engine
        .apply(Command::SetHeatInput {
            node: target,
            power: Watt(FIRE_W),
        })
        .expect("fire accepted");
    assert_eq!(
        node(&engine.snapshot(), "receiving_tank").heat_input_w,
        FIRE_W
    );
    assert_eq!(
        engine.snapshot().tick,
        0,
        "reporting the fire should not have required a tick"
    );

    // It survives ticking, and the temperature moves with it — the two halves
    // of "the reported number is the one the physics is using".
    let cold = {
        let mut undamaged = build(LEAKY);
        for _ in 0..BURN_TICKS {
            undamaged.tick().expect("undamaged run");
        }
        node(&undamaged.snapshot(), "receiving_tank").temperature_k
    };
    for _ in 0..BURN_TICKS {
        engine.tick().expect("burning run");
    }
    let snapshot = engine.snapshot();
    let burnt = node(&snapshot, "receiving_tank");
    assert_eq!(
        burnt.heat_input_w, FIRE_W,
        "the reported fire changed on its own across the burn"
    );
    assert!(
        burnt.temperature_k > cold + 1.0,
        "the tank reporting a {FIRE_W} W fire is no hotter than the undamaged \
         one ({:.3} K vs {cold:.3} K), so the reported number is not the one \
         the energy balance used",
        burnt.temperature_k
    );

    // Put it out: back to a reported zero, and the reported zero is the truth
    // (a repaired node must not keep drawing flames).
    engine
        .apply(Command::SetHeatInput {
            node: target,
            power: Watt::ZERO,
        })
        .expect("fire extinguished");
    assert_eq!(node(&engine.snapshot(), "receiving_tank").heat_input_w, 0.0);
}

/// **The discriminating gate.** A furnace at its setpoint and a furnace with a
/// fire on it are different states, and `heat_load()` — the sum — cannot tell
/// them apart. Wiring the snapshot to that function instead of to the raw
/// field passes every other test in this file and fails this one.
#[test]
fn a_working_furnace_is_not_on_fire() {
    for (src, unit, kind) in [
        (FURNACE, "heater", "furnace"),
        (COOLER, "chiller", "cooler"),
    ] {
        let mut engine = build(src);
        engine.tick().expect("first tick");
        let snapshot = engine.snapshot();
        let unit_node = node(&snapshot, unit);

        // The unit is doing real work — otherwise this proves nothing.
        let duty = serde_json::to_value(&unit_node.kind).expect("kind serializes")["duty"]
            .as_f64()
            .unwrap_or_else(|| panic!("{kind} '{unit}' has no duty to be confused with a fire"));
        assert!(
            duty > 0.0,
            "{kind} '{unit}' is idle, so a heat_load()-based snapshot would \
             report 0.0 here too and this gate would be vacuous"
        );

        assert_eq!(
            unit_node.heat_input_w, 0.0,
            "{kind} '{unit}' reports a {} W fire while merely running at its \
             {duty} W setpoint — the snapshot is wired to energy::heat_load() \
             (fire + unit duty) instead of to node.heat_input (fire alone)",
            unit_node.heat_input_w
        );

        // And a fire on that same unit stacks ON TOP of the setpoint rather
        // than replacing it: both numbers stay separately readable.
        let id = unit_node.id;
        engine
            .apply(Command::SetHeatInput {
                node: id,
                power: Watt(FIRE_W),
            })
            .expect("fire accepted");
        let snapshot = engine.snapshot();
        let unit_node = node(&snapshot, unit);
        assert_eq!(unit_node.heat_input_w, FIRE_W);
        assert_eq!(
            serde_json::to_value(&unit_node.kind).expect("kind serializes")["duty"]
                .as_f64()
                .unwrap(),
            duty,
            "the fire overwrote the operator's setpoint"
        );
    }
}

/// A tank's ambient exchange is the third term `heat_load()` folds in, and it
/// needs its own arm: it is nonzero on a plant with no unit and no fire at
/// all, so the furnace arm above cannot reach it.
///
/// **Built inline rather than loaded, and that is the point.** No scenario in
/// the repo sets a nonzero `ambient_exchange_ua_w_per_k` on a tank, so an arm written
/// against the existing files would report 0.0 for the right reason and pass
/// for the wrong one. The plant below is `tank_pump_valve` with a hot supply
/// tank losing heat to the air — checked to be actually losing it, so the
/// term this gate discriminates against is genuinely present.
#[test]
fn ambient_exchange_is_not_reported_as_a_fire() {
    const AMBIENT: &str = r#"
[meta]
name = "ambient_tank"
description = "A hot tank cooling to the air, with no fire anywhere."

[simulation]
dt = 0.1

[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"

[nodes.supply_tank]
type = "tank"
area_m2 = 20.0
height_m = 10.0
initial_level_m = 8.0
temperature_c = 80.0
ambient_exchange_ua_w_per_k = 200000.0

[nodes.transfer_pump]
type = "pump"
h0_m = 40.0
a = 800.0
on = true

[nodes.discharge_valve]
type = "valve"
kv = 50.0
opening = 0.5

[nodes.receiving_tank]
type = "tank"
area_m2 = 20.0
height_m = 10.0
initial_level_m = 1.0
temperature_c = 20.0

[[pipes]]
name = "suction"
from = "supply_tank"
to = "transfer_pump"
length_m = 10.0
diameter_m = 0.15

[[pipes]]
name = "discharge"
from = "transfer_pump"
to = "discharge_valve"
length_m = 30.0
diameter_m = 0.10

[[pipes]]
name = "fill_line"
from = "discharge_valve"
to = "receiving_tank"
length_m = 20.0
diameter_m = 0.10
elevation_change_m = 5.0
"#;

    let mut engine = build(AMBIENT);
    let start = {
        engine.tick().expect("first tick");
        node(&engine.snapshot(), "supply_tank").temperature_k
    };
    // 200 s. 200 kW/K across a 60 K gap is ~12 MW, so ~2.4 GJ leaves a 160 t
    // tank: a few K, which is what makes the "it really is exchanging" half of
    // this gate hold with room to spare.
    for _ in 0..2000 {
        engine.tick().expect("cooling run");
    }
    let snapshot = engine.snapshot();
    let tank = node(&snapshot, "supply_tank");
    assert!(
        tank.temperature_k < start - 1.0,
        "the hot tank did not cool ({start:.3} K -> {:.3} K), so its ambient \
         term is not active and this gate discriminates nothing",
        tank.temperature_k
    );
    assert_eq!(
        tank.heat_input_w, 0.0,
        "a tank exchanging {:.1} W/K with the air reports a fire — its ambient \
         term is leaking through heat_load() into the snapshot",
        50000.0
    );
}
