//! M54.2 — a gas lock a person must vent (docs/DESIGN.md §59.2; decisions 6–8).
//!
//! M54.0's inline pump plant (M50's plant with `[fidelity] line_flash =
//! "equilibrium"`, no suction key, into 1.5 bar). A running pump whose suction
//! offers 16.5% vapour by volume or more — RELAP5's fully degraded point — locks:
//! it makes no head from the next tick on, whatever its suction offers after,
//! until a person stops it, vents it (`Command::VentPump`) and starts it again.
//! A stop alone, by hand or by a trip, does not clear it; nor does cooling the
//! supply.

use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot};
use refinery_core::units::Kelvin;
use refinery_core::Engine;

/// M54.0's pump plant, at a supply temperature [°C].
fn plant(solver: &str, celsius: f64) -> String {
    format!(
        r#"
[meta]
name = "gas_lock_probe"
description = "A pump that boils its own inlet, and locks."

[simulation]
dt = 0.1

[fidelity]
flow = "{solver}"
thermo = "trouton"
line_flash = "equilibrium"

[[components]]
name = "light_naphtha"
tb_c = 80.0
molar_mass_kg_per_mol = 0.100
density_kg_per_m3 = 680.0
cp_j_per_kg_k = 2200.0

[[components]]
name = "heavy_naphtha"
tb_c = 150.0
molar_mass_kg_per_mol = 0.130
density_kg_per_m3 = 750.0
cp_j_per_kg_k = 2100.0

[nodes.rundown_source]
type = "source"
pressure_bar = 2.4
temperature_c = {celsius}
composition = {{ light_naphtha = 0.7, heavy_naphtha = 0.3 }}

[nodes.feed_pump]
type = "pump"
h0_m = 40.0
a = 800.0
on = true

[nodes.discharge_valve]
type = "valve"
kv = 80.0
opening = 0.6

[nodes.unit_feed]
type = "sink"
pressure_bar = 1.5
temperature_c = 110.0
composition = {{ light_naphtha = 0.7, heavy_naphtha = 0.3 }}

[[pipes]]
name = "suction_line"
from = "rundown_source"
to = "feed_pump"
length_m = 60.0
diameter_m = 0.08

[[pipes]]
name = "discharge"
from = "feed_pump"
to = "discharge_valve"
length_m = 20.0
diameter_m = 0.10

[[pipes]]
name = "feed_line"
from = "discharge_valve"
to = "unit_feed"
length_m = 25.0
diameter_m = 0.10
"#
    )
}

const SOLVERS: [&str; 2] = ["newton", "simple"];

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the scenario must parse");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant must build: {e}"))
}

fn run(engine: &mut Engine, ticks: u32, label: &str) {
    for t in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("{label}: tick {t} failed: {e}"));
        assert!(engine.snapshot().solver.converged, "{label}: tick {t}");
    }
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn flow(s: &Snapshot) -> f64 {
    s.edges
        .iter()
        .find(|e| e.name == "discharge")
        .unwrap()
        .stream
        .mass_flow
        .value()
}

fn locked(engine: &Engine) -> bool {
    let pump = engine.graph.find_node("feed_pump").unwrap();
    match engine.graph.node(pump).kind {
        NodeKind::Pump { gas_locked, .. } => gas_locked,
        _ => unreachable!(),
    }
}

fn command(engine: &mut Engine, command: Command) -> Result<(), String> {
    engine.apply(command).map_err(|e| e.to_string())
}

fn warm(engine: &mut Engine, celsius: f64) {
    let node = engine.graph.find_node("rundown_source").unwrap();
    command(
        engine,
        Command::SetSourceTemperature {
            node,
            temperature: Kelvin(celsius + 273.15),
        },
    )
    .unwrap();
}

fn pump_on(engine: &mut Engine, on: bool) {
    let node = engine.graph.find_node("feed_pump").unwrap();
    command(engine, Command::SetPumpOn { node, on }).unwrap();
}

fn vent(engine: &mut Engine) -> Result<(), String> {
    let node = engine.graph.find_node("feed_pump").unwrap();
    command(engine, Command::VentPump { node })
}

/// The settled flow [kg/s] of a cold start at `celsius`, never locked.
fn cold_flow(solver: &str, celsius: f64) -> f64 {
    let mut engine = build(&plant(solver, celsius));
    run(&mut engine, 40, &format!("{solver} cold at {celsius} °C"));
    assert!(!locked(&engine), "{solver}: locked cold at {celsius} °C");
    flow(&engine.snapshot())
}

/// **On the table's fall the pump does not lock**: at 100 °C its suction offers
/// about 7% vapour by volume and it has lost a tenth of its head, but 16.5% is
/// the lock, and 200 ticks never reach it.
#[test]
fn a_pump_on_the_fall_does_not_lock() {
    for solver in SOLVERS {
        let mut engine = build(&plant(solver, 100.0));
        run(&mut engine, 200, solver);
        let s = engine.snapshot();
        let pump = node(&s, "feed_pump")
            .pump_two_phase
            .expect("vapour offered");
        assert!(pump.void_fraction < 0.165, "{solver}: {pump:?}");
        assert!(!locked(&engine), "{solver}: locked on the fall");
    }
}

/// **A pump whose suction offers 16.5% vapour locks, and stays dead when the
/// supply cools**: warmed to 125 °C the suction offers over nine tenths vapour by
/// volume and the pump locks; cooled back to 100 °C, where a cold pump runs at
/// about 15 kg/s, the locked one makes no head — the flow is what the supply's
/// 2.4 bar alone pushes to the 1.5 bar destination, through a dead pump.
#[test]
fn a_pump_locks_and_stays_dead_when_the_supply_cools() {
    for solver in SOLVERS {
        let mut engine = build(&plant(solver, 100.0));
        run(&mut engine, 40, solver);
        assert!(!locked(&engine));
        warm(&mut engine, 125.0);
        run(&mut engine, 40, &format!("{solver} at 125 °C"));
        assert!(locked(&engine), "{solver}: not locked at 125 °C");
        warm(&mut engine, 100.0);
        run(&mut engine, 40, &format!("{solver} cooled to 100 °C"));
        assert!(locked(&engine), "{solver}: unlocked by cooling");
        let s = engine.snapshot();
        let rise = node(&s, "feed_pump")
            .pump_two_phase
            .map_or(0.0, |p| p.pressure_rise_pa);
        assert_eq!(rise, 0.0, "{solver}: a locked pump made {rise} Pa");
        let cold = cold_flow(solver, 100.0);
        let dead = flow(&s);
        assert!(
            dead < 0.9 * cold,
            "{solver}: locked {dead} kg/s against a running {cold} kg/s"
        );
    }
}

/// **Only a vent on a stopped pump clears the lock**, and the pump then runs as
/// a cold one does: a stop and a start without the vent leave it locked; venting
/// a running pump, a pump that is not locked, or something that is not a pump is
/// refused, by name; stopped, vented and started, it lands on the cold answer.
#[test]
fn only_a_vent_on_a_stopped_pump_clears_the_lock() {
    for solver in SOLVERS {
        let mut engine = build(&plant(solver, 125.0));
        let refused = vent(&mut engine).expect_err("an unlocked pump has nothing to vent");
        assert!(
            refused.contains("feed_pump") && refused.contains("not gas-locked"),
            "{refused}"
        );
        run(&mut engine, 40, solver);
        assert!(locked(&engine), "{solver}: not locked at 125 °C");
        warm(&mut engine, 100.0);

        let refused = vent(&mut engine).expect_err("a running pump is not vented");
        assert!(
            refused.contains("feed_pump") && refused.contains("running"),
            "{refused}"
        );
        let valve = engine.graph.find_node("discharge_valve").unwrap();
        let refused = command(&mut engine, Command::VentPump { node: valve })
            .expect_err("a valve is not a pump");
        assert!(
            refused.contains("discharge_valve") && refused.contains("not a pump"),
            "{refused}"
        );

        pump_on(&mut engine, false);
        run(&mut engine, 5, &format!("{solver} stopped"));
        pump_on(&mut engine, true);
        run(&mut engine, 40, &format!("{solver} restarted unvented"));
        assert!(
            locked(&engine),
            "{solver}: a stop and start cleared the lock"
        );

        pump_on(&mut engine, false);
        run(&mut engine, 5, &format!("{solver} stopped again"));
        vent(&mut engine).expect("a stopped, locked pump is vented");
        assert!(!locked(&engine), "{solver}: the vent left it locked");
        pump_on(&mut engine, true);
        run(&mut engine, 40, &format!("{solver} vented and restarted"));
        assert!(!locked(&engine), "{solver}: relocked on liquid");
        let cold = cold_flow(solver, 100.0);
        let back = flow(&engine.snapshot());
        assert!(
            (back - cold).abs() <= 1e-6 * cold.abs(),
            "{solver}: vented {back} kg/s against cold {cold} kg/s"
        );
    }
}

/// **The lock is part of the engine's state, and only on a flashing plant**: the
/// wire of an unlocked pump carries no key for it, and the same plant with the
/// line flash off never locks however hot its supply (M52 refuses a boiling one,
/// so 118 °C — liquid at the supply, below the pump's own boiling).
#[test]
fn an_unlocked_pump_writes_no_key() {
    let mut engine = build(&plant("newton", 100.0));
    run(&mut engine, 10, "newton");
    let wire = serde_json::to_string(&engine.snapshot()).unwrap();
    assert!(!wire.contains("gas_locked"), "{wire}");

    let mut engine = build(&plant("newton", 118.0).replace("line_flash = \"equilibrium\"", ""));
    run(&mut engine, 200, "no line flash");
    assert!(!locked(&engine), "a plant without the line flash locked");
}
