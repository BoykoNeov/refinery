//! M54.1 — a check valve on a plant whose line flashes (docs/DESIGN.md §59.1;
//! decision 5: a dead pump's line need not run backwards).
//!
//! Two inline plants with `[fidelity] line_flash = "equilibrium"`:
//! - **M54.0's pump plant with a check valve on its discharge**, into a 3 bar
//!   destination above the 2.4 bar supply. Without the disc, a pump its boiling
//!   supply has killed lets the destination's liquid run back through it; with
//!   the disc the line stops.
//! - **M53's let-down line with the control valve replaced by a check valve**: a
//!   boiling stream through the disc, which reads its drive at the mixture's
//!   density.
//!
//! These gates hold the PLANTS: the disc shutting the dead pump's line, every
//! move across the supply's temperatures settling on the cold answer on both
//! fidelities, and the two fidelities agreeing.

use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot};
use refinery_core::units::Kelvin;
use refinery_core::Engine;

const COMPONENTS: &str = r#"
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
"#;

/// M54.0's pump plant with a check valve after the pump, into `destination_bar`.
fn pump_plant(solver: &str, celsius: f64, destination_bar: f64) -> String {
    format!(
        r#"
[meta]
name = "pump_check_valve_probe"
description = "A pump that boils its own inlet, with a check valve on its discharge."

[simulation]
dt = 0.1

[fidelity]
flow = "{solver}"
thermo = "trouton"
line_flash = "equilibrium"
{COMPONENTS}
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
# One tick: the instant lock M54 built, so a lock here is the solve's alone.
gas_fill_time_s = 0.1

[nodes.discharge_check]
type = "check_valve"
kv = 80.0
full_open_bar = 0.1

[nodes.discharge_valve]
type = "valve"
kv = 80.0
opening = 0.6

[nodes.unit_feed]
type = "sink"
pressure_bar = {destination_bar}
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
to = "discharge_check"
length_m = 20.0
diameter_m = 0.10

[[pipes]]
name = "check_spool"
from = "discharge_check"
to = "discharge_valve"
length_m = 2.0
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

/// M53's let-down line with a check valve where the control valve was.
fn letdown_plant(solver: &str, celsius: f64) -> String {
    format!(
        r#"
[meta]
name = "flashing_check_valve_probe"
description = "A hot naphtha supply let down through a check valve."

[simulation]
dt = 0.1

[fidelity]
flow = "{solver}"
thermo = "trouton"
line_flash = "equilibrium"
{COMPONENTS}
[nodes.rundown]
type = "source"
pressure_bar = 2.4
temperature_c = {celsius}
composition = {{ light_naphtha = 0.7, heavy_naphtha = 0.3 }}

[nodes.rundown_check]
type = "check_valve"
kv = 40.0
full_open_bar = 0.1

[nodes.product]
type = "sink"
pressure_bar = 1.2
temperature_c = 100.0
composition = {{ light_naphtha = 0.7, heavy_naphtha = 0.3 }}

[[pipes]]
name = "rundown_line"
from = "rundown"
to = "rundown_check"
length_m = 30.0
diameter_m = 0.08

[[pipes]]
name = "check_outlet"
from = "rundown_check"
to = "product"
length_m = 20.0
diameter_m = 0.08
"#
    )
}

const SOLVERS: [&str; 2] = ["newton", "simple"];

/// Supply temperatures [°C]: liquid throughout, the pump on the table's fall, the
/// supply boiling where it stands, deep in two-phase.
const TEMPERATURES: [f64; 7] = [100.0, 105.0, 110.0, 118.0, 122.0, 125.0, 130.0];

/// Ticks a step runs before it is read: the plants hold nothing, so a step
/// settles within a few ticks (`pump_two_phase_reference.rs`).
const SETTLE_TICKS: u32 = 15;

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the scenario must parse");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant must build: {e}"))
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn edge_flow(s: &Snapshot, name: &str) -> f64 {
    s.edges
        .iter()
        .find(|e| e.name == name)
        .unwrap()
        .stream
        .mass_flow
        .value()
}

/// Run `ticks` ticks, each converging, the last two agreeing on `edge`'s flow;
/// the worst iterations a tick.
fn settle(engine: &mut Engine, ticks: u32, edge: &str, label: &str) -> u32 {
    let mut worst = 0;
    let mut last = f64::NAN;
    for t in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("{label}: tick {t} failed: {e}"));
        let s = engine.snapshot();
        assert!(s.solver.converged, "{label}: tick {t} did not converge");
        worst = worst.max(s.solver.iterations);
        let now = edge_flow(&s, edge);
        if t == ticks {
            // The solvers converge each tick to 1e-8 kg/s absolute, so a settled
            // plant wanders that much (measured on CI: 1.25e-8 kg/s at 6.3 kg/s);
            // the flip this catches moved 4 kg/s.
            assert!(
                (now - last).abs() <= 1e-7 * now.abs().max(1.0),
                "{label}: not settled — {last} then {now} kg/s on the last two ticks"
            );
        }
        last = now;
    }
    worst
}

fn warm_supply(engine: &mut Engine, supply: &str, celsius: f64) {
    let node = engine.graph.find_node(supply).unwrap();
    engine
        .apply(Command::SetSourceTemperature {
            node,
            temperature: Kelvin(celsius + 273.15),
        })
        .expect("a flashing plant takes a boiling supply");
}

/// **The disc stops a dead pump's line from running back**: at 125 °C into 3 bar
/// the pump's suction offers a stream over nine tenths vapour by volume and the
/// pump makes almost no pressure; without the disc the destination's liquid runs
/// back at about 2.1 kg/s (`pump_two_phase_reference.rs`). With it the line is
/// shut — no flow either way, on both fidelities, the same every tick.
#[test]
fn the_disc_stops_a_dead_pumps_line_running_back() {
    for solver in SOLVERS {
        let mut engine = build(&pump_plant(solver, 125.0, 3.0));
        settle(&mut engine, SETTLE_TICKS, "discharge", solver);
        let s = engine.snapshot();
        let flow = edge_flow(&s, "discharge");
        assert!(
            flow.abs() < 1e-6,
            "{solver}: {flow} kg/s through a shut disc"
        );
        let pump = node(&s, "feed_pump")
            .pump_two_phase
            .expect("the suction offers vapour");
        assert!(pump.void_fraction > 0.9, "{solver}: {pump:?}");
    }
}

/// **Every move across the supply's temperatures lands on the cold answer, and
/// stays there**, with the disc on the pump's discharge, into 1.5 and 3 bar, on
/// both fidelities: up from 100 °C to 130 °C and back, each settled flow within
/// 1e-6 of a cold start at that temperature, under iteration caps.
#[test]
fn every_move_with_the_disc_lands_on_the_cold_answer() {
    for destination_bar in [1.5, 3.0] {
        for solver in SOLVERS {
            let mut worst = 0;
            let cold: Vec<f64> = TEMPERATURES
                .iter()
                .map(|&c| {
                    let mut engine = build(&pump_plant(solver, c, destination_bar));
                    let label = format!("{solver} cold at {c} °C into {destination_bar} bar");
                    worst = worst.max(settle(&mut engine, SETTLE_TICKS, "discharge", &label));
                    if vent_if_locked(&mut engine) {
                        worst = worst.max(settle(&mut engine, SETTLE_TICKS, "discharge", &label));
                        assert_locked_again(&engine, &label);
                    }
                    edge_flow(&engine.snapshot(), "discharge")
                })
                .collect();
            let mut path: Vec<usize> = (0..TEMPERATURES.len()).collect();
            path.extend((0..TEMPERATURES.len() - 1).rev());
            let mut engine = build(&pump_plant(solver, TEMPERATURES[0], destination_bar));
            settle(&mut engine, SETTLE_TICKS, "discharge", solver);
            for &k in &path[1..] {
                let celsius = TEMPERATURES[k];
                let was_locked = is_locked(&engine);
                warm_supply(&mut engine, "rundown_source", celsius);
                let label = format!("{solver} moved to {celsius} °C into {destination_bar} bar");
                worst = worst.max(settle(&mut engine, SETTLE_TICKS, "discharge", &label));
                if vent_if_locked(&mut engine) {
                    worst = worst.max(settle(&mut engine, SETTLE_TICKS, "discharge", &label));
                    // A lock carried down from a hotter step is history, not this move's.
                    if !was_locked {
                        assert_locked_again(&engine, &label);
                    }
                }
                let moved = edge_flow(&engine.snapshot(), "discharge");
                assert!(
                    (moved - cold[k]).abs() <= 1e-6 * cold[k].abs().max(1.0),
                    "{label}: moved {moved} kg/s, cold {}",
                    cold[k]
                );
            }
            // Measured worst a tick: into 1.5 bar Newton 7, the game solver 9;
            // into 3 bar, where the disc opens and shuts as the pump dies and
            // revives, 17 and 20.
            let cap = match (solver, destination_bar > 2.0) {
                ("newton", false) => 10,
                ("newton", true) => 22,
                (_, false) => 12,
                (_, true) => 26,
            };
            assert!(
                worst <= cap,
                "{solver} into {destination_bar} bar: {worst} iterations a tick, cap {cap}"
            );
        }
    }
}

/// **A boiling stream passes the disc**: M53's let-down line with a check valve,
/// its supply stepped from liquid to boiling and back, the disc's outlet carrying
/// vapour once the supply boils, every move settled and on the cold answer, and
/// the two fidelities agreeing to 1e-6.
#[test]
fn a_boiling_stream_passes_the_disc_on_both_fidelities() {
    let mut settled: Vec<Vec<f64>> = Vec::new();
    for solver in SOLVERS {
        let mut engine = build(&letdown_plant(solver, TEMPERATURES[0]));
        settle(&mut engine, SETTLE_TICKS, "rundown_line", solver);
        let mut flows = vec![edge_flow(&engine.snapshot(), "rundown_line")];
        for &celsius in &TEMPERATURES[1..] {
            warm_supply(&mut engine, "rundown", celsius);
            let label = format!("{solver} let-down moved to {celsius} °C");
            settle(&mut engine, SETTLE_TICKS, "rundown_line", &label);
            let s = engine.snapshot();
            let moved = edge_flow(&s, "rundown_line");
            assert!(moved > 0.0, "{label}: {moved} kg/s");
            let mut cold = build(&letdown_plant(solver, celsius));
            settle(&mut cold, SETTLE_TICKS, "rundown_line", &label);
            let cold_flow = edge_flow(&cold.snapshot(), "rundown_line");
            assert!(
                (moved - cold_flow).abs() <= 1e-6 * cold_flow.abs().max(1.0),
                "{label}: moved {moved}, cold {cold_flow}"
            );
            flows.push(moved);
        }
        let s = engine.snapshot();
        assert!(
            s.edges
                .iter()
                .find(|e| e.name == "check_outlet")
                .unwrap()
                .stream
                .vapour_fraction
                .is_some(),
            "{solver}: the disc's outlet carries no vapour at 130 °C"
        );
        settled.push(flows);
    }
    for (newton, simple) in settled[0].iter().zip(&settled[1]) {
        assert!(
            (newton - simple).abs() <= 1e-6 * newton.abs().max(1.0),
            "the fidelities disagree: {newton} against {simple} kg/s"
        );
    }
}

/// Stop, vent and start the pump if it is gas-locked (M54.2, §59.2); whether it
/// was. Applied, once settled, to a moved run after each move and to a cold start,
/// so both are read as "the pump running unlocked, re-locking only where its
/// settled state locks it" — and `assert_locked_again` then requires that it
/// does. Until M55 a cold start, or a move, at 118 °C locked on its FIRST tick
/// and ran on once vented: the step tick read the valves' last-tick states
/// (B50), and its suction offered 45% vapour for that one tick (B53, struck).
fn vent_if_locked(engine: &mut Engine) -> bool {
    let pump = engine.graph.find_node("feed_pump").unwrap();
    let NodeKind::Pump { gas_locked, .. } = engine.graph.node(pump).kind else {
        unreachable!()
    };
    if !gas_locked {
        return false;
    }
    for command in [
        Command::SetPumpOn {
            node: pump,
            on: false,
        },
        Command::VentPump { node: pump },
        Command::SetPumpOn {
            node: pump,
            on: true,
        },
    ] {
        engine
            .apply(command)
            .expect("a locked pump is stopped, vented and started");
    }
    true
}

/// A pump `vent_if_locked` vented must lock again once re-settled: the plant it
/// locked on is one whose settled suction locks it, not one a single tick misled
/// (M55.0, docs/DESIGN.md §60.0). This fixture fills its pocket in one tick, so a
/// lock here is the solve's alone.
fn assert_locked_again(engine: &Engine, label: &str) {
    assert!(
        is_locked(engine),
        "{label}: the pump locked, was vented, and runs on unlocked: a lock its settled state does not make"
    );
}

fn is_locked(engine: &Engine) -> bool {
    let pump = engine.graph.find_node("feed_pump").unwrap();
    let NodeKind::Pump { gas_locked, .. } = engine.graph.node(pump).kind else {
        unreachable!()
    };
    gas_locked
}
