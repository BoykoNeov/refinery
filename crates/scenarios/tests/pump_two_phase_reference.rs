//! M54 — a pump in two-phase service (docs/DESIGN.md §59; ledger row B48).
//!
//! An inline plant — M50's pump plant (`pump_cavitation_flow_limit.toml`) with
//! `[fidelity] line_flash = "equilibrium"` and no `npsh_required_m` — hot
//! naphtha drawn through 60 m of 80 mm line by a pump level with its supply,
//! then a valve to a destination at 1.5 bar. Warmed, the supply's liquid boils
//! at the pump's inlet before it boils where it stands, and the pump loses its
//! head to RELAP5's two-phase multiplier on the vapour there.
//!
//! These gates hold the PLANT: every cold start and every move across the
//! supply's temperatures converging on both fidelities and landing on the cold
//! answer, the pump settling on the table's fall, and the suction key refused
//! on a flashing plant.

use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot};
use refinery_core::units::Kelvin;
use refinery_core::Engine;

/// M50's plant with the line flash, at a supply temperature [°C] and M50's
/// 1.5 bar destination.
fn plant(solver: &str, celsius: f64) -> String {
    plant_into(solver, celsius, 1.5)
}

/// The same plant into a destination at `destination_bar` [bar]: at 3 bar, above
/// the 2.4 bar supply, the line runs back through a pump that has lost its push.
fn plant_into(solver: &str, celsius: f64, destination_bar: f64) -> String {
    format!(
        r#"
[meta]
name = "pump_two_phase_probe"
description = "Hot naphtha drawn through a long suction line by a pump that boils its own inlet."

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

/// Supply temperatures [°C] the gates walk: liquid at the pump, the pump on the
/// table's fall, the supply itself boiling, and deep into two-phase.
const TEMPERATURES: [f64; 8] = [95.0, 100.0, 105.0, 110.0, 118.0, 122.0, 125.0, 130.0];

const SOLVERS: [&str; 2] = ["newton", "simple"];

/// Ticks a step runs before it is read. The plant holds nothing — no tank, no
/// vessel — so a step settles within a few ticks (the zero-volume nodes' one-tick
/// enthalpy lag, B50); the flip this file was written against alternated from
/// the first tick, so 15 shows it as surely as 400.
const SETTLE_TICKS: u32 = 15;

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the scenario must parse");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant must build: {e}"))
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn flow(s: &Snapshot) -> f64 {
    edge_flow(s, "suction_line")
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

/// Run `ticks` ticks, each converging, or say which one did not; the last two
/// ticks' flows must agree — a plant that alternates between two answers is not
/// settled, whichever one a sampled tick lands on. The worst iterations a tick.
fn settle(engine: &mut Engine, ticks: u32, label: &str) -> u32 {
    let mut worst = 0;
    let mut last = f64::NAN;
    for t in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("{label}: tick {t} failed: {e}"));
        let s = engine.snapshot();
        assert!(s.solver.converged, "{label}: tick {t} did not converge");
        worst = worst.max(s.solver.iterations);
        if t == ticks {
            let now = flow(&s);
            // The solvers converge each tick to 1e-8 kg/s absolute, so a settled
            // plant wanders that much (measured on CI: 1.25e-8 kg/s at 6.3 kg/s);
            // the flip this catches moved 4 kg/s.
            assert!(
                (now - last).abs() <= 1e-7 * now.abs().max(1.0),
                "{label}: not settled — {last} kg/s then {now} kg/s on the last two ticks"
            );
        }
        last = flow(&s);
    }
    worst
}

fn warm_supply(engine: &mut Engine, celsius: f64) {
    let supply = engine.graph.find_node("rundown_source").unwrap();
    engine
        .apply(Command::SetSourceTemperature {
            node: supply,
            temperature: Kelvin(celsius + 273.15),
        })
        .expect("a flashing plant takes a boiling supply");
}

/// The settled flow [kg/s] of a cold start at `celsius` into `destination_bar`,
/// and its worst iterations a tick.
fn cold_flow(solver: &str, celsius: f64, destination_bar: f64) -> (f64, u32) {
    let mut engine = build(&plant_into(solver, celsius, destination_bar));
    let label = format!("{solver} cold at {celsius} °C into {destination_bar} bar");
    let mut worst = settle(&mut engine, SETTLE_TICKS, &label);
    if vent_if_locked(&mut engine) {
        worst = worst.max(settle(&mut engine, SETTLE_TICKS, &label));
    }
    (flow(&engine.snapshot()), worst)
}

/// **A cold start at 100 °C settles on the table's fall**, on both fidelities:
/// the supply is liquid (its bubble pressure 1.40 bar against 2.4), the pump's
/// inlet is a few percent vapour by volume, inside `[0.07, 0.165]` where the
/// multiplier falls from 0 to 1, and the pump still makes pressure.
#[test]
fn a_cold_start_settles_on_the_tables_fall() {
    for solver in SOLVERS {
        let mut engine = build(&plant(solver, 100.0));
        settle(
            &mut engine,
            SETTLE_TICKS,
            &format!("{solver} cold at 100 °C"),
        );
        let s = engine.snapshot();
        assert_eq!(node(&s, "rundown_source").vapour_fraction, None);
        let pump = node(&s, "feed_pump")
            .pump_two_phase
            .expect("the pump's inlet carries vapour");
        assert!(
            (0.07..0.165).contains(&pump.void_fraction),
            "{solver}: inlet {} vapour by volume",
            pump.void_fraction
        );
        assert!(
            pump.head_multiplier > 0.0 && pump.head_multiplier < 1.0,
            "{solver}: multiplier {}",
            pump.head_multiplier
        );
        assert!(pump.pressure_rise_pa > 0.0, "{solver}: {pump:?}");
    }
}

/// **Every move across the supply's temperatures lands on the cold answer, and
/// stays there**, on both fidelities and into both destinations: up from 95 °C to
/// 130 °C (the supply boiling where it stands from about 121 °C) and back down,
/// `SETTLE_TICKS` a step, each tick converging, the last two ticks agreeing, and each
/// settled flow within 1e-6 of a cold start at that temperature, and no tick
/// taking more than 10 Newton iterations or 12 sweeps. Into 3 bar the line runs
/// back once the pump has lost its push.
#[test]
fn every_move_across_the_supply_temperatures_lands_on_the_cold_answer() {
    for destination_bar in [1.5, 3.0] {
        for solver in SOLVERS {
            let cold: Vec<(f64, u32)> = TEMPERATURES
                .iter()
                .map(|&c| cold_flow(solver, c, destination_bar))
                .collect();
            let mut path: Vec<usize> = (0..TEMPERATURES.len()).collect();
            path.extend((0..TEMPERATURES.len() - 1).rev());
            let mut worst = cold.iter().map(|&(_, w)| w).max().unwrap_or(0);
            let mut engine = build(&plant_into(solver, TEMPERATURES[path[0]], destination_bar));
            settle(
                &mut engine,
                SETTLE_TICKS,
                &format!("{solver} cold into {destination_bar} bar"),
            );
            for &k in &path[1..] {
                let celsius = TEMPERATURES[k];
                warm_supply(&mut engine, celsius);
                let label = format!("{solver} moved to {celsius} °C into {destination_bar} bar");
                worst = worst.max(settle(&mut engine, SETTLE_TICKS, &label));
                if vent_if_locked(&mut engine) {
                    worst = worst.max(settle(&mut engine, SETTLE_TICKS, &label));
                }
                let moved = flow(&engine.snapshot());
                let (cold, _) = cold[k];
                assert!(
                    (moved - cold).abs() <= 1e-6 * cold.abs().max(1.0),
                    "{label}: moved {moved} kg/s, cold {cold} kg/s"
                );
            }
            // Measured worst a tick: Newton 6, the game solver 7, into either
            // destination — with the pump's inlet solved inside the iterate.
            // Stepped instead, Newton took 14 and failed into 3 bar, and the game
            // solver took up to 961 sweeps and failed.
            let cap = if solver == "newton" { 10 } else { 12 };
            assert!(
                worst <= cap,
                "{solver} into {destination_bar} bar: {worst} iterations a tick, cap {cap}"
            );
        }
    }
}

/// **A pump its supply has killed stays dead while the line runs back**: at
/// 125 °C into 3 bar the suction offers a stream over nine tenths vapour by
/// volume, the pump makes almost no pressure, and the destination's liquid runs
/// back through it to the supply — on both fidelities, the same answer every
/// tick. Before the pump read what its suction offers, it read last tick's own
/// contents, and this plant flipped every tick between +2.10 and −2.13 kg/s.
#[test]
fn a_dead_pumps_line_runs_back_and_stays_back() {
    for solver in SOLVERS {
        let mut engine = build(&plant_into(solver, 125.0, 3.0));
        settle(
            &mut engine,
            SETTLE_TICKS,
            &format!("{solver} at 125 °C into 3 bar"),
        );
        let s = engine.snapshot();
        let back = edge_flow(&s, "discharge");
        assert!(back < 0.0, "{solver}: the line runs forward, {back} kg/s");
        let pump = node(&s, "feed_pump")
            .pump_two_phase
            .expect("the suction offers vapour");
        assert!(pump.void_fraction > 0.9, "{solver}: {pump:?}");
        // Not zero: at 97% vapour the table's tail gives back about 40% of the
        // curve, at a mixture density of about 25 kg/m³ — some 7 kPa, against
        // the 2.7 bar the liquid makes.
        assert!(
            pump.pressure_rise_pa < 0.1e5,
            "{solver}: a dead pump made {} Pa",
            pump.pressure_rise_pa
        );
    }
}

/// **The suction key is refused on a flashing plant**, by name (§59 decision 3):
/// there the steam-water table decides a pump's head, and M50's curve could not
/// change it without the head jumping at the bubble pressure.
#[test]
fn the_suction_key_is_refused_on_a_flashing_plant() {
    let src = plant("newton", 100.0).replace("on = true\n", "on = true\nnpsh_required_m = 3.0\n");
    let file = refinery_scenarios::load_str(&src).expect("the scenario must parse");
    let err = match refinery_scenarios::build_engine(&file) {
        Ok(_) => panic!("a flashing plant's pump must not declare npsh_required_m"),
        Err(e) => e.to_string(),
    };
    assert!(
        err.contains("feed_pump") && err.contains("npsh_required_m"),
        "{err}"
    );
}

/// **Two pumps joined by one pipe are refused on a flashing plant**, by the
/// pipe's name: each pump's inlet is solved with its neighbours held, and two
/// pumps back to back are one problem, which no solve poses.
#[test]
fn two_pumps_back_to_back_are_refused_on_a_flashing_plant() {
    let src = plant("newton", 100.0)
        .replace(
            "[nodes.discharge_valve]",
            "[nodes.booster]\ntype = \"pump\"\nh0_m = 20.0\na = 400.0\n\n[nodes.discharge_valve]",
        )
        .replace(
            "name = \"discharge\"\nfrom = \"feed_pump\"",
            "name = \"interstage\"\nfrom = \"feed_pump\"\nto = \"booster\"\nlength_m = 2.0\ndiameter_m = 0.10\n\n[[pipes]]\nname = \"discharge\"\nfrom = \"booster\"",
        );
    let file = refinery_scenarios::load_str(&src).expect("the scenario must parse");
    let err = match refinery_scenarios::build_engine(&file) {
        Ok(_) => panic!("two pumps back to back must be refused on a flashing plant"),
        Err(e) => e.to_string(),
    };
    assert!(
        err.contains("interstage") && err.contains("two pumps"),
        "{err}"
    );
}

/// Stop, vent and start the pump if it is gas-locked (M54.2, §59.2); whether it
/// was. Applied, once settled, to a moved run after each move and to a cold start,
/// so both are read as "the pump running unlocked, re-locking only where its
/// settled state locks it": a cold start can lock on its FIRST tick, where a
/// moved run never passes (at 118 °C with the disc the plant has two answers —
/// the pump alive at 8% vapour, 6.43 kg/s, and dead at 45%, 7.25 kg/s — and the
/// cold seed's first tick lands on the dead one).
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
