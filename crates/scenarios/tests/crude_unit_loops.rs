//! M72.4 — the shipped crude unit whose pumparounds are loops on the map
//! (`scenarios/crude_unit_loops.toml`, docs/DESIGN.md §77.7, §77.12): thirteen
//! trays, three loops each a pump and a cooler, started cold on their start
//! duties and handed to their return temperatures, the boiling-order seed held
//! on the retry (§77.5).

use refinery_core::engine::Engine;
use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot};
use refinery_core::units::KgPerSec;
use refinery_scenarios::{build_engine, load_str};

const PLANT: &str = include_str!("../../../scenarios/crude_unit_loops.toml");
/// Each loop's flow [kg/s] and return [°C], as the file declares them, and
/// the duty it settles at [W] (M72.0's probe, §77.0 finding 7).
const LOOPS: [(f64, f64, f64); 3] = [
    (85.0, 150.0, 15.06e6),
    (95.0, 215.0, 17.05e6),
    (65.0, 270.0, 12.79e6),
];

fn engine(solver: &str) -> Engine {
    let src = PLANT.replace("flow = \"newton\"", &format!("flow = \"{solver}\""));
    build_engine(&load_str(&src).expect("parses")).expect("builds")
}

fn node<'a>(snapshot: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    snapshot
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("no node '{name}'"))
}

fn run(engine: &mut Engine, ticks: u32) -> Result<Snapshot, String> {
    for tick in 0..ticks {
        engine.tick().map_err(|e| format!("tick {tick}: {e}"))?;
    }
    Ok(engine.snapshot())
}

/// A draw's MOLE fraction of its own cut `cut`, by the slate's order.
fn purity(engine: &Engine, snapshot: &Snapshot, draw: &str, cut: usize) -> f64 {
    let moles: Vec<f64> = snapshot
        .edges
        .iter()
        .find(|e| e.name == draw)
        .unwrap_or_else(|| panic!("no edge '{draw}'"))
        .stream
        .composition
        .fractions()
        .iter()
        .enumerate()
        .map(|(c, w)| w / engine.slate.get(c).molar_mass.value())
        .collect();
    moles[cut] / moles.iter().sum::<f64>()
}

/// Every loop at its flow, returned at its setpoint, at the duty the probe
/// measured.
fn loops_as_declared(snapshot: &Snapshot, solver: &str) {
    let operation = node(snapshot, "column").column_operation.clone().unwrap();
    for (run, (flow, back_c, duty)) in operation.loops.iter().zip(LOOPS) {
        assert!(
            (run.flow_kg_s / flow - 1.0).abs() < 1e-9,
            "{solver}: {run:?}"
        );
        assert!(
            (run.return_temperature_k - (back_c + 273.15)).abs() < 1e-9,
            "{solver}: {run:?}"
        );
        assert!(!run.at_capacity, "{solver}: {run:?}");
        assert!(
            (run.duty_w / duty - 1.0).abs() < 1e-3,
            "{solver}: {} W against the probe's {duty} W",
            run.duty_w
        );
    }
}

/// **It starts cold on tick 1** (§77.5), on either flow solver: its loops
/// brought in on their start duties and handed over, every one at its flow and
/// its return temperature on the first tick — the handed-over column, not the
/// one it started on.
fn starts_cold_on_tick_1(solver: &str) {
    let snapshot = run(&mut engine(solver), 1).expect("the plant starts cold");
    loops_as_declared(&snapshot, solver);
}

#[test]
fn it_starts_cold_on_tick_1_newton() {
    starts_cold_on_tick_1("newton");
}

#[test]
fn it_starts_cold_on_tick_1_simple() {
    starts_cold_on_tick_1("simple");
}

/// **The plant runs as its file says**: the loops where they were declared;
/// each product at least five points purer than the ten-tray crude unit's
/// side-cooled column (72.4, 67.9, 66.6 %, §77.0 finding 2) — what the three
/// extra trays are for; the trays inside each loop carrying its circulation on
/// top of their own liquid; the reflux the declared one; the coil far under
/// its tube limit.
#[test]
fn the_plant_runs_as_its_file_says() {
    let mut engine = engine("newton");
    let snapshot = run(&mut engine, 300).expect("the plant runs");
    loops_as_declared(&snapshot, "newton");
    let column = node(&snapshot, "column");
    let operation = column.column_operation.as_ref().unwrap();
    assert_eq!(operation.reflux_ratio, 4.0);
    for (draw, cut, ten_trays) in [
        ("kerosene_draw", 2, 0.724),
        ("diesel_draw", 3, 0.679),
        ("gas_oil_draw", 4, 0.666),
    ] {
        let purity = purity(&engine, &snapshot, draw, cut);
        assert!(
            purity > ten_trays + 0.05,
            "{draw}: {purity} against the ten-tray column's {ten_trays}"
        );
    }
    // Stage 3 carries the kerosene loop's return down to its draw on 4, and
    // so on: each 1.4 times the liquid of the tray above its return at least.
    let liquid = &operation.stage_liquid_mol_s;
    for ret in [3, 6, 9] {
        assert!(
            liquid[ret - 1] > 1.4 * liquid[ret - 2],
            "stage {ret}: {liquid:?}"
        );
    }
    let NodeKind::Furnace { coil, .. } = &engine
        .graph
        .node(engine.graph.find_node("furnace").unwrap())
        .kind
    else {
        panic!("the furnace is a furnace");
    };
    assert!(
        coil.temperature.value() < 550.0 + 273.15 - 100.0,
        "{coil:?}"
    );
}

#[test]
fn the_books_close_every_tick_newton() {
    super::pumparound_loop_reference::the_books_close_every_tick(PLANT, "newton", 300);
}

#[test]
fn the_books_close_every_tick_simple() {
    super::pumparound_loop_reference::the_books_close_every_tick(PLANT, "simple", 300);
}

/// **A short tray cuts its product back; the loop keeps its setting** (M72.3,
/// docs/DESIGN.md §77.13, the user's decision): the reflux walked down to
/// 0.05, the gas oil's tray runs short and its product alone is cut back —
/// every loop at its set flow, none short. A loop takes back to the tray
/// above what it draws, and its cooling MAKES liquid, so slowing one would
/// only dry its tray further (measured: the gas-oil loop up from 65 to 85 kg/s
/// released the gas oil). Turned down to 50 kg/s the gas oil is all but gone
/// (7 % of its share, measured) and the column runs; at 40 there is no product
/// left to give way, and the column refuses as any column past its edge —
/// the general refusal ("could not settle"), which does not name the tray.
#[test]
fn a_short_tray_cuts_its_product_and_the_loops_hold() {
    let mut engine = engine("newton");
    run(&mut engine, 300).expect("the plant runs");
    let column = engine.graph.find_node("column").unwrap();
    let mut snapshot = None;
    for reflux in [2.0, 1.0, 0.5, 0.2, 0.1, 0.05] {
        engine
            .apply(Command::SetRefluxRatio {
                node: column,
                reflux_ratio: reflux,
            })
            .unwrap();
        snapshot = Some(run(&mut engine, 50).unwrap_or_else(|e| panic!("reflux {reflux}: {e}")));
    }
    let snapshot = snapshot.unwrap();
    let cut = &node(&snapshot, "column").column_draws_cut_back;
    assert_eq!(cut.len(), 1, "{cut:?}");
    assert_eq!(cut[0].draw, 3, "the gas oil: {cut:?}");
    assert!(
        cut[0].of_declared > 0.0 && cut[0].of_declared < 1.0,
        "{cut:?}"
    );
    let operation = node(&snapshot, "column").column_operation.clone().unwrap();
    for (run, (flow, _, _)) in operation.loops.iter().zip(LOOPS) {
        assert!((run.flow_kg_s / flow - 1.0).abs() < 1e-9, "{run:?}");
        assert_eq!(run.short_kg_s, 0.0, "{run:?}");
    }
    let gas_oil_loop = |engine: &mut Engine, flow: f64| {
        engine
            .apply(Command::SetLoopFlow {
                node: column,
                pumparound_loop: 2,
                flow: KgPerSec(flow),
            })
            .unwrap();
    };
    gas_oil_loop(&mut engine, 50.0);
    let low = run(&mut engine, 20).expect("the gas-oil loop at 50 kg/s");
    let cut = &node(&low, "column").column_draws_cut_back;
    assert!(cut.len() == 1 && cut[0].of_declared < 0.1, "{cut:?}");
    gas_oil_loop(&mut engine, 40.0);
    let refused = run(&mut engine, 20).expect_err("no product left to give way at 40 kg/s");
    assert!(
        refused.contains("could not settle at reflux ratio 0.05"),
        "{refused}"
    );
}
