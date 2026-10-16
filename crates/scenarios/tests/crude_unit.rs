//! The crude unit (M60.3, docs/DESIGN.md §65.9): a preheated crude, a furnace,
//! and a heat-balance column with three side draws and three pumparounds — the
//! plant the column screen runs, and the range its keys stay inside.

use refinery_core::engine::Engine;
use refinery_core::graph::{NodeId, NodeKind};
use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot};
use refinery_core::units::Watt;
use refinery_scenarios::{build_engine, load_str};

const CRUDE_UNIT: &str = include_str!("../../../scenarios/crude_unit.toml");
/// The pumparounds' duties as shipped [MW], in the order the file lists them.
const PUMPAROUND_MW: [f64; 3] = [15.0, 15.0, 10.0];

fn engine() -> Engine {
    build_engine(&load_str(CRUDE_UNIT).expect("parses")).expect("builds")
}

fn node<'a>(snapshot: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    snapshot
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("no node '{name}'"))
}

fn id(engine: &Engine, name: &str) -> NodeId {
    node(&engine.snapshot(), name).id
}

fn run(engine: &mut Engine, ticks: u32) -> Result<Snapshot, String> {
    for tick in 0..ticks {
        engine.tick().map_err(|e| format!("tick {tick}: {e}"))?;
    }
    Ok(engine.snapshot())
}

/// A draw's mass fraction of one cut, by the slate's order.
fn fraction(snapshot: &Snapshot, draw: &str, cut: usize) -> f64 {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == draw)
        .unwrap_or_else(|| panic!("no edge '{draw}'"))
        .stream
        .composition
        .fractions()[cut]
}

/// **The crude unit runs as its file says**: about two-thirds of the feed
/// flashes, the column runs at its declared reflux (the floor does not bind),
/// each side draw is mostly its own cut, the residue nearly pure, and the
/// furnace coil far under its tube limit. Cold-started on tick 1 — which the
/// sharp seed, the first iteration on constant molar overflow and the
/// relaxation are each needed for (§65.9).
#[test]
fn the_crude_unit_runs() {
    let snapshot = run(&mut engine(), 3000).expect("the crude unit runs");
    let column = node(&snapshot, "column");
    let operation = column
        .column_operation
        .as_ref()
        .expect("a heat-balance column reports");
    assert!(
        (0.55..0.75).contains(&operation.feed_vapour_fraction),
        "{}",
        operation.feed_vapour_fraction
    );
    assert_eq!(operation.reflux_ratio, 4.0);
    // Slate order: light naphtha, heavy naphtha, kerosene, diesel, gas oil, residue.
    for (draw, cut, at_least) in [
        ("kerosene_draw", 2, 0.6),
        ("diesel_draw", 3, 0.6),
        ("gas_oil_draw", 4, 0.6),
        ("residue_draw", 5, 0.95),
    ] {
        let share = fraction(&snapshot, draw, cut);
        assert!(share > at_least, "{draw}: {share}");
    }
    let coil = match &node(&snapshot, "furnace").kind {
        NodeKind::Furnace { coil, .. } => coil.temperature.value(),
        other => panic!("not a furnace with a coil: {other:?}"),
    };
    assert!(coil < 550.0 + 273.15 - 100.0, "the coil at {coil} K");
}

/// The keys the column screen gives, each to the edge of the range it stops
/// at, from the running plant: the furnace 0 and 40 MW, the reflux 2 and 10,
/// the pumparounds 75 % and 300 % of their duties. Every one runs.
#[test]
fn the_screen_range_runs_to_its_edges() {
    let set_pumparounds = |engine: &mut Engine, column: NodeId, scale: f64| {
        for (i, mw) in PUMPAROUND_MW.iter().enumerate() {
            engine
                .apply(Command::SetPumparoundDuty {
                    node: column,
                    pumparound: i as u32,
                    duty: Watt(mw * scale * 1e6),
                })
                .unwrap();
        }
    };
    for (key, value) in [
        ("furnace", 0.0),
        ("furnace", 40.0),
        ("reflux", 2.0),
        ("reflux", 10.0),
        ("pumparounds", 0.75),
        ("pumparounds", 3.0),
    ] {
        let mut engine = engine();
        let (column, furnace) = (id(&engine, "column"), id(&engine, "furnace"));
        run(&mut engine, 1000).expect("the plant settles");
        match key {
            "furnace" => engine
                .apply(Command::SetFurnaceDuty {
                    node: furnace,
                    duty: Watt(value * 1e6),
                })
                .unwrap(),
            "reflux" => engine
                .apply(Command::SetRefluxRatio {
                    node: column,
                    reflux_ratio: value,
                })
                .unwrap(),
            _ => set_pumparounds(&mut engine, column, value),
        }
        run(&mut engine, 1000).unwrap_or_else(|e| panic!("{key} at {value}: {e}"));
    }
}

/// Past the range the run halts, and the reason names the settings it could
/// not settle at (the user's decision, §65.9) — and, since M61, the physical
/// edge it stopped at (§66.2): at a reflux of 0.6 the kerosene tray (stage 3)
/// runs dry, its draw taking all the liquid the reflux sends it.
#[test]
fn past_the_range_the_halt_names_the_settings() {
    let mut engine = engine();
    let column = id(&engine, "column");
    run(&mut engine, 1000).expect("the plant settles");
    engine
        .apply(Command::SetRefluxRatio {
            node: column,
            reflux_ratio: 0.6,
        })
        .unwrap();
    let halt = run(&mut engine, 1000).expect_err("reflux 0.6 is past the range");
    assert!(
        halt.contains("could not settle at reflux ratio 0.6 with its pumparounds at 15.0 MW"),
        "{halt}"
    );
    assert!(halt.contains("The trays at stage 3 run dry"), "{halt}");
}

/// `SetPumparoundDuty` writes one pumparound's duty, which the column's `kind`
/// then lists; it is refused for a pumparound the column does not have, a
/// negative duty, and a node that is not a column with pumparounds.
#[test]
fn the_pumparound_command_writes_one_duty_and_refuses_the_rest() {
    let mut engine = engine();
    let (column, furnace) = (id(&engine, "column"), id(&engine, "furnace"));
    engine
        .apply(Command::SetPumparoundDuty {
            node: column,
            pumparound: 1,
            duty: Watt(22.0e6),
        })
        .unwrap();
    let duties: Vec<f64> = match &node(&engine.snapshot(), "column").kind {
        NodeKind::Column {
            cascade: Some(spec),
            ..
        } => spec.pumparounds.iter().map(|p| p.duty.value()).collect(),
        other => panic!("not a cascade column: {other:?}"),
    };
    assert_eq!(duties, vec![15.0e6, 22.0e6, 10.0e6]);

    for (node, pumparound, duty, expected) in [
        (column, 3, 1.0e6, "there is no pumparound 3"),
        (column, 0, -1.0, ">= 0"),
        (furnace, 0, 1.0e6, "not a column with pumparounds"),
    ] {
        let refusal = engine
            .apply(Command::SetPumparoundDuty {
                node,
                pumparound,
                duty: Watt(duty),
            })
            .unwrap_err()
            .to_string();
        assert!(refusal.contains(expected), "{refusal}");
    }
}

/// Pumparounds are refused at load on the `"cascade"` model, by name: constant
/// molar overflow would read nothing from them.
#[test]
fn pumparounds_need_the_heat_balance_model() {
    let src = CRUDE_UNIT.replace("separation = \"heat_balance\"", "separation = \"cascade\"");
    assert_ne!(src, CRUDE_UNIT);
    let refusal = match build_engine(&load_str(&src).expect("parses")) {
        Ok(_) => panic!("pumparounds on the cascade model must be refused"),
        Err(e) => e.to_string(),
    };
    assert!(
        refusal.contains("declares pumparounds") && refusal.contains("heat_balance"),
        "{refusal}"
    );
}
