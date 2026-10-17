//! M62.4 — the crude unit as a plant that models boiling
//! (`scenarios/crude_unit_boiling.toml`, docs/DESIGN.md §67.16): every piece
//! M62.4 built, on one plant. The pieces are gated each in its own file
//! (`heat_on_boiling_reference.rs`, `product_line_reference.rs`,
//! `coil_cell_reference.rs`, `boiling_jet_reference.rs`); these gates hold the
//! shipped plant to what it is for.

use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::Engine;

const PLANT: &str = include_str!("../../../scenarios/crude_unit_boiling.toml");
const PRODUCTS: [&str; 5] = ["naphtha", "kerosene", "diesel", "gas_oil", "residue"];

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the plant parses");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant builds: {e}"))
}

fn run(engine: &mut Engine, ticks: u32) {
    for t in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
        assert!(
            engine.snapshot().solver.converged,
            "tick {t} did not converge"
        );
    }
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn edge<'a>(s: &'a Snapshot, name: &str) -> &'a EdgeSnapshot {
    s.edges.iter().find(|e| e.name == name).unwrap()
}

/// **The plant boils where a crude unit boils, and only there.** The crude
/// enters its coil liquid above its bubble pressure and leaves it boiling; the
/// transfer line is two-phase into the column; the column draws liquids; every
/// product reaches its tank cooled and liquid, so no tank vents; the cooling
/// water returns below 50 °C; the coil's metal stands far under its 550 °C
/// limit. Measured at 6 000 ticks: 134.4 kg/s, the coil's inlet at 14.7 bar,
/// its outlet 281.4 °C at 2.08 bar and 39.2 % vapour, its metal 318.3 °C
/// (where it starts); the products 40.7–165.8 °C into their tanks, the water
/// back at 45.2–46.0 °C.
#[test]
fn the_boiling_crude_unit_boils_in_its_coil_and_nowhere_downstream() {
    let mut engine = build(PLANT);
    run(&mut engine, 100);
    let s = engine.snapshot();
    let inlet = node(&s, "furnace__cell_1");
    assert!(
        inlet.vapour_fraction.is_none(),
        "the crude enters its coil liquid"
    );
    assert!(
        inlet.pressure_pa > 14e5,
        "the coil's inlet stands at {} Pa",
        inlet.pressure_pa
    );
    let furnace = node(&s, "furnace");
    let share = furnace
        .vapour_fraction
        .expect("the crude leaves its coil boiling");
    assert!(share > 0.3, "the outlet is {share} vapour");
    assert!(
        edge(&s, "transfer_line").stream.vapour_fraction.is_some(),
        "the transfer line is two-phase"
    );
    let metal = match &furnace.kind {
        NodeKind::Furnace { coil, tubes, .. } => {
            assert!(!tubes.state.is_failed(), "the tubes hold");
            coil.temperature.value()
        }
        other => panic!("{other:?}"),
    };
    assert!(
        metal - 273.15 < 400.0,
        "the metal stands at {} °C",
        metal - 273.15
    );
    for product in PRODUCTS {
        let suction = edge(&s, &format!("{product}_suction"));
        assert!(
            suction.stream.vapour_fraction.is_none(),
            "{product}: the draw is liquid"
        );
        assert!(
            suction.stream.mass_flow.value() > 10.0,
            "{product}: a real draw"
        );
        let cooler = node(&s, &format!("{product}_cooler"));
        assert!(cooler.vapour_fraction.is_none(), "{product}: cooled liquid");
        let water = cooler
            .water_cooler
            .expect("the cooler cooled")
            .water_outlet_k
            - 273.15;
        assert!(water < 50.0, "{product}: its water returns at {water} °C");
        let vent = edge(&s, &format!("{product}_tank__boiloff_vent"));
        assert_eq!(
            vent.stream.mass_flow.value(),
            0.0,
            "{product}: its tank vents"
        );
    }
}

/// **Both fidelities agree** on the shipped plant.
#[test]
fn slow_the_boiling_crude_unit_runs_on_both_fidelities() {
    let mut answers = Vec::new();
    for flow in ["newton", "simple"] {
        let mut engine = build(&PLANT.replace("flow = \"newton\"", &format!("flow = \"{flow}\"")));
        run(&mut engine, 150);
        let s = engine.snapshot();
        answers.push((
            edge(&s, "crude_line").stream.mass_flow.value(),
            node(&s, "furnace").temperature_k,
            node(&s, "residue_cooler").temperature_k,
        ));
    }
    let (a, b) = (answers[0], answers[1]);
    assert!(
        (a.0 - b.0).abs() < 1e-6 * a.0 && (a.1 - b.1).abs() < 1e-4 && (a.2 - b.2).abs() < 1e-4,
        "newton {a:?}, simple {b:?}"
    );
}
