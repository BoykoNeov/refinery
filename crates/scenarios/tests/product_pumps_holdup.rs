//! M86.5 (docs/DESIGN.md §84.5): a column that holds liquid, its product
//! lines needing their pumps — the holdup plant (`crude_unit_holdup.toml`,
//! untouched) with a product line on every draw, built inline here as the
//! M86.4 grade plant's (a 15 m fall to a pump of 100 m head, a check valve, a
//! water cooler, 1.5 km of rundown to a tank at the draws' level).

use crate::product_line_reference::build;
use crate::short_draw_line::{flow, set_pump, tick};
use refinery_core::engine::Engine;
use refinery_core::graph::{HoldupState, NodeKind};
use refinery_core::snapshot::Snapshot;

const HOLDUP_PLANT: &str = include_str!("../../../scenarios/crude_unit_holdup.toml");

/// The oil draws, top down, by their line's first pipe's prefix, and the
/// stage each draws from.
const OIL: [(&str, usize); 5] = [
    ("naphtha", 0),
    ("kerosene", 4),
    ("diesel", 7),
    ("gas_oil", 10),
    ("residue", 13),
];

/// Each product's cooling water [kg/s], cooler UA [kW/K] and rundown
/// diameter [m] (`make_pumped.py grade`).
const GRADE: [(&str, f64, f64, f64); 5] = [
    ("naphtha", 75.0, 150.0, 0.154),
    ("kerosene", 115.0, 90.0, 0.131),
    ("diesel", 190.0, 120.0, 0.14),
    ("gas_oil", 150.0, 70.0, 0.119),
    ("residue", 460.0, 100.0, 0.152),
];

/// `src` (a plant whose draws run straight to their tanks through
/// `<product>_draw`, 20 m of 15 cm) with the grade plant's product line on
/// every draw.
fn grade_lines(src: &str) -> String {
    let mut src = src.to_string();
    for (product, water, ua, diameter) in GRADE {
        let tank = format!("{product}_tank");
        let before = src.clone();
        src = src
            .replacen(
                &format!("{{ outlet = \"{tank}\","),
                &format!("{{ outlet = \"{product}_pump\","),
                1,
            )
            .replacen(
                &format!(
                    "name = \"{product}_draw\"\nfrom = \"column\"\nto = \"{tank}\"\n\
                     length_m = 20.0\ndiameter_m = 0.15"
                ),
                &format!(
                    "name = \"{product}_suction\"\nfrom = \"column\"\nto = \"{product}_pump\"\n\
                     length_m = 20.0\ndiameter_m = 0.20\nelevation_change_m = -15.0"
                ),
                1,
            );
        assert_eq!(
            src.matches(&format!("{product}_pump")).count(),
            before.matches(&format!("{product}_pump")).count() + 2,
            "the {product} draw was not rewired"
        );
        src.push_str(&format!(
            r#"
[nodes.{product}_pump]
type = "pump"
h0_m = 100.0
a = 2000.0
on = true

[nodes.{product}_check]
type = "check_valve"
kv = 400.0
full_open_bar = 0.015

[nodes.{product}_cooler]
type = "water_cooler"
ua_kw_per_k = {ua}
water_inlet_c = 30.0
water_flow_kg_per_s = {water}

[[pipes]]
name = "{product}_discharge"
from = "{product}_pump"
to = "{product}_check"
length_m = 30.0
diameter_m = 0.15

[[pipes]]
name = "{product}_check_outlet"
from = "{product}_check"
to = "{product}_cooler"
length_m = 1.0
diameter_m = 0.15

[[pipes]]
name = "{product}_rundown"
from = "{product}_cooler"
to = "{tank}"
length_m = 1500.0
diameter_m = {diameter}
elevation_change_m = 15.0
"#
        ));
    }
    src
}

fn holdup_state(engine: &Engine) -> HoldupState {
    let node = engine.graph.find_node("column").unwrap();
    match &engine.graph.node(node).kind {
        NodeKind::Column {
            cascade: Some(spec),
            ..
        } => spec
            .holdup
            .as_ref()
            .and_then(|h| h.state.clone())
            .expect("a column that holds liquid"),
        _ => panic!("the column is a cascade column"),
    }
}

/// Each component's mass the column holds [kg]: its stages' and its drum's.
fn inventory(held: &HoldupState) -> Vec<f64> {
    let mut total = vec![0.0; held.drum_composition.fractions().len()];
    let stages = held.stage_liquid.iter().zip(&held.stage_composition);
    let drum = std::iter::once((&held.drum_liquid, &held.drum_composition));
    for (mass, composition) in stages.chain(drum) {
        for (slot, w) in total.iter_mut().zip(composition.fractions()) {
            *slot += mass.value() * w;
        }
    }
    total
}

/// The column's worst per-component imbalance over one tick [kg/s]: what
/// came in, less what left (the draws and any slop), less what it now holds
/// more.
fn worst_books(engine: &Engine, before: &[f64], after: &[f64], s: &Snapshot) -> f64 {
    let dt = 0.1;
    let column = s.nodes.iter().find(|n| n.name == "column").unwrap();
    let slop = column
        .column_operation
        .as_ref()
        .map(|op| op.slop.clone())
        .unwrap_or_default();
    let carried = |name: &str, c: usize| {
        let e = s.edges.iter().find(|e| e.name == name).unwrap();
        e.stream.mass_flow.value() * e.stream.composition.fractions()[c]
    };
    let mut worst: f64 = 0.0;
    for c in 0..engine.slate.len() {
        let mut balance = carried("transfer_line", c) - (after[c] - before[c]) / dt;
        for (product, _) in OIL {
            balance -= carried(&format!("{product}_suction"), c);
        }
        for vessel in &slop {
            balance -= vessel.flow_kg_s * vessel.composition.fractions()[c];
        }
        worst = worst.max(balance.abs());
    }
    worst
}

/// The line limit the column ran `draw` (a plan index) at this tick [kg/s],
/// if its line was short.
fn ran_short(engine: &Engine, draw: usize) -> Option<(f64, f64)> {
    let column = engine.graph.find_node("column").unwrap();
    engine.node_states().column_separation[&column]
        .short_draws
        .iter()
        .find(|short| short.draw == draw)
        .map(|short| (short.wanted.value(), short.delivered.value()))
}

/// **A stopped side pump on a column that holds liquid**: the line still
/// carries part of the draw (the column pushes it through the stopped pump),
/// the column runs the draw at what the line delivers and the rest stays on
/// its tray and runs down — the plant runs on, every component's books closing. Was
/// (M86.5's probe): the run halted on the first tick, the column's draw
/// 1.3e-7 of itself over its line's limit — set in moles at the tray's
/// start-of-tick liquid, reported in mass at its end-of-tick liquid.
fn a_stopped_pump_runs_at_its_line_limit(product: &str, index: usize) {
    let src = grade_lines(HOLDUP_PLANT);
    let mut engine = build(&src).unwrap();
    let s = tick(&mut engine, 30);
    let normal = flow(&s, &format!("{product}_suction"));
    set_pump(&mut engine, product, false);
    let mut worst: f64 = 0.0;
    for t in 0..600 {
        let before = inventory(&holdup_state(&engine));
        engine
            .tick()
            .unwrap_or_else(|e| panic!("{product} tick {t}: the plant must run: {e}"));
        let s = engine.snapshot();
        let after = inventory(&holdup_state(&engine));
        worst = worst.max(worst_books(&engine, &before, &after, &s));
        let drawn = flow(&s, &format!("{product}_suction"));
        let cut = drawn / normal;
        assert!(
            cut > 0.5 && cut < 0.6,
            "{product} tick {t}: carries {cut} of its flow"
        );
        let (wanted, got) = ran_short(&engine, index).expect("the line short");
        assert!(wanted > got, "{product} tick {t}: {wanted} {got}");
        assert!(
            (drawn - got).abs() <= 1e-9 * got,
            "{product} tick {t}: drawn {drawn}, its line's limit {got}"
        );
    }
    // As at rest (`column_holdup::each_component_is_conserved_every_tick`).
    assert!(worst < 1e-8, "{product}: {worst} kg/s");
}

#[test]
fn a_stopped_kerosene_pump_on_a_holding_column_runs_at_its_line_limit() {
    a_stopped_pump_runs_at_its_line_limit("kerosene", 1);
}

/// The same for the distillate (what its line cannot take stays in the
/// drum) and the bottoms (in the base): set in moles at the drum's or the
/// base's start-of-tick liquid like a side draw, and held to the limit the
/// same way.
#[test]
fn a_stopped_naphtha_pump_on_a_holding_column_runs_at_its_line_limit() {
    a_stopped_pump_runs_at_its_line_limit("naphtha", 0);
}

#[test]
fn a_stopped_residue_pump_on_a_holding_column_runs_at_its_line_limit() {
    a_stopped_pump_runs_at_its_line_limit("residue", 4);
}
