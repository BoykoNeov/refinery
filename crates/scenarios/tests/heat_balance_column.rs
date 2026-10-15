//! The heat-balance cascade on a shipped plant (M60.1, docs/DESIGN.md §65):
//! `crude_column_hot_feed.toml` runs with its feed 100 K past its bubble point,
//! the `"cascade"` model refuses the same plant, and the column reports how it
//! ran.

use refinery_core::engine::Engine;
use refinery_core::snapshot::Snapshot;
use refinery_scenarios::{build_engine, load_str};

const HOT_FEED: &str = include_str!("../../../scenarios/crude_column_hot_feed.toml");
const SATURATED: &str = include_str!("../../../scenarios/crude_column_cascade.toml");

fn engine(src: &str) -> Engine {
    build_engine(&load_str(src).expect("the scenario should parse")).expect("and build")
}

fn run(engine: &mut Engine, ticks: u32) -> Snapshot {
    for tick in 0..ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick} failed: {e}"));
    }
    engine.snapshot()
}

/// The column node's snapshot.
fn column(snapshot: &Snapshot) -> &refinery_core::snapshot::NodeSnapshot {
    snapshot
        .nodes
        .iter()
        .find(|n| n.name == "column")
        .expect("the plant has a column")
}

fn feed_temperature(snapshot: &Snapshot) -> f64 {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == "hot_feed_line__downstream")
        .expect("the feed line reaches the column")
        .stream
        .temperature
        .value()
}

/// **The hot plant runs**, and its column says how: about two-fifths of the
/// feed flashes, the reflux runs at the declared 2, and the reboiler carries
/// half of what the saturated twin's does — the furnace took the rest.
#[test]
fn the_hot_feed_plant_runs_and_reports_its_flash() {
    let hot = run(&mut engine(HOT_FEED), 3000);
    let saturated = run(&mut engine(SATURATED), 3000);

    assert!(
        feed_temperature(&hot) > 520.0,
        "the furnace should land the feed near 250 °C, got {} K",
        feed_temperature(&hot)
    );
    let operation = column(&hot)
        .column_operation
        .expect("a heat-balance column reports how it ran");
    assert!(
        (0.3..0.55).contains(&operation.feed_vapour_fraction),
        "{}",
        operation.feed_vapour_fraction
    );
    assert_eq!(operation.reflux_ratio, 2.0);

    let hot_duty = column(&hot).column_duty.unwrap();
    let saturated_duty = column(&saturated).column_duty.unwrap();
    assert!(
        hot_duty.reboiler_w < 0.6 * saturated_duty.reboiler_w,
        "reboiler {} W hot against {} W saturated",
        hot_duty.reboiler_w,
        saturated_duty.reboiler_w
    );
    // The stage balances, not the closure arithmetic: a hot feed leaves less
    // vapour below it to strip the bottoms, so the bottoms keeps more kerosene
    // (§65.7: 1.9 % against 0.7 %). The reboiler figure above holds on the
    // closure alone; this one needs the feed's enthalpy on its stage.
    let kerosene = |snapshot: &Snapshot| {
        snapshot
            .edges
            .iter()
            .find(|e| e.name == "bottoms_draw")
            .expect("the bottoms draw")
            .stream
            .composition
            .fractions()[2]
    };
    assert!(
        kerosene(&hot) > 2.0 * kerosene(&saturated),
        "bottoms kerosene {} hot against {} saturated",
        kerosene(&hot),
        kerosene(&saturated)
    );
    // The reflux ratio fixes the top vapour, so the condenser barely moves.
    assert!(
        (hot_duty.condenser_w / saturated_duty.condenser_w - 1.0).abs() < 0.02,
        "condenser {} W hot against {} W saturated",
        hot_duty.condenser_w,
        saturated_duty.condenser_w
    );
    // The `"cascade"` model publishes no operation — that absence is what keeps
    // its four plants byte-identical.
    assert!(column(&saturated).column_operation.is_none());
}

/// **The same plant on the `"cascade"` model is refused** — the boundary
/// ledger row C5 names, and the reason the heat-balance model exists. The
/// refusal is a tick `Err` naming the superheat, not a plausible number.
#[test]
fn the_cascade_model_refuses_the_same_plant() {
    let src = HOT_FEED.replace("separation = \"heat_balance\"", "separation = \"cascade\"");
    assert_ne!(src, HOT_FEED, "the fidelity line should have been swapped");
    let mut engine = engine(&src);
    let refusal = (0..3000)
        .find_map(|_| engine.tick().err())
        .expect("the cascade must refuse a feed 100 K past its bubble point");
    assert!(refusal.to_string().contains("superheated"), "{refusal}");
}

/// The loader refuses `"heat_balance"` exactly where it refuses `"cascade"`,
/// naming the fidelity the file selected.
#[test]
fn the_loader_refuses_a_heat_balance_column_without_a_k_value() {
    let src = HOT_FEED.replace("thermo = \"trouton\"", "thermo = \"constant\"");
    let refusal = match build_engine(&load_str(&src).expect("parses")) {
        Ok(_) => panic!("a heat-balance column on constant thermo must be refused"),
        Err(e) => e.to_string(),
    };
    assert!(
        refusal.contains("separation = \"heat_balance\" with thermo = \"constant\""),
        "{refusal}"
    );
}
