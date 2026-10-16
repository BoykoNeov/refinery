//! The heat-balance cascade on a shipped plant (M60.1, docs/DESIGN.md §65):
//! `crude_column_hot_feed.toml` runs with its feed 100 K past its bubble point,
//! the `"cascade"` model refuses the same plant, and the column reports how it
//! ran.

use refinery_core::engine::Engine;
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::units::Watt;
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
        .as_ref()
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

/// The furnace or the reflux setting, run through the column's limit — the
/// keys the column screen will have (M60.2, docs/DESIGN.md §65.3). Nothing
/// halts: past the limit the reflux the column RUNS at rises over the one it
/// is set to and the condenser carries the furnace's extra heat; a setting
/// below what the floor needs runs at the floor's; a setting above it is run
/// as set.
#[test]
fn slow_the_furnace_and_the_reflux_run_through_the_limit() {
    let mut engine = engine(HOT_FEED);
    let id = |engine: &Engine, name: &str| {
        engine
            .snapshot()
            .nodes
            .iter()
            .find(|n| n.name == name)
            .unwrap_or_else(|| panic!("no node '{name}'"))
            .id
    };
    let furnace = id(&engine, "preheater");
    let column_id = id(&engine, "column");
    // (ran at, condenser W)
    let step = |engine: &mut Engine, label: &str, ticks: u32| -> (f64, f64) {
        for tick in 0..ticks {
            if let Err(e) = engine.tick() {
                panic!("{label}: tick {tick} failed: {e}");
            }
        }
        let snapshot = engine.snapshot();
        let node = column(&snapshot);
        (
            node.column_operation.as_ref().unwrap().reflux_ratio,
            node.column_duty.unwrap().condenser_w,
        )
    };

    let (ran, mut condenser) = step(&mut engine, "the plant as shipped", 3000);
    assert_eq!(ran, 2.0, "under its limit the column runs at its setting");
    let mut previous = ran;
    for megawatts in [70.0, 90.0, 110.0] {
        engine
            .apply(Command::SetFurnaceDuty {
                node: furnace,
                duty: Watt(megawatts * 1e6),
            })
            .unwrap();
        let (ran, now) = step(&mut engine, &format!("furnace at {megawatts} MW"), 1500);
        assert!(
            ran > previous,
            "{megawatts} MW: reflux {ran} after {previous}"
        );
        assert!(
            now > condenser,
            "{megawatts} MW: the condenser carries the heat"
        );
        (previous, condenser) = (ran, now);
    }
    let floor_needs = previous;
    for setting in [1.0, 0.0] {
        engine
            .apply(Command::SetRefluxRatio {
                node: column_id,
                reflux_ratio: setting,
            })
            .unwrap();
        let (ran, _) = step(&mut engine, &format!("reflux set to {setting}"), 600);
        assert!(
            (ran - floor_needs).abs() < 1e-3 * floor_needs,
            "set to {setting}, the column should run at the floor's {floor_needs}, ran {ran}"
        );
    }
    engine
        .apply(Command::SetRefluxRatio {
            node: column_id,
            reflux_ratio: 2.0 * floor_needs,
        })
        .unwrap();
    let (ran, _) = step(&mut engine, "reflux set above the floor", 600);
    assert_eq!(
        ran,
        2.0 * floor_needs,
        "a setting above the floor's is run as set"
    );
}

/// `SetRefluxRatio` writes a cascade column's declared ratio and is refused
/// everywhere it has nothing to write.
#[test]
fn the_reflux_command_is_refused_where_there_is_no_reflux() {
    let mut engine = engine(HOT_FEED);
    let id = |name: &str| {
        engine
            .snapshot()
            .nodes
            .iter()
            .find(|n| n.name == name)
            .unwrap()
            .id
    };
    let (column_id, furnace) = (id("column"), id("preheater"));
    for bad in [-0.1, f64::NAN, f64::INFINITY] {
        let refusal = engine
            .apply(Command::SetRefluxRatio {
                node: column_id,
                reflux_ratio: bad,
            })
            .unwrap_err()
            .to_string();
        assert!(refusal.contains("finite and >= 0"), "{bad}: {refusal}");
    }
    let refusal = engine
        .apply(Command::SetRefluxRatio {
            node: furnace,
            reflux_ratio: 1.0,
        })
        .unwrap_err()
        .to_string();
    assert!(refusal.contains("is not a column"), "{refusal}");

    let mut splitter = self::engine(include_str!("../../../scenarios/crude_column.toml"));
    let cut_point = splitter
        .snapshot()
        .nodes
        .iter()
        .find(|n| n.name == "column")
        .unwrap()
        .id;
    let refusal = splitter
        .apply(Command::SetRefluxRatio {
            node: cut_point,
            reflux_ratio: 1.0,
        })
        .unwrap_err()
        .to_string();
    assert!(refusal.contains("has no reflux to set"), "{refusal}");

    engine
        .apply(Command::SetRefluxRatio {
            node: column_id,
            reflux_ratio: 3.5,
        })
        .unwrap();
    let declared = match &column(&engine.snapshot()).kind {
        refinery_core::graph::NodeKind::Column {
            cascade: Some(spec),
            ..
        } => spec.reflux_ratio,
        other => panic!("not a cascade column: {other:?}"),
    };
    assert_eq!(declared, 3.5, "the consequence is the declared ratio");
}

/// **The shipped plant past its limit** (`crude_column_overheated.toml`): the
/// feed near 350 °C, the column running above its declared reflux of 2, and
/// the declared 2 still on its `kind`.
#[test]
fn the_overheated_plant_runs_above_its_declared_reflux() {
    let snapshot = run(
        &mut engine(include_str!(
            "../../../scenarios/crude_column_overheated.toml"
        )),
        3000,
    );
    assert!(
        feed_temperature(&snapshot) > 615.0,
        "{}",
        feed_temperature(&snapshot)
    );
    let node = column(&snapshot);
    let ran = node.column_operation.as_ref().unwrap().reflux_ratio;
    assert!(
        ran > 3.0,
        "the reflux should have risen well above 2, ran at {ran}"
    );
    match &node.kind {
        refinery_core::graph::NodeKind::Column {
            cascade: Some(spec),
            ..
        } => assert_eq!(spec.reflux_ratio, 2.0, "the setting is not overwritten"),
        other => panic!("not a cascade column: {other:?}"),
    }
}
