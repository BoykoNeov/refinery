//! M64.1 — a column's reboiler held (docs/DESIGN.md §69.1), through the loader
//! and a running plant. The column's equations are gated in `refinery-solvers`'
//! `reference/held_reboiler.rs`; these gates hold the plant's side: the key
//! reaches the column, a held duty is what the snapshot publishes, and the
//! loader refuses a held reboiler where nothing could hold it.
//!
//! The fixture is `scenarios/crude_unit_steam.toml` with `reboiler_duty_mw`
//! added, built here rather than shipped: the shipped files are the regression
//! anchor.

use refinery_core::snapshot::{NodeSnapshot, Snapshot};
use refinery_core::Engine;

const STEAM: &str = include_str!("../../../scenarios/crude_unit_steam.toml");

/// The steamed crude unit with its reboiler held at `duty_mw`.
fn held(duty_mw: &str) -> String {
    let plant = STEAM.replacen(
        "reflux_ratio = 4.0\n",
        &format!("reflux_ratio = 4.0\nreboiler_duty_mw = {duty_mw}\n"),
        1,
    );
    assert_ne!(plant, STEAM);
    plant
}

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the plant parses");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant builds: {e}"))
}

fn refusal(src: &str) -> String {
    let file = match refinery_scenarios::load_str(src) {
        Ok(file) => file,
        Err(e) => return e.to_string(),
    };
    match refinery_scenarios::build_engine(&file) {
        Ok(_) => panic!("the plant must be refused"),
        Err(e) => e.to_string(),
    }
}

fn column(s: &Snapshot) -> &NodeSnapshot {
    s.nodes.iter().find(|n| n.name == "column").unwrap()
}

/// **Held at the plant's own first-tick closure, the plant's first tick is the
/// shipped one.** The shipped plant's reboiler closes at some duty on tick 1;
/// the same plant holding exactly that duty runs tick 1 at the declared reflux
/// of 4, its stages where the shipped plant's are, and publishes the held duty
/// to the bit.
#[test]
fn held_at_its_closure_the_plant_runs_as_shipped() {
    let mut shipped = build(STEAM);
    shipped.tick().unwrap();
    let s = shipped.snapshot();
    let duty_w = column(&s).column_duty.unwrap().reboiler_w;

    let mut engine = build(&held(&format!("{:e}", duty_w / 1.0e6)));
    engine
        .tick()
        .unwrap_or_else(|e| panic!("the held plant runs: {e}"));
    let h = engine.snapshot();
    let published = column(&h).column_duty.unwrap().reboiler_w;
    assert!(
        (published - duty_w).abs() <= 1.0e-9 * duty_w,
        "published {published} W, held {duty_w} W"
    );
    let (a, b) = (
        column(&s).column_operation.clone().unwrap(),
        column(&h).column_operation.clone().unwrap(),
    );
    assert!(
        (b.reflux_ratio - a.reflux_ratio).abs() < 1.0e-6,
        "{} against {}",
        b.reflux_ratio,
        a.reflux_ratio
    );
    for (j, (x, y)) in a
        .stage_temperatures_k
        .iter()
        .zip(&b.stage_temperatures_k)
        .enumerate()
    {
        assert!(
            (x - y).abs() < 1.0e-4,
            "stage {}: {x} K against {y} K",
            j + 1
        );
    }
}

/// **Held below its closure, the plant runs on less reflux.** 10 MW under the
/// shipped plant's tick-1 closure, the column publishes the held duty and runs
/// below its declared reflux of 4 — the duty, not the declared ratio, setting
/// it.
#[test]
fn held_below_its_closure_the_reflux_falls() {
    let mut shipped = build(STEAM);
    shipped.tick().unwrap();
    let duty_w = column(&shipped.snapshot()).column_duty.unwrap().reboiler_w - 10.0e6;

    let mut engine = build(&held(&format!("{:e}", duty_w / 1.0e6)));
    engine
        .tick()
        .unwrap_or_else(|e| panic!("the held plant runs: {e}"));
    let h = engine.snapshot();
    let published = column(&h).column_duty.unwrap().reboiler_w;
    assert!(
        (published - duty_w).abs() <= 1.0e-9 * duty_w,
        "published {published} W, held {duty_w} W"
    );
    let reflux = column(&h).column_operation.clone().unwrap().reflux_ratio;
    assert!(reflux < 4.0 - 0.05, "reflux {reflux}");
}

/// **The loader's refusals, by name** (§69.1): a held reboiler under any model
/// but the heat-balance cascade, and a duty below zero.
#[test]
fn a_held_reboiler_is_refused_where_nothing_holds_it() {
    let e =
        refusal(&held("0.0").replace("separation = \"heat_balance\"", "separation = \"cascade\""));
    assert!(
        e.contains("holds its reboiler") && e.contains("heat_balance"),
        "{e}"
    );
    let e = refusal(&held("-1.0"));
    assert!(
        e.contains("holds its reboiler at -1 MW") && e.contains("at least zero"),
        "{e}"
    );
}
