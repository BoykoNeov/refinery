//! M64.1 — a column's reboiler held (docs/DESIGN.md §69.1), through the loader
//! and a running plant. The column's equations are gated in `refinery-solvers`'
//! `reference/held_reboiler.rs`; these gates hold the plant's side: the key
//! reaches the column, a held duty is what the snapshot publishes, and the
//! loader refuses a held reboiler where nothing could hold it. M64.2's add
//! the plant started with nothing to start from (§69.3): hot, its first tick
//! runs; on a cold coil it is refused by name.
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

/// The steamed crude unit hot and stripped (M64.0's probe plant, DESIGN §69.0):
/// its furnace at 68 MW, its coil starting at `coil_c`, its pumparounds off and
/// its reboiler held at zero.
fn stripped(coil_c: &str) -> String {
    let mut plant = held("0.0");
    for (from, to) in [
        ("duty_mw = 20.0\n", "duty_mw = 68.0\n".to_string()),
        (
            "coil_temperature_c = 322.05\n",
            format!("coil_temperature_c = {coil_c}\n"),
        ),
        (
            "pumparounds = [{ stage = 4, duty_mw = 15.0 }, { stage = 6, duty_mw = 15.0 }, \
             { stage = 7, duty_mw = 10.0 }]",
            "pumparounds = [{ stage = 4, duty_mw = 0.0 }, { stage = 6, duty_mw = 0.0 }, \
             { stage = 7, duty_mw = 0.0 }]"
                .to_string(),
        ),
    ] {
        assert_eq!(plant.matches(from).count(), 1, "{from}");
        plant = plant.replacen(from, &to, 1);
    }
    plant
}

/// **A hot plant holding its reboiler at zero starts on its first tick**
/// (M64.2, docs/DESIGN.md §69.3): nothing to start from, the column is started
/// by the continuation, and the plant runs on. Its bottom at 383.05 °C on its
/// 0.87 kg/s of steam and its reflux at 4.809 are M64.0's warm walk's settled
/// 383.1 °C and 4.810 (§69.0, finding 1), here on the first tick.
#[test]
fn a_hot_plant_holding_zero_starts_on_its_first_tick() {
    let mut engine = build(&stripped("488.0"));
    for tick in 1..=3 {
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        let s = engine.snapshot();
        assert_eq!(column(&s).column_duty.unwrap().reboiler_w, 0.0);
        let operation = column(&s).column_operation.clone().unwrap();
        let bottom = operation.stage_temperatures_k.last().unwrap() - 273.15;
        assert!(
            (bottom - 383.05).abs() < 0.05,
            "tick {tick}: bottom {bottom} °C"
        );
        assert!(
            (operation.reflux_ratio - 4.809).abs() < 1.0e-3,
            "tick {tick}: reflux {}",
            operation.reflux_ratio
        );
    }
}

/// **A cooler feed starts on the base's side cooling** (M64.2, §69.3, the
/// user's decision: derived from the feed). With the coil starting at 405 °C
/// — the settled coil of a 40 MW furnace (§69.0, finding 3) — the base with
/// the plant's own pumparounds, all off, refuses (measured: its warm-up's
/// stage liquid negative); with 1.5 times the feed's latent heat flow spread
/// over the stages above the feed it starts, and the column runs at zero with
/// its bottom near 343.9 °C, its gas oil cut back to 76 % of its share (M67.1,
/// §72.1; before M67 it ran that tray under the minimum liquid, at 348.9 °C).
#[test]
fn a_cooler_feed_starts_on_the_bases_side_cooling() {
    let mut engine = build(&stripped("405.0"));
    engine
        .tick()
        .unwrap_or_else(|e| panic!("the first tick: {e}"));
    let s = engine.snapshot();
    assert_eq!(column(&s).column_duty.unwrap().reboiler_w, 0.0);
    let operation = column(&s).column_operation.clone().unwrap();
    let bottom = operation.stage_temperatures_k.last().unwrap() - 273.15;
    assert!((bottom - 343.88).abs() < 0.05, "bottom {bottom} °C");
    let cut = &column(&s).column_draws_cut_back;
    assert_eq!(cut.len(), 1, "{cut:?}");
    assert_eq!(cut[0].draw, 4, "the gas-oil draw");
    assert!((cut[0].of_declared - 0.764).abs() < 0.005, "{cut:?}");
}

/// **A cold coil cannot start a column without its reboiler** (§69.0, finding
/// 5): the shipped plant's own coil, 322 °C, sends its crude to the column near
/// 290 °C, and the continuation stops short of zero — refused on tick 1 by
/// name. Starting a plant from cold is ledger row C15, not a solver's matter.
#[test]
fn a_cold_coil_cannot_start_a_column_without_its_reboiler() {
    let mut engine = build(&stripped("322.05"));
    let e = engine
        .tick()
        .expect_err("a held column on a cold coil")
        .to_string();
    assert!(
        e.contains("starts from the same column with its reboiler")
            && e.contains("stopped at step"),
        "{e}"
    );
}
