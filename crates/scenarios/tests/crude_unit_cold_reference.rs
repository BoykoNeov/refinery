//! M66.3 — the cold crude unit (`scenarios/crude_unit_cold.toml`, docs/DESIGN.md
//! §71.4): the stripped crude unit before its start-up, the furnace low and the
//! column on its reboiler. It starts where it stays; and started up by the
//! commands a control room would send — the furnace to 58 MW, the pumparounds
//! off, the reboiler walked down from the duty it runs at — it reaches the
//! stripped plant's column, held at zero, never refused. Both fidelities.
//!
//! The numbers are the running plant's (`tools/steptime/prototypes/m66_3/`).
//! Run on to 24 000 ticks the start-up settles on `crude_unit_stripped.toml`'s
//! own column to every printed digit (bottom 363.634 °C, reflux 3.4604, the
//! coil 436.88 °C) — measured, not gated here: the coil takes ~18 000 ticks to
//! settle.

use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot};
use refinery_core::units::Watt;
use refinery_core::Engine;

const COLD: &str = include_str!("../../../scenarios/crude_unit_cold.toml");

fn build(flow: &str) -> Engine {
    let src = COLD.replacen("flow = \"newton\"", &format!("flow = \"{flow}\""), 1);
    let file = refinery_scenarios::load_str(&src).expect("the plant parses");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant builds: {e}"))
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn reboiler_mw(s: &Snapshot) -> f64 {
    node(s, "column").column_duty.unwrap().reboiler_w / 1.0e6
}

fn reflux(s: &Snapshot) -> f64 {
    node(s, "column")
        .column_operation
        .clone()
        .unwrap()
        .reflux_ratio
}

fn bottom_c(s: &Snapshot) -> f64 {
    *node(s, "column")
        .column_operation
        .clone()
        .unwrap()
        .stage_temperatures_k
        .last()
        .unwrap()
        - 273.15
}

fn coil_c(s: &Snapshot) -> f64 {
    match &node(s, "furnace").kind {
        NodeKind::Furnace { coil, .. } => coil.temperature.value() - 273.15,
        other => panic!("not a furnace: {other:?}"),
    }
}

fn charge(s: &Snapshot) -> f64 {
    s.edges
        .iter()
        .find(|e| e.name == "transfer_line")
        .unwrap()
        .stream
        .mass_flow
        .value()
}

/// **It starts where it stays.** Every tick of the first hundred: the
/// reboiler free at the declared reflux of 4 and about 87.95 MW, the bottom
/// 580.18 °C, the charge 112.80 kg/s, the coil within 0.01 K of its declared
/// 322.06 °C.
fn starts_where_it_stays(flow: &str) {
    let mut engine = build(flow);
    for t in 1..=100 {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("{flow}, tick {t}: {e}"));
        let s = engine.snapshot();
        assert_eq!(reflux(&s), 4.0, "{flow}, tick {t}");
        assert!(
            (reboiler_mw(&s) - 87.945).abs() < 0.01,
            "{flow}, tick {t}: reboiler {} MW",
            reboiler_mw(&s)
        );
        assert!(
            (bottom_c(&s) - 580.181).abs() < 0.01,
            "{flow}, tick {t}: bottom {} °C",
            bottom_c(&s)
        );
        assert!(
            (charge(&s) - 112.80).abs() < 0.01,
            "{flow}, tick {t}: charge {} kg/s",
            charge(&s)
        );
        assert!(
            (coil_c(&s) - 322.06).abs() < 0.01,
            "{flow}, tick {t}: coil {} °C",
            coil_c(&s)
        );
    }
}

#[test]
fn the_cold_plant_starts_where_it_stays_newton() {
    starts_where_it_stays("newton");
}

#[test]
fn the_cold_plant_starts_where_it_stays_simple() {
    starts_where_it_stays("simple");
}

/// **The start-up** (§71.3): the furnace to 58 MW on tick 10; the pumparounds
/// off on tick 500; from then, every 10 ticks, the reboiler held at the duty it
/// ran at less 2.5 MW — the screen's key. The duty never rises and the column
/// declines none of it: since M67 (§72.1, the user's decision) it cuts its gas
/// oil back where the feed is still too cool for the step, so the walk
/// reaches zero on tick 820 (M66: tick 2 320, after 1 534 declined ticks).
/// The gas oil is cut from tick 740, to 0.7 % of its share at the walk's foot,
/// and comes back as the feed heats: 64 % on tick 2 000, the reflux 2.895,
/// the bottom 338.3 °C. Never refused, on both fidelities alike.
fn the_start_up(flow: &str) {
    let mut engine = build(flow);
    let id = |name: &str| node(&engine.snapshot(), name).id;
    let (furnace, column) = (id("furnace"), id("column"));
    let mut declined = 0;
    let mut zero_at = None;
    let mut first_cut = None;
    let mut least = f64::INFINITY;
    let mut last = f64::INFINITY;
    for t in 1..=2_400u64 {
        if t == 10 {
            engine
                .apply(Command::SetFurnaceDuty {
                    node: furnace,
                    duty: Watt(58.0e6),
                })
                .unwrap();
        }
        if t == 500 {
            for pumparound in 0..3 {
                engine
                    .apply(Command::SetPumparoundDuty {
                        node: column,
                        pumparound,
                        duty: Watt::ZERO,
                    })
                    .unwrap();
            }
        }
        if t >= 500 && t % 10 == 0 {
            let running = reboiler_mw(&engine.snapshot()) * 1.0e6;
            if running > 0.0 {
                engine
                    .apply(Command::SetReboilerDuty {
                        node: column,
                        duty: Some(Watt((running - 2.5e6).max(0.0))),
                    })
                    .unwrap();
            }
        }
        engine
            .tick()
            .unwrap_or_else(|e| panic!("{flow}, tick {t}: {e}"));
        let s = engine.snapshot();
        let duty = reboiler_mw(&s);
        if t > 500 {
            assert!(duty <= last, "{flow}, tick {t}: {duty} MW after {last} MW");
        }
        last = duty;
        if node(&s, "column").column_reboiler_declined.is_some() {
            declined += 1;
        }
        if duty == 0.0 && zero_at.is_none() {
            zero_at = Some(t);
        }
        // Only the gas oil (draw 4) is ever cut back.
        let cut = &node(&s, "column").column_draws_cut_back;
        assert!(cut.iter().all(|c| c.draw == 4), "{flow}, tick {t}: {cut:?}");
        if let Some(gas_oil) = cut.first() {
            first_cut.get_or_insert(t);
            least = least.min(gas_oil.of_declared);
        }
        if t == 2_000 {
            assert_eq!(duty, 0.0, "{flow}");
            assert!(
                (reflux(&s) - 2.895).abs() < 1.0e-3,
                "{flow}: {}",
                reflux(&s)
            );
            assert!(
                (bottom_c(&s) - 338.31).abs() < 0.01,
                "{flow}: {} °C",
                bottom_c(&s)
            );
            let share = cut.first().map(|c| c.of_declared);
            assert!(
                share.is_some_and(|share| (share - 0.641).abs() < 0.005),
                "{flow}: the gas oil at {share:?}"
            );
        }
    }
    assert_eq!(zero_at, Some(820), "{flow}: the tick it reached zero");
    assert_eq!(declined, 0, "{flow}: declined ticks");
    let first_cut = first_cut.expect("the gas oil is cut back");
    assert!(
        (721..=740).contains(&first_cut),
        "{flow}: first cut on tick {first_cut}"
    );
    assert!(
        (0.005..0.010).contains(&least),
        "{flow}: the gas oil down to {least}"
    );
    let s = engine.snapshot();
    assert_eq!(reboiler_mw(&s), 0.0);
    assert!(node(&s, "column").column_reboiler_declined.is_none());
}

#[test]
fn slow_the_cold_plant_starts_up_newton() {
    the_start_up("newton");
}

#[test]
fn slow_the_cold_plant_starts_up_simple() {
    the_start_up("simple");
}
