//! M86.2 (docs/DESIGN.md §84.2): a product line that cannot take its share
//! delivers what it can, and on a column that holds no liquid the rest
//! overflows to the draw below — the bottoms by subtraction; a bottoms line
//! that cannot take the rest halts the run, by name
//! (`product_line_reference::a_line_that_cannot_deliver_or_a_cooler_that_would_boil_halts`).
//!
//! Every plant here is an inline fixture: the crude unit with a product line
//! on every draw (`product_line_reference::crude_unit_lines`), and the wet
//! crude unit, each with a product's tank raised so its line needs its pump.

use crate::product_line_reference::{build, crude_unit_lines};
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::traits::STOPPED_DRAW_SHARE;
use refinery_core::Engine;

const WET: &str = include_str!("../../../scenarios/crude_unit_wet.toml");

/// The oil draws of both plants, top down, by their line's first pipe.
const OIL: [&str; 5] = ["naphtha", "kerosene", "diesel", "gas_oil", "residue"];

/// `src` with `product`'s tank 40 m above its cooler, not 15: only its pump
/// lifts the product there, so with the pump stopped its line delivers
/// nothing.
fn needs_pump(src: &str, product: &str) -> String {
    let old = format!(
        "name = \"{product}_rundown\"\nfrom = \"{product}_cooler\"\nto = \"{product}_tank\"\n\
         length_m = 100.0\ndiameter_m = 0.15\nelevation_change_m = 15"
    );
    assert!(src.contains(&old), "the {product} rundown was not found");
    src.replacen(
        &old,
        &old.replace("elevation_change_m = 15", "elevation_change_m = 40"),
        1,
    )
}

fn tick(engine: &mut Engine, ticks: u32) -> Snapshot {
    for t in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
    }
    engine.snapshot()
}

fn set_pump(engine: &mut Engine, product: &str, on: bool) {
    let node = engine
        .graph
        .find_node(&format!("{product}_pump"))
        .expect("a product pump");
    engine.apply(Command::SetPumpOn { node, on }).unwrap();
}

fn flow(s: &Snapshot, name: &str) -> f64 {
    s.edges
        .iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("no pipe '{name}'"))
        .stream
        .mass_flow
        .value()
}

/// The oil the column is fed [kg/s]: its feed less any free water.
fn oil_fed(engine: &Engine, s: &Snapshot) -> f64 {
    let feed = s.edges.iter().find(|e| e.name == "transfer_line").unwrap();
    let water: f64 = (0..engine.slate.len())
        .filter(|&c| engine.slate.get(c).name == "water")
        .map(|c| feed.stream.composition.fractions()[c])
        .sum();
    feed.stream.mass_flow.value() * (1.0 - water)
}

/// Each oil draw's share of the oil fed [-].
fn shares(engine: &Engine, s: &Snapshot) -> Vec<f64> {
    let oil = oil_fed(engine, s);
    OIL.iter()
        .map(|p| flow(s, &format!("{p}_suction")) / oil)
        .collect()
}

/// The column's worst per-component imbalance, a share of its feed [-]: a
/// column that holds no liquid sends out what comes in, every component.
fn worst_imbalance(engine: &Engine, s: &Snapshot) -> f64 {
    let into: Vec<&str> = vec!["transfer_line"];
    let mut out: Vec<String> = OIL.iter().map(|p| format!("{p}_suction")).collect();
    if s.edges.iter().any(|e| e.name == "water_draw") {
        out.push("water_draw".into());
    }
    let feed = flow(s, "transfer_line");
    let mut worst: f64 = 0.0;
    for c in 0..engine.slate.len() {
        let carried = |name: &str| {
            let e = s.edges.iter().find(|e| e.name == name).unwrap();
            e.stream.mass_flow.value() * e.stream.composition.fractions()[c]
        };
        let balance: f64 = into.iter().map(|n| carried(n)).sum::<f64>()
            - out.iter().map(|n| carried(n)).sum::<f64>();
        worst = worst.max(balance.abs() / feed);
    }
    worst
}

/// **A stopped pump sends its draw to the draw below**: the kerosene's line
/// needs its pump; stopped, the kerosene runs at a stopped product's share
/// (a millionth of the feed) and the diesel draw takes the rest of its share,
/// tick after tick — not full one tick and cut the next — every component's
/// books closing at the column; started again, both draws go back to their
/// own shares.
#[test]
fn a_stopped_pump_sends_its_draw_to_the_draw_below() {
    let mut engine = build(&needs_pump(&crude_unit_lines(false), "kerosene")).unwrap();
    let s = tick(&mut engine, 30);
    let before = shares(&engine, &s);
    assert!((before[1] - 0.16).abs() < 1e-9, "{before:?}");
    assert!((before[2] - 0.2).abs() < 1e-9, "{before:?}");

    set_pump(&mut engine, "kerosene", false);
    for t in 0..60 {
        let s = tick(&mut engine, 1);
        let now = shares(&engine, &s);
        assert!(
            now[1] <= STOPPED_DRAW_SHARE * (1.0 + 1e-6),
            "tick {t}: the kerosene still runs: {now:?}"
        );
        assert!(
            (now[2] - (0.36 - now[1])).abs() < 1e-9,
            "tick {t}: the diesel did not take the kerosene: {now:?}"
        );
        assert!((now[0] - 0.22).abs() < 1e-9, "tick {t}: {now:?}");
        assert!((now[3] - 0.14).abs() < 1e-9, "tick {t}: {now:?}");
        assert!(worst_imbalance(&engine, &s) < 1e-9, "tick {t}");
        if t % 20 == 0 {
            println!(
                "M86_2 tick {t}: shares {now:?}, imbalance {:e}",
                worst_imbalance(&engine, &s)
            );
        }
    }
    let separation = engine
        .node_states()
        .column_separation
        .values()
        .next()
        .expect("the column separated");
    let short = &separation.short_draws;
    assert_eq!(short.len(), 1, "{short:?}");
    assert_eq!(short[0].draw, 1, "the kerosene is the column's draw 1");

    set_pump(&mut engine, "kerosene", true);
    let s = tick(&mut engine, 3);
    let after = shares(&engine, &s);
    assert!((after[1] - 0.16).abs() < 1e-9, "not released: {after:?}");
    assert!((after[2] - 0.2).abs() < 1e-9, "{after:?}");
    assert!(engine
        .node_states()
        .column_separation
        .values()
        .all(|separation| separation.short_draws.is_empty()));
}

/// **Two stopped pumps in a row pass their draws down both**: the kerosene's
/// and the diesel's lines need their pumps; both stopped, the gas oil takes
/// all three shares.
#[test]
fn two_stopped_pumps_pass_their_draws_down_both() {
    let src = needs_pump(&needs_pump(&crude_unit_lines(false), "kerosene"), "diesel");
    let mut engine = build(&src).unwrap();
    tick(&mut engine, 30);
    set_pump(&mut engine, "kerosene", false);
    set_pump(&mut engine, "diesel", false);
    for t in 0..30 {
        let s = tick(&mut engine, 1);
        let now = shares(&engine, &s);
        assert!(
            now[1] <= STOPPED_DRAW_SHARE * (1.0 + 1e-6),
            "tick {t}: {now:?}"
        );
        assert!(
            now[2] <= STOPPED_DRAW_SHARE * (1.0 + 1e-6),
            "tick {t}: {now:?}"
        );
        assert!(
            (now[3] - (0.5 - now[1] - now[2])).abs() < 1e-9,
            "tick {t}: the gas oil did not take both: {now:?}"
        );
        assert!(worst_imbalance(&engine, &s) < 1e-9, "tick {t}");
    }
}

/// **On the wet crude unit, past its water draw**: the column's draw list
/// carries the condenser's water second, which the column model's plan of
/// oil draws leaves out — the kerosene is the column's draw 2 and the plan's
/// 1. Stopped, its share goes to the diesel and the water draw runs on.
#[test]
fn on_the_wet_crude_unit_the_water_draw_is_passed_over() {
    let mut engine = build(&needs_pump(WET, "kerosene")).unwrap();
    let s = tick(&mut engine, 30);
    let water_before = flow(&s, "water_draw");
    assert!(water_before > 0.0);
    set_pump(&mut engine, "kerosene", false);
    let s = tick(&mut engine, 20);
    let now = shares(&engine, &s);
    assert!(now[1] <= STOPPED_DRAW_SHARE * (1.0 + 1e-6), "{now:?}");
    assert!((now[2] - (0.36 - now[1])).abs() < 1e-9, "{now:?}");
    assert!(worst_imbalance(&engine, &s) < 1e-9);
    assert!(flow(&s, "water_draw") > 0.5 * water_before);
    let separation = engine
        .node_states()
        .column_separation
        .values()
        .next()
        .unwrap();
    assert_eq!(separation.short_draws.len(), 1);
    assert_eq!(separation.short_draws[0].draw, 2, "the column's own index");
}

/// `src` with `product`'s rundown `diameter` [m] wide, not 0.15.
fn narrowed(src: &str, product: &str, diameter: f64) -> String {
    let old = format!(
        "name = \"{product}_rundown\"\nfrom = \"{product}_cooler\"\nto = \"{product}_tank\"\n\
         length_m = 100.0\ndiameter_m = 0.15"
    );
    assert!(src.contains(&old), "the {product} rundown was not found");
    src.replacen(
        &old,
        &old.replace("diameter_m = 0.15", &format!("diameter_m = {diameter}")),
        1,
    )
}

/// **A line that carries part of its share holds it, tick after tick**: the
/// kerosene's rundown narrowed until, its pump running, it carries only part
/// of the kerosene's share. The draw runs at what the line carries, the
/// diesel takes the rest, and the next tick does not try the full share again
/// (it marches at what the column wanted, `Separation::short_draws`): the
/// kerosene stands at one flow, not full one tick and cut the next.
#[test]
fn a_line_that_carries_part_holds_it() {
    let mut engine = build(&narrowed(&crude_unit_lines(false), "kerosene", 0.06)).unwrap();
    let mut last: Option<f64> = None;
    for t in 0..80 {
        let s = tick(&mut engine, 1);
        if t < 10 {
            continue;
        }
        let now = shares(&engine, &s);
        assert!(
            now[1] > 0.01 && now[1] < 0.16 - 1e-3,
            "tick {t}: the narrowed line should carry part of 0.16: {now:?}"
        );
        assert!((now[2] - (0.36 - now[1])).abs() < 1e-9, "tick {t}: {now:?}");
        assert!(worst_imbalance(&engine, &s) < 1e-9, "tick {t}");
        if let Some(last) = last {
            assert!(
                (now[1] - last).abs() < 1e-3 * last,
                "tick {t}: the kerosene jumped {last} -> {}",
                now[1]
            );
        }
        last = Some(now[1]);
    }
    println!(
        "M86_2 narrowed: the kerosene runs at {:?} of the oil fed",
        last
    );
    // Stopped, the line carries less; started again it carries more — but
    // still short of the share, so the first tick after the start is short
    // too, never one at the full share.
    set_pump(&mut engine, "kerosene", false);
    let s = tick(&mut engine, 10);
    let stopped = shares(&engine, &s)[1];
    assert!(stopped < 0.5 * last.unwrap(), "stopped: {stopped}");
    set_pump(&mut engine, "kerosene", true);
    for t in 0..10 {
        let s = tick(&mut engine, 1);
        let now = shares(&engine, &s);
        assert!(
            now[1] < 0.16 - 1e-3,
            "tick {t} after the start: the kerosene tried its full share: {now:?}"
        );
    }
}

/// **A distillate line that carries part of its share sends the rest down**:
/// the naphtha's rundown narrowed, the distillate runs at what its line
/// carries and the rest goes down the column. The kerosene's tray takes what
/// it can at a lower reflux — measured 0.252 of the oil fed, not the whole
/// 0.38 — and the tray cut-back (§72.1) sends the rest to the bottoms.
#[test]
fn a_distillate_line_that_carries_part_sends_the_rest_down() {
    let mut engine = build(&narrowed(&crude_unit_lines(false), "naphtha", 0.06)).unwrap();
    for t in 0..30 {
        let s = tick(&mut engine, 1);
        // Its first tick marches the line at no flow, which it carries.
        if t < 3 {
            continue;
        }
        let now = shares(&engine, &s);
        assert!(now[0] > 0.01 && now[0] < 0.22 - 1e-3, "tick {t}: {now:?}");
        assert!(
            now[1] > 0.16 + 1e-3,
            "tick {t}: the kerosene took none: {now:?}"
        );
        assert!(
            (now.iter().sum::<f64>() - 1.0).abs() < 1e-9,
            "tick {t}: {now:?}"
        );
        assert!(worst_imbalance(&engine, &s) < 1e-9, "tick {t}");
    }
}

/// **A stopped distillate pump halts the run, by name**: a column that holds
/// no liquid cannot run with no distillate at a set reflux ratio — the
/// reflux goes with it — and the refusal names the line that stopped it.
#[test]
fn a_stopped_distillate_pump_halts_naming_it() {
    let mut engine = build(&needs_pump(&crude_unit_lines(false), "naphtha")).unwrap();
    tick(&mut engine, 30);
    set_pump(&mut engine, "naphtha", false);
    let err = (0..3)
        .find_map(|_| engine.tick().err())
        .expect("a column with its distillate stopped cannot run");
    let text = err.to_string();
    assert!(
        text.contains("cannot run while its draw line") && text.contains("naphtha_pump"),
        "{text}"
    );
}
