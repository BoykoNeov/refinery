//! M66.2 — a held reboiler the column declines, on a running plant
//! (docs/DESIGN.md §71.2). The fixture is `scenarios/crude_unit_stripped.toml`
//! with its reboiler FREE, its pumparounds off and its coil part-warmed —
//! the stripped plant's column on its reboiler partway through a start-up,
//! the furnace at its 58 MW — built here rather than shipped. The gates are
//! M66.0's findings 13–15 (`tools/steptime/prototypes/m66_0/FINDINGS.md`):
//! held at zero before the feed can carry it, the column keeps the duty it
//! ran at, says so, and lands on zero by itself once the feed has heated;
//! walked down from its running duty, it follows the feed to zero; neither
//! is ever refused. Since M67 (§72.1) a column held at zero cuts its gas oil
//! back where its tray runs short, so it lands on a cooler feed than M66
//! measured — ~341 °C, not ~362 — and the gates start from a cooler coil.

use refinery_core::graph::NodeId;
use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot};
use refinery_core::units::Watt;
use refinery_core::Engine;

const STRIPPED: &str = include_str!("../../../scenarios/crude_unit_stripped.toml");

/// The stripped crude unit on its reboiler, its coil declared at `coil_c` and
/// its pumparounds at the steamed plant's 15/15/10 MW — a column on its
/// reboiler with none cannot make its cold start (M64.0 finding 5).
fn part_warmed(coil_c: f64) -> String {
    let plant = STRIPPED
        .replacen("reboiler_duty_mw = 0.0\n", "", 1)
        .replacen(
            "coil_temperature_c = 436.88\n",
            &format!("coil_temperature_c = {coil_c}\n"),
            1,
        )
        .replacen(
            "pumparounds = [{ stage = 4, duty_mw = 0.0 }, { stage = 6, duty_mw = 0.0 }, { stage = 7, duty_mw = 0.0 }]",
            "pumparounds = [{ stage = 4, duty_mw = 15.0 }, { stage = 6, duty_mw = 15.0 }, { stage = 7, duty_mw = 10.0 }]",
            1,
        );
    assert!(!plant.contains("reboiler_duty_mw = 0.0\n") && !plant.contains("436.88\n"));
    assert!(plant.contains("duty_mw = 15.0"));
    plant
}

/// The plant started on its pumparounds, then taken off them after tick 1,
/// as a start-up takes them off before the reboiler comes down.
fn started(coil_c: f64) -> Engine {
    let mut engine = build(&part_warmed(coil_c));
    tick(&mut engine);
    let column = column_id(&engine);
    for pumparound in 0..3 {
        engine
            .apply(Command::SetPumparoundDuty {
                node: column,
                pumparound,
                duty: Watt::ZERO,
            })
            .unwrap();
    }
    engine
}

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the plant parses");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant builds: {e}"))
}

fn tick(engine: &mut Engine) {
    engine
        .tick()
        .unwrap_or_else(|e| panic!("tick {}: {e}", engine.snapshot().tick + 1));
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn column_id(engine: &Engine) -> NodeId {
    node(&engine.snapshot(), "column").id
}

fn reboiler_w(s: &Snapshot) -> f64 {
    node(s, "column").column_duty.unwrap().reboiler_w
}

fn feed_c(s: &Snapshot) -> f64 {
    node(s, "furnace").temperature_k - 273.15
}

/// **Held at zero too early, the column keeps its duty until the feed allows,
/// then lands** (finding 13). From a coil at 350 °C the feed reaches the
/// column near 308 °C, under the ~341 °C a column held at zero needs with its
/// gas oil cut back (M67.1). Held at zero on tick 10: every tick until the
/// feed has heated, the reboiler is published at the duty it ran at on tick
/// 9 — exactly — and the decline names the zero asked for and the dry gas-oil
/// tray; then the column is held at zero, no decline, and stays there. Never
/// refused.
#[test]
fn slow_held_at_zero_too_early_the_column_keeps_its_duty_and_lands_later() {
    let mut engine = started(350.0);
    for _ in 1..9 {
        tick(&mut engine);
    }
    let kept = reboiler_w(&engine.snapshot());
    let early = feed_c(&engine.snapshot());
    assert!(kept > 10.0e6 && early < 320.0, "{kept} W on {early} °C");
    let column = column_id(&engine);
    engine
        .apply(Command::SetReboilerDuty {
            node: column,
            duty: Some(Watt::ZERO),
        })
        .unwrap();

    let mut declined_ticks = 0;
    let mut landed = None;
    for t in 10..=1_500u64 {
        tick(&mut engine);
        let s = engine.snapshot();
        let declined = node(&s, "column").column_reboiler_declined.clone();
        match (landed, declined) {
            (None, Some(hold)) => {
                declined_ticks += 1;
                assert_eq!(reboiler_w(&s), kept, "tick {t}: the duty it ran at");
                assert_eq!(hold.asked_w, 0.0);
                assert!(
                    hold.reason.contains("held at 0.000 MW") && hold.reason.contains("stage 7"),
                    "{}",
                    hold.reason
                );
            }
            (None, None) => {
                assert_eq!(reboiler_w(&s), 0.0, "tick {t}");
                landed = Some((t, feed_c(&s)));
            }
            (Some(_), None) => assert_eq!(reboiler_w(&s), 0.0, "tick {t}"),
            (Some((at, _)), Some(_)) => panic!("tick {t}: declined again after landing at {at}"),
        }
    }
    let (at, feed) = landed.expect("the column lands on zero");
    assert!(declined_ticks > 100, "declined only {declined_ticks} ticks");
    assert!(
        (335.0..348.0).contains(&feed),
        "landed at tick {at} on a {feed} °C feed"
    );
}

/// **Walked down from its running duty, the column follows the feed to zero**
/// (findings 14–15). From a coil at 350 °C, every 10 ticks the reboiler is
/// held at the duty it ran at less 2.5 MW, as the screen's key does: the duty
/// never rises, the column declines the steps it cannot take yet (the walk
/// reaches the edge and waits on the feed), and the reboiler reaches zero.
/// Never refused.
#[test]
fn slow_walked_down_from_its_running_duty_the_column_follows_the_feed() {
    let mut engine = started(350.0);
    tick(&mut engine);
    let column = column_id(&engine);
    let mut last = reboiler_w(&engine.snapshot());
    let mut declined_ticks = 0;
    let mut zero_at = None;
    for t in 2..=2_000u64 {
        if t % 10 == 0 && last > 0.0 {
            engine
                .apply(Command::SetReboilerDuty {
                    node: column,
                    duty: Some(Watt((last - 2.5e6).max(0.0))),
                })
                .unwrap();
        }
        tick(&mut engine);
        let s = engine.snapshot();
        let duty = reboiler_w(&s);
        assert!(duty <= last, "tick {t}: {duty} W after {last} W");
        if node(&s, "column").column_reboiler_declined.is_some() {
            declined_ticks += 1;
        }
        if duty == 0.0 && zero_at.is_none() {
            zero_at = Some(t);
        }
        last = duty;
    }
    let at = zero_at.expect("the walk reaches zero");
    assert!(declined_ticks > 100, "declined only {declined_ticks} ticks");
    assert_eq!(last, 0.0, "landed at tick {at}");
}
