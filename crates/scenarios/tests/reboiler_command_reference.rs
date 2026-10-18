//! M66.1 — a column's reboiler held or freed by command (docs/DESIGN.md §71.1),
//! on a running plant. The fixture is `scenarios/crude_unit_stripped.toml`
//! with its reboiler FREE and its pumparounds at the steamed plant's
//! 15/15/10 MW — the stripped plant's column on its reboiler, its coil hot —
//! built here rather than shipped: the shipped files are the regression
//! anchor. The gates are M66.0's findings 2 and 4
//! (`tools/steptime/prototypes/m66_0/FINDINGS.md`): held at its own closure
//! the column runs on as it was, freed again likewise, and held at zero with
//! its pumparounds off it is the stripped plant on the same tick.

use refinery_core::graph::{NodeId, NodeKind};
use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot};
use refinery_core::units::Watt;
use refinery_core::Engine;

const STRIPPED: &str = include_str!("../../../scenarios/crude_unit_stripped.toml");

/// The stripped crude unit with its reboiler free and its pumparounds on.
fn on_its_reboiler() -> String {
    let plant = STRIPPED
        .replacen("reboiler_duty_mw = 0.0\n", "", 1)
        .replacen(
            "pumparounds = [{ stage = 4, duty_mw = 0.0 }, { stage = 6, duty_mw = 0.0 }, { stage = 7, duty_mw = 0.0 }]",
            "pumparounds = [{ stage = 4, duty_mw = 15.0 }, { stage = 6, duty_mw = 15.0 }, { stage = 7, duty_mw = 10.0 }]",
            1,
        );
    assert!(
        !plant.contains(
            "reboiler_duty_mw = 0.0
"
        ) && plant.contains("duty_mw = 15.0")
    );
    plant
}

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the plant parses");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant builds: {e}"))
}

fn ticks(engine: &mut Engine, n: u32) {
    for _ in 0..n {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("tick {}: {e}", engine.snapshot().tick + 1));
    }
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn id(engine: &Engine, name: &str) -> NodeId {
    node(&engine.snapshot(), name).id
}

/// The duty the column's `kind` holds — the command's consequence.
fn held_duty(s: &Snapshot) -> Option<Watt> {
    match &node(s, "column").kind {
        NodeKind::Column {
            cascade: Some(spec),
            ..
        } => spec.reboiler_duty,
        other => panic!("not a cascade column: {other:?}"),
    }
}

fn reboiler_w(s: &Snapshot) -> f64 {
    node(s, "column").column_duty.unwrap().reboiler_w
}

fn reflux(s: &Snapshot) -> f64 {
    node(s, "column")
        .column_operation
        .clone()
        .unwrap()
        .reflux_ratio
}

fn stages(s: &Snapshot) -> Vec<f64> {
    node(s, "column")
        .column_operation
        .clone()
        .unwrap()
        .stage_temperatures_k
}

/// The largest stage-temperature difference between two snapshots [K].
fn stage_gap(a: &Snapshot, b: &Snapshot) -> f64 {
    stages(a)
        .iter()
        .zip(stages(b))
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

/// **Held at its own closure, the column runs on as it was; freed, likewise**
/// (finding 2). Two copies of the plant run 20 ticks; one holds its reboiler
/// at the duty it just published. For three ticks it publishes that duty to
/// the bit and runs where the free copy runs — its reflux, the result now,
/// within 1e-5 of the declared 4.0 the free copy holds, its stages within
/// 1e-4 K. Freed again, it is the free copy on the next tick.
#[test]
fn held_at_its_closure_the_column_runs_on_and_freed_again() {
    let src = on_its_reboiler();
    let mut free = build(&src);
    let mut held = build(&src);
    ticks(&mut free, 20);
    ticks(&mut held, 20);
    let column = id(&held, "column");
    let closure = reboiler_w(&held.snapshot());
    held.apply(Command::SetReboilerDuty {
        node: column,
        duty: Some(Watt(closure)),
    })
    .unwrap();
    assert_eq!(held_duty(&held.snapshot()), Some(Watt(closure)));

    for _ in 0..3 {
        ticks(&mut free, 1);
        ticks(&mut held, 1);
        let (f, h) = (free.snapshot(), held.snapshot());
        assert_eq!(
            reboiler_w(&h),
            closure,
            "the held duty is what it publishes"
        );
        assert_eq!(reflux(&f), 4.0);
        assert!(
            (reflux(&h) - 4.0).abs() < 1.0e-5,
            "held at its closure the reflux is {}",
            reflux(&h)
        );
        assert!(stage_gap(&f, &h) < 1.0e-4, "{} K", stage_gap(&f, &h));
    }

    held.apply(Command::SetReboilerDuty {
        node: column,
        duty: None,
    })
    .unwrap();
    assert_eq!(held_duty(&held.snapshot()), None);
    ticks(&mut free, 1);
    ticks(&mut held, 1);
    let (f, h) = (free.snapshot(), held.snapshot());
    assert_eq!(reflux(&h), 4.0, "freed, the reflux is the declared one");
    assert!(
        (reboiler_w(&h) - reboiler_w(&f)).abs() < 1.0e-6 * reboiler_w(&f),
        "freed {} W against {} W",
        reboiler_w(&h),
        reboiler_w(&f)
    );
    assert!(stage_gap(&f, &h) < 1.0e-6, "{} K", stage_gap(&f, &h));
}

/// **Held at zero with its pumparounds off, on the same tick, it is the
/// stripped plant** (finding 4): from the column on its reboiler, the tick it
/// is handed over lands where `crude_unit_stripped.toml` stands on that tick —
/// the reboiler published as zero, the reflux and every stage the shipped
/// plant's.
#[test]
fn held_at_zero_its_pumparounds_off_it_is_the_stripped_plant() {
    let mut engine = build(&on_its_reboiler());
    let mut stripped = build(STRIPPED);
    ticks(&mut engine, 20);
    ticks(&mut stripped, 21);
    let column = id(&engine, "column");
    assert!(reflux(&engine.snapshot()) == 4.0 && reboiler_w(&engine.snapshot()) > 50.0e6);
    engine
        .apply(Command::SetReboilerDuty {
            node: column,
            duty: Some(Watt::ZERO),
        })
        .unwrap();
    for pumparound in 0..3 {
        engine
            .apply(Command::SetPumparoundDuty {
                node: column,
                pumparound,
                duty: Watt::ZERO,
            })
            .unwrap();
    }
    ticks(&mut engine, 1);
    let (e, s) = (engine.snapshot(), stripped.snapshot());
    assert_eq!(reboiler_w(&e), 0.0);
    assert!(
        (reflux(&e) - reflux(&s)).abs() < 1.0e-6,
        "reflux {} against the stripped plant's {}",
        reflux(&e),
        reflux(&s)
    );
    assert!(stage_gap(&e, &s) < 1.0e-5, "{} K", stage_gap(&e, &s));
}

/// **The command's refusals** (§71.1), each leaving the column as it was: a
/// duty below zero or not finite; a node that is not a column; a cut-point
/// column, which has no reboiler; a column under the `"cascade"` model, which
/// solves no energy balance.
#[test]
fn the_reboiler_command_is_refused_where_nothing_could_hold_it() {
    let mut engine = build(&on_its_reboiler());
    let column = id(&engine, "column");
    for duty in [-1.0, f64::NAN, f64::INFINITY] {
        let e = engine
            .apply(Command::SetReboilerDuty {
                node: column,
                duty: Some(Watt(duty)),
            })
            .unwrap_err()
            .to_string();
        assert!(e.contains("reboiler duty must be finite and >= 0"), "{e}");
        assert_eq!(held_duty(&engine.snapshot()), None);
    }
    let furnace = id(&engine, "furnace");
    let e = engine
        .apply(Command::SetReboilerDuty {
            node: furnace,
            duty: Some(Watt::ZERO),
        })
        .unwrap_err()
        .to_string();
    assert!(e.contains("'furnace') is not a column"), "{e}");

    let mut fcc = build(include_str!("../../../scenarios/fcc_plant.toml"));
    let fractionator = id(&fcc, "fractionator");
    let e = fcc
        .apply(Command::SetReboilerDuty {
            node: fractionator,
            duty: Some(Watt::ZERO),
        })
        .unwrap_err()
        .to_string();
    assert!(e.contains("is a cut-point column"), "{e}");

    let mut cascade = build(include_str!("../../../scenarios/crude_column_cascade.toml"));
    let column = id(&cascade, "column");
    let e = cascade
        .apply(Command::SetReboilerDuty {
            node: column,
            duty: Some(Watt::ZERO),
        })
        .unwrap_err()
        .to_string();
    assert!(
        e.contains("runs under separation \"cascade\"") && e.contains("heat_balance"),
        "{e}"
    );
    assert_eq!(held_duty(&cascade.snapshot()), None);
}
