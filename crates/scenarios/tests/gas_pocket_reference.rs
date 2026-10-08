//! M55.1 — a gas pocket that fills over seconds (docs/DESIGN.md §60.1; ledger
//! row B56; decisions 2–4).
//!
//! The shipped gas-lock plant (`scenarios/pump_gas_lock.toml`). A running pump's
//! pocket fills by `dt` over its fill time while its suction offers 16.5% vapour
//! by volume or more, drains at the same rate while it offers less, takes its
//! share of the push as it goes, and locks the pump when full. These gates hold
//! the pocket's arithmetic, its independence of the timestep, its setting and its
//! refusals, and that no ramp leaves it parked part-full.

use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::units::Kelvin;
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/pump_gas_lock.toml");

const SOLVERS: [&str; 2] = ["newton", "simple"];

/// The demo plant on `solver`, at timestep `dt` [s], with the pump's
/// `gas_fill_time_s` set where `fill_time` is given.
fn plant(solver: &str, dt: f64, fill_time: Option<f64>) -> String {
    let mut src = DEMO
        .replace("flow = \"newton\"", &format!("flow = \"{solver}\""))
        .replace("dt = 0.1", &format!("dt = {dt}"));
    if let Some(seconds) = fill_time {
        src = src.replace(
            "a = 800.0\non = true\n",
            &format!("a = 800.0\non = true\ngas_fill_time_s = {seconds}\n"),
        );
    }
    src
}

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the scenario must parse");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant must build: {e}"))
}

fn tick(engine: &mut Engine, label: &str) -> Snapshot {
    engine
        .tick()
        .unwrap_or_else(|e| panic!("{label}: the tick failed: {e}"));
    let s = engine.snapshot();
    assert!(s.solver.converged, "{label}: the tick did not converge");
    s
}

fn run(engine: &mut Engine, ticks: u32, label: &str) {
    for _ in 0..ticks {
        tick(engine, label);
    }
}

fn pump(engine: &Engine) -> (bool, f64) {
    let id = engine.graph.find_node("feed_pump").unwrap();
    match engine.graph.node(id).kind {
        NodeKind::Pump {
            gas_locked,
            gas_pocket,
            ..
        } => (gas_locked, gas_pocket),
        _ => unreachable!(),
    }
}

fn flow(s: &Snapshot) -> f64 {
    s.edges
        .iter()
        .find(|e| e.name == "suction_line")
        .unwrap()
        .stream
        .mass_flow
        .value()
}

fn warm(engine: &mut Engine, celsius: f64) {
    let node = engine.graph.find_node("rundown_source").unwrap();
    engine
        .apply(Command::SetSourceTemperature {
            node,
            temperature: Kelvin(celsius + 273.15),
        })
        .expect("a flashing plant takes a boiling supply");
}

fn command(engine: &mut Engine, command: Command) -> Result<(), String> {
    engine.apply(command).map_err(|e| e.to_string())
}

/// **No ramp parks the pocket part-full.** The supply warmed from 110 °C in
/// 0.25 °C steps to 121 °C, each held 4 s (longer than the 3 s fill): at the end
/// of every step the pocket is empty or the pump locked, the last two ticks agree,
/// and the pump locks only where its settled suction offers 16.5% or more — from
/// 119.25 °C (19%), not at 119 °C (15.2%) or 118 °C (12.7%), where until M55 a
/// step tick locked it.
#[test]
fn a_ramp_never_leaves_the_pocket_part_full() {
    for solver in SOLVERS {
        let mut engine = build(&plant(solver, 0.1, None));
        warm(&mut engine, 110.0);
        run(&mut engine, 40, solver);
        let mut first_locked = None;
        for step in 0..=44 {
            let celsius = 110.0 + 0.25 * f64::from(step);
            warm(&mut engine, celsius);
            let label = format!("{solver} at {celsius} °C");
            run(&mut engine, 39, &label);
            let before = flow(&engine.snapshot());
            let s = tick(&mut engine, &label);
            let (locked, pocket) = pump(&engine);
            assert!(
                pocket == 0.0 || (locked && pocket == 1.0),
                "{label}: the pocket parked at {pocket}"
            );
            assert!(
                (flow(&s) - before).abs() <= 1e-7 * before.abs().max(1.0),
                "{label}: not settled, {before} then {} kg/s",
                flow(&s)
            );
            if locked && first_locked.is_none() {
                first_locked = Some(celsius);
            }
        }
        let first = first_locked.expect("the ramp reaches the lock");
        assert!(
            (119.25..=120.0).contains(&first),
            "{solver}: first locked at {first} °C"
        );
    }
}

/// **The fill is counted in seconds, not ticks**: the same 2 s surge to 120 °C
/// at a 0.1 s and a 0.05 s tick fills the pocket to the same two thirds, and
/// held there the pump locks at the same time, to within one tick of the
/// coarser step.
#[test]
fn the_pocket_fills_in_seconds_at_any_timestep() {
    for solver in SOLVERS {
        let mut locked_at = Vec::new();
        for dt in [0.1, 0.05] {
            let label = format!("{solver} at dt = {dt}");
            let mut engine = build(&plant(solver, dt, None));
            warm(&mut engine, 110.0);
            run(&mut engine, (4.0 / dt) as u32, &label);
            warm(&mut engine, 120.0);
            run(&mut engine, (2.0 / dt).round() as u32, &label);
            let (locked, pocket) = pump(&engine);
            assert!(!locked, "{label}: locked in 2 s");
            assert!(
                (pocket - 2.0 / 3.0).abs() < 1e-9,
                "{label}: pocket {pocket} after 2 s"
            );
            let mut t = 2.0;
            while !pump(&engine).0 {
                tick(&mut engine, &label);
                t += dt;
                assert!(t < 10.0, "{label}: never locked");
            }
            locked_at.push(t);
        }
        assert!(
            (locked_at[0] - locked_at[1]).abs() <= 0.1 + 1e-9,
            "{solver}: locked at {locked_at:?} s"
        );
        assert!(
            (locked_at[0] - 3.0).abs() <= 0.1 + 1e-9,
            "{solver}: locked at {} s, the fill time is 3 s",
            locked_at[0]
        );
    }
}

/// **`gas_fill_time_s` sets the fill time** of the pump it is declared on: at
/// 1 s a 0.5 s surge fills half the pocket, and held past the lock point the
/// pump locks on the tenth tick.
#[test]
fn the_fill_time_is_the_pumps_own() {
    let mut engine = build(&plant("newton", 0.1, Some(1.0)));
    warm(&mut engine, 110.0);
    run(&mut engine, 20, "1 s");
    warm(&mut engine, 120.0);
    run(&mut engine, 5, "1 s surge");
    let (locked, pocket) = pump(&engine);
    assert!(!locked && (pocket - 0.5).abs() < 1e-9, "{pocket}");
    run(&mut engine, 4, "1 s held");
    assert!(!pump(&engine).0, "locked before the tenth tick");
    run(&mut engine, 1, "1 s held");
    assert!(pump(&engine).0, "not locked on the tenth tick");
}

/// **A part-full pocket takes its share of the push, and gives it back as it
/// drains**: after a 1.5 s surge (half full) the pump on the fall at 110 °C
/// moves less than before it, every tick a little more, and back where it was
/// once empty.
#[test]
fn the_push_comes_back_as_the_pocket_drains() {
    for solver in SOLVERS {
        let mut engine = build(&plant(solver, 0.1, None));
        warm(&mut engine, 110.0);
        run(&mut engine, 40, solver);
        let fall = flow(&engine.snapshot());
        warm(&mut engine, 120.0);
        run(&mut engine, 15, solver);
        assert!((pump(&engine).1 - 0.5).abs() < 1e-9, "{solver}");
        warm(&mut engine, 110.0);
        let mut last = 0.0;
        for t in 1..=15 {
            let s = tick(&mut engine, solver);
            let now = flow(&s);
            let pocket = pump(&engine).1;
            if t < 15 {
                assert!(
                    pocket > 0.0 && now < fall,
                    "{solver} tick {t}: {now} of {fall}"
                );
            }
            if t > 1 {
                assert!(now > last, "{solver} tick {t}: {now} after {last}");
            }
            last = now;
        }
        assert_eq!(pump(&engine).1, 0.0, "{solver}: the pocket did not drain");
        run(&mut engine, 2, solver);
        let back = flow(&engine.snapshot());
        assert!(
            (back - fall).abs() <= 1e-6 * fall,
            "{solver}: {back} against {fall}"
        );
    }
}

/// **The pocket takes its share on a liquid suction too**: half full after a
/// 1.5 s surge, then the supply cooled straight to 100 °C, where the suction
/// offers no vapour at all — the pump moves less than a clean one there until
/// its pocket has drained, and then exactly what it did.
#[test]
fn a_part_full_pocket_takes_its_share_on_a_liquid_suction() {
    for solver in SOLVERS {
        let mut engine = build(&plant(solver, 0.1, None));
        run(&mut engine, 20, solver);
        let liquid = flow(&engine.snapshot());
        warm(&mut engine, 120.0);
        run(&mut engine, 15, solver);
        warm(&mut engine, 100.0);
        let s = tick(&mut engine, solver);
        let pump_node = s.nodes.iter().find(|n| n.name == "feed_pump").unwrap();
        assert_eq!(pump_node.pump_two_phase, None, "{solver}: vapour at 100 °C");
        assert!(
            pump(&engine).1 > 0.0,
            "{solver}: the pocket drained at once"
        );
        assert!(
            flow(&s) < liquid - 0.1,
            "{solver}: {} kg/s with gas in the eye, {liquid} without",
            flow(&s)
        );
        run(&mut engine, 20, solver);
        assert_eq!(pump(&engine).1, 0.0, "{solver}");
        let back = flow(&engine.snapshot());
        assert!(
            (back - liquid).abs() <= 1e-6 * liquid,
            "{solver}: {back} against {liquid}"
        );
    }
}

/// **A stopped pump's pocket holds, and a vent empties it**: stopped half full,
/// it is half full ten seconds later; a vent on it, stopped, is allowed and
/// empties it; a vent on a stopped pump with no gas is refused.
#[test]
fn a_stopped_pocket_holds_until_vented() {
    let mut engine = build(&plant("newton", 0.1, None));
    warm(&mut engine, 110.0);
    run(&mut engine, 20, "newton");
    warm(&mut engine, 120.0);
    run(&mut engine, 15, "surge");
    let id = engine.graph.find_node("feed_pump").unwrap();
    let refused = command(&mut engine, Command::VentPump { node: id })
        .expect_err("a running pump is not vented");
    assert!(refused.contains("still running"), "{refused}");
    command(
        &mut engine,
        Command::SetPumpOn {
            node: id,
            on: false,
        },
    )
    .unwrap();
    warm(&mut engine, 100.0);
    run(&mut engine, 100, "stopped");
    assert!((pump(&engine).1 - 0.5).abs() < 1e-9, "{:?}", pump(&engine));
    command(&mut engine, Command::VentPump { node: id }).expect("gas to vent");
    assert_eq!(pump(&engine), (false, 0.0));
    let refused =
        command(&mut engine, Command::VentPump { node: id }).expect_err("nothing left to vent");
    assert!(refused.contains("holds no gas"), "{refused}");
}

/// **The setting is refused where it would read nothing, and where it is not a
/// time**: on a plant without the line flash, and at zero, below zero and NaN.
#[test]
fn the_fill_time_is_refused_by_name() {
    let no_flash = plant("newton", 0.1, Some(3.0))
        .replace("line_flash = \"equilibrium\"", "")
        .replace("temperature_c = 100.0", "temperature_c = 60.0");
    let file = refinery_scenarios::load_str(&no_flash).expect("parses");
    let err = refinery_scenarios::build_engine(&file)
        .map(|_| ())
        .unwrap_err()
        .to_string();
    assert!(
        err.contains("feed_pump") && err.contains("gas_fill_time_s") && err.contains("line flash"),
        "{err}"
    );
    for bad in ["0.0", "-1.0", "nan"] {
        let src = plant("newton", 0.1, None).replace(
            "a = 800.0\non = true\n",
            &format!("a = 800.0\non = true\ngas_fill_time_s = {bad}\n"),
        );
        let file = refinery_scenarios::load_str(&src).expect("parses");
        let err = refinery_scenarios::build_engine(&file)
            .map(|_| ())
            .unwrap_err()
            .to_string();
        assert!(
            err.contains("feed_pump") && err.contains("positive number of seconds"),
            "{bad}: {err}"
        );
    }
}
