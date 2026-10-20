//! M70.1 timing driver (M70.0's `m70_0_step` without the probe's log): a plant
//! held, its furnace stepped (or COLD=1: the cold start-up), a CSV row per tick.
//!
//! m70drv <scenario.toml> <label> <hold ticks> <furnace MW> <after ticks>   OUT=<dir>

use refinery_core::snapshot::Command;
use refinery_core::units::Watt;
use std::time::Instant;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let (path, label) = (&a[1], &a[2]);
    let hold: u64 = a[3].parse().unwrap();
    let mw: f64 = a[4].parse().unwrap();
    let after: u64 = a[5].parse().unwrap();
    let out = std::env::var("OUT").unwrap_or_else(|_| ".".into());
    let src = std::fs::read_to_string(path).unwrap();
    let file = refinery_scenarios::load_str(&src).expect("parse");
    let mut engine = refinery_scenarios::build_engine(&file).expect("build");
    let furnace = engine.graph.find_node("furnace");
    let column = engine.graph.find_node("column");
    let cold = std::env::var("COLD").is_ok();
    let (furnace_or, column_or) = (furnace, column);
    let mut csv = String::from("tick,wall_us,re_solves,iterations,residual,flashes,questions\n");
    for t in 1..=hold + after {
        let held = std::env::var("HELD").is_ok();
        let furnace = furnace_or.unwrap_or(refinery_core::graph::NodeId(0));
        let column = column_or.unwrap_or(refinery_core::graph::NodeId(0));
        if held {
        } else if cold {
            if t == 10 {
                engine
                    .apply(Command::SetFurnaceDuty { node: furnace, duty: Watt(58.0e6) })
                    .unwrap();
            }
            if t == 500 {
                for pumparound in 0..3 {
                    engine
                        .apply(Command::SetPumparoundDuty { node: column, pumparound, duty: Watt::ZERO })
                        .unwrap();
                }
            }
            if t >= 500 && t % 10 == 0 {
                let s = engine.snapshot();
                let running = s
                    .nodes
                    .iter()
                    .find(|n| n.name == "column")
                    .unwrap()
                    .column_duty
                    .as_ref()
                    .unwrap()
                    .reboiler_w;
                if running > 0.0 {
                    engine
                        .apply(Command::SetReboilerDuty {
                            node: column,
                            duty: Some(Watt((running - 2.5e6).max(0.0))),
                        })
                        .unwrap();
                }
            }
        } else if t == hold + 1 {
            engine
                .apply(Command::SetFurnaceDuty { node: furnace, duty: Watt(mw * 1e6) })
                .unwrap();
        }
        let start = Instant::now();
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        let wall = start.elapsed().as_nanos() as f64 / 1e3;
        let snap = engine.snapshot();
        csv.push_str(&format!(
            "{t},{wall:.1},{},{},{:e},{},{}\n",
            snap.solver.re_solves,
            snap.solver.iterations,
            snap.solver.residual,
            refinery_solvers::line_flash::FLASHES.swap(0, std::sync::atomic::Ordering::Relaxed),
            refinery_solvers::line_flash::QUESTIONS.swap(0, std::sync::atomic::Ordering::Relaxed)
        ));
        if std::env::var("BYINST").is_ok() && t > 1000 && t <= 2500 {
            let by = std::mem::take(&mut *refinery_solvers::line_flash::BY_INSTANCE.lock().unwrap());
            eprintln!("BYINST t={t} {:?}", by.values().collect::<Vec<_>>());
            let misses = std::mem::take(&mut *refinery_solvers::network::MISSES.lock().unwrap());
            {
                use std::io::Write;
                let mut f = std::fs::OpenOptions::new().create(true).append(true).open(format!("{out}/{label}_allmisses.csv")).unwrap();
                for (le, p, x0) in &misses { writeln!(f, "{t},{le:.9e},{p:.12e},{x0:.17e}").unwrap(); }
            }
            let hits = std::mem::take(&mut *refinery_solvers::network::PREV_HITS.lock().unwrap());
            for (le, p) in &hits {
                let near: Vec<String> = snap
                    .nodes
                    .iter()
                    .filter(|n| (n.pressure_pa - p).abs() < 1e-6 * p)
                    .map(|n| n.name.clone())
                    .collect();
                eprintln!("   PREVHIT le={le:.6} K p={p:.3} Pa nodes at that pressure {near:?}");
            }
        } else {
            refinery_solvers::line_flash::BY_INSTANCE.lock().unwrap().clear();
            refinery_solvers::network::PREV_HITS.lock().unwrap().clear();
            refinery_solvers::network::MISSES.lock().unwrap().clear();
        }
        // SNAP=<tick,tick,...>: those ticks' snapshots, one JSON line each.
        if let Ok(ticks) = std::env::var("SNAP") {
            if ticks.split(',').any(|x| x.parse::<u64>().ok() == Some(t)) {
                use std::io::Write;
                let mut f = std::fs::OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(format!("{out}/{label}.jsonl"))
                    .unwrap();
                writeln!(f, "{}", serde_json::to_string(&snap).unwrap()).unwrap();
            }
        }
    }
    std::fs::write(format!("{out}/{label}.csv"), csv).unwrap();
}
