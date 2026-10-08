//! Per-tick wall time through the two boiling plants' stories (M56,
//! docs/DESIGN.md §61.2–61.3).
//!
//! The corpus times a plant held where its file starts it; a frame stutters on
//! its WORST tick, which a calm run never reaches. This plays each story beat
//! by beat — `pump_gas_lock` through the beats `pump_gas_lock_reference.rs`
//! plays, `flashing_rundown` 200 ticks at 115 °C, 200 at 135 °C, 200 back — on
//! both solvers, and reports per tick the fastest of `RUNS` runs (default 5),
//! which filters the machine's own noise: the mean, the 99th percentile, the
//! worst tick and how many ticks exceed the 2.5 ms frame budget (§11, M9.3a).
//!
//! ```text
//! cargo run --release --manifest-path tools/steptime/Cargo.toml -- scenarios
//!   ONLY=pump_gas_lock:simple   one plant and solver
//!   RUNS=1                      runs per story (the fastest is kept per tick)
//!   DUMP=<dir>                  write every tick's time, one file per case
//!   SNAPOUT=<file>              write every tick's snapshot as JSON lines
//!                               (with ONLY; one run)
//! ```
//!
//! To compare two builds, point a copy's path dependencies at a worktree of the
//! other commit and alternate the two in one session: the machine drifts
//! (§11, M9.3a), so read ratios, not milliseconds.

use refinery_core::snapshot::Command;
use refinery_core::units::{Kelvin, Pascal};
use refinery_core::Engine;
use std::time::Instant;

/// One beat of a story: run some ticks, or apply one command.
enum Beat {
    Run(u32),
    Supply(&'static str, f64),
    Destination(f64),
    Pump(bool),
    Vent,
}

/// Frame budget for one tick [ms] (docs/DESIGN.md §11, M9.3a).
const FRAME_BUDGET_MS: f64 = 2.5;

fn build(path: &str, solver: &str) -> Engine {
    let src = std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("{path}: {e}"))
        .replace("flow = \"newton\"", &format!("flow = \"{solver}\""));
    let file = refinery_scenarios::load_str(&src).expect("the scenario must parse");
    refinery_scenarios::build_engine(&file).expect("the plant must build")
}

/// Plays `beats`; the wall time of every tick [ms], and every snapshot as JSON
/// lines when `snapshots` is asked for.
fn story(engine: &mut Engine, beats: &[Beat], snapshots: bool) -> (Vec<f64>, String) {
    let mut times = Vec::new();
    let mut lines = String::new();
    for beat in beats {
        let id = |e: &Engine, n: &str| e.graph.find_node(n).expect("a story's node");
        let command = match *beat {
            Beat::Run(n) => {
                for _ in 0..n {
                    let start = Instant::now();
                    engine.tick().expect("a story's tick");
                    times.push(start.elapsed().as_secs_f64() * 1e3);
                    if snapshots {
                        lines.push_str(&serde_json::to_string(&engine.snapshot()).unwrap());
                        lines.push('\n');
                    }
                }
                continue;
            }
            Beat::Supply(node, celsius) => Command::SetSourceTemperature {
                node: id(engine, node),
                temperature: Kelvin(celsius + 273.15),
            },
            Beat::Destination(bar) => Command::SetReservoirPressure {
                node: id(engine, "unit_feed"),
                pressure: Pascal(bar * 1e5),
            },
            Beat::Pump(on) => Command::SetPumpOn {
                node: id(engine, "feed_pump"),
                on,
            },
            Beat::Vent => Command::VentPump {
                node: id(engine, "feed_pump"),
            },
        };
        // A refusal is part of a story (the vent while running): ignored.
        let _ = engine.apply(command);
    }
    (times, lines)
}

fn main() {
    let dir = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "scenarios".into());
    let only = std::env::var("ONLY").ok();
    let snapout = std::env::var("SNAPOUT").ok();
    let dump = std::env::var("DUMP").ok();
    let runs: usize = match &snapout {
        Some(_) => 1,
        None => std::env::var("RUNS").map_or(5, |r| r.parse().expect("RUNS is a count")),
    };
    use Beat::*;
    let s = "rundown_source";
    let gas_lock = vec![
        Run(20),
        Supply(s, 110.0),
        Run(20),
        Supply(s, 120.0),
        Run(10),
        Supply(s, 110.0),
        Run(20),
        Supply(s, 120.0),
        Run(40),
        Supply(s, 100.0),
        Run(20),
        Vent,
        Pump(false),
        Run(10),
        Vent,
        Pump(true),
        Run(20),
        Destination(3.0),
        Run(20),
        Supply(s, 125.0),
        Run(40),
    ];
    let r = "rundown";
    let rundown = vec![
        Run(200),
        Supply(r, 135.0),
        Run(200),
        Supply(r, 115.0),
        Run(200),
    ];
    for (name, beats) in [("pump_gas_lock", &gas_lock), ("flashing_rundown", &rundown)] {
        for solver in ["simple", "newton"] {
            if only
                .as_ref()
                .is_some_and(|o| *o != format!("{name}:{solver}"))
            {
                continue;
            }
            let path = format!("{dir}/{name}.toml");
            let mut best: Vec<f64> = Vec::new();
            for _ in 0..runs.max(1) {
                let mut engine = build(&path, solver);
                let (times, lines) = story(&mut engine, beats, snapout.is_some());
                if let Some(file) = &snapout {
                    std::fs::write(file, lines).expect("SNAPOUT must be writable");
                }
                if best.is_empty() {
                    best = times;
                } else {
                    for (b, t) in best.iter_mut().zip(times) {
                        *b = b.min(t);
                    }
                }
            }
            if let Some(dir) = &dump {
                let text: String = best.iter().map(|t| format!("{t:.3}\n")).collect();
                std::fs::write(format!("{dir}/steps_{name}_{solver}.txt"), text)
                    .expect("DUMP must be a writable directory");
            }
            // The first tick is a cold start, reported apart.
            let first = best[0];
            let mut rest = best[1..].to_vec();
            let mean = rest.iter().sum::<f64>() / rest.len() as f64;
            let worst_at = rest
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.total_cmp(b.1))
                .map_or(0, |(i, _)| i + 2);
            rest.sort_by(f64::total_cmp);
            let p99 = rest[((rest.len() as f64 * 0.99) as usize).saturating_sub(1)];
            let over = rest.iter().filter(|&&t| t > FRAME_BUDGET_MS).count();
            println!(
                "{name:17} {solver:6} first {first:7.2}  mean {mean:5.2}  p99 {p99:6.2}  worst {:6.2} (tick {worst_at})  over {FRAME_BUDGET_MS} ms {over}/{}",
                rest[rest.len() - 1],
                rest.len()
            );
        }
    }
}
