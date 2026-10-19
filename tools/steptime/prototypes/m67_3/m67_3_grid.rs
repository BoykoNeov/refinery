//! M67.3's measurement driver: the column screen's keys as a measured region
//! (docs/DESIGN.md §72.3). Kept in `tools/steptime/prototypes/m67_3/`; every
//! number in §72.8 came from it.
//!
//! It drives a plant through the same `Session` the gdext binding forwards to,
//! with the scene's own command text, on the shipped engine. Put this file in
//! `crates/godot-ext/tests/`, register it in `suite.rs`, and run the release
//! test binary with `m67_3_grid --nocapture --test-threads 1` and, as env vars:
//! PLANT (`crude`, `stripped`, `cold`), WARM (ticks before the first move,
//! default 1000), GAP (ticks between moves, default 3000), RUN (ticks after
//! the last move, default 6000), JOBS (runs in parallel, default 8) and SPECS
//! (or SPECS_FILE, a file holding them):
//! runs separated by `;`, each a sequence of moves separated by `>`, each move
//! `key=value` pairs joined by `+` and sent on one tick, `@N` after a move the
//! ticks to the next. Keys:
//! - `furnace` MW, `reflux`, `steam` (the valve's opening);
//! - `pa`: on the crude and cold plants a share of the file's pumparound
//!   duties, as the screen's keys send it; on the stripped plant MW on every
//!   pumparound;
//! - `reb` (MW, held), `free`, and `walk=step`: the screen's start-up walk
//!   (`column.gd` WALKS), the reboiler held 10 ticks apart at what it runs at
//!   less `step` MW, for 1 890 ticks.
//!
//! Each run prints `RESULT <spec>: RUNS <reading> | cut <draw:share …>` or
//! `RESULT <spec>: FAIL at t=… (… after the last move; last sent …): reason`.
use refinery_godot_ext::bridge::Session;
use serde_json::Value;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

fn env_u(k: &str, d: i64) -> i64 {
    std::env::var(k).ok().map_or(d, |v| v.parse().unwrap())
}

struct Plant {
    file: &'static str,
    /// The file's pumparound duties [MW]; empty where the keys send MW.
    design_pa_mw: &'static [f64],
    pa_count: usize,
    steam: bool,
}

fn plant(name: &str) -> Plant {
    match name {
        "crude" => Plant { file: "crude_unit.toml", design_pa_mw: &[15.0, 15.0, 10.0], pa_count: 3, steam: false },
        "stripped" => Plant { file: "crude_unit_stripped.toml", design_pa_mw: &[], pa_count: 3, steam: true },
        "cold" => Plant { file: "crude_unit_cold.toml", design_pa_mw: &[15.0, 15.0, 10.0], pa_count: 3, steam: true },
        _ => panic!("PLANT {name}"),
    }
}

fn reading(session: &Session, column: i64, furnace: i64, feed: i64) -> String {
    let s: Value = serde_json::from_str(&session.snapshot_json()).unwrap();
    let col = &s["nodes"][column as usize];
    let op = &col["column_operation"];
    let stages = op["stage_temperatures_k"].as_array().unwrap();
    let feed = s["edges"].as_array().unwrap().iter().find(|e| e["id"].as_i64() == Some(feed)).unwrap()["stream"].clone();
    let declined = if col["column_reboiler_declined"].is_null() { "" } else { "DECLINED " };
    let cut: Vec<String> = col["column_draws_cut_back"]
        .as_array()
        .map(|cuts| {
            cuts.iter()
                .map(|c| format!("{}:{:.3}", c["draw"].as_i64().unwrap(), c["of_declared"].as_f64().unwrap()))
                .collect()
        })
        .unwrap_or_default();
    format!(
        "{declined}R {:.3} flash {:.3} coil {:.1} feed {:.1} C cond {:.2} reb {:.2} MW bottom {:.1} C | cut {}",
        op["reflux_ratio"].as_f64().unwrap(),
        op["feed_vapour_fraction"].as_f64().unwrap(),
        s["nodes"][furnace as usize]["kind"]["coil"]["temperature"].as_f64().unwrap() - 273.15,
        feed["temperature"].as_f64().unwrap() - 273.15,
        col["column_duty"]["condenser_w"].as_f64().unwrap() / 1e6,
        col["column_duty"]["reboiler_w"].as_f64().unwrap() / 1e6,
        stages.last().unwrap().as_f64().unwrap() - 273.15,
        cut.join(" ")
    )
}

fn one_run(plant: &Plant, src: &str, spec: &str, warm: i64, gap: i64, run: i64) -> String {
    let mut session = Session::default();
    assert_eq!(session.load_scenario(src), "null");
    let column = session.node_id("column");
    let furnace = session.node_id("furnace");
    let feed = session.edge_id("transfer_line");
    let valve = if plant.steam { session.node_id("steam_valve") } else { -1 };
    let mut moves: Vec<(i64, &str)> = Vec::new();
    let mut at = warm;
    for mv in spec.split('>') {
        let (body, wait) = mv.split_once('@').map_or((mv, gap), |(b, w)| (b, w.parse().unwrap()));
        moves.push((at, body));
        at += wait;
    }
    let last_move = moves.last().unwrap().0;
    let total = last_move + run;
    let mut walk: Option<(i64, f64)> = None;
    for t in 1..=total {
        if let Some((from, step)) = walk {
            let since = t - 1 - from;
            if since > 0 && since <= 1890 && since % 10 == 0 {
                let s: Value = serde_json::from_str(&session.snapshot_json()).unwrap();
                let running = s["nodes"][column as usize]["column_duty"]["reboiler_w"].as_f64().unwrap() / 1e6;
                if running > 0.0 {
                    let target = (running - step).max(0.0);
                    let cmd = format!(r#"{{"cmd":"set_reboiler_duty","node":{column},"duty":{:?}}}"#, target * 1e6);
                    let out = session.apply_command_json(&cmd);
                    if out != "null" {
                        return format!("RESULT {spec}: COMMAND REFUSED at t={t}: {out}");
                    }
                }
            }
        }
        if let Some(&(_, body)) = moves.iter().find(|(m, _)| *m == t - 1) {
            for kv in body.split('+') {
                let (key, value) = kv.split_once('=').unwrap();
                let v: f64 = value.parse().unwrap();
                let cmds: Vec<String> = match key {
                    "furnace" => vec![format!(r#"{{"cmd":"set_furnace_duty","node":{furnace},"duty":{:?}}}"#, v * 1e6)],
                    "reflux" => vec![format!(r#"{{"cmd":"set_reflux_ratio","node":{column},"reflux_ratio":{v:?}}}"#)],
                    "steam" => vec![format!(r#"{{"cmd":"set_valve_opening","node":{valve},"opening":{v:?}}}"#)],
                    "reb" => vec![format!(r#"{{"cmd":"set_reboiler_duty","node":{column},"duty":{:?}}}"#, v * 1e6)],
                    "free" => vec![format!(r#"{{"cmd":"set_reboiler_duty","node":{column},"duty":null}}"#)],
                    "walk" => {
                        walk = Some((t - 1, v));
                        vec![]
                    }
                    "pa" => (0..plant.pa_count)
                        .map(|i| {
                            let mw = if plant.design_pa_mw.is_empty() { v } else { plant.design_pa_mw[i] * v };
                            format!(r#"{{"cmd":"set_pumparound_duty","node":{column},"pumparound":{i},"duty":{:?}}}"#, mw * 1e6)
                        })
                        .collect(),
                    _ => panic!("key {key}"),
                };
                for cmd in cmds {
                    let out = session.apply_command_json(&cmd);
                    if out != "null" {
                        return format!("RESULT {spec}: COMMAND REFUSED at t={t}: {out}");
                    }
                }
            }
        }
        let out = session.tick();
        if out != "null" {
            let after_last = t - 1 - last_move;
            let reached = moves.iter().filter(|(m, _)| *m < t).last().unwrap().1;
            let reason: String = out.chars().take(600).collect();
            return format!("RESULT {spec}: FAIL at t={t} ({after_last} after the last move; last sent {reached}): {reason}");
        }
    }
    format!("RESULT {spec}: RUNS {}", reading(&session, column, furnace, feed))
}

#[test]
fn m67_3_grid() {
    let Ok(name) = std::env::var("PLANT") else {
        return;
    };
    let plant = plant(&name);
    let src = std::fs::read_to_string(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios").join(plant.file)).unwrap();
    // SPECS_FILE: the specs in a file (a Windows environment variable holds
    // 32 767 characters, a grid's specs far more).
    let specs_text = match std::env::var("SPECS_FILE") {
        Ok(path) => std::fs::read_to_string(path).unwrap(),
        Err(_) => std::env::var("SPECS").unwrap(),
    };
    let specs: Vec<&str> = specs_text.split(';').map(str::trim).filter(|s| !s.is_empty()).collect();
    let (warm, gap, run) = (env_u("WARM", 1000), env_u("GAP", 3000), env_u("RUN", 6000));
    let jobs = env_u("JOBS", 8) as usize;
    let next = AtomicUsize::new(0);
    let results = Mutex::new(vec![String::new(); specs.len()]);
    std::thread::scope(|scope| {
        for _ in 0..jobs {
            scope.spawn(|| loop {
                let i = next.fetch_add(1, Ordering::SeqCst);
                if i >= specs.len() {
                    break;
                }
                let start = std::time::Instant::now();
                let line = one_run(&plant, &src, specs[i], warm, gap, run);
                eprintln!("[{:.0} s] {line}", start.elapsed().as_secs_f64());
                results.lock().unwrap()[i] = line;
            });
        }
    });
    println!("== {name} WARM {warm} GAP {gap} RUN {run}");
    for line in results.into_inner().unwrap() {
        println!("{line}");
    }
}
