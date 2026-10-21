//! M73.2.0 PROBE driver (not a gate): the loop plant with M73.1's five tray
//! sections inline, each stage at its own pressure by `refinery_solvers::
//! probe`. Every run writes JSON lines to `$M73_OUT` (default
//! `W:\temp\claude\m73_2_0\out`). Run with `--ignored --nocapture`.

use std::io::Write;
use std::time::Instant;

use refinery_core::engine::Engine;
use refinery_core::snapshot::{ColumnOperation, Command, Snapshot};
use refinery_scenarios::{build_engine, load_str};
use refinery_solvers::probe::{self, Config, Mode, PassStat, Rule};
use serde_json::json;

const PLANT: &str = include_str!("../../../scenarios/crude_unit_loops.toml");
const ANCHOR: &str = "reflux_ratio = 4.0\nloops = [";
const SECTIONS: [(u32, u32, f64); 5] = [
    (1, 2, 6.1),
    (3, 5, 7.4),
    (6, 8, 8.3),
    (9, 10, 8.8),
    (11, 12, 8.3),
];

fn fixture() -> String {
    let lines: String = SECTIONS
        .iter()
        .map(|(from, to, d)| {
            format!(
                "    {{ from = {from}, to = {to}, diameter_m = {d}, spacing_m = 0.61, \
                 weir_height_m = 0.05, hole_diameter_m = 0.005, hole_area_share = 0.10, \
                 downcomer_share = 0.12, clearance_m = 0.04, passes = 4 }},\n"
            )
        })
        .collect();
    PLANT
        .replace("\r\n", "\n")
        .replace(ANCHOR, &format!("reflux_ratio = 4.0\ntrays = [\n{lines}]\nloops = ["))
}

fn out_dir() -> std::path::PathBuf {
    let dir = std::env::var("M73_OUT").unwrap_or_else(|_| r"W:\temp\claude\m73_2_0\out".into());
    std::fs::create_dir_all(&dir).unwrap();
    dir.into()
}

fn engine() -> Engine {
    build_engine(&load_str(&fixture()).unwrap()).unwrap()
}

fn operation(snapshot: &Snapshot) -> Option<ColumnOperation> {
    snapshot
        .nodes
        .iter()
        .find(|n| n.name == "column")
        .and_then(|n| n.column_operation.clone())
}

/// Each product's mole fraction of its own cut(s).
fn purities(engine: &Engine, snapshot: &Snapshot) -> Vec<f64> {
    [
        ("naphtha_draw", vec![0, 1]),
        ("kerosene_draw", vec![2]),
        ("diesel_draw", vec![3]),
        ("gas_oil_draw", vec![4]),
        ("residue_draw", vec![5]),
    ]
    .iter()
    .map(|(edge, cuts)| {
        let moles: Vec<f64> = snapshot
            .edges
            .iter()
            .find(|e| e.name == *edge)
            .unwrap()
            .stream
            .composition
            .fractions()
            .iter()
            .enumerate()
            .map(|(c, w)| w / engine.slate.get(c).molar_mass.value())
            .collect();
        cuts.iter().map(|c| moles[*c]).sum::<f64>() / moles.iter().sum::<f64>()
    })
    .collect()
}

fn mode_of(name: &str) -> Config {
    let (mode, rule) = match name {
        "off" => (Mode::Off, Rule::Raw),
        "lag_raw" => (Mode::Lag, Rule::Raw),
        "lag_cap" => (Mode::Lag, Rule::Cap),
        "solve_raw" => (Mode::Solve, Rule::Raw),
        "solve_cap" => (Mode::Solve, Rule::Cap),
        other => panic!("{other}"),
    };
    Config {
        mode,
        rule,
        relax: std::env::var("M73_RELAX").map_or(1.0, |r| r.parse().unwrap()),
    }
}

/// One tick, its passes' statistics and the row it writes.
fn tick_row(engine: &mut Engine, tick: u32, label: &str) -> Result<serde_json::Value, String> {
    let start = Instant::now();
    let result = engine.tick();
    let wall = start.elapsed().as_secs_f64();
    probe::end_tick();
    let stats: Vec<PassStat> = probe::take_stats();
    if let Err(e) = result {
        return Err(format!("tick {tick}: {e}"));
    }
    let snapshot = engine.snapshot();
    let op = operation(&snapshot).ok_or("no column operation")?;
    let last = stats.last().cloned().unwrap_or_default();
    let jet: Vec<u32> = op
        .trays
        .iter()
        .filter(|t| t.flood_fraction > 1.0)
        .map(|t| t.stage)
        .collect();
    let downcomer: Vec<u32> = op
        .trays
        .iter()
        .filter(|t| t.downcomer_backup > 1.0)
        .map(|t| t.stage)
        .collect();
    Ok(json!({
        "label": label,
        "tick": tick,
        "wall_s": wall,
        "passes": stats.len(),
        "solves": stats.iter().map(|s| s.solves).sum::<u32>(),
        "used_total_pa": last.used.iter().sum::<f64>(),
        "raw_total_pa": last.raw.iter().sum::<f64>(),
        "capped_total_pa": last.capped.iter().sum::<f64>(),
        "used": last.used,
        "raw": last.raw,
        "capped": last.capped,
        "sum_at_stage": stats.iter().map(|s| s.summation_at_stage).fold(0.0, f64::max),
        "sum_at_top": stats.iter().map(|s| s.summation_at_top).fold(0.0, f64::max),
        "change": last.change,
        "reflux": op.reflux_ratio,
        "feed_vf": op.feed_vapour_fraction,
        "temps_k": op.stage_temperatures_k,
        "flood": op.trays.iter().map(|t| t.flood_fraction).collect::<Vec<_>>(),
        "weep": op.trays.iter().map(|t| t.weep_margin).collect::<Vec<_>>(),
        "backup": op.trays.iter().map(|t| t.downcomer_backup).collect::<Vec<_>>(),
        "jet_flooding": jet,
        "downcomer_flooding": downcomer,
        "weeping": op.trays_weeping,
        "purity": purities(engine, &snapshot),
        "duty": snapshot.nodes.iter().find(|n| n.name == "column").and_then(|n| n.column_duty.clone()),
    }))
}

fn write_rows(file: &str, rows: &[serde_json::Value]) {
    let mut f = std::fs::File::create(out_dir().join(file)).unwrap();
    for row in rows {
        writeln!(f, "{row}").unwrap();
    }
}

/// A run from cold at the file's settings, `ticks` long, every tick a row.
fn cold_run(mode: &str, ticks: u32) -> Vec<serde_json::Value> {
    probe::set_config(mode_of(mode));
    let mut engine = engine();
    let mut rows = Vec::new();
    for tick in 0..ticks {
        match tick_row(&mut engine, tick, mode) {
            Ok(row) => rows.push(row),
            Err(e) => {
                rows.push(json!({ "label": mode, "error": e }));
                break;
            }
        }
    }
    rows
}

/// E1: from cold at the file's settings, 600 ticks, each mode, one thread —
/// the wall column is the cost.
#[test]
#[ignore]
fn m73_2_0_file_settings() {
    let modes = std::env::var("M73_MODES")
        .unwrap_or_else(|_| "off,lag_raw,solve_raw,solve_cap,lag_cap".into());
    let ticks = std::env::var("M73_TICKS").map_or(600, |t| t.parse().unwrap());
    for mode in modes.split(',') {
        let rows = cold_run(mode, ticks);
        let last = rows.last().unwrap();
        println!("{mode}: {} rows, last {}", rows.len(), last);
        write_rows(&format!("file_{mode}.jsonl"), &rows);
    }
}

/// E2: the reflux line. Each setting on a fresh engine: 300 ticks at the
/// file's settings, then the reflux jumped to it, `M73_AFTER` ticks more.
/// Settings run on `M73_THREADS` threads (the wall column then meaningless).
#[test]
#[ignore]
fn m73_2_0_reflux_line() {
    let modes =
        std::env::var("M73_MODES").unwrap_or_else(|_| "off,lag_cap,solve_raw,solve_cap".into());
    let after: u32 = std::env::var("M73_AFTER").map_or(300, |t| t.parse().unwrap());
    let refluxes: Vec<f64> = std::env::var("M73_REFLUX").map_or_else(
        |_| (1..=30).map(|i| f64::from(i) * 0.5).collect(),
        |r| r.split(',').map(|x| x.parse().unwrap()).collect(),
    );
    let threads: usize = std::env::var("M73_THREADS").map_or(14, |t| t.parse().unwrap());
    let mut jobs: Vec<(String, f64)> = Vec::new();
    for mode in modes.split(',') {
        for r in &refluxes {
            jobs.push((mode.to_string(), *r));
        }
    }
    let jobs = std::sync::Arc::new(std::sync::Mutex::new(jobs));
    let results = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let mut handles = Vec::new();
    for _ in 0..threads {
        let jobs = jobs.clone();
        let results = results.clone();
        handles.push(std::thread::spawn(move || loop {
            let Some((mode, reflux)) = jobs.lock().unwrap().pop() else {
                break;
            };
            probe::set_config(mode_of(&mode));
            let mut engine = engine();
            let label = format!("{mode}@{reflux}");
            let mut rows = Vec::new();
            let mut failed = None;
            for tick in 0..(300 + after) {
                if tick == 300 {
                    let node = engine.graph.find_node("column").unwrap();
                    engine
                        .apply(Command::SetRefluxRatio {
                            node,
                            reflux_ratio: reflux,
                        })
                        .unwrap();
                }
                match tick_row(&mut engine, tick, &label) {
                    Ok(row) => {
                        if tick >= 295 {
                            rows.push(row);
                        }
                    }
                    Err(e) => {
                        failed = Some(e);
                        break;
                    }
                }
            }
            if let Some(e) = failed {
                rows.push(json!({ "label": label, "error": e }));
            }
            write_rows(&format!("line_{mode}_{reflux}.jsonl"), &rows);
            results.lock().unwrap().push(label);
        }));
    }
    for h in handles {
        h.join().unwrap();
    }
    println!("{} runs", results.lock().unwrap().len());
}

/// E1b: a cold start straight at the settled profile (no hand-over): the
/// drops a 300-tick run settled at carried into a fresh engine's first tick.
#[test]
#[ignore]
fn m73_2_0_cold_at_profile() {
    for mode in ["solve_raw", "lag_raw"] {
        let rows = cold_run("solve_raw", 300);
        let settled: Vec<f64> = rows.last().unwrap()["used"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect();
        probe::set_config(mode_of(mode));
        probe::carry(settled);
        let mut engine = engine();
        let mut out = Vec::new();
        for tick in 0..5 {
            match tick_row(&mut engine, tick, mode) {
                Ok(row) => out.push(row),
                Err(e) => {
                    out.push(json!({ "label": mode, "error": e }));
                    break;
                }
            }
        }
        for row in &out {
            println!(
                "{mode}: tick {} solves {} dP {} Tbot {} error {}",
                row["tick"], row["solves"], row["used_total_pa"],
                row["temps_k"].as_array().map_or(serde_json::Value::Null, |t| t[t.len() - 1].clone()),
                row["error"]
            );
        }
        write_rows(&format!("cold_at_profile_{mode}.jsonl"), &out);
    }
}
