//! M74.4's measurement driver: M73.4's (`../m73_4/m73_4_grid.rs`) on the
//! holdup plant `crude_unit_holdup.toml` (DESIGN §79.7), the tray plant's keys
//! and a reboiler key: `reb` (MW, held) and `free`. Each run's column holds
//! liquid, so beside the end reading it records, every 10 ticks after the last
//! move, the drum's and the base's levels (their least and most) and the ticks
//! either sat at a physical limit (dry or full, §79.5); and at each of
//! `CHECKS` (ticks after the last move, comma-separated) the products' flows
//! and the levels, to see when a run settles.
//!
//! Put this file in `crates/godot-ext/tests/`, register it in `suite.rs`,
//! build the release test binary, run `m74_4_grid --nocapture --test-threads 1`
//! (`run.sh`). Env: WARM (ticks before the first move, default 1000), GAP
//! (between moves, 3000), RUN (after the last move, 8000), JOBS (4), CHECKS,
//! SPECS or SPECS_FILE (runs separated by `;`, moves by `>`, each move
//! `key=value` pairs joined by `+`, `@N` the ticks to the next). Keys:
//! `furnace` MW, `reflux`, `flow` (a share of the file's loop flows), `ret`
//! (K from the file's returns), `reb` MW held, `free` (any value).
//!
//! Each run prints `RESULT <spec>: RUNS <reading>` or `RESULT <spec>: FAIL at
//! t=… (… after the last move; last sent …): reason`, and its wall time on
//! stderr.
use refinery_godot_ext::bridge::Session;
use serde_json::Value;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

const LOOP_FLOWS: [f64; 3] = [85.0, 95.0, 65.0];
const LOOP_RETURNS_C: [f64; 3] = [150.0, 215.0, 270.0];

fn env_u(k: &str, d: i64) -> i64 {
    std::env::var(k).ok().map_or(d, |v| v.parse().unwrap())
}

fn edge_flow(s: &Value, id: i64) -> f64 {
    s["edges"].as_array().unwrap().iter().find(|e| e["id"].as_i64() == Some(id)).unwrap()["stream"]["mass_flow"]
        .as_f64()
        .unwrap()
}

struct Ids {
    column: i64,
    furnace: i64,
    feed: i64,
    distillate: i64,
    bottoms: i64,
}

fn holdup(s: &Value, ids: &Ids) -> Value {
    s["nodes"][ids.column as usize]["column_operation"]["holdup"].clone()
}

fn reading(s: &Value, ids: &Ids) -> String {
    let col = &s["nodes"][ids.column as usize];
    let op = &col["column_operation"];
    let h = &op["holdup"];
    let stage_list = |key: &str| -> String {
        op[key].as_array().map_or(String::from("-"), |l| {
            l.iter().map(|x| x.as_i64().unwrap().to_string()).collect::<Vec<_>>().join(",")
        })
    };
    let loops: Vec<String> = op["loops"]
        .as_array()
        .map(|ls| {
            ls.iter()
                .map(|l| {
                    format!(
                        "{:.1}MW{}",
                        l["duty_w"].as_f64().unwrap() / 1e6,
                        if l["at_capacity"].as_bool() == Some(true) { "@cap" } else { "" }
                    )
                })
                .collect()
        })
        .unwrap_or_default();
    format!(
        "held {} drum {:.3} {} base {:.3} {} | D {:.2} B {:.2} feed {:.2} kg/s | flood {} weep {} | loops {} R {:.3} coil {:.1} cond {:.2} reb {:.2} MW",
        h["reboiler_held"],
        h["drum_level"].as_f64().unwrap(),
        h["drum_limit"].as_str().unwrap(),
        h["base_level"].as_f64().unwrap(),
        h["base_limit"].as_str().unwrap(),
        edge_flow(s, ids.distillate),
        edge_flow(s, ids.bottoms),
        edge_flow(s, ids.feed),
        stage_list("trays_flooding"),
        stage_list("trays_weeping"),
        loops.join("/"),
        op["reflux_ratio"].as_f64().unwrap(),
        s["nodes"][ids.furnace as usize]["kind"]["coil"]["temperature"].as_f64().unwrap() - 273.15,
        col["column_duty"]["condenser_w"].as_f64().unwrap() / 1e6,
        col["column_duty"]["reboiler_w"].as_f64().unwrap() / 1e6,
    )
}

fn draw_edge(session: &Session, s: &Value, tank: &str) -> i64 {
    let tank = session.node_id(tank);
    s["edges"].as_array().unwrap().iter().find(|e| e["to"].as_i64() == Some(tank)).unwrap()["id"].as_i64().unwrap()
}

fn one_run(src: &str, spec: &str, warm: i64, gap: i64, run: i64, checks: &[i64]) -> String {
    let mut session = Session::default();
    assert_eq!(session.load_scenario(src), "null");
    let s0: Value = serde_json::from_str(&session.snapshot_json()).unwrap();
    let ids = Ids {
        column: session.node_id("column"),
        furnace: session.node_id("furnace"),
        feed: session.edge_id("transfer_line"),
        distillate: draw_edge(&session, &s0, "naphtha_tank"),
        bottoms: draw_edge(&session, &s0, "residue_tank"),
    };
    let column = ids.column;
    let furnace = ids.furnace;
    let mut moves: Vec<(i64, &str)> = Vec::new();
    let mut at = warm;
    for mv in spec.split('>') {
        let (body, wait) = mv.split_once('@').map_or((mv, gap), |(b, w)| (b, w.parse().unwrap()));
        moves.push((at, body));
        at += wait;
    }
    let last_move = moves.last().unwrap().0;
    let total = last_move + run;
    let (mut lo, mut hi) = ([f64::MAX; 2], [f64::MIN; 2]);
    let mut at_limit = [0u32; 2];
    let mut check_text = Vec::new();
    for t in 1..=total {
        if let Some(&(_, body)) = moves.iter().find(|(m, _)| *m == t - 1) {
            for kv in body.split('+') {
                let (key, value) = kv.split_once('=').unwrap();
                let v: f64 = value.parse().unwrap();
                let cmds: Vec<String> = match key {
                    "furnace" => vec![format!(r#"{{"cmd":"set_furnace_duty","node":{furnace},"duty":{:?}}}"#, v * 1e6)],
                    "reflux" => vec![format!(r#"{{"cmd":"set_reflux_ratio","node":{column},"reflux_ratio":{v:?}}}"#)],
                    "reb" => vec![format!(r#"{{"cmd":"set_reboiler_duty","node":{column},"duty":{:?}}}"#, v * 1e6)],
                    "free" => vec![format!(r#"{{"cmd":"set_reboiler_duty","node":{column},"duty":null}}"#)],
                    "flow" => LOOP_FLOWS
                        .iter()
                        .enumerate()
                        .map(|(i, f)| format!(r#"{{"cmd":"set_loop_flow","flow":{:?},"node":{column},"pumparound_loop":{i}}}"#, f * v))
                        .collect(),
                    "ret" => LOOP_RETURNS_C
                        .iter()
                        .enumerate()
                        .map(|(i, c)| format!(r#"{{"cmd":"set_loop_return_temperature","node":{column},"pumparound_loop":{i},"temperature":{:?}}}"#, c + 273.15 + v))
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
            let reached = moves.iter().filter(|(m, _)| *m < t).last().map_or("nothing yet", |m| m.1);
            let reason: String = out.chars().take(600).collect();
            return format!("RESULT {spec}: FAIL at t={t} ({after_last} after the last move; last sent {reached}): {reason}");
        }
        let after = t - last_move;
        if after > 0 && (after % 10 == 0 || checks.contains(&after)) {
            let s: Value = serde_json::from_str(&session.snapshot_json()).unwrap();
            let h = holdup(&s, &ids);
            for (i, (lv, lim)) in [("drum_level", "drum_limit"), ("base_level", "base_limit")].iter().enumerate() {
                let level = h[lv].as_f64().unwrap();
                lo[i] = lo[i].min(level);
                hi[i] = hi[i].max(level);
                if h[lim].as_str() != Some("normal") {
                    at_limit[i] += 10;
                }
            }
            if checks.contains(&after) {
                check_text.push(format!(
                    "@{after}: D {:.3} B {:.3} drum {:.3} base {:.3} reb {:.2}",
                    edge_flow(&s, ids.distillate),
                    edge_flow(&s, ids.bottoms),
                    h["drum_level"].as_f64().unwrap(),
                    h["base_level"].as_f64().unwrap(),
                    s["nodes"][column as usize]["column_duty"]["reboiler_w"].as_f64().unwrap() / 1e6
                ));
            }
        }
    }
    let s: Value = serde_json::from_str(&session.snapshot_json()).unwrap();
    format!(
        "RESULT {spec}: RUNS {} | drum {:.3}-{:.3} limit {} ticks, base {:.3}-{:.3} limit {} ticks | {}",
        reading(&s, &ids),
        lo[0],
        hi[0],
        at_limit[0],
        lo[1],
        hi[1],
        at_limit[1],
        check_text.join(" ; ")
    )
}

#[test]
fn m74_4_grid() {
    let Ok(_) = std::env::var("M74_4") else {
        return;
    };
    let src = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/crude_unit_holdup.toml"),
    )
    .unwrap();
    let specs_text = match std::env::var("SPECS_FILE") {
        Ok(path) => std::fs::read_to_string(path).unwrap(),
        Err(_) => std::env::var("SPECS").unwrap(),
    };
    let specs: Vec<&str> = specs_text.split(';').map(str::trim).filter(|s| !s.is_empty()).collect();
    let (warm, gap, run) = (env_u("WARM", 1000), env_u("GAP", 3000), env_u("RUN", 8000));
    let checks: Vec<i64> = std::env::var("CHECKS")
        .ok()
        .map(|c| c.split(',').map(|x| x.parse().unwrap()).collect())
        .unwrap_or_default();
    let jobs = env_u("JOBS", 4) as usize;
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
                let line = one_run(&src, specs[i], warm, gap, run, &checks);
                eprintln!("[{:.1} s] {line}", start.elapsed().as_secs_f64());
                results.lock().unwrap()[i] = line;
            });
        }
    });
    println!("== holdup WARM {warm} GAP {gap} RUN {run}");
    for line in results.into_inner().unwrap() {
        println!("{line}");
    }
}
