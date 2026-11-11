//! M74.6.0's probe driver (the start from empty): M74.5.0's, with the key
//! `tr` (1 on, 0 off: the total-reflux start, a probe switch on the
//! ticking thread), each stage's liquid [t] in the trace, and the
//! stages holding nothing counted. Was: M74.5.0's probe driver (stages that stop boiling): M74.4's
//! (`tools/steptime/prototypes/m74_4/`) with PLANT `valved` (the holdup plant
//! with a valve on its crude line, kv 5000, as `column_holdup.rs`'s
//! `fixture_valved`), the key `valve` (its opening), each run's stages that
//! stopped boiling counted (a stage's vapour under 1e-3 mol/s), and TRACE=N
//! printing every N ticks a line per run. Was: M74.4's measurement driver: M73.4's (`tools/steptime/prototypes/m73_4/`)
//! on the holdup plant `crude_unit_holdup.toml` — the tray plant's keys and
//! the reboiler's — with each run's holdup read over the whole run, not only at
//! its end: the least and most drum and base levels, the ticks each read dry
//! or full, the ticks a level loop stood at its ceiling, and the least vapour
//! any stage sent up (DESIGN §79). Kept in `tools/steptime/prototypes/m74_4/`.
//!
//! It drives the plant through the same `Session` the gdext binding forwards
//! to, with the scene's own command text, on the shipped engine. Put this file
//! in `crates/godot-ext/tests/`, register it in `suite.rs`, build the release
//! test binary, and run it with `m74_4_grid --nocapture --test-threads 1` and,
//! as env vars: PLANT (`holdup`, or `trays` for the tray plant on the same
//! routes), WARM (ticks before the first move, default 1000), GAP (ticks
//! between moves, default 300), RUN (ticks after the last move, default
//! 12000), JOBS (runs in parallel, default 8) and SPECS or SPECS_FILE: runs
//! separated by `;`, each a sequence of moves separated by `>`, each move
//! `key=value` pairs joined by `+` and sent on one tick, `@N` after a move the
//! ticks to the next. Keys:
//! - `furnace` MW, `reflux`;
//! - `flow`: a share of the file's loop flows (85 / 95 / 65 kg/s); `ret`: the
//!   loops' return temperatures moved by this many K from the file's (150 /
//!   215 / 270 °C);
//! - `reb`: the reboiler held at this many MW; `free=1`: freed; `rebstep`:
//!   the screen's T/G — held at this many MW from the hold if held, from the
//!   duty it runs at if free.
//!
//! A spec may end `~N`: that run's ticks after its last move, in place of RUN.
//!
//! Each run prints `RESULT <spec>: RUNS <reading> | <over the run>` or
//! `RESULT <spec>: FAIL at t=… (… after the last move; last sent …): reason
//! | <over the run so far>`.
use refinery_godot_ext::bridge::Session;
use serde_json::Value;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

/// The loop plant's file flows [kg/s] and return temperatures [°C].
const LOOP_FLOWS: [f64; 3] = [85.0, 95.0, 65.0];
const LOOP_RETURNS_C: [f64; 3] = [150.0, 215.0, 270.0];

fn env_u(k: &str, d: i64) -> i64 {
    std::env::var(k).ok().map_or(d, |v| v.parse().unwrap())
}

/// What a run's holdup did over its ticks after the first move.
#[derive(Default)]
struct OverTheRun {
    drum: (f64, f64),
    base: (f64, f64),
    drum_dry: u32,
    drum_full: u32,
    base_dry: u32,
    base_full: u32,
    drum_max: u32,
    base_max: u32,
    least_vapour: f64,
    least_vapour_stage: usize,
    /// The tick being read, counted from the first move.
    tick: i64,
    reboiler: (f64, f64),
    declined: u32,
    /// Ticks some tray passed no liquid down, the most trays at once, and
    /// the last tick (after the first move) any did.
    dry_tray_ticks: u32,
    most_dry_trays: usize,
    last_dry_tray_tick: i64,
    worst_ms: f64,
    /// Ticks some stage sent up no vapour, the most at once, the last tick
    /// any did, the stages that ever did (bit per stage), and how many at
    /// the end.
    cold_ticks: u32,
    most_cold: usize,
    last_cold_tick: i64,
    cold_stages: u64,
    cold_now: usize,
    /// M74.6.0: ticks some stage held no liquid at all (0 kg), the most at
    /// once, the stages that ever did, the last tick any did, how many at
    /// the end; the least liquid any tray held [kg].
    empty_ticks: u32,
    most_empty: usize,
    empty_stages: u64,
    last_empty_tick: i64,
    empty_now: usize,
    least_held: f64,
}

impl OverTheRun {
    fn new() -> Self {
        OverTheRun {
            drum: (f64::MAX, f64::MIN),
            base: (f64::MAX, f64::MIN),
            least_vapour: f64::MAX,
            least_held: f64::MAX,
            reboiler: (f64::MAX, f64::MIN),
            ..Default::default()
        }
    }

    fn read(&mut self, s: &Value, column: usize) {
        let col = &s["nodes"][column];
        if !col["column_reboiler_declined"].is_null() {
            self.declined += 1;
        }
        let op = &col["column_operation"];
        if let Some(down) = op["stage_liquid_mol_s"].as_array() {
            let dry = down[..down.len() - 1].iter().filter(|l| l.as_f64() == Some(0.0)).count();
            if dry > 0 {
                self.dry_tray_ticks += 1;
                self.most_dry_trays = self.most_dry_trays.max(dry);
                self.last_dry_tray_tick = self.tick;
            }
        }
        let reb = col["column_duty"]["reboiler_w"].as_f64().unwrap_or(0.0) / 1e6;
        self.reboiler = (self.reboiler.0.min(reb), self.reboiler.1.max(reb));
        if let Some(vapour) = op["stage_vapour_mol_s"].as_array() {
            let mut cold = 0;
            for (j, v) in vapour.iter().enumerate() {
                if v.as_f64().unwrap() < 1.0e-3 {
                    cold += 1;
                    self.cold_stages |= 1 << j;
                }
            }
            self.cold_now = cold;
            if cold > 0 {
                self.cold_ticks += 1;
                self.most_cold = self.most_cold.max(cold);
                self.last_cold_tick = self.tick;
            }
            for (j, v) in vapour.iter().enumerate() {
                let v = v.as_f64().unwrap();
                if v < self.least_vapour {
                    self.least_vapour = v;
                    self.least_vapour_stage = j + 1;
                }
            }
        }
        if let Some(held) = col["kind"]["cascade"]["holdup"]["state"]["stage_liquid"].as_array() {
            let mut empty = 0;
            for (j, m) in held.iter().enumerate() {
                let m = m.as_f64().unwrap();
                self.least_held = self.least_held.min(m);
                if m == 0.0 {
                    empty += 1;
                    self.empty_stages |= 1 << j;
                }
            }
            self.empty_now = empty;
            if empty > 0 {
                self.empty_ticks += 1;
                self.most_empty = self.most_empty.max(empty);
                self.last_empty_tick = self.tick;
            }
        }
        let h = &op["holdup"];
        if h.is_null() {
            return;
        }
        let (d, b) = (h["drum_level"].as_f64().unwrap(), h["base_level"].as_f64().unwrap());
        self.drum = (self.drum.0.min(d), self.drum.1.max(d));
        self.base = (self.base.0.min(b), self.base.1.max(b));
        match h["drum_limit"].as_str().unwrap() {
            "dry" => self.drum_dry += 1,
            "full" => self.drum_full += 1,
            _ => {}
        }
        match h["base_limit"].as_str().unwrap() {
            "dry" => self.base_dry += 1,
            "full" => self.base_full += 1,
            _ => {}
        }
        if h["drum_loop_at_max"].as_bool() == Some(true) {
            self.drum_max += 1;
        }
        if h["base_loop_at_max"].as_bool() == Some(true) {
            self.base_max += 1;
        }
    }

    fn text(&self) -> String {
        format!(
            "empty {} ticks, {} at most, stages {}, last at {}, {} at the end, least held {:.1} kg | cold {} ticks, {} at most, stages {}, last at {}, {} at the end | drum {:.3}-{:.3} base {:.3}-{:.3} dry {}/{} full {}/{} atmax {}/{} reb {:.1}-{:.1} MW least vapour {:.0} mol/s on {} declined {} dry trays {} ticks, {} at most, last at {} worst {:.1} ms",
            self.empty_ticks, self.most_empty,
            (0..64).filter(|j| self.empty_stages >> j & 1 == 1).map(|j| (j + 1).to_string()).collect::<Vec<_>>().join(","),
            self.last_empty_tick, self.empty_now, self.least_held,
            self.cold_ticks, self.most_cold,
            (0..64).filter(|j| self.cold_stages >> j & 1 == 1).map(|j| (j + 1).to_string()).collect::<Vec<_>>().join(","),
            self.last_cold_tick, self.cold_now,
            self.drum.0, self.drum.1, self.base.0, self.base.1, self.drum_dry, self.base_dry,
            self.drum_full, self.base_full, self.drum_max, self.base_max, self.reboiler.0,
            self.reboiler.1, self.least_vapour, self.least_vapour_stage, self.declined,
            self.dry_tray_ticks, self.most_dry_trays, self.last_dry_tray_tick, self.worst_ms
        )
    }
}

fn reading(s: &Value, column: usize, furnace: usize, draws: &[i64]) -> String {
    let col = &s["nodes"][column];
    let op = &col["column_operation"];
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
    let flows: Vec<String> = draws
        .iter()
        .map(|&id| {
            let e = s["edges"].as_array().unwrap().iter().find(|e| e["id"].as_i64() == Some(id)).unwrap();
            format!("{:.2}", e["stream"]["mass_flow"].as_f64().unwrap())
        })
        .collect();
    let h = &op["holdup"];
    let held = if h.is_null() {
        String::from("no holdup")
    } else {
        format!(
            "{} drum {:.3} {} base {:.3} {}",
            if h["reboiler_held"].as_bool() == Some(true) { "held" } else { "free" },
            h["drum_level"].as_f64().unwrap(),
            h["drum_limit"].as_str().unwrap(),
            h["base_level"].as_f64().unwrap(),
            h["base_limit"].as_str().unwrap()
        )
    };
    let declined = if col["column_reboiler_declined"].is_null() {
        String::new()
    } else {
        format!(
            "DECLINED {:.1} MW ({}) ",
            col["column_reboiler_declined"]["asked_w"].as_f64().unwrap() / 1e6,
            col["column_reboiler_declined"]["reason"].as_str().unwrap_or("")
        )
    };
    format!(
        "{declined}{held} | flood {} weep {} | loops {} R {:.3} coil {:.1} cond {:.2} reb {:.2} MW | products {}",
        stage_list("trays_flooding"),
        stage_list("trays_weeping"),
        loops.join("/"),
        op["reflux_ratio"].as_f64().unwrap(),
        s["nodes"][furnace]["kind"]["coil"]["temperature"].as_f64().unwrap() - 273.15,
        col["column_duty"]["condenser_w"].as_f64().unwrap() / 1e6,
        col["column_duty"]["reboiler_w"].as_f64().unwrap() / 1e6,
        flows.join("/")
    )
}

/// One trace line: the feed, each stage's vapour [mol/s] and temperature
/// [°C], the drum and base levels, the reboiler [MW].
fn trace(s: &Value, column: usize, feed: i64, draws: &[i64]) -> String {
    let col = &s["nodes"][column];
    let op = &col["column_operation"];
    let list = |key: &str, k: f64, off: f64| -> String {
        op[key].as_array().map_or(String::from("-"), |l| {
            l.iter().map(|x| format!("{:.0}", x.as_f64().unwrap() * k - off)).collect::<Vec<_>>().join(",")
        })
    };
    let fed = s["edges"].as_array().unwrap().iter().find(|e| e["id"].as_i64() == Some(feed)).map_or(0.0, |e| e["stream"]["mass_flow"].as_f64().unwrap());
    let h = &op["holdup"];
    let state = &col["kind"]["cascade"]["holdup"]["state"];
    let held = state["stage_liquid"].as_array().map_or(String::from("-"), |l| {
        l.iter().map(|x| format!("{:.1}", x.as_f64().unwrap() / 1e3)).collect::<Vec<_>>().join(",")
    });
    let drum_t = state["drum_liquid"].as_f64().unwrap_or(f64::NAN) / 1e3;
    let products: Vec<String> = draws
        .iter()
        .map(|&id| {
            s["edges"].as_array().unwrap().iter().find(|e| e["id"].as_i64() == Some(id))
                .map_or(String::from("?"), |e| format!("{:.1}", e["stream"]["mass_flow"].as_f64().unwrap_or(f64::NAN)))
        })
        .collect();
    format!(
        "feed {:.1} held[t] [{}] drum {:.1} t V [{}] T [{}] drum {:.3} base {:.3} reb {:.1} R {:.2} cond {:.1} MW products {}",
        fed,
        held,
        drum_t,
        list("stage_vapour_mol_s", 1.0, 0.0),
        list("stage_temperatures_k", 1.0, 273.15),
        h["drum_level"].as_f64().unwrap_or(f64::NAN),
        h["base_level"].as_f64().unwrap_or(f64::NAN),
        col["column_duty"]["reboiler_w"].as_f64().unwrap_or(0.0) / 1e6,
        op["reflux_ratio"].as_f64().unwrap_or(f64::NAN),
        col["column_duty"]["condenser_w"].as_f64().unwrap_or(f64::NAN) / 1e6,
        products.join("/"),
    )
}

fn one_run(src: &str, spec: &str, warm: i64, gap: i64, run: i64) -> String {
    let (spec, run) = match spec.split_once('~') {
        Some((body, n)) => (body, n.parse().unwrap()),
        None => (spec, run),
    };
    refinery_solvers::probe::set_total_reflux(false);
    let mut session = Session::default();
    assert_eq!(session.load_scenario(src), "null");
    let column = session.node_id("column");
    let furnace = session.node_id("furnace");
    let valve = if src.contains("feed_valve") { session.node_id("feed_valve") } else { -1 };
    let feed_edge = session.edge_id("transfer_line");
    let trace_every = env_u("TRACE", 0);
    let draws: Vec<i64> = ["naphtha_draw", "kerosene_draw", "diesel_draw", "gas_oil_draw", "residue_draw"]
        .iter()
        .map(|d| session.edge_id(d))
        .collect();
    let mut moves: Vec<(i64, &str)> = Vec::new();
    let mut at = warm;
    for mv in spec.split('>') {
        let (body, wait) = mv.split_once('@').map_or((mv, gap), |(b, w)| (b, w.parse().unwrap()));
        moves.push((at, body));
        at += wait;
    }
    let last_move = moves.last().unwrap().0;
    let total = last_move + run;
    let mut over = OverTheRun::new();
    for t in 1..=total {
        if let Some(&(_, body)) = moves.iter().find(|(m, _)| *m == t - 1) {
            for kv in body.split('+') {
                let (key, value) = kv.split_once('=').unwrap();
                let v: f64 = value.parse().unwrap();
                let cmds: Vec<String> = match key {
                    "tr" => {
                        refinery_solvers::probe::set_total_reflux(v > 0.5);
                        vec![]
                    }
                    "valve" => vec![format!(r#"{{"cmd":"set_valve_opening","node":{valve},"opening":{v:?}}}"#)],
                    "furnace" => vec![format!(r#"{{"cmd":"set_furnace_duty","node":{furnace},"duty":{:?}}}"#, v * 1e6)],
                    "reflux" => vec![format!(r#"{{"cmd":"set_reflux_ratio","node":{column},"reflux_ratio":{v:?}}}"#)],
                    "reb" => vec![format!(r#"{{"cmd":"set_reboiler_duty","node":{column},"duty":{:?}}}"#, v * 1e6)],
                    "free" => vec![format!(r#"{{"cmd":"set_reboiler_duty","node":{column},"duty":null}}"#)],
                    "rebstep" => {
                        let s: Value = serde_json::from_str(&session.snapshot_json()).unwrap();
                        let col = &s["nodes"][column as usize];
                        let from = col["kind"]["cascade"]["reboiler_duty"]
                            .as_f64()
                            .unwrap_or_else(|| col["column_duty"]["reboiler_w"].as_f64().unwrap());
                        vec![format!(r#"{{"cmd":"set_reboiler_duty","node":{column},"duty":{:?}}}"#, (from + v * 1e6).max(0.0))]
                    }
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
        let started = std::time::Instant::now();
        let out = session.tick();
        let ms = started.elapsed().as_secs_f64() * 1e3;
        if out != "null" {
            let after_last = t - 1 - last_move;
            let reached = moves.iter().filter(|(m, _)| *m < t).last().map_or("nothing yet", |m| m.1);
            let reason: String = out.chars().take(600).collect();
            return format!(
                "RESULT {spec}: FAIL at t={t} ({after_last} after the last move; last sent {reached}): {reason} | {}",
                over.text()
            );
        }
        if t > warm {
            over.tick = t - warm;
            over.worst_ms = over.worst_ms.max(ms);
            let s: Value = serde_json::from_str(&session.snapshot_json()).unwrap();
            over.read(&s, column as usize);
            if trace_every > 0 && (t - warm) % trace_every == 0 {
                eprintln!("TRACE {spec} t={}: {}", t - warm, trace(&s, column as usize, feed_edge, &draws));
            }
        }
    }
    let s: Value = serde_json::from_str(&session.snapshot_json()).unwrap();
    format!(
        "RESULT {spec}: RUNS {} | {}",
        reading(&s, column as usize, furnace as usize, &draws),
        over.text()
    )
}

#[test]
fn m74_6_0_empty() {
    let Ok(name) = std::env::var("PLANT") else {
        return;
    };
    let file = match name.as_str() {
        "holdup" => "crude_unit_holdup.toml",
        "trays" => "crude_unit_trays.toml",
        "valved" => "crude_unit_holdup.toml",
        "valved_check" => "crude_unit_holdup.toml",
        _ => panic!("PLANT {name}"),
    };
    let mut src = std::fs::read_to_string(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios").join(file)).unwrap();
    if name.starts_with("valved") {
        let before = src.clone();
        src = src
            .replace("[nodes.furnace]", "[nodes.feed_valve]
type = \"valve\"
kv = 5000.0
opening = 1.0

[nodes.furnace]")
            .replace(
                "name = \"crude_line\"
from = \"crude_source\"
to = \"furnace\"",
                "name = \"crude_line\"
from = \"crude_source\"
to = \"feed_valve\"
length_m = 1.0
diameter_m = 0.08

[[pipes]]
name = \"valve_line\"
from = \"feed_valve\"
to = \"furnace\"",
            );
        assert!(src != before && src.contains("valve_line"), "the valve was not put in");
        // `valved_check`: a check valve after it (wide, opening at 0.05 bar),
        // so the line cannot run backwards out of the column.
        if name == "valved_check" {
            src = src
                .replace("[nodes.furnace]", "[nodes.feed_check]
type = \"check_valve\"
kv = 5000.0
full_open_bar = 0.05

[nodes.furnace]")
                .replace(
                    "name = \"valve_line\"
from = \"feed_valve\"
to = \"furnace\"",
                    "name = \"valve_line\"
from = \"feed_valve\"
to = \"feed_check\"
length_m = 1.0
diameter_m = 0.08

[[pipes]]
name = \"check_line\"
from = \"feed_check\"
to = \"furnace\"",
                );
            assert!(src.contains("check_line"), "the check valve was not put in");
        }
    }
    // SPECS_FILE: the specs in a file (a Windows environment variable holds
    // 32 767 characters, a grid's specs far more).
    let specs_text = match std::env::var("SPECS_FILE") {
        Ok(path) => std::fs::read_to_string(path).unwrap(),
        Err(_) => std::env::var("SPECS").unwrap(),
    };
    let specs: Vec<&str> = specs_text.split(';').map(str::trim).filter(|s| !s.is_empty()).collect();
    let (warm, gap, run) = (env_u("WARM", 1000), env_u("GAP", 300), env_u("RUN", 12000));
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
                let line = one_run(&src, specs[i], warm, gap, run);
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
