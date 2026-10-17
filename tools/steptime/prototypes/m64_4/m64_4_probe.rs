//! THROWAWAY M64.4 probe: the column screen's keys on the stripped crude unit.
//!
//! Every number in docs/DESIGN.md §69.10 came from this. It drives the plant
//! through the same `Session` the gdext binding forwards to, with the scene's
//! command text, so what it measures is what the screen does. Put this file in
//! `crates/godot-ext/tests/`, register it in `suite.rs`, and run the release test
//! binary with `m64_4_probe --nocapture --test-threads 1` and, as env vars:
//! WARM (ticks run before the move, default 1000), RUN (ticks after, default
//! 5000), EVERY (report interval, default 500), MOVES (`kind:value`, `+`-joined
//! into one tick, `,` a sequence GAP ticks apart, default 3000; kinds `furnace` MW, `steam` opening, `pa` MW on all three,
//! `pa4`/`pa6`/`pa7` MW on that stage's pumparound), and TIMING (print the
//! wall time of each tick + its snapshot, the screen's per-tick work).
use refinery_godot_ext::bridge::Session;
use serde_json::Value;
use std::time::Instant;

fn env_u(k: &str, d: i64) -> i64 {
    std::env::var(k).ok().map_or(d, |v| v.parse().unwrap())
}

fn reading(session: &Session, ids: &Ids) -> String {
    let s: Value = serde_json::from_str(&session.snapshot_json()).unwrap();
    let col = &s["nodes"][ids.column as usize];
    let op = &col["column_operation"];
    let stages = op["stage_temperatures_k"].as_array().unwrap();
    let edge = |id: i64| {
        s["edges"]
            .as_array()
            .unwrap()
            .iter()
            .find(|e| e["id"].as_i64() == Some(id))
            .unwrap()["stream"]
            .clone()
    };
    format!(
        "bottom {:.2} C  reflux {:.3}  flash {:.3}  coil {:.1} C  feed {:.1} C {:.1} kg/s  steam {:.3} kg/s  cond {:.2} MW  reb {:.3} MW  min-vap-above-feed {:.0}",
        stages.last().unwrap().as_f64().unwrap() - 273.15,
        op["reflux_ratio"].as_f64().unwrap(),
        op["feed_vapour_fraction"].as_f64().unwrap(),
        s["nodes"][ids.furnace as usize]["kind"]["coil"]["temperature"].as_f64().unwrap() - 273.15,
        edge(ids.feed)["temperature"].as_f64().unwrap() - 273.15,
        edge(ids.feed)["mass_flow"].as_f64().unwrap(),
        edge(ids.steam)["mass_flow"].as_f64().unwrap(),
        col["column_duty"]["condenser_w"].as_f64().unwrap() / 1e6,
        col["column_duty"]["reboiler_w"].as_f64().unwrap() / 1e6,
        op["stage_vapour_mol_s"].as_array().unwrap()[..7]
            .iter()
            .map(|v| v.as_f64().unwrap())
            .fold(f64::INFINITY, f64::min),
    )
}

struct Ids {
    furnace: i64,
    column: i64,
    steam_valve: i64,
    feed: i64,
    steam: i64,
}

#[test]
fn m64_4_probe() {
    let src = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/crude_unit_stripped.toml"),
    )
    .unwrap();
    let mut session = Session::default();
    assert_eq!(session.load_scenario(&src), "null");
    let ids = Ids {
        furnace: session.node_id("furnace"),
        column: session.node_id("column"),
        steam_valve: session.node_id("steam_valve"),
        feed: session.edge_id("transfer_line"),
        steam: session.edge_id("stripping_steam"),
    };
    println!(
        "ids: furnace {} column {} steam_valve {} transfer_line {} stripping_steam {}",
        ids.furnace, ids.column, ids.steam_valve, ids.feed, ids.steam
    );
    let warm = env_u("WARM", 1000);
    let run = env_u("RUN", 5000);
    let every = env_u("EVERY", 500);
    let timing = std::env::var("TIMING").is_ok();
    let moves = std::env::var("MOVES").unwrap_or_default();
    let mut slowest = (0.0f64, 0i64);
    let mut block = 0.0f64;
    for t in 1..=warm + run {
        let gap = env_u("GAP", 3000);
        let step = (t - warm - 1) / gap;
        let sequence: Vec<&str> = moves.split(',').filter(|m| !m.is_empty()).collect();
        if t > warm && (t - warm - 1) % gap == 0 && (step as usize) < sequence.len() {
            let moves = sequence[step as usize];
            for mv in moves.split('+') {
                let (kind, value) = mv.split_once(':').unwrap();
                let v: f64 = value.parse().unwrap();
                let (c, f, sv) = (ids.column, ids.furnace, ids.steam_valve);
                let cmds: Vec<String> = match kind {
                    "furnace" => vec![format!(r#"{{"cmd":"set_furnace_duty","duty":{:.1},"node":{f}}}"#, v * 1e6)],
                    "steam" => vec![format!(r#"{{"cmd":"set_valve_opening","node":{sv},"opening":{v:?}}}"#)],
                    "pa" => (0..3)
                        .map(|i| format!(r#"{{"cmd":"set_pumparound_duty","duty":{:.1},"node":{c},"pumparound":{i}}}"#, v * 1e6))
                        .collect(),
                    "pa4" | "pa6" | "pa7" => {
                        let i = ["pa4", "pa6", "pa7"].iter().position(|k| *k == kind).unwrap();
                        vec![format!(r#"{{"cmd":"set_pumparound_duty","duty":{:.1},"node":{c},"pumparound":{i}}}"#, v * 1e6)]
                    }
                    _ => panic!("{kind}"),
                };
                for cmd in cmds {
                    let out = session.apply_command_json(&cmd);
                    assert_eq!(out, "null", "{cmd}");
                }
            }
            println!("t={:5} MOVE {moves}", t - 1);
        }
        let start = Instant::now();
        let out = session.tick();
        let _ = session.snapshot_json();
        let ms = start.elapsed().as_secs_f64() * 1e3;
        block += ms;
        if ms > slowest.0 {
            slowest = (ms, t);
        }
        if out != "null" {
            println!("t={t:5} REFUSED {out}");
            println!("RESULT {moves}: FAIL at t={} ({} after the move)", t, t - warm);
            return;
        }
        if timing && (t <= 3 || (t > warm && t <= warm + 3)) {
            println!("t={t:5} tick+snapshot {ms:.1} ms");
        }
        if t % every == 0 {
            println!("t={t:5} {}  [{:.2} ms/tick over the block]", reading(&session, &ids), block / every as f64);
            block = 0.0;
        }
    }
    println!("RESULT {moves}: RUNS  {}  (slowest tick {:.1} ms at t={})", reading(&session, &ids), slowest.0, slowest.1);
}
