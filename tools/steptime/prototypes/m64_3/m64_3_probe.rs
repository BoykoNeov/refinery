//! THROWAWAY M64.3 probe: the stripped crude unit with its charge on a flow loop.
//!
//! Every number in docs/DESIGN.md §69.9 came from this. Applied to `031fa4d` in a
//! detached worktree: `probe_counters.patch` (the held column's warm failures
//! and continuations counted, `NO_FALLBACK` to skip the continuation mid-run, the
//! warm-start order trial M64.3 then shipped), this file in
//! `crates/scenarios/tests/`, registered in `suite.rs`. Run the release test
//! binary with `m64_3_probe --nocapture` and, as env vars: FURNACE, COIL, PA,
//! KV, OPENING, STEAM, LOOP (on), METER, SP, GAIN, TI, FLOW (simple|newton),
//! BLOCKS (tick counts reported after each), MOVES (`kind:value`, kinds steam,
//! feed, furnace, sp, `pa:index:MW`; `+` joins moves into one tick, `,` makes a
//! sequence), MOVE_TICKS. It builds the plant from `crude_unit_steam.toml` as
//! `make_stripped.py` does.
use refinery_core::engine::Engine;
use refinery_core::snapshot::Command;
use refinery_core::units::Watt;
use refinery_scenarios::{build_engine, load_str};

const STEAM: &str = include_str!("../../../scenarios/crude_unit_steam.toml");

fn env_s(k: &str, d: &str) -> String {
    std::env::var(k).unwrap_or_else(|_| d.to_string())
}

fn rep(s: String, from: &str, to: &str) -> String {
    assert_eq!(s.matches(from).count(), 1, "{from}");
    s.replacen(from, to, 1)
}

fn plant() -> String {
    let mut p = STEAM.to_string();
    p = rep(p, "reflux_ratio = 4.0\n", "reflux_ratio = 4.0\nreboiler_duty_mw = 0.0\n");
    p = rep(p, "duty_mw = 20.0\n", &format!("duty_mw = {}\n", env_s("FURNACE", "60.0")));
    p = rep(p, "coil_temperature_c = 322.05\n", &format!("coil_temperature_c = {}\n", env_s("COIL", "460.0")));
    p = rep(
        p,
        "pumparounds = [{ stage = 4, duty_mw = 15.0 }, { stage = 6, duty_mw = 15.0 }, { stage = 7, duty_mw = 10.0 }]",
        &format!("pumparounds = [{{ stage = 4, duty_mw = {0} }}, {{ stage = 6, duty_mw = {0} }}, {{ stage = 7, duty_mw = {0} }}]", env_s("PA", "0.0")),
    );
    p = rep(p, "kv = 135.0\nopening = 1.0\n", &format!("kv = {}\nopening = {}\n", env_s("KV", "400.0"), env_s("OPENING", "0.5")));
    p = rep(p, "opening = 0.4\nx_t", &format!("opening = {}\nx_t", env_s("STEAM", "0.4")));
    if std::env::var("LOOP").is_ok() {
        p.push_str(&format!(
            "\n[[controls]]\nname = \"charge_flow\"\nmeasurement = {{ pipe = \"{}\", variable = \"flow\" }}\nactuator = \"feed_valve\"\nalgorithm = \"pi\"\nmode = \"auto\"\naction = \"reverse\"\nsetpoint_kg_per_s = {}\ngain_per_kg_per_s = {}\nintegral_time_s = {}\ninitial_output = {}\n",
            env_s("METER", "crude_line"),
            env_s("SP", "112.8"),
            env_s("GAIN", "0.002"),
            env_s("TI", "10.0"),
            env_s("OPENING", "0.5")
        ));
    }
    if let Ok(flow) = std::env::var("FLOW") {
        p = rep(p, "flow = \"newton\"", &format!("flow = \"{flow}\""));
    }
    p
}

fn id(engine: &Engine, name: &str) -> refinery_core::graph::NodeId {
    engine.snapshot().nodes.iter().find(|x| x.name == name).unwrap().id
}

fn after(kind: &str, key: &str) -> f64 {
    kind.find(key)
        .and_then(|i| {
            kind[i + key.len()..]
                .split(|c: char| c == ',' || c == ')' || c == ' ' || c == '}')
                .next()
                .and_then(|v| v.parse::<f64>().ok())
        })
        .unwrap_or(f64::NAN)
}

fn step(engine: &mut Engine, ticks: u32) -> Result<String, String> {
    let clock = std::time::Instant::now();
    let mut worst = 0.0f64;
    let mut first = 0.0f64;
    let c0 = refinery_solvers::probe3::counts();
    for i in 0..ticks {
        let t = std::time::Instant::now();
        engine.tick().map_err(|e| {
            let s = e.to_string();
            s[..1200.min(s.len())].to_string()
        })?;
        let ms = t.elapsed().as_secs_f64() * 1e3;
        if i == 0 {
            first = ms;
        }
        worst = worst.max(ms);
    }
    let c1 = refinery_solvers::probe3::counts();
    let mean = clock.elapsed().as_secs_f64() * 1e3 / ticks as f64;
    let s = engine.snapshot();
    let c = s.nodes.iter().find(|n| n.name == "column").unwrap();
    let op = c.column_operation.clone();
    let duty = c.column_duty;
    let fu = s.nodes.iter().find(|n| n.name == "furnace").unwrap();
    let kind = format!("{:?}", fu.kind);
    let coil = after(&kind[kind.find("coil").unwrap()..], "temperature: Kelvin(") - 273.15;
    let fv = s.nodes.iter().find(|n| n.name == "feed_valve").unwrap();
    let opening = after(&format!("{:?}", fv.kind), "opening: ");
    if std::env::var("DUMP").is_ok() {
        eprintln!("FURNACE {kind}\nCOLUMN {c:?}");
    }
    let edge = |n: &str| s.edges.iter().find(|e| e.name == n).map(|e| e.stream.mass_flow.value()).unwrap_or(f64::NAN);
    let (q, r, tb) = op
        .map(|o| (o.feed_vapour_fraction, o.reflux_ratio, o.stage_temperatures_k.last().copied().unwrap_or(f64::NAN) - 273.15))
        .unwrap_or((f64::NAN, f64::NAN, f64::NAN));
    Ok(format!(
        "t{} open {opening:.4} charge {:.3}/{:.3} feed {:.1}C q {:.4} steam {:.3} | R {:.3} bottom {:.2}C cond {:.2} reb {:.3} MW | coil {:.2}C | warmfail +{} cont +{} | first {first:.1} mean {mean:.2} worst {worst:.1} ms",
        s.tick,
        edge("feed_spool"),
        edge("transfer_line"),
        fu.temperature_k - 273.15,
        q,
        edge("stripping_steam"),
        r,
        tb,
        duty.map_or(f64::NAN, |d| d.condenser_w / 1e6),
        duty.map_or(f64::NAN, |d| d.reboiler_w / 1e6),
        coil,
        c1.0 - c0.0,
        c1.1 - c0.1,
    ))
}

fn report(label: &str, out: &Result<String, String>) -> bool {
    match out {
        Ok(s) => {
            eprintln!("  {label}: {s}");
            true
        }
        Err(e) => {
            eprintln!("  {label}: FAIL {e}");
            false
        }
    }
}

#[test]
fn probe() {
    let src = plant();
    if std::env::var("PRINT").is_ok() {
        println!("{src}");
    }
    let mut engine = match load_str(&src).map_err(|e| e.to_string()).and_then(|f| build_engine(&f).map_err(|e| e.to_string())) { Ok(e) => e, Err(e) => { eprintln!("  LOAD {e}"); return; } };
    let furnace = id(&engine, "furnace");
    let column = id(&engine, "column");
    let valve = id(&engine, "steam_valve");
    let feed = id(&engine, "feed_valve");
    // BLOCKS: tick counts reported after each.
    for b in env_s("BLOCKS", "1,9,90,900").split(',').filter(|x| !x.is_empty()) {
        let n: u32 = b.parse().unwrap();
        if !report(&format!("block {n}"), &step(&mut engine, n)) {
            return;
        }
    }
    // MOVES: kind:value[:value], each followed by MOVE_TICKS ticks (reported 1, then rest).
    let move_ticks: u32 = env_s("MOVE_TICKS", "300").parse().unwrap();
    for m in env_s("MOVES", "").split(',').filter(|x| !x.is_empty()) {
        for one in m.split('+') {
        let parts: Vec<&str> = one.split(':').collect();
        let v: f64 = parts[1].parse().unwrap();
        let cmd = match parts[0] {
            "steam" => Command::SetValveOpening { node: valve, opening: v },
            "feed" => Command::SetValveOpening { node: feed, opening: v },
            "sp" => Command::SetSetpoint { loop_id: refinery_core::graph::LoopId(0), value: refinery_core::graph::ControlledValue::Flow { kg_per_s: refinery_core::units::KgPerSec(v) } },
            "furnace" => Command::SetFurnaceDuty { node: furnace, duty: Watt(v * 1e6) },
            "pa" => Command::SetPumparoundDuty { node: column, pumparound: v as u32, duty: Watt(parts[2].parse::<f64>().unwrap() * 1e6) },
            other => panic!("{other}"),
        };
        if let Err(e) = engine.apply(cmd) {
            eprintln!("  move {m}: REFUSED {e}");
            return;
        }
        }
        if !report(&format!("move {m} tick 1"), &step(&mut engine, 1)) {
            return;
        }
        if move_ticks > 1 && !report(&format!("move {m} +{}", move_ticks - 1), &step(&mut engine, move_ticks - 1)) {
            return;
        }
    }
}
