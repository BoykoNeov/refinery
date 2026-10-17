//! THROWAWAY M64.0 probe walk: the steamed crude unit's reboiler walked to zero.
use refinery_core::engine::Engine;
use refinery_core::snapshot::Command;
use refinery_core::units::Watt;
use refinery_scenarios::{build_engine, load_str};
use refinery_solvers::probe;

fn env_f(k: &str, d: f64) -> f64 {
    std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d)
}

fn id(engine: &Engine, name: &str) -> refinery_core::graph::NodeId {
    engine.snapshot().nodes.iter().find(|x| x.name == name).unwrap().id
}

fn coil_max(kind: &str) -> f64 {
    // The hottest "Kelvin(" after "coil" in the furnace's Debug.
    let Some(i) = kind.find("coil") else { return f64::NAN };
    let mut best = f64::NAN;
    let mut rest = &kind[i..];
    while let Some(j) = rest.find(" temperature: Kelvin(") {
        rest = &rest[j + 21..];
        if let Some(v) = rest.split(')').next().and_then(|v| v.parse::<f64>().ok()) {
            best = best.max(v - 273.15);
        }
    }
    best
}

fn step(engine: &mut Engine, ticks: u32) -> Result<String, String> {
    let clock = std::time::Instant::now();
    let mut worst = 0.0f64;
    for _ in 0..ticks {
        let t = std::time::Instant::now();
        engine.tick().map_err(|e| {
            let s = e.to_string();
            s[..900.min(s.len())].to_string()
        })?;
        worst = worst.max(t.elapsed().as_secs_f64() * 1e3);
    }
    let mean = clock.elapsed().as_secs_f64() * 1e3 / ticks as f64;
    let s = engine.snapshot();
    let c = s.nodes.iter().find(|n| n.name == "column").unwrap();
    let op = c.column_operation.clone();
    let duty = c.column_duty;
    let fu = s.nodes.iter().find(|n| n.name == "furnace").unwrap();
    let kind = format!("{:?}", fu.kind);
    if std::env::var("DUMP").is_ok() {
        eprintln!("FURNACE {kind}\nCOLUMN {c:?}");
    }
    let edge = |n: &str| s.edges.iter().find(|e| e.name == n).map(|e| e.stream.mass_flow.value()).unwrap_or(f64::NAN);
    let (q, r, tb) = op.map(|o| (o.feed_vapour_fraction, o.reflux_ratio, o.stage_temperatures_k.last().copied().unwrap_or(f64::NAN) - 273.15)).unwrap_or((f64::NAN, f64::NAN, f64::NAN));
    Ok(format!(
        "t{} feed {:.1}C q {:.3} charge {:.2} steam {:.3} kg/s | R {:.3} bottom {:.1}C cond {:.2} reb {:.2} MW | coil max {:.1}C | flooded {} | mean {mean:.2} worst {worst:.2} ms\n        {}",
        s.tick,
        fu.temperature_k - 273.15,
        q,
        edge("transfer_line"),
        edge("stripping_steam"),
        r,
        tb,
        duty.map_or(f64::NAN, |d| d.condenser_w / 1e6),
        duty.map_or(f64::NAN, |d| d.reboiler_w / 1e6),
        coil_max(&kind),
        c.column_flooded.is_some(),
        probe::last()
    ))
}

fn report(label: &str, out: &Result<String, String>) -> bool {
    match out {
        Ok(s) => { eprintln!("  {label}: {s}"); true }
        Err(e) => { eprintln!("  {label}: FAIL {e}"); false }
    }
}

#[test]
fn walk() {
    let path = std::env::var("PLANT").unwrap_or("../../scenarios/crude_unit_steam.toml".into());
    let src = std::fs::read_to_string(path).unwrap();
    probe::set_held(None);
    let mut engine = build_engine(&load_str(&src).unwrap()).unwrap();
    let furnace = id(&engine, "furnace");
    let column = id(&engine, "column");
    let valve = id(&engine, "steam_valve");
    if let Ok(cold) = std::env::var("COLD") {
        // Cold: the plant built with every setting at its end state, held from tick 1.
        let mw: f64 = cold.parse().unwrap();
        engine.apply(Command::SetFurnaceDuty { node: furnace, duty: Watt(mw * 1e6) }).unwrap();
        let pas: Vec<f64> = std::env::var("PA").unwrap_or("0,0,0".into()).split(',').map(|v| v.parse().unwrap()).collect();
        for (i, d) in pas.iter().enumerate() {
            engine.apply(Command::SetPumparoundDuty { node: column, pumparound: i as u32, duty: Watt(d * 1e6) }).unwrap();
        }
        probe::set_held(Some(0.0));
        let _ = report("cold tick 1", &step(&mut engine, 1)) && report("cold 3000", &step(&mut engine, 3000)) && report("cold +3000", &step(&mut engine, 3000)) && report("cold +3000", &step(&mut engine, 3000));
        return;
    }
    if !report("start 600", &step(&mut engine, 600)) { return; }
    if let Ok(mw) = std::env::var("BASE_FURNACE") {
        // The shipped plant, reboiler kept: the cost of a furnace step's drift.
        let mw: f64 = mw.parse().unwrap();
        engine.apply(Command::SetFurnaceDuty { node: furnace, duty: Watt(mw * 1e6) }).unwrap();
        for i in 0..8 {
            if !report(&format!("kept, furnace {mw}, block {i}"), &step(&mut engine, 1500)) { return; }
        }
        return;
    }
    let q0 = engine.snapshot().nodes.iter().find(|n| n.name == "column").unwrap().column_duty.unwrap().reboiler_w / 1e6;
    let mut held = q0;
    probe::set_held(Some(held));
    if !report(&format!("held {held:.2}"), &step(&mut engine, 20)) { return; }
    let share = env_f("PA_SHARE", 0.6);
    let base = [15.0, 15.0, 10.0];
    let total: f64 = base.iter().sum();
    let furnace0 = 20.0;
    let ticks = env_f("STEP_TICKS", 300.0) as u32;
    let dq = env_f("DQ", 5.0);
    while held > 0.0 {
        held = (held - dq).max(0.0);
        let removed = q0 - held;
        let pa_scale = (1.0 - share * removed / total).max(0.0);
        let from_pa = total * (1.0 - pa_scale);
        let fire = furnace0 + (removed - from_pa);
        for (i, d) in base.iter().enumerate() {
            engine.apply(Command::SetPumparoundDuty { node: column, pumparound: i as u32, duty: Watt(d * pa_scale * 1e6) }).unwrap();
        }
        engine.apply(Command::SetFurnaceDuty { node: furnace, duty: Watt(fire * 1e6) }).unwrap();
        probe::set_held(Some(held));
        if !report(&format!("reb {held:.1} pa {:.0}% ({:.1} MW) furnace {fire:.1}", pa_scale * 100.0, total * pa_scale), &step(&mut engine, ticks)) { return; }
    }
    if !report("reboiler 0, settle", &step(&mut engine, env_f("SETTLE", 3000.0) as u32)) { return; }
    for op in std::env::var("OPENINGS").unwrap_or_default().split(',').filter(|x| !x.is_empty()) {
        let o: f64 = op.parse().unwrap();
        engine.apply(Command::SetValveOpening { node: valve, opening: o }).unwrap();
        if !report(&format!("steam valve {o}"), &step(&mut engine, 600)) { return; }
    }
    for mw in std::env::var("FURNACE_WALK").unwrap_or_default().split(',').filter(|x| !x.is_empty()) {
        let mw: f64 = mw.parse().unwrap();
        engine.apply(Command::SetFurnaceDuty { node: furnace, duty: Watt(mw * 1e6) }).unwrap();
        if !report(&format!("furnace {mw}"), &step(&mut engine, 1500)) { return; }
    }
}
