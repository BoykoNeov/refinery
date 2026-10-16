//! THROWAWAY M62.0 probe walk.
use refinery_core::engine::Engine;
use refinery_core::snapshot::Command;
use refinery_core::units::Watt;
use refinery_scenarios::{build_engine, load_str};
use refinery_solvers::probe::{self, Steam};

fn step(engine: &mut Engine, ticks: u32) -> Result<String, String> {
    let clock = std::time::Instant::now();
    let mut worst = 0.0f64;
    for _ in 0..ticks {
        let t = std::time::Instant::now();
        engine.tick().map_err(|e| {
            let s = e.to_string();
            s[..700.min(s.len())].to_string()
        })?;
        worst = worst.max(t.elapsed().as_secs_f64() * 1e3);
    }
    let mean = clock.elapsed().as_secs_f64() * 1e3 / ticks as f64;
    let s = engine.snapshot();
    let c = s.nodes.iter().find(|n| n.name == "column").unwrap();
    let op = c.column_operation.clone().unwrap();
    let duty = c.column_duty.unwrap();
    let fu = s.nodes.iter().find(|n| n.name == "furnace").unwrap();
    let kind = format!("{:?}", fu.kind);
    let coil_t = kind.find("FurnaceCoil").and_then(|i| kind[i..].find("temperature: Kelvin(").map(|j| i + j + 20)).map(|i| kind[i..].split(')').next().unwrap().parse::<f64>().unwrap() - 273.15).unwrap_or(f64::NAN);
    let coil = if std::env::var("DUMP").is_ok() { format!("{:?}", fu) } else { kind.find("coil").map(|i| kind[i..(i + 60).min(kind.len())].to_string()).unwrap_or_default() };
    let coil = if coil.is_empty() || std::env::var("DUMP").is_err() { format!("coil {coil_t:.1}C") } else { coil };
    Ok(format!(
        "feed {:.1}C flash {:.3} | R {:.3} cond {:.2} reb(closure) {:.2} MW | {coil} | tick mean {mean:.2} worst {worst:.2} ms\n        {}",
        fu.temperature_k - 273.15,
        op.feed_vapour_fraction,
        op.reflux_ratio,
        duty.condenser_w / 1e6,
        duty.reboiler_w / 1e6,
        probe::last()
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

fn id(engine: &Engine, name: &str) -> refinery_core::graph::NodeId {
    engine.snapshot().nodes.iter().find(|x| x.name == name).unwrap().id
}

#[test]
fn walk() {
    let src = std::fs::read_to_string(std::env::var("PLANT").unwrap()).unwrap();
    let top_mw: f64 = std::env::var("TOP_MW").map(|v| v.parse().unwrap()).unwrap_or(65.0);
    let steams: Vec<f64> = std::env::var("STEAMS").unwrap_or("1,2,4".into()).split(',').map(|v| v.parse().unwrap()).collect();
    if let Ok(cold) = std::env::var("COLD") {
        let kg: f64 = cold.parse().unwrap();
        probe::set(Steam { kg_s: kg, inlet_c: 400.0, held_mw: Some(0.0) });
        let mut engine = build_engine(&load_str(&src).unwrap()).unwrap();
        let ok = report("cold tick 1", &step(&mut engine, 1)) && report("cold 1500", &step(&mut engine, 1500));
        if ok { walk_after(&mut engine, &steams, kg); }
        return;
    }
    probe::set(Steam { kg_s: 0.0, inlet_c: 400.0, held_mw: None });
    let mut engine = build_engine(&load_str(&src).unwrap()).unwrap();
    let furnace = id(&engine, "furnace");
    report("start", &step(&mut engine, 1500));
    let mut mw = 20.0;
    while mw < top_mw {
        mw = (mw + 5.0).min(top_mw);
        engine.apply(Command::SetFurnaceDuty { node: furnace, duty: Watt(mw * 1e6) }).unwrap();
        if !report(&format!("furnace {mw}"), &step(&mut engine, 800)) { return; }
    }
    let column = id(&engine, "column");
    let pa_top: f64 = std::env::var("PA_TOP").map(|v| v.parse().unwrap()).unwrap_or(10.0);
    let mut pa = 0.0;
    while pa < pa_top {
        pa = (pa + 2.5).min(pa_top);
        engine.apply(Command::SetPumparoundDuty { node: column, pumparound: 0, duty: Watt(pa * 1e6) }).unwrap();
        if !report(&format!("top pumparound {pa}"), &step(&mut engine, 300)) { return; }
    }
    let reb = {
        let s = engine.snapshot();
        s.nodes.iter().find(|n| n.name == "column").unwrap().column_duty.unwrap().reboiler_w / 1e6
    };
    // Hold the reboiler where it stands, then add steam, then walk the reboiler down.
    let mut held = reb;
    probe::set(Steam { kg_s: 0.0, inlet_c: 400.0, held_mw: Some(held) });
    if !report(&format!("held {held:.2}, no steam"), &step(&mut engine, 50)) { return; }
    let first_steam = steams[0];
    for s in [0.1, 0.25, 0.5, first_steam].into_iter().filter(|s| *s <= first_steam) {
        probe::set(Steam { kg_s: s, inlet_c: 400.0, held_mw: Some(held) });
        if !report(&format!("held {held:.2}, steam {s}"), &step(&mut engine, 50)) { return; }
    }
    let mut targets: Vec<f64> = vec![];
    let mut t = held;
    while t > 10.0 { t -= 5.0; targets.push(t.max(0.0)); }
    targets.extend([8.0, 6.0, 4.0, 2.0, 1.0, 0.0]);
    for target in targets {
        if target >= held { continue; }
        held = target;
        let share: f64 = std::env::var("PA_SHARE").map(|v| v.parse().unwrap()).unwrap_or(0.0);
        let base = [pa_top, 15.0, 15.0, 10.0];
        let total: f64 = base.iter().sum();
        let scale = (1.0 - share * (reb - held) / total).max(0.0);
        for (i, d) in base.iter().enumerate() {
            engine.apply(Command::SetPumparoundDuty { node: column, pumparound: i as u32, duty: Watt(d * scale * 1e6) }).unwrap();
        }
        eprintln!("    pumparounds at {:.0} % ({:.1} MW total)", scale * 100.0, total * scale);
        probe::set(Steam { kg_s: first_steam, inlet_c: 400.0, held_mw: Some(held) });
        if !report(&format!("reboiler {held:.1}, steam {first_steam}"), &step(&mut engine, 50)) { return; }
    }
    report("reboiler 0, settled", &step(&mut engine, 500));
    walk_after(&mut engine, &steams[1..], first_steam);
}

fn walk_after(engine: &mut Engine, steams: &[f64], mut steam: f64) {
    let furnace = id(engine, "furnace");
    let column = id(engine, "column");
    for s in steams {
        steam = *s;
        probe::set(Steam { kg_s: *s, inlet_c: 400.0, held_mw: Some(0.0) });
        if !report(&format!("reboiler 0, steam {s}"), &step(engine, 300)) { return; }
    }
    let list = |k: &str| -> Vec<f64> { std::env::var(k).map(|v| v.split(',').filter(|x| !x.is_empty()).map(|x| x.parse().unwrap()).collect()).unwrap_or_default() };
    for mw in list("FURNACE_WALK") {
        engine.apply(Command::SetFurnaceDuty { node: furnace, duty: Watt(mw * 1e6) }).unwrap();
        if !report(&format!("steam {steam}, furnace {mw}"), &step(engine, 800)) { return; }
    }
    // Pumparound walk: "i:MW" pairs.
    for pair in std::env::var("PA_WALK").unwrap_or_default().split(',').filter(|x| !x.is_empty()) {
        let (i, mw) = pair.split_once(':').unwrap();
        engine.apply(Command::SetPumparoundDuty { node: column, pumparound: i.parse().unwrap(), duty: Watt(mw.parse::<f64>().unwrap() * 1e6) }).unwrap();
        if !report(&format!("steam {steam}, pumparound {i} at {mw}"), &step(engine, 400)) { return; }
    }
}
