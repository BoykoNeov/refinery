//! THROWAWAY probe (M60.3): warm steps of the crude unit's keys. Not committed.
use refinery_core::engine::Engine;
use refinery_core::snapshot::Command;
use refinery_core::units::Watt;
use refinery_scenarios::{build_engine, load_str};

fn try_step(engine: &mut Engine, ticks: u32) -> Result<String, String> {
    for _ in 0..ticks {
        engine.tick().map_err(|e| {
            let s = e.to_string();
            if s.contains("liquid leaving") {
                "dry".into()
            } else if s.contains("bottoms rate") {
                "bottoms".into()
            } else if s.contains("converge") {
                "no-converge".into()
            } else {
                s[..90.min(s.len())].to_string()
            }
        })?;
    }
    let s = engine.snapshot();
    let c = s.nodes.iter().find(|n| n.name == "column").unwrap();
    let feed = s.edges.iter().find(|e| e.name == "transfer_line__downstream").unwrap();
    Ok(format!(
        "R{:.2}/{:.0}C/q{:.2}",
        c.column_operation.unwrap().reflux_ratio,
        feed.stream.temperature.value() - 273.15,
        c.column_operation.unwrap().feed_vapour_fraction
    ))
}

#[test]
fn probe_warm_keys() {
    let src = std::fs::read_to_string("W:/temp/claude/m60-scripts/crude_unit_probe.toml").unwrap();
    let fresh = || build_engine(&load_str(&src).unwrap()).unwrap();
    let base = fresh();
    let id = |n: &str| base.snapshot().nodes.iter().find(|x| x.name == n).unwrap().id;
    let (col, furnace) = (id("column"), id("furnace"));
    let walks: Vec<(&str, Vec<f64>)> = vec![
        ("furnace MW down", vec![15.0, 10.0, 5.0, 0.0]),
        ("furnace MW up", vec![25.0, 30.0, 35.0, 40.0, 50.0]),
        ("reflux down", vec![3.0, 2.5, 2.0, 1.5]),
        ("reflux up", vec![6.0, 8.0, 10.0]),
        ("pumparounds down", vec![0.9, 0.8, 0.75, 0.6, 0.5]),
        ("pumparounds up", vec![1.5, 2.0, 3.0]),
    ];
    for (name, values) in walks {
        let mut engine = fresh();
        let start = try_step(&mut engine, 1500);
        let mut line = format!("{name:18} start {start:?}:");
        for v in values {
            if name.starts_with("furnace") {
                engine.apply(Command::SetFurnaceDuty { node: furnace, duty: Watt(v * 1e6) }).unwrap();
            } else if name.starts_with("reflux") {
                engine.apply(Command::SetRefluxRatio { node: col, reflux_ratio: v }).unwrap();
            } else {
                for (i, mw) in [15.0, 15.0, 10.0].iter().enumerate() {
                    engine
                        .apply(Command::SetPumparoundDuty { node: col, pumparound: i as u32, duty: Watt(mw * v * 1e6) })
                        .unwrap();
                }
            }
            let out = try_step(&mut engine, 1200);
            line += &format!("  {v}:{}", match &out { Ok(s) => s.clone(), Err(e) => format!("FAIL {e}") });
            if out.is_err() {
                break;
            }
        }
        eprintln!("{line}");
    }
}
