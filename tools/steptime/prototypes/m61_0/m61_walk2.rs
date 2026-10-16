//! THROWAWAY probe (M61.0): from a hot furnace, warm, walk the reflux and the
//! pumparounds down — can the reboiler be brought near zero inside today's solve?
use refinery_core::engine::Engine;
use refinery_core::snapshot::Command;
use refinery_core::units::Watt;
use refinery_scenarios::{build_engine, load_str};

fn step(engine: &mut Engine, ticks: u32) -> Result<String, String> {
    for _ in 0..ticks {
        engine.tick().map_err(|e| {
            let s = e.to_string();
            if s.contains("liquid leaving") {
                "trays run dry".into()
            } else if s.contains("converge") {
                "does not settle".into()
            } else {
                s[..400.min(s.len())].to_string()
            }
        })?;
    }
    let s = engine.snapshot();
    let c = s.nodes.iter().find(|n| n.name == "column").unwrap();
    let op = c.column_operation.clone().unwrap();
    let duty = c.column_duty.unwrap();
    Ok(format!(
        "R {:.2} cond {:.1} reb {:.1}",
        op.reflux_ratio,
        duty.condenser_w / 1e6,
        duty.reboiler_w / 1e6,
    ))
}

#[test]
fn probe_reboiler_to_zero() {
    let src =
        std::fs::read_to_string("crude_unit_hot.toml").unwrap();
    let fresh = || build_engine(&load_str(&src).unwrap()).unwrap();
    let base = fresh();
    let id = |n: &str| base.snapshot().nodes.iter().find(|x| x.name == n).unwrap().id;
    let (col, furnace) = (id("column"), id("furnace"));
    let base_pa = [15.0, 15.0, 10.0];
    for top_mw in [65.0, 80.0] {
        for (walk, settings) in [
            ("reflux", vec![3.0, 2.5, 2.0, 1.5, 1.0, 0.7, 0.5]),
            ("pumparounds", vec![0.75, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0]),
            ("both", vec![0.75, 0.6, 0.5, 0.4, 0.3, 0.2]),
        ] {
            let mut engine = fresh();
            let _ = step(&mut engine, 1500).unwrap();
            let mut mw = 20.0;
            while mw < top_mw {
                mw += 5.0;
                engine
                    .apply(Command::SetFurnaceDuty { node: furnace, duty: Watt(mw * 1e6) })
                    .unwrap();
                step(&mut engine, 800).unwrap();
            }
            let mut line = format!("{top_mw} MW, {walk:11} start {:?}:", step(&mut engine, 1).unwrap());
            for v in settings {
                let pa_scale = match walk {
                    "reflux" => 1.0,
                    _ => v,
                };
                let reflux = match walk {
                    "pumparounds" => 4.0,
                    "reflux" => v,
                    _ => 4.0 * v,
                };
                engine
                    .apply(Command::SetRefluxRatio { node: col, reflux_ratio: reflux })
                    .unwrap();
                for (i, d) in base_pa.iter().enumerate() {
                    engine
                        .apply(Command::SetPumparoundDuty {
                            node: col,
                            pumparound: i as u32,
                            duty: Watt(d * pa_scale * 1e6),
                        })
                        .unwrap();
                }
                let out = step(&mut engine, 800);
                line += &format!(
                    "\n    R{reflux:.2} pa{:.0}%: {}",
                    pa_scale * 100.0,
                    match &out {
                        Ok(s) => s.clone(),
                        Err(e) => format!("FAIL {e}"),
                    }
                );
                if out.is_err() {
                    break;
                }
            }
            eprintln!("{line}");
        }
    }
}
