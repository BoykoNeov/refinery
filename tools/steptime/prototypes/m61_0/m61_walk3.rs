//! THROWAWAY probe (M61.0, check B(b)): from a hot furnace, hold the reboiler's
//! duty at a target and let the reflux float — the shape steam stripping needs.
use refinery_core::engine::Engine;
use refinery_core::snapshot::Command;
use refinery_core::units::Watt;
use refinery_scenarios::{build_engine, load_str};

fn step(engine: &mut Engine, ticks: u32) -> Result<String, String> {
    for _ in 0..ticks {
        engine.tick().map_err(|e| {
            let s = e.to_string();
            s[..400.min(s.len())].to_string()
        })?;
    }
    let s = engine.snapshot();
    let c = s.nodes.iter().find(|n| n.name == "column").unwrap();
    let op = c.column_operation.clone().unwrap();
    let duty = c.column_duty.unwrap();
    let t = &op.stage_temperatures_k;
    let v = &op.stage_vapour_mol_s;
    Ok(format!(
        "R {:.3} cond {:.2} reb {:.2} MW; bottom vapour {:.1} mol/s; bottom {:.0}C",
        op.reflux_ratio,
        duty.condenser_w / 1e6,
        duty.reboiler_w / 1e6,
        v[v.len() - 1],
        t[t.len() - 1] - 273.15
    ))
}

#[test]
fn probe_reboiler_target() {
    let src =
        std::fs::read_to_string("crude_unit_hot.toml").unwrap();
    for top_mw in [65.0, 80.0] {
        std::env::set_var("PROBE_SPEC", "auto");
        let mut engine = build_engine(&load_str(&src).unwrap()).unwrap();
        let furnace = engine
            .snapshot()
            .nodes
            .iter()
            .find(|x| x.name == "furnace")
            .unwrap()
            .id;
        step(&mut engine, 1500).unwrap();
        let mut mw = 20.0;
        while mw < top_mw {
            mw += 5.0;
            engine
                .apply(Command::SetFurnaceDuty { node: furnace, duty: Watt(mw * 1e6) })
                .unwrap();
            step(&mut engine, 800).unwrap();
        }
        let mut line = format!("{top_mw} MW start {:?}", step(&mut engine, 1).unwrap());
        for target in [30.0, 20.0, 10.0, 5.0, 2.0, 1.0, 0.5, 0.2, 0.0] {
            std::env::set_var("PROBE_SPEC", format!("reb:{target}"));
            let out = step(&mut engine, 400);
            line += &format!(
                "\n    reboiler held at {target} MW: {}",
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
