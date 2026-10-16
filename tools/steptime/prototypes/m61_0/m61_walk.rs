//! THROWAWAY probe (M61.0): walk the crude unit's furnace up, warm, with its
//! reboiler kept — how hot a feed can today's heat-balance column take? Not committed.
use refinery_core::engine::Engine;
use refinery_core::snapshot::Command;
use refinery_core::units::Watt;
use refinery_scenarios::{build_engine, load_str};

fn step(engine: &mut Engine, ticks: u32) -> Result<String, String> {
    for _ in 0..ticks {
        engine.tick().map_err(|e| {
            let s = e.to_string();
            s[..160.min(s.len())].to_string()
        })?;
    }
    let s = engine.snapshot();
    let c = s.nodes.iter().find(|n| n.name == "column").unwrap();
    let op = c.column_operation.clone().unwrap();
    let duty = c.column_duty.unwrap();
    let feed = s
        .edges
        .iter()
        .find(|e| e.name == "transfer_line__downstream")
        .unwrap();
    let t = &op.stage_temperatures_k;
    Ok(format!(
        "feed {:.0}C flash {:.3} R {:.2} cond {:.1} reb {:.1} MW feedstage {:.0}C bottom {:.0}C",
        feed.stream.temperature.value() - 273.15,
        op.feed_vapour_fraction,
        op.reflux_ratio,
        duty.condenser_w / 1e6,
        duty.reboiler_w / 1e6,
        t[7] - 273.15,
        t[t.len() - 1] - 273.15
    ))
}

#[test]
fn probe_hot_feed_walk() {
    let path = std::env::var("M61_PLANT")
        .unwrap_or("crude_unit_hot.toml".into());
    let src = std::fs::read_to_string(path).unwrap();
    let mut engine = build_engine(&load_str(&src).unwrap()).unwrap();
    let furnace = engine
        .snapshot()
        .nodes
        .iter()
        .find(|x| x.name == "furnace")
        .unwrap()
        .id;
    eprintln!("start 20 MW: {:?}", step(&mut engine, 1500));
    let duties: Vec<f64> = std::env::var("M61_DUTIES")
        .unwrap_or("25,30,35,40,45,50,55,60,65,70,75,80".into())
        .split(',')
        .map(|v| v.parse().unwrap())
        .collect();
    for mw in duties {
        engine
            .apply(Command::SetFurnaceDuty {
                node: furnace,
                duty: Watt(mw * 1e6),
            })
            .unwrap();
        let out = step(&mut engine, 1500);
        eprintln!("{mw} MW: {out:?}");
        if out.is_err() {
            break;
        }
    }
}
