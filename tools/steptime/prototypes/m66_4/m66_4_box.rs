//! THROWAWAY M66.4 measurement: the cold plant's keys, measured from the running plant (RESULTS.md).
//! Applied to `dcd100d` (M66.3). The start-up story (furnace 58 MW at tick 10, the
//! pumparounds off at 500, the reboiler down 2.5 MW from its running duty every
//! 10 ticks from 500 to WALK_END) plus EVENTS `tick:kind:value`, kinds furnace
//! MW, steam opening, pa (share of 15/15/10), free, down, up.
use refinery_core::snapshot::Command;
use refinery_core::units::Watt;

const COLD: &str = include_str!("../../../scenarios/crude_unit_cold.toml");

#[test]
fn zz_m66_4_box() {
    let Ok(events) = std::env::var("EVENTS") else { return };
    let story = std::env::var("NOSTORY").is_err();
    let walk_end: u64 = std::env::var("WALK_END").map(|v| v.parse().unwrap()).unwrap_or(2_400);
    let ticks: u64 = std::env::var("TICKS").map(|v| v.parse().unwrap()).unwrap_or(3_000);
    let file = refinery_scenarios::load_str(COLD).unwrap();
    let mut engine = refinery_scenarios::build_engine(&file).unwrap();
    let id = |e: &refinery_core::Engine, n: &str| e.snapshot().nodes.iter().find(|x| x.name == n).unwrap().id;
    let (furnace, column, steam) = (id(&engine, "furnace"), id(&engine, "column"), id(&engine, "steam_valve"));
    let events: Vec<(u64, String, f64)> = events
        .split(',')
        .filter(|e| !e.is_empty())
        .map(|e| {
            let p: Vec<&str> = e.split(':').collect();
            (p[0].parse().unwrap(), p[1].to_string(), p.get(2).map(|v| v.parse().unwrap()).unwrap_or(0.0))
        })
        .collect();
    let running = |e: &refinery_core::Engine| {
        e.snapshot().nodes.iter().find(|x| x.name == "column").unwrap().column_duty.unwrap().reboiler_w
    };
    let mut walking = story;
    for t in 1..=ticks {
        let mut cmds = Vec::new();
        if story && t == 10 {
            cmds.push(Command::SetFurnaceDuty { node: furnace, duty: Watt(58.0e6) });
        }
        if story && t == 500 {
            for pumparound in 0..3 {
                cmds.push(Command::SetPumparoundDuty { node: column, pumparound, duty: Watt::ZERO });
            }
        }
        if walking && t >= 500 && t <= walk_end && t % 10 == 0 && running(&engine) > 0.0 {
            cmds.push(Command::SetReboilerDuty { node: column, duty: Some(Watt((running(&engine) - 2.5e6).max(0.0))) });
        }
        for (at, kind, v) in &events {
            if *at != t {
                continue;
            }
            match kind.as_str() {
                "furnace" => cmds.push(Command::SetFurnaceDuty { node: furnace, duty: Watt(v * 1e6) }),
                "steam" => cmds.push(Command::SetValveOpening { node: steam, opening: *v }),
                "pa" => {
                    for (i, d) in [15.0e6, 15.0e6, 10.0e6].iter().enumerate() {
                        cmds.push(Command::SetPumparoundDuty { node: column, pumparound: i as u32, duty: Watt(d * v) });
                    }
                }
                "free" => {
                    walking = false;
                    cmds.push(Command::SetReboilerDuty { node: column, duty: None });
                }
                "down" => cmds.push(Command::SetReboilerDuty { node: column, duty: Some(Watt((running(&engine) - 2.5e6).max(0.0))) }),
                "up" => {
                    walking = false;
                    cmds.push(Command::SetReboilerDuty { node: column, duty: Some(Watt(running(&engine) + 2.5e6)) })
                }
                k => panic!("{k}"),
            }
            let s = engine.snapshot();
            let fe = s.nodes.iter().find(|x| x.name == "furnace").unwrap().temperature_k - 273.15;
            eprintln!("BOX @t{t} {kind}:{v} feed {fe:.1} reb {:.3} MW", running(&engine) / 1e6);
        }
        for c in cmds {
            engine.apply(c).unwrap_or_else(|e| panic!("t{t}: command refused: {e}"));
        }
        if let Err(e) = engine.tick() {
            let s = e.to_string();
            eprintln!("BOX HALT t{t}: {}", &s[..s.len().min(300)]);
            return;
        }
    }
    let s = engine.snapshot();
    let c = s.nodes.iter().find(|x| x.name == "column").unwrap();
    let op = c.column_operation.clone().unwrap();
    eprintln!(
        "BOX OK t{ticks} reb {:.3} MW declined {} R {:.3} bottom {:.1} feed {:.1}",
        c.column_duty.unwrap().reboiler_w / 1e6,
        c.column_reboiler_declined.is_some(),
        op.reflux_ratio,
        op.stage_temperatures_k.last().unwrap() - 273.15,
        s.nodes.iter().find(|x| x.name == "furnace").unwrap().temperature_k - 273.15
    );
}
