//! THROWAWAY M66.3 measurement: where the cold plant settles, and the start-up story (STORY=1).
//! Applied to `4e79120` (M66.2), registered in `crates/scenarios/tests/suite.rs`; PLANT=<file> [FLOW] [STORY] [TICKS] [REPORT].
use refinery_core::graph::NodeKind;

#[test]
fn zz_m66_3_measure() {
    let Ok(path) = std::env::var("PLANT") else { return };
    let mut src = std::fs::read_to_string(path).unwrap();
    if let Ok(flow) = std::env::var("FLOW") {
        src = src.replacen("flow = \"newton\"", &format!("flow = \"{flow}\""), 1);
    }
    let story = std::env::var("STORY").is_ok();
    let file = refinery_scenarios::load_str(&src).unwrap();
    let mut engine = refinery_scenarios::build_engine(&file).unwrap();
    let ticks: u64 = std::env::var("TICKS").map(|v| v.parse().unwrap()).unwrap_or(60_000);
    let every: u64 = std::env::var("REPORT").map(|v| v.parse().unwrap()).unwrap_or(5_000);
    let clock = std::time::Instant::now();
    let id = |e: &refinery_core::Engine, n: &str| e.snapshot().nodes.iter().find(|x| x.name == n).unwrap().id;
    let (furnace, column) = (id(&engine, "furnace"), id(&engine, "column"));
    let mut declined = 0u64;
    let mut zero_at = None;
    use refinery_core::snapshot::Command;
    use refinery_core::units::Watt;
    for t in 1..=ticks {
        if story {
            if t == 10 {
                engine.apply(Command::SetFurnaceDuty { node: furnace, duty: Watt(58.0e6) }).unwrap();
            }
            if t == 500 {
                for pumparound in 0..3 {
                    engine.apply(Command::SetPumparoundDuty { node: column, pumparound, duty: Watt::ZERO }).unwrap();
                }
            }
            if t >= 500 && t % 10 == 0 {
                let s = engine.snapshot();
                let run = s.nodes.iter().find(|x| x.name == "column").unwrap().column_duty.unwrap().reboiler_w;
                if run > 0.0 {
                    engine.apply(Command::SetReboilerDuty { node: column, duty: Some(Watt((run - 2.5e6).max(0.0))) }).unwrap();
                }
            }
        }
        let tick = std::time::Instant::now();
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        {
            let s = engine.snapshot();
            let c = s.nodes.iter().find(|x| x.name == "column").unwrap();
            if c.column_reboiler_declined.is_some() { declined += 1; }
            if story && zero_at.is_none() && c.column_duty.unwrap().reboiler_w == 0.0 {
                zero_at = Some(t);
                eprintln!("ZERO at t{t} after {declined} declined ticks");
            }
        }
        let ms = tick.elapsed().as_secs_f64() * 1e3;
        if t % every == 0 || t <= 3 {
            let s = engine.snapshot();
            let node = |n: &str| s.nodes.iter().find(|x| x.name == n).unwrap();
            let coil = match &node("furnace").kind {
                NodeKind::Furnace { coil, .. } => format!("{coil:?}"),
                other => format!("{other:?}"),
            };
            let opening = match &node("feed_valve").kind {
                NodeKind::Valve { opening, .. } => *opening,
                _ => f64::NAN,
            };
            let c = node("column");
            let op = c.column_operation.clone().unwrap();
            let charge = s.edges.iter().find(|e| e.name == "transfer_line").unwrap().stream.mass_flow.value();
            eprintln!(
                "t{t} ({ms:.1} ms, mean {:.2}) opening {opening:.6} charge {charge:.4} feed {:.3} reb {:.4} R {:.4} bottom {:.3} coil {}",
                clock.elapsed().as_secs_f64() * 1e3 / t as f64,
                node("furnace").temperature_k - 273.15,
                c.column_duty.unwrap().reboiler_w / 1e6,
                op.reflux_ratio,
                op.stage_temperatures_k.last().unwrap() - 273.15,
                &coil[..coil.len().min(160)]
            );
        }
    }
}
