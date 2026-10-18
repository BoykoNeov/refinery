//! THROWAWAY M66.0 probe: the stripped crude unit started from a cold plant.
//!
//! The plant is `crude_unit_stripped.toml` with its reboiler FREE (no
//! `reboiler_duty_mw`), its furnace at FURNACE MW, its coil at COIL °C, its
//! pumparounds at PA (three MW values, `/`-joined) and its steam valve at STEAM.
//! The probe command `SetReboilerDuty` holds or frees the reboiler mid-run.
//!
//! Env: FURNACE, COIL, PA, STEAM, FLOW (simple|newton), TICKS (total), REPORT
//! (report every N ticks), EVENTS (`tick:kind:value`, `,`-joined; kinds furnace
//! MW, steam opening, pa MW (all three, scaled from PA's shares when PASHARE), pa0/pa1/pa2
//! MW, hold MW | `closure` | `free`, sp kg/s), RAMPS (`from:to:kind:target`,
//! kinds furnace, pa (all three, each to target×its PA share), hold (from the
//! closure at `from`), steam), AUTO (`hold` rule: from the tick the feed
//! reaches AUTO_T °C, hold at the closure and ramp to zero at AUTO_RATE MW/s),
//! COMPARE (tick at which a second engine holds at its closure; prints the next
//! ticks of both).
use refinery_core::engine::Engine;
use refinery_core::graph::{ControlledValue, LoopId, NodeId};
use refinery_core::snapshot::Command;
use refinery_core::units::{KgPerSec, Watt};
use refinery_scenarios::{build_engine, load_str};

const STRIPPED: &str = include_str!("../../../scenarios/crude_unit_stripped.toml");

fn env_s(k: &str, d: &str) -> String {
    std::env::var(k).unwrap_or_else(|_| d.to_string())
}
fn env_f(k: &str, d: f64) -> f64 {
    std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d)
}

fn rep(s: String, from: &str, to: &str) -> String {
    assert_eq!(s.matches(from).count(), 1, "{from}");
    s.replacen(from, to, 1)
}

fn pa_list() -> Vec<f64> {
    env_s("PA", "15/15/10").split('/').map(|x| x.parse().unwrap()).collect()
}

fn plant() -> String {
    let mut p = STRIPPED.to_string();
    if std::env::var("HELD").is_err() {
        p = rep(p, "reboiler_duty_mw = 0.0\n", "");
    }
    p = rep(p, "duty_mw = 58.0\n", &format!("duty_mw = {}\n", env_s("FURNACE", "20.0")));
    p = rep(
        p,
        "coil_temperature_c = 436.88\n",
        &format!("coil_temperature_c = {}\n", env_s("COIL", "322.05")),
    );
    let pa = pa_list();
    p = rep(
        p,
        "pumparounds = [{ stage = 4, duty_mw = 0.0 }, { stage = 6, duty_mw = 0.0 }, { stage = 7, duty_mw = 0.0 }]",
        &format!(
            "pumparounds = [{{ stage = 4, duty_mw = {} }}, {{ stage = 6, duty_mw = {} }}, {{ stage = 7, duty_mw = {} }}]",
            pa[0], pa[1], pa[2]
        ),
    );
    p = rep(p, "opening = 0.4\nx_t", &format!("opening = {}\nx_t", env_s("STEAM", "0.4")));
    if let Ok(t) = std::env::var("SOURCE_T") {
        p = rep(p, "temperature_c = 270.0
", &format!("temperature_c = {t}
"));
    }
    if let Ok(flow) = std::env::var("FLOW") {
        p = rep(p, "flow = \"newton\"", &format!("flow = \"{flow}\""));
    }
    p
}

fn id(engine: &Engine, name: &str) -> NodeId {
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

#[derive(Default, Clone, Copy)]
struct State {
    feed_c: f64,
    q: f64,
    reflux: f64,
    bottom_c: f64,
    cond_mw: f64,
    reb_mw: f64,
    coil_c: f64,
    charge: f64,
    steam: f64,
    furnace_mw: f64,
}

fn read(engine: &Engine) -> State {
    let s = engine.snapshot();
    let c = s.nodes.iter().find(|n| n.name == "column").unwrap();
    let fu = s.nodes.iter().find(|n| n.name == "furnace").unwrap();
    let kind = format!("{:?}", fu.kind);
    let coil = after(&kind[kind.find("coil").unwrap()..], "temperature: Kelvin(") - 273.15;
    let duty = after(&kind, "duty: Watt(") / 1e6;
    let edge = |n: &str| {
        s.edges
            .iter()
            .find(|e| e.name == n)
            .map(|e| e.stream.mass_flow.value())
            .unwrap_or(f64::NAN)
    };
    let (q, r, tb) = c
        .column_operation
        .as_ref()
        .map(|o| {
            (
                o.feed_vapour_fraction,
                o.reflux_ratio,
                o.stage_temperatures_k.last().copied().unwrap_or(f64::NAN) - 273.15,
            )
        })
        .unwrap_or((f64::NAN, f64::NAN, f64::NAN));
    State {
        feed_c: fu.temperature_k - 273.15,
        q,
        reflux: r,
        bottom_c: tb,
        cond_mw: c.column_duty.map_or(f64::NAN, |d| d.condenser_w / 1e6),
        reb_mw: c.column_duty.map_or(f64::NAN, |d| d.reboiler_w / 1e6),
        coil_c: coil,
        charge: edge("transfer_line"),
        steam: edge("stripping_steam"),
        furnace_mw: duty,
    }
}

fn line(t: u64, s: &State) -> String {
    format!(
        "t{t} fur {:.1} coil {:.2} feed {:.2} q {:.4} chg {:.3} stm {:.3} | R {:.4} bot {:.3} cond {:.3} reb {:.4}",
        s.furnace_mw, s.coil_c, s.feed_c, s.q, s.charge, s.steam, s.reflux, s.bottom_c, s.cond_mw, s.reb_mw
    )
}

struct Ids {
    furnace: NodeId,
    column: NodeId,
    steam: NodeId,
}

fn cmd(ids: &Ids, kind: &str, v: &str, closure: f64) -> Command {
    let f = || v.parse::<f64>().unwrap();
    let pa = pa_list();
    match kind {
        "furnace" => Command::SetFurnaceDuty { node: ids.furnace, duty: Watt(f() * 1e6) },
        "steam" => Command::SetValveOpening { node: ids.steam, opening: f() },
        "pa0" | "pa1" | "pa2" => Command::SetPumparoundDuty {
            node: ids.column,
            pumparound: kind[2..].parse().unwrap(),
            duty: Watt(f() * 1e6),
        },
        "hold" => Command::SetReboilerDuty {
            node: ids.column,
            duty: match v {
                "free" => None,
                "closure" => Some(Watt(closure * 1e6)),
                _ => Some(Watt(f() * 1e6)),
            },
        },
        "sp" => Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Flow { kg_per_s: KgPerSec(f()) },
        },
        _ => panic!("{kind} {pa:?}"),
    }
}

fn apply_all(engine: &mut Engine, ids: &Ids, kind: &str, v: f64, closure: f64) -> Result<(), String> {
    if kind == "pa" {
        let pa = pa_list();
        let total: f64 = pa.iter().sum();
        for i in 0..3 {
            let share = if total > 0.0 { pa[i] / total } else { 1.0 / 3.0 };
            let duty = if std::env::var("PAEQUAL").is_ok() { v / 3.0 } else { v * share };
            engine
                .apply(cmd(ids, &format!("pa{i}"), &format!("{duty}"), closure))
                .map_err(|e| e.to_string())?;
        }
        Ok(())
    } else {
        engine.apply(cmd(ids, kind, &format!("{v}"), closure)).map_err(|e| e.to_string())
    }
}

#[test]
fn m66_0_probe() {
    let src = plant();
    if std::env::var("PRINT").is_ok() {
        println!("{src}");
    }
    let build = || -> Result<Engine, String> {
        load_str(&src)
            .map_err(|e| e.to_string())
            .and_then(|f| build_engine(&f).map_err(|e| e.to_string()))
    };
    let mut engine = match build() {
        Ok(e) => e,
        Err(e) => {
            eprintln!("  LOAD {e}");
            return;
        }
    };
    let ids = Ids {
        furnace: id(&engine, "furnace"),
        column: id(&engine, "column"),
        steam: id(&engine, "steam_valve"),
    };

    // COMPARE: run to the tick, then one engine holds at its closure.
    if let Ok(at) = std::env::var("COMPARE") {
        let at: u64 = at.parse().unwrap();
        let mut other = build().unwrap();
        for _ in 0..at {
            engine.tick().unwrap();
            other.tick().unwrap();
        }
        let closure = read(&other).reb_mw;
        // The exact published closure, in watts.
        let exact = other.snapshot().nodes.iter().find(|n| n.name == "column").unwrap().column_duty.unwrap().reboiler_w;
        eprintln!("  closure {closure:.9} MW ({exact:.6} W)");
        other
            .apply(Command::SetReboilerDuty { node: ids.column, duty: Some(Watt(exact)) })
            .unwrap();
        for k in 0..env_f("COMPARE_TICKS", 5.0) as u64 {
            let c0 = refinery_solvers::probe6::counts();
            let t0 = std::time::Instant::now();
            engine.tick().unwrap();
            let a = t0.elapsed().as_secs_f64() * 1e3;
            let t1 = std::time::Instant::now();
            let r = other.tick();
            let b = t1.elapsed().as_secs_f64() * 1e3;
            let c1 = refinery_solvers::probe6::counts();
            if let Err(e) = r {
                eprintln!("  held FAIL {e}");
                return;
            }
            let (x, y) = (read(&engine), read(&other));
            let ta = engine.snapshot().nodes.iter().find(|n| n.name == "column").unwrap().column_operation.clone().unwrap().stage_temperatures_k;
            let tb = other.snapshot().nodes.iter().find(|n| n.name == "column").unwrap().column_operation.clone().unwrap().stage_temperatures_k;
            let dt = ta.iter().zip(&tb).map(|(p, q)| (p - q).abs()).fold(0.0, f64::max);
            eprintln!(
                "  +{k} free  {} ({a:.1} ms)\n     held  {} ({b:.1} ms) | dR {:.2e} dT {:.2e} K dreb {:.2e} MW | counts {:?}",
                line(at + k + 1, &x),
                line(at + k + 1, &y),
                (x.reflux - y.reflux).abs(),
                dt,
                (x.reb_mw - y.reb_mw).abs(),
                [c1[0] - c0[0], c1[1] - c0[1], c1[2] - c0[2], c1[3] - c0[3]],
            );
        }
        // Then free it again: the reverse.
        other.apply(Command::SetReboilerDuty { node: ids.column, duty: None }).unwrap();
        for k in 0..3u64 {
            let c0 = refinery_solvers::probe6::counts();
            engine.tick().unwrap();
            let t1 = std::time::Instant::now();
            let r = other.tick();
            let b = t1.elapsed().as_secs_f64() * 1e3;
            let c1 = refinery_solvers::probe6::counts();
            if let Err(e) = r {
                eprintln!("  freed FAIL {e}");
                return;
            }
            let (x, y) = (read(&engine), read(&other));
            eprintln!(
                "  freed +{k} dR {:.2e} dreb {:.2e} MW ({b:.1} ms) counts {:?}",
                (x.reflux - y.reflux).abs(),
                (x.reb_mw - y.reb_mw).abs(),
                [c1[0] - c0[0], c1[1] - c0[1], c1[2] - c0[2], c1[3] - c0[3]],
            );
        }
        return;
    }

    let total: u64 = env_s("TICKS", "3000").parse().unwrap();
    let every: u64 = env_s("REPORT", "300").parse().unwrap();
    let events: Vec<(u64, String, String)> = env_s("EVENTS", "")
        .split(',')
        .filter(|x| !x.is_empty())
        .map(|e| {
            let p: Vec<&str> = e.splitn(3, ':').collect();
            (p[0].parse().unwrap(), p[1].to_string(), p[2].to_string())
        })
        .collect();
    // from:to:kind:target
    let ramps: Vec<(u64, u64, String, f64)> = env_s("RAMPS", "")
        .split(',')
        .filter(|x| !x.is_empty())
        .map(|e| {
            let p: Vec<&str> = e.split(':').collect();
            (p[0].parse().unwrap(), p[1].parse().unwrap(), p[2].to_string(), p[3].parse().unwrap())
        })
        .collect();
    let mut ramp_start: Vec<Option<f64>> = vec![None; ramps.len()];
    let auto_t = std::env::var("AUTO_T").ok().map(|v| v.parse::<f64>().unwrap());
    let auto_rate = env_f("AUTO_RATE", 0.05); // MW per second
    let mut auto_held: Option<f64> = None;

    let mut block_ms = 0.0f64;
    let mut worst = 0.0f64;
    let mut worst_t = 0;
    let mut c_block = refinery_solvers::probe6::counts();
    let mut last = State::default();
    for t in 1..=total {
        let closure = last.reb_mw;
        for (at, kind, v) in &events {
            if *at == t {
                let r = if kind == "pa" {
                    apply_all(&mut engine, &ids, "pa", v.parse().unwrap(), closure)
                } else {
                    engine.apply(cmd(&ids, kind, v, closure)).map_err(|e| e.to_string())
                };
                eprintln!("  @t{t} {kind}:{v} {}", if r.is_ok() { "ok".into() } else { format!("REFUSED {r:?}") });
            }
        }
        for (i, (from, to, kind, target)) in ramps.iter().enumerate() {
            if t < *from || t > *to {
                continue;
            }
            let start = *ramp_start[i].get_or_insert(match kind.as_str() {
                "hold" => closure,
                "furnace" => last.furnace_mw,
                "pa" => pa_list().iter().sum::<f64>(),
                "steam" => env_f("STEAM", 0.4),
                k => panic!("{k}"),
            });
            let frac = (t - from) as f64 / (*to - *from).max(1) as f64;
            let v = start + (target - start) * frac;
            let r = match kind.as_str() {
                "hold" => engine
                    .apply(Command::SetReboilerDuty { node: ids.column, duty: Some(Watt(v.max(0.0) * 1e6)) })
                    .map_err(|e| e.to_string()),
                k => apply_all(&mut engine, &ids, k, v, closure),
            };
            if let Err(e) = r {
                eprintln!("  ramp {kind} REFUSED {e}");
                return;
            }
        }
        // DOWN = from:step_MW:every — the key: the running duty less a step.
        if let Ok(d) = std::env::var("DOWN") {
            let p: Vec<f64> = d.split(':').map(|x| x.parse().unwrap()).collect();
            if t as f64 >= p[0] && ((t as f64 - p[0]) as u64) % (p[2] as u64) == 0 && last.reb_mw > 0.0 {
                let v = (last.reb_mw - p[1]).max(0.0);
                engine
                    .apply(Command::SetReboilerDuty { node: ids.column, duty: Some(Watt(v * 1e6)) })
                    .unwrap();
            }
        }
        if let Some(threshold) = auto_t {
            if auto_held.is_none() && last.feed_c >= threshold {
                auto_held = Some(closure);
                eprintln!("  @t{t} AUTO hold from {closure:.3} MW (feed {:.2} C)", last.feed_c);
            }
            if let Some(h) = auto_held.as_mut() {
                *h = (*h - auto_rate * 0.1).max(0.0);
                engine
                    .apply(Command::SetReboilerDuty { node: ids.column, duty: Some(Watt(*h * 1e6)) })
                    .unwrap();
            }
        }
        if t > 1 && std::env::var("NO_RUNNING").is_err() {
            *refinery_solvers::probe6::RUNNING.lock().unwrap() = Some(last.reb_mw * 1e6);
        }
        refinery_solvers::probe6::DECLINED.store(false, std::sync::atomic::Ordering::Relaxed);
        let t0 = std::time::Instant::now();
        let r = engine.tick();
        let ms = t0.elapsed().as_secs_f64() * 1e3;
        block_ms += ms;
        if ms > worst {
            worst = ms;
            worst_t = t;
        }
        if let Err(e) = r {
            let s = e.to_string();
            eprintln!("  FAIL t{t} ({ms:.1} ms): {}", &s[..1500.min(s.len())]);
            eprintln!("  last {}", line(t - 1, &last));
            if let Ok(how) = std::env::var("RECOVER") {
                let back = engine.apply(cmd(&ids, "hold", &how, closure));
                eprintln!("  RECOVER hold:{how} {back:?}; engine tick now {}", engine.snapshot().tick);
                if std::env::var("RECOVER_ONCE").is_ok() { std::env::remove_var("RECOVER"); }
                continue;
            }
            return;
        }
        last = read(&engine);
        if t % every == 0 || t == total || t == 1 {
            let c = refinery_solvers::probe6::counts();
            let n = if t == 1 { 1 } else if t % every == 0 { every } else { t % every };
            eprintln!(
                "  {} | mean {:.2} worst {worst:.1}@{worst_t} ms | kept/failed {:?} | wf/cont/cold/halved +{:?}",
                line(t, &last),
                block_ms / n as f64,
                refinery_solvers::probe6::fallbacks(),
                [c[0] - c_block[0], c[1] - c_block[1], c[2] - c_block[2], c[3] - c_block[3]],
            );
            c_block = c;
            block_ms = 0.0;
            worst = 0.0;
        }
    }
}
