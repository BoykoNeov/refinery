// M75.4 probes (DESIGN §80.7), not compiled: append to
// crates/scenarios/tests/column_overfill.rs (they use its helpers, `to_corner`
// among them) and run with `-- --ignored --nocapture`. See FINDINGS.md.

/// PROBE (M75.4, removed before commit): the stories the screen's keys allow.
#[test]
#[ignore]
fn probe_overfill_stories() {
    use refinery_core::snapshot::Command;
    let story = std::env::var("STORY").unwrap_or_else(|_| "drain".into());
    let minutes: u32 = std::env::var("MINUTES").ok().and_then(|s| s.parse().ok()).unwrap_or(20);
    let mut engine = engine_of(OVERFILL_PLANT).unwrap();
    let column = engine.graph.find_node("column").unwrap();
    run(&mut engine, 600);
    let send = |engine: &mut Engine, c: Command| engine.apply(c).unwrap();
    match story.as_str() {
        "drain" => {
            send(&mut engine, Command::SetSlopDrain { node: column, open: false });
            send(&mut engine, Command::SetRefluxRatio { node: column, reflux_ratio: 0.5 });
        }
        "drain_only" => send(&mut engine, Command::SetSlopDrain { node: column, open: false }),
        "reflux_only" => send(&mut engine, Command::SetRefluxRatio { node: column, reflux_ratio: 0.5 }),
        "total" => send(&mut engine, Command::SetTotalReflux { node: column, on: true }),
        "total_drain" => {
            send(&mut engine, Command::SetSlopDrain { node: column, open: false });
            send(&mut engine, Command::SetTotalReflux { node: column, on: true });
        }
        "reflux_high" => {
            send(&mut engine, Command::SetSlopDrain { node: column, open: false });
            send(&mut engine, Command::SetRefluxRatio { node: column, reflux_ratio: 15.0 });
        }
        "reboiler" => {
            let mw: f64 = std::env::var("MW").ok().and_then(|s| s.parse().ok()).unwrap_or(120.0);
            send(&mut engine, Command::SetReboilerDuty { node: column, duty: Some(refinery_core::units::Watt(mw * 1e6)) });
        }
        "reboiler_drain" => {
            let mw: f64 = std::env::var("MW").ok().and_then(|s| s.parse().ok()).unwrap_or(120.0);
            send(&mut engine, Command::SetSlopDrain { node: column, open: false });
            send(&mut engine, Command::SetReboilerDuty { node: column, duty: Some(refinery_core::units::Watt(mw * 1e6)) });
        }
        other => panic!("{other}"),
    }
    let mut first_design = None;
    let mut first_ceiling = None;
    let mut peak: f64 = 0.0;
    let mut vent_max: f64 = 0.0;

    for tick in 1..=minutes * 600 {
        let s = engine
            .tick()
            .map(|_| engine.snapshot())
            .unwrap_or_else(|e| panic!("{story} tick {tick}: {e}"));
        let h = reading(&s);
        let c = h.condenser.clone().unwrap();
        let v = vent(&s);
        peak = peak.max(c.pressure_pa);
        vent_max = vent_max.max(v.flow_kg_s);
        if c.at_design && first_design.is_none() {
            first_design = Some(tick);
        }
        if c.at_ceiling && first_ceiling.is_none() {
            first_ceiling = Some(tick);
        }
        if tick % 600 == 0 {
            println!(
                "{story} {:>3} min: P {:.4} bar  vent {:6.2} kg/s  open {:.3}  drum {:.3} {:?}  flooded {:.3}  base {:.3} {:?}  cap {:.1} MW  {}{}{}",
                tick / 600,
                c.pressure_pa / 1e5,
                v.flow_kg_s,
                v.opening,
                h.drum_level,
                h.drum_limit,
                c.flooded_share,
                h.base_level,
                h.base_limit,
                c.capacity_w / 1e6,
                if c.at_capacity { "CAP " } else { "" },
                if c.at_design { "RELIEF " } else { "" },
                if c.at_ceiling { "CEILING" } else { "" },
            );
        }
    }
    println!(
        "{story}: peak {:.4} bar, relief from {:?} s, ceiling from {:?} s, vent max {:.2} kg/s",
        peak / 1e5,
        first_design.map(|t| t as f64 * 0.1),
        first_ceiling.map(|t| t as f64 * 0.1),
        vent_max
    );
}

/// PROBE (removed before commit).
#[test]
#[ignore]
fn probe_drain_story() {
    let shut: bool = std::env::var("SHUT").map(|s| s == "1").unwrap_or(true);
    let mut engine = engine_of(OVERFILL_PLANT).unwrap();
    let column = engine.graph.find_node("column").unwrap();
    run(&mut engine, 600);
    engine.apply(Command::SetReboilerDuty { node: column, duty: None }).unwrap();
    run(&mut engine, 2400);
    if shut {
        engine.apply(Command::SetSlopDrain { node: column, open: false }).unwrap();
    }
    engine.apply(Command::SetRefluxRatio { node: column, reflux_ratio: 0.5 }).unwrap();
    let mut peak: f64 = 0.0;
    for tick in 1..=12000 {
        if tick == 9001 && shut {
            engine.apply(Command::SetSlopDrain { node: column, open: true }).unwrap();
        }
        let s = run(&mut engine, 1);
        let h = reading(&s);
        let c = h.condenser.clone().unwrap();
        let v = vent(&s);
        peak = peak.max(c.pressure_pa);
        if tick % 300 == 0 {
            println!("{:5.1} min: P {:.4} bar vent {:6.2} drum {:.3} {:?} atmax {} flooded {:.3} base {:.3} cap {:.1} MW {}{}{}",
                tick as f64 / 600.0, c.pressure_pa / 1e5, v.flow_kg_s, h.drum_level, h.drum_limit, h.drum_loop_at_max, c.flooded_share, h.base_level,
                c.capacity_w / 1e6, if c.at_capacity {"CAP "} else {""}, if c.at_design {"RELIEF "} else {""}, if c.at_ceiling {"CEILING"} else {""});
        }
    }
    println!("peak {:.4}", peak / 1e5);
}

/// PROBE: one corner.
#[test]
#[ignore]
fn probe_one_corner() {
    let mut engine = engine_of(OVERFILL_PLANT).unwrap();
    run(&mut engine, 600);
    to_corner(&mut engine, 0.0, 0.5, Some(0.0), (0.0, 60.0));
    for tick in 1..=600 {
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        if tick % 50 == 0 || tick > 480 {
            let s = engine.snapshot();
            let op = s
                .nodes
                .iter()
                .find(|n| n.name == "column")
                .unwrap()
                .column_operation
                .clone()
                .unwrap();
            eprintln!(
                "TICK {tick} P {:.1} vapour {:?} T13 {:.2}",
                op.holdup.unwrap().condenser.unwrap().pressure_pa,
                op.stage_vapour_mol_s
                    .iter()
                    .map(|v| (v * 10.0).round() / 10.0)
                    .collect::<Vec<_>>(),
                op.stage_temperatures_k[12]
            );
        }
    }
}

/// PROBE (removed before commit): the holdup plant's key box at its corners,
/// each with the cooling cut or total reflux.
#[test]
#[ignore]
fn probe_box_corners() {
    use refinery_core::units::{Kelvin, KgPerSec, Watt};
    let flows = [85.0, 95.0, 65.0];
    let returns = [150.0, 215.0, 270.0];
    let mut corners = Vec::new();
    for event in ["cooling", "total"] {
        for furnace in [0.0, 40.0] {
            for reflux in [0.5, 15.0] {
                for reboiler in [Some(0.0), Some(215.0), None] {
                    for (share, dk) in [(0.0, 60.0), (2.0, -60.0)] {
                        corners.push((event, furnace, reflux, reboiler, share, dk));
                    }
                }
            }
        }
    }
    let threads: usize = std::env::var("THREADS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(12);
    let next = std::sync::atomic::AtomicUsize::new(0);
    let lines = std::sync::Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| loop {
                let i = next.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                let Some(&(event, furnace, reflux, reboiler, share, dk)) = corners.get(i) else { break };
                let plain = std::env::var("PLAIN").is_ok();
                if plain && event == "total" { continue; }
                let big = std::env::var("BIG").is_ok();
                if big && !(matches!((furnace, reflux, reboiler, share), (0.0, 0.5, Some(0.0), 0.0) | (40.0, 15.0, None, 0.0)) && event == "cooling") { continue; }
                let mut engine = if plain { engine_of(&holdup_with(", steel = \"derived\"")).unwrap() } else if big { engine_of(&OVERFILL_PLANT.replace("margin = 0.05,", "margin = 10.0,")).unwrap() } else { engine_of(OVERFILL_PLANT).unwrap() };
                let column = engine.graph.find_node("column").unwrap();
                let furnace_id = engine.graph.find_node("furnace").unwrap();
                let mut outcome = String::from("ran");
                let (mut peak, mut vent_max, mut ceiling): (f64, f64, u32) = (0.0, 0.0, 0);
                let mut go = || -> Result<(), String> {
                    let mut tick = |engine: &mut Engine, n: u32, t0: u32| -> Result<(), String> {
                        for t in 0..n {
                            engine.tick().map_err(|e| format!("tick {}: {e}", t0 + t))?;
                            let s = engine.snapshot();
                            let Some(c) = reading(&s).condenser else { continue };
                            peak = peak.max(c.pressure_pa);
                            let v = s.nodes.iter().find(|n| n.name == "column").unwrap().column_operation.as_ref().and_then(|o| o.relief.as_ref()).map_or(0.0, |r| r.flow_kg_s);
                            vent_max = vent_max.max(v);
                            ceiling += c.at_ceiling as u32;
                        }
                        Ok(())
                    };
                    tick(&mut engine, 600, 0)?;
                    let e = |r: Result<(), refinery_core::error::SimError>| r.map_err(|e| e.to_string());
                    e(engine.apply(Command::SetFurnaceDuty { node: furnace_id, duty: Watt(furnace * 1e6) }))?;
                    e(engine.apply(Command::SetRefluxRatio { node: column, reflux_ratio: reflux }))?;
                    e(engine.apply(Command::SetReboilerDuty { node: column, duty: reboiler.map(|mw: f64| Watt(mw * 1e6)) }))?;
                    for l in 0..3 {
                        e(engine.apply(Command::SetLoopFlow { node: column, pumparound_loop: l as u32, flow: KgPerSec(flows[l] * share) }))?;
                        e(engine.apply(Command::SetLoopReturnTemperature { node: column, pumparound_loop: l as u32, temperature: Kelvin(returns[l] + 273.15 + dk) }))?;
                    }
                    tick(&mut engine, 3000, 600)?;
                    if plain || big {
                    } else if event == "cooling" {
                        e(engine.apply(Command::SetCondenserCooling { node: column, on: false }))?;
                    } else {
                        e(engine.apply(Command::SetTotalReflux { node: column, on: true }))?;
                    }
                    tick(&mut engine, 6000, 3600)?;
                    if plain || big {
                    } else if event == "cooling" {
                        e(engine.apply(Command::SetCondenserCooling { node: column, on: true }))?;
                    } else {
                        e(engine.apply(Command::SetTotalReflux { node: column, on: false }))?;
                    }
                    tick(&mut engine, 3000, 9600)?;
                    Ok(())
                };
                if let Err(e) = go() {
                    outcome = format!("REFUSED {e}");
                }
                lines.lock().unwrap().push(format!(
                    "{event:8} furnace {furnace:4} reflux {reflux:4} reboiler {:>5} loops {share}/{dk:+}: peak {:.3} bar vent {:6.1} ceiling {:5} ticks — {}",
                    reboiler.map_or("free".to_string(), |m| format!("{m}")), peak / 1e5, vent_max, ceiling, outcome.chars().take(300).collect::<String>()
                ));
            });
        }
    });
    let mut lines = lines.into_inner().unwrap();
    lines.sort();
    for l in &lines {
        println!("{l}");
    }
}

/// PROBE (removed before commit): random walks over the key box with the
/// cooling, total reflux and the drain toggled.
#[test]
#[ignore]
fn probe_random_walks() {
    use refinery_core::units::{Kelvin, KgPerSec, Watt};
    let runs: u64 = std::env::var("RUNS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(24);
    let minutes: u32 = std::env::var("MINUTES")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(30);
    let threads: usize = std::env::var("THREADS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(12);
    let next = std::sync::atomic::AtomicU64::new(0);
    let lines = std::sync::Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| loop {
                let seed = next.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                if seed >= runs {
                    break;
                }
                if let Ok(only) = std::env::var("SEED") {
                    if only.parse::<u64>().unwrap() != seed {
                        continue;
                    }
                }
                let mut rng =
                    0x9E37_79B9_7F4A_7C15u64 ^ (seed.wrapping_mul(0x2545_F491_4F6C_DD1D) + 1);
                let mut rand = move || {
                    rng ^= rng << 13;
                    rng ^= rng >> 7;
                    rng ^= rng << 17;
                    rng
                };
                let mut engine = engine_of(OVERFILL_PLANT).unwrap();
                let column = engine.graph.find_node("column").unwrap();
                let heater = engine.graph.find_node("furnace").unwrap();
                let (mut furnace, mut reflux, mut reboiler, mut flow, mut ret): (
                    f64,
                    f64,
                    Option<f64>,
                    f64,
                    f64,
                ) = (f64::NAN, 4.0, Some(86.7), 1.0, 0.0);
                let (mut cooling_on, mut total, mut drain_open) = (true, false, true);
                let mut log = Vec::new();
                let mut outcome = "ran".to_string();
                let (mut peak, mut ceiling): (f64, u32) = (0.0, 0);
                'run: for minute in 0..minutes {
                    for t in 0..600 {
                        if let Err(e) = engine.tick() {
                            outcome = format!(
                                "REFUSED min {minute} tick {t}: {}",
                                e.to_string().chars().take(200).collect::<String>()
                            );
                            break 'run;
                        }
                        let s = engine.snapshot();
                        if let Some(c) = s
                            .nodes
                            .iter()
                            .find(|n| n.name == "column")
                            .unwrap()
                            .column_operation
                            .as_ref()
                            .and_then(|o| o.holdup.as_ref())
                            .and_then(|h| h.condenser.as_ref())
                        {
                            peak = peak.max(c.pressure_pa);
                            ceiling += c.at_ceiling as u32;
                        }
                    }
                    if furnace.is_nan() {
                        furnace = 20.0;
                    }
                    let jump = rand() % 2 == 0;
                    let pick = |r: u64, lo: f64, hi: f64, step: f64| {
                        lo + ((r % (((hi - lo) / step) as u64 + 1)) as f64) * step
                    };
                    let action = rand() % 9;
                    let sign = if rand() % 2 == 0 { 1.0 } else { -1.0 };
                    let r = rand();
                    let cmd = match action {
                        0 => {
                            furnace = if jump {
                                pick(r, 0.0, 40.0, 5.0)
                            } else {
                                (furnace + 5.0 * sign).clamp(0.0, 40.0)
                            };
                            log.push(format!("F{furnace}"));
                            Command::SetFurnaceDuty {
                                node: heater,
                                duty: Watt(furnace * 1e6),
                            }
                        }
                        1 => {
                            reflux = if jump {
                                pick(r, 0.5, 15.0, 0.5)
                            } else {
                                (reflux + 0.5 * sign).clamp(0.5, 15.0)
                            };
                            log.push(format!("R{reflux}"));
                            Command::SetRefluxRatio {
                                node: column,
                                reflux_ratio: reflux,
                            }
                        }
                        2 => {
                            reboiler = if r % 4 == 0 {
                                None
                            } else if jump || reboiler.is_none() {
                                Some(pick(r / 4, 0.0, 215.0, 2.5))
                            } else {
                                Some((reboiler.unwrap() + 2.5 * sign).clamp(0.0, 215.0))
                            };
                            log.push(format!("B{reboiler:?}"));
                            Command::SetReboilerDuty {
                                node: column,
                                duty: reboiler.map(|m| Watt(m * 1e6)),
                            }
                        }
                        3 => {
                            flow = if jump {
                                pick(r, 0.0, 2.0, 0.25)
                            } else {
                                (flow + 0.25 * sign).clamp(0.0, 2.0)
                            };
                            log.push(format!("L{flow}"));
                            for (i, f) in [85.0, 95.0, 65.0].into_iter().enumerate() {
                                if let Err(e) = engine.apply(Command::SetLoopFlow {
                                    node: column,
                                    pumparound_loop: i as u32,
                                    flow: KgPerSec(f * flow),
                                }) {
                                    outcome = format!("CMD {e}");
                                    break 'run;
                                }
                            }
                            continue;
                        }
                        4 => {
                            ret = if jump {
                                pick(r, -60.0, 60.0, 20.0)
                            } else {
                                (ret + 20.0 * sign).clamp(-60.0, 60.0)
                            };
                            log.push(format!("T{ret:+}"));
                            for (i, c) in [150.0, 215.0, 270.0].into_iter().enumerate() {
                                if let Err(e) = engine.apply(Command::SetLoopReturnTemperature {
                                    node: column,
                                    pumparound_loop: i as u32,
                                    temperature: Kelvin(c + 273.15 + ret),
                                }) {
                                    outcome = format!("CMD {e}");
                                    break 'run;
                                }
                            }
                            continue;
                        }
                        5 | 6 => {
                            cooling_on = !cooling_on;
                            log.push(format!("C{}", cooling_on as u8));
                            Command::SetCondenserCooling {
                                node: column,
                                on: cooling_on,
                            }
                        }
                        7 => {
                            total = !total;
                            log.push(format!("X{}", total as u8));
                            Command::SetTotalReflux {
                                node: column,
                                on: total,
                            }
                        }
                        _ => {
                            drain_open = !drain_open;
                            log.push(format!("D{}", drain_open as u8));
                            Command::SetSlopDrain {
                                node: column,
                                open: drain_open,
                            }
                        }
                    };
                    if let Err(e) = engine.apply(cmd) {
                        outcome = format!("CMD {e}");
                        break 'run;
                    }
                }
                lines.lock().unwrap().push(format!(
                    "seed {seed:2}: peak {:.3} bar ceiling {ceiling:5} — {outcome} | {}",
                    peak / 1e5,
                    log.join(" ")
                ));
            });
        }
    });
    let mut lines = lines.into_inner().unwrap();
    lines.sort();
    for l in &lines {
        println!("{l}");
    }
}
