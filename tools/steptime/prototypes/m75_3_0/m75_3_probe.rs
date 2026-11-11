//! M75.3.0 PROBE ONLY — never committed. A relief valve on the column's top
//! (Kister §9.5, location A) inside the column's pressure solve; every tick
//! read. Run: `--test suite m75_3_probe:: --nocapture`; each story writes
//! W:/temp/claude/m75_3_0/<story>.csv.

use std::io::Write;

use refinery_core::engine::Engine;
use refinery_solvers::heat_balance::probe::{self, Relief};

use crate::column_condenser::{with_limit, PROBE};
use crate::column_empty::{engine_of, reading, run, set_total_reflux, state, Books};

const P_ATM: f64 = 1.01325e5;
/// The holdup plant's set 1.8 bar and derived design pressure, 3.52 bar.

const DESIGN: f64 = 1.8e5 + 1.72e5;
const R_GAS: f64 = 8.314_462_618;

fn env_f(k: &str, d: f64) -> f64 {
    std::env::var(k).ok().map_or(d, |v| v.parse().unwrap())
}

/// The relief per Ludwig 1B Fig. 9.7A (API RP-520): set at the design
/// pressure (the MAWP), a single valve's 10 % of the gauge MAWP to full
/// lift; to atmosphere.
fn relief(kv: f64, blowdown: bool, lagged: bool, capacity_scale: f64) -> Relief {
    let accumulation = 0.10 * (DESIGN - P_ATM);
    Relief {
        kv,
        set: DESIGN,
        accumulation,
        back: env_f("BACK", P_ATM),
        gamma: env_f("GAMMA", 1.1),
        x_t: env_f("XT", 0.7),
        // 7 % of the gauge set pressure (Fig. 9.7A's typical blowdown).
        blowdown: blowdown.then_some(0.07 * (DESIGN - P_ATM)),
        lagged,
        upper: DESIGN + 3.0 * accumulation,
        capacity_scale,
    }
}

/// The relief's Kv by hand that passes `w` kg/s at full lift at `p` with
/// the top at `t`, molar mass `m`: choked, `W = (2/3)·cv·√(998·x_c·p·ρ)`.
fn kv_for(w: f64, p: f64, t: f64, m: f64) -> f64 {
    let rho = p * m / (R_GAS * t);
    let x_c = env_f("GAMMA", 1.1) / 1.4 * env_f("XT", 0.7);
    let cv = w / (2.0 / 3.0 * (998.0 * x_c * p * rho).sqrt());
    cv * 3600.0 * 1e5_f64.sqrt()
}

#[derive(Default)]
struct Summary {
    p_min: f64,
    p_max: f64,
    vent_max: f64,
    lifts: u32,
    reseats: u32,
    openings: u32,
    worst_mass: f64,
    worst_heat: f64,
    most_trials: u32,
    vent_over_top: u32,
}

/// Run `ticks` with `config`, `setup` applied after `warm` ticks at rest.
fn story(
    name: &str,
    config: Relief,
    warm: u32,
    ticks: u32,
    setup: impl Fn(&mut Engine),
) -> (Engine, Summary) {
    let mut engine = engine_of(&with_limit(PROBE)).unwrap();
    probe::set(None);
    run(&mut engine, warm);
    probe::set(Some(config));
    setup(&mut engine);
    let books = Books::of(&engine);
    let mut csv = std::fs::File::create(format!("W:/temp/claude/m75_3_0/{name}.csv")).unwrap();
    writeln!(
        csv,
        "t_s,p_bar,vent_kg_s,opening,lifted,top_vapour_mol_s,vent_mol_s,trials,drum_level,capacity_mw,duty_mw,mass_books,heat_books_w"
    )
    .unwrap();
    let mut s = Summary {
        p_min: f64::INFINITY,
        ..Summary::default()
    };
    let (mut was_lifted, mut was_open) = (false, false);
    for tick in 0..ticks {
        let before = state(&engine);
        let snapshot = match engine.tick() {
            Ok(()) => engine.snapshot(),
            Err(e) => panic!("{name}: tick {tick} refused: {e}"),
        };
        let (mass, heat) = books.close(&engine, &before);
        let v = probe::last().unwrap_or_default();
        let op = reading(&snapshot);
        let condenser = op.condenser.clone().unwrap();
        let duty = snapshot
            .nodes
            .iter()
            .find(|n| n.name == "column")
            .unwrap()
            .column_duty
            .unwrap()
            .condenser_w;
        let p = condenser.pressure_pa;
        s.p_min = s.p_min.min(p);
        s.p_max = s.p_max.max(p);
        s.vent_max = s.vent_max.max(v.kg_s);
        s.worst_mass = s.worst_mass.max(mass.abs());
        s.worst_heat = s.worst_heat.max(heat.abs());
        s.most_trials = s.most_trials.max(v.trials);
        if v.mol_s > v.top_vapour_mol_s {
            s.vent_over_top += 1;
        }
        if v.lifted && !was_lifted {
            s.lifts += 1;
        }
        if !v.lifted && was_lifted {
            s.reseats += 1;
        }
        let open = v.opening > 0.0;
        if open && !was_open {
            s.openings += 1;
        }
        was_lifted = v.lifted;
        was_open = open;
        writeln!(
            csv,
            "{:.1},{:.6},{:.4},{:.5},{},{:.2},{:.2},{},{:.4},{:.3},{:.3},{:.3e},{:.3e}",
            (tick + 1) as f64 * 0.1,
            p / 1e5,
            v.kg_s,
            v.opening,
            v.lifted as u8,
            v.top_vapour_mol_s,
            v.mol_s,
            v.trials,
            op.drum_level,
            condenser.capacity_w / 1e6,
            duty / 1e6,
            mass,
            heat
        )
        .unwrap();
    }
    println!(
        "STORY {name}: P {:.4}..{:.4} bar | vent max {:.2} kg/s | pop lifts {} reseats {} | openings {} | books {:.2e} kg/s {:.1} W | trials ≤ {} | vent>top {}",
        s.p_min / 1e5,
        s.p_max / 1e5,
        s.vent_max,
        s.lifts,
        s.reseats,
        s.openings,
        s.worst_mass,
        s.worst_heat,
        s.most_trials,
        s.vent_over_top
    );
    (engine, s)
}

fn sized_kv() -> f64 {
    env_f("KV", 0.0)
}

/// S0, sizing: the coolant lost, a relief far too big (it holds the
/// pressure just above set) — the vapour it then vents is what the
/// governing case asks; Kv for full lift at set + accumulation by hand.
#[test]
fn m75_3_probe_s0_sizing() {
    let (engine, _) = story("s0_sizing", relief(1.0e5, false, false, 0.0), 100, env_f("S0_TICKS", 1800.0) as u32, |_| {});
    let v = probe::last().unwrap();
    let st = state(&engine);
    let comp = refinery_solvers::molar::MoleFractions::from_mass(&st.drum_composition, &engine.slate)
        .unwrap();
    let m: f64 = comp
        .fractions()
        .iter()
        .enumerate()
        .map(|(c, x)| x * engine.slate.get(c).molar_mass.value())
        .sum();
    let t = st.stage_temperature[0].value();
    let r = relief(0.0, false, false, 0.0);
    let p_full = r.set + r.accumulation;
    println!(
        "SIZING: vent {:.2} kg/s ({:.1} mol/s; top vapour {:.1} mol/s) at {:.4} bar, top {:.1} °C, M {:.4} kg/mol -> Kv at full lift {:.4} bar: {:.0}",
        v.kg_s,
        v.mol_s,
        v.top_vapour_mol_s,
        v.pressure / 1e5,
        t - 273.15,
        m,
        p_full / 1e5,
        kv_for(v.kg_s, p_full, t, m)
    );
}

/// S1: the coolant lost, the sized relief, modulating.
#[test]
fn m75_3_probe_s1_coolant_lost() {
    story("s1_lost", relief(sized_kv(), false, false, 0.0), 100, 3000, |_| {});
}

/// S2: the coolant lost, the sized relief, pop with 7 % blowdown.
#[test]
fn m75_3_probe_s2_coolant_lost_pop() {
    story("s2_lost_pop", relief(sized_kv(), true, false, 0.0), 100, 3000, |_| {});
}

/// S3: total reflux with the feed on (M74.6.0's mistake), the sized
/// relief, modulating.
#[test]
fn m75_3_probe_s3_total_reflux() {
    let ticks = env_f("TR_TICKS", 6000.0) as u32;
    story("s3_tr", relief(sized_kv(), false, false, 1.0), 100, ticks, |e| {
        set_total_reflux(e, true).unwrap()
    });
}

/// S4: S3 with the pop valve.
#[test]
fn m75_3_probe_s4_total_reflux_pop() {
    let ticks = env_f("TR_TICKS", 6000.0) as u32;
    story("s4_tr_pop", relief(sized_kv(), true, false, 1.0), 100, ticks, |e| {
        set_total_reflux(e, true).unwrap()
    });
}

/// S5: S1 with the vent read at last tick's pressure (tick-late).
#[test]
fn m75_3_probe_s5_coolant_lost_lagged() {
    story("s5_lost_lagged", relief(sized_kv(), false, true, 0.0), 100, 3000, |_| {});
}

/// S6: S2 (pop) tick-late.
#[test]
fn m75_3_probe_s6_coolant_lost_pop_lagged() {
    story("s6_lost_pop_lagged", relief(sized_kv(), true, true, 0.0), 100, 3000, |_| {});
}
