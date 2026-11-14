//! M75.1 — a column's condenser as a limit, and the pressure that rises past
//! it (docs/DESIGN.md §80.1): the holdup plant (`crude_unit_holdup.toml`,
//! untouched) with its steel and a `condenser_limit` declared inline.

use refinery_core::graph::{ColumnCondenser, NodeKind};
use refinery_core::snapshot::Snapshot;
use refinery_core::units::{CubicMeter, Kelvin, Kg, Pascal};

use crate::column_empty::{
    engine_of, holdup, holdup_with, reading, refused, run, set_total_reflux, state, state_mut,
    Books, HOLDUP_LINE, HOLDUP_PLANT,
};

/// The holdup line with its steel, which every plant here declares.
const STEEL_LINE: &str =
    "holdup = { drum_m3 = 18.7, base_m3 = 115.4, reboiler = \"held\", steel = \"derived\" }";

/// The probe's condenser: 5 % spare over the settled duty, 35 °C coolant,
/// 100 m³ overhead.
pub(crate) const PROBE: &str = "margin = 0.05, coolant_c = 35.0, overhead_m3 = 100.0";

/// The holdup plant with its steel and the condenser `limit` (the keys
/// inside `condenser_limit = { … }`).
pub(crate) fn with_limit(limit: &str) -> String {
    let steel = holdup_with(", steel = \"derived\"");
    assert!(steel.contains(STEEL_LINE));
    steel.replace(
        STEEL_LINE,
        &format!("{STEEL_LINE}\ncondenser_limit = {{ {limit} }}"),
    )
}

/// A snapshot with its condenser taken off — its declaration and state on
/// the column's kind, its faceplate — to compare with a plant that declares
/// none.
fn without_condenser(mut snapshot: Snapshot) -> String {
    for node in &mut snapshot.nodes {
        if let NodeKind::Column {
            cascade: Some(spec),
            ..
        } = &mut node.kind
        {
            if let Some(holdup) = spec.holdup.as_mut() {
                holdup.condenser = None;
                if let Some(held) = holdup.state.as_mut() {
                    held.column_pressure = None;
                    held.condenser_ua = None;
                }
            }
        }
        if let Some(holdup) = node
            .column_operation
            .as_mut()
            .and_then(|op| op.holdup.as_mut())
        {
            holdup.condenser = None;
        }
    }
    serde_json::to_string(&snapshot).unwrap()
}

/// The key is read as written: the coolant in kelvin, the set pressure the
/// column's declared 1.8 bar, the design pressure the larger of 110 % of it
/// and it plus 1.72 bar — here 3.52 bar.
#[test]
fn the_condenser_limit_is_read_as_written() {
    let engine = engine_of(&with_limit(PROBE)).unwrap();
    assert_eq!(
        holdup(&engine).condenser,
        Some(ColumnCondenser {
            margin: 0.05,
            coolant: Kelvin(35.0 + 273.15),
            overhead_volume: CubicMeter(100.0),
            set_pressure: Pascal(1.8e5),
            design_pressure: Pascal(1.8e5 + 1.72e5),
            relief: None,
            coolant_lost: false,
        })
    );
    // Absent, nothing.
    let plain = engine_of(&holdup_with(", steel = \"derived\"")).unwrap();
    assert_eq!(holdup(&plain).condenser, None);
}

#[test]
fn a_condenser_limit_is_refused_where_it_cannot_run() {
    let says = |limit: &str, words: &str| {
        let e = refused(&with_limit(limit));
        assert!(e.contains(words), "{limit}: {e}");
    };
    says(
        "margin = -0.1, coolant_c = 35.0, overhead_m3 = 100.0",
        "condenser_limit.margin",
    );
    says(
        "margin = 0.05, coolant_c = -300.0, overhead_m3 = 100.0",
        "condenser_limit.coolant_c",
    );
    says(
        "margin = 0.05, coolant_c = 35.0, overhead_m3 = 0.0",
        "condenser_limit.overhead_m3",
    );
    says(
        "margin = 0.05, coolant_c = 35.0, overhead_m3 = -1.0",
        "condenser_limit.overhead_m3",
    );
    says("margin = 0.05, coolant_c = 35.0", "overhead_m3");
    says(
        "margin = 0.05, coolant_c = 35.0, overhead_m3 = 100.0, area_m2 = 3.0",
        "area_m2",
    );
    // On a column that holds no liquid.
    let src = HOLDUP_PLANT.replace("\r\n", "\n");
    assert!(src.contains(HOLDUP_LINE));
    let e = refused(&src.replace(HOLDUP_LINE, &format!("condenser_limit = {{ {PROBE} }}")));
    assert!(e.contains("without holdup"), "{e}");
    // A coolant no colder than the condensate: the condenser cannot be sized.
    let e = engine_of(&with_limit(
        "margin = 0.05, coolant_c = 400.0, overhead_m3 = 100.0",
    ))
    .and_then(|mut engine| engine.tick().map_err(|e| e.to_string()))
    .unwrap_err();
    assert!(e.contains("cannot be sized"), "{e}");
}

/// Its `UA` by hand: `UA = Q₀·(1 + margin)/(T_b − T_coolant)` on the first
/// tick's duty and the drum's condensate temperature (the bubble point at
/// the set pressure, a total condenser's) — ≈ 0.58 MW/K on the holdup plant.
#[test]
fn the_condenser_is_sized_from_the_duty_it_settles_on() {
    let mut engine = engine_of(&with_limit(PROBE)).unwrap();
    let snapshot = run(&mut engine, 1);
    let duty = snapshot
        .nodes
        .iter()
        .find(|n| n.name == "column")
        .unwrap()
        .column_duty
        .unwrap()
        .condenser_w;
    let held = state(&engine);
    let by_hand = duty * 1.05 / (held.drum_temperature.value() - 308.15);
    let ua = held.condenser_ua.unwrap().value();
    assert!((ua / by_hand - 1.0).abs() < 1.0e-12, "{ua} {by_hand}");
    assert!((ua / 1.0e6 - 0.58).abs() < 0.01, "{ua}");
    // At the set pressure it holds the settled duty with the margin to spare.
    let faceplate = reading(&snapshot).condenser.unwrap();
    assert_eq!(faceplate.pressure_pa, 1.8e5);
    assert!((faceplate.capacity_w / (duty * 1.05) - 1.0).abs() < 1.0e-12);
    assert!(!faceplate.at_capacity && !faceplate.at_design);
}

/// **At rest the limit changes nothing**: a running column whose condenser
/// condenses what arrives runs at its set pressure exactly, every flow,
/// temperature and holdup bit-identical to the same plant without the limit.
#[test]
fn at_rest_the_limit_changes_nothing() {
    let mut limited = engine_of(&with_limit(PROBE)).unwrap();
    let mut plain = engine_of(&holdup_with(", steel = \"derived\"")).unwrap();
    for tick in 0..150 {
        limited
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        plain.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        let mut held = state(&limited);
        assert_eq!(held.column_pressure, Some(Pascal(1.8e5)), "tick {tick}");
        held.column_pressure = None;
        held.condenser_ua = None;
        assert_eq!(held, state(&plain), "tick {tick}");
        assert_eq!(
            without_condenser(limited.snapshot()),
            without_condenser(plain.snapshot()),
            "tick {tick}"
        );
    }
    let faceplate = reading(&limited.snapshot()).condenser.unwrap();
    assert!(!faceplate.at_capacity);
}

/// The overhead's vapour slows the rise: the column's own store of heat sets
/// the pace on the holdup plant's 100 m³, but an overhead a thousand times
/// that takes the uncondensed vapour's heat as well — after five seconds of
/// total reflux its pressure has risen measurably less.
#[test]
fn a_larger_overhead_rises_slower() {
    let risen = |overhead: &str| {
        let mut engine = engine_of(&with_limit(&format!(
            "margin = 0.05, coolant_c = 35.0, overhead_m3 = {overhead}"
        )))
        .unwrap();
        run(&mut engine, 100);
        set_total_reflux(&mut engine, true).unwrap();
        run(&mut engine, 50);
        state(&engine).column_pressure.unwrap().value() - 1.8e5
    };
    let (small, large) = (risen("100.0"), risen("100000.0"));
    assert!(small > 0.0, "{small}");
    assert!(large > 0.0 && large < 0.8 * small, "{small} {large}");
}

/// **A pressure above the set falls back to it** (the falling side of the
/// solve, and the flooded condenser taking the pressure back): a column at
/// rest whose pressure is put 0.2 bar above its set — its stages still
/// boiling at the set — falls every tick, lands on the set pressure exactly
/// and stays, its books closing.
#[test]
fn a_pressure_above_the_set_falls_back_to_it() {
    let mut engine = engine_of(&with_limit(PROBE)).unwrap();
    let books = Books::of(&engine);
    run(&mut engine, 100);
    let set = 1.8e5;
    state_mut(&mut engine).column_pressure = Some(Pascal(set + 2.0e4));
    let mut last = set + 2.0e4;
    let mut back = None;
    let mut mass = 0.0f64;
    for tick in 0..300 {
        let before = state(&engine);
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        mass = mass.max(books.close(&engine, &before).0.abs());
        let pressure = state(&engine).column_pressure.unwrap().value();
        assert!(
            pressure >= set && pressure <= last,
            "tick {tick}: {last} → {pressure} Pa"
        );
        match back {
            None if pressure == set => back = Some(tick),
            Some(at) => assert_eq!(pressure, set, "tick {tick}: off the set again from {at}"),
            None => {}
        }
        if back.is_some_and(|at| tick >= at + 50) {
            break;
        }
        last = pressure;
    }
    let back = back.expect("the pressure never came back to the set pressure");
    assert!(back > 0, "it fell 0.2 bar in one tick");
    assert!(!reading(&engine.snapshot()).condenser.unwrap().at_capacity);
    assert!(mass < 1.0e-8, "{mass}");
}

/// On total reflux the column asks its condenser more than it condenses, so
/// the pressure rises — read EVERY tick: once off the set pressure it never
/// returns to it and never steps back down (the pressure solved within the
/// tick, not carried a tick late: the tick-late update rang between the set
/// and the design pressure, which every-100th-tick sampling hid). Then total
/// reflux off: the pressure turns once, to falling. The books close every
/// tick throughout — the mass to 1e-8 kg/s, as at one pressure.
#[test]
fn slow_total_reflux_raises_the_pressure_without_ringing() {
    let mut engine = engine_of(&with_limit(PROBE)).unwrap();
    let books = Books::of(&engine);
    run(&mut engine, 100);
    let set = holdup(&engine).condenser.unwrap().set_pressure.value();
    assert_eq!(state(&engine).column_pressure.unwrap().value(), set);
    set_total_reflux(&mut engine, true).unwrap();
    let mut last = set;
    let mut left = false;
    let (mut mass, mut heat) = (0.0f64, 0.0f64);
    for tick in 0..600 {
        let before = state(&engine);
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        let (m, h) = books.close(&engine, &before);
        mass = mass.max(m.abs());
        heat = heat.max(h.abs());
        let pressure = state(&engine).column_pressure.unwrap().value();
        if left {
            assert!(
                pressure > set,
                "tick {tick}: the pressure fell back to the set pressure from {last} Pa"
            );
        }
        assert!(
            pressure >= last - 1.0,
            "tick {tick}: the pressure stepped down, {last} → {pressure} Pa"
        );
        left |= pressure > set;
        last = pressure;
    }
    assert!(left, "the pressure never left the set pressure");
    // A minute on, 1.89 bar in the probe: a smooth rise, the column's own
    // store of heat setting the pace.
    assert!(last > 1.85e5 && last < 1.95e5, "{last}");
    let faceplate = reading(&engine.snapshot()).condenser.unwrap();
    assert!(faceplate.at_capacity && !faceplate.at_design);
    assert_eq!(faceplate.pressure_pa, last);

    // Total reflux off: the overloaded column takes a while to unload, so
    // the pressure may climb on to one peak (≈ 2.02 bar, 100 s on) before it
    // falls — once, not back and forth.
    set_total_reflux(&mut engine, false).unwrap();
    let (mut peak, mut turns, mut rising) = (last, 0, true);
    for tick in 0..1500 {
        let before = state(&engine);
        engine
            .tick()
            .unwrap_or_else(|e| panic!("off, tick {tick}: {e}"));
        let (m, h) = books.close(&engine, &before);
        mass = mass.max(m.abs());
        heat = heat.max(h.abs());
        let pressure = state(&engine).column_pressure.unwrap().value();
        assert!(pressure > set, "off, tick {tick}: {pressure} Pa");
        if rising && pressure < last - 1.0 || !rising && pressure > last + 1.0 {
            rising = !rising;
            turns += 1;
        }
        peak = peak.max(pressure);
        last = pressure;
    }
    assert_eq!(
        turns, 1,
        "the pressure turned {turns} times after total reflux off"
    );
    assert!(!rising && last < peak - 1.0e3, "{peak} {last}");
    assert!(mass < 1.0e-8, "{mass}");
    assert!(heat < 20.0, "{heat}");
}

/// **A pressure that peaks and turns is followed from where it stands**
/// (the solve's bracket grown from last tick's pressure, docs/DESIGN.md
/// §80.2): the column at rest, its drum put at 1.2 of its volume — the excess
/// to slop on the next tick, the drum's level loop then at its ceiling, the
/// reflux and the duty up — so the pressure rises for minutes, peaks, and
/// falls back to its set. Every tick converges: a bracket's far end tried as
/// a whole step from 2.39 bar to the set 1.8 bar flashed the column and
/// refused the tick at the peak (stage 13 short by 12 MW, tick 1 951). The
/// pressure turns once; the books close every tick.
#[test]
fn slow_a_peaking_pressure_turns_without_a_refused_tick() {
    let mut engine = engine_of(&with_limit(PROBE)).unwrap();
    let books = Books::of(&engine);
    let now = reading(&run(&mut engine, 100)).drum_level;
    let held = state_mut(&mut engine);
    held.drum_liquid = Kg(held.drum_liquid.value() * 1.2 / now);
    let set = 1.8e5;
    let (mut last, mut peak, mut rising, mut turns) = (set, set, true, 0);
    let (mut mass, mut heat) = (0.0f64, 0.0f64);
    let mut back = None;
    for tick in 0..8000 {
        let before = state(&engine);
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        let (m, h) = books.close(&engine, &before);
        mass = mass.max(m.abs());
        heat = heat.max(h.abs());
        let pressure = state(&engine).column_pressure.unwrap().value();
        if rising && pressure < last - 1.0 || !rising && pressure > last + 1.0 {
            rising = !rising;
            turns += 1;
        }
        peak = peak.max(pressure);
        last = pressure;
        if tick % 500 == 499 {
            println!(
                "M75_1 peaking: t={:.0} s {:.4} bar",
                (tick + 1) as f64 * 0.1,
                pressure / 1.0e5
            );
        }
        if !rising && pressure == set {
            back = Some(tick);
            break;
        }
    }
    println!("M75_1 peaking: peak {peak} Pa, back on tick {back:?}, turns {turns}; books {mass} kg/s, {heat} W");
    assert!(peak > 2.3e5, "{peak}");
    assert_eq!(turns, 1, "the pressure turned {turns} times");
    assert!(back.is_some(), "still at {last} Pa");
    assert!(mass < 1.0e-8, "{mass}");
    assert!(heat < 20.0, "{heat}");
}

/// **A small overhead peaks without a refused tick** (the bracket's first
/// trial capped at about the most a tick moves, docs/DESIGN.md §80.2): on
/// 0.01 m³ the overhead's store is ~1 W/Pa, so the root's bound `|r|/store`
/// is the whole span — the far end again. The same peaking story as above
/// runs past the peak (2.39 bar at ~200 s) every tick converging; uncapped,
/// tick 1 929 was refused.
#[test]
fn slow_a_small_overhead_peaks_without_a_refused_tick() {
    let mut engine = engine_of(&with_limit(
        "margin = 0.05, coolant_c = 35.0, overhead_m3 = 0.01",
    ))
    .unwrap();
    let now = reading(&run(&mut engine, 100)).drum_level;
    let held = state_mut(&mut engine);
    held.drum_liquid = Kg(held.drum_liquid.value() * 1.2 / now);
    let mut peak = 0.0f64;
    for tick in 0..2500 {
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        peak = peak.max(state(&engine).column_pressure.unwrap().value());
    }
    let last = state(&engine).column_pressure.unwrap().value();
    assert!(peak > 2.3e5 && last < peak - 1.0e3, "{peak} {last}");
}
