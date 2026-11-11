//! M75.1 — a column's condenser as a limit, and the pressure that rises past
//! it (docs/DESIGN.md §80.1): the holdup plant (`crude_unit_holdup.toml`,
//! untouched) with its steel and a `condenser_limit` declared inline.

use refinery_core::graph::{ColumnCondenser, NodeKind};
use refinery_core::snapshot::Snapshot;
use refinery_core::units::{CubicMeter, Kelvin, Pascal};

use crate::column_empty::{
    engine_of, holdup, holdup_with, reading, refused, run, set_total_reflux, state, Books,
    HOLDUP_LINE, HOLDUP_PLANT,
};

/// The holdup line with its steel, which every plant here declares.
const STEEL_LINE: &str =
    "holdup = { drum_m3 = 18.7, base_m3 = 115.4, reboiler = \"held\", steel = \"derived\" }";

/// The probe's condenser: 5 % spare over the settled duty, 35 °C coolant,
/// 100 m³ overhead.
const PROBE: &str = "margin = 0.05, coolant_c = 35.0, overhead_m3 = 100.0";

/// The holdup plant with its steel and the condenser `limit` (the keys
/// inside `condenser_limit = { … }`).
fn with_limit(limit: &str) -> String {
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

/// On total reflux the column asks its condenser more than it condenses, so
/// the pressure rises — read EVERY tick: once off the set pressure it never
/// returns to it and never steps back down (the pressure solved within the
/// tick, not carried a tick late: the tick-late update rang between the set
/// and the design pressure, which every-100th-tick sampling hid). The books
/// close every tick; the mass to 1e-7 kg/s, not the 1e-8 of a column at its
/// one pressure: while the pressure climbs the step reads up to ~4.5e-8 kg/s,
/// rising and falling with the rate of climb.
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
    assert!(mass < 1.0e-7, "{mass}");
    assert!(heat < 20.0, "{heat}");
}
