//! M75.2 — the reflux drum's drain to slop an operator's, and a drum past
//! full with it shut backing up into the condenser (docs/DESIGN.md §80.3):
//! the holdup plant (`crude_unit_holdup.toml`, untouched) with its steel and
//! the M75.1 `condenser_limit` declared inline.

use refinery_core::engine::Engine;
use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::traits::{LevelLimit, SlopVessel};
use refinery_core::units::Kg;

use crate::column_condenser::{with_limit, PROBE};
use crate::column_empty::{
    engine_of, holdup, holdup_with, reading, run, state, state_mut, Books, HOLDUP_PLANT,
};

/// The holdup plant's drum [m³] and its timestep [s].
const DRUM_M3: f64 = 18.7;
const DT: f64 = 0.1;

/// The condenser's liquid volume by hand [m³] from its `UA` [W/K]: the area
/// at Ludwig Vol. 1's 850 W/m²·K (Rules of Thumb, Heat Exchangers rule 9,
/// p. xxii), the shell side's free volume per tube area on rule 3's 19.0 mm
/// tubes on a 25.4 mm triangle — `((√3/2)·p² − π·d²/4)/(π·d)`, 4.61 L/m².
fn condenser_m3(ua: f64) -> f64 {
    let (d, p) = (0.019, 0.0254);
    let pi = std::f64::consts::PI;
    let per_area = (3.0f64.sqrt() / 2.0 * p * p - pi * d * d / 4.0) / (pi * d);
    assert!((per_area / 4.61e-3 - 1.0).abs() < 1.0e-3, "{per_area}");
    ua / 850.0 * per_area
}

pub(crate) fn set_drain(engine: &mut Engine, open: bool) -> Result<(), String> {
    let node = engine.graph.find_node("column").unwrap();
    engine
        .apply(Command::SetSlopDrain { node, open })
        .map_err(|e| e.to_string())
}

/// The drum's liquid scaled so the next tick reads its level at about
/// `level` (its composition and temperature unmoved; the level read at the
/// start of the last tick taken as the drum's now).
fn plant_drum(engine: &mut Engine, now: f64, level: f64) {
    let held = state_mut(engine);
    held.drum_liquid = Kg(held.drum_liquid.value() * level / now);
}

/// The column's slop this tick: (vessel, kg/s).
fn slop(snapshot: &Snapshot) -> Vec<(SlopVessel, f64)> {
    let node = snapshot.nodes.iter().find(|n| n.name == "column").unwrap();
    node.column_operation
        .as_ref()
        .unwrap()
        .slop
        .iter()
        .map(|s| (s.vessel, s.flow_kg_s))
        .collect()
}

/// A condenser with its settled duty to spare again: a drum past full runs
/// its level loop at its ceiling, the reflux up and the duty with it — about
/// 60 MW at the set pressure on its first tick past full, a third over the
/// settled 45.2 and past the probe's 5 % — and this margin covers that surge
/// with its tubes clear, so what the back-up floods alone decides whether
/// the pressure holds.
const SPARE: &str = "margin = 1.0, coolant_c = 35.0, overhead_m3 = 100.0";

/// The holdup plant with its steel and the condenser `limit`, run 100 ticks
/// (10 s) to rest, its drum's level as last read.
fn running(limit: &str) -> (Engine, f64) {
    let mut engine = engine_of(&with_limit(limit)).unwrap();
    let level = reading(&run(&mut engine, 100)).drum_level;
    (engine, level)
}

/// **The drain is refused where it cannot run**: shut on a column whose
/// condenser is not a limit (no tubes to flood), either way on a column
/// that holds no liquid, on a node that is not a column. Open is taken
/// wherever it applies; shut is taken before the first tick and is on the
/// column's kind — off the wire while open.
#[test]
fn the_drain_is_refused_where_it_cannot_run() {
    let mut plain = engine_of(&holdup_with(", steel = \"derived\"")).unwrap();
    let error = set_drain(&mut plain, false).unwrap_err();
    assert!(error.contains("declares no condenser limit"), "{error}");
    assert!(!holdup(&plain).slop_drain_shut);
    set_drain(&mut plain, true).unwrap();

    let trays = include_str!("../../../scenarios/crude_unit_trays.toml");
    let mut quasi = engine_of(trays).unwrap();
    for open in [true, false] {
        let error = set_drain(&mut quasi, open).unwrap_err();
        assert!(error.contains("holds no liquid"), "{error}");
    }

    let mut engine = engine_of(&with_limit(PROBE)).unwrap();
    let furnace = engine.graph.find_node("furnace").unwrap();
    let error = engine
        .apply(Command::SetSlopDrain {
            node: furnace,
            open: false,
        })
        .unwrap_err()
        .to_string();
    assert!(error.contains("is not a column"), "{error}");
    let kind = |engine: &Engine| {
        let node = engine.graph.find_node("column").unwrap();
        serde_json::to_string(&engine.graph.node(node).kind).unwrap()
    };
    assert!(!kind(&engine).contains("slop_drain_shut"));
    set_drain(&mut engine, false).unwrap();
    assert!(holdup(&engine).slop_drain_shut);
    assert!(kind(&engine).contains("\"slop_drain_shut\":true"));
    let first = run(&mut engine, 1);
    assert!(reading(&first).drain_shut);
    set_drain(&mut engine, true).unwrap();
    assert!(!kind(&engine).contains("slop_drain_shut"));
    let open = run(&mut engine, 1);
    assert!(!serde_json::to_string(&reading(&open))
        .unwrap()
        .contains("drain_shut"));
    // The plant without the limit, and the plain holdup plant, take open.
    let mut bare = engine_of(HOLDUP_PLANT).unwrap();
    set_drain(&mut bare, true).unwrap();
}

/// **A shut drain on a drum not past full changes nothing**: the column at
/// rest with its drain shut from the start runs bit for bit as with it open,
/// but for the flag on its kind and its faceplate.
#[test]
fn a_shut_drain_on_a_drum_not_full_changes_nothing() {
    let mut shut = engine_of(&with_limit(PROBE)).unwrap();
    set_drain(&mut shut, false).unwrap();
    let mut open = engine_of(&with_limit(PROBE)).unwrap();
    let unflagged = |mut snapshot: Snapshot| {
        for node in &mut snapshot.nodes {
            if let NodeKind::Column {
                cascade: Some(spec),
                ..
            } = &mut node.kind
            {
                if let Some(holdup) = spec.holdup.as_mut() {
                    holdup.slop_drain_shut = false;
                }
            }
            if let Some(holdup) = node
                .column_operation
                .as_mut()
                .and_then(|op| op.holdup.as_mut())
            {
                holdup.drain_shut = false;
            }
        }
        serde_json::to_string(&snapshot).unwrap()
    };
    for tick in 0..150 {
        shut.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        open.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        let s = shut.snapshot();
        assert!(reading(&s).drain_shut, "tick {tick}");
        assert!(reading(&s).drum_level < 1.0, "tick {tick}");
        assert_eq!(state(&shut), state(&open), "tick {tick}");
        assert_eq!(unflagged(s), unflagged(open.snapshot()), "tick {tick}");
    }
}

/// **Shut, a drum past full backs up into the condenser** (Kister §13.3),
/// at rest — where the pressure would otherwise hold, so the back-up is all
/// that moves it. The share of the tubes flooded is the drum's liquid past
/// full over the condenser's liquid volume, by hand; the capacity the rest's,
/// by hand; nothing to slop, the drum reading `"backed_up"`. On the
/// condenser with its duty to spare again ([`SPARE`]), a fifth flooded
/// leaves the pressure at its set exactly, the duty within what the free
/// tubes condense; half flooded and it rises. (On the probe's 5 % even 3 %
/// flooded rose 124 Pa — the full drum's own reflux surge. A risen pressure
/// reads a duty near its capacity whatever the shortfall at the set: the
/// column answers ~93 kW a pascal.) The books close.
#[test]
fn a_shut_drain_backs_the_drum_up_into_the_condenser() {
    let set = 1.8e5;
    for (share, rises) in [(0.2, false), (0.5, true)] {
        let (mut engine, now) = running(SPARE);
        let books = Books::of(&engine);
        set_drain(&mut engine, false).unwrap();
        let ua = state(&engine).condenser_ua.unwrap().value();
        let volume = condenser_m3(ua);
        plant_drum(&mut engine, now, 1.0 + share * volume / DRUM_M3);
        let before = state(&engine);
        let s = run(&mut engine, 1);
        let read = reading(&s);
        assert!(read.drain_shut);
        assert_eq!(read.drum_limit, LevelLimit::BackedUp, "{share}");
        assert_eq!(slop(&s), vec![], "{share}");
        let faceplate = read.condenser.unwrap();
        let by_hand = (read.drum_level - 1.0) * DRUM_M3 / volume;
        assert!(
            (faceplate.flooded_share / by_hand - 1.0).abs() < 1.0e-12,
            "{share}: {} against {by_hand}",
            faceplate.flooded_share
        );
        assert!((by_hand / share - 1.0).abs() < 0.01, "{share}: {by_hand}");
        let drum_t = state(&engine).drum_temperature.value();
        let capacity = (1.0 - by_hand) * ua * (drum_t - 308.15);
        assert!(
            (faceplate.capacity_w / capacity - 1.0).abs() < 1.0e-9,
            "{share}: {} against {capacity}",
            faceplate.capacity_w
        );
        let (mass, heat) = books.close(&engine, &before);
        assert!(mass < 1.0e-8, "{share}: {mass}");
        assert!(heat.abs() < 20.0, "{share}: {heat}");
        let pressure = state(&engine).column_pressure.unwrap().value();
        let duty = s
            .nodes
            .iter()
            .find(|n| n.name == "column")
            .unwrap()
            .column_duty
            .unwrap()
            .condenser_w;
        println!(
            "M75_2 backed up {share}: level {:.4}, flooded {:.4}, capacity {:.3} MW, duty {:.3} MW, {pressure} Pa",
            read.drum_level,
            faceplate.flooded_share,
            faceplate.capacity_w / 1.0e6,
            duty / 1.0e6
        );
        if !rises {
            assert!(duty <= faceplate.capacity_w, "{share}: {duty}");
        }
        if rises {
            assert!(
                pressure > set && faceplate.at_capacity,
                "{share}: {pressure}"
            );
        } else {
            assert_eq!(pressure, set, "{share}");
            assert!(!faceplate.at_capacity);
        }
    }
}

/// **Past the condenser flooded whole, to slop with the drain shut**: the
/// drum planted past its own volume and the condenser's — the excess past
/// both leaves within the tick (the drum reading `"full"`, the drain shut),
/// the condenser flooded whole, its capacity nothing.
#[test]
fn past_the_condenser_flooded_whole_the_excess_goes_to_slop() {
    let (mut engine, now) = running(PROBE);
    let books = Books::of(&engine);
    set_drain(&mut engine, false).unwrap();
    let volume = condenser_m3(state(&engine).condenser_ua.unwrap().value());
    // ≈ 680 m² of tubes, ≈ 3.1 m³ — a sixth of the 18.7 m³ drum.
    assert!((volume - 3.1).abs() < 0.1, "{volume}");
    let whole = 1.0 + volume / DRUM_M3;
    plant_drum(&mut engine, now, whole + 0.05);
    let held = state(&engine).drum_liquid.value();
    let before = state(&engine);
    let s = run(&mut engine, 1);
    let read = reading(&s);
    assert!(read.drain_shut);
    assert_eq!(read.drum_limit, LevelLimit::Full);
    let faceplate = read.condenser.unwrap();
    assert_eq!(faceplate.flooded_share, 1.0);
    assert_eq!(faceplate.capacity_w, 0.0);
    assert!(faceplate.at_capacity);
    let out = slop(&s);
    assert_eq!(out.len(), 1, "{out:?}");
    assert_eq!(out[0].0, SlopVessel::Drum);
    let by_hand = held * (1.0 - whole / read.drum_level) / DT;
    assert!(
        (out[0].1 / by_hand - 1.0).abs() < 1.0e-4,
        "{} against {by_hand}",
        out[0].1
    );
    let (mass, heat) = books.close(&engine, &before);
    assert!(mass < 1.0e-8, "{mass}");
    assert!(heat.abs() < 20.0, "{heat}");
}

/// **Opened, the drain takes what it held at once** (§79.18's drain, all the
/// back-up in one tick): the drum backed up half the condenser, the pressure
/// risen; opened, the next tick sends the drum's excess past full to slop,
/// the tubes clear, and the pressure — turning at most once — falls back to
/// its set. The books close every tick.
#[test]
fn opened_the_drain_takes_the_back_up_at_once() {
    let (mut engine, now) = running(SPARE);
    let books = Books::of(&engine);
    set_drain(&mut engine, false).unwrap();
    let volume = condenser_m3(state(&engine).condenser_ua.unwrap().value());
    plant_drum(&mut engine, now, 1.0 + 0.5 * volume / DRUM_M3);
    let shut = run(&mut engine, 5);
    assert_eq!(reading(&shut).drum_limit, LevelLimit::BackedUp);
    let risen = state(&engine).column_pressure.unwrap().value();
    assert!(risen > 1.8e5, "{risen}");
    set_drain(&mut engine, true).unwrap();
    let held = state(&engine).drum_liquid.value();
    let before = state(&engine);
    let s = run(&mut engine, 1);
    let read = reading(&s);
    assert!(!read.drain_shut);
    assert_eq!(read.drum_limit, LevelLimit::Full);
    assert_eq!(read.condenser.unwrap().flooded_share, 0.0);
    let out = slop(&s);
    let by_hand = held * (1.0 - 1.0 / read.drum_level) / DT;
    assert_eq!(out.len(), 1, "{out:?}");
    // To 1e-4, as M74.6's slop (`column_empty`): set in moles at the tick's
    // starting composition, carried at its end's.
    assert!(
        (out[0].1 / by_hand - 1.0).abs() < 1.0e-4,
        "{out:?} {by_hand}"
    );
    let (mass, _) = books.close(&engine, &before);
    assert!(mass < 1.0e-8, "{mass}");
    let mut last = state(&engine).column_pressure.unwrap().value();
    let (mut rising, mut turns) = (last > risen, 0);
    let mut back = None;
    for tick in 0..3000 {
        let before = state(&engine);
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        let (mass, heat) = books.close(&engine, &before);
        assert!(mass < 1.0e-8, "tick {tick}: {mass}");
        assert!(heat.abs() < 20.0, "tick {tick}: {heat}");
        let pressure = state(&engine).column_pressure.unwrap().value();
        if rising && pressure < last - 1.0 || !rising && pressure > last + 1.0 {
            rising = !rising;
            turns += 1;
        }
        last = pressure;
        if pressure == 1.8e5 {
            back = Some(tick);
            break;
        }
    }
    println!("M75_2 opened: risen to {risen} Pa, back at the set on tick {back:?}, turns {turns}");
    assert!(turns <= 1, "the pressure turned {turns} times");
    assert!(back.is_some(), "still at {last} Pa");
}

/// **Backed up, read every tick** (the share of the tubes flooded read off
/// the start-of-tick drum, a slow state; the pressure solved within the
/// tick): the drum planted with half the condenser's volume past full, its
/// drain shut, at rest. Its level loop at its ceiling draws it down — the
/// share flooded falls every tick and nothing goes to slop; the pressure
/// rises while the free tubes cannot condense the duty, turns once and falls
/// back to its set, never stepping back up; the drum reads `"backed_up"`
/// until it is no longer past full. The books close every tick.
#[test]
fn a_backed_up_drum_drawn_down_read_every_tick() {
    let (mut engine, now) = running(SPARE);
    let books = Books::of(&engine);
    set_drain(&mut engine, false).unwrap();
    let volume = condenser_m3(state(&engine).condenser_ua.unwrap().value());
    plant_drum(&mut engine, now, 1.0 + 0.5 * volume / DRUM_M3);
    let set = 1.8e5;
    let (mut flooded, mut last) = (1.0f64, set);
    let (mut mass, mut heat) = (0.0f64, 0.0f64);
    let (mut turns, mut rising, mut peak) = (0, true, set);
    let mut clear = None;
    let mut back = None;
    for tick in 0..3000 {
        let before = state(&engine);
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        let (m, h) = books.close(&engine, &before);
        mass = mass.max(m.abs());
        heat = heat.max(h.abs());
        let s = engine.snapshot();
        assert_eq!(slop(&s), vec![], "tick {tick}");
        let read = reading(&s);
        let share = read.condenser.unwrap().flooded_share;
        assert!(share <= flooded, "tick {tick}: {flooded} → {share}");
        assert_eq!(
            read.drum_limit == LevelLimit::BackedUp,
            share > 0.0,
            "tick {tick}: {:?} at {share}",
            read.drum_limit
        );
        if share == 0.0 && clear.is_none() {
            clear = Some(tick);
        }
        let pressure = state(&engine).column_pressure.unwrap().value();
        if rising && pressure < last - 1.0 || !rising && pressure > last + 1.0 {
            rising = !rising;
            turns += 1;
        }
        peak = peak.max(pressure);
        flooded = share;
        last = pressure;
        if tick % 100 == 99 {
            println!(
                "M75_2 drawn down: t={:.0} s level {:.4} flooded {:.4} {:.4} bar",
                (tick + 1) as f64 * DT,
                read.drum_level,
                share,
                pressure / 1.0e5
            );
        }
        if pressure == set && clear.is_some() {
            back = Some(tick);
            break;
        }
    }
    println!(
        "M75_2 drawn down: clear on tick {clear:?}, peak {peak} Pa, back on tick {back:?}, turns {turns}; books {mass} kg/s, {heat} W"
    );
    assert!(peak > set + 1.0e3, "{peak}");
    assert!(turns <= 1, "the pressure turned {turns} times");
    assert!(
        clear.is_some() && back.is_some(),
        "{clear:?} {back:?} at {last}"
    );
    assert!(mass < 1.0e-8, "{mass}");
    assert!(heat < 20.0, "{heat}");
}

/// **A condenser kept flooded whole drives the pressure to its design**
/// (Kister §13.3: a fully flooded condenser condenses nothing; relief, M75.3,
/// not yet there to take it): the drum re-planted past its own volume and
/// the condenser's before every tick — at rest its level loop would draw it
/// back down — read every tick: nothing condensed by the free tubes, the
/// excess to slop, the pressure rising and never stepping back, landing on
/// the design pressure exactly (the bracket's clamp) and holding there,
/// `at_design`. The books close every tick.
#[test]
fn slow_a_condenser_kept_flooded_whole_rises_to_its_design() {
    let (mut engine, now) = running(PROBE);
    let books = Books::of(&engine);
    set_drain(&mut engine, false).unwrap();
    // The drum's mass per unit of level [kg], from the tick each read.
    let mut per_level = state(&engine).drum_liquid.value() / now;
    let design = holdup(&engine).condenser.unwrap().design_pressure.value();
    let volume = condenser_m3(state(&engine).condenser_ua.unwrap().value());
    let whole = 1.0 + volume / DRUM_M3;
    let mut last = 1.8e5f64;
    let (mut mass, mut heat) = (0.0f64, 0.0f64);
    let mut reached = None;
    for tick in 0..2000 {
        state_mut(&mut engine).drum_liquid = Kg(per_level * (whole + 0.01));
        let before = state(&engine);
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        let (m, h) = books.close(&engine, &before);
        mass = mass.max(m.abs());
        heat = heat.max(h.abs());
        let s = engine.snapshot();
        let read = reading(&s);
        per_level = before.drum_liquid.value() / read.drum_level;
        let faceplate = read.condenser.unwrap();
        assert_eq!(read.drum_limit, LevelLimit::Full, "tick {tick}");
        assert_eq!(faceplate.flooded_share, 1.0, "tick {tick}");
        assert_eq!(faceplate.capacity_w, 0.0, "tick {tick}");
        assert_eq!(slop(&s).len(), 1, "tick {tick}");
        let pressure = state(&engine).column_pressure.unwrap().value();
        assert!(pressure >= last - 1.0, "tick {tick}: {last} → {pressure}");
        assert!(pressure <= design, "tick {tick}: {pressure}");
        assert_eq!(faceplate.at_design, pressure >= design, "tick {tick}");
        if tick % 100 == 99 {
            println!(
                "M75_2 flooded whole: t={:.0} s {:.4} bar",
                (tick + 1) as f64 * DT,
                pressure / 1.0e5
            );
        }
        last = pressure;
        match reached {
            None if pressure == design => reached = Some(tick),
            Some(at) if tick >= at + 50 => break,
            Some(_) => assert_eq!(pressure, design, "tick {tick}"),
            None => {}
        }
    }
    println!("M75_2 flooded whole: at the design on tick {reached:?}; books {mass} kg/s, {heat} W");
    assert!(reached.is_some(), "still at {last} Pa");
    assert!(mass < 1.0e-8, "{mass}");
    assert!(heat < 20.0, "{heat}");
}
