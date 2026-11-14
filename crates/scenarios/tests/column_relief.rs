//! M75.3.1 — a relief valve on a column's top, before the condenser
//! (docs/DESIGN.md §80.5): the holdup plant (`crude_unit_holdup.toml`,
//! untouched) with its steel, the M75.1 `condenser_limit` and its `relief`
//! declared inline — since M75.3.2 (§80.6) a `relief_valve` node on the map,
//! its line from the column to a flare.

use std::collections::BTreeMap;

use refinery_core::engine::Engine;
use refinery_core::graph::{ColumnRelief, NodeKind};
use refinery_core::snapshot::{ReliefOperation, Snapshot};
use refinery_core::units::{Kelvin, Pascal, R_GAS};

use crate::column_condenser::{with_limit, PROBE};
use crate::column_empty::{engine_of, holdup, reading, run, state, Books, FLARE_PIPE};

/// The holdup plant's set pressure and its derived design pressure [Pa abs],
/// and the atmosphere the flare is at.
pub(crate) const SET: f64 = 1.8e5;
pub(crate) const DESIGN: f64 = 1.8e5 + 1.72e5;
pub(crate) const P_ATM: f64 = 101_325.0;
pub(crate) const X_T: f64 = 0.7;
/// The relief valve's node, its line's pipe in and its flare (M75.3.2).
pub(crate) const VALVE: &str = "column_psv";
pub(crate) const INLET_PIPE: &str = "relief_inlet";
pub(crate) const FLARE: &str = "flare";

/// The relief valve's datasheet keys by the book (§80.4): set at the design
/// pressure, full lift 10 % of it gauge above, `kv` as given.
pub(crate) fn book_valve(kv: f64) -> String {
    format!(
        "kv = {kv:?}\nset_pressure_bar = {:?}\naccumulation_bar = {:?}\nx_t = {X_T:?}",
        DESIGN / 1e5,
        0.10 * (DESIGN - P_ATM) / 1e5
    )
}

/// The probe's condenser with a relief: `relief` the keys inside `relief =
/// { … }`, `valve` the relief valve node's keys, `flare` the flare node's,
/// and `pipes` any text appended after the line's two pipes.
pub(crate) fn relief_plant(relief: &str, valve: &str, flare: &str, pipes: &str) -> String {
    let mut src = with_limit(&format!("{PROBE}, relief = {{ {relief} }}"));
    src.push_str(&format!(
        "\n[nodes.{VALVE}]\ntype = \"relief_valve\"\n{valve}\n\n\
         [nodes.{FLARE}]\n{flare}\n\n\
         [[pipes]]\nname = \"{INLET_PIPE}\"\nfrom = \"column\"\nto = \"{VALVE}\"\n\
         length_m = 3.0\ndiameter_m = 0.9\n\n\
         [[pipes]]\nname = \"{FLARE_PIPE}\"\nfrom = \"{VALVE}\"\nto = \"{FLARE}\"\n\
         length_m = 80.0\ndiameter_m = 1.2\n{pipes}"
    ));
    src
}

/// The flare as a sink at atmosphere.
pub(crate) const FLARE_SINK: &str =
    "type = \"sink\"\npressure_bar = 1.01325\ncomposition = { light_naphtha = 1.0 }";

/// The probe's condenser with a relief of `kv` by the book, its line to a
/// flare at atmosphere.
pub(crate) fn with_relief(kv: f64) -> String {
    relief_plant(
        &format!("valve = \"{VALVE}\""),
        &book_valve(kv),
        FLARE_SINK,
        "",
    )
}

/// The column's relief on the faceplate this tick.
pub(crate) fn vent(snapshot: &Snapshot) -> ReliefOperation {
    let node = snapshot.nodes.iter().find(|n| n.name == "column").unwrap();
    node.column_operation
        .as_ref()
        .and_then(|op| op.relief.clone())
        .expect("a column with a relief reads it")
}

/// A snapshot by name, with its relief taken off — the column's faceplate
/// reading of it, the valve and the flare, the line's pipes — to compare with
/// the same plant without one: each node's readings but its id and kind
/// (whose ids the two new nodes shift; the column's state is compared on its
/// own), each pipe's but its id and ends.
pub(crate) fn without_relief(snapshot: &Snapshot) -> BTreeMap<String, serde_json::Value> {
    let mut by_name = BTreeMap::new();
    for node in &snapshot.nodes {
        if node.name == VALVE || node.name == FLARE {
            continue;
        }
        let mut value = serde_json::to_value(node).unwrap();
        let fields = value.as_object_mut().unwrap();
        fields.remove("id");
        fields.remove("kind");
        if let Some(op) = fields
            .get_mut("column_operation")
            .and_then(|op| op.as_object_mut())
        {
            op.remove("relief");
        }
        by_name.insert(format!("node {}", node.name), value);
    }
    for edge in &snapshot.edges {
        if edge.name == INLET_PIPE || edge.name == FLARE_PIPE {
            continue;
        }
        let mut value = serde_json::to_value(edge).unwrap();
        let fields = value.as_object_mut().unwrap();
        fields.remove("id");
        fields.remove("from");
        fields.remove("to");
        by_name.insert(format!("edge {}", edge.name), value);
    }
    by_name
}

/// **The relief is read off its valve**: the `relief_valve` node's datasheet
/// in the element's SI form — `kv` as every valve's, its set and its
/// accumulation as written — the ceiling 121 % of the column's design
/// pressure gauge (Ludwig Vol. 1B Fig. 9.7A), the flare its sink's pressure;
/// the node and its line's two pipes, in order from the column.
#[test]
fn the_relief_is_read_off_its_valve() {
    let engine = engine_of(&with_relief(5000.0)).unwrap();
    let gauge = DESIGN - P_ATM;
    let snapshot = engine.snapshot();
    let node = |name: &str| snapshot.nodes.iter().find(|n| n.name == name).unwrap().id;
    let pipe = |name: &str| snapshot.edges.iter().find(|e| e.name == name).unwrap().id;
    let relief = holdup(&engine).condenser.unwrap().relief.unwrap();
    assert_eq!(relief.cv_max, 5000.0 / (3600.0 * 1e5_f64.sqrt()));
    assert_eq!(relief.x_t, X_T);
    assert!((relief.set_pressure.value() - DESIGN).abs() < 1e-6);
    assert!((relief.accumulation.value() - 0.10 * gauge).abs() < 1e-6);
    assert_eq!(relief.ceiling, Pascal(P_ATM + 1.21 * gauge));
    assert!((relief.flare_pressure.value() - P_ATM).abs() < 1e-6);
    assert_eq!(relief.valve, node(VALVE));
    assert_eq!(relief.line, [pipe(INLET_PIPE), pipe(FLARE_PIPE)]);
    assert!(((P_ATM + 1.21 * gauge) / 1e5 - 4.047).abs() < 1e-3);
    // The node's own numbers, the one owner (§80.6).
    let NodeKind::ReliefValve {
        cv_max,
        set_pressure,
        accumulation,
        x_t,
        ..
    } = snapshot
        .nodes
        .iter()
        .find(|n| n.name == VALVE)
        .unwrap()
        .kind
    else {
        panic!("the valve is a relief valve")
    };
    assert_eq!(
        relief,
        ColumnRelief {
            cv_max,
            x_t: x_t.unwrap(),
            set_pressure,
            accumulation,
            ..relief
        }
    );
    // Absent, nothing.
    assert_eq!(
        holdup(&engine_of(&with_limit(PROBE)).unwrap())
            .condenser
            .unwrap()
            .relief,
        None
    );
}

/// Whether two readings agree to `1e-6` of their size and `1e-9` besides —
/// a level loop's integral stands near zero — every number in
/// them, field by field; everything else exactly.
pub(crate) fn close(a: &serde_json::Value, b: &serde_json::Value) -> bool {
    use serde_json::Value;
    match (a, b) {
        (Value::Number(x), Value::Number(y)) => {
            let (x, y) = (x.as_f64().unwrap(), y.as_f64().unwrap());
            (x - y).abs() <= 1e-6 * x.abs().max(y.abs()) + 1e-9
        }
        (Value::Array(x), Value::Array(y)) => {
            x.len() == y.len() && x.iter().zip(y).all(|(x, y)| close(x, y))
        }
        (Value::Object(x), Value::Object(y)) => {
            x.len() == y.len() && x.iter().all(|(k, v)| y.get(k).is_some_and(|w| close(v, w)))
        }
        (x, y) => x == y,
    }
}

/// **At rest the relief changes nothing**: below its set it is shut, its
/// line carries exactly nothing, and the column runs as the same plant
/// without one — every node's and pipe's readings by name, and the column's
/// state, to `1e-6` of each: not to the bit, since the valve and the flare
/// are two more nodes the network's cold start averages over, which moves
/// the solve's last bits from the first tick, and the column's own Newton
/// carries that to its tolerance — ~2e-8 at most on the trays' vapour
/// (§80.6). Its faceplate reads it shut, venting nothing.
#[test]
fn at_rest_the_relief_changes_nothing() {
    let mut relieved = engine_of(&with_relief(5000.0)).unwrap();
    let mut plain = engine_of(&with_limit(PROBE)).unwrap();
    for tick in 0..150 {
        relieved
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        plain.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        let as_value = |e: &Engine| serde_json::to_value(state(e)).unwrap();
        assert!(
            close(&as_value(&relieved), &as_value(&plain)),
            "tick {tick}: the column's state\n{}\n{}",
            as_value(&relieved),
            as_value(&plain)
        );
        let (with, without) = (relieved.snapshot(), plain.snapshot());
        let (with_by_name, without_by_name) = (without_relief(&with), without_relief(&without));
        assert_eq!(
            with_by_name.keys().collect::<Vec<_>>(),
            without_by_name.keys().collect::<Vec<_>>()
        );
        for (name, value) in &with_by_name {
            assert!(
                close(value, &without_by_name[name]),
                "tick {tick}: {name}\n{value}\n{}",
                without_by_name[name]
            );
        }
        for name in [INLET_PIPE, FLARE_PIPE] {
            let e = with.edges.iter().find(|e| e.name == name).unwrap();
            assert_eq!(e.stream.mass_flow.value(), 0.0, "tick {tick}: {name}");
            assert_eq!(e.stream.latent, None, "tick {tick}: {name}");
        }
        if tick > 0 {
            let shut = vent(&with);
            assert_eq!(shut.opening, 0.0, "tick {tick}");
            assert_eq!(shut.flow_kg_s, 0.0, "tick {tick}");
            assert_eq!(shut.heat_w, 0.0, "tick {tick}");
        }
    }
    assert!(!reading(&relieved.snapshot()).condenser.unwrap().at_ceiling);
}

/// The relief's `kv` by the book rule (Kister §9.3; Ludwig 1B pp.
/// 1618–1619): all the top vapour of normal running vented at full lift,
/// choked — `ṅ = ρ·(2/3)·Cv·√(998·x_c·P/ρ)/M` (ISA-75.01 at `Y = 2/3`), `ρ =
/// P·M/(R·T)`, `x_c = (γ/1.4)·x_T` — from the column at rest: its top
/// stage's vapour [mol/s] and temperature, the shut relief's label its
/// composition (the top stage's vapour), `γ` as the engine's ideal gas.
fn book_kv(engine: &Engine, snapshot: &Snapshot) -> f64 {
    let node = snapshot.nodes.iter().find(|n| n.name == "column").unwrap();
    let op = node.column_operation.as_ref().unwrap();
    let (moles, t) = (op.stage_vapour_mol_s[0], op.stage_temperatures_k[0]);
    let w = vent(snapshot).composition;
    let (mut per_mass_moles, mut cp, mut cv) = (0.0, 0.0, 0.0);
    for (c, w) in w.fractions().iter().enumerate() {
        let member = engine.slate.get(c);
        let m = member.molar_mass.value();
        let cp_v = member.cp_vapour.unwrap_or(member.cp).value();
        per_mass_moles += w / m;
        cp += w * cp_v;
        cv += w * (cp_v - R_GAS / m);
    }
    let molar_mass = 1.0 / per_mass_moles;
    let full_lift = DESIGN + 0.10 * (DESIGN - P_ATM);
    let rho = full_lift * molar_mass / (R_GAS * t);
    let x_c = cp / cv / 1.4 * X_T;
    assert!((full_lift - P_ATM) / full_lift > x_c, "choked at full lift");
    let cv_si = moles * molar_mass / (rho * 2.0 / 3.0 * (998.0 * x_c * full_lift / rho).sqrt());
    cv_si * 3600.0 * 1e5_f64.sqrt()
}

/// What a flooded-whole story read: the highest pressure [Pa], whether the
/// ceiling's alarm sounded, the tick it first did, the most vented [kg/s],
/// the books' worst [kg/s, W], and the pressure the column fell back to.
struct Story {
    peak: f64,
    first_at_ceiling: Option<u32>,
    vent_max: f64,
    mass: f64,
    heat: f64,
    after: f64,
    /// The ticks the vent took more than the top stage sent up — the rest
    /// flashed off the drum (§80.5).
    flashed: u32,
}

/// The condenser's coolant [K], as a frontend cannot set it: the test's
/// own hand on the plant (no command moves it).
pub(crate) fn set_coolant(engine: &mut Engine, coolant: f64) {
    let node = engine.graph.find_node("column").unwrap();
    if let NodeKind::Column {
        cascade: Some(spec),
        ..
    } = &mut engine.graph.node_mut(node).kind
    {
        let condenser = spec.holdup.as_mut().unwrap().condenser.as_mut().unwrap();
        condenser.coolant = Kelvin(coolant);
    }
}

/// The coolant lost — the governing case (Kister §9.3; Ludwig 1B pp.
/// 1618–1619): its temperature put far above the condensate's bubble point,
/// so the condenser condenses nothing and the pressure rises to the relief —
/// for `ticks`; then the coolant back, for `after` ticks, the condenser
/// taking the pressure back. Read every tick: the books close; the
/// faceplate's ceiling is the pressure at it; the relief open exactly
/// above its set.
fn coolant_lost(mut kv_of: impl FnMut(&Engine, &Snapshot) -> f64, ticks: u32, after: u32) -> Story {
    let mut engine = engine_of(&with_relief(5000.0)).unwrap();
    let rest = run(&mut engine, 100);
    let kv = kv_of(&engine, &rest);
    // The same plant with the relief sized; it ran the same 100 ticks.
    let mut engine = engine_of(&with_relief(kv)).unwrap();
    run(&mut engine, 100);
    let condenser = holdup(&engine).condenser.unwrap();
    let relief = condenser.relief.unwrap();
    let books = Books::of(&engine);
    set_coolant(&mut engine, 2000.0);
    let mut story = Story {
        peak: 0.0,
        first_at_ceiling: None,
        vent_max: 0.0,
        mass: 0.0,
        heat: 0.0,
        after: f64::NAN,
        flashed: 0,
    };
    for tick in 0..ticks + after {
        if tick == ticks {
            set_coolant(&mut engine, condenser.coolant.value());
        }
        let before = state(&engine);
        engine
            .tick()
            .unwrap_or_else(|e| panic!("kv {kv:.0}, tick {tick}: {e}"));
        let (m, h) = books.close(&engine, &before);
        story.mass = story.mass.max(m.abs());
        story.heat = story.heat.max(h.abs());
        let s = engine.snapshot();
        let read = reading(&s);
        let pressure = state(&engine).column_pressure.unwrap().value();
        let faceplate = read.condenser.unwrap();
        let vented = vent(&s);
        assert!(
            pressure <= relief.ceiling.value(),
            "tick {tick}: {pressure}"
        );
        assert_eq!(
            faceplate.at_ceiling,
            pressure >= relief.ceiling.value(),
            "tick {tick}"
        );
        assert_eq!(faceplate.at_design, pressure >= DESIGN, "tick {tick}");
        assert_eq!(
            vented.opening > 0.0,
            pressure > DESIGN,
            "tick {tick}: {pressure}"
        );
        if faceplate.at_ceiling && story.first_at_ceiling.is_none() {
            story.first_at_ceiling = Some(tick);
        }
        story.peak = story.peak.max(pressure);
        story.vent_max = story.vent_max.max(vented.flow_kg_s);
        story.after = pressure;
        let node = s.nodes.iter().find(|n| n.name == "column").unwrap();
        let top = node.column_operation.as_ref().unwrap().stage_vapour_mol_s[0];
        let vent_mol: f64 = vented
            .composition
            .fractions()
            .iter()
            .enumerate()
            .map(|(c, w)| vented.flow_kg_s * w / engine.slate.get(c).molar_mass.value())
            .sum();
        if vent_mol > top * (1.0 + 1e-9) {
            story.flashed += 1;
        }
        if tick < ticks {
            assert_eq!(faceplate.capacity_w, 0.0, "tick {tick}");
        }
        if tick % 100 == 99 {
            println!(
                "M75_3 kv {kv:.0}: t={:.0} s {:.4} bar, vent {:.2} kg/s, open {:.3}",
                (tick + 1) as f64 * 0.1,
                pressure / 1e5,
                vented.flow_kg_s,
                vented.opening
            );
        }
    }
    println!(
        "M75_3 kv {kv:.0}: peak {:.4} bar, ceiling from {:?}, vent max {:.2} kg/s, back to {:.4} \
         bar; {} ticks flashed off the drum; books {:.2e} kg/s {:.2} W",
        story.peak / 1e5,
        story.first_at_ceiling,
        story.vent_max,
        story.after / 1e5,
        story.flashed,
        story.mass,
        story.heat
    );
    story
}

/// **A relief sized by the book holds the coolant lost below full lift**:
/// the pressure rises past the set (3.52 bar) and the relief lifts and
/// holds it inside its band, never at the ceiling; the coolant back, the
/// condenser takes it back below the set and the relief shuts. The books
/// close every tick, the vent counted.
#[test]
fn slow_a_book_relief_holds_the_coolant_lost_below_full_lift() {
    let mut kv_read = 0.0;
    let story = coolant_lost(
        |engine, rest| {
            kv_read = book_kv(engine, rest);
            kv_read
        },
        1500,
        1500,
    );
    // About 5 000 (§80.4, by hand on the probe's γ).
    assert!(kv_read > 3500.0 && kv_read < 6500.0, "{kv_read}");
    let full_lift = DESIGN + 0.10 * (DESIGN - P_ATM);
    assert!(story.peak > DESIGN, "{}", story.peak);
    assert!(story.peak < full_lift, "{}", story.peak);
    assert_eq!(story.first_at_ceiling, None);
    assert!(story.vent_max > 1.0, "{}", story.vent_max);
    assert!(story.after < DESIGN, "{}", story.after);
    // The pressure falling, the vent asked past what the column sent up, the
    // rest off the drum — and the books still closed.
    assert!(story.flashed > 0);
    assert!(story.mass < 1.0e-8, "{}", story.mass);
    assert!(story.heat < 20.0, "{}", story.heat);
}

/// **A relief too small is held at its ceiling, its alarm on**: a twentieth
/// of the book's size cannot vent what the column sends up, the pressure
/// passes full lift and is held at 121 % of the design pressure (gauge)
/// exactly, the run going on; the books close every tick.
#[test]
fn slow_a_relief_too_small_is_held_at_its_ceiling() {
    let story = coolant_lost(|engine, rest| book_kv(engine, rest) / 20.0, 2500, 0);
    let ceiling = P_ATM + 1.21 * (DESIGN - P_ATM);
    let at = story.first_at_ceiling.expect("the ceiling's alarm sounds");
    assert!(at < 2400, "{at}");
    assert_eq!(story.peak, ceiling);
    assert_eq!(story.after, ceiling);
    assert!(story.mass < 1.0e-8, "{}", story.mass);
    assert!(story.heat < 20.0, "{}", story.heat);
}

/// The full lift and ceiling the loader derived, and the set, in order.
#[test]
fn the_relief_pressures_are_in_order() {
    let engine = engine_of(&with_relief(5000.0)).unwrap();
    let relief = holdup(&engine).condenser.unwrap().relief.unwrap();
    let full_lift = relief.set_pressure.value() + relief.accumulation.value();
    assert!(SET < relief.set_pressure.value());
    assert!(full_lift < relief.ceiling.value());
    assert!(relief.flare_pressure.value() < SET);
}
