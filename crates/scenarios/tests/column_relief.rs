//! M75.3.1 — a relief valve on a column's top, before the condenser
//! (docs/DESIGN.md §80.5): the holdup plant (`crude_unit_holdup.toml`,
//! untouched) with its steel, the M75.1 `condenser_limit` and its `relief`
//! declared inline. The vent leaves on no pipe (M75.3.2 puts it on the map).

use refinery_core::engine::Engine;
use refinery_core::graph::{ColumnRelief, NodeKind};
use refinery_core::snapshot::{ReliefOperation, Snapshot};
use refinery_core::units::{Kelvin, Pascal, R_GAS};

use crate::column_condenser::{with_limit, PROBE};
use crate::column_empty::{engine_of, holdup, reading, refused, run, state, Books};

/// The holdup plant's set pressure and its derived design pressure [Pa abs],
/// and the atmosphere the flare is at.
const SET: f64 = 1.8e5;
const DESIGN: f64 = 1.8e5 + 1.72e5;
const P_ATM: f64 = 101_325.0;
const X_T: f64 = 0.7;

/// The probe's condenser with a relief of `kv`.
fn with_relief(kv: f64) -> String {
    with_limit(&format!(
        "{PROBE}, relief = {{ kv = {kv:?}, x_t = {X_T:?} }}"
    ))
}

/// The column's relief on the faceplate this tick.
fn vent(snapshot: &Snapshot) -> ReliefOperation {
    let node = snapshot.nodes.iter().find(|n| n.name == "column").unwrap();
    node.column_operation
        .as_ref()
        .and_then(|op| op.relief.clone())
        .expect("a column with a relief reads it")
}

/// A snapshot with its relief taken off — its declaration and its
/// faceplate — to compare with the same plant without one.
fn without_relief(mut snapshot: Snapshot) -> String {
    for node in &mut snapshot.nodes {
        if let NodeKind::Column {
            cascade: Some(spec),
            ..
        } = &mut node.kind
        {
            if let Some(condenser) = spec.holdup.as_mut().and_then(|h| h.condenser.as_mut()) {
                condenser.relief = None;
            }
        }
        if let Some(op) = node.column_operation.as_mut() {
            op.relief = None;
        }
    }
    serde_json::to_string(&snapshot).unwrap()
}

/// **The relief is read as written**, its pressures derived from the
/// design pressure as API RP-520 states them (Ludwig Vol. 1B Fig. 9.7A):
/// set at 3.52 bar, full lift 10 % of it gauge above (3.77 bar), the
/// ceiling 121 % of it gauge (4.05 bar), the flare at atmosphere; `kv` in
/// the element's SI form, as every valve's.
#[test]
fn the_relief_is_read_as_written() {
    let engine = engine_of(&with_relief(5000.0)).unwrap();
    let gauge = DESIGN - P_ATM;
    assert_eq!(
        holdup(&engine).condenser.unwrap().relief,
        Some(ColumnRelief {
            cv_max: 5000.0 / (3600.0 * 1e5_f64.sqrt()),
            x_t: X_T,
            set_pressure: Pascal(DESIGN),
            accumulation: Pascal(0.10 * gauge),
            ceiling: Pascal(P_ATM + 1.21 * gauge),
            flare_pressure: Pascal(P_ATM),
        })
    );
    assert!(((P_ATM + 1.21 * gauge) / 1e5 - 4.047).abs() < 1e-3);
    // Absent, nothing.
    assert_eq!(
        holdup(&engine_of(&with_limit(PROBE)).unwrap())
            .condenser
            .unwrap()
            .relief,
        None
    );
}

/// **A relief is refused where it cannot run**: a `kv` not above zero, an
/// `x_t` outside (0, 1], a key it does not read, a key missing.
#[test]
fn a_relief_is_refused_where_it_cannot_run() {
    let says = |relief: &str, words: &str| {
        let e = refused(&with_limit(&format!("{PROBE}, relief = {{ {relief} }}")));
        assert!(e.contains(words), "{relief}: {e}");
    };
    says("kv = 0.0, x_t = 0.7", "relief.kv");
    says("kv = -5.0, x_t = 0.7", "relief.kv");
    says("kv = nan, x_t = 0.7", "relief.kv");
    says("kv = 5000.0, x_t = 0.0", "relief.x_t");
    says("kv = 5000.0, x_t = 1.5", "relief.x_t");
    says("kv = 5000.0", "x_t");
    says("kv = 5000.0, x_t = 0.7, set_bar = 3.0", "set_bar");
}

/// **At rest the relief changes nothing**: below its set it is shut, and
/// the column runs bit-identical to the same plant without one; its
/// faceplate reads it shut, venting nothing.
#[test]
fn at_rest_the_relief_changes_nothing() {
    let mut relieved = engine_of(&with_relief(5000.0)).unwrap();
    let mut plain = engine_of(&with_limit(PROBE)).unwrap();
    for tick in 0..150 {
        relieved
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        plain.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        assert_eq!(state(&relieved), state(&plain), "tick {tick}");
        assert_eq!(
            without_relief(relieved.snapshot()),
            without_relief(plain.snapshot()),
            "tick {tick}"
        );
        if tick > 0 {
            let shut = vent(&relieved.snapshot());
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
fn set_coolant(engine: &mut Engine, coolant: f64) {
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
