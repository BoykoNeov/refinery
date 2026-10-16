//! M59.4: the trip that stops a hydrogen bed's oil, the wall that leaks
//! hydrogen, and the floor that keeps a starved loop above the atmosphere
//! (docs/DESIGN.md §64.7, §64.13), gated on
//! `scenarios/hydrocracker_hydrogen_leak.toml`, `hydrocracker_makeup.toml` and
//! fixtures cut from them.
//!
//! What is pinned, and against what:
//!
//! 1. a pass that declares a heating value gets a second, dormant hole on its
//!    outlet pipe, and one that does not gets none; the loader refuses a bad
//!    value;
//! 2. D10 lifted for a hydrogen bed only: the trip that shuts the oil is
//!    admitted, and refused when the same trip also shuts the pass's line;
//! 3. with its oil stopped, the bed's first law is the hydrogen's heat alone,
//!    by hand, every tick;
//! 4. the story: the trip stops the oil, the hydrogen sweeps the bed back
//!    under its limit, the trip resets, and the bed settles at the gas's own
//!    temperature — on both solvers;
//! 5. untripped, the wall fails, BOTH holes open to the declared area, the
//!    fire is both leaks at their own heating values, and the loop vents
//!    through the hydrogen hole;
//! 6. the repair asks for both holes patched;
//! 7. §64.11's open edge, closed: the starved makeup plant, punctured on its
//!    gas line at tick 50 000, keeps blowing outward on both solvers.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{LeakRole, NodeKind, TripId, TripState, WallState};
use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot};
use refinery_core::units::{SquareMeter, P_ATM};
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/hydrocracker_hydrogen_leak.toml");
const MAKEUP: &str = include_str!("../../../scenarios/hydrocracker_makeup.toml");

const KELVIN: f64 = 273.15;
/// The demo's numbers, by hand from its file.
const LIMIT_C: f64 = 420.0;
const WALL_C: f64 = 550.0;
const OIL_LHV: f64 = 42.8e6;
const H2_LHV: f64 = 119.96e6;
const RUPTURE_M2: f64 = 1.0e-4;
const BED_J_PER_K: f64 = 30.0e6;
const DT_S: f64 = 0.1;
/// The pass's outlet pipe, which carries the wall's hydrogen hole.
const PASS_OUT: &str = "reactor_gas_outlet";

fn build_src(src: &str, solver: &str) -> Engine {
    let src = src.replacen(r#"flow = "newton""#, &format!(r#"flow = "{solver}""#), 1);
    let file = refinery_scenarios::load_str(&src).expect("the scenario must parse");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("it must build: {e}"))
}

fn refusal(src: &str) -> String {
    let file = refinery_scenarios::load_str(src).expect("the scenario must parse");
    match refinery_scenarios::build_engine(&file) {
        Ok(_) => panic!("the plant must be refused"),
        Err(e) => e.to_string(),
    }
}

fn edit(src: &str, from: &str, to: &str) -> String {
    assert!(src.contains(from), "the fixture must contain {from:?}");
    src.replacen(from, to, 1)
}

const TRIP_ACTIONS: &str = r#"actions = [{ valve = "feed_valve", position = 0.0 }]"#;

fn untripped() -> String {
    edit(DEMO, "limit_c = 420.0", "limit_c = 2000.0")
}

fn tick(engine: &mut Engine, n: u64) {
    for _ in 0..n {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: {e}"));
    }
}

fn apply(engine: &mut Engine, cmd: Command) {
    engine
        .apply(cmd)
        .unwrap_or_else(|e| panic!("the command must be accepted: {e}"));
}

fn refused(engine: &mut Engine, cmd: Command) -> String {
    engine
        .apply(cmd)
        .expect_err("the command must be refused")
        .to_string()
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("a node '{name}'"))
}

fn flow(s: &Snapshot, edge: &str) -> f64 {
    s.edges
        .iter()
        .find(|e| e.name == edge)
        .unwrap_or_else(|| panic!("an edge '{edge}'"))
        .stream
        .mass_flow
        .value()
}

fn edge_id(engine: &Engine, name: &str) -> refinery_core::graph::EdgeId {
    engine
        .graph
        .edge_ids()
        .find(|e| engine.graph.pipe(*e).name == name)
        .unwrap_or_else(|| panic!("an edge named '{name}'"))
}

fn node_id(engine: &Engine, name: &str) -> refinery_core::graph::NodeId {
    engine
        .graph
        .find_node(name)
        .unwrap_or_else(|| panic!("a node named '{name}'"))
}

fn orifice_area(engine: &Engine, name: &str) -> f64 {
    match engine.graph.pipe(edge_id(engine, name)).leak {
        LeakRole::Orifice { area } => area.value(),
        other => panic!("'{name}' is an orifice, not {other:?}"),
    }
}

fn bed_k(s: &Snapshot) -> f64 {
    match &node(s, "bed").kind {
        NodeKind::BedReactor { bed, .. } => bed.temperature.value(),
        other => panic!("bed is a bed reactor, not {other:?}"),
    }
}

fn wall(s: &Snapshot) -> WallState {
    match &node(s, "bed").kind {
        NodeKind::BedReactor { wall, .. } => wall.state,
        other => panic!("bed is a bed reactor, not {other:?}"),
    }
}

fn tripped(s: &Snapshot) -> Option<u64> {
    match s.trips.iter().find(|t| t.name == "bed_high")?.state {
        TripState::Tripped { at_tick, .. } => Some(at_tick),
        _ => None,
    }
}

fn puncture(engine: &mut Engine, pipe: &str, area: f64) {
    let edge = edge_id(engine, pipe);
    apply(
        engine,
        Command::PuncturePipe {
            edge,
            area: SquareMeter(area),
        },
    );
}

/// 1. The hole on the pass's outlet: built, dormant, for a pass that declares
///    hydrogen's heating value; absent on one that does not, so the M59.2 plant
///    keeps its shape. The loader refuses a value that is negative or not a
///    number.
#[test]
fn a_pass_with_a_heating_value_gets_a_second_hole() {
    let engine = build_src(DEMO, "newton");
    let bed = node_id(&engine, "bed");
    let NodeKind::BedReactor { wall, .. } = &engine.graph.node(bed).kind else {
        panic!("bed is a bed reactor");
    };
    let leak = wall.hydrogen_leak.as_ref().expect("a hydrogen leak");
    assert_eq!(leak.heating_value.value(), H2_LHV);
    let hole = edge_id(&engine, "reactor_gas_outlet__leak");
    assert_eq!(leak.hole, Some(hole));
    assert_eq!(orifice_area(&engine, "reactor_gas_outlet__leak"), 0.0);
    assert_eq!(
        engine.graph.pipe(edge_id(&engine, PASS_OUT)).leak,
        LeakRole::Punctureable { orifice: hole },
        "the pass's outlet pipe keeps its name and patches the hole"
    );
    assert_eq!(wall.holes().count(), 2);

    let plain = build_src(MAKEUP, "newton");
    let NodeKind::BedReactor { wall, .. } = &plain.graph.node(node_id(&plain, "bed")).kind else {
        panic!("bed is a bed reactor");
    };
    assert!(wall.hydrogen_leak.is_none());
    assert_eq!(wall.holes().count(), 1);
    assert!(plain
        .graph
        .edge_ids()
        .all(|e| plain.graph.pipe(e).name != "reactor_gas_outlet__leak"));

    // The pass's key, not the gas heater's, which declares the same value.
    let on_pass = "fluid_heating_value_mj_per_kg = 119.96\n\n# The gas cooler";
    for bad in ["-1.0", "nan"] {
        let e = refusal(&edit(
            DEMO,
            on_pass,
            &format!("fluid_heating_value_mj_per_kg = {bad}\n\n# The gas cooler"),
        ));
        assert!(
            e.contains("hydrogen pass 'pass' has fluid_heating_value_mj_per_kg"),
            "{bad}: {e}"
        );
    }
}

/// 2. D10 for a hydrogen bed: the trip that shuts the oil feed is admitted (the
///    demo loads). The same trip shutting the pass's own line as well — the
///    loop valve upstream of it, or a valve between the pass and the
///    separator — is refused in its own words; a valve off the line, the
///    makeup, is not on it. A bed with no pass keeps M57.3's refusal
///    (`bed_trip_reference` gate 6).
#[test]
fn the_oil_trip_is_admitted_unless_it_stops_the_hydrogen_too() {
    build_src(DEMO, "newton");

    let also = |action: &str| {
        edit(
            DEMO,
            TRIP_ACTIONS,
            &format!(r#"actions = [{{ valve = "feed_valve", position = 0.0 }}, {action}]"#),
        )
    };
    let e = refusal(&also(r#"{ valve = "loop_valve", position = 0.0 }"#));
    assert!(
        e.contains("shuts valve 'feed_valve'")
            && e.contains("also shuts valve 'loop_valve'")
            && e.contains("hydrogen pass 'pass'")
            && e.contains("refuse every reset"),
        "the loop valve: {e}"
    );

    // A valve on the outlet side, between the pass and the gas cooler.
    let outlet_valve = edit(
        &edit(
            &also(r#"{ valve = "outlet_valve", position = 0.0 }"#),
            r#"from = "pass"
to = "gas_cooler""#,
            r#"from = "pass"
to = "outlet_valve""#,
        ),
        "[nodes.gas_cooler]",
        r#"[nodes.outlet_valve]
type = "valve"
kv = 60.0
opening = 1.0
x_t = 0.7

[[pipes]]
name = "outlet_valve_line"
from = "outlet_valve"
to = "gas_cooler"
length_m = 5.0
diameter_m = 0.2

[nodes.gas_cooler]"#,
    );
    let e = refusal(&outlet_valve);
    assert!(e.contains("also shuts valve 'outlet_valve'"), "{e}");

    // Partway is not shut, and the makeup is off the pass's line.
    build_src(
        &also(r#"{ valve = "loop_valve", position = 0.5 }"#),
        "newton",
    );
    build_src(
        &also(r#"{ valve = "makeup_valve", position = 0.0 }"#),
        "newton",
    );
}

/// 3. A bed with its oil stopped: no reaction, no fire, and its first law is
///    the hydrogen's heat alone — `C·ΔT_b/Δt` is exactly what the hydrogen
///    carried off, to rounding, every tick for a minute after the trip.
#[test]
fn a_stopped_bed_loses_its_heat_to_the_hydrogen() {
    let mut engine = build_src(DEMO, "newton");
    for _ in 0..6_000 {
        tick(&mut engine, 1);
        if tripped(&engine.snapshot()).is_some() {
            break;
        }
    }
    assert!(tripped(&engine.snapshot()).is_some(), "the trip fires");
    // The valve shuts at the top of the next tick.
    tick(&mut engine, 2);
    let mut before = engine.snapshot();
    for _ in 0..600 {
        tick(&mut engine, 1);
        let s = engine.snapshot();
        assert_eq!(flow(&s, "valve_line"), 0.0, "the oil is stopped");
        let bed = node(&s, "bed");
        assert_eq!(bed.reaction_heat_w, Some(0.0));
        assert_eq!(bed.wall_fire_w, Some(0.0));
        let hydrogen = bed.hydrogen_heat_w.expect("a bed with a pass");
        assert!(hydrogen < -1.0e5, "the hydrogen cools it: {hydrogen} W");
        let stored = BED_J_PER_K * (bed_k(&s) - bed_k(&before)) / DT_S;
        assert!(
            (stored - hydrogen).abs() <= 1e-6 * hydrogen.abs(),
            "tick {}: stored {stored} W, the hydrogen {hydrogen} W",
            s.tick
        );
        before = s;
    }
}

/// One minute's reading: the bed [°C], the oil flow [kg/s], the pass's
/// pressure [bar], the trip's tick, and whether it is still tripped.
type Reading = (f64, f64, f64, Option<u64>, bool);

/// 4. The file's story. The 395 °C feed creeps; the trip fires at 420 °C,
///    about 6 min 15 s in, and shuts the oil. With the oil stopped the
///    hydrogen sweeps the bed: it turns within seconds, under its limit, and a
///    reset refused while the bed stood at its limit is accepted once it is
///    below. By 30 minutes the bed is within a kelvin of the gas heater's
///    395 °C. The loop holds near 152 bar, the bed consuming nothing. The game
///    solver tells the same story.
#[test]
fn slow_the_hydrogen_sweeps_a_stopped_bed_and_the_trip_resets() {
    let story = |solver: &str| -> Vec<Reading> {
        let mut engine = build_src(DEMO, solver);
        let mut reset_at = None;
        (0..30)
            .map(|_| {
                for _ in 0..600 {
                    tick(&mut engine, 1);
                    let s = engine.snapshot();
                    if tripped(&s).is_some() && reset_at.is_none() {
                        // Refused while the trip's own reading is at its limit:
                        // the reading is the top of the tick, the snapshot's the end.
                        let reset = Command::ResetTrip { trip_id: TripId(0) };
                        match engine.apply(reset) {
                            Ok(()) => reset_at = Some(s.tick),
                            Err(says) => {
                                let says = says.to_string();
                                assert!(says.contains("bed_high"), "{says}");
                                assert!(
                                    bed_k(&s) - KELVIN > LIMIT_C - 0.5,
                                    "refused at {} °C: {says}",
                                    bed_k(&s) - KELVIN
                                );
                            }
                        }
                    }
                }
                let s = engine.snapshot();
                let still = s
                    .trips
                    .iter()
                    .any(|t| t.name == "bed_high" && t.state.is_tripped());
                (
                    bed_k(&s) - KELVIN,
                    flow(&s, "valve_line"),
                    node(&s, "pass").pressure_pa / 1e5,
                    reset_at,
                    still,
                )
            })
            .collect()
    };
    let newton = story("newton");
    let reset = newton[29].3.expect("the trip is reset");
    assert!((3_700..3_900).contains(&reset), "reset on tick {reset}");
    assert!(newton.iter().all(|r| !r.4), "reset, it stays armed");
    let peak = newton.iter().map(|r| r.0).fold(f64::NEG_INFINITY, f64::max);
    assert!((418.0..421.0).contains(&peak), "peak {peak} °C");
    assert!(newton[7..].iter().all(|r| r.1 == 0.0), "the oil stays shut");
    let last = newton[29];
    assert!((395.0..396.0).contains(&last.0), "30 min: {} °C", last.0);
    assert!((150.0..154.0).contains(&last.2), "30 min: {} bar", last.2);

    let simple = story("simple");
    let simple_reset = simple[29].3.expect("the trip is reset");
    assert!(
        simple_reset.abs_diff(reset) <= 20,
        "reset on tick {simple_reset} against {reset}"
    );
    for (minute, (n, s)) in newton.iter().zip(&simple).enumerate() {
        assert!(
            (n.0 - s.0).abs() < 0.5,
            "minute {}: {} °C against {} °C",
            minute + 1,
            n.0,
            s.0
        );
    }
}

/// 5. Untripped, the wall fails about 22 minutes in and opens BOTH holes, each
///    to its declared 1 cm²: the oil's on the bed's outlet and the hydrogen's
///    on the pass's. From then on the fire is both leaks at their own heating
///    values, by hand, every tick. The loop vents through the hydrogen hole —
///    an uncontrolled depressuring, under 20 bar within four minutes — and the
///    bed, hottest near 640 °C, settles burning near 517 °C with its loop held
///    near 4.3 bar, where the wide-open makeup feeds the hole and the bed.
#[test]
fn slow_a_failed_wall_leaks_hydrogen_and_burns_it() {
    let mut engine = build_src(&untripped(), "newton");
    let mut failed_at = None;
    for _ in 0..16_000 {
        tick(&mut engine, 1);
        if wall(&engine.snapshot()).is_failed() {
            failed_at = Some(engine.snapshot().tick);
            break;
        }
    }
    let failed_at = failed_at.expect("the wall fails");
    assert!(
        (13_000..13_800).contains(&failed_at),
        "failed on tick {failed_at}"
    );
    assert_eq!(orifice_area(&engine, "product_line__leak"), RUPTURE_M2);
    assert_eq!(
        orifice_area(&engine, "reactor_gas_outlet__leak"),
        RUPTURE_M2
    );
    let at_failure = node(&engine.snapshot(), "pass").pressure_pa / 1e5;

    let mut peak = f64::NEG_INFINITY;
    for minute in 0..20 {
        for _ in 0..600 {
            tick(&mut engine, 1);
            let s = engine.snapshot();
            let oil = flow(&s, "product_line__leak");
            let hydrogen = flow(&s, "reactor_gas_outlet__leak");
            assert!(oil >= 0.0 && hydrogen >= 0.0, "both holes blow outward");
            let by_hand = oil * OIL_LHV + hydrogen * H2_LHV;
            let fire = node(&s, "bed").wall_fire_w.expect("a bed's fire");
            assert!(
                (fire - by_hand).abs() <= 1e-12 * by_hand,
                "tick {}: {fire} W against {by_hand} W",
                s.tick
            );
            peak = peak.max(bed_k(&s) - KELVIN);
        }
        let s = engine.snapshot();
        let pass = node(&s, "pass").pressure_pa / 1e5;
        if minute == 3 {
            assert!(
                pass < 20.0 && pass < at_failure / 2.0,
                "4 min after the failure the loop is at {pass} bar, from {at_failure}"
            );
        }
    }
    assert!((620.0..660.0).contains(&peak), "peak {peak} °C");
    let s = engine.snapshot();
    let settled = bed_k(&s) - KELVIN;
    assert!((510.0..525.0).contains(&settled), "settled {settled} °C");
    let pass = node(&s, "pass").pressure_pa / 1e5;
    assert!((3.5..5.5).contains(&pass), "the loop at {pass} bar");
    assert!(flow(&s, "reactor_gas_outlet__leak") > 0.0, "still leaking");
}

/// 6. The repair: a failed wall with two holes is repaired only once BOTH are
///    patched, each by its own puncture at zero, and the refusal names the one
///    still open. The bed is under its limit throughout, so the holes are all
///    that stands in the way.
#[test]
fn slow_the_repair_asks_for_both_holes_patched() {
    let mut engine = build_src(&untripped(), "newton");
    for _ in 0..16_000 {
        tick(&mut engine, 1);
        if wall(&engine.snapshot()).is_failed() {
            break;
        }
    }
    assert!(wall(&engine.snapshot()).is_failed(), "the wall fails");
    // Forty minutes on, the bed burns near 517 °C, under the wall's limit.
    tick(&mut engine, 24_000);
    assert!(bed_k(&engine.snapshot()) - KELVIN < WALL_C);
    let bed = node_id(&engine, "bed");
    let repair = Command::RepairBedWall { node: bed };

    let says = refused(&mut engine, repair.clone());
    assert!(says.contains("on 'product_line'"), "{says}");
    puncture(&mut engine, "product_line", 0.0);
    let says = refused(&mut engine, repair.clone());
    assert!(
        says.contains("still leaks through a 1.00 cm² hole")
            && says.contains("on 'reactor_gas_outlet'"),
        "{says}"
    );
    puncture(&mut engine, PASS_OUT, 0.0);
    apply(&mut engine, repair);
    assert_eq!(wall(&engine.snapshot()), WallState::Intact);
    tick(&mut engine, 1);
    let s = engine.snapshot();
    assert_eq!(node(&s, "bed").wall_fire_w, Some(0.0), "the fire is out");
}

/// 7. §64.11's open edge, closed by the floor (M59.4, §64.13): the M59.2 plant,
///    tripped, eats its loop down and rests above the atmosphere — no longer
///    at 1 kPa — so a hole punched in its gas line at tick 50 000 (§64.11's
///    exact probe, which then refused the next tick) blows OUTWARD for ten
///    minutes on both solvers, and the loop drains toward the atmosphere from
///    above.
fn a_starved_loop_punctured_blows_outward(solver: &str) {
    let mut engine = build_src(MAKEUP, solver);
    tick(&mut engine, 50_000);
    let before = engine.snapshot();
    assert!(
        node(&before, "pass").pressure_pa > P_ATM.value() + 15.0e3,
        "{solver}: the pass at {} Pa",
        node(&before, "pass").pressure_pa
    );
    puncture(&mut engine, "reactor_gas_line", RUPTURE_M2);
    for _ in 0..6_000 {
        tick(&mut engine, 1);
        let leak = flow(&engine.snapshot(), "reactor_gas_line__leak");
        assert!(leak >= 0.0, "{solver}: the hole draws {leak} kg/s in");
    }
    let s = engine.snapshot();
    let lowest = node(&s, "separator").pressure_pa;
    assert!(
        lowest > P_ATM.value() && lowest < node(&before, "separator").pressure_pa,
        "{solver}: the separator at {lowest} Pa"
    );
}

#[test]
fn slow_a_starved_loop_punctured_blows_outward_newton() {
    a_starved_loop_punctured_blows_outward("newton");
}

#[test]
fn slow_a_starved_loop_punctured_blows_outward_simple() {
    a_starved_loop_punctured_blows_outward("simple");
}
