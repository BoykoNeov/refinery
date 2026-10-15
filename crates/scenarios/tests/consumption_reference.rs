//! M59.2: a catalyst bed's reaction consumes the hydrogen it runs on — the
//! network's first nodal mass sink (docs/DESIGN.md §64.6, §64.11), gated on
//! `scenarios/hydrocracker_makeup.toml` and fixtures cut from it.
//!
//! What is pinned, and against what:
//!
//! 1. the demand is `ṁ_oil · Σᵢ (y_out,i − y_in,i) · uptakeᵢ`, by hand from the
//!    bed's own oil line and product line, every tick;
//! 2. the sink lags it by one tick and follows the pressure: what the gas side
//!    loses on tick `n+1` is tick `n`'s demand times `P(n+1)/P(n)`, to the
//!    solve's tolerance — and the first tick, with no reaction behind it,
//!    draws nothing;
//! 3. the books close on the solve's own flows: the pass loses exactly what is
//!    booked, the loop's stock falls by makeup less vent less consumption, and
//!    the pass's energy closes with the consumed hydrogen's enthalpy — on
//!    forward-flowing ticks and on the ticks the sink pulls gas back in
//!    through the pass's outlet;
//! 4. the story: a trip that only shuts the makeup lets the bed eat its loop
//!    down and starve itself, under the wall's limit — where the same plant
//!    without the trip fails its wall, its makeup unable to keep up;
//! 5. the game solver tells the same story;
//! 6. a starved loop comes to REST at the factor's floor, reacting nothing;
//! 7. the factor's regularisation near vacuum, by hand;
//! 8. the loader's refusals — M59.3's included: the figures ordered along the
//!    reaction's own paths, a left-out lump counting as zero (§64.12);
//! 9. a bed fed products, not the datum lump, consumes on every tick.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{
    HydrogenKinetics, NodeKind, TripState, WallState, HYDROGEN_FACTOR_RAMP_FROM,
    HYDROGEN_FACTOR_ZERO_AT,
};
use refinery_core::snapshot::{NodeSnapshot, Snapshot};
use refinery_core::units::{Pascal, P_ATM};
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/hydrocracker_makeup.toml");

/// The demo's numbers, by hand from its file.
const CP_H2: f64 = 14_300.0;
const DT_S: f64 = 0.1;
const KELVIN: f64 = 273.15;
/// The enthalpy datum the constant-cp model measures from (`energy::T_REF`).
const T_REF_K: f64 = 273.15;
/// kg of hydrogen per kg of each lump formed, in slate order: vgo, distillate,
/// naphtha, gas, hydrogen, water.
const UPTAKE: [f64; 6] = [0.0, 0.020, 0.035, 0.070, 0.0, 0.0];
/// The solves' absolute mass tolerance [kg/s] (both fidelities' default).
const TOL_ABS: f64 = 1e-8;
/// The pass's inflow and outflow pipes, and the loop's other boundaries.
const PASS_IN: &str = "reactor_gas_line__downstream";
const PASS_OUT: &str = "reactor_gas_outlet";

fn build_src(src: &str, solver: &str) -> Engine {
    let src = src.replacen(r#"flow = "newton""#, &format!(r#"flow = "{solver}""#), 1);
    let file = refinery_scenarios::load_str(&src).expect("the scenario must parse");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("it must build: {e}"))
}

fn refused(src: &str) -> String {
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

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("a node '{name}'"))
}

fn stream<'a>(s: &'a Snapshot, edge: &str) -> &'a refinery_core::stream::Stream {
    &s.edges
        .iter()
        .find(|e| e.name == edge)
        .unwrap_or_else(|| panic!("an edge '{edge}'"))
        .stream
}

fn flow(s: &Snapshot, edge: &str) -> f64 {
    stream(s, edge).mass_flow.value()
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

fn separator_kg(s: &Snapshot) -> f64 {
    match &node(s, "separator").kind {
        NodeKind::Vessel(vessel) => vessel.mass.value(),
        other => panic!("separator is a vessel, not {other:?}"),
    }
}

fn tripped(s: &Snapshot) -> Option<u64> {
    match s.trips.iter().find(|t| t.name == "bed_high")?.state {
        TripState::Tripped { at_tick, .. } => Some(at_tick),
        _ => None,
    }
}

fn consumed(s: &Snapshot) -> f64 {
    node(s, "bed")
        .hydrogen_consumed_kg_s
        .expect("a bed whose pass consumes")
}

fn demand(s: &Snapshot) -> f64 {
    node(s, "bed")
        .hydrogen_demand_kg_s
        .expect("a bed whose pass consumes")
}

/// 1. The demand by hand: the oil line's flow times the uptake of what the bed
///    formed, its product line's composition less its feed's.
#[test]
fn the_demand_is_the_uptake_of_what_the_bed_formed() {
    let mut engine = build_src(DEMO, "newton");
    for _ in 0..300 {
        tick(&mut engine, 10);
        let s = engine.snapshot();
        let feed = stream(&s, "valve_line").composition.fractions();
        let products = stream(&s, "product_line").composition.fractions();
        let per_kg: f64 = products
            .iter()
            .zip(feed)
            .zip(UPTAKE)
            .map(|((after, before), uptake)| (after - before) * uptake)
            .sum();
        let by_hand = flow(&s, "valve_line") * per_kg;
        assert!(
            (demand(&s) - by_hand).abs() < 1e-12 * by_hand,
            "tick {}: {} kg/s against {by_hand} kg/s by hand",
            s.tick,
            demand(&s)
        );
    }
    // Ten times the calibration's space velocity: far less than its 2 wt%.
    let s = engine.snapshot();
    let share = demand(&s) / flow(&s, "valve_line");
    assert!((0.003..0.02).contains(&share), "{share} of the feed");
}

/// 2. The sink lags the demand by one tick and follows the pressure: what the
///    pass loses on tick `n+1` is tick `n`'s demand times `P(n+1)/P(n)`, to the
///    solve's own tolerance. The first tick, with no reaction behind it, draws
///    nothing.
#[test]
fn the_sink_is_the_last_ticks_demand_at_this_ticks_pressure() {
    let mut engine = build_src(DEMO, "newton");
    tick(&mut engine, 1);
    let first = engine.snapshot();
    assert!(
        consumed(&first).abs() < 2.0 * TOL_ABS,
        "tick 1 draws {} kg/s",
        consumed(&first)
    );
    assert!(demand(&first) > 0.0, "tick 1's reaction draws next tick");
    let mut before = first;
    // Through the trip and the sag, where the pressure moves fastest.
    for _ in 0..13_000 {
        tick(&mut engine, 1);
        let s = engine.snapshot();
        let p = node(&s, "pass").pressure_pa;
        let p_before = node(&before, "pass").pressure_pa;
        let expected = demand(&before) * p / p_before;
        let scale = flow(&s, PASS_IN).abs().max(flow(&s, PASS_OUT).abs());
        assert!(
            (consumed(&s) - expected).abs() < 2.0 * (TOL_ABS + 1e-6 * scale),
            "tick {}: {} kg/s against {expected} kg/s",
            s.tick,
            consumed(&s)
        );
        before = s;
    }
}

/// The pass's energy closes with the consumed hydrogen's enthalpy: every inflow
/// in at its pipe's temperature, every outflow out at the bed's outlet, the
/// consumed share off the books there, and the rest is the heat the hydrogen
/// gave the bed.
fn pass_books_close(s: &Snapshot) {
    let out = node(s, "pass").temperature_k;
    let bed = node(s, "bed");
    let consumed = consumed(s);
    let consumed_heat = bed.hydrogen_consumed_heat_w.expect("a consuming pass");
    let hydrogen_heat = bed.hydrogen_heat_w.expect("a bed with a pass");
    let enthalpy = |mass: f64, temperature: f64| mass * CP_H2 * (temperature - T_REF_K);
    let (mut inflow, mut outflow, mut net_in) = (0.0, 0.0, 0.0);
    // The pass's inlet is stored INTO it, its outlet OUT of it.
    for (edge, sign) in [(PASS_IN, 1.0), (PASS_OUT, -1.0)] {
        let into_pass = sign * flow(s, edge);
        net_in += into_pass;
        if into_pass > 0.0 {
            inflow += enthalpy(into_pass, stream(s, edge).temperature.value());
        } else {
            outflow += enthalpy(-into_pass, out);
        }
    }
    assert_eq!(
        consumed.to_bits(),
        net_in.to_bits(),
        "tick {}: the pass loses what is booked",
        s.tick
    );
    assert!(
        (consumed_heat - enthalpy(consumed, out)).abs() < 1e-9 * consumed_heat.abs().max(1.0),
        "tick {}: the consumed hydrogen leaves at the outlet temperature",
        s.tick
    );
    let scale = inflow.abs() + outflow.abs() + 1.0;
    assert!(
        (inflow - outflow - consumed_heat - hydrogen_heat).abs() < 1e-9 * scale,
        "tick {}: {inflow} W in, {outflow} W out, {consumed_heat} W consumed, \
         {hydrogen_heat} W to the bed",
        s.tick
    );
}

/// 3. The books close, on the tripped story through its sag and on the
///    untripped one through the ticks its sink pulls gas back in through the
///    pass's outlet. The loop's only stock is the separator: it falls by what
///    came in, less what was vented and consumed, to the solves' tolerance.
#[test]
fn the_books_close_on_the_solves_own_flows() {
    for (src, ticks) in [(DEMO.to_string(), 15_000), (untripped(), 20_000)] {
        let mut engine = build_src(&src, "newton");
        tick(&mut engine, 1);
        let start = engine.snapshot();
        let mut stock_change = 0.0;
        let mut reversed = 0;
        for _ in 0..ticks {
            tick(&mut engine, 1);
            let s = engine.snapshot();
            pass_books_close(&s);
            if flow(&s, PASS_OUT) < 0.0 {
                reversed += 1;
            }
            stock_change +=
                (flow(&s, "makeup_inlet") - flow(&s, "vent_line") - consumed(&s)) * DT_S;
        }
        let end = engine.snapshot();
        let moved = separator_kg(&end) - separator_kg(&start);
        // Every zero-volume node between the separator and the pass balances
        // to the solve's tolerance on every tick, not exactly.
        let allowance = 10.0 * TOL_ABS * DT_S * ticks as f64;
        assert!(
            (moved - stock_change).abs() < allowance,
            "the separator moved {moved} kg against {stock_change} kg booked"
        );
        if src != DEMO {
            assert!(
                reversed > 100,
                "the untripped story pulls gas back in through the pass's outlet on {reversed} \
                 ticks"
            );
        }
    }
}

/// The story to minute `minutes`, a reading each minute: (bed °C, factor,
/// separator bar, wall, trip tick, consumed kg/s, makeup kg/s).
#[allow(clippy::type_complexity)]
fn story(
    src: &str,
    solver: &str,
    minutes: usize,
) -> Vec<(f64, f64, f64, WallState, Option<u64>, f64, f64)> {
    let mut engine = build_src(src, solver);
    (0..minutes)
        .map(|_| {
            tick(&mut engine, 600);
            let s = engine.snapshot();
            (
                bed_k(&s) - KELVIN,
                node(&s, "bed").hydrogen_factor.expect("a bed with a pass"),
                node(&s, "separator").pressure_pa / 1e5,
                wall(&s),
                tripped(&s),
                consumed(&s),
                flow(&s, "makeup_inlet"),
            )
        })
        .collect()
}

/// 4. The file's story. The 395 °C feed creeps; the trip fires at 420 °C, about
///    6 min 20 s in, and only shuts the makeup. The hotter bed eats the loop
///    down — below 40 bar by 20 minutes — and starves itself: it turns near
///    521 °C, under its wall's 550 °C, and is back near its feed at 40 minutes,
///    the reaction nearly stopped and the loop near its 1.2 bar floor (M59.4;
///    under 1 bar before it). Without the trip the makeup valve, wide open,
///    cannot keep up with a run-away bed's demand, but the loop sags too late:
///    the wall fails about 22 minutes in, and by the hour the bed burns at
///    about 517 °C with its loop held where the makeup just feeds the demand.
#[test]
fn cutting_the_makeup_starves_the_runaway() {
    let cut = story(DEMO, "newton", 40);
    let trip_tick = cut[6].4.expect("tripped by minute 7");
    assert!(
        (3_700..3_900).contains(&trip_tick),
        "trip at tick {trip_tick}"
    );
    assert!(cut[19].2 < 40.0, "20 min: {} bar", cut[19].2);
    let peak = cut.iter().map(|r| r.0).fold(f64::NEG_INFINITY, f64::max);
    assert!((510.0..535.0).contains(&peak), "peak {peak} °C");
    assert!(cut.iter().all(|r| r.3 == WallState::Intact));
    let last = cut[39];
    assert!(last.0 < 400.0, "40 min: {} °C", last.0);
    assert!(last.1 < 0.02, "40 min: the reaction runs at {}", last.1);
    // Near the floor (M59.4, §64.13): 1.25 bar, where it was under 1 bar.
    assert!(
        (1.2..1.3).contains(&last.2),
        "40 min: the loop at {} bar",
        last.2
    );

    let open = story(&untripped(), "newton", 60);
    assert!(open.iter().all(|r| r.4.is_none()));
    let failed_by = open
        .iter()
        .position(|r| r.3 != WallState::Intact)
        .expect("the wall fails");
    assert!(
        (21..24).contains(&failed_by),
        "the wall fails in minute {}",
        failed_by + 1
    );
    // The demand outran the makeup on the way up...
    let worst = open.iter().map(|r| r.5).fold(0.0, f64::max);
    let makeup = open.iter().map(|r| r.6).fold(0.0, f64::max);
    assert!(worst > 2.0 * makeup, "{worst} kg/s against {makeup} kg/s");
    // ...and by the hour the loop stands where the makeup feeds the demand.
    let hour = open[59];
    assert!((510.0..525.0).contains(&hour.0), "60 min: {} °C", hour.0);
    assert!(
        (hour.5 - hour.6).abs() < 0.01 * hour.6,
        "60 min: {} kg/s consumed against {} kg/s of makeup",
        hour.5,
        hour.6
    );
    assert!((2.0..10.0).contains(&hour.2), "60 min: {} bar", hour.2);
}

/// 5. The game solver tells the same story, through the sag and the turn.
#[test]
fn the_game_solver_agrees_through_the_sag() {
    let newton = story(DEMO, "newton", 30);
    let simple = story(DEMO, "simple", 30);
    for (minute, (n, s)) in newton.iter().zip(&simple).enumerate() {
        assert!(
            (n.0 - s.0).abs() < 0.05,
            "minute {}: {} °C against {} °C",
            minute + 1,
            n.0,
            s.0
        );
        assert!(
            (n.2 - s.2).abs() < 0.5,
            "minute {}: {} bar against {} bar",
            minute + 1,
            n.2,
            s.2
        );
        assert_eq!(n.4, s.4, "the trip fires on the same tick");
    }
}

/// 6. A loop starved of its makeup comes to rest ABOVE the factor's floor: on
///    the ramp the demand falls with the pass's pressure left above 1.2 bar, so
///    the pass closes on the floor and never crosses it. By 90 minutes the bed
///    draws a ten-thousandth of what it drew at the height of the sag, the pass
///    stands within a hundredth of the floor above it, and the loop's lowest
///    point, the separator on the compressor's suction, stays above the
///    atmosphere by more than 15 kPa (M59.4, docs/DESIGN.md §64.13) — a hole
///    anywhere in the loop blows outward. It once rested at 1 kPa (M59.2).
#[test]
fn a_starved_loop_comes_to_rest() {
    let mut engine = build_src(DEMO, "newton");
    tick(&mut engine, 48_000);
    let at_80 = engine.snapshot();
    tick(&mut engine, 6_000);
    let s = engine.snapshot();
    let bed = node(&s, "bed");
    let factor = bed.hydrogen_factor.expect("a bed with a pass");
    assert!((0.0..1e-4).contains(&factor), "factor {factor}");
    // A ten-thousandth of the 1.5 kg/s the bed drew at the height of the sag.
    assert!(demand(&s) < 1e-4, "{} kg/s", demand(&s));
    let floor = HYDROGEN_FACTOR_ZERO_AT.value();
    let p = node(&s, "pass").pressure_pa;
    let p_80 = node(&at_80, "pass").pressure_pa;
    assert!(p > floor && p < 1.01 * floor, "the pass at {p} Pa");
    // Closing on the floor from above, never crossing it.
    assert!(p < p_80, "{p} Pa against {p_80} Pa ten minutes before");
    let lowest = node(&s, "separator").pressure_pa;
    assert!(
        lowest > P_ATM.value() + 15.0e3,
        "the separator at {lowest} Pa"
    );
    let stock = separator_kg(&s);
    assert!(stock > 1e6 * TOL_ABS * DT_S, "{stock} kg held");
    assert!((bed_k(&s) - KELVIN - 395.0).abs() < 0.5, "at its feed");
}

/// 7. The factor at low pressure, by hand: the power law from 1.8 bar up, a
///    straight line from there to zero at 1.2 bar (M59.4's floor, above the
///    atmosphere), zero below — continuous at both ends.
#[test]
fn the_factor_ramps_to_zero_above_the_atmosphere() {
    let kinetics = HydrogenKinetics {
        order: std::hint::black_box(0.5),
        reference_pressure: Pascal(155.0e5),
    };
    let ramp_from = HYDROGEN_FACTOR_RAMP_FROM.value();
    let zero_at = HYDROGEN_FACTOR_ZERO_AT.value();
    assert_eq!((ramp_from, zero_at), (1.8e5, 1.2e5));
    let top = (ramp_from / 155.0e5).powf(std::hint::black_box(0.5));
    for p in [ramp_from, 2.0e5, 150.0e5, 155.0e5] {
        assert_eq!(
            kinetics.factor(Pascal(p)).to_bits(),
            (p / 155.0e5).powf(std::hint::black_box(0.5)).to_bits(),
            "{p} Pa: the power law"
        );
    }
    for p in [1.5e5, 1.25e5, 1.2e5 + 1e-6] {
        let by_hand = top * (p - zero_at) / (ramp_from - zero_at);
        assert!(
            (kinetics.factor(Pascal(p)) - by_hand).abs() < 1e-15,
            "{p} Pa: the ramp"
        );
    }
    let just_below = kinetics.factor(Pascal(ramp_from * (1.0 - 1e-12)));
    assert!((just_below - top).abs() < 1e-12, "continuous at 1.8 bar");
    for p in [zero_at, P_ATM.value(), 10.0, 0.0, -5.0] {
        assert_eq!(kinetics.factor(Pascal(p)), 0.0, "{p} Pa: nothing reacts");
    }
}

/// 8. The loader's refusals.
#[test]
fn the_loader_refuses_a_misdeclared_uptake() {
    let table = "hydrogen_uptake_kg_per_kg = { distillate = 0.020, naphtha = 0.035, gas = 0.070 }";
    let e = refused(&edit(DEMO, table, "hydrogen_uptake_kg_per_kg = {}"));
    assert!(e.contains("empty hydrogen_uptake_kg_per_kg"), "{e}");
    let e = refused(&edit(
        DEMO,
        table,
        "hydrogen_uptake_kg_per_kg = { kerosene = 0.02 }",
    ));
    assert!(e.contains("'kerosene', which is not in the slate"), "{e}");
    let e = refused(&edit(
        DEMO,
        table,
        "hydrogen_uptake_kg_per_kg = { distillate = -0.02 }",
    ));
    assert!(e.contains("finite and >= 0"), "{e}");
    let e = refused(&edit(
        DEMO,
        table,
        "hydrogen_uptake_kg_per_kg = { hydrogen = 0.02 }",
    ));
    assert!(e.contains("which is a gas"), "{e}");
    // Naphtha is lighter than distillate and may not consume less.
    let e = refused(&edit(
        DEMO,
        table,
        "hydrogen_uptake_kg_per_kg = { distillate = 0.035, naphtha = 0.020, gas = 0.070 }",
    ));
    assert!(e.contains("must not fall as the lumps get lighter"), "{e}");
    // M59.3 (§64.12): a lump the table leaves out counts as zero and is
    // ordered too — naphtha's zero falls below distillate's 0.020.
    let e = refused(&edit(
        DEMO,
        table,
        "hydrogen_uptake_kg_per_kg = { distillate = 0.020 }",
    ));
    assert!(
        e.contains("'naphtha', which the table leaves out and so counts as zero,"),
        "{e}"
    );
    assert!(e.contains("must not fall as the lumps get lighter"), "{e}");
    // The datum may carry a figure, below the lighter lumps'.
    let e = refused(&edit(
        DEMO,
        table,
        "hydrogen_uptake_kg_per_kg = { vgo = 0.030, distillate = 0.020, naphtha = 0.035, gas = 0.070 }",
    ));
    assert!(
        e.contains("per kg of 'vgo', which the reaction cracks into it"),
        "{e}"
    );
    refinery_scenarios::build_engine(
        &refinery_scenarios::load_str(&edit(
            DEMO,
            table,
            "hydrogen_uptake_kg_per_kg = { vgo = 0.010, distillate = 0.020, naphtha = 0.035, gas = 0.070 }",
        ))
        .unwrap(),
    )
    .unwrap_or_else(|e| panic!("an ordered datum figure loads: {e}"));
    // Cooling water is in the slate and in liquid service, and inert.
    let e = refused(&edit(
        DEMO,
        table,
        "hydrogen_uptake_kg_per_kg = { distillate = 0.020, naphtha = 0.035, gas = 0.070, water = 0.1 }",
    ));
    assert!(
        e.contains("'water', which reactions = \"hydrocracking\" neither forms"),
        "{e}"
    );
    // A chemistry with no paths prices nothing.
    let e = refused(&edit(
        DEMO,
        r#"reactions = "hydrocracking""#,
        r#"reactions = "none""#,
    ));
    assert!(
        e.contains("names no path along which its lumps convert"),
        "{e}"
    );
}

/// 9. A bed fed PRODUCTS — the second bed's lot — consumes on every tick.
///
/// M59.3 (§64.12). M59.2's loader admitted a table that left lumps out, and a
/// bed fed pure distillate under `{ distillate = 0.020 }` refused its first
/// tick; under an ordered table every feed draws non-negative hydrogen.
#[test]
fn a_bed_fed_products_consumes() {
    for feed in [
        "{ distillate = 1.0 }",
        "{ naphtha = 1.0 }",
        "{ gas = 1.0 }",
        "{ vgo = 0.5, distillate = 0.3, naphtha = 0.15, gas = 0.05 }",
    ] {
        let src = edit(
            DEMO,
            "temperature_c = 395.0\ncomposition = { vgo = 1.0 }",
            &format!("temperature_c = 395.0\ncomposition = {feed}"),
        );
        let mut engine = build_src(&src, "newton");
        for _ in 0..600 {
            engine
                .tick()
                .unwrap_or_else(|e| panic!("fed {feed}, a tick refused: {e}"));
            let d = demand(&engine.snapshot());
            assert!(d >= 0.0, "fed {feed}: a demand of {d} kg/s");
        }
    }
}
