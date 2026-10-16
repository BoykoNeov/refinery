//! M59.1: hydrogen in the catalyst bed — a recycle loop through a hydrogen pass
//! beside the bed, the factor its pressure sets on every rate, and
//! depressuring (docs/DESIGN.md §64), gated on
//! `scenarios/hydrocracker_hydrogen.toml` and fixtures cut from it.
//!
//! What is pinned, and against what:
//!
//! 1. the factor is `(p/p_ref)^n` of the pass's own solved pressure, to the bit;
//! 2. at a factor of exactly 1 the chemistry is the calibrated bed's, to the bit
//!    — the hydrogen changes nothing it is not meant to;
//! 3. the bed's first law closes with both streams: `C·ΔT/dt = Q_rxn +
//!    ṁ_oil·cp·(T_oil,in − T_out) + Q_H2`, and `Q_H2 = ṁ_H2·cp_H2·(T_H2,in −
//!    T_out)`, both leaving at one temperature;
//! 4. a stopped compressor carries no hydrogen and no heat, and the bed still
//!    reads its pass's pressure;
//! 5. the story: the 395 °C feed creeps, the trip depressures the loop at the
//!    high-rate system's pace, and the starved bed turns and comes back, its
//!    wall intact — where the same plant without the trip runs on;
//! 6. the game solver tells the same story to the trip;
//! 7. the loader's refusals.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{NodeKind, TripState, WallState};
use refinery_core::snapshot::{NodeSnapshot, Snapshot};
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/hydrocracker_hydrogen.toml");

/// The demo's numbers, by hand from its file.
const C_BED_J_PER_K: f64 = 30.0e6;
const CP_OIL: f64 = 3200.0;
const CP_H2: f64 = 14_300.0;
/// The order, read through `black_box` at each use: as a constant, a release
/// build turns `powf(x, 0.5)` into `sqrt(x)`, one ulp from the engine's `powf`.
const ORDER: f64 = 0.5;
const P_REF_PA: f64 = 155.0e5;
const DT_S: f64 = 0.1;
const KELVIN: f64 = 273.15;

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

fn flow(s: &Snapshot, edge: &str) -> f64 {
    s.edges
        .iter()
        .find(|e| e.name == edge)
        .unwrap_or_else(|| panic!("an edge '{edge}'"))
        .stream
        .mass_flow
        .value()
}

fn edge_k(s: &Snapshot, edge: &str) -> f64 {
    s.edges
        .iter()
        .find(|e| e.name == edge)
        .unwrap_or_else(|| panic!("an edge '{edge}'"))
        .stream
        .temperature
        .value()
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

/// 1. The bed's factor is `(p/p_ref)^n` of its pass's solved pressure, to the
///    bit, on every tick of the creep.
#[test]
fn the_factor_is_the_pass_pressure_to_the_half() {
    let mut engine = build_src(DEMO, "newton");
    for _ in 0..50 {
        tick(&mut engine, 1);
        let s = engine.snapshot();
        let pressure = node(&s, "pass").pressure_pa;
        let factor = node(&s, "bed").hydrogen_factor.expect("a bed with a pass");
        assert_eq!(
            factor.to_bits(),
            (pressure / P_REF_PA)
                .powf(std::hint::black_box(ORDER))
                .to_bits(),
            "tick {}: {factor} at {pressure} Pa",
            s.tick
        );
    }
    // Near the design pressure the factor is near 1: the calibration holds.
    let s = engine.snapshot();
    let factor = node(&s, "bed").hydrogen_factor.expect("a bed with a pass");
    assert!((0.97..1.02).contains(&factor), "{factor}");
}

/// 2. A factor of exactly 1 — an order so small that every pressure's power of
///    it rounds to 1 — gives the calibrated bed's chemistry to the bit: the
///    same reaction heat as the bed with its pass taken out, on the tick both
///    start from one state.
#[test]
fn a_factor_of_one_is_the_calibrated_bed() {
    let unit = edit(DEMO, "hydrogen_order = 0.5", "hydrogen_order = 1e-300");
    let mut with_pass = build_src(&unit, "newton");
    tick(&mut with_pass, 1);
    let s = with_pass.snapshot();
    assert_eq!(node(&s, "bed").hydrogen_factor, Some(1.0));
    // The pass out: the gas runs from the heater straight to the cooler.
    let without = edit(
        &unit,
        "[nodes.pass]\ntype = \"bed_hydrogen\"\nbed = \"bed\"\nhydrogen_order = 1e-300\nhydrogen_reference_bar = 155.0\n",
        "[nodes.pass]\ntype = \"junction\"\n",
    );
    let mut bare = build_src(&without, "newton");
    tick(&mut bare, 1);
    let b = bare.snapshot();
    assert_eq!(node(&b, "bed").hydrogen_factor, None);
    assert_eq!(
        flow(&s, "valve_line").to_bits(),
        flow(&b, "valve_line").to_bits()
    );
    assert_eq!(
        node(&s, "bed").reaction_heat_w.map(f64::to_bits),
        node(&b, "bed").reaction_heat_w.map(f64::to_bits),
        "one oil flow, one bed temperature, f = 1: one chemistry"
    );
}

/// 3. The bed's first law closes with both streams, every tick, and both
///    leave at one temperature: the pass's and the bed's fluid.
#[test]
fn the_first_law_closes_with_the_hydrogen() {
    let mut engine = build_src(DEMO, "newton");
    tick(&mut engine, 1);
    let mut before = engine.snapshot();
    for _ in 0..200 {
        tick(&mut engine, 1);
        let s = engine.snapshot();
        let out = node(&s, "bed").temperature_k;
        assert_eq!(
            node(&s, "pass").temperature_k.to_bits(),
            out.to_bits(),
            "tick {}: the hydrogen and the oil leave at one temperature",
            s.tick
        );
        let stored = C_BED_J_PER_K * (bed_k(&s) - bed_k(&before)) / DT_S;
        let reaction = node(&s, "bed").reaction_heat_w.expect("a bed");
        let hydrogen = node(&s, "bed").hydrogen_heat_w.expect("a bed with a pass");
        let oil = flow(&s, "valve_line") * CP_OIL * (edge_k(&s, "valve_line") - out);
        let gas = flow(&s, "reactor_gas_line__downstream")
            * CP_H2
            * (edge_k(&s, "reactor_gas_line__downstream") - out);
        let scale = reaction.abs() + oil.abs() + gas.abs();
        assert!(
            (hydrogen - gas).abs() < 1e-9 * scale,
            "tick {}: the hydrogen's heat {hydrogen} W is its own flow's {gas} W",
            s.tick
        );
        assert!(
            (stored - (reaction + oil + hydrogen)).abs() < 1e-6 * scale,
            "tick {}: stored {stored} W against {reaction} + {oil} + {hydrogen} W",
            s.tick
        );
        before = s;
    }
}

/// 3b. With its oil stopped, a bed cools toward its hydrogen exactly: with the
///     inflow frozen across a tick, `T_end − T_in = (T_start − T_in)·e^(−x)`,
///     `x = ṁ_H2·cp_H2·dt/C` — the step's exactness, which the books above
///     cannot see (they close for any capacity rate). And it is what lets a
///     trip stop a hydrogen bed's feed and still clear (§64.7).
#[test]
fn a_stopped_bed_cools_toward_its_hydrogen_exactly() {
    let src = edit(
        &edit(
            DEMO,
            "bed_temperature_c = 395.0",
            "bed_temperature_c = 450.0",
        ),
        "kv = 110.0\nopening = 1.0",
        "kv = 110.0\nopening = 0.0",
    );
    let mut engine = build_src(&src, "newton");
    tick(&mut engine, 1);
    let mut before = engine.snapshot();
    for _ in 0..20 {
        tick(&mut engine, 1);
        let s = engine.snapshot();
        assert_eq!(flow(&s, "valve_line"), 0.0, "the oil is stopped");
        let gas = flow(&s, "reactor_gas_line__downstream");
        let inlet = edge_k(&s, "reactor_gas_line__downstream");
        let x = gas * CP_H2 * DT_S / C_BED_J_PER_K;
        let start = bed_k(&before);
        let expected = inlet + (start - inlet) * (-x).exp();
        assert!(
            (bed_k(&s) - expected).abs() < 1e-9 * (start - inlet).abs(),
            "tick {}: {} K against {expected} K from {start} K toward {inlet} K",
            s.tick,
            bed_k(&s)
        );
        before = s;
    }
    assert!(
        bed_k(&before) < 450.0 + KELVIN,
        "the hydrogen cools the bed"
    );
}

/// 4. A stopped compressor carries no hydrogen and no heat; the bed still
///    reads the pressure its pass stands at.
#[test]
fn a_stopped_compressor_carries_no_hydrogen() {
    let src = edit(
        DEMO,
        "polytropic_efficiency = 0.75\n",
        "polytropic_efficiency = 0.75\non = false\n",
    );
    let mut engine = build_src(&src, "newton");
    tick(&mut engine, 20);
    let s = engine.snapshot();
    assert!(
        flow(&s, "reactor_gas_outlet").abs() < 1e-6,
        "{} kg/s",
        flow(&s, "reactor_gas_outlet")
    );
    let bed = node(&s, "bed");
    assert!(bed.hydrogen_heat_w.expect("a bed with a pass").abs() < 1.0);
    let pressure = node(&s, "pass").pressure_pa;
    assert_eq!(
        bed.hydrogen_factor.map(f64::to_bits),
        Some(
            (pressure / P_REF_PA)
                .powf(std::hint::black_box(ORDER))
                .to_bits()
        )
    );
}

/// The story to minute `minutes`, a reading each minute: (bed °C, factor,
/// separator bar, wall, trip tick).
fn story(src: &str, solver: &str, minutes: usize) -> Vec<(f64, f64, f64, WallState, Option<u64>)> {
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
            )
        })
        .collect()
}

/// 5. The file's story. The 395 °C feed creeps; the trip fires at 420 °C,
///    6 min 20 s in, opening the vent and shutting the makeup; 15 minutes
///    later the loop stands near a fifth of its pressure — the high-rate
///    system's pace (thepetrosolutions, §64.4) — and the bed, starved, has
///    turned below 470 °C; at 40 minutes it is back near its feed, its wall
///    intact all the way. Without the trip the same bed runs on past them.
#[test]
fn slow_depressuring_catches_the_excursion() {
    let tripped_story = story(DEMO, "newton", 40);
    let trip_tick = tripped_story[6].4.expect("tripped by minute 7");
    assert!(
        (3_700..3_900).contains(&trip_tick),
        "trip at tick {trip_tick}"
    );
    // 15 minutes after the trip: a fifth of the 155 bar design, give or take.
    let at_15 = tripped_story[(trip_tick as usize + 9_000) / 600 - 1].2;
    assert!((25.0..40.0).contains(&at_15), "{at_15} bar 15 min on");
    let peak = tripped_story
        .iter()
        .map(|r| r.0)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!((440.0..470.0).contains(&peak), "peak {peak} °C");
    let last = tripped_story[39];
    assert!(last.0 < 415.0, "40 min: {} °C", last.0);
    assert!(
        last.1 < 0.2,
        "40 min: the reaction runs at {} of itself",
        last.1
    );
    assert!(tripped_story.iter().all(|r| r.3 == WallState::Intact));

    let untripped = edit(DEMO, "limit_c = 420.0", "limit_c = 2000.0");
    let runaway = story(&untripped, "newton", 25);
    assert!(runaway.iter().all(|r| r.4.is_none()));
    assert!(
        runaway[24].0 > tripped_story[24].0 + 50.0,
        "25 min: {} °C left alone against {} °C depressured",
        runaway[24].0,
        tripped_story[24].0
    );
}

/// 6. The game solver tells the same story to just past the trip.
#[test]
fn slow_the_game_solver_agrees_to_the_trip() {
    let newton = story(DEMO, "newton", 7);
    let simple = story(DEMO, "simple", 7);
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

/// 7. The loader's refusals.
#[test]
fn the_loader_refuses_a_misbuilt_hydrogen_loop() {
    // A pass naming a node that does not exist, and one that is not a bed.
    let e = refused(&edit(DEMO, "bed = \"bed\"", "bed = \"nowhere\""));
    assert!(e.contains("not a node"), "{e}");
    let e = refused(&edit(DEMO, "bed = \"bed\"", "bed = \"separator\""));
    assert!(e.contains("not a bed_reactor"), "{e}");
    // The order and the reference pressure, declared and positive.
    let e = refused(&edit(DEMO, "hydrogen_order = 0.5", "hydrogen_order = 0.0"));
    assert!(e.contains("hydrogen_order"), "{e}");
    let e = refused(&edit(
        DEMO,
        "hydrogen_reference_bar = 155.0",
        "hydrogen_reference_bar = -1.0",
    ));
    assert!(e.contains("hydrogen_reference_bar"), "{e}");
    // The compressor's curve.
    let e = refused(&edit(
        DEMO,
        "head_kj_per_kg = 120.0",
        "head_kj_per_kg = 0.0",
    ));
    assert!(e.contains("head_kj_per_kg"), "{e}");
    let e = refused(&edit(
        DEMO,
        "polytropic_efficiency = 0.75",
        "polytropic_efficiency = 1.5",
    ));
    assert!(e.contains("polytropic_efficiency"), "{e}");
    // A second pass through the same bed.
    let e = refused(&edit(
        DEMO,
        "[nodes.gas_cooler]",
        "[nodes.second_pass]\ntype = \"bed_hydrogen\"\nbed = \"bed\"\nhydrogen_order = 0.5\nhydrogen_reference_bar = 155.0\n\n[nodes.gas_cooler]",
    ));
    assert!(e.contains("two hydrogen passes"), "{e}");
    // A compressor in liquid service: on the oil line.
    let e = refused(&edit(
        DEMO,
        "[nodes.feed_valve]\ntype = \"valve\"\nkv = 110.0\nopening = 1.0\n",
        "[nodes.feed_valve]\ntype = \"compressor\"\nhead_kj_per_kg = 1.0\nhead_curve_kj_per_kg_s2_per_m6 = 0.0\npolytropic_efficiency = 0.75\n",
    ));
    assert!(
        e.contains("compressor 'feed_valve' is in liquid service"),
        "{e}"
    );
    // A hydrogen pass in liquid service: the gas loop's pass made a junction,
    // and a pass put on the oil line ahead of the bed.
    let liquid_pass = edit(
        &edit(
            &edit(
                DEMO,
                "[nodes.pass]\ntype = \"bed_hydrogen\"\nbed = \"bed\"\nhydrogen_order = 0.5\nhydrogen_reference_bar = 155.0\n",
                "[nodes.pass]\ntype = \"junction\"\n",
            ),
            "[[pipes]]\nname = \"valve_line\"\nfrom = \"feed_valve\"\nto = \"bed\"",
            "[[pipes]]\nname = \"valve_line\"\nfrom = \"feed_valve\"\nto = \"oil_pass\"\nlength_m = 5.0\ndiameter_m = 0.25\n\n[[pipes]]\nname = \"oil_pass_line\"\nfrom = \"oil_pass\"\nto = \"bed\"",
        ),
        "[nodes.product]",
        "[nodes.oil_pass]\ntype = \"bed_hydrogen\"\nbed = \"bed\"\nhydrogen_order = 0.5\nhydrogen_reference_bar = 155.0\n\n[nodes.product]",
    );
    let e = refused(&liquid_pass);
    assert!(
        e.contains("hydrogen pass 'oil_pass' is in liquid service"),
        "{e}"
    );
}
