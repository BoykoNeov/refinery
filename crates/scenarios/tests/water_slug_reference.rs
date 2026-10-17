//! M62.3 — a tank's water layer (`scenarios/water_slug.toml`, docs/DESIGN.md
//! §67.3): free water stands at a tank's bottom, under the oil, and the outlet
//! there draws it first. The crude tank starts 8 m full with 4 % free water by
//! mass and drains by gravity through a valve into a second tank.
//!
//! These gates hold: the layer on the hand calculation at load; the outlet
//! pure water until the layer is gone, the last of it and the first oil on one
//! tick, oil after; the flow on ISA-75.01 and Darcy–Weisbach at the density the
//! outlet carries, water and oil alike; every component's books each tick; the
//! game solver draining the same slug; a dry tank on a wet slate unchanged to
//! the bit; two outlets sharing one layer; and a wet tank spilling its oil, not
//! its water, over the brim.

use refinery_core::graph::{NodeKind, TankState};
use refinery_core::snapshot::{EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::Engine;

const PLANT: &str = include_str!("../../../scenarios/water_slug.toml");
const DT: f64 = 0.5;
const WATER: usize = 2;
const RHO_WATER: f64 = 998.0;

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the scenario must parse");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant must build: {e}"))
}

fn tick(engine: &mut Engine, t: u64) -> Snapshot {
    engine
        .tick()
        .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
    let s = engine.snapshot();
    assert!(s.solver.converged, "tick {t} did not converge");
    s
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn edge<'a>(s: &'a Snapshot, name: &str) -> &'a EdgeSnapshot {
    s.edges.iter().find(|e| e.name == name).unwrap()
}

fn tank<'a>(s: &'a Snapshot, name: &str) -> &'a TankState {
    match &node(s, name).kind {
        NodeKind::Tank(t) => t,
        _ => panic!("'{name}' is a tank"),
    }
}

/// One tank's mass [kg] of component `c`.
fn held(t: &TankState, c: usize) -> f64 {
    t.mass.value() * t.composition.fractions()[c]
}

/// The oil's own density [kg/m³]: the 50/50 kerosene and diesel behind the
/// water, by ideal mixing.
fn rho_oil() -> f64 {
    1.0 / (0.5 / 800.0 + 0.5 / 850.0)
}

/// What the crude tank's outlet carried over each of `ticks` ticks: the outlet
/// stream's mass flow [kg/s] and water mass fraction, and the drive across the
/// line [Pa] — the crude tank's bottom pressure less the receiving tank's.
fn drain(src: &str, ticks: u64) -> (Engine, Vec<(f64, f64, f64)>) {
    let mut engine = build(src);
    let mut rows = Vec::new();
    for t in 1..=ticks {
        let s = tick(&mut engine, t);
        let outlet = &edge(&s, "tank_outlet").stream;
        rows.push((
            outlet.mass_flow.value(),
            outlet.composition.fractions()[WATER],
            node(&s, "crude_tank").pressure_pa - node(&s, "receiving_tank").pressure_pa,
        ));
    }
    (engine, rows)
}

/// The tick (1-based) the water layer ran out on: the one tick whose outlet
/// carried water and oil together. Exactly one such tick, pure water before it
/// and pure oil after.
fn transition(rows: &[(f64, f64, f64)]) -> usize {
    let blended: Vec<usize> = rows
        .iter()
        .enumerate()
        .filter(|(_, r)| r.1 > 0.0 && r.1 < 1.0)
        .map(|(i, _)| i + 1)
        .collect();
    assert_eq!(
        blended.len(),
        1,
        "the outlet carried water and oil together on ticks {blended:?}, not on one"
    );
    let k = blended[0];
    for (i, r) in rows.iter().enumerate() {
        let t = i + 1;
        if t < k {
            assert_eq!(
                r.1, 1.0,
                "tick {t}, before the layer ran out: the outlet carried {} water",
                r.1
            );
        } else if t > k {
            assert_eq!(
                r.1, 0.0,
                "tick {t}, after the layer ran out: the outlet carried {} water",
                r.1
            );
        }
    }
    k
}

/// **The layer on the hand calculation, and it leaves first.** At load the
/// crude tank holds `m = ρ·A·h` at the mixture's ideal-mixing density, 4 % of it
/// water: 796.4 kg, a layer `m_w/(ρ_w·A)` = 0.2661 m deep under 7.734 m of oil,
/// and the level is both layers. Then the outlet carries pure water until the
/// layer is gone — on the tick the cumulative draw `Σ ṁ·dt` first reaches what
/// stood — carrying exactly the water left and oil after it on that tick, and
/// pure oil after. The receiving tank gathers the water at its own bottom, to
/// the rounding of 800 ticks' books (1e-9); the crude tank keeps none.
#[test]
fn the_water_layer_leaves_first_then_the_oil() {
    let engine = build(PLANT);
    let s0 = engine.snapshot();
    let rho_mix = 1.0 / (0.48 / 800.0 + 0.48 / 850.0 + 0.04 / RHO_WATER);
    let mass = rho_mix * 3.0 * 8.0;
    let water = 0.04 * mass;
    let layer = water / (RHO_WATER * 3.0);
    let crude = tank(&s0, "crude_tank");
    assert!((held(crude, WATER) / water - 1.0).abs() < 1e-12);
    let reported = node(&s0, "crude_tank")
        .water_level_m
        .expect("a wet slate reports the layer");
    assert!(
        (reported / layer - 1.0).abs() < 1e-12,
        "the layer reads {reported} m, the hand calculation {layer} m"
    );
    let oil_height = (mass - water) / (rho_oil_of(&[0.48, 0.48]) * 3.0);
    assert!(
        ((layer + oil_height) / 8.0 - 1.0).abs() < 1e-12,
        "the water layer {layer} m and the oil {oil_height} m are not the 8 m level"
    );
    assert_eq!(node(&s0, "receiving_tank").water_level_m, Some(0.0));

    let (engine, rows) = drain(PLANT, 1000);
    let k = transition(&rows);
    let before: f64 = rows[..k - 1].iter().map(|r| r.0 * DT).sum();
    let on = rows[k - 1].0 * DT;
    assert!(
        before < water && water <= before + on,
        "the layer ran out on tick {k}, after {before} kg drawn of {water} kg (that tick drew {on})"
    );
    let share = (water - before) / on;
    assert!(
        (rows[k - 1].1 / share - 1.0).abs() < 1e-9,
        "on tick {k} the outlet carried {} water, what was left is {share} of the draw",
        rows[k - 1].1
    );
    // About 390 s of water at about 2 kg/s.
    assert!((700..=850).contains(&k), "the layer ran out on tick {k}");

    let s = engine.snapshot();
    assert_eq!(
        held(tank(&s, "crude_tank"), WATER),
        0.0,
        "the crude tank kept water"
    );
    assert_eq!(node(&s, "crude_tank").water_level_m, Some(0.0));
    let gathered = held(tank(&s, "receiving_tank"), WATER);
    assert!(
        (gathered / water - 1.0).abs() < 1e-9,
        "the receiving tank gathered {gathered} kg of the {water} kg of water"
    );
    let receiving_layer = node(&s, "receiving_tank").water_level_m.unwrap();
    assert!((receiving_layer / layer - 1.0).abs() < 1e-9);
}

/// The ideal-mixing density [kg/m³] of the kerosene and diesel at these mass
/// weights.
fn rho_oil_of(weights: &[f64; 2]) -> f64 {
    let total = weights[0] + weights[1];
    1.0 / (weights[0] / total / 800.0 + weights[1] / total / 850.0)
}

/// **The flow on the hand calculation, at the density the outlet carries.** The
/// valve passes `Q = Cv·x·√(ΔP/ρ_rel)` (ISA-75.01, `ρ_rel = ρ/998`) and the two
/// pipes drop `ΔP = f·L·ρ·Q²/(2·D·A²)` (Darcy–Weisbach, `f` = 0.02): each drops
/// a pressure `∝ ṁ²/ρ`, so `ρ·ΔP/ṁ²` is one constant,
/// `1/(998·(Cv·x)²) + f·L/(2·D·A²)` = 1.30427e7, whatever flows. The engine
/// holds it to 1e-4 on pure water, on the tick that mixes the last water with
/// the first oil, and on pure oil — so the slug is a step DOWN in the mass
/// flow by `√(ρ_oil/ρ_w)`, 0.909, when it ends.
#[test]
fn the_flow_follows_the_valve_and_the_line_at_the_density_carried() {
    let cv = 20.0 / (3600.0 * 1e5_f64.sqrt()) * 0.5;
    let area = std::f64::consts::PI * 0.1 * 0.1 / 4.0;
    let constant = 1.0 / (RHO_WATER * cv * cv) + 0.02 * 35.0 / (2.0 * 0.1 * area * area);
    let (_, rows) = drain(PLANT, 1000);
    let k = transition(&rows);
    for t in [1, 2, k / 2, k - 1, k, k + 1, k + 2, 1000] {
        let (flow, water, drive) = rows[t - 1];
        let rho = 1.0 / (water / RHO_WATER + (1.0 - water) / rho_oil());
        let measured = rho * drive / (flow * flow);
        assert!(
            (measured / constant - 1.0).abs() < 1e-4,
            "tick {t} ({water} water): ρ·ΔP/ṁ² = {measured}, the valve and the line {constant}"
        );
    }
    let step = rows[k + 1].0 / rows[k - 2].0;
    let root = (rho_oil() / RHO_WATER).sqrt();
    assert!(
        (step / root - 1.0).abs() < 2e-3,
        "the flow stepped by {step} as the slug ended, √(ρ_oil/ρ_w) = {root}"
    );
}

/// **Every component's books, every tick, through the slug.** Nothing enters
/// the crude tank and nothing boils, so each tick it loses, of each component,
/// exactly what its outlet carried — the water first, the oil only after — and
/// the two tanks together hold each component's starting mass.
#[test]
fn every_component_is_booked_through_the_slug() {
    let mut engine = build(PLANT);
    let s0 = engine.snapshot();
    let start: Vec<f64> = (0..3)
        .map(|c| held(tank(&s0, "crude_tank"), c) + held(tank(&s0, "receiving_tank"), c))
        .collect();
    let mut before = tank(&s0, "crude_tank").clone();
    for t in 1..=900 {
        let s = tick(&mut engine, t);
        let crude = tank(&s, "crude_tank");
        let outlet = &edge(&s, "tank_outlet").stream;
        for (c, total) in start.iter().enumerate() {
            let lost = held(&before, c) - held(crude, c);
            let carried = outlet.mass_flow.value() * outlet.composition.fractions()[c] * DT;
            assert!(
                (lost - carried).abs() <= 1e-9 * before.mass.value(),
                "tick {t}, component {c}: the crude tank lost {lost} kg, its outlet carried {carried}"
            );
            let now = held(crude, c) + held(tank(&s, "receiving_tank"), c);
            assert!(
                (now - total).abs() <= 1e-9 * total.max(1.0),
                "tick {t}, component {c}: the tanks hold {now} kg, they started with {total}"
            );
        }
        before = crude.clone();
    }
}

/// **The game solver drains the same slug.** On the simple flow solver the
/// layer runs out within a tick of the Newton solver's, the flow agrees to
/// 1e-3, and the water all reaches the receiving tank.
#[test]
fn the_game_solver_drains_the_same_slug() {
    let (_, newton) = drain(PLANT, 900);
    let (engine, simple) = drain(
        &PLANT.replace("flow = \"newton\"", "flow = \"simple\""),
        900,
    );
    let (k_newton, k_simple) = (transition(&newton), transition(&simple));
    assert!(
        k_newton.abs_diff(k_simple) <= 1,
        "the layer ran out on tick {k_newton} (Newton) and {k_simple} (game solver)"
    );
    for t in [1, k_newton / 2, 900] {
        assert!(
            (simple[t - 1].0 / newton[t - 1].0 - 1.0).abs() < 1e-3,
            "tick {t}: {} kg/s on the game solver, {} on Newton",
            simple[t - 1].0,
            newton[t - 1].0
        );
    }
    let s = engine.snapshot();
    assert_eq!(held(tank(&s, "crude_tank"), WATER), 0.0);
}

/// **A dry tank on a wet slate is the tank it always was, to the bit.** The
/// slug plant with no water in the crude tank, beside the same plant whose
/// water component does not declare `free_water`: every tank's mass,
/// temperature and composition and every pipe's flow agree exactly over 300
/// ticks. Only the snapshot differs — the wet slate reports a zero layer.
#[test]
fn a_dry_tank_on_a_wet_slate_is_unchanged_to_the_bit() {
    let dry = PLANT.replace(
        "composition = { kerosene = 0.48, diesel = 0.48, water = 0.04 }",
        "composition = { kerosene = 0.5, diesel = 0.5 }",
    );
    let not_free = dry.replace("free_water = true\n", "");
    assert_ne!(dry, PLANT);
    assert_ne!(not_free, dry);
    let (mut wet_slate, mut plain) = (build(&dry), build(&not_free));
    for t in 1..=300 {
        let (a, b) = (tick(&mut wet_slate, t), tick(&mut plain, t));
        for name in ["crude_tank", "receiving_tank"] {
            let (x, y) = (tank(&a, name), tank(&b, name));
            assert_eq!(
                x.mass.value().to_bits(),
                y.mass.value().to_bits(),
                "tick {t}, {name}"
            );
            assert_eq!(
                x.temperature.value().to_bits(),
                y.temperature.value().to_bits()
            );
            assert_eq!(x.composition.fractions(), y.composition.fractions());
            assert_eq!(node(&a, name).water_level_m, Some(0.0));
            assert_eq!(node(&b, name).water_level_m, None);
        }
        for (x, y) in a.edges.iter().zip(&b.edges) {
            assert_eq!(
                x.stream.mass_flow.value().to_bits(),
                y.stream.mass_flow.value().to_bits(),
                "tick {t}, pipe {}",
                x.name
            );
        }
    }
}

/// **Two outlets share one layer.** The crude tank given a second outlet, a
/// valve to a third tank: both carry the SAME stream each tick — water while
/// any stands, the same mix of the last water and the first oil on the tick
/// it runs out — so between them they never take more water than stood, and
/// the crude tank ends with none and never less than none.
#[test]
fn two_outlets_share_one_water_layer() {
    let src = format!(
        "{PLANT}\n\
         [nodes.second_valve]\ntype = \"valve\"\nkv = 10.0\nopening = 0.5\n\n\
         [nodes.third_tank]\ntype = \"tank\"\narea_m2 = 3.0\nheight_m = 10.0\n\
         initial_level_m = 1.0\ntemperature_c = 30.0\n\
         composition = {{ kerosene = 0.5, diesel = 0.5 }}\n\n\
         [[pipes]]\nname = \"second_outlet\"\nfrom = \"crude_tank\"\nto = \"second_valve\"\n\
         length_m = 5.0\ndiameter_m = 0.10\n\n\
         [[pipes]]\nname = \"second_line\"\nfrom = \"second_valve\"\nto = \"third_tank\"\n\
         length_m = 30.0\ndiameter_m = 0.10\n"
    );
    let mut engine = build(&src);
    let s0 = engine.snapshot();
    let water = held(tank(&s0, "crude_tank"), WATER);
    let mut blended = 0;
    for t in 1..=800 {
        let s = tick(&mut engine, t);
        let (first, second) = (
            &edge(&s, "tank_outlet").stream,
            &edge(&s, "second_outlet").stream,
        );
        assert_eq!(
            first.composition.fractions(),
            second.composition.fractions(),
            "tick {t}: the two outlets drew different layers"
        );
        let w = first.composition.fractions()[WATER];
        if w > 0.0 && w < 1.0 {
            blended += 1;
        }
        assert!(held(tank(&s, "crude_tank"), WATER) >= 0.0);
    }
    assert_eq!(
        blended, 1,
        "the layer ran out over {blended} ticks, not one"
    );
    let s = engine.snapshot();
    assert_eq!(held(tank(&s, "crude_tank"), WATER), 0.0);
    let gathered = held(tank(&s, "receiving_tank"), WATER) + held(tank(&s, "third_tank"), WATER);
    assert!(
        (gathered / water - 1.0).abs() < 1e-9,
        "the two tanks gathered {gathered} kg of the {water} kg that stood"
    );
    assert!(
        held(tank(&s, "third_tank"), WATER) > 0.0,
        "the second outlet drew no water"
    );
}

/// **A wet tank spills its oil over the brim, not its water.** The crude tank,
/// its outlet shut, filled past its brim by a supply of the same oil: its
/// overflow carries oil alone, its level comes back to the brim each tick, and
/// the water at its bottom stays where it was, to the kilogram.
#[test]
fn a_wet_tank_spills_its_oil_over_the_brim() {
    let src = PLANT
        .replace("initial_level_m = 8.0", "initial_level_m = 9.9")
        .replace("opening = 0.5", "opening = 0.0")
        + "\n[nodes.supply]\ntype = \"source\"\npressure_bar = 3.0\ntemperature_c = 30.0\n\
           composition = { kerosene = 0.5, diesel = 0.5 }\n\n\
           [[pipes]]\nname = \"fill_line\"\nfrom = \"supply\"\nto = \"crude_tank\"\n\
           length_m = 10.0\ndiameter_m = 0.05\n";
    let mut engine = build(&src);
    let s0 = engine.snapshot();
    let water = held(tank(&s0, "crude_tank"), WATER);
    let mut spilled = false;
    for t in 1..=400 {
        let s = tick(&mut engine, t);
        let overflow = &edge(&s, "crude_tank__overflow").stream;
        if overflow.mass_flow.value() > 0.0 {
            spilled = true;
            assert_eq!(
                overflow.composition.fractions()[WATER],
                0.0,
                "tick {t}: the overflow carried water off the top of a tank with oil above it"
            );
            let crude = tank(&s, "crude_tank");
            let level = crude.level(&engine.slate);
            assert!(
                (level.value() / crude.height.value() - 1.0).abs() < 1e-12,
                "tick {t}: a spilling tank stands at {} m, its brim is {} m",
                level.value(),
                crude.height.value()
            );
        }
        let now = held(tank(&s, "crude_tank"), WATER);
        assert!(
            (now / water - 1.0).abs() < 1e-12,
            "tick {t}: the tank holds {now} kg of water, it started with {water}"
        );
    }
    assert!(spilled, "the tank never reached its brim");
}
