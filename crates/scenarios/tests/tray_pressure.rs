//! M73.2 — each stage at its own pressure (docs/DESIGN.md §78.1, §78.9): on a
//! column that declares its trays, the top stage at the column's declared
//! 1.8 bar and each stage below higher by the drop fed back across the tray
//! above it, capped at 100 % of jet flood; draw and loop lines starting at
//! their stage's pressure. On M73.1's fixture (`tray_hydraulics.rs`: the loop
//! plant with five tray sections inline) and the boiling crude unit with
//! sections inline for its draw lines.

use super::tray_hydraulics::{
    assert_lists_agree, engine_of, fixture, on_solver, operation, run, set_reflux, PLANT,
};
use refinery_core::engine::Engine;
use refinery_core::snapshot::{Command, EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::units::{KgPerSec, Pascal, G};
use refinery_solvers::elements::pipe_resistance;

const COLUMN_PA: f64 = 1.8e5;
/// The loop plant's loops: name, draw stage, return stage.
const LOOPS: [(&str, usize, usize); 3] = [("kero", 4, 3), ("diesel", 7, 6), ("gas_oil", 10, 9)];
/// Each loop pump's curve: `H = 60 m − 1 000·Q²` (the plant's).
const PUMP_HEAD_M: f64 = 60.0;
const PUMP_A: f64 = 1000.0;
const PRODUCTS: [&str; 5] = [
    "naphtha_draw",
    "kerosene_draw",
    "diesel_draw",
    "gas_oil_draw",
    "residue_draw",
];

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn edge<'a>(s: &'a Snapshot, name: &str) -> &'a EdgeSnapshot {
    s.edges.iter().find(|e| e.name == name).unwrap()
}

/// The pressure stage `stage` (one-based) solved at, as published: a tray's
/// own.
fn tray_pressure(s: &Snapshot, stage: usize) -> f64 {
    operation(s).trays[stage - 1].pressure_pa
}

/// **The drop moves the column, and only the column** (M73.2, §78.8 finding
/// 1): the fixture against the loop plant it was made from, 300 ticks from
/// cold. The top tray at the column's declared 1.8 bar, each tray below
/// higher; the column node, which every line into it reads, still at 1.8 bar;
/// the bottom hotter by what the probe measured (5.5 K), the top unmoved;
/// the feed flashing less, 100 mbar higher (0.663 → 0.654); every product
/// within a fraction of a point.
fn the_drop_moves_the_column(solver: &str) {
    let mut with = engine_of(&on_solver(&fixture(), solver)).unwrap();
    let mut bare = engine_of(&on_solver(PLANT, solver)).unwrap();
    let (a, b) = (run(&mut with, 300), run(&mut bare, 300));
    let (trays, plain) = (operation(&a), operation(&b));
    assert_eq!(trays.trays[0].pressure_pa, COLUMN_PA);
    for pair in trays.trays.windows(2) {
        assert!(pair[1].pressure_pa > pair[0].pressure_pa, "{pair:?}");
    }
    assert_eq!(node(&a, "column").pressure_pa, COLUMN_PA);
    assert_eq!(node(&b, "column").pressure_pa, COLUMN_PA);
    let span = trays.trays[11].pressure_pa - COLUMN_PA;
    let bottom = trays.stage_temperatures_k[12] - plain.stage_temperatures_k[12];
    let top = trays.stage_temperatures_k[0] - plain.stage_temperatures_k[0];
    let flash = (trays.feed_vapour_fraction, plain.feed_vapour_fraction);
    let product = PRODUCTS
        .iter()
        .map(|name| {
            let (x, y) = (
                &edge(&a, name).stream.composition,
                &edge(&b, name).stream.composition,
            );
            x.fractions()
                .iter()
                .zip(y.fractions())
                .map(|(p, q)| (p - q).abs())
                .fold(0.0, f64::max)
        })
        .fold(0.0, f64::max);
    // Measured on both solvers: the last tray 111.8 mbar over the top, the
    // bottom +5.49 K, the top +0.03 K, the flash 0.6615 → 0.6532, every
    // product's every cut within 0.0016 of its mass fraction.
    assert!((1.05e4..1.20e4).contains(&span), "{span} Pa");
    assert!((5.0..6.0).contains(&bottom), "{bottom} K");
    assert!(top.abs() < 0.1, "{top} K");
    assert!(flash.0 < flash.1 - 5e-3, "{flash:?}");
    assert!(product < 3e-3, "{product}");
}

#[test]
fn the_drop_moves_the_column_newton() {
    the_drop_moves_the_column("newton");
}

#[test]
fn the_drop_moves_the_column_simple() {
    the_drop_moves_the_column("simple");
}

/// **Past flood the column runs on, its trays still read flooded** (the
/// user's decision, §78.8): walked from a reflux of 4 to 15 the column runs,
/// its stages' pressures rising by the drops capped at jet flood — the span
/// short of what the raw drops add up to — and the lists the raw readings'.
#[test]
fn past_flood_the_column_runs_on_capped() {
    let mut engine = engine_of(&fixture()).unwrap();
    run(&mut engine, 300);
    set_reflux(&mut engine, 15.0);
    let operation = operation(&run(&mut engine, 100));
    assert_lists_agree(&operation);
    assert!(operation.trays_flooding.len() >= 6, "{operation:?}");
    let raw: f64 = operation.trays.iter().map(|t| t.pressure_drop_pa).sum();
    let span = operation.trays[11].pressure_pa - COLUMN_PA;
    // Measured: the last tray 155.7 mbar over the top, the raw drops 373.
    assert!((1.4e4..1.75e4).contains(&span), "{span} Pa");
    assert!(raw > 3.0e4, "{raw} Pa");
}

/// `k·Q|Q| + ρ·g·Δz` [Pa], a pipe's drop at `mass_flow` (Darcy–Weisbach).
fn pipe_drop(engine: &Engine, name: &str, rho: f64, mass_flow: f64) -> f64 {
    let pipe = engine
        .graph
        .edge_ids()
        .map(|e| engine.graph.pipe(e))
        .find(|p| p.name == name)
        .unwrap();
    let k = pipe_resistance(
        pipe.friction_factor,
        pipe.length.value(),
        pipe.diameter.value(),
        rho,
    );
    let q = mass_flow / rho;
    k * q * q.abs() + rho * G * pipe.elevation_change.value()
}

/// **A loop's line marches from its draw stage's pressure** (§78.1): its
/// pump stands at the draw tray's pressure — as the column's previous
/// separation solved it, a tick late as the drawn liquid is — less its
/// suction's friction and 15 m fall, its cooler at the pump's curve less the
/// discharge's friction, each at the loop's last flow and drawn density, by
/// hand to 1e-9; and not at the column's 1.8 bar, which M72.2 marched from.
#[test]
fn a_loop_line_marches_from_its_draw_stage() {
    let mut engine = engine_of(&fixture()).unwrap();
    let slate = engine.slate.clone();
    run(&mut engine, 150);
    let before = engine.snapshot();
    let now = run(&mut engine, 1);
    for (name, draw, _) in LOOPS {
        let suction = format!("{name}_pa_suction");
        let last = edge(&before, &suction).stream.mass_flow.value();
        let drawn = &edge(&before, &suction).stream;
        let rho = |pressure: f64| {
            drawn
                .composition
                .density_at(&slate, Pascal(pressure), drawn.temperature)
                .unwrap()
                .value()
        };
        let start = tray_pressure(&before, draw);
        assert!(start > COLUMN_PA + 1.0e3, "{name}: {start} Pa");
        let pump = start - pipe_drop(&engine, &suction, rho(start), last);
        let published = node(&now, &format!("{name}_pa_pump")).pressure_pa;
        assert!(
            (published - pump).abs() < 1e-9 * pump,
            "{name}: the pump stands at {published} Pa, the march at {pump} Pa"
        );
        let density = rho(pump);
        let q = last / density;
        let rise = density * G * (PUMP_HEAD_M - PUMP_A * q * q.abs());
        let discharge = format!("{name}_pa_discharge");
        let cooler = pump + rise - pipe_drop(&engine, &discharge, density, last);
        let published = node(&now, &format!("{name}_pa_cooler")).pressure_pa;
        assert!(
            (published - cooler).abs() < 1e-9 * cooler,
            "{name}: the cooler stands at {published} Pa, the march at {cooler} Pa"
        );
    }
}

/// **A loop that falls short arrives at its return stage's pressure**
/// (§78.1): the diesel loop asked for 250 kg/s, more than its pump pushes
/// round, runs at the flow at which its return pipe, from the cooler's pinned
/// pressure, lands on its return tray's pressure (stage 6) — by hand to 1e-9 —
/// not on the column's 1.8 bar.
#[test]
fn a_short_loop_arrives_at_its_return_stage() {
    let mut engine = engine_of(&fixture()).unwrap();
    let slate = engine.slate.clone();
    run(&mut engine, 150);
    let column = engine.graph.find_node("column").unwrap();
    engine
        .apply(Command::SetLoopFlow {
            node: column,
            pumparound_loop: 1,
            flow: KgPerSec(250.0),
        })
        .unwrap();
    run(&mut engine, 10);
    let before = engine.snapshot();
    let now = run(&mut engine, 1);
    let delivered = operation(&now).loops[1].flow_kg_s;
    assert!(delivered > 95.0 && delivered < 250.0, "{delivered}");
    let back = &edge(&before, "diesel_pa_return").stream;
    let cooler = node(&now, "diesel_pa_cooler").pressure_pa;
    let rho = back
        .composition
        .density_at(&slate, Pascal(cooler), back.temperature)
        .unwrap()
        .value();
    let arrival = cooler - pipe_drop(&engine, "diesel_pa_return", rho, delivered);
    let target = tray_pressure(&before, 6);
    assert!(target > COLUMN_PA + 1.0e3, "{target} Pa");
    assert!(
        (arrival - target).abs() < 1e-9 * target,
        "arrives at {arrival} Pa against the return tray's {target} Pa"
    );
}

const BOILING: &str = include_str!("../../../scenarios/crude_unit_boiling.toml");

/// The boiling crude unit (`scenarios/crude_unit_boiling.toml`, untouched)
/// with its nine trays in two sections inline (7 m, and 8.5 m for the two
/// below the feed, where 7 m floods): its draws leave by lines, each through
/// a pump and a cooler to its tank.
fn boiling_with_trays() -> String {
    let plant = BOILING.replace("\r\n", "\n");
    let anchor = "feed_stage = 8\nreflux_ratio = 4.0\n";
    assert!(
        plant.contains(anchor),
        "the boiling plant's cascade block moved"
    );
    let section = |from: u32, to: u32, diameter: f64| {
        format!(
            "    {{ from = {from}, to = {to}, diameter_m = {diameter}, spacing_m = 0.61, \
             weir_height_m = 0.05, hole_diameter_m = 0.005, hole_area_share = 0.10, \
             downcomer_share = 0.12, clearance_m = 0.04, passes = 4 }},\n"
        )
    };
    plant.replace(
        anchor,
        &format!(
            "{anchor}trays = [\n{}{}]\n",
            section(1, 7, 7.0),
            section(8, 9, 8.5)
        ),
    )
}

/// **A draw line marches from its stage's pressure** (§78.1): the kerosene
/// pump stands at tray 3's pressure less its suction's friction and fall, and
/// the residue pump at the reboiler's — the last tray's pressure plus the
/// drop across it (its reading, the tray short of flood) — each at its line's
/// last flow and drawn density, by hand; not at the column's 1.8 bar.
///
/// The kerosene to 1e-9 (measured 3e-11); the residue to 1e-6 (measured
/// 3.4e-8, 0.011 Pa: its drawn temperature is the reboiler's to the bit, and
/// the rest is not traced here) — against 1.5 kPa had it started at the last
/// tray's pressure without the drop across it, about 10 kPa at the column's.
#[test]
fn a_draw_line_marches_from_its_stage() {
    let mut engine = engine_of(&boiling_with_trays()).unwrap();
    let slate = engine.slate.clone();
    run(&mut engine, 150);
    let before = engine.snapshot();
    let now = run(&mut engine, 1);
    let trays = &operation(&before).trays;
    assert_eq!(trays.len(), 9);
    assert!(trays[8].flood_fraction < 1.0, "{:?}", trays[8]);
    let reboiler = trays[8].pressure_pa + trays[8].pressure_drop_pa;
    for (name, start, tolerance) in [
        ("kerosene", trays[2].pressure_pa, 1e-9),
        ("residue", reboiler, 1e-6),
    ] {
        assert!(start > COLUMN_PA + 1.0e3, "{name}: {start} Pa");
        let suction = format!("{name}_suction");
        let drawn = &edge(&before, &suction).stream;
        let rho = drawn
            .composition
            .density_at(&slate, Pascal(start), drawn.temperature)
            .unwrap()
            .value();
        let pump = start - pipe_drop(&engine, &suction, rho, drawn.mass_flow.value());
        let published = node(&now, &format!("{name}_pump")).pressure_pa;
        assert!(
            (published - pump).abs() < tolerance * pump,
            "{name}: the pump stands at {published} Pa, the march at {pump} Pa"
        );
    }
}
