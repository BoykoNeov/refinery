//! M86.3 (docs/DESIGN.md §84.3): a check valve on a column's product line.
//! The march solves its disc's lift at the line's flow (the root of `S =
//! α(S)·Q²`, the drive `S` across the disc and its outlet pipe), and holds the
//! line at no flow where the store is out of the column's reach — the check
//! valve the model's lines only behaved as before (§84.2).
//!
//! The plants are inline fixtures: the crude unit with a product line on every
//! draw (`product_line_reference::crude_unit_lines`), a check valve put after
//! each pump's discharge pipe, a metre of pipe from it to the cooler.

use crate::product_line_reference::{
    build, crude_unit_lines, density, edge, node, pipe_drop, run, LINES, PUMP_A, PUMP_HEAD_M,
};
use crate::short_draw_line::{flow, needs_pump, set_pump, shares, tick, worst_imbalance};
use refinery_core::graph::NodeKind;
use refinery_core::traits::STOPPED_DRAW_SHARE;
use refinery_core::units::G;
use refinery_core::Engine;
use refinery_solvers::elements::check_opening;
use refinery_solvers::network::RHO_WATER_REF;

/// `src` with a check valve of `kv` [m³/h at 1 bar] and band `full_open_bar`
/// [bar] after each of `products`' discharge pipes.
fn with_check_valves(src: &str, products: &[&str], kv: f64, full_open_bar: f64) -> String {
    let mut src = src.to_string();
    for product in products {
        let old = format!(
            "name = \"{product}_discharge\"\nfrom = \"{product}_pump\"\nto = \"{product}_cooler\""
        );
        assert!(src.contains(&old), "the {product} discharge was not found");
        src = src.replacen(
            &old,
            &format!(
                "name = \"{product}_discharge\"\nfrom = \"{product}_pump\"\nto = \"{product}_check\""
            ),
            1,
        );
        src.push_str(&format!(
            r#"
[nodes.{product}_check]
type = "check_valve"
kv = {kv}
full_open_bar = {full_open_bar}

[[pipes]]
name = "{product}_check_outlet"
from = "{product}_check"
to = "{product}_cooler"
length_m = 1.0
diameter_m = 0.15
"#
        ));
    }
    src
}

const ALL: [&str; 5] = ["naphtha", "kerosene", "diesel", "gas_oil", "residue"];

/// The check valve's `(cv_max, full_open)` [SI, Pa], as the plant built it.
fn disc(engine: &Engine, product: &str) -> (f64, f64) {
    let id = engine
        .graph
        .find_node(&format!("{product}_check"))
        .expect("a check valve");
    match &engine.graph.node(id).kind {
        NodeKind::CheckValve {
            cv_max, full_open, ..
        } => (*cv_max, full_open.value()),
        other => panic!("{other:?}"),
    }
}

/// The check valve's inlet, by hand from the column: the suction's friction
/// and fall to the pump, its curve, the discharge's friction (as
/// `a_product_line_stands_where_its_march_puts_it` checks), at the draw's last
/// flow `mass_flow` [kg/s] and density `rho` [kg/m³].
fn check_inlet(engine: &Engine, product: &str, rho: f64, mass_flow: f64) -> f64 {
    let pump = 1.8e5 - pipe_drop(engine, &format!("{product}_suction"), rho, mass_flow);
    let q = mass_flow / rho;
    let rise = rho * G * (PUMP_HEAD_M - PUMP_A * q * q.abs());
    pump + rise - pipe_drop(engine, &format!("{product}_discharge"), rho, mass_flow)
}

/// **A check valve on a product line stands where its march puts it, at full
/// lift**, by hand: from its inlet (the pump's curve less the line's friction)
/// the disc and its metre of pipe drop `(ρ_rel/Cv² + k)·Q²` (ISA-75.01 valve
/// law, Darcy–Weisbach), the drive past its 0.015 bar band so the disc is
/// fully up. And the valve moves no flow: every draw is what the same plant
/// without check valves draws, to the bit — the column's split does not read
/// its lines' pressures while they deliver.
#[test]
fn a_check_valve_on_a_product_line_stands_where_its_march_puts_it() {
    let plain = crude_unit_lines(false);
    let mut engine = build(&with_check_valves(&plain, &ALL, 400.0, 0.015)).unwrap();
    let mut without = build(&plain).unwrap();
    let slate = engine.slate.clone();
    const TOLERANCE: f64 = 1e-9;
    run(&mut engine, 150);
    run(&mut without, 150);
    let before = engine.snapshot();
    run(&mut engine, 1);
    run(&mut without, 1);
    let now = engine.snapshot();
    let other = without.snapshot();
    for (product, _, _) in LINES {
        let suction = format!("{product}_suction");
        // The march reads the draw as the previous tick left it, flow and
        // composition both: on this plant the gas oil's is still moving at
        // tick 150, by 1e-8 of its density a tick.
        let mass_flow = edge(&before, &suction).stream.mass_flow.value();
        let rho = density(&slate, &engine, edge(&before, &suction), 1.8e5);
        let inlet = check_inlet(&engine, product, rho, mass_flow);
        let published = node(&now, &format!("{product}_check")).pressure_pa;
        assert!(
            (published - inlet).abs() < TOLERANCE * inlet,
            "{product}: the check valve stands at {published} Pa, by hand {inlet} Pa"
        );
        let (cv, full_open) = disc(&engine, product);
        let q = mass_flow / rho;
        let disc_drop = rho / RHO_WATER_REF / (cv * cv) * q * q;
        let outlet_pipe = pipe_drop(&engine, &format!("{product}_check_outlet"), rho, mass_flow);
        assert!(
            disc_drop + outlet_pipe > full_open,
            "{product}: the disc must be at full lift: drive {} Pa",
            disc_drop + outlet_pipe
        );
        let cooler = inlet - disc_drop - outlet_pipe;
        let published = node(&now, &format!("{product}_cooler")).pressure_pa;
        assert!(
            (published - cooler).abs() < TOLERANCE * cooler,
            "{product}: the cooler stands at {published} Pa, by hand {cooler} Pa"
        );
        println!(
            "M86_3 {product}: {mass_flow:.3} kg/s, disc drop {disc_drop:.1} Pa, cooler {cooler:.0} Pa"
        );
        assert_eq!(
            edge(&now, &suction).stream.mass_flow.value(),
            edge(&other, &suction).stream.mass_flow.value(),
            "{product}: the check valve moved the draw"
        );
        for pipe in ["discharge", "check_outlet", "rundown"] {
            assert_eq!(
                edge(&now, &format!("{product}_{pipe}"))
                    .stream
                    .mass_flow
                    .value(),
                edge(&now, &suction).stream.mass_flow.value(),
                "{product}: the {pipe} carries the draw"
            );
        }
    }
}

/// **A disc inside its band reads its lift off its own drive**: a wide valve
/// (Kv 2000) with a 0.3 bar band, so at the line's flow it sits part-way up.
/// The drive across it, `S` (its drop less the outlet pipe's static head,
/// none here), must be the root the march solved: `S = (ρ_rel/(Cv·φ(S))² +
/// k)·Q²`, `φ` the disc's smoothstep lift over the band (M30,
/// docs/DESIGN.md §33).
#[test]
fn a_disc_inside_its_band_reads_its_lift_off_its_own_drive() {
    let src = with_check_valves(&crude_unit_lines(false), &ALL, 2000.0, 0.3);
    let mut engine = build(&src).unwrap();
    let slate = engine.slate.clone();
    run(&mut engine, 150);
    let before = engine.snapshot();
    run(&mut engine, 1);
    let now = engine.snapshot();
    for (product, _, _) in LINES {
        let suction = format!("{product}_suction");
        let mass_flow = edge(&before, &suction).stream.mass_flow.value();
        let rho = density(&slate, &engine, edge(&before, &suction), 1.8e5);
        let (cv, full_open) = disc(&engine, product);
        let drive = node(&now, &format!("{product}_check")).pressure_pa
            - node(&now, &format!("{product}_cooler")).pressure_pa;
        let lift = check_opening(drive, full_open);
        assert!(
            lift > 0.05 && lift < 0.95,
            "{product}: the disc should sit inside its band: lift {lift}, drive {drive} Pa"
        );
        let q = mass_flow / rho;
        let k = pipe_drop(&engine, &format!("{product}_check_outlet"), rho, mass_flow) / (q * q);
        let cv_eff = cv * lift;
        let needed = (rho / RHO_WATER_REF / (cv_eff * cv_eff) + k) * q * q;
        assert!(
            (drive - needed).abs() < 1e-9 * drive,
            "{product}: drive {drive} Pa, the lift it reads ({lift}) needs {needed} Pa"
        );
        println!("M86_3 band {product}: lift {lift:.4}, drive {drive:.1} Pa");
    }
}

/// **A stopped pump behind a check valve stops its product**: the
/// kerosene's tank raised so only its pump lifts the product there
/// (`short_draw_line::needs_pump`), half of the 40 m climbed in the pipe right
/// after the check valve — so a disc that forgot its outlet's static head at
/// no flow would let the line reach its tank. Stopped, the line delivers
/// nothing — the disc shut, no pipe of it running backwards — and the diesel
/// takes the kerosene's share, every component's books closing at the
/// column; started, the kerosene takes its share again.
#[test]
fn a_stopped_pump_behind_a_check_valve_stops_its_product() {
    let src = with_check_valves(
        &needs_pump(&crude_unit_lines(false), "kerosene"),
        &ALL,
        400.0,
        0.015,
    );
    let rundown =
        "to = \"kerosene_tank\"\nlength_m = 100.0\ndiameter_m = 0.15\nelevation_change_m = 40";
    let outlet = "name = \"kerosene_check_outlet\"\nfrom = \"kerosene_check\"\nto = \"kerosene_cooler\"\nlength_m = 1.0\ndiameter_m = 0.15";
    assert!(src.contains(rundown) && src.contains(outlet));
    let src = src
        .replacen(rundown, &rundown.replace("= 40", "= 20"), 1)
        .replacen(outlet, &format!("{outlet}\nelevation_change_m = 20"), 1);
    let mut engine = build(&src).unwrap();
    let mut last = tick(&mut engine, 30);
    assert!((shares(&engine, &last)[1] - 0.16).abs() < 1e-9);
    set_pump(&mut engine, "kerosene", false);
    for t in 0..30 {
        let s = tick(&mut engine, 1);
        let now = shares(&engine, &s);
        assert!(
            now[1] <= STOPPED_DRAW_SHARE * (1.0 + 1e-6),
            "tick {t}: the kerosene still runs: {now:?}"
        );
        assert!(
            (now[2] - (0.36 - now[1])).abs() < 1e-9,
            "tick {t}: the diesel did not take the kerosene: {now:?}"
        );
        assert!(worst_imbalance(&engine, &s) < 1e-9, "tick {t}");
        for pipe in ["suction", "discharge", "check_outlet", "rundown"] {
            let carried = flow(&s, &format!("kerosene_{pipe}"));
            assert!(carried >= 0.0, "tick {t}: the {pipe} runs back: {carried}");
        }
        // The line pinned at no flow: the shut disc holds the 20 m of
        // kerosene standing above it in its outlet pipe, `ρ·g·Δz`, and
        // nothing else — the kerosene the line held after the previous tick,
        // which the march reads: the stop changes what the column draws
        // there (804.8 → 793.1 kg/m³), and the line carries it a tick late.
        let rho = edge(&last, "kerosene_check_outlet")
            .stream
            .composition
            .mixture_density(&engine.slate)
            .value();
        let held = node(&s, "kerosene_check").pressure_pa - node(&s, "kerosene_cooler").pressure_pa;
        let head = rho * G * 20.0;
        assert!(
            (held - head).abs() < 1e-6 * head,
            "tick {t}: the shut disc holds {held} Pa, its outlet's head is {head} Pa"
        );
        last = s;
    }
    set_pump(&mut engine, "kerosene", true);
    let s = tick(&mut engine, 3);
    assert!((shares(&engine, &s)[1] - 0.16).abs() < 1e-9, "not released");
}
