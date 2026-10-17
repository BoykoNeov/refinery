//! M62.4b — product lines and water coolers (docs/DESIGN.md §67.13): a column's
//! draw runs through a pump and a water cooler to its tank, its pipes carrying
//! the draw, its nodes pinned where the line stands — marched from the column
//! at the draw's last flow — and its cooler's duty a result of the cooling
//! water.
//!
//! The plants: the crude unit under the line flash (`heat_on_boiling_reference`'s
//! fixture) with a product line on every draw — the draw falls 15 m to its pump,
//! which pushes it through its water cooler and up into its tank — and
//! `scenarios/product_cooler.toml`, a boiling naphtha condensed by a water
//! cooler, held to the zone script
//! `tools/steptime/prototypes/m62_4/zone_cooler.py`.

use refinery_core::components::Slate;
use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::traits::LineFlashModel;
use refinery_core::units::{KgPerSec, Pascal, G};
use refinery_core::Engine;
use refinery_solvers::elements::pipe_resistance;
use refinery_solvers::line_flash::EquilibriumLineFlash;
use refinery_solvers::TroutonThermo;

const CRUDE_UNIT: &str = include_str!("../../../scenarios/crude_unit.toml");
const PRODUCT_COOLER: &str = include_str!("../../../scenarios/product_cooler.toml");

/// Each product's cooling water [kg/s] and its cooler's UA [kW/K], sized to
/// return the water near 45 °C.
const LINES: [(&str, f64, f64); 5] = [
    ("naphtha", 75.0, 150.0),
    ("kerosene", 115.0, 90.0),
    ("diesel", 190.0, 120.0),
    ("gas_oil", 150.0, 70.0),
    ("residue", 460.0, 100.0),
];
const PUMP_HEAD_M: f64 = 40.0;
const PUMP_A: f64 = 2000.0;
const SUCTION_DROP_M: f64 = -15.0;
const CP_WATER: f64 = 4186.0;

/// The crude unit with a product line on every draw, on a plant that boils
/// (`flashing`) or not.
fn crude_unit_lines(flashing: bool) -> String {
    let mut src = CRUDE_UNIT.to_string();
    if flashing {
        src = src
            .replace(
                "boiloff = \"none\"",
                "boiloff = \"flash\"\nline_flash = \"equilibrium\"",
            )
            .replace("tube_failure_c = 550.0", "tube_failure_c = 1951.1");
    }
    src = src.replace(
        "name = \"transfer_line\"\nfrom = \"furnace\"\nto = \"column\"\nlength_m = 15.0\ndiameter_m = 0.08",
        "name = \"transfer_line\"\nfrom = \"furnace\"\nto = \"column\"\nlength_m = 15.0\ndiameter_m = 0.40",
    );
    for (product, water, ua) in LINES {
        let tank = format!("{product}_tank");
        let before = src.clone();
        src = src
            .replace(
                &format!("{{ outlet = \"{tank}\","),
                &format!("{{ outlet = \"{product}_pump\","),
            )
            .replace(
                &format!(
                    "name = \"{product}_draw\"\nfrom = \"column\"\nto = \"{tank}\"\nlength_m = 20.0\ndiameter_m = 0.15"
                ),
                &format!(
                    "name = \"{product}_suction\"\nfrom = \"column\"\nto = \"{product}_pump\"\n\
                     length_m = 20.0\ndiameter_m = 0.20\nelevation_change_m = {SUCTION_DROP_M}"
                ),
            );
        assert_ne!(src, before, "the {product} draw was not rewired");
        src.push_str(&format!(
            r#"
[nodes.{product}_pump]
type = "pump"
h0_m = {PUMP_HEAD_M}
a = {PUMP_A}
on = true

[nodes.{product}_cooler]
type = "water_cooler"
ua_kw_per_k = {ua}
water_inlet_c = 30.0
water_flow_kg_per_s = {water}

[[pipes]]
name = "{product}_discharge"
from = "{product}_pump"
to = "{product}_cooler"
length_m = 30.0
diameter_m = 0.15

[[pipes]]
name = "{product}_rundown"
from = "{product}_cooler"
to = "{tank}"
length_m = 100.0
diameter_m = 0.15
elevation_change_m = {}
"#,
            -SUCTION_DROP_M
        ));
    }
    assert_eq!(src.matches("_suction\"").count(), 5);
    src
}

fn build(src: &str) -> Result<Engine, String> {
    let file = refinery_scenarios::load_str(src).map_err(|e| e.to_string())?;
    refinery_scenarios::build_engine(&file).map_err(|e| e.to_string())
}

fn run(engine: &mut Engine, ticks: u32) {
    for t in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
        assert!(
            engine.snapshot().solver.converged,
            "tick {t} did not converge"
        );
    }
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn edge<'a>(s: &'a Snapshot, name: &str) -> &'a EdgeSnapshot {
    s.edges.iter().find(|e| e.name == name).unwrap()
}

/// The density a stream flows at: the line flash's mixture where it boils
/// there, else its liquid's.
fn density(slate: &Slate, engine: &Engine, stream: &EdgeSnapshot, pressure: f64) -> f64 {
    let composition = &stream.stream.composition;
    let temperature = stream.stream.temperature;
    match EquilibriumLineFlash
        .density(
            slate,
            composition,
            temperature,
            Pascal(pressure),
            &TroutonThermo::new(),
            engine.enthalpy(),
        )
        .unwrap()
    {
        Some(two_phase) => two_phase.mixture.value(),
        None => composition
            .density_at(slate, Pascal(pressure), temperature)
            .unwrap()
            .value(),
    }
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

/// **A line stands where its march puts it**: from the column's 1.8 bar, the
/// suction's friction and its 15 m fall give the pump's pressure; the pump's
/// curve, `ρ·g·(h₀ − a·Q|Q|)`, less the discharge's friction, the cooler's —
/// each at the draw's LAST flow (the previous tick's) and the draw's density
/// there, by hand. And every pipe of the line carries the draw.
#[test]
fn a_product_line_stands_where_its_march_puts_it() {
    let mut engine = build(&crude_unit_lines(true)).unwrap();
    let slate = engine.slate.clone();
    const TOLERANCE: f64 = 1e-9;
    run(&mut engine, 150);
    let before = engine.snapshot();
    run(&mut engine, 1);
    let now = engine.snapshot();
    for (product, _, _) in LINES {
        // The draw's LAST flow — the previous tick's, the stored one — at the
        // draw's state as the solve read it: this tick's own sweep on a tick
        // the boiling plant re-solves (M55.0), so to its density tolerance.
        let flow = edge(&before, &format!("{product}_suction"))
            .stream
            .mass_flow
            .value();
        let drawn = edge(&now, &format!("{product}_suction"));
        let column = 1.8e5;
        let rho = density(&slate, &engine, drawn, column);
        let pump = column - pipe_drop(&engine, &format!("{product}_suction"), rho, flow);
        let published = node(&now, &format!("{product}_pump")).pressure_pa;
        assert!(
            (published - pump).abs() < TOLERANCE * pump,
            "{product}: the pump stands at {published} Pa, the march at {pump} Pa"
        );
        // The draw is liquid at the pump: its suction head keeps it so.
        let rho = density(&slate, &engine, drawn, pump);
        let q = flow / rho;
        let rise = rho * G * (PUMP_HEAD_M - PUMP_A * q * q.abs());
        let cooler = pump + rise - pipe_drop(&engine, &format!("{product}_discharge"), rho, flow);
        let published = node(&now, &format!("{product}_cooler")).pressure_pa;
        assert!(
            (published - cooler).abs() < TOLERANCE * cooler,
            "{product}: the cooler stands at {published} Pa, the march at {cooler} Pa"
        );
        let carried = edge(&now, &format!("{product}_suction"))
            .stream
            .mass_flow
            .value();
        for pipe in ["discharge", "rundown"] {
            assert_eq!(
                edge(&now, &format!("{product}_{pipe}"))
                    .stream
                    .mass_flow
                    .value(),
                carried,
                "{product}: the {pipe} carries the draw"
            );
        }
        assert!(
            carried > 1.0,
            "{product}: the line carries a real draw ({carried} kg/s)"
        );
    }
}

/// **A liquid product's water cooler is the counter-flow effectiveness**, by
/// hand: `Q = ε·C_min·(T_h,in − T_c,in)`, `ε = (1 − e^{−NTU(1−C_r)})/(1 −
/// C_r·e^{−NTU(1−C_r)})`, `NTU = UA/C_min` (Incropera & DeWitt, Table 11.3) —
/// its product leaving at `T_in − Q/C_h` (constant heat capacities), liquid,
/// and its water at `T_c,in + Q/C_c`. Measured: every product reaches its tank
/// cooled and nothing vents.
#[test]
fn a_liquid_product_cooler_is_the_counter_flow_effectiveness() {
    let mut engine = build(&crude_unit_lines(true)).unwrap();
    let slate = engine.slate.clone();
    run(&mut engine, 300);
    let s = engine.snapshot();
    for (product, water, ua) in LINES {
        let inflow = edge(&s, &format!("{product}_discharge"));
        let mass_flow = inflow.stream.mass_flow.value();
        let cp = inflow.stream.composition.mixture_cp(&slate).value();
        let hot = mass_flow * cp;
        let cold = water * CP_WATER;
        let (c_min, c_max) = (hot.min(cold), hot.max(cold));
        let c_r = c_min / c_max;
        let ntu = ua * 1e3 / c_min;
        let decay = (-ntu * (1.0 - c_r)).exp();
        let effectiveness = (1.0 - decay) / (1.0 - c_r * decay);
        let t_in = inflow.stream.temperature.value();
        let duty = effectiveness * c_min * (t_in - 303.15);
        let cooler = node(&s, &format!("{product}_cooler"));
        let published = cooler
            .water_cooler
            .expect("a water cooler reports its tick");
        assert!(
            (published.duty_w - duty).abs() < 1e-9 * duty,
            "{product}: duty {} W against {duty} W by hand",
            published.duty_w
        );
        assert!(
            (published.water_outlet_k - (303.15 + duty / cold)).abs() < 1e-9 * 303.15,
            "{product}: the water leaves at {} K",
            published.water_outlet_k
        );
        let out = t_in - duty / hot;
        assert!(
            (cooler.temperature_k - out).abs() < 1e-9 * out,
            "{product}: the product leaves at {} K against {out} K",
            cooler.temperature_k
        );
        assert!(
            cooler.vapour_fraction.is_none(),
            "{product}: a cooled product is liquid"
        );
        let vent = edge(&s, &format!("{product}_tank__boiloff_vent"));
        assert_eq!(
            vent.stream.mass_flow.value(),
            0.0,
            "{product}: its tank vents"
        );
        // Measured: every water returns between 44.6 and 45.4 °C.
        assert!(
            (published.water_outlet_k - 273.15 - 45.0).abs() < 1.0,
            "{product}: the water returns at {} °C",
            published.water_outlet_k - 273.15
        );
    }
}

/// **A condensing water cooler against the zone script.** The product_cooler
/// plant's naphtha arrives boiling (24 % vapour by mass at tick 2000) and leaves
/// liquid; its duty is Kern's zone analysis over `WATER_COOLER_ZONES` = 10
/// equal-duty zones. `tools/steptime/prototypes/m62_4/zone_cooler.py`, on its
/// own flash from the engine's state at tick 2000 below, gives 1 671 629.1022 W
/// at 10 zones — 0.044 % under its 160-zone limit, 1 672 359.45 W (2 zones
/// −0.53 %, 5 −0.12 %, 20 −0.009 %).
#[test]
fn a_condensing_cooler_is_its_zone_analysis() {
    let mut engine = build(PRODUCT_COOLER).unwrap();
    run(&mut engine, 2000);
    let s = engine.snapshot();
    let inflow = edge(&s, "flash_line");
    let cooler = node(&s, "product_cooler");
    // The script's inputs, the engine's state.
    for (what, engine_value, script) in [
        (
            "mass flow",
            inflow.stream.mass_flow.value(),
            8.40623823853848,
        ),
        (
            "arriving temperature",
            inflow.stream.temperature.value(),
            379.7641134946459,
        ),
        (
            "arriving latent heat",
            inflow.stream.latent.unwrap().value(),
            72954.68466717555,
        ),
        ("pressure", cooler.pressure_pa, 141593.34670708922),
    ] {
        assert!(
            (engine_value - script).abs() < 1e-9 * script.abs(),
            "{what}: the engine's {engine_value} is not the script's {script}"
        );
    }
    let duty = cooler.water_cooler.unwrap().duty_w;
    assert!(
        (duty - 1_671_629.102_2).abs() < 1e-3,
        "the zone duty {duty} W against the script's 1 671 629.1022 W"
    );
    assert!(
        cooler.vapour_fraction.is_none(),
        "the cooler condenses all of it"
    );
    assert!(
        inflow.stream.vapour_fraction.unwrap() > 0.2,
        "it arrives boiling"
    );
    let vent = edge(&s, "product_tank__boiloff_vent");
    assert_eq!(vent.stream.mass_flow.value(), 0.0, "the tank vents nothing");
}

/// **Each line's first law, every tick**: what the draw carries out of the
/// column, less the heat its water took, is what reaches its tank — the line's
/// nodes hold nothing and its pipes book no friction (§67.13). And a line plant
/// runs on both fidelities, and with no line flash at all, to the same draws.
#[test]
fn every_product_line_closes_its_books_on_both_fidelities() {
    let mut answers = Vec::new();
    for (solver, flashing) in [("newton", true), ("simple", true), ("newton", false)] {
        let src = crude_unit_lines(flashing)
            .replace("flow = \"newton\"", &format!("flow = \"{solver}\""));
        let mut engine = build(&src).unwrap();
        let mut worst = 0.0f64;
        for t in 1..=200 {
            run(&mut engine, 1);
            let s = engine.snapshot();
            for (product, _, _) in LINES {
                let flux = |pipe: &str| {
                    engine
                        .enthalpy()
                        .stream_enthalpy_flux(
                            &engine.slate,
                            &edge(&s, &format!("{product}_{pipe}")).stream,
                        )
                        .unwrap()
                        .value()
                };
                let duty = node(&s, &format!("{product}_cooler"))
                    .water_cooler
                    .map_or(0.0, |w| w.duty_w);
                let (out, arrived) = (flux("suction"), flux("rundown"));
                worst = worst.max((out - duty - arrived).abs() / out.abs().max(1.0));
                assert_eq!(
                    edge(&s, &format!("{product}_suction")).dissipation_w,
                    0.0,
                    "tick {t}: a line pipe books no friction"
                );
            }
        }
        assert!(
            worst < 1e-12,
            "{solver}: a line's books miss by {worst:e} relative"
        );
        let s = engine.snapshot();
        answers.push(LINES.map(|(product, _, _)| {
            edge(&s, &format!("{product}_rundown"))
                .stream
                .mass_flow
                .value()
        }));
    }
    for (a, b) in answers[0].iter().zip(&answers[1]) {
        assert!(
            (a - b).abs() < 1e-6 * a,
            "the fidelities disagree: {a} against {b}"
        );
    }
    // Without the line flash the feed does not boil and the column splits it
    // otherwise; the lines still run, every draw a real flow.
    assert!(answers[2].iter().all(|&m| m > 1.0), "{:?}", answers[2]);
}

/// **A line declared before its column runs as one declared after it.** The
/// sweep visits the lowest node id that is ready, so a line node numbered
/// below its column would be visited first — with the solve's zero on its
/// pipe in, as no inflow — unless it waits on its column by topology. The
/// naphtha line's pump and cooler moved ahead of the furnace in the file: the
/// same draws, the same duties.
#[test]
fn a_line_declared_before_its_column_waits_for_it() {
    let after = crude_unit_lines(true);
    let start = after.find("\n[nodes.naphtha_pump]").unwrap();
    let end = after[start..].find("\n[[pipes]]").unwrap() + start;
    let block = after[start..end].to_string();
    let before = after[..start].to_string() + &after[end..];
    let before = before.replacen(
        "[nodes.furnace]",
        &format!("{}\n\n[nodes.furnace]", block.trim()),
        1,
    );
    assert!(before.find("[nodes.naphtha_pump]") < before.find("[nodes.column]"));
    let mut results = Vec::new();
    for src in [&after, &before] {
        let mut engine = build(src).unwrap();
        run(&mut engine, 100);
        // Visited before its column, a line node would have mixed no inflow and
        // been HELD — a placeholder no loop may read — before the column's
        // release swept it again: every line node carrying its draw is resolved.
        for (product, _, _) in LINES {
            for unit in ["pump", "cooler"] {
                let id = engine
                    .graph
                    .find_node(&format!("{product}_{unit}"))
                    .unwrap();
                assert!(
                    !engine.node_states().held.contains(&id),
                    "{product}_{unit} is held while its line carries the draw"
                );
            }
        }
        let s = engine.snapshot();
        let cooler = node(&s, "naphtha_cooler");
        results.push((
            edge(&s, "naphtha_rundown").stream.mass_flow.value(),
            cooler.temperature_k,
            cooler.water_cooler.expect("the cooler cooled").duty_w,
        ));
    }
    let (a, b) = (results[0], results[1]);
    assert!(
        (a.0 - b.0).abs() < 1e-9 * a.0
            && (a.1 - b.1).abs() < 1e-9 * a.1
            && (a.2 - b.2).abs() < 1e-9 * a.2,
        "declared after: {a:?}; declared before: {b:?}"
    );
}

/// **A line that cannot deliver its draw halts the run, by name** — its pump
/// stopped — and **a water cooler that would boil its water halts it**; the
/// cooler's own command and the duty command it refuses.
#[test]
fn a_line_that_cannot_deliver_or_a_cooler_that_would_boil_halts() {
    let id = |engine: &Engine, name: &str| engine.graph.find_node(name).unwrap();
    // The residue's tank 40 m above its cooler: only its pump lifts it there.
    let high = crude_unit_lines(true).replace(
        "to = \"residue_tank\"
length_m = 100.0
diameter_m = 0.15
elevation_change_m = 15",
        "to = \"residue_tank\"
length_m = 100.0
diameter_m = 0.15
elevation_change_m = 40",
    );
    assert!(high.contains("elevation_change_m = 40"));
    let mut engine = build(&high).unwrap();
    run(&mut engine, 20);
    let pump = id(&engine, "residue_pump");
    engine
        .apply(Command::SetPumpOn {
            node: pump,
            on: false,
        })
        .unwrap();
    let err = (0..3)
        .find_map(|_| engine.tick().err())
        .expect("a stopped line pump cannot deliver its draw");
    assert!(
        err.to_string().contains("cannot deliver") && err.to_string().contains("residue_pump"),
        "{err}"
    );

    let mut engine = build(&crude_unit_lines(true)).unwrap();
    run(&mut engine, 20);
    let cooler = id(&engine, "residue_cooler");
    engine
        .apply(Command::SetCoolingWaterFlow {
            node: cooler,
            flow: KgPerSec(40.0),
        })
        .unwrap();
    let err = engine
        .tick()
        .expect_err("40 kg/s of water cannot take the residue's heat");
    assert!(
        err.to_string().contains("would boil its cooling water")
            && err.to_string().contains("residue_cooler"),
        "{err}"
    );

    let mut engine = build(&crude_unit_lines(true)).unwrap();
    let cooler = id(&engine, "residue_cooler");
    let pump = id(&engine, "residue_pump");
    for (command, expected) in [
        (
            Command::SetCoolerDuty {
                node: cooler,
                duty: refinery_core::units::Watt(1e6),
            },
            "SetCoolingWaterFlow",
        ),
        (
            Command::SetCoolingWaterFlow {
                node: cooler,
                flow: KgPerSec(-1.0),
            },
            ">= 0",
        ),
        (
            Command::SetCoolingWaterFlow {
                node: pump,
                flow: KgPerSec(10.0),
            },
            "not a water cooler",
        ),
    ] {
        let err = engine.apply(command).expect_err("refused");
        assert!(err.to_string().contains(expected), "{err}");
    }
    engine
        .apply(Command::SetCoolingWaterFlow {
            node: cooler,
            flow: KgPerSec(500.0),
        })
        .unwrap();
    match &engine.graph.node(cooler).kind {
        NodeKind::WaterCooler { water_flow, .. } => assert_eq!(water_flow.value(), 500.0),
        other => panic!("{other:?}"),
    }
}

/// **What a draw line refuses, by name** (§67.13): a node on it that is not a
/// pump or a cooler, a leak path on one of its pipes, a line pump declaring
/// `npsh_required_m`; and a water cooler's keys out of range.
#[test]
fn a_draw_line_refuses_what_it_does_not_model() {
    // On the plant without the line flash, which refuses leak paths and the
    // suction key by itself.
    let base = crude_unit_lines(false);
    let junction = base.replace(
        "[nodes.naphtha_cooler]\ntype = \"water_cooler\"\nua_kw_per_k = 150\nwater_inlet_c = 30.0\nwater_flow_kg_per_s = 75\n",
        "[nodes.naphtha_cooler]\ntype = \"junction\"\n",
    );
    let leak = base.replace(
        "name = \"kerosene_discharge\"\nfrom = \"kerosene_pump\"\nto = \"kerosene_cooler\"\nlength_m = 30.0\ndiameter_m = 0.15",
        "name = \"kerosene_discharge\"\nfrom = \"kerosene_pump\"\nto = \"kerosene_cooler\"\nlength_m = 30.0\ndiameter_m = 0.15\nleak_to = \"outside\"\n\n[nodes.outside]\ntype = \"atmosphere\"",
    );
    let npsh = base.replace(
        "[nodes.diesel_pump]\ntype = \"pump\"\nh0_m = 40\na = 2000\non = true",
        "[nodes.diesel_pump]\ntype = \"pump\"\nh0_m = 40\na = 2000\non = true\nnpsh_required_m = 3.0",
    );
    let hot_water = base.replace("water_inlet_c = 30.0", "water_inlet_c = 100.0");
    for (src, expected) in [
        (junction, "holds only pumps and coolers"),
        (leak, "leak path"),
        (npsh, "npsh_required_m"),
        (hot_water, "water_inlet_c"),
    ] {
        assert!(
            src != base,
            "the fixture for '{expected}' did not change the plant"
        );
        let err = match build(&src) {
            Ok(_) => panic!("must refuse ({expected})"),
            Err(e) => e,
        };
        assert!(
            err.contains(expected),
            "expected a refusal naming {expected}: {err}"
        );
    }
}
