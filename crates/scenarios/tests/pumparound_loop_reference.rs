//! M72.2 — a pumparound loop on the map (docs/DESIGN.md §77.2): a column's
//! loop draws its stage's liquid down to a pump, through a cooler the column
//! drives, and back into the stage above — its pipes carrying the loop's flow,
//! its nodes pinned where the line stands, marched from the column at the
//! loop's last flow, its cooler returning it at exactly the temperature the
//! column's stage balances took.
//!
//! The plant: `data/crude_unit_loops_fixture.toml`, the crude unit with its
//! three pumparounds as loops (§77.0's layout and flows).

use refinery_core::components::Slate;
use refinery_core::energy::T_REF;
use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::units::{Kelvin, KgPerSec, Pascal, Watt, G};
use refinery_core::Engine;
use refinery_solvers::elements::pipe_resistance;

const PLANT: &str = include_str!("data/crude_unit_loops_fixture.toml");
/// Each loop's name, flow [kg/s] and return [°C], as the fixture declares them.
const LOOPS: [(&str, f64, f64); 3] = [
    ("kero", 85.0, 150.0),
    ("diesel", 95.0, 215.0),
    ("gas_oil", 65.0, 270.0),
];
/// The products' draw pipes, the feed's pipe, the column's pressure [Pa].
const PRODUCTS: [&str; 5] = [
    "naphtha_draw",
    "kerosene_draw",
    "diesel_draw",
    "gas_oil_draw",
    "residue_draw",
];
const COLUMN_PA: f64 = 1.8e5;
const PUMP_HEAD_M: f64 = 60.0;
const PUMP_A: f64 = 1000.0;

fn build(src: &str) -> Result<Engine, String> {
    let file = refinery_scenarios::load_str(src).map_err(|e| e.to_string())?;
    refinery_scenarios::build_engine(&file).map_err(|e| e.to_string())
}

fn plant(solver: &str) -> Engine {
    plant_of(PLANT, solver)
}

/// A plant with loops on the flow solver `solver`.
fn plant_of(src: &str, solver: &str) -> Engine {
    build(&src.replace("flow = \"newton\"", &format!("flow = \"{solver}\""))).unwrap()
}

fn tick(engine: &mut Engine, t: u32) {
    engine
        .tick()
        .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn edge<'a>(s: &'a Snapshot, name: &str) -> &'a EdgeSnapshot {
    s.edges.iter().find(|e| e.name == name).unwrap()
}

fn flow(s: &Snapshot, name: &str) -> f64 {
    edge(s, name).stream.mass_flow.value()
}

/// A stream's enthalpy flow [W] on the engine's datum at constant heat
/// capacity: `ṁ·Σ w·cp·(T − T_REF)`.
fn enthalpy_flow(slate: &Slate, stream: &EdgeSnapshot) -> f64 {
    let cp: f64 = stream
        .stream
        .composition
        .fractions()
        .iter()
        .enumerate()
        .map(|(c, w)| w * slate.get(c).cp.value())
        .sum();
    stream.stream.mass_flow.value() * cp * (stream.stream.temperature.value() - T_REF.value())
}

fn cooler_duty(s: &Snapshot, name: &str) -> f64 {
    match &node(s, name).kind {
        NodeKind::Cooler { duty } => duty.value(),
        other => panic!("{name} is {other:?}"),
    }
}

/// **A loop's line carries its circulation and returns at its setpoint**:
/// every pipe of each line carries the flow the column ran, to the bit; the
/// cooler stands at the loop's return temperature and its duty is the
/// column's; the duties are the probe's (§77.0, finding 6).
#[test]
fn a_loop_line_carries_its_circulation_and_returns_at_its_setpoint() {
    let mut engine = plant("newton");
    for t in 1..=300 {
        tick(&mut engine, t);
    }
    let s = engine.snapshot();
    let operation = node(&s, "column").column_operation.clone().unwrap();
    let probe = [14.21e6, 14.90e6, 10.93e6];
    for (k, (name, declared, back)) in LOOPS.iter().enumerate() {
        let run = operation.loops[k];
        assert!(
            (run.flow_kg_s - declared).abs() < 1e-9 * declared,
            "{run:?}"
        );
        for pipe in ["suction", "discharge", "return"] {
            assert_eq!(
                flow(&s, &format!("{name}_pa_{pipe}")),
                run.flow_kg_s,
                "{name}: the {pipe} carries the loop's flow"
            );
        }
        let cooler = node(&s, &format!("{name}_pa_cooler"));
        assert!(
            (cooler.temperature_k - (back + 273.15)).abs() < 1e-9,
            "{name}: the cooler returns at {} K",
            cooler.temperature_k
        );
        assert!(
            (edge(&s, &format!("{name}_pa_return"))
                .stream
                .temperature
                .value()
                - (back + 273.15))
                .abs()
                < 1e-9
        );
        assert_eq!(cooler_duty(&s, &format!("{name}_pa_cooler")), run.duty_w);
        assert!(
            (run.duty_w / probe[k] - 1.0).abs() < 1e-3,
            "{name}: {} W against the probe's {} W",
            run.duty_w,
            probe[k]
        );
    }
    // The trays inside each loop carry its circulation (§77.0, finding 1).
    let liquid = &operation.stage_liquid_mol_s;
    assert!(liquid[1] > 2.0 * liquid[0], "{liquid:?}");
}

/// **The books close every tick**, the column's and each cooler's: the
/// column passes its feed's mass out by its products, each loop's out and back
/// cancelling; its heat — feed, reboiler, each loop's liquid out at its draw
/// stage and back at its return, against the products and the condenser —
/// closes to rounding; each cooler takes out exactly its duty. `src` is a
/// plant whose loops and products are named as the fixture's (M72.4's shipped
/// plant is too).
pub(super) fn the_books_close_every_tick(src: &str, solver: &str, ticks: u32) {
    let mut engine = plant_of(src, solver);
    let slate = engine.slate.clone();
    for t in 1..=ticks {
        tick(&mut engine, t);
        let s = engine.snapshot();
        let feed = edge(&s, "transfer_line");
        let products: f64 = PRODUCTS.iter().map(|p| flow(&s, p)).sum();
        let mass = feed.stream.mass_flow.value();
        assert!(
            (products - mass).abs() < 1e-9 * mass,
            "{solver} tick {t}: {products} kg/s out for {mass} kg/s in"
        );
        let column = node(&s, "column");
        let duty = column.column_duty.unwrap();
        // The feed enters at the column's own mixed temperature.
        let feed_cp: f64 = feed
            .stream
            .composition
            .fractions()
            .iter()
            .enumerate()
            .map(|(c, w)| w * slate.get(c).cp.value())
            .sum();
        let mut energy = mass * feed_cp * (column.temperature_k - T_REF.value()) + duty.reboiler_w
            - duty.condenser_w;
        for p in PRODUCTS {
            energy -= enthalpy_flow(&slate, edge(&s, p));
        }
        for (name, _, _) in LOOPS {
            let out = edge(&s, &format!("{name}_pa_suction"));
            let back = edge(&s, &format!("{name}_pa_return"));
            assert_eq!(out.stream.mass_flow, back.stream.mass_flow);
            energy += enthalpy_flow(&slate, back) - enthalpy_flow(&slate, out);
            // The cooler: what arrives less its duty is what leaves.
            let arriving = enthalpy_flow(&slate, edge(&s, &format!("{name}_pa_discharge")));
            let removed = cooler_duty(&s, &format!("{name}_pa_cooler"));
            let leaving = enthalpy_flow(&slate, back);
            assert!(
                (arriving - removed - leaving).abs() < 1e-9 * arriving.abs(),
                "{solver} tick {t}: {name}'s cooler off by {} W",
                arriving - removed - leaving
            );
        }
        assert!(
            energy.abs() < 1e-9 * duty.reboiler_w,
            "{solver} tick {t}: the column's heat off by {energy} W"
        );
    }
}

#[test]
fn the_books_close_every_tick_newton() {
    the_books_close_every_tick(PLANT, "newton", 300);
}

#[test]
fn the_books_close_every_tick_simple() {
    the_books_close_every_tick(PLANT, "simple", 300);
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

/// **A loop's line stands where its march puts it**: from the column's
/// 1.8 bar, the suction's friction and its 15 m fall give the pump's pressure;
/// the pump's curve less the discharge's friction, the cooler's — each at the
/// loop's LAST flow and its drawn liquid's density, by hand — and the return
/// arrives back at the column at or above its pressure, the rest the line's
/// flow valve.
#[test]
fn a_loop_line_stands_where_its_march_puts_it() {
    let mut engine = plant("newton");
    let slate = engine.slate.clone();
    for t in 1..=150 {
        tick(&mut engine, t);
    }
    let before = engine.snapshot();
    tick(&mut engine, 151);
    let now = engine.snapshot();
    for (name, _, _) in LOOPS {
        // The loop's LAST flow and its drawn liquid as the march read them:
        // the previous tick's (this plant does not boil, so a tick solves
        // once, on the last tick's states).
        let last = flow(&before, &format!("{name}_pa_suction"));
        let drawn = &edge(&before, &format!("{name}_pa_suction")).stream;
        let rho = |pressure: f64| {
            drawn
                .composition
                .density_at(&slate, Pascal(pressure), drawn.temperature)
                .unwrap()
                .value()
        };
        let suction = format!("{name}_pa_suction");
        let pump = COLUMN_PA - pipe_drop(&engine, &suction, rho(COLUMN_PA), last);
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
        assert!(cooler > COLUMN_PA, "{name}: the pump lifts the loop");
    }
}

/// **The two handles**: a loop's flow and its return temperature, each by
/// command, each with its consequence on the next tick — and the refusals:
/// out of range, a loop the column does not have, a node that is no column
/// with loops, and the loop's cooler's duty set by hand.
#[test]
fn the_loop_commands_set_its_flow_and_its_return() {
    let mut engine = plant("newton");
    for t in 1..=100 {
        tick(&mut engine, t);
    }
    let id = |engine: &Engine, name: &str| engine.graph.find_node(name).unwrap();
    let column = id(&engine, "column");
    engine
        .apply(Command::SetLoopFlow {
            node: column,
            pumparound_loop: 1,
            flow: KgPerSec(110.0),
        })
        .unwrap();
    engine
        .apply(Command::SetLoopReturnTemperature {
            node: column,
            pumparound_loop: 0,
            temperature: Kelvin(160.0 + 273.15),
        })
        .unwrap();
    tick(&mut engine, 101);
    let s = engine.snapshot();
    let loops = node(&s, "column").column_operation.clone().unwrap().loops;
    assert!(
        (loops[1].flow_kg_s - 110.0).abs() < 1e-9 * 110.0,
        "{loops:?}"
    );
    assert_eq!(flow(&s, "diesel_pa_return"), loops[1].flow_kg_s);
    assert!((node(&s, "kero_pa_cooler").temperature_k - 433.15).abs() < 1e-9);
    assert!((loops[0].return_temperature_k - 433.15).abs() < 1e-9);

    let refused = |engine: &mut Engine, command: Command, phrase: &str| {
        let message = engine.apply(command).unwrap_err().to_string();
        assert!(message.contains(phrase), "{message}");
    };
    refused(
        &mut engine,
        Command::SetLoopFlow {
            node: column,
            pumparound_loop: 0,
            flow: KgPerSec(-1.0),
        },
        "finite and >= 0",
    );
    refused(
        &mut engine,
        Command::SetLoopReturnTemperature {
            node: column,
            pumparound_loop: 0,
            temperature: Kelvin(f64::NAN),
        },
        "above absolute zero",
    );
    refused(
        &mut engine,
        Command::SetLoopFlow {
            node: column,
            pumparound_loop: 3,
            flow: KgPerSec(10.0),
        },
        "has 3 pumparound loop(s); there is no loop 3",
    );
    let furnace = id(&engine, "furnace");
    refused(
        &mut engine,
        Command::SetLoopFlow {
            node: furnace,
            pumparound_loop: 0,
            flow: KgPerSec(10.0),
        },
        "is not a column with pumparound loops",
    );
    let cooler = id(&engine, "gas_oil_pa_cooler");
    refused(
        &mut engine,
        Command::SetCoolerDuty {
            node: cooler,
            duty: Watt(1.0e6),
        },
        "the cooler of pumparound loop 2 of column 'column'",
    );
}

/// What a loop's line may not be, refused at load by name.
#[test]
fn a_loop_line_that_is_not_one_is_refused() {
    let refusal = |src: String| build(&src).err().expect("refused");
    // Through a valve.
    let valve = PLANT
        .replace(
            "[nodes.kero_pa_cooler]",
            "[nodes.kero_pa_valve]\ntype = \"valve\"\nkv = 500.0\nopening = 1.0\n\n[nodes.kero_pa_cooler]",
        )
        .replace(
            "name = \"kero_pa_discharge\"\nfrom = \"kero_pa_pump\"\nto = \"kero_pa_cooler\"",
            "name = \"kero_pa_discharge\"\nfrom = \"kero_pa_pump\"\nto = \"kero_pa_valve\"\nlength_m = 1.0\ndiameter_m = 0.3\n\n[[pipes]]\nname = \"kero_pa_trim\"\nfrom = \"kero_pa_valve\"\nto = \"kero_pa_cooler\"",
        );
    let message = refusal(valve);
    assert!(
        message.contains("holds only pumps and one cooler"),
        "{message}"
    );
    // A cooling-water cooler.
    let water = PLANT.replace(
        "[nodes.kero_pa_cooler]\ntype = \"cooler\"\nduty_mw = 0.0",
        "[nodes.kero_pa_cooler]\ntype = \"water_cooler\"\nua_kw_per_k = 500.0\nwater_inlet_c = 30.0\nwater_flow_kg_per_s = 100.0",
    );
    let message = refusal(water);
    assert!(message.contains("a cooling-water cooler"), "{message}");
    // A second cooler: the column drives one.
    let two = PLANT
        .replace(
            "[nodes.kero_pa_cooler]",
            "[nodes.kero_pa_trim_cooler]\ntype = \"cooler\"\nduty_mw = 0.0\n\n[nodes.kero_pa_cooler]",
        )
        .replace(
            "name = \"kero_pa_discharge\"\nfrom = \"kero_pa_pump\"\nto = \"kero_pa_cooler\"",
            "name = \"kero_pa_discharge\"\nfrom = \"kero_pa_pump\"\nto = \"kero_pa_trim_cooler\"\nlength_m = 1.0\ndiameter_m = 0.3\n\n[[pipes]]\nname = \"kero_pa_trim\"\nfrom = \"kero_pa_trim_cooler\"\nto = \"kero_pa_cooler\"",
        );
    let message = refusal(two);
    assert!(
        message.contains("has a second cooler, 'kero_pa_cooler'"),
        "{message}"
    );
    // No cooler: the pump straight back.
    let bare = PLANT
        .replace(
            "[nodes.kero_pa_cooler]\ntype = \"cooler\"\nduty_mw = 0.0\n",
            "",
        )
        .replace(
            "name = \"kero_pa_discharge\"\nfrom = \"kero_pa_pump\"\nto = \"kero_pa_cooler\"\nlength_m = 30.0\ndiameter_m = 0.3\n",
            "",
        )
        .replace(
            "name = \"kero_pa_return\"\nfrom = \"kero_pa_cooler\"",
            "name = \"kero_pa_return\"\nfrom = \"kero_pa_pump\"",
        )
        .replace("[[pipes]]\n\n[[pipes]]", "[[pipes]]");
    let message = refusal(bare);
    assert!(message.contains("has no cooler"), "{message}");
    // A line pump with a suction model.
    let npsh = PLANT.replace(
        "[nodes.kero_pa_pump]\ntype = \"pump\"\nh0_m = 60.0",
        "[nodes.kero_pa_pump]\ntype = \"pump\"\nnpsh_required_m = 3.0\nh0_m = 60.0",
    );
    let message = refusal(npsh);
    assert!(message.contains("declares npsh_required_m"), "{message}");
    // A control loop on the loop's cooler.
    let driven = format!(
        "{PLANT}\n[[controls]]\nname = \"kero_return_tc\"\nmeasurement = {{ node = \"kero_pa_cooler\", variable = \"temperature\" }}\nactuator = \"kero_pa_cooler\"\nalgorithm = \"pi\"\nmode = \"auto\"\naction = \"direct\"\nsetpoint_c = 150.0\ngain_per_k = 0.015\nintegral_time_s = 10.0\ninitial_output = 0.25\nmax_duty_mw = 40.0\n"
    );
    let message = refusal(driven);
    assert!(
        message.contains("the pumparound loop cooler of column 'column'"),
        "{message}"
    );
    // A return to a stage other than the one above the draw.
    let skip = PLANT.replace(
        "{ draw_stage = 3, return_stage = 2,",
        "{ draw_stage = 3, return_stage = 1,",
    );
    let message = refusal(skip);
    assert!(
        message.contains("returns to the stage above its draw"),
        "{message}"
    );
}
