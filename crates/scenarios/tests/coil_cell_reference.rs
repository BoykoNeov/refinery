//! M62.4c — the furnace coil in cells, the pressure falling along it
//! (docs/DESIGN.md §67.14): a furnace's coil cut into cells, each a flashing
//! node at its own pressure with its share of the coil's conductance against
//! one metal, joined by segments of the coil's bore.
//!
//! The plants: `scenarios/furnace_heater.toml` (water, no line flash) with its
//! coil in cells of a bore wide enough to carry no friction, where the cells
//! must settle where the lumped coil does; and the crude unit under the line
//! flash, fed through a control valve into a coil in eight cells — 800 m of
//! 24 cm tube — which its crude enters liquid at 15 bar and leaves boiling at
//! 2.1 bar (M62.4c's probe, DESIGN §67.14).

use refinery_core::components::Slate;
use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::traits::LineFlashModel;
use refinery_core::units::{Kelvin, Pascal};
use refinery_core::Engine;
use refinery_solvers::line_flash::EquilibriumLineFlash;
use refinery_solvers::TroutonThermo;

const CRUDE_UNIT: &str = include_str!("../../../scenarios/crude_unit.toml");
const FURNACE_HEATER: &str = include_str!("../../../scenarios/furnace_heater.toml");
const CELLS: usize = 8;

/// The crude unit under the line flash, its feed through a control valve into
/// a coil in `cells` cells (`flow` the solver).
fn crude_coil(cells: usize, flow: &str) -> String {
    let src = CRUDE_UNIT
        .replace(
            "boiloff = \"none\"",
            "boiloff = \"flash\"\nline_flash = \"equilibrium\"",
        )
        .replace("flow = \"newton\"", &format!("flow = \"{flow}\""))
        .replace("tube_failure_c = 550.0", "tube_failure_c = 1951.1")
        .replace(
            "fluid_heating_value_mj_per_kg = 42.686",
            &format!(
                "fluid_heating_value_mj_per_kg = 42.686\ncoil_cells = {cells}\n\
                 coil_length_m = 800.0\ncoil_diameter_m = 0.24"
            ),
        )
        .replace(
            "name = \"transfer_line\"\nfrom = \"furnace\"\nto = \"column\"\nlength_m = 15.0\ndiameter_m = 0.08",
            "name = \"transfer_line\"\nfrom = \"furnace\"\nto = \"column\"\nlength_m = 15.0\ndiameter_m = 0.40",
        )
        .replace(
            "name = \"crude_line\"\nfrom = \"crude_source\"\nto = \"furnace\"\nlength_m = 30.0\ndiameter_m = 0.08",
            "name = \"crude_line\"\nfrom = \"crude_source\"\nto = \"feed_valve\"\nlength_m = 30.0\n\
             diameter_m = 0.30\n\n[[pipes]]\nname = \"feed_spool\"\nfrom = \"feed_valve\"\n\
             to = \"furnace\"\nlength_m = 5.0\ndiameter_m = 0.30",
        )
        .replace(
            "[nodes.furnace]",
            "[nodes.feed_valve]\ntype = \"valve\"\nkv = 135.0\nopening = 1.0\n\n[nodes.furnace]",
        );
    assert!(src.contains("coil_cells") && src.contains("feed_spool") && src.contains("0.40"));
    src
}

/// The water heater with its coil in `cells` cells of a bore that carries
/// next to no friction (10 cm of 50 cm tube against its lines' 40 m of 10 cm;
/// a wider, shorter coil leaves the flow solve a conductance it cannot close).
fn heater_cells(cells: usize) -> String {
    let src = FURNACE_HEATER.replace(
        "fluid_heating_value_mj_per_kg = 0.0",
        &format!(
            "fluid_heating_value_mj_per_kg = 0.0\ncoil_cells = {cells}\ncoil_length_m = 0.1\n\
             coil_diameter_m = 0.5"
        ),
    );
    assert!(src.contains("coil_cells"));
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

fn metal(s: &Snapshot, furnace: &str) -> f64 {
    match &node(s, furnace).kind {
        NodeKind::Furnace { coil, .. } => coil.temperature.value(),
        other => panic!("not a furnace: {other:?}"),
    }
}

/// The coil's nodes in the stream's order: its cells, then the furnace.
fn chain(furnace: &str, cells: usize) -> Vec<String> {
    (1..cells)
        .map(|k| format!("{furnace}__cell_{k}"))
        .chain([furnace.to_string()])
        .collect()
}

/// **On a liquid, a coil in cells settles where the lumped coil does.** With a
/// constant heat capacity each cell's effectiveness `1 − e^{−UA_i/W}` compounds
/// to the whole coil's, `Π e^{−UA_i/W} = e^{−UA/W}`, so the heat the cells take
/// from one metal, `W·(T̄_c − T_in)·(1 − e^{−UA/W})`, is the lumped coil's to
/// the last term — the same steady metal, the same outlet. The heater's coil
/// in eight cells of next to no friction, against itself
/// lumped, settled.
#[test]
fn a_coil_in_cells_settles_where_the_lumped_coil_does() {
    let mut lumped = build(FURNACE_HEATER).unwrap();
    let mut cells = build(&heater_cells(CELLS)).unwrap();
    run(&mut lumped, 3000);
    run(&mut cells, 3000);
    let (a, b) = (lumped.snapshot(), cells.snapshot());
    for (what, lumped, cells) in [
        (
            "outlet",
            node(&a, "heater").temperature_k,
            node(&b, "heater").temperature_k,
        ),
        ("metal", metal(&a, "heater"), metal(&b, "heater")),
        (
            "flow",
            edge(&a, "transfer_line").stream.mass_flow.value(),
            edge(&b, "transfer_line").stream.mass_flow.value(),
        ),
    ] {
        // The flow to its segments' friction (measured 2.9e-6 of it); the
        // outlet and the metal to the solves' tolerance.
        let tolerance = if what == "flow" { 1e-5 } else { 1e-6 };
        assert!(
            (lumped - cells).abs() < tolerance * lumped.abs(),
            "{what}: lumped {lumped}, in cells {cells}"
        );
    }
    // The cells really are a chain: the heat rises cell by cell.
    let temperatures: Vec<f64> = chain("heater", CELLS)
        .iter()
        .map(|n| node(&b, n).temperature_k)
        .collect();
    assert!(
        temperatures.windows(2).all(|w| w[1] > w[0]),
        "{temperatures:?}"
    );
}

/// **The coil's first law, every tick**: what the burners fire is what the
/// metal stores, what goes up the stack, and what the stream took between the
/// coil's inlet and its outlet — the segments' friction heat aside, which the
/// stream carries but the metal did not give (§67.14 rule 4).
#[test]
fn a_coil_in_cells_closes_its_first_law_every_tick() {
    let mut engine = build(&crude_coil(CELLS, "newton")).unwrap();
    let dt = engine.dt().value();
    let segments: Vec<String> = (1..CELLS).map(|k| format!("furnace__coil_{k}")).collect();
    let mut previous = engine.snapshot();
    let mut worst = 0.0f64;
    for t in 1..=150 {
        run(&mut engine, 1);
        let s = engine.snapshot();
        let flux = |name: &str| {
            engine
                .enthalpy()
                .stream_enthalpy_flux(&engine.slate, &edge(&s, name).stream)
                .unwrap()
                .value()
        };
        let (duty, capacity) = match &node(&s, "furnace").kind {
            NodeKind::Furnace { duty, coil, .. } => (duty.value(), coil.heat_capacity.value()),
            _ => unreachable!(),
        };
        let stored = capacity * (metal(&s, "furnace") - metal(&previous, "furnace")) / dt;
        let flue = node(&s, "furnace").flue_loss_w.unwrap();
        let friction: f64 = segments.iter().map(|n| edge(&s, n).dissipation_w).sum();
        // Each published stream stands at its pipe's OUTLET, its own friction
        // heat in it: the transfer line's is the stream's after the coil.
        let after = edge(&s, "transfer_line").dissipation_w;
        let taken = flux("transfer_line") - after - flux("feed_spool") - friction;
        worst = worst.max((duty - stored - flue - taken).abs() / duty);
        assert!(
            friction > 0.0,
            "tick {t}: the coil's segments carry friction"
        );
        previous = s;
    }
    // Measured: 1.3e-14 of the duty, the last cell the remainder.
    assert!(
        worst < 1e-12,
        "the coil's first law misses by {worst:e} of its duty"
    );
}

/// **The crude enters its coil liquid and boils partway along it**, as the
/// pressure falls: the first cell liquid at its inlet's 15 bar, the pressure
/// falling cell by cell to the furnace's 2.1 bar, the vapour rising from
/// the first cell that boils to the outlet's 41 %; and every cell's published
/// state is the line flash's at its own pressure — an independent flash of its
/// own enthalpy lands on it.
#[test]
fn the_crude_enters_its_coil_liquid_and_boils_partway() {
    let mut engine = build(&crude_coil(CELLS, "newton")).unwrap();
    run(&mut engine, 150);
    let s = engine.snapshot();
    let slate: Slate = engine.slate.clone();
    let nodes = chain("furnace", CELLS);
    let first = node(&s, &nodes[0]);
    assert!(
        first.vapour_fraction.is_none(),
        "the crude enters its coil liquid"
    );
    assert!(
        first.pressure_pa > 14e5,
        "its inlet stands at {} Pa",
        first.pressure_pa
    );
    let pressures: Vec<f64> = nodes.iter().map(|n| node(&s, n).pressure_pa).collect();
    assert!(pressures.windows(2).all(|w| w[1] < w[0]), "{pressures:?}");
    let shares: Vec<f64> = nodes
        .iter()
        .map(|n| node(&s, n).vapour_fraction.unwrap_or(0.0))
        .collect();
    assert!(shares.windows(2).all(|w| w[1] >= w[0]), "{shares:?}");
    assert!(shares[CELLS - 1] > 0.35, "the outlet boils: {shares:?}");
    // Each cell's state, against an independent flash of its own enthalpy.
    let composition = &edge(&s, "feed_spool").stream.composition;
    for name in &nodes {
        let cell = node(&s, name);
        let out = s
            .edges
            .iter()
            .find(|e| e.from == cell.id)
            .expect("every cell has a pipe out");
        let h = engine
            .enthalpy()
            .specific_enthalpy(&slate, composition, Kelvin(cell.temperature_k))
            .unwrap()
            .value()
            + out.stream.latent.map_or(0.0, |l| l.value());
        let le = engine
            .enthalpy()
            .temperature_from_enthalpy(&slate, composition, h, 1.0)
            .unwrap();
        let settled = EquilibriumLineFlash::default()
            .settle(
                &slate,
                composition,
                le,
                Pascal(cell.pressure_pa),
                &TroutonThermo::new(),
                engine.enthalpy(),
            )
            .unwrap();
        assert!(
            (settled.temperature.value() - cell.temperature_k).abs() < 1e-6,
            "{name}: published {} K, the flash of its enthalpy {} K",
            cell.temperature_k,
            settled.temperature.value()
        );
    }
}

/// **Each cell takes `G_i·(T̄_c − T_in,i)` from the metal, and each segment
/// drops what Darcy–Weisbach says**, by hand on the boiling coil. The metal's
/// tick average comes back out of the flue: `flue = K_f·(T̄_c − T_a)`,
/// `K_f = Q/(T_f − T_a)` (§40), so `T̄_c = T_a + flue·(T_f − T_a)/Q`. A cell's
/// inlet is its inflow's enthalpy settled at its own pressure — the boiling
/// stream's real temperature, not its liquid-equivalent — and its conductance
/// `G_i = W·(1 − e^{−UA_i/W})` on the stream's capacity rate `W = ṁ·c̄p`. The
/// last cell takes the remainder, which the first-law gate holds. A segment's
/// drop is `k·Q|Q|`, `k = f·L·ρ/(2·D·A²)` on its share of the coil's length and
/// its upwind cell's mixture density there (§58 fork 3) — to the one-pass lag
/// of the density the solve read.
#[test]
fn each_cell_takes_its_share_and_each_segment_its_friction() {
    let mut engine = build(&crude_coil(CELLS, "newton")).unwrap();
    run(&mut engine, 150);
    let s = engine.snapshot();
    let slate: Slate = engine.slate.clone();
    let enthalpy = engine.enthalpy();
    let (duty, ua, flame) = match &node(&s, "furnace").kind {
        NodeKind::Furnace {
            duty,
            coil,
            flame_temperature,
            ..
        } => (
            duty.value(),
            coil.conductance.value(),
            flame_temperature.value(),
        ),
        _ => unreachable!(),
    };
    let air = 293.15;
    let average = air + node(&s, "furnace").flue_loss_w.unwrap() * (flame - air) / duty;
    let composition = edge(&s, "feed_spool").stream.composition.clone();
    let mass_flow = edge(&s, "feed_spool").stream.mass_flow.value();
    let capacity = mass_flow * composition.mixture_cp(&slate).value();
    let conductance = capacity * -(-(ua / CELLS as f64) / capacity).exp_m1();
    let nodes = chain("furnace", CELLS);
    let flux = |e: &EdgeSnapshot| {
        enthalpy
            .stream_enthalpy_flux(&slate, &e.stream)
            .unwrap()
            .value()
    };
    let into = |id| s.edges.iter().find(|e| e.to == id).unwrap();
    let out_of = |id| s.edges.iter().find(|e| e.from == id).unwrap();
    // The state a stream of `h` [J/kg] stands at under `pressure`.
    let settled = |h: f64, pressure: f64| {
        let le = enthalpy
            .temperature_from_enthalpy(&slate, &composition, h, 1.0)
            .unwrap();
        (
            le,
            EquilibriumLineFlash::default()
                .settle(
                    &slate,
                    &composition,
                    le,
                    Pascal(pressure),
                    &TroutonThermo::new(),
                    enthalpy,
                )
                .unwrap(),
        )
    };
    for name in &nodes[..CELLS - 1] {
        let cell = node(&s, name);
        let (inflow, outflow) = (into(cell.id), out_of(cell.id));
        let heat = flux(outflow) - outflow.dissipation_w - flux(inflow);
        let (_, inlet) = settled(flux(inflow) / mass_flow, cell.pressure_pa);
        let share = conductance * (average - inlet.temperature.value());
        assert!(
            (heat - share).abs() < 1e-6 * heat,
            "{name}: took {heat} W, G·(T̄_c − T_in) = {share} W"
        );
        // Its segment out, by Darcy–Weisbach at its own mixture density.
        let next = s.nodes.iter().find(|n| n.id == outflow.to).unwrap();
        let h = (flux(outflow) - outflow.dissipation_w) / mass_flow;
        let le = enthalpy
            .temperature_from_enthalpy(&slate, &composition, h, 1.0)
            .unwrap();
        let rho = match EquilibriumLineFlash::default()
            .density(
                &slate,
                &composition,
                le,
                Pascal(cell.pressure_pa),
                &TroutonThermo::new(),
                enthalpy,
            )
            .unwrap()
        {
            Some(two_phase) => two_phase.mixture.value(),
            None => composition
                .density_at(&slate, Pascal(cell.pressure_pa), le)
                .unwrap()
                .value(),
        };
        let k = refinery_solvers::elements::pipe_resistance(
            0.02,
            800.0 / (CELLS - 1) as f64,
            0.24,
            rho,
        );
        let q = mass_flow / rho;
        let drop = k * q * q.abs();
        let published = cell.pressure_pa - next.pressure_pa;
        assert!(
            (published - drop).abs() < 1e-3 * drop,
            "{name}: its segment drops {published} Pa, Darcy–Weisbach {drop} Pa"
        );
    }
}

/// **Both fidelities agree** on the boiling coil, to their solves' tolerance
/// (measured: 5e-7 of the flow, 1.4e-5 K at the outlet).
#[test]
fn slow_a_coil_in_cells_runs_on_both_fidelities() {
    let mut answers = Vec::new();
    for flow in ["newton", "simple"] {
        let mut engine = build(&crude_coil(CELLS, flow)).unwrap();
        run(&mut engine, 100);
        let s = engine.snapshot();
        answers.push((
            edge(&s, "transfer_line").stream.mass_flow.value(),
            node(&s, "furnace").temperature_k,
            metal(&s, "furnace"),
        ));
    }
    let (a, b) = (answers[0], answers[1]);
    assert!(
        (a.0 - b.0).abs() < 1e-6 * a.0 && (a.1 - b.1).abs() < 1e-4 && (a.2 - b.2).abs() < 1e-4,
        "newton {a:?}, simple {b:?}"
    );
}

/// **With nothing flowing, every cell stands at the metal**, and the metal
/// keeps all the flame lets it absorb: fired = stored + flue.
#[test]
fn a_coil_in_cells_with_no_flow_stands_at_its_metal() {
    let mut engine = build(&heater_cells(4)).unwrap();
    run(&mut engine, 50);
    let feed = engine.graph.find_node("cold_feed").unwrap();
    engine
        .apply(Command::SetReservoirPressure {
            node: feed,
            pressure: Pascal(1.0e5),
        })
        .unwrap();
    let mut previous = engine.snapshot();
    for _ in 0..20 {
        run(&mut engine, 1);
        let s = engine.snapshot();
        assert!(edge(&s, "transfer_line").stream.mass_flow.value().abs() < 1e-9);
        let end = metal(&s, "heater");
        for name in chain("heater", 4) {
            assert_eq!(
                node(&s, &name).temperature_k,
                end,
                "{name} stands at the metal"
            );
        }
        let (duty, capacity) = match &node(&s, "heater").kind {
            NodeKind::Furnace { duty, coil, .. } => (duty.value(), coil.heat_capacity.value()),
            _ => unreachable!(),
        };
        let stored = capacity * (end - metal(&previous, "heater")) / engine.dt().value();
        let flue = node(&s, "heater").flue_loss_w.unwrap();
        assert!(
            (duty - stored - flue).abs() < 1e-9 * duty,
            "{duty} {stored} {flue}"
        );
        previous = s;
    }
}

/// **The coil's keys, refused by name**: apart, one cell, a length or bore
/// that is no tube, a cell's name already taken.
#[test]
fn a_coil_in_cells_refuses_what_is_not_one() {
    let base = heater_cells(4);
    for (src, expected) in [
        (
            base.replace("\ncoil_diameter_m = 0.5", ""),
            "but not all three",
        ),
        (
            base.replace("coil_cells = 4", "coil_cells = 1"),
            "at least two",
        ),
        (
            base.replace("coil_length_m = 0.1", "coil_length_m = 0.0"),
            "coil_length_m",
        ),
        (
            base.replace(
                "[nodes.product]",
                "[nodes.heater__cell_2]\ntype = \"junction\"\n\n[nodes.product]",
            ),
            "already has a node by that name",
        ),
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
