//! M62.4d — the choked boiling jet (docs/DESIGN.md §67.15): a hole on a plant
//! that boils flows as Leung's ω jet — the homogeneous equilibrium expansion
//! through an orifice on the ω equation of state, ω from two line flashes —
//! choked where the flux is greatest.
//!
//! The plant: the flashing rundown (`scenarios/flashing_rundown.toml`) with a
//! leak path on its rundown line, punctured — the 3 bar naphtha there a liquid
//! at 115 °C whose bubble pressure, 2.07 bar, stands above the atmosphere it
//! leaks to (a subcooled jet that boils in the hole), and, warmed to 135 °C,
//! boiling already (a two-phase jet). The ω law's own derivation is gated in
//! `elements.rs`; here the hole's flow is the law at the state the solve read,
//! the law against a direct homogeneous-equilibrium integration on the
//! engine's own flash, and the plant's books through the hole.

use refinery_core::components::{Composition, Slate};
use refinery_core::energy::T_REF;
use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::traits::{EnthalpyModel, LineFlashModel, ThermoModel};
use refinery_core::units::{Kelvin, Pascal, SquareMeter};
use refinery_core::Engine;
use refinery_solvers::elements::{omega_critical_ratio, omega_flux, ORIFICE_CD};
use refinery_solvers::line_flash::EquilibriumLineFlash;
use refinery_solvers::TroutonThermo;

const RUNDOWN: &str = include_str!("../../../scenarios/flashing_rundown.toml");
const HOLE: f64 = 2e-4; // m², a 16 mm hole

fn punctured_rundown(solver: &str) -> String {
    let src = RUNDOWN
        .replace("flow = \"newton\"", &format!("flow = \"{solver}\""))
        .replace(
            "name = \"rundown_line\"\nfrom = \"rundown\"\nto = \"rundown_valve\"\nlength_m = 30.0\ndiameter_m = 0.08",
            "name = \"rundown_line\"\nfrom = \"rundown\"\nto = \"rundown_valve\"\nlength_m = 30.0\n\
             diameter_m = 0.08\nleak_to = \"outside\"\n\n[nodes.outside]\ntype = \"atmosphere\"",
        );
    assert!(src.contains("leak_to"));
    src
}

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the scenario parses");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant builds: {e}"))
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

fn puncture(engine: &mut Engine) {
    let line = engine
        .graph
        .edge_ids()
        .find(|&e| engine.graph.pipe(e).name == "rundown_line")
        .unwrap();
    engine
        .apply(Command::PuncturePipe {
            edge: line,
            area: SquareMeter(HOLE),
        })
        .expect("a hole on a boiling plant is admitted");
}

fn warm(engine: &mut Engine, celsius: f64) {
    let supply = engine.graph.find_node("rundown").unwrap();
    engine
        .apply(Command::SetSourceTemperature {
            node: supply,
            temperature: Kelvin(celsius + 273.15),
        })
        .unwrap();
}

/// The specific volume [m³/kg] of `composition` from `liquid_equivalent`,
/// flashed isenthalpically at `pressure`.
fn volume(
    slate: &Slate,
    enthalpy: &dyn EnthalpyModel,
    composition: &Composition,
    liquid_equivalent: Kelvin,
    pressure: f64,
) -> f64 {
    match EquilibriumLineFlash
        .density(
            slate,
            composition,
            liquid_equivalent,
            Pascal(pressure),
            &TroutonThermo::new(),
            enthalpy,
        )
        .unwrap()
    {
        Some(two_phase) => 1.0 / two_phase.mixture.value(),
        None => {
            1.0 / composition
                .density_at(slate, Pascal(pressure), liquid_equivalent)
                .unwrap()
                .value()
        }
    }
}

/// The hole's inlet at the leak point: its pressure, its liquid-equivalent
/// temperature and its composition, as the last sweep left them.
fn inlet(engine: &Engine) -> (f64, Kelvin, Composition) {
    let point = engine.graph.find_node("rundown_line__leak_point").unwrap();
    let states = engine.node_states();
    let le = states
        .vapour
        .get(&point)
        .map(|v| v.liquid_equivalent)
        .unwrap_or(states.temperature[&point]);
    let pressure = node(&engine.snapshot(), "rundown_line__leak_point").pressure_pa;
    (pressure, le, states.composition[&point].clone())
}

/// `(η_s, ω)` by the API's two-point method, from an independent flash.
fn omega_inlet(engine: &Engine, p0: f64, le: Kelvin, composition: &Composition) -> (f64, f64) {
    let slate = &engine.slate;
    let enthalpy = engine.enthalpy();
    let boiling = EquilibriumLineFlash
        .density(
            slate,
            composition,
            le,
            Pascal(p0),
            &TroutonThermo::new(),
            enthalpy,
        )
        .unwrap()
        .is_some();
    let saturation = if boiling {
        p0
    } else {
        TroutonThermo::new()
            .bubble_pressure(slate, composition, le)
            .unwrap()
            .value()
    };
    let omega = (volume(slate, enthalpy, composition, le, 0.9 * saturation)
        / volume(slate, enthalpy, composition, le, saturation)
        - 1.0)
        / (1.0 / 0.9 - 1.0);
    (saturation / p0, omega.max(0.0))
}

/// The homogeneous equilibrium jet by direct integration on the engine's own
/// flash: `G = max_P √(2∫_P^{P₀} v dP)/v(P)` over throat pressures down to the
/// downstream's, the trapezoid rule on a fine grid.
fn hem_flux(engine: &Engine, p0: f64, p_down: f64, le: Kelvin, composition: &Composition) -> f64 {
    let steps = 4000;
    let (slate, enthalpy) = (&engine.slate, engine.enthalpy());
    let mut work = 0.0;
    let mut best = 0.0f64;
    let mut previous = volume(slate, enthalpy, composition, le, p0);
    for k in 1..=steps {
        let p = p0 - (p0 - p_down) * k as f64 / steps as f64;
        let v = volume(slate, enthalpy, composition, le, p);
        work += 0.5 * (previous + v) * (p0 - p_down) / steps as f64;
        best = best.max((2.0 * work).sqrt() / v);
        previous = v;
    }
    best
}

/// **The hole's flow is Leung's ω jet at the state the solve read**, for a
/// subcooled inlet that boils in the hole and for one already boiling, both
/// choked against the atmosphere: `ṁ = C_d·A·√(P₀/v₀)·g(max(η_a, η_c))`, `ω` and
/// `η_s` by the API's two-point method on an independent flash, settled. And
/// the ω law against the homogeneous equilibrium jet integrated directly on the
/// engine's flash: the ω equation of state is the API's approximation of it.
#[test]
fn a_hole_on_a_boiling_line_is_the_omega_jet() {
    let mut engine = build(&punctured_rundown("newton"));
    run(&mut engine, 200);
    puncture(&mut engine);
    for (case, celsius) in [("subcooled", None), ("boiling", Some(135.0))] {
        if let Some(c) = celsius {
            warm(&mut engine, c);
        }
        run(&mut engine, 300);
        let s = engine.snapshot();
        let (p0, le, composition) = inlet(&engine);
        let (eta_s, omega) = omega_inlet(&engine, p0, le, &composition);
        let v0 = volume(
            &engine.slate,
            engine.enthalpy(),
            &composition,
            le,
            p0 * eta_s.min(1.0),
        )
        .min(volume(
            &engine.slate,
            engine.enthalpy(),
            &composition,
            le,
            p0,
        ));
        let eta_a = 101_325.0 / p0;
        let critical = omega_critical_ratio(eta_s, omega);
        assert!(
            eta_a < critical,
            "{case}: the jet must choke ({eta_a} against {critical})"
        );
        let flux = (p0 / v0).sqrt() * omega_flux(eta_a.max(critical), eta_s, omega);
        let expected = ORIFICE_CD * HOLE * flux;
        let leaked = edge(&s, "rundown_line__leak").stream.mass_flow.value();
        assert!(
            (leaked - expected).abs() < 1e-5 * expected,
            "{case}: the hole leaks {leaked} kg/s, the ω jet {expected} kg/s (η_s {eta_s}, ω {omega})"
        );
        let hem = hem_flux(&engine, p0, 101_325.0, le, &composition);
        // Measured: subcooled, ω 28.9 at η_s 0.704, choked at its bubble
        // pressure (η_c = η_s), 11 040.5 against the HEM's 11 037.6 kg/(m²·s),
        // +0.03 %; boiling, ω 1.64, η_c 0.669, 1 412.4 against 1 396.9, +1.1 %.
        let gap = flux / hem - 1.0;
        assert!(
            gap.abs() < 0.02,
            "{case}: the ω law is {gap:+.4} off the HEM jet"
        );
    }
}

/// Net power [W] and mass [kg/s] across the plant's boundary, and the gross
/// enthalpy flow they are graded against (`flashing_rundown_reference.rs`'s
/// books, the hole one more edge leaving the plant).
fn boundary(s: &Snapshot, slate: &Slate, enthalpy: &dyn EnthalpyModel) -> (f64, f64, f64) {
    let kind_of = |id| &s.nodes.iter().find(|n| n.id == id).unwrap().kind;
    let outside = |k: &NodeKind| {
        matches!(
            k,
            NodeKind::Source { .. } | NodeKind::Sink { .. } | NodeKind::Atmosphere
        )
    };
    let (mut power, mut mass, mut gross) = (0.0, 0.0, 0.0);
    for e in &s.edges {
        let flux = enthalpy
            .stream_enthalpy_flux(slate, &e.stream)
            .unwrap()
            .value();
        match (outside(kind_of(e.from)), outside(kind_of(e.to))) {
            (true, false) => {
                power += flux;
                mass += e.stream.mass_flow.value();
            }
            (false, true) => {
                power += e.dissipation_w - flux;
                mass -= e.stream.mass_flow.value();
            }
            (false, false) => power += e.dissipation_w,
            (true, true) => panic!("an edge with both ends outside"),
        }
        gross += flux.abs();
    }
    (power, mass, gross)
}

fn holdups(s: &Snapshot, slate: &Slate) -> (f64, f64) {
    s.nodes
        .iter()
        .filter_map(|n| match &n.kind {
            NodeKind::Tank(t) => Some((
                t.mass.value()
                    * t.composition.mixture_cp(slate).value()
                    * (t.temperature.value() - T_REF.value()),
                t.mass.value(),
            )),
            _ => None,
        })
        .fold((0.0, 0.0), |(e, m), (de, dm)| (e + de, m + dm))
}

/// **The plant's books close through the hole, every tick**: what the jet
/// carries out, its latent heat included, leaves the plant's energy and mass
/// exactly — through the puncture, the subcooled jet, and the supply warmed to
/// boil.
#[test]
fn the_books_close_through_a_boiling_jet() {
    let mut engine = build(&punctured_rundown("newton"));
    let slate = engine.slate.clone();
    let dt = engine.dt().value();
    let mut previous = engine.snapshot();
    let (mut worst, mut worst_mass) = (0.0f64, 0.0f64);
    for t in 1..=900 {
        if t == 100 {
            puncture(&mut engine);
        }
        if t == 500 {
            warm(&mut engine, 135.0);
        }
        run(&mut engine, 1);
        let s = engine.snapshot();
        let (energy, mass) = holdups(&s, &slate);
        let (energy_before, mass_before) = holdups(&previous, &slate);
        let (power, mass_in, gross) = boundary(&s, &slate, engine.enthalpy());
        worst = worst.max(((energy - energy_before) / dt - power).abs() / gross);
        let throughput: f64 = s
            .edges
            .iter()
            .map(|e| e.stream.mass_flow.value().abs())
            .sum();
        worst_mass = worst_mass.max(((mass - mass_before) / dt - mass_in).abs() / throughput);
        previous = s;
    }
    assert!(
        edge(&previous, "rundown_line__leak")
            .stream
            .mass_flow
            .value()
            > 0.1,
        "the hole leaks"
    );
    assert!(worst <= 1e-8, "energy books: worst {worst:e}");
    assert!(worst_mass <= 1e-8, "mass books: worst {worst_mass:e}");
}

/// **Both fidelities agree** on the boiling jet.
#[test]
fn slow_a_boiling_jet_runs_on_both_fidelities() {
    let mut leaks = Vec::new();
    for solver in ["newton", "simple"] {
        let mut engine = build(&punctured_rundown(solver));
        run(&mut engine, 100);
        puncture(&mut engine);
        warm(&mut engine, 135.0);
        run(&mut engine, 300);
        leaks.push(
            edge(&engine.snapshot(), "rundown_line__leak")
                .stream
                .mass_flow
                .value(),
        );
    }
    assert!((leaks[0] - leaks[1]).abs() < 1e-6 * leaks[0], "{leaks:?}");
}

/// **A furnace's tubes burst on a boiling plant and its hole leaks the jet**:
/// the boiling crude unit's coil overheated past a lowered limit, the burst
/// hole on its transfer line flowing, and the fire it feeds the leaked mass
/// times the fluid's heating value (M37, §42) — the furnace M62.4a refused.
#[test]
fn a_furnace_bursts_on_a_boiling_plant() {
    let src = include_str!("../../../scenarios/crude_unit.toml")
        .replace(
            "boiloff = \"none\"",
            "boiloff = \"flash\"\nline_flash = \"equilibrium\"",
        )
        .replace("tube_failure_c = 550.0", "tube_failure_c = 362.0")
        .replace(
            "fluid_heating_value_mj_per_kg = 42.686",
            "fluid_heating_value_mj_per_kg = 42.686
coil_cells = 2
coil_length_m = 20.0
coil_diameter_m = 0.40",
        )
        .replace(
            "name = \"transfer_line\"\nfrom = \"furnace\"\nto = \"column\"\nlength_m = 15.0\ndiameter_m = 0.08",
            "name = \"transfer_line\"\nfrom = \"furnace\"\nto = \"column\"\nlength_m = 15.0\ndiameter_m = 0.40",
        );
    let mut engine = build(&src);
    let furnace = engine.graph.find_node("furnace").unwrap();
    engine
        .apply(Command::SetFurnaceDuty {
            node: furnace,
            duty: refinery_core::units::Watt(60e6),
        })
        .unwrap();
    let mut burst_at = None;
    for t in 1..=4000 {
        run(&mut engine, 1);
        let s = engine.snapshot();
        if let NodeKind::Furnace { tubes, .. } = &node(&s, "furnace").kind {
            if tubes.state.is_failed() {
                burst_at = Some(t);
                break;
            }
        }
    }
    let t = burst_at.expect("the coil passes its lowered limit and the tubes burst");
    run(&mut engine, 20);
    let s = engine.snapshot();
    let hole = s
        .edges
        .iter()
        .find(|e| e.name.starts_with("transfer_line") && e.name.ends_with("__leak"))
        .expect("the burn-out hole");
    let leaked = hole.stream.mass_flow.value();
    assert!(leaked > 0.0, "burst at tick {t}, the hole leaks nothing");
    assert!(hole.stream.vapour_fraction.is_some(), "the jet boils");
    let fire = node(&s, "furnace").tube_fire_w.unwrap();
    assert!(
        (fire - leaked * 42.686e6).abs() < 1e-9 * fire,
        "the fire {fire} W against {leaked} kg/s burning"
    );
}
