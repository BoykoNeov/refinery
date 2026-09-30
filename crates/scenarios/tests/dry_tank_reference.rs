//! M24.1: a tank that runs dry (docs/DESIGN.md §28, ledger row B29).
//!
//! Until M24 a tank drawn past empty kept delivering — it is pinned at its
//! hydrostatic bottom pressure, and a pinned node supplies whatever its edges
//! draw — and the tank update's `.max(0.0)` booked the overdraw as nothing:
//! 200 149 kg created on `tank_flow_control` over 30 000 ticks. Now the shared
//! solve re-runs such a tank as a STARVED free node supplying `m/dt`, the sweep
//! resolves it as a mixing point, and the clamp is a tripwire.
//!
//! Nothing in the corpus runs dry inside 6 000 ticks, so every gate here runs on
//! the demo (`scenarios/tank_runs_dry.toml`) or on a fixture derived from a
//! shipped plant. The solve-level exits of the driver (a starvation-only repeat,
//! recovery, the union, gate 10's floating tank) are in
//! `crates/solvers/tests/invariants.rs`, on stub passes, because a real plant
//! reaches whichever exit its physics reaches.

use refinery_core::components::Composition;
use refinery_core::engine::EngineConfig;
use refinery_core::graph::{EdgeId, Node, NodeId, NodeKind, PlantGraph};
use refinery_core::snapshot::Command;
use refinery_core::traits::{FlowSolver, HydraulicSolution, InflowEnthalpy, StarvedTank};
use refinery_core::units::{KgPerSec, Pascal, Seconds, Watt, P_ATM};
use refinery_core::{Engine, SimError};
use refinery_solvers::{NewtonFlowSolver, SimpleFlowSolver};

const DEMO: &str = include_str!("../../../scenarios/tank_runs_dry.toml");
const FLOW_LOOP: &str = include_str!("../../../scenarios/tank_flow_control.toml");
const TRIP: &str = include_str!("../../../scenarios/tank_overfill_trip.toml");

/// The engine's own floor below which a holdup has no temperature
/// (`engine::MIN_THERMAL_MASS_KG`, private there).
const THERMAL_FLOOR_KG: f64 = 1e-6;
/// `engine::ROUNDING_MASS_FRACTION`, private there: the share of a holdup's
/// gross traffic its mass update may land below zero by rounding alone.
const ROUNDING_MASS_FRACTION: f64 = 1e-9;

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the fixture must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("the fixture must build: {e}"))
}

fn swap(src: &str, from: &str, to: &str) -> String {
    assert!(
        src.contains(from),
        "the fixture's substitution must land: `{from}` is not in the plant"
    );
    src.replacen(from, to, 1)
}

fn on_simple(src: &str) -> String {
    swap(src, r#"flow = "newton""#, r#"flow = "simple""#)
}

fn tick(engine: &mut Engine) {
    engine
        .tick()
        .unwrap_or_else(|e| panic!("tick {}: {e}", engine.snapshot().tick + 1));
}

fn node(engine: &Engine, name: &str) -> NodeId {
    engine
        .graph
        .find_node(name)
        .unwrap_or_else(|| panic!("the fixture has a node '{name}'"))
}

fn edge(engine: &Engine, name: &str) -> EdgeId {
    engine
        .graph
        .edge_ids()
        .find(|&e| engine.graph.pipe(e).name == name)
        .unwrap_or_else(|| panic!("the fixture has a pipe '{name}'"))
}

fn tank(engine: &Engine, name: &str) -> refinery_core::graph::TankState {
    match &engine.graph.node(node(engine, name)).kind {
        NodeKind::Tank(t) => t.clone(),
        other => panic!("{name} is a tank, not {other:?}"),
    }
}

fn solution(engine: &Engine) -> &HydraulicSolution {
    engine
        .last_solution()
        .expect("a tick has run, so there is a solution")
}

fn starved(engine: &Engine, name: &str) -> Option<StarvedTank> {
    solution(engine).starved.get(&node(engine, name)).copied()
}

fn flow(engine: &Engine, pipe: &str) -> f64 {
    solution(engine).edge_mass_flow[&edge(engine, pipe)]
}

/// Run until the named tank starves with some inventory still in it — the
/// DRYING tick — and return the tank as it stood at the start of that tick.
fn run_to_drying_tick(
    engine: &mut Engine,
    name: &str,
    limit: u64,
) -> refinery_core::graph::TankState {
    for _ in 0..limit {
        let before = tank(engine, name);
        tick(engine);
        if starved(engine, name).is_some() {
            assert!(
                before.mass.value() > THERMAL_FLOOR_KG,
                "the first starved tick must be the one the tank dries on, with an \
                 inventory left to deliver ({:.3e} kg)",
                before.mass.value()
            );
            return before;
        }
    }
    panic!("'{name}' never starved within {limit} ticks");
}

// --- gate 1 -----------------------------------------------------------------

/// `tank_flow_control` with its supply tank started nearly empty, so it runs dry
/// inside the test rather than at tick 13 314. Sealed: two tanks, a pump and a
/// valve, nothing in or out.
fn sealed_fixture() -> String {
    let src = swap(FLOW_LOOP, "initial_level_m = 8.0", "initial_level_m = 0.5");
    // The flow loop holds 12 kg/s, so ~10 000 kg is gone in ~830 ticks.
    src
}

fn holdup_total(engine: &Engine) -> f64 {
    engine
        .graph
        .node_ids()
        .filter_map(|id| match &engine.graph.node(id).kind {
            NodeKind::Tank(t) => Some(t.mass.value()),
            _ => None,
        })
        .sum()
}

/// The graph with `tank` replaced by a Source pinned at its bottom pressure: the
/// plant as the PRE-M24 rule solved it, where a tank supplies whatever its edges
/// draw. Node and edge ids come out identical, because both are assigned in
/// insertion order.
fn pinned_copy(engine: &Engine, tank_id: NodeId) -> PlantGraph {
    let mut copy = PlantGraph::new();
    for id in engine.graph.node_ids() {
        let original = engine.graph.node(id);
        let node = match &original.kind {
            NodeKind::Tank(t) if id == tank_id => Node {
                name: original.name.clone(),
                kind: NodeKind::Source {
                    pressure: t.bottom_pressure(&engine.slate),
                    temperature: t.temperature,
                    composition: t.composition.clone(),
                },
                heat_input: original.heat_input,
            },
            _ => original.clone(),
        };
        assert_eq!(copy.add_node(node), id, "ids must survive the copy");
    }
    for e in engine.graph.edge_ids() {
        let (from, to) = engine.graph.endpoints(e);
        assert_eq!(copy.add_pipe(from, to, engine.graph.pipe(e).clone()), e);
    }
    copy
}

/// **Gate 1: a sealed plant keeps its mass when its supply runs dry.**
///
/// The holdup total may move only by the solve's own node imbalances: each tick
/// every free node's residual is at most the reported worst one, and there are
/// at most three free nodes (the pump, the valve and, once it is dry, the supply
/// tank). That is the bound, accumulated tick by tick from what each solve
/// reported, plus the rounding of summing two ~90 t inventories.
///
/// The CONTROL is the pre-M24 rule, computed rather than remembered: on every
/// starved tick the same state is solved with the supply tank PINNED (a Source
/// at its bottom pressure), and what the old clamp would have booked is
/// `q_wet·dt − m`. Without it this gate would pass on a plant that never
/// starved.
#[test]
fn a_sealed_plant_keeps_its_mass_when_its_supply_runs_dry() {
    for (fidelity, src) in [
        ("newton", sealed_fixture()),
        ("simple", on_simple(&sealed_fixture())),
    ] {
        let mut engine = build(&src);
        let dt = engine.dt().value();
        let supply = node(&engine, "supply_tank");
        let suction = edge(&engine, "suction");
        let total0 = holdup_total(&engine);
        let mut bound = 0.0;
        let mut starved_ticks = 0;
        let mut would_have_created = 0.0;
        for _ in 0..1_200 {
            let mass_old = tank(&engine, "supply_tank").mass.value();
            // The pre-M24 answer, from the same start-of-tick state.
            let pinned = if fidelity == "newton" {
                let copy = pinned_copy(&engine, supply);
                let wet = NewtonFlowSolver::default()
                    .solve(&copy, &engine.slate, engine.node_states(), engine.dt())
                    .expect("the pinned copy solves");
                Some(wet.edge_mass_flow[&suction])
            } else {
                None
            };
            tick(&mut engine);
            let sol = solution(&engine);
            let free_nodes = if sol.starved.is_empty() { 2.0 } else { 3.0 };
            bound += free_nodes * sol.diagnostics.residual * dt + 1e-12 * total0;
            if sol.starved.contains_key(&supply) {
                starved_ticks += 1;
                if let Some(q_wet) = pinned {
                    would_have_created += (q_wet * dt - mass_old).max(0.0);
                }
            }
        }
        let drift = holdup_total(&engine) - total0;
        eprintln!(
            "gate 1 {fidelity}: starved ticks {starved_ticks}, drift {drift:.3e} kg, bound \
             {bound:.3e} kg, old rule would have created {would_have_created:.3} kg"
        );
        assert!(
            starved_ticks > 100,
            "{fidelity}: the supply must run dry inside the run"
        );
        assert!(
            tank(&engine, "supply_tank").mass.value() <= THERMAL_FLOOR_KG,
            "{fidelity}: the supply tank ends empty"
        );
        assert!(
            drift.abs() <= bound,
            "{fidelity}: the sealed plant's holdup moved by {drift:.3e} kg, beyond the \
             {bound:.3e} kg its own solves' residuals allow"
        );
        if fidelity == "newton" {
            assert!(
                would_have_created > 1.0e3,
                "the control must show the pre-M24 rule creating mass on this fixture \
                 ({would_have_created:.3e} kg), or the gate above proves nothing"
            );
        }
    }
}

// --- gate 2 -----------------------------------------------------------------

/// **Gate 2: the drying tick delivers exactly what is left.**
///
/// On the tick a tank starves with inventory in it, the tank ends empty to
/// within its own recorded residual, that residual is inside the solver's own
/// promise at the node, and the tank's pressure is BELOW the one it would pin at
/// while wet — it starved because the network pulled harder than it holds.
///
/// Run at `dt = 0.5` as well as the demo's 1.0: a rule that compared a RATE
/// against a MASS (`q_out > m`, no `dt`) is the demo's own rule at `dt = 1`.
/// At 0.5 it starves a tank that could still have delivered, whose supply `m/dt`
/// then exceeds the wet draw — and the pressure clause is what sees it.
#[test]
fn the_drying_tick_delivers_exactly_what_is_left() {
    for (label, src) in [
        ("dt = 1.0", DEMO.to_string()),
        ("dt = 0.5", swap(DEMO, "dt = 1.0 ", "dt = 0.5 ")),
        (
            "dt = 0.5, simple",
            on_simple(&swap(DEMO, "dt = 1.0 ", "dt = 0.5 ")),
        ),
    ] {
        let mut engine = build(&src);
        let dt = engine.dt().value();
        let before = run_to_drying_tick(&mut engine, "buffer_tank", 6_000);
        let report = starved(&engine, "buffer_tank").expect("starved");
        let out = flow(&engine, "suction");
        let fed = flow(&engine, "into_tank");
        let after = tank(&engine, "buffer_tank");
        let wet_pressure = before.bottom_pressure(&engine.slate).value();
        let pressure = solution(&engine).node_pressure[&node(&engine, "buffer_tank")].value();
        eprintln!(
            "gate 2 {label}: tick {}, m_old {:.6} kg, supply {:.6} kg/s, residual {:.3e}, out \
             {out:.6}, in {fed:.6}, P {pressure:.1} Pa against wet {wet_pressure:.1} Pa, m_new \
             {:.3e}",
            engine.snapshot().tick,
            before.mass.value(),
            report.supply.value(),
            report.residual.value(),
            after.mass.value()
        );
        assert_eq!(
            report.supply.value(),
            before.mass.value() / dt,
            "{label}: the supply is the whole inventory over the tick"
        );
        // The solver's own promise at this node: `tol_abs + tol_rel·scale`
        // (newton 1e-8 + 1e-8·scale; simple 1e-8 + 1e-6·scale).
        let promise = 1e-8 + 1e-6 * out.abs();
        assert!(
            report.residual.value().abs() < promise,
            "{label}: the starved tank's own residual {:.3e} kg/s is outside the solver's \
             promise {promise:.3e} kg/s",
            report.residual.value()
        );
        assert!(
            after.mass.value()
                <= report.residual.value().abs() * dt
                    + ROUNDING_MASS_FRACTION * (before.mass.value() + (out + fed) * dt),
            "{label}: the tank must end the drying tick empty, not holding {:.3e} kg",
            after.mass.value()
        );
        assert!(
            pressure < wet_pressure,
            "{label}: a tank starves because the network pulls harder than it holds, so its \
             solved pressure ({pressure:.1} Pa) must be below the {wet_pressure:.1} Pa it pins \
             at while wet"
        );
    }
}

// --- gate 3 and 3b ----------------------------------------------------------

/// The pass-through the sweep must produce, recomputed here from the published
/// inflow and the tank's start-of-tick state, in the sweep's own order of
/// operations: the tank's own term first, then the one inflow.
fn expected_mix(
    engine: &Engine,
    own: &refinery_core::graph::TankState,
    supply: f64,
) -> (Composition, f64) {
    let into = &engine.graph.pipe(edge(engine, "into_tank")).stream;
    let fed = into.mass_flow.value();
    let mut weights: Vec<f64> = own
        .composition
        .fractions()
        .iter()
        .map(|f| supply * f)
        .collect();
    for (w, f) in weights.iter_mut().zip(into.composition.fractions()) {
        *w += fed * f;
    }
    let composition = Composition::from_weights(&weights).expect("a valid mix");
    let model = engine.enthalpy();
    let slate = &engine.slate;
    let own_rate = model
        .enthalpy_flux(slate, &own.composition, KgPerSec(supply), own.temperature)
        .unwrap()
        .value();
    let own_capacity = supply
        * model
            .spot_cp(slate, &own.composition, own.temperature)
            .unwrap()
            .value();
    let in_rate = 0.0
        + model
            .enthalpy_flux(slate, &into.composition, KgPerSec(fed), into.temperature)
            .unwrap()
            .value();
    let in_capacity = 0.0
        + fed
            * model
                .spot_cp(slate, &into.composition, into.temperature)
                .unwrap()
                .value();
    let temperature = model
        .mix_temperature(
            slate,
            &composition,
            InflowEnthalpy {
                enthalpy_rate: own_rate + in_rate,
                mass_rate: supply + (0.0 + fed),
                capacity_rate: own_capacity + in_capacity,
            },
        )
        .unwrap()
        .value();
    (composition, temperature)
}

/// **Gate 3: a dry tank passes its feed through, as the mix.**
///
/// On every starved tick of the demo — the drying tick, where the tank still
/// holds 53% diesel, and the thousands after it — the tank's resolved state and
/// its outflow's composition equal the mix of what it held (at its supply rate)
/// and what arrived, BIT FOR BIT against the sweep's own formula. And the tank's
/// own books — mass, each component, energy — close over the whole run within
/// the residuals its solves recorded.
///
/// The energy book has a premise and asserts it: the buffer tank carries no
/// fire and no ambient `UA` (row B35 — heat into a dry tank is not conserved).
#[test]
fn a_dry_tank_passes_its_feed_through_as_the_mix() {
    for (fidelity, src) in [("newton", DEMO.to_string()), ("simple", on_simple(DEMO))] {
        let mut engine = build(&src);
        let tank_id = node(&engine, "buffer_tank");
        let premise = engine.graph.node(tank_id);
        assert_eq!(
            premise.heat_input,
            Watt(0.0),
            "premise: no fire on the tank"
        );
        assert_eq!(
            tank(&engine, "buffer_tank").ambient_ua,
            refinery_core::units::WattPerKelvin(0.0),
            "premise: no ambient UA"
        );

        let dt = engine.dt().value();
        let slate = engine.slate.clone();
        let components = slate.len();
        let initial = tank(&engine, "buffer_tank");
        let energy_of = |engine: &Engine, t: &refinery_core::graph::TankState| {
            engine
                .enthalpy()
                .enthalpy_stock(&slate, &t.composition, t.mass, t.temperature)
                .unwrap()
        };
        let e0 = energy_of(&engine, &initial);
        let mut fed_mass = 0.0;
        let mut drawn_mass = 0.0;
        let mut component_net = vec![0.0; components];
        let mut energy_net = 0.0;
        let mut gross = initial.mass.value();
        let mut gross_energy = e0.abs();
        let mut allowance = 0.0;
        let mut starved_ticks = 0;
        // The largest specific enthalpy [J/kg] that left: an over-draw of `δm`
        // carries at most `δm·h_max` out of the energy book with it.
        let mut h_max: f64 = 0.0;
        for _ in 0..2_000 {
            let own = tank(&engine, "buffer_tank");
            tick(&mut engine);
            let into = engine.graph.pipe(edge(&engine, "into_tank")).stream.clone();
            let suction = engine.graph.pipe(edge(&engine, "suction")).stream.clone();
            let fed = into.mass_flow.value();
            let out = suction.mass_flow.value();
            // What left is at the tank's RESOLVED state — the tank is upwind of
            // its suction line — which is the published suction composition, and
            // the tank's resolved temperature (the suction's published temperature
            // is its pump-end outlet, heated by friction).
            let t_left = engine.node_states().temperature[&tank_id];
            fed_mass += fed * dt;
            drawn_mass += out * dt;
            for (net, (arrived, left)) in component_net.iter_mut().zip(
                into.composition
                    .fractions()
                    .iter()
                    .zip(suction.composition.fractions()),
            ) {
                *net += (fed * arrived - out * left) * dt;
            }
            let h_in = engine
                .enthalpy()
                .enthalpy_flux(&slate, &into.composition, into.mass_flow, into.temperature)
                .unwrap()
                .value();
            let h_out = engine
                .enthalpy()
                .enthalpy_flux(&slate, &suction.composition, suction.mass_flow, t_left)
                .unwrap()
                .value();
            energy_net += (h_in - h_out) * dt;
            if out > 0.0 {
                h_max = h_max.max((h_out / out).abs());
            }
            gross += (fed + out) * dt;
            gross_energy += (h_in.abs() + h_out.abs()) * dt;

            if let Some(report) = starved(&engine, "buffer_tank") {
                starved_ticks += 1;
                allowance += report.residual.value().abs() * dt;
                let (composition, temperature) = expected_mix(&engine, &own, report.supply.value());
                assert_eq!(
                    engine.node_states().composition[&tank_id].fractions(),
                    composition.fractions(),
                    "{fidelity} tick {}: the dry tank's composition is the mix",
                    engine.snapshot().tick
                );
                assert_eq!(
                    suction.composition.fractions(),
                    composition.fractions(),
                    "{fidelity} tick {}: its outflow carries the mix",
                    engine.snapshot().tick
                );
                assert_eq!(
                    t_left.value(),
                    temperature,
                    "{fidelity} tick {}: the dry tank's temperature is the mix's",
                    engine.snapshot().tick
                );
            }
        }
        let end = tank(&engine, "buffer_tank");
        let bound = allowance + ROUNDING_MASS_FRACTION * gross;
        let mass_miss = (end.mass.value() - initial.mass.value()) - (fed_mass - drawn_mass);
        let mut worst_component: f64 = 0.0;
        for (c, net) in component_net.iter().enumerate() {
            let held =
                |t: &refinery_core::graph::TankState| t.mass.value() * t.composition.fractions()[c];
            worst_component = worst_component.max(((held(&end) - held(&initial)) - net).abs());
        }
        let energy_miss = (energy_of(&engine, &end) - e0) - energy_net;
        let energy_bound = allowance * h_max + ROUNDING_MASS_FRACTION * gross_energy;
        eprintln!(
            "gate 3 {fidelity}: starved ticks {starved_ticks}, mass miss {mass_miss:.3e} kg, worst \
             component {worst_component:.3e} kg, bound {bound:.3e} kg; energy miss \
             {energy_miss:.3e} J, bound {energy_bound:.3e} J"
        );
        assert!(
            starved_ticks > 700,
            "{fidelity}: the demo runs dry inside 2 000 ticks"
        );
        assert!(
            mass_miss.abs() <= bound,
            "{fidelity}: mass book {mass_miss:.3e} kg"
        );
        assert!(
            worst_component <= bound,
            "{fidelity}: component book {worst_component:.3e} kg"
        );
        assert!(
            energy_miss.abs() <= energy_bound,
            "{fidelity}: energy book {energy_miss:.3e} J"
        );
    }
}

/// **Gate 3b: a fire on a dry tank stays off the stream** (mutation 5).
///
/// No energy balance can see whether the heat went into the passing stream as
/// well: with a fire on a dry tank the correct engine does not close its energy
/// book either (row B35). The stream itself can — the dry tank's resolved
/// temperature, which is what its outflow carries, must be the UNHEATED mix,
/// bit for bit.
#[test]
fn a_fire_on_a_dry_tank_stays_off_the_stream() {
    let mut engine = build(DEMO);
    run_to_drying_tick(&mut engine, "buffer_tank", 6_000);
    tick(&mut engine);
    let tank_id = node(&engine, "buffer_tank");
    engine
        .apply(Command::SetHeatInput {
            node: tank_id,
            power: Watt(2.0e5),
        })
        .expect("a fire is a legal command");
    for _ in 0..5 {
        let own = tank(&engine, "buffer_tank");
        tick(&mut engine);
        let report = starved(&engine, "buffer_tank").expect("still dry: the pump still runs");
        let (_, unheated) = expected_mix(&engine, &own, report.supply.value());
        assert_eq!(
            engine.node_states().temperature[&tank_id].value(),
            unheated,
            "the heat on a dry tank must not reach the stream through the mix"
        );
    }
    // Control: the fire is really there, and 200 kW on ~5 kg/s would be ~20 K.
    assert_eq!(engine.graph.node(tank_id).heat_input, Watt(2.0e5));
}

// --- gate 4 -----------------------------------------------------------------

/// **Gate 4: a starved tank recovers.** The demo's tank runs dry, and its pump
/// is then stopped. A stopped pump still CONDUCTS (M22), so this recovers only
/// because the feed then pushes into the tank faster than the stopped pump's
/// resistance lets it out — which is the premise, checked on the flows. The next
/// tick the tank is wet, its pressure is exactly the bottom pressure it pins at,
/// and it refills.
#[test]
fn a_starved_tank_recovers_when_its_pump_stops() {
    let mut engine = build(DEMO);
    run_to_drying_tick(&mut engine, "buffer_tank", 6_000);
    for _ in 0..10 {
        tick(&mut engine);
    }
    assert!(
        starved(&engine, "buffer_tank").is_some(),
        "dry before the stop"
    );
    let pump = node(&engine, "transfer_pump");
    engine
        .apply(Command::SetPumpOn {
            node: pump,
            on: false,
        })
        .expect("stopping a pump is legal");
    let before = tank(&engine, "buffer_tank");
    tick(&mut engine);
    assert!(
        starved(&engine, "buffer_tank").is_none(),
        "the tick after the pump stops, the tank is wet again"
    );
    let pinned = before.bottom_pressure(&engine.slate).value();
    let pressure = solution(&engine).node_pressure[&node(&engine, "buffer_tank")].value();
    assert_eq!(
        pressure, pinned,
        "a wet tank pins its bottom pressure exactly"
    );
    assert!(
        flow(&engine, "into_tank") > flow(&engine, "suction"),
        "premise: the feed now outruns what leaves through the stopped pump"
    );
    // It refills, by exactly what the network puts into it: a wet tank's own
    // book, over ticks that each gain it mass.
    let dt = engine.dt().value();
    let m1 = tank(&engine, "buffer_tank").mass.value();
    let mut net = 0.0;
    let mut gross = m1;
    let mut last = m1;
    for _ in 0..50 {
        tick(&mut engine);
        assert!(starved(&engine, "buffer_tank").is_none(), "and stays wet");
        let (fed, out) = (flow(&engine, "into_tank"), flow(&engine, "suction"));
        net += (fed - out) * dt;
        gross += (fed + out) * dt;
        let m = tank(&engine, "buffer_tank").mass.value();
        assert!(
            m > last,
            "it gains mass every tick: {last:.6e} -> {m:.6e} kg"
        );
        last = m;
    }
    let m2 = tank(&engine, "buffer_tank").mass.value();
    assert!(
        ((m2 - m1) - net).abs() <= ROUNDING_MASS_FRACTION * gross,
        "it refills by what arrives: {:.6e} kg against {net:.6e} kg",
        m2 - m1
    );
}

// --- gate 5 -----------------------------------------------------------------

/// **Gate 5: the boundary does not cycle.** `tank_overfill_trip`'s receiving tank
/// is drained after the trip and sits on the empty boundary from tick ~10 000,
/// where its drain carries solver noise: M24.0 measured ~10 000 events that touch
/// the old clamp, every other tick on Newton. A starvation-only repeat is
/// ACCEPTED there rather than refused, and the tripwire holds on every one.
///
/// Newton must actually reach the population — a run that never starves would
/// pass this gate on a driver that refuses every repeat.
#[test]
fn a_tank_on_the_empty_boundary_does_not_cycle() {
    for (fidelity, src) in [("newton", TRIP.to_string()), ("simple", on_simple(TRIP))] {
        let mut engine = build(&src);
        let mut starved_ticks = 0;
        for _ in 0..12_500 {
            tick(&mut engine);
            if !solution(&engine).starved.is_empty() {
                starved_ticks += 1;
            }
        }
        eprintln!("gate 5 {fidelity}: starved ticks {starved_ticks} of 12 500");
        if fidelity == "newton" {
            assert!(
                starved_ticks > 100,
                "the drained tank must reach the boundary population ({starved_ticks})"
            );
        }
    }
}

// --- gate 6 -----------------------------------------------------------------

/// A real solver whose answer the test then bends: `over_draw` [kg] more than
/// the tank can give leaves through `suction` on tick `on_tick`, and `report`
/// optionally claims the tank was starved with the given residual [kg/s].
struct Bent {
    inner: NewtonFlowSolver,
    tank: NodeId,
    suction: EdgeId,
    into: EdgeId,
    over_draw: f64,
    report: Option<f64>,
}

impl FlowSolver for Bent {
    fn solve(
        &mut self,
        graph: &PlantGraph,
        slate: &refinery_core::components::Slate,
        previous_states: &refinery_core::energy::NodeStates,
        dt: Seconds,
    ) -> Result<HydraulicSolution, SimError> {
        let mut solution = self.inner.solve(graph, slate, previous_states, dt)?;
        let NodeKind::Tank(tank) = &graph.node(self.tank).kind else {
            unreachable!()
        };
        let mass = tank.mass.value();
        let fed = solution.edge_mass_flow[&self.into];
        solution
            .edge_mass_flow
            .insert(self.suction, (mass + self.over_draw) / dt.value() + fed);
        if let Some(residual) = self.report {
            solution.starved.insert(
                self.tank,
                StarvedTank {
                    supply: KgPerSec(mass / dt.value()),
                    residual: KgPerSec(residual),
                },
            );
        }
        Ok(solution)
    }

    fn name(&self) -> &'static str {
        "bent"
    }
}

fn bent_engine(over_draw: f64, report: Option<f64>) -> Engine {
    let built = build(DEMO);
    let dt = built.dt();
    let tank = node(&built, "buffer_tank");
    let suction = edge(&built, "suction");
    let into = edge(&built, "into_tank");
    let Engine { graph, slate, .. } = built;
    Engine::new(
        graph,
        slate,
        EngineConfig { dt },
        Box::new(Bent {
            inner: NewtonFlowSolver::default(),
            tank,
            suction,
            into,
            over_draw,
            report,
        }),
        Box::new(refinery_solvers::ConstantThermo),
        Box::new(refinery_solvers::NoReactions),
        Box::new(refinery_solvers::CutPointSplitter),
        Box::new(refinery_solvers::NoBoilOff),
        Box::new(refinery_solvers::ConstantEnthalpy),
    )
}

/// **Gate 6: the tripwire fires.** The silent `.max(0.0)` is gone: a solver that
/// delivers mass a holdup did not have is an `Err` naming the holdup, whether
/// the tank was wet (allowance: rounding only) or starved (allowance: its OWN
/// recorded residual). The last arm is the control that the allowance is
/// really the recorded residual: the same over-draw, reported, is clamped.
#[test]
fn the_empty_holdup_tripwire_fires() {
    // A wet tank, over-drawn by 1 kg: far beyond rounding on ~85 t.
    let mut engine = bent_engine(1.0, None);
    let err = engine
        .tick()
        .expect_err("a wet tank drawn 1 kg past empty must fail");
    let text = err.to_string();
    assert!(
        text.contains("tank 'buffer_tank'") && text.contains("past empty"),
        "the error names the tank and what happened: {text}"
    );

    // A starved tank over-drawn by 1 kg against a residual of zero.
    let mut engine = bent_engine(1.0, Some(0.0));
    let err = engine
        .tick()
        .expect_err("a starved tank drawn past its residual must fail");
    assert!(err.to_string().contains("tank 'buffer_tank'"), "{err}");

    // The same 1 kg, reported by the solve as its own residual: clamped.
    let mut engine = bent_engine(1.0, Some(-1.0 / 1.0));
    engine
        .tick()
        .expect("an over-draw inside the solve's own recorded residual is clamped");
    assert_eq!(tank(&engine, "buffer_tank").mass.value(), 0.0);

    // Rounding alone is clamped on a wet tank: 1e-12 of ~85 t is far inside it.
    let mut engine = bent_engine(1.0e-12 * 8.5e4, None);
    engine
        .tick()
        .expect("a rounding-sized over-draw is clamped");
}

// --- gate 7 -----------------------------------------------------------------

/// **Gate 7: a recycle through a dry tank is refused by name** (row B34). The
/// demo with a minimum-flow line from the pump's discharge back to the buffer
/// tank. While the tank holds liquid the loop is broken by its inventory and
/// runs; the tick it dries, the loop has nothing left to break it.
#[test]
fn a_recycle_through_a_dry_tank_is_refused_by_name() {
    let src = swap(
        DEMO,
        "[[pipes]]\nname = \"discharge\"\nfrom = \"transfer_pump\"\nto = \"rundown\"",
        "[nodes.tee]\ntype = \"junction\"\n\n[nodes.recycle_valve]\ntype = \"valve\"\nkv = 5.0\nopening = 1.0\n\n\
         [[pipes]]\nname = \"recycle_out\"\nfrom = \"tee\"\nto = \"recycle_valve\"\nlength_m = 5.0\ndiameter_m = 0.05\n\n\
         [[pipes]]\nname = \"recycle_back\"\nfrom = \"recycle_valve\"\nto = \"buffer_tank\"\nlength_m = 5.0\ndiameter_m = 0.05\n\n\
         [[pipes]]\nname = \"to_tee\"\nfrom = \"transfer_pump\"\nto = \"tee\"\nlength_m = 1.0\ndiameter_m = 0.10\n\n\
         [[pipes]]\nname = \"discharge\"\nfrom = \"tee\"\nto = \"rundown\"",
    );
    let mut engine = build(&src);
    let mut ran = 0;
    let err = loop {
        match engine.tick() {
            Ok(()) => ran += 1,
            Err(e) => break e,
        }
        assert!(ran < 6_000, "the tank with a recycle must still run dry");
    };
    let text = err.to_string();
    assert!(
        ran > 100,
        "the recycle runs while the tank holds liquid ({ran} ticks)"
    );
    assert!(
        text.contains("recycle through a tank that has run dry (buffer_tank)"),
        "the refusal names the dry tank: {text}"
    );
}

// --- gate 8 -----------------------------------------------------------------

/// **Gate 8: both fidelities agree on a starved solve**, I5-style: one state,
/// two solvers. Taken at the start of the drying tick (the tank still holds
/// ~14 kg) and on a tick long dry. The starved SET must be the same and the
/// supply identical to the bit (one expression, `starved_supply`); flows and
/// pressures agree to the game solver's own tolerance.
#[test]
fn both_fidelities_agree_on_a_starved_solve() {
    let mut engine = build(DEMO);
    let dt = engine.dt();
    let tank_id = node(&engine, "buffer_tank");
    // Stop one tick short of drying, by replaying the demo to its known tick.
    let mut probe = build(DEMO);
    run_to_drying_tick(&mut probe, "buffer_tank", 6_000);
    let drying = probe.snapshot().tick;
    for checkpoint in [drying - 1, drying + 500] {
        while engine.snapshot().tick < checkpoint {
            tick(&mut engine);
        }
        let newton = NewtonFlowSolver::default()
            .solve(&engine.graph, &engine.slate, engine.node_states(), dt)
            .expect("newton solves the starved state");
        let simple = SimpleFlowSolver::default()
            .solve(&engine.graph, &engine.slate, engine.node_states(), dt)
            .expect("simple solves the starved state");
        assert_eq!(
            newton.starved.keys().collect::<Vec<_>>(),
            vec![&tank_id],
            "tick {checkpoint}: newton starves the buffer tank"
        );
        assert_eq!(
            simple.starved.keys().collect::<Vec<_>>(),
            vec![&tank_id],
            "tick {checkpoint}: simple starves the same tank"
        );
        assert_eq!(
            newton.starved[&tank_id].supply,
            simple.starved[&tank_id].supply
        );
        let mut worst_flow: f64 = 0.0;
        for (e, f) in &newton.edge_mass_flow {
            worst_flow = worst_flow.max((f - simple.edge_mass_flow[e]).abs() / f.abs().max(1.0));
        }
        let mut worst_pressure: f64 = 0.0;
        for (n, p) in &newton.node_pressure {
            worst_pressure =
                worst_pressure.max((p.value() - simple.node_pressure[n].value()).abs());
        }
        eprintln!("gate 8 tick {checkpoint}: worst flow {worst_flow:.3e} rel, worst pressure {worst_pressure:.3e} Pa");
        assert!(
            worst_flow < 1e-5,
            "tick {checkpoint}: flows differ by {worst_flow:.3e}"
        );
        assert!(
            worst_pressure < 1.0,
            "tick {checkpoint}: pressures differ by {worst_pressure:.3e} Pa"
        );
    }
}

// --- gate 9 -----------------------------------------------------------------

/// **Gate 9: a dry tank publishes the solve's own pressure** (fork 6, the user's
/// decision): below atmospheric — a pump pulling on nothing — with mass zero.
#[test]
fn a_dry_tank_publishes_the_solve_s_own_pressure() {
    let mut engine = build(DEMO);
    run_to_drying_tick(&mut engine, "buffer_tank", 6_000);
    for _ in 0..100 {
        tick(&mut engine);
    }
    let tank_id = node(&engine, "buffer_tank");
    let snapshot = engine.snapshot();
    let published = snapshot
        .nodes
        .iter()
        .find(|n| n.id == tank_id)
        .expect("the tank is published");
    let solved = solution(&engine).node_pressure[&tank_id];
    assert_eq!(
        Pascal(published.pressure_pa),
        solved,
        "the published pressure is the solve's"
    );
    assert!(
        published.pressure_pa < P_ATM.value(),
        "a dry tank under a running pump reads below atmospheric: {} Pa",
        published.pressure_pa
    );
    assert!(tank(&engine, "buffer_tank").mass.value() <= THERMAL_FLOOR_KG);
}
