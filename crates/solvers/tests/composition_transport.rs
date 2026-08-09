//! Composition transport at the ENGINE level (M3.1) — the material sibling of
//! `energy_invariants.rs`, which does the same job for enthalpy.
//!
//! The mixing RULE is unit-tested against hand-built flow maps in
//! `core::energy::tests::composition_mixing`, where a slate with mismatched `cp`
//! can be built directly. What only an engine can show is the parts composing:
//! solve → upwind transport → tank blending, compounding tick over tick.
//!
//!   I7. Per-component mass conservation: for any randomly generated valid
//!       network, the change in each COMPONENT's mass held in tanks equals that
//!       component's mass crossing the plant's reservoir boundary, per tick.
//!
//! WHAT I7 ACTUALLY CATCHES, and why it is not I1 restated. I1 already pins
//! TOTAL mass, and total mass is the sum of these — so anything I7 catches that
//! I1 does not must move mass BETWEEN components while leaving the sum alone.
//! That is exactly the failure mode this milestone's mixing rule invites:
//! weighting a blend by `ṁ·cp` rather than `ṁ` redistributes the fractions and
//! conserves the total to the last bit, passing I1 and every energy balance.
//! Splitting the balance per component is what makes that visible, and it is why
//! the slate below has two cuts with a 4:1 `cp` ratio — on a one-component slate
//! I7 is arithmetically identical to I1 and proves nothing.

use proptest::prelude::*;
use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::engine::{Engine, EngineConfig};
use refinery_core::graph::{Node, NodeKind, Pipe, PlantGraph, TankState};
use refinery_core::units::*;
use refinery_solvers::{ConstantThermo, NewtonFlowSolver, NoReactions};

const DT: Seconds = Seconds(0.1);

/// Two cuts differing in `cp` by 4:1 and in density by a little. Written out
/// here rather than read from any shared fixture: the references below
/// hand-calculate against these numbers, so they must be visible at the point
/// of use.
fn two_cut_slate() -> Slate {
    Slate::new(vec![
        PseudoComponent {
            name: "light".into(),
            tb: Kelvin(338.15),
            molar_mass: KgPerMol(0.1),
            density: Some(KgPerM3(700.0)),
            phase: Phase::Liquid,
            cp: JPerKgK(1000.0),
        },
        PseudoComponent {
            name: "heavy".into(),
            tb: Kelvin(613.15),
            molar_mass: KgPerMol(0.4),
            density: Some(KgPerM3(900.0)),
            phase: Phase::Liquid,
            cp: JPerKgK(4000.0),
        },
    ])
    .expect("a two-component slate is valid")
}

fn engine(graph: PlantGraph) -> Engine {
    Engine::new(
        graph,
        two_cut_slate(),
        EngineConfig { dt: DT },
        Box::new(NewtonFlowSolver::default()),
        Box::new(ConstantThermo),
        Box::new(NoReactions),
    )
}

fn node(name: &str, kind: NodeKind) -> Node {
    Node {
        name: name.into(),
        kind,
        heat_input: Watt::ZERO,
    }
}

fn composition(weights: &[f64]) -> Composition {
    Composition::from_weights(weights).expect("test weights must be a valid composition")
}

/// Isothermal by default: most of this file is about WHAT flows, not how hot it
/// is, and a temperature gradient would let a thermal bug surface here rather
/// than in the file that owns it. `hot_source` is the deliberate exception.
fn source(name: &str, pressure_pa: f64, weights: &[f64]) -> Node {
    hot_source(name, pressure_pa, weights, T_AMBIENT)
}

fn hot_source(name: &str, pressure_pa: f64, weights: &[f64], temperature: Kelvin) -> Node {
    node(
        name,
        NodeKind::Source {
            pressure: Pascal(pressure_pa),
            temperature,
            composition: composition(weights),
        },
    )
}

fn sink(name: &str, pressure_pa: f64, weights: &[f64]) -> Node {
    node(
        name,
        NodeKind::Sink {
            pressure: Pascal(pressure_pa),
            temperature: T_AMBIENT,
            composition: composition(weights),
        },
    )
}

fn tank_node(name: &str, mass_kg: f64, weights: &[f64]) -> Node {
    node(
        name,
        NodeKind::Tank(TankState {
            area: SquareMeter(10.0),
            height: Meter(20.0),
            mass: Kg(mass_kg),
            temperature: T_AMBIENT,
            composition: composition(weights),
            ambient_ua: WattPerKelvin::ZERO,
        }),
    )
}

fn pipe(name: &str, length_m: f64, diameter_m: f64) -> Pipe {
    Pipe {
        name: name.into(),
        length: Meter(length_m),
        diameter: Meter(diameter_m),
        friction_factor: 0.02,
        elevation_change: Meter(0.0),
        leak_area: SquareMeter::ZERO,
        ambient_ua: WattPerKelvin::ZERO,
        stream: refinery_core::stream::Stream::stagnant(2, T_AMBIENT, P_ATM),
    }
}

fn tank_of<'a>(engine: &'a Engine, name: &str) -> &'a TankState {
    let id = engine.graph.find_node(name).expect("node must exist");
    match &engine.graph.node(id).kind {
        NodeKind::Tank(t) => t,
        other => panic!("'{name}' is not a tank: {other:?}"),
    }
}

fn flow_through(engine: &Engine, pipe_name: &str) -> f64 {
    engine
        .snapshot()
        .edges
        .into_iter()
        .find(|e| e.name == pipe_name)
        .expect("snapshot must include every edge")
        .stream
        .mass_flow
        .value()
}

/// The friction power a pipe put into its own stream [W] (M5.1).
fn dissipation_in(engine: &Engine, pipe_name: &str) -> f64 {
    engine
        .snapshot()
        .edges
        .into_iter()
        .find(|e| e.name == pipe_name)
        .expect("snapshot must include every edge")
        .dissipation_w
}

// ---------------------------------------------------------------------------
// Reference — the tank blend, predicted from measured MASS RATIOS.
// ---------------------------------------------------------------------------

/// REFERENCE — a tank fed two different cuts ends the tick at the composition
/// its arriving MASSES imply.
///
/// The expected number is built from quantities this test measures rather than
/// from the blend rule under test: the two leg flows and the tank's own
/// start-of-tick inventory. Over one tick each leg delivers `ṁ·dt` kilograms
/// exactly, so per-component mass balance on the vessel gives
///
/// ```text
/// f_heavy = (m_heavy_in) / (m_start + m_light_in + m_heavy_in)
/// ```
///
/// with `m_start` pure light. That is arithmetic on masses — nothing here calls
/// `Composition::blend`, `mix_compositions`, or `mixture_cp`, so a bug in the
/// blend cannot appear on both sides of the comparison. What the test borrows
/// from the code is the FLOWS, which are I1's business and independently gated.
///
/// THE MUTATION THIS EXISTS FOR: blending by `ṁ·cp` instead of `ṁ`. The heavy
/// cut's cp is 4× the light one's, so that mutation over-weights the heavy leg
/// fourfold while leaving the tank's total mass — and therefore I1 — untouched.
///
/// Both feeds sit at the same pressure and their legs are identical, so the two
/// arrive at comparable rates: a blend dominated by one leg would be insensitive
/// to how the other is weighted, and this test would stop discriminating.
#[test]
fn a_tank_blends_its_two_feeds_by_arriving_mass() {
    let mut graph = PlantGraph::new();
    let light = graph.add_node(source("light_feed", 5.0e5, &[1.0, 0.0]));
    let heavy = graph.add_node(source("heavy_feed", 5.0e5, &[0.0, 1.0]));
    let vessel = graph.add_node(tank_node("vessel", 1_000.0, &[1.0, 0.0]));
    graph.add_pipe(light, vessel, pipe("light_leg", 12.0, 0.12));
    graph.add_pipe(heavy, vessel, pipe("heavy_leg", 12.0, 0.12));

    let mut engine = engine(graph);
    let mass_start = tank_of(&engine, "vessel").mass.value();
    assert_eq!(mass_start, 1_000.0, "the datum this reference is built on");

    engine.tick().expect("one tick must converge");

    let light_in = flow_through(&engine, "light_leg") * DT.value();
    let heavy_in = flow_through(&engine, "heavy_leg") * DT.value();
    assert!(
        light_in > 0.0 && heavy_in > 0.0,
        "both feeds must actually deliver mass, got {light_in} and {heavy_in} kg"
    );
    // Guard the guard: with one leg starved the weighting rule would barely
    // move the answer and this reference would pass under the mutation.
    let ratio = light_in / heavy_in;
    assert!(
        (0.5..=2.0).contains(&ratio),
        "the two legs must deliver comparable mass for this test to discriminate, \
         got a ratio of {ratio}"
    );

    let expected_heavy = heavy_in / (mass_start + light_in + heavy_in);
    let fractions = tank_of(&engine, "vessel").composition.fractions();
    assert!(
        (fractions[1] - expected_heavy).abs() < 1e-12,
        "the vessel must hold {expected_heavy} heavy by mass, got {}. The \
         capacity-rate weighting this guards against would give roughly {}.",
        fractions[1],
        4.0 * heavy_in / (light_in + 4.0 * heavy_in),
    );
}

/// A tank draining loses MASS but not its identity: an outflow leaves at the
/// vessel's own composition, so the fractions must not move at all.
///
/// Pinned because the blend is the natural place to net inflow against outflow,
/// and doing so would quietly re-weight a tank every tick it drains — a drift
/// that conserves total mass, passes I1, and shows up nowhere else.
#[test]
fn a_draining_tank_holds_its_composition() {
    let mut graph = PlantGraph::new();
    let vessel = graph.add_node(tank_node("vessel", 50_000.0, &[0.3, 0.7]));
    let drain = graph.add_node(sink("drain", 1.0e5, &[1.0, 0.0]));
    graph.add_pipe(vessel, drain, pipe("drain_line", 15.0, 0.15));

    let mut engine = engine(graph);
    let before = tank_of(&engine, "vessel").composition.clone();
    for _ in 0..50 {
        engine.tick().expect("draining must converge");
    }
    let after = tank_of(&engine, "vessel");

    assert!(
        after.mass.value() < 50_000.0,
        "the tank must actually drain, or this proves nothing"
    );
    assert_eq!(
        after.composition.fractions(),
        before.fractions(),
        "an outflow removes mass at the tank's OWN composition, which cannot \
         change the fractions"
    );
}

/// A sink driven backwards supplies its OWN composition to the plant.
///
/// The composition twin of `energy_invariants.rs`'s
/// `a_back_fed_sink_supplies_its_own_temperature`, and the reason
/// `NodeKind::Sink` carries a composition at all: an infinite reservoir the
/// network pushes fluid out of has to have something to push.
///
/// Deterministic on purpose. I7's generator does reach reverse flow through a
/// sink, but only for part of its pressure range — leaving the one field that
/// exists for this case covered by chance. This plant reverses every run: the
/// sink sits at 9 bar against a 1 bar feed.
#[test]
fn a_back_fed_sink_supplies_its_own_composition() {
    let mut graph = PlantGraph::new();
    let feed = graph.add_node(source("feed", 1.0e5, &[1.0, 0.0])); // pure light
    let tee = graph.add_node(node("tee", NodeKind::Junction));
    let back = graph.add_node(sink("back", 9.0e5, &[0.0, 1.0])); // pure heavy
    graph.add_pipe(feed, tee, pipe("inlet", 10.0, 0.1));
    graph.add_pipe(tee, back, pipe("outlet", 10.0, 0.1));

    let mut engine = engine(graph);
    engine.tick().expect("tick must converge");

    // Non-vacuity first: the premise is that flow actually reversed.
    let outlet_flow = flow_through(&engine, "outlet");
    assert!(
        outlet_flow < -1.0,
        "the 9 bar sink must drive flow backwards up the outlet, got {outlet_flow} kg/s"
    );

    // The `inlet` pipe is drawn feed→tee but flows tee→feed, so its upwind is
    // the tee — which is itself fed by the back-flowing sink. A transport model
    // reading the edge's declared direction reports pure light here, and the
    // heavy the sink actually supplied never enters the plant.
    let inlet = engine
        .snapshot()
        .edges
        .into_iter()
        .find(|e| e.name == "inlet")
        .expect("inlet must exist");
    let fractions = inlet.stream.composition.fractions().to_vec();
    assert!(
        (fractions[1] - 1.0).abs() < 1e-12,
        "the back-fed inlet stream must carry the sink's pure heavy, not the pure \
         light of the node its arrow points away from; got {fractions:?}"
    );
}

// ---------------------------------------------------------------------------
// Reference — tank energy with a composition-dependent heat capacity.
// ---------------------------------------------------------------------------

/// REFERENCE — a tank whose contents CHANGE COMPOSITION while heating ends the
/// tick at the temperature an energy balance on the new mixture predicts.
///
/// This is the one case where composition and temperature are coupled, and it
/// is the only place the tank's two heat capacities can be told apart. A tank
/// of 1000 kg pure light (cp 1000) at ambient is fed pure heavy (cp 4000) at
/// 400 K. Over one tick `Δm = ṁ·dt` kilograms arrive, and:
///
/// ```text
/// E_old = 1000·cp_light·(T_amb − T_REF)              [J above the datum]
/// E_in  = Δm·cp_heavy·(400 − T_REF)                  [J carried in]
/// m_new = 1000 + Δm
/// cp_new = (1000·cp_light + Δm·cp_heavy) / m_new     [additive heat capacity]
/// T_new = T_REF + (E_old + E_in) / (m_new·cp_new)
/// ```
///
/// Every term is written from the slate's numbers and the measured `Δm`; the
/// only thing borrowed from the engine is the flow, which is I1's business.
/// Note `cp_new` is derived here from ADDITIVITY (`m·cp = Σ m_c·cp_c`), not by
/// calling `mixture_cp` — so the mixing rule appears on one side only.
///
/// THE MUTATION THIS EXISTS FOR: computing the final temperature with the
/// tank's START-of-tick cp. That is bit-identical on a one-component slate and
/// invisible in every isothermal multi-component test, which is precisely why
/// this case had to be written rather than assumed — it is the only gate in the
/// workspace that reads a heat capacity that MOVED.
#[test]
fn a_tank_changing_composition_while_heating_lands_on_its_new_heat_capacity() {
    const CP_LIGHT: f64 = 1000.0;
    const CP_HEAVY: f64 = 4000.0;
    const FEED_T: f64 = 400.0;
    const T_REF_K: f64 = 273.15;

    let mut graph = PlantGraph::new();
    let feed = graph.add_node(hot_source("feed", 5.0e5, &[0.0, 1.0], Kelvin(FEED_T)));
    let vessel = graph.add_node(tank_node("vessel", 1_000.0, &[1.0, 0.0]));
    graph.add_pipe(feed, vessel, pipe("fill", 12.0, 0.12));

    let mut engine = engine(graph);
    engine.tick().expect("one tick must converge");

    let arrived = flow_through(&engine, "fill") * DT.value();
    assert!(
        arrived > 0.0,
        "the feed must deliver mass, got {arrived} kg"
    );

    let energy_old = 1_000.0 * CP_LIGHT * (T_AMBIENT.value() - T_REF_K);
    // The feed's own enthalpy, PLUS the work the fill line dissipated into it on
    // the way (M5.1). Written as an explicit additive term rather than folded
    // into an "arriving temperature" read back from the engine: the tank gains
    // `ṁ·h(T_feed)` from the reservoir and `Φ` from the pipe, and stating them
    // separately keeps the cp claim below — which is what this test is for —
    // exact instead of routed through the transform it does not mean to test.
    let friction_in = dissipation_in(&engine, "fill") * DT.value();
    assert!(
        friction_in > 0.0,
        "the fill line must dissipate something for this term to be under test"
    );
    let energy_in = arrived * CP_HEAVY * (FEED_T - T_REF_K) + friction_in;
    let mass_new = 1_000.0 + arrived;
    let capacity_new = 1_000.0 * CP_LIGHT + arrived * CP_HEAVY;
    let expected = T_REF_K + (energy_old + energy_in) / capacity_new;

    let tank = tank_of(&engine, "vessel");
    assert!(
        (tank.mass.value() - mass_new).abs() < 1e-9,
        "inventory must be {mass_new} kg, got {}",
        tank.mass.value()
    );
    assert!(
        (tank.temperature.value() - expected).abs() < 1e-9,
        "the vessel must land at {expected} K on its NEW heat capacity, got {}. \
         Holding the start-of-tick cp gives {} K.",
        tank.temperature.value(),
        T_REF_K + (energy_old + energy_in) / (mass_new * CP_LIGHT),
    );
    // Guard the guard: the two answers must be far apart, or this cannot
    // discriminate. With a 4:1 cp ratio they are, as long as real mass arrived.
    let stale = T_REF_K + (energy_old + energy_in) / (mass_new * CP_LIGHT);
    assert!(
        (stale - expected).abs() > 1.0,
        "the stale-cp answer must differ by more than a kelvin for this reference \
         to bite, got {stale} vs {expected}"
    );
}

// ---------------------------------------------------------------------------
// I7 — per-component mass conservation on random networks.
// ---------------------------------------------------------------------------

/// `(supply pressures + heavy fractions, sink pressure, tank heavy fraction)`.
/// Lengths are fixed and indexed by position so any shrunk sample stays valid
/// (the discipline `invariants.rs` established).
type PlantInputs = (
    Vec<(f64, f64)>, // supplies: (pressure_pa, heavy mass fraction), 2..=4
    f64,             // sink pressure [Pa]
    f64,             // tank heavy mass fraction
);

fn plant_inputs_strategy() -> impl Strategy<Value = PlantInputs> {
    (
        prop::collection::vec((2.0e5..8.0e5f64, 0.0..1.0f64), 2..=4),
        // Above the tank's bottom pressure for part of this range, so reverse
        // flow out of the sink — and therefore the sink's own composition
        // entering the plant — is inside the generated space.
        1.0e5..3.0e5f64,
        0.0..1.0f64,
    )
}

/// N supplies of differing composition → a mixing junction → a tank → a sink.
///
/// Shaped to make I7 bite: the junction is a real multi-way mix (so interior
/// component flows have to cancel), the tank is the only inventory (so `Δm_c`
/// has one meaning), and every feed composition is independently random — a
/// plant fed one composition everywhere would satisfy any blending rule,
/// correct or not.
///
/// **I7 EXCLUDES reactors, by construction, and this generator is where that is
/// enforced.** It builds only Source/Junction/Tank/Sink — never a `Reactor` —
/// and the exclusion is deliberate, not incidental: a reactor is the one unit
/// that BREAKS per-component mass on purpose (it moves mass between components,
/// e.g. gasoil → gasoline + gas + coke), so a network containing one could not
/// satisfy this balance at all. A splitter-style "green by construction"
/// argument does not rescue it — there is no argument that makes a
/// mass-redistributing unit conserve per-component mass. The reaction's OWN
/// mass-neutrality (Σ products = 1) is guarded elsewhere: `reactor.rs`'s
/// `rows_are_renormalized_so_mass_is_conserved` and the reactor total-mass gate
/// in `scenarios/tests/reactor_reference.rs` (docs/DESIGN.md §5).
fn build_plant(inputs: &PlantInputs) -> PlantGraph {
    let (supplies, sink_p, tank_heavy) = inputs;
    let mut graph = PlantGraph::new();

    let tee = graph.add_node(node("tee", NodeKind::Junction));
    // Big enough that it cannot drain within the run: the mass update clamps at
    // zero, and a clamped tank has stopped conserving mass at all — it would
    // fail this balance for a reason that is nothing to do with composition.
    let tank = graph.add_node(tank_node(
        "vessel",
        100_000.0,
        &[1.0 - tank_heavy, *tank_heavy],
    ));
    let drain = graph.add_node(sink("drain", *sink_p, &[0.5, 0.5]));

    for (i, (pressure, heavy)) in supplies.iter().enumerate() {
        let s = graph.add_node(source(
            &format!("supply{i}"),
            *pressure,
            &[1.0 - heavy, *heavy],
        ));
        graph.add_pipe(s, tee, pipe(&format!("leg{i}"), 12.0, 0.12));
    }
    graph.add_pipe(tee, tank, pipe("fill", 15.0, 0.15));
    graph.add_pipe(tank, drain, pipe("drain_line", 15.0, 0.15));
    graph
}

/// Mass of each component held in tanks [kg].
fn tank_component_mass(engine: &Engine) -> Vec<f64> {
    let mut totals = vec![0.0; engine.slate.len()];
    for id in engine.graph.node_ids() {
        if let NodeKind::Tank(t) = &engine.graph.node(id).kind {
            for (total, fraction) in totals.iter_mut().zip(t.composition.fractions()) {
                *total += t.mass.value() * fraction;
            }
        }
    }
    totals
}

/// Net mass rate of each component entering the plant across its reservoir
/// boundary [kg/s].
///
/// Computed ONLY from reservoir-incident edges, never from the tank's own
/// flows. Routing the accounting around the interior is the whole point: it is
/// what forces the junction's per-component balance to cancel for the books to
/// close, exactly as `boundary_power` does for enthalpy in I6.
fn boundary_component_rate(engine: &Engine) -> Vec<f64> {
    let mut rates = vec![0.0; engine.slate.len()];
    for id in engine.graph.node_ids() {
        let node = engine.graph.node(id);
        if !matches!(
            node.kind,
            NodeKind::Source { .. } | NodeKind::Sink { .. } | NodeKind::Atmosphere
        ) {
            continue;
        }
        for (edge, _other, incoming) in engine.graph.incident(id) {
            let stream = &engine.graph.pipe(edge).stream;
            let flow = stream.mass_flow.value();
            let into_reservoir = if incoming { flow } else { -flow };
            for (rate, fraction) in rates.iter_mut().zip(stream.composition.fractions()) {
                // Into the reservoir is out of the plant, hence the minus.
                *rate -= into_reservoir * fraction;
            }
        }
    }
    rates
}

/// Absolute, in KILOGRAMS, and for the reason the M1 acceptance gate settled
/// on: the quantity being compared is a mass, its magnitude is set by the
/// plant's flow rates rather than by anything the test controls, and a relative
/// tolerance would silently tighten or loosen with the scenario. 1e-6 kg over a
/// 0.1 s tick is far below the flow solver's own convergence, which is the real
/// floor here — per-component balance can never be tighter than the total mass
/// balance it partitions.
const COMPONENT_MASS_TOLERANCE_KG: f64 = 1.0e-6;

proptest! {
    #![proptest_config(ProptestConfig::with_cases(48))]

    #[test]
    fn every_component_is_conserved(inputs in plant_inputs_strategy()) {
        let mut engine = engine(build_plant(&inputs));
        for _ in 0..20 {
            let before = tank_component_mass(&engine);
            if engine.tick().is_err() {
                break; // divergence is legal (I3); nothing left to check
            }
            let after = tank_component_mass(&engine);
            let rates = boundary_component_rate(&engine);

            for (index, ((before, after), rate)) in
                before.iter().zip(&after).zip(&rates).enumerate()
            {
                let expected = rate * DT.value();
                let actual = after - before;
                prop_assert!(
                    (actual - expected).abs() < COMPONENT_MASS_TOLERANCE_KG,
                    "component {index}: tanks gained {actual} kg but {expected} kg \
                     crossed the boundary",
                );
            }
        }
    }

    /// Every fraction stays a fraction: non-negative and summing to one, on
    /// every node, every tick. A blend that leaks negative weight or loses
    /// normalization is still self-consistent enough to satisfy I7 — the mass
    /// it accounts for balances, it just is not a composition any more.
    #[test]
    fn compositions_stay_normalized(inputs in plant_inputs_strategy()) {
        let mut engine = engine(build_plant(&inputs));
        for _ in 0..20 {
            if engine.tick().is_err() {
                break;
            }
            for (name, tank) in engine.snapshot().tanks {
                let fractions = tank.composition.fractions();
                let sum: f64 = fractions.iter().sum();
                prop_assert!(
                    (sum - 1.0).abs() < 1e-9,
                    "tank '{name}' fractions sum to {sum}",
                );
                prop_assert!(
                    fractions.iter().all(|f| *f >= 0.0),
                    "tank '{name}' holds a negative fraction: {fractions:?}",
                );
            }
            for edge in engine.snapshot().edges {
                let sum: f64 = edge.stream.composition.fractions().iter().sum();
                prop_assert!(
                    (sum - 1.0).abs() < 1e-9,
                    "edge '{}' fractions sum to {sum}", edge.name,
                );
            }
        }
    }
}
