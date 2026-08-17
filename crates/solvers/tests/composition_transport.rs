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
use proptest::strategy::ValueTree;
use proptest::test_runner::TestRunner;
use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::engine::{Engine, EngineConfig};
use refinery_core::graph::{
    CascadeSpec, ColumnDraw, LeakRole, Node, NodeKind, Pipe, PlantGraph, TankState,
};
use refinery_core::traits::ThermoModel;
use refinery_core::units::*;
use refinery_solvers::{
    ConstantThermo, CutPointSplitter, MoleFractions, NewtonFlowSolver, NoReactions, StageCascade,
    TroutonThermo,
};

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
        Box::new(CutPointSplitter),
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
        leak: LeakRole::None,
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

// ---------------------------------------------------------------------------
// I7 — the CASCADE COLUMN arm (M7.4c).
//
// The generator above builds only Source/Junction/Tank/Sink, so through M7.4b no
// I-series invariant had ever reached a column of either fidelity (DESIGN §5,
// fork 4, and the claim was re-checked when the note was written). This arm is
// what closes that, and it is a real arm rather than a wider net:
//
//   * At the CUT-POINT fidelity I7 is green **by construction** — a splitter
//     conserves every component identically, `Σᵢ splitᵢ·w_ic = f_c` being an
//     algebraic identity of the weights — so a generated splitter column would
//     have no discriminating power at all. That is M3.2's own argument, and it
//     is why this arm is specifically a CASCADE.
//   * A converged cascade balances per component only to its convergence
//     tolerance, so the budget below is not I7's own — it admits the cascade's
//     gated residual as a second term. That is fork 4's "I7 stops being free".
//
// It is also where the flow split and the composition split are forced to come
// from ONE pass: `Engine::tick` writes `splitᵢ·ṁ_feed` onto each draw edge while
// `edge_composition_at` reads that draw's composition, and the two only telescope
// at the column if they describe the same separation.
//
// WHAT IT CATCHES, MEASURED. The first draft of this comment claimed the arm was
// "the first place in this workspace where I7 can fail for a reason that is
// neither a transport bug nor the flow solver" and had not checked. Three
// mutations were run to check it, and the answer needed all three:
//
//   1. Reversing the draw index on the FLOW write (`Engine::tick`) and reversing
//      it on the COMPOSITION read (`energy::column_draw_at`) are each caught here.
//      Both conserve TOTAL mass exactly, both leave every draw a valid
//      composition, and both make the two halves of one split describe different
//      draws — the failure M3.2's note names, and the one this arm is really for.
//   2. Dropping `residual <= COMPONENT_RESIDUAL_KG_PER_S` from `StageCascade`'s
//      convergence conjunction is caught by **nothing in the workspace**, this arm
//      included. So the residual criterion is a BACKSTOP rather than the binding
//      constraint: the profile and temperature criteria are strictly tighter on
//      every plant any test builds, and they stop the solve first.
//   3. Loosening those two AND dropping the residual — the only way to make a
//      cascade actually stop with an unbounded per-component residual — IS caught
//      here. So the claim above is true after all, but only because of a criterion
//      that never binds, which is a different sentence from the one first written.
//
// Recorded rather than tidied away: (2) alone reads as "the arm has no convergence
// power" and (3) alone reads as "the arm polices convergence", and neither is the
// whole thing.
// ---------------------------------------------------------------------------

/// The cascade arm's own slate, and it is not `two_cut_slate` for reasons that
/// are properties of the CASCADE rather than preferences.
///
/// * **The cuts are 100 K apart, not 275 K.** `two_cut_slate`'s pair have a
///   relative volatility in the hundreds at these temperatures, which separates so
///   completely that every draw is essentially pure and the per-component books
///   close on numbers that are 1 and 0. A ratio near 20 leaves both cuts present
///   in both draws, which is what makes the balance non-trivial.
/// * **The heat capacities are 1500 and 2500, not 1000 and 4000.** The
///   saturated-liquid window this arm has to feed its column inside of is
///   `ε·λ̄/c̄p` in MOLAR terms, so a large `M·cp` shrinks it: at `cp = 4000` and
///   `M = 0.4` the window is a third of a kelvin, and the feed line's own
///   frictional rise would eat it. At this slate it is 0.85–1.66 K across the
///   generated range, against a rise of about 0.06 K per bar of feed-line
///   pressure drop (measured by `measure_cascade_arm_reachability`).
///
/// The 5:2 heat-capacity ratio is still enough that the draws differ in `cp`,
/// which keeps the mixing rule visible; the 5:2 molar-mass ratio keeps the mass ⇄
/// mole boundary inside the cascade live rather than degenerate.
fn cascade_slate() -> Slate {
    Slate::new(vec![
        PseudoComponent {
            name: "light".into(),
            tb: Kelvin(353.15),
            molar_mass: KgPerMol(0.10),
            density: Some(KgPerM3(700.0)),
            phase: Phase::Liquid,
            cp: JPerKgK(1500.0),
        },
        PseudoComponent {
            name: "heavy".into(),
            tb: Kelvin(473.15),
            molar_mass: KgPerMol(0.25),
            density: Some(KgPerM3(900.0)),
            phase: Phase::Liquid,
            cp: JPerKgK(2500.0),
        },
    ])
    .expect("a two-component slate is valid")
}

/// The column's pinned operating pressure [Pa] for every plant in this arm.
///
/// Fixed rather than generated: it is what the feed's bubble point below is
/// computed at, and a column whose pressure moved would need that bubble point
/// recomputed for the same feed — which is a second thing to keep in step for no
/// extra coverage. What the generator varies instead is the SOURCE pressure, so
/// the feed rate and the feed line's dissipation both move.
const CASCADE_PRESSURE: Pascal = Pascal(1.5e5);

fn cascade_engine(graph: PlantGraph) -> Engine {
    Engine::new(
        graph,
        cascade_slate(),
        EngineConfig { dt: DT },
        Box::new(NewtonFlowSolver::default()),
        Box::new(TroutonThermo::new()),
        Box::new(NoReactions),
        Box::new(StageCascade::new()),
    )
}

/// The bubble point of a MASS composition at `pressure` [K], by bisection on
/// `Σ Kᵢ(T)·xᵢ = 1` over mole fractions.
///
/// **A second implementation on purpose**, the same way
/// `scenarios/tests/cascade_column.rs` carries one: `StageCascade`'s own
/// `bubble_point` is private, and this arm needs the number for a different job —
/// to BUILD a plant the cascade will accept, rather than to check one it
/// produced. Using only the published `ThermoModel::k_value` keeps the generator
/// from inheriting a fault in the solve it is meant to exercise.
///
/// `Σ Kᵢ·xᵢ` is monotone increasing in `T` for this model, so bisection converges
/// on the single root; the bracket holds both cuts of `cascade_slate` at every
/// pressure this arm uses.
fn cascade_bubble_point(slate: &Slate, mass: &Composition, pressure: Pascal) -> Kelvin {
    let thermo = TroutonThermo::new();
    let moles = MoleFractions::from_mass(mass, slate).expect("a generated feed is a valid mix");
    let sum_kx = |t: f64| -> f64 {
        moles
            .fractions()
            .iter()
            .enumerate()
            .map(|(c, x)| {
                x * thermo
                    .k_value(slate, c, Kelvin(t), pressure)
                    .expect("K at T > 0")
            })
            .sum()
    };
    let (mut low, mut high) = (200.0_f64, 900.0_f64);
    assert!(
        sum_kx(low) < 1.0 && sum_kx(high) > 1.0,
        "the bracket must straddle the bubble point of a generated feed"
    );
    for _ in 0..200 {
        let mid = 0.5 * (low + high);
        if sum_kx(mid) < 1.0 {
            low = mid;
        } else {
            high = mid;
        }
    }
    Kelvin(0.5 * (low + high))
}

/// `(source pressure, feed light fraction, draw ratio as a share of it,
/// reflux ratio, stages, feed stage as a position in 0..1)`.
///
/// Every field is a scalar and the derived integers are computed from fractions,
/// so any shrunk sample is still a valid plant — the discipline `invariants.rs`
/// established and this file already follows.
type CascadeInputs = (f64, f64, f64, f64, u32, f64);

fn cascade_inputs_strategy() -> impl Strategy<Value = CascadeInputs> {
    (
        // Above the column's 1.5 bar, and capped so the feed stays under the
        // 10 kg/s the tolerance derivation below rests on.
        1.65e5..2.40e5f64,
        // Both cuts genuinely present: a feed that is 99% one component would
        // make the per-component balance a statement about one number.
        0.2..0.8f64,
        // `D/F` as a SHARE of the light cut the feed carries. Written this way
        // rather than as an absolute ratio because the two are not independent:
        // ask a column for more distillate than the feed's light fraction and the
        // distillate must carry heavy, which at a 2.5:1 molar-mass ratio can push
        // an iterate past `Σ D_moles ≤ F_moles` and be refused. That refusal is
        // correct (DESIGN §5, correction 1) and is not what this arm is for.
        0.30..0.95f64,
        0.5..3.0f64,
        3u32..=8,
        0.0..1.0f64,
    )
}

/// Source → cascade column → two product tanks.
///
/// No junction and no sink: the column is the ONLY interior node, so the
/// boundary accounting has exactly one term and any per-component drift is the
/// column's. That is the opposite shaping decision from `build_plant` above,
/// where a multi-way tee is what forces interior cancellation — here the interior
/// is the unit under test and anything else would give it somewhere to hide.
///
/// **The feed temperature is not free, and it is the reason this generator is
/// harder to write than a generated tank.** Constant molar overflow admits a
/// saturated-liquid feed only, and `StageCascade` refuses anything more than 1%
/// of the feed off-phase (M7.4b). So the source sits on the bubble point of its
/// own composition at the COLUMN's pressure, computed per sample — a plant built
/// at a fixed temperature would be refused for most generated feeds, and would
/// look like a solver failure rather than a fixture fault.
fn build_cascade_plant(inputs: &CascadeInputs) -> PlantGraph {
    let (source_pressure, light, ratio_share, reflux, stages, feed_position) = *inputs;
    let slate = cascade_slate();
    let feed = composition(&[light, 1.0 - light]);
    let bubble = cascade_bubble_point(&slate, &feed, CASCADE_PRESSURE);

    let mut graph = PlantGraph::new();
    let source = graph.add_node(hot_source(
        "crude",
        source_pressure,
        &[light, 1.0 - light],
        bubble,
    ));
    // Big enough that neither can drain or overflow inside the run: a clamped
    // tank has stopped conserving mass and would fail this balance for a reason
    // that is nothing to do with the column.
    let top = graph.add_node(tank_node("distillate", 100_000.0, &[1.0, 0.0]));
    let bottom = graph.add_node(tank_node("bottoms", 100_000.0, &[0.0, 1.0]));

    let column = graph.add_node(node(
        "column",
        NodeKind::Column {
            pressure: CASCADE_PRESSURE,
            // The cut-point fidelity's ramp width, unread here. `ZERO` rather
            // than a number, so a reader cannot take it for a setting.
            smearing: Kelvin::ZERO,
            draws: vec![
                ColumnDraw::by_stage(top, 0, Some(light * ratio_share)),
                ColumnDraw::by_stage(bottom, stages, None),
            ],
            cascade: Some(CascadeSpec {
                stages,
                // `1..=stages`, from a position rather than a generated integer
                // so a shrunk sample stays inside the range whatever `stages`
                // shrinks to.
                feed_stage: (1 + (feed_position * stages as f64) as u32).min(stages),
                reflux_ratio: reflux,
            }),
        },
    ));

    graph.add_pipe(source, column, pipe("feed_line", 20.0, 0.05));
    graph.add_pipe(column, top, pipe("top_draw", 20.0, 0.05));
    graph.add_pipe(column, bottom, pipe("bottom_draw", 20.0, 0.05));
    graph
}

/// `StageCascade`'s own gated per-component residual [kg/s] — `1e-5`, which its
/// module derives from I7's `1e-6` kg over a 0.1 s tick. Written out here rather
/// than imported because it is a private constant, and because a reference test
/// reusing the code's own number on both sides proves nothing; if the two drift
/// apart, `measure_cascade_arm_headroom` is what says so.
const CASCADE_RESIDUAL_KG_PER_S: f64 = 1.0e-5;

/// I7's budget for a plant containing a cascade column [kg].
///
/// **Derived, not widened to fit.** Two independent contributions, and the arm
/// admits exactly their sum:
///
/// ```text
/// COMPONENT_MASS_TOLERANCE_KG                  1e-6 kg   the flow solver's own floor
/// CASCADE_RESIDUAL_KG_PER_S · dt               1e-6 kg   the cascade's gated residual
/// ```
///
/// The second term is what fork 4 means by "I7 stops being free". A splitter's
/// contribution to this line is identically zero; a cascade's is bounded by the
/// number its own convergence test refuses to exceed, and that number was chosen
/// to BE I7's — which makes the two exactly equal and this arm's budget exactly
/// twice the base one. `measure_cascade_arm_headroom` measures how much of it is
/// really used, so the equality above cannot quietly become an inequality.
const CASCADE_COMPONENT_MASS_TOLERANCE_KG: f64 =
    COMPONENT_MASS_TOLERANCE_KG + CASCADE_RESIDUAL_KG_PER_S * 0.1;

/// The worst per-component boundary imbalance a cascade plant shows over `ticks`
/// [kg], plus whether the column separated at all. `None` if the plant never
/// ran — a refused feed, a non-converged cascade, or a diverged solve.
///
/// Shared by the proptest and the two measurement harnesses, so the number the
/// budget is justified against is the number the budget is applied to.
fn worst_cascade_component_error(engine: &mut Engine, ticks: u32) -> Option<(f64, f64)> {
    let mut worst: Option<f64> = None;
    let mut separation = 0.0f64;
    for _ in 0..ticks {
        let before = tank_component_mass(engine);
        if engine.tick().is_err() {
            return worst.map(|w| (w, separation));
        }
        let after = tank_component_mass(engine);
        let rates = boundary_component_rate(engine);
        for ((before, after), rate) in before.iter().zip(&after).zip(&rates) {
            let error = ((after - before) - rate * DT.value()).abs();
            worst = Some(worst.map_or(error, |w: f64| w.max(error)));
        }
        // How far apart the two products actually are, in light mass fraction.
        // A column that returned its feed unchanged scores 0 here and would
        // satisfy any conservation law ever written.
        let top = edge_composition_of(engine, "top_draw")[0];
        let bottom = edge_composition_of(engine, "bottom_draw")[0];
        separation = separation.max(top - bottom);
    }
    worst.map(|w| (w, separation))
}

fn edge_composition_of(engine: &Engine, name: &str) -> Vec<f64> {
    engine
        .snapshot()
        .edges
        .into_iter()
        .find(|e| e.name == name)
        .expect("snapshot must include every edge")
        .stream
        .composition
        .fractions()
        .to_vec()
}

/// **The reachability count this arm is not believed without.**
///
/// `a-generated-arm-can-be-born-vacuous` records M5's gas arm reaching its own
/// feature in 0 of 185 samples while the proptest sat green, so a generated arm
/// owes a count before it owes anything else. This one has three ways to be born
/// vacuous and each is counted separately:
///
/// 1. **The plant never runs.** A generated feed off its bubble point, or a
///    specification the cascade cannot converge, is an `Err` every tick — and a
///    proptest treats a `break` on `Err` as "nothing to check", so an arm where
///    every sample failed would be perfectly green.
/// 2. **The column does not separate.** A cascade that handed back its feed would
///    satisfy per-component conservation exactly, for the same reason a splitter
///    does. The count therefore requires a real composition SPREAD between the two
///    products, not merely a tick that returned `Ok`.
/// 3. **The feed rate outruns the tolerance derivation.** The budget below rests
///    on the cascade's residual bound being an absolute kg/s, which it is — but
///    the arm should also stay in the regime where the OTHER convergence criteria
///    are not clamped, so the feed is checked to stay under 10 kg/s.
///
/// Measured: **60/60 plants ran, 60/60 separated**, the best by 0.95 of light
/// mass fraction, the worst feed rate 8.43 kg/s, and the worst feed-line rise
/// 0.062 K against a window of 0.85 K at its tightest. So the generator is not
/// building plants near a refusal, and the arm is not quietly empty.
///
/// It is a measurement and not a gate on the physics — `a-counter-is-not-a-gate`
/// — which is why the conservation proptest is a separate test. What this one
/// gates is the generator.
#[test]
fn measure_cascade_arm_reachability() {
    const SAMPLES: usize = 60;
    let mut runner = TestRunner::deterministic();
    let strategy = cascade_inputs_strategy();

    let mut ran = 0usize;
    let mut separated = 0usize;
    let mut worst_feed = 0.0f64;
    let mut worst_offset = 0.0f64;
    let mut best_separation = 0.0f64;
    for _ in 0..SAMPLES {
        let inputs = strategy
            .new_tree(&mut runner)
            .expect("strategy produces a value")
            .current();
        let mut engine = cascade_engine(build_cascade_plant(&inputs));
        // How far the feed line's own dissipation carries the feed off the bubble
        // point the source was placed on. It has to stay well inside the ±ε·λ̄/c̄p
        // window or the generator is building plants the model refuses.
        let bubble = cascade_bubble_point(
            &engine.slate,
            &composition(&[inputs.1, 1.0 - inputs.1]),
            CASCADE_PRESSURE,
        );
        if let Some((_, separation)) = worst_cascade_component_error(&mut engine, 5) {
            ran += 1;
            worst_feed = worst_feed.max(flow_through(&engine, "feed_line"));
            worst_offset = worst_offset
                .max((edge_temperature_of(&engine, "feed_line") - bubble.value()).abs());
            best_separation = best_separation.max(separation);
            if separation > 0.05 {
                separated += 1;
            }
        }
    }

    assert!(
        ran >= SAMPLES * 9 / 10,
        "only {ran}/{SAMPLES} generated cascade plants ran a single tick. A proptest treats a \
         refused tick as nothing-to-check, so an arm that cannot build an admissible plant is \
         green and vacuous."
    );
    assert!(
        separated >= SAMPLES * 3 / 4,
        "only {separated}/{SAMPLES} generated columns separated their feed by more than 5 \
         points of light mass fraction (best seen: {best_separation:.4}). Per-component mass \
         is conserved identically by a column that separates nothing, so this arm's \
         discriminating power is exactly the samples that do."
    );
    assert!(
        worst_feed < 10.0,
        "a generated plant fed its column {worst_feed} kg/s. The tolerance below is derived \
         from a residual bound in kg/s, and the arm is kept in the regime where the cascade's \
         other convergence criteria are unclamped — see StageCascade's `tolerance`."
    );
    assert!(
        worst_offset < 0.25,
        "the feed line carried a generated feed {worst_offset} K off the bubble point the \
         source was placed on. The saturated-liquid window is 0.85-1.66 K across this slate, \
         so a rise approaching it means the plants are being built near a refusal rather than \
         inside the model."
    );
}

fn edge_temperature_of(engine: &Engine, name: &str) -> f64 {
    engine
        .snapshot()
        .edges
        .into_iter()
        .find(|e| e.name == name)
        .expect("snapshot must include every edge")
        .stream
        .temperature
        .value()
}

/// The budget's justification, executable — `measure_energy_balance_headroom`'s
/// pattern, and for its reason: a budget nobody re-measures rots.
///
/// This one has a second job the energy version does not. Its budget is the SUM
/// of two terms, and if the cascade's contribution ever grew past the bound its
/// own convergence test enforces, the sum would still admit it — so the
/// measurement is what keeps the derivation honest rather than merely arithmetic.
///
/// **What the measurement says, and it is the interesting half.** The worst
/// observed imbalance is **3.4e-7 kg** — 17% of the budget, and the second term
/// of that budget is what admits it.
///
/// **The attribution is measured on THIS plant, and the first attempt at it was
/// wrong.** The obvious control is to run the same accounting on the column-free
/// tee plant and show the cascade's number is much larger. It is not: the tee
/// plant shows **1.7e-7 kg**, within a factor of two, because Newton stops at
/// `1e-8 + 1e-8·throughput` kg/s and that plant moves ~170 kg/s. Two different
/// mechanisms landing on the same order — so the comparison establishes nothing,
/// and the paragraph that used to be here claimed "two orders of magnitude above
/// the tee plant" from an inference that was never run.
///
/// What does settle it is a measurement on the plant itself: this generator
/// builds source, column and two tanks, and **every one of them is
/// pressure-anchored**, so the hydraulic solve has no free node and no unknowns.
/// The assertion below reads the solver's own reported residual back out of the
/// snapshot; it measures **exactly 0**, which leaves the cascade's convergence as
/// the only thing the 3.4e-7 kg can be. Fork 4's "I7 stops being free", from this
/// plant rather than from a comparison with another.
///
/// That assertion is a PREMISE CHECK and cannot fail today — a solve with no
/// unknowns has nothing to be residual about (`a-control-can-be-implied-by-its-
/// assertion`, stated rather than discovered later). What it is for is the edit
/// that puts a valve or a junction on one of these lines: the plant would still
/// look right, the budget's second term would silently start carrying the flow
/// solver's floor as well, and this line is what says so.
///
/// The trigger is consequently half the budget rather than the tenth
/// `measure_energy_balance_headroom` uses. The quantity being bounded there is
/// float noise inherited from a solver tolerance; here it IS a solver tolerance,
/// so a tenth would be asserting the cascade converges an order better than it
/// promises to.
#[test]
fn measure_cascade_arm_headroom() {
    const SAMPLES: usize = 40;
    /// Half the budget: observed 3.4e-7 kg against this 1e-6 kg, so ~3x of
    /// headroom on a number that is deterministic rather than noisy.
    const HEADROOM_TRIGGER: f64 = CASCADE_COMPONENT_MASS_TOLERANCE_KG / 2.0;

    let mut runner = TestRunner::deterministic();
    let strategy = cascade_inputs_strategy();
    let mut worst = 0.0f64;
    let mut checked = 0usize;
    for _ in 0..SAMPLES {
        let inputs = strategy
            .new_tree(&mut runner)
            .expect("strategy produces a value")
            .current();
        let mut engine = cascade_engine(build_cascade_plant(&inputs));
        if let Some((w, _)) = worst_cascade_component_error(&mut engine, 5) {
            worst = worst.max(w);
            checked += 1;
        }
    }

    assert!(checked > SAMPLES / 2, "only {checked}/{SAMPLES} plants ran");
    assert!(
        worst < HEADROOM_TRIGGER,
        "the per-component imbalance across a cascade column is {worst:e} kg, past half the \
         {CASCADE_COMPONENT_MASS_TOLERANCE_KG:e} budget. That budget is the flow solver's floor \
         plus the cascade's own gated residual, so find what grew before relaxing it — a \
         cascade stopping nearer its convergence criterion is a different finding from a \
         transport bug, and only one of them is benign."
    );

    // The attribution, on this plant. Every node here is pressure-anchored, so
    // the hydraulic solve has no unknowns and cannot be contributing the
    // imbalance measured above — which is what leaves the cascade's convergence
    // as the only candidate. Asserted rather than argued, because the FIRST
    // attempt at this attribution (compare against the tee plant) turned out to
    // be measuring two different mechanisms that happen to land on the same
    // order, and it read as confirmation.
    let mut runner = TestRunner::deterministic();
    let strategy = cascade_inputs_strategy();
    let mut worst_solver_residual = 0.0f64;
    for _ in 0..SAMPLES {
        let inputs = strategy
            .new_tree(&mut runner)
            .expect("strategy produces a value")
            .current();
        let mut engine = cascade_engine(build_cascade_plant(&inputs));
        for _ in 0..5 {
            if engine.tick().is_err() {
                break;
            }
            let solve = engine.snapshot().solver;
            assert!(solve.converged, "a generated cascade plant must solve");
            worst_solver_residual = worst_solver_residual.max(solve.residual.abs());
        }
    }
    assert!(
        worst_solver_residual * DT.value() < worst / 100.0,
        "the hydraulic solver's own residual on these plants is {worst_solver_residual:e} kg/s, \
         which over a tick is not negligible against the {worst:e} kg imbalance the budget \
         attributes to the CASCADE. This generator builds no free node, so a solver residual \
         here means the plant's shape changed — and the budget's second term would no longer \
         be measuring what it claims to."
    );
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(32))]

    /// I7 across a CASCADE column — the arm fork 4 says has real discriminating
    /// power, where a cut-point column has none.
    ///
    /// The books are kept the same way the tee plant's are: tank inventories on
    /// one side, the reservoir boundary on the other, and nothing read from the
    /// column itself. What has to cancel for them to close is the draw write
    /// (`splitᵢ·ṁ_feed`, from `Engine::tick`) against the draw compositions
    /// (`edge_composition_at`, from the sweep) — two separate readers of one
    /// `Separation`, which only telescope if they describe the same pass.
    #[test]
    fn every_component_survives_a_cascade_column(inputs in cascade_inputs_strategy()) {
        let mut engine = cascade_engine(build_cascade_plant(&inputs));
        if let Some((worst, separation)) = worst_cascade_component_error(&mut engine, 10) {
            prop_assert!(
                worst < CASCADE_COMPONENT_MASS_TOLERANCE_KG,
                "per-component mass drifted {worst:e} kg across the column against a \
                 {CASCADE_COMPONENT_MASS_TOLERANCE_KG:e} kg budget (separation seen: \
                 {separation:.4})",
            );
        }
    }
}
