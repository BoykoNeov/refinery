//! M14.1: what the RECEIVING end of a boil-off vent must do (docs/DESIGN.md
//! §16 fork 3), on graphs no scenario can produce.
//!
//! Two properties live here rather than beside the shipped demo's gates,
//! because the shipped demo cannot reach either state.
//!
//! - **A vent stored the other way round.** `build_boiloff_vents` always writes
//!   emitter → destination, so every vent in the corpus points outward and a
//!   receiver scoped by *incidence direction* would be right on all eighteen
//!   plants. §16's mutation 5 predicts exactly that — "inert on the demo, which
//!   is the point — the gate for it is a fixture with a vent stored the other
//!   way round, or nothing defends it". This is that fixture.
//! - **A cycle reaching the engine.** The loader refuses one at load, so the
//!   `Engine::tick` backstop that the refusal shares a function with is never
//!   exercised from a file.

use refinery_core::components::{Composition, Slate};
use refinery_core::engine::EngineConfig;
use refinery_core::graph::{LeakRole, Node, NodeId, NodeKind, Pipe, PlantGraph, TankState};
use refinery_core::units::{
    JPerKgK, Kelvin, Kg, KgPerM3, KgPerMol, Meter, Seconds, SquareMeter, Watt, WattPerKelvin,
    P_ATM, T_AMBIENT,
};
use refinery_core::Engine;
use refinery_solvers::{
    CutPointSplitter, FlashBoilOff, NewtonFlowSolver, NoReactions, TroutonThermo,
};

const DT: Seconds = Seconds(0.1);
const TICKS: u32 = 60;

/// The same two-cut naphtha slate `energy_invariants.rs` carries, for its
/// reason: a single-component holdup makes `y = K·x` and `x` the same vector,
/// so a flash cannot be told from a decrement and a drum cannot be told from a
/// bucket.
fn naphtha_slate() -> Slate {
    Slate::new(vec![
        refinery_core::components::PseudoComponent {
            name: "light".into(),
            tb: Kelvin(353.15),
            molar_mass: KgPerMol(0.100),
            density: Some(KgPerM3(680.0)),
            cp: JPerKgK(2200.0),
            cp_shape: None,
            phase: refinery_core::components::Phase::Liquid,
        },
        refinery_core::components::PseudoComponent {
            name: "heavy".into(),
            tb: Kelvin(423.15),
            molar_mass: KgPerMol(0.130),
            density: Some(KgPerM3(750.0)),
            cp: JPerKgK(2100.0),
            cp_shape: None,
            phase: refinery_core::components::Phase::Liquid,
        },
    ])
    .expect("a two-cut slate")
}

fn boiling_engine(graph: PlantGraph) -> Engine {
    Engine::new(
        graph,
        naphtha_slate(),
        EngineConfig { dt: DT },
        Box::new(NewtonFlowSolver::default()),
        Box::new(TroutonThermo::new()),
        Box::new(NoReactions),
        Box::new(CutPointSplitter),
        Box::new(FlashBoilOff),
        Box::new(refinery_solvers::ConstantEnthalpy),
    )
}

fn holdup(name: &str, mass: f64, temperature: f64, light: f64, ua: f64) -> Node {
    Node {
        name: name.into(),
        kind: NodeKind::Tank(TankState {
            area: SquareMeter(10.0),
            height: Meter(20.0),
            mass: Kg(mass),
            temperature: Kelvin(temperature),
            composition: Composition::from_weights(&[light, 1.0 - light])
                .expect("two fractions in (0, 1)"),
            ambient_ua: WattPerKelvin(ua),
        }),
        heat_input: Watt::ZERO,
    }
}

fn vent(name: &str, components: usize, emitter: NodeId) -> Pipe {
    Pipe {
        name: name.into(),
        length: Meter(0.0),
        diameter: Meter(0.0),
        friction_factor: 0.0,
        elevation_change: Meter(0.0),
        ambient_ua: WattPerKelvin::ZERO,
        leak: LeakRole::BoilOffVent { emitter },
        stream: refinery_core::stream::Stream::stagnant(components, T_AMBIENT, P_ATM),
    }
}

/// A hot tank venting into a cooled drum, with the tank → drum vent stored in
/// the direction `stored_forward` asks for. Nothing feeds the tank, so it boils
/// its superheat off over the first few ticks and the drum receives it.
fn two_tank_plant(stored_forward: bool) -> PlantGraph {
    let slate = naphtha_slate();
    let mut graph = PlantGraph::new();
    let tank = graph.add_node(holdup("holdup", 5_000.0, 430.0, 0.5, 0.0));
    let drum = graph.add_node(holdup("drum", 500.0, 300.0, 0.5, 5.0e4));
    let sky = graph.add_node(Node {
        name: "sky".into(),
        kind: NodeKind::Atmosphere,
        heat_input: Watt::ZERO,
    });
    let pipe = vent("holdup__boiloff_vent", slate.len(), tank);
    if stored_forward {
        graph.add_pipe(tank, drum, pipe);
    } else {
        graph.add_pipe(drum, tank, pipe);
    }
    graph.add_pipe(drum, sky, vent("drum__boiloff_vent", slate.len(), drum));
    graph
}

fn final_drum_state(graph: PlantGraph) -> (f64, f64, Vec<f64>, f64) {
    let mut engine = boiling_engine(graph);
    let mut vented = 0.0;
    for t in 1..=TICKS {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        let eid = engine
            .graph
            .edge_ids()
            .find(|e| engine.graph.pipe(*e).name == "holdup__boiloff_vent")
            .expect("the vent");
        vented += engine.graph.pipe(eid).stream.mass_flow.value().abs() * DT.value();
    }
    let drum = engine
        .graph
        .node_ids()
        .find(|id| engine.graph.node(*id).name == "drum")
        .expect("the drum");
    match &engine.graph.node(drum).kind {
        NodeKind::Tank(t) => (
            t.mass.value(),
            t.temperature.value(),
            t.composition.fractions().to_vec(),
            vented,
        ),
        other => panic!("'{other:?}' is not a tank"),
    }
}

/// **The gate for §16's mutation 5.** Ownership is stored on the role, so which
/// endpoint emits a vent does not depend on which way the edge happens to point
/// — and the two graphs below differ in nothing else. A receiver that decided
/// ownership from the incidence direction would swap the two holdups' roles in
/// the reversed graph: the drum would look like the emitter and the hot tank
/// like the receiver, and the drum would neither fill nor warm.
///
/// **Two controls first**, because "the two runs agree" is satisfied by a plant
/// where nothing happens at all: the vent must have carried mass, and the drum
/// must have gained some.
#[test]
fn a_vent_stored_the_other_way_round_moves_the_same_mass() {
    let forward = final_drum_state(two_tank_plant(true));
    let reversed = final_drum_state(two_tank_plant(false));

    assert!(
        forward.3 > 1.0,
        "the hot tank must actually boil, or this test compares two idle plants: \
         {:.6e} kg vented",
        forward.3
    );
    assert!(
        forward.0 > 500.0 + 1.0,
        "and the drum must actually fill: {:.4} kg from 500.0",
        forward.0
    );

    assert_eq!(
        forward.0, reversed.0,
        "the drum's inventory must not depend on which way the vent edge is stored"
    );
    assert_eq!(forward.1, reversed.1, "nor its temperature");
    assert_eq!(forward.2, reversed.2, "nor its composition");
    assert_eq!(forward.3, reversed.3, "nor what the vent carried");
}

/// The engine's own backstop for the fork-5 cycle refusal, on the graph a
/// scenario cannot build because the loader refuses it first.
///
/// The refusal is `PlantGraph::holdup_evaluation_order`'s, and the engine calls
/// it every tick to decide which holdup to update before which — so this is not
/// a rule about cycles bolted on beside the physics, it is the tick loop saying
/// it has no order to run in.
#[test]
fn a_cycle_of_vents_fails_the_tick_it_cannot_order() {
    let slate = naphtha_slate();
    let mut graph = PlantGraph::new();
    let a = graph.add_node(holdup("a", 5_000.0, 430.0, 0.5, 0.0));
    let b = graph.add_node(holdup("b", 5_000.0, 430.0, 0.5, 0.0));
    graph.add_pipe(a, b, vent("a__boiloff_vent", slate.len(), a));
    graph.add_pipe(b, a, vent("b__boiloff_vent", slate.len(), b));

    let mut engine = boiling_engine(graph);
    let message = engine
        .tick()
        .expect_err("a cycle of vents has no evaluation order")
        .to_string();
    assert!(message.contains("form a cycle"), "got {message}");
    // Both members, in the order the sort could not produce them in — not a
    // bare `contains('a')`, which almost any sentence satisfies.
    assert!(
        message.contains("form a cycle: a, b"),
        "both members named: got {message}"
    );
}

/// **The same refusal at DEPTH 2** (M15.1, docs/DESIGN.md §17 gate 4): three
/// holdups, `a → b → c → a`, which no pairwise check can see — every adjacent
/// pair points the right way and only the closure is impossible.
///
/// It is here as well as in the scenario gate because the two callers of
/// `holdup_evaluation_order` are independently defended: M14.1 measured that
/// removing the loader's call fires the scenario gate and leaves this one
/// green, which is the right answer and only visible with both written.
#[test]
fn a_three_holdup_cycle_fails_the_tick_it_cannot_order() {
    let slate = naphtha_slate();
    let mut graph = PlantGraph::new();
    let a = graph.add_node(holdup("a", 5_000.0, 430.0, 0.5, 0.0));
    let b = graph.add_node(holdup("b", 5_000.0, 430.0, 0.5, 0.0));
    let c = graph.add_node(holdup("c", 5_000.0, 430.0, 0.5, 0.0));
    graph.add_pipe(a, b, vent("a__boiloff_vent", slate.len(), a));
    graph.add_pipe(b, c, vent("b__boiloff_vent", slate.len(), b));
    graph.add_pipe(c, a, vent("c__boiloff_vent", slate.len(), c));

    let mut engine = boiling_engine(graph);
    let message = engine
        .tick()
        .expect_err("a three-holdup cycle has no evaluation order")
        .to_string();
    assert!(
        message.contains("form a cycle: a, b, c"),
        "all three members must be named: got {message}"
    );

    // The control, and it is what says the refusal is about the CLOSURE rather
    // than about three holdups in a row: the same chain with the last vent sent
    // to the sky instead runs, and every holdup gets an order.
    let mut graph = PlantGraph::new();
    let a = graph.add_node(holdup("a", 5_000.0, 430.0, 0.5, 0.0));
    let b = graph.add_node(holdup("b", 5_000.0, 380.0, 0.5, 5.0e4));
    let c = graph.add_node(holdup("c", 5_000.0, 300.0, 0.5, 5.0e4));
    let sky = graph.add_node(Node {
        name: "sky".into(),
        kind: NodeKind::Atmosphere,
        heat_input: Watt::ZERO,
    });
    graph.add_pipe(a, b, vent("a__boiloff_vent", slate.len(), a));
    graph.add_pipe(b, c, vent("b__boiloff_vent", slate.len(), b));
    graph.add_pipe(c, sky, vent("c__boiloff_vent", slate.len(), c));
    let mut engine = boiling_engine(graph);
    for t in 1..=TICKS {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("an open chain of three holdups must run: {t}: {e}"));
    }
}
