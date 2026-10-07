//! The `ThermoModel::bubble_pressure` contract at the ENGINE level (M11.1): what
//! the tick does with each KIND of `Err` (docs/DESIGN.md §13).
//!
//! The cavitation pass reads `SimError::Scenario` as "this fidelity has no
//! vapour–liquid equilibrium", reports nothing, and carries on — which is what
//! fourteen of the fifteen pre-M11 plants do on every tick. Every *other* variant
//! is a genuine fault and must fail the tick, because swallowing it would turn a
//! real numerical error into a silent "no criterion here", which is the failure
//! `NodeSnapshot::cavitation` spends a paragraph forbidding one level up.
//!
//! **No shipped plant and no loadable fixture can reach the second arm**: the
//! scenario format selects a thermo model by name, and both selectable models
//! either answer or refuse with `Scenario`. So the guard is a branch that
//! compiles, is cited in a design note as satisfying rule 5, and has never once
//! run — the shape `separation_contract.rs` exists for, and this file is its
//! sibling. The mutation pass measured it: swallowing every variant is caught by
//! nothing in the workspace without this file.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::engine::{Engine, EngineConfig};
use refinery_core::error::SimError;
use refinery_core::graph::{LeakRole, Node, NodeKind, Pipe, PlantGraph};
use refinery_core::traits::ThermoModel;
use refinery_core::units::*;
use refinery_solvers::{CutPointSplitter, NewtonFlowSolver, NoBoilOff, NoReactions, TroutonThermo};

const DT: Seconds = Seconds(0.1);

/// A model that fails the way a REAL fault fails — a `Numerical`, not a
/// `Scenario`. Everything else about it is valid: it answers `k_value` and
/// `dh_vap` exactly as the shipped correlation does, so the only thing this
/// fixture changes is the variant of the one refusal.
struct FaultyThermo(TroutonThermo);

impl ThermoModel for FaultyThermo {
    fn name(&self) -> &'static str {
        "test-faulty"
    }
    fn k_value(
        &self,
        slate: &Slate,
        component: usize,
        temperature: Kelvin,
        pressure: Pascal,
    ) -> Result<f64, SimError> {
        self.0.k_value(slate, component, temperature, pressure)
    }
    fn dh_vap(
        &self,
        slate: &Slate,
        component: usize,
        temperature: Kelvin,
    ) -> Result<JPerMol, SimError> {
        self.0.dh_vap(slate, component, temperature)
    }
    fn bubble_pressure(
        &self,
        _slate: &Slate,
        _composition: &Composition,
        _temperature: Kelvin,
    ) -> Result<Pascal, SimError> {
        Err(SimError::Numerical(
            "the faulty stub's bubble pressure is a genuine fault".into(),
        ))
    }
}

/// The same stub with the OTHER variant — the control, and the reason this file
/// is two tests rather than one. Without it, "the tick fails" would be evidence
/// about the stub rather than about the variant.
struct RefusingThermo;

impl ThermoModel for RefusingThermo {
    fn name(&self) -> &'static str {
        "test-refusing"
    }
    fn k_value(
        &self,
        _slate: &Slate,
        _component: usize,
        _temperature: Kelvin,
        _pressure: Pascal,
    ) -> Result<f64, SimError> {
        Err(SimError::Scenario("no equilibrium".into()))
    }
    fn dh_vap(
        &self,
        _slate: &Slate,
        _component: usize,
        _temperature: Kelvin,
    ) -> Result<JPerMol, SimError> {
        Err(SimError::Scenario("no latent heat".into()))
    }
    fn bubble_pressure(
        &self,
        _slate: &Slate,
        _composition: &Composition,
        _temperature: Kelvin,
    ) -> Result<Pascal, SimError> {
        Err(SimError::Scenario("no bubble pressure".into()))
    }
}

fn one_cut_slate() -> Slate {
    Slate::new(vec![PseudoComponent {
        name: "light".into(),
        tb: Kelvin(353.15),
        molar_mass: KgPerMol(0.1),
        density: Some(KgPerM3(700.0)),
        phase: Phase::Liquid,
        cp: JPerKgK(2000.0),
        cp_shape: None,
    }])
    .expect("a one-component slate is valid")
}

fn node(name: &str, kind: NodeKind) -> Node {
    Node {
        name: name.into(),
        kind,
        heat_input: Watt::ZERO,
    }
}

fn pipe(name: &str) -> Pipe {
    Pipe {
        name: name.into(),
        length: Meter(20.0),
        diameter: Meter(0.1),
        friction_factor: 0.02,
        elevation_change: Meter(0.0),
        leak: LeakRole::None,
        ambient_ua: WattPerKelvin::ZERO,
        stream: refinery_core::stream::Stream::stagnant(1, T_AMBIENT, P_ATM),
    }
}

/// Source → junction → sink: the smallest plant with a node the cavitation
/// criterion is a subject of. A junction, because it is the one subject kind
/// that needs no equipment parameters.
fn junction_plant() -> PlantGraph {
    let mut graph = PlantGraph::new();
    let feed = graph.add_node(node(
        "feed",
        NodeKind::Source {
            pressure: Pascal(3.0e5),
            temperature: Kelvin(380.0),
            composition: Composition::pure(1, 0),
        },
    ));
    let tee = graph.add_node(node("tee", NodeKind::Junction));
    let out = graph.add_node(node(
        "out",
        NodeKind::Sink {
            pressure: Pascal(1.5e5),
            temperature: Kelvin(380.0),
            composition: Composition::pure(1, 0),
        },
    ));
    graph.add_pipe(feed, tee, pipe("in_line"));
    graph.add_pipe(tee, out, pipe("out_line"));
    graph
}

fn engine_with(thermo: Box<dyn ThermoModel>) -> Engine {
    Engine::new(
        junction_plant(),
        one_cut_slate(),
        EngineConfig { dt: DT },
        Box::new(NewtonFlowSolver::default()),
        thermo,
        Box::new(NoReactions),
        Box::new(CutPointSplitter),
        Box::new(NoBoilOff),
        Box::new(refinery_solvers::ConstantEnthalpy),
        Box::new(refinery_solvers::NoLineFlash),
    )
}

/// A thermo model that FAILS — as opposed to one that has nothing to say — fails
/// the tick, rather than being read as "no criterion at this node".
#[test]
fn a_genuine_thermo_fault_fails_the_tick_rather_than_reporting_nothing() {
    let mut engine = engine_with(Box::new(FaultyThermo(TroutonThermo::new())));
    let err = engine
        .tick()
        .expect_err("a Numerical error from bubble_pressure must reach the caller");
    assert!(
        matches!(err, SimError::Numerical(_)),
        "the fault must arrive as itself, not repackaged: {err}"
    );
}

/// And the control: the SAME shape of refusal, in the variant that means "this
/// fidelity cannot answer", leaves the tick alone and simply reports nothing.
///
/// Without this the test above would be evidence that any failing thermo stops a
/// plant — which would make every `thermo = "constant"` plant in the corpus
/// unrunnable, and is exactly the edit the pair of them is here to tell apart.
#[test]
fn a_fidelity_with_no_equilibrium_reports_nothing_and_keeps_ticking() {
    let mut engine = engine_with(Box::new(RefusingThermo));
    for tick in 1..=20u64 {
        engine.tick().unwrap_or_else(|e| {
            panic!("a refusing thermo must not stop a plant: tick {tick}: {e}")
        });
    }
    let snapshot = engine.snapshot();
    let tee = snapshot
        .nodes
        .iter()
        .find(|n| n.name == "tee")
        .expect("the fixture has a junction");
    assert!(
        tee.cavitation.is_none(),
        "a model with no equilibrium must leave the node with NO criterion, not a \
         verdict of false"
    );
    // The control on the control: the node really is a subject, so the silence is
    // the model refusing and not the node being excluded.
    let mut engine = engine_with(Box::new(TroutonThermo::new()));
    for _ in 1..=20 {
        engine
            .tick()
            .expect("the same plant on a model that answers");
    }
    let snapshot = engine.snapshot();
    let tee = snapshot.nodes.iter().find(|n| n.name == "tee").unwrap();
    assert!(
        tee.cavitation.is_some(),
        "the same junction must get a verdict when the model can give one — \
         otherwise the assertion above is about the node kind, not the model"
    );
}
