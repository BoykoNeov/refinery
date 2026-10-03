//! `build_engine`: a parsed `ScenarioFile` to a runnable `Engine`.
//!
//! Unit conversion happens at this boundary and nowhere else (CLAUDE.md rule
//! 4): the file speaks bar, °C and Kv; everything past `node_kind` is SI.

use refinery_core::components::{Composition, CpShape, Phase, PseudoComponent, Slate};
use refinery_core::energy::T_REF;
use refinery_core::engine::{Engine, EngineConfig};
use refinery_core::error::SimError;
use refinery_core::graph::{
    Actuator, CascadeSpec, ColumnDraw, ControlAction, ControlLoop, ControlMode, ControlledValue,
    FurnaceCoil, HeatExchangerCoupling, LeakRole, LoopId, MeasuredVariable, MeasurementPoint, Node,
    NodeId, NodeKind, Pipe, PlantGraph, SetpointRange, TankState, Trip, TripAction, TripDirection,
    TripState, VesselState,
};
use refinery_core::stream::Stream;
use refinery_core::traits::{
    BoilOffModel, Controller, EnthalpyModel, FlowSolver, ReactionModel, SeparationModel,
    ThermoModel,
};
use refinery_core::units::{
    CubicMeter, JPerK, JPerKgK, Kelvin, Kg, KgPerM3, KgPerMol, KgPerSec, Meter, Seconds,
    SquareMeter, Watt, WattPerKelvin, P_ATM, T_AMBIENT,
};
use std::collections::BTreeMap;

use crate::schema::{
    bar_to_pa, c_to_k, kv_to_cv_si, ActuatorDef, ComponentDef, ControlDef, ExchangerDef,
    MeasurementDef, NodeDef, PipeDef, ScenarioFile, TripDef,
};
use crate::validate::{
    plant_phases, refuse_gas_leak, require_compatible_fidelity, require_declared_iff_used,
    require_gas_valve_x_t, seed_component_index, validate_node_def, validate_pipe_def,
    validate_topology,
};

/// Build a runnable engine from a scenario. Steps:
/// 1. Build the Slate (water-only until M3's [components] table exists).
/// 2. Instantiate nodes in file order (unit conversion at this boundary),
///    then pipes, resolving names → NodeIds; unknown names are errors.
/// 3. Validate topology: pumps/valves have exactly 1 in + 1 out edge;
///    every node reachable; at least one pressure-fixing node per
///    connected component (otherwise the hydraulic problem is singular —
///    fail at load with a clear message, not at solve with divergence).
/// 4. Select solver impls from [fidelity]; unknown names are errors
///    listing valid options.
pub fn build_engine(scenario: &ScenarioFile) -> Result<Engine, SimError> {
    // Step 0: fidelity combinations that cannot work, before anything is built.
    // It runs FIRST rather than beside the model selection in step 4, because a
    // cascade column's own config is validated in step 2a and a mis-paired plant
    // would otherwise be told about its draws when the real fault is its thermo.
    require_compatible_fidelity(scenario)?;

    // The boil-off model is selected HERE rather than with the other four in
    // step 4, because it is the one fidelity key that changes the plant's
    // TOPOLOGY: a model that can boil needs a vent edge per holdup to carry the
    // vapour out (docs/DESIGN.md §14 fork 4), and those edges must exist before
    // the topology is validated. Selecting once and carrying the box down is
    // what keeps "which model is this" and "does this model need vents" from
    // becoming two answers to one question.
    let (boiloff, vents_holdups) = select_boiloff(&scenario.fidelity.boiloff)?;

    // Step 1: slate. An absent [[components]] table means the water-only slate,
    // which is what every scenario written before M3 meant — that default is
    // what keeps those files bit-identical rather than merely still-loading.
    let slate = build_slate(&scenario.components)?;

    // Step 2: instantiate nodes in file order (IndexMap preserves it, so node
    // ids are deterministic), then pipes — resolving names → NodeIds. Unit
    // conversion happens here, at the human-friendly ↔ SI boundary.
    let mut graph = PlantGraph::new();
    for (name, def) in &scenario.nodes {
        validate_node_def(name, def)?;
        let kind = node_kind(name, def, &slate)?;
        graph.add_node(Node {
            name: name.clone(),
            kind,
            heat_input: Watt::ZERO,
        });
    }
    // Step 2a: resolve column draws now that every node exists — a draw may name
    // an outlet defined later in the file, exactly like an exchanger coupling.
    // This is where a draw's outlet must be a pressure-fixing node (the
    // free-node-on-a-draw-line rejection, DESIGN §5); the pipe-level topology
    // (one pipe per draw, correct direction) is checked in `validate_topology`
    // once the pipes exist.
    resolve_column_draws(&mut graph, scenario)?;

    for pipe in &scenario.pipes {
        validate_pipe_def(pipe)?;
        let from = graph.find_node(&pipe.from).ok_or_else(|| {
            SimError::Scenario(format!(
                "pipe '{}' references unknown 'from' node '{}'",
                pipe.name, pipe.from
            ))
        })?;
        let to = graph.find_node(&pipe.to).ok_or_else(|| {
            SimError::Scenario(format!(
                "pipe '{}' references unknown 'to' node '{}'",
                pipe.name, pipe.to
            ))
        })?;
        let whole = Pipe {
            name: pipe.name.clone(),
            length: Meter(pipe.length_m),
            diameter: Meter(pipe.diameter_m),
            friction_factor: pipe.friction_factor,
            elevation_change: Meter(pipe.elevation_change_m),
            leak: LeakRole::None,
            ambient_ua: WattPerKelvin(pipe.ambient_exchange_ua_w_per_k),
            // The solver overwrites mass_flow each tick, and transport
            // overwrites the temperature. Seed representative T/P at
            // ambient / atmospheric.
            stream: Stream::stagnant(slate.len(), T_AMBIENT, P_ATM),
        };
        match &pipe.leak_to {
            None => {
                graph.add_pipe(from, to, whole);
            }
            Some(atmosphere) => split_for_leak(&mut graph, pipe, from, to, whole, atmosphere)?,
        }
    }

    // Step 2b: thermally pair the exchanger sides, after every node exists so
    // both ends of a coupling can be resolved regardless of file order.
    build_couplings(&mut graph, &scenario.exchangers)?;

    // Step 2c: the boil-off vents, when the selected model can boil. Before
    // `validate_topology`, so the vents are part of the plant it validates
    // rather than edges that appear behind its back.
    if vents_holdups {
        build_boiloff_vents(&mut graph, slate.len(), &scenario.nodes)?;
    } else {
        // `vent_to` names where a vent GOES, and this plant builds none. Refused
        // rather than ignored, for `PuncturePipe`'s reason (M6.0): a key nothing
        // reads is a file that looks configured and is not. It cannot be refused
        // inside `build_boiloff_vents`, which is exactly the function that is not
        // called here.
        for (name, def) in &scenario.nodes {
            if let NodeDef::Tank {
                vent_to: Some(destination),
                ..
            } = def
            {
                return Err(SimError::Scenario(format!(
                    "tank '{name}' declares vent_to = '{destination}', but this plant's                      `[fidelity] boiloff` selects a model that never boils, so it has no                      vent edges at all for the key to route. Select a model that vents, or                      drop the key (docs/DESIGN.md §16 fork 2)"
                )));
            }
        }
    }

    // Step 2d: every tank's overflow (M23, docs/DESIGN.md §27 fork 2). AFTER the
    // vents, which is load-bearing rather than tidy: built first, the overflow
    // edges take the ids the vents had, and the five boil-off plants' published
    // edge ids move.
    build_overflow_edges(&mut graph, slate.len())?;

    // Step 3: validate topology at load — a clear error here beats solve-time
    // divergence for the same structural fault.
    validate_topology(&graph, &slate)?;

    // Step 3a: the single-phase connected-component guard, which also tells each
    // pipe which phase it carries. Seeding the stream's composition needs the
    // topology, so it happens here rather than in the pipe loop above; the value
    // is a tick-1 density seed only (see `seed_component_index`).
    let phases = plant_phases(&graph, &slate)?;
    require_gas_valve_x_t(&graph, &phases)?;
    refuse_gas_leak(&graph, &phases)?;
    for eid in graph.edge_ids().collect::<Vec<_>>() {
        let (src, _) = graph.endpoints(eid);
        let index = seed_component_index(&slate, phases[src.0 as usize]);
        graph.pipe_mut(eid).stream.composition = Composition::pure(slate.len(), index);
    }

    // Step 3b: the control loops, after every node exists so a loop may name an
    // actuator defined later in the file, exactly like an exchanger coupling or a
    // column draw. It runs after `plant_phases` because a loop's seeded
    // measurement is a real read of the finished plant, not a placeholder.
    build_controls(&mut graph, &slate, &scenario.controls, &scenario.pipes)?;

    // Step 3c: the trips (M22), after every node exists, for the same reason.
    build_trips(&mut graph, &slate, &scenario.trips, &scenario.pipes)?;

    // Step 4: select solver impls from [fidelity]; unknown names are errors
    // listing the valid options.
    let flow: Box<dyn FlowSolver> = match scenario.fidelity.flow.as_str() {
        "newton" => Box::new(refinery_solvers::NewtonFlowSolver::default()),
        "simple" => Box::new(refinery_solvers::SimpleFlowSolver::default()),
        other => {
            return Err(SimError::Scenario(format!(
                "unknown flow solver '{other}' (valid: newton, simple)"
            )))
        }
    };
    let thermo: Box<dyn ThermoModel> = match scenario.fidelity.thermo.as_str() {
        "constant" => Box::new(refinery_solvers::ConstantThermo),
        // Raoult over a Clausius–Clapeyron vapour pressure with Trouton's rule
        // for Δh_vap (M7.2). Selectable from M7.3, when the cascade gave a
        // K-value its first consumer — see `Fidelity::thermo`.
        "trouton" => Box::new(refinery_solvers::TroutonThermo::new()),
        other => {
            return Err(SimError::Scenario(format!(
                "unknown thermo model '{other}' (valid: constant, trouton)"
            )))
        }
    };
    let reactions: Box<dyn ReactionModel> = match scenario.fidelity.reactions.as_str() {
        "none" => Box::new(refinery_solvers::NoReactions),
        // The FCC placeholder table (M4.1). Both reacting fidelities resolve
        // their lumps against the slate by name, so an absent lump is a
        // load-time error, not a solve-time surprise.
        "lookup" => Box::new(refinery_solvers::SimpleLookup::fcc_demo(&slate)?),
        // FCC 4-lump Arrhenius kinetics (M4.2), integrated with fixed-count RK4
        // over the reactor's `tau_s`.
        "fcc" => Box::new(refinery_solvers::FourLump::fcc(&slate)?),
        other => {
            return Err(SimError::Scenario(format!(
                "unknown reaction model '{other}' (valid: none, lookup, fcc)"
            )))
        }
    };

    let separation: Box<dyn SeparationModel> = match scenario.fidelity.separation.as_str() {
        // M3.2's boiling-range splitter, and the default (see `Fidelity`).
        "cut_point" => Box::new(refinery_solvers::CutPointSplitter),
        // The M7.3 equilibrium-stage cascade. Its pairing with `thermo` is
        // already settled by `require_compatible_fidelity` in step 0.
        "cascade" => Box::new(refinery_solvers::StageCascade::new()),
        other => {
            return Err(SimError::Scenario(format!(
                "unknown separation model '{other}' (valid: cut_point, cascade)"
            )))
        }
    };

    // The heat-capacity seam (M16.2). Every pairing it cannot work in was
    // already refused in step 0, in both directions, so this match only has to
    // name the model.
    let enthalpy: Box<dyn EnthalpyModel> = match scenario.fidelity.heat_capacity.as_str() {
        "constant" => Box::new(refinery_solvers::ConstantEnthalpy),
        "linear" => Box::new(refinery_solvers::LinearCpEnthalpy),
        other => {
            return Err(SimError::Scenario(format!(
                "unknown heat capacity model '{other}' (valid: constant, linear)"
            )))
        }
    };

    let config = EngineConfig {
        dt: refinery_core::units::Seconds(scenario.simulation.dt),
    };
    Ok(Engine::new(
        graph, slate, config, flow, thermo, reactions, separation, boiloff, enthalpy,
    ))
}

/// Select the boil-off model, and say whether it needs vent edges.
///
/// **Two answers from one match, deliberately.** The topology a plant is built
/// with and the model it runs must agree about whether a holdup can boil: build
/// the vents without the model and the plant carries three dead edges; select
/// the model without the vents and the first boiling tank fails the tick with
/// nowhere to put its vapour (`Engine::tick` refuses exactly that). Returning
/// the pair makes the agreement structural rather than a convention two call
/// sites keep.
fn select_boiloff(name: &str) -> Result<(Box<dyn BoilOffModel>, bool), SimError> {
    match name {
        // The default, and what every file written before M12 means.
        "none" => Ok((Box::new(refinery_solvers::NoBoilOff), false)),
        // The M12.1 equilibrium flash. Its pairing with `thermo` is already
        // settled by `require_compatible_fidelity` in step 0.
        "flash" => Ok((Box::new(refinery_solvers::FlashBoilOff), true)),
        other => Err(SimError::Scenario(format!(
            "unknown boil-off model '{other}' (valid: none, flash)"
        ))),
    }
}

/// Build one vent edge per liquid holdup, tank → `Atmosphere`, for a plant whose
/// boil-off model can boil (docs/DESIGN.md §14 fork 4).
///
/// **Why the loader builds these and the scenario does not declare them.** The
/// demo this milestone ships is `crude_column_cascade.toml` with ONE line
/// changed, which is the M7 pair pattern it is meant to be diffed against — and
/// a file that had to declare an atmosphere node and a vent pipe per tank would
/// differ by nine lines instead of one, so the difference in the numbers would
/// no longer be attributable to the key. It is also the `leak_to` precedent
/// (M6.0): a leak path is graph surgery performed at LOAD, because a topology
/// that appears mid-run changes the snapshot's shape mid-run.
///
/// **An existing `Atmosphere` is reused rather than joined by a second one.** A
/// plant may already have one (`leaking_line.toml` does); two would be two names
/// for the outside world, and the vapour would leave through whichever the
/// loader happened to pick.
fn build_boiloff_vents(
    graph: &mut PlantGraph,
    components: usize,
    nodes: &indexmap::IndexMap<String, NodeDef>,
) -> Result<(), SimError> {
    let tanks: Vec<NodeId> = graph
        .node_ids()
        .filter(|id| matches!(graph.node(*id).kind, NodeKind::Tank(_)))
        .collect();
    if tanks.is_empty() {
        // A plant with no holdup selects a boil-off model and gets no vents,
        // which is not an error: the key says what a holdup WOULD do.
        return Ok(());
    }

    // Every tank that names a destination, keyed by the tank's node id. Built
    // before the atmosphere is, because a plant on which EVERY tank routes its
    // vent elsewhere still needs one: the receiving drum has a vent of its own,
    // and the file that declared the drum did not have to say where it goes.
    let mut declared: BTreeMap<NodeId, &str> = BTreeMap::new();
    for tank in &tanks {
        let name = graph.node(*tank).name.as_str();
        if let Some(NodeDef::Tank {
            vent_to: Some(destination),
            ..
        }) = nodes.get(name)
        {
            declared.insert(*tank, destination.as_str());
        }
    }

    let existing_atmosphere = graph
        .node_ids()
        .find(|id| matches!(graph.node(*id).kind, NodeKind::Atmosphere));
    let atmosphere = match existing_atmosphere {
        Some(existing) => existing,
        None => {
            const VENT_ATMOSPHERE: &str = "boiloff_atmosphere";
            if graph.find_node(VENT_ATMOSPHERE).is_some() {
                return Err(SimError::Scenario(format!(
                    "this plant selects `[fidelity] boiloff` with a model that vents, whose                      atmosphere node would be named '{VENT_ATMOSPHERE}' — and the plant                      already has a node by that name which is not an atmosphere. Rename it,                      or declare `type = \"atmosphere\"` on it and the vents will use it"
                )));
            }
            graph.add_node(Node {
                name: VENT_ATMOSPHERE.into(),
                kind: NodeKind::Atmosphere,
                heat_input: Watt::ZERO,
            })
        }
    };

    for tank in tanks {
        // Where this tank's vapour goes: the atmosphere above by default, or
        // whatever `vent_to` named (M14, docs/DESIGN.md §16 forks 2 and 5).
        let destination = match declared.get(&tank) {
            None => atmosphere,
            Some(name) => resolve_vent_destination(graph, tank, name)?,
        };
        let name = format!("{}__boiloff_vent", graph.node(tank).name);
        if graph.edge_ids().any(|eid| graph.pipe(eid).name == name) {
            return Err(SimError::Scenario(format!(
                "tank '{}' would vent its boil-off through an edge named '{name}', and this                  plant already has a pipe by that name. Rename it",
                graph.node(tank).name
            )));
        }
        // **Geometry of ZERO, for the leak orifice's reason.** A vent has no
        // length to resist with and no bore that means anything: its flow is
        // prescribed by the holdup's enthalpy balance, and `compile_edge`
        // returns on the role before reading either. Writing plausible numbers
        // here would put a second, silent resistance in a path that has none.
        //
        // The direction is tank → atmosphere, so graph-positive IS outward and
        // the engine's write needs no sign flip.
        graph.add_pipe(
            tank,
            destination,
            Pipe {
                name,
                length: Meter(0.0),
                diameter: Meter(0.0),
                friction_factor: 0.0,
                elevation_change: Meter(0.0),
                ambient_ua: WattPerKelvin(0.0),
                leak: LeakRole::BoilOffVent { emitter: tank },
                stream: Stream::stagnant(components, T_AMBIENT, P_ATM),
            },
        );
    }
    // The cycle refusal, and it has ONE owner rather than a copy here: the
    // engine needs an emitter-before-receiver evaluation order every tick, and a
    // cycle is exactly the graph for which no such order exists
    // (`PlantGraph::holdup_evaluation_order`, docs/DESIGN.md §16 fork 5). Asking
    // for it at LOAD is what turns "the first tick fails" into "the file is
    // refused", and the two answers cannot disagree because they are one
    // function.
    graph.holdup_evaluation_order()?;
    Ok(())
}

/// Build one overflow edge per tank, `<tank>__overflow`, to the plant's first
/// `Atmosphere` — or to a new `overflow_atmosphere` node when the plant has none
/// (M23, docs/DESIGN.md §27 fork 2).
///
/// **Every tank, not only tanks that opt in.** The brim is a property of every
/// shell; a per-tank key would leave the default silently overfilling, which is
/// the state this milestone exists to remove.
///
/// **An existing `Atmosphere` is reused**, the vents' rule, and that includes the
/// vents' own `boiloff_atmosphere` — which is why a boil-off plant with no
/// declared atmosphere keeps its bytes. A NEW atmosphere node is not free: the
/// solvers seed every free node at the mean of the pinned pressures, so one more
/// pinned node at `P_ATM` moves tick 1's starting point inside the tolerance ball
/// (§27 premise 2). Accepted by the user; the reference was re-set once.
///
/// **The name is the frontend's only handle on the edge.** `EdgeSnapshot` does
/// not publish the edge's role, so a declared pipe that took the name would be
/// read as the spill. That is why the collision is refused rather than suffixed.
///
/// **Zero geometry, direction tank → atmosphere**, the vents' convention: its
/// flow is written by the engine, and graph-positive is outward.
fn build_overflow_edges(graph: &mut PlantGraph, components: usize) -> Result<(), SimError> {
    let tanks: Vec<NodeId> = graph
        .node_ids()
        .filter(|id| matches!(graph.node(*id).kind, NodeKind::Tank(_)))
        .collect();
    if tanks.is_empty() {
        return Ok(());
    }
    let existing_atmosphere = graph
        .node_ids()
        .find(|id| matches!(graph.node(*id).kind, NodeKind::Atmosphere));
    let atmosphere = match existing_atmosphere {
        Some(existing) => existing,
        None => {
            const OVERFLOW_ATMOSPHERE: &str = "overflow_atmosphere";
            if graph.find_node(OVERFLOW_ATMOSPHERE).is_some() {
                return Err(SimError::Scenario(format!(
                    "this plant has tanks, whose overflow atmosphere would be named \
                     '{OVERFLOW_ATMOSPHERE}' — and the plant already has a node by that name \
                     which is not an atmosphere. Rename it, or declare `type = \"atmosphere\"` \
                     on it and the overflows will use it (docs/DESIGN.md §27 fork 2)"
                )));
            }
            graph.add_node(Node {
                name: OVERFLOW_ATMOSPHERE.into(),
                kind: NodeKind::Atmosphere,
                heat_input: Watt::ZERO,
            })
        }
    };
    for tank in tanks {
        let name = format!("{}__overflow", graph.node(tank).name);
        if graph.edge_ids().any(|eid| graph.pipe(eid).name == name) {
            return Err(SimError::Scenario(format!(
                "tank '{}' spills over its brim through an edge named '{name}', and this \
                 plant already has a pipe by that name. A frontend finds a tank's spill by \
                 that name, so it cannot be shared. Rename the pipe (docs/DESIGN.md §27 \
                 fork 2)",
                graph.node(tank).name
            )));
        }
        graph.add_pipe(
            tank,
            atmosphere,
            Pipe {
                name,
                length: Meter(0.0),
                diameter: Meter(0.0),
                friction_factor: 0.0,
                elevation_change: Meter(0.0),
                ambient_ua: WattPerKelvin(0.0),
                leak: LeakRole::Overflow { owner: tank },
                stream: Stream::stagnant(components, T_AMBIENT, P_ATM),
            },
        );
    }
    Ok(())
}

/// Resolve a tank's `vent_to` to a node that may receive its vapour
/// (docs/DESIGN.md §16 fork 5).
///
/// Two kinds are admitted and each of the rest is refused **for its own
/// reason**, because M11 fork 4 is the precedent: a trigger naming four node
/// kinds from memory was falsified by the first slice that enumerated the type,
/// and `NodeKind` has fourteen variants. The match below is exhaustive, so a
/// fifteenth cannot be admitted by omission.
fn resolve_vent_destination(
    graph: &PlantGraph,
    tank: NodeId,
    destination: &str,
) -> Result<NodeId, SimError> {
    let tank_name = graph.node(tank).name.clone();
    let target = graph.find_node(destination).ok_or_else(|| {
        SimError::Scenario(format!(
            "tank '{tank_name}' declares vent_to = '{destination}', which is not a node in \
             this plant"
        ))
    })?;
    if target == tank {
        return Err(SimError::Scenario(format!(
            "tank '{tank_name}' declares vent_to = '{destination}', which is itself. A vent \
             carries vapour OUT of the holdup that boiled it; a self-loop would hand the \
             vapour straight back to the inventory the flash took it from, and the tank \
             would boil for ever at no cost"
        )));
    }
    let reason = match &graph.node(target).kind {
        // The two admitted kinds. An `Atmosphere` is today's behaviour written
        // out rather than left to the default; a `Tank` is the recovery drum,
        // and it is the only holdup in this engine whose state is an inventory
        // of LIQUID, which is what a condensate is.
        NodeKind::Atmosphere | NodeKind::Tank(_) => return Ok(target),
        NodeKind::Vessel(_) => {
            "a gas holdup whose state is a pressure, so an arriving condensate has nowhere \
             to be. That is the pressurised two-phase holdup (`docs/DEFERRED.md` B11)"
        }
        NodeKind::Sink { .. } => {
            "a sink, whose composition is DECLARED. Routing a computed vapour into a \
             declared composition is the `Atmosphere` back-feed problem under another name; \
             and a flare is combustion (`docs/DEFERRED.md` B7), not a sink"
        }
        NodeKind::Source { .. } => "a source: pinned, and upstream by definition",
        NodeKind::Column { .. } => {
            "a column, which has its own separation and its own inlet contract. A vent is \
             not a feed, and a cascade's saturated-liquid feed guard would refuse it anyway"
        }
        NodeKind::Reactor { .. } => {
            "a reactor: a zero-volume node with an IMPOSED outlet temperature, so a stream              arriving there loses the state that says how much of it can condense, and the              next paragraph's arithmetic applies as well"
        }
        NodeKind::Junction
        | NodeKind::Pump { .. }
        | NodeKind::Valve { .. }
        | NodeKind::ReliefValve { .. }
        | NodeKind::CheckValve { .. }
        | NodeKind::Furnace { .. }
        | NodeKind::Cooler { .. }
        | NodeKind::HeatExchanger => {
            "a zero-volume node. The condensate would have to leave again in the same tick \
             by a pressure-driven edge, but a vent's flow is PRESCRIBED and the hydraulic \
             solve cannot see it, so the node's mass balance would carry an inflow nothing \
             balances. Not unsupported — arithmetically inconsistent"
        }
    };
    Err(SimError::Scenario(format!(
        "tank '{tank_name}' declares vent_to = '{destination}', which is {reason}. A \
         boil-off vent may end at an `atmosphere` (the default) or at another `tank` \
         (docs/DESIGN.md §16 fork 5)"
    )))
}

/// Build the canonical slate from the `[[components]]` table, in file order.
///
/// An empty table is the water-only slate rather than an error: that is what
/// every pre-M3 scenario means, and making it explicit would churn every file
/// for no gain. Duplicate names ARE an error — `Composition` is written by name,
/// so two cuts called the same thing make a composition ambiguous, and
/// `Slate::index_of` would silently resolve every mention to the first.
/// Build the plant's control loops from `[[controls]]`, after every node exists.
///
/// Declaration order is `LoopId` order and execution order, exactly as `[nodes]`
/// order fixes node ids.
///
/// The refusals here are fork 5's, and each closes a way a file can declare a
/// loop that would run and be wrong rather than fail:
///
/// - tuning that belongs to the other algorithm, in both directions
///   (`integral_time_s` and `initial_output` on `"p"`; either of them missing on
///   `"pi"`),
/// - two loops naming one actuator, which is two writers of one opening with no
///   defined resolution order — split-range and override control are real, and are
///   deferred *with an arbitration*, not left to declaration order,
/// - a level measured on a node that is not a `Tank`, or a pressure measured on
///   one that is not a `Vessel` — three distinct refusals on the pressure side
///   alone (a tank, whose pressure IS its level in a worse unit; a junction, whose
///   pressure is genuinely solved and absent at tick 0, refused as a scope
///   decision since M19; and the boundary kinds, whose pressures are pinned by
///   declaration),
/// - a temperature measured on anything that is not a holdup (M17) or a furnace or
///   cooler OUTLET (M19) — refused with its own reason for the other zero-volume
///   kinds, for a relief valve, for a column or reactor and for a boundary,
/// - a flow (M20, docs/DESIGN.md §24) measured anywhere but a pipe this file
///   DECLARES, which is one of the actuating valve's own two pipes and declares no
///   `leak_to`; `node` and `pipe` both given or neither, and a flow asked of a
///   node or another variable asked of a pipe,
/// - a setpoint or a gain key belonging to ANOTHER variable, every direction,
/// - an actuator the variable cannot pair with (docs/DESIGN.md §21 fork 3's table:
///   a valve for a level, a pressure or a flow, a cooler or furnace for a
///   temperature), each refused pairing with its own reason, and a `ReliefValve`
///   always,
/// - a direction of action the actuator contradicts: a cooler must be direct, a
///   furnace and a flow loop's valve must SAY reverse, and a level or pressure
///   loop's valve must be a drain of its holdup (direct, the default) or a fill
///   that SAYS reverse — one hop, both ways, and a valve that is neither is
///   refused in either direction (M29, docs/DESIGN.md §32),
/// - `max_duty_mw` missing on a cooler or present on a valve, not finite and
///   positive, or a declared cooler duty outside `[0, max_duty_mw]`.
///
/// `PlantGraph::measure` is called here exactly as the tick pass calls it, with the
/// empty `NodeStates` and the absent hydraulic solution that are the truth at load.
/// A pipe's flow therefore has no measurement at load either (§24 fork 2), and
/// takes the outlet loop's rule below unchanged. For a level, a pressure or a
/// holdup's temperature — all STORED quantities — that gives a real measurement, and
/// a snapshot taken before the first tick reports it. For a furnace or cooler
/// OUTLET it gives none, because an outlet is resolved by the tick and does not
/// exist before the first one (docs/DESIGN.md §23): the loop is born without a
/// measurement, and the tick's "no measurement, no action" rule holds it until it
/// has one. It is never given a stand-in. `last_output` is
/// seeded through `PlantGraph::actuator_position` — the reader the tick pass and
/// the MANUAL→AUTO transfer also use — which is what MANUAL would report and what
/// AUTO overwrites on tick 1.
///
/// That same measurement is what a PI loop's memory is derived AGAINST: fork 5's
/// `initial_output` says where the actuator starts, and the integral term is
/// whatever makes the controller ask for that position given the error standing at
/// load. So the declared number is the one a reader can check on the faceplate at
/// tick 0, and the state behind it is derived rather than declared twice. An
/// outlet loop's memory waits for its first measurement and is derived then, from
/// the same declared number (§23 fork 3).
fn build_controls(
    graph: &mut PlantGraph,
    slate: &Slate,
    defs: &[ControlDef],
    pipes: &[PipeDef],
) -> Result<(), SimError> {
    let mut seen_names: Vec<&str> = Vec::new();
    let mut claimed_actuators: Vec<(Actuator, &str)> = Vec::new();

    for def in defs {
        if seen_names.contains(&def.name.as_str()) {
            return Err(SimError::Scenario(format!(
                "two control loops are called '{}'. A loop's name is what its faceplate \
                 is labelled with, so two of them make a snapshot ambiguous",
                def.name
            )));
        }
        seen_names.push(&def.name);

        let variable = parse_variable(def)?;

        let point = resolve_measurement_point(
            graph,
            &format!("control loop '{}'", def.name),
            &def.measurement,
            pipes,
        )?;
        let point_name = graph.point_name(point).to_owned();

        // The measured node must be able to answer for the variable. Asking the
        // graph rather than matching the kind here is deliberate: `measure` is the
        // single owner of where a measurement comes from, so a kind this loader
        // accepted and that reader then rejected is not a state that can exist.
        // Bound rather than discarded: this is both the load-time check that the
        // node can answer for the variable AND the measurement the loop is born
        // holding — and, for a PI loop, the error its declared `initial_output` is
        // back-calculated against. Reading it twice would let the two drift apart
        // in a way nothing downstream could detect.
        let measurement = graph
            .measure(
                slate,
                &refinery_core::energy::NodeStates::default(),
                None,
                point,
                variable,
            )
            .map_err(|e| {
                SimError::Scenario(format!(
                    "control loop '{}' cannot measure {} on {} '{point_name}': {e}",
                    def.name,
                    def.measurement.variable,
                    match point {
                        MeasurementPoint::Node(_) => "node",
                        MeasurementPoint::Pipe(_) => "pipe",
                    }
                ))
            })?;

        let action = match def.action.as_deref() {
            None | Some("direct") => ControlAction::Direct,
            Some("reverse") => ControlAction::Reverse,
            Some(other) => {
                return Err(SimError::Scenario(format!(
                    "control loop '{}' declares unknown action '{other}' (valid: direct, \
                     reverse)",
                    def.name
                )))
            }
        };

        // **What the loop writes: a node, or another loop's setpoint** (M25,
        // docs/DESIGN.md §29 fork 1). A cascade primary's keys are checked here,
        // against its secondary's DECLARATION; the link itself — the depth rule,
        // the pairing, the sign, the range against the secondary's own setpoint
        // check — is checked by `link_cascades` once every loop is built, because
        // a file may declare the secondary after its primary.
        let (actuator, max_duty, setpoint_range) = match &def.actuator {
            ActuatorDef::Loop(link) => {
                let (secondary, range) = cascade_link_keys(def, defs, &link.name)?;
                (Actuator::Loop(secondary), None, Some(range))
            }
            ActuatorDef::Node(actuator_name) => {
                refuse_range_keys_on_node(def)?;
                let actuator = graph.find_node(actuator_name).ok_or_else(|| {
                    SimError::Scenario(format!(
                        "control loop '{}' actuates unknown node '{}'",
                        def.name, def.actuator
                    ))
                })?;

                // **The pairing table, enumerated rather than left to a fall-through**
                // (docs/DESIGN.md §21 fork 3). Which actuators each variable accepts, and
                // for each refused pairing its own reason — and, for the one duty
                // actuator, the loop's declared authority over it. `max_duty_mw` is
                // required on a cooler and refused on a valve, both directions, the
                // `density_kg_per_m3` rule.
                let max_duty = match (variable, &graph.node(actuator).kind) {
                    // Its own reason rather than "not a valve", exactly as
                    // `Command::SetValveOpening` refuses it: a relief valve IS a valve,
                    // and the point is that its opening is not a setpoint at all — it is a
                    // memoryless function of its own inlet pressure, recomputed every
                    // solve (docs/DESIGN.md §3a fork 5). A loop pointed at one would write
                    // a number the next solve overwrites.
                    (_, NodeKind::ReliefValve { .. }) => {
                        return Err(SimError::Scenario(format!(
                            "control loop '{}' actuates '{}', a relief valve. Its opening is \
                     actuated by its own inlet pressure and is recomputed on every solve, \
                     so a controller writing it would be overwritten before the tick ended",
                            def.name, def.actuator
                        )))
                    }
                    // The relief valve's reason, with the disc's own trigger
                    // (docs/DESIGN.md §33).
                    (_, NodeKind::CheckValve { .. }) => {
                        return Err(SimError::Scenario(format!(
                            "control loop '{}' actuates '{}', a check valve. Its disc is moved \
                     by the forward drive across it and is recomputed on every solve, so a \
                     controller writing it would be overwritten before the tick ended",
                            def.name, def.actuator
                        )))
                    }
                    (
                        MeasuredVariable::Level
                        | MeasuredVariable::Pressure
                        | MeasuredVariable::Flow,
                        NodeKind::Valve { .. },
                    ) => {
                        if def.max_duty_mw.is_some() {
                            return Err(SimError::Scenario(format!(
                        "control loop '{}' declares `max_duty_mw` on a valve actuator. That key \
                         is a DUTY actuator's range — a cooler's or a furnace's — and a valve's \
                         opening is \
                         already a fraction, so the number would be read by nothing \
                         (docs/DESIGN.md §21 fork 3)",
                        def.name
                    )));
                        }
                        // **One hop, and that is the whole of the sign check** (docs/DESIGN.md
                        // §24 fork 3). `validate_degrees` holds every valve to exactly one
                        // inlet and one outlet edge by declared direction, so a pipe that
                        // touches the valve IS its whole inlet or its whole outlet, with
                        // nothing branching between them — and opening the valve raises the
                        // flow through it. A meter further off, or a bypass valve beside the
                        // pipe, is E12.
                        //
                        // A column draw or a boil-off vent cannot reach here, and no guard
                        // is written for either: a draw ends at a product store and a vent
                        // runs between holdups, so neither is ever a valve's edge, and this
                        // adjacency rule excludes both structurally (the M8.2 precedent: a
                        // refusal nothing can reach is not a refusal).
                        if let MeasurementPoint::Pipe(pipe) = point {
                            let (from, to) = graph.endpoints(pipe);
                            if from != actuator && to != actuator {
                                return Err(SimError::Scenario(format!(
                            "control loop '{}' measures the flow in pipe '{point_name}', which \
                             is not one of valve '{}''s own two pipes. A flow loop's sign is \
                             checked in ONE hop — a valve has exactly one inlet and one outlet, \
                             so opening it raises the flow in either — and a meter further \
                             from its valve, or a valve bypassing the metered pipe (which is \
                             DIRECT acting), is not admitted (docs/DEFERRED.md E12, \
                             docs/DESIGN.md §24 fork 3)",
                            def.name, def.actuator
                        )));
                            }
                        }
                        None
                    }
                    // The two DUTY actuators share one arm: the same range key, the same
                    // load-time range check, the same position map. Which way each acts is
                    // checked below against the declared `action`, not here.
                    (
                        MeasuredVariable::Temperature,
                        NodeKind::Cooler { duty } | NodeKind::Furnace { duty, .. },
                    ) => {
                        let max_mw = require_keyed(
                            def.max_duty_mw,
                            &def.name,
                            "max_duty_mw",
                            "the duty the loop's full output stands for",
                        )?;
                        if !max_mw.is_finite() || max_mw <= 0.0 {
                            return Err(SimError::Scenario(format!(
                                "control loop '{}' declares `max_duty_mw = {max_mw}`; the loop's \
                         authority must be a finite duty above zero, since its output is a \
                         fraction of it",
                                def.name
                            )));
                        }
                        let max = Watt(max_mw * 1e6);
                        // §21 fork 4, at load: a declared duty outside the loop's range is
                        // a position the loop could never have produced, and MANUAL
                        // tracking would report it as a fraction above 1.
                        if duty.value() > max.value() {
                            return Err(SimError::Scenario(format!(
                                "control loop '{}' actuates '{}', whose declared duty {} MW is \
                         above the loop's `max_duty_mw = {max_mw}`. The loop's output is a \
                         fraction of that range, so a starting duty outside it is a position \
                         the loop could never have produced (docs/DESIGN.md §21 fork 4)",
                                def.name,
                                def.actuator,
                                duty.value() / 1e6
                            )));
                        }
                        Some(max)
                    }
                    // **Refused with a physical reason and not ruled out**: a coolant
                    // valve is the real-world temperature actuator, but this engine's
                    // cooler is a fixed duty with no coolant side, so a valve could only
                    // move a temperature by changing a PROCESS flow — a different loop
                    // with a sign that has to be argued per plant.
                    (MeasuredVariable::Temperature, NodeKind::Valve { .. }) => {
                        return Err(SimError::Scenario(format!(
                    "control loop '{}' holds a temperature with valve '{}'. This engine has no \
                     coolant stream — a cooler is a duty with no coolant side — so a valve can \
                     move a temperature only by changing a PROCESS flow, whose sign depends \
                     on the plant. Actuate a `cooler` instead (docs/DESIGN.md §21 fork 3)",
                    def.name, def.actuator
                )))
                    }
                    (MeasuredVariable::Level, NodeKind::Cooler { .. }) => {
                        return Err(SimError::Scenario(format!(
                    "control loop '{}' holds a level with cooler '{}'. A cooler moves nothing a \
                     level loop measures: a cut's density is a constant in this engine, so a \
                     tank's level does not depend on its temperature (docs/DESIGN.md §21 fork \
                     3)",
                    def.name, def.actuator
                )))
                    }
                    // A SCOPE refusal, and the message must not claim more: a cooler DOES
                    // move a vessel's pressure (`P = m·R·T/(V·M̄)`), so the reason is that
                    // its effect runs through a temperature, not that it is inert.
                    (MeasuredVariable::Pressure, NodeKind::Cooler { .. }) => {
                        return Err(SimError::Scenario(format!(
                    "control loop '{}' holds a pressure with cooler '{}'. A cooler does move a \
                     vessel's pressure, but only THROUGH its temperature, which makes this a \
                     cascade wearing one loop's name — and a pressure over a temperature is not \
                     an admitted cascade pairing either (docs/DEFERRED.md E18). Refused as a \
                     scope decision, not as physics (docs/DESIGN.md §21 fork 3, §29 fork 5)",
                    def.name, def.actuator
                )))
                    }
                    (MeasuredVariable::Level | MeasuredVariable::Pressure, _) => {
                        return Err(SimError::Scenario(format!(
                    "control loop '{}' actuates '{}', which is not a valve. A level or pressure \
                     loop writes a valve's opening; pump speed and the rest are deferred with \
                     their own arguments (docs/DESIGN.md §21 fork 3)",
                    def.name, def.actuator
                )))
                    }
                    (MeasuredVariable::Temperature, _) => {
                        return Err(SimError::Scenario(format!(
                    "control loop '{}' actuates '{}', which is not a cooler. A temperature loop \
                     writes a cooler's or a furnace's duty (docs/DESIGN.md §21 fork 3, §22)",
                    def.name, def.actuator
                )))
                    }
                    (
                        MeasuredVariable::Flow,
                        NodeKind::Cooler { .. } | NodeKind::Furnace { .. },
                    ) => {
                        return Err(SimError::Scenario(format!(
                    "control loop '{}' holds a flow with '{}', a cooler or furnace. A duty moves \
                     heat and no mass: this engine's hydraulics do not depend on a unit's duty, \
                     so the loop would have no effect on what it measures. A flow loop writes a \
                     valve's opening (docs/DESIGN.md §24 fork 3)",
                    def.name, def.actuator
                )))
                    }
                    (MeasuredVariable::Flow, NodeKind::Pump { .. }) => {
                        return Err(SimError::Scenario(format!(
                    "control loop '{}' holds a flow with pump '{}'. A pump's `on` is a switch, \
                     not a fraction a controller can position, and pump speed is not modelled. \
                     A flow loop writes a valve's opening (docs/DESIGN.md §24 fork 3)",
                    def.name, def.actuator
                )))
                    }
                    (MeasuredVariable::Flow, _) => {
                        return Err(SimError::Scenario(format!(
                    "control loop '{}' actuates '{}', which is not a valve. A flow loop writes \
                     a valve's opening, on one of that valve's own two pipes (docs/DESIGN.md \
                     §24 fork 3)",
                    def.name, def.actuator
                )))
                    }
                };

                // **The direction of action, declared and checked** (docs/DESIGN.md §22
                // fork 2). Absent means direct — a true statement about every loop
                // written before M18 — except on a furnace, where a default would make the
                // file's most surprising property invisible — and on a flow loop, for the
                // same reason (M20). Checked against the actuator wherever the sign is
                // physics, in both directions. A valve holding the flow in its own pipe
                // is a valve whose sign IS physics: the pairing table above has already
                // checked the one hop (docs/DESIGN.md §24 fork 3). On a level or pressure
                // loop a valve's sign is topology, and since M29 the loader reads it —
                // drain or fill, one hop — and holds the declaration to it both ways
                // (docs/DESIGN.md §32, `check_holdup_valve_action`).
                if let (
                    MeasuredVariable::Level | MeasuredVariable::Pressure,
                    MeasurementPoint::Node(holdup),
                    NodeKind::Valve { .. },
                ) = (variable, point, &graph.node(actuator).kind)
                {
                    check_holdup_valve_action(
                        def,
                        variable,
                        &point_name,
                        valve_side(graph, holdup, actuator),
                        action,
                    )?;
                }
                let flow_loop = variable == MeasuredVariable::Flow;
                match (&graph.node(actuator).kind, action, def.action.is_some()) {
                    (NodeKind::Valve { .. }, ControlAction::Direct, false) if flow_loop => {
                        return Err(SimError::Scenario(format!(
                            "control loop '{}' holds the flow through valve '{}' and declares no \
                     `action`. Opening a valve RAISES the flow in its own pipe, so this loop is \
                     reverse acting — the industry's own convention for a flow controller — \
                     and that must be declared rather than defaulted: add `action = \
                     \"reverse\"` (docs/DESIGN.md §24 fork 3)",
                            def.name, def.actuator
                        )))
                    }
                    (NodeKind::Valve { .. }, ControlAction::Direct, true) if flow_loop => {
                        return Err(SimError::Scenario(format!(
                    "control loop '{}' declares `action = \"direct\"` on valve '{}', which it \
                     holds a flow with. A direct loop's output must LOWER its measurement as it \
                     rises, and opening a valve raises the flow in its own pipe: this loop \
                     would shut the valve the moment the flow ran low. Declare `action = \
                     \"reverse\"` (docs/DESIGN.md §24 fork 3)",
                    def.name, def.actuator
                )))
                    }
                    (NodeKind::Furnace { .. }, ControlAction::Direct, false) => {
                        return Err(SimError::Scenario(format!(
                    "control loop '{}' actuates furnace '{}' and declares no `action`. More \
                     firing RAISES a temperature, so this loop is reverse acting, and that \
                     must be declared rather than defaulted: add `action = \"reverse\"` \
                     (docs/DESIGN.md §22 fork 2)",
                    def.name, def.actuator
                )))
                    }
                    (NodeKind::Furnace { .. }, ControlAction::Direct, true) => {
                        return Err(SimError::Scenario(format!(
                            "control loop '{}' declares `action = \"direct\"` on furnace '{}'. A \
                     direct loop's output must LOWER its measurement as it rises, and more \
                     firing raises a temperature: this loop would shut the furnace off the \
                     moment the tank ran cold. Declare `action = \"reverse\"` \
                     (docs/DESIGN.md §22 fork 2)",
                            def.name, def.actuator
                        )))
                    }
                    (NodeKind::Cooler { .. }, ControlAction::Reverse, _) => {
                        return Err(SimError::Scenario(format!(
                            "control loop '{}' declares `action = \"reverse\"` on cooler '{}'. A \
                     reverse loop's output must RAISE its measurement as it rises, and more \
                     cooling lowers a temperature: this loop would cool hardest when the tank \
                     is already too cold. A cooler loop is direct acting \
                     (docs/DESIGN.md §22 fork 2)",
                            def.name, def.actuator
                        )))
                    }
                    _ => {}
                }
                (Actuator::Node(actuator), max_duty, None)
            }
        };

        if let Some((_, owner)) = claimed_actuators.iter().find(|(id, _)| *id == actuator) {
            // Two primaries on one secondary is the same refusal one level up
            // (docs/DESIGN.md §29 fork 1): two writers of one setpoint.
            return Err(SimError::Scenario(match &def.actuator {
                ActuatorDef::Loop(link) => format!(
                    "cascade primaries '{owner}' and '{}' both write the setpoint of loop \
                     '{}'. Two writers of one setpoint have no defined resolution order, \
                     exactly as two loops on one valve have none (docs/DESIGN.md §29 fork 1)",
                    def.name, link.name
                ),
                ActuatorDef::Node(_) => format!(
                    "control loops '{owner}' and '{}' both actuate '{}'. Two writers of one \
                     opening have no defined resolution order, and declaration order is not \
                     one — split-range, override and feedforward control are real and are \
                     deferred together, with an arbitration (docs/DESIGN.md §10)",
                    def.name, def.actuator
                ),
            }));
        }
        claimed_actuators.push((actuator, &def.name));

        // **The setpoint and the gain are resolved HERE, above the algorithm
        // match, and that placement is load bearing rather than tidy**
        // (docs/DESIGN.md §12 fork 3). Both `"p"` and `"pi"` need a gain, so
        // fetching it inside each arm would put the bar→Pa conversion at two
        // sites — and fork 3's whole defence against the factor of 100 000 is that
        // the setpoint's `× 1e5` and the gain's `÷ 1e5` are written as one pair
        // that a reader sees together. One site, both directions, or the trap is
        // reopened by the next person who adds an algorithm.
        //
        // Which key carries each comes from the variable, and BOTH directions of
        // the mismatch are refused. Until a second variable existed the "belongs
        // to the other variable" direction was not a state the format could reach
        // — `deny_unknown_fields` refused such a key as *unknown*, a different
        // message — so by the project's own rule it is new work here rather than
        // existing coverage, and it is two refusals rather than one because
        // `setpoint_*` and `gain_per_*` are two different mistakes in a file.
        //
        // With four variables every loop has six foreign keys, so the refusal
        // is a table walked in one order rather than a pair written per arm — the
        // same four messages each arm used to write, and no arm able to forget
        // one of the newer variable's keys.
        for (present, key, belongs_to, instead) in [
            (
                def.setpoint_m.is_some(),
                "setpoint_m",
                MeasuredVariable::Level,
                variable.setpoint_key(),
            ),
            (
                def.gain_per_m.is_some(),
                "gain_per_m",
                MeasuredVariable::Level,
                variable.gain_key(),
            ),
            (
                def.setpoint_bar.is_some(),
                "setpoint_bar",
                MeasuredVariable::Pressure,
                variable.setpoint_key(),
            ),
            (
                def.gain_per_bar.is_some(),
                "gain_per_bar",
                MeasuredVariable::Pressure,
                variable.gain_key(),
            ),
            (
                def.setpoint_c.is_some(),
                "setpoint_c",
                MeasuredVariable::Temperature,
                variable.setpoint_key(),
            ),
            (
                def.gain_per_k.is_some(),
                "gain_per_k",
                MeasuredVariable::Temperature,
                variable.gain_key(),
            ),
            (
                def.setpoint_kg_per_s.is_some(),
                "setpoint_kg_per_s",
                MeasuredVariable::Flow,
                variable.setpoint_key(),
            ),
            (
                def.gain_per_kg_per_s.is_some(),
                "gain_per_kg_per_s",
                MeasuredVariable::Flow,
                variable.gain_key(),
            ),
        ] {
            if belongs_to != variable {
                refuse_foreign_key(present, &def.name, key, belongs_to, variable, instead)?;
            }
        }
        let (setpoint, gain) = match variable {
            MeasuredVariable::Level => {
                let setpoint = declared_value(
                    variable,
                    require_keyed(
                        def.setpoint_m,
                        &def.name,
                        variable.setpoint_key(),
                        "the loop's target",
                    )?,
                );
                // Metres in, metres in the arithmetic: nothing to convert, and
                // that is exactly why the pressure arm below needs a comment.
                let gain =
                    require_keyed(def.gain_per_m, &def.name, variable.gain_key(), "the gain")?;
                (setpoint, gain)
            }
            MeasuredVariable::Pressure => {
                // **The pair.** `ControlledValue::magnitude` returns SI, so the
                // error a controller differences is in Pascals and the gain must
                // be per Pascal. The file declares both in bar; these two
                // conversions are the whole of it, and they are adjacent so that
                // converting one without the other is a visible omission rather
                // than an invisible one. Nothing downstream can catch it: the loop
                // stays stable and is merely mistuned by a factor of 1e5.
                let setpoint = declared_value(
                    variable,
                    require_keyed(
                        def.setpoint_bar,
                        &def.name,
                        variable.setpoint_key(),
                        "the loop's target",
                    )?,
                );
                let gain =
                    require_keyed(def.gain_per_bar, &def.name, variable.gain_key(), "the gain")?
                        / 1e5;
                (setpoint, gain)
            }
            // **Not a pair, and that is the trap** (docs/DESIGN.md §21 fork 5).
            // The setpoint is an absolute temperature and takes the °C → K
            // OFFSET; the gain multiplies a temperature DIFFERENCE, and a
            // difference of 1 °C is 1 K, so it takes nothing. The pressure arm's
            // "convert both at one site" copied here would add 273.15 to the gain.
            MeasuredVariable::Temperature => {
                let setpoint = declared_value(
                    variable,
                    require_keyed(
                        def.setpoint_c,
                        &def.name,
                        variable.setpoint_key(),
                        "the loop's target",
                    )?,
                );
                let gain =
                    require_keyed(def.gain_per_k, &def.name, variable.gain_key(), "the gain")?;
                (setpoint, gain)
            }
            // **Nothing to convert on either side, the first variable so**
            // (docs/DESIGN.md §24 fork 4): the file's kg/s is the engine's kg/s,
            // so the trap the pressure pair and the temperature pair each guard
            // against has no expression here.
            MeasuredVariable::Flow => {
                let setpoint = declared_value(
                    variable,
                    require_keyed(
                        def.setpoint_kg_per_s,
                        &def.name,
                        variable.setpoint_key(),
                        "the loop's target",
                    )?,
                );
                let gain = require_keyed(
                    def.gain_per_kg_per_s,
                    &def.name,
                    variable.gain_key(),
                    "the gain",
                )?;
                (setpoint, gain)
            }
        };
        graph
            .check_setpoint(point, setpoint)
            .map_err(|e| SimError::Scenario(format!("control loop '{}': {e}", def.name)))?;

        let mode = match def.mode.as_str() {
            "auto" => ControlMode::Auto,
            "manual" => ControlMode::Manual,
            other => {
                return Err(SimError::Scenario(format!(
                    "control loop '{}' has unknown mode '{other}' (valid: auto, manual)",
                    def.name
                )))
            }
        };

        let algorithm: Box<dyn Controller> = match def.algorithm.as_str() {
            "p" => {
                // Fork 5's refusal, in the direction that was reachable in M8.2.
                // Its mirror — `integral_time_s` ABSENT with `algorithm = "pi"` —
                // is `require_keyed` in the arm below, and became reachable the
                // moment `"pi"` did, exactly as M8.2's comment predicted.
                if def.integral_time_s.is_some() {
                    return Err(SimError::Scenario(format!(
                        "control loop '{}' sets `integral_time_s` on `algorithm = \"p\"`. A \
                         proportional loop has no integral term to tune, and a tuning \
                         constant no algorithm reads is an authoritative-looking number \
                         nothing consumes",
                        def.name
                    )));
                }
                // Its own reason rather than `deny_unknown_fields`' "unknown key",
                // which is what refused this before M8.3 gave the key a meaning.
                // The distinction is the whole of M8.2's correction 2: this is not
                // a bias a proportional loop could use, it is a MEMORY, and a
                // controller with none cannot be given an initial condition for it.
                if def.initial_output.is_some() {
                    return Err(SimError::Scenario(format!(
                        "control loop '{}' sets `initial_output` on `algorithm = \"p\"`. That \
                         key is the loop's MEMORY, from which a PI controller's integral \
                         term is derived (docs/DESIGN.md §10 fork 5), and `u = K·e` has no \
                         memory for it to be the initial condition of. It is deliberately \
                         not a bias: a manual-reset term would turn a P loop's steady-state \
                         offset into a function of how well the bias was chosen",
                        def.name
                    )));
                }
                Box::new(
                    refinery_solvers::ProportionalController::new(gain).map_err(|e| {
                        SimError::Scenario(format!("control loop '{}': {e}", def.name))
                    })?,
                )
            }
            "pi" => {
                let integral_time_s = require_keyed(
                    def.integral_time_s,
                    &def.name,
                    "integral_time_s",
                    "the integral time",
                )?;
                let initial_output = require_keyed(
                    def.initial_output,
                    &def.name,
                    "initial_output",
                    "the loop's initial memory",
                )?;
                // The setpoint and the measurement standing at load are arguments
                // rather than a later call, so a `PiController` whose memory nobody
                // set is not a value that can exist — fork 5's "no silent zero"
                // holds by construction. An outlet loop's measurement is `None`
                // here, and its memory is PENDING on `initial_output` until the
                // first tick measures the outlet (docs/DESIGN.md §23 fork 3). The range check on `initial_output` lives with the
                // controller, beside the range check `Command::SetValveOpening`
                // applies to the same quantity.
                Box::new(
                    refinery_solvers::PiController::new(
                        gain,
                        integral_time_s,
                        initial_output,
                        measurement,
                        setpoint,
                        action,
                    )
                    .map_err(|e| SimError::Scenario(format!("control loop '{}': {e}", def.name)))?,
                )
            }
            other => {
                return Err(SimError::Scenario(format!(
                    "control loop '{}' selects unknown algorithm '{other}' (valid: p, pi). \
                     Derivative action is deferred: a D term differentiates a measurement \
                     that moves by one solve per tick, and needs a filter and a stated \
                     rule for the setpoint kick before it means anything (docs/DESIGN.md \
                     §10)",
                    def.name
                )))
            }
        };

        // Through the one reader the tick pass and the MANUAL→AUTO transfer use
        // (docs/DESIGN.md §21, site 2). This used to be a match with a `_ => 0.0`
        // arm commented "unreachable" — true while a valve was the only actuator,
        // and a silent zero the moment a second kind was admitted.
        //
        // A cascade primary's position is its secondary's setpoint, which may not
        // be built yet; `link_cascades` seeds it once every loop is. NaN until
        // then, never a plausible zero: a seed that was missed would surface.
        let last_output = match actuator {
            Actuator::Node(_) => graph.actuator_position(actuator, max_duty, None)?,
            Actuator::Loop(_) => f64::NAN,
        };

        graph.add_control(ControlLoop {
            name: def.name.clone(),
            measurement_point: point,
            actuator,
            action,
            max_duty,
            setpoint_range,
            setpoint,
            mode,
            algorithm,
            last_measurement: measurement,
            last_output,
        });
    }
    link_cascades(graph, defs)
}

/// Build the plant's trips from `[[trips]]`, after every node exists (M22,
/// docs/DESIGN.md §26).
///
/// Declaration order is `TripId` order and evaluation order. Each refusal below
/// closes a way a file could declare a trip that would load and then protect
/// nothing, or protect the wrong thing:
///
/// - a furnace's or cooler's OUTLET, absent at load — and a cooler's again
///   whenever it stagnates; a furnace's not since its coil (M34,
///   docs/DESIGN.md §37) — refused by name, each for its own reason (fork 2(c),
///   `docs/DEFERRED.md` E13). The test
///   is the engine's own: `measure` at load, with the empty states and no
///   solution that are the truth there, must return a value — **except a
///   declared pipe's flow** (M33, docs/DESIGN.md §36), which is absent before
///   the first solve and nowhere else, so the trip skips tick 1's pass and
///   compares from tick 2. A kind `measure` cannot answer for at all is refused
///   with `measure`'s own reason, as a loop is. A flow trip may watch ANY
///   declared pipe: the loops' "one of its valve's own two pipes" rule (§24,
///   `docs/DEFERRED.md` E12) is about a loop's sign, and a trip actuates
///   nothing through the pipe it watches,
/// - a missing or unknown `direction`, and a limit key belonging to another
///   variable, or none for this one,
/// - a limit outside its physical range: a level in `[0, height]`, a pressure
///   above zero, a temperature above absolute zero. **Its own range check, not
///   `PlantGraph::check_setpoint`** (fork 6), whose ranges were argued for
///   regulators and whose messages say "setpoint",
/// - an empty `actions` list; an action naming more than one, or none, of
///   `pump`, `valve` and `furnace`; equipment that does not exist, or is not the
///   kind its key says; a relief valve (its opening is its own inlet
///   pressure's); a cooler (`docs/DEFERRED.md` E14: cutting it is the hazard);
///   a valve with no `position` or one outside `[0, 1]`, and a `position` on a
///   pump or a furnace (M32, docs/DESIGN.md §35),
/// - two trips — or one trip twice — giving one valve DIFFERENT safe positions,
///   which would make "the" safe state of that valve two numbers,
/// - a duplicate trip name.
///
/// **A trip and a control loop on one valve are admitted** (fork 5): the trip
/// wins, by forcing the loop to MANUAL when it fires. `docs/DEFERRED.md` E4's
/// refusal is of two REGULATING writers, and a trip is not one.
fn build_trips(
    graph: &mut PlantGraph,
    slate: &Slate,
    defs: &[TripDef],
    pipes: &[PipeDef],
) -> Result<(), SimError> {
    let mut seen_names: Vec<&str> = Vec::new();
    // (valve, safe position, trip name) across every trip, for the conflict
    // refusal.
    let mut valve_positions: Vec<(NodeId, f64, &str)> = Vec::new();

    for def in defs {
        let owner = format!("trip '{}'", def.name);
        if seen_names.contains(&def.name.as_str()) {
            return Err(SimError::Scenario(format!(
                "two trips are called '{}'. A trip's name is what a snapshot labels it with, \
                 so two of them make it ambiguous",
                def.name
            )));
        }
        seen_names.push(&def.name);

        let variable = match def.measurement.variable.as_str() {
            "level" => MeasuredVariable::Level,
            "pressure" => MeasuredVariable::Pressure,
            "temperature" => MeasuredVariable::Temperature,
            "flow" => MeasuredVariable::Flow,
            other => {
                return Err(SimError::Scenario(format!(
                    "{owner} measures unknown variable '{other}' (valid: level, pressure, \
                     temperature, flow)"
                )))
            }
        };

        let point = resolve_measurement_point(graph, &owner, &def.measurement, pipes)?;
        let point_name = graph.point_name(point).to_owned();
        // The admission test is the engine's own reader, called with what is
        // true at load. `Err` is a point that cannot answer for the variable at
        // all, in `measure`'s own words; `Ok(None)` is a quantity that exists
        // only once the plant has run. For a pipe's flow that absence ends with
        // the first solve, and the trip pass skips exactly that one pass (§36
        // fork 1). For a COOLER's outlet it does not end there: a stagnant
        // cooler's outlet is absent mid-run, which is exactly when a safety
        // function would need it. A FURNACE's ends with the first tick since its
        // coil (§37), but no rule admits it yet. Both stay refused (E13).
        let measured = graph
            .measure(
                slate,
                &refinery_core::energy::NodeStates::default(),
                None,
                point,
                variable,
            )
            .map_err(|e| {
                SimError::Scenario(format!(
                    "{owner} cannot measure {} on '{point_name}': {e}",
                    variable.noun()
                ))
            })?;
        if measured.is_none() && !matches!(point, MeasurementPoint::Pipe(_)) {
            let furnace = matches!(
                point,
                MeasurementPoint::Node(n) if matches!(graph.node(n).kind, NodeKind::Furnace { .. })
            );
            return Err(SimError::Scenario(if furnace {
                format!(
                    "{owner} watches the {} of '{point_name}', a furnace's outlet, which \
                     does not exist before the first tick: it is resolved by the tick. Since \
                     its coil (docs/DESIGN.md §37) it exists on every tick after, flowing or \
                     not, but a trip on it is not admitted yet (docs/DEFERRED.md E13). Watch \
                     the flow through the unit, or the holdup the stream runs into, instead",
                    variable.noun()
                )
            } else {
                format!(
                    "{owner} watches the {} of '{point_name}', which does not exist before \
                     the first tick, and not while the unit is stagnant either: a cooler's \
                     outlet is resolved by the tick, from its inflow. For a safety function \
                     a missing measurement is not something to hold still on \
                     (docs/DESIGN.md §26 fork 2), and this one goes missing exactly when the \
                     flow stops, so an outlet trip is deferred until it has a stated rule for \
                     that (docs/DEFERRED.md E13). Watch the holdup the stream runs into, or \
                     the flow through the unit, instead",
                    variable.noun()
                )
            }));
        }

        let direction = match def.direction.as_deref() {
            Some("high") => TripDirection::High,
            Some("low") => TripDirection::Low,
            Some(other) => {
                return Err(SimError::Scenario(format!(
                    "{owner} declares unknown direction '{other}' (valid: high, low)"
                )))
            }
            None => {
                return Err(SimError::Scenario(format!(
                    "{owner} declares no `direction`. \"high\" fires at or above the limit and \
                     \"low\" at or below it; an overfill trip and a low-level trip on one tank \
                     differ only in this word, so it has no default"
                )))
            }
        };

        // The limit: one key per variable, the loops' setpoint keys with
        // `limit_` in place of `setpoint_`, and converted by `declared_value`,
        // the same code that converts a setpoint (fork 6).
        let limit_key = |v: MeasuredVariable| match v {
            MeasuredVariable::Level => "limit_m",
            MeasuredVariable::Pressure => "limit_bar",
            MeasuredVariable::Temperature => "limit_c",
            MeasuredVariable::Flow => "limit_kg_per_s",
        };
        for (present, belongs_to) in [
            (def.limit_m.is_some(), MeasuredVariable::Level),
            (def.limit_bar.is_some(), MeasuredVariable::Pressure),
            (def.limit_c.is_some(), MeasuredVariable::Temperature),
            (def.limit_kg_per_s.is_some(), MeasuredVariable::Flow),
        ] {
            if present && belongs_to != variable {
                return Err(SimError::Scenario(format!(
                    "{owner} watches a {} and declares `{}`, which is a {} trip's limit. A \
                     limit carries its variable's unit in its key; write `{}` instead",
                    variable.noun(),
                    limit_key(belongs_to),
                    belongs_to.noun(),
                    limit_key(variable)
                )));
            }
        }
        let declared = match variable {
            MeasuredVariable::Level => def.limit_m,
            MeasuredVariable::Pressure => def.limit_bar,
            MeasuredVariable::Temperature => def.limit_c,
            MeasuredVariable::Flow => def.limit_kg_per_s,
        }
        .ok_or_else(|| {
            SimError::Scenario(format!(
                "{owner} watches a {} and declares no `{}`. A trip's limit has no default",
                variable.noun(),
                limit_key(variable)
            ))
        })?;
        let limit = declared_value(variable, declared);
        check_trip_limit(graph, &owner, point, limit)?;

        if def.actions.is_empty() {
            return Err(SimError::Scenario(format!(
                "{owner} declares no `actions`. A trip that fires and moves nothing protects \
                 nothing; name at least one `{{ pump = \"…\" }}`, \
                 `{{ valve = \"…\", position = … }}` or `{{ furnace = \"…\" }}`"
            )));
        }
        let mut actions = Vec::with_capacity(def.actions.len());
        for action in &def.actions {
            let named: Vec<(&str, &String)> = [
                ("pump", &action.pump),
                ("valve", &action.valve),
                ("furnace", &action.furnace),
            ]
            .into_iter()
            .filter_map(|(key, name)| name.as_ref().map(|name| (key, name)))
            .collect();
            let (key, name) = match named.as_slice() {
                [one] => *one,
                [] => {
                    return Err(SimError::Scenario(format!(
                        "{owner} has an action naming none of a `pump`, a `valve` or a \
                         `furnace`"
                    )))
                }
                _ => {
                    return Err(SimError::Scenario(format!(
                        "{owner} has an action naming more than one of a `pump`, a `valve` and \
                         a `furnace`. Each action names ONE piece of equipment; list them as \
                         separate actions"
                    )))
                }
            };
            let node = graph.find_node(name).ok_or_else(|| {
                SimError::Scenario(format!("{owner} acts on unknown node '{name}'"))
            })?;
            let built = match (key, &graph.node(node).kind) {
                // Its own reason, as `Command::SetValveOpening` refuses it: a
                // relief valve's opening is a function of its own inlet pressure,
                // recomputed every solve, so a trip could not hold it anywhere.
                (_, NodeKind::ReliefValve { .. }) => {
                    return Err(SimError::Scenario(format!(
                        "{owner} acts on '{name}', a relief valve. Its opening is actuated by \
                         its own inlet pressure and recomputed on every solve, so a trip could \
                         not hold it in a safe state"
                    )))
                }
                (_, NodeKind::CheckValve { .. }) => {
                    return Err(SimError::Scenario(format!(
                        "{owner} acts on '{name}', a check valve. Its disc is moved by the \
                         forward drive across it and recomputed on every solve, so a trip \
                         could not hold it in a safe state"
                    )))
                }
                // Its own reason (M32, docs/DESIGN.md §35 fork 1): a cooler has
                // no safe state a trip could write. Cutting it is LOSING cooling,
                // the failure a trip is meant to answer, and running it flat out is
                // a regulator's job done at the wrong layer.
                (_, NodeKind::Cooler { .. }) => {
                    return Err(SimError::Scenario(format!(
                        "{owner} acts on '{name}', a cooler. A cooler has no safe state a trip \
                         could write: cutting its duty is losing cooling, which is the hazard \
                         rather than the protection. A cooler trip is deferred with its own \
                         trigger (docs/DEFERRED.md E14)"
                    )))
                }
                ("furnace", NodeKind::Furnace { .. }) => {
                    if action.position.is_some() {
                        return Err(SimError::Scenario(format!(
                            "{owner} gives furnace '{name}' a `position`. A furnace's safe state \
                             is its fuel cut, zero duty, and it has no position to hold"
                        )));
                    }
                    TripAction::CutFurnace { furnace: node }
                }
                ("pump", NodeKind::Pump { .. }) => {
                    if action.position.is_some() {
                        return Err(SimError::Scenario(format!(
                            "{owner} gives pump '{name}' a `position`. A pump's safe state is \
                             stopped, and it has no position to hold"
                        )));
                    }
                    TripAction::StopPump { pump: node }
                }
                ("valve", NodeKind::Valve { .. }) => {
                    let position = action.position.ok_or_else(|| {
                        SimError::Scenario(format!(
                            "{owner} names valve '{name}' with no `position`. Most trips shut a \
                             valve, but a vent or dump valve trips OPEN, so the file says which; \
                             there is no default"
                        ))
                    })?;
                    if !position.is_finite() || !(0.0..=1.0).contains(&position) {
                        return Err(SimError::Scenario(format!(
                            "{owner} gives valve '{name}' `position = {position}`, outside \
                             [0, 1]: a valve's opening is a fraction"
                        )));
                    }
                    if let Some((_, other, other_trip)) = valve_positions
                        .iter()
                        .find(|(v, p, _)| *v == node && *p != position)
                    {
                        return Err(SimError::Scenario(format!(
                            "{owner} puts valve '{name}' at {position}, and trip '{other_trip}' \
                             puts it at {other}. Two latched trips on one valve must demand the \
                             same safe state, or the valve has no single one"
                        )));
                    }
                    valve_positions.push((node, position, &def.name));
                    TripAction::SetValve {
                        valve: node,
                        position,
                    }
                }
                (key, _) => {
                    return Err(SimError::Scenario(format!(
                        "{owner} names '{name}' under `{key}`, and it is not a {key}. An action \
                         names its equipment under the key for its kind: `pump` for a pump, \
                         `valve` for a valve, `furnace` for a furnace"
                    )))
                }
            };
            actions.push(built);
        }

        graph.add_trip(Trip {
            name: def.name.clone(),
            measurement_point: point,
            direction,
            limit,
            actions,
            state: TripState::Armed,
            // No trip pass has run: absent until tick 1 (fork 8).
            last_measurement: None,
        });
    }
    Ok(())
}

/// A trip limit's physical range (docs/DESIGN.md §26 fork 6).
///
/// Only the physical bounds: a level in `[0, height]`, a pressure above zero,
/// a temperature above absolute zero, a flow any finite number (M33), each
/// finite. A limit AT a bound is legal — a high level trip at the brim is a real
/// design — and a limit that fires at load is legal too: the plant trips on
/// tick 1 rather than running a tick in a condition its own file calls unsafe.
///
/// **A flow's limit has no sign bound** (docs/DESIGN.md §36 fork 2). The flow is
/// signed by the pipe's declared direction and measured as solved, never
/// clipped (§24, E11), so a low trip at or below zero is a reverse-flow trip and
/// a real design.
fn check_trip_limit(
    graph: &PlantGraph,
    owner: &str,
    point: MeasurementPoint,
    limit: ControlledValue,
) -> Result<(), SimError> {
    let node = match (point, limit) {
        (MeasurementPoint::Node(node), _) => node,
        (MeasurementPoint::Pipe(pipe), ControlledValue::Flow { kg_per_s }) => {
            if !kg_per_s.value().is_finite() {
                return Err(SimError::Scenario(format!(
                    "{owner} has a trip limit of {} kg/s on pipe '{}', which is not a number \
                     a flow can be compared with",
                    kg_per_s.value(),
                    graph.pipe(pipe).name
                )));
            }
            return Ok(());
        }
        // `measure` admits only a flow on a pipe, and refused this pairing
        // before the limit was built.
        (MeasurementPoint::Pipe(pipe), other) => {
            return Err(SimError::Scenario(format!(
                "{owner} has a {:?} trip limit on pipe '{}', which carries only a flow",
                other.variable(),
                graph.pipe(pipe).name
            )))
        }
    };
    let name = &graph.node(node).name;
    let (value, ok, range) = match (limit, &graph.node(node).kind) {
        (ControlledValue::Level { m }, NodeKind::Tank(t)) => (
            m.value(),
            m.value() >= 0.0 && m.value() <= t.height.value(),
            format!("[0, {}] m, the tank's height", t.height.value()),
        ),
        (ControlledValue::Pressure { pa }, _) => {
            (pa.value(), pa.value() > 0.0, "above 0 Pa".to_string())
        }
        (ControlledValue::Temperature { k }, _) => {
            (k.value(), k.value() > 0.0, "above 0 K".to_string())
        }
        // `measure` admitted the pairing, so a level limit is on a tank.
        (other, _) => {
            return Err(SimError::Scenario(format!(
                "{owner} has a {:?} trip limit on '{name}', which cannot carry one",
                other.variable()
            )))
        }
    };
    if !value.is_finite() || !ok {
        return Err(SimError::Scenario(format!(
            "{owner} has a trip limit of {value} (in SI) on '{name}', outside its range {range}"
        )));
    }
    Ok(())
}

/// Where a `[[controls]]` entry measures: a node by name, or a pipe by name among
/// the file's DECLARED `[[pipes]]` (M20, docs/DESIGN.md §24 fork 1).
///
/// `owner` is how a refusal names the entry — `control loop 'x'` or, since M22,
/// `trip 'x'` — so a `[[trips]]` measurement is resolved by the same rules and
/// refused in the same words (docs/DESIGN.md §26 fork 2).
///
/// Which point may carry which variable is not decided here — `PlantGraph::measure`
/// owns that, and refuses a flow asked of a node or a level asked of a pipe with a
/// message naming the other key. What is decided here is only what the NAME
/// refers to:
///
/// - **Exactly one of `node` and `pipe`.** Both, or neither, is refused.
/// - **A pipe is looked up among the declared pipes, not the graph's edges.** The
///   graph also holds edges the loader made — a leak split's `__downstream` half and
///   `__leak` orifice, every boil-off vent — and a file naming one would be metering
///   a pipe it never wrote. Pipe names are not otherwise required to be unique, so a
///   name declared twice is refused as an ambiguous meter rather than resolved to
///   whichever came first.
/// - **A pipe that declares `leak_to` is refused, with its own reason, before any
///   adjacency check** (§24 fork 5): the split gives the declared name to the
///   UPSTREAM half, which ends at the leak junction and not at the valve, and once
///   punctured the two halves carry different flows. The message names the leak
///   rather than "not on the valve". Since `Command::PuncturePipe` acts only on a
///   pipe the loader split, a pipe that passes here can never be punctured mid-run.
/// - The graph edge is then found by name AND declared endpoints AND an ordinary
///   leak role, so a loader-made edge that happened to share the name could not be
///   picked up in its place.
fn resolve_measurement_point(
    graph: &PlantGraph,
    owner: &str,
    measurement: &MeasurementDef,
    pipes: &[PipeDef],
) -> Result<MeasurementPoint, SimError> {
    match (&measurement.node, &measurement.pipe) {
        (Some(node), Some(pipe)) => Err(SimError::Scenario(format!(
            "{owner} names both `node = \"{node}\"` and `pipe = \"{pipe}\"` in \
             `measurement`. A loop measures at ONE point: a node for a level, pressure or \
             temperature, a pipe for a flow (docs/DESIGN.md §24 fork 1)"
        ))),
        (None, None) => Err(SimError::Scenario(format!(
            "{owner} names neither `node` nor `pipe` in `measurement`. A level, \
             pressure or temperature is measured at a `node`, a flow on a `pipe` \
             (docs/DESIGN.md §24 fork 1)"
        ))),
        (Some(node), None) => graph
            .find_node(node)
            .map(MeasurementPoint::Node)
            .ok_or_else(|| SimError::Scenario(format!("{owner} measures unknown node '{node}'"))),
        (None, Some(name)) => {
            let declared: Vec<&PipeDef> = pipes.iter().filter(|p| p.name == *name).collect();
            let pipe = match declared.as_slice() {
                [] => {
                    return Err(SimError::Scenario(format!(
                        "{owner} measures pipe '{name}', which this file does not \
                         declare. A flow is metered only on a pipe the file wrote in `[[pipes]]`: the \
                         loader also makes edges of its own — a leak split's `__downstream` \
                         half and `__leak` orifice, and every boil-off vent — and those are \
                         not meters (docs/DESIGN.md §24 fork 1)"
                    )))
                }
                [one] => *one,
                _ => {
                    return Err(SimError::Scenario(format!(
                        "{owner} measures pipe '{name}', and this file declares {} \
                         pipes by that name, so the meter is ambiguous. Rename one",
                        declared.len()
                    )))
                }
            };
            if let Some(atmosphere) = &pipe.leak_to {
                return Err(SimError::Scenario(format!(
                    "{owner} measures pipe '{name}', which declares `leak_to = \
                     \"{atmosphere}\"`. The loader splits a leaking pipe at a junction and gives \
                     the declared name to the UPSTREAM half, which ends at the leak point — \
                     and once punctured the two halves carry different flows, so the name \
                     no longer names one flow. A flow is metered on a pipe with no leak path \
                     (docs/DESIGN.md §24 fork 5)"
                )));
            }
            let from = graph.find_node(&pipe.from);
            let to = graph.find_node(&pipe.to);
            graph
                .edge_ids()
                .find(|&edge| {
                    let candidate = graph.pipe(edge);
                    candidate.name == *name
                        && candidate.leak == LeakRole::None
                        && Some(graph.endpoints(edge)) == from.zip(to)
                })
                .map(MeasurementPoint::Pipe)
                .ok_or_else(|| {
                    // Every declared pipe without `leak_to` was added whole, under
                    // its own name and endpoints, before this runs; reaching here
                    // is a loader fault, said rather than papered over (rule 5).
                    SimError::Scenario(format!(
                        "{owner} measures declared pipe '{name}', which the \
                         loader did not build as one ordinary edge"
                    ))
                })
        }
    }
}

/// A `[[controls]]` key that is required for this loop's variable or algorithm.
///
/// One helper rather than a refusal per key, so every "you left out the number
/// that decides this loop's behaviour" message says the same thing — and so that
/// no key can acquire a silent default by being forgotten in one branch. `gain`
/// and the integral time have no defaults for the reason `x_T` has none (§3a fork
/// 6): a silent default is an invented value in disguise, and every gate would
/// then pass for whatever was chosen.
/// A `[[controls]]` key that belongs to the OTHER variable (docs/DESIGN.md §12
/// fork 6).
///
/// Two directions and two keys, so four messages, and one helper rather than four
/// literals — every one of them has to name the key that was written, the
/// variable it belongs to, the variable this loop actually measures, and the key
/// that was meant. A file gets the correction, not just the rejection.
///
/// Separate from `deny_unknown_fields`, which is what refused these before a
/// second variable existed: "unknown key" and "that is the pressure loop's key"
/// are different mistakes, and telling an author the first when the second is
/// true sends them looking for a typo they did not make.
/// The variable a `[[controls]]` entry measures, from its `measurement.variable`.
///
/// One function because a cascade primary reads its SECONDARY's variable too,
/// before that loop is built, to know which unit its range keys carry.
fn parse_variable(def: &ControlDef) -> Result<MeasuredVariable, SimError> {
    match def.measurement.variable.as_str() {
        "level" => Ok(MeasuredVariable::Level),
        "pressure" => Ok(MeasuredVariable::Pressure),
        "temperature" => Ok(MeasuredVariable::Temperature),
        // This message used to say flow control was deferred because "a flow
        // lives on an EDGE, which nothing in `measure`'s signature can name".
        // M20 names it: `MeasurementPoint` (docs/DESIGN.md §24 fork 1).
        "flow" => Ok(MeasuredVariable::Flow),
        other => Err(SimError::Scenario(format!(
            "control loop '{}' measures unknown variable '{other}' (valid: level, pressure, \
             temperature, flow)",
            def.name
        ))),
    }
}

/// The four range keys, with the variable each one's unit belongs to.
fn range_keys(def: &ControlDef) -> [(Option<f64>, &'static str, MeasuredVariable); 4] {
    [
        (
            def.range_min_c,
            "range_min_c",
            MeasuredVariable::Temperature,
        ),
        (
            def.range_max_c,
            "range_max_c",
            MeasuredVariable::Temperature,
        ),
        (
            def.range_min_kg_per_s,
            "range_min_kg_per_s",
            MeasuredVariable::Flow,
        ),
        (
            def.range_max_kg_per_s,
            "range_max_kg_per_s",
            MeasuredVariable::Flow,
        ),
    ]
}

/// A range key on a loop that writes a NODE is refused (M25, docs/DESIGN.md §29
/// fork 2) — the `max_duty_mw` rule, the other direction.
fn refuse_range_keys_on_node(def: &ControlDef) -> Result<(), SimError> {
    if let Some((_, key, _)) = range_keys(def).into_iter().find(|(v, _, _)| v.is_some()) {
        return Err(SimError::Scenario(format!(
            "control loop '{}' declares `{key}` and actuates node '{}'. A setpoint range is a \
             cascade PRIMARY's authority over another loop's setpoint (`actuator = {{ loop = \
             \"…\" }}`); a loop writing equipment would read it nowhere (docs/DESIGN.md §29 \
             fork 2)",
            def.name, def.actuator
        )));
    }
    Ok(())
}

/// The refusal for a cascade whose inner loop measures a HOLDUP (docs/DESIGN.md
/// §29 fork 5): a level, a pressure, or a tank's or vessel's temperature.
fn holdup_inner_refusal(primary: &str, secondary: &str, noun: &str) -> SimError {
    SimError::Scenario(format!(
        "cascade primary '{primary}' drives loop '{secondary}', which measures a holdup's \
         {noun}. An inner loop must measure a quantity with no holdup of its own — a pipe's \
         flow, or a furnace's or cooler's outlet — because that is what makes it fast enough \
         to be worth separating: both settle within the tick (docs/DESIGN.md §29 fork 5)"
    ))
}

/// A cascade primary's keys, checked against its secondary's DECLARATION (M25,
/// docs/DESIGN.md §29 fork 2): which loop it drives, and its range in that loop's
/// unit.
///
/// The secondary's `LoopId` is its position in `defs`, which is the id
/// `add_control` gives it — every entry is added once, in order, or the load
/// fails. `link_cascades` asserts that before reading any link.
fn cascade_link_keys(
    def: &ControlDef,
    defs: &[ControlDef],
    secondary_name: &str,
) -> Result<(LoopId, SetpointRange), SimError> {
    let index = defs
        .iter()
        .position(|d| d.name == secondary_name)
        .ok_or_else(|| {
            SimError::Scenario(format!(
                "control loop '{}' drives unknown loop '{secondary_name}'. `actuator = {{ loop = \
                 \"…\" }}` names another `[[controls]]` entry by its `name`",
                def.name
            ))
        })?;
    if def.max_duty_mw.is_some() {
        return Err(SimError::Scenario(format!(
            "control loop '{}' declares `max_duty_mw` and drives loop '{secondary_name}'. That key \
             is a DUTY actuator's range; a cascade primary's authority is a range of its \
             secondary's setpoints (`range_min_*`/`range_max_*`, docs/DESIGN.md §29 fork 2)",
            def.name
        )));
    }
    let secondary_variable = parse_variable(&defs[index])?;
    let (min_key, max_key, min, max) = match secondary_variable {
        MeasuredVariable::Temperature => (
            "range_min_c",
            "range_max_c",
            def.range_min_c,
            def.range_max_c,
        ),
        MeasuredVariable::Flow => (
            "range_min_kg_per_s",
            "range_max_kg_per_s",
            def.range_min_kg_per_s,
            def.range_max_kg_per_s,
        ),
        MeasuredVariable::Level | MeasuredVariable::Pressure => {
            return Err(holdup_inner_refusal(
                &def.name,
                secondary_name,
                secondary_variable.noun(),
            ))
        }
    };
    for (value, key, belongs_to) in range_keys(def) {
        if value.is_some() && belongs_to != secondary_variable {
            return Err(SimError::Scenario(format!(
                "control loop '{}' declares `{key}`, a range over a {} secondary, and drives \
                 loop '{secondary_name}', which measures a {}. The range carries its \
                 SECONDARY's unit: write `{min_key}` and `{max_key}` (docs/DESIGN.md §29 fork \
                 2)",
                def.name,
                belongs_to.noun(),
                secondary_variable.noun()
            )));
        }
    }
    let what = "the secondary setpoint the primary's output stands for at that end";
    let min = require_keyed(min, &def.name, min_key, what)?;
    let max = require_keyed(max, &def.name, max_key, what)?;
    let range = SetpointRange {
        min: declared_value(secondary_variable, min),
        max: declared_value(secondary_variable, max),
    };
    // Compared AFTER conversion, so a range end that missed its conversion — the
    // `+ 273.15` on one temperature key and not the other — is caught here as
    // an empty range rather than shipped as one 300 K wide (§29 mutation 7).
    let (lo, hi) = (range.min.magnitude(), range.max.magnitude());
    if !lo.is_finite() || !hi.is_finite() || lo >= hi {
        return Err(SimError::Scenario(format!(
            "control loop '{}' declares `{min_key} = {min}` and `{max_key} = {max}`. A cascade \
             primary's range must be finite with its bottom strictly below its top: its output \
             is a fraction of that span, and an empty or inverted span is a primary with no \
             authority or with its sign hidden in a range (docs/DESIGN.md §29 fork 2)",
            def.name
        )));
    }
    Ok((LoopId(index as u32), range))
}

/// Check every cascade link once all loops exist, and seed each primary's
/// faceplate from its secondary's setpoint (M25, docs/DESIGN.md §29).
///
/// After the build rather than inside it, because a file may declare the
/// secondary after its primary — and a fixture does, since a demo's own file
/// order makes the pass-2 partition inert (§29 fork 3). In order, for each
/// primary: the depth rule, which is also the cycle refusal (a self-link, a
/// mutual pair and a three-deep chain each with its own message); the pairing
/// and the primary's sign (fork 5); both range ends through the secondary's own
/// `check_setpoint`; the secondary's declared setpoint inside the range; and the
/// seed.
fn link_cascades(graph: &mut PlantGraph, defs: &[ControlDef]) -> Result<(), SimError> {
    if graph.controls().len() != defs.len() {
        return Err(SimError::Scenario(format!(
            "internal: {} `[[controls]]` entries built {} loops, so a loop's position in the \
             file is not its id and no cascade link can be resolved",
            defs.len(),
            graph.controls().len()
        )));
    }
    for (index, def) in defs.iter().enumerate() {
        let primary_id = LoopId(index as u32);
        let primary = &graph.controls()[index];
        let Actuator::Loop(secondary_id) = primary.actuator else {
            continue;
        };
        let range = primary.setpoint_range.ok_or_else(|| {
            SimError::Scenario(format!(
                "internal: cascade primary '{}' was built with no setpoint range",
                primary.name
            ))
        })?;
        if secondary_id == primary_id {
            return Err(SimError::Scenario(format!(
                "control loop '{}' drives itself. A loop whose output is its own setpoint \
                 regulates nothing — and a loop that drives another may not itself be driven, \
                 which is the rule that refuses this (docs/DESIGN.md §29 fork 3)",
                primary.name
            )));
        }
        let secondary = graph.control(secondary_id).ok_or_else(|| {
            SimError::Scenario(format!(
                "internal: cascade primary '{}' drives {secondary_id:?}, which names no loop",
                primary.name
            ))
        })?;
        if let Actuator::Loop(third) = secondary.actuator {
            if third == primary_id {
                return Err(SimError::Scenario(format!(
                    "control loops '{}' and '{}' drive each other. Each would write the other's \
                     setpoint, which is a cycle with no answer — refused by the depth rule: a \
                     loop that drives another may not itself be driven (docs/DESIGN.md §29 fork \
                     3)",
                    primary.name, secondary.name
                )));
            }
            return Err(SimError::Scenario(format!(
                "control loop '{}' drives loop '{}', which itself drives another loop: a chain \
                 three deep. Cascades are two levels only — a loop that drives another may not \
                 itself be driven (docs/DESIGN.md §29 fork 3, docs/DEFERRED.md E17)",
                primary.name, secondary.name
            )));
        }
        if let Some(driver) = graph.primary_of(primary_id) {
            return Err(SimError::Scenario(format!(
                "control loop '{}' drives loop '{}' and is itself driven by '{}': a chain three \
                 deep. Cascades are two levels only — a loop that drives another may not itself \
                 be driven (docs/DESIGN.md §29 fork 3, docs/DEFERRED.md E17)",
                primary.name, secondary.name, driver.name
            )));
        }
        let required = cascade_pairing(graph, primary, secondary)?;
        match (def.action.is_some(), primary.action == required) {
            (false, _) => {
                return Err(SimError::Scenario(format!(
                    "cascade primary '{}' declares no `action`. Its sign is topology — which \
                     way a higher target for '{}' moves its own measurement — and the loader \
                     checked it in one hop: this primary is {} acting, which must be declared \
                     rather than defaulted. Add `action = \"{}\"` (docs/DESIGN.md §29 fork 5)",
                    primary.name,
                    secondary.name,
                    action_word(required),
                    action_word(required)
                )))
            }
            (true, false) => {
                return Err(SimError::Scenario(format!(
                    "cascade primary '{}' declares `action = \"{}\"`, and the plant makes it {} \
                     acting: a higher target for '{}' {} its measurement. With the wrong sign \
                     the primary would push its secondary the wrong way and run away. Declare \
                     `action = \"{}\"` (docs/DESIGN.md §29 fork 5)",
                    primary.name,
                    action_word(primary.action),
                    action_word(required),
                    secondary.name,
                    if required == ControlAction::Reverse {
                        "RAISES"
                    } else {
                        "LOWERS"
                    },
                    action_word(required)
                )))
            }
            (true, true) => {}
        }
        for (end, value) in [("bottom", range.min), ("top", range.max)] {
            graph
                .check_setpoint(secondary.measurement_point, value)
                .map_err(|e| {
                    SimError::Scenario(format!(
                        "cascade primary '{}': the {end} of its range is a setpoint loop '{}' \
                         would refuse, so the primary could reach a position the engine then \
                         refuses mid-tick (docs/DESIGN.md §29 fork 2): {e}",
                        primary.name, secondary.name
                    ))
                })?;
        }
        // Not in the note, and owed by its own fork 4: a cascade OPEN at load —
        // the demo's tick 1, whose outlet does not exist yet — re-seeds the
        // primary against this position, and a position outside [0, 1] is one
        // the seed refuses. So the tick would fail, not the load.
        if !range.contains(secondary.setpoint) {
            return Err(SimError::Scenario(format!(
                "loop '{}' declares setpoint {:?}, outside the range of its cascade primary \
                 '{}' ({:?} to {:?}). The primary's position IS that setpoint as a fraction of \
                 its range, so a setpoint outside it is a position the primary could never have \
                 produced (docs/DESIGN.md §29 fork 2)",
                secondary.name, secondary.setpoint, primary.name, range.min, range.max
            )));
        }
        let position = graph.actuator_position(primary.actuator, None, Some(range))?;
        graph
            .control_mut(primary_id)
            .ok_or_else(|| SimError::Scenario(format!("internal: {primary_id:?} names no loop")))?
            .last_output = position;
    }
    Ok(())
}

/// Which side of a holdup a valve sits on, judged in ONE hop by declared pipe
/// direction (docs/DESIGN.md §32 fork 1).
///
/// `validate_degrees` holds every valve to exactly one inlet and one outlet edge,
/// so a valve whose inlet pipe starts at the holdup is the whole of a drain, and
/// one whose outlet pipe ends there is the whole of a fill. **The single owner of
/// "drain or fill"**: a level or pressure loop's own sign (M29) and a cascade
/// primary's sign over a flow loop (§29 fork 5) both read it, so the two cannot
/// disagree about one valve.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ValveSide {
    /// The valve's inlet pipe starts at the holdup: opening it lowers the holdup.
    Drain,
    /// The valve's outlet pipe ends at the holdup: opening it raises the holdup.
    Fill,
    /// Neither pipe touches the holdup; the sign would need a walk (E21).
    Neither,
    /// Both do (a valve recycling the holdup to itself); no sign exists.
    Both,
}

fn valve_side(graph: &PlantGraph, holdup: NodeId, valve: NodeId) -> ValveSide {
    let one_hop =
        |from: NodeId, to: NodeId| graph.edge_ids().any(|e| graph.endpoints(e) == (from, to));
    match (one_hop(holdup, valve), one_hop(valve, holdup)) {
        (true, false) => ValveSide::Drain,
        (false, true) => ValveSide::Fill,
        (false, false) => ValveSide::Neither,
        (true, true) => ValveSide::Both,
    }
}

/// **A level or pressure loop's direction of action, checked against where its
/// valve sits** (M29, docs/DESIGN.md §32 fork 2; this was `docs/DEFERRED.md` E8).
///
/// A valve's sign on a holdup is topology — the same valve is direct on a drain
/// and reverse on a fill — so the loader reads the side through `valve_side` and
/// holds the declaration to it, BOTH ways. Before M29 only `"reverse"` was
/// refused and the default went unchecked, so a direct loop on a fill valve (a
/// runaway: it opens wider the fuller the tank) loaded and ran:
///
/// | side | absent | `"direct"` | `"reverse"` |
/// |---|---|---|---|
/// | drain | admitted | admitted | refused |
/// | fill | refused: declare it | refused | admitted |
/// | both, neither | refused | refused | refused |
///
/// A fill's action must be DECLARED, as a furnace's and a flow loop's must: the
/// default is a true statement about a drain and a false one about a fill.
fn check_holdup_valve_action(
    def: &ControlDef,
    variable: MeasuredVariable,
    holdup_name: &str,
    side: ValveSide,
    action: ControlAction,
) -> Result<(), SimError> {
    let noun = variable.noun();
    let refusal = match (side, action, def.action.is_some()) {
        (ValveSide::Drain, ControlAction::Direct, _)
        | (ValveSide::Fill, ControlAction::Reverse, _) => return Ok(()),
        (ValveSide::Drain, ControlAction::Reverse, _) => format!(
            "control loop '{}' declares `action = \"reverse\"` on valve '{}', which DRAINS \
             '{holdup_name}' (its inlet pipe starts there). Opening a drain lowers the {noun}, \
             so this loop is direct acting: it would shut the valve the moment the {noun} ran \
             high. Remove the key or declare `action = \"direct\"` (docs/DESIGN.md §32 fork 2)",
            def.name, def.actuator
        ),
        (ValveSide::Fill, ControlAction::Direct, false) => format!(
            "control loop '{}' holds the {noun} of '{holdup_name}' with valve '{}', which FILLS \
             it (its outlet pipe ends there), and declares no `action`. Opening a fill valve \
             RAISES the {noun}, so this loop is reverse acting, and that must be declared \
             rather than defaulted: add `action = \"reverse\"` (docs/DESIGN.md §32 fork 2)",
            def.name, def.actuator
        ),
        (ValveSide::Fill, ControlAction::Direct, true) => format!(
            "control loop '{}' declares `action = \"direct\"` on valve '{}', which FILLS \
             '{holdup_name}' (its outlet pipe ends there). A direct loop's output must LOWER \
             its measurement as it rises, and opening a fill valve raises the {noun}: this \
             loop would open the valve wider the higher the {noun} ran. Declare `action = \
             \"reverse\"` (docs/DESIGN.md §32 fork 2)",
            def.name, def.actuator
        ),
        (ValveSide::Both, _, _) => format!(
            "control loop '{}' holds the {noun} of '{holdup_name}' with valve '{}', which both \
             drains it and fills it, so opening it moves the {noun} neither way the loader can \
             name (docs/DESIGN.md §32 fork 1)",
            def.name, def.actuator
        ),
        (ValveSide::Neither, _, _) => format!(
            "control loop '{}' holds the {noun} of '{holdup_name}' with valve '{}', which is \
             neither a drain (its inlet pipe starting at '{holdup_name}') nor a fill (its \
             outlet pipe ending there). A level or pressure loop's sign is checked in ONE hop, \
             and a valve further off is not admitted in either direction of action \
             (docs/DEFERRED.md E21, docs/DESIGN.md §32 fork 1)",
            def.name, def.actuator
        ),
    };
    Err(SimError::Scenario(refusal))
}

fn action_word(action: ControlAction) -> &'static str {
    match action {
        ControlAction::Direct => "direct",
        ControlAction::Reverse => "reverse",
    }
}

/// The admitted cascade pairings, and the sign each one forces on the primary
/// (docs/DESIGN.md §29 fork 5's table). Every other pairing is refused with its
/// own reason.
///
/// The sign is topology — which way a higher secondary setpoint moves the
/// primary's measurement — so a pairing is admitted only where the loader can
/// check it in ONE hop:
///
/// - a tank's temperature over a furnace's or cooler's outlet whose outlet pipe
///   ends at that tank: REVERSE, whichever unit it is (a hotter outlet target
///   heats the tank, and a cooler's hotter target is less cooling);
/// - a tank's level over the flow on a valve whose inlet pipe starts at the tank
///   (a drain): DIRECT;
/// - a tank's level over the flow on a valve whose outlet pipe ends at the tank (a
///   fill): REVERSE — the first reverse level loop on a valve admitted. M29 admits
///   the same side for a single loop, through the same `valve_side`.
fn cascade_pairing(
    graph: &PlantGraph,
    primary: &ControlLoop,
    secondary: &ControlLoop,
) -> Result<ControlAction, SimError> {
    let one_hop =
        |from: NodeId, to: NodeId| graph.edge_ids().any(|e| graph.endpoints(e) == (from, to));
    let outer = primary.setpoint.variable();
    let inner = secondary.setpoint.variable();
    let holdup = match primary.measurement_point {
        MeasurementPoint::Node(node) => Some((node, &graph.node(node).kind)),
        MeasurementPoint::Pipe(_) => None,
    };
    match (secondary.measurement_point, inner) {
        // The inner loop measures a holdup: refused whatever the outer loop is.
        (MeasurementPoint::Node(node), _)
            if matches!(
                graph.node(node).kind,
                NodeKind::Tank(_) | NodeKind::Vessel(_)
            ) =>
        {
            Err(holdup_inner_refusal(
                &primary.name,
                &secondary.name,
                inner.noun(),
            ))
        }
        // A furnace's or cooler's OUTLET.
        (MeasurementPoint::Node(unit), MeasuredVariable::Temperature) => match (outer, holdup) {
            (MeasuredVariable::Temperature, Some((tank, NodeKind::Tank(_)))) => {
                if one_hop(unit, tank) {
                    Ok(ControlAction::Reverse)
                } else {
                    Err(SimError::Scenario(format!(
                        "cascade primary '{}' holds tank '{}' over the outlet of '{}', \
                             whose outlet pipe does not end at that tank. The primary's sign is \
                             checked in ONE hop — the unit's outlet running straight into the \
                             tank — and a unit further off is not admitted (docs/DESIGN.md §29 \
                             fork 5, docs/DEFERRED.md E18)",
                        primary.name,
                        graph.node(tank).name,
                        graph.node(unit).name
                    )))
                }
            }
            (MeasuredVariable::Temperature, Some((vessel, NodeKind::Vessel(_)))) => {
                Err(SimError::Scenario(format!(
                    "cascade primary '{}' holds the temperature of VESSEL '{}' over an \
                         outlet. The same one-hop rule would serve, but no plant or fixture runs \
                         it, so it is not admitted (docs/DEFERRED.md E18, docs/DESIGN.md §29 \
                         fork 5)",
                    primary.name,
                    graph.node(vessel).name
                )))
            }
            _ => Err(not_admitted(graph, primary, secondary)),
        },
        // A pipe's flow, on the secondary's own valve.
        (MeasurementPoint::Pipe(_), MeasuredVariable::Flow) => match (outer, holdup) {
            (MeasuredVariable::Level, Some((tank, NodeKind::Tank(_)))) => {
                let valve = secondary.actuator.node().ok_or_else(|| {
                    SimError::Scenario(format!(
                        "internal: flow loop '{}' writes no valve",
                        secondary.name
                    ))
                })?;
                match valve_side(graph, tank, valve) {
                    ValveSide::Drain => Ok(ControlAction::Direct),
                    ValveSide::Fill => Ok(ControlAction::Reverse),
                    ValveSide::Neither => Err(SimError::Scenario(format!(
                        "cascade primary '{}' holds the level of tank '{}' over the flow \
                         through valve '{}', which is neither a drain (its inlet pipe starting \
                         at the tank) nor a fill (its outlet pipe ending at it). The primary's \
                         sign is checked in ONE hop, and a valve further off is not admitted \
                         (docs/DESIGN.md §29 fork 5, docs/DEFERRED.md E18)",
                        primary.name,
                        graph.node(tank).name,
                        graph.node(valve).name
                    ))),
                    ValveSide::Both => Err(SimError::Scenario(format!(
                        "cascade primary '{}' holds the level of tank '{}' over the flow \
                         through valve '{}', which both drains the tank and fills it, so \
                         opening it moves the level neither way the loader can name \
                         (docs/DESIGN.md §29 fork 5)",
                        primary.name,
                        graph.node(tank).name,
                        graph.node(valve).name
                    ))),
                }
            }
            (MeasuredVariable::Pressure, _) => Err(SimError::Scenario(format!(
                "cascade primary '{}' holds a pressure over the flow of loop '{}'. The same \
                 one-hop rule would serve a vent's flow, but no plant or fixture runs it, so it \
                 is not admitted (docs/DEFERRED.md E18, docs/DESIGN.md §29 fork 5)",
                primary.name, secondary.name
            ))),
            (MeasuredVariable::Temperature, _) => Err(SimError::Scenario(format!(
                "cascade primary '{}' holds a temperature over the flow of loop '{}'. This \
                 engine has no coolant stream — a cooler is a duty with no coolant side — so a \
                 flow moves a temperature only by changing a PROCESS flow, whose sign depends on \
                 the plant (docs/DESIGN.md §21 fork 3, §29 fork 5, docs/DEFERRED.md E18)",
                primary.name, secondary.name
            ))),
            _ => Err(not_admitted(graph, primary, secondary)),
        },
        _ => Err(not_admitted(graph, primary, secondary)),
    }
}

/// Every cross pairing fork 5 does not admit and does not name: a level over a
/// temperature, a pressure over an outlet, a primary on a pipe or an outlet.
fn not_admitted(graph: &PlantGraph, primary: &ControlLoop, secondary: &ControlLoop) -> SimError {
    SimError::Scenario(format!(
        "cascade primary '{}' holds the {} of '{}' over loop '{}', which holds the {} of '{}'. \
         That is not an admitted cascade pairing: a tank's temperature over a furnace's or \
         cooler's outlet, and a tank's level over the flow through a valve beside it, are the \
         two this engine checks the sign of (docs/DESIGN.md §29 fork 5, docs/DEFERRED.md E18)",
        primary.name,
        primary.setpoint.variable().noun(),
        graph.point_name(primary.measurement_point),
        secondary.name,
        secondary.setpoint.variable().noun(),
        graph.point_name(secondary.measurement_point)
    ))
}

fn refuse_foreign_key(
    present: bool,
    loop_name: &str,
    key: &str,
    belongs_to: MeasuredVariable,
    measures: MeasuredVariable,
    instead: &str,
) -> Result<(), SimError> {
    if present {
        return Err(SimError::Scenario(format!(
            "control loop '{loop_name}' measures {measures:?} and declares `{key}`, which is \
             the {belongs_to:?} loop's key. A setpoint and a gain carry their variable's unit \
             in their key precisely so this is visible; write `{instead}` instead"
        )));
    }
    Ok(())
}

/// A regulated quantity as a file declares it, carried into the engine's SI unit.
///
/// **The one site a setpoint or a trip limit crosses from file units to SI**
/// (docs/DESIGN.md §26 fork 6): a level's metres are metres, a pressure's bar
/// take `× 1e5`, a temperature's °C take `+ 273.15`, a flow's kg/s are kg/s.
/// A `[[controls]]` setpoint and a `[[trips]]` limit both come through here, so
/// M10's trap (bar converted at one site and not another) and M17's (°C left
/// without its offset) cannot come back through a second copy. A GAIN does not:
/// it is a reciprocal, and its conversion stays beside the setpoint's in
/// `build_controls`, where the pair is visible.
fn declared_value(variable: MeasuredVariable, value: f64) -> ControlledValue {
    match variable {
        MeasuredVariable::Level => ControlledValue::Level { m: Meter(value) },
        MeasuredVariable::Pressure => ControlledValue::Pressure {
            pa: bar_to_pa(value),
        },
        MeasuredVariable::Temperature => ControlledValue::Temperature { k: c_to_k(value) },
        MeasuredVariable::Flow => ControlledValue::Flow {
            kg_per_s: KgPerSec(value),
        },
    }
}

fn require_keyed(
    value: Option<f64>,
    loop_name: &str,
    key: &str,
    what: &str,
) -> Result<f64, SimError> {
    value.ok_or_else(|| {
        SimError::Scenario(format!(
            "control loop '{loop_name}' declares no `{key}` — {what} has no default, \
             because a silent default is an invented value in disguise and every gate \
             would then pass for whatever was chosen"
        ))
    })
}

fn build_slate(defs: &[ComponentDef]) -> Result<Slate, SimError> {
    if defs.is_empty() {
        return Ok(Slate::water_only());
    }
    for (i, def) in defs.iter().enumerate() {
        if defs[..i].iter().any(|d| d.name == def.name) {
            return Err(SimError::Scenario(format!(
                "component '{}' is defined twice: names must be unique, because \
                 compositions reference components by name",
                def.name
            )));
        }
        // Every property is a positive physical magnitude; a zero density would
        // divide by zero in `mixture_density`, and a zero cp makes any heat
        // input an infinite temperature rise.
        for (field, value) in [
            ("tb_c", def.tb_c + 273.15),
            ("molar_mass_kg_per_mol", def.molar_mass_kg_per_mol),
        ]
        .into_iter()
        .chain(def.cp_j_per_kg_k.map(|c| ("cp_j_per_kg_k", c)))
        .chain(def.density_kg_per_m3.map(|d| ("density_kg_per_m3", d)))
        {
            if !value.is_finite() || value <= 0.0 {
                return Err(SimError::Scenario(format!(
                    "component '{}' has a non-positive or non-finite {field} \
                     ({value}); every component property must be > 0",
                    def.name
                )));
            }
        }
        // Phase ↔ density correspondence, refused in BOTH directions so neither
        // mistake can produce a plant that loads: a liquid with no density has
        // no density law at all, and a gas with one carries a number nothing
        // reads (docs/DESIGN.md §3a).
        match (component_phase(def)?, def.density_kg_per_m3) {
            (Phase::Liquid, None) => {
                return Err(SimError::Scenario(format!(
                    "liquid component '{}' has no density_kg_per_m3; a liquid's \
                     density is a declared constant at this fidelity",
                    def.name
                )))
            }
            (Phase::Gas, Some(rho)) => {
                return Err(SimError::Scenario(format!(
                    "gas component '{}' declares density_kg_per_m3 = {rho}, which \
                     nothing reads: a gas density is P·M̄/(R·T), computed from \
                     molar_mass_kg_per_mol and the solved pressure. Remove the field.",
                    def.name
                )))
            }
            _ => {}
        }
    }
    Slate::new(
        defs.iter()
            .map(|d| {
                Ok(PseudoComponent {
                    name: d.name.clone(),
                    tb: c_to_k(d.tb_c),
                    molar_mass: KgPerMol(d.molar_mass_kg_per_mol),
                    density: d.density_kg_per_m3.map(KgPerM3),
                    // Under a shape the file declares no constant — the key is
                    // refused there — so `cp` is the shape's OWN value at the
                    // enthalpy datum. That keeps the field meaning one thing
                    // ("this cut's capacity at `T_REF`") rather than two, and it
                    // is a derived number rather than a declared one, which is
                    // what fork 3's trap is actually about.
                    cp: JPerKgK(match (d.cp_j_per_kg_k, component_cp_shape(d)?) {
                        (Some(constant), _) => constant,
                        (None, Some(shape)) => shape.at_datum(T_REF).0,
                        (None, None) => {
                            return Err(SimError::Scenario(format!(
                                "component '{}' declares neither cp_j_per_kg_k nor a cp shape",
                                d.name
                            )))
                        }
                    }),
                    cp_shape: component_cp_shape(d)?,
                    phase: component_phase(d)?,
                })
            })
            .collect::<Result<Vec<_>, SimError>>()?,
    )
}

/// Parse a component's three `cp_shape_*` keys into a [`CpShape`], or `None`.
///
/// **All three or none**, refused otherwise: a slope with no anchor is not a
/// partial specification but an ambiguous one, and the whole point of the anchor
/// pair is that "which temperature is this quoted at" is never implied.
///
/// **Monotonicity is checked HERE, at load, and not at runtime** (§20 fork 2).
/// With `slope >= 0` and `cp(T_REF) > 0` the capacity is positive for every
/// `T >= T_REF`, so `h` is strictly increasing there and its inverse exists in
/// closed form and is single valued — which is what lets the tick loop invert a
/// quadratic instead of defending an iteration count. This project prefers a
/// refusal at load to a diverging solve.
fn component_cp_shape(def: &ComponentDef) -> Result<Option<CpShape>, SimError> {
    let parts = (
        def.cp_shape_anchor_c,
        def.cp_shape_at_anchor_j_per_kg_k,
        def.cp_shape_slope_j_per_kg_k2,
    );
    let (anchor_c, at_anchor, slope) = match parts {
        (None, None, None) => return Ok(None),
        (Some(a), Some(c), Some(s)) => (a, c, s),
        _ => {
            return Err(SimError::Scenario(format!(
                "component '{}' declares only part of a cp shape: all three of \
                 cp_shape_anchor_c, cp_shape_at_anchor_j_per_kg_k and \
                 cp_shape_slope_j_per_kg_k2 are required together, because a slope with no \
                 anchor does not say which temperature its capacity is quoted at",
                def.name
            )))
        }
    };
    for (field, value) in [
        ("cp_shape_anchor_c", anchor_c),
        ("cp_shape_at_anchor_j_per_kg_k", at_anchor),
        ("cp_shape_slope_j_per_kg_k2", slope),
    ] {
        if !value.is_finite() {
            return Err(SimError::Scenario(format!(
                "component '{}' has a non-finite {field} ({value})",
                def.name
            )));
        }
    }
    if at_anchor <= 0.0 {
        return Err(SimError::Scenario(format!(
            "component '{}' has cp_shape_at_anchor_j_per_kg_k = {at_anchor}; a heat capacity \
             must be > 0",
            def.name
        )));
    }
    if slope < 0.0 {
        return Err(SimError::Scenario(format!(
            "component '{}' has cp_shape_slope_j_per_kg_k2 = {slope}: a DECREASING cp is \
             refused at this fidelity. Monotone h is what makes the inversion T(h) a closed \
             form with one root, and a decreasing shape would need the file to declare the \
             temperature range it stays positive over — a key that does not exist \
             (docs/DESIGN.md §20 fork 2).",
            def.name
        )));
    }
    let shape = CpShape {
        anchor_temperature: c_to_k(anchor_c),
        cp_at_anchor: JPerKgK(at_anchor),
        slope,
    };
    let (cp_at_datum, _) = shape.at_datum(T_REF);
    if cp_at_datum <= 0.0 {
        return Err(SimError::Scenario(format!(
            "component '{}' declares a cp shape that is {cp_at_datum:.4} J/(kg·K) at the \
             enthalpy datum {} K — non-positive, so h is not monotone over the range the \
             engine integrates from. Raise cp_shape_at_anchor_j_per_kg_k, lower the slope, or \
             move the anchor.",
            def.name,
            T_REF.value()
        )));
    }
    Ok(Some(shape))
}

/// Parse a component's `phase = "..."` field. Absent is liquid.
fn component_phase(def: &ComponentDef) -> Result<Phase, SimError> {
    match def.phase.as_deref() {
        None | Some("liquid") => Ok(Phase::Liquid),
        Some("gas") => Ok(Phase::Gas),
        Some(other) => Err(SimError::Scenario(format!(
            "component '{}' has unknown phase '{other}' (valid: liquid, gas)",
            def.name
        ))),
    }
}

/// Resolve a node's `composition = { name = weight, ... }` against the slate.
///
/// Weights are normalized (`Composition::from_weights`), so a file may write
/// fractions summing to 1 or raw mass amounts — whichever reads better — and
/// both mean the same thing. Unknown names are rejected rather than ignored: a
/// typo'd cut name would otherwise silently drop that fraction and renormalize
/// the rest, producing a plausible-looking wrong feed.
///
/// An absent composition is only meaningful on a one-component slate, where
/// there is exactly one thing the fluid can be. On a real slate it is refused
/// rather than defaulted — see `NodeDef::Source::composition`.
fn resolve_composition(
    node: &str,
    weights: &Option<BTreeMap<String, f64>>,
    slate: &Slate,
) -> Result<Composition, SimError> {
    let Some(weights) = weights else {
        if slate.len() == 1 {
            return Ok(Composition::pure(1, 0));
        }
        return Err(SimError::Scenario(format!(
            "node '{node}' has no composition, but the slate has {} components. \
             Only a one-component slate has an unambiguous default; write \
             composition = {{ <component> = <weight>, ... }}",
            slate.len()
        )));
    };
    let mut fractions = vec![0.0; slate.len()];
    for (name, weight) in weights {
        let index = slate.index_of(name).ok_or_else(|| {
            SimError::Scenario(format!(
                "node '{node}' references unknown component '{name}'; the slate \
                 defines: {}",
                slate
                    .iter()
                    .map(|c| c.name.as_str())
                    .collect::<Vec<_>>()
                    .join(", ")
            ))
        })?;
        fractions[index] = *weight;
    }
    Composition::from_weights(&fractions)
        .map_err(|e| SimError::Scenario(format!("node '{node}': {e}")))
}

/// Convert a scenario node definition into a core `NodeKind`, applying the
/// human-friendly → SI conversions at this boundary (bar → Pa, °C → K,
/// metric Kv → element cv_si, tank level → mass).
///
/// A tank's initial mass is `ρ·A·h` at the density of ITS OWN contents, not at
/// water's. With a one-component slate the two coincide, which is why this went
/// unnoticed through M1 and M2; with a crude slate, filling a tank with a light
/// cut and computing its mass at 998 kg/m³ would over-charge the inventory by
/// ~40% and break mass conservation at tick zero.
fn node_kind(name: &str, def: &NodeDef, slate: &Slate) -> Result<NodeKind, SimError> {
    Ok(match def {
        NodeDef::Source {
            pressure_bar,
            temperature_c,
            composition,
        } => NodeKind::Source {
            pressure: bar_to_pa(*pressure_bar),
            temperature: c_to_k(*temperature_c),
            composition: resolve_composition(name, composition, slate)?,
        },
        NodeDef::Sink {
            pressure_bar,
            temperature_c,
            composition,
        } => NodeKind::Sink {
            pressure: bar_to_pa(*pressure_bar),
            temperature: c_to_k(*temperature_c),
            // Through the same resolver a source uses, so an absent composition
            // on a real slate is REFUSED rather than defaulted. This is the one
            // place a sink's composition does not mirror its temperature, and
            // the asymmetry is in the physics, not the design: ambient is a
            // defensible neutral temperature to back-feed, and there is no
            // corresponding neutral composition — "the first cut" is a guess
            // that would run.
            composition: resolve_composition(name, composition, slate)?,
        },
        NodeDef::Atmosphere => NodeKind::Atmosphere,
        NodeDef::Tank {
            area_m2,
            height_m,
            initial_level_m,
            temperature_c,
            ambient_exchange_ua_w_per_k,
            // Refused by `validate_node_def`, which runs before this pass; the
            // field exists only so an old spelling fails by name (M15.1).
            retired_ambient_ua_w_per_k: _,
            // Read by `build_boiloff_vents`, which runs after every node exists
            // so a destination declared later in the file resolves — the same
            // reason a column draw and an exchanger coupling are resolved in
            // their own passes rather than here.
            vent_to: _,
            composition,
        } => {
            let area = SquareMeter(*area_m2);
            let composition = resolve_composition(name, composition, slate)?;
            // A tank holds a LIQUID, and the two lines below are why the guard
            // is here rather than left to the connected-component check: both
            // `ρ·A·h` and `bottom_pressure`'s `ρgh` read a stored liquid density,
            // and on a gas composition there is none — an inventory and a
            // hydrostatic head computed from a level are not merely inaccurate
            // for a gas, they name nothing. A gas holdup is the capacitive
            // vessel (M5.3), whose state is pressure, not level.
            if composition.phase(slate)? == Phase::Gas {
                return Err(SimError::Scenario(format!(
                    "tank '{name}' holds a gas-phase composition. A tank's inventory \
                     (ρ·A·h) and head (ρgh) are liquid-level quantities; a gas holdup \
                     is a capacitive vessel, whose state is pressure (docs/DESIGN.md §3a)"
                )));
            }
            // m = ρ·A·h, at the density of the tank's own contents — through
            // `TankState::mass_at_level`, the one owner of that product, so a
            // tank declared exactly full holds exactly the capacity its overflow
            // compares against (M23, docs/DESIGN.md §27 fork 4).
            let mut tank = TankState {
                area,
                height: Meter(*height_m),
                mass: Kg(0.0),
                temperature: c_to_k(*temperature_c),
                composition,
                ambient_ua: WattPerKelvin(*ambient_exchange_ua_w_per_k),
            };
            tank.mass = tank.mass_at_level(slate, Meter(*initial_level_m));
            NodeKind::Tank(tank)
        }
        NodeDef::Vessel {
            volume_m3,
            pressure_bar,
            temperature_c,
            composition,
        } => {
            let composition = resolve_composition(name, composition, slate)?;
            // The mirror of the tank's liquid-only guard, and the other half of
            // the same partition: a holdup is a tank if its state is a level and
            // a vessel if its state is a pressure. `C = V·M̄/(R·T)` is the
            // ideal-gas relation — for an incompressible liquid it is not merely
            // inaccurate, it names nothing, and it would silently produce a
            // capacitance ~5 orders too small and a plant that oscillates.
            if composition.phase(slate)? != Phase::Gas {
                return Err(SimError::Scenario(format!(
                    "vessel '{name}' holds a liquid-phase composition. A vessel's state is \
                     PRESSURE and its capacitance C = V·M̄/(R·T) is the ideal-gas relation; \
                     a liquid holdup is a tank, whose state is level (docs/DESIGN.md §3a)"
                )));
            }
            let mut vessel = VesselState {
                volume: CubicMeter(*volume_m3),
                // Filled in immediately below, from the capacitance this same
                // struct computes. Going through `capacitance` rather than
                // writing `P·V·M̄/(R·T)` out again is what guarantees the vessel's
                // `pressure()` reads back EXACTLY the declared bar figure — and
                // therefore that the accumulation term's `Pⁿ` starts where the
                // file says the plant does.
                mass: Kg(0.0),
                temperature: c_to_k(*temperature_c),
                composition,
            };
            vessel.mass = Kg(bar_to_pa(*pressure_bar).value() * vessel.capacitance(slate));
            NodeKind::Vessel(vessel)
        }
        NodeDef::Pump { h0_m, a, on } => NodeKind::Pump {
            h0: Meter(*h0_m),
            a: *a,
            on: *on,
        },
        // `x_t` is carried through unvalidated HERE and checked in
        // `require_gas_valve_x_t` instead: whether a valve is in gas service is a
        // TOPOLOGICAL fact, and the topology does not exist yet at this point in
        // the load.
        NodeDef::Valve { kv, opening, x_t } => NodeKind::Valve {
            cv_max: kv_to_cv_si(*kv),
            opening: *opening,
            x_t: *x_t,
        },
        NodeDef::ReliefValve {
            kv,
            set_pressure_bar,
            accumulation_bar,
            x_t,
        } => {
            if !accumulation_bar.is_finite() || *accumulation_bar <= 0.0 {
                return Err(SimError::Scenario(format!(
                    "relief valve '{name}' has accumulation_bar = {accumulation_bar}, which must be \
                     > 0. A zero band makes the opening a STEP in pressure, and a discontinuous \
                     characteristic is exactly what elements.rs promises not to hand the Newton \
                     Jacobian."
                )));
            }
            if !set_pressure_bar.is_finite() || *set_pressure_bar <= 0.0 {
                return Err(SimError::Scenario(format!(
                    "relief valve '{name}' has set_pressure_bar = {set_pressure_bar}; a set pressure \
                     is ABSOLUTE and must be positive."
                )));
            }
            NodeKind::ReliefValve {
                cv_max: kv_to_cv_si(*kv),
                set_pressure: bar_to_pa(*set_pressure_bar),
                accumulation: bar_to_pa(*accumulation_bar),
                x_t: *x_t,
            }
        }
        NodeDef::CheckValve {
            kv,
            full_open_bar,
            x_t,
        } => {
            if !full_open_bar.is_finite() || *full_open_bar <= 0.0 {
                return Err(SimError::Scenario(format!(
                    "check valve '{name}' has full_open_bar = {full_open_bar}, which must be > 0. \
                     A zero band makes the opening a STEP in the forward drive, and a \
                     discontinuous characteristic is exactly what elements.rs promises not to \
                     hand the Newton Jacobian (docs/DESIGN.md §33)"
                )));
            }
            // Refused where a control valve's is not, because a valve's opening
            // can carry the shut-off and a check valve has no opening to declare:
            // `kv = 0` would be a blind flange wearing a check valve's name.
            if !kv.is_finite() || *kv <= 0.0 {
                return Err(SimError::Scenario(format!(
                    "check valve '{name}' has kv = {kv}, which must be > 0. A check valve \
                     with no flow coefficient passes nothing in either direction"
                )));
            }
            NodeKind::CheckValve {
                cv_max: kv_to_cv_si(*kv),
                full_open: bar_to_pa(*full_open_bar),
                x_t: *x_t,
            }
        }
        NodeDef::Furnace {
            duty_mw,
            coil_heat_capacity_mj_per_k,
            coil_ua_kw_per_k,
            coil_temperature_c,
        } => NodeKind::Furnace {
            duty: Watt(*duty_mw * 1e6),
            coil: FurnaceCoil {
                heat_capacity: JPerK(*coil_heat_capacity_mj_per_k * 1e6),
                conductance: WattPerKelvin(*coil_ua_kw_per_k * 1e3),
                temperature: c_to_k(*coil_temperature_c),
            },
        },
        NodeDef::Cooler { duty_mw } => NodeKind::Cooler {
            duty: Watt(*duty_mw * 1e6),
        },
        NodeDef::HeatExchanger => NodeKind::HeatExchanger,
        NodeDef::Junction => NodeKind::Junction,
        NodeDef::Column {
            pressure_bar,
            smearing_k,
            ..
        } => NodeKind::Column {
            pressure: bar_to_pa(*pressure_bar),
            // A temperature WIDTH, so no °C→K offset — a 10 °C ramp is 10 K.
            // Absent means 0, the sharp splitter, which is what every file that
            // omits it has always meant; the `Option` exists so the cascade can
            // refuse a value it would not read (`require_declared_iff_used`).
            smearing: Kelvin(smearing_k.unwrap_or(0.0)),
            // Draws are resolved in a second pass, once every node exists so a
            // draw can name an outlet defined later in the file (like couplings).
            draws: Vec::new(),
            // Filled by the same second pass, which is where the declared-iff-used
            // correspondence against `[fidelity] separation` is enforced.
            cascade: None,
        },
        NodeDef::Reactor { t_set_c, tau_s } => NodeKind::Reactor {
            t_set: c_to_k(*t_set_c),
            tau: Seconds(*tau_s),
        },
    })
}

/// Reject node definitions whose numbers are out of physical range, at load
/// rather than at solve.
///
/// Duty is a non-negative MAGNITUDE in both units — direction is the unit's
/// identity, not the sign of its number (see `NodeKind::Cooler`). A negative
/// `furnace` duty used to be the only way to express cooling; now that `cooler`
/// exists it can only be a sign slip, and a silently-chilling furnace is a
/// plausible-looking wrong plant. Rejecting it here is what makes the two-unit
/// design safe rather than merely tidy.
///
/// A tank's `UA` is refused on the same grounds. It is a CONDUCTANCE, not a
/// signed rate: the direction of ambient exchange already comes from
/// `T_AMBIENT − T_tank`, so a negative `UA` does not mean "loses heat" — it
/// inverts the driving force, warming a hot tank further and cooling a cold one,
/// a positive feedback that runs away from ambient instead of towards it.
/// Reject a pipe's `UA` on the same grounds as a tank's, and one sharper one.
///
/// For a tank a negative `UA` inverts the driving force — bad, but the runaway
/// is geometric per tick and bounded by the tick count. For a pipe it lands in
/// an EXPONENT: `exp(−UA/(|ṁ|·cp))` with `UA < 0` is `exp(+x)`, which multiplies
/// the temperature difference from ambient every time the fluid crosses the
/// pipe, and a loop of such pipes diverges to infinity within a few ticks. Same
/// conceptual error, a much shorter fuse.
/// Build a declared pipe as a LEAK PATH: two halves joined by a `Junction`, with
/// a dormant orifice edge from that junction to `atmosphere`.
///
/// **Split at LOAD, not at puncture** (docs/DESIGN.md §3b). Splitting when the
/// damage happens would change the snapshot's shape mid-run, which is a rule-6
/// contract problem for every frontend; splitting at load fixes the topology
/// before tick 0, so a punctured plant and an intact one have the same shape and
/// differ only in one commanded number.
///
/// **Hanging the orifice off an ENDPOINT was not merely inelegant, it was
/// illegal.** `validate_degrees` requires exactly 1-in-1-out of a pump, valve,
/// PSV, furnace, cooler, reactor and exchanger side, so a third edge leaving any
/// of them is a load-time `Err` — and a pipe's upstream endpoint is a pump or a
/// valve constantly (`tank_pump_valve.toml` is nothing but). An endpoint rule
/// would therefore have forbidden leaks on exactly the lines a game most wants
/// to puncture. The midpoint junction has no degree rule on it, and it carries
/// the physically right pressure for a mid-pipe hole into the bargain: neither
/// endpoint's, but the one between them.
///
/// The halves split the length, the elevation and the `UA` evenly and keep the
/// diameter and friction factor, so the two in series are hydraulically the
/// declared pipe: `k ∝ L` adds back to the original, `β = ρ·g·Δz` adds back, and
/// the ambient transform composes over the two halves. **The upstream half keeps
/// the declared NAME** — it is what `PuncturePipe` addresses and where
/// `leak_mass_flow` is reported.
fn split_for_leak(
    graph: &mut PlantGraph,
    def: &PipeDef,
    from: NodeId,
    to: NodeId,
    whole: Pipe,
    atmosphere: &str,
) -> Result<(), SimError> {
    let vent = graph.find_node(atmosphere).ok_or_else(|| {
        SimError::Scenario(format!(
            "pipe '{}' declares leak_to = '{atmosphere}', which is not a node in this plant",
            def.name
        ))
    })?;
    if !matches!(graph.node(vent).kind, NodeKind::Atmosphere) {
        return Err(SimError::Scenario(format!(
            "pipe '{}' declares leak_to = '{atmosphere}', which is a {:?}, not an atmosphere. \
             A leak vents to the outside world; venting it into the plant would be an \
             ordinary pipe, and the scenario should say so",
            def.name,
            graph.node(vent).kind
        )));
    }
    // A COLUMN's pipes cannot be split, and this refusal is here because the
    // failure it prevents is silent. `network::is_column_draw_edge` recognises a
    // draw by its two endpoints — column at one end, one of that column's
    // declared outlets at the other — and `edge_flows` guards a draw's flow to
    // zero on the strength of it, because a draw's flow is PRESCRIBED
    // (`splitᵢ·ṁ_feed`, written post-sweep) and not pressure-driven at all.
    // Split that edge and neither half matches any more, so the guard silently
    // stops applying and the draw becomes a pressure-driven number that is
    // finite, deterministic, mass-conserving and wrong — DESIGN §5's silent
    // hazard, reached by a scenario line that looks entirely reasonable. The feed
    // is refused with it: a column is 1-in-N-out by `validate_degrees`, so a
    // split feed would fail there anyway, but with a message about degrees that
    // names the wrong cause.
    for end in [from, to] {
        if matches!(graph.node(end).kind, NodeKind::Column { .. }) {
            return Err(SimError::Scenario(format!(
                "pipe '{}' declares a leak path but connects to column '{}'. A column's \
                 feed and draw pipes cannot be split: a draw's flow is prescribed by the \
                 feed split, not by pressure, and splitting it would silently turn it \
                 into a pressure-driven flow (docs/DESIGN.md §3b, §5)",
                def.name,
                graph.node(end).name
            )));
        }
    }

    let junction = format!("{}__leak_point", def.name);
    if graph.find_node(&junction).is_some() {
        return Err(SimError::Scenario(format!(
            "pipe '{}' declares a leak path, whose midpoint junction would be named \
             '{junction}' — and this plant already has a node by that name. Rename one",
            def.name
        )));
    }
    let mid = graph.add_node(Node {
        name: junction,
        kind: NodeKind::Junction,
        heat_input: Watt::ZERO,
    });

    // Half a pipe each: k ∝ L and β = ρ·g·Δz both add back to the declared pipe,
    // and UA ∝ exposed area does too.
    let half = |name: String| Pipe {
        name,
        length: Meter(def.length_m / 2.0),
        elevation_change: Meter(def.elevation_change_m / 2.0),
        ambient_ua: WattPerKelvin(def.ambient_exchange_ua_w_per_k / 2.0),
        ..whole.clone()
    };
    let upstream = graph.add_pipe(from, mid, half(def.name.clone()));
    graph.add_pipe(mid, to, half(format!("{}__downstream", def.name)));

    // The orifice runs junction → atmosphere, so positive graph direction is
    // OUTWARD and `leak_mass_flow` needs no sign flip. Its geometry is ZERO on
    // purpose: an orifice has no length to resist with and no bore that means
    // anything (its area is commanded), and `compile_edge` returns before reading
    // either. Should that early return ever be removed, `pipe_resistance` on a
    // zero length and a zero diameter is non-finite and the edge fails loudly at
    // the first solve — which is the point of writing zeros rather than plausible
    // numbers that would quietly become a second resistance in the leak path.
    let orifice = graph.add_pipe(
        mid,
        vent,
        Pipe {
            name: format!("{}__leak", def.name),
            length: Meter(0.0),
            diameter: Meter(0.0),
            elevation_change: Meter(0.0),
            ambient_ua: WattPerKelvin(0.0),
            leak: LeakRole::Orifice {
                area: SquareMeter::ZERO,
            },
            ..whole
        },
    );
    graph.pipe_mut(upstream).leak = LeakRole::Punctureable { orifice };
    Ok(())
}

/// Resolve the `[[exchangers]]` table into graph couplings, rejecting every way
/// a pairing can be malformed.
///
/// The checks are not defensive noise — each rules out a plant that would
/// otherwise RUN and report plausible temperatures:
///
/// - **ε outside (0, 1]** transfers more heat than the inlet temperature
///   difference makes available, crossing the outlets. That is a second-law
///   violation the sweep cannot detect locally, since every intermediate number
///   stays finite and positive.
/// - **A side paired twice** would give one node two partners, and the energy
///   sweep's pair merge silently uses whichever coupling it finds first.
/// - **A side paired with itself** makes the exchanger its own upstream.
/// - **An uncoupled `heat_exchanger` node** is not an exchanger at all — it
///   would behave as a plain junction, transferring nothing, which is exactly
///   what a scenario author who forgot the table would fail to notice.
fn build_couplings(graph: &mut PlantGraph, defs: &[ExchangerDef]) -> Result<(), SimError> {
    let mut paired: BTreeMap<NodeId, String> = BTreeMap::new();

    for def in defs {
        if !(def.effectiveness.is_finite() && def.effectiveness > 0.0 && def.effectiveness <= 1.0) {
            return Err(SimError::Scenario(format!(
                "exchanger '{}'/'{}' has effectiveness = {}: it must lie in (0, 1]. \
                 Above 1 the exchanger would transfer more than the inlet \
                 temperature difference allows and cross the outlet temperatures; \
                 0 or less is not an exchanger.",
                def.side_a, def.side_b, def.effectiveness
            )));
        }
        if def.side_a == def.side_b {
            return Err(SimError::Scenario(format!(
                "exchanger pairs '{}' with itself: the two sides must be \
                 different nodes",
                def.side_a
            )));
        }

        let resolve = |name: &str| -> Result<NodeId, SimError> {
            let id = graph.find_node(name).ok_or_else(|| {
                SimError::Scenario(format!("exchanger references unknown node '{name}'"))
            })?;
            if !matches!(graph.node(id).kind, NodeKind::HeatExchanger) {
                return Err(SimError::Scenario(format!(
                    "exchanger side '{name}' is a {:?}, not a heat_exchanger node",
                    graph.node(id).kind
                )));
            }
            Ok(id)
        };
        let side_a = resolve(&def.side_a)?;
        let side_b = resolve(&def.side_b)?;

        for (id, name) in [(side_a, &def.side_a), (side_b, &def.side_b)] {
            if let Some(other) = paired.get(&id) {
                return Err(SimError::Scenario(format!(
                    "exchanger side '{name}' is paired more than once (already \
                     coupled with '{other}'): each side has exactly one partner"
                )));
            }
            paired.insert(id, name.clone());
        }

        graph.add_coupling(HeatExchangerCoupling {
            side_a,
            side_b,
            effectiveness: def.effectiveness,
        });
    }

    // Every side must be in the table: an unpaired one is a silent plain pipe.
    for id in graph.node_ids().collect::<Vec<_>>() {
        if matches!(graph.node(id).kind, NodeKind::HeatExchanger) && !paired.contains_key(&id) {
            return Err(SimError::Scenario(format!(
                "heat_exchanger node '{}' is not paired in any [[exchangers]] \
                 entry: an unpaired side transfers no heat at all and would run \
                 as a plain junction",
                graph.node(id).name
            )));
        }
    }
    Ok(())
}

/// Resolve every `column` node's draws (name → NodeId) and reject every way a
/// draw list can be malformed. Runs after all nodes exist so a draw may name an
/// outlet defined later in the file, exactly like an exchanger coupling.
///
/// Each check rules out a plant that would otherwise load and be wrong:
///
/// - **Fewer than two draws** separates nothing — use a pipe.
/// - **The catch-all convention** — exactly the LAST draw omits `up_to_c`. The
///   heaviest draw is open-topped so every component lands somewhere; if it had a
///   finite top, components above it would be split into no draw and lost, a
///   silent mass leak. Making "which draw is the residue" a syntactic fact keeps
///   conservation a load-time guarantee.
/// - **Cut points strictly increasing** — a flat or inverted cut is an empty or
///   backwards band.
/// - **Distinct outlets** — the engine maps a draw edge to a draw by its outlet,
///   so the mapping has to be one-to-one.
/// - **Outlet is a product store** (tank/sink/atmosphere) — a free node on a draw
///   line puts a prescribed edge back into the Jacobian (unsupported), and a
///   source or column outlet is pressure-fixing but still wrong (vanishes the
///   product, or chains columns the draw write cannot feed); both refused.
///
/// Under the **cascade** fidelity the boiling-range half of that list is replaced
/// rather than extended: `up_to_c` and `smearing_k` are refused, each draw
/// declares its `stage` and (except the bottoms) its `draw_ratio`, and the column
/// declares a `[cascade]` block. The structural validation of that shape is
/// delegated to `StageCascade::validate` so the loader and the model cannot drift
/// apart; what stays here is what needs the file's vocabulary — which key on which
/// node, and the two scope boundaries (partial condenser, vapour side draw) that
/// have no representation in `core` at all.
fn resolve_column_draws(graph: &mut PlantGraph, scenario: &ScenarioFile) -> Result<(), SimError> {
    let cascade_selected = scenario.fidelity.separation == "cascade";
    for (name, def) in &scenario.nodes {
        let NodeDef::Column {
            draws: draw_defs,
            cascade,
            smearing_k,
            ..
        } = def
        else {
            continue;
        };
        if draw_defs.len() < 2 {
            return Err(SimError::Scenario(format!(
                "column '{name}' has {} draw(s): a column needs at least two, else it \
                 separates nothing (use a pipe).",
                draw_defs.len()
            )));
        }
        require_declared_iff_used(name, cascade_selected, draw_defs, cascade, smearing_k)?;

        let last = draw_defs.len() - 1;
        let mut prev_cut = f64::NEG_INFINITY;
        let mut resolved: Vec<ColumnDraw> = Vec::with_capacity(draw_defs.len());
        for (i, d) in draw_defs.iter().enumerate() {
            // Exactly the last draw omits up_to_c — the open catch-all. Under the
            // cascade this whole question is `stage`'s instead, and the stage
            // numbering is checked by `StageCascade::validate` below.
            let upper_cut_c = match (cascade_selected, i == last, d.up_to_c) {
                (true, _, _) => None,
                (false, true, None) => None,
                (false, false, Some(c)) => Some(c),
                (false, true, Some(_)) => {
                    return Err(SimError::Scenario(format!(
                        "column '{name}' draw '{}' is the heaviest (last) draw and must OMIT \
                         up_to_c: it is the catch-all for everything above the last cut, so a \
                         finite top would drop every heavier component.",
                        d.outlet
                    )))
                }
                (false, false, None) => {
                    return Err(SimError::Scenario(format!(
                        "column '{name}' draw '{}' omits up_to_c but is not the last draw: only \
                         the heaviest (last) draw may — every other draw needs a boiling-range top.",
                        d.outlet
                    )))
                }
            };
            if let Some(c) = upper_cut_c {
                let cut_k = c_to_k(c).value();
                if cut_k <= prev_cut {
                    return Err(SimError::Scenario(format!(
                        "column '{name}' draw '{}' has up_to_c = {c} °C, not strictly above the \
                         previous cut: draws must be listed in ascending boiling order.",
                        d.outlet
                    )));
                }
                prev_cut = cut_k;
            }

            if draw_defs[..i].iter().any(|e| e.outlet == d.outlet) {
                return Err(SimError::Scenario(format!(
                    "column '{name}' draws to '{}' more than once: each draw feeds a distinct \
                     product node.",
                    d.outlet
                )));
            }

            let outlet = graph.find_node(&d.outlet).ok_or_else(|| {
                SimError::Scenario(format!(
                    "column '{name}' draw references unknown outlet node '{}'",
                    d.outlet
                ))
            })?;
            // A draw must end at a PRODUCT STORE. "Pressure-fixing" is necessary
            // (a free node would put a prescribed edge in the Jacobian) but NOT
            // sufficient: `fixed_pressure` is also `Some` for a Source and a
            // Column, and both are silently wrong outlets. Drawing to a Source
            // would vanish the product into an infinite supply — mass "conserved"
            // at the boundary, a plant that runs and lies. Drawing to another
            // Column would chain them, and the post-sweep two-pass draw write
            // reads the upstream draw edge before the downstream column's write
            // lands, so the second column silently sees a zero feed and does
            // nothing. Neither is modelled at this fidelity, so the outlet is
            // restricted to the three product-store kinds explicitly.
            match &graph.node(outlet).kind {
                NodeKind::Tank(_) | NodeKind::Sink { .. } | NodeKind::Atmosphere => {}
                NodeKind::Source { .. } | NodeKind::Column { .. } => {
                    return Err(SimError::Scenario(format!(
                        "column '{name}' draws to '{}', a source or column. A draw must end at a \
                         product store — a tank, sink, or atmosphere. Drawing to a source vanishes \
                         the product into a supply, and chaining columns is not supported at this \
                         fidelity.",
                        d.outlet
                    )));
                }
                _ => {
                    return Err(SimError::Scenario(format!(
                        "column '{name}' draws to '{}', a free (non-pressure-fixing) node. A draw \
                         line must end at a product store (tank/sink/atmosphere); a valve or \
                         junction on a draw is not supported at this fidelity.",
                        d.outlet
                    )));
                }
            }
            resolved.push(if cascade_selected {
                ColumnDraw::by_stage(outlet, d.stage.unwrap_or(0), d.draw_ratio)
            } else {
                ColumnDraw::by_cut(outlet, upper_cut_c.map(c_to_k))
            });
        }

        let spec = match (cascade_selected, cascade) {
            (true, Some(def)) => {
                let spec = CascadeSpec {
                    stages: def.stages,
                    feed_stage: def.feed_stage,
                    reflux_ratio: def.reflux_ratio,
                };
                // One source of truth for the cascade's structural rules: the model
                // that has to satisfy them. Duplicating them here would be two
                // rule sets to keep in step, and M7.1's correction 3 is about the
                // opposite hazard — a contract kept only in the OTHER crate. Both
                // are avoided by having the loader call the model's own check and
                // add the file's vocabulary to whatever it says.
                refinery_solvers::StageCascade::validate(&spec, &resolved)
                    .map_err(|e| SimError::Scenario(format!("column '{name}': {e}")))?;
                Some(spec)
            }
            _ => None,
        };

        let col_id = graph.find_node(name).expect("column node was just added");
        if let NodeKind::Column {
            draws,
            cascade: slot,
            ..
        } = &mut graph.node_mut(col_id).kind
        {
            *draws = resolved;
            *slot = spec;
        }
    }
    Ok(())
}
