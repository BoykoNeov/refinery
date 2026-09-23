//! `build_engine`: a parsed `ScenarioFile` to a runnable `Engine`.
//!
//! Unit conversion happens at this boundary and nowhere else (CLAUDE.md rule
//! 4): the file speaks bar, °C and Kv; everything past `node_kind` is SI.

use refinery_core::components::{Composition, CpShape, Phase, PseudoComponent, Slate};
use refinery_core::energy::T_REF;
use refinery_core::engine::{Engine, EngineConfig};
use refinery_core::error::SimError;
use refinery_core::graph::{
    CascadeSpec, ColumnDraw, ControlLoop, ControlMode, ControlledValue, HeatExchangerCoupling,
    LeakRole, MeasuredVariable, Node, NodeId, NodeKind, Pipe, PlantGraph, TankState, VesselState,
};
use refinery_core::stream::Stream;
use refinery_core::traits::{
    BoilOffModel, Controller, EnthalpyModel, FlowSolver, ReactionModel, SeparationModel,
    ThermoModel,
};
use refinery_core::units::{
    CubicMeter, JPerKgK, Kelvin, Kg, KgPerM3, KgPerMol, Meter, Seconds, SquareMeter, Watt,
    WattPerKelvin, P_ATM, T_AMBIENT,
};
use std::collections::BTreeMap;

use crate::schema::{
    bar_to_pa, c_to_k, kv_to_cv_si, ComponentDef, ControlDef, ExchangerDef, NodeDef, PipeDef,
    ScenarioFile,
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
    build_controls(&mut graph, &slate, &scenario.controls)?;

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
///   pressure is genuinely solved and absent at tick 0; and the boundary kinds,
///   whose pressures are pinned by declaration),
/// - a temperature measured on anything that is not a holdup (M17) — refused with
///   the missing tick-0 rule named for a zero-volume node, and separately for a
///   column or reactor and for a boundary,
/// - a setpoint or a gain key belonging to ANOTHER variable, every direction,
/// - an actuator the variable cannot pair with (docs/DESIGN.md §21 fork 3's table:
///   a valve for a level or a pressure, a cooler for a temperature), each refused
///   pairing with its own reason, and a `ReliefValve` always,
/// - `max_duty_mw` missing on a cooler or present on a valve, not finite and
///   positive, or a declared cooler duty outside `[0, max_duty_mw]`.
///
/// Each loop is born with a real measurement rather than an empty one: all three
/// controlled variables are stored and true from load, so `PlantGraph::measure` is
/// called here exactly as the tick pass calls it, and a snapshot taken before the
/// first tick reports a true level, pressure or temperature. `last_output` is
/// seeded through `PlantGraph::actuator_position` — the reader the tick pass and
/// the MANUAL→AUTO transfer also use — which is what MANUAL would report and what
/// AUTO overwrites on tick 1.
///
/// That same measurement is what a PI loop's memory is derived AGAINST: fork 5's
/// `initial_output` says where the actuator starts, and the integral term is
/// whatever makes the controller ask for that position given the error standing at
/// load. So the declared number is the one a reader can check on the faceplate at
/// tick 0, and the state behind it is derived rather than declared twice.
fn build_controls(
    graph: &mut PlantGraph,
    slate: &Slate,
    defs: &[ControlDef],
) -> Result<(), SimError> {
    let mut seen_names: Vec<&str> = Vec::new();
    let mut claimed_actuators: Vec<(NodeId, &str)> = Vec::new();

    for def in defs {
        if seen_names.contains(&def.name.as_str()) {
            return Err(SimError::Scenario(format!(
                "two control loops are called '{}'. A loop's name is what its faceplate \
                 is labelled with, so two of them make a snapshot ambiguous",
                def.name
            )));
        }
        seen_names.push(&def.name);

        let variable = match def.measurement.variable.as_str() {
            "level" => MeasuredVariable::Level,
            "pressure" => MeasuredVariable::Pressure,
            "temperature" => MeasuredVariable::Temperature,
            // This message used to say temperature control was deferred because
            // "a temperature really is a solved quantity" — false for a holdup,
            // whose temperature is stored on the graph (docs/DESIGN.md §21). Flow's
            // reason was never that one.
            other => {
                return Err(SimError::Scenario(format!(
                    "control loop '{}' measures unknown variable '{other}' (valid: level, \
                     pressure, temperature). Flow control stays deferred: a flow lives on an \
                     EDGE, which nothing in `PlantGraph::measure`'s signature can name \
                     (docs/DESIGN.md §21)",
                    def.name
                )))
            }
        };

        let measurement_node = graph.find_node(&def.measurement.node).ok_or_else(|| {
            SimError::Scenario(format!(
                "control loop '{}' measures unknown node '{}'",
                def.name, def.measurement.node
            ))
        })?;
        let actuator = graph.find_node(&def.actuator).ok_or_else(|| {
            SimError::Scenario(format!(
                "control loop '{}' actuates unknown node '{}'",
                def.name, def.actuator
            ))
        })?;

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
            .measure(slate, measurement_node, variable)
            .map_err(|e| {
                SimError::Scenario(format!(
                    "control loop '{}' cannot measure {} on node '{}': {e}",
                    def.name, def.measurement.variable, def.measurement.node
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
            (MeasuredVariable::Level | MeasuredVariable::Pressure, NodeKind::Valve { .. }) => {
                if def.max_duty_mw.is_some() {
                    return Err(SimError::Scenario(format!(
                        "control loop '{}' declares `max_duty_mw` on a valve actuator. That key \
                         is a DUTY actuator's range — a cooler's — and a valve's opening is \
                         already a fraction, so the number would be read by nothing \
                         (docs/DESIGN.md §21 fork 3)",
                        def.name
                    )));
                }
                None
            }
            (MeasuredVariable::Temperature, NodeKind::Cooler { duty }) => {
                let max_mw = require_keyed(
                    def.max_duty_mw,
                    &def.name,
                    "max_duty_mw",
                    "the cooler duty the loop's full output stands for",
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
                        "control loop '{}' actuates cooler '{}', whose declared duty {} MW is \
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
            // Reverse acting: `u = clamp(K·e + b, 0, 1)` raises the output when
            // the measurement rises, so the actuator's effect must be to LOWER
            // the measurement. More firing raises a temperature. Mapping the duty
            // as `(1 − u)·max` would make it run and is a negative gain in
            // disguise, which E7's sentence forbids.
            (MeasuredVariable::Temperature, NodeKind::Furnace { .. }) => {
                return Err(SimError::Scenario(format!(
                    "control loop '{}' holds a temperature with furnace '{}'. Raising a loop's \
                     output must LOWER its measurement, and more firing raises it: that is \
                     REVERSE action, which needs its own declaration rather than a sign and is \
                     deferred as docs/DEFERRED.md E7 (docs/DESIGN.md §21 fork 2)",
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
                     cascade (docs/DEFERRED.md E2) wearing one loop's name. Refused as a scope \
                     decision, not as physics (docs/DESIGN.md §21 fork 3)",
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
                     writes a cooler's duty (docs/DESIGN.md §21 fork 3)",
                    def.name, def.actuator
                )))
            }
        };

        if let Some((_, owner)) = claimed_actuators.iter().find(|(id, _)| *id == actuator) {
            return Err(SimError::Scenario(format!(
                "control loops '{owner}' and '{}' both actuate '{}'. Two writers of one \
                 opening have no defined resolution order, and declaration order is not \
                 one — split-range, override and feedforward control are real and are \
                 deferred together, with an arbitration (docs/DESIGN.md §10)",
                def.name, def.actuator
            )));
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
        // With three variables every loop has four foreign keys, so the refusal
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
        ] {
            if belongs_to != variable {
                refuse_foreign_key(present, &def.name, key, belongs_to, variable, instead)?;
            }
        }
        let (setpoint, gain) = match variable {
            MeasuredVariable::Level => {
                let setpoint = ControlledValue::Level {
                    m: Meter(require_keyed(
                        def.setpoint_m,
                        &def.name,
                        variable.setpoint_key(),
                        "the loop's target",
                    )?),
                };
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
                let setpoint = ControlledValue::Pressure {
                    pa: bar_to_pa(require_keyed(
                        def.setpoint_bar,
                        &def.name,
                        variable.setpoint_key(),
                        "the loop's target",
                    )?),
                };
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
                let setpoint = ControlledValue::Temperature {
                    k: c_to_k(require_keyed(
                        def.setpoint_c,
                        &def.name,
                        variable.setpoint_key(),
                        "the loop's target",
                    )?),
                };
                let gain =
                    require_keyed(def.gain_per_k, &def.name, variable.gain_key(), "the gain")?;
                (setpoint, gain)
            }
        };
        graph
            .check_setpoint(measurement_node, setpoint)
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
                // rather than a later call, so an unseeded `PiController` is not a
                // value that can exist — fork 5's "no silent zero" holds by
                // construction. The range check on `initial_output` lives with the
                // controller, beside the range check `Command::SetValveOpening`
                // applies to the same quantity.
                Box::new(
                    refinery_solvers::PiController::new(
                        gain,
                        integral_time_s,
                        initial_output,
                        measurement,
                        setpoint,
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
        let last_output = graph.actuator_position(actuator, max_duty)?;

        graph.add_control(ControlLoop {
            name: def.name.clone(),
            measurement_node,
            actuator,
            max_duty,
            setpoint,
            mode,
            algorithm,
            last_measurement: measurement,
            last_output,
        });
    }
    Ok(())
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
            // m = ρ·A·h, at the density of the tank's own contents.
            let density = composition.mixture_density(slate);
            let mass = Kg(density.value() * area.value() * initial_level_m);
            NodeKind::Tank(TankState {
                area,
                height: Meter(*height_m),
                mass,
                temperature: c_to_k(*temperature_c),
                composition,
                ambient_ua: WattPerKelvin(*ambient_exchange_ua_w_per_k),
            })
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
        NodeDef::Furnace { duty_mw } => NodeKind::Furnace {
            duty: Watt(*duty_mw * 1e6),
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
