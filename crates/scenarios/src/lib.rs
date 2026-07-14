//! refinery-scenarios: plant-definition file format (TOML) and the
//! EngineBuilder that instantiates an Engine from it — including the
//! fidelity selection (which solver implementations to plug in).
//!
//! The scenario file is the ONLY place fidelity is chosen. See
//! scenarios/tank_pump_valve.toml for the reference example of the format.
//!
//! Unit conventions in scenario files (converted to SI on load):
//! - pressures in bar (absolute), temperatures in °C, lengths in m,
//!   volumes in m³, flow coefficients as customary metric Kv (m³/h at 1 bar)
//!   — human-friendly at the boundary, SI inside, per CLAUDE.md rule 4.

use refinery_core::components::{Composition, Slate};
use refinery_core::engine::{Engine, EngineConfig};
use refinery_core::error::SimError;
use refinery_core::graph::{Node, NodeId, NodeKind, Pipe, PlantGraph, TankState};
use refinery_core::stream::Stream;
use refinery_core::traits::{FlowSolver, ReactionModel, ThermoModel};
use refinery_core::units::{
    Kelvin, Kg, KgPerM3, Meter, Pascal, SquareMeter, Watt, P_ATM, T_AMBIENT,
};
use serde::Deserialize;
use std::collections::BTreeMap;

#[derive(Debug, Deserialize)]
pub struct ScenarioFile {
    pub meta: Meta,
    pub simulation: Simulation,
    pub fidelity: Fidelity,
    /// Node definitions keyed by unique name; IndexMap preserves file order
    /// so node ids are deterministic and human-predictable.
    pub nodes: indexmap::IndexMap<String, NodeDef>,
    pub pipes: Vec<PipeDef>,
}

#[derive(Debug, Deserialize)]
pub struct Meta {
    pub name: String,
    #[serde(default)]
    pub description: String,
}

#[derive(Debug, Deserialize)]
pub struct Simulation {
    /// Tick length [s].
    pub dt: f64,
}

#[derive(Debug, Deserialize)]
pub struct Fidelity {
    /// "newton" | "simple"
    pub flow: String,
    /// "constant" (M1) — expands in M2+
    #[serde(default = "default_constant")]
    pub thermo: String,
    /// "none" (M1) — expands in M4
    #[serde(default = "default_none")]
    pub reactions: String,
}
fn default_constant() -> String {
    "constant".into()
}
fn default_none() -> String {
    "none".into()
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum NodeDef {
    Source {
        pressure_bar: f64,
        temperature_c: f64,
    },
    Sink {
        pressure_bar: f64,
    },
    Atmosphere,
    Tank {
        area_m2: f64,
        height_m: f64,
        initial_level_m: f64,
        temperature_c: f64,
    },
    Pump {
        h0_m: f64,
        a: f64,
        #[serde(default = "default_true")]
        on: bool,
    },
    Valve {
        kv: f64,
        opening: f64,
    },
    Junction,
}
fn default_true() -> bool {
    true
}

#[derive(Debug, Deserialize)]
pub struct PipeDef {
    pub name: String,
    pub from: String,
    pub to: String,
    pub length_m: f64,
    pub diameter_m: f64,
    #[serde(default = "default_friction")]
    pub friction_factor: f64,
    #[serde(default)]
    pub elevation_change_m: f64,
}
fn default_friction() -> f64 {
    0.02
}

pub fn load_str(toml_src: &str) -> Result<ScenarioFile, SimError> {
    toml::from_str(toml_src).map_err(|e| SimError::Scenario(e.to_string()))
}

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
    // Step 1: slate. Water-only until M3's [components] table exists; take the
    // water density from the slate itself so unit conversions can't drift from
    // the component definition.
    let slate = Slate::water_only();
    let water = Composition::pure(slate.len(), 0);
    let rho_water = water.mixture_density(&slate);

    // Step 2: instantiate nodes in file order (IndexMap preserves it, so node
    // ids are deterministic), then pipes — resolving names → NodeIds. Unit
    // conversion happens here, at the human-friendly ↔ SI boundary.
    let mut graph = PlantGraph::new();
    for (name, def) in &scenario.nodes {
        let kind = node_kind(def, &water, rho_water);
        graph.add_node(Node {
            name: name.clone(),
            kind,
            heat_input: Watt::ZERO,
        });
    }
    for pipe in &scenario.pipes {
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
        graph.add_pipe(
            from,
            to,
            Pipe {
                name: pipe.name.clone(),
                length: Meter(pipe.length_m),
                diameter: Meter(pipe.diameter_m),
                friction_factor: pipe.friction_factor,
                elevation_change: Meter(pipe.elevation_change_m),
                leak_area: SquareMeter::ZERO,
                // Isothermal water for M1; the solver overwrites mass_flow each
                // tick. Seed representative T/P at ambient / atmospheric.
                stream: Stream::stagnant(slate.len(), T_AMBIENT, P_ATM),
            },
        );
    }

    // Step 3: validate topology at load — a clear error here beats solve-time
    // divergence for the same structural fault.
    validate_topology(&graph, &slate)?;

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
    let thermo: Box<dyn ThermoModel> = Box::new(refinery_solvers::ConstantThermo);
    let reactions: Box<dyn ReactionModel> = Box::new(refinery_solvers::NoReactions);

    let config = EngineConfig {
        dt: refinery_core::units::Seconds(scenario.simulation.dt),
    };
    Ok(Engine::new(graph, slate, config, flow, thermo, reactions))
}

/// Convert a scenario node definition into a core `NodeKind`, applying the
/// human-friendly → SI conversions at this boundary (bar → Pa, °C → K,
/// metric Kv → element cv_si, tank level → mass). M1 is water-only, so every
/// material node carries the pure-water composition.
fn node_kind(def: &NodeDef, water: &Composition, rho_water: KgPerM3) -> NodeKind {
    match def {
        NodeDef::Source {
            pressure_bar,
            temperature_c,
        } => NodeKind::Source {
            pressure: bar_to_pa(*pressure_bar),
            temperature: c_to_k(*temperature_c),
            composition: water.clone(),
        },
        NodeDef::Sink { pressure_bar } => NodeKind::Sink {
            pressure: bar_to_pa(*pressure_bar),
        },
        NodeDef::Atmosphere => NodeKind::Atmosphere,
        NodeDef::Tank {
            area_m2,
            height_m,
            initial_level_m,
            temperature_c,
        } => {
            let area = SquareMeter(*area_m2);
            // m = ρ·A·h.
            let mass = Kg(rho_water.value() * area.value() * initial_level_m);
            NodeKind::Tank(TankState {
                area,
                height: Meter(*height_m),
                mass,
                temperature: c_to_k(*temperature_c),
                composition: water.clone(),
            })
        }
        NodeDef::Pump { h0_m, a, on } => NodeKind::Pump {
            h0: Meter(*h0_m),
            a: *a,
            on: *on,
        },
        NodeDef::Valve { kv, opening } => NodeKind::Valve {
            cv_max: kv_to_cv_si(*kv),
            opening: *opening,
        },
        NodeDef::Junction => NodeKind::Junction,
    }
}

fn bar_to_pa(bar: f64) -> Pascal {
    Pascal(bar * 1e5)
}
fn c_to_k(celsius: f64) -> Kelvin {
    Kelvin(celsius + 273.15)
}
/// Metric Kv [m³/h at ΔP = 1 bar, SG = 1] → element cv_si used by
/// `valve_flow`: Q[m³/s] = cv_si·opening·√(dP_Pa/ρ_rel). Starting from the
/// Kv definition Q[m³/h] = Kv·√(dP_bar/SG), convert h→s (÷3600) and bar→Pa
/// inside the root (÷√1e5): cv_si = Kv / (3600·√1e5).
fn kv_to_cv_si(kv: f64) -> f64 {
    kv / (3600.0 * 1e5_f64.sqrt())
}

/// Validate the built graph at load time so a structural fault fails here with
/// a clear message rather than as a divergent solve later.
fn validate_topology(graph: &PlantGraph, slate: &Slate) -> Result<(), SimError> {
    // (a) Pump/valve (1-inlet, 1-outlet) degree. Reuse the solver's canonical
    //     invariant — single owner, no drift — remapped to a load-time error.
    refinery_solvers::network::validate_degrees(graph)
        .map_err(|e| SimError::Scenario(e.to_string()))?;

    // (b) Every connected component must contain at least one pressure-fixing
    //     node (Source/Sink/Atmosphere/Tank), else its hydraulic problem is
    //     singular. Connectivity walks ALL pipes — NOT the solver's
    //     conducting-edge anchoring — so a valve closed at t=0 cannot falsely
    //     sever the network at load. Union-Find keyed on NodeId.0 (dense 0..n
    //     because the graph is freshly built with no removals).
    let ids: Vec<NodeId> = graph.node_ids().collect();
    let mut parent: Vec<usize> = (0..ids.len()).collect();
    for eid in graph.edge_ids() {
        let (a, b) = graph.endpoints(eid);
        uf_union(&mut parent, a.0 as usize, b.0 as usize);
    }
    // Per component root: whether it has a pressure fixer, and its lowest-id
    // member (deterministic representative for the error message).
    let mut has_fixer: BTreeMap<usize, bool> = BTreeMap::new();
    let mut representative: BTreeMap<usize, NodeId> = BTreeMap::new();
    for &id in &ids {
        let root = uf_find(&mut parent, id.0 as usize);
        let fixes = fixed_pressure_node(graph.node(id), slate);
        *has_fixer.entry(root).or_insert(false) |= fixes;
        representative.entry(root).or_insert(id);
    }
    for (root, fixed) in &has_fixer {
        if !*fixed {
            let member = representative[root];
            return Err(SimError::Scenario(format!(
                "network component containing '{}' has no pressure-fixing node \
                 (needs at least one source/sink/atmosphere/tank); its hydraulic \
                 problem is singular",
                graph.node(member).name
            )));
        }
    }
    Ok(())
}

/// True if the node pins a pressure (Source/Sink/Atmosphere/Tank). Delegates to
/// the solver's `fixed_pressure` so load-time and solve-time agree on what
/// counts as a boundary.
fn fixed_pressure_node(node: &Node, slate: &Slate) -> bool {
    refinery_solvers::network::fixed_pressure(node, slate).is_some()
}

fn uf_find(parent: &mut [usize], mut x: usize) -> usize {
    while parent[x] != x {
        parent[x] = parent[parent[x]]; // path halving
        x = parent[x];
    }
    x
}
fn uf_union(parent: &mut [usize], a: usize, b: usize) {
    let (ra, rb) = (uf_find(parent, a), uf_find(parent, b));
    if ra != rb {
        parent[ra] = rb;
    }
}

#[cfg(test)]
mod tests {
    /// The reference scenario file must always parse against the schema.
    #[test]
    fn reference_scenario_parses() {
        let src = include_str!("../../../scenarios/tank_pump_valve.toml");
        let s = super::load_str(src).expect("tank_pump_valve.toml must parse");
        assert_eq!(s.meta.name, "tank_pump_valve");
        assert_eq!(s.nodes.len(), 4);
        assert_eq!(s.pipes.len(), 3);
        assert_eq!(s.fidelity.flow, "newton");
        // IndexMap preserves file order → deterministic node ids.
        assert_eq!(s.nodes.get_index(0).unwrap().0, "supply_tank");
    }

    /// The reference plant must build into a wired graph and actually run:
    /// this is the end-to-end proof that node/pipe instantiation, unit
    /// conversions, and the fold-at-source device direction are all correct.
    #[test]
    fn build_engine_wires_and_runs_the_reference_plant() {
        use refinery_core::graph::NodeKind;

        let src = include_str!("../../../scenarios/tank_pump_valve.toml");
        let s = super::load_str(src).unwrap();
        let mut engine = super::build_engine(&s).expect("reference plant must build");
        assert_eq!(engine.graph.node_count(), 4);
        assert_eq!(engine.graph.edge_count(), 3);

        let receiving_mass = |e: &refinery_core::engine::Engine| {
            let id = e.graph.find_node("receiving_tank").unwrap();
            match &e.graph.node(id).kind {
                NodeKind::Tank(t) => t.mass.value(),
                _ => panic!("receiving_tank must be a tank"),
            }
        };
        let before = receiving_mass(&engine);

        // ~50 ticks exercises convergence + finiteness every tick (the 1000-tick
        // mass-balance run is a separate M1 acceptance gate). The pump drives
        // supply→receiving unambiguously (supply bottom ≈180 kPa, receiving
        // ≈111 kPa, pump adds up to ρg·40 ≈392 kPa against the 5 m ≈49 kPa
        // fill-line rise), so the receiving tank must gain mass.
        for _ in 0..50 {
            engine
                .tick()
                .expect("every tick must converge with no non-finite state");
        }
        let after = receiving_mass(&engine);
        assert!(
            after > before,
            "receiving tank must fill: before={before}, after={after}"
        );
    }

    #[test]
    fn unknown_solver_is_a_clear_error() {
        let src = include_str!("../../../scenarios/tank_pump_valve.toml");
        let mut s = super::load_str(src).unwrap();
        s.fidelity.flow = "quantum".into();
        let err = match super::build_engine(&s) {
            Err(e) => e,
            Ok(_) => panic!("expected an error for unknown solver"),
        };
        assert!(err.to_string().contains("quantum"));
    }
}
