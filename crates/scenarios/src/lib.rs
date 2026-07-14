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

use refinery_core::components::Slate;
use refinery_core::engine::{Engine, EngineConfig};
use refinery_core::error::SimError;
use refinery_core::graph::PlantGraph;
use refinery_core::traits::{FlowSolver, ReactionModel, ThermoModel};
use serde::Deserialize;

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
    let slate = Slate::water_only();
    let graph = PlantGraph::new();
    let _ = (&scenario.nodes, &scenario.pipes); // consumed by step 2

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

    // TODO(M1): steps 2 and 3 per the doc comment above.
    let config = EngineConfig {
        dt: refinery_core::units::Seconds(scenario.simulation.dt),
    };
    Ok(Engine::new(graph, slate, config, flow, thermo, reactions))
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
