//! SimpleFlowSolver — game fidelity. No global solve: each branch's flow is
//! computed from the LAST tick's endpoint pressures via the same element
//! characteristics as the Newton solver, then relaxed:
//!
//! ```text
//! flow_new = flow_old + alpha * (flow_characteristic(dp_old) - flow_old)
//! ```
//!
//! Node pressures update from local mass imbalance (pseudo-compressibility):
//!
//! ```text
//! p_new = p_old + beta * imbalance
//! ```
//!
//! Properties: O(edges) per tick, unconditionally cheap, stable for
//! reasonable α/β, NOT conservative to machine precision (acceptable at
//! game fidelity — document the expected imbalance bound in tests).
//! It intentionally shares elements.rs with the Newton solver so both
//! fidelities agree on steady states to within tolerance (a required
//! cross-check test in M1).

use refinery_core::components::Slate;
use refinery_core::error::SimError;
use refinery_core::graph::PlantGraph;
use refinery_core::traits::{FlowSolver, HydraulicSolution};
use refinery_core::units::Seconds;
use std::collections::BTreeMap;

pub struct SimpleFlowSolver {
    /// Flow relaxation factor (0,1].
    pub alpha: f64,
    /// Pressure correction factor [Pa per (kg/s) of imbalance].
    pub beta: f64,
    state_pressure: BTreeMap<refinery_core::graph::NodeId, f64>,
    state_flow: BTreeMap<refinery_core::graph::EdgeId, f64>,
}

impl Default for SimpleFlowSolver {
    fn default() -> Self {
        Self {
            alpha: 0.3,
            beta: 1e4,
            state_pressure: BTreeMap::new(),
            state_flow: BTreeMap::new(),
        }
    }
}

impl FlowSolver for SimpleFlowSolver {
    fn solve(
        &mut self,
        graph: &PlantGraph,
        slate: &Slate,
        _dt: Seconds,
    ) -> Result<HydraulicSolution, SimError> {
        let _ = (graph, slate, &mut self.state_pressure, &mut self.state_flow);
        todo!("M1: relaxation solve — see module docs")
    }

    fn name(&self) -> &'static str {
        "simple-relaxation"
    }
}
