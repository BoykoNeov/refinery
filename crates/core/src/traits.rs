//! Solver traits — the fidelity seam.
//!
//! `core` defines WHAT must be computed; `refinery-solvers` provides HOW,
//! in simple (game) and complex (research) variants. Scenario config picks
//! implementations at engine build time. Shared engine code must never
//! branch on fidelity.

use crate::components::Slate;
use crate::error::SimError;
use crate::graph::{EdgeId, NodeId, PlantGraph};
use crate::units::{Pascal, Seconds};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Result of one hydraulic solve: node pressures + branch mass flows.
/// BTreeMap for deterministic iteration and stable serialization.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HydraulicSolution {
    pub node_pressure: BTreeMap<NodeId, Pascal>,
    /// Positive = flow in edge direction (source → target), kg/s.
    pub edge_mass_flow: BTreeMap<EdgeId, f64>,
    pub diagnostics: SolveDiagnostics,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SolveDiagnostics {
    pub iterations: u32,
    pub residual: f64,
    pub converged: bool,
}

/// Computes the quasi-steady pressure/flow field for the current network
/// state. Must be pure w.r.t. the graph (read-only); the engine applies
/// the solution to streams afterwards.
pub trait FlowSolver: Send {
    fn solve(
        &mut self,
        graph: &PlantGraph,
        slate: &Slate,
        dt: Seconds,
    ) -> Result<HydraulicSolution, SimError>;

    /// Human-readable identifier for snapshots/logs (e.g. "newton-network").
    fn name(&self) -> &'static str;
}

/// Physical property provider. M1 uses constant-property water; M2+ uses
/// composition/temperature-dependent models. Kept minimal on purpose —
/// extend when a consumer actually needs a property, not before.
pub trait ThermoModel: Send {
    fn name(&self) -> &'static str;
    // Density/cp currently live on Composition (ideal mixing). This trait
    // takes over when non-ideal or T-dependent behavior arrives (M2+),
    // at which point Composition's mixture_* helpers delegate here.
}

/// Chemical conversion inside reactor units. Implementations: NoReactions,
/// lookup-table (simple), FCC 4-lump kinetics (complex). Arrives in M4;
/// the trait exists now so the engine tick has its slot reserved.
pub trait ReactionModel: Send {
    fn name(&self) -> &'static str;
}
