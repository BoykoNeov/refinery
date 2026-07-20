//! Solver traits — the fidelity seam.
//!
//! `core` defines WHAT must be computed; `refinery-solvers` provides HOW,
//! in simple (game) and complex (research) variants. Scenario config picks
//! implementations at engine build time. Shared engine code must never
//! branch on fidelity.

use crate::components::{Composition, Slate};
use crate::error::SimError;
use crate::graph::{EdgeId, NodeId, PlantGraph};
use crate::units::{JPerKg, Kelvin, Pascal, Seconds};
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

/// The outcome of one reactor pass: the product composition and its heat of
/// reaction.
#[derive(Debug, Clone)]
pub struct Reaction {
    /// Product mass-fraction composition (Σ = 1). TOTAL mass is conserved — the
    /// reaction moves mass BETWEEN components, which is exactly what makes a
    /// reactor the first unit to break per-component conservation (I7).
    pub products: Composition,
    /// Specific heat of reaction [J per kg of feed]; positive = endothermic
    /// (heat absorbed). The reactor's reported physical duty adds `ṁ·Δh_rxn`
    /// to the emergent sensible duty (`energy::reactor_duty`).
    pub dh_rxn: JPerKg,
}

/// Chemical conversion inside reactor units. Implementations: `NoReactions`
/// (identity), `SimpleLookup` (conversion table), FCC 4-lump kinetics (M4.2).
///
/// The engine applies `react` INSIDE the energy sweep, so a downstream node
/// sees the product composition the same tick (DESIGN §5).
pub trait ReactionModel: Send {
    fn name(&self) -> &'static str;

    /// Convert one reactor pass's FEED into products at the reactor's held
    /// temperature `temperature` over residence time `tau`.
    ///
    /// Isothermal at a ROT setpoint (DESIGN §5): `temperature` is imposed by the
    /// reactor, so the extent is a pure function of a KNOWN temperature with no
    /// inner temperature solve. `tau` is unused by the lookup fidelity but is
    /// load-bearing for the M4.2 kinetics, which integrate `dC/dτ` over it — it
    /// is in the signature now so that additive swap needs no trait churn.
    ///
    /// `NoReactions` returns the feed unchanged with `Δh_rxn = 0`, so a network
    /// with no reactor node — every scenario before M4 — stays bit-identical.
    ///
    /// # Errors
    /// `SimError` if the model cannot produce a valid product composition (e.g.
    /// a table whose row does not normalize, or a feed lump it does not know).
    fn react(
        &self,
        feed: &Composition,
        temperature: Kelvin,
        tau: Seconds,
        slate: &Slate,
    ) -> Result<Reaction, SimError>;
}
