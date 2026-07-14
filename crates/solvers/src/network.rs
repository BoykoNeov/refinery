//! Shared network compilation — the single source of truth for the fidelity
//! seam. Both `NewtonFlowSolver` and `SimpleFlowSolver` compile the same graph
//! into the same branch characteristics (`elements::QuadraticBranch`), classify
//! nodes identically, and build the returned `HydraulicSolution` through the
//! same `edge_flows` + `finalize` (NaN-scan). Consequently the two fidelities
//! solve the *same* fixed point and their solutions share byte-identical
//! structure — a prerequisite for the I5 cross-fidelity agreement test. The
//! only intended difference between the solvers is HOW they drive the residual
//! to zero (dense Newton vs conductance-scaled relaxation), never the element
//! physics or the boundary classification.

use crate::elements::{pipe_resistance, QuadraticBranch};
use refinery_core::components::Slate;
use refinery_core::error::SimError;
use refinery_core::graph::{EdgeId, Node, NodeId, NodeKind, PlantGraph};
use refinery_core::traits::{HydraulicSolution, SolveDiagnostics};
use refinery_core::units::{Pascal, G, P_ATM};
use std::collections::{BTreeMap, BTreeSet};

/// Valve openings below this snap to fully closed, so a valve "cracked to
/// 1e-9" cannot anchor a subnetwork with a numerically negligible (near-
/// singular) conductance — it is treated as closed instead.
pub const OPEN_EPS: f64 = 1e-6;
/// Reference density for valve SG (ρ_rel). Matches `PseudoComponent::water`
/// so water gives ρ_rel = 1.0.
pub const RHO_WATER_REF: f64 = 998.0;

/// One edge compiled for the solve: its series branch, transport density, and
/// whether it conducts (open path) for connectivity.
pub struct CompiledEdge {
    pub src: NodeId,
    pub tgt: NodeId,
    pub branch: QuadraticBranch,
    /// Transport density [kg/m³] (ṁ = ρ·Q).
    pub rho: f64,
    /// True if the branch can carry flow (α finite & > 0); false = closed.
    pub conducts: bool,
}

/// Boundary classification of the graph's nodes for one solve: which nodes pin
/// pressure, which are free unknowns, which are anchored (reachable from a fixed
/// node through conducting edges), and the deterministic cold-start seed.
pub struct Classification {
    /// Pinned pressures [Pa] for fixed nodes (Source/Sink/Atmosphere/Tank).
    pub fixed: BTreeMap<NodeId, f64>,
    /// Free node ids (Junction/Pump/Valve), ascending — deterministic order.
    pub free: Vec<NodeId>,
    /// Fixed ∪ free-reachable-via-conducting-edges. Free nodes NOT in this set
    /// are floating (indeterminate pressure); their incident edges carry zero.
    pub anchored: BTreeSet<NodeId>,
    /// Deterministic cold-start pressure seed: mean of the fixed pressures, or
    /// P_ATM when the network has no fixed node at all.
    pub cold: f64,
}

/// F6: every Pump/Valve node must have exactly one inlet and one outlet edge.
pub fn validate_degrees(graph: &PlantGraph) -> Result<(), SimError> {
    for nid in graph.node_ids() {
        let node = graph.node(nid);
        if matches!(node.kind, NodeKind::Pump { .. } | NodeKind::Valve { .. }) {
            let inc = graph.incident(nid);
            let n_in = inc.iter().filter(|(_, _, incoming)| *incoming).count();
            let n_out = inc.len() - n_in;
            if n_in != 1 || n_out != 1 {
                return Err(SimError::Numerical(format!(
                    "{} ({:?}) must have exactly 1 inlet + 1 outlet edge, has {n_in} in / {n_out} out",
                    node.name, nid
                )));
            }
        }
    }
    Ok(())
}

/// Pinned pressure for a fixed node, or None if the node is free.
pub fn fixed_pressure(node: &Node, slate: &Slate) -> Option<f64> {
    match &node.kind {
        NodeKind::Source { pressure, .. } => Some(pressure.value()),
        NodeKind::Sink { pressure } => Some(pressure.value()),
        NodeKind::Atmosphere => Some(P_ATM.value()),
        NodeKind::Tank(t) => {
            let rho = t.composition.mixture_density(slate);
            Some(t.bottom_pressure(rho).value())
        }
        NodeKind::Pump { .. } | NodeKind::Valve { .. } | NodeKind::Junction => None,
    }
}

/// Compile one edge into its series branch. The device (if any) at the edge's
/// SOURCE node folds into this outlet edge, per the fold-at-source convention.
pub fn compile_edge(
    graph: &PlantGraph,
    eid: EdgeId,
    slate: &Slate,
) -> Result<CompiledEdge, SimError> {
    let (src, tgt) = graph.endpoints(eid);
    let pipe = graph.pipe(eid);
    let rho = pipe.stream.composition.mixture_density(slate).value();
    // Darcy–Weisbach resistance; a valid pipe always contributes k > 0, which
    // keeps α_tot > 0 so the closed-form inverse never divides by zero.
    let k = pipe_resistance(
        pipe.friction_factor,
        pipe.length.value(),
        pipe.diameter.value(),
        rho,
    );
    if !k.is_finite() || k <= 0.0 {
        return Err(SimError::Numerical(format!(
            "pipe {} ({eid:?}) has non-positive resistance k={k:.3e} (bad length/diameter/friction/ρ)",
            pipe.name
        )));
    }
    // Static head β = ρ·g·Δz (Δz = downstream − upstream elevation).
    let elev_head = rho * G * pipe.elevation_change.value();
    let mut branch = QuadraticBranch::pipe(k, elev_head);

    match &graph.node(src).kind {
        NodeKind::Pump { h0, a, on } => {
            let h0_eff = if *on { h0.value() } else { 0.0 };
            branch = branch.in_series(QuadraticBranch::pump(h0_eff, *a, rho, G));
        }
        NodeKind::Valve { cv_max, opening } => {
            let op = if *opening < OPEN_EPS { 0.0 } else { *opening };
            let rho_rel = rho / RHO_WATER_REF;
            branch = branch.in_series(QuadraticBranch::valve(*cv_max, op, rho_rel));
        }
        _ => {}
    }

    let conducts = branch.alpha.is_finite() && branch.alpha > 0.0;
    Ok(CompiledEdge {
        src,
        tgt,
        branch,
        rho,
        conducts,
    })
}

/// Compile every edge's series branch (pipe ∘ device-at-source), keyed by edge.
pub fn compile_edges(
    graph: &PlantGraph,
    slate: &Slate,
) -> Result<BTreeMap<EdgeId, CompiledEdge>, SimError> {
    let mut compiled = BTreeMap::new();
    for eid in graph.edge_ids() {
        compiled.insert(eid, compile_edge(graph, eid, slate)?);
    }
    Ok(compiled)
}

/// Classify nodes into fixed/free, compute the anchored set and the cold-start
/// seed. Deterministic: node ids iterate ascending, `free` is ascending, and
/// the seed is a pure function of the pinned pressures.
pub fn classify(
    graph: &PlantGraph,
    slate: &Slate,
    compiled: &BTreeMap<EdgeId, CompiledEdge>,
) -> Classification {
    let mut fixed: BTreeMap<NodeId, f64> = BTreeMap::new();
    let mut free: Vec<NodeId> = Vec::new();
    let (mut fixed_sum, mut fixed_cnt) = (0.0, 0usize);
    for nid in graph.node_ids() {
        if let Some(p) = fixed_pressure(graph.node(nid), slate) {
            fixed.insert(nid, p);
            fixed_sum += p;
            fixed_cnt += 1;
        } else {
            free.push(nid);
        }
    }
    // Cold start for nodes without a warm-start value (uniqueness makes the
    // seed affect only the iterate path, never the answer).
    let cold = if fixed_cnt > 0 {
        fixed_sum / fixed_cnt as f64
    } else {
        P_ATM.value()
    };
    let fixed_set: BTreeSet<NodeId> = fixed.keys().copied().collect();
    let anchored = anchored_set(graph, compiled, &fixed_set);
    Classification {
        fixed,
        free,
        anchored,
        cold,
    }
}

/// Nodes reachable from any fixed node through conducting edges (undirected).
/// Free nodes NOT in this set are floating (indeterminate pressure).
pub fn anchored_set(
    graph: &PlantGraph,
    compiled: &BTreeMap<EdgeId, CompiledEdge>,
    fixed: &BTreeSet<NodeId>,
) -> BTreeSet<NodeId> {
    let mut adj: BTreeMap<NodeId, Vec<NodeId>> = BTreeMap::new();
    for eid in graph.edge_ids() {
        let c = &compiled[&eid];
        if c.conducts {
            adj.entry(c.src).or_default().push(c.tgt);
            adj.entry(c.tgt).or_default().push(c.src);
        }
    }
    let mut anchored = fixed.clone();
    let mut stack: Vec<NodeId> = fixed.iter().copied().collect();
    while let Some(n) = stack.pop() {
        if let Some(neigh) = adj.get(&n) {
            for &m in neigh {
                if anchored.insert(m) {
                    stack.push(m);
                }
            }
        }
    }
    anchored
}

/// Mass flow (kg/s) per edge in graph direction; inert edges (either endpoint
/// unanchored) report 0. Returns (flows, throughput = max|ṁ|).
pub fn edge_flows(
    graph: &PlantGraph,
    compiled: &BTreeMap<EdgeId, CompiledEdge>,
    pressures: &BTreeMap<NodeId, f64>,
    anchored: &BTreeSet<NodeId>,
    eps: f64,
) -> (BTreeMap<EdgeId, f64>, f64) {
    let mut flows = BTreeMap::new();
    let mut throughput = 0.0f64;
    for eid in graph.edge_ids() {
        let c = &compiled[&eid];
        let mdot = if anchored.contains(&c.src) && anchored.contains(&c.tgt) {
            let dp = pressures[&c.src] - pressures[&c.tgt];
            c.rho * c.branch.flow(dp, eps)
        } else {
            0.0
        };
        throughput = throughput.max(mdot.abs());
        flows.insert(eid, mdot);
    }
    (flows, throughput)
}

/// Build the solution with a final NaN/Inf scan (rule 5: nothing non-finite
/// escapes a solve). Only called on a converged solve, hence `converged: true`.
pub fn finalize(
    pressures: &BTreeMap<NodeId, f64>,
    flows: BTreeMap<EdgeId, f64>,
    iterations: u32,
    residual: f64,
) -> Result<HydraulicSolution, SimError> {
    let mut node_pressure = BTreeMap::new();
    for (nid, p) in pressures {
        if !p.is_finite() {
            return Err(SimError::NonFiniteState {
                location: format!("{nid:?} pressure"),
            });
        }
        node_pressure.insert(*nid, Pascal(*p));
    }
    for (eid, f) in &flows {
        if !f.is_finite() {
            return Err(SimError::NonFiniteState {
                location: format!("{eid:?} mass flow"),
            });
        }
    }
    Ok(HydraulicSolution {
        node_pressure,
        edge_mass_flow: flows,
        diagnostics: SolveDiagnostics {
            iterations,
            residual,
            converged: true,
        },
    })
}
