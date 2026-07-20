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

/// Every Pump/Valve/Furnace/Cooler/HeatExchanger node must have one inlet and
/// one outlet edge — but for two different reasons, which is worth keeping straight:
///
/// - **Pump/Valve (F6):** a *hydraulic* constraint. Their characteristic folds
///   into the single outlet edge (`compile_edge`'s fold-at-source convention),
///   which is only well defined when there is exactly one of each.
/// - **Furnace/Cooler:** a *process* constraint. Their duty heats or cools "the
///   stream through it", and that phrase only names something with one defined
///   process stream. The mixing formula would happily average N inlets, so
///   nothing numerical forces this — it is rejected because a branched heater
///   means the scenario author meant something the model does not represent.
/// - **HeatExchanger:** the same process constraint, per SIDE. "The stream
///   through this side" is what the ΔT-effectiveness model transfers heat
///   between, and a branched side would leave `C_min` naming nothing definite.
/// - **Column:** the ONE unit that is not 1-in-1-out. It needs exactly one feed
///   (inlet) but exactly `draws.len()` outlets, one per draw. `ṁ_drawᵢ = splitᵢ
///   · ṁ_feed` names one definite feed stream to split; two feeds would leave
///   "the feed's boiling range" ambiguous, and a draw with no outlet edge would
///   silently drop `splitᵢ · ṁ_feed` and break mass conservation.
pub fn validate_degrees(graph: &PlantGraph) -> Result<(), SimError> {
    for nid in graph.node_ids() {
        let node = graph.node(nid);
        let inc = graph.incident(nid);
        let n_in = inc.iter().filter(|(_, _, incoming)| *incoming).count();
        let n_out = inc.len() - n_in;
        let expected: Option<(usize, usize)> = match &node.kind {
            NodeKind::Pump { .. }
            | NodeKind::Valve { .. }
            | NodeKind::Furnace { .. }
            | NodeKind::Cooler { .. }
            // A reactor is the furnace's process constraint too: "the stream it
            // converts" names one definite feed, so it is 1-in-1-out.
            | NodeKind::Reactor { .. }
            | NodeKind::HeatExchanger => Some((1, 1)),
            // One feed, one outlet per draw. The feed is stored INTO the column
            // and each draw OUT of it (the loader enforces the direction), so the
            // graph-direction in/out counts are exactly (1, N).
            NodeKind::Column { draws, .. } => Some((1, draws.len())),
            _ => None,
        };
        if let Some((want_in, want_out)) = expected {
            if n_in != want_in || n_out != want_out {
                return Err(SimError::Numerical(format!(
                    "{} ({:?}) must have exactly {want_in} inlet + {want_out} outlet edge(s), \
                     has {n_in} in / {n_out} out",
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
        NodeKind::Sink { pressure, .. } => Some(pressure.value()),
        NodeKind::Atmosphere => Some(P_ATM.value()),
        NodeKind::Tank(t) => {
            let rho = t.composition.mixture_density(slate);
            Some(t.bottom_pressure(rho).value())
        }
        // A column runs on pressure control: its operating pressure is pinned,
        // exactly like a Source/Sink/Tank, so the feed edge is an ordinary
        // pressure-driven edge into a fixed node and the draws (fixed→fixed) never
        // enter the Jacobian (DESIGN §5).
        NodeKind::Column { pressure, .. } => Some(pressure.value()),
        // Furnaces, coolers and reactors pin no pressure: all are hydraulically
        // pass-throughs, so they are free nodes whose pressure the network
        // determines. A reactor is hydraulically a furnace (DESIGN §5).
        NodeKind::Pump { .. }
        | NodeKind::Valve { .. }
        | NodeKind::Junction
        | NodeKind::Furnace { .. }
        | NodeKind::Cooler { .. }
        | NodeKind::Reactor { .. }
        | NodeKind::HeatExchanger => None,
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
///
/// **Column draw edges are guarded to zero here, not computed.** A draw's flow is
/// `splitᵢ · ṁ_feed`, prescribed by the feed's composition, and both endpoints
/// (column and product tank) are fixed reservoirs — so the pressure-driven
/// `ρ·branch.flow(dp)` this function would otherwise report is a finite,
/// deterministic, mass-conserving *wrong* number that nothing downstream flags
/// (DESIGN §5's silent hazard). It also has the wrong SIGN when a product tank
/// fills above the column pressure, which would feed the sweep a spurious inflow
/// and corrupt the column's mix. The authoritative draw flow is written
/// post-sweep by `Engine::tick`, once the feed composition is resolved; here we
/// only refuse to leak a bogus value. This needs no slate or composition — only
/// the graph topology telling a draw edge from an ordinary one.
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
        let mdot = if is_column_draw_edge(graph, eid) {
            0.0
        } else if anchored.contains(&c.src) && anchored.contains(&c.tgt) {
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

/// True if `edge` is a column draw: one of its endpoints is a `Column` and the
/// *other* endpoint is one of that column's draw outlets. The feed edge — whose
/// far endpoint is the feeder, not a draw — is not matched, so it stays an
/// ordinary pressure-driven edge.
pub fn is_column_draw_edge(graph: &PlantGraph, edge: EdgeId) -> bool {
    let (from, to) = graph.endpoints(edge);
    for (maybe_column, other) in [(from, to), (to, from)] {
        if let NodeKind::Column { draws, .. } = &graph.node(maybe_column).kind {
            if draws.iter().any(|d| d.outlet == other) {
                return true;
            }
        }
    }
    false
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
