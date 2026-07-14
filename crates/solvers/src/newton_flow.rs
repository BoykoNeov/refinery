//! NetworkFlowSolver — complex fidelity. Quasi-steady network hydraulics by
//! Newton–Raphson on node pressures (docs/DESIGN.md §3).
//!
//! Formulation (advisor-settled: folded combined-branch, option (b)):
//! - Unknowns: pressure P_i at every FREE node (Junction, Pump, Valve). The
//!   pump/valve device is a NODE with exactly one in- and one out-edge; its
//!   single unknown pressure is its **inlet/suction port**, and the device's
//!   pressure relation is folded into its OUTLET edge's branch characteristic
//!   (`QuadraticBranch::in_series`). So a reported pump/valve `node_pressure`
//!   is the suction pressure — the discharge is `P_inlet + ρgH(Q)`, computed
//!   in post-processing if a frontend ever needs it (not stored here).
//! - FIXED nodes pin pressure: Source/Sink at their set pressure, Atmosphere
//!   at P_ATM, Tank at hydrostatic bottom pressure (constant within one solve).
//! - Every M1 element is affine in `Q·|Q|` (`dp = α·Q|Q| + β`), so a pipe and
//!   the device folded into it compose in closed form (`Σα, Σβ`) and invert to
//!   `Q(dp)` with no inner solve — see `elements::QuadraticBranch`.
//! - Residual: mass balance at every free node,
//!   `R_i = Σ_incoming ṁ − Σ_outgoing ṁ = 0`, ṁ = ρ·Q(dP_branch).
//! - Jacobian: analytic. With branch conductance `g_e = ρ·dQ/d(dp) ≥ 0`, the
//!   system is `J = −L`, a weighted graph Laplacian: symmetric, negative-
//!   definite once each connected component has a pinned node ⇒ unique
//!   solution. faer dense LU (networks are small; sparse is a later upgrade).
//! - Damping: halve the Newton step until ‖R‖_∞ decreases, max 8 halvings.
//! - Convergence: ‖R‖_∞ < tol_abs + tol_rel·throughput (throughput = max|ṁ_e|),
//!   per DESIGN §3's relative criterion; hard cap max_iter, then
//!   Err(SolverDiverged) carrying the residual history. NEVER return NaN.
//!
//! Reverse flow: a single element's characteristic is odd in dP, but a
//! *combined* branch with a pump or elevation head (β ≠ 0) is odd about
//! `dp = β`, NOT about 0. Reverse-flow tests assert this shifted symmetry.
//!
//! Floating subnetworks (F2): a free node reachable from a fixed node ONLY
//! through a closed valve is not anchored — its pressure is indeterminate.
//! Openings below `OPEN_EPS` snap to fully closed, so "conducting" is a static
//! property of the compiled branch (recomputed each solve as openings change).
//! Unanchored free nodes are pinned (warm-start or P_ATM), excluded from the
//! Newton system, and every edge touching one reports zero flow. This is a
//! legitimate, frequent game state (operator closes a valve), not an error.

use crate::elements::{pipe_resistance, QuadraticBranch};
use refinery_core::components::Slate;
use refinery_core::error::SimError;
use refinery_core::graph::{EdgeId, Node, NodeId, NodeKind, PlantGraph};
use refinery_core::traits::{FlowSolver, HydraulicSolution, SolveDiagnostics};
use refinery_core::units::{Pascal, Seconds, G, P_ATM};
use std::collections::{BTreeMap, BTreeSet};

/// Valve openings below this snap to fully closed, so a valve "cracked to
/// 1e-9" cannot anchor a subnetwork with a numerically negligible (near-
/// singular) conductance — it is treated as closed instead.
const OPEN_EPS: f64 = 1e-6;
/// Reference density for valve SG (ρ_rel). Matches `PseudoComponent::water`
/// so water gives ρ_rel = 1.0.
const RHO_WATER_REF: f64 = 998.0;
/// Max damped-halvings per Newton step (min step 1/256).
const MAX_HALVINGS: u32 = 8;
/// Armijo sufficient-decrease coefficient for the line search.
const ARMIJO_C: f64 = 1e-4;

pub struct NewtonFlowSolver {
    pub max_iter: u32,
    /// Absolute residual tolerance floor [kg/s].
    pub tol_abs_kg_s: f64,
    /// Relative residual tolerance (× network throughput) [kg/s per kg/s].
    pub tol_rel: f64,
    /// Regularization epsilon for sqrt laws [Pa].
    pub eps_dp: f64,
    /// Warm-start pressures from the previous converged solve, keyed by NodeId.
    warm_start: BTreeMap<NodeId, f64>,
}

impl Default for NewtonFlowSolver {
    fn default() -> Self {
        Self {
            max_iter: 50,
            tol_abs_kg_s: 1e-8,
            tol_rel: 1e-8,
            eps_dp: 1.0,
            warm_start: BTreeMap::new(),
        }
    }
}

/// One edge compiled for the solve: its series branch, transport density, and
/// whether it conducts (open path) for connectivity.
struct Compiled {
    src: NodeId,
    tgt: NodeId,
    branch: QuadraticBranch,
    /// Transport density [kg/m³] (ṁ = ρ·Q).
    rho: f64,
    /// True if the branch can carry flow (α finite & > 0); false = closed.
    conducts: bool,
}

impl FlowSolver for NewtonFlowSolver {
    fn solve(
        &mut self,
        graph: &PlantGraph,
        slate: &Slate,
        _dt: Seconds,
    ) -> Result<HydraulicSolution, SimError> {
        // F6: pumps/valves must have exactly one inlet and one outlet edge.
        validate_degrees(graph)?;

        // Compile every edge's series branch (pipe ∘ device-at-source).
        let mut compiled: BTreeMap<EdgeId, Compiled> = BTreeMap::new();
        for eid in graph.edge_ids() {
            compiled.insert(eid, compile_branch(graph, eid, slate)?);
        }

        // Classify nodes; pin fixed pressures.
        let mut pressures: BTreeMap<NodeId, f64> = BTreeMap::new();
        let mut fixed: BTreeSet<NodeId> = BTreeSet::new();
        let mut free: Vec<NodeId> = Vec::new();
        let (mut fixed_sum, mut fixed_cnt) = (0.0, 0usize);
        for nid in graph.node_ids() {
            if let Some(p) = fixed_pressure(graph.node(nid), slate) {
                pressures.insert(nid, p);
                fixed.insert(nid);
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

        // F2: anchored = fixed ∪ free reachable via CONDUCTING edges.
        let anchored = anchored_set(graph, &compiled, &fixed);

        // Newton unknowns = anchored free nodes, ascending (deterministic).
        let mut idx: BTreeMap<NodeId, usize> = BTreeMap::new();
        let mut unknowns: Vec<NodeId> = Vec::new();
        for &nid in &free {
            let seed = if anchored.contains(&nid) {
                idx.insert(nid, unknowns.len());
                unknowns.push(nid);
                self.warm_start.get(&nid).copied().unwrap_or(cold)
            } else {
                // Floating: pinned; its incident edges report zero flow.
                self.warm_start.get(&nid).copied().unwrap_or(P_ATM.value())
            };
            pressures.insert(nid, seed);
        }
        let n = unknowns.len();

        // Trivial: no unknowns (all pinned, or every free node floating) ⇒
        // flows are determined directly. A network with NO fixed node
        // (fixed_cnt == 0) lands here as a benign all-P_ATM, zero-flow Ok; the
        // scenario loader is responsible for rejecting components that lack a
        // pressure reference, so the solver stays lenient rather than Err'ing.
        if n == 0 {
            let (flows, _) = edge_flows(graph, &compiled, &pressures, &anchored, self.eps_dp);
            return finalize(&pressures, flows, 0, 0.0);
        }

        // Damped Newton.
        let mut history: Vec<f64> = Vec::new();
        let (mut r, mut jac, mut throughput) = assemble(
            graph,
            &compiled,
            &pressures,
            &idx,
            &anchored,
            n,
            self.eps_dp,
        );
        let mut res = inf_norm(&r); // ∞-norm: convergence + reporting (per-node imbalance)
        let mut merit = half_sq_norm(&r); // ½‖R‖₂²: smooth line-search merit
        history.push(res);
        let mut iterations = 0u32;
        let converged_at = |res: f64, tp: f64| res < self.tol_abs_kg_s + self.tol_rel * tp;
        let mut converged = converged_at(res, throughput);

        while !converged && iterations < self.max_iter {
            iterations += 1;

            // Newton direction: J·ΔP = −R.
            let neg_r: Vec<f64> = r.iter().map(|x| -x).collect();
            let dp = solve_linear(&jac, &neg_r);
            if dp.iter().any(|x| !x.is_finite()) {
                // Singular/ill-conditioned Jacobian (e.g. an F2 case slipped
                // through) — surfaces here first.
                return Err(diverged(iterations, res, history));
            }

            // Damped line search with the Armijo sufficient-decrease condition
            // on φ = ½‖R‖₂². The exact Newton step is a descent direction with
            // φ'(0) = −‖R‖₂² = −2φ, so we require φ_t ≤ (1 − 2·c·t)·φ. Merely
            // requiring "any decrease" would accept the √-law's near-symmetric
            // overshoot (t=1) and stall; Armijo rejects it and forces t≤½.
            let mut t = 1.0;
            let mut accepted = false;
            for _ in 0..=MAX_HALVINGS {
                let trial = apply_step(&pressures, &unknowns, &idx, &dp, t);
                let (r_t, jac_t, tp_t) =
                    assemble(graph, &compiled, &trial, &idx, &anchored, n, self.eps_dp);
                let merit_t = half_sq_norm(&r_t);
                if merit_t <= (1.0 - 2.0 * ARMIJO_C * t) * merit {
                    pressures = trial;
                    res = inf_norm(&r_t);
                    merit = merit_t;
                    r = r_t;
                    jac = jac_t;
                    throughput = tp_t;
                    accepted = true;
                    break;
                }
                t *= 0.5;
            }
            history.push(res);
            if !accepted {
                // The exact Newton step is always a descent direction for a
                // C¹ residual; failure to decrease means singularity, not a
                // line-search deficiency (advisor Q4).
                return Err(diverged(iterations, res, history));
            }
            converged = converged_at(res, throughput);
        }

        if !converged {
            return Err(diverged(iterations, res, history));
        }

        // Converged: update warm start (converged-only), then build solution.
        for &nid in &free {
            if let Some(&p) = pressures.get(&nid) {
                self.warm_start.insert(nid, p);
            }
        }
        let (flows, _) = edge_flows(graph, &compiled, &pressures, &anchored, self.eps_dp);
        finalize(&pressures, flows, iterations, res)
    }

    fn name(&self) -> &'static str {
        "newton-network"
    }
}

// ---------------------------------------------------------------------------
// Pure helpers (each independently unit-testable).
// ---------------------------------------------------------------------------

fn diverged(iterations: u32, residual: f64, residual_history: Vec<f64>) -> SimError {
    SimError::SolverDiverged {
        iterations,
        residual,
        residual_history,
    }
}

/// F6: every Pump/Valve node must have exactly one inlet and one outlet edge.
fn validate_degrees(graph: &PlantGraph) -> Result<(), SimError> {
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
fn fixed_pressure(node: &Node, slate: &Slate) -> Option<f64> {
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
/// SOURCE node folds into this outlet edge, per the module convention.
fn compile_branch(graph: &PlantGraph, eid: EdgeId, slate: &Slate) -> Result<Compiled, SimError> {
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
    Ok(Compiled {
        src,
        tgt,
        branch,
        rho,
        conducts,
    })
}

/// Nodes reachable from any fixed node through conducting edges (undirected).
/// Free nodes NOT in this set are floating (indeterminate pressure).
fn anchored_set(
    graph: &PlantGraph,
    compiled: &BTreeMap<EdgeId, Compiled>,
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

/// Assemble the residual R and Jacobian J = ∂R/∂P over anchored free nodes.
/// Only ACTIVE edges (both endpoints anchored) contribute; edges touching a
/// floating node are inert (zero flow). Returns (R, J, throughput = max|ṁ|).
fn assemble(
    graph: &PlantGraph,
    compiled: &BTreeMap<EdgeId, Compiled>,
    pressures: &BTreeMap<NodeId, f64>,
    idx: &BTreeMap<NodeId, usize>,
    anchored: &BTreeSet<NodeId>,
    n: usize,
    eps: f64,
) -> (Vec<f64>, Vec<Vec<f64>>, f64) {
    let mut r = vec![0.0; n];
    let mut jac = vec![vec![0.0; n]; n];
    let mut throughput = 0.0f64;
    for eid in graph.edge_ids() {
        let c = &compiled[&eid];
        if !(anchored.contains(&c.src) && anchored.contains(&c.tgt)) {
            continue;
        }
        let dp = pressures[&c.src] - pressures[&c.tgt];
        let mdot = c.rho * c.branch.flow(dp, eps);
        let g = c.rho * c.branch.flow_ddp(dp, eps); // conductance ≥ 0
        throughput = throughput.max(mdot.abs());
        let si = idx.get(&c.src).copied();
        let ti = idx.get(&c.tgt).copied();
        // R_src -= ṁ (outgoing), R_tgt += ṁ (incoming); J = −L.
        if let Some(s) = si {
            r[s] -= mdot;
            jac[s][s] -= g;
            if let Some(t) = ti {
                jac[s][t] += g;
            }
        }
        if let Some(t) = ti {
            r[t] += mdot;
            jac[t][t] -= g;
            if let Some(s) = si {
                jac[t][s] += g;
            }
        }
    }
    (r, jac, throughput)
}

/// Mass flow (kg/s) per edge in graph direction; inert edges report 0.
fn edge_flows(
    graph: &PlantGraph,
    compiled: &BTreeMap<EdgeId, Compiled>,
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

/// Copy `pressures`, advancing each unknown by `t·ΔP`.
fn apply_step(
    pressures: &BTreeMap<NodeId, f64>,
    unknowns: &[NodeId],
    idx: &BTreeMap<NodeId, usize>,
    dp: &[f64],
    t: f64,
) -> BTreeMap<NodeId, f64> {
    let mut out = pressures.clone();
    for &nid in unknowns {
        *out.get_mut(&nid).expect("unknown is a node") += t * dp[idx[&nid]];
    }
    out
}

fn inf_norm(v: &[f64]) -> f64 {
    v.iter().fold(0.0f64, |m, x| m.max(x.abs()))
}

/// ½‖v‖₂² — the smooth merit function minimized by the Newton line search.
fn half_sq_norm(v: &[f64]) -> f64 {
    0.5 * v.iter().map(|x| x * x).sum::<f64>()
}

/// Dense LU solve of J·x = b via faer. Singular systems yield non-finite x,
/// caught by the caller's finiteness check (never a panic, never a NaN escape).
///
/// Determinism (rule 3): faer parallelizes large LU solves via rayon, and
/// parallel float reduction is order-nondeterministic. We pin global
/// parallelism to None so the factorization is bit-reproducible regardless of
/// network size — M1 networks are small enough that faer stays serial anyway,
/// but a later large scenario must not silently break snapshot determinism.
fn solve_linear(jac: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    use faer::prelude::*;
    static SERIAL: std::sync::Once = std::sync::Once::new();
    SERIAL.call_once(|| faer::set_global_parallelism(faer::Parallelism::None));

    let n = b.len();
    let a = faer::Mat::from_fn(n, n, |i, k| jac[i][k]);
    let rhs = faer::Mat::from_fn(n, 1, |i, _| b[i]);
    let x = a.partial_piv_lu().solve(&rhs);
    (0..n).map(|i| x[(i, 0)]).collect()
}

/// Build the solution with a final NaN/Inf scan (rule 5: nothing non-finite
/// escapes a solve).
fn finalize(
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
