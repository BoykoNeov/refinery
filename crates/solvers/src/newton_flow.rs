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

use crate::network::{
    classify, compile_edges, edge_flows, finalize, validate_degrees, CompiledEdge,
};
use refinery_core::components::Slate;
use refinery_core::error::SimError;
use refinery_core::graph::{EdgeId, NodeId, PlantGraph};
use refinery_core::traits::{FlowSolver, HydraulicSolution};
use refinery_core::units::{Seconds, P_ATM};
use std::collections::{BTreeMap, BTreeSet};

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

impl FlowSolver for NewtonFlowSolver {
    fn solve(
        &mut self,
        graph: &PlantGraph,
        slate: &Slate,
        _dt: Seconds,
    ) -> Result<HydraulicSolution, SimError> {
        // F6: pumps/valves must have exactly one inlet and one outlet edge.
        validate_degrees(graph)?;

        // Compile every edge's series branch (pipe ∘ device-at-source) and
        // classify the nodes (fixed/free, anchored set, cold-start seed) — both
        // shared with SimpleFlowSolver so the two fidelities agree by construction.
        let compiled = compile_edges(graph, slate)?;
        let cls = classify(graph, slate, &compiled);
        let anchored = &cls.anchored;
        let free = &cls.free;
        let cold = cls.cold;

        // Pin fixed pressures; free nodes seeded below.
        let mut pressures: BTreeMap<NodeId, f64> = cls.fixed.clone();

        // Newton unknowns = anchored free nodes, ascending (deterministic).
        let mut idx: BTreeMap<NodeId, usize> = BTreeMap::new();
        let mut unknowns: Vec<NodeId> = Vec::new();
        for &nid in free {
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
            let edges = edge_flows(graph, &compiled, &pressures, anchored, self.eps_dp);
            return finalize(&pressures, edges, 0, 0.0);
        }

        // Damped Newton.
        let mut history: Vec<f64> = Vec::new();
        let (mut r, mut jac, mut throughput) =
            assemble(graph, &compiled, &pressures, &idx, anchored, n, self.eps_dp);
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
                    assemble(graph, &compiled, &trial, &idx, anchored, n, self.eps_dp);
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
        for &nid in free {
            if let Some(&p) = pressures.get(&nid) {
                self.warm_start.insert(nid, p);
            }
        }
        let edges = edge_flows(graph, &compiled, &pressures, anchored, self.eps_dp);
        finalize(&pressures, edges, iterations, res)
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

/// Assemble the residual R and Jacobian J = ∂R/∂P over anchored free nodes.
/// Only ACTIVE edges (both endpoints anchored) contribute; edges touching a
/// floating node are inert (zero flow). Returns (R, J, throughput = max|ṁ|).
fn assemble(
    graph: &PlantGraph,
    compiled: &BTreeMap<EdgeId, CompiledEdge>,
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
