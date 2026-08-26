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
    accumulation, compile_edges, edge_flows, finalize, solve_with_active_anchoring,
    validate_degrees, AnchorPass, Capacitance, CompiledEdge, Prepared,
};
use refinery_core::components::Slate;
use refinery_core::energy::NodeStates;
use refinery_core::error::SimError;
use refinery_core::graph::{EdgeId, NodeId, PlantGraph};
use refinery_core::traits::{FlowSolver, HydraulicSolution};
use refinery_core::units::Seconds;
use std::collections::{BTreeMap, BTreeSet};

/// Max damped-halvings per Newton step (min step 1/256).
const MAX_HALVINGS: u32 = 8;
/// Armijo sufficient-decrease coefficient for the line search.
///
/// **This constant is coupled to [`NewtonFlowSolver::max_iter`], and the coupling
/// is the whole of M9.0** (DESIGN §11). Newton applied to the regularised
/// square-root branch law overshoots to the *mirror* of its own drop, shrunk by
/// `2·eps_dp`, so a full step that Armijo accepts walks toward the root at two
/// pascals per iteration for ever. A shut valve leaves a dead leg with exactly
/// one live edge (F6), so the whole failing solve is that scalar case, and it
/// stalls precisely when
///
/// ```text
/// 2·eps_dp·max_iter   <   |Δp₀|   ≲   eps_dp / ARMIJO_C
/// ```
///
/// `eps_dp` cancels — shrinking it cannot help — and the window is empty iff
/// `ARMIJO_C ≥ 1/(2·max_iter)`, i.e. `1e-2` at the default cap of 50. The
/// shipped `5e-2` is a factor of five of margin on that, because the derivation
/// is leading order in `eps_dp/|Δp|` and `1e-2` measurably left a narrow band
/// open. `armijo_c_closes_the_shut_in_stall_window` asserts the relation.
///
/// Costing nothing is a measurement, not a hope: rejecting the full step forces
/// `t = ½`, which lands within `eps_dp` of the root from *any* drop, so the
/// worst-case iterations per pass across all fourteen shipped scenarios FELL,
/// 11 → 10 of 50, when this went from `1e-4` to `5e-2`.
const ARMIJO_C: f64 = 5e-2;

pub struct NewtonFlowSolver {
    /// Hard cap on Newton iterations within one anchoring pass.
    ///
    /// **Lowering this reopens the shut-in stall window** unless `ARMIJO_C` rises
    /// with it — see that constant, and DESIGN §11. Nothing refuses a low value,
    /// because no scenario file can set it; the invariant is asserted against
    /// this struct's `Default` only.
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
        previous_states: &NodeStates,
        dt: Seconds,
    ) -> Result<HydraulicSolution, SimError> {
        // F6: pumps/valves must have exactly one inlet and one outlet edge.
        validate_degrees(graph)?;

        // The classification is an ACTIVE SET, not a constant: a relief valve's
        // opening depends on the pressure this solve is still finding, so the
        // prologue's answer can be stale and the loop is what re-asks it
        // (M8.0, DESIGN §3c). One `pass` below is a whole Newton solve under a
        // fixed classification, which is what keeps the Jacobian's dimension —
        // and therefore the line search's merit comparison — well defined.
        //
        // The warm start moves OUT for the duration: the driver owns writing it,
        // because a pass can converge under a classification the loop then
        // rejects. Taking it also splits the borrow, so the closure may hold
        // `&self` for the tolerances.
        let mut warm_start = std::mem::take(&mut self.warm_start);
        let out =
            solve_with_active_anchoring(graph, slate, previous_states, &mut warm_start, |prep| {
                self.pass(prep, graph, slate, previous_states, dt)
            });
        self.warm_start = warm_start;
        out
    }

    fn name(&self) -> &'static str {
        "newton-network"
    }
}

impl NewtonFlowSolver {
    /// One damped-Newton solve under a FIXED anchoring classification — the body
    /// this solver had before M8.0, minus the prologue (`prepare`, now the
    /// driver's) and minus the warm-start write (also the driver's).
    ///
    /// Returns its final pressure iterate alongside its result, converged or
    /// not: on the plant this loop exists for the pass FAILS, and its iterate is
    /// what says why (DESIGN §3c).
    fn pass(
        &self,
        prep: Prepared,
        graph: &PlantGraph,
        slate: &Slate,
        previous_states: &NodeStates,
        dt: Seconds,
    ) -> AnchorPass {
        let anchored = &prep.anchored;
        let free = &prep.classes.free;
        let mut pressures = prep.pressures;
        let mut compiled = prep.compiled;

        // Newton unknowns = anchored free nodes, ascending (deterministic).
        // Floating free nodes are pinned at their seed; their edges report zero.
        let mut idx: BTreeMap<NodeId, usize> = BTreeMap::new();
        let mut unknowns: Vec<NodeId> = Vec::new();
        for &nid in free {
            if anchored.contains(&nid) {
                idx.insert(nid, unknowns.len());
                unknowns.push(nid);
            }
        }
        let n = unknowns.len();

        // Trivial: no unknowns (all pinned, or every free node floating) ⇒
        // flows are determined directly. A network with no pressure reference at
        // all lands here as a benign all-P_ATM, zero-flow Ok; the scenario loader
        // is responsible for rejecting components that lack one, so the solver
        // stays lenient rather than Err'ing.
        //
        // "No pressure reference" is NOT "no fixed node" since M5.3, and the
        // distinction matters exactly here: a closed gas system has
        // `fixed_cnt == 0` and still reaches the Newton loop with `n > 0`,
        // because a capacitive vessel is an anchored free unknown carrying its
        // own equation. Reading this branch as "no fixed node ⇒ nothing to
        // solve" is how one would conclude such a plant is inert, and it is not
        // — see `two_vessels_and_no_fixed_node_equalise`.
        if n == 0 {
            let edges = edge_flows(graph, &compiled, &pressures, anchored, self.eps_dp);
            let result = finalize(graph, &pressures, edges, 0, 0.0);
            return AnchorPass { result, pressures };
        }

        // Damped Newton.
        let capacitive = &prep.classes.capacitive;
        let mut history: Vec<f64> = Vec::new();
        let (mut r, mut jac, mut throughput) = assemble(
            graph,
            &compiled,
            &pressures,
            &idx,
            anchored,
            capacitive,
            n,
            dt,
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
                // through) — surfaces here first. **This is the frozen-anchoring
                // plant's exit**: a dead leg whose relief shut on the way to the
                // answer leaves a zero row and column. `pressures` is the last
                // ACCEPTED iterate — the bad trial is never stored — so the
                // driver can reclassify from it and try again (DESIGN §3c).
                return AnchorPass {
                    result: Err(diverged(iterations, res, history)),
                    pressures,
                };
            }

            // Damped line search with the Armijo sufficient-decrease condition
            // on φ = ½‖R‖₂². The exact Newton step is a descent direction with
            // φ'(0) = −‖R‖₂² = −2φ, so we require φ_t ≤ (1 − 2·c·t)·φ. Merely
            // requiring "any decrease" would accept the √-law's near-symmetric
            // overshoot (t=1) and stall; Armijo rejects it and forces t≤½.
            //
            // That was true as intent and false as code until M9.0: the mirror
            // step decreases the merit by `2·eps_dp/|Δp|`, so `ARMIJO_C = 1e-4`
            // accepted it below a drop of 10 kPa and the solve crawled 2 Pa at a
            // time. The rejection threshold IS `eps_dp/ARMIJO_C`; see that
            // constant for the relation it must hold against `max_iter`.
            let mut t = 1.0;
            let mut accepted = false;
            for _ in 0..=MAX_HALVINGS {
                let trial = apply_step(&pressures, &unknowns, &idx, &dp, t);
                // Recompile at the trial iterate: a gas edge's frozen density
                // coefficient follows the pressure it is evaluated at, so the
                // merit the line search compares must be the merit of the fully
                // consistent trial, not of the old coefficients at a new
                // pressure. For an all-liquid network this reproduces the same
                // `CompiledEdge` bit for bit (M5.2, `compile_edge`).
                let compiled_t = match compile_edges(graph, slate, previous_states, &trial) {
                    Ok(c) => c,
                    Err(e) => {
                        return AnchorPass {
                            result: Err(e),
                            pressures,
                        }
                    }
                };
                let (r_t, jac_t, tp_t) = assemble(
                    graph,
                    &compiled_t,
                    &trial,
                    &idx,
                    anchored,
                    capacitive,
                    n,
                    dt,
                    self.eps_dp,
                );
                let merit_t = half_sq_norm(&r_t);
                if merit_t <= (1.0 - 2.0 * ARMIJO_C * t) * merit {
                    pressures = trial;
                    compiled = compiled_t;
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
                return AnchorPass {
                    result: Err(diverged(iterations, res, history)),
                    pressures,
                };
            }
            converged = converged_at(res, throughput);
        }

        if !converged {
            return AnchorPass {
                result: Err(diverged(iterations, res, history)),
                pressures,
            };
        }

        let edges = edge_flows(graph, &compiled, &pressures, anchored, self.eps_dp);
        let result = finalize(graph, &pressures, edges, iterations, res);
        AnchorPass { result, pressures }
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
///
/// A CAPACITIVE node adds `−C·(P − Pⁿ)/dt` to its own residual and `−C/dt` to its
/// own diagonal, through the shared `network::accumulation` so the Simple sweep
/// cannot end up solving a different fixed point. The term touches nothing
/// off-diagonal: `m(P)` is a function of that node's pressure alone, so `J`
/// stays the symmetric weighted Laplacian it was, with a strictly more negative
/// diagonal — better conditioned, not merely still invertible.
///
/// This is what makes one solve an implicit-Euler step of a DAE rather than a
/// steady state (DESIGN §3a fork 2). `throughput` deliberately excludes it: the
/// convergence scale is the network's mass flow, and a vessel's accumulation is
/// measured against that, not added to it.
#[allow(clippy::too_many_arguments)]
fn assemble(
    graph: &PlantGraph,
    compiled: &BTreeMap<EdgeId, CompiledEdge>,
    pressures: &BTreeMap<NodeId, f64>,
    idx: &BTreeMap<NodeId, usize>,
    anchored: &BTreeSet<NodeId>,
    capacitive: &BTreeMap<NodeId, Capacitance>,
    n: usize,
    dt: Seconds,
    eps: f64,
) -> (Vec<f64>, Vec<Vec<f64>>, f64) {
    let mut r = vec![0.0; n];
    let mut jac = vec![vec![0.0; n]; n];
    let mut throughput = 0.0f64;
    for (nid, cap) in capacitive {
        if let Some(i) = idx.get(nid) {
            let (term, slope) = accumulation(cap, pressures[nid], dt.value());
            r[*i] += term;
            jac[*i][*i] += slope;
        }
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    /// The relation DESIGN §11 derives, asserted against the shipped defaults.
    ///
    /// This is a gate on two constants that look independent and are not. A
    /// Newton step on the regularised square-root law lands on the mirror of the
    /// branch drop, `2·eps_dp` closer to the root, so an accepted full step
    /// converges only after `|Δp₀|/(2·eps_dp)` iterations. Armijo accepts that
    /// step while `|Δp₀| ≲ eps_dp/ARMIJO_C`, so plants whose drop falls in
    ///
    /// ```text
    /// (2·eps_dp·max_iter, eps_dp/ARMIJO_C]
    /// ```
    ///
    /// stall. `eps_dp` cancels, which is why it does not appear below: the
    /// window is empty iff `ARMIJO_C·2·max_iter ≥ 1`.
    ///
    /// It fires on either half of the coupling — dropping `ARMIJO_C` back toward
    /// `1e-4`, or lowering `max_iter` far enough that the cap can no longer fund
    /// the crawl the constant still permits.
    #[test]
    fn armijo_c_closes_the_shut_in_stall_window() {
        let max_iter = f64::from(NewtonFlowSolver::default().max_iter);
        let closure = ARMIJO_C * 2.0 * max_iter;
        assert!(
            closure >= 1.0,
            "the shut-in stall window is OPEN: ARMIJO_C = {ARMIJO_C:e} against              max_iter = {max_iter}, so a branch drop between {lo:.0} and {hi:.0}              pascals is accepted at t = 1 and then cannot reach the root inside              the cap. Raise ARMIJO_C to at least {need:e}, or raise max_iter to              at least {need_iter:.0} (DESIGN §11, fork 2 — which costs a dense LU              per iteration and is why fork 1 was chosen)",
            lo = 2.0 * NewtonFlowSolver::default().eps_dp * max_iter,
            hi = NewtonFlowSolver::default().eps_dp / ARMIJO_C,
            need = 1.0 / (2.0 * max_iter),
            need_iter = 1.0 / (2.0 * ARMIJO_C),
        );

        // And the margin is deliberate rather than incidental: `1e-2` satisfies
        // the relation exactly and measurably left a band open, because the
        // derivation is leading order in `eps_dp/|Δp|`. Asserting the margin is
        // what stops a later "simplification" to the bare bound.
        assert!(
            closure >= 4.0,
            "ARMIJO_C satisfies the stall relation with no margin (factor              {closure:.2}). DESIGN §11 measures a surviving stall band at the              bare bound; the shipped value carries a factor of five"
        );
    }
}
