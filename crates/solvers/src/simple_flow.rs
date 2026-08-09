//! SimpleFlowSolver — game fidelity. Solves the SAME quasi-steady network as
//! `NewtonFlowSolver` (identical element characteristics, identical boundary
//! classification, both from `crate::network`), but with a matrix-free
//! iteration instead of a global Newton linear solve: **nonlinear Gauss–Seidel**
//! sweeps over the node pressures.
//!
//! Per free node `n`, the mass-balance residual is
//! `imbalance(P_n) = Σ_incoming ṁ − Σ_outgoing ṁ`, a monotone function of `P_n`
//! (raising `P_n` pushes more out / draws less in). A single scalar Newton step
//! for that node — holding neighbours fixed — is
//!
//! ```text
//! ΔP_n = imbalance_n / Σ_e g_e ,   g_e = ρ_e · dQ_e/d(dP) ≥ 0
//! ```
//!
//! because `∂imbalance_n/∂P_n = −Σ_e g_e`. We apply `P_n += ω · ΔP_n` in place
//! (Gauss–Seidel: later nodes in a sweep see earlier nodes' updates), sweeping
//! anchored free nodes in ascending id order until the max node imbalance falls
//! below tolerance. This is exactly diagonal (Jacobi) preconditioning of the
//! same weighted-Laplacian system Newton assembles, so it is scale-invariant
//! across the wide conductance spread of real pipes/valves — a single fixed
//! `beta` (the original sketch) is not, and diverges on stiff branches. `Σ_e g_e`
//! is strictly positive for any anchored free node (it always has ≥1 active,
//! regularized branch), so the update never divides by zero.
//!
//! Properties: O(edges) per sweep, no linear algebra, warm-started from the
//! previous tick's pressures (few sweeps at steady state). It solves the same
//! fixed point as Newton; the only fidelity difference is the looser stopping
//! tolerance. Non-convergence (e.g. a stiff network GS cannot crack in
//! `max_iter` sweeps) is `Err(SolverDiverged)` — never `Ok(unconverged)`, never
//! a NaN escape (rule 5). Cross-fidelity agreement with Newton on well-posed
//! networks is the I5 property test.

use crate::network::{compile_edges, edge_flows, prepare, validate_degrees};
use refinery_core::components::Slate;
use refinery_core::error::SimError;
use refinery_core::graph::{EdgeId, NodeId, PlantGraph};
use refinery_core::traits::{FlowSolver, HydraulicSolution};
use refinery_core::units::Seconds;
use std::collections::BTreeMap;

pub struct SimpleFlowSolver {
    /// Pressure under-relaxation ω ∈ (0, 1]. 1.0 = full node-wise Newton step;
    /// lower damps oscillation on stiff networks at the cost of more sweeps.
    pub omega: f64,
    /// Sweep cap; exceeding it is `Err(SolverDiverged)`. Generous because
    /// Gauss–Seidel needs far more iterations than Newton (relaxation, not
    /// quadratic convergence), especially on a cold first tick.
    pub max_iter: u32,
    /// Absolute residual tolerance floor [kg/s].
    pub tol_abs_kg_s: f64,
    /// Relative residual tolerance (× network throughput) [kg/s per kg/s].
    /// Looser than Newton's; still tight enough that steady-state flows agree
    /// with Newton well within the I5 5% bound.
    pub tol_rel: f64,
    /// Regularization epsilon for sqrt laws [Pa]. Matches Newton so both route
    /// through the same `QuadraticBranch::flow` and agree near zero flow.
    pub eps_dp: f64,
    /// Warm-start pressures from the previous converged solve, keyed by NodeId.
    warm_start: BTreeMap<NodeId, f64>,
}

impl Default for SimpleFlowSolver {
    fn default() -> Self {
        Self {
            omega: 1.0,
            max_iter: 5000,
            tol_abs_kg_s: 1e-8,
            tol_rel: 1e-6,
            eps_dp: 1.0,
            warm_start: BTreeMap::new(),
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
        // Same classification + seeding + compilation as Newton (the fidelity
        // seam), through the same `prepare`.
        validate_degrees(graph)?;
        let prep = prepare(graph, slate, &self.warm_start)?;
        let cls = prep.classes;
        let anchored = &prep.anchored;
        let mut compiled = prep.compiled;
        let mut pressures = prep.pressures;

        // Unknowns = anchored free nodes (ascending, deterministic). Precompute
        // each unknown's incident ACTIVE edges (both endpoints anchored) with
        // orientation, so the sweep is a tight O(edges) inner loop.
        let unknowns: Vec<NodeId> = cls
            .free
            .iter()
            .copied()
            .filter(|nid| anchored.contains(nid))
            .collect();
        let incident: BTreeMap<NodeId, Vec<(EdgeId, bool)>> = unknowns
            .iter()
            .map(|&nid| {
                let edges = graph
                    .incident(nid)
                    .into_iter()
                    .filter(|(eid, _, _)| {
                        let c = &compiled[eid];
                        anchored.contains(&c.src) && anchored.contains(&c.tgt)
                    })
                    .map(|(eid, _, incoming)| (eid, incoming))
                    .collect();
                (nid, edges)
            })
            .collect();

        // Trivial: no unknowns (all pinned, or every free node floating) ⇒
        // flows follow directly. Mirrors Newton's n == 0 branch.
        if unknowns.is_empty() {
            let edges = edge_flows(graph, &compiled, &pressures, anchored, self.eps_dp);
            return crate::network::finalize(&pressures, edges, 0, 0.0);
        }

        // Nonlinear Gauss–Seidel: sweep, then measure the residual on the exact
        // edge flows the solution will report (so the returned solution provably
        // satisfies the reported bound, and matches the invariants-test balance).
        let mut history: Vec<f64> = Vec::new();
        let converged_at = |res: f64, tp: f64| res < self.tol_abs_kg_s + self.tol_rel * tp;
        let mut iterations = 0u32;
        while iterations < self.max_iter {
            iterations += 1;

            // Refresh the frozen density coefficients at the current iterate,
            // once per sweep rather than once per node: within a sweep the
            // node-wise Newton step already treats its neighbours as fixed, so
            // per-node recompilation would refresh coefficients the step is not
            // differentiating anyway. All-liquid networks recompile to identical
            // numbers (M5.2, `network::compile_edge`).
            compiled = compile_edges(graph, slate, &pressures)?;

            // In-place update sweep.
            for &nid in &unknowns {
                let mut imbalance = 0.0;
                let mut g_sum = 0.0;
                for &(eid, incoming) in &incident[&nid] {
                    let c = &compiled[&eid];
                    let dp = pressures[&c.src] - pressures[&c.tgt];
                    let mdot = c.rho * c.branch.flow(dp, self.eps_dp);
                    g_sum += c.rho * c.branch.flow_ddp(dp, self.eps_dp); // ≥ 0
                    imbalance += if incoming { mdot } else { -mdot };
                }
                // g_sum > 0 for any anchored free node; the node-wise Newton
                // step ΔP = imbalance / g_sum drives this node's balance to zero.
                let step = self.omega * imbalance / g_sum;
                if !step.is_finite() {
                    return Err(diverged(iterations, f64::INFINITY, history));
                }
                *pressures.get_mut(&nid).expect("unknown is a node") += step;
            }

            // Residual on the post-sweep flows (throughput = max|ṁ| over all
            // active edges, so the relative tolerance matches Newton's).
            let edges = edge_flows(graph, &compiled, &pressures, anchored, self.eps_dp);
            let (flows, throughput) = (&edges.mass_flow, edges.throughput);
            let mut residual = 0.0f64;
            for &nid in &unknowns {
                let bal: f64 = incident[&nid]
                    .iter()
                    .map(|&(eid, incoming)| {
                        let f = flows[&eid];
                        if incoming {
                            f
                        } else {
                            -f
                        }
                    })
                    .sum();
                residual = residual.max(bal.abs());
            }
            history.push(residual);

            if converged_at(residual, throughput) {
                // Converged: persist warm start (converged-only) and build the
                // solution from the flows just measured.
                for &nid in &cls.free {
                    if let Some(&p) = pressures.get(&nid) {
                        self.warm_start.insert(nid, p);
                    }
                }
                return crate::network::finalize(&pressures, edges, iterations, residual);
            }
            if pressures.values().any(|p| !p.is_finite()) {
                return Err(diverged(iterations, residual, history));
            }
        }

        let residual = history.last().copied().unwrap_or(f64::INFINITY);
        Err(diverged(iterations, residual, history))
    }

    fn name(&self) -> &'static str {
        "simple-relaxation"
    }
}

fn diverged(iterations: u32, residual: f64, residual_history: Vec<f64>) -> SimError {
    SimError::SolverDiverged {
        iterations,
        residual,
        residual_history,
    }
}
