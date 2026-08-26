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

use crate::network::{
    accumulation, compile_edges, edge_flows, solve_with_active_anchoring, validate_degrees,
    AnchorPass, Capacitance, CompiledEdge, Prepared,
};
use refinery_core::components::Slate;
use refinery_core::energy::NodeStates;
use refinery_core::error::SimError;
use refinery_core::graph::{EdgeId, NodeId, PlantGraph};
use refinery_core::traits::{FlowSolver, HydraulicSolution};
use refinery_core::units::Seconds;
use std::collections::BTreeMap;

/// Sufficient-decrease constant for the per-node line search, on the node's own
/// scalar imbalance: a trial is accepted iff `|R_t| ≤ (1 − ARMIJO_C·t)·|R|`.
///
/// **This is `newton_flow`'s Armijo test, one node at a time.** That one compares
/// `φ_t ≤ (1 − 2·c·t)·φ` on `φ = ½‖R‖₂²`; since `φ ∝ R²` on a scalar residual and
/// `(1 − ct)² = 1 − 2ct + O(c²t²)`, the missing factor of two is the square and
/// not a tuning choice. The `c` means the same thing on both sides.
///
/// **It is deliberately a SEPARATE constant from `newton_flow::ARMIJO_C`, not a
/// shared one, and the equal value is a coincidence of two measurements.** There
/// the number is five times a relation bound of `1/(2·max_iter) = 1e-2` at a cap
/// of 50; here the same relation bound is `1e-4` at a cap of 5 000, five hundred
/// times slacker, so nothing about that margin transfers. `5e-2` was re-derived
/// on this solver by sweep (DESIGN §11, M9.1): the knee sits between `1e-3` and
/// `1e-2`, everything above `1e-2` converges the shut-in fixture in 7–8 sweeps at
/// any valve opening, and the cost — which lands entirely on `relief_blowdown` —
/// rises monotonically with strictness, 6% here against 53% at `2e-1`. Sharing
/// one constant would let a re-tuning of Newton's margin move this solver
/// silently.
///
/// **Do not lower it toward the relation's bare bound.** At `1e-4` the fully shut
/// valve converges in 7 sweeps and a valve 1% open still takes 1 239, because the
/// relation describes the dead leg — where progress is additive — and a
/// conducting node's overshoot contracts geometrically instead.
const ARMIJO_C: f64 = 5e-2;

/// Max halvings per node step (min step 1/256). Matches `newton_flow`, and the
/// depth the corpus actually needs is **2**, bisected rather than assumed.
///
/// The closed form in DESIGN §11 says `t = ½` on the regularised square-root law
/// lands within `eps_dp` of the root from any branch drop at all, which reads as
/// "one halving is enough" and would make this a `1`. It is not: at
/// `MAX_HALVINGS = 1` the M8.0 anchoring plant DIVERGES, 20 000 sweeps at residual
/// `5.397e1`. At `2` that plant passes and so does the whole workspace, and `3`
/// changes nothing further. **So one node on that plant needs `t = ¼`, and the
/// closed form does not describe it.**
///
/// Which property of that node puts it outside the form is NOT measured. The form
/// was derived on a dead leg — rule F6 leaves a shut valve's orphaned node exactly
/// one live edge, so the mirror is exact — and the natural reading is that a
/// second live edge shifts the root off the mirror. That is a candidate fitted to
/// a single divergence, not a result, and nothing here tests it.
///
/// `8` is therefore six halvings of margin over anything measured, and is
/// inherited from `newton_flow` rather than derived — it has never been justified
/// there either. Cutting it to the measured `2` would be fitting a constant to
/// today's fourteen plants; cutting it to `1` is refuted.
const MAX_HALVINGS: u32 = 8;

pub struct SimpleFlowSolver {
    /// Pressure under-relaxation ω ∈ (0, 1], applied before the line search.
    ///
    /// **Leave it at 1.0.** Damping used to be this solver's only defence against
    /// the square-root law's overshoot and is no longer: `ARMIJO_C` rejects the
    /// bad step where it occurs, instead of shortening every step on every node
    /// of every plant. The two are the SAME remedy — at `ω = 0.5` the line search
    /// never fires, because the half step already passes its own test, and the
    /// corpus reproduces the pre-M9.1 `ω = 0.5` numbers exactly.
    ///
    /// **Lowering it is a correctness result before it is a cost one.** At
    /// `ω = 0.5` the shut-in fixture's solve returns `Ok` and leaves `3.77e-6`
    /// kg/s through a branch that is shut — inside this solver's own
    /// `tol_abs + tol_rel·throughput` (about `4e-6` at that plant's rate) and
    /// outside the `1e-6` the endpoint gate allows. The wrong answer is reported
    /// as converged; the sweeps are the smaller half of the objection.
    ///
    /// It is also measured to cost rather than to help: worst
    /// sweeps in any tick over 500 ticks of all fourteen shipped scenarios rise
    /// on every one of them, and `relief_blowdown` — whose convergence is driven
    /// by its vessel's own `−C/dt` term rather than by branch conductance — goes
    /// 920 → 1 520 → 3 035 at `ω` of 1.0, 0.75, 0.5 (DESIGN §11, M9.1 fork 3).
    ///
    /// No scenario file can set this; `crates/scenarios/src` never mentions it.
    /// It is a code-level invariant, and it becomes a load-time refusal if the
    /// solver's numerics ever become scenario config.
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
        previous_states: &NodeStates,
        dt: Seconds,
    ) -> Result<HydraulicSolution, SimError> {
        // Same classification + seeding + compilation as Newton (the fidelity
        // seam), through the same `prepare` — and since M8.0 the same active-set
        // loop around it, because the stale classification defeats both
        // fidelities and one driver is what keeps them agreeing (DESIGN §3c).
        validate_degrees(graph)?;
        let mut warm_start = std::mem::take(&mut self.warm_start);
        let out =
            solve_with_active_anchoring(graph, slate, previous_states, &mut warm_start, |prep| {
                self.pass(prep, graph, slate, previous_states, dt)
            });
        self.warm_start = warm_start;
        out
    }

    fn name(&self) -> &'static str {
        "simple-relaxation"
    }
}

impl SimpleFlowSolver {
    /// One Gauss–Seidel solve under a FIXED anchoring classification — the body
    /// this solver had before M8.0, minus the prologue and the warm-start write,
    /// both of which the driver now owns.
    fn pass(
        &self,
        prep: Prepared,
        graph: &PlantGraph,
        slate: &Slate,
        previous_states: &NodeStates,
        dt: Seconds,
    ) -> AnchorPass {
        let cls = prep.classes;
        let capacitive = &cls.capacitive;
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
            let result = crate::network::finalize(graph, &pressures, edges, 0, 0.0);
            return AnchorPass { result, pressures };
        }

        // Nonlinear Gauss–Seidel: sweep, then measure the residual on the exact
        // edge flows the solution will report (so the returned solution provably
        // satisfies the reported bound, and matches the invariants-test balance).
        let mut history: Vec<f64> = Vec::new();
        let converged_at = |res: f64, tp: f64| res < self.tol_abs_kg_s + self.tol_rel * tp;
        let mut iterations = 0u32;
        while iterations < self.max_iter {
            iterations += 1;

            // In-place update sweep, against coefficients valid at the CURRENT
            // pressures — from `prepare` on the first pass, refreshed after each
            // sweep below. Once per sweep rather than once per node: within a
            // sweep the node-wise Newton step already treats its neighbours as
            // fixed, so per-node recompilation would refresh coefficients the
            // step is not differentiating anyway. All-liquid networks recompile
            // to identical numbers (M5.2, `network::compile_edge`).
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
                // A capacitive node carries its own accumulation, through the
                // SAME `network::accumulation` Newton assembles — the shared
                // residual is what keeps the two fidelities on one fixed point.
                // `−C/dt` is a slope like any branch conductance, so it enters
                // `g_sum` with its sign flipped and the node-wise Newton step
                // needs no new algebra: it is the diagonal preconditioning this
                // sweep already performs, now including the vessel's own term.
                if let Some(cap) = capacitive.get(&nid) {
                    let (term, slope) = accumulation(cap, pressures[&nid], dt.value());
                    imbalance += term;
                    g_sum -= slope;
                }
                // g_sum > 0 for any anchored free node; the node-wise Newton
                // step ΔP = imbalance / g_sum drives this node's balance to zero.
                let full = self.omega * imbalance / g_sum;
                if !full.is_finite() {
                    // `g_sum == 0` — an anchored node with no conducting edge
                    // left, which is the frozen-anchoring plant's exit on this
                    // fidelity. The bad step is never applied, so `pressures`
                    // stays finite and the driver can reclassify from it.
                    return AnchorPass {
                        result: Err(diverged(iterations, f64::INFINITY, history)),
                        pressures,
                    };
                }
                // Per-node line search (M9.1, DESIGN §11). Without it this
                // solver applies `full` unconditionally, and on the regularised
                // square-root law `f(x) = x/√(|x|+ε)` a full Newton step lands on
                // the MIRROR of the branch drop, only `2ε` nearer the root. That
                // is not "a big step": it is the worst step available, and the
                // sweep then alternates sign forever, walking in at two pascals
                // apiece. Measured on the shut-in fixture: 2.0000 Pa per sweep
                // from 106 790.90 Pa, i.e. 53 411 sweeps against a cap of 5 000.
                //
                // Newton's stall window closes from either end because Armijo
                // eventually rejects the mirror; with no rejection at all this
                // solver's window is `(2·eps_dp·max_iter, ∞)` and NO `max_iter`
                // closes it. A rejection criterion is not one of several fixes
                // here, it is the only one.
                //
                // Rejecting the full step forces `t = ½`, which the same closed
                // form puts within `eps_dp` of the root from any drop — so this
                // usually costs one halving and buys a converged node.
                let p_now = pressures[&nid];
                let mut t = 1.0;
                let mut step = 0.0;
                for _ in 0..=MAX_HALVINGS {
                    let trial = t * full;
                    let after = node_imbalance_at(
                        nid,
                        p_now + trial,
                        &incident[&nid],
                        &compiled,
                        &pressures,
                        capacitive.get(&nid),
                        dt.value(),
                        self.eps_dp,
                    );
                    if after.abs() <= (1.0 - ARMIJO_C * t) * imbalance.abs() {
                        step = trial;
                        break;
                    }
                    t *= 0.5;
                }
                // `step` is still 0 if nothing was acceptable: leave the node
                // where it is and let its neighbours move it, rather than apply
                // a step the criterion has just rejected. Gauss–Seidel permits
                // that; a global Newton could not, which is why `newton_flow`
                // returns `Err` in the same position.
                //
                // Reachable, and measured rather than assumed: it fires 3 281
                // times across 500 ticks of all fourteen shipped scenarios, and
                // EVERY one of those sites has `|imbalance| ≤ 3.4e-13 kg/s` —
                // five orders below this solver's own `tol_abs_kg_s`. It is the
                // rounding floor on an already-converged node, where the target
                // `(1 − c·t)·|R|` is unreachable and the dropped step is a no-op.
                *pressures.get_mut(&nid).expect("unknown is a node") += step;
            }

            // Refresh the frozen density coefficients at the POST-sweep
            // pressures, before the residual is measured off them.
            //
            // The ordering is load-bearing and M5.3 is what exposed it.
            // Measuring with coefficients compiled before the sweep tests a
            // fixed point nobody solved, and the solution `finalize` then ships
            // is internally inconsistent — a flow computed from one iterate's
            // density at another iterate's pressure. It went unnoticed while
            // every free node was warm-started at a pressure it barely moved
            // from; a capacitive vessel moves ~400 Pa EVERY tick by design, so
            // the sweep converges in one pass and the stale coefficient is
            // never refreshed. That put the two fidelities 2.0e-4 apart on the
            // blowdown, ~4 orders above the residual either one reported, which
            // is how a convergence flag can be honest and the answer still
            // wrong. Bit-identical for an all-liquid network, where
            // `density_at` ignores both arguments.
            compiled = match compile_edges(graph, slate, previous_states, &pressures) {
                Ok(c) => c,
                Err(e) => {
                    return AnchorPass {
                        result: Err(e),
                        pressures,
                    }
                }
            };
            let edges = edge_flows(graph, &compiled, &pressures, anchored, self.eps_dp);
            let (flows, throughput) = (&edges.mass_flow, edges.throughput);
            let mut residual = 0.0f64;
            for &nid in &unknowns {
                let mut bal: f64 = incident[&nid]
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
                // The SAME residual the sweep drove to zero. Measuring only the
                // edge flows would declare a vessel converged the moment its
                // branches balanced each other, which for a blowing-down vessel
                // is never — its inflow and outflow are meant to differ by
                // exactly the accumulation.
                if let Some(cap) = capacitive.get(&nid) {
                    bal += accumulation(cap, pressures[&nid], dt.value()).0;
                }
                residual = residual.max(bal.abs());
            }
            history.push(residual);

            if converged_at(residual, throughput) {
                let result =
                    crate::network::finalize(graph, &pressures, edges, iterations, residual);
                return AnchorPass { result, pressures };
            }
            if pressures.values().any(|p| !p.is_finite()) {
                // The one exit that leaves a NON-finite iterate. The driver
                // refuses to reclassify from it and returns this error as-is,
                // because a classification derived from NaN is arbitrary.
                return AnchorPass {
                    result: Err(diverged(iterations, residual, history)),
                    pressures,
                };
            }
        }

        let residual = history.last().copied().unwrap_or(f64::INFINITY);
        AnchorPass {
            result: Err(diverged(iterations, residual, history)),
            pressures,
        }
    }
}

/// This node's mass-balance residual at a TRIAL pressure, every neighbour held
/// fixed — the same sum the sweep drives to zero, re-evaluated off the iterate.
/// The line search's only probe, and its only cost.
///
/// It must stay the same sum: measuring the trial against anything else would
/// grade a step by a residual nobody is solving, which is M5.3's finding in this
/// very file one paragraph down. So `capacitive` is threaded through and enters
/// via the SAME `network::accumulation`, and the edge flows come from the SAME
/// frozen `compiled` coefficients the step was differentiated against.
#[allow(clippy::too_many_arguments)]
fn node_imbalance_at(
    nid: NodeId,
    p_trial: f64,
    incident: &[(EdgeId, bool)],
    compiled: &BTreeMap<EdgeId, CompiledEdge>,
    pressures: &BTreeMap<NodeId, f64>,
    capacitive: Option<&Capacitance>,
    dt: f64,
    eps_dp: f64,
) -> f64 {
    let mut imbalance = 0.0;
    for &(eid, incoming) in incident {
        let c = &compiled[&eid];
        let at = |n: NodeId| if n == nid { p_trial } else { pressures[&n] };
        let mdot = c.rho * c.branch.flow(at(c.src) - at(c.tgt), eps_dp);
        imbalance += if incoming { mdot } else { -mdot };
    }
    if let Some(cap) = capacitive {
        imbalance += accumulation(cap, p_trial, dt).0;
    }
    imbalance
}

fn diverged(iterations: u32, residual: f64, residual_history: Vec<f64>) -> SimError {
    SimError::SolverDiverged {
        iterations,
        residual,
        residual_history,
    }
}
