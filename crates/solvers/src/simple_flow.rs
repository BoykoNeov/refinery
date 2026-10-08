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
//! tolerance. Non-convergence within `max_iter` sweeps is `Err(SolverDiverged)`
//! — never `Ok(unconverged)`, never a NaN escape (rule 5). Cross-fidelity
//! agreement with Newton on well-posed networks is the I5 property test.
//!
//! **After each sweep, an additive correction (M21.1, DESIGN §25).** Node-wise
//! Gauss–Seidel cannot move two tightly coupled unknowns TOGETHER: a vessel and
//! the zero-volume node across its wide line converge jointly at about
//! `c/(g + c)` per sweep, `g` the line's conductance and `c = C/dt` the vessel's
//! accumulation slope — 0.98850 predicted against 0.988502 measured on
//! `relief_blowdown`, which is why that plant took 920 sweeps and one edit to its
//! line or its `dt` took it past the cap. So each sweep is followed by one common
//! shift per GROUP of unknowns — a scalar Newton step on the group's net
//! imbalance, whose slope is exactly the boundary conductance plus the members'
//! `C/dt` (Settari & Aziz 1973; Hutchinson & Raithby 1986) — over a hierarchy of
//! groups built by heavy-edge matching (Karypis & Kumar 1998). See
//! `build_groups` and `correct_groups`.

use crate::network::{
    accumulation, compile_edge_with, compile_edges_with, edge_flows, meets_node_bar, pump_inlets,
    solve_pump_inlet, solve_remembering_reliefs, solve_with_active_anchoring_with,
    validate_degrees, AnchorPass, Capacitance, CompiledEdge, LineFlash, OwnedLineFlash, Prepared,
    PumpInlet, DENSITY_SLOPE_DELTA,
};
use refinery_core::components::Slate;
use refinery_core::energy::NodeStates;
use refinery_core::error::SimError;
use refinery_core::graph::{EdgeId, NodeId, NodeKind, PlantGraph};
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
/// Those figures are M9.1's, taken before M21.1's group correction, which is now
/// what converges `relief_blowdown` (920 → 8 sweeps); the sweep was not re-run.
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
    /// Measured at M9.1, before the group correction (M21.1) took that plant to 8
    /// sweeps; not re-measured since, so read it as the reason for `1.0`, not as
    /// today's cost.
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
    /// The line flash this solver reads two-phase densities through (M53,
    /// docs/DESIGN.md §58 fork 3). `NoLineFlash` by default, under which every
    /// compile is the liquid one bit for bit; the loader sets it from
    /// `[fidelity] line_flash`.
    pub line_flash: OwnedLineFlash,
}

impl SimpleFlowSolver {
    /// This solver reading two-phase densities through `line_flash` (M53,
    /// docs/DESIGN.md §58 fork 3) — what the loader calls for a plant selecting
    /// `[fidelity] line_flash`.
    pub fn with_line_flash(mut self, line_flash: OwnedLineFlash) -> Self {
        self.line_flash = line_flash;
        self
    }
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
            line_flash: OwnedLineFlash::default(),
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
        // A relief keeps its last answer where the plant has two (M48.1,
        // docs/DESIGN.md §53): the driver may run a second solve on a copy
        // of the plant, so the pass reads whichever graph it is handed.
        let out = solve_remembering_reliefs(graph, &mut warm_start, |graph, warm_start| {
            solve_with_active_anchoring_with(
                graph,
                slate,
                previous_states,
                warm_start,
                dt,
                self.line_flash.view(),
                |prep| self.pass(prep, graph, slate, previous_states, dt),
            )
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

        // The additive correction's groups, built ONCE per pass from the pass's
        // seed compile — the same point the anchored set is frozen (§3c), so the
        // grouping cannot flap while the weights move within the pass.
        //
        // A pump's inlet on a flashing plant is solved in its turn, never stepped
        // (M54, docs/DESIGN.md §59), so it is no group's member either: a common
        // shift would move it off the root its own solve keeps it on.
        let inlets = pump_inlets(graph, self.line_flash.view(), |nid| {
            unknowns.binary_search(&nid).is_ok()
        });
        let stepped: Vec<NodeId> = unknowns
            .iter()
            .copied()
            .filter(|nid| !inlets.iter().any(|i| i.node == *nid))
            .collect();
        let groups = build_groups(&stepped, &incident, &compiled, &pressures, self.eps_dp);

        // Nonlinear Gauss–Seidel: sweep, then measure the residual on the exact
        // edge flows the solution will report (so the returned solution provably
        // satisfies the reported bound, and matches the invariants-test balance).
        let mut history: Vec<f64> = Vec::new();
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
                // A pump's inlet is solved on its own balance's bracket, its
                // neighbours as this sweep has left them (M54, §59). Where it has
                // no root with them held, it takes the ordinary node step below.
                if let Some(&inlet) = inlets.iter().find(|i| i.node == nid) {
                    match solve_pump_inlet(
                        graph,
                        slate,
                        previous_states,
                        self.line_flash.view(),
                        inlet,
                        &mut pressures,
                        self.eps_dp,
                        0.01 * self.tol_abs_kg_s,
                    ) {
                        Ok(true) => continue,
                        Ok(false) => {}
                        Err(e) => {
                            return AnchorPass {
                                result: Err(e),
                                pressures,
                            }
                        }
                    }
                }
                let mut imbalance = 0.0;
                let mut g_sum = 0.0;
                // The node's own convergence scale, `max |ṁ|` over its edges, as
                // the grading below takes it: what the bracketed solve stops on.
                let mut scale = 0.0f64;
                for &(eid, incoming) in &incident[&nid] {
                    // A check valve's edge, and a cavitating pump's, is read
                    // FRESH, at the pressures this sweep has already moved
                    // (`fresh_edge`, §33, §55).
                    let fresh;
                    let c = match fresh_edge(
                        graph,
                        eid,
                        slate,
                        previous_states,
                        &pressures,
                        self.line_flash.view(),
                    ) {
                        Ok(Some(f)) => {
                            fresh = f;
                            &fresh
                        }
                        Ok(None) => &compiled[&eid],
                        Err(e) => {
                            return AnchorPass {
                                result: Err(e),
                                pressures,
                            }
                        }
                    };
                    let dp = pressures[&c.src] - pressures[&c.tgt];
                    let mdot = c.rho * c.branch.flow(dp, self.eps_dp);
                    g_sum += c.conductance(dp, self.eps_dp); // ≥ 0
                                                             // A cavitating pump's head moves with its own node's
                                                             // pressure (M50, §55): that share of the slope is this
                                                             // node's only when it is the pump.
                    if c.src == nid {
                        g_sum += c.suction_share(dp, self.eps_dp);
                    }
                    // A two-phase edge's flow moves with its upwind node's
                    // pressure through its density (M53, §58 fork 3): this
                    // node's share when it is that end, signed as the side of
                    // the edge it stands on.
                    let share = c.density_share(nid, dp, self.eps_dp);
                    if share != 0.0 {
                        g_sum += if c.src == nid { share } else { -share };
                    }
                    imbalance += if incoming { mdot } else { -mdot };
                    scale = scale.max(mdot.abs());
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
                // Beside a pump's inlet the node is stepped on the plant as it
                // answers, the inlet re-solved (M54, docs/DESIGN.md §59): its
                // imbalance there, and the slope of that imbalance by a central
                // difference. Held still instead, the pump's lever made the
                // node's slope ten to a hundred times too steep — up to 200
                // sweeps a tick, and none sufficing on a cold start at 125 °C
                // into a 3 bar destination (measured, M54's probe).
                if !adjacent_inlets(graph, &incident[&nid], &inlets).is_empty() {
                    let reduced = |p: f64| {
                        node_imbalance_at(
                            nid,
                            p,
                            &incident[&nid],
                            &compiled,
                            &pressures,
                            capacitive.get(&nid),
                            dt.value(),
                            self.eps_dp,
                            (graph, slate, previous_states, self.line_flash.view()),
                            (&inlets, 0.01 * self.tol_abs_kg_s),
                        )
                    };
                    let p = pressures[&nid];
                    let delta = DENSITY_SLOPE_DELTA;
                    match (reduced(p), reduced(p + delta), reduced(p - delta)) {
                        (Ok(here), Ok(above), Ok(below)) => {
                            imbalance = here;
                            let slope = (below - above) / (2.0 * delta);
                            if slope > 0.0 && slope.is_finite() {
                                g_sum = slope;
                            }
                        }
                        (Err(e), _, _) | (_, Err(e), _) | (_, _, Err(e)) => {
                            return AnchorPass {
                                result: Err(e),
                                pressures,
                            }
                        }
                    }
                }
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
                let mut residual_at = |trial: f64| {
                    node_imbalance_at(
                        nid,
                        p_now + trial,
                        &incident[&nid],
                        &compiled,
                        &pressures,
                        capacitive.get(&nid),
                        dt.value(),
                        self.eps_dp,
                        (graph, slate, previous_states, self.line_flash.view()),
                        (&inlets, 0.01 * self.tol_abs_kg_s),
                    )
                };
                // **A cavitating pump's own node is solved on its bracket FIRST**
                // (M50, docs/DESIGN.md §55). Its head is flat at the top of the
                // curve and flat at zero, with a knee between, so a Newton step
                // from the flat top overshoots the knee into the dead zone — and
                // the ladder ACCEPTS it, the residual having fallen a little. The
                // group step then shifts the node back, and the pair cycles for
                // ever (measured: 5 000 sweeps on the M50 demo's second tick).
                // The node's imbalance is still monotone in its own pressure, so a
                // full step whose residual changes sign brackets its root.
                let pump_node = incident[&nid]
                    .iter()
                    .any(|(eid, incoming)| !*incoming && compiled[eid].pump_suction.is_some());
                let mut step = if pump_node {
                    match bracketed_step(full, imbalance, &mut residual_at, |r| {
                        meets_node_bar(r, scale, self.tol_abs_kg_s, self.tol_rel)
                    }) {
                        Ok(0.0) => armijo_step(full, imbalance, &mut residual_at),
                        other => other,
                    }
                } else {
                    armijo_step(full, imbalance, &mut residual_at)
                };
                // **When the ladder refuses every step on a node that is NOT
                // converged, solve the node's own equation on the bracket**
                // (M45.1, docs/DESIGN.md §50). A check valve shut against a
                // nearly shut valve is the case: the node sees only the valve's
                // tiny slope, so the Newton step is hundreds of times too long,
                // lands where the disc is wide open, and 1/256 of it still does.
                // The node's imbalance is monotone in its own pressure, so a full
                // step whose residual has the other sign brackets the root.
                // Skipped at the node's own bar: every refusal the shipped plants
                // reach sits at the rounding floor, five orders below it.
                if matches!(step, Ok(0.0))
                    && !meets_node_bar(imbalance, scale, self.tol_abs_kg_s, self.tol_rel)
                {
                    step = bracketed_step(full, imbalance, &mut residual_at, |r| {
                        meets_node_bar(r, scale, self.tol_abs_kg_s, self.tol_rel)
                    });
                }
                let step = match step {
                    Ok(step) => step,
                    Err(e) => {
                        return AnchorPass {
                            result: Err(e),
                            pressures,
                        }
                    }
                };
                // `step` is 0 if nothing was acceptable: leave the node
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

            // The group correction, after the node-wise sweep and BEFORE the
            // recompile-and-grade below, so the convergence test grades the
            // corrected iterate and `finalize` ships a solution consistent with
            // its own coefficients (M5.3's ordering; DESIGN §25 fork 4).
            if let Err(e) = self.correct_groups(
                &groups,
                &incident,
                &compiled,
                &mut pressures,
                capacitive,
                graph,
                slate,
                previous_states,
                dt.value(),
                &inlets,
            ) {
                return AnchorPass {
                    result: Err(e),
                    pressures,
                };
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
            compiled = match compile_edges_with(
                graph,
                slate,
                previous_states,
                &pressures,
                self.line_flash.view(),
            ) {
                Ok(c) => c,
                Err(e) => {
                    return AnchorPass {
                        result: Err(e),
                        pressures,
                    }
                }
            };
            let edges = edge_flows(graph, &compiled, &pressures, anchored, self.eps_dp);
            let flows = &edges.mass_flow;
            let mut graded: Vec<(f64, f64)> = Vec::with_capacity(unknowns.len());
            for &nid in &unknowns {
                let mut bal = 0.0f64;
                // The node's own convergence scale, alongside its imbalance and
                // off the same flows: `max |ṁ|` over its incident active edges.
                // Definition and reasoning live on `network::grade_nodes`, so
                // both fidelities stop on one rule (M9.2).
                let mut scale = 0.0f64;
                for &(eid, incoming) in &incident[&nid] {
                    let f = flows[&eid];
                    bal += if incoming { f } else { -f };
                    scale = scale.max(f.abs());
                }
                // The SAME residual the sweep drove to zero. Measuring only the
                // edge flows would declare a vessel converged the moment its
                // branches balanced each other, which for a blowing-down vessel
                // is never — its inflow and outflow are meant to differ by
                // exactly the accumulation.
                if let Some(cap) = capacitive.get(&nid) {
                    bal += accumulation(cap, pressures[&nid], dt.value()).0;
                }
                graded.push((bal, scale));
            }
            let (residual, converged) =
                crate::network::grade_nodes(graded, self.tol_abs_kg_s, self.tol_rel);
            history.push(residual);

            if converged {
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

    /// The group step for a group beside one or more pump inlets (M54,
    /// docs/DESIGN.md §59.1; `correct_groups`).
    #[allow(clippy::too_many_arguments)] // `correct_groups`' inputs, and the inlets
    fn shift_beside_inlets(
        &self,
        group: &[NodeId],
        beside: &[PumpInlet],
        incident: &BTreeMap<NodeId, Vec<(EdgeId, bool)>>,
        compiled: &BTreeMap<EdgeId, CompiledEdge>,
        pressures: &mut BTreeMap<NodeId, f64>,
        capacitive: &BTreeMap<NodeId, Capacitance>,
        (graph, slate, previous_states): (&PlantGraph, &Slate, &NodeStates),
        dt: f64,
    ) -> Result<(), SimError> {
        let flash = self.line_flash.view();
        let tol = 0.01 * self.tol_abs_kg_s;
        let resolved = |shift: f64| -> Result<f64, SimError> {
            let mut trial = pressures.clone();
            for nid in group {
                if let Some(p) = trial.get_mut(nid) {
                    *p += shift;
                }
            }
            for &inlet in beside {
                solve_pump_inlet(
                    graph,
                    slate,
                    previous_states,
                    flash,
                    inlet,
                    &mut trial,
                    self.eps_dp,
                    tol,
                )?;
            }
            let (imbalance, _) = group_imbalance(
                group,
                0.0,
                Some(TrialCompile {
                    graph,
                    slate,
                    previous_states,
                    pressures: &trial,
                    flash,
                }),
                incident,
                compiled,
                &trial,
                capacitive,
                dt,
                self.eps_dp,
            )?;
            Ok(imbalance)
        };
        let delta = DENSITY_SLOPE_DELTA;
        let r0 = resolved(0.0)?;
        let slope = (resolved(-delta)? - resolved(delta)?) / (2.0 * delta);
        if !(slope > 0.0 && slope.is_finite()) {
            return Ok(());
        }
        let full = r0 / slope;
        let step = armijo_step(full, r0, resolved)?;
        if step != 0.0 {
            for nid in group {
                if let Some(p) = pressures.get_mut(nid) {
                    *p += step;
                }
            }
        }
        Ok(())
    }

    /// The additive correction (DESIGN §25): for each group in `groups`, in order
    /// (finest level first), shift every member's pressure by one common `δ`.
    ///
    /// A common shift leaves every edge INSIDE the group unchanged, so the group's
    /// net imbalance `Σ_{i∈K} R_i` depends only on its boundary edges and its
    /// members' accumulation, and one scalar Newton step on it is
    ///
    /// ```text
    /// δ = Σ_{i∈K} R_i  /  ( Σ_{boundary e} g_e + Σ_{i∈K} C_i/dt )
    /// ```
    ///
    /// That denominator is exactly what resists the pair's slow common mode, and
    /// nothing else. For the symmetric positive-definite linearisation (the
    /// network's weighted Laplacian plus `C/dt`) the step is a Galerkin coarse
    /// correction with a piecewise-constant prolongation, an energy-norm
    /// projection that cannot increase the error. Groups are corrected
    /// SEQUENTIALLY, each from the pressures the previous one left — a
    /// Gauss–Seidel over the coarse unknowns rather than a Jacobi one.
    ///
    /// Three rules, each measured on the prototype in §25:
    ///
    /// - **Skipped when every member already meets the solve's own per-node bar**
    ///   (`network::meets_node_bar`, the stopping test itself, not a new
    ///   constant). On the shipped plants every group step the line search
    ///   rejected sat at the rounding floor (≤ 6.9e-14 kg/s), and each rejection
    ///   costs the whole halving ladder of boundary recompiles.
    /// - **The step gets the per-node step's Armijo test** on `|Σ R_i|`, same
    ///   `ARMIJO_C` and `MAX_HALVINGS`. A rejected step writes NOTHING, so the
    ///   worst a rejection can do is the uncorrected solver's behaviour.
    /// - **The trial recompiles the group's BOUNDARY edges at the trial
    ///   pressures**, where the per-node trial stays frozen (M9.1). Graded on the
    ///   frozen coefficients, `relief_blowdown` at `dt = 1.0` cycles with period
    ///   two inside its PSV's accumulation band (20.180 ↔ 20.562 bar): the frozen
    ///   compile holds the valve's opening fixed, so a 38 kPa group shift passes
    ///   a test that cannot see the opening change. A node step is small and
    ///   local; a group step moves a relief valve's inlet by tens of kPa.
    ///   Internal edges still cancel under a recompile, because an edge has one
    ///   density whichever end reads it — so the cost is the boundary.
    ///
    /// **The baseline `Σ R_i` is taken on the frozen compile and only the trials
    /// recompile.** On a gas boundary edge the Armijo test therefore compares two
    /// residuals from two compiles, the baseline at the sweep's start-of-sweep
    /// density. That is what the prototype measured and it is kept so the
    /// shipped solver reproduces it bit for bit; it is inert on a liquid edge.
    #[allow(clippy::too_many_arguments)]
    fn correct_groups(
        &self,
        groups: &[Vec<NodeId>],
        incident: &BTreeMap<NodeId, Vec<(EdgeId, bool)>>,
        compiled: &BTreeMap<EdgeId, CompiledEdge>,
        pressures: &mut BTreeMap<NodeId, f64>,
        capacitive: &BTreeMap<NodeId, Capacitance>,
        graph: &PlantGraph,
        slate: &Slate,
        previous_states: &NodeStates,
        dt: f64,
        inlets: &[PumpInlet],
    ) -> Result<(), SimError> {
        for group in groups {
            let settled = group.iter().all(|&nid| {
                let (mut bal, mut scale) = (0.0f64, 0.0f64);
                for &(eid, incoming) in &incident[&nid] {
                    let c = &compiled[&eid];
                    let f = c.rho
                        * c.branch
                            .flow(pressures[&c.src] - pressures[&c.tgt], self.eps_dp);
                    bal += if incoming { f } else { -f };
                    scale = scale.max(f.abs());
                }
                if let Some(cap) = capacitive.get(&nid) {
                    bal += accumulation(cap, pressures[&nid], dt).0;
                }
                meets_node_bar(bal, scale, self.tol_abs_kg_s, self.tol_rel)
            });
            if settled {
                continue;
            }

            // A group beside a pump's inlet shifts on the plant as it answers,
            // the inlet re-solved at every trial, its slope a central
            // difference of that (M54, docs/DESIGN.md §59.1). Held still, the
            // inlet made the shift
            // undo the sweep: the check valve and the valve after a regulating
            // pump were pulled 180 kPa down every sweep, 5 000 sweeps running.
            let beside: Vec<PumpInlet> = {
                let mut found: Vec<PumpInlet> = Vec::new();
                for nid in group {
                    for inlet in adjacent_inlets(graph, &incident[nid], inlets) {
                        if !group.contains(&inlet.node) && !found.contains(&inlet) {
                            found.push(inlet);
                        }
                    }
                }
                found
            };
            if !beside.is_empty() {
                self.shift_beside_inlets(
                    group,
                    &beside,
                    incident,
                    compiled,
                    pressures,
                    capacitive,
                    (graph, slate, previous_states),
                    dt,
                )?;
                continue;
            }

            let (r0, d0) = group_imbalance(
                group,
                0.0,
                None,
                incident,
                compiled,
                pressures,
                capacitive,
                dt,
                self.eps_dp,
            )?;
            let full = r0 / d0;
            // `d0 > 0` whenever a member is a vessel or the group has a
            // conducting boundary edge; a group with neither has no slope to
            // step along and is left to the node-wise sweep.
            if !full.is_finite() || d0 <= 0.0 {
                continue;
            }

            let step = armijo_step(full, r0, |shift| {
                let mut trial = pressures.clone();
                for nid in group {
                    if let Some(p) = trial.get_mut(nid) {
                        *p += shift;
                    }
                }
                let (after, _) = group_imbalance(
                    group,
                    shift,
                    Some(TrialCompile {
                        graph,
                        slate,
                        previous_states,
                        pressures: &trial,
                        flash: self.line_flash.view(),
                    }),
                    incident,
                    compiled,
                    pressures,
                    capacitive,
                    dt,
                    self.eps_dp,
                )?;
                Ok(after)
            })?;
            if step != 0.0 {
                for nid in group {
                    if let Some(p) = pressures.get_mut(nid) {
                        *p += step;
                    }
                }
            }
        }
        Ok(())
    }
}

/// The Armijo ladder, shared by the node-wise step and the group step so that
/// "the group step gets the per-node step's test" (DESIGN §25 fork 3) is one
/// function rather than two copies that could drift.
///
/// Tries `t·full` for `t = 1, ½, …, 2^−MAX_HALVINGS`, where `residual_at(s)` is
/// the residual after a step `s` and `residual` the one before it, and returns
/// the first step with `|R(t·full)| ≤ (1 − ARMIJO_C·t)·|R|`. **If none passes it
/// returns exactly `0.0`**, and a caller writes nothing — the worst a rejection
/// can then do is leave the iterate where it was. Returning the last trial
/// instead is the tempting shape (the loop variable is right there) and would
/// write a step the criterion has just refused.
fn armijo_step<E>(
    full: f64,
    residual: f64,
    mut residual_at: impl FnMut(f64) -> Result<f64, E>,
) -> Result<f64, E> {
    let mut t = 1.0;
    for _ in 0..=MAX_HALVINGS {
        let trial = t * full;
        if residual_at(trial)?.abs() <= (1.0 - ARMIJO_C * t) * residual.abs() {
            return Ok(trial);
        }
        t *= 0.5;
    }
    Ok(0.0)
}

/// Most bisections of a node's bracket (M45.1). Not a tuning: each one halves
/// the bracket, and from any step an `f64` pressure can hold, 64 halvings reach
/// adjacent representable values — past that a bisection changes nothing.
const MAX_BISECTIONS: u32 = 64;

/// A node step found by BISECTION, for the node `armijo_step` could not move
/// (M45.1, docs/DESIGN.md §50).
///
/// A node's imbalance is monotone in its own pressure (this file's header), so
/// if the full Newton step's residual has the opposite sign to the current one,
/// the root lies inside `[0, full]`. The bracket is halved toward it until a
/// midpoint `settled` — the node's own convergence bar, the grading's — and
/// that midpoint is the step. Each half keeps the root, so this cannot fail
/// once bracketed; at the cap it returns the near end, whose residual is the
/// current one's sign and no larger (monotonicity), or `0.0` if it never moved.
/// With no bracket — the full step lands on the same side — it returns `0.0`,
/// and the caller writes nothing, as for a refused ladder.
fn bracketed_step<E>(
    full: f64,
    residual: f64,
    mut residual_at: impl FnMut(f64) -> Result<f64, E>,
    settled: impl Fn(f64) -> bool,
) -> Result<f64, E> {
    let at_full = residual_at(full)?;
    if at_full == 0.0 {
        return Ok(full);
    }
    if at_full.signum() == residual.signum() {
        return Ok(0.0);
    }
    let (mut near, mut far) = (0.0, full);
    for _ in 0..MAX_BISECTIONS {
        let mid = 0.5 * (near + far);
        let at_mid = residual_at(mid)?;
        if settled(at_mid) {
            return Ok(mid);
        }
        if at_mid.signum() == residual.signum() {
            near = mid;
        } else {
            far = mid;
        }
    }
    Ok(near)
}

/// What a group trial recompiles its boundary edges against: the trial
/// pressures, and what `network::compile_edge` needs besides.
struct TrialCompile<'a> {
    graph: &'a PlantGraph,
    slate: &'a Slate,
    previous_states: &'a NodeStates,
    pressures: &'a BTreeMap<NodeId, f64>,
    flash: LineFlash<'a>,
}

/// A group's net imbalance `Σ_{i∈K} R_i` [kg/s] with every member shifted by
/// `shift` [Pa], and its slope `Σ_boundary g_e + Σ C_i/dt` [kg/(s·Pa)].
///
/// Internal edges are skipped rather than summed: each appears once with each
/// sign and cancels exactly, and counting them in the slope would swamp it with
/// the very `g` the correction exists to step past. With `recompile` absent the
/// boundary edges use the frozen `compiled` coefficients (the baseline); with it
/// present each is recompiled at the trial pressures (fork 3).
#[allow(clippy::too_many_arguments)]
fn group_imbalance(
    group: &[NodeId],
    shift: f64,
    recompile: Option<TrialCompile<'_>>,
    incident: &BTreeMap<NodeId, Vec<(EdgeId, bool)>>,
    compiled: &BTreeMap<EdgeId, CompiledEdge>,
    pressures: &BTreeMap<NodeId, f64>,
    capacitive: &BTreeMap<NodeId, Capacitance>,
    dt: f64,
    eps_dp: f64,
) -> Result<(f64, f64), SimError> {
    let in_group = |n: &NodeId| group.binary_search(n).is_ok();
    let at = |n: NodeId| pressures[&n] + if in_group(&n) { shift } else { 0.0 };
    let (mut imbalance, mut slope) = (0.0, 0.0);
    for &nid in group {
        for &(eid, incoming) in &incident[&nid] {
            let frozen = &compiled[&eid];
            let other = if frozen.src == nid {
                frozen.tgt
            } else {
                frozen.src
            };
            if in_group(&other) {
                continue;
            }
            let fresh;
            let c = match &recompile {
                Some(r) => {
                    fresh = compile_edge_with(
                        r.graph,
                        eid,
                        r.slate,
                        r.previous_states,
                        r.pressures,
                        r.flash,
                    )?;
                    &fresh
                }
                None => frozen,
            };
            let dp = at(c.src) - at(c.tgt);
            let mdot = c.rho * c.branch.flow(dp, eps_dp);
            imbalance += if incoming { mdot } else { -mdot };
            slope += c.conductance(dp, eps_dp);
            // The pump's suction share moves with the group only when the pump
            // is in it (M50, §55); `other` is outside, so that is `c.src == nid`.
            if c.src == nid {
                slope += c.suction_share(dp, eps_dp);
            }
            // Its upwind density's share moves with the group when that end is
            // in it (M53, §58 fork 3); `other` is outside, so that is `nid`.
            let share = c.density_share(nid, dp, eps_dp);
            if share != 0.0 {
                slope += if c.src == nid { share } else { -share };
            }
        }
        if let Some(cap) = capacitive.get(&nid) {
            let (term, accumulation_slope) = accumulation(cap, at(nid), dt);
            imbalance += term;
            slope -= accumulation_slope;
        }
    }
    Ok((imbalance, slope))
}

/// The additive correction's groups, finest level first: a hierarchy of pairs by
/// greedy heavy-edge matching (the coarsening step of Karypis & Kumar, "A fast and
/// high quality multilevel scheme for partitioning irregular graphs", SIAM J. Sci.
/// Comput. 20 (1998) 359–392). Each returned group is sorted ascending.
///
/// Level 1 pairs each unknown with the unmatched neighbour it is most strongly
/// coupled to, the weight being the edge conductance `g_e = ρ·dQ/dΔP` at the
/// pass's seed pressures; edges are taken heaviest first, ties broken by
/// aggregate index. Each further level pairs the previous level's aggregates the
/// same way, the weight between two being the sum of the edges joining them. It
/// stops when no aggregate has a neighbour, so the top level IS each connected
/// set of unknowns.
///
/// **Why a hierarchy and not just the connected set** (§25 fork 2): two vessels
/// in one set, each behind its own wide relief line, have TWO slow modes, and one
/// common shift removes only their sum. Measured on the `two_vessel` prototype
/// plant: 4 472 sweeps with one group per set, 8 with the hierarchy.
///
/// **No threshold.** A strength cutoff is how most algebraic multigrid picks
/// groups, and it is a fitted constant. Matching pairs every node with its
/// heaviest neighbour however weak; an unhelpful group is paid for by the
/// acceptance test in `correct_groups`, not by a constant.
///
/// **A group of one is not a group.** Its common shift would be a second
/// node-wise step, which changes the iterate on every plant; excluded, the
/// fifteen shipped plants that never form a pair are byte-identical.
///
/// **Every aggregate of two or more is listed at EVERY level it exists at,**
/// including one that found no partner at that level and so is listed again
/// unchanged. That is what the prototype measured, and the per-node-bar skip in
/// `correct_groups` makes the repeat nearly free once the first pass has settled
/// the group. Removing the repeat is a measured change, not a tidy-up.
fn build_groups(
    unknowns: &[NodeId],
    incident: &BTreeMap<NodeId, Vec<(EdgeId, bool)>>,
    compiled: &BTreeMap<EdgeId, CompiledEdge>,
    pressures: &BTreeMap<NodeId, f64>,
    eps_dp: f64,
) -> Vec<Vec<NodeId>> {
    let weight_of = |eid: &EdgeId| -> f64 {
        let c = &compiled[eid];
        let dp = pressures[&c.src] - pressures[&c.tgt];
        c.conductance(dp, eps_dp)
    };
    let mut groups: Vec<Vec<NodeId>> = Vec::new();
    let mut aggregate_of: BTreeMap<NodeId, usize> =
        unknowns.iter().enumerate().map(|(i, n)| (*n, i)).collect();
    let mut members: Vec<Vec<NodeId>> = unknowns.iter().map(|n| vec![*n]).collect();
    loop {
        // Inter-aggregate weights. Each edge is visited from both ends and
        // counted from the lower-indexed aggregate only; edges to a pinned node,
        // and edges inside one aggregate, are not couplings between aggregates.
        let mut weights: BTreeMap<(usize, usize), f64> = BTreeMap::new();
        for &nid in unknowns {
            for &(eid, _) in &incident[&nid] {
                let c = &compiled[&eid];
                let other = if c.src == nid { c.tgt } else { c.src };
                if let (Some(&a), Some(&b)) = (aggregate_of.get(&nid), aggregate_of.get(&other)) {
                    if a < b {
                        *weights.entry((a, b)).or_insert(0.0) += weight_of(&eid);
                    }
                }
            }
        }
        if weights.is_empty() {
            break;
        }
        let mut pairs: Vec<((usize, usize), f64)> = weights.into_iter().collect();
        pairs.sort_by(|x, y| y.1.total_cmp(&x.1).then(x.0.cmp(&y.0)));
        let n = members.len();
        let mut mate: Vec<Option<usize>> = vec![None; n];
        for &((a, b), _) in &pairs {
            if mate[a].is_none() && mate[b].is_none() {
                mate[a] = Some(b);
                mate[b] = Some(a);
            }
        }
        let mut merged: Vec<Vec<NodeId>> = Vec::new();
        let mut new_index = vec![usize::MAX; n];
        for a in 0..n {
            if new_index[a] != usize::MAX {
                continue;
            }
            let mut group = members[a].clone();
            new_index[a] = merged.len();
            if let Some(b) = mate[a] {
                group.extend(members[b].iter().copied());
                new_index[b] = merged.len();
            }
            group.sort();
            merged.push(group);
        }
        if merged.len() == n {
            break;
        }
        for index in aggregate_of.values_mut() {
            *index = new_index[*index];
        }
        members = merged;
        groups.extend(members.iter().filter(|m| m.len() > 1).cloned());
    }
    groups
}

/// This node's mass-balance residual at a TRIAL pressure, every neighbour held
/// fixed — the same sum the sweep drives to zero, re-evaluated off the iterate.
/// The line search's only probe, and its only cost.
///
/// It must stay the same sum: measuring the trial against anything else would
/// grade a step by a residual nobody is solving, which is M5.3's finding in this
/// very file one paragraph down. So `capacitive` is threaded through and enters
/// via the SAME `network::accumulation`, and the edge flows come from the SAME
/// frozen `compiled` coefficients the step was differentiated against — except a
/// check valve's edge, which the step differentiated FRESH and which is
/// therefore recompiled at the trial too (`fresh_edge`, M30).
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
    (graph, slate, previous_states, flash): (&PlantGraph, &Slate, &NodeStates, LineFlash<'_>),
    (inlets, inlet_tol): (&[PumpInlet], f64),
) -> Result<f64, SimError> {
    let mut imbalance = 0.0;
    // The trial's pressures, built only when a check valve's edge needs them —
    // every other node evaluates its trial exactly as it did before M30.
    let mut trial_pressures: Option<BTreeMap<NodeId, f64>> = None;
    // A pump's inlet beside this node is re-solved at the trial (M54, §59): the
    // node is judged on the plant as it answers, the inlet back on its root.
    for inlet in adjacent_inlets(graph, incident, inlets) {
        let at_trial = trial_pressures.get_or_insert_with(|| {
            let mut t = pressures.clone();
            t.insert(nid, p_trial);
            t
        });
        solve_pump_inlet(
            graph,
            slate,
            previous_states,
            flash,
            inlet,
            at_trial,
            eps_dp,
            inlet_tol,
        )?;
    }
    for &(eid, incoming) in incident {
        let fresh;
        let c = if is_fresh_edge(graph, eid, flash) {
            let at_trial = trial_pressures.get_or_insert_with(|| {
                let mut t = pressures.clone();
                t.insert(nid, p_trial);
                t
            });
            fresh = compile_edge_with(graph, eid, slate, previous_states, at_trial, flash)?;
            &fresh
        } else {
            &compiled[&eid]
        };
        let at = |n: NodeId| match &trial_pressures {
            Some(t) => t[&n],
            None if n == nid => p_trial,
            None => pressures[&n],
        };
        let mdot = c.rho * c.branch.flow(at(c.src) - at(c.tgt), eps_dp);
        imbalance += if incoming { mdot } else { -mdot };
    }
    if let Some(cap) = capacitive {
        imbalance += accumulation(cap, p_trial, dt).0;
    }
    Ok(imbalance)
}

/// The pump inlets (`PumpInlet`) at the far end of one of a node's `incident`
/// edges: the ones its step must re-solve (M54, docs/DESIGN.md §59).
fn adjacent_inlets(
    graph: &PlantGraph,
    incident: &[(EdgeId, bool)],
    inlets: &[PumpInlet],
) -> Vec<PumpInlet> {
    if inlets.is_empty() {
        return Vec::new();
    }
    incident
        .iter()
        .filter_map(|&(eid, _)| {
            let (src, tgt) = graph.endpoints(eid);
            inlets
                .iter()
                .find(|i| i.node == src || i.node == tgt)
                .copied()
        })
        .collect()
}

/// True for an edge the node-wise sweep reads FRESH: the outlet edge of a check
/// valve, whose opening is a function of its own drop (docs/DESIGN.md §33), and
/// of a pump with a suction limit, whose head is a function of its own node's
/// pressure (M50, §55) — and EVERY edge of a plant whose line flash carries
/// vapour, whose density is a function of its upwind node's pressure (M53,
/// §58 fork 3).
///
/// The last is measured, not argued: frozen, a valve's outlet edge steps the
/// valve's pressure to the root of its density at the top of the sweep, and the
/// next sweep's density puts the root back across the bubble pressure — on the
/// M53 probe, a supply stepped from 110 °C to 125 °C cycled between 1.857 bar
/// (liquid) and 1.701 bar (208 kg/m³), 2 497 times each, though the slope was in
/// the step. Read fresh, the node's line search judges the true function.
fn is_fresh_edge(graph: &PlantGraph, eid: EdgeId, flash: LineFlash<'_>) -> bool {
    if flash.model.carries_vapour() {
        return true;
    }
    let (src, _) = graph.endpoints(eid);
    matches!(
        graph.node(src).kind,
        NodeKind::CheckValve { .. }
            | NodeKind::Pump {
                suction: Some(_),
                ..
            }
    )
}

/// A check valve's or a cavitating pump's edge compiled at `pressures` (see
/// `is_fresh_edge`), or `None` for any other edge.
///
/// **The node-wise sweep reads a check valve fresh, where it reads every other
/// edge off the coefficients frozen at the top of the sweep** (M30,
/// docs/DESIGN.md §33). The node step's slope carries the opening's share
/// (`CompiledEdge::conductance`), so a frozen opening in the step's own line
/// search judges a Newton step on the true function against a different one —
/// and on the M30 fixture each sweep then made about a third of the progress it
/// should, ~11 sweeps a tick against 3 for a plain valve. M21's group step
/// recompiles its trials for the same reason. A pump with a suction limit is
/// the same case (M50, §55): its head reads its own node's pressure, and the
/// node step's slope carries that share (`CompiledEdge::suction_share`). Scoped
/// to those two so every plant without one sweeps bit for bit as before.
fn fresh_edge(
    graph: &PlantGraph,
    eid: EdgeId,
    slate: &Slate,
    previous_states: &NodeStates,
    pressures: &BTreeMap<NodeId, f64>,
    flash: LineFlash<'_>,
) -> Result<Option<CompiledEdge>, SimError> {
    if !is_fresh_edge(graph, eid, flash) {
        return Ok(None);
    }
    compile_edge_with(graph, eid, slate, previous_states, pressures, flash).map(Some)
}

fn diverged(iterations: u32, residual: f64, residual_history: Vec<f64>) -> SimError {
    SimError::SolverDiverged {
        iterations,
        residual,
        residual_history,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::prepare;
    use refinery_core::components::Composition;
    use refinery_core::graph::{LeakRole, Node, NodeKind, Pipe};
    use refinery_core::stream::Stream;
    use refinery_core::units::{Kelvin, Meter, Pascal, Watt, WattPerKelvin, P_ATM, T_AMBIENT};

    /// **A rejected step writes nothing** (DESIGN §25 gate 6), asserted on the one
    /// ladder both steps use. It defends an edit INSIDE `armijo_step`; an edit at
    /// a call site is not covered.
    ///
    /// §25 specified this gate as a fixture whose group residual sits at the
    /// rounding floor. On the relief plants' numbers that fixture is estimated
    /// to be blind to the mutation it is for: at the floor `full = R/slope` is
    /// so small that even the ladder's last trial, `full/256`, comes out below
    /// one ULP of a pressure, so a step written in error would leave the
    /// iterate bit-identical. That is an estimate, never run, and it scales with
    /// `1/slope`, so a weak-boundary group could differ. What discriminates for
    /// certain is a
    /// rejection with a LARGE step, which a residual that never decreases gives
    /// directly: every trial must be refused and the answer must be exactly
    /// zero, not the last trial. The control is a residual that is linear in the
    /// step, where the full Newton step lands on the root and is taken at `t = 1`.
    #[test]
    fn a_step_every_trial_refuses_is_exactly_zero() {
        let mut probes = 0;
        let step = armijo_step(100.0, 1.0, |s| {
            probes += 1;
            Ok::<_, std::convert::Infallible>(1.0 + s.abs())
        })
        .unwrap_or_else(|never| match never {});
        assert_eq!(
            step.to_bits(),
            0.0f64.to_bits(),
            "a step no trial passed must be exactly +0.0, got {step:e}"
        );
        assert_eq!(
            probes,
            MAX_HALVINGS + 1,
            "every rung of the ladder must have been tried before refusing"
        );

        let control = armijo_step(100.0, 1.0, |s| {
            Ok::<_, std::convert::Infallible>(1.0 - s / 100.0)
        })
        .unwrap_or_else(|never| match never {});
        assert_eq!(
            control, 100.0,
            "the control: an exact Newton step is taken whole"
        );
    }

    fn node(name: &str, kind: NodeKind) -> Node {
        Node {
            name: name.into(),
            kind,
            heat_input: Watt::ZERO,
        }
    }

    fn pipe(name: &str, length_m: f64, diameter_m: f64) -> Pipe {
        Pipe {
            name: name.into(),
            length: Meter(length_m),
            diameter: Meter(diameter_m),
            friction_factor: 0.02,
            elevation_change: Meter(0.0),
            leak: LeakRole::None,
            ambient_ua: WattPerKelvin::ZERO,
            stream: Stream::stagnant(1, T_AMBIENT, P_ATM),
        }
    }

    /// Three junctions in series between a source and a sink,
    /// `feed — a —fat— b —thin— c — out`, prepared as the solver's first pass
    /// would see them: the cold seed puts all three at the mean of the pinned
    /// pressures, 2 bar.
    struct Chain {
        graph: PlantGraph,
        slate: Slate,
        prep: crate::network::Prepared,
        unknowns: Vec<NodeId>,
        incident: BTreeMap<NodeId, Vec<(EdgeId, bool)>>,
        a: NodeId,
        b: NodeId,
        c: NodeId,
    }

    fn chain() -> Chain {
        let water = Composition::pure(1, 0);
        let mut graph = PlantGraph::new();
        let feed = graph.add_node(node(
            "feed",
            NodeKind::Source {
                pressure: Pascal(3.0e5),
                temperature: Kelvin(293.15),
                composition: water.clone(),
            },
        ));
        let a = graph.add_node(node("a", NodeKind::Junction));
        let b = graph.add_node(node("b", NodeKind::Junction));
        let c = graph.add_node(node("c", NodeKind::Junction));
        let out = graph.add_node(node(
            "out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: Kelvin(293.15),
                composition: water,
            },
        ));
        graph.add_pipe(feed, a, pipe("in_line", 20.0, 0.10));
        graph.add_pipe(a, b, pipe("fat", 1.0, 0.30));
        graph.add_pipe(b, c, pipe("thin", 20.0, 0.02));
        graph.add_pipe(c, out, pipe("out_line", 20.0, 0.10));

        let slate = Slate::water_only();
        let prep = prepare(&graph, &slate, &NodeStates::default(), &BTreeMap::new())
            .expect("a liquid chain prepares");
        let unknowns: Vec<NodeId> = prep
            .classes
            .free
            .iter()
            .copied()
            .filter(|n| prep.anchored.contains(n))
            .collect();
        assert_eq!(
            unknowns,
            vec![a, b, c],
            "premise: the three junctions are the unknowns"
        );
        let incident: BTreeMap<NodeId, Vec<(EdgeId, bool)>> = unknowns
            .iter()
            .map(|&n| {
                let edges = graph
                    .incident(n)
                    .into_iter()
                    .map(|(eid, _, incoming)| (eid, incoming))
                    .collect();
                (n, edges)
            })
            .collect();
        Chain {
            graph,
            slate,
            prep,
            unknowns,
            incident,
            a,
            b,
            c,
        }
    }

    /// **The groups are a hierarchy of heaviest-first pairs, and never a node
    /// alone** (DESIGN §25 fork 2; mutations 1, 4 and 6).
    ///
    /// On `chain()`, level 1 must pair `a` with `b` across the fat line (heaviest
    /// first) and leave `c` out rather than list it alone; level 2 must pair that
    /// pair with `c`, which is the whole connected set. So exactly
    /// `[[a, b], [a, b, c]]`:
    /// - one group per connected set only (mutation 1) gives `[[a, b, c]]`;
    /// - admitting a group of one (mutation 4) lists `[c]`. Measured: that moves
    ///   three grouped plants and this test alone catches it. It does NOT move
    ///   the fifteen plants that form no pair, because a plant with no two
    ///   neighbouring unknowns never builds a level to admit anything into;
    /// - lightest-edge matching (mutation 6) pairs `b` with `c` first.
    #[test]
    fn groups_pair_the_heaviest_link_first_and_never_hold_one_node() {
        let ch = chain();
        let groups = build_groups(
            &ch.unknowns,
            &ch.incident,
            &ch.prep.compiled,
            &ch.prep.pressures,
            SimpleFlowSolver::default().eps_dp,
        );
        assert_eq!(groups, vec![vec![ch.a, ch.b], vec![ch.a, ch.b, ch.c]]);
    }

    /// **A group whose members all meet the solve's own per-node bar is not
    /// stepped** (DESIGN §25 fork 3's skip rule; mutation 8).
    ///
    /// The skip is `network::meets_node_bar`, the stopping test itself. Testing
    /// against `tol_abs` alone instead moves nine plants' numbers and fails no
    /// plant-level gate — a bytes claim with nothing in CI defending it, which is
    /// what this test is for.
    ///
    /// At the cold seed `a` takes the feed's whole inflow and passes nothing on,
    /// so its residual EQUALS its own traffic: `|R| = scale`. A solver whose
    /// `tol_rel` is 2 grades that as met; `tol_abs` alone never would. So the
    /// group `[a, b]` must come back bit-identical. The control is the same call
    /// with both tolerances at zero, where nothing is met and the group moves —
    /// without it, a correction that did nothing at all would pass.
    #[test]
    fn a_group_already_meeting_the_node_bar_is_left_alone() {
        let ch = chain();
        let group = vec![vec![ch.a, ch.b]];
        let correct = |tol_abs_kg_s: f64, tol_rel: f64| {
            let solver = SimpleFlowSolver {
                tol_abs_kg_s,
                tol_rel,
                ..SimpleFlowSolver::default()
            };
            let mut pressures = ch.prep.pressures.clone();
            solver
                .correct_groups(
                    &group,
                    &ch.incident,
                    &ch.prep.compiled,
                    &mut pressures,
                    &ch.prep.classes.capacitive,
                    &ch.graph,
                    &ch.slate,
                    &NodeStates::default(),
                    0.1,
                    &[],
                )
                .expect("a liquid group corrects");
            pressures
        };
        let bits = |p: &BTreeMap<NodeId, f64>| -> Vec<u64> {
            [ch.a, ch.b].iter().map(|n| p[n].to_bits()).collect()
        };

        let seed = bits(&ch.prep.pressures);
        assert_eq!(
            bits(&correct(1e-8, 2.0)),
            seed,
            "a group every member of which meets tol_abs + tol_rel·scale must be skipped"
        );
        assert_ne!(
            bits(&correct(0.0, 0.0)),
            seed,
            "control: with nothing met, the same group must be stepped"
        );
    }
}
