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
//!   at P_ATM, Tank at hydrostatic bottom pressure (constant within one solve)
//!   — while it can cover the tick's outflow. A tank the solve would draw past
//!   empty is re-solved as a FREE node supplying `m/dt` (M24, docs/DESIGN.md
//!   §28; `network::solve_with_active_anchoring`).
//! - Every M1 element is affine in `Q·|Q|` (`dp = α·Q|Q| + β`), so a pipe and
//!   the device folded into it compose in closed form (`Σα, Σβ`) and invert to
//!   `Q(dp)` with no inner solve — see `elements::QuadraticBranch`.
//! - Residual: mass balance at every free node,
//!   `R_i = Σ_incoming ṁ − Σ_outgoing ṁ = 0`, ṁ = ρ·Q(dP_branch).
//! - Jacobian: analytic. With branch conductance `g_e = ρ·dQ/d(dp) ≥ 0`, the
//!   system is `J = −L`, a weighted graph Laplacian: symmetric, negative-
//!   definite once each connected component has a pinned node ⇒ unique
//!   solution — **except in a relief valve's accumulation band**, where its
//!   opening rises with its own inlet pressure and that derivative lands in one
//!   column only (M26, docs/DESIGN.md §30). `J` is then not symmetric, which the
//!   solve never assumed: faer's partial-pivot dense LU (networks are small;
//!   sparse is a later upgrade).
//! - Damping: halve the Newton step until ‖R‖_∞ decreases, max 8 halvings.
//! - Convergence, per node: |R_n| < tol_abs + tol_rel·scale_n, where scale_n is
//!   the largest |ṁ| on that node's OWN incident active edges (M9.2,
//!   `network::grade_nodes`) — DESIGN §3's "relative mass-imbalance per node",
//!   which until M9.2 was coded against the whole network's throughput instead.
//!   Hard cap max_iter, then
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

use crate::elements::{cavitation_head_fraction, cavitation_head_fraction_dsigma};
use crate::network::{
    accumulation, compile_edges_with, edge_flows, finalize, pump_inlets, solve_pump_inlet,
    solve_remembering_reliefs, solve_with_active_anchoring_with, validate_degrees, AnchorPass,
    Capacitance, CompiledEdge, OwnedLineFlash, Prepared, PumpInlet,
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
    /// Relative residual tolerance [kg/s per kg/s], multiplied by each node's
    /// OWN incident flow scale rather than by the network's throughput
    /// (`network::grade_nodes`).
    pub tol_rel: f64,
    /// Regularization epsilon for sqrt laws [Pa].
    pub eps_dp: f64,
    /// Warm-start pressures from the previous converged solve, keyed by NodeId.
    warm_start: BTreeMap<NodeId, f64>,
    /// The line flash this solver reads two-phase densities through (M53,
    /// docs/DESIGN.md §58 fork 3). `NoLineFlash` by default, under which every
    /// compile is the liquid one bit for bit; the loader sets it from
    /// `[fidelity] line_flash`.
    pub line_flash: OwnedLineFlash,
}

impl NewtonFlowSolver {
    /// This solver reading two-phase densities through `line_flash` (M53,
    /// docs/DESIGN.md §58 fork 3) — what the loader calls for a plant selecting
    /// `[fidelity] line_flash`.
    pub fn with_line_flash(mut self, line_flash: OwnedLineFlash) -> Self {
        self.line_flash = line_flash;
        self
    }
}

impl Default for NewtonFlowSolver {
    fn default() -> Self {
        Self {
            max_iter: 50,
            tol_abs_kg_s: 1e-8,
            tol_rel: 1e-8,
            eps_dp: 1.0,
            warm_start: BTreeMap::new(),
            line_flash: OwnedLineFlash::default(),
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
        "newton-network"
    }
}

impl NewtonFlowSolver {
    /// Solve every pump inlet in `inlets` at `pressures`, in order (M54,
    /// docs/DESIGN.md §59; `network::solve_pump_inlet`), to a hundredth of the
    /// absolute bar so its residual never decides convergence. One with no root
    /// keeps the pressure it was handed.
    fn solve_pump_inlets(
        &self,
        inlets: &[PumpInlet],
        graph: &PlantGraph,
        slate: &Slate,
        previous_states: &NodeStates,
        pressures: &mut BTreeMap<NodeId, f64>,
    ) -> Result<(), SimError> {
        for &inlet in inlets {
            solve_pump_inlet(
                graph,
                slate,
                previous_states,
                self.line_flash.view(),
                inlet,
                pressures,
                self.eps_dp,
                0.01 * self.tol_abs_kg_s,
            )?;
        }
        Ok(())
    }

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

        // A pump's inlet on a flashing plant is solved inside every iterate, the
        // first included, and its edges recompiled there (M54, §59).
        let inlets = pump_inlets(graph, self.line_flash.view(), |nid| idx.contains_key(&nid));
        if !inlets.is_empty() {
            match self.solve_pump_inlets(&inlets, graph, slate, previous_states, &mut pressures) {
                Ok(()) => {}
                Err(e) => {
                    return AnchorPass {
                        result: Err(e),
                        pressures,
                    }
                }
            }
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
        }

        // Damped Newton.
        let capacitive = &prep.classes.capacitive;
        let mut history: Vec<f64> = Vec::new();
        let (mut r, mut jac, scale) = assemble(
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
        // ∞-norm for reporting, per-node grading for the stop decision — one
        // definition, shared with the Simple sweep (`network::grade_nodes`).
        let grade = |r: &[f64], scale: &[f64]| {
            crate::network::grade_nodes(
                r.iter().copied().zip(scale.iter().copied()),
                self.tol_abs_kg_s,
                self.tol_rel,
            )
        };
        let (mut res, mut converged) = grade(&r, &scale);
        let mut merit = half_sq_norm(&r); // ½‖R‖₂²: smooth line-search merit
        history.push(res);
        let mut iterations = 0u32;

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
            //
            // The ladder starts short of a step that would carry a check valve
            // from full lift to shut (M49, docs/DESIGN.md §54): `band_cut`. Each
            // trial carries a cavitating pump's outlet along the head its
            // suction step buys (M51, §56): `follow_pump_heads`.
            let mut t = band_cut(&compiled, &idx, &dp);
            let mut accepted = false;
            for _ in 0..=MAX_HALVINGS {
                let mut trial = apply_step(&pressures, &unknowns, &idx, &dp, t);
                follow_pump_heads(&compiled, &idx, &dp, t, &mut trial);
                // A pump's inlet is solved at the trial, not stepped (M54,
                // docs/DESIGN.md §59): its balance is held at zero, so the full
                // step on the others is the Schur step. Where it has no root
                // with its neighbours held, the step stands for that node.
                match self.solve_pump_inlets(&inlets, graph, slate, previous_states, &mut trial) {
                    Ok(()) => {}
                    Err(e) => {
                        return AnchorPass {
                            result: Err(e),
                            pressures,
                        }
                    }
                }
                // Recompile at the trial iterate: a gas edge's frozen density
                // coefficient follows the pressure it is evaluated at, so the
                // merit the line search compares must be the merit of the fully
                // consistent trial, not of the old coefficients at a new
                // pressure. For an all-liquid network this reproduces the same
                // `CompiledEdge` bit for bit (M5.2, `compile_edge`).
                let compiled_t = match compile_edges_with(
                    graph,
                    slate,
                    previous_states,
                    &trial,
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
                let (r_t, jac_t, scale_t) = assemble(
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
                    (res, converged) = grade(&r_t, &scale_t);
                    merit = merit_t;
                    r = r_t;
                    jac = jac_t;
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
/// floating node are inert (zero flow). Returns (R, J, per-node scale).
///
/// A CAPACITIVE node adds `−C·(P − Pⁿ)/dt` to its own residual and `−C/dt` to its
/// own diagonal, through the shared `network::accumulation` so the Simple sweep
/// cannot end up solving a different fixed point. The term touches nothing
/// off-diagonal: `m(P)` is a function of that node's pressure alone, so it adds
/// no asymmetry and makes the diagonal strictly more negative — better
/// conditioned, not merely still invertible.
///
/// A RELIEF valve's outlet edge adds one more term, and that one is not
/// symmetric: its opening is a function of its own inlet pressure, so
/// `ṁ·k` (`k = CompiledEdge::relief_opening_log_slope`) belongs in the source's
/// column alone. Zero outside the accumulation band, and skipped when zero, so a
/// plant whose PSV never lifts assembles bit for bit what it did before M26
/// (docs/DESIGN.md §30). A CHECK valve's term is its sibling and IS symmetric:
/// the disc reads the drop across its own branch, so `ṁ·k`
/// (`k = CompiledEdge::check_opening_log_slope`) is added to the edge's
/// conductance `g` (M30, §33).
///
/// This is what makes one solve an implicit-Euler step of a DAE rather than a
/// steady state (DESIGN §3a fork 2). The convergence scale deliberately excludes
/// it: the scale is mass FLOW, and a vessel's accumulation is measured against
/// that, not added to it. M9.2 sharpened the reason without changing the rule —
/// see `network::grade_nodes`.
///
/// The third return is that scale, one entry per unknown in `idx` order:
/// `scale[i] = max |ṁ_e|` over node i's own incident active edges. It replaced a
/// single network-wide `throughput` in M9.2.
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
) -> (Vec<f64>, Vec<Vec<f64>>, Vec<f64>) {
    let mut r = vec![0.0; n];
    let mut jac = vec![vec![0.0; n]; n];
    let mut scale = vec![0.0f64; n];
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
        // Conductance ≥ 0. A check valve's opening reads this branch's own drop,
        // so its share of the slope is a share of THIS term — both columns,
        // symmetric (docs/DESIGN.md §33) — and `conductance` owns the sum.
        let g = c.conductance(dp, eps);
        let si = idx.get(&c.src).copied();
        let ti = idx.get(&c.tgt).copied();
        for end in [si, ti].into_iter().flatten() {
            scale[end] = scale[end].max(mdot.abs());
        }
        // The relief opening's share of `∂ṁ/∂P_src`, which `g` cannot carry
        // because the branch froze the opening. Source column only.
        // A cavitating pump's head moves with its own node's pressure the same
        // way (M50, §55), so its share joins the relief's: source column only,
        // and zero on every other edge.
        let opening_term = mdot * c.relief_opening_log_slope + c.suction_share(dp, eps);
        if opening_term != 0.0 {
            if let Some(s) = si {
                jac[s][s] -= opening_term;
                if let Some(t) = ti {
                    jac[t][s] += opening_term;
                }
            }
        }
        // A two-phase edge's flow moves with its UPWIND node's pressure through
        // its density (M53, docs/DESIGN.md §58 fork 3): that share goes in the
        // upwind node's column, the relief's arrangement for whichever end the
        // stream leaves. Absent on every edge whose stream is liquid.
        if let Some(slope) = c.density_slope {
            let share = slope.share(dp, eps);
            if share != 0.0 {
                if let Some(u) = idx.get(&slope.upwind).copied() {
                    if let Some(s) = si {
                        jac[s][u] -= share;
                    }
                    if let Some(t) = ti {
                        jac[t][u] += share;
                    }
                }
            }
        }
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
    (r, jac, scale)
}

/// The longest step `t ≤ 1` the line search may start from: short of carrying a
/// check valve from FULL LIFT to SHUT in one move (M49, docs/DESIGN.md §54).
///
/// A disc at full lift has its whole conductance in the Jacobian; a shut one has
/// none, so on the far side the step is set without it — on A21's chain by a
/// valve cracked 1% open alone, a megapascal long, where 1/256 of it throws the
/// disc wide open again and the line search gives up. Nor will Armijo refuse the
/// crossing: a shut disc stops the reverse flow that makes the √-law's mirror
/// step bad everywhere else (§11), so the trial's residual can look like
/// progress. Such a step is cut to land the drive at mid-band — where the
/// opening's slope is largest — on the linear change in drive the step
/// predicts (exact in liquid, where `β` is a fixed head).
///
/// One direction only. Shut to wide open crosses the band too, but it lands
/// where the disc's conductance is back in the Jacobian; and a disc inside its
/// band may shut, which is how a solve whose answer has it shut gets there.
/// `1.0` — every step's start before M49 — wherever no disc is at full lift.
fn band_cut(
    compiled: &BTreeMap<EdgeId, CompiledEdge>,
    idx: &BTreeMap<NodeId, usize>,
    dp: &[f64],
) -> f64 {
    let step = |nid: &NodeId| idx.get(nid).map_or(0.0, |&i| dp[i]);
    let mut t: f64 = 1.0;
    for c in compiled.values() {
        let Some(band) = c.check_band else {
            continue;
        };
        if band.full_open.is_nan() || band.full_open <= 0.0 || band.drive < band.full_open {
            continue;
        }
        let change = step(&c.src) - step(&c.tgt);
        if band.drive + change <= 0.0 {
            t = t.min((0.5 * band.full_open - band.drive) / change);
        }
    }
    t
}

/// Moves a cavitating pump's OUTLET with the head its suction step really buys
/// (M51, docs/DESIGN.md §56, ledger row A23): a curved line search, `P(t)`,
/// rather than the straight `P + t·ΔP`.
///
/// A pump in partial cavitation is a lever. Its head is `φ(σ)·ρ·g·h0`, and on
/// the M50 demo one pascal more at the suction is twenty more at the outlet
/// (`dbeta_dp`). The pump's branch then pins `P_out − P_suction − φ·ρ·g·h0`
/// near its small `αQ²`, so the solve's answer lies along a curve shaped like
/// `φ` — and a straight step leaves it. The step is computed on the tangent of
/// `φ`, and a long step from near `φ`'s steepest point runs past the curve's
/// ceiling: throttling the demo from 0.6 to 0.2 predicted `φ` = 1.36 where it
/// is 0.97, put the outlet a bar too high, and the line search could only
/// accept slivers of a step. Newton gave up at 50 iterations, on the move the
/// demo exists to teach, while every cold start converged.
///
/// The cure is a change of unknown at the pump's outlet: measure it from the
/// head the pump delivers, `u = P_out − φ(σ(P_suction))·ρ·g·h0`, in which the
/// branch's drive is linear in the step. Taking the Newton step in `u` and
/// mapping back adds `(φ(σ + t·Δσ) − φ(σ) − φ'(σ)·t·Δσ)·ρ·g·h0` to the
/// outlet's trial pressure. That is second order in `t`, so the path leaves
/// the iterate along the Newton direction and Armijo's sufficient-decrease
/// test is still the right test of it. It takes the failing move in 4
/// iterations.
///
/// Skipped where the outlet is not an unknown (a pinned pressure takes no
/// correction) or the suction is not. Zero on every pump without
/// `npsh_required_m`, which has no `pump_suction`, so no plant without the key
/// can move. Not built, because no plant has one: two keyed pumps into one
/// node (their corrections add), two in series (the downstream pump's
/// correction ignores the upstream one's at its own suction), and a check
/// valve on a keyed pump's outlet (`band_cut` sets `t` before this moves the
/// disc's drive).
fn follow_pump_heads(
    compiled: &BTreeMap<EdgeId, CompiledEdge>,
    idx: &BTreeMap<NodeId, usize>,
    dp: &[f64],
    t: f64,
    trial: &mut BTreeMap<NodeId, f64>,
) {
    for c in compiled.values() {
        let Some(s) = c.pump_suction else {
            continue;
        };
        let (Some(&i), true) = (idx.get(&c.src), idx.contains_key(&c.tgt)) else {
            continue;
        };
        let dsigma = s.dsigma_dp * t * dp[i];
        let actual = cavitation_head_fraction(s.sigma + dsigma);
        let tangent = s.head_fraction + cavitation_head_fraction_dsigma(s.sigma) * dsigma;
        let lift = (actual - tangent) * s.shutoff_head;
        if lift != 0.0 {
            if let Some(outlet) = trial.get_mut(&c.tgt) {
                *outlet += lift;
            }
        }
    }
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
