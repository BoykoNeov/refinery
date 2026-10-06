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

use crate::elements::{
    check_opening, check_opening_slope, fold_gas_valve, gas_orifice, pipe_resistance,
    relief_opening, relief_opening_slope, specific_heat_ratio_factor, QuadraticBranch, CHOKE_BLEND,
    ORIFICE_CD,
};
use refinery_core::components::{Phase, Slate};
use refinery_core::energy::{boundary_temperature, NodeStates};
use refinery_core::error::SimError;
use refinery_core::graph::{
    Blowdown, EdgeId, LeakRole, Node, NodeId, NodeKind, Pipe, PlantGraph, TankState,
};
use refinery_core::traits::{HydraulicSolution, SolveDiagnostics, StarvedTank};
use refinery_core::units::{KgPerSec, Pascal, Seconds, Watt, G, P_ATM};
use std::collections::{BTreeMap, BTreeSet};

/// Valve openings below this snap to fully closed, so a valve "cracked to
/// 1e-9" cannot anchor a subnetwork with a numerically negligible (near-
/// singular) conductance — it is treated as closed instead.
pub const OPEN_EPS: f64 = 1e-6;
/// Reference density for valve SG (ρ_rel). Matches `PseudoComponent::water`
/// so water gives ρ_rel = 1.0.
pub const RHO_WATER_REF: f64 = 998.0;
/// Floor [Pa] on the pressure at which a gas edge's transport density is
/// evaluated.
///
/// A Newton iterate can overshoot to a non-positive pressure on its way to the
/// root; `rho = P*M/(R*T)` would then be zero or negative and `compile_edge`
/// would `Err` out of a *transient* rather than letting the line search reject
/// the step. Flooring the evaluation pressure keeps the coefficient finite and
/// tiny, so the trial's residual is enormous and Armijo discards it - which is
/// the mechanism that already handles every other bad step. At any converged
/// physical solution `P >> 1 Pa`, so the floor never touches an answer; it is a
/// regularisation of the ITERATE, the same role `eps_dp` plays for `sqrt(dp)`.
pub const RHO_EVAL_P_FLOOR: f64 = 1.0;

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
    /// `d ln ṁ / d P_src` [1/Pa] through a relief valve's OPENING, which the
    /// branch above holds frozen at the iterate it was compiled at (M26,
    /// docs/DESIGN.md §30). Newton adds `ṁ·k` to the source column of its
    /// Jacobian; without it, a PSV partly open on a vessel at a long timestep
    /// makes every Newton step overshoot by about a whole step (ledger row A14).
    ///
    /// `0.0` on every edge that is not a relief valve's outlet, on a relief valve
    /// outside its accumulation band (the opening's slope is exactly zero there),
    /// and on one whose opening snapped shut below `OPEN_EPS`. The game fidelity
    /// never reads it: its sweep and group step use `flow_ddp` alone.
    pub relief_opening_log_slope: f64,
    /// `d ln ṁ / dS` [1/Pa] through a CHECK valve's opening, `S` being the
    /// forward drive across its branch (M30, docs/DESIGN.md §33). The relief
    /// term's sibling with one difference that matters: the disc reads
    /// `P_src − P_tgt`, not `P_src` alone, so Newton adds `ṁ·k` to the edge's
    /// CONDUCTANCE — both columns, symmetrically — rather than to the source
    /// column. `0.0` everywhere the relief term is, for the same reasons.
    pub check_opening_log_slope: f64,
}

impl CompiledEdge {
    /// The edge's conductance `dṁ/d(P_src − P_tgt)` [kg/(s·Pa)] at drop `dp`:
    /// the frozen branch's `ρ·dQ/ddp`, plus a check valve's opening share `ṁ·k`
    /// (docs/DESIGN.md §33). The ONE owner of that sum, read by Newton's
    /// assembly and by the game solver's node step, group step and grouping
    /// alike, so the two fidelities cannot differentiate a check valve two ways.
    ///
    /// The share is added only when nonzero, so every edge that is not a check
    /// valve inside its band returns `ρ·dQ/ddp` bit for bit. The game solver
    /// NEEDS it, which the relief term's history did not predict: a check valve
    /// sits inside its band in ordinary running, its opening moves with its
    /// own drop, and a node step on the frozen slope overshoots by the ratio of
    /// the two — about three on the M30 fixture, which diverged at tick 959.
    pub fn conductance(&self, dp: f64, eps: f64) -> f64 {
        let frozen = self.rho * self.branch.flow_ddp(dp, eps);
        if self.check_opening_log_slope == 0.0 {
            return frozen;
        }
        let opening_share = self.rho * self.branch.flow(dp, eps) * self.check_opening_log_slope;
        if opening_share != 0.0 {
            frozen + opening_share
        } else {
            frozen
        }
    }
}

/// Boundary classification of the graph's nodes for one solve: which nodes pin
/// pressure, which are free unknowns, and the deterministic cold-start seed.
///
/// The anchored set is NOT here, and that ordering is load-bearing since M5.2:
/// anchoring needs the compiled edges, compiling a gas edge needs a pressure to
/// evaluate ρ(P,T) at, and that pressure needs the cold seed. `prepare` owns
/// the resulting three-step order so neither solver can get it wrong.
pub struct Classification {
    /// Pinned pressures [Pa] for fixed nodes (Source/Sink/Atmosphere, a WET
    /// Tank, a Column).
    pub fixed: BTreeMap<NodeId, f64>,
    /// Free node ids (Junction/Pump/Valve/Vessel, a STARVED Tank), ascending —
    /// deterministic order. A capacitive vessel is FREE: its pressure is an
    /// unknown the solve determines, it is simply an unknown with an equation of
    /// its own. So is a starved tank (M24, docs/DESIGN.md §28 fork 2).
    pub free: Vec<NodeId>,
    /// Free nodes carrying a term of their own in their own residual, ascending.
    /// A subset of `free`: a vessel's `(C, Pⁿ)`, or a starved tank's supply.
    pub capacitive: BTreeMap<NodeId, Capacitance>,
    /// Deterministic cold-start pressure seed: mean of the fixed pressures, or
    /// P_ATM when the network has no fixed node at all.
    ///
    /// A capacitive node never falls back to it — its own `Pⁿ` is a better cold
    /// start and always exists — which also keeps a closed gas system, where
    /// there are no fixed pressures to average, off a mean of an empty set.
    pub cold: f64,
}

/// Everything both solvers need before their first iteration, built in the one
/// order that is consistent (see `Classification`).
pub struct Prepared {
    pub classes: Classification,
    /// Edges compiled at the seeded pressures. Both solvers RECOMPILE this each
    /// iteration (`compile_edges`) so a gas edge's frozen density coefficient
    /// tracks the pressure iterate; only `anchored` is computed once, from this
    /// first compile, so the anchored set cannot flap mid-solve.
    pub compiled: BTreeMap<EdgeId, CompiledEdge>,
    /// (Fixed ∪ capacitive) ∪ free-reachable-from-those-via-conducting-edges.
    /// Free nodes NOT in this set are floating (indeterminate pressure); their
    /// incident edges carry zero. A capacitive node is always in it, with or
    /// without a conducting path to a reservoir.
    pub anchored: BTreeSet<NodeId>,
    /// Seeded pressures: fixed pinned, free warm-started, else `Pⁿ` for a
    /// capacitive node and the cold mean for any other; floating free
    /// warm-started or at `P_ATM`.
    pub pressures: BTreeMap<NodeId, f64>,
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
            // A relief valve is hydraulically a valve: its characteristic folds
            // into its single outlet edge, so exactly one of each is what makes
            // the fold well defined.
            | NodeKind::ReliefValve { .. }
            // So is a check valve, for the same reason.
            | NodeKind::CheckValve { .. }
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
        // The density is `TankState`'s own (`TankState::density`) rather than
        // computed here, so the level this head is built on and the level a
        // control loop measures are the same number by construction (M8.2).
        //
        // A WET tank. One the solve would draw past empty is classified STARVED
        // instead, before this is asked (`classify`, M24, docs/DESIGN.md §28).
        NodeKind::Tank(t) => Some(t.bottom_pressure(slate).value()),
        // A column runs on pressure control: its operating pressure is pinned,
        // exactly like a Source/Sink/Tank, so the feed edge is an ordinary
        // pressure-driven edge into a fixed node and the draws (fixed→fixed) never
        // enter the Jacobian (DESIGN §5).
        NodeKind::Column { pressure, .. } => Some(pressure.value()),
        // Furnaces, coolers and reactors pin no pressure: all are hydraulically
        // pass-throughs, so they are free nodes whose pressure the network
        // determines. A reactor is hydraulically a furnace (DESIGN §5).
        //
        // A VESSEL is free too, and that is the point of it: a holdup that pinned
        // its pressure would be the rejected explicit scheme (DESIGN §3a fork 2).
        // Its inventory enters through `capacitance` instead, as a term in its own
        // residual — so it is an ANCHOR without being FIXED, which is a
        // distinction this file did not have to make before.
        NodeKind::Pump { .. }
        | NodeKind::Valve { .. }
        | NodeKind::ReliefValve { .. }
        | NodeKind::CheckValve { .. }
        | NodeKind::Junction
        | NodeKind::Furnace { .. }
        | NodeKind::Cooler { .. }
        | NodeKind::Reactor { .. }
        | NodeKind::Vessel(_)
        | NodeKind::HeatExchanger => None,
    }
}

/// The capacitive state of a node that stores mass against pressure, or `None`
/// for every node that does not.
///
/// The counterpart of `fixed_pressure`, and the two are mutually exclusive by
/// construction: a node either pins a pressure, carries a capacitance, or is a
/// zero-volume algebraic junction. Together they are the three node classes
/// DESIGN §3a fork 2 unifies — `C → 0` is the junction, `C → ∞` the reservoir.
pub fn capacitance(node: &Node, slate: &Slate) -> Option<Capacitance> {
    match &node.kind {
        NodeKind::Vessel(vessel) => Some(Capacitance::Vessel {
            c: vessel.capacitance(slate),
            p_prev: vessel.pressure(slate).value(),
        }),
        _ => None,
    }
}

/// One free node's own term in its own mass balance.
///
/// Two forms, and the second is M24's (docs/DESIGN.md §28 fork 2). Both enter
/// the residual through `accumulation` and nowhere else, so the sites that
/// assemble a residual — Newton's `assemble`, the Simple sweep, its grading, its
/// group correction and its per-node trial — needed no second list to keep in
/// step: a starved tank is one more entry in `Classification::capacitive`.
#[derive(Debug, Clone, Copy)]
pub enum Capacitance {
    /// A capacitive vessel.
    Vessel {
        /// `C = dm/dP` [kg/Pa], exact at the vessel's start-of-tick `T` and `M̄`.
        c: f64,
        /// `Pⁿ` [Pa]: the pressure the START-OF-TICK inventory implies, `mⁿ/C`.
        ///
        /// A fact about the state, never a warm start and never the cold seed —
        /// the accumulation term measures `m(P) − mⁿ` from here, so seeding a
        /// vessel anywhere else would make the step integrate from a mass it
        /// never held.
        p_prev: f64,
    },
    /// A STARVED tank: one that would have been drawn past empty this tick, and
    /// is solved instead as a free node supplying everything it holds.
    ///
    /// **Not an anchor**, unlike a vessel: its term is a constant, so its
    /// equation says nothing about its own pressure. A starved tank with no
    /// other pressure reference in its subnetwork floats and its edges carry
    /// zero — which is right, because nothing is driving them (§28 gate 10).
    /// `base_anchors` is where that is decided.
    Starved {
        /// `m/dt` [kg/s] (`starved_supply`): the whole start-of-tick inventory,
        /// delivered over the tick.
        supply: f64,
        /// The bottom pressure [Pa] the tank would pin at while wet, `P_ATM +
        /// ρgh(m)`. Only a SEED, used when neither the previous pass nor the
        /// warm start has a value — it never reaches the residual.
        p_pinned: f64,
    },
}

/// The supply a starved tank delivers [kg/s]: its start-of-tick inventory over
/// one tick. The single expression the classification and the solve's report
/// share, so the rate the solve used and the rate it reports cannot differ.
pub fn starved_supply(tank: &TankState, dt: Seconds) -> f64 {
    tank.mass.value() / dt.value()
}

/// A free node's own term in its own residual [kg/s], and its derivative w.r.t.
/// that node's pressure.
///
/// **A vessel:** the accumulation term `−C·(P − Pⁿ)/dt` and its derivative `−C/dt`.
///
/// **A starved tank:** `+m/dt` and a slope of exactly zero. The supply is mass
/// ENTERING the network at that node — the sign follows the residual's
/// convention below — and it does not depend on the pressure, because a tank
/// that is empty by the end of the tick delivers what it held whatever its
/// pressure. The zero slope leaves the node's diagonal to its branches, so a
/// starved tank is conditioned exactly like a junction.
///
/// Lives HERE, in the shared file, rather than in either solver: it is the same
/// term in the same residual, and both fidelities must inherit it from one
/// definition or I5 stops meaning anything. Newton adds it to `R_i` and to the
/// Jacobian diagonal; the Simple sweep adds it to `imbalance` and to `g_sum`,
/// where `C/dt` is the diagonal preconditioning it already performs.
///
/// The sign follows the residual's convention, `R = Σ ṁ_in − Σ ṁ_out`: mass
/// accumulating in the vessel is mass that did NOT leave, so it subtracts. The
/// derivative is strictly negative, which is why a capacitive diagonal strictly
/// improves the conditioning of `J = −L` rather than merely preserving it.
#[inline]
pub fn accumulation(cap: &Capacitance, pressure: f64, dt: f64) -> (f64, f64) {
    match *cap {
        Capacitance::Vessel { c, p_prev } => (-c * (pressure - p_prev) / dt, -c / dt),
        Capacitance::Starved { supply, .. } => (supply, 0.0),
    }
}

/// The convergence test both fidelities stop on, per node:
///
/// ```text
/// |R_n|  <  tol_abs + tol_rel · scale_n
/// scale_n = max over node n's incident ACTIVE edges of |ṁ_e|   [kg/s]
/// ```
///
/// **The scale is the node's own traffic, not the network's** (M9.2, DESIGN §11).
/// Until M9.2 both solvers compared `‖R‖_∞` against `tol_abs + tol_rel·max_e|ṁ_e|`
/// over the WHOLE graph, so the error budget granted to any one node was set by
/// the largest pipe anywhere in the plant — a quantity with no relation to the
/// equation being graded. DESIGN §3 has specified "relative mass-imbalance per
/// node" since M1; this is the code catching up with it.
///
/// `scale_n ≤ throughput` for every node, so this criterion is **never looser**
/// than the one it replaces, at any node of any plant. The rejected alternative
/// is `Σ_e |ṁ_e|` — the textbook scaled residual — which is looser at any node
/// with more than one live edge and would have let some plants stop earlier than
/// they do today.
///
/// **A vessel's accumulation is deliberately NOT in the scale**, which sharpens
/// the exclusion `assemble` already documents for `throughput`: it is measured
/// against the network's mass flow, not added to it. Including `|−C·ΔP/dt|` would
/// hand `relief_blowdown` a bar twice as loose on the one node whose convergence
/// is driven by that very term.
///
/// The consequence to know: on a dead leg every incident flow is ~0, so the bar
/// collapses to `tol_abs`. That is the intended tightening — a shut branch is
/// exactly where the old rule was slackest.
///
/// **It is very nearly free, measured rather than predicted.** Across 6 000 ticks
/// of all fourteen shipped scenarios the worst iteration count per tick moves on
/// ONE plant, `tank_level_control`, from 3 to 4, and nowhere else; twelve of the
/// fourteen stay byte-identical. Nor does it push any generated plant into `Err`:
/// the reachability harnesses in `tests/invariants.rs` report identical
/// convergence counts either side of the change (chains 238/300, gas 202/205
/// Newton and 187/205 Simple, psv chains 196/400).
///
/// Returns `(‖R‖_∞ [kg/s], converged)`. The reported residual is unchanged: it is
/// still the worst absolute node imbalance, because that is what a diagnostic in
/// kg/s should say.
pub fn grade_nodes(
    residual_and_scale: impl IntoIterator<Item = (f64, f64)>,
    tol_abs: f64,
    tol_rel: f64,
) -> (f64, bool) {
    let mut worst = 0.0f64;
    let mut converged = true;
    for (residual, scale) in residual_and_scale {
        worst = worst.max(residual.abs());
        converged = converged && meets_node_bar(residual, scale, tol_abs, tol_rel);
    }
    (worst, converged)
}

/// One node's half of `grade_nodes`: does a residual `R_n` [kg/s] at a node whose
/// own traffic is `scale_n` [kg/s] meet the bar? Split out so that the game
/// fidelity's group correction (M21.1, DESIGN §25 fork 3) skips a group on
/// literally the test the solve stops on, rather than on a copy of it that could
/// drift — "no new constant" is only true if it is the same code.
pub fn meets_node_bar(residual: f64, scale: f64, tol_abs: f64, tol_rel: f64) -> bool {
    residual.abs() < tol_abs + tol_rel * scale
}

/// Compile one edge into its series branch. The device (if any) at the edge's
/// SOURCE node folds into this outlet edge, per the fold-at-source convention.
///
/// **The transport density is evaluated at the UPWIND node's STATE** — its
/// pressure iterate and its temperature (docs/DESIGN.md §3a). For a liquid this
/// is inert — `density_at` ignores both
/// arguments, so every pre-M5.2 network compiles to bit-identical numbers — but
/// for gas ρ = P·M̄/(R·T) and `P` is the unknown the solve is looking for.
/// Recompiling each iteration at the current iterate is frozen-coefficient
/// Newton: dρ/dP is omitted from the Jacobian, so convergence near the root is
/// linear rather than quadratic, but the fixed point reached is the *consistent*
/// one, which a previous-tick density would not be.
///
/// **Upwind is the higher-pressure endpoint**, and the convention is
/// well-conditioned rather than merely convenient: when the two endpoint
/// pressures are close enough for the flow DIRECTION to be in doubt, the two
/// candidate densities are correspondingly close, so the choice cannot matter
/// much exactly where it is hardest to make. It is exact whenever the branch's
/// `beta` is small against its drop — true for every gas line, where elevation
/// head ρ·g·Δz is ~1e-3 of a liquid's.
/// The stated exception is a PUMP folded into a gas edge, whose β = ρ·g·h0 can
/// make the higher-pressure endpoint the downstream one. No M5 plant has one: a
/// compressor is not a pump, and ΔP = ρ·g·H is the wrong law for a fluid whose
/// density changes through the machine. It is not guarded, because a guard no
/// scenario in this repo can exercise is a guard that cannot be falsified — it
/// un-defers with the first gas plant carrying a machine.
///
/// **The TEMPERATURE is the upwind node's too, not the pipe's stored one**, and
/// the distinction is not cosmetic. `pipe.stream.temperature` is the edge's
/// OUTLET (`graph::Pipe::stream`) — its inlet transformed by whatever heat the
/// pipe traded with ambient AND by its own frictional dissipation. In gas service
/// that last term is not a footnote: expanding an ideal gas across a branch
/// dissipates `Δp/ρ` per kilogram, i.e. `ΔT/T = (γ−1)/γ`, ~20% for a light gas at
/// any pressure ratio worth simulating. Reading it would compile the density of
/// the gas that has already been through the pipe rather than the gas entering
/// it, and — the reason this could not be left as an accepted lag — that offset
/// contains no `dt`. It does not shrink as the step shrinks, so it is a different
/// steady model, not a staleness.
///
/// Composition needs no such correction: `Engine::tick` step 3b writes each
/// stream's composition as its upwind node's, unchanged, because a pipe trades
/// heat and never mass. So the stored copy already IS the upwind value, one tick
/// stale — the same structural staleness §3 accepts for tank levels feeding the
/// quasi-steady solve, and unavoidable here because the solve opens the tick
/// before any node state is resolved.
///
/// **The zero-volume upwind node was the limitation this function used to state,
/// and M5.4 un-defers it** (docs/DESIGN.md §3a fork 6).
/// `energy::boundary_temperature` answers only for a node that HAS a temperature
/// of its own — Source, Sink, Atmosphere, Tank, and the capacitive vessel. A
/// zero-volume upwind node (junction, valve, pump, exchanger side) has none: its
/// temperature is the sweep's mix, which does not exist when the solve runs and
/// is not stored on the graph. Such an edge used to fall back to the pipe's
/// stored OUTLET temperature, which on `gas_line.toml` at steady state compiled
/// the relief line at its own outlet's **375.0 K** instead of the tee's
/// **297.3 K** — a 21% density error, and one containing no `dt`, so it was a
/// different steady model rather than a staleness.
///
/// It un-deferred here because `ρ₁` enters the ISA gas sizing equation under a
/// square root: 21% on ρ is ~10% on ṁ, and an ISA reference gate cannot be an
/// independent published anchor while the density it reads is 21% wrong.
///
/// The order of preference is therefore: the node's OWN temperature where it has
/// one (current, not lagged); else the PREVIOUS tick's resolved value from the
/// sweep (an honest staleness — it shrinks with `dt`); else the stored outlet,
/// which is now reachable only on tick 0, before any sweep has run.
pub fn compile_edge(
    graph: &PlantGraph,
    eid: EdgeId,
    slate: &Slate,
    previous_states: &NodeStates,
    pressures: &BTreeMap<NodeId, f64>,
) -> Result<CompiledEdge, SimError> {
    let (src, tgt) = graph.endpoints(eid);
    let pipe = graph.pipe(eid);
    // A BOIL-OFF VENT compiles to a closed branch and returns before anything
    // else is read — above the density lookup on purpose, because a vent has no
    // hydraulics at all and must not be able to fail on a property it does not
    // use. `alpha = +∞` is what `conducts` already reads as closed, so the vent
    // contributes nothing to any node's residual, nothing to the throughput and
    // nothing to the per-node scale `grade_nodes` stops on. Its flow is written
    // by `Engine::tick` from the holdup's enthalpy balance (docs/DESIGN.md §14).
    // A tank's OVERFLOW is the same kind of edge (M23, §27 fork 3): its flow is
    // whatever stood above the brim at the end of the tick, written by the engine.
    if pipe.leak.is_engine_written() {
        return Ok(CompiledEdge {
            src,
            tgt,
            branch: QuadraticBranch {
                alpha: f64::INFINITY,
                beta: 0.0,
            },
            rho: 1.0,
            conducts: false,
            relief_opening_log_slope: 0.0,
            check_opening_log_slope: 0.0,
        });
    }
    let upwind_node = if pressures[&src] >= pressures[&tgt] {
        src
    } else {
        tgt
    };
    let upwind = pressures[&upwind_node].max(RHO_EVAL_P_FLOOR);
    let temperature = boundary_temperature(&graph.node(upwind_node).kind)
        .or_else(|| previous_states.temperature.get(&upwind_node).copied())
        .unwrap_or(pipe.stream.temperature);
    let rho = pipe
        .stream
        .composition
        .density_at(slate, Pascal(upwind), temperature)
        .map_err(|e| SimError::Numerical(format!("pipe {} ({eid:?}): {e}", pipe.name)))?
        .value();

    // A LEAK ORIFICE compiles to its own characteristic and nothing else, and
    // returns here rather than falling through: it has no pipe geometry to
    // resist with, no elevation to offset by, and no device at its source to
    // fold in (its source is the junction the loader split the pipe at). The
    // early return is what makes `beta = 0` structural — see
    // `QuadraticBranch::orifice` for what depends on that, and
    // `finalize`'s back-feed refusal for who.
    if let LeakRole::Orifice { area } = pipe.leak {
        let mut branch = QuadraticBranch::orifice(area.value(), ORIFICE_CD, rho);
        // In GAS service the hole is the isentropic nozzle law (M37,
        // docs/DESIGN.md §41): a hole venting a pressurised gas line to
        // atmosphere chokes above a drop of about half its absolute pressure, and
        // Torricelli there would overpredict exactly the number a leak exists to
        // report — the reason M6 refused it, here and at load, until now. The
        // branch keeps `beta = 0` and the sign of `dp`, so the back-feed refusal
        // below is unchanged; a dormant hole is returned closed before `γ` is
        // read.
        if pipe.stream.composition.phase(slate)? == Phase::Gas {
            let comp = &pipe.stream.composition;
            let gamma = comp.mixture_cp(slate).value() / comp.mixture_cv(slate).value();
            if !gamma.is_finite() || gamma <= 1.0 {
                return Err(SimError::Numerical(format!(
                    "leak orifice '{}' ({eid:?}) carries a gas whose cp/cv = {gamma:.4} \
                     is not above 1, so it has no choke to size the hole by",
                    pipe.name
                )));
            }
            let dp = pressures[&src] - pressures[&tgt];
            branch = gas_orifice(branch, dp, upwind, gamma);
            if branch.alpha.is_nan() || branch.alpha <= 0.0 {
                return Err(SimError::Numerical(format!(
                    "leak orifice '{}' ({eid:?}) compiled to a resistance of {:e} in gas \
                     service (γ = {gamma:.4}, upstream {upwind:e} Pa)",
                    pipe.name, branch.alpha
                )));
            }
        }
        debug_assert_eq!(branch.beta, 0.0, "an orifice branch must carry no offset");
        let conducts = branch.alpha.is_finite() && branch.alpha > 0.0;
        return Ok(CompiledEdge {
            src,
            tgt,
            branch,
            rho,
            conducts,
            relief_opening_log_slope: 0.0,
            check_opening_log_slope: 0.0,
        });
    }

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
    let dp = pressures[&src] - pressures[&tgt];
    let mut branch = QuadraticBranch::pipe(k, elev_head);
    let pipe_alpha = branch.alpha;
    // A relief valve's `(snapped opening, d opening / d P_src)`, for the log slope
    // below. Stays `None` on every other edge.
    let mut relief: Option<(f64, f64)> = None;
    // A check valve's `(snapped opening, d opening / dS)`, likewise.
    let mut check: Option<(f64, f64)> = None;

    match &graph.node(src).kind {
        NodeKind::Pump { h0, a, on } => {
            let h0_eff = if *on { h0.value() } else { 0.0 };
            branch = branch.in_series(QuadraticBranch::pump(h0_eff, *a, rho, G));
        }
        // A relief valve IS a valve here — same coefficient, same ISA gas law,
        // same fold — and differs only in where `opening` comes from: the plant
        // state rather than an operator setpoint (DESIGN §3a fork 5). Sharing one
        // arm is what guarantees the two cannot drift apart in the element
        // physics, which is the same reason `Cooler` shares `heat_load` with
        // `Furnace` rather than owning a second copy of it.
        //
        // **The pressure the SPRING senses is not the pressure the GAS LAW uses**,
        // and the two are deliberately different reads. A PSV's spring is loaded
        // by the pressure at its own inlet flange — the node's own pressure,
        // whichever way the flow happens to run — while `x` needs the upwind
        // THERMODYNAMIC state. They coincide whenever the valve is relieving,
        // which is the only regime it is designed for; they differ under the
        // reverse flow this fidelity does not refuse.
        kind @ (NodeKind::Valve { .. } | NodeKind::ReliefValve { .. }) => {
            let (cv_max, opening, x_t) = match kind {
                NodeKind::Valve {
                    cv_max,
                    opening,
                    x_t,
                } => (cv_max, *opening, x_t),
                // A LIFTED pop valve is at full lift whatever its inlet reads,
                // and its opening has no slope (M48, docs/DESIGN.md §53 fork 2);
                // the latch itself moves only between ticks.
                NodeKind::ReliefValve {
                    blowdown: Some(Blowdown { lifted: true, .. }),
                    cv_max,
                    x_t,
                    ..
                } => (cv_max, 1.0, x_t),
                NodeKind::ReliefValve {
                    cv_max,
                    set_pressure,
                    accumulation,
                    x_t,
                    ..
                } => (
                    cv_max,
                    relief_opening(pressures[&src], set_pressure.value(), accumulation.value()),
                    x_t,
                ),
                _ => unreachable!("outer pattern admits only the two valve kinds"),
            };
            let opening = &opening;
            let op = if *opening < OPEN_EPS { 0.0 } else { *opening };
            match kind {
                NodeKind::ReliefValve {
                    blowdown: Some(Blowdown { lifted: true, .. }),
                    ..
                } => relief = Some((op, 0.0)),
                NodeKind::ReliefValve {
                    set_pressure,
                    accumulation,
                    ..
                } => {
                    relief = Some((
                        op,
                        relief_opening_slope(
                            pressures[&src],
                            set_pressure.value(),
                            accumulation.value(),
                        ),
                    ));
                }
                _ => {}
            }
            let rho_rel = rho / RHO_WATER_REF;
            let liquid = QuadraticBranch::valve(*cv_max, op, rho_rel);
            branch = fold_gas_service(graph, src, pipe, slate, branch, liquid, *x_t, dp, upwind)?;
        }
        // A check valve is a valve whose opening is read off the FORWARD DRIVE
        // across this branch, `S = dp − β` with `β` the outlet pipe's static head
        // (M30, docs/DESIGN.md §33). `flow` has the sign of `S`, so "shut at
        // `S ≤ 0`" is exactly "would run backwards": reverse flow is zero by
        // `α = +∞`, not by a threshold.
        //
        // In gas service the valve that opening sets is folded exactly as a
        // `Valve`'s is (M31, §34): the opening comes first, from the drive, and
        // the fold splits the drive between valve and pipe afterwards, so nothing
        // is circular. The disc still reads the whole branch's drive rather than
        // the valve's own share the fold computes (row E26, gas or liquid).
        NodeKind::CheckValve {
            cv_max,
            full_open,
            x_t,
        } => {
            let drive = dp - branch.beta;
            let opening = check_opening(drive, full_open.value());
            let op = if opening < OPEN_EPS { 0.0 } else { opening };
            check = Some((op, check_opening_slope(drive, full_open.value())));
            let rho_rel = rho / RHO_WATER_REF;
            let liquid = QuadraticBranch::valve(*cv_max, op, rho_rel);
            branch = fold_gas_service(graph, src, pipe, slate, branch, liquid, *x_t, dp, upwind)?;
        }
        _ => {}
    }

    let conducts = branch.alpha.is_finite() && branch.alpha > 0.0;
    let relief_opening_log_slope = match relief {
        Some((op, slope)) => opening_log_slope(graph, src, pipe_alpha, branch.alpha, op, slope)?,
        None => 0.0,
    };
    let check_opening_log_slope = match check {
        Some((op, slope)) => opening_log_slope(graph, src, pipe_alpha, branch.alpha, op, slope)?,
        None => 0.0,
    };
    Ok(CompiledEdge {
        src,
        tgt,
        branch,
        rho,
        conducts,
        relief_opening_log_slope,
        check_opening_log_slope,
    })
}

/// Compose a valve of any kind with the pipe it discharges through: in closed
/// form in liquid service, through the ISA compressible fold in gas service.
///
/// **One owner for all three valve kinds** (M31, docs/DESIGN.md §34). The
/// relief valve shares `Valve`'s arm for exactly this reason — two copies of the
/// element physics can drift apart — and the check valve, which has its own arm
/// because its opening reads a different quantity, reaches the same law here
/// rather than through a copy of it.
///
/// `x_t = Some`: gas service. `p_up` is the UPWIND node's pressure, deliberately
/// not `src`'s — the fold-at-source convention puts `src` at the valve, and it
/// is the valve's inlet only while the flow runs forward. In reverse the inlet is
/// `tgt`, and `x` is taken from `|dp − β|` against whichever end is upwind so the
/// branch stays odd about `β` (DESIGN §3a fork 6). `ρ` needs no such care: it is
/// already the upwind value by construction.
///
/// `x_t = None`: for a LIQUID stream that is correct and the branch is
/// bit-identical to pre-M5.4. For a GAS stream it is the silent wrong number
/// M5.4 exists to refuse — the incompressible law on a compressible fluid,
/// finite, deterministic, mass-conserving and overpredicting exactly where a
/// relief is read. The loader already refuses that pairing
/// (`require_gas_valve_x_t`, off M5.2's topological analysis), so this is a
/// SECOND door on the same correspondence — and it exists because the loader is
/// not the only way in: the invariant proptests build a `PlantGraph` directly
/// and never call `build_engine`.
#[allow(clippy::too_many_arguments)]
fn fold_gas_service(
    graph: &PlantGraph,
    src: NodeId,
    pipe: &Pipe,
    slate: &Slate,
    branch: QuadraticBranch,
    liquid: QuadraticBranch,
    x_t: Option<f64>,
    dp: f64,
    upwind: f64,
) -> Result<QuadraticBranch, SimError> {
    match x_t {
        Some(x_t) => {
            let comp = &pipe.stream.composition;
            let gamma = comp.mixture_cp(slate).value() / comp.mixture_cv(slate).value();
            let x_choke = specific_heat_ratio_factor(gamma) * x_t;
            if !x_choke.is_finite() || x_choke <= 0.0 {
                return Err(SimError::Numerical(format!(
                    "valve '{}' has a non-positive critical pressure-drop ratio \
                     F_k·x_T = {x_choke:.3e} (γ = {gamma:.4}, x_T = {x_t})",
                    graph.node(src).name
                )));
            }
            Ok(fold_gas_valve(
                branch,
                liquid,
                dp,
                upwind,
                x_choke,
                CHOKE_BLEND,
            ))
        }
        None => {
            if pipe.stream.composition.phase(slate)? == Phase::Gas {
                return Err(SimError::Numerical(format!(
                    "valve '{}' carries a gas-phase stream but has no x_T, so the \
                     incompressible sizing law would be applied to a compressible \
                     fluid (docs/DESIGN.md §3a fork 4)",
                    graph.node(src).name
                )));
            }
            Ok(branch.in_series(liquid))
        }
    }
}

/// A pressure-actuated valve's `d ln ṁ` through its opening [1/Pa], per pascal of
/// whatever the opening reads: `P_src` for a relief valve (M26, docs/DESIGN.md
/// §30), the branch's forward drive for a check valve (M30, §33).
///
/// `ṁ ∝ α_tot^(−½)` with `α_tot = α_pipe + α_v` and `α_v ∝ op^(−2)`, so
/// `∂ln ṁ/∂op = (α_v/α_tot)/op`, times the opening's own slope `dop/dP_src`.
/// Exact for a liquid valve, which composes in closed form. For a gas valve
/// `α_v` is the fold's effective resistance `α_v·(x/x_s)/Y²`, and holding that
/// ratio fixed while the opening moves is an approximation — 1.208e-5 against a
/// centred difference's 1.214e-5 at the twin plant's lift. It changes how fast
/// Newton converges and never where: the line search and the stopping rule both
/// read the true residual.
///
/// `0.0` outside the band (the slope is exactly zero there) and when the opening
/// snapped shut, where the expression is `0·∞/0`. A non-finite result anywhere
/// else is an `Err` rather than a quiet zero (rule 5): a NaN here would reach
/// the LU and be reported as a singular Jacobian, which is the wrong diagnosis.
fn opening_log_slope(
    graph: &PlantGraph,
    src: NodeId,
    pipe_alpha: f64,
    total_alpha: f64,
    opening: f64,
    opening_slope: f64,
) -> Result<f64, SimError> {
    if opening == 0.0 || opening_slope == 0.0 {
        return Ok(0.0);
    }
    // Ohm's law for resistances in series: the valve's share of the total.
    let log_slope = (total_alpha - pipe_alpha) / total_alpha / opening * opening_slope;
    if !log_slope.is_finite() {
        return Err(SimError::Numerical(format!(
            "valve '{}' has a non-finite opening slope d ln ṁ/dP = {log_slope:e} \
             (opening {opening:e}, d opening/dP = {opening_slope:e} 1/Pa, \
             α_pipe = {pipe_alpha:e}, α_total = {total_alpha:e})",
            graph.node(src).name
        )));
    }
    Ok(log_slope)
}

/// Compile every edge's series branch (pipe ∘ device-at-source), keyed by edge,
/// at the given pressure iterate. Called once per solver iteration since M5.2 —
/// see `compile_edge` for why, and why an all-liquid network is unaffected.
pub fn compile_edges(
    graph: &PlantGraph,
    slate: &Slate,
    previous_states: &NodeStates,
    pressures: &BTreeMap<NodeId, f64>,
) -> Result<BTreeMap<EdgeId, CompiledEdge>, SimError> {
    let mut compiled = BTreeMap::new();
    for eid in graph.edge_ids() {
        compiled.insert(
            eid,
            compile_edge(graph, eid, slate, previous_states, pressures)?,
        );
    }
    Ok(compiled)
}

/// The shared solve prologue: classify, seed, compile, anchor, pin floating —
/// in the one order that is self-consistent, and identical for both fidelities.
///
/// The anchored set comes from the SEED compile, which is exact for every
/// element whose `conducts` is constant through a solve and stale for one whose
/// opening the answer decides. `solve_with_active_anchoring` is what corrects
/// that, by re-running this with an override (M8.0, DESIGN §3c); a direct call
/// here is still the first pass and still classifies from the seed.
pub fn prepare(
    graph: &PlantGraph,
    slate: &Slate,
    previous_states: &NodeStates,
    warm_start: &BTreeMap<NodeId, f64>,
) -> Result<Prepared, SimError> {
    prepare_anchored(
        graph,
        slate,
        previous_states,
        warm_start,
        None,
        None,
        &BTreeMap::new(),
    )
}

/// `prepare`, with the anchored set optionally SUPPLIED rather than derived.
///
/// `Some(set)` is how a later active-set pass runs under the classification the
/// previous pass's ANSWER implies instead of the one its own seed implies. The
/// floating pins below follow the override, so a node that has just become
/// anchored stops being parked and one that has just stopped being anchored
/// starts (DESIGN §3c).
///
/// `previous_pass` carries that pass's pressures, and it feeds the free-node
/// SEED only — never the floating pin. The two look like one thing while there
/// is a single pass and are not: seeding an unknown is a path to an answer, so
/// the nearest available start wins, while a floating node's value is what gets
/// REPORTED for a pressure the plant does not determine. A mid-solve iterate is
/// a fine path and a meaningless report, so the pin keeps reading `warm_start` —
/// the value the node last had while it was determinate, else atmospheric, which
/// is the pre-M8.0 convention untouched.
///
/// `starved` maps each tank this pass solves as STARVED to its supply [kg/s]
/// (`starved_supply`). It is empty on every pass of a tick that starves nothing,
/// and then this is exactly the pre-M24 prologue (docs/DESIGN.md §28 fork 3).
pub fn prepare_anchored(
    graph: &PlantGraph,
    slate: &Slate,
    previous_states: &NodeStates,
    warm_start: &BTreeMap<NodeId, f64>,
    previous_pass: Option<&BTreeMap<NodeId, f64>>,
    anchored_override: Option<&BTreeSet<NodeId>>,
    starved: &BTreeMap<NodeId, f64>,
) -> Result<Prepared, SimError> {
    let classes = classify(graph, slate, starved);

    // Seed every free node before compiling, because a gas edge's density is
    // evaluated at a node pressure. Floating nodes are re-pinned below, once
    // there is an anchored set to tell them apart; the intermediate value
    // reaches only `conducts`, which is sign-of-alpha and cannot differ, and the
    // edges it reaches are inert either way.
    let mut pressures = classes.fixed.clone();
    for &nid in &classes.free {
        // A capacitive node falls back to its OWN `Pⁿ`, never to the cold mean:
        // a 10 bar vessel started at the mean of a 10 bar header and a 0 bar
        // flare would begin its first step somewhere its own mass never was.
        //
        // The warm start still wins where there is one, and that ordering is
        // MEASURED rather than reasoned. `Pⁿ` looks like the more principled
        // seed — it is a fact about the inventory rather than a guess — but the
        // two are not competing on principle: the seed cannot move the answer at
        // all, because `Pⁿ` reaches the residual through `Capacitance`
        // independently of where the iterate starts. It is purely a path, so the
        // question is only which path is shorter, and the previous tick's `P*`
        // is nearer this tick's than `Pⁿ` is — they differ by exactly the
        // accumulation the solve is about to add back. Measured on
        // `knockout_drum` over 200 ticks: 172 Newton iterations for the rule
        // below, 277 for `Pⁿ` unconditionally, 168 for a cold-mean fallback that
        // happens to suit this plant (its two reservoirs bracket the answer) and
        // would not suit a lone vessel. A blowdown to vacuum is 400 under all
        // three. Nothing in the suite can tell any of them apart on the ANSWER,
        // which is the honest statement of what this line is worth.
        let seed = previous_pass
            .and_then(|p| p.get(&nid))
            .or_else(|| warm_start.get(&nid))
            .copied()
            .unwrap_or_else(|| match classes.capacitive.get(&nid) {
                Some(Capacitance::Vessel { p_prev, .. }) => *p_prev,
                // Reachable only when a starved pass has no previous pass to
                // seed from, which the driver never runs: the pass that starves
                // a tank always follows the pass that found it over-drawn. Its
                // pinned pressure is the nearest thing it has to a history.
                Some(Capacitance::Starved { p_pinned, .. }) => *p_pinned,
                None => classes.cold,
            });
        pressures.insert(nid, seed);
    }

    let compiled = compile_edges(graph, slate, previous_states, &pressures)?;
    let anchored = match anchored_override {
        Some(a) => a.clone(),
        None => anchored_set(graph, &compiled, &base_anchors(&classes)),
    };

    // A floating free node's pressure is indeterminate and its edges are inert,
    // so it is parked at its warm-start value or at P_ATM rather than at the
    // cold seed, which means nothing for it.
    for &nid in &classes.free {
        if !anchored.contains(&nid) {
            let seed = warm_start.get(&nid).copied().unwrap_or(P_ATM.value());
            pressures.insert(nid, seed);
        }
    }

    Ok(Prepared {
        classes,
        compiled,
        anchored,
        pressures,
    })
}

/// Classify nodes into fixed/free and compute the cold-start seed.
/// Deterministic: node ids iterate ascending, `free` is ascending, and the seed
/// is a pure function of the pinned pressures.
///
/// A tank in `starved` (mapped to its supply [kg/s]) is FREE, with that supply
/// as its own term, instead of pinned (docs/DESIGN.md §28 fork 2). Any other key
/// is ignored: only a tank can starve, and the driver only ever names tanks.
pub fn classify(
    graph: &PlantGraph,
    slate: &Slate,
    starved: &BTreeMap<NodeId, f64>,
) -> Classification {
    let mut fixed: BTreeMap<NodeId, f64> = BTreeMap::new();
    let mut free: Vec<NodeId> = Vec::new();
    let mut capacitive: BTreeMap<NodeId, Capacitance> = BTreeMap::new();
    let (mut fixed_sum, mut fixed_cnt) = (0.0, 0usize);
    for nid in graph.node_ids() {
        let starved_tank = match (&graph.node(nid).kind, starved.get(&nid)) {
            (NodeKind::Tank(tank), Some(&supply)) => Some(Capacitance::Starved {
                supply,
                p_pinned: tank.bottom_pressure(slate).value(),
            }),
            _ => None,
        };
        if let Some(cap) = starved_tank {
            free.push(nid);
            capacitive.insert(nid, cap);
        } else if let Some(p) = fixed_pressure(graph.node(nid), slate) {
            fixed.insert(nid, p);
            fixed_sum += p;
            fixed_cnt += 1;
        } else {
            free.push(nid);
            if let Some(cap) = capacitance(graph.node(nid), slate) {
                capacitive.insert(nid, cap);
            }
        }
    }
    // Cold start for nodes without a warm-start value (uniqueness makes the
    // seed affect only the iterate path, never the answer).
    let cold = if fixed_cnt > 0 {
        fixed_sum / fixed_cnt as f64
    } else {
        P_ATM.value()
    };
    Classification {
        fixed,
        free,
        capacitive,
        cold,
    }
}

/// The nodes that anchor a pressure with no reference to anything outside
/// themselves: the pinned ones AND the capacitive vessels. A vessel's own
/// equation determines its pressure, so it needs no conducting path to a
/// reservoir (DESIGN §3a fork 2) — which is what makes a closed gas system well
/// posed.
///
/// **A starved tank is NOT an anchor** (M24, docs/DESIGN.md §28 fork 2). Its term
/// is a constant supply, so its equation says nothing about its own pressure; it
/// is anchored only if a conducting path reaches something that is. Counting it
/// here would hand a dry tank with nowhere to send its supply an equation with no
/// solution.
///
/// Its own function because the active-set loop recomputes `anchored_set` from a
/// later iterate and must hand it the SAME anchors the seed pass used. Deriving
/// them twice from two copies of this expression is how the two would drift.
pub fn base_anchors(classes: &Classification) -> BTreeSet<NodeId> {
    let mut anchors: BTreeSet<NodeId> = classes.fixed.keys().copied().collect();
    anchors.extend(
        classes
            .capacitive
            .iter()
            .filter(|(_, cap)| matches!(cap, Capacitance::Vessel { .. }))
            .map(|(nid, _)| *nid),
    );
    anchors
}

/// Nodes reachable from any ANCHOR through conducting edges (undirected).
/// Free nodes NOT in this set are floating (indeterminate pressure).
///
/// `anchors` is the pinned nodes plus the capacitive ones. Capacitance anchors
/// because a vessel's residual determines its own pressure with no reference to
/// anything outside it — the `C·(P − Pⁿ)/dt` term is an equation in `P` alone —
/// so "reachable from a pressure reference" is no longer the same question as
/// "reachable from a FIXED node" (DESIGN §3a fork 2).
pub fn anchored_set(
    graph: &PlantGraph,
    compiled: &BTreeMap<EdgeId, CompiledEdge>,
    anchors: &BTreeSet<NodeId>,
) -> BTreeSet<NodeId> {
    let mut adj: BTreeMap<NodeId, Vec<NodeId>> = BTreeMap::new();
    for eid in graph.edge_ids() {
        let c = &compiled[&eid];
        if c.conducts {
            adj.entry(c.src).or_default().push(c.tgt);
            adj.entry(c.tgt).or_default().push(c.src);
        }
    }
    let mut anchored = anchors.clone();
    let mut stack: Vec<NodeId> = anchors.iter().copied().collect();
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

/// How many classifications one solve may run through before the loop gives up
/// on the plant settling. A cycle is caught by REPETITION rather than by this cap
/// (see `solve_with_active_anchoring`), so this bounds only the strictly *growing*
/// case — a chain of never-before-seen sets, which in practice is bounded by the
/// number of pressure-actuated elements in the plant.
pub const MAX_ANCHOR_PASSES: usize = 8;

/// How far [Pa] a dead end may still move between two re-stands and count as
/// standing (M47, docs/DESIGN.md §52). At zero flow both solvers' branches are
/// regularised linear over `eps_dp = 1 Pa`, so a stretch this far off carries
/// about 1e-9 kg/s on A22's chain, two orders under the agreement gate's root
/// bound of 1e-7 kg/s.
const DEAD_END_STANDING_TOL: f64 = 1e-6;

/// How many times a dead end is re-stood from its own recompile before the tie
/// gives up on it (M47, §52). A gas column converges in two or three; the cap
/// only bounds a column that would not.
const MAX_DEAD_END_RESTANDS: usize = 8;

/// What one fidelity's own iteration returns to the active-set driver: its
/// result, and the pressure iterate it ended on.
///
/// The iterate is returned **whether or not the pass converged**, and that is the
/// point of the type. The plant this loop exists for is the one whose pass fails
/// — a dead leg behind a relief that shuts on the way to the answer leaves a zero
/// residual row and a singular Jacobian — and the iterate at the moment of
/// failure is exactly what reveals the relief has shut (DESIGN §3c, fork 2b).
pub struct AnchorPass {
    pub result: Result<HydraulicSolution, SimError>,
    pub pressures: BTreeMap<NodeId, f64>,
}

/// The active-set loop over the anchoring classification, shared by both
/// fidelities (M8.0, DESIGN §3c). Un-defers M5's FINDING 2.
///
/// One `pass` is a whole fidelity solve under a FIXED classification, so the
/// unknown set — and with it the dimension of Newton's Jacobian and the index set
/// its line search compares merits over — cannot move underneath it. Between
/// passes the edges are recompiled at the pressures the pass ended on and the
/// anchored set is recomputed from those; a pass whose set is already a fixed
/// point returns verbatim, which is every plant with no pressure-actuated element
/// in it, at pass one, bit for bit as before this loop existed.
///
/// **The warm start is committed HERE and nowhere else.** A pass may converge
/// under a classification the loop then rejects, and writing the warm start
/// inside the pass would leak that rejected classification's pressures into the
/// next tick — a cross-tick coupling that did not exist while the set was frozen.
/// The rule is one rule about the loop ("the final accepted pass, and only if it
/// converged"), not two rules in two solvers free to drift apart.
///
/// # The second active set: starved tanks (M24, docs/DESIGN.md §28 fork 3)
///
/// A tank is pinned at its hydrostatic bottom pressure while it holds liquid,
/// and a pinned node supplies whatever its edges draw — so a tank drawn faster
/// than it holds would deliver mass it does not have, and the engine's clamp
/// used to book that overdraw as nothing (B29: 200 149 kg created on
/// `tank_flow_control` over 30 000 ticks). Each pass is therefore classified by
/// a PAIR, (anchored set, starved set), in ONE loop with ONE pass budget:
///
/// - **Every solve starts all-wet.** No starvation state is carried between
///   ticks, so a plant on which no tank starves runs the same passes with the
///   same arithmetic as before this set existed.
/// - **Wet → starved** when a CONVERGED pass's net pressure-driven outflow from
///   the tank satisfies `q_out·dt > m`, strictly. The starved tank is then a
///   free node supplying `m/dt` (`Capacitance::Starved`).
/// - **Starved → wet** when a converged pass puts the starved tank's pressure
///   above the pressure it would pin at while wet, `P_ATM + ρgh(m)`.
/// - **A repeat that differs only in the starved set is ACCEPTED as the more
///   starved pass**, not refused. The network's net outflow from a tank falls as
///   the tank's pressure rises, so the pair has one answer, and a repeat can
///   only be the boundary itself inside the solver's tolerance —
///   `tank_overfill_trip`'s drained tank past tick 12 000 is that population. If
///   the pass the repeat surfaced on is the LESS starved one (possible with two
///   tanks), the union of the two sets is run once more with recovery frozen,
///   because the pass that cannot create mass is the one to keep. A repeat that
///   moves the anchored set is still `AnchoringUnsettled`, unchanged.
///
/// **"Anchoring settled" is judged against THIS pass's anchors**, and the next
/// pass's anchored set is built from the NEXT classification's. The two differ
/// exactly when the starved set moves, because a starved tank stops being an
/// anchor (`base_anchors`). Judging the first against the second would read a
/// plain starve-then-recover as an anchoring change.
///
/// The accepted solution carries its starved tanks (supply and own residual)
/// and each vessel's own residual, computed HERE from the accepted pass — before
/// `Engine::tick` overwrites the draw and vent flows it reports as zero — so the
/// engine's empty-holdup clamp is bounded per holdup rather than by the plant's
/// worst node (§28 fork 5).
///
/// **Reported cost understates a starved tick**: `SolveDiagnostics` is the
/// accepted pass's, so the wet pass that found the overdraw is not counted.
pub fn solve_with_active_anchoring<F>(
    graph: &PlantGraph,
    slate: &Slate,
    previous_states: &NodeStates,
    warm_start: &mut BTreeMap<NodeId, f64>,
    dt: Seconds,
    mut pass: F,
) -> Result<HydraulicSolution, SimError>
where
    F: FnMut(Prepared) -> AnchorPass,
{
    let mut previous_pass: Option<BTreeMap<NodeId, f64>> = None;
    let mut override_set: Option<BTreeSet<NodeId>> = None;
    let mut starved: BTreeSet<NodeId> = BTreeSet::new();
    // Set once a starvation-only repeat surfaced on the less starved of its two
    // passes: from then on a tank may starve but not recover, so the loop can
    // only grow toward the pass that cannot create mass.
    let mut recovery_frozen = false;
    // Every classification this solve has run, in order. A REPEAT onto one that
    // has converged is a cycle and is terminal: the plant has two
    // self-consistent answers, and picking the later one would be choosing
    // between them by iteration parity.
    let mut seen: Vec<(BTreeSet<NodeId>, BTreeSet<NodeId>)> = Vec::new();
    // The classifications whose pass CONVERGED. Two answers need two converged
    // passes: a repeat onto a classification only ever run as a failed pass is
    // re-run once rather than refused (M46, docs/DESIGN.md §51).
    let mut converged: Vec<(BTreeSet<NodeId>, BTreeSet<NodeId>)> = Vec::new();
    // The refusal such a re-run postponed. A re-run that fails returns it
    // unchanged, so a plant the re-run cannot answer is refused exactly as
    // before; one that converges joins `converged`, so it is never re-run twice.
    let mut postponed: Option<SimError> = None;

    for _ in 0..MAX_ANCHOR_PASSES {
        let supplies = starved_supplies(graph, &starved, dt);
        let prep = prepare_anchored(
            graph,
            slate,
            previous_states,
            warm_start,
            previous_pass.as_ref(),
            override_set.as_ref(),
            &supplies,
        )?;
        // Taken before the pass consumes `prep`: the free list for the warm-start
        // commit, the anchors so the reclassification below runs against the
        // SAME base set this pass was built from, and the own terms for the
        // holdups' residuals.
        let used = prep.anchored.clone();
        let free = prep.classes.free.clone();
        let anchors = base_anchors(&prep.classes);
        let capacitive = prep.classes.capacitive.clone();
        // What conducted in the compile this pass's classification came from:
        // a dead end's tie is judged in the classification that fills it, and
        // on Newton's road to the repeat that is this one (M45.0, §50).
        let conducting: BTreeSet<EdgeId> = prep
            .compiled
            .iter()
            .filter(|(_, c)| c.conducts)
            .map(|(&eid, _)| eid)
            .collect();
        seen.push((used.clone(), starved.clone()));

        let AnchorPass {
            mut result,
            mut pressures,
        } = pass(prep);

        // A re-run gets this one pass; failing, it is refused as it was before
        // M46, not walked on toward the cap and a different refusal.
        if let Some(refusal) = postponed.take() {
            if result.is_err() {
                return Err(refusal);
            }
        }
        if result.is_ok() {
            converged.push((used.clone(), starved.clone()));
        }

        // A pass that failed BECAUSE its iterate went non-finite is the one case
        // the iterate cannot be reclassified from: the resulting set would be an
        // artefact of NaN, and an arbitrary retry is worse than an honest
        // failure. Simple has exactly such a path; Newton's two give-up paths
        // discard the bad trial and keep a finite incumbent.
        if pressures.values().any(|p| !p.is_finite()) {
            return result;
        }
        let compiled = match compile_edges(graph, slate, previous_states, &pressures) {
            Ok(c) => c,
            // The pass already compiled at these pressures, so this is reachable
            // only from a diverged iterate — whose own error is the better
            // answer, and is what `and` keeps.
            Err(e) => return result.and(Err(e)),
        };
        let next = anchored_set(graph, &compiled, &anchors);
        // Starvation, like anchoring, is reclassified only from a pass that
        // CONVERGED: a failed pass ended on a mid-flight iterate, and the flows
        // it implies are not an answer to test an inventory against.
        let mut next_starved = match &result {
            Ok(solution) => {
                reclassify_starved(graph, slate, solution, &starved, dt, recovery_frozen)
            }
            Err(_) => starved.clone(),
        };

        // The warm start is committed once, from the pass that is kept, and only
        // if it converged — the pre-M24 rule, now with one more reason a pass
        // can be discarded.
        let accept = |result: &mut Result<HydraulicSolution, SimError>,
                      warm_start: &mut BTreeMap<NodeId, f64>,
                      pressures: &BTreeMap<NodeId, f64>| {
            if let Ok(solution) = result {
                for &nid in &free {
                    if let Some(&p) = pressures.get(&nid) {
                        warm_start.insert(nid, p);
                    }
                }
                report_holdups(graph, solution, &supplies, &capacitive, pressures, dt);
            }
        };

        if next == used && next_starved == starved {
            accept(&mut result, warm_start, &pressures);
            return result;
        }
        // The anchored set the NEXT pass runs under, from the next
        // classification's own anchors. Identical to `next` whenever the starved
        // set is unchanged, which is every pass of every plant that starves
        // nothing — so that path is the pre-M24 loop to the bit.
        let mut next_override = if next_starved == starved {
            next.clone()
        } else {
            next_anchored_set(graph, slate, &compiled, &next_starved, dt)
        };
        // A repeat is a CYCLE only when the pass that produced it CONVERGED, and
        // that qualifier is a correction the code made to this slice's own design
        // note. A converged pass under `used` whose answer implies a `next` the
        // solve has already run is a genuine alternation: both states are
        // self-consistent, and choosing between them by iteration parity is the
        // thing being refused. A pass that FAILED ended on a mid-flight iterate,
        // which is not an answer — the classification it implies is a guess, and
        // repeating a guess is not evidence of anything. Those keep iterating,
        // from the new seed, with the cap as the backstop.
        //
        // Measured before the qualifier existed: 13 of 21 repeats followed a
        // failed pass, so refusing on all of them would have refused mostly on
        // guesses (ROADMAP M8.0).
        //
        // The same holds of the classification the repeat lands ON, and M46
        // added that half: one only ever run as a failed pass has not been an
        // answer either, so it is re-run once below rather than refused.
        if result.is_ok() && seen.contains(&(next_override.clone(), next_starved.clone())) {
            // A dead end's tie is not chatter (M45.0, docs/DESIGN.md §50): this
            // pass is kept, with the stretch standing where its one way in
            // leaves it at zero drive. Judged in the classification that FILLS
            // the stretch: this pass's on Newton's road, the next one's on the
            // game solver's.
            if next != used && next_starved == starved {
                let tie = if used.is_superset(&next_override) {
                    dead_end_tie(
                        graph,
                        slate,
                        previous_states,
                        &next_override,
                        &used,
                        &capacitive,
                        |eid| conducting.contains(&eid),
                        &compiled,
                        &pressures,
                    )
                } else if next_override.is_superset(&used) {
                    dead_end_tie(
                        graph,
                        slate,
                        previous_states,
                        &used,
                        &next_override,
                        &capacitive,
                        |eid| compiled[&eid].conducts,
                        &compiled,
                        &pressures,
                    )
                } else {
                    None
                };
                match tie {
                    Some(Tie::Stands(dead_end)) => {
                        if let Ok(solution) = &mut result {
                            settle_dead_end(graph, &dead_end, &mut pressures, solution);
                        }
                        accept(&mut result, warm_start, &pressures);
                        return result;
                    }
                    // A broken column is refused, never re-run (§50 fork 5): a
                    // re-run that converged would report a negative absolute
                    // pressure as an answer.
                    Some(Tie::BelowVacuum) => return Err(chatter(graph, &used, &next_override)),
                    None => {}
                }
            }
            if next != used {
                let refusal = chatter(graph, &used, &next_override);
                // Chatter is two ANSWERS. If the classification this pass points
                // back to has only ever been run as a failed pass, it has not
                // been one yet: run it again, from this pass's answer, once
                // (M46, docs/DESIGN.md §51). Two reliefs in series is the plant:
                // Newton's cold pass, everything anchored, fails with the second
                // relief still shut; the next pass converges and points
                // straight back at it.
                if converged.contains(&(next_override.clone(), next_starved.clone())) {
                    return Err(refusal);
                }
                postponed = Some(refusal);
            } else {
                // A starvation-only repeat: the boundary itself, inside the
                // solver's tolerance (§28 fork 3). Keep the pass that cannot
                // create mass — this one, if it is the more starved of the two.
                if next_starved.is_subset(&starved) {
                    accept(&mut result, warm_start, &pressures);
                    return result;
                }
                recovery_frozen = true;
                next_starved = starved.union(&next_starved).copied().collect();
                next_override = next_anchored_set(graph, slate, &compiled, &next_starved, dt);
            }
        }
        previous_pass = Some(pressures);
        override_set = Some(next_override);
        starved = next_starved;
    }
    Err(SimError::AnchoringUnsettled {
        cycled: false,
        detail: format!(
            "still producing new classifications after {MAX_ANCHOR_PASSES} passes — unlike \
             the cycling case there is no repeat to name, so this is a plant whose anchoring \
             keeps growing rather than one alternating between two answers \
             (docs/DESIGN.md §3c)"
        ),
    })
}

/// A relief's opening [0..1] at its own node's pressure `p` [Pa]: full lift
/// while a pop valve is lifted, its curve otherwise. `None` for any other kind.
fn relief_opening_at(kind: &NodeKind, p: f64) -> Option<f64> {
    match kind {
        NodeKind::ReliefValve {
            blowdown: Some(Blowdown { lifted: true, .. }),
            ..
        } => Some(1.0),
        NodeKind::ReliefValve {
            set_pressure,
            accumulation,
            ..
        } => Some(relief_opening(
            p,
            set_pressure.value(),
            accumulation.value(),
        )),
        _ => None,
    }
}

/// Every relief's open-or-shut state at `pressures`, `true` when open. A relief
/// with no pressure there — before the first solve — reads shut.
fn relief_states(graph: &PlantGraph, pressures: &BTreeMap<NodeId, f64>) -> BTreeMap<NodeId, bool> {
    graph
        .node_ids()
        .filter_map(|nid| {
            let kind = &graph.node(nid).kind;
            relief_opening_at(kind, 0.0)?;
            let open = pressures
                .get(&nid)
                .and_then(|&p| relief_opening_at(kind, p))
                .is_some_and(|opening| opening > 0.0);
            Some((nid, open))
        })
        .collect()
}

/// A relief keeps its last answer where the plant has two (M48.1,
/// docs/DESIGN.md §53, ledger row A22): "start shut, stay as was".
///
/// `solve` is one fidelity's whole solve on the graph it is handed. It runs
/// first on the plant as it is (A). Each relief's REMEMBERED state is read off
/// `warm_start` — the last accepted answer's pressures, so tick 1 remembers every
/// relief shut. A relief A puts in the other state has FLIPPED; with none, A is
/// returned untouched, which is every tick of every plant whose reliefs only
/// lift and reseat on their own inlet.
///
/// Otherwise the solve is repeated (B) on a hydraulic copy of the plant with each
/// flipped relief held where it was — a plain valve at opening 0 or 1 — from the
/// warm start A began with. B is kept when every held relief's own opening at
/// B's pressure agrees with how it was held: then the held valve compiled to the
/// branch the relief itself would have, and B is an exact root of the real
/// plant, chosen for its history. A held relief that disagrees is released and B
/// re-run. A relief that opened because its inlet rose past set always
/// disagrees — held shut, its inlet is no lower — so it is released and A
/// stands; what B keeps is a relief whose own opening put its inlet where it is.
///
/// When B cannot be kept, A is returned with A's own warm start.
pub fn solve_remembering_reliefs<F>(
    graph: &PlantGraph,
    warm_start: &mut BTreeMap<NodeId, f64>,
    mut solve: F,
) -> Result<HydraulicSolution, SimError>
where
    F: FnMut(&PlantGraph, &mut BTreeMap<NodeId, f64>) -> Result<HydraulicSolution, SimError>,
{
    let remembered = relief_states(graph, warm_start);
    if remembered.is_empty() {
        return solve(graph, warm_start);
    }
    let seed = warm_start.clone();
    let answer = solve(graph, warm_start)?;
    let pressures: BTreeMap<NodeId, f64> = answer
        .node_pressure
        .iter()
        .map(|(&nid, p)| (nid, p.value()))
        .collect();
    let mut held: BTreeMap<NodeId, bool> = relief_states(graph, &pressures)
        .into_iter()
        .filter(|(nid, open)| remembered[nid] != *open)
        .map(|(nid, open)| (nid, !open))
        .collect();
    if held.is_empty() {
        return Ok(answer);
    }
    let answer_warm_start = std::mem::replace(warm_start, seed.clone());
    while !held.is_empty() {
        let mut copy = graph.hydraulic_copy();
        for (&nid, &open) in &held {
            let NodeKind::ReliefValve { cv_max, x_t, .. } = graph.node(nid).kind else {
                unreachable!("only reliefs are held")
            };
            copy.node_mut(nid).kind = NodeKind::Valve {
                cv_max,
                opening: if open { 1.0 } else { 0.0 },
                x_t,
            };
        }
        let Ok(kept) = solve(&copy, warm_start) else {
            break;
        };
        let disagree: Vec<NodeId> = held
            .iter()
            .filter(|&(&nid, &open)| {
                let opening = kept
                    .node_pressure
                    .get(&nid)
                    .and_then(|p| relief_opening_at(&graph.node(nid).kind, p.value()));
                opening != Some(if open { 1.0 } else { 0.0 })
            })
            .map(|(&nid, _)| nid)
            .collect();
        if disagree.is_empty() {
            return Ok(kept);
        }
        for nid in disagree {
            held.remove(&nid);
        }
        *warm_start = seed.clone();
    }
    *warm_start = answer_warm_start;
    Ok(answer)
}

/// The refusal of a repeat between the anchored sets `used` and `next`: the
/// plant alternates between two answers (docs/DESIGN.md §3c).
fn chatter(graph: &PlantGraph, used: &BTreeSet<NodeId>, next: &BTreeSet<NodeId>) -> SimError {
    SimError::AnchoringUnsettled {
        cycled: true,
        detail: format!(
            "{} has been anchored before in this tick, so the plant has two \
             self-consistent answers and the element(s) at {} are chattering — a \
             relief whose own discharge re-seats it. A relief's last state picks \
             between two answers only once a solve has found one, which this one \
             has not; a pop valve's reseat below its set is `blowdown_bar` \
             (docs/DESIGN.md §3c, §53)",
            describe_nodes(graph, next),
            describe_nodes(graph, &symmetric_difference(used, next)),
        ),
    }
}

/// A stretch of line closed at its far end, and where each of its nodes
/// stands at zero drive (M45.0, docs/DESIGN.md §50).
struct DeadEnd {
    stretch: BTreeSet<NodeId>,
    /// Each node of the stretch's pressure [Pa], see `dead_end_tie`.
    standing: BTreeMap<NodeId, f64>,
}

/// A repeat `dead_end_tie` has a verdict on.
enum Tie {
    /// A dead end, standing where its one way in leaves it: answered.
    Stands(DeadEnd),
    /// A dead end that would stand at or below vacuum — a broken column. Refused,
    /// and never re-run (M46, docs/DESIGN.md §50 fork 5, §51).
    BelowVacuum,
}

/// Whether a repeat between the anchored sets `floating` and `filled` (a
/// superset of it) is a DEAD END's tie rather than chatter (M45.0,
/// docs/DESIGN.md §50), and if so where the dead end stands.
///
/// The nodes only `filled` anchors form a stretch of line with no inventory in
/// it — no vessel, no starved tank, nothing with a term of its own — joined to
/// the rest of the plant, in the compile `filled` came from, by exactly ONE
/// conducting edge. Mass balance forces that edge's flow to zero, and with it
/// every flow in the stretch, so the two classifications are one answer for
/// every flow in the plant. What they disagree on is the stretch's pressure,
/// and neither has it: floating, it is parked at a stale value; filled, the
/// element on the way in shuts on the way to the answer and leaves it wherever
/// the iterate was (8.99 bar behind a 5 bar header on the first fixture, and a
/// negative absolute pressure on the second).
///
/// The case it exists for is a pump started against its shut discharge valve
/// with a check valve between them: filled, the stretch stands at the pump's
/// pressure and the disc's forward drive is zero, so it shuts; floating, the
/// stretch is parked below the pump's pressure, so it opens.
///
/// **Where it stands.** The liquid the element lets in or out moves until the
/// drive across it is zero, and no further. At zero flow a branch's drop is its
/// offset alone, `P_src − P_tgt = β` (`QuadraticBranch`: static head plus any
/// pump jump), so each node of the stretch is reached from the one outside it
/// by subtracting `β` along each edge followed forward and adding it along each
/// followed backward. `compiled` is the compile at the kept pass's pressures,
/// and stands the stretch first; a gas column's head moves with its pressure,
/// so the stretch is then re-stood from its own recompile until it stops moving
/// (M47, docs/DESIGN.md §52), and the offsets kept are the ones read where it
/// stands.
///
/// **Then the stretch must stay a dead end where it stands**: every OTHER edge
/// across its boundary, recompiled at those pressures, still conducts nothing.
/// A relief chain whose second valve was shut only because the stretch was
/// parked low, and lifts once it is filled, fails it, and is no dead end: the
/// driver then judges it as any other repeat (M46, §51). The way in is not
/// asked: it sits at zero drive by construction, where a disc is shut to within
/// a rounding and a relief that senses only its own inlet may stand open
/// passing nothing.
///
/// **Nor below vacuum.** A stretch rising far enough above a low-pressure way
/// in would stand at a negative absolute pressure, which is a column that has
/// broken rather than a dead end: `Tie::BelowVacuum`, which the driver refuses
/// outright rather than re-running.
///
/// `conducts_in_filled` answers for the compile `filled` was built from: the
/// pass's own on Newton's road to the repeat, the recompile on the game
/// solver's (the two reach it from opposite ends).
#[allow(clippy::too_many_arguments)]
fn dead_end_tie(
    graph: &PlantGraph,
    slate: &Slate,
    previous_states: &NodeStates,
    floating: &BTreeSet<NodeId>,
    filled: &BTreeSet<NodeId>,
    capacitive: &BTreeMap<NodeId, Capacitance>,
    conducts_in_filled: impl Fn(EdgeId) -> bool,
    compiled: &BTreeMap<EdgeId, CompiledEdge>,
    pressures: &BTreeMap<NodeId, f64>,
) -> Option<Tie> {
    let stretch: BTreeSet<NodeId> = filled.difference(floating).copied().collect();
    if stretch.is_empty() || stretch.iter().any(|nid| capacitive.contains_key(nid)) {
        return None;
    }
    let crosses = |eid: EdgeId| {
        let (src, tgt) = graph.endpoints(eid);
        stretch.contains(&src) != stretch.contains(&tgt)
    };
    let mut ways_in = graph
        .edge_ids()
        .filter(|&eid| crosses(eid) && conducts_in_filled(eid));
    let way_in = ways_in.next()?;
    if ways_in.next().is_some() {
        return None;
    }

    let (src, tgt) = graph.endpoints(way_in);
    let stand = |offsets: &BTreeMap<EdgeId, CompiledEdge>| {
        let beta = offsets[&way_in].branch.beta;
        let (first, at_first) = if stretch.contains(&tgt) {
            (tgt, pressures[&src] - beta)
        } else {
            (src, pressures[&tgt] + beta)
        };
        let mut standing: BTreeMap<NodeId, f64> = BTreeMap::new();
        standing.insert(first, at_first);
        let mut stack = vec![first];
        while let Some(nid) = stack.pop() {
            let here = standing[&nid];
            for (eid, other, incoming) in graph.incident(nid) {
                if !stretch.contains(&other)
                    || standing.contains_key(&other)
                    || !conducts_in_filled(eid)
                {
                    continue;
                }
                let beta = offsets[&eid].branch.beta;
                standing.insert(other, if incoming { here + beta } else { here - beta });
                stack.push(other);
            }
        }
        standing
    };

    // The offsets are read where the stretch STANDS, not where the kept pass
    // left it (M47, docs/DESIGN.md §52): a gas column's head moves with its
    // pressure, so the stretch is re-stood from its own recompile until it
    // stops moving. Each pass shrinks the error by about `g·|Δz|·dρ/dP`, 3e-4
    // on A22's 3.8 m drop, and a liquid's density does not move at all, so the
    // first recompile already agrees and the stretch stands where it did.
    let mut standing = stand(compiled);
    let mut restands = 0;
    let recompiled = loop {
        // A stretch standing at or below vacuum is a column the liquid cannot
        // hold up: not a state this rule can name, so the tie is refused as before.
        if standing.values().any(|&p| p <= 0.0) {
            return Some(Tie::BelowVacuum);
        }
        let mut settled = pressures.clone();
        settled.extend(standing.iter().map(|(&nid, &p)| (nid, p)));
        let recompiled = compile_edges(graph, slate, previous_states, &settled).ok()?;
        let restood = stand(&recompiled);
        let moved = standing
            .iter()
            .map(|(nid, p)| (restood[nid] - p).abs())
            .fold(0.0, f64::max);
        if moved <= DEAD_END_STANDING_TOL {
            break recompiled;
        }
        // Still moving after the cap: not a column this rule can stand, so the
        // driver judges the repeat as any other.
        if restands == MAX_DEAD_END_RESTANDS {
            return None;
        }
        restands += 1;
        standing = restood;
    };
    let still_closed = graph
        .edge_ids()
        .filter(|&eid| eid != way_in && crosses(eid))
        .all(|eid| !recompiled[&eid].conducts);
    still_closed.then_some(Tie::Stands(DeadEnd { stretch, standing }))
}

/// Stand a dead end where `dead_end_tie` found it, with nothing flowing in it
/// (M45.0, docs/DESIGN.md §50): every edge with an end in the stretch carries
/// exactly zero and dissipates nothing. The ones that did not conduct already
/// did; the way in and the stretch's own pipes close to the solver's tolerance
/// in the pass, and this is the answer that tolerance stood for.
fn settle_dead_end(
    graph: &PlantGraph,
    dead_end: &DeadEnd,
    pressures: &mut BTreeMap<NodeId, f64>,
    solution: &mut HydraulicSolution,
) {
    for (&nid, &p) in &dead_end.standing {
        pressures.insert(nid, p);
        solution.node_pressure.insert(nid, Pascal(p));
    }
    for eid in graph.edge_ids() {
        let (a, b) = graph.endpoints(eid);
        if dead_end.stretch.contains(&a) || dead_end.stretch.contains(&b) {
            solution.edge_mass_flow.insert(eid, 0.0);
            solution.edge_dissipation.insert(eid, Watt::ZERO);
        }
    }
}

/// Each tank in `starved` mapped to its supply [kg/s], through the one
/// expression the report shares (`starved_supply`).
fn starved_supplies(
    graph: &PlantGraph,
    starved: &BTreeSet<NodeId>,
    dt: Seconds,
) -> BTreeMap<NodeId, f64> {
    starved
        .iter()
        .filter_map(|&nid| match &graph.node(nid).kind {
            NodeKind::Tank(tank) => Some((nid, starved_supply(tank, dt))),
            _ => None,
        })
        .collect()
}

/// The anchored set a pass under `starved` starts from: reachability from THAT
/// classification's anchors over the edges compiled at the last pass's
/// pressures. Edges do not depend on the classification, only on pressures, so
/// only the anchors move.
fn next_anchored_set(
    graph: &PlantGraph,
    slate: &Slate,
    compiled: &BTreeMap<EdgeId, CompiledEdge>,
    starved: &BTreeSet<NodeId>,
    dt: Seconds,
) -> BTreeSet<NodeId> {
    let classes = classify(graph, slate, &starved_supplies(graph, starved, dt));
    anchored_set(graph, compiled, &base_anchors(&classes))
}

/// Net mass flow OUT of `node` [kg/s] over its incident edges, in the solve's own
/// flows. Column draws and boil-off vents are zero there (`edge_flows`), so this
/// is the pressure-driven net outflow — the quantity a tank's inventory must
/// cover. Self-loops carry nothing anywhere and are skipped.
fn net_outflow(graph: &PlantGraph, edge_mass_flow: &BTreeMap<EdgeId, f64>, node: NodeId) -> f64 {
    let mut out = 0.0;
    for (eid, other, incoming) in graph.incident(node) {
        if other == node {
            continue;
        }
        let flow = edge_mass_flow.get(&eid).copied().unwrap_or(0.0);
        out += if incoming { -flow } else { flow };
    }
    out
}

/// The starved set a converged pass implies (docs/DESIGN.md §28 fork 3).
///
/// A wet tank starves when `q_out·dt > m`, strictly — the rule the prototype
/// ran, and the one under which a wet tank can over-draw by rounding only. A
/// starved tank recovers when the solve puts it above the pressure it would pin
/// at while wet: the network is then pushing into it rather than pulling out of
/// it, and a pinned tank is the right model again. `recovery_frozen` suppresses
/// the second rule (see the driver).
fn reclassify_starved(
    graph: &PlantGraph,
    slate: &Slate,
    solution: &HydraulicSolution,
    starved: &BTreeSet<NodeId>,
    dt: Seconds,
    recovery_frozen: bool,
) -> BTreeSet<NodeId> {
    let mut next = BTreeSet::new();
    for nid in graph.node_ids() {
        let NodeKind::Tank(tank) = &graph.node(nid).kind else {
            continue;
        };
        if starved.contains(&nid) {
            let recovers = !recovery_frozen
                && solution
                    .node_pressure
                    .get(&nid)
                    .is_some_and(|p| p.value() > tank.bottom_pressure(slate).value());
            if !recovers {
                next.insert(nid);
            }
        } else if net_outflow(graph, &solution.edge_mass_flow, nid) * dt.value() > tank.mass.value()
        {
            next.insert(nid);
        }
    }
    next
}

/// Attach the accepted pass's per-holdup report to its solution: each starved
/// tank's supply and own residual, and each vessel's own residual (§28 fork 5).
///
/// Both residuals are `R = (own term) + Σ ṁ_in − Σ ṁ_out` at the solution the
/// solve returns — the residual convention of `accumulation`. They are the
/// node's OWN, not `SolveDiagnostics::residual`, which is the plant's worst.
fn report_holdups(
    graph: &PlantGraph,
    solution: &mut HydraulicSolution,
    supplies: &BTreeMap<NodeId, f64>,
    capacitive: &BTreeMap<NodeId, Capacitance>,
    pressures: &BTreeMap<NodeId, f64>,
    dt: Seconds,
) {
    for (&nid, cap) in capacitive {
        let net_out = net_outflow(graph, &solution.edge_mass_flow, nid);
        match cap {
            Capacitance::Starved { supply, .. } => {
                debug_assert_eq!(supplies.get(&nid), Some(supply));
                solution.starved.insert(
                    nid,
                    StarvedTank {
                        supply: KgPerSec(*supply),
                        residual: KgPerSec(supply - net_out),
                    },
                );
            }
            Capacitance::Vessel { .. } => {
                if let Some(&p) = pressures.get(&nid) {
                    let term = accumulation(cap, p, dt.value()).0;
                    solution
                        .vessel_residual
                        .insert(nid, KgPerSec(term - net_out));
                }
            }
        }
    }
}

/// Nodes in one set or the other but not both, for a diagnostic.
fn symmetric_difference(a: &BTreeSet<NodeId>, b: &BTreeSet<NodeId>) -> BTreeSet<NodeId> {
    a.symmetric_difference(b).copied().collect()
}

/// Node ids rendered by NAME for an error message — ascending, so the text is
/// deterministic like everything else the solve produces.
fn describe_nodes(graph: &PlantGraph, nodes: &BTreeSet<NodeId>) -> String {
    if nodes.is_empty() {
        return "{}".to_string();
    }
    let names: Vec<&str> = nodes
        .iter()
        .map(|&nid| graph.node(nid).name.as_str())
        .collect();
    format!("{{{}}}", names.join(", "))
}

/// What one solve reports per edge: the signed mass flow, the power friction
/// dissipates into the stream, and the largest flow magnitude anywhere.
pub struct EdgeResults {
    /// Mass flow [kg/s] in graph direction (positive = source → target).
    pub mass_flow: BTreeMap<EdgeId, f64>,
    /// Friction power [W] into the stream, always ≥ 0. See `edge_flows`.
    pub dissipation: BTreeMap<EdgeId, Watt>,
    /// `max |ṁ|` over all edges [kg/s] — the scale the relative convergence
    /// tolerance is measured against.
    pub throughput: f64,
}

/// Mass flow (kg/s) and frictional dissipation (W) per edge in graph direction;
/// inert edges (either endpoint unanchored) report 0 for both.
///
/// **The dissipation rule needs no new parameter** (docs/DESIGN.md §3a).
/// `QuadraticBranch` already separates the two kinds of pressure term along
/// exactly the physical line: `α·Q|Q|` is friction — pipe wall, valve trim, pump
/// curve droop — and becomes heat in the fluid, while `β` is elevation head
/// (reversible potential) and the pump's pressure jump (shaft work in). So the
/// dissipated power on a branch is
///
/// ```text
/// Φ = α·Q|Q| · Q   [W]
/// ```
///
/// which is ≥ 0 for either flow direction because `α ≥ 0` and `Q|Q|·Q = |Q|³`.
/// It is computed HERE, where `α` and `Q` are both already in hand, rather than
/// recovered downstream from the pressure drop: `(P_up − P_down) − β` would give
/// the same number and would require `core` to know `β`, which is element
/// physics (rule 2).
///
/// Note `Q` is recovered as `ṁ/ρ` rather than re-evaluated from `branch.flow`,
/// so the reported dissipation belongs to the very flow this function reports —
/// including the two cases where that flow is NOT `ρ·branch.flow(dp)`: an inert
/// edge and a column draw. Both then carry `Φ = 0`, which is the honest answer
/// for each. An inert edge is stagnant. A column draw's flow is *prescribed*
/// (`splitᵢ·ṁ_feed`) and is not pressure-driven at all, so `α·Q|Q|` is not its
/// pressure drop and booking it would invent heat; a draw consequently leaves
/// the column at exactly the feed temperature, which is the limitation DESIGN §5
/// already states and `column_reference` already gates.
///
/// **Column draw edges and boil-off vents are guarded to zero here, not
/// computed.** A draw's flow is
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
///
/// A boil-off vent (M12) is the same case with a different owner: its rate comes
/// from the holdup's enthalpy balance, `Engine::tick` writes it post-update, and
/// left pressure-driven it would drain a tank to atmosphere through a pipe
/// nobody declared. It is recognised by its ROLE rather than by its endpoints,
/// because a scenario is free to declare an ordinary pipe from a tank to an
/// atmosphere node and that pipe must stay pressure-driven.
pub fn edge_flows(
    graph: &PlantGraph,
    compiled: &BTreeMap<EdgeId, CompiledEdge>,
    pressures: &BTreeMap<NodeId, f64>,
    anchored: &BTreeSet<NodeId>,
    eps: f64,
) -> EdgeResults {
    let mut mass_flow = BTreeMap::new();
    let mut dissipation = BTreeMap::new();
    let mut throughput = 0.0f64;
    for eid in graph.edge_ids() {
        let c = &compiled[&eid];
        let mdot = if is_column_draw_edge(graph, eid) || graph.pipe(eid).leak.is_engine_written() {
            0.0
        } else if anchored.contains(&c.src) && anchored.contains(&c.tgt) {
            let dp = pressures[&c.src] - pressures[&c.tgt];
            c.rho * c.branch.flow(dp, eps)
        } else {
            0.0
        };
        // Φ = α·Q|Q|·Q with Q = ṁ/ρ. A closed branch has α = +∞ and ṁ = 0, so
        // the product would be ∞·0 = NaN; it is exactly the case with no flow to
        // heat, so it is zero by the same test that makes `flow` return zero.
        let q = mdot / c.rho;
        let phi = if q == 0.0 {
            0.0
        } else {
            c.branch.alpha * q * q.abs() * q
        };
        throughput = throughput.max(mdot.abs());
        mass_flow.insert(eid, mdot);
        dissipation.insert(eid, Watt(phi));
    }
    EdgeResults {
        mass_flow,
        dissipation,
        throughput,
    }
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
/// escapes a solve) and the leak back-feed refusal. Only called on a converged
/// solve, hence `converged: true`.
///
/// **The back-feed refusal is here, in the shared epilogue, for the reason every
/// other shared rule is** — both fidelities must inherit it from one definition
/// or I5 stops meaning anything, and the compiler enforces that by making
/// `graph` a parameter of the one function all four solver exits already go
/// through.
///
/// *What it refuses, and why an `Err`.* An `Atmosphere` node's composition is
/// the first slate component — arbitrary, and honestly labelled so at
/// `energy::boundary_composition`, on the premise that leak edges run only
/// *into* it. A leak edge is pressure-driven like any other, so the premise
/// expires the moment one exists: a plant below `P_ATM` draws that arbitrary
/// component back in, and the result is mass-conserving, finite, deterministic
/// and wrong — DESIGN §5's silent hazard in the one place the code predicted it.
/// Of the four resolutions DESIGN §3b prices, this is the cheap one. Pinning a
/// real air composition is a change to every slate that owns an Atmosphere
/// (the FCC slate has no air-like cut); a one-way orifice is a check valve on a
/// hole, which is not physics — a hole admits air, and pretending it does not is
/// the last option in other clothes (the numerical objection once recorded here,
/// a `conducts` that depends on the pressure iterate, expired with M8.0's
/// active-set anchoring, and M30 built exactly such an element as
/// `NodeKind::CheckValve`); and back-feeding the plant's own composition models air ingress as nothing
/// happening. Refusing costs a game that pulls a leaking line below atmospheric
/// a hard error instead of a plausible picture, and buys the guarantee that no
/// invented composition ever enters the plant.
///
/// *Why the test needs no tolerance.* The refusal is `ṁ < 0` on the nose, and
/// that is exact rather than tight: an orifice branch is built with `beta = 0`
/// (`QuadraticBranch::orifice`, where the early return in `compile_edge` keeps
/// it structural), and `QuadraticBranch::flow`'s sign is `sign(dp − beta)`. So a
/// leak edge carries mass inward **iff** its junction is strictly below `P_ATM`,
/// which is the physical condition itself. There is no band of near-zero flows
/// to argue about: at `dp = 0` the flow is exactly 0 and this does not fire.
pub fn finalize(
    graph: &PlantGraph,
    pressures: &BTreeMap<NodeId, f64>,
    edges: EdgeResults,
    iterations: u32,
    residual: f64,
) -> Result<HydraulicSolution, SimError> {
    for eid in graph.edge_ids() {
        let pipe = graph.pipe(eid);
        let LeakRole::Orifice { area } = pipe.leak else {
            continue;
        };
        // A dormant leak cannot back-feed — it conducts nothing in either
        // direction — so this reads the AREA rather than trusting the flow to be
        // zero, and every reference plant with an undamaged declared leak path
        // stays on the same arm as a plant with no leak path at all.
        if area.value() > 0.0 && edges.mass_flow.get(&eid).copied().unwrap_or(0.0) < 0.0 {
            return Err(SimError::Numerical(format!(
                "leak '{}' ({eid:?}) back-feeds: the plant side is below atmospheric, so \
                 the solve draws {:.4e} kg/s of ATMOSPHERE into the plant. An Atmosphere \
                 node's composition is an arbitrary stand-in (the slate's first \
                 component), so continuing would inject a fluid nobody chose — \
                 mass-conserving, finite and wrong. Close the leak, or keep the line \
                 above {:.0} Pa (docs/DESIGN.md §3b)",
                pipe.name,
                -edges.mass_flow[&eid],
                P_ATM.value()
            )));
        }
    }
    let mut node_pressure = BTreeMap::new();
    for (nid, p) in pressures {
        if !p.is_finite() {
            return Err(SimError::NonFiniteState {
                location: format!("{nid:?} pressure"),
            });
        }
        node_pressure.insert(*nid, Pascal(*p));
    }
    for (eid, f) in &edges.mass_flow {
        if !f.is_finite() {
            return Err(SimError::NonFiniteState {
                location: format!("{eid:?} mass flow"),
            });
        }
    }
    // Dissipation is scanned separately rather than trusted to follow the flow:
    // it is a CUBE of the flow, so an edge whose ṁ is merely large produces a Φ
    // that overflows to +∞ while the flow itself stays finite. It also feeds a
    // temperature directly, where an infinity would surface as a bare
    // non-finite with no edge named.
    for (eid, phi) in &edges.dissipation {
        if !phi.value().is_finite() || phi.value() < 0.0 {
            return Err(SimError::NonFiniteState {
                location: format!("{eid:?} frictional dissipation ({} W)", phi.value()),
            });
        }
    }
    Ok(HydraulicSolution {
        node_pressure,
        edge_mass_flow: edges.mass_flow,
        edge_dissipation: edges.dissipation,
        diagnostics: SolveDiagnostics {
            iterations,
            residual,
            converged: true,
        },
        // Filled by the driver from the pass it ACCEPTS, never by a pass: a pass
        // does not know whether it will be kept (`solve_with_active_anchoring`).
        starved: BTreeMap::new(),
        vessel_residual: BTreeMap::new(),
    })
}
