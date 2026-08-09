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
use refinery_core::energy::boundary_temperature;
use refinery_core::error::SimError;
use refinery_core::graph::{EdgeId, Node, NodeId, NodeKind, PlantGraph};
use refinery_core::traits::{HydraulicSolution, SolveDiagnostics};
use refinery_core::units::{Pascal, Watt, G, P_ATM};
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
}

/// Boundary classification of the graph's nodes for one solve: which nodes pin
/// pressure, which are free unknowns, and the deterministic cold-start seed.
///
/// The anchored set is NOT here, and that ordering is load-bearing since M5.2:
/// anchoring needs the compiled edges, compiling a gas edge needs a pressure to
/// evaluate ρ(P,T) at, and that pressure needs the cold seed. `prepare` owns
/// the resulting three-step order so neither solver can get it wrong.
pub struct Classification {
    /// Pinned pressures [Pa] for fixed nodes (Source/Sink/Atmosphere/Tank).
    pub fixed: BTreeMap<NodeId, f64>,
    /// Free node ids (Junction/Pump/Valve), ascending — deterministic order.
    pub free: Vec<NodeId>,
    /// Deterministic cold-start pressure seed: mean of the fixed pressures, or
    /// P_ATM when the network has no fixed node at all.
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
    /// Fixed ∪ free-reachable-via-conducting-edges. Free nodes NOT in this set
    /// are floating (indeterminate pressure); their incident edges carry zero.
    pub anchored: BTreeSet<NodeId>,
    /// Seeded pressures: fixed pinned, anchored free warm-started or cold,
    /// floating free warm-started or at `P_ATM`.
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
/// STATED LIMITATION, with the measured size. `energy::boundary_temperature`
/// answers only for a node that HAS a temperature of its own — Source, Sink,
/// Atmosphere, Tank, and the capacitive vessel. A zero-volume upwind node
/// (junction, valve, pump, exchanger side) has none: its temperature is the
/// sweep's mix, which does not exist when the solve runs and is not stored on the
/// graph. Those edges keep the stored-outlet fallback and keep the error above.
/// On `gas_line.toml` at steady state that is the relief line compiling at its
/// own outlet's **375.0 K** instead of the tee's **297.3 K** — a 21% density
/// error. It un-defers when the flow solver gains access to the previous tick's
/// resolved node states, which is a `FlowSolver` signature change and belongs to
/// the slice that first needs it: M5.4's PSV plant is vessel → valve → relief
/// line → flare, where the relief line's upwind node is the valve.
pub fn compile_edge(
    graph: &PlantGraph,
    eid: EdgeId,
    slate: &Slate,
    pressures: &BTreeMap<NodeId, f64>,
) -> Result<CompiledEdge, SimError> {
    let (src, tgt) = graph.endpoints(eid);
    let pipe = graph.pipe(eid);
    let upwind_node = if pressures[&src] >= pressures[&tgt] {
        src
    } else {
        tgt
    };
    let upwind = pressures[&upwind_node].max(RHO_EVAL_P_FLOOR);
    let temperature =
        boundary_temperature(&graph.node(upwind_node).kind).unwrap_or(pipe.stream.temperature);
    let rho = pipe
        .stream
        .composition
        .density_at(slate, Pascal(upwind), temperature)
        .map_err(|e| SimError::Numerical(format!("pipe {} ({eid:?}): {e}", pipe.name)))?
        .value();
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

/// Compile every edge's series branch (pipe ∘ device-at-source), keyed by edge,
/// at the given pressure iterate. Called once per solver iteration since M5.2 —
/// see `compile_edge` for why, and why an all-liquid network is unaffected.
pub fn compile_edges(
    graph: &PlantGraph,
    slate: &Slate,
    pressures: &BTreeMap<NodeId, f64>,
) -> Result<BTreeMap<EdgeId, CompiledEdge>, SimError> {
    let mut compiled = BTreeMap::new();
    for eid in graph.edge_ids() {
        compiled.insert(eid, compile_edge(graph, eid, slate, pressures)?);
    }
    Ok(compiled)
}

/// The shared solve prologue: classify, seed, compile, anchor, pin floating —
/// in the one order that is self-consistent, and identical for both fidelities.
pub fn prepare(
    graph: &PlantGraph,
    slate: &Slate,
    warm_start: &BTreeMap<NodeId, f64>,
) -> Result<Prepared, SimError> {
    let classes = classify(graph, slate);

    // Seed every free node before compiling, because a gas edge's density is
    // evaluated at a node pressure. Floating nodes are re-pinned below, once
    // there is an anchored set to tell them apart; the intermediate value
    // reaches only `conducts`, which is sign-of-alpha and cannot differ, and the
    // edges it reaches are inert either way.
    let mut pressures = classes.fixed.clone();
    for &nid in &classes.free {
        let seed = warm_start.get(&nid).copied().unwrap_or(classes.cold);
        pressures.insert(nid, seed);
    }

    let compiled = compile_edges(graph, slate, &pressures)?;
    let fixed_set: BTreeSet<NodeId> = classes.fixed.keys().copied().collect();
    let anchored = anchored_set(graph, &compiled, &fixed_set);

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
pub fn classify(graph: &PlantGraph, slate: &Slate) -> Classification {
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
    Classification { fixed, free, cold }
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
) -> EdgeResults {
    let mut mass_flow = BTreeMap::new();
    let mut dissipation = BTreeMap::new();
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
/// escapes a solve). Only called on a converged solve, hence `converged: true`.
pub fn finalize(
    pressures: &BTreeMap<NodeId, f64>,
    edges: EdgeResults,
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
    })
}
