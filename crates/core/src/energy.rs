//! Transport of the intensive fields: upwind (donor-cell) advection of
//! temperature and composition.
//!
//! Both ride ONE sweep (`resolve_node_states`). They share the topology — the
//! same inflow edges, the same upwind rule, the same ordering, the same recycle
//! rejection — and differ only in the weight each mix uses: enthalpy by `ṁ·cp`,
//! composition by `ṁ` alone. See `mix_compositions` for why that difference is
//! the sharpest edge in this module.
//!
//! The temperature field over the plant graph is *mixed*, and naming the two
//! kinds of node is what makes it tractable (docs/DESIGN.md §4a):
//!
//! - **Inertial nodes** (Tank, Source, Sink, Atmosphere) carry their own
//!   temperature. Tanks integrate it as a slow state; the reservoirs hold it
//!   fixed. Within a tick their temperature is a *boundary condition*, read at
//!   its start-of-tick value.
//! - **Zero-volume nodes** (Junction, Pump, Valve) hold no inventory, so their
//!   temperature is not a state at all — it is *algebraic*, the instantaneous
//!   enthalpy-weighted mix of whatever flows in.
//!
//! An edge's stream temperature is its **upwind** node's temperature, chosen by
//! the sign of the solved flow (so reverse flow is handled without a special
//! case). Zero-volume nodes therefore have to be resolved in flow order —
//! upstream before downstream — which is a topological sort over the *flow*
//! direction of this tick, not the graph's edge direction.
//!
//! That ordering exists unless a recycle passes through zero-volume nodes
//! **only**: any tank or reservoir in a loop breaks the dependency, because its
//! temperature is a start-of-tick constant rather than a function of its
//! inflows. A zero-volume-only recycle would need a simultaneous solve; it is
//! rejected as `SimError::Numerical` rather than silently mis-ordered. No
//! scenario in the workspace builds one (see `docs/DESIGN.md` §4a).
//!
//! Specific enthalpy is `h = cp·(T − T_REF)`. Every enthalpy flux in the engine
//! goes through `enthalpy_flux`, so the reference cancels exactly as long as
//! mass balances — the invariant tests state it against `T_REF` explicitly
//! rather than assuming a zero reference makes it moot.

use crate::components::{Composition, Slate};
use crate::error::SimError;
use crate::graph::{EdgeId, NodeId, NodeKind, PlantGraph};
use crate::units::{JPerKgK, Kelvin, KgPerSec, Watt, WattPerKelvin, T_AMBIENT};
use std::collections::{BTreeMap, BTreeSet};

/// Reference temperature for specific enthalpy: `h = cp·(T − T_REF)` [K].
///
/// Deliberately non-zero (0 °C, the conventional steam-table datum). A zero
/// reference would make `h = cp·T` and hide any code path that forgot the
/// datum entirely; with 273.15 K, dropping it changes the answer, so the
/// energy tests actually discriminate on it.
pub const T_REF: Kelvin = Kelvin(273.15);

/// Enthalpy flux carried by a mass flow: `ṁ·cp·(T − T_REF)`.
///
/// Sign follows `mass_flow`: the caller passes flow *into* the node it is
/// accounting for, so an outflow (negative) subtracts its enthalpy. This is
/// the single definition every energy balance in the engine goes through —
/// tank integration, junction mixing, and the invariant tests all call it, so
/// they cannot disagree about the datum.
#[inline]
pub fn enthalpy_flux(mass_flow: KgPerSec, cp: JPerKgK, temperature: Kelvin) -> Watt {
    Watt(mass_flow.value() * cp.value() * (temperature.value() - T_REF.value()))
}

/// Heat exchanged with the surroundings [W], SIGNED: positive into the body.
///
/// Newton's law of cooling, in the lumped `UA` form (any heat transfer text):
///
/// ```text
/// Q_ambient = UA·(T_AMBIENT − T_body)
/// ```
///
/// This is NOT "heat loss", and the naming carries weight (docs/DESIGN.md §4a).
/// The driving force is a temperature DIFFERENCE, so the one term must heat a
/// body colder than ambient and cool one hotter — a one-directional loss would
/// be wrong for a chilled tank on a warm day, and with the `Cooler` and the
/// `HeatExchanger` in place that is a reachable plant state rather than a
/// hypothetical. The direction falls out of the subtraction, so there is no
/// `if colder` branch, no second code path, and no sign convention of its own:
/// the same move that made `heat_load` the sole owner of the duty sign.
///
/// Ambient is the global `T_AMBIENT`, the same constant the `Atmosphere` node is
/// pinned to. A body sitting in the outside world and the outside world itself
/// must agree on how warm it is; a per-unit ambient would let them disagree.
#[inline]
pub fn ambient_exchange(ua: WattPerKelvin, body_temperature: Kelvin) -> Watt {
    Watt(ua.value() * (T_AMBIENT.value() - body_temperature.value()))
}

/// Outlet temperature of a pipe exchanging heat with ambient [K].
///
/// The analytic plug-flow solution — integrating `ṁ·cp·dT/dx = ua'·(T_AMBIENT −
/// T)` along the pipe, the same Newton's-law driving force as
/// `ambient_exchange` but applied to a body with throughput and no inventory:
///
/// ```text
/// T_out = T_AMBIENT + (T_in − T_AMBIENT)·exp(−UA/(|ṁ|·cp))
/// ```
///
/// Three details are load bearing (docs/DESIGN.md §4a):
///
/// - **`|ṁ|`, not signed `ṁ`.** The flow's sign already did its work upstream,
///   selecting which end is the inlet; the denominator is a capacity RATE, a
///   magnitude. A signed `ṁ` flips the exponent on reverse flow, turning decay
///   into growth — the pipe would run *away* from ambient, and only on reversed
///   edges, which the reference plants do not have.
/// - **Analytic, not Euler.** `exp` cannot cross ambient however large the
///   exponent grows, where a `1 − UA/(ṁ·cp)` step overshoots and then diverges.
///   The tank balance tolerates explicit Euler because its `UA·dt/(m·cp)` is
///   ~1e-6 at refinery scale; here the denominator is a FLOW rather than an
///   inventory and a nearly-closed valve drives it toward zero, so the unstable
///   regime is one throttle away rather than unreachable.
/// - **`UA = 0` needs no special case.** `exp(0) = 1` makes the transform the
///   identity, which is what keeps every pre-existing scenario bit-identical.
///   Short-circuiting it with an `if ua == 0.0` would be worse than redundant:
///   it would mask the zero-flow guard below on exactly the case that reaches
///   it, leaving the guard untested and its falsification vacuous.
///
/// Zero flow is guarded, and only zero. At `ṁ = 0` with `UA = 0` the exponent is
/// `0/0 → NaN`, and a NaN temperature slips past the mixing guard to surface as
/// a bare non-finite with no diagnostic naming the pipe; a closed valve produces
/// such edges today. Every OTHER input is already well behaved — `ṁ = 0` with
/// `UA > 0` gives `−∞ → exp = 0 → T_AMBIENT`, and any nonzero `ṁ`, however
/// small, stays finite — so the test is exact equality, not a threshold. A
/// threshold would be a magic number that also silently flattened legitimately
/// small flows.
///
/// Returning the inlet at zero flow is the physically right answer here, not a
/// convenient one: a stagnant pipe carries no enthalpy either way, the same
/// reasoning that lets transport pick an arbitrary upwind end at exactly zero
/// flow. Modelling a stagnant pipe warming toward ambient needs pipe-wall
/// thermal mass, which is a fidelity step, not a guard.
#[inline]
pub fn pipe_outlet_temperature(
    inlet: Kelvin,
    ua: WattPerKelvin,
    mass_flow: KgPerSec,
    cp: JPerKgK,
) -> Kelvin {
    let capacity_rate = mass_flow.value().abs() * cp.value(); // [W/K]
    if capacity_rate == 0.0 {
        return inlet;
    }
    let decay = (-ua.value() / capacity_rate).exp();
    Kelvin(T_AMBIENT.value() + (inlet.value() - T_AMBIENT.value()) * decay)
}

/// The end of `edge` the fluid comes FROM, given a signed mass flow.
///
/// Upwind by flow SIGN, never by edge direction — the donor-cell rule transport
/// uses, so reverse flow needs no special case. At exactly zero flow the pick is
/// arbitrary (nothing moves either way) and takes `from` to stay deterministic.
///
/// Extracted because temperature and composition are transported by the same
/// rule and must never disagree about which way a pipe runs: one definition, two
/// readers.
fn upwind_end(graph: &PlantGraph, edge: EdgeId, mass_flow: f64) -> NodeId {
    let (from, to) = graph.endpoints(edge);
    if mass_flow >= 0.0 {
        from
    } else {
        to
    }
}

/// The composition crossing `node`'s boundary along `edge`.
///
/// The composition analogue of `edge_temperature_at`, and deliberately simpler:
/// a pipe transforms the TEMPERATURE of what passes through it (ambient
/// exchange) but not its composition — heat crosses the wall, mass does not. So
/// there is no inlet/outlet distinction to make and `node` is not needed: both
/// ends see the upwind node's composition.
///
/// # Errors
/// `SimError::Numerical` if the upwind node's composition is unresolved —
/// returned rather than indexed so a sweep-ordering bug cannot panic (rule 5).
pub fn edge_composition_at<'a>(
    graph: &PlantGraph,
    composition: &'a BTreeMap<NodeId, Composition>,
    edge: EdgeId,
    mass_flow: f64,
) -> Result<&'a Composition, SimError> {
    let upwind = upwind_end(graph, edge, mass_flow);
    composition.get(&upwind).ok_or_else(|| {
        SimError::Numerical(format!(
            "internal: upwind node '{}' of pipe '{}' unresolved during the composition sweep",
            graph.node(upwind).name,
            graph.pipe(edge).name
        ))
    })
}

/// The temperature at which fluid crosses `node`'s boundary along `edge` [K].
///
/// The single owner of "which end of this edge am I looking at". Once a pipe
/// can exchange heat with ambient it has TWO temperatures, and every reader has
/// to resolve the same question — is this node the pipe's inlet or its outlet? —
/// with the same answer. Two call sites applying that rule independently is the
/// mistake the `HeatExchanger` note rejected in its "two independently computed
/// effectiveness terms" form: two copies of a rule that must agree, with nothing
/// forcing them to.
///
/// - `node` is the UPWIND end: the fluid leaves at the node's own temperature,
///   before the pipe has done anything to it. No transform.
/// - `node` is the DOWNSTREAM end: the fluid arrives at the transformed outlet.
///
/// Both readers pass every incident edge through here regardless of direction,
/// which is what makes the difference between the two ends land on the pipe
/// rather than on a node. Debiting a tank at its outflow pipe's OUTLET
/// temperature would charge the tank for heat the pipe traded with ambient
/// downstream of it — invisible while `UA = 0`, and a silent enthalpy error the
/// moment one pipe gets a real value.
///
/// With `UA = 0` both branches return the same number, which is why this change
/// is bit-identical to its predecessor by construction rather than by
/// measurement.
///
/// # Errors
/// `SimError::Numerical` if the upwind node's temperature is not yet resolved —
/// returned rather than indexed so a sweep-ordering bug cannot panic (rule 5).
pub fn edge_temperature_at(
    graph: &PlantGraph,
    slate: &Slate,
    temperature: &BTreeMap<NodeId, Kelvin>,
    composition: &BTreeMap<NodeId, Composition>,
    edge: EdgeId,
    mass_flow: f64,
    node: NodeId,
) -> Result<Kelvin, SimError> {
    let upwind = upwind_end(graph, edge, mass_flow);
    let inlet = temperature.get(&upwind).copied().ok_or_else(|| {
        SimError::Numerical(format!(
            "internal: upwind node '{}' of pipe '{}' unresolved during the temperature sweep",
            graph.node(upwind).name,
            graph.pipe(edge).name
        ))
    })?;
    if node == upwind {
        return Ok(inlet);
    }
    let pipe = graph.pipe(edge);
    // The cp of what is IN the pipe this tick — the upwind node's resolved
    // composition, not the pipe's stored copy of last tick's. See
    // `stream_cp_at` for why the difference is not cosmetic.
    let cp = stream_cp_at(graph, slate, composition, edge, mass_flow)?;
    Ok(pipe_outlet_temperature(
        inlet,
        pipe.ambient_ua,
        KgPerSec(mass_flow),
        cp,
    ))
}

/// The specific heat of the fluid crossing `edge` this tick [J/(kg·K)].
///
/// Off the RESOLVED upwind composition, and deliberately not off
/// `pipe.stream.composition`. The stored copy is written at the end of a tick,
/// so every reader during the tick would see the PREVIOUS one's — and a stream
/// that has just changed composition would be charged the heat capacity of the
/// fluid it replaced.
///
/// That was shipped as an accepted one-tick lag on the grounds that no gate
/// could falsify it, which was true only while every multi-component test was
/// isothermal. It is not a bounded transient: a tank fed a cut whose cp differs
/// from the pipe's stale one books the wrong enthalpy on the very first tick,
/// and `a_tank_changing_composition_while_heating_lands_on_its_new_heat_capacity`
/// is the reference that says so.
///
/// Bit-identical on a one-component slate, where every composition is `[1.0]`
/// and both sources of cp agree by construction — which is what keeps every
/// M1/M2 golden unchanged.
///
/// # Errors
/// `SimError::Numerical` if the upwind node's composition is unresolved.
pub fn stream_cp_at(
    graph: &PlantGraph,
    slate: &Slate,
    composition: &BTreeMap<NodeId, Composition>,
    edge: EdgeId,
    mass_flow: f64,
) -> Result<JPerKgK, SimError> {
    Ok(edge_composition_at(graph, composition, edge, mass_flow)?.mixture_cp(slate))
}

/// Total heat delivered into a node [W]: external heat, plus the operating duty
/// of a fired heater, minus that of a cooler, plus exchange with ambient.
///
/// This function is the single owner of the duty **sign convention**. Both
/// `Furnace` and `Cooler` store `duty` as a non-negative magnitude — "how much
/// heat this unit moves" — and which direction it moves is a property of the
/// unit, applied here. Nothing else in the engine needs to know.
///
/// The terms are separate fields and SUM here rather than sharing storage.
/// `heat_input` is the damage model's hook — `Command::SetHeatInput` sets it —
/// so folding a unit's duty into it would make a fire silently overwrite the
/// operator's setpoint instead of stacking on top of it. Every consumer of "how
/// much heat enters this node" goes through this function, so the two can never
/// drift apart. On a cooler that same sum gives the physically right answer for
/// free: a fire fights the cooling rather than replacing it.
///
/// A TANK's ambient exchange joins the same sum. It belongs here for the reason
/// the duty does — every consumer already asks this function how much heat
/// enters a node, so the tank's energy integration in `Engine::tick` picks the
/// term up without knowing it exists. Only a tank has one: an ambient boundary
/// needs a body with thermal mass and a temperature of its OWN, and a
/// zero-volume node has neither. A pipe does have one, but its outlet is not
/// its inlet plus `Q/(ṁ·cp)` — it needs the analytic plug-flow transform, which
/// is its own change to how transport works (docs/DESIGN.md §4a).
pub fn heat_load(node: &crate::graph::Node) -> Watt {
    let unit_term = match &node.kind {
        NodeKind::Furnace { duty } => duty.value(),
        NodeKind::Cooler { duty } => -duty.value(),
        // The tank's temperature is its START-of-tick value here, which is what
        // makes this an explicit-Euler term like every other slow state.
        NodeKind::Tank(tank) => ambient_exchange(tank.ambient_ua, tank.temperature).value(),
        _ => 0.0,
    };
    Watt(node.heat_input.value() + unit_term)
}

/// Accept a computed temperature [K] only if physics permits it, or fail with
/// `context` explaining what produced the impossible value.
///
/// This is the single owner of "an absolute temperature below zero is an
/// error", for every path that computes one. There is more than one: a
/// zero-volume node mixes its inflows, a tank integrates its thermal
/// inventory, and each can be driven sub-zero by a large enough net heat
/// SINK. The result is FINITE in both cases, so the engine's NaN/Inf checks
/// never see it — without this guard the plant runs on a number physics
/// forbids, propagating downstream as an ordinary temperature.
///
/// Deliberately stated as "any net heat sink", not `if cooler` or
/// `if heat_input < 0`: a negative absolute temperature is broken whatever
/// produced it, and the callers owe nothing to which lever got them there.
///
/// Today exactly one lever reaches it — a cooler's duty, on the mixing path.
/// Nothing reaches the TANK path: `Command::SetHeatInput` refuses a negative
/// fire, a tank carries no duty of its own, and mixing cannot fall below its
/// coldest inflow. The tank guard is therefore cover held in advance, for the
/// ambient exchange that will put a signed Q straight onto that balance. That
/// is the point of a shared checker: the new term arrives already guarded,
/// instead of reopening this on whichever path is newest.
///
/// Err, never clamp. Clamping would report a plausible 0 K instead of the
/// temperature asked for, which is exactly the silently-wrong answer this
/// project treats as worse than a crash.
///
/// # Errors
/// `SimError::Numerical` if `value` is below absolute zero.
pub fn checked_temperature(
    value: f64,
    context: impl FnOnce() -> String,
) -> Result<Kelvin, SimError> {
    if value < 0.0 {
        return Err(SimError::Numerical(context()));
    }
    Ok(Kelvin(value))
}

/// True for nodes with no inventory, whose temperature is an instantaneous
/// mix of their inflows rather than a state (DESIGN §4a).
///
/// Pumps, valves, furnaces and coolers are zero-volume *pass-throughs*: with
/// exactly one inlet and one outlet (enforced by `validate_degrees`) the mixing
/// formula degenerates to "outlet temperature = inlet temperature", plus
/// whatever heat `heat_load` adds. Pump work and valve throttling both dissipate into the
/// stream as heat; at M2 fidelity that rise is neglected (~0.02 K for the
/// reference pump — far below the model's accuracy) and is a documented
/// DESIGN §4a limitation, not an oversight.
pub fn is_zero_volume(kind: &NodeKind) -> bool {
    matches!(
        kind,
        NodeKind::Junction
            | NodeKind::Pump { .. }
            | NodeKind::Valve { .. }
            | NodeKind::Furnace { .. }
            | NodeKind::Cooler { .. }
            | NodeKind::HeatExchanger
    )
}

/// Start-of-tick composition of an inertial node, or `None` for a zero-volume
/// node (whose composition must be mixed from its inflows instead).
///
/// The exact partition `boundary_temperature` makes, and it must stay exact:
/// the sweep seeds one field and mixes the other from the same `Some`/`None`
/// answer, so a node inertial in temperature and zero-volume in composition
/// would be swept in an order its own dependencies do not justify.
///
/// `Atmosphere` is the one node with no composition to state. It is given the
/// first slate component — arbitrary, and honestly so: nothing in M3.1 draws
/// mass out of an Atmosphere node (leak edges run *into* it), so no gate can
/// falsify this choice. It is a compile-time default standing in for a decision
/// the milestone that back-feeds from a leak will have to make properly.
pub fn boundary_composition(kind: &NodeKind, slate: &Slate) -> Option<Composition> {
    match kind {
        NodeKind::Source { composition, .. } => Some(composition.clone()),
        NodeKind::Sink { composition, .. } => Some(composition.clone()),
        NodeKind::Atmosphere => Some(Composition::pure(slate.len(), 0)),
        NodeKind::Tank(tank) => Some(tank.composition.clone()),
        NodeKind::Junction
        | NodeKind::Pump { .. }
        | NodeKind::Valve { .. }
        | NodeKind::Furnace { .. }
        | NodeKind::Cooler { .. }
        | NodeKind::HeatExchanger => None,
    }
}

/// Start-of-tick temperature of an inertial node, or `None` for a zero-volume
/// node (whose temperature must be mixed from its inflows instead).
///
/// `Atmosphere` is pinned to `T_AMBIENT`, symmetric with its pressure being
/// pinned to `P_ATM`: it is the outside world, not a modeled inventory.
pub fn boundary_temperature(kind: &NodeKind) -> Option<Kelvin> {
    match kind {
        NodeKind::Source { temperature, .. } => Some(*temperature),
        NodeKind::Sink { temperature, .. } => Some(*temperature),
        NodeKind::Atmosphere => Some(T_AMBIENT),
        NodeKind::Tank(tank) => Some(tank.temperature),
        // Furnaces and coolers belong here, with the other zero-volume nodes:
        // `None` is what makes the sweep MIX their inflows and apply
        // `heat_load`. Returning `Some(..)` would compile and quietly make one
        // inertial — its duty would never reach the stream.
        NodeKind::Junction
        | NodeKind::Pump { .. }
        | NodeKind::Valve { .. }
        | NodeKind::Furnace { .. }
        | NodeKind::Cooler { .. }
        | NodeKind::HeatExchanger => None,
    }
}

/// Edges carrying flow *into* `node`, as `(edge, upstream node, ṁ into node)`.
///
/// The single definition of "inflow" — the dependency count and the mixing sum
/// both call it, so they cannot drift apart and leave the topological sort
/// waiting on an edge the mix never reads (or vice versa).
///
/// Self-loops are skipped: a pipe from a node to itself transports nothing
/// anywhere, and admitting it would make the node its own upstream dependency.
fn inflow_edges(
    graph: &PlantGraph,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    node: NodeId,
) -> Vec<(EdgeId, NodeId, f64)> {
    let mut inflows = Vec::new();
    // `incident` is sorted by edge id, so the sum order below is deterministic.
    for (edge, other, incoming) in graph.incident(node) {
        if other == node {
            continue;
        }
        let flow = edge_mass_flow.get(&edge).copied().unwrap_or(0.0);
        let into_node = if incoming { flow } else { -flow };
        if into_node > 0.0 {
            inflows.push((edge, other, into_node));
        }
    }
    inflows
}

/// Every node's resolved intensive state for one tick.
///
/// Temperature and composition are resolved TOGETHER, on one sweep, because
/// they are transported by the same topology: the same inflow edges, the same
/// upwind rule, the same Kahn ordering, the same recycle rejection. A second
/// sweep computing composition alongside would be a duplicate of all four, free
/// to drift.
///
/// They are nonetheless mixed with DIFFERENT weights, which is the whole subtle
/// point of this pair — see `mix_compositions`.
#[derive(Debug, Clone, Default)]
pub struct NodeStates {
    /// Resolved temperature [K] per node.
    pub temperature: BTreeMap<NodeId, Kelvin>,
    /// Resolved mass-fraction composition per node.
    pub composition: BTreeMap<NodeId, Composition>,
}

/// Resolve every node's temperature [K] and composition for this tick.
///
/// Inertial nodes seed the field with their start-of-tick temperature;
/// zero-volume nodes are then swept in flow order, each mixing its inflows:
///
/// ```text
/// T_mix = T_REF + Σ(ṁ_in·cp_in·(T_in − T_REF)) / Σ(ṁ_in·cp_in)
/// ```
///
/// which is the enthalpy balance of a zero-volume mixing point (first law with
/// no accumulation and no work), reducing to the mass-weighted mean when every
/// stream shares a `cp`.
///
/// and composition alongside it, by mass alone:
///
/// ```text
/// f_c = Σ(ṁ_in · f_in,c) / Σ ṁ_in
/// ```
///
/// `previous` supplies the fallback for a zero-volume node with **no inflow**:
/// nothing enters it, so its temperature is physically indeterminate — and it
/// carries no enthalpy either way, since mass balance forces zero outflow too.
/// Holding the last resolved value keeps the field finite and reproducible
/// instead of dividing 0/0.
///
/// # Errors
/// `SimError::Numerical` if a recycle among zero-volume nodes leaves the sweep
/// with no valid order (see the module docs).
pub fn resolve_node_states(
    graph: &PlantGraph,
    slate: &Slate,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    previous: &NodeStates,
) -> Result<NodeStates, SimError> {
    let mut temperature: BTreeMap<NodeId, Kelvin> = BTreeMap::new();
    let mut composition: BTreeMap<NodeId, Composition> = BTreeMap::new();

    // 1. Inertial nodes are the roots of the sweep: known before it starts.
    //    Both fields are seeded from the SAME node, so the two boundary helpers
    //    agreeing on which nodes are inertial is load-bearing — asserted here
    //    rather than assumed, because a mismatch would leave a node seeded in
    //    one field and swept in the other, and the sweep would then read an
    //    unresolved upwind and error somewhere far from the cause.
    let mut zero_volume: Vec<NodeId> = Vec::new();
    for id in graph.node_ids() {
        let kind = &graph.node(id).kind;
        match (
            boundary_temperature(kind),
            boundary_composition(kind, slate),
        ) {
            (Some(t), Some(c)) => {
                temperature.insert(id, t);
                composition.insert(id, c);
            }
            (None, None) => zero_volume.push(id),
            _ => {
                return Err(SimError::Numerical(format!(
                    "internal: node '{}' is inertial in one intensive field and \
                     zero-volume in the other — `boundary_temperature` and \
                     `boundary_composition` must partition the node kinds identically",
                    graph.node(id).name
                )))
            }
        }
    }

    // 2. Group the zero-volume nodes into sweep VERTICES. Almost every vertex
    //    is a single node; a heat exchanger's two sides are ONE vertex.
    //
    //    This merge is the whole reason the sweep is not simply per-node. Each
    //    side's outlet depends on the OTHER side's inlet, which is not one of
    //    its own inflow edges — so a per-node sweep would happily mark side A
    //    ready as soon as A's own upstreams cleared and mix it against a stale
    //    partner inlet. That failure converges, serializes and reruns
    //    identically; nothing downstream looks wrong. Merged, a side becomes
    //    ready only when the UNION of both sides' upstreams is resolved, which
    //    is exactly the condition under which both inlets are known.
    //
    //    A vertex is keyed by its LEADER, the lower of the two node ids, so the
    //    key is a function of the graph alone (rule 3).
    let leader_of = |id: NodeId| -> NodeId {
        match graph.exchanger_partner(id) {
            Some((partner, _)) => id.min(partner),
            None => id,
        }
    };
    let mut members: BTreeMap<NodeId, Vec<NodeId>> = BTreeMap::new();
    for &id in &zero_volume {
        members.entry(leader_of(id)).or_default().push(id);
    }

    // 3. Count each vertex's unresolved upstream dependencies. Only zero-volume
    //    upstreams count — an inertial upstream is already known.
    //
    //    A dependency on our OWN vertex IS counted here, and the release step
    //    below deliberately never clears it. That asymmetry is the cycle
    //    detector: one exchanger side feeding the other is a genuine circular
    //    definition — A's outlet depends on B's inlet, which is A's outlet — so
    //    the vertex must never become ready and must fall into the step-5
    //    recycle error. Discounting the self-dependency instead would make the
    //    vertex ready immediately and resolve it against a partner inlet that
    //    does not exist yet.
    let mut pending: BTreeMap<NodeId, usize> = BTreeMap::new();
    for (&leader, sides) in &members {
        let deps = sides
            .iter()
            .flat_map(|&id| inflow_edges(graph, edge_mass_flow, id))
            .filter(|(_, upstream, _)| is_zero_volume(&graph.node(*upstream).kind))
            .count();
        pending.insert(leader, deps);
    }

    // 4. Kahn sweep. BTreeSet, always popping the lowest id: the order is then
    //    a function of the graph alone, never of insertion history (rule 3).
    //    Any valid topological order yields identical temperatures anyway —
    //    each mix reads only already-resolved upstreams — but a deterministic
    //    order keeps the float summation order fixed too.
    let mut ready: BTreeSet<NodeId> = members
        .keys()
        .copied()
        .filter(|leader| pending[leader] == 0)
        .collect();

    let mut resolved = 0usize;
    while let Some(&leader) = ready.iter().next() {
        ready.remove(&leader);
        let sides = &members[&leader];

        // Composition first, and uniformly across both branches: it is settled
        // by mass alone, so an exchanger's two sides are ordinary mixing points
        // to it. Heat crosses between them; mass does not, and a pair whose
        // compositions influenced each other would be modelling a leak.
        for &id in sides {
            let mixed = mix_compositions(graph, slate, edge_mass_flow, &composition, previous, id)?;
            composition.insert(id, mixed);
        }

        match sides.as_slice() {
            [a, b] => {
                // Both outlets from one signed Q, computed from both inlets.
                let (t_a, t_b) = exchange_pair(
                    graph,
                    slate,
                    edge_mass_flow,
                    &temperature,
                    &composition,
                    &previous.temperature,
                    (*a, *b),
                )?;
                temperature.insert(*a, t_a);
                temperature.insert(*b, t_b);
            }
            _ => {
                for &id in sides {
                    let mixed = mix_inflows(
                        graph,
                        slate,
                        edge_mass_flow,
                        &temperature,
                        &composition,
                        &previous.temperature,
                        id,
                    )?;
                    temperature.insert(id, mixed);
                }
            }
        }
        resolved += sides.len();

        // Release the downstream vertices this one feeds. An edge is an inflow
        // to the far node exactly when it is an outflow here: the two views
        // share one `flow`, so `into_other == -into_id` identically. Counting
        // per EDGE (not per node) mirrors `pending`, which counted inflow edges
        // — parallel pipes decrement once each, as they should.
        for &id in sides {
            for (edge, downstream, incoming) in graph.incident(id) {
                if downstream == id || !is_zero_volume(&graph.node(downstream).kind) {
                    continue;
                }
                let downstream_leader = leader_of(downstream);
                if downstream_leader == leader {
                    continue; // self-dependency: counted, never cleared (step 3)
                }
                let flow = edge_mass_flow.get(&edge).copied().unwrap_or(0.0);
                let into_id = if incoming { flow } else { -flow };
                if into_id >= 0.0 {
                    continue; // not an outflow ⇒ not an inflow to `downstream`
                }
                if let Some(count) = pending.get_mut(&downstream_leader) {
                    *count = count.saturating_sub(1);
                    if *count == 0 {
                        ready.insert(downstream_leader);
                    }
                }
            }
        }
    }

    // 5. A node left unresolved means its dependencies never cleared: the only
    //    way that happens is a cycle among zero-volume vertices — which now
    //    includes one side of an exchanger feeding the other, correctly, since
    //    that plant's two outlets really are mutually defined.
    if resolved < zero_volume.len() {
        let stuck: Vec<String> = zero_volume
            .iter()
            .filter(|id| !temperature.contains_key(id))
            .map(|id| graph.node(*id).name.clone())
            .collect();
        return Err(SimError::Numerical(format!(
            "recycle through zero-volume nodes only ({}) — their temperatures are \
             mutually dependent with no inertial node to break the loop, which needs \
             a simultaneous solve the M2 upwind sweep does not implement. Put a tank \
             in the loop.",
            stuck.join(", ")
        )));
    }

    Ok(NodeStates {
        temperature,
        composition,
    })
}

/// Mass-weighted mix of a zero-volume node's inflow compositions.
///
/// ```text
/// f_c = Σ(ṁ_in · f_in,c) / Σ ṁ_in
/// ```
///
/// **The weight is `ṁ`, not `ṁ·cp`** — and that distinction is the one thing
/// this function exists to get right. `mix_inflows` sits ten lines away
/// weighting by `ṁ·cp`, because enthalpy is what mixes there; here the
/// conserved quantity is the mass of each component, and cp has nothing to do
/// with it. Reusing the enthalpy weights is the natural mistake and is nearly
/// invisible: it still conserves TOTAL mass and still passes every energy
/// balance, corrupting only the fractions — so it is caught by the
/// per-component invariant (I7) and the blend reference and by nothing else.
///
/// The two mixes stay consistent for free, which is why composition can ride
/// this sweep without perturbing M2's energy math: cp is linear in composition,
/// so `Σ_c (Σ_s ṁ_s·f_sc)·cp_c = Σ_s ṁ_s·cp_s` — the mixed stream's capacity
/// rate is exactly the sum of its inflows'.
///
/// # Errors
/// `SimError::Numerical` if an upwind composition is unresolved, or if the
/// accumulated weights are not a valid composition.
fn mix_compositions(
    graph: &PlantGraph,
    slate: &Slate,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    composition: &BTreeMap<NodeId, Composition>,
    previous: &NodeStates,
    node: NodeId,
) -> Result<Composition, SimError> {
    // Accumulated in `inflow_edges` order, which is edge-id order — so the
    // float summation order is a function of the graph alone (rule 3).
    let mut weights = vec![0.0; slate.len()];
    let mut total_inflow = 0.0;

    for (edge, _upstream, into_node) in inflow_edges(graph, edge_mass_flow, node) {
        // The raw stored flow, not `into_node`: `upwind_end` picks the end from
        // the SIGN, and `inflow_edges` has already re-signed `into_node`
        // positive-into-this-node, which would name the wrong end on every edge
        // stored pointing inward. Same trap as the temperature path.
        let flow = edge_mass_flow.get(&edge).copied().unwrap_or(0.0);
        let incoming = edge_composition_at(graph, composition, edge, flow)?;
        for (weight, fraction) in weights.iter_mut().zip(incoming.fractions()) {
            *weight += into_node * fraction;
        }
        total_inflow += into_node;
    }

    if total_inflow > 0.0 {
        Composition::from_weights(&weights).map_err(|e| {
            SimError::Numerical(format!(
                "mixing the inflows of '{}' produced no valid composition: {e}",
                graph.node(node).name
            ))
        })
    } else {
        // No inflow: indeterminate but inert, mirroring `mix_inflows`' rule for
        // the same case exactly — mass balance forces zero outflow too, so
        // whatever is held here is carried nowhere. Holding the last resolved
        // value keeps the field reproducible; `Composition::from_weights` errs
        // on an all-zero weight vector, so mixing through it is not an option.
        //
        // With no history at all, the first slate component: the composition
        // counterpart of the temperature path's fallback to ambient, and inert
        // for the same reason.
        Ok(previous
            .composition
            .get(&node)
            .cloned()
            .unwrap_or_else(|| Composition::pure(slate.len(), 0)))
    }
}

/// A node's inflow enthalpy [W] and capacity rate [W/K], or `None` when nothing
/// flows in.
///
/// The single definition of both sums. `mix_inflows` divides them to get a
/// mixed temperature; the exchanger needs the capacity rate itself, to size
/// `C_min`. Computing them in one place keeps the inlet temperature the
/// exchanger transfers heat *from* identical to the one an uncoupled node would
/// have mixed to.
type InflowTotals = Option<(f64, f64)>;

fn inflow_totals(
    graph: &PlantGraph,
    slate: &Slate,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    temperature: &BTreeMap<NodeId, Kelvin>,
    composition: &BTreeMap<NodeId, Composition>,
    node: NodeId,
) -> Result<InflowTotals, SimError> {
    let mut enthalpy = 0.0; // Σ ṁ·cp·(T − T_REF) [W]
    let mut capacity = 0.0; // Σ ṁ·cp [W/K]

    for (edge, _upstream, into_node) in inflow_edges(graph, edge_mass_flow, node) {
        // The transformed OUTLET, not the raw upwind node temperature: with a
        // nonzero pipe `UA` those differ, and mixing the raw value would apply
        // the transform after the mixing that consumes it — i.e. never.
        //
        // `edge_mass_flow`, not `into_node`: the helper picks the upwind end
        // from the flow's SIGN, and `into_node` has already been re-signed
        // positive-into-this-node by `inflow_edges`. Feeding it that would name
        // the wrong end on every edge whose stored direction runs into `node`.
        // (This is the same class of error as passing a signed `ṁ` to the
        // transform, and it is why the helper takes the raw flow.)
        //
        // Resolution is guaranteed by construction — the sweep visits a node
        // only once every zero-volume upstream is done, and inertial ones are
        // seeded — but the helper returns an error rather than indexing, so a
        // sort bug can never panic (rule 5).
        let flow = edge_mass_flow.get(&edge).copied().unwrap_or(0.0);
        let inlet_t =
            edge_temperature_at(graph, slate, temperature, composition, edge, flow, node)?;
        // The same resolved-upwind cp the transform above used, through the same
        // helper: an inlet transformed at one heat capacity and mixed at another
        // would not conserve enthalpy across the pipe.
        let cp = stream_cp_at(graph, slate, composition, edge, flow)?;
        enthalpy += enthalpy_flux(KgPerSec(into_node), cp, inlet_t).value();
        capacity += into_node * cp.value();
    }

    Ok((capacity > 0.0).then_some((enthalpy, capacity)))
}

/// Enthalpy-weighted mix of a zero-volume node's inflows [K].
fn mix_inflows(
    graph: &PlantGraph,
    slate: &Slate,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    temperature: &BTreeMap<NodeId, Kelvin>,
    composition: &BTreeMap<NodeId, Composition>,
    previous: &BTreeMap<NodeId, Kelvin>,
    node: NodeId,
) -> Result<Kelvin, SimError> {
    // Heat — external (a fire) and a furnace's duty alike — joins the same
    // first law: for a node with no accumulation, Σ ṁ·h_in + Q = Σ ṁ·h_out.
    // Without this term `Command::SetHeatInput` would be a silent no-op on
    // every junction (the damage model's "fire on a node" would do nothing at
    // all) and a furnace would be an inert pass-through.
    let heat_input = heat_load(graph.node(node)).value();

    if let Some((enthalpy, capacity)) =
        inflow_totals(graph, slate, edge_mass_flow, temperature, composition, node)?
    {
        let mixed = T_REF.value() + (enthalpy + heat_input) / capacity;
        // A duty that exceeds the sensible heat available in the stream drives
        // the mix below absolute zero; `checked_temperature` owns that rule for
        // this path and the tank's alike (see its docs for why it Errs).
        checked_temperature(mixed, || {
            format!(
                "'{}' cools to {mixed:.2} K, below absolute zero: net heat load \
                 {heat_input:.4e} W exceeds the {capacity:.4e} W/K · {:.2} K of \
                 sensible heat its inflow carries above 0 K. Reduce the duty or \
                 raise the flow through it.",
                graph.node(node).name,
                enthalpy / capacity + T_REF.value(),
            )
        })
    } else {
        // No inflow: indeterminate but inert (mass balance ⇒ no outflow either).
        //
        // KNOWN LIMITATION: any `heat_input` here is dropped. A zero-volume node
        // has no thermal mass, so with no throughput there is nothing for the
        // heat to raise — the honest model of a fire against stagnant inventory
        // puts it on a Tank. Energy is therefore NOT conserved in this one case,
        // which is why `energy_invariants.rs` heats only tanks: it is a gap in
        // the model, not slack the invariant should be widened to tolerate.
        Ok(previous.get(&node).copied().unwrap_or(T_AMBIENT))
    }
}

/// Both outlet temperatures of a coupled `HeatExchanger` pair [K].
///
/// ΔT-effectiveness fidelity (docs/DESIGN.md §4a) — no NTU, no LMTD, no
/// counter- versus co-current distinction at this level:
///
/// ```text
/// C_a = ṁ_a·cp_a,  C_b = ṁ_b·cp_b,  C_min = min(C_a, C_b)
/// Q   = ε·C_min·(T_a_in − T_b_in)
/// T_a_out = T_a_in − Q/C_a        T_b_out = T_b_in + Q/C_b
/// ```
///
/// Three properties this arrangement buys, each deliberate:
///
/// - **One signed `Q`, subtracted from A and added to B.** Energy conserves by
///   construction, for any ε and any pair of capacity rates — there is no
///   balance left to get wrong. Computing each side's outlet from its own
///   independent effectiveness term is the natural-looking alternative and
///   quietly creates or destroys heat.
/// - **Neither side is the hot one.** The sign of `T_a_in − T_b_in` decides the
///   direction, so a service that reverses needs no reconfiguration and no
///   second code path.
/// - **`C_min`, not `C_max`.** With ε ≤ 1 this bounds `Q` by the heat the
///   smaller stream can actually carry, so the outlets cannot cross and the
///   second law holds without a check. The two agree exactly when the capacity
///   rates are equal, which is why the reference case makes them unequal.
///
/// A side with no throughput exchanges nothing: `Q` is zero and each side falls
/// back to the ordinary zero-volume rules. That is physics, not a guard — an
/// exchanger with one stream stopped is a pipe.
fn exchange_pair(
    graph: &PlantGraph,
    slate: &Slate,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    temperature: &BTreeMap<NodeId, Kelvin>,
    composition: &BTreeMap<NodeId, Composition>,
    previous: &BTreeMap<NodeId, Kelvin>,
    (side_a, side_b): (NodeId, NodeId),
) -> Result<(Kelvin, Kelvin), SimError> {
    let (_, effectiveness) = graph.exchanger_partner(side_a).ok_or_else(|| {
        SimError::Numerical(format!(
            "internal: '{}' was swept as an exchanger side but has no coupling",
            graph.node(side_a).name
        ))
    })?;

    let totals_a = inflow_totals(
        graph,
        slate,
        edge_mass_flow,
        temperature,
        composition,
        side_a,
    )?;
    let totals_b = inflow_totals(
        graph,
        slate,
        edge_mass_flow,
        temperature,
        composition,
        side_b,
    )?;

    // Positive duty = heat flowing A → B.
    let duty = match (totals_a, totals_b) {
        (Some((enthalpy_a, capacity_a)), Some((enthalpy_b, capacity_b))) => {
            let inlet_a = T_REF.value() + enthalpy_a / capacity_a;
            let inlet_b = T_REF.value() + enthalpy_b / capacity_b;
            effectiveness * capacity_a.min(capacity_b) * (inlet_a - inlet_b)
        }
        _ => 0.0,
    };

    Ok((
        exchanger_side_outlet(graph, previous, side_a, totals_a, -duty)?,
        exchanger_side_outlet(graph, previous, side_b, totals_b, duty)?,
    ))
}

/// One exchanger side's outlet [K]: its own inflow mix, plus whatever heat it
/// receives — `transferred` from the partner stream, and `heat_load` from a
/// fire, which stacks here exactly as it does on a furnace.
fn exchanger_side_outlet(
    graph: &PlantGraph,
    previous: &BTreeMap<NodeId, Kelvin>,
    node: NodeId,
    totals: InflowTotals,
    transferred: f64,
) -> Result<Kelvin, SimError> {
    let Some((enthalpy, capacity)) = totals else {
        // Same indeterminate-but-inert case `mix_inflows` documents.
        return Ok(previous.get(&node).copied().unwrap_or(T_AMBIENT));
    };
    let heat = heat_load(graph.node(node)).value() + transferred;
    let outlet = T_REF.value() + (enthalpy + heat) / capacity;
    checked_temperature(outlet, || {
        format!(
            "exchanger side '{}' cools to {outlet:.2} K, below absolute zero: it \
             gives up {:.4e} W to its partner stream (plus {:.4e} W of external \
             heat), more than the {capacity:.4e} W/K · {:.2} K of sensible heat \
             its inflow carries above 0 K.",
            graph.node(node).name,
            -transferred,
            heat_load(graph.node(node)).value(),
            enthalpy / capacity + T_REF.value(),
        )
    })
}

#[cfg(test)]
mod tests {
    //! The sweep is tested with HAND-BUILT flow maps, never through a solver.
    //! Its contract is "given these flows, produce these temperatures", so
    //! feeding it flows directly tests exactly that — and lets the recycle case
    //! be built on demand instead of hoping a network happens to circulate.

    use super::*;
    use crate::components::{Composition, Slate};
    use crate::graph::{HeatExchangerCoupling, Node, Pipe, TankState};
    use crate::stream::Stream;
    use crate::units::*;

    fn node(name: &str, kind: NodeKind) -> Node {
        Node {
            name: name.into(),
            kind,
            heat_input: Watt::ZERO,
        }
    }

    fn source(name: &str, temperature: Kelvin) -> Node {
        node(
            name,
            NodeKind::Source {
                pressure: Pascal(2.0e5),
                temperature,
                composition: Composition::pure(1, 0),
            },
        )
    }

    fn tank(name: &str, temperature: Kelvin) -> Node {
        node(
            name,
            NodeKind::Tank(TankState {
                area: SquareMeter(10.0),
                height: Meter(10.0),
                mass: Kg(50_000.0),
                temperature,
                composition: Composition::pure(1, 0),
                ambient_ua: WattPerKelvin::ZERO,
            }),
        )
    }

    fn pipe(name: &str) -> Pipe {
        Pipe {
            name: name.into(),
            length: Meter(10.0),
            diameter: Meter(0.1),
            friction_factor: 0.02,
            elevation_change: Meter(0.0),
            leak_area: SquareMeter::ZERO,
            ambient_ua: WattPerKelvin::ZERO,
            stream: Stream::stagnant(1, T_AMBIENT, P_ATM),
        }
    }

    fn resolve(
        graph: &PlantGraph,
        flows: &BTreeMap<EdgeId, f64>,
    ) -> Result<BTreeMap<NodeId, Kelvin>, SimError> {
        resolve_node_states(graph, &Slate::water_only(), flows, &NodeStates::default())
            .map(|states| states.temperature)
    }

    /// The composition half of the same sweep, for the tests that read it.
    fn resolve_compositions(
        graph: &PlantGraph,
        slate: &Slate,
        flows: &BTreeMap<EdgeId, f64>,
    ) -> Result<BTreeMap<NodeId, Composition>, SimError> {
        resolve_node_states(graph, slate, flows, &NodeStates::default())
            .map(|states| states.composition)
    }

    /// The composition sweep, on a slate whose two cuts have DELIBERATELY
    /// mismatched `cp` — the property that separates a mass-weighted mix from a
    /// capacity-rate-weighted one.
    mod composition_mixing {
        use super::*;
        use crate::components::PseudoComponent;

        fn cut(name: &str, cp: f64) -> PseudoComponent {
            PseudoComponent {
                name: name.into(),
                tb: Kelvin(400.0),
                molar_mass: KgPerMol(0.1),
                density: KgPerM3(800.0),
                cp: JPerKgK(cp),
            }
        }

        /// cp ratio 4:1. Every other property is equal, so nothing but the
        /// weighting rule can move the answer.
        fn two_cuts() -> Slate {
            Slate::new(vec![cut("light", 1000.0), cut("heavy", 4000.0)]).unwrap()
        }

        fn feed(name: &str, fractions: &[f64]) -> Node {
            node(
                name,
                NodeKind::Source {
                    pressure: Pascal(2.0e5),
                    temperature: Kelvin(300.0),
                    composition: Composition::from_weights(fractions).unwrap(),
                },
            )
        }

        /// REFERENCE — the blend, predicted from the MASS RATIO by hand.
        ///
        /// A junction is fed 1 kg/s of pure `light` and 3 kg/s of pure `heavy`.
        /// Per-component mass balance on a vessel with no accumulation:
        ///
        ///   light: 1 kg/s in, of 4 kg/s total  ⇒  f_light = 0.25
        ///   heavy: 3 kg/s in, of 4 kg/s total  ⇒  f_heavy = 0.75
        ///
        /// stated from the flows alone, with no reference to `mix_compositions`
        /// or to `Composition::blend` — the expected numbers cannot be produced
        /// by the same rule they are checking.
        ///
        /// THE MUTATION THIS EXISTS FOR: weight the mix by `ṁ·cp` instead of
        /// `ṁ`, i.e. reuse the enthalpy weights sitting in the same sweep. The
        /// capacity rates here are 1·1000 and 3·4000, so that mutation returns
        /// f_light = 1000/13000 ≈ 0.0769 rather than 0.25. It is a mutation
        /// worth guarding at this size because it is nearly invisible
        /// elsewhere: total mass is still 4 kg/s, every energy balance still
        /// closes, and only the fractions are wrong.
        ///
        /// The 4:1 cp ratio is what gives the two answers room to differ. With
        /// equal cp they coincide exactly and this test would pass under the
        /// mutation — which is the whole reason the slate above is built by
        /// hand instead of reusing water.
        #[test]
        fn a_junction_blends_its_inflows_by_mass_not_by_capacity_rate() {
            let slate = two_cuts();
            let mut g = PlantGraph::new();
            let light_src = g.add_node(feed("light_src", &[1.0, 0.0]));
            let heavy_src = g.add_node(feed("heavy_src", &[0.0, 1.0]));
            let mix = g.add_node(node("mix", NodeKind::Junction));
            let e_light = g.add_pipe(light_src, mix, pipe("light_line"));
            let e_heavy = g.add_pipe(heavy_src, mix, pipe("heavy_line"));

            let flows = BTreeMap::from([(e_light, 1.0), (e_heavy, 3.0)]);
            let composition = resolve_compositions(&g, &slate, &flows).unwrap();

            let mixed = composition[&mix].fractions();
            assert!(
                (mixed[0] - 0.25).abs() < 1e-12 && (mixed[1] - 0.75).abs() < 1e-12,
                "1 kg/s light + 3 kg/s heavy is 25/75 by MASS; got {mixed:?}. The \
                 capacity-rate weighting this guards against gives ~[0.077, 0.923]."
            );
        }

        /// The consistency that lets composition ride the energy sweep without
        /// perturbing it: cp is linear in composition, so the mixed stream's
        /// capacity rate equals the sum of its inflows'.
        ///
        ///   Σ_c (Σ_s ṁ_s·f_sc)·cp_c  =  Σ_s ṁ_s·cp_s
        ///   here: 4 kg/s · (0.25·1000 + 0.75·4000) = 13 000 W/K = 1·1000 + 3·4000
        ///
        /// Stated as a test because it is the reason the two mixes can disagree
        /// about their weights and still describe the same stream. If it failed,
        /// M2's energy balances would drift the moment a slate had two cuts.
        #[test]
        fn the_mixed_capacity_rate_is_the_sum_of_the_inflows() {
            let slate = two_cuts();
            let mut g = PlantGraph::new();
            let light_src = g.add_node(feed("light_src", &[1.0, 0.0]));
            let heavy_src = g.add_node(feed("heavy_src", &[0.0, 1.0]));
            let mix = g.add_node(node("mix", NodeKind::Junction));
            let e_light = g.add_pipe(light_src, mix, pipe("light_line"));
            let e_heavy = g.add_pipe(heavy_src, mix, pipe("heavy_line"));

            let flows = BTreeMap::from([(e_light, 1.0), (e_heavy, 3.0)]);
            let composition = resolve_compositions(&g, &slate, &flows).unwrap();

            let mixed_capacity = 4.0 * composition[&mix].mixture_cp(&slate).value();
            let inflow_capacity = 1.0 * 1000.0 + 3.0 * 4000.0;
            assert!(
                (mixed_capacity - inflow_capacity).abs() < 1e-9,
                "{mixed_capacity} W/K out vs {inflow_capacity} W/K in"
            );
        }

        /// Composition follows the FLOW's direction, not the edge's. Both pipes
        /// below are stored pointing away from the junction and carry negative
        /// flow, so the fluid arrives from the far ends — and a sweep reading
        /// `from` unconditionally would take the junction's own (unresolved)
        /// composition as its inlet.
        ///
        /// This is the composition twin of the reverse-flow temperature case,
        /// and it needs its own gate: the two fields read the upwind end through
        /// one helper now, but nothing in the type system stops that from being
        /// re-derived independently later.
        #[test]
        fn reverse_flow_blends_from_the_far_end() {
            let slate = two_cuts();
            let mut g = PlantGraph::new();
            let mix = g.add_node(node("mix", NodeKind::Junction));
            let light_src = g.add_node(feed("light_src", &[1.0, 0.0]));
            let heavy_src = g.add_node(feed("heavy_src", &[0.0, 1.0]));
            // Stored mix → source, flowing source → mix.
            let e_light = g.add_pipe(mix, light_src, pipe("light_line"));
            let e_heavy = g.add_pipe(mix, heavy_src, pipe("heavy_line"));

            let flows = BTreeMap::from([(e_light, -1.0), (e_heavy, -3.0)]);
            let composition = resolve_compositions(&g, &slate, &flows).unwrap();

            let mixed = composition[&mix].fractions();
            assert!(
                (mixed[0] - 0.25).abs() < 1e-12,
                "reverse flow must blend the same 25/75 as forward flow, got {mixed:?}"
            );
        }

        /// A junction with nothing flowing through it holds a composition rather
        /// than dividing 0/0 — the rule `mix_inflows` applies to temperature,
        /// mirrored. `Composition::from_weights` rejects an all-zero weight
        /// vector, so without the explicit fallback this path would ERROR on a
        /// closed valve, which is an ordinary plant state.
        #[test]
        fn a_stagnant_junction_holds_a_valid_composition() {
            let slate = two_cuts();
            let mut g = PlantGraph::new();
            let src = g.add_node(feed("src", &[0.5, 0.5]));
            let idle = g.add_node(node("idle", NodeKind::Junction));
            let dead = g.add_pipe(src, idle, pipe("dead_leg"));

            let flows = BTreeMap::from([(dead, 0.0)]);

            // With history, it holds what it last saw.
            let previous = NodeStates {
                temperature: BTreeMap::new(),
                composition: BTreeMap::from([(
                    idle,
                    Composition::from_weights(&[0.2, 0.8]).unwrap(),
                )]),
            };
            let held = resolve_node_states(&g, &slate, &flows, &previous)
                .unwrap()
                .composition;
            assert_eq!(held[&idle].fractions(), &[0.2, 0.8]);

            // With none, a valid composition rather than an error or a NaN.
            let fresh = resolve_compositions(&g, &slate, &flows).unwrap();
            let sum: f64 = fresh[&idle].fractions().iter().sum();
            assert!((sum - 1.0).abs() < 1e-12, "must still sum to 1");
        }
    }

    /// Enthalpy weighting, hand-calculated: 1 kg/s at 280 K and 3 kg/s at 320 K
    /// mix to (1·280 + 3·320)/4 = 310 K exactly (equal cp ⇒ mass-weighted mean).
    #[test]
    fn a_junction_mixes_its_inflows_by_enthalpy() {
        let mut g = PlantGraph::new();
        let cold = g.add_node(source("cold", Kelvin(280.0)));
        let hot = g.add_node(source("hot", Kelvin(320.0)));
        let mix = g.add_node(node("mix", NodeKind::Junction));
        let out = g.add_node(node(
            "out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));
        let e_cold = g.add_pipe(cold, mix, pipe("cold_line"));
        let e_hot = g.add_pipe(hot, mix, pipe("hot_line"));
        let e_out = g.add_pipe(mix, out, pipe("outlet"));

        let flows = BTreeMap::from([(e_cold, 1.0), (e_hot, 3.0), (e_out, 4.0)]);
        let temperature = resolve(&g, &flows).expect("an acyclic network must resolve");

        assert!(
            (temperature[&mix].value() - 310.0).abs() < 1e-12,
            "expected the 1:3 mix of 280 K and 320 K to be 310 K, got {}",
            temperature[&mix].value()
        );
    }

    /// Upwind is chosen by FLOW SIGN, not by the graph's edge direction: with
    /// the flow reversed, the junction must take the sink's temperature — the
    /// node the pipe points AT. A model that trusted edge direction would read
    /// the source and report 280 K.
    #[test]
    fn upwind_follows_the_flow_not_the_edge_direction() {
        let mut g = PlantGraph::new();
        let src = g.add_node(source("src", Kelvin(280.0)));
        let mix = g.add_node(node("mix", NodeKind::Junction));
        let back = g.add_node(node(
            "back",
            NodeKind::Sink {
                pressure: Pascal(9.0e5),
                temperature: Kelvin(350.0),
                composition: Composition::pure(1, 0),
            },
        ));
        let e_in = g.add_pipe(src, mix, pipe("inlet"));
        let e_out = g.add_pipe(mix, back, pipe("outlet"));

        // Both edges run backwards: the sink back-feeds through `mix` to `src`.
        let flows = BTreeMap::from([(e_in, -2.0), (e_out, -2.0)]);
        let temperature = resolve(&g, &flows).expect("an acyclic network must resolve");

        assert!(
            (temperature[&mix].value() - 350.0).abs() < 1e-12,
            "reverse flow must carry the sink's 350 K to the junction, got {}",
            temperature[&mix].value()
        );
    }

    /// Q into a zero-volume node with throughput: first law for a heater,
    /// ΔT = Q/(ṁ·cp). 2 kg/s of water and 41 840 W ⇒ exactly +5 K.
    #[test]
    fn heat_input_on_a_junction_raises_its_outlet_temperature() {
        let mut g = PlantGraph::new();
        let src = g.add_node(source("src", Kelvin(300.0)));
        let heater = g.add_node(Node {
            name: "heater".into(),
            kind: NodeKind::Junction,
            heat_input: Watt(2.0 * 4184.0 * 5.0),
        });
        let out = g.add_node(node(
            "out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));
        let e_in = g.add_pipe(src, heater, pipe("inlet"));
        let e_out = g.add_pipe(heater, out, pipe("outlet"));

        let flows = BTreeMap::from([(e_in, 2.0), (e_out, 2.0)]);
        let temperature = resolve(&g, &flows).expect("an acyclic network must resolve");

        assert!(
            (temperature[&heater].value() - 305.0).abs() < 1e-12,
            "Q/(ṁ·cp) must raise 300 K by exactly 5 K, got {}",
            temperature[&heater].value()
        );
    }

    /// A ring of junctions with flow circulating: each one's temperature depends
    /// on the previous, and no inertial node breaks the chain. The sweep must
    /// say so rather than silently resolving in an arbitrary order.
    #[test]
    fn zero_volume_recycle_is_rejected() {
        let mut g = PlantGraph::new();
        let a = g.add_node(node("j_a", NodeKind::Junction));
        let b = g.add_node(node("j_b", NodeKind::Junction));
        let c = g.add_node(node("j_c", NodeKind::Junction));
        let ab = g.add_pipe(a, b, pipe("ab"));
        let bc = g.add_pipe(b, c, pipe("bc"));
        let ca = g.add_pipe(c, a, pipe("ca"));

        let flows = BTreeMap::from([(ab, 1.0), (bc, 1.0), (ca, 1.0)]);
        let err = resolve(&g, &flows).expect_err("a zero-volume-only recycle must be rejected");

        let message = err.to_string();
        assert!(
            message.contains("recycle"),
            "the error must explain what went wrong, got: {message}"
        );
        for name in ["j_a", "j_b", "j_c"] {
            assert!(
                message.contains(name),
                "the error must name the stuck node {name} for diagnosis, got: {message}"
            );
        }
    }

    /// The contrast case that proves the rejection above is about ZERO-VOLUME
    /// recycles specifically, not about loops. The same ring with a tank spliced
    /// in resolves fine: the tank's temperature is a start-of-tick constant, so
    /// the dependency chain terminates — and its 350 K propagates right round.
    #[test]
    fn a_tank_in_the_loop_breaks_the_recycle() {
        let mut g = PlantGraph::new();
        let a = g.add_node(node("j_a", NodeKind::Junction));
        let vessel = g.add_node(tank("vessel", Kelvin(350.0)));
        let c = g.add_node(node("j_c", NodeKind::Junction));
        let ab = g.add_pipe(a, vessel, pipe("ab"));
        let bc = g.add_pipe(vessel, c, pipe("bc"));
        let ca = g.add_pipe(c, a, pipe("ca"));

        let flows = BTreeMap::from([(ab, 1.0), (bc, 1.0), (ca, 1.0)]);
        let temperature = resolve(&g, &flows).expect("a tank in the loop must break the cycle");

        for (id, name) in [(a, "j_a"), (c, "j_c")] {
            assert!(
                (temperature[&id].value() - 350.0).abs() < 1e-12,
                "{name} must inherit the tank's 350 K, got {}",
                temperature[&id].value()
            );
        }
    }

    /// The shared guard, tested directly rather than only through the callers
    /// that reach it. Every path that computes a temperature routes through
    /// this one function, so its contract is worth pinning independently of
    /// which levers happen to reach it. That set moves: a negative
    /// `Command::SetHeatInput` was one until the command started refusing it,
    /// a cooler duty is one now, ambient exchange will be one later. This test
    /// holds whatever the plant can currently do to a node.
    ///
    /// 0 K itself is legal: absolute zero is unreachable, not forbidden, and
    /// erroring on it would reject an exactly-drained stream at the boundary.
    #[test]
    fn checked_temperature_rejects_below_absolute_zero_only() {
        assert_eq!(
            checked_temperature(0.0, || "unused".into())
                .expect("0 K is a legal boundary, not an error")
                .value(),
            0.0
        );
        assert_eq!(
            checked_temperature(300.0, || "unused".into())
                .expect("an ordinary temperature must pass")
                .value(),
            300.0
        );

        let err = checked_temperature(-1e-9, || "the context explains it".into())
            .expect_err("any negative absolute temperature must be rejected, however small");
        assert!(
            err.to_string().contains("the context explains it"),
            "the caller's diagnostic must reach the error, got: {err}"
        );
    }

    // The COOLER path through this guard is not re-tested here: it is pinned
    // end-to-end by `cooler_reference.rs::cooling_below_absolute_zero_is_rejected`,
    // through a real scenario and solver. A copy at this level could only fail
    // together with that one, so it would add a maintenance point and no
    // discrimination. What is genuinely new is the shared checker above.

    // -----------------------------------------------------------------------
    // Heat exchanger
    // -----------------------------------------------------------------------

    /// Two independent streams, coupled. `flows` are hand-built as everywhere
    /// else in this module: the exchanger's contract is thermal, and routing it
    /// through a hydraulic solve would only add a way for the test to fail for
    /// an unrelated reason.
    ///
    /// The hot side is deliberately the SMALLER stream (1 kg/s against 3), so
    /// `C_a ≠ C_b` and `C_min` is distinguishable from `C_max`. With equal
    /// capacity rates the two are the same number and the choice is untestable.
    fn coupled_pair(
        hot_inlet: Kelvin,
        cold_inlet: Kelvin,
        effectiveness: f64,
    ) -> (PlantGraph, BTreeMap<EdgeId, f64>, NodeId, NodeId) {
        let mut g = PlantGraph::new();
        let hot_src = g.add_node(source("hot_src", hot_inlet));
        let hot_side = g.add_node(node("hot_side", NodeKind::HeatExchanger));
        let hot_out = g.add_node(node(
            "hot_out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));
        let cold_src = g.add_node(source("cold_src", cold_inlet));
        let cold_side = g.add_node(node("cold_side", NodeKind::HeatExchanger));
        let cold_out = g.add_node(node(
            "cold_out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));

        let hot_in = g.add_pipe(hot_src, hot_side, pipe("hot_in"));
        let hot_o = g.add_pipe(hot_side, hot_out, pipe("hot_out"));
        let cold_in = g.add_pipe(cold_src, cold_side, pipe("cold_in"));
        let cold_o = g.add_pipe(cold_side, cold_out, pipe("cold_out"));

        g.add_coupling(HeatExchangerCoupling {
            side_a: hot_side,
            side_b: cold_side,
            effectiveness,
        });

        let flows = BTreeMap::from([(hot_in, 1.0), (hot_o, 1.0), (cold_in, 3.0), (cold_o, 3.0)]);
        (g, flows, hot_side, cold_side)
    }

    /// Hand calculation, ΔT-effectiveness (DESIGN §4a):
    ///
    /// ```text
    /// C_hot  = 1·4184 = 4184 W/K      C_cold = 3·4184 = 12552 W/K
    /// C_min  = 4184 W/K               ΔT_in  = 400 − 300 = 100 K
    /// Q      = 0.5 · 4184 · 100 = 209 200 W
    /// T_hot_out  = 400 − 209200/4184  = 350 K exactly
    /// T_cold_out = 300 + 209200/12552 = 300 + 50/3 K
    /// ```
    ///
    /// The asymmetry is the point: the same Q moves both outlets, but by
    /// different amounts, so the test would fail if either side used the wrong
    /// capacity rate. Using `C_max` instead of `C_min` would give Q = 627 600 W
    /// and cool the hot stream to 250 K — BELOW the cold inlet, which is the
    /// second-law violation `C_min` exists to prevent.
    #[test]
    fn an_exchanger_transfers_effectiveness_times_c_min() {
        let (g, flows, hot_side, cold_side) = coupled_pair(Kelvin(400.0), Kelvin(300.0), 0.5);
        let temperature = resolve(&g, &flows).expect("two independent streams must resolve");

        assert!(
            (temperature[&hot_side].value() - 350.0).abs() < 1e-9,
            "hot outlet must be 350 K, got {}",
            temperature[&hot_side].value()
        );
        assert!(
            (temperature[&cold_side].value() - (300.0 + 50.0 / 3.0)).abs() < 1e-9,
            "cold outlet must be 300 + 50/3 K, got {}",
            temperature[&cold_side].value()
        );

        // The duty leaving one stream is the duty entering the other, to the
        // last bit the floats allow: one signed Q, applied twice (DESIGN §4a).
        let given = 4184.0 * (400.0 - temperature[&hot_side].value());
        let taken = 3.0 * 4184.0 * (temperature[&cold_side].value() - 300.0);
        assert!(
            (given - taken).abs() < 1e-6,
            "energy must balance across the exchanger: {given} W out, {taken} W in"
        );
    }

    /// Neither side is hardcoded as the hot one. With the inlets swapped, the
    /// SAME plant must run the heat the other way — the sign of
    /// `T_a_in − T_b_in` is the only thing that decides direction.
    ///
    /// Mirrors the reference above: the small stream now GAINS 50 K and the
    /// large one loses 50/3 K.
    #[test]
    fn heat_flows_from_whichever_side_is_hotter() {
        let (g, flows, side_a, side_b) = coupled_pair(Kelvin(300.0), Kelvin(400.0), 0.5);
        let temperature = resolve(&g, &flows).expect("two independent streams must resolve");

        assert!(
            (temperature[&side_a].value() - 350.0).abs() < 1e-9,
            "the colder small stream must be HEATED to 350 K, got {}",
            temperature[&side_a].value()
        );
        assert!(
            (temperature[&side_b].value() - (400.0 - 50.0 / 3.0)).abs() < 1e-9,
            "the hotter large stream must be COOLED to 400 − 50/3 K, got {}",
            temperature[&side_b].value()
        );
    }

    /// ε scales the duty linearly, and ε = 1 is the thermodynamic limit: the
    /// small stream leaves at exactly the other inlet's temperature, never past
    /// it. This is the boundary `C_min` guarantees and `C_max` would breach.
    #[test]
    fn full_effectiveness_approaches_the_other_inlet_without_crossing_it() {
        let (g, flows, hot_side, cold_side) = coupled_pair(Kelvin(400.0), Kelvin(300.0), 1.0);
        let temperature = resolve(&g, &flows).expect("two independent streams must resolve");

        assert!(
            (temperature[&hot_side].value() - 300.0).abs() < 1e-9,
            "at ε = 1 the C_min stream must reach the other inlet exactly, got {}",
            temperature[&hot_side].value()
        );
        // The second-law bound is each outlet against the OTHER STREAM'S INLET,
        // not against the other outlet. A cold outlet above the hot outlet is
        // ordinary counter-current behaviour, not a violation — and this model
        // draws no co-/counter-current distinction, so asserting the outlets
        // stay ordered would pin a restriction the physics does not impose.
        assert!(
            temperature[&hot_side].value() >= 300.0 - 1e-9,
            "the hot stream must not be cooled below the cold inlet, got {}",
            temperature[&hot_side].value()
        );
        assert!(
            temperature[&cold_side].value() <= 400.0 + 1e-9,
            "the cold stream must not be heated above the hot inlet, got {}",
            temperature[&cold_side].value()
        );
    }

    /// The pair is ONE vertex in the sweep, and this is the test that says so.
    ///
    /// The cold side is fed through a junction, so its inlet is not known until
    /// that junction resolves — while the hot side's own inflow is ready
    /// immediately and carries the lower node id. A per-node sweep therefore
    /// reaches the hot side FIRST and has to read a cold inlet that does not
    /// exist yet. Merged, the pair waits for the union of both sides'
    /// dependencies, which is exactly when both inlets are known.
    #[test]
    fn an_exchanger_pair_waits_for_both_sides_upstreams() {
        let mut g = PlantGraph::new();
        let hot_src = g.add_node(source("hot_src", Kelvin(400.0)));
        let hot_side = g.add_node(node("hot_side", NodeKind::HeatExchanger));
        let hot_out = g.add_node(node(
            "hot_out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));
        let cold_src = g.add_node(source("cold_src", Kelvin(300.0)));
        let cold_side = g.add_node(node("cold_side", NodeKind::HeatExchanger));
        let cold_out = g.add_node(node(
            "cold_out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));
        // Higher node id than either side, so the sweep pops it LAST if it is
        // ordering by node rather than by vertex.
        let mid = g.add_node(node("mid", NodeKind::Junction));

        let hot_in = g.add_pipe(hot_src, hot_side, pipe("hot_in"));
        let hot_o = g.add_pipe(hot_side, hot_out, pipe("hot_out"));
        let cold_feed = g.add_pipe(cold_src, mid, pipe("cold_feed"));
        let cold_in = g.add_pipe(mid, cold_side, pipe("cold_in"));
        let cold_o = g.add_pipe(cold_side, cold_out, pipe("cold_out"));

        g.add_coupling(HeatExchangerCoupling {
            side_a: hot_side,
            side_b: cold_side,
            effectiveness: 0.5,
        });

        let flows = BTreeMap::from([
            (hot_in, 1.0),
            (hot_o, 1.0),
            (cold_feed, 3.0),
            (cold_in, 3.0),
            (cold_o, 3.0),
        ]);
        let temperature = resolve(&g, &flows).expect("the pair must wait for the junction");

        // Same hand calculation as the reference: the junction is a pure
        // pass-through, so the numbers must be untouched by its presence.
        assert!(
            (temperature[&hot_side].value() - 350.0).abs() < 1e-9,
            "hot outlet must still be 350 K, got {}",
            temperature[&hot_side].value()
        );
        assert!(
            (temperature[&cold_side].value() - (300.0 + 50.0 / 3.0)).abs() < 1e-9,
            "cold outlet must still be 300 + 50/3 K, got {}",
            temperature[&cold_side].value()
        );
    }

    /// One side feeding the other is a genuine circular definition — side A's
    /// outlet depends on B's inlet, which IS A's outlet — so it must land in
    /// the existing recycle rejection rather than resolve against a stale
    /// value. This is why the merge skips self-dependencies in BOTH the count
    /// and the release: discounting them in only one place would make this
    /// plant silently ready.
    #[test]
    fn an_exchanger_feeding_its_own_partner_is_rejected() {
        let mut g = PlantGraph::new();
        let src = g.add_node(source("src", Kelvin(400.0)));
        let side_a = g.add_node(node("side_a", NodeKind::HeatExchanger));
        let side_b = g.add_node(node("side_b", NodeKind::HeatExchanger));
        let out = g.add_node(node(
            "out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));
        let feed = g.add_pipe(src, side_a, pipe("feed"));
        let across = g.add_pipe(side_a, side_b, pipe("across"));
        let drain = g.add_pipe(side_b, out, pipe("drain"));

        g.add_coupling(HeatExchangerCoupling {
            side_a,
            side_b,
            effectiveness: 0.5,
        });

        let flows = BTreeMap::from([(feed, 1.0), (across, 1.0), (drain, 1.0)]);
        let err = resolve(&g, &flows)
            .expect_err("an exchanger in series with itself is mutually defined");
        let message = err.to_string();
        assert!(
            message.contains("recycle"),
            "the error must explain what went wrong, got: {message}"
        );
        for name in ["side_a", "side_b"] {
            assert!(
                message.contains(name),
                "the error must name the stuck side {name}, got: {message}"
            );
        }
    }

    /// An exchanger with one stream stopped is a pipe: no throughput on a side
    /// means no capacity rate to transfer against, so the duty is zero and the
    /// running side passes its inlet straight through. Physics, not a guard —
    /// but worth pinning, because the alternative (dividing by a zero capacity)
    /// produces NaN that would propagate as an ordinary temperature.
    #[test]
    fn a_stalled_side_transfers_nothing() {
        let (mut g, mut flows, hot_side, cold_side) =
            coupled_pair(Kelvin(400.0), Kelvin(300.0), 0.5);
        let _ = &mut g;
        for flow in flows.values_mut() {
            // Stop the cold stream only; its two pipes carry 3.0.
            if *flow == 3.0 {
                *flow = 0.0;
            }
        }
        let temperature = resolve(&g, &flows).expect("a stalled side must not break the sweep");

        assert!(
            (temperature[&hot_side].value() - 400.0).abs() < 1e-12,
            "with nothing to exchange with, the hot side must pass 400 K through, got {}",
            temperature[&hot_side].value()
        );
        assert!(
            temperature[&cold_side].value().is_finite(),
            "the stalled side must hold a finite temperature, got {}",
            temperature[&cold_side].value()
        );
    }

    /// A junction nothing flows through is indeterminate (0/0), not broken. It
    /// must hold the last value it saw — finite and reproducible — because mass
    /// balance means it carries no enthalpy anywhere regardless.
    #[test]
    fn a_stagnant_junction_holds_its_previous_temperature() {
        let mut g = PlantGraph::new();
        let src = g.add_node(source("src", Kelvin(280.0)));
        let idle = g.add_node(node("idle", NodeKind::Junction));
        let dead = g.add_pipe(src, idle, pipe("dead_leg"));

        let flows = BTreeMap::from([(dead, 0.0)]);
        let previous = NodeStates {
            temperature: BTreeMap::from([(idle, Kelvin(311.0))]),
            composition: BTreeMap::new(),
        };
        let temperature = resolve_node_states(&g, &Slate::water_only(), &flows, &previous)
            .unwrap()
            .temperature;
        assert_eq!(
            temperature[&idle].value(),
            311.0,
            "must hold the last value"
        );

        // With no history at all it falls back to ambient rather than NaN.
        let fresh = resolve(&g, &flows).unwrap();
        assert_eq!(fresh[&idle].value(), T_AMBIENT.value());
    }

    // -----------------------------------------------------------------------
    // The pipe ambient transform, as a pure function.
    // -----------------------------------------------------------------------
    //
    // Tested here rather than only through an engine because the closed form is
    // worth pinning against a HAND-COMPUTED number, and an engine-level test
    // cannot do that: the mass flow comes out of the solver, so the expected
    // value would have to be built by re-evaluating the same formula the code
    // just evaluated — a tautology that passes whatever the exponent's sign is.
    // The engine-level file (`solvers/tests/pipe_ambient_reference.rs`) tests
    // the complementary thing: that this function is actually WIRED IN, and it
    // cross-checks the physics against the log-mean form, which is independent.
    mod pipe_ambient {
        use super::*;

        /// Water's cp [J/(kg·K)] — written out rather than read from the slate,
        /// so both sides of the reference do not share one source of truth.
        const CP: JPerKgK = JPerKgK(4184.0);

        /// REFERENCE — hand computation at exactly one transfer unit.
        ///
        /// Chosen so `UA/(ṁ·cp)` is exactly 1 and the decay is `e⁻¹`, a constant
        /// that can be written down independently of the code:
        ///
        ///   ṁ·cp   = 1 kg/s · 4184 J/(kg·K) = 4184 W/K
        ///   UA     = 4184 W/K            ⇒ exponent = −1
        ///   e⁻¹    = 0.367879441171442334
        ///   ΔT_in  = 373.15 − 293.15 = 80 K
        ///   T_out  = 293.15 + 80·e⁻¹ = 293.15 + 29.4303552937153867
        ///          = 322.580355293715387 K
        ///
        /// A wrong-signed exponent gives 293.15 + 80·e = 510.6 K, and an Euler
        /// step gives 293.15 + 80·(1−1) = 293.15 K exactly — both far outside
        /// the tolerance, which is round-off because this form is analytic and
        /// has no truncation error to budget for (contrast the tank's 1e-3).
        #[test]
        fn one_transfer_unit_decays_by_exactly_e_inverse() {
            let out =
                pipe_outlet_temperature(Kelvin(373.15), WattPerKelvin(4184.0), KgPerSec(1.0), CP);
            // The hand computation above gives 322.580355293715387; this is
            // that value truncated to what an f64 can actually hold. The
            // difference is ~1e-13, well inside the 1e-9 tolerance, so the
            // reference is still the hand calculation and not a readback.
            let expected = 322.580_355_293_715_4;
            assert!(
                (out.value() - expected).abs() < 1e-9,
                "one transfer unit must decay by e⁻¹ to {expected} K, got {}",
                out.value()
            );
        }

        /// `UA = 0` is the identity, which is what makes every scenario written
        /// before this field existed bit-identical. Asserted EXACTLY: `exp(0)`
        /// is 1.0 to the bit, so any tolerance here would hide a near-miss.
        #[test]
        fn no_ua_is_exactly_the_identity() {
            let inlet = Kelvin(373.15);
            let out = pipe_outlet_temperature(inlet, WattPerKelvin::ZERO, KgPerSec(2.5), CP);
            assert_eq!(out.value(), inlet.value());
        }

        /// The `|ṁ|` requirement. Reversing the flow reverses which end is the
        /// inlet — a decision made before this function is called — so the
        /// transform itself must be blind to the sign.
        ///
        /// Under a signed `ṁ` the exponent flips and this returns 510.6 K: the
        /// pipe would run AWAY from ambient, heating an already-hot stream. That
        /// is the mutation this test exists to catch, and it catches it here
        /// rather than in a plant, so the coverage does not depend on some
        /// scenario happening to reverse a flow.
        #[test]
        fn reverse_flow_decays_identically() {
            let forward =
                pipe_outlet_temperature(Kelvin(373.15), WattPerKelvin(4184.0), KgPerSec(1.0), CP);
            let reverse =
                pipe_outlet_temperature(Kelvin(373.15), WattPerKelvin(4184.0), KgPerSec(-1.0), CP);
            assert_eq!(
                forward.value(),
                reverse.value(),
                "the transform must depend on |ṁ| only — the flow's sign already \
                 picked the inlet end"
            );
        }

        /// The reason for analytic rather than Euler, stated as a test.
        ///
        /// At 100 transfer units an explicit step, `T + (UA/(ṁ·cp))·(T_amb − T)`,
        /// would land at 293.15 − 99·80 ≈ −7627 K: past ambient, past absolute
        /// zero, and diverging further every tick. `exp` cannot cross ambient at
        /// any exponent, so the outlet is bracketed by ambient below and the
        /// inlet above no matter how extreme the ratio.
        #[test]
        fn a_huge_transfer_ratio_approaches_ambient_but_never_crosses_it() {
            let out = pipe_outlet_temperature(
                Kelvin(373.15),
                WattPerKelvin(418_400.0), // 100 transfer units
                KgPerSec(1.0),
                CP,
            );
            assert!(
                out.value() >= T_AMBIENT.value() && out.value() <= 373.15,
                "the outlet must stay between ambient and the inlet, got {}",
                out.value()
            );
            assert!(
                (out.value() - T_AMBIENT.value()).abs() < 1e-9,
                "100 transfer units must land on ambient, got {}",
                out.value()
            );
        }

        /// The zero-flow guard, on the case that actually reaches it.
        ///
        /// `ṁ = 0` with `UA = 0` is `0/0`. Without the guard this returns NaN,
        /// which no downstream check attributes to a pipe — it surfaces as a
        /// bare non-finite. This is the DEFAULT pipe with a closed valve, so it
        /// is the reachable case, not a contrived one.
        #[test]
        fn a_stagnant_uninsulated_pipe_is_finite_not_nan() {
            let out =
                pipe_outlet_temperature(Kelvin(373.15), WattPerKelvin::ZERO, KgPerSec::ZERO, CP);
            assert!(
                out.value().is_finite(),
                "a stagnant pipe must not produce a NaN temperature, got {}",
                out.value()
            );
            assert_eq!(out.value(), 373.15);
        }

        /// The other stagnant case: zero flow with a real `UA`. This one is
        /// finite even without the guard (`−∞ → exp = 0 → ambient`), so it is
        /// documenting a CHOICE rather than preventing a NaN. Holding the inlet
        /// is the honest answer at this fidelity: with no throughput there is no
        /// stream to carry the heat, and a pipe that instantly equilibrated with
        /// ambient the moment its valve shut would be modelling a wall thermal
        /// mass the engine does not have.
        #[test]
        fn a_stagnant_insulated_pipe_holds_its_inlet_rather_than_snapping_to_ambient() {
            let out =
                pipe_outlet_temperature(Kelvin(373.15), WattPerKelvin(4184.0), KgPerSec::ZERO, CP);
            assert_eq!(
                out.value(),
                373.15,
                "a stagnant pipe holds its inlet; snapping to ambient would claim \
                 a wall thermal mass this fidelity does not model"
            );
        }

        /// Ambient exchange is not one-directional. A stream COLDER than ambient
        /// must warm toward it through the same term, with no second code path —
        /// the same property `ambient_exchange` has for tanks, and reachable in
        /// a plant with a `Cooler` upstream.
        #[test]
        fn a_cold_stream_warms_toward_ambient() {
            let out =
                pipe_outlet_temperature(Kelvin(273.15), WattPerKelvin(4184.0), KgPerSec(1.0), CP);
            assert!(
                out.value() > 273.15 && out.value() < T_AMBIENT.value(),
                "a stream below ambient must warm toward it, got {}",
                out.value()
            );
        }
    }
}
