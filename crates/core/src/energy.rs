//! Energy transport: upwind (donor-cell) temperature advection.
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

use crate::components::Slate;
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

/// Resolve every node's temperature for this tick [K].
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
/// `previous` supplies the fallback for a zero-volume node with **no inflow**:
/// nothing enters it, so its temperature is physically indeterminate — and it
/// carries no enthalpy either way, since mass balance forces zero outflow too.
/// Holding the last resolved value keeps the field finite and reproducible
/// instead of dividing 0/0.
///
/// # Errors
/// `SimError::Numerical` if a recycle among zero-volume nodes leaves the sweep
/// with no valid order (see the module docs).
pub fn resolve_node_temperatures(
    graph: &PlantGraph,
    slate: &Slate,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    previous: &BTreeMap<NodeId, Kelvin>,
) -> Result<BTreeMap<NodeId, Kelvin>, SimError> {
    let mut temperature: BTreeMap<NodeId, Kelvin> = BTreeMap::new();

    // 1. Inertial nodes are the roots of the sweep: known before it starts.
    let mut zero_volume: Vec<NodeId> = Vec::new();
    for id in graph.node_ids() {
        match boundary_temperature(&graph.node(id).kind) {
            Some(t) => {
                temperature.insert(id, t);
            }
            None => zero_volume.push(id),
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
        match sides.as_slice() {
            [a, b] => {
                // Both outlets from one signed Q, computed from both inlets.
                let (t_a, t_b) = exchange_pair(
                    graph,
                    slate,
                    edge_mass_flow,
                    &temperature,
                    previous,
                    (*a, *b),
                )?;
                temperature.insert(*a, t_a);
                temperature.insert(*b, t_b);
            }
            _ => {
                for &id in sides {
                    let mixed =
                        mix_inflows(graph, slate, edge_mass_flow, &temperature, previous, id)?;
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

    Ok(temperature)
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
    node: NodeId,
) -> Result<InflowTotals, SimError> {
    let mut enthalpy = 0.0; // Σ ṁ·cp·(T − T_REF) [W]
    let mut capacity = 0.0; // Σ ṁ·cp [W/K]

    for (edge, upstream, into_node) in inflow_edges(graph, edge_mass_flow, node) {
        // Resolved by construction: the sweep only visits a node once every
        // zero-volume upstream is done, and inertial ones are seeded. Return an
        // error rather than indexing, so a sort bug can never panic (rule 5).
        let upstream_t = temperature.get(&upstream).copied().ok_or_else(|| {
            SimError::Numerical(format!(
                "internal: upstream '{}' of '{}' unresolved during the temperature sweep",
                graph.node(upstream).name,
                graph.node(node).name
            ))
        })?;
        let cp = graph.pipe(edge).stream.composition.mixture_cp(slate);
        enthalpy += enthalpy_flux(KgPerSec(into_node), cp, upstream_t).value();
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
        inflow_totals(graph, slate, edge_mass_flow, temperature, node)?
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
    previous: &BTreeMap<NodeId, Kelvin>,
    (side_a, side_b): (NodeId, NodeId),
) -> Result<(Kelvin, Kelvin), SimError> {
    let (_, effectiveness) = graph.exchanger_partner(side_a).ok_or_else(|| {
        SimError::Numerical(format!(
            "internal: '{}' was swept as an exchanger side but has no coupling",
            graph.node(side_a).name
        ))
    })?;

    let totals_a = inflow_totals(graph, slate, edge_mass_flow, temperature, side_a)?;
    let totals_b = inflow_totals(graph, slate, edge_mass_flow, temperature, side_b)?;

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
            stream: Stream::stagnant(1, T_AMBIENT, P_ATM),
        }
    }

    fn resolve(
        graph: &PlantGraph,
        flows: &BTreeMap<EdgeId, f64>,
    ) -> Result<BTreeMap<NodeId, Kelvin>, SimError> {
        resolve_node_temperatures(graph, &Slate::water_only(), flows, &BTreeMap::new())
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
            },
        ));
        let cold_src = g.add_node(source("cold_src", cold_inlet));
        let cold_side = g.add_node(node("cold_side", NodeKind::HeatExchanger));
        let cold_out = g.add_node(node(
            "cold_out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
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
            },
        ));
        let cold_src = g.add_node(source("cold_src", Kelvin(300.0)));
        let cold_side = g.add_node(node("cold_side", NodeKind::HeatExchanger));
        let cold_out = g.add_node(node(
            "cold_out",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
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
        let previous = BTreeMap::from([(idle, Kelvin(311.0))]);
        let temperature =
            resolve_node_temperatures(&g, &Slate::water_only(), &flows, &previous).unwrap();
        assert_eq!(
            temperature[&idle].value(),
            311.0,
            "must hold the last value"
        );

        // With no history at all it falls back to ambient rather than NaN.
        let fresh = resolve(&g, &flows).unwrap();
        assert_eq!(fresh[&idle].value(), T_AMBIENT.value());
    }
}
