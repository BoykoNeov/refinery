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
use crate::units::{JPerKgK, Kelvin, KgPerSec, Watt, T_AMBIENT};
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

/// True for nodes with no inventory, whose temperature is an instantaneous
/// mix of their inflows rather than a state (DESIGN §4a).
///
/// Pumps and valves are zero-volume *pass-throughs*: with exactly one inlet and
/// one outlet (enforced by `validate_degrees`) the mixing formula degenerates
/// to "outlet temperature = inlet temperature". Pump work and valve throttling
/// both dissipate into the stream as heat; at M2 fidelity that rise is
/// neglected (~0.02 K for the reference pump — far below the model's accuracy)
/// and is a documented DESIGN §4a limitation, not an oversight.
pub fn is_zero_volume(kind: &NodeKind) -> bool {
    matches!(
        kind,
        NodeKind::Junction | NodeKind::Pump { .. } | NodeKind::Valve { .. }
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
        NodeKind::Junction | NodeKind::Pump { .. } | NodeKind::Valve { .. } => None,
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

    // 2. Count each zero-volume node's unresolved upstream dependencies. Only
    //    zero-volume upstreams count — an inertial upstream is already known.
    let mut pending: BTreeMap<NodeId, usize> = BTreeMap::new();
    for &id in &zero_volume {
        let deps = inflow_edges(graph, edge_mass_flow, id)
            .iter()
            .filter(|(_, upstream, _)| is_zero_volume(&graph.node(*upstream).kind))
            .count();
        pending.insert(id, deps);
    }

    // 3. Kahn sweep. BTreeSet, always popping the lowest id: the order is then
    //    a function of the graph alone, never of insertion history (rule 3).
    //    Any valid topological order yields identical temperatures anyway —
    //    each mix reads only already-resolved upstreams — but a deterministic
    //    order keeps the float summation order fixed too.
    let mut ready: BTreeSet<NodeId> = zero_volume
        .iter()
        .copied()
        .filter(|id| pending[id] == 0)
        .collect();

    let mut resolved = 0usize;
    while let Some(&id) = ready.iter().next() {
        ready.remove(&id);
        let mixed = mix_inflows(graph, slate, edge_mass_flow, &temperature, previous, id)?;
        temperature.insert(id, mixed);
        resolved += 1;

        // Release the downstream zero-volume nodes this one feeds. An edge is
        // an inflow to the far node exactly when it is an outflow here: the
        // two views share one `flow`, so `into_other == -into_id` identically.
        // Counting per EDGE (not per node) mirrors `pending`, which counted
        // inflow edges — parallel pipes decrement once each, as they should.
        for (edge, downstream, incoming) in graph.incident(id) {
            if downstream == id || !is_zero_volume(&graph.node(downstream).kind) {
                continue;
            }
            let flow = edge_mass_flow.get(&edge).copied().unwrap_or(0.0);
            let into_id = if incoming { flow } else { -flow };
            if into_id >= 0.0 {
                continue; // not an outflow ⇒ not an inflow to `downstream`
            }
            if let Some(count) = pending.get_mut(&downstream) {
                *count = count.saturating_sub(1);
                if *count == 0 {
                    ready.insert(downstream);
                }
            }
        }
    }

    // 4. A node left unresolved means its dependencies never cleared: the only
    //    way that happens is a cycle among zero-volume nodes.
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

/// Enthalpy-weighted mix of a zero-volume node's inflows [K].
fn mix_inflows(
    graph: &PlantGraph,
    slate: &Slate,
    edge_mass_flow: &BTreeMap<EdgeId, f64>,
    temperature: &BTreeMap<NodeId, Kelvin>,
    previous: &BTreeMap<NodeId, Kelvin>,
    node: NodeId,
) -> Result<Kelvin, SimError> {
    // External heat (a fire, a heater) joins the same first law: for a node
    // with no accumulation, Σ ṁ·h_in + Q = Σ ṁ·h_out. Without this term
    // `Command::SetHeatInput` would be a silent no-op on every junction — the
    // damage model's "fire on a node" would do nothing at all.
    let heat_input = graph.node(node).heat_input.value();
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

    if capacity > 0.0 {
        Ok(Kelvin(T_REF.value() + (enthalpy + heat_input) / capacity))
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

#[cfg(test)]
mod tests {
    //! The sweep is tested with HAND-BUILT flow maps, never through a solver.
    //! Its contract is "given these flows, produce these temperatures", so
    //! feeding it flows directly tests exactly that — and lets the recycle case
    //! be built on demand instead of hoping a network happens to circulate.

    use super::*;
    use crate::components::{Composition, Slate};
    use crate::graph::{Node, Pipe, TankState};
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
