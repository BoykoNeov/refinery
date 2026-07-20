//! The plant graph: units as nodes, pipes as edges.
//!
//! Design rules:
//! - Pumps and valves are NODES with exactly one inlet and one outlet edge,
//!   so edges are uniform plain pipes and the solver sees one element kind
//!   per branch: pipe + (optional) node characteristic at its ends.
//! - petgraph is an implementation detail; it must not leak through pub APIs.
//! - Damage is graph surgery: a leak adds an edge to an Atmosphere node,
//!   a fire adds a heat source term to a node. No special-cased physics.

use crate::components::Composition;
use crate::stream::Stream;
use crate::units::*;
use petgraph::stable_graph::{EdgeIndex, NodeIndex, StableDiGraph};
use serde::{Deserialize, Serialize};

/// Stable, serializable node handle (index into the petgraph storage).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct NodeId(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct EdgeId(pub u32);

impl From<NodeId> for NodeIndex {
    fn from(id: NodeId) -> Self {
        NodeIndex::new(id.0 as usize)
    }
}
impl From<EdgeId> for EdgeIndex {
    fn from(id: EdgeId) -> Self {
        EdgeIndex::new(id.0 as usize)
    }
}

// ---------------------------------------------------------------------------
// Nodes (units)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Node {
    /// Scenario-given name, unique per plant; frontends key on this.
    pub name: String,
    pub kind: NodeKind,
    /// External heat input [W] (fires, heaters). Damage model hooks in here.
    pub heat_input: Watt,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum NodeKind {
    /// Infinite feed at fixed pressure/temperature/composition.
    Source {
        pressure: Pascal,
        temperature: Kelvin,
        composition: Composition,
    },
    /// Infinite sink at fixed pressure. `temperature` and `composition` are the
    /// fluid it returns if the network ever drives flow *backwards* into it
    /// (network pressure below the sink's): an infinite reservoir has to have
    /// both to back-feed, and leaving either implicit would make reverse flow
    /// ill-defined.
    ///
    /// `composition` mirrors `temperature` exactly, and for the same reason —
    /// the alternative considered was a stateful sink that remembers what last
    /// flowed into it, which makes the back-fed fluid depend on tick history
    /// rather than on the plant definition. Reverse flow into a sink is not
    /// hypothetical: `energy_invariants.rs`'s chain proptest already generates
    /// it.
    Sink {
        pressure: Pascal,
        temperature: Kelvin,
        composition: Composition,
    },
    /// The outside world; leak edges terminate here. Fixed at P_ATM and,
    /// symmetrically, at T_AMBIENT.
    Atmosphere,
    /// Vertical cylindrical tank, vented (gas blanket pressure = P_ATM for
    /// M1; pressurized vessels are a later fidelity step).
    Tank(TankState),
    /// Centrifugal pump: head curve H(Q) = h0 - a·Q² (Q in m³/s, H in m).
    Pump { h0: Meter, a: f64, on: bool },
    /// Control valve, ISA-style: Q = Cv_eff(opening)·sqrt(dP/SG).
    /// `cv_max` in SI-consistent form (m³/s at 1 Pa dP for SG=1) — the
    /// scenario loader converts from customary Cv units.
    Valve { cv_max: f64, opening: f64 },
    /// Zero-volume mixing point.
    Junction,
    /// Fired heater: a duty delivered into the stream passing through it.
    ///
    /// Zero-volume like a pump or valve — a furnace's tube inventory is
    /// negligible against its duty, so its outlet temperature is algebraic
    /// (`T_out = T_in + Q/(ṁ·cp)`) rather than a state. Hydraulically it is a
    /// plain pass-through at M2: the tube-side pressure drop belongs to the
    /// connecting pipes' resistance, not to a device characteristic.
    ///
    /// `duty` is the heat actually delivered to the process fluid [W], not a
    /// firing rate — combustion efficiency is a later fidelity step. Duty 0 is
    /// an unlit furnace; there is no separate `on` flag because there is
    /// nothing for one to express that 0 does not.
    ///
    /// Deliberately NOT stored in `Node::heat_input`: that field is the damage
    /// model's hook (fires), and a fire on a furnace must ADD to its duty, not
    /// overwrite the operator's setpoint. See `energy::heat_load`.
    Furnace { duty: Watt },
    /// Cooler: a duty *removed* from the stream passing through it.
    ///
    /// Structurally the furnace's mirror — zero-volume, hydraulically a
    /// pass-through, algebraic outlet temperature — and `duty` is likewise a
    /// non-negative magnitude: `energy::heat_load` applies the sign, SUBTRACTING
    /// a cooler's duty where it adds a furnace's.
    ///
    /// A separate unit rather than a negative-duty `Furnace`, deliberately. A
    /// bare signed number in a scenario file cannot be read without knowing
    /// which sign convention its unit uses, and a sign typo would silently turn
    /// a heater into a chiller. With two units the intent is in the name, and
    /// negative duty becomes meaningless input that both the loader and
    /// `Command::SetFurnaceDuty`/`SetCoolerDuty` reject.
    ///
    /// A fire (`Node::heat_input`) on a cooler correctly *fights* the cooling
    /// rather than replacing it, for free — `heat_load` sums the two terms.
    ///
    /// KNOWN LIMITATION: a fixed duty has no coolant-temperature floor, so a
    /// large duty on a small flow cools past the coolant, past ambient, and in
    /// the limit past 0 K. Only the last of those is detectable without a
    /// coolant model, and `mix_inflows` rejects it. Cooling to a realistic
    /// approach temperature is the `HeatExchanger`'s job (M2.2), not this one's.
    Cooler { duty: Watt },
    /// One side of a two-stream heat exchanger.
    ///
    /// A side is an ordinary zero-volume pass-through — hydraulically identical
    /// to a junction — and carries NO parameters of its own. What makes it an
    /// exchanger is the `HeatExchangerCoupling` naming it and its partner; this
    /// variant only says "I am a side", which is what `energy::is_zero_volume`
    /// and `energy::boundary_temperature` need to recognize.
    ///
    /// The effectiveness deliberately lives on the coupling rather than here.
    /// It is a property of the PAIR, and storing it once makes a pair whose two
    /// halves disagree about ε unrepresentable — the same instinct that made
    /// `Furnace` and `Cooler` separate units instead of one signed duty: put the
    /// invariant in the type, not in a convention.
    ///
    /// Neither side is "the hot one". Which way heat flows is decided per tick
    /// by the sign of `T_a_in − T_b_in`, so an exchanger whose duty reverses
    /// (seasonal service, a startup transient) needs no reconfiguration.
    HeatExchanger,
}

/// The thermal pairing of two `HeatExchanger` sides.
///
/// Hydraulically the two sides are unrelated: the flow solver never sees this
/// list, and the streams do not mix. The coupling exists only so the energy
/// sweep knows the pair must be resolved TOGETHER — each side's outlet depends
/// on the other side's inlet, which is not one of its own inflow edges.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeatExchangerCoupling {
    pub side_a: NodeId,
    pub side_b: NodeId,
    /// Effectiveness ε ∈ (0, 1]: the fraction of the thermodynamically maximum
    /// duty `C_min·(T_a_in − T_b_in)` this exchanger actually transfers.
    ///
    /// ε > 1 transfers more heat than the temperature difference makes
    /// available and would cross the outlet temperatures — a second-law
    /// violation — so it is rejected at every entry point. ε = 0 is a nonsense
    /// exchanger (use a plain pipe) and is likewise refused.
    pub effectiveness: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TankState {
    pub area: SquareMeter,
    pub height: Meter,
    pub mass: Kg,
    pub temperature: Kelvin,
    pub composition: Composition,
    /// Ambient heat transfer coefficient × exposed area, `UA` [W/K].
    ///
    /// Drives `Q = UA·(T_AMBIENT − T_tank)` — a SIGNED term, applied by
    /// `energy::ambient_exchange`, which heats a tank colder than ambient and
    /// cools one hotter with no second code path. See `energy::heat_load`.
    ///
    /// Defaults to ZERO: a perfectly insulated tank. That default is load
    /// bearing, not a placeholder — every scenario written before this field
    /// existed stays bit-identical, and `isothermal_plant.rs` keeps testing what
    /// it always tested. A tank that silently started leaking heat the day the
    /// field landed would turn that flat line into a lie.
    #[serde(default = "no_ambient_exchange")]
    pub ambient_ua: WattPerKelvin,
}

/// The `ambient_ua` default: a perfectly insulated body.
///
/// A local function rather than a blanket `Default` on the unit newtypes: `0`
/// is the physically meaningful "no exchange" here, whereas a default `Kelvin`
/// of 0 K would be a silent absurdity waiting for the first struct that forgot
/// to set one.
fn no_ambient_exchange() -> WattPerKelvin {
    WattPerKelvin::ZERO
}

impl TankState {
    pub fn level(&self, density: KgPerM3) -> Meter {
        Meter((self.mass / density).value() / self.area.value())
    }
    /// Hydrostatic pressure at the tank bottom nozzle.
    /// P = P_atm + ρ·g·h (vented tank).
    pub fn bottom_pressure(&self, density: KgPerM3) -> Pascal {
        Pascal(P_ATM.value() + density.value() * G * self.level(density).value())
    }
}

// ---------------------------------------------------------------------------
// Edges (pipes)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pipe {
    pub name: String,
    pub length: Meter,
    pub diameter: Meter,
    /// Darcy friction factor (constant for M1; Colebrook/Haaland later).
    pub friction_factor: f64,
    /// Elevation change target-minus-source [m], for the static head term.
    pub elevation_change: Meter,
    /// Leak orifice area (damage model); 0 = intact.
    pub leak_area: SquareMeter,
    /// Ambient heat transfer coefficient × exposed area, `UA` [W/K].
    ///
    /// Spelled like `TankState::ambient_ua` and defaulting to ZERO for the same
    /// reason, but it does NOT drive the same equation. A tank is a lumped
    /// inventory, so its exchange is one signed `Q` added to its energy balance;
    /// a pipe is a flow-through body with no inventory at this fidelity, so its
    /// exchange is a TRANSFORM along the edge — see
    /// `energy::pipe_outlet_temperature`. Adding `UA·(T_AMBIENT − T)` to a pipe
    /// as if it were a tank would be dimensionally fine and physically wrong.
    ///
    /// The consequence worth flagging at the field: a pipe with a nonzero `UA`
    /// is NO LONGER ISOTHERMAL, which retires the identity M2.1 transport was
    /// built on (an edge's temperature is its upwind node's). Everything that
    /// reads a temperature off an edge must therefore say which END it means,
    /// and go through `energy::edge_temperature_at` to get it.
    #[serde(default = "no_ambient_exchange")]
    pub ambient_ua: WattPerKelvin,
    /// Transported material state, updated by the engine each tick.
    ///
    /// `stream.temperature` is the pipe's OUTLET temperature — the value the
    /// downstream node receives. With `ambient_ua = 0` the two ends agree and
    /// the distinction is invisible; with a nonzero `UA` it is a deliberate
    /// display choice, taken because nothing in the engine consumes this field
    /// (only tests and the snapshot do) and the outlet is the one end a snapshot
    /// reader cannot reconstruct from the upwind node's temperature.
    pub stream: Stream,
}

// ---------------------------------------------------------------------------
// Graph wrapper
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Default)]
pub struct PlantGraph {
    g: StableDiGraph<Node, Pipe>,
    /// Thermal pairings between `HeatExchanger` sides. A `Vec`, not a map:
    /// insertion-ordered iteration is deterministic (rule 3), and the list is
    /// short enough that the linear `partner` lookup costs nothing.
    couplings: Vec<HeatExchangerCoupling>,
}

impl PlantGraph {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_node(&mut self, node: Node) -> NodeId {
        NodeId(self.g.add_node(node).index() as u32)
    }

    pub fn add_pipe(&mut self, from: NodeId, to: NodeId, pipe: Pipe) -> EdgeId {
        EdgeId(self.g.add_edge(from.into(), to.into(), pipe).index() as u32)
    }

    pub fn node(&self, id: NodeId) -> &Node {
        &self.g[NodeIndex::from(id)]
    }
    pub fn node_mut(&mut self, id: NodeId) -> &mut Node {
        &mut self.g[NodeIndex::from(id)]
    }
    pub fn pipe(&self, id: EdgeId) -> &Pipe {
        &self.g[EdgeIndex::from(id)]
    }
    pub fn pipe_mut(&mut self, id: EdgeId) -> &mut Pipe {
        &mut self.g[EdgeIndex::from(id)]
    }

    /// (source, target) node ids of an edge.
    pub fn endpoints(&self, id: EdgeId) -> (NodeId, NodeId) {
        let (a, b) = self.g.edge_endpoints(id.into()).expect("valid edge id");
        (NodeId(a.index() as u32), NodeId(b.index() as u32))
    }

    /// Deterministic iteration: ascending index order, guaranteed stable.
    pub fn node_ids(&self) -> impl Iterator<Item = NodeId> + '_ {
        self.g.node_indices().map(|i| NodeId(i.index() as u32))
    }
    pub fn edge_ids(&self) -> impl Iterator<Item = EdgeId> + '_ {
        self.g.edge_indices().map(|i| EdgeId(i.index() as u32))
    }

    /// Edges incident to a node as (edge, other_node, is_incoming).
    pub fn incident(&self, id: NodeId) -> Vec<(EdgeId, NodeId, bool)> {
        use petgraph::Direction;
        let n: NodeIndex = id.into();
        let mut out = Vec::new();
        for dir in [Direction::Incoming, Direction::Outgoing] {
            for e in self.g.edges_directed(n, dir) {
                use petgraph::visit::EdgeRef;
                let other = if dir == Direction::Incoming {
                    e.source()
                } else {
                    e.target()
                };
                out.push((
                    EdgeId(e.id().index() as u32),
                    NodeId(other.index() as u32),
                    dir == Direction::Incoming,
                ));
            }
        }
        // Deterministic order regardless of petgraph internals.
        out.sort_by_key(|(e, _, _)| *e);
        out
    }

    pub fn node_count(&self) -> usize {
        self.g.node_count()
    }
    pub fn edge_count(&self) -> usize {
        self.g.edge_count()
    }

    /// Thermally pair two `HeatExchanger` sides. Validation of the node kinds
    /// and of ε belongs to the loader, which can name the offending scenario
    /// entry; this is the plain storage operation.
    pub fn add_coupling(&mut self, coupling: HeatExchangerCoupling) {
        self.couplings.push(coupling);
    }

    pub fn couplings(&self) -> &[HeatExchangerCoupling] {
        &self.couplings
    }

    /// The other side of `id`'s exchanger, with the pair's effectiveness.
    ///
    /// Searches both fields, so a coupling may be declared in either order and
    /// no caller has to know whether a given node was written as side A or B.
    pub fn exchanger_partner(&self, id: NodeId) -> Option<(NodeId, f64)> {
        self.couplings.iter().find_map(|c| {
            if c.side_a == id {
                Some((c.side_b, c.effectiveness))
            } else if c.side_b == id {
                Some((c.side_a, c.effectiveness))
            } else {
                None
            }
        })
    }

    pub fn find_node(&self, name: &str) -> Option<NodeId> {
        self.node_ids().find(|id| self.node(*id).name == name)
    }
}
