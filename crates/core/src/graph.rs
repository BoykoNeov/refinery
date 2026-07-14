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
    /// Infinite sink at fixed pressure.
    Sink { pressure: Pascal },
    /// The outside world; leak edges terminate here. Fixed at P_ATM.
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
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TankState {
    pub area: SquareMeter,
    pub height: Meter,
    pub mass: Kg,
    pub temperature: Kelvin,
    pub composition: Composition,
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
    /// Transported material state, updated by the engine each tick.
    pub stream: Stream,
}

// ---------------------------------------------------------------------------
// Graph wrapper
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Default)]
pub struct PlantGraph {
    g: StableDiGraph<Node, Pipe>,
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

    pub fn find_node(&self, name: &str) -> Option<NodeId> {
        self.node_ids().find(|id| self.node(*id).name == name)
    }
}
