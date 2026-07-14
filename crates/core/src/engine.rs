//! The engine: owns the graph and solver implementations, advances time.
//!
//! Tick sequence (see docs/DESIGN.md §1):
//!   commands → hydraulic solve (quasi-steady) → transport → unit dynamics
//!   → validation → snapshot available.

use crate::components::Slate;
use crate::error::SimError;
use crate::graph::{NodeKind, PlantGraph};
use crate::snapshot::{Command, EdgeSnapshot, NodeSnapshot, Snapshot};
use crate::traits::{FlowSolver, HydraulicSolution, ReactionModel, ThermoModel};
use crate::units::*;

pub struct EngineConfig {
    pub dt: Seconds,
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self { dt: Seconds(0.1) }
    }
}

pub struct Engine {
    pub graph: PlantGraph,
    pub slate: Slate,
    config: EngineConfig,
    flow_solver: Box<dyn FlowSolver>,
    #[allow(dead_code)] // slot reserved; used from M2/M4
    thermo: Box<dyn ThermoModel>,
    #[allow(dead_code)]
    reactions: Box<dyn ReactionModel>,
    tick: u64,
    last_solution: Option<HydraulicSolution>,
}

impl Engine {
    pub fn new(
        graph: PlantGraph,
        slate: Slate,
        config: EngineConfig,
        flow_solver: Box<dyn FlowSolver>,
        thermo: Box<dyn ThermoModel>,
        reactions: Box<dyn ReactionModel>,
    ) -> Self {
        Self {
            graph,
            slate,
            config,
            flow_solver,
            thermo,
            reactions,
            tick: 0,
            last_solution: None,
        }
    }

    pub fn apply(&mut self, cmd: Command) -> Result<(), SimError> {
        match cmd {
            Command::SetValveOpening { node, opening } => {
                if !(0.0..=1.0).contains(&opening) || !opening.is_finite() {
                    return Err(SimError::InvalidCommand(format!(
                        "valve opening {opening} outside [0,1]"
                    )));
                }
                match &mut self.graph.node_mut(node).kind {
                    NodeKind::Valve { opening: o, .. } => {
                        *o = opening;
                        Ok(())
                    }
                    _ => Err(SimError::InvalidCommand(format!("{node:?} is not a valve"))),
                }
            }
            Command::SetPumpOn { node, on } => match &mut self.graph.node_mut(node).kind {
                NodeKind::Pump { on: o, .. } => {
                    *o = on;
                    Ok(())
                }
                _ => Err(SimError::InvalidCommand(format!("{node:?} is not a pump"))),
            },
            Command::PuncturePipe { edge, area } => {
                if area.value() < 0.0 || !area.value().is_finite() {
                    return Err(SimError::InvalidCommand(
                        "leak area must be finite, >= 0".into(),
                    ));
                }
                self.graph.pipe_mut(edge).leak_area = area;
                Ok(())
            }
            Command::SetHeatInput { node, power } => {
                self.graph.node_mut(node).heat_input = power;
                Ok(())
            }
        }
    }

    pub fn tick(&mut self) -> Result<(), SimError> {
        let dt = self.config.dt;

        // 1. Quasi-steady hydraulic solve (read-only over graph).
        let solution = self.flow_solver.solve(&self.graph, &self.slate, dt)?;

        // 2. Apply flows to edge streams.
        for eid in self.graph.edge_ids().collect::<Vec<_>>() {
            let flow = *solution
                .edge_mass_flow
                .get(&eid)
                .ok_or_else(|| SimError::Numerical(format!("solver omitted edge {eid:?}")))?;
            let pipe = self.graph.pipe_mut(eid);
            pipe.stream.mass_flow = KgPerSec(flow);
            // Composition/temperature transport: M1 water is trivial
            // (single component, isothermal); real advection lands in M2/M3.
        }

        // 3. Unit dynamics: integrate slow states (tank inventories).
        for nid in self.graph.node_ids().collect::<Vec<_>>() {
            let net_in: f64 = self
                .graph
                .incident(nid)
                .iter()
                .map(|(eid, _, incoming)| {
                    let q = self.graph.pipe(*eid).stream.mass_flow.value();
                    if *incoming {
                        q
                    } else {
                        -q
                    }
                })
                .sum();
            if let NodeKind::Tank(tank) = &mut self.graph.node_mut(nid).kind {
                // Explicit Euler on inventory; adequate for slow tank dynamics.
                tank.mass = Kg((tank.mass.value() + net_in * dt.value()).max(0.0));
            }
        }

        // 4. Validation: nothing non-finite escapes a tick.
        for eid in self.graph.edge_ids() {
            if !self.graph.pipe(eid).stream.all_finite() {
                return Err(SimError::NonFiniteState {
                    location: format!("edge {eid:?}"),
                });
            }
        }

        self.last_solution = Some(solution);
        self.tick += 1;
        Ok(())
    }

    pub fn snapshot(&self) -> Snapshot {
        let sol = self.last_solution.as_ref();
        let nodes = self
            .graph
            .node_ids()
            .map(|id| {
                let n = self.graph.node(id);
                NodeSnapshot {
                    id,
                    name: n.name.clone(),
                    kind: n.kind.clone(),
                    pressure_pa: sol
                        .and_then(|s| s.node_pressure.get(&id))
                        .map_or(f64::NAN, |p| p.value()),
                }
            })
            .collect();
        let edges = self
            .graph
            .edge_ids()
            .map(|id| {
                let p = self.graph.pipe(id);
                let (from, to) = self.graph.endpoints(id);
                EdgeSnapshot {
                    id,
                    name: p.name.clone(),
                    from,
                    to,
                    stream: p.stream.clone(),
                    leak_mass_flow: 0.0, // populated when leak paths land
                }
            })
            .collect();
        let tanks = self
            .graph
            .node_ids()
            .filter_map(|id| match &self.graph.node(id).kind {
                NodeKind::Tank(t) => Some((self.graph.node(id).name.clone(), t.clone())),
                _ => None,
            })
            .collect();
        Snapshot {
            tick: self.tick,
            sim_time: Seconds(self.tick as f64 * self.config.dt.value()),
            nodes,
            edges,
            solver: sol.map(|s| s.diagnostics.clone()).unwrap_or_default(),
            tanks,
        }
    }

    pub fn dt(&self) -> Seconds {
        self.config.dt
    }
}
