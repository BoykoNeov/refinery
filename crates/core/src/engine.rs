//! The engine: owns the graph and solver implementations, advances time.
//!
//! Tick sequence (see docs/DESIGN.md §1):
//!   commands → hydraulic solve (quasi-steady) → transport → unit dynamics
//!   → validation → snapshot available.

use crate::components::Slate;
use crate::energy::{self, T_REF};
use crate::error::SimError;
use crate::graph::{NodeId, NodeKind, PlantGraph};
use crate::snapshot::{Command, EdgeSnapshot, NodeSnapshot, Snapshot};
use crate::traits::{FlowSolver, HydraulicSolution, ReactionModel, ThermoModel};
use crate::units::*;
use std::collections::BTreeMap;

/// Inventory below which a tank has no meaningful temperature [kg].
///
/// `T = T_REF + E/(m·cp)` is singular at `m = 0`, and explicit Euler can
/// overshoot a nearly-empty tank into the mass clamp — at which point mass and
/// energy have both stopped being conserved and the ratio is meaningless, not
/// merely imprecise. Below a milligram the tank is empty for any refinery
/// purpose, so its last temperature is held instead of dividing by ~0.
const MIN_THERMAL_MASS_KG: f64 = 1e-6;

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
    /// Still a reserved slot at M2: transport uses constant-property `cp` off
    /// `Composition` (ideal mixing), which is exactly what `ThermoModel`'s doc
    /// says to leave alone until a consumer needs more. It takes over when
    /// T-dependent or non-ideal properties arrive.
    #[allow(dead_code)]
    thermo: Box<dyn ThermoModel>,
    #[allow(dead_code)] // slot reserved; used from M4
    reactions: Box<dyn ReactionModel>,
    tick: u64,
    last_solution: Option<HydraulicSolution>,
    /// Resolved node temperature field [K] from the last tick. Derived state,
    /// not inventory — the symmetric counterpart of `node_pressure` living in
    /// `HydraulicSolution` rather than on the nodes. Retained across ticks only
    /// to give a zero-volume node with no inflow a reproducible value to hold
    /// (see `energy::resolve_node_temperatures`).
    node_temperature: BTreeMap<NodeId, Kelvin>,
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
            node_temperature: BTreeMap::new(),
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
                if !power.value().is_finite() {
                    return Err(SimError::InvalidCommand("heat input must be finite".into()));
                }
                self.graph.node_mut(node).heat_input = power;
                Ok(())
            }
            Command::SetFurnaceDuty { node, duty } => {
                if !duty.value().is_finite() {
                    return Err(SimError::InvalidCommand(
                        "furnace duty must be finite".into(),
                    ));
                }
                match &mut self.graph.node_mut(node).kind {
                    NodeKind::Furnace { duty: d } => {
                        *d = duty;
                        Ok(())
                    }
                    _ => Err(SimError::InvalidCommand(format!(
                        "{node:?} is not a furnace"
                    ))),
                }
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
            // Composition transport is still trivial at M2 (single-component
            // water); it lands with the pseudo-component slate in M3.
        }

        // 2b. Resolve the node temperature field: inertial nodes contribute
        //     their start-of-tick temperature, zero-volume nodes mix their
        //     inflows in flow order (docs/DESIGN.md §4a).
        let node_temperature = energy::resolve_node_temperatures(
            &self.graph,
            &self.slate,
            &solution.edge_mass_flow,
            &self.node_temperature,
        )?;

        // 2c. Transport: an edge's stream takes its UPWIND node's temperature,
        //     picked by flow sign so reverse flow needs no special case. At
        //     exactly zero flow the pick is arbitrary — the stream carries no
        //     enthalpy either way — so it takes `from` to stay deterministic.
        for eid in self.graph.edge_ids().collect::<Vec<_>>() {
            let (from, to) = self.graph.endpoints(eid);
            let upwind = if self.graph.pipe(eid).stream.mass_flow.value() >= 0.0 {
                from
            } else {
                to
            };
            if let Some(t) = node_temperature.get(&upwind) {
                self.graph.pipe_mut(eid).stream.temperature = *t;
            }
        }

        // 3. Unit dynamics: integrate the tanks' slow states — inventory and
        //    thermal energy. Both are explicit Euler off start-of-tick values.
        //
        //    The energy balance is the first law for a well-mixed open vessel,
        //    d(m·u)/dt = Σ ṁ·h + Q, with liquid u ≈ h = cp·(T − T_REF). It
        //    needs no in/out branch: an outflow edge is upwind of the tank, so
        //    its stream already carries the tank's own temperature, and the
        //    signed flux subtracts exactly the enthalpy that leaves. That is
        //    what makes the discrete balance close to round-off (I6).
        for nid in self.graph.node_ids().collect::<Vec<_>>() {
            let heat_input = self.graph.node(nid).heat_input.value();
            let mut net_mass = 0.0; // [kg/s] into the node
            let mut net_enthalpy = 0.0; // [W] into the node
            for (eid, _other, incoming) in self.graph.incident(nid) {
                let stream = &self.graph.pipe(eid).stream;
                let flow = stream.mass_flow.value();
                let into_node = if incoming { flow } else { -flow };
                let cp = stream.composition.mixture_cp(&self.slate);
                net_mass += into_node;
                net_enthalpy +=
                    energy::enthalpy_flux(KgPerSec(into_node), cp, stream.temperature).value();
            }

            if let NodeKind::Tank(tank) = &mut self.graph.node_mut(nid).kind {
                let cp = tank.composition.mixture_cp(&self.slate).value();
                let mass_old = tank.mass.value();
                let energy_old = mass_old * cp * (tank.temperature.value() - T_REF.value());

                let mass_new = (mass_old + net_mass * dt.value()).max(0.0);
                let energy_new = energy_old + (net_enthalpy + heat_input) * dt.value();

                tank.mass = Kg(mass_new);
                if mass_new > MIN_THERMAL_MASS_KG {
                    tank.temperature = Kelvin(T_REF.value() + energy_new / (mass_new * cp));
                }
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
        for nid in self.graph.node_ids() {
            let node = self.graph.node(nid);
            // Tank inventory is integrated here, so a NaN in it would otherwise
            // escape into the next tick's hydraulics via the hydrostatic head.
            if let NodeKind::Tank(tank) = &node.kind {
                if !tank.mass.is_finite() || !tank.temperature.is_finite() {
                    return Err(SimError::NonFiniteState {
                        location: format!("tank '{}'", node.name),
                    });
                }
            }
            if !node_temperature.get(&nid).is_none_or(|t| t.is_finite()) {
                return Err(SimError::NonFiniteState {
                    location: format!("temperature at node '{}'", node.name),
                });
            }
        }

        self.node_temperature = node_temperature;
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
                    temperature_k: self
                        .node_temperature
                        .get(&id)
                        .map_or(f64::NAN, |t| t.value()),
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
