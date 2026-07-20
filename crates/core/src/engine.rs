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
            // A heat SOURCE, and only a source. This is the damage model's hook
            // — a fire, applied heating — and there is no such thing as a fire
            // that cools, so a negative value here can only be a sign slip. A
            // genuine net heat sink is a unit's own property (a cooler's duty)
            // or, later, ambient exchange, both of which carry their own term.
            // Zero stays legal: it is "the fire is out", the field's default.
            Command::SetHeatInput { node, power } => {
                if !power.value().is_finite() || power.value() < 0.0 {
                    return Err(SimError::InvalidCommand(format!(
                        "heat input must be finite and >= 0 — it is a heat SOURCE (a \
                         fire, applied heating); a net heat sink comes from a cooler's \
                         duty, not a negative fire. Got {} W",
                        power.value()
                    )));
                }
                self.graph.node_mut(node).heat_input = power;
                Ok(())
            }
            // Both duty commands take a non-negative MAGNITUDE; the direction is
            // the unit's, applied by `energy::heat_load`. Negative is rejected
            // rather than quietly meaning "cool with a furnace" — with a
            // dedicated `Cooler` there is no longer anything for a negative duty
            // to express, so it can only be a sign slip.
            Command::SetFurnaceDuty { node, duty } => {
                check_duty(duty, "furnace")?;
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
            Command::SetCoolerDuty { node, duty } => {
                check_duty(duty, "cooler")?;
                match &mut self.graph.node_mut(node).kind {
                    NodeKind::Cooler { duty: d } => {
                        *d = duty;
                        Ok(())
                    }
                    _ => Err(SimError::InvalidCommand(format!(
                        "{node:?} is not a cooler"
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

        // 2c. Transport: an edge's stream takes its OUTLET temperature — its
        //     upwind node's, transformed by whatever heat the pipe traded with
        //     ambient on the way (`energy::edge_temperature_at`). The upwind end
        //     is picked by flow sign, so reverse flow needs no special case.
        //
        //     This field is display only: nothing downstream in the engine reads
        //     it (the tank loop below goes through the same helper instead), so
        //     which of a pipe's two ends it reports is a presentation choice.
        //     While `ambient_ua` is 0 the transform is the identity and this is
        //     bit-identical to the isothermal transport it replaces.
        for eid in self.graph.edge_ids().collect::<Vec<_>>() {
            let (from, to) = self.graph.endpoints(eid);
            let flow = self.graph.pipe(eid).stream.mass_flow.value();
            // The outlet is the end the flow LEAVES by, which mirrors the
            // helper's own upwind pick so the two cannot disagree about which
            // way the pipe runs.
            let downstream = if flow >= 0.0 { to } else { from };
            let outlet = energy::edge_temperature_at(
                &self.graph,
                &self.slate,
                &node_temperature,
                eid,
                flow,
                downstream,
            )?;
            self.graph.pipe_mut(eid).stream.temperature = outlet;
        }

        // 3. Unit dynamics: integrate the tanks' slow states — inventory and
        //    thermal energy. Both are explicit Euler off start-of-tick values.
        //
        //    The energy balance is the first law for a well-mixed open vessel,
        //    d(m·u)/dt = Σ ṁ·h + Q, with liquid u ≈ h = cp·(T − T_REF).
        //
        //    It still needs no in/out branch, but for a narrower reason than it
        //    used to. The old one was that an outflow edge is upwind of the tank
        //    and so already carries the tank's own temperature — which was never
        //    a fact about tanks, only a consequence of edges being ISOTHERMAL,
        //    and it does not survive a pipe with an ambient `UA`. What replaces
        //    it: every incident edge goes through `energy::edge_temperature_at`,
        //    which asks which END this tank sits at. On an outflow edge the tank
        //    is upwind, no transform applies, and the signed flux subtracts
        //    exactly the enthalpy that leaves; on an inflow edge the tank is
        //    downstream and receives the transformed outlet. The branch exists,
        //    it just lives in the helper where both readers share it.
        //
        //    Debiting a tank at its outflow pipe's OUTLET would charge it for
        //    heat the pipe traded with ambient after the fluid had already left
        //    — invisible while every `ambient_ua` is 0, and a silent enthalpy
        //    error the moment one is not. That is what makes the discrete
        //    balance close to round-off (I6).
        for nid in self.graph.node_ids().collect::<Vec<_>>() {
            // Through `heat_load`, not off `heat_input` directly: that function
            // is the single owner of "how much heat enters this node", summing
            // the fire and the unit's own terms with the signs the unit implies.
            // The raw read was replaced back when the two still agreed, on the
            // grounds that it was a second answer to the same question that
            // happened to be right and would go on compiling while quietly
            // ignoring any heat term a tank later gained. Ambient exchange is
            // that term, and it reaches the balance below through this line
            // without the loop being told it exists.
            //
            // KNOWN LIMITATION: the ambient term is explicit Euler like the rest
            // of this balance, so it is only stable while UA·dt/(m·cp) < 2 — a
            // tank approaches ambient geometrically per tick, and a large enough
            // UA on a small enough inventory would oscillate about it and then
            // diverge. At refinery scale that ratio is ~1e-6 (a 1000 kg tank at
            // dt = 0.1 s needs UA > 8e7 W/K to reach it), so a guard would cost
            // a branch to catch input no plant produces. The analytic form is
            // what the PIPE transform needs, where ṁ·cp is small enough to
            // matter (docs/DESIGN.md §4a).
            let heat_input = energy::heat_load(self.graph.node(nid)).value();
            let mut net_mass = 0.0; // [kg/s] into the node
            let mut net_enthalpy = 0.0; // [W] into the node
            for (eid, _other, incoming) in self.graph.incident(nid) {
                let stream = &self.graph.pipe(eid).stream;
                let flow = stream.mass_flow.value();
                let into_node = if incoming { flow } else { -flow };
                let cp = stream.composition.mixture_cp(&self.slate);
                // The raw stored flow, not `into_node`: the helper selects the
                // upwind end from the sign, and `into_node` has been re-signed
                // positive-into-this-tank, which would name the wrong end on
                // every edge stored pointing inward.
                let crossing_t = energy::edge_temperature_at(
                    &self.graph,
                    &self.slate,
                    &node_temperature,
                    eid,
                    flow,
                    nid,
                )?;
                net_mass += into_node;
                net_enthalpy += energy::enthalpy_flux(KgPerSec(into_node), cp, crossing_t).value();
            }

            // The name is read before the tank is borrowed mutably: the guard
            // below needs it for its diagnostic, and a node cannot be borrowed
            // both ways at once.
            let node_name = self.graph.node(nid).name.clone();
            if let NodeKind::Tank(tank) = &mut self.graph.node_mut(nid).kind {
                let cp = tank.composition.mixture_cp(&self.slate).value();
                let mass_old = tank.mass.value();
                let energy_old = mass_old * cp * (tank.temperature.value() - T_REF.value());

                let mass_new = (mass_old + net_mass * dt.value()).max(0.0);
                let energy_new = energy_old + (net_enthalpy + heat_input) * dt.value();

                tank.mass = Kg(mass_new);
                // Guarded exactly like a zero-volume node's mix: a net heat SINK
                // large enough to remove more than the inventory's sensible heat
                // integrates to a finite, sub-zero Kelvin that step 4's NaN/Inf
                // check would wave straight through. The check lives with the
                // mixing one in `energy::checked_temperature` so the two paths
                // cannot drift apart on what "impossible" means.
                //
                // Inside the mass branch on purpose: a nearly-empty tank has no
                // meaningful temperature and holds its last valid one, so it has
                // no computed value to check and must not trip this.
                if mass_new > MIN_THERMAL_MASS_KG {
                    let value = T_REF.value() + energy_new / (mass_new * cp);
                    tank.temperature = energy::checked_temperature(value, || {
                        // Both sides are ENERGIES over this tick, and both are
                        // stated against the START-of-tick inventory that
                        // actually held the heat: `mass_old·cp·T_old` is what
                        // was there above 0 K, and `(net + Q)·dt` is what the
                        // tick took out. Comparing a rate against an energy, or
                        // the drawn energy against the post-drain mass, would
                        // print two numbers that do not explain each other.
                        format!(
                            "tank '{node_name}' cools to {value:.2} K, below absolute zero: over \
                             this tick a net heat load of {:.4e} W removed {:.4e} J, more than \
                             the {:.4e} J of sensible heat its {mass_old:.4e} kg held above 0 K. \
                             Reduce the heat being drawn out of it.",
                            net_enthalpy + heat_input,
                            (net_enthalpy + heat_input) * dt.value(),
                            energy_old + mass_old * cp * T_REF.value(),
                        )
                    })?;
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

/// Validate a heater/cooler duty setpoint: finite and non-negative.
///
/// Shared by both duty commands so the two can never disagree about what a
/// legal setpoint is. `unit` names the kind in the message, since "duty must be
/// >= 0" is only actionable if the operator knows which node rejected it.
fn check_duty(duty: Watt, unit: &str) -> Result<(), SimError> {
    if !duty.value().is_finite() || duty.value() < 0.0 {
        return Err(SimError::InvalidCommand(format!(
            "{unit} duty must be finite and >= 0 (it is a magnitude; the \
             direction is the unit's), got {} W",
            duty.value()
        )));
    }
    Ok(())
}
