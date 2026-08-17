//! The engine: owns the graph and solver implementations, advances time.
//!
//! Tick sequence (see docs/DESIGN.md §1):
//!   commands → hydraulic solve (quasi-steady) → transport → unit dynamics
//!   → validation → snapshot available.

use crate::components::Slate;
use crate::energy::{self, T_REF};
use crate::error::SimError;
use crate::graph::{LeakRole, NodeKind, PlantGraph};
use crate::snapshot::{Command, EdgeSnapshot, NodeSnapshot, Snapshot};
use crate::traits::{FlowSolver, HydraulicSolution, ReactionModel, ThermoModel};
use crate::units::*;

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
    /// Resolved node temperature [K] and composition fields from the last tick.
    /// Derived state, not inventory — the symmetric counterpart of
    /// `node_pressure` living in `HydraulicSolution` rather than on the nodes.
    /// Retained across ticks only to give a zero-volume node with no inflow a
    /// reproducible value to hold (see `energy::resolve_node_states`).
    node_states: energy::NodeStates,
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
            node_states: energy::NodeStates::default(),
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
                    // Refused with its OWN reason rather than falling into "not a
                    // valve", which would be both wrong and confusing: a relief
                    // valve IS a valve, and the point is that its opening is not
                    // an operator setpoint at all. It is a memoryless function of
                    // its own inlet pressure, recomputed every solve
                    // (docs/DESIGN.md §3a fork 5) — so a command that appeared to
                    // set it would be silently overwritten on the next tick.
                    NodeKind::ReliefValve { .. } => Err(SimError::InvalidCommand(format!(
                        "{node:?} is a relief valve: its opening is actuated by its own inlet \
                         pressure, not by command, and would be recomputed on the next solve"
                    ))),
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
            // `edge` names the PIPE the scenario declared, exactly as the JSON
            // contract has always said, and the engine routes the area onto that
            // pipe's dormant orifice. The indirection is the whole of fork C: the
            // loader split the declared pipe in two and hung the orifice off the
            // junction between the halves, so the thing that conducts is not the
            // thing the frontend names (docs/DESIGN.md §3b). `area = 0` is repair,
            // which is why it stays legal.
            Command::PuncturePipe { edge, area } => {
                if area.value() < 0.0 || !area.value().is_finite() {
                    return Err(SimError::InvalidCommand(
                        "leak area must be finite, >= 0".into(),
                    ));
                }
                match self.graph.pipe(edge).leak {
                    LeakRole::Punctureable { orifice } => {
                        self.graph.pipe_mut(orifice).leak = LeakRole::Orifice { area };
                        Ok(())
                    }
                    // Refused rather than silently accepted, because the failure
                    // it prevents is invisible: writing an area onto a pipe with
                    // no leak path stores a number no solver reads, which is the
                    // precise defect M6.0 found this command already had.
                    LeakRole::None => Err(SimError::InvalidCommand(format!(
                        "pipe '{}' ({edge:?}) declares no leak path, so it cannot be \
                         punctured. A pipe is punctureable only where its scenario says \
                         so (`leak_to = \"<atmosphere node>\"`); the leak path is built \
                         at LOAD, because puncturing one at runtime would change the \
                         snapshot's shape mid-run (docs/DESIGN.md §3b)",
                        self.graph.pipe(edge).name
                    ))),
                    LeakRole::Orifice { .. } => Err(SimError::InvalidCommand(format!(
                        "'{}' ({edge:?}) IS a leak orifice, not a pipe that has one. \
                         Puncture the pipe the scenario declared; the engine routes the \
                         area onto its orifice",
                        self.graph.pipe(edge).name
                    ))),
                }
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

        // 1. Quasi-steady hydraulic solve (read-only over graph). `mut` because
        //    step 2b writes prescribed column draw flows back into it once the
        //    feed composition is known — the solver deliberately leaves those at
        //    zero (see `network::edge_flows`).
        let mut solution =
            self.flow_solver
                .solve(&self.graph, &self.slate, &self.node_states, dt)?;

        // 2. Apply flows to edge streams.
        for eid in self.graph.edge_ids().collect::<Vec<_>>() {
            let flow = *solution
                .edge_mass_flow
                .get(&eid)
                .ok_or_else(|| SimError::Numerical(format!("solver omitted edge {eid:?}")))?;
            // Checked here, not where it is consumed. Dissipation feeds a
            // temperature through `energy::dissipation_on`, whose missing-key
            // fallback is zero — an omitted edge would otherwise be a silently
            // unheated stream rather than a solver bug with a name on it.
            if !solution.edge_dissipation.contains_key(&eid) {
                return Err(SimError::Numerical(format!(
                    "solver omitted the frictional dissipation of edge {eid:?}"
                )));
            }
            let pipe = self.graph.pipe_mut(eid);
            pipe.stream.mass_flow = KgPerSec(flow);
        }

        // 2b. Resolve the node temperature AND composition fields: inertial
        //     nodes contribute their start-of-tick values, zero-volume nodes mix
        //     their inflows in flow order (docs/DESIGN.md §4a).
        let node_states = energy::resolve_node_states(
            &self.graph,
            &self.slate,
            &solution.edge_mass_flow,
            &solution.edge_dissipation,
            self.reactions.as_ref(),
            &self.node_states,
        )?;
        let node_temperature = &node_states.temperature;

        // 2b′. Column draws: prescribe ṁ_drawᵢ = splitᵢ · ṁ_feed_now, split by the
        //      feed composition RESOLVED just above (DESIGN §5). This is the one
        //      place the draw flows can be finalized: the split needs the feed
        //      composition, which the sweep only just produced, and the draw
        //      FLOW and the draw COMPOSITION (read in transport below) must be
        //      built from the SAME feed composition or per-component mass fails
        //      to balance at the fixed, zero-volume column. So it runs after the
        //      sweep and before transport; the hydraulic solver reports these
        //      edges as zero (`network::edge_flows`) rather than a bogus
        //      pressure-driven number.
        //
        //      The sweep above is insensitive to the draw magnitudes it ran with
        //      (zero): a column mixes only its inflows, and every draw outlet is
        //      a fixed product node (the loader rejects a free node on a draw
        //      line), which is inertial and breaks any downstream dependency — so
        //      no resolved state changes now that the real flows are written.
        let mut draw_writes: Vec<(crate::graph::EdgeId, f64)> = Vec::new();
        for nid in self.graph.node_ids().collect::<Vec<_>>() {
            let (draws, smearing) = match &self.graph.node(nid).kind {
                NodeKind::Column {
                    draws, smearing, ..
                } => (draws.clone(), *smearing),
                _ => continue,
            };
            let feed_comp = node_states.composition.get(&nid).ok_or_else(|| {
                SimError::Numerical(format!(
                    "internal: column '{}' unresolved in the composition sweep",
                    self.graph.node(nid).name
                ))
            })?;
            let separation = energy::column_separation(&self.slate, feed_comp, &draws, smearing)?;

            // Sum the feed inflow and collect the draw edges. The feed is the one
            // incident edge whose far end is NOT a draw outlet (validate_degrees
            // guarantees exactly one). Draw edges carry the split out of the
            // column regardless of the direction they were stored in.
            let mut feed_into = 0.0;
            let mut draw_edges: Vec<(crate::graph::EdgeId, usize, f64)> = Vec::new();
            for (eid, other, incoming) in self.graph.incident(nid) {
                if let Some(idx) = draws.iter().position(|d| d.outlet == other) {
                    // +1 when the column is the edge's source (graph-direction
                    // flow leaves the column), −1 when it is the target.
                    let sign_out = if incoming { -1.0 } else { 1.0 };
                    draw_edges.push((eid, idx, sign_out));
                } else {
                    let flow = self.graph.pipe(eid).stream.mass_flow.value();
                    feed_into += if incoming { flow } else { -flow };
                }
            }
            if feed_into < 0.0 {
                return Err(SimError::Numerical(format!(
                    "column '{}' has reverse feed flow ({feed_into:.4e} kg/s): the simple \
                     column splits its feed by boiling range, which names nothing when the \
                     feed runs backwards. Check the upstream pressures.",
                    self.graph.node(nid).name
                )));
            }
            for (eid, idx, sign_out) in draw_edges {
                draw_writes.push((eid, sign_out * separation[idx].split * feed_into));
            }
        }
        for (eid, flow) in draw_writes {
            self.graph.pipe_mut(eid).stream.mass_flow = KgPerSec(flow);
            // Keep the hydraulic solution consistent with the streams, so any
            // reader of `edge_mass_flow` (a mass-balance check, a frontend) sees
            // the prescribed draw and not the solver's placeholder zero.
            solution.edge_mass_flow.insert(eid, flow);
        }

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
                node_temperature,
                &node_states.composition,
                eid,
                flow,
                dissipation_of(&solution, eid),
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
                                        // Per-component mass rate arriving [kg/s], and its total. Only
                                        // INFLOWS contribute: an outflow leaves at the tank's own
                                        // composition, which removes mass without moving the fractions, so
                                        // subtracting it here would be double-counting a change that is
                                        // already the identity.
            let mut inflow_component_rate = vec![0.0; self.slate.len()];
            let mut inflow_mass_rate = 0.0; // [kg/s]
            let mut outflow_mass_rate = 0.0; // [kg/s], positive magnitude
            for (eid, _other, incoming) in self.graph.incident(nid) {
                let stream = &self.graph.pipe(eid).stream;
                let flow = stream.mass_flow.value();
                let into_node = if incoming { flow } else { -flow };
                // Off the RESOLVED upwind composition, through the same helper
                // the sweep used: the pipe's stored composition is last tick's,
                // and charging an arriving stream the heat capacity of the
                // fluid it replaced is wrong on the very first tick it changes.
                let cp = energy::stream_cp_at(
                    &self.graph,
                    &self.slate,
                    &node_states.composition,
                    eid,
                    flow,
                )?;
                // The raw stored flow, not `into_node`: the helper selects the
                // upwind end from the sign, and `into_node` has been re-signed
                // positive-into-this-tank, which would name the wrong end on
                // every edge stored pointing inward.
                let crossing_t = energy::edge_temperature_at(
                    &self.graph,
                    &self.slate,
                    node_temperature,
                    &node_states.composition,
                    eid,
                    flow,
                    dissipation_of(&solution, eid),
                    nid,
                )?;
                net_mass += into_node;
                net_enthalpy += energy::enthalpy_flux(KgPerSec(into_node), cp, crossing_t).value();

                if into_node > 0.0 {
                    let arriving = energy::edge_composition_at(
                        &self.graph,
                        &self.slate,
                        &node_states.composition,
                        eid,
                        flow,
                    )?;
                    for (rate, fraction) in
                        inflow_component_rate.iter_mut().zip(arriving.fractions())
                    {
                        *rate += into_node * fraction;
                    }
                    inflow_mass_rate += into_node;
                } else {
                    outflow_mass_rate -= into_node;
                }
            }

            // The name is read before the tank is borrowed mutably: the guard
            // below needs it for its diagnostic, and a node cannot be borrowed
            // both ways at once.
            let node_name = self.graph.node(nid).name.clone();
            if let NodeKind::Tank(tank) = &mut self.graph.node_mut(nid).kind {
                let mass_old = tank.mass.value();
                // The inventory's enthalpy at the composition that actually
                // held it. `cp_new` below is the composition it ends the tick
                // with — the two differ only while a tank's contents are
                // changing, and using one for both would book the enthalpy of a
                // mixture that was never in the vessel.
                let cp_old = tank.composition.mixture_cp(&self.slate).value();
                let energy_old = mass_old * cp_old * (tank.temperature.value() - T_REF.value());

                // Composition: a per-component mass balance over the tick,
                // explicit Euler like every other slow state here.
                //
                //   m_c_new = f_c_old·(m_old − ṁ_out·dt) + ṁ_c,in·dt
                //
                // The outflow term is what makes this close. Fluid LEAVES at the
                // tank's start-of-tick composition — that is what the upwind
                // rule put on the outflow edge, and what the reservoir at the
                // far end was credited with — so the inventory must be debited
                // at the same one. Blending inflow against the full `m_old` and
                // letting the total mass update handle the outflow separately is
                // the natural-looking alternative, and it debits the outflow at
                // the END-of-tick composition instead: total mass still balances
                // exactly, and the per-component books are off by
                // `ṁ_out·dt·(f_new − f_old)` every tick. I7 is what catches it.
                //
                // The weights sum to `m_old + (ṁ_in − ṁ_out)·dt`, which is
                // `mass_new` — so normalizing them is dividing by the very
                // inventory these fractions describe.
                //
                // Skipped entirely with no inflow, rather than run with a zero
                // inflow term: nothing arrived, so the fractions cannot have
                // moved, and re-normalizing `f·k` would rewrite them with a
                // rounding error's worth of drift on every tick a tank merely
                // drains.
                if inflow_mass_rate > 0.0 {
                    tank.composition = energy::blended_holdup_composition(
                        &tank.composition,
                        mass_old,
                        &inflow_component_rate,
                        outflow_mass_rate,
                        dt.value(),
                    )
                    .map_err(|e| {
                        SimError::Numerical(format!(
                            "tank '{node_name}' blended to no valid composition: {e}"
                        ))
                    })?;
                }
                let cp = tank.composition.mixture_cp(&self.slate).value();

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
                            energy_old + mass_old * cp_old * T_REF.value(),
                        )
                    })?;
                }
            } else if let NodeKind::Vessel(vessel) = &mut self.graph.node_mut(nid).kind {
                // The tank's balance over a compressible substance. Mass,
                // composition and energy integrate identically — the fluxes above
                // were accumulated with no idea which kind of holdup they were
                // for — and exactly one thing differs: the inventory's energy is
                // its INTERNAL energy, not its enthalpy.
                //
                // That single substitution is what makes blowdown cooling emerge
                // rather than be modelled (docs/DESIGN.md §3a fork 3). Nothing
                // here computes a temperature drop; the vessel simply loses more
                // enthalpy through the nozzle than it held as internal energy, and
                // `T/Tᵢ = (m/mᵢ)^(γ−1)` falls out. See
                // `energy::specific_internal_energy` for why the datum makes that
                // integral come out — with `u = cv·(T − T_REF)` it does not.
                //
                // The pressure is NOT integrated here. It is the solve's unknown,
                // and `m_new = C·P_solved` holds identically: the accumulation
                // term the residual drove to zero IS this mass update, so the two
                // cannot disagree about how much the vessel took on.
                let mass_old = vessel.mass.value();
                let cv_old = vessel.composition.mixture_cv(&self.slate);
                let cp_old = vessel.composition.mixture_cp(&self.slate);
                let energy_old = mass_old
                    * energy::specific_internal_energy(cv_old, cp_old, vessel.temperature).value();

                if inflow_mass_rate > 0.0 {
                    vessel.composition = energy::blended_holdup_composition(
                        &vessel.composition,
                        mass_old,
                        &inflow_component_rate,
                        outflow_mass_rate,
                        dt.value(),
                    )
                    .map_err(|e| {
                        SimError::Numerical(format!(
                            "vessel '{node_name}' blended to no valid composition: {e}"
                        ))
                    })?;
                }
                let cv = vessel.composition.mixture_cv(&self.slate);
                let cp = vessel.composition.mixture_cp(&self.slate);

                let mass_new = (mass_old + net_mass * dt.value()).max(0.0);
                let energy_new = energy_old + (net_enthalpy + heat_input) * dt.value();

                vessel.mass = Kg(mass_new);
                if mass_new > MIN_THERMAL_MASS_KG {
                    let value =
                        energy::temperature_from_internal_energy(energy_new, mass_new, cv, cp);
                    vessel.temperature = energy::checked_temperature(value, || {
                        format!(
                            "vessel '{node_name}' cools to {value:.2} K, below absolute zero: over \
                             this tick a net heat load of {:.4e} W removed {:.4e} J from the \
                             {:.4e} J of internal energy its {mass_old:.4e} kg held above 0 K. A \
                             vessel blowing down DOES cool — that is the model working — but not \
                             through zero; check the step size and the discharge resistance.",
                            net_enthalpy + heat_input,
                            (net_enthalpy + heat_input) * dt.value(),
                            mass_old * cv_old.value() * vessel.temperature.value(),
                        )
                    })?;
                }
            }
        }

        // 3b. Publish each stream's composition: its upwind node's, unchanged —
        //     a pipe trades heat with ambient, never mass, so there is no
        //     transform to apply.
        //
        //     This field is now OUTPUT, not state that anything inside a tick
        //     reads back. Every consumer of "what is in this pipe" — the
        //     temperature sweep, the tank loop, the pipe transform — goes to the
        //     resolved upwind node through `energy::stream_cp_at` or
        //     `edge_composition_at` instead, so where in the tick this write
        //     lands no longer changes any answer.
        //
        //     It did once. Deriving cp from this stored copy charged an arriving
        //     stream the heat capacity of the fluid it replaced, wrong on the
        //     first tick a composition changed rather than merely lagging; the
        //     reference that says so is
        //     `a_tank_changing_composition_while_heating_lands_on_its_new_heat_capacity`.
        //
        //     ONE reader still takes the lagged value: `network.rs` derives
        //     stream density from it for the hydraulic solve. That lag is
        //     structural rather than incidental — the solve opens the tick, so
        //     there is no resolved composition yet to read — and it is the same
        //     quasi-steady staleness the tank levels feeding that solve already
        //     have.
        for eid in self.graph.edge_ids().collect::<Vec<_>>() {
            let flow = self.graph.pipe(eid).stream.mass_flow.value();
            let upwind = energy::edge_composition_at(
                &self.graph,
                &self.slate,
                &node_states.composition,
                eid,
                flow,
            )?
            .into_owned();
            self.graph.pipe_mut(eid).stream.composition = upwind;
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
            // A vessel's inventory escapes into the next tick's hydraulics the
            // same way a tank's does — through `Pⁿ = m/C`, which is BOTH the
            // accumulation term's datum and the seed. A NaN there would not merely
            // propagate, it would make the residual meaningless.
            if let NodeKind::Vessel(vessel) = &node.kind {
                if !vessel.mass.is_finite() || !vessel.temperature.is_finite() {
                    return Err(SimError::NonFiniteState {
                        location: format!("vessel '{}'", node.name),
                    });
                }
            }
            if !node_temperature.get(&nid).is_none_or(|t| t.is_finite()) {
                return Err(SimError::NonFiniteState {
                    location: format!("temperature at node '{}'", node.name),
                });
            }
        }

        self.node_states = node_states;
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
                        .node_states
                        .temperature
                        .get(&id)
                        .map_or(f64::NAN, |t| t.value()),
                    // The raw field, not `energy::heat_load(n)` — see
                    // `NodeSnapshot::heat_input_w` for why the sum would be
                    // the wrong number to report.
                    heat_input_w: n.heat_input.value(),
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
                    dissipation_w: sol
                        .and_then(|s| s.edge_dissipation.get(&id))
                        .map_or(f64::NAN, |w| w.value()),
                    // The punctured pipe's convenience view of ITS OWN orifice
                    // edge's flow, read from the solution rather than recomputed
                    // — one number, published twice, so it cannot drift from the
                    // edge the balance is actually built on. Outward is positive:
                    // the loader builds the orifice junction → Atmosphere, so
                    // graph direction already IS outward and no sign flip is
                    // needed (a back-feeding leak is refused by the solve before
                    // it can reach here — docs/DESIGN.md §3b).
                    leak_mass_flow: match p.leak {
                        LeakRole::Punctureable { orifice } => sol
                            .and_then(|s| s.edge_mass_flow.get(&orifice))
                            .copied()
                            .unwrap_or(0.0),
                        LeakRole::None | LeakRole::Orifice { .. } => 0.0,
                    },
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

    /// The node states the LAST tick's sweep resolved — and therefore exactly
    /// what the NEXT tick's `FlowSolver::solve` will be handed as
    /// `previous_states`. Empty before the first tick.
    ///
    /// Read-only, and deliberately not part of `Snapshot`: a snapshot is the
    /// frontend contract and this is engine-internal ordering. It is public so a
    /// test can reproduce the engine's own `compile_edge` faithfully — without it,
    /// a test calling `network::prepare` with an empty `NodeStates` silently takes
    /// the tick-0 path and cannot observe the fallback ORDER at all
    /// (docs/DESIGN.md §3a fork 6).
    pub fn node_states(&self) -> &energy::NodeStates {
        &self.node_states
    }
}

/// The friction power [W] this solve put into `edge`'s stream.
///
/// Every edge is present by the check at the top of `tick`, so the fallback is
/// unreachable; it exists so the three readers below share one lookup instead of
/// three copies of the same `get`.
fn dissipation_of(solution: &HydraulicSolution, edge: crate::graph::EdgeId) -> Watt {
    solution
        .edge_dissipation
        .get(&edge)
        .copied()
        .unwrap_or(Watt::ZERO)
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
