//! The engine: owns the graph and solver implementations, advances time.
//!
//! Tick sequence (see docs/DESIGN.md §1):
//!   commands → hydraulic solve (quasi-steady) → transport → unit dynamics
//!   → validation → snapshot available.

use crate::components::{Composition, Slate};
use crate::energy::{self};
use crate::error::SimError;
use crate::graph::{
    ControlMode, ControlledValue, LeakRole, LoopId, MeasuredVariable, NodeId, NodeKind, PlantGraph,
    TripAction, TripId, TripState,
};
use crate::snapshot::{
    CavitationSnapshot, ColumnDuty, Command, ComponentSnapshot, ControlSnapshot, EdgeSnapshot,
    NodeSnapshot, Snapshot, TripSnapshot,
};
use crate::traits::{
    BoilOffModel, EnthalpyModel, FlowSolver, HydraulicSolution, ReactionModel, SeparationModel,
    ThermoModel,
};
use crate::units::*;

/// Inventory below which a tank has no meaningful temperature [kg].
///
/// `T = T_REF + E/(m·cp)` is singular at `m = 0`, so below a milligram — empty
/// for any refinery purpose — a holdup does not divide by ~0. A wet tank holds
/// its last temperature; a STARVED tank takes the composition and temperature
/// of what is passing through it, which is the fluid now in its lines (M24,
/// docs/DESIGN.md §28 fork 4).
///
/// Until M24 this comment said that explicit Euler could overshoot a nearly
/// empty tank into the mass clamp, "at which point mass and energy have both
/// stopped being conserved". That was B29, and it created 200 149 kg on
/// `tank_flow_control`. A tank the solve would draw past empty is now starved
/// inside the solve, and the clamp is a tripwire (`checked_holdup_mass`).
const MIN_THERMAL_MASS_KG: f64 = 1e-6;

/// How far below zero a component's remaining mass may land before it stops
/// being a rounding error, as a fraction of the holdup (M12.1).
///
/// Seven orders above the double-precision spacing of a mass and seven below any
/// over-draw a model could make on purpose, so it separates the two without
/// being fitted to either.
const ROUNDING_MASS_FRACTION: f64 = 1e-9;

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
    /// Reserved and UNREAD from M1 to M7.0 — a slot with `#[allow(dead_code)]`
    /// and zero call sites for six milestones. M7.1 is where it becomes a live
    /// dependency: it is handed to `SeparationModel::separate`, because a K-value
    /// is a thermophysical property and belongs on this trait rather than on the
    /// separation seam (DESIGN §5, fork 2). The splitter ignores it; M7.2 gave
    /// the trait its first method, `k_value`, and M7.3's cascade is what reads
    /// it — so through M7.2 every scenario still selects `ConstantThermo`, whose
    /// `k_value` refuses rather than answers.
    ///
    /// Transport still takes constant-property `cp` off `Composition` (ideal
    /// mixing), which is what `ThermoModel`'s doc says to leave alone until a
    /// consumer needs more.
    thermo: Box<dyn ThermoModel>,
    #[allow(dead_code)] // slot reserved; used from M4
    reactions: Box<dyn ReactionModel>,
    /// How a column divides its feed among its draws. Reaches exactly one call
    /// site — the composition sweep — and its result travels to the sweep's two
    /// consumers through `NodeStates::column_separation`.
    separation: Box<dyn SeparationModel>,
    /// What a liquid holdup does above its own bubble point (M12,
    /// docs/DESIGN.md §14). Reaches one call site — the `Tank` arm of step 3 —
    /// and `NoBoilOff` is what every plant written before M12 selects.
    boiloff: Box<dyn BoilOffModel>,
    /// The specific enthalpy of a mixture, and every capacity derived from it
    /// (M16.2, docs/DESIGN.md §20). Reaches every energy path in the engine —
    /// the transport sweep, both holdup branches, the reactor's duty and the
    /// boil-off's flash — because it owns the datum all of them share.
    ///
    /// `ConstantEnthalpy` is what nineteen of the twenty shipped plants select,
    /// and it reproduces the pre-M16 arithmetic bit for bit rather than merely
    /// the same number.
    enthalpy: Box<dyn EnthalpyModel>,
    tick: u64,
    last_solution: Option<HydraulicSolution>,
    /// Resolved node temperature [K] and composition fields from the last tick.
    /// Derived state, not inventory — the symmetric counterpart of
    /// `node_pressure` living in `HydraulicSolution` rather than on the nodes.
    /// Retained across ticks only to give a zero-volume node with no inflow a
    /// reproducible value to hold (see `energy::resolve_node_states`).
    node_states: energy::NodeStates,
    /// The cavitation criterion at every node that has one, from the last tick
    /// (M11, docs/DESIGN.md §13). Empty before the first tick, and missing an
    /// entry for every node the criterion does not apply to.
    ///
    /// **Beside `last_solution`, not inside `NodeStates`, and that placement is
    /// argued.** `NodeStates` is produced by `resolve_node_states` and consumed
    /// by the NEXT tick's `FlowSolver::solve` as `previous_states`: it has one
    /// owner and one contract. This is a diagnostic over the *pair* (solution,
    /// states), computed once both exist, and putting it in the solver's input
    /// would be a coupling nothing asked for. `reactor_duty` is the
    /// counter-precedent — it lives in `NodeStates` because the sweep is the
    /// only place its inputs meet, and here the sweep is not.
    last_cavitation: std::collections::BTreeMap<NodeId, CavitationSnapshot>,
}

impl Engine {
    #[allow(clippy::too_many_arguments)] // one argument per fidelity seam; see `[fidelity]`
    pub fn new(
        graph: PlantGraph,
        slate: Slate,
        config: EngineConfig,
        flow_solver: Box<dyn FlowSolver>,
        thermo: Box<dyn ThermoModel>,
        reactions: Box<dyn ReactionModel>,
        separation: Box<dyn SeparationModel>,
        boiloff: Box<dyn BoilOffModel>,
        enthalpy: Box<dyn EnthalpyModel>,
    ) -> Self {
        Self {
            graph,
            slate,
            config,
            flow_solver,
            thermo,
            reactions,
            separation,
            boiloff,
            enthalpy,
            tick: 0,
            last_solution: None,
            node_states: energy::NodeStates::default(),
            last_cavitation: std::collections::BTreeMap::new(),
        }
    }

    /// The engine's enthalpy model, for a consumer closing its own energy
    /// balance (M16.2).
    ///
    /// Public because the datum moved behind a seam: an external balance used to
    /// call `energy::enthalpy_flux` and reach the same arithmetic the engine did,
    /// and after §20 the only way to reach it is to ask the same model. A test
    /// that rebuilt the expression instead would be grading the engine against a
    /// second copy of the rule — which is exactly what M13 found its two `dh_vap`
    /// tests doing.
    #[must_use]
    pub fn enthalpy(&self) -> &dyn EnthalpyModel {
        self.enthalpy.as_ref()
    }

    pub fn apply(&mut self, cmd: Command) -> Result<(), SimError> {
        match cmd {
            Command::SetValveOpening { node, opening } => {
                if !(0.0..=1.0).contains(&opening) || !opening.is_finite() {
                    return Err(SimError::InvalidCommand(format!(
                        "valve opening {opening} outside [0,1]"
                    )));
                }
                // Refused while a latched trip holds this valve, unless the write
                // IS the trip's safe position (docs/DESIGN.md §26 fork 4): shutting
                // a shut valve moves nothing, and refusing it would make a
                // frontend's "close" button fail on a plant that is already safe.
                // Its own refusal, ahead of the loop guard below — a trip forces
                // its valve's loops to MANUAL, so the loop guard would not fire.
                if let Some(trip) = self.graph.latched_trip_on(node) {
                    if let Some(position) = trip.valve_position(node) {
                        if opening != position {
                            return Err(SimError::InvalidCommand(format!(
                                "{node:?} ('{}') is held at opening {position} by trip '{}', \
                                 which is latched. Reset the trip first (`reset_trip`) once \
                                 its condition has cleared; the reset moves nothing, and the \
                                 valve can then be reopened by hand",
                                self.graph.node(node).name,
                                trip.name
                            )));
                        }
                    }
                }
                // Refused when a loop in AUTO owns this opening, and the shape is
                // the relief valve's below: a write that survives until the top of
                // the next tick and is then silently overwritten is a command that
                // APPEARS to work and does not (docs/DESIGN.md §10 fork 4). In
                // MANUAL the same command drives the actuator unchanged, which is
                // what MANUAL means.
                if let Some(owner) = self
                    .graph
                    .controls()
                    .iter()
                    .find(|c| c.actuator == node && c.mode == ControlMode::Auto)
                {
                    return Err(SimError::InvalidCommand(format!(
                        "{node:?} ('{}') is actuated by control loop '{}', which is in AUTO: its \
                         opening would be overwritten at the top of the next tick. Put the loop in \
                         MANUAL first (`set_controller_mode`), or move the loop's setpoint \
                         (`set_setpoint`)",
                        self.graph.node(node).name,
                        owner.name
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
            Command::SetPumpOn { node, on } => {
                // The first guard this command has ever had (docs/DESIGN.md §26
                // fork 4). Only a START is refused: stopping a stopped pump moves
                // nothing and is the safe direction.
                if on {
                    if let Some(trip) = self.graph.latched_trip_on(node) {
                        return Err(SimError::InvalidCommand(format!(
                            "{node:?} ('{}') is held stopped by trip '{}', which is latched. \
                             Reset the trip first (`reset_trip`) once its condition has \
                             cleared; the reset restarts nothing, and the pump can then be \
                             started by hand",
                            self.graph.node(node).name,
                            trip.name
                        )));
                    }
                }
                match &mut self.graph.node_mut(node).kind {
                    NodeKind::Pump { on: o, .. } => {
                        *o = on;
                        Ok(())
                    }
                    _ => Err(SimError::InvalidCommand(format!("{node:?} is not a pump"))),
                }
            }
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
                    // A vent is not damage and is not commandable. Its area is
                    // not a handle at all: the flow through it is written by the
                    // holdup's own enthalpy balance every tick (M12), so an area
                    // stored here would be a number no solver reads — the same
                    // defect this command was found to have in M6.0.
                    LeakRole::BoilOffVent { .. } => Err(SimError::InvalidCommand(format!(
                        "'{}' ({edge:?}) is a boil-off vent, not a pipe that can be \n                         punctured. It exists because this plant selects \n                         `[fidelity] boiloff = \"flash\"`, and its flow is prescribed \n                         by the tank's enthalpy balance rather than by any area \n                         (docs/DESIGN.md §14)",
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
                // The cooler's two guards, owed since M18 made a furnace a loop's
                // actuator (docs/DESIGN.md §22 fork 4) — and one function for both
                // duty commands, so neither can grow a guard the other lacks.
                self.check_loop_owned_duty(node, duty, "furnace")?;
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
                self.check_loop_owned_duty(node, duty, "cooler")?;
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
            // A loop is put in AUTO or MANUAL. The transfer is bumpless in both
            // directions (docs/DESIGN.md §10 fork 4), and the two directions cost
            // different things:
            //
            // - AUTO→MANUAL is free, and always was: the actuator already holds
            //   the loop's last output, so a human takes over from where the loop
            //   left it and `last_output` already tracks.
            // - MANUAL→AUTO is not. A loop with memory has been sitting out the
            //   run while a human moved the valve, and taking over would step the
            //   actuator to whatever its stale memory says. So the memory is
            //   SEEDED from the position the actuator is actually at — the same
            //   back-calculation the anti-windup clamp performs, which is why fork
            //   4 does not defer bumpless transfer to a slice after the integral.
            //
            // **The measurement is read fresh here rather than taken from
            // `last_measurement`, and that is what makes the transfer exact.**
            // Commands are applied between ticks, so the state standing now is the
            // state the next tick's control pass will measure; seeding against it
            // means the next `update` recomputes the same error and returns the
            // same position. `last_measurement` is one tick older (fork 3), and
            // seeding against it would make the transfer bumpless only to the
            // extent the plant had stopped moving.
            Command::SetControllerMode { loop_id, mode } => {
                let control = self
                    .graph
                    .control(loop_id)
                    .ok_or_else(|| unknown_loop(loop_id))?;
                // **The refusal that is easy to miss** (docs/DESIGN.md §26 fork 4).
                // This command moves no equipment itself, but a loop in AUTO writes
                // its valve at the top of the next tick, so AUTO on a loop whose
                // valve a latched trip holds would reopen it one tick later. MANUAL
                // stays admitted: it is what the trip already put the loop in.
                if mode == ControlMode::Auto {
                    if let Some(trip) = self.graph.latched_trip_on(control.actuator) {
                        return Err(SimError::InvalidCommand(format!(
                            "control loop '{}' writes '{}', which trip '{}' holds and is \
                             latched. In AUTO the loop would move it at the top of the next \
                             tick. Reset the trip (`reset_trip`), put the valve where the loop \
                             should take over from, and then switch to AUTO",
                            control.name,
                            self.graph.node(control.actuator).name,
                            trip.name
                        )));
                    }
                }
                let seed = if mode == ControlMode::Auto && control.mode == ControlMode::Manual {
                    // **Refused when there is nothing to measure** (docs/DESIGN.md
                    // §23 fork 4): a furnace or cooler outlet before the first
                    // tick, or while it is stagnant. The transfer's whole promise
                    // is "bumpless from now", which needs the error standing now;
                    // seeding later, at the first measurement, would be a
                    // different promise, and seeding against a stand-in is the
                    // fabricated number §23 fork 2 refuses.
                    // A pipe's flow is the same state before the first tick (M20,
                    // §24 fork 2), and the message names the missing quantity by
                    // variable rather than calling every absence a temperature.
                    let measurement = self
                        .graph
                        .measure(
                            &self.slate,
                            &self.node_states,
                            self.last_solution.as_ref(),
                            control.measurement_point,
                            control.setpoint.variable(),
                        )?
                        .ok_or_else(|| {
                            SimError::InvalidCommand(format!(
                                "control loop '{}' has no measurement to transfer against: \
                                 '{}' has no resolved {} yet (before the first tick){}. A \
                                 bumpless transfer back-calculates the loop's memory from the \
                                 error standing NOW; step the plant first, or declare the loop \
                                 `mode = \"auto\"` in the file (docs/DESIGN.md §23 fork 4, §24 \
                                 fork 2)",
                                control.name,
                                self.graph.point_name(control.measurement_point),
                                control.setpoint.variable().noun(),
                                // Only an outlet can go absent AFTER the first
                                // tick: a flow of zero is still a measurement.
                                if control.setpoint.variable() == MeasuredVariable::Temperature {
                                    " or none this tick (no flow through it)"
                                } else {
                                    ""
                                }
                            ))
                        })?;
                    // The SAME reader pass 1 uses (docs/DESIGN.md §21, sites 3
                    // and 6): a transfer that seeded from one notion of position
                    // while the tick ran on another would step the actuator.
                    let position = self
                        .graph
                        .actuator_position(control.actuator, control.max_duty)
                        .map_err(|e| {
                            SimError::InvalidCommand(format!(
                                "control loop '{}' has no position to transfer from: {e}",
                                control.name
                            ))
                        })?;
                    Some((measurement, control.setpoint, control.action, position))
                } else {
                    None
                };
                let control = self
                    .graph
                    .control_mut(loop_id)
                    .ok_or_else(|| unknown_loop(loop_id))?;
                if let Some((measurement, setpoint, action, position)) = seed {
                    // The loop's own action, the one pass 2 will run with: a seed
                    // taken against the other sign steps the first output by
                    // `2·K·e` (docs/DESIGN.md §22 fork 1).
                    control
                        .algorithm
                        .seed_from_output(position, measurement, setpoint, action)?;
                    // The faceplate reports what the loop will hold, not what it
                    // held while it was sitting out: a transfer that reported the
                    // old output would show a jump the plant never made.
                    control.last_output = position;
                }
                control.mode = mode;
                Ok(())
            }
            Command::SetSetpoint { loop_id, value } => {
                let control = self
                    .graph
                    .control(loop_id)
                    .ok_or_else(|| unknown_loop(loop_id))?;
                // Range-checked by the graph, which is the single owner of what
                // a reachable target is — the loader applies the same check to a
                // declared setpoint, so a loop cannot load with a number this
                // command would then refuse.
                // **The refusal `ControlledValue`'s own doc said would become
                // required "the moment a second variant lands"** (docs/DESIGN.md
                // §12 fork 6). M10 is that moment: with two variants this command
                // can change WHAT a loop measures, not merely what it aims at, and
                // a loop whose setpoint is a pressure and whose measurement node is
                // a tank has no measurement at all.
                //
                // It is checked BEFORE `check_setpoint` rather than after, because
                // `check_setpoint`'s own mismatch arms would refuse this for the
                // node's sake ("not a vessel") when the actual mistake is the
                // command's — the same distinction that gives a relief valve its
                // own refusal instead of "not a valve".
                //
                // This guard is also what keeps `ControlledValue::error` sound: it
                // and the tick pass's `measure(setpoint.variable())` are the two
                // things that stop Pascals being subtracted from metres. See that
                // method for the backstop behind them.
                if value.variable() != control.setpoint.variable() {
                    return Err(SimError::InvalidCommand(format!(
                        "control loop '{}' measures {:?} and this setpoint is {:?}. A setpoint does \
                         not change what a loop MEASURES — the measurement node would then be \
                         answering for a variable it may not have at all",
                        control.name,
                        control.setpoint.variable(),
                        value.variable()
                    )));
                }
                self.graph
                    .check_setpoint(control.measurement_point, value)?;
                self.graph
                    .control_mut(loop_id)
                    .ok_or_else(|| unknown_loop(loop_id))?
                    .setpoint = value;
                Ok(())
            }
            // Re-arm a latched trip (docs/DESIGN.md §26 fork 4). It moves no
            // equipment: it lifts the refusals, and a human restarts the plant.
            Command::ResetTrip { trip_id } => {
                let trip = self.graph.trip(trip_id).ok_or_else(|| {
                    SimError::InvalidCommand(format!("{trip_id:?} names no trip on this plant"))
                })?;
                // Resetting an armed trip would do nothing, and a frontend that
                // sends it has a wrong picture of the plant, so it is told.
                if !trip.state.is_tripped() {
                    return Err(SimError::InvalidCommand(format!(
                        "trip '{}' is armed, not tripped: there is nothing to reset",
                        trip.name
                    )));
                }
                // **Read FRESH, not `last_measurement`** — the MANUAL→AUTO seed's
                // reason (M8.3): commands land between ticks, so the state now is
                // what the next tick's trip pass will measure. A reset against the
                // one-tick-old reading could re-arm a trip one tick before it
                // fires again. And through `reached`, the same comparison that
                // fires it, so a trip may be reset exactly when it would not fire.
                let measurement = self
                    .graph
                    .measure(
                        &self.slate,
                        &self.node_states,
                        self.last_solution.as_ref(),
                        trip.measurement_point,
                        trip.limit.variable(),
                    )?
                    .ok_or_else(|| {
                        SimError::Numerical(format!(
                            "internal: trip '{}' has no {} to test its reset against, but the \
                             loader admits only quantities that exist from load \
                             (docs/DESIGN.md §26 fork 2)",
                            trip.name,
                            trip.limit.variable().noun()
                        ))
                    })?;
                if trip.direction.reached(measurement, trip.limit)? {
                    return Err(SimError::InvalidCommand(format!(
                        "trip '{}' cannot be reset: its condition still holds ({:?} against a \
                         {:?} limit of {:?}). A reset re-arms a trip, and one re-armed inside \
                         its own condition would fire again on the next tick",
                        trip.name, measurement, trip.direction, trip.limit
                    )));
                }
                self.graph
                    .trip_mut(trip_id)
                    .ok_or_else(|| {
                        SimError::InvalidCommand(format!("{trip_id:?} names no trip on this plant"))
                    })?
                    .state = TripState::Armed;
                Ok(())
            }
        }
    }

    pub fn tick(&mut self) -> Result<(), SimError> {
        let dt = self.config.dt;

        // 0a. Protection (M22). Trips run FIRST, before the loops, on the same
        //     start-of-tick state (docs/DESIGN.md §26 fork 5). A trip that fires
        //     forces the loops on its valves to MANUAL, so the loop pass below
        //     already sees MANUAL and tracks the tripped position; had the loops
        //     run first, an AUTO loop would update its memory and report an
        //     output on the tripping tick that the trip then overwrote. And the
        //     solve below sees the safe state, so the flow a trip stops is zero
        //     in this tick's own snapshot.
        self.run_trips()?;

        // 0b. Regulation (M8.2). The control loops run at the TOP of the tick, on
        //    the state standing at the start of it, and write their actuators
        //    before anything is solved. See `run_control_loops` for why this is
        //    an ordering decision and not a convenience.
        self.run_control_loops(dt)?;

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
        //     A tank the solve STARVED is a mixing point here too (M24,
        //     docs/DESIGN.md §28 fork 4): its outflow is the mix of what it held
        //     and what it was fed.
        let node_states = energy::resolve_node_states(
            &self.graph,
            &self.slate,
            &solution.edge_mass_flow,
            &solution.edge_dissipation,
            self.reactions.as_ref(),
            self.separation.as_ref(),
            self.thermo.as_ref(),
            self.enthalpy.as_ref(),
            &self.node_states,
            &solution.starved,
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
            let draws = match &self.graph.node(nid).kind {
                NodeKind::Column { draws, .. } => draws.clone(),
                _ => continue,
            };
            // The SAME pass `edge_composition_at` reads for the draw compositions,
            // made once in the sweep above. Recomputing it here would put the two
            // halves of one split behind two calls free to disagree — the failure
            // M3.2 named, now prevented by there being nothing to recompute.
            let separation = node_states.column_separation.get(&nid).ok_or_else(|| {
                SimError::Numerical(format!(
                    "internal: column '{}' unresolved in the composition sweep",
                    self.graph.node(nid).name
                ))
            })?;

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
                // `get`, not `[]`: the model's draw list is parallel to the
                // column's by contract, and `edge_composition_at` refuses to
                // trust that with a panic for the same reason (rule 5).
                let cut = separation.draws.get(idx).ok_or_else(|| {
                    SimError::Numerical(format!(
                        "separation model returned {} draws for column '{}', which has {}",
                        separation.draws.len(),
                        self.graph.node(nid).name,
                        draws.len()
                    ))
                })?;
                draw_writes.push((eid, sign_out * cut.split * feed_into));
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
            // A boil-off vent is written whole — flow, composition AND
            // temperature — by the holdup dynamics below, at the state the
            // vapour actually left in (M12, docs/DESIGN.md §14 fork 4). The
            // upwind rule cannot produce that value: the tank's temperature at
            // this point in the tick is the pre-boil-off one, and the vapour
            // leaves at the bubble point the flash lands on. See step 3.
            if self.graph.pipe(eid).leak.is_boiloff_vent() {
                continue;
            }
            let (from, to) = self.graph.endpoints(eid);
            let flow = self.graph.pipe(eid).stream.mass_flow.value();
            // The outlet is the end the flow LEAVES by, which mirrors the
            // helper's own upwind pick so the two cannot disagree about which
            // way the pipe runs.
            let downstream = if flow >= 0.0 { to } else { from };
            let outlet = energy::edge_temperature_at(
                &self.graph,
                &self.slate,
                self.enthalpy.as_ref(),
                node_temperature,
                &node_states.composition,
                &node_states.column_separation,
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
        // EMITTER-FIRST over the boil-off vents, `node_ids()` order on every
        // plant that has none between two holdups (M14, docs/DESIGN.md §16
        // fork 3). A vent's stream is written at the END of its emitting tank's
        // iteration below; a tank RECEIVING that vent reads it at the top of its
        // own, so the two have to happen in that order or the receiver is one
        // tick behind and `ṁ_v·dt` of mass sits in flight on every tick. Every
        // other edge in this loop debits and credits its endpoints from the same
        // flow inside one tick, and the vent is the only one that could not.
        for nid in self.graph.holdup_evaluation_order()? {
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
                                             // Per-component mass rate LEAVING [kg/s], at each outflow's own
                                             // resolved composition. Read only for a starved tank, whose outflow
                                             // is a mix rather than its own start-of-tick fluid (M24, §28 fork 4);
                                             // summed for every holdup so the loop below has one shape.
            let mut outflow_component_rate = vec![0.0; self.slate.len()];
            for (eid, _other, incoming) in self.graph.incident(nid) {
                // A boil-off vent is two different things at its two ends, and
                // which end this loop is standing at is the whole question
                // (M14, docs/DESIGN.md §16 fork 3).
                //
                // At the EMITTER it is not a flux into this balance, and
                // counting it would remove the vapour twice: the boil-off below
                // debits the inventory directly, and the vent edge exists so
                // that an EXTERNAL mass balance (I1, a frontend, the corpus)
                // sees the mass leave by an accounted path rather than vanish.
                // Its flow is also last tick's at this point in the tick.
                //
                // At the RECEIVER it is an ordinary inflow of condensate — and
                // it is read STRAIGHT OFF THE STREAM rather than through
                // `stream_cp_at` / `edge_temperature_at` / `edge_composition_at`
                // like every other edge. All three of those resolve the UPWIND
                // NODE, which on a vent is the emitting tank: they would hand
                // back the tank's liquid `x` at the tank's own temperature,
                // which is precisely the fork-2 defect M12.1 exists to avoid.
                // The emitter wrote flow, composition, temperature and `latent`
                // onto this edge whole, at the state the vapour actually left
                // in, and the four travel together because they describe one
                // stream.
                //
                // Skipping "any vent" instead of "the vent I emit" is what
                // M12.1 paid for forty lines below, at `engine.rs`'s
                // vent-finding site: a property of the EDGE used as a property
                // of the ENDPOINT.
                let vent_emitter = self.graph.pipe(eid).leak.boiloff_vent_emitter();
                if let Some(emitter) = vent_emitter {
                    if emitter == nid {
                        continue;
                    }
                    let stream = &self.graph.pipe(eid).stream;
                    let flow = stream.mass_flow.value();
                    let into_node = if incoming { flow } else { -flow };
                    // The VAPOUR's own composition, not the upwind node's and
                    // not this holdup's: `y = K·x` is what the edge carries and
                    // what `h` must be evaluated at. The same expression I6b's
                    // boundary term uses, so the balance and the engine charge
                    // the arriving stream identically — and since M16.2 that is
                    // enforced by both reading ONE model rather than by both
                    // spelling one formula.
                    //
                    // `stream_enthalpy_flux`, NOT `enthalpy_flux` — the arriving
                    // vapour's enthalpy is `cp·(T − T_REF) + λ` on this engine's
                    // saturated-liquid datum, and dropping `λ` here is B16's
                    // defect mirrored onto the receiving end (docs/DESIGN.md
                    // §16 fork 1). It is signed the same way the sensible term
                    // below is, because the flux is linear in the stream's own
                    // flow and `incoming` is what says which way that points.
                    let flux = self
                        .enthalpy
                        .stream_enthalpy_flux(&self.slate, stream)?
                        .value();
                    net_mass += into_node;
                    net_enthalpy += if incoming { flux } else { -flux };
                    if into_node > 0.0 {
                        let arriving = stream.composition.clone();
                        for (rate, fraction) in
                            inflow_component_rate.iter_mut().zip(arriving.fractions())
                        {
                            *rate += into_node * fraction;
                        }
                        inflow_mass_rate += into_node;
                    } else {
                        outflow_mass_rate -= into_node;
                        for (rate, fraction) in outflow_component_rate
                            .iter_mut()
                            .zip(stream.composition.fractions())
                        {
                            *rate -= into_node * fraction;
                        }
                    }
                    continue;
                }
                let stream = &self.graph.pipe(eid).stream;
                let flow = stream.mass_flow.value();
                let into_node = if incoming { flow } else { -flow };
                // The raw stored flow, not `into_node`: the helper selects the
                // upwind end from the sign, and `into_node` has been re-signed
                // positive-into-this-tank, which would name the wrong end on
                // every edge stored pointing inward.
                let crossing_t = energy::edge_temperature_at(
                    &self.graph,
                    &self.slate,
                    self.enthalpy.as_ref(),
                    node_temperature,
                    &node_states.composition,
                    &node_states.column_separation,
                    eid,
                    flow,
                    dissipation_of(&solution, eid),
                    nid,
                )?;
                // Off the RESOLVED upwind composition, through the same helper
                // the sweep used: the pipe's stored composition is last tick's,
                // and charging an arriving stream the heat capacity of the
                // fluid it replaced is wrong on the very first tick it changes.
                let crossing = energy::edge_composition_at(
                    &self.graph,
                    &node_states.column_separation,
                    &node_states.composition,
                    eid,
                    flow,
                )?;
                net_mass += into_node;
                net_enthalpy += self
                    .enthalpy
                    .enthalpy_flux(&self.slate, &crossing, KgPerSec(into_node), crossing_t)?
                    .value();

                if into_node > 0.0 {
                    let arriving = energy::edge_composition_at(
                        &self.graph,
                        &node_states.column_separation,
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
                    for (rate, fraction) in
                        outflow_component_rate.iter_mut().zip(crossing.fractions())
                    {
                        *rate -= into_node * fraction;
                    }
                }
            }

            // The name is read before the tank is borrowed mutably: the guard
            // below needs it for its diagnostic, and a node cannot be borrowed
            // both ways at once.
            let node_name = self.graph.node(nid).name.clone();
            // The boil-off vent this holdup writes, if the plant has one. Found
            // before the borrow for the same reason as the name, and by ROLE
            // rather than by "an edge to an Atmosphere": a declared pipe from a
            // tank to an atmosphere node is an ordinary pressure-driven pipe and
            // must stay one.
            //
            // **Restricted to the vent this node EMITS, and that is not a
            // refinement.** Every vent on the plant is also incident to its
            // destination, and this loop runs over every node — so a node that
            // merely receives a vent must not find it here, have no boil-off of
            // its own, and write a zero over a rate the emitter published.
            // Measured before it was understood (M12.1): the tank's inventory
            // fell by 2 539 kg over 6 000 ticks while its vent reported 0.0 kg/s
            // on every one of them, so the mass balance the vent exists to close
            // was the thing being broken.
            //
            // **M12.1's `NodeKind::Tank` test was the right fix for a plant
            // whose vents all end at an `Atmosphere`, and it is NOT the fix
            // here** (M14, docs/DESIGN.md §16 fork 3). It is loop-invariant — it
            // asks about `nid`, never about the edge — so it reduces to "I am a
            // Tank and this is a vent", which on a recovery drum is satisfied by
            // the vent of every tank feeding it. `.find` would return the first
            // of those in edge order and the drum would publish its own boil-off
            // onto a source tank's edge. Ownership is the property that was
            // meant all along, and `boiloff_vent_emitter` is where it lives.
            let vent = self
                .graph
                .incident(nid)
                .into_iter()
                .find(|(eid, _, _)| self.graph.pipe(*eid).leak.boiloff_vent_emitter() == Some(nid))
                .map(|(eid, _, _)| eid);
            // What the vent carries this tick: a rate, the equilibrium vapour,
            // and the temperature it left at. `None` while nothing boils, which
            // is every tick on a plant that selects `boiloff = "none"`.
            let mut vented: Option<(KgPerSec, Composition, Kelvin, JPerKg)> = None;
            // What the solve did with this holdup, read before the borrow: a
            // starved tank's supply and own residual, or a vessel's own residual.
            // Both bound the empty-holdup tripwire below (§28 fork 5).
            let starved = solution.starved.get(&nid).copied();
            let vessel_residual = solution.vessel_residual.get(&nid).copied();
            // Gross traffic through the holdup over the tick [kg]: the sum the
            // mass update's rounding lives in.
            let gross_mass_rate = inflow_mass_rate + outflow_mass_rate;
            // What is passing through a starved tank, for when it ends the tick
            // below the thermal floor.
            let passing = (
                node_states.composition.get(&nid).cloned(),
                node_states.temperature.get(&nid).copied(),
            );
            if let NodeKind::Tank(tank) = &mut self.graph.node_mut(nid).kind {
                let mass_old = tank.mass.value();
                // The inventory's enthalpy at the composition that actually
                // held it. `cp_new` below is the composition it ends the tick
                // with — the two differ only while a tank's contents are
                // changing, and using one for both would book the enthalpy of a
                // mixture that was never in the vessel.
                let energy_old = self.enthalpy.enthalpy_stock(
                    &self.slate,
                    &tank.composition,
                    Kg(mass_old),
                    tank.temperature,
                )?;

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
                //
                // A STARVED tank is the one exception (M24, §28 fork 4): its
                // outflow is the mix the sweep resolved, not its start-of-tick
                // fluid, so it is debited at that mix, below, once its end-of-
                // tick mass says whether it has an inventory left to describe.
                if inflow_mass_rate > 0.0 && starved.is_none() {
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
                let over_draw_allowance = starved.map_or(0.0, |report| {
                    (-report.residual.value()).max(0.0) * dt.value()
                });
                let mass_new = checked_holdup_mass(
                    &format!("tank '{node_name}'"),
                    mass_old,
                    mass_old + net_mass * dt.value(),
                    mass_old + gross_mass_rate * dt.value(),
                    over_draw_allowance,
                )?;
                let energy_new = energy_old + (net_enthalpy + heat_input) * dt.value();

                if starved.is_some() {
                    if mass_new > MIN_THERMAL_MASS_KG {
                        // What is left is the inflow the solve does not see (a
                        // column draw, a received vent) and the solve's own
                        // residual here; debited at what left, component by
                        // component, and bounded like the total.
                        let weights = energy::starved_holdup_weights(
                            &tank.composition,
                            mass_old,
                            &inflow_component_rate,
                            &outflow_component_rate,
                            dt.value(),
                        );
                        let mut kept = Vec::with_capacity(weights.len());
                        for (c, weight) in weights.into_iter().enumerate() {
                            kept.push(checked_holdup_mass(
                                &format!("component {c} of tank '{node_name}'"),
                                mass_old,
                                weight,
                                mass_old + gross_mass_rate * dt.value(),
                                over_draw_allowance,
                            )?);
                        }
                        tank.composition = Composition::from_weights(&kept).map_err(|e| {
                            SimError::Numerical(format!(
                                "dry tank '{node_name}' kept no valid composition: {e}"
                            ))
                        })?;
                    } else if let (Some(composition), Some(temperature)) = passing.clone() {
                        // Below the floor it is the fluid now in its lines. A
                        // temperature loop on a dry tank reads what is passing,
                        // not a number held from the last time it had liquid.
                        tank.composition = composition;
                        tank.temperature = temperature;
                    }
                }

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
                    // The inversion, and it is the model's rather than a division
                    // here: under a shaped `cp` this is a quadratic root and under
                    // the constant one it is `T_REF + energy/(mass·cp)` to the bit
                    // (docs/DESIGN.md §20 fork 2).
                    let value = self
                        .enthalpy
                        .temperature_from_enthalpy(
                            &self.slate,
                            &tank.composition,
                            energy_new,
                            mass_new,
                        )?
                        .value();
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
                             the {:.4e} J of sensible heat its {mass_old:.4e} kg held above \
                             the datum. Reduce the heat being drawn out of it.",
                            net_enthalpy + heat_input,
                            (net_enthalpy + heat_input) * dt.value(),
                            energy_old,
                        )
                    })?;
                }

                // The boil-off (M12, docs/DESIGN.md §14). HERE, in the same tick
                // that let the holdup reach the superheated state, because the
                // constraint is algebraic: a one-tick lag would not vanish as
                // `dt → 0`, which is M3.2's test for a lag that is really a
                // defect.
                //
                // The branch below contains no arithmetic that differs between
                // the two fidelities — the flash fraction, its caps and the
                // vapour composition all live in the model (§14 fork 6, as
                // amended by fork 9). What is here is WHEN to ask and what to do
                // with the answer, which is what rule 2 means by selecting an
                // implementation rather than branching on a flag.
                //
                // **`P_ATM`, not the tank's node pressure, and that is a
                // correction to the note.** §14 fork 3 writes `T_bub(P_node, x)`;
                // a tank's node pressure is its BOTTOM pressure, which on the
                // demo's geometry is up to 80 kPa above the blanket — a
                // different question by ~10 K of bubble point. `NodeKind::Tank`
                // is documented as vented, its free surface is at atmospheric,
                // and that is the pressure a well-mixed atmospheric holdup
                // boils at. Reading the bottom pressure would also make the term
                // a function of LEVEL, so a tank would stop boiling as it filled.
                if let Some(boil) = self.boiloff.boil_off(
                    &self.slate,
                    &tank.composition,
                    Kg(mass_new),
                    tank.temperature,
                    P_ATM,
                    self.thermo.as_ref(),
                    self.enthalpy.as_ref(),
                )? {
                    let vapour_mass = boil.vapour_mass.value();
                    // Rule 5's backstop on a seam: a model that returns more
                    // vapour than the holdup holds would drive the inventory
                    // negative, and the mass update below has no branch that
                    // would notice.
                    if !vapour_mass.is_finite() || !(0.0..=mass_new).contains(&vapour_mass) {
                        return Err(SimError::Numerical(format!(
                            "boil-off model '{}' boiled {vapour_mass} kg off tank                              '{node_name}', which holds {mass_new:.4e} kg",
                            self.boiloff.name()
                        )));
                    }
                    let remaining = mass_new - vapour_mass;
                    // Per component, at the VAPOUR's composition — the fork.
                    // Debiting at the tank's own `x` conserves mass exactly and
                    // leaves the fractions untouched for ever, which every
                    // conservation test in this workspace passes.
                    //
                    // Guarded by the same inventory floor as the temperature
                    // above: a holdup that boils away to nothing has no
                    // composition and no temperature left to compute, and holds
                    // its last valid ones.
                    if remaining > MIN_THERMAL_MASS_KG {
                        //
                        // **The `max(0.0)` is a ROUNDING guard and the bound
                        // beside it is what keeps it one.** The model caps the
                        // vaporisation so that no component is over-drawn, and
                        // where that cap BINDS the subtraction below is
                        // `w·m − y·((w/y)·m)` — exactly zero in real arithmetic
                        // and a few ULP either side of it in floating point, so
                        // a bare `from_weights` refuses a composition that is
                        // correct. Anything more than a rounding error below
                        // zero is a model over-drawing a component, and that is
                        // an `Err` with the component named rather than a clamp.
                        let mut weights = Vec::with_capacity(self.slate.len());
                        for (c, (x, y)) in tank
                            .composition
                            .fractions()
                            .iter()
                            .zip(boil.vapour.fractions())
                            .enumerate()
                        {
                            let left = x * mass_new - y * vapour_mass;
                            if left < -ROUNDING_MASS_FRACTION * mass_new {
                                return Err(SimError::Numerical(format!(
                                    "boil-off model '{}' over-draws component {c} of tank                                      '{node_name}': the tank holds {:.4e} kg of it and the                                      flash takes {:.4e} kg",
                                    self.boiloff.name(),
                                    x * mass_new,
                                    y * vapour_mass
                                )));
                            }
                            weights.push(left.max(0.0));
                        }
                        tank.composition =
                            Composition::from_weights(&weights).map_err(|e| {
                                SimError::Numerical(format!(
                                    "tank '{node_name}' has no valid composition left after                                      boiling {vapour_mass:.4e} kg off {mass_new:.4e} kg: {e}.                                      The vapour is enriched, so an over-drawn component runs                                      negative before the total does"
                                ))
                            })?;
                        tank.temperature = boil.liquid_temperature;
                    }
                    tank.mass = Kg(remaining);
                    vented = Some((
                        KgPerSec(vapour_mass / dt.value()),
                        boil.vapour,
                        boil.liquid_temperature,
                        boil.latent_heat,
                    ));
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
                let energy_old = mass_old
                    * self
                        .enthalpy
                        .specific_internal_energy(
                            &self.slate,
                            &vessel.composition,
                            vessel.temperature,
                        )?
                        .value();

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
                // The vessel's clamp is a tripwire bounded by its own residual,
                // chosen after measuring that no vessel in the corpus or the
                // test suite ever comes within 0.026 kg of empty (§28 fork 5).
                let over_draw_allowance = vessel_residual
                    .map_or(0.0, |residual| (-residual.value()).max(0.0) * dt.value());
                let mass_new = checked_holdup_mass(
                    &format!("vessel '{node_name}'"),
                    mass_old,
                    mass_old + net_mass * dt.value(),
                    mass_old + gross_mass_rate * dt.value(),
                    over_draw_allowance,
                )?;
                let energy_new = energy_old + (net_enthalpy + heat_input) * dt.value();

                vessel.mass = Kg(mass_new);
                if mass_new > MIN_THERMAL_MASS_KG {
                    let value = self
                        .enthalpy
                        .temperature_from_internal_energy(
                            &self.slate,
                            &vessel.composition,
                            energy_new,
                            mass_new,
                        )?
                        .value();
                    vessel.temperature = energy::checked_temperature(value, || {
                        format!(
                            "vessel '{node_name}' cools to {value:.2} K, below absolute zero: over \
                             this tick a net heat load of {:.4e} W removed {:.4e} J from the \
                             {:.4e} J of internal energy its {mass_old:.4e} kg held above the \
                             vessel blowing down DOES cool — that is the model working — but not \
                             through zero; check the step size and the discharge resistance.",
                            net_enthalpy + heat_input,
                            (net_enthalpy + heat_input) * dt.value(),
                            energy_old,
                        )
                    })?;
                }
            }

            // The vent, written whole and written here — after the holdup update
            // it reports, which is why it is not step 2b′ where a column draw is
            // written (§14 fork 4). A draw needs the resolved FEED composition; a
            // vent needs the holdup update that precedes it.
            //
            // Written on every tick a vent exists, including the ticks nothing
            // boils: a stale rate left on the edge would keep venting mass that
            // the inventory is no longer losing.
            if let Some(eid) = vent {
                // The latent term is the FOURTH thing written here and it is
                // cleared on the ticks nothing boils, for the same reason the
                // flow is zeroed: a stale `Some(λ)` on an idle vent would claim
                // energy is leaving a tank that has stopped boiling (M13,
                // docs/DESIGN.md §15 fork 4).
                let (flow, composition, temperature, latent) = vented.map_or_else(
                    || {
                        (
                            KgPerSec(0.0),
                            self.graph.pipe(eid).stream.composition.clone(),
                            self.graph.pipe(eid).stream.temperature,
                            None,
                        )
                    },
                    |(flow, composition, temperature, latent)| {
                        (flow, composition, temperature, Some(latent))
                    },
                );
                // **Signed by the edge's own direction, not assumed outward.**
                // `build_boiloff_vents` stores every vent emitter → destination,
                // so this is `+rate` on every plant in the corpus — but the
                // ownership fix above makes a vent stored the other way round a
                // representable graph, and its receiver reads this flow through
                // the same `if incoming` rule every ordinary edge uses. Writing
                // an unconditional `+rate` would make that fixture move mass
                // backwards (docs/DESIGN.md §16 fork 3, mutation 5).
                let outward = self.graph.endpoints(eid).0 == nid;
                let pipe = self.graph.pipe_mut(eid);
                pipe.stream.mass_flow = if outward {
                    flow
                } else {
                    KgPerSec(-flow.value())
                };
                pipe.stream.composition = composition;
                pipe.stream.temperature = temperature;
                pipe.stream.latent = latent;
                // Kept consistent with the stream for the same reason a draw is:
                // any reader of `edge_mass_flow` — a mass balance, a frontend,
                // the snapshot's dissipation lookup — must see the prescribed
                // flow and not the solver's placeholder zero.
                let signed = self.graph.pipe(eid).stream.mass_flow.value();
                solution.edge_mass_flow.insert(eid, signed);
            } else if vented.is_some() {
                // A model produced a boil-off on a holdup with nowhere to vent
                // it. Refused rather than dropped: dropping it is the bare
                // decrement fork 4 rejects, and it would break I1 on a plant
                // that looks fine.
                return Err(SimError::Numerical(format!(
                    "tank '{node_name}' boiled off vapour but owns no vent edge. The loader                      builds one per tank — to the plant's `Atmosphere` by default, or to                      whatever that tank's `vent_to` names — when `[fidelity] boiloff`                      selects a model that can boil; a graph built by hand must do the same                      (docs/DESIGN.md §14 fork 4, §16 fork 2)"
                )));
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
            // Not a vent: the upwind node is the tank, so this pass would
            // overwrite the equilibrium vapour `y = K·x` with the LIQUID
            // composition it left behind. That is the fork-2 defect written
            // back onto the edge — the tank's own books would still move, but
            // every external reader, I7 included, would see the vapour leaving
            // at `x`.
            if self.graph.pipe(eid).leak.is_boiloff_vent() {
                continue;
            }
            let flow = self.graph.pipe(eid).stream.mass_flow.value();
            let upwind = energy::edge_composition_at(
                &self.graph,
                &node_states.column_separation,
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

        // 5. The cavitation criterion (M11, docs/DESIGN.md §13). A DIAGNOSTIC:
        //    nothing in the forward solve reads it, the hydraulics are untouched,
        //    and a node reported as boiling still carries whatever the solve says
        //    it carries. What the engine gains is the ability to SAY SO — §3 used
        //    to tell frontends to infer it from a negative absolute pressure,
        //    which fires late by the fluid's whole vapour pressure.
        //
        //    Here rather than in `snapshot()` because `snapshot` takes `&self`
        //    and returns no `Result`: a criterion evaluated there would have to
        //    swallow a model's `Err`, which rule 5 forbids and which would turn
        //    "this model cannot answer" into "healthy". Here both halves are in
        //    hand — the solved pressures, and the temperatures and compositions
        //    the sweep just resolved.
        let mut cavitation = std::collections::BTreeMap::new();
        for nid in self.graph.node_ids() {
            if !cavitation_subject(&self.graph.node(nid).kind) {
                continue;
            }
            let (Some(pressure), Some(temperature), Some(composition)) = (
                solution.node_pressure.get(&nid),
                node_states.temperature.get(&nid),
                node_states.composition.get(&nid),
            ) else {
                continue;
            };
            // A vapour does not cavitate — it is already vapour. `Composition`
            // is the existing owner of that question (M5.2's density dispatch
            // asks the same one), so no second notion of phase is invented here.
            // A mixed-phase composition is an `Err` everywhere else in the
            // engine and stays one.
            if composition.phase(&self.slate)? != crate::components::Phase::Liquid {
                continue;
            }
            let bubble = match self
                .thermo
                .bubble_pressure(&self.slate, composition, *temperature)
            {
                Ok(bubble) => bubble,
                // `Scenario` is the model saying it has no vapour-liquid
                // equilibrium at all — a legitimate configuration, and the one
                // fourteen of the fifteen shipped plants are in. It reports
                // nothing rather than reporting `false`, which would be a clean
                // bill of health nothing computed. Every other variant is a real
                // fault and fails the tick.
                Err(SimError::Scenario(_)) => continue,
                Err(other) => return Err(other),
            };
            cavitation.insert(
                nid,
                CavitationSnapshot {
                    bubble_pressure_pa: bubble.value(),
                    cavitating: pressure.value() < bubble.value(),
                },
            );
        }

        self.node_states = node_states;
        self.last_solution = Some(solution);
        self.last_cavitation = cavitation;
        self.tick += 1;
        Ok(())
    }

    /// Run every trip, in declaration order, on the state standing at the top of
    /// this tick (docs/DESIGN.md §26).
    ///
    /// **Write once, then refuse** (fork 3). An armed trip whose condition is
    /// reached latches and writes its safe states on this tick; after that it
    /// does not rewrite them. What holds them is `Engine::apply`, which refuses
    /// every command that would move tripped equipment. A trip that rewrote its
    /// safe state every tick would hide a missing refusal: the command would
    /// return `Ok` and be quietly undone at the next tick, which is the "command
    /// that appears to work and does not" defect `SetValveOpening`'s loop guard
    /// exists to prevent.
    ///
    /// **The hold check** backs those refusals up. On every tick a latched trip
    /// confirms its equipment is still where it put it, and fails the tick if
    /// not. It is unreachable while every refusal holds, and turns a missing one
    /// into a loud failure instead of a silent restart.
    ///
    /// Passes as in `run_control_loops`: every trip measures and checks before any
    /// trip writes, so two trips see the same state whatever their order.
    fn run_trips(&mut self) -> Result<(), SimError> {
        if self.graph.trips().is_empty() {
            // Every plant written before M22 takes this exit, which is why they
            // are byte-identical.
            return Ok(());
        }
        // The tick this pass belongs to: the counter moves at the END of
        // `tick`, so during tick N it still reads N - 1, and a snapshot of this
        // tick will carry N.
        let this_tick = self.tick + 1;

        // Pass 1 — measure, and check the equipment every LATCHED trip holds.
        let mut measured: Vec<ControlledValue> = Vec::with_capacity(self.graph.trips().len());
        for trip in self.graph.trips() {
            // Every quantity a trip may watch is stored and exists from load
            // (fork 2(c)), so `None` here is the loader's admission check and
            // `measure` disagreeing — an engine fault, not a quiet hold. A
            // safety function holding still on a missing measurement is the
            // wrong default (§26 fork 2), so it is not allowed to happen quietly.
            let measurement = self
                .graph
                .measure(
                    &self.slate,
                    &self.node_states,
                    self.last_solution.as_ref(),
                    trip.measurement_point,
                    trip.limit.variable(),
                )?
                .ok_or_else(|| {
                    SimError::Numerical(format!(
                        "internal: trip '{}' has no {} to compare with its limit, but the \
                         loader admits only quantities that exist from load \
                         (docs/DESIGN.md §26 fork 2)",
                        trip.name,
                        trip.limit.variable().noun()
                    ))
                })?;
            measured.push(measurement);
            if trip.state.is_tripped() {
                for action in &trip.actions {
                    self.check_trip_holds(&trip.name, *action)?;
                }
            }
        }

        // Pass 2 — latch the trips whose condition is reached, and collect what
        // they write. A trip already latched stays latched whatever the
        // measurement now says: that is the latch (fork 4).
        let mut writes: Vec<TripAction> = Vec::new();
        for (trip, measurement) in self.graph.trips_mut().iter_mut().zip(measured) {
            trip.last_measurement = Some(measurement);
            if trip.state == TripState::Armed && trip.direction.reached(measurement, trip.limit)? {
                trip.state = TripState::Tripped { at_tick: this_tick };
                writes.extend(trip.actions.iter().copied());
            }
        }

        // Pass 3 — write the safe states, and force every loop on a tripped
        // valve to MANUAL (fork 5). MANUAL tracks, so its faceplate shows the
        // valve's real position from this tick, and a PI loop's memory is left
        // alone: after a reset and a human reopening the valve, AUTO is the
        // existing bumpless transfer, seeded from wherever the valve stands.
        for action in writes {
            match action {
                TripAction::StopPump { pump } => match &mut self.graph.node_mut(pump).kind {
                    NodeKind::Pump { on, .. } => *on = false,
                    _ => return Err(trip_equipment_fault(&self.graph, pump, "pump")),
                },
                TripAction::SetValve { valve, position } => {
                    match &mut self.graph.node_mut(valve).kind {
                        NodeKind::Valve { opening, .. } => *opening = position,
                        _ => return Err(trip_equipment_fault(&self.graph, valve, "valve")),
                    }
                    for control in self.graph.controls_mut() {
                        if control.actuator == valve {
                            control.mode = ControlMode::Manual;
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// The hold check for one action of one latched trip: its equipment must
    /// still be in the safe state the trip wrote (docs/DESIGN.md §26 fork 3).
    fn check_trip_holds(&self, trip: &str, action: TripAction) -> Result<(), SimError> {
        let moved = match (action, &self.graph.node(action.equipment()).kind) {
            (TripAction::StopPump { .. }, NodeKind::Pump { on, .. }) => *on,
            (TripAction::SetValve { position, .. }, NodeKind::Valve { opening, .. }) => {
                *opening != position
            }
            (TripAction::StopPump { pump }, _) => {
                return Err(trip_equipment_fault(&self.graph, pump, "pump"))
            }
            (TripAction::SetValve { valve, .. }, _) => {
                return Err(trip_equipment_fault(&self.graph, valve, "valve"))
            }
        };
        if moved {
            return Err(SimError::Numerical(format!(
                "internal: trip '{trip}' is latched and '{}' is no longer in the safe state it \
                 wrote. Every command that could move it is refused while the trip is latched \
                 (docs/DESIGN.md §26 fork 4), so one of those refusals is missing",
                self.graph.node(action.equipment()).name
            )));
        }
        Ok(())
    }

    /// Run every control loop, in declaration order, on the state standing at
    /// the top of this tick.
    ///
    /// **Before the hydraulic solve, and that is a decision rather than an
    /// ordering convenience** (docs/DESIGN.md §10 fork 3). A loop reading *this*
    /// tick's solved state and writing an actuator opening would change a solver
    /// input after the solve, requiring a re-solve whose answer would change the
    /// input again — an algebraic loop, the same shape §3c rejected for
    /// per-iteration reclassification. The cost is one `dt` of measurement lag,
    /// which is what a real sampled controller has: a DCS acts on the previous
    /// scan, and that IS the physical system rather than an approximation of it.
    /// It is also the staleness §3 already accepts everywhere else — the
    /// quasi-steady solve is driven by tank levels integrated at the end of the
    /// previous tick.
    ///
    /// **Three passes rather than one, and the reason is the borrow checker
    /// telling the truth about the data flow.** Measuring reads the graph's
    /// nodes, running a controller mutates the loop's own state, and writing an
    /// opening mutates a node — one fused loop would hold a node reference
    /// across a controller call. Splitting them also makes the ordering explicit:
    /// **every loop measures before any loop writes**, so two loops on one plant
    /// see the same start-of-tick state regardless of declaration order, and
    /// declaration order decides only who wins a contested write. (Nothing can
    /// contest one today — the loader refuses two loops on one actuator.)
    fn run_control_loops(&mut self, dt: Seconds) -> Result<(), SimError> {
        if self.graph.controls().is_empty() {
            // The pre-M8 plants take this exit, which is why they are
            // byte-identical: not one line below runs for a plant with no loop.
            return Ok(());
        }

        // Pass 1 — measure, and read each actuator's current position. Both are
        // reads of the state BEFORE this tick's solve: the graph, and — for a
        // furnace or cooler outlet — the last tick's resolved states, which are
        // what this tick's solve is about to be handed (docs/DESIGN.md §23), and
        // — for a pipe's flow — the last tick's hydraulic solution (§24), `None`
        // on the first tick.
        let mut sampled: Vec<(Option<ControlledValue>, f64)> =
            Vec::with_capacity(self.graph.controls().len());
        for control in self.graph.controls() {
            let measurement = self.graph.measure(
                &self.slate,
                &self.node_states,
                self.last_solution.as_ref(),
                control.measurement_point,
                control.setpoint.variable(),
            )?;
            // One owner of "where the actuator stands", shared with the loader's
            // seed and the MANUAL→AUTO transfer. Its error arm is rule 5's
            // backstop: the loader builds only valve-without-range and
            // cooler-with-range, so it is reachable only from a hand-built graph.
            let position = self
                .graph
                .actuator_position(control.actuator, control.max_duty)?;
            sampled.push((measurement, position));
        }

        // Pass 2 — run each controller and record what it wants written.
        let mut writes: Vec<Option<(NodeId, Option<Watt>, f64)>> =
            Vec::with_capacity(sampled.len());
        for (control, (measurement, position)) in self.graph.controls_mut().iter_mut().zip(sampled)
        {
            control.last_measurement = measurement;
            match (control.mode, measurement) {
                // **No measurement, no action** (docs/DESIGN.md §23 fork 2): a
                // furnace or cooler outlet before the first tick, or while it is
                // stagnant. The loop writes nothing, its faceplate TRACKS the
                // actuator exactly as MANUAL's does, and its memory — seeded or
                // still pending — is not touched, so it resumes from where it
                // stood. Acting on a stand-in would be worse than waiting: a PI
                // loop seeds against whatever it first measures.
                (ControlMode::Auto, None) => {
                    control.last_output = position;
                    writes.push(None);
                }
                (ControlMode::Auto, Some(measurement)) => {
                    let output = control.algorithm.update(
                        measurement,
                        control.setpoint,
                        control.action,
                        dt,
                    )?;
                    // Checked here rather than trusted from the seam: the range
                    // is the actuator's, not the algorithm's, and an out-of-range
                    // position reaching `Valve::opening` is a plant state
                    // `Command::SetValveOpening` would have refused from a human.
                    if !output.is_finite() || !(0.0..=1.0).contains(&output) {
                        return Err(SimError::Numerical(format!(
                            "control loop '{}' ({}) produced actuator position {output}, \
                             which is not a finite fraction in [0, 1]",
                            control.name,
                            control.algorithm.name()
                        )));
                    }
                    control.last_output = output;
                    writes.push(Some((control.actuator, control.max_duty, output)));
                }
                // MANUAL writes nothing and TRACKS: the faceplate reports the
                // actuator's real opening, which is what a DCS shows and what
                // makes AUTO→MANUAL transfer free (fork 4). The measurement is
                // still taken, so the loop-off counterfactual is a run of the
                // same plant with a truthful faceplate rather than of a plant
                // with the loop deleted.
                (ControlMode::Manual, _) => {
                    control.last_output = position;
                    writes.push(None);
                }
            }
        }

        // Pass 3 — write the actuators, through the inverse of pass 1's reader:
        // a valve's opening is the output itself, a cooler's duty is
        // `output · max_duty`. Its error arm is unreachable — pass 1 already read
        // this same pairing, and nothing between the two passes can change a
        // node's kind — and is an `Err` rather than an `unwrap` because rule 5 is
        // about what the engine may do, not about what it can prove.
        for (actuator, max_duty, output) in writes.into_iter().flatten() {
            self.graph
                .set_actuator_position(actuator, max_duty, output)?;
        }
        Ok(())
    }

    /// The two guards a loop-owned DUTY actuator gets, shared by
    /// `SetCoolerDuty` and `SetFurnaceDuty`: refused while the owning loop is in
    /// AUTO, and refused above the loop's range in either mode. `unit` names the
    /// actuator kind in the message.
    fn check_loop_owned_duty(&self, node: NodeId, duty: Watt, unit: &str) -> Result<(), SimError> {
        // A duty actuator a loop owns gets the valve's guard, and the reason is
        // the same one: in AUTO the write survives until the top of the
        // next tick and is then silently overwritten (docs/DESIGN.md §21,
        // site 8). Before M17 no loop could own a cooler, and before M18 none
        // could own a furnace, so the guard was not owed.
        //
        // **The range refusal applies in EITHER mode** (§21 fork 4). In
        // MANUAL a human may drive the duty — that is what MANUAL means —
        // but not past the loop's declared authority: MANUAL tracking would
        // then report a position above 1, and a MANUAL→AUTO transfer would
        // back-calculate a memory from an output the loop could never have
        // produced.
        if let Some(owner) = self.graph.controls().iter().find(|c| c.actuator == node) {
            if owner.mode == ControlMode::Auto {
                return Err(SimError::InvalidCommand(format!(
                    "{node:?} ('{}') is actuated by control loop '{}', which is in AUTO: \
                     its duty would be overwritten at the top of the next tick. Put the \
                     loop in MANUAL first (`set_controller_mode`), or move the loop's \
                     setpoint (`set_setpoint`)",
                    self.graph.node(node).name,
                    owner.name
                )));
            }
            if let Some(max) = owner.max_duty {
                if duty.value() > max.value() {
                    return Err(SimError::InvalidCommand(format!(
                        "{unit} duty {} W on '{}' is above the {} W authority of control \
                         loop '{}', which owns it. The loop's output is a fraction of \
                         that range, so a duty outside it is a position the loop could \
                         never have produced and cannot transfer from",
                        duty.value(),
                        self.graph.node(node).name,
                        max.value(),
                        owner.name
                    )));
                }
            }
        }
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
                    // Both-or-neither: `Separation` documents them as always
                    // `Some` together, and `zip` states that here rather than
                    // letting a half-filled pair reach a frontend as a duty of 0.
                    column_duty: self
                        .node_states
                        .column_separation
                        .get(&id)
                        .and_then(|s| s.condenser_duty.zip(s.reboiler_duty))
                        .map(|(condenser, reboiler)| ColumnDuty {
                            condenser_w: condenser.value(),
                            reboiler_w: reboiler.value(),
                        }),
                    // Absent wherever the criterion does not apply, which is
                    // three different sentences a frontend must read as
                    // "unknown" — see `NodeSnapshot::cavitation`.
                    cavitation: self.last_cavitation.get(&id).copied(),
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
                        // A VENT is not a leak: `leak_mass_flow` is the
                        // frontend's "this pipe is spraying" number, and a
                        // boil-off vent is ordinary operation of a plant that
                        // declares `boiloff = "flash"`. Its flow is on the vent
                        // EDGE, where every other prescribed flow is read.
                        LeakRole::None
                        | LeakRole::Orifice { .. }
                        | LeakRole::BoilOffVent { .. } => 0.0,
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
        // One faceplate per loop, in declaration order — which is `LoopId` order
        // and execution order both. `measurement` is the value the controller
        // ACTED ON, carried on the loop since the top of the tick, and not a
        // fresh read of the graph: those differ by one `dt`, and reporting the
        // fresh one would hide the lag from the person debugging it.
        let controls = self
            .graph
            .controls()
            .iter()
            .enumerate()
            .map(|(i, c)| ControlSnapshot {
                id: LoopId(i as u32),
                name: c.name.clone(),
                algorithm: c.algorithm.name().to_string(),
                mode: c.mode,
                action: c.action,
                setpoint: c.setpoint,
                measurement: c.last_measurement,
                output: c.last_output,
            })
            .collect();
        // One entry per trip, in declaration order. `measurement` is what the
        // last trip pass compared, not a fresh read — the loops' rule.
        let trips = self
            .graph
            .trips()
            .iter()
            .enumerate()
            .map(|(i, t)| TripSnapshot {
                id: TripId(i as u32),
                name: t.name.clone(),
                direction: t.direction,
                limit: t.limit,
                measurement: t.last_measurement,
                state: t.state,
            })
            .collect();
        // The slate, in ITS OWN order — `Slate::iter` walks the declaration
        // order that `Composition`'s fractions index into, and a frontend zips
        // the two. Any reordering here (sorting by name, say) would silently
        // pair every tank's fractions with the wrong densities, which is the
        // failure `scenarios/tests/snapshot_slate.rs` sizes rather than assumes.
        let slate = self
            .slate
            .iter()
            .map(|c| ComponentSnapshot {
                name: c.name.clone(),
                density_kg_per_m3: c.density.map(|d| d.value()),
            })
            .collect();
        Snapshot {
            tick: self.tick,
            sim_time: Seconds(self.tick as f64 * self.config.dt.value()),
            slate,
            nodes,
            edges,
            solver: sol.map(|s| s.diagnostics.clone()).unwrap_or_default(),
            tanks,
            controls,
            trips,
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

    /// The LAST tick's hydraulic solution, as the engine finished it (draw and
    /// vent flows written in). `None` before the first tick.
    ///
    /// Read-only and not part of `Snapshot`, for the same reason as
    /// `node_states`: it is engine-internal. It is public so a test can read
    /// what the solve reported about the holdups it bounded — a starved tank's
    /// supply and own residual, a vessel's own residual (docs/DESIGN.md §28
    /// fork 5) — rather than re-deriving them from the snapshot's flows, which
    /// would compare the engine's arithmetic against a copy of itself.
    pub fn last_solution(&self) -> Option<&HydraulicSolution> {
        self.last_solution.as_ref()
    }
}

/// Which node kinds the cavitation criterion is about (docs/DESIGN.md §13
/// fork 4).
///
/// **Enumerated rather than matched with a catch-all, and the reason is
/// measured.** `docs/DEFERRED.md` B1's trigger named four kinds — pump, valve,
/// junction, exchanger — and across all fifteen shipped plants the set of nodes
/// of those kinds at which the engine can evaluate a bubble pressure is EMPTY:
/// every one of them is on a plant whose thermo model refuses, and the one plant
/// that can answer has none of those kinds in it. Its only flow-path node is a
/// FURNACE. A four-name list inherited without walking the type would have left
/// this criterion with no reachable node at all.
///
/// A new `NodeKind` must therefore be classified here deliberately; there is no
/// `_ =>` arm to fall into.
fn cavitation_subject(kind: &NodeKind) -> bool {
    match kind {
        // The zero-volume hydraulic path. A pump's node sits at its SUCTION
        // pressure (a device folds into its outlet edge), which is exactly the
        // failure §3's paragraph is about; a throttle's node sits upstream of
        // its own drop, so what flashes there is what the line delivers to it.
        NodeKind::Pump { .. }
        | NodeKind::Valve { .. }
        | NodeKind::ReliefValve { .. }
        | NodeKind::Junction
        | NodeKind::HeatExchanger => true,
        // A fired heater's outlet is where a refiner expects a liquid to boil —
        // that is what a heater is for, and vaporizing in the TUBES rather than
        // downstream is a real and expensive failure. Both are zero-volume flow-
        // path nodes like the five above; they are called out separately only
        // because B1's trigger omitted them.
        NodeKind::Furnace { .. } | NodeKind::Cooler { .. } => true,
        // Holdups, and this exclusion is load-bearing. A tank or vessel below
        // its bubble point is a TWO-PHASE INVENTORY (`docs/DEFERRED.md` B3), not
        // cavitation, which is a flow-path phenomenon. Two shipped plants are
        // already in that state — `crude_column`'s naphtha tank sits at 0.30× its
        // own bubble pressure — so without this the criterion would fire on them
        // and report a boiling product tank as a cavitating pump.
        NodeKind::Tank(_) | NodeKind::Vessel(_) => false,
        // Declared boundaries: their pressures are typed into the scenario file
        // rather than solved, so grading one grades the author's arithmetic. B1
        // carried a distance for five milestones that was a declared sink's
        // pressure, which is the cautionary case.
        NodeKind::Source { .. } | NodeKind::Sink { .. } | NodeKind::Atmosphere => false,
        // A column is AT its bubble point by definition — that is what a column
        // is. Measured over 6 000 ticks, `crude_column_cascade`'s sits between
        // 0.988 and 1.000 of it, so a signal here would report the model working.
        NodeKind::Column { .. } => false,
        // A reactor IMPOSES its outlet temperature, so the temperature the
        // criterion would read is a setpoint rather than a resolved state; and
        // the FCC slate declares a `gas` lump with tb = -40 °C as a liquid, which
        // makes a liquid bubble-point test meaningless there (B3 again).
        NodeKind::Reactor { .. } => false,
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

/// A trip action naming a node of the wrong kind. The loader builds only a pump
/// under a pump action and a valve under a valve action, so this is reachable
/// only from a hand-built graph, and said rather than skipped (rule 5).
fn trip_equipment_fault(graph: &PlantGraph, node: NodeId, kind: &str) -> SimError {
    SimError::Scenario(format!(
        "a trip action names '{}' as a {kind}, and it is not one",
        graph.node(node).name
    ))
}

/// The refusal for a `LoopId` that names no loop.
///
/// A `LoopId` arrives from outside the engine, so an out-of-range one is a
/// command to refuse rather than a bug to index-panic on (rule 5). Shared by both
/// loop commands so the two cannot phrase the same fault differently.
fn unknown_loop(loop_id: LoopId) -> SimError {
    SimError::InvalidCommand(format!("{loop_id:?} names no control loop on this plant"))
}

/// A holdup's end-of-tick mass [kg]: `raw` clamped at zero, but only by as
/// much as the tick could legitimately have over-drawn it (M24,
/// docs/DESIGN.md §28 fork 5).
///
/// Until M24 this was a silent `.max(0.0)`, and it created 200 149 kg on
/// `tank_flow_control` (B29). Now:
///
/// - `raw ≥ 0` returns `raw` unchanged, so every holdup that does not run dry
///   updates exactly as before.
/// - A shortfall up to `ROUNDING_MASS_FRACTION·gross + over_draw_allowance` is
///   clamped. `gross` is the holdup plus the tick's traffic through it
///   (`m + Σ|ṁ|·dt`), because that is the sum the rounding is in — the boil-off
///   guard's rule. `over_draw_allowance` is the holdup's OWN solve residual
///   times `dt` where the solve drew more than it held (a starved tank's or a
///   vessel's), never the plant's worst node.
/// - Anything beyond is an `Err` naming the holdup and both numbers: a solver
///   delivered mass the holdup did not have.
///
/// A wet tank's allowance is zero: the rule that starves it (`q_out·dt > m`)
/// leaves it at most a rounding error to over-draw.
fn checked_holdup_mass(
    holdup: &str,
    mass_old: f64,
    raw: f64,
    gross: f64,
    over_draw_allowance: f64,
) -> Result<f64, SimError> {
    let bound = ROUNDING_MASS_FRACTION * gross + over_draw_allowance;
    if raw < -bound {
        return Err(SimError::Numerical(format!(
            "{holdup} would end the tick at {raw:.6e} kg — drawn {:.6e} kg past empty \
             from the {mass_old:.6e} kg it held, beyond the {bound:.6e} kg its own solve \
             residual and rounding allow. The hydraulic solve delivered mass this holdup \
             did not have (docs/DESIGN.md §28 fork 5)",
            -raw
        )));
    }
    Ok(raw.max(0.0))
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
