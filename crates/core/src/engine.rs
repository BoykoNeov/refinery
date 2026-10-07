//! The engine: owns the graph and solver implementations, advances time.
//!
//! Tick sequence (see docs/DESIGN.md §1):
//!   commands → hydraulic solve (quasi-steady) → transport → unit dynamics
//!   → validation → snapshot available.

use crate::components::{Composition, Slate};
use crate::energy::{self};
use crate::error::SimError;
use crate::graph::{
    Actuator, ActuatorLimit, ControlAction, ControlLoop, ControlMode, ControlledValue, LeakRole,
    LoopId, MeasuredVariable, MeasurementPoint, NodeId, NodeKind, PlantGraph, SetpointRange, Trip,
    TripAction, TripId, TripReset, TripState, TubeState,
};
use crate::snapshot::{
    CavitationSnapshot, ColumnDuty, Command, ComponentSnapshot, ControlSnapshot, EdgeSnapshot,
    NodeSnapshot, PumpSuctionSnapshot, RestartBar, Snapshot, SupplyBoiling, TripSnapshot, TripStop,
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
    /// Whether each supply's liquid is clear of boiling, from the last tick
    /// (M52, docs/DESIGN.md §57). `last_cavitation`'s placement and reason:
    /// computed with the tick's diagnostics, where a model's `Err` can fail the
    /// tick instead of being swallowed. Empty before the first tick.
    last_supply_boiling: std::collections::BTreeMap<NodeId, SupplyBoiling>,
    /// What the trips have taken from each piece of equipment they hold (M40,
    /// docs/DESIGN.md §45): written when the FIRST trip latches on it, read and
    /// dropped when the LAST lets go. Keyed per equipment, not per trip, because
    /// two trips can cut one furnace, and the second to latch would otherwise
    /// remember the safe state the first wrote and "restart" it dark.
    held_equipment: std::collections::BTreeMap<NodeId, HeldEquipment>,
    /// Equipment whose stop ended without the trips handing it back, and why
    /// (M43, docs/DESIGN.md §48): written when the last trip lets go, dropped
    /// at the next stop or at the top of the first tick that finds it restarted.
    /// Only published, never acted on — `NodeSnapshot::trip_stop`.
    not_restarted: std::collections::BTreeMap<NodeId, NotRestarted>,
}

/// One stop that ended without a restart — see `Engine::not_restarted`.
#[derive(Debug, Clone)]
struct NotRestarted {
    /// The trip action that let go of it, which names its safe state.
    action: TripAction,
    /// The first tick that ran with the trips let go.
    at_tick: u64,
    /// Why, from `restart_bars`; never empty.
    bars: Vec<RestartBar>,
}

/// One piece of equipment as it stood before the trips stopped it (M40,
/// docs/DESIGN.md §45).
#[derive(Debug, Clone)]
struct HeldEquipment {
    /// Its own state before the first trip wrote its safe state.
    before: EquipmentBefore,
    /// The loop that was in AUTO on it then, if any. One at most: two
    /// regulating writers of one actuator are refused at load (E4).
    auto_loop: Option<LoopId>,
    /// What this stop has met that keeps the equipment for a person to restart:
    /// a trip whose reset restarts nothing, a press, a burst while it was held
    /// (M42, docs/DESIGN.md §47). Empty while every trip that has held it allows
    /// a restart. Only ever added to, so one `Manual` trip, one emergency stop
    /// or one burst holds for the rest of the stop; new tubes do not take one
    /// out. Kept per cause, not as one flag, so a frontend is told which (M43,
    /// §48). The tubes IN PLACE are not here: `restart_bars` asks them fresh.
    bars: std::collections::BTreeSet<RestartBar>,
    /// The `restart_permissives` of every trip that has held it during this
    /// stop (M44, docs/DESIGN.md §49): asked fresh by `restart_bars`, as the
    /// tubes are, since a permissive clears and un-clears with its reading.
    permissives: std::collections::BTreeSet<TripId>,
}

/// A trip-stoppable piece of equipment's own state, one variant per
/// `TripAction`.
#[derive(Debug, Clone, Copy)]
enum EquipmentBefore {
    Pump { on: bool },
    Valve { opening: f64 },
    Furnace { duty: Watt },
}

/// What a MANUAL→AUTO transfer seeds a loop's memory from: the measurement
/// standing now, the loop's setpoint and action, and the actuator's position.
type AutoSeed = (ControlledValue, ControlledValue, ControlAction, f64);

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
            last_supply_boiling: std::collections::BTreeMap::new(),
            held_equipment: std::collections::BTreeMap::new(),
            not_restarted: std::collections::BTreeMap::new(),
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
        // **An id naming nothing is refused here, before any arm runs** (M27,
        // docs/DESIGN.md §8). The graph indexes directly, so an out-of-range id
        // used to panic — rule 5's failure. One check at the top rather than one
        // per arm, because several arms reach the graph before their own lookup:
        // the trip guard's message, `check_loop_owned_duty`, `PuncturePipe`'s
        // `pipe(edge)`. Loops and trips are looked up through `Option` already.
        self.check_command_ids(&cmd)?;
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
                    .find(|c| c.actuator == Actuator::Node(node) && c.mode == ControlMode::Auto)
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
                    // The relief valve's reason, with the disc's own trigger: the
                    // forward drive across it (docs/DESIGN.md §33).
                    NodeKind::CheckValve { .. } => Err(SimError::InvalidCommand(format!(
                        "{node:?} is a check valve: its disc is moved by the forward drive \
                         across it, not by command, and would be recomputed on the next solve"
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
                    // Not damage either, and not commandable for the vent's
                    // reason: its flow is whatever stood above the brim at the
                    // end of the tick, so an area stored here is a number no
                    // solver reads (M23, docs/DESIGN.md §27 fork 3).
                    LeakRole::Overflow { .. } => Err(SimError::InvalidCommand(format!(
                        "'{}' ({edge:?}) is a tank's overflow, not a pipe that can be \
                         punctured. The loader builds one for every tank, and its flow is \
                         whatever liquid stands above the tank's brim at the end of a tick, \
                         not anything an area would set (docs/DESIGN.md §27)",
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
                // Refused while a latched trip has cut this furnace's fuel, unless
                // the write IS the cut (M32, docs/DESIGN.md §35): zero on a cut
                // furnace moves nothing, as stopping a stopped pump does. Ahead of
                // the loop guard, for `SetValveOpening`'s reason — the trip forced
                // the loop to MANUAL, so the loop guard would not fire.
                if duty.value() != 0.0 {
                    if let Some(trip) = self.graph.latched_trip_on(node) {
                        return Err(SimError::InvalidCommand(format!(
                            "{node:?} ('{}') has its fuel cut by trip '{}', which is latched. \
                             Reset the trip first (`reset_trip`) once its condition has \
                             cleared; the reset relights nothing, and the furnace can then be \
                             fired by hand",
                            self.graph.node(node).name,
                            trip.name
                        )));
                    }
                }
                // The cooler's two guards, owed since M18 made a furnace a loop's
                // actuator (docs/DESIGN.md §22 fork 4) — and one function for both
                // duty commands, so neither can grow a guard the other lacks.
                self.check_loop_owned_duty(node, duty, "furnace")?;
                match &mut self.graph.node_mut(node).kind {
                    NodeKind::Furnace { duty: d, .. } => {
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
            // A reservoir's pressure (M52, docs/DESIGN.md §57). Nothing to guard
            // beyond the value: no trip acts on a reservoir and no loop actuates
            // one. The solve reads pinned pressures fresh each tick
            // (`network::classify`), so the write is the whole change.
            Command::SetReservoirPressure { node, pressure } => {
                if !pressure.value().is_finite() || pressure.value() <= 0.0 {
                    return Err(SimError::InvalidCommand(format!(
                        "a reservoir's pressure must be finite and > 0 Pa (absolute). Got {} Pa",
                        pressure.value()
                    )));
                }
                let name = self.graph.node(node).name.clone();
                match &self.graph.node(node).kind {
                    NodeKind::Source {
                        temperature,
                        composition,
                        ..
                    } => {
                        self.refuse_boiling_supply(
                            &name,
                            pressure,
                            *temperature,
                            &composition.clone(),
                        )?;
                    }
                    NodeKind::Sink { .. } => {}
                    NodeKind::Tank(_) => {
                        return Err(SimError::InvalidCommand(format!(
                            "{node:?} ('{name}') is a tank: its pressure is the weight of its \
                             own liquid, and setting it would mean setting its level, which \
                             would create or destroy liquid. Only a supply or a destination \
                             takes a pressure"
                        )))
                    }
                    NodeKind::Atmosphere => {
                        return Err(SimError::InvalidCommand(format!(
                            "{node:?} ('{name}') is the atmosphere, fixed at 1 atm. Only a \
                             supply or a destination takes a pressure"
                        )))
                    }
                    _ => {
                        return Err(SimError::InvalidCommand(format!(
                            "{node:?} ('{name}') is not a supply or a destination (a source or \
                             a sink), so it has no pinned pressure to set"
                        )))
                    }
                }
                match &mut self.graph.node_mut(node).kind {
                    NodeKind::Source { pressure: p, .. } | NodeKind::Sink { pressure: p, .. } => {
                        *p = pressure;
                        Ok(())
                    }
                    // Unreachable: the match above returned for every other kind.
                    // An `Err` rather than a panic, for rule 5.
                    _ => Err(SimError::InvalidCommand(format!(
                        "{node:?} is not a source or a sink"
                    ))),
                }
            }
            // A supply's temperature (M52, docs/DESIGN.md §57). Read by the next
            // tick's sweep off the kind, as every boundary temperature is.
            Command::SetSourceTemperature { node, temperature } => {
                if !temperature.value().is_finite() || temperature.value() <= 0.0 {
                    return Err(SimError::InvalidCommand(format!(
                        "a supply's temperature must be finite and > 0 K. Got {} K",
                        temperature.value()
                    )));
                }
                let name = self.graph.node(node).name.clone();
                match &self.graph.node(node).kind {
                    NodeKind::Source {
                        pressure,
                        composition,
                        ..
                    } => {
                        self.refuse_boiling_supply(
                            &name,
                            *pressure,
                            temperature,
                            &composition.clone(),
                        )?;
                    }
                    // Its own reason: a sink has a temperature, but it is the fluid
                    // it hands back on a reverse flow, not an operating condition.
                    NodeKind::Sink { .. } => {
                        return Err(SimError::InvalidCommand(format!(
                            "{node:?} ('{name}') is a destination (a sink). Its temperature is \
                             only the fluid it would hand back if the plant drove flow \
                             backwards into it, and is fixed by the plant file; only a supply's \
                             temperature can be set"
                        )))
                    }
                    _ => {
                        return Err(SimError::InvalidCommand(format!(
                            "{node:?} ('{name}') is not a supply (a source)"
                        )))
                    }
                }
                match &mut self.graph.node_mut(node).kind {
                    NodeKind::Source { temperature: t, .. } => {
                        *t = temperature;
                        Ok(())
                    }
                    _ => Err(SimError::InvalidCommand(format!(
                        "{node:?} is not a source"
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
                // its actuator at the top of the next tick, so AUTO on a loop whose
                // actuator a latched trip holds would move it one tick later. MANUAL
                // stays admitted: it is what the trip already put the loop in.
                //
                // A cascade PRIMARY has no node to ask about and is exempt
                // (docs/DESIGN.md §29 fork 4): it moves no equipment, and the
                // secondary's own refusal is the one that holds the valve — a
                // primary in AUTO over a tripped secondary is simply open.
                if mode == ControlMode::Auto {
                    if let Some(node) = control.actuator.node() {
                        if let Some(trip) = self.graph.latched_trip_on(node) {
                            return Err(SimError::InvalidCommand(format!(
                                "control loop '{}' writes '{}', which trip '{}' holds and is \
                                 latched. In AUTO the loop would move it at the top of the next \
                                 tick. Reset the trip (`reset_trip`), put the equipment where \
                                 the loop should take over from, and then switch to AUTO",
                                control.name,
                                self.graph.node(node).name,
                                trip.name
                            )));
                        }
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
                    let seed = self.auto_transfer_seed(loop_id)?;
                    Some(seed.ok_or_else(|| {
                        SimError::InvalidCommand(format!(
                            "control loop '{}' has no measurement to transfer against:                              '{}' has no resolved {} yet (before the first tick){}. A                              bumpless transfer back-calculates the loop's memory from the                              error standing NOW; step the plant first, or declare the loop                              `mode = \"auto\"` in the file (docs/DESIGN.md §23 fork 4, §24                              fork 2)",
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
                    })?)
                } else {
                    None
                };
                match seed {
                    Some(seed) => self.transfer_to_auto(loop_id, seed),
                    None => {
                        self.graph
                            .control_mut(loop_id)
                            .ok_or_else(|| unknown_loop(loop_id))?
                            .mode = mode;
                        Ok(())
                    }
                }
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
                // **A cascade secondary's setpoint is its primary's actuator**
                // (M25, docs/DESIGN.md §29 fork 4), so it takes a valve's two
                // guards, word for word. Refused while the primary is in AUTO: the
                // write would be overwritten at the top of the next tick, a
                // command that appears to work and does not. In MANUAL a human
                // moves it — that is what "the primary in MANUAL" means — but not
                // outside the primary's range, `check_loop_owned_duty`'s rule:
                // the primary's faceplate would track a position outside [0, 1],
                // and its MANUAL→AUTO transfer would back-calculate from an
                // output it could never have produced.
                if let Some(primary) = self.graph.primary_of(loop_id) {
                    if primary.mode == ControlMode::Auto {
                        return Err(SimError::InvalidCommand(format!(
                            "control loop '{}' has its setpoint written by cascade primary \
                             '{}', which is in AUTO: the setpoint would be overwritten at the \
                             top of the next tick. Put '{}' in MANUAL first \
                             (`set_controller_mode`), or move ITS setpoint (`set_setpoint`)",
                            control.name, primary.name, primary.name
                        )));
                    }
                    if let Some(range) = primary.setpoint_range {
                        if !range.contains(value) {
                            return Err(SimError::InvalidCommand(format!(
                                "setpoint {:?} for control loop '{}' is outside the range of \
                                 cascade primary '{}', which owns it ({:?} to {:?}). The \
                                 primary's output is a fraction of that range, so a setpoint \
                                 outside it is a position the primary could never have \
                                 produced and cannot transfer from (docs/DESIGN.md §29 fork 4)",
                                value, control.name, primary.name, range.min, range.max
                            )));
                        }
                    }
                }
                self.graph
                    .control_mut(loop_id)
                    .ok_or_else(|| unknown_loop(loop_id))?
                    .setpoint = value;
                Ok(())
            }
            // Replace burst tubes (M37, docs/DESIGN.md §42): re-arm the burn-out
            // and nothing else. The hole is patched first, by `PuncturePipe` at
            // zero, so "the leak stopped" and "the tubes were replaced" stay two
            // events a player can see — `ResetTrip`'s shape.
            Command::ReplaceTubes { node } => {
                let name = self.graph.node(node).name.clone();
                let NodeKind::Furnace { coil, tubes, .. } = &self.graph.node(node).kind else {
                    return Err(SimError::InvalidCommand(format!(
                        "{node:?} ('{name}') is not a furnace, so it has no tubes to replace"
                    )));
                };
                if !tubes.state.is_failed() {
                    return Err(SimError::InvalidCommand(format!(
                        "furnace '{name}' has intact tubes: there is nothing to replace"
                    )));
                }
                // The player's order: patch the hole, let the coil cool, replace.
                let hole = self.burnout_hole(node, tubes.hole)?;
                if let LeakRole::Orifice { area } = self.graph.pipe(hole).leak {
                    if area.value() > 0.0 {
                        let pipe = self
                            .graph
                            .edge_ids()
                            .find(|&e| {
                                self.graph.pipe(e).leak == LeakRole::Punctureable { orifice: hole }
                            })
                            .map(|e| self.graph.pipe(e).name.clone())
                            .unwrap_or_default();
                        return Err(SimError::InvalidCommand(format!(
                            "furnace '{name}' still leaks through a {:.2} cm² hole: patch it \
                             first (`puncture_pipe` at area 0 on '{pipe}'), then replace the \
                             tubes",
                            area.value() * 1e4
                        )));
                    }
                }
                // Measured FRESH: the coil is a state, and the one standing now is
                // what the next tick's burn-out pass will compare.
                if tubes.limit_reached(coil.temperature) {
                    return Err(SimError::InvalidCommand(format!(
                        "furnace '{name}' coil is at {:.1} °C, at or past its tubes' {:.1} °C \
                         limit: new tubes would burst on the next tick. Let the coil cool \
                         first (cut the fuel)",
                        coil.temperature.value() - 273.15,
                        tubes.failure_temperature.value() - 273.15
                    )));
                }
                if let NodeKind::Furnace { tubes, .. } = &mut self.graph.node_mut(node).kind {
                    tubes.state = TubeState::Intact;
                }
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
                let Some(measurement) = self.graph.measure(
                    &self.slate,
                    &self.node_states,
                    self.last_solution.as_ref(),
                    trip.measurement_point,
                    trip.limit.variable(),
                )?
                else {
                    // **Before the first solve this is a press, not a fault**
                    // (M38, docs/DESIGN.md §43 fork 4). A flow and a furnace's
                    // outlet are absent until tick 1 (§36, §39), and a trip
                    // pressed by hand before then is tripped with nothing to
                    // compare. Refused, not guessed: re-arming a safety function
                    // on a missing reading is the wrong default (§26 fork 2).
                    if self.last_solution.is_none() {
                        return Err(SimError::InvalidCommand(format!(
                            "trip '{}' cannot be reset yet: its {} is measured only after \
                             the first tick, and a reset compares a fresh reading against \
                             the limit. Run a tick, then reset",
                            trip.name,
                            trip.limit.variable().noun()
                        )));
                    }
                    return Err(SimError::Numerical(format!(
                        "internal: trip '{}' has no {} to test its reset against, but the \
                         loader admits only quantities that exist from load, a flow and a \
                         furnace's outlet, which are absent only before the first solve \
                         (docs/DESIGN.md §26 fork 2, §36, §39)",
                        trip.name,
                        trip.limit.variable().noun()
                    )));
                };
                if trip.direction.reached(measurement, trip.limit)? {
                    return Err(SimError::InvalidCommand(format!(
                        "trip '{}' cannot be reset: its condition still holds ({:?} against a \
                         {:?} limit of {:?}). A reset re-arms a trip, and one re-armed inside \
                         its own condition would fire again on the next tick",
                        trip.name, measurement, trip.direction, trip.limit
                    )));
                }
                let trip = self.graph.trip_mut(trip_id).ok_or_else(|| {
                    SimError::InvalidCommand(format!("{trip_id:?} names no trip on this plant"))
                })?;
                trip.state = TripState::Armed;
                // A `Manual` trip restarts nothing; one whose reset restarts
                // (M40, docs/DESIGN.md §45) hands back what no other latched
                // trip still holds, NOW, as the press writes at the command.
                let actions = trip.actions.clone();
                self.release_equipment(&actions)
            }
            // Fire one trip by hand (M38, docs/DESIGN.md §43): the trip's own
            // latch and safe states, written NOW rather than at the next trip
            // pass. A press that returned `Ok` and left the pump startable until
            // the next tick would be a command that appears to work and does not
            // (§10 fork 4), so the refusals hold from the moment it lands.
            Command::ManualTrip { trip_id } => {
                let at_tick = self.tick + 1;
                let trip = self.graph.trip_mut(trip_id).ok_or_else(|| {
                    SimError::InvalidCommand(format!("{trip_id:?} names no trip on this plant"))
                })?;
                // Pressing a latched trip would do nothing, and a frontend that
                // sends it has a wrong picture of the plant — `ResetTrip`'s
                // refusal of an armed trip, the other way round.
                if trip.state.is_tripped() {
                    return Err(SimError::InvalidCommand(format!(
                        "trip '{}' is already tripped: there is nothing to press",
                        trip.name
                    )));
                }
                trip.state = TripState::Tripped {
                    at_tick,
                    by_hand: true,
                };
                let writes = trip.actions.clone();
                // Pressed by hand, so never restartable (M40, §45): whatever
                // this trip's reset mode, a person restarts what an emergency
                // stop stopped.
                self.hold_equipment(trip_id, Some(RestartBar::PressedByHand))?;
                self.write_trip_actions(writes)
            }
        }
    }

    pub fn tick(&mut self) -> Result<(), SimError> {
        let dt = self.config.dt;

        // A stop that ended without a restart is forgotten once a person has
        // restarted the equipment (M43, docs/DESIGN.md §48), so cutting it
        // again by hand later does not bring the old reason back. Report-only
        // state: nothing below reads it.
        let not_restarted = std::mem::take(&mut self.not_restarted);
        self.not_restarted = not_restarted
            .into_iter()
            .filter(|(_, ended)| self.still_stopped(ended.action))
            .collect();

        // 0a. Protection (M22). Trips run FIRST, before the loops, on the same
        //     start-of-tick state (docs/DESIGN.md §26 fork 5). A trip that fires
        //     forces the loops on its valves to MANUAL, so the loop pass below
        //     already sees MANUAL and tracks the tripped position; had the loops
        //     run first, an AUTO loop would update its memory and report an
        //     output on the tripping tick that the trip then overwrote. And the
        //     solve below sees the safe state, so the flow a trip stops is zero
        //     in this tick's own snapshot.
        self.run_trips()?;

        // 0a″. Pop reliefs (M48, docs/DESIGN.md §53 fork 3). Each one's latch
        //     moves here, between ticks, from the pressure its spring sensed in
        //     the last solve — never inside a solve, which reads it as a fixed
        //     curve. Beside the trips and on the same start-of-tick state; a
        //     trip writes pumps, valves and duties and a latch reads only a
        //     pressure, so the order between them is free.
        self.run_relief_latches();

        // 0a‴. Pump suctions (M50, docs/DESIGN.md §55). Each pump with a suction
        //     limit takes the bubble pressure the last tick's cavitation
        //     criterion found at it, and the solve below reads it as a fixed
        //     number — the relief latch's arrangement. Reads only the last
        //     tick's report, so its order among these passes is free.
        self.run_pump_suctions();

        // 0a′. Damage the plant does to itself (M37, docs/DESIGN.md §42). A
        //     furnace whose coil stands at or past its tubes' limit bursts them,
        //     on the same start-of-tick state the trips read. After the trips,
        //     and it does not matter which way round: a trip writes pumps,
        //     valves and duties, a burn-out writes a hole, and neither reads what
        //     the other writes. So a trip on a coil and a burn-out reached on the
        //     same tick both happen — protection does not un-burst a tube.
        self.run_burnouts()?;

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
            dt,
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
            //
            // A tank's OVERFLOW likewise (M23, §27 fork 3): it leaves at the
            // tank's END-of-tick state, which does not exist yet at this point.
            if self.graph.pipe(eid).leak.is_engine_written() {
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
                // A tank's own overflow is not a flux into its balance: the
                // spill below debits the inventory directly, after the boil-off,
                // and the edge carries the solve's zero at this point in the
                // tick anyway. Skipped by OWNER, not by kind or by incidence, so
                // the statement does not depend on that zero (M23, §27 fork 3).
                if self.graph.pipe(eid).leak.overflow_owner() == Some(nid) {
                    continue;
                }
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
            // The overflow this tank OWNS, found by owner for the vent's reason
            // above, and what it carries this tick: a rate, and the liquid's own
            // end-of-tick composition and temperature. `None` while the tank is
            // at or below its brim (M23, docs/DESIGN.md §27).
            let overflow = self
                .graph
                .incident(nid)
                .into_iter()
                .find(|(eid, _, _)| self.graph.pipe(*eid).leak.overflow_owner() == Some(nid))
                .map(|(eid, _, _)| eid);
            let mut spilled: Option<(KgPerSec, Composition, Kelvin)> = None;
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

                // The brim (M23, docs/DESIGN.md §27). An ideal overflow: whatever
                // liquid stands above `ρ(x)·A·H` at the END of the tick leaves in
                // that same tick, so the level never reads above the shell.
                //
                // - **After the boil-off** (fork 4): the flash is a property of the
                //   whole superheated inventory, and the liquid that spills was
                //   part of it; spilling first would leave a boiling tank below
                //   its brim by the boiled mass.
                // - **On MASS, strictly** (fork 4): `capacity` is `ρ·A·H` in the
                //   loader's own association, so a tank declared exactly full ties
                //   exactly and spills nothing. A level comparison spills a
                //   rounding error forever on the compositions whose declared
                //   level reads one ULP high (M22).
                // - **At the end-of-tick composition**: the capacity is how much
                //   of THIS liquid fits.
                // - **Temperature and composition are not recomputed.** Removing
                //   part of a well-mixed liquid changes neither, and the energy
                //   that leaves is `m_spill·h(T, x)`, carried by the edge.
                let capacity = tank.capacity(&self.slate).value();
                if tank.mass.value() > capacity {
                    let excess = tank.mass.value() - capacity;
                    tank.mass = Kg(capacity);
                    spilled = Some((
                        KgPerSec(excess / dt.value()),
                        tank.composition.clone(),
                        tank.temperature,
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
            // boils. The FLOW could not go stale anyway — step 2 writes the
            // solve's zero onto every engine-written edge at the top of each
            // tick (measured by M23.1's mutation 6, docs/DESIGN.md §27) — so the
            // zero here is a second guard. The `latent` it clears is not: step 2
            // never touches it.
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

            // The overflow, written whole and written here, after the holdup
            // update it reports, for the vent's reasons. Written on EVERY tick
            // the edge exists, as `0.0` when nothing spills. A stale rate cannot
            // survive in any case — step 2 wrote the solve's zero onto this edge
            // at the top of the tick, which is why M23.1's mutation 6 (write only
            // when spilling) is inert — so this zero is a second guard, kept so
            // the edge's four fields are always written together. `latent` is
            // always `None` — a spill is liquid (docs/DESIGN.md §27 fork 4).
            if let Some(eid) = overflow {
                let (flow, composition, temperature) = spilled.unwrap_or_else(|| {
                    (
                        KgPerSec(0.0),
                        self.graph.pipe(eid).stream.composition.clone(),
                        self.graph.pipe(eid).stream.temperature,
                    )
                });
                // Signed by the edge's own direction, the vent's rule: the
                // loader stores tank → atmosphere, but a graph built by hand may
                // not, and the far end reads the flow through `if incoming`.
                let outward = self.graph.endpoints(eid).0 == nid;
                let pipe = self.graph.pipe_mut(eid);
                pipe.stream.mass_flow = if outward {
                    flow
                } else {
                    KgPerSec(-flow.value())
                };
                pipe.stream.composition = composition;
                pipe.stream.temperature = temperature;
                pipe.stream.latent = None;
                let signed = self.graph.pipe(eid).stream.mass_flow.value();
                solution.edge_mass_flow.insert(eid, signed);
            } else if let Some((flow, _, _)) = spilled {
                // A tank over its brim that owns no overflow edge. Refused, for
                // the vent's reason: dropping the excess is mass vanishing by no
                // accounted path, and keeping it is the level above the shell
                // this milestone exists to remove. The loader builds one per
                // tank; a graph built by hand must do the same.
                return Err(SimError::Numerical(format!(
                    "tank '{node_name}' filled past its brim ({:.4e} kg/s above it) but owns \
                     no overflow edge. The loader builds one per tank, `<tank>__overflow` to \
                     the plant's `Atmosphere`; a graph built by hand must do the same \
                     (docs/DESIGN.md §27 fork 2)",
                    flow.value()
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
            //
            // Nor an overflow, which step 3 wrote at the tank's end-of-tick
            // composition: the upwind rule would read the resolved START-of-tick
            // one (M23, §27 fork 4).
            if self.graph.pipe(eid).leak.is_engine_written() {
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

        // The supplies' own conditions (M52, docs/DESIGN.md §57): read off their
        // kinds, which only a command moves, so this is what the next tick feeds.
        // Before anything is committed, as the cavitation criterion is: a model's
        // `Err` here fails the tick with the engine still at the last one.
        let mut supply = std::collections::BTreeMap::new();
        for nid in self.graph.node_ids() {
            if let NodeKind::Source {
                temperature,
                composition,
                ..
            } = &self.graph.node(nid).kind
            {
                let check =
                    supply_boiling(self.thermo.as_ref(), &self.slate, *temperature, composition)?;
                supply.insert(nid, check);
            }
        }

        // 5. Furnace coils (M34, docs/DESIGN.md §37). The sweep integrated each
        //    coil across the tick, from its start-of-tick temperature, because
        //    the sweep is where its inlet is known; the result is committed here,
        //    with every other state, so nothing above read a half-written tick.
        for (&nid, &end) in &node_states.coil_temperature {
            if let NodeKind::Furnace { coil, .. } = &mut self.graph.node_mut(nid).kind {
                coil.temperature = end;
            }
        }

        self.node_states = node_states;
        self.last_solution = Some(solution);
        self.last_cavitation = cavitation;
        self.last_supply_boiling = supply;
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
    /// Move every pop relief's latch from the last solve's pressure at its own
    /// node (M48, docs/DESIGN.md §53): shut lifts above set, lifted reseats
    /// below `set − blowdown` (`Blowdown::after`). Before the first solve there is
    /// no pressure to read and every latch stays as loaded, shut.
    fn run_relief_latches(&mut self) {
        let Some(solution) = &self.last_solution else {
            return;
        };
        for nid in self.graph.node_ids().collect::<Vec<_>>() {
            let Some(&inlet_pressure) = solution.node_pressure.get(&nid) else {
                continue;
            };
            if let NodeKind::ReliefValve {
                set_pressure,
                blowdown: Some(blowdown),
                ..
            } = &mut self.graph.node_mut(nid).kind
            {
                *blowdown = blowdown.after(inlet_pressure, *set_pressure);
            }
        }
    }

    /// Hand every pump with a suction limit the bubble pressure of its own liquid
    /// from the last tick's cavitation criterion (M50, docs/DESIGN.md §55):
    /// `None` before the first tick, and wherever the last tick had no criterion
    /// at the pump — a gas, which does not cavitate. The load refuses the key on
    /// a plant whose thermo model has no bubble pressure, so that is not a third
    /// way to `None`.
    fn run_pump_suctions(&mut self) {
        for nid in self.graph.node_ids().collect::<Vec<_>>() {
            let bubble = self
                .last_cavitation
                .get(&nid)
                .map(|c| Pascal(c.bubble_pressure_pa));
            if let NodeKind::Pump {
                suction: Some(suction),
                ..
            } = &mut self.graph.node_mut(nid).kind
            {
                suction.bubble_pressure = bubble;
            }
        }
    }

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
        let mut measured: Vec<Option<ControlledValue>> =
            Vec::with_capacity(self.graph.trips().len());
        for trip in self.graph.trips() {
            let measurement = self.graph.measure(
                &self.slate,
                &self.node_states,
                self.last_solution.as_ref(),
                trip.measurement_point,
                trip.limit.variable(),
            )?;
            // **One absence is admitted, and it is exactly one trip pass long**
            // (M33, docs/DESIGN.md §36 fork 1): a pipe's flow before the first
            // solve. That is not a failed instrument but a plant that has not run
            // yet, and from tick 2 on every flow is measured — zero included. The
            // trip stays armed and compares nothing on that pass. Every OTHER
            // absence is the loader's admission check and `measure` disagreeing
            // — an engine fault, not a quiet hold: a safety function holding
            // still on a missing measurement is the wrong default (§26 fork 2),
            // so it is not allowed to happen quietly.
            //
            // **A furnace's OUTLET is the second such quantity** (M35,
            // docs/DESIGN.md §39): resolved by the sweep, so absent before the first
            // tick, and since the coil (§37) present on every tick after, flowing or
            // not. The exemption names it exactly — a furnace node, measured for
            // temperature — so a cooler's outlet, absent whenever it stagnates,
            // could not slip through it even if the loader let one in.
            let solved_only = match trip.measurement_point {
                MeasurementPoint::Pipe(_) => true,
                MeasurementPoint::Node(node) => {
                    matches!(self.graph.node(node).kind, NodeKind::Furnace { .. })
                        && trip.limit.variable() == MeasuredVariable::Temperature
                }
                MeasurementPoint::Coil(_) => false,
            };
            let before_first_solve = solved_only && self.last_solution.is_none();
            if measurement.is_none() && !before_first_solve {
                return Err(SimError::Numerical(format!(
                    "internal: trip '{}' has no {} to compare with its limit, but the \
                     loader admits only quantities that exist from load, a flow and a \
                     furnace's outlet, which are absent only before the first tick \
                     (docs/DESIGN.md §26 fork 2, §36, §39)",
                    trip.name,
                    trip.limit.variable().noun()
                )));
            }
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
        //
        // **A trip that resets itself** (M40, docs/DESIGN.md §45) re-arms here,
        // on the same reading, once it stands past its `reset_at` on the safe
        // side — through `reached`, the comparison that fires it. Only a trip
        // latched BEFORE this pass and not by hand: one latched on this pass
        // cannot clear on it, and an emergency stop waits for a person.
        let mut latched: Vec<(TripId, Option<RestartBar>)> = Vec::new();
        let mut released: Vec<Vec<TripAction>> = Vec::new();
        for (index, (trip, measurement)) in
            self.graph.trips_mut().iter_mut().zip(measured).enumerate()
        {
            // Pass 1 let through only a flow or a furnace's outlet before the
            // first solve: the trip stays as it stands — armed, or latched by a
            // press (M38) — and its snapshot shows no measurement, as before tick 1.
            let Some(measurement) = measurement else {
                continue;
            };
            trip.last_measurement = Some(measurement);
            match (trip.state, trip.reset) {
                (TripState::Armed, _) => {
                    if trip.direction.reached(measurement, trip.limit)? {
                        trip.state = TripState::Tripped {
                            at_tick: this_tick,
                            by_hand: false,
                        };
                        let bar =
                            (!trip.reset.restarts()).then_some(RestartBar::ResetRestartsNothing);
                        latched.push((TripId(index as u32), bar));
                    }
                }
                (TripState::Tripped { by_hand: false, .. }, TripReset::Auto { reset_at }) => {
                    if !trip.direction.reached(measurement, reset_at)? {
                        trip.state = TripState::Armed;
                        released.push(trip.actions.clone());
                    }
                }
                (TripState::Tripped { .. }, _) => {}
            }
        }

        // Pass 3 — record what the latching trips take (before anything is
        // written, so the record is the plant as it stood), hand back what the
        // re-armed ones release, then write the safe states. A trip latched on
        // this pass is already `Tripped` above, so `release_equipment` sees it
        // holding its equipment whichever of the first two loops runs first:
        // equipment one trip lets go of as another latches on it stays stopped.
        for &(trip, bar) in &latched {
            self.hold_equipment(trip, bar)?;
        }
        for actions in &released {
            self.release_equipment(actions)?;
        }
        let writes = latched
            .iter()
            .filter_map(|&(trip, _)| self.graph.trip(trip))
            .flat_map(|trip| trip.actions.iter().copied())
            .collect();
        self.write_trip_actions(writes)
    }

    /// Write tripped equipment's safe states, and force every loop on it to
    /// MANUAL (docs/DESIGN.md §26 fork 5). Shared by the trip pass and a press
    /// (`Command::ManualTrip`, §43), so a trip does the same thing however it
    /// fired.
    ///
    /// MANUAL tracks, so its faceplate shows the equipment's real position from
    /// this tick, and a PI loop's memory is left alone: after a reset and a human
    /// restoring the equipment, AUTO is the existing bumpless transfer, seeded
    /// from wherever it stands. The mode change is asked of EVERY action's
    /// equipment, not of a valve's alone, so a kind a loop can actuate cannot be
    /// added to the actions without its loops yielding.
    fn write_trip_actions(&mut self, writes: Vec<TripAction>) -> Result<(), SimError> {
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
                }
                TripAction::CutFurnace { furnace } => {
                    match &mut self.graph.node_mut(furnace).kind {
                        NodeKind::Furnace { duty, .. } => *duty = Watt::ZERO,
                        _ => return Err(trip_equipment_fault(&self.graph, furnace, "furnace")),
                    }
                }
            }
            let equipment = action.equipment();
            for control in self.graph.controls_mut() {
                if control.actuator == Actuator::Node(equipment) {
                    control.mode = ControlMode::Manual;
                }
            }
        }
        Ok(())
    }

    /// Remember what a latching trip takes from its equipment, BEFORE its safe
    /// states are written (M40, docs/DESIGN.md §45).
    ///
    /// The first trip to hold a piece of equipment records it as it stands; a
    /// later one only adds to `bars`, so the record is always the state before
    /// the STOP, never a safe state another trip wrote. `bar` is this trip's
    /// say: `None` if its reset mode allows a restart and it was not pressed.
    /// Its `restart_permissives` join the record's, for the whole stop (M44).
    fn hold_equipment(&mut self, trip: TripId, bar: Option<RestartBar>) -> Result<(), SimError> {
        let Some(trip) = self.graph.trip(trip) else {
            return Err(SimError::Numerical(format!(
                "internal: {trip:?} latched and names no trip on this plant"
            )));
        };
        let (actions, permissives) = (trip.actions.clone(), trip.restart_permissives.clone());
        for action in actions {
            let node = action.equipment();
            if !self.held_equipment.contains_key(&node) {
                let before = match (action, &self.graph.node(node).kind) {
                    (TripAction::StopPump { .. }, NodeKind::Pump { on, .. }) => {
                        EquipmentBefore::Pump { on: *on }
                    }
                    (TripAction::SetValve { .. }, NodeKind::Valve { opening, .. }) => {
                        EquipmentBefore::Valve { opening: *opening }
                    }
                    (TripAction::CutFurnace { .. }, NodeKind::Furnace { duty, .. }) => {
                        EquipmentBefore::Furnace { duty: *duty }
                    }
                    (TripAction::StopPump { .. }, _) => {
                        return Err(trip_equipment_fault(&self.graph, node, "pump"))
                    }
                    (TripAction::SetValve { .. }, _) => {
                        return Err(trip_equipment_fault(&self.graph, node, "valve"))
                    }
                    (TripAction::CutFurnace { .. }, _) => {
                        return Err(trip_equipment_fault(&self.graph, node, "furnace"))
                    }
                };
                // A new stop: whatever the last one ended in is history now.
                self.not_restarted.remove(&node);
                let auto_loop = self
                    .graph
                    .controls()
                    .iter()
                    .position(|c| c.actuator == Actuator::Node(node) && c.mode == ControlMode::Auto)
                    .map(|i| LoopId(i as u32));
                self.held_equipment.insert(
                    node,
                    HeldEquipment {
                        before,
                        auto_loop,
                        bars: std::collections::BTreeSet::new(),
                        permissives: std::collections::BTreeSet::new(),
                    },
                );
            }
            if let Some(held) = self.held_equipment.get_mut(&node) {
                held.bars.extend(bar);
                held.permissives.extend(permissives.iter().copied());
            }
        }
        Ok(())
    }

    /// A trip has been reset — by a person or by itself — and these were its
    /// actions: hand back every piece of equipment no other latched trip still
    /// holds, if its record allows (M40, docs/DESIGN.md §45).
    ///
    /// Called AFTER the trip is `Armed` and after any trip latching in the same
    /// pass is `Tripped`, so "still held" counts both, and a trip that latches on
    /// the equipment as another lets go keeps it stopped.
    fn release_equipment(&mut self, actions: &[TripAction]) -> Result<(), SimError> {
        for &action in actions {
            let node = action.equipment();
            if self.graph.latched_trip_on(node).is_some() {
                continue;
            }
            // Absent only if this trip names the equipment twice and the first
            // mention already released it.
            let Some(held) = self.held_equipment.remove(&node) else {
                continue;
            };
            // No restart onto burst tubes (M41, docs/DESIGN.md §46): the trip
            // re-arms, the record is dropped, and the furnace stays dark for a
            // person — the refusal is of the restart, not of the reset. Here,
            // not in `restart_equipment`'s furnace arm, because a furnace under
            // an AUTO loop is relit through the transfer and never reaches it.
            // This asks the tubes IN PLACE; a burst during the stop, on tubes
            // since replaced, is in the record's `bars` (M42, §47).
            let bars = self.restart_bars(node, &held);
            if bars.is_empty() {
                self.restart_equipment(node, held)?;
            } else {
                // Kept for the snapshot (M43, §48): the record that held the
                // reason is dropped here, and this is the moment a frontend
                // needs it. The next tick is the first to run let go, in the
                // trip pass (still inside it) and after a command alike.
                self.not_restarted.insert(
                    node,
                    NotRestarted {
                        action,
                        at_tick: self.tick + 1,
                        bars,
                    },
                );
            }
        }
        Ok(())
    }

    /// The equipment still stands where `action` left it — at its safe state,
    /// with no loop on it in AUTO — so a stop that ended without a restart is
    /// still waiting for a person (M43, docs/DESIGN.md §48). Exact comparisons:
    /// the trip wrote these values, and any other value is someone else's.
    fn still_stopped(&self, action: TripAction) -> bool {
        let node = action.equipment();
        let at_safe_state = match (action, &self.graph.node(node).kind) {
            (TripAction::StopPump { .. }, NodeKind::Pump { on, .. }) => !*on,
            (TripAction::SetValve { position, .. }, NodeKind::Valve { opening, .. }) => {
                *opening == position
            }
            (TripAction::CutFurnace { .. }, NodeKind::Furnace { duty, .. }) => duty.value() == 0.0,
            // The loader checked each action's kind; a mismatch has no safe
            // state to stand at, and this is only a report.
            _ => false,
        };
        at_safe_state
            && !self
                .graph
                .controls()
                .iter()
                .any(|c| c.actuator == Actuator::Node(node) && c.mode == ControlMode::Auto)
    }

    /// What the snapshot says of `node`'s stop — see `NodeSnapshot::trip_stop`.
    fn trip_stop(&self, node: NodeId) -> Option<TripStop> {
        if let Some(held) = self.held_equipment.get(&node) {
            return Some(TripStop::Held {
                barred_by: self.restart_bars(node, held),
            });
        }
        let ended = self.not_restarted.get(&node)?;
        // Checked here too, not only at the top of a tick, so a person's
        // restart between ticks shows at once.
        self.still_stopped(ended.action)
            .then(|| TripStop::NotRestarted {
                at_tick: ended.at_tick,
                barred_by: ended.bars.clone(),
            })
    }

    /// Why `node`'s stop would not end in a restart if the last trip holding
    /// it let go now (M43, docs/DESIGN.md §48): the record's own `bars`, then
    /// M41's question of the tubes in place. Empty: the trips hand it back.
    ///
    /// **The one place the verdict is made.** `release_equipment` acts on it
    /// and the snapshot publishes it, so what a frontend is told and what the
    /// engine does cannot come apart. In the `Ord` order of `RestartBar`.
    fn restart_bars(&self, node: NodeId, held: &HeldEquipment) -> Vec<RestartBar> {
        let mut bars: Vec<RestartBar> = held.bars.iter().copied().collect();
        if self.tubes_forbid_restart(node) {
            bars.push(RestartBar::TubesBurst);
        }
        // No restart that another trip on the equipment would undo (M44,
        // docs/DESIGN.md §49). A latched one still holds it and is not asked
        // here: `release_equipment` never reaches a held node.
        if self
            .graph
            .trips()
            .iter()
            .any(|t| !t.state.is_tripped() && t.acts_on(node) && self.trip_condition_stands(t))
        {
            bars.push(RestartBar::TripAboutToFire);
        }
        // The start permissives the file named on the trips that held it (M44,
        // §49). Ids the loader resolved, so a missing one cannot happen; it
        // would read as not clear rather than be skipped.
        if held.permissives.iter().any(|&id| {
            self.graph
                .trip(id)
                .is_none_or(|t| t.state.is_tripped() || self.trip_condition_stands(t))
        }) {
            bars.push(RestartBar::PermissiveNotClear);
        }
        bars
    }

    /// Whether `trip`'s condition stands on a FRESH reading — what its next
    /// pass would compare (M44, docs/DESIGN.md §49).
    ///
    /// **A reading that cannot be had counts as standing.** A restart is an
    /// action, and a safety function does not act on a missing measurement
    /// (§26 fork 2). The absence is a flow or a furnace outlet before the first
    /// solve; the error is an engine fault the next trip pass returns loudly on
    /// the same reading. Either way this only keeps equipment dark, so the
    /// verdict stays the one infallible function both callers read.
    fn trip_condition_stands(&self, trip: &Trip) -> bool {
        match self.graph.measure(
            &self.slate,
            &self.node_states,
            self.last_solution.as_ref(),
            trip.measurement_point,
            trip.limit.variable(),
        ) {
            Ok(Some(reading)) => trip.direction.reached(reading, trip.limit).unwrap_or(true),
            Ok(None) | Err(_) => true,
        }
    }

    /// The equipment is a furnace whose tubes have burst, or will burst on the
    /// next burn-out pass (`FurnaceTubes::burst_or_bursting`, M41). Read FRESH:
    /// in the trip pass the coil standing is the one this tick's burn-out pass
    /// compares, and between ticks the one the next tick's will.
    fn tubes_forbid_restart(&self, node: NodeId) -> bool {
        match &self.graph.node(node).kind {
            NodeKind::Furnace { coil, tubes, .. } => tubes.burst_or_bursting(coil.temperature),
            _ => false,
        }
    }

    /// Put one piece of equipment back as it stood before the stop (M40).
    ///
    /// **A loop that was in AUTO takes it back through the bumpless transfer**,
    /// seeded from where the equipment stands now — its safe state — so a relit
    /// furnace's loop ramps from zero firing rather than jumping to the firing
    /// that tripped it, and the equipment itself is not written. **With no such
    /// loop, or one with nothing to measure now** (a cooler outlet a shut valve
    /// starved), the equipment is written back to its pre-trip state and that
    /// loop stays in MANUAL for a person, as `SetControllerMode` would refuse it.
    fn restart_equipment(&mut self, node: NodeId, held: HeldEquipment) -> Result<(), SimError> {
        if let Some(loop_id) = held.auto_loop {
            if let Some(seed) = self.auto_transfer_seed(loop_id)? {
                return self.transfer_to_auto(loop_id, seed);
            }
        }
        match (held.before, &mut self.graph.node_mut(node).kind) {
            (EquipmentBefore::Pump { on }, NodeKind::Pump { on: now, .. }) => *now = on,
            (EquipmentBefore::Valve { opening }, NodeKind::Valve { opening: now, .. }) => {
                *now = opening
            }
            (EquipmentBefore::Furnace { duty }, NodeKind::Furnace { duty: now, .. }) => *now = duty,
            (EquipmentBefore::Pump { .. }, _) => {
                return Err(trip_equipment_fault(&self.graph, node, "pump"))
            }
            (EquipmentBefore::Valve { .. }, _) => {
                return Err(trip_equipment_fault(&self.graph, node, "valve"))
            }
            (EquipmentBefore::Furnace { .. }, _) => {
                return Err(trip_equipment_fault(&self.graph, node, "furnace"))
            }
        }
        Ok(())
    }

    /// What a MANUAL→AUTO transfer of `loop_id` would seed from, read FRESH
    /// (docs/DESIGN.md §10 fork 4); `None` when the loop has nothing to measure
    /// now. Shared by `Command::SetControllerMode` and a trip's restart (M40).
    fn auto_transfer_seed(&self, loop_id: LoopId) -> Result<Option<AutoSeed>, SimError> {
        let control = self
            .graph
            .control(loop_id)
            .ok_or_else(|| unknown_loop(loop_id))?;
        let Some(measurement) = self.graph.measure(
            &self.slate,
            &self.node_states,
            self.last_solution.as_ref(),
            control.measurement_point,
            control.setpoint.variable(),
        )?
        else {
            return Ok(None);
        };
        // The SAME reader pass 1 uses (docs/DESIGN.md §21, sites 3 and 6): a
        // transfer that seeded from one notion of position while the tick ran on
        // another would step the actuator. On a cascade primary this is its
        // secondary's setpoint, as a fraction of the primary's range (§29 fork 2).
        let position = self
            .graph
            .actuator_position(control.actuator, control.max_duty, control.setpoint_range)
            .map_err(|e| {
                SimError::InvalidCommand(format!(
                    "control loop '{}' has no position to transfer from: {e}",
                    control.name
                ))
            })?;
        Ok(Some((
            measurement,
            control.setpoint,
            control.action,
            position,
        )))
    }

    /// Seed `loop_id`'s memory from `seed` and put it in AUTO: the bumpless
    /// MANUAL→AUTO transfer (docs/DESIGN.md §10 fork 4).
    fn transfer_to_auto(&mut self, loop_id: LoopId, seed: AutoSeed) -> Result<(), SimError> {
        let (measurement, setpoint, action, position) = seed;
        let control = self
            .graph
            .control_mut(loop_id)
            .ok_or_else(|| unknown_loop(loop_id))?;
        // The loop's own action, the one pass 2 will run with: a seed taken
        // against the other sign steps the first output by `2·K·e`
        // (docs/DESIGN.md §22 fork 1).
        control
            .algorithm
            .seed_from_output(position, measurement, setpoint, action)?;
        // The faceplate reports what the loop will hold, not what it held while
        // it was sitting out: a transfer that reported the old output would show
        // a jump the plant never made.
        control.last_output = position;
        control.mode = ControlMode::Auto;
        Ok(())
    }

    /// Burst the tubes of every furnace whose coil stands at or past their
    /// limit, on the state standing at the top of this tick (M37,
    /// docs/DESIGN.md §42).
    ///
    /// **Write once, latch, hold nothing.** An intact furnace at its limit opens
    /// its burn-out hole to `rupture_area` — or leaves it wider, if a hand
    /// puncture already opened it more — and latches `Failed` with this tick.
    /// Unlike a trip, nothing is then held: a burst tube is damage, and patching
    /// the hole (`PuncturePipe` at zero) is the player's to do. Failed tubes
    /// stay failed until `Command::ReplaceTubes`, so a hole opened again by
    /// hand on them burns again. The one thing a burst does to a trip's hold
    /// is narrow it: a furnace a trip holds dark is no longer restartable
    /// (M42, §47).
    ///
    /// Measure then write, like the trips: every furnace is checked before any
    /// hole is opened, though no burn-out reads what another writes.
    fn run_burnouts(&mut self) -> Result<(), SimError> {
        let this_tick = self.tick + 1;
        let mut bursts: Vec<(NodeId, Option<crate::graph::EdgeId>, SquareMeter)> = Vec::new();
        for nid in self.graph.node_ids() {
            if let NodeKind::Furnace { coil, tubes, .. } = &self.graph.node(nid).kind {
                if !tubes.state.is_failed() && tubes.limit_reached(coil.temperature) {
                    bursts.push((nid, tubes.hole, tubes.rupture_area));
                }
            }
        }
        for (nid, hole, rupture_area) in bursts {
            let hole = self.burnout_hole(nid, hole)?;
            let open = match self.graph.pipe(hole).leak {
                LeakRole::Orifice { area } => area,
                _ => unreachable!("`burnout_hole` returns only a leak orifice"),
            };
            self.graph.pipe_mut(hole).leak = LeakRole::Orifice {
                area: SquareMeter(open.value().max(rupture_area.value())),
            };
            if let NodeKind::Furnace { tubes, .. } = &mut self.graph.node_mut(nid).kind {
                tubes.state = TubeState::Failed { at_tick: this_tick };
            }
            // A burst during a stop makes its restart a person's (M42,
            // docs/DESIGN.md §47). On the stop's record, so new tubes fitted
            // before the trip lets go do not undo it, and the next stop starts
            // clean. Only a furnace a trip holds: a lit one has no stop to mark.
            if let Some(held) = self.held_equipment.get_mut(&nid) {
                held.bars.insert(RestartBar::TubesBurstDuringStop);
            }
        }
        Ok(())
    }

    /// A furnace's burn-out hole, checked to be a leak orifice in this graph.
    /// The loader builds one on every furnace; a graph built by hand without one
    /// is refused when its tubes fail rather than skipped (rule 5).
    fn burnout_hole(
        &self,
        furnace: NodeId,
        hole: Option<crate::graph::EdgeId>,
    ) -> Result<crate::graph::EdgeId, SimError> {
        let name = &self.graph.node(furnace).name;
        let hole = hole.ok_or_else(|| {
            SimError::Scenario(format!(
                "furnace '{name}' has burst its tubes but owns no burn-out hole. The loader \
                 builds one on every furnace's outlet pipe; a graph built by hand must do the \
                 same (docs/DESIGN.md §42)"
            ))
        })?;
        if !self.graph.has_edge(hole)
            || !matches!(self.graph.pipe(hole).leak, LeakRole::Orifice { .. })
        {
            return Err(SimError::Numerical(format!(
                "internal: furnace '{name}' names {hole:?} as its burn-out hole, which is not \
                 a leak orifice in this plant (docs/DESIGN.md §42)"
            )));
        }
        Ok(hole)
    }

    /// The hold check for one action of one latched trip: its equipment must
    /// still be in the safe state the trip wrote (docs/DESIGN.md §26 fork 3).
    fn check_trip_holds(&self, trip: &str, action: TripAction) -> Result<(), SimError> {
        let moved = match (action, &self.graph.node(action.equipment()).kind) {
            (TripAction::StopPump { .. }, NodeKind::Pump { on, .. }) => *on,
            (TripAction::SetValve { position, .. }, NodeKind::Valve { opening, .. }) => {
                *opening != position
            }
            (TripAction::CutFurnace { .. }, NodeKind::Furnace { duty, .. }) => *duty != Watt::ZERO,
            (TripAction::StopPump { pump }, _) => {
                return Err(trip_equipment_fault(&self.graph, pump, "pump"))
            }
            (TripAction::SetValve { valve, .. }, _) => {
                return Err(trip_equipment_fault(&self.graph, valve, "valve"))
            }
            (TripAction::CutFurnace { furnace }, _) => {
                return Err(trip_equipment_fault(&self.graph, furnace, "furnace"))
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

    /// Run every control loop on the state standing at the top of this tick:
    /// cascade primaries first, then every other loop, each half in declaration
    /// order.
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
    /// see the same start-of-tick state regardless of declaration order.
    ///
    /// **Pass 2 runs in two halves since M25** (docs/DESIGN.md §29 fork 3). A
    /// cascade primary's actuator is its secondary's SETPOINT, so every primary
    /// runs and writes that setpoint before any other loop updates, and the
    /// secondary acts on this tick's target with no added lag. That is not the
    /// algebraic loop above: nothing in pass 2 reads the plant, both measurements
    /// are from pass 1, and the solve comes after. Declaration order decides
    /// nothing a reader could see: the loader admits two levels only and one
    /// writer per actuator, so no write can be contested and no primary reads
    /// another primary's output. A plant with no cascade runs its loops in
    /// declaration order through the same arithmetic it always did.
    ///
    /// **A primary whose secondary will not act this tick is OPEN** (§29 fork 4):
    /// the secondary is not in AUTO — a human or a trip put it in MANUAL — or it
    /// has no measurement, an outlet before the first tick or while stagnant, a
    /// pipe's flow before the first tick. Pass 1 already knows both. An open
    /// primary writes nothing, its faceplate tracks the secondary's setpoint, and
    /// its memory is RE-SEEDED against that position every open tick, so on
    /// closing it resumes from exactly where the secondary stands.
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
        // on the first tick. A cascade primary's position is its secondary's
        // setpoint as it stood at the top of the tick (§29 fork 3).
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
            // backstop: the loader builds only the pairings it accepts, so it is
            // reachable only from a hand-built graph.
            let position = self.graph.actuator_position(
                control.actuator,
                control.max_duty,
                control.setpoint_range,
            )?;
            sampled.push((measurement, position));
        }
        // The opening each loop holds for a stopped pump (M45.1, docs/DESIGN.md
        // §50): in AUTO, with the pump it names off. Read here with everything
        // else this tick acts on — after the trips, so a pump a trip stopped
        // this tick is held for from this tick. A held loop is never a cascade
        // primary or secondary (refused at load), so nothing below asks it.
        let mut pump_stopped: Vec<Option<f64>> = Vec::with_capacity(sampled.len());
        for control in self.graph.controls() {
            let held = match control.on_pump_stop {
                Some(hold) if control.mode == ControlMode::Auto => {
                    match self.graph.node(hold.pump).kind {
                        NodeKind::Pump { on, .. } => (!on).then_some(hold.output),
                        _ => {
                            return Err(SimError::Numerical(format!(
                                "internal: control loop '{}' holds for node '{}', which is not \
                                 a pump; the loader refuses that (docs/DESIGN.md §50)",
                                control.name,
                                self.graph.node(hold.pump).name
                            )))
                        }
                    }
                }
                _ => None,
            };
            pump_stopped.push(held);
        }
        // Whether each loop will ACT this tick — AUTO, with something to measure.
        // Taken from pass 1 alone, after the trips have run, so a secondary a trip
        // forced to MANUAL this tick already reads as not acting.
        let will_act: Vec<bool> = self
            .graph
            .controls()
            .iter()
            .zip(&sampled)
            .map(|(c, (measurement, _))| c.mode == ControlMode::Auto && measurement.is_some())
            .collect();

        // Each loop's saturation latch (E19, docs/DESIGN.md §38), updated from
        // this pass's sample and the setpoint standing at the top of the tick —
        // before any primary writes a new one — and then read, as a whole, by
        // pass 2a. Every loop keeps one; only a cascade secondary's is read.
        let mut saturation: Vec<Option<ActuatorLimit>> = Vec::with_capacity(sampled.len());
        for (i, &(measurement, position)) in sampled.iter().enumerate() {
            let control = &mut self.graph.controls_mut()[i];
            control.saturated = saturation_latch(
                control.saturated,
                will_act[i],
                measurement,
                control.setpoint,
                control.action,
                position,
            );
            saturation.push(control.saturated);
        }

        // Each primary's secondary's ACTION, read before pass 2a borrows the
        // loops mutably: it decides which way the secondary's limit blocks.
        let inner_action: Vec<Option<ControlAction>> = self
            .graph
            .controls()
            .iter()
            .map(|c| {
                c.actuator
                    .driven_loop()
                    .and_then(|l| self.graph.controls().get(l.0 as usize))
                    .map(|secondary| secondary.action)
            })
            .collect();

        // Pass 2a — the cascade primaries, each writing its secondary's setpoint
        // before any secondary updates (§29 fork 3).
        let mut setpoint_writes: Vec<(Actuator, Option<SetpointRange>, f64)> = Vec::new();
        for (i, &(measurement, position)) in sampled.iter().enumerate() {
            let control = &mut self.graph.controls_mut()[i];
            let Actuator::Loop(secondary) = control.actuator else {
                continue;
            };
            let open = !*will_act.get(secondary.0 as usize).ok_or_else(|| {
                SimError::Numerical(format!(
                    "internal: cascade primary '{}' drives {secondary:?}, which names no loop",
                    control.name
                ))
            })?;
            // The inner loop's limit (M28, docs/DESIGN.md §31): its saturation
            // latch from pass 1 (E19, §38), beside `open`, and for the same reason
            // — it is a fact about the secondary that the primary must know before
            // it acts.
            let hold = inner_limit(saturation[secondary.0 as usize], inner_action[i]);
            if let Some(output) = step_loop(control, measurement, position, open, hold, dt)? {
                setpoint_writes.push((control.actuator, control.setpoint_range, output));
            }
        }
        for (actuator, range, output) in setpoint_writes {
            self.graph
                .set_actuator_position(actuator, None, range, output)?;
        }

        // Pass 2b — every other loop, a cascade secondary on this tick's target.
        let mut writes: Vec<(Actuator, Option<Watt>, f64)> = Vec::with_capacity(sampled.len());
        for (i, &(measurement, position)) in sampled.iter().enumerate() {
            let control = &mut self.graph.controls_mut()[i];
            if control.actuator.driven_loop().is_some() {
                continue;
            }
            if let Some(output) = pump_stopped[i] {
                track_stopped_pump(control, measurement, output)?;
                writes.push((control.actuator, control.max_duty, output));
                continue;
            }
            if let Some(output) = step_loop(control, measurement, position, false, None, dt)? {
                writes.push((control.actuator, control.max_duty, output));
            }
        }

        // Pass 3 — write the equipment, through the inverse of pass 1's reader:
        // a valve's opening is the output itself, a cooler's duty is
        // `output · max_duty`. Its error arm is unreachable — pass 1 already read
        // this same pairing, and nothing between the two passes can change a
        // node's kind — and is an `Err` rather than an `unwrap` because rule 5 is
        // about what the engine may do, not about what it can prove. A cascade
        // primary's write happened in pass 2a, and only equipment is written here.
        for (actuator, max_duty, output) in writes {
            self.graph
                .set_actuator_position(actuator, max_duty, None, output)?;
        }
        Ok(())
    }

    /// Refuse a supply command that would leave the supply's liquid boiling at
    /// its own pressure (M52, docs/DESIGN.md §57, ledger row B46): its bubble
    /// pressure at `temperature` above `pressure`. The loader refuses the same
    /// file through the same [`supply_boiling`]. Where the thermo model cannot
    /// tell, nothing is refused; `NodeSnapshot::supply_boiling` says so.
    fn refuse_boiling_supply(
        &self,
        name: &str,
        pressure: Pascal,
        temperature: Kelvin,
        composition: &Composition,
    ) -> Result<(), SimError> {
        if let SupplyBoiling::Measured { bubble_pressure_pa } =
            supply_boiling(self.thermo.as_ref(), &self.slate, temperature, composition)?
        {
            if bubble_pressure_pa > pressure.value() {
                return Err(SimError::InvalidCommand(format!(
                    "supply '{name}' would be boiling: at {:.2} °C its liquid boils at any pressure below \
                     {:.4} bar, and the supply would stand at {:.4} bar. The engine carries a \
                     supply as liquid only, so it cannot feed one that is partly vapour \
                     (docs/DEFERRED.md B46). Raise the pressure or lower the temperature",
                    temperature.value() - 273.15,
                    bubble_pressure_pa / 1e5,
                    pressure.value() / 1e5
                )));
            }
        }
        Ok(())
    }

    /// Refuse a command whose node or edge id this plant does not hold.
    ///
    /// The match has no `_` arm on purpose — the bridge's `referent` does the
    /// same — so a new `Command` variant does not build until it says what it
    /// addresses, and cannot slip past this check by default. Loop and trip ids
    /// are not checked here: their arms look them up through `Option` and refuse
    /// a miss with their own messages.
    fn check_command_ids(&self, cmd: &Command) -> Result<(), SimError> {
        let missing = match *cmd {
            Command::SetValveOpening { node, .. }
            | Command::SetPumpOn { node, .. }
            | Command::SetHeatInput { node, .. }
            | Command::SetFurnaceDuty { node, .. }
            | Command::SetCoolerDuty { node, .. }
            | Command::SetReservoirPressure { node, .. }
            | Command::SetSourceTemperature { node, .. } => {
                (!self.graph.has_node(node)).then(|| format!("{node:?} names no node"))
            }
            Command::PuncturePipe { edge, .. } => {
                (!self.graph.has_edge(edge)).then(|| format!("{edge:?} names no pipe"))
            }
            Command::ReplaceTubes { node } => {
                (!self.graph.has_node(node)).then(|| format!("{node:?} names no node"))
            }
            Command::SetControllerMode { .. }
            | Command::SetSetpoint { .. }
            | Command::ResetTrip { .. }
            | Command::ManualTrip { .. } => None,
        };
        match missing {
            Some(what) => Err(SimError::InvalidCommand(format!(
                "{what} on this plant. Ids come from the plant's own snapshot; an id \
                 from another plant, or from before a reload, addresses nothing"
            ))),
            None => Ok(()),
        }
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
        if let Some(owner) = self
            .graph
            .controls()
            .iter()
            .find(|c| c.actuator == Actuator::Node(node))
        {
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
                    // The solve's own report — see `NodeSnapshot::pump_suction`.
                    pump_suction: sol.and_then(|s| s.pump_suction.get(&id)).map(|s| {
                        PumpSuctionSnapshot {
                            npsh_available_m: s.npsh_available.value(),
                            head_fraction: s.head_fraction,
                        }
                    }),
                    // The solve's own verdict, never the tank's mass — see
                    // `NodeSnapshot::running_dry`.
                    running_dry: sol.is_some_and(|s| s.starved.contains_key(&id)),
                    // The last sweep's, absent before the first tick and on
                    // every node that is not a furnace — see
                    // `NodeSnapshot::flue_loss_w`.
                    flue_loss_w: self.node_states.flue_loss.get(&id).map(|w| w.value()),
                    // Likewise — see `NodeSnapshot::tube_fire_w`.
                    tube_fire_w: self.node_states.tube_fire.get(&id).map(|w| w.value()),
                    trip_stop: self.trip_stop(id),
                    // Every supply, from the last tick — see
                    // `NodeSnapshot::supply_boiling`.
                    supply_boiling: self.last_supply_boiling.get(&id).copied(),
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
                        | LeakRole::BoilOffVent { .. }
                        | LeakRole::Overflow { .. } => 0.0,
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
                watches: c.measurement_point,
                algorithm: c.algorithm.name().to_string(),
                mode: c.mode,
                action: c.action,
                setpoint: c.setpoint,
                measurement: c.last_measurement,
                output: c.last_output,
                drives: c.actuator.driven_loop(),
                on_pump_stop: c.on_pump_stop,
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
                watches: t.measurement_point,
                direction: t.direction,
                limit: t.limit,
                reset: t.reset,
                restart_permissives: t.restart_permissives.clone(),
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
        // A check valve's node is its INLET, upstream of the disc, so what
        // flashes there is what the line delivers to it — a throttle's reason.
        | NodeKind::CheckValve { .. }
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

/// What can be said about a supply's liquid boiling at `temperature` [K] (M52,
/// docs/DESIGN.md §57, ledger row B46): its bubble pressure, or that the supply
/// is a gas, or that `thermo` has no vapour–liquid equilibrium to ask.
///
/// The one rule behind three callers — the loader's refusal of a supply that
/// boils at its declared pressure, the two supply commands' refusals, and
/// `NodeSnapshot::supply_boiling` — so the file, the command and the report
/// cannot come to disagree about what boiling is. A caller compares the
/// `Measured` pressure with the supply's own.
///
/// Phase is `Composition::phase`, the engine's one notion of it, as the
/// cavitation criterion asks it; a mixed-phase composition is its `Err`.
///
/// # Errors
/// A mixed-phase composition, or a thermo fault other than "no equilibrium"
/// (`SimError::Scenario`, which is `CannotTell`).
pub fn supply_boiling(
    thermo: &dyn ThermoModel,
    slate: &Slate,
    temperature: Kelvin,
    composition: &Composition,
) -> Result<SupplyBoiling, SimError> {
    if composition.phase(slate)? != crate::components::Phase::Liquid {
        return Ok(SupplyBoiling::Gas);
    }
    match thermo.bubble_pressure(slate, composition, temperature) {
        Ok(bubble) => Ok(SupplyBoiling::Measured {
            bubble_pressure_pa: bubble.value(),
        }),
        Err(SimError::Scenario(_)) => Ok(SupplyBoiling::CannotTell),
        Err(other) => Err(other),
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

/// One loop's pass-2 step: what it records on its faceplate, and the position it
/// wants written, if any.
///
/// `open` is true only for a cascade primary whose secondary will not act this
/// tick (docs/DESIGN.md §29 fork 4). Every other loop passes `false` and runs the
/// arithmetic every loop ran before M25.
/// Which way a cascade primary may not move its secondary's setpoint this tick,
/// because the secondary's own actuator is already at a limit (M28,
/// docs/DESIGN.md §31). `None` everywhere else.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InnerLimit {
    /// The primary may not RAISE the secondary's setpoint.
    NoRaise,
    /// The primary may not LOWER the secondary's setpoint.
    NoLower,
}

impl InnerLimit {
    /// Does moving the primary from `position` to `output` push the secondary
    /// further into its limit? Both are fractions of the primary's range, which
    /// maps onto the secondary's setpoint increasing (`SetpointRange`).
    fn blocks(self, position: f64, output: f64) -> bool {
        match self {
            InnerLimit::NoRaise => output > position,
            InnerLimit::NoLower => output < position,
        }
    }
}

/// The limit a saturated secondary puts on its primary, given the secondary's
/// `action`; `None` for a secondary that is not saturated, and for a loop that
/// drives no other.
///
/// **What "saturated" means is the secondary's latch** (`saturation_latch`, E19,
/// docs/DESIGN.md §38), not its position on this tick. M28 read the position
/// exactly, and the exact test LEAKED (§31, row E19): near a limit a PI output
/// dips a hair off and back, and on each such tick the primary was free. M34's
/// coil made the leak decide an outcome — a furnace held at full fire was at
/// exactly 1 on 4 444 of 8 000 ticks, and its primary walked to its range top.
/// The latch closes it without the "near enough" tolerance §31 declined to
/// invent.
///
/// **The direction is the secondary's action.** A REVERSE secondary's output
/// rises with its setpoint (the error is `setpoint − measurement`), so at its
/// top the primary may not raise the setpoint; a DIRECT one's output falls as its
/// setpoint rises, so at its top the primary may not lower it. A P secondary is
/// treated the same: its output clamps the same way.
fn inner_limit(
    saturated: Option<ActuatorLimit>,
    action: Option<ControlAction>,
) -> Option<InnerLimit> {
    match (action?, saturated?) {
        (ControlAction::Reverse, ActuatorLimit::Top)
        | (ControlAction::Direct, ActuatorLimit::Bottom) => Some(InnerLimit::NoRaise),
        (ControlAction::Direct, ActuatorLimit::Top)
        | (ControlAction::Reverse, ActuatorLimit::Bottom) => Some(InnerLimit::NoLower),
    }
}

/// A loop's saturation latch for this tick (E19, docs/DESIGN.md §38), from last
/// tick's latch and this tick's start-of-tick sample.
///
/// **Set** whenever the position is exactly at a limit — M28's test, and exact
/// for M28's reason: a loop's output is clamped with `f64::clamp`, which returns
/// exactly `0.0` or `1.0`, and the position is read back bare (a valve) or as
/// `duty / max_duty` of a duty written as `output · max_duty`, which IEEE
/// arithmetic returns exactly.
///
/// **Held** off the limit while the loop's error keeps the sign that drove it
/// there: positive at the top (the loop still wants more), negative at the
/// bottom. That is the leak's whole mechanism stated as a condition. A pinned PI
/// loop back-calculates its memory against this tick's error, and on a lagging
/// plant next tick's error has shrunk a little, so its output lands a hair inside
/// the limit — for one to three ticks at a time on M34's coil — while the
/// actuator's authority is still spent and the measurement still short of its
/// target. No threshold is involved: the sign of the error is the loop's own.
///
/// **Cleared** when the error reaches zero or crosses it (the measurement has
/// caught its target, and the loop is regulating again), when the loop is not
/// acting (MANUAL, or nothing to measure — an open cascade has nothing to hold),
/// or by reaching the other limit, which sets that one instead.
fn saturation_latch(
    previous: Option<ActuatorLimit>,
    acting: bool,
    measurement: Option<ControlledValue>,
    setpoint: ControlledValue,
    action: ControlAction,
    position: f64,
) -> Option<ActuatorLimit> {
    if !acting {
        return None;
    }
    if position == 1.0 {
        return Some(ActuatorLimit::Top);
    }
    if position == 0.0 {
        return Some(ActuatorLimit::Bottom);
    }
    let error = ControlledValue::error(measurement?, setpoint, action);
    match previous {
        Some(ActuatorLimit::Top) if error > 0.0 => Some(ActuatorLimit::Top),
        Some(ActuatorLimit::Bottom) if error < 0.0 => Some(ActuatorLimit::Bottom),
        _ => None,
    }
}

/// One tick of a loop holding its valve for a stopped pump (M45.1,
/// docs/DESIGN.md §50): the caller writes `output`; here the faceplate shows it
/// and the memory is re-seeded against it — output tracking, the open
/// cascade's arm with a declared position in place of a tracked one — so the
/// tick the pump runs again, the loop resumes from `output` with no surplus
/// wound up while nothing came through. With nothing to measure the memory is
/// left alone, as the blind arm of `step_loop` leaves it.
fn track_stopped_pump(
    control: &mut ControlLoop,
    measurement: Option<ControlledValue>,
    output: f64,
) -> Result<(), SimError> {
    control.last_measurement = measurement;
    if let Some(measurement) = measurement {
        control.algorithm.seed_from_output(
            output,
            measurement,
            control.setpoint,
            control.action,
        )?;
    }
    control.last_output = output;
    Ok(())
}

fn step_loop(
    control: &mut ControlLoop,
    measurement: Option<ControlledValue>,
    position: f64,
    open: bool,
    hold: Option<InnerLimit>,
    dt: Seconds,
) -> Result<Option<f64>, SimError> {
    control.last_measurement = measurement;
    match (control.mode, measurement) {
        // **No measurement, no action** (docs/DESIGN.md §23 fork 2): a furnace or
        // cooler outlet before the first tick, or while it is stagnant. The loop
        // writes nothing, its faceplate TRACKS the actuator exactly as MANUAL's
        // does, and its memory — seeded or still pending — is not touched, so it
        // resumes from where it stood. Acting on a stand-in would be worse than
        // waiting: a PI loop seeds against whatever it first measures.
        (ControlMode::Auto, None) => {
            control.last_output = position;
            Ok(None)
        }
        // **An open cascade** (§29 fork 4): the secondary is not using its
        // setpoint, so writing it would integrate against a plant that is not
        // answering. The primary writes nothing and tracks — and, unlike the blind
        // loop above, RE-SEEDS its memory against the tracked position: here the
        // error exists and the position is real, and leaving the memory untouched
        // closes the cascade with a bump (1.37 K of outlet setpoint on the hand
        // probe, against one tick of control).
        (ControlMode::Auto, Some(measurement)) if open => {
            control.algorithm.seed_from_output(
                position,
                measurement,
                control.setpoint,
                control.action,
            )?;
            control.last_output = position;
            Ok(None)
        }
        (ControlMode::Auto, Some(measurement)) => {
            let output =
                control
                    .algorithm
                    .update(measurement, control.setpoint, control.action, dt)?;
            // **The inner loop at its limit** (M28, docs/DESIGN.md §31): the
            // primary may not push its secondary's setpoint further in the
            // direction that saturated it. Its own clamp's conditional
            // integration, extended to the inner actuator's limit, and written as
            // the open cascade's arm because that is what it is in one direction:
            // nothing is written, the faceplate tracks, and the memory is
            // back-calculated against the held position — so the primary resumes
            // from where the setpoint stands, with no surplus to spend. Moving
            // the other way, out of the limit, is untouched.
            if hold.is_some_and(|h| h.blocks(position, output)) {
                control.algorithm.seed_from_output(
                    position,
                    measurement,
                    control.setpoint,
                    control.action,
                )?;
                control.last_output = position;
                return Ok(None);
            }
            // Checked here rather than trusted from the seam: the range is the
            // actuator's, not the algorithm's, and an out-of-range position
            // reaching `Valve::opening` is a plant state `Command::SetValveOpening`
            // would have refused from a human.
            if !output.is_finite() || !(0.0..=1.0).contains(&output) {
                return Err(SimError::Numerical(format!(
                    "control loop '{}' ({}) produced actuator position {output}, which is not \
                     a finite fraction in [0, 1]",
                    control.name,
                    control.algorithm.name()
                )));
            }
            control.last_output = output;
            Ok(Some(output))
        }
        // MANUAL writes nothing and TRACKS: the faceplate reports the actuator's
        // real opening, which is what a DCS shows and what makes AUTO→MANUAL
        // transfer free (fork 4). The measurement is still taken, so the loop-off
        // counterfactual is a run of the same plant with a truthful faceplate
        // rather than of a plant with the loop deleted.
        (ControlMode::Manual, _) => {
            control.last_output = position;
            Ok(None)
        }
    }
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
