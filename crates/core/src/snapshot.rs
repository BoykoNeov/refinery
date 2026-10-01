//! Frontend contract: `Command` in, `Snapshot` out. Both plain serde data.
//! Frontends never touch engine internals.

use crate::graph::{
    ControlAction, ControlMode, ControlledValue, EdgeId, LoopId, NodeId, NodeKind, TankState,
    TripDirection, TripId, TripState,
};
use crate::stream::Stream;
use crate::traits::SolveDiagnostics;
use crate::units::{Seconds, SquareMeter, Watt};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
pub enum Command {
    SetValveOpening {
        node: NodeId,
        opening: f64,
    },
    SetPumpOn {
        node: NodeId,
        on: bool,
    },
    /// Damage: puncture a pipe. Engine adds/updates a leak path to Atmosphere.
    PuncturePipe {
        edge: EdgeId,
        area: SquareMeter,
    },
    /// Damage: external heat on a node (fire). 0 to extinguish.
    ///
    /// Distinct from `SetFurnaceDuty`/`SetCoolerDuty`: this is heat the plant
    /// did not ask for, and it stacks on top of a unit's duty rather than
    /// replacing it — so a fire adds to a furnace and fights a cooler.
    SetHeatInput {
        node: NodeId,
        power: Watt,
    },
    /// Operating setpoint of a fired heater [W delivered to the process fluid].
    /// Must be >= 0; 0 shuts it down.
    SetFurnaceDuty {
        node: NodeId,
        duty: Watt,
    },
    /// Operating setpoint of a cooler [W REMOVED from the process fluid].
    /// Must be >= 0; 0 shuts it down.
    ///
    /// Separate from `SetFurnaceDuty` rather than one signed `SetDuty`, because
    /// the same positive number would mean opposite things depending on the
    /// node's kind — unreadable at the call site. See `NodeKind::Cooler`.
    SetCoolerDuty {
        node: NodeId,
        duty: Watt,
    },
    /// Put one control loop in `Auto` (it drives its actuator) or `Manual` (a
    /// human does). See [`crate::graph::ControlMode`].
    ///
    /// **MANUAL→AUTO is not just a flag flip**: the loop's algorithm is seeded
    /// from the position the actuator is actually at, so a loop taking over from
    /// a human does not step the valve (§10 fork 4). That is the same
    /// back-calculation the anti-windup clamp performs, and a controller with no
    /// memory implements it as an explicit no-op — a proportional loop therefore
    /// DOES step the valve on transfer, to whatever `K·e` says, which is the
    /// controller being what it is rather than a defect.
    SetControllerMode {
        loop_id: LoopId,
        mode: ControlMode,
    },
    /// Move one loop's target.
    ///
    /// The value carries its own unit, so a setpoint in the wrong variable is
    /// not something this command can express — the reason
    /// [`ControlledValue`] is a tagged enum rather than a bare `f64` (§10 fork
    /// 4). Range- and finiteness-checked like every other command argument: a
    /// level setpoint must be finite and within the tank's own height, since a
    /// target the plant cannot reach is a loop pinned at saturation forever.
    SetSetpoint {
        loop_id: LoopId,
        value: ControlledValue,
    },
    /// Re-arm one latched trip (M22, docs/DESIGN.md §26 fork 4).
    ///
    /// **It restarts nothing.** The pump stays stopped and the valve stays where
    /// the trip put it; the reset only lifts the refusals that held them, so a
    /// human can then restart the equipment by hand. "The trip cleared" and
    /// "the plant restarted" stay two events a player can see.
    ///
    /// Refused while the trip's condition still holds — measured FRESH at the
    /// command, since the state standing now is what the next tick will
    /// measure — on a trip that is not tripped, and on an id naming no trip.
    ResetTrip {
        trip_id: TripId,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeSnapshot {
    pub id: NodeId,
    pub name: String,
    pub kind: NodeKind,
    pub pressure_pa: f64,
    /// Resolved node temperature [K] — a tank's own state, a reservoir's fixed
    /// value, or a zero-volume node's mixed inflow temperature. NaN before the
    /// first tick, like `pressure_pa`.
    pub temperature_k: f64,
    /// External heat forced onto this node [W] — the damage model's fire, set
    /// by [`Command::SetHeatInput`] and by nothing else.
    ///
    /// **This is `node.heat_input`, deliberately NOT `energy::heat_load()`.**
    /// That function returns the fire PLUS the node's own unit term — a
    /// furnace's duty, a cooler's negative duty, a tank's ambient exchange —
    /// and reporting the sum here would show every furnace in every scenario
    /// as being on fire. A furnace doing its job and a furnace with a fire on
    /// it are different states, and this field is the one that tells them
    /// apart. The operating setpoints stay where they already are, on `kind`.
    ///
    /// Real from load, not NaN before the first tick: it is a *stored*
    /// quantity, like a tank's temperature, not a *solved* one.
    pub heat_input_w: f64,
    /// A column's condenser and reboiler heat duties [W], when its separation
    /// fidelity computes them (M7.4b).
    ///
    /// **Absent, not zero, wherever there is nothing to report** — on every node
    /// that is not a column, on a column before its first tick, and on a
    /// cut-point column, whose fidelity has no such equipment at all
    /// (`traits::Separation::condenser_duty`). Reporting `0.0` there would be a
    /// number no model produced, and `Command::SetHeatInput`'s own lesson runs
    /// the other way round: a field nothing reports is an oversight, a field
    /// reporting what nothing computed is worse.
    ///
    /// `skip_serializing_if` is what keeps the twelve existing scenarios
    /// byte-identical — the same move `ColumnDraw`'s M7.3 fields made, and the
    /// reason this is an added field rather than a widened one.
    ///
    /// An emergent DIAGNOSTIC: nothing in the forward solve is driven by it. It
    /// is here because a game reads fuel off a reboiler and cooling water off a
    /// condenser, and because a plant-level energy balance at a cascade column
    /// does not close without it — the draws leave at differing tray
    /// temperatures, so the difference of these two IS the column's net external
    /// heat.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub column_duty: Option<ColumnDuty>,
    /// Whether the liquid at this node is boiling, and the bubble pressure the
    /// verdict was made against (M11, docs/DESIGN.md §13).
    ///
    /// **`None` means "there is no criterion at this node", never "healthy".**
    /// Three different things produce it, and a frontend must render all three
    /// as *unknown*: the node is not on the hydraulic flow path (a holdup below
    /// its bubble point is a two-phase inventory, which is a different and
    /// deferred problem); its fluid is a gas, which cannot cavitate because it
    /// is already vapour; or the plant's `ThermoModel` has no vapour–liquid
    /// equilibrium, which is fourteen of the fifteen shipped plants.
    ///
    /// That is `column_duty`'s shape and `column_duty`'s argument. Reporting
    /// `cavitating: false` where nothing was computed would be
    /// [`NodeSnapshot::heat_input_w`]'s lesson pointed the other way — not a
    /// field nothing reports, but a field reporting what nothing computed, and
    /// here it would be a clean bill of health issued by a model that has no
    /// opinion.
    ///
    /// Also `None` before the first tick, where there is no solved pressure to
    /// compare and no resolved temperature to evaluate at. `pressure_pa` reports
    /// NaN there; this reports absence, because a NaN bubble pressure is a
    /// number the model never produced and rule 5 forbids handing one out.
    ///
    /// `skip_serializing_if` is what keeps every plant that cannot answer
    /// byte-identical, the move `column_duty` and `ColumnDraw`'s M7.3 fields
    /// both made.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cavitation: Option<CavitationSnapshot>,
}

/// The cavitation criterion at one node — see [`NodeSnapshot::cavitation`].
///
/// Two fields carrying one relationship, deliberately. The `bool` is the
/// deliverable: docs/DESIGN.md §3 used to tell frontends to infer cavitation
/// from a negative absolute pressure, which fires late by the fluid's whole
/// vapour pressure, so the engine says it rather than leaving it to be
/// inferred. The pressure is what makes the verdict auditable and what a margin
/// gauge draws — a frontend holding only the `bool` cannot show a plant getting
/// closer.
///
/// Two fields carrying one relationship is also how they drift, so their
/// agreement is a gate rather than an assumption (`EdgeSnapshot::leak_mass_flow`
/// has the same shape and the same defence).
///
/// **The engine reports this; it does not act on it.** A cavitating pump still
/// delivers its full head, because the vapour that would spoil it is mass in a
/// phase the state vector does not have (`docs/DEFERRED.md` B3). The
/// disagreement is known, named, and preferred to a clamp that would make a
/// converged solve stop conserving.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CavitationSnapshot {
    /// Bubble pressure of this node's liquid at its resolved temperature [Pa] —
    /// `ThermoModel::bubble_pressure`, the pressure below which it boils.
    pub bubble_pressure_pa: f64,
    /// `pressure_pa < bubble_pressure_pa`: the engine's own verdict.
    pub cavitating: bool,
}

/// A column's two emergent heat duties [W] — see `NodeSnapshot::column_duty`.
///
/// Both are non-negative MAGNITUDES with the direction in the name, the
/// `Furnace`/`Cooler` convention: a condenser removes heat, a reboiler adds it,
/// and a signed pair would make a condenser that heats representable.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ColumnDuty {
    /// Heat REMOVED at the total condenser [W], ≥ 0.
    pub condenser_w: f64,
    /// Heat ADDED at the reboiler [W], ≥ 0.
    pub reboiler_w: f64,
}

/// One control loop's faceplate — what a frontend draws and what §7's rule
/// ("every command must have a reported consequence") requires exist.
///
/// `Command::SetSetpoint` writing a field no snapshot reports would be M6.0's
/// `PuncturePipe` with the direction reversed: the defect §7 already documents.
/// So the surface is specified with the commands rather than discovered later.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlSnapshot {
    pub id: LoopId,
    /// Scenario-given loop name — what a faceplate is labelled with.
    pub name: String,
    /// The algorithm driving it ("proportional", "proportional_integral"), from
    /// `Controller::name`.
    pub algorithm: String,
    pub mode: ControlMode,
    /// Which way the output moves the measurement (M18, docs/DESIGN.md §22).
    ///
    /// **Skipped when `direct`**, so every loop that predates M18 publishes the
    /// bytes it always did, and `default` reads an absent key back as direct —
    /// admissible by M8.5's test because absence is a TRUE statement about every
    /// such loop. Published at all because the error the controller saw is
    /// `measurement − setpoint` on a direct loop and `setpoint − measurement` on a
    /// reverse one: without this field a reader rebuilding it gets the wrong sign
    /// on every furnace loop.
    #[serde(default, skip_serializing_if = "ControlAction::is_direct")]
    pub action: ControlAction,
    /// The target. Carries its unit in its own tagged form, because a bare
    /// `setpoint` would be a number whose unit depends on a sibling field — see
    /// [`ControlledValue`].
    pub setpoint: ControlledValue,
    /// **The measurement the controller ACTED ON**, not a re-read of what is true
    /// now.
    ///
    /// Those differ by one tick: the loop runs at the top of the tick on the
    /// state standing at the start of it (docs/DESIGN.md §10 fork 3). Reporting
    /// the fresh one would make a lagging loop look instantaneous — hiding the
    /// lag from precisely the person debugging it.
    ///
    /// Same type as `setpoint` by construction, so a loop cannot report a
    /// setpoint in one variable against a measurement in another, and the
    /// difference a reader takes between them — in the order `action` names — is
    /// the error the controller saw.
    ///
    /// **Absent when the loop had nothing to act on** (M19, docs/DESIGN.md §23):
    /// a furnace or cooler OUTLET before the first tick — so at load and after
    /// tick 1 — and while it is stagnant. A frontend renders the absence as
    /// *unknown*, never as zero; on those ticks the loop wrote nothing and
    /// `output` is the actuator's own position. Skipped rather than written as
    /// `null`, and `default` reads the absent key back as `None`. Every loop that
    /// measures a stored quantity has a value from load, so this is `Some` on
    /// every loop written before M19, and `Option<T>` holding a value serializes
    /// exactly as `T` — their bytes cannot have moved.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub measurement: Option<ControlledValue>,
    /// Actuator position, dimensionless in `[0, 1]`.
    ///
    /// In `Auto` this is the controller's output and is what was written to the
    /// actuator. In `Manual` the loop writes nothing and this tracks the
    /// actuator's real position, which is what a DCS faceplate shows.
    ///
    /// **Still a bare fraction now that an actuator is not a valve** (M17,
    /// docs/DESIGN.md §21). This doc used to say it "gains a unit question only
    /// when an actuator that is not a valve un-defers", and a cooler is that
    /// actuator. The answer: for a valve it is the opening, for a cooler it is the
    /// duty as a fraction of the loop's declared `max_duty_mw` — and for a
    /// furnace (M18) the same, whichever way the loop acts: a reverse loop's
    /// faceplate reads the real firing fraction, which is why the inverted
    /// `(1 − u)·max` map was rejected. The watts are already published on the
    /// node's own `kind.duty`, so a second copy here would be a second owner.
    ///
    /// On a cascade primary (M25) it is the secondary's setpoint as a fraction
    /// of the primary's declared range; the setpoint itself is published on the
    /// secondary's own faceplate, so it is not copied here either.
    pub output: f64,
    /// The loop whose setpoint this one writes, when this loop is a cascade
    /// PRIMARY (M25, docs/DESIGN.md §29 fork 7).
    ///
    /// A faceplate needs the link to draw it, and a frontend may not reach into
    /// the engine for it (rule 6). **Skipped when `None`**, so every loop that
    /// writes equipment — every loop before M25 — publishes the bytes it always
    /// did, and `default` reads the absent key back as `None`, a true statement
    /// about those loops.
    ///
    /// **"Open" is not a field.** A cascade is open whenever its secondary will
    /// not act — not in AUTO, or nothing to measure — and both are already on the
    /// secondary's faceplate (`mode`, `measurement`), read through this link. A
    /// field saying it would be a second owner of one fact.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub drives: Option<LoopId>,
}

/// One trip, as a frontend draws it (M22, docs/DESIGN.md §26 fork 8).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TripSnapshot {
    pub id: TripId,
    /// Scenario-given trip name.
    pub name: String,
    pub direction: TripDirection,
    /// The limit, carrying its own unit the way a loop's setpoint does.
    pub limit: ControlledValue,
    /// The measurement the last trip pass compared against `limit`.
    ///
    /// **Absent only before the first tick**, when no pass has run. Every
    /// quantity a trip may watch exists from load (fork 2), so after tick 1 this
    /// is always present. Skipped when absent, as a loop's is.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub measurement: Option<ControlledValue>,
    /// `{"status":"armed"}`, or `{"status":"tripped","at_tick":…}` from the tick
    /// whose pass fired it until a reset.
    pub state: TripState,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EdgeSnapshot {
    pub id: EdgeId,
    pub name: String,
    pub from: NodeId,
    pub to: NodeId,
    pub stream: Stream,
    /// Power friction dissipated into this stream [W], from the last solve;
    /// NaN before the first tick, like `NodeSnapshot::pressure_pa`.
    ///
    /// `Φ = α·Q|Q|·Q` — pipe wall, valve trim and pump curve droop, but never
    /// elevation head or the pump's own jump, which are reversible (DESIGN §3a).
    /// A device folds into its outlet edge, so a valve's throttling heat appears
    /// on the edge LEAVING it, and `stream.temperature` (the edge's outlet)
    /// already carries the rise it causes.
    pub dissipation_w: f64,
    /// Mass escaping through this pipe's leak path [kg/s], ≥ 0 outward.
    ///
    /// Nonzero only on a pipe the scenario declared punctureable — specifically
    /// the UPSTREAM half of it, which keeps the declared pipe's name and is the
    /// end a frontend draws a spray from. Every other edge reports 0.0,
    /// including the orifice edge itself: it is in `edges` as an ordinary edge
    /// carrying this same mass as its own `stream.mass_flow`, and reporting the
    /// number twice on the same edge would say the plant lost it twice.
    ///
    /// **This is a convenience VIEW of the orifice edge's flow, and two fields
    /// carrying one quantity is how they drift** — so their agreement is a gate
    /// (`leak_reference::snapshot_leak_flow_matches_the_orifice_edge`), not an
    /// assumption. 0.0 before the first solve, where the orifice has no flow yet;
    /// unlike `dissipation_w` this is not NaN there, because "no leak has flowed"
    /// is a true statement about a plant that has not run, not a missing one.
    pub leak_mass_flow: f64,
}

/// One pseudo-component, as a frontend needs to read it — the slate's
/// **name** and the **density** that turns a mass into a volume (M8.5).
///
/// Deliberately not the whole [`crate::components::PseudoComponent`]: `tb`,
/// `molar_mass` and `cp` are inputs to models that run *inside* the engine, and
/// a frontend that read them could only recompute what the engine already
/// reports. These two are the ones a frontend cannot obtain any other way — the
/// name to label a fraction with, the density to size a level by.
///
/// Order is the slate's own declaration order, which is what makes it usable:
/// `TankState::composition`'s `mass_fractions` index into exactly this list
/// (`components::Slate` — "order is canonical"). A frontend zips the two.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ComponentSnapshot {
    /// The scenario's own name for this cut, e.g. `"heavy_naphtha"`.
    pub name: String,
    /// Liquid density at reference conditions [kg/m³], or `null` for a
    /// gas-phase component, whose density is `P·M̄/(R·T)` and not a constant
    /// (`components::PseudoComponent::density`).
    ///
    /// **On the fill-level path this is never `null`**, and that is enforced
    /// rather than hoped: the loader refuses a tank whose composition is
    /// gas-phase (`components::Composition::mixture_density`), and the one other
    /// holdup kind — `NodeKind::Vessel` — has a *pressure* for a state, not a
    /// level. So a frontend computing `ρ = 1/Σ(fᵢ/ρᵢ)` over a tank's own
    /// nonzero fractions cannot meet one, and needs no fallback for it. It is an
    /// `Option` because the *slate* may still carry gas cuts that no tank holds.
    ///
    /// Spelled like the scenario key `density_kg_per_m3`, so the number a
    /// frontend reads back is named the same as the number an author wrote.
    pub density_kg_per_m3: Option<f64>,
}

/// Complete observable state after a tick. Serializable (JSON for humans,
/// bincode if profiling ever demands it). Golden-snapshot tests compare
/// these byte-for-byte for determinism.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Snapshot {
    pub tick: u64,
    pub sim_time: Seconds,
    /// The engine's pseudo-component slate, in declaration order (M8.5).
    ///
    /// **This is what lets a frontend compute a tank's fill level**, which is
    /// the M6.2 deferral this closes: a tank reports mass [kg], area [m²] and
    /// height [m], and `h = m/(ρ·A)` needs a density the snapshot did not carry.
    /// The fix is the slate rather than a precomputed `level_m` field, because
    /// the same deferral names two other triggers a level would not have served
    /// — a per-component readout and a component *name* — and because a level is
    /// a view of data a frontend now holds, not a measurement only the engine
    /// can make.
    ///
    /// **No `default`, and no `skip_serializing_if`** — the opposite of every
    /// other field added to this struct, and the difference is real. `controls:
    /// []` and `column_duty: None` are true statements about a plant (it has no
    /// loops; that node is not a column). An empty slate is not a statement, it
    /// is impossible: `components::Slate::new` refuses one, so every engine that
    /// exists has at least one component. A `default` would let a document
    /// written before this field deserialize into a `Snapshot` whose slate says
    /// "no components", which is `heat_input_w`'s lesson pointed the other way —
    /// not a field nothing reports, but a field reporting what nothing holds.
    ///
    /// **Constant for the life of an engine, and repeated on every snapshot
    /// anyway.** The alternative is a header emitted once, which would make line
    /// 400 of a JSON-lines run uninterpretable on its own and would give the
    /// Godot bridge — whose entire outward surface is `snapshot_json` — nowhere
    /// to put it. The cost is ~40 bytes per component per emitted snapshot.
    ///
    /// This is the field whose arrival moved every scenario's snapshot bytes;
    /// see docs/ROADMAP.md M8.5 for the measurement.
    pub slate: Vec<ComponentSnapshot>,
    pub nodes: Vec<NodeSnapshot>,
    pub edges: Vec<EdgeSnapshot>,
    pub solver: SolveDiagnostics,
    /// Convenience view for frontends: (node name, tank state).
    pub tanks: Vec<(String, TankState)>,
    /// One entry per control loop, in declaration order — which is also
    /// execution order and `LoopId` order.
    ///
    /// A list beside the nodes, and deliberately **no per-node controller
    /// field**: that would be the inverse of `NodeSnapshot::column_duty`'s
    /// argument, reporting "no loop here" on every node of every plant. Absent
    /// where there is nothing to report, and the report lives with the loop that
    /// owns it.
    ///
    /// `skip_serializing_if` is what keeps the thirteen pre-M8 scenarios
    /// byte-identical — none declares a loop, so the key is not emitted at all.
    /// The same move `ColumnDraw` and `column_duty` both made, and `default` is
    /// its other half, so a snapshot written before M8.2 still deserializes.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub controls: Vec<ControlSnapshot>,
    /// One entry per trip, in declaration order — `TripId` order and evaluation
    /// order (M22, docs/DESIGN.md §26 fork 8).
    ///
    /// `controls`' shape and argument: absent where there is nothing to report,
    /// which is every plant written before M22, and `skip_serializing_if` is what
    /// keeps those byte-identical.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub trips: Vec<TripSnapshot>,
}
