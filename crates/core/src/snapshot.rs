//! Frontend contract: `Command` in, `Snapshot` out. Both plain serde data.
//! Frontends never touch engine internals.

use crate::graph::{EdgeId, NodeId, NodeKind, TankState};
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

/// Complete observable state after a tick. Serializable (JSON for humans,
/// bincode if profiling ever demands it). Golden-snapshot tests compare
/// these byte-for-byte for determinism.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Snapshot {
    pub tick: u64,
    pub sim_time: Seconds,
    pub nodes: Vec<NodeSnapshot>,
    pub edges: Vec<EdgeSnapshot>,
    pub solver: SolveDiagnostics,
    /// Convenience view for frontends: (node name, tank state).
    pub tanks: Vec<(String, TankState)>,
}
