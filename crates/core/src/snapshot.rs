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
    /// Distinct from `SetFurnaceDuty`: this is heat the plant did not ask for,
    /// and it stacks on top of a furnace's duty rather than replacing it.
    SetHeatInput {
        node: NodeId,
        power: Watt,
    },
    /// Operating setpoint of a fired heater [W delivered to the process fluid].
    /// 0 shuts it down.
    SetFurnaceDuty {
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
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EdgeSnapshot {
    pub id: EdgeId,
    pub name: String,
    pub from: NodeId,
    pub to: NodeId,
    pub stream: Stream,
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
