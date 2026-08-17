//! The translation layer: the half of M6 that DESIGN §8 calls **gateable**.
//!
//! Untrusted text in, plain data out. No Godot types appear here — the
//! `godot` feature's binding is a thin marshalling shell over this module, so
//! that everything with a decision in it can be tested by `cargo test`.
//! (gdext's `Dictionary`/`GString` need a live Godot runtime; a translation
//! layer written in terms of them would be untestable by this repo's means.)
//!
//! # What this module is responsible for
//!
//! 1. **The trust boundary.** `Engine::apply` indexes its graph directly
//!    (`core/src/graph.rs:552-563`), so an out-of-range `NodeId`/`EdgeId`
//!    **panics**. Every in-repo caller takes its ids from a snapshot and is
//!    in-range by construction; a frontend holding a stale id is not. Ids are
//!    therefore validated *here*, against the set this bridge read from the
//!    engine, and an id that fails is never forwarded. See
//!    `core_panics_on_an_out_of_range_id` in the tests — a characterization
//!    test that pins the behaviour this guard exists for, so the guard cannot
//!    quietly become decorative.
//!
//!    Not fixed in `core`, deliberately: CLAUDE.md rule 1 says a change to
//!    `core` needed to satisfy a frontend means the *adapter* is wrong, and
//!    the reachability above says untrusted input is the only way in.
//!    **Un-defers if** a second untrusted-input frontend appears, or any
//!    in-repo caller gains the ability to construct an out-of-range id.
//!
//! 2. **Names.** `Command` addresses nodes and edges by numeric id, and that
//!    JSON shape is a frontend contract (DESIGN §7, §3b) — so this module does
//!    not invent a second, name-addressed command format. It exposes lookup
//!    instead: a scene resolves `"discharge_valve"` to an id once at startup
//!    and sends the contract's own JSON thereafter. One wire format, plus a
//!    phone book.
//!
//! 3. **Errors.** `SimError` becomes an `ErrorReport { code, message }` — a
//!    stable string a scene can branch on, plus prose for a log.
//!
//! # Pre-tick state: floats are `null`, and that is not a bug
//!
//! Before the first tick, `node.pressure_pa`, `node.temperature_k` and
//! `edge.dissipation_w` are NaN by design (`core/src/snapshot.rs`), and
//! `serde_json` writes NaN as `null`. This module emits the engine's own JSON
//! unchanged, nulls included: substituting a number would invent data the
//! solver has not produced. Two consequences worth knowing before a scene
//! author reports them as defects:
//!
//! - Pre-tick JSON does **not** deserialize back into a `Snapshot` (`null` is
//!   not an `f64`). Post-tick JSON does.
//! - Pre-tick, `nodes[i].temperature_k` is `null` while
//!   `nodes[i].kind.temperature` (a tank's own state) is a real number. The
//!   first is a *solved* quantity, the second is *stored* — they are not
//!   two views of one thing.
//!
//! `tick_index() == 0` is the test for "nothing has been solved yet".

use std::collections::BTreeMap;

use refinery_core::graph::{EdgeId, NodeId};
use refinery_core::snapshot::Command;
use refinery_core::{Engine, SimError};
use serde::Serialize;
use thiserror::Error;

/// Everything that can go wrong between a frontend and the engine.
///
/// `Sim` keeps the whole `SimError` — including `SolverDiverged`'s
/// per-iteration `residual_history` — for Rust callers and tests.
/// [`ErrorReport`], the shape that crosses into a game, drops the history:
/// it is unbounded, it grows with the iteration count, and it would be
/// marshalled every failed tick inside `_physics_process` for a payload no
/// HUD reads. The summary (`iterations`, final `residual`) survives in the
/// message, because `SimError`'s own `Display` already carries it.
#[derive(Debug, Error)]
pub enum BridgeError {
    #[error("command is not valid JSON for a Command: {0}")]
    BadJson(String),

    #[error("{0}")]
    UnknownId(String),

    #[error("{0}")]
    UnknownName(String),

    #[error("{0}")]
    DuplicateName(String),

    #[error(transparent)]
    Sim(#[from] SimError),
}

/// What crosses into the game: a stable code plus prose.
///
/// The code is the part a scene may branch on and is therefore a contract —
/// treat it like the `#[serde(tag = "cmd")]` names. The message is for logs
/// and is free to change.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct ErrorReport {
    pub code: &'static str,
    pub message: String,
}

impl From<&BridgeError> for ErrorReport {
    fn from(err: &BridgeError) -> Self {
        // Exhaustive on purpose, both levels: a new `BridgeError` or a new
        // `SimError` variant must be given a code here or the crate stops
        // building. That is the whole mechanism keeping the code set honest.
        let code = match err {
            BridgeError::BadJson(_) => "bad_json",
            BridgeError::UnknownId(_) => "unknown_id",
            BridgeError::UnknownName(_) => "unknown_name",
            BridgeError::DuplicateName(_) => "duplicate_name",
            BridgeError::Sim(sim) => match sim {
                SimError::SolverDiverged { .. } => "solver_diverged",
                SimError::NonFiniteState { .. } => "non_finite_state",
                SimError::InvalidCommand(_) => "invalid_command",
                SimError::Scenario(_) => "scenario",
                SimError::Numerical(_) => "numerical",
            },
        };
        ErrorReport {
            code,
            message: err.to_string(),
        }
    }
}

/// What a command addresses. Extracting this is what forces the match on
/// `Command` to be exhaustive — see [`Bridge::apply_command_json`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Referent {
    Node(NodeId),
    Edge(EdgeId),
}

/// The id a command addresses, for validation before it reaches the engine.
///
/// **Wildcard-free on purpose.** A new `Command` variant does not compile
/// until its referent is declared here, which is the only thing standing
/// between a future command and the unvalidated-id panic described at the top
/// of this module. If a variant is ever added that addresses nothing, add a
/// `Referent::None` arm — deliberately, not by falling through a `_`.
fn referent(cmd: &Command) -> Referent {
    match cmd {
        Command::SetValveOpening { node, .. } => Referent::Node(*node),
        Command::SetPumpOn { node, .. } => Referent::Node(*node),
        Command::PuncturePipe { edge, .. } => Referent::Edge(*edge),
        Command::SetHeatInput { node, .. } => Referent::Node(*node),
        Command::SetFurnaceDuty { node, .. } => Referent::Node(*node),
        Command::SetCoolerDuty { node, .. } => Referent::Node(*node),
    }
}

/// One engine plus the phone book a frontend needs to talk to it.
pub struct Bridge {
    engine: Engine,
    /// name → id. `BTreeMap`, not `HashMap`: nothing here runs in the tick
    /// loop, but `node_names()` feeds a scene and iteration order must not be
    /// a source of run-to-run difference (CLAUDE.md rule 3).
    nodes: BTreeMap<String, NodeId>,
    edges: BTreeMap<String, EdgeId>,
    /// The ids the engine actually has, as read from its first snapshot.
    /// Kept separately from the maps above so that validation stays correct
    /// independently of naming.
    node_ids: Vec<u32>,
    edge_ids: Vec<u32>,
}

impl Bridge {
    /// Build an engine from scenario TOML **source**, not a path.
    ///
    /// Deliberate: Godot addresses files as `res://…`, which is not a
    /// filesystem path once a game is exported into a `.pck`. Reading is the
    /// binding's job (via Godot's `FileAccess`); translation is this module's.
    ///
    /// Fails if two nodes or two edges share a name — see [`Bridge::node_id`].
    pub fn load(toml_src: &str) -> Result<Self, BridgeError> {
        let file = refinery_scenarios::load_str(toml_src)?;
        let engine = refinery_scenarios::build_engine(&file)?;

        // Names and ids are real before the first tick even though the solved
        // floats are not, so the phone book can be built immediately.
        let snapshot = engine.snapshot();

        let mut nodes = BTreeMap::new();
        let mut node_ids = Vec::with_capacity(snapshot.nodes.len());
        for node in &snapshot.nodes {
            node_ids.push(node.id.0);
            if nodes.insert(node.name.clone(), node.id).is_some() {
                return Err(BridgeError::DuplicateName(format!(
                    "two nodes are named '{}', so a frontend cannot address either \
                     one by name",
                    node.name
                )));
            }
        }

        let mut edges = BTreeMap::new();
        let mut edge_ids = Vec::with_capacity(snapshot.edges.len());
        for edge in &snapshot.edges {
            edge_ids.push(edge.id.0);
            if edges.insert(edge.name.clone(), edge.id).is_some() {
                return Err(BridgeError::DuplicateName(format!(
                    "two edges are named '{}', so a frontend cannot address either \
                     one by name. Note that declaring `leak_to` on a pipe adds \
                     '<pipe>__downstream' and '<pipe>__leak' (docs/DESIGN.md §3b)",
                    edge.name
                )));
            }
        }

        Ok(Bridge {
            engine,
            nodes,
            edges,
            node_ids,
            edge_ids,
        })
    }

    /// Advance one fixed timestep.
    ///
    /// On `Err` the engine is left wherever it got to; this bridge adds no
    /// rollback and does not refuse later ticks. A diverged solve is not
    /// necessarily terminal — closing a valve may make the next one solve —
    /// so the decision to stop, reload or carry on belongs to the frontend.
    pub fn tick(&mut self) -> Result<(), BridgeError> {
        self.engine.tick().map_err(BridgeError::Sim)
    }

    /// The engine's own snapshot JSON, verbatim. See the module docs for what
    /// `null` means in it before the first tick.
    ///
    /// Infallible by type, not by luck: `Snapshot` is vectors, tuples,
    /// strings and floats, with no map keys and no `Serialize` impl that can
    /// fail, so `serde_json` has nothing to fail on. Returning a `Result`
    /// would hand a scene an error branch that can never be taken.
    pub fn snapshot_json(&self) -> String {
        serde_json::to_string(&self.engine.snapshot())
            .unwrap_or_else(|err| format!(r#"{{"bridge_error":"{err}"}}"#))
    }

    /// Parse one `Command` (the contract's own JSON, DESIGN §7), validate the
    /// id it addresses, and apply it.
    ///
    /// Nothing reaches the engine until both steps pass, so an invalid
    /// command cannot leave the plant half-modified.
    pub fn apply_command_json(&mut self, json: &str) -> Result<(), BridgeError> {
        let cmd: Command =
            serde_json::from_str(json).map_err(|err| BridgeError::BadJson(err.to_string()))?;

        match referent(&cmd) {
            Referent::Node(id) => {
                if !self.node_ids.contains(&id.0) {
                    return Err(BridgeError::UnknownId(format!(
                        "no node with id {} in this plant (it has {} nodes). Resolve a \
                         name with node_id() rather than assuming an id survives a \
                         scenario edit",
                        id.0,
                        self.node_ids.len()
                    )));
                }
            }
            Referent::Edge(id) => {
                if !self.edge_ids.contains(&id.0) {
                    return Err(BridgeError::UnknownId(format!(
                        "no edge with id {} in this plant (it has {} edges). Resolve a \
                         name with edge_id() rather than assuming an id survives a \
                         scenario edit",
                        id.0,
                        self.edge_ids.len()
                    )));
                }
            }
        }

        self.engine.apply(cmd).map_err(BridgeError::Sim)
    }

    /// Resolve a node name to the id `Command` wants. Ids are Godot integers
    /// (`i64`) because that is what a scene will hold them in.
    pub fn node_id(&self, name: &str) -> Result<i64, BridgeError> {
        self.nodes
            .get(name)
            .map(|id| i64::from(id.0))
            .ok_or_else(|| BridgeError::UnknownName(format!("no node named '{name}'")))
    }

    /// Resolve an edge name to the id `Command` wants.
    ///
    /// For a pipe the scenario declared punctureable, the declared name
    /// resolves to the **upstream half** — the edge `PuncturePipe` addresses
    /// and the one carrying `leak_mass_flow` (DESIGN §3b). The other two edges
    /// the split creates are reachable as `<name>__downstream` and
    /// `<name>__leak`.
    pub fn edge_id(&self, name: &str) -> Result<i64, BridgeError> {
        self.edges
            .get(name)
            .map(|id| i64::from(id.0))
            .ok_or_else(|| BridgeError::UnknownName(format!("no edge named '{name}'")))
    }

    /// Tick index of the last completed tick; `0` means nothing has been
    /// solved yet, which is exactly when snapshot floats are `null`.
    pub fn tick_index(&self) -> u64 {
        self.engine.snapshot().tick
    }

    /// Every node name, ascending. For a scene building its own lookup.
    pub fn node_names(&self) -> Vec<String> {
        self.nodes.keys().cloned().collect()
    }

    /// Every edge name, ascending.
    pub fn edge_names(&self) -> Vec<String> {
        self.edges.keys().cloned().collect()
    }
}
