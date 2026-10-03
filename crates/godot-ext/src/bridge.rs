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
//! 1. **The trust boundary.** Ids are validated *here*, against the set this
//!    bridge read from the engine, and an id that fails is never forwarded —
//!    it comes back as `unknown_id`, a stable code a scene can branch on.
//!
//!    Until M27 this guard was the only thing standing between a stale id and
//!    a panic, because `Engine::apply` indexed its graph directly. M27 moved a
//!    refusal into `core` (rule 5: the engine never panics), so an
//!    out-of-range id reaching the engine is now an ordinary
//!    `SimError::InvalidCommand`. **The guard stays anyway, and is not
//!    redundant**: without it the same mistake would reach a scene as
//!    `invalid_command`, the code that also means "a valve opening above 1",
//!    and a scene could no longer tell "your id is stale, re-resolve the name"
//!    from "your value is wrong". `core_refuses_an_out_of_range_id` in the
//!    tests pins the engine half; `an_out_of_range_id_is_refused_by_the_bridge_
//!    not_forwarded` pins this one.
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
//! 4. **Not-yet-loaded.** [`Session`] is a [`Bridge`] that may not exist yet.
//!    A Godot node is constructed before it is told which scenario to run, so
//!    something must answer "what happens if you tick before loading?" — and
//!    answering it in the binding would put a decision on the side `cargo
//!    test` cannot reach, which is the failure this whole split exists to
//!    prevent. Every `Session` method is total: JSON out instead of `Result`,
//!    `-1` instead of a missing id, and a code for every refusal. The binding
//!    is then one forwarding line per method.
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

use refinery_core::graph::{EdgeId, LoopId, NodeId, TripId};
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

    /// A [`Session`] method was called before any scenario loaded.
    #[error("no scenario is loaded; call load_scenario() first")]
    NotLoaded,

    /// Only the binding constructs this: `res://` paths are Godot's to read
    /// (a `.pck` is not a filesystem), so the failure arrives here already
    /// worded. It lives in this enum anyway, so the code below stays the one
    /// exhaustive place where a frontend-visible code is decided.
    #[error("{0}")]
    FileUnreadable(String),

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
            BridgeError::NotLoaded => "not_loaded",
            BridgeError::FileUnreadable(_) => "file_unreadable",
            BridgeError::UnknownId(_) => "unknown_id",
            BridgeError::UnknownName(_) => "unknown_name",
            BridgeError::DuplicateName(_) => "duplicate_name",
            BridgeError::Sim(sim) => match sim {
                SimError::SolverDiverged { .. } => "solver_diverged",
                SimError::NonFiniteState { .. } => "non_finite_state",
                SimError::InvalidCommand(_) => "invalid_command",
                SimError::Scenario(_) => "scenario",
                SimError::Numerical(_) => "numerical",
                SimError::AnchoringUnsettled { .. } => "anchoring_unsettled",
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
    /// A control loop (M8.2). Carried for exhaustiveness and **deliberately not
    /// validated here**, which is the same reachability argument that put the
    /// node and edge guards here in the first place: `Engine::apply` looks a
    /// `LoopId` up through `PlantGraph::control`/`control_mut`, which return
    /// `Option` and refuse an out-of-range id as an invalid command. There is no
    /// panic for a guard to stand in front of. It carries its id anyway so that
    /// the match stays wildcard-free. Since M27.1 the guard against a loop lookup
    /// that indexes belongs in `Engine::check_command_ids`, which refuses an
    /// unknown node or edge id the same way; this arm would add only an
    /// `unknown_id` code for loops.
    Loop(LoopId),
    /// A trip (M22, docs/DESIGN.md §26). Forwarded unchecked for `Loop`'s
    /// reason: `Engine::apply` looks a `TripId` up through `PlantGraph::trip`,
    /// which returns `Option` and refuses an out-of-range id as an invalid
    /// command, so there is no panic for a guard here to stand in front of.
    Trip(TripId),
}

/// The id a command addresses, for validation before it reaches the engine.
///
/// **Wildcard-free on purpose.** A new `Command` variant does not compile
/// until its referent is declared here, which is the only thing standing
/// between a future command and an id that reaches a scene as the wrong error
/// code (see the top of this module). `Engine::apply`'s own check is
/// wildcard-free for the same reason. If a variant is ever added that addresses nothing, add a
/// `Referent::None` arm — deliberately, not by falling through a `_`.
fn referent(cmd: &Command) -> Referent {
    match cmd {
        Command::SetValveOpening { node, .. } => Referent::Node(*node),
        Command::SetPumpOn { node, .. } => Referent::Node(*node),
        Command::PuncturePipe { edge, .. } => Referent::Edge(*edge),
        Command::SetHeatInput { node, .. } => Referent::Node(*node),
        Command::SetFurnaceDuty { node, .. } => Referent::Node(*node),
        Command::SetCoolerDuty { node, .. } => Referent::Node(*node),
        Command::SetControllerMode { loop_id, .. } => Referent::Loop(*loop_id),
        Command::SetSetpoint { loop_id, .. } => Referent::Loop(*loop_id),
        Command::ReplaceTubes { node } => Referent::Node(*node),
        Command::ResetTrip { trip_id } => Referent::Trip(*trip_id),
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
            // Forwarded unchecked, on purpose — see `Referent::Loop`. A frontend
            // reads a loop's id and name together off `snapshot.controls`, so
            // there is no name->id phone book here the way there is for nodes and
            // edges; adding one is a frontend affordance and belongs with M8.5's
            // Godot slice, not with the engine seam.
            Referent::Loop(_) => {}
            // The same, for a trip: its id and name travel together on
            // `snapshot.trips`.
            Referent::Trip(_) => {}
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

// ---------------------------------------------------------------- Session

/// What a lookup returns when there is nothing to look up.
///
/// Ids are `u32` in the engine, so no real id is negative and the sentinel
/// cannot collide with one. A scene tests `id < 0`; it does not get an error
/// object, because resolving a name is a startup step a scene either got right
/// or must fix in its own source.
pub const MISSING_ID: i64 = -1;

/// The result of a fallible call, as the game reads it: `null` when it worked,
/// an [`ErrorReport`] object when it did not.
///
/// `null` rather than an empty string or a `success: true` field, so GDScript's
/// `JSON.parse_string(...)` yields a falsy value on the happy path and a
/// dictionary with `code`/`message` otherwise. One shape, one branch.
pub fn outcome_json(result: Result<(), BridgeError>) -> String {
    match result {
        Ok(()) => "null".to_string(),
        Err(err) => serde_json::to_string(&ErrorReport::from(&err))
            // `ErrorReport` is two strings; serde has nothing to fail on. The
            // fallback exists so this function cannot panic inside a frame.
            .unwrap_or_else(|_| {
                r#"{"code":"bridge_error","message":"unserializable"}"#.to_string()
            }),
    }
}

/// A [`Bridge`] that may not exist yet, with every method total.
///
/// **This type exists so the gdext binding contains no decisions.** A Godot
/// node is constructed before it is told which scenario to run, so something
/// must answer "what happens if you tick before loading?" — and anything
/// answering that in terms of `GString`/`Variant` would be code `cargo test`
/// cannot reach (DESIGN §8). So it is answered here: every method takes and
/// returns plain Rust, every failure mode has a code, and the binding is a
/// one-line forward per method.
///
/// Errors are returned as JSON rather than `Result`, because that is the shape
/// that survives the crossing. [`Bridge`] keeps the `Result` API for Rust
/// callers and tests.
#[derive(Default)]
pub struct Session {
    bridge: Option<Bridge>,
}

impl Session {
    /// An empty session. Every accessor reports `not_loaded` until
    /// [`Session::load_scenario`] succeeds.
    pub fn new() -> Self {
        Self::default()
    }

    /// Load scenario TOML **source** (the binding reads `res://` itself).
    ///
    /// A failed load leaves the previously loaded plant in place, untouched —
    /// the same rule a refused `Command` follows. Swapping in a half-built
    /// plant, or dropping the running one, would make a typo in a scenario
    /// path destroy a running game.
    pub fn load_scenario(&mut self, toml_src: &str) -> String {
        match Bridge::load(toml_src) {
            Ok(bridge) => {
                self.bridge = Some(bridge);
                outcome_json(Ok(()))
            }
            Err(err) => outcome_json(Err(err)),
        }
    }

    /// Whether a plant is loaded.
    pub fn is_loaded(&self) -> bool {
        self.bridge.is_some()
    }

    /// Advance one fixed timestep. See [`Bridge::tick`] for what an `Err`
    /// leaves behind.
    pub fn tick(&mut self) -> String {
        outcome_json(match self.bridge.as_mut() {
            Some(bridge) => bridge.tick(),
            None => Err(BridgeError::NotLoaded),
        })
    }

    /// Apply one `Command` in the contract's own JSON (DESIGN §7).
    pub fn apply_command_json(&mut self, json: &str) -> String {
        outcome_json(match self.bridge.as_mut() {
            Some(bridge) => bridge.apply_command_json(json),
            None => Err(BridgeError::NotLoaded),
        })
    }

    /// The engine's snapshot JSON, or the JSON `null` when nothing is loaded.
    ///
    /// `null` is deliberately the same "nothing here" value the pre-tick
    /// float fields use, so a scene that already handles those handles this.
    pub fn snapshot_json(&self) -> String {
        match self.bridge.as_ref() {
            Some(bridge) => bridge.snapshot_json(),
            None => "null".to_string(),
        }
    }

    /// Resolve a node name, or [`MISSING_ID`] for an unknown name **or** an
    /// unloaded session. The two are not distinguished: both mean "you cannot
    /// address that", and a scene's response to either is to fix its source.
    pub fn node_id(&self, name: &str) -> i64 {
        self.bridge
            .as_ref()
            .and_then(|bridge| bridge.node_id(name).ok())
            .unwrap_or(MISSING_ID)
    }

    /// Resolve an edge name, or [`MISSING_ID`]. For a punctureable pipe this
    /// is the upstream half — see [`Bridge::edge_id`].
    pub fn edge_id(&self, name: &str) -> i64 {
        self.bridge
            .as_ref()
            .and_then(|bridge| bridge.edge_id(name).ok())
            .unwrap_or(MISSING_ID)
    }

    /// Tick index, or [`MISSING_ID`] when nothing is loaded — distinguishable
    /// from the `0` of a loaded-but-unsolved plant, which is a state a scene
    /// legitimately sees.
    pub fn tick_index(&self) -> i64 {
        match self.bridge.as_ref() {
            Some(bridge) => bridge.tick_index() as i64,
            None => MISSING_ID,
        }
    }

    /// Every node name as a JSON array, or `null` when nothing is loaded.
    pub fn node_names_json(&self) -> String {
        names_json(self.bridge.as_ref().map(Bridge::node_names))
    }

    /// Every edge name as a JSON array, or `null` when nothing is loaded.
    pub fn edge_names_json(&self) -> String {
        names_json(self.bridge.as_ref().map(Bridge::edge_names))
    }
}

fn names_json(names: Option<Vec<String>>) -> String {
    match names {
        Some(names) => serde_json::to_string(&names).unwrap_or_else(|_| "null".to_string()),
        None => "null".to_string(),
    }
}
