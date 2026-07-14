//! GDExtension adapter — sketch for M6. Intentionally not compiled until
//! then (workspace-excluded). Shape it will take with the `godot` crate:
//!
//! ```ignore
//! use godot::prelude::*;
//!
//! struct RefineryExtension;
//! #[gdextension]
//! unsafe impl ExtensionLibrary for RefineryExtension {}
//!
//! /// A Godot Node wrapping one engine instance.
//! #[derive(GodotClass)]
//! #[class(base=Node)]
//! pub struct RefinerySim {
//!     engine: Option<refinery_core::Engine>,
//!     base: Base<Node>,
//! }
//!
//! #[godot_api]
//! impl RefinerySim {
//!     #[func]
//!     fn load_scenario(&mut self, path: GString) -> bool { /* build_engine */ }
//!
//!     /// Call from _physics_process. Sync single-threaded first;
//!     /// move to a worker thread + snapshot channel only if profiling
//!     /// shows tick time threatening the frame budget (DESIGN.md §8).
//!     #[func]
//!     fn tick(&mut self) -> bool { /* engine.tick(), report Err as signal */ }
//!
//!     /// Whole snapshot as JSON string for prototyping; add typed
//!     /// accessors (get_tank_level(name) -> f64, etc.) for per-frame
//!     /// hot paths before shipping.
//!     #[func]
//!     fn snapshot_json(&self) -> GString { /* serde_json */ }
//!
//!     #[func]
//!     fn send_command_json(&mut self, cmd: GString) -> bool { /* Command */ }
//! }
//! ```
//!
//! Rules: this crate owns ALL translation between engine types and Godot
//! types. No Godot type crosses into core/solvers/scenarios, ever.
