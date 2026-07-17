//! refinery-core: types, plant graph, engine loop, and solver traits.
//!
//! HARD RULES (see CLAUDE.md):
//! - No Godot, no I/O, no solver implementations in this crate.
//! - Deterministic: no HashMap iteration, no wall clock, no thread RNG.
//! - No panics: fallible paths return `SimError`.
//! - SI units via `units` newtypes on all public APIs.

pub mod components;
pub mod energy;
pub mod engine;
pub mod error;
pub mod graph;
pub mod snapshot;
pub mod stream;
pub mod traits;
pub mod units;

pub use engine::{Engine, EngineConfig};
pub use error::SimError;
pub use snapshot::{Command, Snapshot};
