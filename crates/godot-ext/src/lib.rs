//! GDExtension adapter. Two halves, split by the `godot` feature — see
//! `Cargo.toml` and DESIGN §8.
//!
//! Rules: this crate owns ALL translation between engine types and Godot
//! types. No Godot type crosses into core/solvers/scenarios, ever.

pub mod bridge;

#[cfg(feature = "godot")]
mod binding;
