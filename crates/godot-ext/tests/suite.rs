//! Every integration test of this crate, as one test program.
//!
//! Each file below was its own test program; one program links the crate once
//! instead of once per file, and the harness runs every test in parallel
//! rather than one file after another. Run one file's tests with a filter:
//! `cargo test -p refinery-godot-ext --test suite <module>::`.

#[path = "bridge.rs"]
mod bridge;
#[path = "column_screen.rs"]
mod column_screen;
#[path = "furnace_screen.rs"]
mod furnace_screen;
#[path = "pump_screen.rs"]
mod pump_screen;
#[path = "reactor_screen.rs"]
mod reactor_screen;
