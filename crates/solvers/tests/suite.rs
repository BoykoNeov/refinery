//! Every integration test of this crate, as one test program.
//!
//! Each file below was its own test program; one program links the crate once
//! instead of once per file, and the harness runs every test in parallel
//! rather than one file after another. Run one file's tests with a filter:
//! `cargo test -p refinery-solvers --test suite <module>::`.

#[path = "cavitation_contract.rs"]
mod cavitation_contract;
#[path = "composition_transport.rs"]
mod composition_transport;
#[path = "cracked_valve_disc.rs"]
mod cracked_valve_disc;
#[path = "dead_end_disc.rs"]
mod dead_end_disc;
#[path = "energy_invariants.rs"]
mod energy_invariants;
#[path = "enthalpy_model.rs"]
mod enthalpy_model;
#[path = "fidelity_agreement.rs"]
mod fidelity_agreement;
#[path = "gas_valve_invariants.rs"]
mod gas_valve_invariants;
#[path = "invariants.rs"]
mod invariants;
#[path = "newton_reference.rs"]
mod newton_reference;
#[path = "pipe_ambient_reference.rs"]
mod pipe_ambient_reference;
#[path = "pump_suction_contract.rs"]
mod pump_suction_contract;
#[path = "reference/cascade.rs"]
mod reference_cascade;
#[path = "reference/compressor.rs"]
mod reference_compressor;
#[path = "reference/flash.rs"]
mod reference_flash;
#[path = "reference/four_lump.rs"]
mod reference_four_lump;
#[path = "reference/free_water.rs"]
mod reference_free_water;
#[path = "reference/hydrocracking.rs"]
mod reference_hydrocracking;
#[path = "reference/line_flash.rs"]
mod reference_line_flash;
#[path = "reference/orifice.rs"]
mod reference_orifice;
#[path = "reference/two_phase_head.rs"]
mod reference_two_phase_head;
#[path = "reference/vapour_pressure.rs"]
mod reference_vapour_pressure;
#[path = "reference/water.rs"]
mod reference_water;
#[path = "separation_contract.rs"]
mod separation_contract;
#[path = "vapour_recovery_contract.rs"]
mod vapour_recovery_contract;
