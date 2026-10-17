//! Every integration test of this crate, as one test program.
//!
//! Each file below was its own test program; one program links the crate once
//! instead of once per file, and the harness runs every test in parallel
//! rather than one file after another. Run one file's tests with a filter:
//! `cargo test -p refinery-scenarios --test suite <module>::`.

#[path = "bed_reactor_reference.rs"]
mod bed_reactor_reference;
#[path = "bed_trip_reference.rs"]
mod bed_trip_reference;
#[path = "bed_wall_reference.rs"]
mod bed_wall_reference;
#[path = "boiloff_reference.rs"]
mod boiloff_reference;
#[path = "burnout_reference.rs"]
mod burnout_reference;
#[path = "burst_during_stop_reference.rs"]
mod burst_during_stop_reference;
#[path = "capacitive_vessel_reference.rs"]
mod capacitive_vessel_reference;
#[path = "cascade_column.rs"]
mod cascade_column;
#[path = "cascade_control_reference.rs"]
mod cascade_control_reference;
#[path = "cavitation_reference.rs"]
mod cavitation_reference;
#[path = "check_valve_reference.rs"]
mod check_valve_reference;
#[path = "choked_flow_reference.rs"]
mod choked_flow_reference;
#[path = "coil_trip_reference.rs"]
mod coil_trip_reference;
#[path = "column_reference.rs"]
mod column_reference;
#[path = "consumption_reference.rs"]
mod consumption_reference;
#[path = "control_reference.rs"]
mod control_reference;
#[path = "cooler_reference.rs"]
mod cooler_reference;
#[path = "crude_column_cascade.rs"]
mod crude_column_cascade;
#[path = "crude_unit.rs"]
mod crude_unit;
#[path = "dissipation_reference.rs"]
mod dissipation_reference;
#[path = "dry_fired_reference.rs"]
mod dry_fired_reference;
#[path = "dry_tank_reference.rs"]
mod dry_tank_reference;
#[path = "fcc_plant_reference.rs"]
mod fcc_plant_reference;
#[path = "fidelity_selection.rs"]
mod fidelity_selection;
#[path = "fill_valve_reference.rs"]
mod fill_valve_reference;
#[path = "fire_reporting.rs"]
mod fire_reporting;
#[path = "flashing_check_valve_reference.rs"]
mod flashing_check_valve_reference;
#[path = "flashing_rundown_reference.rs"]
mod flashing_rundown_reference;
#[path = "flow_control_reference.rs"]
mod flow_control_reference;
#[path = "flow_trip_reference.rs"]
mod flow_trip_reference;
#[path = "furnace_reference.rs"]
mod furnace_reference;
#[path = "furnace_trip_reference.rs"]
mod furnace_trip_reference;
#[path = "gas_check_valve_reference.rs"]
mod gas_check_valve_reference;
#[path = "gas_density_reference.rs"]
mod gas_density_reference;
#[path = "gas_lock_reference.rs"]
mod gas_lock_reference;
#[path = "gas_pocket_reference.rs"]
mod gas_pocket_reference;
#[path = "heat_balance_column.rs"]
mod heat_balance_column;
#[path = "heat_recovery_reference.rs"]
mod heat_recovery_reference;
#[path = "hydrocracker_reference.rs"]
mod hydrocracker_reference;
#[path = "hydrogen_leak_reference.rs"]
mod hydrogen_leak_reference;
#[path = "hydrogen_quench_reference.rs"]
mod hydrogen_quench_reference;
#[path = "hydrogen_reference.rs"]
mod hydrogen_reference;
#[path = "inner_limit_reference.rs"]
mod inner_limit_reference;
#[path = "isothermal_plant.rs"]
mod isothermal_plant;
#[path = "kv_reference.rs"]
mod kv_reference;
#[path = "leak_reference.rs"]
mod leak_reference;
#[path = "level_control_demo.rs"]
mod level_control_demo;
#[path = "line_flash_reference.rs"]
mod line_flash_reference;
#[path = "loop_watches.rs"]
mod loop_watches;
#[path = "m1_acceptance.rs"]
mod m1_acceptance;
#[path = "manual_trip_reference.rs"]
mod manual_trip_reference;
#[path = "methane_cp_reference.rs"]
mod methane_cp_reference;
#[path = "outlet_control_reference.rs"]
mod outlet_control_reference;
#[path = "overflow_reference.rs"]
mod overflow_reference;
#[path = "pressure_control_demo.rs"]
mod pressure_control_demo;
#[path = "pressure_control_reference.rs"]
mod pressure_control_reference;
#[path = "pump_cavitation_reference.rs"]
mod pump_cavitation_reference;
#[path = "pump_gas_lock_reference.rs"]
mod pump_gas_lock_reference;
#[path = "pump_stop_reference.rs"]
mod pump_stop_reference;
#[path = "pump_two_phase_reference.rs"]
mod pump_two_phase_reference;
#[path = "quench_reference.rs"]
mod quench_reference;
#[path = "reactor_reference.rs"]
mod reactor_reference;
#[path = "relief_blowdown_reference.rs"]
mod relief_blowdown_reference;
#[path = "relief_slope_reference.rs"]
mod relief_slope_reference;
#[path = "relief_valve_reference.rs"]
mod relief_valve_reference;
#[path = "restart_permissive_reference.rs"]
mod restart_permissive_reference;
#[path = "restart_tubes_reference.rs"]
mod restart_tubes_reference;
#[path = "reverse_action_reference.rs"]
mod reverse_action_reference;
#[path = "slate_loading.rs"]
mod slate_loading;
#[path = "snapshot_slate.rs"]
mod snapshot_slate;
#[path = "step_tick_reference.rs"]
mod step_tick_reference;
#[path = "stiff_pair_reference.rs"]
mod stiff_pair_reference;
#[path = "supply_command_reference.rs"]
mod supply_command_reference;
#[path = "temperature_control_demo.rs"]
mod temperature_control_demo;
#[path = "temperature_control_reference.rs"]
mod temperature_control_reference;
#[path = "temperature_heating_demo.rs"]
mod temperature_heating_demo;
#[path = "trip_demo.rs"]
mod trip_demo;
#[path = "trip_reference.rs"]
mod trip_reference;
#[path = "trip_reset_reference.rs"]
mod trip_reset_reference;
#[path = "trip_stop_reference.rs"]
mod trip_stop_reference;
#[path = "trip_watches.rs"]
mod trip_watches;
#[path = "upwind_temperature_reference.rs"]
mod upwind_temperature_reference;
#[path = "vapour_recovery_reference.rs"]
mod vapour_recovery_reference;
#[path = "vapour_recovery_train.rs"]
mod vapour_recovery_train;
#[path = "water_properties.rs"]
mod water_properties;
#[path = "water_slug_reference.rs"]
mod water_slug_reference;
#[path = "wet_rundown_reference.rs"]
mod wet_rundown_reference;
