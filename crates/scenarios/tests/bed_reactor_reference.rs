//! M57.1: the catalyst bed as wired — the loader's three keys, the engine's
//! write-back, the snapshot, and the refusals of docs/DESIGN.md §62.2.
//!
//! `core::energy`'s `bed_tests` pin the bed's step against hand calculations on
//! hand-built flows: its books, its lag, its residence time, its zero flow and a
//! heat-absorbing reaction settling below its feed. Re-deriving them here would
//! add nothing. What is NOT covered there, and is what this file is for:
//!
//! 1. `type = "bed_reactor"` and its three keys reaching a `CatalystBed` in SI;
//! 2. the bed's temperature written back by `Engine::tick` and published on the
//!    snapshot, so a real hydraulic solve relaxes it as the lumped-capacitance
//!    hand calculation says, and its residence time follows the solved flow;
//! 3. the load-time refusals: a reaction model not solved exactly at every
//!    residence time, a bed in gas service, and each key out of range or absent.

use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{NodeSnapshot, Snapshot};

/// A hot liquid feed through a bed loaded hotter than it, into a sink. A small
/// pressure difference keeps the friction heat in the feed line to about a
/// hundredth of a kelvin, so the bed relaxes onto the supply's temperature.
const PLANT: &str = r#"
[meta]
name = "bed_fixture"

[simulation]
dt = 0.1

[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"

[[components]]
name = "vgo"
tb_c = 450.0
molar_mass_kg_per_mol = 0.4
density_kg_per_m3 = 900.0
cp_j_per_kg_k = 2500.0

[nodes.feed]
type = "source"
pressure_bar = 1.2
temperature_c = 380.0
composition = { vgo = 1.0 }

[nodes.bed]
type = "bed_reactor"
bed_heat_capacity_mj_per_k = 2.0
catalyst_volume_m3 = 3.0
bed_temperature_c = 420.0
# The wall (M58): never reached here.
wall_failure_c = 3000.0
wall_rupture_area_cm2 = 1.0
fluid_heating_value_mj_per_kg = 42.8
wall_fire_share = 0.1

[nodes.out]
type = "sink"
pressure_bar = 1.0
composition = { vgo = 1.0 }

[[pipes]]
name = "feed_line"
from = "feed"
to = "bed"
length_m = 20.0
diameter_m = 0.1

[[pipes]]
name = "product_line"
from = "bed"
to = "out"
length_m = 20.0
diameter_m = 0.1
"#;

const FEED_K: f64 = 380.0 + 273.15;
const BED_K: f64 = 420.0 + 273.15;
const CAPACITY_J_PER_K: f64 = 2.0e6;
const VOLUME_M3: f64 = 3.0;
const DENSITY: f64 = 900.0;
const CP: f64 = 2500.0;
const DT: f64 = 0.1;

fn node<'a>(snap: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    snap.nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("the fixture defines node '{name}'"))
}

fn bed_temperature(snap: &Snapshot) -> f64 {
    match &node(snap, "bed").kind {
        NodeKind::BedReactor { bed, .. } => bed.temperature.value(),
        other => panic!("'bed' must load as a bed reactor, got {other:?}"),
    }
}

fn build(src: &str) -> Result<refinery_core::engine::Engine, String> {
    let file = refinery_scenarios::load_str(src).map_err(|e| e.to_string())?;
    refinery_scenarios::build_engine(&file).map_err(|e| e.to_string())
}

/// The refusal a file that must not load gets, as text.
fn refused(src: &str) -> String {
    match build(src) {
        Ok(_) => panic!("this file must be refused at load"),
        Err(e) => e,
    }
}

#[test]
fn the_loader_builds_the_bed_from_its_three_keys() {
    let engine = build(PLANT).expect("the bed fixture must build");
    let snap = engine.snapshot();
    let NodeKind::BedReactor { bed, .. } = &node(&snap, "bed").kind else {
        panic!("'bed' must load as a bed reactor");
    };
    // MJ/K → J/K, m³ as written, °C → K: each against an independent constant.
    assert_eq!(bed.heat_capacity.value(), CAPACITY_J_PER_K);
    assert_eq!(bed.catalyst_volume.value(), VOLUME_M3);
    assert!((bed.temperature.value() - BED_K).abs() < 1e-12);
    // Before the first tick there is no sweep to report.
    assert!(node(&snap, "bed").reaction_heat_w.is_none());
    assert!(node(&snap, "bed").residence_time_s.is_none());
}

/// Through the real solve and `Engine::tick`'s write-back, a bed with no
/// reaction relaxes onto its feed as `T_in + (T₀ − T_in)·e^(−ṁ·cp·t/C)`, and its
/// residence time is `V·ρ/ṁ` on the solved flow.
#[test]
fn a_bed_with_no_reaction_relaxes_onto_its_feed_through_the_engine() {
    let mut engine = build(PLANT).expect("the bed fixture must build");
    engine.tick().expect("tick 1");
    let snap = engine.snapshot();
    let mass_flow = snap
        .edges
        .iter()
        .find(|e| e.name == "feed_line")
        .expect("the fixture defines 'feed_line'")
        .stream
        .mass_flow
        .value();
    assert!(mass_flow > 1.0, "the fixture must flow ({mass_flow} kg/s)");
    let tau = node(&snap, "bed")
        .residence_time_s
        .expect("a flowing bed reports its residence time");
    let expected_tau = VOLUME_M3 * DENSITY / mass_flow;
    assert!(
        (tau - expected_tau).abs() <= 1e-12 * expected_tau,
        "τ = {tau} s, by hand {expected_tau} s"
    );
    assert_eq!(node(&snap, "bed").reaction_heat_w, Some(0.0));

    let rate = mass_flow * CP / CAPACITY_J_PER_K;
    let mut ticks = 1;
    let mut last = bed_temperature(&snap);
    while ticks < 600 {
        engine.tick().expect("the bed fixture ticks");
        ticks += 1;
        let now = bed_temperature(&engine.snapshot());
        assert!(now < last, "the bed cools monotonically onto its feed");
        last = now;
    }
    let expected = FEED_K + (BED_K - FEED_K) * (-rate * DT * f64::from(ticks)).exp();
    assert!(
        (last - expected).abs() < 0.02,
        "after {ticks} ticks the bed is at {last} K; the lag says {expected} K"
    );
    assert!(
        (BED_K - last) > 0.2 * (BED_K - FEED_K),
        "the fixture must relax visibly ({last} K)"
    );
}

#[test]
fn a_bed_is_refused_with_a_reaction_model_not_solved_at_every_residence_time() {
    for model in ["lookup", "fcc"] {
        let src = PLANT.replace("reactions = \"none\"", &format!("reactions = \"{model}\""));
        let err = refused(&src);
        assert!(
            err.contains("bed reactor 'bed'") && err.contains("§62.2"),
            "{model}: {err}"
        );
    }
}

#[test]
fn a_bed_in_gas_service_is_refused() {
    let src = PLANT.replace("density_kg_per_m3 = 900.0", "phase = \"gas\"");
    let err = refused(&src);
    assert!(
        err.contains("gas service") && err.contains("§62.2"),
        "{err}"
    );
}

#[test]
fn each_bed_key_is_required_and_range_checked() {
    for key in [
        "bed_heat_capacity_mj_per_k",
        "catalyst_volume_m3",
        "bed_temperature_c",
    ] {
        let line = PLANT
            .lines()
            .find(|l| l.starts_with(key))
            .expect("the fixture declares every key");
        let err = refused(&PLANT.replace(line, ""));
        assert!(err.contains(key), "absent {key}: {err}");
    }
    for (line, bad) in [
        (
            "bed_heat_capacity_mj_per_k = 2.0",
            "bed_heat_capacity_mj_per_k = 0.0",
        ),
        ("catalyst_volume_m3 = 3.0", "catalyst_volume_m3 = -1.0"),
        ("bed_temperature_c = 420.0", "bed_temperature_c = -300.0"),
    ] {
        let err = refused(&PLANT.replace(line, bad));
        let key = bad.split(' ').next().unwrap();
        assert!(
            err.contains(key) && err.contains("bed reactor 'bed'"),
            "{bad}: {err}"
        );
    }
}

/// A bed is one feed in, one product out, as a furnace is: its residence time is
/// "the" feed's flow. A second outlet is refused at load, by the degree rule the
/// solvers also enforce.
#[test]
fn a_bed_with_two_outlets_is_refused() {
    let src = format!(
        "{PLANT}
[[pipes]]
name = \"second_outlet\"
from = \"bed\"
to = \"out\"
length_m = 20.0
diameter_m = 0.1
"
    );
    let err = refused(&src);
    assert!(
        err.contains("bed") && err.contains("exactly 1 inlet + 1 outlet"),
        "{err}"
    );
}

/// A control LOOP on a bed through its own FEED valve is refused AT LOAD (M57.3,
/// docs/DESIGN.md §62.10; narrowed by M57.4, §62.11): a loop on a bed acts
/// through a quench only, and a valve on the bed's own line mixes nothing in.
/// The quench cells are `quench_reference.rs`'s.
#[test]
fn a_loop_on_a_bed_is_refused_at_load() {
    let src = format!(
        "{}
[nodes.feed_valve]
type = \"valve\"
kv = 200.0
opening = 1.0

[[pipes]]
name = \"valve_line\"
from = \"feed_valve\"
to = \"bed\"
length_m = 5.0
diameter_m = 0.1

[[controls]]
name = \"bed_temperature\"
measurement = {{ node = \"bed\", variable = \"temperature\" }}
actuator = \"feed_valve\"
algorithm = \"p\"
mode = \"auto\"
setpoint_c = 400.0
gain_per_k = 0.01
initial_output = 0.5
",
        PLANT.replace(
            "from = \"feed\"
to = \"bed\"",
            "from = \"feed\"
to = \"feed_valve\""
        )
    );
    assert!(
        src.contains("to = \"feed_valve\""),
        "the fixture rewrite must apply"
    );
    let err = refused(&src);
    assert!(
        err.contains("catalyst bed 'bed'")
            && err.contains("not a quench on that bed")
            && err.contains("without joining another stream")
            && err.contains("§62.11"),
        "{err}"
    );
}
