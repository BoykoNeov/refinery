//! M59.3: a HYDROGEN quench between two catalyst beds (docs/DESIGN.md §64.5,
//! §64.12), gated on `scenarios/hydrocracker_hydrogen_quench.toml` and fixtures
//! cut from it.
//!
//! Cold hydrogen off the recycle compressor's discharge joins the first bed's
//! hydrogen outflow ahead of the second bed's pass, through M57.4's quench
//! machinery (§62.11): `quench_layout` now admits a mix ahead of a bed's
//! hydrogen pass, a compressor and a tee on the quench line, and a gas vessel
//! as its origin. The gates:
//!
//! 1. the cascade catches the second bed the plant is born heading for, and
//!    holds it, on both solvers;
//! 2. the quench cools through the pass: the mix is the two streams' blend to
//!    the bit's neighbourhood, and what the quench takes off the discharge the
//!    first bed's pass no longer gets;
//! 3. in MANUAL the backup trip depressures the loop and the second bed comes
//!    back under its wall's limit; with no trip either its wall fails;
//! 4. the freeze, on a quench drawn from a hydrogen supply whose temperature a
//!    command sets: MANUAL on the first pass that reads it warm, held, AUTO
//!    refused while it stands and admitted once it is cold again;
//! 5. the single-loop form, on the quench valve itself;
//! 6. the layouts refused: a stream joining the quench line, a quench of the
//!    wrong phase either way, a mix feeding another bed's pass, a quench not
//!    cold at load;
//! 7. the faceplate's `quench` on the bed loop alone, read from the separator
//!    at load.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{ControlMode, ControlledValue, LoopId, NodeKind, TripState, WallState};
use refinery_core::snapshot::{Command, ControlSnapshot, Snapshot};
use refinery_core::units::Kelvin;
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/hydrocracker_hydrogen_quench.toml");

const SETPOINT_C: f64 = 400.0;
/// Thirty minutes of 0.1 s ticks.
const HALF_HOUR: u64 = 18_000;

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the fixture must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("the fixture must build: {e}"))
}

fn refusal(src: &str) -> String {
    match refinery_scenarios::load_str(src) {
        Err(e) => e.to_string(),
        Ok(file) => match refinery_scenarios::build_engine(&file) {
            Ok(_) => panic!("this plant should not have loaded"),
            Err(e) => e.to_string(),
        },
    }
}

fn swap(src: &str, from: &str, to: &str) -> String {
    assert_eq!(
        src.matches(from).count(),
        1,
        "the fixture's substitution must land exactly once: {from:?}"
    );
    src.replacen(from, to, 1)
}

fn with_solver(src: &str, solver: &str) -> String {
    swap(src, r#"flow = "newton""#, &format!(r#"flow = "{solver}""#))
}

fn tick(engine: &mut Engine) {
    let t = engine.snapshot().tick + 1;
    engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
}

fn bed_c(engine: &Engine, name: &str) -> f64 {
    let id = engine.graph.find_node(name).expect("a bed");
    match &engine.graph.node(id).kind {
        NodeKind::BedReactor { bed, .. } => bed.temperature.value() - 273.15,
        other => panic!("'{name}' is a catalyst bed, not {other:?}"),
    }
}

fn wall(engine: &Engine, name: &str) -> WallState {
    let id = engine.graph.find_node(name).expect("a bed");
    match &engine.graph.node(id).kind {
        NodeKind::BedReactor { wall, .. } => wall.state,
        other => panic!("'{name}' is a catalyst bed, not {other:?}"),
    }
}

fn opening(engine: &Engine, name: &str) -> f64 {
    let id = engine.graph.find_node(name).expect("a valve");
    match engine.graph.node(id).kind {
        NodeKind::Valve { opening, .. } => opening,
        ref other => panic!("'{name}' is a valve, not {other:?}"),
    }
}

fn edge<'a>(snapshot: &'a Snapshot, name: &str) -> &'a refinery_core::stream::Stream {
    &snapshot
        .edges
        .iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("no pipe '{name}'"))
        .stream
}

fn flow(snapshot: &Snapshot, name: &str) -> f64 {
    edge(snapshot, name).mass_flow.value()
}

fn faceplate<'a>(snapshot: &'a Snapshot, name: &str) -> &'a ControlSnapshot {
    snapshot
        .controls
        .iter()
        .find(|c| c.name == name)
        .unwrap_or_else(|| panic!("no loop '{name}'"))
}

fn loop_id(snapshot: &Snapshot, name: &str) -> LoopId {
    faceplate(snapshot, name).id
}

/// Whether the bed loop's last quench check found the quench cold.
fn quench_cold(snapshot: &Snapshot) -> bool {
    faceplate(snapshot, "bed2_temperature")
        .quench
        .expect("the bed loop publishes its quench check")
        .is_cold()
}

fn flow_target(snapshot: &Snapshot) -> f64 {
    match faceplate(snapshot, "quench_flow").setpoint {
        ControlledValue::Flow { kg_per_s } => kg_per_s.value(),
        other => panic!("a flow loop's target is a flow, not {other:?}"),
    }
}

/// The bed loop's block, from its name to the line before the flow loop.
fn bed_loop_block() -> &'static str {
    let start = DEMO
        .find("[[controls]]\nname = \"bed2_temperature\"")
        .expect("the demo's bed loop");
    let end = DEMO
        .find("[[controls]]\nname = \"quench_flow\"")
        .expect("the demo's flow loop");
    &DEMO[start..end]
}

/// The demo with its bed loop in MANUAL.
fn bed_loop_manual(src: &str) -> String {
    let block = bed_loop_block();
    swap(
        src,
        block,
        &block.replace("mode = \"auto\"", "mode = \"manual\""),
    )
}

/// The demo with its trip out of reach.
fn without_trip(src: &str) -> String {
    swap(src, "limit_c = 420.0", "limit_c = 2000.0")
}

/// The single-loop form (gate 5): the bed loop writes the quench valve itself,
/// and the flow loop is gone. The gain from a sweep (K 0.01, 0.025, 0.05 per
/// K at T_i 300 s): the one that peaks under a kelvin over the target.
fn single_loop(src: &str) -> String {
    let start = src
        .find("[[controls]]\nname = \"bed2_temperature\"")
        .expect("the demo's bed loop");
    let end = src.find("# The backup: fires").expect("the demo's trip");
    let single = r#"[[controls]]
name = "bed2_temperature"
measurement = { node = "bed2", variable = "temperature" }
actuator = "quench_valve"
algorithm = "pi"
mode = "auto"
action = "direct"
setpoint_c = 400.0
gain_per_k = 0.05
integral_time_s = 300.0
initial_output = 0.03

"#;
    format!("{}{single}{}", &src[..start], &src[end..])
}

/// The demo with its quench drawn from a hydrogen SUPPLY at the discharge's
/// pressure and temperature (159.5 bar, 67.8 °C, measured ten minutes in)
/// instead of the tee, so a command can set the quench's temperature (gate 4).
/// The supply ADDS its hydrogen to the loop, which the makeup cannot take
/// out — the loop would fill to the supply's pressure and the quench lose its
/// drop — so the separator's pressure is held on the vent instead (a drain,
/// direct), the makeup shut.
fn quench_from_supply(src: &str) -> String {
    let src = swap(
        src,
        "[nodes.quench_check]",
        "[nodes.quench_gas]\ntype = \"source\"\npressure_bar = 159.5\ntemperature_c = 67.8\n\
         composition = { hydrogen = 1.0 }\n\n[nodes.quench_check]",
    );
    let src = swap(
        &src,
        "actuator = \"makeup_valve\"\nalgorithm = \"pi\"\nmode = \"auto\"\naction = \"reverse\"",
        "actuator = \"vent_valve\"\nalgorithm = \"pi\"\nmode = \"auto\"\naction = \"direct\"",
    );
    swap(
        &src,
        "name = \"quench_supply\"\nfrom = \"tee\"",
        "name = \"quench_supply\"\nfrom = \"quench_gas\"",
    )
}

/// **Gate 1: the cascade catches the second bed.** Born at 395 °C with the
/// quench at its 0.1 kg/s minimum, the second bed takes the first bed's
/// ~404 °C oil and hydrogen and heads for the runaway gate 3 shows. The loop
/// raises the quench: the bed peaks under half a kelvin over its target and
/// settles onto it, the valve about a quarter open, the quench cold and the
/// loop in AUTO on every tick, the trip armed throughout. The first bed warms
/// a little as the quench takes its hydrogen, and stays calm.
fn the_cascade_catches_the_second_bed(solver: &str) {
    let mut engine = build(&with_solver(DEMO, solver));
    let mut peak = f64::MIN;
    let mut widest = 0.0_f64;
    for _ in 0..HALF_HOUR {
        tick(&mut engine);
        let snapshot = engine.snapshot();
        assert!(quench_cold(&snapshot), "{solver}: the quench stays cold");
        assert_eq!(
            faceplate(&snapshot, "bed2_temperature").mode,
            ControlMode::Auto,
            "{solver}: the bed loop stays in AUTO"
        );
        assert_eq!(snapshot.trips[0].state, TripState::Armed, "{solver}");
        peak = peak.max(bed_c(&engine, "bed2"));
        widest = widest.max(opening(&engine, "quench_valve"));
    }
    let end = bed_c(&engine, "bed2");
    assert!(
        peak > SETPOINT_C && peak < SETPOINT_C + 0.5,
        "{solver}: the bed peaks just over its target, at {peak:.3} °C"
    );
    assert!(
        (end - SETPOINT_C).abs() < 0.02,
        "{solver}: settled on the target, {end:.4} °C"
    );
    assert!(
        widest > 0.2 && widest < 0.4,
        "{solver}: the quench valve opens, at most {widest:.3}"
    );
    let first = bed_c(&engine, "bed1");
    assert!(
        first > 404.0 && first < 405.5,
        "{solver}: the first bed stays calm, {first:.3} °C"
    );
    let quench = flow(&engine.snapshot(), "quench_line");
    assert!(
        quench > 0.8 && quench < 1.0,
        "{solver}: about 0.9 kg/s of cold hydrogen holds it, {quench:.4}"
    );
}

#[test]
fn slow_the_cascade_catches_the_second_bed_newton() {
    the_cascade_catches_the_second_bed("newton");
}

#[test]
fn slow_the_cascade_catches_the_second_bed_simple() {
    the_cascade_catches_the_second_bed("simple");
}

/// **Gate 2: the quench cools through the pass.** Hydrogen at constant `c_p`
/// mixes by mass, so the second pass's inlet is the blend of the first pass's
/// outflow and the quench to rounding; the discharge divides at the tee into
/// exactly the two; and the hydrogen the quench takes the first bed's pass no
/// longer gets.
#[test]
fn the_quench_cools_through_the_pass() {
    let mut engine = build(DEMO);
    tick(&mut engine);
    let at_load = engine.snapshot();
    for _ in 0..6_000 {
        tick(&mut engine);
    }
    let s = engine.snapshot();
    let (main, quench, into) = (
        edge(&s, "interbed_gas"),
        edge(&s, "quench_line"),
        edge(&s, "bed2_gas_line"),
    );
    let (m1, mq) = (main.mass_flow.value(), quench.mass_flow.value());
    let blend = (m1 * main.temperature.value() + mq * quench.temperature.value()) / (m1 + mq);
    // The junction's own temperature: a pipe's stream is its OUTLET, warmed
    // by its own dissipation (§4a), so the mix's outlet pipe is not the mix.
    let mix_k = s
        .nodes
        .iter()
        .find(|n| n.name == "mix")
        .expect("the mix")
        .temperature_k;
    assert!(
        (mix_k - blend).abs() < 1e-9 * blend,
        "the mix is the blend: {mix_k} K against {blend} K"
    );
    assert!(
        (into.mass_flow.value() - (m1 + mq)).abs() < 1e-7,
        "the mix carries both"
    );
    assert!(
        into.temperature.value() < main.temperature.value() - 40.0,
        "the quench cools the second pass's hydrogen by tens of kelvin"
    );
    let tee = flow(&s, "discharge_line") - flow(&s, "loop_line") - flow(&s, "quench_supply");
    assert!(tee.abs() < 1e-7, "the tee divides the discharge, {tee:e}");
    let (before, after) = (
        flow(&at_load, "reactor_gas_line"),
        flow(&s, "reactor_gas_line"),
    );
    assert!(
        after < before - 0.2,
        "the first bed's hydrogen falls as the quench opens: {before:.3} -> {after:.3} kg/s"
    );
    // The faceplate's two temperatures are the two pipes'.
    let reading = faceplate(&s, "bed2_temperature").quench.expect("published");
    let last = engine.snapshot();
    assert!(reading.quench_k.value() < reading.joins_k.value() - 300.0);
    assert!(
        (reading.joins_k.value() - edge(&last, "interbed_gas").temperature.value()).abs() < 0.05,
        "the main stream is the first pass's outflow, {reading:?}"
    );
}

/// **Gate 3: in MANUAL the backup trip depressures the loop.** The bed loop in
/// MANUAL holds the quench at its minimum; the second bed reaches the trip's
/// 420 °C five minutes in (tick 3 047 on both solvers), the trip opens the
/// vent and shuts the makeup, and the starved bed turns near 432 °C and is
/// back under 390 °C by forty minutes, both walls intact.
fn in_manual_the_trip_depressures_the_loop(solver: &str) {
    let mut engine = build(&with_solver(&bed_loop_manual(DEMO), solver));
    let mut fired = None;
    let mut peak = f64::MIN;
    for _ in 0..24_000 {
        tick(&mut engine);
        let snapshot = engine.snapshot();
        if fired.is_none() && snapshot.trips[0].state.is_tripped() {
            fired = Some(snapshot.tick);
            assert_eq!(opening(&engine, "vent_valve"), 1.0, "{solver}: vented");
            assert_eq!(opening(&engine, "makeup_valve"), 0.0, "{solver}: shut");
        }
        peak = peak.max(bed_c(&engine, "bed2"));
    }
    let fired = fired.unwrap_or_else(|| panic!("{solver}: the trip must fire"));
    assert!(
        (2_400..3_600).contains(&fired),
        "{solver}: the trip fires between 4 and 6 minutes, on tick {fired}"
    );
    assert!(
        peak > 425.0 && peak < 440.0,
        "{solver}: the starved bed turns near 432 °C, {peak:.2}"
    );
    let end = bed_c(&engine, "bed2");
    assert!(end < 390.0, "{solver}: back near its feed, {end:.2} °C");
    for bed in ["bed1", "bed2"] {
        assert_eq!(wall(&engine, bed), WallState::Intact, "{solver}: {bed}");
    }
}

#[test]
fn slow_in_manual_the_trip_depressures_the_loop_newton() {
    in_manual_the_trip_depressures_the_loop("newton");
}

#[test]
fn slow_in_manual_the_trip_depressures_the_loop_simple() {
    in_manual_the_trip_depressures_the_loop("simple");
}

/// **Gate 3, continued: with no trip either, the second bed's wall fails**
/// 28 minutes in (tick 17 081 on Newton), past 550 °C.
#[test]
fn slow_with_no_trip_either_the_wall_fails() {
    let mut engine = build(&without_trip(&bed_loop_manual(DEMO)));
    let mut failed = None;
    for _ in 0..HALF_HOUR {
        tick(&mut engine);
        if failed.is_none() && wall(&engine, "bed2") != WallState::Intact {
            failed = Some(engine.snapshot().tick);
        }
    }
    let failed = failed.expect("the second bed's wall fails");
    assert!(
        (15_000..18_000).contains(&failed),
        "between 25 and 30 minutes, tick {failed}"
    );
    assert_eq!(wall(&engine, "bed1"), WallState::Intact);
}

/// **Gate 4: the freeze.** On the demo with its quench drawn from a 67.8 °C
/// hydrogen supply, the loop holds the bed; then the supply is heated to
/// 420 °C, over the first bed's ~405 °C outflow. The tick after the command
/// carries the hot quench through its pipes; the pass at the top of the tick
/// after THAT reads it and switches the loop to MANUAL. The flow target is
/// held, AUTO is refused while the quench stands warm, and once the supply is
/// cold again AUTO is admitted on the first tick the check reads it cold.
fn a_warm_quench_freezes_the_loop(solver: &str) {
    let mut engine = build(&with_solver(&quench_from_supply(DEMO), solver));
    for _ in 0..6_000 {
        tick(&mut engine);
    }
    let snapshot = engine.snapshot();
    assert!(quench_cold(&snapshot), "{solver}: cold while holding");
    assert_eq!(
        faceplate(&snapshot, "bed2_temperature").mode,
        ControlMode::Auto
    );
    let id = loop_id(&snapshot, "bed2_temperature");
    let held = flow_target(&snapshot);
    let supply = engine.graph.find_node("quench_gas").expect("the supply");
    engine
        .apply(Command::SetSourceTemperature {
            node: supply,
            temperature: Kelvin(420.0 + 273.15),
        })
        .unwrap_or_else(|e| panic!("{solver}: heat the supply: {e}"));
    let command_tick = snapshot.tick;
    tick(&mut engine);
    let carried = engine.snapshot();
    assert!(
        quench_cold(&carried) && faceplate(&carried, "bed2_temperature").mode == ControlMode::Auto,
        "{solver}: the first tick's pass still reads the cold pipe"
    );
    tick(&mut engine);
    let frozen = engine.snapshot();
    assert_eq!(frozen.tick, command_tick + 2);
    let plate = faceplate(&frozen, "bed2_temperature");
    let reading = plate.quench.expect("published");
    assert!(!reading.is_cold(), "{solver}: {reading:?}");
    // The quench valve's dissipation warms the gas a few kelvin past the
    // supply (§64.12), so the reading sits just over 420 °C.
    let quench_c = reading.quench_k.value() - 273.15;
    assert!(
        quench_c > 420.0 && quench_c < 430.0,
        "{solver}: the quench reads the hot supply, {quench_c:.2} °C"
    );
    assert_eq!(plate.mode, ControlMode::Manual, "{solver}: frozen");
    let target = flow_target(&frozen);
    for _ in 0..600 {
        tick(&mut engine);
        assert_eq!(
            flow_target(&engine.snapshot()),
            target,
            "{solver}: the flow target is held while frozen"
        );
    }
    assert!(
        (target - held).abs() < 0.2,
        "{solver}: held near where it stood, {target} against {held}"
    );
    let auto = |engine: &mut Engine| {
        engine.apply(Command::SetControllerMode {
            loop_id: id,
            mode: ControlMode::Auto,
        })
    };
    let err = auto(&mut engine)
        .expect_err("AUTO is refused while the quench is warm")
        .to_string();
    assert!(
        err.contains("bed2_temperature") && err.contains("not colder than"),
        "{solver}: {err}"
    );
    engine
        .apply(Command::SetSourceTemperature {
            node: supply,
            temperature: Kelvin(67.8 + 273.15),
        })
        .unwrap_or_else(|e| panic!("{solver}: cool the supply: {e}"));
    assert!(
        auto(&mut engine).is_err(),
        "{solver}: still warm in its pipe"
    );
    tick(&mut engine);
    auto(&mut engine).unwrap_or_else(|e| panic!("{solver}: AUTO once cold: {e}"));
    for _ in 0..HALF_HOUR / 2 {
        tick(&mut engine);
    }
    let end = bed_c(&engine, "bed2");
    assert!(
        (end - SETPOINT_C).abs() < 0.5,
        "{solver}: back in AUTO, the loop brings the bed back, {end:.3} °C"
    );
}

#[test]
fn slow_a_warm_quench_freezes_the_loop_newton() {
    a_warm_quench_freezes_the_loop("newton");
}

#[test]
fn slow_a_warm_quench_freezes_the_loop_simple() {
    a_warm_quench_freezes_the_loop("simple");
}

/// **Gate 5: the single-loop form.** The bed loop on the quench valve itself,
/// declared direct, holds the bed as the cascade does.
fn a_single_loop_on_the_quench_valve_holds_the_bed(solver: &str) {
    let mut engine = build(&with_solver(&single_loop(DEMO), solver));
    let mut peak = f64::MIN;
    for _ in 0..HALF_HOUR {
        tick(&mut engine);
        assert!(quench_cold(&engine.snapshot()), "{solver}");
        peak = peak.max(bed_c(&engine, "bed2"));
    }
    let end = bed_c(&engine, "bed2");
    assert!(peak < SETPOINT_C + 1.0, "{solver}: peak {peak:.3} °C");
    assert!(
        (end - SETPOINT_C).abs() < 0.05,
        "{solver}: settled, {end:.4} °C"
    );
}

#[test]
fn slow_a_single_loop_on_the_quench_valve_holds_the_bed_newton() {
    a_single_loop_on_the_quench_valve_holds_the_bed("newton");
}

#[test]
fn slow_a_single_loop_on_the_quench_valve_holds_the_bed_simple() {
    a_single_loop_on_the_quench_valve_holds_the_bed("simple");
}

/// **Gate 6: the layouts refused**, each on a substring of its own message.
#[test]
fn a_hydrogen_quench_of_the_wrong_layout_is_refused() {
    // A stream joining the quench line: a second hydrogen supply into the tee.
    let err = refusal(&swap(
        DEMO,
        "[nodes.tee]",
        "[nodes.spare_gas]\ntype = \"source\"\npressure_bar = 160.0\ntemperature_c = 40.0\n\
         composition = { hydrogen = 1.0 }\n\n[[pipes]]\nname = \"spare_line\"\n\
         from = \"spare_gas\"\nto = \"tee\"\nlength_m = 10.0\ndiameter_m = 0.05\n\n[nodes.tee]",
    ));
    assert!(
        err.contains("passes junction 'tee', where another stream joins it"),
        "{err}"
    );

    // A LIQUID quench into the hydrogen pass's mix: a liquid supply joins the
    // loop, which the phase guard refuses before any loop is built.
    let liquid = swap(
        &quench_from_supply(DEMO),
        "temperature_c = 67.8\ncomposition = { hydrogen = 1.0 }",
        "temperature_c = 67.8\ncomposition = { vgo = 1.0 }",
    );
    let err = refusal(&liquid);
    assert!(
        err.contains("are connected, so their streams can mix"),
        "{err}"
    );

    // A GAS quench into the oil between the beds: the hydrogen joins the oil.
    let mut gas_into_oil = swap(
        DEMO,
        "name = \"interbed_line\"\nfrom = \"bed1\"\nto = \"bed2\"",
        "name = \"interbed_line\"\nfrom = \"bed1\"\nto = \"oil_mix\"",
    );
    gas_into_oil = swap(
        &gas_into_oil,
        "[nodes.mix]",
        "[nodes.oil_mix]\ntype = \"junction\"\n\n[[pipes]]\nname = \"oil_mix_line\"\n\
         from = \"oil_mix\"\nto = \"bed2\"\nlength_m = 5.0\ndiameter_m = 0.25\n\n[nodes.mix]",
    );
    gas_into_oil = swap(
        &gas_into_oil,
        "name = \"quench_line\"\nfrom = \"quench_valve\"\nto = \"mix\"",
        "name = \"quench_line\"\nfrom = \"quench_valve\"\nto = \"oil_mix\"",
    );
    let err = refusal(&gas_into_oil);
    assert!(
        err.contains("are connected, so their streams can mix"),
        "{err}"
    );

    // A loop on the FIRST bed over the quench that feeds the second's pass.
    let err = refusal(&swap(
        DEMO,
        "name = \"bed2_temperature\"\nmeasurement = { node = \"bed2\"",
        "name = \"bed2_temperature\"\nmeasurement = { node = \"bed1\"",
    ));
    assert!(
        err.contains("not a quench on that bed") && err.contains("reaches 'pass2', not the bed"),
        "{err}"
    );

    // A separator hotter than the second bed at load: more of it would heat.
    let err = refusal(&swap(
        DEMO,
        "pressure_bar = 150.0\ntemperature_c = 50.0",
        "pressure_bar = 150.0\ntemperature_c = 450.0",
    ));
    assert!(
        err.contains("not colder than the bed's") && err.contains("from 'separator'"),
        "{err}"
    );
}

/// **Gate 7: the faceplate.** The bed loop alone publishes its check, read at
/// load from the separator — a gas vessel, M59.3's new origin — against the
/// second bed.
#[test]
fn the_faceplate_reads_the_separator_at_load() {
    let engine = build(DEMO);
    let snapshot = engine.snapshot();
    let reading = faceplate(&snapshot, "bed2_temperature")
        .quench
        .expect("the bed loop publishes its check from load");
    assert!(
        (reading.quench_k.value() - (50.0 + 273.15)).abs() < 1e-9
            && (reading.joins_k.value() - (395.0 + 273.15)).abs() < 1e-9,
        "at load: the separator against the bed, {reading:?}"
    );
    for name in ["quench_flow", "separator_pressure", "gas_heater_outlet"] {
        assert!(faceplate(&snapshot, name).quench.is_none(), "{name}");
    }
    let wire = serde_json::to_string(&snapshot).expect("a snapshot serializes");
    assert_eq!(wire.matches("\"quench\":").count(), 1);
}
