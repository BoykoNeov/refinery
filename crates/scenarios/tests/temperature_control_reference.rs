//! Gates for the third controlled variable — a holdup's temperature (M17.1,
//! docs/DESIGN.md §21).
//!
//! **Two things are new here and only one of them is the variable.** A
//! temperature is measured the way a level and a pressure already are: stored on
//! the graph, real at tick 0, one match arm in `PlantGraph::measure`. The new
//! machinery is the ACTUATOR. Every loop before this one wrote a valve's opening;
//! this one writes a cooler's duty, a quantity in watts mapped from the loop's
//! `[0, 1]` output through the loop's own `max_duty_mw`. So most of this file is
//! about the actuator side — the units of the gain, the range, the command guard,
//! the transfer — and the measurement side is gated on the demo and on one vessel.
//!
//! The fixtures are declared here rather than shipped: a plant built to expose
//! one behaviour is a fixture, and the files in `scenarios/` are the regression
//! anchor. The wired demo lives in `temperature_control_demo.rs`.

use refinery_core::error::SimError;
use refinery_core::graph::{ControlAction, ControlMode, ControlledValue, LoopId, NodeKind};
use refinery_core::snapshot::Command;
use refinery_core::units::{Kelvin, Meter, Pascal, Watt};
use refinery_core::Engine;

// ------------------------------------------------------------------ fixtures

/// The demo's plant without its loop: hot water → `chiller` → `hold_tank` →
/// drain → sink. `CHILLER_TYPE` and `CHILLER_DUTY` are substituted so the same
/// plant can carry a furnace or an out-of-range duty for the refusal sweep.
const LIQUID_PLANT: &str = r#"
[meta]
name = "temperature_fixture"

[simulation]
dt = 1.0

[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"

[nodes.hot_feed]
type = "source"
pressure_bar = 1.6
temperature_c = 80.0

[nodes.chiller]
type = "CHILLER_TYPE"
duty_mw = CHILLER_DUTY

[nodes.hold_tank]
type = "tank"
area_m2 = 3.0
height_m = 10.0
initial_level_m = 4.968
temperature_c = 80.0

[nodes.drain_valve]
type = "valve"
kv = 150.0
opening = 0.5

[nodes.rundown]
type = "sink"
pressure_bar = 1.01325

[[pipes]]
name = "feed_line"
from = "hot_feed"
to = "chiller"
length_m = 20.0
diameter_m = 0.10

[[pipes]]
name = "cooled_line"
from = "chiller"
to = "hold_tank"
length_m = 10.0
diameter_m = 0.10

[[pipes]]
name = "drain_line"
from = "hold_tank"
to = "drain_valve"
length_m = 10.0
diameter_m = 0.15

[[pipes]]
name = "rundown_line"
from = "drain_valve"
to = "rundown"
length_m = 10.0
diameter_m = 0.15
"#;

fn liquid(chiller_type: &str, duty_mw: &str) -> String {
    LIQUID_PLANT
        .replace("CHILLER_TYPE", chiller_type)
        .replace("CHILLER_DUTY", duty_mw)
}

/// A `[[controls]]` table whose lines are given whole, so each refusal case
/// below differs from the accepted loop in exactly the line it is about.
fn control(lines: &[&str]) -> String {
    format!(
        "\n[[controls]]\nname = \"tank_temperature\"\n{}\n",
        lines.join("\n")
    )
}

/// The demo's loop, line for line.
const PI_LINES: [&str; 9] = [
    r#"measurement = { node = "hold_tank", variable = "temperature" }"#,
    r#"actuator = "chiller""#,
    r#"algorithm = "pi""#,
    r#"mode = "auto""#,
    "setpoint_c = 60.0",
    "gain_per_k = 0.1",
    "integral_time_s = 600.0",
    "initial_output = 0.25",
    "max_duty_mw = 2.0",
];

fn pi_plant() -> String {
    format!("{}{}", liquid("cooler", "0.5"), control(&PI_LINES))
}

/// The accepted loop with one line replaced (matched by its key) or, if `with`
/// is `None`, removed.
fn pi_with(key: &str, with: Option<&str>) -> Vec<String> {
    let mut lines: Vec<String> = Vec::new();
    for line in PI_LINES {
        if line.starts_with(key) {
            if let Some(w) = with {
                lines.push(w.to_string());
            }
        } else {
            lines.push(line.to_string());
        }
    }
    lines
}

/// A gas receiver fed through a cooler from a hot header, vented to flare
/// through a fixed valve — the vessel arm of fork 1, which the demo does not
/// exercise. Measured: it holds 120 °C at u = 0.3126 of 0.1 MW, where its manual
/// twin settles at 136.49 °C, and its startup drives the cooler to full duty on
/// the way (compression heats the receiver as it fills), so it reaches the
/// anti-windup arm with no command.
const GAS_PLANT: &str = r#"
[meta]
name = "vessel_temperature_fixture"

[simulation]
dt = 0.1

[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"

[[components]]
name = "fuel_gas"
phase = "gas"
tb_c = -161.5
molar_mass_kg_per_mol = 0.016043
cp_j_per_kg_k = 2220.0

[nodes.header]
type = "source"
pressure_bar = 30.0
temperature_c = 150.0

[nodes.gas_cooler]
type = "cooler"
duty_mw = 0.02

[nodes.receiver]
type = "vessel"
volume_m3 = 2.0
pressure_bar = 12.0
temperature_c = 150.0

[nodes.vent_valve]
type = "valve"
kv = 12.0
opening = 0.30
x_t = 0.72

[nodes.flare]
type = "sink"
pressure_bar = 1.1
temperature_c = 20.0

[[pipes]]
name = "make_up"
from = "header"
to = "gas_cooler"
length_m = 10.0
diameter_m = 0.021

[[pipes]]
name = "cooled_make_up"
from = "gas_cooler"
to = "receiver"
length_m = 10.0
diameter_m = 0.021

[[pipes]]
name = "vent_line"
from = "receiver"
to = "vent_valve"
length_m = 10.0
diameter_m = 0.05

[[pipes]]
name = "flare_line"
from = "vent_valve"
to = "flare"
length_m = 30.0
diameter_m = 0.10
"#;

const GAS_LOOP: &str = r#"
[[controls]]
name = "receiver_temperature"
measurement = { node = "receiver", variable = "temperature" }
actuator = "gas_cooler"
algorithm = "pi"
mode = "MODE"
setpoint_c = 120.0
gain_per_k = 0.05
integral_time_s = 30.0
initial_output = 0.2
max_duty_mw = 0.1
"#;

// ------------------------------------------------------------------- helpers

fn engine_from(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("fixture parses");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("fixture builds: {e}"))
}

/// The text of the refusal a scenario earns, whichever stage produced it.
fn refusal(src: &str) -> String {
    match refinery_scenarios::load_str(src) {
        Err(e) => e.to_string(),
        Ok(file) => match refinery_scenarios::build_engine(&file) {
            Ok(_) => panic!("this plant should not have loaded"),
            Err(e) => e.to_string(),
        },
    }
}

fn run(engine: &mut Engine, ticks: u64) {
    for t in 1..=ticks {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
    }
}

fn output(engine: &Engine) -> f64 {
    engine.snapshot().controls[0].output
}

fn measured_k(engine: &Engine) -> f64 {
    match engine.snapshot().controls[0].measurement {
        ControlledValue::Temperature { k } => k.value(),
        other => panic!("this loop measures a temperature, not {other:?}"),
    }
}

fn cooler_duty_w(engine: &Engine, name: &str) -> f64 {
    let id = engine.graph.find_node(name).expect("the cooler exists");
    match engine.graph.node(id).kind {
        NodeKind::Cooler { duty } => duty.value(),
        ref other => panic!("{name} is a cooler, not {other:?}"),
    }
}

// ----------------------------------------------------- gate 4: the units trap

/// **Gate 4. The first output of a PROPORTIONAL loop is `K·e` by hand, with the
/// setpoint in kelvin and the gain per kelvin untouched.**
///
/// A proportional loop and not the demo's PI loop, deliberately: a PI loop's
/// memory is back-calculated at load, so its first output is `initial_output`
/// whatever `K` is, and both unit mutations would pass a gate built on it.
///
/// The fixture's numbers land the answer mid-range — 80 °C against a 70 °C
/// setpoint at 0.05 per K is 0.5 — so each mutation it exists for saturates:
/// adding the °C offset to the gain makes `K = 273.2`, and leaving it off the
/// setpoint makes `e = 353.15 − 70 = 283`; either clamps the cooler to 1. The duty
/// the cooler is left at is asserted as well, because the output is a fraction
/// and the actuator is not: `duty = u · max_duty`.
#[test]
fn the_setpoint_takes_the_offset_and_the_gain_takes_nothing() {
    let lines = [
        r#"measurement = { node = "hold_tank", variable = "temperature" }"#,
        r#"actuator = "chiller""#,
        r#"algorithm = "p""#,
        r#"mode = "auto""#,
        "setpoint_c = 70.0",
        "gain_per_k = 0.05",
        "max_duty_mw = 2.0",
    ];
    let mut engine = engine_from(&format!("{}{}", liquid("cooler", "0.5"), control(&lines)));
    engine.tick().expect("tick 1");

    let error_k: f64 = (80.0 + 273.15) - (70.0 + 273.15);
    let expected = 0.05 * error_k;
    assert!(
        (expected - 0.5).abs() < 1.0e-12,
        "the fixture's own arithmetic must land interior, and computed {expected}"
    );
    let u = output(&engine);
    assert!(
        (u - expected).abs() < 1.0e-12,
        "a proportional loop's first output is K·e: 0.05 /K × {error_k} K = {expected}, \
         and the loop produced {u}. An output of 1 means the offset went to the GAIN \
         (0.05 → 273.2 per K) or missed the SETPOINT (e = 283 K)"
    );
    let duty = cooler_duty_w(&engine, "chiller");
    assert!(
        (duty - expected * 2.0e6).abs() <= 1.0e-9 * 2.0e6,
        "the cooler is written u · max_duty = {} W, and holds {duty} W",
        expected * 2.0e6
    );
}

// ------------------------------------------------------ gate 5: the wire form

/// **Gate 5. The tag on the bytes.** A Rust match on `ControlledValue::Temperature`
/// passes under any serde tag, and a new variant carrying an old variant's tag is
/// exactly how M10.1's sharpest mutation escaped every other test.
#[test]
fn a_temperature_travels_as_a_tagged_kelvin_number() {
    let json = serde_json::to_string(&ControlledValue::Temperature { k: Kelvin(333.15) })
        .expect("serializes");
    assert_eq!(json, r#"{"variable":"temperature","k":333.15}"#);
    let back: ControlledValue = serde_json::from_str(&json).expect("round trips");
    assert_eq!(back, ControlledValue::Temperature { k: Kelvin(333.15) });
}

// --------------------------------------------------- gate 6: the refusal sweep

/// **Gate 6. Every refusal, each asserting a substring distinctive to its OWN
/// message** — M16.2 found two boundary refusals passing on the wrong half of a
/// disjunction, so no case here accepts one of two phrasings.
///
/// Each case differs from an accepted plant in one place, and the accepted plant
/// is built first, so a refusal cannot be passing because the fixture was broken
/// some other way.
#[test]
fn every_refused_temperature_loop_is_refused_for_its_own_reason() {
    engine_from(&pi_plant());
    engine_from(&format!("{GAS_PLANT}{}", GAS_LOOP.replace("MODE", "auto")));

    let refs = |v: &Vec<String>| v.iter().map(String::as_str).collect::<Vec<_>>().join("\n");
    let loop_of = |lines: Vec<String>| {
        format!(
            "\n[[controls]]\nname = \"tank_temperature\"\n{}\n",
            refs(&lines)
        )
    };
    let cases: Vec<(&str, String, &str)> = vec![
        (
            "a zero-volume node's temperature (the furnace-outlet measurement)",
            format!(
                "{}{}",
                liquid("cooler", "0.5"),
                loop_of(pi_with(
                    "measurement",
                    Some(r#"measurement = { node = "chiller", variable = "temperature" }"#)
                ))
            ),
            "stated rule for what a loop measures at tick 0",
        ),
        (
            "a boundary's temperature",
            format!(
                "{}{}",
                liquid("cooler", "0.5"),
                loop_of(pi_with(
                    "measurement",
                    Some(r#"measurement = { node = "hot_feed", variable = "temperature" }"#)
                ))
            ),
            "is a boundary",
        ),
        (
            "a furnace actuating a temperature with no declared action",
            format!(
                "{}{}",
                liquid("furnace", "0.5"),
                loop_of(PI_LINES.iter().map(|s| s.to_string()).collect())
            ),
            "declares no `action`",
        ),
        (
            "a valve actuating a temperature",
            format!(
                "{}{}",
                liquid("cooler", "0.5"),
                loop_of(pi_with("actuator", Some(r#"actuator = "drain_valve""#)))
            ),
            "no coolant stream",
        ),
        (
            "a cooler actuating a level",
            format!(
                "{}{}",
                liquid("cooler", "0.5"),
                control(&[
                    r#"measurement = { node = "hold_tank", variable = "level" }"#,
                    r#"actuator = "chiller""#,
                    r#"algorithm = "p""#,
                    r#"mode = "auto""#,
                    "setpoint_m = 5.0",
                    "gain_per_m = 0.2",
                    "max_duty_mw = 2.0",
                ])
            ),
            "a cut's density is a constant",
        ),
        (
            "a cooler actuating a pressure (a scope refusal, not a physics one)",
            format!(
                "{GAS_PLANT}{}",
                control(&[
                    r#"measurement = { node = "receiver", variable = "pressure" }"#,
                    r#"actuator = "gas_cooler""#,
                    r#"algorithm = "p""#,
                    r#"mode = "auto""#,
                    "setpoint_bar = 12.0",
                    "gain_per_bar = 0.1",
                    "max_duty_mw = 0.1",
                ])
            ),
            "wearing one loop's name",
        ),
        (
            "`max_duty_mw` missing on a cooler",
            format!(
                "{}{}",
                liquid("cooler", "0.5"),
                loop_of(pi_with("max_duty_mw", None))
            ),
            "declares no `max_duty_mw`",
        ),
        (
            "`max_duty_mw` on a valve",
            format!(
                "{}{}",
                liquid("cooler", "0.5"),
                control(&[
                    r#"measurement = { node = "hold_tank", variable = "level" }"#,
                    r#"actuator = "drain_valve""#,
                    r#"algorithm = "p""#,
                    r#"mode = "auto""#,
                    "setpoint_m = 5.0",
                    "gain_per_m = 0.2",
                    "max_duty_mw = 2.0",
                ])
            ),
            "a DUTY actuator's range",
        ),
        (
            "a non-positive `max_duty_mw`",
            format!(
                "{}{}",
                liquid("cooler", "0.0"),
                loop_of(pi_with("max_duty_mw", Some("max_duty_mw = 0.0")))
            ),
            "finite duty above zero",
        ),
        (
            "a declared cooler duty above the loop's range",
            format!(
                "{}{}",
                liquid("cooler", "2.5"),
                loop_of(pi_with("max_duty_mw", Some("max_duty_mw = 2.0")))
            ),
            "above the loop's `max_duty_mw",
        ),
        (
            "`setpoint_c` on a level loop",
            format!(
                "{}{}",
                liquid("cooler", "0.5"),
                control(&[
                    r#"measurement = { node = "hold_tank", variable = "level" }"#,
                    r#"actuator = "drain_valve""#,
                    r#"algorithm = "p""#,
                    r#"mode = "auto""#,
                    "setpoint_m = 5.0",
                    "gain_per_m = 0.2",
                    "setpoint_c = 60.0",
                ])
            ),
            "declares `setpoint_c`, which is the Temperature loop's key",
        ),
        (
            "`setpoint_m` on a temperature loop",
            format!(
                "{}{}",
                liquid("cooler", "0.5"),
                loop_of(pi_with("setpoint_c", Some("setpoint_m = 5.0")))
            ),
            "declares `setpoint_m`, which is the Level loop's key",
        ),
        (
            "`gain_per_bar` on a temperature loop",
            format!(
                "{}{}",
                liquid("cooler", "0.5"),
                loop_of(pi_with("gain_per_k", Some("gain_per_bar = 0.1")))
            ),
            "declares `gain_per_bar`, which is the Pressure loop's key",
        ),
        (
            "a setpoint below absolute zero",
            format!(
                "{}{}",
                liquid("cooler", "0.5"),
                loop_of(pi_with("setpoint_c", Some("setpoint_c = -300.0")))
            ),
            "above absolute zero",
        ),
    ];
    for (what, src, needle) in cases {
        let text = refusal(&src);
        assert!(
            text.contains(needle),
            "refusing {what}: the message must contain {needle:?}, and said: {text}"
        );
    }
}

// ------------------------------------------------- gate 7: the command guard

/// **Gate 7. `SetCoolerDuty` on a loop's cooler: refused in AUTO, accepted in
/// MANUAL, refused above the loop's range in both.**
///
/// The AUTO half first shows what the refusal protects — a duty written around
/// the command (straight onto the graph, which is what an unguarded command
/// would do) is gone one tick later — or the gate would prove only that an error
/// is returned.
#[test]
fn a_loop_owned_cooler_refuses_a_write_the_loop_would_overwrite() {
    let mut engine = engine_from(&pi_plant());
    let chiller = engine.graph.find_node("chiller").expect("chiller");
    run(&mut engine, 5);

    // What an unguarded write would have done.
    engine.graph.node_mut(chiller).kind = NodeKind::Cooler { duty: Watt(1.5e6) };
    engine.tick().expect("tick");
    let after = cooler_duty_w(&engine, "chiller");
    assert!(
        (after - 1.5e6).abs() > 1.0e5,
        "a duty written under a loop in AUTO is overwritten at the top of the next \
         tick — it read {after} W — which is the silent loss the guard exists for"
    );

    let refused = engine
        .apply(Command::SetCoolerDuty {
            node: chiller,
            duty: Watt(1.5e6),
        })
        .expect_err("a cooler under a loop in AUTO must refuse a manual duty");
    let text = refused.to_string();
    assert!(
        text.contains("tank_temperature") && text.contains("AUTO"),
        "the refusal must name the owning loop and why: {text}"
    );
    assert!(matches!(refused, SimError::InvalidCommand(_)));
    assert!(
        engine
            .apply(Command::SetCoolerDuty {
                node: chiller,
                duty: Watt(2.5e6),
            })
            .is_err(),
        "above the range is refused in AUTO too"
    );

    engine
        .apply(Command::SetControllerMode {
            loop_id: LoopId(0),
            mode: ControlMode::Manual,
        })
        .expect("manual");
    let above = engine
        .apply(Command::SetCoolerDuty {
            node: chiller,
            duty: Watt(2.5e6),
        })
        .expect_err("in MANUAL a duty above the loop's authority is still refused");
    assert!(
        above.to_string().contains("authority"),
        "the MANUAL refusal must name the loop's range: {above}"
    );
    engine
        .apply(Command::SetCoolerDuty {
            node: chiller,
            duty: Watt(1.5e6),
        })
        .expect("in MANUAL, within range, the command drives the cooler");
    engine.tick().expect("tick");
    assert_eq!(
        cooler_duty_w(&engine, "chiller"),
        1.5e6,
        "the write survives"
    );
    assert_eq!(
        output(&engine),
        0.75,
        "and the faceplate tracks it as a fraction of the loop's range, 1.5 / 2 MW"
    );
}

/// **The MANUAL→AUTO transfer is bumpless on a duty actuator.**
///
/// It catches pass 3 writing `u` watts (§21 mutation 7). It does NOT catch
/// MANUAL tracking the raw duty (mutation 8), and cannot: the transfer seeds from
/// the position owner fresh and never reads what MANUAL tracked. That mutation is
/// gate 7's and the demo's MANUAL twin's.
///
/// The seed and the tick read one function, so the transfer's position IS the
/// next tick's position; the only arithmetic between them is the back-calculated
/// memory, `b = u − K·e` then `u = K·e + b`, which is a few ULP.
#[test]
fn a_cooler_transfers_from_manual_to_auto_without_a_step() {
    let mut engine = engine_from(&pi_plant());
    let chiller = engine.graph.find_node("chiller").expect("chiller");
    engine
        .apply(Command::SetControllerMode {
            loop_id: LoopId(0),
            mode: ControlMode::Manual,
        })
        .expect("manual");
    engine
        .apply(Command::SetCoolerDuty {
            node: chiller,
            duty: Watt(1.5e6),
        })
        .expect("manual duty");
    run(&mut engine, 50);
    engine
        .apply(Command::SetControllerMode {
            loop_id: LoopId(0),
            mode: ControlMode::Auto,
        })
        .expect("auto");
    assert_eq!(
        output(&engine),
        0.75,
        "the faceplate reports the held position"
    );
    engine.tick().expect("first tick in AUTO");
    let duty = cooler_duty_w(&engine, "chiller");
    assert!(
        (duty - 1.5e6).abs() <= 1.0e-12 * 2.0e6,
        "the first AUTO tick must leave the cooler where MANUAL left it, to the \
         back-calculation's few ULP of its 2 MW range — it moved to {duty} W"
    );
}

// --------------------------------------------- gate 8: the error's soundness

/// **Gate 8. `ControlledValue::error` refuses to difference two variables — every
/// mismatched pair of the three, under BOTH directions of action.** With two
/// variants there were two pairs; the third variant adds four, and a backstop
/// tested on the pairs that existed before it is a backstop for the variants that
/// existed before it. M18 gave `error` a second arm (docs/DESIGN.md §22), and the
/// same argument makes the backstop owed on that arm too.
#[test]
fn every_mismatched_pair_of_variables_differences_to_nan() {
    let values = [
        ControlledValue::Level { m: Meter(4.0) },
        ControlledValue::Pressure { pa: Pascal(5.0e5) },
        ControlledValue::Temperature { k: Kelvin(333.15) },
    ];
    for action in [ControlAction::Direct, ControlAction::Reverse] {
        for a in values {
            for b in values {
                let e = ControlledValue::error(a, b, action);
                if a.variable() == b.variable() {
                    assert_eq!(e, 0.0, "{a:?} against itself, {action:?}");
                } else {
                    assert!(
                        e.is_nan(),
                        "{a:?} − {b:?} must be NaN under {action:?}, and gave {e}"
                    );
                }
            }
        }
    }
}

// ---------------------------------------------------- fork 1: the vessel arm

/// **A vessel's temperature is stored too** — fork 1's vessel arm, argued rather
/// than inherited, on a plant the demo does not cover. The loop measures the
/// declared temperature at tick 0 while the snapshot's own is NaN, holds its
/// setpoint, and its manual twin does not.
#[test]
fn a_vessel_temperature_is_measured_from_load_and_held() {
    let mut engine = engine_from(&format!("{GAS_PLANT}{}", GAS_LOOP.replace("MODE", "auto")));
    assert_eq!(
        measured_k(&engine),
        150.0 + 273.15,
        "stored, exact from load"
    );
    let id = engine.graph.find_node("receiver").expect("receiver");
    let snap = engine.snapshot();
    let node = snap.nodes.iter().find(|n| n.id == id).expect("in snapshot");
    assert!(
        node.temperature_k.is_nan(),
        "the tick-resolved one is not, yet"
    );

    let mut peak = 0.0_f64;
    for t in 1..=5_000_u64 {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        peak = peak.max(output(&engine));
    }
    assert_eq!(
        peak, 1.0,
        "the receiver's startup drives its cooler to full duty — this fixture's \
         wired exercise of the anti-windup arm"
    );
    let held = measured_k(&engine) - 273.15;
    assert!(
        (held - 120.0).abs() < 1.0e-3,
        "held at 120 °C; reads {held}"
    );

    let mut manual = engine_from(&format!(
        "{GAS_PLANT}{}",
        GAS_LOOP.replace("MODE", "manual")
    ));
    run(&mut manual, 5_000);
    let parked = measured_k(&manual) - 273.15;
    assert!(
        (parked - 120.0).abs() > 10.0,
        "the parked loop settles well away from 120 °C (measured 136.49); reads {parked}"
    );
}
