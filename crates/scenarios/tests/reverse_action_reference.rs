//! Gates for reverse action — a loop whose output RAISES its measurement (M18.1,
//! docs/DESIGN.md §22).
//!
//! **The sign moved onto the loop, and every gate here is about keeping it in
//! one place.** `ControlledValue::error(measurement, setpoint, action)` is the
//! only site that knows which way a loop acts; the load-time seed, the
//! MANUAL→AUTO seed and the tick's update all pass the loop's own action to it,
//! and the snapshot publishes the same action so a reader can rebuild the error.
//! A furnace is the actuator that needs it, so the rest of the file is the
//! furnace joining the actuator side M17 built for the cooler: the range, the
//! command guard, the refusals.
//!
//! The fixtures are the shipped demo with lines replaced, so each case differs
//! from a plant known to load in exactly the line it is about. The demo's own
//! gates are in `temperature_heating_demo.rs`.

use refinery_core::error::SimError;
use refinery_core::graph::{ControlAction, ControlMode, ControlledValue, LoopId, NodeKind};
use refinery_core::snapshot::{Command, ControlSnapshot};
use refinery_core::units::Watt;
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/tank_temperature_heating.toml");

/// The three shipped plants whose loops are DIRECT — every loop written before
/// M18. Their published bytes must carry no `action` key.
const DIRECT_LOOP_PLANTS: [(&str, &str); 3] = [
    (
        "tank_level_control",
        include_str!("../../../scenarios/tank_level_control.toml"),
    ),
    (
        "vessel_pressure_control",
        include_str!("../../../scenarios/vessel_pressure_control.toml"),
    ),
    (
        "tank_temperature_control",
        include_str!("../../../scenarios/tank_temperature_control.toml"),
    ),
];

// ------------------------------------------------------------------ helpers

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

/// The demo with one exact line replaced (or removed, with `""`). Asserts the
/// line exists, so a case cannot pass because its edit silently did nothing.
fn demo_with(line: &str, with: &str) -> String {
    let needle = format!("\n{line}\n");
    assert!(DEMO.contains(&needle), "the demo has no line {line:?}");
    let replacement = if with.is_empty() {
        "\n".to_string()
    } else {
        format!("\n{with}\n")
    };
    DEMO.replacen(&needle, &replacement, 1)
}

fn run(engine: &mut Engine, ticks: u64) {
    for t in 1..=ticks {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
    }
}

fn control(engine: &Engine) -> ControlSnapshot {
    engine.snapshot().controls[0].clone()
}

fn heater_duty_w(engine: &Engine) -> f64 {
    let id = engine.graph.find_node("heater").expect("heater");
    match engine.graph.node(id).kind {
        NodeKind::Furnace { duty } => duty.value(),
        ref other => panic!("heater is a furnace, not {other:?}"),
    }
}

fn kelvin(value: ControlledValue) -> f64 {
    match value {
        ControlledValue::Temperature { k } => k.value(),
        other => panic!("a temperature loop, not {other:?}"),
    }
}

// ------------------------------------------------------ gate 1: the sign, by hand

/// **Gate 1. The first output of a reverse PROPORTIONAL loop is
/// `K·(setpoint − measurement)` by hand, and a reader rebuilds it from the
/// snapshot.**
///
/// Proportional, not the demo's PI loop, for M17 gate 4's reason: a PI loop's
/// first output is its seeded `initial_output` whatever the sign, so a gate
/// built on it cannot see the sign. The tank is at 40 °C against a 50 °C
/// setpoint at 0.05 per K, so the answer is 0.5 — mid-range — and a loop that
/// ignored its action would compute `−0.5` and clamp the furnace to zero.
///
/// The reconstruction half is the reason the action is on the snapshot at all
/// (§22 fork 1): the published measurement, setpoint and action must give back
/// the published output, with the error taken the way `action` says.
#[test]
fn a_reverse_loop_fires_harder_the_colder_its_tank_is() {
    let src = demo_with(r#"algorithm = "pi""#, r#"algorithm = "p""#);
    let src = src.replace("setpoint_c = 60.0", "setpoint_c = 50.0");
    let src = src.replace("gain_per_k = 0.1", "gain_per_k = 0.05");
    let src = src
        .replace("integral_time_s = 600.0\n", "")
        .replace("initial_output = 0.25\n", "");
    let mut engine = engine_from(&src);
    engine.tick().expect("tick 1");

    let expected: f64 = 0.05 * ((50.0 + 273.15) - (40.0 + 273.15));
    assert!(
        (expected - 0.5).abs() < 1.0e-12,
        "the fixture's own arithmetic must land interior, and computed {expected}"
    );
    let c = control(&engine);
    assert_eq!(c.action, ControlAction::Reverse);
    assert!(
        (c.output - expected).abs() < 1.0e-12,
        "a reverse proportional loop's first output is K·(setpoint − measurement) = \
         {expected}; it produced {}. Zero means the action was ignored",
        c.output
    );
    let rebuilt = 0.05
        * ControlledValue::error(
            c.measurement
                .expect("a stored quantity is measured from load"),
            c.setpoint,
            c.action,
        );
    assert_eq!(
        rebuilt, c.output,
        "the snapshot's three values must rebuild the output the controller produced"
    );
    let unsigned = 0.05
        * (kelvin(
            c.measurement
                .expect("a stored quantity is measured from load"),
        ) - kelvin(c.setpoint));
    assert!(
        unsigned < 0.0,
        "and a reader who ignored the published action would get {unsigned}, which \
         is why it is published"
    );
    assert!(
        (heater_duty_w(&engine) - expected * 2.0e6).abs() <= 1.0e-9 * 2.0e6,
        "the furnace is written u · max_duty"
    );
}

// ----------------------------------------------------- gate 5: MANUAL→AUTO

/// **Gate 5. MANUAL→AUTO is bumpless on a reverse loop**, through
/// `Engine::apply`, with the tank far below setpoint.
///
/// The transfer seeds the memory against the error standing at the moment of
/// transfer. With the tank ~19 K cold the reverse error is about +19 K; a seed
/// taken against the direct error would hold `b ≈ 0.75 + 1.9`, and the first AUTO
/// tick would slam the furnace to full.
#[test]
fn a_furnace_transfers_from_manual_to_auto_without_a_step() {
    let mut engine = engine_from(&demo_with(r#"mode = "auto""#, r#"mode = "manual""#));
    let heater = engine.graph.find_node("heater").expect("heater");
    engine
        .apply(Command::SetFurnaceDuty {
            node: heater,
            duty: Watt(1.5e6),
        })
        .expect("in MANUAL, within range, the command drives the furnace");
    run(&mut engine, 50);
    let cold = kelvin(
        control(&engine)
            .measurement
            .expect("a stored quantity is measured from load"),
    ) - 273.15;
    assert!(
        60.0 - cold > 15.0,
        "the gate needs a large error at transfer; the tank reads {cold} °C"
    );
    engine
        .apply(Command::SetControllerMode {
            loop_id: LoopId(0),
            mode: ControlMode::Auto,
        })
        .expect("auto");
    assert_eq!(
        control(&engine).output,
        0.75,
        "the faceplate reports the held position"
    );
    engine.tick().expect("first tick in AUTO");
    let duty = heater_duty_w(&engine);
    assert!(
        (duty - 1.5e6).abs() <= 1.0e-12 * 2.0e6,
        "the first AUTO tick must leave the furnace where MANUAL left it, to the \
         back-calculation's few ULP of its 2 MW range — it moved to {duty} W"
    );
}

// ----------------------------------------------------- gate 6: the wire form

/// **Gate 6's direct half. A direct loop publishes NO `action` key**, on the
/// bytes of all three shipped direct-loop plants — which is what keeps them
/// byte-identical — and a snapshot without the key reads back as direct.
///
/// Asserted on the serialized text because the Rust field is `Direct` either
/// way; only the bytes can tell a skipped key from a written one.
#[test]
fn a_direct_loop_publishes_no_action_and_reads_back_as_direct() {
    for (name, src) in DIRECT_LOOP_PLANTS {
        let engine = engine_from(src);
        assert_eq!(engine.snapshot().controls[0].action, ControlAction::Direct);
        let json = serde_json::to_string(&engine.snapshot().controls[0]).expect("serializes");
        assert!(
            !json.contains("action"),
            "{name}: a direct loop must publish no `action` key, and published {json}"
        );
        let back: ControlSnapshot = serde_json::from_str(&json).expect("round trips");
        assert_eq!(
            back.action,
            ControlAction::Direct,
            "{name}: absent reads as direct"
        );
    }
    assert_eq!(
        serde_json::to_string(&ControlAction::Reverse).expect("serializes"),
        r#""reverse""#
    );
}

// --------------------------------------------------- gate 7: the refusal sweep

/// **Gate 7. Every refusal M18 adds or rewords, each asserting a substring
/// distinctive to its OWN message** (M16.2: a disjunction passes on its wrong
/// half). The demo is built first, so no case passes on a broken fixture.
#[test]
fn every_refused_direction_is_refused_for_its_own_reason() {
    engine_from(DEMO);
    let cooler = DEMO.replace(r#"type = "furnace""#, r#"type = "cooler""#);
    engine_from(&cooler.replace("action = \"reverse\"\n", ""));

    let cases: Vec<(&str, String, &str)> = vec![
        (
            "a furnace loop that declares no action",
            demo_with(r#"action = "reverse""#, ""),
            "declares no `action`",
        ),
        (
            "a furnace loop declared direct",
            demo_with(r#"action = "reverse""#, r#"action = "direct""#),
            "more firing raises a temperature",
        ),
        (
            "a cooler loop declared reverse",
            cooler.clone(),
            "more cooling lowers a temperature",
        ),
        (
            "an unknown action",
            demo_with(r#"action = "reverse""#, r#"action = "inverse""#),
            "unknown action 'inverse'",
        ),
        (
            "a valve declared reverse (a fill valve's sign is topology)",
            format!(
                "{}\n{}",
                DEMO.split("[[controls]]").next().expect("demo has a loop"),
                "[[controls]]\nname = \"tank_level\"\nmeasurement = { node = \"hold_tank\", \
                 variable = \"level\" }\nactuator = \"drain_valve\"\nalgorithm = \"p\"\nmode \
                 = \"auto\"\naction = \"reverse\"\nsetpoint_m = 5.0\ngain_per_m = 0.2\n"
            ),
            "docs/DEFERRED.md E8",
        ),
        (
            "a furnace loop with no `max_duty_mw`",
            demo_with("max_duty_mw = 2.0", ""),
            "declares no `max_duty_mw`",
        ),
        (
            "a declared furnace duty above the loop's range",
            demo_with("duty_mw = 0.5", "duty_mw = 2.5"),
            "above the loop's `max_duty_mw",
        ),
        (
            "a furnace actuating a level",
            format!(
                "{}\n{}",
                DEMO.split("[[controls]]").next().expect("demo has a loop"),
                "[[controls]]\nname = \"tank_level\"\nmeasurement = { node = \"hold_tank\", \
                 variable = \"level\" }\nactuator = \"heater\"\nalgorithm = \"p\"\nmode = \
                 \"auto\"\nsetpoint_m = 5.0\ngain_per_m = 0.2\n"
            ),
            "which is not a valve",
        ),
        (
            "a negative gain (reverse action written as a sign)",
            demo_with("gain_per_k = 0.1", "gain_per_k = -0.1"),
            "reverse action written as a sign",
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

// ------------------------------------------------- gate 8: the command guard

/// **Gate 8. `SetFurnaceDuty` on a loop's furnace: refused in AUTO, accepted in
/// MANUAL, refused above the loop's range in both** — the two guards
/// `SetCoolerDuty` got in M17, which the furnace command lacked.
///
/// The AUTO half first shows what the refusal protects — a duty written around
/// the command (straight onto the graph, which is what an unguarded command
/// would do) is gone one tick later.
#[test]
fn a_loop_owned_furnace_refuses_a_write_the_loop_would_overwrite() {
    let mut engine = engine_from(DEMO);
    let heater = engine.graph.find_node("heater").expect("heater");
    run(&mut engine, 5);

    engine.graph.node_mut(heater).kind = NodeKind::Furnace { duty: Watt(1.5e6) };
    engine.tick().expect("tick");
    let after = heater_duty_w(&engine);
    assert!(
        (after - 1.5e6).abs() > 1.0e5,
        "a duty written under a loop in AUTO is overwritten at the top of the next \
         tick — it read {after} W — which is the silent loss the guard exists for"
    );

    let refused = engine
        .apply(Command::SetFurnaceDuty {
            node: heater,
            duty: Watt(1.5e6),
        })
        .expect_err("a furnace under a loop in AUTO must refuse a manual duty");
    let text = refused.to_string();
    assert!(
        text.contains("tank_temperature") && text.contains("AUTO"),
        "the refusal must name the owning loop and why: {text}"
    );
    assert!(matches!(refused, SimError::InvalidCommand(_)));

    engine
        .apply(Command::SetControllerMode {
            loop_id: LoopId(0),
            mode: ControlMode::Manual,
        })
        .expect("manual");
    let above = engine
        .apply(Command::SetFurnaceDuty {
            node: heater,
            duty: Watt(2.5e6),
        })
        .expect_err("in MANUAL a duty above the loop's authority is still refused");
    assert!(
        above.to_string().contains("furnace duty") && above.to_string().contains("authority"),
        "the MANUAL refusal must name the furnace and the loop's range: {above}"
    );
    engine
        .apply(Command::SetFurnaceDuty {
            node: heater,
            duty: Watt(1.5e6),
        })
        .expect("in MANUAL, within range, the command drives the furnace");
    engine.tick().expect("tick");
    assert_eq!(heater_duty_w(&engine), 1.5e6, "the write survives");
    assert_eq!(
        control(&engine).output,
        0.75,
        "and the faceplate tracks it as a fraction of the loop's range, 1.5 / 2 MW"
    );
}

/// A furnace NO loop owns is unguarded, as before M18: the guards are about the
/// loop, not about furnaces. Without this control, a guard that refused every
/// `SetFurnaceDuty` would pass gate 8.
#[test]
fn a_furnace_no_loop_owns_takes_any_duty() {
    let src = DEMO.split("[[controls]]").next().expect("demo has a loop");
    let mut engine = engine_from(src);
    let heater = engine.graph.find_node("heater").expect("heater");
    engine
        .apply(Command::SetFurnaceDuty {
            node: heater,
            duty: Watt(5.0e6),
        })
        .expect("an unowned furnace takes a duty above any loop's range");
    assert_eq!(heater_duty_w(&engine), 5.0e6);
}
