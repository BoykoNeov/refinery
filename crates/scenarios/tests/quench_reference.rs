//! M57.4: quench between two catalyst beds (docs/DESIGN.md §62.11).
//!
//! A loop holds the second bed of `scenarios/hydrocracker_quench.toml` at 380 °C
//! by mixing the reactor's own cooled product into the oil between the beds — a
//! cascade, the bed loop setting the target of a flow loop on the quench valve.
//! More quench cools the bed only while the quench is colder than the oil it
//! joins, which is a temperature, not a layout; so the loader checks the layout
//! (`quench_layout`) and the engine checks the temperatures at load and at the
//! top of every tick, switching a bed loop in AUTO whose quench is not cold to
//! MANUAL, to be put back by hand. The gates:
//!
//! 1. the loop catches the runaway the plant is born heading for;
//! 2. in MANUAL the backup trip catches it instead, by cutting the preheater;
//!    and with no trip either the bed runs away and its drum warms until the
//!    quench is no longer cold, when AUTO is refused;
//! 3. the freeze, on a quench drawn from a supply whose temperature a command
//!    sets: MANUAL on the first pass that reads the quench warm, the output
//!    held, AUTO refused while it stands and admitted once it is cold again;
//! 4. the single-loop form, on the quench valve itself;
//! 5. every refusal of the admission table and of `quench_layout`;
//! 6. the faceplate's `quench` only on a loop that carries a check.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{ControlMode, ControlledValue, LoopId, NodeKind, TripState};
use refinery_core::snapshot::{Command, ControlSnapshot, Snapshot};
use refinery_core::units::Kelvin;
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/hydrocracker_quench.toml");

const SETPOINT_C: f64 = 380.0;
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

fn opening(engine: &Engine, name: &str) -> f64 {
    let id = engine.graph.find_node(name).expect("a valve");
    match engine.graph.node(id).kind {
        NodeKind::Valve { opening, .. } => opening,
        ref other => panic!("'{name}' is a valve, not {other:?}"),
    }
}

fn duty_w(engine: &Engine, name: &str) -> f64 {
    let id = engine.graph.find_node(name).expect("a furnace");
    match engine.graph.node(id).kind {
        NodeKind::Furnace { duty, .. } => duty.value(),
        ref other => panic!("'{name}' is a furnace, not {other:?}"),
    }
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

/// The demo's trip block, from its comment to the end of the file.
fn without_trip(src: &str) -> String {
    let start = src
        .find("# The backup: fires AT OR ABOVE 400")
        .expect("the demo's trip");
    src[..start].to_string()
}

/// The single-loop form (gate 4): the bed loop writes the quench valve itself,
/// and the flow loop is gone. Gain and reset time from a sweep (K 0.01–0.05
/// per K, T_i 300 s): the pair that peaks about half a kelvin over the target.
fn single_loop(src: &str) -> String {
    let start = src
        .find("[[controls]]\nname = \"bed2_temperature\"")
        .expect("the demo's bed loop");
    let end = src
        .find("[[controls]]\nname = \"preheat_outlet\"")
        .expect("the demo's preheat loop");
    let single = r#"[[controls]]
name = "bed2_temperature"
measurement = { node = "bed2", variable = "temperature" }
actuator = "quench_valve"
algorithm = "pi"
mode = "auto"
action = "direct"
setpoint_c = 380.0
gain_per_k = 0.025
integral_time_s = 300.0
initial_output = 0.01

"#;
    format!("{}{single}{}", &src[..start], &src[end..])
}

/// The demo with its quench drawn from a SUPPLY at 135 °C — the drum's
/// temperature at load — instead of the drum, so a command can set the
/// quench's temperature (gate 3). The drum keeps its level loop.
fn quench_from_supply(src: &str) -> String {
    let src = swap(
        src,
        "[nodes.quench_pump]",
        "[nodes.quench_supply]\ntype = \"source\"\npressure_bar = 1.4\ntemperature_c = 135.0\n\
         composition = { vgo = 0.85, distillate = 0.15 }\n\n[nodes.quench_pump]",
    );
    swap(
        &src,
        "name = \"quench_suction\"\nfrom = \"product_drum\"",
        "name = \"quench_suction\"\nfrom = \"quench_supply\"",
    )
}

/// **Gate 1: the loop catches the runaway.** Born with the second bed at 375 °C
/// and the quench at its 0.25 kg/s minimum, the plant heads for the runaway
/// gate 2 shows. The loop raises the quench as the bed heats: the bed peaks
/// under half a kelvin over its target and settles onto it, the valve under a
/// fifth open, the quench cold on every tick and the trip never firing.
#[test]
fn the_loop_catches_the_runaway() {
    for solver in ["newton", "simple"] {
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
            (end - SETPOINT_C).abs() < 0.01,
            "{solver}: settled on the target, {end:.4} °C"
        );
        assert!(
            widest > 0.05 && widest < 0.2,
            "{solver}: the quench valve opens, at most {widest:.3}"
        );
        let first = bed_c(&engine, "bed1");
        assert!(
            first > 381.0 && first < 383.0,
            "{solver}: the first bed stays calm, {first:.3} °C"
        );
    }
}

/// **Gate 2: in MANUAL the backup trip catches it.** The demo with its bed
/// loop in MANUAL holds the quench at its minimum; the second bed heats to the
/// trip's 400 °C inside a quarter of an hour, the trip cuts the preheater, and
/// the plant cools toward its 300 °C feed.
#[test]
fn in_manual_the_trip_catches_it() {
    for solver in ["newton", "simple"] {
        let mut engine = build(&with_solver(&bed_loop_manual(DEMO), solver));
        let mut fired = None;
        for _ in 0..HALF_HOUR {
            tick(&mut engine);
            let snapshot = engine.snapshot();
            if fired.is_none() && snapshot.trips[0].state.is_tripped() {
                fired = Some(snapshot.tick);
                assert_eq!(
                    duty_w(&engine, "preheater"),
                    0.0,
                    "{solver}: the fire is cut"
                );
            }
        }
        let fired = fired.unwrap_or_else(|| panic!("{solver}: the trip must fire"));
        assert!(
            (6_000..9_000).contains(&fired),
            "{solver}: the trip fires between 10 and 15 minutes, on tick {fired}"
        );
        let end = bed_c(&engine, "bed2");
        assert!(
            end < 310.0,
            "{solver}: the second bed cools toward its feed, {end:.2} °C"
        );
    }
}

/// **Gate 2, continued: with no trip either, the quench goes warm.** Run from
/// the demo's own start, the bed runs away past 740 °C and its hot product
/// warms the drum past the first bed's ~385 °C outflow after about an hour and
/// a half (measured on the CLI, docs/DESIGN.md §62.11) — too long for a debug
/// gate, so this one starts where that run is headed: the bed run away at
/// 742 °C and the drum already at 360 °C, still colder than the bed, so the
/// plant loads. The drum warms past the main stream inside a quarter of an
/// hour. The bed loop is in MANUAL, so nothing freezes — but AUTO is refused
/// from that pass on, in words that say why.
#[test]
fn with_no_trip_either_the_quench_goes_warm() {
    for solver in ["newton", "simple"] {
        // Both walls out of reach (M58): this gate is about the quench, and a
        // bed loaded at 742 °C is past the shipped 550 °C limit.
        let no_walls = without_trip(&bed_loop_manual(DEMO))
            .replace("wall_failure_c = 550.0", "wall_failure_c = 3000.0");
        assert_eq!(no_walls.matches("wall_failure_c = 3000.0").count(), 2);
        let src = swap(
            &swap(
                &no_walls,
                "bed_temperature_c = 375.0",
                "bed_temperature_c = 742.0",
            ),
            "temperature_c = 135.0\ncomposition = { vgo = 0.85",
            "temperature_c = 360.0\ncomposition = { vgo = 0.85",
        );
        let mut engine = build(&with_solver(&src, solver));
        let id = loop_id(&engine.snapshot(), "bed2_temperature");
        let mut warm_at = None;
        for _ in 0..9_000 {
            tick(&mut engine);
            if !quench_cold(&engine.snapshot()) {
                warm_at = Some(engine.snapshot().tick);
                break;
            }
        }
        let warm_at = warm_at.unwrap_or_else(|| panic!("{solver}: the quench must go warm"));
        assert!(
            (3_000..9_000).contains(&warm_at),
            "{solver}: the drum passes the main stream between 5 and 15 minutes, tick {warm_at}"
        );
        let runaway = bed_c(&engine, "bed2");
        assert!(
            runaway > 740.0,
            "{solver}: the bed ran away, {runaway:.1} °C"
        );
        let err = engine
            .apply(Command::SetControllerMode {
                loop_id: id,
                mode: ControlMode::Auto,
            })
            .expect_err("AUTO on a loop whose quench is warm is refused");
        let err = err.to_string();
        assert!(
            err.contains("bed2_temperature") && err.contains("not colder than"),
            "{solver}: {err}"
        );
        assert_eq!(
            faceplate(&engine.snapshot(), "bed2_temperature").mode,
            ControlMode::Manual
        );
    }
}

/// **Gate 3: the freeze.** On the demo with its quench drawn from a 135 °C
/// supply, the loop holds the bed; then the supply is heated to 395 °C, over
/// the first bed's ~382 °C outflow. The tick after the command carries the hot
/// quench through its pipe; the pass at the top of the tick after THAT reads it
/// and switches the loop to MANUAL, and the faceplate shows both temperatures.
/// The flow target it set is held, AUTO is refused while the quench stands
/// warm, and once the supply is cold again AUTO is admitted on the first tick
/// the check reads it cold.
#[test]
fn a_warm_quench_freezes_the_loop() {
    for solver in ["newton", "simple"] {
        let mut engine = build(&with_solver(&quench_from_supply(DEMO), solver));
        for _ in 0..6_000 {
            tick(&mut engine);
        }
        let snapshot = engine.snapshot();
        assert!(quench_cold(&snapshot), "{solver}: cold while holding");
        let id = loop_id(&snapshot, "bed2_temperature");
        let held = flow_target(&snapshot);
        let supply = engine.graph.find_node("quench_supply").expect("the supply");
        engine
            .apply(Command::SetSourceTemperature {
                node: supply,
                temperature: Kelvin(395.0 + 273.15),
            })
            .unwrap_or_else(|e| panic!("{solver}: heat the supply: {e}"));
        let command_tick = snapshot.tick;
        tick(&mut engine);
        let carried = engine.snapshot();
        assert!(
            quench_cold(&carried)
                && faceplate(&carried, "bed2_temperature").mode == ControlMode::Auto,
            "{solver}: the first tick's pass still reads the cold pipe"
        );
        tick(&mut engine);
        let frozen = engine.snapshot();
        assert_eq!(frozen.tick, command_tick + 2);
        let plate = faceplate(&frozen, "bed2_temperature");
        let reading = plate.quench.expect("published");
        assert!(!reading.is_cold(), "{solver}: {reading:?}");
        assert!(
            (reading.quench_k.value() - 273.15 - 395.0).abs() < 1.0,
            "{solver}: the quench reads the hot supply, {reading:?}"
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
            (target - held).abs() < 0.5,
            "{solver}: held near where it stood, {target} against {held}"
        );
        let err = engine
            .apply(Command::SetControllerMode {
                loop_id: id,
                mode: ControlMode::Auto,
            })
            .expect_err("AUTO is refused while the quench is warm")
            .to_string();
        assert!(err.contains("not colder than"), "{solver}: {err}");
        // Frozen, the quench is the player's: its flow target moves by hand,
        // inside the bed loop's range (§29 fork 4's rule for a primary in
        // MANUAL), down to the minimum.
        let flow = loop_id(&engine.snapshot(), "quench_flow");
        engine
            .apply(Command::SetSetpoint {
                loop_id: flow,
                value: ControlledValue::Flow {
                    kg_per_s: refinery_core::units::KgPerSec(0.25),
                },
            })
            .unwrap_or_else(|e| panic!("{solver}: the quench turned down by hand: {e}"));
        tick(&mut engine);
        assert_eq!(flow_target(&engine.snapshot()), 0.25, "{solver}");

        engine
            .apply(Command::SetSourceTemperature {
                node: supply,
                temperature: Kelvin(135.0 + 273.15),
            })
            .unwrap_or_else(|e| panic!("{solver}: cool the supply: {e}"));
        // Read fresh: the pipe still carries the hot quench until a solve has
        // run on the cold supply, so AUTO is refused now and admitted after one
        // tick — the reading the next pass would act on.
        let auto = |engine: &mut Engine| {
            engine.apply(Command::SetControllerMode {
                loop_id: id,
                mode: ControlMode::Auto,
            })
        };
        assert!(
            auto(&mut engine).is_err(),
            "{solver}: still warm in its pipe"
        );
        tick(&mut engine);
        auto(&mut engine).unwrap_or_else(|e| panic!("{solver}: AUTO once cold: {e}"));
        assert_eq!(
            faceplate(&engine.snapshot(), "bed2_temperature").mode,
            ControlMode::Auto
        );
        for _ in 0..HALF_HOUR / 2 {
            tick(&mut engine);
        }
        let end = bed_c(&engine, "bed2");
        assert!(
            (end - SETPOINT_C).abs() < 0.5,
            "{solver}: back in AUTO, the loop brings the bed back, {end:.3} °C"
        );
    }
}

/// **Gate 4: the single-loop form.** The bed loop on the quench valve itself,
/// declared direct, holds the bed as the cascade does.
#[test]
fn a_single_loop_on_the_quench_valve_holds_the_bed() {
    for solver in ["newton", "simple"] {
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
}

/// **Gate 5: every refusal**, each on a substring of its own message.
#[test]
fn every_loop_on_a_bed_that_is_not_a_quench_is_refused() {
    let single = single_loop(DEMO);
    let bed_loop = |src: &str, from: &str, to: &str| -> String { swap(src, from, to) };

    // The single loop's action: absent, and reverse.
    let err = refusal(&bed_loop(
        &single,
        "action = \"direct\"\nsetpoint_c = 380.0",
        "setpoint_c = 380.0",
    ));
    assert!(
        err.contains("declares no `action`") && err.contains("§62.11"),
        "{err}"
    );
    let err = refusal(&bed_loop(
        &single,
        "action = \"direct\"\nsetpoint_c = 380.0",
        "action = \"reverse\"\nsetpoint_c = 380.0",
    ));
    assert!(
        err.contains("`action = \"reverse\"` on quench valve"),
        "{err}"
    );
    // A duty range on a valve.
    let err = refusal(&bed_loop(
        &single,
        "initial_output = 0.01\n\n[[controls]]\nname = \"preheat_outlet\"",
        "initial_output = 0.01\nmax_duty_mw = 5.0\n\n[[controls]]\nname = \"preheat_outlet\"",
    ));
    assert!(err.contains("max_duty_mw"), "{err}");

    // Not a quench: a valve whose line reaches no junction.
    let without_drum_loop = {
        let start = single
            .find("[[controls]]\nname = \"drum_level\"")
            .expect("the drum loop");
        let end = single[start..].find("# The backup").expect("the trip") + start;
        format!("{}{}", &single[..start], &single[end..])
    };
    let err = refusal(&bed_loop(
        &without_drum_loop,
        "actuator = \"quench_valve\"",
        "actuator = \"rundown_valve\"",
    ));
    assert!(
        err.contains("not a quench on that bed")
            && err.contains("rundown_valve")
            && err.contains("without joining another stream"),
        "{err}"
    );
    // Not a quench on THIS bed: the mix feeds bed2, and the loop measures bed1.
    let err = refusal(&bed_loop(
        &single,
        "measurement = { node = \"bed2\", variable = \"temperature\" }\nactuator = \"quench_valve\"",
        "measurement = { node = \"bed1\", variable = \"temperature\" }\nactuator = \"quench_valve\"",
    ));
    assert!(err.contains("reaches 'bed2', not the bed"), "{err}");
    // A mix with a third inlet.
    let third = swap(
        &single,
        "[nodes.mix]",
        "[nodes.spare]\ntype = \"source\"\npressure_bar = 8.0\ntemperature_c = 300.0\n\
         composition = { vgo = 1.0 }\n\n[nodes.mix]",
    )
        + "\n[[pipes]]\nname = \"spare_line\"\nfrom = \"spare\"\nto = \"mix\"\nlength_m = 5.0\n\
         diameter_m = 0.05\n";
    let err = refusal(&third);
    assert!(err.contains("has 3 inlet pipes"), "{err}");
    // A furnace on the quench line can heat it.
    let heated = swap(
        &swap(
            &single,
            "[nodes.quench_check]",
            "[nodes.quench_heater]\ntype = \"furnace\"\nduty_mw = 0.0\n\
             coil_heat_capacity_mj_per_k = 1.0\ncoil_ua_kw_per_k = 20.0\n\
             coil_temperature_c = 135.0\nflame_temperature_c = 1951.1\n\
             tube_failure_c = 650.0\ntube_rupture_area_cm2 = 5.0\n\
             fluid_heating_value_mj_per_kg = 42.0\n\n[nodes.quench_check]",
        ),
        "name = \"quench_discharge\"\nfrom = \"quench_pump\"\nto = \"quench_check\"",
        "name = \"quench_discharge\"\nfrom = \"quench_pump\"\nto = \"quench_heater\"",
    )
        + "\n[[pipes]]\nname = \"heater_line\"\nfrom = \"quench_heater\"\nto = \"quench_check\"\n\
         length_m = 5.0\ndiameter_m = 0.15\n";
    let err = refusal(&heated);
    assert!(err.contains("which can heat it"), "{err}");
    // A quench that is not cold at load: hotter than the bed, and level with it
    // — "cold" is strictly colder, and a tie is not.
    for drum_c in ["390.0", "375.0"] {
        let err = refusal(&swap(
            &single,
            "temperature_c = 135.0\ncomposition = { vgo = 0.85",
            &format!("temperature_c = {drum_c}\ncomposition = {{ vgo = 0.85"),
        ));
        assert!(
            err.contains("not colder than the bed's") && err.contains("product_drum"),
            "{drum_c} °C: {err}"
        );
    }

    // A duty on a bed: the preheater, and the cooler, singly.
    for (unit, unit_keys) in [
        ("preheater", "max_duty_mw = 26.0"),
        ("product_cooler", "max_duty_mw = 60.0"),
    ] {
        let src = bed_loop(
            &bed_loop(
                &without_heater_loop(&single),
                "actuator = \"quench_valve\"",
                &format!("actuator = \"{unit}\""),
            ),
            "initial_output = 0.01\n",
            &format!("initial_output = 0.5\n{unit_keys}\n"),
        );
        let err = refusal(&src);
        assert!(
            err.contains(&format!("through the duty of '{unit}'")),
            "{unit}: {err}"
        );
    }

    // The cascade's cells. Primary over a flow loop that is not a quench.
    let rundown_flow = bed_loop(
        &swap(
            DEMO,
            "measurement = { pipe = \"quench_line\", variable = \"flow\" }\nactuator = \"quench_valve\"",
            "measurement = { pipe = \"rundown_line\", variable = \"flow\" }\nactuator = \"rundown_valve\"",
        ),
        "[[controls]]\nname = \"drum_level\"",
        "[[controls]]\nname = \"drum_level_unused\"",
    );
    let rundown_flow = drop_loop(&rundown_flow, "drum_level_unused");
    let err = refusal(&rundown_flow);
    assert!(
        err.contains("cascade primary 'bed2_temperature'")
            && err.contains("not a quench on that bed"),
        "{err}"
    );
    // Primary over the preheater's outlet loop.
    let over_heater = swap(
        &drop_loop(DEMO, "quench_flow"),
        "actuator = { loop = \"quench_flow\" }",
        "actuator = { loop = \"preheat_outlet\" }",
    );
    let over_heater = swap(
        &over_heater,
        "range_min_kg_per_s = 0.25\nrange_max_kg_per_s = 10.0",
        "range_min_c = 340.0\nrange_max_c = 375.0",
    );
    let err = refusal(&over_heater);
    assert!(err.contains("through the duty of 'preheater'"), "{err}");
    // The primary declared reverse: the cascade's own sign rule.
    let block = bed_loop_block();
    let err = refusal(&swap(
        DEMO,
        block,
        &block.replace("action = \"direct\"", "action = \"reverse\""),
    ));
    assert!(err.contains("the plant makes it direct"), "{err}");
    // A bed loop as a cascade SECONDARY.
    let under_drum = single.clone()
        + "\n[[controls]]\nname = \"drum_temperature\"\n\
           measurement = { node = \"product_drum\", variable = \"temperature\" }\n\
           actuator = { loop = \"bed2_temperature\" }\nalgorithm = \"pi\"\nmode = \"auto\"\n\
           action = \"direct\"\nsetpoint_c = 135.0\ngain_per_k = 0.1\nintegral_time_s = 600.0\n\
           initial_output = 0.5\nrange_min_c = 370.0\nrange_max_c = 390.0\n";
    let err = refusal(&under_drum);
    assert!(err.contains("inner loop is its FAST one"), "{err}");
}

/// The single-loop fixture less the preheater's own loop, so the preheater is
/// free for a bed loop to name (two writers of one duty are refused first).
fn without_heater_loop(src: &str) -> String {
    drop_loop(src, "preheat_outlet")
}

/// `src` with the `[[controls]]` block named `name` removed.
fn drop_loop(src: &str, name: &str) -> String {
    let start = src
        .find(&format!("[[controls]]\nname = \"{name}\""))
        .unwrap_or_else(|| panic!("no loop '{name}' to drop"));
    let rest = &src[start + 1..];
    let end = [
        rest.find("[[controls]]"),
        rest.find("# The backup"),
        rest.find("[[trips]]"),
    ]
    .into_iter()
    .flatten()
    .min()
    .map(|i| i + start + 1)
    .unwrap_or(src.len());
    format!("{}{}", &src[..start], &src[end..])
}

/// **Gate 6: the faceplate's `quench` is published only by a loop carrying a
/// check** — the bed loop, from load — and is absent from the wire everywhere
/// else, so no earlier plant's bytes moved.
#[test]
fn only_a_quench_loop_publishes_its_check() {
    let engine = build(DEMO);
    let snapshot = engine.snapshot();
    for plate in &snapshot.controls {
        assert_eq!(
            plate.quench.is_some(),
            plate.name == "bed2_temperature",
            "{}",
            plate.name
        );
    }
    let reading = faceplate(&snapshot, "bed2_temperature")
        .quench
        .expect("from load");
    assert!(
        (reading.quench_k.value() - (135.0 + 273.15)).abs() < 1e-9
            && (reading.joins_k.value() - (375.0 + 273.15)).abs() < 1e-9,
        "at load: the drum against the bed, {reading:?}"
    );
    let wire = serde_json::to_string(&snapshot).expect("a snapshot serializes");
    assert_eq!(
        wire.matches("\"quench\":").count(),
        1,
        "one faceplate carries it"
    );

    let scenarios = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
    let mut seen = 0;
    for entry in std::fs::read_dir(&scenarios).expect("the scenarios folder") {
        let path = entry.expect("an entry").path();
        if path.extension().and_then(|e| e.to_str()) != Some("toml")
            || path.file_stem().and_then(|s| s.to_str()) == Some("hydrocracker_quench")
        {
            continue;
        }
        let src = std::fs::read_to_string(&path).expect("a scenario reads");
        let engine = build(&src);
        let wire = serde_json::to_string(&engine.snapshot()).expect("serializes");
        assert!(!wire.contains("\"quench\":"), "{}", path.display());
        seen += 1;
    }
    assert!(seen >= 48, "every earlier plant, {seen}");
}
