//! M29: a level or pressure loop on a FILL valve — reverse action on a valve,
//! `docs/DEFERRED.md` row E8, built as `docs/DESIGN.md` §32 specifies.
//!
//! The rule under test: a valve's sign on a holdup is TOPOLOGY, so the loader
//! reads it in one hop through `valve_side` — a valve whose inlet pipe starts at
//! the holdup is a drain (direct, the default), one whose outlet pipe ends there
//! is a fill (must SAY reverse) — and holds the declaration to it both ways. A
//! valve that is neither is refused in either direction (E21).
//!
//! Gates, one claim each:
//!
//! 1. **The demo holds where the parked loop runs to the roof**, on both
//!    fidelities, with its actuator interior throughout and `reverse` published.
//! 2. **Parked, the demo IS the drain demo's plant**: with both loops in MANUAL,
//!    every node and edge of the two files is bit-identical on every tick. So
//!    the two files differ in the loop and nothing else.
//! 3. **It starts and transfers without stepping its actuator** — the load-time
//!    seed and the MANUAL→AUTO seed both use the loop's own (reverse) action.
//! 4. **A one-metre step down shuts the fill on exactly one tick** and the loop
//!    recovers onto the new setpoint.
//! 5. **The pressure half**: a make-up valve holding a vessel, a fixture derived
//!    in-test from `vessel_pressure_control.toml`, against its parked twin.
//! 6. **Every side-against-action combination is refused for its own reason**,
//!    for a level and for a pressure.
//! 7. **E22, characterised**: a stopped pump drains the tank backwards through
//!    the fill, and the loop pins the fill wide open. Bounded, not fixed.

use refinery_core::graph::{ControlAction, ControlMode, ControlledValue, LoopId, NodeKind};
use refinery_core::snapshot::Command;
use refinery_core::units::Meter;
use refinery_core::Engine;
use refinery_scenarios::{build_engine, load_str};

const DEMO: &str = include_str!("../../../scenarios/tank_level_fill_control.toml");
const DRAIN_DEMO: &str = include_str!("../../../scenarios/tank_level_control.toml");
const PRESSURE_DEMO: &str = include_str!("../../../scenarios/vessel_pressure_control.toml");

/// The demo file's own declared numbers.
const SETPOINT_M: f64 = 4.0;
const DECLARED_FILL_OPENING: f64 = 0.5;
const DECLARED_DRAIN_OPENING: f64 = 0.2;
const TANK_HEIGHT_M: f64 = 10.0;

fn build(src: &str) -> Engine {
    build_engine(&load_str(src).expect("the plant must parse"))
        .unwrap_or_else(|e| panic!("the plant must build: {e}"))
}

fn refusal(what: &str, src: &str) -> String {
    match load_str(src) {
        Err(e) => e.to_string(),
        Ok(file) => match build_engine(&file) {
            Ok(_) => panic!("{what}: this plant should not have loaded"),
            Err(e) => e.to_string(),
        },
    }
}

fn swap(src: &str, from: &str, to: &str) -> String {
    assert!(
        src.contains(from),
        "the fixture's substitution must land: `{from}` is not in the plant"
    );
    src.replacen(from, to, 1)
}

fn manual(src: &str) -> String {
    swap(src, r#"mode = "auto""#, r#"mode = "manual""#)
}

fn tick(engine: &mut Engine, t: u64) {
    engine
        .tick()
        .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
}

fn measured(engine: &Engine) -> f64 {
    match engine.snapshot().controls[0]
        .measurement
        .expect("a stored quantity is measured from load")
    {
        ControlledValue::Level { m } => m.value(),
        ControlledValue::Pressure { pa } => pa.value(),
        other => panic!("these loops measure a level or a pressure, not {other:?}"),
    }
}

fn output(engine: &Engine) -> f64 {
    engine.snapshot().controls[0].output
}

fn opening(engine: &Engine, name: &str) -> f64 {
    let id = engine
        .graph
        .find_node(name)
        .unwrap_or_else(|| panic!("the plant declares a node '{name}'"));
    match engine.graph.node(id).kind {
        NodeKind::Valve { opening, .. } => opening,
        _ => panic!("'{name}' is a valve"),
    }
}

fn edge_flow(engine: &Engine, name: &str) -> f64 {
    engine
        .snapshot()
        .edges
        .iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("the plant declares a pipe '{name}'"))
        .stream
        .mass_flow
        .value()
}

/// The pressure fixture: `vessel_pressure_control.toml` with a make-up VALVE on
/// the make-up line, the loop moved onto it, and the vent left fixed at its
/// declared 0.30. Sized by a sweep: the make-up settles near 0.263, interior.
fn make_up_fixture() -> String {
    let plant = swap(
        PRESSURE_DEMO,
        "name = \"make_up\"\nfrom = \"header\"\nto = \"receiver\"",
        "name = \"make_up\"\nfrom = \"header\"\nto = \"make_up_valve\"",
    );
    let plant = swap(
        &plant,
        "[nodes.vent_valve]",
        "[nodes.make_up_valve]\ntype = \"valve\"\nkv = 12.0\nopening = 0.5\nx_t = 0.72\n\n\
         [[pipes]]\nname = \"make_up_tail\"\nfrom = \"make_up_valve\"\nto = \"receiver\"\n\
         length_m = 2.0\ndiameter_m = 0.05\n\n[nodes.vent_valve]",
    );
    let plant = swap(
        &plant,
        "actuator = \"vent_valve\"",
        "actuator = \"make_up_valve\"\naction = \"reverse\"",
    );
    swap(&plant, "initial_output = 0.30", "initial_output = 0.5")
}

// ------------------------------------------------------------------ gate 1

/// The demo holds 4 m; parked, the same file runs to the roof. Both fidelities,
/// because a valve loop is where the game solver's sweeps meet the controller.
///
/// Measured (Newton and simple agree to 1e-9): peak 4.007483 m, inside 0.01 m
/// of the setpoint from tick 1 570–1 580, the fill never leaving
/// [0.2598, 0.5099], 3.998491 m at tick 6 000.
#[test]
fn the_fill_loop_holds_its_level_where_the_parked_loop_runs_to_the_roof() {
    for solver in ["newton", "simple"] {
        let plant = swap(DEMO, "flow = \"newton\"", &format!("flow = \"{solver}\""));
        let mut auto = build(&plant);
        assert_eq!(
            auto.snapshot().controls[0].action,
            ControlAction::Reverse,
            "{solver}: a fill-valve loop publishes its action"
        );
        let (mut peak, mut low, mut high) = (f64::MIN, f64::MAX, f64::MIN);
        let mut last_outside = 0;
        for t in 1..=6_000 {
            tick(&mut auto, t);
            let (level, fill) = (measured(&auto), opening(&auto, "discharge_valve"));
            peak = peak.max(level);
            low = low.min(fill);
            high = high.max(fill);
            if (level - SETPOINT_M).abs() > 0.01 {
                last_outside = t;
            }
        }
        assert!(
            peak < SETPOINT_M + 0.01,
            "{solver}: the tuning overshoots by under a centimetre (measured 4.007483 m), \
             peaked at {peak} m"
        );
        assert!(
            (1_500..=1_650).contains(&last_outside),
            "{solver}: inside 0.01 m of the setpoint from tick ~1 580, last outside at \
             {last_outside}"
        );
        assert!(
            low > 0.25 && high < 0.52,
            "{solver}: the fill stays interior, measured [0.2598, 0.5099], ran [{low}, {high}]"
        );
        assert_eq!(
            opening(&auto, "level_valve"),
            DECLARED_DRAIN_OPENING,
            "{solver}: the drain is fixed in this file"
        );

        let mut parked = build(&manual(&plant));
        for t in 1..=6_000 {
            tick(&mut parked, t);
        }
        assert_eq!(opening(&parked, "discharge_valve"), DECLARED_FILL_OPENING);
        assert!(
            measured(&parked) > 0.9 * TANK_HEIGHT_M,
            "{solver}: parked, the fill at 0.5 overruns the fixed drain (measured 9.190452 m), \
             ended at {} m",
            measured(&parked)
        );
    }
}

// ------------------------------------------------------------------ gate 2

/// Parked, the fill demo and the drain demo are ONE plant: the fill at 0.5, the
/// drain at 0.2, and neither loop writing. Every node and every edge is
/// bit-identical on every tick, so the two files differ only in their loop.
#[test]
fn parked_the_fill_demo_is_the_drain_demo_bit_for_bit() {
    let mut fill = build(&manual(DEMO));
    let mut drain = build(&manual(DRAIN_DEMO));
    for t in 1..=6_000 {
        tick(&mut fill, t);
        tick(&mut drain, t);
        let (a, b) = (fill.snapshot(), drain.snapshot());
        assert_eq!(
            serde_json::to_string(&a.nodes).expect("serializes"),
            serde_json::to_string(&b.nodes).expect("serializes"),
            "tick {t}: the parked plants' nodes differ"
        );
        assert_eq!(
            serde_json::to_string(&a.edges).expect("serializes"),
            serde_json::to_string(&b.edges).expect("serializes"),
            "tick {t}: the parked plants' edges differ"
        );
    }
}

// ------------------------------------------------------------------ gate 3

/// The two seeds a PI loop's memory has — at load from `initial_output`, and at
/// MANUAL→AUTO from the actuator's position — are both back-calculated against
/// the loop's OWN error. Taken against the other sign, the first output would
/// step by `2·K·e` (M18's finding): here 2 × 0.5 × 2 m = 2, a full stroke.
/// Measured exact in both cases.
#[test]
fn the_fill_loop_starts_and_transfers_without_stepping_its_valve() {
    let mut engine = build(DEMO);
    tick(&mut engine, 1);
    assert_eq!(
        output(&engine),
        DECLARED_FILL_OPENING,
        "the first output is the declared `initial_output`"
    );

    for t in 2..=3_000 {
        tick(&mut engine, t);
    }
    engine
        .apply(Command::SetControllerMode {
            loop_id: LoopId(0),
            mode: ControlMode::Manual,
        })
        .expect("a loop can be parked");
    let valve = engine.graph.find_node("discharge_valve").expect("declared");
    engine
        .apply(Command::SetValveOpening {
            node: valve,
            opening: 0.4,
        })
        .expect("a parked loop's valve is the operator's");
    for t in 3_001..=3_200 {
        tick(&mut engine, t);
    }
    // The level is now well off the setpoint, so a wrongly-signed seed would
    // show. Assert that, or the transfer below proves nothing.
    assert!(
        (measured(&engine) - SETPOINT_M).abs() > 0.1,
        "control: the hand-set fill must have moved the level, read {} m",
        measured(&engine)
    );
    engine
        .apply(Command::SetControllerMode {
            loop_id: LoopId(0),
            mode: ControlMode::Auto,
        })
        .expect("a loop can be resumed");
    tick(&mut engine, 3_201);
    assert_eq!(
        output(&engine),
        0.4,
        "the first AUTO output is the position the operator left"
    );
}

// ------------------------------------------------------------------ gate 4

/// From the settled demo, the setpoint stepped DOWN a metre subtracts
/// `K × 1 m = 0.5` from an output near 0.26: the fill shuts. It sits at exactly
/// 0 for ONE tick — the back-calculation parks the memory at the clamp, and the
/// next tick's error is a hair smaller, so the output leaves 0 by `K·Δe` (M18's
/// (iv)) — then opens only as fast as the level falls. Measured: shut on tick
/// 3 001 alone, a 2.989823 m trough, inside 0.01 m of 3 m from tick 4 724.
#[test]
fn a_step_down_shuts_the_fill_for_one_tick_and_the_loop_recovers() {
    let mut engine = build(DEMO);
    for t in 1..=3_000 {
        tick(&mut engine, t);
    }
    engine
        .apply(Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Level { m: Meter(3.0) },
        })
        .expect("a level inside the tank is a valid target");
    let mut shut = Vec::new();
    let (mut trough, mut last_outside) = (f64::MAX, 0);
    for t in 3_001..=9_000 {
        tick(&mut engine, t);
        if opening(&engine, "discharge_valve") == 0.0 {
            shut.push(t);
        }
        trough = trough.min(measured(&engine));
        if (measured(&engine) - 3.0).abs() > 0.01 {
            last_outside = t;
        }
    }
    assert_eq!(
        shut,
        vec![3_001],
        "the fill is shut on the step's tick alone"
    );
    assert!(
        trough > 2.98,
        "the level undershoots by about a centimetre (measured 2.989823 m), trough {trough} m"
    );
    assert!(
        (4_650..=4_800).contains(&last_outside),
        "inside 0.01 m of 3 m from ~4 724, last outside at {last_outside}"
    );
}

// ------------------------------------------------------------------ gate 5

/// A make-up valve holding a vessel at 20 bar — E8's other named plant. The
/// loader reads the make-up valve as a fill of `receiver` and the loop declares
/// reverse. Parked, the same fixture settles at 23.45 bar: a wrong number, not
/// a runaway, because the vent's flow rises with the vessel's own pressure (the
/// M10.1 finding). `dt = 0.1`, so 6 000 ticks are ten minutes.
///
/// Measured: peak 20.0109 bar, inside 0.01 bar from tick 2 050–2 060, the
/// make-up valve at its declared 0.5 on tick 1 and in [0.2631, 0.4908] after.
#[test]
fn a_make_up_valve_holds_a_vessel_where_its_parked_twin_does_not() {
    let plant = make_up_fixture();
    let mut auto = build(&plant);
    let (mut peak, mut low, mut high, mut last_outside) = (f64::MIN, f64::MAX, f64::MIN, 0);
    for t in 1..=6_000 {
        tick(&mut auto, t);
        let bar = measured(&auto) / 1e5;
        let make_up = opening(&auto, "make_up_valve");
        peak = peak.max(bar);
        low = low.min(make_up);
        high = high.max(make_up);
        if (bar - 20.0).abs() > 0.01 {
            last_outside = t;
        }
    }
    assert!(peak < 20.02, "peak {peak} bar (measured 20.0109)");
    assert!(
        (1_950..=2_150).contains(&last_outside),
        "inside 0.01 bar from ~2 060, last outside at {last_outside}"
    );
    assert!(
        low > 0.25 && high <= 0.5,
        "the make-up stays interior after its declared 0.5 start, measured [0.2631, 0.5], \
         ran [{low}, {high}]"
    );

    let mut parked = build(&manual(&plant));
    for t in 1..=6_000 {
        tick(&mut parked, t);
    }
    let parked_bar = measured(&parked) / 1e5;
    assert!(
        parked_bar > 23.0,
        "parked, the make-up at 0.5 settles far above 20 bar (measured 23.4513), read \
         {parked_bar}"
    );
}

// ------------------------------------------------------------------ gate 6

/// Every row of `check_holdup_valve_action`'s table that refuses, for a level
/// and for a pressure, each asserted on a substring of its OWN message.
#[test]
fn every_side_against_action_mismatch_is_refused_for_its_own_reason() {
    // Every substitution is anchored on the loop's own lines: the demo's header
    // comment quotes `action = "reverse"` too, and an unanchored swap edits it.
    const LOOP_ACTION: &str = "actuator = \"discharge_valve\"\naction = \"reverse\"";
    const MAKE_UP_ACTION: &str = "actuator = \"make_up_valve\"\naction = \"reverse\"";
    let no_action = swap(DEMO, LOOP_ACTION, "actuator = \"discharge_valve\"");
    let neither = swap(
        DEMO,
        "measurement = { node = \"receiving_tank\", variable = \"level\" }",
        "measurement = { node = \"supply_tank\", variable = \"level\" }",
    );
    let recycle = swap(
        DEMO,
        "[nodes.rundown]",
        "[nodes.recycle_valve]\ntype = \"valve\"\nkv = 10.0\nopening = 0.1\n\n\
         [[pipes]]\nname = \"recycle_out\"\nfrom = \"receiving_tank\"\nto = \"recycle_valve\"\n\
         length_m = 5.0\ndiameter_m = 0.05\n\n[[pipes]]\nname = \"recycle_back\"\n\
         from = \"recycle_valve\"\nto = \"receiving_tank\"\nlength_m = 5.0\ndiameter_m = 0.05\n\n\
         [nodes.rundown]",
    );
    let recycle = swap(
        &recycle,
        "actuator = \"discharge_valve\"",
        "actuator = \"recycle_valve\"",
    );
    let recycle = swap(&recycle, "initial_output = 0.5", "initial_output = 0.1");

    let cases: Vec<(&str, String, &[&str])> = vec![
        (
            "a fill valve with no action",
            no_action.clone(),
            &["which FILLS it", "declares no `action`"],
        ),
        (
            "a fill valve declared direct",
            swap(
                DEMO,
                LOOP_ACTION,
                "actuator = \"discharge_valve\"\naction = \"direct\"",
            ),
            &[
                "which FILLS",
                "open the valve wider the higher the level ran",
            ],
        ),
        (
            "a drain valve declared reverse",
            swap(
                DRAIN_DEMO,
                "gain_per_m = 0.25\n",
                "gain_per_m = 0.25\naction = \"reverse\"\n",
            ),
            &["which DRAINS 'receiving_tank'"],
        ),
        (
            "a valve two hops from its tank, reverse",
            neither.clone(),
            &["neither a drain", "docs/DEFERRED.md E21"],
        ),
        (
            "a valve two hops from its tank, direct (refused since M29)",
            swap(&neither, LOOP_ACTION, "actuator = \"discharge_valve\""),
            &["neither a drain", "docs/DEFERRED.md E21"],
        ),
        (
            "a valve that both drains and fills its tank",
            recycle,
            &["both drains it and fills it"],
        ),
        (
            "a make-up valve with no action",
            swap(
                &make_up_fixture(),
                MAKE_UP_ACTION,
                "actuator = \"make_up_valve\"",
            ),
            &["which FILLS it", "RAISES the pressure"],
        ),
        (
            "a make-up valve declared direct",
            swap(
                &make_up_fixture(),
                MAKE_UP_ACTION,
                "actuator = \"make_up_valve\"\naction = \"direct\"",
            ),
            &["open the valve wider the higher the pressure ran"],
        ),
        (
            "a vent declared reverse",
            swap(
                PRESSURE_DEMO,
                "gain_per_bar = 0.10\n",
                "gain_per_bar = 0.10\naction = \"reverse\"\n",
            ),
            &["which DRAINS 'receiver'"],
        ),
    ];
    for (what, plant, expected) in cases {
        let message = refusal(what, &plant);
        for phrase in expected {
            assert!(
                message.contains(phrase),
                "{what}: expected the refusal to say `{phrase}`, and it said: {message}"
            );
        }
    }
}

// ------------------------------------------------------------------ gate 7

/// E22, characterised rather than fixed. A stopped pump keeps its resistance
/// (M22: "a stopped pump conducts"), and the receiving tank sits 5 m above the
/// supply's floor, so with the pump off the tank flows BACKWARDS through the
/// fill. Its level falls under the setpoint, so the loop opens the fill — the
/// right sign for a level that is low, and the wrong thing to do to a tank
/// draining through it. The fill pins at 1 and stays there; the flow turns
/// forward again once the two tanks balance, and the level settles near 0.95 m.
/// Bounded by the clamp: no NaN, no `Err`.
///
/// Measured: −3.96 kg/s through the fill 300 ticks after the stop, the fill at 1
/// by 600, the flow forward again by 900, 0.9516 m at +3 000.
#[test]
fn a_stopped_pump_drains_the_tank_back_through_its_wide_open_fill() {
    let mut engine = build(DEMO);
    for t in 1..=3_000 {
        tick(&mut engine, t);
    }
    let pump = engine.graph.find_node("transfer_pump").expect("declared");
    engine
        .apply(Command::SetPumpOn {
            node: pump,
            on: false,
        })
        .expect("a pump can be stopped");
    let mut most_backward = 0.0_f64;
    for t in 3_001..=6_000 {
        tick(&mut engine, t);
        most_backward = most_backward.min(edge_flow(&engine, "fill_line"));
    }
    assert!(
        most_backward < -3.0,
        "the tank drains backwards through the fill, most backward {most_backward} kg/s"
    );
    assert_eq!(output(&engine), 1.0, "the loop pins the fill wide open");
    assert!(
        measured(&engine) < 1.5,
        "the level ends far under the setpoint (measured 0.9516 m), read {} m",
        measured(&engine)
    );
}
