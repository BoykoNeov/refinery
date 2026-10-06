//! M45.1: a loop that holds its valve while its pump is stopped —
//! `docs/DEFERRED.md` row E25, built as `docs/DESIGN.md` §50 specifies.
//!
//! `on_pump_stop = { pump = "…", output = … }` on a `[[controls]]` entry: while
//! the loop is in AUTO and that pump is off, the loop writes `output` to its
//! valve every tick and re-seeds its memory against it (output tracking), so
//! the pump's restart — by a person or by a trip — resumes from the held
//! position instead of from a valve the loop wound wide open.
//!
//! Gates, one claim each:
//!
//! 1. **It holds and ramps**: on the M30 demo, stopped for 3 000 ticks, the
//!    fill valve stands exactly at the declared output on every tick with the
//!    loop still in AUTO; on restart the first output is the held one to within
//!    a step, and the flow climbs instead of stepping. Gate 7 of
//!    `check_valve_reference.rs` is the control: 24.80 kg/s on the same tick.
//! 2. **A person's valve is left alone**: a loop in MANUAL holds nothing.
//! 3. **A running pump changes nothing**: with the pump on, the key is inert,
//!    bit for bit.
//! 4. **The faceplate publishes the declaration**, and a loop without it
//!    publishes the bytes it always did.
//! 5. **Every refusal, each on its own message.**
//! 6. **The demo** (`tank_level_fill_pump_hold.toml`): a pump trip that resets
//!    itself restarts the pump four times in 6 000 ticks, each restart a ramp;
//!    its twin without the key surges to 23.6 kg/s on every restart.

use refinery_core::graph::{ControlMode, LoopId, NodeKind, OnPumpStop};
use refinery_core::snapshot::Command;
use refinery_core::Engine;
use refinery_scenarios::{build_engine, load_str};

const DEMO: &str = include_str!("../../../scenarios/tank_level_fill_check_valve.toml");
const COOLER: &str = include_str!("../../../scenarios/tank_temperature_control.toml");
const HOLD_DEMO: &str = include_str!("../../../scenarios/tank_level_fill_pump_hold.toml");

/// The key, as the fixtures declare it.
const KEY: &str = "on_pump_stop = { pump = \"transfer_pump\", output = 0.0 }\n";

const FIDELITIES: [&str; 2] = ["newton", "simple"];

fn swap(src: &str, from: &str, to: &str) -> String {
    assert!(
        src.contains(from),
        "the fixture's substitution must land: `{from}` is not in the plant"
    );
    src.replacen(from, to, 1)
}

fn on(fidelity: &str, src: &str) -> String {
    swap(src, "flow = \"newton\"", &format!("flow = \"{fidelity}\""))
}

/// The demo with its trip removed (the `[[trips]]` table is the file's last):
/// these gates stop and start the pump by command.
fn untripped(src: &str) -> String {
    let at = src.find("\n[[trips]]\n").expect("the demo declares a trip");
    src[..=at].to_string()
}

/// The untripped demo with the level loop holding on its pump's stop.
fn holding(src: &str) -> String {
    swap(
        src,
        "initial_output = 0.5\n",
        &format!("initial_output = 0.5\n{KEY}"),
    )
}

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

fn assert_refused(what: &str, src: &str, needle: &str) {
    let err = refusal(what, src);
    assert!(
        err.contains(needle),
        "{what}: refused for its own reason, expected `{needle}` in: {err}"
    );
}

fn tick(engine: &mut Engine, label: &str, t: u64) {
    engine
        .tick()
        .unwrap_or_else(|e| panic!("{label}: the plant must run: tick {t}: {e}"));
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

fn fill_opening(engine: &Engine) -> f64 {
    let node = engine.graph.find_node("discharge_valve").expect("declared");
    match engine.graph.node(node).kind {
        NodeKind::Valve { opening, .. } => opening,
        ref other => panic!("the fill is a valve, read {other:?}"),
    }
}

fn set_pump(engine: &mut Engine, on: bool) {
    let pump = engine.graph.find_node("transfer_pump").expect("declared");
    engine
        .apply(Command::SetPumpOn { node: pump, on })
        .expect("the pump takes the command");
}

// ------------------------------------------------------------------ gate 1

/// **The loop holds its valve while its pump is stopped, and ramps on restart.**
/// The control is gate 7 of `check_valve_reference.rs`: without the key the
/// loop pins the fill wide open and the restart puts 24.80 kg/s through it on
/// the first tick, against 6.90 settled.
#[test]
fn the_loop_holds_its_valve_while_its_pump_is_stopped_and_ramps_on_restart() {
    for fidelity in FIDELITIES {
        let mut engine = build(&on(fidelity, &holding(&untripped(DEMO))));
        for t in 1..=3_000 {
            tick(&mut engine, fidelity, t);
        }
        let settled = edge_flow(&engine, "fill_line");
        set_pump(&mut engine, false);
        for t in 3_001..=6_000 {
            tick(&mut engine, fidelity, t);
            let face = &engine.snapshot().controls[0];
            assert_eq!(fill_opening(&engine), 0.0, "{fidelity} tick {t}: held shut");
            assert_eq!(
                face.output, 0.0,
                "{fidelity} tick {t}: the faceplate says so"
            );
            assert_eq!(
                face.mode,
                ControlMode::Auto,
                "{fidelity} tick {t}: still AUTO"
            );
        }
        set_pump(&mut engine, true);
        tick(&mut engine, fidelity, 6_001);
        let first_output = engine.snapshot().controls[0].output;
        let first_flow = edge_flow(&engine, "fill_line");
        let mut steepest = first_flow;
        let mut previous = first_flow;
        let mut peak = first_flow;
        for t in 6_002..=6_100 {
            tick(&mut engine, fidelity, t);
            let flow = edge_flow(&engine, "fill_line");
            steepest = steepest.max(flow - previous);
            peak = peak.max(flow);
            previous = flow;
        }
        eprintln!(
            "MEASURE {fidelity}: settled {settled}, first output {first_output}, first flow \
             {first_flow}, steepest rise {steepest} kg/s a tick, peak to 6100 {peak}"
        );
        assert!(
            first_output < 0.01,
            "{fidelity}: the restart resumes from the held output, read {first_output}"
        );
        assert!(
            first_flow < 0.5,
            "{fidelity}: the first tick passes a trickle, not the pump curve: {first_flow} kg/s"
        );
        assert!(
            steepest < 0.5,
            "{fidelity}: the flow climbs, it does not step: {steepest} kg/s in one tick"
        );
    }
}

// ------------------------------------------------------------------ gate 2

/// **A person's valve is left alone.** The key belongs to the loop, and a loop
/// in MANUAL writes nothing: the opening a person set stays through the stop.
#[test]
fn a_manual_loop_is_left_to_its_person() {
    let mut engine = build(&holding(&untripped(DEMO)));
    for t in 1..=100 {
        tick(&mut engine, "running", t);
    }
    let fill = engine.graph.find_node("discharge_valve").expect("declared");
    engine
        .apply(Command::SetControllerMode {
            loop_id: LoopId(0),
            mode: ControlMode::Manual,
        })
        .expect("the loop can be put in MANUAL");
    engine
        .apply(Command::SetValveOpening {
            node: fill,
            opening: 0.6,
        })
        .expect("a person sets the valve");
    set_pump(&mut engine, false);
    for t in 101..=300 {
        tick(&mut engine, "stopped, MANUAL", t);
        assert_eq!(fill_opening(&engine), 0.6, "tick {t}: the person's opening");
    }
}

// ------------------------------------------------------------------ gate 3

/// **A running pump changes nothing.** With the key and without it, the demo
/// publishes the same numbers on every tick while the pump runs.
#[test]
fn with_the_pump_running_the_key_changes_nothing() {
    for fidelity in FIDELITIES {
        let mut with = build(&on(fidelity, &holding(&untripped(DEMO))));
        let mut without = build(&on(fidelity, &untripped(DEMO)));
        for t in 1..=3_000 {
            tick(&mut with, fidelity, t);
            tick(&mut without, fidelity, t);
            let (a, b) = (with.snapshot(), without.snapshot());
            for (x, y) in a.edges.iter().zip(&b.edges) {
                assert_eq!(
                    x.stream.mass_flow.value().to_bits(),
                    y.stream.mass_flow.value().to_bits(),
                    "{fidelity} tick {t}: '{}'",
                    x.name
                );
            }
            assert_eq!(
                a.controls[0].output.to_bits(),
                b.controls[0].output.to_bits(),
                "{fidelity} tick {t}: the loop's output"
            );
        }
    }
}

// ------------------------------------------------------------------ gate 4

/// **The faceplate publishes the declaration, not a "holding" flag** (DESIGN
/// §50): whether the loop holds is its `mode` and the pump's own `on`, both
/// already published — the rule `ControlSnapshot::drives` set for an open
/// cascade. A loop without the key publishes the bytes it always did.
#[test]
fn the_faceplate_publishes_the_declaration() {
    let engine = build(&holding(&untripped(DEMO)));
    let pump = engine.graph.find_node("transfer_pump").expect("declared");
    assert_eq!(
        engine.snapshot().controls[0].on_pump_stop,
        Some(OnPumpStop { pump, output: 0.0 })
    );
    let plain = build(&untripped(DEMO));
    let json = serde_json::to_string(&plain.snapshot().controls[0]).expect("serializes");
    assert!(
        !json.contains("on_pump_stop"),
        "a loop without the key publishes no such field: {json}"
    );
}

// ------------------------------------------------------------------ gate 5

/// The demo's loop rewritten as a level-over-flow cascade on the fill (the
/// pairing `cascade_control_reference.rs` builds as `level_fill`).
fn fill_cascade(primary_extra: &str, secondary_extra: &str) -> String {
    let plant = untripped(DEMO);
    let at = plant
        .find("\n[[controls]]\n")
        .expect("the demo declares a loop");
    format!(
        "{}
[[controls]]
name = \"level_master\"
measurement = {{ node = \"receiving_tank\", variable = \"level\" }}
actuator = {{ loop = \"fill_flow\" }}
algorithm = \"pi\"
mode = \"auto\"
action = \"reverse\"
setpoint_m = 4.0
gain_per_m = 0.25
integral_time_s = 600.0
initial_output = 0.375
range_min_kg_per_s = 1.0
range_max_kg_per_s = 25.0
{primary_extra}
[[controls]]
name = \"fill_flow\"
measurement = {{ pipe = \"fill_line\", variable = \"flow\" }}
actuator = \"discharge_valve\"
algorithm = \"pi\"
mode = \"auto\"
action = \"reverse\"
setpoint_kg_per_s = 10.0
gain_per_kg_per_s = 0.02
integral_time_s = 10.0
initial_output = 0.5
{secondary_extra}",
        &plant[..=at]
    )
}

#[test]
fn every_refusal_names_its_own_reason() {
    // The cascade fixture loads as it stands, so the refusals below are the key's.
    build(&fill_cascade("", ""));

    assert_refused(
        "a proportional loop",
        &swap(
            &swap(
                &holding(&untripped(DEMO)),
                "algorithm = \"pi\"",
                "algorithm = \"p\"",
            ),
            "integral_time_s = 600.0\ninitial_output = 0.5\n",
            "",
        ),
        "has no memory to hold",
    );
    assert_refused(
        "a loop on a cooler",
        &swap(
            COOLER,
            "initial_output = 0.25\n",
            "initial_output = 0.25\non_pump_stop = { pump = \"chiller\", output = 0.0 }\n",
        ),
        "actuates cooler 'chiller'",
    );
    assert_refused(
        "a cascade primary",
        &fill_cascade(KEY, ""),
        "is a cascade primary",
    );
    assert_refused(
        "a cascade secondary",
        &fill_cascade("", KEY),
        "is a cascade secondary",
    );
    assert_refused(
        "an output past the valve's range",
        &swap(
            &holding(&untripped(DEMO)),
            "output = 0.0 }",
            "output = 1.5 }",
        ),
        "not an opening in [0, 1]",
    );
    assert_refused(
        "an unknown pump",
        &swap(
            &holding(&untripped(DEMO)),
            "pump = \"transfer_pump\"",
            "pump = \"nope\"",
        ),
        "names no node 'nope'",
    );
    assert_refused(
        "a node that is not a pump",
        &swap(
            &holding(&untripped(DEMO)),
            "pump = \"transfer_pump\"",
            "pump = \"supply_tank\"",
        ),
        "'supply_tank', which is not a pump",
    );
    assert_refused(
        "a misspelt key in the table",
        &swap(
            &holding(&untripped(DEMO)),
            "output = 0.0 }",
            "opening = 0.0 }",
        ),
        "opening",
    );
}

// ------------------------------------------------------------------ gate 6

/// One 6 000-tick run of the hold demo: each tick the pump came back on, and
/// the fill's flow on that tick.
fn restarts(src: &str, label: &str) -> Vec<(u64, f64)> {
    let mut engine = build(src);
    let pump = engine.graph.find_node("transfer_pump").expect("declared");
    let mut was_on = true;
    let mut out = Vec::new();
    for t in 1..=6_000 {
        tick(&mut engine, label, t);
        let on = matches!(
            engine.graph.node(pump).kind,
            NodeKind::Pump { on: true, .. }
        );
        if on && !was_on {
            out.push((t, edge_flow(&engine, "fill_line")));
        }
        was_on = on;
    }
    out
}

/// **The demo**: the supply trip stops the pump at 7.0 m and restarts it at
/// 7.5 m by itself (M40's `auto`), on ticks 1 162, 2 496, 3 824 and 5 148; on
/// each restart tick the fill passes a trickle. Its twin without the key puts
/// the pump curve through the wide-open fill on every restart, empties the
/// supply's deadband in about 110 ticks, and cycles seven times.
#[test]
fn the_demo_ramps_where_its_twin_surges() {
    for fidelity in FIDELITIES {
        let held = restarts(&on(fidelity, HOLD_DEMO), fidelity);
        let ticks: Vec<u64> = held.iter().map(|&(t, _)| t).collect();
        assert_eq!(
            ticks,
            [1_162, 2_496, 3_824, 5_148],
            "{fidelity}: the restarts"
        );
        for &(t, flow) in &held {
            assert!(
                flow < 0.1,
                "{fidelity} tick {t}: a trickle, read {flow} kg/s"
            );
        }
        let twin = restarts(
            &on(
                fidelity,
                &swap(
                    HOLD_DEMO,
                    "on_pump_stop = { pump = \"transfer_pump\", output = 0.0 }
",
                    "",
                ),
            ),
            &format!("{fidelity} twin"),
        );
        assert_eq!(
            twin.len(),
            7,
            "{fidelity}: the twin cycles seven times: {twin:?}"
        );
        for &(t, flow) in &twin {
            assert!(
                flow > 20.0,
                "{fidelity} twin tick {t}: the surge, read {flow} kg/s"
            );
        }
    }
}
