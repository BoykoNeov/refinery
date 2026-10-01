//! M25.1: cascade control — one loop sets another loop's target (docs/DESIGN.md
//! §29).
//!
//! A cascade PRIMARY declares `actuator = { loop = "…" }`, and its output is the
//! SECONDARY's setpoint as a fraction of a range declared on the primary. Pass 2
//! of the tick's control pass runs primaries first, so the secondary acts on this
//! tick's target; a primary whose secondary will not act this tick — not in AUTO,
//! or nothing to measure — is OPEN, writes nothing, tracks, and re-seeds its
//! memory every open tick.
//!
//! The demo is `scenarios/furnace_cascade_control.toml`: the M18 heater plant with
//! the tank loop outside and the M19 outlet loop inside. The hand model of §29
//! (premise 1) predicted its numbers, and they are asserted here against bands
//! stated beside each one. The level pairing and the cooler pairing ship on
//! fixtures, because no shipped plant has a tank with both a feed and a metered
//! drain, and the demo is a furnace (§29 premise 4, gate 9).
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{ControlMode, ControlledValue, LoopId, NodeKind};
use refinery_core::snapshot::{Command, ControlSnapshot};
use refinery_core::units::{Kelvin, Watt};
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/furnace_cascade_control.toml");
const TANK_LOOP: &str = include_str!("../../../scenarios/tank_temperature_heating.toml");
const OUTLET_LOOP: &str = include_str!("../../../scenarios/furnace_outlet_control.toml");
const LEVEL: &str = include_str!("../../../scenarios/tank_level_control.toml");
const COOLER: &str = include_str!("../../../scenarios/tank_temperature_control.toml");

/// The demo's declared range, °C, named once.
const RANGE_MIN_C: f64 = 40.0;
const RANGE_MAX_C: f64 = 65.0;
const TANK_SETPOINT_C: f64 = 60.0;
/// The band every settling claim in §29 is stated against.
const BAND_K: f64 = 0.06;

// ------------------------------------------------------------------- helpers

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the fixture must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("the fixture must build: {e}"))
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

fn tick(engine: &mut Engine) {
    let t = engine.snapshot().tick + 1;
    engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
}

fn run(engine: &mut Engine, ticks: u64) {
    for _ in 0..ticks {
        tick(engine);
    }
}

fn faceplate(engine: &Engine, name: &str) -> ControlSnapshot {
    engine
        .snapshot()
        .controls
        .into_iter()
        .find(|c| c.name == name)
        .unwrap_or_else(|| panic!("no loop called '{name}'"))
}

fn loop_id(engine: &Engine, name: &str) -> LoopId {
    faceplate(engine, name).id
}

/// A node's published temperature, °C.
fn node_c(engine: &Engine, name: &str) -> f64 {
    engine
        .snapshot()
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("no node called '{name}'"))
        .temperature_k
        - 273.15
}

/// A loop's setpoint, in kelvin, bare.
fn setpoint_k(engine: &Engine, name: &str) -> f64 {
    match faceplate(engine, name).setpoint {
        ControlledValue::Temperature { k } => k.value(),
        other => panic!("'{name}' holds a temperature, not {other:?}"),
    }
}

fn setpoint_flow(engine: &Engine, name: &str) -> f64 {
    match faceplate(engine, name).setpoint {
        ControlledValue::Flow { kg_per_s } => kg_per_s.value(),
        other => panic!("'{name}' holds a flow, not {other:?}"),
    }
}

/// The demo's range map, in the order `SetpointRange::position` evaluates it.
fn demo_position(setpoint_k: f64) -> f64 {
    let min = RANGE_MIN_C + 273.15;
    (setpoint_k - min) / ((RANGE_MAX_C + 273.15) - min)
}

fn duty_w(engine: &Engine, unit: &str) -> f64 {
    let id = engine.graph.find_node(unit).expect("the unit exists");
    match engine.graph.node(id).kind {
        NodeKind::Furnace { duty } | NodeKind::Cooler { duty } => duty.value(),
        ref other => panic!("'{unit}' is a furnace or cooler, not {other:?}"),
    }
}

fn tank_level_m(engine: &Engine, tank: &str) -> f64 {
    let id = engine.graph.find_node(tank).expect("the tank exists");
    match &engine.graph.node(id).kind {
        NodeKind::Tank(t) => t.level(&engine.slate).value(),
        other => panic!("'{tank}' is a tank, not {other:?}"),
    }
}

fn set_mode(engine: &mut Engine, name: &str, mode: ControlMode) {
    let loop_id = loop_id(engine, name);
    engine
        .apply(Command::SetControllerMode { loop_id, mode })
        .unwrap_or_else(|e| panic!("'{name}' to {mode:?}: {e}"));
}

fn set_setpoint_c(engine: &mut Engine, name: &str, c: f64) -> Result<(), String> {
    let loop_id = loop_id(engine, name);
    engine
        .apply(Command::SetSetpoint {
            loop_id,
            value: ControlledValue::Temperature {
                k: Kelvin(c + 273.15),
            },
        })
        .map_err(|e| e.to_string())
}

fn fire(engine: &mut Engine, node: &str, watts: f64) {
    let node = engine.graph.find_node(node).expect("the node exists");
    engine
        .apply(Command::SetHeatInput {
            node,
            power: Watt(watts),
        })
        .unwrap_or_else(|e| panic!("a fire is accepted: {e}"));
}

/// Replace exactly one occurrence, or fail: a substitution that finds nothing
/// returns the plant unchanged, and a gate built on it would be testing the
/// shipped file while saying it tests another (the M19.1 lesson).
fn sub(src: &str, from: &str, to: &str) -> String {
    assert_eq!(
        src.matches(from).count(),
        1,
        "the fixture's substitution must land exactly once: {from:?}"
    );
    src.replacen(from, to, 1)
}

/// The plant text before its first `[[controls]]`, and each control block.
fn split_controls(src: &str) -> (String, Vec<String>) {
    let mut parts = src.split("\n[[controls]]\n");
    let plant = parts.next().expect("a plant").to_string();
    let blocks: Vec<String> = parts.map(|b| format!("\n[[controls]]\n{b}")).collect();
    (plant, blocks)
}

/// The demo with its two loops declared the other way round: the SECONDARY
/// first. A demo's own file order makes an evaluation order inert (M15.1), so
/// the partition is defended on this.
fn secondary_first() -> String {
    let (plant, blocks) = split_controls(DEMO);
    assert_eq!(blocks.len(), 2, "the demo declares two loops");
    assert!(
        blocks[0].contains("actuator = { loop = "),
        "primary first in the demo"
    );
    format!("{plant}{}\n{}", blocks[1], blocks[0])
}

/// The demo with a FEED VALVE between the source and the heater, so the furnace
/// can be starved of flow (M19.1's fixture, on the cascade).
fn with_feed_valve() -> String {
    let plant = sub(
        DEMO,
        "[nodes.heater]\ntype = \"furnace\"",
        "[nodes.feed_valve]\ntype = \"valve\"\nkv = 150.0\nopening = 1.0\n\n\
         [nodes.heater]\ntype = \"furnace\"",
    );
    let plant = sub(
        &plant,
        "name = \"feed_line\"\nfrom = \"cool_feed\"\nto = \"heater\"",
        "name = \"feed_line\"\nfrom = \"cool_feed\"\nto = \"feed_valve\"",
    );
    format!(
        "{plant}\n[[pipes]]\nname = \"valve_line\"\nfrom = \"feed_valve\"\nto = \"heater\"\n\
         length_m = 2.0\ndiameter_m = 0.10\n"
    )
}

fn set_valve(engine: &mut Engine, valve: &str, opening: f64) {
    let node = engine.graph.find_node(valve).expect("the valve exists");
    engine
        .apply(Command::SetValveOpening { node, opening })
        .unwrap_or_else(|e| panic!("no loop owns '{valve}': {e}"));
}

/// The level plant without its loop.
fn level_plant() -> String {
    let (plant, blocks) = split_controls(LEVEL);
    assert_eq!(blocks.len(), 1, "the level demo declares one loop");
    plant
}

/// A level held by the flow through the DRAIN (§29 fork 5: direct): the outer
/// loop on `receiving_tank`'s level, the inner on `level_valve`'s outlet pipe.
fn level_drain(outer_action: &str) -> String {
    format!(
        "{}
[[controls]]
name = \"level_master\"
measurement = {{ node = \"receiving_tank\", variable = \"level\" }}
actuator = {{ loop = \"drain_flow\" }}
algorithm = \"pi\"
mode = \"auto\"
{outer_action}
setpoint_m = 4.0
gain_per_m = 0.25
integral_time_s = 600.0
initial_output = 0.375
range_min_kg_per_s = 1.0
range_max_kg_per_s = 25.0

[[controls]]
name = \"drain_flow\"
measurement = {{ pipe = \"rundown_line\", variable = \"flow\" }}
actuator = \"level_valve\"
algorithm = \"pi\"
mode = \"auto\"
action = \"reverse\"
setpoint_kg_per_s = 10.0
gain_per_kg_per_s = 0.02
integral_time_s = 10.0
initial_output = 0.2
",
        level_plant()
    )
}

/// A level held by the flow through the FILL (§29 fork 5: reverse): the inner
/// loop on `discharge_valve`'s outlet pipe, which ends at the tank; the drain is
/// a fixed valve.
fn level_fill(outer_action: &str) -> String {
    format!(
        "{}
[[controls]]
name = \"level_master\"
measurement = {{ node = \"receiving_tank\", variable = \"level\" }}
actuator = {{ loop = \"fill_flow\" }}
algorithm = \"pi\"
mode = \"auto\"
{outer_action}
setpoint_m = 4.0
gain_per_m = 0.25
integral_time_s = 600.0
initial_output = 0.375
range_min_kg_per_s = 1.0
range_max_kg_per_s = 25.0

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
",
        sub(
            &level_plant(),
            "opening = 0.2       # equal to `initial_output` below, and that is the point",
            "opening = 0.3",
        )
    )
}

/// The cooler pairing (§29 gate 9): `tank_temperature_control.toml`'s plant, the
/// heater demo's mirror — an 80 °C feed, a cooler whose outlet pipe ends at the
/// tank. The outlet loop is DIRECT inside (a cooler's sign), the tank loop REVERSE
/// outside over 55–80 °C: a hotter outlet target is less cooling, and the feed is
/// the top because a cooler cannot heat.
fn cooler_cascade(outer_action: &str) -> String {
    let (plant, _) = split_controls(COOLER);
    format!(
        "{plant}
[[controls]]
name = \"tank_temperature\"
measurement = {{ node = \"hold_tank\", variable = \"temperature\" }}
actuator = {{ loop = \"outlet_temperature\" }}
algorithm = \"pi\"
mode = \"auto\"
{outer_action}
setpoint_c = 60.0
gain_per_k = 0.1331
integral_time_s = 600.0
initial_output = 0.6
range_min_c = 55.0
range_max_c = 80.0

[[controls]]
name = \"outlet_temperature\"
measurement = {{ node = \"chiller\", variable = \"temperature\" }}
actuator = \"chiller\"
algorithm = \"pi\"
mode = \"auto\"
setpoint_c = 70.0
gain_per_k = 0.015
integral_time_s = 10.0
initial_output = 0.25
max_duty_mw = 2.0
"
    )
}

/// The demo after `ticks`, built fresh. Nothing in this file shares an engine
/// between gates: an engine is not `Clone` (a loop's memory may not be forked).
fn demo_after(src: &str, ticks: u64) -> Engine {
    let mut engine = build(src);
    run(&mut engine, ticks);
    engine
}

/// The last tick, counted from the run's start, on which `name`'s temperature was
/// outside `BAND_K` of `target_c`, plus one — the tick it is inside from.
fn inside_from(engine: &mut Engine, ticks: u64, name: &str, target_c: f64) -> u64 {
    let start = engine.snapshot().tick;
    let mut last_out = start;
    for _ in 0..ticks {
        tick(engine);
        if (node_c(engine, name) - target_c).abs() > BAND_K {
            last_out = engine.snapshot().tick;
        }
    }
    last_out - start + 1
}

// --------------------------------------------- gate 2: startup, under the cap

/// **Gate 2. The demo's startup, against the hand model's numbers.**
///
/// §29 premise 3 predicted, from a hand model that reproduces both single loops on
/// this plant (premise 1): the tank inside 0.06 K of 60 °C from tick 2 578 (the
/// open first tick included), the outlet never above the 65 °C range top, and the
/// outer loop on its clamp on 69 ticks. Measured: **2 579, 64.9925 °C, 69**. The
/// band on the settling tick is ±50 ticks, the width M19.1 and M18.1 found
/// between a hand model and the engine on this same plant (one to two ticks)
/// widened by an order of magnitude so a tuning-preserving refactor of the tick
/// does not trip it, and still far inside the two controls below.
///
/// **The controls are read AT the cascade's own settling tick** (§29, "Corrected
/// before building"): the tank-loop file has already fired its outlet to
/// 66.92 °C, past the cap, and the outlet-loop file's tank reads 58.32 °C there,
/// 1.7 K outside the band. At tick 6 000 that second control reads 59.939 °C and
/// would pass by 0.001 K, which is why it is not read there.
#[test]
fn the_cascade_starts_up_under_its_cap_and_reaches_its_own_clamp() {
    let mut engine = build(DEMO);
    let primary = "tank_temperature";
    let mut hottest_outlet = f64::MIN;
    let mut at_top = 0;
    let mut last_out = 0;
    for _ in 0..6000 {
        tick(&mut engine);
        let t = engine.snapshot().tick;
        if (node_c(&engine, "hold_tank") - TANK_SETPOINT_C).abs() > BAND_K {
            last_out = t;
        }
        hottest_outlet = hottest_outlet.max(node_c(&engine, "heater"));
        if faceplate(&engine, primary).output == 1.0 {
            at_top += 1;
        }
    }
    let settled = last_out + 1;
    assert!(
        (2528..=2628).contains(&settled),
        "the hand model puts the tank inside 0.06 K from tick 2 578; the engine from {settled}"
    );
    assert!(
        hottest_outlet <= RANGE_MAX_C,
        "the outlet must never run hotter than the primary's range top, 65 °C: it reached \
         {hottest_outlet} °C"
    );
    assert!(
        at_top > 0,
        "the shipped file must reach the outer loop's clamp (M8.4's coverage gap); it sat at \
         u = 1 on {at_top} ticks (hand: 69)"
    );

    let mut tank_loop = build(TANK_LOOP);
    let mut outlet_loop = build(OUTLET_LOOP);
    let mut tank_loop_hottest = f64::MIN;
    for _ in 0..settled {
        tick(&mut tank_loop);
        tick(&mut outlet_loop);
        tank_loop_hottest = tank_loop_hottest.max(node_c(&tank_loop, "heater"));
    }
    assert!(
        tank_loop_hottest > 66.9,
        "control: the tank loop alone fires its outlet past the cap (66.92 °C), got \
         {tank_loop_hottest}"
    );
    let outlet_loop_tank = node_c(&outlet_loop, "hold_tank");
    assert!(
        TANK_SETPOINT_C - outlet_loop_tank > 1.0,
        "control: the outlet loop alone leaves the tank well outside the band at tick \
         {settled} (58.32 °C), got {outlet_loop_tank}"
    );
}

/// **Tick 1 is an OPEN cascade** (§29 fork 4). The outlet does not exist at load,
/// so the secondary will not act; the primary writes nothing, and its faceplate
/// reads the secondary's declared 50 °C as a position, 0.4. From tick 2 the
/// secondary measures and the primary writes.
#[test]
fn the_first_tick_is_open_and_the_primary_writes_nothing() {
    let mut engine = build(DEMO);
    let declared = setpoint_k(&engine, "outlet_temperature");
    assert_eq!(
        faceplate(&engine, "tank_temperature").output,
        demo_position(declared),
        "at load the primary's faceplate is the secondary's declared setpoint as a position"
    );
    tick(&mut engine);
    assert_eq!(
        setpoint_k(&engine, "outlet_temperature"),
        declared,
        "tick 1: the secondary has nothing to measure, so its primary writes nothing"
    );
    assert_eq!(
        faceplate(&engine, "tank_temperature").output,
        demo_position(declared)
    );
    assert_eq!(faceplate(&engine, "outlet_temperature").measurement, None);
    tick(&mut engine);
    assert_ne!(
        setpoint_k(&engine, "outlet_temperature"),
        declared,
        "tick 2: the secondary measures, the cascade closes, and the primary writes"
    );
}

// -------------------------------------------- gate 3: the same-tick hand-off

/// **Gate 3. A primary's new target reaches the furnace on the SAME tick.**
///
/// On a running plant the primary's setpoint is stepped on one engine and not on
/// its twin, and the furnace duty must differ on the very next tick: the primary
/// runs first, writes the secondary's setpoint, and the secondary acts on it.
/// With the secondary first it acts on last tick's target and the duty is
/// bit-identical on that tick (§29 mutation 1). Repeated with the loops declared
/// the other way round, because the demo declares the primary first and a file's
/// own order makes an evaluation order inert (M15.1).
#[test]
fn a_primary_hands_its_new_target_to_the_furnace_on_the_same_tick() {
    for (label, src) in [
        ("demo order", DEMO.to_string()),
        ("secondary first", secondary_first()),
    ] {
        let mut stepped = demo_after(&src, 3000);
        let mut twin = demo_after(&src, 3000);
        assert_eq!(
            duty_w(&stepped, "heater"),
            duty_w(&twin, "heater"),
            "{label}: the twins are one plant until the step"
        );
        set_setpoint_c(&mut stepped, "tank_temperature", 61.0)
            .unwrap_or_else(|e| panic!("{label}: the primary's setpoint is its own: {e}"));
        tick(&mut stepped);
        tick(&mut twin);
        assert_ne!(
            setpoint_k(&stepped, "outlet_temperature"),
            setpoint_k(&twin, "outlet_temperature"),
            "{label}: the primary wrote a new target"
        );
        assert_ne!(
            duty_w(&stepped, "heater"),
            duty_w(&twin, "heater"),
            "{label}: the secondary must act on THIS tick's target — a duty equal to the \
             twin's means it ran on last tick's"
        );
    }
}

// ------------------------------------------- gates 4 and 5: the disturbances

/// **Gate 4. A fire at the HEATER is rejected before the tank sees much of it.**
///
/// A furnace with no thermal mass passes a heater fire straight to the tank, so
/// the tank loop alone pays for it at the tank's pace (+0.820 K on the engine,
/// §29 premise 2). Inside a cascade the outlet loop sees it the next tick and
/// takes the fire's duty off the furnace: hand +0.0753 K, measured +0.0753 K.
#[test]
fn a_heater_fire_is_taken_out_by_the_inner_loop() {
    let mut cascade = demo_after(DEMO, 6000);
    let mut tank_loop = demo_after(TANK_LOOP, 6000);
    for engine in [&mut cascade, &mut tank_loop] {
        fire(engine, "heater", 0.3e6);
    }
    let (mut cascade_peak, mut tank_loop_peak) = (0.0_f64, 0.0_f64);
    for _ in 0..3000 {
        tick(&mut cascade);
        tick(&mut tank_loop);
        cascade_peak = cascade_peak.max((node_c(&cascade, "hold_tank") - TANK_SETPOINT_C).abs());
        tank_loop_peak =
            tank_loop_peak.max((node_c(&tank_loop, "hold_tank") - TANK_SETPOINT_C).abs());
    }
    assert!(
        cascade_peak <= 0.1,
        "the cascade must hold the tank within 0.1 K of a heater fire (hand 0.0753 K), got \
         {cascade_peak}"
    );
    assert!(
        tank_loop_peak >= 0.8,
        "control: the tank loop alone must feel the fire at the tank (0.820 K), got \
         {tank_loop_peak}"
    );
}

/// **Gate 5. A fire at the TANK is corrected by the outer loop**, which the outlet
/// loop alone cannot even see.
///
/// The cascade is back inside 0.06 K after +2 006 ticks (hand +2 004; asserted
/// within 2 100). The outlet loop alone holds its outlet perfectly and leaves the
/// tank 5.04 K above where it stood when the fire started, still rising at
/// +6 000 — measured from its OWN pre-fire temperature, because that plant had
/// not finished its startup at tick 6 000 (59.939 °C); from 60 °C it reads 4.98.
#[test]
fn a_tank_fire_is_corrected_by_the_outer_loop() {
    let mut cascade = demo_after(DEMO, 6000);
    let mut outlet_loop = demo_after(OUTLET_LOOP, 6000);
    let outlet_loop_before = node_c(&outlet_loop, "hold_tank");
    for engine in [&mut cascade, &mut outlet_loop] {
        fire(engine, "hold_tank", 0.3e6);
    }
    let back_inside = inside_from(&mut cascade, 6000, "hold_tank", TANK_SETPOINT_C);
    run(&mut outlet_loop, 6000);
    assert!(
        back_inside <= 2100,
        "the cascade must bring the tank back inside 0.06 K (hand +2 004), got +{back_inside}"
    );
    let outlet_loop_rise = node_c(&outlet_loop, "hold_tank") - outlet_loop_before;
    assert!(
        outlet_loop_rise >= 4.9,
        "control: nothing in the outlet loop measures the tank, so the fire stands (5.04 K), \
         got {outlet_loop_rise}"
    );
}

// --------------------------------------------- gate 6: open, and close again

/// **Gate 6. The secondary in MANUAL opens the cascade, and closing it is
/// bumpless.**
///
/// While a human drives the furnace by hand and a tank fire burns, the primary
/// writes nothing — the secondary's setpoint is bit-identical on every open tick —
/// and its faceplate equals that setpoint as a position, to the bit (§29 mutations
/// 2 and 6). Then the secondary goes back to AUTO in the engine's order: `apply`
/// seeds the secondary against its current, frozen setpoint, and the next tick's
/// primary moves it by one tick of control. Hand: −0.0039 K; measured −0.0039 K.
/// A primary whose memory sat untouched while open moves it by 1.37 K (hand,
/// mutation 5), so the bound is 0.01 K.
#[test]
fn an_open_cascade_writes_nothing_and_closes_without_a_bump() {
    let mut engine = demo_after(DEMO, 6000);
    set_mode(&mut engine, "outlet_temperature", ControlMode::Manual);
    let heater = engine.graph.find_node("heater").expect("a heater");
    engine
        .apply(Command::SetFurnaceDuty {
            node: heater,
            duty: Watt(1.0e6),
        })
        .unwrap_or_else(|e| panic!("in MANUAL a human drives the furnace: {e}"));
    fire(&mut engine, "hold_tank", 0.3e6);

    let frozen = setpoint_k(&engine, "outlet_temperature");
    for _ in 0..300 {
        tick(&mut engine);
        let primary = faceplate(&engine, "tank_temperature");
        assert_eq!(
            primary.mode,
            ControlMode::Auto,
            "the primary itself stays in AUTO"
        );
        assert_eq!(
            setpoint_k(&engine, "outlet_temperature"),
            frozen,
            "tick {}: an open primary writes nothing",
            engine.snapshot().tick
        );
        assert_eq!(
            primary.output,
            demo_position(frozen),
            "tick {}: the open primary's faceplate is its secondary's SETPOINT as a position",
            engine.snapshot().tick
        );
    }

    set_mode(&mut engine, "outlet_temperature", ControlMode::Auto);
    tick(&mut engine);
    let step = setpoint_k(&engine, "outlet_temperature") - frozen;
    assert!(
        step != 0.0 && step.abs() <= 0.01,
        "closing must move the secondary's setpoint by one tick of control (hand −0.0039 K), \
         not by a stale memory (1.37 K): it moved {step:+e} K"
    );
}

// ------------------------------------------------- gate 7: a trip opens it

/// **Gate 7. A trip on the inner loop's valve opens the cascade**, on the level
/// fixture (§29 fork 4: "trips need no new rule"). The trip forces the flow loop on
/// its valve to MANUAL, and from that very tick its primary writes nothing and
/// tracks — with no line of trip code knowing a primary exists.
#[test]
fn a_trip_on_the_inner_valve_opens_the_cascade() {
    let src = format!(
        "{}
[[trips]]
name = \"high_level\"
measurement = {{ node = \"receiving_tank\", variable = \"level\" }}
direction = \"high\"
limit_m = 3.0
actions = [{{ valve = \"level_valve\", position = 1.0 }}]
",
        level_drain("action = \"direct\"")
    );
    let mut engine = build(&src);
    let mut before = setpoint_flow(&engine, "drain_flow");
    let mut tripped_at = None;
    for _ in 0..3000 {
        tick(&mut engine);
        let t = engine.snapshot().tick;
        let now = setpoint_flow(&engine, "drain_flow");
        if tripped_at.is_none() && engine.snapshot().trips[0].state.is_tripped() {
            tripped_at = Some(t);
        }
        if tripped_at.is_some() {
            assert_eq!(
                faceplate(&engine, "drain_flow").mode,
                ControlMode::Manual,
                "tick {t}: the trip put the inner loop in MANUAL"
            );
            assert_eq!(
                now, before,
                "tick {t}: from the tripping tick the primary writes nothing"
            );
            assert_eq!(
                faceplate(&engine, "level_master").output,
                (now - 1.0) / (25.0 - 1.0),
                "tick {t}: and tracks the setpoint it left"
            );
        }
        before = now;
    }
    let at = tripped_at.expect("the fixture's level must cross 3.0 m and trip");
    assert!(at > 1, "the trip fires on a crossing, mid-run (tick {at})");
}

// ------------------------------------------------------ gate 8: refusals

/// The demo with its PRIMARY's block edited by `edit` — the demo declares two
/// `action` lines, so a whole-file substitution would hit the wrong one.
fn demo_primary(edit: impl Fn(&str) -> String) -> String {
    let (plant, blocks) = split_controls(DEMO);
    format!("{plant}{}{}", edit(&blocks[0]), blocks[1])
}

fn demo_secondary(edit: impl Fn(&str) -> String) -> String {
    let (plant, blocks) = split_controls(DEMO);
    format!("{plant}{}{}", blocks[0], edit(&blocks[1]))
}

fn assert_refused(label: &str, src: &str, needle: &str) {
    let message = refusal(src);
    assert!(
        message.contains(needle),
        "{label}: refused, but not by its own message (wanted {needle:?}): {message}"
    );
}

/// **Gate 8, the commands.** A secondary under an AUTO primary refuses a setpoint
/// command (the primary owns it), admits one under a MANUAL primary, and refuses
/// one outside the primary's range even then — `check_loop_owned_duty`'s rule,
/// which the note did not list and its own fork 4 needs.
#[test]
fn a_secondary_setpoint_belongs_to_its_primary() {
    let mut engine = demo_after(DEMO, 10);
    let owned = set_setpoint_c(&mut engine, "outlet_temperature", 55.0)
        .expect_err("a primary in AUTO owns its secondary's setpoint");
    assert!(
        owned.contains("has its setpoint written by cascade primary 'tank_temperature'"),
        "{owned}"
    );
    set_mode(&mut engine, "tank_temperature", ControlMode::Manual);
    set_setpoint_c(&mut engine, "outlet_temperature", 55.0)
        .unwrap_or_else(|e| panic!("under a MANUAL primary a human moves it: {e}"));
    assert_eq!(setpoint_k(&engine, "outlet_temperature"), 55.0 + 273.15);
    tick(&mut engine);
    assert_eq!(
        faceplate(&engine, "tank_temperature").output,
        demo_position(55.0 + 273.15),
        "the MANUAL primary's faceplate tracks the human's setpoint"
    );
    let outside = set_setpoint_c(&mut engine, "outlet_temperature", 70.0)
        .expect_err("outside the primary's range even in MANUAL");
    assert!(
        outside.contains("outside the range of cascade primary"),
        "{outside}"
    );
    // And the primary's own setpoint is its own, in either mode.
    set_setpoint_c(&mut engine, "tank_temperature", 61.0)
        .unwrap_or_else(|e| panic!("a primary has no primary: {e}"));
}

/// **Gate 8, at load: the links.** Each refusal by its own message.
#[test]
fn a_cascade_link_is_refused_where_it_could_not_regulate() {
    const SUPERVISOR: &str = "
[[controls]]
name = \"supervisor\"
measurement = { node = \"hold_tank\", variable = \"level\" }
actuator = { loop = \"tank_temperature\" }
algorithm = \"p\"
mode = \"auto\"
action = \"direct\"
setpoint_m = 4.968
gain_per_m = 0.1
range_min_c = 40.0
range_max_c = 70.0
";
    assert_refused(
        "a chain three deep",
        &format!("{DEMO}{SUPERVISOR}"),
        "a chain three deep",
    );
    assert_refused(
        "a self-link",
        &demo_primary(|b| {
            sub(
                b,
                "actuator = { loop = \"outlet_temperature\" }",
                "actuator = { loop = \"tank_temperature\" }",
            )
        }),
        "drives itself",
    );
    assert_refused(
        "a mutual pair",
        &demo_secondary(|b| {
            let b = sub(
                b,
                "actuator = \"heater\"",
                "actuator = { loop = \"tank_temperature\" }",
            );
            sub(
                &b,
                "max_duty_mw = 2.0",
                "range_min_c = 40.0\nrange_max_c = 80.0",
            )
        }),
        "drive each other",
    );
    let (_, blocks) = split_controls(DEMO);
    assert_refused(
        "two primaries on one secondary",
        &format!(
            "{DEMO}{}",
            blocks[0].replace("name = \"tank_temperature\"", "name = \"rival\"")
        ),
        "both write the setpoint of loop 'outlet_temperature'",
    );
    assert_refused(
        "an unknown secondary",
        &demo_primary(|b| {
            sub(
                b,
                "{ loop = \"outlet_temperature\" }",
                "{ loop = \"nobody\" }",
            )
        }),
        "drives unknown loop 'nobody'",
    );
    assert!(
        refinery_scenarios::load_str(&demo_primary(|b| sub(
            b,
            "{ loop = \"outlet_temperature\" }",
            "{ loops = \"outlet_temperature\" }"
        )))
        .is_err(),
        "a misspelt link table is refused (by serde's untagged message, accepted as such)"
    );
}

/// **Gate 8, at load: the range keys.**
#[test]
fn a_cascade_range_is_refused_unless_every_end_is_a_setpoint_the_secondary_takes() {
    assert_refused(
        "a missing range end",
        &demo_primary(|b| sub(b, "range_max_c = 65.0\n", "")),
        "declares no `range_max_c`",
    );
    assert_refused(
        "a foreign range key",
        &demo_primary(|b| {
            sub(
                b,
                "range_max_c = 65.0\n",
                "range_max_c = 65.0\nrange_min_kg_per_s = 1.0\n",
            )
        }),
        "a range over a flow secondary",
    );
    assert_refused(
        "a range end the secondary would refuse",
        &demo_primary(|b| sub(b, "range_min_c = 40.0", "range_min_c = -300.0")),
        "the bottom of its range is a setpoint loop 'outlet_temperature' would refuse",
    );
    assert_refused(
        "an empty range",
        &demo_primary(|b| sub(b, "range_min_c = 40.0", "range_min_c = 65.0")),
        "strictly below its top",
    );
    assert_refused(
        "a flow range starting at zero",
        &sub(
            &level_drain("action = \"direct\""),
            "range_min_kg_per_s = 1.0",
            "range_min_kg_per_s = 0.0",
        ),
        "not a finite flow above zero",
    );
    assert_refused(
        "a range on a loop that writes a node",
        &demo_secondary(|b| {
            sub(
                b,
                "max_duty_mw = 2.0",
                "max_duty_mw = 2.0\nrange_min_c = 40.0",
            )
        }),
        "declares `range_min_c` and actuates node 'heater'",
    );
    assert_refused(
        "a duty range on a primary",
        &demo_primary(|b| {
            sub(
                b,
                "range_min_c = 40.0",
                "range_min_c = 40.0\nmax_duty_mw = 2.0",
            )
        }),
        "declares `max_duty_mw` and drives loop",
    );
    assert_refused(
        "a secondary declared outside its primary's range",
        &demo_secondary(|b| sub(b, "setpoint_c = 50.0", "setpoint_c = 70.0")),
        "outside the range of its cascade primary",
    );
}

/// **Gate 8, at load: the pairings and the primary's sign** (§29 fork 5). Both
/// admitted pairings refuse a wrong sign and a missing one; the cooler pairing —
/// the one the demo does not run — loads under a reverse primary and refuses a
/// direct one; a unit two hops from the tank, a holdup inner loop and the cross
/// pairs are refused, each with its own reason.
#[test]
fn a_cascade_pairing_is_admitted_only_where_its_sign_is_checked() {
    let reverse = "action = \"reverse\"";
    assert_refused(
        "the demo's primary declared direct",
        &demo_primary(|b| sub(b, reverse, "action = \"direct\"")),
        "declares `action = \"direct\"`, and the plant makes it reverse acting",
    );
    assert_refused(
        "the demo's primary with no action",
        &demo_primary(|b| sub(b, &format!("{reverse}\n"), "")),
        "declares no `action`",
    );
    assert_refused(
        "a drain primary declared reverse",
        &level_drain(reverse),
        "declares `action = \"reverse\"`, and the plant makes it direct acting",
    );
    assert_refused(
        "a drain primary with no action",
        &level_drain(""),
        "declares no `action`",
    );
    assert_refused(
        "a fill primary declared direct",
        &level_fill("action = \"direct\""),
        "declares `action = \"direct\"`, and the plant makes it reverse acting",
    );
    assert_refused(
        "a fill primary with no action",
        &level_fill(""),
        "declares no `action`",
    );
    assert_refused(
        "a cooler-outlet primary declared direct",
        &cooler_cascade("action = \"direct\""),
        "declares `action = \"direct\"`, and the plant makes it reverse acting",
    );
    build(&cooler_cascade(reverse));

    let two_hops = sub(
        DEMO,
        "name = \"heated_line\"\nfrom = \"heater\"\nto = \"hold_tank\"",
        "name = \"heated_line\"\nfrom = \"heater\"\nto = \"mid\"",
    );
    let two_hops = sub(
        &two_hops,
        "[nodes.hold_tank]",
        "[nodes.mid]\ntype = \"junction\"\n\n[nodes.hold_tank]",
    );
    let two_hops = sub(
        &two_hops,
        "\n[[controls]]\nname = \"tank_temperature\"",
        "\n[[pipes]]\nname = \"mid_line\"\nfrom = \"mid\"\nto = \"hold_tank\"\nlength_m = 1.0\n\
         diameter_m = 0.10\n\n[[controls]]\nname = \"tank_temperature\"",
    );
    assert_refused(
        "a unit two hops from the tank",
        &two_hops,
        "whose outlet pipe does not end at that tank",
    );

    let over_tank_loop = format!(
        "{TANK_LOOP}
[[controls]]
name = \"level_master\"
measurement = {{ node = \"hold_tank\", variable = \"level\" }}
actuator = {{ loop = \"tank_temperature\" }}
algorithm = \"p\"
mode = \"auto\"
action = \"direct\"
setpoint_m = 4.968
gain_per_m = 0.1
range_min_c = 40.0
range_max_c = 70.0
"
    );
    assert_refused(
        "an inner loop on a holdup's temperature",
        &over_tank_loop,
        "measures a holdup's temperature",
    );
    let over_level_loop = format!(
        "{LEVEL}
[[controls]]
name = \"master\"
measurement = {{ node = \"supply_tank\", variable = \"level\" }}
actuator = {{ loop = \"receiving_level\" }}
algorithm = \"p\"
mode = \"auto\"
action = \"direct\"
setpoint_m = 8.0
gain_per_m = 0.1
"
    );
    assert_refused(
        "an inner loop on a holdup's level",
        &over_level_loop,
        "measures a holdup's level",
    );
    let level_over_outlet = demo_primary(|b| {
        let b = sub(b, "variable = \"temperature\"", "variable = \"level\"");
        let b = sub(&b, "setpoint_c = 60.0", "setpoint_m = 4.968");
        sub(&b, "gain_per_k = 0.133", "gain_per_m = 0.133")
    });
    assert_refused(
        "a level over an outlet temperature",
        &level_over_outlet,
        "not an admitted cascade pairing",
    );
    let temperature_over_flow = {
        let src = level_drain("action = \"direct\"");
        let src = sub(
            &src,
            "measurement = { node = \"receiving_tank\", variable = \"level\" }",
            "measurement = { node = \"receiving_tank\", variable = \"temperature\" }",
        );
        let src = sub(&src, "setpoint_m = 4.0", "setpoint_c = 20.0");
        sub(&src, "gain_per_m = 0.25", "gain_per_k = 0.25")
    };
    assert_refused(
        "a temperature over a flow",
        &temperature_over_flow,
        "has no coolant stream",
    );
}

// --------------------------------------- gate 9: the pairings the demo lacks

/// **Gate 9, the level pairing.** On `tank_level_control.toml`'s plant, a level
/// held through the flow on its DRAIN (the primary direct) and through the flow
/// on its FILL (reverse — the first reverse level loop on a valve, admitted
/// because the loader checked the side in one hop). Both start at 2 m and must
/// settle on 4 m; both must have moved their secondary's setpoint off its declared
/// 10 kg/s, or the inner loop alone would be doing the holding.
#[test]
fn a_level_is_held_through_a_drain_flow_and_through_a_fill_flow() {
    for (label, src, inner) in [
        ("drain", level_drain("action = \"direct\""), "drain_flow"),
        ("fill", level_fill("action = \"reverse\""), "fill_flow"),
    ] {
        let mut engine = build(&src);
        run(&mut engine, 6000);
        let level = tank_level_m(&engine, "receiving_tank");
        assert!(
            (level - 4.0).abs() < 0.01,
            "{label}: the cascade must hold 4 m (3.9924 / 3.9999 measured), got {level}"
        );
        let target = setpoint_flow(&engine, inner);
        assert!(
            (target - 10.0).abs() > 0.1,
            "{label}: the primary must have moved the flow target off its declared 10 kg/s, \
             got {target}"
        );
    }
}

/// **Gate 9, the cooler pairing.** The heater demo mirrored: an 80 °C feed, the
/// cooler's outlet loop DIRECT inside, the tank loop REVERSE outside over 55–80 °C.
/// A wrong outer sign is where this pairing would hide one (a reverse primary over
/// a direct secondary), so it runs rather than only loading. Hand: inside 0.06 K
/// of 60 °C from tick 2 580, the outer at a clamp on 69 ticks; measured: 2 580 and
/// 69, the outlet never below the 55 °C range bottom.
#[test]
fn the_cooler_pairing_settles_as_the_heater_demo_mirrored() {
    let mut engine = build(&cooler_cascade("action = \"reverse\""));
    let mut coldest_outlet = f64::MAX;
    let mut at_a_clamp = 0;
    let mut last_out = 0;
    for _ in 0..6000 {
        tick(&mut engine);
        if (node_c(&engine, "hold_tank") - TANK_SETPOINT_C).abs() > BAND_K {
            last_out = engine.snapshot().tick;
        }
        coldest_outlet = coldest_outlet.min(node_c(&engine, "chiller"));
        let u = faceplate(&engine, "tank_temperature").output;
        if u == 0.0 || u == 1.0 {
            at_a_clamp += 1;
        }
    }
    let settled = last_out + 1;
    assert!(
        (2530..=2630).contains(&settled),
        "the hand model puts the cooler cascade inside 0.06 K from tick 2 580; the engine \
         from {settled}"
    );
    assert!(
        at_a_clamp > 0,
        "the outer loop reaches its clamp (hand 69 ticks)"
    );
    assert!(
        coldest_outlet >= 55.0,
        "the outlet never runs colder than the range bottom: {coldest_outlet} °C"
    );
}

// ------------------------------------------------- gate 10: the wire form

/// **Gate 10. `drives` is on the primary's bytes and on no one else's.**
///
/// Asserted on the serialized faceplates, because a new plant has no corpus
/// baseline to move (M10.1's escape) and a Rust match passes under any key. The
/// six loop plants written before M25 must carry no `drives` key at all.
#[test]
fn only_a_primary_publishes_what_it_drives() {
    let engine = build(DEMO);
    let primary = serde_json::to_string(&faceplate(&engine, "tank_temperature")).unwrap();
    let secondary = serde_json::to_string(&faceplate(&engine, "outlet_temperature")).unwrap();
    assert!(primary.contains(r#""drives":1"#), "{primary}");
    assert!(!secondary.contains("drives"), "{secondary}");
    for json in [&primary, &secondary] {
        let back: ControlSnapshot = serde_json::from_str(json).expect("round trips");
        assert_eq!(&serde_json::to_string(&back).unwrap(), json);
    }
    for (name, src) in [
        ("furnace_outlet_control", OUTLET_LOOP),
        ("tank_temperature_heating", TANK_LOOP),
        ("tank_temperature_control", COOLER),
        ("tank_level_control", LEVEL),
        (
            "tank_flow_control",
            include_str!("../../../scenarios/tank_flow_control.toml"),
        ),
        (
            "vessel_pressure_control",
            include_str!("../../../scenarios/vessel_pressure_control.toml"),
        ),
    ] {
        let mut engine = build(src);
        tick(&mut engine);
        let json = serde_json::to_string(&engine.snapshot().controls).unwrap();
        assert!(!json.is_empty() && json != "[]", "{name} declares a loop");
        assert!(!json.contains("drives"), "{name}: {json}");
    }
}

// ------------------------------------------------- gate 11: a stall opens it

/// **Gate 11. A stall opens the cascade**, on the demo with a valve on its feed.
///
/// Shut the feed on a settled plant: from the next tick the outlet is stagnant,
/// the secondary has nothing to measure, and its primary is open. The tank's
/// target is raised to 62 °C while it is open; the primary writes nothing for the
/// whole stall, and when the flow returns the outlet target resumes where it
/// stood — re-seeded against the standing error, so its first closed tick is one
/// tick of control (hand 60.0 °C), not the 65 °C range top a primary that
/// integrated through the stall reaches (§29 mutation 13).
///
/// **The setpoint is raised one tick AFTER the feed is shut, and that order is a
/// correction to the hand probe** (§29, "Corrections from building it"). A loop
/// measures the previous tick's outlet, so the tick on which the flow stops is
/// still a CLOSED tick; raise the target in the same command batch and the
/// primary's proportional term acts on the 2 K step once — `K·Δe` = 0.27 of the
/// range, which clamps at 65 °C before the stall is ever seen. That is a setpoint
/// kick, not an open-cascade defect, and it is not what this gate is about.
#[test]
fn a_stall_opens_the_cascade_and_the_target_resumes_where_it_stood() {
    let mut engine = demo_after(&with_feed_valve(), 6000);
    set_valve(&mut engine, "feed_valve", 0.0);
    tick(&mut engine);
    set_setpoint_c(&mut engine, "tank_temperature", 62.0)
        .unwrap_or_else(|e| panic!("the primary's own setpoint: {e}"));
    let stood = setpoint_k(&engine, "outlet_temperature");
    for _ in 0..300 {
        tick(&mut engine);
        assert_eq!(
            faceplate(&engine, "outlet_temperature").measurement,
            None,
            "tick {}: no flow, so a stagnant outlet and nothing to measure",
            engine.snapshot().tick
        );
        assert_eq!(
            setpoint_k(&engine, "outlet_temperature"),
            stood,
            "tick {}: the primary writes nothing while its secondary cannot act",
            engine.snapshot().tick
        );
    }
    set_valve(&mut engine, "feed_valve", 1.0);
    // The first tick back still measures the stagnant outlet the stall left; the
    // second is the first the secondary can act on, and the cascade closes there.
    tick(&mut engine);
    assert_eq!(setpoint_k(&engine, "outlet_temperature"), stood);
    tick(&mut engine);
    assert!(faceplate(&engine, "outlet_temperature")
        .measurement
        .is_some());
    let moved = setpoint_k(&engine, "outlet_temperature") - stood;
    assert!(
        moved != 0.0 && moved.abs() < 0.1,
        "the outlet target must resume where it stood (hand 60.0 °C), one tick of control \
         away — not at the 65 °C top a primary that integrated through the stall reaches: \
         it moved {moved:+e} K from {} °C",
        stood - 273.15
    );
}
