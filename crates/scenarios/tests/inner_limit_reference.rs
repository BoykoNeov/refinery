//! M28: a cascade primary held by its secondary's limit (docs/DESIGN.md §31,
//! ledger row E16).
//!
//! When a secondary's own actuator sits at a limit, its primary may not move the
//! secondary's setpoint further in the direction that saturated it: nothing is
//! written, the primary's faceplate tracks, and its memory is back-calculated
//! against the held position. Moving the other way is untouched.
//!
//! No shipped plant reaches the rule — the demo's furnace never saturates, which
//! was E16's argument for deferring it — so every gate runs on a fixture derived
//! in-test from a shipped file, and each fixture is one row of the rule's
//! direction table: a REVERSE secondary at its top (a furnace too small for its
//! range), a DIRECT one at its top (a cooler too small), a reverse one at its
//! BOTTOM (a fire on the furnace), and the release out of a limit. The numbers in
//! bands are the hand model's (§31), which reproduces the shipped engine on the
//! unfixed rule to one tick; the unfixed engine's own numbers are quoted beside
//! them as the counterfactual each band excludes.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::ControlledValue;
use refinery_core::snapshot::{Command, ControlSnapshot};
use refinery_core::units::{Kelvin, Watt};
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/furnace_cascade_control.toml");
const COOLER: &str = include_str!("../../../scenarios/tank_temperature_control.toml");

const PRIMARY: &str = "tank_temperature";
const SECONDARY: &str = "outlet_temperature";
const TANK_SETPOINT_C: f64 = 60.0;
/// The band every settling claim in §29 and §31 is stated against.
const BAND_K: f64 = 0.06;

// ------------------------------------------------------------------- helpers

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the fixture must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("the fixture must build: {e}"))
}

fn tick(engine: &mut Engine) {
    let t = engine.snapshot().tick + 1;
    engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
}

/// Replace exactly one occurrence, or fail (the M19.1 lesson: a substitution
/// that lands nowhere tests the shipped file while claiming to test another).
fn sub(src: &str, from: &str, to: &str) -> String {
    assert_eq!(
        src.matches(from).count(),
        1,
        "the fixture's substitution must land exactly once: {from:?}"
    );
    src.replacen(from, to, 1)
}

fn faceplate(engine: &Engine, name: &str) -> ControlSnapshot {
    engine
        .snapshot()
        .controls
        .into_iter()
        .find(|c| c.name == name)
        .unwrap_or_else(|| panic!("no loop called '{name}'"))
}

fn setpoint_k(engine: &Engine) -> f64 {
    match faceplate(engine, SECONDARY).setpoint {
        ControlledValue::Temperature { k } => k.value(),
        other => panic!("the secondary holds a temperature, not {other:?}"),
    }
}

fn tank_c(engine: &Engine) -> f64 {
    engine
        .snapshot()
        .nodes
        .iter()
        .find(|n| n.name == "hold_tank")
        .expect("a tank")
        .temperature_k
        - 273.15
}

/// The primary's range map, in the order `SetpointRange::position` evaluates it.
fn position(setpoint_k: f64, min_c: f64, max_c: f64) -> f64 {
    let min = min_c + 273.15;
    (setpoint_k - min) / ((max_c + 273.15) - min)
}

/// The heater demo with a furnace of `max_duty_mw` — the inner loop's authority.
fn furnace(max_duty_mw: &str) -> String {
    sub(
        DEMO,
        "max_duty_mw = 2.0",
        &format!("max_duty_mw = {max_duty_mw}"),
    )
}

/// §29 gate 9's cooler pairing — an 80 °C feed, the outlet loop DIRECT inside,
/// the tank loop reverse outside over 55–80 °C — with a cooler of
/// `max_duty_mw`. At 1.3 MW the cooler reaches only 58.4 °C at the outlet, so
/// the range's bottom (55 °C) is past its authority.
fn cooler(max_duty_mw: &str) -> String {
    let plant = COOLER
        .split("\n[[controls]]\n")
        .next()
        .expect("a plant before its loop");
    format!(
        "{plant}
[[controls]]
name = \"{PRIMARY}\"
measurement = {{ node = \"hold_tank\", variable = \"temperature\" }}
actuator = {{ loop = \"{SECONDARY}\" }}
algorithm = \"pi\"
mode = \"auto\"
action = \"reverse\"
setpoint_c = 60.0
gain_per_k = 0.1331
integral_time_s = 600.0
initial_output = 0.6
range_min_c = 55.0
range_max_c = 80.0

[[controls]]
name = \"{SECONDARY}\"
measurement = {{ node = \"chiller\", variable = \"temperature\" }}
actuator = \"chiller\"
algorithm = \"pi\"
mode = \"auto\"
setpoint_c = 70.0
gain_per_k = 0.015
integral_time_s = 10.0
initial_output = 0.25
max_duty_mw = {max_duty_mw}
"
    )
}

/// What one tick did to the cascade, read around it.
struct Step {
    /// The secondary's output at the TOP of the tick — what the rule reads.
    inner_before: f64,
    setpoint_before: f64,
    setpoint_after: f64,
    /// The primary's faceplate after the tick.
    primary_output: f64,
}

fn step(engine: &mut Engine) -> Step {
    let inner_before = faceplate(engine, SECONDARY).output;
    let setpoint_before = setpoint_k(engine);
    tick(engine);
    Step {
        inner_before,
        setpoint_before,
        setpoint_after: setpoint_k(engine),
        primary_output: faceplate(engine, PRIMARY).output,
    }
}

/// The tick a run is inside `BAND_K` of `target_c` from, counted from `start`.
fn inside_from(trajectory: &[(u64, f64)], target_c: f64, start: u64) -> u64 {
    let last_out = trajectory
        .iter()
        .filter(|(_, c)| (c - target_c).abs() > BAND_K)
        .map(|(t, _)| *t)
        .max()
        .unwrap_or(start);
    last_out - start + 1
}

// --------------------------------- gate 1: a reverse secondary at its top

/// **Gate 1. A furnace too small for its range: the primary stops asking.**
///
/// The demo with a 1.3 MW furnace, whose outlet tops out at 61.6 °C under a
/// 40–65 °C range. Before M28 the primary walked the outlet target to the range
/// top while the furnace sat at full fire for 1 694 ticks, and then had to walk it
/// back: 0.395 K over 60 °C, inside 0.06 K from tick 4 445 (unfixed engine). With
/// the hold: 60.0038 °C, inside from 3 301, 331 ticks at full fire (hand); the
/// engine gives 3 302 and 332.
///
/// On every tick the furnace began at full fire the setpoint does not RISE (a
/// reverse secondary's output rises with its setpoint), and on every tick it is
/// held the primary's faceplate is the setpoint as a position, to the bit — the
/// open cascade's identity (§29 gate 6), which catches a hold that clamps the
/// written value but reports the one it did not write.
#[test]
fn a_furnace_too_small_for_its_range_holds_its_primary() {
    let mut engine = build(&furnace("1.3"));
    let mut trajectory = Vec::new();
    let mut held = 0;
    let mut at_full_fire = 0;
    for _ in 0..8000 {
        let s = step(&mut engine);
        let t = engine.snapshot().tick;
        trajectory.push((t, tank_c(&engine)));
        if faceplate(&engine, SECONDARY).output == 1.0 {
            at_full_fire += 1;
        }
        if s.inner_before == 1.0 {
            assert!(
                s.setpoint_after <= s.setpoint_before,
                "tick {t}: the furnace was at full fire and the primary raised its target \
                 {} -> {} K",
                s.setpoint_before,
                s.setpoint_after
            );
            if s.setpoint_after == s.setpoint_before {
                held += 1;
                assert_eq!(
                    s.primary_output,
                    position(s.setpoint_after, 40.0, 65.0),
                    "tick {t}: a held primary's faceplate is its secondary's setpoint"
                );
            }
        }
    }
    assert!(
        held > 100,
        "the hold must actually be reached: {held} ticks"
    );
    let peak = trajectory.iter().map(|(_, c)| *c).fold(f64::MIN, f64::max);
    assert!(
        peak - TANK_SETPOINT_C < 0.02,
        "hand 0.0038 K over 60 °C, unfixed 0.395 K: {:.4} K",
        peak - TANK_SETPOINT_C
    );
    let settled = inside_from(&trajectory, TANK_SETPOINT_C, 0);
    assert!(
        (3250..=3350).contains(&settled),
        "hand inside 0.06 K from tick 3 301, unfixed 4 445: {settled}"
    );
    assert!(
        (300..=370).contains(&at_full_fire),
        "hand 331 ticks at full fire, unfixed 1 694: {at_full_fire}"
    );
}

// ---------------------------------- gate 2: a direct secondary at its top

/// **Gate 2. A cooler too small for its range: the same hold, the other way.**
///
/// A cooler's outlet loop is DIRECT, so its duty FALLS as its setpoint rises: at
/// full duty the primary may not LOWER the setpoint. A rule that ignored the
/// secondary's action would block the raise instead and let the primary lower
/// the target into deeper saturation, which is what the first assertion sees.
/// Unfixed: 1 695 ticks at full duty, inside from 4 448. Engine with the hold:
/// 333 and 3 305 — the furnace's numbers mirrored.
#[test]
fn a_cooler_too_small_for_its_range_holds_its_primary_the_other_way() {
    let mut engine = build(&cooler("1.3"));
    let mut trajectory = Vec::new();
    let mut held = 0;
    for _ in 0..8000 {
        let s = step(&mut engine);
        let t = engine.snapshot().tick;
        trajectory.push((t, tank_c(&engine)));
        if s.inner_before == 1.0 {
            assert!(
                s.setpoint_after >= s.setpoint_before,
                "tick {t}: the cooler was at full duty and the primary LOWERED its target \
                 {} -> {} K",
                s.setpoint_before,
                s.setpoint_after
            );
            if s.setpoint_after == s.setpoint_before {
                held += 1;
            }
        }
    }
    assert!(
        held > 100,
        "the hold must actually be reached: {held} ticks"
    );
    let settled = inside_from(&trajectory, TANK_SETPOINT_C, 0);
    assert!(
        (3255..=3355).contains(&settled),
        "engine inside 0.06 K from tick 3 305, unfixed 4 448: {settled}"
    );
}

// ------------------------------- gate 3: a reverse secondary at its bottom

/// **Gate 3. A fire on the furnace drives it to zero: the hold at the BOTTOM.**
///
/// The demo settled, then 1.5 MW of fire on the heater for 2 000 ticks: the
/// outlet reaches 65 °C with the furnace cold, so its loop sits at zero and the
/// tank climbs. A reverse secondary at its bottom blocks a LOWER setpoint. Before
/// M28 the primary walked the target down to the range bottom (40 °C) and, when
/// the fire went out, the tank fell to 58.52 °C and took 2 485 ticks to come back
/// inside (unfixed engine; hand 58.52 and 2 484). A rule that held only at the
/// TOP fails this gate's first assertion.
///
/// **The hold LEAKS here, and the bands are the engine's, not the hand model's**
/// (§31, "Corrections from building it"). The flow through the furnace drifts
/// with the tank's level, so the cold furnace's outlet drifts too, and on a tick
/// it drifts toward its target the inner loop's output rises a hair off zero.
/// That tick the primary is free, and it steps the target down by one tick of
/// control. Held on 984 of the 2 000 ticks, the target ends at 52.37 °C — not
/// the hand model's 58.3 (constant flow, no drift), and not the unfixed 40. After
/// the fire: 0.074 K under 60 °C (hand 0.005, unfixed 1.48), back inside after
/// 1 832 ticks (hand 1 834). Row E19 carries the leak.
#[test]
fn a_fire_that_drives_the_furnace_to_zero_holds_its_primary_at_the_bottom() {
    let mut engine = build(DEMO);
    for _ in 0..6000 {
        tick(&mut engine);
    }
    let heater = engine.graph.find_node("heater").expect("a heater");
    let fire = |engine: &mut Engine, watts: f64| {
        engine
            .apply(Command::SetHeatInput {
                node: heater,
                power: Watt(watts),
            })
            .unwrap_or_else(|e| panic!("a fire is accepted: {e}"));
    };
    fire(&mut engine, 1.5e6);
    let mut held = 0;
    for _ in 0..2000 {
        let s = step(&mut engine);
        if s.inner_before == 0.0 {
            assert!(
                s.setpoint_after >= s.setpoint_before,
                "tick {}: the furnace was cold and the primary lowered its target {} -> {} K",
                engine.snapshot().tick,
                s.setpoint_before,
                s.setpoint_after
            );
            if s.setpoint_after == s.setpoint_before {
                held += 1;
            }
        }
    }
    assert!(
        held > 100,
        "the hold must actually be reached: {held} ticks"
    );
    let held_at = setpoint_k(&engine) - 273.15;
    assert!(
        (51.5..=53.5).contains(&held_at),
        "the target leaks down to 52.37 °C (engine; hand without the drift 58.3, unfixed \
         40, the range bottom): {held_at} °C"
    );

    fire(&mut engine, 0.0);
    let start = engine.snapshot().tick;
    let mut trajectory = Vec::new();
    for _ in 0..4000 {
        tick(&mut engine);
        trajectory.push((engine.snapshot().tick, tank_c(&engine)));
    }
    let coldest = trajectory.iter().map(|(_, c)| *c).fold(f64::MAX, f64::min);
    assert!(
        TANK_SETPOINT_C - coldest < 0.1,
        "after the fire, engine 0.074 K under 60 °C (hand 0.005), unfixed 1.48 K: {:.4} K",
        TANK_SETPOINT_C - coldest
    );
    let settled = inside_from(&trajectory, TANK_SETPOINT_C, start);
    assert!(
        (1782..=1882).contains(&settled),
        "engine back inside 0.06 K after 1 832 ticks (hand 1 834), unfixed 2 485: {settled}"
    );
}

// ---------------------------------------- gate 4: out of the limit at once

/// **Gate 4. The hold blocks one direction only.**
///
/// A 1.1 MW furnace cannot reach 60 °C at all (its outlet tops out at 58.3 °C),
/// so after 8 000 ticks it sits at full fire with its primary held. The tank's
/// setpoint is then stepped down to 55 °C: on that very tick the primary LOWERS
/// the target, out of the limit, though the furnace began the tick at full fire.
/// A hold blocking both directions would freeze it.
///
/// **What the hold costs, stated rather than hidden** (§31, "the release"): it
/// parks the primary's memory at the held position less the standing error's
/// proportional share, so the step's proportional kick lands lower than the
/// unfixed rule's, which parks it at the range top. The tank then dips to
/// 54.08 °C on a 55 °C target (hand), against the unfixed 54.87. The band
/// asserts that, so that a change to it is a decision.
#[test]
fn a_held_primary_moves_out_of_the_limit_at_once() {
    let mut engine = build(&furnace("1.1"));
    for _ in 0..8000 {
        tick(&mut engine);
    }
    assert_eq!(
        faceplate(&engine, SECONDARY).output,
        1.0,
        "premise: the furnace is at full fire"
    );
    let loop_id = faceplate(&engine, PRIMARY).id;
    engine
        .apply(Command::SetSetpoint {
            loop_id,
            value: ControlledValue::Temperature {
                k: Kelvin(55.0 + 273.15),
            },
        })
        .unwrap_or_else(|e| panic!("the tank's setpoint moves: {e}"));
    let s = step(&mut engine);
    assert_eq!(
        s.inner_before, 1.0,
        "the furnace began the tick at full fire"
    );
    assert!(
        s.setpoint_after < s.setpoint_before,
        "the primary moved its target OUT of the limit at once: {} -> {} K",
        s.setpoint_before,
        s.setpoint_after
    );
    let mut coldest = f64::MAX;
    for _ in 0..4000 {
        tick(&mut engine);
        coldest = coldest.min(tank_c(&engine));
    }
    assert!(
        (53.98..=54.18).contains(&coldest),
        "hand 54.08 °C (unfixed 54.87): {coldest:.4} °C"
    );
}
