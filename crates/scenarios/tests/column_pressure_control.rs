//! M89.1 — a column's pressure held by a loop on its condenser
//! (`"controlled"`, docs/DESIGN.md §86.1; plan
//! `docs/plans/column-pressure-from-physics.md`, 1b.1): the share of the
//! condenser's tubes free to condense the output of a PI pressure loop
//! (Kister, *Distillation Operation*, §17.2.2), the pressure a state either
//! side of the set.

use refinery_core::engine::Engine;
use refinery_core::graph::{ControlMode, LoopId};
use refinery_core::snapshot::Command;
use refinery_core::traits::ColumnPressureLevel;
use refinery_core::units::Kg;

use crate::column_drain::set_drain;
use crate::column_empty::{
    engine_of, holdup, holdup_with, reading, refused, run, state, state_mut, Books,
};
use crate::column_relief::{set_coolant, SET};

const OVERFILL_PLANT: &str = include_str!("../../../scenarios/crude_unit_overfill.toml");

/// The overfill plant with its condenser's `margin` and a PI pressure loop
/// on it, in `mode`, its tuning `gain_per_bar` and `integral_time_s`, its
/// free share starting at `1 / (1 + margin)` — the share that condenses the
/// settled duty at the set.
pub(crate) fn controlled(margin: f64, gain_per_bar: f64, integral_s: f64, mode: &str) -> String {
    let src = OVERFILL_PLANT.replace("\r\n", "\n");
    let old = "condenser_limit = { margin = 0.05,";
    assert!(src.contains(old), "the overfill plant's condenser");
    let src = src.replace(old, &format!("condenser_limit = {{ margin = {margin},"));
    format!(
        "{src}
[[controls]]
name = \"column_pressure\"
measurement = {{ node = \"column\", variable = \"pressure\" }}
actuator = \"column\"
algorithm = \"pi\"
mode = \"{mode}\"
setpoint_bar = 1.8
gain_per_bar = {gain_per_bar}
integral_time_s = {integral_s}
initial_output = {}
",
        1.0 / (1.0 + margin)
    )
}

/// The column's top pressure [Pa] as its condenser's faceplate reads it.
fn pressure(engine: &Engine) -> f64 {
    reading(&engine.snapshot()).condenser.unwrap().pressure_pa
}

/// Put the condenser's free share by hand.
fn free_share(engine: &mut Engine, share: f64) -> Result<(), String> {
    let node = engine.graph.find_node("column").unwrap();
    engine
        .apply(Command::SetCondenserFreeShare { node, share })
        .map_err(|e| e.to_string())
}

/// The tuning the tests run (DESIGN §86.1), from this plant's own answers.
/// Open loop, a free share 0.05 off its rest moves the pressure ~30 Pa a
/// tick at once — 0.06 bar/s a unit share, near an integrator — and keeps
/// going for minutes (11 kPa at three). Closed, 15 K warmer or colder water:
/// at 2 per bar and 60 s the pressure is still 50–100 Pa off after five
/// minutes; at 4 per bar and 30 s within ~15 Pa; at 8 per bar and 30 s within
/// 5 Pa in two minutes, no overshoot past a few pascals. The loop's own gain
/// a tick, 8 · 0.06 · 0.1 ≈ 0.05, is far below the 2 at which a discrete
/// integrator rings.
const GAIN: f64 = 8.0;
const INTEGRAL_S: f64 = 30.0;
/// The margin the tests' condenser carries: its share free at rest
/// `1/(1 + margin)` = 0.8, with reach both ways.
const MARGIN: f64 = 0.25;

/// The tests' plant in `mode`.
fn plant(mode: &str) -> String {
    controlled(MARGIN, GAIN, INTEGRAL_S, mode)
}

/// The pressure loop's faceplate output [-].
fn output(engine: &Engine) -> f64 {
    engine.snapshot().controls[0].output
}

/// `src` with `column_pressure = "<level>"` written into its `[fidelity]`.
fn at_level(src: &str, level: &str) -> String {
    src.replacen(
        "[fidelity]",
        &format!("[fidelity]\ncolumn_pressure = \"{level}\""),
        1,
    )
}

/// **A column's pressure loop is refused where it could not act, or would
/// act the wrong way**: on a column whose condenser is not a limit (no tubes
/// to flood); reverse acting (freeing tubes LOWERS the pressure); holding a
/// setpoint other than the column's own `pressure_bar`; with a duty range
/// on a condenser whose position is already a fraction. And
/// `"controlled"` is refused on a plant that declares no such loop.
#[test]
fn a_pressure_loop_is_refused_where_it_cannot_act() {
    let says = |src: &str, words: &[&str]| {
        let e = refused(src);
        for word in words {
            assert!(e.contains(word), "{word}: {e}");
        }
    };
    let src = plant("auto");
    let tail = src.split("[[controls]]").nth(1).unwrap().to_string();
    says(
        &format!("{}\n[[controls]]{tail}", holdup_with("")),
        &["column_pressure", "condenser_limit"],
    );
    says(
        &src.replace(
            "algorithm = \"pi\"",
            "algorithm = \"pi\"\naction = \"reverse\"",
        ),
        &["reverse", "direct acting"],
    );
    says(
        &src.replace("setpoint_bar = 1.8", "setpoint_bar = 1.9"),
        &["pressure_bar", "one"],
    );
    says(
        &src.replace(
            "algorithm = \"pi\"",
            "algorithm = \"pi\"\nmax_duty_mw = 5.0",
        ),
        &["max_duty_mw", "fraction"],
    );
    says(
        &at_level(OVERFILL_PLANT, "controlled"),
        &["column_pressure", "pressure loop"],
    );
    engine_of(&src.replace(
        "algorithm = \"pi\"",
        "algorithm = \"pi\"\naction = \"direct\"",
    ))
    .unwrap();
}

/// **Below `"controlled"` the loop is not built** (DESIGN §86.1): the plant
/// that declares it, run at `"limit"`, is the same plant without the loop —
/// every node, pipe and the column's state, to the bit — but for its
/// condenser saying what it declares and the level it ran at; it has no
/// loop on its faceplates, and the condenser's free share by hand is
/// refused, naming the level that builds the loop.
#[test]
fn below_controlled_the_loop_is_not_built() {
    let mut limited = engine_of(&at_level(&plant("auto"), "limit")).unwrap();
    let mut plain = engine_of(
        controlled(MARGIN, GAIN, INTEGRAL_S, "auto")
            .split("[[controls]]")
            .next()
            .unwrap(),
    )
    .unwrap();
    assert!(limited.snapshot().controls.is_empty());
    assert_eq!(
        holdup(&limited).condenser.unwrap().declared,
        ColumnPressureLevel::Controlled
    );
    assert_eq!(holdup(&limited).condenser.unwrap().free_share, 1.0);
    for tick in 0..20 {
        run(&mut limited, 1);
        run(&mut plain, 1);
        let ours = serde_json::to_string(&limited.snapshot())
            .unwrap()
            .replace(",\"declared\":\"controlled\"", "")
            .replace(",\"level\":\"limit\"", "");
        let theirs = serde_json::to_string(&plain.snapshot()).unwrap();
        assert_eq!(ours, theirs, "tick {tick}");
        assert_eq!(
            reading(&limited.snapshot()).condenser.unwrap().level,
            Some(ColumnPressureLevel::Limit)
        );
    }
    let e = free_share(&mut limited, 0.5).unwrap_err();
    assert!(e.contains("controlled"), "{e}");
}

/// **At rest the loop holds its set**: 60 s in AUTO, the column within 10 Pa
/// of 1.8 bar every tick (the search's tolerance is 1 Pa about the tick's
/// root; the loop trims the share as the plant settles) and the condenser's
/// share free at `1/(1 + margin)` within a hundredth. Read at load, before a
/// tick, the share is already the loop's `initial_output`: what a screen
/// shows of the plant it has just loaded.
#[test]
fn at_rest_the_loop_holds_its_set() {
    let mut engine = engine_of(&plant("auto")).unwrap();
    assert_eq!(
        holdup(&engine).condenser.unwrap().free_share,
        1.0 / (1.0 + MARGIN)
    );
    for tick in 1..=600 {
        run(&mut engine, 1);
        let p = pressure(&engine);
        assert!((p - SET).abs() <= 10.0, "tick {tick}: {p}");
        assert_eq!(reading(&engine.snapshot()).condenser.unwrap().level, None);
    }
    let u = output(&engine);
    assert!((u - 1.0 / (1.0 + MARGIN)).abs() < 0.01, "{u}");
    assert!((holdup(&engine).condenser.unwrap().free_share - u).abs() < 1e-12);
}

/// **The loop rides out warmer and colder cooling water** (the coolant put
/// 15 K either side of its 35 °C by hand, M89.2 makes it a command): warmer,
/// the condenser condenses less, the pressure rises and the loop frees
/// tubes; colder, the pressure falls BELOW its set — what M75's flooded
/// condenser could not do (fork B) — and the loop floods them. Either way
/// the pressure turns once and is back within 100 Pa of the set in five
/// minutes, never alternating tick to tick (sampled every tick: M75's
/// ringing hid behind even-tick samples, §80.2), and the books close every
/// tick. A loop acting the wrong way would run the pressure away instead.
#[test]
fn slow_the_loop_rides_out_warmer_and_colder_water() {
    for (shift, rises) in [(15.0, true), (-15.0, false)] {
        let mut engine = engine_of(&plant("auto")).unwrap();
        run(&mut engine, 100);
        let rest = output(&engine);
        let books = Books::of(&engine);
        set_coolant(&mut engine, 308.15 + shift);
        let (mut extreme, mut turns) = (SET, 0);
        let (mut worst_mass, mut worst_heat) = (0.0_f64, 0.0_f64);
        let mut last = (SET, 0.0_f64);
        for tick in 1..=3000 {
            let before = state(&engine);
            run(&mut engine, 1);
            let (mass, heat) = books.close(&engine, &before);
            worst_mass = worst_mass.max(mass.abs());
            worst_heat = worst_heat.max(heat.abs());
            let p = pressure(&engine);
            extreme = if rises {
                extreme.max(p)
            } else {
                extreme.min(p)
            };
            let step = p - last.0;
            if step * last.1 < 0.0 {
                turns += 1;
            }
            last = (p, if step == 0.0 { last.1 } else { step });
            if tick > 2700 {
                assert!((p - SET).abs() < 100.0, "{shift}: tick {tick}: {p}");
            }
        }
        let u = output(&engine);
        println!("M89_1 coolant {shift:+}: extreme {extreme:.0} Pa, turns {turns}, output {rest:.4} -> {u:.4}, books {worst_mass:.2e} kg/s {worst_heat:.2} W");
        if rises {
            assert!(
                extreme > SET + 1000.0 && u > rest,
                "{shift}: {extreme}, {u}"
            );
        } else {
            assert!(
                extreme < SET - 1000.0 && u < rest,
                "{shift}: {extreme}, {u}"
            );
        }
        assert!(turns <= 2, "{shift}: {turns} turns");
        assert!(worst_mass < 1e-8, "{shift}: {worst_mass} kg/s");
        assert!(worst_heat < 20.0, "{shift}: {worst_heat} W");
    }
}

/// **In MANUAL the share is the hand's**: refused by hand while the loop is
/// in AUTO; in MANUAL put at 0.75 it stays there, the pressure leaving its
/// set; back in AUTO the loop takes over from 0.75 without a bump and brings
/// the pressure back.
#[test]
fn in_manual_the_share_is_the_hands() {
    let mut engine = engine_of(&plant("auto")).unwrap();
    run(&mut engine, 50);
    let e = free_share(&mut engine, 0.75).unwrap_err();
    assert!(e.contains("AUTO"), "{e}");
    let mode = |engine: &mut Engine, mode| {
        engine
            .apply(Command::SetControllerMode {
                loop_id: LoopId(0),
                mode,
            })
            .unwrap()
    };
    mode(&mut engine, ControlMode::Manual);
    free_share(&mut engine, 0.75).unwrap();
    for tick in 1..=300 {
        run(&mut engine, 1);
        assert_eq!(
            holdup(&engine).condenser.unwrap().free_share,
            0.75,
            "tick {tick}"
        );
        assert_eq!(output(&engine), 0.75, "tick {tick}");
    }
    let off = pressure(&engine);
    assert!(off > SET + 1000.0, "{off}");
    mode(&mut engine, ControlMode::Auto);
    run(&mut engine, 1);
    let first = output(&engine);
    assert!((first - 0.75).abs() < 0.01, "a bump: {first}");
    run(&mut engine, 1500);
    assert!(pressure(&engine) < off, "{}", pressure(&engine));
}

/// **The loop and the drum's back-up flood one shell** (DESIGN §86.1): the
/// tubes under the higher of the two liquid levels are flooded, so the share
/// free is the lesser of the loop's and the back-up's — the capacity read
/// against `UA·(T_drum − T_coolant)·min(u, 1 − f)` by hand, with the loop's
/// level the higher and then the back-up's.
#[test]
fn the_loop_and_the_back_up_flood_one_shell() {
    const DRUM_M3: f64 = 18.7;
    for (share, backed) in [(0.6, 0.2), (0.6, 0.5)] {
        let mut engine = engine_of(&plant("manual")).unwrap();
        let now = reading(&run(&mut engine, 100)).drum_level;
        free_share(&mut engine, share).unwrap();
        set_drain(&mut engine, false).unwrap();
        let ua = state(&engine).condenser_ua.unwrap().value();
        let (d, p) = (0.019_f64, 0.0254_f64);
        let pi = std::f64::consts::PI;
        let per_area = (3.0f64.sqrt() / 2.0 * p * p - pi * d * d / 4.0) / (pi * d);
        let volume = ua / 850.0 * per_area;
        let held = state_mut(&mut engine);
        held.drum_liquid = Kg(held.drum_liquid.value() * (1.0 + backed * volume / DRUM_M3) / now);
        let s = run(&mut engine, 1);
        let read = reading(&s);
        let faceplate = read.condenser.unwrap();
        let f = (read.drum_level - 1.0) * DRUM_M3 / volume;
        assert!(
            (faceplate.flooded_share / f - 1.0).abs() < 1e-12,
            "{backed}"
        );
        let drum_t = state(&engine).drum_temperature.value();
        let capacity = share.min(1.0 - f) * ua * (drum_t - 308.15);
        assert!(
            (faceplate.capacity_w / capacity - 1.0).abs() < 1e-9,
            "{share}, {backed}: {} against {capacity}",
            faceplate.capacity_w
        );
    }
}

/// **The set against the water** (the plan's reference check for 1b.1,
/// DESIGN §86.1): at 1.8 bar a crude unit's condensate boils at about
/// 110–130 °C, far above 35 °C water, so the loop's set lies well above its
/// floor (the condensate's bubble pressure at the water's temperature). The
/// condenser is sized on the first tick by `UA = Q₀·(1 + margin)/(T_b −
/// T_coolant)` — its capacity then, read by hand, is `UA·(T_b − T_coolant)`
/// — and water warmer than the condensate's bubble point at the set, the set
/// below the floor, is refused there, naming both temperatures. Water a
/// little colder than the condensate is a set the loop can still reach: the
/// column held at it.
#[test]
fn the_set_is_refused_where_the_water_cannot_reach_it() {
    let mut engine = engine_of(&plant("auto")).unwrap();
    run(&mut engine, 1);
    let condensate = state(&engine).drum_temperature.value();
    assert!(
        (383.15..=403.15).contains(&condensate),
        "the condensate boils at {:.1} °C at the set",
        condensate - 273.15
    );
    let ua = state(&engine).condenser_ua.unwrap().value();
    let capacity = reading(&engine.snapshot()).condenser.unwrap().capacity_w;
    assert!((capacity / (ua * (condensate - 308.15)) - 1.0).abs() < 1e-12);

    let mut warm = engine_of(&plant("auto").replace("coolant_c = 35.0", "coolant_c = 250.0"))
        .expect("the water's temperature is read on the first tick");
    let e = warm.tick().unwrap_err().to_string();
    for word in ["cannot be sized", "116.6 °C", "250.0 °C"] {
        assert!(e.contains(word), "{word}: {e}");
    }

    let mut near =
        engine_of(&plant("auto").replace("coolant_c = 35.0", "coolant_c = 110.0")).unwrap();
    for tick in 1..=50 {
        run(&mut near, 1);
        assert!(
            (pressure(&near) - SET).abs() <= 10.0,
            "tick {tick}: {}",
            pressure(&near)
        );
    }
}
