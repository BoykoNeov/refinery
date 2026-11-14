//! M75.4 — the overfill plant shipped (`scenarios/crude_unit_overfill.toml`,
//! docs/DESIGN.md §80.7): the holdup plant with its steel, the M75.1
//! condenser limit, the M75.2 drain and the M75.3 relief valve on the map —
//! exactly the plant M75's inline tests build (`column_relief::with_relief`),
//! its valve Kv 5 000, the M75.3.0 probe's book-rule figure (the user's
//! decision, 2026-11-14; where it comes from, DESIGN §80.7). These gates keep the file and the fixtures from
//! drifting apart, so every M75 test speaks for the shipped plant.

use std::collections::BTreeMap;

use refinery_core::engine::Engine;
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::units::Pascal;

use crate::column_empty::{engine_of, holdup, holdup_with, reading, run, state, Books, FLARE_PIPE};
use crate::column_relief::{
    close, set_coolant, vent, with_relief, DESIGN, INLET_PIPE, P_ATM, SET, X_T,
};

const OVERFILL_PLANT: &str = include_str!("../../../scenarios/crude_unit_overfill.toml");
/// A crude unit whose column holds no liquid.
const CRUDE_UNIT: &str = include_str!("../../../scenarios/crude_unit.toml");

/// The relief valve the plant declares: the M75.3.0 probe's book-rule
/// figure (DESIGN §80.4, §80.7).
const KV: f64 = 5000.0;

/// Every node's and pipe's readings by name, its id and ends taken off (the
/// file's nodes stand in another order than the fixture's, which shifts ids).
fn by_name(snapshot: &Snapshot) -> BTreeMap<String, serde_json::Value> {
    let mut named = BTreeMap::new();
    for node in &snapshot.nodes {
        let mut value = serde_json::to_value(node).unwrap();
        value.as_object_mut().unwrap().remove("id");
        named.insert(format!("node {}", node.name), value);
    }
    for edge in &snapshot.edges {
        let mut value = serde_json::to_value(edge).unwrap();
        let fields = value.as_object_mut().unwrap();
        for key in ["id", "from", "to"] {
            fields.remove(key);
        }
        named.insert(format!("edge {}", edge.name), value);
    }
    named
}

/// **The shipped plant is the fixture**: its condenser, its drain and its
/// relief declared as M75's inline tests declare them — the probe's
/// condenser (5 %, 35 °C, 100 m³), the book's valve (set at the design
/// pressure, full lift 10 % of it gauge above, `x_t` 0.7) of Kv 5 000 — and
/// over 150 ticks it runs as that fixture does, every node, pipe and the
/// column's state by name to `1e-6` (the valve's set and accumulation are
/// written to the bar's sixth digit in the file, the fixture's to the bit).
#[test]
fn the_shipped_plant_is_the_fixture() {
    let mut shipped = engine_of(OVERFILL_PLANT).unwrap();
    let mut fixture = engine_of(&with_relief(KV)).unwrap();
    let (ours, theirs) = (holdup(&shipped), holdup(&fixture));
    let (ours, theirs) = (ours.condenser.unwrap(), theirs.condenser.unwrap());
    assert_eq!(ours.margin, theirs.margin);
    assert_eq!(ours.coolant, theirs.coolant);
    assert_eq!(ours.overhead_volume, theirs.overhead_volume);
    assert_eq!(ours.set_pressure, Pascal(SET));
    assert_eq!(ours.design_pressure, theirs.design_pressure);
    let (ours, theirs) = (ours.relief.unwrap(), theirs.relief.unwrap());
    assert_eq!(ours.cv_max, KV / (3600.0 * 1e5_f64.sqrt()));
    assert_eq!(ours.cv_max, theirs.cv_max);
    assert_eq!(ours.x_t, X_T);
    assert!((ours.set_pressure.value() - DESIGN).abs() < 1e-6);
    assert!((ours.set_pressure.value() - theirs.set_pressure.value()).abs() < 1e-6);
    assert!((ours.accumulation.value() - 0.10 * (DESIGN - P_ATM)).abs() < 1e-6);
    assert_eq!(ours.ceiling, theirs.ceiling);
    assert!((ours.flare_pressure.value() - P_ATM).abs() < 1e-6);
    for tick in 0..150 {
        shipped
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        fixture
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        let as_value = |e: &Engine| serde_json::to_value(state(e)).unwrap();
        assert!(
            close(&as_value(&shipped), &as_value(&fixture)),
            "tick {tick}: the column's state"
        );
        let (ours, theirs) = (by_name(&shipped.snapshot()), by_name(&fixture.snapshot()));
        assert_eq!(
            ours.keys().collect::<Vec<_>>(),
            theirs.keys().collect::<Vec<_>>()
        );
        for (name, value) in &ours {
            assert!(
                close(value, &theirs[name]),
                "tick {tick}: {name}\n{value}\n{}",
                theirs[name]
            );
        }
    }
}

/// **At rest the shipped plant holds its set pressure, its relief shut**:
/// over 600 ticks the column stands at 1.8 bar exactly, the condenser within
/// its capacity, nothing backed up, the relief shut and its line to the
/// flare carrying exactly nothing, no alarm.
#[test]
fn at_rest_the_shipped_plant_holds_its_set_pressure() {
    let mut engine = engine_of(OVERFILL_PLANT).unwrap();
    for tick in 1..=600 {
        let snapshot = run(&mut engine, 1);
        let condenser = reading(&snapshot).condenser.unwrap();
        assert_eq!(condenser.pressure_pa, SET, "tick {tick}");
        assert!(!condenser.at_capacity, "tick {tick}");
        assert!(!condenser.at_design, "tick {tick}");
        assert!(!condenser.at_ceiling, "tick {tick}");
        assert_eq!(condenser.flooded_share, 0.0, "tick {tick}");
        for name in [INLET_PIPE, FLARE_PIPE] {
            let line = snapshot.edges.iter().find(|e| e.name == name).unwrap();
            assert_eq!(line.stream.mass_flow.value(), 0.0, "tick {tick}: {name}");
        }
        if tick > 1 {
            let shut = vent(&snapshot);
            assert_eq!(shut.opening, 0.0, "tick {tick}");
            assert_eq!(shut.flow_kg_s, 0.0, "tick {tick}");
        }
    }
}

/// Cut (`on = false`) or restore a column's condenser cooling water.
fn cooling(engine: &mut Engine, on: bool) -> Result<(), String> {
    let node = engine.graph.find_node("column").unwrap();
    engine
        .apply(Command::SetCondenserCooling { node, on })
        .map_err(|e| e.to_string())
}

/// The column's kind as the wire carries it.
fn kind_json(engine: &Engine) -> String {
    let snapshot = engine.snapshot();
    let node = snapshot.nodes.iter().find(|n| n.name == "column").unwrap();
    serde_json::to_string(&node.kind).unwrap()
}

/// **The cooling is cut only where a condenser can run out**: refused on a
/// node that is no column, on a column that holds no liquid, and on one
/// whose condenser is not a limit — each by name; on the overfill plant cut
/// it reads on the column's kind, restored it is off the wire again.
#[test]
fn the_cooling_is_cut_only_where_a_condenser_can_run_out() {
    let mut shipped = engine_of(OVERFILL_PLANT).unwrap();
    let source = shipped.graph.find_node("crude_source").unwrap();
    let e = shipped
        .apply(Command::SetCondenserCooling {
            node: source,
            on: false,
        })
        .unwrap_err()
        .to_string();
    assert!(e.contains("is not a column"), "{e}");
    let e = cooling(&mut engine_of(CRUDE_UNIT).unwrap(), false).unwrap_err();
    assert!(e.contains("holds no liquid"), "{e}");
    let e = cooling(&mut engine_of(&holdup_with("")).unwrap(), false).unwrap_err();
    assert!(e.contains("declares no condenser limit"), "{e}");
    // Restoring is refused there too: nothing cools a condenser that is no
    // limit.
    let e = cooling(&mut engine_of(&holdup_with("")).unwrap(), true).unwrap_err();
    assert!(e.contains("declares no condenser limit"), "{e}");

    assert!(!kind_json(&shipped).contains("coolant_lost"));
    cooling(&mut shipped, false).unwrap();
    assert!(holdup(&shipped).condenser.unwrap().coolant_lost);
    assert!(kind_json(&shipped).contains("\"coolant_lost\":true"));
    cooling(&mut shipped, true).unwrap();
    assert!(!holdup(&shipped).condenser.unwrap().coolant_lost);
    assert!(!kind_json(&shipped).contains("coolant_lost"));
}

/// **Cutting the cooling is the tests' own hand, to the bit**: M75.3's
/// stories lost the coolant by putting its temperature far above the
/// condensate's bubble point (`column_relief::set_coolant`), so the
/// condenser condensed nothing; the command does the same, and every node,
/// pipe and the column's state read exactly as that hand's — 90 s cut, the
/// relief lifting, then 30 s restored — but the column's kind, which
/// carries how.
#[test]
fn slow_cutting_the_cooling_is_the_tests_own_hand() {
    let mut commanded = engine_of(&with_relief(KV)).unwrap();
    let mut handled = engine_of(&with_relief(KV)).unwrap();
    run(&mut commanded, 100);
    run(&mut handled, 100);
    let coolant = holdup(&handled).condenser.unwrap().coolant.value();
    cooling(&mut commanded, false).unwrap();
    set_coolant(&mut handled, 2000.0);
    let without_kind = |engine: &Engine| {
        let mut named = by_name(&engine.snapshot());
        named
            .get_mut("node column")
            .unwrap()
            .as_object_mut()
            .unwrap()
            .remove("kind");
        named
    };
    let mut opened = false;
    for tick in 0..1200 {
        if tick == 900 {
            cooling(&mut commanded, true).unwrap();
            set_coolant(&mut handled, coolant);
        }
        commanded
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        handled
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        assert_eq!(
            serde_json::to_value(state(&commanded)).unwrap(),
            serde_json::to_value(state(&handled)).unwrap(),
            "tick {tick}: the column's state"
        );
        assert_eq!(
            without_kind(&commanded),
            without_kind(&handled),
            "tick {tick}"
        );
        opened |= vent(&commanded.snapshot()).flow_kg_s > 0.0;
    }
    // The story went somewhere: the pressure rose and the relief vented.
    assert!(opened, "the relief never opened");
}

/// **The shipped plant rides out its cooling lost** — the governing case its
/// relief is sized for (Kister §9.3), the screen's key: 5 minutes cut, then
/// 3 restored. The pressure leaves its set at once and reaches the relief's
/// set, 3.52 bar, in about a minute (57.4 s measured); the valve holds it
/// inside its band, below full lift (3.77 bar) — 3.65 bar at most, ~72 kg/s
/// to the flare two minutes in — never at the ceiling; the flare's pipe
/// carries the vent to the bit. Restored, the condenser takes the pressure
/// back: the relief shuts, and the column is at 1.8 bar exactly before the
/// end. The books close every tick.
#[test]
fn slow_the_shipped_plant_rides_out_its_cooling_lost() {
    let mut engine = engine_of(OVERFILL_PLANT).unwrap();
    run(&mut engine, 600);
    let books = Books::of(&engine);
    let full_lift = DESIGN + 0.10 * (DESIGN - P_ATM);
    cooling(&mut engine, false).unwrap();
    let (mut peak, mut vent_max, mut first_relief) = (0.0_f64, 0.0_f64, None);
    let (mut worst_mass, mut worst_heat) = (0.0_f64, 0.0_f64);
    for tick in 1..=4800 {
        if tick == 3001 {
            cooling(&mut engine, true).unwrap();
        }
        let before = state(&engine);
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        let (mass, heat) = books.close(&engine, &before);
        worst_mass = worst_mass.max(mass.abs());
        worst_heat = worst_heat.max(heat.abs());
        let snapshot = engine.snapshot();
        let condenser = reading(&snapshot).condenser.unwrap();
        let relief = vent(&snapshot);
        let flare = snapshot
            .edges
            .iter()
            .find(|e| e.name == FLARE_PIPE)
            .unwrap();
        assert_eq!(
            flare.stream.mass_flow.value(),
            relief.flow_kg_s,
            "tick {tick}"
        );
        assert!(!condenser.at_ceiling, "tick {tick}");
        if tick <= 3000 {
            assert_eq!(
                condenser.capacity_w, 0.0,
                "tick {tick}: cut, it condenses nothing"
            );
            assert!(condenser.pressure_pa > SET, "tick {tick}");
        }
        if condenser.at_design && first_relief.is_none() {
            first_relief = Some(tick);
        }
        peak = peak.max(condenser.pressure_pa);
        vent_max = vent_max.max(relief.flow_kg_s);
        if tick > 4200 {
            assert_eq!(condenser.pressure_pa, SET, "tick {tick}: back at its set");
            assert_eq!(relief.flow_kg_s, 0.0, "tick {tick}");
        }
    }
    let first_relief = first_relief.expect("the relief lifted") as f64 * 0.1;
    assert!((50.0..65.0).contains(&first_relief), "{first_relief} s");
    assert!(peak > DESIGN && peak < full_lift, "{peak}");
    assert!((peak / 1e5 - 3.65).abs() < 0.01, "{peak}");
    assert!((vent_max - 72.0).abs() < 2.0, "{vent_max}");
    assert!(worst_mass < 1e-9, "{worst_mass} kg/s");
    assert!(worst_heat < 20.0, "{worst_heat} W");
}

/// The holdup plant's key box at one corner (M74.4, DESIGN §79.8): the
/// furnace [MW], the reflux, the reboiler held [MW] or free, the loops'
/// flow as a share of the file's and their return [K] from the file's.
fn to_corner(
    engine: &mut Engine,
    furnace: f64,
    reflux: f64,
    reboiler: Option<f64>,
    loops: (f64, f64),
) {
    use refinery_core::units::{Kelvin, KgPerSec, Watt};
    let column = engine.graph.find_node("column").unwrap();
    let heater = engine.graph.find_node("furnace").unwrap();
    let mut send = |command: Command| engine.apply(command).unwrap();
    send(Command::SetFurnaceDuty {
        node: heater,
        duty: Watt(furnace * 1e6),
    });
    send(Command::SetRefluxRatio {
        node: column,
        reflux_ratio: reflux,
    });
    send(Command::SetReboilerDuty {
        node: column,
        duty: reboiler.map(|mw| Watt(mw * 1e6)),
    });
    // The file's loops: their flows [kg/s] and returns [°C].
    for (i, (flow, ret)) in [(85.0, 150.0), (95.0, 215.0), (65.0, 270.0)]
        .into_iter()
        .enumerate()
    {
        send(Command::SetLoopFlow {
            node: column,
            pumparound_loop: i as u32,
            flow: KgPerSec(flow * loops.0),
        });
        send(Command::SetLoopReturnTemperature {
            node: column,
            pumparound_loop: i as u32,
            temperature: Kelvin(ret + 273.15 + loops.1),
        });
    }
}

/// **A jump to the key box's corner runs while the pressure moves**: the
/// furnace at 40 MW, the reflux at 15, the reboiler free, the loops stopped
/// and returning 60 K warm — a corner the holdup plant runs (M74.4) — sends
/// the condenser past its capacity, and as the pressure rises the bottom
/// stage nearly stops boiling: a trial pressure inside the bracket stalls
/// the column's step. Refused 11 ticks in before the fix (docs/DESIGN.md
/// §80.7); it runs.
#[test]
fn a_jump_to_the_key_boxs_corner_runs_while_the_pressure_moves() {
    let mut engine = engine_of(OVERFILL_PLANT).unwrap();
    run(&mut engine, 600);
    to_corner(&mut engine, 40.0, 15.0, None, (0.0, 60.0));
    for tick in 1..=600 {
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
    }
}

/// **A jump to the key box's coldest corner runs while the pressure
/// moves**: the furnace out, the reflux at 0.5, the reboiler held at
/// nothing, the loops stopped and returning 60 K warm — a corner the holdup
/// plant runs (M74.4) — the pressure climbing ~25 Pa a tick; 50 s on, the
/// bracket's first trial a few hundred pascals up stalls the column's step,
/// where the answer a few pascals up does not. Refused at tick 499 before the
/// fix (docs/DESIGN.md §80.7); it runs.
#[test]
fn slow_a_jump_to_the_key_boxs_coldest_corner_runs_while_the_pressure_moves() {
    let mut engine = engine_of(OVERFILL_PLANT).unwrap();
    run(&mut engine, 600);
    to_corner(&mut engine, 0.0, 0.5, Some(0.0), (0.0, 60.0));
    for tick in 1..=600 {
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
    }
}

/// **A walk whose falsi trial and the bracket's midpoint both stall runs**:
/// one of the random walks measured for key 9's range (DESIGN §80.7, seed 16
/// of `tools/steptime/prototypes/m75_4/`), replayed — a key a minute from
/// tick 600: total reflux on, the reflux to 9.5, the loops' returns +20 K
/// then back, the reflux to 10, the loops' flow to 175 %, the furnace to
/// 25 MW, total reflux off and on again, the cooling cut. 17 ticks after the
/// cut a falsi trial stalls the column's step, and so does the bracket's
/// midpoint — a band of trial pressures that fail, the answer below it.
/// Refused there with a midpoint-only retreat; the retreat halfway towards
/// each end in turn finds a trial that converges, and the walk runs on.
#[test]
fn slow_a_walk_whose_falsi_and_midpoint_both_stall_runs() {
    use refinery_core::units::{Kelvin, KgPerSec, Watt};
    let mut engine = engine_of(OVERFILL_PLANT).unwrap();
    let column = engine.graph.find_node("column").unwrap();
    let heater = engine.graph.find_node("furnace").unwrap();
    let returns = |dk: f64| -> Vec<Command> {
        [150.0, 215.0, 270.0]
            .into_iter()
            .enumerate()
            .map(|(i, c)| Command::SetLoopReturnTemperature {
                node: column,
                pumparound_loop: i as u32,
                temperature: Kelvin(c + 273.15 + dk),
            })
            .collect()
    };
    let minutes: Vec<Vec<Command>> = vec![
        vec![Command::SetTotalReflux {
            node: column,
            on: true,
        }],
        vec![Command::SetRefluxRatio {
            node: column,
            reflux_ratio: 9.5,
        }],
        returns(20.0),
        returns(0.0),
        vec![Command::SetRefluxRatio {
            node: column,
            reflux_ratio: 10.0,
        }],
        [85.0, 95.0, 65.0]
            .into_iter()
            .enumerate()
            .map(|(i, f)| Command::SetLoopFlow {
                node: column,
                pumparound_loop: i as u32,
                flow: KgPerSec(f * 1.75),
            })
            .collect(),
        vec![Command::SetFurnaceDuty {
            node: heater,
            duty: Watt(25.0e6),
        }],
        vec![Command::SetTotalReflux {
            node: column,
            on: false,
        }],
        vec![Command::SetTotalReflux {
            node: column,
            on: true,
        }],
        vec![Command::SetCondenserCooling {
            node: column,
            on: false,
        }],
    ];
    run(&mut engine, 600);
    for (minute, commands) in minutes.into_iter().enumerate() {
        for command in commands {
            engine.apply(command).unwrap();
        }
        for tick in 1..=600 {
            engine
                .tick()
                .unwrap_or_else(|e| panic!("minute {minute}, tick {tick}: {e}"));
        }
    }
}
