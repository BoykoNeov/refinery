//! M89.0 — a column's pressure level switched (`[fidelity] column_pressure`,
//! docs/DESIGN.md §86; plan `docs/plans/column-pressure-from-physics.md`,
//! fork A): `"held"` pins a column at its set pressure whatever its
//! condenser declares, `"limit"` is M75's condenser that can run out. The
//! key is a ceiling: absent, each column runs at the highest level its own
//! declarations support; written, it may only lower one, and a level whose
//! keys no column declares is refused. A frontend's cap (the CLI's
//! `--column-pressure`) lowers the same way and is never refused.

use refinery_core::engine::Engine;
use refinery_core::snapshot::Command;
use refinery_core::traits::ColumnPressureLevel;
use refinery_scenarios::{build_engine, load_str};

use crate::column_empty::{engine_of, holdup_with, reading, refused, run};
use crate::column_relief::{set_coolant, vent, SET};

const OVERFILL_PLANT: &str = include_str!("../../../scenarios/crude_unit_overfill.toml");
/// A crude unit whose column holds no liquid, so declares no condenser.
const CRUDE_UNIT: &str = include_str!("../../../scenarios/crude_unit.toml");

/// `src` with `column_pressure = "<level>"` written into its `[fidelity]`.
fn at_level(src: &str, level: &str) -> String {
    assert!(src.contains("[fidelity]"), "the plant has a fidelity block");
    src.replacen(
        "[fidelity]",
        &format!("[fidelity]\ncolumn_pressure = \"{level}\""),
        1,
    )
}

/// `src` built with a frontend's cap on its columns' pressure level.
fn capped(src: &str, cap: &str) -> Result<Engine, String> {
    let mut file = load_str(src).map_err(|e| e.to_string())?;
    file.fidelity.column_pressure_cap = Some(cap.to_string());
    build_engine(&file).map_err(|e| e.to_string())
}

/// Cut (`on = false`) or restore the column's condenser cooling water.
fn cooling(engine: &mut Engine, on: bool) -> Result<(), String> {
    let node = engine.graph.find_node("column").unwrap();
    engine
        .apply(Command::SetCondenserCooling { node, on })
        .map_err(|e| e.to_string())
}

/// Every snapshot of `ticks` ticks, as the wire carries it.
fn wire(engine: &mut Engine, ticks: u32) -> Vec<String> {
    (0..ticks)
        .map(|tick| {
            engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
            serde_json::to_string(&engine.snapshot()).unwrap()
        })
        .collect()
}

/// **An unknown level is refused, naming the ones there are; a level no
/// column's declarations support is refused, naming the key it needs** —
/// `"limit"` on a plant whose columns declare no `condenser_limit`, on a
/// column that holds liquid and on one that holds none. `"held"` needs no
/// key, so it builds anywhere (lowering a plant with nothing to lower is
/// what it was already).
#[test]
fn a_level_no_column_declares_is_refused() {
    let e = refused(&at_level(OVERFILL_PLANT, "nonsense"));
    assert!(
        e.contains("nonsense") && e.contains("held") && e.contains("limit"),
        "{e}"
    );
    for plant in [holdup_with(""), CRUDE_UNIT.to_string()] {
        let e = refused(&at_level(&plant, "limit"));
        assert!(
            e.contains("column_pressure") && e.contains("condenser_limit"),
            "{e}"
        );
        engine_of(&at_level(&plant, "held")).unwrap();
    }
    engine_of(&at_level(OVERFILL_PLANT, "limit")).unwrap();
}

/// **A cap lowers and is never refused**: an unknown one is refused, but
/// `"limit"` capping a plant whose columns declare nothing builds — the
/// plant as it was, to the bit; `"held"` capping the overfill plant runs as
/// the file's own `"held"` does, to the bit; and a cap above the file's key
/// leaves the key's level.
#[test]
fn a_cap_only_lowers() {
    let Err(e) = capped(OVERFILL_PLANT, "nonsense") else {
        panic!("an unknown cap built");
    };
    assert!(e.contains("nonsense") && e.contains("held"), "{e}");
    let plain = holdup_with("");
    assert_eq!(
        wire(&mut capped(&plain, "limit").unwrap(), 3),
        wire(&mut engine_of(&plain).unwrap(), 3)
    );
    let held = wire(
        &mut engine_of(&at_level(OVERFILL_PLANT, "held")).unwrap(),
        3,
    );
    assert_eq!(wire(&mut capped(OVERFILL_PLANT, "held").unwrap(), 3), held);
    assert_eq!(
        wire(
            &mut capped(&at_level(OVERFILL_PLANT, "held"), "limit").unwrap(),
            3
        ),
        held
    );
}

/// **The default is the highest level the plant declares**: the overfill
/// plant with `column_pressure = "limit"` written runs as the shipped file
/// does, every snapshot to the bit, its faceplate saying nothing of a level
/// (the column runs at the one it declares).
#[test]
fn limit_written_is_the_overfill_plants_own() {
    let mut written = engine_of(&at_level(OVERFILL_PLANT, "limit")).unwrap();
    let mut shipped = engine_of(OVERFILL_PLANT).unwrap();
    assert_eq!(wire(&mut written, 20), wire(&mut shipped, 20));
    let condenser = reading(&shipped.snapshot()).condenser.unwrap();
    assert_eq!(condenser.level, None);
    assert!(!serde_json::to_string(&shipped.snapshot())
        .unwrap()
        .contains("\"level\""));
}

/// **Held, the column stands at its set pressure whatever its condenser
/// can do**: the coolant put far above the condensate's bubble point by
/// hand (the condenser condensing nothing), the limit plant's pressure
/// leaves its set within seconds while the held plant's stays on it to the
/// bit every tick, its relief shut, its faceplate reading `level = "held"`
/// — the keys it does not read, said so.
#[test]
fn slow_held_the_column_stands_at_its_set_pressure() {
    let mut held = engine_of(&at_level(OVERFILL_PLANT, "held")).unwrap();
    let mut limit = engine_of(OVERFILL_PLANT).unwrap();
    run(&mut held, 50);
    run(&mut limit, 50);
    set_coolant(&mut held, 2000.0);
    set_coolant(&mut limit, 2000.0);
    let mut risen = false;
    for tick in 1..=100 {
        let ours = run(&mut held, 1);
        let condenser = reading(&ours).condenser.unwrap();
        assert_eq!(condenser.pressure_pa, SET, "tick {tick}");
        assert_eq!(
            condenser.level,
            Some(ColumnPressureLevel::Held),
            "tick {tick}"
        );
        assert!(!condenser.at_capacity, "tick {tick}");
        assert_eq!(vent(&ours).flow_kg_s, 0.0, "tick {tick}");
        let theirs = reading(&run(&mut limit, 1)).condenser.unwrap();
        risen |= theirs.pressure_pa > SET + 1000.0;
    }
    assert!(risen, "the limit plant's pressure never left its set");
    assert!(serde_json::to_string(&held.snapshot())
        .unwrap()
        .contains("\"level\":\"held\""));
}

/// **A held column's cooling water is not a control**: cutting it would
/// change nothing, so it is refused, saying why and how to have it act; on
/// the limit plant it is taken.
#[test]
fn held_the_cooling_water_is_refused() {
    let mut held = engine_of(&at_level(OVERFILL_PLANT, "held")).unwrap();
    for on in [false, true] {
        let e = cooling(&mut held, on).unwrap_err();
        assert!(e.contains("held") && e.contains("column_pressure"), "{e}");
    }
    cooling(&mut engine_of(OVERFILL_PLANT).unwrap(), false).unwrap();
}
