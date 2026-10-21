//! M73.1 — a column's trays declared and their hydraulics reported
//! (docs/DESIGN.md §78.2, §78.3): the loop plant (`scenarios/crude_unit_loops.
//! toml`, untouched) with tray sections added inline — the slice's test
//! fixture, and M73.2's (`tray_pressure.rs`, each stage at its own pressure);
//! the new plant is M73.3's.
//!
//! The sections are sized as §78.2 says, offline with the probe's checked
//! script (`tools/steptime/prototypes/m73_1/`) on the file's settings: each
//! section's diameter its widest tray's at 80 % of flood, four passes, 0.61 m
//! spacing, a 50 mm weir, 5 mm holes at 10 % of the active area, downcomers
//! 12 % of the column, the apron 10 mm under the weir.

use refinery_core::engine::Engine;
use refinery_core::snapshot::{ColumnOperation, Command, Snapshot};
use refinery_scenarios::{build_engine, load_str};

pub(super) const PLANT: &str = include_str!("../../../scenarios/crude_unit_loops.toml");
const CASCADE_PLANT: &str = include_str!("../../../scenarios/crude_column_cascade.toml");
const ANCHOR: &str = "reflux_ratio = 4.0\nloops = [";
/// The loop plant's twelve trays in five sections, each loop's two stages
/// (3–4, 6–7, 9–10) in one: (from, to, diameter [m]).
const SECTIONS: [Section; 5] = [
    (1, 2, 6.1),
    (3, 5, 7.4),
    (6, 8, 8.3),
    (9, 10, 8.8),
    (11, 12, 8.3),
];

/// A section: its first and last stage, its diameter [m].
type Section = (u32, u32, f64);

/// One section's line, every number but the stages and the diameter the
/// fixture's.
fn section(from: u32, to: u32, diameter: f64) -> String {
    format!(
        "    {{ from = {from}, to = {to}, diameter_m = {diameter}, spacing_m = 0.61, \
         weir_height_m = 0.05, hole_diameter_m = 0.005, hole_area_share = 0.10, \
         downcomer_share = 0.12, clearance_m = 0.04, passes = 4 }},\n"
    )
}

fn trays_of(sections: &[Section]) -> String {
    let lines: String = sections
        .iter()
        .map(|(from, to, diameter)| section(*from, *to, *diameter))
        .collect();
    format!("trays = [\n{lines}]\n")
}

/// The loop plant with `trays` in its cascade block.
fn with_trays(trays: &str) -> String {
    let plant = PLANT.replace("\r\n", "\n");
    assert!(
        plant.contains(ANCHOR),
        "the loop plant's cascade block moved"
    );
    plant.replace(ANCHOR, &format!("reflux_ratio = 4.0\n{trays}loops = ["))
}

pub(super) fn fixture() -> String {
    with_trays(&trays_of(&SECTIONS))
}

pub(super) fn on_solver(src: &str, solver: &str) -> String {
    src.replace("flow = \"newton\"", &format!("flow = \"{solver}\""))
}

pub(super) fn engine_of(src: &str) -> Result<Engine, String> {
    let file = load_str(src).map_err(|e| e.to_string())?;
    build_engine(&file).map_err(|e| e.to_string())
}

pub(super) fn run(engine: &mut Engine, ticks: u32) -> Snapshot {
    for tick in 0..ticks {
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
    }
    engine.snapshot()
}

pub(super) fn operation(snapshot: &Snapshot) -> ColumnOperation {
    snapshot
        .nodes
        .iter()
        .find(|n| n.name == "column")
        .and_then(|n| n.column_operation.clone())
        .expect("the column reports its operation")
}

pub(super) fn set_reflux(engine: &mut Engine, reflux_ratio: f64) {
    let node = engine.graph.find_node("column").unwrap();
    engine
        .apply(Command::SetRefluxRatio { node, reflux_ratio })
        .unwrap();
}

/// The lists are the trays' own verdicts, and nothing else (§78.3): every
/// tray past jet flood or with its downcomer past its limit is named
/// flooding, every tray under its weep point weeping, and no other.
pub(super) fn assert_lists_agree(operation: &ColumnOperation) {
    let flooding: Vec<u32> = operation
        .trays
        .iter()
        .filter(|t| t.flood_fraction > 1.0 || t.downcomer_backup > 1.0)
        .map(|t| t.stage)
        .collect();
    let weeping: Vec<u32> = operation
        .trays
        .iter()
        .filter(|t| t.weep_margin < 1.0)
        .map(|t| t.stage)
        .collect();
    assert_eq!(operation.trays_flooding, flooding, "{operation:?}");
    assert_eq!(operation.trays_weeping, weeping, "{operation:?}");
}

/// **At the file's settings every tray works**: twelve trays, top first, each
/// inside its window — under 85 % of flood, its holes 1.2 times its weep
/// point or more, its downcomer under its limit — none named; the column's
/// pressure drop 100–150 mbar, what the checked set gave the probe's designs
/// (127 mbar, §78.4.1).
#[test]
fn the_trays_work_at_the_files_settings() {
    let mut engine = engine_of(&fixture()).expect("the fixture builds");
    let operation = operation(&run(&mut engine, 300));
    let stages: Vec<u32> = operation.trays.iter().map(|t| t.stage).collect();
    assert_eq!(stages, (1..=12).collect::<Vec<u32>>());
    for tray in &operation.trays {
        assert!(
            tray.flood_fraction > 0.5 && tray.flood_fraction < 0.85,
            "{tray:?}"
        );
        assert!(tray.weep_margin > 1.2, "{tray:?}");
        assert!(tray.downcomer_backup < 0.95, "{tray:?}");
        assert!(tray.pressure_drop_pa > 0.0, "{tray:?}");
    }
    assert!(operation.trays_flooding.is_empty() && operation.trays_weeping.is_empty());
    let drop_mbar: f64 = operation
        .trays
        .iter()
        .map(|t| t.pressure_drop_pa)
        .sum::<f64>()
        / 100.0;
    assert!((100.0..150.0).contains(&drop_mbar), "{drop_mbar} mbar");
}

/// **A loop's draw tray carries its circulation** (§78.4.1's loads): the
/// liquid leaving it is what goes down, what is drawn and what goes into the
/// loop — the circulation its return tray, one up and in the same section,
/// passed down to it. So the two trays' downcomers run within 5 % of each
/// other (measured 1–4 %); leave the loop's draw out and the draw tray's
/// liquid falls by the whole circulation.
#[test]
fn a_loops_draw_tray_carries_its_circulation() {
    let mut engine = engine_of(&fixture()).expect("the fixture builds");
    let operation = operation(&run(&mut engine, 300));
    for draw in [4, 7, 10] {
        let (ret, drawn) = (&operation.trays[draw - 2], &operation.trays[draw - 1]);
        assert!(
            (drawn.downcomer_backup / ret.downcomer_backup - 1.0).abs() < 0.05,
            "tray {draw} against {}: {drawn:?} {ret:?}",
            draw - 1
        );
    }
}

/// **The reflux takes the trays out of their window both ways, and the lists
/// say which** (§78.0 finding 3a, re-run in §78.4.1): at a reflux of 1 the
/// trays weep and none floods; at 6 they flood and none weeps. At every step
/// the lists are exactly the trays past their limits.
#[test]
fn the_reflux_floods_and_weeps_the_trays_by_name() {
    let mut engine = engine_of(&fixture()).expect("the fixture builds");
    run(&mut engine, 300);
    for reflux in [2.0, 1.0, 1.5, 3.0, 5.0, 6.0] {
        set_reflux(&mut engine, reflux);
        let operation = operation(&run(&mut engine, 100));
        assert_lists_agree(&operation);
        if reflux == 1.0 {
            assert!(operation.trays_flooding.is_empty(), "{operation:?}");
            assert!(operation.trays_weeping.len() >= 6, "{operation:?}");
        }
        if reflux == 3.0 {
            assert!(operation.trays_flooding.is_empty(), "{operation:?}");
            assert!(operation.trays_weeping.is_empty(), "{operation:?}");
        }
        if reflux == 6.0 {
            assert!(operation.trays_flooding.len() >= 6, "{operation:?}");
            assert!(operation.trays_weeping.is_empty(), "{operation:?}");
        }
    }
}

/// **A column without trays publishes no tray key**: the shipped plants'
/// bytes do not move (the corpus says so of every plant; this says why).
#[test]
fn a_column_without_trays_writes_no_tray_key() {
    let mut engine = engine_of(PLANT).unwrap();
    let json = serde_json::to_string(&run(&mut engine, 20)).unwrap();
    assert!(!json.contains("\"trays"), "a tray key on a plant with none");
}

fn refused(src: &str) -> String {
    match engine_of(src) {
        Ok(_) => panic!("built a plant that should have been refused"),
        Err(e) => e,
    }
}

/// **Sections that do not cover the trays, top down, once each, are refused
/// by name**, and so is a loop whose stages are in two sections (§78.2).
#[test]
fn sections_that_miss_a_tray_are_refused() {
    let cases: [(&[Section], &str); 4] = [
        (
            &[(1, 2, 6.1), (4, 12, 8.3)],
            "runs from stage 4 to 12; the sections run top down",
        ),
        (&[(1, 2, 6.1), (3, 11, 8.3)], "the sections end at stage 11"),
        (&[(1, 2, 6.1), (3, 13, 8.3)], "the trays end at stage 12"),
        (
            &[(1, 3, 6.1), (4, 12, 8.3)],
            "the pumparound loop drawn from stage 4 returns to stage 3 in another section",
        ),
    ];
    for (sections, says) in cases {
        let error = refused(&with_trays(&trays_of(sections)));
        assert!(error.contains("column 'column'"), "{error}");
        assert!(error.contains(says), "{sections:?}: {error}");
    }
}

/// **A number outside the range its correlation was checked over is refused
/// by name** (§78.4.1): each change below made to the first section only.
#[test]
fn a_tray_outside_its_correlations_is_refused() {
    let cases = [
        (
            "spacing_m = 0.61",
            "spacing_m = 1.0",
            "spacing_m = 1 is outside",
        ),
        (
            "weir_height_m = 0.05",
            "weir_height_m = 0.02",
            "weir_height_m = 0.02 is under",
        ),
        (
            "weir_height_m = 0.05",
            "weir_height_m = 0.1",
            "over 15 % of the tray spacing",
        ),
        (
            "hole_diameter_m = 0.005",
            "hole_diameter_m = 0.02",
            "hole_diameter_m = 0.02 is over",
        ),
        (
            "hole_area_share = 0.10",
            "hole_area_share = 0.08",
            "hole_area_share = 0.08 is outside",
        ),
        (
            "downcomer_share = 0.12",
            "downcomer_share = 0.5",
            "downcomer_share = 0.5 must be",
        ),
        (
            "clearance_m = 0.04",
            "clearance_m = 0.05",
            "clearance_m = 0.05 is not under the weir",
        ),
        ("passes = 4", "passes = 5", "passes = 5 is outside"),
        ("diameter_m = 6.1", "diameter_m = 0.0", "diameter_m is 0"),
        (
            "passes = 4",
            "passes = 4, plate_thickness_m = 0.005",
            "plate_thickness_m",
        ),
    ];
    for (from, to, says) in cases {
        let src = fixture().replacen(from, to, 1);
        assert_ne!(src, fixture(), "{from} not in the fixture");
        let error = refused(&src);
        assert!(error.contains(says), "{to}: {error}");
    }
}

/// **Trays on a column the heat balance does not solve are refused**: their
/// loads are its stage balances' (§78.2).
#[test]
fn trays_on_a_constant_overflow_column_are_refused() {
    let plant = CASCADE_PLANT.replace("\r\n", "\n");
    let anchor = "reflux_ratio = 2.0\n";
    assert!(plant.contains(anchor));
    let src = plant.replace(anchor, &format!("{anchor}{}", trays_of(&[(1, 7, 3.0)])));
    let error = refused(&src);
    assert!(error.contains("declares its trays"), "{error}");
}
