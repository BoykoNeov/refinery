//! M74.1 — a column's holdup declared, and its state filled on its first tick
//! (docs/DESIGN.md §79.2): the tray plant (`scenarios/crude_unit_trays.toml`,
//! untouched) with a holdup block added inline. The column is still
//! quasi-steady in this slice: the state is filled from the first tick's
//! settled loads and carried unchanged, and nothing else the plant does moves.

use refinery_core::engine::Engine;
use refinery_core::graph::{HoldupState, NodeKind};
use refinery_core::snapshot::Snapshot;
use refinery_scenarios::{build_engine, load_str};

const TRAY_PLANT: &str = include_str!("../../../scenarios/crude_unit_trays.toml");
const LOOP_PLANT: &str = include_str!("../../../scenarios/crude_unit_loops.toml");
const CASCADE_PLANT: &str = include_str!("../../../scenarios/crude_column_cascade.toml");
const WET_PLANT: &str = include_str!("../../../scenarios/crude_unit_wet.toml");
const STRIPPED_PLANT: &str = include_str!("../../../scenarios/crude_unit_stripped.toml");

/// The fixture's holdup: the drum and the base as M74.4 will size them, near
/// enough — the numbers matter here only through the half-full start.
const HOLDUP: &str = "holdup = { drum_m3 = 20.0, base_m3 = 60.0, reboiler = \"held\" }";

/// `src` with `line` added to its column's cascade block, after the block's
/// `reflux_ratio`.
fn with_line(src: &str, line: &str) -> String {
    let src = src.replace("\r\n", "\n");
    let block = src
        .find("[nodes.column.cascade]")
        .expect("the plant has a cascade block");
    let at = block
        + src[block..]
            .find("reflux_ratio = ")
            .expect("the cascade block declares its reflux");
    let end = at + src[at..].find('\n').expect("a line end");
    format!("{}\n{line}{}", &src[..end], &src[end..])
}

fn fixture() -> String {
    with_line(TRAY_PLANT, HOLDUP)
}

fn engine_of(src: &str) -> Result<Engine, String> {
    let file = load_str(src).map_err(|e| e.to_string())?;
    build_engine(&file).map_err(|e| e.to_string())
}

fn run(engine: &mut Engine, ticks: u32) -> Snapshot {
    for tick in 0..ticks {
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
    }
    engine.snapshot()
}

fn state(engine: &Engine) -> Option<HoldupState> {
    let node = engine.graph.find_node("column").unwrap();
    match &engine.graph.node(node).kind {
        NodeKind::Column {
            cascade: Some(spec),
            ..
        } => spec.holdup.as_ref().and_then(|h| h.state.clone()),
        _ => panic!("the column is a cascade column"),
    }
}

fn refused(src: &str) -> String {
    match engine_of(src) {
        Ok(_) => panic!("built a plant that should have been refused"),
        Err(e) => e,
    }
}

/// Each tray's liquid at the file's settings [kg], M74.1.0's table
/// (`tools/steptime/prototypes/m74_1_0/levels_AB_base5.txt`): the heads the engine
/// recorded, summed offline in Python — after 3 000 ticks, where this test
/// reads the first.
const PROBE_TRAYS_KG: [f64; 12] = [
    1946.0, 1996.0, 3494.0, 3508.0, 2816.0, 4631.0, 4739.0, 3899.0, 4778.0, 4669.0, 3287.0, 5015.0,
];

/// **Declared, the state is empty until the first tick fills it**, and then
/// it holds what the column holds: each tray its probe's offline sum to
/// 0.5 % (measured: under 0.03 % on ten trays, 0.3–0.4 % on the feed tray and
/// the one under it, whose loads move between the first tick and the
/// 3 000th); the base
/// and the drum half their volume of a hot liquid (500–900 kg/m³); every
/// composition whole; the drum's the distillate's, the naphtha draw's, to the
/// bit, and the kerosene tray's (stage 4) the kerosene draw's.
#[test]
fn a_column_that_holds_liquid_starts_full() {
    let mut engine = engine_of(&fixture()).unwrap();
    assert_eq!(state(&engine), None, "a state before the first tick");
    let snapshot = run(&mut engine, 1);
    let held = state(&engine).expect("the first tick fills the state");
    assert_eq!(held.stage_liquid.len(), 13);
    assert_eq!(held.stage_composition.len(), 13);
    for (j, (liquid, probe)) in held.stage_liquid[..12]
        .iter()
        .zip(PROBE_TRAYS_KG)
        .enumerate()
    {
        let kg = liquid.value();
        assert!(
            (kg / probe - 1.0).abs() < 0.005,
            "tray {}: {kg} kg against {probe}",
            j + 1
        );
    }
    let base_density = held.stage_liquid[12].value() / (0.5 * 60.0);
    let drum_density = held.drum_liquid.value() / (0.5 * 20.0);
    for density in [base_density, drum_density] {
        assert!((500.0..900.0).contains(&density), "{density} kg/m³");
    }
    for composition in held
        .stage_composition
        .iter()
        .chain(std::iter::once(&held.drum_composition))
    {
        let sum: f64 = composition.fractions().iter().sum();
        assert!((sum - 1.0).abs() < 1e-12, "{sum}");
    }
    let drawn = |tank: &str| {
        let tank = engine.graph.find_node(tank).unwrap();
        let edge = snapshot
            .edges
            .iter()
            .find(|e| e.to == tank)
            .expect("a draw line into the tank");
        edge.stream.composition.clone()
    };
    assert_eq!(held.drum_composition, drawn("naphtha_tank"));
    assert_eq!(held.stage_composition[3], drawn("kerosene_tank"));
}

/// **In this slice the state is carried as filled**: nothing moves it after
/// the first tick.
#[test]
fn the_state_is_carried_unchanged() {
    let mut engine = engine_of(&fixture()).unwrap();
    run(&mut engine, 1);
    let first = state(&engine).unwrap();
    run(&mut engine, 30);
    assert_eq!(state(&engine).unwrap(), first);
}

/// **Holdup moves nothing else**: thirty ticks with and without it, the
/// snapshots equal to the bit once the column's holdup is taken out.
#[test]
fn holdup_moves_nothing_else() {
    let strip = |snapshot: &Snapshot| {
        let mut json = serde_json::to_value(snapshot).unwrap();
        for node in json["nodes"].as_array_mut().unwrap() {
            if let Some(cascade) = node["kind"]["cascade"].as_object_mut() {
                cascade.remove("holdup");
            }
        }
        json
    };
    let with = run(&mut engine_of(&fixture()).unwrap(), 30);
    let without = run(&mut engine_of(TRAY_PLANT).unwrap(), 30);
    assert!(serde_json::to_string(&with).unwrap().contains("\"holdup\""));
    assert_eq!(strip(&with), strip(&without));
}

/// **A column without holdup writes no holdup key**: the shipped plants'
/// bytes do not move.
#[test]
fn a_column_without_holdup_writes_no_holdup_key() {
    let json = serde_json::to_string(&run(&mut engine_of(TRAY_PLANT).unwrap(), 2)).unwrap();
    assert!(
        !json.contains("holdup"),
        "a holdup key on a plant with none"
    );
}

/// **Refused by name where it cannot run yet** (§79.2): a column without
/// trays, a wet one, a steamed one, one not on the heat-balance cascade; a
/// reboiler neither held nor free; a held reboiler declared twice; a volume
/// that is not a positive number; a key the column does not read.
#[test]
fn holdup_is_refused_where_it_cannot_run() {
    let cases = [
        (
            with_line(LOOP_PLANT, HOLDUP),
            "holdup: the column declares no trays",
        ),
        (
            with_line(WET_PLANT, HOLDUP),
            "holdup: the column draws free water",
        ),
        (
            with_line(STRIPPED_PLANT, HOLDUP),
            "declares holdup and steam",
        ),
        (
            with_line(CASCADE_PLANT, HOLDUP),
            "declares holdup, but [fidelity] separation is \"cascade\"",
        ),
        (
            with_line(TRAY_PLANT, &HOLDUP.replace("\"held\"", "\"auto\"")),
            "holdup.reboiler = \"auto\"",
        ),
        (
            with_line(TRAY_PLANT, &format!("{HOLDUP}\nreboiler_duty_mw = 80.0")),
            "declares both reboiler_duty_mw and holdup.reboiler",
        ),
        (
            with_line(
                TRAY_PLANT,
                &HOLDUP.replace("drum_m3 = 20.0", "drum_m3 = 0.0"),
            ),
            "drum_m3 = 0; a working volume is a positive number",
        ),
        (
            with_line(
                TRAY_PLANT,
                &HOLDUP.replace("base_m3 = 60.0", "base_m3 = -1.0"),
            ),
            "base_m3 = -1; a working volume is a positive number",
        ),
        (
            with_line(
                TRAY_PLANT,
                &HOLDUP.replace("base_m3 = 60.0", "base_m3 = nan"),
            ),
            "base_m3 = NaN; a working volume is a positive number",
        ),
        (
            with_line(TRAY_PLANT, &HOLDUP.replace(" }", ", sump_m3 = 5.0 }")),
            "unknown field `sump_m3`",
        ),
    ];
    for (src, says) in cases {
        let error = refused(&src);
        assert!(error.contains(says), "expected {says:?}: {error}");
    }
}

/// **The file's words land where they say**: `"held"` and `"free"` start the
/// column held and free, and each volume is its own.
#[test]
fn the_declaration_is_read_as_written() {
    for (word, held) in [("held", true), ("free", false)] {
        let src = with_line(
            TRAY_PLANT,
            &HOLDUP.replace("\"held\"", &format!("\"{word}\"")),
        );
        let engine = engine_of(&src).unwrap();
        let node = engine.graph.find_node("column").unwrap();
        let NodeKind::Column {
            cascade: Some(spec),
            ..
        } = &engine.graph.node(node).kind
        else {
            panic!("the column is a cascade column");
        };
        let holdup = spec.holdup.as_ref().expect("the holdup is declared");
        assert_eq!(holdup.start_held, held, "{word}");
        assert_eq!(holdup.drum_volume.value(), 20.0);
        assert_eq!(holdup.base_volume.value(), 60.0);
        assert_eq!(spec.reboiler_duty, None, "{word}");
    }
}
