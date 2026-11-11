//! M74.6 — a column's steel, and the start from empty (docs/DESIGN.md §79.16):
//! the holdup plant started empty and cold (`scenarios/crude_unit_empty.toml`),
//! and the holdup plant (`crude_unit_holdup.toml`, untouched) with its steel
//! declared inline.

use refinery_core::components::Composition;
use refinery_core::engine::Engine;
use refinery_core::graph::{ColumnHoldup, ColumnSteel, EdgeId, HoldupState, NodeKind};
use refinery_core::snapshot::Snapshot;
use refinery_core::traits::LevelLimit;
use refinery_core::units::{CubicMeter, Kg};
use refinery_scenarios::{build_engine, load_str};

const EMPTY_PLANT: &str = include_str!("../../../scenarios/crude_unit_empty.toml");
const HOLDUP_PLANT: &str = include_str!("../../../scenarios/crude_unit_holdup.toml");

/// The holdup plant's holdup line, and the empty plant's.
const HOLDUP_LINE: &str = "holdup = { drum_m3 = 18.7, base_m3 = 115.4, reboiler = \"held\" }";
const EMPTY_LINE: &str = "holdup = { drum_m3 = 18.7, base_m3 = 115.4, reboiler = \"held\", \
                          steel = \"derived\", start = \"empty\" }";

/// The holdup plant with its holdup line's keys replaced by `keys`.
fn holdup_with(keys: &str) -> String {
    let src = HOLDUP_PLANT.replace("\r\n", "\n");
    assert!(src.contains(HOLDUP_LINE), "the holdup plant's holdup line");
    src.replace(
        HOLDUP_LINE,
        &format!("holdup = {{ drum_m3 = 18.7, base_m3 = 115.4, reboiler = \"held\"{keys} }}"),
    )
}

fn engine_of(src: &str) -> Result<Engine, String> {
    let file = load_str(src).map_err(|e| e.to_string())?;
    build_engine(&file).map_err(|e| e.to_string())
}

fn refused(src: &str) -> String {
    match engine_of(src) {
        Ok(_) => panic!("built a plant that should have been refused"),
        Err(e) => e,
    }
}

fn holdup(engine: &Engine) -> ColumnHoldup {
    let node = engine.graph.find_node("column").unwrap();
    match &engine.graph.node(node).kind {
        NodeKind::Column {
            cascade: Some(spec),
            ..
        } => *spec.holdup.clone().expect("the column holds liquid"),
        _ => panic!("the column is a cascade column"),
    }
}

fn state(engine: &Engine) -> HoldupState {
    holdup(engine)
        .state
        .expect("the first tick filled the state")
}

fn run(engine: &mut Engine, ticks: u32) -> Snapshot {
    for tick in 0..ticks {
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
    }
    engine.snapshot()
}

fn line_flow(snapshot: &Snapshot, name: &str) -> f64 {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == name)
        .expect(name)
        .stream
        .mass_flow
        .value()
}

fn reading(snapshot: &Snapshot) -> refinery_core::snapshot::HoldupOperation {
    let node = snapshot.nodes.iter().find(|n| n.name == "column").unwrap();
    node.column_operation
        .as_ref()
        .and_then(|op| op.holdup)
        .expect("a column that holds liquid reads its holdup")
}

/// The product lines out of the column and the stage each draws from (0 the
/// drum): the naphtha, kerosene, diesel, gas oil and residue.
const DRAWS: [(&str, usize); 5] = [
    ("naphtha_draw", 0),
    ("kerosene_draw", 4),
    ("diesel_draw", 7),
    ("gas_oil_draw", 10),
    ("residue_draw", 13),
];

fn edge(engine: &Engine, name: &str) -> EdgeId {
    engine
        .snapshot()
        .edges
        .iter()
        .find(|e| e.name == name)
        .expect(name)
        .id
}

/// Each component the column holds [kg], its stages and drum.
fn inventory(held: &HoldupState) -> Vec<f64> {
    let mut total = vec![0.0; held.drum_composition.fractions().len()];
    let stages = held.stage_liquid.iter().zip(&held.stage_composition);
    let drum = std::iter::once((&held.drum_liquid, &held.drum_composition));
    for (mass, composition) in stages.chain(drum) {
        for (slot, w) in total.iter_mut().zip(composition.fractions()) {
            *slot += mass.value() * w;
        }
    }
    total
}

/// The heat the column holds [J] on the engine's datum (0 °C): each held
/// liquid's `Σ w·cp·(T − 0 °C)` (the slate's constant heat capacities), and
/// each stage's steel's `m·cp_steel·(T − 0 °C)`.
fn held_heat(held: &HoldupState, cp: &[f64], steel: &[Kg]) -> f64 {
    let heat = |mass: f64, composition: &Composition, t: f64| {
        let cp_mix: f64 = composition
            .fractions()
            .iter()
            .zip(cp)
            .map(|(w, c)| w * c)
            .sum();
        mass * cp_mix * (t - 273.15)
    };
    let mut total = heat(
        held.drum_liquid.value(),
        &held.drum_composition,
        held.drum_temperature.value(),
    );
    for j in 0..held.stage_liquid.len() {
        let t = held.stage_temperature[j].value();
        total += heat(held.stage_liquid[j].value(), &held.stage_composition[j], t);
        if let Some(mass) = steel.get(j) {
            total += mass.value() * refinery_solvers::trays::STEEL_CP * (t - 273.15);
        }
    }
    total
}

/// Each stage's steel [kg] as the column derives it.
fn steel_of(engine: &Engine) -> Vec<Kg> {
    let node = engine.graph.find_node("column").unwrap();
    match &engine.graph.node(node).kind {
        NodeKind::Column {
            cascade: Some(spec),
            ..
        } => {
            let held = spec.holdup.as_ref().unwrap();
            match held.steel {
                Some(steel) => {
                    refinery_solvers::trays::stage_steel(spec, held.base_volume, steel).unwrap()
                }
                None => Vec::new(),
            }
        }
        _ => panic!("the column is a cascade column"),
    }
}

/// The column's books over the tick just run, from the state `before` it:
/// each component's feed less its products less what the column came to
/// hold [kg/s], the worst; and the heat's [W] — the reboiler and the feed
/// less the products, the condenser and the loops' coolers, less what its
/// liquid and steel came to hold.
struct Books {
    feed: EdgeId,
    draws: Vec<(EdgeId, usize)>,
    cp: Vec<f64>,
    steel: Vec<Kg>,
}

impl Books {
    fn of(engine: &Engine) -> Self {
        Books {
            feed: edge(engine, "transfer_line"),
            draws: DRAWS
                .iter()
                .map(|(name, stage)| (edge(engine, name), *stage))
                .collect(),
            cp: (0..engine.slate.len())
                .map(|c| engine.slate.get(c).cp.value())
                .collect(),
            steel: steel_of(engine),
        }
    }

    /// The worst component's imbalance [kg/s] and the heat's [W].
    fn close(&self, engine: &Engine, before: &HoldupState) -> (f64, f64) {
        let dt = 0.1;
        let after = state(engine);
        let snapshot = engine.snapshot();
        let flow = |id: EdgeId| {
            let e = snapshot.edges.iter().find(|e| e.id == id).unwrap();
            (
                e.stream.mass_flow.value(),
                e.stream.composition.fractions().to_vec(),
            )
        };
        let (fed, z) = flow(self.feed);
        let (held_before, held_after) = (inventory(before), inventory(&after));
        let mut worst: f64 = 0.0;
        for c in 0..z.len() {
            let mut balance = fed * z[c] - (held_after[c] - held_before[c]) / dt;
            for (id, _) in &self.draws {
                let (drawn, w) = flow(*id);
                balance -= drawn * w[c];
            }
            worst = worst.max(balance.abs());
        }

        let node = snapshot.nodes.iter().find(|n| n.name == "column").unwrap();
        let duty = node.column_duty.unwrap();
        let op = node.column_operation.clone().unwrap();
        let carried = |id: EdgeId, t: f64| {
            let (mass, w) = flow(id);
            let cp_mix: f64 = w.iter().zip(&self.cp).map(|(w, c)| w * c).sum();
            mass * cp_mix * (t - 273.15)
        };
        let mut heat = duty.reboiler_w + carried(self.feed, node.temperature_k)
            - duty.condenser_w
            - op.loops.iter().map(|l| l.duty_w).sum::<f64>()
            - (held_heat(&after, &self.cp, &self.steel) - held_heat(before, &self.cp, &self.steel))
                / dt;
        for (id, stage) in &self.draws {
            let t = match stage {
                0 => after.drum_temperature.value(),
                stage => after.stage_temperature[stage - 1].value(),
            };
            heat -= carried(*id, t);
        }
        (worst, heat)
    }
}

/// **The steel is derived from the trays** (Ludwig Vol. 1 eq. 11-55's shell
/// term and a 60 kg/m² deck): a hand calculation on the holdup plant's six
/// sections — tray 1, 5.9 m across on 0.76 m, `π·5.9·0.76·0.015·7850 =
/// 1658.7 kg` of shell and `π·5.9²/4·60 = 1640.4 kg` of deck; the base, 8.5 m
/// across, a shell `115.4/(π·8.5²/4) = 2.034 m` tall and one head. 59.0 t on
/// the trays, 13.1 t at the base, as M74.6.0's probe had it (§79.14). A
/// declared mass is shared in the same proportions.
#[test]
fn the_steel_is_derived_from_the_trays() {
    let engine = engine_of(EMPTY_PLANT).unwrap();
    let steel = steel_of(&engine);
    let pi = std::f64::consts::PI;
    let tray = |d: f64| pi * d * 0.76 * 0.015 * 7850.0 + pi * d * d / 4.0 * 60.0;
    let diameters = [5.9, 5.9, 7.3, 7.3, 7.3, 8.4, 8.4, 8.4, 8.5, 8.5, 7.0, 8.5];
    let mut expected: Vec<f64> = diameters.iter().map(|d| tray(*d)).collect();
    let area = pi * 8.5 * 8.5 / 4.0;
    expected.push((pi * 8.5 * (115.4 / area) + area) * 0.015 * 7850.0);
    assert_eq!(steel.len(), 13);
    for (j, (got, want)) in steel.iter().zip(&expected).enumerate() {
        assert!(
            (got.value() / want - 1.0).abs() < 1.0e-12,
            "stage {}: {} kg against {want}",
            j + 1,
            got.value()
        );
    }
    assert!((tray(5.9) - (1658.7 + 1640.4)).abs() < 0.1);
    let trays: f64 = steel[..12].iter().map(|m| m.value()).sum();
    assert!((trays / 1000.0 - 59.0).abs() < 0.05, "{trays}");
    assert!((steel[12].value() / 1000.0 - 13.1).abs() < 0.05);

    // Declared at twice the derived total: every stage twice its own.
    let total: f64 = steel.iter().map(|m| m.value()).sum();
    let node = engine.graph.find_node("column").unwrap();
    let NodeKind::Column {
        cascade: Some(spec),
        ..
    } = &engine.graph.node(node).kind
    else {
        panic!("the column is a cascade column");
    };
    let declared = refinery_solvers::trays::stage_steel(
        spec,
        CubicMeter(115.4),
        ColumnSteel::Declared {
            mass: Kg(2.0 * total),
        },
    )
    .unwrap();
    for (twice, once) in declared.iter().zip(&steel) {
        assert!((twice.value() / once.value() - 2.0).abs() < 1.0e-12);
    }
}

/// **The declaration is read as written**: the empty plant's steel derived
/// and its start empty; `steel_t` a declared mass in tonnes; neither key, no
/// steel and a running start, as every plant before M74.6.
#[test]
fn the_declaration_is_read_as_written() {
    let empty = holdup(&engine_of(EMPTY_PLANT).unwrap());
    assert_eq!(empty.steel, Some(ColumnSteel::Derived));
    assert!(empty.start_empty);
    let declared = holdup(&engine_of(&holdup_with(", steel_t = 80.0")).unwrap());
    assert_eq!(
        declared.steel,
        Some(ColumnSteel::Declared { mass: Kg(80_000.0) })
    );
    assert!(!declared.start_empty);
    let plain = holdup(&engine_of(HOLDUP_PLANT).unwrap());
    assert_eq!(plain.steel, None);
    assert!(!plain.start_empty);
    let running =
        holdup(&engine_of(&holdup_with(", steel = \"derived\", start = \"running\"")).unwrap());
    assert!(!running.start_empty);
}

/// **Refused where it cannot run**: an empty start without the steel (a
/// stage holding nothing has nothing in its energy balance that reads its
/// temperature, §79.14), both steel keys, an unknown steel or start, a mass
/// that is not above zero.
#[test]
fn an_empty_start_or_steel_is_refused_where_it_cannot_run() {
    for (keys, says) in [
        (", start = \"empty\"", "without its steel"),
        (", steel = \"derived\", steel_t = 80.0", "not both"),
        (", steel = \"heavy\"", "holdup.steel = \"heavy\""),
        (", steel_t = 0.0", "finite and above zero"),
        (", steel_t = -5.0", "finite and above zero"),
        (
            ", steel = \"derived\", start = \"cold\"",
            "holdup.start = \"cold\"",
        ),
    ] {
        let error = refused(&holdup_with(keys));
        assert!(error.contains(says), "{keys}: {error}");
        assert!(error.contains("§79.16"), "{keys}: {error}");
    }
    let error = refused(&holdup_with(", stee = 1.0"));
    assert!(error.contains("unknown field"), "{error}");
}

/// **The empty plant's file is the holdup plant's with two keys**: so the
/// two cannot drift apart unseen.
#[test]
fn the_empty_plant_is_the_holdup_plant_started_empty() {
    let strip = |src: &str| -> String {
        let src = src.replace("\r\n", "\n");
        let body = &src[src.find("[simulation]").unwrap()..];
        body.to_string()
    };
    let empty = strip(EMPTY_PLANT).replace(EMPTY_LINE, HOLDUP_LINE);
    assert_eq!(empty, strip(HOLDUP_PLANT));
}

/// **With its steel, a running column's books close every tick**: the
/// holdup plant with `steel = "derived"`, 400 ticks with the reflux moved at
/// the 100th — each component to round-off, the heat with the steel's
/// stored heat counted.
#[test]
fn with_the_steel_the_books_close_every_tick() {
    let mut engine = engine_of(&holdup_with(", steel = \"derived\"")).unwrap();
    let books = Books::of(&engine);
    engine.tick().unwrap();
    let (mut mass, mut heat): (f64, f64) = (0.0, 0.0);
    for t in 0..400 {
        if t == 100 {
            let node = engine.graph.find_node("column").unwrap();
            engine
                .apply(refinery_core::snapshot::Command::SetRefluxRatio {
                    node,
                    reflux_ratio: 6.0,
                })
                .unwrap();
        }
        let before = state(&engine);
        engine.tick().unwrap();
        let (m, h) = books.close(&engine, &before);
        mass = mass.max(m);
        heat = heat.max(h.abs());
    }
    // Measured 2.0e-10 kg/s and 3.1 W (without the steel, 9.0e-11 and 4.6,
    // `column_holdup::`).
    assert!(mass < 1.0e-8, "{mass}");
    assert!(heat < 20.0, "{heat}");
}

/// **The empty column's first tick is a step from empty**: every stage but
/// the feed's and the drum holding nothing, at 20 °C, the feed's tray what one
/// tick of crude left on it; the drum and base read dry, their levels zero;
/// nothing drawn.
#[test]
fn an_empty_column_starts_from_nothing() {
    let mut engine = engine_of(EMPTY_PLANT).unwrap();
    let first = run(&mut engine, 1);
    let held = state(&engine);
    let read = reading(&first);
    let fed = line_flow(&first, "transfer_line");
    for (j, held_kg) in held.stage_liquid.iter().enumerate() {
        if j == 10 {
            // The feed's tray (stage 11): what one tick of the crude left on
            // it (measured 10.4 kg, warmed 3.2 K on its steel).
            assert!(held_kg.value() > 0.0 && held_kg.value() <= fed * 0.1 + 1.0e-9);
        } else {
            assert_eq!(held_kg.value(), 0.0, "stage {}", j + 1);
        }
    }
    assert_eq!(held.drum_liquid.value(), 0.0);
    // An empty drum keeps its temperature: the start's.
    assert_eq!(held.drum_temperature.value(), 293.15);
    for (j, t) in held.stage_temperature.iter().enumerate() {
        if j != 10 {
            assert_eq!(t.value(), 293.15, "stage {}", j + 1);
        }
    }
    for (name, _) in DRAWS {
        assert_eq!(line_flow(&first, name), 0.0, "{name}");
    }
    assert_eq!(read.drum_limit, LevelLimit::Dry);
    assert_eq!(read.base_limit, LevelLimit::Dry);
    assert_eq!(read.drum_level, 0.0);
    assert_eq!(read.base_level, 0.0);
}

/// **The empty column fills and lines out** (M74.6.0's probe, §79.14): the
/// empty plant 25 minutes from its first tick, the crude at its full
/// preheat, the reboiler held at 86.7 MW. Measured (2026-11-11): the hot
/// feed's vapour condenses on the cold steel tray by tray, a stage holding
/// exactly nothing for the first 1778 ticks (3.0 minutes); the drum 15 % at
/// 4 minutes, the kerosene and diesel at their set draws from 6–7 minutes, the
/// base 10 % at 15; at 24 minutes the naphtha within 0.6 % of the running
/// plant's (the four top products within 2 %); the base's peak 612.8 °C (the
/// running column's 592 °C, §79.14). The books close every tick, through the
/// empty stages: each component to 1.5e-10 kg/s, the heat to 4.7 W.
#[test]
fn slow_the_empty_column_fills_and_lines_out() {
    let mut running = engine_of(HOLDUP_PLANT).unwrap();
    let design = run(&mut running, 1);
    let mut engine = engine_of(EMPTY_PLANT).unwrap();
    let books = Books::of(&engine);
    engine.tick().unwrap();
    let (mut mass, mut heat): (f64, f64) = (0.0, 0.0);
    let (mut empty_ticks, mut last_empty) = (0, 0);
    let mut base_peak: f64 = 0.0;
    for t in 1..14_400u32 {
        let before = state(&engine);
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        let (m, h) = books.close(&engine, &before);
        mass = mass.max(m);
        heat = heat.max(h.abs());
        let held = state(&engine);
        if held.stage_liquid.iter().any(|m| m.value() == 0.0) {
            empty_ticks += 1;
            last_empty = t;
        }
        base_peak = base_peak.max(held.stage_temperature[12].value() - 273.15);
        let snapshot = || engine.snapshot();
        if t == 3000 {
            assert!(
                reading(&snapshot()).drum_level > 0.1,
                "the drum at 5 minutes"
            );
        }
        if t == 4800 {
            let s = snapshot();
            for name in ["kerosene_draw", "diesel_draw"] {
                let (now, set) = (line_flow(&s, name), line_flow(&design, name));
                assert!(
                    (now / set - 1.0).abs() < 1.0e-3,
                    "{name} at 8 minutes: {now} against {set}"
                );
            }
        }
    }
    let end = engine.snapshot();
    for (name, _) in &DRAWS[..4] {
        let (now, set) = (line_flow(&end, name), line_flow(&design, name));
        assert!(
            (now / set - 1.0).abs() < 0.02,
            "{name} at 24 minutes: {now} against {set}"
        );
    }
    assert!(
        empty_ticks > 1000,
        "{empty_ticks}: the column did not start empty"
    );
    assert!(
        last_empty < 2400,
        "a stage still empty at tick {last_empty}"
    );
    assert!(
        reading(&end).base_level > 0.25,
        "{}",
        reading(&end).base_level
    );
    assert!((590.0..620.0).contains(&base_peak), "{base_peak} °C");
    assert!(mass < 1.0e-8, "{mass}");
    assert!(heat < 20.0, "{heat}");
}
