//! M74.6 — a column's steel, and the start from empty (docs/DESIGN.md §79.16):
//! the holdup plant started empty and cold (`scenarios/crude_unit_empty.toml`),
//! and the holdup plant (`crude_unit_holdup.toml`, untouched) with its steel
//! declared inline.

use refinery_core::components::Composition;
use refinery_core::engine::Engine;
use refinery_core::graph::{ColumnHoldup, ColumnSteel, EdgeId, HoldupState, NodeKind};
use refinery_core::snapshot::Snapshot;
use refinery_core::traits::{LevelLimit, SlopVessel};
use refinery_core::units::{CubicMeter, Kg};
use refinery_scenarios::{build_engine, load_str};

const EMPTY_PLANT: &str = include_str!("../../../scenarios/crude_unit_empty.toml");
pub(crate) const HOLDUP_PLANT: &str = include_str!("../../../scenarios/crude_unit_holdup.toml");

/// The holdup plant's holdup line, and the empty plant's.
pub(crate) const HOLDUP_LINE: &str =
    "holdup = { drum_m3 = 18.7, base_m3 = 115.4, reboiler = \"held\" }";
const EMPTY_LINE: &str = "holdup = { drum_m3 = 18.7, base_m3 = 115.4, reboiler = \"held\", \
                          steel = \"derived\", start = \"empty\" }";

/// The holdup plant with its holdup line's keys replaced by `keys`.
pub(crate) fn holdup_with(keys: &str) -> String {
    let src = HOLDUP_PLANT.replace("\r\n", "\n");
    assert!(src.contains(HOLDUP_LINE), "the holdup plant's holdup line");
    src.replace(
        HOLDUP_LINE,
        &format!("holdup = {{ drum_m3 = 18.7, base_m3 = 115.4, reboiler = \"held\"{keys} }}"),
    )
}

pub(crate) fn engine_of(src: &str) -> Result<Engine, String> {
    let file = load_str(src).map_err(|e| e.to_string())?;
    build_engine(&file).map_err(|e| e.to_string())
}

pub(crate) fn refused(src: &str) -> String {
    match engine_of(src) {
        Ok(_) => panic!("built a plant that should have been refused"),
        Err(e) => e,
    }
}

pub(crate) fn holdup(engine: &Engine) -> ColumnHoldup {
    let node = engine.graph.find_node("column").unwrap();
    match &engine.graph.node(node).kind {
        NodeKind::Column {
            cascade: Some(spec),
            ..
        } => *spec.holdup.clone().expect("the column holds liquid"),
        _ => panic!("the column is a cascade column"),
    }
}

pub(crate) fn state(engine: &Engine) -> HoldupState {
    holdup(engine)
        .state
        .expect("the first tick filled the state")
}

pub(crate) fn state_mut(engine: &mut Engine) -> &mut HoldupState {
    let node = engine.graph.find_node("column").unwrap();
    match &mut engine.graph.node_mut(node).kind {
        NodeKind::Column {
            cascade: Some(spec),
            ..
        } => spec
            .holdup
            .as_mut()
            .and_then(|h| h.state.as_mut())
            .expect("the first tick filled the state"),
        _ => panic!("the column is a cascade column"),
    }
}

pub(crate) fn run(engine: &mut Engine, ticks: u32) -> Snapshot {
    for tick in 0..ticks {
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
    }
    engine.snapshot()
}

pub(crate) fn line_flow(snapshot: &Snapshot, name: &str) -> f64 {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == name)
        .expect(name)
        .stream
        .mass_flow
        .value()
}

pub(crate) fn reading(snapshot: &Snapshot) -> refinery_core::snapshot::HoldupOperation {
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
pub(crate) struct Books {
    feed: EdgeId,
    draws: Vec<(EdgeId, usize)>,
    cp: Vec<f64>,
    steel: Vec<Kg>,
    /// The column's relief line's pipe into the flare, where it has one
    /// (M75.3.2, docs/DESIGN.md §80.6): the vent counted off it.
    flare: Option<EdgeId>,
}

/// The name of a column relief line's pipe into its flare in the fixtures
/// (`column_relief::with_relief`).
pub(crate) const FLARE_PIPE: &str = "flare_header";

impl Books {
    pub(crate) fn of(engine: &Engine) -> Self {
        Books {
            // The pipe INTO the column: the transfer line is split at its
            // leak point, and the network closes that junction only to its
            // tolerance — on a column whose pressure moves every tick the
            // upstream half reads ~4e-8 kg/s off what the column was fed
            // (M75.1, docs/DESIGN.md §80.2).
            feed: edge(engine, "transfer_line__downstream"),
            draws: DRAWS
                .iter()
                .map(|(name, stage)| (edge(engine, name), *stage))
                .collect(),
            cp: (0..engine.slate.len())
                .map(|c| engine.slate.get(c).cp.value())
                .collect(),
            steel: steel_of(engine),
            flare: engine
                .snapshot()
                .edges
                .iter()
                .find(|e| e.name == FLARE_PIPE)
                .map(|e| e.id),
        }
    }

    /// The worst component's imbalance [kg/s] and the heat's [W].
    pub(crate) fn close(&self, engine: &Engine, before: &HoldupState) -> (f64, f64) {
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
        let node = snapshot.nodes.iter().find(|n| n.name == "column").unwrap();
        let duty = node.column_duty.unwrap();
        let op = node.column_operation.clone().unwrap();
        let (fed, z) = flow(self.feed);
        let (held_before, held_after) = (inventory(before), inventory(&after));
        let mut worst: f64 = 0.0;
        for c in 0..z.len() {
            let mut balance = fed * z[c] - (held_after[c] - held_before[c]) / dt;
            for (id, _) in &self.draws {
                let (drawn, w) = flow(*id);
                balance -= drawn * w[c];
            }
            // And what went to slop, on no pipe (M74.6, docs/DESIGN.md
            // §79.18).
            for slop in &op.slop {
                balance -= slop.flow_kg_s * slop.composition.fractions()[c];
            }
            // And what its relief vented: off its line's pipe into the flare
            // (M75.3.2, §80.6), else on no pipe (M75.3, §80.5) — once.
            match (self.flare, &op.relief) {
                (Some(flare), _) => {
                    let (vented, w) = flow(flare);
                    balance -= vented * w[c];
                }
                (None, Some(relief)) => {
                    balance -= relief.flow_kg_s * relief.composition.fractions()[c];
                }
                (None, None) => {}
            }
            worst = worst.max(balance.abs());
        }

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
        for slop in &op.slop {
            let cp_mix: f64 = slop
                .composition
                .fractions()
                .iter()
                .zip(&self.cp)
                .map(|(w, c)| w * c)
                .sum();
            heat -= slop.flow_kg_s * cp_mix * (slop.temperature_k - 273.15);
        }
        // The vent's heat: its pipe's whole enthalpy, the latent heat it
        // carries included (M75.3.2), else the column's own figure for it
        // (the top's vapour, M75.3).
        match (self.flare, &op.relief) {
            (Some(flare), _) => {
                let e = snapshot.edges.iter().find(|e| e.id == flare).unwrap();
                let latent = e.stream.latent.map_or(0.0, |l| l.value());
                heat -= carried(flare, e.stream.temperature.value())
                    + e.stream.mass_flow.value() * latent;
            }
            (None, Some(relief)) => heat -= relief.heat_w,
            (None, None) => {}
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

pub(crate) fn set_total_reflux(engine: &mut Engine, on: bool) -> Result<(), String> {
    let node = engine.graph.find_node("column").unwrap();
    engine
        .apply(refinery_core::snapshot::Command::SetTotalReflux { node, on })
        .map_err(|e| e.to_string())
}

/// **Total reflux is refused where it cannot run** (§79.17): on a column
/// without its steel (refused at full reboiler, §79.15), on a column that
/// holds no liquid, on a node that is not a column. Off is always taken
/// where it applies.
#[test]
fn total_reflux_is_refused_where_it_cannot_run() {
    let mut plain = engine_of(HOLDUP_PLANT).unwrap();
    let error = set_total_reflux(&mut plain, true).unwrap_err();
    assert!(error.contains("declares no steel"), "{error}");
    assert!(!holdup(&plain).total_reflux);
    set_total_reflux(&mut plain, false).unwrap();

    let trays = include_str!("../../../scenarios/crude_unit_trays.toml");
    let mut quasi = engine_of(trays).unwrap();
    let error = set_total_reflux(&mut quasi, true).unwrap_err();
    assert!(error.contains("holds no liquid"), "{error}");

    let mut engine = engine_of(EMPTY_PLANT).unwrap();
    let furnace = engine.graph.find_node("furnace").unwrap();
    let error = engine
        .apply(refinery_core::snapshot::Command::SetTotalReflux {
            node: furnace,
            on: true,
        })
        .unwrap_err()
        .to_string();
    assert!(error.contains("is not a column"), "{error}");
    set_total_reflux(&mut engine, true).unwrap();
    assert!(holdup(&engine).total_reflux);
}

/// **On total reflux the products shut and the drum holds on its reflux**
/// (§79.17; Kister §12.8): the holdup plant with its steel, running; switched
/// on, from the next tick the distillate and the three side draws carry
/// nothing, the bottoms goes on, the drum's outflow — now all reflux — is
/// what it was (the level loop's output times `R + 1`), the reading says so;
/// the books close every tick; switched off, the side draws are back at
/// their set flows on the next tick.
#[test]
fn on_total_reflux_the_products_shut() {
    let mut engine = engine_of(&holdup_with(", steel = \"derived\"")).unwrap();
    let books = Books::of(&engine);
    let before = run(&mut engine, 100);
    assert!(!reading(&before).total_reflux);
    let node = before.nodes.iter().find(|n| n.name == "column").unwrap();
    assert!(!serde_json::to_string(&node.kind)
        .unwrap()
        .contains("total_reflux"));
    assert!(!serde_json::to_string(&reading(&before))
        .unwrap()
        .contains("total_reflux"));
    let kerosene = line_flow(&before, "kerosene_draw");
    set_total_reflux(&mut engine, true).unwrap();
    let (mut mass, mut heat): (f64, f64) = (0.0, 0.0);
    for t in 0..300 {
        let start = state(&engine);
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        let (m, h) = books.close(&engine, &start);
        mass = mass.max(m);
        heat = heat.max(h.abs());
        let s = engine.snapshot();
        for (name, _) in &DRAWS[..4] {
            assert_eq!(line_flow(&s, name), 0.0, "{name} on tick {t}");
        }
        assert!(line_flow(&s, "residue_draw") > 0.0);
        assert!(reading(&s).total_reflux);
    }
    let on = engine.snapshot();
    // The drum's outflow all reflux, the same as before the switch: its level
    // moved by the column's own answer only (measured 0.500 to 0.511 over the
    // 30 s; with the reflux alone sent back, a fifth of the outflow short, it
    // would rise by about a tenth).
    let level = reading(&on).drum_level;
    assert!((level - 0.5).abs() < 0.03, "{level}");
    let node = on.nodes.iter().find(|n| n.name == "column").unwrap();
    assert!(serde_json::to_string(&node.kind)
        .unwrap()
        .contains("\"total_reflux\":true"));
    assert!(mass < 1.0e-8, "{mass}");
    assert!(heat < 20.0, "{heat}");
    set_total_reflux(&mut engine, false).unwrap();
    let off = run(&mut engine, 1);
    assert!(!reading(&off).total_reflux);
    let back = line_flow(&off, "kerosene_draw");
    // To 1e-4, as the holdup tests' draws (measured 1.9e-5): the draw is set
    // in moles at the tick's starting composition, carried at its end's.
    assert!(
        (back / kerosene - 1.0).abs() < 1.0e-4,
        "{back} against {kerosene}"
    );
    assert!(line_flow(&off, "naphtha_draw") > 0.0);
}

/// **On total reflux a drum past full sends its excess to slop, not to the
/// naphtha** (M74.6, docs/DESIGN.md §79.18 — the case §79.14 named: before,
/// 15 kg/s of naphtha off a full drum, the operator's mistake partly hiding
/// itself): the holdup plant with its steel on total reflux, its drum put at
/// 1.2 of its volume — the naphtha line exactly 0, the slop the sixth of the
/// drum over full in the tick, the drum reading `"full"` (the alarm), the
/// books closing with the slop counted.
#[test]
fn on_total_reflux_a_full_drum_goes_to_slop() {
    let mut engine = engine_of(&holdup_with(", steel = \"derived\"")).unwrap();
    let books = Books::of(&engine);
    run(&mut engine, 100);
    set_total_reflux(&mut engine, true).unwrap();
    let now = reading(&run(&mut engine, 10)).drum_level;
    let held = state_mut(&mut engine);
    held.drum_liquid = Kg(held.drum_liquid.value() * 1.2 / now);
    let held_kg = held.drum_liquid.value();
    let start = state(&engine);
    let after = run(&mut engine, 1);
    let read = reading(&after);
    assert!(read.total_reflux);
    assert_eq!(read.drum_limit, LevelLimit::Full);
    assert_eq!(line_flow(&after, "naphtha_draw"), 0.0);
    let node = after.nodes.iter().find(|n| n.name == "column").unwrap();
    let slop = &node.column_operation.as_ref().unwrap().slop;
    assert_eq!(slop.len(), 1, "{slop:?}");
    assert_eq!(slop[0].vessel, SlopVessel::Drum);
    let overflow = held_kg * (1.0 - 1.0 / read.drum_level) / 0.1;
    assert!(
        (slop[0].flow_kg_s / overflow - 1.0).abs() < 1.0e-4,
        "{} kg/s against {overflow}",
        slop[0].flow_kg_s
    );
    let (mass, heat) = books.close(&engine, &start);
    println!(
        "M74_6 slop on total reflux: {} kg/s against {overflow}; books {mass} kg/s, {heat} W",
        slop[0].flow_kg_s
    );
    assert!(mass < 1.0e-8, "{mass}");
    assert!(heat.abs() < 20.0, "{heat}");
}

/// **A start from empty on total reflux** (Kister §12.8's start-up, the
/// crude coming in): on from before the first tick, six minutes, then off.
/// Measured: nothing leaves but the bottoms (each product line exactly 0 for
/// the six minutes); the drum fills to 0.46 and holds on its reflux; nothing
/// floods; switched off, the kerosene at its set draw on the next tick, the
/// gas oil within about four minutes. The books close every tick (6.1e-11
/// kg/s, 5.1 W).
#[test]
fn slow_an_empty_start_on_total_reflux() {
    let mut running = engine_of(HOLDUP_PLANT).unwrap();
    let design = run(&mut running, 1);
    let mut engine = engine_of(EMPTY_PLANT).unwrap();
    set_total_reflux(&mut engine, true).unwrap();
    let books = Books::of(&engine);
    engine.tick().unwrap();
    let (mut mass, mut heat): (f64, f64) = (0.0, 0.0);
    for t in 1..6000u32 {
        if t == 3600 {
            let s = engine.snapshot();
            let read = reading(&s);
            assert!(
                (0.4..0.55).contains(&read.drum_level),
                "{}",
                read.drum_level
            );
            assert_eq!(read.drum_limit, LevelLimit::Normal);
            let node = s.nodes.iter().find(|n| n.name == "column").unwrap();
            assert!(node
                .column_operation
                .as_ref()
                .unwrap()
                .trays_flooding
                .is_empty());
            set_total_reflux(&mut engine, false).unwrap();
        }
        let before = state(&engine);
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        let (m, h) = books.close(&engine, &before);
        mass = mass.max(m);
        heat = heat.max(h.abs());
        let s = engine.snapshot();
        if t < 3600 {
            for (name, _) in &DRAWS[..4] {
                assert_eq!(line_flow(&s, name), 0.0, "{name} on tick {t}");
            }
        }
        if t == 3600 {
            let (now, set) = (
                line_flow(&s, "kerosene_draw"),
                line_flow(&design, "kerosene_draw"),
            );
            assert!((now / set - 1.0).abs() < 1.0e-3, "{now} against {set}");
        }
    }
    let end = engine.snapshot();
    let (now, set) = (
        line_flow(&end, "gas_oil_draw"),
        line_flow(&design, "gas_oil_draw"),
    );
    assert!(
        (now / set - 1.0).abs() < 1.0e-3,
        "the gas oil: {now} against {set}"
    );
    assert!(mass < 1.0e-8, "{mass}");
    assert!(heat < 20.0, "{heat}");
}
