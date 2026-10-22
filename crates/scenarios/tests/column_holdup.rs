//! M74.1 — a column's holdup declared, and its state filled on its first tick
//! (docs/DESIGN.md §79.2): the tray plant (`scenarios/crude_unit_trays.toml`,
//! untouched) with a holdup block added inline. The column is still
//! quasi-steady in this slice: the state is filled from the first tick's
//! settled loads and carried unchanged, and nothing else the plant does moves.

use refinery_core::components::Composition;
use refinery_core::engine::Engine;
use refinery_core::graph::{HoldupState, NodeKind};
use refinery_core::snapshot::{Command, Snapshot};
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

/// The fixture with its reboiler free: scheme A, today's settled answers.
fn fixture_free() -> String {
    with_line(TRAY_PLANT, &HOLDUP.replace("\"held\"", "\"free\""))
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

/// **M74.2's measurement** (docs/DESIGN.md §79.2, open; run with
/// `M74_MEASURE` set, else it returns): the screen's biggest one-press moves
/// under both schemes — A, the reboiler free; B, held at the duty its first
/// tick settled on — each from 100 ticks at the file's settings, then 6 000
/// ticks (ten minutes). Printed per run: the least vapour any stage carried,
/// where and when; the drum's and the base's liquid against their first
/// tick's (normal) as a share; the mean and worst wall time a tick; or the
/// refusal.
#[test]
fn m74_2_measure_the_big_moves() {
    if std::env::var("M74_MEASURE").is_err() {
        return;
    }
    use refinery_core::snapshot::Command;
    use refinery_core::units::{KgPerSec, Watt};
    let moves: [(&str, f64); 5] = [
        ("reflux", 0.5),
        ("reflux", 8.0),
        ("reflux", 15.0),
        ("flow", 0.0),
        ("flow", 2.0),
    ];
    let flows = [85.0, 95.0, 65.0];
    let mut runs = Vec::new();
    for scheme in ["A", "B"] {
        for (key, value) in moves {
            runs.push((scheme, key, value));
        }
    }
    // M74_RUNS: a list like "A:reflux:0.5,A:rest:0" in place of the ten.
    if let Ok(list) = std::env::var("M74_RUNS") {
        runs = list
            .split(',')
            .map(|run| {
                let mut parts = run.trim().split(':');
                let scheme = match parts.next() {
                    Some("B") => "B",
                    Some("T") => "T",
                    _ => "A",
                };
                let key = match parts.next() {
                    Some("reflux") => "reflux",
                    Some("flow") => "flow",
                    _ => "rest",
                };
                let value: f64 = parts.next().unwrap().parse().unwrap();
                (scheme, key, value)
            })
            .collect();
    }
    let ticks: usize = std::env::var("M74_TICKS")
        .ok()
        .and_then(|t| t.trim().parse().ok())
        .unwrap_or(6000);
    let lines: Vec<String> = std::thread::scope(|scope| {
        let handles: Vec<_> = runs
            .iter()
            .map(|&(scheme, key, value)| {
                scope.spawn(move || {
                    let src = if scheme == "T" { TRAY_PLANT.to_string() } else { fixture() };
                    let mut engine = engine_of(&src).unwrap();
                    let column = engine.graph.find_node("column").unwrap();
                    engine.tick().unwrap();
                    let normal = state(&engine);
                    if scheme == "B" {
                        let duty = engine.snapshot().nodes[column.0 as usize]
                            .column_duty
                            .as_ref()
                            .map(|d| d.reboiler_w)
                            .unwrap();
                        engine
                            .apply(Command::SetReboilerDuty {
                                node: column,
                                duty: Some(Watt(duty)),
                            })
                            .unwrap();
                    }
                    for _ in 0..99 {
                        engine.tick().unwrap();
                    }
                    match key {
                        "rest" => {}
                        "reflux" => engine
                            .apply(Command::SetRefluxRatio {
                                node: column,
                                reflux_ratio: value,
                            })
                            .unwrap(),
                        _ => {
                            for (i, f) in flows.iter().enumerate() {
                                engine
                                    .apply(Command::SetLoopFlow {
                                        node: column,
                                        pumparound_loop: i as u32,
                                        flow: KgPerSec(f * value),
                                    })
                                    .unwrap();
                            }
                        }
                    }
                    let (mut least, mut at, mut when) = (f64::INFINITY, 0, 0);
                    let (mut drum_lo, mut drum_hi, mut base_lo, mut base_hi) =
                        (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
                    let (mut total, mut worst) = (0.0, 0.0f64);
                    for t in 0..ticks {
                        let start = std::time::Instant::now();
                        if let Err(e) = engine.tick() {
                            let reason: String = e.to_string().chars().take(300).collect();
                            return format!(
                                "{scheme} {key}={value}: REFUSED at {:.1} s: {reason} | least \
                                 vapour so far {least:.1} mol/s stage {at} at {:.1} s",
                                t as f64 * 0.1,
                                when as f64 * 0.1
                            );
                        }
                        let wall = start.elapsed().as_secs_f64() * 1e3;
                        if let Some(every) = std::env::var("M74_EVERY").ok().and_then(|e| e.trim().parse::<usize>().ok()) {
                            if (t + 1) % every == 0 {
                                let s = engine.snapshot();
                                let d = s.nodes[column.0 as usize].column_duty.unwrap();
                                println!("M74EVERY {scheme} {key}={value} {:.1} min reb {:.3} MW", (t + 1) as f64 / 600.0, d.reboiler_w / 1e6);
                            }
                        }
                        total += wall;
                        worst = worst.max(wall);
                        let snapshot = engine.snapshot();
                        let op = snapshot.nodes[column.0 as usize]
                            .column_operation
                            .as_ref()
                            .unwrap();
                        for (j, v) in op.stage_vapour_mol_s.iter().enumerate() {
                            if *v < least {
                                least = *v;
                                at = j + 1;
                                when = t;
                            }
                        }
                        let (drum, base) = match (state(&engine), &normal) {
                            (Some(held), Some(normal)) => (
                                held.drum_liquid.value() / normal.drum_liquid.value() - 1.0,
                                held.stage_liquid[12].value() / normal.stage_liquid[12].value()
                                    - 1.0,
                            ),
                            _ => (0.0, 0.0),
                        };
                        drum_lo = drum_lo.min(drum);
                        drum_hi = drum_hi.max(drum);
                        base_lo = base_lo.min(base);
                        base_hi = base_hi.max(base);
                    }
                    let snapshot = engine.snapshot();
                    let op = snapshot.nodes[column.0 as usize].column_operation.clone().unwrap();
                    let duty = snapshot.nodes[column.0 as usize].column_duty.unwrap();
                    println!(
                        "M74PROFILE {scheme} {key}={value}: V {:?} T {:?} reb {:.4} MW cond {:.4} MW",
                        op.stage_vapour_mol_s.iter().map(|v| (v * 10.0).round() / 10.0).collect::<Vec<_>>(),
                        op.stage_temperatures_k.iter().map(|t| ((t - 273.15) * 100.0).round() / 100.0).collect::<Vec<_>>(),
                        duty.reboiler_w / 1e6,
                        duty.condenser_w / 1e6
                    );
                    format!(
                        "{scheme} {key}={value}: RUNS | least vapour {least:.1} mol/s stage {at} \
                         at {:.1} s | drum {:+.0}..{:+.0}% base {:+.0}..{:+.0}% | {:.2} ms a \
                         tick, worst {:.1}",
                        when as f64 * 0.1,
                        drum_lo * 100.0,
                        drum_hi * 100.0,
                        base_lo * 100.0,
                        base_hi * 100.0,
                        total / ticks as f64,
                        worst
                    )
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });
    for line in lines {
        println!("M74MEASURE {line}");
    }
}

/// Each component's mass the column holds [kg]: its stages' and its drum's.
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

/// The heat the column's liquid holds [J] on the engine's datum: each held
/// liquid's `Σ w·cp·(T − 0 °C)` (the slate's constant heat capacities).
fn held_heat(held: &HoldupState, cp: &[f64]) -> f64 {
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
        total += heat(
            held.stage_liquid[j].value(),
            &held.stage_composition[j],
            held.stage_temperature[j].value(),
        );
    }
    total
}

/// Each draw's pipe out of the column and the stage it draws from: the
/// naphtha off the drum (0), the kerosene 4, the diesel 7, the gas oil 10,
/// the residue 13 (the tray plant's draws run straight to their tanks).
fn draw_edges(engine: &Engine) -> Vec<(usize, refinery_core::graph::EdgeId)> {
    let snapshot = engine.snapshot();
    [
        ("naphtha_draw", 0),
        ("kerosene_draw", 4),
        ("diesel_draw", 7),
        ("gas_oil_draw", 10),
        ("residue_draw", 13),
    ]
    .iter()
    .map(|(name, stage)| {
        let edge = snapshot.edges.iter().find(|e| e.name == *name).expect(name);
        (*stage, edge.id)
    })
    .collect()
}

fn set_reflux(engine: &mut Engine, reflux_ratio: f64) {
    let node = engine.graph.find_node("column").unwrap();
    engine
        .apply(Command::SetRefluxRatio { node, reflux_ratio })
        .unwrap();
}

/// **A column at rest stays with the tray plant**: 3 000 ticks untouched,
/// the column that holds liquid and the one that does not run on the same
/// answer — each stage's vapour and temperature, the duties.
#[test]
fn a_column_at_rest_stays_with_the_tray_plant() {
    let with = run(&mut engine_of(&fixture_free()).unwrap(), 3000);
    let without = run(&mut engine_of(TRAY_PLANT).unwrap(), 3000);
    let read = |s: &Snapshot| {
        let node = s.nodes.iter().find(|n| n.name == "column").unwrap();
        (
            node.column_operation.clone().unwrap(),
            node.column_duty.unwrap(),
        )
    };
    let ((a, a_duty), (b, b_duty)) = (read(&with), read(&without));
    let vapour = a
        .stage_vapour_mol_s
        .iter()
        .zip(&b.stage_vapour_mol_s)
        .map(|(x, y)| (x / y - 1.0).abs())
        .fold(0.0, f64::max);
    let temperature = a
        .stage_temperatures_k
        .iter()
        .zip(&b.stage_temperatures_k)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max);
    let reboiler = (a_duty.reboiler_w / b_duty.reboiler_w - 1.0).abs();
    let condenser = (a_duty.condenser_w / b_duty.condenser_w - 1.0).abs();
    // Measured at 3 000 ticks: vapour 0.41 %, 0.80 K, the duties 0.19 %, the
    // column still closing on the tray plant's own drift after its first
    // tick (an hour on, 0.02 %, 0.06 K, 0.01 %: no slow drift of its own).
    assert!(vapour < 0.01, "{vapour}");
    assert!(temperature < 1.5, "{temperature}");
    assert!(
        reboiler < 0.005 && condenser < 0.005,
        "{reboiler} {condenser}"
    );
}

/// **Each component's books close every tick, from the engine's own pipes**:
/// what the feed line brings less what every draw line takes is what the
/// column came to hold — at rest and after the reflux moves.
#[test]
fn each_component_is_conserved_every_tick() {
    let mut engine = engine_of(&fixture()).unwrap();
    let feed = engine
        .snapshot()
        .edges
        .iter()
        .find(|e| e.name == "transfer_line")
        .unwrap()
        .id;
    let draws: Vec<_> = draw_edges(&engine)
        .into_iter()
        .map(|(_, edge)| edge)
        .collect();
    engine.tick().unwrap();
    let dt = 0.1;
    let mut worst: f64 = 0.0;
    for t in 0..400 {
        if t == 100 {
            set_reflux(&mut engine, 6.0);
        }
        let before = inventory(&state(&engine).unwrap());
        engine.tick().unwrap();
        let after = inventory(&state(&engine).unwrap());
        let snapshot = engine.snapshot();
        let flow = |edge| {
            let e = snapshot.edges.iter().find(|e| e.id == edge).unwrap();
            (
                e.stream.mass_flow.value(),
                e.stream.composition.fractions().to_vec(),
            )
        };
        let (fed, z) = flow(feed);
        for c in 0..z.len() {
            let mut balance = fed * z[c] - (after[c] - before[c]) / dt;
            for edge in &draws {
                let (drawn, w) = flow(*edge);
                balance -= drawn * w[c];
            }
            worst = worst.max(balance.abs());
        }
    }
    // Measured 9.0e-11 kg/s: round-off on a column holding 60 t.
    assert!(worst < 1.0e-8, "{worst}");
}

/// **The column's heat closes every tick**: the reboiler and the feed less
/// the products, the condenser and the loops' coolers is the heat its liquid
/// came to hold — at rest and after the reflux moves.
#[test]
fn the_heat_is_conserved_every_tick() {
    let mut engine = engine_of(&fixture()).unwrap();
    let feed = engine
        .snapshot()
        .edges
        .iter()
        .find(|e| e.name == "transfer_line")
        .unwrap()
        .id;
    let cp: Vec<f64> = (0..engine.slate.len())
        .map(|c| engine.slate.get(c).cp.value())
        .collect();
    let lines = draw_edges(&engine);
    engine.tick().unwrap();
    let dt = 0.1;
    let mut worst: f64 = 0.0;
    for t in 0..400 {
        if t == 100 {
            set_reflux(&mut engine, 6.0);
        }
        let before = held_heat(&state(&engine).unwrap(), &cp);
        engine.tick().unwrap();
        let held = state(&engine).unwrap();
        let after = held_heat(&held, &cp);
        let snapshot = engine.snapshot();
        let node = snapshot.nodes.iter().find(|n| n.name == "column").unwrap();
        let duty = node.column_duty.unwrap();
        let op = node.column_operation.clone().unwrap();
        let carried = |edge, t: f64| {
            let e = snapshot.edges.iter().find(|e| e.id == edge).unwrap();
            let cp_mix: f64 = e
                .stream
                .composition
                .fractions()
                .iter()
                .zip(&cp)
                .map(|(w, c)| w * c)
                .sum();
            e.stream.mass_flow.value() * cp_mix * (t - 273.15)
        };
        let mut balance = duty.reboiler_w + carried(feed, node.temperature_k)
            - duty.condenser_w
            - op.loops.iter().map(|l| l.duty_w).sum::<f64>()
            - (after - before) / dt;
        for (stage, edge) in &lines {
            let t = match stage {
                0 => held.drum_temperature.value(),
                stage => held.stage_temperature[stage - 1].value(),
            };
            balance -= carried(*edge, t);
        }
        worst = worst.max(balance.abs());
    }
    // Measured 4.6 W. Each stage's balance is met to the step's tolerance
    // times the column's duty scale, about 1.4 W, so thirteen stages to 18 W.
    assert!(worst < 20.0, "{worst}");
}

/// **More liquid over a weir stands higher**: the reflux from 4 to 5, the
/// trays come to hold more within two minutes (M74.1.0 finding 2: +3.6 t
/// settled).
#[test]
fn more_reflux_puts_more_liquid_on_the_trays() {
    let mut engine = engine_of(&fixture()).unwrap();
    run(&mut engine, 100);
    let trays = |e: &Engine| {
        state(e).unwrap().stage_liquid[..12]
            .iter()
            .map(|m| m.value())
            .sum::<f64>()
    };
    let before = trays(&engine);
    set_reflux(&mut engine, 5.0);
    run(&mut engine, 1200);
    let after = trays(&engine);
    // Measured +508 kg in two minutes, the reboiler not yet answering.
    assert!(after - before > 250.0, "{before} -> {after}");
}

/// **Under scheme A a key move lands where the tray plant does** (§79.1:
/// A keeps today's settled answers): the reflux 4 → 5 against the tray plant
/// at 5, the drum and base on Kister's sizes. The column closes on it about
/// an e-fold every 40 minutes (measured over two hours: the reboiler 90.8 MW
/// at 10 minutes, 96.0 at 60, 97.2 at 120, against the tray plant's 97.55;
/// the stages within 0.7 K at 120) — slower than M74.0's frozen-K estimate —
/// so the test reads it at 30 and 60 minutes: within 2 % at the hour, the gap
/// more than halved over the second half hour (measured 3.38 → 1.52 MW).
#[test]
fn slow_scheme_a_lands_on_the_settled_column() {
    let small = HOLDUP
        .replace("drum_m3 = 20.0", "drum_m3 = 10.0")
        .replace("base_m3 = 60.0", "base_m3 = 30.0")
        .replace("\"held\"", "\"free\"");
    let mut without = engine_of(TRAY_PLANT).unwrap();
    run(&mut without, 100);
    set_reflux(&mut without, 5.0);
    let settled = reboiler_w(&run(&mut without, 3000));
    let mut with = engine_of(&with_line(TRAY_PLANT, &small)).unwrap();
    run(&mut with, 100);
    set_reflux(&mut with, 5.0);
    let half_hour = (reboiler_w(&run(&mut with, 18000)) - settled).abs();
    let hour = (reboiler_w(&run(&mut with, 18000)) - settled).abs();
    assert!(hour / settled < 0.02, "{hour} W off {settled} W");
    assert!(hour < 0.6 * half_hour, "{half_hour} W → {hour} W");
}

fn reboiler_w(snapshot: &Snapshot) -> f64 {
    let node = snapshot.nodes.iter().find(|n| n.name == "column").unwrap();
    node.column_duty.as_ref().unwrap().reboiler_w
}

/// **The level loops hold their levels, under either scheme**: the reflux
/// 4 → 5, five minutes on, the drum and the base stay near half full —
/// under A the reboiler moving, under B (the reboiler held at the duty the
/// first tick settled on) the bottoms moving and the reboiler at its hold.
#[test]
fn the_level_loops_hold_their_levels() {
    for held in [false, true] {
        let src = if held { fixture() } else { fixture_free() };
        let mut engine = engine_of(&src).unwrap();
        let first = run(&mut engine, 1);
        let normal = state(&engine).unwrap();
        let duty = reboiler_w(&first);
        run(&mut engine, 99);
        set_reflux(&mut engine, 5.0);
        let (mut drum, mut base, mut reboiler_moved): (f64, f64, f64) = (0.0, 0.0, 0.0);
        for _ in 0..3000 {
            let snapshot = run(&mut engine, 1);
            let now = state(&engine).unwrap();
            drum = drum.max((now.drum_liquid.value() / normal.drum_liquid.value() - 1.0).abs());
            base = base
                .max((now.stage_liquid[12].value() / normal.stage_liquid[12].value() - 1.0).abs());
            reboiler_moved = reboiler_moved.max((reboiler_w(&snapshot) / duty - 1.0).abs());
        }
        // Measured: the drum within 6.7 % of normal under both; the base 1.0 %
        // under A, 2.6 % under B; the reboiler moved 2.7 % under A, not at
        // all under B.
        assert!(
            drum < 0.10 && base < 0.05,
            "held {held}: drum {drum}, base {base}"
        );
        if held {
            assert_eq!(reboiler_moved, 0.0, "the held reboiler moved");
        } else {
            assert!(
                reboiler_moved > 0.01,
                "A's reboiler did not answer: {reboiler_moved}"
            );
        }
    }
}
