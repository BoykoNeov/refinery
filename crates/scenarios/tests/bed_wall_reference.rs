//! M58: what a runaway breaks — a catalyst bed's wall fails past its metal
//! limit (docs/DESIGN.md §63), gated on `scenarios/hydrocracker_wall_failure.toml`
//! and on fixtures cut from it.
//!
//! What is pinned, and against what:
//!
//! 1. the wall fails on the tick AFTER the bed first ends one at or past its
//!    limit — the burn-out pass reads the start-of-tick state — and the hole
//!    opens to exactly the declared area, on both fidelities;
//! 2. the fire is the leak times the heating value, to the bit, and zero on an
//!    intact wall;
//! 3. the declared share of the fire heats the bed: at zero flow through the
//!    bed, its temperature rises by exactly `s·L·dt/C` a tick, and not at all
//!    with a share of zero;
//! 4. a hole punched by hand in an INTACT wall leaks and does not burn; in a
//!    FAILED wall it burns;
//! 5. a failure keeps a wider hand hole;
//! 6. `RepairBedWall`: its four refusals, its success, and a wall that fails
//!    again — and the player's way back, a chilled feed;
//! 7. every bed in the repo owns a hole, and the loader's refusals.
//!
//! Every gate but the first runs the demo with its bed made 60 times lighter
//! (0.5 MJ/K), so a debug build gets through each story in a few hundred
//! ticks: the heat capacity sets how fast a bed gets somewhere, not where.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{LeakRole, NodeKind, WallState};
use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot};
use refinery_core::units::{Kelvin, SquareMeter};
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/hydrocracker_wall_failure.toml");

/// The demo's numbers, by hand from its file.
const LIMIT_K: f64 = 550.0 + 273.15;
const HOLE_M2: f64 = 1.0e-4;
const HEATING_J_PER_KG: f64 = 42.8 * 1e6;
const SHARE: f64 = 0.1;
const DT_S: f64 = 0.1;
/// The light fixture's bed heat capacity [J/K].
const LIGHT_C_J_PER_K: f64 = 0.5e6;
/// Measured (both fidelities): the first tick whose top pass finds the bed at
/// or past its limit; the bed is at 550.13 °C at the end of it.
const FAIL_TICK: u64 = 2_750;

fn build_src(src: &str, solver: &str) -> Engine {
    let src = src.replacen(r#"flow = "newton""#, &format!(r#"flow = "{solver}""#), 1);
    let file = refinery_scenarios::load_str(&src).expect("the scenario must parse");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("it must build: {e}"))
}

/// The demo with a bed 60 times lighter.
fn light() -> String {
    let src = DEMO.replace(
        "bed_heat_capacity_mj_per_k = 30.0",
        "bed_heat_capacity_mj_per_k = 0.5",
    );
    assert!(src.contains("bed_heat_capacity_mj_per_k = 0.5"));
    src
}

fn tick(engine: &mut Engine, n: u64) {
    for _ in 0..n {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: {e}"));
    }
}

fn bed_node(s: &Snapshot) -> &NodeSnapshot {
    s.nodes.iter().find(|n| n.name == "bed").expect("a bed")
}

fn bed_k(engine: &Engine) -> f64 {
    let id = engine.graph.find_node("bed").expect("a bed");
    match &engine.graph.node(id).kind {
        NodeKind::BedReactor { bed, .. } => bed.temperature.value(),
        other => panic!("bed is a bed reactor, not {other:?}"),
    }
}

fn wall_state(engine: &Engine) -> WallState {
    let id = engine.graph.find_node("bed").expect("a bed");
    match &engine.graph.node(id).kind {
        NodeKind::BedReactor { wall, .. } => wall.state,
        other => panic!("bed is a bed reactor, not {other:?}"),
    }
}

fn hole_area(engine: &Engine) -> f64 {
    let id = engine.graph.find_node("bed").expect("a bed");
    let NodeKind::BedReactor { wall, .. } = &engine.graph.node(id).kind else {
        panic!("bed is a bed reactor")
    };
    match engine
        .graph
        .pipe(wall.hole.expect("a loaded bed owns a hole"))
        .leak
    {
        LeakRole::Orifice { area } => area.value(),
        other => panic!("the hole is an orifice, not {other:?}"),
    }
}

fn leak(s: &Snapshot) -> f64 {
    s.edges
        .iter()
        .find(|e| e.name == "product_line__leak")
        .expect("the wall's hole")
        .stream
        .mass_flow
        .value()
}

fn fire(s: &Snapshot) -> f64 {
    bed_node(s)
        .wall_fire_w
        .expect("a bed publishes its wall fire from tick 1")
}

fn edge_id(engine: &Engine, name: &str) -> refinery_core::graph::EdgeId {
    engine
        .graph
        .edge_ids()
        .find(|e| engine.graph.pipe(*e).name == name)
        .unwrap_or_else(|| panic!("an edge named '{name}'"))
}

fn node_id(engine: &Engine, name: &str) -> refinery_core::graph::NodeId {
    engine
        .graph
        .find_node(name)
        .unwrap_or_else(|| panic!("a node named '{name}'"))
}

fn apply(engine: &mut Engine, cmd: Command) {
    engine
        .apply(cmd)
        .unwrap_or_else(|e| panic!("the command must be accepted: {e}"));
}

fn refused(engine: &mut Engine, cmd: Command) -> String {
    engine
        .apply(cmd)
        .expect_err("the command must be refused")
        .to_string()
}

/// Run until the wall fails; panics past `bound` ticks rather than spinning
/// for ever on an engine that never fails it.
fn run_to_failure(engine: &mut Engine, bound: u64) {
    for _ in 0..bound {
        if wall_state(engine).is_failed() {
            return;
        }
        tick(engine, 1);
    }
    assert!(
        wall_state(engine).is_failed(),
        "the wall must fail within {bound} ticks; the bed is at {:.2} K",
        bed_k(engine)
    );
}

fn patch(engine: &mut Engine) {
    let pipe = edge_id(engine, "product_line");
    apply(
        engine,
        Command::PuncturePipe {
            edge: pipe,
            area: SquareMeter(0.0),
        },
    );
}

fn set_feed_c(engine: &mut Engine, celsius: f64) {
    let feed = node_id(engine, "vgo_feed");
    apply(
        engine,
        Command::SetSourceTemperature {
            node: feed,
            temperature: Kelvin(celsius + 273.15),
        },
    );
}

/// **Gate 1 — the wall fails on the tick after the bed reaches its limit.**
///
/// The pass reads the START-of-tick bed, which is the end of the last tick, so
/// the tick the bed first ENDS at or past 550 °C is still intact, and the next
/// one is `Failed { at_tick }` with the hole open to exactly the declared area
/// — and leaking, and burning, from that same tick. Both fidelities, on the
/// shipped file at its own heat capacity, and the tick is the measured one the
/// design note quotes.
#[test]
fn the_wall_fails_on_the_tick_after_the_bed_reaches_its_limit() {
    for solver in ["newton", "simple"] {
        let mut engine = build_src(DEMO, solver);
        let mut reached = None;
        for t in 1..=FAIL_TICK + 5 {
            tick(&mut engine, 1);
            match (reached, wall_state(&engine)) {
                (None, WallState::Intact) => {
                    assert_eq!(hole_area(&engine), 0.0, "{solver}: intact, tick {t}");
                    assert_eq!(fire(&engine.snapshot()), 0.0, "{solver}: intact, tick {t}");
                    if bed_k(&engine) >= LIMIT_K {
                        reached = Some(t);
                    }
                }
                (Some(r), WallState::Failed { at_tick }) => {
                    assert_eq!(at_tick, r + 1, "{solver}: fails the tick after");
                    assert_eq!(at_tick, t, "{solver}: latched on the tick it failed");
                    assert_eq!(at_tick, FAIL_TICK, "{solver}: the measured tick");
                    assert_eq!(hole_area(&engine), HOLE_M2, "{solver}: the declared hole");
                    let s = engine.snapshot();
                    assert!(leak(&s) > 1.0, "{solver}: it leaks {} kg/s", leak(&s));
                    assert!(fire(&s) > 40e6, "{solver}: it burns {} W", fire(&s));
                    break;
                }
                (r, w) => panic!("{solver}: tick {t}, reached {r:?}, wall {w:?}"),
            }
        }
        assert!(
            wall_state(&engine).is_failed(),
            "{solver}: the wall must have failed by tick {}",
            FAIL_TICK + 5
        );
    }
}

/// **Gate 2 — the fire is the leak times the heating value**, to the bit, on
/// every tick the wall is failed and the hole leaks, and zero on an intact
/// wall and on a patched hole.
#[test]
fn the_fire_is_the_leak_times_the_heating_value() {
    let mut engine = build_src(&light(), "newton");
    for _ in 0..30 {
        tick(&mut engine, 1);
        assert!(
            !wall_state(&engine).is_failed(),
            "the light bed fails later"
        );
        assert_eq!(fire(&engine.snapshot()), 0.0);
    }
    run_to_failure(&mut engine, 500);
    for _ in 0..50 {
        let s = engine.snapshot();
        assert!(leak(&s) > 0.0);
        assert_eq!(fire(&s), leak(&s).max(0.0) * HEATING_J_PER_KG);
        tick(&mut engine, 1);
    }
    patch(&mut engine);
    tick(&mut engine, 1);
    assert_eq!(
        fire(&engine.snapshot()),
        0.0,
        "a patched hole burns nothing"
    );
    assert!(wall_state(&engine).is_failed(), "patching repairs nothing");
}

/// **Gate 3 — the declared share of the fire heats the bed.**
///
/// The feed valve is shut on a failed, leaking bed: nothing flows through the
/// bed, so nothing reacts in it and nothing carries its heat away, while the
/// product line still drains out of the hole from downstream and burns. The
/// bed's first law is then the fire alone: `C·ΔT/dt = s·L`, tick by tick. A
/// twin with a share of zero holds its bed exactly.
#[test]
fn the_declared_share_of_the_fire_heats_the_bed() {
    for share in [SHARE, 0.0] {
        let src = light().replace(
            "wall_fire_share = 0.1",
            &format!("wall_fire_share = {share:?}"),
        );
        let mut engine = build_src(&src, "newton");
        run_to_failure(&mut engine, 500);
        let valve = node_id(&engine, "feed_valve");
        apply(
            &mut engine,
            Command::SetValveOpening {
                node: valve,
                opening: 0.0,
            },
        );
        tick(&mut engine, 5);
        for t in 0..20 {
            let start = bed_k(&engine);
            tick(&mut engine, 1);
            let s = engine.snapshot();
            let through = bed_node(&s).reaction_heat_w.expect("a bed reports");
            assert_eq!(through, 0.0, "share {share}: no flow, no reaction");
            let burnt = fire(&s);
            assert!(
                burnt > 1e6,
                "share {share}: the hole still burns ({burnt} W)"
            );
            let heated = (bed_k(&engine) - start) * LIGHT_C_J_PER_K / DT_S;
            if share == 0.0 {
                assert_eq!(heated, 0.0, "a share of zero heats nothing, tick {t}");
            } else {
                let expected = share * burnt;
                assert!(
                    ((heated - expected) / expected).abs() < 1e-9,
                    "tick {t}: the bed took {heated} W, s·L is {expected} W"
                );
            }
        }
    }
}

/// **Gate 4 — a hand hole in an INTACT wall leaks and does not burn; in a
/// FAILED wall it burns.**
#[test]
fn a_hand_hole_burns_only_in_a_failed_wall() {
    let mut engine = build_src(&light(), "newton");
    let pipe = edge_id(&engine, "product_line");
    apply(
        &mut engine,
        Command::PuncturePipe {
            edge: pipe,
            area: SquareMeter(HOLE_M2),
        },
    );
    tick(&mut engine, 5);
    let s = engine.snapshot();
    assert!(!wall_state(&engine).is_failed());
    assert!(leak(&s) > 0.5, "an intact wall's hand hole leaks");
    assert_eq!(fire(&s), 0.0, "and burns nothing");

    run_to_failure(&mut engine, 500);
    patch(&mut engine);
    tick(&mut engine, 1);
    assert_eq!(fire(&engine.snapshot()), 0.0);
    apply(
        &mut engine,
        Command::PuncturePipe {
            edge: pipe,
            area: SquareMeter(HOLE_M2),
        },
    );
    tick(&mut engine, 1);
    let s = engine.snapshot();
    assert!(fire(&s) > 1e6, "a hand hole in a failed wall burns");
    assert_eq!(fire(&s), leak(&s) * HEATING_J_PER_KG);
}

/// **Gate 5 — a failure keeps a wider hand hole.**
#[test]
fn a_failure_keeps_a_wider_hand_hole() {
    let mut engine = build_src(&light(), "newton");
    let pipe = edge_id(&engine, "product_line");
    apply(
        &mut engine,
        Command::PuncturePipe {
            edge: pipe,
            area: SquareMeter(5.0 * HOLE_M2),
        },
    );
    run_to_failure(&mut engine, 500);
    assert_eq!(hole_area(&engine), 5.0 * HOLE_M2);
}

/// **Gate 6 — `RepairBedWall`: four refusals, success, a second failure, and
/// the player's way back.**
///
/// In the player's order: not a bed; an intact wall; the hole still open
/// (naming the pipe to patch); the bed still at or past its limit (naming the
/// cure). Then the feed is chilled to 200 °C — the only feed that brings a
/// runaway bed under its limit (§63.0) — and the repair is admitted the first
/// tick the bed ends below 550 °C, moves no hole, and re-arms the wall: put back
/// on the 385 °C feed it runs away again and fails again.
#[test]
fn repair_bed_wall_refusals_success_and_a_second_failure() {
    let mut engine = build_src(&light(), "newton");
    let bed = node_id(&engine, "bed");
    let valve = node_id(&engine, "feed_valve");

    let says = refused(&mut engine, Command::RepairBedWall { node: valve });
    assert!(says.contains("not a catalyst bed"), "{says}");
    let says = refused(&mut engine, Command::RepairBedWall { node: bed });
    assert!(says.contains("intact wall"), "{says}");

    run_to_failure(&mut engine, 500);
    let first = wall_state(&engine);
    tick(&mut engine, 100);
    let says = refused(&mut engine, Command::RepairBedWall { node: bed });
    assert!(
        says.contains("still leaks") && says.contains("'product_line'"),
        "{says}"
    );

    patch(&mut engine);
    let says = refused(&mut engine, Command::RepairBedWall { node: bed });
    assert!(
        says.contains("at or past its wall's 550.0 °C limit") && says.contains("colder feed"),
        "{says}"
    );

    set_feed_c(&mut engine, 200.0);
    let mut below = false;
    for _ in 0..1_000 {
        tick(&mut engine, 1);
        if bed_k(&engine) < LIMIT_K {
            below = true;
            break;
        }
        let says = refused(&mut engine, Command::RepairBedWall { node: bed });
        assert!(says.contains("limit"), "{says}");
    }
    assert!(below, "a 200 °C feed brings the bed under its limit");
    apply(&mut engine, Command::RepairBedWall { node: bed });
    assert_eq!(wall_state(&engine), WallState::Intact);
    assert_eq!(hole_area(&engine), 0.0, "a repair moves no hole");
    let says = refused(&mut engine, Command::RepairBedWall { node: bed });
    assert!(says.contains("intact wall"), "{says}");

    set_feed_c(&mut engine, 385.0);
    run_to_failure(&mut engine, 2_000);
    assert_ne!(
        wall_state(&engine),
        first,
        "a second failure, on a later tick"
    );
    assert_eq!(hole_area(&engine), HOLE_M2);
}

/// **Gate 7 — every bed in the repo owns a hole**, on its outlet pipe's
/// midpoint, named for that pipe and dormant at load; the demo vents to the
/// plant's outside the loader made for it.
#[test]
fn every_bed_in_the_repo_owns_a_wall_hole() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
    let mut beds = 0;
    let mut files: Vec<_> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "toml"))
        .collect();
    files.sort();
    for path in files {
        let file = refinery_scenarios::load_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        let engine = refinery_scenarios::build_engine(&file)
            .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        for nid in engine.graph.node_ids() {
            let NodeKind::BedReactor { wall, .. } = &engine.graph.node(nid).kind else {
                continue;
            };
            beds += 1;
            assert_eq!(wall.state, WallState::Intact);
            let hole = wall
                .hole
                .unwrap_or_else(|| panic!("{}: no hole", path.display()));
            assert_eq!(
                engine.graph.pipe(hole).leak,
                LeakRole::Orifice {
                    area: SquareMeter::ZERO
                },
                "{}: a dormant orifice",
                path.display()
            );
            let outlet = engine
                .graph
                .edge_ids()
                .find(|&e| engine.graph.endpoints(e).0 == nid)
                .expect("a bed has an outlet");
            assert_eq!(
                engine.graph.pipe(outlet).leak,
                LeakRole::Punctureable { orifice: hole },
                "{}: the hole is on the bed's own outlet",
                path.display()
            );
            assert_eq!(
                engine.graph.pipe(hole).name,
                format!("{}__leak", engine.graph.pipe(outlet).name)
            );
        }
    }
    // Seven since M59.1 (docs/DESIGN.md §64): `hydrocracker_hydrogen.toml`'s bed.
    assert_eq!(beds, 7, "six shipped hydrocracker files, seven beds");
    let engine = build_src(DEMO, "newton");
    assert!(engine.graph.find_node("burnout_atmosphere").is_some());
}

/// **Gate 7, the loader's refusals**: each key out of range, a key missing,
/// and a flow meter on a bed's outlet.
#[test]
fn the_loader_refuses_a_bad_wall() {
    let refuse = |from: &str, to: &str, says: &str| {
        let src = DEMO.replace(from, to);
        assert_ne!(src, DEMO, "the fixture must change '{from}'");
        let err = refinery_scenarios::load_str(&src)
            .and_then(|f| refinery_scenarios::build_engine(&f).map(|_| ()))
            .expect_err("the loader must refuse it")
            .to_string();
        assert!(err.contains(says), "{from} → {to}: {err}");
    };
    refuse(
        "wall_failure_c = 550.0",
        "wall_failure_c = 450.0",
        "wall_failure_c",
    );
    refuse(
        "wall_failure_c = 550.0",
        "wall_failure_c = nan",
        "wall_failure_c",
    );
    refuse(
        "wall_rupture_area_cm2 = 1.0",
        "wall_rupture_area_cm2 = 0.0",
        "wall_rupture_area_cm2",
    );
    refuse(
        "fluid_heating_value_mj_per_kg = 42.8",
        "fluid_heating_value_mj_per_kg = -1.0",
        "fluid_heating_value_mj_per_kg",
    );
    refuse(
        "wall_fire_share = 0.1",
        "wall_fire_share = 1.5",
        "wall_fire_share",
    );
    refuse(
        "wall_fire_share = 0.1",
        "wall_fire_share = -0.1",
        "wall_fire_share",
    );
    refuse("wall_fire_share = 0.1\n", "", "wall_fire_share");
    // A flow meter on the bed's outlet, as a trip's measurement.
    let metered = format!(
        "{DEMO}\n[[trips]]\nname = \"low_product\"\n\
         measurement = {{ pipe = \"product_line\", variable = \"flow\" }}\n\
         direction = \"low\"\nlimit_kg_per_s = 1.0\n\
         actions = [{{ valve = \"feed_valve\", position = 0.5 }}]\n"
    );
    let err = refinery_scenarios::load_str(&metered)
        .and_then(|f| refinery_scenarios::build_engine(&f).map(|_| ()))
        .expect_err("a metered bed outlet must be refused")
        .to_string();
    assert!(err.contains("the outlet of catalyst bed 'bed'"), "{err}");
}
