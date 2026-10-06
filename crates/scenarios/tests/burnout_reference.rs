//! M37: a furnace's tube burn-out — damage the ENGINE raises from a state
//! (docs/DESIGN.md §42), gated on `scenarios/furnace_burnout.toml` and on
//! fixtures cut from the shipped files.
//!
//! What is pinned, and against what:
//!
//! 1. the tubes burst on the tick AFTER the coil first ends one at or past its
//!    limit — the burn-out pass reads the start-of-tick state, as a trip does —
//!    and the hole opens to exactly the declared area, on both fidelities;
//! 2. the fire is the leak times the heating value, to the bit, and zero on
//!    intact tubes;
//! 3. the fire is FUEL: it fires through the flame law, so the settled stack
//!    loss is `(Q + L)·(T_c − T_a)/(T_f − T_a)` and the coil never crosses its
//!    flame;
//! 4. the plant's energy books close with the fire counted, and do not without;
//! 5. a hole punched by hand in INTACT tubes leaks and does not burn; in FAILED
//!    tubes it burns;
//! 6. `ReplaceTubes`: its four refusals, its success, and tubes that burst again;
//! 7. every furnace in the repo owns a hole, and the loader's refusals;
//! 8. a GAS furnace bursts and burns its gas through a choked hole (M37.0).
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{LeakRole, NodeKind, TubeState};
use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot};
use refinery_core::units::{SquareMeter, Watt};
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/furnace_burnout.toml");

/// The demo's numbers, by hand from its file.
const LIMIT_K: f64 = 550.0 + 273.15;
const HOLE_M2: f64 = 1.0e-4;
const HEATING_J_PER_KG: f64 = 42.8 * 1e6;
const DUTY_W: f64 = 1.5e6;
const FLAME_K: f64 = 1951.1 + 273.15;
const AIR_K: f64 = 293.15;
const COIL_C_J_PER_K: f64 = 1.5e6;
const DT_S: f64 = 1.0;
/// Measured (both fidelities): the first tick whose top pass finds the coil at
/// or past its limit. The coil ends tick 1 144 at 550.08 °C.
const BURST_TICK: u64 = 1_145;

fn build_src(src: &str, solver: &str) -> Engine {
    let src = src.replacen(r#"flow = "newton""#, &format!(r#"flow = "{solver}""#), 1);
    let file = refinery_scenarios::load_str(&src).expect("the scenario must parse");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("it must build: {e}"))
}

fn tick(engine: &mut Engine, n: u64) {
    for _ in 0..n {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: {e}"));
    }
}

fn heater(s: &Snapshot) -> &NodeSnapshot {
    s.nodes
        .iter()
        .find(|n| n.name == "heater")
        .expect("a heater")
}

fn coil_k(engine: &Engine) -> f64 {
    let id = engine.graph.find_node("heater").expect("a heater");
    match &engine.graph.node(id).kind {
        NodeKind::Furnace { coil, .. } => coil.temperature.value(),
        other => panic!("heater is a furnace, not {other:?}"),
    }
}

fn tube_state(engine: &Engine) -> TubeState {
    let id = engine.graph.find_node("heater").expect("a heater");
    match &engine.graph.node(id).kind {
        NodeKind::Furnace { tubes, .. } => tubes.state,
        other => panic!("heater is a furnace, not {other:?}"),
    }
}

fn hole_area(engine: &Engine) -> f64 {
    let id = engine.graph.find_node("heater").expect("a heater");
    let NodeKind::Furnace { tubes, .. } = &engine.graph.node(id).kind else {
        panic!("heater is a furnace")
    };
    match engine
        .graph
        .pipe(tubes.hole.expect("a loaded furnace owns a hole"))
        .leak
    {
        LeakRole::Orifice { area } => area.value(),
        other => panic!("the hole is an orifice, not {other:?}"),
    }
}

fn leak(s: &Snapshot) -> f64 {
    s.edges
        .iter()
        .find(|e| e.name == "heated_line__leak")
        .expect("the burn-out hole")
        .stream
        .mass_flow
        .value()
}

fn fire(s: &Snapshot) -> f64 {
    heater(s)
        .tube_fire_w
        .expect("a furnace publishes its fire from tick 1")
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

fn refused(engine: &mut Engine, cmd: Command) -> String {
    engine
        .apply(cmd)
        .expect_err("the command must be refused")
        .to_string()
}

/// **Gate 1 — the tubes burst on the tick after the coil reaches its limit.**
///
/// The pass reads the START-of-tick coil, which is the end of the last tick, so
/// the tick the coil first ENDS at or past 550 °C is still intact, and the next
/// one is `Failed { at_tick }` with the hole open to exactly the declared area —
/// and leaking, and burning, from that same tick. Both fidelities, and the tick
/// is the measured one the file quotes.
#[test]
fn the_tubes_burst_on_the_tick_after_the_coil_reaches_their_limit() {
    for solver in ["newton", "simple"] {
        let mut engine = build_src(DEMO, solver);
        let mut reached = None;
        for t in 1..=BURST_TICK + 5 {
            tick(&mut engine, 1);
            match (reached, tube_state(&engine)) {
                (None, TubeState::Intact) => {
                    assert_eq!(
                        hole_area(&engine),
                        0.0,
                        "{solver}, tick {t}: intact tubes, no hole"
                    );
                    if coil_k(&engine) >= LIMIT_K {
                        reached = Some(t);
                    }
                }
                (Some(r), TubeState::Failed { at_tick }) => {
                    assert_eq!(
                        at_tick,
                        r + 1,
                        "{solver}: the pass after the coil reached it"
                    );
                    assert_eq!(hole_area(&engine), HOLE_M2, "{solver}: the declared hole");
                }
                (r, state) => {
                    panic!("{solver}, tick {t}: {state:?} with the limit reached at {r:?}")
                }
            }
        }
        assert_eq!(
            reached.map(|r| r + 1),
            Some(BURST_TICK),
            "{solver}: the measured burst tick"
        );
        let s = engine.snapshot();
        assert!(
            leak(&s) > 0.5 && fire(&s) > 1e7,
            "{solver}: it leaks and burns"
        );
    }
}

/// **Gate 2 — the fire is the leak times the heating value, to the bit**, and
/// exactly zero (with exactly zero leak) on every intact tick.
#[test]
fn the_fire_is_what_leaks_times_its_heating_value() {
    let mut engine = build_src(DEMO, "newton");
    for t in 1..=BURST_TICK + 50 {
        tick(&mut engine, 1);
        let s = engine.snapshot();
        if t < BURST_TICK {
            assert_eq!((leak(&s), fire(&s)), (0.0, 0.0), "tick {t}: intact");
        } else {
            assert_eq!(
                fire(&s).to_bits(),
                (leak(&s).max(0.0) * HEATING_J_PER_KG).to_bits(),
                "tick {t}: {} W of fire on {} kg/s",
                fire(&s),
                leak(&s)
            );
        }
    }
}

/// **Gate 3 — the fire is FUEL.** Settled, the coil absorbs nothing net and the
/// stack carries `K_f·(T_c − T_a)` with `K_f = (Q + L)/(T_f − T_a)`: the duty AND
/// the fire. A fire kept out of the flame law (a commanded fire's treatment)
/// would put `K_f = Q/(T_f − T_a)` here, 24 times smaller. And the coil never
/// passes its flame, fire and all.
#[test]
fn the_fire_burns_through_the_flame_law() {
    let mut engine = build_src(DEMO, "newton");
    for _ in 1..=3_000 {
        tick(&mut engine, 1);
        assert!(coil_k(&engine) < FLAME_K, "the coil passed its flame");
    }
    let s = engine.snapshot();
    let settled = coil_k(&engine);
    let fuel = DUTY_W + fire(&s);
    let flue = heater(&s)
        .flue_loss_w
        .expect("a furnace publishes its stack");
    approx::assert_relative_eq!(
        flue,
        fuel * (settled - AIR_K) / (FLAME_K - AIR_K),
        max_relative = 1e-9
    );
    let duty_only = DUTY_W * (settled - AIR_K) / (FLAME_K - AIR_K);
    assert!(
        flue > 20.0 * duty_only,
        "the stack carries the fire's share too"
    );
}

/// The enthalpy crossing the plant's boundary, plus the duty and the fire, less
/// the stack and plus every pipe's friction [W] — everything but the coil's
/// stored heat. An inflow's friction is inside its published (outlet) flux; an
/// outflow's is too, and is credited back as the source it is.
/// `with_fire = false` is the counterfactual.
fn boundary_power(engine: &Engine, s: &Snapshot, with_fire: bool) -> f64 {
    let outside = |id| {
        matches!(
            s.nodes
                .iter()
                .find(|n| n.id == id)
                .expect("the edge's node")
                .kind,
            NodeKind::Source { .. } | NodeKind::Sink { .. } | NodeKind::Atmosphere
        )
    };
    let mut power = 0.0;
    for e in &s.edges {
        let flux = engine
            .enthalpy()
            .stream_enthalpy_flux(&engine.slate, &e.stream)
            .expect("the engine priced this stream")
            .value();
        match (outside(e.from), outside(e.to)) {
            (true, false) => power += flux,
            // An OUTFLOW's published temperature is its outlet, so the friction
            // it dissipated on the way out is inside its flux — and that friction
            // is a source inside the boundary, credited here (the burn-out hole,
            // venting at a bar's drop, dissipates ~90 W of it).
            (false, true) => power += e.dissipation_w - flux,
            (false, false) => power += e.dissipation_w,
            (true, true) => panic!("an edge with both ends outside"),
        }
    }
    let h = heater(s);
    power += DUTY_W - h.flue_loss_w.expect("a stack") + h.heat_input_w;
    if with_fire {
        power += fire(s);
    }
    power
}

/// **Gate 4 — the books close with the fire counted, and not without.** The
/// coil is this plant's only store: `C·ΔT_c/dt` against the boundary power,
/// every tick from load to well past the burst.
#[test]
fn the_books_close_with_the_fire_counted() {
    let mut engine = build_src(DEMO, "newton");
    let (mut worst, mut worst_without) = (0.0_f64, 0.0_f64);
    for t in 1..=BURST_TICK + 200 {
        let before = coil_k(&engine);
        tick(&mut engine, 1);
        let stored = COIL_C_J_PER_K * (coil_k(&engine) - before) / DT_S;
        let s = engine.snapshot();
        let scale = DUTY_W + fire(&s);
        worst = worst.max((boundary_power(&engine, &s, true) - stored).abs() / scale);
        if t >= BURST_TICK {
            worst_without =
                worst_without.max((boundary_power(&engine, &s, false) - stored).abs() / scale);
        }
    }
    assert!(
        worst < 1e-9,
        "the books leave {worst:e} of the firing unaccounted"
    );
    assert!(
        worst_without > 0.5,
        "without the fire the books must not close: {worst_without:e}"
    );
}

/// **Gate 5 — a hole punched by hand burns only in FAILED tubes.** Intact
/// tubes with a hand-punched hole leak gas oil and light nothing; once the
/// tubes have burst, patched the hole and punched it again, the fire returns,
/// since the tubes stay failed until they are replaced.
#[test]
fn a_hand_punched_hole_burns_only_in_failed_tubes() {
    let mut engine = build_src(DEMO, "newton");
    tick(&mut engine, 100);
    let line = edge_id(&engine, "heated_line");
    engine
        .apply(Command::PuncturePipe {
            edge: line,
            area: SquareMeter(HOLE_M2),
        })
        .expect("every furnace outlet is punctureable");
    tick(&mut engine, 20);
    let s = engine.snapshot();
    assert!(leak(&s) > 0.5, "the hand hole leaks");
    assert_eq!(fire(&s), 0.0, "intact tubes: a leak, not a fire");
    assert_eq!(tube_state(&engine), TubeState::Intact);

    // Patch it, let the tubes burst on their own, patch that, and punch again.
    engine
        .apply(Command::PuncturePipe {
            edge: line,
            area: SquareMeter(0.0),
        })
        .expect("a patch");
    tick(&mut engine, BURST_TICK);
    assert!(tube_state(&engine).is_failed(), "the coil burst them");
    engine
        .apply(Command::PuncturePipe {
            edge: line,
            area: SquareMeter(0.0),
        })
        .expect("a patch");
    tick(&mut engine, 1);
    assert_eq!(fire(&engine.snapshot()), 0.0, "patched: the fire is out");
    engine
        .apply(Command::PuncturePipe {
            edge: line,
            area: SquareMeter(HOLE_M2),
        })
        .expect("punctured again");
    tick(&mut engine, 1);
    assert!(fire(&engine.snapshot()) > 1e7, "failed tubes burn again");
}

/// **Gate 6 — replacing the tubes.** Refused on a non-furnace, on intact
/// tubes, while the hole is open (patch first) and while the coil is still at or
/// past its limit (cool first); then admitted, which re-arms them and moves
/// nothing else; and re-lit, the new tubes burst again, on a later tick.
/// A burst never SHRINKS a hole: punched wider by hand before the tubes burst,
/// the hole stays at the hand's area when they do, and burns there.
#[test]
fn a_burst_keeps_a_wider_hand_hole() {
    let mut engine = build_src(DEMO, "newton");
    tick(&mut engine, 1_000);
    let line = edge_id(&engine, "heated_line");
    engine
        .apply(Command::PuncturePipe {
            edge: line,
            area: SquareMeter(2.0 * HOLE_M2),
        })
        .expect("a hand hole twice the rupture");
    // Bounded: the M37.1 mutation pass found this loop spinning for ever on an
    // engine that never bursts, which is a hang rather than a failure.
    let mut waited = 0;
    while !tube_state(&engine).is_failed() {
        tick(&mut engine, 1);
        waited += 1;
        assert!(waited < 5_000, "the tubes never burst");
    }
    assert_eq!(hole_area(&engine), 2.0 * HOLE_M2, "the wider hole is kept");
    // The hole opens at the TOP of the burst tick, so that tick's own solve
    // already carries the leak, and its sweep already burns it.
    assert!(
        fire(&engine.snapshot()) > 0.0,
        "it burns from its burst tick"
    );
}

#[test]
fn replacing_the_tubes_rearms_them_and_nothing_else() {
    let mut engine = build_src(DEMO, "newton");
    let heater_id = node_id(&engine, "heater");
    let line = edge_id(&engine, "heated_line");
    let valve_id = node_id(&engine, "feed_valve");
    let says = refused(&mut engine, Command::ReplaceTubes { node: valve_id });
    assert!(says.contains("is not a furnace"), "{says}");
    let says = refused(&mut engine, Command::ReplaceTubes { node: heater_id });
    assert!(says.contains("intact tubes"), "{says}");

    tick(&mut engine, BURST_TICK + 10);
    let says = refused(&mut engine, Command::ReplaceTubes { node: heater_id });
    assert!(
        says.contains("patch it first") && says.contains("'heated_line'"),
        "{says}"
    );
    engine
        .apply(Command::PuncturePipe {
            edge: line,
            area: SquareMeter(0.0),
        })
        .expect("a patch");
    let says = refused(&mut engine, Command::ReplaceTubes { node: heater_id });
    assert!(says.contains("cool"), "{says}");

    // Cut the fuel and let the oil cool the coil below its limit.
    engine
        .apply(Command::SetFurnaceDuty {
            node: heater_id,
            duty: Watt::ZERO,
        })
        .expect("cut");
    let mut cooled = 0;
    while coil_k(&engine) >= LIMIT_K {
        tick(&mut engine, 1);
        cooled += 1;
        assert!(cooled < 5_000, "the coil never cooled");
    }
    let hole_before = hole_area(&engine);
    engine
        .apply(Command::ReplaceTubes { node: heater_id })
        .expect("cooled and patched: new tubes");
    assert_eq!(tube_state(&engine), TubeState::Intact);
    assert_eq!(hole_area(&engine), hole_before, "replacing moves no hole");

    engine
        .apply(Command::SetFurnaceDuty {
            node: heater_id,
            duty: Watt(DUTY_W),
        })
        .expect("relit");
    let mut more = 0;
    while !tube_state(&engine).is_failed() {
        tick(&mut engine, 1);
        more += 1;
        assert!(more < 5_000, "the new tubes never burst");
    }
    let TubeState::Failed { at_tick } = tube_state(&engine) else {
        unreachable!()
    };
    assert!(
        at_tick > BURST_TICK + 10,
        "a second burst, later: {at_tick}"
    );
    assert_eq!(hole_area(&engine), HOLE_M2);
}

/// **Gate 7 — every furnace in the repo owns a hole**, an orifice hung off its
/// outlet pipe's split, and the loader-made atmosphere is named as its first
/// builder would have named it.
#[test]
fn every_furnace_in_the_repo_owns_a_burnout_hole() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
    let mut furnaces = 0;
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
            let NodeKind::Furnace { tubes, .. } = &engine.graph.node(nid).kind else {
                continue;
            };
            furnaces += 1;
            let hole = tubes
                .hole
                .unwrap_or_else(|| panic!("{}: no hole", path.display()));
            assert!(matches!(
                engine.graph.pipe(hole).leak,
                LeakRole::Orifice { .. }
            ));
            let outlet = engine
                .graph
                .edge_ids()
                .find(|e| engine.graph.pipe(*e).leak == LeakRole::Punctureable { orifice: hole })
                .unwrap_or_else(|| panic!("{}: no punctureable half", path.display()));
            assert_eq!(
                engine.graph.endpoints(outlet).0,
                nid,
                "the furnace's OWN outlet"
            );
        }
    }
    assert_eq!(
        furnaces, 19,
        "fifteen furnace plants, the burn-out demo, M40's self-resetting trip demo, \
         M43's burst-during-a-stop demo and M44's start-permissive demo"
    );
    let heater_only = build_src(
        include_str!("../../../scenarios/furnace_heater.toml"),
        "newton",
    );
    assert!(
        heater_only.graph.find_node("burnout_atmosphere").is_some(),
        "no tank: its own"
    );
}

/// The loader's refusals: a furnace's outlet cannot be metered by flow (it is
/// split), and the loader-made atmosphere's name cannot be taken by another
/// kind of node.
#[test]
fn the_loader_refuses_what_the_burnout_hole_makes_ambiguous() {
    let base = include_str!("../../../scenarios/furnace_heater.toml");
    let metered = format!(
        "{base}\n[[trips]]\nname = \"outlet_flow\"\nmeasurement = {{ pipe = \"transfer_line\", \
         variable = \"flow\" }}\ndirection = \"low\"\nlimit_kg_per_s = 1.0\nactions = \
         [{{ furnace = \"heater\" }}]\n"
    );
    let file = refinery_scenarios::load_str(&metered).expect("it parses");
    let says = refinery_scenarios::build_engine(&file)
        .err()
        .expect("refused")
        .to_string();
    assert!(says.contains("the outlet of furnace 'heater'"), "{says}");

    let taken = base
        .replace("[nodes.product]", "[nodes.burnout_atmosphere]")
        .replace("to = \"product\"", "to = \"burnout_atmosphere\"");
    let file = refinery_scenarios::load_str(&taken).expect("it parses");
    let says = refinery_scenarios::build_engine(&file)
        .err()
        .expect("refused")
        .to_string();
    assert!(says.contains("which is not an atmosphere"), "{says}");
}

/// **Gate 8 — a GAS furnace bursts and burns its gas** (M37.0's hole, §41): the
/// shipped gas drum with its limit lowered under where it settles. Methane
/// vents choked through the hole and burns at 50 MJ/kg.
#[test]
fn a_gas_furnace_bursts_and_burns_its_gas() {
    let src = include_str!("../../../scenarios/fired_gas_drum.toml")
        .replace("tube_failure_c = 850.0", "tube_failure_c = 600.0");
    let mut engine = build_src(&src, "newton");
    let mut t = 0;
    while !tube_state(&engine).is_failed() {
        tick(&mut engine, 1);
        t += 1;
        assert!(t < 20_000, "the gas coil never reached 600 °C");
    }
    tick(&mut engine, 5);
    let s = engine.snapshot();
    let hole = s
        .edges
        .iter()
        .find(|e| e.name == "transfer_line__leak")
        .expect("the gas drum's hole");
    let leaked = hole.stream.mass_flow.value();
    let junction = s
        .nodes
        .iter()
        .find(|n| n.name == "transfer_line__leak_point")
        .expect("its junction")
        .pressure_pa;
    let drop_ratio = (junction - refinery_core::units::P_ATM.value()) / junction;
    assert!(
        leaked > 0.0 && drop_ratio > 0.45,
        "a choked gas leak: {leaked} kg/s at x = {drop_ratio}"
    );
    let fire = heater(&s).tube_fire_w.expect("a fire");
    assert_eq!(fire.to_bits(), (leaked * (50.0 * 1e6)).to_bits());
}

/// **The M36 dry-fired demo bursts too, with nothing to burn.** Its coil
/// passes 550 °C once its feed has failed, so its tubes burst on tick 1 766 on
/// both fidelities — and water through a dry line is a hole that leaks nothing
/// and burns nothing: the hole carries at most ~1e-10 kg/s of rounding, and the
/// fire is exactly zero on every tick.
#[test]
fn the_dry_fired_demo_bursts_with_nothing_to_burn() {
    let src = include_str!("../../../scenarios/furnace_dry_fired.toml");
    for solver in ["newton", "simple"] {
        let mut engine = build_src(src, solver);
        let mut burst = None;
        for t in 1..=2_500u64 {
            tick(&mut engine, 1);
            if burst.is_none() && tube_state(&engine).is_failed() {
                burst = Some(t);
            }
            let s = engine.snapshot();
            assert_eq!(fire(&s), 0.0, "{solver}, tick {t}: water does not burn");
            let leaked = s
                .edges
                .iter()
                .find(|e| e.name == "heated_line__leak")
                .expect("the hole")
                .stream
                .mass_flow
                .value();
            assert!(
                leaked.abs() < 1e-9,
                "{solver}, tick {t}: a dry line leaks {leaked}"
            );
        }
        assert_eq!(burst, Some(1_766), "{solver}: the measured burst tick");
    }
}
