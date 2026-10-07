//! M52 — a supply's pressure and temperature, and a destination's pressure, by
//! command (docs/DESIGN.md §57, ledger rows F5 and B46).
//!
//! `Command::SetReservoirPressure` moves a source's or a sink's pinned
//! pressure; `Command::SetSourceTemperature` moves a source's temperature. Both
//! take hold on the next tick. These gates hold the M50 demo,
//! `scenarios/pump_cavitation_flow_limit.toml`, to a hand calculation at the
//! moved supply — written out here, not read back from the solver — and to the
//! cold start a file edited to the same value gives, on both fidelities and for
//! every ordered move between the values tried. They pin the one-tick lag on a
//! temperature step, the refusals (a boiling supply among them, the user's
//! DECISION), and `NodeSnapshot::supply_boiling`, which says on every supply
//! whether the plant could check at all ("accept it, but say so").

use refinery_core::graph::{NodeId, NodeKind, PumpSuction};
use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot, SupplyBoiling};
use refinery_core::units::{Kelvin, Pascal};
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/pump_cavitation_flow_limit.toml");
const FURNACE: &str = include_str!("../../../scenarios/furnace_heater.toml");
const GAS: &str = include_str!("../../../scenarios/gas_line.toml");
const LEAKY: &str = include_str!("../../../scenarios/leaking_line.toml");

const G: f64 = 9.806_65;

// The demo's own numbers, restated so the hand calculation below is computed
// outside the engine (as `pump_cavitation_reference.rs` restates them).
const SOURCE_PA: f64 = 2.4e5;
const SINK_PA: f64 = 1.5e5;
const FRICTION: f64 = 0.02; // the schema default; no pipe declares one
const SUCTION: (f64, f64) = (60.0, 0.08);
const DISCHARGE: (f64, f64) = (20.0, 0.10);
const FEED: (f64, f64) = (25.0, 0.10);
const KV: f64 = 80.0;
const OPENING: f64 = 0.6;
const H0_M: f64 = 40.0;
const A: f64 = 800.0;
const NPSH3_M: f64 = 3.0;

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the scenario must parse");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant must build: {e}"))
}

fn run(engine: &mut Engine, ticks: u64) {
    for t in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
    }
}

fn node<'a>(snapshot: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    snapshot
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("the plant declares a node '{name}'"))
}

fn id(engine: &Engine, name: &str) -> NodeId {
    node(&engine.snapshot(), name).id
}

/// Mass flow [kg/s] on the pipe named `name`.
fn flow(snapshot: &Snapshot, name: &str) -> f64 {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("the plant declares a pipe '{name}'"))
        .stream
        .mass_flow
        .value()
}

/// The bubble pressure [Pa] the tick's solve read off the pump's kind, which
/// the engine set at the top of the tick from the tick before.
fn solved_against(snapshot: &Snapshot) -> f64 {
    match node(snapshot, "feed_pump").kind {
        NodeKind::Pump {
            suction:
                Some(PumpSuction {
                    bubble_pressure: Some(bubble),
                    ..
                }),
            ..
        } => bubble.value(),
        ref other => panic!("the demo pump carries a bubble pressure after tick 1: {other:?}"),
    }
}

fn rho() -> f64 {
    1.0 / (0.7 / 680.0 + 0.3 / 750.0)
}

fn phi(sigma: f64) -> f64 {
    if sigma <= 0.0 {
        0.0
    } else {
        1.0 - (-(1.0f64 / 0.03).ln() * sigma * sigma).exp()
    }
}

/// The demo's steady state by hand at a given supply and destination pressure
/// [Pa]: `pump_cavitation_reference.rs`'s series loop with both ends free.
/// Returns the mass flow [kg/s].
fn hand_calculation(source_pa: f64, sink_pa: f64, bubble: f64) -> f64 {
    let rho = rho();
    let pipe = |(length, diameter): (f64, f64)| {
        let area = std::f64::consts::PI * diameter * diameter / 4.0;
        FRICTION * length * rho / (2.0 * diameter * area * area)
    };
    let cv = KV / (3600.0 * 1e5f64.sqrt()) * OPENING;
    let valve = (rho / 998.0) / (cv * cv);
    let excess = |q: f64| {
        let suction = source_pa - pipe(SUCTION) * q * q.abs();
        let share = phi((suction - bubble) / (rho * G * NPSH3_M));
        let rise = share * rho * G * (H0_M - A * q * q.abs());
        suction + rise - (pipe(DISCHARGE) + pipe(FEED) + valve) * q * q.abs() - sink_pa
    };
    let (mut low, mut high) = (-1.0, 1.0);
    for _ in 0..200 {
        let mid = 0.5 * (low + high);
        if excess(mid) > 0.0 {
            low = mid;
        } else {
            high = mid;
        }
    }
    low * rho
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

fn on_solver(src: &str, solver: &str) -> String {
    src.replace("flow = \"newton\"", &format!("flow = \"{solver}\""))
}

/// The demo with its supply declared at `bar` and `celsius`, and its
/// destination at `sink_bar` — the cold start a command is graded against.
fn declared(src: &str, bar: f64, celsius: f64, sink_bar: f64) -> String {
    let edited = src
        .replace(
            "pressure_bar = 2.4\ntemperature_c = 110.0",
            &format!("pressure_bar = {bar:?}\ntemperature_c = {celsius:?}"),
        )
        .replace(
            "pressure_bar = 1.5\ntemperature_c = 110.0",
            &format!("pressure_bar = {sink_bar:?}\ntemperature_c = 110.0"),
        );
    assert!(
        edited.contains(&format!(
            "pressure_bar = {bar:?}\ntemperature_c = {celsius:?}"
        )),
        "the supply edit went in"
    );
    edited
}

fn set_pressure(engine: &mut Engine, name: &str, bar: f64) -> Result<(), String> {
    let node = id(engine, name);
    engine
        .apply(Command::SetReservoirPressure {
            node,
            pressure: Pascal(bar * 1e5),
        })
        .map_err(|e| e.to_string())
}

fn set_temperature(engine: &mut Engine, name: &str, celsius: f64) -> Result<(), String> {
    let node = id(engine, name);
    engine
        .apply(Command::SetSourceTemperature {
            node,
            temperature: Kelvin(celsius + 273.15),
        })
        .map_err(|e| e.to_string())
}

/// **The cure, by hand** (§57 gate 1). Raised from 2.4 to 3.0 bar mid-run, the
/// supply's new working point is the hand calculation's at 3.0 bar, and the pump
/// has back much of the head it was losing. Raising the destination from 1.5 to
/// 2.0 bar instead cuts the flow to the hand calculation's there.
#[test]
fn a_moved_supply_or_destination_lands_where_the_hand_calculation_puts_it() {
    let mut engine = build(DEMO);
    run(&mut engine, 60);
    let before = node(&engine.snapshot(), "feed_pump")
        .pump_suction
        .expect("reported")
        .head_fraction;
    set_pressure(&mut engine, "rundown_source", 3.0).expect("a supply takes a pressure");
    run(&mut engine, 60);
    let s = engine.snapshot();
    let mdot = hand_calculation(3.0e5, SINK_PA, solved_against(&s));
    assert!(
        rel(flow(&s, "discharge"), mdot) < 1e-4,
        "flow {} against {mdot}",
        flow(&s, "discharge")
    );
    let after = node(&s, "feed_pump").pump_suction.unwrap().head_fraction;
    assert!(
        before < 0.3 && after > 0.6,
        "the head comes back: {before} -> {after}"
    );

    let mut engine = build(DEMO);
    run(&mut engine, 60);
    let open = flow(&engine.snapshot(), "discharge");
    set_pressure(&mut engine, "unit_feed", 2.0).expect("a destination takes a pressure");
    run(&mut engine, 60);
    let s = engine.snapshot();
    let mdot = hand_calculation(SOURCE_PA, 2.0e5, solved_against(&s));
    assert!(rel(flow(&s, "discharge"), mdot) < 1e-4);
    assert!(flow(&s, "discharge") < open, "back-pressure costs flow");
}

/// **Cooling the liquid is the stronger cure** (§57 gate 2): 20 K off the
/// supply gives the pump nearly its whole head back at the same opening, and
/// the flow is the hand calculation's against the colder liquid's bubble
/// pressure.
#[test]
fn cooling_the_supply_gives_the_head_back() {
    let mut engine = build(DEMO);
    run(&mut engine, 60);
    set_temperature(&mut engine, "rundown_source", 90.0).expect("a supply takes a temperature");
    run(&mut engine, 60);
    let s = engine.snapshot();
    let bubble = solved_against(&s);
    assert!(
        bubble < 1.1e5,
        "the colder naphtha boils lower: {bubble} Pa"
    );
    let mdot = hand_calculation(SOURCE_PA, SINK_PA, bubble);
    assert!(
        rel(flow(&s, "discharge"), mdot) < 1e-4,
        "flow {} against {mdot}",
        flow(&s, "discharge")
    );
    assert!(node(&s, "feed_pump").pump_suction.unwrap().head_fraction > 0.95);
}

/// **Every move mid-run lands where a cold start at the new value does, on both
/// fidelities** (§57 gate 3; M51's gate, at the pinned end). A supply step is a
/// sudden change in the pump's suction, the kind of change that made Newton
/// give up in M51. Every ordered pair of six supply pressures, of six supply
/// temperatures, and of three destination pressures; each step taken on a plant
/// settled 30 ticks, held to the cold answer after 30 more and to an iteration
/// cap on every one of those ticks.
#[test]
fn every_supply_move_mid_run_lands_on_the_cold_answer() {
    const PRESSURES: [f64; 6] = [2.0, 2.2, 2.4, 2.6, 2.8, 3.0];
    const TEMPERATURES: [f64; 6] = [90.0, 100.0, 105.0, 110.0, 115.0, 120.0];
    const SINKS: [f64; 3] = [1.0, 1.5, 2.0];
    /// One lever: the values it moves between, how a command moves it, the
    /// file a cold start declares for a value (supply bar, supply °C, destination
    /// bar), and Newton's iteration cap.
    struct Lever {
        name: &'static str,
        values: &'static [f64],
        mover: fn(&mut Engine, f64) -> Result<(), String>,
        file: fn(f64) -> (f64, f64, f64),
        newton_cap: u32,
    }
    // Worst iterations on any tick after a move, measured: Newton 6 (supply
    // pressure), 12 (supply temperature, which swings the pump from nearly its
    // whole head to none — a cold start of this plant takes 9), 5 (destination);
    // the game solver 37, 36 and 35 sweeps, under M51's 80. Newton's cap is what
    // tells a solve that takes the answer from one that crawls to it (M51's
    // failing throttle spent 27 to 50, then gave up).
    let levers = [
        Lever {
            name: "supply pressure",
            values: &PRESSURES,
            mover: |e, v| set_pressure(e, "rundown_source", v),
            file: |v| (v, 110.0, 1.5),
            newton_cap: 8,
        },
        Lever {
            name: "supply temperature",
            values: &TEMPERATURES,
            mover: |e, v| set_temperature(e, "rundown_source", v),
            file: |v| (2.4, v, 1.5),
            newton_cap: 14,
        },
        Lever {
            name: "destination pressure",
            values: &SINKS,
            mover: |e, v| set_pressure(e, "unit_feed", v),
            file: |v| (2.4, 110.0, v),
            newton_cap: 8,
        },
    ];
    for solver in ["newton", "simple"] {
        let src = on_solver(DEMO, solver);
        for &Lever {
            name: lever,
            values,
            mover,
            file,
            newton_cap,
        } in &levers
        {
            let cap = if solver == "newton" { newton_cap } else { 80 };
            let cold: Vec<f64> = values
                .iter()
                .map(|&v| {
                    let (bar, celsius, sink) = file(v);
                    let mut engine = build(&declared(&src, bar, celsius, sink));
                    run(&mut engine, 60);
                    flow(&engine.snapshot(), "discharge")
                })
                .collect();
            for (i, &from) in values.iter().enumerate() {
                for (j, &to) in values.iter().enumerate() {
                    if i == j {
                        continue;
                    }
                    let mut engine = build(&src);
                    mover(&mut engine, from).expect("the start is legal");
                    run(&mut engine, 30);
                    mover(&mut engine, to).expect("the move is legal");
                    let mut worst = 0;
                    for t in 1..=30 {
                        if let Err(e) = engine.tick() {
                            panic!("{solver}: {lever} {from} -> {to}, tick {t} after: {e}");
                        }
                        worst = worst.max(engine.snapshot().solver.iterations);
                    }
                    assert!(
                        worst <= cap,
                        "{solver}: {lever} {from} -> {to} took {worst} iterations, cap {cap}"
                    );
                    let landed = flow(&engine.snapshot(), "discharge");
                    assert!(
                        rel(landed, cold[j]) < 1e-6,
                        "{solver}: {lever} {from} -> {to} lands on {landed} kg/s, a cold \
                         start on {}",
                        cold[j]
                    );
                }
            }
        }
    }
}

/// **The pump hears of a temperature step one tick late** (§57 fork 3). The
/// bubble pressure it solves against is handed over between ticks from the
/// liquid the last tick resolved at its suction, so the first tick after a step
/// still reads the old liquid's; the second reads the new one's. That liquid
/// was warmed by friction at the OLD flow, so tick 2 is close but not landed
/// (measured 2.4e-4 off the cold start); the flow and the friction heat agree
/// geometrically after that (2.1e-7 at tick 3), and by tick 4 the plant is on
/// the cold start to Newton's tolerance. Pinned so the lag stays one tick.
#[test]
fn the_pump_hears_of_a_temperature_step_one_tick_late() {
    let mut engine = build(DEMO);
    run(&mut engine, 60);
    let old = solved_against(&engine.snapshot());
    set_temperature(&mut engine, "rundown_source", 90.0).unwrap();
    run(&mut engine, 1);
    let first = engine.snapshot();
    assert_eq!(
        solved_against(&first).to_bits(),
        old.to_bits(),
        "tick 1 solves against the old liquid"
    );
    let reported = node(&first, "feed_pump")
        .cavitation
        .expect("graded")
        .bubble_pressure_pa;
    assert!(
        reported < 1.1e5,
        "but the tick resolves the new liquid at the pump: {reported}"
    );
    run(&mut engine, 1);
    let second = engine.snapshot();
    assert!(
        rel(solved_against(&second), reported) < 1e-9,
        "tick 2 solves against the liquid tick 1 resolved"
    );
    let mut cold = build(&declared(DEMO, 2.4, 90.0, 1.5));
    run(&mut cold, 60);
    let target = flow(&cold.snapshot(), "discharge");
    let off = rel(flow(&second, "discharge"), target);
    assert!(
        (1e-5..1e-3).contains(&off),
        "tick 2 is near but not on it: {off:e}"
    );
    run(&mut engine, 2);
    let off = rel(flow(&engine.snapshot(), "discharge"), target);
    assert!(off < 1e-7, "tick 4 is on it: {off:e}");
}

/// **A boiling supply is refused, by command and by file** (§57 fork 2, the
/// user's DECISION: "refuse it for today, modeling part-vapour supply in the
/// future" — ledger row B46). At 2.4 bar the demo's naphtha boils from about
/// 120.8 °C; at 110 °C, below about 1.83 bar. A refused command moves nothing.
#[test]
fn a_boiling_supply_is_refused_by_command_and_by_file() {
    let mut engine = build(DEMO);
    run(&mut engine, 5);
    for why in [
        set_temperature(&mut engine, "rundown_source", 125.0).unwrap_err(),
        set_pressure(&mut engine, "rundown_source", 1.8).unwrap_err(),
    ] {
        assert!(
            why.contains("rundown_source") && why.contains("boiling") && why.contains("B46"),
            "{why}"
        );
    }
    match &node(&engine.snapshot(), "rundown_source").kind {
        NodeKind::Source {
            pressure,
            temperature,
            ..
        } => {
            assert_eq!(
                pressure.value(),
                SOURCE_PA,
                "the refused write moved nothing"
            );
            assert_eq!(temperature.value(), 110.0 + 273.15);
        }
        other => panic!("{other:?}"),
    }
    // Just inside, and legal.
    set_temperature(&mut engine, "rundown_source", 120.0).expect("120 °C is below boiling");
    set_pressure(&mut engine, "rundown_source", 1.9).expect_err("1.9 bar at 120 °C boils");
    run(&mut engine, 5);

    let file = refinery_scenarios::load_str(&declared(DEMO, 2.4, 125.0, 1.5)).unwrap();
    let why = match refinery_scenarios::build_engine(&file) {
        Ok(_) => panic!("a boiling source must be refused at load"),
        Err(e) => e.to_string(),
    };
    assert!(
        why.contains("rundown_source") && why.contains("boiling"),
        "{why}"
    );
}

/// **Nonsense values and the wrong nodes are refused, each with its reason.**
#[test]
fn the_commands_refuse_nonsense_and_the_wrong_nodes() {
    let mut engine = build(DEMO);
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(set_pressure(&mut engine, "rundown_source", bad)
            .unwrap_err()
            .contains("finite and > 0"));
        assert!(set_pressure(&mut engine, "unit_feed", bad)
            .unwrap_err()
            .contains("finite and > 0"));
    }
    for bad in [-273.15, -300.0, f64::NAN] {
        assert!(set_temperature(&mut engine, "rundown_source", bad)
            .unwrap_err()
            .contains("finite and > 0"));
    }
    let why = set_temperature(&mut engine, "unit_feed", 100.0).unwrap_err();
    assert!(
        why.contains("destination") && why.contains("backwards"),
        "{why}"
    );
    let why = set_pressure(&mut engine, "feed_pump", 2.0).unwrap_err();
    assert!(why.contains("not a supply or a destination"), "{why}");

    let mut leaky = build(LEAKY);
    let why = set_pressure(&mut leaky, "supply_tank", 2.0).unwrap_err();
    assert!(why.contains("tank") && why.contains("level"), "{why}");
    let why = set_pressure(&mut leaky, "outside", 2.0).unwrap_err();
    assert!(why.contains("atmosphere"), "{why}");
    let why = set_temperature(&mut leaky, "transfer_pump", 50.0).unwrap_err();
    assert!(why.contains("not a supply"), "{why}");
}

/// **Every supply says whether the plant could check it** (§57 fork 4, the
/// user's DECISION: "accept it, but say so"). The demo measures its naphtha's
/// bubble pressure at the supply; a liquid plant whose thermo cannot tell takes
/// a supply at 500 °C unchecked and says `cannot_tell`; a gas supply says
/// `gas`. Nothing before the first tick, nothing on a node that is not a supply.
#[test]
fn every_supply_says_whether_the_plant_could_check_it() {
    let mut engine = build(DEMO);
    assert!(node(&engine.snapshot(), "rundown_source")
        .supply_boiling
        .is_none());
    run(&mut engine, 5);
    let s = engine.snapshot();
    let SupplyBoiling::Measured { bubble_pressure_pa } = node(&s, "rundown_source")
        .supply_boiling
        .expect("a supply reports")
    else {
        panic!("the demo's thermo measures it")
    };
    // The pump's liquid is the supply's, 0.03 K warmer from friction.
    let at_pump = node(&s, "feed_pump").cavitation.unwrap().bubble_pressure_pa;
    assert!(rel(bubble_pressure_pa, at_pump) < 1e-3);
    assert!(bubble_pressure_pa < SOURCE_PA);
    assert!(node(&s, "feed_pump").supply_boiling.is_none());
    assert!(node(&s, "unit_feed").supply_boiling.is_none());

    let mut furnace = build(FURNACE);
    set_temperature(&mut furnace, "cold_feed", 500.0).expect("no bubble pressure, no refusal");
    run(&mut furnace, 5);
    assert_eq!(
        node(&furnace.snapshot(), "cold_feed").supply_boiling,
        Some(SupplyBoiling::CannotTell)
    );

    let mut gas = build(GAS);
    run(&mut gas, 5);
    assert_eq!(
        node(&gas.snapshot(), "gas_header").supply_boiling,
        Some(SupplyBoiling::Gas)
    );

    let json = serde_json::to_string(&s).unwrap();
    assert!(
        json.contains("\"supply_boiling\":{\"check\":\"measured\",\"bubble_pressure_pa\":"),
        "{json}"
    );
    let json = serde_json::to_string(&furnace.snapshot()).unwrap();
    assert!(json.contains("\"supply_boiling\":{\"check\":\"cannot_tell\"}"));
    let json = serde_json::to_string(&gas.snapshot()).unwrap();
    assert!(json.contains("\"supply_boiling\":{\"check\":\"gas\"}"));
}

/// **The same commands give the same plant, bit for bit** (rule 3).
#[test]
fn the_commands_are_deterministic() {
    let script = |engine: &mut Engine| {
        run(engine, 20);
        set_temperature(engine, "rundown_source", 100.0).unwrap();
        run(engine, 20);
        set_pressure(engine, "rundown_source", 2.8).unwrap();
        set_pressure(engine, "unit_feed", 1.2).unwrap();
        run(engine, 20);
        serde_json::to_string(&engine.snapshot()).unwrap()
    };
    let (mut a, mut b) = (build(DEMO), build(DEMO));
    assert_eq!(script(&mut a), script(&mut b));
}
