//! M50 — what cavitation DOES to a pump (docs/DESIGN.md §55, ledger row B9).
//!
//! A pump that declares `npsh_required_m` delivers `φ·H(Q)`, with `φ` falling
//! from 1 to 0 as its net positive suction head available,
//! `NPSHa = (P_suction − P_bubble)/(ρ·g)`, falls toward zero. These gates hold
//! the shipped demo, `scenarios/pump_cavitation_flow_limit.toml`, to a hand
//! calculation written out here from the published anchors — not read back from
//! the solver — and pin the arms the demo does not reach: a pump with no working
//! point at all, the first tick, the key absent, and the load refusals.
//!
//! `φ`'s anchors, restated rather than imported: 3% of the head lost where
//! `NPSHa = NPSH3`, which is NPSH3's definition (ANSI/HI 9.6.1), and no head at
//! all at the bubble pressure. Between them §55's chosen shape,
//! `φ = 1 − exp(−k·σ²)`, `σ = NPSHa/NPSH3`.

use refinery_core::graph::{NodeId, NodeKind, PumpSuction};
use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot};
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/pump_cavitation_flow_limit.toml");
const M11_DEMO: &str = include_str!("../../../scenarios/cavitating_pump.toml");

const G: f64 = 9.806_65;

// The demo's own numbers, restated so the hand calculation below is computed
// outside the engine.
const SOURCE_PA: f64 = 2.4e5;
const SINK_PA: f64 = 1.5e5;
const FRICTION: f64 = 0.02; // the schema default; no pipe declares one
const SUCTION: (f64, f64) = (60.0, 0.08);
const DISCHARGE: (f64, f64) = (20.0, 0.10);
const FEED: (f64, f64) = (25.0, 0.10);
const KV: f64 = 80.0;
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

/// The bubble pressure [Pa] the tick's solve read: the one on the pump's kind,
/// which the engine set at the top of the tick from the tick before.
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

/// The naphtha's density [kg/m³]: mass-weighted harmonic mean of 680 and 750.
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

/// The demo's steady state by hand: one unknown, the flow, found by bisection
/// on the series loop `source → suction line → pump → discharge → valve → feed
/// line → sink`. Darcy–Weisbach pipes, ISA liquid valve, `H(Q) = h0 − a·Q²`
/// scaled by `φ` at the pump's own suction. `npsh3 = None` is the pump without
/// the key. Returns (mass flow [kg/s], suction [Pa], `φ`).
fn hand_calculation(opening: f64, bubble: f64, npsh3: Option<f64>) -> (f64, f64, f64) {
    let rho = rho();
    let pipe = |(length, diameter): (f64, f64)| {
        let area = std::f64::consts::PI * diameter * diameter / 4.0;
        FRICTION * length * rho / (2.0 * diameter * area * area)
    };
    let cv = KV / (3600.0 * 1e5f64.sqrt()) * opening;
    let valve = (rho / 998.0) / (cv * cv);
    let excess = |q: f64| {
        let suction = SOURCE_PA - pipe(SUCTION) * q * q.abs();
        let share = match npsh3 {
            Some(n) => phi((suction - bubble) / (rho * G * n)),
            None => 1.0,
        };
        let rise = share * rho * G * (H0_M - A * q * q.abs());
        let delivered = suction + rise - (pipe(DISCHARGE) + pipe(FEED) + valve) * q * q.abs();
        (delivered - SINK_PA, suction, share)
    };
    let (mut low, mut high) = (-1.0, 1.0);
    for _ in 0..200 {
        let mid = 0.5 * (low + high);
        if excess(mid).0 > 0.0 {
            low = mid;
        } else {
            high = mid;
        }
    }
    let (_, suction, share) = excess(low);
    (low * rho, suction, share)
}

/// The demo without the key: the same file with `npsh_required_m` removed.
fn without_key(src: &str) -> String {
    src.replace("npsh_required_m = 3.0\n", "")
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

/// **The working point, by hand** (§55 gate 1). At the declared 0.6 opening the
/// pump settles in partial cavitation — interior to the knee, not at either flat
/// end — and the engine's flow, suction margin and head fraction are the hand
/// calculation's. The bubble pressure is the one the solve read, so the gate
/// grades the hydraulics and not the one-tick lag on temperature.
#[test]
fn the_demo_pump_settles_where_the_hand_calculation_puts_it() {
    let mut engine = build(DEMO);
    run(&mut engine, 600);
    let s = engine.snapshot();
    let bubble = solved_against(&s);
    let (mdot, suction, share) = hand_calculation(0.6, bubble, Some(NPSH3_M));
    let pump = node(&s, "feed_pump");
    let reported = pump
        .pump_suction
        .expect("a pump with the key reports its suction");

    assert!(
        (0.1..0.5).contains(&share),
        "the control: the hand calculation puts the pump inside the knee, got φ = {share}"
    );
    assert!(
        rel(flow(&s, "discharge"), mdot) < 1e-4,
        "flow {} against {mdot}",
        flow(&s, "discharge")
    );
    assert!(
        rel(pump.pressure_pa, suction) < 1e-6,
        "suction {} against {suction}",
        pump.pressure_pa
    );
    assert!(
        (reported.head_fraction - share).abs() < 1e-4,
        "head fraction {} against {share}",
        reported.head_fraction
    );
    let npsha = (suction - bubble) / (rho() * G);
    assert!(
        (reported.npsh_available_m - npsha).abs() < 1e-3,
        "NPSHa {} m against {npsha} m",
        reported.npsh_available_m
    );
}

/// **The pump loses flow, and the boiling lamp says "not boiling" while it does**
/// (§55 gate 2). Without the key the same plant carries half as much again with
/// its suction below the bubble point; with it the suction settles just ABOVE —
/// M11's lamp reads false — and the pump delivers about a quarter of its head.
#[test]
fn the_pump_loses_flow_and_the_lamp_disagrees_as_designed() {
    let mut engine = build(DEMO);
    run(&mut engine, 600);
    let with = engine.snapshot();
    let mut twin = build(&without_key(DEMO));
    run(&mut twin, 600);
    let without = twin.snapshot();

    assert!(
        flow(&with, "discharge") < 0.75 * flow(&without, "discharge"),
        "cavitation costs flow: {} against {} without the key",
        flow(&with, "discharge"),
        flow(&without, "discharge")
    );
    let twin_pump = node(&without, "feed_pump");
    assert!(
        twin_pump.cavitation.expect("the twin is graded").cavitating,
        "the control: without the key the suction is below the bubble point"
    );
    assert!(twin_pump.pump_suction.is_none(), "no key, no report");

    let pump = node(&with, "feed_pump");
    assert!(
        !pump.cavitation.expect("the demo is graded").cavitating,
        "the bulk suction stands above the bubble pressure"
    );
    assert!(pump.pump_suction.expect("reported").head_fraction < 0.5);
}

/// **Both fidelities answer the demo alike** (§55 gate 3), at the declared
/// opening and after the operator opens the valve wide.
#[test]
fn both_fidelities_agree_on_the_demo() {
    let mut newton = build(DEMO);
    let mut simple = build(&DEMO.replace("flow = \"newton\"", "flow = \"simple\""));
    for opening in [0.6, 1.0] {
        for engine in [&mut newton, &mut simple] {
            let valve = id(engine, "discharge_valve");
            engine
                .apply(Command::SetValveOpening {
                    node: valve,
                    opening,
                })
                .expect("the valve takes an opening");
            run(engine, 300);
        }
        let (a, b) = (newton.snapshot(), simple.snapshot());
        assert!(
            rel(flow(&b, "discharge"), flow(&a, "discharge")) < 1e-5,
            "opening {opening}: game solver {} against Newton {}",
            flow(&b, "discharge"),
            flow(&a, "discharge")
        );
        let (fa, fb) = (
            node(&a, "feed_pump").pump_suction.unwrap().head_fraction,
            node(&b, "feed_pump").pump_suction.unwrap().head_fraction,
        );
        assert!(
            (fa - fb).abs() < 1e-5,
            "opening {opening}: φ {fb} against {fa}"
        );
    }
}

/// **The flow stops answering the valve, and throttling gives the head back**
/// (§55 gate 4) — the operator's lesson the demo is for. Opening from 0.6 to
/// full buys under 5% more flow where the key-less twin gains over 20%;
/// throttling to 0.2 restores the whole curve.
#[test]
fn opening_up_buys_little_and_throttling_restores_the_head() {
    let at = |src: &str, opening: f64| {
        let mut engine = build(src);
        let valve = id(&engine, "discharge_valve");
        engine
            .apply(Command::SetValveOpening {
                node: valve,
                opening,
            })
            .expect("the valve takes an opening");
        run(&mut engine, 600);
        engine.snapshot()
    };
    let (part, wide) = (at(DEMO, 0.6), at(DEMO, 1.0));
    let gain = flow(&wide, "discharge") / flow(&part, "discharge") - 1.0;
    let twin = without_key(DEMO);
    let twin_gain = flow(&at(&twin, 1.0), "discharge") / flow(&at(&twin, 0.6), "discharge") - 1.0;
    assert!(
        gain < 0.05,
        "opening up buys {:.1}% with the key",
        100.0 * gain
    );
    assert!(
        twin_gain > 0.20,
        "the control: {:.1}% without it",
        100.0 * twin_gain
    );

    let throttled = at(DEMO, 0.2);
    let pump = node(&throttled, "feed_pump");
    assert!(pump.pump_suction.unwrap().head_fraction > 0.999);
    let (mdot, _, _) = hand_calculation(0.2, solved_against(&throttled), Some(NPSH3_M));
    assert!(rel(flow(&throttled, "discharge"), mdot) < 1e-4);
}

/// **A stopped pump has no head to lose** (§55 fork 2): with the key it is the
/// same resistance a stopped pump without the key is, and it reports nothing.
/// Stopped mid-run, so the bubble pressure is in hand.
///
/// Not bit for bit: the 50 ticks before the stop ran different flows, so the
/// two solves start warm from different points and agree to Newton's tolerance
/// (measured 4.7e-9). The edit this defends — a stopped pump's resistance
/// scaled by `φ` — moves the flow by about 2.6e-7 here (`φ ≈ 0.99998` on a
/// pump that is 2.6% of the line's resistance), thirteen times the bar.
#[test]
fn a_stopped_pump_is_the_old_stopped_pump() {
    let stop = |src: &str| {
        let mut engine = build(src);
        run(&mut engine, 50);
        let pump = id(&engine, "feed_pump");
        engine
            .apply(Command::SetPumpOn {
                node: pump,
                on: false,
            })
            .expect("the pump stops");
        run(&mut engine, 50);
        engine.snapshot()
    };
    let (with, without) = (stop(DEMO), stop(&without_key(DEMO)));
    assert!(node(&with, "feed_pump").pump_suction.is_none());
    assert!(
        rel(flow(&with, "discharge"), flow(&without, "discharge")) < 2e-8,
        "{} against {}",
        flow(&with, "discharge"),
        flow(&without, "discharge")
    );
}

/// **The first tick runs the whole curve** (§55 fork 4): no tick has yet
/// resolved the pump's liquid, so there is no bubble pressure to read. Its flow
/// is the key-less twin's, bit for bit, and nothing is reported.
#[test]
fn the_first_tick_runs_the_whole_curve() {
    let mut engine = build(DEMO);
    let mut twin = build(&without_key(DEMO));
    run(&mut engine, 1);
    run(&mut twin, 1);
    let (s, t) = (engine.snapshot(), twin.snapshot());
    assert_eq!(
        flow(&s, "discharge").to_bits(),
        flow(&t, "discharge").to_bits()
    );
    assert!(node(&s, "feed_pump").pump_suction.is_none());
    run(&mut engine, 1);
    assert!(node(&engine.snapshot(), "feed_pump").pump_suction.is_some());
}

/// **A pump with no working point falls to zero head and the line runs
/// backwards** (§55 fork 2, the user's decision: "head to zero, no tricks").
/// M11's pump, 25 m above its supply, has its suction below the bubble point
/// even at zero flow; given the key it is a plain pipe, and the sink, above
/// what the source can hold at that height, drives the line in reverse —
/// what a vapour-locked pump with no check valve does. Asserted on a copy:
/// the shipped file is a regression anchor.
#[test]
fn a_pump_with_no_working_point_falls_to_zero_head_and_backflows() {
    let src = M11_DEMO.replace(
        "a = 800.0\non = true\n",
        "a = 800.0\non = true\nnpsh_required_m = 3.0\n",
    );
    assert_ne!(src, M11_DEMO, "the key went in");
    for solver in ["newton", "simple"] {
        let mut engine = build(&src.replace("flow = \"newton\"", &format!("flow = \"{solver}\"")));
        run(&mut engine, 600);
        let s = engine.snapshot();
        let suction = node(&s, "suction").pump_suction.expect("reported");
        assert_eq!(suction.head_fraction, 0.0, "{solver}: no head at all");
        assert!(
            suction.npsh_available_m < 0.0,
            "{solver}: below the bubble pressure"
        );
        assert!(
            flow(&s, "discharge") < 0.0,
            "{solver}: the line runs backwards"
        );
    }
}

/// **The wire form** (§55 gate 5): the demo's pump publishes `pump_suction`
/// with both numbers, and the M11 demo, without the key, publishes neither key.
#[test]
fn the_wire_carries_the_report_only_where_the_key_is() {
    let mut engine = build(DEMO);
    run(&mut engine, 3);
    let json = serde_json::to_string(&engine.snapshot()).unwrap();
    assert!(
        json.contains("\"pump_suction\":{\"npsh_available_m\":"),
        "{json}"
    );
    assert!(json.contains("\"head_fraction\":"));
    assert!(json.contains("\"suction\":{\"npsh_required\":3.0,\"bubble_pressure\":"));

    let mut old = build(M11_DEMO);
    run(&mut old, 3);
    let json = serde_json::to_string(&old.snapshot()).unwrap();
    assert!(
        !json.contains("pump_suction") && !json.contains("npsh"),
        "{json}"
    );
}

fn refusal(src: &str) -> String {
    let file = refinery_scenarios::load_str(src).expect("the scenario must parse");
    match refinery_scenarios::build_engine(&file) {
        Ok(_) => panic!("the plant must be refused"),
        Err(e) => e.to_string(),
    }
}

/// **Refused where nothing could read it** (§55 fork 5): on a plant whose thermo
/// model has no bubble pressure, the pump would deliver its whole curve for
/// ever — the very disagreement the key exists to end.
#[test]
fn the_key_is_refused_where_thermo_has_no_bubble_pressure() {
    let why = refusal(&DEMO.replace("thermo = \"trouton\"", "thermo = \"constant\""));
    assert!(
        why.contains("feed_pump") && why.contains("bubble pressure"),
        "{why}"
    );
}

/// **Refused in gas service** — a vapour cannot cavitate.
#[test]
fn the_key_is_refused_in_gas_service() {
    let src = r#"
[meta]
name = "gas_pump"
[simulation]
dt = 0.1
[fidelity]
flow = "newton"
thermo = "trouton"
[[components]]
name = "methane"
phase = "gas"
tb_c = -161.5
molar_mass_kg_per_mol = 0.016
cp_j_per_kg_k = 2200.0
[nodes.supply]
type = "source"
pressure_bar = 2.0
temperature_c = 20.0
composition = { methane = 1.0 }
[nodes.blower]
type = "pump"
h0_m = 10.0
a = 100.0
npsh_required_m = 3.0
[nodes.out]
type = "sink"
pressure_bar = 1.5
temperature_c = 20.0
composition = { methane = 1.0 }
[[pipes]]
name = "a"
from = "supply"
to = "blower"
length_m = 10.0
diameter_m = 0.1
[[pipes]]
name = "b"
from = "blower"
to = "out"
length_m = 10.0
diameter_m = 0.1
"#;
    let why = refusal(src);
    assert!(
        why.contains("blower") && why.contains("gas service"),
        "{why}"
    );
}

/// **Refused unless a positive number of metres.**
#[test]
fn the_key_is_refused_unless_positive() {
    for bad in ["0.0", "-1.0"] {
        let why =
            refusal(&DEMO.replace("npsh_required_m = 3.0", &format!("npsh_required_m = {bad}")));
        assert!(
            why.contains("npsh_required_m") && why.contains("positive"),
            "{bad}: {why}"
        );
    }
}
