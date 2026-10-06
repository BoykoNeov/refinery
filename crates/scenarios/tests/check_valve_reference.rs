//! M30: a check (non-return) valve — `docs/DEFERRED.md` row E22, built as
//! `docs/DESIGN.md` §33 specifies.
//!
//! The element under test: a liquid valve whose opening is a memoryless function
//! of the FORWARD DRIVE across its branch — shut at or below zero, so a branch
//! that would run backwards carries exactly nothing, fully open at
//! `full_open_bar`, the relief valve's smoothstep between.
//!
//! Gates, one claim each:
//!
//! 1. **The disc stops the backflow its twin drains through**: on the demo, the
//!    check valve's own pipe never carries a negative flow and carries exactly
//!    zero from the trip until the drive turns forward, on both fidelities,
//!    where the twin with a plain valve in its place runs backwards.
//! 2. **At full lift it IS a plain valve, bit for bit**: until the trip, the demo
//!    and its plain-valve twin publish identical numbers on every tick.
//! 3. **The disc reopens**: once the receiving tank has drained below the
//!    supply's head, flow resumes forward through the stopped pump and the disc,
//!    on both fidelities. Asserted inside gate 1's test, which pins the tick.
//! 4. **The opening's slope is the flow's derivative**: `CompiledEdge::
//!    conductance` matches a centred difference of the recompiled flow inside
//!    the band, and the opening term is exactly zero outside it.
//! 5. **A disc that rides its band in ordinary running runs on both
//!    fidelities**: the game solver needs the opening's share of the slope, or
//!    it diverges.
//! 6. **Every refusal, each on its own message.**
//! 7. **E25, characterised**: restarting the pump into the fill valve the loop
//!    pinned open while the disc was shut is a surge.
//! 8. **A disc publishes a cavitation criterion** under a model that has one.
//! 9. **The pump starts against its shut fill valve** (M45.0, `docs/DESIGN.md`
//!    §50): the stretch between the disc and the shut valve is a dead end, and
//!    the tick solves with nothing flowing, on both fidelities. Until M45 the
//!    first tick after the start was refused as chatter.
//! 10. **A fill valve cracked open behind the disc runs on the game solver**
//!     (M45.1, `docs/DESIGN.md` §50): the pump on, the fill a hair open, as a
//!     level loop leaves it when it opens from shut. Until M45.1 every opening up
//!     to 0.3% diverged on the game solver and 1% took 2 681 sweeps.

use refinery_core::energy::NodeStates;
use refinery_core::graph::{ControlMode, EdgeId, LoopId, NodeId, NodeKind, PlantGraph, TripState};
use refinery_core::snapshot::Command;
use refinery_core::units::Pascal;
use refinery_core::Engine;
use refinery_scenarios::{build_engine, load_str};
use refinery_solvers::network::{compile_edge, CompiledEdge};
use refinery_solvers::NewtonFlowSolver;
use std::collections::BTreeMap;

const DEMO: &str = include_str!("../../../scenarios/tank_level_fill_check_valve.toml");

/// The demo's check valve, as declared.
const CHECK_DECL: &str = "type = \"check_valve\"\nkv = 200.0\nfull_open_bar = 0.015";
/// The same body as a plain valve held fully open: the twin.
const PLAIN_DECL: &str = "type = \"valve\"\nkv = 200.0\nopening = 1.0";

const FIDELITIES: [&str; 2] = ["newton", "simple"];

fn swap(src: &str, from: &str, to: &str) -> String {
    assert!(
        src.contains(from),
        "the fixture's substitution must land: `{from}` is not in the plant"
    );
    src.replacen(from, to, 1)
}

fn on(fidelity: &str, src: &str) -> String {
    swap(src, "flow = \"newton\"", &format!("flow = \"{fidelity}\""))
}

/// The demo with a plain valve, held fully open, where the check valve is.
fn twin(src: &str) -> String {
    swap(src, CHECK_DECL, PLAIN_DECL)
}

/// The demo with its trip removed (the `[[trips]]` table is the file's last).
fn untripped(src: &str) -> String {
    // The table, not the header comment's mention of it.
    let at = src
        .find(
            "
[[trips]]
",
        )
        .expect("the demo declares a trip");
    src[..=at].to_string()
}

fn build(src: &str) -> Engine {
    build_engine(&load_str(src).expect("the plant must parse"))
        .unwrap_or_else(|e| panic!("the plant must build: {e}"))
}

fn refusal(what: &str, src: &str) -> String {
    match load_str(src) {
        Err(e) => e.to_string(),
        Ok(file) => match build_engine(&file) {
            Ok(_) => panic!("{what}: this plant should not have loaded"),
            Err(e) => e.to_string(),
        },
    }
}

fn tick(engine: &mut Engine, label: &str, t: u64) {
    engine
        .tick()
        .unwrap_or_else(|e| panic!("{label}: the plant must run: tick {t}: {e}"));
}

fn edge_flow(engine: &Engine, name: &str) -> f64 {
    engine
        .snapshot()
        .edges
        .iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("the plant declares a pipe '{name}'"))
        .stream
        .mass_flow
        .value()
}

fn tripped_at(engine: &Engine) -> Option<u64> {
    match engine.snapshot().trips[0].state {
        TripState::Armed => None,
        TripState::Tripped { at_tick, .. } => Some(at_tick),
    }
}

fn level(engine: &Engine) -> f64 {
    match engine.snapshot().controls[0]
        .measurement
        .expect("a tank's level is measured from load")
    {
        refinery_core::graph::ControlledValue::Level { m } => m.value(),
        other => panic!("the demo's loop holds a level, read {other:?}"),
    }
}

// ------------------------------------------------------------------ gate 1

/// What one run of the demo (or its twin) measured over 6 000 ticks.
struct Run {
    trip: Option<u64>,
    /// Most negative flow through `fill_line`.
    most_backward_fill: f64,
    /// Ticks the check valve's own pipe carried exactly zero.
    shut_ticks: u64,
    /// Most negative flow through the check valve's own pipe.
    most_backward_disc: f64,
    /// First tick after the trip with forward flow through the disc.
    reopened: Option<u64>,
    final_disc_flow: f64,
    final_level: f64,
    /// Lowest level after the trip, and the tick it was read.
    lowest_level: (f64, u64),
    level_at_3000: f64,
}

fn run(label: &str, src: &str, has_disc: bool) -> Run {
    let mut engine = build(src);
    let mut out = Run {
        trip: None,
        most_backward_fill: 0.0,
        shut_ticks: 0,
        most_backward_disc: 0.0,
        reopened: None,
        final_disc_flow: 0.0,
        final_level: 0.0,
        lowest_level: (f64::INFINITY, 0),
        level_at_3000: 0.0,
    };
    for t in 1..=6_000 {
        tick(&mut engine, label, t);
        out.trip = tripped_at(&engine);
        out.most_backward_fill = out.most_backward_fill.min(edge_flow(&engine, "fill_line"));
        let disc = edge_flow(&engine, "check_outlet");
        out.most_backward_disc = out.most_backward_disc.min(disc);
        if has_disc && disc == 0.0 {
            out.shut_ticks += 1;
        }
        if out.trip.is_some() && out.reopened.is_none() && out.shut_ticks > 0 && disc > 0.0 {
            out.reopened = Some(t);
        }
        out.final_disc_flow = disc;
        if out.trip.is_some() && level(&engine) < out.lowest_level.0 {
            out.lowest_level = (level(&engine), t);
        }
        if t == 3_000 {
            out.level_at_3000 = level(&engine);
        }
    }
    out.final_level = level(&engine);
    eprintln!(
        "MEASURE {label}: trip {:?}, most backward fill {:e}, shut ticks {}, most backward disc \
         {:e}, reopened {:?}, final disc flow {}, final level {}, lowest {:?}",
        out.trip,
        out.most_backward_fill,
        out.shut_ticks,
        out.most_backward_disc,
        out.reopened,
        out.final_disc_flow,
        out.final_level,
        out.lowest_level
    );
    out
}

#[test]
fn the_disc_stops_the_backflow_its_twin_drains_through() {
    for fidelity in FIDELITIES {
        let demo = run(&format!("{fidelity}/demo"), &on(fidelity, DEMO), true);
        let plain = run(
            &format!("{fidelity}/twin"),
            &on(fidelity, &twin(DEMO)),
            false,
        );

        // The control first: without the disc, the stopped pump passes the
        // receiving tank's head backwards (E22).
        assert_eq!(
            plain.trip, demo.trip,
            "{fidelity}: both trip on the same tick"
        );
        assert!(
            plain.most_backward_disc < -3.0,
            "{fidelity}: the twin runs backwards through the plain valve, most backward {} kg/s",
            plain.most_backward_disc
        );

        // `>= 0.0` admits the IEEE negative zero a shut branch reports under a
        // reverse drive (`flow` is `−x/∞`), which IS zero.
        assert!(
            demo.most_backward_disc >= 0.0,
            "{fidelity}: the disc never passes a backward flow, most backward {} kg/s",
            demo.most_backward_disc
        );
        let trip = demo.trip.expect("the demo's pump trips");
        let reopened = demo.reopened.expect("the disc reopens");
        assert_eq!(
            demo.shut_ticks,
            reopened - trip,
            "{fidelity}: shut on exactly every tick from the trip to the reopening"
        );
        assert_eq!(
            (trip, reopened),
            (2_142, 3_160),
            "{fidelity}: the measured stretch"
        );

        // What it buys, and what it does not. While the disc is shut the tank
        // falls only through its own drain, so it stands higher than the twin's
        // (2.27 m against 1.87 m at tick 3 000); it is NOT kept, because the drain
        // still runs, and by tick 6 000 the twin stands a little higher (0.985 m
        // against 0.947 m), the water it pushed back into the supply having come
        // forward again.
        assert!(
            demo.level_at_3000 > plain.level_at_3000 + 0.3,
            "{fidelity}: the disc holds the tank up while it is shut: {} m against {} m",
            demo.level_at_3000,
            plain.level_at_3000
        );
        assert!(
            demo.final_level < 1.0 && plain.final_level < 1.0,
            "{fidelity}: and neither tank is kept: {} m and {} m against a 4 m setpoint",
            demo.final_level,
            plain.final_level
        );
    }
}

// ------------------------------------------------------------------ gate 2

/// Every published number that a plain valve and a check valve at full lift
/// share, as bits: node pressures and temperatures, edge streams, the loop.
fn published(engine: &Engine) -> Vec<u64> {
    let snap = engine.snapshot();
    let mut bits = Vec::new();
    for n in &snap.nodes {
        bits.push(n.pressure_pa.to_bits());
        bits.push(n.temperature_k.to_bits());
    }
    for e in &snap.edges {
        bits.push(e.stream.mass_flow.value().to_bits());
        bits.push(e.stream.temperature.value().to_bits());
    }
    for c in &snap.controls {
        bits.push(c.output.to_bits());
    }
    bits
}

#[test]
fn at_full_lift_it_is_a_plain_valve() {
    for fidelity in FIDELITIES {
        let mut demo = build(&on(fidelity, DEMO));
        let mut plain = build(&on(fidelity, &twin(DEMO)));
        let (mut before, mut after) = (0.0_f64, 0.0_f64);
        let mut trip = None;
        for t in 1..=3_000 {
            tick(&mut demo, "demo", t);
            tick(&mut plain, "twin", t);
            let gap = published(&demo)
                .iter()
                .zip(published(&plain))
                .map(|(a, b)| {
                    let (a, b) = (f64::from_bits(*a), f64::from_bits(b));
                    (a - b).abs() / a.abs().max(b.abs()).max(1.0)
                })
                .fold(0.0_f64, f64::max);
            trip = trip.or(tripped_at(&demo));
            if trip.is_none() {
                before = before.max(gap);
            } else {
                after = after.max(gap);
            }
        }
        eprintln!("MEASURE {fidelity}: before the trip {before:e}, after {after:e}, trip {trip:?}");
        // Not bit for bit: tick 1 starts cold, and its iterates cross the disc's
        // band on the way to full lift, so the two solves take different paths
        // to one root and stop inside their own tolerance of it (measured
        // 2.4e-9 on Newton, 1.3e-6 on the game solver). The bound is ten times
        // each fidelity's own `tol_rel`, the solver's promise, not a fit.
        let bound = if fidelity == "newton" { 1e-7 } else { 1e-5 };
        assert!(
            trip.is_some(),
            "{fidelity}: the demo trips inside 3 000 ticks"
        );
        assert!(
            before < bound,
            "{fidelity}: at full lift the disc is a plain valve, worst relative gap {before:e}"
        );
        assert!(
            after > 0.5,
            "{fidelity}: and once the pump stops it is not, worst relative gap {after:e}"
        );
    }
}

// ------------------------------------------------------------------ gate 4

/// A check valve between a pinned header and a sink, water, so the valve
/// composes with its pipe in closed form and the slope is exact.
const DISC: &str = r#"
[meta]
name = "check_slope"
[simulation]
dt = 1.0
[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"

[nodes.header]
type = "source"
pressure_bar = 2.0
temperature_c = 20.0

[nodes.disc]
type = "check_valve"
kv = 60.0
full_open_bar = 0.3

[nodes.drain]
type = "sink"
pressure_bar = 1.0
temperature_c = 20.0

[[pipes]]
name = "riser"
from = "header"
to = "disc"
length_m = 5.0
diameter_m = 0.10

# UPHILL, so the branch carries a static head: "shut at zero drive" must mean
# zero drive NET of it, or a disc between `0` and `β` of raw drop would open onto
# a branch running backwards.
[[pipes]]
name = "tail"
from = "disc"
to = "drain"
length_m = 5.0
diameter_m = 0.10
elevation_change_m = 3.0
"#;

const FULL_OPEN_PA: f64 = 0.3e5;
const DRAIN_PA: f64 = 1.0e5;

fn node(graph: &PlantGraph, name: &str) -> NodeId {
    graph
        .find_node(name)
        .unwrap_or_else(|| panic!("plant must declare a '{name}' node"))
}

fn pipe(graph: &PlantGraph, name: &str) -> EdgeId {
    graph
        .edge_ids()
        .find(|e| graph.pipe(*e).name == name)
        .unwrap_or_else(|| panic!("plant must declare a '{name}' pipe"))
}

/// The tail's static head `β` [Pa]: what the disc's drive is net of.
fn tail_head(engine: &Engine) -> f64 {
    let (c, _) = tail_with_disc_at(engine, DRAIN_PA + 2.0e5);
    c.branch.beta
}

fn tail_with_disc_at(engine: &Engine, p_disc: f64) -> (CompiledEdge, f64) {
    let graph = &engine.graph;
    let mut p: BTreeMap<NodeId, f64> = graph.node_ids().map(|n| (n, DRAIN_PA)).collect();
    p.insert(node(graph, "header"), 3.0e5);
    p.insert(node(graph, "disc"), p_disc);
    let eid = pipe(graph, "tail");
    let c = compile_edge(graph, eid, &engine.slate, &NodeStates::default(), &p)
        .expect("the tail compiles");
    (c, p_disc - DRAIN_PA)
}

/// The tail compiled with the disc at forward `drive` NET of the static head;
/// returns the edge and its raw drop `dp`.
fn tail_at(engine: &Engine, drive: f64) -> (CompiledEdge, f64) {
    tail_with_disc_at(engine, DRAIN_PA + tail_head(engine) + drive)
}

fn mass_flow(engine: &Engine, drive: f64) -> f64 {
    let eps = NewtonFlowSolver::default().eps_dp;
    let (c, dp) = tail_at(engine, drive);
    c.rho * c.branch.flow(dp, eps)
}

#[test]
fn the_opening_slope_is_the_flow_derivative_and_vanishes_outside_the_band() {
    let engine = build(DISC);
    let eps = NewtonFlowSolver::default().eps_dp;

    // Inside the band: the conductance against a centred difference of the
    // FULLY recompiled flow, at three depths into the band.
    for fraction in [0.2, 0.5, 0.8] {
        let drive = fraction * FULL_OPEN_PA;
        let (c, dp) = tail_at(&engine, drive);
        assert!(
            c.check_opening_log_slope > 0.0,
            "inside the band the opening term is live, read {}",
            c.check_opening_log_slope
        );
        let h = 1e-3 * FULL_OPEN_PA;
        let centred = (mass_flow(&engine, drive + h) - mass_flow(&engine, drive - h)) / (2.0 * h);
        let analytic = c.conductance(dp, eps);
        let frozen = c.rho * c.branch.flow_ddp(dp, eps);
        let relative = (analytic - centred).abs() / centred.abs();
        eprintln!(
            "MEASURE band {fraction}: analytic {analytic:e} centred {centred:e} frozen {frozen:e} \
             relative {relative:e}"
        );
        assert!(
            relative < 1e-5,
            "at {fraction} of the band: conductance {analytic:e} against a centred \
             difference {centred:e} (frozen alone {frozen:e})"
        );
        assert!(
            analytic > 1.5 * frozen,
            "the opening's share is not a rounding term: {analytic:e} against frozen {frozen:e}"
        );
    }

    // Above the band: full lift, the term exactly zero, and the branch exactly
    // a plain valve's at opening 1.
    let (above, dp) = tail_at(&engine, 2.0 * FULL_OPEN_PA);
    assert_eq!(
        above.check_opening_log_slope, 0.0,
        "full lift has no opening slope"
    );
    assert_eq!(
        above.conductance(dp, eps).to_bits(),
        (above.rho * above.branch.flow_ddp(dp, eps)).to_bits()
    );
    let plain = build(&swap(
        DISC,
        "type = \"check_valve\"\nkv = 60.0\nfull_open_bar = 0.3",
        "type = \"valve\"\nkv = 60.0\nopening = 1.0",
    ));
    let (plain_tail, _) = tail_at(&plain, 2.0 * FULL_OPEN_PA);
    assert_eq!(
        above.branch.alpha.to_bits(),
        plain_tail.branch.alpha.to_bits(),
        "at full lift the disc's branch is a plain valve's at opening 1"
    );

    // Backwards: shut, conducting nothing, carrying exactly nothing — including
    // at a RAW drop that is forward but short of the static head.
    let head = tail_head(&engine);
    assert!(head > 2.0e4, "the fixture's tail is uphill, β = {head} Pa");
    for drive in [-0.5 * FULL_OPEN_PA, -0.5 * head, 0.0] {
        let (shut, dp) = tail_at(&engine, drive);
        assert!(!shut.conducts, "a disc with no forward drive is shut");
        assert_eq!(shut.check_opening_log_slope, 0.0);
        assert_eq!(
            shut.rho * shut.branch.flow(dp, eps),
            0.0,
            "and passes nothing"
        );
    }

    // The gas door at `compile_edge`: a graph built past the loader. Since M31 a
    // disc in gas service compiles when it carries `x_t` (docs/DESIGN.md §34,
    // `gas_check_valve_reference.rs`); without one it is the incompressible law
    // on a compressible fluid, refused as every other valve kind refuses it.
    let gas = include_str!("../../../scenarios/gas_valve.toml");
    let mut gas_plant = build(gas);
    let valve = node(&gas_plant.graph, "control_valve");
    gas_plant.graph.node_mut(valve).kind = NodeKind::CheckValve {
        cv_max: 1e-3,
        full_open: Pascal(1e4),
        x_t: None,
    };
    let p: BTreeMap<NodeId, f64> = gas_plant.graph.node_ids().map(|n| (n, 5e5)).collect();
    let eid = pipe(&gas_plant.graph, "outlet_run");
    let err = compile_edge(
        &gas_plant.graph,
        eid,
        &gas_plant.slate,
        &NodeStates::default(),
        &p,
    )
    .err()
    .expect("a check valve in gas service with no x_T does not compile");
    assert!(
        err.to_string()
            .contains("carries a gas-phase stream but has no x_T"),
        "refused for its own reason: {err}"
    );
}

// ------------------------------------------------------------------ gate 5

/// The demo with no trip and a disc whose band is wider than its running
/// drive, so it rides the band in ordinary operation.
fn in_band() -> String {
    swap(
        &untripped(DEMO),
        "full_open_bar = 0.015",
        "full_open_bar = 0.1",
    )
}

#[test]
fn a_disc_riding_its_band_runs_on_both_fidelities() {
    for fidelity in FIDELITIES {
        let mut engine = build(&on(fidelity, &in_band()));
        let (mut worst, mut total) = (0u32, 0u64);
        let (mut least_drive, mut most_drive) = (f64::INFINITY, f64::NEG_INFINITY);
        let graph_disc = engine.graph.find_node("discharge_check").expect("declared");
        let graph_fill = engine.graph.find_node("discharge_valve").expect("declared");
        for t in 1..=3_000 {
            tick(&mut engine, fidelity, t);
            let d = engine
                .last_solution()
                .expect("a tick leaves a solution")
                .diagnostics
                .iterations;
            worst = worst.max(d);
            total += u64::from(d);
            let snap = engine.snapshot();
            let p = |id: NodeId| snap.nodes[id.0 as usize].pressure_pa;
            let drive = p(graph_disc) - p(graph_fill);
            least_drive = least_drive.min(drive);
            most_drive = most_drive.max(drive);
        }
        eprintln!(
            "MEASURE {fidelity}: worst {worst} total {total} drive [{least_drive}, {most_drive}] Pa"
        );
        assert!(
            0.0 < least_drive && most_drive < 0.1e5,
            "{fidelity}: the disc rides its band for the whole run, drive [{least_drive}, \
             {most_drive}] Pa against full lift at 10 000 Pa"
        );
    }
}

// ------------------------------------------------------------------ gate 6

#[test]
fn every_refusal_names_its_own_reason() {
    let cases: Vec<(&str, String, &str)> = vec![
        (
            "a zero band",
            swap(DEMO, "full_open_bar = 0.015", "full_open_bar = 0.0"),
            "which must be > 0. A zero band",
        ),
        (
            "a negative band",
            swap(DEMO, "full_open_bar = 0.015", "full_open_bar = -0.1"),
            "which must be > 0. A zero band",
        ),
        (
            "no band",
            swap(DEMO, "\nfull_open_bar = 0.015", ""),
            "full_open_bar",
        ),
        (
            "no flow coefficient",
            swap(
                DEMO,
                CHECK_DECL,
                "type = \"check_valve\"\nkv = 0.0\nfull_open_bar = 0.015",
            ),
            "passes nothing in either direction",
        ),
        (
            "a loop actuating the disc",
            swap(
                DEMO,
                "actuator = \"discharge_valve\"",
                "actuator = \"discharge_check\"",
            ),
            "a check valve. Its disc is moved",
        ),
        (
            "a trip acting on the disc",
            swap(
                DEMO,
                "  { pump = \"transfer_pump\" },",
                "  { valve = \"discharge_check\", position = 0.0 },",
            ),
            "a check valve. Its disc is moved by the",
        ),
        (
            "a disc with two outlets",
            swap(
                DEMO,
                "[[pipes]]\nname = \"fill_line\"",
                "[[pipes]]\nname = \"second_outlet\"\nfrom = \"discharge_check\"\nto = \
                 \"receiving_tank\"\nlength_m = 1.0\ndiameter_m = 0.10\n\n[[pipes]]\nname = \
                 \"fill_line\"",
            ),
            "must have exactly 1 inlet + 1 outlet",
        ),
        (
            "a disc in gas service with no x_t",
            swap(
                include_str!("../../../scenarios/gas_valve.toml"),
                "type = \"valve\"\nkv = 25.0\nopening = 1.0\nx_t = 0.72",
                "type = \"check_valve\"\nkv = 25.0\nfull_open_bar = 0.1",
            ),
            "is in gas service and must declare `x_t`",
        ),
    ];
    for (what, src, needle) in cases {
        let message = refusal(what, &src);
        assert!(
            message.contains(needle),
            "{what}: refused for its own reason, expected `{needle}` in: {message}"
        );
    }

    // And by command: a disc has no opening to set.
    let mut engine = build(DEMO);
    let disc = engine.graph.find_node("discharge_check").expect("declared");
    let err = engine
        .apply(Command::SetValveOpening {
            node: disc,
            opening: 0.5,
        })
        .expect_err("a check valve's opening is not a command");
    assert!(
        err.to_string().contains("is a check valve"),
        "refused for its own reason: {err}"
    );
}

// ------------------------------------------------------------------ gate 7

/// E25, characterised rather than fixed. While the disc is shut the level loop
/// pins the fill valve open (the right sign for a falling level, with nothing
/// coming through), so restarting the pump drives the full pump curve through
/// a wide-open fill.
#[test]
fn restarting_into_the_pinned_fill_is_a_surge() {
    let mut engine = build(&untripped(DEMO));
    let pump = engine.graph.find_node("transfer_pump").expect("declared");
    for t in 1..=3_000 {
        tick(&mut engine, "running", t);
    }
    let settled = edge_flow(&engine, "fill_line");
    engine
        .apply(Command::SetPumpOn {
            node: pump,
            on: false,
        })
        .expect("a pump can be stopped");
    for t in 3_001..=6_000 {
        tick(&mut engine, "stopped", t);
    }
    let pinned = engine.snapshot().controls[0].output;
    engine
        .apply(Command::SetPumpOn {
            node: pump,
            on: true,
        })
        .expect("a pump can be restarted");
    let mut peak = 0.0_f64;
    for t in 6_001..=6_100 {
        tick(&mut engine, "restarted", t);
        peak = peak.max(edge_flow(&engine, "fill_line"));
    }
    eprintln!("MEASURE surge: settled {settled}, pinned {pinned}, peak {peak}");
    assert_eq!(
        pinned, 1.0,
        "the loop pinned the fill open while the disc was shut"
    );
    assert!(
        peak > 3.0 * settled,
        "the restart surges: peak {peak} kg/s against {settled} kg/s settled"
    );
}

// ------------------------------------------------------------------ gate 8

/// **A disc is a cavitation subject** (`cavitation_subject`, DESIGN §33 fork 5):
/// its node is its inlet, upstream of the disc, so what flashes there is what
/// the line delivers. Every other gate here runs `thermo = "constant"`, which
/// has no criterion at all, so excluding the disc would pass all of them; this
/// gate selects the model that can answer, with the plain fill valve beside it
/// as the control that the switch took effect.
#[test]
fn a_disc_publishes_a_cavitation_criterion() {
    let mut engine = build(&swap(DEMO, "thermo = \"constant\"", "thermo = \"trouton\""));
    for t in 1..=5 {
        tick(&mut engine, "trouton", t);
    }
    let snap = engine.snapshot();
    let criterion = |name: &str| {
        snap.nodes
            .iter()
            .find(|n| n.name == name)
            .unwrap_or_else(|| panic!("the plant declares '{name}'"))
            .cavitation
            .as_ref()
            .map(|c| c.cavitating)
    };
    assert_eq!(
        criterion("discharge_valve"),
        Some(false),
        "the control: a plain valve publishes a criterion under this model"
    );
    assert_eq!(
        criterion("discharge_check"),
        Some(false),
        "and so does the disc"
    );
}

// ------------------------------------------------------------------ gate 9

fn node_pressure(engine: &Engine, name: &str) -> f64 {
    engine
        .snapshot()
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("the plant declares '{name}'"))
        .pressure_pa
}

/// **A pump started against its shut discharge valve runs** (M45.0, DESIGN
/// §50). The stop parks the loop in MANUAL with the fill valve shut, as a
/// person — or a trip — would; the start then dead-heads the pump with the disc
/// between it and the valve. Filled, that stretch stands at the pump's pressure
/// and the disc's drive is zero, so it shuts; floating, it is parked below, so
/// the disc opens — the active-set loop read the alternation as chatter and
/// refused the tick on both fidelities. It is a dead end: nothing flows, and
/// the stretch reads the disc's inlet pressure (its 1 m outlet is level).
#[test]
fn the_pump_starts_against_its_shut_fill_valve() {
    for fidelity in FIDELITIES {
        let mut engine = build(&on(fidelity, &untripped(DEMO)));
        let pump = engine.graph.find_node("transfer_pump").expect("declared");
        let fill = engine.graph.find_node("discharge_valve").expect("declared");
        for t in 1..=3_000 {
            tick(&mut engine, fidelity, t);
        }
        engine
            .apply(Command::SetPumpOn {
                node: pump,
                on: false,
            })
            .expect("a pump can be stopped");
        engine
            .apply(Command::SetControllerMode {
                loop_id: LoopId(0),
                mode: ControlMode::Manual,
            })
            .expect("the loop can be put in MANUAL");
        engine
            .apply(Command::SetValveOpening {
                node: fill,
                opening: 0.0,
            })
            .expect("a valve in MANUAL can be shut");
        for t in 3_001..=3_100 {
            tick(&mut engine, fidelity, t);
        }
        let stopped = node_pressure(&engine, "discharge_check");
        engine
            .apply(Command::SetPumpOn {
                node: pump,
                on: true,
            })
            .expect("a pump can be started");
        let mut worst_iterations = 0;
        for t in 3_101..=3_400 {
            tick(&mut engine, &format!("{fidelity}, dead-headed"), t);
            worst_iterations = worst_iterations.max(engine.snapshot().solver.iterations);
            assert_eq!(edge_flow(&engine, "fill_line"), 0.0, "{fidelity} tick {t}");
            assert_eq!(
                edge_flow(&engine, "check_outlet"),
                0.0,
                "{fidelity} tick {t}"
            );
            let at_disc = node_pressure(&engine, "discharge_check");
            let stretch = node_pressure(&engine, "discharge_valve");
            assert!(
                (stretch - at_disc).abs() < 1.0,
                "{fidelity} tick {t}: the stretch stands at the disc's inlet, {stretch} Pa against {at_disc} Pa"
            );
            assert!(
                at_disc > stopped + 2.0e5,
                "{fidelity} tick {t}: the started pump puts its head on the line, {at_disc} Pa against {stopped} Pa stopped"
            );
        }
        eprintln!(
            "MEASURE dead-headed {fidelity}: stopped {stopped} Pa, running {} Pa, worst iterations a tick {worst_iterations}",
            node_pressure(&engine, "discharge_check")
        );
    }
}

// ------------------------------------------------------------------ gate 10

/// **A fill valve cracked open behind the disc runs on both fidelities, and
/// they agree** (M45.1, DESIGN §50). The fill is shut by hand while the pump is
/// stopped, then the pump starts and the fill is cracked open to each opening;
/// ten ticks run. The game solver's node step on the fill's node used to be
/// hundreds of times too long while the disc was shut, and every step was
/// refused; it now solves the node's own equation on the bracket.
#[test]
fn a_fill_cracked_open_behind_the_disc_runs_on_both_fidelities() {
    for opening in [1e-5, 1.5e-4, 1e-3, 1e-2, 3e-2] {
        let mut flows = Vec::new();
        for fidelity in FIDELITIES {
            let mut engine = build(&on(fidelity, &untripped(DEMO)));
            let pump = engine.graph.find_node("transfer_pump").expect("declared");
            let fill = engine.graph.find_node("discharge_valve").expect("declared");
            for t in 1..=3_000 {
                tick(&mut engine, fidelity, t);
            }
            for cmd in [
                Command::SetPumpOn {
                    node: pump,
                    on: false,
                },
                Command::SetControllerMode {
                    loop_id: LoopId(0),
                    mode: ControlMode::Manual,
                },
                Command::SetValveOpening {
                    node: fill,
                    opening: 0.0,
                },
            ] {
                engine.apply(cmd).expect("the stop is accepted");
            }
            for t in 3_001..=3_100 {
                tick(&mut engine, fidelity, t);
            }
            for cmd in [
                Command::SetPumpOn {
                    node: pump,
                    on: true,
                },
                Command::SetValveOpening {
                    node: fill,
                    opening,
                },
            ] {
                engine.apply(cmd).expect("the start is accepted");
            }
            let mut worst = 0;
            for t in 3_101..=3_110 {
                tick(&mut engine, &format!("{fidelity}, fill at {opening}"), t);
                worst = worst.max(engine.snapshot().solver.iterations);
            }
            eprintln!("MEASURE cracked {opening} {fidelity}: worst iterations {worst}");
            assert!(worst < 50, "{fidelity}, fill at {opening}: {worst} a tick");
            flows.push(edge_flow(&engine, "fill_line"));
        }
        let (newton, game) = (flows[0], flows[1]);
        assert!(
            (newton - game).abs() <= 1e-5 * newton.abs().max(1e-3),
            "fill at {opening}: Newton {newton} kg/s against the game solver's {game}"
        );
    }
}
