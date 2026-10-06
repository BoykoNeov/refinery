//! A pop relief valve: blowdown hysteresis (M48.0, docs/DESIGN.md §53, ledger
//! row B6), on `relief_pop_cycle.toml` and inline fixtures.
//!
//! The demo is `relief_blowdown.toml` with one key added, `blowdown_bar = 1.4`.
//! The hand numbers are the spec restated: set 20 bar, reseat 20 − 1.4 =
//! 18.6 bar (API 520 Part I's blowdown: the drop below set at which a lifted
//! valve reseats). The latch moves at the TOP of a tick from the pressure the
//! valve's spring sensed in the LAST solve, so every rule below reads tick
//! `t − 1`'s solved pressure at the valve and tick `t`'s latch.

use refinery_core::graph::NodeKind;
use refinery_scenarios::NodeDef;

const POP: &str = include_str!("../../../scenarios/relief_pop_cycle.toml");
const MEMORYLESS: &str = include_str!("../../../scenarios/relief_blowdown.toml");

const SET_BAR: f64 = 20.0;
const RESEAT_BAR: f64 = 20.0 - 1.4;

fn load(text: &str) -> refinery_scenarios::ScenarioFile {
    refinery_scenarios::load_str(text).expect("parses")
}

/// One tick of the demo, as its snapshot would show it.
struct Tick {
    /// The latch this tick's flows were computed with.
    lifted: bool,
    /// The solved pressure [bar] at the valve's own node: what its spring sensed.
    psv_bar: f64,
    receiver_bar: f64,
    /// Through the valve's outlet line [kg/s].
    flare: f64,
    /// Every node pressure [Pa] and edge flow [kg/s], for bit comparisons.
    pressures: Vec<f64>,
    flows: Vec<f64>,
}

fn run(file: &refinery_scenarios::ScenarioFile, ticks: usize) -> Vec<Tick> {
    let mut engine = refinery_scenarios::build_engine(file).expect("builds");
    let psv = engine.graph.find_node("psv").expect("psv");
    let receiver = engine.graph.find_node("receiver").expect("receiver");
    let flare = engine
        .graph
        .edge_ids()
        .find(|e| engine.graph.pipe(*e).name == "flare_line")
        .expect("flare_line");
    let mut out = Vec::with_capacity(ticks);
    for i in 0..ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("tick {}: {e}", i + 1));
        let solution = engine.last_solution().expect("solved");
        let lifted = match &engine.graph.node(psv).kind {
            NodeKind::ReliefValve { blowdown, .. } => blowdown.is_some_and(|b| b.lifted),
            other => panic!("psv must be a relief valve, got {other:?}"),
        };
        out.push(Tick {
            lifted,
            psv_bar: solution.node_pressure[&psv].value() / 1e5,
            receiver_bar: solution.node_pressure[&receiver].value() / 1e5,
            flare: engine.graph.pipe(flare).stream.mass_flow.value(),
            pressures: solution.node_pressure.values().map(|p| p.value()).collect(),
            flows: solution.edge_mass_flow.values().copied().collect(),
        });
    }
    out
}

fn set_blowdown(file: &mut refinery_scenarios::ScenarioFile, value: Option<f64>) {
    match file.nodes.get_mut("psv") {
        Some(NodeDef::ReliefValve { blowdown_bar, .. }) => *blowdown_bar = value,
        other => panic!("expected a relief_valve, got {other:?}"),
    }
}

/// Ticks (1-based) whose latch differs from the tick before's, and the new value.
fn flips(run: &[Tick]) -> Vec<(usize, bool)> {
    run.windows(2)
        .enumerate()
        .filter(|(_, w)| w[0].lifted != w[1].lifted)
        .map(|(i, w)| (i + 2, w[1].lifted))
        .collect()
}

/// Gate 1 (fork 1): the shut curve IS the M5 curve. Until the valve first lifts,
/// the demo is bit-identical to its memoryless twin on every node pressure and
/// every edge flow — and afterwards it is not, so the comparison is not vacuous.
#[test]
fn a_pop_valve_runs_the_memoryless_valves_bits_until_it_first_lifts() {
    let pop = run(&load(POP), 400);
    let memoryless = run(&load(MEMORYLESS), 400);
    let first_lift = pop.iter().position(|t| t.lifted).expect("the valve lifts");
    assert_eq!(first_lift + 1, 127, "pinned: the first lifted tick");
    for (i, (a, b)) in pop.iter().zip(&memoryless).take(first_lift).enumerate() {
        assert_eq!(a.pressures, b.pressures, "tick {}: pressures", i + 1);
        assert_eq!(a.flows, b.flows, "tick {}: flows", i + 1);
    }
    assert_ne!(
        pop[first_lift].flows, memoryless[first_lift].flows,
        "the first lifted tick must differ from the memoryless valve"
    );
}

/// Gate 2 (forks 2 and 3): the first tick whose start-of-tick inlet stands above
/// set runs lifted, and lifted is NOT the M5 curve: it relieves at an inlet
/// below set, where the M5 curve is exactly shut.
#[test]
fn the_tick_after_the_inlet_stands_above_set_runs_at_full_lift() {
    let ticks = run(&load(POP), 400);
    let above = ticks
        .iter()
        .position(|t| t.psv_bar > SET_BAR)
        .expect("the inlet reaches set");
    assert!(
        !ticks[above].lifted,
        "the latch does not move inside a solve"
    );
    let next = &ticks[above + 1];
    assert!(next.lifted, "the tick after runs lifted");
    assert!(
        next.psv_bar < SET_BAR,
        "full lift draws the inlet below set at once ({} bar)",
        next.psv_bar
    );
    assert!(
        next.flare > 0.1,
        "lifted below set must relieve, the M5 curve would pass nothing: {} kg/s",
        next.flare
    );
}

/// Gate 3 (fork 3), against the hand numbers: the latch obeys set and reseat
/// exactly, and between them the same inlet pressure finds the valve shut on
/// the way up and open on the way down.
#[test]
fn at_one_pressure_it_is_shut_on_the_way_up_and_open_on_the_way_down() {
    let ticks = run(&load(POP), 2000);
    for (i, w) in ticks.windows(2).enumerate() {
        let (sensed, tick) = (w[0].psv_bar, i + 2);
        let expected = if w[0].lifted {
            sensed >= RESEAT_BAR
        } else {
            sensed > SET_BAR
        };
        assert_eq!(
            w[1].lifted, expected,
            "tick {tick}: latch {} after an inlet of {sensed} bar (was {})",
            w[1].lifted, w[0].lifted
        );
    }
    // The same pressure, read in the tick's own solve: shut, the M5 curve
    // passes exactly nothing below set; lifted, the valve relieves.
    let mut up = 0;
    let mut down = 0;
    for t in &ticks {
        if !(RESEAT_BAR + 0.05..SET_BAR - 0.05).contains(&t.psv_bar) {
            continue;
        }
        if t.lifted {
            down += 1;
            assert!(t.flare > 0.1, "open on the way down: {} kg/s", t.flare);
        } else {
            up += 1;
            assert_eq!(t.flare, 0.0, "shut on the way up, at {} bar", t.psv_bar);
        }
    }
    assert!(
        up > 100 && down > 100,
        "both ways measured: {up} up, {down} down"
    );
}

/// Gate 4: the receiver saw-tooths between reseat and set, on both fidelities,
/// with the same lift and reseat ticks.
#[test]
fn the_receiver_saw_tooths_on_both_fidelities() {
    let mut runs = Vec::new();
    for flow in ["newton", "simple"] {
        let mut file = load(POP);
        file.fidelity.flow = flow.to_string();
        let ticks = run(&file, 6000);
        let lifts = flips(&ticks).iter().filter(|(_, lifted)| *lifted).count();
        assert!(lifts >= 90, "{flow}: {lifts} lifts in 6000 ticks");
        let first = ticks.iter().position(|t| t.lifted).expect("lifts");
        for (i, t) in ticks.iter().enumerate().skip(first) {
            assert!(
                (RESEAT_BAR - 0.1..SET_BAR + 0.1).contains(&t.receiver_bar),
                "{flow}: tick {} receiver at {} bar",
                i + 1,
                t.receiver_bar
            );
        }
        runs.push(flips(&ticks));
    }
    assert_eq!(runs[0], runs[1], "both fidelities lift and reseat together");
}

/// Gate 5 (fork 5): an inlet line that loses more than the blowdown at full
/// lift drops the valve's own inlet below reseat in the solve that lifts it, so
/// it chatters — the failure API 520 Part II's inlet-loss limit exists for. The
/// demo's 5 m × 60 mm line loses about 0.07 bar at full lift; a 0.05 bar
/// blowdown is inside that.
///
/// Measured, not every tick: mostly open-shut-open, with a second open tick
/// whenever the receiver has crept far enough above set to hold the inlet over
/// reseat once more (`11010101…`). So the gate is the rate and the run length:
/// the demo lifts about 3 times per 200 ticks and stays lifted for ~36.
#[test]
fn an_inlet_line_that_loses_more_than_the_blowdown_chatters() {
    let mut file = load(POP);
    set_blowdown(&mut file, Some(0.05));
    let ticks = run(&file, 600);
    let first = ticks.iter().position(|t| t.lifted).expect("lifts");
    let window = &ticks[first..first + 200];
    let lifts = flips(window).iter().filter(|(_, lifted)| *lifted).count();
    assert!(lifts >= 80, "{lifts} lifts in 200 ticks");
    let longest = window
        .split(|t| !t.lifted)
        .map(<[Tick]>::len)
        .max()
        .unwrap_or(0);
    assert!(longest <= 2, "lifted for {longest} ticks in a row");
}

fn expect_refusal(file: &refinery_scenarios::ScenarioFile, what: &str) -> String {
    match refinery_scenarios::build_engine(file) {
        Ok(_) => panic!("{what}"),
        Err(e) => e.to_string(),
    }
}

const GAS: &str = r#"
[meta]
name = "fixture"
description = "fixture"
[simulation]
dt = 0.1
[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"
[[components]]
name = "fuel_gas"
phase = "gas"
tb_c = -161.5
molar_mass_kg_per_mol = 0.016043
cp_j_per_kg_k = 2220.0
"#;

const GAS_NO_VESSEL: &str = r#"
[nodes.header]
type = "source"
pressure_bar = 30.0
temperature_c = 20.0
[nodes.tee]
type = "junction"
[nodes.psv]
type = "relief_valve"
kv = 12.0
set_pressure_bar = 20.0
accumulation_bar = 1.0
x_t = 0.72
blowdown_bar = 1.4
[nodes.flare]
type = "sink"
pressure_bar = 1.1
temperature_c = 20.0
[[pipes]]
name = "a"
from = "header"
to = "tee"
length_m = 20.0
diameter_m = 0.021
[[pipes]]
name = "b"
from = "tee"
to = "psv"
length_m = 5.0
diameter_m = 0.06
[[pipes]]
name = "c"
from = "psv"
to = "flare"
length_m = 30.0
diameter_m = 0.1
"#;

const VESSEL_ON_THE_TEE: &str = r#"
[nodes.drum]
type = "vessel"
volume_m3 = 2.0
pressure_bar = 12.0
temperature_c = 20.0
[[pipes]]
name = "d"
from = "drum"
to = "tee"
length_m = 2.0
diameter_m = 0.1
"#;

const LIQUID: &str = r#"
[meta]
name = "fixture"
description = "fixture"
[simulation]
dt = 0.1
[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"
[nodes.supply]
type = "source"
pressure_bar = 10.0
temperature_c = 20.0
[nodes.psv]
type = "relief_valve"
kv = 5.0
set_pressure_bar = 8.0
accumulation_bar = 0.5
blowdown_bar = 0.5
[nodes.drain]
type = "sink"
pressure_bar = 1.0
temperature_c = 20.0
[[pipes]]
name = "a"
from = "supply"
to = "psv"
length_m = 5.0
diameter_m = 0.05
[[pipes]]
name = "b"
from = "psv"
to = "drain"
length_m = 5.0
diameter_m = 0.05
"#;

/// Gate 6 (forks 4 and 6): the loader refuses a blowdown out of range, in
/// liquid service, and in gas with no vessel behind the valve — and accepts one
/// reached through a junction.
#[test]
fn a_blowdown_without_a_gas_cushion_or_out_of_range_is_refused() {
    for bad in [0.0, -1.0, f64::NAN, SET_BAR, 25.0] {
        let mut file = load(POP);
        set_blowdown(&mut file, Some(bad));
        let msg = expect_refusal(&file, &format!("blowdown_bar = {bad} must not load"));
        assert!(msg.contains("blowdown_bar"), "names the field: {msg}");
    }
    let msg = expect_refusal(&load(LIQUID), "a liquid pop valve must not load");
    assert!(msg.contains("liquid service"), "names the service: {msg}");
    let msg = expect_refusal(
        &load(&format!("{GAS}{GAS_NO_VESSEL}")),
        "a pop valve with no vessel behind it must not load",
    );
    assert!(msg.contains("no gas vessel"), "names the reason: {msg}");
    refinery_scenarios::build_engine(&load(&format!("{GAS}{GAS_NO_VESSEL}{VESSEL_ON_THE_TEE}")))
        .expect("a vessel reached through a junction is a cushion");

    let mut liquid = load(LIQUID);
    set_blowdown(&mut liquid, None);
    refinery_scenarios::build_engine(&liquid).expect("the fixture loads without the key");
}
