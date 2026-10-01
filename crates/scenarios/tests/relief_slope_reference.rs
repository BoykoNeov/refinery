//! Newton's relief slope (M26.1, docs/DESIGN.md §30, ledger row A14).
//!
//! **The mechanism.** A PSV's opening is read off its own inlet pressure and
//! frozen into the compiled branch, so the branch's `flow_ddp` holds the opening
//! fixed. On a vessel at a long timestep the missing "opens wider" share of
//! `∂ṁ/∂P_src` is as large as everything else holding the vessel, so each Newton
//! step overshoots by about a whole step — ratio −0.855 measured on the twin plant
//! at `dt = 1.0` — and Armijo accepts every one. The twin failed at tick 17; the
//! twin's drum ALONE crawled 19 iterations through its lift, and `relief_blowdown`
//! at `dt = 1.0` took 20. A14's "two relieving vessels" was not the mechanism.
//!
//! **The fix** is the missing term, `ṁ·k` with
//! `k = CompiledEdge::relief_opening_log_slope`, in the source column of
//! Newton's Jacobian. These gates:
//!
//! | gate | asserts |
//! |---|---|
//! | 1 | the term equals a centred difference of the full-recompile flow (gas 2%, liquid 1e-6), and is exactly `0.0` wherever it should be |
//! | 2 | the twin plant at `dt = 1.0` runs 6 000 ticks on Newton (failed at tick 17) |
//! | 3 | on that plant and on `relief_blowdown` at `dt = 1.0`, no tick takes more Newton iterations than the cold start |
//!
//! Gate 4 (what must not move) and gate 5 (the proptest reachability counts)
//! are measurements against baselines recorded before the change, reported in
//! §30's "Corrections from building it", not tests: CI commits no baseline.

use refinery_core::energy::NodeStates;
use refinery_core::graph::{EdgeId, NodeId, PlantGraph};
use refinery_core::units::Seconds;
use refinery_scenarios::ScenarioFile;
use refinery_solvers::network::{compile_edge, CompiledEdge};
use refinery_solvers::NewtonFlowSolver;
use std::collections::BTreeMap;

const RELIEF: &str = include_str!("../../../scenarios/relief_blowdown.toml");
const TWIN: &str = include_str!("../../../scenarios/relief_twin_vessels.toml");
const VALVE: &str = include_str!("../../../scenarios/tank_pump_valve.toml");

/// `relief_blowdown.toml`'s declared relief spec, restated rather than read back.
const SET_PA: f64 = 20.0e5;
const ACCUMULATION_PA: f64 = 1.0e5;
/// Its flare's pressure.
const FLARE_PA: f64 = 1.1e5;

/// A liquid relief valve between a pinned source and a sink. Water (no
/// `[[components]]`), so the valve composes with its pipe in closed form and the
/// log slope is EXACT, which is what lets gate 1's liquid half be tight.
const LIQUID_RELIEF: &str = r#"
[meta]
name = "liquid_relief"
[simulation]
dt = 1.0
[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"

[nodes.header]
type = "source"
pressure_bar = 4.0
temperature_c = 20.0

[nodes.psv]
type = "relief_valve"
kv = 60.0
set_pressure_bar = 3.0
accumulation_bar = 0.3

[nodes.drain]
type = "sink"
pressure_bar = 1.01325

[[pipes]]
name = "riser"
from = "header"
to = "psv"
length_m = 5.0
diameter_m = 0.10

[[pipes]]
name = "tail"
from = "psv"
to = "drain"
length_m = 5.0
diameter_m = 0.10
"#;

fn load(src: &str) -> ScenarioFile {
    refinery_scenarios::load_str(src).expect("scenario parses")
}

/// A built plant's graph and slate, which is all `compile_edge` reads.
fn plant(src: &str) -> refinery_core::engine::Engine {
    refinery_scenarios::build_engine(&load(src)).expect("builds")
}

fn edge(graph: &PlantGraph, name: &str) -> EdgeId {
    graph
        .edge_ids()
        .find(|e| graph.pipe(*e).name == name)
        .unwrap_or_else(|| panic!("plant must declare a '{name}' pipe"))
}

fn node(graph: &PlantGraph, name: &str) -> NodeId {
    graph
        .find_node(name)
        .unwrap_or_else(|| panic!("plant must declare a '{name}' node"))
}

/// Every node at `fill`, with the named overrides.
fn pressures(graph: &PlantGraph, fill: f64, set: &[(&str, f64)]) -> BTreeMap<NodeId, f64> {
    let mut out: BTreeMap<NodeId, f64> = graph.node_ids().map(|n| (n, fill)).collect();
    for (name, p) in set {
        out.insert(node(graph, name), *p);
    }
    out
}

fn compiled(
    engine: &refinery_core::engine::Engine,
    eid: EdgeId,
    p: &BTreeMap<NodeId, f64>,
) -> CompiledEdge {
    compile_edge(&engine.graph, eid, &engine.slate, &NodeStates::default(), p)
        .expect("edge compiles")
}

/// The edge's mass flow at `p`, through a FULL recompile: the opening, the
/// density and the gas fold all follow the pressures.
fn mass_flow(
    engine: &refinery_core::engine::Engine,
    eid: EdgeId,
    p: &BTreeMap<NodeId, f64>,
) -> f64 {
    let c = compiled(engine, eid, p);
    let eps = NewtonFlowSolver::default().eps_dp;
    c.rho * c.branch.flow(p[&c.src] - p[&c.tgt], eps)
}

/// A plant built from `src` with the relief valve `psv`'s set pressure moved by
/// `by_pa`.
fn with_set_moved(src: &str, psv: &str, by_pa: f64) -> refinery_core::engine::Engine {
    let mut file = load(src);
    match file.nodes.get_mut(psv) {
        Some(refinery_scenarios::NodeDef::ReliefValve {
            set_pressure_bar, ..
        }) => *set_pressure_bar += by_pa / 1e5,
        other => panic!("'{psv}' must be a relief valve, got {other:?}"),
    }
    refinery_scenarios::build_engine(&file).expect("builds")
}

/// `(the opening term ṁ·k, the opening's share by centred difference, the
/// frozen-opening slope g)` on the PSV's outlet `outlet` at the pressures `p`.
///
/// **The difference moves the SET pressure, not the inlet pressure.** The opening
/// is `smoothstep((P_src − P_set)/accumulation)`, so lowering `P_set` by `h`
/// opens the valve exactly as raising `P_src` by `h` would — while the pressure
/// drop, the upwind density and the gas fold's `x = s/p_up` stay where they are.
/// Differencing in `P_src` instead measures those too, and they are row A18's
/// terms, which Newton omits on purpose: above the band, where the opening term
/// is exactly zero, they are 2.2e-7 against `g`'s 2.5e-7, and near full lift they
/// are 4% of the total slope. The opening's share is what this gate is about.
fn opening_share(
    src: &str,
    psv: &str,
    outlet: &str,
    p_named: &[(&str, f64)],
    h: f64,
) -> (f64, f64, f64) {
    let engine = plant(src);
    let eid = edge(&engine.graph, outlet);
    let p = pressures(&engine.graph, p_named[0].1, p_named);
    let c = compiled(&engine, eid, &p);
    let eps = NewtonFlowSolver::default().eps_dp;
    let dp = p[&c.src] - p[&c.tgt];
    let mdot = c.rho * c.branch.flow(dp, eps);
    let g = c.rho * c.branch.flow_ddp(dp, eps);
    let lower = with_set_moved(src, psv, -h);
    let upper = with_set_moved(src, psv, h);
    let share = (mass_flow(&lower, eid, &p) - mass_flow(&upper, eid, &p)) / (2.0 * h);
    (mdot * c.relief_opening_log_slope, share, g)
}

/// **Gate 1, gas half — the term is the opening's share of `∂ṁ/∂P_src`.** On
/// `relief_blowdown`'s PSV outlet, across its band, the term agrees with the
/// opening's share by centred difference (see `opening_share`) to 1%.
///
/// The 1% is the gas fold's approximation and nothing else: the term holds
/// `(x/x_s)/Y²` fixed while the opening moves (DESIGN §30 fork 3). Measured
/// worst over the band: see `measure_the_relief_slope_gates`.
///
/// The control comes first: the frozen-opening slope `g` must be far below the
/// opening's share, or the gate could not tell the term from its absence.
#[test]
fn the_opening_term_matches_a_centred_difference_on_a_gas_relief() {
    for t in [0.1, 0.3, 0.5, 0.7, 0.9, 0.95] {
        let (term, share, g) = opening_share(
            RELIEF,
            "psv",
            "flare_line",
            &[("psv", SET_PA + t * ACCUMULATION_PA), ("flare", FLARE_PA)],
            1.0,
        );
        assert!(
            g < 0.1 * share,
            "control, t = {t}: the frozen-opening slope {g:e} must be well below the \
             opening's share {share:e}, or this gate cannot see the opening term"
        );
        let rel = (term - share).abs() / share;
        assert!(
            rel < 1e-2,
            "t = {t}: the opening term {term:e} against the opening's share by centred \
             difference {share:e} — off by {rel:.3e}"
        );
    }
}

/// **Gate 1, liquid half.** A liquid relief valve composes with its pipe in
/// closed form, so the term is the EXACT opening share and must match the centred
/// difference to 1e-6.
#[test]
fn the_opening_term_is_exact_on_a_liquid_relief() {
    let (set, band) = (3.0e5, 0.3e5);
    for t in [0.1, 0.3, 0.5, 0.7, 0.9, 0.95] {
        let (term, share, g) = opening_share(
            LIQUID_RELIEF,
            "psv",
            "tail",
            &[("psv", set + t * band), ("drain", 1.01325e5)],
            1.0,
        );
        assert!(
            g < 0.5 * share,
            "control, t = {t}: the frozen-opening slope {g:e} must be well below the \
             opening's share {share:e}"
        );
        let rel = (term - share).abs() / share;
        assert!(
            rel < 1e-6,
            "t = {t}: the opening term {term:e} against the opening's share by centred \
             difference {share:e} — off by {rel:.3e}; a liquid relief's term is exact"
        );
    }
}

/// **Gate 1, the zeros.** The term is exactly `+0.0` — not small, not `-0.0` —
/// below set, at set, above set + accumulation, and in the sliver just above set
/// where the opening is positive but snaps shut below `OPEN_EPS` (there the
/// expression is `0·∞/0`). This is what keeps a plant whose PSV never lifts bit
/// for bit what it was.
#[test]
fn the_opening_term_is_exactly_zero_outside_the_band_and_when_snapped_shut() {
    let engine = plant(RELIEF);
    let outlet = edge(&engine.graph, "flare_line");
    for (what, p_psv) in [
        ("below set", SET_PA - 0.5 * ACCUMULATION_PA),
        ("at set", SET_PA),
        ("in the snapped sliver", SET_PA + 1e-4 * ACCUMULATION_PA),
        ("at full lift", SET_PA + ACCUMULATION_PA),
        ("above full lift", SET_PA + 3.0 * ACCUMULATION_PA),
    ] {
        let p = pressures(
            &engine.graph,
            SET_PA,
            &[("psv", p_psv), ("flare", FLARE_PA)],
        );
        let k = compiled(&engine, outlet, &p).relief_opening_log_slope;
        assert_eq!(
            k.to_bits(),
            0.0f64.to_bits(),
            "{what}: the relief opening's log slope must be exactly +0.0, got {k:e}"
        );
    }
    // The sliver is a real case, not a duplicate of "at set": the opening's own
    // slope is nonzero there, and only the snapped opening makes the term zero.
    let sliver_slope = refinery_solvers::elements::relief_opening_slope(
        SET_PA + 1e-4 * ACCUMULATION_PA,
        SET_PA,
        ACCUMULATION_PA,
    );
    assert!(
        sliver_slope > 0.0,
        "the sliver must have a nonzero opening slope ({sliver_slope:e}), or the \
         case above does not exercise the snapped-opening guard"
    );
}

/// **Gate 1, every other edge.** Only a relief valve's outlet carries the term:
/// a pipe into a PSV, a make-up line, and an operator valve's outlet all carry
/// exactly `0.0`, in band or not. An operator valve's opening is a setpoint, not
/// a function of pressure, so any nonzero term there is simply wrong.
#[test]
fn only_a_relief_valves_outlet_carries_the_term() {
    let engine = plant(RELIEF);
    let in_band = pressures(
        &engine.graph,
        SET_PA + 0.5 * ACCUMULATION_PA,
        &[("header", 30.0e5), ("flare", FLARE_PA)],
    );
    for name in ["make_up", "psv_inlet"] {
        let k = compiled(&engine, edge(&engine.graph, name), &in_band).relief_opening_log_slope;
        assert_eq!(
            k.to_bits(),
            0.0f64.to_bits(),
            "'{name}' must carry +0.0, got {k:e}"
        );
    }
    let outlet = compiled(&engine, edge(&engine.graph, "flare_line"), &in_band);
    assert!(
        outlet.relief_opening_log_slope > 0.0,
        "control: the PSV's own outlet must carry the term at mid-band"
    );

    let valve_plant = plant(VALVE);
    let p = pressures(&valve_plant.graph, 3.0e5, &[]);
    for eid in valve_plant.graph.edge_ids() {
        let k = compiled(&valve_plant, eid, &p).relief_opening_log_slope;
        assert_eq!(
            k.to_bits(),
            0.0f64.to_bits(),
            "'{}' on tank_pump_valve must carry +0.0, got {k:e}",
            valve_plant.graph.pipe(eid).name
        );
    }
}

/// Run a plant on Newton and return each tick's iteration count, failing with the
/// tick and the error if any tick fails.
fn newton_iterations(mut file: ScenarioFile, dt: f64, ticks: u64) -> Vec<u32> {
    file.fidelity.flow = "newton".to_string();
    file.simulation.dt = dt;
    let mut engine = refinery_scenarios::build_engine(&file).expect("builds");
    assert_eq!(
        engine.dt(),
        Seconds(dt),
        "the override must reach the engine"
    );
    (1..=ticks)
        .map(|tick| {
            engine.tick().unwrap_or_else(|e| {
                panic!("{} at dt = {dt}: tick {tick} failed: {e}", file.meta.name)
            });
            engine.snapshot().solver.iterations
        })
        .collect()
}

/// **Gate 3's assertion**: no tick takes more Newton iterations than the cold
/// start. A comparison rather than a ceiling, because the claim is that lifting a
/// relief valve is no harder than starting from nothing.
fn assert_no_tick_beats_the_cold_start(iterations: &[u32], what: &str) {
    let cold = iterations[0];
    let (worst_tick, worst) = iterations
        .iter()
        .enumerate()
        .max_by_key(|(i, n)| (**n, std::cmp::Reverse(*i)))
        .map(|(i, n)| (i + 1, *n))
        .expect("at least one tick");
    assert!(
        worst <= cold,
        "{what}: tick {worst_tick} took {worst} Newton iterations against the cold \
         start's {cold} — the relief band is harder than starting from nothing, which is \
         the missing opening slope's signature (DESIGN §30)"
    );
}

/// **Gates 2 and 3 on the twin plant at `dt = 1.0`.** The shipped file sits at
/// `dt = 0.1` because Newton failed it at tick 17 at one second: the drum's PSV
/// lifts there and every step overshot. It runs 6 000 ticks now, and its lift is
/// no harder than its cold start (7 iterations, at tick 1).
#[test]
fn the_twin_plant_runs_at_a_one_second_tick_on_newton() {
    let iterations = newton_iterations(load(TWIN), 1.0, 6000);
    assert_no_tick_beats_the_cold_start(&iterations, "relief_twin_vessels at dt = 1.0");
}

/// **Gate 3 on `relief_blowdown` at `dt = 1.0`.** One vessel, which never failed:
/// its lift tick took 20 iterations against a cap of 50 and nobody looked (§25's
/// first table recorded it as unaffected). Now 6, at tick 1.
#[test]
fn a_single_relief_lifts_no_harder_than_a_cold_start_at_a_one_second_tick() {
    let iterations = newton_iterations(load(RELIEF), 1.0, 6000);
    assert_no_tick_beats_the_cold_start(&iterations, "relief_blowdown at dt = 1.0");
}

/// Measurement, not a gate: the worst ticks the gates above quote.
/// `cargo test --release -p refinery-scenarios --test relief_slope_reference -- \
/// --ignored --nocapture`
#[test]
#[ignore = "measurement, not a gate"]
fn measure_the_relief_slope_gates() {
    for (name, src) in [("relief_twin_vessels", TWIN), ("relief_blowdown", RELIEF)] {
        let iterations = newton_iterations(load(src), 1.0, 6000);
        let worst = iterations.iter().max().expect("ticks");
        let total: u32 = iterations.iter().sum();
        println!(
            "{name} at dt = 1.0: tick 1 {}, worst {worst}, total {total}",
            iterations[0]
        );
    }
    for t in [0.1, 0.3, 0.5, 0.7, 0.8, 0.9, 0.95] {
        let (term, share, g) = opening_share(
            RELIEF,
            "psv",
            "flare_line",
            &[("psv", SET_PA + t * ACCUMULATION_PA), ("flare", FLARE_PA)],
            1.0,
        );
        let (lterm, lshare, _) = opening_share(
            LIQUID_RELIEF,
            "psv",
            "tail",
            &[("psv", 3.0e5 + t * 0.3e5), ("drain", 1.01325e5)],
            1.0,
        );
        println!(
            "t = {t}: gas term {term:.6e}, share {share:.6e}, g {g:.3e}, off by {:.3e}; \
             liquid off by {:.3e}",
            (term - share).abs() / share,
            (lterm - lshare).abs() / lshare
        );
    }
}
