//! Analytic hand-calculation reference for the valve and the M1 reference
//! plant (docs/ROADMAP.md §M1, last open box).
//!
//! Location note: CLAUDE.md puts reference cases in `solvers/tests/reference/`,
//! but the quantity under test here — the `Kv → cv_si` conversion — is private
//! to `refinery-scenarios` and only observable through the loader. The test has
//! to live in the crate that owns the conversion, so this is a deliberate
//! deviation rather than an oversight.
//!
//! What this file exists to catch: a wrong Kv conversion is *invisible* to every
//! other M1 test. It still conserves mass, still converges, still reruns
//! bit-identically, and both fidelities agree on the same wrong number — all
//! green. Only a comparison against an externally-defined value finds it.
//!
//! The two tests pin different things, and the distinction matters:
//!
//! - `kv_definition_*` is **truly independent**: its expected value comes from
//!   the published definition of metric Kv (IEC 60534-2-1 / ISA-75.01), not
//!   from any formula in this workspace.
//! - `tank_pump_valve_*` pins the plant's *magnitude*. Its series-resistance
//!   arithmetic necessarily mirrors `QuadraticBranch`'s algebra — that is the
//!   inherent ceiling of any network hand-calc. It catches wrong constants,
//!   sign/fold errors and unit slips, which is the point; it is not a check of
//!   the model's formulation. The Kv test above is what pins the conversion
//!   independently.

use refinery_core::components::Slate;
use refinery_core::graph::{EdgeId, NodeKind, PlantGraph};
use refinery_core::traits::FlowSolver;
use refinery_core::units::Seconds;
use refinery_solvers::elements::valve_flow;
use refinery_solvers::{NewtonFlowSolver, SimpleFlowSolver};

const REFERENCE_PLANT: &str = include_str!("../../../scenarios/tank_pump_valve.toml");

/// Pressure drop at which metric Kv is defined [Pa] = 1 bar.
const KV_DEFINITION_DP_PA: f64 = 1e5;
/// Seconds per hour — the "per hour" in Kv's m³/h.
const SECONDS_PER_HOUR: f64 = 3600.0;

// ---------------------------------------------------------------------------
// Test A — the Kv conversion, against the published definition.
// ---------------------------------------------------------------------------

/// Minimal well-posed plant exercising the loader's valve path: the valve needs
/// exactly one inlet and one outlet edge, and the component needs a pressure
/// reference, or `validate_topology` rejects it before we can read `cv_max`.
fn valve_scenario(kv: f64, opening: f64) -> String {
    format!(
        r#"
[meta]
name = "kv_probe"
description = "Single valve, used to read back the loader's Kv conversion."

[simulation]
dt = 0.1

[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"

[nodes.src]
type = "source"
pressure_bar = 2.0
temperature_c = 20.0

[nodes.probe_valve]
type = "valve"
kv = {kv}
opening = {opening}

[nodes.snk]
type = "sink"
pressure_bar = 1.0

[[pipes]]
name = "in"
from = "src"
to = "probe_valve"
length_m = 1.0
diameter_m = 0.1

[[pipes]]
name = "out"
from = "probe_valve"
to = "snk"
length_m = 1.0
diameter_m = 0.1
"#
    )
}

/// Read `cv_max` back out of a built graph's valve — the loader's conversion
/// output, which is otherwise private.
fn loaded_cv_si(kv: f64, opening: f64) -> f64 {
    let file = refinery_scenarios::load_str(&valve_scenario(kv, opening)).expect("probe parses");
    let engine = refinery_scenarios::build_engine(&file).expect("probe builds");
    let valve = engine
        .graph
        .find_node("probe_valve")
        .expect("probe has a valve");
    match engine.graph.node(valve).kind {
        NodeKind::Valve { cv_max, .. } => cv_max,
        ref other => panic!("probe_valve must load as a Valve, got {other:?}"),
    }
}

/// THE definitional check, and the one genuinely independent assertion in this
/// file: a valve of coefficient Kv passes **Kv cubic metres of water per hour
/// at a 1 bar drop, SG = 1**, wide open (IEC 60534-2-1 / ISA-75.01).
///
/// Applying that to the SI law the solver actually uses,
/// `Q[m³/s] = cv_si·√(dP[Pa]/SG)`, the definition must come back out:
///
///   Q(dP = 1e5, SG = 1, opening = 1) = cv_si·√1e5  ==  Kv/3600 [m³/s]
///
/// The expected side is the *definition*; nothing here re-derives the loader's
/// `cv_si = Kv/(3600·√1e5)`, so a botched conversion cannot hide in both sides.
/// The ISA law is applied inline rather than through `elements::valve_flow`,
/// whose zero-flow regularization would blur an exact identity into a ~5e-6
/// tolerance argument for no benefit.
#[test]
fn kv_definition_holds_through_the_loader() {
    // Spread over decades: a conversion error is almost always a constant
    // factor, but scale-dependent slips (e.g. a stray square) show up here.
    for kv in [1.0f64, 50.0, 250.0, 1600.0] {
        let cv_si = loaded_cv_si(kv, 1.0);

        // ISA-75.01 in SI, wide open: Q = cv_si·√(dP/SG), SG = 1 for water.
        let q_at_one_bar = cv_si * KV_DEFINITION_DP_PA.sqrt();
        // The definition of Kv: that flow is Kv m³/h, expressed in m³/s.
        let expected = kv / SECONDS_PER_HOUR;

        approx::assert_relative_eq!(q_at_one_bar, expected, max_relative = 1e-12);
    }
}

/// At half open a Kv = 50 valve passes 25 m³/h at 1 bar.
///
/// Two separable claims, both load-bearing for the hand calc below:
///
/// 1. `cv_max` is the **wide-open** coefficient. The loader stores the Kv
///    conversion unscaled and the trim is applied at solve time
///    (`QuadraticBranch::valve`), so `cv_max` must NOT move with `opening` —
///    scaling it in the loader as well would square the trim.
/// 2. Trim is **linear** (`cv_eff = cv_si·opening`). That is a convention
///    chosen in `elements.rs`, not physics — equal-percentage is the other
///    common trim and is a documented future option. `HAND_CALC_MASS_FLOW_KG_S`
///    assumes linear at 50% open, so if the convention ever changes this test
///    should fail alongside the hand calc rather than let it drift silently.
#[test]
fn valve_at_half_open_passes_half_the_kv_flow() {
    let kv = 50.0;
    let cv_si = loaded_cv_si(kv, 1.0);

    // (1) The conversion is opening-independent.
    approx::assert_relative_eq!(loaded_cv_si(kv, 0.5), cv_si, max_relative = 1e-12);
    // ...and the opening itself passes through the loader untouched.
    let file = refinery_scenarios::load_str(&valve_scenario(kv, 0.5)).expect("parses");
    let engine = refinery_scenarios::build_engine(&file).expect("builds");
    let valve = engine.graph.find_node("probe_valve").expect("has a valve");
    match engine.graph.node(valve).kind {
        NodeKind::Valve { opening, .. } => {
            approx::assert_relative_eq!(opening, 0.5, max_relative = 1e-12)
        }
        ref other => panic!("expected a Valve, got {other:?}"),
    }

    // (2) The composed physical claim, loader + element together. `eps` is set
    // tiny so the zero-flow regularization cannot blur an exact identity.
    let q_half = valve_flow(KV_DEFINITION_DP_PA, cv_si, 0.5, 1.0, 1e-9);
    approx::assert_relative_eq!(q_half, 0.5 * kv / SECONDS_PER_HOUR, max_relative = 1e-9);
}

// ---------------------------------------------------------------------------
// Test B — the reference plant's flow magnitude, against a hand calculation.
// ---------------------------------------------------------------------------

/// Hand-calculated mass flow through `tank_pump_valve.toml` at its INITIAL
/// levels (supply 8.0 m, receiving 1.0 m) [kg/s].
///
/// Derivation — walk the pressure along the single series path
/// `supply_tank → suction → pump → discharge → valve → fill_line → receiving_tank`
/// and find the flow Q [m³/s] that lands exactly on the receiving tank's bottom
/// pressure. With ρ = 998 kg/m³, g = 9.80665 m/s², P_atm = 101325 Pa:
///
/// - Tank bottoms (hydrostatic, both vented): P = P_atm + ρ·g·h
///   supply    = 101325 + 998·9.80665·8.0 = 179621.2936 Pa;
///   receiving = 101325 + 998·9.80665·1.0 = 111112.0367 Pa
/// - Pipes (Darcy–Weisbach, dP = k·Q², k = f·L·ρ/(2·D·A²), A = πD²/4, f = 0.02):
///   suction   (L=10, D=0.15): k = 2.13053e6 Pa/(m³/s)²;
///   discharge (L=30, D=0.10): k = 4.85369e7 Pa/(m³/s)²;
///   fill_line (L=20, D=0.10): k = 3.23579e7 Pa/(m³/s)²
/// - Pump (H = h0 − a·Q², h0 = 40 m, a = 800): rise dP = ρ·g·H
/// - Valve, HALF open, cv_eff = cv_si·0.5 with `cv_si` taken from the **Kv
///   definition** (Kv = 50 m³/h at 1 bar, SG 1 ⇒ cv_si = (50/3600)/√1e5 =
///   4.392052e-5), NOT from the loader's `kv_to_cv_si` — reusing the code's own
///   conversion here would hide a bug in it on both sides of the assertion, and
///   this test's whole job is to catch exactly that. ISA inverted: dP =
///   SG·(Q/cv_eff)².
/// - fill_line also lifts 5 m: static dP = ρ·g·5 = 48935.18 Pa.
///
/// Solving that balance for Q: Q = 0.01378085 m³/s ⇒ ṁ = ρ·Q = 13.753287 kg/s.
/// (Root-found to 1e-15; the roadmap's "~13.7 kg/s" estimate agrees.)
const HAND_CALC_MASS_FLOW_KG_S: f64 = 13.753287;

/// Tolerance for the hand calc [relative].
///
/// Slack is dominated by the solver's `eps_dp = 1.0` Pa sqrt-regularization,
/// which stiffens each of the three branches by ~1 Pa against a ~411 kPa total
/// driving head — a ~4e-6 relative shift in Q. 1e-3 clears that by ~250x while
/// staying orders of magnitude tighter than any plausible conversion bug (the
/// smallest realistic Kv slip, dropping the √ on the bar→Pa factor, is off by
/// ~300x).
const HAND_CALC_TOLERANCE: f64 = 1e-3;

fn reference_graph() -> PlantGraph {
    let file = refinery_scenarios::load_str(REFERENCE_PLANT).expect("reference scenario parses");
    refinery_scenarios::build_engine(&file)
        .expect("reference plant builds")
        .graph
}

fn edge_by_name(graph: &PlantGraph, name: &str) -> EdgeId {
    graph
        .edge_ids()
        .find(|e| graph.pipe(*e).name == name)
        .unwrap_or_else(|| panic!("reference plant must have a '{name}' pipe"))
}

/// The roadmap's M1 hand-calc box: pump fills a tank through a valve, compare
/// the steady flow to the analytic value.
///
/// Solved directly on the freshly-built graph rather than through `tick()`: the
/// plant is closed, so the levels equalize and there is no true steady state.
/// The well-defined anchor is the quasi-steady solve at the scenario's *initial*
/// levels, which is exactly the state the hand calc above describes.
///
/// Both fidelities are pinned to the same analytic number, not just to each
/// other. `fidelity_agreement.rs` proves they agree; agreement alone is
/// satisfied by two solvers being wrong together, which a shared bad Kv
/// conversion would produce.
#[test]
fn tank_pump_valve_flow_matches_hand_calc() {
    let graph = reference_graph();
    let slate = Slate::water_only();
    let dt = Seconds(0.1);

    let newton = NewtonFlowSolver::default()
        .solve(&graph, &slate, dt)
        .expect("the reference plant must converge");
    let simple = SimpleFlowSolver::default()
        .solve(&graph, &slate, dt)
        .expect("the reference plant must converge under the simple fidelity too");

    let discharge = edge_by_name(&graph, "discharge");
    for (fidelity, sol) in [("newton", &newton), ("simple", &simple)] {
        let flow = sol.edge_mass_flow[&discharge];
        assert!(
            flow > 0.0,
            "{fidelity}: the pump must drive flow supply → receiving, got {flow} kg/s"
        );
        approx::assert_relative_eq!(
            flow,
            HAND_CALC_MASS_FLOW_KG_S,
            max_relative = HAND_CALC_TOLERANCE
        );
    }
}

/// The path is a pure series chain, so every edge carries the same mass flow.
/// This is what makes the single hand-calculated number above meaningful for
/// the whole plant rather than just for one edge.
#[test]
fn reference_plant_series_path_carries_one_flow() {
    let graph = reference_graph();
    let sol = NewtonFlowSolver::default()
        .solve(&graph, &Slate::water_only(), Seconds(0.1))
        .expect("converges");

    let suction = sol.edge_mass_flow[&edge_by_name(&graph, "suction")];
    for name in ["discharge", "fill_line"] {
        approx::assert_relative_eq!(
            sol.edge_mass_flow[&edge_by_name(&graph, name)],
            suction,
            max_relative = 1e-6
        );
    }
}
