//! Gas density `ρ = P·M̄/(R·T)` as wired (docs/ROADMAP.md §M5.2, docs/DESIGN.md
//! §3a fork 1) — the reference plant, the upwind convention, the single-phase
//! load-time guards, and the phase-correct pipe seed.
//!
//! Location note, the same deliberate deviation `kv_reference.rs` records: the
//! quantities under test here are only observable through the loader (the phase
//! field, the slate it builds, the topological guard, the pipe seed), so the
//! tests live in the crate that owns them rather than in
//! `solvers/tests/reference/`.
//!
//! **What each gate pins, and what it deliberately does not.**
//!
//! The hand calc below carries ONE density per edge — its upwind node's — while
//! a real gas expands along the pipe and its density falls with it. So this file
//! pins the **density law and the upwind convention as implemented**; it is not
//! a physics anchor for compressible pipe flow, and the error it accepts grows
//! with the pressure ratio across the edge. That is what M5.4's ISA gas-sizing
//! anchor is for, and why the roadmap orders the two slices rather than merging
//! them. Stated here so nobody later reads a passing test as more than it is.
//!
//! Its second ceiling is `kv_reference`'s: the series algebra necessarily mirrors
//! `QuadraticBranch`'s, so it catches wrong constants, unit slips and a wrong
//! choice of evaluation state — not an error in the model's formulation. What is
//! genuinely independent is `R_GAS` and the declared `M̄`: neither the value of
//! `ρ` nor its dependence on `P` is read back from the workspace.

use refinery_core::graph::{EdgeId, NodeId, PlantGraph};
use refinery_core::traits::FlowSolver;
use refinery_core::units::Seconds;
use refinery_scenarios::NodeDef;
use refinery_solvers::{NewtonFlowSolver, SimpleFlowSolver};

const GAS_PLANT: &str = include_str!("../../../scenarios/gas_line.toml");

// --- The reference plant's declared constants, restated here ------------------
// Restated rather than read back: a test that fetches its expected inputs from
// the object under test can only ever check self-consistency.

/// CODATA universal gas constant [J/(mol·K)], exact since the 2019 SI redefinition.
const R: f64 = 8.314_462_618_153_24;
/// `gas_line.toml`'s fuel gas: methane's molar mass [kg/mol].
const M_BAR: f64 = 0.016_043;
/// Both pipes' stream temperature on the first solve [K] = `T_AMBIENT`, which is
/// what `Stream::stagnant` seeds and also what the file declares for the header
/// (20 °C), so the two agree and the hand calc is exact rather than approximate.
///
/// This is why every gate here solves the FRESHLY BUILT graph instead of ticking:
/// once transport runs, M5.1's frictional dissipation raises the stream
/// temperature (to ~375 K at steady state on this plant, a real effect of a
/// 130 kW dissipation into 0.9 kg/s of gas) and the density would then depend on
/// a thermal history rather than on declared numbers.
const T_SEED_K: f64 = 293.15;
const P_HEADER_PA: f64 = 10.0e5;
const P_FLARE_PA: f64 = 1.0e5;
const FRICTION: f64 = 0.02;

/// `2·D·A²/(f·L)` for one pipe — the geometry group in `ṁ² = ρ_upwind·ΔP·K`.
///
/// Derivation: Darcy–Weisbach gives `ΔP = f·(L/D)·ρ·v²/2` with `v = Q/A`, i.e.
/// `ΔP = f·L·ρ·Q²/(2·D·A²)`. With `ṁ = ρ·Q`, `ṁ² = ρ²Q² = ρ·ΔP·2DA²/(f·L)`.
fn geometry_group(length_m: f64, diameter_m: f64) -> f64 {
    let area = std::f64::consts::PI * diameter_m * diameter_m / 4.0;
    2.0 * diameter_m * area * area / (FRICTION * length_m)
}

/// `ρ/P` for the reference gas at the seed temperature [kg/(m³·Pa)] — the whole
/// content of the ideal-gas law, factored so the P-dependence is explicit.
fn density_per_pascal() -> f64 {
    M_BAR / (R * T_SEED_K)
}

/// The two pipes of `gas_line.toml`, as the file declares them.
fn header_group() -> f64 {
    geometry_group(100.0, 0.08)
}
fn relief_group() -> f64 {
    geometry_group(100.0, 0.05)
}

/// Hand calculation: the tee pressure and the mass flow, with each edge's
/// density taken at ITS OWN upwind node.
///
/// The two edges in series carry the same `ṁ`, so with `ρ = c·P`:
///
/// ```text
/// ṁ² = c·P₀·(P₀ − P_tee)·K_header  =  c·P_tee·(P_tee − P₁)·K_relief
/// ```
///
/// `c` cancels from the pressure equation — which is worth noticing, because it
/// means the TEE PRESSURE alone pins the upwind convention independently of the
/// gas constant, while the FLOW pins the magnitude of `ρ`. The remaining
/// quadratic in `P_tee` is
///
/// ```text
/// K_relief·P_tee² + (K_header·P₀ − K_relief·P₁)·P_tee − K_header·P₀² = 0
/// ```
///
/// with one positive root (the constant term is negative, so the roots straddle
/// zero).
fn upwind_prediction(p_header: f64) -> (f64, f64) {
    let (k_h, k_r) = (header_group(), relief_group());
    let a = k_r;
    let b = k_h * p_header - k_r * P_FLARE_PA;
    let c = -k_h * p_header * p_header;
    let p_tee = (-b + (b * b - 4.0 * a * c).sqrt()) / (2.0 * a);
    let mass_flow = (density_per_pascal() * p_header * (p_header - p_tee) * k_h).sqrt();
    (p_tee, mass_flow)
}

/// Tolerance for the hand calcs [relative].
///
/// **Derived, not tuned.** The solver regularizes each `√dp` over `eps_dp = 1.0`
/// Pa (`NewtonFlowSolver::eps_dp`), which stiffens a branch by ~1 Pa against its
/// drop. The smaller drop on this plant is the header run's ~73 kPa, so the
/// worst relative softening of a flow is `½·(1/73 000) ≈ 7e-6` — the right order.
/// Measured by `measure_hand_calc_headroom` over both cases: newton 1.5e-6 and
/// 7.2e-7, simple 9.3e-6 and 1.05e-5 (its looser `tol_rel = 1e-6` adds its own
/// convergence slack and dominates here). 1e-4 clears the worst by ~9.5x.
///
/// **Do not loosen past ~1e-3.** The mutations this file exists to catch are not
/// all gross: taking BOTH edges' density at the source rather than each at its
/// own upwind moves the flow by 3.5% (see
/// `each_edge_takes_the_density_of_its_own_upwind_node`), so the headroom above
/// this tolerance is ~35x, not "orders of magnitude".
const HAND_CALC_TOLERANCE: f64 = 1e-4;

fn build(toml_src: &str) -> Result<refinery_core::engine::Engine, refinery_core::error::SimError> {
    refinery_scenarios::build_engine(&refinery_scenarios::load_str(toml_src)?)
}

/// Build and require a REFUSAL, returning the message. `Engine` is not `Debug`,
/// so `expect_err` is unavailable — and matching explicitly also lets the panic
/// say which scenario was wrongly accepted.
fn expect_refusal(toml_src: &str, what: &str) -> String {
    match build(toml_src) {
        Ok(_) => panic!("{what}"),
        Err(e) => e.to_string(),
    }
}

fn edge_by_name(graph: &PlantGraph, name: &str) -> EdgeId {
    graph
        .edge_ids()
        .find(|e| graph.pipe(*e).name == name)
        .unwrap_or_else(|| panic!("plant must have a '{name}' pipe"))
}

fn node_by_name(graph: &PlantGraph, name: &str) -> NodeId {
    graph
        .find_node(name)
        .unwrap_or_else(|| panic!("plant must have a '{name}' node"))
}

/// The reference plant with its header pressure overridden, so the SAME geometry
/// is measured at two pressures. Mutating the loaded file rather than templating
/// a second TOML keeps one source of truth for the plant.
fn gas_plant_at(p_header_bar: f64) -> refinery_core::engine::Engine {
    let mut file = refinery_scenarios::load_str(GAS_PLANT).expect("gas_line.toml parses");
    match file.nodes.get_mut("gas_header") {
        Some(NodeDef::Source { pressure_bar, .. }) => *pressure_bar = p_header_bar,
        other => panic!("gas_line.toml must define gas_header as a source, got {other:?}"),
    }
    refinery_scenarios::build_engine(&file).expect("gas_line.toml builds")
}

// ---------------------------------------------------------------------------
// A — the reference: ρ = P·M̄/(R·T), at two pressures.
// ---------------------------------------------------------------------------

/// The flow and the tee pressure match the ideal-gas hand calc, **at two header
/// pressures**, under both fidelities.
///
/// Two pressures rather than one is the anti-circularity move: a single point
/// can be matched by any density that happens to be right there — a hardcoded
/// constant, a density taken at the wrong node, a `T` in Celsius — whereas the
/// pair pins `ρ ∝ P`, which is the whole content of the law. Doubling the header
/// to 20 bar takes the flow from 0.9861 to 2.0225 kg/s, a factor of **2.051**;
/// a density frozen at its 10 bar value would predict 1.4835 kg/s — 27% low —
/// because it credits the larger `ΔP` and not the denser gas. No P-independent
/// density can sit on both numbers.
#[test]
fn gas_flow_matches_the_ideal_gas_hand_calc_at_two_pressures() {
    for p_bar in [10.0f64, 20.0] {
        let engine = gas_plant_at(p_bar);
        let (expected_tee, expected_flow) = upwind_prediction(p_bar * 1e5);

        let mut newton = NewtonFlowSolver::default();
        let mut simple = SimpleFlowSolver::default();
        for (fidelity, solver) in [
            ("newton", &mut newton as &mut dyn FlowSolver),
            ("simple", &mut simple as &mut dyn FlowSolver),
        ] {
            let sol = solver
                .solve(&engine.graph, &engine.slate, Seconds(0.1))
                .unwrap_or_else(|e| panic!("{fidelity} must converge on the gas plant: {e}"));

            let flow = sol.edge_mass_flow[&edge_by_name(&engine.graph, "header_run")];
            let tee = sol.node_pressure[&node_by_name(&engine.graph, "tee")].value();

            assert!(
                flow > 0.0,
                "{fidelity} @ {p_bar} bar: gas must flow header → flare, got {flow} kg/s"
            );
            approx::assert_relative_eq!(tee, expected_tee, max_relative = HAND_CALC_TOLERANCE);
            approx::assert_relative_eq!(flow, expected_flow, max_relative = HAND_CALC_TOLERANCE);
        }
    }
}

/// The series path carries one flow, which is what makes the single number above
/// meaningful for the plant rather than for one edge. It is NOT free here the way
/// it is for a liquid: the two edges compile at different densities, so equal
/// mass flow through them is a statement about the solve, not about `ρ` cancelling.
/// Measurement, not a gate: prints the deviation each fidelity lands at, which
/// is where `HAND_CALC_TOLERANCE`'s "derived, not tuned" claim comes from.
/// Run with `cargo test -p refinery-scenarios --test gas_density_reference -- /// --ignored --nocapture`.
#[test]
#[ignore = "measurement, not a gate"]
fn measure_hand_calc_headroom() {
    for p_bar in [10.0f64, 20.0] {
        let engine = gas_plant_at(p_bar);
        let (expected_tee, expected_flow) = upwind_prediction(p_bar * 1e5);
        let mut newton = NewtonFlowSolver::default();
        let mut simple = SimpleFlowSolver::default();
        for (fidelity, solver) in [
            ("newton", &mut newton as &mut dyn FlowSolver),
            ("simple", &mut simple as &mut dyn FlowSolver),
        ] {
            let sol = solver
                .solve(&engine.graph, &engine.slate, Seconds(0.1))
                .expect("converges");
            let flow = sol.edge_mass_flow[&edge_by_name(&engine.graph, "header_run")];
            let tee = sol.node_pressure[&node_by_name(&engine.graph, "tee")].value();
            println!(
                "{p_bar:>4} bar {fidelity:>6}: flow {flow:.9} (rel dev {:.3e}),                  tee {tee:.3} (rel dev {:.3e}), iterations {}",
                (flow - expected_flow).abs() / expected_flow,
                (tee - expected_tee).abs() / expected_tee,
                sol.diagnostics.iterations
            );
        }
    }
}

#[test]
fn both_gas_edges_carry_the_same_mass_flow() {
    let engine = gas_plant_at(10.0);
    let sol = NewtonFlowSolver::default()
        .solve(&engine.graph, &engine.slate, Seconds(0.1))
        .expect("converges");
    approx::assert_relative_eq!(
        sol.edge_mass_flow[&edge_by_name(&engine.graph, "relief_line")],
        sol.edge_mass_flow[&edge_by_name(&engine.graph, "header_run")],
        max_relative = 1e-6
    );
}

/// Each edge takes the density of **its own** upwind node, not one density for
/// the whole stream.
///
/// The plausible slip is one `ρ` per stream — the liquid habit, where it is even
/// true. Its prediction is derivable in closed form and is DIFFERENT: with a
/// single `ρ` the density cancels from the pressure balance entirely, leaving the
/// linear `K_h·(P₀ − P_tee) = K_r·(P_tee − P₁)`, i.e. a tee at 9.216 bar instead
/// of 9.269 and a flow 3.5% high. This test asserts the measured flow sits on the
/// per-edge prediction and is far from the shared-density one, so a passing run
/// means the distinction is real and correctly resolved rather than merely
/// unobservable.
#[test]
fn each_edge_takes_the_density_of_its_own_upwind_node() {
    let engine = gas_plant_at(10.0);
    let sol = NewtonFlowSolver::default()
        .solve(&engine.graph, &engine.slate, Seconds(0.1))
        .expect("converges");
    let flow = sol.edge_mass_flow[&edge_by_name(&engine.graph, "header_run")];

    // The alternative: one density for both edges. It cancels, so P_tee is the
    // conductance-weighted mean and the flow follows from the header run alone.
    let (k_h, k_r) = (header_group(), relief_group());
    let shared_tee = (k_h * P_HEADER_PA + k_r * P_FLARE_PA) / (k_h + k_r);
    let shared_flow =
        (density_per_pascal() * P_HEADER_PA * (P_HEADER_PA - shared_tee) * k_h).sqrt();

    let (_, per_edge_flow) = upwind_prediction(P_HEADER_PA);
    approx::assert_relative_eq!(flow, per_edge_flow, max_relative = HAND_CALC_TOLERANCE);

    let gap = (shared_flow - per_edge_flow).abs() / per_edge_flow;
    assert!(
        gap > 0.02,
        "the shared-density alternative must be distinguishable for this test to \
         mean anything; it differs by only {gap:.4}"
    );
    assert!(
        (flow - shared_flow).abs() / shared_flow > 0.5 * gap,
        "measured flow {flow} sits closer to the shared-density prediction \
         {shared_flow} than to the per-edge one {per_edge_flow}"
    );
}

// ---------------------------------------------------------------------------
// B — the single-phase guards, which are what make the two-phase deferral loud.
// ---------------------------------------------------------------------------

/// A two-cut slate: one liquid, one gas, in that order — so component 0 is the
/// LIQUID one, which is what makes the pipe-seed gate below able to fail.
const MIXED_SLATE: &str = r#"
[[components]]
name = "condensate"
tb_c = 100.0
molar_mass_kg_per_mol = 0.018
density_kg_per_m3 = 998.0
cp_j_per_kg_k = 4184.0

[[components]]
name = "fuel_gas"
phase = "gas"
tb_c = -161.5
molar_mass_kg_per_mol = 0.016043
cp_j_per_kg_k = 2220.0
"#;

const PREAMBLE: &str = r#"
[meta]
name = "phase_probe"

[simulation]
dt = 0.1

[fidelity]
flow = "newton"
"#;

/// A gas source and a liquid sink joined by one pipe — the case a per-composition
/// check cannot see, because each composition on its own is single-phase.
#[test]
fn two_phases_in_one_connected_component_are_refused() {
    let src = format!(
        r#"{PREAMBLE}{MIXED_SLATE}
[nodes.gas_side]
type = "source"
pressure_bar = 10.0
temperature_c = 20.0
composition = {{ fuel_gas = 1.0 }}

[nodes.liquid_side]
type = "sink"
pressure_bar = 1.0
composition = {{ condensate = 1.0 }}

[[pipes]]
name = "line"
from = "gas_side"
to = "liquid_side"
length_m = 10.0
diameter_m = 0.1
"#
    );
    let text = expect_refusal(&src, "a gas line feeding a liquid sink must be refused");
    assert!(
        text.contains("gas_side") && text.contains("liquid_side"),
        "the refusal must name BOTH offending nodes, since either could be the \
         mistake; got: {text}"
    );
}

/// The same two nodes, NOT connected — two single-phase sub-plants in one file,
/// which DESIGN §3a explicitly allows and which the guard must not refuse.
///
/// This is the other half of the guard: a check that refuses everything is not a
/// guard, it is a ban, and it would make a gas relief system unbuildable
/// alongside the liquid plant it protects.
#[test]
fn two_single_phase_sub_plants_in_one_file_are_accepted() {
    build(&two_sub_plant_scenario()).expect("disjoint gas and liquid sub-plants must build");
}

/// One node whose own composition mixes phases — refused at the node that
/// declares it, before any topology is considered.
#[test]
fn a_single_composition_mixing_phases_is_refused() {
    let src = format!(
        r#"{PREAMBLE}{MIXED_SLATE}
[nodes.wet_gas]
type = "source"
pressure_bar = 10.0
temperature_c = 20.0
composition = {{ fuel_gas = 0.5, condensate = 0.5 }}

[nodes.out]
type = "sink"
pressure_bar = 1.0
composition = {{ fuel_gas = 1.0 }}

[[pipes]]
name = "line"
from = "wet_gas"
to = "out"
length_m = 10.0
diameter_m = 0.1
"#
    );
    let text = expect_refusal(&src, "a two-phase composition must be refused");
    assert!(
        text.contains("wet_gas"),
        "the refusal must name the node that declares it; got: {text}"
    );
}

/// A tank of gas is refused: its inventory `ρ·A·h` and its head `ρgh` are
/// liquid-level quantities that name nothing for a gas, and both read a stored
/// density a gas component does not have. A gas holdup is M5.3's capacitive
/// vessel, whose state is pressure.
#[test]
fn a_tank_holding_gas_is_refused() {
    let src = format!(
        r#"{PREAMBLE}{MIXED_SLATE}
[nodes.gas_tank]
type = "tank"
area_m2 = 10.0
height_m = 5.0
initial_level_m = 2.0
temperature_c = 20.0
composition = {{ fuel_gas = 1.0 }}

[nodes.out]
type = "sink"
pressure_bar = 1.0
composition = {{ fuel_gas = 1.0 }}

[[pipes]]
name = "line"
from = "gas_tank"
to = "out"
length_m = 10.0
diameter_m = 0.1
"#
    );
    let text = expect_refusal(&src, "a gas-filled tank must be refused");
    assert!(
        text.contains("gas_tank"),
        "the refusal must name the tank; got: {text}"
    );
}

// ---------------------------------------------------------------------------
// C — the phase ↔ density correspondence on the component definition.
// ---------------------------------------------------------------------------

fn slate_only_scenario(components: &str) -> String {
    format!(
        r#"{PREAMBLE}{components}
[nodes.src]
type = "source"
pressure_bar = 2.0
temperature_c = 20.0
composition = {{ probe = 1.0 }}

[nodes.snk]
type = "sink"
pressure_bar = 1.0
composition = {{ probe = 1.0 }}

[[pipes]]
name = "line"
from = "src"
to = "snk"
length_m = 10.0
diameter_m = 0.1
"#
    )
}

/// A gas component declaring a liquid density is refused, rather than having the
/// field quietly ignored.
///
/// The field would be read by nothing — a gas density is `P·M̄/(R·T)` — so
/// ignoring it costs no arithmetic. It is refused because an authoritative-looking
/// number that no code reads is how a scenario author comes to believe the model
/// uses something it does not; the same argument that keeps a "gas Cv" out of
/// M5.4 and a `cat_oil_ratio` out of M4.2.
#[test]
fn a_gas_component_declaring_a_density_is_refused() {
    let text = expect_refusal(
        &slate_only_scenario(
            r#"
[[components]]
name = "probe"
phase = "gas"
tb_c = -100.0
molar_mass_kg_per_mol = 0.028
density_kg_per_m3 = 1.2
cp_j_per_kg_k = 1040.0
"#,
        ),
        "a gas component with a declared density must be refused",
    );
    assert!(
        text.contains("probe") && text.contains("density"),
        "the refusal must name the component and the field; got: {text}"
    );
}

/// A liquid component with no density is refused: unlike a gas it has no
/// equation of state to fall back on, so there would be no density law at all.
#[test]
fn a_liquid_component_without_a_density_is_refused() {
    let text = expect_refusal(
        &slate_only_scenario(
            r#"
[[components]]
name = "probe"
tb_c = 100.0
molar_mass_kg_per_mol = 0.018
cp_j_per_kg_k = 4184.0
"#,
        ),
        "a liquid component with no density must be refused",
    );
    assert!(
        text.contains("probe"),
        "the refusal must name the component; got: {text}"
    );
}

/// An unrecognized phase is an error listing the valid options, not a silent
/// fallback to liquid — `phase = "vapour"` must not load as a liquid.
#[test]
fn an_unknown_phase_is_refused() {
    let text = expect_refusal(
        &slate_only_scenario(
            r#"
[[components]]
name = "probe"
phase = "vapour"
tb_c = -100.0
molar_mass_kg_per_mol = 0.028
cp_j_per_kg_k = 1040.0
"#,
        ),
        "an unknown phase must be refused",
    );
    assert!(
        text.contains("vapour") && text.contains("liquid") && text.contains("gas"),
        "the refusal must name the bad value and the valid ones; got: {text}"
    );
}

// ---------------------------------------------------------------------------
// D — the pipe's initial composition seed, which has no analogue before M5.2.
// ---------------------------------------------------------------------------

/// A liquid sub-plant and a gas sub-plant in one file, sharing a slate whose
/// FIRST component is the liquid one. The gas sub-plant reproduces `gas_line`'s
/// geometry so the same hand calc applies.
fn two_sub_plant_scenario() -> String {
    format!(
        r#"{PREAMBLE}{MIXED_SLATE}
[nodes.water_source]
type = "source"
pressure_bar = 2.0
temperature_c = 20.0
composition = {{ condensate = 1.0 }}

[nodes.water_sink]
type = "sink"
pressure_bar = 1.0
composition = {{ condensate = 1.0 }}

[nodes.gas_header]
type = "source"
pressure_bar = 10.0
temperature_c = 20.0
composition = {{ fuel_gas = 1.0 }}

[nodes.tee]
type = "junction"

[nodes.flare]
type = "sink"
pressure_bar = 1.0
composition = {{ fuel_gas = 1.0 }}

[[pipes]]
name = "water_line"
from = "water_source"
to = "water_sink"
length_m = 100.0
diameter_m = 0.08

[[pipes]]
name = "header_run"
from = "gas_header"
to = "tee"
length_m = 100.0
diameter_m = 0.08

[[pipes]]
name = "relief_line"
from = "tee"
to = "flare"
length_m = 100.0
diameter_m = 0.05
"#
    )
}

/// A gas line on a slate whose first component is a LIQUID still carries gas.
///
/// The pipe's stored composition is seeded at load, before any transport has
/// run. Its only reader that reaches the SOLVE is `compile_edge`'s transport
/// density (`EdgeSnapshot` also publishes it, so it is frontend-visible for one
/// tick, but that cannot move a number). Seeding every pipe with component 0 —
/// which is what the code did from M3.1 until this slice — gives a gas line the
/// density of water on its first solve: not stale, wrong by a factor of ~150,
/// and silent, because a plant of two reservoirs and a pipe converges happily
/// on it.
///
/// This is the gate for `seed_component_index`, and the only one that can fail
/// on it: every other plant in the repo has an all-liquid slate, where the first
/// liquid component IS index 0 and the fix is invisible by construction.
#[test]
fn a_gas_line_on_a_mixed_slate_is_seeded_with_gas_not_component_zero() {
    let engine = build(&two_sub_plant_scenario()).expect("two sub-plants build");
    let sol = NewtonFlowSolver::default()
        .solve(&engine.graph, &engine.slate, Seconds(0.1))
        .expect("converges");

    let (_, expected_flow) = upwind_prediction(P_HEADER_PA);
    approx::assert_relative_eq!(
        sol.edge_mass_flow[&edge_by_name(&engine.graph, "header_run")],
        expected_flow,
        max_relative = HAND_CALC_TOLERANCE
    );
}

// ---------------------------------------------------------------------------
// E — WHICH END of the edge the transport temperature comes from.
// ---------------------------------------------------------------------------

/// Two gas reservoirs and one pipe: every boundary of this plant is pinned, so
/// its flow is a constant of the plant and not a function of tick count.
///
/// Deliberately the topology the M5.2 note called useless — `n == 0`, the Newton
/// loop never runs. That is exactly what makes it the right fixture here: with no
/// free node there is no iterate, no convergence slack and no thermal state
/// anywhere, so the ONLY thing that can move the flow between two ticks is which
/// temperature `compile_edge` evaluated `ρ = P·M̄/(R·T)` at.
fn two_reservoir_gas_scenario() -> String {
    format!(
        r#"{PREAMBLE}{MIXED_SLATE}
[nodes.gas_header]
type = "source"
pressure_bar = 10.0
temperature_c = 20.0
composition = {{ fuel_gas = 1.0 }}

[nodes.flare]
type = "sink"
pressure_bar = 1.0
composition = {{ fuel_gas = 1.0 }}

[[pipes]]
name = "header_run"
from = "gas_header"
to = "flare"
length_m = 100.0
diameter_m = 0.08
"#
    )
}

/// A gas edge compiles its density at its upwind NODE's temperature, not at its
/// own stored outlet.
///
/// `pipe.stream.temperature` is the edge's OUTLET — the inlet plus its ambient
/// transform plus its own frictional dissipation. Reading it for the transport
/// density asks what the gas looks like *after* it has crossed the pipe, which is
/// the wrong end, and in gas service it is not a small wrong end: expanding an
/// ideal gas across a branch dissipates `Δp/ρ` per kilogram, so `ΔT/T = (γ−1)/γ`
/// — here a 61 K rise on a 293 K feed, worth ~17% of the density and ~9% of the
/// flow.
///
/// What makes this a defect rather than an accepted lag, and the reason it is
/// fixed ahead of M5.3 rather than inside it: the offset contains no `dt`. `Φ`
/// and `ṁ` are both instantaneous, so `Φ/(ṁ·cp)` is the same at any step size. It
/// is a different steady model, not a truncation — no tolerance can be derived
/// around it and no order-of-convergence gate would diagnose it, because it does
/// not converge to zero. M5.3's blowdown rate gate rests on the identity
/// `ρ_edge = m_vessel/V`, which holds exactly only once the edge's temperature is
/// the vessel's own.
///
/// The gate is behavioural rather than a readback of the density: this plant has
/// no state, so **tick 1 and tick 5 must produce the identical flow**, bit for
/// bit. The stored-outlet reader fails it on tick 2.
#[test]
fn a_gas_edges_density_follows_its_upwind_node_not_its_own_outlet() {
    let mut engine = build(&two_reservoir_gas_scenario()).expect("two-reservoir gas plant builds");
    let line = edge_by_name(&engine.graph, "header_run");

    // ṁ² = ρ_source·ΔP·K, the same hand calc as every gate above, at the source's
    // DECLARED temperature — which is the claim under test.
    let expected =
        (density_per_pascal() * P_HEADER_PA * (P_HEADER_PA - P_FLARE_PA) * header_group()).sqrt();

    let mut flows = Vec::new();
    for _ in 0..5 {
        engine
            .tick()
            .expect("a plant of two reservoirs and a pipe ticks");
        flows.push(engine.graph.pipe(line).stream.mass_flow.value());
    }

    // Vacuity guard: the two candidate temperatures must actually be far apart,
    // or a passing run would mean nothing. The stored outlet is the source
    // temperature plus this edge's own dissipation.
    let stored = engine.graph.pipe(line).stream.temperature.value();
    assert!(
        stored - T_SEED_K > 40.0,
        "this gate is vacuous unless the pipe's stored outlet is far from its \
         upwind node: outlet {stored:.2} K vs source {T_SEED_K:.2} K"
    );

    for (tick, flow) in flows.iter().enumerate() {
        approx::assert_relative_eq!(*flow, expected, max_relative = HAND_CALC_TOLERANCE);
        // Bit-identical, not merely close: nothing in this plant is inertial, so
        // there is no physical reason for any tick to differ from the first.
        assert_eq!(
            flow.to_bits(),
            flows[0].to_bits(),
            "tick {} flow {flow} differs from tick 1's {}: the only state that \
             changed between them is the pipe's stored outlet temperature, which \
             the transport density must not be reading",
            tick + 1,
            flows[0]
        );
    }
}

/// The liquid sub-plant in the same file is untouched by its gas neighbour —
/// the seed is per connected component, not per file.
#[test]
fn the_liquid_sub_plant_beside_a_gas_one_still_carries_liquid() {
    let engine = build(&two_sub_plant_scenario()).expect("two sub-plants build");
    let sol = NewtonFlowSolver::default()
        .solve(&engine.graph, &engine.slate, Seconds(0.1))
        .expect("converges");

    // ṁ² = ρ·ΔP·K with the DECLARED liquid density, no equation of state.
    let expected = (998.0 * 1.0e5 * geometry_group(100.0, 0.08)).sqrt();
    approx::assert_relative_eq!(
        sol.edge_mass_flow[&edge_by_name(&engine.graph, "water_line")],
        expected,
        max_relative = HAND_CALC_TOLERANCE
    );
}
