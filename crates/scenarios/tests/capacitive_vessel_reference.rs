//! The capacitive gas vessel (docs/ROADMAP.md §M5.3, docs/DESIGN.md §3a forks
//! 2 & 3) — the accumulation term in the shared residual, capacitance as an
//! anchor, and the internal-energy datum that makes blowdown cooling emerge.
//!
//! **Three gates, because no one of them covers it**, and they fail under
//! different mutations by construction:
//!
//! - **The design decision** (§A). The 1 m³ drum fork 2's table is built on, run
//!   to a converged bounded state, with `dt·g/C` MEASURED at the plant's own
//!   fixed point rather than read off the note. This is the gate that falsifies
//!   the verdict — without it, "the explicit scheme would have diverged" is a
//!   claim about a scheme nobody ran.
//! - **The thermodynamic first integral** (§B). `T/Tᵢ = (m/mᵢ)^(γ−1)` on an
//!   ordinary blowdown. Path-independent — no `t`, no resistance, no downstream
//!   pressure — so it pins the `cv` balance and the gas law WITHOUT pinning the
//!   time integration, which is exactly what the third gate then does.
//! - **The time coupling** (§C). The first integral has no `t` in it, so
//!   something must pin the capacitance–orifice *rate*. A zero-pressure sink
//!   makes the ODE elementary and closed-form, and an order-of-convergence check
//!   backs the tolerance — the only gate that catches a degraded scheme that
//!   still converges (the M4.2 RK4 lesson).
//!
//! **Ceiling of §C, named rather than glossed** in the manner of M4.2's
//! envelope. A vacuum sink drives the incompressible orifice law far outside
//! where it is physical: a real gas expanding to zero pressure chokes, and this
//! branch law does not know that. §C therefore pins the NUMERICS of the
//! capacitance/energy coupling and says nothing about the fidelity of the flow
//! law. The flow law's physical anchor is M5.4's IEC 60534-2-1 gate.

use refinery_core::engine::Engine;
use refinery_core::graph::{EdgeId, NodeId, NodeKind, PlantGraph};
use refinery_solvers::network;
use std::collections::BTreeMap;

const DRUM_PLANT: &str = include_str!("../../../scenarios/knockout_drum.toml");

// --- Constants restated from the files, never read back from the object under
// --- test: a gate that fetches its expected inputs from the code it checks can
// --- only ever verify self-consistency.

/// CODATA universal gas constant [J/(mol·K)], exact since the 2019 SI.
const R: f64 = 8.314_462_618_153_24;

/// `knockout_drum.toml`'s gas and geometry, as the file declares them.
const DRUM_M_BAR: f64 = 0.030;
const DRUM_VOLUME_M3: f64 = 1.0;
const DRUM_T_K: f64 = 300.0;
const DRUM_DT_S: f64 = 0.1;

/// The blowdown fixtures' gas: methane, matching `gas_line.toml`'s cut.
const BLOWDOWN_M_BAR: f64 = 0.016_043;
const BLOWDOWN_CP: f64 = 2220.0;
const BLOWDOWN_V_M3: f64 = 1.0;
const BLOWDOWN_T0_K: f64 = 293.15;
const BLOWDOWN_P0_PA: f64 = 10.0e5;
const BLOWDOWN_LENGTH_M: f64 = 50.0;
const BLOWDOWN_DIAMETER_M: f64 = 0.01;
const FRICTION: f64 = 0.02;

/// `cv = cp − R/M̄` and `γ = cp/cv` for the blowdown gas — computed here from
/// the DECLARED `cp` and `M̄`, so the gates below do not read the workspace's own
/// `mixture_cv` back into their expectations.
fn blowdown_cv() -> f64 {
    BLOWDOWN_CP - R / BLOWDOWN_M_BAR
}
fn blowdown_gamma() -> f64 {
    BLOWDOWN_CP / blowdown_cv()
}

/// `2·D·A²/(f·L)`, the geometry group in `ṁ² = ρ·ΔP·K` (derived in
/// `gas_density_reference.rs`).
fn geometry_group(length_m: f64, diameter_m: f64) -> f64 {
    let area = std::f64::consts::PI * diameter_m * diameter_m / 4.0;
    2.0 * diameter_m * area * area / (FRICTION * length_m)
}

fn blowdown_group() -> f64 {
    geometry_group(BLOWDOWN_LENGTH_M, BLOWDOWN_DIAMETER_M)
}

/// Initial inventory `mᵢ = P·V·M̄/(R·T)` of the blowdown fixtures [kg].
fn blowdown_initial_mass() -> f64 {
    BLOWDOWN_P0_PA * BLOWDOWN_V_M3 * BLOWDOWN_M_BAR / (R * BLOWDOWN_T0_K)
}

// ---------------------------------------------------------------------------
// Fixtures.
// ---------------------------------------------------------------------------

/// A vessel discharging through one pipe into a sink at `sink_bar`.
///
/// Two variants, one string: `sink_bar = 1.01325` is §B's ordinary blowdown to
/// atmosphere, `0.0` is §C's vacuum sink. Nothing else differs, which is what
/// lets §B and §C disagree only about what they pin.
fn blowdown_scenario(sink_bar: f64, dt: f64) -> String {
    format!(
        r#"
[meta]
name = "blowdown"

[simulation]
dt = {dt}

[fidelity]
flow = "newton"

[[components]]
name = "fuel_gas"
phase = "gas"
tb_c = -161.5
molar_mass_kg_per_mol = {BLOWDOWN_M_BAR}
cp_j_per_kg_k = {BLOWDOWN_CP}

[nodes.drum]
type = "vessel"
volume_m3 = {BLOWDOWN_V_M3}
pressure_bar = 10.0
temperature_c = 20.0

[nodes.outlet]
type = "sink"
pressure_bar = {sink_bar}
temperature_c = 20.0

[[pipes]]
name = "blowdown_line"
from = "drum"
to = "outlet"
length_m = {BLOWDOWN_LENGTH_M}
diameter_m = {BLOWDOWN_DIAMETER_M}
"#
    )
}

fn build(toml_src: &str) -> Result<Engine, refinery_core::error::SimError> {
    refinery_scenarios::build_engine(&refinery_scenarios::load_str(toml_src)?)
}

fn expect_refusal(toml_src: &str, what: &str) -> String {
    match build(toml_src) {
        Ok(_) => panic!("{what}"),
        Err(e) => e.to_string(),
    }
}

fn node_by_name(graph: &PlantGraph, name: &str) -> NodeId {
    graph
        .find_node(name)
        .unwrap_or_else(|| panic!("plant must have a '{name}' node"))
}

fn edge_by_name(graph: &PlantGraph, name: &str) -> EdgeId {
    graph
        .edge_ids()
        .find(|e| graph.pipe(*e).name == name)
        .unwrap_or_else(|| panic!("plant must have a '{name}' pipe"))
}

/// `(mass [kg], temperature [K])` of a vessel node.
fn vessel_state(engine: &Engine, name: &str) -> (f64, f64) {
    match &engine.graph.node(node_by_name(&engine.graph, name)).kind {
        NodeKind::Vessel(v) => (v.mass.value(), v.temperature.value()),
        other => panic!("'{name}' must be a vessel, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// A — the design-decision gate: the case the rejected explicit scheme diverges on.
// ---------------------------------------------------------------------------

/// The 1 m³ drum of fork 2's table converges to a bounded steady state, and the
/// explicit alternative provably would NOT have.
///
/// This is the gate the roadmap calls the one most easily left out, and it is
/// the only one that can falsify fork 2's verdict rather than assume it. It does
/// two things a convergence check alone would not:
///
/// 1. It measures `g = Σ_e ρ·dQ/d(dp)`, the total conductance of the drum's
///    branches, at the plant's OWN converged state — not from the note's
///    `g ≈ ṁ/2ΔP` estimate — and confirms `dt·g/C` is outside the explicit
///    scheme's stability bound of 2. The rejected scheme's amplification factor
///    is `|1 − dt·g/C|`, and it is > 1 here, so a `tank`-with-a-gas-pressure-law
///    drum would oscillate with GROWING amplitude on this ordinary plant.
/// 2. It confirms the semi-implicit scheme lands on the balance point anyway,
///    with in-flow and out-flow equal — i.e. the term that buys the stability
///    did not buy it by damping the physics away.
#[test]
fn the_knockout_drum_converges_where_the_explicit_scheme_would_diverge() {
    let mut engine = build(DRUM_PLANT).expect("knockout_drum.toml builds");
    for _ in 0..300 {
        engine.tick().expect("the drum plant ticks");
    }

    let drum = node_by_name(&engine.graph, "drum");
    let inlet = edge_by_name(&engine.graph, "inlet_run");
    let outlet = edge_by_name(&engine.graph, "outlet_run");
    let (mass, temperature) = vessel_state(&engine, "drum");

    // Converged: the drum is at its balance point, so the two runs carry one flow.
    let flow_in = engine.graph.pipe(inlet).stream.mass_flow.value();
    let flow_out = engine.graph.pipe(outlet).stream.mass_flow.value();
    approx::assert_relative_eq!(flow_in, flow_out, max_relative = 1e-6);
    assert!(
        flow_in > 0.0 && flow_in.is_finite(),
        "the drum must pass gas header → downstream, got {flow_in} kg/s"
    );

    // Bounded, and bounded BY the plant: the drum's pressure sits between the
    // two reservoirs it is wired to. An unstable scheme's first excursion leaves
    // this interval, and nothing else in the engine would refuse it.
    let capacitance = DRUM_VOLUME_M3 * DRUM_M_BAR / (R * temperature);
    let pressure = mass / capacitance;
    assert!(
        (9.8e5..=10.2e5).contains(&pressure),
        "the drum settled at {pressure:.1} Pa, outside the 9.8–10.2 bar its own \
         reservoirs bracket"
    );

    // --- The verdict, measured on this plant --------------------------------
    // C at the table's stated state, from the file's own numbers.
    let c_table = DRUM_VOLUME_M3 * DRUM_M_BAR / (R * DRUM_T_K);
    approx::assert_relative_eq!(c_table, 1.203e-5, max_relative = 1e-3);

    // g = Σ_e ρ_e · dQ_e/d(dp) over the drum's branches, at the converged state.
    let prepared = network::prepare(&engine.graph, &engine.slate, &BTreeMap::new())
        .expect("the converged plant prepares");
    let mut g = 0.0;
    for (eid, _other, _incoming) in engine.graph.incident(drum) {
        let compiled = &prepared.compiled[&eid];
        let dp = prepared.pressures[&compiled.src] - prepared.pressures[&compiled.tgt];
        g += compiled.rho * compiled.branch.flow_ddp(dp, 1.0);
    }

    // Measured: 8.35. Fork 2's table says 4.2 for this drum, and the factor of
    // two is not a discrepancy — the table costs a single 20 kg/s branch, and
    // this plant wires the drum to two of them (`g` is the SUM over its
    // branches). The verdict is if anything stronger on the plant than in the
    // note that predicted it.
    let ratio = DRUM_DT_S * g / capacitance;
    assert!(
        ratio > 2.0,
        "this gate is vacuous unless the plant is genuinely outside the explicit \
         scheme's stability bound: measured dt·g/C = {ratio:.3}, needs > 2. If a \
         change to the plant put it back inside, the milestone's central design \
         decision has stopped being tested."
    );
    // The rejected scheme's own recurrence, δmⁿ⁺¹ = δmⁿ·(1 − dt·g/C), run out
    // rather than asserted about: an amplification factor above 1 means the
    // error GROWS every tick, which is divergence and not merely inaccuracy.
    let amplification = (1.0 - ratio).abs();
    assert!(
        amplification > 1.0,
        "the explicit scheme must actually amplify here, got |1 − dt·g/C| = \
         {amplification:.3}"
    );
    let mut deviation: f64 = 1.0;
    for _ in 0..20 {
        deviation *= 1.0 - ratio;
    }
    assert!(
        deviation.abs() > 100.0,
        "20 ticks of the rejected recurrence must blow a unit deviation up, got \
         {deviation:.3e}"
    );
}

/// Measurement, not a gate: the numbers §A's assertions are sized from.
/// `cargo test -p refinery-scenarios --test capacitive_vessel_reference -- \
///  --ignored --nocapture`
#[test]
#[ignore = "measurement, not a gate"]
fn measure_drum_stability_ratio() {
    let mut engine = build(DRUM_PLANT).expect("builds");
    for _ in 0..300 {
        engine.tick().expect("ticks");
    }
    let drum = node_by_name(&engine.graph, "drum");
    let (mass, temperature) = vessel_state(&engine, "drum");
    let capacitance = DRUM_VOLUME_M3 * DRUM_M_BAR / (R * temperature);
    let prepared =
        network::prepare(&engine.graph, &engine.slate, &BTreeMap::new()).expect("prepares");
    let mut g = 0.0;
    for (eid, _o, _i) in engine.graph.incident(drum) {
        let c = &prepared.compiled[&eid];
        let dp = prepared.pressures[&c.src] - prepared.pressures[&c.tgt];
        g += c.rho * c.branch.flow_ddp(dp, 1.0);
    }
    println!(
        "drum: P = {:.1} Pa, m = {mass:.5} kg, T = {temperature:.3} K, C = {capacitance:.4e} kg/Pa, \
         g = {g:.4e} kg/(s·Pa), dt·g/C = {:.4}",
        mass / capacitance,
        DRUM_DT_S * g / capacitance
    );
}

// ---------------------------------------------------------------------------
// B — the thermodynamic first integral: blowdown cooling, path-independent.
// ---------------------------------------------------------------------------

/// `T/Tᵢ = (m/mᵢ)^(γ−1)` on an ordinary blowdown to atmosphere.
///
/// Nothing in the engine computes a temperature drop. The vessel loses enthalpy
/// through its nozzle (`ṁ·cp·(T − T_REF)`) while holding internal energy
/// (`m·u`), and the difference between the two — the flow work the gas does
/// pushing itself out — is the cooling. That it lands on this exponent is the
/// evidence the `cv` rule and the ideal-gas law are both right and consistent.
///
/// **Path-independent, deliberately.** The relation contains no `t`, no pipe
/// resistance and no downstream pressure, so it holds at every point of the run
/// regardless of how fast the vessel emptied. It therefore pins the energy
/// balance WITHOUT pinning the time integration, and cannot be satisfied by a
/// scheme that gets the rate wrong — which is what §C is for.
///
/// **The two mutations it exists to catch**, and neither is exotic:
/// - `u = cp·(T − T_REF)` (the tank's rule, applied to a gas) removes the flow
///   work entirely and the vessel blows down ISOTHERMALLY.
/// - `u = cv·(T − T_REF)` — the rule docs/DESIGN.md §3a fork 3 originally stated
///   — is datum-inconsistent with `h = cp·(T − T_REF)` and gives
///   `(T − T_REF)/(Tᵢ − T_REF) = (m/mᵢ)^(γ−1)` instead. On this fixture that is a
///   2.4 K drop where the correct rule gives 35 K, so it is not a tolerance
///   question. See `energy::specific_internal_energy`.
#[test]
fn a_blowing_down_vessel_follows_the_adiabatic_first_integral() {
    let mut engine = build(&blowdown_scenario(1.01325, 0.1)).expect("blowdown builds");
    let (m_initial, t_initial) = vessel_state(&engine, "drum");
    approx::assert_relative_eq!(m_initial, blowdown_initial_mass(), max_relative = 1e-12);

    let exponent = blowdown_gamma() - 1.0;
    let mut sampled = 0;
    let mut deepest = 1.0f64;
    for tick in 1..=3000 {
        engine.tick().expect("the blowdown ticks");
        if tick % 250 != 0 {
            continue;
        }
        let (mass, temperature) = vessel_state(&engine, "drum");
        let ratio = mass / m_initial;
        deepest = deepest.min(ratio);
        approx::assert_relative_eq!(
            temperature / t_initial,
            ratio.powf(exponent),
            max_relative = FIRST_INTEGRAL_TOLERANCE
        );
        sampled += 1;
    }

    assert!(sampled >= 10, "the run must sample the integral repeatedly");
    // Vacuity guards. The relation is trivially satisfied at m = mᵢ, so the run
    // must actually empty the vessel a long way AND the cooling must be large
    // enough that the datum-inconsistent alternative is nowhere near it.
    assert!(
        deepest < 0.7,
        "the vessel must blow down far enough for the exponent to bite, reached \
         m/mᵢ = {deepest:.4}"
    );
    let correct_drop = t_initial * (1.0 - deepest.powf(exponent));
    let datum_slipped_drop = (t_initial - 273.15) * (1.0 - deepest.powf(exponent));
    assert!(
        correct_drop > 10.0 * datum_slipped_drop,
        "the datum-inconsistent rule must be distinguishable: correct drop \
         {correct_drop:.2} K vs slipped {datum_slipped_drop:.2} K"
    );
}

/// Tolerance for §B [relative].
///
/// **Derived, not tuned.** The first integral is exact for the continuous
/// system; what separates the simulation from it is the tick, and only through
/// two O(dt) lags — the temperature is integrated explicitly while the mass is
/// implicit, and the vessel's `T` (hence `C` and the outflow density) is the
/// START-of-tick value. Both are first order, so the deviation scales with the
/// per-tick fractional mass change, `ṁ·dt/m ≈ 4.3e-4` at the start of this run
/// and smaller after.
///
/// Measured by `measure_first_integral_deviation`, and first order as predicted:
/// worst deviation 1.48e-4 at `dt = 0.2`, **7.40e-5 at the `dt = 0.1` this gate
/// runs at**, 3.70e-5 at 0.05 — halving the step halves it. 1e-3 clears the
/// worst by 13.5x.
///
/// **Do not loosen past ~1e-2**, where it would stop discriminating: the
/// isothermal slip (`u = cp·(T − T_REF)`) and the datum slip (`u = cv·(T −
/// T_REF)`) are both O(1) errors here, but a `γ` off by a few percent — a `cv`
/// built from the wrong molar mass, say — is not.
const FIRST_INTEGRAL_TOLERANCE: f64 = 1e-3;

#[test]
#[ignore = "measurement, not a gate"]
fn measure_first_integral_deviation() {
    for dt in [0.2f64, 0.1, 0.05] {
        let mut engine = build(&blowdown_scenario(1.01325, dt)).expect("builds");
        let (m_initial, t_initial) = vessel_state(&engine, "drum");
        let exponent = blowdown_gamma() - 1.0;
        let ticks = (300.0 / dt) as usize;
        let mut worst = 0.0f64;
        for _ in 0..ticks {
            engine.tick().expect("ticks");
            let (mass, temperature) = vessel_state(&engine, "drum");
            let predicted = (mass / m_initial).powf(exponent) * t_initial;
            worst = worst.max((temperature - predicted).abs() / predicted);
        }
        let (mass, temperature) = vessel_state(&engine, "drum");
        println!(
            "dt = {dt:<5}: worst relative deviation {worst:.4e}, final m/mᵢ = {:.5}, T = {temperature:.3} K",
            mass / m_initial
        );
    }
}

// ---------------------------------------------------------------------------
// C — the time coupling: the capacitance–orifice RATE, in closed form.
// ---------------------------------------------------------------------------

/// The exact solution of `dm/dt = −A₀·m^((1+γ)/2)` for a vessel discharging to a
/// zero-pressure sink [kg].
///
/// With `ṁ = √(K·ρ·P)`, `ρ = m/V` and the adiabatic `P = Pᵢ·(m/mᵢ)^γ`, the flow
/// collapses to a pure power of the inventory, `ṁ = A₀·m^((1+γ)/2)` with
/// `A₀ = √(K·Pᵢ/(V·mᵢ^γ))`, and the ODE separates:
///
/// ```text
/// m/mᵢ = (1 + ½(γ−1)·A₀·mᵢ^((γ−1)/2)·t)^(−2/(γ−1))
/// ```
///
/// The sink is at ZERO so `ΔP = P` with no second term — that is the whole
/// reason for a fixture no real plant has, and the ceiling this file's header
/// states.
fn closed_form_mass(t: f64) -> f64 {
    let gamma = blowdown_gamma();
    let m_initial = blowdown_initial_mass();
    let a0 = (blowdown_group() * BLOWDOWN_P0_PA / (BLOWDOWN_V_M3 * m_initial.powf(gamma))).sqrt();
    let k = 0.5 * (gamma - 1.0) * a0 * m_initial.powf(0.5 * (gamma - 1.0));
    m_initial * (1.0 + k * t).powf(-2.0 / (gamma - 1.0))
}

/// The blowdown's inventory tracks the closed form in TIME, not merely along the
/// path §B pins.
///
/// §B has no `t` in it: a scheme that integrated at half speed, or that used the
/// wrong density on the discharge edge, would satisfy it exactly while being
/// wrong about when anything happens. This gate is what stops that, and it is
/// the milestone's only rate gate.
///
/// **What makes the closed form an anchor rather than a readback**: the identity
/// `ρ_edge = m/V`. The discharge edge compiles its density at the vessel's own
/// pressure and temperature, and the capacitance is stated at the same `T`, so
/// `m_new = C·P_solved` gives `ρ_edge = C·P/V = m_new/V` EXACTLY — the same `ρ`
/// the derivation above substitutes. That identity holds only because a gas
/// edge takes its temperature from the upwind NODE; with the pipe's own outlet
/// temperature it would be off by `(γ−1)/γ` and this gate would fail by ~20%
/// at every step size.
///
/// **Where the tolerance comes from, and where it does not.** The scheme is
/// implicit Euler in mass and explicit in temperature, both O(dt), so the error
/// is first order and `ORDER_OF_CONVERGENCE` below is what actually pins it.
/// The `eps_dp = 1.0` Pa regularisation is the other error term and it grows as
/// the vessel empties, so it is budgeted at the END of the run rather than the
/// start: the run stops at ~5.8 bar, where `eps/(2P) ≈ 9e-7` — three orders
/// below the truncation, so the convergence gate measures the scheme and not the
/// regularisation.
#[test]
fn the_blowdown_rate_matches_the_closed_form() {
    let dt = 0.1;
    let mut engine = build(&blowdown_scenario(0.0, dt)).expect("vacuum blowdown builds");
    for tick in 1..=1000 {
        engine.tick().expect("the vacuum blowdown ticks");
        if tick % 200 != 0 {
            continue;
        }
        let (mass, _) = vessel_state(&engine, "drum");
        let expected = closed_form_mass(tick as f64 * dt);
        approx::assert_relative_eq!(mass, expected, max_relative = RATE_TOLERANCE);
    }

    // Vacuity: the run must actually go somewhere, or the closed form is being
    // matched at m ≈ mᵢ where every scheme agrees.
    let (mass, _) = vessel_state(&engine, "drum");
    let drained = 1.0 - mass / blowdown_initial_mass();
    assert!(
        drained > 0.3,
        "the run must drain a substantial fraction for the rate to be pinned, \
         got {drained:.3}"
    );
}

/// Tolerance for §C [relative].
///
/// **Derived from implicit Euler's O(dt) truncation, then measured**, and
/// CONFIRMED to be truncation rather than a fudge by
/// `implicit_euler_converges_at_first_order` — which is the only thing that can
/// tell a derived tolerance from a lucky one. `measure_convergence_order` gives
/// 6.06e-4 at `dt = 0.8` falling to 1.92e-5 at 0.025, i.e. **7.61e-5 at the
/// `dt = 0.1` this gate runs at**. 1e-3 clears it by 13x.
///
/// The other error term is the `eps_dp = 1.0` Pa regularisation, and it is
/// budgeted at the END of the run rather than the start because its relative
/// weight GROWS as the vessel empties: the run stops at ~5.8 bar, where
/// `eps/(2P) ≈ 8.6e-7`. Three orders below the truncation, so the convergence
/// ladder measures the scheme and not the regularisation — which is the failure
/// mode that would otherwise make the order look right for the wrong reason.
const RATE_TOLERANCE: f64 = 1e-3;

/// Halving `dt` halves the error — the gate that separates "implicit Euler" from
/// "something that converges to the right answer more slowly".
///
/// The M4.2 lesson, applied to the other kind of integrator: a closed form alone
/// cannot catch a DEGRADED scheme, because a degraded scheme still lands on the
/// closed form as `dt → 0`. Only the RATE at which it gets there identifies the
/// method. Implicit Euler is first order, so the error ratio between successive
/// halvings must approach 2 — and a scheme that had silently become, say,
/// explicit Euler with a different constant would show the same order but a
/// different coefficient, which is why `the_blowdown_rate_matches_the_closed_form`
/// pins the absolute value too. The two together are what one cannot fake.
///
/// The step sizes are chosen to be in the asymptotic regime, which is measured
/// rather than assumed: `measure_convergence_order` prints the ratios, and they
/// must be near 2 across the whole ladder rather than only at its fine end.
#[test]
fn implicit_euler_converges_at_first_order() {
    let horizon = 100.0;
    let mut errors = Vec::new();
    for dt in [0.4f64, 0.2, 0.1, 0.05] {
        let mut engine = build(&blowdown_scenario(0.0, dt)).expect("builds");
        let ticks = (horizon / dt).round() as usize;
        for _ in 0..ticks {
            engine.tick().expect("ticks");
        }
        let (mass, _) = vessel_state(&engine, "drum");
        let expected = closed_form_mass(horizon);
        errors.push((mass - expected).abs() / expected);
    }

    for window in errors.windows(2) {
        let ratio = window[0] / window[1];
        assert!(
            (1.7..=2.3).contains(&ratio),
            "halving dt must halve a first-order error; got a ratio of \
             {ratio:.4} in the ladder {errors:?}. Below ~1.7 the scheme is not \
             first order (or the coarse end is not yet asymptotic); above ~2.3 \
             it is converging faster than implicit Euler can, which means the \
             error being measured is not truncation."
        );
    }
    // The coarsest error must be big enough that the ladder is measuring
    // truncation and not round-off.
    assert!(
        errors[0] > 1e-4,
        "the coarse end must carry real truncation error, got {:.3e}",
        errors[0]
    );
}

#[test]
#[ignore = "measurement, not a gate"]
fn measure_convergence_order() {
    let horizon = 100.0;
    let mut previous: Option<f64> = None;
    for dt in [0.8f64, 0.4, 0.2, 0.1, 0.05, 0.025] {
        let mut engine = build(&blowdown_scenario(0.0, dt)).expect("builds");
        let ticks = (horizon / dt).round() as usize;
        for _ in 0..ticks {
            engine.tick().expect("ticks");
        }
        let (mass, _) = vessel_state(&engine, "drum");
        let expected = closed_form_mass(horizon);
        let error = (mass - expected).abs() / expected;
        match previous {
            Some(p) => println!(
                "dt = {dt:<6}: rel error {error:.6e}   ratio {:.4}",
                p / error
            ),
            None => println!("dt = {dt:<6}: rel error {error:.6e}"),
        }
        previous = Some(error);
    }
}

// ---------------------------------------------------------------------------
// D — capacitance as an ANCHOR, and the load-time guards.
// ---------------------------------------------------------------------------

/// A closed gas system with NO fixed node anywhere is well posed — the first
/// time that has been true.
///
/// Two vessels joined by a pipe: nothing pins a pressure, so before capacitance
/// this plant was singular twice over. The loader refused it (no pressure-fixing
/// node in the component) and, had it got past that, `anchored_set` would have
/// called both nodes floating and zeroed the pipe. Both had to change, and this
/// is the only gate in the repo that can see either.
///
/// The physics it must produce is unmistakable: gas flows from the full vessel
/// to the empty one and the two pressures meet. Mass is conserved between them
/// exactly, since there is nowhere else for it to go.
#[test]
fn two_vessels_and_no_fixed_node_equalise() {
    let src = format!(
        r#"
[meta]
name = "closed_gas_system"

[simulation]
dt = 0.1

[fidelity]
flow = "newton"

[[components]]
name = "fuel_gas"
phase = "gas"
tb_c = -161.5
molar_mass_kg_per_mol = {BLOWDOWN_M_BAR}
cp_j_per_kg_k = {BLOWDOWN_CP}

[nodes.full]
type = "vessel"
volume_m3 = 1.0
pressure_bar = 10.0
temperature_c = 20.0

[nodes.empty]
type = "vessel"
volume_m3 = 1.0
pressure_bar = 1.0
temperature_c = 20.0

[[pipes]]
name = "equaliser"
from = "full"
to = "empty"
length_m = 10.0
diameter_m = 0.02
"#
    );
    let mut engine = build(&src).expect("a closed gas system must build — capacitance anchors it");
    let (m_full_0, _) = vessel_state(&engine, "full");
    let (m_empty_0, _) = vessel_state(&engine, "empty");

    for _ in 0..5000 {
        engine.tick().expect("the closed system ticks");
    }
    let (m_full, t_full) = vessel_state(&engine, "full");
    let (m_empty, t_empty) = vessel_state(&engine, "empty");

    // Mass is conserved to round-off: the system is closed, so I1's accumulation
    // term is the WHOLE balance here rather than a correction to a throughput.
    approx::assert_relative_eq!(m_full + m_empty, m_full_0 + m_empty_0, max_relative = 1e-12);
    assert!(
        m_full < m_full_0 && m_empty > m_empty_0,
        "gas must move from the full vessel to the empty one"
    );

    // And they equalise. Pressure, not mass: the two vessels end at different
    // temperatures (one expanded and cooled, the other was compressed and
    // heated), so equal pressure at unequal `T` means unequal mass — which is
    // the answer, and would be wrong if the gate compared inventories.
    // P = m·R·T/(V·M̄), both vessels being 1 m³.
    let pressure_of = |m: f64, t: f64| m * R * t / BLOWDOWN_M_BAR;
    let p_full = pressure_of(m_full, t_full);
    let p_empty = pressure_of(m_empty, t_empty);
    approx::assert_relative_eq!(p_full, p_empty, max_relative = 1e-4);
}

/// I2 for a vessel: draining four orders of magnitude to a VACUUM sink stays
/// finite, positive and monotone — it never rings, never goes negative and never
/// divides by an empty inventory.
///
/// The regime every other gate here avoids on purpose. §C stops at ~5.8 bar so
/// its tolerance is truncation rather than regularisation; this one runs past
/// that deliberately, down to where `P` is comparable to `RHO_EVAL_P_FLOOR` and
/// `eps_dp`, because that is where an accumulation term divided by a vanishing
/// capacitance would show up. Nothing is asserted about ACCURACY down there —
/// the flow law is far outside its range and the gas would have liquefied long
/// since (the two-phase deferral) — only that the engine does not produce a
/// number physics forbids, which is rule 5's whole content.
#[test]
fn a_vessel_draining_to_vacuum_stays_finite_and_monotone() {
    let mut engine = build(&blowdown_scenario(0.0, 0.1)).expect("builds");
    let (mut last_mass, mut last_temperature) = vessel_state(&engine, "drum");
    let initial_mass = last_mass;

    for tick in 1..=20_000 {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick} failed: {e}"));
        let (mass, temperature) = vessel_state(&engine, "drum");
        assert!(
            mass.is_finite() && mass > 0.0,
            "tick {tick}: inventory {mass} is not a positive finite mass"
        );
        assert!(
            temperature.is_finite() && temperature > 0.0,
            "tick {tick}: temperature {temperature} K is not above absolute zero"
        );
        assert!(
            mass <= last_mass && temperature <= last_temperature,
            "tick {tick}: a vessel with no inflow must only lose mass and cool, \
             got {mass} kg / {temperature} K after {last_mass} / {last_temperature}"
        );
        last_mass = mass;
        last_temperature = temperature;
    }

    // Vacuity: the run must actually reach the hard regime.
    assert!(
        last_mass < 1e-2 * initial_mass,
        "the drain must cross several orders for this to test anything, reached \
         {last_mass:.3e} of {initial_mass:.3e} kg"
    );
}

/// A vessel of LIQUID is refused, the mirror of `a_tank_holding_gas_is_refused`.
///
/// `C = V·M̄/(R·T)` is the ideal-gas relation; for an incompressible liquid it
/// does not name a smaller number, it names nothing. Left unguarded it would
/// produce a capacitance ~5 orders too small and a plant that oscillates for a
/// reason no diagnostic would explain.
#[test]
fn a_vessel_holding_liquid_is_refused() {
    let src = r#"
[meta]
name = "liquid_vessel"

[simulation]
dt = 0.1

[fidelity]
flow = "newton"

[[components]]
name = "condensate"
tb_c = 100.0
molar_mass_kg_per_mol = 0.018
density_kg_per_m3 = 998.0
cp_j_per_kg_k = 4184.0

[nodes.drum]
type = "vessel"
volume_m3 = 1.0
pressure_bar = 5.0
temperature_c = 20.0

[nodes.out]
type = "sink"
pressure_bar = 1.0

[[pipes]]
name = "line"
from = "drum"
to = "out"
length_m = 10.0
diameter_m = 0.05
"#;
    let text = expect_refusal(src, "a liquid-filled vessel must be refused");
    assert!(
        text.contains("drum") && text.contains("tank"),
        "the refusal must name the vessel and point at the kind that fits; got: {text}"
    );
}

/// A vessel's declared geometry and state must be positive. Each of the three
/// divides into `C = V·M̄/(R·T)` or into `m = C·P`, and a non-positive value in
/// any of them yields a finite, plausible-looking number rather than a failure.
#[test]
fn a_vessel_with_impossible_geometry_or_state_is_refused() {
    for (field, value, needle) in [
        ("volume_m3", "0.0", "volume_m3"),
        ("pressure_bar", "-1.0", "pressure_bar"),
        ("temperature_c", "-300.0", "temperature_c"),
    ] {
        let src = format!(
            r#"
[meta]
name = "bad_vessel"

[simulation]
dt = 0.1

[fidelity]
flow = "newton"

[[components]]
name = "fuel_gas"
phase = "gas"
tb_c = -161.5
molar_mass_kg_per_mol = {BLOWDOWN_M_BAR}
cp_j_per_kg_k = {BLOWDOWN_CP}

[nodes.drum]
type = "vessel"
volume_m3 = {volume}
pressure_bar = {pressure}
temperature_c = {temperature}

[nodes.out]
type = "sink"
pressure_bar = 1.0

[[pipes]]
name = "line"
from = "drum"
to = "out"
length_m = 10.0
diameter_m = 0.05
"#,
            volume = if field == "volume_m3" { value } else { "1.0" },
            pressure = if field == "pressure_bar" {
                value
            } else {
                "5.0"
            },
            temperature = if field == "temperature_c" {
                value
            } else {
                "20.0"
            },
        );
        let text = expect_refusal(&src, &format!("{field} = {value} must be refused"));
        assert!(
            text.contains("drum") && text.contains(needle),
            "the refusal must name the vessel and the field; got: {text}"
        );
    }
}

/// Both fidelities inherit the accumulation term from the same shared residual,
/// so they solve the same fixed point on a capacitive plant (I5) — in BOTH of
/// the regimes that term can be in.
///
/// Not free, and not implied by the pre-M5.3 agreement tests: the term enters
/// Newton through the Jacobian diagonal and the Simple sweep through `g_sum`,
/// which are different pieces of code. They agree because both call
/// `network::accumulation`; a hand-inlined copy in either would drift here.
///
/// **Two fixtures, because `C/dt` and `Σg` trade places between them and the
/// Simple sweep's step is `imbalance/g_sum`** — which regime dominates that
/// denominator decides whether the relaxation is being driven by the vessel or
/// by its branches:
///
/// - the lone blowdown is **capacitance-dominated**: `C/dt ≈ 6.6e-5` against a
///   single branch `g ≈ 1.5e-8`, so the vessel's own term is ~4000x the
///   conductance and effectively sets the step by itself;
/// - `knockout_drum` is **branch-dominated**: `C/dt ≈ 1.2e-4` against
///   `Σg ≈ 1.0e-3`, so the accumulation is ~11% of the denominator and the two
///   pieces have to cooperate.
///
/// Testing only the first would leave the claim standing on the case where the
/// accumulation term drowns everything else out, which is the easy one.
#[test]
fn both_fidelities_agree_on_a_capacitive_plant() {
    let run = |src: &str, ticks: usize| {
        let mut engine = build(src).expect("builds");
        for _ in 0..ticks {
            engine.tick().expect("ticks");
        }
        vessel_state(&engine, "drum")
    };
    let as_simple = |src: &str| src.replace(r#"flow = "newton""#, r#"flow = "simple""#);

    // Capacitance-dominated: the lone vessel blowing down.
    let blowdown = blowdown_scenario(1.01325, 0.1);
    let (m_newton, t_newton) = run(&blowdown, 500);
    let (m_simple, t_simple) = run(&as_simple(&blowdown), 500);
    approx::assert_relative_eq!(m_simple, m_newton, max_relative = 1e-5);
    approx::assert_relative_eq!(t_simple, t_newton, max_relative = 1e-5);
    // Vacuity: the run has to have DONE something for agreement to mean anything.
    assert!(m_newton < 0.95 * blowdown_initial_mass());

    // Branch-dominated: the drum, where the accumulation is a minority of
    // `g_sum` and the branch conductances drive the relaxation.
    let (m_newton, t_newton) = run(DRUM_PLANT, 300);
    let (m_simple, t_simple) = run(&as_simple(DRUM_PLANT), 300);
    approx::assert_relative_eq!(m_simple, m_newton, max_relative = 1e-5);
    approx::assert_relative_eq!(t_simple, t_newton, max_relative = 1e-5);
    // Vacuity: the drum must have moved off its declared 8 bar start, or both
    // fidelities would be agreeing about the initial condition.
    assert!(
        m_newton > 1.2 * 8.0e5 * DRUM_VOLUME_M3 * DRUM_M_BAR / (R * DRUM_T_K),
        "the drum must have filled substantially, got {m_newton:.5} kg"
    );
}

/// `m_new = C·P_solved` — the DAE consistency the closed-form gate rests on,
/// asserted directly instead of only through its consequences.
///
/// The accumulation term the solver drove to zero and the mass update
/// `Engine::tick` performs are two pieces of code in two crates, and they are
/// only the same statement if `C` and `Pⁿ` mean the same thing on both sides.
/// When they do, the vessel's end-of-tick inventory is exactly `C·P*` at the
/// START-of-tick temperature — which is also what makes the discharge edge's
/// `ρ = P·M̄/(R·T)` equal `m/V` and the §C closed form an anchor rather than a
/// readback.
///
/// **§C does NOT cover this, which was worth finding out by mutation rather than
/// assuming either way.** Evaluating `C` at a fixed 293.15 K instead of the
/// vessel's own temperature — a 12% error by the end of §C's run — fails this
/// gate and NOTHING else in the workspace, §C included. The reason is structural
/// rather than lucky: `Pⁿ` is still `m/C_true`, so to leading order in `dt` the
/// trajectory is `dm/dt = −ṁ(m/C_true)` and `C` only scales the *implicit
/// correction*, an O(dt) effect on an O(dt) term. A rate gate cannot see that;
/// only the algebraic identity can. It is also the non-tautological counterpart
/// of `the_vessels_accumulation_closes_the_mass_balance`, which compares the
/// inventory change against the very flows the engine computed it from and so
/// never leaves one crate.
#[test]
fn the_solved_pressure_and_the_integrated_mass_are_the_same_statement() {
    let mut engine = build(&blowdown_scenario(1.01325, 0.1)).expect("builds");
    for tick in 1..=200 {
        // `C` is stated at the START-of-tick temperature, so it has to be read
        // before the tick that consumes it.
        let (_, t_before) = vessel_state(&engine, "drum");
        let capacitance = BLOWDOWN_V_M3 * BLOWDOWN_M_BAR / (R * t_before);

        engine.tick().expect("ticks");

        let snapshot = engine.snapshot();
        let solved = snapshot
            .nodes
            .iter()
            .find(|n| n.name == "drum")
            .expect("the drum is in the snapshot")
            .pressure_pa;
        let (mass, _) = vessel_state(&engine, "drum");

        // The identity is exact in exact arithmetic, so what bounds it here is
        // the SOLVE, not round-off: the residual is only driven below
        // `tol_abs + tol_rel·ṁ`, and whatever is left of it is a mass rate that
        // the tick integrates for `dt`. Hence `|m − C·P| ≤ residual·dt`, which
        // is checked against the solver's OWN reported residual rather than a
        // number chosen to pass — the assertion self-scales with however well
        // the solve actually converged.
        let slack = snapshot.solver.residual * DRUM_DT_S + f64::EPSILON * mass;
        assert!(
            (mass - capacitance * solved).abs() <= 2.0 * slack,
            "tick {tick}: m = {mass} but C·P = {}, a gap of {:.3e} against the \
             {:.3e} the reported residual allows — the accumulation term and the \
             mass update have stopped being the same statement",
            capacitance * solved,
            (mass - capacitance * solved).abs(),
            slack
        );
        // And the derived absolute bound, so the gate still says something if
        // the solver ever reports a residual of zero: tol_abs·dt/m ≈ 1.5e-10.
        // Measured at 1.7e-11.
        approx::assert_relative_eq!(mass, capacitance * solved, max_relative = 1e-9);
        assert!(tick < 200 || mass < blowdown_initial_mass());
    }
}

/// I4 for the first new inertial node kind since `Tank`: two fresh engines run
/// the same capacitive plant to byte-identical snapshots.
///
/// A vessel adds serialized state (`VesselState` inside `NodeKind`) and a new
/// `BTreeMap` in the solve. Neither should be able to break determinism — rule 3
/// is structural, not incidental — but the regression anchor could not cover
/// `knockout_drum.toml`, because the plant did not exist at the baseline commit,
/// so this plant had never been run twice and compared. "Expect" is what rule 3
/// exists to replace.
#[test]
fn a_capacitive_plant_reruns_bit_identically() {
    let run = || {
        let mut engine = build(DRUM_PLANT).expect("builds");
        let mut snapshots = Vec::new();
        for tick in 1..=200 {
            engine.tick().expect("ticks");
            if tick % 20 == 0 {
                snapshots
                    .push(serde_json::to_vec(&engine.snapshot()).expect("snapshot must serialize"));
            }
        }
        snapshots
    };
    let first = run();
    assert_eq!(first.len(), 10, "the gate must compare real snapshots");
    assert_eq!(
        first,
        run(),
        "two fresh engines on the same capacitive plant must produce byte-identical \
         snapshots (rule 3, I4)"
    );
}

/// I1 with accumulation: over a blowdown, everything that left the vessel
/// arrived at the sink, and the vessel's inventory fell by exactly that much.
///
/// The invariant `invariants.rs` states — `Σ inflow = Σ outflow + Δ inventory` —
/// on the first node kind whose accumulation lives INSIDE the hydraulic solve.
/// Unlike a reactor against I7, this is an ordinary accumulation the invariant is
/// for, not a term outside its frame: the residual the solver drove to zero is
/// the same balance this test recomputes from the reported edge flows.
#[test]
fn the_vessels_accumulation_closes_the_mass_balance() {
    let dt = 0.1;
    let mut engine = build(&blowdown_scenario(1.01325, dt)).expect("builds");
    let line = edge_by_name(&engine.graph, "blowdown_line");
    let (m_initial, _) = vessel_state(&engine, "drum");

    let mut discharged = 0.0;
    for _ in 0..1000 {
        engine.tick().expect("ticks");
        discharged += engine.graph.pipe(line).stream.mass_flow.value() * dt;
    }
    let (m_final, _) = vessel_state(&engine, "drum");

    // Exact to the solver's own convergence tolerance, not merely close: the
    // engine's mass update IS `m + Σṁ·dt`, and the solve made `Σṁ` equal the
    // accumulation. Any drift here means the two disagree about the same term.
    approx::assert_relative_eq!(m_initial - m_final, discharged, max_relative = 1e-9);
    assert!(
        discharged > 0.2 * m_initial,
        "the balance must be checked over a real discharge, got {discharged:.4} kg \
         of {m_initial:.4}"
    );
}
