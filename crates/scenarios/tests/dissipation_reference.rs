//! Frictional dissipation into the stream (M5.1), on the reference plant.
//!
//! M2.2 deferred pump work and valve throttling heat with an explicit re-opening
//! condition: *measure the rise where it should be largest, and build it only if
//! a gate on that number can fail for the right reason.* The measurement re-opens
//! it — on `tank_pump_valve` at its reference state the valve dissipates
//! **0.094309 K**, 94× the 1e-3 K tolerance the ambient tests already carry, on
//! the plant that has been in the repo since M1 rather than on a contrived
//! high-head service (docs/DESIGN.md §3a).
//!
//! Two gates, pinning different things, because neither covers the other:
//!
//! 1. **The absolute hand calc.** Every edge's temperature rise against
//!    `α·Q|Q|/(ρ·cp)` computed here from the scenario's own geometry and the
//!    **independently derived 13.753287 kg/s** that `kv_reference.rs` pins. It
//!    catches a wrong constant, a dropped term, and — because the three edges
//!    carry three very different numbers — the fold-at-source convention that
//!    decides *which* edge a device's friction lands on.
//!
//!    Its ceiling, stated in the manner `kv_reference` states its own: the `α`
//!    values here necessarily mirror `elements.rs`'s series algebra, so this
//!    catches wrong constants, sign slips and unit errors, not an error in the
//!    formulation. Gate 2 is what has no such ceiling.
//!
//! 2. **The mechanical-energy closure.** Summed around the path, every branch
//!    obeys `dp = α·Q|Q| + β`, so
//!
//!    ```text
//!    Σ_e α_e·Q|Q|  =  (P_supply − P_receiving) + ρ·g·h0 − ρ·g·Δz
//!    ```
//!
//!    The right-hand side is built from the solved PRESSURE field and the two
//!    reversible `β` terms the scenario file declares (the pump's 40 m shutoff
//!    head, the fill line's 5 m rise). It never touches `α`, so it is derived
//!    independently of the quantity under test — which is exactly what the
//!    absolute gate cannot claim. It is also the executable form of DESIGN §3a's
//!    central argument: **`α` is dissipative and `β` is not**. Book elevation as
//!    heat and the two sides part company by 12%.
//!
//! Deliberately NOT here: I6 with the dissipation budget, which needs the random
//! generator and lives in `solvers/tests/energy_invariants.rs`; and the flat-line
//! trap for heat written to an edge's inlet rather than its outlet, which is
//! `isothermal_plant.rs`'s `supply_tank` assertion.

use refinery_core::engine::Engine;
use refinery_core::graph::NodeKind;
use refinery_core::snapshot::Snapshot;

const SCENARIO: &str = include_str!("../../../scenarios/tank_pump_valve.toml");

/// The reference plant's steady mass flow at its initial levels [kg/s].
///
/// Copied from `kv_reference.rs`'s `HAND_CALC_MASS_FLOW_KG_S`, which derives it
/// analytically from the Kv definition rather than from any workspace formula.
/// Every expected value below is built from THIS number and never from a
/// snapshot readback: reading `ṁ` back would make each gate agree with whatever
/// `α` the code used, since the same `α` produced the flow.
const HAND_CALC_MASS_FLOW_KG_S: f64 = 13.753287;

/// Water's density and heat capacity [kg/m³, J/(kg·K)], mirroring
/// `PseudoComponent::water`. Written out rather than read back from the slate for
/// the reason every reference in this workspace does: sourcing both sides of a
/// comparison from the code proves only self-consistency.
const RHO_WATER: f64 = 998.0;
const CP_WATER: f64 = 4184.0;
/// Standard gravity [m/s²], matching `refinery_core::units::G`.
const G: f64 = 9.806_65;

const PLANT_TEMPERATURE_K: f64 = 293.15;

/// Tolerance on the absolute rises [K].
///
/// **Derived, not tuned.** The solver inverts `dp − β = α·Q|Q|` through the
/// regularized `smooth_signed_sqrt(x, eps)` with `eps_dp = 1.0` Pa, so the flow
/// it converges to satisfies that relation only to within the regularization —
/// which shows up as a few Pa on a 411 kPa total, i.e. `3/(ρ·cp) ≈ 7e-7 K`. The
/// solver's own mass tolerance (1e-8 relative) is four orders below that and does
/// not bind. 5e-6 K leaves ~7× headroom over the regularization floor while
/// sitting four orders below the 0.0958 K the fill line actually moves.
const RISE_TOLERANCE_K: f64 = 5e-6;

/// Tolerance on the mechanical-energy closure [Pa]. The same regularization
/// floor as above, expressed in its native units: DESIGN §3a measures the gap at
/// **3 Pa in 411 052**, so 20 Pa is ~7× headroom and still 5e-5 of the total.
const CLOSURE_TOLERANCE_PA: f64 = 20.0;

fn engine_at_reference_state() -> Engine {
    let file = refinery_scenarios::load_str(SCENARIO).expect("reference scenario must parse");
    let mut engine = refinery_scenarios::build_engine(&file).expect("reference plant must build");
    // ONE tick: the levels are still the file's 8.0 m / 1.0 m, which is the state
    // `kv_reference` derives 13.753287 kg/s for. Running longer would drain the
    // supply tank and move the flow off the number every expectation rests on.
    engine.tick().expect("the reference plant must converge");
    engine
}

fn edge_temperature(snapshot: &Snapshot, name: &str) -> f64 {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("the reference plant must have a '{name}' pipe"))
        .stream
        .temperature
        .value()
}

fn edge_dissipation(snapshot: &Snapshot, name: &str) -> f64 {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("the reference plant must have a '{name}' pipe"))
        .dissipation_w
}

fn node_pressure(snapshot: &Snapshot, name: &str) -> f64 {
    snapshot
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("the reference plant must have a '{name}' node"))
        .pressure_pa
}

/// Darcy–Weisbach `k` such that `dP = k·Q|Q|`, from the pipe's geometry.
/// `k = f·L·ρ/(2·D·A²)`, `A = πD²/4` — the standard form, written out here so the
/// expected values below owe nothing to `elements::pipe_resistance`.
fn pipe_alpha(length_m: f64, diameter_m: f64) -> f64 {
    let area = std::f64::consts::PI * diameter_m * diameter_m / 4.0;
    0.02 * length_m * RHO_WATER / (2.0 * diameter_m * area * area)
}

/// The control valve's `α` [Pa/(m³/s)²] at 50% open, from the **published Kv
/// definition** (IEC 60534-2-1 / ISA-75.01: a Kv valve passes Kv m³/h of water at
/// 1 bar, SG 1) rather than from the loader's conversion — the same
/// anti-circularity move `kv_reference.rs` makes.
fn valve_alpha() -> f64 {
    let cv_si = (50.0 / 3600.0) / 1.0e5f64.sqrt();
    let cv_eff = cv_si * 0.5; // opening = 0.5, linear trim
    1.0 / (cv_eff * cv_eff) // rho_rel = ρ/998 = 1 for water
}

/// The pump's dissipative coefficient: `H(Q) = h0 − a·Q|Q|` gives a pressure
/// drop `ρ·g·a·Q|Q|`, so `α = ρ·g·a`. Note the shutoff head `h0` does NOT appear:
/// it is `β`, shaft work into the fluid, and crediting it as heat is exactly the
/// mutation gate 2 exists to catch.
fn pump_alpha() -> f64 {
    RHO_WATER * G * 800.0
}

/// Volumetric flow at the reference state [m³/s].
fn reference_q() -> f64 {
    HAND_CALC_MASS_FLOW_KG_S / RHO_WATER
}

/// Temperature rise a branch of resistance `alpha` imposes [K]:
/// `ΔT = α·Q|Q| / (ρ·cp)`, which is `Φ/(ṁ·cp)` with `Φ = α·Q|Q|·Q` and
/// `ṁ = ρ·Q` — the `Q` cancels, which is why this needs no separate power step.
fn rise(alpha: f64) -> f64 {
    let q = reference_q();
    alpha * q * q.abs() / (RHO_WATER * CP_WATER)
}

/// Every edge, with the `α` the fold-at-source convention gives it: the device at
/// an edge's SOURCE node folds into that edge, so the pump's droop rides
/// `discharge` and the valve's trim rides `fill_line`.
fn expected_edge_rises() -> Vec<(&'static str, f64)> {
    vec![
        ("suction", pipe_alpha(10.0, 0.15)),
        ("discharge", pipe_alpha(30.0, 0.10) + pump_alpha()),
        ("fill_line", pipe_alpha(20.0, 0.10) + valve_alpha()),
    ]
}

// ---------------------------------------------------------------------------
// Gate 1 — the absolute hand calc.
// ---------------------------------------------------------------------------

/// REFERENCE — each edge's outlet sits its own `α·Q|Q|/(ρ·cp)` above its inlet.
///
/// The three numbers span three orders of magnitude — 9.7e-5 K on the suction
/// line, 2.6e-3 K across the pump, **9.578e-2 K across the valve** — which is
/// what makes this more than one coincidence. A device folded onto the wrong edge
/// swaps two of them and fails loudly; a term dropped from `α` fails exactly the
/// edge that carries it.
///
/// The plant is 293.15 K throughout with no heat anywhere, so an edge's *inlet*
/// is its upwind node's temperature and the rise is read directly against the
/// chain: each edge's outlet is the previous cumulative total plus its own share.
#[test]
fn each_edge_heats_its_stream_by_its_own_frictional_drop() {
    let engine = engine_at_reference_state();
    let snapshot = engine.snapshot();

    let mut cumulative = 0.0;
    for (name, alpha) in expected_edge_rises() {
        cumulative += rise(alpha);
        let expected = PLANT_TEMPERATURE_K + cumulative;
        let actual = edge_temperature(&snapshot, name);
        assert!(
            (actual - expected).abs() < RISE_TOLERANCE_K,
            "'{name}' must leave at {expected} K ({PLANT_TEMPERATURE_K} + {cumulative} K of \
             accumulated friction), got {actual} — a difference of {:.3e} K",
            actual - expected
        );
    }

    // The number DESIGN §3a re-opened the deferral on, isolated: the valve's own
    // share of `fill_line`. Its pipe contributes 0.001472 K of the edge's
    // 0.095781, so the valve is 98.5% of it — this is the 0.094309 K the design
    // note measured, not a number that happens to be near it.
    let valve_only = rise(valve_alpha());
    assert!(
        (valve_only - 0.094_309).abs() < 1e-6,
        "the hand calc must reproduce DESIGN §3a's 0.094309 K valve rise, got {valve_only}"
    );

    // Non-vacuity: if the plant were not flowing, every rise above would be zero
    // and the chain would pass by accident.
    let flow = snapshot
        .edges
        .iter()
        .find(|e| e.name == "fill_line")
        .expect("fill_line exists")
        .stream
        .mass_flow
        .value();
    assert!(
        (flow - HAND_CALC_MASS_FLOW_KG_S).abs() < 1e-3,
        "the expected rises are built on {HAND_CALC_MASS_FLOW_KG_S} kg/s; the plant is \
         carrying {flow} kg/s, so they describe a different plant"
    );
}

/// The solver's reported `Φ` is the same physics in power units: `Φ = ṁ·cp·ΔT`.
///
/// Not a restatement of the gate above. That one reads `stream.temperature`,
/// which is `core`'s TRANSFORM; this reads `EdgeSnapshot::dissipation_w`, which
/// is what `solvers` put on the seam. The two are separate halves of DESIGN §3a's
/// split — the solver owns `α·Q|Q|·Q` because `β` is element physics `core` must
/// not know — and a fault in either alone shows up in exactly one of these.
#[test]
fn the_reported_dissipation_matches_the_rise_it_causes() {
    let engine = engine_at_reference_state();
    let snapshot = engine.snapshot();

    for (name, alpha) in expected_edge_rises() {
        let q = reference_q();
        let expected = alpha * q * q.abs() * q; // Φ = α·Q|Q|·Q [W]
        let actual = edge_dissipation(&snapshot, name);
        assert!(
            actual >= 0.0,
            "'{name}' reports {actual} W: friction can only ever heat, whichever way \
             the flow runs"
        );
        approx::assert_relative_eq!(actual, expected, max_relative = 1e-4);
    }
}

// ---------------------------------------------------------------------------
// Gate 2 — the mechanical-energy closure.
// ---------------------------------------------------------------------------

/// REFERENCE — the plant's total friction equals the pressure the pump put in,
/// minus what elevation took back, minus what is left across the plant.
///
/// ```text
/// Σ_e α_e·Q|Q|  =  (P_supply − P_receiving) + ρ·g·h0 − ρ·g·Δz
/// ```
///
/// Every quantity on the right is either a solved node pressure or a `β` the
/// scenario file states outright, so this closes without ever evaluating `α` —
/// it is an independent check on the dissipation rule rather than a readback of
/// it. Derivation: each branch obeys `dp_e = α_e·Q|Q| + β_e`, and the drops
/// telescope along the series chain to `P_supply − P_receiving`; rearranging for
/// `Σ α_e·Q|Q|` gives the line above, with `Σβ = −ρ·g·h0 + ρ·g·Δz` (the pump's
/// jump is a NEGATIVE drop).
///
/// **This is the gate that discriminates `α` from `β`.** Credit the whole branch
/// drop as heat and the left side gains the pump's 391 481 Pa and loses the
/// elevation's 48 935 — off by a factor of two on one side, 12% on the total.
/// That mutation leaves gate 1 green on the suction line, which carries neither.
#[test]
fn total_dissipation_closes_the_mechanical_energy_balance() {
    let engine = engine_at_reference_state();
    let snapshot = engine.snapshot();

    // Left side: Σ Φ_e / Q, the friction the model booked as heat.
    let flow = snapshot
        .edges
        .iter()
        .find(|e| e.name == "fill_line")
        .expect("fill_line exists")
        .stream
        .mass_flow
        .value();
    let q = flow / RHO_WATER;
    let dissipated_pa: f64 = snapshot.edges.iter().map(|e| e.dissipation_w).sum::<f64>() / q;

    // Right side: the pressure field plus the two reversible offsets.
    let net_drop =
        node_pressure(&snapshot, "supply_tank") - node_pressure(&snapshot, "receiving_tank");
    let pump_jump = RHO_WATER * G * 40.0; // h0_m = 40.0 in the scenario
    let elevation = RHO_WATER * G * 5.0; // fill_line's elevation_change_m = 5.0
    let expected_pa = net_drop + pump_jump - elevation;

    assert!(
        (dissipated_pa - expected_pa).abs() < CLOSURE_TOLERANCE_PA,
        "friction booked {dissipated_pa:.3} Pa but the mechanical balance says \
         {expected_pa:.3} Pa (net drop {net_drop:.3} + pump {pump_jump:.3} − elevation \
         {elevation:.3}); off by {:.3} Pa. Elevation head and the pump's jump are \
         reversible and must NOT appear as heat.",
        dissipated_pa - expected_pa
    );

    // Non-vacuity: the two reversible terms are the whole point, and they are
    // large. If either were near zero this would close for any rule at all.
    assert!(
        pump_jump > 10.0 * CLOSURE_TOLERANCE_PA && elevation > 10.0 * CLOSURE_TOLERANCE_PA,
        "the β terms must be big enough for their exclusion to be testable"
    );
}

/// The plant's total rise, as one number: 0.098442 K from tank to tank.
///
/// DESIGN §3a's bottom line, and the figure the deferral was re-opened on. Kept
/// separate from the per-edge chain because it is the quantity a reader of the
/// design note will look for, and because it stays meaningful if the plant's
/// internal edge structure ever changes.
#[test]
fn the_reference_plant_warms_by_the_measured_total() {
    let engine = engine_at_reference_state();
    let snapshot = engine.snapshot();

    let total: f64 = expected_edge_rises().iter().map(|(_, a)| rise(*a)).sum();
    assert!(
        (total - 0.098_441_5).abs() < 1e-6,
        "the hand calc must reproduce DESIGN §3a's 0.098441 K total, got {total}"
    );

    let delivered = edge_temperature(&snapshot, "fill_line") - PLANT_TEMPERATURE_K;
    assert!(
        (delivered - total).abs() < RISE_TOLERANCE_K,
        "water must reach the receiving tank {total} K above the {PLANT_TEMPERATURE_K} K \
         it left the supply tank at, got {delivered} K"
    );

    // The supply tank is upwind of everything and takes nothing back: it must
    // still be at exactly its initial temperature. See `isothermal_plant.rs`,
    // where this is the gate for heat written to an edge's inlet.
    let supply = match &engine
        .graph
        .node(engine.graph.find_node("supply_tank").unwrap())
        .kind
    {
        NodeKind::Tank(t) => t.temperature.value(),
        other => panic!("supply_tank is not a tank: {other:?}"),
    };
    assert!(
        (supply - PLANT_TEMPERATURE_K).abs() < 1e-12,
        "the supply tank is upstream of every source of friction and must not warm; \
         got {supply} K"
    );
}
