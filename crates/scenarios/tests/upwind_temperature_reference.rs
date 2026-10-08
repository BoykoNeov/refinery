//! The upwind temperature a gas edge compiles its density at, once the plant has
//! been TICKED (docs/ROADMAP.md §M5.4, docs/DESIGN.md §3a fork 6).
//!
//! **Why this file exists at all, and why it could not be part of
//! `gas_density_reference.rs`.** That file's header commits, deliberately, to
//! solving the FRESHLY BUILT graph in every gate: before the first tick both
//! pipes still carry `Stream::stagnant`'s seed temperature, so the hand calc is
//! exact rather than dependent on a thermal history. That choice is right for
//! what it pins — the density law and the upwind convention — and it has one
//! consequence nobody noticed until M5.4: **the fallback taken when the upwind
//! node is ZERO-VOLUME is never exercised by any gate in this workspace.**
//!
//! It was measurably wrong. Until M5.4 such an edge compiled at the pipe's stored
//! OUTLET temperature — its inlet plus the ambient exchange and the frictional
//! dissipation the pipe added — which on this plant is 375.5 K against the tee's
//! real 297.2 K. The correction moves `gas_line`'s steady flow from
//! **0.885409 to 0.979114 kg/s, +10.6%**, and the entire test suite stayed green
//! across that change: 16 of 18 scenario × fidelity runs were byte-identical and
//! the two that moved were this plant, unwatched. That is the hole this file
//! closes.
//!
//! **What is pinned here, and what is not.** The closed form below is
//! `gas_density_reference`'s, generalized so the two edges may carry different
//! TEMPERATURES as well as different pressures. It therefore inherits that file's
//! stated ceiling — one density per edge, while a real gas expands along the pipe
//! — and adds nothing to it. What is genuinely new is the discrimination: the
//! measured flow is asserted to sit on the prediction built from the TEE's
//! resolved temperature and to be far from the one built from the relief line's
//! own outlet temperature. Those two differ by 7.9% here, so a passing run means
//! the distinction is resolved correctly rather than being unobservable.

use refinery_core::graph::{EdgeId, NodeId, PlantGraph};

const GAS_PLANT: &str = include_str!("../../../scenarios/gas_line.toml");

// --- The reference plant's declared constants, restated -----------------------
// Restated rather than read back, `gas_density_reference`'s convention: a test
// that fetches its expected inputs from the object under test can only ever
// check self-consistency.

/// CODATA universal gas constant [J/(mol·K)], exact since the 2019 SI redefinition.
const R: f64 = 8.314_462_618_153_24;
/// `gas_line.toml`'s fuel gas: methane's molar mass [kg/mol].
const M_BAR: f64 = 0.016_043;
/// The header source's declared temperature [K] (20 °C). It is a `Source`, so it
/// has a temperature of its OWN and `boundary_temperature` answers for it — the
/// header run is unaffected by anything in this file, which is what makes the
/// relief line the only moving part.
const T_HEADER_K: f64 = 293.15;
const P_HEADER_PA: f64 = 10.0e5;
const P_FLARE_PA: f64 = 1.0e5;
const FRICTION: f64 = 0.02;

/// Tolerance [relative], the same figure and the same derivation as
/// `gas_density_reference::HAND_CALC_TOLERANCE`: `eps_dp = 1.0` Pa against the
/// smaller ~72 kPa branch drop is a ~7e-6 softening, and the extra term here is
/// the one-tick lag, which is exactly zero once the plant is stationary (asserted
/// below rather than assumed). Measured deviation on this gate: 1.9e-6.
const HAND_CALC_TOLERANCE: f64 = 1e-4;

/// `2·D·A²/(f·L)` for one pipe — the geometry group in `ṁ² = ρ_upwind·ΔP·K`.
/// Darcy–Weisbach `ΔP = f·L·ρ·Q²/(2·D·A²)` with `ṁ = ρ·Q`.
fn geometry_group(length_m: f64, diameter_m: f64) -> f64 {
    let area = std::f64::consts::PI * diameter_m * diameter_m / 4.0;
    2.0 * diameter_m * area * area / (FRICTION * length_m)
}
fn header_group() -> f64 {
    geometry_group(100.0, 0.08)
}
fn relief_group() -> f64 {
    geometry_group(100.0, 0.05)
}

/// Hand calculation: tee pressure and mass flow with each edge's density taken at
/// its own upwind PRESSURE **and its own upwind TEMPERATURE**.
///
/// `gas_density_reference::upwind_prediction` is the `t_tee == t_header` case of
/// this, where `ρ/P` is one constant and cancels from the pressure balance. Here
/// it does not cancel — that is the whole point — so the balance is
///
/// ```text
/// A·P₀·(P₀ − P_t) = B·P_t·(P_t − P₁),   A = K_h·M̄/(R·T_header),
///                                       B = K_r·M̄/(R·T_tee)
/// ```
///
/// i.e. `B·P_t² + (A·P₀ − B·P₁)·P_t − A·P₀² = 0`, whose constant term is negative
/// so the roots straddle zero and the positive one is physical.
fn prediction(t_tee_k: f64) -> (f64, f64) {
    let c_header = M_BAR / (R * T_HEADER_K);
    let c_tee = M_BAR / (R * t_tee_k);
    let (a_coef, b_coef) = (c_header * header_group(), c_tee * relief_group());
    let (a, b, c) = (
        b_coef,
        a_coef * P_HEADER_PA - b_coef * P_FLARE_PA,
        -a_coef * P_HEADER_PA * P_HEADER_PA,
    );
    let p_tee = (-b + (b * b - 4.0 * a * c).sqrt()) / (2.0 * a);
    let mass_flow = (c_header * P_HEADER_PA * (P_HEADER_PA - p_tee) * header_group()).sqrt();
    (p_tee, mass_flow)
}

fn build() -> refinery_core::engine::Engine {
    let file = refinery_scenarios::load_str(GAS_PLANT).expect("gas_line.toml parses");
    refinery_scenarios::build_engine(&file).expect("gas_line.toml builds")
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

/// One observation of the ticked plant: what the solve produced, and the two
/// temperatures that compete to be the relief line's upwind value.
struct Observed {
    flow: f64,
    tee_pressure: f64,
    tee_temperature: f64,
    relief_outlet_temperature: f64,
}

fn run(ticks: usize) -> Observed {
    let mut engine = build();
    for i in 0..ticks {
        engine.tick().unwrap_or_else(|e| panic!("tick {i}: {e}"));
    }
    let graph = &engine.graph;
    let relief = edge_by_name(graph, "relief_line");
    let tee = node_by_name(graph, "tee");
    let snap = engine.snapshot();
    Observed {
        flow: graph.pipe(relief).stream.mass_flow.value(),
        tee_pressure: snap
            .nodes
            .iter()
            .find(|n| n.id == tee)
            .expect("tee in snapshot")
            .pressure_pa,
        tee_temperature: snap
            .nodes
            .iter()
            .find(|n| n.id == tee)
            .expect("tee in snapshot")
            .temperature_k,
        relief_outlet_temperature: graph.pipe(relief).stream.temperature.value(),
    }
}

// ---------------------------------------------------------------------------
// The gate.
// ---------------------------------------------------------------------------

/// A gas edge whose upwind node is ZERO-VOLUME compiles its density at that
/// node's resolved temperature, not at its own stored outlet temperature.
///
/// The plant is `source → tee → flare`, so the relief line's upwind node is a
/// `Junction`: it has no temperature of its own and `boundary_temperature`
/// answers `None` for it. The header run's does (a `Source`), which is what makes
/// this a one-variable experiment.
///
/// Three premises are ASSERTED rather than assumed, because each of them would
/// make the gate vacuous if it failed quietly:
///
/// 1. the two candidate temperatures are far apart (they are not, on a plant with
///    no dissipation — and this gate would then be testing nothing);
/// 2. the tee's temperature is stationary tick to tick, so the one-tick lag
///    between the sweep and the next solve contributes exactly zero and the
///    steady closed form is the right comparison;
/// 3. the measured flow is FAR from the stored-outlet prediction, so a pass means
///    the two models were told apart and not merely that one of them fits.
#[test]
fn a_zero_volume_upwind_node_supplies_its_resolved_temperature() {
    let late = run(40);
    let earlier = run(39);

    // (1) The two candidates must actually differ. 297 K vs 360 K here — the
    // relief line dissipates ~135 kW into ~1 kg/s of gas.
    let gap = (late.relief_outlet_temperature - late.tee_temperature).abs();
    assert!(
        gap > 20.0,
        "premise: the tee's resolved temperature ({:.3} K) and the relief line's \
         outlet ({:.3} K) must differ enough for this gate to discriminate; gap {gap:.3} K",
        late.tee_temperature,
        late.relief_outlet_temperature
    );

    // (2) Stationary, so the one-tick lag is not confounding the comparison.
    approx::assert_relative_eq!(
        late.tee_temperature,
        earlier.tee_temperature,
        max_relative = 1e-12
    );

    // The claim: density at the TEE's temperature.
    let (expected_tee_p, expected_flow) = prediction(late.tee_temperature);
    approx::assert_relative_eq!(late.flow, expected_flow, max_relative = HAND_CALC_TOLERANCE);
    approx::assert_relative_eq!(
        late.tee_pressure,
        expected_tee_p,
        max_relative = HAND_CALC_TOLERANCE
    );

    // (3) And far from the alternative it replaced. 7.9% apart — ~800x the
    // tolerance, which is the headroom this discrimination has.
    let (_, stored_outlet_flow) = prediction(late.relief_outlet_temperature);
    let separation = (late.flow - stored_outlet_flow).abs() / expected_flow;
    assert!(
        separation > 100.0 * HAND_CALC_TOLERANCE,
        "the stored-outlet prediction ({stored_outlet_flow:.6} kg/s) must be far from \
         the measured {:.6} kg/s for this gate to mean anything; separation {separation:.3e}",
        late.flow
    );
}

/// The FIRST tick is solved at the tee's own temperature too (M55.0,
/// docs/DESIGN.md §60.0, ledger row B50): before any sweep there is no resolved
/// state, so the tick's first solve reads the pipe's stored temperature — the
/// header's 20 °C — and the engine then re-solves the tick on the states that
/// solve's flows resolve, until the densities the solve reads stop moving.
///
/// Until M55 the first tick kept the fallback's answer and the plant moved on
/// tick two, by 7.1e-3 of its flow (this gate asserted both) — 70 times the
/// tolerance, the separation this discrimination has. Now the first tick
/// sits on the hand calculation at the tee's resolved temperature, and the
/// second does not move — the stationary plant's one-tick lag is gone from tick
/// one on. `gas_density_reference`'s gates solve the untouched graph through the
/// solver directly and are unaffected.
#[test]
fn the_first_tick_is_solved_at_the_tees_own_temperature() {
    let first = run(1);
    // The fallback and the tee must differ for this to discriminate anything.
    assert!(
        (first.tee_temperature - T_HEADER_K).abs() > 1.0,
        "premise: the tee ({:.3} K) must sit away from the header's 20 °C",
        first.tee_temperature
    );
    let (expected_tee_p, expected_flow) = prediction(first.tee_temperature);
    approx::assert_relative_eq!(
        first.flow,
        expected_flow,
        max_relative = HAND_CALC_TOLERANCE
    );
    approx::assert_relative_eq!(
        first.tee_pressure,
        expected_tee_p,
        max_relative = HAND_CALC_TOLERANCE
    );
    let (_, fallback_flow) = prediction(T_HEADER_K);
    assert!(
        (first.flow - fallback_flow).abs() / expected_flow > 50.0 * HAND_CALC_TOLERANCE,
        "the first tick ({:.6} kg/s) must be far from the stored-temperature answer \
         ({fallback_flow:.6} kg/s) for this gate to mean anything",
        first.flow
    );

    let second = run(2);
    approx::assert_relative_eq!(second.flow, first.flow, max_relative = 1e-8);
}

// ---------------------------------------------------------------------------
// The ORDER of the fallback — a separate claim, on a different plant.
// ---------------------------------------------------------------------------

/// A node that HAS a temperature of its own wins over the previous tick's
/// resolved value for it.
///
/// The two agree for an inertial node at steady state, which is why this needs a
/// plant whose holdup temperature is still MOVING; `knockout_drum` filling from
/// 8 bar is one. Swapping the two arms changes that plant's trajectory — measured,
/// both fidelities — and until this gate existed nothing in the workspace could
/// tell the difference.
///
/// What pins it is the identity M5.3 states in prose and rests gate (iii) on:
/// `m_new = C·P_solved` with `C = V·M̄/(R·T)`, so the discharge edge's
/// `ρ = P·M̄/(R·T)` **is** the vessel's own `m/V`, exactly. That is true only
/// while the edge and the capacitance are evaluated at the SAME temperature —
/// the vessel's current one. Reading the previous tick's resolved value instead
/// leaves `C` current and `ρ` stale, and the identity breaks by the ratio of the
/// two temperatures.
///
/// Note this gate reproduces the engine's own compile rather than approximating
/// it: `Engine::node_states` is what the next tick's solve would be handed, and
/// passing an empty `NodeStates` here would silently take the tick-0 path and
/// observe nothing.
#[test]
fn a_node_with_its_own_temperature_outranks_the_previous_resolved_value() {
    use refinery_core::graph::NodeKind;
    use std::collections::BTreeMap;

    const DRUM_PLANT: &str = include_str!("../../../scenarios/knockout_drum.toml");
    const DRUM_VOLUME_M3: f64 = 1.0;

    let file = refinery_scenarios::load_str(DRUM_PLANT).expect("knockout_drum.toml parses");
    let mut engine = refinery_scenarios::build_engine(&file).expect("knockout_drum.toml builds");
    // Early enough that the drum is still filling, so its temperature is moving.
    for i in 0..5 {
        engine.tick().unwrap_or_else(|e| panic!("tick {i}: {e}"));
    }

    let drum = node_by_name(&engine.graph, "drum");
    let (mass, temperature) = match &engine.graph.node(drum).kind {
        NodeKind::Vessel(v) => (v.mass.value(), v.temperature.value()),
        other => panic!("knockout_drum must define drum as a vessel, got {other:?}"),
    };
    let stale = engine
        .node_states()
        .temperature
        .get(&drum)
        .copied()
        .expect("the sweep resolves every node")
        .value();

    // Premise: the two candidates must differ, or this gate is vacuous. They do
    // while the drum is filling — it is being compression-heated.
    let separation = (temperature - stale).abs() / temperature;
    assert!(
        separation > 1e-4,
        "premise: the drum's own temperature ({temperature:.6} K) and the previous \
         tick's resolved value ({stale:.6} K) must differ for this gate to \
         discriminate; separation {separation:.3e}"
    );

    // The drum is above the downstream sink, so it is the outlet run's upwind end.
    let outlet = edge_by_name(&engine.graph, "outlet_run");
    let prepared = refinery_solvers::network::prepare(
        &engine.graph,
        &engine.slate,
        engine.node_states(),
        &BTreeMap::new(),
    )
    .expect("the ticked plant prepares");
    let compiled = &prepared.compiled[&outlet];
    assert!(
        prepared.pressures[&compiled.src] >= prepared.pressures[&compiled.tgt],
        "premise: the drum must be the outlet run's upwind end"
    );

    // ρ_edge == m/V, exactly — round-off, not a physical tolerance, because the
    // two expressions are the same product of the same three numbers.
    approx::assert_relative_eq!(compiled.rho, mass / DRUM_VOLUME_M3, max_relative = 1e-12);
}

/// Measurement, not a gate. Prints the two predictions against the measured flow,
/// which is where this file's "+10.6%" and "7.9% apart" figures come from.
/// `cargo test -p refinery-scenarios --test upwind_temperature_reference -- \
/// --ignored --nocapture`
#[test]
#[ignore = "measurement, not a gate"]
fn measure_the_upwind_temperature_choice() {
    let o = run(40);
    let (tee_p, tee_flow) = prediction(o.tee_temperature);
    let (out_p, out_flow) = prediction(o.relief_outlet_temperature);
    println!(
        "measured        flow {:.9} kg/s, tee {:.3} Pa",
        o.flow, o.tee_pressure
    );
    println!(
        "at T_tee   {:.3} K: flow {tee_flow:.9} (rel dev {:.3e}), tee {tee_p:.3}",
        o.tee_temperature,
        (o.flow - tee_flow).abs() / tee_flow
    );
    println!(
        "at T_out   {:.3} K: flow {out_flow:.9} (rel dev {:.3e}), tee {out_p:.3}",
        o.relief_outlet_temperature,
        (o.flow - out_flow).abs() / out_flow
    );
}
