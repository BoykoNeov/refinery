//! The furnace, as wired: `scenarios/furnace_heater.toml` end to end (M2.2).
//!
//! `core::energy`'s unit tests already prove the first law for a heated
//! zero-volume node — `heat_input_on_a_junction_raises_its_outlet_temperature`
//! puts 41 840 W into 2 kg/s of water and gets exactly +5 K. Re-running that
//! arithmetic against a `Furnace` node would add nothing: it feeds the sweep a
//! hand-built flow map, so it cannot see the loader, the solver, or the engine
//! tick.
//!
//! What is NOT covered anywhere else, and is what this file exists for:
//!   1. the `duty_mw` → W conversion in the loader (a 1e6 that nothing else reads),
//!   2. the duty reaching `mix_inflows` through a real hydraulic solve,
//!   3. the transport step writing the heated temperature onto the OUTLET
//!      stream and not the inlet one.
//!
//! Lives in `scenarios/` for the same reason as `isothermal_plant.rs`: it needs
//! the TOML and its loader, and `scenarios` depends on `solvers`, so the
//! reverse import would be a dependency cycle.

use refinery_core::engine::Engine;
use refinery_core::snapshot::EdgeSnapshot;
use refinery_scenarios::{NodeDef, ScenarioFile};

const SCENARIO: &str = include_str!("../../../scenarios/furnace_heater.toml");

/// The feed temperature the TOML declares, in SI (20 °C).
const FEED_K: f64 = 293.15;

/// The duty the TOML declares, in SI. Written here as WATTS while the file
/// says `duty_mw = 1.0`: that is the point of this constant. If the loader's
/// MW→W conversion is dropped, scaled wrong, or applied twice, the predicted
/// ΔT below misses by orders of magnitude. Deriving it by calling the loader's
/// own conversion would make the test agree with any bug in it.
const DUTY_W: f64 = 1.0e6;

/// Ticks to run before reading. The plant has no inventory, so the hydraulic
/// solve is at steady state from tick 1; a handful of ticks just confirms it
/// stays there rather than creeping.
const TICKS: u64 = 20;

/// The temperature field is built from products and one division of
/// O(1)-magnitude doubles, so relative error is a few ulp. 1e-9 K on a ~300 K
/// value is ~1e-12 relative — far below any modelling defect, far above float
/// noise.
const TOLERANCE_K: f64 = 1e-9;

fn build(duty_mw: f64) -> Engine {
    let mut file: ScenarioFile =
        refinery_scenarios::load_str(SCENARIO).expect("the furnace scenario must parse");
    match file
        .nodes
        .get_mut("heater")
        .expect("the scenario must define a 'heater' node")
    {
        NodeDef::Furnace { duty_mw: d } => *d = duty_mw,
        other => panic!("'heater' must be a furnace, got {other:?}"),
    }
    refinery_scenarios::build_engine(&file).expect("the furnace plant must build")
}

fn run(duty_mw: f64) -> Engine {
    let mut engine = build(duty_mw);
    for tick in 1..=TICKS {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick} at {duty_mw} MW failed: {e:?}"));
    }
    engine
}

fn edge(engine: &Engine, name: &str) -> EdgeSnapshot {
    engine
        .snapshot()
        .edges
        .into_iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("the furnace plant must have a '{name}' pipe"))
}

/// First law across the heater: `T_out = T_in + Q/(ṁ·cp)`.
///
/// The flow is read from the solution rather than predicted — this plant's
/// hydraulics are M1's business and already pinned by `kv_reference`. What is
/// *not* read from the engine is the duty: `DUTY_W` comes from the TOML's
/// human-facing number converted by hand, so the loader's MW→W step is under
/// test rather than assumed.
#[test]
fn the_furnace_delivers_its_duty_to_the_stream() {
    let engine = run(1.0);
    let inlet = edge(&engine, "feed_line");
    let outlet = edge(&engine, "transfer_line");

    let mass_flow = inlet.stream.mass_flow.value();
    assert!(
        mass_flow > 1.0,
        "the plant must actually be flowing for the duty to land anywhere, got {mass_flow} kg/s"
    );
    let cp = inlet.stream.composition.mixture_cp(&engine.slate).value();
    let expected = FEED_K + DUTY_W / (mass_flow * cp);

    assert!(
        (outlet.stream.temperature.value() - expected).abs() < TOLERANCE_K,
        "outlet stream must leave at {expected} K (= {FEED_K} + {DUTY_W}/({mass_flow}·{cp})), \
         got {}",
        outlet.stream.temperature.value()
    );

    // Direction: heat goes DOWNSTREAM. The upwind pick writes the furnace's
    // temperature onto its outlet edge only — an inlet edge that warmed up
    // would mean the transport step read the wrong end of the pipe, which the
    // ΔT check above cannot distinguish on its own.
    assert!(
        (inlet.stream.temperature.value() - FEED_K).abs() < TOLERANCE_K,
        "the feed line is upstream of the heater and must stay at {FEED_K} K, got {}",
        inlet.stream.temperature.value()
    );
}

/// ΔT must be exactly proportional to duty.
///
/// This is the gate that needs no flow reading at all, and so has no shared
/// term with the test above. The hydraulics here are temperature-independent
/// (constant density and viscosity at M2), so ṁ and cp are bit-identical
/// across the three runs and cancel exactly in the ratio: the first law then
/// predicts ΔT(2 MW) = 2·ΔT(1 MW) as an exact float identity, not an
/// approximation. A duty that leaked in through some path proportional to
/// something else — a fixed offset, a squared term, a unit slip that is only
/// *linear* — would break the ratio while still producing a plausible number.
#[test]
fn outlet_temperature_rise_is_linear_in_duty() {
    let flow_of = |e: &Engine| edge(e, "feed_line").stream.mass_flow.value();
    let rise_of = |e: &Engine| edge(e, "transfer_line").stream.temperature.value() - FEED_K;

    let one = run(1.0);
    let two = run(2.0);
    assert_eq!(
        flow_of(&one),
        flow_of(&two),
        "hydraulics must be temperature-independent at M2, or the ratio below \
         is not a clean test of the energy balance"
    );

    let (rise_one, rise_two) = (rise_of(&one), rise_of(&two));
    assert!(
        rise_one > 1.0,
        "1 MW must produce a rise big enough to be worth halving, got {rise_one} K"
    );
    assert!(
        (rise_two - 2.0 * rise_one).abs() < TOLERANCE_K,
        "doubling the duty must exactly double the rise: {rise_one} K → {rise_two} K"
    );
}

/// An unlit furnace is a pass-through, exactly.
///
/// The flat-line counterpart to the tests above, and the same trap
/// `isothermal_plant.rs` sets for the reference plant: every gate here that
/// measures a temperature *difference* would still pass if the furnace added a
/// constant offset, or if `heat_load` picked up a stray term. At duty 0 the
/// whole plant is 20 °C and any such term shows up immediately.
#[test]
fn a_furnace_at_zero_duty_is_exactly_isothermal() {
    let engine = run(0.0);
    let snapshot = engine.snapshot();
    assert_eq!(
        snapshot.edges.len(),
        2,
        "the furnace plant has two pipes; a vacuous loop would prove nothing"
    );
    for edge in snapshot.edges {
        assert!(
            (edge.stream.temperature.value() - FEED_K).abs() < TOLERANCE_K,
            "with the heater shut down, stream '{}' must stay at {FEED_K} K, got {}",
            edge.name,
            edge.stream.temperature.value()
        );
    }
    for node in snapshot.nodes {
        assert!(
            (node.temperature_k - FEED_K).abs() < TOLERANCE_K,
            "with the heater shut down, node '{}' must stay at {FEED_K} K, got {}",
            node.name,
            node.temperature_k
        );
    }
}

/// A fire on a furnace must ADD to its duty, not replace it.
///
/// This is the whole reason `duty` is a field of its own rather than reusing
/// `Node::heat_input` (see `energy::heat_load`). If the two shared storage,
/// `SetHeatInput` would silently zero the operator's setpoint — the plant
/// would run *colder* during a fire — and every other test in this file would
/// still pass, because none of them sets both.
#[test]
fn a_fire_stacks_on_top_of_the_operating_duty() {
    use refinery_core::snapshot::Command;
    use refinery_core::units::Watt;

    let lit = run(1.0);
    let rise_from_duty = edge(&lit, "transfer_line").stream.temperature.value() - FEED_K;
    // Without this the test is vacuous under any mutation that stops the duty
    // reaching the stream at all: 0 K doubles to 0 K and the ratio holds.
    assert!(
        rise_from_duty > 1.0,
        "the duty alone must produce a real rise for the sum below to mean \
         anything, got {rise_from_duty} K"
    );

    // Same plant, plus a fire worth exactly the same power as the duty.
    let mut burning = build(1.0);
    let heater = burning
        .graph
        .find_node("heater")
        .expect("the scenario must define a 'heater' node");
    burning
        .apply(Command::SetHeatInput {
            node: heater,
            power: Watt(DUTY_W),
        })
        .expect("a fire on a furnace is a valid command");
    for tick in 1..=TICKS {
        burning
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick} with a fire failed: {e:?}"));
    }

    let rise_with_fire = edge(&burning, "transfer_line").stream.temperature.value() - FEED_K;
    assert!(
        (rise_with_fire - 2.0 * rise_from_duty).abs() < TOLERANCE_K,
        "a fire of Q on a furnace already firing Q must double the rise \
         ({rise_from_duty} K → expected {}), got {rise_with_fire} K — if it \
         merely matched, the fire overwrote the duty",
        2.0 * rise_from_duty
    );
}
