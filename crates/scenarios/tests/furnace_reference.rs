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

fn node_temperature(engine: &Engine, name: &str) -> f64 {
    engine
        .snapshot()
        .nodes
        .into_iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("the furnace plant must have a '{name}' node"))
        .temperature_k
}

/// The most any edge in this plant can warm itself by friction [K].
///
/// Since M5.1 every edge dissipates `α·Q|Q|·Q` into its own stream, so no gate
/// here can assert an exact 293.15 K downstream of a flowing pipe any more. Where
/// a gate's claim is really about DIRECTION — which end of a pipe a term reached
/// — this bound stands in for the old exact equality, and it still discriminates
/// by two orders of magnitude: the frictional rises here are hundredths of a
/// kelvin against a duty rise of ~4.3 K. Where a gate's claim is an exact
/// identity, it is restated below in a form friction cancels out of rather than
/// relaxed to a bound.
const FRICTION_BOUND_K: f64 = 0.5;

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

    // Stated across the FURNACE — from what arrives to what the node resolves to
    // — rather than from the source's 293.15 K to the outlet stream. Both of
    // those ends move once friction exists (the feed line warms on the way in,
    // the transfer line on the way out) and neither movement belongs to the duty.
    // Written this way the two frictional terms are simply not inside the claim,
    // so this stays an EXACT first law instead of an approximate one carrying
    // slack for a nuisance term.
    let arriving = inlet.stream.temperature.value();
    let expected = arriving + DUTY_W / (mass_flow * cp);
    let heated = node_temperature(&engine, "heater");
    assert!(
        (heated - expected).abs() < TOLERANCE_K,
        "the heater must resolve to {expected} K (= {arriving} arriving + \
         {DUTY_W}/({mass_flow}·{cp})), got {heated}"
    );

    // Direction: heat goes DOWNSTREAM. The upwind pick writes the furnace's
    // temperature onto its outlet edge only — an inlet edge that warmed up
    // would mean the transport step read the wrong end of the pipe, which the
    // ΔT check above cannot distinguish on its own.
    assert!(
        arriving > FEED_K && arriving - FEED_K < FRICTION_BOUND_K,
        "the feed line is upstream of the heater and must carry only its own \
         friction above {FEED_K} K, not any part of the ~4 K duty rise; got {arriving}"
    );
    // ...and the outlet stream really does carry the heated temperature on, plus
    // its own friction and nothing else.
    let leaving = outlet.stream.temperature.value();
    assert!(
        leaving > heated && leaving - heated < FRICTION_BOUND_K,
        "the transfer line must carry the heater's {heated} K downstream, got {leaving}"
    );
}

/// ΔT must be exactly AFFINE in duty, with the intercept being friction alone.
///
/// This is the gate that needs no flow reading at all, and so has no shared term
/// with the test above. The hydraulics here are temperature-independent (constant
/// density and viscosity at M2), so ṁ, cp and every edge's `Φ` are bit-identical
/// across the three runs: the frictional part of the rise is the SAME number in
/// each and cancels out of a second difference.
///
/// Before M5.1 this was `ΔT(2 MW) = 2·ΔT(1 MW)`, which friction breaks — it adds
/// a duty-independent intercept. The equal-second-difference form below is that
/// claim with the intercept divided out, and it is strictly stronger: it pins
/// linearity AND identifies the intercept as exactly the unlit rise, so a term
/// that scaled with duty while also leaking a constant fails here where the old
/// ratio could absorb it. A duty proportional to something else — a squared term,
/// a unit slip that is only *linear* — still breaks it while producing a
/// plausible number.
#[test]
fn outlet_temperature_rise_is_affine_in_duty() {
    let flow_of = |e: &Engine| edge(e, "feed_line").stream.mass_flow.value();
    let rise_of = |e: &Engine| edge(e, "transfer_line").stream.temperature.value() - FEED_K;

    let zero = run(0.0);
    let one = run(1.0);
    let two = run(2.0);
    assert_eq!(
        flow_of(&one),
        flow_of(&two),
        "hydraulics must be temperature-independent at M2, or the differences below \
         are not a clean test of the energy balance"
    );
    assert_eq!(
        flow_of(&zero),
        flow_of(&one),
        "the unlit run must be hydraulically identical too, or its rise is not the \
         same intercept the other two carry"
    );

    let (rise_zero, rise_one, rise_two) = (rise_of(&zero), rise_of(&one), rise_of(&two));
    assert!(
        rise_one - rise_zero > 1.0,
        "1 MW must produce a rise big enough to be worth differencing, got {} K",
        rise_one - rise_zero
    );
    assert!(
        (rise_two - rise_one - (rise_one - rise_zero)).abs() < TOLERANCE_K,
        "each extra MW must add exactly the same rise: {rise_zero} K unlit, \
         {rise_one} K at 1 MW, {rise_two} K at 2 MW"
    );
    // The intercept is this plant's own friction — named, rather than left as an
    // unexplained constant the test quietly tolerates.
    assert!(
        rise_zero > 0.0 && rise_zero < FRICTION_BOUND_K,
        "the unlit intercept must be friction and nothing else, got {rise_zero} K"
    );
}

/// An unlit furnace is a pass-through, exactly.
///
/// The trap `isothermal_plant.rs` sets for the reference plant: every gate here
/// that measures a temperature *difference* would still pass if the furnace added
/// a constant offset, or if `heat_load` picked up a stray term.
///
/// Since M5.1 the plant is no longer 20 °C throughout — the two pipes warm
/// themselves by friction — so the trap moves to the one relation friction cannot
/// reach: an unlit furnace mixes its single inflow and adds nothing, so it must
/// resolve to EXACTLY what that inflow delivers. A stray term in `heat_load`
/// breaks that identity at 1e-12 whatever the pipes are doing, which is a tighter
/// statement than the old flat line rather than a looser one.
#[test]
fn a_furnace_at_zero_duty_is_an_exact_pass_through() {
    let engine = run(0.0);
    let arriving = edge(&engine, "feed_line").stream.temperature.value();
    let heated = node_temperature(&engine, "heater");
    assert!(
        (heated - arriving).abs() < 1e-12,
        "an unlit furnace mixes one inflow and adds nothing: it must sit at exactly \
         the {arriving} K that arrives, got {heated}"
    );

    let snapshot = engine.snapshot();
    assert_eq!(
        snapshot.edges.len(),
        2,
        "the furnace plant has two pipes; a vacuous loop would prove nothing"
    );
    // And nothing anywhere may move by more than the plant's own friction, which
    // bounds any stray term two orders below the duty this plant normally carries.
    for edge in snapshot.edges {
        let t = edge.stream.temperature.value();
        assert!(
            t > FEED_K && t - FEED_K < FRICTION_BOUND_K,
            "with the heater shut down, stream '{}' must carry only its own friction \
             above {FEED_K} K, got {t}",
            edge.name
        );
    }
    for node in snapshot.nodes {
        assert!(
            node.temperature_k >= FEED_K && node.temperature_k - FEED_K < FRICTION_BOUND_K,
            "with the heater shut down, node '{}' must carry only friction above \
             {FEED_K} K, got {}",
            node.name,
            node.temperature_k
        );
    }
}

/// A furnace duty is a non-negative magnitude, at both entry points.
///
/// Since M2.2 gave cooling its own `Cooler` unit, a negative furnace duty
/// expresses nothing — it can only be a sign slip, and a furnace that quietly
/// chills is exactly the plausible-looking wrong plant the two-unit split
/// exists to prevent (DESIGN §4a).
///
/// The command half must be sent to a REAL furnace to mean anything: on any
/// other node kind `SetFurnaceDuty` returns `InvalidCommand` from the wrong-kind
/// arm whether or not the duty was ever range-checked, so the assertion would
/// hold with the guard deleted. That is why this lives here rather than beside
/// its cooler counterpart in `cooler_reference.rs`.
#[test]
fn negative_furnace_duty_is_refused() {
    use refinery_core::error::SimError;
    use refinery_core::snapshot::Command;
    use refinery_core::units::Watt;

    // At load. `Engine` is not `Debug`, so unwrap the Result by hand.
    let mut file: ScenarioFile = refinery_scenarios::load_str(SCENARIO).expect("must parse");
    match file.nodes.get_mut("heater").expect("a 'heater' node") {
        NodeDef::Furnace { duty_mw: d } => *d = -1.0,
        other => panic!("'heater' must be a furnace, got {other:?}"),
    }
    match refinery_scenarios::build_engine(&file) {
        Ok(_) => panic!("a negative furnace duty_mw must not build"),
        Err(e) => assert!(
            matches!(e, SimError::Scenario(_)) && e.to_string().contains("furnace"),
            "must fail as a Scenario error naming the unit, got {e:?}"
        ),
    }

    // At the command boundary, against the real furnace.
    let mut engine = build(1.0);
    let heater = engine.graph.find_node("heater").expect("a 'heater' node");
    let error = engine
        .apply(Command::SetFurnaceDuty {
            node: heater,
            duty: Watt(-1.0),
        })
        .expect_err("a negative duty setpoint must be refused");
    assert!(
        matches!(error, SimError::InvalidCommand(_)),
        "a bad setpoint is an invalid command, got {error:?}"
    );
    assert!(
        !error.to_string().contains("is not a furnace"),
        "this must be refused for its NEGATIVE duty, not rejected as the wrong \
         node kind — otherwise the guard is untested: {error}"
    );

    // The refusal must leave the setpoint alone rather than half-apply it.
    for _ in 1..=TICKS {
        engine.tick().expect("the furnace plant must still run");
    }
    let rise = edge(&engine, "transfer_line").stream.temperature.value() - FEED_K;
    assert!(
        rise > 1.0,
        "the rejected command must not have disturbed the 1 MW setpoint, got a \
         {rise} K rise"
    );
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

    // Measured against the UNLIT plant rather than against the feed temperature.
    // Since M5.1 the transfer line carries its own friction too, and that
    // intercept is duty-independent, so leaving it in would make "double the
    // rise" false for entirely correct physics. The three runs are hydraulically
    // identical, so subtracting the unlit run removes it exactly.
    let unlit = run(0.0);
    let friction = edge(&unlit, "transfer_line").stream.temperature.value() - FEED_K;
    let lit = run(1.0);
    let rise_from_duty = edge(&lit, "transfer_line").stream.temperature.value() - FEED_K - friction;
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

    let rise_with_fire =
        edge(&burning, "transfer_line").stream.temperature.value() - FEED_K - friction;
    assert!(
        (rise_with_fire - 2.0 * rise_from_duty).abs() < TOLERANCE_K,
        "a fire of Q on a furnace already firing Q must double the rise \
         ({rise_from_duty} K → expected {}), got {rise_with_fire} K — if it \
         merely matched, the fire overwrote the duty",
        2.0 * rise_from_duty
    );
}
