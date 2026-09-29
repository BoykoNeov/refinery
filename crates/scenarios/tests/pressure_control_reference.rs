//! Gates for the second controlled variable — a vessel's pressure (M10.1,
//! DESIGN §12).
//!
//! **What this file is really testing is M8's claim that the control seam was
//! variable-agnostic.** M8.2 built `Controller`, `ControlLoop`, `ControlledValue`
//! and the tick pass with exactly one variable in them, and asserted that adding
//! another would need no new machinery. The only way to find out is to add one,
//! and the answer is recorded here rather than in prose: `Engine::run_control_loops`
//! is UNCHANGED by this milestone. It already called
//! `measure(&slate, control.measurement_node, control.setpoint.variable())`, and
//! that line reads a vessel's pressure without knowing it did anything new. (That
//! call is quoted as it stood at M10. M19 added the resolved states and M20 the
//! hydraulic solution and a `MeasurementPoint` in place of the node — each a new
//! argument for a variable a vessel's arm ignores, so the claim above still holds
//! for the pressure arm.)
//!
//! **The premise that deferred this variable was false, and gate 1 is the one
//! that says so.** DESIGN §10 fork 3 split plant quantities into stored and
//! solved, and put pressure on the solved side — "lives in `last_solution` /
//! `NodeStates`, both of which are empty before the first tick. A loop on either
//! has no measurement at tick 0 and needs a stated rule for that tick." A
//! `Vessel`'s pressure is `m/C` with the mass on the graph. It is stored, it is
//! real from load, and the tick-0 rule that paragraph promised was never owed.
//!
//! Gate 1 is written to assert that and NOT to be a round trip. The equality half
//! alone would be near-tautological — the loader builds the initial mass as
//! `P_declared · capacitance(slate)` through the same `capacitance` method
//! `pressure` divides by, so measuring it back is a round trip that could only
//! catch an asymmetric fault (M7.2's rule). **The `NaN` half is what
//! discriminates**: the same node's `pressure_pa` on the same snapshot is `NaN`,
//! because the solve genuinely has not run. Two quantities, two independent
//! paths, one of them absent — that is the finding.
//!
//! The fixtures are declared here rather than shipped, exactly as
//! `control_reference.rs` declares its own: a plant built to expose one behaviour
//! is a fixture, and the files in `scenarios/` are the regression anchor. The
//! wired demo lives in `pressure_control_demo.rs`.

use refinery_core::graph::{
    ControlAction, ControlledValue, LoopId, MeasuredVariable, MeasurementPoint, NodeId,
};
use refinery_core::snapshot::Command;
use refinery_core::units::{Meter, Pascal};
use refinery_core::Engine;

// ------------------------------------------------------------------ fixtures

/// A gas receiver on a make-up line, vented to flare through a controlled valve.
///
/// The shape of `relief_blowdown.toml` with the relief valve replaced by an
/// ordinary one, which is the pair DESIGN §12 fork 5 asks the demo to make. Sized
/// so the vent sits interior at steady state: an operating point pressed against
/// a clamp would make every assertion below a statement about the clamp.
///
/// `x_t` is not decoration. A gas-only slate makes every node in this plant gas
/// by topology, so `require_gas_valve_x_t` refuses the fixture without it — and
/// refuses it on the liquid fixture below. It is a design input the file cannot
/// dodge by omitting.
const GAS_PLANT: &str = r#"
[meta]
name = "pressure_fixture"

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

[nodes.header]
type = "source"
pressure_bar = 30.0
temperature_c = 20.0

[nodes.receiver]
type = "vessel"
volume_m3 = 2.0
pressure_bar = 12.0
temperature_c = 20.0

[nodes.vent_valve]
type = "valve"
kv = 12.0
opening = 0.30
x_t = 0.72

[nodes.flare]
type = "sink"
pressure_bar = 1.1
temperature_c = 20.0

[[pipes]]
name = "make_up"
from = "header"
to = "receiver"
length_m = 20.0
diameter_m = 0.021

[[pipes]]
name = "vent_line"
from = "receiver"
to = "vent_valve"
length_m = 10.0
diameter_m = 0.05

[[pipes]]
name = "flare_line"
from = "vent_valve"
to = "flare"
length_m = 30.0
diameter_m = 0.10
"#;

/// The declared receiver pressure, in bar and in Pascals. Named once so no
/// assertion below can drift away from the fixture it is about.
const DECLARED_BAR: f64 = 12.0;
const DECLARED_PA: f64 = DECLARED_BAR * 1.0e5;

/// A PI loop holding the receiver at 20 bar, the demo's tuning.
const PI_LOOP: &str = r#"
[[controls]]
name = "receiver_pressure"
measurement = { node = "receiver", variable = "pressure" }
actuator = "vent_valve"
algorithm = "pi"
mode = "auto"
setpoint_bar = 20.0
gain_per_bar = 0.10
integral_time_s = 30.0
initial_output = 0.30
"#;

/// A liquid tank plant, for the refusals that need the OTHER variable in play.
///
/// Deliberately minimal — it exists to be refused, and every gate that runs it to
/// completion is in `control_reference.rs` already. No `x_t` anywhere: the same
/// rule that requires one on the gas fixture refuses one here.
const LIQUID_PLANT: &str = r#"
[meta]
name = "level_fixture"

[simulation]
dt = 1.0

[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"

[nodes.feed]
type = "source"
pressure_bar = 5.0
temperature_c = 20.0

[nodes.control_tank]
type = "tank"
area_m2 = 3.0
height_m = 10.0
initial_level_m = 4.0
temperature_c = 20.0

[nodes.drain_valve]
type = "valve"
kv = 30.0
opening = 0.30

[nodes.rundown]
type = "sink"
pressure_bar = 1.01325

[[pipes]]
name = "fill_line"
from = "feed"
to = "control_tank"
length_m = 10.0
diameter_m = 0.10

[[pipes]]
name = "drain_line"
from = "control_tank"
to = "drain_valve"
length_m = 10.0
diameter_m = 0.15

[[pipes]]
name = "rundown_line"
from = "drain_valve"
to = "rundown"
length_m = 10.0
diameter_m = 0.15
"#;

// ------------------------------------------------------------------- helpers

fn engine_from(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("fixture parses");
    refinery_scenarios::build_engine(&file).expect("fixture builds")
}

/// The text of the refusal a scenario earns, whichever stage produced it.
///
/// A `deny_unknown_fields` rejection lands in `load_str` and everything else in
/// `build_engine`; both are refusals of the same file, and the caller asserts on
/// the message rather than on which stage said it.
fn refusal(src: &str) -> String {
    match refinery_scenarios::load_str(src) {
        Err(e) => e.to_string(),
        Ok(file) => match refinery_scenarios::build_engine(&file) {
            Ok(_) => panic!("this scenario was expected to be refused, and loaded"),
            Err(e) => e.to_string(),
        },
    }
}

fn receiver_id(engine: &Engine) -> NodeId {
    engine
        .graph
        .find_node("receiver")
        .expect("the gas fixture has a receiver")
}

/// The pressure the loop ACTED ON, off the faceplate a frontend reads.
fn measured_pa(engine: &Engine) -> f64 {
    match engine
        .snapshot()
        .controls
        .first()
        .expect("the fixture declares one loop")
        .measurement
        .expect("a stored quantity is measured from load")
    {
        ControlledValue::Pressure { pa } => pa.value(),
        other => panic!("this fixture's loop measures a pressure, not {other:?}"),
    }
}

/// The receiver's pressure as the SNAPSHOT reports it — read from the last
/// solve, and `NaN` until one has run.
///
/// A different quantity by a different path from `measured_pa`, which is the
/// whole of gate 1 and gate 2.
fn snapshot_pa(engine: &Engine) -> f64 {
    let id = receiver_id(engine);
    engine
        .snapshot()
        .nodes
        .iter()
        .find(|n| n.id == id)
        .expect("the receiver is in the snapshot")
        .pressure_pa
}

/// The receiver's solved pressure and the temperature that solve was taken at,
/// off one snapshot so the pair cannot come from two different ticks.
fn solved_and_temp(engine: &Engine) -> (f64, f64) {
    let id = receiver_id(engine);
    let snapshot = engine.snapshot();
    let node = snapshot
        .nodes
        .iter()
        .find(|n| n.id == id)
        .expect("the receiver is in the snapshot");
    (node.pressure_pa, node.temperature_k)
}

fn output_now(engine: &Engine) -> f64 {
    engine
        .snapshot()
        .controls
        .first()
        .expect("the fixture declares one loop")
        .output
}

fn run(engine: &mut Engine, ticks: u64) {
    for t in 1..=ticks {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
    }
}

// ------------------------------------------ gate 1: the measurement at tick 0

/// **Gate 1. A vessel-pressure loop has a real measurement before the first
/// tick, and the note that deferred this variable said it could not.**
///
/// Two assertions on two independently computed quantities:
///
/// - the loop's reported measurement is the declared 12 bar, read from the
///   graph's stored mass;
/// - the same node's `pressure_pa` on the same snapshot is `NaN`, read from
///   `last_solution`, which is `None`.
///
/// **The second is the discriminating one.** The first is close to a round trip:
/// the loader builds the mass as `P · capacitance(slate)` and `pressure` divides
/// by the same `capacitance`, so it can only catch an asymmetric fault. Together
/// they say the thing worth saying — the two numbers exist on different
/// schedules, and the one a controller needs is the one that exists first.
#[test]
fn a_pressure_loop_has_a_real_measurement_before_the_first_tick() {
    let engine = engine_from(&format!("{GAS_PLANT}{PI_LOOP}"));

    let measured = measured_pa(&engine);
    // Exact to the round trip's own arithmetic, not to a chosen band: two
    // multiplications and a division by the same factor. A relative 1e-12 is the
    // width of that, and the fixture is a thousandfold inside it.
    assert!(
        ((measured - DECLARED_PA) / DECLARED_PA).abs() < 1.0e-12,
        "before any tick, the loop's measurement must be the vessel's declared \
         pressure of {DECLARED_PA} Pa, and it read {measured} Pa"
    );

    let solved = snapshot_pa(&engine);
    assert!(
        solved.is_nan(),
        "the same node's snapshot pressure comes from the last SOLVE and no solve \
         has run, so it must be NaN — it read {solved}. If this ever becomes a \
         number, gate 1 has stopped discriminating and `measure` may be reading \
         the solved pressure rather than the stored mass"
    );
}

/// The mirror of gate 1 on the OTHER side of fork 1: a junction's pressure really
/// is solved and absent before the first tick, and is still refused.
///
/// This is what keeps gate 1 from reading as "pressure is stored". It is stored
/// on the one node kind that holds mass. **The refusal's REASON changed in M19**
/// (docs/DESIGN.md §23 fork 5): it used to be the missing tick-0 rule, and §23
/// states that rule — "no measurement, no action" — for a furnace outlet. So the
/// refusal is now a scope decision naming docs/DEFERRED.md E9, and the old
/// sentence must be GONE: a message still saying the rule is missing would be the
/// expired sentence M10.1 found left on the page.
#[test]
fn a_junction_pressure_is_refused_as_a_scope_decision_now_the_tick_zero_rule_exists() {
    // The junction sits in the vent line, so it has the two edges a junction
    // needs and the plant is otherwise the fixture.
    let plant = GAS_PLANT
        .replace(
            r#"name = "vent_line"
from = "receiver"
to = "vent_valve""#,
            r#"name = "vent_line"
from = "receiver"
to = "vent_tee""#,
        )
        .replace(
            r#"[nodes.flare]"#,
            r#"[nodes.vent_tee]
type = "junction"

[nodes.flare]"#,
        );
    let plant = format!(
        r#"{plant}
[[pipes]]
name = "tee_line"
from = "vent_tee"
to = "vent_valve"
length_m = 2.0
diameter_m = 0.05
"#
    );
    let loop_on_tee = PI_LOOP.replace(r#"node = "receiver""#, r#"node = "vent_tee""#);

    let message = refusal(&format!("{plant}{loop_on_tee}"));
    assert!(
        message.contains("is a junction, which holds nothing")
            && message.contains("junction-pressure control is not built, as a scope decision")
            && message.contains("docs/DEFERRED.md E9"),
        "a junction's pressure is an unknown of the solve, and the refusal has to say \
         so and name itself a scope decision pointing at E9 — it said: {message}"
    );
    assert!(
        !message.contains("it needs a stated rule for"),
        "the old reason — that no tick-0 rule exists — is false since docs/DESIGN.md \
         §23 and must not survive in the message: {message}"
    );
    // **M20 expired M19's wording in turn** (docs/DESIGN.md §24, site 4). It said
    // the rule "is applied only to a furnace or cooler outlet", and `measure` now
    // applies it to a pipe's flow too, from the very hydraulic solution a
    // junction's pressure would be read from. The path exists; only the scope
    // decision is left, and the message has to say so.
    assert!(
        !message.contains("is applied only to a furnace or cooler outlet"),
        "M19's reason — the rule applied only to an outlet — is false since §24: {message}"
    );
    assert!(
        message.contains("Both halves of reading one exist"),
        "and the reworded message must say the rule AND the solution both exist: {message}"
    );
}

// ------------------------------------------- gate 2: the one-Euler-step offset

/// **Gate 2. During the transient the loop acts on the mass the PREVIOUS tick's
/// solve put in the vessel, not on the pressure standing beside it.**
///
/// M8.5 measured this for a tank at 0.67 Pa, 8.6e-6 relative, and read it as "a
/// snapshot's tank pressure and its tank mass are one Euler step apart". **On a
/// vessel that reading is too simple, and the first draft of this gate asserted
/// it and failed.** The clean identity is on the MASS: the solve closes
/// `C·(P − Pⁿ)/dt = Σṁ` on the vessel node and the integrator then advances the
/// mass with the same flows, so `mⁿ⁺¹ = C·P_solved`. The PRESSURE does not
/// inherit that, because a vessel's capacitance `C = V·M̄/(R·T)` is itself a
/// function of a state that moved: the receiver heats as it fills, so `m/C` is
/// re-evaluated at a new temperature and the raw numbers miss by
/// **653.6 Pa, 4.078e-4 relative — which is exactly `ΔT/T`** (0.1308 K on
/// 320.85 K). Predicting one from the other was wrong; measuring is what found
/// the temperature term.
///
/// So the gate divides it out. `P = m·R·T/(V·M̄)`, so `P/T` is proportional to the
/// mass alone, and the assertion is that the loop's `P/T` at tick N+1 is the
/// solve's `P/T` at tick N. A `measure` that read the solved pressure instead of
/// the stored mass misses this by 1.2e-3 relative against a 1e-6 bar — four
/// orders, not a near miss.
///
/// **It has to be read during the transient.** At steady state every number here
/// is the same number and the assertion is vacuous, so the fixture is run 100
/// ticks — 10 s into an 800 s fill — and the first assertion is the control that
/// says so.
#[test]
fn the_loop_acts_on_the_mass_the_previous_solve_left_in_the_vessel() {
    let mut engine = engine_from(&format!("{GAS_PLANT}{PI_LOOP}"));
    run(&mut engine, 100);

    let (solved_at_n, temp_at_n) = solved_and_temp(&engine);
    let measured_at_n = measured_pa(&engine);

    engine.tick().expect("tick 101");
    let measured_at_n_plus_1 = measured_pa(&engine);
    let (solved_at_n_plus_1, temp_at_n_plus_1) = solved_and_temp(&engine);

    // The control, first: this gate is about a gap, so the plant has to be MOVING
    // by far more than the agreement being claimed.
    let step = solved_at_n_plus_1 - solved_at_n;
    assert!(
        step > 1.0e3,
        "gate 2 is vacuous at steady state and must be read during the transient: \
         one tick moved the receiver {step} Pa, which is not enough to tell a \
         one-step lag from agreement"
    );

    // The identity, with the gas law's temperature term divided out.
    let acted_on = measured_at_n_plus_1 / temp_at_n_plus_1;
    let left_by_solve = solved_at_n / temp_at_n;
    let miss = (acted_on - left_by_solve).abs() / left_by_solve;
    assert!(
        miss < 1.0e-6,
        "`P/T` is proportional to the vessel's mass alone, so the loop's \
         {measured_at_n_plus_1} Pa at {temp_at_n_plus_1} K must carry the same mass as the \
         previous solve's {solved_at_n} Pa at {temp_at_n} K — they differ by {miss} \
         relative. A `measure` reading the SOLVED pressure instead of the stored \
         mass misses this by ~1.2e-3"
    );

    // And the lag is real rather than nominal: the loop is not acting on the
    // pressure standing in the snapshot beside it. Raw, undivided, because this
    // half is about what a frontend sees.
    let lag = solved_at_n - measured_at_n;
    assert!(
        lag > 1.0e3,
        "the loop must report the measurement it ACTED ON, one step behind the \
         snapshot's solved pressure — it reported {measured_at_n} Pa against the \
         snapshot's {solved_at_n} Pa, {lag} Pa apart. A loop reporting the fresh \
         read would look instantaneous and hide its own lag"
    );
}

// ------------------------------------------------------ gate 3: the unit pair

/// The gain and the setpoint for gate 3, chosen so the hand calculation lands
/// interior and BOTH halves of the trap land on a clamp.
///
/// The receiver is declared at 12 bar and the loop aims at 10, so the error is
/// `+2 bar = +2e5 Pa` and a gain of `0.15 / bar` is `1.5e-6 / Pa`. The first
/// output is then `1.5e-6 × 2e5 = 0.30`, comfortably inside `[0, 1]`.
///
/// Convert the setpoint and not the gain and the output is `0.15 × 2e5`; convert
/// the gain and not the setpoint and the error becomes `1.2e6 − 10 = 1.2e6` Pa
/// against `1.5e-6`. Both saturate at 1.0, which is why the assertion is on the
/// interior number rather than on "it did not blow up".
const GATE3_GAIN_PER_BAR: f64 = 0.15;
const GATE3_SETPOINT_BAR: f64 = 10.0;

/// A PROPORTIONAL loop, deliberately: `u = clamp(K·e, 0, 1)` with no memory, so
/// the first output is a hand calculation and not a seeded number.
const P_LOOP_GATE3: &str = r#"
[[controls]]
name = "receiver_pressure"
measurement = { node = "receiver", variable = "pressure" }
actuator = "vent_valve"
algorithm = "p"
mode = "auto"
setpoint_bar = 10.0
gain_per_bar = 0.15
"#;

/// **Gate 3. The setpoint and the gain are converted at the same site, and this
/// is the only thing that can tell.**
///
/// DESIGN §12 fork 3 names the trap in advance: `ControlledValue::magnitude`
/// returns SI, so a controller's error is in Pascals and its gain must be per
/// Pascal, while the file declares both in bar. **Converting one and not the
/// other is a factor of 100 000 that nothing else in this workspace can catch** —
/// the loop stays stable, the solver converges, mass is conserved and the run
/// reruns bit-identically; it is merely tuned five orders of magnitude away. A
/// round trip through the loader would not see it either, since it would read
/// back whatever it wrote.
///
/// So the assertion is against arithmetic done here, by hand, from the file's own
/// declared numbers: `0.30`, and neither 1.0 nor 3.0e-6.
#[test]
fn the_setpoint_and_the_gain_are_converted_at_the_same_site() {
    let mut engine = engine_from(&format!("{GAS_PLANT}{P_LOOP_GATE3}"));
    // One tick: the control pass runs at the TOP of it, on the state standing at
    // load, so the error it sees is exactly the declared 12 bar against 10.
    engine.tick().expect("tick 1");

    let error_pa = DECLARED_PA - GATE3_SETPOINT_BAR * 1.0e5;
    let gain_per_pa = GATE3_GAIN_PER_BAR / 1.0e5;
    let expected = gain_per_pa * error_pa;
    assert!(
        (expected - 0.30).abs() < 1.0e-12,
        "the fixture's own arithmetic must land interior for this gate to \
         discriminate, and computed {expected}"
    );

    let output = output_now(&engine);
    assert!(
        (output - expected).abs() < 1.0e-9,
        "a proportional loop's first output is `K·e` with both sides in SI: \
         {gain_per_pa} /Pa × {error_pa} Pa = {expected}, and the loop produced \
         {output}. A mismatch of exactly 1.0 means one of the two conversions was \
         skipped — the gain and the setpoint both convert by 1e5, at one site, and \
         either alone saturates this valve"
    );
}

// ---------------------------------------------- gate 4: P versus PI, on gas

/// Gate 4's disturbance: the setpoint moves, and the two algorithms answer
/// differently.
///
/// M8.3's version used a leak. The identity it rests on is the algorithm's, not
/// the disturbance's — `u = K·e` forces `Δmeasurement = Δu/K + Δsetpoint` for a
/// proportional loop at any two steady states — so a setpoint step is the same
/// gate with one fewer moving part, and it is also the disturbance a pressure
/// plant can be given without inventing a hole in it.
const STEP_BAR: f64 = 2.0;

fn settle_and_step(algorithm: &str) -> (f64, f64, f64, f64) {
    let table = match algorithm {
        "p" => r#"
[[controls]]
name = "receiver_pressure"
measurement = { node = "receiver", variable = "pressure" }
actuator = "vent_valve"
algorithm = "p"
mode = "auto"
setpoint_bar = 20.0
gain_per_bar = 0.10
"#
        .to_string(),
        _ => PI_LOOP.to_string(),
    };
    let mut engine = engine_from(&format!("{GAS_PLANT}{table}"));
    run(&mut engine, 20_000);
    let before = (measured_pa(&engine), output_now(&engine));

    engine
        .apply(Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Pressure {
                pa: Pascal((20.0 - STEP_BAR) * 1.0e5),
            },
        })
        .expect("a pressure setpoint on a pressure loop is legal");
    run(&mut engine, 20_000);
    (
        before.0,
        before.1,
        measured_pa(&engine),
        output_now(&engine),
    )
}

/// **Gate 4. M8.3's pair, unchanged, on the other variable.**
///
/// Nothing about the proportional/integral distinction is level-specific, and
/// running the same gate on a pressure plant is what shows that rather than
/// asserts it. The discriminating claim is M8.3's and is repeated here because it
/// is the one that is easy to get wrong: **"the PI loop has no offset" does not
/// discriminate**, since a proportional loop's offset is `e = u/K` and a large
/// enough gain shrinks it toward zero with no integral term anywhere. What a
/// proportional loop cannot do is move its output while holding its measurement.
///
/// So: the P half's measurement move must equal `Δu/K + Δsetpoint`, an identity
/// of `u = K·e`; the PI half must land ON the stepped setpoint.
#[test]
fn the_integral_term_holds_a_pressure_a_proportional_loop_can_only_offset() {
    let (p_meas_0, p_out_0, p_meas_1, p_out_1) = settle_and_step("p");
    let (_pi_meas_0, pi_out_0, pi_meas_1, pi_out_1) = settle_and_step("pi");

    // The controls, first. A setpoint step that moved neither valve makes every
    // assertion below a statement about a plant nothing happened to.
    let p_travel = p_out_1 - p_out_0;
    let pi_travel = pi_out_1 - pi_out_0;
    assert!(
        p_travel.abs() > 0.05 && pi_travel.abs() > 0.05,
        "the setpoint step must move both loops' vents measurably, and moved \
         {p_travel:.4} (P) and {pi_travel:.4} (PI)"
    );

    // The P half. `u = K·e` at two steady states gives
    // `Δmeasurement = Δu/K + Δsetpoint`, with the gain in SI.
    let gain_per_pa = 0.10 / 1.0e5;
    let forced = p_travel / gain_per_pa - STEP_BAR * 1.0e5;
    let moved = p_meas_1 - p_meas_0;
    assert!(
        (moved - forced).abs() < 2.0e3,
        "a proportional loop's measurement move is forced by its own output move: \
         it moved {moved} Pa where its {p_travel:.6} of vent travel and the \
         {STEP_BAR} bar step force {forced} Pa. If these disagree, the error term \
         or the gain is not what `u = K·e` says"
    );

    // The offset has to be big enough to be the visible half of the pair.
    let p_offset = p_meas_1 - (20.0 - STEP_BAR) * 1.0e5;
    assert!(
        p_offset.abs() > 1.0e4,
        "the proportional loop's steady-state offset is the whole point of this \
         pair, and it sat only {p_offset} Pa off its setpoint"
    );

    // The PI half lands ON the stepped setpoint. 100 Pa on 18 bar: a stated band
    // five orders inside the offset above, not the measured number rounded up.
    let pi_offset = pi_meas_1 - (20.0 - STEP_BAR) * 1.0e5;
    assert!(
        pi_offset.abs() < 1.0e2,
        "the integral term must return the pressure TO the stepped setpoint, and \
         it ended {pi_offset} Pa away"
    );
    assert!(
        pi_offset.abs() * 20.0 < p_offset.abs(),
        "the pair does not separate: the PI loop sat {pi_offset} Pa off setpoint \
         and the P loop {p_offset} Pa, for comparable vent travel"
    );
}

// ------------------------------------------------------ gate 5: the refusals

/// **Gate 5a — fork 1's three refusals, each asserted on its own message.**
///
/// "The file fails to load" is not a gate: three different mistakes have to
/// produce three different sentences, or a reader who makes one of them is sent
/// looking for another. The junction arm is the one with a deferral behind it and
/// has its own test above.
#[test]
fn a_pressure_measured_on_a_node_that_cannot_answer_is_refused_with_its_own_reason() {
    // A tank. Refused because the density cancels out of `P_atm + ρ·g·h`, so this
    // is a level loop in a worse unit — and the message names what was meant.
    let tank_loop = r#"
[[controls]]
name = "tank_pressure"
measurement = { node = "control_tank", variable = "pressure" }
actuator = "drain_valve"
algorithm = "p"
mode = "auto"
setpoint_bar = 1.2
gain_per_bar = 0.5
"#;
    let message = refusal(&format!("{LIQUID_PLANT}{tank_loop}"));
    assert!(
        message.contains("tank") && message.contains("level"),
        "a tank's pressure IS its level in a worse unit, and the refusal has to \
         name `variable = \"level\"` as the thing that was meant — it said: {message}"
    );

    // A source. Refused for a third reason again: its pressure is pinned by
    // declaration, so regulating it is regulating the scenario file.
    let source_loop = PI_LOOP.replace(r#"node = "receiver""#, r#"node = "header""#);
    let message = refusal(&format!("{GAS_PLANT}{source_loop}"));
    assert!(
        message.contains("pinned by declaration"),
        "a source's pressure is a boundary condition rather than a state, and the \
         refusal has to say so rather than borrowing the tank's or the junction's \
         reason — it said: {message}"
    );
}

/// **Gate 5b — fork 2's bound, and the asymmetry with the level arm.**
///
/// A vessel has no geometric analogue of a tank's height, so the bound is
/// finiteness and strict positivity and nothing more. The part worth a test is
/// the asymmetry: **`0` is refused here and legal there**. A level setpoint of
/// zero is "drain it"; a pressure setpoint of zero is a vacuum the ideal-gas
/// relation reaches only at zero mass, so a loop given one sits pinned forever.
///
/// Both halves in one test, because a bound asserted only on the side that
/// refuses is a bound that could be refusing everything.
#[test]
fn a_pressure_setpoint_must_be_positive_where_a_level_setpoint_may_be_zero() {
    let zero = PI_LOOP.replace("setpoint_bar = 20.0", "setpoint_bar = 0.0");
    let message = refusal(&format!("{GAS_PLANT}{zero}"));
    assert!(
        message.contains("vacuum") || message.contains("zero mass"),
        "a pressure setpoint of 0 is unreachable at any finite mass and must be \
         refused with that reason — it said: {message}"
    );

    let negative = PI_LOOP.replace("setpoint_bar = 20.0", "setpoint_bar = -1.0");
    let message = refusal(&format!("{GAS_PLANT}{negative}"));
    assert!(
        message.contains("absolute pressure"),
        "a negative absolute pressure must be refused — it said: {message}"
    );

    // The other side of the asymmetry: the same number, the other variable, and
    // it LOADS. Without this half the arm above is passed by a bound that refuses
    // every setpoint there is.
    let drain_at_zero = r#"
[[controls]]
name = "tank_level"
measurement = { node = "control_tank", variable = "level" }
actuator = "drain_valve"
algorithm = "p"
mode = "auto"
setpoint_m = 0.0
gain_per_m = 0.5
"#;
    let engine = engine_from(&format!("{LIQUID_PLANT}{drain_at_zero}"));
    assert_eq!(
        engine.snapshot().controls[0].setpoint,
        ControlledValue::Level { m: Meter(0.0) },
        "a level setpoint of 0 is 'drain it' and stays legal; only the pressure \
         arm refuses zero, and the asymmetry is stated at both"
    );
}

/// **Gate 5c — fork 6's cross-variable keys, all four of them.**
///
/// Neither of these was covered before M10, and the reason is the project's own
/// rule rather than an oversight: with one variable, "this key belongs to the
/// other variable" was not a state the format could reach. `deny_unknown_fields`
/// refused such a key as *unknown*, which is a different mistake and a different
/// message. So these are new work, and they are FOUR refusals rather than one —
/// two keys times two directions — because they are four different files with
/// four different mistakes in them.
#[test]
fn a_key_belonging_to_the_other_variable_is_refused_in_both_directions() {
    // Pressure loop, level keys.
    let m_on_pressure = PI_LOOP.replace(
        "setpoint_bar = 20.0",
        "setpoint_bar = 20.0\nsetpoint_m = 4.0",
    );
    let message = refusal(&format!("{GAS_PLANT}{m_on_pressure}"));
    assert!(
        message.contains("setpoint_m") && message.contains("setpoint_bar"),
        "a `setpoint_m` on a pressure loop must be refused as the LEVEL loop's key \
         and told which key was meant — it said: {message}"
    );

    let gain_m_on_pressure = PI_LOOP.replace(
        "gain_per_bar = 0.10",
        "gain_per_bar = 0.10\ngain_per_m = 0.25",
    );
    let message = refusal(&format!("{GAS_PLANT}{gain_m_on_pressure}"));
    assert!(
        message.contains("gain_per_m") && message.contains("gain_per_bar"),
        "a `gain_per_m` on a pressure loop is a separate mistake from a \
         `setpoint_m` and earns its own message — it said: {message}"
    );

    // Level loop, pressure keys. The mirror direction, which is what makes this a
    // two-directional refusal rather than a one-sided guard.
    let level_loop = r#"
[[controls]]
name = "tank_level"
measurement = { node = "control_tank", variable = "level" }
actuator = "drain_valve"
algorithm = "p"
mode = "auto"
setpoint_m = 4.0
gain_per_m = 0.25
"#;
    let bar_on_level =
        level_loop.replace("setpoint_m = 4.0", "setpoint_m = 4.0\nsetpoint_bar = 2.0");
    let message = refusal(&format!("{LIQUID_PLANT}{bar_on_level}"));
    assert!(
        message.contains("setpoint_bar") && message.contains("setpoint_m"),
        "a `setpoint_bar` on a level loop must be refused as the PRESSURE loop's \
         key — it said: {message}"
    );

    let gain_bar_on_level =
        level_loop.replace("gain_per_m = 0.25", "gain_per_m = 0.25\ngain_per_bar = 0.1");
    let message = refusal(&format!("{LIQUID_PLANT}{gain_bar_on_level}"));
    assert!(
        message.contains("gain_per_bar") && message.contains("gain_per_m"),
        "a `gain_per_bar` on a level loop earns its own message — it said: {message}"
    );
}

/// **Gate 5d — the refusal `ControlledValue`'s own doc promised.**
///
/// That type recorded, in M8.2, that with one variant "the setpoint's variable
/// disagrees with the loop's" is unrepresentable and there is deliberately no
/// guard — and that **the moment a second variant lands the refusal becomes
/// required**. This is that moment, and this test is the payment.
///
/// It is not a cosmetic guard. `ControlledValue::error` subtracts two magnitudes,
/// and the only things standing between it and subtracting metres from Pascals
/// are this refusal and the tick pass reading `measure(setpoint.variable())`. The
/// second half of the test asserts the backstop behind them: a mismatched pair
/// differences to `NaN` rather than to a plausible number.
#[test]
fn a_setpoint_of_the_wrong_variable_is_refused_and_would_not_silently_subtract() {
    let mut engine = engine_from(&format!("{GAS_PLANT}{PI_LOOP}"));
    let err = engine
        .apply(Command::SetSetpoint {
            loop_id: LoopId(0),
            value: ControlledValue::Level { m: Meter(4.0) },
        })
        .expect_err("a level setpoint on a pressure loop must be refused");
    let message = err.to_string();
    assert!(
        message.contains("measures") && message.contains("Level"),
        "the refusal has to say that a setpoint does not change what a loop \
         MEASURES, and name both variables — it said: {message}"
    );

    // The loop is untouched: a refused command must not half-apply.
    assert_eq!(
        engine.snapshot().controls[0].setpoint,
        ControlledValue::Pressure {
            pa: Pascal(20.0 * 1.0e5)
        },
        "a refused setpoint must leave the loop holding the one it had"
    );

    // The backstop. If this refusal or the tick pass's `setpoint.variable()` is
    // ever removed, the error term must not quietly return a number.
    let crossed = ControlledValue::error(
        ControlledValue::Pressure { pa: Pascal(5.0e5) },
        ControlledValue::Level { m: Meter(4.0) },
        ControlAction::Direct,
    );
    assert!(
        crossed.is_nan(),
        "differencing a pressure against a level must not produce a number: it \
         produced {crossed}, which is metres subtracted from Pascals and is the \
         shape the one-variant era's 'same type by construction' argument stopped \
         covering the moment a second variant landed"
    );
}

/// The measurement path and the SETPOINT path are separate matches, and this is
/// the one that catches them drifting apart.
///
/// `PlantGraph::measure` decides which kinds can answer for a variable;
/// `check_setpoint` decides what a reachable target is. Both match on
/// `(variable, kind)`, and a kind accepted by one and refused by the other is a
/// loop that loads and then cannot be commanded, or the reverse.
#[test]
fn the_measurement_and_the_setpoint_agree_on_which_kinds_answer_for_a_pressure() {
    let engine = engine_from(&format!("{GAS_PLANT}{PI_LOOP}"));
    let receiver = receiver_id(&engine);
    let header = engine.graph.find_node("header").expect("a header");

    assert!(
        engine
            .graph
            .measure(
                &engine.slate,
                engine.node_states(),
                None,
                MeasurementPoint::Node(receiver),
                MeasuredVariable::Pressure
            )
            .is_ok()
            && engine
                .graph
                .check_setpoint(
                    MeasurementPoint::Node(receiver),
                    ControlledValue::Pressure { pa: Pascal(15.0e5) }
                )
                .is_ok(),
        "a vessel must answer for a pressure on both paths"
    );
    assert!(
        engine
            .graph
            .measure(
                &engine.slate,
                engine.node_states(),
                None,
                MeasurementPoint::Node(header),
                MeasuredVariable::Pressure
            )
            .is_err()
            && engine
                .graph
                .check_setpoint(
                    MeasurementPoint::Node(header),
                    ControlledValue::Pressure { pa: Pascal(15.0e5) }
                )
                .is_err(),
        "a source must be refused on both paths: one accepting it would be a loop \
         that loads and cannot be commanded, or a command that outlives its load"
    );
}
