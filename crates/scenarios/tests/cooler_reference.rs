//! The cooler, as wired: `scenarios/cooler_chiller.toml` end to end (M2.2).
//!
//! Structurally the `Cooler` is the `Furnace`'s mirror, and `furnace_reference.rs`
//! already pins everything the two share — the loader's MW→W conversion path,
//! the duty surviving a real hydraulic solve, the mixing formula, linearity in
//! duty, the flat line at duty 0. Repeating those against a cooler would test
//! the same code twice.
//!
//! What is genuinely NEW here, and all this file covers:
//!   1. the SIGN — `heat_load` must SUBTRACT a cooler's duty, and nothing else
//!      in the workspace has an opinion about which way heat moves,
//!   2. `heat_input` and duty OPPOSING each other rather than merely summing,
//!   3. the absolute-zero guard in `mix_inflows`, which is unreachable from the
//!      proptest generators (coolers are excluded from them for the same
//!      stagnant-node reason furnaces are) and so has no other coverage at all,
//!   4. negative duty being refused at both entry points, which is what makes
//!      "two units, each with a positive magnitude" safe rather than a
//!      convention nothing enforces,
//!   5. the same refusal for a negative `heat_input` — no cooling fires, so
//!      every net heat sink belongs to a unit that declares itself one.
//!
//! Lives in `scenarios/` for the same reason as `furnace_reference.rs`: it needs
//! the TOML and its loader, and `scenarios` depends on `solvers`, so the reverse
//! import would be a dependency cycle.

use refinery_core::engine::Engine;
use refinery_core::error::SimError;
use refinery_core::snapshot::{Command, EdgeSnapshot};
use refinery_core::units::Watt;
use refinery_scenarios::{NodeDef, ScenarioFile};

const SCENARIO: &str = include_str!("../../../scenarios/cooler_chiller.toml");

/// The feed temperature the TOML declares, in SI (80 °C).
const FEED_K: f64 = 353.15;

/// The duty the TOML declares, in SI. Written in WATTS while the file says
/// `duty_mw = 1.0`, for the same reason as in `furnace_reference.rs`: deriving
/// it from the loader's own conversion would make the test agree with any bug
/// in that conversion.
const DUTY_W: f64 = 1.0e6;

const TICKS: u64 = 20;
const TOLERANCE_K: f64 = 1e-9;

/// Parse the scenario with the chiller's duty overridden [MW].
fn scenario_with(duty_mw: f64) -> ScenarioFile {
    let mut file: ScenarioFile =
        refinery_scenarios::load_str(SCENARIO).expect("the cooler scenario must parse");
    match file
        .nodes
        .get_mut("chiller")
        .expect("the scenario must define a 'chiller' node")
    {
        NodeDef::Cooler { duty_mw: d } => *d = duty_mw,
        other => panic!("'chiller' must be a cooler, got {other:?}"),
    }
    file
}

fn build(duty_mw: f64) -> Engine {
    refinery_scenarios::build_engine(&scenario_with(duty_mw)).expect("the cooler plant must build")
}

fn run(engine: &mut Engine) -> Result<(), SimError> {
    for _ in 1..=TICKS {
        engine.tick()?;
    }
    Ok(())
}

fn edge(engine: &Engine, name: &str) -> EdgeSnapshot {
    engine
        .snapshot()
        .edges
        .into_iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("the cooler plant must have a '{name}' pipe"))
}

/// First law across the cooler, with the sign that makes it a cooler:
/// `T_out = T_in − Q/(ṁ·cp)`.
///
/// The whole point is the minus. A `heat_load` that returned `+duty` for a
/// `Cooler` — the single-character slip this unit invites — produces an equally
/// well-formed plant that is exactly as wrong as it is plausible, and only a
/// signed absolute check catches it. Comparing magnitudes, or asserting the
/// outlet merely "differs from" the inlet, would not.
#[test]
fn the_cooler_removes_its_duty_from_the_stream() {
    let mut engine = build(1.0);
    run(&mut engine).expect("the cooler plant must run");
    let inlet = edge(&engine, "feed_line");
    let outlet = edge(&engine, "transfer_line");

    let mass_flow = inlet.stream.mass_flow.value();
    assert!(
        mass_flow > 1.0,
        "the plant must actually be flowing for the duty to land anywhere, got {mass_flow} kg/s"
    );
    let cp = inlet.stream.composition.mixture_cp(&engine.slate).value();
    let expected = FEED_K - DUTY_W / (mass_flow * cp);

    assert!(
        (outlet.stream.temperature.value() - expected).abs() < TOLERANCE_K,
        "outlet stream must leave at {expected} K (= {FEED_K} − {DUTY_W}/({mass_flow}·{cp})), \
         got {} — if it came out ABOVE the feed, the cooler is heating",
        outlet.stream.temperature.value()
    );
    assert!(
        outlet.stream.temperature.value() < FEED_K,
        "a cooler must lower the temperature; got {} K against a {FEED_K} K feed",
        outlet.stream.temperature.value()
    );
}

/// A fire on a cooler FIGHTS it: Q in against Q out leaves the stream untouched.
///
/// The counterpart to `furnace_reference.rs`'s fire-stacking gate, and a
/// stronger one, because it lands on a flat line rather than a difference. Every
/// gate that measures a ΔT passes under a constant offset; this one does not. It
/// holds only if `heat_load` sums `heat_input − duty` with genuinely opposite
/// signs — a cooler whose duty ADDED would show 2Q of heating here, and one
/// whose duty REPLACED `heat_input` would show Q of it.
#[test]
fn a_fire_on_a_cooler_cancels_its_duty() {
    let mut engine = build(1.0);
    let chiller = engine
        .graph
        .find_node("chiller")
        .expect("the scenario must define a 'chiller' node");

    // Establish that the duty alone does something, or the cancellation below is
    // vacuous: 0 K cancels 0 K under any mutation that drops the duty entirely.
    let mut cooling_only = build(1.0);
    run(&mut cooling_only).expect("the cooler plant must run");
    let drop_from_duty = FEED_K
        - edge(&cooling_only, "transfer_line")
            .stream
            .temperature
            .value();
    assert!(
        drop_from_duty > 1.0,
        "the duty alone must produce a real drop for the cancellation to mean \
         anything, got {drop_from_duty} K"
    );

    engine
        .apply(Command::SetHeatInput {
            node: chiller,
            power: Watt(DUTY_W),
        })
        .expect("a fire on a cooler is a valid command");
    run(&mut engine).expect("the cooler plant must run with a fire on it");

    let outlet = edge(&engine, "transfer_line").stream.temperature.value();
    assert!(
        (outlet - FEED_K).abs() < TOLERANCE_K,
        "a fire of Q on a cooler removing Q must leave the stream at its {FEED_K} K \
         feed temperature, got {outlet} K"
    );
}

/// A fire is a heat SOURCE; a negative one is refused, not applied.
///
/// The companion to the duty check below, and the same argument: `heat_input` is
/// the damage model's hook, and there is no damage that chills a unit. A cooling
/// fire would be a second, undeclared way to spend heat — one that bypasses the
/// `Cooler` the model added for exactly that job, and that could push a node's
/// balance to a place no plant reaches. Refusing it keeps every net heat sink
/// the property of a unit that declares itself one.
///
/// Zero is asserted legal in the same breath, because "the fire is out" must
/// remain expressible: a guard written `<= 0` would pass a test that only
/// checked the negative case, and would then refuse to extinguish a fire.
#[test]
fn a_cooling_fire_is_refused_but_no_fire_is_allowed() {
    let mut engine = build(1.0);
    let chiller = engine.graph.find_node("chiller").expect("a 'chiller' node");

    let error = engine
        .apply(Command::SetHeatInput {
            node: chiller,
            power: Watt(-DUTY_W),
        })
        .expect_err("a negative heat input must be refused");
    assert!(
        matches!(error, SimError::InvalidCommand(_)),
        "a cooling fire is an invalid command, not a numerical failure: got {error:?}"
    );

    engine
        .apply(Command::SetHeatInput {
            node: chiller,
            power: Watt::ZERO,
        })
        .expect("extinguishing a fire must stay legal");

    // The refusal must not have half-applied: the plant still cools by exactly
    // its scenario duty, with no leftover heat term either way.
    run(&mut engine).expect("the cooler plant must still run");
    let drop = FEED_K - edge(&engine, "transfer_line").stream.temperature.value();
    let cooling_only = {
        let mut e = build(1.0);
        run(&mut e).expect("the cooler plant must run");
        FEED_K - edge(&e, "transfer_line").stream.temperature.value()
    };
    assert!(
        (drop - cooling_only).abs() < TOLERANCE_K,
        "a refused fire must leave the plant exactly as it was: expected a \
         {cooling_only} K drop, got {drop} K"
    );
}

/// Cooling past absolute zero is an error, not a number.
///
/// The stream carries ~82 MW of sensible heat above 0 K at this flow, so 200 MW
/// asks for a temperature physics does not have. The result would be FINITE —
/// no NaN, no Inf — so every existing guard in the engine waves it through and a
/// negative Kelvin propagates downstream as an ordinary temperature.
///
/// This gate is the only coverage the check has: coolers are kept out of the
/// proptest generators (a stagnant zero-volume node drops its duty, a documented
/// model gap — see `mix_inflows`), so no invariant can reach it.
#[test]
fn cooling_below_absolute_zero_is_rejected() {
    let mut engine = build(200.0);
    let error = run(&mut engine).expect_err("cooling past 0 K must be an error, not a number");

    assert!(
        matches!(error, SimError::Numerical(_)),
        "over-cooling is a numerical failure, not a bad command: got {error:?}"
    );
    let message = error.to_string();
    for expected in ["chiller", "absolute zero"] {
        assert!(
            message.contains(expected),
            "the error must say what happened and where ('{expected}'), got: {message}"
        );
    }
}

/// Duty is a magnitude at both entry points; negative is refused, not applied.
///
/// This is what makes the two-unit design a guarantee rather than a convention.
/// With a `Cooler` in the model there is nothing left for a negative furnace
/// duty to express, so one can only be a sign slip — and a furnace that quietly
/// chills is precisely the plausible-looking wrong plant the split exists to
/// prevent. Both the loader and the commands are checked, because a scenario
/// file and a frontend are independent ways in.
#[test]
fn negative_duty_is_refused_at_every_entry_point() {
    // (a) At load.
    // `Engine` is not `Debug` (it holds boxed solver traits), so unwrap the
    // Result by hand rather than with `expect_err`.
    let error = match refinery_scenarios::build_engine(&scenario_with(-1.0)) {
        Ok(_) => panic!("a negative duty_mw must not build"),
        Err(e) => e,
    };
    assert!(
        matches!(error, SimError::Scenario(_)),
        "a bad scenario number must fail as a Scenario error, got {error:?}"
    );
    assert!(
        error.to_string().contains("cooler"),
        "the error must name the offending unit, got: {error}"
    );

    // (b) At the command boundary. Only `SetCoolerDuty` is checked here, and
    //     that restraint is the point: sending `SetFurnaceDuty` to this plant's
    //     chiller would look like extra coverage and be worth nothing, since the
    //     wrong-kind arm returns `InvalidCommand` whether or not the duty was
    //     ever range-checked. The furnace side is exercised against a real
    //     furnace in `furnace_reference.rs`.
    let mut engine = build(1.0);
    let chiller = engine.graph.find_node("chiller").expect("a 'chiller' node");
    let error = engine
        .apply(Command::SetCoolerDuty {
            node: chiller,
            duty: Watt(-1.0),
        })
        .expect_err("a negative duty setpoint must be refused");
    assert!(
        matches!(error, SimError::InvalidCommand(_)),
        "a bad setpoint is an invalid command, got {error:?}"
    );

    // And the refusal must have left the setpoint alone rather than half-applied
    // it: the plant still cools by exactly its scenario duty.
    run(&mut engine).expect("the cooler plant must still run");
    let drop = FEED_K - edge(&engine, "transfer_line").stream.temperature.value();
    assert!(
        drop > 1.0,
        "the rejected commands must not have disturbed the 1 MW setpoint, got a \
         {drop} K drop"
    );
}
