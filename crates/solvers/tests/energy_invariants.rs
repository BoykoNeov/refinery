//! Energy transport at the ENGINE level (M2 slice 1) — the thermal sibling of
//! `invariants.rs`, which does the same job for mass.
//!
//! These tests need a running `Engine`, which needs solver implementations, so
//! they live here rather than in `core`: `core` cannot depend on `solvers`
//! (CLAUDE.md rule 1), and the sweep's own contract — flows in, temperatures
//! out — is unit-tested against hand-built flow maps in `core::energy`. What
//! only an engine can show is the parts composing: solve → upwind transport →
//! thermal integration, compounding tick over tick.
//!
//!   I6. Energy conservation: for any randomly generated valid network, the
//!       change in tank thermal energy equals the enthalpy crossing the plant's
//!       reservoir boundary plus external heat, per tick.
//!
//! WHAT I6 ACTUALLY CATCHES (the same honesty the mass tests are held to). It
//! is *not* the tank update equation restated — that would be a tautology. Per
//! tank, `ΔE_i = dt·(Σ_e flux_e→i + Q_i)` is true by construction. But summing
//! over tanks only telescopes to the boundary flux if every *interior*
//! zero-volume node conserves enthalpy exactly, and that is a real claim: it
//! holds only when the mixing formula is consistent with the mass balance the
//! hydraulic solver independently produced. Get the upwind direction wrong at a
//! junction, weight the mix by anything other than ṁ·cp, or drop an edge, and
//! the interior stops cancelling and this fails. It is the exact analogue of
//! I1: the invariant lives in what the edges share, not in one node's update.
//!
//! Its accuracy floor is inherited, not intrinsic: enthalpy cancels at a
//! junction only as well as mass balances there, so I6 can never be tighter
//! than the flow solver's convergence tolerance. See `ENERGY_TOLERANCE`.
//!
//! The reference cases below are the other half — I6 pins *consistency*, and a
//! plant can be perfectly self-consistent at the wrong temperature. Each
//! reference predicts an absolute number from physics that owes nothing to the
//! code: the first law for a heated tank, and symmetry for a mixing tee.

use proptest::prelude::*;
use proptest::strategy::{Strategy, ValueTree};
use proptest::test_runner::TestRunner;
use refinery_core::components::{Composition, Slate};
use refinery_core::energy::{enthalpy_flux, T_REF};
use refinery_core::engine::{Engine, EngineConfig};
use refinery_core::graph::{Node, NodeKind, Pipe, PlantGraph, TankState};
use refinery_core::units::*;
use refinery_solvers::{ConstantThermo, NewtonFlowSolver, NoReactions};

const DT: Seconds = Seconds(0.1);
/// Water's cp at the slate's reference conditions [J/(kg·K)]. Mirrors
/// `PseudoComponent::water`; the reference cases hand-calculate against it, so
/// it is written out rather than read back from the slate — reusing the code's
/// own number on both sides of a reference test proves nothing.
const CP_WATER: f64 = 4184.0;

fn engine(graph: PlantGraph) -> Engine {
    Engine::new(
        graph,
        Slate::water_only(),
        EngineConfig { dt: DT },
        Box::new(NewtonFlowSolver::default()),
        Box::new(ConstantThermo),
        Box::new(NoReactions),
    )
}

fn node(name: &str, kind: NodeKind) -> Node {
    Node {
        name: name.into(),
        kind,
        heat_input: Watt::ZERO,
    }
}

fn source(name: &str, pressure_pa: f64, temperature: Kelvin) -> Node {
    node(
        name,
        NodeKind::Source {
            pressure: Pascal(pressure_pa),
            temperature,
            composition: Composition::pure(1, 0),
        },
    )
}

fn sink(name: &str, pressure_pa: f64, temperature: Kelvin) -> Node {
    node(
        name,
        NodeKind::Sink {
            pressure: Pascal(pressure_pa),
            temperature,
        },
    )
}

fn tank_node(name: &str, mass_kg: f64, temperature: Kelvin, heat: Watt) -> Node {
    tank_node_with_ambient(name, mass_kg, temperature, heat, WattPerKelvin::ZERO)
}

/// A tank with an ambient boundary. `tank_node` above delegates here with
/// `UA = 0` — a perfectly insulated tank — so every test written before ambient
/// exchange existed keeps the plant it was written against.
fn tank_node_with_ambient(
    name: &str,
    mass_kg: f64,
    temperature: Kelvin,
    heat: Watt,
    ambient_ua: WattPerKelvin,
) -> Node {
    Node {
        name: name.into(),
        kind: NodeKind::Tank(TankState {
            area: SquareMeter(10.0),
            height: Meter(20.0),
            mass: Kg(mass_kg),
            temperature,
            composition: Composition::pure(1, 0),
            ambient_ua,
        }),
        heat_input: heat,
    }
}

fn pipe(name: &str, length_m: f64, diameter_m: f64) -> Pipe {
    Pipe {
        name: name.into(),
        length: Meter(length_m),
        diameter: Meter(diameter_m),
        friction_factor: 0.02,
        elevation_change: Meter(0.0),
        leak_area: SquareMeter::ZERO,
        stream: refinery_core::stream::Stream::stagnant(1, T_AMBIENT, P_ATM),
    }
}

fn tank_temperature(engine: &Engine, name: &str) -> f64 {
    let id = engine.graph.find_node(name).expect("node must exist");
    match &engine.graph.node(id).kind {
        NodeKind::Tank(t) => t.temperature.value(),
        other => panic!("'{name}' is not a tank: {other:?}"),
    }
}

fn node_temperature(engine: &Engine, name: &str) -> f64 {
    let id = engine.graph.find_node(name).expect("node must exist");
    engine
        .snapshot()
        .nodes
        .into_iter()
        .find(|n| n.id == id)
        .expect("snapshot must include every node")
        .temperature_k
}

fn edge_temperature(engine: &Engine, name: &str) -> f64 {
    engine
        .snapshot()
        .edges
        .into_iter()
        .find(|e| e.name == name)
        .expect("snapshot must include every edge")
        .stream
        .temperature
        .value()
}

// ---------------------------------------------------------------------------
// Reference cases: absolute numbers predicted from physics, not from the code.
// ---------------------------------------------------------------------------

/// REFERENCE — the first law for a closed, heated vessel: with no flow,
/// `m·cp·dT/dt = Q`, so `T(t) = T₀ + Q·t/(m·cp)` exactly.
///
/// Source: any thermodynamics text; this is the definition of heat capacity.
/// It is the only M2 anchor whose expected value is *fully* independent of the
/// engine — no solver output is read, and the closed form is a straight line.
///
/// The numbers are chosen so the arithmetic is exact and checkable by eye:
/// 1000 kg of water is `m·cp` = 4.184e6 J/K, and Q = 418 400 W raises it by
/// 0.1 K/s. Over 100 ticks × 0.1 s = 10 s the tank must gain exactly 1.000 K.
///
/// Note explicit Euler is not an approximation here: with `m` constant, every
/// step adds exactly `Q·dt/(m·cp)`, so the discrete result equals the
/// continuous one and the tolerance is pure float round-off (~1e-13 measured
/// over the run; 1e-9 leaves room without admitting a real error).
#[test]
fn a_heated_tank_follows_the_first_law() {
    let mut graph = PlantGraph::new();
    // A lone tank with no pipes: nothing for the hydraulics to do, which
    // isolates the thermal inventory from transport entirely.
    graph.add_node(tank_node("kettle", 1000.0, Kelvin(293.15), Watt(418_400.0)));
    let mut engine = engine(graph);

    for tick in 1..=100 {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick} must succeed: {e:?}"));
    }

    let expected = 293.15 + 418_400.0 * 10.0 / (1000.0 * CP_WATER); // = 294.15 K
    let actual = tank_temperature(&engine, "kettle");
    assert!(
        (actual - expected).abs() < 1e-9,
        "10 s of 418.4 kW into 1000 kg of water must give exactly {expected} K, got {actual}"
    );
}

/// REFERENCE — cooling is the same law with the sign flipped. Guards against an
/// `abs()` or a sign slip in the heat term that the heating case cannot see.
#[test]
fn a_cooled_tank_follows_the_first_law() {
    let mut graph = PlantGraph::new();
    graph.add_node(tank_node(
        "chiller",
        1000.0,
        Kelvin(293.15),
        Watt(-418_400.0),
    ));
    let mut engine = engine(graph);

    for _ in 0..100 {
        engine.tick().expect("tick must succeed");
    }

    let expected = 293.15 - 1.0;
    let actual = tank_temperature(&engine, "chiller");
    assert!(
        (actual - expected).abs() < 1e-9,
        "10 s of -418.4 kW must drop 1000 kg of water to exactly {expected} K, got {actual}"
    );
}

/// FORWARD COVER — the cooling case run past the point physics allows. A tank's
/// thermal inventory is integrated in the engine, NOT through the zero-volume
/// mixing sweep, so the sub-zero guard that lives in `mix_inflows` never
/// watched this path: the only protection a tank had was the NaN/Inf check,
/// and a sub-zero Kelvin is perfectly finite.
///
/// No COMMAND can set a tank up this way: `SetHeatInput` refuses a negative
/// fire, a tank carries no duty, and mixing cannot fall below its coldest
/// inflow. The net heat sink is therefore built straight onto the field, which
/// is what ambient exchange will do to this same balance once it lands. That is
/// the condition under test — held in advance, so the term arrives guarded
/// rather than opening the hole and being caught afterwards.
///
/// The plant is `a_cooled_tank_follows_the_first_law` with the clock run on:
/// -418.4 kW drops 1000 kg of water by 0.1 K/s, so from 293.15 K it reaches
/// 0 K after 2931.5 s = 29 315 ticks and must fail on the tick after. Asking
/// for 30 000 leaves the run comfortably past the boundary.
///
/// The assertion is that it ERRS — not that it clamps at 0 K. A clamp would
/// report a plausible temperature the plant never had, which this project
/// treats as worse than a stopped simulation.
#[test]
fn a_tank_cooled_below_absolute_zero_is_rejected() {
    let mut graph = PlantGraph::new();
    graph.add_node(tank_node(
        "overcooled",
        1000.0,
        Kelvin(293.15),
        Watt(-418_400.0),
    ));
    let mut engine = engine(graph);

    let mut failed_at = None;
    for tick in 1..=30_000 {
        if let Err(e) = engine.tick() {
            failed_at = Some((tick, e));
            break;
        }
        // Until it does fail, every reported temperature must be physical.
        let t = tank_temperature(&engine, "overcooled");
        assert!(
            t >= 0.0,
            "tick {tick} reported {t} K — a sub-zero temperature escaped as a finite value"
        );
    }

    let (tick, error) = failed_at.expect(
        "cooling 1000 kg of water at 418.4 kW for 3000 s must drive it below 0 K and be rejected",
    );
    // 0 K is crossed during tick 29 316; allow a tick either side for the
    // float arithmetic rather than pinning the exact step.
    assert!(
        (29_315..=29_317).contains(&tick),
        "must fail as it crosses 0 K around tick 29 316, not before or long after; got {tick}"
    );
    let message = error.to_string();
    assert!(
        message.contains("absolute zero") && message.contains("overcooled"),
        "the error must name the tank and what went wrong, got: {message}"
    );
}

// ---------------------------------------------------------------------------
// Ambient exchange (tanks)
// ---------------------------------------------------------------------------

/// `UA` for the ambient cases [W/K]. With 1000 kg of water (`m·cp` = 4.184e6
/// J/K) and `dt` = 0.1 s this makes the per-tick decay factor
/// `α = UA·dt/(m·cp)` exactly 1e-4, which is what lets the reference below state
/// its truncation error in closed form. Physically it is an absurdly
/// well-coupled tank; the point here is arithmetic that can be checked by eye.
const AMBIENT_UA: f64 = 4184.0;
/// `α = UA·dt/(m·cp)` for the plants below — the fraction of the remaining gap
/// to ambient that one Euler tick closes.
const ALPHA: f64 = 1e-4;

/// REFERENCE — one tick of ambient exchange against the hand calculation, for a
/// tank BELOW ambient. `Q = UA·(T_amb − T)` with `T₀` 20 K below ambient is
/// +83 680 W, and over one 0.1 s tick that is `α·20 K` = exactly 2 mK of
/// WARMING.
///
/// One tick, deliberately: the discrete step is exact here (the driving force is
/// evaluated once, at the start-of-tick temperature), so the tolerance is pure
/// float round-off, with no Euler truncation to absorb. That makes this the
/// sharpest possible statement of the term's magnitude. The multi-tick case
/// below is where the integrator's error enters, and it is tested separately so
/// the two failure modes cannot be confused for one another.
#[test]
fn one_tick_of_ambient_exchange_warms_a_cold_tank_by_the_hand_calculation() {
    let mut graph = PlantGraph::new();
    graph.add_node(tank_node_with_ambient(
        "cold_store",
        1000.0,
        Kelvin(273.15), // 20 K below ambient
        Watt::ZERO,
        WattPerKelvin(AMBIENT_UA),
    ));
    let mut engine = engine(graph);
    engine.tick().expect("tick must succeed");

    let expected = 273.15 + ALPHA * 20.0; // 273.152 K
    let actual = tank_temperature(&engine, "cold_store");
    assert!(
        (actual - expected).abs() < 1e-9,
        "a tank 20 K below ambient must WARM to {expected} K in one tick, got {actual}"
    );
}

/// REFERENCE — the same tank, the same `UA`, the same tick, on the other side of
/// ambient: 20 K ABOVE it must cool by exactly the same 2 mK.
///
/// This is the test the whole "not heat loss" naming exists for (DESIGN §4a).
/// One signed `Q = UA·(T_amb − T_body)` has to run both ways, and a
/// one-directional loss — or an `abs()`, or a branch on which side is warmer —
/// passes the warming case above and fails here. The symmetry is exact because
/// the two plants are mirror images about ambient, so the expected number needs
/// no separate derivation: it is the first one's, reflected.
#[test]
fn the_same_term_cools_a_tank_above_ambient() {
    let mut graph = PlantGraph::new();
    graph.add_node(tank_node_with_ambient(
        "hot_store",
        1000.0,
        Kelvin(313.15), // 20 K above ambient
        Watt::ZERO,
        WattPerKelvin(AMBIENT_UA),
    ));
    let mut engine = engine(graph);
    engine.tick().expect("tick must succeed");

    let expected = 313.15 - ALPHA * 20.0; // 313.148 K
    let actual = tank_temperature(&engine, "hot_store");
    assert!(
        (actual - expected).abs() < 1e-9,
        "a tank 20 K above ambient must COOL to {expected} K in one tick, got {actual}"
    );
}

/// REFERENCE — Newton's law of cooling, integrated. With constant mass and no
/// flow, `m·cp·dT/dt = UA·(T_amb − T)` has the closed-form solution
///
/// ```text
/// T(t) = T_amb + (T₀ − T_amb)·exp(−UA·t/(m·cp))
/// ```
///
/// Source: Newton's law of cooling, any heat transfer text. Run for
/// `UA·t/(m·cp)` = 1 exactly — 10 000 ticks × 0.1 s = 1000 s — where the tank
/// has closed `1 − 1/e` ≈ 63.2% of its initial 20 K gap. That point is chosen
/// because it discriminates: a term off by a constant factor, or applied to the
/// wrong ΔT, lands visibly elsewhere on the curve, whereas testing near t = 0 or
/// t = ∞ would pass for almost any decaying model.
///
/// THE TOLERANCE IS EULER TRUNCATION, NOT ROUND-OFF — this is why it is 1e-3 K
/// where the single-tick cases above are 1e-9. Explicit Euler produces
/// `(1 − α)^N` where the analytic solution has `exp(−αN)`, and with `α` = 1e-4,
/// `N` = 10 000:
///
/// ```text
/// (1−α)^N = exp(N·ln(1−α)) = exp(−1 − N·α²/2 − …) ≈ e⁻¹·(1 − 5e-5)
/// error ≈ 20 K · e⁻¹ · 5e-5 ≈ 3.7e-4 K
/// ```
///
/// so 1e-3 K admits the integrator's known error with room to spare while
/// staying 4 orders of magnitude below the 12.6 K the tank actually moves.
/// Asserting against `(1 − α)^N` instead would tighten the number and destroy
/// the test: that formula IS the integrator, so it would agree with any Euler
/// implementation of any wrong `Q`.
#[test]
fn a_tank_approaches_ambient_on_newtons_law_of_cooling() {
    let mut graph = PlantGraph::new();
    graph.add_node(tank_node_with_ambient(
        "cooling_store",
        1000.0,
        Kelvin(273.15),
        Watt::ZERO,
        WattPerKelvin(AMBIENT_UA),
    ));
    let mut engine = engine(graph);

    let ambient = T_AMBIENT.value();
    let mut previous = 273.15;
    for tick in 1..=10_000 {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick} must succeed: {e:?}"));
        let now = tank_temperature(&engine, "cooling_store");
        // Approach, never arrival and never overshoot. A tank driven only by
        // ambient exchange must close the gap monotonically and stop at
        // ambient: the driving force shrinks with the gap, so crossing it would
        // mean the step overshot — the failure the PIPE transform must use the
        // analytic form to avoid, and which this tank's α ≪ 1 rules out here.
        assert!(
            now > previous && now < ambient,
            "tick {tick}: must warm monotonically toward but never past ambient \
             ({ambient} K); went {previous} → {now}"
        );
        previous = now;
    }

    let expected = ambient + (273.15 - ambient) * (-1.0f64).exp(); // ≈ 285.7924 K
    let actual = tank_temperature(&engine, "cooling_store");
    assert!(
        (actual - expected).abs() < 1e-3,
        "after one time constant the tank must sit at {expected} K \
         (63.2% of the way to ambient), got {actual}"
    );
}

/// REFERENCE — a mixing tee, predicted from SYMMETRY rather than from the
/// mixing formula: two supply legs identical in every respect except
/// temperature must, by symmetry, carry identical flows, so the mixed stream
/// sits at the plain arithmetic mean. 80 °C and 20 °C ⇒ exactly 50 °C.
///
/// This is what makes it a real reference and not a restatement of the code:
/// the expected value is fixed by the plant's symmetry before any code runs. A
/// mix weighted by the wrong quantity, or upwinded from the wrong end, lands
/// somewhere other than dead centre.
///
/// (Symmetry is exact only because M1/M2 water has constant properties — a
/// T-dependent density would perturb the two legs apart. When `ThermoModel`
/// takes over property evaluation, this test's premise needs rechecking.)
#[test]
fn symmetric_mixing_lands_on_the_arithmetic_mean() {
    let mut graph = PlantGraph::new();
    let hot = graph.add_node(source("hot", 5.0e5, Kelvin(353.15))); // 80 °C
    let cold = graph.add_node(source("cold", 5.0e5, Kelvin(293.15))); // 20 °C
    let tee = graph.add_node(node("tee", NodeKind::Junction));
    let out = graph.add_node(sink("out", 1.0e5, T_AMBIENT));

    // The two supply legs are byte-identical in geometry; only T differs.
    graph.add_pipe(hot, tee, pipe("hot_leg", 10.0, 0.1));
    graph.add_pipe(cold, tee, pipe("cold_leg", 10.0, 0.1));
    graph.add_pipe(tee, out, pipe("outlet", 10.0, 0.15));

    let mut engine = engine(graph);
    engine.tick().expect("tick must converge");

    let expected = 323.15; // 50 °C
    let mixed = node_temperature(&engine, "tee");
    assert!(
        (mixed - expected).abs() < 1e-6,
        "equal flows of 80 °C and 20 °C water must mix to {expected} K, got {mixed}"
    );
    // ...and the mixed temperature must actually be transported downstream.
    let outlet = edge_temperature(&engine, "outlet");
    assert!(
        (outlet - expected).abs() < 1e-6,
        "the outlet stream must carry the mixed {expected} K, got {outlet}"
    );

    // Non-vacuity: the legs must really be flowing. A plant where both legs sat
    // at zero would "mix" to the fallback and could pass by accident.
    let hot_flow = engine
        .snapshot()
        .edges
        .into_iter()
        .find(|e| e.name == "hot_leg")
        .expect("hot_leg must exist")
        .stream
        .mass_flow
        .value();
    assert!(
        hot_flow > 1.0,
        "the supply legs must carry real flow for the mix to mean anything, got {hot_flow} kg/s"
    );
}

/// REFERENCE — a sink driven backwards feeds the plant at ITS OWN temperature.
///
/// Two things fail if upwinding trusts the graph's edge direction instead of
/// the flow sign: the junction reports the source's 20 °C instead of the sink's
/// 80 °C, and `Sink::temperature` is dead code. The sink is at 9 bar against a
/// 1 bar source, so the flow must run backwards up both pipes.
#[test]
fn a_back_fed_sink_supplies_its_own_temperature() {
    let mut graph = PlantGraph::new();
    let feed = graph.add_node(source("feed", 1.0e5, Kelvin(293.15))); // 20 °C
    let tee = graph.add_node(node("tee", NodeKind::Junction));
    let back = graph.add_node(sink("back", 9.0e5, Kelvin(353.15))); // 80 °C

    graph.add_pipe(feed, tee, pipe("inlet", 10.0, 0.1));
    graph.add_pipe(tee, back, pipe("outlet", 10.0, 0.1));

    let mut engine = engine(graph);
    engine.tick().expect("tick must converge");

    // Non-vacuity first: the premise is that flow actually reversed.
    let outlet_flow = engine
        .snapshot()
        .edges
        .into_iter()
        .find(|e| e.name == "outlet")
        .expect("outlet must exist")
        .stream
        .mass_flow
        .value();
    assert!(
        outlet_flow < -1.0,
        "the 9 bar sink must drive flow backwards up the outlet, got {outlet_flow} kg/s"
    );

    let mixed = node_temperature(&engine, "tee");
    assert!(
        (mixed - 353.15).abs() < 1e-9,
        "the back-feeding sink must supply its own 353.15 K, got {mixed}"
    );

    // The STREAM on the inlet must carry 353.15 K too, and asserting it is not
    // redundant: node temperatures come from the sweep's own upwind logic,
    // while stream temperatures are assigned separately by the engine tick.
    // Only this line covers the latter — a flipped upwind there leaves the
    // node field above perfectly correct while every pipe reports the wrong
    // end. `inlet` is drawn feed→tee but flowing tee→feed, so its upwind is
    // `tee`; a model reading the edge's declared direction reports 293.15.
    let inlet = edge_temperature(&engine, "inlet");
    assert!(
        (inlet - 353.15).abs() < 1e-9,
        "the back-fed inlet stream must carry the upwind 353.15 K, not the 293.15 K \
         of the node its arrow points away from; got {inlet}"
    );
}

// (The matching "an isothermal plant stays isothermal" regression is in
// `scenarios/tests/isothermal_plant.rs`: it needs the reference TOML, and
// `scenarios` depends on `solvers`, so it cannot be reached from here.)

// ---------------------------------------------------------------------------
// I6 — energy conservation on random thermal networks.
// ---------------------------------------------------------------------------

/// `(supply pressures+temps, tank T, tank heat, sink pressure, sink T)`.
/// Lengths are fixed and indexed by position so any shrunk sample stays valid
/// (the same discipline `invariants.rs` follows).
type PlantInputs = (
    Vec<(f64, f64)>, // supplies: (pressure_pa, temperature_k), 2..=4 of them
    f64,             // tank temperature [K]
    f64,             // tank heat input [W]
    f64,             // sink pressure [Pa]
    f64,             // sink temperature [K]
);

fn plant_inputs_strategy() -> impl Strategy<Value = PlantInputs> {
    (
        prop::collection::vec((2.0e5..8.0e5f64, 280.0..360.0f64), 2..=4),
        280.0..360.0f64,
        -5.0e5..5.0e5f64,
        // A sink above the tank's ~1.2 bar bottom pressure back-feeds it, so
        // reverse flow through the tank leg is part of the generated space.
        1.0e5..3.0e5f64,
        280.0..360.0f64,
    )
}

/// N supplies at different temperatures → a mixing junction → a tank → a sink.
///
/// Deliberately shaped to make I6 bite: the junction is a real multi-way mix
/// (so interior enthalpy has to cancel), the tank is the only inventory (so
/// `ΔE` has one unambiguous meaning), and every temperature is independently
/// random (so nothing cancels by accident — an isothermal plant would satisfy
/// any balance, correct or not).
fn build_thermal_plant(inputs: &PlantInputs) -> PlantGraph {
    let (supplies, tank_t, tank_q, sink_p, sink_t) = inputs;
    let mut graph = PlantGraph::new();

    let tee = graph.add_node(node("tee", NodeKind::Junction));
    // Big enough that it cannot drain within the run: the mass update clamps at
    // zero, and a clamped tank has stopped conserving mass — so it would fail an
    // ENERGY balance for a reason that is nothing to do with energy.
    let tank = graph.add_node(tank_node(
        "vessel",
        100_000.0,
        Kelvin(*tank_t),
        Watt(*tank_q),
    ));
    let drain = graph.add_node(sink("drain", *sink_p, Kelvin(*sink_t)));

    for (i, (pressure, temperature)) in supplies.iter().enumerate() {
        let s = graph.add_node(source(
            &format!("supply{i}"),
            *pressure,
            Kelvin(*temperature),
        ));
        graph.add_pipe(s, tee, pipe(&format!("leg{i}"), 12.0, 0.12));
    }
    graph.add_pipe(tee, tank, pipe("fill", 15.0, 0.15));
    graph.add_pipe(tank, drain, pipe("drain_line", 15.0, 0.15));
    graph
}

/// Total thermal energy held in tanks [J], against T_REF.
fn tank_energy(engine: &Engine) -> f64 {
    engine
        .graph
        .node_ids()
        .filter_map(|id| match &engine.graph.node(id).kind {
            NodeKind::Tank(t) => {
                Some(t.mass.value() * CP_WATER * (t.temperature.value() - T_REF.value()))
            }
            _ => None,
        })
        .sum()
}

/// Net enthalpy entering the plant across its reservoir boundary [W], plus the
/// external heat applied to tanks.
///
/// Computed ONLY from reservoir-incident edges — never from the tanks' own
/// fluxes. That is the whole point: routing the accounting around the interior
/// is what forces the junction's enthalpy to cancel for the books to balance.
fn boundary_power(engine: &Engine) -> f64 {
    let mut power = 0.0;
    for id in engine.graph.node_ids() {
        let node = engine.graph.node(id);
        match node.kind {
            NodeKind::Source { .. } | NodeKind::Sink { .. } | NodeKind::Atmosphere => {
                for (edge, _other, incoming) in engine.graph.incident(id) {
                    let stream = &engine.graph.pipe(edge).stream;
                    let flow = stream.mass_flow.value();
                    let into_reservoir = if incoming { flow } else { -flow };
                    // Into the reservoir is out of the plant, hence the minus.
                    power -= enthalpy_flux(
                        KgPerSec(into_reservoir),
                        JPerKgK(CP_WATER),
                        stream.temperature,
                    )
                    .value();
                }
            }
            NodeKind::Tank(_) => power += node.heat_input.value(),
            _ => {}
        }
    }
    power
}

/// Worst |ΔE − dt·boundary_power| observed over a tick, relative to the
/// enthalpy actually moved that tick. Shared by the proptest and the
/// measurement harness that justifies the tolerance.
fn worst_relative_energy_error(engine: &mut Engine, ticks: u32) -> Option<f64> {
    let mut worst: Option<f64> = None;
    for _ in 0..ticks {
        let before = tank_energy(engine);
        if engine.tick().is_err() {
            return worst; // divergence is legal (I3); nothing to check
        }
        let expected = DT.value() * boundary_power(engine);
        let actual = tank_energy(engine) - before;

        // Scale: the enthalpy genuinely in motion this tick. A pure relative
        // error against `expected` would explode when the plant is near
        // balance (expected ≈ 0 while real enthalpy still crosses the tee).
        let scale = engine
            .graph
            .edge_ids()
            .map(|e| {
                let s = &engine.graph.pipe(e).stream;
                DT.value()
                    * enthalpy_flux(s.mass_flow, JPerKgK(CP_WATER), s.temperature)
                        .value()
                        .abs()
            })
            .fold(1.0f64, f64::max);
        let relative = (actual - expected).abs() / scale;
        worst = Some(worst.map_or(relative, |w: f64| w.max(relative)));
    }
    worst
}

/// I6 budget, relative to the enthalpy moved per tick.
///
/// Set from measurement, and the measurement is worth reading, because the
/// floor here is NOT float round-off — it is the hydraulic solver's mass
/// tolerance, inherited one-for-one.
///
/// A junction's enthalpy only cancels as exactly as its *mass* balances, and
/// Newton stops at `‖R‖ < 1e-8 + 1e-8·throughput` kg/s (`NewtonFlowSolver`
/// defaults). That leftover ε kg/s leaves the tee carrying `cp·ε·(T−T_REF)`
/// watts of unbalanced enthalpy, against a scale of `ṁ·cp·(T−T_REF)` — so the
/// relative energy error lands at ≈ ε/ṁ ≈ 1e-8, with cp and ΔT cancelling out.
/// `measure_energy_balance_headroom` observes 9.4e-9 worst case over 200
/// plants, which is that prediction confirmed to within a factor of 1.
///
/// So this budget cannot be tightened past ~1e-8 without tightening the flow
/// solver first: energy conservation rides on mass conservation and cannot be
/// better than it. 1e-6 sits ~100x above the real floor, and still ~5 orders
/// BELOW the smallest defect worth catching — dropping one leg of a 3-way tee,
/// or upwinding from the wrong end, misses by O(0.1) relative, not O(1e-7).
const ENERGY_TOLERANCE: f64 = 1e-6;

/// The tolerance's justification, executable.
///
/// A budget nobody re-measures rots: the error creeps up, the budget gets
/// nudged, and one day the test is green because it stopped asserting anything.
/// This fails the moment the real error exceeds a tenth of the budget, forcing
/// the question "what grew?" instead of "what number makes this pass?".
#[test]
fn measure_energy_balance_headroom() {
    const SAMPLES: usize = 200;
    /// The error must stay an order of magnitude clear of the budget. At the
    /// time of writing it is ~9.4e-9 against this 1e-7 trigger.
    const HEADROOM_TRIGGER: f64 = ENERGY_TOLERANCE / 10.0;

    let mut runner = TestRunner::deterministic();
    let strategy = plant_inputs_strategy();

    let mut worst = 0.0f64;
    let mut checked = 0usize;
    for _ in 0..SAMPLES {
        let inputs = strategy
            .new_tree(&mut runner)
            .expect("strategy produces a value")
            .current();
        let mut e = engine(build_thermal_plant(&inputs));
        if let Some(w) = worst_relative_energy_error(&mut e, 5) {
            worst = worst.max(w);
            checked += 1;
        }
    }

    assert!(checked > SAMPLES / 2, "only {checked}/{SAMPLES} plants ran");
    assert!(
        worst < HEADROOM_TRIGGER,
        "energy balance error {worst:e} has grown past a tenth of the {ENERGY_TOLERANCE:e} \
         budget. Expected ~9.4e-9, the flow solver's mass tolerance showing through. Find \
         what grew before relaxing the budget — if the solver's tolerance did not change, \
         this is a real transport bug, not noise."
    );
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(200))]

    /// I6 — tank energy changes by exactly the enthalpy crossing the plant
    /// boundary plus external heat. See the module docs for why the interior
    /// cancellation makes this a real claim rather than the update restated.
    #[test]
    fn thermal_plant_conserves_energy(inputs in plant_inputs_strategy()) {
        let mut e = engine(build_thermal_plant(&inputs));
        if let Some(worst) = worst_relative_energy_error(&mut e, 5) {
            prop_assert!(
                worst <= ENERGY_TOLERANCE,
                "energy imbalance {worst:e} exceeds the {ENERGY_TOLERANCE:e} budget"
            );
        }
    }

    /// I2/I3 for the thermal path: temperatures stay finite and physical. A
    /// mixed temperature must land within the range of what feeds it — mixing
    /// can never produce a stream hotter than its hottest input, and the
    /// generated bounds are 280..360 K.
    #[test]
    fn temperatures_stay_within_their_inputs(inputs in plant_inputs_strategy()) {
        // Heat input would legitimately push a tank outside its feed range, so
        // it is zeroed here: this bound is a claim about TRANSPORT alone.
        let (supplies, tank_t, _, sink_p, sink_t) = inputs;
        let mut e = engine(build_thermal_plant(&(supplies, tank_t, 0.0, sink_p, sink_t)));
        for _ in 0..5 {
            if e.tick().is_err() {
                return Ok(()); // divergence is legal (I3)
            }
            for edge in e.snapshot().edges {
                let t = edge.stream.temperature.value();
                prop_assert!(t.is_finite(), "stream '{}' went non-finite", edge.name);
                prop_assert!(
                    (279.0..=361.0).contains(&t),
                    "stream '{}' at {t} K is outside the 280..360 K its feeds span — \
                     transport invented energy",
                    edge.name
                );
            }
        }
    }
}
