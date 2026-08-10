//! Ambient heat exchange along a PIPE, at the engine level (M2.2).
//!
//! The transform itself — `energy::pipe_outlet_temperature` — is pinned against
//! a hand computation in `core::energy`'s unit tests, where the mass flow can be
//! chosen to make the exponent exactly 1. These tests do the part that only a
//! running engine can: prove the transform is WIRED IN, at the right point in
//! the tick, on the right end of each edge.
//!
//! That distinction matters because the obvious engine-level assertion is a
//! tautology. With `ṁ` read back from the snapshot, "expected" would be the
//! same exponential the engine just evaluated, and the test would pass for a
//! flipped sign, a wrong `cp`, or an Euler step. So the physics here is checked
//! against the **log-mean temperature difference** instead:
//!
//! ```text
//! ṁ·cp·(T_in − T_out)  ==  UA · LMTD,
//!   LMTD = ((T_in − T_amb) − (T_out − T_amb)) / ln((T_in − T_amb)/(T_out − T_amb))
//! ```
//!
//! which is the standard heat-exchanger design equation, derived independently
//! of the exponential form and equal to it only if the transform is right. It is
//! an identity, not an approximation: substituting `T_out − T_amb = ΔT_in·e⁻ᴺ`
//! with `N = UA/(ṁ·cp)` collapses `UA·LMTD` to `ṁ·cp·ΔT_in·(1 − e⁻ᴺ)` exactly.
//! An Euler step, a signed `ṁ`, or a `C_max`-style capacity slip all break it.
//!
//! **M5.1 generalized that log-mean rather than loosening it.** A pipe now also
//! dissipates friction into its own stream, so its profile decays toward
//! `T* = T_ambient + Φ/UA` — where the friction it makes balances the heat it
//! sheds — instead of toward ambient. The identity survives verbatim with `T_amb`
//! replaced by `T*`, because the `Φ` the source adds and the `Φ` that
//! integrating `UA·(T − T_amb)` over the offset produces cancel exactly. It is
//! still checked at 1e-12 relative, and it still degenerates to the line above
//! when `Φ = 0`. Adding slack for the new term instead would have left the test
//! passing for a transform that got the coupling wrong.
//!
//! Deliberately NOT covered here: I6. A pipe with a nonzero `UA` breaks that
//! invariant's telescoping sum by construction — the enthalpy leaving one node
//! is not the enthalpy arriving at the next, and the difference went to ambient,
//! which sits on no node. `energy_invariants.rs` keeps its generated pipes at
//! `UA = 0` for that reason, and the closure case below is what covers the same
//! ground for a single pipe until an ambient term is added to the invariant.
//! (Its `Φ` term, by contrast, IS in I6's budget — friction is sensible heat the
//! model computes exactly, so it is accounted rather than excluded.)

use refinery_core::components::{Composition, Slate};
use refinery_core::engine::{Engine, EngineConfig};
use refinery_core::graph::{LeakRole, Node, NodeKind, Pipe, PlantGraph, TankState};
use refinery_core::units::*;
use refinery_solvers::{ConstantThermo, NewtonFlowSolver, NoReactions};

const DT: Seconds = Seconds(0.1);
/// Water's cp [J/(kg·K)], written out rather than read back from the slate: a
/// reference that sources both sides from the code proves only self-consistency.
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
            composition: Composition::pure(1, 0),
        },
    )
}

fn tank_node(name: &str, mass_kg: f64, temperature: Kelvin) -> Node {
    node(
        name,
        NodeKind::Tank(TankState {
            area: SquareMeter(10.0),
            height: Meter(20.0),
            mass: Kg(mass_kg),
            temperature,
            composition: Composition::pure(1, 0),
            ambient_ua: WattPerKelvin::ZERO,
        }),
    )
}

/// A perfectly insulated pipe — the default, and what every test written before
/// this box existed was implicitly using.
fn pipe(name: &str, length_m: f64, diameter_m: f64) -> Pipe {
    pipe_with_ambient(name, length_m, diameter_m, WattPerKelvin::ZERO)
}

fn pipe_with_ambient(
    name: &str,
    length_m: f64,
    diameter_m: f64,
    ambient_ua: WattPerKelvin,
) -> Pipe {
    Pipe {
        name: name.into(),
        length: Meter(length_m),
        diameter: Meter(diameter_m),
        friction_factor: 0.02,
        elevation_change: Meter(0.0),
        leak: LeakRole::None,
        ambient_ua,
        stream: refinery_core::stream::Stream::stagnant(1, T_AMBIENT, P_ATM),
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

fn edge_flow(engine: &Engine, name: &str) -> f64 {
    engine
        .snapshot()
        .edges
        .into_iter()
        .find(|e| e.name == name)
        .expect("snapshot must include every edge")
        .stream
        .mass_flow
        .value()
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

/// The friction power the solve booked into an edge's stream [W].
fn edge_dissipation(engine: &Engine, name: &str) -> f64 {
    engine
        .snapshot()
        .edges
        .into_iter()
        .find(|e| e.name == name)
        .expect("snapshot must include every edge")
        .dissipation_w
}

// ---------------------------------------------------------------------------

/// REFERENCE — the enthalpy a pipe loses equals the heat it gives to ambient,
/// cross-checked against the log-mean form (see the module docs for why that is
/// an independent statement rather than a restatement).
///
/// Plant: a hot source feeds a junction through a pipe with a real `UA`. The
/// junction is zero-volume, so its resolved temperature IS the pipe's outlet
/// with nothing else mixed in.
#[test]
fn the_enthalpy_a_pipe_loses_equals_the_heat_it_gives_to_ambient() {
    const UA: f64 = 8000.0; // [W/K]
    let inlet_t = 373.15; // 100 °C, 80 K above ambient

    let mut graph = PlantGraph::new();
    let hot = graph.add_node(source("hot", 5.0e5, Kelvin(inlet_t)));
    let outlet = graph.add_node(node("outlet", NodeKind::Junction));
    let drain = graph.add_node(sink("drain", 1.0e5, T_AMBIENT));

    graph.add_pipe(
        hot,
        outlet,
        pipe_with_ambient("hot_line", 10.0, 0.1, WattPerKelvin(UA)),
    );
    graph.add_pipe(outlet, drain, pipe("drain_line", 10.0, 0.1));

    let mut engine = engine(graph);
    engine.tick().expect("tick must converge");

    let flow = edge_flow(&engine, "hot_line");
    let outlet_t = node_temperature(&engine, "outlet");

    // Non-vacuity, both halves. Without real flow the LMTD is undefined and the
    // identity below is 0 == 0; without a real temperature DROP the transform
    // could be the identity and still pass.
    assert!(
        flow > 0.1,
        "the pipe must carry real flow for this to mean anything, got {flow} kg/s"
    );
    assert!(
        inlet_t - outlet_t > 1.0,
        "the pipe must actually cool the stream; inlet {inlet_t} K, outlet {outlet_t} K"
    );
    // ...and it must cool TOWARD ambient, not past it. A sign slip that landed
    // below ambient would still show a "drop".
    assert!(
        outlet_t > T_AMBIENT.value(),
        "a stream above ambient must stay above it, got {outlet_t} K"
    );

    let duty_from_enthalpy = flow * CP_WATER * (inlet_t - outlet_t); // [W]

    // The log-mean is taken about the pipe's ASYMPTOTE, not about ambient.
    //
    // Since M5.1 the pipe also dissipates friction into its own stream, so the
    // profile decays toward `T* = T_ambient + Φ/UA` — the temperature at which
    // the friction it generates exactly balances the heat it sheds — rather than
    // toward ambient. Substituting `T(ξ) − T* = (T_in − T*)·e^{−βξ}` into the
    // energy balance leaves
    //
    //     ṁ·cp·(T_in − T_out) = UA·logmean(T_in − T*, T_out − T*)
    //
    // where the `−Φ` from the friction source and the `+Φ` from integrating
    // `UA·(T − T_ambient)` over the offset cancel identically. So this is STILL an
    // identity and still checked at round-off — it is not the old statement with
    // slack added for a new term, which would have quietly stopped discriminating.
    // At `Φ = 0` it degenerates to the pure-ambient LMTD it replaces.
    let dissipation = edge_dissipation(&engine, "hot_line");
    let asymptote = T_AMBIENT.value() + dissipation / UA;
    let delta_in = inlet_t - asymptote;
    let delta_out = outlet_t - asymptote;
    let lmtd = (delta_in - delta_out) / (delta_in / delta_out).ln();
    let duty_from_lmtd = UA * lmtd; // [W]

    // Non-vacuity for the generalization: if Φ were zero this would be the old
    // test, and the new algebra would be untested.
    assert!(
        dissipation > 0.0,
        "the pipe must actually dissipate for the T* form to be under test, got \
         {dissipation} W"
    );

    // Round-off, not a physical tolerance: the two forms are algebraically
    // identical, so the only difference is float evaluation order. Relative,
    // because the duty is ~1e5 W and an absolute 1e-9 would be stricter than
    // f64 can express at that magnitude.
    let relative = (duty_from_enthalpy - duty_from_lmtd).abs() / duty_from_lmtd.abs();
    assert!(
        relative < 1e-12,
        "the enthalpy drop ({duty_from_enthalpy:.6} W) must equal UA·logmean about \
         T* = {asymptote:.6} K ({duty_from_lmtd:.6} W); relative difference {relative:.3e}"
    );
}

/// GATE — the transform runs INSIDE the sweep, not after it.
///
/// The discriminating plant is a `UA` pipe between two ZERO-VOLUME nodes. The
/// downstream junction's temperature is resolved by mixing its inflows during
/// the sweep, so if the mix reads the raw upwind node temperature instead of the
/// transformed edge outlet, `after` reports the same value as `before` and the
/// pipe's `UA` does nothing at all.
///
/// A pipe feeding a TANK cannot prove this: the tank integrates over the tick
/// and its temperature barely moves either way, so the ordering error would hide
/// inside a small number. This is the analogue of the exchanger's pair-merge
/// ordering test, and it is the gate the "raw upwind" mutation must fail.
#[test]
fn a_ua_pipe_between_two_zero_volume_nodes_cools_the_downstream_one() {
    let mut graph = PlantGraph::new();
    let hot = graph.add_node(source("hot", 5.0e5, Kelvin(373.15)));
    let before = graph.add_node(node("before", NodeKind::Junction));
    let after = graph.add_node(node("after", NodeKind::Junction));
    let drain = graph.add_node(sink("drain", 1.0e5, T_AMBIENT));

    graph.add_pipe(hot, before, pipe("feed", 10.0, 0.1));
    graph.add_pipe(
        before,
        after,
        pipe_with_ambient("lagged_run", 10.0, 0.1, WattPerKelvin(8000.0)),
    );
    graph.add_pipe(after, drain, pipe("drain_line", 10.0, 0.1));

    let mut engine = engine(graph);
    engine.tick().expect("tick must converge");

    let upstream = node_temperature(&engine, "before");
    let downstream = node_temperature(&engine, "after");

    assert!(
        upstream - downstream > 1.0,
        "the downstream junction must see the pipe's transformed OUTLET, not its \
         upwind node's temperature — got {upstream} K before and {downstream} K \
         after, a difference of {:.6} K. If these are equal the transform is \
         running after the mixing that consumes it, i.e. never.",
        upstream - downstream
    );
    assert!(
        downstream > T_AMBIENT.value(),
        "the downstream junction must stay above ambient, got {downstream} K"
    );

    // The insulated feed pipe must NOT have COOLED anything — otherwise this test
    // would pass on a plant that cooled every pipe regardless of `UA`. It does
    // warm it very slightly, by its own friction, which is a different term with
    // the opposite sign and its own gates; the discrimination this line needs is
    // the direction, and a couple of hundredths of a kelvin up is unmistakably
    // not the ~10 K down the lagged run produces.
    assert!(
        upstream > 373.15 && upstream - 373.15 < 0.5,
        "the UA = 0 feed pipe must not cool its stream (it may warm a little from \
         friction), got {upstream} K"
    );
}

/// GATE — a tank is debited at ITS OWN temperature, not its outflow pipe's
/// outlet.
///
/// Fluid leaves a tank at the tank's temperature and cools afterwards, IN the
/// pipe. The heat the pipe then trades with ambient is not the tank's to lose.
/// Reading the tank's outflow edge at the outlet temperature would debit the
/// tank for it — invisible while every `UA` is 0, and a silent enthalpy error
/// the moment one is not, which is exactly why it is worth a gate.
///
/// A pure-outflow tank is the sharpest possible probe, because the correct
/// answer is a temperature that does not move AT ALL: draining a well-mixed
/// vessel removes mass and enthalpy in exactly the proportion that leaves `T`
/// unchanged. Debit at the pipe's (cooler) outlet instead and less enthalpy
/// leaves than the mass warrants, so the tank *heats up* while draining — a
/// wrong sign in an obvious direction rather than a small numerical shift.
#[test]
fn a_tank_is_debited_at_its_own_temperature_not_its_outflow_pipes_outlet() {
    /// One tick of a tank draining through a pipe with the given `UA`, as
    /// `(tank temperature [K], tank mass [kg], drain flow [kg/s])`.
    fn drain_tank_through(ua: f64) -> (f64, f64, f64) {
        let mut graph = PlantGraph::new();
        let tank = graph.add_node(tank_node("kettle", 1000.0, Kelvin(373.15)));
        let drain = graph.add_node(sink("drain", 1.0e5, T_AMBIENT));
        graph.add_pipe(
            tank,
            drain,
            pipe_with_ambient("drain_line", 10.0, 0.1, WattPerKelvin(ua)),
        );

        let mut engine = engine(graph);
        engine.tick().expect("tick must converge");
        // The TANK'S OWN integrated state, not `node_temperature`. The snapshot's
        // node temperature comes from the sweep's start-of-tick field, which for
        // a tank is a boundary condition — it does not move within the tick, so
        // reading it here would report 373.15 K whatever the balance did and the
        // test would pass under any mutation. (It did, before this was fixed.)
        let (temperature, mass) = match &engine.graph.node(tank).kind {
            NodeKind::Tank(t) => (t.temperature.value(), t.mass.value()),
            _ => unreachable!("kettle is a tank"),
        };
        (temperature, mass, edge_flow(&engine, "drain_line"))
    }

    let (insulated, insulated_mass, flow) = drain_tank_through(0.0);
    let (lagged, lagged_mass, _) = drain_tank_through(50_000.0);

    // Non-vacuity. "Unchanged" is trivially true of a plant doing nothing, so
    // the tank must really be exporting mass — and the pipe must really be
    // cooling, or the two cases are the same plant twice.
    assert!(
        flow > 0.1,
        "the tank must actually be draining for the debit to be exercised, got {flow} kg/s"
    );
    assert!(
        insulated_mass < 1000.0,
        "the tank's inventory must have fallen, got {insulated_mass} kg"
    );
    assert_eq!(
        insulated_mass, lagged_mass,
        "the hydraulics must be identical in both cases — only the pipe's UA differs"
    );

    // The physics: pure outflow at the tank's own temperature is isothermal.
    assert!(
        (insulated - 373.15).abs() < 1e-9,
        "draining a well-mixed tank removes mass and enthalpy in the proportion \
         that leaves T unchanged, got {insulated} K"
    );
    assert_eq!(
        insulated, lagged,
        "a tank's temperature must not depend on how much heat its OUTFLOW pipe \
         loses downstream — the fluid left at the tank's temperature. Insulated \
         {insulated} K vs lagged {lagged} K. A tank that HEATS here is being \
         debited at the pipe's outlet."
    );
}

/// The other half of that matrix: a tank on the DOWNSTREAM end of a `UA` pipe,
/// which must be credited at the pipe's cooled OUTLET rather than at its
/// upstream node's temperature.
///
/// Worth its own case because no other test reaches this branch, and no mutation
/// in this file's falsification set can. Debiting-at-outlet — the mutation the
/// test above catches — is a NO-OP on an inflow edge, since `stream.temperature`
/// already holds the transformed value there. So the transformed-inflow path
/// into a tank was correct only by construction until this asserted it: the
/// helper's downstream branch is pinned by the junction cases, and the tank
/// loop's argument passing by the outflow case, but the two composing was not.
///
/// A tank is a *heat capacity*, so the assertion is on the direction and
/// magnitude of one tick's warming rather than an absolute temperature: the
/// tank must warm toward the pipe's outlet, and by strictly LESS than if the
/// pipe had delivered its inlet temperature undiminished.
#[test]
fn a_tank_fed_through_a_ua_pipe_is_credited_at_the_pipes_outlet() {
    /// One tick of a hot source filling a cold tank through a pipe with the
    /// given `UA`, as `(tank temperature [K], pipe outlet [K])`.
    fn fill_tank_through(ua: f64) -> (f64, f64) {
        let mut graph = PlantGraph::new();
        let hot = graph.add_node(source("hot", 5.0e5, Kelvin(373.15)));
        let tank = graph.add_node(tank_node("receiver", 1000.0, T_AMBIENT));
        graph.add_pipe(
            hot,
            tank,
            pipe_with_ambient("fill_line", 10.0, 0.1, WattPerKelvin(ua)),
        );

        let mut engine = engine(graph);
        engine.tick().expect("tick must converge");
        let temperature = match &engine.graph.node(tank).kind {
            NodeKind::Tank(t) => t.temperature.value(),
            _ => unreachable!("receiver is a tank"),
        };
        (temperature, edge_temperature(&engine, "fill_line"))
    }

    let (insulated, insulated_outlet) = fill_tank_through(0.0);
    let (lagged, lagged_outlet) = fill_tank_through(40_000.0);

    // Non-vacuity: the insulated case must deliver essentially the source's own
    // temperature (its own friction warms it by a few hundredths of a kelvin —
    // a different term, pinned elsewhere, and two orders below the ~10 K the
    // lagging removes), and the lagged case must actually have cooled on the way.
    assert!(
        insulated_outlet > 373.15 && insulated_outlet - 373.15 < 0.5,
        "the UA = 0 pipe must deliver the source's 373.15 K, give or take its own \
         friction, got {insulated_outlet} K"
    );
    assert!(
        insulated_outlet - lagged_outlet > 1.0,
        "the lagged pipe must arrive cooler; {insulated_outlet} K vs {lagged_outlet} K"
    );

    // Both tanks warm — they are fed above ambient either way.
    assert!(
        lagged > T_AMBIENT.value(),
        "a tank fed above ambient must warm, got {lagged} K"
    );
    // ...but the lagged one warms strictly less, because it is credited at the
    // outlet. Reading the raw upstream node instead would make these EQUAL: the
    // pipe's UA would do nothing to the tank at all.
    assert!(
        lagged < insulated,
        "a tank fed through a lagged pipe must warm less than through an insulated \
         one — it is credited at the pipe's OUTLET, not its inlet. Got {lagged} K \
         lagged vs {insulated} K insulated; equal values mean the tank is reading \
         the raw upstream temperature."
    );
}

/// The zero-flow guard, reached the way a plant actually reaches it: a closed
/// valve. Without the guard the `0/0` in the exponent produces a NaN that no
/// check attributes to a pipe.
///
/// The pipes here are DEFAULT (`UA = 0`), which is the point — `ṁ = 0` with a
/// nonzero `UA` is already finite on its own (`−∞ → exp = 0`), so only the
/// default case tests the guard. Removing the guard must fail this test.
#[test]
fn a_closed_valve_leaves_finite_temperatures() {
    let mut graph = PlantGraph::new();
    let supply = graph.add_node(source("supply", 5.0e5, Kelvin(373.15)));
    let valve = graph.add_node(node(
        "shut",
        NodeKind::Valve {
            cv_max: 1e-3,
            opening: 0.0,
            x_t: None,
        },
    ));
    let drain = graph.add_node(sink("drain", 1.0e5, T_AMBIENT));

    graph.add_pipe(supply, valve, pipe("inlet", 10.0, 0.1));
    graph.add_pipe(valve, drain, pipe("outlet", 10.0, 0.1));

    let mut engine = engine(graph);
    engine
        .tick()
        .expect("a closed valve must not break the tick");

    // Non-vacuity: the premise is that flow really is zero. If the valve leaked,
    // the guard would never be reached and this test would prove nothing.
    let flow = edge_flow(&engine, "inlet");
    assert!(
        flow.abs() < 1e-9,
        "the valve must be fully shut for the guard to be exercised, got {flow} kg/s"
    );

    for name in ["inlet", "outlet"] {
        let t = edge_temperature(&engine, name);
        assert!(
            t.is_finite(),
            "pipe '{name}' must carry a finite temperature behind a closed valve, got {t}"
        );
    }
    assert!(node_temperature(&engine, "shut").is_finite());
}

/// REFERENCE — reverse flow cools toward ambient too.
///
/// The sink is at 9 bar against a 1 bar source, so flow runs backwards up both
/// pipes and the junction is fed by the SINK. The `UA` pipe is therefore
/// traversed against its stored direction, which is the case a signed `ṁ` in the
/// transform's denominator would get wrong — and only this case, since every
/// other plant here flows the way its edges point.
#[test]
fn a_back_fed_pipe_cools_toward_ambient_rather_than_away_from_it() {
    let mut graph = PlantGraph::new();
    let feed = graph.add_node(source("feed", 1.0e5, T_AMBIENT));
    let tee = graph.add_node(node("tee", NodeKind::Junction));
    let back = graph.add_node(sink("back", 9.0e5, Kelvin(373.15))); // 100 °C

    graph.add_pipe(feed, tee, pipe("inlet", 10.0, 0.1));
    // Stored tee → back, but the flow runs back → tee.
    // A larger UA than the forward cases use: the 8 bar drop drives a much
    // bigger flow here, so the same UA would be a fraction of a transfer unit
    // and the cooling would be lost in the noise of a threshold.
    graph.add_pipe(
        tee,
        back,
        pipe_with_ambient("reversed_run", 10.0, 0.1, WattPerKelvin(60_000.0)),
    );

    let mut engine = engine(graph);
    engine.tick().expect("tick must converge");

    // Non-vacuity: the premise is that the flow actually reversed.
    let flow = edge_flow(&engine, "reversed_run");
    assert!(
        flow < -0.1,
        "the sink must be driving flow backwards up the pipe, got {flow} kg/s"
    );

    let tee_t = node_temperature(&engine, "tee");
    assert!(
        tee_t < 373.15 - 1.0,
        "a back-fed pipe must still cool its stream, got {tee_t} K from a 373.15 K sink"
    );
    // The mutation this catches: a signed `ṁ` flips the exponent and drives the
    // outlet AWAY from ambient, i.e. above the 373.15 K it started at.
    assert!(
        tee_t > T_AMBIENT.value(),
        "the outlet must land between ambient and the inlet, got {tee_t} K"
    );
}
