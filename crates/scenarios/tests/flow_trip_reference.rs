//! M33: a trip on a pipe's flow — the low-flow furnace trip (docs/DESIGN.md §36,
//! docs/DEFERRED.md E13's flow clause).
//!
//! A flow is the one trip measurement that is absent at load: it is read from the
//! last hydraulic solution, and there is none before tick 1 (§24). The rule (§36
//! fork 1) is that the absence lasts exactly one trip pass, tick 1's, and the trip
//! stays armed and compares nothing on it. Every gate below is about that rule or
//! about the trip doing what a pump or valve trip already does, on a flow:
//!
//! - **gate 1**, the demo (`furnace_low_flow_trip.toml`) on both fidelities: no
//!   measurement on tick 1; armed and its untripped twin to the bit through tick
//!   3 016; `Tripped { at_tick: 3017 }` from 3 017; the furnace at zero duty and its
//!   outlet EXACTLY its inlet stream from then on; and every flow identical to the
//!   twin's on every tick to 6 000, because a fuel cut moves no flow. The twin's
//!   outlet passes 1 000 °C — what the trip prevents.
//! - **gate 2**, the wire form: no `measurement` key on tick 1, and a flow limit
//!   and measurement carrying their own unit from tick 2.
//! - **gate 3**, the latch, the one-tick lag and the hand-back, on a fixture whose
//!   flow can be RESTORED (the demo's cannot: its tank only drains). A shut-down
//!   feed trips the furnace one tick after the low flow is solved; the latch holds
//!   the cut after the flow is back; a reset is refused until a solve has seen the
//!   flow back, and relights nothing; then a relight is admitted.
//! - **gate 4**, the one-tick window, pinned: a plant LOADED with a lit furnace on
//!   too little flow fires for tick 1 unprotected and trips on tick 2.
//! - **gate 5**, the cold start: a plant loaded with no flow and the furnace out
//!   trips on tick 2 with nothing to cut, and the latch is then a start permissive
//!   — the furnace cannot be lit until the flow is established and the trip reset.
//!   That is why no timed startup bypass is built (it stays `docs/DEFERRED.md`
//!   E15).
//!
//! The load-time refusals are cases in `trip_reference.rs`'s sweep, so the sweep
//! stays the single owner of "every trip the loader cannot honour".

use refinery_core::graph::{ControlledValue, NodeKind, TripId, TripState};
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::units::Watt;
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/furnace_low_flow_trip.toml");

/// The demo's own trip block, verbatim with its comment, so removing it must land.
const DEMO_TRIP: &str = r#"# Fires AT OR BELOW 2 kg/s on the feed (docs/DESIGN.md §26 fork 6) and cuts the
# furnace's fuel. The flow is the last solve's, so on tick 1 there is none and
# the trip compares nothing (§36 fork 1); from tick 2 it compares every tick.
[[trips]]
name = "heater_low_flow"
measurement = { pipe = "feed_line", variable = "flow" }
direction = "low"
limit_kg_per_s = 2.0
actions = [{ furnace = "heater" }]
"#;

/// The tick whose trip pass fires the demo's trip, measured on the engine on
/// both fidelities before it was written here: the feed ends tick 3 016 at
/// 1.99951 kg/s, and the pass at the top of 3 017 compares that.
const DEMO_TRIP_TICK: u64 = 3017;
const LIMIT_KG_PER_S: f64 = 2.0;
const DUTY_W: f64 = 0.6e6;

/// The demo's charge tank, verbatim, so a fixture can swap in a source whose
/// flow does not fade and can be restored.
const CHARGE_TANK: &str = r#"[nodes.charge_tank]
type = "tank"
area_m2 = 1.25
height_m = 12.0
initial_level_m = 10.0
temperature_c = 20.0"#;

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the fixture must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("the fixture must build: {e}"))
}

fn swap(src: &str, from: &str, to: &str) -> String {
    assert!(
        src.contains(from),
        "the fixture's substitution must land: `{from}` is not in the plant"
    );
    src.replace(from, to)
}

fn with_solver(src: &str, solver: &str) -> String {
    swap(src, r#"flow = "newton""#, &format!(r#"flow = "{solver}""#))
}

/// The demo with its tank replaced by a 2 bar source — a feed that holds about
/// 4.9 kg/s for as long as the valve is open — and the feed valve and furnace
/// declared as given, on the given solver.
fn source_fixture(solver: &str, valve_opening: f64, duty_mw: f64) -> Engine {
    let src = swap(
        DEMO,
        CHARGE_TANK,
        "[nodes.charge_tank]\ntype = \"source\"\npressure_bar = 2.0\ntemperature_c = 20.0",
    );
    let src = swap(
        &src,
        "kv = 18.0\nopening = 1.0",
        &format!("kv = 18.0\nopening = {valve_opening:?}"),
    );
    let src = swap(&src, "duty_mw = 0.6", &format!("duty_mw = {duty_mw:?}"));
    build(&with_solver(&src, solver))
}

fn tick(engine: &mut Engine) {
    let t = engine.snapshot().tick + 1;
    engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
}

fn duty_w(engine: &Engine) -> f64 {
    let id = engine.graph.find_node("heater").expect("a heater");
    match engine.graph.node(id).kind {
        NodeKind::Furnace { duty, .. } => duty.value(),
        ref other => panic!("'heater' is a furnace, not {other:?}"),
    }
}

fn feed_flow(snapshot: &Snapshot) -> f64 {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == "feed_line")
        .expect("a feed line")
        .stream
        .mass_flow
        .value()
}

/// The furnace node's resolved temperature, K: its outlet.
fn outlet_k(snapshot: &Snapshot) -> f64 {
    snapshot
        .nodes
        .iter()
        .find(|n| n.name == "heater")
        .expect("a heater")
        .temperature_k
}

/// The furnace's coil temperature, K (M34, docs/DESIGN.md §37): a state, read off
/// the graph, at the END of the last tick.
fn coil_k(engine: &Engine) -> f64 {
    let id = engine.graph.find_node("heater").expect("a heater");
    match &engine.graph.node(id).kind {
        NodeKind::Furnace { coil, .. } => coil.temperature.value(),
        other => panic!("'heater' is a furnace, not {other:?}"),
    }
}

/// The demo's `coil_heat_capacity_mj_per_k = 0.6`, in SI.
const COIL_C_J_PER_K: f64 = 0.6e6;

/// The furnace's inlet stream temperature, K: what a cut furnace passes on once
/// its coil has cooled.
fn inlet_k(snapshot: &Snapshot) -> f64 {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == "valve_line")
        .expect("the furnace's inlet pipe")
        .stream
        .temperature
        .value()
}

fn outlet_c(snapshot: &Snapshot) -> f64 {
    outlet_k(snapshot) - 273.15
}

fn state(engine: &Engine) -> TripState {
    engine.snapshot().trips[0].state
}

fn set_duty(engine: &mut Engine, watts: f64) -> Result<(), String> {
    let node = engine.graph.find_node("heater").expect("a heater");
    engine
        .apply(Command::SetFurnaceDuty {
            node,
            duty: Watt(watts),
        })
        .map_err(|e| e.to_string())
}

fn set_valve(engine: &mut Engine, opening: f64) {
    let node = engine.graph.find_node("feed_valve").expect("a feed valve");
    engine
        .apply(Command::SetValveOpening { node, opening })
        .unwrap_or_else(|e| panic!("the feed valve to {opening}: {e}"));
}

fn reset(engine: &mut Engine) -> Result<(), String> {
    engine
        .apply(Command::ResetTrip { trip_id: TripId(0) })
        .map_err(|e| e.to_string())
}

fn expect_refused(result: Result<(), String>, what: &str, says: &str) {
    match result {
        Ok(()) => panic!("{what} should have been refused"),
        Err(e) => assert!(
            e.contains(says),
            "{what} was refused for another reason: {e}"
        ),
    }
}

// ------------------------------------------------------------------- gate 1

/// **Gate 1, on both fidelities.** The fuel cut lands on the measured tick, one
/// tick after the solve that took the feed below its limit; from that tick's own
/// snapshot the furnace adds no heat of its own — what its fluid still picks up is
/// exactly what its coil gives back as it cools (M34, docs/DESIGN.md §37); and
/// the cut moves no flow, so the plant's flows are its twin's to the bit for the
/// whole run.
///
/// **Before M34 the outlet equalled the inlet exactly from the cut on.** With a
/// coil, the metal is still at 87.7 °C when the fuel goes, so the fluid keeps
/// warming while it cools: 86.73 °C at the end of the cutting tick, 20.003 °C by
/// tick 4 000 (89.09 °C on the cutting tick before M36's flame, docs/DESIGN.md
/// §40). That is asserted as the first law with zero duty, every tick.
#[test]
fn the_demo_cuts_the_fuel_on_its_tick_and_moves_no_flow() {
    for solver in ["newton", "simple"] {
        // Neither may burst its tubes (M37, docs/DESIGN.md §42): the untripped
        // twin's dry coil passes the shipped 550 °C at tick 5 253, and the hole
        // that opens carries ~1e-10 kg/s of rounding — damage, not the cut this
        // gate compares. So both take a limit above their flame.
        let never = |src: &str| swap(src, "tube_failure_c = 550.0", "tube_failure_c = 3000.0");
        let mut demo = build(&with_solver(&never(DEMO), solver));
        let mut twin = build(&with_solver(&never(&swap(DEMO, DEMO_TRIP, "")), solver));
        assert!(twin.snapshot().trips.is_empty(), "the twin has no trip");

        let mut previous_coil = coil_k(&demo);
        for t in 1..=6000u64 {
            tick(&mut demo);
            tick(&mut twin);
            let snapshot = demo.snapshot();
            let twin_snapshot = twin.snapshot();
            let trip = &snapshot.trips[0];
            if t == 1 {
                // The pass ran before any solve: nothing to compare (§36 fork 1).
                assert_eq!(trip.measurement, None, "{solver}: no flow on tick 1");
                assert_eq!(trip.state, TripState::Armed, "{solver}");
            } else {
                // From tick 2 the pass compares the LAST solve's flow, which is
                // the previous snapshot's.
                assert!(trip.measurement.is_some(), "{solver}, tick {t}");
            }
            if t < DEMO_TRIP_TICK {
                assert_eq!(trip.state, TripState::Armed, "{solver}, tick {t}");
                assert_eq!(duty_w(&demo), DUTY_W, "{solver}, tick {t}");
                // An armed trip is inert: the plant is its twin to the bit.
                assert_eq!(
                    serde_json::to_string(&snapshot.nodes).unwrap(),
                    serde_json::to_string(&twin_snapshot.nodes).unwrap(),
                    "{solver}, tick {t}: an armed trip moved the plant"
                );
                assert_eq!(
                    serde_json::to_string(&snapshot.edges).unwrap(),
                    serde_json::to_string(&twin_snapshot.edges).unwrap(),
                    "{solver}, tick {t}: an armed trip moved the plant"
                );
            } else {
                assert_eq!(
                    trip.state,
                    TripState::Tripped {
                        at_tick: DEMO_TRIP_TICK
                    },
                    "{solver}, tick {t}: tripped on its tick, and latched"
                );
                assert_eq!(duty_w(&demo), 0.0, "{solver}, tick {t}: the fuel is cut");
                // A cut furnace adds no heat of its own: its coil only cools,
                // the fluid leaves between its inlet and the coil, and what the
                // fluid gains is exactly what the coil lost — the first law at
                // the furnace with zero duty.
                let coil = coil_k(&demo);
                let (outlet, inlet) = (outlet_k(&snapshot), inlet_k(&snapshot));
                // Only cools — or, once it has cooled to its inlet within rounding, is
                // warmed back to it and no further: never above the larger of the two.
                assert!(
                    coil <= previous_coil.max(inlet),
                    "{solver}, tick {t}: a cut coil only cools"
                );
                // The flow INTO the furnace, on its own inlet pipe: the game
                // solver closes the valve upstream of it only to its tolerance,
                // so the feed line's flow differs from it in the sixth digit.
                let flow = snapshot
                    .edges
                    .iter()
                    .find(|e| e.name == "valve_line")
                    .expect("the furnace's inlet pipe")
                    .stream
                    .mass_flow
                    .value();
                if flow > 0.0 {
                    // Between its inlet and the coil, in either order: late in a run the
                    // coil has cooled to its inlet within rounding and may sit a hair
                    // below. Widened by what one rounding step of the STORED coil
                    // carries into the fluid, `C·ulp(T_c)/(dt·ṁ·cp)`, which matters
                    // only on the vanishing trickle (energy::furnace_coil).
                    let cp_in = snapshot.edges[0]
                        .stream
                        .composition
                        .mixture_cp(&demo.slate)
                        .value();
                    let rounding = COIL_C_J_PER_K * previous_coil * f64::EPSILON / (flow * cp_in);
                    let (low, high) = (inlet.min(previous_coil), inlet.max(previous_coil));
                    assert!(
                        low - rounding <= outlet && outlet <= high + rounding,
                        "{solver}, tick {t}: the fluid leaves at {outlet} K, outside its inlet                          {inlet} K and the coil's {previous_coil} K"
                    );
                    let cp = snapshot.edges[0]
                        .stream
                        .composition
                        .mixture_cp(&demo.slate)
                        .value();
                    let gained = flow * cp * (outlet - inlet);
                    let released = COIL_C_J_PER_K * (previous_coil - coil);
                    assert!(
                        (gained - released).abs() <= 1.0e-6 * released.max(1.0),
                        "{solver}, tick {t}: the fluid gained {gained} W, the coil gave \
                         {released} W"
                    );
                }
            }
            previous_coil = coil_k(&demo);
            // A fuel cut moves no flow: at constant density nothing hydraulic
            // reads a furnace's duty. So the condition never clears on this
            // plant, and its latch is gated on gate 3's fixture instead.
            let flows = |s: &Snapshot| {
                s.edges
                    .iter()
                    .map(|e| e.stream.mass_flow.value())
                    .collect::<Vec<_>>()
            };
            assert_eq!(
                flows(&snapshot),
                flows(&twin_snapshot),
                "{solver}, tick {t}: the cut moved a flow"
            );
            if t == DEMO_TRIP_TICK - 2 {
                assert!(
                    feed_flow(&snapshot) > LIMIT_KG_PER_S,
                    "{solver}: the feed is above its limit at the end of tick {t}: {}",
                    feed_flow(&snapshot)
                );
            }
            if t == DEMO_TRIP_TICK - 1 {
                assert!(
                    feed_flow(&snapshot) <= LIMIT_KG_PER_S,
                    "{solver}: the feed crosses its limit at the end of tick {t}: {}",
                    feed_flow(&snapshot)
                );
            }
            if t == DEMO_TRIP_TICK + 100 {
                // Reachable on the demo itself: the feed only fades.
                expect_refused(
                    reset(&mut demo),
                    "a reset on a feed still below its limit",
                    "condition still holds",
                );
            }
            if t == 5000 {
                // What the trip prevents: the twin, still fired on a 0.096 kg/s
                // trickle, reads 368 °C at its furnace outlet (measured; 419 °C
                // before M36's flame sent part of the duty up the stack, and
                // 1 521 °C before M34's coil, which bounds the outlet by its own
                // temperature instead of letting it grow as 1/ṁ).
                assert!(
                    outlet_c(&twin_snapshot) > 350.0,
                    "{solver}: {}",
                    outlet_c(&twin_snapshot)
                );
                assert!(
                    outlet_c(&snapshot) < 20.01,
                    "{solver}: {}",
                    outlet_c(&snapshot)
                );
            }
        }
    }
}

// ------------------------------------------------------------------- gate 2

/// **Gate 2, the wire form.** Tick 1's trip has no `measurement` key at all
/// (skipped, not `null`), and from tick 2 the limit and the measurement carry
/// their unit in their tag, as a flow loop's setpoint does. Asserted on the
/// bytes: a Rust match passes under any tag (M10.1).
#[test]
fn a_flow_trip_serialises_no_measurement_on_tick_one_and_its_unit_after() {
    let mut demo = build(DEMO);
    tick(&mut demo);
    let one = serde_json::to_string(&demo.snapshot().trips).unwrap();
    assert!(!one.contains("measurement"), "{one}");
    assert!(
        one.contains(r#""limit":{"variable":"flow","kg_per_s":2.0}"#),
        "{one}"
    );
    tick(&mut demo);
    let two = serde_json::to_string(&demo.snapshot().trips).unwrap();
    assert!(
        two.contains(r#""measurement":{"variable":"flow","kg_per_s":"#),
        "{two}"
    );
    assert!(two.contains(r#""state":{"status":"armed"}"#), "{two}");
}

// ------------------------------------------------------------------- gate 3

/// **Gate 3, the latch, the lag, and the hand-back**, on a feed that can be
/// restored.
///
/// - Throttling the feed valve lands in the NEXT tick's solve; the furnace fires
///   that tick on the low flow — its coil, which the fluid now takes less from,
///   heats (M34: before the coil, the OUTLET jumped 20 K on this one tick; the
///   coil absorbs most of it, which is what a coil is for); the pass at the top
///   of the tick after compares that solve's flow and cuts. A flow trip acts one tick
///   after the solve that crossed its limit, as every trip compares the state
///   standing at the top of its tick.
/// - A relight and a reset are refused while the flow is low.
/// - Reopening the valve does not re-arm anything: the LATCH holds the cut with
///   the flow back above its limit.
/// - A reset in the same breath as the reopening is refused: it reads the last
///   solve, which has not seen the valve move. One tick later it is admitted.
/// - The reset relights nothing — the coil goes on cooling; a human relights,
///   and the coil and the outlet heat again.
#[test]
fn a_restored_feed_leaves_the_cut_latched_until_a_reset_and_a_relight() {
    for solver in ["newton", "simple"] {
        let mut engine = source_fixture(solver, 1.0, 0.6);
        for _ in 0..20 {
            tick(&mut engine);
        }
        let running = engine.snapshot();
        let running_coil = coil_k(&engine);
        assert!(
            feed_flow(&running) > 4.0,
            "{solver}: {}",
            feed_flow(&running)
        );
        assert_eq!(state(&engine), TripState::Armed);

        // Throttle the feed. The next solve sees it.
        set_valve(&mut engine, 0.2);
        tick(&mut engine);
        let throttled = engine.snapshot();
        let throttled_tick = throttled.tick;
        assert!(
            feed_flow(&throttled) < LIMIT_KG_PER_S,
            "{solver}: {}",
            feed_flow(&throttled)
        );
        assert_eq!(
            state(&engine),
            TripState::Armed,
            "{solver}: one tick of lag"
        );
        assert_eq!(duty_w(&engine), DUTY_W);
        assert!(
            coil_k(&engine) > running_coil + 0.1,
            "{solver}: the furnace fired one tick on the low flow, into its coil: {} then \
             {} K",
            running_coil,
            coil_k(&engine)
        );
        tick(&mut engine);
        assert_eq!(
            state(&engine),
            TripState::Tripped {
                at_tick: throttled_tick + 1
            },
            "{solver}"
        );
        assert_eq!(duty_w(&engine), 0.0, "{solver}: cut on the tripping tick");

        expect_refused(set_duty(&mut engine, DUTY_W), "relighting", "fuel cut");
        expect_refused(
            reset(&mut engine),
            "a reset on a low feed",
            "condition still holds",
        );
        set_duty(&mut engine, 0.0).expect("writing the cut itself is admitted");

        // Restore the feed. The latch, not the flow, now holds the cut.
        set_valve(&mut engine, 1.0);
        expect_refused(
            reset(&mut engine),
            "a reset before any solve has seen the valve reopen",
            "condition still holds",
        );
        for _ in 0..10 {
            tick(&mut engine);
            assert!(feed_flow(&engine.snapshot()) > 4.0, "{solver}");
            assert!(state(&engine).is_tripped(), "{solver}: the latch holds");
            assert_eq!(duty_w(&engine), 0.0, "{solver}");
        }
        expect_refused(
            set_duty(&mut engine, DUTY_W),
            "relighting while latched",
            "fuel cut",
        );

        reset(&mut engine).expect("the feed is back, so the reset is admitted");
        assert_eq!(state(&engine), TripState::Armed);
        assert_eq!(duty_w(&engine), 0.0, "{solver}: a reset relights nothing");
        let before = coil_k(&engine);
        tick(&mut engine);
        assert_eq!(duty_w(&engine), 0.0, "{solver}");
        assert!(
            coil_k(&engine) < before && outlet_k(&engine.snapshot()) <= before,
            "{solver}: still unlit, so the coil only cools and the fluid takes no more \
             than it gives"
        );

        set_duty(&mut engine, DUTY_W).expect("after the reset a human may relight");
        let unlit = coil_k(&engine);
        tick(&mut engine);
        assert_eq!(state(&engine), TripState::Armed, "{solver}");
        assert!(coil_k(&engine) > unlit, "{solver}: the relit coil heats");
        // 0.6 MW on about 4.9 kg/s of water: about 29 K, once the coil (34 s) has
        // caught up.
        for _ in 0..300 {
            tick(&mut engine);
        }
        assert!(
            outlet_c(&engine.snapshot()) > 45.0,
            "{solver}: {}",
            outlet_c(&engine.snapshot())
        );
    }
}

// ------------------------------------------------------------------- gate 4

/// **Gate 4, the one-tick window, pinned.** A plant loaded with its furnace lit
/// and its feed throttled below the limit runs tick 1 unprotected — the pass had
/// no flow to compare — and trips on tick 2. This is the price of the rule in
/// §36 fork 1, stated rather than hidden: exactly one tick, and only on a plant
/// whose own file declares the unsafe state.
///
/// **Since M34 that tick lands on the coil** (docs/DESIGN.md §37). Before the
/// coil, tick 1's outlet read above 80 °C on the throttled feed; now the coil,
/// loaded at the full-feed 53.94 °C, takes the tick's duty less what the slower
/// fluid carries off, and the outlet stays near the coil. What the window costs
/// is the coil's rise, and after the cut the coil only cools.
#[test]
fn a_plant_loaded_lit_on_a_low_feed_fires_for_tick_one_and_trips_on_tick_two() {
    let mut engine = source_fixture("newton", 0.2, 0.6);
    tick(&mut engine);
    let one = engine.snapshot();
    assert_eq!(one.trips[0].measurement, None);
    assert_eq!(one.trips[0].state, TripState::Armed);
    assert_eq!(duty_w(&engine), DUTY_W, "lit through tick 1");
    assert!(feed_flow(&one) < LIMIT_KG_PER_S, "{}", feed_flow(&one));
    // About 1.5 kg/s against 0.6 MW: the coil heats on tick 1, the cost of the
    // window, and the outlet cannot leave hotter than the coil.
    let loaded_coil = 53.94 + 273.15;
    let after_one = coil_k(&engine);
    assert!(
        after_one > loaded_coil,
        "the coil took the window's duty: {after_one} K"
    );
    assert!(outlet_k(&one) <= after_one, "{}", outlet_c(&one));

    tick(&mut engine);
    assert_eq!(state(&engine), TripState::Tripped { at_tick: 2 });
    assert_eq!(duty_w(&engine), 0.0);
    assert!(coil_k(&engine) < after_one, "cut, the coil only cools");
}

// ------------------------------------------------------------------- gate 5

/// **Gate 5, the cold start, and why no startup bypass is built.** A plant
/// loaded with its feed shut and its furnace out trips on tick 2 with nothing to
/// cut, and the hold check finds the furnace out on every tick after. The latch
/// is then a START PERMISSIVE: the furnace cannot be lit until the feed is
/// established, a solve has seen it, and the trip is reset — the sequence a real
/// low-flow interlock imposes. A timer that held the trip off would let a human
/// light the furnace before any flow, which is what the trip is for
/// (`docs/DEFERRED.md` E15 keeps the timed bypass).
#[test]
fn a_cold_start_trips_with_nothing_to_cut_and_the_latch_is_the_start_permissive() {
    let mut engine = source_fixture("newton", 0.0, 0.0);
    tick(&mut engine);
    assert_eq!(engine.snapshot().trips[0].measurement, None);
    assert_eq!(state(&engine), TripState::Armed);
    tick(&mut engine);
    assert_eq!(state(&engine), TripState::Tripped { at_tick: 2 });
    assert_eq!(
        engine.snapshot().trips[0].measurement,
        Some(ControlledValue::Flow {
            kg_per_s: refinery_core::units::KgPerSec(0.0)
        }),
        "a shut valve's flow is a measurement of zero, not an absence"
    );
    for _ in 0..5 {
        tick(&mut engine);
        assert_eq!(duty_w(&engine), 0.0);
    }

    expect_refused(
        set_duty(&mut engine, DUTY_W),
        "lighting with no feed",
        "fuel cut",
    );
    expect_refused(
        reset(&mut engine),
        "a reset with no feed",
        "condition still holds",
    );

    set_valve(&mut engine, 1.0);
    expect_refused(
        set_duty(&mut engine, DUTY_W),
        "lighting before the reset",
        "fuel cut",
    );
    tick(&mut engine);
    assert!(feed_flow(&engine.snapshot()) > 4.0);
    reset(&mut engine).expect("the feed is established");
    set_duty(&mut engine, DUTY_W).expect("the permissive is met");
    tick(&mut engine);
    assert!(outlet_c(&engine.snapshot()) > 45.0);
    assert_eq!(state(&engine), TripState::Armed);
}

/// **A flow limit has no sign bound** (§36 fork 2): the flow is signed by the
/// pipe's declared direction and never clipped (§24, E11), so a low trip at or
/// below zero is a reverse-flow trip, and it loads.
#[test]
fn a_reverse_flow_limit_loads() {
    let engine = build(&swap(DEMO, "limit_kg_per_s = 2.0", "limit_kg_per_s = -0.5"));
    assert_eq!(
        engine.snapshot().trips[0].limit,
        ControlledValue::Flow {
            kg_per_s: refinery_core::units::KgPerSec(-0.5)
        }
    );
}
