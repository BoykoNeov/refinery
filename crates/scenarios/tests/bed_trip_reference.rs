//! M57.3: a trip on a catalyst bed (docs/DESIGN.md §62.10).
//!
//! A bed's temperature is a state, present from load, written back by
//! `Engine::tick` at the end of every tick (§62.8) — so a trip may watch it as it
//! watches a furnace's coil, compared from tick 1's pass. What the trip DOES is
//! the decision §62.10 records, and this file gates both halves of it:
//!
//! - **cutting the preheater saves the bed.** The demo,
//!   `scenarios/hydrocracker_trip.toml`, is the runaway bed of M57.2 behind a
//!   fired preheater; the trip cuts the fire at 400 °C and the flowing oil
//!   carries the bed's heat away. Its untripped twin runs away to ~748 °C.
//! - **stopping the feed would trap it.** At zero flow a bed holds its
//!   temperature exactly (§62.8; no wall loss, §62.7), so a trip that shut the
//!   feed would hold the bed at the reading that fired it, forever, and refuse
//!   every reset. The loader refuses a bed trip that fully shuts a valve on the
//!   bed's own line (gate 6), so the trap is shown with the feed shut by hand
//!   (gate 4).
//!
//! A loop on a bed stays refused (M57.4's); that refusal is in
//! `bed_reactor_reference.rs`, beside the bed's other refusals.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::graph::{ControlledValue, MeasurementPoint, NodeKind, TripId, TripState};
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/hydrocracker_trip.toml");

/// The demo's trip block, verbatim with its comment, so removing it must land.
const DEMO_TRIP: &str = r#"
# Fires AT OR ABOVE 400 °C on the bed — a state on the graph (docs/DESIGN.md
# §62.10), present from load — and cuts the preheater's fuel. The feed is left
# flowing on purpose: it is what cools the bed. Reset by hand; the reset
# relights nothing.
[[trips]]
name = "bed_high"
measurement = { node = "bed", variable = "temperature" }
direction = "high"
limit_c = 400.0
actions = [{ furnace = "preheater" }]
"#;

/// The tick whose trip pass fires `bed_high`, measured on both fidelities
/// before it was written here: the bed first ends a tick at or over 400 °C on
/// tick 3 697, six minutes and ten seconds in.
const DEMO_TRIP_TICK: u64 = 3698;
const BED_LIMIT_C: f64 = 400.0;
const FEED_C: f64 = 300.0;
/// Thirty minutes of 0.1 s ticks: long enough for the twin to finish running
/// away and for the tripped bed to settle onto its feed.
const LONG_RUN: u64 = 18_000;

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the fixture must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("the fixture must build: {e}"))
}

fn refusal(src: &str) -> String {
    match refinery_scenarios::load_str(src) {
        Err(e) => e.to_string(),
        Ok(file) => match refinery_scenarios::build_engine(&file) {
            Ok(_) => panic!("this plant should not have loaded"),
            Err(e) => e.to_string(),
        },
    }
}

fn swap(src: &str, from: &str, to: &str) -> String {
    assert_eq!(
        src.matches(from).count(),
        1,
        "the fixture's substitution must land exactly once: {from:?}"
    );
    src.replacen(from, to, 1)
}

fn with_solver(src: &str, solver: &str) -> String {
    swap(src, r#"flow = "newton""#, &format!(r#"flow = "{solver}""#))
}

fn tick(engine: &mut Engine) {
    let t = engine.snapshot().tick + 1;
    engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
}

fn run(engine: &mut Engine, ticks: u64) {
    for _ in 0..ticks {
        tick(engine);
    }
}

/// The bed's temperature, K: a state, the END of the last tick.
fn bed_k(engine: &Engine) -> f64 {
    let id = engine.graph.find_node("bed").expect("a bed");
    match &engine.graph.node(id).kind {
        NodeKind::BedReactor { bed } => bed.temperature.value(),
        other => panic!("'bed' is a catalyst bed, not {other:?}"),
    }
}

fn bed_c(engine: &Engine) -> f64 {
    bed_k(engine) - 273.15
}

fn duty_w(engine: &Engine) -> f64 {
    let id = engine.graph.find_node("preheater").expect("a preheater");
    match engine.graph.node(id).kind {
        NodeKind::Furnace { duty, .. } => duty.value(),
        ref other => panic!("'preheater' is a furnace, not {other:?}"),
    }
}

/// The flow into the bed, kg/s, on the last snapshot.
fn bed_feed_kg_per_s(snapshot: &Snapshot) -> f64 {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == "heated_line")
        .expect("the bed's feed pipe")
        .stream
        .mass_flow
        .value()
}

fn measured_k(snapshot: &Snapshot) -> Option<f64> {
    snapshot.trips[0].measurement.map(|m| match m {
        ControlledValue::Temperature { k } => k.value(),
        other => panic!("a temperature trip measures a temperature, not {other:?}"),
    })
}

/// The plant as a frontend sees it, less the trips: what an ARMED trip must
/// not move.
fn plant_bytes(snapshot: &Snapshot) -> String {
    serde_json::to_string(&(&snapshot.nodes, &snapshot.edges)).expect("a snapshot serializes")
}

fn reset(engine: &mut Engine) -> Result<(), String> {
    engine
        .apply(Command::ResetTrip { trip_id: TripId(0) })
        .map_err(|e| e.to_string())
}

// ------------------------------------------------------------------- gate 1

/// **Gate 1, on both fidelities. The trip reads the bed's own state, from tick
/// 1, and fires on the first pass after the bed crosses its limit.**
///
/// Bit-identical to its untripped twin until the cut. On every pass the trip's
/// published reading IS the bed's temperature at the end of the tick before, bit
/// for bit — the state, not the fluid leaving the bed (which differs from it in
/// the fourth decimal). The bed ends tick 3 697 at or over 400 °C and tick 3 696
/// under it, so the pass at the top of 3 698 fires, and on that tick's own
/// snapshot the preheater reads zero duty with the feed still flowing.
#[test]
fn the_bed_trip_reads_the_bed_and_cuts_the_fire_on_its_tick() {
    for solver in ["newton", "simple"] {
        let mut demo = build(&with_solver(DEMO, solver));
        let mut twin = build(&with_solver(&swap(DEMO, DEMO_TRIP, ""), solver));
        let bed_id = demo.graph.find_node("bed").expect("a bed");
        assert_eq!(
            demo.snapshot().trips[0].watches,
            MeasurementPoint::Node(bed_id),
            "{solver}: the trip watches the bed node"
        );
        for t in 1..=DEMO_TRIP_TICK {
            let bed_before = bed_k(&demo);
            tick(&mut demo);
            tick(&mut twin);
            let snapshot = demo.snapshot();
            assert_eq!(
                measured_k(&snapshot),
                Some(bed_before),
                "{solver}, tick {t}: the trip compares the bed's state, from tick 1"
            );
            if t < DEMO_TRIP_TICK {
                assert_eq!(
                    snapshot.trips[0].state,
                    TripState::Armed,
                    "{solver}, tick {t}"
                );
                assert!(bed_before - 273.15 < BED_LIMIT_C, "{solver}, tick {t}");
                assert_eq!(
                    plant_bytes(&snapshot),
                    plant_bytes(&twin.snapshot()),
                    "{solver}, tick {t}: an armed trip moved the plant"
                );
            } else {
                assert!(
                    bed_before - 273.15 >= BED_LIMIT_C,
                    "{solver}: fired on {}",
                    bed_before - 273.15
                );
                assert_eq!(
                    snapshot.trips[0].state,
                    TripState::Tripped {
                        at_tick: DEMO_TRIP_TICK,
                        by_hand: false
                    },
                    "{solver}: tripped on its tick"
                );
                assert_eq!(duty_w(&demo), 0.0, "{solver}: the fire is cut");
                let flow = bed_feed_kg_per_s(&snapshot);
                assert!(
                    (flow - 49.9).abs() < 0.1,
                    "{solver}: the feed keeps flowing, {flow} kg/s"
                );
            }
        }
    }
}

// ------------------------------------------------------------------- gate 2

/// **Gate 2, on both fidelities. Cutting the fire saves the bed; its twin runs
/// away.**
///
/// After the cut the coil gives its heat back and the feed falls toward 300 °C.
/// The bed turns fifteen seconds after the cut, at 400.39 °C (measured) — under
/// a kelvin past the limit that fired it — and settles onto its feed: under 301 °C at thirty minutes, with
/// the trip still latched (a manual reset). The twin, with no trip, is past
/// 740 °C by then, nearly all its oil cracked to light ends (§62.9's hot
/// branch, on a 385 °C feed).
#[test]
fn cutting_the_fire_saves_the_bed_that_its_twin_loses() {
    for solver in ["newton", "simple"] {
        let mut demo = build(&with_solver(DEMO, solver));
        let mut twin = build(&with_solver(&swap(DEMO, DEMO_TRIP, ""), solver));
        let mut peak_c = f64::MIN;
        for _ in 0..LONG_RUN {
            tick(&mut demo);
            peak_c = peak_c.max(bed_c(&demo));
        }
        run(&mut twin, LONG_RUN);
        assert!(
            peak_c < BED_LIMIT_C + 1.0,
            "{solver}: the bed turns at once, peaking at {peak_c} °C"
        );
        let end_c = bed_c(&demo);
        assert!(
            end_c > FEED_C && end_c < FEED_C + 1.0,
            "{solver}: the bed settles onto its feed, at {end_c} °C"
        );
        assert!(demo.snapshot().trips[0].state.is_tripped(), "{solver}");
        let twin_c = bed_c(&twin);
        assert!(
            twin_c > 740.0,
            "{solver}: the twin runs away, to {twin_c} °C"
        );
    }
}

// ------------------------------------------------------------------- gate 3

/// **Gate 3. The trip can be reset once the bed is back under its limit, and
/// the reset relights nothing.**
///
/// Refused while the bed is at or over 400 °C, in the reset's own words;
/// admitted on the first look under it: the bed ends tick 4 008 under its limit,
/// 31 s after the cut (measured). The
/// preheater stays dark: relighting it at its old firing is the creep again, so
/// that is a person's act, not the reset's.
#[test]
fn the_bed_trip_resets_once_the_bed_cools_and_relights_nothing() {
    let mut engine = build(DEMO);
    run(&mut engine, DEMO_TRIP_TICK);
    assert!(engine.snapshot().trips[0].state.is_tripped());
    let mut admitted_at = None;
    for _ in 0..6000 {
        let bed = bed_c(&engine);
        if bed >= BED_LIMIT_C {
            let refused = reset(&mut engine).expect_err("over the limit, refused");
            assert!(refused.contains("condition still holds"), "{refused}");
        } else {
            reset(&mut engine).unwrap_or_else(|e| panic!("under the limit at {bed}: {e}"));
            admitted_at = Some(engine.snapshot().tick);
            break;
        }
        tick(&mut engine);
    }
    let admitted_at = admitted_at.expect("the flowing feed cools the bed under its limit");
    assert!(
        admitted_at < DEMO_TRIP_TICK + 1200,
        "within two minutes of the cut, not {admitted_at}"
    );
    assert_eq!(engine.snapshot().trips[0].state, TripState::Armed);
    assert_eq!(duty_w(&engine), 0.0, "a reset relights nothing");
    // Re-armed and still falling a minute on: it does not fire again.
    let at_reset_c = bed_c(&engine);
    run(&mut engine, 600);
    assert_eq!(engine.snapshot().trips[0].state, TripState::Armed);
    assert!(
        bed_c(&engine) < at_reset_c,
        "{} after {at_reset_c}",
        bed_c(&engine)
    );
}

// ------------------------------------------------------------------- gate 4

/// **Gate 4, on both fidelities. Why a bed trip may not shut the bed's line: a
/// stopped bed is trapped at the reading that fired the trip.**
///
/// The loader refuses that trip (gate 6), so the trap is shown here the only
/// way left to reach it: the shipped trip cuts the fire, and a person shuts
/// `feed_valve` by hand on the trip's own tick. From that tick the bed's feed is
/// exactly zero, and a bed at zero flow holds its temperature bit for bit
/// (§62.8): no reaction heat (no fresh oil) and no heat lost (no wall, §62.7).
/// So the bed sits at the very value the trip fired on, over its limit, and
/// every reset is refused, for as long as anyone waits.
#[test]
fn a_shut_feed_traps_the_bed_over_its_limit() {
    for solver in ["newton", "simple"] {
        let mut engine = build(&with_solver(DEMO, solver));
        run(&mut engine, DEMO_TRIP_TICK - 1);
        let feed_valve = engine.graph.find_node("feed_valve").expect("a feed valve");
        engine
            .apply(Command::SetValveOpening {
                node: feed_valve,
                opening: 0.0,
            })
            .unwrap_or_else(|e| panic!("{solver}: shutting the feed by hand: {e}"));
        let fired_on = bed_k(&engine);
        tick(&mut engine);
        assert!(engine.snapshot().trips[0].state.is_tripped(), "{solver}");
        assert!(fired_on - 273.15 >= BED_LIMIT_C, "{solver}");
        for minute in 0..10 {
            let snapshot = engine.snapshot();
            assert_eq!(
                bed_feed_kg_per_s(&snapshot),
                0.0,
                "{solver}, minute {minute}"
            );
            assert_eq!(
                bed_k(&engine),
                fired_on,
                "{solver}, minute {minute}: the stopped bed holds bit for bit"
            );
            let refused = reset(&mut engine).expect_err("a trapped bed never resets");
            assert!(refused.contains("condition still holds"), "{refused}");
            run(&mut engine, 600);
        }
    }
}

// ------------------------------------------------------------------- gate 5

/// **Gate 5. A bed answers for its temperature and nothing else.** A trip on its
/// level or pressure is refused at load with `measure`'s own reason, as on any
/// node that has none.
#[test]
fn a_bed_trip_on_anything_but_its_temperature_is_refused() {
    for (variable, limit, reason) in [
        ("level", "limit_m = 1.0", "is not a tank"),
        ("pressure", "limit_bar = 10.0", "no pressure of its own"),
    ] {
        let src = swap(
            &swap(
                DEMO,
                r#"measurement = { node = "bed", variable = "temperature" }"#,
                &format!(r#"measurement = {{ node = "bed", variable = "{variable}" }}"#),
            ),
            "limit_c = 400.0",
            limit,
        );
        let err = refusal(&src);
        assert!(
            err.contains("trip 'bed_high'") && err.contains(reason),
            "{variable}: {err}"
        );
    }
}

// ------------------------------------------------------------------- gate 6

/// The demo's trip with one more action.
fn demo_trip_also(action: &str) -> String {
    swap(
        DEMO,
        r#"actions = [{ furnace = "preheater" }]"#,
        &format!(r#"actions = [{{ furnace = "preheater" }}, {action}]"#),
    )
}

/// **Gate 6. A bed trip that fully shuts a valve on the bed's own line is
/// refused at load; one on a branch, or one closing only partway, is not.**
///
/// The trap gate 4 shows, refused before it can run. `feed_valve` sits ahead of
/// the preheater, past the leak split the loader puts on every furnace outlet
/// (§42), so the walk must step over that split's dormant orifice to see it. An
/// outlet valve between the bed and its product is the same trap from the other
/// side. A quench valve joining the line at a junction is admitted shut: the
/// feed still reaches the bed past it. So is `feed_valve` held at a tenth open:
/// some oil still flows, and it carries the heat away.
#[test]
fn a_bed_trip_that_shuts_the_beds_own_line_is_refused() {
    let err = refusal(&demo_trip_also(
        r#"{ valve = "feed_valve", position = 0.0 }"#,
    ));
    assert!(
        err.contains("trip 'bed_high'")
            && err.contains("valve 'feed_valve'")
            && err.contains("bed's own line")
            && err.contains("refuse every reset"),
        "the shut feed: {err}"
    );

    let with_outlet_valve = swap(
        &swap(DEMO, r#"to = "product""#, r#"to = "outlet_valve""#),
        "[nodes.product]",
        r#"[nodes.outlet_valve]
type = "valve"
kv = 110.0
opening = 1.0

[[pipes]]
name = "outlet_line"
from = "outlet_valve"
to = "product"
length_m = 5.0
diameter_m = 0.25

[nodes.product]"#,
    );
    build(&with_outlet_valve);
    let err = refusal(&swap(
        &with_outlet_valve,
        r#"actions = [{ furnace = "preheater" }]"#,
        r#"actions = [{ furnace = "preheater" }, { valve = "outlet_valve", position = 0.0 }]"#,
    ));
    assert!(
        err.contains("trip 'bed_high'") && err.contains("valve 'outlet_valve'"),
        "the shut outlet: {err}"
    );

    let with_quench = swap(
        &swap(DEMO, r#"to = "bed""#, r#"to = "mix""#),
        "[nodes.product]",
        r#"[nodes.quench]
type = "source"
pressure_bar = 8.0
temperature_c = 300.0
composition = { vgo = 1.0 }

[nodes.quench_valve]
type = "valve"
kv = 20.0
opening = 1.0

[nodes.mix]
type = "junction"

[[pipes]]
name = "quench_supply"
from = "quench"
to = "quench_valve"
length_m = 5.0
diameter_m = 0.1

[[pipes]]
name = "quench_line"
from = "quench_valve"
to = "mix"
length_m = 5.0
diameter_m = 0.1

[[pipes]]
name = "mixed_line"
from = "mix"
to = "bed"
length_m = 5.0
diameter_m = 0.25

[nodes.product]"#,
    );
    build(&swap(
        &with_quench,
        r#"actions = [{ furnace = "preheater" }]"#,
        r#"actions = [{ furnace = "preheater" }, { valve = "quench_valve", position = 0.0 }]"#,
    ));

    build(&demo_trip_also(
        r#"{ valve = "feed_valve", position = 0.1 }"#,
    ));
}
