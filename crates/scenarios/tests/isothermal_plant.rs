//! The reference plant's thermal flat line (M2.1), now that friction heats it
//! (M5.1).
//!
//! Lives here rather than beside the other energy tests in
//! `solvers/tests/energy_invariants.rs` because it needs the reference TOML and
//! its loader, and `scenarios` depends on `solvers` — the reverse import would
//! be a dependency cycle.
//!
//! **What this file used to assert, and why it changed.** `tank_pump_valve` was
//! 20 °C throughout and had no heat input anywhere, so 293.15 K was not an
//! approximation to hold within a tolerance — it was a number that could not
//! move at all. That made it a very sharp trap for *spurious* energy terms: every
//! other M2 gate exercises a temperature DIFFERENCE and would stay green if the
//! engine added a constant offset, leaked pump work into the stream, or
//! mishandled `T_REF` symmetrically at both ends of a balance.
//!
//! Frictional dissipation is one of those terms, and it is no longer spurious. It
//! has no free parameter and therefore no honest "off" default (docs/DESIGN.md
//! §3a), so this plant now warms and this test had to change — **the first
//! deliberate break of the bit-identical regression anchor in the project.** A
//! `dissipation = false` scenario flag would have preserved the flat line by
//! making a physics term optional, which is a fidelity `if` wearing a config
//! file's clothes; it was considered and rejected.
//!
//! **The replacement is strictly stronger than what it retires**, because the
//! trap survives in two forms rather than being lost:
//!
//! - The **downstream** assertions are now against a *predicted* 0.098442 K
//!   instead of against zero. A spurious offset still fails; so does a missing
//!   one. Asserting against a number the engine must hit is a tighter constraint
//!   than asserting against a number it must not leave.
//! - The **`supply_tank` assertion is still an exact flat line**, and that is not
//!   a leftover — it is a gate no other test in the workspace provides. The
//!   supply tank is upwind of every source of friction in the plant, and it has
//!   no inflow, so its balance is `dE/dt = −ṁ·h(T_tank)` against `dm/dt = −ṁ`:
//!   `T` is constant *exactly*, for as long as it drains. Write an edge's
//!   frictional heat to its INLET rather than its outlet and this tank is charged
//!   for the discharge line's heat, which is DESIGN §3a's third mutation — and it
//!   is reachable only through the **tank-loop reader**, the same second reader
//!   the M2.2 pipe-ambient work found unguarded. `receiving_tank` cannot catch it
//!   (it is downstream, where inlet and outlet swap without changing its books).

use refinery_core::engine::Engine;
use refinery_core::graph::NodeKind;

const SCENARIO: &str = include_str!("../../../scenarios/tank_pump_valve.toml");

/// Every node and pipe in the reference plant starts at 20 °C.
const PLANT_TEMPERATURE_K: f64 = 293.15;

/// Long enough for the levels (and so the flow, and so the friction) to move
/// substantially, which is the point: the *shape* of the answer must hold
/// through all of it.
const TICKS: u64 = 200;

/// Pure float round-off on a value that is only ever copied, never computed from
/// a difference. Unchanged from the original flat-line test, because the flat
/// line it now guards — the supply tank's — really is exact.
const TOLERANCE_K: f64 = 1e-9;

/// The frictional rise from the supply tank to the receiving tank at the plant's
/// INITIAL levels [K]: 0.094309 across the valve, 0.003776 through the three
/// pipes, 0.000356 across the pump's curve droop (docs/DESIGN.md §3a).
///
/// Pinned to round-off by `dissipation_reference.rs`, which derives it from the
/// scenario's geometry and `kv_reference`'s independently derived 13.753287
/// kg/s. Here it is a *bound*, not an equality: as the supply tank drains the
/// head falls, the flow falls, and the rise falls with `Q²`. So the plant can
/// only ever be warmer than its feed and cooler than this.
const INITIAL_RISE_K: f64 = 0.098_441_5;

fn tank_temperature(engine: &Engine, name: &str) -> f64 {
    let id = engine
        .graph
        .find_node(name)
        .unwrap_or_else(|| panic!("the reference plant must have a '{name}'"));
    match &engine.graph.node(id).kind {
        NodeKind::Tank(tank) => tank.temperature.value(),
        other => panic!("'{name}' is not a tank: {other:?}"),
    }
}

#[test]
fn the_supply_tank_stays_exactly_isothermal_while_the_plant_warms() {
    let file = refinery_scenarios::load_str(SCENARIO).expect("reference scenario must parse");
    let mut engine = refinery_scenarios::build_engine(&file).expect("reference plant must build");

    for tick in 1..=TICKS {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick} failed: {e:?}"));

        // The exact flat line, upwind of everything. See the module docs: this is
        // the gate for frictional heat written to an edge's inlet.
        let supply = tank_temperature(&engine, "supply_tank");
        assert!(
            (supply - PLANT_TEMPERATURE_K).abs() < TOLERANCE_K,
            "tick {tick}: supply_tank drifted to {supply} K. It is upstream of every \
             source of friction in this plant and has no inflow, so its temperature is \
             constant exactly — a drift here means an edge's heat was booked at its \
             INLET, charging the tank for heat the fluid picked up after it left."
        );

        // Everything downstream warms, and by a bounded, monotone-shrinking
        // amount. Zero would mean the term never landed; more than the initial
        // rise would mean it landed twice, or that elevation head was credited
        // as heat.
        let received = tank_temperature(&engine, "receiving_tank");
        assert!(
            received > PLANT_TEMPERATURE_K && received < PLANT_TEMPERATURE_K + INITIAL_RISE_K,
            "tick {tick}: receiving_tank at {received} K must sit strictly between its \
             {PLANT_TEMPERATURE_K} K feed and {PLANT_TEMPERATURE_K} + {INITIAL_RISE_K} K \
             — it is fed a stream warmed by friction, blended into a large cold inventory"
        );

        let snapshot = engine.snapshot();
        assert_eq!(
            snapshot.edges.len(),
            3,
            "the reference plant has three pipes; a vacuous loop would prove nothing"
        );
        for edge in snapshot.edges {
            let actual = edge.stream.temperature.value();
            assert!(
                actual > PLANT_TEMPERATURE_K
                    && actual <= PLANT_TEMPERATURE_K + INITIAL_RISE_K + TOLERANCE_K,
                "tick {tick}: stream '{}' at {actual} K must be warmer than the \
                 {PLANT_TEMPERATURE_K} K it started from and no warmer than the plant's \
                 initial total rise",
                edge.name
            );
            assert!(
                edge.dissipation_w > 0.0,
                "tick {tick}: stream '{}' reports {} W of friction — with the plant \
                 flowing, every edge dissipates, and a zero here would make the bounds \
                 above vacuous",
                edge.name,
                edge.dissipation_w
            );
        }
    }
}

/// The first tick lands on the predicted number, not merely inside the bound.
///
/// The bound above is what can be asserted for 200 ticks as the levels move; this
/// is the sharp statement, at the one state whose flow is independently known.
/// Both are needed: the bound would pass for a term half the right size, and this
/// alone would not notice the plant going wrong on tick 2.
#[test]
fn the_first_tick_delivers_the_predicted_rise() {
    let file = refinery_scenarios::load_str(SCENARIO).expect("reference scenario must parse");
    let mut engine = refinery_scenarios::build_engine(&file).expect("reference plant must build");
    engine.tick().expect("the reference plant must converge");

    let fill = engine
        .snapshot()
        .edges
        .into_iter()
        .find(|e| e.name == "fill_line")
        .expect("the reference plant must have a fill_line")
        .stream
        .temperature
        .value();
    let rise = fill - PLANT_TEMPERATURE_K;
    assert!(
        (rise - INITIAL_RISE_K).abs() < 5e-6,
        "at the initial levels the water must reach the receiving tank {INITIAL_RISE_K} K \
         above its {PLANT_TEMPERATURE_K} K feed, got {rise} K"
    );
}
