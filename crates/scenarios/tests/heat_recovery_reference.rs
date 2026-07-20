//! The heat exchanger, as wired: `scenarios/heat_recovery.toml` end to end (M2.2).
//!
//! `core::energy`'s unit tests already pin the ΔT-effectiveness arithmetic and
//! the pair-merge in the sweep, on hand-built flow maps. Repeating that here
//! would test the same code twice. What only this level can see:
//!
//!   1. the `[[exchangers]]` table reaching the graph as a coupling at all —
//!      the loader is the only thing that builds one, and a plant whose table
//!      was ignored still runs, still converges, and reports two streams that
//!      simply never exchange heat,
//!   2. the coupling surviving a REAL hydraulic solve, where the two sides get
//!      genuinely different flows nothing in the file states,
//!   3. `C_min` being the smaller capacity rate of two the test did not choose,
//!   4. the outlets reaching the STREAMS, not just the node temperature field,
//!   5. the loader's refusals, each of which admits a plant that would run.
//!
//! Lives in `scenarios/` for the same reason as `furnace_reference.rs`: it needs
//! the TOML and its loader, and `scenarios` depends on `solvers`, so the reverse
//! import would be a dependency cycle.

use refinery_core::engine::Engine;
use refinery_core::error::SimError;
use refinery_core::snapshot::EdgeSnapshot;
use refinery_scenarios::ScenarioFile;

const SCENARIO: &str = include_str!("../../../scenarios/heat_recovery.toml");

/// The inlet temperatures the TOML declares, in SI (200 °C and 20 °C).
const HOT_IN_K: f64 = 473.15;
const COLD_IN_K: f64 = 293.15;

/// The effectiveness the TOML declares. Written out rather than read back from
/// the parsed file, so a loader that dropped or mangled it cannot make the
/// expectation agree with itself.
const EFFECTIVENESS: f64 = 0.6;

const TICKS: u64 = 20;
const TOLERANCE_K: f64 = 1e-9;

fn parse() -> ScenarioFile {
    refinery_scenarios::load_str(SCENARIO).expect("the heat recovery scenario must parse")
}

fn build() -> Engine {
    refinery_scenarios::build_engine(&parse()).expect("the heat recovery plant must build")
}

fn run(engine: &mut Engine) {
    for _ in 1..=TICKS {
        engine.tick().expect("the heat recovery plant must run");
    }
}

/// Build and require a refusal, returning the error.
///
/// `Engine` is not `Debug` (it owns boxed solver trait objects), so `expect_err`
/// is unavailable — and a bespoke panic message per call site is clearer anyway.
fn refuse(file: &ScenarioFile, why: &str) -> SimError {
    match refinery_scenarios::build_engine(file) {
        Ok(_) => panic!("{why}"),
        Err(err) => err,
    }
}

fn edge(engine: &Engine, name: &str) -> EdgeSnapshot {
    engine
        .snapshot()
        .edges
        .into_iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("the heat recovery plant must have a '{name}' pipe"))
}

/// The reference case: `Q = ε·C_min·(T_hot_in − T_cold_in)`, split between the
/// two streams by their own capacity rates.
///
/// The flows are NOT stated in the scenario — the hydraulic solve produces them
/// from the pipe sizes and driving pressures — so the test reads them back and
/// derives the expected outlets from the model's own formula, evaluated
/// independently here. That leaves it pinning the composition of loader,
/// solver, coupling and sweep rather than any one of them in isolation.
///
/// The `C_min` assertion below is the discriminating one: with the two capacity
/// rates deliberately far apart, using `C_max` instead would roughly triple the
/// duty and miss both outlets by tens of kelvin.
#[test]
fn the_exchanger_transfers_effectiveness_times_c_min() {
    let mut engine = build();
    run(&mut engine);

    let hot_in = edge(&engine, "hot_feed_line");
    let hot_out = edge(&engine, "hot_product_line");
    let cold_in = edge(&engine, "cold_feed_line");
    let cold_out = edge(&engine, "cold_product_line");

    let cp = hot_in.stream.composition.mixture_cp(&engine.slate).value();
    let hot_flow = hot_in.stream.mass_flow.value();
    let cold_flow = cold_in.stream.mass_flow.value();
    assert!(
        cold_flow > 1.0 && hot_flow > 0.1,
        "both streams must actually be flowing for heat to move: \
         hot {hot_flow} kg/s, cold {cold_flow} kg/s"
    );

    let capacity_hot = hot_flow * cp;
    let capacity_cold = cold_flow * cp;
    // The premise of the whole case, on both counts (see the TOML's header).
    // If the plant ever drifts towards equal capacity rates, `C_min` stops
    // being distinguishable from `C_max`; and if C_min ever moves off the hot
    // side, a per-side-effectiveness bug stops being detectable here at all —
    // the wrong formula would coincide with the right answer. Either way this
    // file would keep passing while testing less than it claims, so it fails
    // loudly instead of passing vacuously.
    assert!(
        capacity_hot < 0.5 * capacity_cold,
        "the reference plant must keep C_min on the HOT side and the two rates \
         far apart: hot {capacity_hot} W/K, cold {capacity_cold} W/K"
    );

    let duty = EFFECTIVENESS * capacity_hot.min(capacity_cold) * (HOT_IN_K - COLD_IN_K);
    let expected_hot = HOT_IN_K - duty / capacity_hot;
    let expected_cold = COLD_IN_K + duty / capacity_cold;

    assert!(
        (hot_out.stream.temperature.value() - expected_hot).abs() < TOLERANCE_K,
        "hot outlet must be {expected_hot} K, got {}",
        hot_out.stream.temperature.value()
    );
    assert!(
        (cold_out.stream.temperature.value() - expected_cold).abs() < TOLERANCE_K,
        "cold outlet must be {expected_cold} K, got {}",
        cold_out.stream.temperature.value()
    );

    // The inlets must be untouched: the exchanger writes its result onto the
    // OUTLET streams. Heating the inlet instead is a transport bug that the
    // outlet assertions alone would not distinguish from a correct plant.
    assert!(
        (hot_in.stream.temperature.value() - HOT_IN_K).abs() < TOLERANCE_K
            && (cold_in.stream.temperature.value() - COLD_IN_K).abs() < TOLERANCE_K,
        "the feed lines must still carry their source temperatures, got hot {} / cold {}",
        hot_in.stream.temperature.value(),
        cold_in.stream.temperature.value()
    );
}

/// Energy conserves ACROSS the coupling: what one stream loses, the other gains.
///
/// This is the property the single-signed-Q design buys, and it is worth an
/// assertion of its own because it holds for any ε and any capacity rates —
/// including ones this scenario does not produce. An implementation that gave
/// each side its own effectiveness term would match neither.
#[test]
fn the_exchanger_neither_creates_nor_destroys_heat() {
    let mut engine = build();
    run(&mut engine);

    let hot_in = edge(&engine, "hot_feed_line");
    let hot_out = edge(&engine, "hot_product_line");
    let cold_in = edge(&engine, "cold_feed_line");
    let cold_out = edge(&engine, "cold_product_line");
    let cp = hot_in.stream.composition.mixture_cp(&engine.slate).value();

    let given = hot_in.stream.mass_flow.value()
        * cp
        * (hot_in.stream.temperature.value() - hot_out.stream.temperature.value());
    let taken = cold_in.stream.mass_flow.value()
        * cp
        * (cold_out.stream.temperature.value() - cold_in.stream.temperature.value());

    assert!(
        given > 1.0e5,
        "the exchanger must be moving real heat for this to mean anything, got {given} W"
    );
    assert!(
        (given - taken).abs() < 1e-6 * given.abs(),
        "the hot stream gives up {given} W but the cold stream takes {taken} W"
    );
}

/// Neither outlet passes the other stream's INLET — the second-law bound this
/// fidelity gets for free from `C_min` with ε ≤ 1.
///
/// Deliberately NOT "the outlets stay ordered": a cold outlet above the hot
/// outlet is ordinary counter-current behaviour, and this model draws no
/// co-/counter-current distinction, so asserting that would pin a restriction
/// the physics does not impose.
#[test]
fn neither_stream_passes_the_others_inlet() {
    let mut engine = build();
    run(&mut engine);

    let hot_out = edge(&engine, "hot_product_line").stream.temperature.value();
    let cold_out = edge(&engine, "cold_product_line")
        .stream
        .temperature
        .value();

    let between_the_inlets = COLD_IN_K..=HOT_IN_K;
    assert!(
        between_the_inlets.contains(&hot_out),
        "the hot stream must land between the two inlets, got {hot_out} K"
    );
    assert!(
        between_the_inlets.contains(&cold_out),
        "the cold stream must land between the two inlets, got {cold_out} K"
    );
}

// ---------------------------------------------------------------------------
// Loader refusals
// ---------------------------------------------------------------------------
//
// Each of these admits a plant that BUILDS and RUNS. That is the standard for
// earning a guard here: a malformed pairing that crashed would need no check.

/// ε > 1 transfers more heat than the inlet difference makes available, driving
/// the C_min stream past the other inlet — a second-law violation every
/// intermediate number in the sweep stays finite and plausible through.
#[test]
fn effectiveness_above_one_is_refused() {
    for effectiveness in [1.0001, 2.0, f64::INFINITY] {
        let mut file = parse();
        file.exchangers[0].effectiveness = effectiveness;
        let err = refuse(
            &file,
            "effectiveness above 1 must be refused, got a running plant",
        );
        assert!(
            err.to_string().contains("(0, 1]"),
            "the error must state the valid range, got: {err}"
        );
    }
    // ε = 1 is the limit, not an error: it is the perfect exchanger.
    let mut file = parse();
    file.exchangers[0].effectiveness = 1.0;
    assert!(
        refinery_scenarios::build_engine(&file).is_ok(),
        "ε = 1 is a legal perfect exchanger, not an error"
    );
}

/// ε ≤ 0 is not an exchanger. Zero would run as a plain pipe — silently
/// transferring nothing while the plant looks correctly configured — and
/// negative would run the heat BACKWARDS, from cold to hot.
#[test]
fn effectiveness_at_or_below_zero_is_refused() {
    for effectiveness in [0.0, -0.5] {
        let mut file = parse();
        file.exchangers[0].effectiveness = effectiveness;
        refuse(&file, "effectiveness at or below 0 must be refused");
    }
}

/// A `heat_exchanger` node left out of the table behaves as a plain junction:
/// it transfers nothing at all, and nothing else about the plant looks wrong.
/// Forgetting the entry is the easiest mistake this format allows, so it is the
/// one most worth refusing.
#[test]
fn an_unpaired_exchanger_side_is_refused() {
    let mut file = parse();
    file.exchangers.clear();
    let err = refuse(&file, "an unpaired exchanger side must be refused");
    assert!(
        err.to_string().contains("not paired"),
        "the error must say the side is unpaired, got: {err}"
    );
}

/// One node with two partners. `exchanger_partner` returns the first coupling
/// it finds, so the plant would run — exchanging heat with one partner and
/// silently ignoring the other.
#[test]
fn pairing_a_side_twice_is_refused() {
    let mut file = parse();
    let duplicate = refinery_scenarios::ExchangerDef {
        side_a: "hx_hot".into(),
        side_b: "hx_cold".into(),
        effectiveness: 0.3,
    };
    file.exchangers.push(duplicate);
    let err = refuse(&file, "pairing a side twice must be refused");
    assert!(
        err.to_string().contains("more than once"),
        "the error must say the side is paired twice, got: {err}"
    );
}

/// A side paired with itself would make the exchanger its own upstream.
#[test]
fn pairing_a_side_with_itself_is_refused() {
    let mut file = parse();
    file.exchangers[0].side_b = "hx_hot".into();
    refuse(&file, "a self-paired side must be refused");
}

/// A coupling naming an ordinary node is a scenario that means something the
/// model does not represent — and, left unchecked, would make `is_zero_volume`
/// and the pair merge disagree about what that node is.
#[test]
fn pairing_a_non_exchanger_node_is_refused() {
    let mut file = parse();
    file.exchangers[0].side_b = "cold_source".into();
    let err = refuse(
        &file,
        "pairing a source as an exchanger side must be refused",
    );
    assert!(
        err.to_string().contains("not a heat_exchanger"),
        "the error must say what the node actually is, got: {err}"
    );
}
