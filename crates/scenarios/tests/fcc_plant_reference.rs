//! The 4-lump kinetics as WIRED: `scenarios/fcc_plant.toml` end to end (M4.2).
//!
//! `solvers/tests/reference/four_lump.rs` already pins the kinetics themselves —
//! the published plant envelope, the closed-form solutions, the integrator's
//! order of convergence — against the model in isolation, and re-deriving any of
//! that here would add nothing.
//!
//! What is NOT covered anywhere else, and is what this file exists for:
//!   1. the `reactions = "fcc"` fidelity selection reaching a real
//!      `NodeKind::Reactor` (M4.1's `reactor_reference.rs` covers `"lookup"`,
//!      which is a different arm of the same match),
//!   2. the kinetics running inside a real hydraulic solve and transport sweep,
//!      on a feed flow the TOML never states,
//!   3. **the reactor's conversion RESPONDING to its setpoint** — the property
//!      that separates kinetics from M4.1's fixed table, and one no test of the
//!      lookup fidelity could ever have,
//!   4. **coke routed to the bottoms draw** — DESIGN §5 accepts coke as an
//!      awkward pseudo-component on the explicit grounds that "a downstream
//!      column routes it to bottoms". Until this plant existed, no scenario put a
//!      reactor upstream of a column, so that claim was untested.
//!
//! Lives in `scenarios/` for the same reason as `reactor_reference.rs`: it needs
//! the TOML and its loader, and `scenarios` depends on `solvers`.

use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{EdgeSnapshot, NodeSnapshot};

/// `Engine` is not `Debug`, so the `Ok` arm cannot be unwrapped by `expect_err`.
fn build_error(scenario: &str) -> String {
    let file = refinery_scenarios::load_str(scenario).expect("the file must still parse");
    match refinery_scenarios::build_engine(&file) {
        Ok(_) => panic!("the plant must be refused at load"),
        Err(e) => e.to_string(),
    }
}

const SCENARIO: &str = include_str!("../../../scenarios/fcc_plant.toml");

/// Slate order in the file: gas, gasoline, gasoil, coke.
const GAS: usize = 0;
const GASOLINE: usize = 1;
const GASOIL: usize = 2;
const COKE: usize = 3;

/// The `t_set_c = 527.0` the TOML declares, in SI (K).
const T_SET_K: f64 = 527.0 + 273.15;

/// Enough ticks for the tanks to take delivery; the plant has no inventory
/// upstream of them, so the hydraulic solve is at steady state from tick 1.
const TICKS: u64 = 200;

fn edge<'a>(snap: &'a [EdgeSnapshot], name: &str) -> &'a EdgeSnapshot {
    snap.iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("scenario must define edge '{name}'"))
}

fn node<'a>(snap: &'a [NodeSnapshot], name: &str) -> &'a NodeSnapshot {
    snap.iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("scenario must define node '{name}'"))
}

fn run(scenario: &str, ticks: u64) -> refinery_core::snapshot::Snapshot {
    let file = refinery_scenarios::load_str(scenario).expect("the fcc plant must parse");
    let mut engine = refinery_scenarios::build_engine(&file).expect("the fcc plant must build");
    for _ in 0..ticks {
        engine.tick().expect("the fcc plant must tick");
    }
    engine.snapshot()
}

#[test]
fn the_kinetics_crack_the_feed_inside_a_real_solve() {
    let snap = run(SCENARIO, TICKS);
    let feed = &edge(&snap.edges, "hot_feed_line").stream;
    let effluent = &edge(&snap.edges, "reactor_effluent").stream;

    // Pure gasoil in — the reaction has not touched the feed side.
    let feed_frac = feed.composition.fractions();
    assert!(
        (feed_frac[GASOIL] - 1.0).abs() < 1e-9,
        "the riser feed must be pure gasoil, got {feed_frac:?}"
    );
    assert!(
        feed.mass_flow.value() > 0.0,
        "the plant must actually be flowing"
    );

    // Cracked slate out, in the published band. The bands are the reference
    // test's, restated at the plant scale rather than re-derived: what is new
    // here is that the feed flow, the feed temperature and the pressure field
    // all come from a real solve the TOML never states.
    let y = effluent.composition.fractions();
    assert!(
        (0.40..=0.50).contains(&y[GASOLINE]),
        "gasoline must land in the published 40-50 wt% band, got {}",
        y[GASOLINE]
    );
    assert!(
        (0.04..=0.07).contains(&y[COKE]),
        "coke must land in the published 4-7 wt% band, got {}",
        y[COKE]
    );
    assert!(
        (0.70..=0.85).contains(&(1.0 - y[GASOIL])),
        "conversion must land in the published 70-85 wt% band, got {}",
        1.0 - y[GASOIL]
    );

    // The setpoint is imposed on the reactor outlet, not mixed from the feed —
    // and the feed is genuinely at a different temperature, so this is not
    // vacuously true.
    let riser = node(&snap.nodes, "riser");
    assert!(
        (riser.temperature_k - T_SET_K).abs() < 1e-6,
        "the reactor must hold t_set = {T_SET_K} K, got {}",
        riser.temperature_k
    );
    assert!(
        (feed.temperature.value() - T_SET_K).abs() > 50.0,
        "the feed must reach the riser well below t_set, or the setpoint gate proves nothing"
    );
}

/// Conversion must RESPOND to the reactor's temperature setpoint. This is the
/// gate M4.1 could not have: `SimpleLookup`'s demo table is a single open-topped
/// band, so its yields are identical at every `t_set`, and every other gate in
/// this file would pass unchanged against it. A model whose Arrhenius shift was
/// dropped — `k(T) = k_ref` — is a plausible, mass-conserving, bit-identical
/// implementation that only this test can see.
#[test]
fn conversion_follows_the_reactor_setpoint() {
    let cooler = SCENARIO.replace("t_set_c = 527.0", "t_set_c = 470.0");
    assert_ne!(
        cooler, SCENARIO,
        "the setpoint substitution must have applied"
    );

    let hot = run(SCENARIO, 20);
    let cool = run(&cooler, 20);

    let conversion = |snap: &refinery_core::snapshot::Snapshot| {
        1.0 - edge(&snap.edges, "reactor_effluent")
            .stream
            .composition
            .fractions()[GASOIL]
    };
    let (hot_conversion, cool_conversion) = (conversion(&hot), conversion(&cool));
    assert!(
        cool_conversion < hot_conversion - 0.02,
        "a 57 K lower riser outlet temperature must convert measurably less gas oil: \
         {cool_conversion} vs {hot_conversion}"
    );
}

/// DESIGN §5's claim, finally exercised: coke is given a boiling point it does
/// not physically have precisely so a downstream column can route it, and the
/// bottoms draw is where it must land. The gate is two-sided — coke IN the
/// bottoms and ABSENT from the lighter draws — because a splitter that sent coke
/// everywhere would still conserve mass.
#[test]
fn coke_leaves_with_the_bottoms() {
    let snap = run(SCENARIO, TICKS);
    let gas_draw = edge(&snap.edges, "gas_draw").stream.composition.fractions();
    let gasoline_draw = edge(&snap.edges, "gasoline_draw")
        .stream
        .composition
        .fractions();
    let bottoms = edge(&snap.edges, "bottoms_draw")
        .stream
        .composition
        .fractions();

    assert!(
        bottoms[COKE] > 0.1,
        "the bottoms draw must carry the coke, got {bottoms:?}"
    );
    assert!(
        gas_draw[COKE] < 1e-6 && gasoline_draw[COKE] < 1e-6,
        "no coke may leave with the light draws (gas {}, gasoline {})",
        gas_draw[COKE],
        gasoline_draw[COKE]
    );
    // The light draws took their own cuts, so the routing is a real separation
    // rather than everything falling to the catch-all.
    assert!(
        gas_draw[GAS] > 0.99 && gasoline_draw[GASOLINE] > 0.99,
        "each light draw must carry its own cut (gas {:?}, gasoline {:?})",
        gas_draw,
        gasoline_draw
    );
    // And the coke reaches its tank, not just the pipe.
    let tank = node(&snap.nodes, "bottoms_tank");
    match &tank.kind {
        NodeKind::Tank(state) => assert!(
            state.composition.fractions()[COKE] > 1e-3,
            "the bottoms tank must accumulate coke, got {:?}",
            state.composition.fractions()
        ),
        other => panic!("bottoms_tank must be a tank, got {other:?}"),
    }
}

/// Chemistry moves mass BETWEEN components and a column splits it; neither
/// creates or destroys any. Across the pair, the three draws must still sum to
/// the reactor feed.
#[test]
fn the_reactor_and_column_together_are_mass_neutral() {
    let snap = run(SCENARIO, TICKS);
    let feed = edge(&snap.edges, "hot_feed_line").stream.mass_flow.value();
    let draws: f64 = ["gas_draw", "gasoline_draw", "bottoms_draw"]
        .iter()
        .map(|name| edge(&snap.edges, name).stream.mass_flow.value())
        .sum();
    assert!(
        (draws - feed).abs() < 1e-9 * feed.max(1.0),
        "the draws must sum to the reactor feed: {draws} vs {feed} kg/s"
    );
}

/// The fidelity name is part of the scenario contract: an unknown one must be
/// refused at load with the valid options listed, and `fcc` must be among them.
#[test]
fn the_reaction_fidelity_names_are_enumerated() {
    let bogus = SCENARIO.replace(r#"reactions = "fcc""#, r#"reactions = "arrhenius""#);
    let message = build_error(&bogus);
    assert!(
        message.contains("arrhenius") && message.contains("fcc"),
        "the error must name the bad model and list the valid ones, got: {message}"
    );
}

/// The lumps are resolved against the slate BY NAME at load time, so a plant
/// that selects the kinetics without the lumps they need fails while the
/// scenario is being read rather than mid-solve.
#[test]
fn a_slate_without_the_lumps_is_refused_at_load() {
    let renamed = SCENARIO.replace(r#"name = "coke""#, r#"name = "residue""#);
    let message = build_error(&renamed);
    assert!(
        message.contains("coke"),
        "the error must name the missing lump, got: {message}"
    );
}
