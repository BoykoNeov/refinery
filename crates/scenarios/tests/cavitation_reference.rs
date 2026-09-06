//! M11: the cavitation criterion, on the plants it can and cannot be evaluated
//! on (docs/DESIGN.md §13).
//!
//! The property half — that the bubble pressure is the right number — lives in
//! `solvers`: the exact anchor (`P_bub = P_ATM` for a pure cut at its own `tb`,
//! for any Trouton constant) in `thermo.rs`, and the magnitude envelope in
//! `tests/reference/vapour_pressure.rs`, which M11 ties to `bubble_pressure`
//! rather than duplicating. This file is about the OTHER half: which nodes get a
//! verdict, which get silence, and what the difference between the two means.
//!
//! The distinction every gate here turns on is **`None` versus `false`**. `None`
//! is "there is no criterion at this node" — the wrong node kind, a gas, or a
//! thermo model with no vapour–liquid equilibrium — and `false` is a model that
//! looked and found the liquid was not boiling. Asserting `cavitating == false`
//! where `is_none()` is meant is the weaker test, and measurably so: the holdup
//! the exclusion is about spends its first 1 824 ticks ABOVE its bubble point,
//! where a *broken* exclusion would report `false` as well — so a gate written
//! that way would pass on the engine it is meant to catch, at most run lengths.
//! The gate that defends the node-kind clause has to be written on the `Option`,
//! and it has to run long enough for the tank to actually be boiling.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect` on the engine itself.

use refinery_core::snapshot::{NodeSnapshot, Snapshot};
use refinery_core::traits::ThermoModel;
use refinery_core::units::Kelvin;
use refinery_core::Engine;
use refinery_solvers::TroutonThermo;

const DEMO: &str = include_str!("../../../scenarios/cavitating_pump.toml");
const CASCADE: &str = include_str!("../../../scenarios/crude_column_cascade.toml");
const CONSTANT_PLANT: &str = include_str!("../../../scenarios/tank_pump_valve.toml");
const GAS_PLANT: &str = include_str!("../../../scenarios/gas_valve.toml");

/// The demo file's own declared numbers, named once so an assertion cannot drift
/// away from the plant it is about.
const DEMO_TEMPERATURE_K: f64 = 383.15;
const LIGHT_MASS_FRACTION: f64 = 0.7;
const HEAVY_MASS_FRACTION: f64 = 0.3;
const LIGHT_MOLAR_MASS: f64 = 0.100;
const HEAVY_MOLAR_MASS: f64 = 0.130;
const LIGHT_TB_K: f64 = 353.15;
const HEAVY_TB_K: f64 = 423.15;

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("a shipped scenario must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("a shipped scenario must build: {e}"))
}

fn run(engine: &mut Engine, ticks: u64) {
    for t in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
    }
}

fn node<'a>(snapshot: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    snapshot
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("the plant declares a node '{name}'"))
}

/// Trouton's saturated vapour pressure [Pa], written out from the correlation
/// rather than read back from the model.
///
/// `Psat = P_atm · exp[(C/R)·(1 − tb/T)]`. This is the one place in this file
/// where the model's own arithmetic is reproduced, and it is deliberate: the
/// plant-level gate below has to compare the number the ENGINE published against
/// a number computed outside it, or it grades the code against itself.
fn trouton_psat(tb_k: f64, t_k: f64) -> f64 {
    const TROUTON_CONSTANT: f64 = 88.0;
    const R_GAS: f64 = 8.314_462_618;
    101_325.0 * ((TROUTON_CONSTANT / R_GAS) * (1.0 - tb_k / t_k)).exp()
}

/// `crude_column_cascade.toml`'s declared molar masses [kg/mol] and normal
/// boiling points [K], in the file's own component order. Written out so the
/// control below rests on the plant definition rather than on anything the
/// engine computed.
const CASCADE_MOLAR_MASSES: [f64; 5] = [0.100, 0.130, 0.170, 0.220, 0.400];
const CASCADE_TB_K: [f64; 5] = [353.15, 423.15, 493.15, 573.15, 673.15];

/// Raoult's bubble pressure [Pa] of a MASS composition, by hand.
///
/// `n_c = w_c/M_c`, `x_c = n_c/Σn`, `P_bub = Σ x_c·Psat_c(T)`. The mole
/// conversion is written out rather than called, because a control that shares
/// the code it is controlling for is not a control.
fn bubble_pressure_by_hand(
    mass_fractions: &[f64],
    molar_masses: &[f64],
    tb_k: &[f64],
    t_k: f64,
) -> f64 {
    let moles: Vec<f64> = mass_fractions
        .iter()
        .zip(molar_masses)
        .map(|(w, m)| w / m)
        .collect();
    let total: f64 = moles.iter().sum();
    moles
        .iter()
        .zip(tb_k)
        .map(|(n, tb)| (n / total) * trouton_psat(*tb, t_k))
        .sum()
}

/// The demo mixture's bubble pressure [Pa] by hand, weighted by MOLE fractions.
fn hand_calculated_bubble_pressure() -> f64 {
    let n_light = LIGHT_MASS_FRACTION / LIGHT_MOLAR_MASS;
    let n_heavy = HEAVY_MASS_FRACTION / HEAVY_MOLAR_MASS;
    let total = n_light + n_heavy;
    (n_light / total) * trouton_psat(LIGHT_TB_K, DEMO_TEMPERATURE_K)
        + (n_heavy / total) * trouton_psat(HEAVY_TB_K, DEMO_TEMPERATURE_K)
}

/// The same sum weighted by MASS fractions — the mutation, not the model.
fn mass_weighted_bubble_pressure() -> f64 {
    LIGHT_MASS_FRACTION * trouton_psat(LIGHT_TB_K, DEMO_TEMPERATURE_K)
        + HEAVY_MASS_FRACTION * trouton_psat(HEAVY_TB_K, DEMO_TEMPERATURE_K)
}

/// **Gate 5 — the whole content of §3's correction, asserted.**
///
/// DESIGN §3 told frontends for ten milestones to read a NEGATIVE absolute node
/// pressure as "cavitating". A liquid boils below its own vapour pressure, which
/// is positive, so that marker fires late by exactly the vapour pressure — 0.58 m
/// of suction lift on cold water, and on this plant's hot naphtha 1.8 bar. The
/// demo's pump sits in that band: boiling, and at a comfortable positive
/// pressure the old marker says nothing about.
///
/// Both halves are asserted, because either alone is passed by something else: a
/// plant that reports `cavitating` at a negative pressure has merely rediscovered
/// §3's marker, and a plant at a positive pressure that reports nothing is every
/// other plant in the corpus.
#[test]
fn the_demo_reports_boiling_at_a_positive_absolute_pressure() {
    let mut engine = build(DEMO);
    run(&mut engine, 200);
    let snapshot = engine.snapshot();
    let pump = node(&snapshot, "suction");

    let cavitation = pump
        .cavitation
        .expect("the demo's pump is a subject of the criterion and its thermo can answer");
    assert!(
        cavitation.cavitating,
        "the demo exists to cavitate: {:.1} Pa against a bubble pressure of {:.1} Pa",
        pump.pressure_pa, cavitation.bubble_pressure_pa
    );
    assert!(
        pump.pressure_pa > 0.0,
        "§3's old marker (a negative absolute pressure) must still be SILENT here — \
         that is what makes this the late-marker case rather than a rediscovery of it. \
         Pump pressure {:.1} Pa",
        pump.pressure_pa
    );
    // And not marginally positive: the band §3's marker misses is the whole of
    // this plant's operating point, not a sliver at the edge of it.
    assert!(
        pump.pressure_pa > 1.0e5,
        "the demo's pump should sit a bar or more above zero while boiling, so the \
         gap between the two criteria is not a rounding question: {:.1} Pa",
        pump.pressure_pa
    );
}

/// **Gate 3 at plant level — the published number, against a hand calculation.**
///
/// The control is asserted first: on this mixture the mole- and mass-weighted
/// sums are far enough apart that this assertion can tell them apart. Without
/// that, the gate would be passed by a mass-weighted implementation on any
/// single-component plant (`degenerate-fixture-disables-the-code-path`), which
/// is what every other liquid plant in `scenarios/` is.
#[test]
fn the_demos_bubble_pressure_matches_a_mole_weighted_hand_calculation() {
    let by_mole = hand_calculated_bubble_pressure();
    let by_mass = mass_weighted_bubble_pressure();
    let separation = (by_mole - by_mass).abs() / by_mole;
    assert!(
        separation > 0.05,
        "the demo's slate must make mole and mass weighting distinguishable, or the \
         assertion below proves nothing: they differ by {separation:.4}"
    );

    let mut engine = build(DEMO);
    run(&mut engine, 200);
    let snapshot = engine.snapshot();
    for name in ["header", "suction", "discharge_valve"] {
        let published = node(&snapshot, name)
            .cavitation
            .unwrap_or_else(|| panic!("'{name}' is a subject of the criterion"))
            .bubble_pressure_pa;
        // A loose relative bound on purpose: the resolved node temperature is the
        // source's, plus whatever frictional dissipation the line put in — a few
        // millikelvin, which moves the bubble pressure in the fifth figure. The
        // mutation this gate is for moves it in the second.
        let error = (published - by_mole).abs() / by_mole;
        assert!(
            error < 1.0e-3,
            "'{name}' publishes {published:.1} Pa against a hand-calculated \
             {by_mole:.1} Pa ({error:.2e} relative); the mass-weighted sum would be \
             {by_mass:.1} Pa"
        );
    }
}

/// **`docs/DEFERRED.md` B3's trigger evidence, made durable.**
///
/// The row carried, for a milestone, a distance of `0.30×` on `crude_column`. That
/// number came from a standalone script, and that plant declares
/// `thermo = "constant"`, whose `bubble_pressure` is an `Err` — so **no engine
/// configuration of it produces any margin at all**. It was the same error that was
/// corrected twice on B1 before B1 closed, and the reason it survived is that the
/// number lived in a scratch directory rather than in a test. This gate is the
/// correction made durable: the margin is produced by the **engine's own**
/// `ThermoModel`, on the plant's **own declared fidelity**, with no edit to any file.
///
/// **Two assertions, not one crossing tick.** The crossing is at tick 1 825 —
/// measured both with this model and with `bubble_pressure_by_hand`, which agree
/// exactly — but pinning a single tick would fail on a legitimate change in the last
/// digit, because the margin is nearly flat there (deciles 1.002, 0.961, 0.944). So
/// the gate brackets it: comfortably above at tick 1 000, comfortably below at 2 500.
/// Both halves are load-bearing — the early one is what proves the tank is not simply
/// born boiling, and without it the late one would pass on a plant that was never
/// healthy.
///
/// **This gate inverts when B3 is fixed, and that is the intended end of it.** A
/// plant that no longer stores a boiling liquid is the goal. When the late assertion
/// fails because the margin rose above 1.0, delete this test and close the row —
/// do not relax the bound.
#[test]
fn the_cascade_naphtha_tank_stores_a_boiling_liquid_on_its_own_declared_fidelity() {
    // The fidelity is part of the claim. A margin is a distance only if the model
    // the FILE selects can produce it; if this line changes, the number below stops
    // being engine-producible and this gate stops meaning what it says.
    assert!(
        CASCADE.contains("thermo = \"trouton\""),
        "as shipped, `crude_column_cascade` must declare a thermo fidelity that can
         answer a bubble pressure, or this margin is not engine-producible"
    );

    let thermo = TroutonThermo::new();
    let margin = |engine: &Engine| -> f64 {
        let snapshot = engine.snapshot();
        let (_, tank) = snapshot
            .tanks
            .iter()
            .find(|(name, _)| name == "naphtha_tank")
            .expect("the cascade plant has a naphtha tank");
        let bubble = thermo
            .bubble_pressure(
                &engine.slate,
                &tank.composition,
                Kelvin(tank.temperature.value()),
            )
            .expect("the plant's own thermo fidelity answers");
        node(&snapshot, "naphtha_tank").pressure_pa / bubble.value()
    };

    // Named once so a message cannot drift away from the tick it reports. The
    // mutation that moved the early sample past the crossing failed with a
    // message still naming tick 1 000 — this repo's "a correct comment over a
    // wrong constant" in miniature, found by running the mutation.
    const EARLY_TICK: u64 = 1_000;
    const LATE_TICK: u64 = 2_500;

    let mut engine = build(CASCADE);

    // Above its bubble point early: the state this row is about EMERGES during
    // the run, which is exactly why a load-time refusal cannot catch it. Without
    // this half, the gate would pass on a plant that was never healthy.
    run(&mut engine, EARLY_TICK);
    let early = margin(&engine);
    assert!(
        early > 1.0,
        "the tank must start ABOVE its bubble point: at tick {EARLY_TICK} it sits at {early:.4}"
    );

    // And below it later: docs/DEFERRED.md B3's trigger, "holds an inventory
    // below its own bubble point", met by this plant on its own declared
    // fidelity. If this ever reads above 1.0, B3 is fixed — delete the gate and
    // close the row rather than relaxing the bound.
    run(&mut engine, LATE_TICK - EARLY_TICK);
    let late = margin(&engine);
    assert!(
        late < 1.0,
        "B3's trigger: at tick {LATE_TICK} the naphtha tank sits at
         {late:.4} of its bubble pressure"
    );
}

/// **Gate 4 — the holdup exclusion, on a plant that is already in the state.**
///
/// `crude_column_cascade`'s naphtha tank is below its own bubble pressure from
/// tick 1825 onward: the column draws at real tray temperatures (M7.4a), the
/// tank has no cooler, and the model's own correlation says what it stores is
/// boiling — 0.957× at the tick this gate reads. That is `docs/DEFERRED.md` B3, a
/// two-phase INVENTORY, and not cavitation, which is a flow-path phenomenon. The
/// exclusion is what stops the two rows claiming each other's evidence.
///
/// **The control is computed here rather than read from the snapshot**, and it
/// has to be: an excluded node publishes no bubble pressure, so the only way to
/// show the exclusion is doing work — rather than sitting in front of a tank that
/// happens to be healthy — is to evaluate the criterion independently and find
/// that it WOULD fire. Without it, this test would pass on a plant whose tank was
/// nowhere near boiling, which is exactly what it looked like before tick 1825.
///
/// **Asserted on the `Option`, not on the verdict.** `cavitating == false` is the
/// weaker form: it is what a *broken* exclusion would report at any tick before
/// 1825, so a gate written that way would pass on the engine it is meant to
/// catch, at most run lengths.
#[test]
fn a_boiling_holdup_reports_no_criterion_while_the_flow_path_reports_one() {
    let mut engine = build(CASCADE);
    run(&mut engine, 2500);
    let snapshot = engine.snapshot();

    // The control: the tank IS below its bubble point, by a calculation that
    // touches none of the engine's own cavitation code.
    let (_, tank_state) = snapshot
        .tanks
        .iter()
        .find(|(name, _)| name == "naphtha_tank")
        .expect("the cascade plant has a naphtha tank");
    let bubble = bubble_pressure_by_hand(
        tank_state.composition.fractions(),
        &CASCADE_MOLAR_MASSES,
        &CASCADE_TB_K,
        tank_state.temperature.value(),
    );
    let tank = node(&snapshot, "naphtha_tank");
    let margin = tank.pressure_pa / bubble;
    assert!(
        margin < 1.0,
        "the control: this gate is only about an exclusion if the tank would \
         otherwise fire. At this tick it sits at {margin:.4} of its bubble pressure \
         ({:.1} Pa against {bubble:.1} Pa) — run the plant longer, or the plant has \
         changed",
        tank.pressure_pa
    );

    for holdup in ["naphtha_tank", "distillate_tank", "bottoms_tank"] {
        assert!(
            node(&snapshot, holdup).cavitation.is_none(),
            "'{holdup}' is a holdup: a tank below its bubble point is a two-phase \
             inventory (DEFERRED B3), not cavitation, and must report NO criterion — \
             not a verdict of `false`"
        );
    }
    // A column is AT its bubble point by definition; a signal there would report
    // the model working.
    assert!(
        node(&snapshot, "column").cavitation.is_none(),
        "a column is at its bubble point by construction and is excluded"
    );
    // Declared boundaries are typed into the file, not solved.
    assert!(
        node(&snapshot, "crude_source").cavitation.is_none(),
        "a source's pressure is declared, not solved"
    );

    let furnace = node(&snapshot, "preheater");
    let cavitation = furnace.cavitation.expect(
        "the furnace IS a subject — it is the only node in the whole shipped corpus at \
         which the engine can evaluate this criterion, which is why B1's four-name node \
         list (pump, valve, junction, exchanger) had no reachable subject at all",
    );
    assert!(
        !cavitation.cavitating,
        "the cascade's preheater runs above its bubble point ({:.1} Pa against \
         {:.1} Pa); if this now fails the plant has changed, not the criterion",
        furnace.pressure_pa, cavitation.bubble_pressure_pa
    );
}

/// **Gate 6 — a model that cannot answer reports nothing, on every node.**
///
/// `thermo = "constant"` has no vapour–liquid equilibrium, and fourteen of the
/// fifteen shipped plants select it. The mistake this prevents is not an `Err`
/// escaping — it is an `Err` being turned into a clean bill of health, which is
/// what `cavitating: false` on a pump, a valve and two tanks would be.
///
/// The control is the node kinds: this plant HAS a pump and two valves, so the
/// silence is the model refusing rather than the subject list being empty.
#[test]
fn a_plant_whose_thermo_cannot_answer_reports_nothing_anywhere() {
    let mut engine = build(CONSTANT_PLANT);
    run(&mut engine, 50);
    let snapshot = engine.snapshot();
    assert!(
        snapshot.nodes.iter().any(|n| n.name == "transfer_pump"),
        "the control: this plant must still contain a subject node kind"
    );
    for n in &snapshot.nodes {
        assert!(
            n.cavitation.is_none(),
            "'{}' reports a cavitation verdict on a plant whose thermo fidelity has no \
             vapour-liquid equilibrium",
            n.name
        );
    }
}

/// **Mutation 6's gate — a gas node reports nothing, and it needs a fixture.**
///
/// A vapour does not cavitate; it is already vapour. Every gas plant in
/// `scenarios/` selects `thermo = "constant"` and so refuses one step earlier,
/// which means the shipped corpus cannot catch a dropped phase check at all —
/// this fixture is `gas_valve.toml` with its fidelity line changed, which is the
/// only edit made.
///
/// The control is that the same fixture's valve HAS a solved pressure and a
/// resolved temperature, so what silences it is the phase and not missing state.
#[test]
fn a_gas_node_reports_no_criterion_even_when_the_model_could_answer() {
    let fixture = GAS_PLANT.replace("thermo = \"constant\"", "thermo = \"trouton\"");
    assert!(
        fixture.contains("thermo = \"trouton\""),
        "the fixture's one edit must have applied"
    );
    let mut engine = build(&fixture);
    run(&mut engine, 50);
    let snapshot = engine.snapshot();

    // The control: the slate is gas-phase (a gas cut has no constant liquid
    // density, which is how a snapshot shows one), and the valve node is fully
    // resolved.
    assert!(
        snapshot.slate.iter().all(|c| c.density_kg_per_m3.is_none()),
        "the control: this fixture's cuts must be gas-phase"
    );
    let valve = node(&snapshot, "control_valve");
    assert!(
        valve.pressure_pa.is_finite() && valve.temperature_k.is_finite(),
        "the control: the valve must be a fully resolved node, so that the silence \
         below is the phase check and not absent state"
    );
    assert!(
        valve.cavitation.is_none(),
        "a gas node cannot cavitate — it is already vapour — and must report no \
         criterion rather than a verdict against a liquid bubble point"
    );
}

/// **The two published fields carry one relationship, so they must not drift.**
///
/// `EdgeSnapshot::leak_mass_flow` has the same shape and the same defence: a
/// convenience view beside the number it is a view of, gated rather than
/// assumed. Checked at every node of every tick rather than at the endpoint,
/// which is the M9.3 rule.
#[test]
fn the_verdict_always_agrees_with_the_number_it_was_made_from() {
    let mut engine = build(DEMO);
    for tick in 1..=600u64 {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the demo must run: tick {tick}: {e}"));
        let snapshot = engine.snapshot();
        for n in &snapshot.nodes {
            let Some(c) = n.cavitation else { continue };
            assert_eq!(
                c.cavitating,
                n.pressure_pa < c.bubble_pressure_pa,
                "tick {tick}, node '{}': verdict {} against {:.3} Pa and a bubble \
                 pressure of {:.3} Pa",
                n.name,
                c.cavitating,
                n.pressure_pa,
                c.bubble_pressure_pa
            );
        }
    }
}

/// **The demo sits INTERIOR to both regimes for its whole run**, which is the
/// coverage requirement M8.4 recorded after its own wired loop never reached its
/// saturation arm, and M10.1 sized its vent to meet.
///
/// A demo that crossed its bubble point at the last tick would exercise the
/// criterion at exactly one operating point and would be one retune away from
/// exercising it at none.
#[test]
fn both_arms_stay_interior_for_the_whole_run() {
    let mut engine = build(DEMO);
    let (mut worst_boiling, mut worst_healthy) = (f64::NEG_INFINITY, f64::INFINITY);
    for tick in 1..=600u64 {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the demo must run: tick {tick}: {e}"));
        let snapshot = engine.snapshot();
        let ratio = |name: &str| {
            let n = node(&snapshot, name);
            n.pressure_pa / n.cavitation.expect("a subject node").bubble_pressure_pa
        };
        worst_boiling = worst_boiling.max(ratio("suction"));
        worst_healthy = worst_healthy.min(ratio("header").min(ratio("discharge_valve")));
    }
    assert!(
        worst_boiling < 0.85,
        "the pump must stay well inside the boiling regime all run, not touch it: \
         worst margin {worst_boiling:.4}"
    );
    assert!(
        worst_healthy > 1.4,
        "the header and the discharge valve must stay well clear all run, so the \
         non-firing arm is a measurement and not a near miss: worst margin \
         {worst_healthy:.4}"
    );
}

/// **Gate 7 — the wire form, asserted on the serialized bytes.**
///
/// M10.1's lesson: a byte-identity baseline protects the OLD plants and has no
/// power over the file a slice ADDS, so what this slice adds needs an assertion
/// of its own — and it has to be on the JSON. A Rust match on
/// `Some(CavitationSnapshot { cavitating: true, .. })` passes under any serde
/// tag, any key name and any field order; a frontend reads none of those things
/// through Rust.
#[test]
fn the_demo_reports_its_verdict_on_the_wire() {
    let mut engine = build(DEMO);
    run(&mut engine, 200);
    let snapshot = engine.snapshot();
    let json = serde_json::to_string(&snapshot).expect("a snapshot must serialize");

    let value: serde_json::Value = serde_json::from_str(&json).unwrap();
    let nodes = value["nodes"].as_array().unwrap();
    let find = |name: &str| {
        nodes
            .iter()
            .find(|n| n["name"] == name)
            .unwrap_or_else(|| panic!("'{name}' must be in the snapshot"))
    };

    let pump = find("suction");
    assert_eq!(
        pump["cavitation"]["cavitating"],
        serde_json::json!(true),
        "the demo's pump must publish `cavitation.cavitating = true`, in those words: \
         {}",
        pump["cavitation"]
    );
    assert!(
        pump["cavitation"]["bubble_pressure_pa"].as_f64().unwrap() > 0.0,
        "the verdict must ship beside the number it was made from"
    );
    assert_eq!(
        find("header")["cavitation"]["cavitating"],
        serde_json::json!(false),
        "the healthy arm must publish a verdict of false, not absence"
    );
    // And absence really is absence: an excluded node emits no key at all, which
    // is what keeps every plant that cannot answer byte-identical.
    assert!(
        find("rundown_source").get("cavitation").is_none(),
        "an excluded node must emit no `cavitation` key at all, not `null`"
    );
}
