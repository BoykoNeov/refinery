//! M12.1: the two-phase holdup (docs/DESIGN.md §14).
//!
//! The gates the design note named before anything was built, plus the refusals
//! that keep a fifth `[fidelity]` key from repeating what happened to the first.
//!
//! **The gate the note refuses to write, kept here so the next reader does not
//! reach for it.** "The mass removed times the latent heat equals the excess
//! enthalpy" is not a gate: it is the DEFINITION of the mass removed. That is
//! the M7.4b / M7.4c trap — a quantity defined to close a balance can never be
//! gated by that balance — and this project has now walked into it four times.
//! What replaces it is `a_boiling_tank_parks_on_its_own_bubble_point`, whose two
//! sides are a temperature trajectory the engine integrated and a thermodynamic
//! property recomputed from the published composition.
//!
//! **The trap the whole file is arranged around.** Removing mass at the tank's
//! own composition conserves mass exactly, passes I1 and I7 and every other
//! conservation test in the workspace, and never moves the tank's fractions. So
//! `the_vapour_leaves_at_y_not_x` is the only gate here that can tell a flash
//! from a decrement, and everything else is passed by the broken version.

use refinery_core::snapshot::{EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::traits::ThermoModel;
use refinery_core::units::{Kelvin, Pascal};
use refinery_core::Engine;
use refinery_solvers::{bubble_temperature, MoleFractions, TroutonThermo};

const ANCHOR: &str = include_str!("../../../scenarios/crude_column_cascade.toml");
const DEMO: &str = include_str!("../../../scenarios/crude_column_boiloff.toml");

/// Ticks the naphtha tank needs to reach its bubble point on the anchor plant,
/// measured in M12.0 and re-measured here: it first boils at tick 1 825. Every
/// gate about a BOILING tank has to run past it, and a gate that samples earlier
/// is asserting on the run length instead of the physics.
const TICKS: u64 = 6_000;
const BOILING_TANK: &str = "naphtha_tank";
const VENT: &str = "naphtha_tank__boiloff_vent";

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

fn edge<'a>(snapshot: &'a Snapshot, name: &str) -> &'a EdgeSnapshot {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("the plant carries an edge '{name}'"))
}

/// A tank's mass and mass-fraction composition, off the snapshot.
fn tank(snapshot: &Snapshot, name: &str) -> (f64, Vec<f64>, f64) {
    match &node(snapshot, name).kind {
        refinery_core::graph::NodeKind::Tank(t) => (
            t.mass.value(),
            t.composition.fractions().to_vec(),
            t.temperature.value(),
        ),
        other => panic!("node '{name}' is a {other:?}, not a tank"),
    }
}

/// The bubble temperature of a published composition, recomputed OUTSIDE the
/// engine through the same thermodynamics the plant declares.
///
/// It is the workspace's one root find, reached the way the boil-off model
/// reaches it. That makes this a check that the engine's *trajectory* lands on a
/// thermodynamic property, not a check that one arithmetic expression equals
/// itself — the two sides come from a Euler integration and a root find with
/// nothing in common but the composition.
fn bubble_point_of(slate: &refinery_core::components::Slate, fractions: &[f64]) -> f64 {
    let composition =
        refinery_core::components::Composition::from_weights(fractions).expect("a composition");
    let moles = MoleFractions::from_mass(&composition, slate).expect("mole fractions");
    bubble_temperature(
        slate,
        &TroutonThermo::new(),
        &moles,
        refinery_core::units::P_ATM,
        "the gate's tank",
    )
    .expect("the tank's mixture has a bubble point")
    .value()
}

// ---------------------------------------------------------------------------
// Gate 1 — a flash, not a decrement
// ---------------------------------------------------------------------------

/// **The gate for §14 fork 2, and the only one that separates a flash from a
/// decrement.**
///
/// The vapour leaves at `y = K·x`, which is richer in the light cut than the
/// liquid it came from, so a boiling tank gets HEAVIER than the same tank fed
/// the same stream with the term switched off. Removing mass at `x` instead
/// conserves mass just as exactly and leaves the fractions untouched.
///
/// **Two controls first, M9.3b's shape.** The tank is *declared* pure
/// `light_naphtha`, and for a pure fluid `y = K·x` and `x` are the same vector —
/// so a gate sampling early, or asserting against the file's own number, would
/// be comparing a composition the tank holds only at tick 0. And the two plants
/// must be at the same inflow, or "the compositions differ" is satisfied by two
/// plants that were never the same.
#[test]
fn the_vapour_leaves_at_y_not_x() {
    let mut boiling = build(DEMO);
    let mut inert = build(ANCHOR);
    run(&mut boiling, TICKS);
    run(&mut inert, TICKS);
    let (boiling_snapshot, inert_snapshot) = (boiling.snapshot(), inert.snapshot());

    let (_, hot_x, _) = tank(&boiling_snapshot, BOILING_TANK);
    let (_, cold_x, _) = tank(&inert_snapshot, BOILING_TANK);

    // Control 1: the tank is a real MIXTURE, not the pure cut the file declares.
    // Without this the assertion below is about a vector with one entry in it,
    // where the fork under test does not exist.
    assert!(
        cold_x[0] < 0.9 && cold_x[1] > 0.1,
        "the tank is still essentially pure ({cold_x:?}); it is *declared* \
         `light_naphtha = 1.0` and only becomes a mixture as the draw fills it, so this \
         gate is sampling before the thing it is about exists"
    );
    // Control 2: the term is actually running on this plant.
    let vented = edge(&boiling_snapshot, VENT).stream.mass_flow.value();
    assert!(
        vented > 0.0,
        "nothing is leaving the vent ({vented} kg/s), so every assertion below is about \
         a term that never fired"
    );

    // **The assertion that actually discriminates, and it is NOT the two-plant
    // comparison below.** §14 fork 2 calls the composition comparison "the only
    // gate that separates a flash from a decrement"; the mutation pass falsified
    // that. Replacing `y = K·x` with the tank's own `x` — the decrement — leaves
    // every plant-level assertion below GREEN, because a decrement still removes
    // mass, and a tank holding less mass blends the incoming draw in faster, so
    // its composition parts company with its twin's either way.
    //
    // What cannot be faked is the vapour's OWN composition, published on the
    // vent edge: it must be richer in the light cut than the liquid it left. A
    // decrement puts `x` on that edge, and `x` is not richer than itself.
    let vapour = &edge(&boiling_snapshot, VENT).stream.composition;
    assert!(
        vapour.fractions()[0] > hot_x[0] + 0.05,
        "the vent carries {:?} and the tank holds {hot_x:?}. The vapour of a boiling          mixture is enriched in the light cut — `y = K·x` — and a vent carrying the tank's          own composition is a DECREMENT wearing a flash's clothes: it conserves mass          exactly, passes I1 and I7, and moves nothing",
        vapour.fractions()
    );

    // The two-plant comparison, kept because it is what a reader will look for
    // and because it pins the DIRECTION of the plant-level effect — but it is
    // not what defends fork 2.
    assert!(
        hot_x[0] < cold_x[0],
        "the boiling tank holds MORE light naphtha ({:.6}) than the same tank with the \
         term off ({:.6}). The vapour is enriched — `y = K·x` — so a tank that boils must \
         get heavier. Equal fractions are the signature of a DECREMENT at `x`, which \
         conserves mass exactly and moves nothing",
        hot_x[0],
        cold_x[0]
    );
    assert!(
        hot_x[1] > cold_x[1],
        "the light cut left but the heavy fraction did not rise ({:.6} against {:.6})",
        hot_x[1],
        cold_x[1]
    );
}

// ---------------------------------------------------------------------------
// Gate 2 — the vent is what makes the mass balance close
// ---------------------------------------------------------------------------

/// **I1 over the boiling tank, with the vent counted — and the counterfactual.**
///
/// `Σ inflow = Σ outflow + Δ inventory`. The vapour leaves by an edge, so the
/// ordinary balance closes with no change to I1 itself. The second half is what
/// makes this a gate rather than an accounting identity: drop the vent term and
/// the balance must BREAK, or the assertion above is passed by an engine that
/// never boiled anything.
#[test]
fn the_mass_balance_closes_only_with_the_vent_counted() {
    let mut engine = build(DEMO);
    let start = engine.snapshot();
    let (mass_before, _, _) = tank(&start, BOILING_TANK);
    run(&mut engine, TICKS);
    let end = engine.snapshot();
    let (mass_after, _, _) = tank(&end, BOILING_TANK);

    // Rates at the END of the run, which is where the plant is at steady state:
    // the draw and the vent have both settled, so a one-tick rate stands for the
    // window this compares over.
    let draw = edge(&end, "naphtha_draw").stream.mass_flow.value();
    let vent = edge(&end, VENT).stream.mass_flow.value();
    assert!(
        vent > 0.0,
        "the vent carries {vent} kg/s, so the counterfactual below cannot fail and this \
         gate proves nothing"
    );

    // Δ inventory over the last tick, taken as the difference of the two settled
    // rates: what arrives, less what boils away.
    let dt = 0.1;
    let mut before = build(DEMO);
    run(&mut before, TICKS - 1);
    let (mass_one_tick_earlier, _, _) = tank(&before.snapshot(), BOILING_TANK);
    let accumulation = (mass_after - mass_one_tick_earlier) / dt;

    let residual = draw - vent - accumulation;
    let scale = draw.abs().max(vent.abs());
    assert!(
        residual.abs() <= 1e-9 * scale,
        "the tank's mass balance does not close with the vent counted: in {draw:.6} kg/s, \
         out {vent:.6} kg/s, accumulating {accumulation:.6} kg/s, residual {residual:.3e}"
    );

    // The counterfactual: the same balance with the vent term dropped is the
    // engine that decrements an inventory with no accounted path out of it.
    let without_vent = draw - accumulation;
    assert!(
        without_vent.abs() > 1e-6 * scale,
        "dropping the vent term leaves a residual of {without_vent:.3e} kg/s, which is \
         indistinguishable from zero — so this plant is not actually venting and the gate \
         above closes for the wrong reason"
    );
    // And the inventory really did grow more slowly than the draw alone would
    // have filled it.
    assert!(
        mass_after - mass_before < draw * dt * TICKS as f64,
        "the tank took on the whole draw, so nothing boiled off it"
    );
}

// ---------------------------------------------------------------------------
// Gate 3 — the tank parks on its bubble point
// ---------------------------------------------------------------------------

/// **A temperature trajectory against a thermodynamic property**, which is what
/// replaces the tautological duty gate the note refuses.
///
/// The boil-off is an algebraic constraint — the holdup cannot be above its own
/// bubble point — applied in the same tick that let it get there. So the
/// published temperature IS the bubble point of what the tank holds, and the
/// only slack is that the engine solved the root at the composition BEFORE the
/// flash while this recomputes it after.
///
/// **`ε` is derived from that, not chosen** (M2's truncation-tolerance rule, and
/// M8.3's 4.66e-5 lesson about bounds that merely look tight): it is the
/// distance the bubble point itself moves over one tick of composition change,
/// measured from two consecutive snapshots of this same run.
#[test]
fn a_boiling_tank_parks_on_its_own_bubble_point() {
    let mut engine = build(DEMO);
    run(&mut engine, TICKS - 1);
    let earlier = engine.snapshot();
    run(&mut engine, 1);
    let now = engine.snapshot();

    let (_, x_earlier, _) = tank(&earlier, BOILING_TANK);
    let (_, x_now, temperature) = tank(&now, BOILING_TANK);
    let bubble_now = bubble_point_of(&engine.slate, &x_now);
    let bubble_earlier = bubble_point_of(&engine.slate, &x_earlier);

    // The control: the tank is boiling at all. A tank below its bubble point
    // satisfies "T ≤ T_bub" trivially and would pass a one-sided assertion.
    assert!(
        edge(&now, VENT).stream.mass_flow.value() > 0.0,
        "the tank is not boiling at tick {TICKS}, so parking on its bubble point is not \
         being tested"
    );

    let epsilon = (bubble_now - bubble_earlier).abs() + 1e-9 * bubble_now;
    assert!(
        (temperature - bubble_now).abs() <= epsilon,
        "the tank sits at {temperature:.6} K and its own bubble point is {bubble_now:.6} K \
         — {:.3e} K apart, against a one-tick bubble-point drift of {epsilon:.3e} K. The \
         boil-off is supposed to leave the holdup ON its bubble point in the same tick it \
         reached it",
        temperature - bubble_now
    );
}

// ---------------------------------------------------------------------------
// Gate 5 — the key is what moves the plant, and only where it is selected
// ---------------------------------------------------------------------------

/// **M8.5's verification, not M8.5's prediction.** The anchor declares
/// `boiloff = "none"` explicitly so the pair reads as a pair; strip the line and
/// the plant must produce the same snapshot byte for byte, because the default
/// IS `"none"`.
#[test]
fn declaring_the_default_changes_no_byte_of_the_anchor() {
    let stripped: String = ANCHOR
        .lines()
        .filter(|line| line.trim() != "boiloff = \"none\"")
        .collect::<Vec<_>>()
        .join("\n");
    assert_ne!(
        stripped.len(),
        ANCHOR.len(),
        "the anchor no longer declares `boiloff = \"none\"`, so this gate is stripping \
         nothing"
    );

    let mut declared = build(ANCHOR);
    let mut absent = build(&stripped);
    // 200 ticks rather than 6 000: this is a byte-identity claim about parsing,
    // and every tick of it is the same claim.
    run(&mut declared, 200);
    run(&mut absent, 200);
    assert_eq!(
        serde_json::to_string(&declared.snapshot()).unwrap(),
        serde_json::to_string(&absent.snapshot()).unwrap(),
        "declaring the default moved a number. `boiloff = \"none\"` must be exactly what a \
         file with no such key means"
    );
}

/// A plant that does not select a boiling model carries **no vent edges at
/// all** — the key's whole footprint on the sixteen shipped plants.
#[test]
fn the_default_builds_no_vents() {
    let engine = build(ANCHOR);
    let snapshot = engine.snapshot();
    assert!(
        !snapshot.edges.iter().any(|e| e.name.contains("boiloff")),
        "a plant on `boiloff = \"none\"` grew vent edges: {:?}",
        snapshot.edges.iter().map(|e| &e.name).collect::<Vec<_>>()
    );
    assert!(
        !snapshot.nodes.iter().any(|n| n.name.contains("boiloff")),
        "a plant on `boiloff = \"none\"` grew an atmosphere node it never asked for"
    );

    // And the demo has exactly one vent per tank, pointing outward.
    let demo = build(DEMO).snapshot();
    let vents: Vec<&String> = demo
        .edges
        .iter()
        .filter(|e| e.name.ends_with("__boiloff_vent"))
        .map(|e| &e.name)
        .collect();
    assert_eq!(
        vents.len(),
        3,
        "the demo has three tanks and {} vents: {vents:?}",
        vents.len()
    );
}

// ---------------------------------------------------------------------------
// Gate 6 — the everything-flashes case terminates honestly
// ---------------------------------------------------------------------------

/// A tank fed a stream far above its own bubble point: **more than all of what
/// arrives would have to flash**, so the tank cannot fill.
///
/// Asserted against an INFLOW, not an inventory (§14 fork 3, correction 2). The
/// inventory version of this gate defends a state the correction prevents from
/// ever arising, which would be the fifth specified gate in this project with no
/// power over its own subject.
#[test]
fn a_tank_fed_above_its_own_bubble_point_does_not_fill_and_does_not_break() {
    const SCALDING: &str = r#"
[meta]
name = "scalding_tank"
description = "A tank fed naphtha at 800 K, far above the temperature its contents boil at."

[simulation]
dt = 0.1

[fidelity]
flow = "newton"
thermo = "trouton"
boiloff = "flash"

[[components]]
name = "light_naphtha"
tb_c = 80.0
molar_mass_kg_per_mol = 0.100
density_kg_per_m3 = 680.0
cp_j_per_kg_k = 2200.0

[[components]]
name = "heavy_naphtha"
tb_c = 150.0
molar_mass_kg_per_mol = 0.130
density_kg_per_m3 = 750.0
cp_j_per_kg_k = 2100.0

[nodes.hot_source]
type = "source"
pressure_bar = 3.0
temperature_c = 526.85
composition = { light_naphtha = 0.5, heavy_naphtha = 0.5 }

[nodes.feed_valve]
type = "valve"
kv = 40.0
opening = 1.0

[nodes.product_tank]
type = "tank"
area_m2 = 8.0
height_m = 12.0
initial_level_m = 0.5
temperature_c = 40.0
composition = { light_naphtha = 0.5, heavy_naphtha = 0.5 }

[[pipes]]
name = "feed_line"
from = "hot_source"
to = "feed_valve"
length_m = 10.0
diameter_m = 0.10

[[pipes]]
name = "tank_line"
from = "feed_valve"
to = "product_tank"
length_m = 10.0
diameter_m = 0.10
"#;
    let mut engine = build(SCALDING);
    let start = engine.snapshot();
    let (mass_before, _, _) = tank(&start, "product_tank");

    for t in 1..=2_000u64 {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("a tank that cannot fill must still RUN: tick {t}: {e}"));
        let snapshot = engine.snapshot();
        let (mass, fractions, temperature) = tank(&snapshot, "product_tank");
        assert!(
            mass >= 0.0 && mass.is_finite(),
            "tick {t}: the tank's inventory is {mass} kg"
        );
        assert!(
            temperature > 0.0 && temperature.is_finite(),
            "tick {t}: the tank sits at {temperature} K"
        );
        assert!(
            fractions.iter().all(|f| f.is_finite() && *f >= 0.0),
            "tick {t}: the tank holds {fractions:?}"
        );
    }

    let end = engine.snapshot();
    let (mass_after, _, _) = tank(&end, "product_tank");
    let arriving = edge(&end, "tank_line").stream.mass_flow.value();
    let vented = edge(&end, "product_tank__boiloff_vent")
        .stream
        .mass_flow
        .value();

    // The control: the plant is genuinely in the everything-flashes regime —
    // the arriving stream is above the tank's own bubble point by more than its
    // latent heat can absorb.
    let (_, fractions, _) = tank(&end, "product_tank");
    let bubble = bubble_point_of(&engine.slate, &fractions);
    let arriving_t = edge(&end, "tank_line").stream.temperature.value();
    assert!(
        arriving_t > bubble + 300.0,
        "the feed arrives at {arriving_t:.1} K into a tank boiling at {bubble:.1} K, which \
         is not the regime this gate is about"
    );
    assert!(
        arriving > 0.0 && vented > 0.0,
        "nothing is flowing ({arriving} kg/s in, {vented} kg/s out of the vent)"
    );
    // The claim: essentially everything that arrives leaves again, so the tank
    // does not fill. Not "the inventory is constant" — the tank still holds what
    // it started with, minus what boiled off it.
    assert!(
        (arriving - vented).abs() < 0.05 * arriving,
        "the tank is filling: {arriving:.4} kg/s arrives and only {vented:.4} kg/s boils \
         away. Above `f = 1` the enthalpy constraint has no solution, and the honest \
         answer is that everything arriving flashes"
    );
    assert!(
        mass_after <= mass_before,
        "the tank grew from {mass_before:.3} kg to {mass_after:.3} kg on a feed that \
         entirely flashes"
    );
}

// ---------------------------------------------------------------------------
// Gate 7 — the two files differ, and differ because of the key
// ---------------------------------------------------------------------------

/// **The gate that makes gate 5's silence mean "declined" rather than
/// "ignored", and the only thing standing between this key and the
/// `thermo = "nonsense"` defect** — parsed from M1 to M7.2 and never read.
///
/// **Its control is not the obvious one.** "The two files differ" is also
/// satisfied by a demo that differs for an unrelated reason — a mistyped draw
/// ratio, a different tank geometry — and such a gate would be green while
/// proving nothing about the key. So the documents are compared FIRST.
///
/// **Correction to §14 gate 7, made while writing it:** "identical except for
/// the `boiloff` line" cannot be literally true. The corpus matches baseline
/// rows by plant NAME, so the two files must differ in `[meta] name` as well,
/// and in the `description` beside it. The three exemptions are named here
/// rather than waved at, and comment lines are excluded because the demo has to
/// explain itself.
#[test]
fn the_demo_differs_from_its_twin_in_one_line_and_in_a_tenth_of_the_naphtha() {
    fn body(src: &str) -> Vec<String> {
        src.lines()
            .map(str::trim_end)
            .filter(|line| !line.trim_start().starts_with('#') && !line.trim().is_empty())
            .map(str::to_string)
            .collect()
    }
    let (anchor, demo) = (body(ANCHOR), body(DEMO));
    assert_eq!(
        anchor.len(),
        demo.len(),
        "the two files no longer have the same number of declaration lines, so they are \
         not a pair"
    );
    let differing: Vec<(&String, &String)> = anchor
        .iter()
        .zip(&demo)
        .filter(|(a, b)| a != b)
        .collect::<Vec<_>>();
    let exempt = |line: &str| line.starts_with("name = ") || line.starts_with("description = ");
    let unexplained: Vec<&(&String, &String)> = differing
        .iter()
        .filter(|(a, b)| {
            !(exempt(a) && exempt(b))
                && !(a.starts_with("boiloff = ") && b.starts_with("boiloff = "))
        })
        .collect();
    assert!(
        unexplained.is_empty(),
        "the demo differs from its twin somewhere other than the `boiloff` key and the \
         plant's own name: {unexplained:?}. Any difference in the NUMBERS below would then \
         be attributable to the plant rather than to the key under test"
    );
    assert!(
        differing
            .iter()
            .any(|(a, b)| a.starts_with("boiloff = ") && b.starts_with("boiloff = ")),
        "the two files agree about `boiloff`, so this pair cannot discriminate anything"
    );

    // And now the numbers. `f_in = 0.236` was measured in M12.0 on the arriving
    // draw; the assertion is that roughly that share of the naphtha product
    // leaves as vapour, rather than merely that something changed.
    let mut boiling = build(DEMO);
    let mut inert = build(ANCHOR);
    run(&mut boiling, TICKS);
    run(&mut inert, TICKS);
    let (hot, hot_x, hot_t) = tank(&boiling.snapshot(), BOILING_TANK);
    let (cold, cold_x, cold_t) = tank(&inert.snapshot(), BOILING_TANK);

    assert!(
        hot < cold,
        "the boiling plant's naphtha tank holds {hot:.3} kg and the inert one holds \
         {cold:.3} kg. A key that is parsed and then ignored produces exactly this: an \
         all-identical corpus and a demo that matches its twin"
    );
    assert_ne!(
        hot_t, cold_t,
        "the two tanks are at the same temperature, so the term is not parking one of them \
         on its bubble point"
    );
    assert_ne!(hot_x, cold_x, "the two tanks hold the same mixture");

    // **The size of the difference, against the plant's own numbers rather than
    // against a remembered figure.** At steady state the tank is held ON its
    // bubble point, so the superheat one tick's inflow adds is what that tick
    // must boil away — and the inventory cancels out of the ratio:
    //
    //     vent / draw  =  c̄p · (T_draw − T_bub) / Δh̄_vap  =  f_in
    //
    // **Not a tautology**, which is what the note's own gate 4 warns about: the
    // model never sees `T_draw`. It is handed the tank's temperature and
    // composition and nothing else, so this relation ties a rate the model
    // produced to a temperature it never read.
    //
    // **Correction to §14's second table, measured here.** The note predicts
    // `f_in = 0.236` from `T_bub = 354.3 K`, which is the bubble point of the
    // tank's DECLARED pure `light_naphtha` — the tick-0 composition, and exactly
    // the trap the note names one section earlier for gate 1. Against the
    // mixture the tank actually holds (`T_bub = 369.77 K` at tick 6 000) the
    // prediction is `f_in = 0.1282`, and the plant delivers `0.1090`.
    let snapshot = boiling.snapshot();
    let vented = edge(&snapshot, VENT).stream.mass_flow.value();
    let draw = edge(&snapshot, "naphtha_draw");
    let drawn = draw.stream.mass_flow.value();
    let share = vented / drawn;

    let (_, fractions, tank_t) = tank(&snapshot, BOILING_TANK);
    let composition =
        refinery_core::components::Composition::from_weights(&fractions).expect("a composition");
    let bubble = bubble_point_of(&boiling.slate, &fractions);
    let thermo = TroutonThermo::new();
    let mut latent = 0.0;
    for (c, &w) in fractions.iter().enumerate() {
        if w > 0.0 {
            latent += w * thermo
                .dh_vap(&boiling.slate, c, Kelvin(tank_t))
                .expect("trouton answers")
                .value()
                / boiling.slate.get(c).molar_mass.value();
        }
    }
    let cp = composition.mixture_cp(&boiling.slate).value();
    let predicted = cp * (draw.stream.temperature.value() - bubble) / latent;

    assert!(
        share > 0.05,
        "only {:.2}% of the naphtha draw boils off. A key that is PARSED AND THEN IGNORED          produces exactly this — an all-identical corpus and a demo that matches its twin          — which is what happened to `thermo` from M1 to M7.2",
        100.0 * share
    );
    // 25%: the measured gap is 15%, and its direction and mechanism are known —
    // the tank's bubble point RISES as the flash enriches it, and superheat
    // spent lifting the bubble point is superheat that never has to be boiled
    // away. A band tight enough to exclude that would be fitted to one tick of
    // one plant.
    assert!(
        (share - predicted).abs() <= 0.25 * predicted,
        "the vent carries {:.4} of the draw and the plant's own numbers predict          {predicted:.4} — draw at {:.2} K into a tank boiling at {bubble:.2} K,          c̄p = {cp:.1} J/kg/K, Δh̄_vap = {latent:.0} J/kg. The rate and the temperature it          is supposed to follow from have parted company",
        share,
        draw.stream.temperature.value()
    );
}

// ---------------------------------------------------------------------------
// The refusals — a fifth key must not repeat what happened to the first
// ---------------------------------------------------------------------------

/// An unknown value is refused, and the refusal is EXERCISED.
///
/// `thermo` was parsed and then ignored from M1 to M7.2, so `thermo =
/// "nonsense"` loaded a working plant — found by wiring a neighbour key beside
/// it rather than by a test, because nothing reached the value so nothing could
/// fail on it. A key whose value is read only when some other key is set is born
/// in that state.
#[test]
fn an_unknown_boiloff_model_is_refused() {
    let src = ANCHOR.replace("boiloff = \"none\"", "boiloff = \"evaporation\"");
    let file = refinery_scenarios::load_str(&src).expect("the document still parses");
    let err = refinery_scenarios::build_engine(&file)
        .err()
        .expect("an unknown boil-off model must be refused at load");
    let message = err.to_string();
    assert!(
        message.contains("evaporation") && message.contains("none, flash"),
        "the refusal must name the value and the valid ones: {message}"
    );
}

/// `boiloff = "flash"` with `thermo = "constant"` is refused at LOAD.
///
/// Without it the pairing loads happily and the plant runs — and boils nothing,
/// because `ConstantThermo::bubble_pressure` is an `Err` the model reads as
/// "this fidelity cannot answer". That is worse than the cascade's version of
/// the same mistake, which at least fails loudly at tick 1: here a scenario
/// author would see a plant that selects `flash` and behaves exactly like
/// `none`.
#[test]
fn flashing_a_model_with_no_equilibrium_is_refused() {
    const PLANT: &str = include_str!("../../../scenarios/tank_pump_valve.toml");
    // `tank_pump_valve.toml` already declares `thermo = "constant"`, which is
    // the half of the pairing that makes it illegal — so only the boil-off key
    // is added, and the plant's own declaration is asserted rather than
    // duplicated (a second `thermo` key is a TOML parse error, not a fidelity
    // refusal, and would have this gate passing for the wrong reason).
    assert!(
        PLANT.contains("thermo = \"constant\""),
        "this plant no longer declares the constant thermo model, so the pairing under          test is not the one being built"
    );
    let src = PLANT.replace(
        "[fidelity]",
        "[fidelity]
boiloff = \"flash\"",
    );
    let file = refinery_scenarios::load_str(&src).expect("the document parses");
    let err = refinery_scenarios::build_engine(&file)
        .err()
        .expect("flash + constant must be refused at load");
    let message = err.to_string();
    assert!(
        message.contains("boiloff") && message.contains("trouton"),
        "the refusal must name the pairing and the way out of it: {message}"
    );
}

/// The pressure a vented tank boils at is its BLANKET pressure, not the
/// hydrostatic pressure at its floor.
///
/// A correction to §14 fork 3, which writes `T_bub(P_node, x)`: a tank's node
/// pressure is its bottom pressure, up to 80 kPa above atmospheric on this
/// geometry — a different question by several kelvin of bubble point, and one
/// that would make the term a function of LEVEL, so a tank would stop boiling as
/// it filled. `NodeKind::Tank` is documented as vented; its free surface is at
/// atmospheric.
#[test]
fn a_vented_tank_boils_at_atmospheric_not_at_its_own_floor() {
    let mut engine = build(DEMO);
    run(&mut engine, TICKS);
    let snapshot = engine.snapshot();
    let (_, fractions, temperature) = tank(&snapshot, BOILING_TANK);
    let floor = node(&snapshot, BOILING_TANK).pressure_pa;

    // The control: the two pressures are far enough apart to tell the two
    // answers apart at all.
    assert!(
        floor > Pascal(refinery_core::units::P_ATM.value()).value() + 1_000.0,
        "the tank's floor is at {floor:.0} Pa, barely above atmospheric, so this gate \
         cannot discriminate"
    );
    let at_atmospheric = bubble_point_of(&engine.slate, &fractions);
    assert!(
        (temperature - at_atmospheric).abs() < 0.5,
        "the tank parks at {temperature:.3} K and its bubble point at ATMOSPHERIC is \
         {at_atmospheric:.3} K"
    );
}
