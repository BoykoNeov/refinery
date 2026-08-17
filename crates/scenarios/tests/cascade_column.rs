//! M7.3: the stage cascade AS WIRED — loader → hydraulic solve → composition
//! sweep → prescribed draw flows.
//!
//! The cascade's algebra is gated in isolation in
//! `refinery-solvers/tests/reference/cascade.rs`, against Fenske, the M7.2 flash
//! and the null case. What those cannot see, and these cover, is the column as a
//! plant: a scenario file selecting `separation = "cascade"` with
//! `thermo = "trouton"`, a real solve handing it a feed flow the file never
//! states, the draw flows coming out as the declared mass ratios times that flow,
//! and every fidelity mismatch refused when the file is READ rather than on the
//! first tick.
//!
//! `Engine` is not `Debug` (boxed solver traits), so refusals are unwrapped by
//! hand rather than with `expect_err`.

use refinery_core::components::{Composition, Slate};
use refinery_core::engine::Engine;
use refinery_core::error::SimError;
use refinery_core::graph::NodeKind;
use refinery_core::traits::ThermoModel;
use refinery_core::units::{Kelvin, Pascal, T_AMBIENT};
use refinery_scenarios::{build_engine, load_str};
use refinery_solvers::{MoleFractions, TroutonThermo};

/// A two-cut slate whose molar masses differ by 2×, so the mass ⇄ mole boundary
/// inside the cascade is live rather than degenerate. The boiling points are what
/// `TroutonThermo` turns into K-values.
const TWO_CUT_SLATE: &str = r#"
[[components]]
name = "light"
tb_c = 60.0
molar_mass_kg_per_mol = 0.100
density_kg_per_m3 = 700.0
cp_j_per_kg_k = 2000.0

[[components]]
name = "heavy"
tb_c = 160.0
molar_mass_kg_per_mol = 0.200
density_kg_per_m3 = 900.0
cp_j_per_kg_k = 2000.0
"#;

fn build(src: &str) -> Result<Engine, SimError> {
    build_engine(&load_str(src).expect("scenario should parse"))
}

fn build_err(src: &str, context: &str) -> String {
    match build(src) {
        Ok(_) => panic!("{context}: the plant built instead of being refused"),
        Err(e) => e.to_string(),
    }
}

fn edge_flow(engine: &Engine, name: &str) -> f64 {
    let eid = engine
        .graph
        .edge_ids()
        .find(|e| engine.graph.pipe(*e).name == name)
        .unwrap_or_else(|| panic!("edge '{name}' should exist"));
    engine.graph.pipe(eid).stream.mass_flow.value()
}

fn edge_composition(engine: &Engine, name: &str) -> Vec<f64> {
    let eid = engine
        .graph
        .edge_ids()
        .find(|e| engine.graph.pipe(*e).name == name)
        .unwrap();
    engine
        .graph
        .pipe(eid)
        .stream
        .composition
        .fractions()
        .to_vec()
}

/// A source → cascade column → two-tank plant. The pressure is above atmospheric
/// so the K-values are not all pinned at the cuts' own boiling points, and the
/// feed is hot enough for the light cut to be genuinely volatile.
fn cascade_plant(extra_fidelity: &str, column_body: &str) -> String {
    format!(
        r#"
[meta]
name = "cascade_ref"

[simulation]
dt = 0.1

[fidelity]
flow = "newton"
separation = "cascade"
{extra_fidelity}
{TWO_CUT_SLATE}

[nodes.feed]
type = "source"
pressure_bar = 5.0
temperature_c = 120.0
composition = {{ light = 0.5, heavy = 0.5 }}

[nodes.column]
type = "column"
pressure_bar = 1.5
{column_body}

[nodes.top_tank]
type = "tank"
area_m2 = 10.0
height_m = 12.0
initial_level_m = 0.5
temperature_c = 30.0
composition = {{ light = 1.0 }}

[nodes.bottom_tank]
type = "tank"
area_m2 = 10.0
height_m = 12.0
initial_level_m = 0.5
temperature_c = 30.0
composition = {{ heavy = 1.0 }}

[[pipes]]
name = "feed_line"
from = "feed"
to = "column"
length_m = 20.0
diameter_m = 0.12

[[pipes]]
name = "top_draw"
from = "column"
to = "top_tank"
length_m = 20.0
diameter_m = 0.10

[[pipes]]
name = "bottom_draw"
from = "column"
to = "bottom_tank"
length_m = 20.0
diameter_m = 0.10
"#
    )
}

/// The two-product column every test below deviates from by one key.
const HEALTHY_COLUMN: &str = r#"draws = [
    { outlet = "top_tank", stage = 0, draw_ratio = 0.4 },
    { outlet = "bottom_tank", stage = 6 },
]

[nodes.column.cascade]
stages = 6
feed_stage = 3
reflux_ratio = 2.0
"#;

fn healthy() -> String {
    cascade_plant("thermo = \"trouton\"", HEALTHY_COLUMN)
}

/// **The wiring gate.** A scenario file selects the cascade and the Trouton
/// K-values, the plant loads, and one real tick produces draw flows that are the
/// declared MASS ratios times a feed rate the file never states — the property
/// fork 3 exists to protect, and the one an absolute `D = 3.0 kg/s` would destroy.
///
/// The column also has to have actually separated. Without that assertion the
/// flow check would pass equally well for a splitter, a pipe, or a cascade that
/// silently returned its feed.
#[test]
fn a_cascade_column_loads_and_splits_a_feed_the_file_never_states() {
    let mut engine = build(&healthy()).expect("the cascade plant should build");
    engine.tick().expect("tick should converge");

    let feed = edge_flow(&engine, "feed_line");
    let top = edge_flow(&engine, "top_draw");
    let bottom = edge_flow(&engine, "bottom_draw");
    assert!(
        feed > 0.0,
        "the feed rate is hydraulically determined and must be positive, got {feed}"
    );
    approx::assert_relative_eq!(top, 0.4 * feed, max_relative = 1e-12);
    approx::assert_relative_eq!(bottom, 0.6 * feed, max_relative = 1e-12);
    approx::assert_relative_eq!(top + bottom, feed, max_relative = 1e-12);

    let top_composition = edge_composition(&engine, "top_draw");
    let bottom_composition = edge_composition(&engine, "bottom_draw");
    assert!(
        top_composition[0] > 0.9 && bottom_composition[0] < 0.2,
        "a six-stage column at R = 2 must concentrate the light cut overhead; got \
         top {top_composition:?} and bottom {bottom_composition:?}"
    );

    // Per-component mass across the column, to I7's own bound. Not free at this
    // fidelity — a splitter conserved it identically, a cascade converges to it.
    let feed_composition = edge_composition(&engine, "feed_line");
    for c in 0..2 {
        let out = top * top_composition[c] + bottom * bottom_composition[c];
        approx::assert_abs_diff_eq!(out, feed * feed_composition[c], epsilon = 1e-5);
    }
}

/// The cascade equipment written in the file reaches the graph unchanged, and it
/// is on the COLUMN rather than on the engine — per-node config, as fork 2 says.
#[test]
fn the_cascade_equipment_reaches_the_column() {
    let engine = build(&healthy()).unwrap();
    let column = engine.graph.find_node("column").unwrap();
    match &engine.graph.node(column).kind {
        NodeKind::Column { cascade, draws, .. } => {
            let spec = cascade
                .as_ref()
                .expect("a cascade column carries its equipment");
            assert_eq!(spec.stages, 6);
            assert_eq!(spec.feed_stage, 3);
            assert_eq!(spec.reflux_ratio, 2.0);
            assert_eq!(draws[0].stage, Some(0));
            assert_eq!(draws[0].draw_ratio, Some(0.4));
            assert_eq!(draws[1].stage, Some(6));
            assert_eq!(draws[1].draw_ratio, None);
            assert!(
                draws.iter().all(|d| d.upper_cut.is_none()),
                "a cascade draw carries no boiling-range top"
            );
        }
        other => panic!("expected a column, got {other:?}"),
    }
}

/// **The pairing M7.2 left owing.** `separation = "cascade"` with
/// `thermo = "constant"` is refused at LOAD. Until this arm existed the only
/// guard was `ConstantThermo::k_value`'s `Err`, which a plant would not reach
/// until its first tick — surfacing inside the composition sweep as a solver
/// failure rather than as the configuration mistake it is.
#[test]
fn a_cascade_on_the_constant_thermo_is_refused_at_load() {
    let m = build_err(
        &cascade_plant("thermo = \"constant\"", HEALTHY_COLUMN),
        "a cascade with no K-value",
    );
    assert!(
        m.contains("cascade") && m.contains("constant") && m.contains("trouton"),
        "the refusal must name both halves of the pairing and the fix, got: {m}"
    );

    // And the default is `constant`, so OMITTING the key must be refused the same
    // way. A refusal that only fires when the wrong value is written explicitly
    // would miss every file that never mentions thermo at all — which is every
    // file in this repo.
    let m = build_err(
        &cascade_plant("", HEALTHY_COLUMN),
        "a cascade with no thermo key",
    );
    assert!(
        m.contains("cascade") && m.contains("trouton"),
        "an omitted thermo key must be refused as the constant it defaults to, got: {m}"
    );
}

/// `thermo = "trouton"` is selectable, and an unknown thermo still lists every
/// valid value — the arm that would rot if a model were added to `build_engine`
/// and not to the message beside it.
#[test]
fn the_thermo_arm_offers_both_models() {
    build(&healthy()).expect("trouton must be selectable");
    let m = build_err(
        &cascade_plant("thermo = \"phlogiston\"", HEALTHY_COLUMN),
        "an unimplemented thermo model",
    );
    assert!(
        m.contains("phlogiston") && m.contains("constant") && m.contains("trouton"),
        "the error must name the bad value and list every valid one, got: {m}"
    );
}

/// **Declared-iff-used, cascade → splitter.** A cascade column may not carry the
/// splitter's fields: they would be authoritative-looking numbers the running
/// model never reads, which is exactly what M7.1 measured on `smearing_k`.
#[test]
fn a_cascade_column_may_not_carry_the_splitters_fields() {
    let with_cut = HEALTHY_COLUMN.replace(
        r#"{ outlet = "top_tank", stage = 0, draw_ratio = 0.4 }"#,
        r#"{ outlet = "top_tank", stage = 0, draw_ratio = 0.4, up_to_c = 100.0 }"#,
    );
    let m = build_err(
        &cascade_plant("thermo = \"trouton\"", &with_cut),
        "a cascade draw with a boiling-range top",
    );
    assert!(
        m.contains("up_to_c") && m.contains("STAGE"),
        "the refusal must name the field and the fidelity that owns it, got: {m}"
    );

    let with_smearing = format!("smearing_k = 25.0\n{HEALTHY_COLUMN}");
    let m = build_err(
        &cascade_plant("thermo = \"trouton\"", &with_smearing),
        "a cascade column with a smearing width",
    );
    assert!(
        m.contains("smearing_k") && m.contains("cut point"),
        "the refusal must say why a cascade has nothing to smear, got: {m}"
    );
}

// ---------------------------------------------------------------------------
// M7.4a — the draws leave at their tray temperatures.
// ---------------------------------------------------------------------------

/// The bubble point of a MASS composition at `pressure` [K], by bisection on
/// `Σ Kᵢ(T)·xᵢ = 1` over mole fractions.
///
/// Deliberately a second implementation rather than a call into the cascade's own
/// `bubble_point`, which is private: this is the gate's independent side. It uses
/// only the published `ThermoModel::k_value` and the mass ⇄ mole boundary, so a
/// cascade that computed its profile wrongly would disagree with it.
///
/// `Σ Kᵢ·xᵢ` is monotone increasing in `T` (every `K` is), so bisection converges
/// on the single root; the bracket is wide enough to hold both cuts of the test
/// slate at any pressure this file uses.
fn bubble_point_of(slate: &Slate, mass: &Composition, pressure: Pascal) -> f64 {
    let thermo = TroutonThermo::new();
    let moles = MoleFractions::from_mass(mass, slate).expect("a draw carries a valid composition");
    let sum_kx = |t: f64| -> f64 {
        moles
            .fractions()
            .iter()
            .enumerate()
            .map(|(c, x)| {
                x * thermo
                    .k_value(slate, c, Kelvin(t), pressure)
                    .expect("K at T > 0")
            })
            .sum()
    };

    let (mut low, mut high) = (200.0_f64, 800.0_f64);
    assert!(
        sum_kx(low) < 1.0 && sum_kx(high) > 1.0,
        "the bracket must straddle the bubble point"
    );
    for _ in 0..200 {
        let mid = 0.5 * (low + high);
        if sum_kx(mid) < 1.0 {
            low = mid;
        } else {
            high = mid;
        }
    }
    0.5 * (low + high)
}

fn edge_temperature(engine: &Engine, name: &str) -> f64 {
    let eid = engine
        .graph
        .edge_ids()
        .find(|e| engine.graph.pipe(*e).name == name)
        .unwrap_or_else(|| panic!("edge '{name}' should exist"));
    engine.graph.pipe(eid).stream.temperature.value()
}

/// **The M7.4a gate.** Each draw leaves at ITS OWN tray's temperature, and the two
/// disagree.
///
/// Three properties at once, and the third is what makes the first two non-vacuous:
///
/// 1. **Ordered, strictly.** The overhead draw is colder than the bottoms. An
///    unwired column arm hands every draw the column's single mixed temperature, so
///    a non-strict comparison would pass on the very bug this slice fixes.
/// 2. **Neither is the feed's.** The feed enters at 120 °C and the column has no
///    other inflow, so its mixed temperature IS the feed's, and that is the exact
///    number an unwired arm returns for every draw. Both must land clear of it.
///    They land clear on the SAME side, which is a property of this fixture rather
///    than of columns and is worth knowing before M7.4b: see the assertion.
/// 3. **Each equals the bubble point of its own composition**, recomputed here from
///    the published K-values rather than by calling the cascade's own (private)
///    routine. This is the assertion with real physics in it: an off-by-one in the
///    one-based-file to zero-based-profile translation leaves both temperatures
///    real, both ordered and both away from the feed, and fails here.
///
///    It is **not** the sole catcher of anything measured — mutating that
///    translation also fails the wired plant's split-and-balance test, and
///    mis-taking the distillate's composition also fails the M7.3 flash reduction.
///    What this assertion holds is INDEPENDENCE: a second derivation of the
///    saturated-liquid contract, agreeing with the cascade's to 1e-6, so a wrong
///    profile has to fool two unrelated computations. Said plainly because the
///    first version of this comment claimed unique coverage it had not measured.
///
/// The contract behind (3): under a total condenser with liquid draws, every draw
/// leaves as a saturated liquid, so its temperature is the bubble point of the
/// composition it carries — for the distillate its own, for a stage draw that
/// stage's liquid. That holds for both draw locations, which is why one identity
/// covers both.
#[test]
fn each_draw_leaves_at_its_own_tray_temperature() {
    let mut engine = build(&healthy()).expect("the cascade plant should build");
    engine.tick().expect("tick should converge");

    let column = engine.graph.find_node("column").unwrap();
    let pressure = match &engine.graph.node(column).kind {
        NodeKind::Column { pressure, .. } => *pressure,
        other => panic!("expected a column, got {other:?}"),
    };

    let top = edge_temperature(&engine, "top_draw");
    let bottom = edge_temperature(&engine, "bottom_draw");
    let feed = edge_temperature(&engine, "feed_line");

    // (1) Strict, not `<=`: equal temperatures are exactly the unwired case.
    assert!(
        top < bottom,
        "the overhead draw must be strictly colder than the bottoms; got top {top} K and \
         bottom {bottom} K — equal values mean the column arm is not wired at all"
    );

    // (2) Neither is the feed's. The column's only inflow is the feed line, so its
    //     mixed temperature IS the feed's, and a draw still reading the node rather
    //     than the tray would sit exactly on this number.
    //
    //     Both land BELOW it, and that is a fault in this FIXTURE rather than a
    //     property of columns. The feed enters at 120 °C, above the bubble point of
    //     a 50/50 mix at 1.5 bar, so it is superheated relative to the column it
    //     feeds — and constant molar overflow admits only a saturated-liquid feed
    //     (DESIGN §5, M7.3 correction 5). Nothing enforces that today, so this
    //     plant has always been off-model. **M7.4b makes it an `Err` and moves this
    //     fixture's feed onto its bubble point**; when it does, this assertion is
    //     expected to keep holding with the two draws straddling the feed instead.
    assert!(
        (feed - top).abs() > 1.0 && (feed - bottom).abs() > 1.0,
        "neither draw may sit on the column's mixed feed temperature ({feed} K), which is \
         what an unwired arm returns; got top {top} K and bottom {bottom} K"
    );

    // (3) Each against the bubble point of the composition it actually carries.
    for (name, temperature) in [("top_draw", top), ("bottom_draw", bottom)] {
        let carried = Composition::from_weights(&edge_composition(&engine, name))
            .expect("a draw carries a valid composition");
        let saturated = bubble_point_of(&engine.slate, &carried, pressure);
        approx::assert_relative_eq!(temperature, saturated, max_relative = 1e-6);
    }
}

/// **Where the arm sits, not just that it exists.** A draw pipe with a real
/// `ambient_ua` transforms its inlet on the way — and the inlet it starts from must
/// be the TRAY's temperature, not the column's mixed one.
///
/// The first version of this comment claimed the test above would pass under a
/// MISPLACED arm — one that resolves the tray only on the early "this node is the
/// upwind end" return and leaves the downstream branch reading the node map.
/// **Mutated, and the claim is false**: `Engine::tick` stores a pipe's temperature
/// from its DOWNSTREAM end (`engine.rs`, the transport loop), so the early return
/// is not the branch that test reads at all, and it fails on the misplacement too.
/// Recorded rather than quietly deleted — the reasoning was the reason this test
/// was written.
///
/// What it is actually for is narrower and still worth having. Every pipe in every
/// scenario file in this workspace has `ambient_ua = 0`, where the transform is the
/// identity; this is the ONLY place a column's draw pipe carries a live one. So it
/// is the only gate that the arm and the transform **compose** — that the number
/// the arm resolves is the number the transform starts from, rather than one it
/// resolves and then drops. From the tray (~346 K here) a mild heat loss lands just
/// below it; from the column's mixed ~393 K it lands just below THAT.
///
/// The `UA` is deliberately small. A large one drives the outlet toward ambient
/// from either inlet, which would make a wrong placement pass — the bound has to
/// be reached because the arm read the right inlet, not because the exponential
/// swamped the difference. The control below measures that directly.
#[test]
fn a_draw_pipes_ambient_transform_starts_from_the_tray() {
    let with_ua = healthy().replace(
        "name = \"top_draw\"\nfrom = \"column\"\nto = \"top_tank\"\nlength_m = 20.0\ndiameter_m = 0.10",
        "name = \"top_draw\"\nfrom = \"column\"\nto = \"top_tank\"\nlength_m = 20.0\ndiameter_m = 0.10\nambient_ua_w_per_k = 40.0",
    );
    assert!(
        with_ua.contains("ambient_ua_w_per_k"),
        "the fixture edit must have applied — a silent no-op replace would make this \
         test a copy of the one above"
    );

    let mut engine = build(&with_ua).expect("a draw pipe may carry an ambient UA");
    engine.tick().expect("tick should converge");

    let column = engine.graph.find_node("column").unwrap();
    let pressure = match &engine.graph.node(column).kind {
        NodeKind::Column { pressure, .. } => *pressure,
        other => panic!("expected a column, got {other:?}"),
    };

    let outlet = edge_temperature(&engine, "top_draw");
    let feed = edge_temperature(&engine, "feed_line");
    let carried = Composition::from_weights(&edge_composition(&engine, "top_draw"))
        .expect("a draw carries a valid composition");
    let tray = bubble_point_of(&engine.slate, &carried, pressure);

    // Cooling, so the outlet is below the inlet — whichever inlet was used. The
    // discriminating half is that it is below the TRAY: an inlet of the column's
    // mixed temperature would leave it stranded up near the feed.
    assert!(
        outlet < tray,
        "a draw pipe losing heat must leave below its tray temperature; got outlet \
         {outlet} K against a tray at {tray} K"
    );
    // The control: this `UA` is mild enough that starting from the column's mixed
    // temperature would NOT have decayed past the tray. Without this the assertion
    // above could be satisfied by a large-enough UA regardless of the inlet, which
    // is the "green for the wrong reason" trap.
    let ua_ok = tray + (feed - tray) * 0.5;
    assert!(
        outlet < ua_ok,
        "the UA must be mild enough that a wrong inlet would be visible: an outlet at \
         {outlet} K is not clear of the {ua_ok} K midpoint between the tray ({tray} K) \
         and the feed ({feed} K)"
    );
    assert!(
        outlet > T_AMBIENT.value(),
        "the analytic transform cannot cross ambient; got {outlet} K"
    );
}

/// **Declared-iff-used, splitter → cascade.** And the mirror: a cut-point column
/// may not carry the cascade's fields.
#[test]
fn a_cut_point_column_may_not_carry_the_cascades_fields() {
    let splitter_column = r#"smearing_k = 0.0
draws = [
    { outlet = "top_tank", up_to_c = 100.0 },
    { outlet = "bottom_tank" },
]
"#;
    // The control: the same plant on the splitter fidelity builds.
    let source = cascade_plant("", splitter_column).replace("separation = \"cascade\"\n", "");
    build(&source).expect("the same plant on the cut-point fidelity must build");

    for (key, snippet) in [
        (
            "stage",
            r#"{ outlet = "top_tank", up_to_c = 100.0, stage = 0 }"#,
        ),
        (
            "draw_ratio",
            r#"{ outlet = "top_tank", up_to_c = 100.0, draw_ratio = 0.4 }"#,
        ),
        (
            "phase",
            r#"{ outlet = "top_tank", up_to_c = 100.0, phase = "liquid" }"#,
        ),
    ] {
        let m = build_err(
            &source.replace(r#"{ outlet = "top_tank", up_to_c = 100.0 }"#, snippet),
            "a cut-point draw with a cascade field",
        );
        assert!(
            m.contains(key) && m.contains("cascade"),
            "the refusal must name {key} and the fidelity that owns it, got: {m}"
        );
    }

    let with_block = format!(
        "{splitter_column}\n[nodes.column.cascade]\nstages = 4\nfeed_stage = 2\nreflux_ratio = 1.0\n"
    );
    let m = build_err(
        &cascade_plant("", &with_block).replace("separation = \"cascade\"\n", ""),
        "a cut-point column with cascade equipment",
    );
    assert!(
        m.contains("cascade") && m.contains("boiling range"),
        "the refusal must say the splitter would read none of it, got: {m}"
    );
}

/// **Fork 0's two narrowed deferrals, each refused at load with its reason.**
///
/// M7's whole scope boundary is bought by one condition: with a total condenser
/// and all-liquid draws, every kilogram vaporized inside the column condenses
/// inside it, so the latent flows cancel and the external balance stays purely
/// sensible against this workspace's single enthalpy datum. A partial condenser
/// or a vapour side draw breaks that. Both are things a FILE can say — which is
/// the point: a refusal of something the format cannot express is not a refusal.
#[test]
fn a_partial_condenser_and_a_vapour_draw_are_refused_naming_the_deferral() {
    let partial = HEALTHY_COLUMN.replace(
        "reflux_ratio = 2.0",
        "reflux_ratio = 2.0\ncondenser = \"partial\"",
    );
    let m = build_err(
        &cascade_plant("thermo = \"trouton\"", &partial),
        "a partial condenser",
    );
    assert!(
        m.contains("partial") && m.contains("latent") && m.contains("Two-phase"),
        "the refusal must name the condenser type and the deferral it narrows to, got: {m}"
    );

    // Written on a SIDE draw rather than the distillate, so it is the draw's own
    // arm being reached and not the condenser's.
    let vapour_side = r#"draws = [
    { outlet = "top_tank", stage = 0, draw_ratio = 0.4 },
    { outlet = "bottom_tank", stage = 6, phase = "vapour" },
]

[nodes.column.cascade]
stages = 6
feed_stage = 3
reflux_ratio = 2.0
"#;
    let m = build_err(
        &cascade_plant("thermo = \"trouton\"", vapour_side),
        "a vapour side draw",
    );
    assert!(
        m.contains("vapour") && m.contains("latent") && m.contains("Two-phase"),
        "the refusal must name the phase and the deferral it narrows to, got: {m}"
    );

    // The explicit liquid spelling is a no-op, not an error — otherwise the key
    // would be write-only and nobody could say what they mean.
    let explicit_liquid = HEALTHY_COLUMN.replace(
        r#"{ outlet = "bottom_tank", stage = 6 }"#,
        r#"{ outlet = "bottom_tank", stage = 6, phase = "liquid" }"#,
    );
    build(&cascade_plant("thermo = \"trouton\"", &explicit_liquid))
        .expect("an explicitly liquid draw is the supported case");
}

/// A malformed cascade geometry is refused at LOAD, and the message carries both
/// the model's reason and the loader's column name.
///
/// The loader delegates this to `StageCascade::validate` — the same code
/// `separate` runs — so the two cannot drift apart. This test is what pins that
/// the delegation actually happens: a loader that skipped it would build these
/// plants and fail on the first tick instead.
#[test]
fn a_malformed_cascade_geometry_is_refused_at_load() {
    let cases = [
        (
            "the bottoms is not at the reboiler",
            HEALTHY_COLUMN.replace("stage = 6 }", "stage = 5 }"),
            "not 6",
        ),
        (
            "the distillate is not at the condenser",
            HEALTHY_COLUMN.replace("stage = 0,", "stage = 1,"),
            "not 0",
        ),
        (
            "the feed stage is off the end",
            HEALTHY_COLUMN.replace("feed_stage = 3", "feed_stage = 9"),
            "feed_stage",
        ),
        (
            "the reflux ratio is negative",
            HEALTHY_COLUMN.replace("reflux_ratio = 2.0", "reflux_ratio = -1.0"),
            "reflux_ratio",
        ),
        (
            "the bottoms declares a ratio",
            HEALTHY_COLUMN.replace("stage = 6 }", "stage = 6, draw_ratio = 0.6 }"),
            "must not",
        ),
        (
            "the distillate declares none",
            HEALTHY_COLUMN.replace(", draw_ratio = 0.4", ""),
            "declares no draw_ratio",
        ),
        // Not delegated: an omitted `stage` would otherwise be read as an
        // authoritative 0 and complain about the ORDERING instead.
        (
            "a draw declares no stage",
            HEALTHY_COLUMN.replace("stage = 0, ", ""),
            "declares no stage",
        ),
    ];
    for (what, body, expected) in cases {
        let m = build_err(&cascade_plant("thermo = \"trouton\"", &body), what);
        assert!(
            m.contains(expected) && m.contains("column 'column'"),
            "{what}: the refusal must carry the model's reason ({expected}) and the loader's \
             node name, got: {m}"
        );
    }
}
