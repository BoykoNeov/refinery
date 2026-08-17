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

use refinery_core::engine::Engine;
use refinery_core::error::SimError;
use refinery_core::graph::NodeKind;
use refinery_scenarios::{build_engine, load_str};

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
