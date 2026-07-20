//! M3.1: the `[[components]]` slate table and per-node compositions, as wired
//! through the loader.
//!
//! `core::components` already had `Slate`, `PseudoComponent` and `Composition`
//! before this milestone, and their arithmetic is unit-tested there. What was
//! missing — and what these gates cover — is that nothing ever BUILT a slate of
//! more than one component: `build_engine` hardcoded `Slate::water_only()` and
//! every material node got pure water regardless of what the file said.
//!
//! These tests therefore pin the loader, not the mixing rules: file order
//! becoming canonical slate order, compositions resolving by NAME against that
//! order, normalization, and the refusals that stop a plausible-looking wrong
//! plant from running.

use refinery_scenarios::{build_engine, load_str};

/// Two cuts with deliberately round densities, so a mixture density can be
/// derived by hand without the code's help. See `tank_mass_uses_its_own_density`.
const TWO_CUT_SLATE: &str = r#"
[meta]
name = "two_cut"

[simulation]
dt = 0.1

[fidelity]
flow = "newton"

[[components]]
name = "light"
tb_c = 65.0
molar_mass_kg_per_mol = 0.100
density_kg_per_m3 = 700.0
cp_j_per_kg_k = 2100.0

[[components]]
name = "heavy"
tb_c = 340.0
molar_mass_kg_per_mol = 0.400
density_kg_per_m3 = 900.0
cp_j_per_kg_k = 1900.0
"#;

/// A minimal pressure-anchored plant: source → tank. Appended to a slate.
fn with_plant(slate: &str, source_composition: &str, tank_composition: &str) -> String {
    format!(
        r#"{slate}
[nodes.feed]
type = "source"
pressure_bar = 5.0
temperature_c = 20.0
{source_composition}

[nodes.storage]
type = "tank"
area_m2 = 10.0
height_m = 12.0
initial_level_m = 2.0
temperature_c = 20.0
{tank_composition}

[[pipes]]
name = "feed_line"
from = "feed"
to = "storage"
length_m = 20.0
diameter_m = 0.10
"#
    )
}

fn build(src: &str) -> Result<refinery_core::engine::Engine, refinery_core::error::SimError> {
    build_engine(&load_str(src).expect("scenario should parse"))
}

/// `Engine` is not `Debug` (it holds boxed solver traits), so `expect_err` is
/// unavailable — unwrap the refusal by hand.
fn build_err(src: &str, context: &str) -> String {
    match build(src) {
        Ok(_) => panic!("{context}: the plant built instead of being refused"),
        Err(e) => e.to_string(),
    }
}

/// The mass fractions on a named source node.
fn source_fractions(engine: &refinery_core::engine::Engine, name: &str) -> Vec<f64> {
    let id = engine.graph.find_node(name).expect("node should exist");
    let refinery_core::graph::NodeKind::Source { composition, .. } = &engine.graph.node(id).kind
    else {
        panic!("'{name}' should be a source");
    };
    composition.fractions().to_vec()
}

/// File order is canonical slate order, and a composition written by NAME lands
/// on the right POSITION regardless of the order the names appear in.
///
/// This is the gate that separates "resolved by name" from "zipped in order":
/// the source's composition below lists `heavy` first, so a loader that paired
/// weights with slate positions in the order it read them would put 0.25 on
/// `light` and 0.75 on `heavy` — exactly backwards, and still summing to 1.
#[test]
fn composition_resolves_by_name_not_by_position() {
    let engine = build(&with_plant(
        TWO_CUT_SLATE,
        r#"composition = { heavy = 0.25, light = 0.75 }"#,
        r#"composition = { light = 1.0 }"#,
    ))
    .expect("plant should build");

    assert_eq!(engine.slate.len(), 2);
    assert_eq!(
        engine.slate.get(0).name,
        "light",
        "file order is slate order"
    );
    assert_eq!(engine.slate.get(1).name, "heavy");

    assert_eq!(
        source_fractions(&engine, "feed"),
        &[0.75, 0.25],
        "weights must follow their names, not the order they were written"
    );
}

/// Weights are normalized, so a file may state fractions or raw mass amounts and
/// both mean the same feed. Pins that the two spellings are bit-identical.
#[test]
fn weights_are_normalized() {
    let fractions = build(&with_plant(
        TWO_CUT_SLATE,
        r#"composition = { light = 0.75, heavy = 0.25 }"#,
        r#"composition = { light = 1.0 }"#,
    ))
    .expect("plant should build");
    let amounts = build(&with_plant(
        TWO_CUT_SLATE,
        r#"composition = { light = 30.0, heavy = 10.0 }"#,
        r#"composition = { light = 4.0 }"#,
    ))
    .expect("plant should build");

    assert_eq!(
        source_fractions(&fractions, "feed"),
        source_fractions(&amounts, "feed")
    );
}

/// A tank's initial mass is `ρ_mixture · A · h` at the density of its OWN
/// contents — not at water's.
///
/// The expected number is derived here from the ideal-liquid blending rule
/// (volume-fraction weighting, i.e. mass-weighted `1/ρ`) rather than by calling
/// `mixture_density`, so a bug in that rule cannot appear on both sides:
///
///   1/ρ = 0.5/700 + 0.5/900  ⇒  ρ = 2·700·900/(700+900) = 787.5 kg/m³
///
/// which is the harmonic mean at a 50/50 mass split, exact in binary at these
/// values. With A = 10 m² and h = 2 m the inventory is 15 750 kg exactly.
///
/// This gate exists because the loader computed tank mass at WATER's density
/// through M1 and M2 — invisible while every slate had one component, and a
/// ~27% inventory error the moment one didn't (19 960 kg here). Total mass
/// would still have been conserved from that wrong datum onward, so no
/// conservation test could have caught it.
#[test]
fn tank_mass_uses_its_own_density() {
    let engine = build(&with_plant(
        TWO_CUT_SLATE,
        r#"composition = { light = 1.0 }"#,
        r#"composition = { light = 1.0, heavy = 1.0 }"#,
    ))
    .expect("plant should build");

    let snapshot = engine.snapshot();
    let (_, tank) = snapshot
        .tanks
        .iter()
        .find(|(name, _)| name == "storage")
        .expect("storage tank");

    assert!(
        (tank.mass.value() - 15_750.0).abs() < 1e-9,
        "tank mass should be 787.5 kg/m³ × 10 m² × 2 m = 15750 kg, got {}",
        tank.mass.value()
    );
    // Guard the guard: if these two densities ever coincide with water's the
    // test above would pass for the wrong reason.
    assert!(
        (tank.mass.value() - 998.0 * 20.0).abs() > 1000.0,
        "the water-density answer must be far from the right one, or this test \
         cannot discriminate"
    );
}

/// A scenario with no `[[components]]` table is the water-only slate, and its
/// material nodes need no composition — that default is what keeps every
/// pre-M3 file meaning exactly what it meant.
#[test]
fn absent_slate_is_water_only_and_needs_no_composition() {
    let src = with_plant(
        r#"
[meta]
name = "no_slate"

[simulation]
dt = 0.1

[fidelity]
flow = "newton"
"#,
        "",
        "",
    );
    let engine = build(&src).expect("a slate-less plant should still build");
    assert_eq!(engine.slate.len(), 1);
    assert_eq!(engine.slate.get(0).name, "water");
}

/// On a real slate an absent composition is REFUSED, not defaulted to the first
/// cut. Defaulting would make a crude source pure light naphtha — a plant that
/// runs, converges, and is wrong.
#[test]
fn missing_composition_on_a_multi_component_slate_is_refused() {
    let message = build_err(
        &with_plant(TWO_CUT_SLATE, "", r#"composition = { light = 1.0 }"#),
        "a source with no composition on a 2-component slate",
    );
    assert!(
        message.contains("feed") && message.contains("no composition"),
        "the error should name the node and the missing composition: {message}"
    );
}

/// A typo'd component name is refused rather than ignored. Silently dropping it
/// would renormalize the remaining weights to 1 and yield a different feed than
/// the file describes.
#[test]
fn unknown_component_name_is_refused() {
    let message = build_err(
        &with_plant(
            TWO_CUT_SLATE,
            r#"composition = { light = 0.5, kerosine = 0.5 }"#,
            r#"composition = { light = 1.0 }"#,
        ),
        "an unknown component name",
    );
    assert!(
        message.contains("kerosine") && message.contains("light, heavy"),
        "the error should name the typo and list the real slate: {message}"
    );
}

/// Duplicate component names are refused: compositions are written by name, so
/// two cuts sharing one would make every mention of it ambiguous — and
/// `Slate::index_of` would quietly resolve all of them to the first.
#[test]
fn duplicate_component_names_are_refused() {
    let slate = TWO_CUT_SLATE.replace(r#"name = "heavy""#, r#"name = "light""#);
    let message = build_err(
        &with_plant(
            &slate,
            r#"composition = { light = 1.0 }"#,
            r#"composition = { light = 1.0 }"#,
        ),
        "a duplicated component name",
    );
    assert!(
        message.contains("defined twice"),
        "unexpected error: {message}"
    );
}

/// Every component property is a positive magnitude. A zero density divides by
/// zero in the mixture rule; a zero cp turns any heat input into an infinite
/// temperature rise. Each is checked separately so one guard cannot stand in
/// for the others.
#[test]
fn non_positive_component_properties_are_refused() {
    for (field, replacement) in [
        ("density_kg_per_m3 = 700.0", "density_kg_per_m3 = 0.0"),
        ("cp_j_per_kg_k = 2100.0", "cp_j_per_kg_k = 0.0"),
        (
            "molar_mass_kg_per_mol = 0.100",
            "molar_mass_kg_per_mol = -0.1",
        ),
        // Tb is checked in KELVIN: −273.15 °C is absolute zero, and a cut
        // boiling below it is not a physical cut. Writing a negative °C here
        // instead would be a legitimate (if cold) boiling point.
        ("tb_c = 65.0", "tb_c = -300.0"),
    ] {
        let slate = TWO_CUT_SLATE.replace(field, replacement);
        let message = build_err(
            &with_plant(
                &slate,
                r#"composition = { light = 1.0 }"#,
                r#"composition = { light = 1.0 }"#,
            ),
            replacement,
        );
        assert!(
            message.contains("non-positive"),
            "'{replacement}' should be refused as non-positive, got: {message}"
        );
    }
}
