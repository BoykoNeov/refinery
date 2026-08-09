//! M3.2 reference: the fixed cut-point column, AS WIRED — loader → hydraulic
//! solve → composition sweep → prescribed draw flows → transport.
//!
//! The separation math is hand-checked in isolation in `core::energy`'s
//! `column_separation_tests`. What those cannot see, and these cover, is the
//! column as a plant: the loader building it, a real solve handing it a feed flow
//! the file never states, the draw flows coming out as `splitᵢ · ṁ_feed` and not
//! a pressure-driven number, the draws carrying their cut compositions, and
//! per-component mass balancing across a feed whose composition MOVES tick to
//! tick — the case that separates the correct post-sweep split from the stale-
//! split bug the DESIGN note warned about.
//!
//! `Engine` is not `Debug` (boxed solver traits), so refusals are unwrapped by
//! hand rather than with `expect_err`.

use refinery_core::engine::Engine;
use refinery_core::error::SimError;
use refinery_core::graph::NodeKind;
use refinery_scenarios::{build_engine, load_str};

/// A three-cut slate at 100/200/300 °C. Only the boiling points matter to the
/// split; the other properties are plausible round numbers.
const THREE_CUT_SLATE: &str = r#"
[[components]]
name = "light"
tb_c = 100.0
molar_mass_kg_per_mol = 0.100
density_kg_per_m3 = 700.0
cp_j_per_kg_k = 2000.0

[[components]]
name = "middle"
tb_c = 200.0
molar_mass_kg_per_mol = 0.200
density_kg_per_m3 = 800.0
cp_j_per_kg_k = 2000.0

[[components]]
name = "heavy"
tb_c = 300.0
molar_mass_kg_per_mol = 0.300
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

/// Mass flow and composition of a named edge's stream, post-tick.
fn edge_stream(engine: &Engine, name: &str) -> (f64, Vec<f64>) {
    let eid = engine
        .graph
        .edge_ids()
        .find(|e| engine.graph.pipe(*e).name == name)
        .unwrap_or_else(|| panic!("edge '{name}' should exist"));
    let s = &engine.graph.pipe(eid).stream;
    (s.mass_flow.value(), s.composition.fractions().to_vec())
}

fn edge_temperature(engine: &Engine, name: &str) -> f64 {
    let eid = engine
        .graph
        .edge_ids()
        .find(|e| engine.graph.pipe(*e).name == name)
        .unwrap();
    engine.graph.pipe(eid).stream.temperature.value()
}

/// A source → column → three-tank plant. `smearing_k` and the draw pipe
/// diameters are parameters so the silent-bogus-flow gate can make one draw pipe
/// absurdly restrictive without changing anything else.
fn source_column_plant(smearing_k: f64, heavy_draw_diameter_m: f64) -> String {
    format!(
        r#"
[meta]
name = "column_ref"

[simulation]
dt = 0.1

[fidelity]
flow = "newton"
{THREE_CUT_SLATE}

[nodes.feed]
type = "source"
pressure_bar = 5.0
temperature_c = 250.0
composition = {{ light = 0.2, middle = 0.5, heavy = 0.3 }}

[nodes.column]
type = "column"
pressure_bar = 1.5
smearing_k = {smearing_k}
draws = [
    {{ outlet = "light_tank", up_to_c = 150.0 }},
    {{ outlet = "middle_tank", up_to_c = 250.0 }},
    {{ outlet = "heavy_tank" }},
]

[nodes.light_tank]
type = "tank"
area_m2 = 10.0
height_m = 12.0
initial_level_m = 0.5
temperature_c = 30.0
composition = {{ light = 1.0 }}

[nodes.middle_tank]
type = "tank"
area_m2 = 10.0
height_m = 12.0
initial_level_m = 0.5
temperature_c = 30.0
composition = {{ middle = 1.0 }}

[nodes.heavy_tank]
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
name = "light_draw"
from = "column"
to = "light_tank"
length_m = 20.0
diameter_m = 0.10

[[pipes]]
name = "middle_draw"
from = "column"
to = "middle_tank"
length_m = 20.0
diameter_m = 0.10

[[pipes]]
name = "heavy_draw"
from = "column"
to = "heavy_tank"
length_m = 20.0
diameter_m = {heavy_draw_diameter_m}
"#
    )
}

/// THE reference gate the DESIGN note demands: a per-draw COMPOSITION vector, not
/// a mass balance. Sharp splitter (smearing 0), feed [0.2, 0.5, 0.3] with cuts
/// at 150/250 °C, so each pseudo-component lands wholly in one band:
///
///   light draw  (< 150): pure light,  split 0.2
///   middle draw (150–250): pure middle, split 0.5
///   heavy draw  (> 250): pure heavy,  split 0.3
///
/// Read off the DRAW STREAMS after one tick, so this exercises the full path the
/// isolated `column_separation` test cannot: the loader building the column and
/// its draws, the solver handing it a feed flow the file never states, and
/// transport publishing each draw's cut composition through the column arm of
/// `edge_composition_at`.
#[test]
fn each_draw_carries_its_cut_composition() {
    let mut engine = build(&source_column_plant(0.0, 0.10)).expect("plant should build");
    engine.tick().expect("tick should converge");

    let (feed_flow, _) = edge_stream(&engine, "feed_line");
    assert!(
        feed_flow > 0.0,
        "the feed must actually flow, got {feed_flow}"
    );

    for (draw, want_split, want_comp) in [
        ("light_draw", 0.2, [1.0, 0.0, 0.0]),
        ("middle_draw", 0.5, [0.0, 1.0, 0.0]),
        ("heavy_draw", 0.3, [0.0, 0.0, 1.0]),
    ] {
        let (flow, comp) = edge_stream(&engine, draw);
        assert!(
            (flow - want_split * feed_flow).abs() < 1e-9,
            "{draw} flow should be {want_split}·feed = {}, got {flow}",
            want_split * feed_flow
        );
        for (c, w) in want_comp.iter().enumerate() {
            assert!(
                (comp[c] - w).abs() < 1e-12,
                "{draw} comp[{c}] should be {w}, got {}",
                comp[c]
            );
        }
    }
}

/// Mass neutrality as wired: `Σ draws = feed`, to machine precision, because
/// `Σ splitᵢ = 1`. Not a substitute for the composition gate above (a splitter
/// conserves total mass no matter how wrong its cut boundaries are) — it pins the
/// OTHER half, that the prescribed draws leave the fixed, zero-volume column with
/// nothing created or destroyed.
#[test]
fn the_draws_sum_to_the_feed() {
    let mut engine = build(&source_column_plant(0.0, 0.10)).expect("plant should build");
    engine.tick().expect("tick should converge");

    let (feed, _) = edge_stream(&engine, "feed_line");
    let draws: f64 = ["light_draw", "middle_draw", "heavy_draw"]
        .iter()
        .map(|d| edge_stream(&engine, d).0)
        .sum();
    assert!(
        (draws - feed).abs() < 1e-9,
        "the column must be mass-neutral: Σ draws {draws} vs feed {feed}"
    );
}

/// A draw is `splitᵢ · ṁ_feed`, prescribed by composition — NOT `ρ·branch.flow`,
/// its pipe's pressure-driven flow. Making the heavy draw's pipe absurdly
/// restrictive (a 5 mm bore against the others' 100 mm) would slash a
/// pressure-driven flow; here it must not move the draw at all. This pins the
/// engine's post-sweep override (`2b′`): the draw follows the split, not the
/// pipe. (The `edge_flows` GUARD that keeps the bogus number out of the SWEEP in
/// the first place is pinned separately below, since here the draws flow outward
/// and the override masks a missing guard.)
#[test]
fn a_draw_ignores_its_pipe_resistance() {
    let wide = {
        let mut e = build(&source_column_plant(0.0, 0.10)).unwrap();
        e.tick().unwrap();
        edge_stream(&e, "heavy_draw").0
    };
    let narrow = {
        let mut e = build(&source_column_plant(0.0, 0.005)).unwrap();
        e.tick().unwrap();
        edge_stream(&e, "heavy_draw").0
    };
    assert!(
        (wide - narrow).abs() < 1e-9,
        "the heavy draw is split·ṁ_feed and must not depend on its pipe bore: \
         wide-pipe {wide} vs 5 mm-pipe {narrow}"
    );
    assert!(wide > 0.0, "the draw should carry real flow, got {wide}");
}

/// The **silent bogus draw flow** guard in `network::edge_flows`, which has no
/// analogue in earlier milestones — and whose real hazard the override above
/// hides. When a product tank fills above the column pressure, its draw edge's
/// *pressure-driven* flow runs the WRONG WAY (tank → column). Left ungated, that
/// spurious inflow feeds the composition sweep and contaminates the column's
/// feed mix, so the very split that prescribes the draws is computed from a
/// polluted feed. The guard reports the draw as zero to the sweep instead.
///
/// Here `heavy_tank` starts at 10 m (bottom pressure ≈ 1.9 bar > the column's
/// 1.5 bar), so its draw would back-feed residue. With the guard, the split is
/// still the clean feed's: `light` = 0.2 of the feed, exactly as with an empty
/// tank. A draw being insensitive to its product tank's back-pressure is the
/// stated behaviour of this fidelity (the column keeps pushing into a full tank).
///
/// Falsified by removing the `is_column_draw_edge` guard: the back-fed residue
/// pollutes the sweep, the light split drops below 0.2, and this fails.
#[test]
fn a_full_product_tank_does_not_pollute_the_split() {
    // Same plant, but heavy_tank pre-filled so its draw would back-feed.
    let full = source_column_plant(0.0, 0.10).replace(
        r#"[nodes.heavy_tank]
type = "tank"
area_m2 = 10.0
height_m = 12.0
initial_level_m = 0.5"#,
        r#"[nodes.heavy_tank]
type = "tank"
area_m2 = 10.0
height_m = 12.0
initial_level_m = 10.0"#,
    );
    let mut engine = build(&full).expect("plant should build");
    engine.tick().expect("tick should converge");

    let (feed, _) = edge_stream(&engine, "feed_line");
    let (light, _) = edge_stream(&engine, "light_draw");
    assert!(
        (light / feed - 0.2).abs() < 1e-9,
        "the light split must stay the clean feed's 0.2 regardless of how full a \
         product tank is; got {} — a full tank back-fed the column's feed mix",
        light / feed
    );
}

/// Draws leave at the FEED temperature (DESIGN §5 — representing tray
/// temperatures needs the complex column). With insulated pipes every draw
/// stream is the column's feed-mix temperature, whatever the split.
///
/// **Since M5.1 this gate carries a second decision.** Every edge now dissipates
/// friction into its own stream — except a column draw, which reports `Φ = 0`
/// because its flow is *prescribed* (`splitᵢ·ṁ_feed`) rather than pressure-driven,
/// so `α·Q|Q|` is not its pressure drop and booking it would invent heat
/// (`network::edge_flows`). That choice is exactly what keeps the equality below
/// exact: give a draw a nonzero `Φ` and the three draws leave at three different
/// temperatures, none of them the feed's, and this fails. The feed LINE is an
/// ordinary pressure-driven edge and does warm itself, which is why the absolute
/// check below is a bound rather than an equality.
#[test]
fn draws_leave_at_the_feed_temperature() {
    let mut engine = build(&source_column_plant(0.0, 0.10)).expect("plant should build");
    engine.tick().expect("tick should converge");

    let feed_t = edge_temperature(&engine, "feed_line");
    for draw in ["light_draw", "middle_draw", "heavy_draw"] {
        let t = edge_temperature(&engine, draw);
        assert!(
            (t - feed_t).abs() < 1e-9,
            "{draw} should leave at the feed temperature {feed_t} K, got {t}"
        );
    }
    // The source declares 250 °C and nothing HEATS the stream; the feed line adds
    // only its own friction, two orders below anything a unit would do.
    assert!(
        feed_t > 523.15 && feed_t - 523.15 < 0.5,
        "the feed enters at 250 °C and only its own pipe friction is added, expected \
         just above 523.15 K, got {feed_t}"
    );
    // A draw books no friction of its own, which is what makes the equality above
    // exact rather than approximate — asserted directly so the reason is gated and
    // not merely commented.
    for draw in ["light_draw", "middle_draw", "heavy_draw"] {
        let phi = engine
            .snapshot()
            .edges
            .into_iter()
            .find(|e| e.name == draw)
            .expect("every draw must appear in the snapshot")
            .dissipation_w;
        assert_eq!(
            phi, 0.0,
            "a column draw's flow is prescribed, not pressure-driven, so it must book \
             no frictional dissipation; '{draw}' reports {phi} W"
        );
    }
}

/// Reverse feed flow is refused, not silently split. "The feed splits by boiling
/// range" names nothing when the feed runs backwards, and splitting a negative
/// flow would produce negative draws. Here the source sits BELOW the column
/// pressure, so the feed edge drives fluid out of the column back toward the
/// source — the tick must error rather than run.
#[test]
fn reverse_feed_flow_is_refused() {
    // Source at 1 bar, column at 5 bar: the feed edge runs backwards.
    let src = source_column_plant(0.0, 0.10)
        .replace("pressure_bar = 5.0", "pressure_bar = 1.0") // source
        .replace("pressure_bar = 1.5", "pressure_bar = 5.0"); // column
    let mut engine = build(&src).expect("the plant should still BUILD; the fault is at run time");
    match engine.tick() {
        Ok(()) => panic!("a backwards feed must error, not split a negative flow"),
        Err(e) => {
            let m = e.to_string();
            assert!(
                matches!(e, SimError::Numerical(_))
                    && m.contains("column")
                    && m.contains("reverse feed"),
                "the error must name the column and the reverse feed, got: {m}"
            );
        }
    }
}

/// The falsification gate the DESIGN note's "I7 green by construction" quietly
/// depends on, and the one a fixed-feed reference cannot provide. Per-component
/// mass balances at the column only if the draw FLOW split and the draw
/// COMPOSITION split are built from the SAME feed composition. On a fixed feed
/// the sweep-fresh and the previous-tick-stale feed compositions coincide, so a
/// stale-split bug is invisible; here the column is fed from a TANK being
/// displaced by a source of different composition, so the feed composition MOVES
/// every tick and the two split sources diverge.
///
/// The plant is closed but for the source: mass enters only there, leaves
/// nowhere, so per component `Σ_tanks inventory = initial + Σ source·dt`. The
/// column, being zero-volume, must pass every component straight through — which
/// it does only if `Σᵢ splitᵢ·compᵢ = f_feed` with a consistent split.
///
/// Falsified by the Option-A mutation (split the feed edge's STORED, one-tick-
/// stale composition in the flow computation): the per-component sum then drifts
/// by `ṁ_feed·dt·(f_stale − f_fresh)` every tick and this balance breaks — worst
/// on tick 0, where the stored composition is the stagnant seed, not the feed.
#[test]
fn per_component_mass_survives_a_moving_feed_composition() {
    // source (light-rich) displaces a heavy-rich charge in feed_tank, so the
    // column feed drifts tick to tick. All three draws stay active.
    let src = format!(
        r#"
[meta]
name = "column_transient"

[simulation]
dt = 0.1

[fidelity]
flow = "newton"
{THREE_CUT_SLATE}

[nodes.source]
type = "source"
pressure_bar = 8.0
temperature_c = 200.0
composition = {{ light = 0.6, middle = 0.3, heavy = 0.1 }}

[nodes.feed_tank]
type = "tank"
area_m2 = 6.0
height_m = 14.0
initial_level_m = 7.0
temperature_c = 200.0
composition = {{ light = 0.1, middle = 0.3, heavy = 0.6 }}

[nodes.column]
type = "column"
pressure_bar = 1.2
smearing_k = 15.0
draws = [
    {{ outlet = "light_tank", up_to_c = 150.0 }},
    {{ outlet = "middle_tank", up_to_c = 250.0 }},
    {{ outlet = "heavy_tank" }},
]

[nodes.light_tank]
type = "tank"
area_m2 = 10.0
height_m = 12.0
initial_level_m = 0.5
temperature_c = 30.0
composition = {{ light = 1.0 }}

[nodes.middle_tank]
type = "tank"
area_m2 = 10.0
height_m = 12.0
initial_level_m = 0.5
temperature_c = 30.0
composition = {{ middle = 1.0 }}

[nodes.heavy_tank]
type = "tank"
area_m2 = 10.0
height_m = 12.0
initial_level_m = 0.5
temperature_c = 30.0
composition = {{ heavy = 1.0 }}

[[pipes]]
name = "feed_fill"
from = "source"
to = "feed_tank"
length_m = 20.0
diameter_m = 0.10

[[pipes]]
name = "feed_line"
from = "feed_tank"
to = "column"
length_m = 20.0
diameter_m = 0.12

[[pipes]]
name = "light_draw"
from = "column"
to = "light_tank"
length_m = 20.0
diameter_m = 0.10

[[pipes]]
name = "middle_draw"
from = "column"
to = "middle_tank"
length_m = 20.0
diameter_m = 0.10

[[pipes]]
name = "heavy_draw"
from = "column"
to = "heavy_tank"
length_m = 20.0
diameter_m = 0.10
"#
    );
    let mut engine = build(&src).expect("transient plant should build");
    let dt = engine.dt().value();
    let n_components = 3;

    // The source composition is fixed; every kilogram it delivers is booked by
    // component through the feed_fill edge.
    let source_comp: Vec<f64> = {
        let id = engine.graph.find_node("source").unwrap();
        let NodeKind::Source { composition, .. } = &engine.graph.node(id).kind else {
            unreachable!()
        };
        composition.fractions().to_vec()
    };

    let tank_inventory = |e: &Engine, c: usize| -> f64 {
        ["feed_tank", "light_tank", "middle_tank", "heavy_tank"]
            .iter()
            .map(|name| {
                let id = e.graph.find_node(name).unwrap();
                match &e.graph.node(id).kind {
                    NodeKind::Tank(t) => t.mass.value() * t.composition.fractions()[c],
                    _ => unreachable!(),
                }
            })
            .sum()
    };

    let initial: Vec<f64> = (0..n_components)
        .map(|c| tank_inventory(&engine, c))
        .collect();
    let mut delivered = vec![0.0; n_components]; // cumulative source input per component

    let mut moved = false;
    for tick in 0..40 {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick} must converge: {e}"));
        // The feed_fill flow just applied moved this much mass in over `dt`.
        let (fill, _) = edge_stream(&engine, "feed_fill");
        for c in 0..n_components {
            delivered[c] += fill * dt * source_comp[c];
        }
        if edge_stream(&engine, "feed_line").0 > 1e-6 {
            moved = true;
        }
        for c in 0..n_components {
            let have = tank_inventory(&engine, c);
            let want = initial[c] + delivered[c];
            assert!(
                (have - want).abs() < 1e-4,
                "tick {tick}, component {c}: inventory {have} kg vs conserved {want} kg \
                 (drift {:.3e} kg). Per-component mass leaked at the column — the draw \
                 flow and composition splits disagree on the feed.",
                have - want
            );
        }
    }
    assert!(
        moved,
        "the column must actually process feed for this gate to mean anything"
    );
}

/// Determinism (I4) for a column plant. The I4 proptest generators build only
/// chains and trees, so no random case ever exercises a column; this pins the
/// demo plant instead. A column's order-sensitive parts — the BTreeMap sweep, the
/// file-order draw vector, the two-pass draw write — are all deterministic by
/// construction, and this is the byte-level proof: two fresh engines, serialized
/// each tick, must render identical f64 bits.
#[test]
fn the_demo_column_plant_reruns_bit_identically() {
    let src = include_str!("../../../scenarios/crude_column.toml");
    let run = || -> Vec<Vec<u8>> {
        let mut engine = build(src).expect("demo plant should build");
        (0..50)
            .map(|tick| {
                engine
                    .tick()
                    .unwrap_or_else(|e| panic!("tick {tick} must converge: {e}"));
                serde_json::to_vec(&engine.snapshot()).expect("snapshot must serialize")
            })
            .collect()
    };
    let first = run();
    let second = run();
    assert_eq!(first.len(), 50, "a run must capture one snapshot per tick");
    for (i, (a, b)) in first.iter().zip(&second).enumerate() {
        assert!(
            a == b,
            "column demo diverged at tick {}:\n  first:  {}\n  second: {}",
            i + 1,
            String::from_utf8_lossy(a),
            String::from_utf8_lossy(b)
        );
    }
}

// ---------------------------------------------------------------------------
// Loader validation — each refusal stops a plant that would otherwise load and
// be wrong (DESIGN §5). Most fire in `resolve_column_draws`, before the pipes
// exist, so a malformed draw list is caught regardless of the wiring.
// ---------------------------------------------------------------------------

/// A source → column → three-tank plant whose column `draws` block and any extra
/// nodes are the test's to vary. The draw pipes always go to the three tanks; a
/// draw-list-shape fault is caught before they matter.
fn loader_plant(draws_block: &str, extra_nodes: &str) -> String {
    format!(
        r#"
[meta]
name = "col_loader"

[simulation]
dt = 0.1

[fidelity]
flow = "newton"
{THREE_CUT_SLATE}

[nodes.feed]
type = "source"
pressure_bar = 5.0
temperature_c = 250.0
composition = {{ light = 0.2, middle = 0.5, heavy = 0.3 }}

[nodes.column]
type = "column"
pressure_bar = 1.5
draws = {draws_block}
{extra_nodes}
[nodes.light_tank]
type = "tank"
area_m2 = 10.0
height_m = 12.0
initial_level_m = 0.5
temperature_c = 30.0
composition = {{ light = 1.0 }}

[nodes.middle_tank]
type = "tank"
area_m2 = 10.0
height_m = 12.0
initial_level_m = 0.5
temperature_c = 30.0
composition = {{ middle = 1.0 }}

[nodes.heavy_tank]
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
name = "light_draw"
from = "column"
to = "light_tank"
length_m = 20.0
diameter_m = 0.10

[[pipes]]
name = "middle_draw"
from = "column"
to = "middle_tank"
length_m = 20.0
diameter_m = 0.10

[[pipes]]
name = "heavy_draw"
from = "column"
to = "heavy_tank"
length_m = 20.0
diameter_m = 0.10
"#
    )
}

/// A `smearing_k` written in a file reaches the column unchanged. It is a
/// temperature WIDTH, so — unlike the pressure next to it — it takes no °C→K
/// offset; this pins the ABSENCE of a conversion, the mistake the bar/°C
/// neighbours invite.
#[test]
fn smearing_reaches_the_column_unscaled() {
    let src = loader_plant(
        r#"[
    { outlet = "light_tank", up_to_c = 150.0 },
    { outlet = "middle_tank", up_to_c = 250.0 },
    { outlet = "heavy_tank" },
]"#,
        "",
    )
    .replace(
        "pressure_bar = 1.5",
        "pressure_bar = 1.5\nsmearing_k = 30.0",
    );
    let engine = build(&src).expect("plant should build");
    let id = engine.graph.find_node("column").unwrap();
    match &engine.graph.node(id).kind {
        NodeKind::Column { smearing, .. } => assert_eq!(
            smearing.value(),
            30.0,
            "30 K in the file must be 30 K on the column, unscaled"
        ),
        _ => panic!("column must be a column"),
    }
}

/// The heaviest (last) draw must OMIT `up_to_c` — it is the open catch-all. A
/// finite top on it would leave every component above that cut assigned to no
/// draw and silently dropped, a mass leak no conservation test could see.
#[test]
fn the_last_draw_must_be_open_topped() {
    let m = build_err(
        &loader_plant(
            r#"[
    { outlet = "light_tank", up_to_c = 150.0 },
    { outlet = "middle_tank", up_to_c = 250.0 },
    { outlet = "heavy_tank", up_to_c = 350.0 },
]"#,
            "",
        ),
        "a last draw with a finite up_to_c",
    );
    assert!(
        m.contains("column") && m.contains("catch-all"),
        "the error must explain the catch-all rule, got: {m}"
    );
}

/// Conversely, only the last draw may omit `up_to_c`: a middle draw without a top
/// has no upper boundary.
#[test]
fn a_non_last_draw_must_have_a_top() {
    let m = build_err(
        &loader_plant(
            r#"[
    { outlet = "light_tank" },
    { outlet = "middle_tank", up_to_c = 250.0 },
    { outlet = "heavy_tank" },
]"#,
            "",
        ),
        "a non-last draw without up_to_c",
    );
    assert!(
        m.contains("column") && m.contains("last draw"),
        "the error must say only the last draw may omit up_to_c, got: {m}"
    );
}

/// Cut points must be strictly increasing: draws are listed in ascending boiling
/// order, and a flat or inverted cut is an empty or backwards band.
#[test]
fn cut_points_must_ascend() {
    let m = build_err(
        &loader_plant(
            r#"[
    { outlet = "light_tank", up_to_c = 250.0 },
    { outlet = "middle_tank", up_to_c = 150.0 },
    { outlet = "heavy_tank" },
]"#,
            "",
        ),
        "descending cut points",
    );
    assert!(
        m.contains("column") && m.contains("ascending"),
        "the error must name the ascending-order requirement, got: {m}"
    );
}

/// A free (non-pressure-fixing) node on a draw line is refused: it puts a
/// prescribed edge back into the Jacobian, which M3.2 does not support. The
/// restriction is stated, not half-implemented (DESIGN §5).
#[test]
fn a_draw_to_a_free_node_is_refused() {
    let m = build_err(
        &loader_plant(
            r#"[
    { outlet = "light_tank", up_to_c = 150.0 },
    { outlet = "bypass", up_to_c = 250.0 },
    { outlet = "heavy_tank" },
]"#,
            "[nodes.bypass]\ntype = \"junction\"\n\n",
        ),
        "a draw to a junction",
    );
    assert!(
        m.contains("bypass") && m.contains("free"),
        "the error must name the free node on the draw line, got: {m}"
    );
}

/// A draw to a SOURCE is refused. A source pins pressure, so the free-node guard
/// alone (which checks only that) would wave it through — but a source is an
/// infinite supply, and a draw into it vanishes the product while total mass
/// still "balances" at the boundary: a plant that runs and lies. The outlet is
/// restricted to product stores explicitly, not merely to pressure-fixers.
#[test]
fn a_draw_to_a_source_is_refused() {
    let m = build_err(
        &loader_plant(
            r#"[
    { outlet = "light_tank", up_to_c = 150.0 },
    { outlet = "feed", up_to_c = 250.0 },
    { outlet = "heavy_tank" },
]"#,
            "",
        ),
        "a draw to a source",
    );
    assert!(
        m.contains("feed") && m.contains("source"),
        "the error must name the source outlet and why, got: {m}"
    );
}

/// A draw to ANOTHER COLUMN is refused. It too pins pressure, so it slips the
/// free-node guard — but the post-sweep, two-pass draw write reads an upstream
/// column's draw edge (guarded to zero in the solve) before the downstream
/// column's own write lands, so a chained column silently sees a zero feed and
/// does nothing. Chaining columns is not modelled at this fidelity; refuse it at
/// load rather than run a silently-dead second column.
#[test]
fn a_draw_to_another_column_is_refused() {
    let m = build_err(
        &loader_plant(
            r#"[
    { outlet = "light_tank", up_to_c = 150.0 },
    { outlet = "col2", up_to_c = 250.0 },
    { outlet = "heavy_tank" },
]"#,
            r#"[nodes.col2]
type = "column"
pressure_bar = 1.2
draws = [
    { outlet = "middle_tank", up_to_c = 200.0 },
    { outlet = "light_tank" },
]

"#,
        ),
        "a draw to another column",
    );
    assert!(
        m.contains("col2") && m.contains("column"),
        "the error must name the column outlet and why, got: {m}"
    );
}

/// A draw to a node that does not exist is a clear error, not a silent skip.
#[test]
fn a_draw_to_an_unknown_node_is_refused() {
    let m = build_err(
        &loader_plant(
            r#"[
    { outlet = "light_tank", up_to_c = 150.0 },
    { outlet = "ghost_tank", up_to_c = 250.0 },
    { outlet = "heavy_tank" },
]"#,
            "",
        ),
        "a draw to an unknown node",
    );
    assert!(
        m.contains("ghost_tank") && m.contains("unknown"),
        "the error must name the missing outlet, got: {m}"
    );
}

/// Two draws to one product node is refused: the engine maps a draw edge to a
/// draw by outlet, so the mapping must be one-to-one.
#[test]
fn a_repeated_outlet_is_refused() {
    let m = build_err(
        &loader_plant(
            r#"[
    { outlet = "light_tank", up_to_c = 150.0 },
    { outlet = "light_tank", up_to_c = 250.0 },
    { outlet = "heavy_tank" },
]"#,
            "",
        ),
        "a repeated outlet",
    );
    assert!(
        m.contains("light_tank") && m.contains("more than once"),
        "the error must name the doubled outlet, got: {m}"
    );
}

/// A column with fewer than two draws separates nothing.
#[test]
fn a_single_draw_column_is_refused() {
    let m = build_err(
        &loader_plant(r#"[ { outlet = "light_tank" } ]"#, ""),
        "a one-draw column",
    );
    assert!(
        m.contains("column") && m.contains("at least two"),
        "the error must require at least two draws, got: {m}"
    );
}

/// Every draw needs exactly one outlet pipe. Declaring three draws but wiring
/// only two leaves the third's `splitᵢ · ṁ_feed` with nowhere to go — caught by
/// the column degree rule, which knows a column is 1-in-N-out.
#[test]
fn a_draw_without_an_outlet_pipe_is_refused() {
    // Valid draw list, but delete the heavy draw's pipe.
    let src = loader_plant(
        r#"[
    { outlet = "light_tank", up_to_c = 150.0 },
    { outlet = "middle_tank", up_to_c = 250.0 },
    { outlet = "heavy_tank" },
]"#,
        "",
    )
    .replace(
        r#"[[pipes]]
name = "heavy_draw"
from = "column"
to = "heavy_tank"
length_m = 20.0
diameter_m = 0.10
"#,
        "",
    );
    let m = build_err(&src, "a column missing a draw pipe");
    assert!(
        m.contains("column") && (m.contains("outlet") || m.contains("1 in / 2 out")),
        "the error must flag the column's missing outlet edge, got: {m}"
    );
}
