//! The heat-capacity seam AS WIRED, and the published envelope its one shipped
//! shape has to sit inside (M16.2, docs/DESIGN.md §20).
//!
//! **Three different kinds of claim live here and they are not worth the same.**
//!
//! - `the_declared_shape_tracks_the_published_methane_tabulation` is the only
//!   gate in this milestone whose expected value comes from OUTSIDE the
//!   workspace. §20 fork 3 moves the citation rather than removing it — the demo
//!   file ships numbers, and by M13 gate 3's rule a shipped number with no
//!   outside envelope is a consistency check wearing a physics label. This is
//!   that envelope.
//! - The demo gates say the seam is WIRED and moves a published number. They say
//!   nothing about whether the shape is right; a wrong shape would move numbers
//!   just as happily.
//! - The refusal sweep says the milestone's own boundary is enforced rather than
//!   documented, and the defaults gate closes M10.1's escape — a byte-identity
//!   baseline has no power over the file a slice ADDS, so the new key's default
//!   is asserted on the serialized document rather than inferred from the corpus
//!   staying still.

use refinery_scenarios::{build_engine, load_str};

const DEMO: &str = include_str!("../../../scenarios/fired_gas_drum.toml");

// ---------------------------------------------------------------------------
// Gate 2 — the reference envelope.
// ---------------------------------------------------------------------------

/// NIST-JANAF Shomate coefficients for methane GAS, 298–1300 K.
///
/// Source: NIST Chemistry WebBook, SRD 69, methane (CAS 74-82-8), gas-phase
/// thermochemistry data, read from the page at
/// `https://webbook.nist.gov/cgi/cbook.cgi?ID=C74828&Units=SI&Mask=1`. The
/// coefficients are attributed there to **Chase, M.W. Jr., NIST-JANAF
/// Thermochemical Tables, 4th ed., J. Phys. Chem. Ref. Data Monograph 9 (1998)**.
///
/// **Read, not recalled**, which is the rule §18 applied when it rejected a
/// search-engine paraphrase of Watson–Nelson. And it is METHANE rather than
/// §18's n-hexane, because §19 (P4) found that §18's own anchor covers no
/// component in this corpus: all five gas plants declare `fuel_gas` at
/// `molar_mass = 0.016043`, which is methane.
///
/// ```text
/// Cp° [J/(mol·K)] = A + B·t + C·t² + D·t³ + E/t²,   t = T[K]/1000
/// ```
const SHOMATE: [f64; 5] = [-0.703_029, 108.477_3, -42.521_57, 5.862_788, 0.678_565];
const METHANE_MOLAR_MASS: f64 = 0.016_043;

/// The published capacity on a MASS basis [J/(kg·K)].
fn published_cp(temperature: f64) -> f64 {
    let t = temperature / 1000.0;
    let [a, b, c, d, e] = SHOMATE;
    (a + b * t + c * t * t + d * t * t * t + e / (t * t)) / METHANE_MOLAR_MASS
}

/// The demo's declared shape against the published tabulation, over the plant's
/// own operating span.
///
/// **What the band is sized from, and it is measured in this test rather than
/// chosen.** The declared shape is a straight line and methane's capacity is
/// curved, so the only error between them is the linear fit's own residual over
/// `[300, 800]` — computed here from the Shomate itself, so the number the bound
/// rests on is derived on the spot and not asserted. The gate then requires the
/// shipped shape to sit inside **1.4× that residual**, which a wrong anchor or a
/// wrong slope cannot do: the counterfactual below is the flat 2220 J/(kg·K) the
/// other five gas plants declare, and it misses by **43%** at 800 K.
///
/// M13 gate 3's shape exactly — a ratio inside a band sized from a named
/// one-sided error, with the thing it would fail on stated beside it.
#[test]
fn the_declared_shape_tracks_the_published_methane_tabulation() {
    let file = load_str(DEMO).expect("the demo must parse");
    let component = file.components.first().expect("the demo declares one cut");
    let anchor_k = component.cp_shape_anchor_c.expect("a declared anchor") + 273.15;
    let cp_at_anchor = component
        .cp_shape_at_anchor_j_per_kg_k
        .expect("a declared capacity");
    let slope = component
        .cp_shape_slope_j_per_kg_k2
        .expect("a declared slope");
    let declared = |t: f64| cp_at_anchor + slope * (t - anchor_k);

    // The best a straight line can do over this span — the irreducible residual
    // the bound is sized from.
    let samples: Vec<f64> = (0..=500).map(|i| 300.0 + f64::from(i)).collect();
    let n = samples.len() as f64;
    let (sx, sy): (f64, f64) = samples
        .iter()
        .fold((0.0, 0.0), |(sx, sy), &t| (sx + t, sy + published_cp(t)));
    let (mx, my) = (sx / n, sy / n);
    let (sxy, sxx): (f64, f64) = samples.iter().fold((0.0, 0.0), |(sxy, sxx), &t| {
        (
            sxy + (t - mx) * (published_cp(t) - my),
            sxx + (t - mx) * (t - mx),
        )
    });
    let best_slope = sxy / sxx;
    let best = |t: f64| my + best_slope * (t - mx);
    let irreducible = samples
        .iter()
        .map(|&t| ((best(t) - published_cp(t)) / published_cp(t)).abs())
        .fold(0.0f64, f64::max);

    let worst = samples
        .iter()
        .map(|&t| ((declared(t) - published_cp(t)) / published_cp(t)).abs())
        .fold(0.0f64, f64::max);

    assert!(
        irreducible > 0.01 && irreducible < 0.02,
        "the best straight line's own residual over 300-800 K should be ~1.7%, got {:.4}%",
        100.0 * irreducible
    );
    assert!(
        worst <= 1.4 * irreducible,
        "the declared shape must sit inside 1.4x the best line's residual: {:.4}% against \
         {:.4}%",
        100.0 * worst,
        100.0 * irreducible
    );

    // The counterfactual, so the band above is a real constraint rather than a
    // wide one: the constant this corpus's five other gas plants declare.
    let flat_miss = samples
        .iter()
        .map(|&t| ((2220.0 - published_cp(t)) / published_cp(t)).abs())
        .fold(0.0f64, f64::max);
    assert!(
        flat_miss > 0.40,
        "the constant this gate is an alternative to must miss by a lot: {:.1}%",
        100.0 * flat_miss
    );
}

/// Below the fit's own range the declared shape is an EXTRAPOLATION, and the
/// engine integrates through it: `h` starts at `energy::T_REF` = 273.15 K, which
/// is 26.85 K under the fit's low end.
///
/// The load-time monotonicity refusal is what makes that safe, and this is the
/// statement of what it guarantees on this particular file — positive and rising
/// all the way down to the datum, so `h` is strictly increasing and its inverse
/// is single valued over everything the plant can reach.
#[test]
fn the_declared_shape_stays_monotone_down_to_the_enthalpy_datum() {
    let file = load_str(DEMO).expect("the demo must parse");
    let c = file.components.first().unwrap();
    let anchor_k = c.cp_shape_anchor_c.unwrap() + 273.15;
    let cp_at_anchor = c.cp_shape_at_anchor_j_per_kg_k.unwrap();
    let slope = c.cp_shape_slope_j_per_kg_k2.unwrap();
    assert!(slope >= 0.0);
    let at_datum = cp_at_anchor + slope * (273.15 - anchor_k);
    assert!(
        at_datum > 1500.0,
        "the extrapolation to the datum must stay a plausible capacity, got {at_datum}"
    );
    // And the extrapolated distance is small enough to name: 26.85 K.
    assert!((anchor_k - 273.15 - 26.85).abs() < 1e-9);
}

// ---------------------------------------------------------------------------
// The demo, wired.
// ---------------------------------------------------------------------------

/// A helper that runs the demo for `ticks` and reports the drum's temperature,
/// mass and the plant's feed rate.
fn run(source: &str, ticks: u64) -> (f64, f64, f64, f64) {
    let file = load_str(source).expect("the plant must parse");
    let mut engine = build_engine(&file).expect("the plant must build");
    for _ in 0..ticks {
        engine.tick().expect("the plant must tick");
    }
    let s = engine.snapshot();
    let node = |name: &str| {
        s.nodes
            .iter()
            .find(|n| n.name == name)
            .unwrap_or_else(|| panic!("node '{name}'"))
    };
    let drum = node("surge_drum");
    let mass = match &drum.kind {
        refinery_core::graph::NodeKind::Vessel(v) => v.mass.value(),
        other => panic!("surge_drum is not a vessel: {other:?}"),
    };
    (
        node("heater").temperature_k,
        drum.temperature_k,
        mass,
        s.edges[0].stream.mass_flow.value(),
    )
}

/// The same plant with the shape stripped and the corpus's flat methane constant
/// in its place — the counterfactual, built from the demo's own text so the two
/// cannot drift apart.
///
/// **A text substitution rather than a shipped twin, and the pair pattern this
/// departs from is worth naming.** M7, M12, M14 and M15 each shipped two files
/// differing in ONE key, meant to be diffed. That is not available here: the
/// loader refuses a shaped component beside `heat_capacity = "constant"` and
/// refuses `cp_j_per_kg_k` beside `"linear"`, so the twins would differ in four
/// lines, not one — which is a direct consequence of fork 3 making the shape
/// declare its own anchor pair. Shipping a second corpus row whose only purpose
/// is to be a control is worse than deriving it here.
fn constant_twin() -> String {
    let mut out = String::new();
    for line in DEMO.lines() {
        let t = line.trim_start();
        if t.starts_with("cp_shape_anchor_c") || t.starts_with("cp_shape_at_anchor_j_per_kg_k") {
            continue;
        }
        if t.starts_with("cp_shape_slope_j_per_kg_k2") {
            out.push_str("cp_j_per_kg_k = 2220.0\n");
            continue;
        }
        if t.starts_with("heat_capacity = \"linear\"") {
            out.push_str("heat_capacity = \"constant\"\n");
            continue;
        }
        out.push_str(line);
        out.push('\n');
    }
    out
}

/// Ticks to the SETTLED drum. 6 000 (ten minutes) until M34; its coil
/// (docs/DESIGN.md §37) settles in about 380 s on its own and, coupled to the
/// drum, the plant's slowest mode is about 870 s, so at 6 000 ticks the drum sat
/// 92 K short of where it settles. The constant-`cp` twin is slower still, about
/// 1 165 s (0.38 K short at 80 000 ticks, 0.012 K at 120 000). 150 000 ticks is
/// thirteen of the twin's time constants, and both land on the settled values
/// the pre-M34 engine reached by tick 6 000, inside the gates' 1e-4: the coil
/// moves the road to the steady state, not the steady state.
const SETTLED_TICKS: u64 = 150_000;

/// The knob moves a number a consumer reads, which is this project's own
/// `smearing_k` bar for whether a fidelity key is real.
///
/// Measured settled (6 000 ticks before M34, `SETTLED_TICKS` since): the drum
/// settles at **802.4 K** under the shape and **1063.2 K** under the constant,
/// and holds **6.56 kg** against **5.01 kg**. (1106.9 K and 4.82 kg before M36's
/// flame, docs/DESIGN.md §40: the demo's fired duty was re-derived so the SHAPED
/// drum still settles at 802.4 K, and the twin, firing the same fuel, runs a
/// hotter coil that loses more of it up the stack. That is a negative feedback
/// the constant model's error now has to push against, so the two models part by
/// 261 K rather than 304 — still hundreds.) Before M34 the transient was wider
/// still — the constant model took the heater through 2209 K where the shape said
/// 999 K, the same duty divided by a capacity 43% too low. The coil removed that
/// overshoot: it delivers the duty at its own pace, and the shaped heater now
/// rises to its settled 792 K without passing it.
///
/// This is what §20's clause required of a demo and none of the five existing gas
/// plants could deliver: a HOLDUP whose temperature moves over a span where the
/// shape error is large.
#[test]
fn the_shaped_demo_settles_hundreds_of_kelvin_from_its_constant_twin() {
    let (heater, drum, mass, flow) = run(DEMO, SETTLED_TICKS);
    let (heater_c, drum_c, mass_c, flow_c) = run(&constant_twin(), SETTLED_TICKS);

    approx::assert_relative_eq!(drum, 802.4378, max_relative = 1e-4);
    approx::assert_relative_eq!(drum_c, 1063.1675, max_relative = 1e-4);
    assert!(
        drum_c - drum > 250.0,
        "the two models must part company by hundreds of Kelvin: {drum} vs {drum_c}"
    );
    assert!(
        heater_c > heater + 240.0,
        "and at the heater too: {heater} vs {heater_c}"
    );
    // The inventory moves with it — a vessel's mass is `P·V·M̄/(R·T)`, so a
    // temperature this wrong is a holdup this wrong.
    assert!(
        mass / mass_c > 1.30,
        "the drum's inventory must move too: {mass} vs {mass_c} kg"
    );
    // And so does the plant's own throughput, through the gas density.
    assert!(flow > flow_c * 1.10, "feed rate {flow} vs {flow_c} kg/s");
}

/// The demo reaches the state its own comments claim: a gas holdup crossing most
/// of the fit's range, and settling inside it.
#[test]
fn the_demo_actually_traverses_the_span_its_shape_is_fitted_over() {
    let (_, start, _, _) = run(DEMO, 1);
    let (_, settled, _, _) = run(DEMO, SETTLED_TICKS);
    assert!(
        start < 400.0,
        "the drum must start near ambient, got {start} K"
    );
    assert!(
        (780.0..820.0).contains(&settled),
        "and settle at the top of the fitted range, got {settled} K"
    );
    assert!(
        settled - start > 400.0,
        "the excursion is the point: {start} -> {settled} K"
    );
}

// ---------------------------------------------------------------------------
// The refusals — the milestone's own boundary, enforced rather than documented.
// ---------------------------------------------------------------------------

fn refusal(source: &str) -> String {
    match load_str(source) {
        Err(e) => e.to_string(),
        Ok(file) => match build_engine(&file) {
            Err(e) => e.to_string(),
            Ok(_) => panic!("this plant should not have loaded"),
        },
    }
}

/// Every pairing the shaped model cannot be in, refused at load with a message
/// that names the pairing.
///
/// **Both directions, as `require_compatible_fidelity` already does for
/// `separation` and `boiloff` against `thermo`.** A shape while the constant is
/// selected is a number nothing reads; the constant while a shape is selected is
/// the same fault mirrored; a shaped plant with no components is a model with no
/// data; and the last two are this slice's own boundary — a cascade column and a
/// compressible valve both take a capacity from a path the enthalpy model does
/// not reach, so a plant pairing either with a shape would run half its
/// arithmetic on each.
#[test]
fn every_incompatible_pairing_is_refused_by_name() {
    // (1) a shape nothing reads
    let e = refusal(&DEMO.replace(
        "
heat_capacity = \"linear\"
",
        "
heat_capacity = \"constant\"
",
    ));
    assert!(
        e.contains("declares a cp shape") && e.contains("nothing reads"),
        "{e}"
    );

    // (1c) a constant nothing reads
    let with_constant = DEMO.replace(
        "cp_shape_anchor_c = 26.85",
        "cp_j_per_kg_k = 2220.0\ncp_shape_anchor_c = 26.85",
    );
    let e = refusal(&with_constant);
    assert!(
        e.contains("declares cp_j_per_kg_k") && e.contains("nothing reads"),
        "{e}"
    );

    // (1b) and the mirror: the constant model with no constant
    let no_constant = constant_twin().replace("cp_j_per_kg_k = 2220.0\n", "");
    let e = refusal(&no_constant);
    assert!(e.contains("declares no cp_j_per_kg_k"), "{e}");

    // (2) fork 5 — the six plants that fall back to the water-only slate
    let no_components = DEMO
        .lines()
        .filter(|l| {
            let t = l.trim_start();
            !(t.starts_with("[[components]]")
                || t.starts_with("name = \"fuel_gas\"")
                || t.starts_with("phase = \"gas\"")
                || t.starts_with("tb_c")
                || t.starts_with("molar_mass_kg_per_mol")
                || t.starts_with("cp_shape_"))
        })
        .collect::<Vec<_>>()
        .join("\n");
    let e = refusal(&no_components);
    assert!(e.contains("no [[components]] block"), "{e}");

    // (3) a partly-shaped slate — one of the three keys gone, which is
    //     `build.rs`'s all-three-or-none check rather than the pairing rule.
    let partial = DEMO.replace("cp_shape_slope_j_per_kg_k2 = 3.5214071112\n", "");
    let e = refusal(&partial);
    assert!(e.contains("only part of a cp shape"), "{e}");

    // (3b) and NO shape at all, which is the pairing rule and is a DIFFERENT
    //      refusal from (3). The sweep's first draft had only the partial case,
    //      so removing the pairing rule left this sweep green (§20's mutation 6).
    let unshaped = DEMO
        .lines()
        .filter(|l| !l.trim_start().starts_with("cp_shape_"))
        .collect::<Vec<_>>()
        .join("\n");
    let e = refusal(&unshaped);
    assert!(
        e.contains("declares no cp shape") && e.contains("mass-weighted sum"),
        "{e}"
    );

    // (4) the cascade's duties
    let cascade = DEMO.replace(
        "
heat_capacity = \"linear\"
",
        "
separation = \"cascade\"
thermo = \"trouton\"
heat_capacity = \"linear\"
",
    );
    let cascade = cascade.replace("thermo = \"constant\"\n", "");
    let e = refusal(&cascade);
    // A distinctive substring of THIS refusal's own message, not merely the word
    // "cascade" — which any of the cascade loader's own refusals would also
    // carry. The first draft asserted `separation = \\"cascade\\"` (with the
    // backslashes, which the error does not contain) OR the bare word, so the
    // bare word was doing all the work: M14.1's "refused one pass earlier for
    // the wrong reason with the right exit code", one level down.
    assert!(e.contains("with separation = \"cascade\""), "{e}");

    // (5) a compressible valve's gamma
    let with_valve = DEMO.replace(
        "[nodes.fuel_sink]",
        "[nodes.trim_valve]\ntype = \"valve\"\nkv = 20.0\nopening = 0.6\nx_t = 0.72\n\n[nodes.fuel_sink]",
    );
    let with_valve = with_valve.replace(
        "name = \"drum_outlet\"\nfrom = \"surge_drum\"\nto = \"fuel_sink\"",
        "name = \"drum_outlet\"\nfrom = \"surge_drum\"\nto = \"trim_valve\"",
    ) + "\n[[pipes]]\nname = \"valve_outlet\"\nfrom = \"trim_valve\"\nto = \"fuel_sink\"\nlength_m = 5.0\ndiameter_m = 0.04\n";
    let e = refusal(&with_valve);
    assert!(
        e.contains("declaring x_t") && e.contains("trim_valve"),
        "{e}"
    );

    // And an unknown model name, the shape every other fidelity key's refusal has.
    let e = refusal(&DEMO.replace(
        "
heat_capacity = \"linear\"
",
        "
heat_capacity = \"quadratic\"
",
    ));
    assert!(e.contains("unknown heat capacity model"), "{e}");
}

/// The new key's DEFAULT, asserted on the parsed document rather than inferred
/// from the corpus staying still.
///
/// **M10.1's escape, closed in advance.** A byte-identity baseline has no power
/// over the file a slice adds — that is how M10.1's serde-tag mutation passed the
/// whole corpus on both fidelities. Here the analogous escape is the default:
/// give `heat_capacity` a default of `"linear"` and every one of the nineteen
/// shipped plants would fail to LOAD, which the corpus does catch. Give the demo
/// the wrong value and nothing outside this file notices. So both ends are pinned
/// here: the default is `"constant"`, and the demo is the only file in
/// `scenarios/` that says otherwise.
#[test]
fn the_key_defaults_to_constant_and_exactly_one_shipped_plant_says_otherwise() {
    // The M1 reference plant, which predates this key by fifteen milestones and
    // says nothing about it — the real statement of what the default MEANS, as
    // opposed to what a hand-written stub would say.
    const OLDEST: &str = include_str!("../../../scenarios/tank_pump_valve.toml");
    assert!(!OLDEST.contains("heat_capacity"));
    let oldest = load_str(OLDEST).expect("the M1 reference plant must parse");
    assert_eq!(oldest.fidelity.heat_capacity, "constant");

    let demo = load_str(DEMO).expect("the demo must parse");
    assert_eq!(demo.fidelity.heat_capacity, "linear");

    let mut shaped = Vec::new();
    for entry in std::fs::read_dir(concat!(env!("CARGO_MANIFEST_DIR"), "/../../scenarios"))
        .expect("the scenarios directory")
    {
        let path = entry.expect("a directory entry").path();
        if path.extension().and_then(|e| e.to_str()) != Some("toml") {
            continue;
        }
        let src = std::fs::read_to_string(&path).expect("a readable scenario");
        let file = load_str(&src).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        if file.fidelity.heat_capacity != "constant" {
            shaped.push(file.meta.name);
        }
    }
    assert_eq!(
        shaped,
        vec!["fired_gas_drum".to_string()],
        "exactly one shipped plant may select a shaped capacity"
    );
}
