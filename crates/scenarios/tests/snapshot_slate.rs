//! M8.5: the slate on the snapshot, and the tank level a frontend computes
//! from it — M6.2's deferral, closed.
//!
//! **What was missing.** A tank reports mass [kg], area [m²] and height [m].
//! Turning that into a fill fraction is `h = m/(ρ·A)`, and the snapshot carried
//! no density: compositions crossed as bare `mass_fractions`, with neither the
//! per-component densities they index nor even the component *names*. The M6.2
//! scene therefore drew mass on a shared scale and said so in a comment, rather
//! than hardcoding water's 998 kg/m³ and drawing a confident level that is wrong
//! for every other plant. `Snapshot::slate` is the deferral's own stated fix.
//!
//! **These gates read JSON, not the `Snapshot` struct, and that is the point.**
//! The consumer is `demo/plant.gd` through `bridge::snapshot_json`, so the
//! contract that can break is the set of KEY NAMES — `slate`,
//! `density_kg_per_m3`, `composition.mass_fractions`, `area`, `height`, `mass`.
//! A test written against the typed struct would keep passing through a rename
//! that silently emptied the scene's arithmetic. Nothing else in the workspace
//! pins those keys.
//!
//! # The independent side does not exist, and it is worth knowing why
//!
//! The obvious second path to a level is the tank's own reported pressure: a
//! tank pins `P = P_ATM + ρ·g·h` (`network::fixed_pressure` → `bottom_pressure`),
//! so `(P − P_ATM)/(ρ·g)` looks like a level derived by the *solver* to check the
//! *snapshot's* against. It is not. Substitute `h = m/(ρ·A)` and the density
//! cancels exactly:
//!
//! ```text
//! P − P_ATM = ρ·g·h = ρ·g·m/(ρ·A) = m·g/A
//! ```
//!
//! A tank's hydrostatic pressure carries **no density information at all** — it
//! is mass over area, and a snapshot shipping `cp` in the density slot would
//! move both sides by the same factor and agree. That is asserted below rather
//! than merely argued (`a_tanks_pressure_carries_no_density_to_gate_one`),
//! because the next person to want an independent side will reach for it first.
//!
//! The same cancellation kills every other candidate: mass balance, holdup,
//! transport — density is observable only through a *volume*, and the only
//! volume any scenario declares is `initial_level_m`. So the load-time level is
//! the one anchor outside the code, it exists only at tick 0, and
//! [`a_frontend_reconstructs_the_declared_level_from_the_snapshot_alone`] is
//! this file's real gate. What it pins is the WIRING — right field, right order,
//! nothing dropped. The reciprocal mixing rule itself is `components.rs`'s, and
//! is unit-tested there; a gate here could only mirror it.

use refinery_core::units::{G, P_ATM};
use refinery_scenarios::{build_engine, load_str};
use serde_json::Value;

const CRUDE_COLUMN: &str = include_str!("../../../scenarios/crude_column.toml");
const LEAKING_LINE: &str = include_str!("../../../scenarios/leaking_line.toml");

/// Every scenario this repo ships, so the invariants below are claims about the
/// whole fleet rather than about whichever file was convenient.
const SHIPPED: &[(&str, &str)] = &[
    (
        "cooler_chiller",
        include_str!("../../../scenarios/cooler_chiller.toml"),
    ),
    (
        "crude_column",
        include_str!("../../../scenarios/crude_column.toml"),
    ),
    (
        "crude_column_cascade",
        include_str!("../../../scenarios/crude_column_cascade.toml"),
    ),
    (
        "fcc_plant",
        include_str!("../../../scenarios/fcc_plant.toml"),
    ),
    (
        "fcc_reactor",
        include_str!("../../../scenarios/fcc_reactor.toml"),
    ),
    (
        "furnace_heater",
        include_str!("../../../scenarios/furnace_heater.toml"),
    ),
    ("gas_line", include_str!("../../../scenarios/gas_line.toml")),
    (
        "gas_valve",
        include_str!("../../../scenarios/gas_valve.toml"),
    ),
    (
        "heat_recovery",
        include_str!("../../../scenarios/heat_recovery.toml"),
    ),
    (
        "knockout_drum",
        include_str!("../../../scenarios/knockout_drum.toml"),
    ),
    (
        "leaking_line",
        include_str!("../../../scenarios/leaking_line.toml"),
    ),
    (
        "relief_blowdown",
        include_str!("../../../scenarios/relief_blowdown.toml"),
    ),
    (
        "tank_level_control",
        include_str!("../../../scenarios/tank_level_control.toml"),
    ),
    (
        "tank_pump_valve",
        include_str!("../../../scenarios/tank_pump_valve.toml"),
    ),
];

/// The snapshot as a frontend sees it: JSON, parsed generically.
///
/// Generic rather than `serde_json::from_str::<Snapshot>` because a pre-tick
/// snapshot does not deserialize back — `pressure_pa` is NaN and serializes as
/// `null` (`bridge.rs` module docs) — and tick 0 is where this file's anchor
/// lives.
fn snapshot_json(src: &str, ticks: u64) -> Value {
    let file = load_str(src).expect("scenario loads");
    let mut engine = build_engine(&file).expect("engine builds");
    for t in 1..=ticks {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
    }
    serde_json::to_value(engine.snapshot()).expect("snapshot serializes")
}

/// Every tank node in a snapshot, as `(name, kind object)`.
fn tanks(snapshot: &Value) -> Vec<(String, &Value)> {
    snapshot["nodes"]
        .as_array()
        .expect("nodes is an array")
        .iter()
        .filter(|n| n["kind"]["type"] == "tank")
        .map(|n| {
            (
                n["name"].as_str().expect("a node name").to_string(),
                &n["kind"],
            )
        })
        .collect()
}

fn tank(snapshot: &Value, name: &str) -> Value {
    tanks(snapshot)
        .into_iter()
        .find(|(n, _)| n == name)
        .unwrap_or_else(|| panic!("no tank named '{name}'"))
        .1
        .clone()
}

/// **The frontend's own arithmetic**, in Rust: the exact computation
/// `demo/plant.gd` performs, over exactly the JSON keys it reads.
///
/// `densities` is passed in rather than read from `snapshot["slate"]` so the
/// same function can be handed a DELIBERATELY WRONG order — see
/// [`the_slate_travels_in_declaration_order_not_sorted_by_name`]. The honest
/// path is `slate_densities(snapshot)`.
fn level_from_snapshot(tank_kind: &Value, densities: &[f64]) -> f64 {
    let fractions: Vec<f64> = tank_kind["composition"]["mass_fractions"]
        .as_array()
        .expect("a composition carries mass_fractions")
        .iter()
        .map(|f| f.as_f64().expect("a mass fraction is a number"))
        .collect();
    assert_eq!(
        fractions.len(),
        densities.len(),
        "a frontend zips fractions against the slate; the two lengths must match"
    );
    // Mixture liquid density, 1/ρ mass-weighted — ideal liquid blending, the
    // rule `Composition::mixture_density` implements. Zero fractions are skipped
    // so a slate carrying gas cuts (null density) does not poison a liquid tank.
    let inverse: f64 = fractions
        .iter()
        .zip(densities)
        .filter(|(f, _)| **f > 0.0)
        .map(|(f, rho)| f / rho)
        .sum();
    let density = 1.0 / inverse;
    tank_kind["mass"].as_f64().expect("a tank mass")
        / (density * tank_kind["area"].as_f64().expect("a tank area"))
}

/// The slate's densities in the order the snapshot ships them.
///
/// Panics on a `null` — which on a tank's slate is unreachable and is the
/// invariant [`no_tank_anywhere_holds_a_component_without_a_density`] proves,
/// stated here as the assertion `demo/plant.gd` is entitled to omit.
fn slate_densities(snapshot: &Value) -> Vec<f64> {
    snapshot["slate"]
        .as_array()
        .expect("a snapshot carries a slate")
        .iter()
        .map(|c| {
            c["density_kg_per_m3"]
                .as_f64()
                .unwrap_or_else(|| panic!("component '{}' has no density", c["name"]))
        })
        .collect()
}

// ---------------------------------------------------------------------------
// The anchor: a declared level, read back out
// ---------------------------------------------------------------------------

/// **This file's real gate.** At tick 0 a tank's mass is `ρ·A·h` for the `h` its
/// scenario file declares (`scenarios/src/lib.rs`, the level → mass conversion),
/// so reconstructing `m/(ρ·A)` from the snapshot must return that declared
/// number — and the declared number is written in a TOML file, outside every
/// code path involved.
///
/// Run on the five-cut crude plant *and* on `leaking_line`, the plant the M6.2
/// scene actually draws. The crude plant is what gives the gate teeth: its three
/// product tanks are pure cuts at slate indices 0, 2 and 4 with densities 680,
/// 800 and 950 kg/m³, so a slate that arrived misordered or carrying the wrong
/// field pairs at least two of them with a density 18–40% wrong. `leaking_line`
/// is water-only and cannot discriminate anything — it is here because a gate on
/// the demo plant is what says the scene's own arithmetic is right.
#[test]
fn a_frontend_reconstructs_the_declared_level_from_the_snapshot_alone() {
    // (tank, the `initial_level_m` its file declares, and the fill fraction
    // that level is in that file's shell — stated as its own number rather than
    // as `declared/height`, which the assertion above already implies and which
    // would make this half a tautology.)
    type Tank = (&'static str, f64, f64);
    let cases: &[(&str, &[Tank])] = &[
        (
            CRUDE_COLUMN,
            &[
                // 0.5 m in a 12 m shell.
                ("naphtha_tank", 0.5, 0.5 / 12.0),
                ("distillate_tank", 0.5, 0.5 / 12.0),
                ("bottoms_tank", 0.5, 0.5 / 12.0),
            ],
        ),
        (
            LEAKING_LINE,
            // 8 m and 1 m in 10 m shells — the two bars the M6.2 scene draws.
            &[("supply_tank", 8.0, 0.80), ("receiving_tank", 1.0, 0.10)],
        ),
    ];
    for (src, wanted) in cases {
        let snapshot = snapshot_json(src, 0);
        let densities = slate_densities(&snapshot);
        for (name, declared, fraction) in *wanted {
            let kind = tank(&snapshot, name);
            let level = level_from_snapshot(&kind, &densities);
            // Tight, and derived rather than chosen: the round trip is
            // `(ρ·A·h)/(ρ·A)` in f64, which is exact but for two roundings.
            approx::assert_relative_eq!(level, *declared, max_relative = 1e-12);
            // And the fill FRACTION, which is the quantity the scene draws.
            approx::assert_relative_eq!(
                level / kind["height"].as_f64().expect("a tank height"),
                *fraction,
                max_relative = 1e-12
            );
        }
    }
}

/// The discriminator: the slate must travel in DECLARATION order, because that
/// is the order `mass_fractions` indexes into (`components::Slate` — "order is
/// canonical"). Any other order pairs every tank with someone else's density.
///
/// Two wrong orders are run, and the margins are reported rather than assumed,
/// because neither moves every tank:
///
/// | tank | cut | ρ | reversed | sorted by name |
/// |---|---|---|---|---|
/// | naphtha_tank | light_naphtha (0) | 680 | 950, **−28%** | 850 (diesel), **−20%** |
/// | distillate_tank | kerosene (2) | 800 | 800, *unmoved* | 800, *unmoved* |
/// | bottoms_tank | residue (4) | 950 | 680, **+40%** | 950, *unmoved* |
///
/// Kerosene sits at the centre of a five-cut slate, so a reversal cannot move
/// it, and it is alphabetically third of five, so a sort cannot either. A gate
/// asserting "all three tanks move" would be false; a gate run only on the
/// distillate tank would be vacuous under both mutations. The assertion is
/// therefore per-tank and states which case is which.
#[test]
fn the_slate_travels_in_declaration_order_not_sorted_by_name() {
    let snapshot = snapshot_json(CRUDE_COLUMN, 0);
    let shipped = slate_densities(&snapshot);
    assert_eq!(
        shipped,
        vec![680.0, 750.0, 800.0, 850.0, 950.0],
        "the slate arrives in the order crude_column.toml declares it"
    );

    let names: Vec<&str> = snapshot["slate"]
        .as_array()
        .unwrap()
        .iter()
        .map(|c| c["name"].as_str().unwrap())
        .collect();
    assert_eq!(
        names,
        vec![
            "light_naphtha",
            "heavy_naphtha",
            "kerosene",
            "diesel",
            "residue"
        ],
        "names travel with their densities, in the same declaration order"
    );

    let mut reversed = shipped.clone();
    reversed.reverse();
    // Sorted by name: diesel 850, heavy_naphtha 750, kerosene 800,
    // light_naphtha 680, residue 950.
    let sorted = vec![850.0, 750.0, 800.0, 680.0, 950.0];

    // (tank, declared level, moved by a reversal?, moved by a name sort?)
    for (name, declared, reversal_moves, sort_moves) in [
        ("naphtha_tank", 0.5, true, true),
        ("distillate_tank", 0.5, false, false),
        ("bottoms_tank", 0.5, true, false),
    ] {
        let kind = tank(&snapshot, name);
        approx::assert_relative_eq!(
            level_from_snapshot(&kind, &shipped),
            declared,
            max_relative = 1e-12
        );
        for (wrong, moves, label) in [
            (&reversed, reversal_moves, "reversed"),
            (&sorted, sort_moves, "name-sorted"),
        ] {
            let under_wrong = level_from_snapshot(&kind, wrong);
            let error = (under_wrong - declared).abs() / declared;
            if moves {
                assert!(
                    error > 0.15,
                    "a {label} slate must move {name}'s level by a visible margin, \
                     got {:.1}% ({under_wrong:.4} m against a declared {declared} m)",
                    error * 100.0
                );
            } else {
                assert!(
                    error < 1e-12,
                    "{name}'s cut is invariant under a {label} slate — see this \
                     test's table; got {:.1}%",
                    error * 100.0
                );
            }
        }
    }
}

/// A tank that has become a genuine MIXTURE still reconstructs — the case a
/// single-component plant cannot reach, and the one the wired demo does not
/// cover (`leaking_line.toml` is water only, where every mixing rule agrees).
///
/// **This half is a mirror and is labelled as one.** After tick 0 there is no
/// anchor outside the code left to check against (see the module docs: density
/// cancels out of every other observable), so the comparison is against
/// `TankState::level`, the engine's own. Its power is over the wiring — a
/// dropped component, a misordered slate, `cp` in the density slot — not over
/// the mixing rule, which `components.rs` unit-tests.
///
/// What it adds beyond the tick-0 gate is that the mixture is REAL: the naphtha
/// tank starts as pure light naphtha and fills from a draw running roughly half
/// heavy naphtha (`crude_column.toml`'s header: 48.8 / 51.2 at the 155 °C cut),
/// so both assertions below would fail on a tank that stayed pure.
#[test]
fn a_tank_that_has_become_a_mixture_reconstructs_the_engines_own_level() {
    const TICKS: u64 = 600;
    let file = load_str(CRUDE_COLUMN).expect("scenario loads");
    let mut engine = build_engine(&file).expect("engine builds");
    for t in 1..=TICKS {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
    }
    let snapshot = serde_json::to_value(engine.snapshot()).unwrap();
    let densities = slate_densities(&snapshot);
    let kind = tank(&snapshot, "naphtha_tank");

    // Not vacuous: two cuts present, in real quantity.
    let fractions: Vec<f64> = kind["composition"]["mass_fractions"]
        .as_array()
        .unwrap()
        .iter()
        .map(|f| f.as_f64().unwrap())
        .collect();
    let present = fractions.iter().filter(|f| **f > 0.01).count();
    assert!(
        present >= 2,
        "after {TICKS} ticks the naphtha tank must hold a mixture, not a pure \
         cut — this gate is about mixing. Fractions: {fractions:?}"
    );

    // And the mixing rule is actually doing something: the mixture density sits
    // strictly between its two pure cuts, away from both. Without this a slate
    // whose densities were all equal would pass everything above.
    let inverse: f64 = fractions
        .iter()
        .zip(&densities)
        .filter(|(f, _)| **f > 0.0)
        .map(|(f, rho)| f / rho)
        .sum();
    let mixture = 1.0 / inverse;
    assert!(
        mixture > 680.0 * 1.005 && mixture < 750.0 * 0.995,
        "the mixture density must lie strictly between light (680) and heavy \
         (750) naphtha and differ from both, got {mixture:.2} kg/m³"
    );

    let engine_level = match &engine
        .graph
        .node(
            engine
                .graph
                .node_ids()
                .find(|id| engine.graph.node(*id).name == "naphtha_tank")
                .expect("the tank exists"),
        )
        .kind
    {
        refinery_core::graph::NodeKind::Tank(t) => t.level(&engine.slate).value(),
        _ => unreachable!("naphtha_tank is a tank"),
    };
    approx::assert_relative_eq!(
        level_from_snapshot(&kind, &densities),
        engine_level,
        max_relative = 1e-12
    );
    // The tank filled, so this is not the tick-0 gate repeated.
    assert!(
        engine_level > 0.6,
        "the naphtha tank must have filled past its declared 0.5 m, got {engine_level:.3} m"
    );
}

// ---------------------------------------------------------------------------
// Fleet-wide invariants
// ---------------------------------------------------------------------------

/// The `Option` on `density_kg_per_m3` is never `None` on the fill-level path,
/// across every scenario this repo ships — the invariant that lets
/// `demo/plant.gd` divide by it with no fallback branch.
///
/// It is not a coincidence and not a property of these particular files: the
/// loader refuses a tank whose composition is gas-phase, and the only other
/// holdup kind, `NodeKind::Vessel`, has a pressure for a state rather than a
/// level. This gate is what would notice if that guard were ever relaxed.
#[test]
fn no_tank_anywhere_holds_a_component_without_a_density() {
    let mut tanks_seen = 0;
    for (name, src) in SHIPPED {
        let snapshot = snapshot_json(src, 0);
        let slate = snapshot["slate"].as_array().expect("a slate").clone();
        for (tank_name, kind) in tanks(&snapshot) {
            tanks_seen += 1;
            let fractions = kind["composition"]["mass_fractions"].as_array().unwrap();
            for (i, f) in fractions.iter().enumerate() {
                if f.as_f64().unwrap() <= 0.0 {
                    continue;
                }
                assert!(
                    slate[i]["density_kg_per_m3"].is_f64(),
                    "{name}: tank '{tank_name}' holds component '{}' at fraction {f}, \
                     which reports no density — a frontend cannot size its level",
                    slate[i]["name"]
                );
            }
        }
    }
    // A count, so a loop that stopped visiting tanks cannot pass by finding
    // nothing — the vacuous-counter shape this repo has shipped twice. Measured
    // at 15 across the fourteen shipped files; `>=` so adding a plant is not a
    // test failure, and adding one with no tank is caught by nothing here
    // because there is nothing to catch.
    assert!(
        tanks_seen >= 15,
        "the shipped fleet carries 15 tanks; this sweep visited {tanks_seen}"
    );
}

/// ...and the `Option` is not decorative either: a gas cut reports `null`.
///
/// The two halves belong together. Without this one, a snapshot that shipped
/// `0.0` — or the liquid rule's answer — for a gas component would pass the gate
/// above, and a frontend dividing by it would draw a level for a fluid that has
/// none.
#[test]
fn a_gas_component_reports_no_density() {
    let snapshot = snapshot_json(include_str!("../../../scenarios/gas_line.toml"), 0);
    let slate = snapshot["slate"].as_array().unwrap();
    assert!(
        slate
            .iter()
            .any(|c| c["density_kg_per_m3"].is_null() && c["name"] == "fuel_gas"),
        "gas_line's single gas cut must report a null density, got {slate:?}"
    );
    // No tank in that plant, so the two invariants do not contradict — stated
    // here because it is the reason both can hold at once.
    assert!(
        tanks(&snapshot).is_empty(),
        "gas_line has no tanks; if it gained one the loader would refuse it"
    );
}

/// Every node's composition indexes the same slate. A frontend zips the two by
/// position, so a length mismatch anywhere is a silent misread rather than an
/// error.
#[test]
fn every_composition_is_the_length_of_the_slate() {
    for (name, src) in SHIPPED {
        let snapshot = snapshot_json(src, 0);
        let width = snapshot["slate"].as_array().unwrap().len();
        assert!(width >= 1, "{name}: a slate is never empty");
        for node in snapshot["nodes"].as_array().unwrap() {
            if let Some(fractions) = node["kind"]["composition"]["mass_fractions"].as_array() {
                assert_eq!(
                    fractions.len(),
                    width,
                    "{name}: node '{}' carries {} fractions against a slate of {width}",
                    node["name"],
                    fractions.len()
                );
            }
        }
        for edge in snapshot["edges"].as_array().unwrap() {
            let fractions = edge["stream"]["composition"]["mass_fractions"]
                .as_array()
                .unwrap_or_else(|| panic!("{name}: edge '{}' has no composition", edge["name"]));
            assert_eq!(fractions.len(), width, "{name}: edge '{}'", edge["name"]);
        }
    }
}

/// The claim from this file's module docs, asserted rather than argued: a tank's
/// reported pressure is `m·g/A` above atmospheric, with the density cancelled
/// out — so it cannot be used as an independent check on the slate's density.
///
/// Kept because it is the first thing someone will reach for when strengthening
/// this file. It is a real gate on the tank's pressure datum (that the reported
/// pressure is the BOTTOM nozzle's, and that its datum is `P_ATM` rather than
/// gauge) — it is simply powerless against the density, which is what the name
/// says.
///
/// **The identity holds against the PREVIOUS tick's mass, and finding that out
/// is what this test cost.** A snapshot's tank pressure comes from the
/// hydraulic solve, which runs before transport and inventory integration
/// (DESIGN §1: solve → transport → unit dynamics), while its `mass` is what the
/// integration left. So the two reported numbers are one Euler step apart —
/// 0.67 Pa on `leaking_line`'s supply tank, 8.6e-6 relative, small enough that a
/// tolerance picked to "look tight" would have hidden it. Comparing against
/// tick 0's mass makes it exact instead, and states the offset rather than
/// absorbing it.
#[test]
fn a_tanks_pressure_carries_no_density_to_gate_one() {
    let file = load_str(LEAKING_LINE).expect("scenario loads");
    let mut engine = build_engine(&file).expect("engine builds");
    // Pre-tick: masses are real (stored), pressures are NaN (solved).
    let before = serde_json::to_value(engine.snapshot()).unwrap();
    engine.tick().expect("one tick");
    let after = serde_json::to_value(engine.snapshot()).unwrap();

    let mut checked = 0;
    for (name, kind_before) in tanks(&before) {
        let node = after["nodes"]
            .as_array()
            .unwrap()
            .iter()
            .find(|n| n["name"] == name.as_str())
            .unwrap()
            .clone();
        let gauge = node["pressure_pa"].as_f64().unwrap() - P_ATM.value();
        // ρ has cancelled: this is mass over area, with no density in it.
        let mass_over_area =
            kind_before["mass"].as_f64().unwrap() * G / kind_before["area"].as_f64().unwrap();
        approx::assert_relative_eq!(gauge, mass_over_area, max_relative = 1e-12);

        // And the same comparison against the FRESH mass does not hold — which
        // is what makes the sentence above a measurement rather than a story.
        let fresh = tank(&after, &name)["mass"].as_f64().unwrap() * G
            / kind_before["area"].as_f64().unwrap();
        assert!(
            (gauge - fresh).abs() > 1e-3,
            "{name}: the one-step offset must be visible, got {:.3e} Pa",
            (gauge - fresh).abs()
        );
        checked += 1;
    }
    assert_eq!(checked, 2, "leaking_line has two tanks");
}
