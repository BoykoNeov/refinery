//! A column whose cold start refuses, started again from its pumparounds halved
//! and walked back (M65.1, docs/DESIGN.md §70.1).
//!
//! The reference is the column itself, reached the other way. The crude unit's
//! own column (ten stages, four oil draws and the water draw, pumparounds of 15,
//! 15 and 10 MW on stages 4, 6 and 7, 1.8 bar) fed as the shipped wet crude
//! unit on a crude 2 % water feeds it on tick 1 — 76.052 kg/s at a 658.327 K
//! liquid-equivalent temperature, read off the plant's snapshot. Cold at its
//! 40 MW, M61's start refuses (M65.0's probe: this feed starts cold only from
//! 0.45 to 0.95 of them); started at 28 MW, inside the band, and solved warm at
//! 40 MW from there, Newton lands on one column — which the retried start must
//! be.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::graph::{CascadeSpec, ColumnDraw, NodeId, Pumparound};
use refinery_core::traits::{ColumnPass, Separation, SeparationModel};
use refinery_core::units::{JPerKgK, JPerMol, Kelvin, KgPerM3, KgPerMol, KgPerSec, Pascal, Watt};
use refinery_solvers::{HeatBalanceCascade, TroutonThermo};

const FEED_KG_PER_S: f64 = 76.052;
const FEED_K: f64 = 658.327;
const WATER: f64 = 0.02;
/// The shipped pumparounds [MW] and their stages.
const PUMPAROUNDS: [(u32, f64); 3] = [(4, 15.0), (6, 15.0), (7, 10.0)];

fn slate() -> Slate {
    let cut = |name: &str, tb_c: f64, molar_mass: f64, density: f64, cp: f64| PseudoComponent {
        name: name.into(),
        tb: Kelvin(tb_c + 273.15),
        molar_mass: KgPerMol(molar_mass),
        density: Some(KgPerM3(density)),
        cp: JPerKgK(cp),
        cp_shape: None,
        phase: Phase::Liquid,
        dh_vap: None,
        cp_vapour: None,
        free_water: false,
    };
    Slate::new(vec![
        cut("light_naphtha", 70.0, 0.095, 680.0, 2200.0),
        cut("heavy_naphtha", 150.0, 0.13, 750.0, 2100.0),
        cut("kerosene", 220.0, 0.17, 800.0, 2050.0),
        cut("diesel", 300.0, 0.22, 850.0, 2000.0),
        cut("gas_oil", 380.0, 0.3, 900.0, 1950.0),
        cut("residue", 560.0, 0.55, 980.0, 1850.0),
        PseudoComponent {
            name: "water".into(),
            tb: Kelvin(373.15),
            molar_mass: KgPerMol(0.018015),
            density: Some(KgPerM3(998.0)),
            cp: JPerKgK(4184.0),
            cp_shape: None,
            phase: Phase::Liquid,
            dh_vap: Some(JPerMol(40_650.0)),
            cp_vapour: Some(JPerKgK(2000.0)),
            free_water: true,
        },
    ])
    .unwrap()
}

/// The shipped wet crude's oil, scaled to make room for 2 % water.
fn feed() -> Composition {
    let oil = [0.07976, 0.13958, 0.15952, 0.1994, 0.13958, 0.27916];
    let total: f64 = oil.iter().sum();
    let mut weights: Vec<f64> = oil.iter().map(|x| x / total * (1.0 - WATER)).collect();
    weights.push(WATER);
    Composition::from_weights(&weights).unwrap()
}

fn draws() -> Vec<ColumnDraw> {
    vec![
        ColumnDraw::by_stage(NodeId(1), 0, Some(0.22)),
        ColumnDraw::water_draw(NodeId(2)),
        ColumnDraw::by_stage(NodeId(3), 3, Some(0.16)),
        ColumnDraw::by_stage(NodeId(4), 5, Some(0.2)),
        ColumnDraw::by_stage(NodeId(5), 7, Some(0.14)),
        ColumnDraw::by_stage(NodeId(6), 10, None),
    ]
}

/// The column with its pumparounds at `share` of the shipped [-].
fn spec(share: f64) -> CascadeSpec {
    CascadeSpec {
        stages: 10,
        feed_stage: 8,
        reflux_ratio: 4.0,
        pumparounds: PUMPAROUNDS
            .iter()
            .map(|(stage, mw)| Pumparound {
                stage: *stage,
                duty: Watt(mw * share * 1e6),
            })
            .collect(),
        steam: None,
        reboiler_duty: None,
    }
}

/// One pass, handed `seed` as the last tick's answer and `earlier` as this
/// tick's earlier pass.
fn separate(
    spec: &CascadeSpec,
    seed: Option<&Separation>,
    earlier: Option<&Separation>,
) -> Result<Separation, String> {
    let slate = slate();
    let draws = draws();
    HeatBalanceCascade::new()
        .separate(
            &ColumnPass {
                slate: &slate,
                draws: &draws,
                smearing: Kelvin(0.0),
                pressure: Pascal(1.8e5),
                feed: &feed(),
                feed_flow: KgPerSec(FEED_KG_PER_S),
                temperature: Kelvin(FEED_K),
                cascade: Some(spec),
                seed: seed.and_then(|s| s.profile.as_ref()),
                earlier_pass: earlier.and_then(|s| s.profile.as_ref()),
                steam: None,
            },
            &TroutonThermo::new(),
        )
        .map_err(|e| e.to_string())
}

fn temperatures(separation: &Separation) -> Vec<f64> {
    separation
        .profile
        .as_ref()
        .expect("a profile")
        .temperatures
        .iter()
        .map(|t| t.value())
        .collect()
}

/// The worst relative difference between two separations' stage temperatures,
/// reflux, duties and draw splits.
fn worst_difference(a: &Separation, b: &Separation) -> f64 {
    let relative = |x: f64, y: f64| (x - y).abs() / y.abs().max(1e-12);
    let mut worst = 0.0f64;
    for (x, y) in temperatures(a).iter().zip(temperatures(b)) {
        worst = worst.max(relative(*x, y));
    }
    worst = worst.max(relative(a.reflux_ratio.unwrap(), b.reflux_ratio.unwrap()));
    worst = worst.max(relative(
        a.reboiler_duty.unwrap().value(),
        b.reboiler_duty.unwrap().value(),
    ));
    worst = worst.max(relative(
        a.condenser_duty.unwrap().value(),
        b.condenser_duty.unwrap().value(),
    ));
    for (x, y) in a.draws.iter().zip(&b.draws) {
        if y.split > 0.0 {
            worst = worst.max(relative(x.split, y.split));
        }
    }
    worst
}

/// **Started cold at its own pumparounds, the column is the one Newton finds
/// warm from inside its band.** The reference: the column at 28 MW, cold (no
/// retry: inside the band), then at 40 MW warm from it.
#[test]
fn a_column_started_from_halved_pumparounds_is_the_column() {
    let inside = separate(&spec(0.7), None, None).expect("inside the band, it starts cold");
    let reference = separate(&spec(1.0), Some(&inside), None).expect("warm, it runs");
    let retried = separate(&spec(1.0), None, None).expect("the retry starts it");
    let worst = worst_difference(&retried, &reference);
    assert!(worst < 1e-8, "worst relative difference {worst:e}");
    // The bottom stands where a reboiler-closed crude column does.
    let bottom = *temperatures(&retried).last().unwrap() - 273.15;
    assert!(
        (550.0..620.0).contains(&bottom),
        "the bottom at {bottom} °C"
    );
}

/// **Twice the pumparounds start from a quarter** — the first halving refuses
/// too (M65.0, finding 6) — and land on the column Newton finds warm.
#[test]
fn twice_the_pumparounds_start_from_a_quarter() {
    let inside = separate(&spec(0.7), None, None).expect("inside the band");
    let reference = separate(&spec(2.0), Some(&inside), None).expect("warm, it runs");
    let retried = separate(&spec(2.0), None, None).expect("the retry starts it");
    let worst = worst_difference(&retried, &reference);
    assert!(worst < 1e-8, "worst relative difference {worst:e}");
}

/// **Handed this tick's earlier pass, the column starts from it** rather than
/// walking again (M65.1: a later pass of the tick pays no second walk). The
/// earlier pass here is a neighbouring column's (95 % of the pumparounds), so
/// the answer is the warm solve from it to the bit — which the walk's is not.
#[test]
fn a_retried_column_starts_from_its_earlier_pass() {
    let walked = separate(&spec(1.0), None, None).expect("the retry starts it");
    let earlier = separate(&spec(0.95), Some(&walked), None).expect("a neighbouring column, warm");
    let later = separate(&spec(1.0), None, Some(&earlier)).expect("from the earlier pass");
    let warm = separate(&spec(1.0), Some(&earlier), None).expect("warm from it");
    assert_eq!(temperatures(&later), temperatures(&warm));
    assert_eq!(later.reflux_ratio, warm.reflux_ratio);
    assert_ne!(
        temperatures(&later),
        temperatures(&walked),
        "the gate cannot tell the earlier pass from a second walk"
    );
}
