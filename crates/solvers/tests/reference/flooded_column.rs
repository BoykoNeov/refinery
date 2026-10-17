//! M63.4 — a column flooded with water (docs/DESIGN.md §68.4, ledger row C13):
//! a feed at least `FLOODED_WATER_SHARE` water that the heat-balance cascade
//! cannot separate passes straight through for the tick — its water out by the
//! water draw at water's bubble point, its oil to the bottoms whole at the
//! feed's temperature — and the refusal rides the answer. M62.5's eight-stage
//! column (`reference/wet_column.rs`) separates to 16 % water and refuses from
//! 18 %; the books here are the column's own: what flows in, plus the reboiler,
//! is what flows out, plus the condenser.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::energy::T_REF;
use refinery_core::graph::{CascadeSpec, ColumnDraw, NodeId, Pumparound};
use refinery_core::traits::{ColumnPass, ColumnSteam, Separation, SeparationModel, ThermoModel};
use refinery_core::units::{JPerKgK, JPerMol, Kelvin, KgPerM3, KgPerMol, KgPerSec, Pascal};
use refinery_solvers::heat_balance::FLOODED_WATER_SHARE;
use refinery_solvers::{bubble_temperature, HeatBalanceCascade, MoleFractions, TroutonThermo};

const WATER: usize = 4;
const FEED_KG_PER_S: f64 = 10.0;
const PRESSURE: f64 = 1.8e5;
const FEED_C: f64 = 230.0;
const OIL: [f64; 4] = [0.2, 0.3, 0.3, 0.2];

fn slate() -> Slate {
    let cut = |name: &str, tb_c: f64, molar_mass: f64, cp: f64| PseudoComponent {
        name: name.into(),
        tb: Kelvin(tb_c + 273.15),
        molar_mass: KgPerMol(molar_mass),
        density: Some(KgPerM3(800.0)),
        cp: JPerKgK(cp),
        cp_shape: None,
        phase: Phase::Liquid,
        dh_vap: None,
        cp_vapour: None,
        free_water: false,
    };
    Slate::new(vec![
        cut("light", 70.0, 0.095, 2200.0),
        cut("naphtha", 150.0, 0.13, 2100.0),
        cut("kerosene", 220.0, 0.17, 2050.0),
        cut("diesel", 300.0, 0.22, 2000.0),
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

/// The feed: `OIL` diluted to `1 − water` by mass.
fn feed(water: f64) -> Composition {
    let mut weights: Vec<f64> = OIL.iter().map(|x| x * (1.0 - water)).collect();
    weights.push(water);
    Composition::from_weights(&weights).unwrap()
}

/// Distillate, the water draw, a side draw on stage 3, the bottoms.
fn draws() -> Vec<ColumnDraw> {
    vec![
        ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
        ColumnDraw::water_draw(NodeId(2)),
        ColumnDraw::by_stage(NodeId(3), 3, Some(0.25)),
        ColumnDraw::by_stage(NodeId(4), 8, None),
    ]
}

fn spec(pumparounds: Vec<Pumparound>) -> CascadeSpec {
    CascadeSpec {
        stages: 8,
        feed_stage: 5,
        reflux_ratio: 2.0,
        pumparounds,
        steam: None,
    }
}

fn separate(
    slate: &Slate,
    feed: &Composition,
    spec: &CascadeSpec,
    steam: Option<ColumnSteam>,
) -> Result<Separation, String> {
    HeatBalanceCascade::new()
        .separate(
            &ColumnPass {
                slate,
                draws: &draws(),
                smearing: Kelvin(0.0),
                pressure: Pascal(PRESSURE),
                feed,
                feed_flow: KgPerSec(FEED_KG_PER_S),
                temperature: Kelvin(FEED_C + 273.15),
                cascade: Some(spec),
                seed: None,
                steam,
            },
            &TroutonThermo::new(),
        )
        .map_err(|e| e.to_string())
}

/// A liquid stream's enthalpy flux on the engine's datum [W]: `ṁ·c_p·(T − T_REF)`.
fn flux(slate: &Slate, composition: &Composition, mass_flow: f64, temperature: Kelvin) -> f64 {
    mass_flow * composition.mixture_cp(slate).value() * (temperature.value() - T_REF.value())
}

/// **Too wet to separate, the feed passes through** — at 30 % water, past the
/// 18 % where this column's solve refuses: the water draw takes every drop of
/// water, a liquid at water's bubble point at the column's pressure; the
/// bottoms takes the oil whole at the feed's temperature; the distillate and
/// the side draw nothing. The refusal rides the answer, and no profile does.
/// The column's own books close: the feed's flux plus the reboiler's duty is
/// what the draws carry out plus the condenser's.
#[test]
fn a_feed_too_wet_to_separate_passes_through() {
    let slate = slate();
    let wet = feed(0.30);
    let separation = separate(&slate, &wet, &spec(Vec::new()), None).unwrap();
    let reason = separation
        .water_flooded
        .as_deref()
        .expect("the column flooded");
    assert!(reason.contains("heat-balance cascade"), "{reason}");
    assert!(separation.profile.is_none());
    assert_eq!(separation.feed_vapour_fraction, None);

    let [distillate, water, side, bottoms] = &separation.draws[..] else {
        panic!("four draws, the water draw second");
    };
    assert_eq!((distillate.split, side.split), (0.0, 0.0));
    assert!(
        (water.split - 0.30).abs() < 1e-12,
        "water draw {}",
        water.split
    );
    assert!(
        (bottoms.split - 0.70).abs() < 1e-12,
        "bottoms {}",
        bottoms.split
    );
    assert_eq!(water.composition.fractions()[WATER], 1.0);
    assert_eq!(bottoms.composition.fractions()[WATER], 0.0);
    for (got, want) in bottoms.composition.fractions()[..4].iter().zip(OIL) {
        assert!(
            (got - want).abs() < 1e-12,
            "the oil passes whole: {got} against {want}"
        );
    }
    let boiling = bubble_temperature(
        &slate,
        &TroutonThermo::new(),
        &MoleFractions::from_mass(&water.composition, &slate).unwrap(),
        Pascal(PRESSURE),
        "a test",
    )
    .unwrap();
    assert_eq!(water.temperature, boiling);
    assert_eq!(bottoms.temperature, Kelvin(FEED_C + 273.15));

    let into = flux(&slate, &wet, FEED_KG_PER_S, Kelvin(FEED_C + 273.15))
        + separation.reboiler_duty.unwrap().value();
    let out: f64 = separation
        .draws
        .iter()
        .map(|d| {
            flux(
                &slate,
                &d.composition,
                d.split * FEED_KG_PER_S,
                d.temperature,
            )
        })
        .sum::<f64>()
        + separation.condenser_duty.unwrap().value();
    assert!(
        (into - out).abs() < 1e-9 * into,
        "the flooded column's books: {into} W in, {out} W out"
    ); // Boiled up and condensed: the reboiler takes the water to vapour at its
       // bubble point, the condenser takes back exactly its latent heat there.
    let latent = TroutonThermo::new()
        .dh_vap(&slate, WATER, boiling)
        .unwrap()
        .value()
        / 0.018015;
    let condenser = separation.condenser_duty.unwrap().value();
    let water_flow = 0.30 * FEED_KG_PER_S;
    assert!(
        (condenser - water_flow * latent).abs() < 1e-9 * condenser,
        "the condenser takes {condenser} W, the water's latent heat {} W",
        water_flow * latent
    );
}

/// **A feed the column can separate, it separates** — at 10 % water, under
/// where this column refuses, it fractionates as M62.5's does, flooding
/// nothing; and a feed all water is §67.17 rule 10's, not a flood.
#[test]
fn a_feed_the_column_can_separate_is_not_flooded() {
    let slate = slate();
    let separation = separate(&slate, &feed(0.10), &spec(Vec::new()), None).unwrap();
    assert_eq!(separation.water_flooded, None);
    assert!(separation.profile.is_some());
    let water = separate(&slate, &feed(1.0), &spec(Vec::new()), None).unwrap();
    assert_eq!(water.water_flooded, None);
    assert_eq!(water.draws[1].split, 1.0);
}

/// **Under the share, a refusal still halts with its reason** — at 10 % water,
/// a reflux of 0.01 that leaves the side draw's tray dry refuses as it always
/// did: the pass-through answers water, never a setting out of range. The same
/// setting at 30 % water floods, the refusal kept in the answer.
#[test]
fn under_the_share_a_refusal_is_not_answered() {
    const { assert!(0.10 < FLOODED_WATER_SHARE) };
    let slate = slate();
    let starved = CascadeSpec {
        reflux_ratio: 0.01,
        ..spec(Vec::new())
    };
    let err = separate(&slate, &feed(0.10), &starved, None)
        .expect_err("a reflux of 0.01 runs the side draw's tray dry");
    assert!(err.contains("heat-balance cascade"), "{err}");
    let flooded = separate(&slate, &feed(0.30), &starved, None).unwrap();
    assert!(flooded.water_flooded.is_some());
}

/// **Steam counts toward the share, and leaves by the water draw** — a feed
/// at 10 % water under 3 kg/s of steam is 30.8 % water by everything that
/// enters: the column refuses it, passes it through, and its water draw takes
/// the feed's water and the steam's together; every split is a share of the
/// feed and the steam (M62.6a).
#[test]
fn steam_counts_toward_the_share_and_leaves_by_the_water_draw() {
    let slate = slate();
    let steam = ColumnSteam {
        stage: 8,
        flow: KgPerSec(3.0),
        temperature: Kelvin(400.0 + 273.15),
    };
    let separation = separate(&slate, &feed(0.10), &spec(Vec::new()), Some(steam)).unwrap();
    assert!(
        separation.water_flooded.is_some(),
        "the steamed column floods"
    );
    let throughput = FEED_KG_PER_S + 3.0;
    let [_, water, _, bottoms] = &separation.draws[..] else {
        panic!("four draws");
    };
    assert!(
        (water.split * throughput - (1.0 + 3.0)).abs() < 1e-9,
        "the water draw takes {} kg/s",
        water.split * throughput
    );
    assert!((bottoms.split * throughput - 9.0).abs() < 1e-9);
}
