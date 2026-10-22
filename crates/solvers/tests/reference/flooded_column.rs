//! M63.4 — a column flooded with water (docs/DESIGN.md §68.4, ledger row C13):
//! a feed at least `FLOODED_WATER_SHARE` water that the heat-balance cascade
//! cannot separate passes straight through for the tick — its water out by the
//! water draw at water's bubble point — and the refusal rides the answer. Its
//! oil, below the feed's bubble point, falls to the bottoms whole at the feed's
//! temperature (M63.4); above it the column is a flash zone with no trays
//! (M65.2, §70.2): the oil's liquid to the bottoms at the flash temperature,
//! its vapour condensed to the distillate at its bubble point. M62.5's
//! eight-stage column (`reference/wet_column.rs`) separates to 16 % water and
//! refuses from 18 %; the books here are the column's own: what flows in, plus
//! the reboiler, is what flows out, plus the condenser.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::energy::T_REF;
use refinery_core::graph::{CascadeSpec, ColumnDraw, NodeId, Pumparound};
use refinery_core::traits::{
    ColumnPass, ColumnSteam, DrawSeparation, Separation, SeparationModel, ThermoModel,
};
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
        reboiler_duty: None,
        loops: Vec::new(),
        trays: Vec::new(),
        holdup: None,
    }
}

fn separate(
    slate: &Slate,
    feed: &Composition,
    spec: &CascadeSpec,
    steam: Option<ColumnSteam>,
) -> Result<Separation, String> {
    separate_at(slate, feed, spec, steam, FEED_C)
}

/// One pass with the feed at `feed_c` [°C].
fn separate_at(
    slate: &Slate,
    feed: &Composition,
    spec: &CascadeSpec,
    steam: Option<ColumnSteam>,
    feed_c: f64,
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
                temperature: Kelvin(feed_c + 273.15),
                cascade: Some(spec),
                seed: None,
                running_reboiler: None,
                earlier_pass: None,
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

/// A bubble point [K] of `composition` at the column's pressure, computed here.
fn bubble(slate: &Slate, composition: &Composition) -> Kelvin {
    bubble_temperature(
        slate,
        &TroutonThermo::new(),
        &MoleFractions::from_mass(composition, slate).unwrap(),
        Pascal(PRESSURE),
        "a test",
    )
    .unwrap()
}

/// The column's own books [W]: the feed's flux plus the reboiler's duty, less
/// what the draws carry out and the condenser's duty, over `throughput` [kg/s].
fn books(slate: &Slate, separation: &Separation, inflow: f64, throughput: f64) -> (f64, f64) {
    let into = inflow + separation.reboiler_duty.unwrap().value();
    let out: f64 = separation
        .draws
        .iter()
        .map(|d| flux(slate, &d.composition, d.split * throughput, d.temperature))
        .sum::<f64>()
        + separation.condenser_duty.unwrap().value();
    (into, out)
}

/// The oil's mass flows [kg/s] a draw carries, by cut.
fn oil_cuts(draw: &DrawSeparation, throughput: f64) -> Vec<f64> {
    draw.composition.fractions()[..WATER]
        .iter()
        .map(|x| x * draw.split * throughput)
        .collect()
}

/// **Too wet to separate, a hot feed floods the column as a flash zone**
/// (M65.2, §70.2) — at 30 % water, past the 18 % where this column's solve
/// refuses, and at 230 °C, above the feed's bubble point: the water draw takes
/// every drop of water, a liquid at water's bubble point at the column's
/// pressure; the oil splits between the distillate (its vapour, condensed at its
/// own bubble point) and the bottoms (its liquid, at the flash temperature,
/// between the feed's bubble point and the feed's temperature), cut by cut
/// every kilogram accounted for, the light cuts overhead and the heavy below;
/// the side draw carries nothing. The bottoms' liquid stands below its own
/// bubble point at the column's pressure — it boiled at the oil's partial
/// pressure — so a pump can take it. The refusal rides the answer, and no
/// profile does. The books close, and the condenser takes the water's latent
/// heat and what the oil gives up besides.
#[test]
fn a_hot_feed_too_wet_to_separate_floods_as_a_flash_zone() {
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
    let feed_bubble = bubble(&slate, &wet);
    assert!(feed_bubble.value() < FEED_C + 273.15, "the feed flashes");
    assert_eq!(side.split, 0.0);
    assert!(distillate.split > 0.0 && bottoms.split > 0.0);
    assert!(
        (water.split - 0.30).abs() < 1e-12,
        "water draw {}",
        water.split
    );
    assert_eq!(water.composition.fractions()[WATER], 1.0);
    assert_eq!(water.temperature, bubble(&slate, &water.composition));

    // Every cut's kilograms: overhead and bottoms together are the feed's.
    let (over, below) = (
        oil_cuts(distillate, FEED_KG_PER_S),
        oil_cuts(bottoms, FEED_KG_PER_S),
    );
    for (c, want) in OIL.iter().enumerate() {
        let got = over[c] + below[c];
        let want = want * 0.70 * FEED_KG_PER_S;
        assert!(
            (got - want).abs() < 1e-12 * FEED_KG_PER_S,
            "cut {c}: {got} against {want}"
        );
    }
    assert_eq!(distillate.composition.fractions()[WATER], 0.0);
    assert_eq!(bottoms.composition.fractions()[WATER], 0.0);
    // The light cuts go overhead, the heavy stay below.
    assert!(over[0] / below[0].max(1e-30) > over[3] / below[3].max(1e-30));

    // The bottoms at the flash temperature, between the bubble point and the
    // feed, and below its own bubble point at the column's pressure; the
    // distillate condensed at its own.
    let flash = bottoms.temperature.value();
    assert!(
        feed_bubble.value() < flash && flash < FEED_C + 273.15,
        "flash at {flash} K"
    );
    assert!(bubble(&slate, &bottoms.composition).value() >= flash);
    assert_eq!(
        distillate.temperature,
        bubble(&slate, &distillate.composition)
    );
    assert_eq!(side.temperature, bottoms.temperature);

    let inflow = flux(&slate, &wet, FEED_KG_PER_S, Kelvin(FEED_C + 273.15));
    let (into, out) = books(&slate, &separation, inflow, FEED_KG_PER_S);
    assert!(
        (into - out).abs() < 1e-9 * into,
        "the flooded column's books: {into} W in, {out} W out"
    );
    // The condenser: the water's latent heat at its bubble point, boiled up
    // and condensed, and what the oil gives up between the feed and its draws.
    let latent = TroutonThermo::new()
        .dh_vap(&slate, WATER, water.temperature)
        .unwrap()
        .value()
        / 0.018015;
    let oil_gives = flux(
        &slate,
        &feed(0.0),
        0.70 * FEED_KG_PER_S,
        Kelvin(FEED_C + 273.15),
    ) - flux(
        &slate,
        &distillate.composition,
        distillate.split * FEED_KG_PER_S,
        distillate.temperature,
    ) - flux(
        &slate,
        &bottoms.composition,
        bottoms.split * FEED_KG_PER_S,
        bottoms.temperature,
    );
    let condenser = separation.condenser_duty.unwrap().value();
    let water_flow = 0.30 * FEED_KG_PER_S;
    assert!(oil_gives > 0.0);
    assert!(
        (condenser - water_flow * latent - oil_gives).abs() < 1e-9 * condenser,
        "the condenser takes {condenser} W, the water's latent heat {} W and the oil's {oil_gives} W",
        water_flow * latent
    );
}

/// **Below its bubble point, a flooded feed's oil passes whole** (M63.4's rule,
/// kept): at 30 % water and 60 °C, under the feed's bubble point at the
/// column's pressure, the bottoms takes the oil whole at the feed's
/// temperature, the distillate and the side draw nothing, and the condenser
/// exactly the water's latent heat — the reboiler having boiled it up.
#[test]
fn a_cool_feed_too_wet_to_separate_passes_its_oil_whole() {
    const COOL_C: f64 = 60.0;
    let slate = slate();
    let wet = feed(0.30);
    assert!(
        bubble(&slate, &wet).value() > COOL_C + 273.15,
        "the feed does not flash"
    );
    let separation = separate_at(&slate, &wet, &spec(Vec::new()), None, COOL_C).unwrap();
    assert!(separation.water_flooded.is_some(), "the column flooded");
    let [distillate, water, side, bottoms] = &separation.draws[..] else {
        panic!("four draws, the water draw second");
    };
    assert_eq!((distillate.split, side.split), (0.0, 0.0));
    assert!(
        (bottoms.split - 0.70).abs() < 1e-12,
        "bottoms {}",
        bottoms.split
    );
    for (got, want) in bottoms.composition.fractions()[..4].iter().zip(OIL) {
        assert!(
            (got - want).abs() < 1e-12,
            "the oil passes whole: {got} against {want}"
        );
    }
    assert_eq!(bottoms.temperature, Kelvin(COOL_C + 273.15));
    let inflow = flux(&slate, &wet, FEED_KG_PER_S, Kelvin(COOL_C + 273.15));
    let (into, out) = books(&slate, &separation, inflow, FEED_KG_PER_S);
    assert!((into - out).abs() < 1e-9 * into, "{into} W in, {out} W out");
    let latent = TroutonThermo::new()
        .dh_vap(&slate, WATER, water.temperature)
        .unwrap()
        .value()
        / 0.018015;
    let condenser = separation.condenser_duty.unwrap().value();
    assert!((condenser - 0.30 * FEED_KG_PER_S * latent).abs() < 1e-9 * condenser);
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
/// the feed's water and the steam's together, the oil the rest; every split is
/// a share of the feed and the steam (M62.6a).
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
    let [distillate, water, _, bottoms] = &separation.draws[..] else {
        panic!("four draws");
    };
    assert!(
        (water.split * throughput - (1.0 + 3.0)).abs() < 1e-9,
        "the water draw takes {} kg/s",
        water.split * throughput
    );
    // The oil, overhead and below together (the feed flashes, M65.2).
    assert!(((distillate.split + bottoms.split) * throughput - 9.0).abs() < 1e-9);
}
