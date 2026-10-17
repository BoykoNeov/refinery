//! Steam into a column on a stage, against an independent solve (M62.6,
//! docs/DESIGN.md §67.18).
//!
//! The reference is `tools/steptime/prototypes/m62_6/steam_column.py` (its
//! output kept beside it, `reference.txt`): M62.5's MESH solve of the same
//! column with one stream added — steam, pure water at 400 °C, joining its
//! stage's water balance and bringing `S·H_w(T_s)` to that stage's energy
//! balance. The engine instead walks the water vapour down from the free water
//! and takes each cut's K at the oil's partial pressure; the two meet only if
//! both are right.
//!
//! M62.5's `dry_trays` column (8 stages, feed on 5, four cuts and 1 % water at
//! 1.8 bar) with 0.15 kg/s of steam:
//!
//! - `steam_bottom`, on the reboiler (stage 8): the steam's heat is the
//!   reboiler's closure's, its partial pressure strips every stage;
//! - `steam_mid`, on stage 7: the steam's heat is a NEWTON EQUATION's — the
//!   case where a stage balance without the steam is caught, since on the
//!   reboiler only the closure reads it;
//! - `steam_dry_feed`, on stage 8 under a feed with no water: the column's
//!   water is the steam's alone.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::graph::{CascadeSpec, ColumnDraw, NodeId};
use refinery_core::traits::{ColumnPass, ColumnSteam, Separation, SeparationModel, ThermoModel};
use refinery_core::units::{JPerKgK, JPerMol, Kelvin, KgPerM3, KgPerMol, KgPerSec, Pascal};
use refinery_solvers::{HeatBalanceCascade, StageCascade, TroutonThermo};

const WATER: usize = 4;
const FEED_KG_PER_S: f64 = 10.0;
const STEAM_KG_PER_S: f64 = 0.15;
const STEAM_C: f64 = 400.0;
const PRESSURE: f64 = 1.8e5;
const FEED_C: f64 = 230.0;

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

/// The steam's liquid-equivalent temperature [K] at `temperature_c`, all
/// vapour: `cp·(T − T_REF) + λ(T)` per kilogram over `cp`, `λ` by Kirchhoff on
/// the vapour's heat capacity — the line flash's own steam, as the engine
/// hands it to a column.
fn liquid_equivalent(temperature_c: f64) -> Kelvin {
    let (cp, cp_vapour, molar_mass) = (4184.0, 2000.0, 0.018015);
    let latent = 40_650.0 / molar_mass + (cp_vapour - cp) * (temperature_c - 100.0);
    Kelvin(273.15 + temperature_c + latent / cp)
}

fn feed(water: f64) -> Composition {
    let mut weights: Vec<f64> = [0.2, 0.3, 0.3, 0.2]
        .iter()
        .map(|x| x * (1.0 - water))
        .collect();
    weights.push(water);
    Composition::from_weights(&weights).unwrap()
}

fn draws() -> Vec<ColumnDraw> {
    vec![
        ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
        ColumnDraw::water_draw(NodeId(2)),
        ColumnDraw::by_stage(NodeId(3), 3, Some(0.25)),
        ColumnDraw::by_stage(NodeId(4), 8, None),
    ]
}

fn spec() -> CascadeSpec {
    CascadeSpec {
        stages: 8,
        feed_stage: 5,
        reflux_ratio: 2.0,
        pumparounds: Vec::new(),
        steam: None,
    }
}

fn steam(stage: u32, flow: f64) -> ColumnSteam {
    ColumnSteam {
        stage,
        flow: KgPerSec(flow),
        temperature: liquid_equivalent(STEAM_C),
    }
}

fn separate(
    model: &dyn SeparationModel,
    feed: &Composition,
    feed_flow: f64,
    draws: &[ColumnDraw],
    steam: Option<ColumnSteam>,
) -> Result<Separation, String> {
    let slate = slate();
    let spec = spec();
    model
        .separate(
            &ColumnPass {
                slate: &slate,
                draws,
                smearing: Kelvin(0.0),
                pressure: Pascal(PRESSURE),
                feed,
                feed_flow: KgPerSec(feed_flow),
                temperature: Kelvin(FEED_C + 273.15),
                cascade: Some(&spec),
                seed: None,
                steam,
            },
            &TroutonThermo::new(),
        )
        .map_err(|e| e.to_string())
}

/// The reference's numbers for one case (`reference.txt`).
struct Expected {
    water: f64,
    stage: u32,
    temperatures_c: [f64; 8],
    condenser_c: f64,
    water_draw_kg_s: f64,
    distillate_kg_s: f64,
    distillate_light: f64,
    bottoms_kg_s: f64,
    bottoms_kerosene: f64,
    condenser_w: f64,
    reboiler_w: f64,
    feed_vapour_fraction: f64,
}

const STEAM_BOTTOM: Expected = Expected {
    water: 0.01,
    stage: 8,
    temperatures_c: [
        129.395735, 151.530999, 161.455072, 176.145261, 192.422501, 222.808573, 240.547276,
        256.898857,
    ],
    condenser_c: 85.97275209147142,
    water_draw_kg_s: 0.25,
    distillate_kg_s: 2.97,
    distillate_light: 0.6385111592364642,
    bottoms_kg_s: 4.455,
    bottoms_kerosene: 0.5492118604235973,
    condenser_w: 4161433.136778026,
    reboiler_w: 2626344.36843716,
    feed_vapour_fraction: 0.48122816505874716,
};

const STEAM_MID: Expected = Expected {
    water: 0.01,
    stage: 7,
    temperatures_c: [
        129.395673, 151.533922, 161.467802, 176.175667, 192.453958, 222.893574, 240.84777,
        265.03145,
    ],
    condenser_c: 85.97256223090199,
    water_draw_kg_s: 0.25,
    distillate_kg_s: 2.97,
    distillate_light: 0.6385192051023385,
    bottoms_kg_s: 4.455,
    bottoms_kerosene: 0.5489816165785336,
    condenser_w: 4161437.8080378883,
    reboiler_w: 2699902.7395379376,
    feed_vapour_fraction: 0.48122816505874716,
};

const STEAM_DRY_FEED: Expected = Expected {
    water: 0.0,
    stage: 8,
    temperatures_c: [
        131.648034, 154.072897, 164.14286, 179.096364, 195.845484, 222.698813, 240.183687,
        256.509766,
    ],
    condenser_c: 85.97560505622573,
    water_draw_kg_s: 0.15,
    distillate_kg_s: 3.0,
    distillate_light: 0.6383860830861446,
    bottoms_kg_s: 4.5,
    bottoms_kerosene: 0.548994918820949,
    condenser_w: 4004513.058099778,
    reboiler_w: 2529745.1691132886,
    feed_vapour_fraction: 0.4463411582943201,
};

fn close(what: &str, got: f64, want: f64, tolerance: f64) {
    assert!(
        (got - want).abs() <= tolerance,
        "{what}: {got} against the reference's {want} (tolerance {tolerance})"
    );
}

fn against_the_reference(expected: &Expected) {
    let separation = separate(
        &HeatBalanceCascade::new(),
        &feed(expected.water),
        FEED_KG_PER_S,
        &draws(),
        Some(steam(expected.stage, STEAM_KG_PER_S)),
    )
    .unwrap();
    let profile = separation.profile.as_ref().unwrap();
    for (j, (got, want)) in profile
        .temperatures
        .iter()
        .zip(expected.temperatures_c)
        .enumerate()
    {
        close(
            &format!("stage {} temperature [°C]", j + 1),
            got.value() - 273.15,
            want,
            1e-3,
        );
    }
    // No tray holds free water: every stage is hotter than water's dew point.
    for (j, w) in profile.free_water.iter().enumerate() {
        assert_eq!(*w, 0.0, "stage {} holds free water", j + 1);
    }
    // Every split is a share of the feed AND the steam (§67.18 rule 3).
    let throughput = FEED_KG_PER_S + STEAM_KG_PER_S;
    let [distillate, water, _side, bottoms] = &separation.draws[..] else {
        panic!("four draws, the water draw second");
    };
    close(
        "the condenser [°C]",
        water.temperature.value() - 273.15,
        expected.condenser_c,
        1e-3,
    );
    close(
        "the water draw [kg/s]",
        water.split * throughput,
        expected.water_draw_kg_s,
        1e-9,
    );
    assert_eq!(water.composition.fractions()[WATER], 1.0);
    close(
        "the distillate [kg/s]",
        distillate.split * throughput,
        expected.distillate_kg_s,
        1e-9,
    );
    close(
        "the distillate's light cut",
        distillate.composition.fractions()[0],
        expected.distillate_light,
        1e-6,
    );
    close(
        "the bottoms [kg/s]",
        bottoms.split * throughput,
        expected.bottoms_kg_s,
        1e-9,
    );
    assert_eq!(bottoms.composition.fractions()[WATER], 0.0);
    close(
        "the bottoms' kerosene",
        bottoms.composition.fractions()[2],
        expected.bottoms_kerosene,
        1e-6,
    );
    let split: f64 = separation.draws.iter().map(|d| d.split).sum();
    close("Σ split", split, 1.0, 1e-12);
    close(
        "the condenser duty [W]",
        separation.condenser_duty.unwrap().value(),
        expected.condenser_w,
        1e-5 * expected.condenser_w,
    );
    close(
        "the reboiler duty [W]",
        separation.reboiler_duty.unwrap().value(),
        expected.reboiler_w,
        1e-5 * expected.condenser_w,
    );
    close(
        "the feed's flash share",
        separation.feed_vapour_fraction.unwrap(),
        expected.feed_vapour_fraction,
        1e-9,
    );
    close(
        "the reflux ratio",
        separation.reflux_ratio.unwrap(),
        2.0,
        0.0,
    );
}

#[test]
fn steam_on_the_reboiler_strips_the_column_and_leaves_by_the_water_draw() {
    against_the_reference(&STEAM_BOTTOM);
}

#[test]
fn steam_on_a_stripping_tray_enters_that_trays_energy_balance() {
    against_the_reference(&STEAM_MID);
}

#[test]
fn steam_into_a_dry_feed_is_the_columns_only_water() {
    against_the_reference(&STEAM_DRY_FEED);
}

/// **A steam line carrying nothing is a column without steam, bit for bit** —
/// M62.5's answer, which the reference's `steam_zero` reproduces too.
#[test]
fn a_shut_steam_line_is_the_column_without_steam_bit_for_bit() {
    let model = HeatBalanceCascade::new();
    let wet = feed(0.01);
    let without = separate(&model, &wet, FEED_KG_PER_S, &draws(), None).unwrap();
    let shut = separate(&model, &wet, FEED_KG_PER_S, &draws(), Some(steam(8, 0.0))).unwrap();
    for (a, b) in without.draws.iter().zip(&shut.draws) {
        assert_eq!(a.split, b.split);
        assert_eq!(a.temperature, b.temperature);
        assert_eq!(a.composition.fractions(), b.composition.fractions());
    }
    assert_eq!(without.condenser_duty, shut.condenser_duty);
    assert_eq!(without.reboiler_duty, shut.reboiler_duty);
}

/// **Steam with no feed** — the feed pump tripped, the steam still on — is
/// answered: all of it condenses and leaves by the water draw at water's
/// boiling point, the condenser taking its heat down to liquid there, the
/// reboiler idle.
#[test]
fn steam_alone_condenses_into_the_water_draw() {
    let slate = slate();
    let separation = separate(
        &HeatBalanceCascade::new(),
        &feed(0.01),
        0.0,
        &draws(),
        Some(steam(8, STEAM_KG_PER_S)),
    )
    .unwrap();
    let water = &separation.draws[1];
    assert_eq!(water.split, 1.0);
    for (i, draw) in separation.draws.iter().enumerate() {
        if i != 1 {
            assert_eq!(draw.split, 0.0);
        }
    }
    let boiling = water.temperature.value() - 273.15;
    assert!((boiling - 117.0).abs() < 1.5, "{boiling} °C");
    let cp = 4184.0;
    let brought = STEAM_KG_PER_S * cp * (liquid_equivalent(STEAM_C).value() - 273.15);
    let left = STEAM_KG_PER_S * cp * boiling;
    close(
        "the condenser duty [W]",
        separation.condenser_duty.unwrap().value(),
        brought - left,
        1e-9 * brought,
    );
    assert_eq!(separation.reboiler_duty.unwrap().value(), 0.0);
    // Its heat at 400 °C and 1.8 bar, against steam tables' 3 276 kJ/kg less
    // the 491 kJ/kg of water boiling there (IAPWS-IF97).
    let per_kg = (brought - left) / STEAM_KG_PER_S;
    assert!(
        (per_kg / (3276.0e3 - 491.0e3) - 1.0).abs() < 0.01,
        "{per_kg}"
    );
    let _ = TroutonThermo::new().dh_vap(&slate, WATER, water.temperature);
}

/// **Refused by name**: steam on a stage the column does not have, steam into a
/// column with no water draw, and steam under the `"cascade"` model, whose
/// stages have no energy balances for it to enter.
#[test]
fn steam_the_column_cannot_take_is_refused() {
    let model = HeatBalanceCascade::new();
    let wet = feed(0.01);
    let e = separate(&model, &wet, FEED_KG_PER_S, &draws(), Some(steam(9, 0.15))).unwrap_err();
    assert!(e.contains("stage 9") && e.contains("1..=8"), "{e}");
    let e = separate(&model, &wet, FEED_KG_PER_S, &draws(), Some(steam(0, 0.15))).unwrap_err();
    assert!(e.contains("stage 0"), "{e}");
    let no_water_draw: Vec<ColumnDraw> = draws().into_iter().filter(|d| !d.water).collect();
    let e = separate(
        &model,
        &feed(0.0),
        FEED_KG_PER_S,
        &no_water_draw,
        Some(steam(8, 0.15)),
    )
    .unwrap_err();
    assert!(e.contains("no water draw"), "{e}");
    let e = separate(
        &StageCascade::new(),
        &feed(0.0),
        FEED_KG_PER_S,
        &no_water_draw,
        Some(steam(8, 0.15)),
    )
    .unwrap_err();
    assert!(e.contains("heat_balance"), "{e}");
}

/// **The steamed column's answer does not depend on where its solve starts**:
/// cold, warm from its own profile, and warm with every seed 10 K off.
#[test]
fn the_steamed_column_is_start_insensitive() {
    let slate = slate();
    let spec = spec();
    let draws = draws();
    let feed = feed(0.01);
    let run = |model: &HeatBalanceCascade, seed| {
        model
            .separate(
                &ColumnPass {
                    slate: &slate,
                    draws: &draws,
                    smearing: Kelvin(0.0),
                    pressure: Pascal(PRESSURE),
                    feed: &feed,
                    feed_flow: KgPerSec(FEED_KG_PER_S),
                    temperature: Kelvin(FEED_C + 273.15),
                    cascade: Some(&spec),
                    seed,
                    steam: Some(steam(7, STEAM_KG_PER_S)),
                },
                &TroutonThermo::new(),
            )
            .unwrap()
    };
    let cold = run(&HeatBalanceCascade::new(), None);
    let profile = cold.profile.clone().unwrap();
    let warm = run(&HeatBalanceCascade::new(), Some(&profile));
    let offset = run(
        &HeatBalanceCascade::with_seed_offset(Kelvin(10.0)),
        Some(&profile),
    );
    for other in [&warm, &offset] {
        let theirs = other.profile.as_ref().unwrap();
        for (a, b) in profile.temperatures.iter().zip(&theirs.temperatures) {
            close("a stage temperature [K]", b.value(), a.value(), 1e-4);
        }
        for (a, b) in cold.draws.iter().zip(&other.draws) {
            close("a split", b.split, a.split, 1e-9);
        }
    }
}
