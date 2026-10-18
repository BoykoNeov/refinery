//! A column whose feed carries free water, against an independent solve
//! (M62.5, docs/DESIGN.md §67.17).
//!
//! The reference is `tools/steptime/prototypes/m62_5/wet_column.py` (its output
//! kept beside it, `reference.txt`): the MESH equations of the same column
//! solved whole by scipy — each cut's K at the TOTAL pressure over a vapour
//! that holds water, the free water an unknown per tray, the wet trays found
//! by flipping until they hold. The engine takes each cut's K at the oil's
//! partial pressure, sweeps the liquids by Thomas and walks the water vapour
//! down from the free water; the two meet only if both are right.
//!
//! Two cases on one 8-stage column (feed on 5, four cuts and water):
//!
//! - `dry_trays`, 1 % water at 1.8 bar: the water rises from the feed tray as
//!   vapour, no tray holds any;
//! - `wet_trays`, 4 % water at 1 atm, declared reflux 1: the floor raises the
//!   reflux to 2.152, and free water stands on trays 1–5 and re-boils on the
//!   one below — checked by `check_engine.py`, the same equations with the
//!   floor, solved from the engine's own state.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::graph::{CascadeSpec, ColumnDraw, NodeId};
use refinery_core::traits::{ColumnPass, Separation, SeparationModel, ThermoModel};
use refinery_core::units::{JPerKgK, JPerMol, Kelvin, KgPerM3, KgPerMol, KgPerSec, Pascal};
use refinery_solvers::{HeatBalanceCascade, TroutonThermo};

const WATER: usize = 4;
const FEED_KG_PER_S: f64 = 10.0;

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

/// One of the reference's cases.
struct Case {
    oil: [f64; 4],
    water: f64,
    pressure: f64,
    feed_c: f64,
    reflux_ratio: f64,
    distillate_ratio: f64,
    side: (u32, f64),
}

const DRY_TRAYS: Case = Case {
    oil: [0.2, 0.3, 0.3, 0.2],
    water: 0.01,
    pressure: 1.8e5,
    feed_c: 230.0,
    reflux_ratio: 2.0,
    distillate_ratio: 0.3,
    side: (3, 0.25),
};

const WET_TRAYS: Case = Case {
    oil: [0.3, 0.3, 0.2, 0.2],
    water: 0.04,
    pressure: 1.01325e5,
    feed_c: 200.0,
    reflux_ratio: 1.0,
    distillate_ratio: 0.25,
    side: (6, 0.15),
};

impl Case {
    fn feed(&self, water: f64) -> Composition {
        let mut weights: Vec<f64> = self.oil.iter().map(|x| x * (1.0 - water)).collect();
        weights.push(water);
        Composition::from_weights(&weights).unwrap()
    }

    fn draws(&self, with_water_draw: bool) -> Vec<ColumnDraw> {
        let mut draws = vec![ColumnDraw::by_stage(
            NodeId(1),
            0,
            Some(self.distillate_ratio),
        )];
        if with_water_draw {
            draws.push(ColumnDraw::water_draw(NodeId(2)));
        }
        draws.push(ColumnDraw::by_stage(
            NodeId(3),
            self.side.0,
            Some(self.side.1),
        ));
        draws.push(ColumnDraw::by_stage(NodeId(4), 8, None));
        draws
    }

    fn spec(&self) -> CascadeSpec {
        CascadeSpec {
            stages: 8,
            feed_stage: 5,
            reflux_ratio: self.reflux_ratio,
            pumparounds: Vec::new(),
            steam: None,
            reboiler_duty: None,
        }
    }

    fn run_with(&self, slate: &Slate, feed: &Composition, draws: &[ColumnDraw]) -> Separation {
        let spec = self.spec();
        HeatBalanceCascade::new()
            .separate(
                &ColumnPass {
                    slate,
                    draws,
                    smearing: Kelvin(0.0),
                    pressure: Pascal(self.pressure),
                    feed,
                    feed_flow: KgPerSec(FEED_KG_PER_S),
                    temperature: Kelvin(self.feed_c + 273.15),
                    cascade: Some(&spec),
                    seed: None,
                    running_reboiler: None,
                    earlier_pass: None,
                    steam: None,
                },
                &TroutonThermo::new(),
            )
            .unwrap()
    }

    fn run(&self) -> Separation {
        let slate = slate();
        self.run_with(&slate, &self.feed(self.water), &self.draws(true))
    }
}

/// The reference's numbers for one case (`reference.txt`).
struct Expected {
    temperatures_c: [f64; 8],
    free_water: [f64; 8],
    condenser_c: f64,
    water_draw_kg_s: f64,
    distillate_kg_s: f64,
    distillate_light: f64,
    bottoms_kg_s: f64,
    bottoms_kerosene: f64,
    condenser_w: f64,
    reboiler_w: f64,
    /// The reflux ratio the column runs at: the declared one, or above it
    /// where the floor binds.
    reflux_ratio: f64,
    /// The share of the feed's moles vapour once it has flashed adiabatically
    /// at the column's pressure, water and oil (§67.17 rule 11) — the
    /// reference's `feed_flash_share`.
    feed_vapour_fraction: f64,
}

const DRY_EXPECTED: Expected = Expected {
    temperatures_c: [
        132.783913, 155.334582, 165.461166, 180.484047, 197.260794, 231.58102, 248.585526,
        265.065749,
    ],
    free_water: [0.0; 8],
    condenser_c: 86.08688996425269,
    water_draw_kg_s: 0.1,
    distillate_kg_s: 2.97,
    distillate_light: 0.638302886379038,
    bottoms_kg_s: 4.455,
    bottoms_kerosene: 0.5490840167692286,
    condenser_w: 3869018.965083436,
    reboiler_w: 2866533.383717073,
    reflux_ratio: 2.0,
    feed_vapour_fraction: 0.48122816505874716,
};

/// `check_engine.txt`: at the declared reflux of 1 this column's stripping
/// vapour falls under the floor (20 % of the oil's moles), so the engine
/// raises its reflux to 2.152 to hold stage 6 there, and the wet zone moves to
/// trays 1–5. The reference solves the same equations with that floor, from
/// the engine's state, and settles where the engine did (residual 1.2e-15, its
/// own checks held: no dry tray past saturation, no wet one with negative free
/// water, no stage under the floor).
const WET_EXPECTED: Expected = Expected {
    temperatures_c: [
        62.227842, 62.349099, 63.329378, 69.168583, 82.747566, 98.652314, 121.61348, 155.359552,
    ],
    free_water: [
        0.09840151922832972,
        0.9066440533493163,
        6.217217400705946,
        22.625006182112454,
        3.2624485247551265,
        0.0,
        0.0,
        0.0,
    ],
    condenser_c: 62.21353147373918,
    water_draw_kg_s: 0.4,
    distillate_kg_s: 2.4,
    distillate_light: 0.9999165877088064,
    bottoms_kg_s: 5.76,
    bottoms_kerosene: 0.282311946802264,
    condenser_w: 3340191.207385565,
    reboiler_w: 1547003.1382592362,
    reflux_ratio: 2.1517153755875884,
    feed_vapour_fraction: 0.5773175681778613,
};

fn close(what: &str, got: f64, want: f64, tolerance: f64) {
    assert!(
        (got - want).abs() <= tolerance,
        "{what}: {got} against the reference's {want} (tolerance {tolerance})"
    );
}

fn against_the_reference(case: &Case, expected: &Expected) {
    let separation = case.run();
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
    for (j, (got, want)) in profile
        .free_water
        .iter()
        .zip(expected.free_water)
        .enumerate()
    {
        close(
            &format!("stage {} free water [mol/s]", j + 1),
            *got,
            want,
            1e-4,
        );
    }
    let [distillate, water, _side, bottoms] = &separation.draws[..] else {
        panic!("four draws, the water draw second");
    };
    close(
        "the condenser [°C]",
        water.temperature.value() - 273.15,
        expected.condenser_c,
        1e-3,
    );
    assert_eq!(distillate.temperature, water.temperature);
    close(
        "the water draw [kg/s]",
        water.split * FEED_KG_PER_S,
        expected.water_draw_kg_s,
        1e-9,
    );
    assert_eq!(water.composition.fractions()[WATER], 1.0);
    close(
        "the distillate [kg/s]",
        distillate.split * FEED_KG_PER_S,
        expected.distillate_kg_s,
        1e-9,
    );
    assert_eq!(distillate.composition.fractions()[WATER], 0.0);
    close(
        "the distillate's light cut",
        distillate.composition.fractions()[0],
        expected.distillate_light,
        1e-6,
    );
    close(
        "the bottoms [kg/s]",
        bottoms.split * FEED_KG_PER_S,
        expected.bottoms_kg_s,
        1e-9,
    );
    close(
        "the bottoms' kerosene",
        bottoms.composition.fractions()[2],
        expected.bottoms_kerosene,
        1e-6,
    );
    let split: f64 = separation.draws.iter().map(|d| d.split).sum();
    close("Σ split", split, 1.0, 1e-12);
    let condenser = separation.condenser_duty.unwrap().value();
    let reboiler = separation.reboiler_duty.unwrap().value();
    close(
        "the condenser duty [W]",
        condenser,
        expected.condenser_w,
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
        expected.reflux_ratio,
        1e-6,
    );
    close(
        "the reboiler duty [W]",
        reboiler,
        expected.reboiler_w,
        1e-5 * expected.condenser_w,
    );
}

#[test]
fn water_rides_up_a_column_whose_trays_are_hotter_than_its_dew_point() {
    against_the_reference(&DRY_TRAYS, &DRY_EXPECTED);
}

#[test]
fn free_water_stands_on_the_top_trays_and_reboils_below_them() {
    against_the_reference(&WET_TRAYS, &WET_EXPECTED);
}

/// **A dry feed on a wet slate is the dry column, bit for bit**: no standing
/// water, so none of M62.5 runs — the oil draws are those of the same column
/// on a slate with no water at all, and the water draw carries nothing.
#[test]
fn a_dry_feed_on_a_wet_slate_is_the_dry_column_bit_for_bit() {
    let wet_slate = slate();
    let case = &DRY_TRAYS;
    let wet = case.run_with(&wet_slate, &case.feed(0.0), &case.draws(true));
    let mut components: Vec<PseudoComponent> = (0..wet_slate.len())
        .map(|c| wet_slate.get(c).clone())
        .collect();
    components[WATER].free_water = false;
    let dry_slate = Slate::new(components).unwrap();
    let dry = case.run_with(&dry_slate, &case.feed(0.0), &case.draws(false));
    assert_eq!(wet.draws.len(), dry.draws.len() + 1);
    assert_eq!(wet.draws[1].split, 0.0);
    for (w, d) in [&wet.draws[0], &wet.draws[2], &wet.draws[3]]
        .into_iter()
        .zip(&dry.draws)
    {
        assert_eq!(w.split, d.split);
        assert_eq!(w.temperature, d.temperature);
        assert_eq!(w.composition.fractions(), d.composition.fractions());
    }
    assert_eq!(wet.condenser_duty, dry.condenser_duty);
    assert_eq!(wet.reboiler_duty, dry.reboiler_duty);
}

/// **A feed that is all water** — the slug a crude tank sends first (M62.3) —
/// is answered: every drop by the water draw at water's boiling point at the
/// column's pressure, the oil draws empty.
#[test]
fn a_feed_of_water_alone_leaves_by_the_water_draw_at_its_boiling_point() {
    let slate = slate();
    let case = &DRY_TRAYS;
    let all_water = Case {
        feed_c: 100.0,
        ..DRY_TRAYS
    };
    let separation = all_water.run_with(&slate, &case.feed(1.0), &case.draws(true));
    let water = &separation.draws[1];
    assert_eq!(water.split, 1.0);
    // Water boils at 1.8 bar near 117 °C (steam tables 116.9 °C).
    let boiling = water.temperature.value() - 273.15;
    assert!((boiling - 117.0).abs() < 1.5, "{boiling} °C");
    for (i, draw) in separation.draws.iter().enumerate() {
        if i != 1 {
            assert_eq!(draw.split, 0.0);
        }
    }
    let latent = FEED_KG_PER_S / 0.018015
        * TroutonThermo::new()
            .dh_vap(&slate, WATER, water.temperature)
            .unwrap()
            .value();
    close(
        "the condenser duty [W]",
        separation.condenser_duty.unwrap().value(),
        latent,
        1e-6 * latent,
    );
}

/// **A wet feed with nowhere to decant its water is refused by name.**
#[test]
fn a_wet_feed_without_a_water_draw_is_refused() {
    let slate = slate();
    let case = &DRY_TRAYS;
    let spec = case.spec();
    let draws = case.draws(false);
    let feed = case.feed(case.water);
    let error = HeatBalanceCascade::new()
        .separate(
            &ColumnPass {
                slate: &slate,
                draws: &draws,
                smearing: Kelvin(0.0),
                pressure: Pascal(case.pressure),
                feed: &feed,
                feed_flow: KgPerSec(FEED_KG_PER_S),
                temperature: Kelvin(case.feed_c + 273.15),
                cascade: Some(&spec),
                seed: None,
                running_reboiler: None,
                earlier_pass: None,
                steam: None,
            },
            &TroutonThermo::new(),
        )
        .unwrap_err()
        .to_string();
    assert!(error.contains("no water draw"), "{error}");
}

/// **The wet column's answer does not depend on where its solve starts**
/// (DESIGN §5 fork 5): cold, warm from its own profile — whose free water
/// starts the wet set — and warm with every seed temperature 10 K off. A
/// system that switches trays between wet and dry is where a second steady
/// state would hide.
#[test]
fn the_wet_column_is_start_insensitive() {
    let slate = slate();
    let case = &WET_TRAYS;
    let spec = case.spec();
    let draws = case.draws(true);
    let feed = case.feed(case.water);
    let separate = |model: &HeatBalanceCascade, seed| {
        model
            .separate(
                &ColumnPass {
                    slate: &slate,
                    draws: &draws,
                    smearing: Kelvin(0.0),
                    pressure: Pascal(case.pressure),
                    feed: &feed,
                    feed_flow: KgPerSec(FEED_KG_PER_S),
                    temperature: Kelvin(case.feed_c + 273.15),
                    cascade: Some(&spec),
                    seed,
                    running_reboiler: None,
                    earlier_pass: None,
                    steam: None,
                },
                &TroutonThermo::new(),
            )
            .unwrap()
    };
    let cold = separate(&HeatBalanceCascade::new(), None);
    let profile = cold.profile.clone().unwrap();
    assert!(profile.free_water.iter().any(|w| *w > 0.0));
    let warm = separate(&HeatBalanceCascade::new(), Some(&profile));
    let offset = separate(
        &HeatBalanceCascade::with_seed_offset(Kelvin(10.0)),
        Some(&profile),
    );
    for other in [&warm, &offset] {
        let theirs = other.profile.as_ref().unwrap();
        for (a, b) in profile.temperatures.iter().zip(&theirs.temperatures) {
            close("a stage temperature [K]", b.value(), a.value(), 1e-4);
        }
        for (a, b) in profile.free_water.iter().zip(&theirs.free_water) {
            close("a stage's free water [mol/s]", *b, *a, 1e-5);
        }
        for (a, b) in cold.draws.iter().zip(&other.draws) {
            close("a draw's split", b.split, a.split, 1e-9);
        }
    }
}
