//! M57.2: the hydrocracking chemistry against checks that do not share its
//! method (docs/DESIGN.md §62.9).
//!
//! The model sums Bateman chains. Re-deriving those chains here would only check
//! the transcription, so every gate below reaches the same numbers another way:
//!
//! 1. the slope at `τ = 0` is the rate network applied to the feed — a path
//!    wired to the wrong lump, or a constant on the wrong path, moves it;
//! 2. reacting for `τ₁` and then `τ₂` is reacting for `τ₁ + τ₂` — the semigroup
//!    property only the true solution of a linear ODE has;
//! 3. a fixed-step RK4 written here, with many steps, agrees;
//! 4. at `τ = +∞` everything ends as gas, and a long finite `τ` meets that limit
//!    with no jump; with naphtha not cracking, the limit is a routing by hand;
//! 5. the calibration lands in Song 2017's industrial envelope (a regression
//!    lock on the calibration, not a validation — see the module note);
//! 6. the rates speed up about 4% per kelvin at 390 °C, as §62.3 says.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::traits::ReactionModel;
use refinery_core::units::{JPerKgK, Kelvin, KgPerM3, KgPerMol, Seconds};
use refinery_solvers::hydrocracking::T_REF_K;
use refinery_solvers::{Hydrocracking, HydrocrackingParams};

const R_GAS: f64 = 8.314_462_618;

/// Slate order deliberately not the model's lump order, plus an inert, so a
/// positional mix-up cannot pass by coincidence.
const NAMES: [&str; 5] = ["gas", "naphtha", "steam", "vgo", "distillate"];

fn slate() -> Slate {
    Slate::new(
        NAMES
            .iter()
            .map(|n| PseudoComponent {
                name: (*n).into(),
                tb: Kelvin(500.0),
                molar_mass: KgPerMol(0.2),
                density: Some(KgPerM3(800.0)),
                phase: Phase::Liquid,
                cp: JPerKgK(3200.0),
                cp_shape: None,
            })
            .collect(),
    )
    .expect("test slate")
}

/// A composition from model-order lumps `[vgo, distillate, naphtha, gas]` and
/// the inert's share.
fn feed(lumps: [f64; 4], inert: f64) -> Composition {
    Composition::from_weights(&[lumps[3], lumps[2], inert, lumps[0], lumps[1]]).expect("feed")
}

/// Model-order lumps out of a slate-order composition.
fn lumps(c: &Composition) -> [f64; 4] {
    let f = c.fractions();
    [f[3], f[4], f[1], f[0]]
}

/// A mixed feed: every lump present, so every path carries flux at once.
fn mixed() -> Composition {
    feed([0.55, 0.15, 0.08, 0.02], 0.20)
}

fn react(model: &Hydrocracking, c: &Composition, celsius: f64, tau: f64) -> Composition {
    model
        .react(c, Kelvin(celsius + 273.15), Seconds(tau), &slate())
        .expect("react")
        .products
}

/// The rate constants at `kelvin`, by the Arrhenius law about 390 °C [s⁻¹].
fn rates_at(params: &HydrocrackingParams, kelvin: f64) -> [f64; 6] {
    let shift = (-params.e_act / R_GAS * (1.0 / kelvin - 1.0 / T_REF_K)).exp();
    params.k_ref.map(|k| k * shift)
}

/// `dy/dτ` of the network, written from its figure (Sadighi 2010): six paths,
/// each leaving one lump and entering a lighter one.
fn network(y: [f64; 4], k: &[f64; 6]) -> [f64; 4] {
    let [vgo, distillate, naphtha, _] = y;
    [
        -(k[0] + k[1] + k[2]) * vgo,
        k[0] * vgo - (k[3] + k[4]) * distillate,
        k[1] * vgo + k[3] * distillate - k[5] * naphtha,
        k[2] * vgo + k[4] * distillate + k[5] * naphtha,
    ]
}

/// Classical RK4 over `[0, tau]` in `steps` equal steps.
fn rk4(mut y: [f64; 4], k: &[f64; 6], tau: f64, steps: usize) -> [f64; 4] {
    let h = tau / steps as f64;
    let add = |y: [f64; 4], d: [f64; 4], s: f64| [0, 1, 2, 3].map(|i| y[i] + s * d[i]);
    for _ in 0..steps {
        let a = network(y, k);
        let b = network(add(y, a, h / 2.0), k);
        let c = network(add(y, b, h / 2.0), k);
        let d = network(add(y, c, h), k);
        y = [0, 1, 2, 3].map(|i| y[i] + h / 6.0 * (a[i] + 2.0 * b[i] + 2.0 * c[i] + d[i]));
    }
    y
}

#[test]
fn the_initial_slope_is_the_rate_network_applied_to_the_feed() {
    let params = HydrocrackingParams::song_2017();
    let model = Hydrocracking::song_2017(&slate()).expect("model");
    let h = 1e-3; // s: the slope's truncation is h/2·|K²y|, ~1e-6 of it
    for celsius in [390.0, 425.0] {
        let k = rates_at(&params, celsius + 273.15);
        let y0 = lumps(&mixed());
        let want = network(y0, &k);
        let got = lumps(&react(&model, &mixed(), celsius, h));
        for i in 0..4 {
            let slope = (got[i] - y0[i]) / h;
            assert!(
                (slope - want[i]).abs() < 1e-5 * want[i].abs(),
                "{celsius} °C, lump {i}: slope {slope:e}, the network says {:e}",
                want[i]
            );
        }
    }
}

#[test]
fn reacting_for_two_times_in_turn_is_reacting_for_their_sum() {
    let model = Hydrocracking::song_2017(&slate()).expect("model");
    for (first, second) in [(100.0, 260.0), (3600.0, 7200.0), (0.5, 40_000.0)] {
        let once = lumps(&react(&model, &mixed(), 400.0, first + second));
        let halfway = react(&model, &mixed(), 400.0, first);
        let twice = lumps(&react(&model, &halfway, 400.0, second));
        for i in 0..4 {
            assert!(
                (once[i] - twice[i]).abs() < 1e-13,
                "τ = {first} + {second} s, lump {i}: {} at once, {} in turn",
                once[i],
                twice[i]
            );
        }
    }
}

#[test]
fn a_fine_rk4_agrees_with_the_exact_solution() {
    let params = HydrocrackingParams::song_2017();
    let model = Hydrocracking::song_2017(&slate()).expect("model");
    for (celsius, tau) in [(390.0, 3600.0), (440.0, 900.0), (360.0, 20_000.0)] {
        let k = rates_at(&params, celsius + 273.15);
        let want = rk4(lumps(&mixed()), &k, tau, 20_000);
        let got = lumps(&react(&model, &mixed(), celsius, tau));
        for i in 0..4 {
            assert!(
                (got[i] - want[i]).abs() < 1e-10,
                "{celsius} °C, τ = {tau} s, lump {i}: exact {}, RK4 {}",
                got[i],
                want[i]
            );
        }
    }
}

/// Two cracking lumps sharing one total rate is the case a naive Bateman
/// formula divides by zero on; the model's divided differences take it exactly.
#[test]
fn two_lumps_with_one_total_rate_are_still_exact() {
    let mut params = HydrocrackingParams::song_2017();
    // vgo's total rate made distillate's: k1 + k2 + k3 = k4 + k5.
    let distillate_total = params.k_ref[3] + params.k_ref[4];
    let vgo_total: f64 = params.k_ref[..3].iter().sum();
    for k in &mut params.k_ref[..3] {
        *k *= distillate_total / vgo_total;
    }
    let model = Hydrocracking::with_params(&slate(), params.clone()).expect("model");
    let k = rates_at(&params, T_REF_K);
    let (a, b) = (k[0] + k[1] + k[2], k[3] + k[4]);
    assert!(
        (a - b).abs() < 1e-15 * b,
        "the fixture must coincide: {a} vs {b}"
    );
    let want = rk4(lumps(&mixed()), &k, 7200.0, 20_000);
    let got = lumps(&react(&model, &mixed(), T_REF_K - 273.15, 7200.0));
    for i in 0..4 {
        assert!(
            (got[i] - want[i]).abs() < 1e-10,
            "lump {i}: {} vs {}",
            got[i],
            want[i]
        );
    }
}

#[test]
fn an_infinite_residence_time_ends_everything_as_gas() {
    let model = Hydrocracking::song_2017(&slate()).expect("model");
    let out = react(&model, &mixed(), 380.0, f64::INFINITY);
    let y = lumps(&out);
    assert_eq!(
        [y[0], y[1], y[2]],
        [0.0, 0.0, 0.0],
        "every cracking lump empties"
    );
    assert!(
        (y[3] - 0.80).abs() < 1e-14,
        "all 0.80 of the lumps is gas: {}",
        y[3]
    );
    assert!(
        (out.fractions()[2] - 0.20).abs() < 1e-15,
        "the inert is untouched"
    );

    // A long finite τ meets the limit with no jump: the slowest rate at 380 °C
    // is naphtha's, ~7e-6 s⁻¹, so 1e8 s leaves e^(−700) of anything.
    for tau in [1.0e8, 1.0e300] {
        let finite = lumps(&react(&model, &mixed(), 380.0, tau));
        for i in 0..4 {
            assert!(
                (finite[i] - y[i]).abs() < 1e-14,
                "τ = {tau:e} s, lump {i}: {} against the limit {}",
                finite[i],
                y[i]
            );
        }
    }
}

/// With naphtha not cracking, the limit is not all gas, and it is a routing a
/// hand can do: vgo splits by its rate constants' shares, and what reaches
/// distillate splits again by distillate's.
#[test]
fn with_naphtha_stable_the_limit_is_a_routing_by_hand() {
    let mut params = HydrocrackingParams::song_2017();
    params.k_ref[5] = 0.0;
    let model = Hydrocracking::with_params(&slate(), params.clone()).expect("model");
    let k = params.k_ref;
    let (vgo_total, distillate_total) = (k[0] + k[1] + k[2], k[3] + k[4]);
    let via_distillate = k[0] / vgo_total;
    let naphtha = k[1] / vgo_total + via_distillate * k[3] / distillate_total;
    let gas = k[2] / vgo_total + via_distillate * k[4] / distillate_total;
    let y = lumps(&react(
        &model,
        &feed([1.0, 0.0, 0.0, 0.0], 0.0),
        390.0,
        f64::INFINITY,
    ));
    assert_eq!([y[0], y[1]], [0.0, 0.0]);
    assert!(
        (y[2] - naphtha).abs() < 1e-15,
        "naphtha {} vs {naphtha}",
        y[2]
    );
    assert!((y[3] - gas).abs() < 1e-15, "gas {} vs {gas}", y[3]);
}

/// Song et al. (2017), Fig. 2: an industrial hydrocracker's outlet at 390 °C —
/// bottoms 27, distillate 47, naphtha 21, light ends 4 wt% — at an LHSV of
/// 1 h⁻¹ (`τ` = 3 600 s). The constants were FITTED to these figures
/// (normalised from their 99% sum), so this is a regression lock: it catches a
/// later slip in how `τ` or the temperature enters, not a wrong calibration.
#[test]
fn the_calibration_lands_in_song_2017s_industrial_envelope() {
    let model = Hydrocracking::song_2017(&slate()).expect("model");
    let pure_vgo = feed([1.0, 0.0, 0.0, 0.0], 0.0);
    let out = model
        .react(&pure_vgo, Kelvin(663.15), Seconds(3600.0), &slate())
        .expect("react");
    let y = lumps(&out.products);
    for (got, (name, published)) in y.iter().zip([
        ("bottoms", 0.27),
        ("distillate", 0.47),
        ("naphtha", 0.21),
        ("light ends", 0.04),
    ]) {
        assert!(
            (got - published).abs() < 0.01,
            "{name}: {got:.4} against Song 2017's {published}"
        );
    }
    // The heat it releases, from the chosen formation enthalpies by hand:
    // −(0.4747·350 + 0.2121·600 + 0.0404·1 200) kJ/kg ≈ −342 kJ/kg.
    let by_hand = -(y[1] * 350.0e3 + y[2] * 600.0e3 + y[3] * 1_200.0e3);
    assert!(
        (out.dh_rxn.value() - by_hand).abs() < 1e-6,
        "Δh_rxn {} J/kg, by hand {by_hand}",
        out.dh_rxn.value()
    );
    assert!((-345.0e3..-338.0e3).contains(&by_hand), "{by_hand}");
}

/// §62.3: "at 390 °C, about 4% per kelvin". The initial cracking rate of a pure
/// vgo feed, one kelvin apart.
#[test]
fn the_reaction_speeds_up_about_four_percent_per_kelvin_at_390_c() {
    let model = Hydrocracking::song_2017(&slate()).expect("model");
    let pure_vgo = feed([1.0, 0.0, 0.0, 0.0], 0.0);
    let cracked = |celsius: f64| 1.0 - lumps(&react(&model, &pure_vgo, celsius, 1.0))[0];
    let per_kelvin = cracked(391.0) / cracked(390.0) - 1.0;
    assert!(
        (0.039..0.043).contains(&per_kelvin),
        "{:.4} per kelvin",
        per_kelvin
    );
}
