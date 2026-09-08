//! The heat-capacity seam, at the level of the models themselves (M16.2,
//! docs/DESIGN.md §20).
//!
//! **What lives here and what does not.** These gates ask the two
//! `EnthalpyModel` implementations questions directly, with hand-built slates and
//! no plant. That separation is §20's own: gate 1 is the only test in the
//! workspace with power over *the shape*, because §18's finding is that a uniform
//! `cp` scale cancels exactly in enthalpy-weighted mixing — so no plant-level
//! comparison of a scaled twin can tell a shape from a constant. The DEMO proves
//! something different, that the seam is wired and moves a published number, and
//! it is gated in `refinery-scenarios`.

use refinery_core::components::{Composition, CpShape, Phase, PseudoComponent, Slate};
use refinery_core::energy::T_REF;
use refinery_core::traits::{EnthalpyModel, InflowEnthalpy};
use refinery_core::units::{JPerKgK, Kelvin, Kg, KgPerSec, R_GAS};
use refinery_solvers::{ConstantEnthalpy, LinearCpEnthalpy};

/// The demo's own methane shape (see `scenarios/fired_gas_drum.toml`), so these
/// gates and the wired plant cannot disagree about what is being integrated.
const ANCHOR_K: f64 = 300.0;
const CP_AT_ANCHOR: f64 = 2187.35;
const SLOPE: f64 = 3.521_407_111_2;
const METHANE_MOLAR_MASS: f64 = 0.016_043;

fn shape() -> CpShape {
    CpShape {
        anchor_temperature: Kelvin(ANCHOR_K),
        cp_at_anchor: JPerKgK(CP_AT_ANCHOR),
        slope: SLOPE,
    }
}

/// `(cp(T_REF), slope)` — the shape on the engine's own datum, which is the pair
/// every consumer actually reads.
fn at_datum() -> (f64, f64) {
    shape().at_datum(T_REF)
}

fn shaped_slate() -> Slate {
    let s = shape();
    Slate::new(vec![PseudoComponent {
        name: "fuel_gas".into(),
        tb: Kelvin(111.65),
        molar_mass: refinery_core::units::KgPerMol(METHANE_MOLAR_MASS),
        density: None,
        cp: JPerKgK(s.at_datum(T_REF).0),
        cp_shape: Some(s),
        phase: Phase::Gas,
    }])
    .expect("a one-cut slate")
}

/// The same cut with the shape removed and the corpus's flat 2220 J/(kg·K) in its
/// place — the number the other five gas plants declare.
fn flat_slate() -> Slate {
    Slate::new(vec![PseudoComponent {
        name: "fuel_gas".into(),
        tb: Kelvin(111.65),
        molar_mass: refinery_core::units::KgPerMol(METHANE_MOLAR_MASS),
        density: None,
        cp: JPerKgK(2220.0),
        cp_shape: None,
        phase: Phase::Gas,
    }])
    .expect("a one-cut slate")
}

fn pure() -> Composition {
    Composition::pure(1, 0)
}

// ---------------------------------------------------------------------------
// Gate 1 — two equal-width spans. THE gate with power over the shape.
// ---------------------------------------------------------------------------

/// A mean capacity over `[300, 340]` and over `[700, 740]`.
///
/// **Equal width is load-bearing**: unequal intervals would be passed by a model
/// that merely reported the interval back, and a wider interval at a higher
/// temperature would confound "the shape rises" with "the interval is longer".
/// The two answers must differ by exactly the slope times the gap between the
/// interval midpoints, which is a prediction of the declared shape and not of the
/// engine.
///
/// **The constant model is the control and must return the SAME number twice.**
/// Without it this gate would pass on a model that returned any temperature-
/// dependent nonsense, and with it the gate says the difference comes from the
/// declared shape rather than from the plumbing.
///
/// **The equal-width difference alone is NOT enough, and the mutation pass is
/// what said so** (§20). Mutation 1 — return the spot value at `t₁` where a mean
/// is wanted — passes every assertion above: for a linear shape the mean over
/// `[a, b]` is the spot value at the MIDPOINT, so reading it at the left edge
/// instead shifts both answers by the same `s·(b − a)/2` and the difference the
/// gate measures is unmoved. What closes it is the midpoint identity itself,
/// asserted below together with the magnitude by which the endpoint values miss
/// it — a number the declared shape predicts, not a tolerance.
#[test]
fn the_mean_capacity_over_two_equal_spans_differs_only_under_a_shape() {
    let shaped = shaped_slate();
    let flat = flat_slate();
    let x = pure();
    let low = (Kelvin(300.0), Kelvin(340.0));
    let high = (Kelvin(700.0), Kelvin(740.0));

    let cold = LinearCpEnthalpy
        .mean_cp(&shaped, &x, low.0, low.1)
        .expect("a declared shape answers");
    let hot = LinearCpEnthalpy
        .mean_cp(&shaped, &x, high.0, high.1)
        .expect("a declared shape answers");

    // The midpoints are 320 K and 720 K, so the difference is `slope · 400`.
    let (_, slope) = at_datum();
    approx::assert_relative_eq!(
        hot.value() - cold.value(),
        slope * 400.0,
        max_relative = 1e-12
    );
    assert!(
        hot.value() > cold.value() * 1.4,
        "the two means must be distinguishable by more than rounding: {} vs {}",
        cold.value(),
        hot.value()
    );

    // The mean IS the spot value at the interval's midpoint, and is not the spot
    // value at either end of it. `slope · 20` is the half-width times the slope —
    // 70.4 J/(kg·K) on this shape — so an implementation that read the capacity
    // at an endpoint would be off by that, in a gate that otherwise cannot see it.
    let midpoint = LinearCpEnthalpy
        .spot_cp(&shaped, &x, Kelvin(320.0))
        .expect("a declared shape answers");
    approx::assert_relative_eq!(cold.value(), midpoint.value(), max_relative = 1e-12);
    for (end, sign) in [(low.0, 1.0), (low.1, -1.0)] {
        let spot = LinearCpEnthalpy
            .spot_cp(&shaped, &x, end)
            .expect("a declared shape answers");
        approx::assert_relative_eq!(
            cold.value() - spot.value(),
            sign * slope * 20.0,
            max_relative = 1e-12
        );
        assert!(
            (cold.value() - spot.value()).abs() > 60.0,
            "the midpoint must be distinguishable from the endpoints: {} vs {}",
            cold.value(),
            spot.value()
        );
    }

    // The control: one constant, two intervals, one answer — bit for bit.
    let cold_flat = ConstantEnthalpy.mean_cp(&flat, &x, low.0, low.1).unwrap();
    let hot_flat = ConstantEnthalpy.mean_cp(&flat, &x, high.0, high.1).unwrap();
    assert_eq!(cold_flat.value().to_bits(), hot_flat.value().to_bits());
}

// ---------------------------------------------------------------------------
// Gate 4 — the inversion round-trip.
// ---------------------------------------------------------------------------

/// `T(h(T)) = T` and `T(u(T)) = T` across the demo's whole range, to a bound
/// DERIVED from the shape rather than chosen.
///
/// **The derivation.** The round trip loses only floating-point precision: `h` is
/// computed to a relative accuracy of a few `f64` epsilons, and reading a
/// temperature back divides that error by `dh/dT = cp`. So the absolute error is
/// bounded by `k·ε·h(T)/cp(T)`, which over this range is at most
/// `k · 2.2e-16 · 1.6e6 / 2100 ≈ k · 1.7e-13 K` for a small `k`. The assertion
/// uses `1e-9 K` — four orders above that and, per M9.3a's rule, far BELOW the
/// tolerance of anything that grades it: the energy balances this feeds work at
/// `1e-8` relative on temperatures of several hundred Kelvin, i.e. microkelvins.
///
/// The internal-energy leg is the one that would survive a dropped constant term
/// only by accident, which is why both are here.
#[test]
fn the_inversion_round_trips_across_the_demos_range() {
    let slate = shaped_slate();
    let x = pure();
    let mass = 7.0;
    let mut worst_h: f64 = 0.0;
    let mut worst_u: f64 = 0.0;
    for step in 0..=120 {
        let t = Kelvin(T_REF.value() + f64::from(step) * 5.5);
        let h = LinearCpEnthalpy
            .enthalpy_stock(&slate, &x, Kg(mass), t)
            .expect("a declared shape answers");
        let back = LinearCpEnthalpy
            .temperature_from_enthalpy(&slate, &x, h, mass)
            .expect("h is monotone over this range");
        worst_h = worst_h.max((back.value() - t.value()).abs());

        let u = LinearCpEnthalpy
            .specific_internal_energy(&slate, &x, t)
            .expect("a declared shape answers")
            .value()
            * mass;
        let back_u = LinearCpEnthalpy
            .temperature_from_internal_energy(&slate, &x, u, mass)
            .expect("u is monotone over this range");
        worst_u = worst_u.max((back_u.value() - t.value()).abs());
    }
    assert!(
        worst_h < 1e-9 && worst_u < 1e-9,
        "round trip must close to the float noise: h {worst_h:.3e} K, u {worst_u:.3e} K"
    );
}

/// The other root of the same quadratic is not a temperature this plant can be
/// in — which is what makes "take the other root" a mutation gate 4 can fail on
/// rather than a coin flip.
///
/// §20's mutation 4 was "stop the inverter after one step", and it has no subject:
/// the shipped inversion is closed form and takes no steps. This is its
/// substitute, and it is checked here as a PROPERTY rather than run only as an
/// edit — the discarded branch `q/a` lands at `−2·cp(T_REF)/slope` below the
/// datum, i.e. hundreds of Kelvin below absolute zero, where the shape's capacity
/// has already gone negative.
#[test]
fn the_discarded_quadratic_root_is_not_a_reachable_temperature() {
    let (c0, s) = at_datum();
    // Roots of ½·s·y² + c₀·y − h = 0 with h from a real state.
    let h = c0 * 500.0 + 0.5 * s * 500.0 * 500.0;
    let disc = (c0 * c0 + 2.0 * s * h).sqrt();
    let other = -(c0 + disc) / s; // q/a

    // Read the taken root back THROUGH the model rather than re-deriving it
    // here. The first draft of this test computed `2·h/(c₀ + q)` by hand and
    // asserted that; it therefore defended the arithmetic and not the branch,
    // and the mutation pass found it green under the other root (§20).
    let taken = LinearCpEnthalpy
        .temperature_from_enthalpy(&shaped_slate(), &pure(), h * 3.0, 3.0)
        .expect("a real state inverts")
        .value()
        - T_REF.value();
    approx::assert_relative_eq!(taken, 500.0, max_relative = 1e-12);
    assert!(
        T_REF.value() + other < 0.0,
        "the discarded root must be unphysical, not merely different: {} K",
        T_REF.value() + other
    );
    // And it is where the declared capacity has already gone negative.
    assert!(c0 + s * other < 0.0);
}

// ---------------------------------------------------------------------------
// Gate 5 — the datum.
// ---------------------------------------------------------------------------

/// Every path integrates from `energy::T_REF`, asserted against the constant
/// explicitly rather than left moot by a zero reference.
///
/// `T_REF` is 273.15 K on purpose (`energy::T_REF`: "a zero reference would make
/// `h = cp·T` and hide any code path that forgot the datum entirely"). So the
/// discriminating statement is not `h(T_REF) == 0` alone — it is that the
/// integral from 0 K differs by `∫₀^{T_REF} cp`, which for this shape is
/// **5.7e5 J/kg**, four significant figures of a real enthalpy.
#[test]
fn every_path_integrates_from_t_ref_and_not_from_zero() {
    let slate = shaped_slate();
    let x = pure();
    for model in [
        &LinearCpEnthalpy as &dyn EnthalpyModel,
        &ConstantEnthalpy as &dyn EnthalpyModel,
    ] {
        let at_datum = model
            .specific_enthalpy(&slate, &x, T_REF)
            .expect("the datum is in range")
            .value();
        assert_eq!(
            at_datum,
            0.0,
            "{}: h(T_REF) must be exactly zero",
            model.name()
        );
    }

    let (c0, s) = at_datum();
    // What a path that forgot the datum would have booked instead.
    let from_zero = c0 * T_REF.value() - 0.5 * s * T_REF.value() * T_REF.value()
        + 0.5 * s * T_REF.value() * T_REF.value();
    let missed = c0 * T_REF.value() + 0.5 * s * T_REF.value() * T_REF.value()
        - 0.5 * s * T_REF.value() * T_REF.value();
    assert!(
        (from_zero - missed).abs() < 1e-6 && missed > 5.0e5,
        "the datum must be worth a real enthalpy, got {missed:.4e} J/kg"
    );

    // And the stock and the flux carry the same datum as the specific value —
    // one model, three entry points, no second reference state.
    let t = Kelvin(700.0);
    let h = LinearCpEnthalpy
        .specific_enthalpy(&slate, &x, t)
        .unwrap()
        .value();
    let stock = LinearCpEnthalpy
        .enthalpy_stock(&slate, &x, Kg(3.0), t)
        .unwrap();
    let flux = LinearCpEnthalpy
        .enthalpy_flux(&slate, &x, KgPerSec(3.0), t)
        .unwrap()
        .value();
    approx::assert_relative_eq!(stock, 3.0 * h, max_relative = 1e-15);
    approx::assert_relative_eq!(flux, 3.0 * h, max_relative = 1e-15);
}

// ---------------------------------------------------------------------------
// Gate 6 — `h − u = (R/M̄)·T` at TWO temperatures.
// ---------------------------------------------------------------------------

/// The thermodynamic relation that fixes `u` against `h`, on a gas, at two
/// temperatures.
///
/// **One temperature is vacuous here and that is the whole point** (§20's
/// correction to §18). A shaped `h` left with M5.3's closed-form
/// `u = cv·T − cp·T_REF` — reading `cv` and `cp` at some single anchor — agrees
/// with the relation at exactly one temperature and drifts linearly away from it
/// at every other. The second temperature is what sees the drift.
#[test]
fn enthalpy_minus_internal_energy_is_the_gas_constant_term_at_two_temperatures() {
    let slate = shaped_slate();
    let x = pure();
    let offset = R_GAS / METHANE_MOLAR_MASS;
    approx::assert_relative_eq!(
        x.gas_constant_offset(&slate).value(),
        offset,
        max_relative = 1e-12
    );
    for t in [Kelvin(310.0), Kelvin(800.0)] {
        let h = LinearCpEnthalpy
            .specific_enthalpy(&slate, &x, t)
            .unwrap()
            .value();
        let u = LinearCpEnthalpy
            .specific_internal_energy(&slate, &x, t)
            .unwrap()
            .value();
        approx::assert_relative_eq!(h - u, offset * t.value(), max_relative = 1e-12);
    }

    // The slip this gate exists for, sized so the two-temperature requirement is
    // visible rather than asserted: keep M5.3's closed form `u = cv·T − cp·T_REF`
    // and freeze its capacities at whatever value makes `u` exactly right at
    // 310 K. It then misses by a real number at 800 K.
    //
    // **The frozen capacity is the MEAN over `[T_REF, 310]`, not the spot value at
    // 310** — and getting that wrong was this test's own first draft. M5.3's form
    // is `cp·(T − T_REF) − (R/M̄)·T` rearranged, so the `cp` that reproduces `u` at
    // one temperature is the one that reproduces `h` there, which is an average
    // from the datum. A spot value misses by 2.9% at the very temperature it was
    // frozen at — the spot/mean conflation, appearing inside the gate written to
    // catch it.
    let cp_310 = LinearCpEnthalpy
        .mean_cp(&slate, &x, T_REF, Kelvin(310.0))
        .unwrap()
        .value();
    let frozen = |t: f64| (cp_310 - offset) * t - cp_310 * T_REF.value();
    let honest = |t: f64| {
        LinearCpEnthalpy
            .specific_internal_energy(&slate, &x, Kelvin(t))
            .unwrap()
            .value()
    };
    approx::assert_relative_eq!(frozen(310.0), honest(310.0), max_relative = 1e-12);
    assert!(
        (frozen(800.0) - honest(800.0)).abs() / honest(800.0).abs() > 0.20,
        "a one-temperature check must be unable to see this"
    );
}

// ---------------------------------------------------------------------------
// Gate 7 — the SPOT value, on the one site that genuinely wants one.
// ---------------------------------------------------------------------------

/// `γ = cp/cv` at two temperatures, and it must MOVE under a shape.
///
/// The spot/mean conflation is the error §20 enumerates three consumers to
/// prevent, and `γ` is where it would be invisible: a ratio at one temperature
/// answered with a mean over an interval is still a plausible number near 1.3.
/// The constant model is the control again.
#[test]
fn the_heat_capacity_ratio_moves_with_temperature_only_under_a_shape() {
    let slate = shaped_slate();
    let x = pure();
    let offset = x.gas_constant_offset(&slate).value();
    let gamma = |t: f64| {
        let cp = LinearCpEnthalpy
            .spot_cp(&slate, &x, Kelvin(t))
            .unwrap()
            .value();
        cp / (cp - offset)
    };
    let cold = gamma(300.0);
    let hot = gamma(800.0);
    assert!(
        cold > 1.2 && hot > 1.1,
        "a gas γ must stay above 1: {cold}, {hot}"
    );
    assert!(
        cold - hot > 0.10,
        "γ must fall measurably as cp rises: {cold} → {hot}"
    );

    let flat = flat_slate();
    let gamma_flat = |t: f64| {
        let cp = ConstantEnthalpy
            .spot_cp(&flat, &x, Kelvin(t))
            .unwrap()
            .value();
        cp / (cp - offset)
    };
    assert_eq!(gamma_flat(300.0).to_bits(), gamma_flat(800.0).to_bits());
}

// ---------------------------------------------------------------------------
// Fork 7 — `mean_cp` at `T₂ = T₁`.
// ---------------------------------------------------------------------------

/// An isothermal interval returns the SPOT value, not a NaN.
///
/// `0/0` is what the difference quotient gives there, and rule 5 makes that a
/// guard rather than a footnote. Exact equality, not a threshold: the state is
/// reachable — a tank sitting at ambient, a stagnant edge — and a threshold would
/// be a magic number that also flattened legitimately small intervals, which the
/// second half of this test pins by checking an interval one ULP wide still
/// behaves like an interval.
#[test]
fn an_isothermal_interval_returns_the_spot_capacity() {
    let slate = shaped_slate();
    let x = pure();
    let t = Kelvin(640.0);
    let mean = LinearCpEnthalpy.mean_cp(&slate, &x, t, t).unwrap();
    let spot = LinearCpEnthalpy.spot_cp(&slate, &x, t).unwrap();
    assert_eq!(mean.value().to_bits(), spot.value().to_bits());
    assert!(mean.value().is_finite());

    // A genuinely tiny interval is NOT flattened onto the guard — it goes through
    // the quotient — and the MEASURED consequence is worth stating rather than
    // asserting away: one ULP wide, `(h(t₂) − h(t₁))/(t₂ − t₁)` cancels to
    // **3072 J/(kg·K)** against a true 3384.6, having kept no significant digits
    // at all.
    //
    // **That is the honest cost of fork 7's exact equality, and what makes it
    // survivable is that every consumer multiplies the mean back by the same
    // interval.** The flash's `c̄p·ΔT`, the boundary balance's `c̄p·(T − T_REF)`:
    // the cancelled quotient times the width reproduces `h(t₂) − h(t₁)` to a ULP,
    // because the width is exactly what was divided out. So the asserted property
    // is the one consumers rely on, not an accuracy the number does not have —
    // and a threshold guard instead of exact equality would not have fixed this,
    // it would only have moved the meaningless band and made it larger.
    let next = Kelvin(f64::from_bits(t.value().to_bits() + 1));
    assert_ne!(next.value(), t.value());
    let narrow = LinearCpEnthalpy.mean_cp(&slate, &x, t, next).unwrap();
    assert!(narrow.value().is_finite() && narrow.value() > 0.0);
    let width = next.value() - t.value();
    let by_quotient = narrow.value() * width;
    let by_integral = LinearCpEnthalpy
        .specific_enthalpy(&slate, &x, next)
        .unwrap()
        .value()
        - LinearCpEnthalpy
            .specific_enthalpy(&slate, &x, t)
            .unwrap()
            .value();
    assert_eq!(
        by_quotient.to_bits(),
        by_integral.to_bits(),
        "the mean times its own interval must reproduce the enthalpy difference"
    );
    // And the quotient itself really has lost its digits — recorded, not hidden.
    assert!(
        (narrow.value() - spot.value()).abs() / spot.value() > 0.05,
        "this test documents a cancellation; if it has gone away, say so"
    );
}

// ---------------------------------------------------------------------------
// The bit-identity override, gated so it cannot hide a disagreement.
// ---------------------------------------------------------------------------

/// `ConstantEnthalpy` spells out five expressions it could compose out of
/// `specific_enthalpy`. This asserts the two forms are the SAME NUMBER to a few
/// ULP — so the override can only ever be holding a floating-point grouping, not
/// a different rule.
///
/// The grouping it holds is not cosmetic: regrouping `enthalpy_flux` from
/// `(ṁ·cp)·(T − T_REF)` to `ṁ·(cp·(T − T_REF))`, with no shape and no seam
/// anywhere, moves 11 of the 19 pre-M16 plants on each fidelity. That is what the
/// corpus gate would otherwise fail on, and this is what stops the fix from being
/// a place to hide.
#[test]
fn the_constant_models_hand_held_groupings_agree_with_the_composed_forms() {
    let slate = flat_slate();
    let x = pure();
    let cp = x.mixture_cp(&slate).value();
    let cv = x.mixture_cv(&slate).value();
    for t in [Kelvin(280.0), Kelvin(450.0), Kelvin(900.0)] {
        let h = ConstantEnthalpy
            .specific_enthalpy(&slate, &x, t)
            .unwrap()
            .value();
        approx::assert_relative_eq!(
            ConstantEnthalpy
                .enthalpy_flux(&slate, &x, KgPerSec(3.7), t)
                .unwrap()
                .value(),
            3.7 * h,
            max_relative = 4.0 * f64::EPSILON
        );
        approx::assert_relative_eq!(
            ConstantEnthalpy
                .enthalpy_stock(&slate, &x, Kg(3.7), t)
                .unwrap(),
            3.7 * h,
            max_relative = 4.0 * f64::EPSILON
        );
        approx::assert_relative_eq!(
            ConstantEnthalpy
                .specific_internal_energy(&slate, &x, t)
                .unwrap()
                .value(),
            h - (cp - cv) * t.value(),
            max_relative = 8.0 * f64::EPSILON
        );
        approx::assert_relative_eq!(
            ConstantEnthalpy
                .temperature_from_enthalpy(&slate, &x, 3.7 * h, 3.7)
                .unwrap()
                .value(),
            t.value(),
            max_relative = 4.0 * f64::EPSILON
        );
        approx::assert_relative_eq!(
            ConstantEnthalpy
                .temperature_from_internal_energy(
                    &slate,
                    &x,
                    3.7 * ConstantEnthalpy
                        .specific_internal_energy(&slate, &x, t)
                        .unwrap()
                        .value(),
                    3.7
                )
                .unwrap()
                .value(),
            t.value(),
            max_relative = 8.0 * f64::EPSILON
        );
        // The mix: `T_REF + Σṁh/Σṁcp` against `T(Σṁh/Σṁ)`, one stream.
        approx::assert_relative_eq!(
            ConstantEnthalpy
                .mix_temperature(
                    &slate,
                    &x,
                    InflowEnthalpy {
                        enthalpy_rate: 3.7 * h,
                        mass_rate: 3.7,
                        capacity_rate: 3.7 * cp,
                    }
                )
                .unwrap()
                .value(),
            t.value(),
            max_relative = 4.0 * f64::EPSILON
        );

        // `mean_cp` is the ONE override that must be exact rather than close.
        //
        // The other five above are asserted to a few ULP because their grouping
        // is a floating-point detail with no consumer that amplifies it. This one
        // has such a consumer: the flash fraction at `boiloff.rs` divides the
        // mean by `Δh̄_vap` and integrates the result into a tank's inventory, so
        // a few ULP become a moved snapshot. The mutation pass measured it —
        // giving this method the difference-quotient form instead moves
        // `crude_column_boiloff`, `crude_column_recovery` and
        // `crude_column_recovery_train` on BOTH fidelities and fails not one test
        // in the workspace (§20's mutation 5). The corpus baseline that caught it
        // is a file on the measurer's disk; CI commits none. So the claim is
        // asserted here, on the bits, where a test can hold it.
        //
        // The spans are deliberately not round. `(h(t₂) − h(t₁))/(t₂ − t₁)` is
        // EXACT for a tidy capacity over a tidy interval — 300 K to 700 K on a
        // flat 2220 J/(kg·K) reproduces `cp` to the bit — and the first draft of
        // this assertion used exactly that and was green under the mutation. The
        // engine's arguments are a bubble point and a tank temperature, neither
        // of them round, and there roughly two in five spans differ by an ULP.
        let spans = [
            (t, t),
            (
                Kelvin(351.097_778_228_717_97),
                Kelvin(588.593_903_789_472_2),
            ),
            (Kelvin(368.183_580_830_352_3), Kelvin(644.103_755_580_387_2)),
            (
                Kelvin(302.049_173_319_622_75),
                Kelvin(434.050_198_976_393_35),
            ),
        ];
        let mut quotient_differs = 0;
        for (t1, t2) in spans {
            assert_eq!(
                ConstantEnthalpy
                    .mean_cp(&slate, &x, t1, t2)
                    .unwrap()
                    .value()
                    .to_bits(),
                cp.to_bits(),
                "the constant model's mean capacity must BE `mixture_cp`, bit for bit"
            );
            if t2.value() != t1.value() {
                let h1 = ConstantEnthalpy
                    .specific_enthalpy(&slate, &x, t1)
                    .unwrap()
                    .value();
                let h2 = ConstantEnthalpy
                    .specific_enthalpy(&slate, &x, t2)
                    .unwrap()
                    .value();
                if ((h2 - h1) / (t2.value() - t1.value())).to_bits() != cp.to_bits() {
                    quotient_differs += 1;
                }
            }
        }
        assert!(
            quotient_differs > 0,
            "this assertion is only worth making while the two forms actually              disagree somewhere; if they no longer do, say so rather than              deleting it"
        );
    }
}

/// A mixture's shape is the mass-weighted sum of its components', and the model
/// refuses a mixture that is only partly shaped rather than dropping the
/// unshaped cut's contribution.
///
/// The refusal is a load-time one on every shipped path, so this is the
/// hand-built-slate arm — the state the loader's message says is the only way to
/// get here.
#[test]
fn a_partly_shaped_mixture_is_refused_rather_than_silently_weighted() {
    let shaped = shape();
    let slate = Slate::new(vec![
        PseudoComponent {
            name: "shaped".into(),
            tb: Kelvin(111.65),
            molar_mass: refinery_core::units::KgPerMol(0.016),
            density: None,
            cp: JPerKgK(shaped.at_datum(T_REF).0),
            cp_shape: Some(shaped),
            phase: Phase::Gas,
        },
        PseudoComponent {
            name: "flat".into(),
            tb: Kelvin(231.0),
            molar_mass: refinery_core::units::KgPerMol(0.044),
            density: None,
            cp: JPerKgK(1700.0),
            cp_shape: None,
            phase: Phase::Gas,
        },
    ])
    .expect("a two-cut slate");

    let mixed = Composition::from_weights(&[0.5, 0.5]).unwrap();
    assert!(LinearCpEnthalpy
        .specific_enthalpy(&slate, &mixed, Kelvin(500.0))
        .is_err());

    // A zero-fraction unshaped cut does NOT vote, on the same argument
    // `Composition::phase` uses: otherwise a shaped sub-plant could not share a
    // file with anything else.
    let only_shaped = Composition::pure(2, 0);
    let h = LinearCpEnthalpy
        .specific_enthalpy(&slate, &only_shaped, Kelvin(500.0))
        .expect("the unshaped cut carries no mass");
    let (c0, s) = at_datum();
    let y = 500.0 - T_REF.value();
    approx::assert_relative_eq!(h.value(), c0 * y + 0.5 * s * y * y, max_relative = 1e-12);
}
