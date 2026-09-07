//! Reference case for the M7.2 K-value correlation: the MAGNITUDE of
//! `TroutonThermo`'s vapour pressure, against a tabulated one.
//!
//! This file exists because of DESIGN §5, correction 4, which is worth restating
//! because it is the whole reason the gate is here and not beside the code. The
//! exact identities in `solvers/src/thermo.rs` — `K = 1` at `(tb, P_ATM)`, `K`
//! monotone in `T`, a heavier cut lower at fixed `T` — all hold for **any**
//! Trouton constant, because a Clausius–Clapeyron form integrates *from* the
//! boiling-point anchor. They are not weak evidence about the constant; they are
//! **structurally incapable** of being evidence about it. Only a magnitude
//! comparison can police it, so only this file can, and merging the two families
//! into one test would produce something that proves neither.
//!
//! **What this is, and its ceiling.** It is a REGRESSION LOCK on one empirical
//! constant, not a validation of the correlation. Trouton's rule is a
//! correlation fitted to data; comparing it against tabulated data for one
//! hydrocarbon tells you the constant has not been fat-fingered and the form has
//! not been algebraically mangled. It does not tell you the model is right for a
//! 600 K vacuum-residue cut, and nothing in this workspace does.
//!
//! **The source, read rather than recalled** (`published-anchor-envelope`).
//! NIST Chemistry WebBook, n-hexane (CAS 110-54-3), phase-change data, Antoine
//! equation `log₁₀(P) = A − B/(T + C)` with P in **bar** and T in **K**:
//!
//! ```text
//!   A = 4.00266   B = 1171.53   C = −48.784
//!   valid 286.18 – 342.69 K
//!   Willingham, Taylor, Pignocco & Rossini (1945), as tabulated by NIST
//! ```
//!
//! Fetched from <https://webbook.nist.gov/cgi/cbook.cgi?ID=C110543&Mask=4>. The
//! validity range is part of what was read and is respected below: this gate
//! samples only inside it, because outside it the tabulation is the thing that
//! is unsupported, not the model.
//!
//! **The boiling point comes from the same coefficients**, by solving the
//! Antoine equation for `P = 1 atm`, rather than from a second remembered
//! number. That keeps the entire comparison resting on one citation — there is
//! no recalled quantity anywhere in this file.

use refinery_core::components::{Phase, PseudoComponent, Slate};
use refinery_core::traits::ThermoModel;
use refinery_core::units::{JPerKgK, Kelvin, KgPerM3, KgPerMol, P_ATM};
use refinery_solvers::TroutonThermo;

/// The model's saturated vapour pressure [Pa], read through the public API.
///
/// `K = Psat/P`, so evaluating at `P = P_ATM` and multiplying back is exact —
/// not an approximation of a private method. Going through `k_value` rather than
/// exposing `saturation_pressure` keeps the model's state validation on the only
/// path into it (rule 5); a public vapour pressure would be the one entry that
/// skips it and could hand out an `inf`.
fn model_psat(thermo: &TroutonThermo, slate: &Slate, temperature: f64) -> f64 {
    thermo
        .k_value(slate, 0, Kelvin(temperature), P_ATM)
        .expect("a hydrocarbon at a positive temperature is a valid state")
        * P_ATM.value()
}

/// NIST Antoine coefficients for n-hexane, set 2 (see the module comment).
const ANTOINE_A: f64 = 4.00266;
const ANTOINE_B: f64 = 1171.53;
const ANTOINE_C: f64 = -48.784;
/// The tabulation's own validity range [K]. Sampling outside it would compare
/// the model against an extrapolation, not against data.
const VALID_LOW: f64 = 286.18;
const VALID_HIGH: f64 = 342.69;

/// Tabulated saturated vapour pressure of n-hexane [Pa].
fn antoine_pa(temperature: f64) -> f64 {
    // log10(P/bar) = A − B/(T + C); 1 bar = 1e5 Pa.
    10f64.powf(ANTOINE_A - ANTOINE_B / (temperature + ANTOINE_C)) * 1.0e5
}

/// n-hexane's normal boiling point [K], derived from the SAME coefficients by
/// inverting the Antoine equation at 1 atm. `log₁₀(101325/1e5) = log₁₀(1.01325)`.
fn normal_boiling_point() -> f64 {
    ANTOINE_B / (ANTOINE_A - (P_ATM.value() / 1.0e5).log10()) - ANTOINE_C
}

/// A one-cut slate standing for n-hexane. Only `tb` is read by a K-value at this
/// fidelity; the rest are placeholders and are marked as such rather than
/// carrying real hexane numbers that nothing consumes.
fn hexane_slate() -> Slate {
    Slate::new(vec![PseudoComponent {
        name: "n_hexane".into(),
        tb: Kelvin(normal_boiling_point()),
        molar_mass: KgPerMol(0.086_18),
        density: Some(KgPerM3(655.0)),
        cp: JPerKgK(2260.0),
        phase: Phase::Liquid,
    }])
    .unwrap()
}

/// Eleven evenly spaced samples across the tabulation's validity range.
fn samples() -> impl Iterator<Item = f64> {
    (0..=10).map(|i| VALID_LOW + (VALID_HIGH - VALID_LOW) * f64::from(i) / 10.0)
}

/// The envelope: Trouton's rule reproduces n-hexane's tabulated vapour pressure
/// within `[0.9, 1.2]` across the whole validity range.
///
/// The band is asymmetric because the error is: the anchor forces agreement at
/// the boiling point (top of the range) and the deviation grows monotonically as
/// the sample cools, reaching about `+10%` at 286 K. A symmetric band would
/// therefore be half slack.
#[test]
fn trouton_reproduces_the_tabulated_vapour_pressure_within_the_envelope() {
    let slate = hexane_slate();
    let thermo = TroutonThermo::new();
    for t in samples() {
        let ratio = model_psat(&thermo, &slate, t) / antoine_pa(t);
        assert!(
            (0.9..=1.2).contains(&ratio),
            "at {t:.2} K, the Trouton-to-tabulated ratio is {ratio:.4}, outside [0.9, 1.2]"
        );
    }
}

/// The envelope is FALSIFIABLE — a Trouton constant wrong by ±30% escapes it.
///
/// Without this the previous test would be unfalsifiable in the way that
/// matters: an envelope nothing is ever shown to fall outside of catches
/// nothing, and this milestone's whole justification for admitting one fitted
/// constant is that a gate polices it (DESIGN §5, correction 6).
///
/// **The escape is asserted at the COLD end specifically, and that is the
/// physics rather than a convenience.** Near `tb` the Clausius–Clapeyron anchor
/// pins `Psat` to `P_atm` whatever the constant, so at 342 K a 30%-wrong
/// constant lands at 1.007 and 0.992 — comfortably inside the band. All the
/// discriminating power lives at the cold end, where the exponent has room to
/// diverge; a gate sampling only near the boiling point would be as blind as the
/// exact identities are. Stated so a later slice does not "simplify" the range.
#[test]
fn a_wrong_trouton_constant_escapes_the_envelope() {
    let slate = hexane_slate();
    let tabulated = antoine_pa(VALID_LOW);
    for factor in [0.7, 1.3] {
        let wrong = TroutonThermo::with_trouton_constant(TroutonThermo::TROUTON_CONSTANT * factor);
        let ratio = model_psat(&wrong, &slate, VALID_LOW) / tabulated;
        assert!(
            !(0.9..=1.2).contains(&ratio),
            "a Trouton constant {factor}× the shipped one must leave the envelope at \
             the cold end of the range, but gave ratio {ratio:.4}"
        );
    }

    // And the same wrong constants are INSIDE the band near the boiling point —
    // the measurement behind this test's placement, not an incidental remark.
    for factor in [0.7, 1.3] {
        let wrong = TroutonThermo::with_trouton_constant(TroutonThermo::TROUTON_CONSTANT * factor);
        let ratio = model_psat(&wrong, &slate, VALID_HIGH) / antoine_pa(VALID_HIGH);
        assert!(
            (0.9..=1.2).contains(&ratio),
            "the anchor is expected to hide a wrong constant near tb — if this now \
             fails, the envelope has become sensitive there and the comment above \
             is stale (ratio {ratio:.4})"
        );
    }
}

/// The derived boiling point agrees with the anchor the model uses: feeding the
/// Antoine-inverted `tb` back through the tabulation returns 1 atm.
///
/// This is not circular — it checks that `normal_boiling_point()` inverts the
/// equation correctly, which is the one step in this file that could be wrong
/// without either envelope test noticing (both would simply shift together).
#[test]
fn the_derived_boiling_point_returns_one_atmosphere() {
    approx::assert_relative_eq!(
        antoine_pa(normal_boiling_point()),
        P_ATM.value(),
        max_relative = 1e-12
    );
    // And it is inside the tabulation's validity range, so the anchor itself is
    // supported by the data rather than extrapolated to.
    let tb = normal_boiling_point();
    assert!(
        (VALID_LOW..=VALID_HIGH).contains(&tb),
        "the anchor {tb:.2} K must lie inside the tabulated range"
    );
}

/// The bubble pressure of a PURE liquid IS that component's vapour pressure —
/// so `bubble_pressure` inherits this file's envelope instead of needing a
/// second one (M11, docs/DESIGN.md §13 gate 2).
///
/// **Exactness is what makes the inheritance sound.** If the two agreed only
/// approximately, this file's `[0.9, 1.2]` band would be graded against a
/// slightly different quantity than the one the engine publishes, and the
/// difference would live exactly where nobody looks. Asserted across the whole
/// validity range, at the shipped constant and at the two deliberately wrong
/// ones — the identity holds for all three, which is the same property (and the
/// same limitation) the `K = 1` anchor has.
#[test]
fn a_pure_liquids_bubble_pressure_is_its_own_vapour_pressure() {
    let slate = hexane_slate();
    let pure = refinery_core::components::Composition::pure(slate.len(), 0);
    for c in [
        TroutonThermo::TROUTON_CONSTANT,
        TroutonThermo::TROUTON_CONSTANT * 0.7,
        TroutonThermo::TROUTON_CONSTANT * 1.3,
    ] {
        let thermo = TroutonThermo::with_trouton_constant(c);
        for t in samples() {
            let bubble = thermo
                .bubble_pressure(&slate, &pure, Kelvin(t))
                .expect("a pure hydrocarbon at a positive temperature is a valid state");
            approx::assert_relative_eq!(
                bubble.value(),
                model_psat(&thermo, &slate, t),
                max_relative = 1e-15
            );
        }
    }
}

/// And therefore the envelope itself, stated on the method the engine calls
/// rather than on the one it happens to be built from.
///
/// A reader looking for "is the number the cavitation signal compares against
/// any good?" should find the answer under that name, not have to follow the
/// identity above to `k_value`.
#[test]
fn the_published_bubble_pressure_is_inside_the_tabulated_envelope() {
    let slate = hexane_slate();
    let pure = refinery_core::components::Composition::pure(slate.len(), 0);
    let thermo = TroutonThermo::new();
    for t in samples() {
        let ratio = thermo
            .bubble_pressure(&slate, &pure, Kelvin(t))
            .expect("valid state")
            .value()
            / antoine_pa(t);
        assert!(
            (0.9..=1.2).contains(&ratio),
            "at {t:.2} K the bubble pressure is {ratio:.4}× the tabulated vapour \
             pressure, outside [0.9, 1.2]"
        );
    }
}

// ---------------------------------------------------------------------------
// M13 gate 3 — `dh_vap`'s own magnitude, which nothing anchored before
// ---------------------------------------------------------------------------
//
// **Why this is a separate gate and not covered by the envelope above.**
// `TroutonThermo` reads `trouton_constant` in two independent methods:
// `saturation_pressure` buries it in an exponent, and `dh_vap` returns `C·tb`
// directly. The tests above police the first path only, so a `dh_vap` that
// returned half of Trouton's rule while `k_value` stayed correct escapes every
// one of them — and it escapes M13's own boundary-energy gate too, because the
// flash would then boil twice as much mass at half the latent heat per kilogram
// and the books would close to the last digit (docs/DESIGN.md §15 gate 3).
// Without this file's next two tests, that gate is a consistency check wearing
// a physics label.
//
// **The anchor is the SLOPE of the same citation, not a second one.** The
// module comment's Antoine coefficients are a fit to measured vapour pressures;
// Clausius–Clapeyron turns that fit's slope into a latent heat with no new
// data:
//
// ```text
//   ln P = ln(10)·(A − B/(T + C))          d(ln P)/dT = ln(10)·B/(T + C)²
//   d(ln P)/dT = Δh_vap /(R·T²)            ⇒  Δh_vap = R·T²·ln(10)·B/(T + C)²
// ```
//
// That keeps the whole file resting on one citation, exactly as
// `normal_boiling_point` does — no recalled kJ/mol figure appears anywhere
// here, which is the `published-anchor-envelope` rule.

/// Universal gas constant [J/(mol·K)], CODATA exact since the 2019 SI.
const R_GAS: f64 = 8.314_462_618_153_24;

/// n-hexane's heat of vaporisation at its normal boiling point [J/mol],
/// from the Antoine fit by Clausius–Clapeyron.
fn antoine_dh_vap(temperature: f64) -> f64 {
    R_GAS * temperature * temperature * std::f64::consts::LN_10 * ANTOINE_B
        / (temperature + ANTOINE_C).powi(2)
}

/// The symbolic slope above is checked against a numerical derivative of
/// `antoine_pa` itself.
///
/// **Two paths to one number, because the factor available to slip here is
/// silent.** `antoine_pa` works in bar and base-10 logs; the differentiation
/// introduces `ln 10` and a `(T + C)²`, and getting either wrong produces a
/// latent heat that is smooth, positive, and off by a constant — which the
/// envelope below would then absorb or reject for the wrong reason. A central
/// difference of the function the file already trusts has none of that
/// algebra in it.
#[test]
fn the_antoine_slope_agrees_with_a_numerical_derivative() {
    let tb = normal_boiling_point();
    let h = 1.0e-4;
    let numerical = (antoine_pa(tb + h).ln() - antoine_pa(tb - h).ln()) / (2.0 * h);
    let symbolic = std::f64::consts::LN_10 * ANTOINE_B / (tb + ANTOINE_C).powi(2);
    approx::assert_relative_eq!(symbolic, numerical, max_relative = 1e-8);
}

/// The envelope: Trouton's rule reproduces the Antoine fit's own latent heat at
/// the normal boiling point to within `[0.85, 1.15]`.
///
/// **Where the band comes from, stated rather than fitted.** Two known errors
/// sit between the two sides, and both are one-sided:
///
///   * The anchor is the IDEAL-GAS Clausius–Clapeyron form — it drops the
///     vapour's non-ideality and the liquid's molar volume, which at a
///     hydrocarbon's normal boiling point makes it read a few percent HIGH.
///     So the model is expected to sit slightly below 1.
///   * Trouton's rule is a one-parameter correlation across all
///     non-associating liquids, quoted as good to about ten percent.
///
/// ±15% covers both with room, and it is the band, not the measurement, that
/// had to be decided in advance: the measured ratio is **0.9859**, which is
/// where a correlation that is 1.4% under an anchor biased high should land.
/// A tighter band would be fitted to n-hexane; a looser one could not fail.
#[test]
fn trouton_reproduces_the_tabulated_latent_heat_within_the_envelope() {
    let slate = hexane_slate();
    let tb = normal_boiling_point();
    let model = TroutonThermo::new()
        .dh_vap(&slate, 0, Kelvin(tb))
        .expect("a hydrocarbon at its boiling point is a valid state")
        .value();
    let ratio = model / antoine_dh_vap(tb);
    assert!(
        (0.85..=1.15).contains(&ratio),
        "Trouton's rule gives n-hexane {model:.1} J/mol at its normal boiling point \
         ({tb:.2} K) against the Antoine fit's own {:.1} J/mol — a ratio of {ratio:.4}, \
         outside [0.85, 1.15]",
        antoine_dh_vap(tb)
    );
}

/// The envelope is FALSIFIABLE, and the mutation it is sized against is named.
///
/// `docs/DESIGN.md` §15's mutation table has one entry predicted caught by this
/// gate ALONE — `dh_vap` scaled by ½ — because M13's boundary-energy balance
/// closes to the last digit under it: the flash boils twice the mass at half
/// the latent heat per kilogram and every term in the books moves together.
/// This asserts that the ½ really does escape, and that a ±20% error does too,
/// so the band is not merely wide enough to admit the truth.
#[test]
fn a_wrong_latent_heat_escapes_the_envelope() {
    let slate = hexane_slate();
    let tb = normal_boiling_point();
    let anchor = antoine_dh_vap(tb);
    for factor in [0.5, 0.8, 1.2, 2.0] {
        let ratio = TroutonThermo::with_trouton_constant(TroutonThermo::TROUTON_CONSTANT * factor)
            .dh_vap(&slate, 0, Kelvin(tb))
            .expect("valid state")
            .value()
            / anchor;
        assert!(
            !(0.85..=1.15).contains(&ratio),
            "a Trouton constant scaled by {factor} still lands at {ratio:.4}× the \
             tabulated latent heat, inside the envelope — so the band cannot catch the \
             mutation §15 says only it can catch"
        );
    }
}
