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
