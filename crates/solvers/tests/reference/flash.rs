//! Reference case for the M7.2 single-stage isothermal flash.
//!
//! The expected values here come from solving Rachford–Rice **by hand**, on
//! paper, for a binary chosen so the algebra closes in exact fractions. That is
//! the "analytic solution derived independently of the code" this directory is
//! for — no published table is involved, and none is needed, which is what
//! DESIGN §5 means by "the strongest gates here are derivable from first
//! principles".
//!
//! The K-values are **supplied by the test** through `ConstantAlphaThermo`, so
//! nothing in this file depends on the Trouton correlation being right. That
//! separation is the point: `tests/reference/vapour_pressure.rs` polices the
//! correlation, and this file polices the flash algebra, and a single test doing
//! both would police neither (DESIGN §5, "three families").

use refinery_core::components::{Phase, PseudoComponent, Slate};
use refinery_core::units::{JPerKgK, Kelvin, KgPerM3, KgPerMol, Pascal, P_ATM};
use refinery_solvers::{flash_isothermal, ConstantAlphaThermo, FlashResult, MoleFractions};

/// A slate carrying nothing the flash reads. `ConstantAlphaThermo` supplies the
/// K-values, so `tb` is inert here — deliberately, so that a change to the
/// correlation cannot move this file's numbers.
fn slate(n: usize) -> Slate {
    Slate::new(
        (0..n)
            .map(|i| PseudoComponent {
                name: format!("cut{i}"),
                tb: Kelvin(300.0 + 50.0 * i as f64),
                molar_mass: KgPerMol(0.1),
                density: Some(KgPerM3(800.0)),
                cp: JPerKgK(2000.0),
                phase: Phase::Liquid,
            })
            .collect(),
    )
    .unwrap()
}

/// The state is inert too: `ConstantAlphaThermo` ignores `(T, P)`, which is what
/// lets the hand calc be about Rachford–Rice and nothing else.
const T: Kelvin = Kelvin(400.0);
const P: Pascal = P_ATM;

fn flash(z: &[f64], k: &[f64]) -> FlashResult {
    let s = slate(z.len());
    let thermo = ConstantAlphaThermo::new(&s, k.to_vec()).unwrap();
    flash_isothermal(&MoleFractions::from_amounts(z).unwrap(), &s, &thermo, T, P)
        .expect("a valid two-phase state must flash")
}

/// THE HAND CALCULATION. Equimolar binary, `K = [4, ¼]`:
///
/// ```text
///   f(β) = 0.5·3/(1+3β) + 0.5·(−0.75)/(1−0.75β) = 0
///   ⇒ 1.5·(1 − 0.75β) = 0.375·(1 + 3β)
///   ⇒ 1.5 − 1.125β = 0.375 + 1.125β
///   ⇒ β = 1.125/2.25 = 1/2
///
///   x₀ = 0.5/(1 + 3·½)      = 0.5/2.5   = 0.2
///   x₁ = 0.5/(1 − 0.75·½)   = 0.5/0.625 = 0.8
///   y₀ = 4·0.2  = 0.8       y₁ = ¼·0.8  = 0.2
/// ```
///
/// Every number is exact, so the tolerance is machine precision rather than a
/// judgement call — and the mirror symmetry (`x` is `y` reversed, because
/// `K₀ = 1/K₁` on an equimolar feed) is a structural property of this case that
/// an off-by-one in the objective would break.
///
/// THE MUTATION THIS EXISTS FOR: the classical slip is writing the objective as
/// `Σ z(K−1)/(1 + β(K−1))` with the sign of `(K−1)` dropped somewhere, or
/// bisecting on `Σy − 1` instead. Both still return a β in `[0,1]` and
/// still normalize to compositions summing to 1 — the invariant tests beside
/// this one stay green. Only the value is wrong, and only this assertion sees it.
#[test]
fn an_equimolar_binary_flashes_to_the_hand_computed_split() {
    let r = flash(&[0.5, 0.5], &[4.0, 0.25]);

    approx::assert_relative_eq!(r.vapour_fraction, 0.5, max_relative = 1e-12);
    approx::assert_relative_eq!(r.liquid.fractions()[0], 0.2, max_relative = 1e-12);
    approx::assert_relative_eq!(r.liquid.fractions()[1], 0.8, max_relative = 1e-12);
    approx::assert_relative_eq!(r.vapour.fractions()[0], 0.8, max_relative = 1e-12);
    approx::assert_relative_eq!(r.vapour.fractions()[1], 0.2, max_relative = 1e-12);
}

/// A feed BELOW its bubble point stays wholly liquid: `Σ z·K < 1`, so `β = 0`
/// and `x = z` exactly.
///
/// The vapour reported at `β = 0` is the **incipient bubble** — normalized
/// `K·z` — which is a real quantity and not a placeholder: `K = [0.5, 0.2]`,
/// `z = [0.5, 0.5]` gives `K·z = [0.25, 0.10]`, summing to 0.35, so the first
/// bubble is `[5/7, 2/7]`. It is richer in the lighter cut than the liquid it
/// came from, which is the whole reason a column works.
#[test]
fn a_subcooled_feed_stays_liquid_and_reports_its_incipient_bubble() {
    let r = flash(&[0.5, 0.5], &[0.5, 0.2]);

    assert_eq!(r.vapour_fraction, 0.0);
    approx::assert_relative_eq!(r.liquid.fractions()[0], 0.5, max_relative = 1e-12);
    approx::assert_relative_eq!(r.liquid.fractions()[1], 0.5, max_relative = 1e-12);
    approx::assert_relative_eq!(r.vapour.fractions()[0], 5.0 / 7.0, max_relative = 1e-12);
    approx::assert_relative_eq!(r.vapour.fractions()[1], 2.0 / 7.0, max_relative = 1e-12);
}

/// A feed ABOVE its dew point is wholly vapour: `Σ z/K < 1`, so `β = 1` and
/// `y = z` exactly, with the liquid reporting the incipient dew.
///
/// `K = [4, 2]`, `z = [0.5, 0.5]`: `z/K = [0.125, 0.25]`, summing to 0.375, so
/// the first drop is `[1/3, 2/3]` — richer in the heavier cut, the mirror of the
/// bubble case above.
#[test]
fn a_superheated_feed_stays_vapour_and_reports_its_incipient_dew() {
    let r = flash(&[0.5, 0.5], &[4.0, 2.0]);

    assert_eq!(r.vapour_fraction, 1.0);
    approx::assert_relative_eq!(r.vapour.fractions()[0], 0.5, max_relative = 1e-12);
    approx::assert_relative_eq!(r.vapour.fractions()[1], 0.5, max_relative = 1e-12);
    approx::assert_relative_eq!(r.liquid.fractions()[0], 1.0 / 3.0, max_relative = 1e-12);
    approx::assert_relative_eq!(r.liquid.fractions()[1], 2.0 / 3.0, max_relative = 1e-12);
}

/// THE NULL GATE: with every `K = 1` there is no separation, at any feed.
///
/// This is the one-stage form of the `α = 1` gate DESIGN §5 puts on the cascade,
/// and it is worth having here because it fails LOUDLY under a family of bugs
/// the hand calc can miss — anything that makes the split depend on component
/// order, or that treats `K − 1 = 0` as a division rather than a zero.
///
/// `β` is genuinely indeterminate here (`f(β) ≡ 0`: the feed is at its bubble
/// point and its dew point simultaneously), so the ZERO is a documented
/// convention, not a physical result. The compositions are not a convention —
/// they are `z` for any β — and they are what this asserts hardest.
#[test]
fn unit_k_values_separate_nothing_and_pin_beta_by_convention() {
    for z in [
        [0.5, 0.5].as_slice(),
        [0.1, 0.9].as_slice(),
        [0.25, 0.25, 0.5].as_slice(),
    ] {
        let k = vec![1.0; z.len()];
        let r = flash(z, &k);
        assert_eq!(
            r.vapour_fraction, 0.0,
            "with no separation the vapour fraction is indeterminate; this \
             workspace's convention is 0"
        );
        for (c, zc) in z.iter().enumerate() {
            approx::assert_relative_eq!(r.liquid.fractions()[c], zc, max_relative = 1e-12);
            approx::assert_relative_eq!(r.vapour.fractions()[c], zc, max_relative = 1e-12);
        }
    }
}

/// The overall mole balance closes per component: `z = (1−β)·x + β·y`.
///
/// Rachford–Rice is *derived* from this, so a correct implementation satisfies
/// it identically — which is exactly why it is stated as its own gate on a
/// three-component feed with a wide K spread. It is the check that would survive
/// a change of algorithm, and the one a future warm-started or Newton-based
/// flash must still pass.
#[test]
fn the_overall_mole_balance_closes_per_component() {
    let z = [0.2, 0.3, 0.5];
    let r = flash(&z, &[10.0, 1.0, 0.01]);
    let beta = r.vapour_fraction;
    assert!(
        beta > 0.05 && beta < 0.95,
        "this feed must be genuinely two-phase for the balance to be a real \
         check, got β = {beta}"
    );
    for (c, zc) in z.iter().enumerate() {
        approx::assert_relative_eq!(
            (1.0 - beta) * r.liquid.fractions()[c] + beta * r.vapour.fractions()[c],
            zc,
            max_relative = 1e-10
        );
    }
}
