//! Isothermal flash on one stage — Rachford–Rice (M7.2).
//!
//! Given a feed, a temperature and a pressure, split it into equilibrium liquid
//! and vapour. This is the single equilibrium stage that M7.3 stacks `N` times;
//! DESIGN §5 names "a single stage must reproduce a hand-computed Rachford–Rice
//! flash" as one of the cascade's own gates, which only exists if the one-stage
//! case is built and pinned first.
//!
//! Everything here is **molar**. The mass boundary is `molar::MoleFractions`,
//! one layer out.

use refinery_core::components::Slate;
use refinery_core::error::SimError;
use refinery_core::traits::ThermoModel;
use refinery_core::units::{Kelvin, Pascal};

use crate::molar::MoleFractions;

/// One stage's equilibrium split.
#[derive(Debug, Clone)]
pub struct FlashResult {
    /// Molar vapour fraction `β = V/F`, in `[0, 1]`.
    ///
    /// MOLAR, not mass — the mass vapour fraction differs whenever the phases
    /// have different mean molar masses, which is always. Nothing in M7.2
    /// converts it; M7.3's internal flows are molar throughout and the
    /// conversion happens at the draws.
    pub vapour_fraction: f64,
    /// Equilibrium liquid mole fractions `x`.
    ///
    /// At `β = 1` there is no liquid, and this is the composition of the
    /// incipient dew — the first drop. It falls out of the same formula rather
    /// than being a special case, which is why there is no `Option` here.
    pub liquid: MoleFractions,
    /// Equilibrium vapour mole fractions `y = K·x`. At `β = 0` this is the
    /// incipient bubble; see `liquid`.
    pub vapour: MoleFractions,
}

/// Halvings of `[0, 1]`. `2⁻⁶⁰ ≈ 9e-19` is below the spacing of an `f64` in that
/// interval, so the bracket collapses to adjacent floats and further iterations
/// are no-ops — which is what makes this loop a fixed cost with no convergence
/// arm to fail. Bisection rather than Newton on purpose: the bracket is
/// guaranteed (below), so robustness is free and there is no divergence case to
/// diagnose.
const BISECTION_STEPS: u32 = 60;

/// Isothermal flash of `feed` at `(temperature, pressure)`.
///
/// Rachford–Rice: with `β` the molar vapour fraction, mole balance and
/// equilibrium (`y_c = K_c·x_c`) give
///
/// ```text
///   x_c = z_c / (1 + β(K_c − 1))
///   y_c = K_c·z_c / (1 + β(K_c − 1))
///   f(β) = Σ_c (y_c − x_c) = Σ_c z_c(K_c − 1) / (1 + β(K_c − 1)) = 0
/// ```
///
/// Rachford & Rice, *Trans. AIME* 195 (1952); the objective is the `Σy − Σx`
/// form rather than `Σy = 1`, because it is the one that is monotone.
///
/// **Why bisection needs no safeguards here.** `f'(β) = −Σ z_c(K_c−1)²/(…)² ≤ 0`,
/// so `f` is non-increasing; and every denominator `1 + β(K_c−1)` is linear in
/// `β` between `1` (at `β = 0`) and `K_c > 0` (at `β = 1`), so on `[0, 1]` it
/// stays strictly positive and `f` is smooth. A root in `(0, 1)` therefore
/// exists iff `f(0) > 0 > f(1)`, which is exactly the two-phase test below.
///
/// **The degenerate case is decided, not discovered.** If every `K_c = 1` then
/// `f(β) ≡ 0` and `β` is genuinely indeterminate — the feed is at its bubble
/// point and its dew point at once. `f(0) ≤ 0` catches it first, so this returns
/// `β = 0`, with `x = y = z`. The compositions are right for any `β`; only the
/// number is a convention, and it is a deterministic one.
///
/// # Errors
/// `SimError` if the thermo model cannot supply a K-value at this state (the
/// `constant` fidelity never can), if it supplies one that is not finite and
/// positive, or if the feed does not match the slate.
pub fn flash_isothermal(
    feed: &MoleFractions,
    slate: &Slate,
    thermo: &dyn ThermoModel,
    temperature: Kelvin,
    pressure: Pascal,
) -> Result<FlashResult, SimError> {
    if feed.len() != slate.len() {
        return Err(SimError::Numerical(format!(
            "flash feed has {} mole fractions for a {}-component slate",
            feed.len(),
            slate.len()
        )));
    }
    let z = feed.fractions();
    let mut k = Vec::with_capacity(slate.len());
    for c in 0..slate.len() {
        let kc = thermo.k_value(slate, c, temperature, pressure)?;
        // The K vector is where a bad number can actually enter, and it comes
        // from a trait impl that may live in another crate — which M7.1's
        // correction 3 is exactly about not trusting. A non-positive K makes
        // `Σ z/K` infinite and a non-finite one poisons the objective; both
        // would otherwise surface as a meaningless β rather than as this.
        if !kc.is_finite() || kc <= 0.0 {
            return Err(SimError::NonFiniteState {
                location: format!(
                    "thermo model '{}' returned K = {kc} for '{}' at {} K, {} Pa; a \
                     K-value must be finite and > 0",
                    thermo.name(),
                    slate.get(c).name,
                    temperature.value(),
                    pressure.value()
                ),
            });
        }
        k.push(kc);
    }

    // f(β) = Σ z(K−1)/(1 + β(K−1)), non-increasing on [0, 1].
    let objective = |beta: f64| -> f64 {
        z.iter()
            .zip(&k)
            .map(|(zc, kc)| zc * (kc - 1.0) / (1.0 + beta * (kc - 1.0)))
            .sum()
    };

    // f(0) = Σ z·K − 1 (bubble-point test) and f(1) = 1 − Σ z/K (dew-point test).
    let at_bubble = objective(0.0);
    let at_dew = objective(1.0);
    let beta = if at_bubble <= 0.0 {
        // Below the bubble point: all liquid. Also the all-K = 1 case.
        0.0
    } else if at_dew >= 0.0 {
        // Above the dew point: all vapour.
        1.0
    } else {
        let (mut lo, mut hi) = (0.0f64, 1.0f64);
        for _ in 0..BISECTION_STEPS {
            let mid = 0.5 * (lo + hi);
            if objective(mid) > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    };

    let x: Vec<f64> = z
        .iter()
        .zip(&k)
        .map(|(zc, kc)| zc / (1.0 + beta * (kc - 1.0)))
        .collect();
    let y: Vec<f64> = x.iter().zip(&k).map(|(xc, kc)| xc * kc).collect();

    // Both are normalized on construction. At a root Σx and Σy are already 1;
    // at a clamped β (all liquid, all vapour) the OTHER phase's raw sum is not,
    // and normalizing is what makes it the incipient bubble or dew rather than
    // an unnormalized vector — the same defensive normalization the cut-point
    // splitter applies to its splits.
    let liquid = MoleFractions::from_amounts(&x)?;
    let vapour = MoleFractions::from_amounts(&y)?;

    // No `beta.is_finite()` check here, deliberately: `beta` is 0.0, 1.0, or a
    // midpoint of `[0, 1]`, so there is no path on which it is not finite. A
    // guard nothing can reach reads as coverage it is not
    // (`a-command-can-be-a-no-op`). The reachable failure is a bad K, guarded
    // above, and a degenerate phase vector, which `from_amounts` refuses with a
    // better message than this function could write.

    Ok(FlashResult {
        vapour_fraction: beta,
        liquid,
        vapour,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ConstantAlphaThermo, ConstantThermo};
    use refinery_core::components::{Phase, PseudoComponent, Slate};
    use refinery_core::units::{JPerKgK, KgPerM3, KgPerMol, P_ATM};

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

    fn flash(z: &[f64], k: &[f64]) -> FlashResult {
        let s = slate(z.len());
        let thermo = ConstantAlphaThermo::new(&s, k.to_vec()).unwrap();
        flash_isothermal(
            &MoleFractions::from_amounts(z).unwrap(),
            &s,
            &thermo,
            Kelvin(400.0),
            P_ATM,
        )
        .unwrap()
    }

    /// Both phase compositions sum to 1 for every regime — two-phase, all
    /// liquid, all vapour, and a wide K spread. The invariant the caller relies
    /// on to hand either phase back across the mass boundary.
    #[test]
    fn both_phases_are_normalized_in_every_regime() {
        let cases: [(&[f64], &[f64]); 5] = [
            (&[0.5, 0.5], &[4.0, 0.25]),
            (&[0.5, 0.5], &[0.5, 0.2]),
            (&[0.5, 0.5], &[4.0, 2.0]),
            (&[0.2, 0.3, 0.5], &[10.0, 1.0, 0.01]),
            (&[0.1, 0.9], &[1.0, 1.0]),
        ];
        for (z, k) in cases {
            let r = flash(z, k);
            let sx: f64 = r.liquid.fractions().iter().sum();
            let sy: f64 = r.vapour.fractions().iter().sum();
            approx::assert_relative_eq!(sx, 1.0, max_relative = 1e-12);
            approx::assert_relative_eq!(sy, 1.0, max_relative = 1e-12);
            assert!(
                (0.0..=1.0).contains(&r.vapour_fraction),
                "β must be a fraction, got {} for K = {k:?}",
                r.vapour_fraction
            );
        }
    }

    /// `y_c = K_c · x_c` for every component — the equilibrium relation itself,
    /// asserted on the OUTPUT rather than assumed from the derivation. It
    /// survives the defensive normalization only because both phases are scaled
    /// by the same factor at a true root.
    #[test]
    fn the_output_satisfies_the_equilibrium_relation() {
        let k = [3.0, 1.0, 0.2];
        let r = flash(&[0.3, 0.3, 0.4], &k);
        for (c, kc) in k.iter().enumerate() {
            approx::assert_relative_eq!(
                r.vapour.fractions()[c],
                kc * r.liquid.fractions()[c],
                max_relative = 1e-10
            );
        }
    }

    /// A thermo model that cannot supply a K-value fails the flash rather than
    /// letting it proceed on a default. `ConstantThermo` is the fidelity every
    /// scenario in the repo selects, so this is the path a mis-paired plant
    /// takes until M7.3's loader refuses the pairing outright.
    #[test]
    fn a_thermo_that_has_no_k_value_fails_the_flash() {
        let s = slate(2);
        let result = flash_isothermal(
            &MoleFractions::from_amounts(&[0.5, 0.5]).unwrap(),
            &s,
            &ConstantThermo,
            Kelvin(400.0),
            P_ATM,
        );
        assert!(
            result.is_err(),
            "a flash on a fidelity with no phase equilibrium must fail"
        );
    }

    /// A thermo model returning a nonsense K is refused, naming the model and
    /// the component.
    ///
    /// The guard exists because `ThermoModel` is a trait and an implementation
    /// can live in another crate — M7.1's correction 3 is exactly about not
    /// trusting a contract kept elsewhere. Neither model in this workspace can
    /// violate it (`TroutonThermo` checks, `ConstantAlphaThermo` validates at
    /// construction), so without a deliberately broken stub the guard would be
    /// unreachable by every test in the repo, which is the shape
    /// `a-counter-is-not-a-gate` records.
    #[test]
    fn a_thermo_returning_a_nonsense_k_is_refused_by_name() {
        struct BrokenThermo(f64);
        impl ThermoModel for BrokenThermo {
            fn name(&self) -> &'static str {
                "broken-stub"
            }
            fn k_value(
                &self,
                _slate: &Slate,
                _component: usize,
                _temperature: Kelvin,
                _pressure: Pascal,
            ) -> Result<f64, SimError> {
                Ok(self.0)
            }
        }

        let s = slate(2);
        for bad in [f64::NAN, f64::INFINITY, 0.0, -1.0] {
            let message = flash_isothermal(
                &MoleFractions::from_amounts(&[0.5, 0.5]).unwrap(),
                &s,
                &BrokenThermo(bad),
                Kelvin(400.0),
                P_ATM,
            )
            .expect_err("a K of {bad} must not be flashed on")
            .to_string();
            assert!(
                message.contains("broken-stub") && message.contains("cut0"),
                "the refusal must name the model and the component, got: {message}"
            );
        }
    }

    /// A feed whose length does not match the slate is refused. Cheap, and it is
    /// the mismatch a cascade built from a per-stage vector can actually make.
    #[test]
    fn a_feed_of_the_wrong_length_is_refused() {
        let s = slate(3);
        let thermo = ConstantAlphaThermo::new(&s, vec![2.0, 1.0, 0.5]).unwrap();
        assert!(flash_isothermal(
            &MoleFractions::from_amounts(&[0.5, 0.5]).unwrap(),
            &s,
            &thermo,
            Kelvin(400.0),
            P_ATM,
        )
        .is_err());
    }
}
