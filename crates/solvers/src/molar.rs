//! Mole fractions — the basis a separation is actually computed in (M7.2).
//!
//! Everything in this workspace is mass: SI mass flows, `Composition` mass
//! fractions, mass balances. Vapour–liquid equilibrium is **molar**: `K = y/x`
//! is a ratio of mole fractions, and a Rachford–Rice flash is a mole balance.
//! So there is a conversion, and DESIGN §5 fork 1 says where it runs — at the
//! unit's boundary, twice, feed in and draws out.
//!
//! **This is a separate type rather than a second `Composition`, and that is
//! structural rather than tidy.** A `Composition` is handed to
//! `mixture_density`, `mixture_cp` and `density_at`, all of which would accept a
//! mole-fraction vector and return a finite, deterministic, plausible, wrong
//! number — the failure shape fork 1 spends a page on. Handing mole fractions to
//! a mass-basis property function is not a mistake to remember here; it does not
//! typecheck.
//!
//! Kept in `solvers` for the same reason: `core` has no business knowing that a
//! molar basis exists, because nothing on the plant graph is molar.

use refinery_core::components::{Composition, Slate};
use refinery_core::error::SimError;
use refinery_core::units::KgPerMol;

/// Mole fractions over the slate, in slate order. Invariant: non-negative and
/// summing to 1 — every constructor normalizes.
#[derive(Debug, Clone, PartialEq)]
pub struct MoleFractions {
    fractions: Vec<f64>,
}

impl MoleFractions {
    /// Normalizes the given molar amounts (or any quantity proportional to
    /// them). Errors on negative, non-finite, or all-zero input.
    pub fn from_amounts(amounts: &[f64]) -> Result<Self, SimError> {
        if amounts.iter().any(|n| !n.is_finite() || *n < 0.0) {
            return Err(SimError::Numerical(
                "molar amounts must be finite and >= 0".into(),
            ));
        }
        let total: f64 = amounts.iter().sum();
        if total <= 0.0 {
            return Err(SimError::Numerical("molar amounts sum to zero".into()));
        }
        Ok(Self {
            fractions: amounts.iter().map(|n| n / total).collect(),
        })
    }

    /// Mass fractions → mole fractions: `n_c ∝ w_c / M_c`.
    ///
    /// **Divide by the molar mass.** The slip is multiplying, and it is worth
    /// naming because it is invisible to the obvious test: `w·M` in both
    /// directions round-trips exactly, so closure proves nothing here. What
    /// catches it is a hand-computed fraction on a mixture whose molar masses
    /// differ — see `mass_to_mole_divides_by_the_molar_mass`, where the two
    /// rules produce exactly each other's answer reversed.
    ///
    /// That is measured, not assumed. Inverting the rule in `from_mass` ALONE is
    /// caught by the round trip; inverting it in **both** directions leaves the
    /// round trip green and fails only the hand calc. The pair of mutations was
    /// run, and this is the one that distinguishes them.
    ///
    /// The reciprocal weighting is the same rule `Composition::mean_molar_mass`
    /// already uses (`1/M̄ = Σ w_c/M_c`), for the same reason: moles are what add.
    ///
    /// # Errors
    /// `SimError` if a component carries a non-positive or non-finite molar
    /// mass, or if no component has a positive fraction.
    pub fn from_mass(composition: &Composition, slate: &Slate) -> Result<Self, SimError> {
        let mut amounts = Vec::with_capacity(slate.len());
        for (c, &w) in composition.fractions().iter().enumerate() {
            let molar_mass = slate.get(c).molar_mass.value();
            if !molar_mass.is_finite() || molar_mass <= 0.0 {
                return Err(SimError::Numerical(format!(
                    "component '{}' has a non-positive or non-finite molar mass ({molar_mass} \
                     kg/mol), so its mole fraction is undefined",
                    slate.get(c).name
                )));
            }
            amounts.push(w / molar_mass);
        }
        Self::from_amounts(&amounts)
    }

    /// Mole fractions → mass fractions: `w_c ∝ n_c · M_c`. The inverse of
    /// `from_mass`, and the boundary a draw crosses on its way back to the sweep.
    ///
    /// # Errors
    /// `SimError` if the resulting weights are not a valid `Composition`.
    pub fn to_mass(&self, slate: &Slate) -> Result<Composition, SimError> {
        let weights: Vec<f64> = self
            .fractions
            .iter()
            .enumerate()
            .map(|(c, n)| n * slate.get(c).molar_mass.value())
            .collect();
        Composition::from_weights(&weights)
    }

    /// Mean molar mass on a MOLAR basis: `M̄ = Σ x_c·M_c` [kg/mol].
    ///
    /// The arithmetic mean weighted by mole fraction — where
    /// `Composition::mean_molar_mass` is the *reciprocal* mean weighted by mass
    /// fraction (`1/M̄ = Σ w_c/M_c`). Same physical number for the same mixture;
    /// the formulas differ only because the weightings do, and
    /// `the_two_mean_molar_masses_agree` pins that they do not drift apart.
    ///
    /// This is the factor the cascade converts a **mass** draw ratio through to
    /// get a molar draw rate (DESIGN §5, correction 1), so it is the one place
    /// where a wrong mean would move a flow rather than a fraction.
    pub fn mean_molar_mass(&self, slate: &Slate) -> KgPerMol {
        KgPerMol(
            self.fractions
                .iter()
                .enumerate()
                .map(|(c, n)| n * slate.get(c).molar_mass.value())
                .sum(),
        )
    }

    pub fn fractions(&self) -> &[f64] {
        &self.fractions
    }

    pub fn len(&self) -> usize {
        self.fractions.len()
    }

    pub fn is_empty(&self) -> bool {
        self.fractions.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use refinery_core::components::{Phase, PseudoComponent, Slate};
    use refinery_core::units::{JPerKgK, Kelvin, KgPerM3, KgPerMol};

    /// Methane and propane — molar masses 2.75× apart, the same mixture
    /// `core::components` uses to discriminate the reciprocal mean-molar-mass
    /// rule from the naive one. A slate whose components had similar molar
    /// masses could not tell the right conversion from the wrong one.
    fn two_gases() -> Slate {
        Slate::new(
            [("methane", 0.016), ("propane", 0.044)]
                .iter()
                .map(|(name, m)| PseudoComponent {
                    name: (*name).into(),
                    tb: Kelvin(120.0),
                    molar_mass: KgPerMol(*m),
                    density: None,
                    cp: JPerKgK(2000.0),
                    phase: Phase::Gas,
                })
                .collect(),
        )
        .unwrap()
    }

    fn liquid_slate() -> Slate {
        Slate::new(
            [("light", 0.100), ("heavy", 0.400)]
                .iter()
                .map(|(name, m)| PseudoComponent {
                    name: (*name).into(),
                    tb: Kelvin(400.0),
                    molar_mass: KgPerMol(*m),
                    density: Some(KgPerM3(800.0)),
                    cp: JPerKgK(2000.0),
                    phase: Phase::Liquid,
                })
                .collect(),
        )
        .unwrap()
    }

    /// `n_c ∝ w_c / M_c`, against a hand calculation — the gate the round trip
    /// below **cannot** be.
    ///
    /// A 50/50 mass mixture makes the arithmetic exact and the discrimination
    /// total: `x_methane = M_propane/(M_methane + M_propane) = 0.044/0.060 =
    /// 11/15`, while multiplying instead of dividing gives `0.016/0.060 = 4/15`
    /// — *precisely the other component's answer*. The two rules do not merely
    /// differ here; they swap the mixture end for end, and both sum to 1, and
    /// both round-trip. Only this assertion separates them.
    #[test]
    fn mass_to_mole_divides_by_the_molar_mass() {
        let slate = two_gases();
        let half = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let moles = MoleFractions::from_mass(&half, &slate).unwrap();

        approx::assert_relative_eq!(moles.fractions()[0], 11.0 / 15.0, max_relative = 1e-12);
        approx::assert_relative_eq!(moles.fractions()[1], 4.0 / 15.0, max_relative = 1e-12);

        // The lighter component is the more numerous one per kilogram. Stated as
        // its own assertion because it is the sanity a reader checks first, and
        // it is the half of the claim that survives a change of numbers.
        assert!(
            moles.fractions()[0] > half.fractions()[0],
            "per kilogram there are MORE moles of the lighter cut, so its mole \
             fraction must exceed its mass fraction"
        );
    }

    /// mass → mole → mass returns the original composition.
    ///
    /// **This is necessary and it is not sufficient**, which is why it sits
    /// below the hand calc rather than instead of it: `n ∝ w·M` in both
    /// directions closes exactly too. It catches a conversion applied in only
    /// one direction, or a normalization that loses a component — not an
    /// inverted rule. Both mutations were run: one-sided fails here, two-sided
    /// passes here and fails the hand calc above.
    #[test]
    fn mass_to_mole_to_mass_is_the_identity() {
        for slate in [two_gases(), liquid_slate()] {
            for weights in [[0.5, 0.5], [0.9, 0.1], [0.02, 0.98], [1.0, 0.0], [0.0, 1.0]] {
                let original = Composition::from_weights(&weights).unwrap();
                let back = MoleFractions::from_mass(&original, &slate)
                    .unwrap()
                    .to_mass(&slate)
                    .unwrap();
                for (a, b) in original.fractions().iter().zip(back.fractions()) {
                    approx::assert_abs_diff_eq!(a, b, epsilon = 1e-14);
                }
            }
        }
    }

    /// The molar-basis mean and `Composition`'s mass-basis reciprocal mean are
    /// the same number for the same mixture. Two formulas, one quantity — and
    /// the mixture is 4× apart in molar mass so the two means are far from equal
    /// to each other's mistakes.
    #[test]
    fn the_two_mean_molar_masses_agree() {
        let slate = liquid_slate();
        for weights in [[0.5, 0.5], [0.9, 0.1], [0.15, 0.85]] {
            let mass = Composition::from_weights(&weights).unwrap();
            let moles = MoleFractions::from_mass(&mass, &slate).unwrap();
            approx::assert_relative_eq!(
                moles.mean_molar_mass(&slate).value(),
                mass.mean_molar_mass(&slate).value(),
                max_relative = 1e-13
            );
        }
    }

    /// A pure component is pure on either basis — the case where the two rules
    /// agree, recorded so nobody mistakes it for coverage. The slate-loading
    /// gate learned this shape twice already (`degenerate-fixture-disables-the-
    /// code-path`).
    #[test]
    fn a_pure_cut_is_pure_on_both_bases() {
        let slate = two_gases();
        let pure = Composition::pure(2, 1);
        let moles = MoleFractions::from_mass(&pure, &slate).unwrap();
        approx::assert_relative_eq!(moles.fractions()[1], 1.0, max_relative = 1e-12);
        assert_eq!(moles.fractions()[0], 0.0);
    }

    /// Mole fractions sum to 1 and are non-negative; the constructors refuse
    /// input that cannot make that true.
    #[test]
    fn the_constructors_normalize_or_refuse() {
        let sum: f64 = MoleFractions::from_amounts(&[3.0, 1.0, 4.0])
            .unwrap()
            .fractions()
            .iter()
            .sum();
        approx::assert_relative_eq!(sum, 1.0, max_relative = 1e-15);

        assert!(MoleFractions::from_amounts(&[0.0, 0.0]).is_err());
        assert!(MoleFractions::from_amounts(&[1.0, -1.0]).is_err());
        assert!(MoleFractions::from_amounts(&[1.0, f64::NAN]).is_err());
    }

    /// A zero molar mass is refused rather than divided by. Reachable only from
    /// a hand-built slate today — the loader has its own guard — which is
    /// exactly why the conversion carries its own.
    #[test]
    fn a_zero_molar_mass_is_refused() {
        let slate = Slate::new(vec![PseudoComponent {
            name: "impossible".into(),
            tb: Kelvin(400.0),
            molar_mass: KgPerMol(0.0),
            density: Some(KgPerM3(800.0)),
            cp: JPerKgK(2000.0),
            phase: Phase::Liquid,
        }])
        .unwrap();
        let message = MoleFractions::from_mass(&Composition::pure(1, 0), &slate)
            .expect_err("a zero molar mass has no mole fraction")
            .to_string();
        assert!(
            message.contains("impossible"),
            "the refusal must name the component, got: {message}"
        );
    }
}
