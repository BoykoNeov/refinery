//! Pseudo-component slate and stream composition.
//!
//! Crude is modeled as boiling-point cuts (pseudo-components), the standard
//! process-simulation approach. A scenario defines its slate once; every
//! `Composition` in that engine indexes into it. Water-only scenarios use a
//! one-component slate.

use crate::error::SimError;
use crate::units::{JPerKgK, Kelvin, KgPerM3, KgPerMol};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PseudoComponent {
    pub name: String,
    /// True-boiling-point of the cut (used later for K-values / cut splits).
    pub tb: Kelvin,
    pub molar_mass: KgPerMol,
    /// Liquid density at reference conditions.
    pub density: KgPerM3,
    pub cp: JPerKgK,
}

impl PseudoComponent {
    pub fn water() -> Self {
        Self {
            name: "water".into(),
            tb: Kelvin(373.15),
            molar_mass: KgPerMol(0.018),
            density: KgPerM3(998.0),
            cp: JPerKgK(4184.0),
        }
    }
}

/// The ordered component list for one engine instance. Order is canonical:
/// `Composition` fractions index into it. Immutable after engine build.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Slate {
    components: Vec<PseudoComponent>,
}

impl Slate {
    pub fn new(components: Vec<PseudoComponent>) -> Result<Self, SimError> {
        if components.is_empty() {
            return Err(SimError::Scenario("slate must have >= 1 component".into()));
        }
        Ok(Self { components })
    }
    pub fn water_only() -> Self {
        Self {
            components: vec![PseudoComponent::water()],
        }
    }
    pub fn len(&self) -> usize {
        self.components.len()
    }
    pub fn is_empty(&self) -> bool {
        false // enforced at construction
    }
    pub fn get(&self, i: usize) -> &PseudoComponent {
        &self.components[i]
    }
    pub fn iter(&self) -> impl Iterator<Item = &PseudoComponent> {
        self.components.iter()
    }
    pub fn index_of(&self, name: &str) -> Option<usize> {
        self.components.iter().position(|c| c.name == name)
    }
}

/// Mass fractions over the slate. Invariant: fractions sum to 1 (±1e-9)
/// and are all >= 0. Constructors normalize; mutation goes through methods
/// that preserve the invariant.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Composition {
    mass_fractions: Vec<f64>,
}

impl Composition {
    /// Normalizes the given weights. Errors on negative, non-finite, or
    /// all-zero input.
    pub fn from_weights(weights: &[f64]) -> Result<Self, SimError> {
        if weights.iter().any(|w| !w.is_finite() || *w < 0.0) {
            return Err(SimError::Scenario(
                "composition weights must be finite and >= 0".into(),
            ));
        }
        let sum: f64 = weights.iter().sum();
        if sum <= 0.0 {
            return Err(SimError::Scenario("composition weights sum to zero".into()));
        }
        Ok(Self {
            mass_fractions: weights.iter().map(|w| w / sum).collect(),
        })
    }

    pub fn pure(slate_len: usize, index: usize) -> Self {
        let mut f = vec![0.0; slate_len];
        f[index] = 1.0;
        Self { mass_fractions: f }
    }

    pub fn fractions(&self) -> &[f64] {
        &self.mass_fractions
    }

    /// Mass-weighted blend of two compositions (e.g. at a junction or into
    /// a tank inventory). Weights are the mixing mass amounts.
    pub fn blend(a: &Self, wa: f64, b: &Self, wb: f64) -> Result<Self, SimError> {
        debug_assert_eq!(a.mass_fractions.len(), b.mass_fractions.len());
        let weights: Vec<f64> = a
            .mass_fractions
            .iter()
            .zip(&b.mass_fractions)
            .map(|(fa, fb)| fa * wa + fb * wb)
            .collect();
        Self::from_weights(&weights)
    }

    /// Mixture liquid density: volume-fraction weighting (1/ρ mass-weighted),
    /// the correct rule for ideal liquid blending.
    pub fn mixture_density(&self, slate: &Slate) -> KgPerM3 {
        let inv_rho: f64 = self
            .mass_fractions
            .iter()
            .enumerate()
            .map(|(i, f)| f / slate.get(i).density.value())
            .sum();
        KgPerM3(1.0 / inv_rho)
    }

    /// Mixture heat capacity: mass-fraction weighted.
    pub fn mixture_cp(&self, slate: &Slate) -> JPerKgK {
        JPerKgK(
            self.mass_fractions
                .iter()
                .enumerate()
                .map(|(i, f)| f * slate.get(i).cp.value())
                .sum(),
        )
    }
}
