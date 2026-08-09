//! Pseudo-component slate and stream composition.
//!
//! Crude is modeled as boiling-point cuts (pseudo-components), the standard
//! process-simulation approach. A scenario defines its slate once; every
//! `Composition` in that engine indexes into it. Water-only scenarios use a
//! one-component slate.

use crate::error::SimError;
use crate::units::{JPerKgK, Kelvin, KgPerM3, KgPerMol, Pascal, R_GAS};
use serde::{Deserialize, Serialize};

/// Which phase a pseudo-component is in.
///
/// **Phase is a property of the COMPONENT, not of the stream** (docs/DESIGN.md
/// §3a, fork 1). A vapour fraction on the stream is the two-phase model, which
/// M5 defers entirely; what a component's phase buys instead is a density law —
/// `PseudoComponent::density` for a liquid, `ρ = P·M̄/(R·T)` for a gas.
///
/// The deferral is made loud rather than silent by a load-time guard: every
/// connected component of the plant graph must be all-gas or all-liquid, so a
/// model with no phase equilibrium is never handed a two-phase mixture to
/// quietly volume-average. See `refinery_scenarios`' phase analysis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Phase {
    Liquid,
    Gas,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PseudoComponent {
    pub name: String,
    /// True-boiling-point of the cut (used later for K-values / cut splits).
    pub tb: Kelvin,
    pub molar_mass: KgPerMol,
    /// Liquid density at reference conditions. `None` for a gas-phase
    /// component, whose density is `P·M̄/(R·T)` and never a stored constant —
    /// carrying a condensed-phase density that nothing reads would be an
    /// authoritative-looking number with no effect, which is the failure mode
    /// this workspace refuses. The loader enforces the correspondence
    /// (declared iff liquid), so `mixture_density` may assume it.
    pub density: Option<KgPerM3>,
    pub cp: JPerKgK,
    pub phase: Phase,
}

impl PseudoComponent {
    pub fn water() -> Self {
        Self {
            name: "water".into(),
            tb: Kelvin(373.15),
            molar_mass: KgPerMol(0.018),
            density: Some(KgPerM3(998.0)),
            cp: JPerKgK(4184.0),
            phase: Phase::Liquid,
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
    ///
    /// **Liquid only, and the LOADER is what makes that true.** Since M5.2 the
    /// only callers are the two tank paths — `network::fixed_pressure`'s
    /// hydrostatic head and the scenario loader's `ρ·A·h` inventory — and the
    /// loader refuses a tank whose composition is gas-phase, which is precisely
    /// why that guard sits at the tank rather than being left to the
    /// connected-component check (docs/DESIGN.md §3a). Transport does NOT come
    /// through here any more; it goes through `density_at`.
    ///
    /// There is deliberately no safety net beyond that guard: with it removed, a
    /// gas component's missing density contributes `f/0 = ∞` and the mixture
    /// density comes out as **zero**, which on a tank means a 0 kg inventory and
    /// a bare-atmospheric bottom pressure — silently, in release. The
    /// `debug_assert` is the only thing that fires, and only in test builds,
    /// which is exactly how the tank guard's own falsification caught it.
    pub fn mixture_density(&self, slate: &Slate) -> KgPerM3 {
        let inv_rho: f64 = self
            .mass_fractions
            .iter()
            .enumerate()
            .filter(|(_, f)| **f > 0.0)
            .map(|(i, f)| {
                let component = slate.get(i);
                debug_assert!(
                    component.density.is_some(),
                    "mixture_density on a composition containing gas-phase '{}' \
                     (fraction {f}) — the single-phase guard should have refused this",
                    component.name
                );
                f / component.density.map_or(0.0, |d| d.value())
            })
            .sum();
        KgPerM3(1.0 / inv_rho)
    }

    /// Mean molar mass `M̄` [kg/mol]: `1/M̄ = Σ(wᵢ/Mᵢ)`.
    ///
    /// Mass-fraction weighting of the RECIPROCAL — the exact analogue of the
    /// liquid `mixture_density` rule above, and the correct one: `M̄` is total
    /// mass over total moles, and moles are what add.
    pub fn mean_molar_mass(&self, slate: &Slate) -> KgPerMol {
        let inv_m: f64 = self
            .mass_fractions
            .iter()
            .enumerate()
            .filter(|(_, f)| **f > 0.0)
            .map(|(i, f)| f / slate.get(i).molar_mass.value())
            .sum();
        KgPerMol(1.0 / inv_m)
    }

    /// The single phase of this composition, or `Err` if it mixes phases.
    ///
    /// Components with a zero mass fraction do not vote: a gas sub-plant on a
    /// slate that also defines liquid cuts carries zeros for those, and calling
    /// that mixture two-phase would refuse every mixed slate outright.
    pub fn phase(&self, slate: &Slate) -> Result<Phase, SimError> {
        let mut seen: Option<Phase> = None;
        for (i, f) in self.mass_fractions.iter().enumerate() {
            if *f <= 0.0 {
                continue;
            }
            let component = slate.get(i);
            match seen {
                None => seen = Some(component.phase),
                Some(p) if p == component.phase => {}
                Some(p) => {
                    return Err(SimError::Scenario(format!(
                        "composition mixes phases: '{}' is {:?} but an earlier component \
                         is {p:?}. This model has no phase equilibrium, so a two-phase \
                         mixture would be volume-averaged into a fluid that is neither \
                         (docs/DESIGN.md §3a)",
                        component.name, component.phase
                    )))
                }
            }
        }
        // Unreachable: fractions are normalized to sum to 1, so at least one is
        // positive. Liquid is the answer that changes nothing if it ever is.
        Ok(seen.unwrap_or(Phase::Liquid))
    }

    /// Density [kg/m³] at the given state, dispatched on the composition's
    /// phase: the stored liquid rule above, or ideal gas `ρ = P·M̄/(R·T)`.
    ///
    /// This is a MATERIAL property branching on a material property — not a
    /// fidelity `if` (CLAUDE.md rule 2). A gas and a liquid are different
    /// substances, not two accuracies of the same one; the fidelity knobs here
    /// would be real-gas `Z` and `cp(T)`, both deferred (docs/DESIGN.md §3a).
    ///
    /// `pressure` and `temperature` are ignored for a liquid — this fidelity has
    /// no compressibility or thermal expansion for the condensed phase, which is
    /// what §3's incompressible-liquid assumption already says.
    pub fn density_at(
        &self,
        slate: &Slate,
        pressure: Pascal,
        temperature: Kelvin,
    ) -> Result<KgPerM3, SimError> {
        match self.phase(slate)? {
            Phase::Liquid => Ok(self.mixture_density(slate)),
            // Ideal gas law, ρ = P·M̄/(R·T) (docs/DESIGN.md §3a). Real-gas Z is
            // deferred: ideal is adequate well away from the critical point.
            Phase::Gas => {
                let m_bar = self.mean_molar_mass(slate).value();
                let t = temperature.value();
                if !t.is_finite() || t <= 0.0 {
                    return Err(SimError::Numerical(format!(
                        "gas density at a non-positive or non-finite temperature ({t} K)"
                    )));
                }
                Ok(KgPerM3(pressure.value() * m_bar / (R_GAS * t)))
            }
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::Pascal;

    fn gas(name: &str, molar_mass: f64) -> PseudoComponent {
        PseudoComponent {
            name: name.into(),
            tb: Kelvin(120.0),
            molar_mass: KgPerMol(molar_mass),
            density: None,
            cp: JPerKgK(2000.0),
            phase: Phase::Gas,
        }
    }

    fn liquid(name: &str, density: f64) -> PseudoComponent {
        PseudoComponent {
            name: name.into(),
            tb: Kelvin(400.0),
            molar_mass: KgPerMol(0.1),
            density: Some(KgPerM3(density)),
            cp: JPerKgK(2000.0),
            phase: Phase::Liquid,
        }
    }

    /// `1/M̄ = Σ(wᵢ/Mᵢ)` — the reciprocal rule, and the ONLY gate in the
    /// workspace that can see it.
    ///
    /// Every gas plant in the repo carries a pure single-component gas, where
    /// the reciprocal rule and the naive `M̄ = Σ(wᵢ·Mᵢ)` agree exactly, so the
    /// wired reference cannot discriminate. On a 50/50 methane/propane mixture
    /// they differ by 28%: `1/M̄ = 0.5/0.016 + 0.5/0.044 = 42.6136` gives
    /// M̄ = 0.0234668, while the mass-weighted slip gives 0.030.
    #[test]
    fn mean_molar_mass_weights_the_reciprocal() {
        let slate = Slate::new(vec![gas("methane", 0.016), gas("propane", 0.044)]).unwrap();
        let half = Composition::from_weights(&[0.5, 0.5]).unwrap();
        approx::assert_relative_eq!(
            half.mean_molar_mass(&slate).value(),
            1.0 / (0.5 / 0.016 + 0.5 / 0.044),
            max_relative = 1e-12
        );
        // The slip this pins against, stated so the gate's discrimination is
        // visible rather than implied.
        let mass_weighted = 0.5 * 0.016 + 0.5 * 0.044;
        assert!(
            (half.mean_molar_mass(&slate).value() - mass_weighted).abs() / mass_weighted > 0.2,
            "the two rules must be distinguishable on this mixture"
        );
        // A pure cut is the same under either rule — which is exactly why the
        // wired gas reference cannot carry this claim.
        approx::assert_relative_eq!(
            Composition::pure(2, 1).mean_molar_mass(&slate).value(),
            0.044,
            max_relative = 1e-12
        );
    }

    /// `ρ = P·M̄/(R·T)`, at two pressures and two temperatures — so the gate
    /// pins both dependences rather than one product that happens to land right.
    #[test]
    fn gas_density_follows_the_ideal_gas_law() {
        let slate = Slate::new(vec![gas("methane", 0.016_043)]).unwrap();
        let pure = Composition::pure(1, 0);
        for (p, t) in [(1.0e5, 293.15), (10.0e5, 293.15), (10.0e5, 400.0)] {
            let expected = p * 0.016_043 / (R_GAS * t);
            approx::assert_relative_eq!(
                pure.density_at(&slate, Pascal(p), Kelvin(t))
                    .unwrap()
                    .value(),
                expected,
                max_relative = 1e-12
            );
        }
    }

    /// A liquid's density ignores `P` and `T` — this fidelity has no
    /// compressibility or thermal expansion for the condensed phase (§3), and
    /// that is what keeps every pre-M5.2 network bit-identical under a solve
    /// that now passes a pressure iterate into `compile_edge`.
    #[test]
    fn liquid_density_ignores_pressure_and_temperature() {
        let slate = Slate::new(vec![liquid("water", 998.0)]).unwrap();
        let pure = Composition::pure(1, 0);
        let at_ambient = pure
            .density_at(&slate, Pascal(101_325.0), Kelvin(293.15))
            .unwrap();
        let at_pressure = pure
            .density_at(&slate, Pascal(50.0e5), Kelvin(450.0))
            .unwrap();
        assert_eq!(at_ambient.value().to_bits(), at_pressure.value().to_bits());
        approx::assert_relative_eq!(at_ambient.value(), 998.0, max_relative = 1e-12);
    }

    /// Zero-fraction components do not vote on a composition's phase. Without
    /// this, every mixed slate would be refused outright and a gas sub-plant
    /// could not share a file with the liquid plant it protects.
    #[test]
    fn a_zero_fraction_component_does_not_vote_on_phase() {
        let slate = Slate::new(vec![liquid("condensate", 998.0), gas("methane", 0.016)]).unwrap();
        assert_eq!(
            Composition::pure(2, 0).phase(&slate).unwrap(),
            Phase::Liquid
        );
        assert_eq!(Composition::pure(2, 1).phase(&slate).unwrap(), Phase::Gas);
        assert!(Composition::from_weights(&[0.5, 0.5])
            .unwrap()
            .phase(&slate)
            .is_err());
    }
}
