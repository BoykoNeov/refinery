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

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PseudoComponent {
    pub name: String,
    /// The cut's true boiling point **at `units::P_ATM`** — the NORMAL boiling
    /// point, and the pressure matters as of M7.2.
    ///
    /// It was unstated while the only reader was the cut-point splitter, which
    /// compares `tb` against a cut temperature and never against a pressure. A
    /// K-value correlation anchors on it instead (`ThermoModel::k_value`:
    /// `K = 1` at `T = tb`, `P = P_ATM`), so the reference pressure is now
    /// load-bearing rather than descriptive.
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
    /// The temperature dependence of this cut's heat capacity, when the file
    /// declares one (M16.2, docs/DESIGN.md §20 fork 3).
    ///
    /// `None` is not "a flat shape" — it is "this component declares no shape",
    /// which is what nineteen of the twenty shipped plants mean and what the
    /// loader REFUSES to pair with `[fidelity] heat_capacity = "linear"`. The
    /// two are different statements for the same reason `Stream::latent` is
    /// `None` on a liquid rather than `Some(0)`.
    ///
    /// `cp` above stays *the* constant, unshadowed: a shaped component declares
    /// its own anchor pair rather than re-interpreting `cp_j_per_kg_k` as "the
    /// value at some unwritten temperature" (§20 fork 3's named trap, and B17's
    /// precedent — a key whose meaning was implicit).
    ///
    /// `skip_serializing_if` because `PseudoComponent` is `Serialize` and a new
    /// always-present key would move every plant's bytes for a field nineteen of
    /// them do not have.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cp_shape: Option<CpShape>,
    pub phase: Phase,
}

/// A declared temperature dependence for one cut's heat capacity [J/(kg·K)]:
///
/// ```text
/// cp(T) = cp_at_anchor + slope·(T − anchor_temperature)
/// ```
///
/// **Linear, and that is a verdict rather than a starting point** (§20 fork 2,
/// verdict (3)): a linear `cp` makes `h(T) = ∫cp` quadratic and the inversion
/// `T(h)` a quadratic formula — exact, no iteration, and therefore no evaluation
/// bound to defend inside the tick loop, where an inversion runs once per holdup
/// per tick rather than once per solve.
///
/// **The anchor is a PAIR and the slope is separate**, so "which temperature is
/// `cp` quoted at" is written in the file rather than implied by a constant that
/// already means something else.
///
/// **`slope >= 0` is a load-time refusal, not a convention.** With a
/// non-negative slope and `cp(T_REF) > 0` — both checked at load — `cp` is
/// positive for every `T >= T_REF` and `h` is strictly increasing there, so the
/// inverse exists in closed form and is single valued without the file having to
/// declare a validity range. A decreasing `cp` would need one, and there is no
/// key for it; that is deferred rather than approximated (docs/DESIGN.md §20).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CpShape {
    /// The temperature the declared capacity is quoted AT [K].
    pub anchor_temperature: Kelvin,
    /// The capacity at `anchor_temperature` [J/(kg·K)].
    pub cp_at_anchor: JPerKgK,
    /// `dcp/dT` [J/(kg·K²)]. Refused negative at load — see the type docs.
    pub slope: f64,
}

impl CpShape {
    /// This shape re-anchored at `energy::T_REF`, as the pair
    /// `(cp(T_REF), slope)`.
    ///
    /// Every consumer wants the shape on the engine's own datum, and moving the
    /// anchor there once — rather than carrying `anchor_temperature` into the
    /// integral — is what keeps `h`, its mean and its inverse from each having
    /// to remember where the file's author chose to quote the number.
    #[must_use]
    pub fn at_datum(&self, t_ref: Kelvin) -> (f64, f64) {
        (
            self.cp_at_anchor.value()
                + self.slope * (t_ref.value() - self.anchor_temperature.value()),
            self.slope,
        )
    }
}

impl PseudoComponent {
    pub fn water() -> Self {
        Self {
            name: "water".into(),
            tb: Kelvin(373.15),
            molar_mass: KgPerMol(0.018),
            density: Some(KgPerM3(998.0)),
            cp: JPerKgK(4184.0),
            // Water declares no shape, and that is fork 5's whole verdict: the
            // six plants with no `[[components]]` block reach this constructor,
            // so a shape here would move their bytes for a milestone whose demo
            // is elsewhere. The loader refuses the shaped model on those plants
            // instead (docs/DESIGN.md §20 fork 5).
            cp_shape: None,
            phase: Phase::Liquid,
        }
    }
}

/// The ordered component list for one engine instance. Order is canonical:
/// `Composition` fractions index into it. Immutable after engine build.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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

    /// Mixture heat capacity at constant VOLUME [J/(kg·K)]: mass-fraction
    /// weighted, `cv = cp` for a liquid component and `cp − R/M` for a gas one.
    ///
    /// The phase branch is legitimate for the reason `density_at`'s is — it is a
    /// property law branching on a material property, not a fidelity `if`
    /// (docs/DESIGN.md §3a fork 3). And it must be a branch: applying `cp − R/M`
    /// to water shifts its `cv` by 11%, which would move every golden in the
    /// workspace. For a liquid this returns `mixture_cp` bit for bit.
    ///
    /// Weighting the per-component `cv` by mass fraction is the SAME rule as
    /// `cp − R/M̄` on the mixture, not an approximation of it:
    /// `Σ fᵢ(cpᵢ − R/Mᵢ) = cp̄ − R·Σ(fᵢ/Mᵢ) = cp̄ − R/M̄`, because `M̄` is defined by
    /// exactly that reciprocal sum (`mean_molar_mass`). Both forms are gated
    /// against each other below, so neither can drift.
    ///
    /// Why this exists at all: a gas holdup's internal energy is not its
    /// enthalpy. Using `cp` for a blowing-down vessel overstates its stored
    /// energy by a factor of `γ` and, with it, suppresses the cooling that makes
    /// a relief event look like one.
    pub fn mixture_cv(&self, slate: &Slate) -> JPerKgK {
        JPerKgK(
            self.mass_fractions
                .iter()
                .enumerate()
                .map(|(i, f)| {
                    let component = slate.get(i);
                    let cv = match component.phase {
                        Phase::Liquid => component.cp.value(),
                        Phase::Gas => component.cp.value() - R_GAS / component.molar_mass.value(),
                    };
                    f * cv
                })
                .sum(),
        )
    }

    /// `Σ_{gas} fᵢ·R/Mᵢ` [J/(kg·K)]: the gap between a mixture's `cp` and its
    /// `cv`, and it is **temperature independent** whatever `cp` does.
    ///
    /// That independence is the whole of §20's correction to §18. `h − u = P/ρ =
    /// (R/M̄)·T` is fixed by thermodynamics, so `cv(T) = cp(T) − R/M̄` holds
    /// exactly under any shape and `u(T) = h(T) − (R/M̄)·T` is a one-line
    /// generalisation rather than the ill-formed expression §18 predicted. A
    /// liquid contributes nothing, mirroring `mixture_cv`'s phase branch, so on
    /// an all-liquid mixture this is exactly zero and `u = h`.
    ///
    /// **Not `mixture_cp − mixture_cv`**, although it equals that algebraically:
    /// this sums the offsets directly, which is what a shaped model needs (it has
    /// no single `cp` to subtract from) and what keeps the quantity independent
    /// of any capacity at all.
    pub fn gas_constant_offset(&self, slate: &Slate) -> JPerKgK {
        JPerKgK(
            self.mass_fractions
                .iter()
                .enumerate()
                .map(|(i, f)| {
                    let component = slate.get(i);
                    match component.phase {
                        Phase::Liquid => 0.0,
                        Phase::Gas => f * R_GAS / component.molar_mass.value(),
                    }
                })
                .sum(),
        )
    }

    /// The mixture's declared shape, re-anchored at `t_ref`: `(cp(t_ref), slope)`
    /// such that `cp_mix(T) = cp(t_ref) + slope·(T − t_ref)`.
    ///
    /// A mass-weighted sum of linear shapes is itself linear, which is why the
    /// mixture can be reduced to one pair once and every consumer — the integral,
    /// the mean, the spot value and the inverse — then read the same two numbers.
    /// Re-anchoring at the datum here rather than in each consumer is what stops
    /// four expressions from having to agree about where the file quoted `cp`.
    ///
    /// `None` when any component carrying a nonzero fraction declares no shape.
    /// A zero-fraction component does not vote, on the same argument as `phase`:
    /// a gas sub-plant on a slate that also defines liquid cuts carries zeros for
    /// those, and letting them vote would refuse every mixed slate outright.
    pub fn mixture_cp_shape(&self, slate: &Slate, t_ref: Kelvin) -> Option<(f64, f64)> {
        let mut cp_at_ref = 0.0;
        let mut slope = 0.0;
        for (i, f) in self.mass_fractions.iter().enumerate() {
            if *f <= 0.0 {
                continue;
            }
            let (c, s) = slate.get(i).cp_shape?.at_datum(t_ref);
            cp_at_ref += f * c;
            slope += f * s;
        }
        Some((cp_at_ref, slope))
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
            cp_shape: None,
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
            cp_shape: None,
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

    /// `cv` by mass-weighted components equals `cp̄ − R/M̄` on the mixture — the
    /// two forms of the same rule, gated against each other so neither drifts.
    ///
    /// On a 50/50 methane/propane blend the two molar masses differ by 2.75×, so
    /// this is a real agreement and not an identity read off a pure cut.
    #[test]
    fn mixture_cv_agrees_with_cp_minus_r_over_mean_molar_mass() {
        let slate = Slate::new(vec![gas("methane", 0.016), gas("propane", 0.044)]).unwrap();
        let half = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let from_mean =
            half.mixture_cp(&slate).value() - R_GAS / half.mean_molar_mass(&slate).value();
        approx::assert_relative_eq!(
            half.mixture_cv(&slate).value(),
            from_mean,
            max_relative = 1e-12
        );
        // And it is a real correction, not a rounding: γ = cp/cv is well above 1.
        let gamma = half.mixture_cp(&slate).value() / half.mixture_cv(&slate).value();
        assert!(
            gamma > 1.1,
            "a gas mixture's γ must be distinguishable from 1, got {gamma}"
        );
    }

    /// A LIQUID's `cv` is its `cp`, bit for bit — the phase branch is what keeps
    /// every all-liquid golden in the workspace unchanged. Applying the gas rule
    /// to water would shift it by 11%, which the second assertion sizes.
    #[test]
    fn a_liquids_cv_is_its_cp_exactly() {
        let slate = Slate::new(vec![PseudoComponent::water()]).unwrap();
        let water = Composition::pure(1, 0);
        assert_eq!(
            water.mixture_cv(&slate).value().to_bits(),
            water.mixture_cp(&slate).value().to_bits()
        );
        let if_gas_rule = 4184.0 - R_GAS / 0.018;
        assert!(
            (4184.0 - if_gas_rule) / 4184.0 > 0.10,
            "the slip this branch prevents must be large enough to matter"
        );
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
