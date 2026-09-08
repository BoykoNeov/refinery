//! The heat-capacity seam's two models (M16.2, docs/DESIGN.md §20).
//!
//! [`ConstantEnthalpy`] is what every plant written before M16 means: one number
//! per cut, `h = c̄p·(T − T_REF)`. [`LinearCpEnthalpy`] integrates a `cp(T)` the
//! FILE declares, per component, and derives both capacities from that integral.
//!
//! **Neither is a refinement of the other in the way a solver tolerance is.** The
//! constant model is not "the linear one with a zero slope" — it reads a
//! different key (`cp_j_per_kg_k` rather than the shape's own anchor pair), and a
//! plant selecting it while a component declares a shape is refused at load, in
//! both directions. That is the same argument `boiloff` used against `"none"`
//! being a degenerate `"flash"` (§14 fork 9).
//!
//! # Why the constant model spells out expressions it could compose
//!
//! Every method below that looks redundant — a flux that could be `ṁ·h`, a stock
//! that could be `m·h`, a mix that could invert its own `h` — is holding a
//! floating-point GROUPING, not a formula. `enthalpy_flux` was `(ṁ·cp)·(T −
//! T_REF)` from M2.1 to M16.1, and regrouping it to `ṁ·(cp·(T − T_REF))` with no
//! shape and no seam anywhere moves **11 of the 19** shipped plants on each
//! fidelity (measured in a detached worktree before this module was written, and
//! not the same 11 on the two fidelities). Fork 4 promises these nineteen plants
//! stay identical *by construction*; this is where that promise is kept.
//!
//! `stream_enthalpy_flux` is the exception and is left to the trait's provided
//! implementation, because the pre-M16 expression was already `ṁ·(h + λ)` — the
//! grouping the seam wants — so composing it changes nothing.

use refinery_core::components::{Composition, Slate};
use refinery_core::energy::T_REF;
use refinery_core::error::SimError;
use refinery_core::traits::{EnthalpyModel, InflowEnthalpy};
use refinery_core::units::{JPerKg, JPerKgK, Kelvin, Kg, KgPerSec, Watt};

/// One constant heat capacity per cut — the pre-M16 arithmetic, unchanged to the
/// bit.
///
/// Reads `PseudoComponent::cp` and nothing else. A component's `cp_shape`, if it
/// has one, is invisible here; the loader refuses that pairing rather than
/// letting a declared number go unread (§20 fork 4).
#[derive(Debug, Default, Clone, Copy)]
pub struct ConstantEnthalpy;

impl EnthalpyModel for ConstantEnthalpy {
    fn name(&self) -> &'static str {
        "constant-cp"
    }

    fn specific_enthalpy(
        &self,
        slate: &Slate,
        composition: &Composition,
        temperature: Kelvin,
    ) -> Result<JPerKg, SimError> {
        let cp = composition.mixture_cp(slate).value();
        Ok(JPerKg(cp * (temperature.value() - T_REF.value())))
    }

    /// `(ṁ·cp)·(T − T_REF)` — the pre-M16 `energy::enthalpy_flux`, associativity
    /// included. See the module docs for why the parentheses are the point.
    fn enthalpy_flux(
        &self,
        slate: &Slate,
        composition: &Composition,
        mass_flow: KgPerSec,
        temperature: Kelvin,
    ) -> Result<Watt, SimError> {
        let cp = composition.mixture_cp(slate).value();
        Ok(Watt(
            mass_flow.value() * cp * (temperature.value() - T_REF.value()),
        ))
    }

    /// `(m·cp)·(T − T_REF)` — the pre-M16 tank branch's inline stock.
    fn enthalpy_stock(
        &self,
        slate: &Slate,
        composition: &Composition,
        mass: Kg,
        temperature: Kelvin,
    ) -> Result<f64, SimError> {
        let cp = composition.mixture_cp(slate).value();
        Ok(mass.value() * cp * (temperature.value() - T_REF.value()))
    }

    /// The constant, whatever interval is asked for — which is the right answer
    /// for this model and not a shortcut: with `cp` flat the mean over any
    /// interval IS `cp`.
    ///
    /// **Returned verbatim rather than as the difference quotient `(h(t₂) −
    /// h(t₁))/(t₂ − t₁)`.** The two agree to a few ULP and not to the bit, and
    /// the flash fraction at `boiloff.rs` divides by `Δh̄_vap` and integrates the
    /// result into a tank's inventory — so the quotient form moves every boiling
    /// plant. The interval is genuinely unused here; `t1 == t2` therefore needs no
    /// guard in this model, and fork 7's `0/0` belongs to the shaped one.
    fn mean_cp(
        &self,
        slate: &Slate,
        composition: &Composition,
        _t1: Kelvin,
        _t2: Kelvin,
    ) -> Result<JPerKgK, SimError> {
        Ok(composition.mixture_cp(slate))
    }

    fn spot_cp(
        &self,
        slate: &Slate,
        composition: &Composition,
        _temperature: Kelvin,
    ) -> Result<JPerKgK, SimError> {
        Ok(composition.mixture_cp(slate))
    }

    /// `u = cv·T − cp·T_REF` — M5.3's expression, and the reason it is not
    /// `cv·(T − T_REF)` is the whole of blowdown cooling
    /// (docs/DESIGN.md §3a fork 3).
    fn specific_internal_energy(
        &self,
        slate: &Slate,
        composition: &Composition,
        temperature: Kelvin,
    ) -> Result<JPerKg, SimError> {
        let cv = composition.mixture_cv(slate).value();
        let cp = composition.mixture_cp(slate).value();
        Ok(JPerKg(cv * temperature.value() - cp * T_REF.value()))
    }

    /// `T_REF + energy/(mass·cp)` — the pre-M16 tank inversion, one division by a
    /// product, which is why the trait hands over `(energy, mass)` unseparated.
    fn temperature_from_enthalpy(
        &self,
        slate: &Slate,
        composition: &Composition,
        energy: f64,
        mass: f64,
    ) -> Result<Kelvin, SimError> {
        let cp = composition.mixture_cp(slate).value();
        Ok(Kelvin(T_REF.value() + energy / (mass * cp)))
    }

    /// `(energy/mass + cp·T_REF)/cv` — the pre-M16 vessel inversion.
    fn temperature_from_internal_energy(
        &self,
        slate: &Slate,
        composition: &Composition,
        energy: f64,
        mass: f64,
    ) -> Result<Kelvin, SimError> {
        let cv = composition.mixture_cv(slate).value();
        let cp = composition.mixture_cp(slate).value();
        Ok(Kelvin((energy / mass + cp * T_REF.value()) / cv))
    }

    /// `T_REF + Σṁh/Σṁcp` — the expression `mix_inflows` has evaluated since
    /// M2.1. `mass_rate` is deliberately unread: dividing the enthalpy by it
    /// first and the capacity second is the same number and different bits.
    fn mix_temperature(
        &self,
        _slate: &Slate,
        _composition: &Composition,
        totals: InflowEnthalpy,
    ) -> Result<Kelvin, SimError> {
        Ok(Kelvin(
            T_REF.value() + totals.enthalpy_rate / totals.capacity_rate,
        ))
    }
}

/// A heat capacity linear in temperature, declared per component in the scenario
/// file: `cp(T) = cp_at_anchor + slope·(T − anchor_temperature)`.
///
/// # What this model is, and what it is not
///
/// **It ships no correlation.** The coefficients come from the FILE, on
/// `density_kg_per_m3`'s precedent — "a declared constant at this fidelity"
/// (§20 fork 3). That keeps the Watson–Nelson paraphrase §18 ruled inadmissible
/// out of the engine, and it collapses this milestone's blast radius to an
/// optional field plus a scenario file. It **moves** the citation rather than
/// removing it: the demo file ships numbers, and by M13 gate 3's rule a shipped
/// number with no envelope from outside the workspace is a consistency check
/// wearing a physics label. That envelope is `methane_cp_reference.rs`.
///
/// # The arithmetic, and why it is closed form
///
/// A mass-weighted sum of linear shapes is linear, so one composition reduces to
/// one pair `(c₀, s)` at the datum — `Composition::mixture_cp_shape` — and with
/// `y = T − T_REF`:
///
/// ```text
/// cp(T) = c₀ + s·y
/// h(T)  = c₀·y + ½·s·y²
/// u(T)  = h(T) − (R/M̄)·T
/// ```
///
/// Both inversions are then quadratics, solved in the numerically stable form
/// `x = c/q` rather than the schoolbook `(−b ± √D)/2a`. That choice is not
/// tidiness: `a = s/2` goes to zero as the shape flattens, and the schoolbook
/// root that survives that limit is the one whose numerator also vanishes. The
/// `c/q` branch degenerates gracefully to the constant model's own `h/c₀`, so a
/// nearly-flat declared shape does not divide by nearly zero. **The other root is
/// the mutation** (§20's substitute for one that had no subject): it is the
/// branch where `cp` has already gone negative, unreachable for any shape this
/// loader admits, and it is what gate 4 fails on.
#[derive(Debug, Default, Clone, Copy)]
pub struct LinearCpEnthalpy;

impl LinearCpEnthalpy {
    /// `(c₀, s)`: the mixture's capacity at `T_REF` and its slope.
    ///
    /// # Errors
    /// `SimError::Scenario` when a component carrying mass declares no shape.
    /// That is a load-time refusal in every shipped path (§20 fork 4), so
    /// reaching it means a caller built a `Slate` by hand — a fault, reported as
    /// one rather than a NaN.
    fn shape(slate: &Slate, composition: &Composition) -> Result<(f64, f64), SimError> {
        composition.mixture_cp_shape(slate, T_REF).ok_or_else(|| {
            SimError::Scenario(
                "the linear heat-capacity model was asked about a mixture whose components do \
                 not all declare a `cp` shape. The loader refuses this pairing, so this state \
                 is only reachable from a hand-built slate"
                    .into(),
            )
        })
    }

    /// `h(T) = c₀·y + ½·s·y²` with `y = T − T_REF` [J/kg].
    fn h(c0: f64, s: f64, temperature: Kelvin) -> f64 {
        let y = temperature.value() - T_REF.value();
        c0 * y + 0.5 * s * y * y
    }

    /// The physical root of `½·s·y² + b·y − k = 0` [K above `T_REF`].
    ///
    /// One helper for both inversions: the enthalpy one has `b = c₀`, the
    /// internal-energy one `b = c₀ − R/M̄`, and nothing else differs. Stable
    /// branch, positive-`b` case — every admissible shape has `cp(T_REF) > 0` and
    /// a non-negative slope, so `b > 0` for the enthalpy inversion, and `cv > 0`
    /// for the other, both enforced at load.
    fn invert(b: f64, s: f64, k: f64, what: &str) -> Result<f64, SimError> {
        let discriminant = b * b + 2.0 * s * k;
        if discriminant < 0.0 {
            return Err(SimError::Numerical(format!(
                "the declared cp shape has no {what} at {k:.6e} J/kg: the quadratic \
                 ½·s·y² + b·y − k with b = {b:.6e} J/(kg·K), s = {s:.6e} J/(kg·K²) has \
                 discriminant {discriminant:.6e}. The state is below the temperature at which \
                 this shape's capacity would go negative, which is outside the range the load-time \
                 monotonicity check admits"
            )));
        }
        let denominator = b + discriminant.sqrt();
        if denominator <= 0.0 || !denominator.is_finite() {
            return Err(SimError::Numerical(format!(
                "the declared cp shape inverts to a degenerate root for {what} at {k:.6e} J/kg \
                 (b = {b:.6e}, s = {s:.6e})"
            )));
        }
        Ok(2.0 * k / denominator)
    }

    /// Guard the divisor a specific quantity is about to be taken over.
    fn per_unit(energy: f64, mass: f64, what: &str) -> Result<f64, SimError> {
        if mass <= 0.0 || !mass.is_finite() {
            return Err(SimError::Numerical(format!(
                "cannot read a temperature back from {what} over a mass of {mass} kg"
            )));
        }
        Ok(energy / mass)
    }
}

impl EnthalpyModel for LinearCpEnthalpy {
    fn name(&self) -> &'static str {
        "linear-cp"
    }

    fn specific_enthalpy(
        &self,
        slate: &Slate,
        composition: &Composition,
        temperature: Kelvin,
    ) -> Result<JPerKg, SimError> {
        let (c0, s) = Self::shape(slate, composition)?;
        Ok(JPerKg(Self::h(c0, s, temperature)))
    }

    fn enthalpy_flux(
        &self,
        slate: &Slate,
        composition: &Composition,
        mass_flow: KgPerSec,
        temperature: Kelvin,
    ) -> Result<Watt, SimError> {
        let h = self.specific_enthalpy(slate, composition, temperature)?;
        Ok(Watt(mass_flow.value() * h.value()))
    }

    fn enthalpy_stock(
        &self,
        slate: &Slate,
        composition: &Composition,
        mass: Kg,
        temperature: Kelvin,
    ) -> Result<f64, SimError> {
        let h = self.specific_enthalpy(slate, composition, temperature)?;
        Ok(mass.value() * h.value())
    }

    /// `(h(t₂) − h(t₁))/(t₂ − t₁)`, and the SPOT value at `t₂ == t₁`.
    ///
    /// Exact equality rather than a threshold (§20 fork 7): the limit is the spot
    /// value, so the guard returns a right answer and not a convenient one, and a
    /// threshold would be a magic number that also flattened legitimately small
    /// intervals.
    ///
    /// For this shape the quotient reduces to `c₀ + ½·s·(y₁ + y₂)` — the capacity
    /// at the interval's MIDPOINT — which is worth noticing and not worth
    /// shipping: writing the midpoint form directly would be a second expression
    /// of the integral, and the drift between two such expressions is the datum
    /// fork 1 exists to protect.
    fn mean_cp(
        &self,
        slate: &Slate,
        composition: &Composition,
        t1: Kelvin,
        t2: Kelvin,
    ) -> Result<JPerKgK, SimError> {
        if t2.value() == t1.value() {
            return self.spot_cp(slate, composition, t1);
        }
        let (c0, s) = Self::shape(slate, composition)?;
        Ok(JPerKgK(
            (Self::h(c0, s, t2) - Self::h(c0, s, t1)) / (t2.value() - t1.value()),
        ))
    }

    fn spot_cp(
        &self,
        slate: &Slate,
        composition: &Composition,
        temperature: Kelvin,
    ) -> Result<JPerKgK, SimError> {
        let (c0, s) = Self::shape(slate, composition)?;
        Ok(JPerKgK(c0 + s * (temperature.value() - T_REF.value())))
    }

    /// `u(T) = h(T) − (R/M̄)·T`.
    ///
    /// §18 predicted this expression would become ill-formed under a shape,
    /// because M5.3's closed form holds two capacities at two temperatures. §20
    /// corrected that: `R/M̄` is temperature independent, so the RELATION
    /// generalises in one line and only the closed form is lost.
    fn specific_internal_energy(
        &self,
        slate: &Slate,
        composition: &Composition,
        temperature: Kelvin,
    ) -> Result<JPerKg, SimError> {
        let h = self
            .specific_enthalpy(slate, composition, temperature)?
            .value();
        let offset = composition.gas_constant_offset(slate).value();
        Ok(JPerKg(h - offset * temperature.value()))
    }

    fn temperature_from_enthalpy(
        &self,
        slate: &Slate,
        composition: &Composition,
        energy: f64,
        mass: f64,
    ) -> Result<Kelvin, SimError> {
        let (c0, s) = Self::shape(slate, composition)?;
        let h = Self::per_unit(energy, mass, "an enthalpy")?;
        Ok(Kelvin(
            T_REF.value() + Self::invert(c0, s, h, "temperature")?,
        ))
    }

    /// Invert `u = ½·s·y² + (c₀ − R/M̄)·y − (R/M̄)·T_REF`.
    ///
    /// The constant term is what makes this a different quadratic from the
    /// enthalpy one rather than the same one shifted, and dropping it is M5.3's
    /// `cv·(T − T_REF)` slip wearing a shape.
    fn temperature_from_internal_energy(
        &self,
        slate: &Slate,
        composition: &Composition,
        energy: f64,
        mass: f64,
    ) -> Result<Kelvin, SimError> {
        let (c0, s) = Self::shape(slate, composition)?;
        let offset = composition.gas_constant_offset(slate).value();
        let u = Self::per_unit(energy, mass, "an internal energy")?;
        let k = u + offset * T_REF.value();
        Ok(Kelvin(
            T_REF.value() + Self::invert(c0 - offset, s, k, "temperature")?,
        ))
    }

    /// `T(Σṁh/Σṁ)` — the mixed SPECIFIC ENTHALPY, inverted.
    ///
    /// A different formula from the constant model's `T_REF + Σṁh/Σṁcp`, not a
    /// different grouping of it: enthalpy is what mixes, and a capacity-weighted
    /// temperature average is only the same answer while `cp` is flat.
    fn mix_temperature(
        &self,
        slate: &Slate,
        composition: &Composition,
        totals: InflowEnthalpy,
    ) -> Result<Kelvin, SimError> {
        self.temperature_from_enthalpy(slate, composition, totals.enthalpy_rate, totals.mass_rate)
    }
}
