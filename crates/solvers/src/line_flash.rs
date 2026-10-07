//! What a liquid stream does below its bubble point — the M53 line-flash seam
//! (docs/DESIGN.md §58, `[fidelity] line_flash`).
//!
//! Two implementations of `LineFlashModel`, and neither refines the other.
//! `NoLineFlash` carries every stream as the liquid it was declared — what every
//! plant written before M53 says, and the default. `EquilibriumLineFlash` boils
//! a supply above its bubble point where it stands, and re-boils the stream at
//! every zero-volume node as its pressure falls.
//!
//! **The two flashes are different on purpose** (§58 fork 2). A supply HOLDS its
//! temperature — the heat its vapour carries comes from outside the plant — so it
//! is an isothermal flash at its declared `(T, P)`, `flash_isothermal`. A valve or
//! a junction holds nothing: its vapour's latent heat can only come from the
//! liquid, so it is an ISENTHALPIC flash, and the stream cools as it boils. An
//! isothermal flash there creates the latent heat from nowhere — the first
//! measurement of the shipped plants' lines made exactly that error and read
//! 27–70% vapour where the energy balance allows 5–15% (ROADMAP M53).
//!
//! Everything about the vapour's ENERGY is on this engine's saturated-liquid
//! datum (`energy::T_REF`): a stream's specific enthalpy is `h(T) + latent`, with
//! `latent = q·λ` per kilogram of the whole stream (`Stream::latent`).

use refinery_core::components::{Composition, Slate};
use refinery_core::error::SimError;
use refinery_core::traits::{
    EnthalpyModel, LineFlashModel, Settled, ThermoModel, TwoPhaseDensity, VapourShare,
};
use refinery_core::units::{JPerKg, Kelvin, KgPerM3, Pascal, R_GAS};

use crate::bubble::bubble_temperature;
use crate::flash::{flash_isothermal, FlashResult};
use crate::molar::MoleFractions;

/// Every stream is the liquid it was declared: the `[fidelity] line_flash =
/// "none"` implementation, and the default.
///
/// **The default because that is what a pre-M53 file MEANS** — the `boiloff`
/// precedent (§14 fork 9). Its `settle` hands the mix back untouched, so a plant
/// selecting it keeps every bit it had.
pub struct NoLineFlash;

impl LineFlashModel for NoLineFlash {
    fn name(&self) -> &'static str {
        "none"
    }

    fn carries_vapour(&self) -> bool {
        false
    }

    fn supply(
        &self,
        _slate: &Slate,
        _composition: &Composition,
        _temperature: Kelvin,
        _pressure: Pascal,
        _thermo: &dyn ThermoModel,
        _enthalpy: &dyn EnthalpyModel,
    ) -> Result<Option<VapourShare>, SimError> {
        Ok(None)
    }

    fn settle(
        &self,
        _slate: &Slate,
        _composition: &Composition,
        liquid_equivalent: Kelvin,
        _pressure: Pascal,
        _thermo: &dyn ThermoModel,
        _enthalpy: &dyn EnthalpyModel,
    ) -> Result<Settled, SimError> {
        Ok(Settled {
            temperature: liquid_equivalent,
            vapour: None,
        })
    }

    fn density(
        &self,
        _slate: &Slate,
        _composition: &Composition,
        _liquid_equivalent: Kelvin,
        _pressure: Pascal,
        _thermo: &dyn ThermoModel,
        _enthalpy: &dyn EnthalpyModel,
    ) -> Result<Option<TwoPhaseDensity>, SimError> {
        Ok(None)
    }
}

/// Vapour–liquid equilibrium on the line: `[fidelity] line_flash =
/// "equilibrium"`.
///
/// Needs a `ThermoModel` with K-values and a bubble pressure — `trouton` — which
/// the loader requires of a plant selecting it.
pub struct EquilibriumLineFlash;

/// Halvings of the isenthalpic flash's temperature bracket. Its width is the
/// liquid-equivalent temperature's rise over the bubble point — at most a few
/// hundred kelvin — so `2⁻⁶⁰` of it is below an `f64`'s spacing there, and the
/// loop is a fixed cost with no convergence arm to fail (the `flash_isothermal`
/// argument).
const BISECTION_STEPS: u32 = 60;

/// One equilibrium state at `(T, P)`, the numbers every caller here reads.
struct Split {
    /// Vapour's share of the MASS [-] from the flash (not yet the energy's).
    mass_fraction: f64,
    /// The vapour's heat of vaporisation per kilogram of VAPOUR [J/kg].
    latent_per_vapour: f64,
    /// Ideal-gas vapour density `P·M̄_v/(R·T)` [kg/m³].
    vapour_density: f64,
    /// The equilibrium liquid's density [kg/m³].
    liquid_density: f64,
}

impl EquilibriumLineFlash {
    /// The isothermal flash at `(temperature, pressure)`, read in mass.
    fn split(
        slate: &Slate,
        feed: &MoleFractions,
        temperature: Kelvin,
        pressure: Pascal,
        thermo: &dyn ThermoModel,
    ) -> Result<Split, SimError> {
        let FlashResult {
            vapour_fraction: beta,
            liquid,
            vapour,
        } = flash_isothermal(feed, slate, thermo, temperature, pressure)?;
        let vapour_molar_mass = vapour.mean_molar_mass(slate).value();
        let liquid_molar_mass = liquid.mean_molar_mass(slate).value();
        // Molar to mass: β moles of vapour weigh β·M̄_v.
        let mass_fraction = beta * vapour_molar_mass
            / (beta * vapour_molar_mass + (1.0 - beta) * liquid_molar_mass);
        // λ of the VAPOUR — what boils is `y`, not the liquid it left — per
        // kilogram: Σ y_c·Δh_vap,c [J/mol] over M̄_v [kg/mol].
        let mut latent_molar = 0.0;
        for (c, y) in vapour.fractions().iter().enumerate() {
            latent_molar += y * thermo.dh_vap(slate, c, temperature)?.value();
        }
        let latent_per_vapour = latent_molar / vapour_molar_mass;
        // Ideal gas, ρ = P·M̄/(R·T) (docs/DESIGN.md §3a).
        let vapour_density = pressure.value() * vapour_molar_mass / (R_GAS * temperature.value());
        let liquid_density = liquid.to_mass(slate)?.mixture_density(slate).value();
        let split = Split {
            mass_fraction,
            latent_per_vapour,
            vapour_density,
            liquid_density,
        };
        if [
            split.mass_fraction,
            split.latent_per_vapour,
            split.vapour_density,
            split.liquid_density,
        ]
        .iter()
        .any(|v| !v.is_finite())
            || split.latent_per_vapour <= 0.0
            || split.vapour_density <= 0.0
            || split.liquid_density <= 0.0
        {
            return Err(SimError::Numerical(format!(
                "line flash at {:.3} K, {:.1} Pa: vapour share {}, latent heat {} J/kg, \
                 densities {} / {} kg/m³ — a flash must give finite, positive properties",
                temperature.value(),
                pressure.value(),
                split.mass_fraction,
                split.latent_per_vapour,
                split.vapour_density,
                split.liquid_density
            )));
        }
        Ok(split)
    }

    /// The isenthalpic flash: the node's temperature, its vapour and the two
    /// densities, from its liquid-equivalent temperature at `pressure`.
    ///
    /// `h(T) + q·λ = H` with `H = h(T_le)`. The temperature is found by
    /// bisection on `g(T) = h(T) + q_flash(T)·λ(T) − H` over
    /// `[T_bubble(P), T_le]`, where `g < 0` at the bubble point (`T_le` is above
    /// it) and `g ≥ 0` at `T_le`. **Then `q` is taken from the ENERGY, not from
    /// the flash**: `q = (H − h(T*))/λ(T*)`. On a mixture the two agree at the
    /// root; on a pure component they cannot — its flash jumps from all liquid
    /// to all vapour at one temperature, which the bisection pins and only the
    /// energy can divide (M53's spike tripped on exactly that step). And the
    /// energy's `q` makes `h(T*) + q·λ = H` hold to rounding by construction.
    #[allow(clippy::type_complexity)]
    fn settle_full(
        slate: &Slate,
        composition: &Composition,
        liquid_equivalent: Kelvin,
        pressure: Pascal,
        thermo: &dyn ThermoModel,
        enthalpy: &dyn EnthalpyModel,
    ) -> Result<(Settled, Option<(Split, f64)>), SimError> {
        let liquid = Settled {
            temperature: liquid_equivalent,
            vapour: None,
        };
        let feed = MoleFractions::from_mass(composition, slate)?;
        let bubble = bubble_temperature(slate, thermo, &feed, pressure, "a line flash")?;
        if liquid_equivalent.value() <= bubble.value() {
            return Ok((liquid, None));
        }
        let target = enthalpy
            .specific_enthalpy(slate, composition, liquid_equivalent)?
            .value();
        let excess = |t: f64| -> Result<f64, SimError> {
            let split = Self::split(slate, &feed, Kelvin(t), pressure, thermo)?;
            let sensible = enthalpy
                .specific_enthalpy(slate, composition, Kelvin(t))?
                .value();
            Ok(sensible + split.mass_fraction * split.latent_per_vapour - target)
        };
        let (mut low, mut high) = (bubble.value(), liquid_equivalent.value());
        for _ in 0..BISECTION_STEPS {
            let mid = 0.5 * (low + high);
            if excess(mid)? < 0.0 {
                low = mid;
            } else {
                high = mid;
            }
        }
        let temperature = 0.5 * (low + high);
        let split = Self::split(slate, &feed, Kelvin(temperature), pressure, thermo)?;
        let sensible = enthalpy
            .specific_enthalpy(slate, composition, Kelvin(temperature))?
            .value();
        // The energy's vapour share (see the doc above), clamped to `[0, 1]`
        // against the rounding of a root found to an f64's spacing.
        let mass_fraction = ((target - sensible) / split.latent_per_vapour).clamp(0.0, 1.0);
        if mass_fraction <= 0.0 {
            return Ok((liquid, None));
        }
        let settled = Settled {
            temperature: Kelvin(temperature),
            vapour: Some(VapourShare {
                mass_fraction,
                latent: JPerKg(mass_fraction * split.latent_per_vapour),
                liquid_equivalent,
            }),
        };
        Ok((settled, Some((split, mass_fraction))))
    }
}

impl LineFlashModel for EquilibriumLineFlash {
    fn name(&self) -> &'static str {
        "equilibrium"
    }

    fn carries_vapour(&self) -> bool {
        true
    }

    fn supply(
        &self,
        slate: &Slate,
        composition: &Composition,
        temperature: Kelvin,
        pressure: Pascal,
        thermo: &dyn ThermoModel,
        enthalpy: &dyn EnthalpyModel,
    ) -> Result<Option<VapourShare>, SimError> {
        let feed = MoleFractions::from_mass(composition, slate)?;
        let split = Self::split(slate, &feed, temperature, pressure, thermo)?;
        if split.mass_fraction <= 0.0 {
            return Ok(None);
        }
        let latent = split.mass_fraction * split.latent_per_vapour;
        // The liquid-equivalent temperature: all-liquid at the same enthalpy,
        // `h(T_le) = h(T) + latent` — one kilogram's worth, through the stock
        // inversion every holdup uses.
        let held = enthalpy
            .specific_enthalpy(slate, composition, temperature)?
            .value();
        let liquid_equivalent =
            enthalpy.temperature_from_enthalpy(slate, composition, held + latent, 1.0)?;
        Ok(Some(VapourShare {
            mass_fraction: split.mass_fraction,
            latent: JPerKg(latent),
            liquid_equivalent,
        }))
    }

    fn settle(
        &self,
        slate: &Slate,
        composition: &Composition,
        liquid_equivalent: Kelvin,
        pressure: Pascal,
        thermo: &dyn ThermoModel,
        enthalpy: &dyn EnthalpyModel,
    ) -> Result<Settled, SimError> {
        Self::settle_full(
            slate,
            composition,
            liquid_equivalent,
            pressure,
            thermo,
            enthalpy,
        )
        .map(|(settled, _)| settled)
    }

    fn density(
        &self,
        slate: &Slate,
        composition: &Composition,
        liquid_equivalent: Kelvin,
        pressure: Pascal,
        thermo: &dyn ThermoModel,
        enthalpy: &dyn EnthalpyModel,
    ) -> Result<Option<TwoPhaseDensity>, SimError> {
        let (_, two_phase) = Self::settle_full(
            slate,
            composition,
            liquid_equivalent,
            pressure,
            thermo,
            enthalpy,
        )?;
        Ok(two_phase.map(|(split, q)| {
            // Homogeneous equilibrium: 1/ρ = q/ρ_v + (1 − q)/ρ_l (Wallis 1969,
            // ch. 2) — specific volumes add by mass.
            let liquid_volume = (1.0 - q) / split.liquid_density;
            let specific_volume = q / split.vapour_density + liquid_volume;
            TwoPhaseDensity {
                mixture: KgPerM3(1.0 / specific_volume),
                liquid_volume_share: liquid_volume / specific_volume,
            }
        }))
    }
}
