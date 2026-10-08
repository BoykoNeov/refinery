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
//! is an isothermal flash at its declared `(T, P)`, `flash_on_the_line`. A valve or
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
use crate::flash::FlashResult;
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

/// Evaluations the isenthalpic flash's temperature search may spend (M56) — a
/// STRUCTURAL bound, not the working number (the bubble point's arrangement,
/// `bubble::BUBBLE_POINT_MAX_EVALUATIONS`). The search bisects whenever its
/// bracket has not halved over its last two steps, so it halves at least once
/// every two evaluations; the bracket — the liquid-equivalent temperature's
/// rise over the bubble point, at most a few thousand kelvin — reaches `2ε·T`
/// within about 60 halvings from anywhere. Measured over both boiling plants'
/// stories on the game solver: 7–43 evaluations, most 8–20.
const TEMPERATURE_MAX_EVALUATIONS: u32 = 200;

/// Evaluations the line flash's Rachford–Rice search may spend (M56): a
/// backstop past which it is an `Err`, never a silent answer. Its safeguard
/// (`rtsafe`) bisects whenever a Newton step leaves the bracket or fails to
/// halve the step before last; the bracket over `[0, 1]` closes at `2ε` of its
/// top or the smallest normal `f64`, which 1 022 halvings reach from anywhere.
/// Measured over both boiling plants' stories on the game solver: 3–12
/// evaluations, most 4–6, every one closed by the rounding test.
const VAPOUR_FRACTION_MAX_EVALUATIONS: u32 = 2_100;

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
        } = flash_on_the_line(feed, slate, thermo, temperature, pressure)?;
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
    /// `h(T) + q·λ = H` with `H = h(T_le)`. The temperature is the root of
    /// `g(T) = h(T) + q_flash(T)·λ(T) − H` over `[T_bubble(P), T_le]`, where
    /// `g < 0` at the bubble point (`T_le` is above it) and `g ≥ 0` at `T_le`,
    /// found to an `f64`'s spacing by a bracketed search (`temperature_root`).
    /// **Then `q` is taken from the ENERGY, not from the flash**:
    /// `q = (H − h(T*))/λ(T*)`. On a mixture the two agree at the root; on a pure
    /// component they cannot — its flash jumps from all liquid to all vapour at
    /// one temperature, which the bracket pins and only the energy can divide
    /// (M53's spike tripped on exactly that step). And the energy's `q` makes
    /// `h(T*) + q·λ = H` hold to rounding by construction.
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
        let temperature = temperature_root(bubble.value(), liquid_equivalent.value(), excess)?;
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

/// The root of `excess` on `[low, high]`, an increasing function with
/// `excess(low) < 0 ≤ excess(high)`, closed to `2ε·T` — the temperature search
/// of `settle_full` (M56, docs/DESIGN.md §61.3).
///
/// Until M56 this was 60 halvings, each an isothermal flash; profiled, that loop
/// was most of a boiling plant's tick, and the gas-lock story's worst tick was
/// 0.7 s on the game solver. It is the bubble point's search now (`bubble.rs`,
/// M9.3a): regula falsi with Illinois weighting (Dowell & Jarratt, *BIT* 11
/// (1971) 168), a bisection whenever the bracket has not halved over two steps
/// (Brent 1973, ch. 4), so `TEMPERATURE_MAX_EVALUATIONS` bounds it by
/// construction. Written here rather than taken out of `bubble.rs`, whose
/// arithmetic the cascade's plants are anchored on.
///
/// **The ends are read, which the halving loop never did.** A stream barely
/// above its bubble point can give `excess(low) ≥ 0`, or rounding
/// `excess(high) < 0`; a secant through two same-signed ends leaves the bracket.
/// There the root is the end the halving loop would have closed on, and that
/// end is the answer.
fn temperature_root(
    low: f64,
    high: f64,
    excess: impl Fn(f64) -> Result<f64, SimError>,
) -> Result<f64, SimError> {
    let (mut low, mut high) = (low, high);
    let (mut low_excess, mut high_excess) = (excess(low)?, excess(high)?);
    if low_excess >= 0.0 {
        return Ok(low);
    }
    if high_excess < 0.0 {
        return Ok(high);
    }
    // Which end moved last: `-1` the low, `1` the high, `0` before the first.
    let mut moved_last = 0i32;
    let (mut width_two_ago, mut width_one_ago) = (high - low, high - low);
    for _ in 0..TEMPERATURE_MAX_EVALUATIONS {
        let width = high - low;
        if width <= 2.0 * f64::EPSILON * high {
            return Ok(0.5 * (low + high));
        }
        let force_bisection = width > 0.5 * width_two_ago;
        let secant = (low * high_excess - high * low_excess) / (high_excess - low_excess);
        // The negated conjunction sends a NaN secant to the midpoint too (see
        // `bubble.rs` for why it must not be "simplified").
        let middle = if force_bisection || !(secant > low && secant < high) {
            0.5 * (low + high)
        } else {
            secant
        };
        let middle_excess = excess(middle)?;
        if middle_excess < 0.0 {
            low = middle;
            low_excess = middle_excess;
            if moved_last == -1 {
                high_excess *= 0.5;
            }
            moved_last = -1;
        } else {
            high = middle;
            high_excess = middle_excess;
            if moved_last == 1 {
                low_excess *= 0.5;
            }
            moved_last = 1;
        }
        width_two_ago = width_one_ago;
        width_one_ago = width;
    }
    Err(SimError::Numerical(format!(
        "line flash: the temperature search did not close [{low}, {high}] K within \
         {TEMPERATURE_MAX_EVALUATIONS} evaluations"
    )))
}

/// The isothermal flash a line's density reads (M56, docs/DESIGN.md §61.3):
/// `flash::flash_isothermal`'s split with its Rachford–Rice root found by
/// Newton's method inside the bracket, not by 60 halvings.
///
/// Rachford & Rice, *Trans. AIME* 195 (1952): `f(β) = Σ z(K−1)/(1 + β(K−1))`,
/// non-increasing on `[0, 1]` with `f'(β) = −Σ z(K−1)²/(1 + β(K−1))²`. A Newton
/// step that leaves the bracket, or does not halve the step before last,
/// bisects instead (`rtsafe`, Press et al., *Numerical Recipes*, §9.4). Closed
/// when `f` is within its own rounding (measured: without that test, a third
/// of the searches ground on through noise-driven bisections to 30–60
/// evaluations; with it, every measured search closes there), or when the
/// bracket is `2ε` of its top or the smallest normal `f64` wide;
/// `VAPOUR_FRACTION_MAX_EVALUATIONS` is the backstop.
///
/// **A second function, deliberately.** `flash_isothermal` is the cascade's,
/// and its arithmetic is the anchor of every column plant; changing its search
/// would move them for a cost they do not have. Its refusals are kept here word
/// for word in substance: a feed of the wrong length, a K that is not finite
/// and positive.
fn flash_on_the_line(
    feed: &MoleFractions,
    slate: &Slate,
    thermo: &dyn ThermoModel,
    temperature: Kelvin,
    pressure: Pascal,
) -> Result<FlashResult, SimError> {
    if feed.len() != slate.len() {
        return Err(SimError::Numerical(format!(
            "line flash feed has {} mole fractions for a {}-component slate",
            feed.len(),
            slate.len()
        )));
    }
    let z = feed.fractions();
    let mut k = Vec::with_capacity(slate.len());
    for c in 0..slate.len() {
        let kc = thermo.k_value(slate, c, temperature, pressure)?;
        if !kc.is_finite() || kc <= 0.0 {
            return Err(SimError::NonFiniteState {
                location: format!(
                    "thermo model '{}' returned K = {kc} for '{}' at {} K, {} Pa in a line \
                     flash; a K-value must be finite and > 0",
                    thermo.name(),
                    slate.get(c).name,
                    temperature.value(),
                    pressure.value()
                ),
            });
        }
        k.push(kc);
    }
    // Rachford–Rice, f(β) = Σ z(K−1)/(1 + β(K−1)), and its slope.
    let objective = |beta: f64| -> f64 {
        z.iter()
            .zip(&k)
            .map(|(zc, kc)| zc * (kc - 1.0) / (1.0 + beta * (kc - 1.0)))
            .sum()
    };
    // What rounding alone can leave in that sum: `n·ε·Σ|term|`, the textbook
    // bound on a recursive sum (Higham, *Accuracy and Stability of Numerical
    // Algorithms*, 2nd ed., §4.2). A value inside it is a root to an f64.
    let rounding = |beta: f64| -> f64 {
        z.len() as f64
            * f64::EPSILON
            * z.iter()
                .zip(&k)
                .map(|(zc, kc)| (zc * (kc - 1.0) / (1.0 + beta * (kc - 1.0))).abs())
                .sum::<f64>()
    };
    let slope = |beta: f64| -> f64 {
        -z.iter()
            .zip(&k)
            .map(|(zc, kc)| {
                let denominator = 1.0 + beta * (kc - 1.0);
                zc * (kc - 1.0) * (kc - 1.0) / (denominator * denominator)
            })
            .sum::<f64>()
    };
    let beta = if objective(0.0) <= 0.0 {
        // Below the bubble point, or every K = 1: all liquid (`flash_isothermal`).
        0.0
    } else if objective(1.0) >= 0.0 {
        1.0
    } else {
        let (mut low, mut high) = (0.0f64, 1.0f64);
        // `rtsafe` (Press et al., *Numerical Recipes*, 3rd ed., §9.4): the
        // step before last, against which a Newton step must at least halve.
        let (mut step_before_last, mut step) = (1.0f64, 1.0f64);
        let mut beta = 0.5;
        let mut closed = false;
        for _ in 0..VAPOUR_FRACTION_MAX_EVALUATIONS {
            let value = objective(beta);
            // At the root to rounding: a Newton step from here is noise, and
            // the safeguard would halve the bracket shut for nothing.
            if value.abs() <= rounding(beta) {
                closed = true;
                break;
            }
            if value > 0.0 {
                low = beta;
            } else {
                high = beta;
            }
            let derivative = slope(beta);
            let newton = beta - value / derivative;
            // Bisect where Newton leaves the bracket or is not at least halving
            // its steps — what keeps a slow or wandering Newton bounded.
            let next = if !(newton > low && newton < high)
                || (2.0 * value).abs() > (step_before_last * derivative).abs()
            {
                step_before_last = step;
                step = 0.5 * (high - low);
                low + step
            } else {
                step_before_last = step;
                step = beta - newton;
                newton
            };
            let width = high - low;
            let done = width <= 2.0 * f64::EPSILON * high || width <= f64::MIN_POSITIVE;
            beta = next;
            if done {
                closed = true;
                break;
            }
        }
        if !closed {
            return Err(SimError::Numerical(format!(
                "line flash at {} K, {} Pa: Rachford–Rice did not close within \
                 {VAPOUR_FRACTION_MAX_EVALUATIONS} evaluations",
                temperature.value(),
                pressure.value()
            )));
        }
        beta
    };
    let x: Vec<f64> = z
        .iter()
        .zip(&k)
        .map(|(zc, kc)| zc / (1.0 + beta * (kc - 1.0)))
        .collect();
    let y: Vec<f64> = x.iter().zip(&k).map(|(xc, kc)| xc * kc).collect();
    Ok(FlashResult {
        vapour_fraction: beta,
        liquid: MoleFractions::from_amounts(&x)?,
        vapour: MoleFractions::from_amounts(&y)?,
    })
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

#[cfg(test)]
mod search_tests {
    //! The two searches M56 put in the line flash (docs/DESIGN.md §61.3),
    //! pinned to the halving loops they replaced.

    use super::*;
    use crate::flash::flash_isothermal;
    use crate::{ConstantAlphaThermo, ConstantEnthalpy, TroutonThermo};
    use refinery_core::components::{Phase, PseudoComponent};
    use refinery_core::units::{JPerKgK, KgPerM3, KgPerMol, P_ATM};

    fn slate(n: usize) -> Slate {
        Slate::new(
            (0..n)
                .map(|i| PseudoComponent {
                    name: format!("cut{i}"),
                    tb: Kelvin(300.0 + 40.0 * i as f64),
                    molar_mass: KgPerMol(0.08 + 0.02 * i as f64),
                    density: Some(KgPerM3(650.0 + 30.0 * i as f64)),
                    cp: JPerKgK(2200.0 - 50.0 * i as f64),
                    cp_shape: None,
                    phase: Phase::Liquid,
                })
                .collect(),
        )
        .unwrap()
    }

    /// The flashing rundown's naphtha.
    fn naphtha() -> Slate {
        let cut = |name: &str, tb_c: f64, molar_mass: f64, density: f64, cp: f64| PseudoComponent {
            name: name.into(),
            tb: Kelvin(tb_c + 273.15),
            molar_mass: KgPerMol(molar_mass),
            density: Some(KgPerM3(density)),
            cp: JPerKgK(cp),
            cp_shape: None,
            phase: Phase::Liquid,
        };
        Slate::new(vec![
            cut("light_naphtha", 80.0, 0.100, 680.0, 2200.0),
            cut("heavy_naphtha", 150.0, 0.130, 750.0, 2100.0),
        ])
        .unwrap()
    }

    /// Over every regime Rachford–Rice has — a vapour fraction a hair above
    /// zero, one a hair below one, a near-pure feed, K-values six decades apart,
    /// many components — the line's Newton search lands where the cascade's 60
    /// halvings do, and so do both phases.
    #[test]
    fn the_line_flash_finds_the_cascades_root() {
        let cases: Vec<(Vec<f64>, Vec<f64>)> = vec![
            (vec![0.5, 0.5], vec![4.0, 0.25]),
            (vec![0.5, 0.5], vec![1.000_001, 0.999_998_5]),
            (vec![0.3, 0.7], vec![1.43, 0.999]),
            (vec![0.999_999, 0.000_001], vec![1.2, 0.01]),
            (vec![0.2, 0.3, 0.5], vec![1e3, 1.0, 1e-3]),
            (vec![0.9, 0.1], vec![1.05, 0.1]),
            (vec![0.01, 0.99], vec![120.0, 0.999]),
            // Newton from β = 0.5 lands at −0.83: the bracket must catch it.
            (vec![0.01, 0.99], vec![1000.0, 0.5]),
            // Newton from β = 0.5 lands at 1.44, past the pole at 1.01, and
            // walks away from the root at 0.995 unless the bracket stops it.
            (vec![0.99, 0.01], vec![3.0, 0.01]),
            (
                vec![0.05, 0.1, 0.15, 0.2, 0.2, 0.15, 0.1, 0.05],
                vec![30.0, 9.0, 3.0, 1.4, 0.8, 0.3, 0.05, 0.004],
            ),
        ];
        for (z, k) in cases {
            let s = slate(z.len());
            let thermo = ConstantAlphaThermo::new(&s, k.clone()).unwrap();
            let feed = MoleFractions::from_amounts(&z).unwrap();
            let halved = flash_isothermal(&feed, &s, &thermo, Kelvin(400.0), P_ATM).unwrap();
            let line = flash_on_the_line(&feed, &s, &thermo, Kelvin(400.0), P_ATM).unwrap();
            let (a, b) = (halved.vapour_fraction, line.vapour_fraction);
            assert!(
                (a - b).abs() <= 1e-12 * a.max(1e-300) || (a - b).abs() <= 1e-15,
                "K = {k:?}: β {b} against the halving loop's {a}"
            );
            for (phase, (x, y)) in [
                (halved.liquid.fractions(), line.liquid.fractions()),
                (halved.vapour.fractions(), line.vapour.fractions()),
            ]
            .iter()
            .enumerate()
            {
                for (p, q) in x.iter().zip(*y) {
                    assert!(
                        (p - q).abs() <= 1e-12,
                        "K = {k:?}, phase {phase}: {q} against {p}"
                    );
                }
            }
        }
    }

    /// The temperature the search settles on is the 60-halving loop's, to a
    /// nanokelvin: on the naphtha blend across the flashing rundown's range, a
    /// stream a nanokelvin over its bubble point, and the pure cut whose flash
    /// jumps at its boiling point (the bracket's low end).
    #[test]
    fn the_settled_temperature_is_the_halving_loops() {
        let s = naphtha();
        let thermo = TroutonThermo::new();
        let enthalpy = ConstantEnthalpy;
        let blend = Composition::from_weights(&[0.7, 0.3]).unwrap();
        let light = Composition::pure(2, 0);
        let mut cases: Vec<(&Composition, f64, f64)> = Vec::new();
        for t in [373.15, 388.15, 398.15, 408.15, 430.0] {
            for p in [0.9e5, 1.4e5, 2.0e5, 2.6e5] {
                cases.push((&blend, t, p));
            }
        }
        cases.push((&light, 373.15, P_ATM.value()));
        cases.push((&light, 353.150_001, P_ATM.value()));
        let feed_of = |c: &Composition| MoleFractions::from_mass(c, &s).unwrap();
        let bubble_of = |c: &Composition, p: f64| {
            bubble_temperature(&s, &thermo, &feed_of(c), Pascal(p), "a test")
                .unwrap()
                .value()
        };
        cases.push((&blend, bubble_of(&blend, 1.4e5) + 1e-9, 1.4e5));
        let mut searched = 0;
        for (composition, t_le, p) in cases {
            let bubble = bubble_of(composition, p);
            if t_le <= bubble {
                continue;
            }
            searched += 1;
            let feed = feed_of(composition);
            let target = enthalpy
                .specific_enthalpy(&s, composition, Kelvin(t_le))
                .unwrap()
                .value();
            let excess = |t: f64| -> Result<f64, SimError> {
                let split = EquilibriumLineFlash::split(&s, &feed, Kelvin(t), Pascal(p), &thermo)?;
                let sensible = enthalpy
                    .specific_enthalpy(&s, composition, Kelvin(t))?
                    .value();
                Ok(sensible + split.mass_fraction * split.latent_per_vapour - target)
            };
            let (mut low, mut high) = (bubble, t_le);
            for _ in 0..60 {
                let mid = 0.5 * (low + high);
                if excess(mid).unwrap() < 0.0 {
                    low = mid;
                } else {
                    high = mid;
                }
            }
            let halved = 0.5 * (low + high);
            let found = temperature_root(bubble, t_le, excess).unwrap();
            assert!(
                (found - halved).abs() <= 1e-9,
                "{t_le} K at {p} Pa: {found} K against the halving loop's {halved}"
            );
        }
        assert!(searched >= 20, "only {searched} cases boiled");
    }

    /// Ends of the same sign are answered by the end the halving loop would
    /// have closed on, never by a secant outside the bracket.
    #[test]
    fn a_bracket_whose_ends_agree_is_answered_by_its_end() {
        let at_low = temperature_root(400.0, 410.0, |t| Ok(t - 399.0)).unwrap();
        assert_eq!(at_low, 400.0);
        let on_low = temperature_root(400.0, 410.0, |t| Ok(t - 400.0)).unwrap();
        assert_eq!(on_low, 400.0);
        let at_high = temperature_root(400.0, 410.0, |t| Ok(t - 411.0)).unwrap();
        assert_eq!(at_high, 410.0);
        let inside = temperature_root(400.0, 410.0, |t| Ok((t - 403.0).powi(3))).unwrap();
        assert!((inside - 403.0).abs() < 1e-9, "{inside}");
    }
}
