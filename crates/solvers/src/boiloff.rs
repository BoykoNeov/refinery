//! What a liquid holdup does when it is above its own bubble point — the M12
//! fidelity seam (docs/DESIGN.md §14, `[fidelity] boiloff`).
//!
//! Two implementations of `BoilOffModel`, and neither is a refinement of the
//! other (§14 fork 9). `NoBoilOff` says a product tank never boils however hot
//! the column runs; `FlashBoilOff` vaporises the superheat and vents the vapour
//! to `Atmosphere` with nothing downstream of it, which `docs/DEFERRED.md` B12
//! and B13 already say is incomplete. A plant chooses which statement it is
//! making.
//!
//! **The trap this module is built around, named before it was written.**
//! Removing mass at the holdup's OWN composition conserves mass exactly, passes
//! I1 and I7 and every other conservation test in the workspace, and never moves
//! the tank's fractions — so the light cut that is doing the boiling stays in the
//! tank for ever. A boil-off is a flash, not a decrement, and the vapour leaves
//! at `y_c = K_c·x_c`.

use refinery_core::components::{Composition, Phase, Slate};
use refinery_core::error::SimError;
use refinery_core::traits::{BoilOff, BoilOffModel, ThermoModel};
use refinery_core::units::{JPerKg, Kelvin, Kg, Pascal};

use crate::bubble::bubble_temperature;
use crate::molar::MoleFractions;

/// A holdup never boils: the `[fidelity] boiloff = "none"` implementation, and
/// the default.
///
/// **The default because that is what a pre-M12 file MEANS**, not merely what
/// keeps one loading — the `separation` and `phase` precedent (M5.2). Every
/// scenario written before this milestone was authored by someone who never
/// considered a two-phase holdup, and fourteen of the sixteen declare a thermo
/// model that could not evaluate one anyway.
pub struct NoBoilOff;

impl BoilOffModel for NoBoilOff {
    fn name(&self) -> &'static str {
        "none"
    }

    /// Always `Ok(None)`, on every holdup, in every state.
    ///
    /// It reads no argument, and that is the point of the seam: the ARITHMETIC
    /// that differs between the two fidelities is here and in `FlashBoilOff`,
    /// not in an `if` inside `Engine::tick` (CLAUDE.md rule 2).
    fn boil_off(
        &self,
        _slate: &Slate,
        _composition: &Composition,
        _mass: Kg,
        _temperature: Kelvin,
        _pressure: Pascal,
        _thermo: &dyn ThermoModel,
    ) -> Result<Option<BoilOff>, SimError> {
        Ok(None)
    }
}

/// An equilibrium flash of the superheat: `[fidelity] boiloff = "flash"`.
///
/// The mass that boils is the mass whose latent heat absorbs the excess
/// enthalpy, and the liquid is left exactly on its bubble point:
///
/// ```text
/// f = c̄p · (T − T_bub(P, x)) / Δh̄_vap        [fraction of the holdup]
/// ```
///
/// **An enthalpy constraint, not a rate law** (§14 fork 3). A mass-transfer
/// coefficient times the superheat would need a constant nobody has, and a
/// number nothing anchors is the authoritative-looking-figure failure this
/// workspace refuses. Every quantity above already exists:
/// `Composition::mixture_cp`, `ThermoModel::dh_vap` mass-weighted, and the
/// bubble temperature M9.3a's root find already computes for a cascade stage.
pub struct FlashBoilOff;

/// The cheap question asked before the expensive one.
///
/// `ThermoModel::bubble_pressure` is a weighted sum of closed forms; the bubble
/// TEMPERATURE is a bracketed root find (~2.5 µs on the shipped five-cut slate).
/// They answer the same question in opposite directions — a liquid at `T` with
/// `P < P_bub(T)` is exactly a liquid at `P` with `T > T_bub(P)` — so the sum
/// gates the root find and a tank that is not boiling costs no solve at all.
/// That is what makes the design note's "per boiling tank per tick, not per
/// node" true rather than approximately true, and it reuses M11's criterion
/// instead of inventing a second notion of "is this boiling".
fn is_boiling(
    slate: &Slate,
    composition: &Composition,
    temperature: Kelvin,
    pressure: Pascal,
    thermo: &dyn ThermoModel,
) -> Result<bool, SimError> {
    match thermo.bubble_pressure(slate, composition, temperature) {
        Ok(bubble) => Ok(pressure.value() < bubble.value()),
        // The model has no vapour–liquid equilibrium at all — `thermo =
        // "constant"`, which is fourteen of the sixteen shipped plants. Not a
        // fault: it is the M11 arm exactly, and it means "no term here" rather
        // than "not boiling". The two are the same arithmetic downstream, which
        // is why this collapses them where `NodeSnapshot::cavitation` could not.
        Err(SimError::Scenario(_)) => Ok(false),
        Err(other) => Err(other),
    }
}

impl BoilOffModel for FlashBoilOff {
    fn name(&self) -> &'static str {
        "flash"
    }

    fn boil_off(
        &self,
        slate: &Slate,
        composition: &Composition,
        mass: Kg,
        temperature: Kelvin,
        pressure: Pascal,
        thermo: &dyn ThermoModel,
    ) -> Result<Option<BoilOff>, SimError> {
        // Spelled `is_finite() || <= 0` rather than `!(m > 0)`: both reject a
        // NaN, and only this one survives clippy's `neg_cmp_op_on_partial_ord`.
        if !mass.value().is_finite() || mass.value() <= 0.0 {
            return Ok(None);
        }
        // A vapour does not boil — it is already vapour. `Composition` is the
        // existing owner of that question (M5.2's density dispatch and M11's
        // criterion both ask it), so no second notion of phase is invented here.
        // A mixed-phase composition is an `Err` everywhere else in the engine
        // and stays one.
        if composition.phase(slate)? != Phase::Liquid {
            return Ok(None);
        }
        if !is_boiling(slate, composition, temperature, pressure, thermo)? {
            return Ok(None);
        }

        let x = MoleFractions::from_mass(composition, slate)?;
        let bubble = bubble_temperature(slate, thermo, &x, pressure, "a boiling holdup")?;

        // Δh̄_vap on a MASS basis: `dh_vap` is per mole, so each component's
        // latent heat is divided by its own molar mass before the mass-fraction
        // weighting. Evaluated AT the bubble point, which is the state the
        // vaporisation happens at rather than the superheated one it starts
        // from — the same choice the K-values below make, and for the same
        // reason: the two must describe one equilibrium or `Σ y` is not 1.
        let mut dh_vap = 0.0;
        for (c, &w) in composition.fractions().iter().enumerate() {
            if w == 0.0 {
                continue;
            }
            let molar_mass = slate.get(c).molar_mass.value();
            if !molar_mass.is_finite() || molar_mass <= 0.0 {
                return Err(SimError::Numerical(format!(
                    "component '{}' has a non-positive or non-finite molar mass \
                     ({molar_mass} kg/mol), so its latent heat per kilogram is undefined",
                    slate.get(c).name
                )));
            }
            dh_vap += w * thermo.dh_vap(slate, c, bubble)?.value() / molar_mass;
        }
        if !dh_vap.is_finite() || dh_vap <= 0.0 {
            return Err(SimError::Numerical(format!(
                "thermo model '{}' gives a boiling holdup a mass-weighted latent heat of \
                 {dh_vap} J/kg at {} K; the flash fraction c̄p·ΔT/Δh̄_vap divides by it",
                thermo.name(),
                bubble.value()
            )));
        }

        let superheat = temperature.value() - bubble.value();
        if !superheat.is_finite() || superheat <= 0.0 {
            // The two questions disagreed: `bubble_pressure` says this liquid is
            // below its bubble pressure and the root find puts its bubble
            // temperature at or above where it sits. Within a model that is
            // consistent with itself this is the resolution of the root find,
            // not a fault — so the answer is "nothing boils this tick", never a
            // negative mass.
            return Ok(None);
        }

        // **Clamped at 1, and the clamp is a specification rather than defensive
        // coding** (§14 fork 3, correction 2). Three tanks on the flipped FCC
        // plant receive a stream so far above their own bubble point that more
        // than all of what arrives would have to flash — a column drawing at
        // 800 K into a tank whose contents boil at 374 K. The constraint has no
        // solution there: there is not enough mass to absorb the arriving
        // enthalpy at the bubble point, and the honest answer is that everything
        // boils off and the tank does not fill.
        let cp = composition.mixture_cp(slate).value();
        let wanted = cp * superheat / dh_vap;
        if !wanted.is_finite() {
            return Err(SimError::Numerical(format!(
                "a boiling holdup at {} K has a non-finite flash fraction ({wanted}) against \
                 a bubble point of {} K and a latent heat of {dh_vap:.4e} J/kg",
                temperature.value(),
                bubble.value()
            )));
        }
        // **Subsumed by the per-component cap below, provably, and kept
        // anyway.** `min_c(w_c/y_c)` cannot exceed 1: `Σ w = Σ y = 1`, so no
        // mixture has every component's liquid fraction above its vapour
        // fraction. The clamp therefore never binds, and the mutation pass
        // confirms it fails nothing. It stays because it is where a reader looks
        // for §14 fork 3's specification — the honest label is "subsumed", not
        // "defensive".
        let fraction = wanted.clamp(0.0, 1.0);
        if fraction == 0.0 {
            return Ok(None);
        }

        // `y_c = K_c·x_c`, normalised. THE fork: `x` here instead — a decrement
        // at the holdup's own composition — conserves mass exactly and never
        // moves the tank's fractions, and no conservation test in this workspace
        // can see the difference.
        //
        // At the bubble point `Σ K_c·x_c = 1` by definition, so the
        // normalisation below is very nearly the identity; it is done anyway
        // because "very nearly" is the root find's resolution and a composition
        // must sum to exactly 1.
        let mut amounts = Vec::with_capacity(slate.len());
        for (c, &xc) in x.fractions().iter().enumerate() {
            amounts.push(thermo.k_value(slate, c, bubble, pressure)? * xc);
        }
        let vapour = MoleFractions::from_amounts(&amounts)?.to_mass(slate)?;

        // **The clamp at 1 is not enough, and this is a correction to §14 fork
        // 3 rather than a detail of it.** The vapour is richer in the light cuts
        // than the liquid, so `w_c·m − y_c·m_v` goes NEGATIVE for an enriched
        // component long before the total does: at `f = 1` with `y ≠ x` the
        // model would be removing more light naphtha than the tank holds while
        // its total mass balance still closed. `Composition::from_weights` would
        // refuse the leftover, so the failure is loud rather than silent — but
        // it is reachable on `f ≥ 1`, which §14's own second table says three
        // shipped-slate tanks reach.
        //
        // The cap is the largest fraction no component is over-drawn at. It
        // makes the everything-flashes case *asymptotic* rather than
        // instantaneous — the tank empties over many ticks, enriching in the
        // heavies as it goes, which is what a boiling tank does — and it binds
        // only there: the shipped demo's `f = 0.236` is well under it.
        //
        // `y_c > 0` implies `w_c > 0`, since `y = K·x` and a K is finite, so the
        // ratio below is never `0/0`.
        let mut cap = fraction;
        for (c, &y) in vapour.fractions().iter().enumerate() {
            if y > 0.0 {
                cap = cap.min(composition.fractions()[c] / y);
            }
        }
        let applied = fraction.min(cap);
        if !applied.is_finite() || applied <= 0.0 {
            return Ok(None);
        }

        // Where the whole enthalpy constraint was satisfied, the liquid is left
        // AT its bubble point — that is what the constraint says, and returning
        // the root find's own answer keeps gate 3 comparing two independently
        // computed numbers. Where a component cap cut the vaporisation short,
        // less latent heat left than the superheat asked for, and the honest
        // temperature is the one that energy allows: `m·c̄p·ΔT = m_v·Δh̄_vap`
        // over the SAME inventory the fraction is stated against. It stays above
        // the bubble point, so the holdup boils again next tick.
        let liquid_temperature = if applied >= wanted {
            bubble
        } else {
            Kelvin(temperature.value() - applied * dh_vap / cp)
        };

        Ok(Some(BoilOff {
            vapour_mass: Kg(applied * mass.value()),
            vapour,
            liquid_temperature,
            // The number this flash was SIZED against, handed on rather than
            // thrown away after the division above (M13, docs/DESIGN.md §15
            // fork 2). Recomputing it downstream has three plausible spellings
            // and all of them leave a residual; this has one.
            latent_heat: JPerKg(dh_vap),
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::thermo::TroutonThermo;
    use crate::ConstantThermo;
    use refinery_core::components::{PseudoComponent, Slate};
    use refinery_core::units::{JPerKgK, KgPerM3, KgPerMol};

    /// The M7.3/M12 five-cut crude slate, the one `crude_column_cascade.toml`
    /// declares.
    fn crude_slate() -> Slate {
        Slate::new(vec![
            PseudoComponent {
                name: "light_naphtha".into(),
                tb: Kelvin(353.15),
                molar_mass: KgPerMol(0.100),
                density: Some(KgPerM3(680.0)),
                cp: JPerKgK(2200.0),
                phase: Phase::Liquid,
            },
            PseudoComponent {
                name: "heavy_naphtha".into(),
                tb: Kelvin(423.15),
                molar_mass: KgPerMol(0.130),
                density: Some(KgPerM3(750.0)),
                cp: JPerKgK(2100.0),
                phase: Phase::Liquid,
            },
        ])
        .expect("two-cut slate")
    }

    #[test]
    fn the_none_fidelity_boils_nothing_however_hot() {
        let slate = crude_slate();
        let hot = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let out = NoBoilOff
            .boil_off(
                &slate,
                &hot,
                Kg(1000.0),
                Kelvin(900.0),
                Pascal(101_325.0),
                &TroutonThermo::new(),
            )
            .expect("no boil-off cannot fail");
        assert!(
            out.is_none(),
            "the 'none' fidelity returned a boil-off; it is the model that says a product \
             tank never boils"
        );
    }

    /// A model with no vapour–liquid equilibrium reports NO TERM, not a term of
    /// zero — the arm fourteen of the sixteen shipped plants are on.
    #[test]
    fn a_thermo_model_that_cannot_answer_gives_no_term() {
        let slate = crude_slate();
        let hot = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let out = FlashBoilOff
            .boil_off(
                &slate,
                &hot,
                Kg(1000.0),
                Kelvin(900.0),
                Pascal(101_325.0),
                &ConstantThermo,
            )
            .expect("a model that cannot answer is not a fault");
        assert!(
            out.is_none(),
            "`thermo = \"constant\"` produced a boil-off; its k_value, dh_vap and \
             bubble_pressure are all `Err` by design, so there is nothing to compute"
        );
    }

    /// The composition of what leaves is NOT the composition of what stays —
    /// the one property that separates a flash from a decrement, asserted here
    /// on the model itself and again on a wired plant.
    #[test]
    fn the_vapour_is_richer_in_the_light_cut_than_the_liquid() {
        let slate = crude_slate();
        let liquid = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let out = FlashBoilOff
            .boil_off(
                &slate,
                &liquid,
                Kg(1000.0),
                Kelvin(420.0),
                Pascal(101_325.0),
                &TroutonThermo::new(),
            )
            .expect("trouton answers")
            .expect("a 50/50 naphtha mix at 420 K and one atmosphere is boiling");
        assert!(
            out.vapour.fractions()[0] > liquid.fractions()[0],
            "the vapour ({:.4}/{:.4}) is not richer in light naphtha than the liquid it \
             left ({:.4}/{:.4}) — that is a decrement at `x`, not a flash at `y = K·x`",
            out.vapour.fractions()[0],
            out.vapour.fractions()[1],
            liquid.fractions()[0],
            liquid.fractions()[1],
        );
    }

    /// A liquid BELOW its bubble point has no term, and the cheap gate is what
    /// says so — the root find is never reached.
    #[test]
    fn a_cold_holdup_has_no_term() {
        let slate = crude_slate();
        let cold = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let out = FlashBoilOff
            .boil_off(
                &slate,
                &cold,
                Kg(1000.0),
                Kelvin(300.0),
                Pascal(101_325.0),
                &TroutonThermo::new(),
            )
            .expect("trouton answers");
        assert!(
            out.is_none(),
            "a naphtha mix at 300 K and one atmosphere is 60 K below its bubble point and \
             boiled anyway"
        );
    }

    /// **The everything-flashes case, written before the happy path.** A holdup
    /// far enough above its bubble point that `c̄p·ΔT` exceeds its own latent
    /// heat asks for more mass than it has — and asks for more of the LIGHT cut
    /// than it has well before that, because the vapour is enriched.
    ///
    /// The property asserted is the one the engine depends on: every component's
    /// leftover is non-negative, so the subtraction that follows a boil-off
    /// cannot produce a composition `Composition::from_weights` refuses. Two
    /// controls come first, because without them this passes on a model that
    /// boils nothing at all.
    #[test]
    fn a_wildly_superheated_holdup_never_over_draws_a_component() {
        let slate = crude_slate();
        let liquid = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let mass = Kg(1000.0);
        let out = FlashBoilOff
            .boil_off(
                &slate,
                &liquid,
                mass,
                Kelvin(800.0),
                Pascal(101_325.0),
                &TroutonThermo::new(),
            )
            .expect("trouton answers")
            .expect("400 K of superheat boils");

        // Control 1: the fixture is genuinely in the everything-flashes regime —
        // the enthalpy constraint alone asks for more than the whole inventory,
        // so the caps below are being exercised rather than sitting inert.
        let unclamped =
            liquid.mixture_cp(&slate).value() * (800.0 - 353.0) / (0.5 * 3.9e5 + 0.5 * 3.4e5);
        assert!(
            unclamped > 1.5,
            "the fixture is no longer in the everything-flashes regime (unclamped fraction \
             {unclamped:.3}), so nothing here is being capped"
        );
        // Control 2: something actually boiled. Every assertion below is passed
        // by a model that returns a vapour mass of zero.
        assert!(
            out.vapour_mass.value() > 0.1 * mass.value(),
            "only {:.4e} kg of a {} kg holdup boiled at 400 K of superheat",
            out.vapour_mass.value(),
            mass.value()
        );

        for (c, &y) in out.vapour.fractions().iter().enumerate() {
            let held = liquid.fractions()[c] * mass.value();
            let left = held - y * out.vapour_mass.value();
            assert!(
                left >= 0.0,
                "component '{}' is over-drawn: the holdup holds {held:.4e} kg and the flash \
                 takes {:.4e} kg of it. A total clamp at f = 1 does not prevent this — the \
                 vapour is enriched, so an enriched component runs out first",
                slate.get(c).name,
                y * out.vapour_mass.value()
            );
        }
        // And the liquid left behind is still superheated, because the cap cut
        // the vaporisation short of what the enthalpy balance asked for. A
        // holdup parked at its bubble point here would mean energy was destroyed
        // rather than carried out.
        assert!(
            out.liquid_temperature.value() > 400.0,
            "the capped flash left the liquid at {} K, at or below the bubble point it \
             could not have reached with the mass it was allowed to boil",
            out.liquid_temperature.value()
        );
    }
}
