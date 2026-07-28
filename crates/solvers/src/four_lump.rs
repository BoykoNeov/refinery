//! `FourLump`: the complex-fidelity reactor — FCC 4-lump Arrhenius kinetics
//! integrated with fixed-count RK4 over the residence time. Selected by
//! `reactions = "fcc"`; it swaps in behind the same `ReactionModel` trait as
//! `SimpleLookup` with no change above it (M4.2).
//!
//! # The reaction network
//!
//! The classical four-lump scheme (gas oil, gasoline, light gases, coke):
//!
//! ```text
//!                  k2
//!        ┌──────────────────────► light gases ◄───────┐
//!        │                                            │ k4
//!   gas oil ──────k1──────► gasoline ─────────────────┤
//!        │                       │ k5                 │
//!        └──────────k3──────────►└──────────────► coke
//! ```
//!
//! **Gas oil cracking is second order in its mass fraction; gasoline cracking is
//! first order.** Both facts are stated explicitly by Olufemi, Latinwo &
//! Olukayode, *Riser Reactor Simulation in a Fluid Catalytic Cracking Unit*,
//! Chemical and Process Engineering Research 7 (2013) 12–21, §"Cracking reaction
//! kinetics", and independently by Bunny, Pathak, Arya & Dutt, *Optimization of
//! Gasoline Yield in FCC riser unit Using RSM*, RJPBCS 6(4) (2015) 1269, which
//! also gives the network as Fig. 2. (Olufemi's printed eqs. (15)–(16) put the
//! gasoline paths at `x₃²`, which contradicts that same paper's stated
//! first-order assumption and Bunny's rate expressions — it is a typo in the
//! paper, and this module uses first order in the gasoline fraction.)
//!
//! Catalyst activity decays as `φ(t) = exp(−α·t)` (Weekman's exponential decay,
//! as used by Olufemi et al. eq. 17). The consequence that makes this model
//! analytically checkable: φ depends only on time, so it is a pure
//! **reparametrization** of the residence coordinate,
//! `θ(t) = ∫₀ᵗ φ = (1 − e^{−α t})/α`, and every rate law integrates in θ exactly
//! as it would in t without decay. `tests/reference/four_lump.rs` uses that to
//! pin the kinetics against closed forms.
//!
//! # Units, and the trap this milestone invites
//!
//! Published FCC rate constants are usually per unit **catalyst mass** (the riser
//! models multiply by catalyst holdup and density) or are quoted against *space
//! time* in hours rather than riser residence time in seconds. Dropping such a
//! constant into a `τ = 3 s` residence time gives a conversion wrong by orders of
//! magnitude that still converges, still conserves total mass, and still reruns
//! bit-identically.
//!
//! This model therefore states its convention once, here: **`k₁…k₃` are in
//! (mass fraction · second)⁻¹ and `k₄`, `k₅` in second⁻¹, against the reactor's
//! `tau` in seconds, with the catalyst loading folded in** at the reference
//! catalyst-to-oil ratio of the plant the constants were calibrated against
//! (COR ≈ 6 kg/kg). The reactor node models no catalyst inventory
//! (DESIGN §5 defers the regenerator loop), so a separate COR term would have no
//! second value to take and nothing that could falsify it.
//!
//! # Where the numbers come from — calibrated, not transcribed
//!
//! The constants in [`FourLumpParams::fcc`] are **calibrated** to reproduce the
//! published industrial-riser product distribution within its spread, NOT
//! transcribed from a published parameter table. Every source that tabulates
//! `k₁…k₅` for this network sits behind a paywall that could not be read, and
//! transcribing constants from a search summary is exactly the silent-wrong-number
//! failure this workspace refuses to ship.
//!
//! The anchor that IS published, and that `tests/reference/four_lump.rs` gates
//! against, is the plant data reproduced by Olufemi et al. (2013) Tables 1–4 from
//! Ali & Rohani (1997): four industrial cases at riser outlet 795–808 K with
//! catalyst-to-oil 5.43–7.20, giving gasoline 41.78–46.90 wt%, coke 5.34–5.83
//! wt%, and 79 wt% gas oil conversion. That is a coarser anchor than a point
//! match against a tabulated `k` set — it pins the model to an envelope, which
//! is enough to falsify a residence-time or catalyst-loading unit slip (those miss
//! by decades, not percent) and is honest about catching nothing subtler. The
//! closed-form gate covers the subtler faults.

use refinery_core::components::{Composition, Slate};
use refinery_core::error::SimError;
use refinery_core::traits::{Reaction, ReactionModel};
use refinery_core::units::{JPerKg, Kelvin, Seconds};

/// Universal gas constant [J/(mol·K)].
const R_GAS: f64 = 8.314_462_618;

/// Reference temperature for the rate constants [K] — near the riser outlet
/// temperature of the plant cases the set is calibrated against.
pub const T_REF_K: f64 = 800.0;

/// RK4 substeps per `react` call. **Fixed count, never adaptive**: an adaptive
/// controller would make the product composition depend on floating-point
/// comparisons whose outcome can differ between runs, breaking bit-identical
/// reruns (CLAUDE.md rule 3). 64 steps over a few seconds of residence puts the
/// integration error far below the model's own accuracy — measured in
/// `tests/reference/four_lump.rs`.
pub const RK4_SUBSTEPS: usize = 64;

/// The kinetic parameter set: five rate constants at [`T_REF_K`] with their
/// activation energies, the catalyst decay coefficient, and the lump formation
/// enthalpies that give the pass its heat of reaction.
///
/// Rate constant units follow the module note: `k1..k3` (gas oil, second order)
/// are (mass fraction · s)⁻¹, `k4`/`k5` (gasoline, first order) are s⁻¹.
#[derive(Debug, Clone)]
pub struct FourLumpParams {
    /// Rate constants at [`T_REF_K`], in path order:
    /// `[gasoil→gasoline, gasoil→gas, gasoil→coke, gasoline→gas, gasoline→coke]`.
    pub k_ref: [f64; 5],
    /// Activation energies [J/mol] in the same path order. Applied as
    /// `k(T) = k_ref · exp(−E/R · (1/T − 1/T_ref))`, the reference-temperature
    /// form used by Bunny et al. (2015) (their eq. with `k_{j,756K}`), which is
    /// algebraically the Arrhenius law with the pre-exponential absorbed into
    /// the reference rate.
    pub e_act: [f64; 5],
    /// Catalyst decay coefficient at [`T_REF_K`] [s⁻¹] in `φ = exp(−α·t)`.
    pub alpha_ref: f64,
    /// Activation energy of the decay coefficient [J/mol].
    pub e_alpha: f64,
    /// Lump formation enthalpies [J/kg], gas oil datum = 0, in lump order
    /// `[gasoil, gasoline, gas, coke]`. Only DIFFERENCES matter, so the datum is
    /// free; the pass's heat of reaction is `Σ_i (y_out,i − y_in,i)·h_f,i`, which
    /// is path-independent (a state function) and identically zero when nothing
    /// reacts. Positive = endothermic per `Reaction::dh_rxn`, so cracking to
    /// lighter lumps must give the products the HIGHER formation enthalpy.
    pub h_form: [f64; 4],
}

impl FourLumpParams {
    /// The calibrated FCC set — see the module note on provenance: these are
    /// **calibrated to land inside the published Ali & Rohani plant envelope**
    /// (gasoline 41.8–46.9 wt%, coke 5.3–5.8 wt%, 79 wt% conversion at riser
    /// outlet ~800 K), not transcribed from a published table.
    ///
    /// The activation-energy spread is a modelling choice within the 30–70 kJ/mol
    /// band typical of lumped FCC kinetics, ordered to give the conventional
    /// selectivity shift with riser outlet temperature: the light-gas paths carry
    /// the highest activation energies and the coke path the lowest, so raising
    /// the setpoint moves yield from coke and gasoline toward light gases.
    ///
    /// The formation enthalpies are likewise calibrated, to put the overall
    /// endotherm of a full-conversion pass at a few hundred kJ per kg of feed —
    /// the magnitude quoted for FCC cracking by Sadeghbeigi, *Fluid Catalytic
    /// Cracking Handbook*. Per-lump formation enthalpies for "light gases" and
    /// "coke" are not published quantities; these are chosen numbers with a
    /// published SUM, which is the honest way to state it.
    pub fn fcc() -> Self {
        Self {
            k_ref: [1.120, 0.284, 0.090, 0.155, 0.010],
            e_act: [55.0e3, 70.0e3, 35.0e3, 65.0e3, 60.0e3],
            alpha_ref: 0.120,
            e_alpha: 45.0e3,
            h_form: [0.0, 3.0e5, 8.0e5, -3.0e5],
        }
    }

    /// The rate constants and decay coefficient evaluated at `temperature`.
    fn at(&self, temperature: Kelvin) -> ([f64; 5], f64) {
        // Arrhenius about the reference temperature:
        // k(T) = k_ref · exp(−E/R · (1/T − 1/T_ref)).
        let shift = |e: f64| (-e / R_GAS * (1.0 / temperature.value() - 1.0 / T_REF_K)).exp();
        let mut k = [0.0; 5];
        for (i, k_i) in k.iter_mut().enumerate() {
            *k_i = self.k_ref[i] * shift(self.e_act[i]);
        }
        (k, self.alpha_ref * shift(self.e_alpha))
    }
}

/// Slate positions of the four kinetic lumps, resolved by NAME at construction
/// (DESIGN §5 fork 1: the lumps ARE slate members, like column draws and
/// exchanger sides). Order is the model's own, NOT the slate's.
#[derive(Debug, Clone, Copy)]
struct LumpIndices {
    gasoil: usize,
    gasoline: usize,
    gas: usize,
    coke: usize,
}

/// FCC 4-lump Arrhenius kinetics (complex fidelity).
pub struct FourLump {
    params: FourLumpParams,
    lumps: LumpIndices,
    substeps: usize,
}

/// The four lump mass fractions carried through the integration, in the model's
/// own order: gas oil, gasoline, light gases, coke.
type Lumps = [f64; 4];

impl FourLump {
    /// Build the model with the calibrated FCC parameter set, resolving the four
    /// lumps against `slate` by name.
    ///
    /// # Errors
    /// `SimError::Scenario` if the slate lacks any of `gasoil`, `gasoline`,
    /// `gas`, `coke`.
    pub fn fcc(slate: &Slate) -> Result<Self, SimError> {
        Self::with_params(slate, FourLumpParams::fcc())
    }

    /// Build the model with an explicit parameter set. Exists so the reference
    /// tests can drive the integrator with constants whose closed-form solution
    /// is elementary (e.g. no gasoline cracking), which the calibrated set's
    /// solution is not.
    ///
    /// # Errors
    /// `SimError::Scenario` if the slate lacks a named lump, or if any parameter
    /// is negative or non-finite — a negative rate constant would run a cracking
    /// path backwards and a negative decay coefficient would make the catalyst
    /// gain activity, both of which are meaningless rather than merely extreme.
    pub fn with_params(slate: &Slate, params: FourLumpParams) -> Result<Self, SimError> {
        Self::with_substeps(slate, params, RK4_SUBSTEPS)
    }

    /// As [`FourLump::with_params`], with the RK4 substep count given explicitly.
    ///
    /// The count is an INTEGRATOR setting, not a kinetic parameter, and it is
    /// exposed for one reason: it lets a test *measure* the truncation error and
    /// its order of convergence against the closed-form solution, instead of
    /// asserting against the integrator's own arithmetic — which would agree
    /// with any RK4 implementation of any wrong rate law. Scenarios always get
    /// [`RK4_SUBSTEPS`], so the shipped plant stays deterministic.
    ///
    /// # Errors
    /// `SimError::Scenario` if `substeps` is zero (no integration would happen)
    /// or if any parameter is invalid, as [`FourLump::with_params`].
    pub fn with_substeps(
        slate: &Slate,
        params: FourLumpParams,
        substeps: usize,
    ) -> Result<Self, SimError> {
        if substeps == 0 {
            return Err(SimError::Scenario(
                "4-lump RK4 substep count must be at least 1".into(),
            ));
        }
        let index = |name: &str| -> Result<usize, SimError> {
            slate.index_of(name).ok_or_else(|| {
                SimError::Scenario(format!(
                    "reactions = \"fcc\" (4-lump kinetics) needs a '{name}' lump in the slate; \
                     the model cracks gasoil into gasoline, gas and coke"
                ))
            })
        };
        let lumps = LumpIndices {
            gasoil: index("gasoil")?,
            gasoline: index("gasoline")?,
            gas: index("gas")?,
            coke: index("coke")?,
        };
        for (i, k) in params.k_ref.iter().enumerate() {
            if !k.is_finite() || *k < 0.0 {
                return Err(SimError::Scenario(format!(
                    "4-lump rate constant k{} = {k} must be finite and >= 0",
                    i + 1
                )));
            }
        }
        if !params.alpha_ref.is_finite() || params.alpha_ref < 0.0 {
            return Err(SimError::Scenario(format!(
                "4-lump catalyst decay coefficient alpha = {} must be finite and >= 0",
                params.alpha_ref
            )));
        }
        if params
            .e_act
            .iter()
            .chain(&params.h_form)
            .any(|v| !v.is_finite())
            || !params.e_alpha.is_finite()
        {
            return Err(SimError::Scenario(
                "4-lump activation energies and formation enthalpies must be finite".into(),
            ));
        }
        Ok(Self {
            params,
            lumps,
            substeps,
        })
    }

    /// The lump derivatives `dy/dt` at contact time `t`.
    ///
    /// Gas oil cracks second order in its own mass fraction, gasoline first order
    /// (see the module note); every term leaves one lump and enters another, so
    /// the four derivatives sum to zero identically — that is the reactor's
    /// load-bearing mass conservation, the kinetic analog of `SimpleLookup`'s row
    /// renormalization, and it holds for ANY parameter set including a nonsense
    /// one.
    fn derivatives(y: &Lumps, k: &[f64; 5], alpha: f64, t: f64) -> Lumps {
        let phi = (-alpha * t).exp(); // Weekman exponential catalyst decay
        let (gasoil, gasoline) = (y[0], y[1]);
        let go2 = gasoil * gasoil;
        // Paths, in the order of `FourLumpParams::k_ref`.
        let to_gasoline = k[0] * go2;
        let gasoil_to_gas = k[1] * go2;
        let gasoil_to_coke = k[2] * go2;
        let gasoline_to_gas = k[3] * gasoline;
        let gasoline_to_coke = k[4] * gasoline;
        [
            -phi * (to_gasoline + gasoil_to_gas + gasoil_to_coke),
            phi * (to_gasoline - gasoline_to_gas - gasoline_to_coke),
            phi * (gasoil_to_gas + gasoline_to_gas),
            phi * (gasoil_to_coke + gasoline_to_coke),
        ]
    }

    /// Integrate the lump ODEs over `[0, tau]` with this model's fixed substep
    /// count ([`RK4_SUBSTEPS`] unless built by [`FourLump::with_substeps`]).
    fn integrate(&self, mut y: Lumps, k: &[f64; 5], alpha: f64, tau: Seconds) -> Lumps {
        let h = tau.value() / self.substeps as f64;
        let mut t = 0.0;
        for _ in 0..self.substeps {
            let a = Self::derivatives(&y, k, alpha, t);
            let b = Self::derivatives(&step(&y, &a, h * 0.5), k, alpha, t + h * 0.5);
            let c = Self::derivatives(&step(&y, &b, h * 0.5), k, alpha, t + h * 0.5);
            let d = Self::derivatives(&step(&y, &c, h), k, alpha, t + h);
            for i in 0..4 {
                y[i] += h / 6.0 * (a[i] + 2.0 * b[i] + 2.0 * c[i] + d[i]);
            }
            t += h;
        }
        y
    }
}

/// `y + h·dy`, the RK4 stage state.
fn step(y: &Lumps, dy: &Lumps, h: f64) -> Lumps {
    [
        y[0] + h * dy[0],
        y[1] + h * dy[1],
        y[2] + h * dy[2],
        y[3] + h * dy[3],
    ]
}

impl ReactionModel for FourLump {
    fn name(&self) -> &'static str {
        "fcc"
    }

    fn react(
        &self,
        feed: &Composition,
        temperature: Kelvin,
        tau: Seconds,
        _slate: &Slate,
    ) -> Result<Reaction, SimError> {
        if !tau.value().is_finite() || tau.value() < 0.0 {
            return Err(SimError::Numerical(format!(
                "reactor residence time tau = {} s must be finite and >= 0",
                tau.value()
            )));
        }
        let f = feed.fractions();
        let (k, alpha) = self.params.at(temperature);
        let before: Lumps = [
            f[self.lumps.gasoil],
            f[self.lumps.gasoline],
            f[self.lumps.gas],
            f[self.lumps.coke],
        ];
        let after = self.integrate(before, &k, alpha, tau);

        // The reaction moves mass only BETWEEN the four lumps: any other slate
        // member is inert here and keeps its fraction. Rates are written in
        // overall mass fractions, so an inert simply dilutes the reacting mixture
        // — a stated modelling choice, not an oversight.
        let mut weights = f.to_vec();
        weights[self.lumps.gasoil] = after[0];
        weights[self.lumps.gasoline] = after[1];
        weights[self.lumps.gas] = after[2];
        weights[self.lumps.coke] = after[3];

        // The reactor total-mass gate, in code. `Composition::from_weights`
        // renormalizes, which would SILENTLY absorb a rate matrix whose columns
        // do not sum to zero, so the sum is checked BEFORE normalizing rather
        // than trusted afterwards. The tolerance is RK4 round-off on ~1.0.
        let sum: f64 = weights.iter().sum();
        if !sum.is_finite() || (sum - 1.0).abs() > 1e-9 {
            return Err(SimError::Numerical(format!(
                "4-lump reaction did not conserve total mass: product fractions sum to {sum}, \
                 expected 1 — the rate network's columns must sum to zero"
            )));
        }
        if let Some(negative) = after.iter().find(|v| **v < 0.0) {
            return Err(SimError::Numerical(format!(
                "4-lump reaction produced a negative lump fraction ({negative}); the residence \
                 time or rate constants are outside the range the integrator can hold"
            )));
        }
        let products = Composition::from_weights(&weights).map_err(|e| {
            SimError::Numerical(format!(
                "4-lump reactor products are not a composition: {e}"
            ))
        })?;

        // Heat of reaction from lump formation enthalpies: path-independent, and
        // identically zero when nothing reacted (tau = 0 or an inert feed).
        let dh_rxn: f64 = (0..4)
            .map(|i| (after[i] - before[i]) * self.params.h_form[i])
            .sum();
        Ok(Reaction {
            products,
            dh_rxn: JPerKg(dh_rxn),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use refinery_core::components::PseudoComponent;
    use refinery_core::units::{JPerKgK, KgPerM3, KgPerMol};

    /// Slate order deliberately differs from the model's lump order, so a
    /// positional mix-up cannot pass by coincidence.
    fn fcc_slate() -> Slate {
        slate_of(&["gas", "gasoline", "gasoil", "coke"])
    }

    fn slate_of(names: &[&str]) -> Slate {
        Slate::new(
            names
                .iter()
                .map(|n| PseudoComponent {
                    name: (*n).into(),
                    tb: Kelvin(500.0),
                    molar_mass: KgPerMol(0.1),
                    density: KgPerM3(800.0),
                    cp: JPerKgK(2000.0),
                })
                .collect(),
        )
        .expect("test slate")
    }

    /// Cracking must come out ENDOTHERMIC — positive `dh_rxn` per the trait's
    /// sign convention. The formation enthalpies are free to have any datum, so
    /// the sign is a property of their DIFFERENCES; getting them backwards would
    /// make the reactor report a duty that heats the plant while it cracks, and
    /// `energy::reactor_duty` would propagate that sign faithfully.
    #[test]
    fn cracking_is_endothermic() {
        let slate = fcc_slate();
        let model = FourLump::fcc(&slate).expect("model");
        let out = model
            .react(
                &Composition::pure(4, 2), // pure gasoil
                Kelvin(T_REF_K),
                Seconds(3.0),
                &slate,
            )
            .expect("react");
        assert!(
            out.dh_rxn.value() > 0.0,
            "FCC cracking absorbs heat; dh_rxn must be positive (endothermic), got {}",
            out.dh_rxn.value()
        );
    }

    /// A pass with no residence time changes nothing and costs nothing. This is
    /// the identity limit of the integrator, and it separates the two things a
    /// zero reading could mean: `dh_rxn` is zero here because the COMPOSITION did
    /// not move, not because the formation enthalpies cancel.
    #[test]
    fn a_zero_residence_time_is_the_identity() {
        let slate = fcc_slate();
        let model = FourLump::fcc(&slate).expect("model");
        let feed = Composition::from_weights(&[0.05, 0.2, 0.7, 0.05]).expect("feed");
        let out = model
            .react(&feed, Kelvin(T_REF_K), Seconds(0.0), &slate)
            .expect("react");
        for (got, want) in out.products.fractions().iter().zip(feed.fractions()) {
            assert!((got - want).abs() < 1e-15, "{got} != {want}");
        }
        assert_eq!(out.dh_rxn.value(), 0.0);
    }

    /// Total mass is conserved for a MIXED feed — every lump present, so every
    /// path in the network carries flux at once and the rate matrix's columns
    /// have to sum to zero for the books to balance.
    ///
    /// **Asserting that the products sum to 1 would be VACUOUS**, and this test
    /// said so before it said anything else: `Composition::from_weights`
    /// normalizes, so that total is 1 whatever the kinetics did. Two things are
    /// asserted instead, and both are falsifiable. First, `react` must SUCCEED —
    /// the real guard is the pre-normalization sum check inside it, and a leaking
    /// column makes that return `Err` rather than a plausible composition.
    /// Second, on a slate carrying an INERT the four lumps' combined fraction is
    /// a genuinely free quantity: a leak changes it even after normalization,
    /// because the inert's share moves to absorb the difference.
    #[test]
    fn every_path_active_still_conserves_total_mass() {
        let slate = slate_of(&["gas", "gasoline", "gasoil", "coke", "steam"]);
        let model = FourLump::fcc(&slate).expect("model");
        let feed = Composition::from_weights(&[0.08, 0.2, 0.5, 0.02, 0.2]).expect("feed");
        let lump_mass = |c: &Composition| c.fractions()[..4].iter().sum::<f64>();

        let out = model
            .react(&feed, Kelvin(830.0), Seconds(4.0), &slate)
            .expect("a conserving network must react without error");
        assert!(
            (lump_mass(&out.products) - lump_mass(&feed)).abs() < 1e-12,
            "the reacting lumps must keep their combined share of a slate that also \
             carries an inert: {} in vs {} out",
            lump_mass(&feed),
            lump_mass(&out.products)
        );
        // And the reaction actually ran, so the conservation above is not the
        // trivial one an identity model would also pass.
        assert!(
            out.products.fractions()[2] < 0.5 - 1e-3,
            "gasoil must have been consumed"
        );
    }

    /// A slate member the kinetics do not name is INERT: it keeps its fraction
    /// exactly. Stated as a modelling choice in the module note (rates are in
    /// overall mass fractions, so an inert dilutes the mixture), and worth a gate
    /// because the alternative — renormalizing the four lumps to 1 and losing the
    /// inert — would conserve nothing and still produce a valid composition.
    #[test]
    fn an_unnamed_slate_member_passes_through_untouched() {
        let slate = slate_of(&["gas", "gasoline", "gasoil", "coke", "steam"]);
        let model = FourLump::fcc(&slate).expect("model");
        let feed = Composition::from_weights(&[0.0, 0.0, 0.7, 0.0, 0.3]).expect("feed");
        let out = model
            .react(&feed, Kelvin(T_REF_K), Seconds(3.0), &slate)
            .expect("react");
        let y = out.products.fractions();
        assert!(
            (y[4] - 0.3).abs() < 1e-12,
            "the inert steam fraction must be untouched, got {}",
            y[4]
        );
        assert!(y[2] < 0.7 - 1e-3, "the gasoil must still have cracked");
    }

    /// The lumps are resolved by NAME at construction, so a slate that cannot
    /// express the reaction is a load-time error rather than a solve-time
    /// surprise — the same contract `SimpleLookup::fcc_demo` has.
    #[test]
    fn a_slate_missing_a_lump_is_refused() {
        let missing = slate_of(&["gas", "gasoline", "gasoil"]); // no coke
        assert!(FourLump::fcc(&missing).is_err());
    }

    /// Parameters that are meaningless rather than merely extreme are refused:
    /// a negative rate constant runs a cracking path backwards, a negative decay
    /// coefficient makes the catalyst gain activity with time, and zero substeps
    /// integrates nothing at all while returning a plausible-looking feed.
    #[test]
    fn meaningless_parameters_are_refused() {
        let slate = fcc_slate();
        let mut negative_rate = FourLumpParams::fcc();
        negative_rate.k_ref[1] = -0.1;
        assert!(FourLump::with_params(&slate, negative_rate).is_err());

        let mut growing_catalyst = FourLumpParams::fcc();
        growing_catalyst.alpha_ref = -0.05;
        assert!(FourLump::with_params(&slate, growing_catalyst).is_err());

        assert!(FourLump::with_substeps(&slate, FourLumpParams::fcc(), 0).is_err());
    }

    /// Catalyst decay must SLOW the conversion. `alpha` enters only through
    /// `φ = exp(−α t)`, and a sign slip there would still give a monotone
    /// composition trajectory, still conserve mass, and still land somewhere
    /// plausible — it would just crack more, not less, as the catalyst aged.
    #[test]
    fn catalyst_decay_reduces_conversion() {
        let slate = fcc_slate();
        let feed = Composition::pure(4, 2);
        let convert = |alpha: f64| {
            let params = FourLumpParams {
                alpha_ref: alpha,
                ..FourLumpParams::fcc()
            };
            let model = FourLump::with_params(&slate, params).expect("model");
            let out = model
                .react(&feed, Kelvin(T_REF_K), Seconds(3.0), &slate)
                .expect("react");
            1.0 - out.products.fractions()[2]
        };
        assert!(
            convert(0.4) < convert(0.0),
            "a faster-decaying catalyst must convert LESS gas oil in the same residence time"
        );
    }
}
