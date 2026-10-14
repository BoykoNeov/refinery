//! `Hydrocracking`: four-lump first-order VGO hydrocracking, solved exactly at
//! every residence time. Selected by `reactions = "hydrocracking"` (M57.2,
//! docs/DESIGN.md §62.3); the catalyst bed's chemistry, and the reason a bed can
//! run away: it RELEASES heat, faster the hotter it gets.
//!
//! # The reaction network
//!
//! The four-lump network of Sadighi, Ahmad & Rashidzadeh, *Korean J. Chem. Eng.*
//! 27 (2010) 1099–1108, restated by Faraji et al., *ACS Omega* 5 (2020)
//! 16595–16601: vacuum gas oil cracks to distillate, naphtha and gas; distillate
//! to naphtha and gas; naphtha to gas. Six paths:
//!
//! ```text
//!   vgo ──k1──► distillate ──k4──► naphtha ──k6──► gas
//!    │ └────────────k2────────────►  ▲               ▲
//!    │                distillate ─────────k5─────────┤
//!    └──────────────────────k3───────────────────────┘
//! ```
//!
//! **First order in each lump, hydrogen in excess** so no rate reads it
//! (Faraji 2020, §3 assumption 3; Song et al., *Chem. Eng. Trans.* 61 (2017)
//! 673–678, §2.1 assumption 2). Rates are written in mass fractions against the
//! space time `τ` [s], `1/LHSV`, with the catalyst folded into the constant —
//! the convention fixed-bed kinetics are quoted in.
//!
//! # Solved exactly — the Bateman chains
//!
//! A first-order network is linear, `dy/dτ = K·y`, and this one is a chain with
//! no loops (every path goes lighter), the shape of a radioactive decay chain.
//! Its solution is a sum over the CHAINS that reach each lump (Bateman, *Proc.
//! Camb. Phil. Soc.* 15 (1910) 423): a chain `i₀ → i₁ → … → iₘ` carries
//!
//! ```text
//!   y_{i₀}(0) · k(i₀→i₁)·…·k(iₘ₋₁→iₘ) · (−1)ᵐ · f[λ₀, …, λₘ],   f(λ) = e^(−λτ)
//! ```
//!
//! into `iₘ`, with `λⱼ` the total cracking rate out of `iⱼ` (zero for gas) and
//! `f[…]` the divided difference of `e^(−λτ)` over those rates. Every lump —
//! gas included — is its own sum, so the lumps' total is NOT conserved by
//! construction, and the check in `react` is a real guard on the arithmetic.
//!
//! **The divided differences are taken stably.** The two-rate one is written
//! with no subtraction of nearby exponentials, `f[a, b] = −e^(−aτ)·(1 −
//! e^(−(b−a)τ))/(b − a)` for `a ≤ b`, through `expm1`, so it is exact at equal
//! rates (`−τ·e^(−aτ)`) and at small `τ`. Higher orders recurse on the rates
//! sorted, dividing by the widest span; that span vanishes only if all three
//! cracking lumps share one total rate, which the constructor refuses. With ONE
//! activation energy (below) the rates' ratios are the same at every
//! temperature, so that refusal holds for every bed temperature at once.
//!
//! **At `τ = +∞`** (a bed with no flow, §62.2) every chain's limit is closed:
//! `Π 1/λⱼ` over its non-final lumps when it ends in a lump that does not crack,
//! zero otherwise. With every lump cracking, everything ends as gas.
//!
//! The work per call is fixed: fifteen chains, none longer than four lumps
//! (rule 3, and §62.3's promise).
//!
//! # Where the numbers come from — calibrated and chosen, not transcribed
//!
//! No reachable source tabulates the six rate constants with their temperature
//! dependence and a heat of reaction for this network (§62.3). So:
//!
//! - **The rate constants are CALIBRATED** to Song 2017's industrial outlet
//!   yields (Fig. 2: bottoms 27, distillate 47, naphtha 21, light ends 4 wt%,
//!   which sum to 99 and are normalised here to 27.27 / 47.47 / 21.21 / 4.04) at
//!   390 °C and an LHSV of 1 h⁻¹ — chosen inside Faraji 2020's pilot range of
//!   0.8–1.05 h⁻¹. Six constants, three independent yields: the three secondary
//!   ones (distillate → naphtha, distillate → gas, naphtha → gas) are CHOSEN at
//!   0.10, 0.03 and 0.04 h⁻¹, and the three primary ones are then solved to hit
//!   the targets exactly. `tests/reference/hydrocracking.rs` gates the result
//!   inside a ±1 wt% envelope: a regression lock on the calibration, not a
//!   validation of it — one anchor used twice (the ceiling `four_lump.rs`
//!   names, for the same reason).
//! - **The activation energy is CHOSEN**, `E = 150 kJ/mol`, one value for the
//!   whole network (Song 2017 gives one per cracking lump; this model takes the
//!   same one for all three). It sets how fast the bed speeds up with heat:
//!   about 4% per kelvin at 390 °C.
//! - **The formation enthalpies are CHOSEN** game values: 0, −350, −600 and
//!   −1 200 kJ/kg for vgo, distillate, naphtha and gas. Hydrogen is not in the
//!   slate (§62.7), so the heat of the hydrogen each path adds is folded into
//!   its products' figure. Everything cracked to gas is the hottest state a bed
//!   can reach: `1 200 kJ/kg` over a `c̄p` of 3.2 kJ/(kg·K) is a rise of 375 K.

use refinery_core::components::{Composition, Slate};
use refinery_core::error::SimError;
use refinery_core::traits::{Reaction, ReactionModel};
use refinery_core::units::{JPerKg, Kelvin, Seconds};

/// Universal gas constant [J/(mol·K)].
const R_GAS: f64 = 8.314_462_618;

/// Reference temperature of the rate constants [K]: 390 °C, Song 2017's
/// industrial reactor temperature the calibration is quoted at.
pub const T_REF_K: f64 = 663.15;

/// One hour [s] — the constants are fitted per hour, against `1/LHSV`.
const HOUR: f64 = 3600.0;

/// The smallest spread the three cracking lumps' total rates may have, as a
/// fraction of the largest. Below it the chains' divided differences divide by
/// a span that rounds to nothing (see the module note).
const MIN_RATE_SPREAD: f64 = 1e-3;

/// The kinetic parameter set.
///
/// Paths, in this order everywhere: `[vgo→distillate, vgo→naphtha, vgo→gas,
/// distillate→naphtha, distillate→gas, naphtha→gas]`.
#[derive(Debug, Clone)]
pub struct HydrocrackingParams {
    /// First-order rate constants at [`T_REF_K`] [s⁻¹], in path order.
    pub k_ref: [f64; 6],
    /// The activation energy of every path [J/mol], applied as
    /// `k(T) = k_ref · exp(−E/R · (1/T − 1/T_ref))` — Arrhenius about a
    /// reference temperature, the pre-exponential absorbed into `k_ref`.
    pub e_act: f64,
    /// Lump formation enthalpies [J/kg] in lump order `[vgo, distillate,
    /// naphtha, gas]`, vgo the datum. The pass's heat of reaction is
    /// `Σ (y_out − y_in)·h_f`: path-independent and zero when nothing reacts.
    /// Negative `Δh_rxn` RELEASES heat (`Reaction::dh_rxn`'s sign), so cracking
    /// gives its products the LOWER figure.
    pub h_form: [f64; 4],
}

impl HydrocrackingParams {
    /// The calibrated set (see the module note): primary constants solved to
    /// Song 2017's normalised yields at 390 °C and LHSV 1 h⁻¹, secondary ones,
    /// the activation energy and the formation enthalpies chosen.
    pub fn song_2017() -> Self {
        Self {
            k_ref: [
                0.916_986_030 / HOUR,
                0.334_767_219 / HOUR,
                0.047_529_735 / HOUR,
                0.100 / HOUR,
                0.030 / HOUR,
                0.040 / HOUR,
            ],
            e_act: 150.0e3,
            h_form: [0.0, -350.0e3, -600.0e3, -1_200.0e3],
        }
    }

    /// The six rate constants at `temperature` [s⁻¹].
    fn at(&self, temperature: Kelvin) -> [f64; 6] {
        // Arrhenius about the reference temperature:
        // k(T) = k_ref · exp(−E/R · (1/T − 1/T_ref)).
        let shift = (-self.e_act / R_GAS * (1.0 / temperature.value() - 1.0 / T_REF_K)).exp();
        self.k_ref.map(|k| k * shift)
    }
}

/// Slate positions of the four lumps, resolved by NAME at construction (the
/// lumps ARE slate members, DESIGN §5 fork 1). Order is the model's own.
#[derive(Debug, Clone, Copy)]
struct LumpIndices {
    vgo: usize,
    distillate: usize,
    naphtha: usize,
    gas: usize,
}

/// Four-lump first-order hydrocracking, exact at every residence time.
pub struct Hydrocracking {
    params: HydrocrackingParams,
    lumps: LumpIndices,
}

/// The four lump mass fractions in the model's order: vgo, distillate,
/// naphtha, gas.
type Lumps = [f64; 4];

/// `k[from][to]` for `from < to` (zero elsewhere) [s⁻¹], from the path order.
fn rate_matrix(k: &[f64; 6]) -> [[f64; 4]; 4] {
    let mut m = [[0.0; 4]; 4];
    m[0][1] = k[0];
    m[0][2] = k[1];
    m[0][3] = k[2];
    m[1][2] = k[3];
    m[1][3] = k[4];
    m[2][3] = k[5];
    m
}

/// Each lump's total cracking rate `λ` [s⁻¹]: the sum of its row.
fn exit_rates(m: &[[f64; 4]; 4]) -> [f64; 4] {
    m.map(|row| row.iter().sum())
}

impl Hydrocracking {
    /// The calibrated model, its four lumps resolved against `slate` by name.
    ///
    /// # Errors
    /// `SimError::Scenario` if the slate lacks any of `vgo`, `distillate`,
    /// `naphtha`, `gas`.
    pub fn song_2017(slate: &Slate) -> Result<Self, SimError> {
        Self::with_params(slate, HydrocrackingParams::song_2017())
    }

    /// The model with an explicit parameter set — for the reference tests.
    ///
    /// # Errors
    /// `SimError::Scenario` if the slate lacks a named lump; if a rate constant
    /// is negative or not finite (it would run a path backwards); if the
    /// activation energy or a formation enthalpy is not finite; or if the three
    /// cracking lumps' total rates do not spread by at least
    /// [`MIN_RATE_SPREAD`] of the largest (the module note's refusal).
    pub fn with_params(slate: &Slate, params: HydrocrackingParams) -> Result<Self, SimError> {
        let index = |name: &str| -> Result<usize, SimError> {
            slate.index_of(name).ok_or_else(|| {
                SimError::Scenario(format!(
                    "reactions = \"hydrocracking\" needs a '{name}' lump in the slate; the model \
                     cracks vgo into distillate, naphtha and gas (docs/DESIGN.md §62.3)"
                ))
            })
        };
        let lumps = LumpIndices {
            vgo: index("vgo")?,
            distillate: index("distillate")?,
            naphtha: index("naphtha")?,
            gas: index("gas")?,
        };
        for (i, k) in params.k_ref.iter().enumerate() {
            if !k.is_finite() || *k < 0.0 {
                return Err(SimError::Scenario(format!(
                    "hydrocracking rate constant k{} = {k} must be finite and >= 0",
                    i + 1
                )));
            }
        }
        if !params.e_act.is_finite() || params.h_form.iter().any(|h| !h.is_finite()) {
            return Err(SimError::Scenario(
                "hydrocracking activation energy and formation enthalpies must be finite".into(),
            ));
        }
        let rates = exit_rates(&rate_matrix(&params.k_ref));
        let cracking = &rates[..3];
        let largest = cracking.iter().copied().fold(0.0, f64::max);
        let smallest = cracking.iter().copied().fold(f64::INFINITY, f64::min);
        if largest > 0.0 && largest - smallest < MIN_RATE_SPREAD * largest {
            return Err(SimError::Scenario(format!(
                "hydrocracking: the three cracking lumps' total rates ({:?} s⁻¹) all coincide; \
                 the exact solution divides by their spread (docs/DESIGN.md §62.9)",
                cracking
            )));
        }
        Ok(Self { params, lumps })
    }

    /// The lumps after a residence time `tau` [s] (`+∞` allowed) at the rate
    /// matrix `m`, summed chain by chain (the module note).
    fn evolve(before: &Lumps, m: &[[f64; 4]; 4], tau: f64) -> Lumps {
        let rates = exit_rates(m);
        let mut after = [0.0; 4];
        for (source, &mass) in before.iter().enumerate() {
            if mass == 0.0 {
                continue;
            }
            let mut chain = [source; 4];
            walk(&mut chain, 1, 1.0, m, &rates, tau, mass, &mut after);
        }
        after
    }
}

/// Add the chain `chain[..len]` (with its rate-constant product `product`) to
/// the lump it ends in, then extend it by every lump its last one cracks to.
#[allow(clippy::too_many_arguments)]
fn walk(
    chain: &mut [usize; 4],
    len: usize,
    product: f64,
    m: &[[f64; 4]; 4],
    rates: &[f64; 4],
    tau: f64,
    mass: f64,
    after: &mut Lumps,
) {
    let last = chain[len - 1];
    let mut nodes = [0.0; 4];
    for (node, &lump) in nodes.iter_mut().zip(&chain[..len]) {
        *node = rates[lump];
    }
    after[last] += mass * product * chain_weight(&mut nodes[..len], tau);
    for next in last + 1..4 {
        let k = m[last][next];
        if k > 0.0 {
            chain[len] = next;
            walk(chain, len + 1, product * k, m, rates, tau, mass, after);
        }
    }
}

/// The weight `(−1)ᵐ·f[λ₀, …, λₘ]` of one chain, `f(λ) = e^(−λτ)` (Bateman,
/// 1910) — the fraction of a unit of its first lump that sits in its last after
/// `τ`, per unit of its rate-constant product. Non-negative by construction;
/// rounding at a vanishing `τ` is clamped to zero, never let through negative.
///
/// `rates` is reordered (sorted ascending) in place.
fn chain_weight(rates: &mut [f64], tau: f64) -> f64 {
    let last = rates[rates.len() - 1];
    if tau == f64::INFINITY {
        // The limit: a chain ending in a lump that does not crack keeps
        // Π 1/λⱼ over the others (each > 0, or the chain's product is zero and
        // `walk` never reached here); one ending in a cracking lump has emptied.
        if last > 0.0 {
            return 0.0;
        }
        return rates[..rates.len() - 1].iter().map(|r| 1.0 / r).product();
    }
    rates.sort_by(f64::total_cmp);
    let sign = if rates.len() % 2 == 1 { 1.0 } else { -1.0 };
    let weight = sign * divided_difference(rates, tau);
    // Not `f64::max`, which would turn a NaN into a plausible zero and hide it
    // from `react`'s mass check.
    if weight < 0.0 {
        0.0
    } else {
        weight
    }
}

/// `f[x₀, …, xₙ]` for `f(x) = e^(−x·τ)`, the rates sorted ascending, `τ`
/// finite. Two rates in one stable step; more by the recursion over the widest
/// span (`xₙ − x₀`, nonzero unless every rate coincides, which
/// `Hydrocracking::with_params` refuses).
fn divided_difference(x: &[f64], tau: f64) -> f64 {
    match x.len() {
        1 => (-x[0] * tau).exp(),
        2 => {
            // f[a, b] = −e^(−aτ)·(1 − e^(−(b−a)τ))/(b − a), a ≤ b, and −τ·e^(−aτ)
            // at b = a: no subtraction of nearby exponentials.
            let spread = x[1] - x[0];
            let lag = if spread > 0.0 {
                -(-spread * tau).exp_m1() / spread
            } else {
                tau
            };
            -(-x[0] * tau).exp() * lag
        }
        n => {
            (divided_difference(&x[1..], tau) - divided_difference(&x[..n - 1], tau))
                / (x[n - 1] - x[0])
        }
    }
}

impl ReactionModel for Hydrocracking {
    fn name(&self) -> &'static str {
        "hydrocracking"
    }

    fn react(
        &self,
        feed: &Composition,
        temperature: Kelvin,
        tau: Seconds,
        _slate: &Slate,
    ) -> Result<Reaction, SimError> {
        let tau = tau.value();
        if tau.is_nan() || tau < 0.0 {
            return Err(SimError::Numerical(format!(
                "hydrocracking residence time tau = {tau} s must be >= 0 (+inf allowed)"
            )));
        }
        if !temperature.value().is_finite() || temperature.value() <= 0.0 {
            return Err(SimError::Numerical(format!(
                "hydrocracking at {} K: the rates need a finite absolute temperature",
                temperature.value()
            )));
        }
        let f = feed.fractions();
        let before: Lumps = [
            f[self.lumps.vgo],
            f[self.lumps.distillate],
            f[self.lumps.naphtha],
            f[self.lumps.gas],
        ];
        let after = Self::evolve(&before, &rate_matrix(&self.params.at(temperature)), tau);

        // The lumps' total, checked: each lump is its own chain sum, so a chain
        // missed or counted twice changes it (the module note).
        let (sum_in, sum_out): (f64, f64) = (before.iter().sum(), after.iter().sum());
        if !sum_out.is_finite() || (sum_out - sum_in).abs() > 1e-9 {
            return Err(SimError::Numerical(format!(
                "hydrocracking did not conserve the lumps' mass: {sum_in} in, {sum_out} out \
                 after tau = {tau} s at {} K",
                temperature.value()
            )));
        }

        // Any slate member the network does not name is inert and keeps its
        // fraction, as in `FourLump`.
        let mut weights = f.to_vec();
        weights[self.lumps.vgo] = after[0];
        weights[self.lumps.distillate] = after[1];
        weights[self.lumps.naphtha] = after[2];
        weights[self.lumps.gas] = after[3];
        let products = Composition::from_weights(&weights).map_err(|e| {
            SimError::Numerical(format!("hydrocracking products are not a composition: {e}"))
        })?;

        // Heat of reaction from the formation enthalpies: path-independent, and
        // zero when nothing reacted.
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
    use refinery_core::components::{Phase, PseudoComponent};
    use refinery_core::units::{JPerKgK, KgPerM3, KgPerMol};

    /// Slate order deliberately differs from the model's lump order.
    fn slate_of(names: &[&str]) -> Slate {
        Slate::new(
            names
                .iter()
                .map(|n| PseudoComponent {
                    name: (*n).into(),
                    tb: Kelvin(500.0),
                    molar_mass: KgPerMol(0.2),
                    density: Some(KgPerM3(800.0)),
                    phase: Phase::Liquid,
                    cp: JPerKgK(3200.0),
                    cp_shape: None,
                })
                .collect(),
        )
        .expect("test slate")
    }

    fn slate() -> Slate {
        slate_of(&["gas", "naphtha", "steam", "vgo", "distillate"])
    }

    /// No residence time, no change: the products ARE the feed, bit for bit,
    /// and nothing is released — zero because nothing moved, not because the
    /// formation enthalpies cancel.
    #[test]
    fn a_zero_residence_time_is_the_identity() {
        let slate = slate();
        let model = Hydrocracking::song_2017(&slate).expect("model");
        let feed = Composition::from_weights(&[0.02, 0.08, 0.2, 0.55, 0.15]).expect("feed");
        let out = model
            .react(&feed, Kelvin(T_REF_K), Seconds(0.0), &slate)
            .expect("react");
        assert_eq!(out.products.fractions(), feed.fractions());
        assert_eq!(out.dh_rxn.value(), 0.0);
    }

    /// Cracking RELEASES heat: negative `Δh_rxn` by the trait's sign. Backwards
    /// formation enthalpies would make the bed cool as it cracks, and settle —
    /// the FCC bed of §62.0, which cannot run away.
    #[test]
    fn cracking_releases_heat() {
        let slate = slate();
        let model = Hydrocracking::song_2017(&slate).expect("model");
        let out = model
            .react(
                &Composition::pure(5, 3),
                Kelvin(T_REF_K),
                Seconds(3600.0),
                &slate,
            )
            .expect("react");
        assert!(out.dh_rxn.value() < 0.0, "{}", out.dh_rxn.value());
    }

    /// A slate member the network does not name keeps its fraction exactly,
    /// and the four lumps keep their combined share: a free quantity on a slate
    /// with an inert, so a chain missed or counted twice would move it.
    #[test]
    fn an_inert_passes_untouched_and_the_lumps_keep_their_share() {
        let slate = slate();
        let model = Hydrocracking::song_2017(&slate).expect("model");
        let feed = Composition::from_weights(&[0.02, 0.08, 0.2, 0.55, 0.15]).expect("feed");
        let out = model
            .react(&feed, Kelvin(700.0), Seconds(5000.0), &slate)
            .expect("react");
        let y = out.products.fractions();
        assert!((y[2] - 0.2).abs() < 1e-15, "the inert moved: {}", y[2]);
        assert!(y[3] < 0.55 - 0.1, "the vgo must have cracked: {}", y[3]);
    }

    #[test]
    fn a_slate_missing_a_lump_is_refused() {
        let missing = slate_of(&["gas", "naphtha", "vgo"]); // no distillate
        let err = Hydrocracking::song_2017(&missing).err().expect("refused");
        assert!(err.to_string().contains("'distillate'"), "{err}");
    }

    /// Meaningless rather than extreme: a negative constant runs a path
    /// backwards; a non-finite activation energy or enthalpy is no number; and
    /// three cracking lumps sharing one total rate leave the exact solution
    /// dividing by nothing.
    #[test]
    fn meaningless_parameters_are_refused() {
        let slate = slate();
        let mut backwards = HydrocrackingParams::song_2017();
        backwards.k_ref[3] = -1e-5;
        assert!(Hydrocracking::with_params(&slate, backwards).is_err());

        let mut no_energy = HydrocrackingParams::song_2017();
        no_energy.e_act = f64::NAN;
        assert!(Hydrocracking::with_params(&slate, no_energy).is_err());

        let mut no_heat = HydrocrackingParams::song_2017();
        no_heat.h_form[2] = f64::INFINITY;
        assert!(Hydrocracking::with_params(&slate, no_heat).is_err());

        // vgo, distillate and naphtha each cracking at 1e-4 s⁻¹ in total.
        let mut coincident = HydrocrackingParams::song_2017();
        coincident.k_ref = [0.5e-4, 0.3e-4, 0.2e-4, 0.6e-4, 0.4e-4, 1.0e-4];
        let err = Hydrocracking::with_params(&slate, coincident)
            .err()
            .expect("refused");
        assert!(err.to_string().contains("coincide"), "{err}");
        // A spread just past the limit is admitted.
        let mut spread = HydrocrackingParams::song_2017();
        spread.k_ref = [0.5e-4, 0.3e-4, 0.2e-4, 0.6e-4, 0.4e-4, 0.998e-4];
        assert!(Hydrocracking::with_params(&slate, spread).is_ok());
    }

    /// `τ = +∞` is an answer (a bed with no flow); a NaN or negative `τ`, or a
    /// temperature that is not a positive finite number, is an error.
    #[test]
    fn a_meaningless_residence_time_or_temperature_is_an_error() {
        let slate = slate();
        let model = Hydrocracking::song_2017(&slate).expect("model");
        let feed = Composition::pure(5, 3);
        let at = |t: f64, tau: f64| model.react(&feed, Kelvin(t), Seconds(tau), &slate);
        assert!(at(T_REF_K, f64::INFINITY).is_ok());
        assert!(at(T_REF_K, f64::NAN).is_err());
        assert!(at(T_REF_K, -1.0).is_err());
        assert!(at(0.0, 10.0).is_err());
        assert!(at(f64::INFINITY, 10.0).is_err());
    }
}
