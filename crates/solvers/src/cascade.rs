//! The equilibrium-stage cascade — the complex separation fidelity (M7.3).
//!
//! `N` stages under a **total** condenser, with the reboiler counted as stage `N`
//! and the condenser counted as no stage at all (DESIGN §5, correction 3 — the
//! convention the Fenske gate's exponent rests on). Constant molar overflow, so
//! the composition and temperature profile need K-values and nothing else; the
//! duties, which need `Δh_vap`, are M7.4.
//!
//! The method is Wang–Henke's bubble-point form (Wang & Henke, *Hydrocarbon
//! Processing* 45 (1966) 155): for each component the stage material balances are
//! tridiagonal in that component's liquid mole fractions, so one Thomas sweep per
//! component gives the whole profile at fixed K; each stage's temperature then
//! follows from its own bubble point, and the two alternate to a fixed point.
//!
//! Everything inside is **molar**. Mass enters twice and only twice — the feed
//! composition on the way in, and each draw's composition and ratio on the way
//! out — which is fork 1's structural boundary, enforced by `MoleFractions` being
//! a different type from `Composition` rather than by remembering.
//!
//! **What is refused rather than approximated.** A column with no cascade
//! equipment, a non-positive feed, an infeasible flow specification, a stage whose
//! bubble-point equation has no root in the bracket, a singular stage row, and a
//! profile that has not converged. Rule 5: none of these becomes a plausible
//! number.

use refinery_core::components::{Composition, Slate};
use refinery_core::energy::T_REF;
use refinery_core::error::SimError;
use refinery_core::graph::{CascadeSpec, ColumnDraw};
use refinery_core::traits::{
    CascadeProfile, ColumnPass, DrawSeparation, Separation, SeparationModel, ThermoModel,
};
use refinery_core::units::{Kelvin, Pascal, Watt};

use crate::bubble::bubble_temperature_unnormalized;
use crate::molar::MoleFractions;

/// I7's per-component mass tolerance expressed as a FLOW: `COMPONENT_MASS_
/// TOLERANCE_KG = 1e-6` kg over the 0.1 s tick every scenario in this repo runs.
///
/// **Derived, not chosen** (DESIGN §5, fork 4). The per-component balance can
/// never be tighter than the total mass balance it partitions, so the cascade's
/// convergence criterion is I7's own number and not one picked to make a test
/// pass. It is an absolute flow for the same reason I7's is absolute — the M1
/// acceptance-gate argument.
const COMPONENT_RESIDUAL_KG_PER_S: f64 = 1e-5;

/// Equilibrium-stage distillation: the complex (research) separation fidelity,
/// selected by `[fidelity] separation = "cascade"`.
///
/// **What it is specified by, and what it refuses to be specified by.** Two
/// degrees of freedom remain once pressure, feed, stage count and feed stage are
/// fixed, and both are **dimensionless**: the molar reflux ratio `R = L/D` on
/// `CascadeSpec`, and one **mass** ratio per draw on `ColumnDraw::draw_ratio`. An
/// absolute distillate rate is inadmissible and the reason is M3.2's, not a
/// preference: a column whose product flow is prescribed either freezes its own
/// feed or creates mass in a zero-volume node, and does it while converging,
/// conserving and rerunning bit-identically (DESIGN §5, fork 3).
///
/// **Where the mass ⇄ mole boundary actually bites.** `D/F` is a mass ratio, so
/// the total mass out equals the total mass in *exactly* — the splits are the
/// declared ratios and sum to 1 by subtraction. The cascade's own molar
/// distillate rate `D` is then **not an input**: it is `(D/F)·ṁ_feed / M̄_D`, and
/// `M̄_D` depends on the distillate composition still being solved for. That inner
/// coupling is real and it is why per-component mass closes only to a tolerance
/// (fork 4), where a splitter closed it identically.
///
/// **The one thing this fidelity does NOT compute yet.** Both duties are
/// `Watt::ZERO`, because `Q_reb ≈ V̄·Δh_vap` is mostly latent and `Δh_vap` is
/// M7.4's box. Nothing reads either field before M7.4, so the zero moves no
/// number — but unlike the splitter's zero it is *not* the honest answer for this
/// fidelity, and the difference is worth stating rather than sharing a comment.
pub struct StageCascade {
    max_iterations: u32,
    seed_offset: Kelvin,
}

impl StageCascade {
    /// Outer (profile ⇄ bubble-point) iterations before the solve is declared
    /// failed.
    ///
    /// **Measured rather than guessed, and the measurement found one stiff case
    /// worth naming.** Successive substitution here converges linearly, and an
    /// ordinary column is done in under 20 passes — a 10-stage, `α = 4`, `R = 5`
    /// binary at `D/F = 0.35` converges in 20. The same column at `D/F = 0.5`
    /// needs about 1300, and the reason is physical rather than numerical: at that
    /// ratio the distillate is asked for *exactly* the light component the feed
    /// contains, so the specification sits on a pinch and both products approach
    /// purity. The molar-mass coupling is NOT the slow mode — repeating the
    /// measurement on a slate whose two cuts have equal molar mass gives the same
    /// two numbers.
    ///
    /// So the cap is set above the stiff case rather than above the easy one. It
    /// still bounds the cost of a pathological plant, and a column that reaches it
    /// gets an `Err` carrying its residual.
    pub const DEFAULT_MAX_ITERATIONS: u32 = 2000;

    pub fn new() -> Self {
        Self {
            max_iterations: Self::DEFAULT_MAX_ITERATIONS,
            seed_offset: Kelvin::ZERO,
        }
    }

    /// A cascade capped at `max_iterations` outer iterations.
    ///
    /// **This exists for a gate**, the same way `TroutonThermo::
    /// with_trouton_constant` does: fork 4 requires the non-convergence `Err` be
    /// reached deliberately, and a cap of one or two iterations is the only way to
    /// reach it without inventing a divergent plant. A threshold nothing is shown
    /// to cross is the shape `a-counter-is-not-a-gate` records.
    pub fn with_max_iterations(max_iterations: u32) -> Self {
        Self {
            max_iterations,
            seed_offset: Kelvin::ZERO,
        }
    }

    /// A cascade whose cold seed temperature is offset by `seed_offset`.
    ///
    /// Also for a gate. Fork 5 allows a warm start because "a warm start changes
    /// the iteration count, not the fixed point" — and that claim is only worth
    /// anything if it is measured, so the reference is run cold and *also* from a
    /// deliberately displaced seed, and the two must agree within tolerance.
    /// (There is no warm start in M7.3: `separate` takes `&self` and is contracted
    /// pure, so the cascade has nowhere to keep a previous profile.)
    pub fn with_seed_offset(seed_offset: Kelvin) -> Self {
        Self {
            max_iterations: Self::DEFAULT_MAX_ITERATIONS,
            seed_offset,
        }
    }

    /// The column's two heat duties [W], as emergent diagnostics.
    ///
    /// **Why they are not two symmetric envelope balances, which is what M7.4b
    /// set out to build.** The condenser's envelope is exact and needs nothing
    /// from the column's interior; the reboiler's does not have that property,
    /// and neither does any interior stage. Constant molar overflow fixes `L`
    /// and `V` instead of solving each stage's energy balance, so every interior
    /// stage carries an energy residual, and summing them **telescopes** to
    /// roughly `V·[λ̄(y_1) − λ̄(y_N)]` rather than cancelling. On this
    /// workspace's own two-cut fixture the molar latent heats differ by 30%
    /// (`88·333` against `88·433` J/mol), while the sensible external balance
    /// the pair is supposed to bracket is about 9% of a duty. Two locally-exact
    /// envelopes would therefore disagree with the column's own external balance
    /// by three times the quantity that balance measures — a diagnostic pair
    /// that reports a plant creating or destroying energy, which is worse than a
    /// coarse one.
    ///
    /// So the pair is built the other way round:
    ///
    /// ```text
    /// Q_cond = V·λ̄(y₁) + V·c̄p(y₁)·(T₁ − T_cond)          exact, local
    /// Q_reb  = Q_cond + [Σᵢ ṁᵢ·cpᵢ·(Tᵢ − T_REF) − ṁ_F·cp_F·(T_F − T_REF)]
    /// ```
    ///
    /// The condenser is determined by physics; the reboiler is the duty that
    /// **closes the column's external energy balance**, which is the property a
    /// plant-level energy invariant needs and the one a locally-exact pair would
    /// not have. What it costs is stated rather than hidden: `Q_reb` differs
    /// from its own local envelope `≈ V·λ̄(y_N)` by that same CMO telescoping
    /// term, so the reboiler duty carries the formulation's error and the
    /// condenser duty does not.
    ///
    /// **And that is why the roadmap's "duty difference" box cannot be its own
    /// gate.** `Q_reb − Q_cond` is the external sensible balance *by
    /// construction* here, so asserting it equals the external sensible balance
    /// is the tautology M4's two-duty lesson warns about, one level up. The real
    /// gates are: the condenser duty reconstructed independently from the public
    /// return and a published `k_value` (`tests/reference/cascade.rs`), the
    /// `α = 1` case where the whole pair reduces to one hand-computed `V·λ̄(z)`
    /// with no sensible term at all, and an envelope on the reboiler.
    ///
    /// **The second condenser term is real physics, not a CMO artifact.** A
    /// total condenser takes saturated VAPOUR in at `y₁` and puts saturated
    /// LIQUID out at the same composition, and those two states are at different
    /// temperatures: `T₁` is the dew point of `y₁` (which is exactly what
    /// `Σ_c K_c(T₁)·x₁_c = 1` makes stage 1's bubble point) and `T_cond` is its
    /// bubble point, which is lower. Reading a total condenser as pure latent
    /// heat is wrong by `V·c̄p·(T₁ − T_cond)`, and on a wide-boiling cut that is
    /// not small.
    #[allow(clippy::too_many_arguments)]
    fn duties(
        &self,
        slate: &Slate,
        thermo: &dyn ThermoModel,
        flows: &Flows,
        distillate: &DrawState,
        stage_one_t: Kelvin,
        condenser_t: Kelvin,
        draws: &[DrawSeparation],
        pass: &ColumnPass<'_>,
    ) -> Result<(Watt, Watt), SimError> {
        let vapour = flows.vapour;
        let y_one = distillate.moles.fractions();

        // Latent: the whole uniform vapour rate condenses, at the composition it
        // condenses AT. Evaluated at `condenser_t` because that is the state the
        // latent heat is released into; `TroutonThermo` ignores the argument, and
        // a `T`-dependent Δh_vap would want exactly this one.
        let latent = vapour * mixture_dh_vap(slate, thermo, y_one, condenser_t)?;
        // Sensible: dew point down to bubble point, same composition, all of it
        // as liquid once condensed.
        let sensible =
            vapour * mixture_molar_cp(slate, y_one) * (stage_one_t.value() - condenser_t.value());
        let condenser_duty = latent + sensible;

        // The external sensible balance, in the ENGINE's datum (`energy::T_REF`)
        // rather than a datum local to this function — `datum-consistency-in-a-
        // holdup-balance`. It cancels out of the difference analytically, but the
        // number these duties must be comparable with is `energy::enthalpy_flux`,
        // and a second datum is how the two drift.
        let feed_flux = pass.feed_flow.value()
            * pass.feed.mixture_cp(slate).value()
            * (pass.temperature.value() - T_REF.value());
        let mut draw_flux = 0.0;
        for draw in draws {
            draw_flux += draw.split
                * pass.feed_flow.value()
                * draw.composition.mixture_cp(slate).value()
                * (draw.temperature.value() - T_REF.value());
        }
        let reboiler_duty = condenser_duty + (draw_flux - feed_flux);

        for (name, duty) in [("condenser", condenser_duty), ("reboiler", reboiler_duty)] {
            // Two checks with different standing, and it is worth not conflating
            // them.
            //
            // The FINITENESS half is rule 5's mandated post-solve check ("check
            // for NaN/Inf after every solve"), and it needs no reachability
            // argument to earn its place.
            //
            // The NON-NEGATIVE half is the sign convention: `Separation`'s fields
            // are magnitudes with the direction in the name (the
            // `Furnace`/`Cooler` convention), so a condenser that heats is not
            // representable and a negative number here would be a solve that has
            // left the physical region. **Nothing has been shown to reach it** —
            // `the_duties_stay_non_negative_where_it_is_hardest` pushes five
            // adversarial configurations at it and the closest comes within 3.7 kW
            // on a 311 kW duty. That is deliberately weaker than the claim
            // `flash.rs` makes when it DELETES an unreachable guard, which rests
            // on a proof ("beta is 0.0, 1.0, or a midpoint"). No such proof exists
            // here, so the check stays and the margin is gated instead.
            if !duty.is_finite() || duty < 0.0 {
                return Err(SimError::Numerical(format!(
                    "stage cascade: the {name} duty came out as {duty:.4e} W. Both duties are \
                     non-negative magnitudes — a condenser removes heat and a reboiler adds it \
                     — so a negative one is a profile that has left the physical region rather \
                     than a column running backwards."
                )));
            }
        }
        Ok((Watt(condenser_duty), Watt(reboiler_duty)))
    }

    /// Check a column's cascade geometry without solving it — what the LOADER
    /// calls so a malformed cascade column is refused when the file is read
    /// rather than on its first tick.
    ///
    /// It is the same code `separate` runs, deliberately: two copies of the
    /// stage-numbering and draw-ratio rules would be two things to keep in step,
    /// and the loader's own job is the half that needs a file's vocabulary (which
    /// key on which node, and the scope boundaries that have no representation in
    /// `core` at all).
    ///
    /// # Errors
    /// `SimError` for any geometry `separate` would refuse: a zero stage count, a
    /// feed stage outside it, a negative reflux ratio, draws that do not run from
    /// the condenser to the reboiler in order, a draw carrying the cut-point
    /// fidelity's `upper_cut`, and a mis-declared set of draw ratios.
    pub fn validate(spec: &CascadeSpec, draws: &[ColumnDraw]) -> Result<(), SimError> {
        // The component count is not part of a column's geometry and nothing on
        // this path reads it — `CascadePlan::components` is for the solve. Passed
        // as 0 rather than invented, and the plan is discarded immediately.
        CascadePlan::new(spec, draws, 0).map(|_| ())
    }
}

impl Default for StageCascade {
    fn default() -> Self {
        Self::new()
    }
}

impl SeparationModel for StageCascade {
    fn name(&self) -> &'static str {
        "cascade"
    }

    fn separate(
        &self,
        pass: &ColumnPass<'_>,
        thermo: &dyn ThermoModel,
    ) -> Result<Separation, SimError> {
        let slate = pass.slate;
        let spec = pass.cascade.ok_or_else(|| {
            SimError::Scenario(
                "separation = \"cascade\" but this column declares no cascade equipment: a \
                 stage cascade needs [nodes.<column>.cascade] with stages, feed_stage and \
                 reflux_ratio. The cut-point splitter is the fidelity that needs none."
                    .into(),
            )
        })?;

        let feed_flow = pass.feed_flow.value();
        if !feed_flow.is_finite() || feed_flow < 0.0 {
            return Err(SimError::Numerical(format!(
                "stage cascade has a negative or non-finite column feed ({feed_flow:.4e} \
                 kg/s): every internal flow is proportional to it, so there is no profile to \
                 solve. `ColumnPass::feed_flow` is the sweep's INFLOW SUM and cannot be \
                 negative, so this is a caller constructing a pass by hand."
            )));
        }

        // The geometry is validated before the flow is looked at, so which plants
        // are legal never depends on how much is flowing through them.
        let plan = CascadePlan::new(spec, pass.draws, slate.len())?;

        // **M7.1's correction 4, and it is a convention rather than a refusal.**
        // That correction predicted a cascade would "solve on the zero and fail
        // with a worse message before the guard that names the cause is ever
        // reached" — true, because `separate` runs INSIDE the sweep while
        // `Engine::tick`'s reverse-feed refusal runs after it, and at zero feed
        // every internal molar flow is zero and the first stage row is singular.
        //
        // The fix is not to refuse. `column_feed_flow` reports a zero for two
        // different states — a column running BACKWARDS, and a column with nothing
        // flowing at all — and only the first is an error. Refusing both would kill
        // a tick the moment an operator shut a feed valve, and would make the two
        // separation fidelities disagree about which plants are legal: the splitter
        // handles an idle column perfectly well, since a fraction of nothing is
        // nothing. So an idle cascade returns its DECLARED splits, and the reverse-
        // feed case falls through to the engine's own guard, which is what
        // correction 4 wanted reached.
        //
        // Nothing is carried anywhere at zero flow, so the compositions and
        // temperature here are inert placeholders — the same move the splitter
        // makes for a draw whose band catches no component, and the same "decided,
        // not discovered" move `flash_isothermal` makes for its all-`K = 1` case.
        if feed_flow == 0.0 {
            return Ok(Separation {
                // An idle column solved nothing, so it has nothing to seed the
                // next tick with. Publishing the seed here would hand the next
                // tick a profile no iteration produced — a plausible-wrong number
                // of exactly the shape `condenser_duty`'s `None` exists to avoid.
                profile: None,
                draws: plan
                    .mass_ratios
                    .iter()
                    .map(|split| DrawSeparation {
                        split: *split,
                        composition: pass.feed.clone(),
                        temperature: pass.temperature,
                    })
                    .collect(),
                // `Some(ZERO)`, not `None`: an idle column really does have zero
                // duty, and that is an ANSWER — this fidelity has a condenser and
                // a reboiler, they just have nothing to do. `None` is the
                // splitter's claim, that there is no such equipment to report on.
                condenser_duty: Some(Watt::ZERO),
                reboiler_duty: Some(Watt::ZERO),
            });
        }

        let feed = MoleFractions::from_mass(pass.feed, slate)?;
        let feed_molar = feed_flow / feed.mean_molar_mass(slate).value();

        // Derived from I7's number and this column's feed rate, per fork 4.
        //
        // Clamped at both ends, and each end has a reason. A very small feed would
        // otherwise buy an enormous tolerance and a vacuous gate; a very large one
        // would ask for more precision than a chain of `nc` normalizations and `N`
        // Thomas sweeps has to give, and the solve would fail on arithmetic rather
        // than on physics.
        let tolerance = (COMPONENT_RESIDUAL_KG_PER_S / feed_flow).clamp(1.0e-12, 1.0e-6);

        // Cold start: every stage at the FEED's bubble point, which is a property
        // of the feed alone and so is deterministic and history-free. Not the
        // feed's resolved temperature: that is whatever the sweep mixed, and a
        // cascade seeded at a temperature far off its own profile wastes iterations
        // for no gain.
        let seed = bubble_temperature_unnormalized(
            slate,
            thermo,
            feed.fractions(),
            pass.pressure,
            "the feed",
        )?;

        // The saturated-liquid precondition, enforced rather than assumed. The
        // seed IS the feed's bubble point, so this costs one comparison.
        check_saturated_liquid_feed(slate, thermo, feed.fractions(), seed, pass)?;

        // The warm start (M9.3b), and BOTH profiles or neither. The outer
        // convergence test is a conjunction over the temperatures and the liquid
        // compositions, so seeding one and not the other leaves the unseeded half
        // walking in from cold and the solve is barely faster — measured at 38.0
        // → 35.0 outer iterations per solve for the temperatures alone, against
        // 1.006 for both. `CascadeProfile` is one struct for that reason, so the
        // half-warm start is not a state this code can be in.
        //
        // A seed of the wrong shape is IGNORED rather than reshaped: it is
        // contracted a hint, the only way to get one is a plant whose stage count
        // or slate changed under a node id, and a padded profile is a worse start
        // than the feed's own bubble point. `seed_offset` applies on top of
        // whichever start is used, which is what lets the start-insensitivity
        // gate perturb a WARM start and not only a cold one.
        let warm = pass.seed.filter(|previous| {
            previous.temperatures.len() == plan.stages
                && previous.liquid.len() == plan.stages
                && previous.liquid.iter().all(|row| row.len() == slate.len())
        });
        let mut stage_t: Vec<f64> = match warm {
            Some(previous) => previous
                .temperatures
                .iter()
                .map(|t| t.value() + self.seed_offset.value())
                .collect(),
            None => vec![seed.value() + self.seed_offset.value(); plan.stages],
        };
        let mut liquid: Vec<Vec<f64>> = match warm {
            Some(previous) => previous.liquid.clone(),
            None => vec![feed.fractions().to_vec(); plan.stages],
        };

        let mut iterations = 0u32;
        let mut residual = f64::INFINITY;
        let mut profile_change = f64::INFINITY;
        let mut temperature_change = f64::INFINITY;
        let mut converged = false;
        let mut draws: Vec<DrawState> = Vec::new();
        // K at the seed profile. Recomputed at the END of each pass rather than
        // the start, so the profile and its K-values are only ever evaluated
        // once per iteration — the bubble-point bisections dominate the cost of
        // this solve, and doing them twice per pass would double a tick.
        let mut k = stage_k_values_profile(slate, thermo, &stage_t, pass.pressure)?;

        while iterations < self.max_iterations {
            iterations += 1;

            // Molar draw rates, from the mass ratios and the CURRENT draw
            // compositions. This is the inner coupling correction 1 names: `M̄_D`
            // is an iterate, so `D` is too.
            let previous = plan.draw_states(&liquid, &k, slate)?;
            let flows = plan.flows(spec, &previous, feed_flow, feed_molar)?;

            // One Thomas sweep per component: the stage balances are tridiagonal
            // in that component's liquid mole fractions at fixed K.
            let mut columns = Vec::with_capacity(slate.len());
            for (c, &z) in feed.fractions().iter().enumerate() {
                columns.push(plan.solve_component(&flows, &k, c, z, feed_molar)?);
            }
            // Transposed from one column per component to one row per stage, which
            // is the shape a stage's composition and its bubble point are read in.
            let amounts: Vec<Vec<f64>> = (0..plan.stages)
                .map(|j| columns.iter().map(|column| column[j]).collect())
                .collect();
            let mut next: Vec<Vec<f64>> = Vec::with_capacity(plan.stages);
            for (j, row) in amounts.iter().enumerate() {
                next.push(
                    MoleFractions::from_amounts(row)
                        .map_err(|e| {
                            SimError::Numerical(format!(
                                "stage cascade: stage {} of {} produced no valid liquid \
                                 composition ({e}). The stage balances are exact at fixed \
                                 K-values, so this is an infeasible specification rather than \
                                 drift.",
                                j + 1,
                                plan.stages
                            ))
                        })?
                        .fractions()
                        .to_vec(),
                );
            }

            profile_change = max_abs_difference(&liquid, &next);
            liquid = next;

            // Bubble point per stage, at the NEW liquid composition.
            let mut next_t = Vec::with_capacity(plan.stages);
            for (j, x) in liquid.iter().enumerate() {
                next_t.push(
                    bubble_temperature_unnormalized(
                        slate,
                        thermo,
                        x,
                        pass.pressure,
                        &format!("stage {} of {}", j + 1, plan.stages),
                    )?
                    .value(),
                );
            }
            temperature_change = stage_t
                .iter()
                .zip(&next_t)
                .map(|(a, b)| (a - b).abs() / b)
                .fold(0.0f64, f64::max);
            stage_t = next_t;

            // The draws as they now stand, and the per-component residual that
            // fork 4 makes a gate rather than a diagnostic.
            k = stage_k_values_profile(slate, thermo, &stage_t, pass.pressure)?;
            draws = plan.draw_states(&liquid, &k, slate)?;
            residual = plan.component_residual(&draws, pass.feed, slate) * feed_flow;

            if profile_change <= tolerance
                && temperature_change <= tolerance
                && residual <= COMPONENT_RESIDUAL_KG_PER_S
            {
                converged = true;
                break;
            }
        }

        if !converged {
            // **Which criteria are unmet, named one by one.** Fork 4 asks for the
            // per-component residual to be a GATE and not a counter, and the
            // convergence test is a conjunction — so a message that reported only
            // the residual would read identically whether the residual was the
            // binding constraint or merely along for the ride. Naming each unmet
            // criterion is what lets a test assert the residual is genuinely one
            // of them (`a-counter-is-not-a-gate`).
            let mut unmet = Vec::new();
            if profile_change > tolerance {
                unmet.push(format!(
                    "the liquid profile still moved {profile_change:.3e} against a tolerance \
                     of {tolerance:.3e}"
                ));
            }
            if temperature_change > tolerance {
                unmet.push(format!(
                    "the temperature profile still moved {temperature_change:.3e} (relative) \
                     against a tolerance of {tolerance:.3e}"
                ));
            }
            if residual > COMPONENT_RESIDUAL_KG_PER_S {
                unmet.push(format!(
                    "the worst per-component mass residual is {residual:.3e} kg/s against a \
                     bound of {COMPONENT_RESIDUAL_KG_PER_S:.3e} kg/s"
                ));
            }
            // Fork 5: an `Err`, never the previous tick's ANSWER. M9.3b makes
            // that sentence need its distinction stated, because a previous
            // profile is now a legitimate INPUT: it enters as a seed and leaves
            // as `Separation::profile` only on the converged path below, so a
            // failed solve publishes nothing and the next tick starts cold.
            // What stays forbidden is what fork 5 forbade — carrying a stale
            // answer forward and calling it this tick's, which would make the
            // column's output depend on run length.
            return Err(SimError::Numerical(format!(
                "stage cascade did not converge in {iterations} iterations: {}. The bound on \
                 the residual is I7's own number — 1e-6 kg per component over a 0.1 s tick — \
                 over this column's feed rate, not a figure chosen to pass. A previous \
                 tick's profile may SEED this solve but is never its answer, and a solve \
                 that fails publishes no profile — so this is a refusal, not a stale \
                 number.",
                unmet.join("; ")
            )));
        }

        // The distillate leaves the total condenser as a saturated liquid, so its
        // temperature is its OWN bubble point, not stage 1's. Computed once here
        // rather than inside the loop: nothing in the iteration reads it.
        let condenser_t = bubble_temperature_unnormalized(
            slate,
            thermo,
            draws[0].moles.fractions(),
            pass.pressure,
            "the total condenser",
        )?;

        let mut result = Vec::with_capacity(draws.len());
        for (i, draw) in draws.iter().enumerate() {
            result.push(DrawSeparation {
                split: plan.mass_ratios[i],
                composition: draw.moles.to_mass(slate)?,
                // Real tray temperatures, from this solve's own profile — not the
                // feed temperature the splitter returns. The cascade computes the
                // profile anyway (a K-value needs a temperature), so filling the
                // field costs nothing and returning the feed temperature would be
                // a number this model knows to be wrong. M7.4 wires the reader.
                temperature: match plan.locations[i] {
                    DrawLocation::Condenser => condenser_t,
                    DrawLocation::Stage(j) => Kelvin(stage_t[j]),
                },
            });
        }

        // The converged flow profile. Recomputed from the CONVERGED draw states
        // rather than reusing the loop's last `flows`, which was built from the
        // iterate before it — one iteration stale, which is inside tolerance but
        // is not the number this solve settled on.
        let flows = plan.flows(spec, &draws, feed_flow, feed_molar)?;
        let (condenser_duty, reboiler_duty) = self.duties(
            slate,
            thermo,
            &flows,
            &draws[0],
            Kelvin(stage_t[0]),
            condenser_t,
            &result,
            pass,
        )?;

        Ok(Separation {
            draws: result,
            // Published only on the converged path: every `return` above this one
            // is an `Err`, so a profile leaves this function only when the outer
            // loop met all three of its criteria. A non-converged profile handed
            // to the next tick would be a seed no fixed point stands behind.
            profile: Some(CascadeProfile {
                temperatures: stage_t.iter().copied().map(Kelvin).collect(),
                liquid: liquid.clone(),
            }),
            condenser_duty: Some(condenser_duty),
            reboiler_duty: Some(reboiler_duty),
        })
    }
}

/// Where a draw leaves from, resolved from `ColumnDraw::stage` once.
///
/// `Stage(j)` is a ZERO-based index into the stage profile, so the reboiler is
/// `Stage(stages − 1)`; the file's `stage` is one-based with `0` meaning the
/// condenser. The translation happens in `CascadePlan::new` and nowhere else.
#[derive(Debug, Clone, Copy, PartialEq)]
enum DrawLocation {
    Condenser,
    Stage(usize),
}

/// One draw's current state: its mole fractions and the mean molar mass that
/// converts its mass ratio into a molar rate.
struct DrawState {
    moles: MoleFractions,
    mean_molar_mass: f64,
}

/// The molar flow profile at one iteration. Constant molar overflow with a
/// saturated-liquid feed makes the vapour rate uniform and the liquid rate
/// piecewise-constant, so this is a short vector rather than a solve.
struct Flows {
    /// Liquid entering stage `j` from above; `liquid_in[0]` is the reflux `L`.
    liquid_in: Vec<f64>,
    /// Liquid leaving stage `j` downwards. Zero at the reboiler, which passes
    /// nothing down — what leaves it is the bottoms draw.
    liquid_down: Vec<f64>,
    /// Total liquid drawn OFF stage `j` (a side draw, or the bottoms at the
    /// reboiler); zero elsewhere.
    liquid_drawn: Vec<f64>,
    /// Uniform vapour rate `V = (R+1)·D` [mol/s].
    vapour: f64,
    /// Molar reflux `L = R·D` [mol/s]. The molar distillate `D = V − L` is an
    /// ITERATE rather than an input — see `StageCascade`'s docs — but only the
    /// reflux appears in a stage row, through the condenser fold.
    reflux: f64,
}

/// Everything about a cascade column that does not change between iterations: the
/// validated geometry, the draw locations, and the mass ratios.
struct CascadePlan {
    stages: usize,
    /// Zero-based index of the feed stage.
    feed_stage: usize,
    locations: Vec<DrawLocation>,
    /// Each draw's share of the feed MASS. Exact, sums to 1 by subtraction.
    mass_ratios: Vec<f64>,
    /// Stage index → the draw that leaves from it, if any.
    draw_at_stage: Vec<Option<usize>>,
    components: usize,
}

impl CascadePlan {
    /// Validate the geometry once, so the iteration below can read it without a
    /// guard on every access.
    ///
    /// The loader refuses the same things at load time with file vocabulary; this
    /// is the model refusing them on its own behalf, because a `SeparationModel`
    /// can be constructed by anything (M7.1's correction 3 — a contract kept in
    /// another crate is not a contract).
    fn new(spec: &CascadeSpec, draws: &[ColumnDraw], components: usize) -> Result<Self, SimError> {
        let stages = spec.stages as usize;
        if stages == 0 {
            return Err(SimError::Scenario(
                "stage cascade has 0 stages: a column needs at least one equilibrium stage, \
                 and stage N is the reboiler (the total condenser is not a stage)."
                    .into(),
            ));
        }
        if spec.feed_stage < 1 || spec.feed_stage as usize > stages {
            return Err(SimError::Scenario(format!(
                "stage cascade feed_stage = {} is outside 1..={stages}: stages are numbered \
                 from the top, and stage {stages} is the reboiler.",
                spec.feed_stage
            )));
        }
        if !spec.reflux_ratio.is_finite() || spec.reflux_ratio < 0.0 {
            return Err(SimError::Scenario(format!(
                "stage cascade reflux_ratio = {} must be finite and >= 0 (0 is a column run \
                 with no reflux, which is legal and is the single-stage flash at stages = 1).",
                spec.reflux_ratio
            )));
        }
        if draws.len() < 2 {
            return Err(SimError::Scenario(format!(
                "stage cascade has {} draw(s): a column needs a distillate at stage 0 and a \
                 bottoms at stage {stages}, at minimum.",
                draws.len()
            )));
        }

        let last = draws.len() - 1;
        let mut locations = Vec::with_capacity(draws.len());
        let mut mass_ratios = Vec::with_capacity(draws.len());
        let mut draw_at_stage = vec![None; stages];
        let mut previous_stage: Option<u32> = None;
        let mut declared_total = 0.0;

        for (i, draw) in draws.iter().enumerate() {
            if draw.upper_cut.is_some() {
                return Err(SimError::Scenario(format!(
                    "stage cascade: draw {i} carries an upper_cut (a boiling-range top), which \
                     belongs to the cut-point splitter. This fidelity locates a draw by STAGE; \
                     the two are mutually exclusive by design so neither can carry a field the \
                     other ignores."
                )));
            }
            let stage = draw.stage.ok_or_else(|| {
                SimError::Scenario(format!(
                    "stage cascade: draw {i} declares no stage. Every draw needs one: 0 is the \
                     total condenser (the distillate), {stages} is the reboiler (the bottoms), \
                     and 1..{stages} are liquid side draws."
                ))
            })?;
            if let Some(previous) = previous_stage {
                if stage <= previous {
                    return Err(SimError::Scenario(format!(
                        "stage cascade: draw {i} is at stage {stage}, not strictly below the \
                         previous draw's stage {previous}. Draws are listed top-down, which is \
                         also lightest-first — the same ordering the splitter's cut points use."
                    )));
                }
            } else if stage != 0 {
                return Err(SimError::Scenario(format!(
                    "stage cascade: the first draw is at stage {stage}, not 0. Stage 0 is the \
                     total condenser and its draw is the distillate; a column without one is \
                     not a column this fidelity models."
                )));
            }
            previous_stage = Some(stage);
            if i == last {
                if stage as usize != stages {
                    return Err(SimError::Scenario(format!(
                        "stage cascade: the last draw is at stage {stage}, not {stages}. The \
                         heaviest draw is the bottoms and leaves the reboiler, which is stage \
                         {stages} by this workspace's convention (the reboiler counts, the \
                         total condenser does not)."
                    )));
                }
            } else if stage as usize >= stages {
                return Err(SimError::Scenario(format!(
                    "stage cascade: draw {i} is at stage {stage} but is not the last draw. Only \
                     the bottoms may leave stage {stages}."
                )));
            }

            locations.push(if stage == 0 {
                DrawLocation::Condenser
            } else {
                DrawLocation::Stage(stage as usize - 1)
            });

            // The bottoms is `1 − Σ others` and is never declared — fork 3.
            if i == last {
                if draw.draw_ratio.is_some() {
                    return Err(SimError::Scenario(
                        "stage cascade: the last (bottoms) draw declares a draw_ratio. It must \
                         not: the bottoms is 1 − Σ of the others by subtraction, which is what \
                         makes Σ splitᵢ = 1 an identity of the specification rather than of the \
                         arithmetic."
                            .into(),
                    ));
                }
                mass_ratios.push(1.0 - declared_total);
            } else {
                let ratio = draw.draw_ratio.ok_or_else(|| {
                    SimError::Scenario(format!(
                        "stage cascade: draw {i} declares no draw_ratio. Every draw but the \
                         bottoms needs one — a MASS fraction of the feed (D/F for the \
                         distillate, S_i/F for a side draw)."
                    ))
                })?;
                if !ratio.is_finite() || ratio <= 0.0 || ratio >= 1.0 {
                    return Err(SimError::Scenario(format!(
                        "stage cascade: draw {i} has draw_ratio = {ratio}, which must be finite \
                         and strictly inside (0, 1). A zero draw is a draw that is not there; a \
                         ratio of 1 leaves no bottoms."
                    )));
                }
                declared_total += ratio;
                mass_ratios.push(ratio);
            }
        }

        let bottoms = mass_ratios[last];
        if bottoms <= 0.0 {
            return Err(SimError::Scenario(format!(
                "stage cascade: the declared draw ratios sum to {declared_total}, leaving \
                 {bottoms} for the bottoms. They must sum to strictly less than 1 — the \
                 bottoms is what is left over, and a column that draws off its whole feed \
                 above the reboiler has no bottoms product."
            )));
        }

        for (i, location) in locations.iter().enumerate() {
            if let DrawLocation::Stage(j) = location {
                draw_at_stage[*j] = Some(i);
            }
        }

        Ok(Self {
            stages,
            feed_stage: spec.feed_stage as usize - 1,
            locations,
            mass_ratios,
            draw_at_stage,
            components,
        })
    }

    /// Each draw's composition at the current profile.
    ///
    /// The distillate is `y_1 = K_1·x_1`, because a TOTAL condenser condenses all
    /// of stage 1's vapour — that is the fold this whole formulation rests on, and
    /// it is why the condenser is not a stage. Every other draw is liquid off its
    /// own stage.
    fn draw_states(
        &self,
        liquid: &[Vec<f64>],
        k: &[Vec<f64>],
        slate: &Slate,
    ) -> Result<Vec<DrawState>, SimError> {
        let mut states = Vec::with_capacity(self.locations.len());
        for location in &self.locations {
            let moles = match location {
                DrawLocation::Condenser => {
                    let vapour: Vec<f64> =
                        liquid[0].iter().zip(&k[0]).map(|(x, k)| x * k).collect();
                    MoleFractions::from_amounts(&vapour).map_err(|e| {
                        SimError::Numerical(format!(
                            "stage cascade: the total condenser received no valid vapour \
                             composition from stage 1 ({e})"
                        ))
                    })?
                }
                DrawLocation::Stage(j) => MoleFractions::from_amounts(&liquid[*j])?,
            };
            let mean_molar_mass = moles.mean_molar_mass(slate).value();
            states.push(DrawState {
                moles,
                mean_molar_mass,
            });
        }
        Ok(states)
    }

    /// The molar flow profile implied by the current draw compositions.
    ///
    /// Constant molar overflow with a saturated-liquid feed: the vapour rate is
    /// `V = (R+1)·D` on every stage, and the liquid rate steps up by `F` at the
    /// feed stage and down by each side draw. The bottoms rate is **not** taken
    /// from its mass ratio here but from the mole balance `B = L_in + F − V` at
    /// the reboiler, so the profile is internally consistent at every iteration
    /// and the mismatch against the declared bottoms mass shows up where fork 4
    /// wants it: in the per-component residual.
    fn flows(
        &self,
        spec: &CascadeSpec,
        draws: &[DrawState],
        feed_mass: f64,
        feed_molar: f64,
    ) -> Result<Flows, SimError> {
        let last = draws.len() - 1;
        let mut drawn = vec![0.0; self.stages];
        let mut molar_rates = vec![0.0; draws.len()];
        for i in 0..last {
            molar_rates[i] = self.mass_ratios[i] * feed_mass / draws[i].mean_molar_mass;
        }
        for (j, slot) in self.draw_at_stage.iter().enumerate() {
            if let Some(i) = slot {
                if *i != last {
                    drawn[j] = molar_rates[*i];
                }
            }
        }

        let distillate = molar_rates[0];
        let reflux = spec.reflux_ratio * distillate;
        let vapour = (spec.reflux_ratio + 1.0) * distillate;

        let mut liquid_in = vec![0.0; self.stages];
        let mut liquid_down = vec![0.0; self.stages];
        liquid_in[0] = reflux;
        for j in 0..self.stages - 1 {
            let feed_here = if j == self.feed_stage {
                feed_molar
            } else {
                0.0
            };
            let down = liquid_in[j] + feed_here - drawn[j];
            if !down.is_finite() || down < 0.0 {
                return Err(SimError::Numerical(format!(
                    "stage cascade: the liquid leaving stage {} is {down:.4e} mol/s, which is \
                     negative. The side draws above it remove more liquid than the reflux and \
                     the feed supply — an infeasible specification, not a failed solve.",
                    j + 1
                )));
            }
            liquid_down[j] = down;
            liquid_in[j + 1] = down;
        }

        let reboiler = self.stages - 1;
        let feed_here = if reboiler == self.feed_stage {
            feed_molar
        } else {
            0.0
        };
        // `L_in + F − V` at the reboiler is algebraically `F − D − Σ S`, because
        // `V − L = D` telescopes through the whole column. The local form is
        // written here because it is the statement AT the reboiler; the global
        // form is what the message below explains, since it is the one a reader
        // can act on.
        let bottoms = liquid_in[reboiler] + feed_here - vapour;
        if !bottoms.is_finite() || bottoms <= 0.0 {
            let taken: f64 = molar_rates[..last].iter().sum();
            return Err(SimError::Numerical(format!(
                "stage cascade: the draws above the reboiler are carrying {taken:.4e} mol/s \
                 away from a feed of {feed_molar:.4e} mol/s, so the bottoms rate would be \
                 {bottoms:.4e} mol/s. Note the draw ratios are MASS ratios while these are \
                 MOLE rates, so this can happen even though the mass ratios sum to less than \
                 1: a distillate rich in the light cuts has a smaller mean molar mass than the \
                 feed. A CONVERGED split cannot violate it — every kilogram out came from a \
                 kilogram in, so the moles out cannot exceed the moles in — which makes this \
                 an iterate that has left the physical region rather than an answer that is \
                 merely wrong. Refused rather than clamped: a negative bottoms rate produces a \
                 profile that is finite, deterministic and meaningless."
            )));
        }
        if !distillate.is_finite() || distillate <= 0.0 {
            return Err(SimError::Numerical(format!(
                "stage cascade: the molar distillate rate is {distillate:.4e} mol/s"
            )));
        }
        drawn[reboiler] = bottoms;
        liquid_down[reboiler] = 0.0;

        Ok(Flows {
            liquid_in,
            liquid_down,
            liquid_drawn: drawn,
            vapour,
            reflux,
        })
    }

    /// One component's liquid mole fractions down the column, by the Thomas
    /// algorithm on the stage material balances.
    ///
    /// For an interior stage `j`, with `y = K·x` and no holdup:
    ///
    /// ```text
    ///   L_{j−1}·x_{j−1} − [(L_j + U_j) + V·K_j]·x_j + V·K_{j+1}·x_{j+1} = −F·z·[j = feed]
    /// ```
    ///
    /// The **total condenser folds into stage 1**: all of stage 1's vapour becomes
    /// liquid, of which `L` returns as reflux and `D` leaves as distillate, so
    /// `x_0 = y_1 = K_1·x_1` and the first row's off-diagonal above collapses into
    /// its diagonal, leaving `−[(L_1 + U_1) + D·K_1]` — because `V − L = D`. That
    /// substitution is the whole reason `N` excludes the condenser (note
    /// correction 3): there is no row for it.
    ///
    /// The reboiler row is the same equation with `L_N = 0` and `U_N = B`: what
    /// leaves it downward is the bottoms product, and what leaves it upward is the
    /// boilup `V·K_N`.
    fn solve_component(
        &self,
        flows: &Flows,
        k: &[Vec<f64>],
        component: usize,
        feed_fraction: f64,
        feed_molar: f64,
    ) -> Result<Vec<f64>, SimError> {
        debug_assert!(component < self.components);
        let n = self.stages;
        let mut sub = vec![0.0; n];
        let mut diagonal = vec![0.0; n];
        let mut sup = vec![0.0; n];
        let mut rhs = vec![0.0; n];

        for j in 0..n {
            let k_here = k[j][component];
            let leaving = flows.liquid_down[j] + flows.liquid_drawn[j];
            diagonal[j] = -(leaving + flows.vapour * k_here);
            if j == 0 {
                // The condenser fold: + L·K_1, so the diagonal becomes
                // −[(L_1 + U_1) + D·K_1] since V − L = D.
                diagonal[0] += flows.reflux * k_here;
            } else {
                sub[j] = flows.liquid_in[j];
            }
            if j + 1 < n {
                sup[j] = flows.vapour * k[j + 1][component];
            }
            if j == self.feed_stage {
                rhs[j] = -feed_molar * feed_fraction;
            }
        }

        // Thomas: forward elimination, then back substitution.
        let mut sup_prime = vec![0.0; n];
        let mut rhs_prime = vec![0.0; n];
        let mut denominator = diagonal[0];
        for j in 0..n {
            if j > 0 {
                denominator = diagonal[j] - sub[j] * sup_prime[j - 1];
            }
            if !denominator.is_finite() || denominator == 0.0 {
                return Err(SimError::Numerical(format!(
                    "stage cascade: stage {} of {n} has a singular material balance for \
                     component {component} (pivot {denominator}). A stage with no liquid and \
                     no vapour leaving it has no composition to solve for.",
                    j + 1
                )));
            }
            sup_prime[j] = sup[j] / denominator;
            rhs_prime[j] = if j == 0 {
                rhs[0] / denominator
            } else {
                (rhs[j] - sub[j] * rhs_prime[j - 1]) / denominator
            };
        }
        let mut x = vec![0.0; n];
        x[n - 1] = rhs_prime[n - 1];
        for j in (0..n - 1).rev() {
            x[j] = rhs_prime[j] - sup_prime[j] * x[j + 1];
        }
        Ok(x)
    }

    /// The worst per-component mismatch between what the declared MASS ratios say
    /// leaves the column and what the feed brought in, as a fraction of the feed.
    ///
    /// This is the number fork 4 makes a **gate**. It is not zero by construction
    /// at this fidelity, and that is the sharp break from the splitter: the molar
    /// balance the tridiagonal system enforces is exact, but the bottoms *mass*
    /// comes from `1 − Σ others` while its molar rate comes from the mole balance,
    /// and the two agree only at the fixed point.
    fn component_residual(&self, draws: &[DrawState], feed: &Composition, slate: &Slate) -> f64 {
        let mut worst = 0.0f64;
        for c in 0..self.components {
            let mut out = 0.0;
            for (i, draw) in draws.iter().enumerate() {
                // A draw's MASS composition, which is the basis its ratio is in.
                let numerator = draw.moles.fractions()[c] * slate.get(c).molar_mass.value();
                out += self.mass_ratios[i] * numerator / draw.mean_molar_mass;
            }
            worst = worst.max((out - feed.fractions()[c]).abs());
        }
        worst
    }
}

/// The largest fraction of the feed that may be off-phase before the constant-
/// molar-overflow formulation is refused.
///
/// **This is the model's own admissibility bound, and it is one number with a
/// stated basis rather than a tolerance picked to pass** (`euler-truncation-
/// tolerance`). Constant molar overflow admits a **saturated-liquid** feed only:
/// feed quality `q` sets the stripping-section liquid rate through
/// `L' = L + q·F`, so a feed off its bubble point does not merely carry a
/// different enthalpy — it changes the cascade the profile is solved on
/// (DESIGN §5, correction 5). `q` as a parameter is deferred with tray
/// hydraulics and Murphree efficiency.
///
/// The mapping from a temperature offset to a quality error is the flash it
/// represents: a liquid `ΔT` above its bubble point carries `c̄p·ΔT` J/mol of
/// excess enthalpy, which vaporizes `c̄p·ΔT/λ̄` of it at the feed stage. So the
/// admissible window is `ΔT_max = ε·λ̄/c̄p`, which is a *derived* number that
/// differs per feed rather than a constant in Kelvin — on the M7.3 fixture's
/// slate it is about 1.2 K, on a slate of heavier cuts it is wider.
///
/// `ε = 1%` is the chosen half of it, and what makes it a measurement rather than
/// a preference is `the_window_bounds_an_error_the_model_cannot_show_you`. That
/// test also names the uncomfortable fact about this guard: the feed temperature
/// reaches **nothing** in this formulation except the duties' feed-enthalpy term
/// — there is no `q`, so the profile, the splits and every draw composition are
/// bit-identical across the whole window. The violation is therefore invisible to
/// the model, which is exactly why it must be *checked* rather than observed, and
/// why the only quantity `ε` can be calibrated against is the reboiler duty (it
/// shifts by `ε·(F/V)·(λ̄(z)/λ̄(y₁))`, about `0.93·ε` on that fixture). The number
/// is quoted in the refusal so an operator can act on it.
///
/// **Two-sided.** A SUBCOOLED feed is equally inadmissible and for the mirror
/// reason — it condenses extra reflux at the feed stage, `q > 1` — so the same
/// window applies below the bubble point. Refusing only superheat would let
/// half the violation through silently.
const MAX_FEED_PHASE_ERROR: f64 = 0.01;

/// Refuse a feed that is not a saturated liquid.
///
/// This is the asymmetry DESIGN §5 argues for next to M7.3's correction 5: an
/// **idle** column is a state the cascade *can* answer, so refusing it would
/// make fidelity change legality for nothing; a feed off its bubble point is a
/// state it *cannot* answer, and the rule for that is already this workspace's
/// (`ConstantThermo::k_value` errs rather than returning a plausible number).
/// Refuse what you cannot answer; never refuse what you can.
fn check_saturated_liquid_feed(
    slate: &Slate,
    thermo: &dyn ThermoModel,
    feed: &[f64],
    bubble: Kelvin,
    pass: &ColumnPass<'_>,
) -> Result<(), SimError> {
    let latent = mixture_dh_vap(slate, thermo, feed, bubble)?;
    let heat_capacity = mixture_molar_cp(slate, feed);
    if !heat_capacity.is_finite() || heat_capacity <= 0.0 {
        return Err(SimError::Numerical(format!(
            "stage cascade: the feed's molar heat capacity is {heat_capacity:.4e} J/(mol·K), so \
             there is no window in which it counts as a saturated liquid."
        )));
    }
    let window = MAX_FEED_PHASE_ERROR * latent / heat_capacity;
    let offset = pass.temperature.value() - bubble.value();
    if !offset.is_finite() || offset.abs() > window {
        let (word, consequence) = if offset > 0.0 {
            (
                "superheated",
                "it would flash at the feed stage, so part of the feed arrives as vapour",
            )
        } else {
            (
                "subcooled",
                "it would condense extra reflux at the feed stage",
            )
        };
        let fraction = (heat_capacity * offset / latent).abs();
        return Err(SimError::Scenario(format!(
            "stage cascade: the feed reaches this column at {:.2} K, which is {:.2} K {word} \
             against its own bubble point of {:.2} K at {:.4e} Pa. Constant molar overflow \
             admits a SATURATED-LIQUID feed only — feed quality q sets the internal liquid rate \
             through L' = L + q·F, so {consequence}, and that changes the cascade rather than \
             just an enthalpy term. About {:.1}% of the feed is off-phase against an admissible \
             {:.1}%, which is a window of ±{window:.2} K here (ε·Δh_vap/cp, so it is wider on a \
             heavier slate). Bring the feed to {:.2} K, or wait for feed quality q — deferred \
             with tray hydraulics and Murphree efficiency (DESIGN §5, \"Deferred from M7\").",
            pass.temperature.value(),
            offset.abs(),
            bubble.value(),
            pass.pressure.value(),
            fraction * 100.0,
            MAX_FEED_PHASE_ERROR * 100.0,
            bubble.value(),
        )));
    }
    Ok(())
}

/// The molar latent heat of a mixture [J/mol]: `Σ_c x_c·Δh_vap_c`.
///
/// Linear in MOLE fraction — an ideal solution has no excess enthalpy of mixing,
/// which is the same assumption Raoult's law already makes in `k_value`. Mixing
/// on mass fractions instead would be the M4.2 slip (units, not the ODE) with
/// the two bases swapped.
fn mixture_dh_vap(
    slate: &Slate,
    thermo: &dyn ThermoModel,
    fractions: &[f64],
    temperature: Kelvin,
) -> Result<f64, SimError> {
    let mut total = 0.0;
    for (c, x) in fractions.iter().enumerate() {
        total += x * thermo.dh_vap(slate, c, temperature)?.value();
    }
    if !total.is_finite() || total <= 0.0 {
        return Err(SimError::Numerical(format!(
            "stage cascade: a mixture's heat of vaporization came out as {total:.4e} J/mol"
        )));
    }
    Ok(total)
}

/// The molar heat capacity of a LIQUID mixture [J/(mol·K)]: `Σ_c x_c·M_c·cp_c`.
///
/// `PseudoComponent::cp` is per KILOGRAM, so the molar mass is the bridge and it
/// is inside the sum rather than applied to the result — `Σ x_c M_c cp_c` is not
/// `M̄·Σ x_c cp_c` unless every cut has the same molar mass, and the M7.3 fixture
/// exists precisely because they do not.
///
/// Liquid only, which is all this needs: every draw is a liquid and the internal
/// vapour is only ever *condensed* here, never carried as a sensible stream.
fn mixture_molar_cp(slate: &Slate, fractions: &[f64]) -> f64 {
    fractions
        .iter()
        .enumerate()
        .map(|(c, x)| {
            let component = slate.get(c);
            x * component.molar_mass.value() * component.cp.value()
        })
        .sum()
}

/// Every component's K at every stage of a temperature profile.
fn stage_k_values_profile(
    slate: &Slate,
    thermo: &dyn ThermoModel,
    stage_t: &[f64],
    pressure: Pascal,
) -> Result<Vec<Vec<f64>>, SimError> {
    let mut k = Vec::with_capacity(stage_t.len());
    for (j, t) in stage_t.iter().enumerate() {
        k.push(stage_k_values(slate, thermo, Kelvin(*t), pressure, j + 1)?);
    }
    Ok(k)
}

/// Every component's K at one stage, guarded.
///
/// The guard is here rather than trusted from the model for the reason
/// `flash_isothermal` gives: `ThermoModel` is a trait and an implementation can
/// live in another crate, so a non-finite or non-positive K must be refused where
/// it enters rather than surfacing as a meaningless profile.
fn stage_k_values(
    slate: &Slate,
    thermo: &dyn ThermoModel,
    temperature: Kelvin,
    pressure: Pascal,
    stage: usize,
) -> Result<Vec<f64>, SimError> {
    let mut k = Vec::with_capacity(slate.len());
    for c in 0..slate.len() {
        let value = thermo.k_value(slate, c, temperature, pressure)?;
        if !value.is_finite() || value <= 0.0 {
            return Err(SimError::NonFiniteState {
                location: format!(
                    "thermo model '{}' returned K = {value} for '{}' on cascade stage {stage} \
                     at {} K, {} Pa; a K-value must be finite and > 0",
                    thermo.name(),
                    slate.get(c).name,
                    temperature.value(),
                    pressure.value()
                ),
            });
        }
        k.push(value);
    }
    Ok(k)
}

fn max_abs_difference(a: &[Vec<f64>], b: &[Vec<f64>]) -> f64 {
    a.iter()
        .zip(b)
        .flat_map(|(row_a, row_b)| row_a.iter().zip(row_b))
        .map(|(x, y)| (x - y).abs())
        .fold(0.0f64, f64::max)
}

/// The refusals, each reached on purpose.
///
/// Separate from `tests/reference/cascade.rs`, which polices the *algebra*: these
/// are the plants the cascade declines to solve at all. Rule 5 says a model that
/// cannot answer says so, and DESIGN §5 fork 0 says which scope boundaries are
/// refused rather than half-supported — a refusal nothing reaches is not a
/// boundary, it is a comment.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::ConstantAlphaThermo;
    use refinery_core::components::{Phase, PseudoComponent};
    use refinery_core::graph::NodeId;
    use refinery_core::units::{JPerKgK, JPerMol, KgPerM3, KgPerMol, KgPerSec, P_ATM};
    use std::cell::Cell;

    fn slate() -> Slate {
        Slate::new(
            [("light", 350.0, 0.100), ("heavy", 450.0, 0.200)]
                .iter()
                .map(|(name, tb, molar_mass)| PseudoComponent {
                    name: (*name).into(),
                    tb: Kelvin(*tb),
                    molar_mass: KgPerMol(*molar_mass),
                    density: Some(KgPerM3(800.0)),
                    cp: JPerKgK(2000.0),
                    phase: Phase::Liquid,
                })
                .collect(),
        )
        .unwrap()
    }

    /// The bubble point of this fixture's 50/50 mass feed at `P_ATM` [K].
    ///
    /// **Hand-computed, and it is the fixture's feed temperature** — from M7.4b a
    /// cascade refuses a feed off its bubble point, so `400.0` (which these tests
    /// used through M7.4a, and which is 16 K superheated) is no longer a plant
    /// this model admits. Written out rather than bisected for, so the constant is
    /// a second derivation and not a copy of the solver's:
    ///
    /// ```text
    ///   mole fractions of a 50/50 MASS mix at M = 0.1, 0.2:  x = [2/3, 1/3]
    ///   Σ K_c(T)·x_c = (T/400)^10 · [2·(2/3) + 0.5·(1/3)] = 1.5·(T/400)^10
    ///   = 1  ⇒  T = 400·(2/3)^0.1 = 384.1056 K
    /// ```
    const FEED_BUBBLE_K: f64 = 384.105_6;

    fn thermo(slate: &Slate) -> ConstantAlphaThermo {
        ConstantAlphaThermo::with_temperature_exponent(slate, vec![2.0, 0.5], Kelvin(400.0), 10.0)
            .unwrap()
            // A duty needs a latent heat, and these refusal fixtures reach the
            // duty calculation on the healthy path.
            .with_dh_vap(slate, vec![30_000.0, 40_000.0])
            .unwrap()
    }

    /// The bubble point of a mass composition, by bisection on the published
    /// `k_value` — for the bespoke fixtures below, whose slates and K vectors
    /// differ from `slate()`'s and so cannot share `FEED_BUBBLE_K`.
    ///
    /// A cascade refuses a feed off its bubble point (M7.4b), so a refusal test
    /// that wants to reach some *other* guard has to get past this one first.
    /// That ordering is deliberate — the feed's admissibility is checked before
    /// anything is solved — and it means these fixtures now assert their guard
    /// fires on a plant the formulation actually admits, which is strictly more
    /// than they asserted before.
    fn saturated_feed(slate: &Slate, thermo: &dyn ThermoModel, feed: &Composition) -> Kelvin {
        let moles = MoleFractions::from_mass(feed, slate).unwrap();
        let sum_kx = |t: f64| -> f64 {
            moles
                .fractions()
                .iter()
                .enumerate()
                .map(|(c, x)| x * thermo.k_value(slate, c, Kelvin(t), P_ATM).unwrap())
                .sum()
        };
        let (mut low, mut high) = (100.0_f64, 1500.0_f64);
        assert!(sum_kx(low) < 1.0 && sum_kx(high) > 1.0);
        for _ in 0..200 {
            let mid = 0.5 * (low + high);
            if sum_kx(mid) < 1.0 {
                low = mid;
            } else {
                high = mid;
            }
        }
        Kelvin(0.5 * (low + high))
    }

    fn spec(stages: u32, feed_stage: u32, reflux_ratio: f64) -> CascadeSpec {
        CascadeSpec {
            stages,
            feed_stage,
            reflux_ratio,
        }
    }

    /// A distillate and a bottoms — the shape every refusal below is a single
    /// deviation from.
    fn healthy_draws(stages: u32) -> Vec<ColumnDraw> {
        vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
            ColumnDraw::by_stage(NodeId(2), stages, None),
        ]
    }

    fn attempt(
        spec: Option<&CascadeSpec>,
        draws: &[ColumnDraw],
        feed_flow: f64,
    ) -> Result<Separation, SimError> {
        let slate = slate();
        let thermo = thermo(&slate);
        let feed = Composition::from_weights(&[0.5, 0.5]).unwrap();
        StageCascade::new().separate(
            &ColumnPass {
                slate: &slate,
                draws,
                smearing: Kelvin(0.0),
                pressure: P_ATM,
                feed: &feed,
                feed_flow: KgPerSec(feed_flow),
                temperature: Kelvin(FEED_BUBBLE_K),
                cascade: spec,
                seed: None,
            },
            &thermo,
        )
    }

    fn refusal(spec: Option<&CascadeSpec>, draws: &[ColumnDraw], feed_flow: f64) -> String {
        attempt(spec, draws, feed_flow)
            .expect_err("this column must be refused")
            .to_string()
    }

    /// The fixture the refusals deviate from actually solves. Without this, every
    /// assertion below would be satisfied by a cascade that refuses everything.
    /// A `ThermoModel` that counts K-value evaluations.
    ///
    /// The cost claim this slice makes is a claim about how many times
    /// `Σ K·x − 1` is evaluated per bubble point, and wall time cannot make it:
    /// it moves with the machine, the build profile and whatever else is running.
    /// The counter is the probe's own unit (`ROADMAP`, M9.3), made into a gate.
    struct CountingThermo {
        inner: ConstantAlphaThermo,
        k_values: Cell<u32>,
    }

    impl ThermoModel for CountingThermo {
        fn name(&self) -> &'static str {
            self.inner.name()
        }

        fn k_value(
            &self,
            slate: &Slate,
            component: usize,
            temperature: Kelvin,
            pressure: Pascal,
        ) -> Result<f64, SimError> {
            self.k_values.set(self.k_values.get() + 1);
            self.inner.k_value(slate, component, temperature, pressure)
        }

        fn dh_vap(
            &self,
            slate: &Slate,
            component: usize,
            temperature: Kelvin,
        ) -> Result<JPerMol, SimError> {
            self.inner.dh_vap(slate, component, temperature)
        }

        fn bubble_pressure(
            &self,
            slate: &Slate,
            composition: &refinery_core::components::Composition,
            temperature: Kelvin,
        ) -> Result<Pascal, SimError> {
            self.inner.bubble_pressure(slate, composition, temperature)
        }
    }

    /// `Σ K·x = 1` has a closed form under this fixture's thermo, so the search
    /// can be gated against algebra rather than against its own previous answer.
    ///
    /// ```text
    ///   Σ K_c(T)·x_c = (T/400)^10 · Σ α_c·x_c = 1  ⇒  T = 400·(Σ α_c·x_c)^(−1/10)
    /// ```
    fn exact_bubble_point(x: &[f64]) -> f64 {
        let alpha = [2.0, 0.5];
        let sum: f64 = x.iter().zip(alpha).map(|(xc, a)| a * xc).sum();
        400.0 * sum.powf(-0.1)
    }

    /// The search lands on the root, to the resolution its own constant claims.
    ///
    /// **This is the gate that stops the resolution being loosened**, and it is
    /// the reason M9.3a took its speed out of the method instead. `DEFERRED.md`
    /// A2 proposed stopping the old bisection at the caller's `1e-6` tolerance;
    /// that would land ~4e-4 K from the root here and fail this by nine orders of
    /// magnitude, which is the point — the cascade's outer test DIFFERENCES two of
    /// these, so a coarse bubble point gives that difference a noise floor at
    /// exactly the bar it is graded against.
    ///
    /// The bound is derived, not fitted: the loop exits with a bracket no wider
    /// than `2·ε·T` and returns its midpoint, so the answer is within `ε·T` of the
    /// root plus whatever the excess evaluation's own rounding costs (that is
    /// `noise/slope ≈ 2e-16/0.0375 ≈ 5e-15` K here). `4·ε·T` covers both with the
    /// test's own `powf` rounding to spare.
    #[test]
    fn the_bubble_point_lands_on_the_root_the_algebra_gives() {
        let slate = slate();
        let thermo = thermo(&slate);
        // The feed mixture, and a pure light cut — one composition could agree
        // with the closed form by coincidence, two of them at different roots
        // (384 K and 373 K) cannot.
        for x in [vec![2.0 / 3.0, 1.0 / 3.0], vec![1.0, 0.0]] {
            let expected = exact_bubble_point(&x);
            let found =
                bubble_temperature_unnormalized(&slate, &thermo, &x, P_ATM, "the gate's fixture")
                    .expect("the fixture's excess crosses zero inside the bracket")
                    .value();
            let bound = 4.0 * f64::EPSILON * expected;
            assert!(
                (found - expected).abs() <= bound,
                "bubble point {found:.15e} K against the closed form {expected:.15e} K:                  off by {:.3e} K, bound {bound:.3e} K",
                (found - expected).abs()
            );
        }
    }

    /// ...and reaches it in fewer than half the evaluations bisection needs.
    ///
    /// **Both halves in one test on purpose.** A cost gate alone is passed by a
    /// search that stops early, and an accuracy gate alone is passed by the 60
    /// halvings this slice replaced; the claim is the conjunction, so the
    /// assertion is too.
    ///
    /// The bound is bisection's own number rather than a chosen one: closing
    /// `[50, 2000]` to `2·ε·T` takes 55 halvings at this root, and the loop
    /// M9.3a replaced spent a fixed 60. `30` is half of that, so this fails the
    /// moment the method degrades to bisection class — it is not sized to today's
    /// measurement, which is 15.
    #[test]
    fn the_bubble_point_beats_bisection_on_evaluations() {
        let slate = slate();
        let thermo = CountingThermo {
            inner: thermo(&slate),
            k_values: Cell::new(0),
        };
        let x = vec![2.0 / 3.0, 1.0 / 3.0];
        let expected = exact_bubble_point(&x);

        let found =
            bubble_temperature_unnormalized(&slate, &thermo, &x, P_ATM, "the gate's fixture")
                .expect("the fixture's excess crosses zero inside the bracket")
                .value();

        // One evaluation of `Σ K·x − 1` is one K-value per component.
        let evaluations = thermo.k_values.get() / slate.len() as u32;
        assert!(
            (found - expected).abs() <= 4.0 * f64::EPSILON * expected,
            "the cheap answer must still be the right one: {found:.15e} K against              {expected:.15e} K"
        );
        // 20, against a measured 15. The bound exists to catch the loss of the
        // LOGARITHM, which is the whole speed-up: dropping it costs 38, 54 or 65
        // depending on which bisection safeguard is left in place, so every way
        // of reverting it clears 20 by at least 18 evaluations.
        //
        // What this gate deliberately does NOT catch is a change to the
        // safeguard, because with the logarithm in place all three variants cost
        // 15 or 16 and there is no speed to defend. The safeguard buys the
        // worst-case bound behind `BUBBLE_POINT_MAX_EVALUATIONS`, and a bound on
        // a slate nobody has run yet is not a thing an evaluation count on this
        // fixture can measure. Recorded rather than papered over with a bound
        // tight enough to fire on 16 — that would be fitting a constant to this
        // one composition.
        assert!(
            evaluations <= 20,
            "the bubble point spent {evaluations} evaluations of the excess function;              15 is the measured cost, bisection needs 55 to reach the same bracket width,              and the loop this replaced spent a fixed 60"
        );
    }

    #[test]
    fn the_healthy_fixture_solves() {
        attempt(Some(&spec(4, 2, 2.0)), &healthy_draws(4), 10.0)
            .expect("the fixture the refusal tests deviate from must itself be a valid column");
    }

    /// A cut-point column handed to the cascade has no equipment, and the message
    /// says which key is missing rather than dividing by a zero stage count.
    #[test]
    fn a_column_with_no_cascade_equipment_is_refused() {
        let message = refusal(None, &healthy_draws(4), 10.0);
        assert!(
            message.contains("cascade") && message.contains("reflux_ratio"),
            "the refusal must name the missing config, got: {message}"
        );
    }

    /// A negative or non-finite feed is refused. It cannot come from the sweep —
    /// `column_feed_flow` sums INFLOWS — so the only caller who can produce one is
    /// a hand-built pass, which is exactly what this test is.
    #[test]
    fn a_negative_feed_is_refused() {
        for feed_flow in [-3.0, f64::NAN] {
            let message = refusal(Some(&spec(4, 2, 2.0)), &healthy_draws(4), feed_flow);
            assert!(
                message.contains("negative or non-finite column feed"),
                "a feed of {feed_flow} must be refused by cause, got: {message}"
            );
        }
    }

    /// **M7.1's correction 4, discharged — and the discharge is a convention, not
    /// a refusal.**
    ///
    /// An idle column returns its declared splits rather than an `Err`, for two
    /// reasons that point the same way. `column_feed_flow` reports a zero for a
    /// column running BACKWARDS *and* for one with nothing flowing at all, and
    /// only the first is a fault — the engine's own post-sweep guard is what names
    /// it, and correction 4's whole complaint was that a cascade would fail before
    /// that guard was reached. And the cut-point splitter handles an idle column
    /// perfectly well, so refusing here would make the two fidelities disagree
    /// about which plants are legal — the hazard `ConstantAlphaThermo::k_value`'s
    /// own comment names, in the other direction.
    ///
    /// Asserted against the SPLITTER rather than against a literal, so the parity
    /// is the claim rather than a number that happens to match today.
    #[test]
    fn an_idle_column_matches_the_splitter_instead_of_failing() {
        let slate = slate();
        let feed = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let cascade_draws = healthy_draws(4);
        let spec = spec(4, 2, 2.0);
        let idle = attempt(Some(&spec), &cascade_draws, 0.0)
            .expect("an idle cascade column is idle, not broken");

        let split_draws = [
            ColumnDraw::by_cut(NodeId(1), Some(Kelvin(400.0))),
            ColumnDraw::by_cut(NodeId(2), None),
        ];
        let splitter = crate::CutPointSplitter
            .separate(
                &ColumnPass {
                    slate: &slate,
                    draws: &split_draws,
                    smearing: Kelvin(0.0),
                    pressure: P_ATM,
                    feed: &feed,
                    feed_flow: KgPerSec(0.0),
                    temperature: Kelvin(400.0),
                    cascade: None,
                    seed: None,
                },
                &thermo(&slate),
            )
            .expect("the splitter has always been fine with an idle column");

        assert_eq!(idle.draws.len(), splitter.draws.len());
        let total: f64 = idle.draws.iter().map(|d| d.split).sum();
        approx::assert_abs_diff_eq!(total, 1.0, epsilon = 1e-15);
        assert_eq!(idle.draws[0].split, 0.3, "the declared distillate ratio");
        for d in &idle.draws {
            assert_eq!(
                d.composition.fractions(),
                feed.fractions(),
                "nothing flows, so a draw carries the feed as an inert placeholder"
            );
            assert_eq!(d.temperature, Kelvin(FEED_BUBBLE_K));
        }
        // M7.4b: `Some(ZERO)`, not `None`. An idle cascade column HAS a condenser
        // and a reboiler with nothing to do, which is a different statement from
        // the splitter's "there is no such equipment here" — and the two would be
        // indistinguishable if this fidelity reported an absence.
        assert_eq!(idle.condenser_duty, Some(Watt::ZERO));
        assert_eq!(idle.reboiler_duty, Some(Watt::ZERO));
        assert_eq!(splitter.condenser_duty, None);
        assert_eq!(splitter.reboiler_duty, None);
    }

    /// A malformed cascade is refused whether or not anything is flowing: the
    /// geometry is validated before the feed rate is looked at, so which plants
    /// are legal never depends on how much is going through them.
    #[test]
    fn an_idle_column_is_still_validated() {
        let broken = vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
            ColumnDraw::by_stage(NodeId(2), 3, None),
        ];
        assert!(refusal(Some(&spec(4, 2, 2.0)), &broken, 0.0).contains("not 4"));
    }

    /// The geometry: a stage count, a feed stage inside it, and a reflux ratio that
    /// is a ratio.
    #[test]
    fn an_impossible_geometry_is_refused() {
        let draws = healthy_draws(4);
        assert!(refusal(Some(&spec(0, 1, 2.0)), &draws, 10.0).contains("0 stages"));
        assert!(refusal(Some(&spec(4, 0, 2.0)), &draws, 10.0).contains("feed_stage"));
        assert!(refusal(Some(&spec(4, 5, 2.0)), &draws, 10.0).contains("feed_stage"));
        assert!(refusal(Some(&spec(4, 2, -1.0)), &draws, 10.0).contains("reflux_ratio"));
        assert!(refusal(Some(&spec(4, 2, f64::NAN)), &draws, 10.0).contains("reflux_ratio"));
    }

    /// **The declared-iff-used correspondence, in its inverse direction.** A draw
    /// carrying a boiling-range top is a splitter's draw; this fidelity locates a
    /// draw by stage. Neither may carry the other's field, so that a file cannot
    /// declare something the running model silently ignores (DESIGN §5, fork 2).
    #[test]
    fn a_draw_carrying_the_other_fidelity_field_is_refused() {
        let mixed = vec![
            ColumnDraw::by_cut(NodeId(1), Some(Kelvin(400.0))),
            ColumnDraw::by_cut(NodeId(2), None),
        ];
        let message = refusal(Some(&spec(4, 2, 2.0)), &mixed, 10.0);
        assert!(
            message.contains("upper_cut") && message.contains("STAGE"),
            "the refusal must name the field and the fidelity that owns it, got: {message}"
        );

        // The heaviest draw of a splitter column carries NO upper_cut, so it slips
        // past the check above and lands on the missing-stage arm instead. Both
        // arms are needed; a column refused only by the first would still be
        // reachable through its own last draw.
        let no_stage = vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
            ColumnDraw::by_cut(NodeId(2), None),
        ];
        assert!(refusal(Some(&spec(4, 2, 2.0)), &no_stage, 10.0).contains("declares no stage"));
    }

    /// The draw list must start at the condenser, end at the reboiler, and descend
    /// the column in between — the same top-down ordering the splitter's cut points
    /// use, which is also lightest-first.
    #[test]
    fn a_mislocated_draw_is_refused() {
        let one_draw = vec![ColumnDraw::by_stage(NodeId(1), 0, None)];
        assert!(refusal(Some(&spec(4, 2, 2.0)), &one_draw, 10.0).contains("at minimum"));

        let no_distillate = vec![
            ColumnDraw::by_stage(NodeId(1), 1, Some(0.3)),
            ColumnDraw::by_stage(NodeId(2), 4, None),
        ];
        assert!(refusal(Some(&spec(4, 2, 2.0)), &no_distillate, 10.0).contains("not 0"));

        let short_bottoms = vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
            ColumnDraw::by_stage(NodeId(2), 3, None),
        ];
        assert!(refusal(Some(&spec(4, 2, 2.0)), &short_bottoms, 10.0).contains("not 4"));

        let out_of_order = vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.2)),
            ColumnDraw::by_stage(NodeId(2), 3, Some(0.2)),
            ColumnDraw::by_stage(NodeId(3), 2, Some(0.2)),
            ColumnDraw::by_stage(NodeId(4), 4, None),
        ];
        assert!(refusal(Some(&spec(4, 2, 2.0)), &out_of_order, 10.0).contains("strictly below"));

        let side_draw_below_the_reboiler = vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.2)),
            ColumnDraw::by_stage(NodeId(2), 4, Some(0.2)),
            ColumnDraw::by_stage(NodeId(3), 5, None),
        ];
        assert!(
            refusal(Some(&spec(4, 2, 2.0)), &side_draw_below_the_reboiler, 10.0)
                .contains("Only the bottoms")
        );
    }

    /// Every draw but the bottoms declares a mass ratio; the bottoms declares none
    /// and gets `1 − Σ others`. That asymmetry is what makes `Σ splitᵢ = 1` an
    /// identity of the specification (fork 3), so both halves of it are refused.
    #[test]
    fn a_misdeclared_draw_ratio_is_refused() {
        let bottoms_declared = vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
            ColumnDraw::by_stage(NodeId(2), 4, Some(0.7)),
        ];
        assert!(refusal(Some(&spec(4, 2, 2.0)), &bottoms_declared, 10.0).contains("must not"));

        let distillate_bare = vec![
            ColumnDraw::by_stage(NodeId(1), 0, None),
            ColumnDraw::by_stage(NodeId(2), 4, None),
        ];
        assert!(refusal(Some(&spec(4, 2, 2.0)), &distillate_bare, 10.0)
            .contains("declares no draw_ratio"));

        for bad in [0.0, 1.0, -0.2, 1.5, f64::NAN] {
            let out_of_range = vec![
                ColumnDraw::by_stage(NodeId(1), 0, Some(bad)),
                ColumnDraw::by_stage(NodeId(2), 4, None),
            ];
            assert!(
                refusal(Some(&spec(4, 2, 2.0)), &out_of_range, 10.0).contains("(0, 1)"),
                "a draw ratio of {bad} must be refused"
            );
        }

        let nothing_left_over = vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.6)),
            ColumnDraw::by_stage(NodeId(2), 2, Some(0.5)),
            ColumnDraw::by_stage(NodeId(3), 4, None),
        ];
        assert!(refusal(Some(&spec(4, 2, 2.0)), &nothing_left_over, 10.0).contains("bottoms"));
    }

    /// The bottoms rate going non-positive — and **finding a case that reaches it
    /// corrected what the guard was for.**
    ///
    /// The first version of this test tried to reach it with a big reflux ratio,
    /// on the reasoning that a hard-boiling column runs its reboiler dry. That is
    /// wrong, and the algebra says so: `V − L = D` telescopes, so the bottoms rate
    /// is `F − D − Σ S` and does not depend on `R` at all. Nor can a *converged*
    /// split ever breach it, because every kilogram out came from a kilogram in
    /// and so the moles out cannot exceed the moles in.
    ///
    /// What does reach it is an **iterate** that has left the physical region:
    /// a slate whose cuts are 20× apart in molar mass, a volatility of 80, and a
    /// distillate ratio of half the mass of a feed that is only a tenth light. The
    /// cascade's intermediate distillate is then nearly pure light, whose mean
    /// molar mass is small enough that half the feed's MASS is more than all of
    /// its MOLES. That is exactly the coupling correction 1 introduced by putting
    /// `D/F` on a mass basis, and it is why the guard cannot be replaced by
    /// validating the ratios at load.
    #[test]
    fn a_non_positive_bottoms_rate_is_refused() {
        let wide_slate = Slate::new(
            [("light", 350.0, 0.020), ("heavy", 450.0, 0.400)]
                .iter()
                .map(|(name, tb, molar_mass)| PseudoComponent {
                    name: (*name).into(),
                    tb: Kelvin(*tb),
                    molar_mass: KgPerMol(*molar_mass),
                    density: Some(KgPerM3(800.0)),
                    cp: JPerKgK(2000.0),
                    phase: Phase::Liquid,
                })
                .collect(),
        )
        .unwrap();
        let wide_thermo = ConstantAlphaThermo::with_temperature_exponent(
            &wide_slate,
            vec![8.0, 0.1],
            Kelvin(400.0),
            10.0,
        )
        .unwrap()
        .with_dh_vap(&wide_slate, vec![30_000.0, 40_000.0])
        .unwrap();
        let draws = vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.5)),
            ColumnDraw::by_stage(NodeId(2), 2, None),
        ];
        let spec = spec(2, 1, 0.1);
        let feed = Composition::from_weights(&[0.1, 0.9]).unwrap();
        let message = StageCascade::new()
            .separate(
                &ColumnPass {
                    slate: &wide_slate,
                    draws: &draws,
                    smearing: Kelvin(0.0),
                    pressure: P_ATM,
                    feed: &feed,
                    feed_flow: KgPerSec(10.0),
                    temperature: saturated_feed(&wide_slate, &wide_thermo, &feed),
                    cascade: Some(&spec),
                    seed: None,
                },
                &wide_thermo,
            )
            .expect_err("this column drives its own iterate out of the physical region")
            .to_string();
        assert!(
            message.contains("bottoms rate would be") && message.contains("MASS ratios"),
            "the refusal must report the rate and name the basis mismatch behind it,              got: {message}"
        );
    }

    /// Side draws above the feed that take more liquid than the reflux supplies.
    /// The reboiler guard above cannot see this one — it fires further up the
    /// column — and a plant guarded only at the reboiler would reach a negative
    /// liquid rate first (`a-fixed-plant-cannot-gate-a-second-door`).
    #[test]
    fn side_draws_that_dry_the_column_are_refused() {
        let draws = vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.1)),
            ColumnDraw::by_stage(NodeId(2), 1, Some(0.4)),
            ColumnDraw::by_stage(NodeId(3), 2, Some(0.4)),
            ColumnDraw::by_stage(NodeId(4), 6, None),
        ];
        let message = refusal(Some(&spec(6, 5, 0.2)), &draws, 10.0);
        assert!(
            message.contains("negative") && message.contains("liquid leaving stage"),
            "a column drained above its feed must be refused by cause, got: {message}"
        );
    }

    /// A thermo fidelity with no phase equilibrium fails the cascade rather than
    /// letting it run on a default. This is the pairing the loader refuses at load
    /// in M7.3's second commit; the model's own arm is what stands behind it.
    #[test]
    fn a_thermo_with_no_k_value_fails_the_cascade() {
        let slate = slate();
        let feed = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let draws = healthy_draws(4);
        let spec = spec(4, 2, 2.0);
        let result = StageCascade::new().separate(
            &ColumnPass {
                slate: &slate,
                draws: &draws,
                smearing: Kelvin(0.0),
                pressure: P_ATM,
                feed: &feed,
                feed_flow: KgPerSec(10.0),
                temperature: Kelvin(400.0),
                cascade: Some(&spec),
                seed: None,
            },
            &crate::ConstantThermo,
        );
        assert!(
            result.is_err(),
            "a cascade on a fidelity with no vapour-liquid equilibrium must fail"
        );
    }

    // -----------------------------------------------------------------------
    // M7.4b — the saturated-liquid feed becomes a guard.
    // -----------------------------------------------------------------------

    /// A pass at `offset` Kelvin from this fixture's feed bubble point.
    fn at_feed_offset(offset: f64) -> Result<Separation, SimError> {
        let slate = slate();
        let thermo = thermo(&slate);
        let feed = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let draws = healthy_draws(4);
        let spec = spec(4, 2, 2.0);
        StageCascade::new().separate(
            &ColumnPass {
                slate: &slate,
                draws: &draws,
                smearing: Kelvin(0.0),
                pressure: P_ATM,
                feed: &feed,
                feed_flow: KgPerSec(10.0),
                temperature: Kelvin(FEED_BUBBLE_K + offset),
                cascade: Some(&spec),
                seed: None,
            },
            &thermo,
        )
    }

    /// **The saturated-liquid feed is a precondition, and from M7.4b it is
    /// enforced.**
    ///
    /// Constant molar overflow admits a saturated liquid only: feed quality `q`
    /// sets the stripping-section liquid rate through `L' = L + q·F`, so a feed
    /// off its bubble point changes the *cascade*, not just an enthalpy term. Both
    /// directions are refused — a superheated feed flashes at the feed stage, a
    /// subcooled one condenses extra reflux — and refusing only the first would
    /// let half the violation through.
    ///
    /// This is the asymmetry DESIGN §5 argues next to M7.3's correction 5: an
    /// idle column is a state this model CAN answer and so is not refused; a feed
    /// it cannot answer for is. Refuse what you cannot answer; never refuse what
    /// you can.
    #[test]
    fn a_feed_off_its_bubble_point_is_refused_in_both_directions() {
        for (offset, word) in [(20.0, "superheated"), (-20.0, "subcooled")] {
            let message = at_feed_offset(offset)
                .expect_err("constant molar overflow admits a saturated-liquid feed only")
                .to_string();
            assert!(
                message.contains(word),
                "the refusal must name which way the feed is off, got: {message}"
            );
            assert!(
                message.contains("bubble point") && message.contains("384."),
                "the refusal must report the bubble point the feed should be at, got: {message}"
            );
            assert!(
                message.contains("quality q"),
                "the refusal must name the deferral that would un-defer it (DESIGN §5, \
                 \"Deferred from M7\"), got: {message}"
            );
        }
        // The window is a WINDOW, not a demand for the exact float: a scenario
        // author cannot be expected to type a bubble point to sixteen digits, and
        // a guard that required it would be unusable rather than strict.
        at_feed_offset(0.5).expect("a feed a half-kelvin off saturation is still admissible");
        at_feed_offset(-0.5).expect("and the same below");
    }

    /// **What the window bounds is an error this model does not represent at all
    /// — which is the sharpest possible reason for it to be a refusal.**
    ///
    /// Trace the feed temperature through `separate` and it reaches exactly three
    /// places: this guard, the idle placeholder, and the feed enthalpy term of the
    /// duties. It does **not** reach the seed (that is the feed's bubble point,
    /// not its resolved temperature), the flow profile, the stage balances or the
    /// K-values — because there is no `q` in this formulation. So a superheated
    /// feed does not make the cascade produce a slightly different answer; it
    /// makes it produce the *same* answer to a different question.
    ///
    /// That is measured here rather than argued, and it is the whole shape of the
    /// gate:
    ///
    /// 1. Every draw composition is **bit-identical** across the window, and so is
    ///    the condenser duty. `assert_eq!`, not a tolerance — a tolerance here
    ///    would be the vacuous-control shape (`a-control-can-be-implied-by-its-
    ///    assertion`), since these numbers cannot move at all.
    /// 2. The reboiler duty is the one thing that does move, by exactly
    ///    `−ṁ_F·cp_F·ΔT` — it carries the feed enthalpy. Asserted against that
    ///    closed form, not a bound.
    /// 3. At the edge of the window that shift is `ε·(F/V)·(λ̄(z)/λ̄(y₁))` ≈ 0.93%
    ///    of the duty for `ε = 1%`. **This is what makes `ε` a measured number
    ///    rather than a preference**: the one quantity the violation is visible in
    ///    moves by about `ε`, so the knob means what its name says.
    ///
    /// The first point is the reason for the third. Because the model is blind to
    /// the violation everywhere else, no amount of running it can reveal a feed
    /// that is off-model — which is precisely why the precondition has to be
    /// checked rather than observed.
    #[test]
    fn the_window_bounds_an_error_the_model_cannot_show_you() {
        let slate = slate();
        let saturated = at_feed_offset(0.0).expect("the fixture is admissible on its bubble point");

        // ε·Δh_vap/c̄p for this fixture's feed — the guard's own derivation,
        // written out here so the two are independent:
        //   z = [2/3, 1/3];  Δh_vap = [30 000, 40 000];  M·cp = [200, 400] J/(mol·K)
        //   λ̄  = (2/3)·30 000 + (1/3)·40 000 = 33 333.3 J/mol
        //   c̄p = (2/3)·200    + (1/3)·400    =    266.67 J/(mol·K)
        //   window = 0.01·33 333.3/266.67 = 1.25 K
        let window = 1.25;
        let feed = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let feed_heat_capacity = 10.0 * feed.mixture_cp(&slate).value(); // ṁ_F·cp_F [W/K]

        for offset in [window * 0.98, -window * 0.98, 0.5, -0.5] {
            let moved = at_feed_offset(offset)
                .unwrap_or_else(|e| panic!("{offset} K is inside the window: {e}"));

            // (1) The profile cannot see the feed temperature.
            for (i, (a, b)) in saturated.draws.iter().zip(&moved.draws).enumerate() {
                assert_eq!(
                    a.composition.fractions(),
                    b.composition.fractions(),
                    "draw {i}'s composition moved with the feed temperature ({offset:+} K). \
                     Nothing in the stage balances reads it, so this can only mean feed \
                     quality has been wired in — at which point the guard is the wrong \
                     mechanism and `q` is the right one."
                );
                assert_eq!(a.temperature, b.temperature, "draw {i}'s tray temperature");
            }
            assert_eq!(
                saturated.condenser_duty, moved.condenser_duty,
                "the condenser duty is a function of the profile alone"
            );

            // (2) The reboiler duty carries the feed enthalpy, exactly.
            let expected = saturated.reboiler_duty.unwrap().value() - feed_heat_capacity * offset;
            approx::assert_relative_eq!(
                moved.reboiler_duty.unwrap().value(),
                expected,
                max_relative = 1e-12
            );
        }

        // (3) And at the edge, that shift is about ε — which is what the knob
        //     claims to mean. Bracketed rather than pinned: the ratio is
        //     `ε·(F/V)·(λ̄(z)/λ̄(y₁))`, near 1 for an ordinary column but not equal
        //     to it, and pinning it would be pinning this fixture's reflux ratio.
        let shift = feed_heat_capacity * window / saturated.reboiler_duty.unwrap().value();
        assert!(
            (0.5 * MAX_FEED_PHASE_ERROR..2.0 * MAX_FEED_PHASE_ERROR).contains(&shift),
            "at the edge of the window the reboiler duty moves by {shift:.4e}, which is not \
             the same order as the {:.1}% the window admits being off-phase. Either the \
             window derivation or the duty has drifted from what ε names.",
            MAX_FEED_PHASE_ERROR * 100.0
        );
    }

    /// The window is **derived per feed**, not a constant in Kelvin: it is
    /// `ε·Δh_vap/c̄p`, so a slate of heavier cuts (more latent heat per mole for
    /// the same heat capacity) admits a wider temperature band.
    ///
    /// This is what stops the bound from being a magic number. A fixed `±1.2 K`
    /// would mean something quite different on a slate whose latent heat is twice
    /// as large, and the guard would be strict in one plant and loose in another
    /// for no physical reason.
    #[test]
    fn the_admissible_window_scales_with_the_feeds_own_latent_heat() {
        let slate = slate();
        let feed = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let draws = healthy_draws(4);
        let spec = spec(4, 2, 2.0);

        // The same K-values, so the same bubble point and the same profile — only
        // the latent heats are doubled.
        let attempt = |dh: Vec<f64>, offset: f64| {
            let thermo = ConstantAlphaThermo::with_temperature_exponent(
                &slate,
                vec![2.0, 0.5],
                Kelvin(400.0),
                10.0,
            )
            .unwrap()
            .with_dh_vap(&slate, dh)
            .unwrap();
            StageCascade::new()
                .separate(
                    &ColumnPass {
                        slate: &slate,
                        draws: &draws,
                        smearing: Kelvin(0.0),
                        pressure: P_ATM,
                        feed: &feed,
                        feed_flow: KgPerSec(10.0),
                        temperature: Kelvin(FEED_BUBBLE_K + offset),
                        cascade: Some(&spec),
                        seed: None,
                    },
                    &thermo,
                )
                .is_ok()
        };

        // 2.0 K is outside the ~1.25 K window of the shipped latent heats and
        // inside the ~2.5 K window of doubled ones. One offset, two verdicts —
        // which is the whole claim, and neither half alone would make it.
        assert!(
            !attempt(vec![30_000.0, 40_000.0], 2.0),
            "2 K must be outside the window a 33 kJ/mol feed admits"
        );
        assert!(
            attempt(vec![60_000.0, 80_000.0], 2.0),
            "2 K must be inside the window a 67 kJ/mol feed admits"
        );
    }

    /// **The duty sign convention, pushed at rather than asserted.**
    ///
    /// `Separation`'s duties are non-negative MAGNITUDES with the direction in the
    /// name (the `Furnace`/`Cooler` convention), so `duties` refuses a negative
    /// one. That refusal is not reachable by anything here, and this test is the
    /// record of trying: five configurations chosen to drive the reboiler duty
    /// below zero — inverted relative volatility so the distillate is the HEAVY
    /// cut, a near-total distillate at `D/F = 0.95`, zero reflux, and a latent
    /// heat of 1 J/mol to shrink the term the sensible balance is added to.
    ///
    /// All five stay positive. The closest is `D/F = 0.9` at `R = 0`, where the
    /// reboiler exceeds the condenser by 3.7 kW on a 311 kW duty. The reason is
    /// structural rather than lucky: the feed is a saturated liquid, so it sits
    /// *inside* the column's own temperature profile, and no split of it across
    /// draws that straddle it has made the mass-weighted draw enthalpy fall below
    /// the feed's by more than the boilup carries.
    ///
    /// So the guard stays and its docstring claims only what this measured —
    /// **not shown reachable**, which is a weaker statement than `flash.rs` makes
    /// when it deletes an unreachable check ("`beta` is 0.0, 1.0, or a midpoint,
    /// so there is no path on which it is not finite" — a proof, which this is
    /// not). What the test itself gates is the positive claim: this sign
    /// convention holds where it is hardest to hold.
    #[test]
    fn the_duties_stay_non_negative_where_it_is_hardest() {
        let slate = slate();
        let feed = Composition::from_weights(&[0.5, 0.5]).unwrap();
        let mut closest = f64::INFINITY;

        for (dh, d_over_f, reflux) in [
            (1.0, 0.9, 2.0),
            (1.0, 0.9, 0.0),
            (1.0, 0.8, 0.0),
            (10.0, 0.9, 0.0),
            (1.0, 0.95, 0.0),
        ] {
            // Inverted volatility: the heavier cut is the volatile one, so the
            // distillate is heavy and its MOLAR rate stays under the feed's even
            // at `D/F = 0.95` — which is what lets the mass ratio go this high
            // without the bottoms guard firing first.
            let thermo = ConstantAlphaThermo::with_temperature_exponent(
                &slate,
                vec![0.5, 2.0],
                Kelvin(400.0),
                10.0,
            )
            .unwrap()
            .with_dh_vap(&slate, vec![dh, dh])
            .unwrap();
            let draws = vec![
                ColumnDraw::by_stage(NodeId(1), 0, Some(d_over_f)),
                ColumnDraw::by_stage(NodeId(2), 4, None),
            ];
            let spec = spec(4, 2, reflux);
            let separation = StageCascade::new()
                .separate(
                    &ColumnPass {
                        slate: &slate,
                        draws: &draws,
                        smearing: Kelvin(0.0),
                        pressure: P_ATM,
                        feed: &feed,
                        feed_flow: KgPerSec(10.0),
                        temperature: saturated_feed(&slate, &thermo, &feed),
                        cascade: Some(&spec),
                        seed: None,
                    },
                    &thermo,
                )
                .unwrap_or_else(|e| {
                    panic!("Δh_vap = {dh}, D/F = {d_over_f}, R = {reflux} must solve: {e}")
                });

            let condenser = separation.condenser_duty.unwrap().value();
            let reboiler = separation.reboiler_duty.unwrap().value();
            assert!(
                condenser >= 0.0 && reboiler >= 0.0,
                "Δh_vap = {dh}, D/F = {d_over_f}, R = {reflux}: got a condenser at \
                 {condenser} W and a reboiler at {reboiler} W"
            );
            closest = closest.min(reboiler - condenser);
        }

        // The margin, so that a change which makes the refusal REACHABLE shows up
        // here as a number closing rather than as a guard quietly gaining its
        // first caller. `a-counter-is-not-a-gate` in the other direction: this
        // records how far the threshold is from being crossed.
        assert!(
            closest > 0.0 && closest < 1.0e4,
            "the closest this slate comes to a negative reboiler duty is {closest} W. If that \
             has gone far positive the configurations above have stopped being adversarial; \
             if it has gone negative, `duties`' refusal is now reachable and owes a test that \
             reaches it deliberately."
        );
    }
}
