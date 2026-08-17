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
use refinery_core::error::SimError;
use refinery_core::graph::{CascadeSpec, ColumnDraw};
use refinery_core::traits::{ColumnPass, DrawSeparation, Separation, SeparationModel, ThermoModel};
use refinery_core::units::{Kelvin, Pascal, Watt};

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

/// The bracket every stage's bubble point is sought in [K], and the number of
/// halvings of it.
///
/// Wide on purpose: a cut's boiling point is slate data and a column's pressure is
/// operator config, so the bubble point of a mixture is not bounded by anything
/// this module knows. `1950·2⁻⁶⁰ ≈ 1.7e-15` K is below the spacing of an `f64`
/// here, so the bracket collapses to adjacent floats and the loop is a fixed cost
/// with no convergence arm of its own — the same argument `flash::BISECTION_STEPS`
/// makes.
const BUBBLE_POINT_LOW_K: f64 = 50.0;
const BUBBLE_POINT_HIGH_K: f64 = 2000.0;
const BUBBLE_POINT_STEPS: u32 = 60;

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

        // The feed FLOW, which the splitter never read. `ColumnPass::feed_flow` is
        // the sweep's inflow SUM, so a column running backwards arrives here as a
        // zero rather than a negative number, and `Engine::tick`'s reverse-feed
        // refusal fires only AFTER the sweep — this call is inside it. M7.1's
        // correction 4 predicted exactly this: without the guard below, a zero feed
        // makes every internal molar flow zero and the first stage row singular, so
        // the column would fail on linear algebra before the guard that names the
        // cause is ever reached. This is that revisit.
        let feed_flow = pass.feed_flow.value();
        if !feed_flow.is_finite() || feed_flow <= 0.0 {
            return Err(SimError::Numerical(format!(
                "stage cascade has a non-positive column feed ({feed_flow:.4e} kg/s): the \
                 cascade's internal flows are all proportional to it, so there is no profile \
                 to solve. A zero here is what the sweep reports for a column whose feed runs \
                 BACKWARDS; the engine's own reverse-feed refusal names that case, and it \
                 fires after the sweep this call is inside."
            )));
        }

        let plan = CascadePlan::new(spec, pass.draws, slate.len())?;
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
        let seed = bubble_point(slate, thermo, feed.fractions(), pass.pressure, "the feed")?;
        let mut stage_t = vec![seed.value() + self.seed_offset.value(); plan.stages];
        let mut liquid: Vec<Vec<f64>> = vec![feed.fractions().to_vec(); plan.stages];

        let mut iterations = 0u32;
        let mut residual = f64::INFINITY;
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

            let profile_change = max_abs_difference(&liquid, &next);
            liquid = next;

            // Bubble point per stage, at the NEW liquid composition.
            let mut next_t = Vec::with_capacity(plan.stages);
            for (j, x) in liquid.iter().enumerate() {
                next_t.push(
                    bubble_point(
                        slate,
                        thermo,
                        x,
                        pass.pressure,
                        &format!("stage {} of {}", j + 1, plan.stages),
                    )?
                    .value(),
                );
            }
            let temperature_change = stage_t
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
            // Fork 5: an `Err`, never a held previous profile. Holding one would
            // make the column's output depend on tick history, which destroys the
            // property that makes a column's reference a clean hand calculation.
            return Err(SimError::Numerical(format!(
                "stage cascade did not converge in {iterations} iterations: worst \
                 per-component mass residual {residual:.3e} kg/s against a bound of \
                 {COMPONENT_RESIDUAL_KG_PER_S:.3e} kg/s (I7's 1e-6 kg over a 0.1 s tick). \
                 The profile is not held over from a previous tick — a column's answer must \
                 not depend on run length."
            )));
        }

        // The distillate leaves the total condenser as a saturated liquid, so its
        // temperature is its OWN bubble point, not stage 1's. Computed once here
        // rather than inside the loop: nothing in the iteration reads it.
        let condenser_t = bubble_point(
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

        Ok(Separation {
            draws: result,
            // See the type's docs: both need `Δh_vap`, which is M7.4.
            condenser_duty: Watt::ZERO,
            reboiler_duty: Watt::ZERO,
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

/// The temperature at which a liquid of composition `x` boils at `pressure`:
/// `Σ_c K_c(T)·x_c = 1`.
///
/// Bisection on a wide fixed bracket, and **no root in the bracket is an `Err`**
/// naming the model — never a fallback temperature. That refusal is what makes
/// this function safe to hand an arbitrary `ThermoModel`, and it is the arm a
/// K-value with no temperature dependence at all lands in: `Σ K·x` is then a
/// constant, so the equation has no solution unless the constant happens to be 1.
/// The alternative — detect a flat model and hold some convention — cannot
/// distinguish "K does not depend on T" from "the root is outside my bracket",
/// and the second case is the finite-deterministic-plausible-wrong shape this
/// workspace keeps catching.
fn bubble_point(
    slate: &Slate,
    thermo: &dyn ThermoModel,
    x: &[f64],
    pressure: Pascal,
    what: &str,
) -> Result<Kelvin, SimError> {
    let excess = |t: f64| -> Result<f64, SimError> {
        let mut sum = 0.0;
        for (c, xc) in x.iter().enumerate() {
            let k = thermo.k_value(slate, c, Kelvin(t), pressure)?;
            if !k.is_finite() || k <= 0.0 {
                return Err(SimError::NonFiniteState {
                    location: format!(
                        "thermo model '{}' returned K = {k} for '{}' at {t} K while finding the \
                         bubble point of {what}",
                        thermo.name(),
                        slate.get(c).name
                    ),
                });
            }
            sum += k * xc;
        }
        Ok(sum - 1.0)
    };

    let low = excess(BUBBLE_POINT_LOW_K)?;
    let high = excess(BUBBLE_POINT_HIGH_K)?;
    if low > 0.0 || high < 0.0 {
        return Err(SimError::Numerical(format!(
            "stage cascade: {what} has no bubble point between {BUBBLE_POINT_LOW_K} K and \
             {BUBBLE_POINT_HIGH_K} K at {} Pa under thermo model '{}': Σ K·x − 1 is \
             {low:.4e} at the bottom of the bracket and {high:.4e} at the top, so it never \
             crosses zero. A model whose K-values do not depend on temperature lands here by \
             construction, and it cannot drive a cascade: a stage's temperature IS its bubble \
             point.",
            pressure.value(),
            thermo.name()
        )));
    }

    let (mut low_t, mut high_t) = (BUBBLE_POINT_LOW_K, BUBBLE_POINT_HIGH_K);
    for _ in 0..BUBBLE_POINT_STEPS {
        let middle = 0.5 * (low_t + high_t);
        if excess(middle)? <= 0.0 {
            low_t = middle;
        } else {
            high_t = middle;
        }
    }
    Ok(Kelvin(0.5 * (low_t + high_t)))
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
    use refinery_core::units::{JPerKgK, KgPerM3, KgPerMol, KgPerSec, P_ATM};

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

    fn thermo(slate: &Slate) -> ConstantAlphaThermo {
        ConstantAlphaThermo::with_temperature_exponent(slate, vec![2.0, 0.5], Kelvin(400.0), 10.0)
            .unwrap()
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
                temperature: Kelvin(400.0),
                cascade: spec,
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

    /// **M7.1's correction 4, discharged.** The sweep reports a reversed column
    /// feed as a ZERO, and `Engine::tick`'s reverse-feed refusal fires only after
    /// the sweep this call sits inside — so without this guard the cascade would
    /// reach a singular first stage and fail on linear algebra before the engine's
    /// diagnostic is ever produced. The message names the cause and points at the
    /// engine's own guard.
    #[test]
    fn a_non_positive_feed_is_refused_before_the_algebra() {
        for feed_flow in [0.0, -3.0, f64::NAN] {
            let message = refusal(Some(&spec(4, 2, 2.0)), &healthy_draws(4), feed_flow);
            assert!(
                message.contains("non-positive column feed") && message.contains("BACKWARDS"),
                "a feed of {feed_flow} must be refused by cause, got: {message}"
            );
        }
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
                    temperature: Kelvin(400.0),
                    cascade: Some(&spec),
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
            },
            &crate::ConstantThermo,
        );
        assert!(
            result.is_err(),
            "a cascade on a fidelity with no vapour-liquid equilibrium must fail"
        );
    }
}
