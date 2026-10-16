//! The heat-balance cascade — a column fed hot from its furnace (M60,
//! docs/DESIGN.md §65).
//!
//! The same equilibrium stages as [`StageCascade`](crate::StageCascade) — the
//! plan, one Thomas sweep per component, each stage at its own bubble point,
//! the draws, the per-component residual gate, the condenser duty — with one
//! thing different: **each stage's vapour comes from its own energy balance**,
//! not from constant molar overflow. That is Wang & Henke's bubble-point method
//! as it was published (*Hydrocarbon Processing* 45 (1966) 155), with the stage
//! energy balances M7.3 replaced by a uniform vapour rate.
//!
//! What it buys, measured on `crude_column_cascade.toml`'s crude (§65.0):
//!
//! - **Any feed temperature.** The feed's enthalpy enters its stage's balance,
//!   so a feed hot from its furnace (or cold) is a column this model answers.
//!   `StageCascade` refuses one more than `ε·λ̄/c̄p` off its bubble point.
//! - **A reboiler duty that agrees with its own stage.** The closure
//!   (`Q_reb = Q_cond + Σ draws − feed`) and the reboiler stage's own balance
//!   agree at the fixed point; under constant molar overflow they were a third
//!   apart on crude, and a hot feed drove them to opposite signs.
//!
//! It reads only liquid `cp` and `Δh_vap`, the two properties `column_duties`
//! already reads, so DESIGN §5 fork 1's verdict holds: no vapour density, and
//! nothing two-phase on the plant graph.

use refinery_core::components::{Composition, Slate};
use refinery_core::energy::T_REF;
use refinery_core::error::SimError;
use refinery_core::graph::CascadeSpec;
use refinery_core::traits::{
    CascadeProfile, ColumnPass, DrawSeparation, Separation, SeparationModel, ThermoModel,
};
use refinery_core::units::{Kelvin, Pascal, Watt};

use crate::bubble::bubble_temperature_unnormalized;
use crate::cascade::{
    bubble_profile, column_duties, max_abs_difference, mixture_dh_vap, mixture_molar_cp,
    stage_k_values_profile, CascadePlan, DrawLocation, DrawState, Flows,
    COMPONENT_RESIDUAL_KG_PER_S,
};
use crate::column_newton::{ColumnEquations, Solved, Specification, Start, Unsolved};
use crate::flash::flash_isothermal;
use crate::molar::MoleFractions;

/// Outer iterations on a COLD start whose draws' molar masses come from a sharp
/// split of the feed by boiling order rather than from the iterate.
///
/// **A convergence aid, measured, not physics** (DESIGN §65.0, §65.5). A draw's
/// molar rate is its MASS ratio over its current molar mass, and a first
/// iterate with every stage at the feed's composition gives the light draws
/// too many moles: on a crude with a 0.55 kg/mol residue the moles drawn above
/// the reboiler exceeded the feed's and tick 1 failed on a negative bottoms —
/// on `StageCascade` as well. Ten iterations of the sharp split started it (102
/// iterations to converge at a saturated feed); one or three did not.
const SHARP_SEED_ITERATIONS: u32 = 10;

/// The smallest vapour any stage below the top may carry, as a share of the
/// feed's molar rate [-] (DESIGN §65.3).
///
/// **A GAME VALUE**, standing in for the tray hydraulics C1 defers — a real tray
/// weeps below its turndown. What it does: when the feed brings more heat than
/// the declared reflux carries away, a stage's vapour would fall toward zero
/// and the solve stiffen (1 575 iterations, then the cap, measured in §65.0);
/// held here, the top vapour and the reflux rise instead and the column keeps
/// running — the user's decision 2. 0.20 is the measured smallest that keeps the
/// solve out of the stiff zone on the demo crude with margin; 0.05 did not.
pub(crate) const VAPOUR_FLOOR_OF_FEED: f64 = 0.20;

/// How far each iteration's vapour profile moves from the previous one toward
/// what the stage balances ask for [-].
///
/// **A convergence aid, measured, not physics**: on the crude unit (§65.9) a
/// full step from the first iterate leaves the physical region (a negative
/// liquid above the feed) where a half step does not. It moves no fixed point —
/// at convergence the relaxed and the raw profiles agree, and the converged
/// flows are taken raw.
const RELAXATION: f64 = 0.5;

/// Fixed-point iterations a COLD start runs before Newton takes over
/// (M61, docs/DESIGN.md §66.1).
///
/// **A convergence aid, measured, not physics**: on the crude unit Newton
/// converged from 30 and from 10 fixed-point iterations (4 and 12 Newton steps)
/// and left the physical region from 3 (§66.0). The fixed point carries every
/// cold aid above — the sharp seed, the first iteration on constant molar
/// overflow, the relaxation — so Newton starts inside the region they reach.
const COLD_WARM_UP_ITERATIONS: u32 = 30;

/// Outer iterations of the fixed point when it is solved to convergence on its
/// own — the reference [`HeatBalanceCascade::fixed_point`] gates Newton
/// against. M60's cap and its argument (the pinch case is the stiff one).
#[cfg_attr(not(test), allow(dead_code))]
const FIXED_POINT_MAX_ITERATIONS: u32 = 2000;

/// How a heat-balance cascade solves its equations.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Method {
    /// Newton on every temperature, vapour and draw rate at once (M61,
    /// §66.1), after a short fixed-point warm-up on a cold start.
    Newton,
    /// M60's fixed point alone, solved to convergence — kept as the reference
    /// the tests gate Newton against, where both converge.
    #[cfg_attr(not(test), allow(dead_code))]
    FixedPoint,
}

/// The heat-balance cascade: the complex separation fidelity for a column fed
/// off its bubble point, selected by `[fidelity] separation = "heat_balance"`.
///
/// Specified exactly as [`StageCascade`](crate::StageCascade) is — a molar
/// reflux ratio and one MASS ratio per draw but the bottoms (DESIGN §5 fork 3)
/// — and solved on the same stages. See the module docs for what differs.
pub struct HeatBalanceCascade {
    max_iterations: u32,
    seed_offset: Kelvin,
    method: Method,
}

impl HeatBalanceCascade {
    /// Newton steps before the solve is declared failed (M61). A warm tick
    /// takes 2; a cold start 4–12 after its warm-up (§66.0).
    pub const DEFAULT_MAX_ITERATIONS: u32 = 50;

    pub fn new() -> Self {
        Self {
            max_iterations: Self::DEFAULT_MAX_ITERATIONS,
            seed_offset: Kelvin::ZERO,
            method: Method::Newton,
        }
    }

    /// A cascade capped at `max_iterations` Newton steps (and as many warm-up
    /// iterations, if fewer than the usual) — for the gate that reaches the
    /// non-convergence `Err` on purpose.
    pub fn with_max_iterations(max_iterations: u32) -> Self {
        Self {
            max_iterations,
            ..Self::new()
        }
    }

    /// A cascade whose seed temperatures are offset by `seed_offset` — for the
    /// start-insensitivity gate (a warm start changes the iteration count, not
    /// the fixed point; DESIGN §5 fork 5).
    pub fn with_seed_offset(seed_offset: Kelvin) -> Self {
        Self {
            seed_offset,
            ..Self::new()
        }
    }

    /// M60's fixed point alone, to convergence: the reference Newton agrees
    /// with wherever both converge (§66.1).
    #[cfg(test)]
    pub(crate) fn fixed_point() -> Self {
        Self {
            max_iterations: FIXED_POINT_MAX_ITERATIONS,
            seed_offset: Kelvin::ZERO,
            method: Method::FixedPoint,
        }
    }
}

impl Default for HeatBalanceCascade {
    fn default() -> Self {
        Self::new()
    }
}

/// What a solve hands to the column's published answer: the converged
/// temperatures, liquids and flows and the reflux they ran at.
struct Profile {
    temperatures: Vec<f64>,
    liquid: Vec<Vec<f64>>,
    flows: Flows,
    reflux_ratio: f64,
}

/// One column's fixed-point state after some iterations.
struct FixedPointState {
    temperatures: Vec<f64>,
    liquid: Vec<Vec<f64>>,
    k: Vec<Vec<f64>>,
    draws: Vec<DrawState>,
    /// The last iteration's vapour profile; `None` before any.
    vapour: Option<Vec<f64>>,
    converged: bool,
    iterations: u32,
    /// What was unmet when the iterations ran out, in words.
    unmet: Vec<String>,
}

/// Everything a column's solve reads that the iterations do not change.
struct ColumnSetup<'a> {
    pass: &'a ColumnPass<'a>,
    thermo: &'a dyn ThermoModel,
    spec: &'a CascadeSpec,
    plan: CascadePlan,
    feed: MoleFractions,
    /// The feed's bubble point at the column's pressure — the cold seed.
    seed: Kelvin,
    feed_molar: f64,
    feed_enthalpy: f64,
    tolerance: f64,
}

impl SeparationModel for HeatBalanceCascade {
    fn name(&self) -> &'static str {
        "heat_balance"
    }

    fn separate(
        &self,
        pass: &ColumnPass<'_>,
        thermo: &dyn ThermoModel,
    ) -> Result<Separation, SimError> {
        let slate = pass.slate;
        let spec = pass.cascade.ok_or_else(|| {
            SimError::Scenario(
                "separation = \"heat_balance\" but this column declares no cascade equipment: \
                 a heat-balance cascade needs [nodes.<column>.cascade] with stages, feed_stage \
                 and reflux_ratio. The cut-point splitter is the fidelity that needs none."
                    .into(),
            )
        })?;

        let feed_flow = pass.feed_flow.value();
        if !feed_flow.is_finite() || feed_flow < 0.0 {
            return Err(SimError::Numerical(format!(
                "heat-balance cascade has a negative or non-finite column feed ({feed_flow:.4e} \
                 kg/s): every internal flow is proportional to it, so there is no profile to \
                 solve. `ColumnPass::feed_flow` is the sweep's INFLOW SUM and cannot be \
                 negative, so this is a caller constructing a pass by hand."
            )));
        }

        let plan = CascadePlan::new(spec, pass.draws, slate.len())?;

        // An idle column: `StageCascade`'s convention and its argument — the
        // declared splits, inert placeholders for what nothing carries, and
        // zero duties as an ANSWER rather than `None`. Returned BEFORE the feed
        // is read at all (M7.1's correction 4: never refuse what you can
        // answer) — an idle feed's composition and temperature are placeholders
        // a bubble point or a flash may not be able to evaluate. No flash share:
        // a share of nothing is not a number the model produced, so the
        // snapshot's `column_operation` is absent while the column is idle.
        if feed_flow == 0.0 {
            return Ok(Separation {
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
                condenser_duty: Some(Watt::ZERO),
                reboiler_duty: Some(Watt::ZERO),
                feed_vapour_fraction: None,
                reflux_ratio: Some(spec.reflux_ratio),
            });
        }

        let feed = MoleFractions::from_mass(pass.feed, slate)?;
        let seed = bubble_temperature_unnormalized(
            slate,
            thermo,
            feed.fractions(),
            pass.pressure,
            "the feed",
        )?;
        let feed_vapour_fraction =
            adiabatic_flash_share(slate, thermo, &feed, pass.temperature, seed, pass.pressure)?;

        let feed_molar = feed_flow / feed.mean_molar_mass(slate).value();
        // `StageCascade`'s tolerance, derived from I7 over this feed (fork 4).
        let tolerance = (COMPONENT_RESIDUAL_KG_PER_S / feed_flow).clamp(1.0e-12, 1.0e-6);
        // The feed's molar enthalpy on the engine's liquid datum — the number
        // `energy::enthalpy_flux` carries into the column, so the stage balance
        // and the plant's books read the same feed.
        let feed_enthalpy = liquid_enthalpy(slate, feed.fractions(), pass.temperature);

        // The warm start (M9.3b): both profiles or neither, a wrong-shaped seed
        // ignored — `StageCascade`'s rules.
        let warm = pass.seed.filter(|previous| {
            previous.temperatures.len() == plan.stages
                && previous.liquid.len() == plan.stages
                && previous.liquid.iter().all(|row| row.len() == slate.len())
        });
        let setup = ColumnSetup {
            pass,
            thermo,
            spec,
            plan,
            feed,
            seed,
            feed_molar,
            feed_enthalpy,
            tolerance,
        };

        let profile = match self.method {
            Method::FixedPoint => self.solve_by_fixed_point(&setup, warm)?,
            Method::Newton => self.solve_by_newton(&setup, warm)?,
        };
        let plan = &setup.plan;

        let k = stage_k_values_profile(slate, thermo, &profile.temperatures, pass.pressure)?;
        let draws = plan.draw_states(&profile.liquid, &k, slate)?;
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
                temperature: match plan.locations[i] {
                    DrawLocation::Condenser => condenser_t,
                    DrawLocation::Stage(j) => Kelvin(profile.temperatures[j]),
                },
            });
        }
        let (condenser_duty, reboiler_duty) = column_duties(
            slate,
            thermo,
            &profile.flows,
            &draws[0],
            Kelvin(profile.temperatures[0]),
            condenser_t,
            &result,
            pass,
            plan.side_heat.iter().sum(),
        )?;

        Ok(Separation {
            draws: result,
            profile: Some(CascadeProfile {
                temperatures: profile.temperatures.iter().copied().map(Kelvin).collect(),
                liquid: profile.liquid,
                vapour: profile.flows.vapour.clone(),
            }),
            condenser_duty: Some(condenser_duty),
            reboiler_duty: Some(reboiler_duty),
            feed_vapour_fraction: Some(feed_vapour_fraction),
            reflux_ratio: Some(profile.reflux_ratio),
        })
    }
}

impl HeatBalanceCascade {
    /// M60's fixed point, from `warm` or cold, for at most `cap` iterations.
    fn fixed_point_iterations(
        &self,
        setup: &ColumnSetup<'_>,
        warm: Option<&CascadeProfile>,
        cap: u32,
    ) -> Result<FixedPointState, SimError> {
        let pass = setup.pass;
        let slate = pass.slate;
        let thermo = setup.thermo;
        let plan = &setup.plan;
        let spec = setup.spec;
        let feed_flow = pass.feed_flow.value();
        let tolerance = setup.tolerance;

        let mut stage_t: Vec<f64> = match warm {
            Some(previous) => previous
                .temperatures
                .iter()
                .map(|t| t.value() + self.seed_offset.value())
                .collect(),
            None => vec![setup.seed.value() + self.seed_offset.value(); plan.stages],
        };
        let mut liquid: Vec<Vec<f64>> = match warm {
            Some(previous) => previous.liquid.clone(),
            None => vec![setup.feed.fractions().to_vec(); plan.stages],
        };
        let sharp = match warm {
            Some(_) => None,
            None => Some(sharp_split_molar_masses(plan, pass.feed, slate)),
        };

        let mut iterations = 0u32;
        let mut residual = f64::INFINITY;
        let mut profile_change = f64::INFINITY;
        let mut temperature_change = f64::INFINITY;
        let mut converged = false;
        // A warm start relaxes its first stage balance toward the previous
        // tick's vapour (`CascadeProfile::vapour`); a cold one has none.
        let mut last_vapour: Option<Vec<f64>> = warm
            .map(|previous| previous.vapour.clone())
            .filter(|vapour| vapour.len() == plan.stages);
        let mut k = stage_k_values_profile(slate, thermo, &stage_t, pass.pressure)?;
        let mut draws = plan.draw_states(&liquid, &k, slate)?;

        while iterations < cap {
            iterations += 1;

            let mut previous = plan.draw_states(&liquid, &k, slate)?;
            if let Some(masses) = &sharp {
                if iterations <= SHARP_SEED_ITERATIONS {
                    for (state, mass) in previous.iter_mut().zip(masses) {
                        if let Some(mass) = mass {
                            state.mean_molar_mass = *mass;
                        }
                    }
                }
            }
            // A cold start's first iteration on constant molar overflow: from a
            // flat seed profile the stage balances are wild — on the crude unit
            // (§65.9) a vapour 30 times the feed's, refused on tick 1.
            let flows = if warm.is_none() && iterations == 1 {
                plan.flows(spec, &previous, feed_flow, setup.feed_molar)?
            } else {
                stage_balance_flows(
                    plan,
                    spec,
                    &StageState {
                        slate,
                        thermo,
                        draws: &previous,
                        liquid: &liquid,
                        k: &k,
                        temperatures: &stage_t,
                        pressure: pass.pressure,
                        previous_vapour: last_vapour.as_deref(),
                    },
                    feed_flow,
                    setup.feed_molar,
                    setup.feed_enthalpy,
                )?
                .0
            };
            last_vapour = Some(flows.vapour.clone());

            let next = plan.liquid_profile(&flows, &k, &setup.feed, setup.feed_molar)?;
            profile_change = max_abs_difference(&liquid, &next);
            liquid = next;

            let next_t = bubble_profile(slate, thermo, &liquid, pass.pressure)?;
            temperature_change = stage_t
                .iter()
                .zip(&next_t)
                .map(|(a, b)| (a - b).abs() / b)
                .fold(0.0f64, f64::max);
            stage_t = next_t;

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

        let mut unmet = Vec::new();
        if !converged {
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
        }
        Ok(FixedPointState {
            temperatures: stage_t,
            liquid,
            k,
            draws,
            vapour: last_vapour,
            converged,
            iterations,
            unmet,
        })
    }

    /// M60's solve: the fixed point to convergence, its flows from the
    /// CONVERGED draw states (`StageCascade`'s reason: the loop's last flows are
    /// one iterate stale).
    fn solve_by_fixed_point(
        &self,
        setup: &ColumnSetup<'_>,
        warm: Option<&CascadeProfile>,
    ) -> Result<Profile, SimError> {
        let state = self.fixed_point_iterations(setup, warm, self.max_iterations)?;
        if !state.converged {
            return Err(SimError::Numerical(format!(
                "heat-balance cascade (the fixed point) did not converge in {} iterations: \
                 {}.{}",
                state.iterations,
                state.unmet.join("; "),
                settings_sentence(setup.spec)
            )));
        }
        let pass = setup.pass;
        let (flows, reflux_ratio) = stage_balance_flows(
            &setup.plan,
            setup.spec,
            &StageState {
                slate: pass.slate,
                thermo: setup.thermo,
                draws: &state.draws,
                liquid: &state.liquid,
                k: &state.k,
                temperatures: &state.temperatures,
                pressure: pass.pressure,
                previous_vapour: None,
            },
            pass.feed_flow.value(),
            setup.feed_molar,
            setup.feed_enthalpy,
        )?;
        Ok(Profile {
            temperatures: state.temperatures,
            liquid: state.liquid,
            flows,
            reflux_ratio,
        })
    }

    /// M61's solve (§66.1): Newton warm from the previous tick, or after a
    /// short fixed-point warm-up cold — and cold again if the warm one fails.
    fn solve_by_newton(
        &self,
        setup: &ColumnSetup<'_>,
        warm: Option<&CascadeProfile>,
    ) -> Result<Profile, SimError> {
        let pass = setup.pass;
        let slate = pass.slate;
        let plan = &setup.plan;
        let latent = mixture_dh_vap(slate, setup.thermo, setup.feed.fractions(), setup.seed)?;
        let equations = ColumnEquations {
            plan,
            spec: setup.spec,
            slate,
            thermo: setup.thermo,
            pressure: pass.pressure,
            feed: &setup.feed,
            feed_composition: pass.feed,
            feed_mass: pass.feed_flow.value(),
            feed_molar: setup.feed_molar,
            feed_enthalpy: setup.feed_enthalpy,
            energy_scale: setup.feed_molar * latent,
            tolerance: setup.tolerance,
        };

        let mut warm_failure = None;
        if let Some(previous) = warm.filter(|p| p.vapour.len() == plan.stages) {
            let temperatures: Vec<f64> = previous
                .temperatures
                .iter()
                .map(|t| t.value() + self.seed_offset.value())
                .collect();
            let k = stage_k_values_profile(slate, setup.thermo, &temperatures, pass.pressure)?;
            let draws = plan.draw_states(&previous.liquid, &k, slate)?;
            // A warm Newton that fails is retried cold, where the aids are.
            match self.newton(&equations, &temperatures, &previous.vapour, &draws) {
                Ok(profile) => return Ok(profile),
                Err(unsolved) => warm_failure = Some(unsolved),
            }
        }

        let refusal = |reason: &str, cause: &str| {
            SimError::Numerical(format!(
                "heat-balance cascade {reason}.{cause}{}",
                settings_sentence(setup.spec)
            ))
        };
        let cause_of =
            |unsolved: &Unsolved| physical_cause(plan, unsolved.flows.as_deref(), setup.feed_molar);
        let cap = COLD_WARM_UP_ITERATIONS.min(self.max_iterations);
        let state = match self.fixed_point_iterations(setup, None, cap) {
            Ok(state) => state,
            // The cold warm-up refused outright (an infeasible first iterate):
            // the warm attempt's last iterate, if there was one, is where the
            // edge shows.
            Err(e) => {
                return Err(match warm_failure {
                    Some(warm) => refusal(&warm.reason, &cause_of(&warm)),
                    None => refusal(&format!("could not start: {e}"), ""),
                })
            }
        };
        let vapour = state
            .vapour
            .clone()
            .unwrap_or_else(|| vec![setup.feed_molar; plan.stages]);
        match self.newton(&equations, &state.temperatures, &vapour, &state.draws) {
            Ok(profile) => Ok(profile),
            // The cold failure is reported, with the warm one's physical cause
            // where only the warm one's last iterate shows an edge.
            Err(cold) => {
                let mut cause = cause_of(&cold);
                if cause.is_empty() {
                    if let Some(warm) = &warm_failure {
                        cause = cause_of(warm);
                    }
                }
                Err(refusal(&cold.reason, &cause))
            }
        }
    }

    /// Newton under the vapour floor's active set (§66.3): the declared reflux
    /// first, and where that leaves a stage under the floor, the floor on the
    /// stage that binds with the reflux an unknown. A start that ran above its
    /// declared reflux tries the floor first, on the stage it bound.
    fn newton(
        &self,
        equations: &ColumnEquations<'_>,
        temperatures: &[f64],
        vapour: &[f64],
        draws: &[DrawState],
    ) -> Result<Profile, Unsolved> {
        let plan = equations.plan;
        let n = plan.stages;
        let draw_rates: Vec<f64> = (0..plan.mass_ratios.len() - 1)
            .map(|i| plan.mass_ratios[i] * equations.feed_mass / draws[i].mean_molar_mass)
            .collect();
        let start = Start {
            temperatures,
            vapour,
            draw_rates: &draw_rates,
        };
        let declared = equations.spec.reflux_ratio;
        let floor = VAPOUR_FLOOR_OF_FEED * equations.feed_molar;
        let holds = |solved: &Solved| {
            solved.reflux_ratio >= declared
                && solved.flows.vapour[1..]
                    .iter()
                    .all(|v| *v >= floor * (1.0 - 1.0e-9))
        };
        let lowest = |vapour: &[f64]| {
            (1..n)
                .min_by(|a, b| vapour[*a].total_cmp(&vapour[*b]).then(a.cmp(b)))
                .unwrap_or(1)
        };
        let profile = |solved: Solved| Profile {
            temperatures: solved.temperatures,
            liquid: solved.liquid,
            flows: solved.flows,
            reflux_ratio: solved.reflux_ratio,
        };

        let ran = vapour[0] / draw_rates[0] - 1.0;
        if ran > declared * (1.0 + 1.0e-6) {
            if let Ok(solved) = equations.solve(
                &start,
                Specification::Floor(lowest(vapour)),
                self.max_iterations,
            ) {
                if holds(&solved) {
                    return Ok(profile(solved));
                }
            }
        }
        let declared_solve = equations.solve(&start, Specification::Reflux, self.max_iterations);
        let mut binding = match &declared_solve {
            Ok(solved) if solved.flows.vapour[1..].iter().all(|v| *v >= floor) => {
                return declared_solve.map(profile);
            }
            Ok(solved) => lowest(&solved.flows.vapour),
            // A declared solve that failed with every stage of its start above
            // the floor did not fail on the floor: report it.
            Err(_) if vapour[1..].iter().all(|v| *v >= floor) => {
                return declared_solve.map(profile);
            }
            Err(_) => lowest(vapour),
        };
        let mut last = None;
        for _ in 1..n {
            match equations.solve(&start, Specification::Floor(binding), self.max_iterations) {
                Ok(solved) if holds(&solved) => return Ok(profile(solved)),
                Ok(solved) => {
                    match (1..n).find(|j| solved.flows.vapour[*j] < floor * (1.0 - 1.0e-9)) {
                        Some(j) if j != binding => binding = j,
                        _ => break,
                    }
                }
                Err(unsolved) => {
                    last = Some(unsolved);
                    break;
                }
            }
        }
        Err(match (declared_solve, last) {
            (_, Some(unsolved)) => unsolved,
            (Err(unsolved), None) => unsolved,
            (Ok(solved), None) => Unsolved {
                reason: "found no reflux at which every stage's vapour stands at the floor".into(),
                flows: Some(Box::new(solved.flows)),
            },
        })
    }
}

/// The settings a player can act on (§65.9): past the crude unit's working
/// range the solve stops settling, and the settings it was given are the plain
/// part of the reason.
fn settings_sentence(spec: &CascadeSpec) -> String {
    let duties: Vec<String> = spec
        .pumparounds
        .iter()
        .map(|p| format!("{:.1} MW on stage {}", p.duty.value() / 1e6, p.stage))
        .collect();
    format!(
        " The column could not settle at reflux ratio {}{} — outside the range it runs in.",
        spec.reflux_ratio,
        if duties.is_empty() {
            String::new()
        } else {
            format!(" with its pumparounds at {}", duties.join(", "))
        }
    )
}

/// A physical edge the last iterate shows, in words (§66.2), or nothing: a tray
/// whose liquid has run out. (The probe's other edge, a reboiler with nothing
/// left to boil up, cannot be reached here: the vapour floor holds the
/// reboiler's boil-up at 20 % of the feed.)
///
/// A share of the feed's molar rate below which a flow counts as gone: 1 %. The
/// line search never steps a flow through zero, so a solve that stopped at an
/// edge stopped with that flow close to it — measured at exactly zero on the
/// crude unit's dry kerosene tray (§66.0).
fn physical_cause(plan: &CascadePlan, flows: Option<&Flows>, feed_molar: f64) -> String {
    const GONE_SHARE_OF_FEED: f64 = 0.01;
    let Some(flows) = flows else {
        return String::new();
    };
    let gone = GONE_SHARE_OF_FEED * feed_molar;
    let n = plan.stages;
    let driest = (0..n - 1)
        .filter(|j| flows.liquid_down[*j] <= gone)
        .min_by(|a, b| flows.liquid_down[*a].total_cmp(&flows.liquid_down[*b]));
    match driest {
        Some(j) => format!(
            " The trays at stage {} run dry: the liquid leaving it has fallen to {:.3} mol/s, \
             the draws above it taking all the liquid the column makes there.",
            j + 1,
            flows.liquid_down[j]
        ),
        None => String::new(),
    }
}

/// What one iteration's stage balances read: the profile as it stands.
struct StageState<'a> {
    slate: &'a Slate,
    thermo: &'a dyn ThermoModel,
    draws: &'a [DrawState],
    liquid: &'a [Vec<f64>],
    k: &'a [Vec<f64>],
    temperatures: &'a [f64],
    pressure: Pascal,
    /// The previous iteration's vapour profile, toward which this one is
    /// relaxed (`RELAXATION`); `None` for the raw balances.
    previous_vapour: Option<&'a [f64]>,
}

/// The molar flows from each stage's energy balance, top-down from the
/// condenser (DESIGN §65.0).
///
/// With the stage temperatures and compositions of the current iterate fixed,
/// a stage's material and energy balances leave two unknowns once the stage
/// above is known — the vapour rising into it and the liquid leaving it:
///
/// ```text
/// V_{j+1} = [L_{j−1}·(h_j − h_{j−1}) + F_j·(h_j − h_F) + V_j·(H_j − h_j)] / (H_{j+1} − h_j)
/// L_j     = L_{j−1} + V_{j+1} + F_j − U_j − V_j
/// ```
///
/// (Wang & Henke 1966; the same rearrangement as Seader & Henley's bubble-point
/// method). `h` is a liquid's molar enthalpy on the engine's datum, `H` a
/// saturated vapour's (`h(y, T) + λ̄(y)`, `y = K·x` normalised), `L₀` the reflux
/// at the total condenser's bubble point and `V₁ = (R+1)·D`. The side draws
/// `U_j` leave at their stage's liquid enthalpy, which is why they appear only
/// in the material balance. The bottoms is the reboiler's mole balance,
/// `B = L_{N−1} + F_N − V_N`, as under constant molar overflow.
///
/// Returns the flows and the reflux ratio they ran at: the declared one, or
/// above it where the vapour floor binds (`VAPOUR_FLOOR_OF_FEED`, §65.3).
fn stage_balance_flows(
    plan: &CascadePlan,
    spec: &CascadeSpec,
    state: &StageState<'_>,
    feed_mass: f64,
    feed_molar: f64,
    feed_enthalpy: f64,
) -> Result<(Flows, f64), SimError> {
    let slate = state.slate;
    let stages = plan.stages;
    let last = state.draws.len() - 1;

    let mut drawn = vec![0.0; stages];
    for (j, slot) in plan.draw_at_stage.iter().enumerate() {
        if let Some(i) = slot {
            if *i != last {
                drawn[j] = plan.mass_ratios[*i] * feed_mass / state.draws[*i].mean_molar_mass;
            }
        }
    }
    let distillate = plan.mass_ratios[0] * feed_mass / state.draws[0].mean_molar_mass;
    if !distillate.is_finite() || distillate <= 0.0 {
        return Err(SimError::Numerical(format!(
            "heat-balance cascade: the molar distillate rate is {distillate:.4e} mol/s"
        )));
    }
    let reflux = spec.reflux_ratio * distillate;

    let mut liquid_enthalpies = Vec::with_capacity(stages);
    let mut vapour_enthalpies = Vec::with_capacity(stages);
    for j in 0..stages {
        let t = Kelvin(state.temperatures[j]);
        liquid_enthalpies.push(liquid_enthalpy(slate, &state.liquid[j], t));
        let y = equilibrium_vapour(&state.liquid[j], &state.k[j])?;
        vapour_enthalpies
            .push(liquid_enthalpy(slate, &y, t) + mixture_dh_vap(slate, state.thermo, &y, t)?);
    }
    // The reflux: stage 1's vapour, condensed, at its own bubble point.
    let condensate = equilibrium_vapour(&state.liquid[0], &state.k[0])?;
    let condenser_t = bubble_temperature_unnormalized(
        slate,
        state.thermo,
        &condensate,
        state.pressure,
        "the total condenser",
    )?;
    let reflux_enthalpy = liquid_enthalpy(slate, &condensate, condenser_t);

    // A pumparound's heat, per mole: the stage balances are molar flows times
    // molar enthalpies, so its watts enter as they are (J/s against mol/s ·
    // J/mol).
    let side_heat = &plan.side_heat;
    // The top-down sweep from a top vapour `top` and a reflux `top − D`. With
    // the enthalpies of this iterate fixed, every rate it returns is AFFINE in
    // `top`: the liquid from above enters each balance linearly and the
    // material balance carries it down (§65.3).
    let sweep = |top: f64, reflux: f64| -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let mut vapour = vec![0.0; stages];
        let mut liquid_in = vec![0.0; stages];
        let mut liquid_down = vec![0.0; stages];
        vapour[0] = top;
        liquid_in[0] = reflux;
        let mut enthalpy_above = reflux_enthalpy;
        for j in 0..stages - 1 {
            let feed_here = if j == plan.feed_stage {
                feed_molar
            } else {
                0.0
            };
            let here = liquid_enthalpies[j];
            // Energy balance on stage j: the liquid from above warmed to this
            // stage, the feed brought to it, the vapour leaving it made and a
            // pumparound's heat taken out, all paid for by the vapour arriving
            // from below condensing here.
            vapour[j + 1] = (liquid_in[j] * (here - enthalpy_above)
                + feed_here * (here - feed_enthalpy)
                + vapour[j] * (vapour_enthalpies[j] - here)
                + side_heat[j])
                / (vapour_enthalpies[j + 1] - here);
            // Material balance on stage j.
            liquid_down[j] = liquid_in[j] + vapour[j + 1] + feed_here - drawn[j] - vapour[j];
            liquid_in[j + 1] = liquid_down[j];
            enthalpy_above = here;
        }
        (vapour, liquid_in, liquid_down)
    };

    // At the declared reflux first.
    let declared_top = reflux + distillate;
    let (mut vapour, mut liquid_in, mut liquid_down) = sweep(declared_top, reflux);
    let mut reflux = reflux;
    let mut reflux_ratio = spec.reflux_ratio;

    // **The floor** (§65.3): every stage's vapour below the top held at
    // `VAPOUR_FLOOR_OF_FEED · F`. A stage under it lifts the top vapour, and so
    // the reflux, to the smallest value that holds every stage there — one
    // division per stage on the affine coefficients a second sweep gives.
    let floor = VAPOUR_FLOOR_OF_FEED * feed_molar;
    if vapour[1..].iter().any(|v| v.is_nan() || *v < floor) {
        let (lifted, _, _) = sweep(declared_top + distillate, reflux + distillate);
        let mut top = declared_top;
        for j in 1..stages {
            if vapour[j] >= floor {
                continue;
            }
            // dV_j / dV_1, from two sweeps one distillate apart.
            let slope = (lifted[j] - vapour[j]) / distillate;
            if !slope.is_finite() || slope <= 0.0 {
                return Err(SimError::Numerical(format!(
                    "heat-balance cascade: the vapour rising into stage {} is {:.4e} mol/s, \
                     under the floor of {floor:.4e}, and more reflux does not raise it (its \
                     rate of change with the top vapour is {slope:.4e}). The column has no \
                     reflux at which this stage boils up.",
                    j + 1,
                    vapour[j]
                )));
            }
            top = top.max(declared_top + (floor - vapour[j]) / slope);
        }
        reflux = top - distillate;
        reflux_ratio = reflux / distillate;
        (vapour, liquid_in, liquid_down) = sweep(top, reflux);
    }

    // Relaxation toward the previous iterate (`RELAXATION`): the vapour below
    // the top moves part of the way, and the liquid follows from the material
    // balances — before the checks, because a full step can leave the physical
    // region where the relaxed one does not. At the fixed point the two agree.
    if let Some(previous) = state.previous_vapour.filter(|p| p.len() == stages) {
        for j in 1..stages {
            vapour[j] = previous[j] + RELAXATION * (vapour[j] - previous[j]);
        }
        for j in 0..stages - 1 {
            let feed_here = if j == plan.feed_stage {
                feed_molar
            } else {
                0.0
            };
            liquid_down[j] = liquid_in[j] + vapour[j + 1] + feed_here - drawn[j] - vapour[j];
            liquid_in[j + 1] = liquid_down[j];
        }
    }

    for j in 0..stages - 1 {
        if !vapour[j + 1].is_finite() || vapour[j + 1] <= 0.0 {
            return Err(SimError::Numerical(format!(
                "heat-balance cascade: the vapour rising into stage {} came out as {:.4e} \
                 mol/s after the floor was applied — a sweep the floor's own algebra says \
                 cannot happen.",
                j + 1,
                vapour[j + 1]
            )));
        }
        let down = liquid_down[j];
        if !down.is_finite() || down < 0.0 {
            return Err(SimError::Numerical(format!(
                "heat-balance cascade: the liquid leaving stage {} is {down:.4e} mol/s, which \
                 is negative. The side draws above it take more liquid than the stage \
                 balances make — an infeasible specification: real crude columns make that \
                 liquid with pumparounds.",
                j + 1
            )));
        }
    }

    let reboiler = stages - 1;
    let feed_here = if reboiler == plan.feed_stage {
        feed_molar
    } else {
        0.0
    };
    let bottoms = liquid_in[reboiler] + feed_here - vapour[reboiler];
    if !bottoms.is_finite() || bottoms <= 0.0 {
        let taken: f64 = drawn.iter().sum::<f64>() + distillate;
        return Err(SimError::Numerical(format!(
            "heat-balance cascade: the draws above the reboiler carry {taken:.4e} mol/s away \
             from a feed of {feed_molar:.4e} mol/s, so the bottoms rate would be \
             {bottoms:.4e} mol/s — an iterate that has left the physical region (the draw \
             ratios are MASS ratios; a converged split cannot violate it)."
        )));
    }
    drawn[reboiler] = bottoms;
    liquid_down[reboiler] = 0.0;

    Ok((
        Flows {
            liquid_in,
            liquid_down,
            liquid_drawn: drawn,
            vapour,
            reflux,
        },
        reflux_ratio,
    ))
}

/// A liquid's molar enthalpy [J/mol] on the engine's datum: `Σ x·M·cp·(T − T_REF)`
/// — `energy::enthalpy_flux`'s, per mole.
pub(crate) fn liquid_enthalpy(slate: &Slate, fractions: &[f64], temperature: Kelvin) -> f64 {
    mixture_molar_cp(slate, fractions) * (temperature.value() - T_REF.value())
}

/// The vapour in equilibrium with a stage's liquid, `y = K·x` normalised.
pub(crate) fn equilibrium_vapour(liquid: &[f64], k: &[f64]) -> Result<Vec<f64>, SimError> {
    let amounts: Vec<f64> = liquid.iter().zip(k).map(|(x, k)| x * k).collect();
    MoleFractions::from_amounts(&amounts)
        .map(|y| y.fractions().to_vec())
        .map_err(|e| {
            SimError::Numerical(format!(
                "heat-balance cascade: a stage's equilibrium vapour has no valid composition \
                 ({e})"
            ))
        })
}

/// Each draw's mean molar mass [kg/mol] under a SHARP split of the feed by
/// boiling order: the draws, lightest first, each take the next-lightest cuts
/// up to its mass ratio. `None` for a draw the split gives no mass (it keeps
/// the iterate's own molar mass).
///
/// A cold start's estimate of what each draw will be, so the first iterations'
/// molar draw rates are near their converged values (`SHARP_SEED_ITERATIONS`).
fn sharp_split_molar_masses(
    plan: &CascadePlan,
    feed: &Composition,
    slate: &Slate,
) -> Vec<Option<f64>> {
    let mut order: Vec<usize> = (0..slate.len()).collect();
    // `total_cmp` on finite boiling points, then the index: deterministic.
    order.sort_by(|a, b| {
        slate
            .get(*a)
            .tb
            .value()
            .total_cmp(&slate.get(*b).tb.value())
            .then(a.cmp(b))
    });
    let fractions = feed.fractions();
    let mut remaining: Vec<f64> = order.iter().map(|c| fractions[*c]).collect();
    plan.mass_ratios
        .iter()
        .map(|ratio| {
            let mut need = *ratio;
            let (mut mass, mut moles) = (0.0, 0.0);
            for (slot, c) in order.iter().enumerate() {
                if need <= 0.0 {
                    break;
                }
                let take = remaining[slot].min(need);
                remaining[slot] -= take;
                need -= take;
                mass += take;
                moles += take / slate.get(*c).molar_mass.value();
            }
            (moles > 0.0).then(|| mass / moles)
        })
        .collect()
}

/// Bisection halvings of the flash temperature's bracket. 64 halvings of a
/// bracket under 10³ K reach below an `f64`'s spacing there, so the loop is a
/// fixed cost with no convergence arm — `flash_isothermal`'s argument.
const FLASH_TEMPERATURE_HALVINGS: u32 = 64;

/// The share of a feed's MOLES that is vapour once it has flashed adiabatically
/// at `pressure` [-], in `[0, 1]`.
///
/// The feed arrives as a liquid at `temperature` (the graph carries it so) and
/// boils by cooling itself (M53's lesson, DESIGN §58): the flash temperature
/// `T*` is where the isothermal flash's vapour share equals the share the
/// sensible heat pays for, `c̄p(z)·(T_F − T*)/λ̄(y(T*))`. The share returned is
/// the ENERGY's at that root (M53's rule for a pure component's jump), clamped
/// to `[0, 1]`. Zero at or below the bubble point.
fn adiabatic_flash_share(
    slate: &Slate,
    thermo: &dyn ThermoModel,
    feed: &MoleFractions,
    temperature: Kelvin,
    bubble: Kelvin,
    pressure: Pascal,
) -> Result<f64, SimError> {
    let feed_temperature = temperature.value();
    if feed_temperature <= bubble.value() {
        return Ok(0.0);
    }
    let heat_capacity = mixture_molar_cp(slate, feed.fractions());
    let energy_share = |t: f64| -> Result<(f64, f64), SimError> {
        let flash = flash_isothermal(feed, slate, thermo, Kelvin(t), pressure)?;
        let latent = mixture_dh_vap(slate, thermo, flash.vapour.fractions(), Kelvin(t))?;
        Ok((
            flash.vapour_fraction,
            heat_capacity * (feed_temperature - t) / latent,
        ))
    };
    let (mut low, mut high) = (bubble.value(), feed_temperature);
    for _ in 0..FLASH_TEMPERATURE_HALVINGS {
        let middle = 0.5 * (low + high);
        let (flashed, paid_for) = energy_share(middle)?;
        if flashed < paid_for {
            low = middle;
        } else {
            high = middle;
        }
    }
    let (_, share) = energy_share(0.5 * (low + high))?;
    if !share.is_finite() {
        return Err(SimError::Numerical(format!(
            "heat-balance cascade: the feed's flash share came out as {share}"
        )));
    }
    Ok(share.clamp(0.0, 1.0))
}

/// The heat-balance cascade's own gates (M60.1, docs/DESIGN.md §65.6). The
/// fixtures are `cascade.rs`'s kind: constant-α K-values with a temperature
/// exponent, so a bubble point exists and nothing here leans on Trouton.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ConstantAlphaThermo, StageCascade};
    use refinery_core::components::{Phase, PseudoComponent};
    use refinery_core::graph::{ColumnDraw, NodeId};
    use refinery_core::units::{JPerKgK, KgPerM3, KgPerMol, KgPerSec, P_ATM};

    /// Three cuts — light, middle, heavy — all at one liquid `cp` [J/(kg·K)].
    fn slate_with_cp(cp: f64) -> Slate {
        Slate::new(
            [
                ("light", 350.0, 0.100),
                ("middle", 420.0, 0.150),
                ("heavy", 500.0, 0.250),
            ]
            .iter()
            .map(|(name, tb, molar_mass)| PseudoComponent {
                name: (*name).into(),
                tb: Kelvin(*tb),
                molar_mass: KgPerMol(*molar_mass),
                density: Some(KgPerM3(800.0)),
                cp: JPerKgK(cp),
                cp_shape: None,
                phase: Phase::Liquid,
                dh_vap: None,
                cp_vapour: None,
                free_water: false,
            })
            .collect(),
        )
        .unwrap()
    }

    /// `K_c(T) = k_c·(T/400)^10`; the molar latent heats are the test's.
    fn thermo(slate: &Slate, dh_vap: [f64; 3]) -> ConstantAlphaThermo {
        ConstantAlphaThermo::with_temperature_exponent(
            slate,
            vec![3.0, 1.0, 0.25],
            Kelvin(400.0),
            10.0,
        )
        .unwrap()
        .with_dh_vap(slate, dh_vap.to_vec())
        .unwrap()
    }

    const UNEQUAL_DH_VAP: [f64; 3] = [30_000.0, 36_000.0, 45_000.0];
    const FEED_FLOW: f64 = 10.0;

    fn feed() -> Composition {
        Composition::from_weights(&[0.3, 0.3, 0.4]).unwrap()
    }

    fn spec(reflux_ratio: f64) -> CascadeSpec {
        CascadeSpec {
            stages: 8,
            feed_stage: 4,
            reflux_ratio,
            pumparounds: Vec::new(),
        }
    }

    fn draws() -> Vec<ColumnDraw> {
        vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
            ColumnDraw::by_stage(NodeId(2), 8, None),
        ]
    }

    fn bubble(slate: &Slate, thermo: &dyn ThermoModel) -> Kelvin {
        let moles = MoleFractions::from_mass(&feed(), slate).unwrap();
        bubble_temperature_unnormalized(slate, thermo, moles.fractions(), P_ATM, "test").unwrap()
    }

    fn run(
        model: &dyn SeparationModel,
        slate: &Slate,
        thermo: &dyn ThermoModel,
        temperature: Kelvin,
        reflux_ratio: f64,
        seed: Option<&CascadeProfile>,
    ) -> Result<Separation, SimError> {
        let feed = feed();
        let draws = draws();
        let spec = spec(reflux_ratio);
        model.separate(
            &ColumnPass {
                slate,
                draws: &draws,
                smearing: Kelvin(0.0),
                pressure: P_ATM,
                feed: &feed,
                feed_flow: KgPerSec(FEED_FLOW),
                temperature,
                cascade: Some(&spec),
                seed,
            },
            thermo,
        )
    }

    /// The largest difference between two separations' draw mass fractions.
    fn composition_gap(a: &Separation, b: &Separation) -> f64 {
        a.draws
            .iter()
            .zip(&b.draws)
            .flat_map(|(x, y)| {
                x.composition
                    .fractions()
                    .iter()
                    .zip(y.composition.fractions())
                    .map(|(p, q)| (p - q).abs())
                    .collect::<Vec<_>>()
            })
            .fold(0.0, f64::max)
    }

    /// The reboiler stage's own energy balance [W], rebuilt from the converged
    /// profile the solve published: `Q = V_N·H_N + B·h_N − L_{N−1}·h_{N−1}`.
    fn reboiler_stage_duty(
        slate: &Slate,
        thermo: &dyn ThermoModel,
        separation: &Separation,
        temperature: Kelvin,
        reflux_ratio: f64,
    ) -> f64 {
        let profile = separation.profile.as_ref().unwrap();
        let stage_t: Vec<f64> = profile.temperatures.iter().map(|t| t.value()).collect();
        let k = stage_k_values_profile(slate, thermo, &stage_t, P_ATM).unwrap();
        let spec = spec(reflux_ratio);
        let plan = CascadePlan::new(&spec, &draws(), slate.len()).unwrap();
        let states = plan.draw_states(&profile.liquid, &k, slate).unwrap();
        let moles = MoleFractions::from_mass(&feed(), slate).unwrap();
        let feed_molar = FEED_FLOW / moles.mean_molar_mass(slate).value();
        let (flows, _) = stage_balance_flows(
            &plan,
            &spec,
            &StageState {
                slate,
                thermo,
                draws: &states,
                liquid: &profile.liquid,
                k: &k,
                temperatures: &stage_t,
                pressure: P_ATM,
                previous_vapour: None,
            },
            FEED_FLOW,
            feed_molar,
            liquid_enthalpy(slate, moles.fractions(), temperature),
        )
        .unwrap();
        let n = plan.stages - 1;
        let h = |j: usize| liquid_enthalpy(slate, &profile.liquid[j], Kelvin(stage_t[j]));
        let y = equilibrium_vapour(&profile.liquid[n], &k[n]).unwrap();
        let vapour_enthalpy = liquid_enthalpy(slate, &y, Kelvin(stage_t[n]))
            + mixture_dh_vap(slate, thermo, &y, Kelvin(stage_t[n])).unwrap();
        flows.vapour[n] * vapour_enthalpy + flows.liquid_drawn[n] * h(n)
            - flows.liquid_in[n] * h(n - 1)
    }

    /// **The closure and the reboiler's own stage agree**, at a cold, a
    /// saturated and two hot feeds. `column_duties` closes the column's
    /// EXTERNAL energy balance; the reboiler stage's balance is computed here
    /// from the published profile alone. Under constant molar overflow these
    /// were a third apart on crude (§65.0); here they meet at the fixed point.
    #[test]
    fn the_reboiler_closure_is_its_own_stage_balance() {
        let slate = slate_with_cp(2000.0);
        let thermo = thermo(&slate, UNEQUAL_DH_VAP);
        let bubble = bubble(&slate, &thermo);
        for offset in [-10.0, 0.0, 15.0, 30.0] {
            let temperature = Kelvin(bubble.value() + offset);
            let separation = run(
                &HeatBalanceCascade::new(),
                &slate,
                &thermo,
                temperature,
                2.0,
                None,
            )
            .unwrap();
            let closure = separation.reboiler_duty.unwrap().value();
            let stage = reboiler_stage_duty(&slate, &thermo, &separation, temperature, 2.0);
            assert!(
                (closure - stage).abs() <= 1e-6 * closure.abs(),
                "feed {offset:+} K: closure {closure} W against the stage's {stage} W"
            );
        }
    }

    /// **The reduction**: with every molar latent heat equal and the sensible
    /// heat negligible (cp a millionth of a real one), every stage balance says
    /// `V_{j+1} = V_j` and the heat-balance cascade IS constant molar overflow
    /// — the same draws as `StageCascade` on the same saturated feed. And with
    /// unequal latent heats they must NOT agree, or the gate would pass with
    /// the vapour profile never read.
    #[test]
    fn with_equal_latent_heats_it_is_constant_molar_overflow() {
        let slate = slate_with_cp(1e-3);
        let equal = thermo(&slate, [36_000.0; 3]);
        let saturated = bubble(&slate, &equal);
        let balanced = run(
            &HeatBalanceCascade::new(),
            &slate,
            &equal,
            saturated,
            2.0,
            None,
        )
        .unwrap();
        let overflow = run(&StageCascade::new(), &slate, &equal, saturated, 2.0, None).unwrap();
        let gap = composition_gap(&balanced, &overflow);
        assert!(gap < 1e-5, "draws differ by {gap}");

        let unequal = thermo(&slate, UNEQUAL_DH_VAP);
        let saturated = bubble(&slate, &unequal);
        let balanced = run(
            &HeatBalanceCascade::new(),
            &slate,
            &unequal,
            saturated,
            2.0,
            None,
        )
        .unwrap();
        let overflow = run(&StageCascade::new(), &slate, &unequal, saturated, 2.0, None).unwrap();
        let gap = composition_gap(&balanced, &overflow);
        assert!(
            gap > 1e-3,
            "unequal latent heats moved the draws only {gap}"
        );
    }

    /// A hotter feed takes load off the reboiler, flashes more of itself, and
    /// leaves more of the light cut in the bottoms — monotonically, across a
    /// cold, a saturated and two hot feeds.
    #[test]
    fn a_hotter_feed_unloads_the_reboiler_and_weakens_the_stripping() {
        let slate = slate_with_cp(2000.0);
        let thermo = thermo(&slate, UNEQUAL_DH_VAP);
        let bubble = bubble(&slate, &thermo);
        let mut previous: Option<(f64, f64, f64)> = None;
        for offset in [-10.0, 0.0, 15.0, 30.0] {
            let separation = run(
                &HeatBalanceCascade::new(),
                &slate,
                &thermo,
                Kelvin(bubble.value() + offset),
                2.0,
                None,
            )
            .unwrap();
            let reboiler = separation.reboiler_duty.unwrap().value();
            let share = separation.feed_vapour_fraction.unwrap();
            let light_in_bottoms = separation.draws[1].composition.fractions()[0];
            if offset <= 0.0 {
                assert_eq!(
                    share, 0.0,
                    "a feed at or below its bubble point flashes nothing"
                );
            }
            if let Some((reboiler_before, share_before, light_before)) = previous {
                assert!(
                    reboiler < reboiler_before,
                    "{offset:+} K: {reboiler} W after {reboiler_before} W"
                );
                assert!(share >= share_before);
                assert!(light_in_bottoms > light_before);
            }
            previous = Some((reboiler, share, light_in_bottoms));
        }
    }

    /// **The flash share against a hand calculation**: a single-component feed
    /// boils at one temperature, so the adiabatic flash stops there and the
    /// share is what the superheat pays for, `M·cp·ΔT/λ` —
    /// `0.150 · 2000 · 15 / 36 000 = 0.125` for the middle cut 15 K over.
    #[test]
    fn a_pure_feed_flashes_what_its_superheat_pays_for() {
        let slate = slate_with_cp(2000.0);
        let thermo = thermo(&slate, UNEQUAL_DH_VAP);
        let pure = MoleFractions::from_amounts(&[0.0, 1.0, 0.0]).unwrap();
        let boiling =
            bubble_temperature_unnormalized(&slate, &thermo, pure.fractions(), P_ATM, "pure")
                .unwrap();
        let share = adiabatic_flash_share(
            &slate,
            &thermo,
            &pure,
            Kelvin(boiling.value() + 15.0),
            boiling,
            P_ATM,
        )
        .unwrap();
        assert!((share - 0.125).abs() < 1e-9, "{share}");
    }

    /// A warm start, a cold start and a displaced seed reach the same fixed
    /// point (DESIGN §5 fork 5) — on a hot feed, where the stage balances do
    /// the work.
    #[test]
    fn the_answer_does_not_depend_on_where_the_solve_starts() {
        let slate = slate_with_cp(2000.0);
        let thermo = thermo(&slate, UNEQUAL_DH_VAP);
        let hot = Kelvin(bubble(&slate, &thermo).value() + 20.0);
        let cold = run(&HeatBalanceCascade::new(), &slate, &thermo, hot, 2.0, None).unwrap();
        for offset in [-5.0, 5.0] {
            let displaced = run(
                &HeatBalanceCascade::with_seed_offset(Kelvin(offset)),
                &slate,
                &thermo,
                hot,
                2.0,
                None,
            )
            .unwrap();
            assert!(composition_gap(&cold, &displaced) < 1e-5);
        }
        let warm = run(
            &HeatBalanceCascade::with_seed_offset(Kelvin(3.0)),
            &slate,
            &thermo,
            hot,
            2.0,
            cold.profile.as_ref(),
        )
        .unwrap();
        assert!(composition_gap(&cold, &warm) < 1e-5);
    }

    /// **An idle column is idle, not broken** — `StageCascade`'s rule (M7.1
    /// correction 4), held on the inputs an idle column can really be handed:
    /// a placeholder feed of the heaviest cut alone at a non-finite
    /// temperature. The declared splits, zero duties, no flash share.
    #[test]
    fn an_idle_column_is_answered_whatever_its_feed_holds() {
        let slate = slate_with_cp(2000.0);
        let thermo = thermo(&slate, UNEQUAL_DH_VAP);
        let draws = draws();
        let spec = spec(2.0);
        for temperature in [Kelvin(f64::NAN), Kelvin(300.0), Kelvin(900.0)] {
            let idle = HeatBalanceCascade::new()
                .separate(
                    &ColumnPass {
                        slate: &slate,
                        draws: &draws,
                        smearing: Kelvin(0.0),
                        pressure: P_ATM,
                        feed: &Composition::pure(3, 2),
                        feed_flow: KgPerSec(0.0),
                        temperature,
                        cascade: Some(&spec),
                        seed: None,
                    },
                    &thermo,
                )
                .unwrap_or_else(|e| panic!("{temperature:?}: an idle column refused: {e}"));
            assert_eq!(idle.draws[0].split, 0.3);
            assert_eq!(idle.reboiler_duty, Some(Watt::ZERO));
            assert_eq!(idle.feed_vapour_fraction, None);
        }
    }

    /// The non-convergence `Err` is reached, naming what is unmet: the worst of
    /// Newton's equations, and the settings it could not settle at.
    #[test]
    fn a_solve_out_of_iterations_is_refused() {
        let slate = slate_with_cp(2000.0);
        let thermo = thermo(&slate, UNEQUAL_DH_VAP);
        let hot = Kelvin(bubble(&slate, &thermo).value() + 20.0);
        let message = run(
            &HeatBalanceCascade::with_max_iterations(2),
            &slate,
            &thermo,
            hot,
            2.0,
            None,
        )
        .unwrap_err()
        .to_string();
        assert!(
            message.contains("did not converge in 2 iterations"),
            "{message}"
        );
        assert!(message.contains("the worst equation is"), "{message}");
        assert!(message.contains("reflux ratio 2"), "{message}");
    }

    /// **Past the limit the reflux rises** (§65.3, the user's decision 2): a
    /// feed far too hot for its declared reflux would ask for vapour flowing
    /// DOWN below the feed; held at the floor instead, the column runs at a
    /// higher reflux ratio and reports it — and every stage's vapour stands at
    /// or above the floor. Below the limit the declared ratio is reported
    /// exactly: the floor does not bind in normal operation.
    #[test]
    fn past_its_limit_the_column_raises_its_reflux() {
        let slate = slate_with_cp(2000.0);
        let thermo = thermo(&slate, UNEQUAL_DH_VAP);
        let bubble = bubble(&slate, &thermo);
        let normal = run(
            &HeatBalanceCascade::new(),
            &slate,
            &thermo,
            Kelvin(bubble.value() + 15.0),
            0.5,
            None,
        )
        .unwrap();
        assert_eq!(normal.reflux_ratio, Some(0.5));

        let scorching = Kelvin(bubble.value() + 400.0);
        let hot = run(
            &HeatBalanceCascade::new(),
            &slate,
            &thermo,
            scorching,
            0.5,
            None,
        )
        .unwrap_or_else(|e| panic!("a feed past the limit must run: {e}"));
        let ran = hot.reflux_ratio.unwrap();
        assert!(ran > 0.5, "the reflux should have risen, ran at {ran}");

        // Every stage's vapour, rebuilt from the published profile, at or above
        // the floor — and one of them ON it, or the floor did not bind.
        let profile = hot.profile.as_ref().unwrap();
        let stage_t: Vec<f64> = profile.temperatures.iter().map(|t| t.value()).collect();
        let k = stage_k_values_profile(&slate, &thermo, &stage_t, P_ATM).unwrap();
        let spec = spec(0.5);
        let plan = CascadePlan::new(&spec, &draws(), slate.len()).unwrap();
        let states = plan.draw_states(&profile.liquid, &k, &slate).unwrap();
        let moles = MoleFractions::from_mass(&feed(), &slate).unwrap();
        let feed_molar = FEED_FLOW / moles.mean_molar_mass(&slate).value();
        let (flows, rebuilt) = stage_balance_flows(
            &plan,
            &spec,
            &StageState {
                slate: &slate,
                thermo: &thermo,
                draws: &states,
                liquid: &profile.liquid,
                k: &k,
                temperatures: &stage_t,
                pressure: P_ATM,
                previous_vapour: None,
            },
            FEED_FLOW,
            feed_molar,
            liquid_enthalpy(&slate, moles.fractions(), scorching),
        )
        .unwrap();
        assert!(
            (rebuilt - ran).abs() < 1e-6 * ran,
            "{rebuilt} against {ran}"
        );
        let floor = VAPOUR_FLOOR_OF_FEED * feed_molar;
        let lowest = flows.vapour[1..]
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        assert!(
            (lowest - floor).abs() < 1e-6 * floor,
            "the lowest stage vapour {lowest} should sit on the floor {floor}"
        );
    }
}

/// The gate the M60.1 mutation pass found missing: a heavy-residue cold start.
#[cfg(test)]
mod start_tests {
    use super::*;
    use crate::ConstantAlphaThermo;
    use refinery_core::components::{Phase, PseudoComponent};
    use refinery_core::graph::{ColumnDraw, NodeId};
    use refinery_core::units::{JPerKgK, KgPerM3, KgPerMol, KgPerSec, P_ATM};

    /// Four cuts down to a residue of 0.55 kg/mol — §65.0's shape: light draws
    /// with many moles per kilogram, a bottoms with few.
    fn heavy_slate() -> Slate {
        Slate::new(
            [
                ("naphtha", 360.0, 0.100),
                ("kerosene", 470.0, 0.170),
                ("gas_oil", 600.0, 0.300),
                ("residue", 800.0, 0.550),
            ]
            .iter()
            .map(|(name, tb, molar_mass)| PseudoComponent {
                name: (*name).into(),
                tb: Kelvin(*tb),
                molar_mass: KgPerMol(*molar_mass),
                density: Some(KgPerM3(850.0)),
                cp: JPerKgK(2000.0),
                cp_shape: None,
                phase: Phase::Liquid,
                dh_vap: None,
                cp_vapour: None,
                free_water: false,
            })
            .collect(),
        )
        .unwrap()
    }

    fn heavy_thermo(slate: &Slate) -> ConstantAlphaThermo {
        ConstantAlphaThermo::with_temperature_exponent(
            slate,
            vec![4.0, 1.0, 0.15, 0.01],
            Kelvin(450.0),
            10.0,
        )
        .unwrap()
        .with_dh_vap(slate, vec![30_000.0, 40_000.0, 55_000.0, 80_000.0])
        .unwrap()
    }

    fn solve(
        model: &HeatBalanceCascade,
        offset: f64,
        seed: Option<&CascadeProfile>,
    ) -> Result<Separation, SimError> {
        let slate = heavy_slate();
        let thermo = heavy_thermo(&slate);
        let feed = Composition::from_weights(&[0.25, 0.25, 0.2, 0.3]).unwrap();
        let moles = MoleFractions::from_mass(&feed, &slate).unwrap();
        let bubble =
            bubble_temperature_unnormalized(&slate, &thermo, moles.fractions(), P_ATM, "feed")
                .unwrap();
        let draws = vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.25)),
            ColumnDraw::by_stage(NodeId(2), 3, Some(0.25)),
            ColumnDraw::by_stage(NodeId(3), 8, None),
        ];
        let spec = CascadeSpec {
            stages: 8,
            feed_stage: 6,
            reflux_ratio: 2.0,
            pumparounds: Vec::new(),
        };
        model.separate(
            &ColumnPass {
                slate: &slate,
                draws: &draws,
                smearing: Kelvin(0.0),
                pressure: P_ATM,
                feed: &feed,
                feed_flow: KgPerSec(10.0),
                temperature: Kelvin(bubble.value() + offset),
                cascade: Some(&spec),
                seed,
            },
            &thermo,
        )
    }

    /// **A heavy-residue crude starts cold** — §65.0's finding: on a slate
    /// whose residue is 0.55 kg/mol, a first iterate at the feed's composition
    /// gives the light draws more moles than the feed has. The sharp
    /// boiling-order seed is what starts it (the shipped `StageCascade` fails
    /// it on tick 1), at a saturated and two hot feeds.
    #[test]
    fn a_heavy_residue_crude_starts_cold() {
        for offset in [0.0, 20.0, 40.0] {
            let separation = solve(&HeatBalanceCascade::new(), offset, None)
                .unwrap_or_else(|e| panic!("feed {offset:+} K: {e}"));
            let share = separation.feed_vapour_fraction.unwrap();
            assert!(offset == 0.0 || share > 0.1, "{offset}: {share}");
        }
    }
}

/// Pumparounds (M60.3, docs/DESIGN.md §65.4): heat taken off a stage.
#[cfg(test)]
mod pumparound_tests {
    use super::*;
    use crate::{ConstantAlphaThermo, StageCascade};
    use refinery_core::components::{Phase, PseudoComponent};
    use refinery_core::graph::{ColumnDraw, NodeId, Pumparound};
    use refinery_core::units::{JPerKgK, KgPerM3, KgPerMol, KgPerSec, P_ATM};

    fn slate() -> Slate {
        Slate::new(
            [
                ("light", 350.0, 0.100),
                ("middle", 420.0, 0.150),
                ("heavy", 500.0, 0.250),
            ]
            .iter()
            .map(|(name, tb, molar_mass)| PseudoComponent {
                name: (*name).into(),
                tb: Kelvin(*tb),
                molar_mass: KgPerMol(*molar_mass),
                density: Some(KgPerM3(800.0)),
                cp: JPerKgK(2000.0),
                cp_shape: None,
                phase: Phase::Liquid,
                dh_vap: None,
                cp_vapour: None,
                free_water: false,
            })
            .collect(),
        )
        .unwrap()
    }

    fn thermo(slate: &Slate) -> ConstantAlphaThermo {
        ConstantAlphaThermo::with_temperature_exponent(
            slate,
            vec![3.0, 1.0, 0.25],
            Kelvin(400.0),
            10.0,
        )
        .unwrap()
        .with_dh_vap(slate, vec![30_000.0, 36_000.0, 45_000.0])
        .unwrap()
    }

    fn spec(pumparounds: Vec<Pumparound>) -> CascadeSpec {
        CascadeSpec {
            stages: 8,
            feed_stage: 4,
            reflux_ratio: 2.0,
            pumparounds,
        }
    }

    fn draws() -> Vec<ColumnDraw> {
        vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
            ColumnDraw::by_stage(NodeId(2), 8, None),
        ]
    }

    fn separate(model: &dyn SeparationModel, spec: &CascadeSpec) -> Result<Separation, SimError> {
        let slate = slate();
        let thermo = thermo(&slate);
        let feed = Composition::from_weights(&[0.3, 0.3, 0.4]).unwrap();
        let moles = MoleFractions::from_mass(&feed, &slate).unwrap();
        let hot = Kelvin(
            bubble_temperature_unnormalized(&slate, &thermo, moles.fractions(), P_ATM, "feed")
                .unwrap()
                .value()
                + 15.0,
        );
        let draws = draws();
        model.separate(
            &ColumnPass {
                slate: &slate,
                draws: &draws,
                smearing: Kelvin(0.0),
                pressure: P_ATM,
                feed: &feed,
                feed_flow: KgPerSec(10.0),
                temperature: hot,
                cascade: Some(spec),
                seed: None,
            },
            &thermo,
        )
    }

    /// **A pumparound's heat comes back through the reboiler**: the column's
    /// external balance moves by exactly the heat taken out, so with the
    /// condenser held by the reflux the reboiler carries it — to within what
    /// the moved profile changes in the draws' sensible heat, a few per cent.
    /// And the stage it cools makes more liquid below it.
    #[test]
    fn a_pumparound_is_paid_for_by_the_reboiler() {
        let bare = separate(&HeatBalanceCascade::new(), &spec(vec![])).unwrap();
        let duty = 200_000.0;
        let cooled = separate(
            &HeatBalanceCascade::new(),
            &spec(vec![Pumparound {
                stage: 3,
                duty: Watt(duty),
            }]),
        )
        .unwrap();
        let extra = cooled.reboiler_duty.unwrap().value() - bare.reboiler_duty.unwrap().value();
        assert!(
            (extra / duty - 1.0).abs() < 0.1,
            "the reboiler rose {extra} W for a {duty} W pumparound"
        );
        assert_eq!(cooled.reflux_ratio, Some(2.0));
    }

    /// **The closure is the reboiler stage's own balance with a pumparound
    /// in it** — the stage balances, not the closure arithmetic: the closure
    /// counts the pumparound's heat by formula, and only a stage balance that
    /// takes it off its stage agrees with it.
    #[test]
    fn with_a_pumparound_the_closure_is_still_the_reboiler_stage() {
        let slate = slate();
        let thermo = thermo(&slate);
        let spec = spec(vec![Pumparound {
            stage: 3,
            duty: Watt(200_000.0),
        }]);
        let separation = separate(&HeatBalanceCascade::new(), &spec).unwrap();
        let profile = separation.profile.as_ref().unwrap();
        let stage_t: Vec<f64> = profile.temperatures.iter().map(|t| t.value()).collect();
        let k = stage_k_values_profile(&slate, &thermo, &stage_t, P_ATM).unwrap();
        let plan = CascadePlan::new(&spec, &draws(), slate.len()).unwrap();
        let states = plan.draw_states(&profile.liquid, &k, &slate).unwrap();
        let feed = Composition::from_weights(&[0.3, 0.3, 0.4]).unwrap();
        let moles = MoleFractions::from_mass(&feed, &slate).unwrap();
        let feed_molar = 10.0 / moles.mean_molar_mass(&slate).value();
        let hot = Kelvin(
            bubble_temperature_unnormalized(&slate, &thermo, moles.fractions(), P_ATM, "feed")
                .unwrap()
                .value()
                + 15.0,
        );
        let (flows, _) = stage_balance_flows(
            &plan,
            &spec,
            &StageState {
                slate: &slate,
                thermo: &thermo,
                draws: &states,
                liquid: &profile.liquid,
                k: &k,
                temperatures: &stage_t,
                pressure: P_ATM,
                previous_vapour: None,
            },
            10.0,
            feed_molar,
            liquid_enthalpy(&slate, moles.fractions(), hot),
        )
        .unwrap();
        let n = plan.stages - 1;
        let h = |j: usize| liquid_enthalpy(&slate, &profile.liquid[j], Kelvin(stage_t[j]));
        let y = equilibrium_vapour(&profile.liquid[n], &k[n]).unwrap();
        let vapour_enthalpy = liquid_enthalpy(&slate, &y, Kelvin(stage_t[n]))
            + mixture_dh_vap(&slate, &thermo, &y, Kelvin(stage_t[n])).unwrap();
        let stage = flows.vapour[n] * vapour_enthalpy + flows.liquid_drawn[n] * h(n)
            - flows.liquid_in[n] * h(n - 1);
        let closure = separation.reboiler_duty.unwrap().value();
        assert!(
            (closure - stage).abs() <= 1e-6 * closure,
            "closure {closure} W against the stage's {stage} W"
        );
    }

    /// Where a pumparound may not go, and what may not be one — refused by the
    /// plan both models share.
    #[test]
    fn a_misplaced_pumparound_is_refused() {
        let at = |stage: u32, duty: f64| Pumparound {
            stage,
            duty: Watt(duty),
        };
        for (pumparounds, expected) in [
            (vec![at(1, 1e5)], "strictly between"),
            (vec![at(8, 1e5)], "strictly between"),
            (vec![at(3, -1.0)], "finite and >= 0"),
            (vec![at(3, f64::NAN)], "finite and >= 0"),
            (vec![at(3, 1e5), at(3, 2e5)], "more than one pumparound"),
        ] {
            let message = separate(&HeatBalanceCascade::new(), &spec(pumparounds))
                .unwrap_err()
                .to_string();
            assert!(message.contains(expected), "{message}");
        }
    }

    /// Constant molar overflow cannot answer a pumparound and says so.
    #[test]
    fn the_overflow_cascade_refuses_pumparounds() {
        let message = separate(
            &StageCascade::new(),
            &spec(vec![Pumparound {
                stage: 3,
                duty: Watt(1e5),
            }]),
        )
        .unwrap_err()
        .to_string();
        assert!(message.contains("heat_balance"), "{message}");
    }
}

/// Newton against the fixed point (M61, docs/DESIGN.md §66): the same
/// equations, so the same answer wherever both converge — and a column only
/// Newton can run.
#[cfg(test)]
mod newton_tests {
    use super::*;
    use crate::{ConstantAlphaThermo, TroutonThermo};
    use refinery_core::components::{Phase, PseudoComponent};
    use refinery_core::graph::{ColumnDraw, NodeId, Pumparound};
    use refinery_core::units::{JPerKgK, KgPerM3, KgPerMol, KgPerSec, P_ATM};

    /// The largest relative gap between two separations: every draw's mass
    /// fractions (absolute), both duties and the reflux they ran at.
    fn gap(a: &Separation, b: &Separation) -> f64 {
        let mut worst = 0.0f64;
        for (x, y) in a.draws.iter().zip(&b.draws) {
            for (p, q) in x
                .composition
                .fractions()
                .iter()
                .zip(y.composition.fractions())
            {
                worst = worst.max((p - q).abs());
            }
        }
        let relative = |p: f64, q: f64| (p - q).abs() / p.abs().max(q.abs()).max(1.0);
        worst = worst.max(relative(
            a.condenser_duty.unwrap().value(),
            b.condenser_duty.unwrap().value(),
        ));
        worst = worst.max(relative(
            a.reboiler_duty.unwrap().value(),
            b.reboiler_duty.unwrap().value(),
        ));
        worst.max(relative(a.reflux_ratio.unwrap(), b.reflux_ratio.unwrap()))
    }

    fn three_cut_slate() -> Slate {
        Slate::new(
            [
                ("light", 350.0, 0.100),
                ("middle", 420.0, 0.150),
                ("heavy", 500.0, 0.250),
            ]
            .iter()
            .map(|(name, tb, molar_mass)| PseudoComponent {
                name: (*name).into(),
                tb: Kelvin(*tb),
                molar_mass: KgPerMol(*molar_mass),
                density: Some(KgPerM3(800.0)),
                cp: JPerKgK(2000.0),
                cp_shape: None,
                phase: Phase::Liquid,
                dh_vap: None,
                cp_vapour: None,
                free_water: false,
            })
            .collect(),
        )
        .unwrap()
    }

    /// **Where both converge, Newton lands where the fixed point does** — on
    /// the fixtures M60 gated the fixed point on: a cold, a saturated and two
    /// hot feeds, a pumparound, and a feed so hot the vapour floor binds (the
    /// floor as Newton's active set, §66.3, against M60.2's sweep). Agreement to
    /// a millionth: both stop at the same tolerance from the same root.
    #[test]
    fn newton_lands_where_the_fixed_point_does() {
        let slate = three_cut_slate();
        let thermo = ConstantAlphaThermo::with_temperature_exponent(
            &slate,
            vec![3.0, 1.0, 0.25],
            Kelvin(400.0),
            10.0,
        )
        .unwrap()
        .with_dh_vap(&slate, vec![30_000.0, 36_000.0, 45_000.0])
        .unwrap();
        let feed = Composition::from_weights(&[0.3, 0.3, 0.4]).unwrap();
        let moles = MoleFractions::from_mass(&feed, &slate).unwrap();
        let bubble =
            bubble_temperature_unnormalized(&slate, &thermo, moles.fractions(), P_ATM, "feed")
                .unwrap();
        let draws = vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
            ColumnDraw::by_stage(NodeId(2), 8, None),
        ];
        let cases = [
            ("cold", -10.0, 2.0, None),
            ("saturated", 0.0, 2.0, None),
            ("hot", 15.0, 2.0, None),
            ("hotter", 30.0, 2.0, None),
            ("pumparound", 15.0, 2.0, Some(200_000.0)),
            ("past the floor", 400.0, 0.5, None),
        ];
        for (label, offset, reflux_ratio, pumparound) in cases {
            let spec = CascadeSpec {
                stages: 8,
                feed_stage: 4,
                reflux_ratio,
                pumparounds: pumparound
                    .map(|duty| {
                        vec![Pumparound {
                            stage: 3,
                            duty: Watt(duty),
                        }]
                    })
                    .unwrap_or_default(),
            };
            let pass = ColumnPass {
                slate: &slate,
                draws: &draws,
                smearing: Kelvin(0.0),
                pressure: P_ATM,
                feed: &feed,
                feed_flow: KgPerSec(10.0),
                temperature: Kelvin(bubble.value() + offset),
                cascade: Some(&spec),
                seed: None,
            };
            let newton = HeatBalanceCascade::new()
                .separate(&pass, &thermo)
                .unwrap_or_else(|e| panic!("{label}: Newton: {e}"));
            let fixed = HeatBalanceCascade::fixed_point()
                .separate(&pass, &thermo)
                .unwrap_or_else(|e| panic!("{label}: the fixed point: {e}"));
            let gap = gap(&newton, &fixed);
            assert!(
                gap < 1e-6,
                "{label}: Newton and the fixed point differ by {gap}"
            );
        }
        // And the floor bound on the last case, or it compared nothing.
        let spec = CascadeSpec {
            stages: 8,
            feed_stage: 4,
            reflux_ratio: 0.5,
            pumparounds: Vec::new(),
        };
        let scorching = HeatBalanceCascade::new()
            .separate(
                &ColumnPass {
                    slate: &slate,
                    draws: &draws,
                    smearing: Kelvin(0.0),
                    pressure: P_ATM,
                    feed: &feed,
                    feed_flow: KgPerSec(10.0),
                    temperature: Kelvin(bubble.value() + 400.0),
                    cascade: Some(&spec),
                    seed: None,
                },
                &thermo,
            )
            .unwrap();
        assert!(scorching.reflux_ratio.unwrap() > 0.5);
    }

    /// **A floor that stops binding lets go** (§66.3): a column held at the
    /// floor (a feed 60 K past its bubble point, the reflux lifted to 0.79
    /// above its declared 0.5) whose feed then cools to 40 K past it runs at
    /// exactly its declared reflux again — warm from the floored profile, which
    /// tries the floor first and must refuse the floored answer it finds there,
    /// a reflux of 0.37, below the declared one (the M61.1 mutation pass).
    #[test]
    fn a_floor_that_stops_binding_lets_go() {
        let slate = three_cut_slate();
        let thermo = ConstantAlphaThermo::with_temperature_exponent(
            &slate,
            vec![3.0, 1.0, 0.25],
            Kelvin(400.0),
            10.0,
        )
        .unwrap()
        .with_dh_vap(&slate, vec![30_000.0, 36_000.0, 45_000.0])
        .unwrap();
        let feed = Composition::from_weights(&[0.3, 0.3, 0.4]).unwrap();
        let moles = MoleFractions::from_mass(&feed, &slate).unwrap();
        let bubble =
            bubble_temperature_unnormalized(&slate, &thermo, moles.fractions(), P_ATM, "feed")
                .unwrap();
        let draws = vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
            ColumnDraw::by_stage(NodeId(2), 8, None),
        ];
        let spec = CascadeSpec {
            stages: 8,
            feed_stage: 4,
            reflux_ratio: 0.5,
            pumparounds: Vec::new(),
        };
        let at = |offset: f64, seed: Option<&CascadeProfile>| {
            HeatBalanceCascade::new()
                .separate(
                    &ColumnPass {
                        slate: &slate,
                        draws: &draws,
                        smearing: Kelvin(0.0),
                        pressure: P_ATM,
                        feed: &feed,
                        feed_flow: KgPerSec(10.0),
                        temperature: Kelvin(bubble.value() + offset),
                        cascade: Some(&spec),
                        seed,
                    },
                    &thermo,
                )
                .unwrap()
        };
        let floored = at(60.0, None);
        assert!(floored.reflux_ratio.unwrap() > 0.7);
        let cooled = at(40.0, floored.profile.as_ref());
        assert_eq!(cooled.reflux_ratio, Some(0.5));
    }

    /// The crude unit's column (`scenarios/crude_unit.toml`, §65.9): six cuts,
    /// Trouton K-values, ten stages, three side draws, three pumparounds.
    struct CrudeUnit {
        slate: Slate,
        feed: Composition,
        draws: Vec<ColumnDraw>,
    }

    impl CrudeUnit {
        fn new() -> Self {
            let cuts = [
                ("light_naphtha", 70.0, 0.095, 680.0, 2200.0),
                ("heavy_naphtha", 150.0, 0.13, 750.0, 2100.0),
                ("kerosene", 220.0, 0.17, 800.0, 2050.0),
                ("diesel", 300.0, 0.22, 850.0, 2000.0),
                ("gas_oil", 380.0, 0.30, 900.0, 1950.0),
                ("residue", 560.0, 0.55, 980.0, 1850.0),
            ];
            let slate = Slate::new(
                cuts.iter()
                    .map(|(name, tb_c, molar_mass, density, cp)| PseudoComponent {
                        name: (*name).into(),
                        tb: Kelvin(tb_c + 273.15),
                        molar_mass: KgPerMol(*molar_mass),
                        density: Some(KgPerM3(*density)),
                        cp: JPerKgK(*cp),
                        cp_shape: None,
                        phase: Phase::Liquid,
                        dh_vap: None,
                        cp_vapour: None,
                        free_water: false,
                    })
                    .collect(),
            )
            .unwrap();
            Self {
                slate,
                feed: Composition::from_weights(&[0.08, 0.14, 0.16, 0.20, 0.14, 0.28]).unwrap(),
                draws: vec![
                    ColumnDraw::by_stage(NodeId(1), 0, Some(0.22)),
                    ColumnDraw::by_stage(NodeId(2), 3, Some(0.16)),
                    ColumnDraw::by_stage(NodeId(3), 5, Some(0.20)),
                    ColumnDraw::by_stage(NodeId(4), 7, Some(0.14)),
                    ColumnDraw::by_stage(NodeId(5), 10, None),
                ],
            }
        }

        fn spec(reflux_ratio: f64, pumparound_scale: f64) -> CascadeSpec {
            CascadeSpec {
                stages: 10,
                feed_stage: 8,
                reflux_ratio,
                pumparounds: [(4, 15.0e6), (6, 15.0e6), (7, 10.0e6)]
                    .iter()
                    .map(|(stage, duty)| Pumparound {
                        stage: *stage,
                        duty: Watt(duty * pumparound_scale),
                    })
                    .collect(),
            }
        }

        fn separate(
            &self,
            model: &HeatBalanceCascade,
            spec: &CascadeSpec,
            seed: Option<&CascadeProfile>,
        ) -> Result<Separation, SimError> {
            model.separate(
                &ColumnPass {
                    slate: &self.slate,
                    draws: &self.draws,
                    smearing: Kelvin(0.0),
                    pressure: Pascal(1.8e5),
                    feed: &self.feed,
                    feed_flow: KgPerSec(104.0),
                    temperature: Kelvin(355.0 + 273.15),
                    cascade: Some(spec),
                    seed,
                },
                &TroutonThermo::new(),
            )
        }
    }

    /// **Newton runs the crude unit where the fixed point cannot** (§66.0,
    /// finding 3): from the running column, the pumparounds halved in one step
    /// — M60's measured edge, where the fixed point stops settling — run on
    /// Newton, warm, and agree with nothing the fixed point produced, because
    /// it produced nothing. And at the running column itself, where both
    /// settle, they agree.
    #[test]
    fn newton_runs_the_crude_unit_where_the_fixed_point_cannot() {
        let unit = CrudeUnit::new();
        let running = CrudeUnit::spec(4.0, 1.0);
        let newton = unit
            .separate(&HeatBalanceCascade::new(), &running, None)
            .expect("the crude unit runs on Newton");
        let fixed = unit
            .separate(&HeatBalanceCascade::fixed_point(), &running, None)
            .expect("the crude unit runs on the fixed point");
        let gap = gap(&newton, &fixed);
        assert!(gap < 1e-6, "Newton and the fixed point differ by {gap}");

        let halved = CrudeUnit::spec(4.0, 0.5);
        let seed = newton.profile.as_ref();
        let fixed = unit.separate(&HeatBalanceCascade::fixed_point(), &halved, seed);
        assert!(
            fixed.is_err(),
            "the fixed point settled at half the pumparound duty; this gate needs a case it \
             cannot run"
        );
        let halved_run = unit
            .separate(&HeatBalanceCascade::new(), &halved, seed)
            .unwrap_or_else(|e| panic!("Newton at half the pumparound duty: {e}"));
        // Less heat taken out above the feed: the reboiler carries less.
        assert!(halved_run.reboiler_duty.unwrap().value() < newton.reboiler_duty.unwrap().value());
    }

    /// **A dry tray is refused by name** (§66.2): at a reflux of 0.6 the
    /// kerosene tray (stage 3) runs dry — its draw takes all the liquid the
    /// reflux sends it — and the refusal says so beside the settings.
    #[test]
    fn a_dry_tray_is_refused_by_name() {
        let unit = CrudeUnit::new();
        let running = unit
            .separate(&HeatBalanceCascade::new(), &CrudeUnit::spec(4.0, 1.0), None)
            .unwrap();
        let message = unit
            .separate(
                &HeatBalanceCascade::new(),
                &CrudeUnit::spec(0.6, 1.0),
                running.profile.as_ref(),
            )
            .unwrap_err()
            .to_string();
        assert!(
            message.contains("The trays at stage 3 run dry"),
            "{message}"
        );
        assert!(message.contains("reflux ratio 0.6"), "{message}");
    }
}
