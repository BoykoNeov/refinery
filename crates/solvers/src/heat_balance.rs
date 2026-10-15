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

/// The heat-balance cascade: the complex separation fidelity for a column fed
/// off its bubble point, selected by `[fidelity] separation = "heat_balance"`.
///
/// Specified exactly as [`StageCascade`](crate::StageCascade) is — a molar
/// reflux ratio and one MASS ratio per draw but the bottoms (DESIGN §5 fork 3)
/// — and solved on the same stages. See the module docs for what differs.
pub struct HeatBalanceCascade {
    max_iterations: u32,
    seed_offset: Kelvin,
}

impl HeatBalanceCascade {
    /// Outer iterations before the solve is declared failed — `StageCascade`'s
    /// cap and its argument (the pinch case is the stiff one).
    pub const DEFAULT_MAX_ITERATIONS: u32 = 2000;

    pub fn new() -> Self {
        Self {
            max_iterations: Self::DEFAULT_MAX_ITERATIONS,
            seed_offset: Kelvin::ZERO,
        }
    }

    /// A cascade capped at `max_iterations` outer iterations — for the gate
    /// that reaches the non-convergence `Err` on purpose.
    pub fn with_max_iterations(max_iterations: u32) -> Self {
        Self {
            max_iterations,
            seed_offset: Kelvin::ZERO,
        }
    }

    /// A cascade whose seed temperatures are offset by `seed_offset` — for the
    /// start-insensitivity gate (a warm start changes the iteration count, not
    /// the fixed point; DESIGN §5 fork 5).
    pub fn with_seed_offset(seed_offset: Kelvin) -> Self {
        Self {
            max_iterations: Self::DEFAULT_MAX_ITERATIONS,
            seed_offset,
        }
    }
}

impl Default for HeatBalanceCascade {
    fn default() -> Self {
        Self::new()
    }
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
        let sharp = match warm {
            Some(_) => None,
            None => Some(sharp_split_molar_masses(&plan, pass.feed, slate)),
        };

        let mut iterations = 0u32;
        let mut residual = f64::INFINITY;
        let mut profile_change = f64::INFINITY;
        let mut temperature_change = f64::INFINITY;
        let mut converged = false;
        let mut draws: Vec<DrawState> = Vec::new();
        let mut k = stage_k_values_profile(slate, thermo, &stage_t, pass.pressure)?;

        while iterations < self.max_iterations {
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
            let flows = stage_balance_flows(
                &plan,
                spec,
                &StageState {
                    slate,
                    thermo,
                    draws: &previous,
                    liquid: &liquid,
                    k: &k,
                    temperatures: &stage_t,
                    pressure: pass.pressure,
                },
                feed_flow,
                feed_molar,
                feed_enthalpy,
            )?;

            let next = plan.liquid_profile(&flows, &k, &feed, feed_molar)?;
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

        if !converged {
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
            return Err(SimError::Numerical(format!(
                "heat-balance cascade did not converge in {iterations} iterations: {}. The \
                 bound on the residual is I7's own number over this column's feed rate; a \
                 solve that fails publishes no profile, so this is a refusal, not a stale \
                 number.",
                unmet.join("; ")
            )));
        }

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
                    DrawLocation::Stage(j) => Kelvin(stage_t[j]),
                },
            });
        }

        // The converged flows, from the CONVERGED draw states (`StageCascade`'s
        // reason: the loop's last flows are one iterate stale).
        let flows = stage_balance_flows(
            &plan,
            spec,
            &StageState {
                slate,
                thermo,
                draws: &draws,
                liquid: &liquid,
                k: &k,
                temperatures: &stage_t,
                pressure: pass.pressure,
            },
            feed_flow,
            feed_molar,
            feed_enthalpy,
        )?;
        let (condenser_duty, reboiler_duty) = column_duties(
            slate,
            thermo,
            &flows,
            &draws[0],
            Kelvin(stage_t[0]),
            condenser_t,
            &result,
            pass,
            0.0,
        )?;

        Ok(Separation {
            draws: result,
            profile: Some(CascadeProfile {
                temperatures: stage_t.iter().copied().map(Kelvin).collect(),
                liquid,
            }),
            condenser_duty: Some(condenser_duty),
            reboiler_duty: Some(reboiler_duty),
            feed_vapour_fraction: Some(feed_vapour_fraction),
            reflux_ratio: Some(spec.reflux_ratio),
        })
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
fn stage_balance_flows(
    plan: &CascadePlan,
    spec: &CascadeSpec,
    state: &StageState<'_>,
    feed_mass: f64,
    feed_molar: f64,
    feed_enthalpy: f64,
) -> Result<Flows, SimError> {
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

    let mut vapour = vec![0.0; stages];
    let mut liquid_in = vec![0.0; stages];
    let mut liquid_down = vec![0.0; stages];
    vapour[0] = reflux + distillate;
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
        // stage, the feed brought to it, the vapour leaving it made, all paid
        // for by the vapour arriving from below condensing here.
        let rising = (liquid_in[j] * (here - enthalpy_above)
            + feed_here * (here - feed_enthalpy)
            + vapour[j] * (vapour_enthalpies[j] - here))
            / (vapour_enthalpies[j + 1] - here);
        if !rising.is_finite() || rising <= 0.0 {
            return Err(SimError::Numerical(format!(
                "heat-balance cascade: the vapour rising into stage {} came out as {rising:.4e} \
                 mol/s. The stage above it is given more heat than its vapour and liquid can \
                 carry away at reflux ratio {} — a feed too hot for the declared reflux.",
                j + 1,
                spec.reflux_ratio
            )));
        }
        vapour[j + 1] = rising;
        // Material balance on stage j.
        let down = liquid_in[j] + rising + feed_here - drawn[j] - vapour[j];
        if !down.is_finite() || down < 0.0 {
            return Err(SimError::Numerical(format!(
                "heat-balance cascade: the liquid leaving stage {} is {down:.4e} mol/s, which \
                 is negative. The side draws above it take more liquid than the stage \
                 balances make — an infeasible specification: real crude columns make that \
                 liquid with pumparounds.",
                j + 1
            )));
        }
        liquid_down[j] = down;
        liquid_in[j + 1] = down;
        enthalpy_above = here;
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

    Ok(Flows {
        liquid_in,
        liquid_down,
        liquid_drawn: drawn,
        vapour,
        reflux,
    })
}

/// A liquid's molar enthalpy [J/mol] on the engine's datum: `Σ x·M·cp·(T − T_REF)`
/// — `energy::enthalpy_flux`'s, per mole.
pub(crate) fn liquid_enthalpy(slate: &Slate, fractions: &[f64], temperature: Kelvin) -> f64 {
    mixture_molar_cp(slate, fractions) * (temperature.value() - T_REF.value())
}

/// The vapour in equilibrium with a stage's liquid, `y = K·x` normalised.
fn equilibrium_vapour(liquid: &[f64], k: &[f64]) -> Result<Vec<f64>, SimError> {
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
        let flows = stage_balance_flows(
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

    /// The non-convergence `Err` is reached, naming what is unmet.
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
        assert!(message.contains("liquid profile"), "{message}");
    }

    /// A feed far too hot for its reflux: the stage balances ask for vapour
    /// flowing DOWN into the stage below the feed, and the model says so
    /// rather than solving on a negative flow.
    #[test]
    fn a_feed_too_hot_for_its_reflux_is_refused() {
        let slate = slate_with_cp(2000.0);
        let thermo = thermo(&slate, UNEQUAL_DH_VAP);
        let scorching = Kelvin(bubble(&slate, &thermo).value() + 400.0);
        let message = run(
            &HeatBalanceCascade::new(),
            &slate,
            &thermo,
            scorching,
            0.5,
            None,
        )
        .unwrap_err()
        .to_string();
        assert!(
            message.contains("too hot for the declared reflux"),
            "{message}"
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
