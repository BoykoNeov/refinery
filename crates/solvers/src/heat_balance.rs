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
//! nothing two-phase on the plant graph — on every column but one that
//! declares its trays (M73, docs/DESIGN.md §78.1), whose trays read the
//! vapour's density and whose stages each solve at their own pressure, the
//! drops across the trays above fed back (M73.2, §78.9, `fractionate`).

use refinery_core::components::{Composition, Slate};
use refinery_core::energy::T_REF;
use refinery_core::error::SimError;
use refinery_core::graph::{CascadeSpec, ColumnHoldup, HoldupState, Pumparound};
use refinery_core::traits::{
    CascadeProfile, ColumnPass, CutBackDraw, DeclinedHold, DrawSeparation, HoldupReading,
    LevelLimit, LoopRun, Separation, SeparationModel, ThermoModel,
};
use refinery_core::units::{Kelvin, Kg, KgPerSec, Pascal, Watt};

use crate::bubble::{bubble_temperature, bubble_temperature_unnormalized};
use crate::cascade::{
    bubble_profile, column_duties, max_abs_difference, mixture_dh_vap, mixture_molar_cp,
    split_water_draw, stage_k_values_profile, steam_enthalpy_flux, CascadePlan, DrawLocation,
    DrawState, Flows, COMPONENT_RESIDUAL_KG_PER_S,
};
use crate::column_newton::{ColumnEquations, Solved, Specification, Start, Unsolved};
use crate::column_step::{ColumnStep, StepStart};
use crate::flash::{flash_isothermal, FlashResult};
use crate::line_flash::flash_on_the_line;
use crate::molar::MoleFractions;

/// How far a column that holds liquid lets its trays' pressure drops lag what
/// their loads ask [s] (M74, docs/DESIGN.md §79.4): the time the vapour held
/// between two trays takes to pass — on the tray plant about 1 700 mol of
/// vapour in each 0.76 m of a 7–8 m column at 1.9 bar, against about
/// 1 000 mol/s rising, 1.7 s. **A GAME VALUE** of that size. Without it the
/// stages' pressures answer the vapour within the tick, their boiling points
/// with them, and the heat the liquid stores as they move — over a 0.1 s tick
/// tens of megawatts — swung the vapour tick to tick until the step diverged.
const PRESSURE_LAG_S: f64 = 2.0;

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
///
/// A taller column needs it longer (M72.0's probe, docs/DESIGN.md §77.0
/// finding 8): on 11, 12 and 13 trays the draws above the reboiler carried
/// more moles than the feed once it ended, and held for the whole warm-up
/// ([`COLD_WARM_UP_ITERATIONS`]) every layout started. So a column with
/// pumparound loops whose cold start refuses tries again with it held (§77.5)
/// — a retry, and on loop columns only (the user's decision, §77.12): on every
/// other column it would start some that the column screen's tables measured
/// refusing (the cold crude unit at 62.5 MW, its pumparounds off).
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

/// Newton solves a held column's continuation walks in (M64.2, docs/DESIGN.md
/// §69.3), from its base's closure to its held duty.
///
/// **A convergence aid, measured, not physics**: M64.0's probe walked the
/// steamed crude unit's 41.8 MW to zero in ten and landed on the warm walk's
/// answer (§69.0, finding 5).
const CONTINUATION_STEPS: u32 = 10;

/// The side cooling a held column's base carries in all, its own pumparounds
/// topped up to it, as a share of the feed's latent heat flow [-] (M64.2,
/// docs/DESIGN.md §69.3; the user's decision: derived from the feed, no key).
///
/// **A convergence aid, measured, not physics**: with none, the base of M64.0's
/// stripped crude unit refuses once its feed arrives cooler than settled (its
/// coil started at 405 or 430 °C against 488); with 0.75 to 3 times the feed's
/// latent heat flow, spread over the stages above the feed or on its three
/// pumparounds, every start from a coil at 405–512 °C ran and landed on the
/// same column. M62.0's probe asked for 1.5–2 (§67.0, finding 4). TOPPED UP,
/// not added: the steamed crude unit's own 40 MW with 1.5 times on top
/// refused on a later pass of its first tick (its warm-up's bottoms negative).
const BASE_SIDE_COOLING: f64 = 1.5;

/// How many times a column whose cold start refuses halves its pumparounds to
/// start again (M65.1, docs/DESIGN.md §70.1), each start then walked back to
/// the column's own in [`CONTINUATION_STEPS`].
///
/// **A convergence aid, measured, not physics**: the wet crude unit on a crude
/// 2–20 % water starts cold only in a narrow band of pumparounds, closing as the
/// water rises (0.45–0.95 of its 40 MW at 2 %, 0.55–0.65 at 10 %), while warm it
/// runs from zero to three times them; its 40 MW start from a half, 80 MW from
/// a quarter, at every share (M65.0's probe, §70.0). Four halvings reach a
/// sixteenth.
const PUMPAROUND_HALVINGS: u32 = 4;

/// The liquid a cut-back side draw leaves flowing below its tray, as a share of
/// the feed's molar rate (M67.1, docs/DESIGN.md §72.1): 5 %, the user's
/// choice. A game value standing in for tray hydraulics (C1), which would say
/// when a tray weeps dry. M67.0 measured the least liquid under a draw over the
/// corpus at 6.55 % (the stripped crude unit's gas-oil tray), so at 5 % every
/// shipped plant is what it was to the bit, and at 10 % that one moves.
const CUT_BACK_LIQUID_OF_FEED: f64 = 0.05;

/// The most any tray's pressure drop may move between a column's last two
/// solves and count as settled [Pa] (M73.2, docs/DESIGN.md §78.9).
///
/// **A convergence aid, measured, not physics**: M73.2.0's probe (§78.8
/// findings 4–5) landed within 0.11 Pa a tray and 0.0003 K at the bottom from
/// no drops and from the settled ones at this tolerance; each re-solve cuts
/// the drops' error 10–15 times.
const DROP_TOLERANCE: Pascal = Pascal(0.5);

/// Solves a column with trays may take for its drops to settle before it
/// refuses (M73.2). **A convergence aid**: the probe measured 9 from cold, 3–5
/// on a tick a key moves and one on a steady tick, over the loop plant's
/// whole reflux range (§78.8 finding 5).
const MAX_DROP_SOLVES: u32 = 25;

/// Each stage's pressure [Pa], top first: the column's at the top, each stage
/// below higher by the drop across the tray above it, `P_{j+1} = P_j + ΔP_j`
/// (M73.2, docs/DESIGN.md §78.1). `drops` empty: every stage at the column's
/// one pressure; otherwise one per tray, every stage but the reboiler.
fn stage_pressures(top: Pascal, stages: usize, drops: &[Pascal]) -> Result<Vec<Pascal>, SimError> {
    if drops.is_empty() {
        return Ok(vec![top; stages]);
    }
    if drops.len() + 1 != stages {
        return Err(SimError::Numerical(format!(
            "internal: {} tray pressure drops for a column of {stages} stages",
            drops.len()
        )));
    }
    let mut pressures = Vec::with_capacity(stages);
    let mut here = top.value();
    pressures.push(top);
    for drop in drops {
        here += drop.value();
        pressures.push(Pascal(here));
    }
    Ok(pressures)
}

/// A cut-back draw is released once the share it would run at passes its
/// declared one by this much [-], so a draw that sits ON the minimum is not
/// released and cut back again on the last bit.
const RELEASE_MARGIN: f64 = 1.0e-6;

/// A start's draw is cut back once it would take more than its tray can
/// leave the minimum by this share of the feed's molar rate — a rounding
/// margin, so a draw that a solve left ON the minimum is not cut again.
const RECUT_MARGIN_OF_FEED: f64 = 1.0e-9;

/// A cut-back draw's floor, as a share of the feed's molar rate: a draw cut
/// to nothing runs here, a product that has stopped (decision 6, §72.1) —
/// not zero, which the solve's rates may not reach.
const STOPPED_DRAW_OF_FEED: f64 = 1.0e-6;

/// A share of the feed's molar rate below which a flow counts as gone: 1 %. The
/// line search never steps a flow through zero, so a solve that stopped at an
/// edge stopped with that flow close to it — measured at exactly zero on the
/// crude unit's dry kerosene tray (§66.0). `physical_cause` names a tray this
/// dry; a failure that leaves a draw's tray this dry cuts that draw back.
const DRY_TRAY_OF_FEED: f64 = 0.01;

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
    /// Whether a refused cold start tries again with the boiling-order seed
    /// held (M72.4, docs/DESIGN.md §77.5): off only for the gate that shows
    /// the first start refuses.
    seed_retry: bool,
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
            seed_retry: true,
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
            seed_retry: true,
        }
    }

    /// Newton without the boiling-order seed's retry (M72.4, §77.5): the gate
    /// that the shipped 13-tray column refuses its first cold start.
    #[cfg(test)]
    pub(crate) fn without_seed_retry() -> Self {
        Self {
            seed_retry: false,
            ..Self::new()
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
    /// Water vapour and free water leaving each stage [mol/s], and each stage's
    /// water saturation `y_w/K_w` (M62.5); all three empty on a dry feed.
    water_vapour: Vec<f64>,
    free_water: Vec<f64>,
    saturation: Vec<f64>,
    /// Each draw's share of the feed's (the oil's) mass it ran at: its
    /// declared one, or a cut-back draw's own (M67.1).
    ratios: Vec<f64>,
    /// The side draws it ran cut back, as plan indices (M67.1, §72.1).
    cut_back: Vec<usize>,
}

impl Profile {
    /// The stages holding free water — where a solve from this profile starts
    /// its wet set (§67.17 rule 6). Empty on a dry feed.
    fn wet_stages(&self) -> Vec<usize> {
        (0..self.free_water.len())
            .filter(|j| self.free_water[*j] > 0.0)
            .collect()
    }

    fn from_solved(solved: Solved) -> Self {
        Self {
            temperatures: solved.temperatures,
            liquid: solved.liquid,
            flows: solved.flows,
            reflux_ratio: solved.reflux_ratio,
            water_vapour: solved.water_vapour,
            free_water: solved.free_water,
            saturation: solved.saturation,
            ratios: solved.ratios,
            cut_back: Vec::new(),
        }
    }
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
#[derive(Clone)]
struct ColumnSetup<'a> {
    pass: &'a ColumnPass<'a>,
    thermo: &'a dyn ThermoModel,
    spec: &'a CascadeSpec,
    plan: CascadePlan,
    /// What the cascade separates, as mole fractions: the whole feed, or on a
    /// feed with free water the OIL (§67.17 rule 1).
    feed: MoleFractions,
    /// The same as MASS fractions, and its rate [kg/s].
    feed_composition: Composition,
    feed_mass: f64,
    /// The feed's bubble point at the column's pressure — the cold seed.
    seed: Kelvin,
    feed_molar: f64,
    /// The whole feed's enthalpy flux per mole of `feed` [J/mol] — on a dry
    /// feed its own molar enthalpy.
    feed_enthalpy: f64,
    tolerance: f64,
    /// The feed's free water, `None` on a dry feed.
    water: Option<WaterFeed>,
    /// Whether a held column whose warm start fails may start by M64.2's
    /// continuation. `false` only for the duty a column is asked to move its
    /// hold to (M66.2, docs/DESIGN.md §71.2): the continuation there costs
    /// 160–204 ms a held-back tick and reaches nothing the warm start does not.
    continuation: bool,
    /// The cold warm-up's iterations on the feed's sharp split by boiling
    /// order ([`SHARP_SEED_ITERATIONS`]; the whole warm-up on the retry of a
    /// refused cold start, M72.4, docs/DESIGN.md §77.5).
    sharp_seed: u32,
    /// Each stage's pressure [Pa], top first (M73.2, docs/DESIGN.md §78.9):
    /// the column's one pressure on every stage of a column without trays.
    /// Every stage's K-values and bubble point read their own; the condenser
    /// the column's (`pass.pressure`).
    pressures: Vec<Pascal>,
}

impl SeparationModel for HeatBalanceCascade {
    fn name(&self) -> &'static str {
        "heat_balance"
    }

    /// The column fractionates; a held reboiler it cannot run at is declined
    /// for the duty it ran at last (`hold_or_decline`, M66.2, docs/DESIGN.md
    /// §71.2); a feed too wet to separate that its solve refuses passes
    /// straight through instead (`flooded_pass_through`, M63.4, §68.4).
    fn separate(
        &self,
        pass: &ColumnPass<'_>,
        thermo: &dyn ThermoModel,
    ) -> Result<Separation, SimError> {
        // A column that holds liquid steps from its state, its reboiler's
        // duty given whether held or set by its level loop (M74,
        // docs/DESIGN.md §79.2): no decline, no pass-through.
        let stepping = pass
            .cascade
            .and_then(|spec| spec.holdup.as_ref())
            .is_some_and(|holdup| holdup.state.is_some());
        if stepping {
            return self.fractionate(pass, thermo, true);
        }
        match self.hold_or_decline(pass, thermo) {
            Ok(separation) => Ok(separation),
            Err(refusal) => flooded_pass_through(pass, thermo, refusal),
        }
    }
}

/// A refusal's own words: a numerical failure's reason without its kind, so
/// one refusal can quote another (M65.1's arrangement).
fn message(refusal: SimError) -> String {
    match refusal {
        SimError::Numerical(reason) => reason,
        other => other.to_string(),
    }
}

impl HeatBalanceCascade {
    /// A column whose held reboiler duty differs from the duty it last ran at
    /// (M66.2, docs/DESIGN.md §71.2): Newton warm at the held duty, no
    /// continuation; failing, the column held at its running duty — warm, then
    /// the continuation — published at that duty with the decline named. A
    /// later pass of a tick whose earlier pass declined goes straight to the
    /// running duty. Every other column fractionates as it always has.
    fn hold_or_decline(
        &self,
        pass: &ColumnPass<'_>,
        thermo: &dyn ThermoModel,
    ) -> Result<Separation, SimError> {
        let held = pass.cascade.and_then(|spec| spec.reboiler_duty);
        let (Some(spec), Some(held), Some(running)) = (pass.cascade, held, pass.running_reboiler)
        else {
            return self.fractionate(pass, thermo, true);
        };
        if running.duty == held {
            return self.fractionate(pass, thermo, true);
        }
        // The held duty, unless this tick's earlier pass already declined it.
        let reason = match running.declined.filter(|earlier| earlier.asked == held) {
            Some(earlier) => earlier.reason.clone(),
            None => match self.fractionate(pass, thermo, false) {
                Ok(separation) => return Ok(separation),
                Err(refusal) => message(refusal),
            },
        };
        let kept_spec = CascadeSpec {
            reboiler_duty: Some(running.duty),
            ..spec.clone()
        };
        let kept_pass = ColumnPass {
            cascade: Some(&kept_spec),
            ..*pass
        };
        match self.fractionate(&kept_pass, thermo, true) {
            Ok(mut separation) => {
                separation.reboiler_declined = Some(DeclinedHold {
                    asked: held,
                    reason,
                });
                Ok(separation)
            }
            Err(kept) => Err(SimError::Numerical(format!(
                "{reason} It could not keep the {:.3} MW its reboiler ran at either \
                 (docs/DESIGN.md §71.2): {}",
                running.duty.value() / 1.0e6,
                message(kept)
            ))),
        }
    }

    /// One pass of the heat-balance cascade: every draw's split and the duties,
    /// or a refusal that says why the column could not settle. `continuation`
    /// is `ColumnSetup::continuation`.
    ///
    /// **A column that declares its trays solves each stage at its own
    /// pressure** (M73.2, docs/DESIGN.md §78.9): solved, the drops its trays
    /// feed back read ([`crate::trays::capped_drop`]), solved again at them —
    /// each re-solve warm from the last — until no drop moves by
    /// [`DROP_TOLERANCE`]. The first guess is the drops the profile it starts
    /// from was solved at, or none; a hint, so the answer does not hang on it
    /// (§78.8 finding 4). Every other column solves once, at its one pressure,
    /// as it always has.
    fn fractionate(
        &self,
        pass: &ColumnPass<'_>,
        thermo: &dyn ThermoModel,
        continuation: bool,
    ) -> Result<Separation, SimError> {
        let Some(spec) = pass.cascade.filter(|spec| !spec.trays.is_empty()) else {
            return self.fractionate_at(pass, thermo, continuation, &[]);
        };
        let trays = (spec.stages as usize).saturating_sub(1);
        // The drops the profile this solve starts from was solved at — the
        // same profile `fractionate_at` warm-starts from.
        let start = spec.reboiler_duty.and(pass.earlier_pass).or(pass.seed);
        let mut drops = start
            .map(|profile| profile.tray_drops.clone())
            .filter(|drops| drops.len() == trays)
            .unwrap_or_else(|| vec![Pascal(0.0); trays]);
        // A column that holds liquid, past its first tick: each tray's drop
        // follows what its loads ask behind the vapour held between the trays
        // ([`PRESSURE_LAG_S`], M74, docs/DESIGN.md §79.4), from the drops its
        // last tick left — solved within the tick (implicit), so the stages'
        // pressures answer this tick's vapour.
        let lagged = spec
            .holdup
            .as_ref()
            .and_then(|holdup| holdup.state.as_ref().map(|state| (holdup, state)))
            .filter(|(_, state)| state.tray_drops.len() == trays)
            .map(|(holdup, state)| {
                let dt = holdup.tick.value();
                (state.tray_drops.clone(), dt / (PRESSURE_LAG_S + dt))
            });
        if let Some((previous, _)) = &lagged {
            drops = previous.clone();
        }
        let mut latest: Option<CascadeProfile> = None;
        let mut change = f64::INFINITY;
        for _ in 0..MAX_DROP_SOLVES {
            // Each re-solve warm from the last one's profile — for a held
            // column, which starts from its earlier pass, that too.
            let at = ColumnPass {
                seed: latest.as_ref().or(pass.seed),
                earlier_pass: pass.earlier_pass.and(latest.as_ref()).or(pass.earlier_pass),
                ..*pass
            };
            let mut separation = self.fractionate_at(&at, thermo, continuation, &drops)?;
            // A pass that did not fractionate (an idle column) has no trays
            // to read and nothing to re-solve.
            if separation.trays.len() != trays {
                return Ok(separation);
            }
            let fed: Vec<Pascal> = separation
                .trays
                .iter()
                .map(|tray| tray.fed_back_drop)
                .collect();
            // The drops the next solve takes: what the loads ask, or on a
            // column that holds liquid the lag's step toward it.
            let next: Vec<Pascal> = match &lagged {
                None => fed,
                Some((previous, weight)) => previous
                    .iter()
                    .zip(&fed)
                    .map(|(from, to)| Pascal(from.value() + weight * (to.value() - from.value())))
                    .collect(),
            };
            change = next
                .iter()
                .zip(&drops)
                .map(|(read, used)| (read.value() - used.value()).abs())
                .fold(0.0, f64::max);
            if change < DROP_TOLERANCE.value() {
                if let Some(profile) = separation.profile.as_mut() {
                    profile.tray_drops = next.clone();
                }
                if let Some(state) = separation.holdup.as_mut() {
                    state.tray_drops = next;
                }
                return Ok(separation);
            }
            drops = next;
            latest = separation.profile;
        }
        Err(SimError::Numerical(format!(
            "heat-balance cascade: the trays' pressure drops still moved {change:.3} Pa after \
             {MAX_DROP_SOLVES} solves, each stage at the pressure the last one's trays fed \
             back (docs/DESIGN.md §78.9).{}",
            settings_sentence(spec)
        )))
    }

    /// [`Self::fractionate`] once, each tray's stage below it `drops` higher
    /// than the stage above (one per tray, top first; empty — every stage at
    /// the column's one pressure — on a column without trays).
    fn fractionate_at(
        &self,
        pass: &ColumnPass<'_>,
        thermo: &dyn ThermoModel,
        continuation: bool,
        drops: &[Pascal],
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

        // The condenser's water draw (M62.5) is the water balance's, never the
        // plan's: the plan is the oil's draws, the water draw put back in its
        // place in every answer.
        let (oil_draws, water_draw) = split_water_draw(pass.draws)?;
        let plan = CascadePlan::new(spec, &oil_draws, slate.len())?;
        let pressures = stage_pressures(pass.pressure, plan.stages, drops)?;
        let water_draw = match water_draw {
            Some(at) => Some((
                at,
                slate.free_water().ok_or_else(|| {
                    SimError::Scenario(
                        "heat-balance cascade: this column declares a water draw on a slate \
                         with no free_water member, so there is no water to decant."
                            .into(),
                    )
                })?,
            )),
            None => None,
        };

        // Steam on its own stage (M62.6, docs/DESIGN.md §67.18): only a flowing
        // line is a steam the column takes; a closed one is a column without.
        let steam = steam_water(pass, slate, &plan, water_draw)?;
        // Everything entering the column [kg/s]: every split is a share of it.
        let throughput = pass.throughput().value();

        // Only steam flowing in: it leaves by the water draw, condensed.
        if feed_flow == 0.0 {
            if let Some(steam) = steam {
                return water_only_inflow(pass, thermo, &plan, water_draw, steam, 0.0, spec);
            }
        }

        // An idle column: `StageCascade`'s convention and its argument — the
        // declared splits, inert placeholders for what nothing carries, and
        // zero duties as an ANSWER rather than `None`. Returned BEFORE the feed
        // is read at all (M7.1's correction 4: never refuse what you can
        // answer) — an idle feed's composition and temperature are placeholders
        // a bubble point or a flash may not be able to evaluate. No flash share:
        // a share of nothing is not a number the model produced, so the
        // snapshot's `column_operation` is absent while the column is idle.
        // A column that holds liquid goes on boiling and draining with no
        // feed (M74, docs/DESIGN.md §79.2): it steps from its state.
        if feed_flow == 0.0 {
            if let Some((holdup, state)) = spec
                .holdup
                .as_ref()
                .and_then(|holdup| holdup.state.as_ref().map(|state| (holdup, state)))
            {
                let at_feed = &state.stage_composition[plan.feed_stage];
                let setup = ColumnSetup {
                    pass,
                    thermo,
                    spec,
                    feed: MoleFractions::from_mass(at_feed, slate)?,
                    feed_composition: at_feed.clone(),
                    feed_mass: 0.0,
                    seed: state.stage_temperature[plan.feed_stage],
                    feed_molar: 0.0,
                    feed_enthalpy: 0.0,
                    tolerance: (COMPONENT_RESIDUAL_KG_PER_S / state.settled_bottoms.value())
                        .clamp(1.0e-12, 1.0e-6),
                    water: None,
                    continuation,
                    sharp_seed: SHARP_SEED_ITERATIONS,
                    pressures,
                    plan,
                };
                return stepped_pass(&setup, holdup, state, 0.0);
            }
        }
        if feed_flow == 0.0 {
            let mut draws: Vec<DrawSeparation> = plan
                .mass_ratios
                .iter()
                .map(|split| DrawSeparation {
                    split: *split,
                    composition: pass.feed.clone(),
                    temperature: pass.temperature,
                })
                .collect();
            if let Some((at, water)) = water_draw {
                draws.insert(
                    at,
                    DrawSeparation {
                        split: 0.0,
                        composition: pure(slate, water)?,
                        temperature: pass.temperature,
                    },
                );
            }
            return Ok(Separation {
                profile: None,
                draws,
                condenser_duty: Some(Watt::ZERO),
                reboiler_duty: Some(Watt::ZERO),
                feed_vapour_fraction: None,
                reflux_ratio: Some(spec.reflux_ratio),
                reboiler_declined: None,
                cut_back: Vec::new(),
                water_flooded: None,
                loops: Vec::new(),
                stage_liquid: Vec::new(),
                trays: Vec::new(),
                holdup: None,
                holdup_reading: None,
                draw_flows: Vec::new(),
            });
        }

        // Free water in the feed (§67.17 rule 1): the cascade separates the
        // OIL, the water a balance of its own. A feed with none takes M61's
        // arithmetic bit for bit, on a wet slate or a dry one.
        let whole = MoleFractions::from_mass(pass.feed, slate)?;
        let standing = whole.standing_water(slate);
        let (feed_composition, feed_mass, water) = match (standing, steam) {
            (None, None) => (pass.feed.clone(), feed_flow, None),
            // A dry feed under steam: the water is the steam's alone.
            (None, Some(steam)) => {
                let index = slate.free_water().ok_or_else(|| {
                    SimError::Scenario(
                        "heat-balance cascade: steam on a slate with no free_water member".into(),
                    )
                })?;
                (
                    pass.feed.clone(),
                    feed_flow,
                    Some(WaterFeed::new(slate, index, 0.0, Some(steam))),
                )
            }
            (Some(index), steam) => {
                if water_draw.is_none() {
                    return Err(SimError::Scenario(format!(
                        "heat-balance cascade: the feed carries free water ('{}', {:.3e} of \
                         its mass) and this column declares no water draw to decant it at \
                         the condenser — {{ outlet = \"...\", stage = 0, phase = \"water\" }} \
                         (docs/DESIGN.md §67.17).",
                        slate.get(index).name,
                        pass.feed.fractions()[index]
                    )));
                }
                let oil: Vec<f64> = pass
                    .feed
                    .fractions()
                    .iter()
                    .enumerate()
                    .map(|(c, x)| if c == index { 0.0 } else { *x })
                    .collect();
                let oil_share: f64 = oil.iter().sum();
                let water_molar =
                    feed_flow * pass.feed.fractions()[index] / slate.get(index).molar_mass.value();
                if oil_share <= 0.0 {
                    return match steam {
                        None => oil_free_feed(pass, thermo, &plan, water_draw, spec.reflux_ratio),
                        Some(steam) => water_only_inflow(
                            pass,
                            thermo,
                            &plan,
                            water_draw,
                            steam,
                            water_molar,
                            spec,
                        ),
                    };
                }
                (
                    Composition::from_weights(&oil)?,
                    feed_flow * oil_share,
                    Some(WaterFeed::new(slate, index, water_molar, steam)),
                )
            }
        };
        let feed = MoleFractions::from_mass(&feed_composition, slate)?;
        // The feed flashes at its stage's pressure (M73.2, docs/DESIGN.md
        // §78.1) — the column's one pressure on a column without trays.
        let feed_pressure = pressures[plan.feed_stage];
        let seed = bubble_temperature_unnormalized(
            slate,
            thermo,
            feed.fractions(),
            feed_pressure,
            "the feed",
        )?;
        let feed_vapour_fraction = match &water {
            None => {
                adiabatic_flash_share(slate, thermo, &feed, pass.temperature, seed, feed_pressure)?
            }
            // The whole feed's share, oil and water, by the free-water flash
            // from its own wet bubble point (§67.17 rule 11).
            Some(_) => {
                let bubble = bubble_temperature(slate, thermo, &whole, feed_pressure, "the feed")?;
                adiabatic_flash_share(
                    slate,
                    thermo,
                    &whole,
                    pass.temperature,
                    bubble,
                    feed_pressure,
                )?
            }
        };

        let feed_molar = feed_mass / feed.mean_molar_mass(slate).value();
        // `StageCascade`'s tolerance, derived from I7 over this feed (fork 4).
        let tolerance = (COMPONENT_RESIDUAL_KG_PER_S / feed_flow).clamp(1.0e-12, 1.0e-6);
        // The feed's molar enthalpy on the engine's liquid datum — the number
        // `energy::enthalpy_flux` carries into the column, so the stage balance
        // and the plant's books read the same feed. On a wet feed, the WHOLE
        // feed's flux per mole of oil: the oil's moles carry the material
        // balances, its water's latent heat is paid on the feed stage.
        let feed_enthalpy = match &water {
            None => liquid_enthalpy(slate, feed.fractions(), pass.temperature),
            Some(_) => {
                let whole_molar = feed_flow / whole.mean_molar_mass(slate).value();
                liquid_enthalpy(slate, whole.fractions(), pass.temperature) * whole_molar
                    / feed_molar
            }
        };

        // The warm start (M9.3b): both profiles or neither, a wrong-shaped seed
        // ignored — `StageCascade`'s rules. A held column starts from this
        // tick's earlier pass where there is one, ahead of the last tick's
        // (M64.2, M64.3, docs/DESIGN.md §69.3, §69.9): its start from nothing
        // costs ten solves, and a last tick it could not follow would send
        // every pass of the tick to them.
        let earlier = spec.reboiler_duty.and(pass.earlier_pass);
        let warm = earlier.or(pass.seed).filter(|previous| {
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
            feed_composition,
            feed_mass,
            seed,
            feed_molar,
            feed_enthalpy,
            tolerance,
            water,
            continuation,
            sharp_seed: SHARP_SEED_ITERATIONS,
            pressures,
        };

        // A column that holds liquid, past its first tick: one step from the
        // state its last tick left (M74, docs/DESIGN.md §79.2).
        if let Some((holdup, state)) = spec
            .holdup
            .as_ref()
            .and_then(|holdup| holdup.state.as_ref().map(|state| (holdup, state)))
        {
            return stepped_pass(&setup, holdup, state, feed_vapour_fraction);
        }

        let profile = match self.method {
            Method::FixedPoint => self.solve_by_fixed_point(&setup, warm)?,
            Method::Newton => self.solve_by_newton(&setup, warm)?,
        };
        let plan = &setup.plan;

        let mut k = stage_k_values_profile(slate, thermo, &profile.temperatures, &setup.pressures)?;
        if setup.water.is_some() {
            k = effective_k(k, &profile.flows.vapour, &profile.water_vapour);
        }
        let draws = plan.draw_states(&profile.liquid, &k, slate)?;
        let condenser_t = match &setup.water {
            None => bubble_temperature_unnormalized(
                slate,
                thermo,
                draws[0].moles.fractions(),
                pass.pressure,
                "the total condenser",
            )?,
            Some(water) => condenser_temperature(
                slate,
                thermo,
                draws[0].moles.fractions(),
                profile.flows.vapour[0],
                water,
                profile.water_vapour[0],
                pass.pressure,
            )?,
        };
        let last = draws.len() - 1;
        let mut result = Vec::with_capacity(pass.draws.len());
        for (i, draw) in draws.iter().enumerate() {
            let temperature = match plan.locations[i] {
                DrawLocation::Condenser => condenser_t,
                DrawLocation::Stage(j) => Kelvin(profile.temperatures[j]),
            };
            result.push(match &setup.water {
                None => DrawSeparation {
                    split: profile.ratios[i],
                    composition: draw.moles.to_mass(slate)?,
                    temperature,
                },
                // A share of the OIL fed, as a split of the whole feed; the
                // bottoms carries whatever free water reaches the reboiler
                // (§67.17 rule 8).
                Some(water) => {
                    let oil_mass = profile.ratios[i] * setup.feed_mass;
                    let water_mass = if i == last {
                        profile.free_water[plan.stages - 1] * water.molar_mass
                    } else {
                        0.0
                    };
                    let mut masses: Vec<f64> = draw
                        .moles
                        .to_mass(slate)?
                        .fractions()
                        .iter()
                        .map(|x| x * oil_mass)
                        .collect();
                    masses[water.index] += water_mass;
                    DrawSeparation {
                        split: (oil_mass + water_mass) / throughput,
                        composition: Composition::from_weights(&masses)?,
                        temperature,
                    }
                }
            });
        }
        // The water decanted at the condenser: its draw, and the heat its
        // condensing and cooling gives up (§67.17 rule 5).
        let mut water_condensed = 0.0;
        if let Some((at, index)) = water_draw {
            let split = match &setup.water {
                None => 0.0,
                Some(water) => {
                    let overhead = profile.water_vapour[0];
                    let top = Kelvin(profile.temperatures[0]);
                    water_condensed = overhead
                        * (water.vapour_enthalpy(slate, thermo, top)?
                            - water.liquid_enthalpy(condenser_t));
                    overhead * water.molar_mass / throughput
                }
            };
            result.insert(
                at,
                DrawSeparation {
                    split,
                    composition: pure(slate, index)?,
                    temperature: condenser_t,
                },
            );
        }
        // Each loop as it ran, and the trays' liquid where a loop loads them
        // (M72.1, §77.1).
        let runs = loop_runs(plan, spec, slate, &profile)?;
        let stage_liquid = if spec.loops.is_empty() {
            Vec::new()
        } else {
            profile.flows.liquid_down.clone()
        };
        // Each tray's hydraulics at the converged loads, on a column that
        // declares its trays (M73.1, docs/DESIGN.md §78.3).
        let trays = if spec.trays.is_empty() {
            Vec::new()
        } else {
            crate::trays::column_trays(
                slate,
                spec,
                &setup.pressures,
                &profile.temperatures,
                &profile.liquid,
                &k,
                &profile.flows,
                &profile.water_vapour,
                &profile.free_water,
            )?
        };
        let (condenser_duty, reboiler_duty) = column_duties(
            slate,
            thermo,
            &profile.flows,
            &draws[0],
            Kelvin(profile.temperatures[0]),
            condenser_t,
            &result,
            pass,
            plan.side_heat.iter().sum::<f64>()
                + runs.iter().map(|run| run.duty.value()).sum::<f64>(),
            water_condensed,
            spec.reboiler_duty.map(|duty| duty.value()),
        )?;
        // The cut-back draws by the column's own index, the water draw put back.
        let cut_back = profile
            .cut_back
            .iter()
            .map(|i| CutBackDraw {
                draw: match water_draw {
                    Some((at, _)) if at <= *i => i + 1,
                    _ => *i,
                },
                of_declared: profile.ratios[*i] / plan.mass_ratios[*i],
            })
            .collect();
        // A column that holds liquid starts full on its first tick, from these
        // settled loads (M74, docs/DESIGN.md §79.2).
        let holdup = match &spec.holdup {
            Some(declared) if declared.state.is_none() => Some(starting_holdup(
                slate,
                spec,
                declared,
                &setup.pressures,
                &profile,
                &k,
                draws[0].moles.fractions(),
                condenser_t,
                Settled {
                    distillate: KgPerSec(profile.ratios[0] * setup.feed_mass),
                    bottoms: KgPerSec(profile.ratios[profile.ratios.len() - 1] * setup.feed_mass),
                    reboiler: reboiler_duty,
                },
            )?),
            _ => None,
        };

        Ok(Separation {
            draws: result,
            profile: Some(CascadeProfile {
                temperatures: profile.temperatures.iter().copied().map(Kelvin).collect(),
                liquid: profile.liquid,
                vapour: profile.flows.vapour.clone(),
                free_water: profile.free_water,
                reflux_ratio: Some(profile.reflux_ratio),
                cut_back: profile.cut_back,
                // Filled by `fractionate` once the drops settle.
                tray_drops: Vec::new(),
            }),
            condenser_duty: Some(condenser_duty),
            reboiler_duty: Some(reboiler_duty),
            feed_vapour_fraction: Some(feed_vapour_fraction),
            reflux_ratio: Some(profile.reflux_ratio),
            reboiler_declined: None,
            cut_back,
            water_flooded: None,
            loops: runs,
            stage_liquid,
            trays,
            holdup_reading: holdup.as_ref().map(|state| HoldupReading {
                held: state.held,
                drum_level: 0.5,
                base_level: 0.5,
                drum_limit: LevelLimit::Normal,
                base_limit: LevelLimit::Normal,
            }),
            holdup,
            draw_flows: Vec::new(),
        })
    }
}

/// The flows a column that holds liquid settled on its first tick, which its
/// level loops' outputs are multiples of (M74, docs/DESIGN.md §79.2).
struct Settled {
    distillate: KgPerSec,
    bottoms: KgPerSec,
    reboiler: Watt,
}

/// What a column that holds liquid holds on its first tick (M74,
/// docs/DESIGN.md §79.2): each tray what its weir and downcomers hold at its
/// settled loads (`trays::column_liquid`), the base (the reboiler's stage) and
/// the reflux drum half full of their liquid, hot; each at its settled
/// composition and temperature, the trays' vapour loads beside them, and the
/// flows it settled on. `distillate` is the drum's mole fractions, at
/// `condenser`.
#[allow(clippy::too_many_arguments)]
fn starting_holdup(
    slate: &Slate,
    spec: &CascadeSpec,
    declared: &ColumnHoldup,
    pressures: &[Pascal],
    profile: &Profile,
    k: &[Vec<f64>],
    distillate: &[f64],
    condenser: Kelvin,
    settled: Settled,
) -> Result<HoldupState, SimError> {
    let stages = profile.temperatures.len();
    let mut stage_liquid = crate::trays::column_liquid(
        slate,
        spec,
        pressures,
        &profile.temperatures,
        &profile.liquid,
        k,
        &profile.flows,
        &profile.water_vapour,
        &profile.free_water,
    )?;
    let (tray_vapour, tray_vapour_density) = crate::trays::tray_vapour_loads(
        slate,
        spec,
        pressures,
        &profile.temperatures,
        &profile.liquid,
        k,
        &profile.flows,
        &profile.water_vapour,
        &profile.free_water,
    )?
    .into_iter()
    .unzip();
    let base = stages - 1;
    let base_density = crate::trays::hot_liquid_density(
        slate,
        &profile.liquid[base],
        Kelvin(profile.temperatures[base]),
    );
    stage_liquid.push(Kg(0.5
        * declared.base_volume.value()
        * base_density.value()));
    let drum_density = crate::trays::hot_liquid_density(slate, distillate, condenser);
    let stage_composition = profile
        .liquid
        .iter()
        .map(|x| MoleFractions::from_amounts(x)?.to_mass(slate))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(HoldupState {
        stage_liquid,
        stage_composition,
        drum_liquid: Kg(0.5 * declared.drum_volume.value() * drum_density.value()),
        drum_composition: MoleFractions::from_amounts(distillate)?.to_mass(slate)?,
        stage_temperature: profile.temperatures.iter().copied().map(Kelvin).collect(),
        drum_temperature: condenser,
        tray_vapour,
        tray_vapour_density,
        // Filled by `fractionate` once the drops settle.
        tray_drops: Vec::new(),
        settled_distillate: settled.distillate,
        settled_bottoms: settled.bottoms,
        settled_reboiler: settled.reboiler,
        drum_integral: 0.0,
        base_integral: 0.0,
        held: declared.start_held,
        last_bottoms: settled.bottoms,
        last_reboiler: settled.reboiler,
    })
}

/// One tick of a column that holds liquid (M74, docs/DESIGN.md §79.2): the
/// step from the state its last tick left (`column_step`), published as a
/// pass — every draw's flow, its trays' readings and loops, and the state
/// the engine writes back.
fn stepped_pass(
    setup: &ColumnSetup<'_>,
    holdup: &ColumnHoldup,
    state: &HoldupState,
    feed_vapour_fraction: f64,
) -> Result<Separation, SimError> {
    let pass = setup.pass;
    let slate = pass.slate;
    let plan = &setup.plan;
    let spec = setup.spec;
    let n = plan.stages;
    // The energy balances' scale: the feed's latent heat flow, or with no
    // feed the duty the column settled on.
    let energy_scale = if setup.feed_molar > 0.0 {
        setup.feed_molar * mixture_dh_vap(slate, setup.thermo, setup.feed.fractions(), setup.seed)?
    } else {
        state.settled_reboiler.value()
    };
    let step = ColumnStep {
        plan,
        spec,
        holdup,
        state,
        slate,
        thermo: setup.thermo,
        pressure: pass.pressure,
        stage_pressures: &setup.pressures,
        feed: &setup.feed,
        feed_mass: setup.feed_mass,
        feed_molar: setup.feed_molar,
        feed_enthalpy: setup.feed_enthalpy,
        energy_scale,
        tolerance: setup.tolerance,
    };
    // The last tick's answer is where Newton starts — a hint; failing one,
    // the state's temperatures and the vapour its trays carried.
    let seeded = pass
        .seed
        .filter(|seed| seed.temperatures.len() == n && seed.vapour.len() == n);
    let (temperatures, vapour): (Vec<f64>, Vec<f64>) = match seeded {
        Some(seed) => (
            seed.temperatures.iter().map(|t| t.value()).collect(),
            seed.vapour.clone(),
        ),
        None => {
            let mut vapour = Vec::with_capacity(n);
            let top = state.settled_distillate.value() * (spec.reflux_ratio + 1.0)
                / crate::cascade::molar_mass(
                    slate,
                    MoleFractions::from_mass(&state.drum_composition, slate)?.fractions(),
                );
            vapour.push(top);
            for j in 1..n {
                let below = MoleFractions::from_mass(&state.stage_composition[j], slate)?;
                vapour.push(
                    state.tray_vapour[j - 1].value()
                        / crate::cascade::molar_mass(slate, below.fractions()),
                );
            }
            (
                state.stage_temperature.iter().map(|t| t.value()).collect(),
                vapour,
            )
        }
    };
    let stepped = step.step(&StepStart {
        temperatures: &temperatures,
        vapour: &vapour,
    })?;

    let mut next = stepped.state;
    let (tray_vapour, tray_vapour_density) = crate::trays::tray_vapour_loads(
        slate,
        spec,
        &setup.pressures,
        &stepped.temperatures,
        &stepped.liquid,
        &stepped.k,
        &stepped.flows,
        &[],
        &[],
    )?
    .into_iter()
    .unzip();
    next.tray_vapour = tray_vapour;
    next.tray_vapour_density = tray_vapour_density;

    let throughput = pass.throughput().value();
    let mut draws = Vec::with_capacity(plan.locations.len());
    let mut draw_flows = Vec::with_capacity(plan.locations.len());
    let mut ratios = Vec::with_capacity(plan.locations.len());
    for (i, location) in plan.locations.iter().enumerate() {
        let masses = &stepped.draw_masses[i];
        let mass: f64 = masses.iter().sum();
        draw_flows.push(KgPerSec(mass));
        ratios.push(if setup.feed_mass > 0.0 {
            mass / setup.feed_mass
        } else {
            0.0
        });
        // A draw its level loop has shut carries nothing: its line holds the
        // liquid it would draw.
        let composition = if mass > 0.0 {
            Composition::from_weights(masses)?
        } else {
            let liquid = match location {
                DrawLocation::Condenser => &stepped.drum_liquid,
                DrawLocation::Stage(j) => &stepped.liquid[*j],
            };
            MoleFractions::from_amounts(liquid)?.to_mass(slate)?
        };
        draws.push(DrawSeparation {
            // A share of the feed for whatever reads one; the engine writes
            // `draw_flows`.
            split: if throughput > 0.0 {
                mass / throughput
            } else {
                0.0
            },
            composition,
            temperature: match location {
                DrawLocation::Condenser => stepped.drum_temperature,
                DrawLocation::Stage(j) => Kelvin(stepped.temperatures[*j]),
            },
        });
    }
    let profile = Profile {
        temperatures: stepped.temperatures,
        liquid: stepped.liquid,
        flows: stepped.flows,
        reflux_ratio: spec.reflux_ratio,
        water_vapour: Vec::new(),
        free_water: Vec::new(),
        saturation: Vec::new(),
        ratios,
        cut_back: stepped.cut_back.iter().map(|(i, _)| *i).collect(),
    };
    let runs = loop_runs(plan, spec, slate, &profile)?;
    let stage_liquid = if spec.loops.is_empty() {
        Vec::new()
    } else {
        profile.flows.liquid_down.clone()
    };
    let trays = crate::trays::column_trays(
        slate,
        spec,
        &setup.pressures,
        &profile.temperatures,
        &profile.liquid,
        &stepped.k,
        &profile.flows,
        &[],
        &[],
    )?;
    Ok(Separation {
        draws,
        profile: Some(CascadeProfile {
            temperatures: profile.temperatures.iter().copied().map(Kelvin).collect(),
            liquid: profile.liquid,
            vapour: profile.flows.vapour.clone(),
            free_water: Vec::new(),
            reflux_ratio: Some(spec.reflux_ratio),
            cut_back: Vec::new(),
            // Filled by `fractionate` once the drops settle.
            tray_drops: Vec::new(),
        }),
        condenser_duty: Some(Watt(stepped.condenser_duty)),
        reboiler_duty: Some(Watt(stepped.reboiler_duty)),
        feed_vapour_fraction: Some(feed_vapour_fraction),
        reflux_ratio: Some(spec.reflux_ratio),
        reboiler_declined: None,
        cut_back: stepped
            .cut_back
            .iter()
            .map(|(draw, of_declared)| CutBackDraw {
                draw: *draw,
                of_declared: *of_declared,
            })
            .collect(),
        water_flooded: None,
        loops: runs,
        stage_liquid,
        trays,
        holdup: Some(next),
        holdup_reading: Some(stepped.reading),
        draw_flows,
    })
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
        let feed_flow = setup.feed_mass;
        let tolerance = setup.tolerance;
        // A wet feed's water as the warm-up carries it (§67.17 rule 7): inert
        // vapour on dry stages, the feed's water above its stage and none
        // below, each cut's K at the oil's partial pressure.
        let dry_water = match &setup.water {
            Some(water) => Some(water_vapour_profile(plan, water, &vec![0.0; plan.stages])?),
            None => None,
        };
        let k_at = |temperatures: &[f64], oil_vapour: &[f64]| -> Result<Vec<Vec<f64>>, SimError> {
            let k = stage_k_values_profile(slate, thermo, temperatures, &setup.pressures)?;
            Ok(match &dry_water {
                None => k,
                Some(water_vapour) => effective_k(k, oil_vapour, water_vapour),
            })
        };

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
            None => Some(sharp_split_molar_masses(
                plan,
                &setup.feed_composition,
                slate,
            )),
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
        let start_vapour = last_vapour
            .clone()
            .unwrap_or_else(|| vec![setup.feed_molar; plan.stages]);
        let mut k = k_at(&stage_t, &start_vapour)?;
        let mut draws = plan.draw_states(&liquid, &k, slate)?;

        while iterations < cap {
            iterations += 1;

            let mut previous = plan.draw_states(&liquid, &k, slate)?;
            if let Some(masses) = &sharp {
                if iterations <= setup.sharp_seed {
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
                        water: setup.water.as_ref().zip(dry_water.as_deref()),
                    },
                    feed_flow,
                    setup.feed_molar,
                    setup.feed_enthalpy,
                )?
                .0
            };
            last_vapour = Some(flows.vapour.clone());
            if dry_water.is_some() {
                k = k_at(&stage_t, &flows.vapour)?;
            }

            let next = plan.liquid_profile(&flows, &k, &setup.feed, setup.feed_molar)?;
            profile_change = max_abs_difference(&liquid, &next);
            liquid = next;

            let next_t = match &dry_water {
                None => bubble_profile(slate, thermo, &liquid, &setup.pressures)?,
                Some(water_vapour) => oil_bubble_profile(
                    slate,
                    thermo,
                    &liquid,
                    &flows.vapour,
                    water_vapour,
                    &setup.pressures,
                )?,
            };
            temperature_change = stage_t
                .iter()
                .zip(&next_t)
                .map(|(a, b)| (a - b).abs() / b)
                .fold(0.0f64, f64::max);
            stage_t = next_t;

            k = k_at(&stage_t, &flows.vapour)?;
            draws = plan.draw_states(&liquid, &k, slate)?;
            residual = plan.component_residual(&draws, &setup.feed_composition, slate) * feed_flow;

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
        if setup.spec.reboiler_duty.is_some() {
            return Err(SimError::Scenario(
                "heat-balance cascade (the fixed point): a held reboiler is Newton's alone \
                 (docs/DESIGN.md §69.1)."
                    .into(),
            ));
        }
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
        let dry_water = match &setup.water {
            Some(water) => Some(water_vapour_profile(
                &setup.plan,
                water,
                &vec![0.0; setup.plan.stages],
            )?),
            None => None,
        };
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
                water: setup.water.as_ref().zip(dry_water.as_deref()),
            },
            setup.feed_mass,
            setup.feed_molar,
            setup.feed_enthalpy,
        )?;
        let free_water = if dry_water.is_some() {
            vec![0.0; setup.plan.stages]
        } else {
            Vec::new()
        };
        Ok(Profile {
            temperatures: state.temperatures,
            liquid: state.liquid,
            flows,
            reflux_ratio,
            water_vapour: dry_water.unwrap_or_default(),
            free_water,
            saturation: Vec::new(),
            ratios: setup.plan.mass_ratios.clone(),
            cut_back: Vec::new(),
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
            stage_pressures: &setup.pressures,
            feed: &setup.feed,
            feed_composition: &setup.feed_composition,
            feed_mass: setup.feed_mass,
            feed_molar: setup.feed_molar,
            feed_enthalpy: setup.feed_enthalpy,
            energy_scale: setup.feed_molar * latent,
            tolerance: setup.tolerance,
            water: setup.water.as_ref(),
            cut_back: &[],
            liquid_minimum: CUT_BACK_LIQUID_OF_FEED * setup.feed_molar,
        };

        let mut warm_failure = None;
        if let Some(previous) = warm.filter(|p| p.vapour.len() == plan.stages) {
            // A warm Newton that fails is retried cold, where the aids are.
            match self.warm_newton(setup, &equations, previous)? {
                Ok(profile) => return Ok(profile),
                Err(unsolved) => warm_failure = Some(unsolved),
            }
        }

        // A held column starts by a continuation from the same column with its
        // reboiler (M64.2, docs/DESIGN.md §69.3): M61's cold start below lands
        // a column without one nowhere.
        if let Some(held) = setup.spec.reboiler_duty {
            if let (false, Some(warm)) = (setup.continuation, &warm_failure) {
                return Err(SimError::Numerical(format!(
                    "heat-balance cascade: {}.{}{}",
                    warm.reason,
                    physical_cause(plan, warm.flows.as_deref(), setup.feed_molar),
                    settings_sentence(setup.spec)
                )));
            }
            return self.continuation(setup, &equations, held.value());
        }
        let refused = match self.cold_path(setup, &equations, warm_failure.as_ref()) {
            Ok(profile) => return Ok(profile),
            Err(refused) => refused,
        };
        // A column with pumparound loops whose cold start refuses, halvings
        // and all, tries once more with the feed's boiling-order seed held for
        // its whole warm-up (M72.4, docs/DESIGN.md §77.5, §77.12): a taller
        // column needs it longer. Only a loop column — on any other it would
        // start columns the screen's tables measured refusing — and not where
        // the feed floods, which no seed fractionates (§70.2).
        if !self.seed_retry
            || plan.loops.is_empty()
            || setup.sharp_seed >= COLD_WARM_UP_ITERATIONS
            || flooding(setup.pass).is_some()
        {
            return Err(refused);
        }
        let held = ColumnSetup {
            sharp_seed: COLD_WARM_UP_ITERATIONS,
            ..setup.clone()
        };
        self.cold_path(&held, &equations, warm_failure.as_ref())
            .map_err(|retry| match refused {
                SimError::Numerical(reason) => SimError::Numerical(format!(
                    "{reason} Started again with its feed's boiling-order seed held for its \
                     whole warm-up (docs/DESIGN.md §77.5), it did not start either: {retry}"
                )),
                other => other,
            })
    }

    /// An unheld column's way in from cold: M61's cold start, then — refused —
    /// its pumparounds halved (M65.1, docs/DESIGN.md §70.1).
    fn cold_path(
        &self,
        setup: &ColumnSetup<'_>,
        equations: &ColumnEquations<'_>,
        warm_failure: Option<&Unsolved>,
    ) -> Result<Profile, SimError> {
        let plan = &setup.plan;
        let refused = match self.cold_start(setup, equations, warm_failure) {
            Ok(profile) => return Ok(profile),
            Err(refused) => refused,
        };
        // A cold start that refuses starts again from its pumparounds halved
        // (M65.1, docs/DESIGN.md §70.1) — side coolers' duties and loops'
        // start duties (M72.4, §77.5); a column with none has nothing to halve,
        // and a feed wet enough to flood floods (M65.2, §70.2) — its halvings
        // cost half a second a tick on the flooded crude unit, and a column fed
        // that much water does not fractionate.
        let nothing_to_halve = plan.side_heat.iter().all(|heat| *heat == 0.0)
            && plan.loops.iter().all(|l| l.start_duty == 0.0);
        if nothing_to_halve || flooding(setup.pass).is_some() {
            return Err(refused);
        }
        self.halved_start(setup, equations)
            .map_err(|how| match refused {
                SimError::Numerical(reason) => SimError::Numerical(format!("{reason} {how}")),
                other => other,
            })
    }

    /// Newton from `previous`, a profile of this column's shape (the last
    /// tick's, or this tick's earlier pass): its temperatures offset by the
    /// seed offset, its draws read off its liquid. The outer `Err` is a profile
    /// whose K-values or draws cannot be read; the inner one Newton's failure.
    fn warm_newton(
        &self,
        setup: &ColumnSetup<'_>,
        equations: &ColumnEquations<'_>,
        previous: &CascadeProfile,
    ) -> Result<Result<Profile, Unsolved>, SimError> {
        let pass = setup.pass;
        let slate = pass.slate;
        let plan = &setup.plan;
        let temperatures: Vec<f64> = previous
            .temperatures
            .iter()
            .map(|t| t.value() + self.seed_offset.value())
            .collect();
        let k = stage_k_values_profile(slate, setup.thermo, &temperatures, &setup.pressures)?;
        let draws = plan.draw_states(&previous.liquid, &k, slate)?;
        // The last tick's wet stages start this one's wet set (§67.17
        // rule 6) — a hint: on a dry feed there is none.
        let free_water: &[f64] =
            if setup.water.is_some() && previous.free_water.len() == plan.stages {
                &previous.free_water
            } else {
                &[]
            };
        let wet: Vec<usize> = (0..free_water.len())
            .filter(|j| free_water[*j] > 0.0)
            .collect();
        // The last tick's cut-back draws start this one's set (M67.1) — a hint:
        // an index that is not one of this plan's side draws on a tray is not
        // read.
        let carried: Vec<usize> = previous
            .cut_back
            .iter()
            .copied()
            .filter(|i| {
                *i > 0
                    && *i < plan.mass_ratios.len() - 1
                    && matches!(plan.locations[*i], DrawLocation::Stage(_))
            })
            .collect();
        let equations = &ColumnEquations {
            cut_back: &carried,
            ..*equations
        };
        Ok(self.newton_with_water(
            equations,
            &temperatures,
            &previous.vapour,
            &draw_rates(plan, &draws, setup.feed_mass, slate, &previous.liquid),
            previous.reflux_ratio,
            wet,
            free_water,
        ))
    }

    /// A cold start. A column with pumparound loops starts on their start
    /// duties — each return `h_d − Q_start/P` — and hands them to their held
    /// return temperatures in [`CONTINUATION_STEPS`] Newton solves, each from
    /// the last, the last the column's own equations exactly (M72.4,
    /// docs/DESIGN.md §77.5): from the warm-up's rough temperatures a held
    /// return implies a duty far off, and the stage below the feed loses its
    /// vapour (§77.0, finding 7). Every other column: [`Self::cold_start_on`].
    fn cold_start(
        &self,
        setup: &ColumnSetup<'_>,
        equations: &ColumnEquations<'_>,
        warm_failure: Option<&Unsolved>,
    ) -> Result<Profile, SimError> {
        if setup.plan.loops.is_empty() {
            return self.cold_start_on(setup, equations, warm_failure);
        }
        let handed = |share: f64| {
            let mut plan = setup.plan.clone();
            for l in plan.loops.iter_mut() {
                l.hand_over = share;
            }
            plan
        };
        let start_plan = handed(0.0);
        let start_setup = ColumnSetup {
            plan: start_plan.clone(),
            ..setup.clone()
        };
        let start_equations = ColumnEquations {
            plan: &start_plan,
            ..*equations
        };
        let mut profile = self.cold_start_on(&start_setup, &start_equations, warm_failure)?;
        for step in 1..=CONTINUATION_STEPS {
            let last = step == CONTINUATION_STEPS;
            let step_plan = handed(f64::from(step) / f64::from(CONTINUATION_STEPS));
            let walked = ColumnEquations {
                plan: &step_plan,
                ..*equations
            };
            let step_equations = if last { equations } else { &walked };
            profile = self
                .newton_with_water(
                    step_equations,
                    &profile.temperatures,
                    &profile.flows.vapour,
                    &profile_draw_rates(&setup.plan, &profile),
                    Some(profile.reflux_ratio),
                    profile.wet_stages(),
                    &profile.free_water,
                )
                .map_err(|unsolved| {
                    SimError::Numerical(format!(
                        "heat-balance cascade started on its pumparound loops' start duties \
                         (docs/DESIGN.md §77.5), and the hand-over to their return \
                         temperatures stopped at step {step} of {CONTINUATION_STEPS}: Newton \
                         {}.{}{}",
                        unsolved.reason,
                        physical_cause(&setup.plan, unsolved.flows.as_deref(), setup.feed_molar),
                        settings_sentence(setup.spec)
                    ))
                })?;
        }
        Ok(profile)
    }

    /// M61's cold start (§66.1): a short fixed-point warm-up, then Newton. A
    /// refusal names the cold failure, with the warm one's physical cause where
    /// only the warm attempt's last iterate shows an edge.
    fn cold_start_on(
        &self,
        setup: &ColumnSetup<'_>,
        equations: &ColumnEquations<'_>,
        warm_failure: Option<&Unsolved>,
    ) -> Result<Profile, SimError> {
        let plan = &setup.plan;
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
                    Some(warm) => refusal(&warm.reason, &cause_of(warm)),
                    None => refusal(&format!("could not start: {e}"), ""),
                })
            }
        };
        let vapour = state
            .vapour
            .clone()
            .unwrap_or_else(|| vec![setup.feed_molar; plan.stages]);
        match self.newton_with_water(
            equations,
            &state.temperatures,
            &vapour,
            &draw_rates(
                plan,
                &state.draws,
                setup.feed_mass,
                setup.pass.slate,
                &state.liquid,
            ),
            None,
            Vec::new(),
            &[],
        ) {
            Ok(profile) => Ok(profile),
            // The cold failure is reported, with the warm one's physical cause
            // where only the warm one's last iterate shows an edge.
            Err(cold) => {
                let mut cause = cause_of(&cold);
                if cause.is_empty() {
                    if let Some(warm) = warm_failure {
                        cause = cause_of(warm);
                    }
                }
                Err(refusal(&cold.reason, &cause))
            }
        }
    }

    /// An unheld column whose cold start refused (M65.1, docs/DESIGN.md §70.1):
    /// Newton from this tick's earlier pass, where the engine hands one; else
    /// the same column with every pumparound halved, cold, up to
    /// [`PUMPAROUND_HALVINGS`] times, the first that starts walked back to the
    /// column's own pumparounds ([`Self::walked_back`]). `Err` is a sentence
    /// saying how the last attempt failed, for the cold refusal to carry.
    fn halved_start(
        &self,
        setup: &ColumnSetup<'_>,
        equations: &ColumnEquations<'_>,
    ) -> Result<Profile, String> {
        let plan = &setup.plan;
        // The walk is paid once a tick: a later pass starts where it landed.
        let earlier = setup.pass.earlier_pass.filter(|previous| {
            previous.temperatures.len() == plan.stages
                && previous.vapour.len() == plan.stages
                && previous.liquid.len() == plan.stages
                && previous
                    .liquid
                    .iter()
                    .all(|row| row.len() == setup.pass.slate.len())
        });
        if let Some(earlier) = earlier {
            if let Ok(Ok(profile)) = self.warm_newton(setup, equations, earlier) {
                return Ok(profile);
            }
        }
        let mut failure = String::new();
        for halvings in 1..=PUMPAROUND_HALVINGS {
            match self.walked_back(setup, equations, 0.5f64.powi(halvings as i32)) {
                Ok(profile) => return Ok(profile),
                Err(how) => failure = how,
            }
        }
        Err(format!(
            "Started again from its pumparounds halved, up to {PUMPAROUND_HALVINGS} times \
             (docs/DESIGN.md §70.1), it did not start either: {failure}"
        ))
    }

    /// The column with every pumparound at `share` of its duty [-], started
    /// cold as [`Self::cold_start`] starts any column, then walked back to the
    /// column's own pumparounds in [`CONTINUATION_STEPS`] Newton solves, each
    /// from the last, the last the column's own equations exactly — M64.2's walk
    /// with the reboiler closing the column. `Err` says where it stopped.
    fn walked_back(
        &self,
        setup: &ColumnSetup<'_>,
        equations: &ColumnEquations<'_>,
        share: f64,
    ) -> Result<Profile, String> {
        let plan = &setup.plan;
        let mut base_plan = plan.clone();
        for heat in base_plan.side_heat.iter_mut() {
            *heat *= share;
        }
        // And each loop's start duty (M72.4, §77.5): the cold start hands the
        // loop to its return temperature, so the walk back has none to walk.
        for l in base_plan.loops.iter_mut() {
            l.start_duty *= share;
        }
        let base_spec = CascadeSpec {
            pumparounds: setup
                .spec
                .pumparounds
                .iter()
                .map(|pumparound| Pumparound {
                    stage: pumparound.stage,
                    duty: Watt(pumparound.duty.value() * share),
                })
                .collect(),
            ..setup.spec.clone()
        };
        let base_setup = ColumnSetup {
            pass: setup.pass,
            thermo: setup.thermo,
            spec: &base_spec,
            plan: base_plan.clone(),
            feed: setup.feed.clone(),
            feed_composition: setup.feed_composition.clone(),
            feed_mass: setup.feed_mass,
            seed: setup.seed,
            feed_molar: setup.feed_molar,
            feed_enthalpy: setup.feed_enthalpy,
            tolerance: setup.tolerance,
            water: setup.water.clone(),
            continuation: setup.continuation,
            sharp_seed: setup.sharp_seed,
            pressures: setup.pressures.clone(),
        };
        let base_equations = ColumnEquations {
            plan: &base_plan,
            spec: &base_spec,
            ..*equations
        };
        let mut profile = self
            .cold_start(&base_setup, &base_equations, None)
            .map_err(|refused| match refused {
                SimError::Numerical(reason) => format!("at {share} of them, {reason}"),
                other => format!("at {share} of them, {other}"),
            })?;
        for step in 1..=CONTINUATION_STEPS {
            let last = step == CONTINUATION_STEPS;
            let fraction = f64::from(step) / f64::from(CONTINUATION_STEPS);
            let mut step_plan = base_plan.clone();
            for (heat, target) in step_plan.side_heat.iter_mut().zip(&plan.side_heat) {
                *heat += fraction * (target - *heat);
            }
            let walked = ColumnEquations {
                plan: &step_plan,
                ..*equations
            };
            let step_equations = if last { equations } else { &walked };
            profile = self
                .newton_with_water(
                    step_equations,
                    &profile.temperatures,
                    &profile.flows.vapour,
                    &profile_draw_rates(plan, &profile),
                    Some(profile.reflux_ratio),
                    profile.wet_stages(),
                    &profile.free_water,
                )
                .map_err(|unsolved| {
                    format!(
                        "started at {share} of them, the walk back stopped at step {step} of \
                         {CONTINUATION_STEPS}, the pumparounds at {:.3} MW in all: Newton {}",
                        step_plan.side_heat.iter().sum::<f64>() / 1.0e6,
                        unsolved.reason
                    )
                })?;
        }
        Ok(profile)
    }

    /// A held column's start from nothing (M64.2, docs/DESIGN.md §69.3): the
    /// BASE — the same column with its reboiler closing it, at its declared
    /// reflux and the base pumparounds — solved as M61 solves any column cold;
    /// then [`CONTINUATION_STEPS`] Newton solves, each from the last, walking
    /// the held duty from the base's closure to `held` [W] and the pumparounds
    /// from the base's to the column's own. The last step is the column's own
    /// equations exactly. A base that refuses, or a step that fails, is a
    /// refusal naming where the walk stopped.
    fn continuation(
        &self,
        setup: &ColumnSetup<'_>,
        equations: &ColumnEquations<'_>,
        held: f64,
    ) -> Result<Profile, SimError> {
        let plan = &setup.plan;
        let refusal = |reason: String, flows: Option<&Flows>| {
            SimError::Numerical(format!(
                "heat-balance cascade: a column holding its reboiler at {:.3} MW starts from \
                 the same column with its reboiler (docs/DESIGN.md §69.3), and {reason}.{}{}",
                held / 1.0e6,
                physical_cause(plan, flows, setup.feed_molar),
                settings_sentence(setup.spec)
            ))
        };

        // The base: the declared column, its reboiler closing it.
        let mut base_plan = plan.clone();
        base_plan.side_heat = base_side_heat(plan, equations.energy_scale);
        let base_spec = CascadeSpec {
            reboiler_duty: None,
            pumparounds: setup
                .spec
                .pumparounds
                .iter()
                .map(|pumparound| Pumparound {
                    stage: pumparound.stage,
                    duty: Watt(base_plan.side_heat[pumparound.stage as usize - 1]),
                })
                .collect(),
            ..setup.spec.clone()
        };
        let base_setup = ColumnSetup {
            pass: setup.pass,
            thermo: setup.thermo,
            spec: &base_spec,
            plan: base_plan.clone(),
            feed: setup.feed.clone(),
            feed_composition: setup.feed_composition.clone(),
            feed_mass: setup.feed_mass,
            seed: setup.seed,
            feed_molar: setup.feed_molar,
            feed_enthalpy: setup.feed_enthalpy,
            tolerance: setup.tolerance,
            water: setup.water.clone(),
            continuation: setup.continuation,
            sharp_seed: setup.sharp_seed,
            pressures: setup.pressures.clone(),
        };
        let base_equations = ColumnEquations {
            plan: &base_plan,
            spec: &base_spec,
            ..*equations
        };
        let cap = COLD_WARM_UP_ITERATIONS.min(self.max_iterations);
        let state = self
            .fixed_point_iterations(&base_setup, None, cap)
            .map_err(|e| refusal(format!("that column could not start: {e}"), None))?;
        let vapour = state
            .vapour
            .clone()
            .unwrap_or_else(|| vec![setup.feed_molar; plan.stages]);
        let mut profile = self
            .newton_with_water(
                &base_equations,
                &state.temperatures,
                &vapour,
                &draw_rates(
                    plan,
                    &state.draws,
                    setup.feed_mass,
                    setup.pass.slate,
                    &state.liquid,
                ),
                None,
                Vec::new(),
                &[],
            )
            .map_err(|unsolved| {
                refusal(
                    format!("that column {}", unsolved.reason),
                    unsolved.flows.as_deref(),
                )
            })?;
        let start_duty = base_equations
            .implied_reboiler_duty(
                &Start {
                    temperatures: &profile.temperatures,
                    vapour: &profile.flows.vapour,
                    draw_rates: &profile_draw_rates(plan, &profile),
                    free_water: &profile.free_water,
                },
                profile.reflux_ratio,
                &profile.wet_stages(),
            )
            .map_err(|e| refusal(format!("its reboiler's duty could not be read: {e}"), None))?;

        // The walk: duty and pumparounds together, a straight line in `λ`.
        for step in 1..=CONTINUATION_STEPS {
            let last = step == CONTINUATION_STEPS;
            let share = f64::from(step) / f64::from(CONTINUATION_STEPS);
            let duty = start_duty + share * (held - start_duty);
            let mut step_plan = base_plan.clone();
            for (heat, target) in step_plan.side_heat.iter_mut().zip(&plan.side_heat) {
                *heat += share * (target - *heat);
            }
            let step_spec = CascadeSpec {
                reboiler_duty: Some(Watt(duty)),
                ..setup.spec.clone()
            };
            let walked = ColumnEquations {
                plan: &step_plan,
                spec: &step_spec,
                ..*equations
            };
            let step_equations = if last { equations } else { &walked };
            profile = self
                .newton_with_water(
                    step_equations,
                    &profile.temperatures,
                    &profile.flows.vapour,
                    &profile_draw_rates(plan, &profile),
                    Some(profile.reflux_ratio),
                    profile.wet_stages(),
                    &profile.free_water,
                )
                .map_err(|unsolved| {
                    refusal(
                        format!(
                            "the walk from its closure of {:.3} MW stopped at step {step} of \
                             {CONTINUATION_STEPS}, the reboiler at {:.3} MW: Newton {}",
                            start_duty / 1.0e6,
                            if last { held } else { duty } / 1.0e6,
                            unsolved.reason
                        ),
                        unsolved.flows.as_deref(),
                    )
                })?;
        }
        Ok(profile)
    }

    /// Newton under the cut-back draws' active set (M67.1, docs/DESIGN.md
    /// §72.1), around the wet stages' and the floor's. The first solve is the
    /// one this column has always had — every draw at its declared share, from
    /// the start as given — unless the start carries a set (the last tick's,
    /// `CascadeProfile::cut_back`), so a column that never runs a draw tray
    /// short is what it was to the bit, and one held at an edge solves once a
    /// tick. From an answer or a failure the set moves:
    ///
    /// - an answer whose draw tray runs under the minimum liquid cuts that draw
    ///   back, and one whose cut-back draw would take more than its declared
    ///   share releases it; the next solve starts from that answer;
    /// - a failure whose last iterate runs a draw tray dry (`physical_cause`'s
    ///   one per cent of the feed, or the minimum where that is more) cuts that
    ///   draw back; the next solve starts from this call's start.
    ///
    /// A start's cut-back draws are set top-down ([`cut_start`]).
    ///
    /// Where no set holds, the first answer stands if there was one (a draw
    /// tray short of the minimum but wet, as before), else the failure — and a
    /// carried set that found nothing gives way to the first solve from the
    /// start as given, so a wrong hint costs a solve, never an answer.
    #[allow(clippy::too_many_arguments)]
    fn newton_with_water(
        &self,
        equations: &ColumnEquations<'_>,
        temperatures: &[f64],
        vapour: &[f64],
        draw_rates: &[f64],
        reflux_ratio: Option<f64>,
        wet: Vec<usize>,
        free_water: &[f64],
    ) -> Result<Profile, Unsolved> {
        let plan = equations.plan;
        let minimum = equations.liquid_minimum;
        let side: Vec<usize> = (1..plan.mass_ratios.len() - 1)
            .filter(|i| matches!(plan.locations[*i], DrawLocation::Stage(_)))
            .collect();
        let tray_of = |i: usize| match plan.locations[i] {
            DrawLocation::Stage(j) => j,
            DrawLocation::Condenser => 0,
        };
        let mut cut: Vec<usize> = equations.cut_back.to_vec();
        let mut rates = draw_rates.to_vec();
        if !cut.is_empty() {
            cut_start(equations, vapour, &mut rates, reflux_ratio, &mut cut);
        }
        let mut start_temperatures = temperatures.to_vec();
        let mut start_vapour = vapour.to_vec();
        let mut start_reflux = reflux_ratio;
        let mut start_wet = wet.clone();
        let mut start_free_water = free_water.to_vec();
        // The first answer at the declared shares, which stands where no set
        // holds; never one with a draw cut back, which the release may reject.
        let mut uncut: Option<Profile> = None;
        let mut failure = None;
        for _ in 0..side.len() + 2 {
            let uncut_attempt = cut.is_empty();
            let attempt = ColumnEquations {
                cut_back: &cut,
                ..*equations
            };
            let result = self.newton_water_set(
                &attempt,
                &start_temperatures,
                &start_vapour,
                &rates,
                start_reflux,
                start_wet.clone(),
                &start_free_water,
            );
            match result {
                Ok(mut profile) => {
                    // Released: a cut-back draw its tray could fill at its share
                    // (the margin keeps a draw ON the minimum from flickering).
                    let release: Vec<usize> = cut
                        .iter()
                        .copied()
                        .filter(|i| {
                            profile.ratios[*i] > plan.mass_ratios[*i] * (1.0 + RELEASE_MARGIN)
                        })
                        .collect();
                    let add: Vec<usize> = side
                        .iter()
                        .copied()
                        .filter(|i| {
                            !cut.contains(i)
                                && profile.flows.liquid_down[tray_of(*i)] < minimum * (1.0 - 1.0e-9)
                        })
                        .collect();
                    if release.is_empty() && add.is_empty() {
                        profile.cut_back = cut;
                        return Ok(profile);
                    }
                    cut.retain(|i| !release.contains(i));
                    cut.extend(add);
                    cut.sort_unstable();
                    // The next solve from this answer: its own rates, the
                    // cut-back ones re-cut on its vapour and reflux. A
                    // released draw starts at the rate it ran at, which its
                    // tray supplied, not its declared one: on this answer's
                    // vapour the declared one need not fit (measured: back up
                    // from a reflux of 0.6 to 1 on the crude unit, the start
                    // left the physical region and the draw was cut again).
                    rates = profile_draw_rates(plan, &profile);
                    start_temperatures = profile.temperatures.clone();
                    start_vapour = profile.flows.vapour.clone();
                    start_reflux = Some(profile.reflux_ratio);
                    start_wet = profile.wet_stages();
                    start_free_water = profile.free_water.clone();
                    cut_start(equations, &start_vapour, &mut rates, start_reflux, &mut cut);
                    // Nor is a released draw cut again before its next solve
                    // (M72.3, docs/DESIGN.md §77.13): the start cuts a draw for
                    // a tray between it and the next draw, the solve holds its
                    // own — measured on the 13-tray loop column at a reflux of
                    // 0.05, the diesel was cut for the tray under it, ran over
                    // its share, was released and cut again until the passes
                    // ran out, and the gas oil's tray stood at 2 mol/s uncut.
                    cut.retain(|i| !release.contains(i));
                    if uncut.is_none() && uncut_attempt {
                        uncut = Some(profile);
                    }
                    continue;
                }
                Err(unsolved) => {
                    let dry = minimum.max(DRY_TRAY_OF_FEED * equations.feed_molar) * (1.0 + 1.0e-9);
                    let add: Vec<usize> = match unsolved.flows.as_deref() {
                        Some(flows) => side
                            .iter()
                            .copied()
                            .filter(|i| !cut.contains(i) && flows.liquid_down[tray_of(*i)] <= dry)
                            .collect(),
                        None => Vec::new(),
                    };
                    failure = Some(unsolved);
                    if add.is_empty() {
                        break;
                    }
                    cut.extend(add);
                    cut.sort_unstable();
                }
            }
            rates = draw_rates.to_vec();
            cut_start(equations, &start_vapour, &mut rates, start_reflux, &mut cut);
        }
        if let Some(profile) = uncut {
            return Ok(profile);
        }
        if !equations.cut_back.is_empty() {
            let declared = ColumnEquations {
                cut_back: &[],
                ..*equations
            };
            return self.newton_water_set(
                &declared,
                temperatures,
                vapour,
                draw_rates,
                reflux_ratio,
                wet,
                free_water,
            );
        }
        Err(failure.unwrap_or(Unsolved {
            reason: "found no set of cut-back draws that holds".into(),
            flows: None,
        }))
    }

    /// Newton under the wet stages' active set (§67.17 rule 6), around the
    /// floor's: solve with `wet`; every dry stage past saturation turns wet, or
    /// else every wet stage whose free water came out negative turns dry, and
    /// the next solve starts again from THIS call's start, carrying only the
    /// free water the last one found — a solve on the wrong set ends where no
    /// column stands (a dry tray under a wet one boiling its water at 315 K,
    /// measured on the wet-trays reference), and Newton from there fails. A dry
    /// feed has no wet set and is `newton` alone.
    #[allow(clippy::too_many_arguments)]
    fn newton_water_set(
        &self,
        equations: &ColumnEquations<'_>,
        temperatures: &[f64],
        vapour: &[f64],
        draw_rates: &[f64],
        reflux_ratio: Option<f64>,
        mut wet: Vec<usize>,
        free_water: &[f64],
    ) -> Result<Profile, Unsolved> {
        if equations.water.is_none() {
            return self.newton(
                equations,
                temperatures,
                vapour,
                draw_rates,
                reflux_ratio,
                &[],
                &[],
            );
        }
        let plan = equations.plan;
        let n = plan.stages;
        let mut profile = self.newton(
            equations,
            temperatures,
            vapour,
            draw_rates,
            reflux_ratio,
            &wet,
            free_water,
        )?;
        for _ in 0..=n {
            let over: Vec<usize> = (0..n)
                .filter(|j| !wet.contains(j) && profile.saturation[*j] > 1.0 + SATURATION_MARGIN)
                .collect();
            let negative: Vec<usize> = wet
                .iter()
                .copied()
                .filter(|j| profile.free_water[*j] < 0.0)
                .collect();
            if negative.is_empty() && over.is_empty() {
                return Ok(profile);
            }
            // Every stage that flips, flips together: one at a time walks
            // through sets no column stands in — the top tray wet alone sends
            // its water to a dry tray below, which boils it at 35 °C (measured
            // on the wet-trays reference, §67.17).
            if negative.is_empty() {
                wet.extend(over);
                wet.sort_unstable();
            } else {
                wet.retain(|j| !negative.contains(j));
            }
            profile = self.newton(
                equations,
                temperatures,
                vapour,
                draw_rates,
                reflux_ratio,
                &wet,
                &profile.free_water,
            )?;
        }
        Err(Unsolved {
            reason: format!(
                "found no set of wet stages that holds in {} solves: a stage's free water and \
                 its saturation kept trading places",
                n + 2
            ),
            flows: Some(Box::new(profile.flows)),
        })
    }

    /// Newton under the vapour floor's active set (§66.3): the declared reflux
    /// first, and where that leaves a stage under the floor, the floor on the
    /// stage that binds with the reflux an unknown. A start that ran above its
    /// declared reflux with a stage at the floor tries the floor first, on the
    /// stage it bound; one above it off the floor (the reflux lowered, M72.5,
    /// §77.14) tries the declared reflux first and the floor after it.
    #[allow(clippy::too_many_arguments)]
    fn newton(
        &self,
        equations: &ColumnEquations<'_>,
        temperatures: &[f64],
        vapour: &[f64],
        draw_rates: &[f64],
        reflux_ratio: Option<f64>,
        wet: &[usize],
        free_water: &[f64],
    ) -> Result<Profile, Unsolved> {
        let plan = equations.plan;
        let n = plan.stages;
        let start = Start {
            temperatures,
            vapour,
            draw_rates,
            free_water,
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
        let profile = Profile::from_solved;

        // A held reboiler (M64, §69.1): its duty the equation, the reflux the
        // result, no floor — one solve, no active set.
        if let Some(duty) = equations.spec.reboiler_duty {
            let mut start_vapour = vapour.to_vec();
            if let Some(ran) = reflux_ratio {
                // A warm start's own reflux, carried by its top vapour.
                start_vapour[0] = (ran + 1.0) * draw_rates[0];
            }
            let start = Start {
                vapour: &start_vapour,
                ..start
            };
            return equations
                .solve(
                    &start,
                    Specification::Held(duty.value()),
                    wet,
                    self.max_iterations,
                )
                .map(profile);
        }

        // The reflux the start ran at: a warm start's own, where its profile
        // carries it, else its top vapour over its distillate.
        let ran = reflux_ratio.unwrap_or_else(|| vapour[0] / draw_rates[0] - 1.0);
        let above = ran > declared * (1.0 + 1.0e-6);
        // Above the declared reflux AT the floor: the floor bound the start, so
        // it is tried first (§66.3). Above it off the floor, the player lowered
        // the reflux, and the floor solve has no answer to find (§77.14).
        let at_floor = vapour[1..]
            .iter()
            .any(|v| *v <= floor * (1.0 + AT_FLOOR_MARGIN));
        let floor_first = above && at_floor;
        let floor_solve = || {
            equations
                .solve(
                    &start,
                    Specification::Floor(lowest(vapour)),
                    wet,
                    self.max_iterations,
                )
                .ok()
                .filter(|solved| holds(solved))
        };
        if floor_first {
            if let Some(solved) = floor_solve() {
                return Ok(profile(solved));
            }
        }
        let declared_solve =
            equations.solve(&start, Specification::Reflux, wet, self.max_iterations);
        // The floor-first attempt, late, wherever the declared solve did not
        // return a column standing above the floor — so every answer the
        // floor-first order returns, this order returns too.
        let stands = matches!(&declared_solve,
            Ok(solved) if solved.flows.vapour[1..].iter().all(|v| *v >= floor));
        if above && !floor_first && !stands {
            if let Some(solved) = floor_solve() {
                return Ok(profile(solved));
            }
        }
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
            match equations.solve(
                &start,
                Specification::Floor(binding),
                wet,
                self.max_iterations,
            ) {
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

/// A dry stage counts as past its saturation once its water vapour share is
/// this much over `K_w` [-] — a rounding margin, so a stage that a solve left
/// AT saturation (one just dried) is not flipped back on its last bit.
const SATURATION_MARGIN: f64 = 1.0e-9;

/// A start's stage vapour within this share over the vapour floor counts as
/// AT the floor [-] (M72.5, docs/DESIGN.md §77.14): a column Newton held at the
/// floor carries the floor's own number on that stage to rounding (1e-16), so
/// a millionth sees it, and no column standing freely above the floor sits
/// that close to it. A cold start's floored answer sits 2e-6 over: its first
/// warm tick takes the declared-first order, the same answer.
const AT_FLOOR_MARGIN: f64 = 1.0e-6;

/// A start's side draws cut back top-down (M67.1, docs/DESIGN.md §72.1): each
/// in `cut`, and each that would leave less than the minimum liquid on a tray
/// between it and the next draw, set to what leaves exactly that — never more
/// than its rate in `rates`, never under [`STOPPED_DRAW_OF_FEED`] — and added
/// to `cut`. The balance is the solve's own (`ColumnEquations::evaluate`):
/// the vapour into the condenser `(R + 1)·D` at the reflux the solve starts at,
/// every other vapour `vapour`'s.
fn cut_start(
    equations: &ColumnEquations<'_>,
    vapour: &[f64],
    rates: &mut [f64],
    reflux_ratio: Option<f64>,
    cut: &mut Vec<usize>,
) {
    let plan = equations.plan;
    let n = plan.stages;
    let draws = rates.len();
    let ran = reflux_ratio.unwrap_or(vapour[0] / rates[0] - 1.0);
    // The reflux the solve starts at: a held column's its own, else the
    // commanded one (a floor that binds only adds liquid).
    let ratio = match equations.spec.reboiler_duty {
        Some(_) => ran,
        None => equations.spec.reflux_ratio,
    };
    // As the solve's balance: the vapour into the condenser is (R + 1)·D.
    let top_vapour = (ratio + 1.0) * rates[0];
    // What each stage's balance adds to the liquid passing down, its draw
    // aside: the vapour from below and the feed, less the vapour leaving.
    let gain: Vec<f64> = (0..n - 1)
        .map(|j| {
            let feed_here = if j == plan.feed_stage {
                equations.feed_molar
            } else {
                0.0
            };
            let leaving = if j == 0 { top_vapour } else { vapour[j] };
            vapour[j + 1] + feed_here - leaving
        })
        .collect();
    let draw_here = |j: usize| matches!(plan.draw_at_stage[j], Some(i) if i > 0 && i < draws);
    let mut liquid = ratio * rates[0];
    for j in 0..n - 1 {
        let mut down = liquid + gain[j];
        if draw_here(j) {
            let i = plan.draw_at_stage[j].unwrap_or(0);
            // The lowest the liquid falls, relative to this tray's, on the
            // stages from here down to the next draw.
            let mut run = 0.0_f64;
            let mut lowest = 0.0_f64;
            for (k, stage_gain) in gain.iter().enumerate().skip(j + 1) {
                if draw_here(k) {
                    break;
                }
                run += stage_gain;
                lowest = lowest.min(run);
            }
            let most = down + lowest - equations.liquid_minimum;
            // A draw that leaves its tray exactly the minimum — a released
            // one, starting at the rate it ran at cut back — is not cut again
            // on rounding.
            if cut.contains(&i) || rates[i] - most > RECUT_MARGIN_OF_FEED * equations.feed_molar {
                rates[i] = most
                    .min(rates[i])
                    .max(STOPPED_DRAW_OF_FEED * equations.feed_molar);
                if !cut.contains(&i) {
                    cut.push(i);
                }
            }
            down -= rates[i];
        }
        liquid = down;
    }
    cut.sort_unstable();
}

/// Each draw but the bottoms' molar rate [mol/s], from its mass ratio and its
/// current molar mass — then each running loop's circulation [mol/s] at
/// `liquid`, the unknowns Newton carries after the draws' (§77.1).
fn draw_rates(
    plan: &CascadePlan,
    draws: &[DrawState],
    feed_mass: f64,
    slate: &Slate,
    liquid: &[Vec<f64>],
) -> Vec<f64> {
    let mut rates: Vec<f64> = (0..plan.mass_ratios.len() - 1)
        .map(|i| plan.mass_ratios[i] * feed_mass / draws[i].mean_molar_mass)
        .collect();
    rates.extend(plan.loop_molar_flows(slate, liquid));
    rates
}

/// Each draw but the bottoms' molar rate [mol/s] as a solved profile ran it:
/// the distillate the condenser's, every side draw its stage's — then each
/// running loop's circulation [mol/s].
fn profile_draw_rates(plan: &CascadePlan, profile: &Profile) -> Vec<f64> {
    let mut rates: Vec<f64> = (0..plan.mass_ratios.len() - 1)
        .map(|i| match plan.locations[i] {
            DrawLocation::Condenser => profile.flows.vapour[0] - profile.flows.reflux,
            DrawLocation::Stage(j) => profile.flows.liquid_drawn[j],
        })
        .collect();
    rates.extend(
        plan.loops
            .iter()
            .map(|l| profile.flows.circulation_out[l.draw]),
    );
    rates
}

/// How each declared loop ran on a solved profile (M72.1, docs/DESIGN.md
/// §77.1), in `CascadeSpec::loops`' order: a running loop's circulation, its
/// return and the duty `P·(h_d − h_ret)` [W]; a loop at zero flow at its
/// setpoint with no duty. Each draws its stage's liquid at that stage's
/// temperature, running or not — the state its line's first pipe carries
/// (M72.2, §77.2).
fn loop_runs(
    plan: &CascadePlan,
    spec: &CascadeSpec,
    slate: &Slate,
    profile: &Profile,
) -> Result<Vec<LoopRun>, SimError> {
    let molar: Vec<f64> = plan
        .loops
        .iter()
        .map(|l| profile.flows.circulation_out[l.draw])
        .collect();
    let returns = plan.loop_returns(slate, &profile.liquid, &profile.temperatures, &molar);
    let mut runs = Vec::with_capacity(spec.loops.len());
    for declared in &spec.loops {
        let draw = declared.draw_stage as usize - 1;
        runs.push(LoopRun {
            flow: KgPerSec(0.0),
            return_temperature: declared.return_temperature,
            duty: Watt::ZERO,
            at_capacity: false,
            drawn: MoleFractions::from_amounts(&profile.liquid[draw])?.to_mass(slate)?,
            drawn_temperature: Kelvin(profile.temperatures[draw]),
        });
    }
    for ((l, flow), back) in plan.loops.iter().zip(&molar).zip(&returns) {
        let x = &profile.liquid[l.draw];
        let drawn = liquid_enthalpy(slate, x, Kelvin(profile.temperatures[l.draw]));
        // The return's temperature from its enthalpy: `h = c̄p·(T − T_REF)`
        // per mole at constant heat capacity, the datum `liquid_enthalpy`'s.
        let molar_cp = mixture_molar_cp(slate, x);
        let run = &mut runs[l.index];
        run.flow = KgPerSec(flow * crate::cascade::molar_mass(slate, x));
        run.return_temperature = Kelvin(T_REF.value() + back.enthalpy / molar_cp);
        run.duty = Watt(flow * (drawn - back.enthalpy));
        run.at_capacity = back.at_capacity;
    }
    Ok(runs)
}

/// The heat each stage's side cooling removes [W] on a held column's base
/// (M64.2, docs/DESIGN.md §69.3): the column's own pumparounds, topped up to
/// [`BASE_SIDE_COOLING`] times the feed's latent heat flow (`energy_scale`,
/// [W]) in all, what is missing spread evenly over the stages between the top
/// and the feed. A column whose own already reach it, or fed onto its top two
/// stages with none between, takes its own alone.
fn base_side_heat(plan: &CascadePlan, energy_scale: f64) -> Vec<f64> {
    let mut heat = plan.side_heat.clone();
    let between = plan.feed_stage.saturating_sub(1);
    let missing = BASE_SIDE_COOLING * energy_scale - heat.iter().sum::<f64>();
    if between > 0 && missing > 0.0 {
        let each = missing / between as f64;
        for stage in heat.iter_mut().take(plan.feed_stage).skip(1) {
            *stage += each;
        }
    }
    heat
}

/// Pure `component`, as mass fractions — the water draw's composition.
fn pure(slate: &Slate, component: usize) -> Result<Composition, SimError> {
    let mut weights = vec![0.0; slate.len()];
    weights[component] = 1.0;
    Composition::from_weights(&weights)
}

/// A feed with no oil at all — the water slug a tank sends first (M62.3) —
/// answered rather than refused (§67.17 rule 10): every drop leaves by the
/// water draw, at water's own bubble point at the column's pressure, boiled up
/// and condensed; the oil draws carry nothing. The condenser takes the
/// water's latent heat there, and the reboiler closes the column's external
/// balance on it — a feed hotter than steam at that point would need a
/// negative reboiler and is refused, as `column_duties` refuses one.
fn oil_free_feed(
    pass: &ColumnPass<'_>,
    thermo: &dyn ThermoModel,
    plan: &CascadePlan,
    water_draw: Option<(usize, usize)>,
    reflux_ratio: f64,
) -> Result<Separation, SimError> {
    let slate = pass.slate;
    let Some((at, water)) = water_draw else {
        return Err(SimError::Scenario(
            "heat-balance cascade: the feed is all water and this column has no water draw".into(),
        ));
    };
    let feed_flow = pass.feed_flow.value();
    let boiling = bubble_temperature(
        slate,
        thermo,
        &MoleFractions::from_mass(pass.feed, slate)?,
        pass.pressure,
        "the feed's water",
    )?;
    let mut draws: Vec<DrawSeparation> = plan
        .mass_ratios
        .iter()
        .map(|_| DrawSeparation {
            split: 0.0,
            composition: pass.feed.clone(),
            temperature: pass.temperature,
        })
        .collect();
    draws.insert(
        at,
        DrawSeparation {
            split: 1.0,
            composition: pure(slate, water)?,
            temperature: boiling,
        },
    );
    let component = slate.get(water);
    let condenser =
        feed_flow / component.molar_mass.value() * thermo.dh_vap(slate, water, boiling)?.value();
    let reboiler =
        condenser + feed_flow * component.cp.value() * (boiling.value() - pass.temperature.value());
    if !reboiler.is_finite() || reboiler < 0.0 || !condenser.is_finite() {
        return Err(SimError::Numerical(format!(
            "heat-balance cascade: the feed is all water, arriving at {:.2} K above steam at \
             {:.2} K, so its column would need a reboiler of {reboiler:.4e} W — heat taken out, \
             which a reboiler does not do.",
            pass.temperature.value(),
            boiling.value()
        )));
    }
    Ok(Separation {
        draws,
        profile: None,
        condenser_duty: Some(Watt(condenser)),
        reboiler_duty: Some(Watt(reboiler)),
        feed_vapour_fraction: None,
        reflux_ratio: Some(reflux_ratio),
        reboiler_declined: None,
        cut_back: Vec::new(),
        water_flooded: None,
        loops: Vec::new(),
        stage_liquid: Vec::new(),
        trays: Vec::new(),
        holdup: None,
        holdup_reading: None,
        draw_flows: Vec::new(),
    })
}

/// The least share of a column's inflow by mass that is water — the feed's
/// free water and the steam together — for a refused solve to be answered by
/// passing the feed through (M63.4, docs/DESIGN.md §68.4). A GAME VALUE, set
/// under what was measured: the crude unit's column separates to 40 % water and
/// refuses from 50 %, M62.5's eight-stage test column separates to 16 % and
/// refuses from 18 %. Under it a refusal halts the run with its reason, as any
/// column's does.
pub const FLOODED_WATER_SHARE: f64 = 0.15;

/// What a flooded column's pass-through needs of its pass, or `None` where it
/// would not pass the feed through: no free water on the slate, no cascade or
/// no water draw, a plan or a steam rate that does not read, or the feed's free
/// water and the steam together under [`FLOODED_WATER_SHARE`] of the inflow.
/// The cold start's halvings ask it too (M65.2, docs/DESIGN.md §70.2): a feed
/// that would flood is not retried.
struct Flooding<'a> {
    index: usize,
    spec: &'a CascadeSpec,
    at: usize,
    plan: CascadePlan,
    steam: Option<SteamWater>,
    feed_flow: f64,
    throughput: f64,
    molar_mass: f64,
    feed_water: f64,
    water_mass: f64,
}

fn flooding<'a>(pass: &ColumnPass<'a>) -> Option<Flooding<'a>> {
    let slate = pass.slate;
    let (Some(index), Some(spec)) = (slate.free_water(), pass.cascade) else {
        return None;
    };
    let Ok((oil_draws, Some(at))) = split_water_draw(pass.draws) else {
        return None;
    };
    let plan = CascadePlan::new(spec, &oil_draws, slate.len()).ok()?;
    let steam = steam_water(pass, slate, &plan, Some((at, index))).ok()?;
    let feed_flow = pass.feed_flow.value();
    let throughput = pass.throughput().value();
    let molar_mass = slate.get(index).molar_mass.value();
    let feed_water = feed_flow * pass.feed.fractions()[index];
    let water_mass = feed_water + steam.map_or(0.0, |s| s.molar * molar_mass);
    if !throughput.is_finite() || throughput <= 0.0 || water_mass / throughput < FLOODED_WATER_SHARE
    {
        return None;
    }
    Some(Flooding {
        index,
        spec,
        at,
        plan,
        steam,
        feed_flow,
        throughput,
        molar_mass,
        feed_water,
        water_mass,
    })
}

/// A column whose solve refused a feed at least `FLOODED_WATER_SHARE` water
/// (M63.4, docs/DESIGN.md §68.4) — a tank's water slug running out, the one
/// tick it sends its last water and its first oil together (M62.3; ledger row
/// C13) — answered rather than halted: the column does not fractionate this
/// tick. Every drop of water, the feed's and the steam's, leaves by the water
/// draw at water's bubble point at the column's pressure, boiled up and
/// condensed as `water_only_inflow` takes it. The oil, below the feed's bubble
/// point, falls to the bottoms whole at the feed's temperature, carrying its
/// own heat, and the other draws carry nothing (M63.4). Above it the column is
/// a flash zone with no trays (M65.2, §70.2): the whole feed flashes
/// adiabatically at the column's pressure, as the feed's flash share is read
/// every tick; the oil's liquid falls to the bottoms at the flash temperature,
/// its vapour is condensed to the top draw at its bubble point, the other draws
/// stand full of the bottoms' liquid, not moving, and the condenser takes what
/// the oil gives up. The refusal rides the answer (`Separation::water_flooded`),
/// so a pass-through never hides a cause that is not the water.
///
/// Anything short of that — a drier feed, no water draw, no free water on the
/// slate — returns the refusal untouched.
fn flooded_pass_through(
    pass: &ColumnPass<'_>,
    thermo: &dyn ThermoModel,
    refusal: SimError,
) -> Result<Separation, SimError> {
    let slate = pass.slate;
    let Some(Flooding {
        index,
        spec,
        at,
        plan,
        steam,
        feed_flow,
        throughput,
        molar_mass,
        feed_water,
        water_mass,
    }) = flooding(pass)
    else {
        return Err(refusal);
    };
    let oil_mass = feed_flow - feed_water;
    let oil: Vec<f64> = pass
        .feed
        .fractions()
        .iter()
        .enumerate()
        .map(|(c, x)| if c == index { 0.0 } else { *x })
        .collect();

    // The water: in at its own share of the feed's flux and the steam's, out
    // a liquid at its bubble point — `water_only_inflow`'s balance.
    let water = WaterFeed::new(slate, index, feed_water / molar_mass, steam);
    let boiling = bubble_temperature(
        slate,
        thermo,
        &MoleFractions::from_mass(&pure(slate, index)?, slate)?,
        pass.pressure,
        "the column's water",
    )?;
    let inflow =
        feed_water * slate.get(index).cp.value() * (pass.temperature.value() - T_REF.value())
            + steam.map_or(0.0, |s| s.enthalpy_flux);
    let total = water.total_molar();
    let liquid = total * water.liquid_enthalpy(boiling);
    let vapour = total * water.vapour_enthalpy(slate, thermo, boiling)?;
    let reboiler = (vapour - inflow).max(0.0);
    let condenser = inflow + reboiler - liquid;
    if !condenser.is_finite() || condenser < 0.0 || !reboiler.is_finite() {
        return Err(refusal);
    }

    let bottoms = plan.mass_ratios.len() - 1;
    let whole = MoleFractions::from_mass(pass.feed, slate)?;
    let feed_bubble = bubble_temperature(slate, thermo, &whole, pass.pressure, "the feed")?;
    if oil_mass > 0.0 && pass.temperature.value() > feed_bubble.value() {
        let Some(top) = plan
            .locations
            .iter()
            .position(|location| matches!(location, DrawLocation::Condenser))
        else {
            // A flashing feed with no top draw has nowhere to send its vapour.
            return Err(refusal);
        };
        let zone = FlashZone::new(pass, thermo, &whole, feed_bubble, index, &oil)?;
        let condenser = condenser + zone.condenser;
        if !condenser.is_finite() || condenser < 0.0 {
            return Err(refusal);
        }
        let mut draws: Vec<DrawSeparation> = (0..plan.mass_ratios.len())
            .map(|_| DrawSeparation {
                split: 0.0,
                composition: zone.standing.0.clone(),
                temperature: zone.standing.1,
            })
            .collect();
        if zone.bottoms_mass > 0.0 {
            draws[bottoms] = DrawSeparation {
                split: zone.bottoms_mass / throughput,
                composition: zone.bottoms.clone(),
                temperature: zone.temperature,
            };
        }
        if let Some((mass, composition, temperature)) = zone.overhead {
            draws[top] = DrawSeparation {
                split: mass / throughput,
                composition,
                temperature,
            };
        }
        draws.insert(
            at,
            DrawSeparation {
                split: water_mass / throughput,
                composition: pure(slate, index)?,
                temperature: boiling,
            },
        );
        return Ok(Separation {
            draws,
            profile: None,
            condenser_duty: Some(Watt(condenser)),
            reboiler_duty: Some(Watt(reboiler)),
            feed_vapour_fraction: None,
            reflux_ratio: Some(spec.reflux_ratio),
            reboiler_declined: None,
            cut_back: Vec::new(),
            water_flooded: Some(refusal.to_string()),
            loops: Vec::new(),
            stage_liquid: Vec::new(),
            trays: Vec::new(),
            holdup: None,
            holdup_reading: None,
            draw_flows: Vec::new(),
        });
    }
    let mut draws: Vec<DrawSeparation> = (0..plan.mass_ratios.len())
        .map(|_| DrawSeparation {
            split: 0.0,
            composition: pass.feed.clone(),
            temperature: pass.temperature,
        })
        .collect();
    if oil_mass > 0.0 {
        draws[bottoms] = DrawSeparation {
            split: oil_mass / throughput,
            composition: Composition::from_weights(&oil)?,
            temperature: pass.temperature,
        };
    }
    draws.insert(
        at,
        DrawSeparation {
            split: water_mass / throughput,
            composition: pure(slate, index)?,
            temperature: boiling,
        },
    );
    Ok(Separation {
        draws,
        profile: None,
        condenser_duty: Some(Watt(condenser)),
        reboiler_duty: Some(Watt(reboiler)),
        feed_vapour_fraction: None,
        reflux_ratio: Some(spec.reflux_ratio),
        reboiler_declined: None,
        cut_back: Vec::new(),
        water_flooded: Some(refusal.to_string()),
        loops: Vec::new(),
        stage_liquid: Vec::new(),
        trays: Vec::new(),
        holdup: None,
        holdup_reading: None,
        draw_flows: Vec::new(),
    })
}

/// A flooded column's flash zone (M65.2, docs/DESIGN.md §70.2): the whole feed
/// flashed adiabatically at the column's pressure, the oil split by it.
struct FlashZone {
    /// The flash temperature [K]: the bottoms' and every standing draw's.
    temperature: Kelvin,
    /// The oil's liquid [kg/s] and its composition.
    bottoms_mass: f64,
    bottoms: Composition,
    /// The oil's vapour condensed: [kg/s], composition, and its bubble point
    /// at the column's pressure [K]. `None` when no oil flashes.
    overhead: Option<(f64, Composition, Kelvin)>,
    /// What an empty draw's line stands full of, and at what temperature [K]:
    /// the bottoms' liquid at the flash temperature (below its own bubble point
    /// at the column's pressure: it boiled at the oil's partial pressure), or,
    /// where every drop of oil flashed, the overhead condensed at its bubble
    /// point.
    standing: (Composition, Kelvin),
    /// The heat the oil gives up to the condenser [W]: its inflow on the
    /// engine's liquid datum less the bottoms' and the overhead's outflows.
    condenser: f64,
}

impl FlashZone {
    fn new(
        pass: &ColumnPass<'_>,
        thermo: &dyn ThermoModel,
        whole: &MoleFractions,
        bubble: Kelvin,
        water: usize,
        oil: &[f64],
    ) -> Result<Self, SimError> {
        let slate = pass.slate;
        let root = Kelvin(adiabatic_flash_temperature(
            slate,
            thermo,
            whole,
            pass.temperature,
            bubble,
            pass.pressure,
        )?);
        let flash = feed_flash(slate, thermo, whole, root, pass.pressure)?;
        // Moles of feed per second; each cut's liquid `(1 - beta)·x` of them,
        // the rest of the cut its vapour, so the oil's books close by
        // construction.
        let feed_flow = pass.feed_flow.value();
        let feed_molar = feed_flow / whole.mean_molar_mass(slate).value();
        let beta = flash.vapour_fraction;
        let mut liquid = vec![0.0; slate.len()];
        let mut vapour = vec![0.0; slate.len()];
        for c in (0..slate.len()).filter(|&c| c != water) {
            let cut = feed_flow * oil[c];
            let held = (feed_molar
                * (1.0 - beta)
                * flash.liquid.fractions()[c]
                * slate.get(c).molar_mass.value())
            .clamp(0.0, cut);
            liquid[c] = held;
            vapour[c] = cut - held;
        }
        let bottoms_mass: f64 = liquid.iter().sum();
        let overhead_mass: f64 = vapour.iter().sum();
        // A stream's enthalpy flux on the engine's liquid datum [W]:
        // `Σ m·cp·(T − T_REF)`, `energy::enthalpy_flux`'s for a liquid.
        let datum = |masses: &[f64], t: Kelvin| -> Result<f64, SimError> {
            let mass: f64 = masses.iter().sum();
            if mass <= 0.0 {
                return Ok(0.0);
            }
            Ok(mass
                * Composition::from_weights(masses)?.mixture_cp(slate).value()
                * (t.value() - T_REF.value()))
        };
        let inflow: Vec<f64> = oil.iter().map(|x| x * feed_flow).collect();
        let mut condenser = datum(&inflow, pass.temperature)? - datum(&liquid, root)?;
        let overhead = if overhead_mass > 0.0 {
            let composition = Composition::from_weights(&vapour)?;
            let condensed = bubble_temperature(
                slate,
                thermo,
                &MoleFractions::from_mass(&composition, slate)?,
                pass.pressure,
                "the flooded column's overhead",
            )?;
            condenser -= datum(&vapour, condensed)?;
            Some((overhead_mass, composition, condensed))
        } else {
            None
        };
        let (bottoms, standing) = match &overhead {
            Some((_, composition, condensed)) if bottoms_mass <= 0.0 => {
                (composition.clone(), (composition.clone(), *condensed))
            }
            _ => {
                let bottoms = Composition::from_weights(&liquid)?;
                (bottoms.clone(), (bottoms, root))
            }
        };
        Ok(FlashZone {
            temperature: root,
            bottoms_mass,
            standing,
            bottoms,
            overhead,
            condenser,
        })
    }
}

/// The steam the column takes this tick (M62.6, docs/DESIGN.md §67.18), or
/// `None` when its line carries none — a column with its steam valve shut is a
/// column without steam, M62.5's arithmetic bit for bit. Refused: steam on a
/// stage the column does not have, and steam into a column with no water draw
/// to decant its water.
fn steam_water(
    pass: &ColumnPass<'_>,
    slate: &Slate,
    plan: &CascadePlan,
    water_draw: Option<(usize, usize)>,
) -> Result<Option<SteamWater>, SimError> {
    let Some(steam) = pass.steam else {
        return Ok(None);
    };
    let flow = steam.flow.value();
    if !flow.is_finite() || flow < 0.0 {
        return Err(SimError::Numerical(format!(
            "heat-balance cascade: the steam's rate is {flow:.4e} kg/s; the engine hands a \
             column a steam rate that is finite and never negative."
        )));
    }
    if steam.stage < 1 || steam.stage as usize > plan.stages {
        return Err(SimError::Scenario(format!(
            "heat-balance cascade: steam enters stage {}, and this column's stages are \
             1..={} (docs/DESIGN.md §67.18).",
            steam.stage, plan.stages
        )));
    }
    let Some((_, index)) = water_draw else {
        return Err(SimError::Scenario(
            "heat-balance cascade: this column takes steam and declares no water draw to \
             decant its water at the condenser — { outlet = \"...\", stage = 0, phase = \
             \"water\" } (docs/DESIGN.md §67.18)."
                .into(),
        ));
    };
    if flow == 0.0 {
        return Ok(None);
    }
    let enthalpy_flux = steam_enthalpy_flux(slate, pass)?.unwrap_or(0.0);
    Ok(Some(SteamWater {
        stage: steam.stage as usize - 1,
        molar: flow / slate.get(index).molar_mass.value(),
        enthalpy_flux,
    }))
}

/// A column whose only inflow is water — steam with no feed (a feed pump
/// tripped while the steam stays on), or steam over a feed that is all water —
/// answered rather than refused (M62.6, docs/DESIGN.md §67.18): every drop
/// leaves by the water draw at water's bubble point at the column's pressure,
/// the oil draws carry nothing. The condenser takes what the water brings above
/// liquid at that point; the reboiler adds only what it takes to boil it all up
/// — none for steam, which arrives above saturation.
fn water_only_inflow(
    pass: &ColumnPass<'_>,
    thermo: &dyn ThermoModel,
    plan: &CascadePlan,
    water_draw: Option<(usize, usize)>,
    steam: SteamWater,
    feed_water_molar: f64,
    spec: &CascadeSpec,
) -> Result<Separation, SimError> {
    let slate = pass.slate;
    let Some((at, index)) = water_draw else {
        return Err(SimError::Scenario(
            "heat-balance cascade: this column takes steam and has no water draw".into(),
        ));
    };
    let water = WaterFeed::new(slate, index, feed_water_molar, Some(steam));
    let boiling = bubble_temperature(
        slate,
        thermo,
        &MoleFractions::from_mass(&pure(slate, index)?, slate)?,
        pass.pressure,
        "the column's water",
    )?;
    let feed_flux = pass.feed_flow.value()
        * pass.feed.mixture_cp(slate).value()
        * (pass.temperature.value() - T_REF.value());
    let inflow = feed_flux + steam.enthalpy_flux;
    let total = water.total_molar();
    let liquid = total * water.liquid_enthalpy(boiling);
    let vapour = total * water.vapour_enthalpy(slate, thermo, boiling)?;
    let reboiler = (vapour - inflow).max(0.0);
    let condenser = inflow + reboiler - liquid;
    if !condenser.is_finite() || condenser < 0.0 || !reboiler.is_finite() {
        return Err(SimError::Numerical(format!(
            "heat-balance cascade: a column whose only inflow is water came out with a \
             condenser duty of {condenser:.4e} W and a reboiler duty of {reboiler:.4e} W."
        )));
    }
    let mut draws: Vec<DrawSeparation> = plan
        .mass_ratios
        .iter()
        .map(|_| DrawSeparation {
            split: 0.0,
            composition: pass.feed.clone(),
            temperature: pass.temperature,
        })
        .collect();
    draws.insert(
        at,
        DrawSeparation {
            split: 1.0,
            composition: pure(slate, index)?,
            temperature: boiling,
        },
    );
    Ok(Separation {
        draws,
        profile: None,
        condenser_duty: Some(Watt(condenser)),
        reboiler_duty: Some(Watt(reboiler)),
        feed_vapour_fraction: None,
        reflux_ratio: Some(spec.reflux_ratio),
        reboiler_declined: None,
        cut_back: Vec::new(),
        water_flooded: None,
        loops: Vec::new(),
        stage_liquid: Vec::new(),
        trays: Vec::new(),
        holdup: None,
        holdup_reading: None,
        draw_flows: Vec::new(),
    })
}

/// The settings a player can act on (§65.9): past the crude unit's working
/// range the solve stops settling, and the settings it was given are the plain
/// part of the reason. A held column's reflux is a result, not a setting
/// (M64, §69.1): its held duty is named instead.
fn settings_sentence(spec: &CascadeSpec) -> String {
    let duties: Vec<String> = spec
        .pumparounds
        .iter()
        .map(|p| format!("{:.1} MW on stage {}", p.duty.value() / 1e6, p.stage))
        .collect();
    let (setting, joint) = match spec.reboiler_duty {
        None => (format!("at reflux ratio {}", spec.reflux_ratio), " with"),
        Some(duty) => (
            format!("with its reboiler held at {:.3} MW", duty.value() / 1e6),
            " and",
        ),
    };
    format!(
        " The column could not settle {setting}{} — outside the range it runs in.",
        if duties.is_empty() {
            String::new()
        } else {
            format!("{joint} its pumparounds at {}", duties.join(", "))
        }
    )
}

/// A physical edge the last iterate shows, in words (§66.2), or nothing: a tray
/// whose liquid has run out. (The probe's other edge, a reboiler with nothing
/// left to boil up, cannot be reached here: the vapour floor holds the
/// reboiler's boil-up at 20 % of the feed.)
///
/// A flow counts as gone at [`DRY_TRAY_OF_FEED`] of the feed.
fn physical_cause(plan: &CascadePlan, flows: Option<&Flows>, feed_molar: f64) -> String {
    let Some(flows) = flows else {
        return String::new();
    };
    let gone = DRY_TRAY_OF_FEED * feed_molar;
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
    /// A wet feed's water and the water vapour leaving each stage [mol/s]
    /// (§67.17 rule 7); `None` on a dry feed.
    water: Option<(&'a WaterFeed, &'a [f64])>,
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
    let condenser_t = match state.water {
        None => bubble_temperature_unnormalized(
            slate,
            state.thermo,
            &condensate,
            state.pressure,
            "the total condenser",
        )?,
        Some((water, water_vapour)) => condenser_temperature(
            slate,
            state.thermo,
            &condensate,
            reflux + distillate,
            water,
            water_vapour[0],
            state.pressure,
        )?,
    };
    let reflux_enthalpy = liquid_enthalpy(slate, &condensate, condenser_t);
    // The water vapour's enthalpy flux leaving each stage [W] (zero free water
    // on the warm-up's stages): a stage's balance pays for the water it sends
    // up and is paid by the water arriving from below.
    let mut water_flux = vec![0.0; stages];
    if let Some((water, water_vapour)) = state.water {
        for j in 0..stages {
            water_flux[j] = water_vapour[j]
                * water.vapour_enthalpy(slate, state.thermo, Kelvin(state.temperatures[j]))?;
        }
    }

    // A pumparound's heat, per mole: the stage balances are molar flows times
    // molar enthalpies, so its watts enter as they are (J/s against mol/s ·
    // J/mol). A loop (M72.1, §77.1) adds its circulation to the material
    // balances and, in the sweep's form, `P·(h_r − h_ret)` on its return stage
    // — `R_j + (in_j − out_j)·h_j`, its draw cancelling on the stage it leaves.
    let molar = plan.loop_molar_flows(slate, state.liquid);
    let (into, out) = plan.circulation(&molar);
    let returns = plan.loop_returns(slate, state.liquid, state.temperatures, &molar);
    let removed = plan.removed_heat(&liquid_enthalpies, &molar, &returns);
    let side_heat: Vec<f64> = (0..stages)
        .map(|j| removed[j] + (into[j] - out[j]) * liquid_enthalpies[j])
        .collect();
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
            vapour[j + 1] = if let Some((water, _)) = state.water {
                (liquid_in[j] * (here - enthalpy_above)
                    + feed_here * (here - feed_enthalpy)
                    + vapour[j] * (vapour_enthalpies[j] - here)
                    + side_heat[j]
                    + water_flux[j]
                    - water_flux[j + 1]
                    - SteamWater::flux_into(water.steam.as_ref(), j))
                    / (vapour_enthalpies[j + 1] - here)
            } else {
                (liquid_in[j] * (here - enthalpy_above)
                    + feed_here * (here - feed_enthalpy)
                    + vapour[j] * (vapour_enthalpies[j] - here)
                    + side_heat[j])
                    / (vapour_enthalpies[j + 1] - here)
            };
            // Material balance on stage j.
            liquid_down[j] =
                liquid_in[j] + vapour[j + 1] + feed_here + into[j] - drawn[j] - out[j] - vapour[j];
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
            liquid_down[j] =
                liquid_in[j] + vapour[j + 1] + feed_here + into[j] - drawn[j] - out[j] - vapour[j];
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
            circulation_in: into,
            circulation_out: out,
        },
        reflux_ratio,
    ))
}

/// A liquid's molar enthalpy [J/mol] on the engine's datum: `Σ x·M·cp·(T − T_REF)`
/// — `energy::enthalpy_flux`'s, per mole.
pub(crate) fn liquid_enthalpy(slate: &Slate, fractions: &[f64], temperature: Kelvin) -> f64 {
    mixture_molar_cp(slate, fractions) * (temperature.value() - T_REF.value())
}

/// A column's free water (M62.5, docs/DESIGN.md §67.17): where water sits on
/// the slate, how much of it the feed brings, and any steam (M62.6, §67.18).
#[derive(Clone)]
pub(crate) struct WaterFeed {
    /// The slate's `free_water` member.
    pub(crate) index: usize,
    /// The feed's water [mol/s], entering on the feed stage.
    pub(crate) feed_molar: f64,
    /// Water's molar mass [kg/mol].
    pub(crate) molar_mass: f64,
    /// Liquid water's molar heat capacity [J/(mol·K)], `M·cp`.
    molar_cp: f64,
    /// The steam into its stage, `None` on a column with none flowing.
    pub(crate) steam: Option<SteamWater>,
}

/// Steam into a stage (M62.6, docs/DESIGN.md §67.18).
#[derive(Debug, Clone, Copy)]
pub(crate) struct SteamWater {
    /// The stage it enters, zero-based.
    pub(crate) stage: usize,
    /// Its rate [mol/s], positive.
    pub(crate) molar: f64,
    /// Its enthalpy flux on the engine's datum [W], latent heat included — the
    /// steam line's own, `ṁ·cp·(T_le − T_REF)` at its liquid-equivalent
    /// temperature.
    pub(crate) enthalpy_flux: f64,
}

impl SteamWater {
    /// The steam's enthalpy flux into stage `j` [W]: its own on its stage,
    /// zero elsewhere.
    pub(crate) fn flux_into(steam: Option<&SteamWater>, j: usize) -> f64 {
        match steam {
            Some(steam) if steam.stage == j => steam.enthalpy_flux,
            _ => 0.0,
        }
    }
}

impl WaterFeed {
    fn new(slate: &Slate, index: usize, feed_molar: f64, steam: Option<SteamWater>) -> Self {
        let water = slate.get(index);
        Self {
            index,
            feed_molar,
            molar_mass: water.molar_mass.value(),
            molar_cp: water.molar_mass.value() * water.cp.value(),
            steam,
        }
    }

    /// All the water entering the column [mol/s]: the feed's and the steam's.
    pub(crate) fn total_molar(&self) -> f64 {
        match self.steam {
            None => self.feed_molar,
            Some(steam) => self.feed_molar + steam.molar,
        }
    }

    /// Liquid water's molar enthalpy [J/mol] on the engine's datum.
    pub(crate) fn liquid_enthalpy(&self, temperature: Kelvin) -> f64 {
        self.molar_cp * (temperature.value() - T_REF.value())
    }

    /// Water vapour's molar enthalpy [J/mol]: the liquid's plus the latent heat
    /// at `temperature` — Kirchhoff's on a declared vapour heat capacity (§67.1),
    /// the line flash's own vapour.
    pub(crate) fn vapour_enthalpy(
        &self,
        slate: &Slate,
        thermo: &dyn ThermoModel,
        temperature: Kelvin,
    ) -> Result<f64, SimError> {
        Ok(self.liquid_enthalpy(temperature)
            + thermo.dh_vap(slate, self.index, temperature)?.value())
    }
}

/// The water vapour leaving each stage [mol/s], from the free water leaving
/// each stage downward (§67.17 rule 2): `W_1 = F_w + S − FW_N`, then
/// `W_{j+1} = W_j + FW_j − FW_{j−1} − F_w[j = f] − S[j = s]` top-down — each
/// stage's water balance, the reflux bringing none (it is decanted), the steam
/// `S` joining its own stage (M62.6, §67.18).
///
/// Below a dry feed stage the water vapour is `F_w − F_w`, zero exactly; with
/// free water it can come out a rounding error under zero, which is zero — a
/// share of the feed's water under `WATER_ROUNDING`. Further under is an
/// iterate outside the physical region, an `Err`.
pub(crate) fn water_vapour_profile(
    plan: &CascadePlan,
    water: &WaterFeed,
    free_water: &[f64],
) -> Result<Vec<f64>, SimError> {
    const WATER_ROUNDING: f64 = 1.0e-9;
    let n = plan.stages;
    let mut vapour = vec![0.0; n];
    vapour[0] = water.total_molar() - free_water[n - 1];
    for j in 0..n - 1 {
        let from_above = if j == 0 { 0.0 } else { free_water[j - 1] };
        let fed = if j == plan.feed_stage {
            water.feed_molar
        } else {
            0.0
        };
        vapour[j + 1] = match water.steam {
            Some(steam) if steam.stage == j => {
                vapour[j] + free_water[j] - from_above - fed - steam.molar
            }
            _ => vapour[j] + free_water[j] - from_above - fed,
        };
    }
    for (j, v) in vapour.iter_mut().enumerate() {
        if *v < 0.0 {
            if !v.is_finite() || *v < -WATER_ROUNDING * water.total_molar() {
                return Err(SimError::Numerical(format!(
                    "heat-balance cascade: the water vapour leaving stage {} is {v:.4e} mol/s",
                    j + 1
                )));
            }
            *v = 0.0;
        }
    }
    Ok(vapour)
}

/// The total condenser's temperature [K] on a feed with free water: the WET
/// bubble point of the oil it condenses over the water decanted with it
/// (§67.17 rule 5), `Σ K_c·y_c + P_w/P = 1`; the dry bubble point of the oil
/// where no water reaches it.
pub(crate) fn condenser_temperature(
    slate: &Slate,
    thermo: &dyn ThermoModel,
    condensate: &[f64],
    oil_vapour: f64,
    water: &WaterFeed,
    water_vapour: f64,
    pressure: Pascal,
) -> Result<Kelvin, SimError> {
    if water_vapour <= 0.0 {
        return bubble_temperature_unnormalized(
            slate,
            thermo,
            condensate,
            pressure,
            "the total condenser",
        );
    }
    let mut amounts: Vec<f64> = condensate.iter().map(|y| y * oil_vapour).collect();
    amounts[water.index] += water_vapour;
    let mixture = MoleFractions::from_amounts(&amounts)?;
    bubble_temperature(
        slate,
        thermo,
        &mixture,
        pressure,
        "the total condenser's oil over its water",
    )
}

/// Each cut's K at the oil's partial pressure `P·V/(V + W)` (§67.17 rule 3),
/// from the total pressure's: Raoult's `K ∝ 1/P`, so times `(V + W)/V`.
fn effective_k(mut k: Vec<Vec<f64>>, oil_vapour: &[f64], water_vapour: &[f64]) -> Vec<Vec<f64>> {
    for ((row, v), w) in k.iter_mut().zip(oil_vapour).zip(water_vapour) {
        let scale = (v + w) / v;
        for value in row {
            *value *= scale;
        }
    }
    k
}

/// Every stage's bubble point at the oil's partial pressure `P·V/(V + W)`
/// (§67.17 rule 3), `P` the stage's own (`pressures`, one per stage) — the
/// warm-up's stage temperatures on a wet feed.
fn oil_bubble_profile(
    slate: &Slate,
    thermo: &dyn ThermoModel,
    liquid: &[Vec<f64>],
    oil_vapour: &[f64],
    water_vapour: &[f64],
    pressures: &[Pascal],
) -> Result<Vec<f64>, SimError> {
    let mut temperatures = Vec::with_capacity(liquid.len());
    for (j, x) in liquid.iter().enumerate() {
        let pressure = pressures[j].value() * oil_vapour[j] / (oil_vapour[j] + water_vapour[j]);
        temperatures.push(
            bubble_temperature_unnormalized(
                slate,
                thermo,
                x,
                Pascal(pressure),
                &format!("stage {} of {}", j + 1, liquid.len()),
            )?
            .value(),
        );
    }
    Ok(temperatures)
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
    if temperature.value() <= bubble.value() {
        return Ok(0.0);
    }
    let root = adiabatic_flash_temperature(slate, thermo, feed, temperature, bubble, pressure)?;
    let (_, share) = flash_energy_share(slate, thermo, feed, temperature, pressure, root)?;
    if !share.is_finite() {
        return Err(SimError::Numerical(format!(
            "heat-balance cascade: the feed's flash share came out as {share}"
        )));
    }
    Ok(share.clamp(0.0, 1.0))
}

/// The flash temperature `T*` [K] of a feed above its `bubble` point (M65.2,
/// split out of [`adiabatic_flash_share`], its arithmetic unchanged): the root,
/// by bisection between the bubble point and the feed's temperature, of the
/// isothermal flash's vapour share against the share the sensible heat pays for.
fn adiabatic_flash_temperature(
    slate: &Slate,
    thermo: &dyn ThermoModel,
    feed: &MoleFractions,
    temperature: Kelvin,
    bubble: Kelvin,
    pressure: Pascal,
) -> Result<f64, SimError> {
    let (mut low, mut high) = (bubble.value(), temperature.value());
    for _ in 0..FLASH_TEMPERATURE_HALVINGS {
        let middle = 0.5 * (low + high);
        let (flashed, paid_for) =
            flash_energy_share(slate, thermo, feed, temperature, pressure, middle)?;
        if flashed < paid_for {
            low = middle;
        } else {
            high = middle;
        }
    }
    Ok(0.5 * (low + high))
}

/// At a trial flash temperature `t` [K]: the isothermal flash's vapour share and
/// the share the feed's sensible heat from `temperature` down to `t` pays for.
fn flash_energy_share(
    slate: &Slate,
    thermo: &dyn ThermoModel,
    feed: &MoleFractions,
    temperature: Kelvin,
    pressure: Pascal,
    t: f64,
) -> Result<(f64, f64), SimError> {
    let heat_capacity = mixture_molar_cp(slate, feed.fractions());
    let flash = feed_flash(slate, thermo, feed, Kelvin(t), pressure)?;
    let latent = mixture_dh_vap(slate, thermo, flash.vapour.fractions(), Kelvin(t))?;
    Ok((
        flash.vapour_fraction,
        heat_capacity * (temperature.value() - t) / latent,
    ))
}

/// A column feed's isothermal flash: a feed with free water by the free-water
/// flash (§67.17 rule 11); every other feed by the flash it always took.
fn feed_flash(
    slate: &Slate,
    thermo: &dyn ThermoModel,
    feed: &MoleFractions,
    temperature: Kelvin,
    pressure: Pascal,
) -> Result<FlashResult, SimError> {
    match feed.standing_water(slate) {
        None => flash_isothermal(feed, slate, thermo, temperature, pressure),
        Some(_) => flash_on_the_line(feed, slate, thermo, temperature, pressure),
    }
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
            steam: None,
            reboiler_duty: None,
            loops: Vec::new(),
            trays: Vec::new(),
            holdup: None,
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
                running_reboiler: None,
                earlier_pass: None,
                steam: None,
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
        let k =
            stage_k_values_profile(slate, thermo, &stage_t, &vec![P_ATM; stage_t.len()]).unwrap();
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
                water: None,
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
                        running_reboiler: None,
                        earlier_pass: None,
                        steam: None,
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
        let k =
            stage_k_values_profile(&slate, &thermo, &stage_t, &vec![P_ATM; stage_t.len()]).unwrap();
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
                water: None,
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
            steam: None,
            reboiler_duty: None,
            loops: Vec::new(),
            trays: Vec::new(),
            holdup: None,
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
                running_reboiler: None,
                earlier_pass: None,
                steam: None,
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
            steam: None,
            reboiler_duty: None,
            loops: Vec::new(),
            trays: Vec::new(),
            holdup: None,
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
                running_reboiler: None,
                earlier_pass: None,
                steam: None,
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
        let k =
            stage_k_values_profile(&slate, &thermo, &stage_t, &vec![P_ATM; stage_t.len()]).unwrap();
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
                water: None,
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
    use refinery_core::graph::{ColumnDraw, NodeId, Pumparound, PumparoundLoop, TraySection};
    use refinery_core::units::{JPerKgK, JPerMol, KgPerM3, KgPerMol, KgPerSec, Meter, P_ATM};

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
                steam: None,
                reboiler_duty: None,
                loops: Vec::new(),
                trays: Vec::new(),
                holdup: None,
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
                running_reboiler: None,
                earlier_pass: None,
                steam: None,
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
            steam: None,
            reboiler_duty: None,
            loops: Vec::new(),
            trays: Vec::new(),
            holdup: None,
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
                    running_reboiler: None,
                    earlier_pass: None,
                    steam: None,
                },
                &thermo,
            )
            .unwrap();
        assert!(scorching.reflux_ratio.unwrap() > 0.5);
    }

    /// **A floor that stops binding lets go** (§66.3): a column held at the
    /// floor (a feed 60 K past its bubble point, the reflux lifted to 0.79
    /// above its declared 0.5) whose feed then cools to 40 K past it runs at
    /// exactly its declared reflux again — warm from the floored profile, whose
    /// floor attempt must refuse the floored answer it finds there, a reflux of
    /// 0.37, below the declared one (the M61.1 mutation pass). The profile is a
    /// cold start's, 2e-6 over the floor, so since M72.5 that attempt is the
    /// late one, after the declared solve leaves a stage under the floor.
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
            steam: None,
            reboiler_duty: None,
            loops: Vec::new(),
            trays: Vec::new(),
            holdup: None,
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
                        running_reboiler: None,
                        earlier_pass: None,
                        steam: None,
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
                steam: None,
                reboiler_duty: None,
                loops: Vec::new(),
                trays: Vec::new(),
                holdup: None,
            }
        }

        fn separate(
            &self,
            model: &HeatBalanceCascade,
            spec: &CascadeSpec,
            seed: Option<&CascadeProfile>,
        ) -> Result<Separation, SimError> {
            self.separate_on(model, spec, seed, &TroutonThermo::new())
        }

        fn separate_on(
            &self,
            model: &HeatBalanceCascade,
            spec: &CascadeSpec,
            seed: Option<&CascadeProfile>,
            thermo: &dyn ThermoModel,
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
                    running_reboiler: None,
                    earlier_pass: None,
                    steam: None,
                },
                thermo,
            )
        }
    }

    /// The shipped 13-tray crude unit's column (`scenarios/crude_unit_loops.toml`,
    /// M72.4, docs/DESIGN.md §77.7): the crude unit's cuts and feed, the draws
    /// on trays 4, 7 and 10, the feed on 11, three pumparound loops 4→3, 7→6,
    /// 10→9 at 85 / 95 / 65 kg/s returning at 150 / 215 / 270 °C, brought in
    /// cold at 15 / 15 / 10 MW.
    fn thirteen_trays() -> (CrudeUnit, CascadeSpec) {
        let mut unit = CrudeUnit::new();
        unit.draws = vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.22)),
            ColumnDraw::by_stage(NodeId(2), 4, Some(0.16)),
            ColumnDraw::by_stage(NodeId(3), 7, Some(0.20)),
            ColumnDraw::by_stage(NodeId(4), 10, Some(0.14)),
            ColumnDraw::by_stage(NodeId(5), 13, None),
        ];
        let loops = [
            (4, 85.0, 150.0, 15.0e6),
            (7, 95.0, 215.0, 15.0e6),
            (10, 65.0, 270.0, 10.0e6),
        ]
        .iter()
        .map(|(draw, flow, back_c, start)| PumparoundLoop {
            draw_stage: *draw,
            return_stage: draw - 1,
            flow: KgPerSec(*flow),
            return_temperature: Kelvin(back_c + 273.15),
            cooler_capacity: Watt(30.0e6),
            start_duty: Watt(*start),
            line: Vec::new(),
            cooler: None,
        })
        .collect();
        let spec = CascadeSpec {
            stages: 13,
            feed_stage: 11,
            reflux_ratio: 4.0,
            pumparounds: Vec::new(),
            steam: None,
            reboiler_duty: None,
            loops,
            trays: Vec::new(),
            holdup: None,
        };
        (unit, spec)
    }

    /// **The 13-tray column with loops starts cold only on the retry** (§77.5):
    /// started as every column starts — its loops on their start duties and
    /// handed over, then halved — it refuses (the boiling-order seed ends too
    /// soon on thirteen trays, §77.0 finding 8); with the seed held for its
    /// whole warm-up it starts, and every loop returns at its setpoint at the
    /// duty §77.0 finding 7 measured (15.06, 17.05, 12.79 MW).
    #[test]
    fn the_thirteen_tray_column_with_loops_starts_cold_on_the_retry() {
        let (unit, spec) = thirteen_trays();
        let refused = unit
            .separate(&HeatBalanceCascade::without_seed_retry(), &spec, None)
            .expect_err("the first cold start refuses");
        let message = refused.to_string();
        assert!(
            message.contains("Started again from its pumparounds halved, up to 4 times"),
            "{message}"
        );
        let started = unit
            .separate(&HeatBalanceCascade::new(), &spec, None)
            .expect("the retry starts it");
        let probe = [15.06e6, 17.05e6, 12.79e6];
        for ((run, declared), duty) in started.loops.iter().zip(&spec.loops).zip(probe) {
            assert!(!run.at_capacity, "{run:?}");
            assert!(
                (run.return_temperature.value() - declared.return_temperature.value()).abs() < 1e-9,
                "{run:?}"
            );
            assert!((run.duty.value() / duty - 1.0).abs() < 2e-3, "{run:?}");
        }
    }

    /// The 13-tray column with its trays in the loop plant fixture's six
    /// sections (`crates/scenarios/tests/tray_hydraulics.rs`, sized on the
    /// multipass layout of docs/DESIGN.md §78.10: 5.9 / 7.3 / 8.4 / 8.5 /
    /// 7.0 / 8.5 m, each loop's two stages in one), at `reflux_ratio`.
    fn thirteen_trays_with_trays(reflux_ratio: f64) -> (CrudeUnit, CascadeSpec) {
        let (unit, spec) = thirteen_trays();
        let trays = [
            (1, 2, 5.9, 0.18, 0.10),
            (3, 5, 7.3, 0.22, 0.15),
            (6, 8, 8.4, 0.24, 0.15),
            (9, 10, 8.5, 0.18, 0.12),
            (11, 11, 7.0, 0.18, 0.12),
            (12, 12, 8.5, 0.28, 0.15),
        ]
        .iter()
        .map(|(from, to, diameter, downcomer, holes)| TraySection {
            from: *from,
            to: *to,
            diameter: Meter(*diameter),
            spacing: Meter(0.76),
            weir_height: Meter(0.05),
            hole_diameter: Meter(0.005),
            hole_area_share: *holes,
            downcomer_share: *downcomer,
            clearance: Meter(0.045),
            passes: 4,
        })
        .collect();
        (
            unit,
            CascadeSpec {
                reflux_ratio,
                trays,
                ..spec
            },
        )
    }

    /// `|Σ K(T_j, P_j)·x_j − 1|` at its worst over the stages, each stage's
    /// K at `pressure_at(j)` — the bubble point every stage solved to.
    fn worst_summation(
        unit: &CrudeUnit,
        profile: &CascadeProfile,
        pressure_at: impl Fn(usize) -> Pascal,
    ) -> f64 {
        let thermo = TroutonThermo::new();
        let mut worst: f64 = 0.0;
        for (j, (t, x)) in profile.temperatures.iter().zip(&profile.liquid).enumerate() {
            let mut sum = 0.0;
            for (c, xc) in x.iter().enumerate() {
                sum += thermo.k_value(&unit.slate, c, *t, pressure_at(j)).unwrap() * xc;
            }
            worst = worst.max((sum - 1.0).abs());
        }
        worst
    }

    /// **Each stage solves at its own pressure** (M73.2, docs/DESIGN.md
    /// §78.9): the top at the column's 1.8 bar, each stage below higher by the
    /// drop fed back across the tray above it (to the re-solve's 0.5 Pa), the
    /// reboiler the last tray's plus its drop; every stage at its bubble point
    /// at ITS pressure and far off it at the top's (the probe's check, §78.8:
    /// 8.4e-8 and 4.3e-2); the feed flashed at its stage's pressure, not the
    /// top's.
    #[test]
    fn each_stage_solves_at_its_own_pressure() {
        let (unit, spec) = thirteen_trays_with_trays(4.0);
        let separation = unit
            .separate(&HeatBalanceCascade::new(), &spec, None)
            .unwrap();
        let trays = &separation.trays;
        assert_eq!(trays.len(), 12);
        assert_eq!(trays[0].pressure, Pascal(1.8e5));
        for pair in trays.windows(2) {
            let rise = pair[1].pressure.value() - pair[0].pressure.value();
            assert!(
                (rise - pair[0].fed_back_drop.value()).abs() < DROP_TOLERANCE.value(),
                "{pair:?}"
            );
            assert!(rise > 0.0, "{pair:?}");
        }
        let last = trays[11];
        assert_eq!(
            separation.stage_pressure(13),
            Some(Pascal(last.pressure.value() + last.fed_back_drop.value()))
        );
        assert_eq!(separation.stage_pressure(14), None);
        assert_eq!(separation.stage_pressure(0), None);
        let pressures: Vec<Pascal> = (1..=13)
            .map(|stage| separation.stage_pressure(stage).unwrap())
            .collect();
        let profile = separation.profile.as_ref().unwrap();
        let at_stage = worst_summation(&unit, profile, |j| pressures[j]);
        let at_top = worst_summation(&unit, profile, |_| Pascal(1.8e5));
        // Measured 4.7e-7 and 7.3e-2; the column's span 131.5 mbar on the
        // fixture's sections (§78.10; on M73.2's, 4.1e-7, 6.8e-2 and
        // 122.3 mbar, the probe's 122.1 on the scenario's thermodynamics).
        assert!(at_stage < 1e-6, "{at_stage}");
        assert!(at_top > 4e-2, "{at_top}");
        let span = pressures[12].value() - 1.8e5;
        assert!((1.30e4..1.36e4).contains(&span), "{span} Pa");
        // The feed's flash share, at its stage's pressure (stage 11).
        let feed = MoleFractions::from_mass(&unit.feed, &unit.slate).unwrap();
        let thermo = TroutonThermo::new();
        let share_at = |pressure: Pascal| {
            let bubble = bubble_temperature_unnormalized(
                &unit.slate,
                &thermo,
                feed.fractions(),
                pressure,
                "the feed",
            )
            .unwrap();
            adiabatic_flash_share(
                &unit.slate,
                &thermo,
                &feed,
                Kelvin(355.0 + 273.15),
                bubble,
                pressure,
            )
            .unwrap()
        };
        // Measured 0.6531 at the feed stage, 0.6619 at the top (M73.2's
        // sections 0.6536; the probe's 0.654 and 0.663).
        let share = separation.feed_vapour_fraction.unwrap();
        assert_eq!(share, share_at(pressures[10]));
        assert!(share < share_at(Pascal(1.8e5)) - 5e-3, "{share}");
    }

    /// **The answer does not hang on the drops it starts from** (M73.2,
    /// §78.8 finding 4, the seed contract of `ColumnPass::seed`): from cold,
    /// from its own settled profile, from that profile with its drops taken
    /// away and with every drop 200 mbar, the column lands on one answer —
    /// within the re-solve's tolerance (the probe: 0.11 Pa a tray, 0.0003 K
    /// at the bottom).
    #[test]
    fn the_answer_does_not_hang_on_the_drops_it_starts_from() {
        let (unit, spec) = thirteen_trays_with_trays(4.0);
        let model = HeatBalanceCascade::new();
        let cold = unit.separate(&model, &spec, None).unwrap();
        let settled = cold.profile.clone().unwrap();
        let no_drops = CascadeProfile {
            tray_drops: Vec::new(),
            ..settled.clone()
        };
        let absurd = CascadeProfile {
            tray_drops: vec![Pascal(2.0e4); 12],
            ..settled.clone()
        };
        for (name, seed) in [
            ("settled", &settled),
            ("no drops", &no_drops),
            ("absurd drops", &absurd),
        ] {
            let warm = unit.separate(&model, &spec, Some(seed)).unwrap();
            let pressure = warm
                .trays
                .iter()
                .zip(&cold.trays)
                .map(|(a, b)| (a.pressure.value() - b.pressure.value()).abs())
                .fold(0.0, f64::max);
            let temperature = warm
                .profile
                .as_ref()
                .unwrap()
                .temperatures
                .iter()
                .zip(&settled.temperatures)
                .map(|(a, b)| (a.value() - b.value()).abs())
                .fold(0.0, f64::max);
            // Measured 0.67 Pa and 3.3e-4 K from the settled profile, 0.61 Pa
            // and 3.0e-4 K from the absurd drops, 5e-10 from none (M73.2's
            // sections: 0.61, 0.56 Pa).
            assert!(pressure < 1.0, "{name}: {pressure} Pa");
            assert!(temperature < 5e-3, "{name}: {temperature} K");
        }
    }

    /// **Past flood the drop fed back is capped, and the trays still read
    /// flooded** (M73.2, the user's decision, §78.8): at a reflux of 15,
    /// walked from 4, the flooded trays feed back less than they read and
    /// every other tray exactly what it reads; the stages' pressures rise by
    /// the capped drops, so the column's span falls short of the raw drops'
    /// sum.
    #[test]
    fn past_flood_the_drop_fed_back_is_capped() {
        let model = HeatBalanceCascade::new();
        let (unit, spec) = thirteen_trays_with_trays(4.0);
        let at_four = unit.separate(&model, &spec, None).unwrap();
        let (_, high) = thirteen_trays_with_trays(15.0);
        let flooded = unit
            .separate(&model, &high, at_four.profile.as_ref())
            .unwrap();
        let mut past = 0;
        for tray in &flooded.trays {
            let jet = tray.flood_fraction > 1.0;
            if jet {
                past += 1;
                assert!(tray.fed_back_drop < tray.pressure_drop, "{tray:?}");
            } else {
                assert_eq!(tray.fed_back_drop, tray.pressure_drop, "{tray:?}");
            }
        }
        let raw: f64 = flooded.trays.iter().map(|t| t.pressure_drop.value()).sum();
        let span = flooded.stage_pressure(13).unwrap().value() - 1.8e5;
        // Measured: all twelve past jet flood, the span 184.9 mbar against
        // the raw drops' 420 (M73.2's sections: 169.5 and 375; the probe:
        // 150–170 capped, 360 raw).
        assert!(past >= 10, "{past}");
        assert!((1.7e4..2.0e4).contains(&span), "{span} Pa");
        assert!(raw > 3.0e4, "{raw} Pa");
    }

    /// **On a loop column a short tray's product is cut back, and the set
    /// settles** (M72.3, docs/DESIGN.md §77.13): at a reflux of 0.05 the
    /// gas oil is cut back, and the gas-oil loop turned up — which wets its
    /// tray — lets it take more, its tray held at the minimum, then turned back
    /// down. Before M72.3 a released draw was cut again by the next start, the
    /// passes ran out at 80 kg/s and the uncut answer stood, the gas-oil tray
    /// at 2 mol/s against a minimum of 24; back down at 75 the column refused.
    #[test]
    fn a_short_trays_product_is_cut_back_on_a_loop_column() {
        let (unit, mut spec) = thirteen_trays();
        let model = HeatBalanceCascade::new();
        let mut last = unit.separate(&model, &spec, None).unwrap();
        for reflux in [2.0, 1.0, 0.5, 0.3, 0.2, 0.1, 0.05] {
            spec.reflux_ratio = reflux;
            last = unit
                .separate(&model, &spec, last.profile.as_ref())
                .unwrap_or_else(|e| panic!("reflux {reflux}: {e}"));
        }
        let mut held = Vec::new();
        let mut shares = Vec::new();
        for flow in [70.0, 75.0, 80.0, 75.0] {
            spec.loops[2].flow = KgPerSec(flow);
            last = unit
                .separate(&model, &spec, last.profile.as_ref())
                .unwrap_or_else(|e| panic!("the gas-oil loop at {flow} kg/s: {e}"));
            assert_eq!(last.cut_back.len(), 1, "{flow} kg/s: {:?}", last.cut_back);
            assert_eq!(last.cut_back[0].draw, 3, "{flow} kg/s: the gas oil");
            held.push(last.stage_liquid[9]);
            shares.push(last.cut_back[0].of_declared);
        }
        // The gas-oil tray held at one minimum throughout, the share rising
        // with the loop and back.
        for liquid in &held {
            assert!((liquid / held[0] - 1.0).abs() < 1e-6, "{held:?}");
        }
        assert!(shares[0] < shares[1] && shares[1] < shares[2], "{shares:?}");
        assert!((shares[3] / shares[1] - 1.0).abs() < 1e-6, "{shares:?}");
    }

    /// **The hand-over ends on the column's own equations** (§77.5): the cold
    /// start's answer is a solution of the column with its returns held — Newton
    /// warm from it lands where it stands.
    #[test]
    fn the_hand_over_ends_on_the_columns_own_equations() {
        let (unit, spec) = thirteen_trays();
        let cold = unit
            .separate(&HeatBalanceCascade::new(), &spec, None)
            .unwrap();
        let warm = unit
            .separate(&HeatBalanceCascade::new(), &spec, cold.profile.as_ref())
            .unwrap();
        let gap = gap(&cold, &warm);
        assert!(
            gap < 1e-6,
            "the cold start and Newton warm from it differ by {gap}"
        );
        for (a, b) in cold.loops.iter().zip(&warm.loops) {
            assert!(
                (a.duty.value() / b.duty.value() - 1.0).abs() < 1e-6,
                "{a:?} {b:?}"
            );
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

    /// **A draw whose tray would run dry is cut back, and released when the
    /// column recovers** (M67.1, §72.1). At a reflux of 0.6 the kerosene
    /// draw (stage 3) would take all the liquid the reflux sends it — refused
    /// by name until M66 (§66.2). Now it takes what leaves its tray the
    /// minimum: cut back, reported, its split its declared share times the
    /// share it ran at, and the bottoms the rest. Back at a reflux of 4 from
    /// that answer, the carried cut is released and the column is the one
    /// that never cut, to Newton's tolerance.
    /// **A column with a draw cut back is insensitive to its start** (M67.1):
    /// at a reflux of 0.6 the answer from the uncut column at 4 and from the
    /// cut-back answer itself (its cut carried) is one answer, to Newton's
    /// tolerance — the carried cut is a hint. Started cold it is refused, as
    /// before M67: the cold warm-up runs the constant-molar-overflow cascade at
    /// the declared shares, which has no cut-back (§72.4, deferred).
    #[test]
    fn a_cut_back_column_is_start_insensitive() {
        let unit = CrudeUnit::new();
        let low = CrudeUnit::spec(0.6, 1.0);
        let running = unit
            .separate(&HeatBalanceCascade::new(), &CrudeUnit::spec(4.0, 1.0), None)
            .unwrap();
        let from_running = unit
            .separate(&HeatBalanceCascade::new(), &low, running.profile.as_ref())
            .unwrap();
        let from_itself = unit
            .separate(
                &HeatBalanceCascade::new(),
                &low,
                from_running.profile.as_ref(),
            )
            .unwrap();
        assert_eq!(from_itself.cut_back.len(), 1, "{:?}", from_itself.cut_back);
        let gap = gap(&from_itself, &from_running);
        assert!(gap < 1e-6, "the starts differ by {gap}");
        let cold = unit
            .separate(&HeatBalanceCascade::new(), &low, None)
            .expect_err("no cut-back in the cold warm-up")
            .to_string();
        assert!(cold.contains("could not start"), "{cold}");
    }

    #[test]
    fn a_dry_trays_draw_is_cut_back_and_released() {
        let unit = CrudeUnit::new();
        let running = unit
            .separate(&HeatBalanceCascade::new(), &CrudeUnit::spec(4.0, 1.0), None)
            .unwrap();
        assert!(running.cut_back.is_empty(), "{:?}", running.cut_back);
        let short = unit
            .separate(
                &HeatBalanceCascade::new(),
                &CrudeUnit::spec(0.6, 1.0),
                running.profile.as_ref(),
            )
            .unwrap_or_else(|e| panic!("reflux 0.6: {e}"));
        assert_eq!(short.cut_back.len(), 1, "{:?}", short.cut_back);
        let kerosene = short.cut_back[0];
        assert_eq!(kerosene.draw, 1);
        assert!(
            kerosene.of_declared > 0.0 && kerosene.of_declared < 1.0,
            "{kerosene:?}"
        );
        assert_eq!(
            short.profile.as_ref().unwrap().cut_back,
            vec![1],
            "the cut is carried"
        );
        let declared = running.draws[1].split;
        assert!((short.draws[1].split - declared * kerosene.of_declared).abs() < 1e-12);
        let total: f64 = short.draws.iter().map(|d| d.split).sum();
        assert!((total - 1.0).abs() < 1e-12, "the splits sum to {total}");

        let recovered = unit
            .separate(
                &HeatBalanceCascade::new(),
                &CrudeUnit::spec(4.0, 1.0),
                short.profile.as_ref(),
            )
            .unwrap();
        assert!(recovered.cut_back.is_empty(), "{:?}", recovered.cut_back);
        let back = gap(&recovered, &running);
        assert!(back < 1e-6, "released, the column differs by {back}");

        // A step back up to a reflux of 1.1, where the kerosene's tray fills
        // its share again: the carried cut converges with the draw taking more
        // than its share, and is released — the answer the column gives from
        // the uncut start. (The released draw starts at the rate it ran at,
        // its tray ON the minimum: measured, a start that cut it again on
        // rounding never released it here.)
        let one = CrudeUnit::spec(1.1, 1.0);
        let released = unit
            .separate(&HeatBalanceCascade::new(), &one, short.profile.as_ref())
            .unwrap_or_else(|e| panic!("back to a reflux of 1.1: {e}"));
        assert!(released.cut_back.is_empty(), "{:?}", released.cut_back);
        let uncut = unit
            .separate(&HeatBalanceCascade::new(), &one, running.profile.as_ref())
            .unwrap();
        let apart = gap(&released, &uncut);
        assert!(
            apart < 1e-6,
            "released at 1.1, the column differs by {apart}"
        );
    }

    /// A thermo model's K-values, counted: a solve's cost in the residual's own
    /// unit, which wall time cannot gate (it moves with the machine and the
    /// profile).
    struct Counting<M> {
        inner: M,
        k_values: std::cell::Cell<u64>,
    }

    impl<M: ThermoModel> Counting<M> {
        fn new(inner: M) -> Self {
            Self {
                inner,
                k_values: std::cell::Cell::new(0),
            }
        }

        /// The K-values `run` evaluated.
        fn count<T>(&self, run: impl FnOnce() -> T) -> (T, u64) {
            let before = self.k_values.get();
            let out = run();
            (out, self.k_values.get() - before)
        }
    }

    impl<M: ThermoModel> ThermoModel for Counting<M> {
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
            composition: &Composition,
            temperature: Kelvin,
        ) -> Result<Pascal, SimError> {
            self.inner.bubble_pressure(slate, composition, temperature)
        }
    }

    /// The K-values of two key presses on a column settled before each: the
    /// reflux lowered 4 → 2, and raised 2 → 4.
    fn lowered_and_raised(unit: &CrudeUnit, spec: &CascadeSpec) -> (u64, u64) {
        let model = HeatBalanceCascade::new();
        let thermo = Counting::new(TroutonThermo::new());
        let at = |reflux_ratio: f64| CascadeSpec {
            reflux_ratio,
            ..spec.clone()
        };
        let settled = |reflux: f64| {
            let spec = at(reflux);
            let mut run = unit.separate_on(&model, &spec, None, &thermo).unwrap();
            for _ in 0..3 {
                run = unit
                    .separate_on(&model, &spec, run.profile.as_ref(), &thermo)
                    .unwrap();
            }
            run
        };
        let press = |from: f64, to: f64| {
            let start = settled(from);
            let (run, cost) = thermo.count(|| {
                unit.separate_on(&model, &at(to), start.profile.as_ref(), &thermo)
                    .unwrap()
            });
            assert_eq!(run.reflux_ratio, Some(to), "the floor does not bind");
            cost
        };
        (press(4.0, 2.0), press(2.0, 4.0))
    }

    /// **A lowered reflux solves the declared reflux first** (M72.5, C11,
    /// docs/DESIGN.md §77.14): warm from the column at 4, the start runs above
    /// the declared 2 because the player lowered it, not because the floor
    /// bound. The floor-first solve has no answer there; it ground to a
    /// failure before the declared solve converged in 4 Newton steps, so the
    /// reflux lowered 4 → 2 cost 4.3 times the K-values of the reflux raised
    /// 2 → 4 on the ten-tray crude unit (90 450 against 21 048) and 9.3 times
    /// on the 13-tray loop column (265 392 against 28 572; the probe's
    /// `tools/steptime/prototypes/m72_5/FINDINGS.md`). Declared first, 1.07
    /// and 1.01 times: the two key presses one cost.
    #[test]
    fn a_lowered_reflux_solves_the_declared_reflux_first() {
        let unit = CrudeUnit::new();
        let ten = CrudeUnit::spec(4.0, 1.0);
        let (thirteen_unit, thirteen) = thirteen_trays();
        for (label, unit, spec) in [
            ("ten trays", &unit, &ten),
            ("thirteen", &thirteen_unit, &thirteen),
        ] {
            let (lowered, raised) = lowered_and_raised(unit, spec);
            assert!(
                lowered <= 2 * raised,
                "{label}: the reflux lowered 4 → 2 spent {lowered} K-values, raised 2 → 4 \
                 {raised}"
            );
        }
    }

    /// **A reflux lowered onto the floor lands on it** (M72.5, §77.14): a feed
    /// 60 K past its bubble point, running free of the floor at a reflux of 2,
    /// lowered to 0.5 — the start above the declared reflux and off the floor,
    /// so the declared reflux is solved first, leaves a stage under the floor,
    /// and the floor is solved after it: the column the floor holds at a reflux
    /// of 0.79, as from cold.
    #[test]
    fn a_reflux_lowered_onto_the_floor_lands_on_it() {
        let slate = three_cut_slate();
        let thermo = Counting::new(
            ConstantAlphaThermo::with_temperature_exponent(
                &slate,
                vec![3.0, 1.0, 0.25],
                Kelvin(400.0),
                10.0,
            )
            .unwrap()
            .with_dh_vap(&slate, vec![30_000.0, 36_000.0, 45_000.0])
            .unwrap(),
        );
        let feed = Composition::from_weights(&[0.3, 0.3, 0.4]).unwrap();
        let moles = MoleFractions::from_mass(&feed, &slate).unwrap();
        let bubble =
            bubble_temperature_unnormalized(&slate, &thermo, moles.fractions(), P_ATM, "feed")
                .unwrap();
        let draws = vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
            ColumnDraw::by_stage(NodeId(2), 8, None),
        ];
        let at = |reflux_ratio: f64, seed: Option<&CascadeProfile>| {
            let spec = CascadeSpec {
                stages: 8,
                feed_stage: 4,
                reflux_ratio,
                pumparounds: Vec::new(),
                steam: None,
                reboiler_duty: None,
                loops: Vec::new(),
                trays: Vec::new(),
                holdup: None,
            };
            HeatBalanceCascade::new()
                .separate(
                    &ColumnPass {
                        slate: &slate,
                        draws: &draws,
                        smearing: Kelvin(0.0),
                        pressure: P_ATM,
                        feed: &feed,
                        feed_flow: KgPerSec(10.0),
                        temperature: Kelvin(bubble.value() + 60.0),
                        cascade: Some(&spec),
                        seed,
                        running_reboiler: None,
                        earlier_pass: None,
                        steam: None,
                    },
                    &thermo,
                )
                .unwrap()
        };
        let free = at(2.0, None);
        assert_eq!(free.reflux_ratio, Some(2.0), "the floor does not bind at 2");
        let lowered = at(0.5, free.profile.as_ref());
        assert!(
            lowered.reflux_ratio.unwrap() > 0.7,
            "{:?}",
            lowered.reflux_ratio
        );
        let cold = at(0.5, None);
        let apart = gap(&lowered, &cold);
        assert!(apart < 1e-6, "lowered and cold differ by {apart}");

        // Held there, the next tick solves the floor first: a floored column's
        // settled tick costs what a free one's does.
        let free_settled = at(2.0, free.profile.as_ref());
        let (_, free_tick) = thermo.count(|| at(2.0, free_settled.profile.as_ref()));
        let (held, floored_tick) = thermo.count(|| at(0.5, lowered.profile.as_ref()));
        let drift = gap(&held, &lowered);
        assert!(drift < 1e-6, "held at the floor, the column moved {drift}");
        assert!(
            floored_tick <= 2 * free_tick,
            "a settled tick held at the floor spent {floored_tick} K-values, a free one \
             {free_tick}"
        );
    }
}

#[cfg(test)]
mod steam_tests {
    use super::*;
    use crate::TroutonThermo;
    use refinery_core::components::{Phase, PseudoComponent};
    use refinery_core::graph::{ColumnDraw, NodeId};
    use refinery_core::traits::ColumnSteam;
    use refinery_core::units::{JPerKgK, JPerMol, KgPerM3, KgPerMol, KgPerSec};

    /// `reference/steam_column.rs`'s slate: four cuts and free water.
    fn slate() -> Slate {
        let cut = |name: &str, tb_c: f64, molar_mass: f64, cp: f64| PseudoComponent {
            name: name.into(),
            tb: Kelvin(tb_c + 273.15),
            molar_mass: KgPerMol(molar_mass),
            density: Some(KgPerM3(800.0)),
            cp: JPerKgK(cp),
            cp_shape: None,
            phase: Phase::Liquid,
            dh_vap: None,
            cp_vapour: None,
            free_water: false,
        };
        Slate::new(vec![
            cut("light", 70.0, 0.095, 2200.0),
            cut("naphtha", 150.0, 0.13, 2100.0),
            cut("kerosene", 220.0, 0.17, 2050.0),
            cut("diesel", 300.0, 0.22, 2000.0),
            PseudoComponent {
                name: "water".into(),
                tb: Kelvin(373.15),
                molar_mass: KgPerMol(0.018015),
                density: Some(KgPerM3(998.0)),
                cp: JPerKgK(4184.0),
                cp_shape: None,
                phase: Phase::Liquid,
                dh_vap: Some(JPerMol(40_650.0)),
                cp_vapour: Some(JPerKgK(2000.0)),
                free_water: true,
            },
        ])
        .unwrap()
    }

    /// **Newton lands where the fixed point does with steam in the column**
    /// (M62.6, docs/DESIGN.md §67.18): the fixed point is Newton's cold
    /// warm-up, and solved to convergence on its own it is the reference
    /// Newton is gated against — so the steam must be in its stage balances
    /// too, on a stripping tray and on the reboiler.
    #[test]
    fn newton_lands_where_the_fixed_point_does_under_steam() {
        let slate = slate();
        let thermo = TroutonThermo::new();
        let feed = Composition::from_weights(&[0.198, 0.297, 0.297, 0.198, 0.01]).unwrap();
        let draws = vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
            ColumnDraw::water_draw(NodeId(2)),
            ColumnDraw::by_stage(NodeId(3), 3, Some(0.25)),
            ColumnDraw::by_stage(NodeId(4), 8, None),
        ];
        let spec = CascadeSpec {
            stages: 8,
            feed_stage: 5,
            reflux_ratio: 2.0,
            pumparounds: Vec::new(),
            steam: None,
            reboiler_duty: None,
            loops: Vec::new(),
            trays: Vec::new(),
            holdup: None,
        };
        // 0.15 kg/s of steam at 400 °C, as its liquid-equivalent temperature.
        let latent = 40_650.0 / 0.018015 + (2000.0 - 4184.0) * 300.0;
        let steam_le = Kelvin(273.15 + 400.0 + latent / 4184.0);
        for stage in [7, 8] {
            let pass = ColumnPass {
                slate: &slate,
                draws: &draws,
                smearing: Kelvin(0.0),
                pressure: Pascal(1.8e5),
                feed: &feed,
                feed_flow: KgPerSec(10.0),
                temperature: Kelvin(273.15 + 230.0),
                cascade: Some(&spec),
                seed: None,
                running_reboiler: None,
                earlier_pass: None,
                steam: Some(ColumnSteam {
                    stage,
                    flow: KgPerSec(0.15),
                    temperature: steam_le,
                }),
            };
            let newton = HeatBalanceCascade::new().separate(&pass, &thermo).unwrap();
            let fixed = HeatBalanceCascade::fixed_point()
                .separate(&pass, &thermo)
                .unwrap_or_else(|e| panic!("stage {stage}: the fixed point: {e}"));
            let (a, b) = (newton.profile.unwrap(), fixed.profile.unwrap());
            for (x, y) in a.temperatures.iter().zip(&b.temperatures) {
                assert!(
                    (x.value() - y.value()).abs() < 1e-4,
                    "stage {stage}: Newton {x:?}, the fixed point {y:?}"
                );
            }
            for (x, y) in newton.draws.iter().zip(&fixed.draws) {
                assert!((x.split - y.split).abs() < 1e-6, "stage {stage}");
            }
        }
    }
}

/// Pumparound loops (M72.1, docs/DESIGN.md §77.1): liquid drawn off a stage,
/// cooled to a held temperature and returned one stage up.
#[cfg(test)]
mod loop_tests {
    use super::*;
    use crate::ConstantAlphaThermo;
    use refinery_core::components::{Phase, PseudoComponent};
    use refinery_core::graph::{ColumnDraw, NodeId, PumparoundLoop, TraySection};
    use refinery_core::units::{JPerKgK, KgPerM3, KgPerMol, KgPerSec, Meter, P_ATM};

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

    fn spec(loops: Vec<PumparoundLoop>) -> CascadeSpec {
        CascadeSpec {
            stages: 8,
            feed_stage: 4,
            reflux_ratio: 2.0,
            pumparounds: Vec::new(),
            steam: None,
            reboiler_duty: None,
            loops,
            trays: Vec::new(),
            holdup: None,
        }
    }

    fn draws() -> Vec<ColumnDraw> {
        vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
            ColumnDraw::by_stage(NodeId(2), 8, None),
        ]
    }

    fn feed() -> Composition {
        Composition::from_weights(&[0.3, 0.3, 0.4]).unwrap()
    }

    fn separate(model: &dyn SeparationModel, spec: &CascadeSpec) -> Result<Separation, SimError> {
        let slate = slate();
        let thermo = thermo(&slate);
        let feed = feed();
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
                running_reboiler: None,
                earlier_pass: None,
                steam: None,
            },
            &thermo,
        )
    }

    /// The loop the gates run: drawn off stage 3, returned to stage 2, 3 kg/s
    /// (about a third of the feed's moles) returned 20 K under stage 3's
    /// temperature without it.
    fn the_loop(capacity: f64) -> PumparoundLoop {
        let bare = separate(&HeatBalanceCascade::new(), &spec(Vec::new())).unwrap();
        let stage_three = bare.profile.unwrap().temperatures[2].value();
        PumparoundLoop {
            draw_stage: 3,
            return_stage: 2,
            flow: KgPerSec(3.0),
            return_temperature: Kelvin(stage_three - 20.0),
            cooler_capacity: Watt(capacity),
            start_duty: Watt(0.0),
            line: Vec::new(),
            cooler: None,
        }
    }

    /// A stage's liquid and vapour molar enthalpies [J/mol] on a solved
    /// profile, by hand from its temperature and liquid.
    fn enthalpies(separation: &Separation) -> (Vec<f64>, Vec<f64>) {
        let slate = slate();
        let thermo = thermo(&slate);
        let profile = separation.profile.as_ref().unwrap();
        let t: Vec<f64> = profile.temperatures.iter().map(|t| t.value()).collect();
        let k = stage_k_values_profile(&slate, &thermo, &t, &vec![P_ATM; t.len()]).unwrap();
        let mut liquid = Vec::new();
        let mut vapour = Vec::new();
        for j in 0..t.len() {
            let x = &profile.liquid[j];
            let y = equilibrium_vapour(x, &k[j]).unwrap();
            liquid.push(liquid_enthalpy(&slate, x, Kelvin(t[j])));
            vapour.push(
                liquid_enthalpy(&slate, &y, Kelvin(t[j]))
                    + mixture_dh_vap(&slate, &thermo, &y, Kelvin(t[j])).unwrap(),
            );
        }
        (liquid, vapour)
    }

    /// **A loop returns at its cooler's setpoint, at its declared flow, and
    /// its duty is the result**: `P·(h_d − h_ret)` by hand from the draw
    /// stage's liquid, `P` the flow over that liquid's molar mass.
    #[test]
    fn a_loop_returns_at_its_setpoint_and_its_duty_is_the_result() {
        let declared = the_loop(10.0e6);
        let separation =
            separate(&HeatBalanceCascade::new(), &spec(vec![declared.clone()])).unwrap();
        let run = separation.loops[0].clone();
        assert!(!run.at_capacity);
        assert!(
            (run.return_temperature.value() - declared.return_temperature.value()).abs() < 1e-9,
            "{run:?}"
        );
        assert!((run.flow.value() - 3.0).abs() < 1e-6, "{run:?}");
        let slate = slate();
        let profile = separation.profile.as_ref().unwrap();
        let x = &profile.liquid[2];
        let molar = 3.0 / crate::cascade::molar_mass(&slate, x);
        let by_hand = molar
            * (liquid_enthalpy(&slate, x, profile.temperatures[2])
                - liquid_enthalpy(&slate, x, declared.return_temperature));
        assert!(by_hand > 0.0);
        assert!(
            (run.duty.value() / by_hand - 1.0).abs() < 1e-6,
            "the loop's duty {} W against {by_hand} W by hand",
            run.duty.value()
        );
        assert_eq!(separation.stage_liquid.len(), 8);
    }

    /// **The stages a loop joins balance with it, by hand** from what the
    /// column publishes: the return stage takes the circulation in at
    /// `h_ret` and sends it down with its own liquid; the draw stage sends it
    /// out at its own enthalpy.
    #[test]
    fn the_stages_a_loop_joins_balance_by_hand() {
        let declared = the_loop(10.0e6);
        let separation =
            separate(&HeatBalanceCascade::new(), &spec(vec![declared.clone()])).unwrap();
        let slate = slate();
        let profile = separation.profile.as_ref().unwrap();
        let (h, big_h) = enthalpies(&separation);
        let v = &profile.vapour;
        let l = &separation.stage_liquid;
        let x = &profile.liquid[2];
        let p = 3.0 / crate::cascade::molar_mass(&slate, x);
        let h_ret = liquid_enthalpy(&slate, x, declared.return_temperature);
        // Stage 2 (index 1): the stage-1 liquid and the stage-3 vapour in, the
        // circulation returned; its liquid and vapour out.
        let material = l[0] + v[2] + p - l[1] - v[1];
        assert!(
            material.abs() < 1e-9 * v[1],
            "stage 2's moles off by {material}"
        );
        let scale = v[1] * big_h[1];
        let energy = l[0] * h[0] + v[2] * big_h[2] + p * h_ret - l[1] * h[1] - v[1] * big_h[1];
        assert!(
            energy.abs() < 1e-6 * scale,
            "stage 2's energy off by {energy} W"
        );
        // Stage 3 (index 2): the circulation leaves with its own liquid.
        let material = l[1] + v[3] - p - l[2] - v[2];
        assert!(
            material.abs() < 1e-9 * v[2],
            "stage 3's moles off by {material}"
        );
        let energy = l[1] * h[1] + v[3] * big_h[3] - (l[2] + p) * h[2] - v[2] * big_h[2];
        assert!(
            energy.abs() < 1e-6 * scale,
            "stage 3's energy off by {energy} W"
        );
        // The trays between carry the circulation: stage 2 sends down more
        // than stage 1 does by more than the loop's own flow less its vapour.
        assert!(
            l[1] > p,
            "stage 2's liquid {} under the circulation {p}",
            l[1]
        );
    }

    /// **The closure is the reboiler stage's own balance with a loop in the
    /// column**: the closure counts the loop's duty by formula; only stage
    /// balances that take its liquid out and back agree with it.
    #[test]
    fn with_a_loop_the_closure_is_still_the_reboiler_stage() {
        let separation =
            separate(&HeatBalanceCascade::new(), &spec(vec![the_loop(10.0e6)])).unwrap();
        let profile = separation.profile.as_ref().unwrap();
        let (h, big_h) = enthalpies(&separation);
        let n = 7;
        let into = separation.stage_liquid[n - 1];
        let bottoms = into - profile.vapour[n];
        let stage = profile.vapour[n] * big_h[n] + bottoms * h[n] - into * h[n - 1];
        let closure = separation.reboiler_duty.unwrap().value();
        assert!(
            (closure - stage).abs() <= 1e-6 * closure,
            "closure {closure} W against the stage's {stage} W"
        );
    }

    /// **Newton lands where the fixed point does** with a loop's circulation
    /// among its unknowns.
    /// **With trays, Newton and the fixed point land on one column** (M73.2,
    /// docs/DESIGN.md §78.9): the fixed point's K-values and bubble points
    /// read each stage's pressure as Newton's do, so the two land on the same
    /// stage pressures, to the re-solve's tolerance, and the same stage
    /// temperatures — and not on the column without trays. On Trouton's
    /// K-values, which fall with the pressure: this module's constant-alpha
    /// ones do not read it.
    #[test]
    fn with_trays_newton_lands_where_the_fixed_point_does() {
        let bare = spec(vec![the_loop(10.0e6)]);
        let trayed = CascadeSpec {
            trays: vec![TraySection {
                from: 1,
                to: 7,
                diameter: Meter(1.2),
                spacing: Meter(0.61),
                weir_height: Meter(0.05),
                hole_diameter: Meter(0.005),
                hole_area_share: 0.10,
                downcomer_share: 0.12,
                clearance: Meter(0.04),
                passes: 1,
            }],
            ..bare.clone()
        };
        let separate = |model: &HeatBalanceCascade, spec: &CascadeSpec| {
            let slate = slate();
            let feed = feed();
            let draws = draws();
            model
                .separate(
                    &ColumnPass {
                        slate: &slate,
                        draws: &draws,
                        smearing: Kelvin(0.0),
                        pressure: P_ATM,
                        feed: &feed,
                        feed_flow: KgPerSec(10.0),
                        temperature: Kelvin(400.0),
                        cascade: Some(spec),
                        seed: None,
                        running_reboiler: None,
                        earlier_pass: None,
                        steam: None,
                    },
                    &crate::TroutonThermo::new(),
                )
                .unwrap()
        };
        let newton = separate(&HeatBalanceCascade::new(), &trayed);
        let fixed = separate(&HeatBalanceCascade::fixed_point(), &trayed);
        let plain = separate(&HeatBalanceCascade::new(), &bare);
        let gap = |a: &Separation, b: &Separation| {
            let (a, b) = (a.profile.as_ref().unwrap(), b.profile.as_ref().unwrap());
            a.temperatures
                .iter()
                .zip(&b.temperatures)
                .map(|(p, q)| (p.value() - q.value()).abs())
                .fold(0.0, f64::max)
        };
        let pressure = newton
            .trays
            .iter()
            .zip(&fixed.trays)
            .map(|(a, b)| (a.pressure.value() - b.pressure.value()).abs())
            .fold(0.0, f64::max);
        let span = newton.stage_pressure(8).unwrap().value() - P_ATM.value();
        // Measured: the span 88.5 mbar; the two solves 1.1e-3 K and 0.05 Pa
        // apart (7.1e-3 K without trays, their own tolerances); the trays
        // move a stage 3.58 K.
        assert!(span > 1.0e3, "{span} Pa");
        assert!(pressure < 1.0, "{pressure} Pa");
        let solves = gap(&newton, &fixed);
        assert!(solves < 1e-2, "{solves} K");
        let moved = gap(&newton, &plain);
        assert!(moved > 1.0, "{moved} K");
    }

    #[test]
    fn newton_lands_where_the_fixed_point_does_with_a_loop() {
        let spec = spec(vec![the_loop(10.0e6)]);
        let newton = separate(&HeatBalanceCascade::new(), &spec).unwrap();
        let fixed = separate(&HeatBalanceCascade::fixed_point(), &spec).unwrap();
        let mut worst = 0.0f64;
        for (a, b) in newton.draws.iter().zip(&fixed.draws) {
            for (p, q) in a
                .composition
                .fractions()
                .iter()
                .zip(b.composition.fractions())
            {
                worst = worst.max((p - q).abs());
            }
        }
        let relative = |p: f64, q: f64| (p - q).abs() / p.abs().max(q.abs());
        worst = worst.max(relative(
            newton.reboiler_duty.unwrap().value(),
            fixed.reboiler_duty.unwrap().value(),
        ));
        assert!(worst < 1e-6, "Newton and the fixed point differ by {worst}");
        // The loop's duty is a temperature DIFFERENCE times a heat capacity, so
        // it magnifies the two solves' agreement on the draw stage's
        // temperature by `T_d/(T_d − T_ret)` — about 18 here.
        let draw = newton.profile.as_ref().unwrap().temperatures[2].value();
        let magnified = draw / (draw - spec.loops[0].return_temperature.value());
        let duty = relative(newton.loops[0].duty.value(), fixed.loops[0].duty.value());
        assert!(
            duty < 1e-6 * magnified,
            "the loop's duty differs by {duty}, against {} magnified",
            1e-6 * magnified
        );
    }

    /// **Past its capacity the cooler takes out its capacity and the return
    /// warms** (§77.3): asked for twice what it can do, the loop's duty is
    /// the capacity and it returns above its setpoint.
    #[test]
    fn past_its_capacity_the_return_warms() {
        let free = separate(&HeatBalanceCascade::new(), &spec(vec![the_loop(10.0e6)])).unwrap();
        let capacity = 0.5 * free.loops[0].duty.value();
        let declared = the_loop(capacity);
        let held = separate(&HeatBalanceCascade::new(), &spec(vec![declared.clone()])).unwrap();
        let run = held.loops[0].clone();
        assert!(run.at_capacity, "{run:?}");
        assert!((run.duty.value() / capacity - 1.0).abs() < 1e-9, "{run:?}");
        assert!(
            run.return_temperature.value() > declared.return_temperature.value() + 1.0,
            "{run:?}"
        );
    }

    /// **A loop at zero flow is no loop**: the column answers exactly as it
    /// does without one, and reports the loop idle.
    #[test]
    fn a_loop_at_zero_flow_is_no_loop() {
        let bare = separate(&HeatBalanceCascade::new(), &spec(Vec::new())).unwrap();
        let idle_loop = PumparoundLoop {
            flow: KgPerSec(0.0),
            ..the_loop(10.0e6)
        };
        let idle = separate(&HeatBalanceCascade::new(), &spec(vec![idle_loop])).unwrap();
        for (a, b) in bare.draws.iter().zip(&idle.draws) {
            assert_eq!(a.composition.fractions(), b.composition.fractions());
            assert_eq!(a.split, b.split);
        }
        assert_eq!(bare.reboiler_duty, idle.reboiler_duty);
        assert_eq!(idle.loops[0].flow.value(), 0.0);
        assert_eq!(idle.loops[0].duty.value(), 0.0);
        assert!(bare.loops.is_empty() && bare.stage_liquid.is_empty());
    }

    /// Where a loop may not go, and what it may not be — refused by the plan.
    #[test]
    fn a_misplaced_loop_is_refused() {
        let good = PumparoundLoop {
            draw_stage: 3,
            return_stage: 2,
            flow: KgPerSec(3.0),
            return_temperature: Kelvin(380.0),
            cooler_capacity: Watt(1.0e6),
            start_duty: Watt(0.0),
            line: Vec::new(),
            cooler: None,
        };
        let refusal = |loops: Vec<PumparoundLoop>, pumparounds: Vec<Pumparound>| {
            let spec = CascadeSpec {
                pumparounds,
                ..spec(loops)
            };
            match CascadePlan::new(&spec, &draws(), 3) {
                Ok(_) => panic!("{spec:?} was admitted"),
                Err(e) => e.to_string(),
            }
        };
        let cases = [
            (
                vec![PumparoundLoop {
                    draw_stage: 1,
                    return_stage: 0,
                    ..good.clone()
                }],
                "strictly between the top stage",
            ),
            (
                vec![PumparoundLoop {
                    draw_stage: 8,
                    return_stage: 7,
                    ..good.clone()
                }],
                "strictly between the top stage",
            ),
            (
                vec![PumparoundLoop {
                    return_stage: 1,
                    ..good.clone()
                }],
                "returns to the stage above its draw",
            ),
            (
                vec![PumparoundLoop {
                    flow: KgPerSec(-1.0),
                    ..good.clone()
                }],
                "must be finite and >= 0",
            ),
            (
                vec![PumparoundLoop {
                    cooler_capacity: Watt(0.0),
                    ..good.clone()
                }],
                "must be finite and positive",
            ),
            (
                vec![PumparoundLoop {
                    start_duty: Watt(-1.0),
                    ..good.clone()
                }],
                "starts at a duty",
            ),
            (
                vec![
                    good.clone(),
                    PumparoundLoop {
                        draw_stage: 4,
                        return_stage: 3,
                        ..good.clone()
                    },
                ],
                "stage 3 is in two pumparounds",
            ),
        ];
        for (loops, phrase) in cases {
            let message = refusal(loops, Vec::new());
            assert!(message.contains(phrase), "{message}");
        }
        let message = refusal(
            vec![good.clone()],
            vec![Pumparound {
                stage: 2,
                duty: Watt(1.0e5),
            }],
        );
        assert!(
            message.contains("stage 2 is in two pumparounds"),
            "{message}"
        );
    }
}
