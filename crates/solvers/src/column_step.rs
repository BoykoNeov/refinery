//! One tick of a column that holds liquid (M74, docs/DESIGN.md §79.2).
//!
//! The heat-balance cascade's stages, each holding the liquid its tray (or the
//! base, or the reflux drum) holds, stepped one tick from the state the last
//! tick left — implicit Euler on the compositions:
//!
//! ```text
//!   given      each stage's moles n_jc and the drum's (the state), the feed,
//!              the liquid off each tray from what it holds (explicit), the
//!              side draws and loops at their set flows, the level loops'
//!              distillate D and bottoms B (or reboiler duty Q), reflux R·D
//!   unknowns   T_j and V_j, every stage
//!   n'_jc − n_jc = dt·(in − out)_c        per component, the Thomas sweep with
//!                                         n'/dt on its diagonal, the drum a
//!                                         row above the top stage
//!   S_j = Σ_c K_jc·x'_jc − 1              every stage (x' unnormalised)
//!   E_j = Σ in·h_in − Σ out·h_out − h_j·(H'_j − H_j)/dt − R_j (+ Q at the base)
//! ```
//!
//! The liquid each stage stores is carried at its own enthalpy; the heat its
//! liquid stores as its temperature moves is left out (§79.0 finding 7). Every
//! liquid flow is set before the solve, in moles at the start-of-tick
//! compositions, so the only unknowns are the temperatures and the vapours.
//! The state the tick leaves and the products' component flows are both read
//! off the sweep's own amounts, so each component is conserved to round-off
//! whatever the Newton tolerance (the column's books: feed − products =
//! what it came to hold).

use refinery_core::components::{Composition, Slate};
use refinery_core::error::SimError;
use refinery_core::graph::{CascadeSpec, ColumnHoldup, HoldupState};
use refinery_core::traits::{HoldupReading, LevelLimit, ThermoModel};
use refinery_core::units::{Kelvin, Kg, KgPerSec, Pascal, Watt};

use crate::bubble::bubble_temperature_unnormalized;
use crate::cascade::{mixture_dh_vap, CascadePlan, DrawLocation, Flows};
use crate::column_newton::solve_dense;
use crate::heat_balance::{equilibrium_vapour, liquid_enthalpy};
use crate::molar::MoleFractions;

/// The level loops' proportional gain [-]: a share of the loop's settled flow
/// per share of its normal holdup the level stands off it. **A GAME VALUE**
/// (M74.1.0's probe, `tools/steptime/prototypes/m74_1_0/`, finding 4): with
/// [`LEVEL_INTEGRAL_S`] it holds the drum and the base within about ±20 % of
/// their normal holdups on the screen's moderate key moves under scheme A.
pub(crate) const LEVEL_GAIN: f64 = 2.0;

/// The level loops' integral time [s]. **A GAME VALUE**, with [`LEVEL_GAIN`].
pub(crate) const LEVEL_INTEGRAL_S: f64 = 300.0;

/// The level under which a drum's reflux pump loses its suction, or a base's
/// reboiler tubes uncover, as a share of its working volume [-]: below it
/// each sends out in proportion to the level left (M74, docs/DESIGN.md §79.2,
/// the user's physical limits). **A GAME VALUE**: a low-level trip sits at
/// about a tenth of a vessel's working range (Kister 1990, §15.14: the
/// portion below the low-level alarm, 2 minutes of product, about a tenth of
/// the drum the table sizes).
pub(crate) const LOW_LEVEL: f64 = 0.1;

/// The most a stage temperature may move in one Newton step [K] — a damping
/// bound, not physics (`column_newton`'s value).
const MAX_TEMPERATURE_STEP_K: f64 = 25.0;

/// The share of its distance to zero a vapour may close in one step [-].
const FRACTION_TO_BOUNDARY: f64 = 0.9;

/// Newton steps before a tick's step is refused.
const MAX_ITERATIONS: u32 = 40;

/// Step halvings before a Newton step is declared to have no descent.
const MAX_HALVINGS: u32 = 30;

/// The relative step on a tray's liquid its hydraulic slope is read over.
const SLOPE_DIFFERENCE: f64 = 1.0e-6;

/// Forward-difference steps of the Jacobian: absolute for a temperature [K],
/// relative for a vapour.
const TEMPERATURE_DIFFERENCE_K: f64 = 1.0e-4;
const VAPOUR_DIFFERENCE: f64 = 1.0e-7;

/// One column's tick, everything but the unknowns fixed.
pub(crate) struct ColumnStep<'a> {
    pub(crate) plan: &'a CascadePlan,
    pub(crate) spec: &'a CascadeSpec,
    pub(crate) holdup: &'a ColumnHoldup,
    pub(crate) state: &'a HoldupState,
    pub(crate) slate: &'a Slate,
    pub(crate) thermo: &'a dyn ThermoModel,
    /// The condenser's pressure [Pa]: the column's declared one.
    pub(crate) pressure: Pascal,
    /// Each stage's pressure [Pa], top first.
    pub(crate) stage_pressures: &'a [Pascal],
    /// The feed's mole fractions, its mass [kg/s] and molar [mol/s] rates and
    /// its molar enthalpy on the engine's datum [J/mol].
    pub(crate) feed: &'a MoleFractions,
    pub(crate) feed_mass: f64,
    pub(crate) feed_molar: f64,
    pub(crate) feed_enthalpy: f64,
    /// The scale an energy balance's residual is taken against [W].
    pub(crate) energy_scale: f64,
    /// The convergence tolerance every scaled residual must meet [-].
    pub(crate) tolerance: f64,
}

/// Where a tick's Newton starts: each stage's temperature [K] and vapour
/// [mol/s] — a hint, the last tick's answer.
pub(crate) struct StepStart<'a> {
    pub(crate) temperatures: &'a [f64],
    pub(crate) vapour: &'a [f64],
}

/// What one tick came to.
pub(crate) struct Stepped {
    /// Each stage's temperature [K] and liquid, normalised mole fractions.
    pub(crate) temperatures: Vec<f64>,
    pub(crate) liquid: Vec<Vec<f64>>,
    /// Each stage's K-values.
    pub(crate) k: Vec<Vec<f64>>,
    /// The flows the tick ran at [mol/s].
    pub(crate) flows: Flows,
    /// The drum's liquid, normalised mole fractions, and its temperature
    /// [K]: its bubble point.
    pub(crate) drum_liquid: Vec<f64>,
    pub(crate) drum_temperature: Kelvin,
    /// Each draw's component flows [kg/s per component], in the plan's order:
    /// the distillate, the side draws, the bottoms.
    pub(crate) draw_masses: Vec<Vec<f64>>,
    /// The reboiler's duty [W] and the condenser's [W].
    pub(crate) reboiler_duty: f64,
    pub(crate) condenser_duty: f64,
    /// The side draws a tray short of liquid cut back, as plan indices, and
    /// the share of their set flow each ran at.
    pub(crate) cut_back: Vec<(usize, f64)>,
    /// The levels and limits the tick read, and which stream held the base.
    pub(crate) reading: HoldupReading,
    /// The state the tick leaves, but for the trays' vapour loads, which the
    /// caller reads off the trays (`trays::tray_vapour_loads`).
    pub(crate) state: HoldupState,
}

/// The liquid flows a tick runs at, set before its solve [mol/s].
struct SetFlows {
    /// The liquid off each tray at the start of the tick, from what it holds
    /// (down, drawn and into a loop); zero at the base.
    off: Vec<f64>,
    /// How fast that liquid answers what the tray holds [1/s]: `dL/dM` at the
    /// start of the tick, a tray's hydraulic lag's inverse.
    slope: Vec<f64>,
    /// Each stage's liquid molar enthalpy at the start of the tick [J/mol],
    /// and the drum's.
    enthalpy: Vec<f64>,
    drum_enthalpy: f64,
    /// Liquid drawn off each stage: the side draws, and the bottoms at the
    /// base.
    drawn: Vec<f64>,
    /// Each running loop's circulation.
    loops: Vec<f64>,
    distillate: f64,
    reflux: f64,
    /// The reboiler's duty [W].
    reboiler: f64,
    /// Each side draw's share of its set flow it ran at, where under one.
    cut_back: Vec<(usize, f64)>,
    /// The levels and limits the tick started at, and which stream held the
    /// base.
    reading: HoldupReading,
    /// The base's level loop's integral this tick ran on — the state's, or
    /// on a tick the hold changed, the hand-over's.
    base_integral: f64,
    /// The bottoms [kg/s].
    bottoms_mass: f64,
}

/// One evaluation of the residuals.
struct Evaluation {
    residual: Vec<f64>,
    /// Each stage's end-of-tick amounts, and the drum's [mol].
    amounts: Vec<Vec<f64>>,
    drum_amounts: Vec<f64>,
    /// The unnormalised mole fractions the sweep returned, stage and drum.
    x: Vec<Vec<f64>>,
    x_drum: Vec<f64>,
    liquid: Vec<Vec<f64>>,
    k: Vec<Vec<f64>>,
    flows: Flows,
    drum_temperature: Kelvin,
    condenser_duty: f64,
}

impl ColumnStep<'_> {
    fn molar_masses(&self) -> Vec<f64> {
        (0..self.slate.len())
            .map(|c| self.slate.get(c).molar_mass.value())
            .collect()
    }

    /// A held liquid's moles of each component [mol].
    fn moles_of(&self, mass: Kg, composition: &Composition) -> Vec<f64> {
        composition
            .fractions()
            .iter()
            .zip(self.molar_masses())
            .map(|(w, m)| mass.value() * w / m)
            .collect()
    }

    /// The tick: the liquid flows set, then Newton on the temperatures and
    /// vapours from `start`.
    pub(crate) fn step(&self, start: &StepStart<'_>) -> Result<Stepped, SimError> {
        let plan = self.plan;
        let n = plan.stages;
        let state = self.state;
        let shapes = state.stage_liquid.len() == n
            && state.stage_composition.len() == n
            && state.stage_temperature.len() == n
            && state.tray_vapour.len() == n - 1
            && state.tray_vapour_density.len() == n - 1
            && start.temperatures.len() == n
            && start.vapour.len() == n;
        if !shapes {
            return Err(SimError::Numerical(format!(
                "internal: a column's holdup state does not match its {n} stages \
                 (docs/DESIGN.md §79.2)."
            )));
        }
        let amounts: Vec<Vec<f64>> = (0..n)
            .map(|j| self.moles_of(state.stage_liquid[j], &state.stage_composition[j]))
            .collect();
        let drum = self.moles_of(state.drum_liquid, &state.drum_composition);
        let set = self.set_flows(&amounts, &drum)?;

        let mut u: Vec<f64> = start.temperatures.to_vec();
        u.extend_from_slice(start.vapour);
        let mut evaluation = self.evaluate(&u, &amounts, &drum, &set)?;
        let mut iterations = 0;
        while norm(&evaluation.residual) > self.tolerance {
            if iterations == MAX_ITERATIONS {
                return Err(self.unconverged(&evaluation, iterations));
            }
            iterations += 1;
            let jacobian = self.jacobian(&u, &evaluation.residual, &amounts, &drum, &set)?;
            let negated: Vec<f64> = evaluation.residual.iter().map(|r| -r).collect();
            let Some(mut delta) = solve_dense(jacobian, negated) else {
                return Err(self.unconverged(&evaluation, iterations));
            };
            // Damped: no temperature by more than MAX_TEMPERATURE_STEP_K, no
            // vapour through zero.
            let mut scale: f64 = 1.0;
            for (j, d) in delta.iter().enumerate() {
                if j < n {
                    if d.abs() > MAX_TEMPERATURE_STEP_K {
                        scale = scale.min(MAX_TEMPERATURE_STEP_K / d.abs());
                    }
                } else if *d < 0.0 {
                    scale = scale.min(FRACTION_TO_BOUNDARY * u[j] / -d);
                }
            }
            for d in &mut delta {
                *d *= scale;
            }
            let current = norm(&evaluation.residual);
            let mut accepted = None;
            let mut length = 1.0;
            for _ in 0..MAX_HALVINGS {
                let trial: Vec<f64> = u.iter().zip(&delta).map(|(a, d)| a + length * d).collect();
                if let Ok(next) = self.evaluate(&trial, &amounts, &drum, &set) {
                    if norm(&next.residual) < current {
                        accepted = Some((trial, next));
                        break;
                    }
                }
                length *= 0.5;
            }
            let Some((trial, next)) = accepted else {
                return Err(self.unconverged(&evaluation, iterations));
            };
            u = trial;
            evaluation = next;
        }
        self.finish(&u, evaluation, &set)
    }

    /// The liquid off each tray from what it holds, the draws and loops off
    /// it, the level loops' distillate, bottoms and reboiler duty (§79.2
    /// steps 1–2), all from the start-of-tick state, in moles at its
    /// compositions.
    fn set_flows(&self, amounts: &[Vec<f64>], drum: &[f64]) -> Result<SetFlows, SimError> {
        let plan = self.plan;
        let n = plan.stages;
        let state = self.state;
        let mw = self.molar_masses();
        let fractions = |a: &[f64]| -> Result<Vec<f64>, SimError> {
            Ok(MoleFractions::from_amounts(a)?.fractions().to_vec())
        };
        let stage_x: Vec<Vec<f64>> = amounts
            .iter()
            .map(|a| fractions(a))
            .collect::<Result<_, _>>()?;
        let drum_x = fractions(drum)?;
        let mean = |x: &[f64]| x.iter().zip(&mw).map(|(x, m)| x * m).sum::<f64>();

        // The side draws' and the loops' set mass flows on each tray [kg/s].
        let mut side = vec![0.0; n];
        let mut side_draw: Vec<Option<usize>> = vec![None; n];
        let last = plan.mass_ratios.len() - 1;
        for (i, location) in plan.locations.iter().enumerate() {
            if let DrawLocation::Stage(j) = location {
                if i != 0 && i != last {
                    side[*j] = plan.mass_ratios[i] * self.feed_mass;
                    side_draw[*j] = Some(i);
                }
            }
        }
        let mut looped = vec![0.0; n];
        for l in &plan.loops {
            looped[l.draw] += l.mass_flow;
        }

        let mut off_tray = vec![0.0; n];
        let mut slope = vec![0.0; n];
        let mut drawn = vec![0.0; n];
        let mut loop_share = vec![1.0; n];
        let mut cut_back = Vec::new();
        for j in 0..n - 1 {
            let section = crate::trays::section_for(self.spec, j as u32 + 1)?;
            let off = crate::trays::liquid_off_tray(
                self.slate,
                section,
                state.stage_liquid[j],
                &stage_x[j],
                state.stage_temperature[j],
                state.tray_vapour[j],
                state.tray_vapour_density[j],
            )?
            .value();
            // A tray short of its draws gives what it has: the side draw
            // first, then the loop, then nothing goes down.
            let side_mass = side[j].min(off);
            let loop_mass = looped[j].min(off - side_mass);
            if side[j] > 0.0 && side_mass < side[j] {
                if let Some(i) = side_draw[j] {
                    cut_back.push((i, side_mass / side[j]));
                }
            }
            if looped[j] > 0.0 {
                loop_share[j] = loop_mass / looped[j];
            }
            let molar = mean(&stage_x[j]);
            drawn[j] = side_mass / molar;
            off_tray[j] = off / molar;
            // The tray's hydraulic slope, by a forward difference on what it
            // holds.
            let more = Kg(state.stage_liquid[j].value() * (1.0 + SLOPE_DIFFERENCE));
            let off_more = crate::trays::liquid_off_tray(
                self.slate,
                section,
                more,
                &stage_x[j],
                state.stage_temperature[j],
                state.tray_vapour[j],
                state.tray_vapour_density[j],
            )?
            .value();
            slope[j] = (off_more - off) / (more.value() - state.stage_liquid[j].value());
        }
        let loops = plan
            .loops
            .iter()
            .map(|l| l.mass_flow * loop_share[l.draw] / mean(&stage_x[l.draw]))
            .collect();

        // The level loops (§79.2 step 2): each level its liquid over its
        // working volume, its departure from half full a share of its normal
        // holdup.
        let level = |mass: Kg, x: &[f64], t: Kelvin, volume: f64| {
            let density = crate::trays::hot_liquid_density(self.slate, x, t).value();
            mass.value() / (density * volume)
        };
        let drum_volume = self.holdup.drum_volume.value();
        let base_volume = self.holdup.base_volume.value();
        let drum_level = level(
            state.drum_liquid,
            &drum_x,
            state.drum_temperature,
            drum_volume,
        );
        let base_level = level(
            state.stage_liquid[n - 1],
            &stage_x[n - 1],
            state.stage_temperature[n - 1],
            base_volume,
        );
        let (drum_error, base_error) = (drum_level / 0.5 - 1.0, base_level / 0.5 - 1.0);
        let output = |error: f64, integral: f64| (1.0 + LEVEL_GAIN * error + integral).max(0.0);
        let held = self.spec.reboiler_duty.is_some();
        // A hold changed since the last tick hands the base's level to the
        // other stream from where that stream stood (§79.2, the toggle): its
        // loop's integral set so its output is what it last ran at.
        let base_integral = if held == state.held {
            state.base_integral
        } else {
            let (now, settled) = if held {
                (state.last_bottoms.value(), state.settled_bottoms.value())
            } else {
                (state.last_reboiler.value(), state.settled_reboiler.value())
            };
            now / settled - 1.0 - LEVEL_GAIN * base_error
        };
        let dt = self.holdup.tick.value();
        let controlled = state.settled_distillate.value() * output(drum_error, state.drum_integral);
        let mut overflow = 0.0;
        let (mut bottoms_mass, mut reboiler) = match self.spec.reboiler_duty {
            // Scheme B: the reboiler held, the bottoms holds the base's level.
            Some(duty) => (
                state.settled_bottoms.value() * output(base_error, base_integral),
                duty.value(),
            ),
            // Scheme A: the reboiler holds the base's level, the bottoms at its
            // share of the feed.
            None => (
                plan.mass_ratios[last] * self.feed_mass,
                state.settled_reboiler.value() * output(base_error, base_integral),
            ),
        };
        // The physical limits (§79.2): a drum low enough that the reflux
        // pump loses its suction sends out only what that leaves, reflux and
        // distillate alike; a base low enough that the reboiler's tubes
        // uncover boils only in proportion; a drum or base past full passes
        // its excess on with its product within the tick.
        let mut drum_limit = LevelLimit::Normal;
        let mut base_limit = LevelLimit::Normal;
        let mut drum_out_share = 1.0;
        if drum_level < LOW_LEVEL {
            drum_out_share = (drum_level / LOW_LEVEL).max(0.0);
            drum_limit = LevelLimit::Dry;
        }
        if base_level < LOW_LEVEL {
            reboiler *= (base_level / LOW_LEVEL).max(0.0);
            base_limit = LevelLimit::Dry;
        }
        if drum_level > 1.0 {
            overflow = state.drum_liquid.value() * (1.0 - 1.0 / drum_level) / dt;
            drum_limit = LevelLimit::Full;
        }
        if base_level > 1.0 {
            bottoms_mass += state.stage_liquid[n - 1].value() * (1.0 - 1.0 / base_level) / dt;
            base_limit = LevelLimit::Full;
        }
        drawn[n - 1] = bottoms_mass / mean(&stage_x[n - 1]);
        let distillate = (controlled * drum_out_share + overflow) / mean(&drum_x);
        let reflux = self.spec.reflux_ratio * controlled * drum_out_share / mean(&drum_x);
        let enthalpy = (0..n)
            .map(|j| liquid_enthalpy(self.slate, &stage_x[j], state.stage_temperature[j]))
            .collect();
        let drum_enthalpy = liquid_enthalpy(self.slate, &drum_x, state.drum_temperature);
        Ok(SetFlows {
            off: off_tray,
            slope,
            enthalpy,
            drum_enthalpy,
            drawn,
            loops,
            distillate,
            reflux,
            reboiler,
            cut_back,
            reading: HoldupReading {
                held,
                drum_level,
                base_level,
                drum_limit,
                base_limit,
            },
            base_integral,
            bottoms_mass,
        })
    }

    /// The residuals at `u` (the temperatures, then the vapours). An `Err` is
    /// an iterate outside the physical region: the line search treats it as a
    /// step too long.
    fn evaluate(
        &self,
        u: &[f64],
        amounts: &[Vec<f64>],
        drum: &[f64],
        set: &SetFlows,
    ) -> Result<Evaluation, SimError> {
        let plan = self.plan;
        let slate = self.slate;
        let n = plan.stages;
        let nc = slate.len();
        let dt = self.holdup.tick.value();
        let outside = |what: &str| SimError::Numerical(format!("column step: {what}"));
        let temperatures = &u[..n];
        let vapour = &u[n..2 * n];
        if temperatures.iter().any(|t| !t.is_finite() || *t <= 0.0) {
            return Err(outside("a stage temperature left the physical region"));
        }
        if vapour.iter().any(|v| !v.is_finite() || *v <= 0.0) {
            return Err(outside("a vapour rate left the physical region"));
        }
        let (into, out) = plan.circulation(&set.loops);
        let feed_on = |j: usize| {
            if j == plan.feed_stage {
                self.feed_molar
            } else {
                0.0
            }
        };
        // Each stage's moles at the end of the tick, and the liquid off each
        // tray, top down: the start-of-tick flow plus its slope times what the
        // tray comes to hold more — backward Euler on the tray's hydraulics,
        // so the liquid answers this tick's vapour (§79.4: lagged a tick, the
        // vapour's answer to a liquid flow, which the stage energy balances
        // amplify up the column, made the column ring and diverge).
        let held: Vec<f64> = amounts.iter().map(|a| a.iter().sum()).collect();
        let mut liquid_in = vec![0.0; n];
        let mut down = vec![0.0; n];
        let mut held_next = Vec::with_capacity(n);
        liquid_in[0] = set.reflux;
        for j in 0..n {
            let from_below = if j + 1 < n { vapour[j + 1] } else { 0.0 };
            let arriving = liquid_in[j] + from_below + into[j] + feed_on(j);
            if j + 1 < n {
                let k = set.slope[j] * dt;
                let off = (set.off[j] + k * (arriving - vapour[j])) / (1.0 + k);
                // A tray whose liquid over its weir falls short of its draws
                // within the tick sends nothing down; its draws come out of
                // what it holds (§79.6: the feed back in one tick, the side
                // draws at their rate on trays the cut drained).
                down[j] = (off - set.drawn[j] - out[j]).max(0.0);
                liquid_in[j + 1] = down[j];
            }
            held_next.push(held[j] + dt * (arriving - down[j] - set.drawn[j] - out[j] - vapour[j]));
        }
        let drum_held: f64 = drum.iter().sum();
        let drum_next = drum_held + dt * (vapour[0] - set.reflux - set.distillate);
        if let Some(j) = held_next.iter().position(|h| h.is_nan() || *h <= 0.0) {
            let which = if j == n - 1 {
                format!("stage {} (the base)", j + 1)
            } else {
                format!("stage {}", j + 1)
            };
            return Err(outside(&format!("{which} ran out of liquid")));
        }
        if drum_next.is_nan() || drum_next <= 0.0 {
            return Err(outside("the reflux drum ran out of liquid"));
        }

        let mut k = Vec::with_capacity(n);
        for (j, t) in temperatures.iter().enumerate() {
            let mut row = Vec::with_capacity(nc);
            for c in 0..nc {
                let value = self
                    .thermo
                    .k_value(slate, c, Kelvin(*t), self.stage_pressures[j])?;
                if !value.is_finite() || value <= 0.0 {
                    return Err(outside("a K-value left the physical region"));
                }
                row.push(value);
            }
            k.push(row);
        }

        // Each component down the column, the drum a row above the top stage
        // (row 0), each row's end-of-tick moles over dt on its diagonal.
        let rows = n + 1;
        let mut x = vec![vec![0.0; nc]; n];
        let mut x_drum = vec![0.0; nc];
        for c in 0..nc {
            let mut lower = vec![0.0; rows];
            let mut diagonal = vec![0.0; rows];
            let mut upper = vec![0.0; rows];
            let mut rhs = vec![0.0; rows];
            diagonal[0] = drum_next / dt + set.reflux + set.distillate;
            upper[0] = -vapour[0] * k[0][c];
            rhs[0] = drum[c] / dt;
            for j in 0..n {
                let r = j + 1;
                lower[r] = -liquid_in[j];
                diagonal[r] =
                    held_next[j] / dt + down[j] + set.drawn[j] + out[j] + vapour[j] * k[j][c];
                if j + 1 < n {
                    upper[r] = -(vapour[j + 1] * k[j + 1][c] + into[j]);
                }
                rhs[r] = amounts[j][c] / dt + feed_on(j) * self.feed.fractions()[c];
            }
            let solved = thomas(&lower, &diagonal, &upper, &rhs)
                .ok_or_else(|| outside("a component's sweep is singular"))?;
            x_drum[c] = solved[0];
            for j in 0..n {
                x[j][c] = solved[j + 1];
            }
        }
        if x.iter()
            .flatten()
            .chain(&x_drum)
            .any(|v| !v.is_finite() || *v < 0.0)
        {
            return Err(outside("a mole fraction left the physical region"));
        }

        let mut residual = Vec::with_capacity(2 * n);
        for (row, k_row) in x.iter().zip(&k) {
            residual.push(row.iter().zip(k_row).map(|(x, k)| x * k).sum::<f64>() - 1.0);
        }
        let normalise = |row: &[f64]| -> Result<Vec<f64>, SimError> {
            Ok(MoleFractions::from_amounts(row)?.fractions().to_vec())
        };
        let liquid: Vec<Vec<f64>> = x.iter().map(|r| normalise(r)).collect::<Result<_, _>>()?;
        let drum_liquid = normalise(&x_drum)?;
        let mut h = Vec::with_capacity(n);
        let mut big_h = Vec::with_capacity(n);
        for j in 0..n {
            let t = Kelvin(temperatures[j]);
            let y = equilibrium_vapour(&liquid[j], &k[j])?;
            h.push(liquid_enthalpy(slate, &liquid[j], t));
            big_h.push(liquid_enthalpy(slate, &y, t) + mixture_dh_vap(slate, self.thermo, &y, t)?);
        }
        let drum_temperature = bubble_temperature_unnormalized(
            slate,
            self.thermo,
            &drum_liquid,
            self.pressure,
            "the reflux drum",
        )?;
        let reflux_enthalpy = liquid_enthalpy(slate, &drum_liquid, drum_temperature);
        let returns = plan.loop_returns(slate, &liquid, temperatures, &set.loops);
        let removed = plan.removed_heat(&h, &set.loops, &returns);
        for j in 0..n {
            let above = if j == 0 { reflux_enthalpy } else { h[j - 1] };
            let from_below = if j + 1 < n {
                vapour[j + 1] * big_h[j + 1]
            } else {
                0.0
            };
            let duty = if j == n - 1 { set.reboiler } else { 0.0 };
            // The heat the stage's liquid comes to hold: its moles and its
            // enthalpy at the end of the tick against the start's.
            let stored = (held_next[j] * h[j] - held[j] * set.enthalpy[j]) / dt;
            let imbalance =
                liquid_in[j] * above + from_below + feed_on(j) * self.feed_enthalpy + duty
                    - (down[j] + set.drawn[j]) * h[j]
                    - vapour[j] * big_h[j]
                    - removed[j]
                    - stored;
            residual.push(imbalance / self.energy_scale);
        }
        if residual.iter().any(|r| !r.is_finite()) {
            return Err(outside("a residual is not finite"));
        }
        let amounts_next: Vec<Vec<f64>> = x
            .iter()
            .zip(&held_next)
            .map(|(row, h)| row.iter().map(|v| v * h).collect())
            .collect();
        let drum_amounts: Vec<f64> = x_drum.iter().map(|v| v * drum_next).collect();
        // The condenser takes what the top vapour brings less what leaves the
        // drum and what its liquid comes to hold.
        let condenser_duty = vapour[0] * big_h[0]
            - (set.reflux + set.distillate) * reflux_enthalpy
            - (drum_next * reflux_enthalpy - drum_held * set.drum_enthalpy) / dt;
        let flows = Flows {
            liquid_in,
            liquid_down: down,
            liquid_drawn: set.drawn.clone(),
            vapour: vapour.to_vec(),
            reflux: set.reflux,
            circulation_in: into,
            circulation_out: out,
        };
        Ok(Evaluation {
            residual,
            amounts: amounts_next,
            drum_amounts,
            x,
            x_drum,
            liquid,
            k,
            flows,
            drum_temperature,
            condenser_duty,
        })
    }

    fn jacobian(
        &self,
        u: &[f64],
        base: &[f64],
        amounts: &[Vec<f64>],
        drum: &[f64],
        set: &SetFlows,
    ) -> Result<Vec<Vec<f64>>, SimError> {
        let n = self.plan.stages;
        let m = u.len();
        let mut columns = Vec::with_capacity(m);
        for i in 0..m {
            let step = if i < n {
                TEMPERATURE_DIFFERENCE_K
            } else {
                VAPOUR_DIFFERENCE * u[i].abs().max(1.0)
            };
            let mut moved = u.to_vec();
            moved[i] += step;
            let evaluation = self.evaluate(&moved, amounts, drum, set)?;
            columns.push(
                evaluation
                    .residual
                    .iter()
                    .zip(base)
                    .map(|(a, b)| (a - b) / step)
                    .collect::<Vec<f64>>(),
            );
        }
        Ok((0..m)
            .map(|row| (0..m).map(|col| columns[col][row]).collect())
            .collect())
    }

    /// A tick whose Newton did not converge: refused, naming what was
    /// unmet — a stage whose vapour its balance would put at zero names the
    /// stage and its shortfall [W] (§79.2).
    fn unconverged(&self, evaluation: &Evaluation, iterations: u32) -> SimError {
        let n = self.plan.stages;
        let worst = evaluation
            .residual
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
            .map(|(i, r)| (i, *r));
        let what = match worst {
            Some((i, r)) if i >= n => format!(
                "stage {}'s energy balance short by {:.3} MW",
                i - n + 1,
                r * self.energy_scale / 1.0e6
            ),
            Some((i, r)) => format!("stage {}'s bubble point off by {r:.3e}", i + 1),
            None => "nothing".to_string(),
        };
        let thinnest = evaluation
            .flows
            .vapour
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.total_cmp(b.1))
            .map(|(j, v)| format!(" (the least vapour: stage {}, {v:.3} mol/s)", j + 1))
            .unwrap_or_default();
        SimError::Numerical(format!(
            "column step: the tick did not converge after {iterations} Newton steps — \
             {what}{thinnest} (docs/DESIGN.md §79.2)."
        ))
    }

    /// The tick's answer from its converged evaluation: the state it leaves
    /// and the products, both off the sweep's amounts.
    fn finish(
        &self,
        u: &[f64],
        evaluation: Evaluation,
        set: &SetFlows,
    ) -> Result<Stepped, SimError> {
        let plan = self.plan;
        let n = plan.stages;
        let dt = self.holdup.tick.value();
        let mw = self.molar_masses();
        let to_mass =
            |moles: &[f64]| -> Vec<f64> { moles.iter().zip(&mw).map(|(m, w)| m * w).collect() };
        let held_mass = |moles: &[f64]| -> Result<(Kg, Composition), SimError> {
            let masses = to_mass(moles);
            Ok((Kg(masses.iter().sum()), Composition::from_weights(&masses)?))
        };
        let mut stage_liquid = Vec::with_capacity(n);
        let mut stage_composition = Vec::with_capacity(n);
        for a in &evaluation.amounts {
            let (mass, composition) = held_mass(a)?;
            stage_liquid.push(mass);
            stage_composition.push(composition);
        }
        let (drum_liquid, drum_composition) = held_mass(&evaluation.drum_amounts)?;

        // Each draw's component flows, off the sweep's own unnormalised
        // fractions (the flow times x' is what the sweep took out).
        let flows_of = |rate: f64, x: &[f64]| -> Vec<f64> {
            x.iter().zip(&mw).map(|(x, m)| rate * x * m).collect()
        };
        let mut draw_masses = Vec::with_capacity(plan.locations.len());
        for location in &plan.locations {
            draw_masses.push(match location {
                DrawLocation::Condenser => flows_of(set.distillate, &evaluation.x_drum),
                DrawLocation::Stage(j) => flows_of(set.drawn[*j], &evaluation.x[*j]),
            });
        }

        // The level loops' integrals, each held while its stream is pinned at
        // zero and the level would push it lower, or while its level is past
        // a limit (no wind-up).
        let state = self.state;
        let reading = set.reading;
        let integrate = |integral: f64, level: f64, limit: LevelLimit| {
            let error = level / 0.5 - 1.0;
            let pinned = 1.0 + LEVEL_GAIN * error + integral <= 0.0 && error < 0.0;
            if pinned || limit != LevelLimit::Normal {
                integral
            } else {
                integral + dt * LEVEL_GAIN / LEVEL_INTEGRAL_S * error
            }
        };
        let next = HoldupState {
            stage_liquid,
            stage_composition,
            drum_liquid,
            drum_composition,
            stage_temperature: u[..n].iter().copied().map(Kelvin).collect(),
            drum_temperature: evaluation.drum_temperature,
            tray_vapour: Vec::new(),
            tray_vapour_density: Vec::new(),
            tray_drops: Vec::new(),
            settled_distillate: state.settled_distillate,
            settled_bottoms: state.settled_bottoms,
            settled_reboiler: state.settled_reboiler,
            drum_integral: integrate(state.drum_integral, reading.drum_level, reading.drum_limit),
            base_integral: integrate(set.base_integral, reading.base_level, reading.base_limit),
            held: reading.held,
            last_bottoms: KgPerSec(set.bottoms_mass),
            last_reboiler: Watt(set.reboiler),
        };
        Ok(Stepped {
            temperatures: u[..n].to_vec(),
            liquid: evaluation.liquid,
            k: evaluation.k,
            flows: evaluation.flows,
            drum_liquid: MoleFractions::from_amounts(&evaluation.x_drum)?
                .fractions()
                .to_vec(),
            drum_temperature: evaluation.drum_temperature,
            draw_masses,
            reboiler_duty: set.reboiler,
            condenser_duty: evaluation.condenser_duty,
            cut_back: set.cut_back.clone(),
            reading,
            state: next,
        })
    }
}

/// The largest absolute residual.
fn norm(residual: &[f64]) -> f64 {
    residual.iter().fold(0.0, |m, r| m.max(r.abs()))
}

/// The tridiagonal solve: `lower[i]·x[i−1] + diagonal[i]·x[i] + upper[i]·x[i+1]
/// = rhs[i]` (Thomas). `None` on a zero pivot.
fn thomas(lower: &[f64], diagonal: &[f64], upper: &[f64], rhs: &[f64]) -> Option<Vec<f64>> {
    let rows = diagonal.len();
    let mut c = vec![0.0; rows];
    let mut d = vec![0.0; rows];
    let mut pivot = diagonal[0];
    if pivot == 0.0 {
        return None;
    }
    c[0] = upper[0] / pivot;
    d[0] = rhs[0] / pivot;
    for i in 1..rows {
        pivot = diagonal[i] - lower[i] * c[i - 1];
        if pivot == 0.0 {
            return None;
        }
        c[i] = upper[i] / pivot;
        d[i] = (rhs[i] - lower[i] * d[i - 1]) / pivot;
    }
    let mut x = vec![0.0; rows];
    x[rows - 1] = d[rows - 1];
    for i in (0..rows - 1).rev() {
        x[i] = d[i] - c[i] * x[i + 1];
    }
    Some(x)
}
