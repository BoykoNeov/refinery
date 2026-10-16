//! Newton on the heat-balance cascade's own equations (M61, docs/DESIGN.md §66).
//!
//! The fixed point in [`heat_balance`](crate::heat_balance) corrects the
//! temperatures from the bubble points and the vapours from the stage energy
//! balances one after the other; on a wide-boiling crude it stalls or oscillates
//! well inside the column's physical range (§66.0, finding 3). This module
//! solves the same equations SIMULTANEOUSLY — Tomich's arrangement of the
//! bubble-point method's unknowns (*AIChE J.* 16 (1970) 229) with the draws'
//! molar rates added, because those hang on the draws' compositions:
//!
//! ```text
//!   unknowns   T_j (every stage), V_j (every stage below the top), U_k (every draw
//!              but the bottoms), and R when the vapour floor binds
//!   S_j = Σ_c K_jc·x_jc − 1                          every stage (x unnormalised)
//!   E_j = L_{j−1}·h_{j−1} + V_{j+1}·H_{j+1} + F_j·h_F
//!         − (L_j + U_j)·h_j − V_j·H_j − Q_j           every stage but the reboiler
//!   W_k = U_k·M̄(x_k) − r_k·F                         every draw but the bottoms
//!   V_b = 0.20·F                                      the binding stage, floor active
//! ```
//!
//! Given the unknowns, everything else is the fixed point's own arithmetic: the
//! top vapour `(R+1)·D`, the liquids from the stage material balances top-down,
//! the bottoms from the reboiler's mole balance, the K-values at each `T_j`, each
//! component's liquid profile by the same Thomas sweep. So where both methods
//! converge they agree, and the reboiler's duty is still the closure.

use refinery_core::components::{Composition, Slate};
use refinery_core::error::SimError;
use refinery_core::graph::CascadeSpec;
use refinery_core::traits::ThermoModel;
use refinery_core::units::{Kelvin, Pascal};

use crate::bubble::bubble_temperature_unnormalized;
use crate::cascade::{
    mixture_dh_vap, CascadePlan, DrawLocation, Flows, COMPONENT_RESIDUAL_KG_PER_S,
};
use crate::heat_balance::{equilibrium_vapour, liquid_enthalpy, VAPOUR_FLOOR_OF_FEED};
use crate::molar::MoleFractions;

/// The most a stage temperature may move in one Newton step [K].
///
/// A damping bound, not physics: a K-value is exponential in temperature, so an
/// undamped first step from a rough iterate can throw a stage hundreds of kelvin
/// and every K-value with it. 25 K is the probe's value (§66.0), which converged
/// on every case it was run on.
const MAX_TEMPERATURE_STEP_K: f64 = 25.0;

/// The share of its distance to zero a flow may close in one step [-] — the
/// interior-point "fraction to the boundary" rule, so no vapour, draw or reflux
/// is ever stepped onto zero or through it.
const FRACTION_TO_BOUNDARY: f64 = 0.9;

/// Step halvings before a Newton step is declared to have no descent.
const MAX_HALVINGS: u32 = 30;

/// Armijo's sufficient-decrease constant on the residual norm.
const ARMIJO: f64 = 1.0e-4;

/// Forward-difference steps of the Jacobian: absolute for a temperature [K],
/// relative for a flow or the reflux ratio.
const TEMPERATURE_DIFFERENCE_K: f64 = 1.0e-4;
const FLOW_DIFFERENCE: f64 = 1.0e-7;

/// What a solve holds fixed besides the draws' mass ratios.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Specification {
    /// The declared reflux ratio.
    Reflux,
    /// The vapour leaving this stage (zero-based) held at the floor, with the
    /// reflux ratio an unknown — M60.2's rule as an active set (§66.3).
    Floor(usize),
}

/// One column's equations, everything but the unknowns fixed.
pub(crate) struct ColumnEquations<'a> {
    pub(crate) plan: &'a CascadePlan,
    pub(crate) spec: &'a CascadeSpec,
    pub(crate) slate: &'a Slate,
    pub(crate) thermo: &'a dyn ThermoModel,
    pub(crate) pressure: Pascal,
    pub(crate) feed: &'a MoleFractions,
    /// The same feed as MASS fractions — the basis I7's per-component residual
    /// is taken in.
    pub(crate) feed_composition: &'a Composition,
    /// The feed's mass rate [kg/s].
    pub(crate) feed_mass: f64,
    /// The feed's molar rate [mol/s].
    pub(crate) feed_molar: f64,
    /// The feed's molar enthalpy on the engine's datum [J/mol].
    pub(crate) feed_enthalpy: f64,
    /// The residuals' scale for an energy balance [W]: the feed's molar rate
    /// times its own molar latent heat, the column's characteristic duty.
    pub(crate) energy_scale: f64,
    /// The convergence tolerance every scaled residual must meet [-].
    pub(crate) tolerance: f64,
}

/// Where a solve starts: temperatures [K], vapours leaving each stage [mol/s]
/// (the first is what the condenser takes), and each draw but the bottoms' molar
/// rate [mol/s].
pub(crate) struct Start<'a> {
    pub(crate) temperatures: &'a [f64],
    pub(crate) vapour: &'a [f64],
    pub(crate) draw_rates: &'a [f64],
}

/// A converged solve.
pub(crate) struct Solved {
    pub(crate) temperatures: Vec<f64>,
    /// Each stage's liquid, mole fractions.
    pub(crate) liquid: Vec<Vec<f64>>,
    pub(crate) flows: Flows,
    /// The reflux ratio the column ran at: the declared one, or above it where
    /// the floor binds.
    pub(crate) reflux_ratio: f64,
}

/// A solve that did not converge: why, and the flows at its last iterate —
/// which is where a physical edge (a dry tray, a starved reboiler) shows.
pub(crate) struct Unsolved {
    pub(crate) reason: String,
    pub(crate) flows: Option<Box<Flows>>,
}

/// One evaluation: the residuals and what they were built from.
struct Evaluation {
    residual: Vec<f64>,
    /// The worst per-component mass imbalance of the draws against the feed
    /// [kg/s] — I7's number, `CascadePlan::component_residual` times the feed.
    component_residual: f64,
    liquid: Vec<Vec<f64>>,
    flows: Flows,
}

impl ColumnEquations<'_> {
    fn draw_count(&self) -> usize {
        self.plan.mass_ratios.len() - 1
    }

    fn unknowns(&self, specification: Specification) -> usize {
        let n = self.plan.stages;
        2 * n - 1
            + self.draw_count()
            + usize::from(matches!(specification, Specification::Floor(_)))
    }

    fn reflux_ratio(&self, u: &[f64], specification: Specification) -> f64 {
        match specification {
            Specification::Reflux => self.spec.reflux_ratio,
            Specification::Floor(_) => u[u.len() - 1],
        }
    }

    /// The residuals at `u`. An `Err` is an iterate outside the physical region
    /// (a negative flow, a temperature a K-value refuses): the line search
    /// treats it as a step too long.
    fn evaluate(&self, u: &[f64], specification: Specification) -> Result<Evaluation, SimError> {
        let plan = self.plan;
        let slate = self.slate;
        let n = plan.stages;
        let draws = self.draw_count();
        let temperatures = &u[..n];
        let rates = &u[2 * n - 1..2 * n - 1 + draws];
        let reflux_ratio = self.reflux_ratio(u, specification);
        let outside = |what: &str| SimError::Numerical(format!("column Newton: {what}"));
        if temperatures.iter().any(|t| !t.is_finite() || *t <= 0.0) {
            return Err(outside("a stage temperature left the physical region"));
        }
        if !rates.iter().all(|r| positive(*r)) || !positive(reflux_ratio) {
            return Err(outside(
                "a draw rate or the reflux left the physical region",
            ));
        }

        let distillate = rates[0];
        let mut vapour = Vec::with_capacity(n);
        vapour.push((reflux_ratio + 1.0) * distillate);
        vapour.extend_from_slice(&u[n..2 * n - 1]);
        if !vapour.iter().all(|v| positive(*v)) {
            return Err(outside("a vapour rate left the physical region"));
        }
        let mut drawn = vec![0.0; n];
        for (j, slot) in plan.draw_at_stage.iter().enumerate() {
            if let Some(i) = slot {
                // The distillate leaves through the condenser, not off stage 1,
                // and the bottoms comes from the mole balance below.
                if *i > 0 && *i < draws {
                    drawn[j] = rates[*i];
                }
            }
        }
        let reflux = reflux_ratio * distillate;
        let mut liquid_in = vec![0.0; n];
        let mut liquid_down = vec![0.0; n];
        liquid_in[0] = reflux;
        for j in 0..n - 1 {
            let feed_here = if j == plan.feed_stage {
                self.feed_molar
            } else {
                0.0
            };
            liquid_down[j] = liquid_in[j] + vapour[j + 1] + feed_here - drawn[j] - vapour[j];
            liquid_in[j + 1] = liquid_down[j];
        }
        let feed_last = if n - 1 == plan.feed_stage {
            self.feed_molar
        } else {
            0.0
        };
        let bottoms = liquid_in[n - 1] + feed_last - vapour[n - 1];
        if !liquid_down[..n - 1].iter().all(|l| non_negative(*l)) || !positive(bottoms) {
            return Err(outside("a liquid rate left the physical region"));
        }
        drawn[n - 1] = bottoms;
        liquid_down[n - 1] = 0.0;
        let flows = Flows {
            liquid_in,
            liquid_down,
            liquid_drawn: drawn,
            vapour,
            reflux,
        };

        let mut k = Vec::with_capacity(n);
        for t in temperatures {
            let mut row = Vec::with_capacity(slate.len());
            for c in 0..slate.len() {
                let value = self.thermo.k_value(slate, c, Kelvin(*t), self.pressure)?;
                if !value.is_finite() || value <= 0.0 {
                    return Err(outside("a K-value left the physical region"));
                }
                row.push(value);
            }
            k.push(row);
        }
        // Each component's liquid down the column: the Thomas sweep the fixed
        // point uses, left UNNORMALISED so the summation can read it.
        let mut amounts = vec![vec![0.0; slate.len()]; n];
        for (c, &z) in self.feed.fractions().iter().enumerate() {
            let column = plan.solve_component(&flows, &k, c, z, self.feed_molar)?;
            for (row, x) in amounts.iter_mut().zip(column) {
                row[c] = x;
            }
        }

        let mut residual = Vec::with_capacity(self.unknowns(specification));
        // The summation, one per stage: `Σ K·x = 1` (the bubble point).
        for (row, k_row) in amounts.iter().zip(&k) {
            residual.push(row.iter().zip(k_row).map(|(x, k)| x * k).sum::<f64>() - 1.0);
        }
        let mut liquid = Vec::with_capacity(n);
        for row in &amounts {
            liquid.push(MoleFractions::from_amounts(row)?.fractions().to_vec());
        }
        let mut h = Vec::with_capacity(n);
        let mut big_h = Vec::with_capacity(n);
        for j in 0..n {
            let t = Kelvin(temperatures[j]);
            let y = equilibrium_vapour(&liquid[j], &k[j])?;
            h.push(liquid_enthalpy(slate, &liquid[j], t));
            big_h.push(liquid_enthalpy(slate, &y, t) + mixture_dh_vap(slate, self.thermo, &y, t)?);
        }
        // The reflux: stage 1's vapour, condensed, at its own bubble point —
        // the fixed point's `reflux_enthalpy`.
        let condensate = equilibrium_vapour(&liquid[0], &k[0])?;
        let condenser_t = bubble_temperature_unnormalized(
            slate,
            self.thermo,
            &condensate,
            self.pressure,
            "the total condenser",
        )?;
        let reflux_enthalpy = liquid_enthalpy(slate, &condensate, condenser_t);
        // Each stage's energy balance but the reboiler's (which is the closure).
        for j in 0..n - 1 {
            let feed_here = if j == plan.feed_stage {
                self.feed_molar
            } else {
                0.0
            };
            let above = if j == 0 { reflux_enthalpy } else { h[j - 1] };
            let imbalance = flows.liquid_in[j] * above
                + flows.vapour[j + 1] * big_h[j + 1]
                + feed_here * self.feed_enthalpy
                - (flows.liquid_down[j] + flows.liquid_drawn[j]) * h[j]
                - flows.vapour[j] * big_h[j]
                - plan.side_heat[j];
            residual.push(imbalance / self.energy_scale);
        }
        // Each draw's mass ratio: its molar rate times its molar mass.
        for (i, rate) in rates.iter().enumerate() {
            let composition = match plan.locations[i] {
                DrawLocation::Condenser => &condensate,
                DrawLocation::Stage(j) => &liquid[j],
            };
            let molar_mass: f64 = composition
                .iter()
                .enumerate()
                .map(|(c, x)| x * slate.get(c).molar_mass.value())
                .sum();
            residual
                .push((rate * molar_mass - plan.mass_ratios[i] * self.feed_mass) / self.feed_mass);
        }
        if let Specification::Floor(stage) = specification {
            let floor = VAPOUR_FLOOR_OF_FEED * self.feed_molar;
            residual.push((flows.vapour[stage] - floor) / self.feed_molar);
        }
        if residual.iter().any(|r| !r.is_finite()) {
            return Err(outside("a residual is not finite"));
        }
        let draws = plan.draw_states(&liquid, &k, slate)?;
        let component_residual =
            plan.component_residual(&draws, self.feed_composition, slate) * self.feed_mass;
        Ok(Evaluation {
            residual,
            component_residual,
            liquid,
            flows,
        })
    }

    /// Newton from `start` under `specification`, at most `max_iterations`
    /// steps.
    pub(crate) fn solve(
        &self,
        start: &Start<'_>,
        specification: Specification,
        max_iterations: u32,
    ) -> Result<Solved, Unsolved> {
        let n = self.plan.stages;
        let mut u: Vec<f64> = start.temperatures.to_vec();
        u.extend_from_slice(&start.vapour[1..n]);
        u.extend_from_slice(start.draw_rates);
        if let Specification::Floor(_) = specification {
            // The reflux the start ran at, never below the declared one: the
            // floor only ever raises it.
            let ran = start.vapour[0] / start.draw_rates[0] - 1.0;
            u.push(ran.max(self.spec.reflux_ratio));
        }
        let mut current = self.evaluate(&u, specification).map_err(|e| Unsolved {
            reason: format!("its starting point is outside the physical region ({e})"),
            flows: None,
        })?;
        let norm = |r: &[f64]| r.iter().map(|x| x * x).sum::<f64>().sqrt();
        let mut current_norm = norm(&current.residual);
        let mut iterations = 0u32;
        loop {
            if self.converged(&current) {
                return Ok(Solved {
                    temperatures: u[..n].to_vec(),
                    liquid: current.liquid,
                    flows: current.flows,
                    reflux_ratio: self.reflux_ratio(&u, specification),
                });
            }
            if iterations >= max_iterations {
                return Err(Unsolved {
                    reason: format!(
                        "did not converge in {iterations} iterations: {}",
                        self.unmet(&current)
                    ),
                    flows: Some(Box::new(current.flows)),
                });
            }
            iterations += 1;

            let jacobian = self.jacobian(&u, &current.residual, specification);
            let negative: Vec<f64> = current.residual.iter().map(|r| -r).collect();
            let Some(step) = solve_dense(jacobian, negative) else {
                return Err(Unsolved {
                    reason: format!(
                        "its Jacobian is singular at iteration {iterations}: {}",
                        self.unmet(&current)
                    ),
                    flows: Some(Box::new(current.flows)),
                });
            };
            let mut scale = 1.0f64;
            for d in &step[..n] {
                if d.abs() > MAX_TEMPERATURE_STEP_K {
                    scale = scale.min(MAX_TEMPERATURE_STEP_K / d.abs());
                }
            }
            for (value, d) in u[n..].iter().zip(&step[n..]) {
                if value + scale * d <= 0.0 {
                    scale = scale.min(FRACTION_TO_BOUNDARY * value / -d);
                }
            }
            let mut accepted = None;
            for _ in 0..=MAX_HALVINGS {
                let trial: Vec<f64> = u.iter().zip(&step).map(|(a, d)| a + scale * d).collect();
                if let Ok(evaluation) = self.evaluate(&trial, specification) {
                    let trial_norm = norm(&evaluation.residual);
                    if trial_norm <= (1.0 - ARMIJO * scale) * current_norm
                        || self.converged(&evaluation)
                    {
                        accepted = Some((trial, evaluation, trial_norm));
                        break;
                    }
                }
                scale *= 0.5;
            }
            let Some((trial, evaluation, trial_norm)) = accepted else {
                return Err(Unsolved {
                    reason: format!(
                        "found no step that improves it at iteration {iterations}: {}",
                        self.unmet(&current)
                    ),
                    flows: Some(Box::new(current.flows)),
                });
            };
            u = trial;
            current = evaluation;
            current_norm = trial_norm;
        }
    }

    /// Every scaled residual within the tolerance AND the per-component mass
    /// balance within I7's bound — the fixed point's own gate, which the
    /// tolerance alone does not imply: a summation off by the tolerance moves a
    /// normalised composition by about as much, which over the feed is I7's
    /// bound itself.
    fn converged(&self, evaluation: &Evaluation) -> bool {
        evaluation
            .residual
            .iter()
            .all(|r| r.abs() <= self.tolerance)
            && evaluation.component_residual <= COMPONENT_RESIDUAL_KG_PER_S
    }

    /// The worst residual, in words.
    fn unmet(&self, evaluation: &Evaluation) -> String {
        let residual = &evaluation.residual;
        let n = self.plan.stages;
        let (worst, size) = residual
            .iter()
            .enumerate()
            .fold((0, 0.0f64), |best, (i, r)| {
                if r.abs() > best.1 {
                    (i, r.abs())
                } else {
                    best
                }
            });
        let what = if worst < n {
            format!("the bubble point of stage {}", worst + 1)
        } else if worst < 2 * n - 1 {
            format!("the energy balance of stage {}", worst - n + 1)
        } else if worst < 2 * n - 1 + self.draw_count() {
            format!("the mass ratio of draw {}", worst - (2 * n - 1) + 1)
        } else {
            "the vapour floor".to_string()
        };
        if size <= self.tolerance {
            return format!(
                "the worst per-component mass residual is {:.3e} kg/s against a bound of \
                 {COMPONENT_RESIDUAL_KG_PER_S:.3e} kg/s",
                evaluation.component_residual
            );
        }
        format!(
            "the worst equation is {what}, off by {size:.3e} (scaled) against a tolerance of {:.3e}",
            self.tolerance
        )
    }

    /// The Jacobian by forward differences — one evaluation per unknown, a
    /// backward difference where the forward one leaves the physical region.
    fn jacobian(&self, u: &[f64], residual: &[f64], specification: Specification) -> Vec<Vec<f64>> {
        let n = self.plan.stages;
        let size = u.len();
        let mut jacobian = vec![vec![0.0; size]; size];
        let mut probe = u.to_vec();
        for column in 0..size {
            let step = if column < n {
                TEMPERATURE_DIFFERENCE_K
            } else {
                FLOW_DIFFERENCE * u[column].abs().max(1.0e-3)
            };
            probe[column] = u[column] + step;
            let (shifted, signed) = match self.evaluate(&probe, specification) {
                Ok(e) => (Some(e.residual), step),
                Err(_) => {
                    probe[column] = u[column] - step;
                    (
                        self.evaluate(&probe, specification)
                            .ok()
                            .map(|e| e.residual),
                        -step,
                    )
                }
            };
            probe[column] = u[column];
            // Neither side evaluable: the column stays zero and the solve
            // reports the singular Jacobian it makes.
            if let Some(shifted) = shifted {
                for (row, (a, b)) in shifted.iter().zip(residual).enumerate() {
                    jacobian[row][column] = (a - b) / signed;
                }
            }
        }
        jacobian
    }
}

/// `A·x = b` by Gaussian elimination with partial pivoting (Golub & Van Loan,
/// *Matrix Computations*, 4th ed., alg. 3.4.1) — at most ~25 unknowns on a
/// shipped column, so dense and by hand. `None` on a singular matrix or a
/// non-finite answer.
fn solve_dense(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for column in 0..n {
        let pivot =
            (column..n).max_by(|i, j| a[*i][column].abs().total_cmp(&a[*j][column].abs()))?;
        if a[pivot][column] == 0.0 || !a[pivot][column].is_finite() {
            return None;
        }
        a.swap(column, pivot);
        b.swap(column, pivot);
        let (above, below) = a.split_at_mut(column + 1);
        let pivot_row = &above[column];
        for (offset, row) in below.iter_mut().enumerate() {
            let factor = row[column] / pivot_row[column];
            if factor != 0.0 {
                for (x, p) in row[column..].iter_mut().zip(&pivot_row[column..]) {
                    *x -= factor * p;
                }
                b[column + 1 + offset] -= factor * b[column];
            }
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let known: f64 = a[row][row + 1..]
            .iter()
            .zip(&x[row + 1..])
            .map(|(coefficient, value)| coefficient * value)
            .sum();
        x[row] = (b[row] - known) / a[row][row];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/// `x > 0`, false for NaN — a flow or ratio inside the physical region.
fn positive(x: f64) -> bool {
    x > 0.0
}

/// `x >= 0`, false for NaN.
fn non_negative(x: f64) -> bool {
    x >= 0.0
}

#[cfg(test)]
mod tests {
    use super::solve_dense;

    #[test]
    fn the_dense_solve_pivots_and_refuses_a_singular_matrix() {
        // A zero leading pivot that only row exchange gets past.
        let x = solve_dense(
            vec![
                vec![0.0, 2.0, 1.0],
                vec![1.0, 1.0, 0.0],
                vec![2.0, 0.0, 3.0],
            ],
            vec![7.0, 3.0, 11.0],
        )
        .unwrap();
        for (got, want) in x.iter().zip([1.0, 2.0, 3.0]) {
            assert!((got - want).abs() < 1e-12, "{x:?}");
        }
        assert!(solve_dense(vec![vec![1.0, 2.0], vec![2.0, 4.0]], vec![1.0, 2.0]).is_none());
    }
}
