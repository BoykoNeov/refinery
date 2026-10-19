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
//!   E_N with Q_reb held                               the reboiler held (M64), R unknown
//! ```
//!
//! Given the unknowns, everything else is the fixed point's own arithmetic: the
//! top vapour `(R+1)·D`, the liquids from the stage material balances top-down,
//! the bottoms from the reboiler's mole balance, the K-values at each `T_j`, each
//! component's liquid profile by the same Thomas sweep. So where both methods
//! converge they agree, and the reboiler's duty is still the closure.
//!
//! **Water** (M62.5, docs/DESIGN.md §67.17): on a feed with free water the
//! cascade is the OIL's, and water a balance of its own. Each wet stage adds
//! its free water `FW_j` to the unknowns and its saturation to the equations;
//! the water vapour follows top-down from them (`water_vapour_profile`), each
//! cut's K is taken at the oil's partial pressure, and every energy balance
//! carries water's enthalpy:
//!
//! ```text
//!   unknowns   … and FW_j on every wet stage
//!   S_j        Σ_c K_jc·x_jc·(V_j + W_j)/V_j − 1
//!   E_j        … + W_{j+1}·H_w,j+1 + FW_{j−1}·h_w,j−1 − W_j·H_w,j − FW_j·h_w,j
//!              (+ S·H_S on the stage steam enters, M62.6, §67.18)
//!   Z_j = W_j·(1 − K_w,j) − K_w,j·V_j                 every wet stage (y_w = P_w/P)
//! ```

use refinery_core::components::{Composition, Slate};
use refinery_core::error::SimError;
use refinery_core::graph::CascadeSpec;
use refinery_core::traits::ThermoModel;
use refinery_core::units::{Kelvin, Pascal};

use crate::bubble::bubble_temperature_unnormalized;
use crate::cascade::{
    mixture_dh_vap, CascadePlan, DrawLocation, Flows, COMPONENT_RESIDUAL_KG_PER_S,
};
use crate::heat_balance::{
    condenser_temperature, equilibrium_vapour, liquid_enthalpy, water_vapour_profile, WaterFeed,
    VAPOUR_FLOOR_OF_FEED,
};
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
    /// The reboiler's duty held at this value [W], with the reflux ratio an
    /// unknown (M64, docs/DESIGN.md §69.1): the bottom stage's energy balance,
    /// the closure everywhere else, is the equation. No floor — the reflux is
    /// the one unknown a floor would move (§69.2).
    Held(f64),
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
    /// The feed's free water (M62.5), or `None` on a feed with none — where
    /// every equation is M61's, bit for bit. With it, `feed`, `feed_mass`,
    /// `feed_molar` and `feed_composition` are the OIL's, and `feed_enthalpy`
    /// is the WHOLE feed's enthalpy flux per mole of oil.
    pub(crate) water: Option<&'a WaterFeed>,
    /// The side draws cut back (M67.1, docs/DESIGN.md §72.1), as indices into
    /// the plan's draws: each one's row holds the liquid leaving its tray at
    /// `liquid_minimum` in place of its mass ratio, and the draw takes what
    /// that leaves. Empty on a column that runs every draw at its share.
    pub(crate) cut_back: &'a [usize],
    /// The liquid a cut-back draw leaves flowing below its tray [mol/s].
    pub(crate) liquid_minimum: f64,
}

/// Where a solve starts: temperatures [K], vapours leaving each stage [mol/s]
/// (the first is what the condenser takes), and each draw but the bottoms' molar
/// rate [mol/s].
pub(crate) struct Start<'a> {
    pub(crate) temperatures: &'a [f64],
    pub(crate) vapour: &'a [f64],
    pub(crate) draw_rates: &'a [f64],
    /// Free water leaving each stage downward [mol/s], read on the wet stages;
    /// empty for none.
    pub(crate) free_water: &'a [f64],
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
    /// Water vapour leaving each stage [mol/s] (M62.5); empty on a dry feed.
    pub(crate) water_vapour: Vec<f64>,
    /// Free water leaving each stage downward [mol/s]; empty on a dry feed.
    pub(crate) free_water: Vec<f64>,
    /// Each stage's water vapour share over its saturation share, `y_w/K_w`
    /// [-]: past 1 a dry stage would hold free water. Empty on a dry feed.
    pub(crate) saturation: Vec<f64>,
    /// Each draw's share of the feed mass it ran at (M67.1): the declared
    /// ones, a cut-back draw's own, the bottoms the rest.
    pub(crate) ratios: Vec<f64>,
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
    water_vapour: Vec<f64>,
    free_water: Vec<f64>,
    saturation: Vec<f64>,
    ratios: Vec<f64>,
}

impl ColumnEquations<'_> {
    fn draw_count(&self) -> usize {
        self.plan.mass_ratios.len() - 1
    }

    fn unknowns(&self, specification: Specification, wet: &[usize]) -> usize {
        self.free_water_offset(specification) + wet.len()
    }

    /// Where the wet stages' free water starts in the unknowns: after the
    /// temperatures, the vapours, the draws and the floor's reflux.
    fn free_water_offset(&self, specification: Specification) -> usize {
        let n = self.plan.stages;
        2 * n - 1
            + self.draw_count()
            + usize::from(matches!(
                specification,
                Specification::Floor(_) | Specification::Held(_)
            ))
    }

    fn reflux_ratio(&self, u: &[f64], specification: Specification) -> f64 {
        match specification {
            Specification::Reflux => self.spec.reflux_ratio,
            Specification::Floor(_) | Specification::Held(_) => {
                u[2 * self.plan.stages - 1 + self.draw_count()]
            }
        }
    }

    /// The residuals at `u`. An `Err` is an iterate outside the physical region
    /// (a negative flow, a temperature a K-value refuses): the line search
    /// treats it as a step too long.
    fn evaluate(
        &self,
        u: &[f64],
        specification: Specification,
        wet: &[usize],
    ) -> Result<Evaluation, SimError> {
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

        // The water: the free water on the wet stages, the vapour from it.
        let mut free_water = Vec::new();
        let mut water_vapour = Vec::new();
        if let Some(water) = self.water {
            free_water = vec![0.0; n];
            let offset = self.free_water_offset(specification);
            for (slot, j) in wet.iter().enumerate() {
                free_water[*j] = u[offset + slot];
            }
            water_vapour = water_vapour_profile(plan, water, &free_water)
                .map_err(|_| outside("a water vapour rate left the physical region"))?;
        }

        let mut k = Vec::with_capacity(n);
        let mut water_k = Vec::new();
        for (j, t) in temperatures.iter().enumerate() {
            let mut row = Vec::with_capacity(slate.len());
            for c in 0..slate.len() {
                let value = self.thermo.k_value(slate, c, Kelvin(*t), self.pressure)?;
                if !value.is_finite() || value <= 0.0 {
                    return Err(outside("a K-value left the physical region"));
                }
                row.push(value);
            }
            if let Some(water) = self.water {
                // Each cut at the oil's partial pressure `P·V/(V + W)`: Raoult's
                // K ∝ 1/P, so the total pressure's K times `(V + W)/V`.
                water_k.push(row[water.index]);
                let scale = (flows.vapour[j] + water_vapour[j]) / flows.vapour[j];
                for value in &mut row {
                    *value *= scale;
                }
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

        let mut residual = Vec::with_capacity(self.unknowns(specification, wet));
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
        let condenser_t = match self.water {
            None => bubble_temperature_unnormalized(
                slate,
                self.thermo,
                &condensate,
                self.pressure,
                "the total condenser",
            )?,
            Some(water) => condenser_temperature(
                slate,
                self.thermo,
                &condensate,
                flows.vapour[0],
                water,
                water_vapour[0],
                self.pressure,
            )?,
        };
        let reflux_enthalpy = liquid_enthalpy(slate, &condensate, condenser_t);
        // Water's own enthalpies at each stage, liquid and vapour [J/mol].
        let mut water_h = Vec::new();
        let mut water_big_h = Vec::new();
        if let Some(water) = self.water {
            for t in temperatures {
                water_h.push(water.liquid_enthalpy(Kelvin(*t)));
                water_big_h.push(water.vapour_enthalpy(slate, self.thermo, Kelvin(*t))?);
            }
        }
        // Each stage's energy balance but the reboiler's (which is the closure),
        // and the reboiler's too where its duty is held.
        for j in 0..n {
            let held = match specification {
                Specification::Held(duty) if j == n - 1 => duty,
                _ if j == n - 1 => continue,
                _ => 0.0,
            };
            let feed_here = if j == plan.feed_stage {
                self.feed_molar
            } else {
                0.0
            };
            let above = if j == 0 { reflux_enthalpy } else { h[j - 1] };
            // Nothing rises into the bottom stage but the steam, its own term.
            let from_below = if j + 1 < n {
                flows.vapour[j + 1] * big_h[j + 1]
            } else {
                0.0
            };
            let imbalance =
                flows.liquid_in[j] * above + from_below + feed_here * self.feed_enthalpy + held
                    - (flows.liquid_down[j] + flows.liquid_drawn[j]) * h[j]
                    - flows.vapour[j] * big_h[j]
                    - plan.side_heat[j];
            let imbalance = if let Some(water) = self.water {
                let from_above = if j == 0 {
                    0.0
                } else {
                    free_water[j - 1] * water_h[j - 1]
                };
                let water_from_below = if j + 1 < n {
                    water_vapour[j + 1] * water_big_h[j + 1]
                } else {
                    0.0
                };
                let imbalance = imbalance + water_from_below + from_above
                    - water_vapour[j] * water_big_h[j]
                    - free_water[j] * water_h[j];
                // The steam into its own stage (M62.6, docs/DESIGN.md §67.18).
                match water.steam {
                    Some(steam) if steam.stage == j => imbalance + steam.enthalpy_flux,
                    _ => imbalance,
                }
            } else {
                imbalance
            };
            residual.push(imbalance / self.energy_scale);
        }
        // Each draw's mass ratio: its molar rate times its molar mass.
        let mut ratios = plan.mass_ratios.clone();
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
            match plan.locations[i] {
                // A cut-back draw (M67.1, §72.1): its tray's liquid down held
                // at the minimum, its share what that leaves it.
                DrawLocation::Stage(j) if self.cut_back.contains(&i) => {
                    residual.push((flows.liquid_down[j] - self.liquid_minimum) / self.feed_molar);
                    ratios[i] = rate * molar_mass / self.feed_mass;
                }
                _ => residual.push(
                    (rate * molar_mass - plan.mass_ratios[i] * self.feed_mass) / self.feed_mass,
                ),
            }
        }
        // The bottoms takes what the others leave of the feed.
        if !self.cut_back.is_empty() {
            let last = ratios.len() - 1;
            ratios[last] = 1.0 - ratios[..last].iter().sum::<f64>();
            if ratios[last] <= 0.0 {
                return Err(outside("the bottoms' share left the physical region"));
            }
        }
        if let Specification::Floor(stage) = specification {
            let floor = VAPOUR_FLOOR_OF_FEED * self.feed_molar;
            residual.push((flows.vapour[stage] - floor) / self.feed_molar);
        }
        // Each wet stage's vapour saturated with water: `y_w = K_w`, or
        // `W·(1 − K_w) = K_w·V` with `V` the oil vapour.
        for j in wet {
            residual.push(
                (water_vapour[*j] * (1.0 - water_k[*j]) - water_k[*j] * flows.vapour[*j])
                    / self.feed_molar,
            );
        }
        let saturation = water_vapour
            .iter()
            .zip(&water_k)
            .zip(&flows.vapour)
            .map(|((w, kw), v)| w / ((v + w) * kw))
            .collect();
        if residual.iter().any(|r| !r.is_finite()) {
            return Err(outside("a residual is not finite"));
        }
        let draws = plan.draw_states(&liquid, &k, slate)?;
        let component_residual =
            plan.component_residual_with(&draws, self.feed_composition, slate, &ratios)
                * self.feed_mass;
        Ok(Evaluation {
            residual,
            component_residual,
            liquid,
            flows,
            water_vapour,
            free_water,
            saturation,
            ratios,
        })
    }

    /// The reboiler duty [W] a solved column's bottom stage implies — its
    /// energy balance held at zero, negated (M64.2, docs/DESIGN.md §69.3): where
    /// the continuation's walk starts. `start` is the solved profile and
    /// `reflux_ratio` the reflux it ran at.
    pub(crate) fn implied_reboiler_duty(
        &self,
        start: &Start<'_>,
        reflux_ratio: f64,
        wet: &[usize],
    ) -> Result<f64, SimError> {
        let n = self.plan.stages;
        let mut u: Vec<f64> = start.temperatures.to_vec();
        u.extend_from_slice(&start.vapour[1..n]);
        u.extend_from_slice(start.draw_rates);
        u.push(reflux_ratio);
        for j in wet {
            u.push(start.free_water.get(*j).copied().unwrap_or(0.0));
        }
        let evaluation = self.evaluate(&u, Specification::Held(0.0), wet)?;
        // The held balance follows the other stages' (§69.1): after the n
        // bubble points and the n − 1 energy balances above it.
        Ok(-evaluation.residual[2 * n - 1] * self.energy_scale)
    }

    /// Newton from `start` under `specification`, at most `max_iterations`
    /// steps.
    pub(crate) fn solve(
        &self,
        start: &Start<'_>,
        specification: Specification,
        wet: &[usize],
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
        if let Specification::Held(_) = specification {
            // The reflux the start ran at: the held duty sets it, not the
            // declared one.
            u.push(start.vapour[0] / start.draw_rates[0] - 1.0);
        }
        for j in wet {
            u.push(start.free_water.get(*j).copied().unwrap_or(0.0));
        }
        // The free water's sign is the wet set's to judge (§67.17 rule 6), so
        // the fraction-to-the-boundary rule stops short of it.
        let bounded = u.len() - wet.len();
        let mut current = self
            .evaluate(&u, specification, wet)
            .map_err(|e| Unsolved {
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
                    water_vapour: current.water_vapour,
                    free_water: current.free_water,
                    saturation: current.saturation,
                    ratios: current.ratios,
                });
            }
            if iterations >= max_iterations {
                return Err(Unsolved {
                    reason: format!(
                        "did not converge in {iterations} iterations: {}",
                        self.unmet(&current, specification)
                    ),
                    flows: Some(Box::new(current.flows)),
                });
            }
            iterations += 1;

            let jacobian = self.jacobian(&u, &current.residual, specification, wet);
            let negative: Vec<f64> = current.residual.iter().map(|r| -r).collect();
            let Some(step) = solve_dense(jacobian, negative) else {
                return Err(Unsolved {
                    reason: format!(
                        "its Jacobian is singular at iteration {iterations}: {}",
                        self.unmet(&current, specification)
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
            for (value, d) in u[n..bounded].iter().zip(&step[n..bounded]) {
                if value + scale * d <= 0.0 {
                    scale = scale.min(FRACTION_TO_BOUNDARY * value / -d);
                }
            }
            let mut accepted = None;
            for _ in 0..=MAX_HALVINGS {
                let trial: Vec<f64> = u.iter().zip(&step).map(|(a, d)| a + scale * d).collect();
                if let Ok(evaluation) = self.evaluate(&trial, specification, wet) {
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
                        self.unmet(&current, specification)
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
    fn unmet(&self, evaluation: &Evaluation, specification: Specification) -> String {
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
        // A held reboiler's balance follows the other stages' (§69.1).
        let held = usize::from(matches!(specification, Specification::Held(_)));
        let energies = 2 * n - 1 + held;
        let what = if worst < n {
            format!("the bubble point of stage {}", worst + 1)
        } else if worst < energies {
            format!("the energy balance of stage {}", worst - n + 1)
        } else if worst < energies + self.draw_count() {
            format!("the mass ratio of draw {}", worst - energies + 1)
        } else if worst < self.free_water_offset(specification) {
            "the vapour floor".to_string()
        } else {
            "the water saturation of a wet stage".to_string()
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
    fn jacobian(
        &self,
        u: &[f64],
        residual: &[f64],
        specification: Specification,
        wet: &[usize],
    ) -> Vec<Vec<f64>> {
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
            let (shifted, signed) = match self.evaluate(&probe, specification, wet) {
                Ok(e) => (Some(e.residual), step),
                Err(_) => {
                    probe[column] = u[column] - step;
                    (
                        self.evaluate(&probe, specification, wet)
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
