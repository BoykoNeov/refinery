//! M73.2.0 PROBE (not engine code): each stage of a column with trays solved
//! at its own pressure, the top at the column's, each stage below higher by
//! the trays above it (docs/DESIGN.md §78.1). Three modes:
//!
//! - `Off`: one pressure, as M73.1 ships; the drops are only read.
//! - `Lag`: this tick's stages at the drops the LAST tick's solves left
//!   (`end_tick` moves them over); the first tick, and after a reset, at one
//!   pressure.
//! - `Solve`: within the pass, solve, read the drops, solve again at them,
//!   until the drops move less than `TOLERANCE_PA` — what a solve with the
//!   pressures inside Newton would land on, NOT its cost.
//!
//! Past flood, `Rule::Raw` takes the correlation's drop as it reads;
//! `Rule::Cap` the drop with the vapour cut to exactly 100 % of jet flood.

use std::cell::RefCell;

use refinery_core::error::SimError;
use refinery_core::traits::{CascadeProfile, ColumnPass, Separation, ThermoModel};
use refinery_core::units::Pascal;

use crate::cascade::PROBE_STAGE_PRESSURES;
use crate::trays::PROBE_CAPPED;

/// The drops' largest move between two solves that counts as settled [Pa].
pub const TOLERANCE_PA: f64 = 0.5;
/// Solves a `Solve` pass may take before it refuses.
pub const MAX_SOLVES: u32 = 60;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mode {
    Off,
    Lag,
    Solve,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Rule {
    Raw,
    Cap,
}

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub mode: Mode,
    pub rule: Rule,
    /// How far each `Solve` iteration moves the drops toward what the solve
    /// read [-]; 1 is plain substitution.
    pub relax: f64,
}

/// One pass over a column with trays.
#[derive(Debug, Clone, Default)]
pub struct PassStat {
    /// Column solves this pass took.
    pub solves: u32,
    /// The drops the returned answer was solved at [Pa], one per tray.
    pub used: Vec<f64>,
    /// The drops the returned answer read, raw [Pa].
    pub raw: Vec<f64>,
    /// The same, capped at 100 % of jet flood [Pa].
    pub capped: Vec<f64>,
    /// The largest `|Σ K(T_j, P_j)·x_j − 1|` over the stages, at each stage's
    /// own pressure — what proves every K was taken there.
    pub summation_at_stage: f64,
    /// The same at the column's one (top) pressure.
    pub summation_at_top: f64,
    /// The last solve's move in the drops [Pa].
    pub change: f64,
}

#[derive(Default)]
struct State {
    lag: Option<Vec<f64>>,
    pending: Option<Vec<f64>>,
    stats: Vec<PassStat>,
}

thread_local! {
    static CONFIG: RefCell<Config> = const {
        RefCell::new(Config { mode: Mode::Off, rule: Rule::Raw, relax: 1.0 })
    };
    static STATE: RefCell<State> = RefCell::new(State::default());
}

/// This thread's mode and rule; the carried drops cleared.
pub fn set_config(config: Config) {
    CONFIG.with(|c| *c.borrow_mut() = config);
    reset();
}

/// Every carried drop and statistic cleared — a run's start.
pub fn reset() {
    STATE.with(|s| *s.borrow_mut() = State::default());
    PROBE_STAGE_PRESSURES.with(|p| *p.borrow_mut() = None);
}

/// The tick ended: its last pass's drops are what the next tick's `Lag`
/// stages stand at.
pub fn end_tick() {
    STATE.with(|s| {
        let mut s = s.borrow_mut();
        if let Some(pending) = s.pending.take() {
            s.lag = Some(pending);
        }
    });
}

/// The next pass starts from these drops, as if a last tick had left them —
/// a cold start straight at a pressure profile.
pub fn carry(drops: Vec<f64>) {
    STATE.with(|s| s.borrow_mut().lag = Some(drops));
}

/// The passes since the last call.
pub fn take_stats() -> Vec<PassStat> {
    STATE.with(|s| std::mem::take(&mut s.borrow_mut().stats))
}

fn pressures(top: f64, stages: usize, drops: &[f64]) -> Vec<f64> {
    let mut p = vec![top; stages];
    for j in 0..stages.saturating_sub(1) {
        p[j + 1] = p[j] + drops.get(j).copied().unwrap_or(0.0);
    }
    p
}

fn summation(
    pass: &ColumnPass<'_>,
    thermo: &dyn ThermoModel,
    profile: &CascadeProfile,
    pressure_at: &dyn Fn(usize) -> f64,
) -> Result<f64, SimError> {
    let mut worst: f64 = 0.0;
    for (j, (t, x)) in profile.temperatures.iter().zip(&profile.liquid).enumerate() {
        let mut sum = 0.0;
        for (c, xc) in x.iter().enumerate() {
            sum += thermo.k_value(pass.slate, c, *t, Pascal(pressure_at(j)))? * xc;
        }
        worst = worst.max((sum - 1.0).abs());
    }
    Ok(worst)
}

fn solve_at(
    pass: &ColumnPass<'_>,
    stages_pressure: Vec<f64>,
    inner: &dyn Fn(&ColumnPass<'_>) -> Result<Separation, SimError>,
) -> Result<(Separation, Vec<f64>), SimError> {
    PROBE_STAGE_PRESSURES.with(|p| *p.borrow_mut() = Some(stages_pressure));
    let result = inner(pass);
    PROBE_STAGE_PRESSURES.with(|p| *p.borrow_mut() = None);
    let separation = result?;
    let capped = PROBE_CAPPED.with(|c| c.borrow().clone());
    Ok((separation, capped))
}

pub(crate) fn fractionate(
    pass: &ColumnPass<'_>,
    thermo: &dyn ThermoModel,
    inner: impl Fn(&ColumnPass<'_>) -> Result<Separation, SimError>,
) -> Result<Separation, SimError> {
    let Some(spec) = pass.cascade.filter(|s| !s.trays.is_empty()) else {
        return inner(pass);
    };
    let config = CONFIG.with(|c| *c.borrow());
    let stages = spec.stages as usize;
    let trays = stages - 1;
    let top = pass.pressure.value();
    let zeros = vec![0.0; trays];
    let carried = STATE.with(|s| s.borrow().lag.clone()).filter(|d| d.len() == trays);

    let mut used = match config.mode {
        Mode::Off => zeros.clone(),
        Mode::Lag | Mode::Solve => carried.unwrap_or_else(|| zeros.clone()),
    };
    let mut seed: Option<CascadeProfile> = None;
    let mut solves = 0;
    loop {
        solves += 1;
        let at = ColumnPass {
            seed: seed.as_ref().or(pass.seed),
            ..*pass
        };
        let (separation, capped) = solve_at(&at, pressures(top, stages, &used), &inner)?;
        let raw: Vec<f64> = separation.trays.iter().map(|t| t.pressure_drop.value()).collect();
        let read = match config.rule {
            Rule::Raw => raw.clone(),
            Rule::Cap => capped.clone(),
        };
        let change = read
            .iter()
            .zip(&used)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        let settled = config.mode != Mode::Solve || change < TOLERANCE_PA;
        if settled {
            let (at_stage, at_top) = match &separation.profile {
                Some(profile) => {
                    let p = pressures(top, stages, &used);
                    (
                        summation(pass, thermo, profile, &|j| p[j])?,
                        summation(pass, thermo, profile, &|_| top)?,
                    )
                }
                None => (f64::NAN, f64::NAN),
            };
            STATE.with(|s| {
                let mut s = s.borrow_mut();
                s.pending = Some(read.clone());
                s.stats.push(PassStat {
                    solves,
                    used: used.clone(),
                    raw,
                    capped,
                    summation_at_stage: at_stage,
                    summation_at_top: at_top,
                    change,
                });
            });
            return Ok(separation);
        }
        if solves >= MAX_SOLVES {
            return Err(SimError::Numerical(format!(
                "M73.2.0 probe: the drops still moved {change:.3} Pa after {solves} solves"
            )));
        }
        for (u, r) in used.iter_mut().zip(&read) {
            *u += config.relax * (r - *u);
        }
        seed = separation.profile.clone();
    }
}
