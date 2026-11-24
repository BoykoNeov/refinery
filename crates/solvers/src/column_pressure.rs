//! A column's top pressure each tick (M89.0, docs/DESIGN.md §86): the
//! implementations of `[fidelity] column_pressure`, the
//! [`ColumnPressureModel`] seam. Each searches the column's steps at trial
//! top pressures ([`PressureTrials`]) for the tick's.
//!
//! - [`HeldPressure`] (`"held"`): the column pinned at its set pressure,
//!   one step, whatever its condenser can do.
//! - [`CondenserLimitPressure`] (`"limit"`, M75.1, docs/DESIGN.md §80.2):
//!   the flooded condenser holding the set pressure while it condenses what
//!   arrives; past it the pressure rising, backward Euler on the overhead's
//!   balance, solved by regula falsi (Illinois) on a bracket grown from
//!   where the pressure stands. Moved here from `heat_balance` unchanged.

use refinery_core::error::SimError;
use refinery_core::traits::{
    ColumnPressureLevel, ColumnPressureModel, PressureBalance, PressureTrials,
};
use refinery_core::units::Pascal;

/// `"held"`: the column stands at its set pressure every tick, its
/// condenser's capacity read for its faceplate only (M89.0, docs/DESIGN.md
/// §86).
#[derive(Debug, Clone, Copy, Default)]
pub struct HeldPressure;

impl ColumnPressureModel for HeldPressure {
    fn level(&self) -> ColumnPressureLevel {
        ColumnPressureLevel::Held
    }

    fn top_pressure(&self, trials: &mut dyn PressureTrials) -> Result<Pascal, SimError> {
        let set = trials.set_pressure();
        trials.trial(set)?;
        Ok(set)
    }
}

/// `"limit"`: the condenser a limit (M75.1–M75.4, docs/DESIGN.md §80.2,
/// §80.7). The tick's top pressure `P` solves
///
/// `r(P) = V·λ/(R·T·dt)·(P − P₀) − (Q(P) − C(P)) = 0`
///
/// on `[set, upper]`, `r` rising with `P`. The first trial at last tick's
/// pressure (clamped to the bracket): at the set pressure with the condenser
/// coping (`r ≥ 0`) that is the answer — the flooded condenser holding it,
/// one step, as on every column without the limit. Else the bracket grows
/// from there toward the end the residual points at, never a whole step at
/// the far end, and regula falsi narrows it.
#[derive(Debug, Clone, Copy, Default)]
pub struct CondenserLimitPressure;

impl ColumnPressureModel for CondenserLimitPressure {
    fn level(&self) -> ColumnPressureLevel {
        ColumnPressureLevel::Limit
    }

    fn top_pressure(&self, trials: &mut dyn PressureTrials) -> Result<Pascal, SimError> {
        let set = trials.set_pressure().value();
        let upper = trials.upper().value();
        let tick = trials.tick().value();
        let start = trials.previous().value().clamp(set, upper);
        let first = trials.trial(Pascal(start))?;
        let residual = first.residual.value();
        if (residual >= 0.0 && start <= set) || residual == 0.0 {
            // At the set pressure, the condenser condensing what arrives: its
            // control floods the tubes it does not need.
            return Ok(Pascal(start));
        }
        // The bracket grown from where the pressure stands, never a whole
        // step at its far end: the root lies within `|r|/store` of the start
        // (the residual's slope is at least the store's — the duty falls and
        // the capacity rises with `P`), capped at about the most a tick
        // moves, doubled until it brackets, clamped to [set, design]. A step
        // tried at the far end of a pressure at its peak (2.39 bar to the set
        // 1.8 in one tick) flashed the column and refused the tick
        // (docs/DESIGN.md §80.2).
        let rising = residual < 0.0;
        let (end, toward) = if rising { (upper, 1.0) } else { (set, -1.0) };
        let mut distance = (residual.abs() / first.store.value())
            .min(BRACKET_RATE_PA_PER_S * tick)
            .max(PRESSURE_TOLERANCE);
        let mut near = (start, first);
        let mut retreats = 0;
        loop {
            let pressure = if rising {
                (start + toward * distance).min(end)
            } else {
                (start + toward * distance).max(end)
            };
            // A trial whose step does not converge is a search point, not
            // the tick's answer: the bracket retreats halfway back to the
            // last trial that converged (docs/DESIGN.md §80.7) — a column
            // whose stage is near to stopping boiling can stall Newton at one
            // pressure and settle a few hundred pascals nearer. Refused only
            // when the retreat reaches that trial, or after
            // `MAX_PRESSURE_SOLVES` retreats. A tick none of whose trials
            // fails takes exactly the path it took before.
            let trial = match trials.trial(Pascal(pressure)) {
                Ok(trial) => trial,
                Err(failed) => {
                    retreats += 1;
                    if retreats > MAX_PRESSURE_SOLVES
                        || (pressure - near.0).abs() <= PRESSURE_TOLERANCE
                    {
                        return Err(failed);
                    }
                    distance = 0.5 * ((pressure - start).abs() + (near.0 - start).abs());
                    continue;
                }
            };
            let r = trial.residual.value();
            let beyond = if rising { r > 0.0 } else { r < 0.0 };
            if r == 0.0 {
                return Ok(Pascal(pressure));
            } else if beyond {
                return illinois(trials, near, (pressure, trial)).map(Pascal);
            } else if pressure == end {
                // The design pressure, the most it rises to (with a relief,
                // its ceiling, the alarm); or the set, the condenser taking
                // the pressure back.
                return Ok(Pascal(pressure));
            }
            near = (pressure, trial);
            distance *= 2.0;
        }
    }
}

/// The most trial steps a column's top pressure takes within a tick
/// (M75.1, docs/DESIGN.md §80.1) before the tick is refused.
const MAX_PRESSURE_SOLVES: u32 = 40;

/// The top pressure's tolerance within a tick [Pa] (M75.1): the bracket,
/// or the last move, narrower than this.
const PRESSURE_TOLERANCE: f64 = 1.0;

/// The most the bracket's first trial moves the top pressure a second
/// [Pa/s] (docs/DESIGN.md §80.2): about three times M75.0's hand figure, 0.3
/// bar/s on a 30 MW shortfall into 100 m³ — 10 kPa a tick of 0.1 s. Past it
/// the bracket doubles.
const BRACKET_RATE_PA_PER_S: f64 = 1.0e5;

/// A trial pressure [Pa] and its balance.
type PressureTrial = (f64, PressureBalance);

/// Regula falsi with the Illinois modification on a bracketing pair — one
/// end's residual below zero, the other's above, in either order — for the
/// top pressure (M75.1, docs/DESIGN.md §80.1). Deterministic, and never
/// leaves the bracket. Returns the pressure [Pa] of its last trial.
fn illinois(
    trials: &mut dyn PressureTrials,
    one: PressureTrial,
    other: PressureTrial,
) -> Result<f64, SimError> {
    let (low, high) = if one.1.residual.value() < 0.0 {
        (one, other)
    } else {
        (other, one)
    };
    let (mut a, mut ra) = (low.0, low.1.residual.value());
    let (mut b, mut rb) = (high.0, high.1.residual.value());
    let mut side = 0i8;
    let mut last = f64::NAN;
    for _ in 0..MAX_PRESSURE_SOLVES {
        let falsi = (a * rb - b * ra) / (rb - ra);
        if !falsi.is_finite() {
            break;
        }
        // A trial whose step does not converge is a search point, not the
        // tick's answer (docs/DESIGN.md §80.7): the search retreats from it
        // halfway towards each end of the bracket in turn — both ends
        // converged — until a trial does, the bracket then narrowed by it as
        // by any other. Refused when both retreats reach their ends. A solve
        // none of whose trials fails takes exactly the path it took before.
        let (pressure, trial) = match trials.trial(Pascal(falsi)) {
            Ok(trial) => (falsi, trial),
            Err(failed) => {
                let (mut toward_a, mut toward_b) = (falsi, falsi);
                let mut found = None;
                for _ in 0..MAX_PRESSURE_SOLVES {
                    toward_a = 0.5 * (a + toward_a);
                    let a_open = (toward_a - a).abs() > PRESSURE_TOLERANCE;
                    if a_open {
                        if let Ok(trial) = trials.trial(Pascal(toward_a)) {
                            found = Some((toward_a, trial));
                            break;
                        }
                    }
                    toward_b = 0.5 * (toward_b + b);
                    let b_open = (toward_b - b).abs() > PRESSURE_TOLERANCE;
                    if b_open {
                        if let Ok(trial) = trials.trial(Pascal(toward_b)) {
                            found = Some((toward_b, trial));
                            break;
                        }
                    }
                    if !a_open && !b_open {
                        break;
                    }
                }
                match found {
                    Some(found) => {
                        side = 0;
                        found
                    }
                    None => return Err(failed),
                }
            }
        };
        let residual = trial.residual.value();
        if residual == 0.0
            || (pressure - last).abs() < PRESSURE_TOLERANCE
            || b - a < PRESSURE_TOLERANCE
        {
            return Ok(pressure);
        }
        last = pressure;
        if residual < 0.0 {
            a = pressure;
            ra = residual;
            if side == -1 {
                rb *= 0.5;
            }
            side = -1;
        } else {
            b = pressure;
            rb = residual;
            if side == 1 {
                ra *= 0.5;
            }
            side = 1;
        }
    }
    Err(SimError::Numerical(format!(
        "the column's top pressure did not settle within {MAX_PRESSURE_SOLVES} trials \
         between {:.4} and {:.4} bar: its condenser's balance read {ra:.4e} and {rb:.4e} W \
         at the two (docs/DESIGN.md §80.1).",
        a / 1.0e5,
        b / 1.0e5,
    )))
}
