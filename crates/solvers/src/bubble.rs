//! The bubble TEMPERATURE of a liquid mixture: `Σ_c K_c(T)·x_c = 1` solved for
//! `T` (M9.3a's root find, promoted out of `cascade.rs` in M12.1).
//!
//! **Promoted rather than duplicated, and rather than added to `ThermoModel`**
//! (docs/DESIGN.md §14 fork 5). A second bracket somewhere else in the workspace
//! would be two notions of one number; a fourth trait method would put an
//! ITERATION behind a trait whose other three members are closed forms, and
//! invite an implementation with a different bracket. This is already the
//! workspace's one answer to the question, already measured and already bounded,
//! so the boil-off model reaches the same function the cascade does.
//!
//! It has two callers with two different needs, which is why it has two doors:
//! the cascade iterates bare `Vec<f64>` stage profiles and calls the
//! crate-private one, and everybody else goes through `bubble_temperature`,
//! whose `MoleFractions` argument makes handing it MASS fractions fail to
//! compile. Normalising at the typed door would have been the tidier promotion
//! and it would have moved the cascade's arithmetic — a re-division by a sum
//! that is 1 only to within a rounding error — which is a regression anchor on
//! sixteen plants.

use refinery_core::components::Slate;
use refinery_core::error::SimError;
use refinery_core::traits::ThermoModel;
use refinery_core::units::{Kelvin, Pascal};

use crate::molar::MoleFractions;

/// The temperature at which a liquid of mole-fraction composition `x` boils at
/// `pressure`, for callers outside this crate's separation internals.
///
/// `what` names the subject in any error message — "the feed", "stage 3 of 8",
/// "tank 'naphtha_tank'" — and is the only thing that tells one caller's failure
/// from another's.
///
/// **The typed argument is the point.** `MoleFractions` is a different type from
/// `Composition` precisely so that handing a mass-fraction vector to a molar
/// equilibrium does not typecheck (`molar.rs`), and a bare `&[f64]` here would
/// re-open that door for every future caller. `Σ K·x` on mass fractions is
/// finite, deterministic, plausible and wrong.
///
/// # Errors
/// As `bubble_temperature_unnormalized`: no root in the bracket (which is where a
/// model with no temperature dependence lands, by construction), or a `K` the
/// model cannot produce.
pub fn bubble_temperature(
    slate: &Slate,
    thermo: &dyn ThermoModel,
    x: &MoleFractions,
    pressure: Pascal,
    what: &str,
) -> Result<Kelvin, SimError> {
    bubble_temperature_unnormalized(slate, thermo, x.fractions(), pressure, what)
}

/// The bracket every stage's bubble point is sought in [K], and the relative width
/// the search closes it to.
///
/// Wide on purpose: a cut's boiling point is slate data and a column's pressure is
/// operator config, so the bubble point of a mixture is not bounded by anything
/// this module knows.
///
/// **The resolution is the one thing here that may NOT be loosened, and the
/// ledger row this slice strikes said the opposite.** Through M9.2 this bracket
/// was halved a fixed 60 times, and `DEFERRED.md` row A2 read that as ~38 wasted
/// halvings against a convergence test of `1e-6` relative — "22 steps would meet
/// it". They would not. The cascade's outer test differences two bubble points
/// (`|T − T'|/T' ≤ tolerance`), so a search that resolves `T` only to a relative
/// width `w` gives that difference a noise floor of `w`: two calls at
/// nearly-identical compositions land in different sub-intervals and differ by
/// `w` for no physical reason. The outer loop therefore needs `w ≪ tolerance`,
/// not `w ≈ tolerance` — at `w = tolerance` the column converges by luck and at
/// `w` a little above it the column `Err`s. Closing the bracket to the float
/// spacing keeps `w` at ~1e-16 relative, which is what makes the outer
/// tolerance mean what it says.
///
/// So M9.3a took the cost out of the METHOD instead: the same resolution, reached
/// by a bracketed superlinear search in 15 evaluations rather than 60 halvings.
const BUBBLE_POINT_LOW_K: f64 = 50.0;
const BUBBLE_POINT_HIGH_K: f64 = 2000.0;
const BUBBLE_POINT_RESOLUTION: f64 = 2.0 * f64::EPSILON;

/// Evaluations of `Σ K·x − 1` one bubble point may spend — a STRUCTURAL bound,
/// not the working number.
///
/// The search bisects whenever the previous step failed to halve the bracket, so
/// the width halves at least once every two evaluations whatever the model does.
/// `[50, 2000]` is 8.8e16 times `2·ε·T` at the bottom of the bracket, so 57
/// halvings close it from anywhere in it and 114 evaluations always reach the
/// resolution: the loop exits on the bracket width and never on this budget. It
/// is here so a pathological model is bounded by construction, the way the old
/// fixed 60-step loop was, and it is twice that only in the case that cannot
/// happen.
const BUBBLE_POINT_MAX_EVALUATIONS: u32 = 120;

/// The temperature at which a liquid of composition `x` boils at `pressure`:
/// `Σ_c K_c(T)·x_c = 1`. `x` is MOLE fractions, already normalised — the door
/// the cascade's own iterate comes through, which is why it is a bare slice and
/// why it is crate-private. Every caller outside `cascade.rs` uses
/// `bubble_temperature`.
///
/// A bracketed search on a wide fixed bracket, and **no root in the bracket is an
/// `Err`**
/// naming the model — never a fallback temperature. That refusal is what makes
/// this function safe to hand an arbitrary `ThermoModel`, and it is the arm a
/// K-value with no temperature dependence at all lands in: `Σ K·x` is then a
/// constant, so the equation has no solution unless the constant happens to be 1.
/// The alternative — detect a flat model and hold some convention — cannot
/// distinguish "K does not depend on T" from "the root is outside my bracket",
/// and the second case is the finite-deterministic-plausible-wrong shape this
/// workspace keeps catching.
pub(crate) fn bubble_temperature_unnormalized(
    slate: &Slate,
    thermo: &dyn ThermoModel,
    x: &[f64],
    pressure: Pascal,
    what: &str,
) -> Result<Kelvin, SimError> {
    // The search's function is `ln Σ K·x`, and the logarithm is the whole of the
    // speed-up. The safeguard below is here for a WORST case, not an average one.
    //
    // The root is where `Σ K·x = 1`, so the logarithm has the same root, the same
    // sign either side of it (`Σ K·x > 0` always) and the same monotonicity, but a
    // very different SHAPE. A K-value is exponential-ish in temperature for any
    // model worth the name: Clausius−Clapeyron gives `ln K ~ A − B/T`, and
    // this workspace's test model is a power law. Over a bracket as wide as
    // `[50, 2000]` K that means `Σ K·x` itself spans SEVEN orders of magnitude
    // (~1e-9 to ~1e7 on the M7.3 fixture) and THIRTY-EIGHT on the shipped
    // `crude_column_cascade` slate, whose endpoints were measured at
    // `ln Σ K·x = -81.8` and `+6.85` — far apart, and both comfortably finite,
    // so the underflow arm below is a guard rather than the usual case. A
    // secant through two points of a function that steep lands almost on top of
    // the low endpoint every time, which is what regula falsi is famous for
    // crawling on.
    //
    // Evaluations to the same bracket width on that fixture, every way round
    // rather than argued — the two knobs swept independently, because the first
    // draft of this comment asserted a table from memory and every cell of it
    // was wrong:
    //
    // ```text
    //                                  Σ K·x − 1      ln Σ K·x
    //   no bisection safeguard               54              15
    //   bisect if ONE step failed
    //   to halve the bracket                 38              16
    //   bisect if TWO steps failed
    //   to halve the bracket                 65              15
    // ```
    //
    // Plain bisection needs 55 and the loop this replaced always spent 60. Read
    // the columns, not the cells: WITH the logarithm every safeguard costs 15 or
    // 16, and without it none costs less than 38. The transform is the change;
    // the safeguard is worth at most one evaluation and on this fixture is worth
    // none. Anyone tuning the safeguard for speed is tuning the wrong knob.
    //
    // So why keep it? Because the cell that matters for the safeguard is not in
    // this table. A secant with no bisection safeguard has NO bound on how many
    // steps it can take — regula falsi retains one endpoint forever on a convex
    // function, and 15 is what it happens to cost on this fixture, not what it
    // is guaranteed to cost on a slate nobody has run yet. The two-step test is
    // what makes `BUBBLE_POINT_MAX_EVALUATIONS` a structural bound rather than a
    // hopeful one, and that bound is why this function can return an answer
    // instead of an `Err` on a plant it has never seen. It buys a guarantee,
    // not a number.
    let ln_excess = |t: f64| -> Result<f64, SimError> {
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
        // Every K is positive and finite and every `x` is non-negative, so the
        // sum is too; it can still UNDERFLOW to zero far below the root, where
        // `ln` gives negative infinity. That is a correct sign for the bracket,
        // and the secant guard below rejects a non-finite interpolation, so the
        // search bisects its way out rather than stepping on an infinity.
        Ok(sum.ln())
    };

    let low = ln_excess(BUBBLE_POINT_LOW_K)?;
    let high = ln_excess(BUBBLE_POINT_HIGH_K)?;
    if low > 0.0 || high < 0.0 {
        return Err(SimError::Numerical(format!(
            "{what} has no bubble point between {BUBBLE_POINT_LOW_K} K and \
             {BUBBLE_POINT_HIGH_K} K at {} Pa under thermo model '{}': ln Σ K·x is \
             {low:.4e} at the bottom of the bracket and {high:.4e} at the top, so Σ K·x \
             never crosses 1. A model whose K-values do not depend on temperature lands here by \
             construction, and it cannot drive a cascade: a stage's temperature IS its bubble \
             point.",
            pressure.value(),
            thermo.name()
        )));
    }

    // Regula falsi with Illinois weighting and a bisection safeguard: the secant
    // through the bracket's two endpoints, with the RETAINED endpoint's value
    // halved whenever it has been retained twice running (Illinois; Dowell &
    // Jarratt, *BIT* 11 (1971) 168), and a bisection whenever the bracket has
    // not halved over the last TWO steps (Brent's progress test, *Algorithms
    // for Minimization without Derivatives*, 1973, ch. 4). 15 evaluations on
    // the M7.3 fixture, against the 60 the fixed halving loop always spent.
    // reached the same bracket width.
    //
    // **Bracket-preserving, and that is a requirement rather than a preference.**
    // This function is contracted to be safe handed an ARBITRARY `ThermoModel`:
    // `Σ K·x` is monotone in T for every model in this workspace, but nothing in
    // the trait says so, and an unsafeguarded secant on a non-monotone excess can
    // step outside and return a temperature that is finite, deterministic and
    // wrong — the shape rule 5 exists to refuse. Every step below keeps
    // `low_excess ≤ 0 ≤ high_excess`, so the root the two endpoint evaluations
    // proved is still bracketed when the loop ends.
    let (mut low_t, mut high_t) = (BUBBLE_POINT_LOW_K, BUBBLE_POINT_HIGH_K);
    let (mut low_excess, mut high_excess) = (low, high);
    // Which endpoint moved on the previous step: `-1` the low one, `1` the high
    // one, `0` on the first step. This is what makes the weighting Illinois's
    // rather than plain false position, and it is the whole of the acceleration.
    let mut moved_last = 0i32;
    // The bracket two steps ago, which is what the progress test below is
    // measured against.
    let mut width_two_ago = high_t - low_t;
    let mut width_one_ago = high_t - low_t;
    for _ in 0..BUBBLE_POINT_MAX_EVALUATIONS {
        let width = high_t - low_t;
        if width <= BUBBLE_POINT_RESOLUTION * high_t {
            break;
        }
        // Brent's progress test, over TWO steps rather than one.
        //
        // The one-step version is the obvious one and it costs most of the
        // win: it turns roughly every other evaluation into a bisection,
        // because a secant step that shrinks the bracket to 0.55 of itself —
        // real progress — fails it. See the table above the search's
        // function for what each combination actually costs; this one is 15
        // and the one-step version of the same code is 36.
        //
        // The guarantee survives the relaxation: whenever the width has not
        // halved over the last two steps a bisection is forced, so it halves
        // at worst once per two evaluations and 114 of them close this
        // bracket from anywhere in it.
        let force_bisection = width > 0.5 * width_two_ago;

        let secant = (low_t * high_excess - high_t * low_excess) / (high_excess - low_excess);
        // The secant is used only where it is strictly inside the bracket; a
        // non-finite one (two equal excesses) and one that lands on an endpoint
        // both fall back to the midpoint, which always shrinks the bracket.
        //
        // The NEGATED conjunction is load-bearing and must not be "simplified"
        // to `secant <= low_t || secant >= high_t`. Those read as the same
        // condition and are not: a NaN secant compares false against both
        // bounds, so the negated form falls to bisection while the rewritten
        // form takes the secant branch and writes a NaN into the bracket.
        let middle = if force_bisection || !(secant > low_t && secant < high_t) {
            0.5 * (low_t + high_t)
        } else {
            secant
        };

        // Evaluated once and reused: this call IS the cost this slice is about,
        // and the old loop's one-evaluation-per-step budget is the thing being
        // beaten rather than matched.
        let middle_excess = ln_excess(middle)?;
        if middle_excess <= 0.0 {
            low_t = middle;
            low_excess = middle_excess;
            if moved_last == -1 {
                high_excess *= 0.5;
            }
            moved_last = -1;
        } else {
            high_t = middle;
            high_excess = middle_excess;
            if moved_last == 1 {
                low_excess *= 0.5;
            }
            moved_last = 1;
        }

        width_two_ago = width_one_ago;
        width_one_ago = width;
    }
    Ok(Kelvin(0.5 * (low_t + high_t)))
}
