//! Property tests for `elements::fold_gas_valve`'s inner solve — the SCALAR
//! half of the box M5.4 left open.
//!
//! M5.4c deferred this with the reason stated: every M5.4 gate is a fixed
//! plant, so the bisection had "never met adversarial inputs — `p_up` at the
//! 1 Pa floor against a large drop, `x_choke` near zero, `α_pipe` orders away
//! from `α_valve`. Those are argued finite ... but argued is not tried."
//!
//! **What this file does NOT buy, measured rather than assumed.** The
//! ROADMAP put this box on the I5 generator "because I5 is what would catch a
//! bad inner solve: Newton and Simple would disagree". That premise is false
//! and one mutation shows it: both fidelities consume the same
//! `QuadraticBranch` from the same per-iterate `compile_edges`, so a degraded
//! bisection moves `α(dp)` identically for both and they converge to the same
//! wrong root together. Nor is "no gate sees the bisection" true — dropping
//! `GAS_VALVE_BISECTIONS` from 60 fails `elements`'
//! `the_folded_branch_reproduces_the_inner_solve` all the way up to **24**
//! halvings, and goes green at **32**. That gate has real discriminating
//! power; what it does not have is BREADTH. It runs one `α_pipe`, one
//! `α_valve`, one `p_up` and one `x_choke` over six drops.
//!
//! So the claim here is coverage of the PARAMETER SPACE, not of the bisection
//! depth: the bracket-and-monotonicity argument in `fold_gas_valve`'s doc is
//! what has never been tried, and the places it could fail are exactly the
//! extremes — `Q_gas` overflowing at tiny `α_liquid`, `s/p_up` blowing up at
//! the pressure floor, `α_eff` going non-finite at `x_choke` near zero.
//!
//! Four properties, over that space:
//!   G1. **Well-formed.** A finite, positive valve coefficient folds to a
//!       finite, positive branch, and the fold adds no pressure offset of its
//!       own. An infinite (closed) valve stays infinite.
//!   G2. **The returned coefficient really solves the series**, against a root
//!       resolved INDEPENDENTLY — by a bisection in log `s` rather than in
//!       `s`, which is the one difference that matters here (below).
//!   G3. **Compressibility never helps.** `α_eff ≥ α_liquid` everywhere, so
//!       the gas branch is never less resistive than its incompressible self.
//!       The one-line statement of "the incompressible law overpredicts",
//!       as an inequality over the whole space instead of at one plant.
//!   G4. **Odd about `β`.** `x` is taken from `|dp − β|`, so reversing the
//!       drop about the pipe's static head must return the SAME coefficient,
//!       bit for bit (DESIGN §3a fork 6).

use proptest::prelude::*;
use proptest::strategy::{Strategy, ValueTree};
use proptest::test_runner::TestRunner;
use refinery_solvers::elements::{expansion, fold_gas_valve, gas_valve_flow, QuadraticBranch};
use refinery_solvers::network::RHO_EVAL_P_FLOOR;

/// One generated fold: (pipe, valve, dp, p_up, x_choke, blend, s_total).
#[derive(Debug, Clone, Copy)]
struct Fold {
    pipe: QuadraticBranch,
    valve: QuadraticBranch,
    dp: f64,
    p_up: f64,
    x_choke: f64,
    blend: f64,
    /// `|dp − β|`, the folded branch drop the inner solve must split.
    s_total: f64,
}

/// Log-uniform over `[10^lo, 10^hi]` — the right measure for a coefficient
/// that spans decades, and what makes "`α_pipe` orders away from `α_valve`"
/// the common case rather than a tail event.
fn log_uniform(lo: f64, hi: f64) -> impl Strategy<Value = f64> {
    (lo..hi).prop_map(|e| 10f64.powf(e))
}

/// The adversarial parameter space, with every range derived from what the
/// engine can actually build rather than picked for looking extreme.
///
/// * `α_valve = ρ_rel/cv_eff²`. A gas at 1 Pa..100 bar gives `ρ_rel` in
///   1e-8..0.1; `cv_eff` runs from a large valve wide open (~1e-3) down to
///   `OPEN_EPS` times a small one (~1e-12), since below `OPEN_EPS` the
///   opening is snapped to zero and the closed branch is taken instead. That
///   is 1e-2..1e23, and both ends are reachable.
/// * `α_pipe = f·L·ρ/(2·D·A²)` over the loader's ranges is ~1e-5..1e9;
///   1e-6..1e10 brackets it.
/// * `p_up` starts at `RHO_EVAL_P_FLOOR` — the clamp `compile_edge` applies,
///   so it is exactly the smallest value the fold can be handed.
/// * `x_choke = F_k·x_T ≤ ~1.2`; the low end is the "near zero" the deferral
///   named.
/// * The drop is generated as a MULTIPLE of `x_choke·p_up`, so `s_total` is
///   controlled exactly and the sample spans deeply unchoked (1e-6) to
///   deeply choked (50) rather than landing wherever the pressures fall.
fn fold_strategy() -> impl Strategy<Value = Fold> {
    (
        log_uniform(-6.0, 10.0),                                        // alpha_pipe
        log_uniform(-2.0, 23.0),                                        // alpha_valve
        -1.0e4..1.0e4f64,                                               // beta (pipe static head)
        (RHO_EVAL_P_FLOOR.log10()..7.0f64).prop_map(|e| 10f64.powf(e)), // p_up
        log_uniform(-9.0, 0.08),                                        // x_choke
        log_uniform(-6.0, 1.7),                                         // x / x_choke
        any::<bool>(),                                                  // forward or reverse
        prop_oneof![3 => Just(0.0f64), 1 => 0.0..0.3f64],               // blend
    )
        .prop_map(
            |(alpha_pipe, alpha_valve, beta, p_up, x_choke, x_rel, forward, blend)| {
                let target = x_rel * x_choke * p_up;
                let dp = if forward {
                    beta + target
                } else {
                    beta - target
                };
                // The drop the FOLD will see, `(dp − β).abs()`, recomputed here
                // rather than carried over from `target`. Forming `dp` rounds,
                // and for a small target against a large `β` it rounds hard —
                // at `β = 8e4` and `target = 4e-11` the two differ by 20%. The
                // reference root must be found for the same equation the fold
                // is solving, so the bookkeeping follows the arithmetic instead
                // of the intent.
                let s_total = (dp - beta).abs();
                Fold {
                    pipe: QuadraticBranch {
                        alpha: alpha_pipe,
                        beta,
                    },
                    valve: QuadraticBranch {
                        alpha: alpha_valve,
                        beta: 0.0,
                    },
                    dp,
                    p_up,
                    x_choke,
                    blend,
                    s_total,
                }
            },
        )
}

/// The valve's share of the drop, resolved independently of the shipped
/// scheme by bisecting in **log `s`** instead of in `s`.
///
/// Same equation — `g(s) = s + α_pipe·Q_gas(s)²`, evaluated through the
/// production `gas_valve_flow`, because the equation IS the specification and
/// re-typing it would only test the retyping. A different *root-finder*,
/// though, and that difference is the whole point: `fold_gas_valve` halves
/// `[0, s_total]`, so after 60 steps it knows the root to an ABSOLUTE
/// `s_total·2⁻⁶⁰`. Its doc calls that "exact rather than tight enough", which
/// is true in absolute terms and false in relative ones — when `α_pipe ≫
/// α_valve` the root sits many decades below `s_total` (at the ratio 7e11 this
/// strategy reaches, `s/s_total ≈ 1e-12`), and an absolute bracket resolves it
/// to only ~1e-6 relative. Halving in log space is uniform in relative terms
/// and so has no such blind spot.
///
/// This is the M4.2 order-of-convergence move: a second scheme for the same
/// quantity, so what is compared is two answers rather than one answer with
/// itself.
fn reference_share(f: &Fold) -> f64 {
    let g = |s: f64| {
        s + f.pipe.alpha * gas_valve_flow(f.valve.alpha, s, f.p_up, f.x_choke, f.blend).powi(2)
    };
    // `g` is continuous and strictly increasing with `g(0) = 0` (as `s → 0`
    // the valve is always UNCHOKED, so `Q_gas → 0`), and `g(s_total) ≥
    // s_total`. 25 decades below the drop is therefore a safe lower bracket
    // for any `α_pipe/α_valve` this strategy can build (≤ ~1e12), and the
    // bracket is asserted rather than assumed.
    let (mut lo, mut hi) = ((f.s_total * 1e-25).log2(), f.s_total.log2());
    assert!(
        g(lo.exp2()) < f.s_total,
        "log bracket lost: g({}) >= s_total {}",
        lo.exp2(),
        f.s_total
    );
    // 80 halvings of an 83-octave interval resolves `s` to ~1e-22 relative,
    // six orders below double precision, so the reference is exact for this
    // comparison's purposes.
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        if g(mid.exp2()) < f.s_total {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    (0.5 * (lo + hi)).exp2()
}

/// The branch coefficient the reference root implies.
fn reference_alpha(f: &Fold) -> f64 {
    let s = reference_share(f);
    let (x_s, y) = expansion(s / f.p_up, f.x_choke, f.blend);
    let ratio = if x_s > 0.0 { s / (x_s * f.p_up) } else { 1.0 };
    f.pipe.alpha + f.valve.alpha * ratio / (y * y)
}

/// Budget for G2, set from the MEASUREMENT below rather than chosen.
///
/// Measured floor at the shipped 60 halvings: **3.97e-15** over 4000 samples,
/// i.e. float noise. The budget sits ~2500× above that and ~23× below the
/// **1.16e-10** a 32-halving scheme produces, so it is derived from both ends
/// rather than from either alone.
///
/// Sweeping the count confirms the deviation is the SCHEME and not the
/// arithmetic — 4.56e-13 at 40 halvings, 1.16e-10 at 32, 2.98e-8 at 24, a
/// factor ~2⁸ per 8 halvings, which is bisection's own rate. It also places
/// this gate against the one that already exists: `elements`'
/// `the_folded_branch_reproduces_the_inner_solve` catches a degradation to 24
/// and goes green at 32; this one catches 32 and goes green at 40. The margin
/// on the shipped constant is therefore extended, not duplicated.
///
/// Note what this compares and why it is the honest quantity: the FOLDED
/// coefficient `α_tot`, not the valve's share alone. Downstream readers
/// consume `α_tot`, and where `α_pipe` dominates, an imprecise `α_eff` is
/// genuinely invisible to the physics — a gate that stayed sharp there would
/// be pinning a number nothing reads. The comparison therefore tightens
/// exactly as the valve starts to matter, which is the right sensitivity
/// profile rather than a uniform one.
const ALPHA_BUDGET: f64 = 1e-11;

// ---------------------------------------------------------------------------
// Measurement (the `measure_energy_balance_headroom` pattern): report the real
// floor so the budget above is derived rather than tuned until green.
// ---------------------------------------------------------------------------

#[test]
fn measure_gas_fold_alpha_headroom() {
    const SAMPLES: usize = 4000;
    let mut runner = TestRunner::deterministic();
    let strat = fold_strategy();

    let mut worst = 0.0f64;
    let mut worst_case: Option<Fold> = None;
    let mut choked = 0usize;
    for _ in 0..SAMPLES {
        let f = strat
            .new_tree(&mut runner)
            .expect("strategy produces a value")
            .current();
        if f.s_total <= 0.0 {
            continue;
        }
        // Choked measured on the SHARE the valve actually takes, not on the
        // whole branch drop: `s ≤ s_total`, and the generated `α_pipe` often
        // dominates, so `s_total/p_up ≥ x_choke` is necessary and nowhere near
        // sufficient. Counting the wrong one would report a plateau coverage
        // the samples never reach.
        if reference_share(&f) / f.p_up >= f.x_choke {
            choked += 1;
        }
        let folded = fold_gas_valve(f.pipe, f.valve, f.dp, f.p_up, f.x_choke, f.blend);
        let reference = reference_alpha(&f);
        let r = (folded.alpha - reference).abs() / reference;
        if r > worst {
            worst = r;
            worst_case = Some(f);
        }
    }
    println!("worst relative alpha deviation over {SAMPLES} samples: {worst:.3e}");
    println!("choked samples (by the valve's own share): {choked}/{SAMPLES}");
    println!("worst case: {worst_case:?}");
    assert!(
        worst < ALPHA_BUDGET,
        "measured deviation {worst:.3e} is at or above the {ALPHA_BUDGET:.0e} budget — \
         the budget is no longer above the floor it was derived from"
    );
    // Non-vacuity: the space must actually reach the choked branch, or G2/G3
    // only ever exercise the degenerate `Y → 1` path where the gas fold is the
    // liquid fold and every assertion here would hold for a valve with no
    // compressible correction at all.
    assert!(
        choked * 10 >= SAMPLES,
        "only {choked}/{SAMPLES} samples were choked at the VALVE — the strategy has \
         drifted off the branch this file exists to cover"
    );
}

// ---------------------------------------------------------------------------
// Properties.
// ---------------------------------------------------------------------------

proptest! {
    #![proptest_config(ProptestConfig::with_cases(2000))]

    /// G1 + G2 + G3 on one sample — they share a fold, and splitting them
    /// would triple the work to say the same thing.
    #[test]
    fn the_gas_fold_is_well_formed_and_solves_its_series(f in fold_strategy()) {
        let folded = fold_gas_valve(f.pipe, f.valve, f.dp, f.p_up, f.x_choke, f.blend);

        // G1: finite in, finite out. Nothing in this space may produce a NaN,
        // which is the whole content of "argued finite, not tried".
        prop_assert!(
            folded.alpha.is_finite() && folded.alpha > 0.0,
            "fold produced alpha={} from a finite positive valve", folded.alpha
        );
        prop_assert_eq!(folded.beta, f.pipe.beta, "the valve must add no offset");
        prop_assert!(folded.flow(f.dp, 1e-9).is_finite(), "branch flow must be finite");

        // G3: the compressible correction only ever ADDS resistance.
        // `α_eff = α_valve·(x/x_s)/Y²` with `x ≥ x_s` and `Y ≤ 1`, so both
        // factors are ≥ 1 — and a sign slip in either inverts the inequality.
        let liquid = f.pipe.alpha + f.valve.alpha;
        prop_assert!(
            folded.alpha >= liquid * (1.0 - 1e-12),
            "gas fold {} is LESS resistive than the incompressible fold {liquid}",
            folded.alpha
        );

        // G2: the root condition, against the independently resolved share.
        if f.s_total > 0.0 {
            let reference = reference_alpha(&f);
            let deviation = (folded.alpha - reference).abs() / reference;
            prop_assert!(
                deviation <= ALPHA_BUDGET,
                "folded alpha {} deviates {deviation:.3e} from the log-resolved root's \
                 {reference} — the returned coefficient does not solve g(s) = s_total",
                folded.alpha
            );
        }
    }

    /// G4: the branch is odd about `β`. Reversing the drop about the pipe's
    /// static head must give a bit-identical coefficient — `x` is evaluated
    /// from `|dp − β|` precisely so a gas valve behaves the same in reverse.
    ///
    /// The assumption is not a weakening of the claim, it is what makes the
    /// claim testable at all: `β ± d` are two separate roundings, so for a
    /// large `β` and a small `d` the two calls are handed drops that genuinely
    /// differ in their last bits, and the ~4-ulp disagreement that follows is
    /// the TEST's construction rather than an asymmetry in the fold. Skipping
    /// those samples keeps the assertion exact instead of trading it for a
    /// tolerance that would hide a real asymmetry of the same size.
    #[test]
    fn the_gas_fold_is_odd_about_the_static_head(f in fold_strategy()) {
        let (dp_fwd, dp_rev) = (f.pipe.beta + f.s_total, f.pipe.beta - f.s_total);
        prop_assume!((dp_fwd - f.pipe.beta) == -(dp_rev - f.pipe.beta));
        let forward = fold_gas_valve(f.pipe, f.valve, dp_fwd, f.p_up, f.x_choke, f.blend);
        let reverse = fold_gas_valve(f.pipe, f.valve, dp_rev, f.p_up, f.x_choke, f.blend);
        prop_assert_eq!(
            forward.alpha, reverse.alpha,
            "the fold is not odd about beta: forward {} vs reverse {}",
            forward.alpha, reverse.alpha
        );
    }

    /// A closed valve (`α = +∞`) folds to a closed branch and never to a NaN.
    /// It is the one input that takes `fold_gas_valve`'s early return, and
    /// `∞ · 0` in `α_eff` is exactly what that return exists to avoid — so it
    /// is generated rather than argued.
    #[test]
    fn a_closed_gas_valve_folds_to_a_closed_branch(f in fold_strategy()) {
        let shut = QuadraticBranch { alpha: f64::INFINITY, beta: 0.0 };
        let folded = fold_gas_valve(f.pipe, shut, f.dp, f.p_up, f.x_choke, f.blend);
        prop_assert!(folded.alpha.is_infinite(), "a shut valve must stay shut");
        prop_assert_eq!(folded.flow(f.dp, 1e-9), 0.0, "a shut branch must pass exactly zero");
    }
}
