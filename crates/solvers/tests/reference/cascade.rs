//! Reference cases for the M7.3 equilibrium-stage cascade.
//!
//! **Family 1 of DESIGN §5's three: the cascade ALGEBRA, through a relative
//! volatility supplied by the test.** Every K-value in this file comes from
//! `ConstantAlphaThermo`, so nothing here depends on the Trouton correlation
//! being right — that is `tests/reference/vapour_pressure.rs`'s job, and note
//! correction 4 gives the sharp reason the two must stay apart: the exact K
//! identities are *structurally incapable* of catching a wrong Trouton constant,
//! so a test that mixed the two would police neither.
//!
//! The anchors are derived rather than transcribed: Fenske's relation at total
//! reflux, the Rachford–Rice flash the single-stage case must reduce to, and the
//! null case where nothing can separate. No published table is involved and none
//! is needed.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::graph::{CascadeSpec, ColumnDraw, NodeId};
use refinery_core::traits::{ColumnPass, Separation, SeparationModel, ThermoModel};
use refinery_core::units::{JPerKgK, Kelvin, KgPerM3, KgPerMol, KgPerSec, P_ATM};
use refinery_solvers::{flash_isothermal, ConstantAlphaThermo, MoleFractions, StageCascade};

const T_REF: Kelvin = Kelvin(400.0);

/// The exponent in `K_c(T) = k_c·(T/T_ref)^n`.
///
/// **Any positive `n` leaves `α` untouched**, which is the whole reason the
/// scaling is admissible in a file that claims to test algebra with no
/// correlation in it: `α_ij = k_i/k_j` cancels the shared factor exactly, so
/// Fenske's `α^N` stays exact. What `n` buys is a bubble point — with `n = 0` the
/// equation `Σ_c K_c(T)·x_c = 1` has no root at all, and a cascade stage has no
/// temperature. That is a correction from building M7.3: the M7.2 constant-α
/// model, whose stated purpose was exactly this file, could not have driven one
/// stage of a cascade.
const EXPONENT: f64 = 10.0;

const FEED_FLOW: f64 = 10.0;

/// What "the same answer" means when comparing two converged solves, or a solve
/// against an independent model.
///
/// **Derived from the cascade's own convergence criterion, not chosen to make an
/// assertion pass.** That criterion is I7's `1e-5` kg/s over this column's feed
/// rate — `1e-5/10 = 1e-6` as a mass fraction — so two solves that both satisfy
/// it may legitimately differ by that much, and asserting anything tighter would
/// be asserting that the solver is more converged than it promises to be. The
/// temperature form is the same relative figure at these profiles' ~400 K,
/// rounded up to a milli-kelvin.
const CONVERGED_TOLERANCE: f64 = 1.0e-6;
const CONVERGED_TOLERANCE_K: f64 = 1.0e-3;

fn slate(cuts: &[(&str, f64, f64)]) -> Slate {
    Slate::new(
        cuts.iter()
            .map(|(name, tb, molar_mass)| PseudoComponent {
                name: (*name).into(),
                tb: Kelvin(*tb),
                molar_mass: KgPerMol(*molar_mass),
                density: Some(KgPerM3(800.0)),
                cp: JPerKgK(2000.0),
                phase: Phase::Liquid,
            })
            .collect(),
    )
    .unwrap()
}

/// A binary whose two cuts differ in molar mass by 2×, so the mass ⇄ mole
/// boundary is live in every number here. A slate of equal molar masses would
/// make the two bases coincide and hide the conversion entirely — the shape
/// `degenerate-fixture-disables-the-code-path` records.
fn binary() -> Slate {
    slate(&[("light", 350.0, 0.100), ("heavy", 450.0, 0.200)])
}

/// K-values supplied by the test, scaled by a power of temperature. See
/// `EXPONENT`.
fn thermo(slate: &Slate, k: Vec<f64>) -> ConstantAlphaThermo {
    ConstantAlphaThermo::with_temperature_exponent(slate, k, T_REF, EXPONENT).unwrap()
}

struct Column {
    spec: CascadeSpec,
    draws: Vec<ColumnDraw>,
}

/// A distillate and a bottoms, nothing between them.
fn two_product(stages: u32, feed_stage: u32, reflux_ratio: f64, d_over_f: f64) -> Column {
    Column {
        spec: CascadeSpec {
            stages,
            feed_stage,
            reflux_ratio,
        },
        draws: vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(d_over_f)),
            ColumnDraw::by_stage(NodeId(2), stages, None),
        ],
    }
}

fn try_run(
    cascade: &StageCascade,
    slate: &Slate,
    thermo: &ConstantAlphaThermo,
    column: &Column,
    feed_mass: &[f64],
) -> Result<Separation, refinery_core::error::SimError> {
    let feed = Composition::from_weights(feed_mass).unwrap();
    cascade.separate(
        &ColumnPass {
            slate,
            draws: &column.draws,
            smearing: Kelvin(0.0),
            pressure: P_ATM,
            feed: &feed,
            feed_flow: KgPerSec(FEED_FLOW),
            temperature: T_REF,
            cascade: Some(&column.spec),
        },
        thermo,
    )
}

fn run(
    cascade: &StageCascade,
    slate: &Slate,
    thermo: &ConstantAlphaThermo,
    column: &Column,
    feed_mass: &[f64],
) -> Separation {
    try_run(cascade, slate, thermo, column, feed_mass).expect("this reference column is feasible")
}

/// Mole fraction of the light (component 0) cut in draw `i`. The draws come back
/// as MASS compositions, so reading this crosses the boundary fork 1 built.
fn light_mole_fraction(separation: &Separation, slate: &Slate, i: usize) -> f64 {
    MoleFractions::from_mass(&separation.draws[i].composition, slate)
        .unwrap()
        .fractions()[0]
}

/// `(x_D/(1−x_D))·((1−x_B)/x_B)` — Fenske's separation ratio, on a MOLAR basis
/// because vapour–liquid equilibrium is molar.
fn fenske_ratio(separation: &Separation, slate: &Slate) -> f64 {
    let top = light_mole_fraction(separation, slate, 0);
    let bottom = light_mole_fraction(separation, slate, 1);
    (top / (1.0 - top)) * ((1.0 - bottom) / bottom)
}

/// The reflux ratios the order-of-convergence assertion is made over.
///
/// **The window was measured, not assumed** — the lesson
/// `integrator-order-of-convergence` records is that you find out which range is
/// asymptotic before writing the tolerance. Sweeping `R` from 40 to 20480 for
/// `N ∈ {2, 3, 5, 10}` gives error-halving ratios that reach 2 in different
/// places: `N = 2` is asymptotic from about `R = 160`, `N = 5` only from about
/// `R = 320`, and `N = 10` is still at 1.95 at `R = 20480`. The window below is
/// where all of `N ∈ {2, 3, 5}` are asymptotic together.
const REFLUX_WINDOW: [f64; 5] = [320.0, 640.0, 1280.0, 2560.0, 5120.0];

/// **Fenske at total reflux**, in the only form the specification set admits —
/// as a limit, plus an exact bound that holds everywhere.
///
/// Total reflux means `D = 0`, `R = ∞` and `F = 0`, which fork 3's ratio-only
/// specification cannot express and which is not a well-posed boundary-value
/// problem anyway (the profile is then determined only up to scale). The textbook
/// states Fenske as a *limiting* relation for exactly that reason, so this gate is
/// two claims about the limit rather than one evaluation at it:
///
/// 1. **The bound is exact and non-asymptotic.** `α^N` is the separation at
///    minimum stages, so no finite reflux can beat it: `ratio ≤ α^N` at every `N`
///    and every `R`, to roundoff. This is the half that polices the stage-counting
///    convention. If `N` excluded the reboiler, the same column would be claiming
///    `α^(N+1)` and would breach the bound by a whole factor of `α`.
/// 2. **The approach is first order in `1/R`.** At finite reflux
///    `y_{j+1} = (R·x_j + x_D)/(R+1)`, which differs from the total-reflux
///    `y_{j+1} = x_j` by `(x_D − x_j)/(R+1) = O(1/R)`. So doubling `R` must halve
///    the error and the ratio of successive errors must approach 2 — a sharper
///    statement than any single tolerance at one large `R`, and one that stays
///    away from the cancellation a huge `R` would introduce.
///
/// The *over*-counted convention is caught by the second half: the limit would
/// then be `α^(N−1)`, the error would stall near `1 − 1/α` instead of falling, and
/// the halving ratios would go to 1.
#[test]
fn fenske_bounds_the_separation_and_is_approached_at_first_order() {
    let slate = binary();
    let alpha = 4.0;
    let thermo = thermo(&slate, vec![2.0, 2.0 / alpha]);
    let cascade = StageCascade::new();

    for stages in [2u32, 3, 5] {
        let exact = alpha.powi(stages as i32);
        let mut errors = Vec::new();
        for reflux in REFLUX_WINDOW {
            let column = two_product(stages, stages.div_ceil(2), reflux, 0.35);
            let separation = run(&cascade, &slate, &thermo, &column, &[0.5, 0.5]);
            let ratio = fenske_ratio(&separation, &slate);
            assert!(
                ratio <= exact * (1.0 + 1e-9),
                "N = {stages}, R = {reflux}: the separation ratio {ratio:.6e} exceeds \
                 Fenske's α^N = {exact:.6e}. No finite reflux can separate better than total \
                 reflux, so a ratio above the bound means the stage count in the exponent \
                 disagrees with the stage count in the cascade."
            );
            errors.push((ratio / exact - 1.0).abs());
        }

        for window in errors.windows(2) {
            let halving = window[0] / window[1];
            assert!(
                (1.90..=2.02).contains(&halving),
                "N = {stages}: doubling the reflux ratio must HALVE the distance to Fenske's \
                 limit (first order in 1/R). Got {halving:.4} between successive errors \
                 {:.4e} and {:.4e}; the whole sequence over R = {REFLUX_WINDOW:?} was \
                 {errors:?}.",
                window[0],
                window[1]
            );
        }
        assert!(
            *errors.last().unwrap() < *errors.first().unwrap() / 8.0,
            "N = {stages}: sixteenfold reflux must shrink the Fenske error by at least eight; \
             got {errors:?}"
        );
    }
}

/// `N = 10` is measured and **excluded from the order assertion above**, on
/// purpose and in the open.
///
/// The per-stage error is `O(1/R)` and it compounds over `N` stages, so the reflux
/// ratio at which the sequence turns asymptotic grows faster than `N` does: at
/// `N = 10` the halving ratio is still 1.95 at `R = 20480`. Asserting the order
/// there would need either a reflux ratio where the linear algebra is resolving a
/// difference in its last digits, or a band so wide it asserts nothing.
///
/// What is still exact at `N = 10` is the bound and the direction, so those are
/// what this asserts. A gate that narrows its own coverage should say which part
/// it dropped rather than quietly not covering it.
#[test]
fn a_tall_column_still_obeys_the_bound_and_improves_with_reflux() {
    let slate = binary();
    let alpha = 4.0;
    let thermo = thermo(&slate, vec![2.0, 2.0 / alpha]);
    let cascade = StageCascade::new();
    let exact = alpha.powi(10);

    let mut previous = f64::INFINITY;
    for reflux in [320.0f64, 1280.0, 5120.0, 20480.0] {
        let column = two_product(10, 5, reflux, 0.35);
        let separation = run(&cascade, &slate, &thermo, &column, &[0.5, 0.5]);
        let ratio = fenske_ratio(&separation, &slate);
        assert!(
            ratio <= exact * (1.0 + 1e-9),
            "N = 10, R = {reflux}: ratio {ratio:.6e} exceeds α^10 = {exact:.6e}"
        );
        let error = (ratio / exact - 1.0).abs();
        assert!(
            error < previous,
            "N = 10: more reflux must move the separation TOWARDS Fenske's limit; at \
             R = {reflux} the error {error:.4e} is not below the previous {previous:.4e}"
        );
        previous = error;
    }
    assert!(
        previous < 0.05,
        "N = 10 at R = 20480 should be within 5% of α^10; got {previous:.4e}"
    );
}

/// **One stage reduces to the M7.2 flash.** With `N = 1` and `R = 0` there is no
/// reflux and no rectification: the single stage takes `F`, boils up `V = D` and
/// drops `B = F − D`, which is an isothermal flash at that stage's own temperature
/// with a vapour fraction of `D/F` **on a molar basis**.
///
/// The two are independent code paths — a tridiagonal stage balance with a
/// condenser fold, against a Rachford–Rice bisection — so their agreement pins the
/// fold, the flow profile and the bubble-point solve at once.
///
/// **And it is where the mass ⇄ mole boundary can actually fail.** `D/F = 0.4` is
/// a MASS ratio (correction 1) while `β` is molar, and on this slate the two
/// differ by a quarter. A cascade that confused the bases would land on a different
/// `β`, and the flash — which never sees a mass fraction — would disagree. The last
/// assertion in this test is what keeps that discrimination from silently
/// evaporating if the slate is ever edited.
#[test]
fn a_single_stage_at_zero_reflux_is_the_rachford_rice_flash() {
    let slate = binary();
    let thermo = thermo(&slate, vec![2.0, 0.5]);
    let column = two_product(1, 1, 0.0, 0.4);
    let feed_mass = [0.5, 0.5];
    let separation = run(&StageCascade::new(), &slate, &thermo, &column, &feed_mass);

    // The stage's own temperature, which the cascade solved for. Draw 1 sits at
    // stage 1, which IS the reboiler when N = 1.
    let stage_t = separation.draws[1].temperature;

    let feed = Composition::from_weights(&feed_mass).unwrap();
    let feed_moles = MoleFractions::from_mass(&feed, &slate).unwrap();
    let flash = flash_isothermal(&feed_moles, &slate, &thermo, stage_t, P_ATM).unwrap();

    let distillate = MoleFractions::from_mass(&separation.draws[0].composition, &slate).unwrap();
    let bottoms = MoleFractions::from_mass(&separation.draws[1].composition, &slate).unwrap();
    for c in 0..slate.len() {
        approx::assert_relative_eq!(
            distillate.fractions()[c],
            flash.vapour.fractions()[c],
            max_relative = CONVERGED_TOLERANCE
        );
        approx::assert_relative_eq!(
            bottoms.fractions()[c],
            flash.liquid.fractions()[c],
            max_relative = CONVERGED_TOLERANCE
        );
    }

    // **The condenser's own temperature**, which nothing else in this milestone
    // reads — `DrawSeparation::temperature` gains its consumer in M7.4 — and which
    // is therefore the one number here that could be quietly wrong. It is not
    // stage 1's temperature: the distillate is the CONDENSED vapour, a lighter
    // liquid, so it boils lower at the same pressure. Asserted twice, because the
    // ordering alone would still pass if the field simply echoed stage 1.
    let condenser_t = separation.draws[0].temperature;
    assert!(
        condenser_t.value() < stage_t.value(),
        "the distillate is condensed vapour and boils below the stage that made it: got \
         {} K against a stage at {} K",
        condenser_t.value(),
        stage_t.value()
    );
    let bubble: f64 = distillate
        .fractions()
        .iter()
        .enumerate()
        .map(|(c, x)| x * thermo.k_value(&slate, c, condenser_t, P_ATM).unwrap())
        .sum();
    approx::assert_relative_eq!(bubble, 1.0, max_relative = CONVERGED_TOLERANCE);

    // The flash's molar vapour fraction must equal the cascade's molar distillate
    // rate over its molar feed rate — the conversion the mass ratio went through.
    let feed_molar = FEED_FLOW / feed_moles.mean_molar_mass(&slate).value();
    let distillate_molar =
        separation.draws[0].split * FEED_FLOW / distillate.mean_molar_mass(&slate).value();
    let beta = distillate_molar / feed_molar;
    approx::assert_relative_eq!(
        flash.vapour_fraction,
        beta,
        max_relative = CONVERGED_TOLERANCE
    );
    assert!(
        (beta - separation.draws[0].split).abs() > 1000.0 * CONVERGED_TOLERANCE,
        "this gate only discriminates while the molar vapour fraction {beta:.6} is separated \
         from the MASS ratio {:.6} it was derived from by far more than the tolerance the \
         assertion above compares at. On a slate whose cuts had equal molar masses the two \
         bases would coincide and that assertion would prove nothing.",
        separation.draws[0].split
    );
}

/// **The null gate: `α = 1` separates nothing, at any `N`.** Give every cut the
/// same K and the stage balances become identical for every component, so the
/// profile is the feed on every stage and each draw carries the feed composition
/// unchanged — no matter how many stages, how much reflux, or how the draws are
/// sized.
///
/// This is the gate that catches a cascade that "separates" for a reason other
/// than volatility: an off-by-one in the stage indexing, a draw reading the wrong
/// stage, a flow profile that leaks. Every one of those moves a composition here,
/// where the physics says nothing may move.
#[test]
fn equal_volatility_produces_no_separation_at_any_stage_count() {
    let slate = slate(&[
        ("light", 350.0, 0.100),
        ("middle", 400.0, 0.150),
        ("heavy", 450.0, 0.200),
    ]);
    let thermo = thermo(&slate, vec![1.5, 1.5, 1.5]);
    let feed_mass = [0.2, 0.3, 0.5];
    let feed = Composition::from_weights(&feed_mass).unwrap();

    for (stages, feed_stage, reflux) in [(1u32, 1u32, 0.0), (2, 1, 1.5), (5, 3, 2.0), (9, 4, 0.75)]
    {
        // Three draws wherever the geometry allows one, so the side-draw path is
        // exercised rather than left to the two-product case.
        let column = if stages >= 3 {
            Column {
                spec: CascadeSpec {
                    stages,
                    feed_stage,
                    reflux_ratio: reflux,
                },
                draws: vec![
                    ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
                    ColumnDraw::by_stage(NodeId(2), stages - 1, Some(0.2)),
                    ColumnDraw::by_stage(NodeId(3), stages, None),
                ],
            }
        } else {
            two_product(stages, feed_stage, reflux, 0.3)
        };

        let separation = run(&StageCascade::new(), &slate, &thermo, &column, &feed_mass);
        for (i, draw) in separation.draws.iter().enumerate() {
            for c in 0..slate.len() {
                approx::assert_abs_diff_eq!(
                    draw.composition.fractions()[c],
                    feed.fractions()[c],
                    epsilon = 1e-12
                );
            }
            assert!(
                draw.split > 0.0,
                "N = {stages}: draw {i} takes no mass, so its composition asserts nothing"
            );
        }
    }
}

/// **Every draw's split is the mass ratio it was declared with, and they sum to
/// 1.** Fork 3's exactness claim, with correction 1's mechanism under it: the
/// ratios are mass, at the boundary, so total mass closes by construction rather
/// than at convergence. The bottoms is `1 − Σ others` and is never declared.
///
/// The per-component balance is the part that is NOT free here, and it is checked
/// against the same 1e-5 kg/s the cascade's own convergence criterion is derived
/// from. A splitter conserved every component identically; a cascade only
/// converges to it (fork 4).
#[test]
fn the_splits_are_the_declared_mass_ratios_and_the_components_balance() {
    let slate = binary();
    let thermo = thermo(&slate, vec![2.0, 0.5]);
    let column = Column {
        spec: CascadeSpec {
            stages: 6,
            feed_stage: 3,
            reflux_ratio: 2.0,
        },
        draws: vec![
            ColumnDraw::by_stage(NodeId(1), 0, Some(0.25)),
            ColumnDraw::by_stage(NodeId(2), 4, Some(0.15)),
            ColumnDraw::by_stage(NodeId(3), 6, None),
        ],
    };
    let separation = run(&StageCascade::new(), &slate, &thermo, &column, &[0.5, 0.5]);

    assert_eq!(separation.draws[0].split, 0.25);
    assert_eq!(separation.draws[1].split, 0.15);
    approx::assert_abs_diff_eq!(separation.draws[2].split, 0.60, epsilon = 1e-15);
    let total: f64 = separation.draws.iter().map(|d| d.split).sum();
    approx::assert_abs_diff_eq!(total, 1.0, epsilon = 1e-15);

    let feed = Composition::from_weights(&[0.5, 0.5]).unwrap();
    for c in 0..slate.len() {
        let out: f64 = separation
            .draws
            .iter()
            .map(|d| d.split * d.composition.fractions()[c])
            .sum();
        assert!(
            (out - feed.fractions()[c]).abs() * FEED_FLOW <= 1.0e-5,
            "component {c}: {:.9e} kg/s out against {:.9e} kg/s in exceeds I7's 1e-5 kg/s",
            out * FEED_FLOW,
            feed.fractions()[c] * FEED_FLOW
        );
    }

    // The column actually separated — otherwise the balance above would be the
    // trivial one and this whole plant would be a pipe.
    assert!(
        light_mole_fraction(&separation, &slate, 0) > light_mole_fraction(&separation, &slate, 2),
        "the distillate must be lighter than the bottoms"
    );

    // The draws come out cold-to-hot down the column: a distillate off a total
    // condenser is the coldest product, the bottoms off the reboiler the hottest.
    let temperatures: Vec<f64> = separation
        .draws
        .iter()
        .map(|d| d.temperature.value())
        .collect();
    assert!(
        temperatures[0] < temperatures[1] && temperatures[1] < temperatures[2],
        "draw temperatures must increase down the column, got {temperatures:?}"
    );
}

/// **The non-convergence `Err`, reached on purpose.** Fork 4 requires the
/// per-component residual be a *gate* and not a counter, which means something has
/// to cross it. A cascade capped at two outer passes cannot have converged, and the
/// message must carry the residual it failed at.
///
/// The second half is what makes this a gate rather than a tautology: the SAME
/// column at the default cap converges. Without that, the test would be satisfied
/// equally well by a cascade that could never converge at all.
#[test]
fn a_cascade_capped_too_low_fails_with_its_residual() {
    let slate = binary();
    let thermo = thermo(&slate, vec![2.0, 0.5]);
    let column = two_product(8, 4, 2.0, 0.3);
    let feed = [0.5, 0.5];

    let message = try_run(
        &StageCascade::with_max_iterations(2),
        &slate,
        &thermo,
        &column,
        &feed,
    )
    .expect_err("two outer passes cannot converge an eight-stage column")
    .to_string();
    assert!(
        message.contains("did not converge")
            && message.contains("per-component mass residual")
            && message.contains("bound of"),
        "the refusal must name what failed, and the per-component residual must be one of \
         the criteria it NAMES — a residual that merely appears in a message while some \
         other criterion does the failing is a counter, not a gate. Got: {message}"
    );

    try_run(&StageCascade::new(), &slate, &thermo, &column, &feed).expect(
        "the same column at the default cap must converge — otherwise the assertion above is \
         satisfied by a cascade that can never converge at all",
    );
}

/// **Start-insensitivity.** Fork 5 permits a warm start on the grounds that "a
/// warm start changes the iteration count, not the fixed point". That is a claim
/// about this solver, so it is measured: the same column solved from a seed
/// displaced 60 K in either direction must give the same profile.
///
/// M7.3 has no warm start (`separate` takes `&self` and is contracted pure), so
/// this gate is what stands between fork 5's permission and the milestone that
/// takes it up.
#[test]
fn the_answer_does_not_depend_on_where_the_solve_started() {
    let slate = binary();
    let thermo = thermo(&slate, vec![2.0, 0.5]);
    let column = two_product(7, 4, 2.5, 0.3);
    let feed = [0.4, 0.6];

    let cold = run(&StageCascade::new(), &slate, &thermo, &column, &feed);
    for offset in [-60.0f64, 60.0] {
        let perturbed = run(
            &StageCascade::with_seed_offset(Kelvin(offset)),
            &slate,
            &thermo,
            &column,
            &feed,
        );
        for (i, (a, b)) in cold.draws.iter().zip(&perturbed.draws).enumerate() {
            for c in 0..slate.len() {
                approx::assert_abs_diff_eq!(
                    a.composition.fractions()[c],
                    b.composition.fractions()[c],
                    epsilon = CONVERGED_TOLERANCE
                );
            }
            approx::assert_abs_diff_eq!(
                a.temperature.value(),
                b.temperature.value(),
                epsilon = CONVERGED_TOLERANCE_K
            );
            assert_eq!(
                a.split, b.split,
                "draw {i}: the mass ratios are declared, not solved, so they cannot move at all"
            );
        }
    }
}

/// A K-value with no temperature dependence has no bubble point, and the cascade
/// says so instead of inventing a stage temperature.
///
/// This is the arm that a "detect a flat model and hold some convention" design
/// would have hidden. A detector cannot tell "K does not depend on T" from "the
/// root is outside my bracket", and the second is the
/// finite-deterministic-plausible-wrong shape this workspace keeps catching, so
/// both are refused and the refusal names the model.
#[test]
fn a_temperature_independent_k_has_no_bubble_point_and_is_refused() {
    let slate = binary();
    let flat = ConstantAlphaThermo::new(&slate, vec![2.0, 0.5]).unwrap();
    let column = two_product(4, 2, 2.0, 0.3);
    let feed = Composition::from_weights(&[0.5, 0.5]).unwrap();

    let message = StageCascade::new()
        .separate(
            &ColumnPass {
                slate: &slate,
                draws: &column.draws,
                smearing: Kelvin(0.0),
                pressure: P_ATM,
                feed: &feed,
                feed_flow: KgPerSec(FEED_FLOW),
                temperature: T_REF,
                cascade: Some(&column.spec),
            },
            &flat,
        )
        .expect_err("a constant K cannot satisfy Σ K·x = 1 except by coincidence")
        .to_string();
    assert!(
        message.contains("bubble point") && message.contains("constant_alpha"),
        "the refusal must say what could not be found and which model could not supply it, \
         got: {message}"
    );
}
