//! Reference cases for M54's two-phase head multiplier (docs/DESIGN.md §59):
//! RELAP5/MOD3 Code Manual Vol. I (NUREG/CR-5535-V1), §3.5.4, Table 3.5-3 —
//! the Semiscale and WCL steam-water pump tests — smoothed by the monotone
//! piecewise-cubic Hermite interpolant of Fritsch & Carlson (1980).
//!
//! The table's six points are the published numbers. The values between them
//! were computed by hand from Fritsch & Carlson's formulas in a few lines of
//! Python outside the repo, not from this code:
//! - the slope at `α = 0.08` is the weighted harmonic mean of the secants either
//!   side, `(w₀ + w₁)/(w₀/d₀ + w₁/d₁)` with `h = (0.01, 0.085)`,
//!   `w = (2h₁ + h₀, h₁ + 2h₀) = (0.18, 0.105)`, `d = (74, 0.26/0.085)`:
//!   7.753 128 313 891 834;
//! - the slope at `α = 1` is the three-point end formula,
//!   `((2h₄ + h₃)·d₄ − h₄·d₃)/(h₃ + h₄)` with `h = (0.735, 0.1)`,
//!   `d = (0, −10)`: −11.197 604 790 419 163;
//! - every other slope is zero (a flat neighbour or a change of sign);
//! - the midpoints are the cubic Hermite basis at `t = ½`.

use refinery_solvers::elements::two_phase_head_multiplier;

/// RELAP5 Table 3.5-3, `(α, M_H)`.
const TABLE: [(f64, f64); 6] = [
    (0.0, 0.0),
    (0.07, 0.0),
    (0.08, 0.74),
    (0.165, 1.0),
    (0.9, 1.0),
    (1.0, 0.0),
];

/// **Every published point, exactly**: the interpolant passes through the table.
#[test]
fn passes_through_every_point_of_the_table() {
    for (alpha, m) in TABLE {
        let got = two_phase_head_multiplier(alpha);
        assert!((got - m).abs() <= 1e-15, "M({alpha}) = {got}, table {m}");
    }
}

/// **Between the points, the hand calculation**: the three segments whose
/// shape is not flat, at their midpoints.
#[test]
fn matches_the_hand_calculation_between_the_points() {
    for (alpha, expected) in [
        (0.075, 0.360_308_589_607_635_2),
        (0.1225, 0.952_376_988_335_100_8),
        (0.95, 0.639_970_059_880_239_6),
    ] {
        let got = two_phase_head_multiplier(alpha);
        assert!(
            (got - expected).abs() <= 1e-14,
            "M({alpha}) = {got}, hand {expected}"
        );
    }
    // The flat stretches stay flat: untouched to 7%, all gone from 16.5% to 90%.
    for alpha in [0.0, 0.03, 0.069_999] {
        assert_eq!(two_phase_head_multiplier(alpha), 0.0, "{alpha}");
    }
    for alpha in [0.2, 0.5, 0.899_999] {
        assert_eq!(two_phase_head_multiplier(alpha), 1.0, "{alpha}");
    }
}

/// **C¹, and no overshoot.** At each interior point the slope from the left
/// equals the slope from the right — the table's straight lines would kink
/// there, which the network's characteristics must not (DESIGN §3a fork 4) —
/// and on every segment the curve stays between its two ends, rising or falling
/// with them.
#[test]
fn is_smooth_at_the_points_and_monotone_between_them() {
    // One-sided differences carry the curvature times the step: at most about
    // 4.5e4 × 1e-8 here, against a corner of 3 to 74 in the table's lines.
    let h = 1e-8;
    for &(alpha, _) in &TABLE[1..TABLE.len() - 1] {
        let left = (two_phase_head_multiplier(alpha) - two_phase_head_multiplier(alpha - h)) / h;
        let right = (two_phase_head_multiplier(alpha + h) - two_phase_head_multiplier(alpha)) / h;
        assert!(
            (left - right).abs() <= 1e-3,
            "kink at {alpha}: {left} from the left, {right} from the right"
        );
    }
    for pair in TABLE.windows(2) {
        let ((x0, y0), (x1, y1)) = (pair[0], pair[1]);
        let (lo, hi) = (y0.min(y1), y0.max(y1));
        let mut last = y0;
        for k in 1..=1000 {
            let alpha = x0 + (x1 - x0) * f64::from(k) / 1000.0;
            let m = two_phase_head_multiplier(alpha);
            assert!(
                (lo..=hi).contains(&m),
                "M({alpha}) = {m} outside [{lo}, {hi}]"
            );
            assert!(
                (m - last) * (y1 - y0) >= -1e-15,
                "M turns back at {alpha}: {last} then {m}"
            );
            last = m;
        }
    }
}

/// **Outside `[0, 1]` the share is clamped to it**: no vapour is no loss, and
/// a share past one is pure vapour.
#[test]
fn clamps_outside_the_unit_interval() {
    assert_eq!(two_phase_head_multiplier(-0.2), 0.0);
    assert_eq!(
        two_phase_head_multiplier(1.3),
        two_phase_head_multiplier(1.0)
    );
}
