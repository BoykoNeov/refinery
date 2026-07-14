//! Hydraulic element characteristics — flow/pressure relations for pipes,
//! valves, pumps. Shared by BOTH the Newton network solver and the simple
//! relaxation solver: fidelity differs in how the network is solved, never
//! in the element physics.
//!
//! Conventions:
//! - Everything SI. Q = volumetric flow [m³/s], dP = upstream − downstream [Pa].
//! - All characteristics are C¹-smooth through zero flow / zero dP, because
//!   the Newton Jacobian must stay finite. sqrt-law devices use the
//!   regularization  x/sqrt(|x|+eps)  instead of  sign(x)·sqrt(|x|).

/// Regularized signed square root: ≈ sign(x)·sqrt(|x|) away from 0, linear
/// near 0 with finite slope. eps in the units of x.
#[inline]
pub fn smooth_signed_sqrt(x: f64, eps: f64) -> f64 {
    x / (x.abs() + eps).sqrt()
}

/// d/dx of `smooth_signed_sqrt`.
#[inline]
pub fn smooth_signed_sqrt_deriv(x: f64, eps: f64) -> f64 {
    let a = x.abs() + eps;
    (a - 0.5 * x.abs()) / a.powf(1.5)
}

/// Darcy–Weisbach pipe resistance coefficient k such that dP = k·Q·|Q|.
/// dP = f · (L/D) · ρ v²/2,  v = Q/A  ⇒  k = f·L·ρ / (2·D·A²).
/// Constant friction factor for M1 (Haaland correlation is a later upgrade).
pub fn pipe_resistance(friction_factor: f64, length_m: f64, diameter_m: f64, rho: f64) -> f64 {
    let area = core::f64::consts::PI * diameter_m * diameter_m / 4.0;
    friction_factor * length_m * rho / (2.0 * diameter_m * area * area)
}

/// Pipe flow from pressure drop (inverse of dP = k·Q·|Q|), smooth at 0:
/// Q = smooth_signed_sqrt(dP, eps)/sqrt(k).
pub fn pipe_flow(dp: f64, k: f64, eps: f64) -> f64 {
    smooth_signed_sqrt(dp, eps) / k.sqrt()
}
pub fn pipe_flow_ddp(dp: f64, k: f64, eps: f64) -> f64 {
    smooth_signed_sqrt_deriv(dp, eps) / k.sqrt()
}

/// Valve flow, ISA-75.01 form in SI: Q = cv_eff · sqrt(dP/ρ_rel).
/// `cv_si` already folds unit conversion (scenario loader's job).
/// Equal-percentage vs linear trim: start LINEAR (cv_eff = cv_si·opening),
/// note in scenario format when trim curves are added.
pub fn valve_flow(dp: f64, cv_si: f64, opening: f64, rho_rel: f64, eps: f64) -> f64 {
    let cv_eff = cv_si * opening.clamp(0.0, 1.0);
    cv_eff * smooth_signed_sqrt(dp / rho_rel, eps)
}
pub fn valve_flow_ddp(dp: f64, cv_si: f64, opening: f64, rho_rel: f64, eps: f64) -> f64 {
    let cv_eff = cv_si * opening.clamp(0.0, 1.0);
    cv_eff * smooth_signed_sqrt_deriv(dp / rho_rel, eps) / rho_rel
}

/// Centrifugal pump head [m] at volumetric flow Q: H(Q) = h0 − a·Q·|Q|
/// (|Q| keeps it defined for reverse flow; real pumps in reverse are a
/// later refinement — document if it ever matters).
/// Pressure rise: dP_pump = ρ·g·H(Q). A pump that is `off` contributes
/// h0 = 0 and acts as a (high-resistance) pipe.
pub fn pump_pressure_rise(q: f64, h0: f64, a: f64, rho: f64, g: f64) -> f64 {
    rho * g * (h0 - a * q * q.abs())
}
pub fn pump_pressure_rise_dq(q: f64, a: f64, rho: f64, g: f64) -> f64 {
    rho * g * (-2.0 * a * q.abs())
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn smooth_sqrt_matches_sqrt_away_from_zero() {
        // Far from the regularization region the smooth form must agree
        // with sign(x)·sqrt(|x|) to <0.1%.
        let eps = 1.0;
        for x in [1e4f64, 1e6, -1e4, -1e6] {
            let exact = x.signum() * x.abs().sqrt();
            assert_relative_eq!(smooth_signed_sqrt(x, eps), exact, max_relative = 1e-3);
        }
    }

    #[test]
    fn smooth_sqrt_finite_slope_at_zero() {
        let eps = 1.0;
        assert!(smooth_signed_sqrt_deriv(0.0, eps).is_finite());
        assert!(smooth_signed_sqrt_deriv(0.0, eps) > 0.0);
    }

    #[test]
    fn pipe_resistance_hand_calc() {
        // f=0.02, L=100 m, D=0.1 m, ρ=1000 kg/m³:
        // A = π·0.01/4 = 7.85398e-3 m²
        // k = 0.02·100·1000 / (2·0.1·A²) = 2000/(0.2·6.1685e-5) = 1.6211e8
        let k = pipe_resistance(0.02, 100.0, 0.1, 1000.0);
        assert_relative_eq!(k, 1.6211e8, max_relative = 1e-3);
    }
}
