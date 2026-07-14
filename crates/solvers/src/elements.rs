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

/// Series-composable branch characteristic covering every M1 hydraulic
/// element. Each is affine in `Q·|Q|`:  `dp = alpha·Q·|Q| + beta`, with
/// `dp = P_upstream − P_downstream` [Pa], `Q` = volumetric flow [m³/s].
/// `alpha ≥ 0` is the quadratic resistance; `beta` is the pressure offset
/// (elevation head, and the pump's pressure "jump"). Because the form is
/// closed under series composition (`Σalpha, Σbeta`), a pipe and the pump or
/// valve folded into its end compose in **closed form** — the combined branch
/// inverts to `Q(dp)` with no inner scalar solve (see `newton_flow`).
///
/// Regularization note (matters for later cross-fidelity tests): `flow`
/// applies `eps` [Pa] to the *combined* shifted drop `dp − beta`, whereas the
/// standalone `valve_flow` applies `eps` to `dp/rho_rel`. The two agree away
/// from zero but differ within the O(eps) smoothing zone for non-unit SG. The
/// Newton solver only ever uses this type, so it is self-consistent; a future
/// SimpleFlowSolver cross-check near zero flow must account for this (or route
/// through this same type).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadraticBranch {
    /// Quadratic resistance [Pa/(m³/s)²]. `+∞` ⇒ fully closed (flow → 0).
    pub alpha: f64,
    /// Pressure offset [Pa]: static head + pump jump.
    pub beta: f64,
}

impl QuadraticBranch {
    /// Plain pipe: `alpha = k` (Darcy–Weisbach, from `pipe_resistance`),
    /// `beta = elev_head_pa = rho·g·Δz` static head (Δz = downstream − upstream
    /// elevation), so at zero flow a higher downstream sits at lower pressure.
    pub fn pipe(k: f64, elev_head_pa: f64) -> Self {
        Self {
            alpha: k,
            beta: elev_head_pa,
        }
    }

    /// Control valve (ISA): `Q = cv_eff·sqrt(dp/rho_rel)` ⇒
    /// `alpha = rho_rel / cv_eff²`, `beta = 0`. `opening = 0` ⇒ `cv_eff = 0` ⇒
    /// `alpha = +∞` (closed): `flow` then yields exactly 0 by IEEE arithmetic,
    /// never NaN.
    pub fn valve(cv_si: f64, opening: f64, rho_rel: f64) -> Self {
        let cv_eff = cv_si * opening.clamp(0.0, 1.0);
        Self {
            alpha: rho_rel / (cv_eff * cv_eff),
            beta: 0.0,
        }
    }

    /// Centrifugal pump `H(Q) = h0 − a·Q·|Q|`, rise `dP = rho·g·H`. Across the
    /// device `P_out = P_in + rho·g·H`, so its contribution to the series
    /// *drop* is `dp = −rho·g·H = rho·g·a·Q|Q| − rho·g·h0`: `alpha = rho·g·a`
    /// (≥0), `beta = −rho·g·h0` (the negative offset is the pressure jump). An
    /// off pump passes `h0 = 0` (beta = 0) and acts as pure resistance.
    pub fn pump(h0: f64, a: f64, rho: f64, g: f64) -> Self {
        Self {
            alpha: rho * g * a,
            beta: -rho * g * h0,
        }
    }

    /// Compose two branches in series: resistances and offsets add.
    pub fn in_series(self, other: Self) -> Self {
        Self {
            alpha: self.alpha + other.alpha,
            beta: self.beta + other.beta,
        }
    }

    /// Volumetric flow from branch pressure drop, C¹-smooth through zero.
    /// Invert `dp − beta = alpha·Q·|Q|` ⇒
    /// `Q = smooth_signed_sqrt(dp − beta, eps)/sqrt(alpha)`.
    /// `alpha = +∞` (closed) ⇒ `Q = 0`.
    #[inline]
    pub fn flow(&self, dp: f64, eps: f64) -> f64 {
        smooth_signed_sqrt(dp - self.beta, eps) / self.alpha.sqrt()
    }

    /// `d(flow)/d(dp)` — the branch conductance `g_e ≥ 0` used in the Jacobian.
    /// `alpha = +∞` ⇒ `0`.
    #[inline]
    pub fn flow_ddp(&self, dp: f64, eps: f64) -> f64 {
        smooth_signed_sqrt_deriv(dp - self.beta, eps) / self.alpha.sqrt()
    }
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

    #[test]
    fn branch_pipe_matches_pipe_flow() {
        // A QuadraticBranch pipe must reproduce the standalone pipe_flow away
        // from zero (no elevation ⇒ beta = 0).
        let k = 1.6211e8;
        let b = QuadraticBranch::pipe(k, 0.0);
        for dp in [1e5f64, -1e5, 3e6] {
            assert_relative_eq!(b.flow(dp, 1.0), pipe_flow(dp, k, 1.0), max_relative = 1e-12);
            assert_relative_eq!(
                b.flow_ddp(dp, 1.0),
                pipe_flow_ddp(dp, k, 1.0),
                max_relative = 1e-12
            );
        }
    }

    #[test]
    fn branch_closed_valve_is_zero_not_nan() {
        // opening = 0 ⇒ alpha = +∞ ⇒ flow and conductance are exactly 0.
        let b = QuadraticBranch::valve(1e-3, 0.0, 1.0);
        assert!(b.alpha.is_infinite());
        assert_eq!(b.flow(5e5, 1.0), 0.0);
        assert_eq!(b.flow_ddp(5e5, 1.0), 0.0);
        assert!(b.flow(5e5, 1.0).is_finite());
    }

    #[test]
    fn branch_pump_offset_and_shifted_oddness() {
        // Pump: beta = −rho·g·h0. At zero net drop the pump drives flow; the
        // combined branch is odd about dp = beta, NOT about 0.
        let (rho, g, h0, a) = (1000.0, 9.806_65, 20.0, 1.0e3);
        let b = QuadraticBranch::pump(h0, a, rho, g);
        let beta = -rho * g * h0;
        assert_relative_eq!(b.beta, beta, max_relative = 1e-12);
        assert!(b.alpha > 0.0);
        // Shifted oddness: flow(beta + d) = −flow(beta − d).
        for d in [1e4f64, 5e5, 2e6] {
            assert_relative_eq!(
                b.flow(beta + d, 1.0),
                -b.flow(beta - d, 1.0),
                max_relative = 1e-12
            );
        }
    }

    #[test]
    fn branch_series_composition_adds() {
        // pipe ∘ pump: alpha and beta add; invert round-trips to the same Q.
        let (rho, g) = (1000.0, 9.806_65);
        let pipe = QuadraticBranch::pipe(2.0e7, rho * g * 5.0); // 5 m rise
        let pump = QuadraticBranch::pump(30.0, 8.0e5, rho, g);
        let comp = pipe.in_series(pump);
        assert_relative_eq!(comp.alpha, pipe.alpha + pump.alpha, max_relative = 1e-12);
        assert_relative_eq!(comp.beta, pipe.beta + pump.beta, max_relative = 1e-12);
        // Forward-then-inverse: pick Q, compute dp = alpha·Q|Q| + beta, recover Q.
        let q = 0.05;
        let dp = comp.alpha * q * q.abs() + comp.beta;
        // eps → 0 limit: use a tiny eps so the smooth form ≈ exact sqrt.
        assert_relative_eq!(comp.flow(dp, 1e-6), q, max_relative = 1e-4);
    }
}
