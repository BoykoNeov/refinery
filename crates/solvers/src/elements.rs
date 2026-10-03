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

/// Discharge coefficient of a leak orifice — the vena-contracta loss in
/// `Q = Cd·A·√(2·dp/ρ)`.
///
/// 0.61 is the textbook sharp-edged-orifice value for fully turbulent
/// incompressible flow, where `Cd` is essentially Reynolds-independent and every
/// standard treatment lands in 0.60–0.62 (the contraction coefficient ~0.62
/// times a velocity coefficient ~0.98). It is quoted here as a **modelling
/// constant with a bracket**, not transcribed from a table this repo has read:
/// a hole punched in a pipe by damage has no defined edge geometry, so a `Cd`
/// carried to three figures would be false precision about the damage, not about
/// the arithmetic.
///
/// It is deliberately NOT a scenario parameter yet. Nothing in the model can
/// currently tell 0.60 from 0.62 — the leak rate scales linearly in `Cd`, so the
/// choice is exactly a ±2% statement about a hole whose size is itself commanded
/// by a game — and a knob no gate can discriminate is a knob that invites tuning
/// the plant through it. It un-defers with a scenario that needs two leaks of
/// *different* geometry in one plant.
pub const ORIFICE_CD: f64 = 0.61;

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

    /// Sharp-edged orifice discharging through an area `a` [m²] into a fluid of
    /// density `rho` [kg/m³] — the leak path's whole characteristic.
    ///
    /// Torricelli through a vena contracta: `Q = Cd·A·√(2·dp/ρ)`, so inverting
    /// to this file's form gives `alpha = ρ/(2·Cd²·A²)` and `beta = 0`.
    ///
    /// **`beta = 0` is structural here, and something depends on it.** The
    /// back-feed refusal (`network::finalize`) needs no tolerance precisely
    /// because `flow`'s sign is `sign(dp − beta)`: with `beta = 0` a leak edge
    /// carries mass inward *iff* the plant side is strictly below `P_ATM`, which
    /// is the physical condition itself rather than a threshold someone picked.
    /// An orifice given an elevation head, or composed in series with a pipe,
    /// would keep that refusal compiling and quietly make it fire at the wrong
    /// pressure — so the orifice edge is this branch ALONE, and
    /// `network::compile_edge` asserts the loader gave it no geometry to fold.
    ///
    /// `a = 0` (a dormant, undamaged leak) ⇒ `alpha = +∞`, which `flow` already
    /// reads as exactly zero flow and `compile_edge`'s `conducts` already reads
    /// as closed. A dormant leak needs no special case anywhere.
    pub fn orifice(a: f64, cd: f64, rho: f64) -> Self {
        Self {
            alpha: rho / (2.0 * cd * cd * a * a),
            beta: 0.0,
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

// ---------------------------------------------------------------------------
// Compressible (gas) valve sizing — IEC 60534-2-1 / ISA-75.01. See
// docs/DESIGN.md §3a forks 4 and 6.
// ---------------------------------------------------------------------------

/// Width of the smoothstep band around the choke point, as a FRACTION of the
/// critical ratio `x_choke`. **Zero, and that is a finding rather than an
/// omission.**
///
/// DESIGN §3a fork 4 shipped choking on the premise that "a choke cap is a kink",
/// which would break this file's C¹ contract, and specified a smoothstep to
/// repair it. **The premise is false for this particular `Y`.** The standard's
/// expansion factor is built so the sizing curve meets the plateau with zero
/// slope: with `Y = 1 − x/(3·x_c)` and `x_s = x` below the choke,
///
/// ```text
/// d(Y·√x_s)/dx |_(x→x_c⁻)  =  −√x_c/(3x_c) + (1 − 1/3)/(2√x_c)  =  0
/// ```
///
/// and above the choke both factors are constant, so the derivative is 0 on that
/// side too. The frozen coefficient inherits it: `α_eff = α·x/(Y²·x_s)` has
/// `dα_eff/dx = 2.25·α/x_c` on **both** sides. Measured: the two one-sided limits
/// agree to round-off (5.6e-17 and 0.0 for the flow; equal to 1e-12 for `α`).
/// So the exact clamp is already C¹ — for the flow *and* for the assembled
/// Jacobian entry, which is the thing fork 4 was actually protecting.
///
/// A blend would therefore buy no continuity while introducing a fabricated
/// numerical parameter that biases the answer (`smooth_min` sits slightly BELOW
/// both arguments inside the band). That is the `cat_oil_ratio` argument, so the
/// shipped value is 0 and the model is exactly the published equation. The
/// parameter survives in the signatures because it is what makes the claim
/// falsifiable: a nonzero band must not move the answer, and C¹ must hold at
/// zero band.
///
/// Only the SECOND derivative jumps at the choke, and Newton needs C¹.
pub const CHOKE_BLEND: f64 = 0.0;

/// Bisection steps in `fold_gas_valve`'s inner solve — a FIXED COUNT, not a
/// tolerance.
///
/// A float-tolerance exit makes the iteration count depend on the iterate, which
/// is a determinism hazard the moment anything about the arithmetic shifts; a
/// fixed count is bit-reproducible by construction (rule 3). It costs nothing
/// anywhere else, because it runs only on a gas VALVE edge.
///
/// **This doc used to claim 60 halvings put the bracket "below 1e-18 of it, i.e.
/// to round-off, so this is exact rather than tight enough". That is true in
/// ABSOLUTE terms and false in relative ones**, and the correction is worth
/// keeping because the difference is where the scheme is weakest. The interval
/// halved is `[0, s_total]`, so the bracket is `s_total·2⁻⁶⁰` regardless of
/// where the root actually sits — and when `α_pipe ≫ α_valve` the root sits
/// many decades below `s_total` (at a ratio of 7e11 it is ~1e-12 of it), so the
/// RELATIVE resolution there is ~1e-6, not 1e-18.
///
/// It remains far more than enough, and that is now measured rather than
/// asserted (`solvers/tests/gas_valve_invariants.rs`): against a root resolved
/// in log space the shipped coefficient deviates by 3.97e-15 worst case over
/// the reachable parameter space, and the gates go green at 40 halvings and
/// fail at 32. The margin on 60 is real; the reasoning offered for it was not.
pub const GAS_VALVE_BISECTIONS: u32 = 60;

/// Ratio of specific heats to the standard's air datum: `F_k = γ/1.40`.
///
/// DERIVED from the slate rather than declared — `γ = cp/cv` with `cv` from
/// `Composition::mixture_cv`, which M5.3 already made phase-conditional. Nothing
/// to invent and nothing to put in a TOML file (DESIGN §3a fork 4).
#[inline]
pub fn specific_heat_ratio_factor(gamma: f64) -> f64 {
    gamma / 1.40
}

/// C¹ smooth minimum: exactly `min(a, b)` outside a band of half-width `w`, and
/// a quadratic blend inside it.
///
/// `min(a,b) − (w − |a−b|)²/(4w)`. Value and slope both match `min` at
/// `|a−b| = w` (the correction and its derivative vanish there), and the slope
/// passes smoothly through ½ at `a = b` instead of jumping 1 → 0. The blend is
/// one-sided-conservative: inside the band the result is slightly BELOW both
/// arguments, never above.
#[inline]
pub fn smooth_min(a: f64, b: f64, w: f64) -> f64 {
    let d = (a - b).abs();
    if d >= w || w <= 0.0 {
        a.min(b)
    } else {
        let slack = w - d;
        a.min(b) - slack * slack / (4.0 * w)
    }
}

/// The sizing pressure-drop ratio and the expansion factor at ratio `x`.
///
/// `x_s = smooth_min(x, x_choke)` and `Y = 1 − x_s/(3·x_choke)`, so `Y` runs from
/// 1 at zero drop to exactly **2/3** at and beyond the choke — the standard's
/// value, and independent of whatever `x_T` produced `x_choke`.
///
/// **The clamp is on `x_s`, which the sizing equation uses INSIDE the square root
/// as well as inside `Y`.** Scaling only `Y` would leave `√Δp` in the flow and
/// produce a model that reads as choked while having no plateau at all
/// (DESIGN §3a fork 6).
#[inline]
pub fn expansion(x: f64, x_choke: f64, blend: f64) -> (f64, f64) {
    // `x >= 0` always (callers pass the magnitude of a drop) and `blend < 1`, so
    // `|x − x_choke| >= x_choke > w` near zero and `x_s = x`: the band is never
    // straddled there and `x_s` cannot go negative.
    let x_s = smooth_min(x, x_choke, blend * x_choke);
    (x_s, 1.0 - x_s / (3.0 * x_choke))
}

/// Volumetric flow [m³/s] through a gas valve ALONE at its own pressure drop `s`
/// [Pa] ≥ 0, from the liquid branch that valve compiles to.
///
/// The standard's mass form is `W = C·N₆·Y·√(x·p₁·ρ₁)`, and `x·p₁ = Δp`
/// identically, so in this workspace's coherent-SI convention it is exactly the
/// liquid valve equation with `Δp` replaced by `x_s·p₁` and multiplied by `Y`:
///
/// ```text
/// Q_gas(s) = Y · Q_liquid(x_s·p_up) = Y · √(x_s·p_up / α_liquid)
/// ```
///
/// **This is why the parameter count stays at one.** `Cv` is the SAME coefficient
/// the standard uses for liquid sizing, so `α_liquid` here is `QuadraticBranch::
/// valve`'s, unchanged, and at `Y = 1` the two expressions coincide bit for bit.
/// A separate "gas Cv" field must not appear (DESIGN §3a fork 4).
#[inline]
pub fn gas_valve_flow(alpha_liquid: f64, s: f64, p_up: f64, x_choke: f64, blend: f64) -> f64 {
    let (x_s, y) = expansion(s / p_up, x_choke, blend);
    y * (x_s * p_up / alpha_liquid).sqrt()
}

/// A spring-loaded PSV's opening at inlet pressure `p_inlet` [Pa].
///
/// Shut at or below `set_pressure`, fully open at `set_pressure + accumulation`,
/// and a cubic smoothstep `t²(3 − 2t)` in between — whose derivative vanishes at
/// both ends, so the characteristic is C¹ where it meets both limits and this
/// file's contract survives an element whose area moves.
///
/// **Memoryless, and that is the whole design** (docs/DESIGN.md §3a fork 5). The
/// opening is a pure function of one pressure: no state, no tick history, no
/// tuning constants. A real PSV recloses BELOW its set pressure (blowdown
/// hysteresis) and can chatter; both need element state, and element state is
/// what turns a characteristic into a controller — which is the subsystem M5
/// declines to open. Given up deliberately, not overlooked.
///
/// It is the plant state, not the operator, that moves this — which makes it the
/// first element in the project whose area depends on the solve. It costs no new
/// machinery: M5.2 already recompiles every edge at every iterate, so an opening
/// read off the current pressure is exactly what that loop was built to carry.
#[inline]
pub fn relief_opening(p_inlet: f64, set_pressure: f64, accumulation: f64) -> f64 {
    if !accumulation.is_finite() || accumulation <= 0.0 {
        // Degenerate band: a step at the set pressure. Guarded rather than
        // permitted — the loader refuses it — so this is a floor, not a mode.
        return if p_inlet > set_pressure { 1.0 } else { 0.0 };
    }
    let t = ((p_inlet - set_pressure) / accumulation).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// `d(opening)/d(p_inlet)` [1/Pa] of [`relief_opening`]: `6·t·(1 − t)/accumulation`.
///
/// **Exactly `0.0` outside the band**, because the smoothstep meets both limits
/// with a vanishing slope and `t` is clamped before it is used. That is what keeps
/// a plant whose PSV never lifts bit-identical to the solver before M26: the term
/// this feeds is skipped when it is zero (docs/DESIGN.md §30 fork 3).
///
/// Newton needs it because the opening is frozen into the compiled branch, so
/// the branch's own `flow_ddp` holds the opening fixed. On a vessel at a long
/// timestep the missing "opens wider" share is as large as everything else
/// holding the vessel, and leaving it out makes each Newton step overshoot by
/// about a whole step (§30, ledger row A14).
#[inline]
pub fn relief_opening_slope(p_inlet: f64, set_pressure: f64, accumulation: f64) -> f64 {
    if !accumulation.is_finite() || accumulation <= 0.0 {
        // The degenerate step has no derivative to offer; the loader refuses it.
        return 0.0;
    }
    let t = ((p_inlet - set_pressure) / accumulation).clamp(0.0, 1.0);
    6.0 * t * (1.0 - t) / accumulation
}

/// A check valve's opening at forward drive `drive` [Pa] across its branch (M30,
/// docs/DESIGN.md §33).
///
/// The relief valve's curve with its set point at zero: shut at or below no
/// forward drive, full lift at `full_open`, the cubic smoothstep between. One
/// shape for both pressure-actuated discs, so the two cannot drift apart in the
/// part that keeps this file's C¹ contract.
#[inline]
pub fn check_opening(drive: f64, full_open: f64) -> f64 {
    relief_opening(drive, 0.0, full_open)
}

/// `d(opening)/d(drive)` [1/Pa] of [`check_opening`]. Exactly `0.0` outside the
/// band, as [`relief_opening_slope`] is.
#[inline]
pub fn check_opening_slope(drive: f64, full_open: f64) -> f64 {
    relief_opening_slope(drive, 0.0, full_open)
}

/// Fold a gas valve into the pipe it discharges through, as ONE
/// `QuadraticBranch` whose valve resistance is frozen at the current iterate.
///
/// **Why a fold needs an inner solve here and nowhere else in this project.**
/// Every other element is affine in `Q·|Q|`, so a series composition is `Σα, Σβ`
/// and inverts in closed form. The gas valve is not: `Y` and the choke clamp
/// depend on `x = Δp_valve/p₁`, the valve's OWN share of the drop, and a compiled
/// edge only knows the drop across the whole folded branch. The valve's share is
/// therefore the root of
///
/// ```text
/// g(s) = s + α_pipe·Q_gas(s)²  =  |dp − β| ,     s ∈ [0, |dp − β|]
/// ```
///
/// `g(0) = 0`, `g(S) ≥ S`, and `g` is strictly increasing (the `s` term alone is,
/// and `Q_gas` is non-decreasing once the choke is smoothed), so the root exists,
/// is unique, and bisection is unconditionally robust — no failure mode to report
/// and no divergence to diagnose. That is what makes an inner solve acceptable
/// here (DESIGN §3a fork 6).
///
/// Two alternatives were rejected in the note rather than discovered in code:
/// attributing the WHOLE branch drop to the valve applies a valve's `x_T` to a
/// pipe's friction, and taking `x` from one unchoked predictor pass converges to
/// a fixed point that is not the ISA solution at all — silently, since the
/// pressures stop moving and the solver reports convergence.
///
/// The result is exact rather than a secant fit: with
/// `α_eff = α_liquid·x/(Y²·x_s)` one has `α_eff·Q_gas(s)² = s` identically, so
/// `(α_pipe + α_eff)·Q² = s + α_pipe·Q² = S` and the returned branch's own
/// `flow(dp)` reproduces `Q_gas` — modulo `eps_dp`, which every branch in the
/// project already carries. That identity is asserted rather than trusted, since
/// every downstream reader consumes the BRANCH and not this function.
pub fn fold_gas_valve(
    pipe: QuadraticBranch,
    valve: QuadraticBranch,
    dp: f64,
    p_up: f64,
    x_choke: f64,
    blend: f64,
) -> QuadraticBranch {
    // A closed valve is α = +∞ and stays that way: `flow` yields exactly 0 by
    // IEEE arithmetic, and there is no drop to split.
    if !valve.alpha.is_finite() || valve.alpha <= 0.0 || !p_up.is_finite() || p_up <= 0.0 {
        return pipe.in_series(valve);
    }
    let s_total = (dp - pipe.beta).abs();
    if !s_total.is_finite() || s_total <= 0.0 {
        // Zero net drop: `x → 0`, so `x_s = x`, `Y = 1` and the valve degenerates
        // to its liquid self. Taking the limit rather than bisecting on an empty
        // interval also keeps the `x_s = 0` division below unreachable.
        return pipe.in_series(valve);
    }

    let g = |s: f64| s + pipe.alpha * gas_valve_flow(valve.alpha, s, p_up, x_choke, blend).powi(2);
    let (mut lo, mut hi) = (0.0f64, s_total);
    for _ in 0..GAS_VALVE_BISECTIONS {
        let mid = 0.5 * (lo + hi);
        if g(mid) < s_total {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let s = 0.5 * (lo + hi);

    let (x_s, y) = expansion(s / p_up, x_choke, blend);
    // `x/x_s = s/(x_s·p_up)`, which is exactly 1 below the choke and grows in
    // proportion to the drop above it — the plateau, written as a coefficient.
    // Guarded at `x_s = 0` only for completeness: `s > 0` there implies `x_s > 0`.
    let ratio = if x_s > 0.0 { s / (x_s * p_up) } else { 1.0 };
    let alpha_eff = valve.alpha * ratio / (y * y);
    pipe.in_series(QuadraticBranch {
        alpha: alpha_eff,
        beta: 0.0,
    })
}

// ---------------------------------------------------------------------------
// Compressible (gas) leak orifice — the isentropic nozzle law (M37,
// docs/DESIGN.md §41).
// ---------------------------------------------------------------------------

/// The isentropic critical pressure-DROP ratio `x* = 1 − (2/(γ+1))^(γ/(γ−1))`:
/// the drop, as a fraction of the absolute upstream pressure, at which an ideal
/// nozzle chokes (the throat reaches sonic velocity). 0.4717 for `γ = 1.4`.
///
/// Derived, not declared: the textbook result for isentropic flow of a perfect
/// gas through a converging nozzle (Saint-Venant and Wantzel's law), with `γ`
/// taken from the slate as `fold_gas_service` takes it. Nothing to invent and
/// nothing in a TOML file.
#[inline]
pub fn isentropic_critical_drop_ratio(gamma: f64) -> f64 {
    // `−expm1(ln(r*))`: exact to round-off at every γ, including γ → 1⁺.
    -((gamma / (gamma - 1.0)) * (2.0 / (gamma + 1.0)).ln()).exp_m1()
}

/// The isentropic flow function `ψ(x) = γ/(γ−1)·(r^(2/γ) − r^((γ+1)/γ))`,
/// `r = 1 − x`, with `x` clamped at the choke `x*` — so that the mass flow of a
/// gas through a nozzle of effective area `Cd·A` from upstream state `(p, ρ)` is
///
/// ```text
/// ṁ = Cd·A·√(2·ρ·p·ψ(x))        (Saint-Venant–Wantzel; choked for x ≥ x*)
/// ```
///
/// **`ψ(x) → x` as `x → 0`**, which is Torricelli: `Cd·A·√(2·ρ·Δp)`. **And `ψ` has
/// zero slope at `x*`** — the unclamped `ψ` is MAXIMAL there, which is what makes
/// the throat sonic — so the clamp meets the plateau with no kink, and the
/// element is C¹ through the choke with no blend at all (the property
/// `CHOKE_BLEND` documents for the valve's `Y`, holding here for the same kind of
/// reason).
///
/// Computed as `e^{b·L}·expm1((a−b)·L)`, `L = ln(1 − x)`, `a = 2/γ`,
/// `b = (γ+1)/γ`, rather than as the difference of two powers: near `x = 0` both
/// powers are ≈ 1 and their difference would cancel to noise, which is exactly
/// the small-leak regime this function must reduce to Torricelli in.
#[inline]
pub fn isentropic_flow_function(x: f64, gamma: f64) -> f64 {
    let x_s = x.min(isentropic_critical_drop_ratio(gamma));
    let log_r = (-x_s).ln_1p();
    let a = 2.0 / gamma;
    let b = (gamma + 1.0) / gamma;
    gamma / (gamma - 1.0) * (b * log_r).exp() * ((a - b) * log_r).exp_m1()
}

/// A leak orifice in GAS service, as the branch `compile_edge` hands the solvers:
/// its liquid branch (`QuadraticBranch::orifice`, Torricelli) with an effective
/// resistance that reproduces the isentropic nozzle law at drop `dp`.
///
/// `Q = √(dp/α_eff)` must equal `ṁ/ρ` with `ṁ` from [`isentropic_flow_function`],
/// so `α_eff = α_liquid · x/ψ(x)`, `x = |dp|/p_up`. Below the choke `x/ψ` is a
/// compressibility correction that rises from exactly 1; above it `ψ` is the
/// plateau and `α_eff` grows in proportion to the drop — the mass flow stops
/// rising with the drop and rises only with the upstream state.
///
/// **Two properties the back-feed refusal (`network::finalize`) relies on are
/// kept by construction**: `beta = 0`, and the flow keeps the sign of `dp`
/// (`α_eff > 0` scales the magnitude only). **A dormant hole stays closed**:
/// `α_liquid = +∞` is returned untouched, before anything else is read. **A zero
/// drop is the liquid branch bit for bit**: the `x → 0` limit, taken rather than
/// evaluated as `0/0`.
///
/// `p_up` is the UPWIND absolute pressure and `rho` the upwind density
/// (`compile_edge` already evaluates it there), i.e. the stagnation state of
/// whichever end the gas leaves. The Jacobian freezes `α_eff` at the iterate, as
/// it does a gas valve's (`docs/DEFERRED.md` A18).
pub fn gas_orifice(liquid: QuadraticBranch, dp: f64, p_up: f64, gamma: f64) -> QuadraticBranch {
    if !liquid.alpha.is_finite() || liquid.alpha <= 0.0 || !p_up.is_finite() || p_up <= 0.0 {
        return liquid;
    }
    let x = dp.abs() / p_up;
    if !x.is_finite() || x <= 0.0 {
        return liquid;
    }
    // `ψ > 0` for every `x > 0` and `γ > 1` (both factors are positive). A
    // degenerate `γ` gives a non-finite or non-positive `α_eff`, which the caller
    // (`compile_edge`) refuses by name rather than handing to a solver.
    QuadraticBranch {
        alpha: liquid.alpha * (x / isentropic_flow_function(x, gamma)),
        beta: 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// The air value by hand: `1 − (2/2.4)^3.5 = 1 − (5/6)^3.5`.
    #[test]
    fn the_critical_drop_ratio_for_air_is_the_textbook_one() {
        assert_relative_eq!(
            isentropic_critical_drop_ratio(1.4),
            1.0 - (5.0f64 / 6.0).powf(3.5),
            max_relative = 1e-14
        );
    }

    /// `ψ(x)/x → 1` as `x → 0` — the gas hole is Torricelli for a small drop —
    /// and the leading correction is `−3x/(2γ)`: expanding `r^a − r^b` to second
    /// order gives `(b−a)·x + (a−b)(a+b−1)·x²/2` with `b − a = (γ−1)/γ` and
    /// `a + b − 1 = 3/γ`. The cancellation-free form must still resolve it at 1e-12,
    /// where the difference of the two powers would be pure noise.
    #[test]
    fn the_flow_function_reduces_to_torricelli_at_small_drops() {
        let gamma = 1.3;
        for x in [1e-12f64, 1e-9, 1e-6, 1e-3] {
            let ratio = isentropic_flow_function(x, gamma) / x;
            assert_relative_eq!(
                ratio,
                1.0 - 3.0 * x / (2.0 * gamma),
                max_relative = x * x + 1e-14
            );
        }
    }

    /// C¹ through the choke with no blend: the one-sided slopes of `ψ` meet at
    /// zero, because the unclamped `ψ` is maximal at `x*`.
    #[test]
    fn the_flow_function_is_flat_where_it_meets_the_choke() {
        for gamma in [1.1f64, 1.3, 1.4, 1.67] {
            let xc = isentropic_critical_drop_ratio(gamma);
            let h = 1e-6;
            let below =
                (isentropic_flow_function(xc, gamma) - isentropic_flow_function(xc - h, gamma)) / h;
            let above =
                (isentropic_flow_function(xc + h, gamma) - isentropic_flow_function(xc, gamma)) / h;
            assert!(below.abs() < 1e-5, "γ = {gamma}: slope below {below:e}");
            assert_eq!(above, 0.0, "γ = {gamma}: the plateau is exactly flat");
        }
    }

    /// A zero drop is the liquid branch bit for bit; a dormant hole stays closed.
    #[test]
    fn a_gas_hole_at_zero_drop_or_zero_area_is_its_liquid_self() {
        let liquid = QuadraticBranch::orifice(1e-4, ORIFICE_CD, 5.0);
        let at_zero = gas_orifice(liquid, 0.0, 1e6, 1.3);
        assert_eq!(at_zero.alpha.to_bits(), liquid.alpha.to_bits());
        assert_eq!(at_zero.beta, 0.0);
        let dormant = QuadraticBranch::orifice(0.0, ORIFICE_CD, 5.0);
        assert!(gas_orifice(dormant, 9e5, 1e6, 1.3).alpha.is_infinite());
    }

    /// `beta = 0` and a resistance even in `dp`, so the flow keeps the sign of
    /// the drop — the two facts `network::finalize`'s back-feed refusal needs.
    #[test]
    fn a_gas_hole_is_odd_in_the_drop() {
        let liquid = QuadraticBranch::orifice(1e-4, ORIFICE_CD, 5.0);
        for dp in [1e3f64, 3e5, 9e5] {
            let fwd = gas_orifice(liquid, dp, 1e6, 1.3);
            let rev = gas_orifice(liquid, -dp, 1e6, 1.3);
            assert_eq!(fwd.beta, 0.0);
            assert_eq!(fwd.alpha.to_bits(), rev.alpha.to_bits());
            assert_eq!(fwd.flow(dp, 1.0), -rev.flow(-dp, 1.0));
        }
    }

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

    // -----------------------------------------------------------------------
    // Gas valve sizing (M5.4b). What each of these pins is stated, because the
    // magnitude gates are restatements of the formula and the others are not
    // (DESIGN §3a fork 6).
    // -----------------------------------------------------------------------

    /// A test fixture roughly at `gas_valve.toml`'s state: methane at 10 bar.
    fn gas_fixture() -> (QuadraticBranch, QuadraticBranch, f64, f64) {
        let rho = 6.582; // P·M̄/(R·T) at 10 bar, 293.15 K
        let cv_si = 25.0 / (3600.0 * 1e5_f64.sqrt());
        let valve = QuadraticBranch::valve(cv_si, 1.0, rho / 998.0);
        let pipe = QuadraticBranch::pipe(pipe_resistance(0.02, 10.0, 0.15, rho), 0.0);
        let x_choke = 0.670_91; // F_k·x_T for γ = 1.30455, x_T = 0.72
        (pipe, valve, x_choke, 1.0e6)
    }

    /// `Y` is exactly 2/3 at the choke and stays there beyond it.
    ///
    /// A specific number from the standard, and independent of whatever `x_T`
    /// produced `x_choke` — which is what makes it worth asserting separately
    /// from any magnitude.
    #[test]
    fn expansion_is_two_thirds_at_and_beyond_the_choke() {
        for x_choke in [0.14f64, 0.354, 0.671] {
            for x in [x_choke, 1.3 * x_choke, 10.0 * x_choke] {
                let (x_s, y) = expansion(x, x_choke, 0.0);
                assert_relative_eq!(y, 2.0 / 3.0, max_relative = 1e-12);
                // And the clamp is on `x_s` too, which is what gives a plateau
                // rather than merely a reduced coefficient.
                assert_relative_eq!(x_s, x_choke, max_relative = 1e-12);
            }
            // Below the choke nothing is clamped and Y < 1 strictly.
            let (x_s, y) = expansion(0.5 * x_choke, x_choke, 0.0);
            assert_relative_eq!(x_s, 0.5 * x_choke, max_relative = 1e-12);
            assert_relative_eq!(y, 1.0 - 1.0 / 6.0, max_relative = 1e-12);
        }
    }

    /// The exact clamp is C¹ — the property `CHOKE_BLEND = 0` rests on.
    ///
    /// Both the flow and the frozen `α_eff` meet the plateau with matching
    /// one-sided slopes, because the standard's `Y` is constructed that way. This
    /// is the gate DESIGN §3a fork 4 asked for under the name "Jacobian
    /// continuity across the choke point"; what changed is that no smoothing is
    /// needed to obtain it.
    #[test]
    fn the_exact_choke_clamp_is_c1_in_both_the_flow_and_the_coefficient() {
        let (pipe, valve, x_choke, p_up) = gas_fixture();
        let s_choke = x_choke * p_up;
        let h = 1e-6 * s_choke;

        let flow = |s: f64| gas_valve_flow(valve.alpha, s, p_up, x_choke, 0.0);
        let slope_below = (flow(s_choke - h) - flow(s_choke - 2.0 * h)) / h;
        let slope_above = (flow(s_choke + 2.0 * h) - flow(s_choke + h)) / h;
        // Both limits are ZERO, so compare against the curve's own scale rather
        // than to each other: a relative test on two near-zero numbers is noise.
        let scale = flow(s_choke) / s_choke;
        assert!(
            slope_below.abs() < 1e-4 * scale && slope_above.abs() < 1e-4 * scale,
            "flow slopes at the choke must both vanish: below {slope_below:.3e}, \
             above {slope_above:.3e}, scale {scale:.3e}"
        );

        // The coefficient's slope is NOT zero — it is 2.25·α/x_choke on both
        // sides, which is the stronger statement: the two branches happen to have
        // the same nonzero derivative rather than both being flat.
        let alpha_of = |dp: f64| fold_gas_valve(pipe, valve, dp, p_up, x_choke, 0.0).alpha;
        let below = (alpha_of(s_choke - h) - alpha_of(s_choke - 2.0 * h)) / h;
        let above = (alpha_of(s_choke + 2.0 * h) - alpha_of(s_choke + h)) / h;
        assert!(above > 0.0, "the coefficient must be rising past the choke");
        assert_relative_eq!(below, above, max_relative = 1e-3);
    }

    /// A nonzero blend band does not move the answer.
    ///
    /// The band-insensitivity gate DESIGN §3a fork 4 asks for, at two widths an
    /// order apart — and now with a stronger reading than it was written with:
    /// since the exact clamp is already C¹, the band is a perturbation that buys
    /// nothing, and this gate is what says so quantitatively.
    #[test]
    fn the_blend_band_does_not_move_the_answer() {
        let (pipe, valve, x_choke, p_up) = gas_fixture();
        for x in [0.3f64, 0.99, 1.05, 1.4] {
            let dp = x * x_choke * p_up;
            let exact = fold_gas_valve(pipe, valve, dp, p_up, x_choke, 0.0).alpha;
            for blend in [0.002f64, 0.02] {
                let blended = fold_gas_valve(pipe, valve, dp, p_up, x_choke, blend).alpha;
                let moved = (blended - exact).abs() / exact;
                assert!(
                    moved < 1e-4,
                    "blend {blend} at x/x_choke = {x} moved α by {moved:.3e}"
                );
            }
        }
    }

    /// The folded branch reproduces the inner solve's own flow.
    ///
    /// `α_eff = α_liquid·x/(Y²·x_s)` is constructed so that
    /// `α_eff·Q_gas(s)² = s` identically, hence `(α_pipe + α_eff)·Q² = S` and
    /// `branch.flow(dp)` IS `Q_gas`. **Every downstream reader consumes the
    /// branch and not `gas_valve_flow`**, so if these two ever part company the
    /// solver is running a different law from the one this file documents and
    /// nothing else would notice. Agreement to ~1e-12 is also what says the
    /// bisection ran to round-off, which is why no iteration-count sweep is
    /// needed.
    #[test]
    fn the_folded_branch_reproduces_the_inner_solve() {
        let (pipe, valve, x_choke, p_up) = gas_fixture();
        for x in [0.05f64, 0.5, 0.999, 1.0, 1.2, 1.34] {
            let dp = x * x_choke * p_up;
            let branch = fold_gas_valve(pipe, valve, dp, p_up, x_choke, 0.0);
            let q_branch = branch.flow(dp, 1e-9);
            // The valve's own share, recovered from the coefficient the fold
            // returned, then fed back through the sizing law.
            let s = (branch.alpha - pipe.alpha) * q_branch * q_branch;
            let q_law = gas_valve_flow(valve.alpha, s, p_up, x_choke, 0.0);
            assert_relative_eq!(q_branch, q_law, max_relative = 1e-10);
            // And the split it found actually balances the series.
            assert_relative_eq!(
                s + pipe.alpha * q_branch * q_branch,
                dp,
                max_relative = 1e-9
            );
        }
    }

    /// The gas fold degenerates to the ordinary liquid fold as `x → 0`, and does
    /// so at FIRST order in `x` — which is `Y`'s leading term.
    ///
    /// Two code paths agreeing, not either one read back, and the reason a "gas
    /// Cv" field must not exist: the standard uses one coefficient for both
    /// services, so the incompressible limit has to come out exactly.
    #[test]
    fn the_gas_fold_degenerates_to_the_liquid_fold_as_x_vanishes() {
        let (pipe, valve, x_choke, p_up) = gas_fixture();
        let liquid = pipe.in_series(valve).alpha;
        let mut previous: Option<(f64, f64)> = None;
        for x in [1e-2f64, 1e-3, 1e-4] {
            let dp = x * x_choke * p_up;
            let gas = fold_gas_valve(pipe, valve, dp, p_up, x_choke, 0.0).alpha;
            let deviation = (gas - liquid).abs() / liquid;
            assert!(
                deviation < 3.0 * x,
                "at x/x_choke = {x} the gas fold deviates from the liquid one by \
                 {deviation:.3e}, which is not first order"
            );
            if let Some((x_prev, dev_prev)) = previous {
                // Tenfold smaller x ⇒ tenfold smaller deviation, within 20%.
                let order = (dev_prev / deviation) / (x_prev / x);
                assert!(
                    (0.8..1.25).contains(&order),
                    "the degeneracy must be first order in x; measured ratio {order:.3}"
                );
            }
            previous = Some((x, deviation));
        }
    }

    /// A gas valve branch stays odd about `β`, so reverse flow is the mirror of
    /// forward flow rather than an unphysical `Y > 1`.
    ///
    /// The standard's equation is written for forward flow, and `x < 0` would
    /// give `Y > 1` — an expansion factor that INCREASES the flow, unbounded as
    /// the reversal deepens. `fold_gas_valve` evaluates `x` from `|dp − β|`, so
    /// the shifted oddness `elements.rs` has asserted since M1 survives.
    ///
    /// STATED LIMITATION, not an omission (DESIGN §3a fork 6): this means a gas
    /// valve chokes symmetrically in both directions. Right for a control valve;
    /// wrong for a PSV, which passes no reverse flow at all. Same register as
    /// fork 5's "no blowdown hysteresis".
    #[test]
    fn a_gas_valve_branch_is_odd_about_beta() {
        let (pipe_flat, valve, x_choke, p_up) = gas_fixture();
        let pipe = QuadraticBranch::pipe(pipe_flat.alpha, 2.0e4); // a real β
        for d in [1.0e5f64, 4.0e5, 9.0e5] {
            let fwd = fold_gas_valve(pipe, valve, pipe.beta + d, p_up, x_choke, 0.0);
            let rev = fold_gas_valve(pipe, valve, pipe.beta - d, p_up, x_choke, 0.0);
            assert_relative_eq!(fwd.alpha, rev.alpha, max_relative = 1e-12);
            assert_relative_eq!(
                fwd.flow(pipe.beta + d, 1e-9),
                -rev.flow(pipe.beta - d, 1e-9),
                max_relative = 1e-10
            );
        }
    }

    /// A closed gas valve is still exactly zero, never NaN — the M1 guarantee,
    /// re-asserted because the gas path adds a division by `Y²` and a bisection
    /// that a closed valve must not enter.
    #[test]
    fn a_closed_gas_valve_is_zero_not_nan() {
        let (pipe, _, x_choke, p_up) = gas_fixture();
        let shut = QuadraticBranch::valve(1e-3, 0.0, 1.0);
        let branch = fold_gas_valve(pipe, shut, 9.0e5, p_up, x_choke, 0.0);
        assert!(branch.alpha.is_infinite());
        assert_eq!(branch.flow(9.0e5, 1.0), 0.0);
        assert_eq!(branch.flow_ddp(9.0e5, 1.0), 0.0);
    }

    /// The PSV's opening curve: shut at set, full at set + accumulation, and a
    /// cubic smoothstep between — with a VANISHING slope at both ends.
    ///
    /// The flat ends are the point, not decoration. A linear ramp would give the
    /// same endpoints and the same monotone shape, and would put a slope
    /// discontinuity exactly where the valve cracks and where it saturates — two
    /// kinks in a characteristic this file promises to keep C¹, on the one element
    /// whose area moves with the iterate. Asserting the midpoint alone cannot tell
    /// the two apart (both give ½), so the slopes are what this gate checks.
    #[test]
    fn the_relief_opening_is_a_smoothstep_with_flat_ends() {
        let (set, band) = (20.0e5, 1.0e5);
        assert_eq!(relief_opening(set, set, band), 0.0);
        assert_eq!(relief_opening(set - 1.0e5, set, band), 0.0);
        assert_eq!(relief_opening(set + band, set, band), 1.0);
        assert_eq!(relief_opening(set + 5.0 * band, set, band), 1.0);
        // Monotone, and symmetric about the midpoint: t²(3−2t) + (1−t)²(1+2t) = 1.
        for t in [0.1f64, 0.25, 0.5, 0.75, 0.9] {
            let up = relief_opening(set + t * band, set, band);
            let down = relief_opening(set + (1.0 - t) * band, set, band);
            assert_relative_eq!(up + down, 1.0, max_relative = 1e-12);
            assert_relative_eq!(up, t * t * (3.0 - 2.0 * t), max_relative = 1e-12);
        }
        // Flat where it meets both limits — a linear ramp gives 1/band at both.
        let h = 1e-4 * band;
        let scale = 1.0 / band;
        for edge in [set, set + band] {
            let slope = (relief_opening(edge + h, set, band) - relief_opening(edge - h, set, band))
                / (2.0 * h);
            assert!(
                slope.abs() < 0.05 * scale,
                "the opening must meet its limit with a vanishing slope; at \
                 {edge:.0} Pa it is {slope:.3e} against a linear ramp's {scale:.3e}"
            );
        }
    }

    /// `F_k = γ/1.40`, and `γ = 1.40` is the air datum the standard normalizes to.
    #[test]
    fn the_specific_heat_ratio_factor_is_unity_for_air() {
        assert_relative_eq!(specific_heat_ratio_factor(1.40), 1.0, max_relative = 1e-15);
        assert_relative_eq!(
            specific_heat_ratio_factor(1.30),
            1.30 / 1.40,
            max_relative = 1e-15
        );
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
