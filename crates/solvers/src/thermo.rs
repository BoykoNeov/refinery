//! K-value fidelities: `ThermoModel::k_value`, the first property the trait
//! reserved since M1 actually carries (M7.2, DESIGN §5 fork 2).
//!
//! Two implementations, and they exist for different jobs. `TroutonThermo` is
//! the physical one a plant runs on. `ConstantAlphaThermo` hands a test its own
//! K-values, so a separation gate can be written against algebra that does not
//! depend on any correlation being right — the split DESIGN §5 insists on under
//! "three families, and conflating them proves neither".

use refinery_core::components::Slate;
use refinery_core::error::SimError;
use refinery_core::traits::ThermoModel;
use refinery_core::units::{Kelvin, Pascal, P_ATM, R_GAS};

/// Validate the state a K-value is asked for, shared by both implementations so
/// neither can be the one that lets a NaN through (rule 5).
fn check_state(
    slate: &Slate,
    component: usize,
    temperature: Kelvin,
    pressure: Pascal,
    model: &str,
) -> Result<(), SimError> {
    if component >= slate.len() {
        return Err(SimError::Numerical(format!(
            "{model}: component index {component} is off the end of a {}-component slate",
            slate.len()
        )));
    }
    let t = temperature.value();
    if !t.is_finite() || t <= 0.0 {
        return Err(SimError::Numerical(format!(
            "{model}: K-value at a non-positive or non-finite temperature ({t} K)"
        )));
    }
    let p = pressure.value();
    if !p.is_finite() || p <= 0.0 {
        return Err(SimError::Numerical(format!(
            "{model}: K-value at a non-positive or non-finite pressure ({p} Pa)"
        )));
    }
    Ok(())
}

/// Raoult's law over a Clausius–Clapeyron vapour pressure anchored at each cut's
/// normal boiling point, with `Δh_vap` from Trouton's rule — the complex
/// (research) K-value fidelity.
///
/// ```text
/// K_c = Psat_c(T) / P                                    Raoult + ideal vapour
/// ln(Psat/P_atm) = −(Δh_vap/R)·(1/T − 1/tb)              Clausius–Clapeyron,
///                                                        Δh_vap taken constant,
///                                                        integrated from (tb, P_atm)
/// Δh_vap = C·tb                                          Trouton's rule
/// ⇒ Psat_c(T) = P_atm · exp[(C/R)·(1 − tb_c/T)]
/// ```
///
/// **What this rests on, stated narrowly** (DESIGN §5, correction 6): `tb` is
/// slate data, but Trouton's constant `C` is **one empirical fitted number** and
/// is not. What makes that admissible here — where M4 refused a parameter set
/// outright — is that it carries a gate of each kind: an exact identity that
/// does **not** depend on `C` (`K = 1` at `T = tb`, `P = P_ATM`, since the
/// integration starts there), and a published **envelope** that does. Neither
/// alone is enough, and correction 4 gives the sharp reason: the identity is
/// structurally incapable of catching a wrong `C`.
///
/// Where it is weakest, so a reader does not have to find out: Trouton's rule is
/// an entropy-of-vaporization *correlation*, and it is poorest for associating
/// fluids — water's true `Δs_vap` is nearer 109 J/(mol·K), so this form
/// overstates water's vapour pressure by about 60% at 50 °C. It is good to
/// roughly 10% over the reference hydrocarbon's whole tabulated range
/// (`tests/reference/vapour_pressure.rs`), which is the class of fluid a crude
/// slate is made of.
///
/// No overflow arm is needed and that is a property of the form, not luck: the
/// exponent `(C/R)(1 − tb/T)` is bounded above by `C/R` for every `T > 0`, so
/// `Psat ≤ P_atm·e^(C/R)` — about 4 GPa at `C = 88` — and below by nothing worse
/// than an underflow to zero as `T → 0⁺`, which the temperature guard already
/// refuses.
pub struct TroutonThermo {
    /// Trouton's constant `C = Δh_vap/tb` [J/(mol·K)].
    trouton_constant: f64,
}

impl TroutonThermo {
    /// Trouton's rule: the entropy of vaporization at the normal boiling point
    /// is roughly `88 J/(mol·K)` for a non-associating liquid. The shipped
    /// value, and the one the envelope gate is measured at.
    pub const TROUTON_CONSTANT: f64 = 88.0;

    /// The model at the shipped constant.
    pub fn new() -> Self {
        Self::with_trouton_constant(Self::TROUTON_CONSTANT)
    }

    /// The model at an arbitrary constant.
    ///
    /// This exists **for the gates**, and it is what makes two of them possible
    /// at all: the exact identities are run at two different constants to prove
    /// they cannot see one (turning DESIGN §5 correction 4 from prose into a
    /// test), and the envelope is run at a deliberately wrong one to prove it
    /// can. An envelope nothing is shown to fall outside of is not a gate.
    pub fn with_trouton_constant(trouton_constant: f64) -> Self {
        Self { trouton_constant }
    }

    /// Saturated vapour pressure of one cut [Pa].
    ///
    /// **Private on purpose.** As a public method it would be the one path into
    /// this model that skips `check_state`, so a caller with a negative
    /// temperature would get `exp` of a large positive — an `inf` handed out by
    /// a public API, which rule 5 forbids and which `k_value`'s own guard would
    /// have caught. A consumer that wants a vapour pressure asks for
    /// `k_value(…, P_ATM)` and multiplies: `K = Psat/P` makes that exact, and it
    /// is how the envelope gate gets its number.
    fn saturation_pressure(&self, slate: &Slate, component: usize, temperature: Kelvin) -> f64 {
        let tb = slate.get(component).tb.value();
        P_ATM.value() * ((self.trouton_constant / R_GAS) * (1.0 - tb / temperature.value())).exp()
    }
}

impl Default for TroutonThermo {
    fn default() -> Self {
        Self::new()
    }
}

impl ThermoModel for TroutonThermo {
    fn name(&self) -> &'static str {
        "trouton"
    }

    fn k_value(
        &self,
        slate: &Slate,
        component: usize,
        temperature: Kelvin,
        pressure: Pascal,
    ) -> Result<f64, SimError> {
        check_state(slate, component, temperature, pressure, "trouton")?;
        // Raoult: K = Psat(T)/P, ISA-free — Smith, Van Ness & Abbott,
        // *Introduction to Chemical Engineering Thermodynamics*, modified Raoult
        // with γ = 1 and φ = 1 (ideal solution, ideal vapour).
        let k = self.saturation_pressure(slate, component, temperature) / pressure.value();
        if !k.is_finite() {
            return Err(SimError::NonFiniteState {
                location: format!(
                    "trouton k_value for '{}' at {} K, {} Pa",
                    slate.get(component).name,
                    temperature.value(),
                    pressure.value()
                ),
            });
        }
        Ok(k)
    }
}

/// K-values supplied per component, times an optional power of temperature:
///
/// ```text
/// K_c(T) = k_c · (T/T_ref)^n         n = 0 by default ⇒ K is a constant
/// ```
///
/// "Constant relative volatility" is the classical shortcut, and this is its
/// honest shape on this trait: the stored numbers ARE the K-values (up to a
/// factor shared by every component), so the relative volatility of any pair
/// `α_ij = K_i/K_j = k_i/k_j` is constant **for any `n`**, which is exactly the
/// condition the Fenske and null gates need.
///
/// **Not selectable from scenario TOML, and that is deliberate rather than an
/// oversight.** The vector has no representation in a plant file — it is
/// per-component data with no physical origin, which is the thing this workspace
/// refuses to let a scenario carry. It exists so a separation gate can be
/// written against cascade *algebra* with the correlation held out of it
/// (DESIGN §5, "the cascade algebra, through a relative volatility supplied by
/// the test"); its consumers are tests.
///
/// **Why `n` exists at all — M7.2's version of this type could not do the job it
/// was written for.** A cascade stage's temperature comes from its bubble point,
/// `Σ_c K_c(T)·x_c = 1`. With `n = 0` that equation has **no root**: the left
/// side does not depend on `T`. So the M7.2 type, whose whole stated purpose was
/// to let a separation gate run with the correlation held out, could not have
/// supported one stage of a cascade. Any monotone `T`-dependence fixes it without
/// touching `α`, and a power law is the cheapest one that stays positive for every
/// `T > 0`. That is a correction from building M7.3, recorded in DESIGN §5.
///
/// `n = 0` is still the default because a flash is *isothermal* — it is handed its
/// temperature and never solves for one — and the M7.2 flash gates read the
/// stored numbers back exactly.
pub struct ConstantAlphaThermo {
    k_values: Vec<f64>,
    /// Reference temperature of the scaling [K]; inert at `exponent = 0`.
    t_ref: Kelvin,
    /// `n` in `K_c = k_c·(T/T_ref)^n`. `0` is the constant model.
    exponent: f64,
}

impl ConstantAlphaThermo {
    /// One K per slate position, in slate order, independent of `T` and `P`.
    ///
    /// # Errors
    /// `SimError` if the vector does not match the slate length, or carries a
    /// non-finite or non-positive K — a zero K is a component that can never
    /// leave the liquid, which makes `Σ z/K` in a flash infinite rather than
    /// merely large.
    pub fn new(slate: &Slate, k_values: Vec<f64>) -> Result<Self, SimError> {
        Self::with_temperature_exponent(slate, k_values, Kelvin(1.0), 0.0)
    }

    /// `K_c(T) = k_c·(T/t_ref)^exponent` — the same fixed relative volatility,
    /// made monotone in `T` so a bubble point exists. See the type's docs.
    ///
    /// # Errors
    /// As `new`, plus a non-positive or non-finite `t_ref` and a non-finite
    /// `exponent`. A NEGATIVE exponent is allowed and refused nowhere: it is a
    /// model whose K falls with temperature, which is unphysical but is exactly
    /// the sort of thing a gate may want to hand a cascade on purpose.
    pub fn with_temperature_exponent(
        slate: &Slate,
        k_values: Vec<f64>,
        t_ref: Kelvin,
        exponent: f64,
    ) -> Result<Self, SimError> {
        if k_values.len() != slate.len() {
            return Err(SimError::Scenario(format!(
                "constant-alpha thermo has {} K-values for a {}-component slate",
                k_values.len(),
                slate.len()
            )));
        }
        if let Some((i, k)) = k_values
            .iter()
            .enumerate()
            .find(|(_, k)| !k.is_finite() || **k <= 0.0)
        {
            return Err(SimError::Scenario(format!(
                "constant-alpha K for '{}' must be finite and > 0, got {k}",
                slate.get(i).name
            )));
        }
        if !t_ref.value().is_finite() || t_ref.value() <= 0.0 {
            return Err(SimError::Scenario(format!(
                "constant-alpha reference temperature must be finite and > 0, got {} K",
                t_ref.value()
            )));
        }
        if !exponent.is_finite() {
            return Err(SimError::Scenario(format!(
                "constant-alpha temperature exponent must be finite, got {exponent}"
            )));
        }
        Ok(Self {
            k_values,
            t_ref,
            exponent,
        })
    }
}

impl ThermoModel for ConstantAlphaThermo {
    fn name(&self) -> &'static str {
        "constant_alpha"
    }

    fn k_value(
        &self,
        slate: &Slate,
        component: usize,
        temperature: Kelvin,
        pressure: Pascal,
    ) -> Result<f64, SimError> {
        // The state is validated even when the answer ignores it: a caller that
        // reaches here with a 0 K tray has a bug either way, and a model that
        // silently accepts a state its sibling refuses would make the two
        // fidelities disagree about which plants are legal.
        check_state(slate, component, temperature, pressure, "constant_alpha")?;
        let k = self.k_values[component];
        // Branched rather than always multiplying by `powf(0.0)`: at n = 0 this
        // must return the stored number BIT-for-bit, because the M7.2 flash gates
        // compare it with `assert_eq!`.
        if self.exponent == 0.0 {
            return Ok(k);
        }
        let scaled = k * (temperature.value() / self.t_ref.value()).powf(self.exponent);
        if !scaled.is_finite() || scaled <= 0.0 {
            return Err(SimError::NonFiniteState {
                location: format!(
                    "constant_alpha k_value for '{}' at {} K: k = {k} scaled by \
                     (T/{})^{} gave {scaled}",
                    slate.get(component).name,
                    temperature.value(),
                    self.t_ref.value(),
                    self.exponent
                ),
            });
        }
        Ok(scaled)
    }
}

/// The EXACT identities — the family that holds for any Trouton constant, and
/// therefore the family that cannot police one (DESIGN §5, correction 4). The
/// magnitude gate lives apart, in `tests/reference/vapour_pressure.rs`, against
/// a tabulated vapour pressure.
#[cfg(test)]
mod tests {
    use super::*;
    use refinery_core::components::{Phase, PseudoComponent, Slate};
    use refinery_core::units::{JPerKgK, KgPerM3, KgPerMol};

    /// Cuts distinguished only by boiling point — every other property is a
    /// placeholder, since a K-value at this fidelity reads `tb` and nothing else.
    fn slate_with_tbs(tbs: &[f64]) -> Slate {
        Slate::new(
            tbs.iter()
                .enumerate()
                .map(|(i, &tb)| PseudoComponent {
                    name: format!("cut{i}"),
                    tb: Kelvin(tb),
                    molar_mass: KgPerMol(0.1),
                    density: Some(KgPerM3(800.0)),
                    cp: JPerKgK(2000.0),
                    phase: Phase::Liquid,
                })
                .collect(),
        )
        .unwrap()
    }

    /// The constants the identity tests are run at. `88` is shipped; the other
    /// two are deliberately wrong by ±30%, and running every identity at all
    /// three is what turns "these identities cannot see the constant" from a
    /// claim in the DESIGN note into a gate.
    const CONSTANTS: [f64; 3] = [TroutonThermo::TROUTON_CONSTANT, 61.6, 114.4];

    /// `K = 1` at `T = tb`, `P = P_ATM` — the anchor the Clausius–Clapeyron form
    /// integrates FROM, so it is exact to machine precision and holds for every
    /// constant. That independence is the point: it is why this test passing
    /// says nothing about whether the correlation's magnitude is right.
    #[test]
    fn k_is_one_at_the_normal_boiling_point_for_any_trouton_constant() {
        let slate = slate_with_tbs(&[300.0, 341.89, 600.0]);
        for c in CONSTANTS {
            let thermo = TroutonThermo::with_trouton_constant(c);
            for i in 0..slate.len() {
                let k = thermo
                    .k_value(&slate, i, slate.get(i).tb, P_ATM)
                    .expect("a cut at its own boiling point is a valid state");
                approx::assert_relative_eq!(k, 1.0, max_relative = 1e-12);
            }
        }
    }

    /// `K` is strictly increasing in `T` at fixed `P`: heat a cut and more of it
    /// wants to be vapour. Also true for every constant.
    #[test]
    fn k_increases_with_temperature_for_any_trouton_constant() {
        let slate = slate_with_tbs(&[400.0]);
        for c in CONSTANTS {
            let thermo = TroutonThermo::with_trouton_constant(c);
            let mut previous = 0.0;
            for t in [250.0, 300.0, 350.0, 400.0, 450.0, 500.0] {
                let k = thermo.k_value(&slate, 0, Kelvin(t), P_ATM).unwrap();
                assert!(
                    k > previous,
                    "K must increase with T (constant {c}): at {t} K got {k}, \
                     which is not above the previous {previous}"
                );
                previous = k;
            }
        }
    }

    /// A HEAVIER cut (higher `tb`) has the LOWER `K` at the same `(T, P)` — the
    /// ordering that makes a separation separate at all, and the third identity
    /// that survives any constant.
    #[test]
    fn a_heavier_cut_has_the_lower_k_for_any_trouton_constant() {
        let slate = slate_with_tbs(&[350.0, 450.0, 550.0]);
        for c in CONSTANTS {
            let thermo = TroutonThermo::with_trouton_constant(c);
            let k: Vec<f64> = (0..slate.len())
                .map(|i| thermo.k_value(&slate, i, Kelvin(420.0), P_ATM).unwrap())
                .collect();
            assert!(
                k[0] > k[1] && k[1] > k[2],
                "K must fall as tb rises (constant {c}), got {k:?}"
            );
        }
    }

    /// `K ∝ 1/P` at fixed `T` — Raoult's other half, and the one an envelope
    /// against a *vapour pressure* cannot see, because it divides the system
    /// pressure out before comparing.
    #[test]
    fn k_is_inversely_proportional_to_pressure() {
        let slate = slate_with_tbs(&[400.0]);
        let thermo = TroutonThermo::new();
        let at_one_atm = thermo.k_value(&slate, 0, Kelvin(380.0), P_ATM).unwrap();
        let at_five_atm = thermo
            .k_value(&slate, 0, Kelvin(380.0), P_ATM * 5.0)
            .unwrap();
        approx::assert_relative_eq!(at_five_atm * 5.0, at_one_atm, max_relative = 1e-12);
    }

    /// The states a K-value is refused at, rather than answered wrongly (rule 5).
    /// Both models share the check, so both are asserted — a guard on one of two
    /// implementations is the shape `a-fixed-plant-cannot-gate-a-second-door`
    /// records.
    #[test]
    fn an_impossible_state_is_refused_by_both_models() {
        let slate = slate_with_tbs(&[400.0, 500.0]);
        let constant_alpha = ConstantAlphaThermo::new(&slate, vec![2.0, 0.5]).unwrap();
        let models: [&dyn ThermoModel; 2] = [&TroutonThermo::new(), &constant_alpha];
        for m in models {
            let n = m.name();
            assert!(
                m.k_value(&slate, 2, Kelvin(400.0), P_ATM).is_err(),
                "{n}: a component index off the slate must be refused"
            );
            assert!(
                m.k_value(&slate, 0, Kelvin(0.0), P_ATM).is_err(),
                "{n}: a zero temperature must be refused"
            );
            assert!(
                m.k_value(&slate, 0, Kelvin(f64::NAN), P_ATM).is_err(),
                "{n}: a NaN temperature must be refused"
            );
            assert!(
                m.k_value(&slate, 0, Kelvin(400.0), Pascal(0.0)).is_err(),
                "{n}: a zero pressure must be refused"
            );
        }
    }

    /// `ConstantThermo` — the fidelity every scenario before M7.2 selects — has
    /// no vapour–liquid equilibrium, and says so instead of returning a number.
    ///
    /// This test is the arm's ONLY reader until M7.3 makes `separation =
    /// "cascade"` selectable and the loader can refuse the pairing at build
    /// time. Without it the `Err` would be a branch nothing reaches, which is
    /// `a-command-can-be-a-no-op` in a different costume.
    #[test]
    fn the_constant_thermo_refuses_a_k_value_rather_than_inventing_one() {
        let slate = slate_with_tbs(&[400.0]);
        let message = crate::ConstantThermo
            .k_value(&slate, 0, Kelvin(400.0), P_ATM)
            .expect_err("the constant fidelity has no phase equilibrium")
            .to_string();
        assert!(
            message.contains("trouton"),
            "the refusal must name a fidelity that CAN answer, got: {message}"
        );
    }

    /// The constant-α model returns its own numbers, unchanged by state — which
    /// is what a gate written against cascade algebra needs — and refuses a
    /// vector that does not match the slate.
    #[test]
    fn constant_alpha_returns_its_own_k_values_at_every_state() {
        let slate = slate_with_tbs(&[400.0, 500.0]);
        let thermo = ConstantAlphaThermo::new(&slate, vec![2.5, 0.4]).unwrap();
        for (t, p) in [(300.0, 1.0e5), (700.0, 40.0e5)] {
            assert_eq!(
                thermo.k_value(&slate, 0, Kelvin(t), Pascal(p)).unwrap(),
                2.5
            );
            assert_eq!(
                thermo.k_value(&slate, 1, Kelvin(t), Pascal(p)).unwrap(),
                0.4
            );
        }
        assert!(ConstantAlphaThermo::new(&slate, vec![2.5]).is_err());
        assert!(ConstantAlphaThermo::new(&slate, vec![2.5, 0.0]).is_err());
        assert!(ConstantAlphaThermo::new(&slate, vec![2.5, -1.0]).is_err());
    }

    /// The scaled form keeps the relative volatility EXACTLY fixed while making
    /// `K` monotone in `T`. Both halves matter and they are why the field exists:
    /// `α` fixed is what the Fenske gate rests on, and monotone `K` is what gives
    /// a cascade stage a bubble point at all.
    ///
    /// The pair of assertions is also the discrimination: a scaling applied
    /// per-component (say `k_c·(T/T_ref)^c`) would still be monotone and would
    /// still be "temperature-dependent", and would break `α` silently.
    #[test]
    fn the_scaled_form_moves_k_without_moving_alpha() {
        let slate = slate_with_tbs(&[400.0, 500.0]);
        let thermo = ConstantAlphaThermo::with_temperature_exponent(
            &slate,
            vec![4.0, 1.0],
            Kelvin(400.0),
            8.0,
        )
        .unwrap();

        let mut previous = 0.0;
        for t in [300.0, 400.0, 500.0, 600.0] {
            let light = thermo.k_value(&slate, 0, Kelvin(t), P_ATM).unwrap();
            let heavy = thermo.k_value(&slate, 1, Kelvin(t), P_ATM).unwrap();
            approx::assert_relative_eq!(light / heavy, 4.0, max_relative = 1e-13);
            assert!(
                light > previous,
                "K must rise with T: {light} after {previous}"
            );
            previous = light;
        }
        // The reference temperature is where the stored numbers are returned
        // unchanged — the anchor that makes a hand calculation possible.
        approx::assert_relative_eq!(
            thermo.k_value(&slate, 0, Kelvin(400.0), P_ATM).unwrap(),
            4.0,
            max_relative = 1e-14
        );
    }

    /// `new` is the `exponent = 0` case bit-for-bit, so every M7.2 flash gate
    /// keeps comparing with `assert_eq!` rather than a tolerance.
    #[test]
    fn the_unscaled_constructor_is_the_zero_exponent_case_exactly() {
        let slate = slate_with_tbs(&[400.0, 500.0]);
        let plain = ConstantAlphaThermo::new(&slate, vec![2.5, 0.4]).unwrap();
        let explicit = ConstantAlphaThermo::with_temperature_exponent(
            &slate,
            vec![2.5, 0.4],
            Kelvin(400.0),
            0.0,
        )
        .unwrap();
        for t in [250.0, 900.0] {
            for c in 0..2 {
                assert_eq!(
                    plain.k_value(&slate, c, Kelvin(t), P_ATM).unwrap(),
                    explicit.k_value(&slate, c, Kelvin(t), P_ATM).unwrap()
                );
            }
        }
    }

    /// The scaling's own parameters are refused when they cannot define a model.
    #[test]
    fn an_impossible_scaling_is_refused() {
        let slate = slate_with_tbs(&[400.0]);
        for (t_ref, exponent) in [
            (0.0, 1.0),
            (-400.0, 1.0),
            (f64::NAN, 1.0),
            (400.0, f64::NAN),
            (400.0, f64::INFINITY),
        ] {
            assert!(
                ConstantAlphaThermo::with_temperature_exponent(
                    &slate,
                    vec![2.0],
                    Kelvin(t_ref),
                    exponent
                )
                .is_err(),
                "t_ref = {t_ref}, exponent = {exponent} must be refused"
            );
        }
    }
}
