//! Reference cases for the FCC 4-lump kinetics (M4.2).
//!
//! `tests/reference/` is where CLAUDE.md puts the gates that pin a unit model
//! against a hand calculation or a published example, as opposed to the
//! invariant/property tests alongside it in `tests/`. What belongs here is a
//! number whose expected value comes from OUTSIDE the workspace — a standard, a
//! paper, or an analytic solution derived independently of the code — and every
//! case must say which of those it is, because they have very different ceilings.
//! (Declared as an explicit `[[test]]` target in `Cargo.toml`: Cargo only
//! auto-discovers `tests/*.rs` at the top level.)
//!
//! This file carries two KINDS of gate, and the split is deliberate:
//!
//! 1. **The published envelope** — industrial riser data. It catches what this
//!    milestone actually invites: a residence-time or catalyst-loading unit slip,
//!    or a rate constant wrong by a decade. Those miss by orders of magnitude. It
//!    catches nothing subtler, and — the part that is easy to overclaim — it does
//!    not independently validate the parameter set, because that set was FITTED
//!    to this envelope. It is a regression lock on the calibration. See the gate
//!    itself for the full statement.
//! 2. **Closed forms** — the ODE solved analytically, independently of the
//!    integrator. These catch transposed stoichiometry, a wrong constant at the
//!    percent level, a shared activation energy, and (through the order-of-
//!    convergence gate) a dropped or mis-weighted RK4 stage. They cannot catch a
//!    wrong *parameter set*, since they are told the parameters.
//!
//! Neither alone is sufficient; the first pins the physics to reality at coarse
//! resolution, the second pins the arithmetic to the model at fine resolution.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::traits::ReactionModel;
use refinery_core::units::{JPerKgK, Kelvin, KgPerM3, KgPerMol, Seconds};
use refinery_solvers::four_lump::{FourLump, FourLumpParams, RK4_SUBSTEPS, T_REF_K};

/// Slate positions. Deliberately NOT the model's internal lump order
/// (gasoil, gasoline, gas, coke) — the model resolves its lumps by NAME, and a
/// slate that happened to match its order would hide a positional mix-up.
const GAS: usize = 0;
const GASOLINE: usize = 1;
const GASOIL: usize = 2;
const COKE: usize = 3;

fn fcc_slate() -> Slate {
    let cut = |name: &str, tb: f64, mm: f64, rho: f64, cp: f64| PseudoComponent {
        name: name.into(),
        tb: Kelvin(tb),
        molar_mass: KgPerMol(mm),
        density: Some(KgPerM3(rho)),
        phase: Phase::Liquid,
        cp: JPerKgK(cp),
    };
    Slate::new(vec![
        cut("gas", 233.15, 0.030, 400.0, 2500.0),
        cut("gasoline", 373.15, 0.100, 720.0, 2200.0),
        cut("gasoil", 673.15, 0.300, 900.0, 2000.0),
        cut("coke", 1173.15, 0.600, 1200.0, 1500.0),
    ])
    .expect("the four FCC lumps must form a slate")
}

/// A parameter set with NO temperature dependence and no gasoline cracking, so
/// the closed form below is elementary and exact. `h_form` is zeroed too: these
/// gates are about the composition trajectory, and the heat of reaction has its
/// own gates in `four_lump.rs`.
fn gasoil_only_params() -> FourLumpParams {
    FourLumpParams {
        k_ref: [0.8, 0.3, 0.1, 0.0, 0.0],
        e_act: [0.0; 5],
        alpha_ref: 0.15,
        e_alpha: 0.0,
        h_form: [0.0; 4],
    }
}

/// Tolerance for the closed-form gates, DERIVED from measurement rather than
/// tuned until green. The floor is RK4 truncation at the shipped substep count,
/// measured by `the_integrator_converges_at_fourth_order` below: 5.6e-9 on the
/// 800 K case and 5.4e-8 at 900 K, where the rates are faster and the same step
/// buys less. This sits ~20x above the worst of those and ~4 orders below the
/// smallest defect these gates exist to catch (a transposed split moves a
/// fraction by ~0.1), so the headroom costs no discrimination.
const CLOSED_FORM_TOL: f64 = 1e-6;

/// `θ(τ) = ∫₀^τ exp(−α t) dt = (1 − e^{−ατ})/α` — the reparametrization that
/// makes Weekman's exponential catalyst decay a pure change of the residence
/// coordinate. Written here from the integral, not read from the model.
fn theta(alpha: f64, tau: f64) -> f64 {
    (1.0 - (-alpha * tau).exp()) / alpha
}

/// **Gate 1 — the published anchor.** The calibrated set, at a riser outlet
/// temperature and residence time in the published range, must reproduce the
/// industrial product distribution.
///
/// Source: Olufemi, Latinwo & Olukayode, *Riser Reactor Simulation in a Fluid
/// Catalytic Cracking Unit*, Chemical and Process Engineering Research 7 (2013)
/// 12–21, Tables 1–4, reproducing four industrial cases of Ali & Rohani (1997):
/// riser outlet 795–808 K, catalyst-to-oil 5.43–7.20 kg/kg, gasoline
/// 41.78–46.90 wt%, coke 5.34–5.83 wt%, and (Table 1) 79.0 wt% gas oil
/// conversion with 46.0 wt% gasoline.
///
/// **What this gate proves, and what it does not.** The constants in
/// `FourLumpParams::fcc` were CALIBRATED to this envelope, and this gate then
/// checks they land in it — one published anchor used twice. So it is a
/// **regression lock on the calibration, not an independent validation of the
/// parameter set**: it fails loudly if a later change moves the model out of the
/// band (a `tau` unit slip, a dropped catalyst-loading fold, a refactor that
/// scales a rate constant — all of which miss by decades, not percent), and it
/// says nothing about whether these five constants are the right five. Only a
/// point match against an independently tabulated set could say that, and the
/// sources carrying one were unreachable (see the module note). This is the same
/// ceiling `scenarios/tests/kv_reference.rs` names for M1's network hand calc.
///
/// The catalyst-to-oil ratio is folded into the rate constants rather than
/// carried as a reactor parameter, so this gate is stated at the plant's COR by
/// construction and cannot discriminate a COR change.
///
/// One band is weaker than the others and is marked as such below: light gases
/// are not tabulated in the source at all.
#[test]
fn the_calibrated_set_lands_in_the_published_plant_envelope() {
    let slate = fcc_slate();
    let model = FourLump::fcc(&slate).expect("the calibrated set must build on the FCC slate");
    let feed = Composition::pure(slate.len(), GASOIL);

    // A riser outlet temperature inside the published 795–808 K band, and a
    // residence time inside the 1–5 s a modern riser runs at.
    let out = model
        .react(&feed, Kelvin(800.0), Seconds(3.0), &slate)
        .expect("the calibrated set must react a pure gasoil feed");
    let y = out.products.fractions();
    let conversion = 1.0 - y[GASOIL];

    let band = |what: &str, value: f64, lo: f64, hi: f64| {
        assert!(
            value >= lo && value <= hi,
            "{what} = {:.2} wt% is outside the published plant envelope [{:.1}, {:.1}] wt% \
             — a residence-time or catalyst-loading unit slip lands decades away from this band",
            value * 100.0,
            lo * 100.0,
            hi * 100.0
        );
    };
    // Bands are the plant spread widened to the nearest whole percent, not
    // tuned to the model: gasoline 41.78–46.90 → [40, 50], coke 5.34–5.83 →
    // [4, 7], conversion 79.0 → [70, 85]. Those three are tabulated values.
    band("gasoline", y[GASOLINE], 0.40, 0.50);
    band("coke", y[COKE], 0.04, 0.07);
    band("gas oil conversion", conversion, 0.70, 0.85);
    // The light-gas band is the LEAST supported of the four and is deliberately
    // the widest. The source tabulates no light-gas yield; ~29 wt% is a residual
    // (100 − gasoline − coke − unconverted) spliced across two of its tables,
    // and one of those, Table 1, is internally inconsistent — it lists coke at
    // 30.0 wt% alongside 79.0 wt% conversion, which cannot both hold, and Table 2
    // gives 5.60 for the same quantity. Read this assertion as "light gases are a
    // major product, not a trace", which is all the source will actually support.
    band("light gases", y[GAS], 0.22, 0.35);

    // Endothermic, at the few-hundred-kJ/kg magnitude quoted for FCC cracking.
    // Sign convention: positive = heat absorbed (`Reaction::dh_rxn`).
    let dh = out.dh_rxn.value();
    assert!(
        (2.0e5..=5.0e5).contains(&dh),
        "cracking must absorb a few hundred kJ per kg of feed, got {dh} J/kg"
    );
}

/// **Gate 2 — the closed form for second-order gas oil cracking under
/// exponential catalyst decay**, with the gasoline paths switched off so the
/// solution is elementary.
///
/// With `dy₁/dt = −K·y₁²·φ(t)` and `φ = e^{−αt}`, substituting `θ = ∫φ` gives
/// `dy₁/dθ = −K y₁²`, hence `1/y₁ = 1/y₁(0) + Kθ`. Each product lump then
/// receives its own path's share of the converted gas oil, `kᵢ/K·(1 − y₁)`,
/// because all three are fed by the same `y₁²` term.
///
/// This is derived from the rate law, not read back from the code: it pins the
/// SECOND-ORDER law (a first-order slip gives exponential decay, a different
/// number entirely), the θ reparametrization, and the split ratios — swapping
/// the gas and coke constants fails it while conserving mass exactly.
#[test]
fn second_order_gasoil_decay_matches_the_closed_form() {
    let slate = fcc_slate();
    let params = gasoil_only_params();
    let model =
        FourLump::with_params(&slate, params.clone()).expect("the test set must build a model");

    let tau = 3.0;
    let k = params.k_ref;
    let total = k[0] + k[1] + k[2];
    let th = theta(params.alpha_ref, tau);
    let gasoil = 1.0 / (1.0 + total * th);
    let converted = 1.0 - gasoil;

    let out = model
        .react(
            &Composition::pure(slate.len(), GASOIL),
            Kelvin(T_REF_K),
            Seconds(tau),
            &slate,
        )
        .expect("the test set must react");
    let y = out.products.fractions();

    const TOL: f64 = CLOSED_FORM_TOL;
    assert!(
        (y[GASOIL] - gasoil).abs() < TOL,
        "gas oil must follow 1/(1 + Kθ) = {gasoil}, got {}",
        y[GASOIL]
    );
    assert!(
        (y[GASOLINE] - k[0] / total * converted).abs() < TOL,
        "gasoline must take k1/K of the converted gas oil ({}), got {}",
        k[0] / total * converted,
        y[GASOLINE]
    );
    assert!(
        (y[GAS] - k[1] / total * converted).abs() < TOL,
        "light gases must take k2/K of the converted gas oil ({}), got {}",
        k[1] / total * converted,
        y[GAS]
    );
    assert!(
        (y[COKE] - k[2] / total * converted).abs() < TOL,
        "coke must take k3/K of the converted gas oil ({}), got {}",
        k[2] / total * converted,
        y[COKE]
    );
}

/// **Gate 3 — the closed form for FIRST-order gasoline cracking.** A pure
/// gasoline feed leaves the gas oil paths inert (`y₁ = 0`), so the gasoline
/// balance is linear: `dy₂/dθ = −(k₄+k₅)y₂`, giving `y₂ = e^{−(k₄+k₅)θ}`.
///
/// It needs its own case because gate 2 cannot see the gasoline paths at all —
/// they are switched off there — and because the order of the gasoline law is
/// exactly what the source paper misprints (`x₃²` in Olufemi et al.'s eqs.
/// (15)–(16), contradicting its own stated first-order assumption). A second-
/// order gasoline law would pass gate 2 and the mass gates untouched.
#[test]
fn first_order_gasoline_cracking_matches_the_closed_form() {
    let slate = fcc_slate();
    let params = FourLumpParams {
        k_ref: [0.8, 0.3, 0.1, 0.2, 0.05],
        e_act: [0.0; 5],
        alpha_ref: 0.15,
        e_alpha: 0.0,
        h_form: [0.0; 4],
    };
    let model =
        FourLump::with_params(&slate, params.clone()).expect("the test set must build a model");

    let tau = 3.0;
    let (to_gas, to_coke) = (params.k_ref[3], params.k_ref[4]);
    let cracked = to_gas + to_coke;
    let th = theta(params.alpha_ref, tau);
    let gasoline = (-cracked * th).exp();
    let converted = 1.0 - gasoline;

    let out = model
        .react(
            &Composition::pure(slate.len(), GASOLINE),
            Kelvin(T_REF_K),
            Seconds(tau),
            &slate,
        )
        .expect("the test set must react a pure gasoline feed");
    let y = out.products.fractions();

    const TOL: f64 = CLOSED_FORM_TOL;
    assert!(
        (y[GASOLINE] - gasoline).abs() < TOL,
        "gasoline must decay first order to exp(-(k4+k5)θ) = {gasoline}, got {}",
        y[GASOLINE]
    );
    assert!(
        (y[GAS] - to_gas / cracked * converted).abs() < TOL,
        "light gases must take k4/(k4+k5) of the cracked gasoline ({}), got {}",
        to_gas / cracked * converted,
        y[GAS]
    );
    assert!(
        (y[COKE] - to_coke / cracked * converted).abs() < TOL,
        "coke must take k5/(k4+k5) of the cracked gasoline ({}), got {}",
        to_coke / cracked * converted,
        y[COKE]
    );
    assert!(
        y[GASOIL].abs() < TOL,
        "a pure gasoline feed makes no gas oil, got {}",
        y[GASOIL]
    );
}

/// **Gate 4 — Arrhenius scaling about the reference temperature**, with a
/// DIFFERENT activation energy on every path.
///
/// `k(T) = k_ref·exp(−E/R·(1/T − 1/T_ref))`, so at `T ≠ T_ref` both the total
/// rate and the SPLIT between paths move. Distinct energies are the point: a
/// model that applied one shared activation energy (or applied the shift to the
/// sum rather than to each constant) would still scale the total correctly and
/// would still conserve mass, but the split ratios would be frozen at their
/// reference values, and only this gate sees that.
#[test]
fn rate_constants_scale_by_arrhenius_about_the_reference_temperature() {
    const R_GAS: f64 = 8.314_462_618;
    let slate = fcc_slate();
    let params = FourLumpParams {
        k_ref: [0.8, 0.3, 0.1, 0.0, 0.0],
        e_act: [40.0e3, 80.0e3, 20.0e3, 0.0, 0.0],
        alpha_ref: 0.15,
        e_alpha: 0.0, // isolate the rate constants from the decay coefficient
        h_form: [0.0; 4],
    };
    let model =
        FourLump::with_params(&slate, params.clone()).expect("the test set must build a model");

    let (temperature, tau) = (900.0, 3.0);
    let shifted: Vec<f64> = (0..3)
        .map(|i| {
            params.k_ref[i] * (-params.e_act[i] / R_GAS * (1.0 / temperature - 1.0 / T_REF_K)).exp()
        })
        .collect();
    let total: f64 = shifted.iter().sum();
    let th = theta(params.alpha_ref, tau);
    let gasoil = 1.0 / (1.0 + total * th);
    let converted = 1.0 - gasoil;

    let out = model
        .react(
            &Composition::pure(slate.len(), GASOIL),
            Kelvin(temperature),
            Seconds(tau),
            &slate,
        )
        .expect("the test set must react above the reference temperature");
    let y = out.products.fractions();

    const TOL: f64 = CLOSED_FORM_TOL;
    assert!(
        (y[GASOIL] - gasoil).abs() < TOL,
        "gas oil at {temperature} K must follow the shifted rate constants ({gasoil}), got {}",
        y[GASOIL]
    );
    assert!(
        (y[COKE] - shifted[2] / total * converted).abs() < TOL,
        "the coke split must use the coke path's OWN activation energy ({}), got {}",
        shifted[2] / total * converted,
        y[COKE]
    );
    // The shift is not cosmetic: at 900 K the low-E coke path loses share to the
    // high-E gas path relative to the reference temperature. Asserting the
    // DIRECTION as well as the value means a sign slip in the exponent fails
    // here for a reason a reader can see.
    let reference_split = params.k_ref[2] / (params.k_ref[0] + params.k_ref[1] + params.k_ref[2]);
    assert!(
        shifted[2] / total < reference_split,
        "raising T must shift selectivity AWAY from the lowest-activation-energy path"
    );
}

/// **Gate 5 — the integrator's order of convergence.** The closed-form gates
/// above pass at the shipped substep count with room to spare, so they cannot
/// tell a correct RK4 from a degraded one; this gate can.
///
/// Halving the step must cut the error by ~2⁴ = 16 for a fourth-order method. A
/// dropped stage, a mis-weighted `(a + 2b + 2c + d)/6`, or a stage evaluated at
/// the wrong time collapses that to first or second order and fails here, while
/// leaving mass conservation and the coarse envelope untouched.
///
/// It also MEASURES what the shipped count buys, so the tolerances above are
/// derived rather than tuned: the error at [`RK4_SUBSTEPS`] is asserted to be
/// far below them.
///
/// The step range matters and was measured, not assumed. Below 32 substeps this
/// problem is NOT in the asymptotic regime — the error changes sign between 8
/// and 16 steps, so the naive `error(4)/error(8)` ratio reads 245 and means
/// nothing. Measured errors on the gas oil fraction: 9.6e-7 (16), 8.3e-8 (32),
/// 5.6e-9 (64), 3.6e-10 (128), i.e. ratios 11.5, 14.7, 15.5 climbing to 16.
#[test]
fn the_integrator_converges_at_fourth_order() {
    let slate = fcc_slate();
    let params = gasoil_only_params();
    let tau = 3.0;
    let total = params.k_ref[0] + params.k_ref[1] + params.k_ref[2];
    let exact = 1.0 / (1.0 + total * theta(params.alpha_ref, tau));

    let error_at = |substeps: usize| {
        let model = FourLump::with_substeps(&slate, params.clone(), substeps)
            .expect("the test set must build at any substep count");
        let out = model
            .react(
                &Composition::pure(slate.len(), GASOIL),
                Kelvin(T_REF_K),
                Seconds(tau),
                &slate,
            )
            .expect("the test set must react");
        (out.products.fractions()[GASOIL] - exact).abs()
    };

    // Inside the asymptotic range, and still coarse enough that truncation, not
    // round-off, dominates.
    let (e32, e64, e128) = (error_at(32), error_at(64), error_at(128));
    for (steps, ratio) in [(64, e32 / e64), (128, e64 / e128)] {
        assert!(
            (12.0..=20.0).contains(&ratio),
            "halving the step to {steps} substeps must cut the error ~16x for a fourth-order \
             method; got {ratio}x — the RK4 stage weights or times are wrong. A second-order \
             method would read ~4 here and a first-order one ~2."
        );
    }

    // What the shipped count actually delivers, and why the closed-form gates
    // above can carry their tolerance honestly. Measured: 5.6e-9.
    let shipped = error_at(RK4_SUBSTEPS);
    assert!(
        shipped < CLOSED_FORM_TOL / 20.0,
        "at {RK4_SUBSTEPS} substeps the truncation error must sit well below the \
         {CLOSED_FORM_TOL} the closed-form gates allow; measured {shipped}"
    );
}
