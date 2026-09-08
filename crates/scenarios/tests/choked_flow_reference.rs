//! Choked gas flow through a control valve, as wired (docs/ROADMAP.md §M5.4b,
//! docs/DESIGN.md §3a forks 4 and 6).
//!
//! **What each gate is worth, stated up front, because they are not alike.**
//!
//! DESIGN §3a originally promised this file "an independent published anchor,
//! exactly as `kv_reference` anchors M1 — its expected value comes from the
//! standard, not from any formula in the workspace." **That was an overclaim and
//! fork 6 corrects it before this file exists.** `kv_reference` earns that status
//! because the `Kv` *definition* is a physical statement — a `Kv` valve passes
//! `Kv` m³/h of water at 1 bar, SG 1 — from which the test derives `cv_si` by a
//! route the workspace does not contain. There is no analogous non-formula
//! statement behind `Y = 1 − x/(3·F_k·x_T)`. So `the_choked_flow_matches_the_isa_
//! sizing_equation_at_two_x_t` below is a transcription/units/algebra check —
//! `kv_reference`'s *network hand calc* ceiling — and is labelled as one.
//!
//! What carries content the implementation cannot supply to itself:
//!
//! - **the plateau** (`a_choked_valve_is_insensitive_to_downstream_pressure`) —
//!   a PROPERTY no unchoked law has, asserted on the folded branch with real pipe
//!   resistance either side, and paired with an unchoked control so it cannot
//!   pass by the plant simply being insensitive;
//! - **`F_k` derived from the slate** (`the_choke_point_moves_with_the_slates_
//!   heat_capacity_ratio`) — nothing declares it, and changing `cp` alone moves
//!   the choked rate by the predicted `√F_k`;
//! - `Y = 2/3` at the choke and the `Y → 1` degeneracy onto the liquid branch,
//!   which live in `solvers::elements::tests` because they are statements about
//!   the element and need no plant.
//!
//! The equation FORM — `Y = 1 − x/(3·F_k·x_T)`, `F_k = γ/1.40`, choke at
//! `x = F_k·x_T` where `Y = 2/3` — has been checked against sources outside this
//! repository. **The standard itself was not read**; its text is paywalled. That
//! is the honest statement, and it is why the gates above are the ones leaned on.

use refinery_core::graph::{EdgeId, PlantGraph};
use refinery_core::traits::FlowSolver;
use refinery_core::units::Seconds;
use refinery_scenarios::NodeDef;
use refinery_solvers::{NewtonFlowSolver, SimpleFlowSolver};

const PLANT: &str = include_str!("../../../scenarios/gas_valve.toml");

// --- The plant's declared constants, restated rather than read back -----------

/// CODATA universal gas constant [J/(mol·K)].
const R: f64 = 8.314_462_618_153_24;
/// Methane's molar mass [kg/mol], as `gas_valve.toml` declares it.
const M_BAR: f64 = 0.016_043;
/// The declared heat capacity [J/(kg·K)].
const CP: f64 = 2220.0;
/// The declared valve size [m³/h at 1 bar, SG 1] and its `x_T`.
const KV: f64 = 25.0;
const X_T: f64 = 0.72;
const P_HEADER_PA: f64 = 10.0e5;
/// Both pipes, as declared, and the friction factor the loader defaults to.
const PIPE_L: f64 = 10.0;
const PIPE_D: f64 = 0.15;
const FRICTION: f64 = 0.02;
/// Seed stream temperature [K] = the header's declared 20 °C. Every gate here
/// solves the FRESHLY BUILT graph, `gas_density_reference`'s convention and for
/// the same reason plus one more: M5.1's dissipation puts ~130 kW into ~1 kg/s of
/// gas across this valve, and on a ticked plant the valve node's resolved
/// temperature would then depend on the downstream pressure — which would make
/// the plateau approximate rather than exact and blunt the gate that matters most.
const T_SEED_K: f64 = 293.15;
/// The reference density for `ρ_rel`, matching `network::RHO_WATER_REF`.
const RHO_REF: f64 = 998.0;

/// Tolerance [relative]. Derived the way `gas_density_reference`'s is: the
/// `eps_dp = 1.0` Pa regularisation against the ~900 kPa valve drop is ~6e-7, and
/// the inlet run's ~300 Pa drop is the small one at ~2e-3 of ITS own scale but
/// enters the answer only through `P₁`, a 3e-4 Pa effect. Measured deviation:
/// 1.6e-6 (newton) and 1.1e-5 (simple, whose looser `tol_rel` dominates).
const TOLERANCE: f64 = 1e-4;

/// `γ = cp/cv` with `cv = cp − R/M̄` for a gas — M5.3's phase-conditional rule,
/// restated here rather than called, so the test does not read its expectation
/// back out of the object under test.
fn gamma() -> f64 {
    CP / (CP - R / M_BAR)
}

/// `cv_si` from metric `Kv`, derived from the Kv DEFINITION exactly as
/// `kv_reference` does rather than by calling the loader's converter: `Kv` m³/h
/// at 1 bar, SG 1 ⇒ `cv_si = Kv/(3600·√1e5)`.
fn cv_si() -> f64 {
    KV / (3600.0 * 1e5_f64.sqrt())
}

fn pipe_k(rho: f64) -> f64 {
    let area = std::f64::consts::PI * PIPE_D * PIPE_D / 4.0;
    FRICTION * PIPE_L * rho / (2.0 * PIPE_D * area * area)
}

/// The choked mass flow of the whole plant, in closed form.
///
/// Past the choke the valve passes `ṁ = Cv·(2/3)·√(ρ_ref·ρ₁·x_c·P₁)` with
/// `ρ₁ = P₁·M̄/(R·T)`, so `ṁ = A·P₁` with `A` a constant. `P₁` is the header
/// pressure less the inlet run's drop at that same flow, and the inlet run takes
/// its density at the HEADER (its own upwind end), so
///
/// ```text
/// ṁ = A·(P_header − k_in·(ṁ/ρ_header)²)
/// ```
///
/// a quadratic in `ṁ` with one positive root. Note the downstream pressure does
/// not appear — that is the plateau, and it is why this function takes no `P₂`.
fn choked_prediction(x_t: f64, cp: f64) -> f64 {
    let gamma = cp / (cp - R / M_BAR);
    let x_choke = (gamma / 1.40) * x_t;
    let rho_header = P_HEADER_PA * M_BAR / (R * T_SEED_K);
    let a = cv_si() * (2.0 / 3.0) * (RHO_REF * x_choke * M_BAR / (R * T_SEED_K)).sqrt();
    let k_in = pipe_k(rho_header);
    let (qa, qb, qc) = (k_in * a / (rho_header * rho_header), 1.0, -a * P_HEADER_PA);
    (-qb + (qb * qb - 4.0 * qa * qc).sqrt()) / (2.0 * qa)
}

fn build_from(file: &refinery_scenarios::ScenarioFile) -> refinery_core::engine::Engine {
    refinery_scenarios::build_engine(file).expect("gas_valve.toml builds")
}

fn load() -> refinery_scenarios::ScenarioFile {
    refinery_scenarios::load_str(PLANT).expect("gas_valve.toml parses")
}

fn set_x_t(file: &mut refinery_scenarios::ScenarioFile, value: Option<f64>) {
    match file.nodes.get_mut("control_valve") {
        Some(NodeDef::Valve { x_t, .. }) => *x_t = value,
        other => panic!("gas_valve.toml must define control_valve as a valve, got {other:?}"),
    }
}

fn set_flare(file: &mut refinery_scenarios::ScenarioFile, bar: f64) {
    match file.nodes.get_mut("flare") {
        Some(NodeDef::Sink { pressure_bar, .. }) => *pressure_bar = bar,
        other => panic!("gas_valve.toml must define flare as a sink, got {other:?}"),
    }
}

fn edge_by_name(graph: &PlantGraph, name: &str) -> EdgeId {
    graph
        .edge_ids()
        .find(|e| graph.pipe(*e).name == name)
        .unwrap_or_else(|| panic!("plant must have a '{name}' pipe"))
}

/// Solve the freshly built graph and report the valve's throughput.
fn solve(file: &refinery_scenarios::ScenarioFile, fidelity: &str) -> f64 {
    let engine = build_from(file);
    let mut newton = NewtonFlowSolver::default();
    let mut simple = SimpleFlowSolver::default();
    let solver: &mut dyn FlowSolver = match fidelity {
        "newton" => &mut newton,
        _ => &mut simple,
    };
    let sol = solver
        .solve(
            &engine.graph,
            &engine.slate,
            &Default::default(),
            Seconds(0.1),
        )
        .unwrap_or_else(|e| panic!("{fidelity} must converge on gas_valve: {e}"));
    sol.edge_mass_flow[&edge_by_name(&engine.graph, "outlet_run")]
}

// ---------------------------------------------------------------------------
// A — magnitude. A transcription/units/algebra check, NOT an independent anchor.
// ---------------------------------------------------------------------------

/// The choked rate matches the ISA sizing equation, at **two `x_T` values**.
///
/// Two values rather than one is the anti-circularity move M5.2 makes with
/// pressure: `ṁ_choked ∝ √(F_k·x_T)`, so `x_T = 0.72 → 0.38` must take the flow
/// down by `√(0.38/0.72) = 0.7265`, and no `x_T`-independent implementation can
/// sit on both numbers. What it does NOT do is validate that 0.72 is the right
/// figure for any real valve — nothing in this workspace could, and the plant
/// file marks its citation as secondary.
///
/// The ceiling of the magnitude itself is `kv_reference`'s network-hand-calc one:
/// the closed form mirrors the model's own series algebra, so it catches wrong
/// constants, unit slips and a wrong evaluation state — not an error in the
/// formulation. The gates in sections B and C are what do not have that ceiling.
#[test]
fn the_choked_flow_matches_the_isa_sizing_equation_at_two_x_t() {
    let mut previous: Option<(f64, f64)> = None;
    for x_t in [0.72f64, 0.38] {
        let mut file = load();
        set_x_t(&mut file, Some(x_t));
        let expected = choked_prediction(x_t, CP);
        for fidelity in ["newton", "simple"] {
            let flow = solve(&file, fidelity);
            assert!(
                flow > 0.0,
                "{fidelity} @ x_T {x_t}: gas must flow header → flare, got {flow}"
            );
            approx::assert_relative_eq!(flow, expected, max_relative = TOLERANCE);
        }
        if let Some((x_prev, flow_prev)) = previous {
            // The DEPENDENCE, measured rather than assumed: ṁ ∝ √(x_T).
            let ratio = solve(&file, "newton") / flow_prev;
            approx::assert_relative_eq!(ratio, (x_t / x_prev).sqrt(), max_relative = 1e-3);
        }
        previous = Some((x_t, solve(&file, "newton")));
    }
}

/// The plant really is past its choke point — otherwise every gate in this file
/// is measuring the unchoked branch and says nothing about choking at all.
///
/// Asserted from the plant's own numbers rather than from the file's comment:
/// `x = (P₁ − P₂)/P₁` against `F_k·x_T`, with `P₁` the valve node's solved
/// pressure.
#[test]
fn the_reference_plant_is_genuinely_choked() {
    let engine = build_from(&load());
    let valve = engine
        .graph
        .find_node("control_valve")
        .expect("plant has control_valve");
    let flare = engine.graph.find_node("flare").expect("plant has flare");
    let sol = NewtonFlowSolver::default()
        .solve(
            &engine.graph,
            &engine.slate,
            &Default::default(),
            Seconds(0.1),
        )
        .expect("converges");
    let p1 = sol.node_pressure[&valve].value();
    let p2 = sol.node_pressure[&flare].value();
    let x = (p1 - p2) / p1;
    let x_choke = (gamma() / 1.40) * X_T;
    assert!(
        x > 1.2 * x_choke,
        "the reference plant must sit well past its choke: x = {x:.4}, \
         F_k·x_T = {x_choke:.4}. If a change to the plant put it back below, \
         this slice has stopped being tested."
    );
}

// ---------------------------------------------------------------------------
// B — the plateau. A property, not a magnitude.
// ---------------------------------------------------------------------------

/// Below the critical ratio the flow stops responding to downstream pressure.
///
/// This is the gate an unclamped `Y` fails and no incompressible law can pass:
/// halving the flare pressure from 1.0 to 0.5 bar changes the branch drop by 11%
/// and must change the flow by nothing. The pipes either side are real
/// resistance, so this is a statement about the FOLDED branch — the extra ΔP goes
/// into the valve's own share while `ṁ` stays pinned by `P₁` and `F_k·x_T`.
///
/// The unchoked control is what stops it passing for the wrong reason: at 6.0 and
/// 5.0 bar downstream the same plant is below its choke point, and there the flow
/// MUST move. A model that had simply lost its downstream sensitivity would fail
/// that half.
#[test]
fn a_choked_valve_is_insensitive_to_downstream_pressure() {
    let flow_at = |bar: f64| {
        let mut file = load();
        set_flare(&mut file, bar);
        solve(&file, "newton")
    };

    // Choked: 1.0 bar vs 0.5 bar.
    let (deep, deeper) = (flow_at(1.0), flow_at(0.5));
    let moved = (deeper - deep).abs() / deep;
    assert!(
        moved < TOLERANCE,
        "a choked valve must not respond to downstream pressure: 1.0 bar gives \
         {deep:.9} kg/s, 0.5 bar gives {deeper:.9} kg/s, moved {moved:.3e}"
    );

    // Unchoked control: 7.0 bar vs 4.0 bar, i.e. x = 0.30 and x = 0.60, both
    // comfortably below the critical 0.671 (equivalently, both above the 3.29 bar
    // floor that ratio implies), where the flow is pressure-driven again.
    let (mild, less_mild) = (flow_at(7.0), flow_at(4.0));
    let responds = (less_mild - mild).abs() / mild;
    assert!(
        responds > 0.05,
        "premise: below the choke the flow must respond to downstream pressure, \
         or the gate above passes for the wrong reason. 6.0 bar gives {mild:.9}, \
         5.0 bar gives {less_mild:.9}, moved {responds:.3e}"
    );
    assert!(
        less_mild > mild,
        "and it must respond in the right direction: more drop, more flow"
    );
}

/// `P₁` is the UPWIND node's pressure, not the edge's source node's.
///
/// The fold-at-source convention puts `src` in the same few lines as the valve
/// being folded, and `src` is the valve's inlet **only while the flow runs
/// forward** — so this is the slip the code invites, and no forward-flowing plant
/// can see it. Reversing the plant separates them by an order of magnitude: with
/// the header at 1 bar and the flare at 10, gas runs flare → valve → header, the
/// `outlet_run` edge's upwind end is the FLARE at 10 bar, and `src` is the valve
/// node sitting at ~1 bar. Reading `src` would size the choke against a tenth of
/// the real inlet pressure.
///
/// The expected magnitude needs no readback: past the choke `ṁ = A·P₁` with `A`
/// independent of pressure, and in reverse `P₁` is the flare — a pinned
/// reservoir with no pipe between it and the valve — so it is exactly
/// `A·10 bar`. Forward it is `A·P_valve`, slightly less because the inlet run
/// drops ~300 Pa first, which is why the two differ by a predictable 1.0003 and
/// not at all otherwise.
///
/// This also exercises the STATED LIMITATION (DESIGN §3a fork 6): a gas valve
/// chokes symmetrically, passing reverse flow as readily as forward. Right for a
/// control valve, wrong for a PSV, and recorded in the same register as fork 5's
/// "no blowdown hysteresis".
#[test]
fn the_sizing_pressure_is_the_upwind_nodes_not_the_edge_sources() {
    let mut file = load();
    match file.nodes.get_mut("header") {
        Some(NodeDef::Source { pressure_bar, .. }) => *pressure_bar = 1.0,
        other => panic!("expected header to be a source, got {other:?}"),
    }
    set_flare(&mut file, 10.0);

    let reversed = solve(&file, "newton");
    assert!(
        reversed < 0.0,
        "the reversed plant must drive gas flare → header, got {reversed} kg/s"
    );

    // ṁ = A·P₁ with P₁ = the flare's pinned 10 bar, exactly.
    let x_choke = (gamma() / 1.40) * X_T;
    let a = cv_si() * (2.0 / 3.0) * (RHO_REF * x_choke * M_BAR / (R * T_SEED_K)).sqrt();
    approx::assert_relative_eq!(reversed.abs(), a * 10.0e5, max_relative = TOLERANCE);

    // And it is measurably NOT the `src` reading, which would be ~10x smaller —
    // stated so a pass cannot be a coincidence of two nearby numbers.
    let forward = solve(&load(), "newton");
    let separation = reversed.abs() / forward;
    assert!(
        (1.0002..1.0005).contains(&separation),
        "reverse and forward must differ only by the inlet run's drop; got a \
         ratio of {separation:.6}"
    );
}

// ---------------------------------------------------------------------------
// C — F_k is derived from the slate, not declared.
// ---------------------------------------------------------------------------

/// Changing the slate's heat capacity alone moves the choke point.
///
/// `F_k = γ/1.40` with `γ = cp/(cp − R/M̄)`, so `cp` is the only thing that moves
/// here — the density, the valve and the geometry are untouched, and `cp` does
/// not enter the sizing equation any other way at this fidelity. The choked flow
/// must therefore scale as `√F_k`, which pins that `F_k` comes from the
/// composition rather than from a constant somewhere in the solver.
///
/// A hardcoded `F_k = 1` (the plausible slip: treating every gas as air) predicts
/// no movement at all, and is 3.5% off on the reference cut before this
/// substitution is even made.
#[test]
fn the_choke_point_moves_with_the_slates_heat_capacity_ratio() {
    let heavy_cp = 5000.0;
    let mut file = load();
    match file.components.first_mut() {
        Some(c) => c.cp_j_per_kg_k = Some(heavy_cp),
        None => panic!("gas_valve.toml must declare a component"),
    }
    let flow = solve(&file, "newton");
    let expected = choked_prediction(X_T, heavy_cp);
    approx::assert_relative_eq!(flow, expected, max_relative = TOLERANCE);

    // The MOVEMENT, so the gate is not just a second magnitude check: √(F_k'/F_k).
    let base = solve(&load(), "newton");
    let g_heavy = heavy_cp / (heavy_cp - R / M_BAR);
    let ratio = (g_heavy / gamma()).sqrt();
    approx::assert_relative_eq!(flow / base, ratio, max_relative = 1e-3);
    assert!(
        (1.0 - ratio).abs() > 0.05,
        "premise: the two heat capacities must give materially different choke \
         points, got a ratio of {ratio:.4}"
    );
}

// ---------------------------------------------------------------------------
// D — loader guards. Each refused for its own reason, and each direction.
// ---------------------------------------------------------------------------

fn expect_refusal(file: &refinery_scenarios::ScenarioFile, what: &str) -> String {
    match refinery_scenarios::build_engine(file) {
        Ok(_) => panic!("{what}"),
        Err(e) => e.to_string(),
    }
}

/// A valve in gas service with no `x_t` is refused — no silent default.
///
/// A default would be an invented value in disguise: both the sizing gate and the
/// plateau gate would pass for whatever it was, which is exactly the circularity
/// that defers pump `η` (DESIGN §3a fork 4).
#[test]
fn a_gas_valve_without_x_t_is_refused() {
    let mut file = load();
    set_x_t(&mut file, None);
    let msg = expect_refusal(&file, "a gas valve with no x_t must not load");
    assert!(
        msg.contains("x_t") && msg.contains("gas"),
        "the refusal must name the missing field and the service: {msg}"
    );
}

/// A valve in LIQUID service carrying an `x_t` is refused too.
///
/// The other direction, enforced for the reason that keeps a "gas Cv" out of the
/// milestone: a number nothing reads is how an author comes to believe the model
/// uses something it does not. `PseudoComponent::density` is guarded both ways
/// for the same reason.
#[test]
fn a_liquid_valve_declaring_x_t_is_refused() {
    let mut file =
        refinery_scenarios::load_str(include_str!("../../../scenarios/tank_pump_valve.toml"))
            .expect("tank_pump_valve.toml parses");
    let valve = file
        .nodes
        .iter_mut()
        .find_map(|(_, def)| match def {
            NodeDef::Valve { x_t, .. } => Some(x_t),
            _ => None,
        })
        .expect("tank_pump_valve has a valve");
    *valve = Some(0.72);
    let msg = expect_refusal(&file, "a liquid valve declaring x_t must not load");
    assert!(
        msg.contains("liquid") && msg.contains("x_t"),
        "the refusal must name the service and the field: {msg}"
    );
}

/// `x_t` outside (0, 1) is refused: it is a fraction of the inlet absolute
/// pressure, and a non-positive one would put the choke point at or below zero
/// drop — where `expansion` would divide by it.
#[test]
fn an_out_of_range_x_t_is_refused() {
    for bad in [0.0f64, -0.5, 1.0, 3.0] {
        let mut file = load();
        set_x_t(&mut file, Some(bad));
        let msg = expect_refusal(&file, &format!("x_t = {bad} must not load"));
        assert!(
            msg.contains("x_t"),
            "the refusal must name the field, got: {msg}"
        );
    }
}

/// Measurement, not a gate: the deviations `TOLERANCE` is sized from.
/// `cargo test -p refinery-scenarios --test choked_flow_reference -- \
/// --ignored --nocapture`
#[test]
#[ignore = "measurement, not a gate"]
fn measure_choked_flow_headroom() {
    for x_t in [0.72f64, 0.38] {
        let mut file = load();
        set_x_t(&mut file, Some(x_t));
        let expected = choked_prediction(x_t, CP);
        for fidelity in ["newton", "simple"] {
            let flow = solve(&file, fidelity);
            println!(
                "x_T {x_t:.2} {fidelity:>6}: flow {flow:.9} kg/s, predicted {expected:.9}, \
                 rel dev {:.3e}",
                (flow - expected).abs() / expected
            );
        }
    }
}
