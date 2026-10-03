//! Reference case for the leak orifice (M6.1), against a hand calculation.
//!
//! The element under test is `QuadraticBranch::orifice` as `compile_edge` wires
//! it: Torricelli through a vena contracta, `Q = Cd·A·√(2·Δp/ρ)`.
//!
//! **The plant is Source → orifice → Atmosphere and nothing else, on purpose.**
//! A leak in a real plant hangs off a junction whose pressure the network
//! determines, so a gate built on one would be a *series* hand calc — the
//! orifice law entangled with a pipe's Darcy resistance and a pump curve, where
//! a compensating error in either can hide. With both endpoints pinned there are
//! no free nodes at all, the pressure drop across the orifice is a declared
//! number, and what the assertion sees is the orifice law alone.
//!
//! **What it pins and what it cannot.** Like every reference case in this repo
//! it can catch a wrong constant, a unit slip or a dropped factor — the ×2 under
//! the root, `Cd` squared instead of linear, `ρ` in the wrong place — and it
//! cannot catch an error in the model's *formulation*, because the expected
//! value is computed from the same law the code implements. What keeps it from
//! being a tautology is that no number below is read back from the workspace:
//! `Cd`, `A`, `ρ` and `Δp` are restated here, the arithmetic is carried out
//! longhand in the comment, and `EXPECTED_LEAK_KG_S` is a literal. Change the
//! discharge coefficient in `elements.rs` and this test fails, which is the
//! whole point of it.
//!
//! `Cd = 0.61` is a modelling constant with a ±2% bracket (see
//! `elements::ORIFICE_CD`), so this gate pins the arithmetic AROUND it, not the
//! coefficient's physical truth. A tolerance loose enough to accept 0.60–0.62
//! would accept a dropped `Cd` on a wide enough hole and is not what this is for.

use refinery_core::components::{Composition, Slate};
use refinery_core::graph::{LeakRole, Node, NodeKind, Pipe, PlantGraph};
use refinery_core::traits::FlowSolver;
use refinery_core::units::*;
use refinery_solvers::{NewtonFlowSolver, SimpleFlowSolver};

// --- The hand calculation, restated ------------------------------------------
//
//   Cd = 0.61          sharp-edged orifice, fully turbulent (elements.rs)
//   A  = 1.0e-4 m²     the commanded hole: 1 cm², a 11.3 mm bore
//   ρ  = 998 kg/m³     water at 20 °C (the water-only slate's declared density)
//   Δp = 2.0e5 Pa      3.01325 bara upstream − 1.01325 bara atmosphere
//
//   2·Δp/ρ = 400000/998          = 400.801 603 206 m²/s²
//   √(2·Δp/ρ)                    =  20.020 030 050 m/s   (ideal jet velocity)
//   Cd·√(2·Δp/ρ)                 =  12.212 218 331 m/s   (effective velocity)
//   Q  = Cd·A·√(2·Δp/ρ)          =   1.221 221 833e-3 m³/s
//   ṁ  = ρ·Q                     =   1.218 779 389 kg/s
//
// The engine reaches the same number by a different route — it inverts
// `α = ρ/(2·Cd²·A²)` through `Q = √(Δp/α)` — which is the algebra above
// rearranged, not an independent derivation. See the module note.

/// Commanded orifice area [m²].
const AREA_M2: f64 = 1.0e-4;
/// Upstream source pressure [Pa]: `P_ATM + 2 bar`, chosen so Δp is exactly 2e5.
const P_UPSTREAM_PA: f64 = 101_325.0 + 2.0e5;
/// The hand calculation's answer [kg/s], as a literal.
const EXPECTED_LEAK_KG_S: f64 = 1.218_779_389;

/// Source at `P_UPSTREAM_PA` → orifice of `area` → Atmosphere. Returns the
/// graph and the orifice's edge id.
fn orifice_plant(area: f64) -> (PlantGraph, refinery_core::graph::EdgeId) {
    let mut graph = PlantGraph::new();
    let src = graph.add_node(Node {
        name: "line".into(),
        kind: NodeKind::Source {
            pressure: Pascal(P_UPSTREAM_PA),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
        heat_input: Watt::ZERO,
    });
    let air = graph.add_node(Node {
        name: "outside".into(),
        kind: NodeKind::Atmosphere,
        heat_input: Watt::ZERO,
    });
    // Zero geometry, exactly as the loader builds one: an orifice has no length
    // to resist with and no bore that means anything. `compile_edge` returns
    // before reading either.
    let hole = graph.add_pipe(
        src,
        air,
        Pipe {
            name: "hole".into(),
            length: Meter(0.0),
            diameter: Meter(0.0),
            friction_factor: 0.02,
            elevation_change: Meter(0.0),
            leak: LeakRole::Orifice {
                area: SquareMeter(area),
            },
            ambient_ua: WattPerKelvin::ZERO,
            stream: refinery_core::stream::Stream::stagnant(1, T_AMBIENT, P_ATM),
        },
    );
    (graph, hole)
}

fn solve_leak(area: f64, solver: &mut dyn FlowSolver) -> f64 {
    let (graph, hole) = orifice_plant(area);
    let solution = solver
        .solve(
            &graph,
            &Slate::water_only(),
            &Default::default(),
            Seconds(0.1),
        )
        .expect("orifice plant solves");
    solution.edge_mass_flow[&hole]
}

/// The solvers' `eps_dp` [Pa]: the regularization that keeps `√(Δp)` C¹ through
/// zero. Restated here rather than imported — it is an input to the tolerance
/// derivation below, and reading it back would make that derivation follow the
/// code wherever it went.
const EPS_DP_PA: f64 = 1.0;

/// The gap this gate must tolerate, DERIVED rather than picked to fit.
///
/// The engine does not evaluate `√(Δp)`; it evaluates `Δp/√(Δp + ε)`, which for
/// `ε ≪ Δp` is `√(Δp)·(1 + ε/Δp)^(−1/2) ≈ √(Δp)·(1 − ε/(2·Δp))`. So the solved
/// leak must fall SHORT of the longhand Torricelli value by
/// `ε/(2·Δp) = 1/(4e5) = 2.5e-6` relative — a property of the regularization,
/// not of the orifice, and it shrinks as `1/Δp` on a bigger leak.
///
/// The tolerance is that number with 20% of headroom. It is deliberately not the
/// `1e-9` this file would use for an unregularized element: asserting against
/// the engine's own smoothed formula to buy back the last 2.5e-6 would gate the
/// arithmetic against itself, which is exactly what a reference case must not
/// do. `deviation_from_torricelli_is_the_regularization` below then pins that
/// the gap really is this and not a small modelling error wearing its clothes.
const MAX_RELATIVE: f64 = 3.0e-6;

#[test]
fn orifice_flow_matches_the_torricelli_hand_calculation() {
    approx::assert_relative_eq!(
        solve_leak(AREA_M2, &mut NewtonFlowSolver::default()),
        EXPECTED_LEAK_KG_S,
        max_relative = MAX_RELATIVE
    );
}

/// The gap above is the regularization and nothing else — checked, because a
/// tolerance wide enough to hide a 2.5e-6 error is wide enough to hide any other
/// error of that size, and the only defence is knowing which one is there.
///
/// Two independent claims: the solved flow is BELOW the ideal one (the smoothing
/// can only reduce `√`, never raise it), and the shortfall is within 20% of the
/// derived `ε/(2·Δp)`. A modelling slip would have to be under-predicting by
/// 2.5 parts per million exactly to survive both.
#[test]
fn deviation_from_torricelli_is_the_regularization() {
    let solved = solve_leak(AREA_M2, &mut NewtonFlowSolver::default());
    let shortfall = (EXPECTED_LEAK_KG_S - solved) / EXPECTED_LEAK_KG_S;
    let predicted = EPS_DP_PA / (2.0 * 2.0e5);
    assert!(
        shortfall > 0.0,
        "the regularized root can only undershoot; got {shortfall:e}"
    );
    approx::assert_relative_eq!(shortfall, predicted, max_relative = 0.2);
}

/// The same plant through the other fidelity. A network with no free nodes is
/// solved by neither Newton nor relaxation — both go straight to `edge_flows` —
/// so what this actually pins is that the two share one element definition and
/// one epilogue. It is cheap, and it is the arm that would catch a leak added to
/// only one of them.
#[test]
fn both_fidelities_size_the_orifice_identically() {
    let newton = solve_leak(AREA_M2, &mut NewtonFlowSolver::default());
    let simple = solve_leak(AREA_M2, &mut SimpleFlowSolver::default());
    assert_eq!(
        newton.to_bits(),
        simple.to_bits(),
        "the orifice is compiled once and shared, so the two fidelities must not \
         merely agree — they must produce the same bits: {newton} vs {simple}"
    );
}

/// `ṁ ∝ A` at fixed Δp, which is the structural claim underneath the constant:
/// area enters only through `α = ρ/(2·Cd²·A²)`, so doubling the hole doubles the
/// leak. This is what a `Cd`-independent reader of the model can check, and it
/// fails for a wrong POWER of `A` (a `√A` or an `A²` law) where the point gate
/// above, calibrated at one area, would not.
#[test]
fn leak_rate_is_linear_in_orifice_area() {
    let mut solver = NewtonFlowSolver::default();
    let single = solve_leak(AREA_M2, &mut solver);
    let double = solve_leak(2.0 * AREA_M2, &mut solver);
    let quadruple = solve_leak(4.0 * AREA_M2, &mut solver);
    approx::assert_relative_eq!(double / single, 2.0, max_relative = 1e-9);
    approx::assert_relative_eq!(quadruple / single, 4.0, max_relative = 1e-9);
}

/// A dormant leak conducts nothing — `α = +∞`, which `flow` reads as exactly
/// zero and `conducts` reads as closed. Exact zero, not a tolerance: this is the
/// state every reference plant in the repo sits in, so a leak path that trickled
/// would corrupt every scenario that declared one.
#[test]
fn a_zero_area_orifice_carries_exactly_no_flow() {
    assert_eq!(solve_leak(0.0, &mut NewtonFlowSolver::default()), 0.0);
    assert_eq!(solve_leak(0.0, &mut SimpleFlowSolver::default()), 0.0);
}

// --- The hole in GAS service (M37, docs/DESIGN.md §41) ------------------------
//
// Until M37 a gas orifice was refused here. It is now the isentropic nozzle law,
// `ṁ = Cd·A·√(2·ρ·p·ψ(x))`, and the gates below pin it against a hand
// calculation carried out in 30-digit arithmetic OUTSIDE the workspace
// (`W:\temp\claude\m37`, mpmath), with every input restated here:
//
//   methane: M = 0.016043 kg/mol, cp = 2220 J/(kg·K), R = 8.31446261815324
//   cv = cp − R/M = 1701.740 9…      γ = cp/cv = 1.304 547 943 859 438
//   r* = (2/(γ+1))^(γ/(γ−1)) = 0.544 907 186 346 855   x* = 1 − r* = 0.455 092 813 653 145
//   T = 293.15 K (the source), Cd = 0.61, A = 1e-4 m², downstream P_ATM
//
//   CHOKED, p = 10 bara: ρ = p·M/(R·T) = 6.582 054 893 351 58 kg/m³, x = 0.898 675 > x*
//     ψ(x*) = γ/(γ−1)·(r*^(2/γ) − r*^((γ+1)/γ)) = 0.223 167 460 739 304
//     ṁ = Cd·A·√(2·ρ·p·ψ) = 0.104 554 088 154 429 kg/s
//     and by the classical choked form, ṁ* = Cd·A·p·√(γ·M/(R·T))·(2/(γ+1))^((γ+1)/(2(γ−1))),
//     the same 0.104 554 088 154 429 to all thirty digits — two algebraic routes
//     to one number, so the gate is not the code's own expression read back.
//     Torricelli at the same state would say 0.209 810 kg/s: twice the truth.
//
//   SUBCRITICAL, p = 1.5 bara: ρ = 0.987 308 234 002 737, x = 0.3245 < x*
//     ψ(x) = 0.205 439 671 497 131      ṁ = 0.015 047 314 489 232 kg/s
//     (Torricelli: 0.018 911 kg/s, 26% high.)

/// Methane as a one-component gas slate, the fixture's declared numbers.
fn methane() -> Slate {
    Slate::new(vec![refinery_core::components::PseudoComponent {
        name: "methane".into(),
        tb: Kelvin(111.7),
        molar_mass: KgPerMol(0.016_043),
        density: None, // a gas has no stored density; it is P·M̄/(R·T)
        cp: JPerKgK(2220.0),
        cp_shape: None,
        phase: refinery_core::components::Phase::Gas,
    }])
    .expect("single-component gas slate")
}

/// Source at `upstream_pa` → hole of `area` → Atmosphere, in methane.
fn solve_gas_leak(area: f64, upstream_pa: f64, solver: &mut dyn FlowSolver) -> f64 {
    let (mut graph, hole) = orifice_plant(area);
    let line = graph.find_node("line").expect("the source");
    if let NodeKind::Source { pressure, .. } = &mut graph.node_mut(line).kind {
        *pressure = Pascal(upstream_pa);
    }
    let solution = solver
        .solve(&graph, &methane(), &Default::default(), Seconds(0.1))
        .expect("a gas hole solves");
    solution.edge_mass_flow[&hole]
}

/// Choked: the hand calculation above, to the regularization's derived shortfall
/// `ε/(2·Δp) = 1/(2·898 675) = 5.6e-7`, with headroom.
#[test]
fn a_choked_gas_hole_matches_the_isentropic_hand_calculation() {
    approx::assert_relative_eq!(
        solve_gas_leak(AREA_M2, 1.0e6, &mut NewtonFlowSolver::default()),
        0.104_554_088_154_429,
        max_relative = 1.0e-6
    );
}

/// Below the choke: shortfall `1/(2·48 675) = 1.03e-5`, with headroom. This is
/// the gate that fails if the compressibility correction is dropped (Torricelli
/// is 26% high here) or if the clamp is applied below the choke.
#[test]
fn a_subcritical_gas_hole_matches_the_isentropic_hand_calculation() {
    approx::assert_relative_eq!(
        solve_gas_leak(AREA_M2, 1.5e5, &mut NewtonFlowSolver::default()),
        0.015_047_314_489_232,
        max_relative = 1.3e-5
    );
}

/// Choked, the flow is set by the UPSTREAM state alone: at fixed temperature
/// `ṁ* ∝ p·√(ρ/p) ∝ p`, so doubling the line pressure doubles the leak — where
/// an unchoked square-root law would give `√(ρ·Δp)`, a ratio of about 2.1 here.
#[test]
fn a_choked_gas_hole_scales_with_the_upstream_pressure_alone() {
    let mut solver = NewtonFlowSolver::default();
    let ten = solve_gas_leak(AREA_M2, 1.0e6, &mut solver);
    let twenty = solve_gas_leak(AREA_M2, 2.0e6, &mut solver);
    approx::assert_relative_eq!(twenty / ten, 2.0, max_relative = 1.0e-6);
}

/// Shared element definition: both fidelities produce the same bits in gas too.
#[test]
fn both_fidelities_size_the_gas_hole_identically() {
    for upstream in [1.5e5, 1.0e6] {
        let newton = solve_gas_leak(AREA_M2, upstream, &mut NewtonFlowSolver::default());
        let simple = solve_gas_leak(AREA_M2, upstream, &mut SimpleFlowSolver::default());
        assert_eq!(newton.to_bits(), simple.to_bits(), "{newton} vs {simple}");
    }
}

/// A dormant gas hole is closed before `γ` is read: exactly no flow.
#[test]
fn a_zero_area_gas_hole_carries_exactly_no_flow() {
    assert_eq!(
        solve_gas_leak(0.0, 1.0e6, &mut NewtonFlowSolver::default()),
        0.0
    );
    assert_eq!(
        solve_gas_leak(0.0, 1.0e6, &mut SimpleFlowSolver::default()),
        0.0
    );
}

/// The back-feed refusal survives the gas law: a gas line below atmospheric with
/// an open hole would draw the Atmosphere's stand-in composition in, and is
/// refused exactly as a liquid one is (`network::finalize`). This holds only
/// because the gas branch keeps `beta = 0` and the sign of the drop.
#[test]
fn a_gas_hole_below_atmospheric_is_still_refused() {
    let err = {
        let (mut graph, _) = orifice_plant(AREA_M2);
        let line = graph.find_node("line").expect("the source");
        if let NodeKind::Source { pressure, .. } = &mut graph.node_mut(line).kind {
            *pressure = Pascal(0.5e5);
        }
        NewtonFlowSolver::default()
            .solve(&graph, &methane(), &Default::default(), Seconds(0.1))
            .expect_err("a gas hole drawing atmosphere in must be refused")
    };
    assert!(err.to_string().contains("back-feeds"), "{err}");
}
