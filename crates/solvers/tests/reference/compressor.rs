//! Reference case for the recycle compressor (M59.1, docs/DESIGN.md §64.3),
//! against a hand calculation.
//!
//! The element under test is `QuadraticBranch::compressor` as `compile_edge`
//! wires it: a polytropic head `H(Q) = H₀ − a·Q²` turned into a pressure rise at
//! the SUCTION density, corrected for compressibility by the polytropic relation
//! at the shutoff head.
//!
//! **The plant is Source → pipe → compressor → pipe → Sink**, and the gate is
//! read where the pipes say nothing: at ZERO flow. With nothing flowing, no pipe
//! drops any pressure, the compressor's node sits at the source's pressure, and
//! the sink pressure at which the flow is zero is the shutoff discharge alone —
//! the polytropic law, untangled from Darcy. Above it the flow runs back, below
//! it forward.
//!
//! What keeps it from being a tautology, as `orifice.rs` says of its own: no
//! number below is read back from the workspace. The hydrogen's properties, the
//! head and the efficiency are restated, the arithmetic is longhand in the
//! comment, and the expected discharge is a literal. The incompressible fan law
//! would put the shutoff 0.38 bar lower — 280 times the bracket this gate
//! reads at — so a dropped multiplier fails it.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::graph::{LeakRole, Node, NodeKind, Pipe, PlantGraph};
use refinery_core::traits::FlowSolver;
use refinery_core::units::*;
use refinery_solvers::{NewtonFlowSolver, SimpleFlowSolver};

// --- The hand calculation, restated ------------------------------------------
//
//   hydrogen    M = 2.016e-3 kg/mol, cp = 14 300 J/(kg·K)
//   R           8.314 462 618 153 24 J/(mol·K)
//   cv = cp − R/M                    = 10 175.762 590 J/(kg·K)
//   γ  = cp/cv                       =      1.405 300 082
//   η_p = 0.75
//   σ  = (γ − 1)/(γ·η_p)             =      0.384 544 281
//   suction     P₁ = 150 bar, T₁ = 323.15 K
//   ρ₁ = P₁·M/(R·T₁)                 =     11.254 946 670 kg/m³
//   H₀ = 120 kJ/kg
//   x  = σ·H₀·ρ₁/P₁                  =      0.034 624 203
//   P₂ = P₁·(1 + x)^(1/σ)            = 16 388 273.882 Pa      (163.883 bar)
//   (incompressible: P₁ + ρ₁·H₀      = 16 350 593.600 Pa, 0.377 bar lower)

const M_H2: f64 = 2.016e-3;
const CP_H2: f64 = 14_300.0;
const T_SUCTION_K: f64 = 323.15;
const P_SUCTION_PA: f64 = 150.0e5;
const HEAD_J_PER_KG: f64 = 120.0e3;
const CURVE: f64 = 100.0e3;
const ETA: f64 = 0.75;
/// The hand calculation's shutoff discharge [Pa], as a literal.
const SHUTOFF_PA: f64 = 16_388_273.882;

fn hydrogen() -> Slate {
    Slate::new(vec![PseudoComponent {
        name: "hydrogen".into(),
        phase: Phase::Gas,
        tb: Kelvin(20.28),
        molar_mass: KgPerMol(M_H2),
        density: None,
        cp: JPerKgK(CP_H2),
        cp_shape: None,
        dh_vap: None,
        cp_vapour: None,
        free_water: false,
    }])
    .expect("a one-gas slate")
}

fn pipe(name: &str) -> Pipe {
    Pipe {
        name: name.into(),
        length: Meter(20.0),
        diameter: Meter(0.2),
        friction_factor: 0.02,
        elevation_change: Meter(0.0),
        leak: LeakRole::None,
        ambient_ua: WattPerKelvin::ZERO,
        stream: refinery_core::stream::Stream::stagnant(
            1,
            Kelvin(T_SUCTION_K),
            Pascal(P_SUCTION_PA),
        ),
    }
}

/// Source at `suction` → compressor → Sink at `discharge` [Pa]; the mass flow
/// through the compressor's outlet edge [kg/s].
fn solve(suction: f64, discharge: f64, on: bool, solver: &mut dyn FlowSolver) -> f64 {
    let gas = Composition::pure(1, 0);
    let mut graph = PlantGraph::new();
    let src = graph.add_node(Node {
        name: "separator".into(),
        kind: NodeKind::Source {
            pressure: Pascal(suction),
            temperature: Kelvin(T_SUCTION_K),
            composition: gas.clone(),
        },
        heat_input: Watt::ZERO,
    });
    let machine = graph.add_node(Node {
        name: "compressor".into(),
        kind: NodeKind::Compressor {
            head: JPerKg(HEAD_J_PER_KG),
            a: CURVE,
            polytropic_efficiency: ETA,
            on,
        },
        heat_input: Watt::ZERO,
    });
    let sink = graph.add_node(Node {
        name: "header".into(),
        kind: NodeKind::Sink {
            pressure: Pascal(discharge),
            temperature: Kelvin(T_SUCTION_K),
            composition: gas.clone(),
        },
        heat_input: Watt::ZERO,
    });
    let mut suction_pipe = pipe("suction");
    suction_pipe.stream.composition = gas.clone();
    let mut discharge_pipe = pipe("discharge");
    discharge_pipe.stream.composition = gas;
    graph.add_pipe(src, machine, suction_pipe);
    let out = graph.add_pipe(machine, sink, discharge_pipe);
    let solution = solver
        .solve(&graph, &hydrogen(), &Default::default(), Seconds(0.1))
        .unwrap_or_else(|e| panic!("the compressor plant solves: {e}"));
    solution.edge_mass_flow[&out]
}

/// The flow is zero at the polytropic shutoff discharge: forward a hair below
/// it, backward a hair above it, on both fidelities. The bracket is 1e-6 of the
/// discharge, 16 Pa.
#[test]
fn the_shutoff_discharge_is_the_polytropic_hand_calculation() {
    let below = SHUTOFF_PA * (1.0 - 1e-6);
    let above = SHUTOFF_PA * (1.0 + 1e-6);
    for (name, solver) in [
        (
            "newton",
            &mut NewtonFlowSolver::default() as &mut dyn FlowSolver,
        ),
        ("simple", &mut SimpleFlowSolver::default()),
    ] {
        let forward = solve(P_SUCTION_PA, below, true, solver);
        let backward = solve(P_SUCTION_PA, above, true, solver);
        assert!(
            forward > 0.0 && backward < 0.0,
            "{name}: the flow must cross zero between {below} and {above} Pa: {forward}, \
             {backward} kg/s"
        );
    }
}

/// At a fixed suction temperature every term of the branch is proportional to
/// the suction density — the pipes' Darcy drop `∝ ρ·Q²`, the rise `m·ρ·H(Q)`
/// with `m` a function of the head and temperature alone — so halving both
/// pressures leaves the volume flow where it was and halves the mass flow. The
/// probe's assumption (§64.0): a compressor at fixed speed moves less hydrogen
/// as its loop depressures, and the cooling the hydrogen gives fades with it.
#[test]
fn the_mass_flow_follows_the_suction_pressure() {
    let mut solver = NewtonFlowSolver::default();
    let full = solve(P_SUCTION_PA, 155.0e5, true, &mut solver);
    let half = solve(P_SUCTION_PA / 2.0, 155.0e5 / 2.0, true, &mut solver);
    assert!(
        full > 1.0,
        "a working compressor moves the gas: {full} kg/s"
    );
    // The regularisation `ε/(2·Δp)` differs between the two (Δp halves), by
    // about 1e-6 of the flow: the tolerance is ten times that.
    approx::assert_relative_eq!(half / full, 0.5, max_relative = 1e-5);
}

/// A stopped compressor makes no head: it is the resistance of its curve
/// alone, so gas runs from the higher pressure to the lower whichever way that
/// is.
#[test]
fn a_stopped_compressor_makes_no_head() {
    let mut solver = NewtonFlowSolver::default();
    let up = solve(P_SUCTION_PA, P_SUCTION_PA + 1.0e5, false, &mut solver);
    let down = solve(P_SUCTION_PA, P_SUCTION_PA - 1.0e5, false, &mut solver);
    assert!(up < 0.0, "a bar above the suction runs back: {up} kg/s");
    assert!(down > 0.0, "a bar below runs forward: {down} kg/s");
}

/// The two fidelities share the element and agree on a working compressor to
/// the solve's own tolerance.
#[test]
fn both_fidelities_move_the_same_gas() {
    let newton = solve(
        P_SUCTION_PA,
        155.0e5,
        true,
        &mut NewtonFlowSolver::default(),
    );
    let simple = solve(
        P_SUCTION_PA,
        155.0e5,
        true,
        &mut SimpleFlowSolver::default(),
    );
    approx::assert_relative_eq!(newton, simple, max_relative = 1e-6);
}
