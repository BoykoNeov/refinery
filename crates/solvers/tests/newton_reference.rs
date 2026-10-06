//! Reference cases for `NewtonFlowSolver` against hand calculations and
//! closed-form symmetry arguments (CLAUDE.md → Testing: reference cases).
//!
//! Water only (single-component slate), SI throughout. Element physics is
//! Darcy–Weisbach pipe + ISA valve + quadratic pump head (see
//! `elements.rs`); these tests pin the *network* solve on top of it.

use refinery_core::components::{Composition, Slate};
use refinery_core::graph::{LeakRole, Node, NodeKind, Pipe, PlantGraph, TankState};
use refinery_core::traits::FlowSolver;
use refinery_core::units::*;
use refinery_solvers::elements::pipe_resistance;
use refinery_solvers::NewtonFlowSolver;

fn water_stream() -> refinery_core::stream::Stream {
    refinery_core::stream::Stream::stagnant(1, T_AMBIENT, P_ATM)
}

fn node(name: &str, kind: NodeKind) -> Node {
    Node {
        name: name.into(),
        kind,
        heat_input: Watt(0.0),
    }
}

fn pipe(name: &str, length_m: f64, diameter_m: f64) -> Pipe {
    Pipe {
        name: name.into(),
        length: Meter(length_m),
        diameter: Meter(diameter_m),
        friction_factor: 0.02,
        elevation_change: Meter(0.0),
        leak: LeakRole::None,
        ambient_ua: WattPerKelvin::ZERO,
        stream: water_stream(),
    }
}

const RHO_WATER: f64 = 998.0;

// ---------------------------------------------------------------------------

/// Source → pipe → Sink: no free nodes. The flow follows directly from the
/// fixed end pressures and the Darcy–Weisbach characteristic. Hand calc:
///   k = f·L·ρ/(2·D·A²), A = πD²/4
///   Q = √(ΔP)/√k,  ṁ = ρ·Q.
#[test]
fn source_pipe_sink_hand_calc() {
    let mut g = PlantGraph::new();
    let src = g.add_node(node(
        "src",
        NodeKind::Source {
            pressure: Pascal(2.0e5),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    let snk = g.add_node(node(
        "snk",
        NodeKind::Sink {
            pressure: Pascal(1.0e5),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    let e = g.add_pipe(src, snk, pipe("line", 10.0, 0.1));

    let mut solver = NewtonFlowSolver::default();
    let sol = solver
        .solve(&g, &Slate::water_only(), &Default::default(), Seconds(0.1))
        .expect("well-posed network must converge");

    // Hand calc.
    let k = pipe_resistance(0.02, 10.0, 0.1, RHO_WATER);
    let q = (1.0e5f64).sqrt() / k.sqrt();
    let expected = RHO_WATER * q; // ≈ 78.46 kg/s

    let flow = sol.edge_mass_flow[&e];
    approx::assert_relative_eq!(flow, expected, max_relative = 1e-3);
    assert!(flow > 0.0, "flow must be source → sink");
    assert!(sol.diagnostics.converged);
}

/// Source → pipeA → Junction → pipeB → Sink with identical pipes. By symmetry
/// the junction pressure is the arithmetic mean of the ends, and the two edge
/// flows are equal (mass conservation across the junction).
#[test]
fn symmetric_junction_midpoint_pressure() {
    let mut g = PlantGraph::new();
    let src = g.add_node(node(
        "src",
        NodeKind::Source {
            pressure: Pascal(3.0e5),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    let jn = g.add_node(node("jn", NodeKind::Junction));
    let snk = g.add_node(node(
        "snk",
        NodeKind::Sink {
            pressure: Pascal(1.0e5),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    let e1 = g.add_pipe(src, jn, pipe("a", 10.0, 0.1));
    let e2 = g.add_pipe(jn, snk, pipe("b", 10.0, 0.1));

    let mut solver = NewtonFlowSolver::default();
    let sol = solver
        .solve(&g, &Slate::water_only(), &Default::default(), Seconds(0.1))
        .expect("converges");

    let p_jn = sol.node_pressure[&jn].value();
    approx::assert_relative_eq!(p_jn, 2.0e5, max_relative = 1e-6);
    // Series ⇒ equal flows (to the solver's residual tolerance).
    approx::assert_relative_eq!(
        sol.edge_mass_flow[&e1],
        sol.edge_mass_flow[&e2],
        max_relative = 1e-6
    );
}

/// A pump between equal-pressure ends must still drive flow from suction to
/// discharge (source → sink), and mass is conserved through the pump node.
/// Turning the pump off must reduce the flow (no head to push against the end
/// pressures being equal ⇒ ~zero flow).
#[test]
fn pump_drives_flow_between_equal_pressures() {
    let build = |on: bool| {
        let mut g = PlantGraph::new();
        let src = g.add_node(node(
            "src",
            NodeKind::Source {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));
        let pmp = g.add_node(node(
            "pmp",
            NodeKind::Pump {
                h0: Meter(40.0),
                a: 5.0e3,
                on,
                suction: None,
            },
        ));
        let snk = g.add_node(node(
            "snk",
            NodeKind::Sink {
                pressure: Pascal(1.0e5),
                temperature: T_AMBIENT,
                composition: Composition::pure(1, 0),
            },
        ));
        let e_in = g.add_pipe(src, pmp, pipe("in", 5.0, 0.15));
        let e_out = g.add_pipe(pmp, snk, pipe("out", 5.0, 0.15));
        (g, e_in, e_out)
    };

    let slate = Slate::water_only();

    let (g_on, in_on, out_on) = build(true);
    let sol_on = NewtonFlowSolver::default()
        .solve(&g_on, &slate, &Default::default(), Seconds(0.1))
        .expect("converges");
    // Mass conservation through the pump node (one in, one out).
    approx::assert_relative_eq!(
        sol_on.edge_mass_flow[&in_on],
        sol_on.edge_mass_flow[&out_on],
        max_relative = 1e-6
    );
    let flow_on = sol_on.edge_mass_flow[&out_on];
    assert!(
        flow_on > 1.0,
        "pump on must push meaningful flow, got {flow_on}"
    );

    let (g_off, _in_off, out_off) = build(false);
    let sol_off = NewtonFlowSolver::default()
        .solve(&g_off, &slate, &Default::default(), Seconds(0.1))
        .expect("converges");
    let flow_off = sol_off.edge_mass_flow[&out_off];
    assert!(
        flow_off.abs() < 1e-3,
        "pump off with equal end pressures ⇒ ~no flow, got {flow_off}"
    );
}

/// A closed valve between two still-anchored ends (source upstream, sink
/// reachable downstream through a plain pipe) drives every flow on that path to
/// zero. Here NOTHING floats — both `vlv` and `jn` stay anchored — so this
/// exercises the α=∞ (zero-conductance) branch inside a normal Newton solve,
/// distinct from the floating-subnetwork path tested below.
#[test]
fn closed_valve_blocks_flow_both_ends_anchored() {
    let mut g = PlantGraph::new();
    let src = g.add_node(node(
        "src",
        NodeKind::Source {
            pressure: Pascal(4.0e5),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    // Valve fully closed (opening 0).
    let vlv = g.add_node(node(
        "vlv",
        NodeKind::Valve {
            cv_max: 1e-3,
            opening: 0.0,
            x_t: None,
        },
    ));
    let jn = g.add_node(node("jn", NodeKind::Junction));
    let snk = g.add_node(node(
        "snk",
        NodeKind::Sink {
            pressure: Pascal(1.0e5),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    let e_in = g.add_pipe(src, vlv, pipe("in", 5.0, 0.1));
    let e_out = g.add_pipe(vlv, jn, pipe("out", 5.0, 0.1)); // valve folds here
    let e_tail = g.add_pipe(jn, snk, pipe("tail", 5.0, 0.1));

    let mut solver = NewtonFlowSolver::default();
    let sol = solver
        .solve(&g, &Slate::water_only(), &Default::default(), Seconds(0.1))
        .expect("closed valve is a valid state, not an error");

    assert!(sol.diagnostics.converged);
    // e_out carries exactly 0 (α=∞); e_in/e_tail converge to ~0 through the
    // Newton solve (P_vlv → P_src, P_jn → P_snk).
    assert_eq!(sol.edge_mass_flow[&e_out], 0.0);
    for e in [e_in, e_tail] {
        assert!(
            sol.edge_mass_flow[&e].abs() < 1e-6,
            "no flow past a closed valve, got {}",
            sol.edge_mass_flow[&e]
        );
    }
    for p in sol.node_pressure.values() {
        assert!(p.value().is_finite());
    }
}

/// F2 (the real floating path): a closed valve severs a downstream subnetwork
/// that has NO other route to any fixed node, so those nodes are unanchored and
/// excluded from the Newton system. The non-obvious part is the PUMP: it would
/// "want" to drive flow with no pressure reference, but the active-edge gate
/// (both endpoints must be anchored) forces its edge to exactly zero. The
/// solver must not diverge — this is the operator-closed-the-valve state.
#[test]
fn floating_subnetwork_with_pump_reports_zero_flow() {
    let mut g = PlantGraph::new();
    let src = g.add_node(node(
        "src",
        NodeKind::Source {
            pressure: Pascal(2.0e5),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    let vlv = g.add_node(node(
        "vlv",
        NodeKind::Valve {
            cv_max: 1e-3,
            opening: 0.0, // closed → severs everything below it,
            x_t: None,
        },
    ));
    let pmp = g.add_node(node(
        "pmp",
        NodeKind::Pump {
            h0: Meter(50.0),
            a: 1e3,
            on: true, // running pump inside the dead subnetwork
            suction: None,
        },
    ));
    let dead = g.add_node(node("dead", NodeKind::Junction)); // dead-ended, no sink
    let e_in = g.add_pipe(src, vlv, pipe("in", 5.0, 0.1));
    let e_mid = g.add_pipe(vlv, pmp, pipe("mid", 5.0, 0.1)); // valve folds here (closed)
    let e_pump = g.add_pipe(pmp, dead, pipe("pump_out", 5.0, 0.1)); // pump folds here

    let sol = NewtonFlowSolver::default()
        .solve(&g, &Slate::water_only(), &Default::default(), Seconds(0.1))
        .expect("a floating subnetwork is pinned, not an error");

    assert!(sol.diagnostics.converged);
    // Everything downstream of the closed valve is floating ⇒ exactly zero,
    // INCLUDING the pump edge (the gate beats the pump's drive).
    assert_eq!(sol.edge_mass_flow[&e_mid], 0.0, "closed valve edge");
    assert_eq!(
        sol.edge_mass_flow[&e_pump], 0.0,
        "pump edge in dead subnetwork"
    );
    assert!(
        sol.edge_mass_flow[&e_in].abs() < 1e-6,
        "no flow into the closed valve, got {}",
        sol.edge_mass_flow[&e_in]
    );
    for p in sol.node_pressure.values() {
        assert!(p.value().is_finite(), "floating nodes are pinned, not NaN");
    }
}

/// A merge/split junction with THREE incident edges exercises the solver's
/// `Σ_in − Σ_out` mass balance over more than two terms (chains never do). Two
/// identical outlets to equal sinks ⇒ the inlet splits evenly and the totals
/// balance.
#[test]
fn tee_junction_conserves_mass() {
    let mut g = PlantGraph::new();
    let src = g.add_node(node(
        "src",
        NodeKind::Source {
            pressure: Pascal(3.0e5),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    let jn = g.add_node(node("jn", NodeKind::Junction));
    let snk1 = g.add_node(node(
        "snk1",
        NodeKind::Sink {
            pressure: Pascal(1.0e5),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    let snk2 = g.add_node(node(
        "snk2",
        NodeKind::Sink {
            pressure: Pascal(1.0e5),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    let e_a = g.add_pipe(src, jn, pipe("a", 10.0, 0.12)); // inlet
    let e_b = g.add_pipe(jn, snk1, pipe("b", 10.0, 0.1)); // outlet 1
    let e_c = g.add_pipe(jn, snk2, pipe("c", 10.0, 0.1)); // outlet 2

    let sol = NewtonFlowSolver::default()
        .solve(&g, &Slate::water_only(), &Default::default(), Seconds(0.1))
        .expect("converges");

    let (a, b, c) = (
        sol.edge_mass_flow[&e_a],
        sol.edge_mass_flow[&e_b],
        sol.edge_mass_flow[&e_c],
    );
    // Conservation at the junction: inflow = sum of outflows.
    approx::assert_relative_eq!(a, b + c, max_relative = 1e-6);
    // Symmetric outlets ⇒ even split.
    approx::assert_relative_eq!(b, c, max_relative = 1e-6);
    assert!(a > 0.0 && b > 0.0 && c > 0.0, "flow src → both sinks");
}

/// F6: a pump/valve node without exactly one inlet and one outlet edge is a
/// structural error the solver reports rather than trusting the loader.
#[test]
fn pump_wrong_degree_is_an_error() {
    let mut g = PlantGraph::new();
    let src = g.add_node(node(
        "src",
        NodeKind::Source {
            pressure: Pascal(2.0e5),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    // Pump with an inlet but no outlet edge (degree violation).
    let pmp = g.add_node(node(
        "pmp",
        NodeKind::Pump {
            h0: Meter(10.0),
            a: 1e3,
            on: true,
            suction: None,
        },
    ));
    g.add_pipe(src, pmp, pipe("in", 5.0, 0.1));

    let err = NewtonFlowSolver::default()
        .solve(&g, &Slate::water_only(), &Default::default(), Seconds(0.1))
        .expect_err("degree violation must be an error");
    let msg = err.to_string();
    assert!(
        msg.contains("inlet") && msg.contains("outlet"),
        "clear message: {msg}"
    );
}

/// A tank pins pressure at its hydrostatic bottom nozzle; draining to a lower
/// sink must produce flow out of the tank consistent with that head.
#[test]
fn tank_hydrostatic_head_drives_flow() {
    let mut g = PlantGraph::new();
    let area = 10.0;
    let level = 5.0;
    let mass = RHO_WATER * area * level; // ρ·A·h
    let tank = g.add_node(node(
        "tank",
        NodeKind::Tank(TankState {
            area: SquareMeter(area),
            height: Meter(10.0),
            mass: Kg(mass),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
            ambient_ua: WattPerKelvin::ZERO,
        }),
    ));
    let snk = g.add_node(node(
        "snk",
        NodeKind::Sink {
            pressure: Pascal(P_ATM.value()),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    let e = g.add_pipe(tank, snk, pipe("drain", 8.0, 0.1));

    let sol = NewtonFlowSolver::default()
        .solve(&g, &Slate::water_only(), &Default::default(), Seconds(0.1))
        .expect("converges");

    // Bottom pressure = P_ATM + ρ·g·h; sink at P_ATM ⇒ ΔP = ρ·g·h drives out.
    let dp = RHO_WATER * G * level;
    let k = pipe_resistance(0.02, 8.0, 0.1, RHO_WATER);
    let expected = RHO_WATER * (dp.sqrt() / k.sqrt());
    approx::assert_relative_eq!(sol.edge_mass_flow[&e], expected, max_relative = 2e-3);
    assert!(sol.edge_mass_flow[&e] > 0.0, "tank drains toward the sink");
}
