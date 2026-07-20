//! I5 — cross-fidelity agreement on WELL-POSED networks. `NewtonFlowSolver`
//! (dense Newton) and `SimpleFlowSolver` (conductance-scaled Gauss–Seidel) solve
//! the *same* quasi-steady fixed point through the shared `network` compilation;
//! the only difference is how they drive the residual to zero. On any well-posed
//! network both must converge and their steady-state flows must agree.
//!
//! These curated cases are the load-bearing, NON-VACUOUS half of I5: every one
//! is guaranteed to converge on both solvers (unlike the random cross-check in
//! `invariants.rs`, which legally skips when Simple diverges on a stiff graph).
//! They span the element kinds and the topologies the acceptance criterion
//! cares about — including the `tank → pump → valve → sink` reference scenario.
//!
//! Agreement is asserted at 1e-2 relative on every non-negligible edge — far
//! inside the 5% I5 contract, because both solvers converge to a tight residual
//! on these conditioned networks (Simple's tol_rel=1e-6 ⇒ flows match Newton to
//! ~1e-5 in practice); 1% is a deliberately generous guard against tolerance
//! drift, not a claim the methods only agree to 1%.

use refinery_core::components::{Composition, Slate};
use refinery_core::graph::{EdgeId, Node, NodeKind, Pipe, PlantGraph, TankState};
use refinery_core::traits::{FlowSolver, HydraulicSolution};
use refinery_core::units::*;
use refinery_solvers::{NewtonFlowSolver, SimpleFlowSolver};

const RHO_WATER: f64 = 998.0;

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
fn source(p: f64) -> NodeKind {
    NodeKind::Source {
        pressure: Pascal(p),
        temperature: T_AMBIENT,
        composition: Composition::pure(1, 0),
    }
}
fn sink(p: f64) -> NodeKind {
    NodeKind::Sink {
        pressure: Pascal(p),
        temperature: T_AMBIENT,
    }
}
/// Pipe with an optional elevation change; friction fixed at 0.02 (Darcy).
fn pipe_ez(name: &str, l: f64, d: f64, elev: f64) -> Pipe {
    Pipe {
        name: name.into(),
        length: Meter(l),
        diameter: Meter(d),
        friction_factor: 0.02,
        elevation_change: Meter(elev),
        leak_area: SquareMeter(0.0),
        ambient_ua: WattPerKelvin::ZERO,
        stream: water_stream(),
    }
}
fn pipe(name: &str, l: f64, d: f64) -> Pipe {
    pipe_ez(name, l, d, 0.0)
}

/// Solve `g` with both fidelities; assert both converge and every
/// non-negligible edge flow agrees within `max_rel`. Edges below an absolute
/// floor (0.1% of Newton throughput, plus 1e-6 kg/s) are skipped — relative
/// error is meaningless on a near-zero flow.
fn assert_agree(g: &PlantGraph, edges: &[EdgeId], max_rel: f64) {
    let slate = Slate::water_only();
    let newton: HydraulicSolution = NewtonFlowSolver::default()
        .solve(g, &slate, Seconds(0.1))
        .expect("newton must converge on a well-posed network");
    let simple: HydraulicSolution = SimpleFlowSolver::default()
        .solve(g, &slate, Seconds(0.1))
        .expect("simple must converge on a well-posed network");
    assert!(newton.diagnostics.converged && simple.diagnostics.converged);

    let throughput = newton
        .edge_mass_flow
        .values()
        .fold(0.0f64, |m, &f| m.max(f.abs()));
    let floor = 1e-6 + 1e-3 * throughput;
    for &e in edges {
        let (fa, fb) = (newton.edge_mass_flow[&e], simple.edge_mass_flow[&e]);
        let scale = fa.abs().max(fb.abs());
        if scale < floor {
            continue;
        }
        let rel = (fa - fb).abs() / scale;
        assert!(
            rel <= max_rel,
            "fidelity flow mismatch at {e:?}: newton={fa}, simple={fb} (rel {rel} > {max_rel})"
        );
    }
}

/// No free nodes: exercises the shared `edge_flows` path identically on both.
#[test]
fn agree_source_pipe_sink() {
    let mut g = PlantGraph::new();
    let src = g.add_node(node("src", source(2.0e5)));
    let snk = g.add_node(node("snk", sink(1.0e5)));
    let e = g.add_pipe(src, snk, pipe("line", 10.0, 0.1));
    assert_agree(&g, &[e], 1e-2);
}

/// One free junction; the midpoint pressure is the cold-start mean, so Simple
/// nails it in one sweep — still must match Newton.
#[test]
fn agree_symmetric_junction() {
    let mut g = PlantGraph::new();
    let src = g.add_node(node("src", source(3.0e5)));
    let jn = g.add_node(node("jn", NodeKind::Junction));
    let snk = g.add_node(node("snk", sink(1.0e5)));
    let e1 = g.add_pipe(src, jn, pipe("a", 10.0, 0.1));
    let e2 = g.add_pipe(jn, snk, pipe("b", 10.0, 0.1));
    assert_agree(&g, &[e1, e2], 1e-2);
}

/// A 3-degree tee: the Gauss–Seidel node update must balance Σ over three
/// incident edges, not two — the branching case chains never build.
#[test]
fn agree_tee_junction() {
    let mut g = PlantGraph::new();
    let src = g.add_node(node("src", source(3.0e5)));
    let jn = g.add_node(node("jn", NodeKind::Junction));
    let snk1 = g.add_node(node("snk1", sink(1.0e5)));
    let snk2 = g.add_node(node("snk2", sink(1.5e5)));
    let e_a = g.add_pipe(src, jn, pipe("a", 10.0, 0.12));
    let e_b = g.add_pipe(jn, snk1, pipe("b", 10.0, 0.1));
    let e_c = g.add_pipe(jn, snk2, pipe("c", 14.0, 0.09));
    assert_agree(&g, &[e_a, e_b, e_c], 1e-2);
}

/// A running pump with a free suction node whose converged pressure is far from
/// the cold-start mean — the case that actually exercises Simple's sweeps
/// (~7 iterations empirically). Asymmetric pipes prevent a trivial midpoint.
#[test]
fn agree_pump_asymmetric() {
    let mut g = PlantGraph::new();
    let src = g.add_node(node("src", source(1.0e5)));
    let pmp = g.add_node(node(
        "pmp",
        NodeKind::Pump {
            h0: Meter(40.0),
            a: 5.0e3,
            on: true,
        },
    ));
    let jn = g.add_node(node("jn", NodeKind::Junction));
    let snk = g.add_node(node("snk", sink(1.0e5)));
    let e_in = g.add_pipe(src, pmp, pipe("in", 5.0, 0.15));
    let e_mid = g.add_pipe(pmp, jn, pipe("mid", 8.0, 0.08));
    let e_out = g.add_pipe(jn, snk, pipe("out", 12.0, 0.1));
    assert_agree(&g, &[e_in, e_mid, e_out], 1e-2);
}

/// A tank pins its hydrostatic bottom pressure; draining through a pipe to a
/// lower sink must give the same flow on both fidelities.
#[test]
fn agree_tank_drain() {
    let mut g = PlantGraph::new();
    let (area, level) = (10.0, 5.0);
    let tank = g.add_node(node(
        "tank",
        NodeKind::Tank(TankState {
            area: SquareMeter(area),
            height: Meter(10.0),
            mass: Kg(RHO_WATER * area * level),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
            ambient_ua: WattPerKelvin::ZERO,
        }),
    ));
    let snk = g.add_node(node("snk", sink(P_ATM.value())));
    let e = g.add_pipe(tank, snk, pipe("drain", 8.0, 0.1));
    assert_agree(&g, &[e], 1e-2);
}

/// The acceptance reference topology: tank → pump → valve → sink, with a lift.
/// Both free device nodes (pump suction, valve inlet) carry real sweeps, and a
/// partially-open valve sets a finite resistance both solvers must reconcile.
#[test]
fn agree_tank_pump_valve() {
    let mut g = PlantGraph::new();
    let (area, level) = (12.0, 6.0);
    let tank = g.add_node(node(
        "supply_tank",
        NodeKind::Tank(TankState {
            area: SquareMeter(area),
            height: Meter(10.0),
            mass: Kg(RHO_WATER * area * level),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
            ambient_ua: WattPerKelvin::ZERO,
        }),
    ));
    let pmp = g.add_node(node(
        "feed_pump",
        NodeKind::Pump {
            h0: Meter(50.0),
            a: 4.0e3,
            on: true,
        },
    ));
    let vlv = g.add_node(node(
        "control_valve",
        NodeKind::Valve {
            cv_max: 3.0e-3,
            opening: 0.6,
        },
    ));
    let snk = g.add_node(node("delivery", sink(1.2e5)));
    // Pump folds into its outlet edge; valve folds into its outlet edge.
    let e_suction = g.add_pipe(tank, pmp, pipe("suction", 6.0, 0.15));
    let e_disch = g.add_pipe(pmp, vlv, pipe_ez("discharge", 20.0, 0.12, 8.0)); // 8 m lift
    let e_deliver = g.add_pipe(vlv, snk, pipe("delivery_line", 15.0, 0.1));
    assert_agree(&g, &[e_suction, e_disch, e_deliver], 1e-2);
}
