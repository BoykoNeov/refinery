//! A check valve beside a dead end (M45.0, docs/DESIGN.md §50).
//!
//! A disc that faces a stretch of line closed at its far end — a pump started
//! against its shut discharge valve, with the check valve between them — carries
//! no flow whichever way the disc stands. Open, the stretch fills to the pump's
//! pressure and the drive across the disc is zero, so it shuts; shut, the
//! stretch's pressure is not determined by anything, is parked at a stale value
//! below the pump's, and the disc opens. Before M45 the active-set loop read that
//! alternation as chatter and refused the tick on both fidelities.
//!
//! Both orientations — the dead end after the disc and before it — on both
//! fidelities, because the two solvers reach the repeat from opposite ends.

use refinery_core::components::{Composition, Slate};
use refinery_core::error::SimError;
use refinery_core::graph::{EdgeId, LeakRole, Node, NodeId, NodeKind, Pipe, PlantGraph};
use refinery_core::traits::{FlowSolver, HydraulicSolution};
use refinery_core::units::*;
use refinery_solvers::{NewtonFlowSolver, SimpleFlowSolver};

fn node(name: &str, kind: NodeKind) -> Node {
    Node {
        name: name.into(),
        kind,
        heat_input: Watt(0.0),
    }
}

/// 5 m of 50 mm line rising `elevation_m`.
fn pipe(name: &str, elevation_m: f64) -> Pipe {
    Pipe {
        name: name.into(),
        length: Meter(5.0),
        diameter: Meter(0.05),
        friction_factor: 0.02,
        elevation_change: Meter(elevation_m),
        leak: LeakRole::None,
        ambient_ua: WattPerKelvin::ZERO,
        stream: refinery_core::stream::Stream::stagnant(1, T_AMBIENT, P_ATM),
    }
}

fn source(name: &str, pressure: f64) -> NodeKind {
    let _ = name;
    NodeKind::Source {
        pressure: Pascal(pressure),
        temperature: T_AMBIENT,
        composition: Composition::pure(1, 0),
    }
}

fn sink(pressure: f64) -> NodeKind {
    NodeKind::Sink {
        pressure: Pascal(pressure),
        temperature: T_AMBIENT,
        composition: Composition::pure(1, 0),
    }
}

fn disc() -> NodeKind {
    NodeKind::CheckValve {
        cv_max: 1e-3,
        full_open: Pascal(1.5e3),
        x_t: None,
    }
}

fn shut_valve() -> NodeKind {
    NodeKind::Valve {
        cv_max: 1e-3,
        opening: 0.0,
        x_t: None,
    }
}

/// The ids a test reads back.
struct Plant {
    graph: PlantGraph,
    /// The disc node, the dead end's first node past it, and the shut valve
    /// that closes the dead end.
    disc: NodeId,
    pocket: NodeId,
    valve: NodeId,
    edges: Vec<EdgeId>,
}

/// `header_pa` header → disc → POCKET → shut valve → 1 bar sink. The disc's
/// own outlet pipe rises `rise_m` and the pipe on to the valve `valve_rise_m`,
/// so the static heads `β` are nonzero when asked — the first on the way in,
/// the second inside the dead end.
fn pocket_after_disc(header_pa: f64, rise_m: f64, valve_rise_m: f64) -> Plant {
    let mut g = PlantGraph::new();
    let header = g.add_node(node("header", source("header", header_pa)));
    let d = g.add_node(node("disc", disc()));
    let pocket = g.add_node(node("pocket", NodeKind::Junction));
    let valve = g.add_node(node("valve", shut_valve()));
    let out = g.add_node(node("out", sink(1.0e5)));
    let edges = vec![
        g.add_pipe(header, d, pipe("to_disc", 0.0)),
        g.add_pipe(d, pocket, pipe("disc_out", rise_m)),
        g.add_pipe(pocket, valve, pipe("to_valve", valve_rise_m)),
        g.add_pipe(valve, out, pipe("valve_out", 0.0)),
    ];
    Plant {
        graph: g,
        disc: d,
        pocket,
        valve,
        edges,
    }
}

/// 5 bar header → shut valve → POCKET → disc → 0.5 bar sink: a stretch closed
/// upstream, whose stale (atmospheric) pressure would push the disc open into
/// a receiver below it. The disc's outlet rises `rise_m` (negative: falls).
fn pocket_before_disc(rise_m: f64) -> Plant {
    let mut g = PlantGraph::new();
    let header = g.add_node(node("header", source("header", 5.0e5)));
    let valve = g.add_node(node("valve", shut_valve()));
    let pocket = g.add_node(node("pocket", NodeKind::Junction));
    let d = g.add_node(node("disc", disc()));
    let out = g.add_node(node("out", sink(0.5e5)));
    let edges = vec![
        g.add_pipe(header, valve, pipe("to_valve", 0.0)),
        g.add_pipe(valve, pocket, pipe("valve_out", 0.0)),
        g.add_pipe(pocket, d, pipe("to_disc", 0.0)),
        g.add_pipe(d, out, pipe("disc_out", rise_m)),
    ];
    Plant {
        graph: g,
        disc: d,
        pocket,
        valve,
        edges,
    }
}

fn solvers() -> [(&'static str, Box<dyn FlowSolver>); 2] {
    [
        ("newton", Box::new(NewtonFlowSolver::default())),
        ("simple", Box::new(SimpleFlowSolver::default())),
    ]
}

/// Three solves in a row on one solver, so the second and third start from the
/// warm start the first committed — the engine's tick-to-tick situation.
fn solve_thrice(plant: &Plant, solver: &mut dyn FlowSolver, label: &str) -> HydraulicSolution {
    let slate = Slate::water_only();
    let mut last = None;
    for pass in 1..=3 {
        let solution = solver
            .solve(&plant.graph, &slate, &Default::default(), Seconds(1.0))
            .unwrap_or_else(|e| panic!("{label}: solve {pass} must succeed: {e}"));
        last = Some(solution);
    }
    last.expect("three solves ran")
}

/// Exactly zero on the disc's own outlet, the edge the tie is about; the
/// header-side pipe closes to the solver's own tolerance, like any node.
fn assert_no_flow(plant: &Plant, solution: &HydraulicSolution, label: &str) {
    for &e in &plant.edges {
        let flow = solution.edge_mass_flow[&e];
        let name = &plant.graph.pipe(e).name;
        if name == "disc_out" {
            assert_eq!(flow, 0.0, "{label}: the disc passes exactly nothing");
        } else {
            assert!(
                flow.abs() < 1e-9,
                "{label}: a dead end carries nothing, read {flow} kg/s on {name}"
            );
        }
    }
    for (nid, p) in &solution.node_pressure {
        assert!(
            p.value() > 0.0,
            "{label}: {} reads {} Pa",
            plant.graph.node(*nid).name,
            p.value()
        );
    }
}

/// The dead end after the disc: the tick solves, nothing flows, and the
/// stretch reads the header's pressure — what a pump dead-headed against its
/// shut discharge valve puts on its discharge line.
#[test]
fn a_dead_end_after_a_disc_solves_with_no_flow() {
    let plant = pocket_after_disc(5.0e5, 0.0, 0.0);
    for (name, mut solver) in solvers() {
        let label = format!("pocket after the disc, {name}");
        let solution = solve_thrice(&plant, solver.as_mut(), &label);
        assert_no_flow(&plant, &solution, &label);
        let pocket = solution.node_pressure[&plant.pocket].value();
        let at_disc = solution.node_pressure[&plant.disc].value();
        assert!(
            (pocket - at_disc).abs() < 1.0,
            "{label}: the stretch fills to the disc's inlet pressure: {pocket} Pa against \
             {at_disc} Pa"
        );
    }
}

/// The dead end before the disc, draining toward a receiver below it: the same
/// tie, reached from the other side of the disc.
#[test]
fn a_dead_end_before_a_disc_solves_with_no_flow() {
    let plant = pocket_before_disc(0.0);
    for (name, mut solver) in solvers() {
        let label = format!("pocket before the disc, {name}");
        let solution = solve_thrice(&plant, solver.as_mut(), &label);
        assert_no_flow(&plant, &solution, &label);
        // Drained to the receiver's pressure through a level disc: 0.5 bar.
        let pocket = solution.node_pressure[&plant.pocket].value();
        assert!(
            (pocket - 0.5e5).abs() < 1.0,
            "{label}: the stretch drains to the receiver's pressure, read {pocket} Pa"
        );
    }
}

/// With an uphill outlet the stretch stands below the disc's inlet by the
/// outlet pipe's static head, `ρ·g·Δz`: the disc is at zero forward drive, not
/// zero pressure difference (docs/DESIGN.md §33, "`β` is load-bearing"). And
/// the static head goes on inside the dead end: the valve, a metre higher
/// again, stands one more metre's head below the pocket.
#[test]
fn a_dead_end_up_a_rise_reads_the_static_head_below_the_disc() {
    let plant = pocket_after_disc(5.0e5, 2.0, 1.0);
    for (name, mut solver) in solvers() {
        let label = format!("pocket up a 2 m rise, {name}");
        let solution = solve_thrice(&plant, solver.as_mut(), &label);
        assert_no_flow(&plant, &solution, &label);
        let drop = solution.node_pressure[&plant.disc].value()
            - solution.node_pressure[&plant.pocket].value();
        // ρ·g·Δz for water at 2 m: 998 × 9.80665 × 2 ≈ 19.57 kPa.
        assert!(
            (19.0e3..20.2e3).contains(&drop),
            "{label}: the stretch stands one static head below the disc, read {drop} Pa"
        );
        let inside = solution.node_pressure[&plant.pocket].value()
            - solution.node_pressure[&plant.valve].value();
        // And 1 m more: ≈ 9.79 kPa.
        assert!(
            (9.5e3..10.1e3).contains(&inside),
            "{label}: the valve stands a metre's head below the pocket, read {inside} Pa"
        );
    }
}

/// **A dead end that would stand below vacuum is refused, not answered.** The
/// stretch sits at the top of a 15 m fall to a 0.5 bar receiver: drained to
/// zero drive it would read 0.5 bar less 15 m of water, about −97 kPa absolute —
/// a column that has broken, which no pressure in this model can name. The
/// tie is refused as it was before M45.0, on both fidelities.
#[test]
fn a_dead_end_below_vacuum_is_refused() {
    let plant = pocket_before_disc(-15.0);
    let slate = Slate::water_only();
    for (name, mut solver) in solvers() {
        let outcome = solver.solve(&plant.graph, &slate, &Default::default(), Seconds(1.0));
        assert!(
            matches!(
                outcome,
                Err(SimError::AnchoringUnsettled { cycled: true, .. })
            ),
            "{name}: refused as the cycle it was, got {:?}",
            outcome.map(|s| s.node_pressure)
        );
    }
}
