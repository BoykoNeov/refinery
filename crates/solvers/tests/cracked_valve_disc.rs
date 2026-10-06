//! A check valve ahead of a nearly shut valve (M45.1, docs/DESIGN.md §50).
//!
//! A pump — here a 5 bar header — feeds a check valve, 1 m of pipe, and a valve
//! cracked open, as a level loop leaves its fill valve when it starts opening
//! from shut. The game solver's node step on the valve's node sees only the
//! valve's tiny slope while the disc is shut, so the Newton step is hundreds of
//! times too long and lands where the disc is wide open; the line search's 1/256
//! still overshoots, every step is refused, and the node crawls. Before M45.1
//! the solve diverged at 5 000 sweeps with the valve 0.001% open, and took 208
//! with it 0.015% open; on the M30 demo every opening up to 0.3% diverged and 1%
//! took 2 681 sweeps. Now the node's own equation is solved on the bracket.
//!
//! Newton is the reference here: both fidelities solve one fixed point, and the
//! game solver must land on Newton's flow. Newton itself did not solve one of
//! these cases until M49 (ledger A21, docs/DESIGN.md §54).

use refinery_core::components::{Composition, Slate};
use refinery_core::error::SimError;
use refinery_core::graph::{LeakRole, Node, NodeKind, Pipe, PlantGraph};
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

/// `length_m` of 100 mm line, level.
fn pipe(name: &str, length_m: f64) -> Pipe {
    Pipe {
        name: name.into(),
        length: Meter(length_m),
        diameter: Meter(0.1),
        friction_factor: 0.02,
        elevation_change: Meter(0.0),
        leak: LeakRole::None,
        ambient_ua: WattPerKelvin::ZERO,
        stream: refinery_core::stream::Stream::stagnant(1, T_AMBIENT, P_ATM),
    }
}

/// The disc's coefficient at full lift, SI; the valve's is a quarter of it.
const DISC_CV: f64 = 200.0 / 3600.0 / 10.0;

/// 5 bar header → 30 m → disc (`band_pa` from shut to full lift) → 1 m → valve
/// at `opening` → 20 m → 1 bar sink: the M30 demo's discharge, cut to its parts.
fn cracked(opening: f64, band_pa: f64) -> PlantGraph {
    let mut g = PlantGraph::new();
    let header = g.add_node(node(
        "header",
        NodeKind::Source {
            pressure: Pascal(5.0e5),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    let disc = g.add_node(node(
        "disc",
        NodeKind::CheckValve {
            cv_max: DISC_CV,
            full_open: Pascal(band_pa),
            x_t: None,
        },
    ));
    let valve = g.add_node(node(
        "valve",
        NodeKind::Valve {
            cv_max: DISC_CV / 4.0,
            opening,
            x_t: None,
        },
    ));
    let out = g.add_node(node(
        "out",
        NodeKind::Sink {
            pressure: Pascal(1.0e5),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    g.add_pipe(header, disc, pipe("to_disc", 30.0));
    g.add_pipe(disc, valve, pipe("disc_out", 1.0));
    g.add_pipe(valve, out, pipe("valve_out", 20.0));
    g
}

fn solve(solver: &mut dyn FlowSolver, g: &PlantGraph) -> Result<HydraulicSolution, SimError> {
    solver.solve(g, &Slate::water_only(), &Default::default(), Seconds(1.0))
}

/// One flow through the chain, read off its last pipe.
fn chain_flow(solution: &HydraulicSolution) -> f64 {
    *solution
        .edge_mass_flow
        .values()
        .last()
        .expect("the chain has pipes")
}

/// **The game solver lands on Newton's answer, in a handful of sweeps, at every
/// opening from cracked to a tenth** — three disc bands: the M30 demo's 0.015
/// bar, ten times it and a tenth of it. The flow climbs four decades over the
/// openings; the sweep count stays under ten, and Newton's under its cap.
#[test]
fn a_cracked_valve_behind_a_disc_solves_on_both_fidelities() {
    for band in [1.5e2, 1.5e3, 1.5e4] {
        for opening in [1e-5, 1.5e-4, 1e-3, 1e-2, 0.1] {
            let g = cracked(opening, band);
            let label = format!("opening {opening}, band {band} Pa");
            let game = solve(&mut SimpleFlowSolver::default(), &g)
                .unwrap_or_else(|e| panic!("{label}: the game solver must solve: {e}"));
            assert!(
                game.diagnostics.iterations < 10,
                "{label}: {} sweeps",
                game.diagnostics.iterations
            );
            let reference = solve(&mut NewtonFlowSolver::default(), &g)
                .unwrap_or_else(|e| panic!("{label}: Newton must solve: {e}"));
            assert!(
                reference.diagnostics.iterations < 50,
                "{label}: {} Newton iterations",
                reference.diagnostics.iterations
            );
            let (a, b) = (chain_flow(&game), chain_flow(&reference));
            assert!(
                (a - b).abs() <= 1e-6 * b.abs().max(1e-3),
                "{label}: the game solver's {a} kg/s against Newton's {b}"
            );
        }
    }
}

/// **Newton answers the valve 1% open behind the demo's narrow band** (ledger
/// A21, closed by M49). Its second step used to carry the disc from wide open to
/// shut in one move — both nodes above the header, the disc's slope gone from the
/// Jacobian — and Armijo accepted it, because a shut disc hides the reverse flow
/// that makes the mirror step bad everywhere else. From there the cracked valve's
/// slope alone set the step, 1/256 of it still threw the disc wide open, and the
/// line search gave up at about 51 kg/s. The game solver's answer is 8.695 kg/s.
#[test]
fn newton_answers_a_valve_1_percent_open_behind_a_narrow_disc() {
    let g = cracked(1e-2, 1.5e3);
    let newton = solve(&mut NewtonFlowSolver::default(), &g)
        .unwrap_or_else(|e| panic!("Newton must answer A21's case: {e}"));
    let game = solve(&mut SimpleFlowSolver::default(), &g).expect("the game solver answers it");
    let (a, b) = (chain_flow(&game), chain_flow(&newton));
    assert!((b - 8.695).abs() < 1e-3, "Newton's {b} kg/s");
    assert!(
        (a - b).abs() <= 1e-6 * b,
        "the game solver's {a} kg/s against Newton's {b}"
    );
}
