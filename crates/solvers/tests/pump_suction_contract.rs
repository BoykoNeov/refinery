//! M50 — the cavitating pump's head curve and its slope (docs/DESIGN.md §55).
//!
//! Two contracts the plant-level gates in
//! `crates/scenarios/tests/pump_cavitation_reference.rs` lean on without seeing
//! directly: `φ` is anchored where §55 says and is C¹ (the network keeps its
//! characteristics C¹, §3a fork 4), and `CompiledEdge::suction_share` IS the
//! derivative of the branch's flow with respect to the pump's own pressure —
//! the term both solvers add, and which a frozen branch cannot carry.

use refinery_core::components::{Composition, Slate};
use refinery_core::energy::NodeStates;
use refinery_core::graph::{LeakRole, Node, NodeId, NodeKind, Pipe, PlantGraph, PumpSuction};
use refinery_core::units::*;
use refinery_solvers::elements::{cavitation_head_fraction, cavitation_head_fraction_dsigma};
use refinery_solvers::network::compile_edge;
use std::collections::BTreeMap;

/// **The anchors** (§55 fork 1): 3% of the head lost at `NPSHa = NPSH3`
/// (ANSI/HI 9.6.1's definition), none left at the bubble pressure, and the
/// whole curve, bit for bit, with suction to spare.
#[test]
fn the_head_fraction_is_anchored_where_the_definition_puts_it() {
    assert!((cavitation_head_fraction(1.0) - 0.97).abs() < 1e-15);
    assert_eq!(cavitation_head_fraction(0.0), 0.0);
    assert_eq!(cavitation_head_fraction(-2.0), 0.0);
    assert_eq!(
        cavitation_head_fraction(3.3),
        1.0,
        "past σ ≈ 3.24 it is exactly 1"
    );
    let mut last = 0.0;
    for i in 1..=400 {
        let phi = cavitation_head_fraction(i as f64 * 0.01);
        assert!(phi >= last, "monotone in the margin");
        last = phi;
    }
}

/// **C¹, and the slope is the curve's** (§3a fork 4): the derivative is zero
/// at the bubble pressure from both sides, and matches a central difference
/// everywhere else.
#[test]
fn the_head_fraction_is_smooth_and_its_slope_is_its_own() {
    assert_eq!(cavitation_head_fraction_dsigma(0.0), 0.0);
    assert_eq!(cavitation_head_fraction_dsigma(-1.0), 0.0);
    assert!(cavitation_head_fraction_dsigma(1e-9) < 1e-7);
    let h = 1e-6;
    for sigma in [0.05, 0.2, 0.378, 0.7, 1.0, 1.5, 2.5] {
        let numeric =
            (cavitation_head_fraction(sigma + h) - cavitation_head_fraction(sigma - h)) / (2.0 * h);
        let exact = cavitation_head_fraction_dsigma(sigma);
        assert!(
            (numeric - exact).abs() < 1e-7 * exact.max(1.0),
            "σ = {sigma}: {numeric} against {exact}"
        );
    }
}

fn node(name: &str, kind: NodeKind) -> Node {
    Node {
        name: name.into(),
        kind,
        heat_input: Watt(0.0),
    }
}

fn pipe(name: &str) -> Pipe {
    Pipe {
        name: name.into(),
        length: Meter(20.0),
        diameter: Meter(0.1),
        friction_factor: 0.02,
        elevation_change: Meter(0.0),
        leak: LeakRole::None,
        ambient_ua: WattPerKelvin::ZERO,
        stream: refinery_core::stream::Stream::stagnant(1, T_AMBIENT, P_ATM),
    }
}

/// Water from a source through a pump with a suction limit to a sink; the
/// bubble pressure is set by hand, as the engine would between ticks.
fn plant(bubble: f64) -> (PlantGraph, NodeId, NodeId, refinery_core::graph::EdgeId) {
    let mut g = PlantGraph::new();
    let src = g.add_node(node(
        "src",
        NodeKind::Source {
            pressure: Pascal(2.0e5),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    let pump = g.add_node(node(
        "pump",
        NodeKind::Pump {
            h0: Meter(40.0),
            a: 800.0,
            on: true,
            suction: Some(PumpSuction {
                npsh_required: Meter(3.0),
                bubble_pressure: Some(Pascal(bubble)),
            }),
            gas_locked: false,
            gas_pocket: 0.0,
            gas_fill_time: None,
        },
    ));
    let snk = g.add_node(node(
        "snk",
        NodeKind::Sink {
            pressure: Pascal(4.0e5),
            temperature: T_AMBIENT,
            composition: Composition::pure(1, 0),
        },
    ));
    g.add_pipe(src, pump, pipe("in"));
    let out = g.add_pipe(pump, snk, pipe("out"));
    (g, pump, snk, out)
}

/// **`suction_share` is `∂ṁ/∂P_pump` beyond the frozen conductance** (§55
/// fork 3), across the knee: recompiling the branch at the pump's pressure
/// ± δ, with the far end held, moves the flow by exactly conductance + share.
/// Without the share a node step on the pump sees the frozen slope only — on
/// the M50 demo that slope is several times too small.
#[test]
fn the_suction_share_is_the_flows_own_derivative() {
    let slate = Slate::water_only();
    let states = NodeStates::default();
    let bubble = 1.5e5;
    let rho_g = 998.0 * G;
    let mut largest_ratio = 0.0f64;
    for sigma in [0.1, 0.3, 0.6, 1.0, 1.4] {
        let (g, pump, snk, out) = plant(bubble);
        let p = bubble + sigma * 3.0 * rho_g;
        let at = |pp: f64| {
            let pressures = BTreeMap::from([(g.endpoints(out).0, pp), (snk, 4.0e5)]);
            compile_edge(&g, out, &slate, &states, &pressures).expect("the edge compiles")
        };
        let flow_at = |pp: f64| {
            let c = at(pp);
            c.rho * c.branch.flow(pp - 4.0e5, 1.0)
        };
        let h = 1.0;
        let numeric = (flow_at(p + h) - flow_at(p - h)) / (2.0 * h);
        let c = at(p);
        let dp = p - 4.0e5;
        let conductance = c.conductance(dp, 1.0);
        let share = c.suction_share(dp, 1.0);
        assert_eq!(g.endpoints(out).0, pump);
        assert!(
            (numeric - (conductance + share)).abs() < 1e-5 * numeric.abs(),
            "σ = {sigma}: d ṁ/dP {numeric} against {conductance} + {share}"
        );
        largest_ratio = largest_ratio.max(share / conductance);
    }
    assert!(
        largest_ratio > 3.0,
        "the control: across the knee the share dwarfs the frozen slope ({largest_ratio})"
    );
}

/// **The key absent, or the pump stopped, compiles the pre-M50 branch** —
/// nothing reported, nothing added to the slope.
#[test]
fn no_suction_limit_no_share() {
    let slate = Slate::water_only();
    let states = NodeStates::default();
    let (mut g, pump, snk, out) = plant(1.5e5);
    let pressures = BTreeMap::from([(pump, 1.6e5), (snk, 4.0e5)]);
    let with = compile_edge(&g, out, &slate, &states, &pressures).unwrap();
    assert!(with.pump_suction.is_some());
    if let NodeKind::Pump { suction, on, .. } = &mut g.node_mut(pump).kind {
        *suction = None;
        *on = true;
    }
    let without = compile_edge(&g, out, &slate, &states, &pressures).unwrap();
    assert!(without.pump_suction.is_none());
    assert_eq!(without.suction_share(-2.4e5, 1.0), 0.0);
}
