//! The `SeparationModel` contract at the ENGINE level (M7.1): what happens when
//! an implementation returns the wrong shape.
//!
//! `Separation::draws` is parallel to the column's own draw list BY CONTRACT, and
//! both readers — `Engine::tick`'s draw write and `energy::edge_composition_at` —
//! index it. A trait contract kept by an impl in another crate is exactly what
//! rule 5 says not to trust with a `[]`, so both check the length instead. Those
//! two guards are unreachable by every other test in this workspace, because the
//! only implementation that exists returns one entry per draw and always will.
//!
//! So the stub below breaks the contract ON PURPOSE. Without it the guards are
//! two branches that compile, are cited in a design note as satisfying rule 5,
//! and have never once run.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::engine::{Engine, EngineConfig};
use refinery_core::error::SimError;
use refinery_core::graph::{ColumnDraw, LeakRole, Node, NodeKind, Pipe, PlantGraph, TankState};
use refinery_core::traits::{ColumnPass, DrawSeparation, Separation, SeparationModel, ThermoModel};
use refinery_core::units::*;
use refinery_solvers::{ConstantThermo, NewtonFlowSolver, NoReactions};

const DT: Seconds = Seconds(0.1);

/// A model that returns ONE draw no matter how many the column has — the
/// contract violation, in its smallest form. Everything else about it is
/// deliberately valid: the split is finite, the composition is real, and it
/// converges. Only the LENGTH is wrong, which is the failure a `[]` turns into a
/// panic inside the tick loop rather than a `SimError` with a name on it.
struct ShortSplitter;
impl SeparationModel for ShortSplitter {
    fn name(&self) -> &'static str {
        "test-short"
    }
    fn separate(
        &self,
        pass: &ColumnPass<'_>,
        _thermo: &dyn ThermoModel,
    ) -> Result<Separation, SimError> {
        Ok(Separation {
            profile: None,
            draws: vec![DrawSeparation {
                split: 1.0,
                composition: pass.feed.clone(),
                temperature: pass.temperature,
            }],
            condenser_duty: None,
            reboiler_duty: None,
        })
    }
}

fn two_cut_slate() -> Slate {
    Slate::new(vec![
        PseudoComponent {
            name: "light".into(),
            tb: Kelvin(338.15),
            molar_mass: KgPerMol(0.1),
            density: Some(KgPerM3(700.0)),
            phase: Phase::Liquid,
            cp: JPerKgK(2000.0),
        },
        PseudoComponent {
            name: "heavy".into(),
            tb: Kelvin(613.15),
            molar_mass: KgPerMol(0.4),
            density: Some(KgPerM3(900.0)),
            phase: Phase::Liquid,
            cp: JPerKgK(2000.0),
        },
    ])
    .expect("a two-component slate is valid")
}

fn node(name: &str, kind: NodeKind) -> Node {
    Node {
        name: name.into(),
        kind,
        heat_input: Watt::ZERO,
    }
}

fn tank(name: &str) -> Node {
    node(
        name,
        NodeKind::Tank(TankState {
            area: SquareMeter(10.0),
            height: Meter(20.0),
            mass: Kg(1000.0),
            temperature: T_AMBIENT,
            composition: Composition::pure(2, 0),
            ambient_ua: WattPerKelvin::ZERO,
        }),
    )
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
        stream: refinery_core::stream::Stream::stagnant(2, T_AMBIENT, P_ATM),
    }
}

/// Source → column → two product tanks: the smallest plant with a column, built
/// by hand because the loader would only ever hand it `CutPointSplitter`.
///
/// The draws land on TANKS, which is not a convenience: the loader refuses a free
/// node on a draw line, so a product store is the only legal outlet and this
/// plant is the shape every column in the workspace has.
fn column_plant() -> PlantGraph {
    let mut graph = PlantGraph::new();
    let feed = graph.add_node(node(
        "feed",
        NodeKind::Source {
            pressure: Pascal(5.0e5),
            temperature: Kelvin(523.15),
            composition: Composition::from_weights(&[0.5, 0.5]).unwrap(),
        },
    ));
    let light = graph.add_node(tank("light_tank"));
    let heavy = graph.add_node(tank("heavy_tank"));
    let column = graph.add_node(node(
        "column",
        NodeKind::Column {
            pressure: Pascal(1.5e5),
            smearing: Kelvin(0.0),
            draws: vec![
                ColumnDraw::by_cut(light, Some(Kelvin(450.0))),
                ColumnDraw::by_cut(heavy, None),
            ],
            cascade: None,
        },
    ));
    graph.add_pipe(feed, column, pipe("feed_line"));
    graph.add_pipe(column, light, pipe("light_draw"));
    graph.add_pipe(column, heavy, pipe("heavy_draw"));
    graph
}

fn engine_with(separation: Box<dyn SeparationModel>) -> Engine {
    Engine::new(
        column_plant(),
        two_cut_slate(),
        EngineConfig { dt: DT },
        Box::new(NewtonFlowSolver::default()),
        Box::new(ConstantThermo),
        Box::new(NoReactions),
        separation,
    )
}

/// A model returning fewer draws than the column has is an `Err` naming BOTH
/// counts — not a panic, and not a silently dropped draw.
///
/// The draw write in `Engine::tick` is the first reader to reach the mismatch, so
/// that is the guard this exercises;
/// `a_short_draw_list_is_refused_by_the_composition_reader` covers the other one
/// directly, since only one of the two can fire per tick.
///
/// THE MUTATION THIS EXISTS FOR: either guard rewritten as `separation.draws[i]`.
/// The engine would then panic inside the tick loop — a crash where rule 5
/// requires a diagnostic, and one no other test in the workspace can reach.
#[test]
fn a_short_draw_list_is_refused_by_the_draw_write() {
    let mut engine = engine_with(Box::new(ShortSplitter));

    let err = match engine.tick() {
        Ok(()) => panic!("a 1-draw result for a 2-draw column must not tick cleanly"),
        Err(e) => e.to_string(),
    };
    assert!(
        err.contains("column") && err.contains('1') && err.contains('2'),
        "the error must name the column and both counts, got: {err}"
    );
}

/// The same violation, reached at the OTHER guard — `energy::edge_composition_at`,
/// which the tick never gets to once the draw write has refused.
///
/// Called directly with a separations map built by hand, which is the only way to
/// reach it: the two guards are mutually exclusive within a tick, so a test that
/// went through `Engine::tick` would silently be re-testing the first one.
#[test]
fn a_short_draw_list_is_refused_by_the_composition_reader() {
    use refinery_core::energy;
    use std::collections::BTreeMap;

    let graph = column_plant();
    let slate = two_cut_slate();
    let column = graph.find_node("column").expect("the column exists");
    let heavy_draw = graph
        .edge_ids()
        .find(|e| graph.pipe(*e).name == "heavy_draw")
        .expect("the heavy draw exists");
    let feed = Composition::from_weights(&[0.5, 0.5]).unwrap();

    // Resolved fields as a mid-sweep state would hold them, with a separation
    // one entry SHORT of the column's two draws.
    let mut composition = BTreeMap::new();
    composition.insert(column, feed.clone());
    let mut separations = BTreeMap::new();
    separations.insert(
        column,
        Separation {
            profile: None,
            draws: vec![DrawSeparation {
                split: 1.0,
                composition: feed,
                temperature: Kelvin(523.15),
            }],
            condenser_duty: None,
            reboiler_duty: None,
        },
    );

    // Positive flow: the column is this edge's upwind end, so the column arm runs.
    let err = match energy::edge_composition_at(&graph, &separations, &composition, heavy_draw, 1.0)
    {
        Ok(c) => panic!("draw 1 of a 1-entry separation must not resolve, got {c:?}"),
        Err(e) => e.to_string(),
    };
    assert!(
        err.contains("column") && err.contains('1') && err.contains('2'),
        "the error must name the column and both counts, got: {err}"
    );
    let _ = slate; // the guard fires before any slate-dependent work
}
