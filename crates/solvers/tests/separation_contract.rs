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
use refinery_solvers::{ConstantThermo, NewtonFlowSolver, NoBoilOff, NoReactions};

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
            feed_vapour_fraction: None,
            reflux_ratio: None,
            reboiler_declined: None,
            cut_back: Vec::new(),
            water_flooded: None,
            loops: Vec::new(),
            stage_liquid: Vec::new(),
            trays: Vec::new(),
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
            cp_shape: None,
            dh_vap: None,
            cp_vapour: None,
            free_water: false,
        },
        PseudoComponent {
            name: "heavy".into(),
            tb: Kelvin(613.15),
            molar_mass: KgPerMol(0.4),
            density: Some(KgPerM3(900.0)),
            phase: Phase::Liquid,
            cp: JPerKgK(2000.0),
            cp_shape: None,
            dh_vap: None,
            cp_vapour: None,
            free_water: false,
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
        Box::new(NoBoilOff),
        Box::new(refinery_solvers::ConstantEnthalpy),
        Box::new(refinery_solvers::NoLineFlash),
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
            feed_vapour_fraction: None,
            reflux_ratio: None,
            reboiler_declined: None,
            cut_back: Vec::new(),
            water_flooded: None,
            loops: Vec::new(),
            stage_liquid: Vec::new(),
            trays: Vec::new(),
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

/// What one `separate` call was handed: the last tick's profile and this tick's
/// earlier pass, each read by the marker its first stage temperature carries.
type Handed = (Option<f64>, Option<f64>);

/// A model that splits its feed in half and records what each call was handed,
/// its profile marked with the call's own number so a later call can say whose
/// profile reached it.
struct Recorder(std::sync::Arc<std::sync::Mutex<Vec<Handed>>>);
impl SeparationModel for Recorder {
    fn name(&self) -> &'static str {
        "test-recorder"
    }
    fn separate(
        &self,
        pass: &ColumnPass<'_>,
        _thermo: &dyn ThermoModel,
    ) -> Result<Separation, SimError> {
        let marker = |profile: Option<&refinery_core::traits::CascadeProfile>| {
            profile.map(|p| p.temperatures[0].value())
        };
        let mut calls = self.0.lock().unwrap();
        calls.push((marker(pass.seed), marker(pass.earlier_pass)));
        let call = calls.len() as f64;
        let half = DrawSeparation {
            split: 0.5,
            composition: pass.feed.clone(),
            temperature: pass.temperature,
        };
        Ok(Separation {
            profile: Some(refinery_core::traits::CascadeProfile {
                temperatures: vec![Kelvin(call)],
                liquid: vec![vec![0.5, 0.5]],
                vapour: Vec::new(),
                free_water: Vec::new(),
                reflux_ratio: None,
                cut_back: Vec::new(),
            }),
            draws: vec![half.clone(), half],
            condenser_duty: None,
            reboiler_duty: None,
            feed_vapour_fraction: None,
            reflux_ratio: None,
            reboiler_declined: None,
            cut_back: Vec::new(),
            water_flooded: None,
            loops: Vec::new(),
            stage_liquid: Vec::new(),
            trays: Vec::new(),
        })
    }
}

/// **A tick's later pass is handed its earlier one** (M64.2, docs/DESIGN.md
/// §69.3): the first tick re-solves (M55.0), and every pass after its first
/// gets the pass just before it as `earlier_pass`, beside no `seed` — there is
/// no last tick. The next tick's first pass gets the last tick's answer as its
/// `seed` and no earlier pass.
///
/// THE MUTATION THIS EXISTS FOR: the engine handing `None` on its re-solves.
/// A held column then pays its ten-solve continuation on every pass of its
/// first tick, and nothing but the wall clock would say so.
#[test]
fn a_later_pass_is_handed_the_earlier_one() {
    let calls = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let mut engine = engine_with(Box::new(Recorder(calls.clone())));
    engine.tick().unwrap();
    let first: Vec<Handed> = calls.lock().unwrap().clone();
    assert!(
        first.len() >= 2,
        "the first tick re-solves at least once: {first:?}"
    );
    assert_eq!(first[0], (None, None), "a tick's first pass: {first:?}");
    for (pass, handed) in first.iter().enumerate().skip(1) {
        assert_eq!(*handed, (None, Some(pass as f64)), "pass {pass}: {first:?}");
    }

    engine.tick().unwrap();
    let second = calls.lock().unwrap()[first.len()];
    assert_eq!(
        second,
        (Some(first.len() as f64), None),
        "the next tick's first pass"
    );
}

/// What one `separate` call was handed as its running reboiler: the duty and
/// the earlier decline's reason.
type Running = Option<(f64, Option<String>)>;

/// A model that publishes its call's number as its reboiler duty, declines on
/// every even call, and records the running reboiler each call was handed.
struct RunningRecorder(std::sync::Arc<std::sync::Mutex<Vec<Running>>>);
impl SeparationModel for RunningRecorder {
    fn name(&self) -> &'static str {
        "test-running-recorder"
    }
    fn separate(
        &self,
        pass: &ColumnPass<'_>,
        _thermo: &dyn ThermoModel,
    ) -> Result<Separation, SimError> {
        let mut calls = self.0.lock().unwrap();
        calls.push(pass.running_reboiler.map(|running| {
            (
                running.duty.value(),
                running.declined.map(|declined| declined.reason.clone()),
            )
        }));
        let call = calls.len();
        let half = DrawSeparation {
            split: 0.5,
            composition: pass.feed.clone(),
            temperature: pass.temperature,
        };
        Ok(Separation {
            profile: None,
            draws: vec![half.clone(), half],
            condenser_duty: Some(Watt::ZERO),
            reboiler_duty: Some(Watt(call as f64)),
            feed_vapour_fraction: None,
            reflux_ratio: None,
            reboiler_declined: call.is_multiple_of(2).then(|| {
                refinery_core::traits::DeclinedHold {
                    asked: Watt::ZERO,
                    reason: format!("call {call}"),
                }
            }),
            cut_back: Vec::new(),
            water_flooded: None,
            loops: Vec::new(),
            stage_liquid: Vec::new(),
            trays: Vec::new(),
        })
    }
}

/// **A pass is handed the duty its column last ran at** (M66.2, docs/DESIGN.md
/// §71.2): a plant's first pass none; every later pass of a tick the earlier
/// pass's duty, with its decline where it declined; the next tick's first pass
/// the last tick's duty, its decline left behind.
///
/// THE MUTATION THIS EXISTS FOR: the engine reading only the last tick. A
/// column that declined then asks for its held duty again on every later pass
/// of the tick, and nothing but the wall clock would say so.
#[test]
fn a_pass_is_handed_the_duty_its_column_last_ran_at() {
    let calls = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let mut engine = engine_with(Box::new(RunningRecorder(calls.clone())));
    engine.tick().unwrap();
    let first: Vec<Running> = calls.lock().unwrap().clone();
    assert!(first.len() >= 2, "the first tick re-solves: {first:?}");
    assert_eq!(first[0], None, "a plant's first pass");
    for (pass, handed) in first.iter().enumerate().skip(1) {
        let earlier = pass; // the earlier call's number, one-based
        let declined = earlier.is_multiple_of(2).then(|| format!("call {earlier}"));
        assert_eq!(
            *handed,
            Some((earlier as f64, declined)),
            "pass {pass}: {first:?}"
        );
    }
    engine.tick().unwrap();
    let next = calls.lock().unwrap()[first.len()].clone();
    assert_eq!(
        next,
        Some((first.len() as f64, None)),
        "the next tick's first pass"
    );
}
