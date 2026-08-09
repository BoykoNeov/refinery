//! Property tests — the backbone of solver trust (CLAUDE.md → Testing).
//!
//! Invariants, for ANY randomly generated valid network:
//!   I1. Mass conservation: Σ inflow = Σ outflow + Δ inventory per tick,
//!       to 1e-8 relative (Newton) / documented looser bound (Simple).
//!   I2. No negative inventories, pressures below absolute zero, or NaN/Inf
//!       anywhere in the snapshot, ever.
//!   I3. Solver terminates: Ok(converged) or Err(SolverDiverged). Never
//!       Ok with non-finite values.
//!   I4. Determinism: same scenario + same commands ⇒ byte-identical
//!       serialized snapshots across two fresh engine instances.
//!   I5. Fidelity agreement: Newton and Simple steady states match within
//!       5% on flows for well-posed networks. Breadth here (random chain/tree,
//!       compared only when BOTH solvers converge — a stiff network Simple
//!       cannot crack is skipped, exactly as I3 accepts SolverDiverged). The
//!       load-bearing, guaranteed-convergence half lives in
//!       `fidelity_agreement.rs`; `simple_agrees_on_a_healthy_fraction` below
//!       guards that this random half is not vacuously skipped.
//!
//! Two generators, complementary:
//!
//!   * CHAIN — Source → (Junction | Pump | Valve)* → Sink joined by pipes.
//!     Every node is 1-in/1-out, so it is a targeted reverse-flow-through-device
//!     path (randomized end pressures exercise both flow directions, hence the
//!     combined-branch's shifted-oddness reverse path). It does NOT branch.
//!
//!   * TREE — a hub-biased random tree: interior Junctions with 3+ incident
//!     edges (parents drawn from the first `HUB_SPAN` nodes so branching is
//!     structural, not a full-size-only fluke that shrinking erases — see
//!     `strategy_actually_branches`, which is the guard that this generator
//!     earns its keep). Pumps/valves are inserted by SUBDIVIDING an edge, which
//!     gives the device exactly one inlet + one outlet edge (F6 by
//!     construction). Leaves are fixed Source/Sink at random pressures; every
//!     open branch conducts, so the whole tree is anchored (no floating).
//!
//! WHAT THE TREE TEST ACTUALLY CATCHES (be honest — it is NOT a correctness
//! oracle). The per-node balance recomputes each residual R_i from the RETURNED
//! edge flows, and the solver only returns Ok when ‖R‖ is already below tol, so
//! on a fully anchored tree the balance is near-tautological on the pressures.
//! Its real value is:
//!   - `edge_flows` (post-processing) must agree with `assemble` (the Newton
//!     residual): they are SEPARATE functions, and a divergence between them —
//!     a sign flip, a missed edge, a wrong device fold — shows up here as a
//!     broken balance at a 3+ degree node that chains never build.
//!   - no NaN/Inf escapes a solve over branching topologies (I2/I3);
//!   - determinism holds with device nodes interleaved (I4).
//!
//! What it does NOT catch: a converged-but-WRONG answer. There is no closed
//! form for a random network, and a bad Jacobian typically surfaces as
//! `SolverDiverged` (which these tests accept as legal per I3). Correctness on
//! random networks is I5's job (Newton vs SimpleFlowSolver agreement, landing
//! with the Simple solver); this generator does not attempt it.
//!
//! Floating-subnetwork pinning (a closed valve severing part of the graph) is a
//! separate path, covered by hand-checkable cases in `newton_reference.rs`
//! (`floating_subnetwork_with_pump_reports_zero_flow`, `closed_valve_*`); the
//! generators here keep every branch conducting on purpose.

use proptest::prelude::*;
use proptest::strategy::{Strategy, ValueTree};
use proptest::test_runner::TestRunner;
use refinery_core::components::{Composition, PseudoComponent, Slate};
use refinery_core::error::SimError;
use refinery_core::graph::{Node, NodeKind, Pipe, PlantGraph};
use refinery_core::stream::Stream;
use refinery_core::traits::{FlowSolver, HydraulicSolution};
use refinery_core::units::*;
use refinery_solvers::{NewtonFlowSolver, SimpleFlowSolver};

// ---------------------------------------------------------------------------
// The plant-level FLUID (M5.4's open box). Phase is a property of the whole
// plant here, not of a node: `compile_edge` resolves a phase-mixing
// composition to `Err` through `Composition::phase`, so generating one would
// manufacture an error rather than exercise a solve. That is the same
// single-phase-connected-component rule the loader enforces, applied at the
// only place a directly-built `PlantGraph` can enforce it.
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct Fluid {
    slate: Slate,
    composition: Composition,
    /// `Some` in gas service, `None` in liquid — the SAME correspondence
    /// `compile_edge` refuses to see broken. Threading it from one place means
    /// no generated plant can be the pairing that refusal exists for, and the
    /// refusal itself keeps its own dedicated (hand-built) gate.
    x_t: Option<f64>,
}

impl Fluid {
    /// Water on the one-component slate: byte-for-byte what every generator in
    /// this file built before the gas arm, so every liquid case keeps its
    /// meaning and its history.
    fn liquid() -> Self {
        Self {
            slate: Slate::water_only(),
            composition: Composition::pure(1, 0),
            x_t: None,
        }
    }

    /// TWO gas cuts, not one, and the second is not decoration.
    /// `components.rs` records that every gas plant in the repo carries a pure
    /// single component, where `1/M̄ = Σ(wᵢ/Mᵢ)` and the naive `Σ(wᵢ·Mᵢ)`
    /// coincide — so the reciprocal rule reaching `γ` (and through `F_k`, the
    /// choke point) inside `compile_edge` has no wired coverage at all. A
    /// methane/propane blend separates the two by ~28%.
    fn gas(propane_weight: f64, x_t: f64) -> Self {
        let slate = Slate::new(vec![
            PseudoComponent {
                name: "methane".into(),
                tb: Kelvin(111.65),
                molar_mass: KgPerMol(0.016043),
                density: None,
                cp: JPerKgK(2220.0),
                phase: refinery_core::components::Phase::Gas,
            },
            PseudoComponent {
                name: "propane".into(),
                tb: Kelvin(231.05),
                molar_mass: KgPerMol(0.044096),
                density: None,
                cp: JPerKgK(1670.0),
                phase: refinery_core::components::Phase::Gas,
            },
        ])
        .expect("two-component gas slate");
        Self {
            slate,
            composition: Composition::from_weights(&[1.0 - propane_weight, propane_weight])
                .expect("both weights positive"),
            x_t: Some(x_t),
        }
    }

    fn is_gas(&self) -> bool {
        self.x_t.is_some()
    }
}

/// Liquid and gas at 1:1, so a 400-case run puts ~200 through each. Weights
/// stay clear of 0 and 1 so both cuts are genuinely present; `x_T` spans the
/// range IEC 60534-2-1's typical-value table covers for common trim.
fn fluid_strategy() -> impl Strategy<Value = Fluid> {
    prop_oneof![
        1 => Just(Fluid::liquid()),
        1 => (0.05..0.95f64, 0.1..0.9f64).prop_map(|(w, x_t)| Fluid::gas(w, x_t)),
    ]
}

// ---------------------------------------------------------------------------
// Shared node/pipe/edge builders.
// ---------------------------------------------------------------------------

fn source(p: f64, fluid: &Fluid) -> Node {
    Node {
        name: "src".into(),
        kind: NodeKind::Source {
            pressure: Pascal(p),
            temperature: T_AMBIENT,
            composition: fluid.composition.clone(),
        },
        heat_input: Watt(0.0),
    }
}

fn sink(p: f64, fluid: &Fluid) -> Node {
    Node {
        name: "snk".into(),
        kind: NodeKind::Sink {
            pressure: Pascal(p),
            // These are hydraulic tests: the solver never reads a temperature,
            // so ambient keeps them isothermal and out of the way. Thermal
            // transport gets its own generators in `energy_invariants.rs`.
            // A gas edge DOES read one — `ρ = P·M̄/(R·T)` — and takes it from
            // the upwind node, which is why ambient here is load-bearing for
            // the gas arm rather than merely tidy.
            temperature: T_AMBIENT,
            composition: fluid.composition.clone(),
        },
        heat_input: Watt(0.0),
    }
}

/// (length_m, diameter_m, friction_factor, elevation_change_m).
fn pipe(p: (f64, f64, f64, f64), name: &str, fluid: &Fluid) -> Pipe {
    let (length, diameter, friction_factor, elevation) = p;
    Pipe {
        name: name.into(),
        length: Meter(length),
        diameter: Meter(diameter),
        friction_factor,
        elevation_change: Meter(elevation),
        leak_area: SquareMeter(0.0),
        ambient_ua: WattPerKelvin::ZERO,
        stream: Stream {
            composition: fluid.composition.clone(),
            ..Stream::stagnant(fluid.slate.len(), T_AMBIENT, P_ATM)
        },
    }
}

fn pipe_strategy() -> impl Strategy<Value = (f64, f64, f64, f64)> {
    (1.0..50.0f64, 0.05..0.3f64, 0.01..0.05f64, -5.0..5.0f64)
}

fn all_finite(sol: &HydraulicSolution) -> bool {
    sol.node_pressure.values().all(|p| p.value().is_finite())
        && sol.edge_mass_flow.values().all(|f| f.is_finite())
}

// ---------------------------------------------------------------------------
// CHAIN generator: Source → mids* → Sink (no branching; reverse-flow path).
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
enum Mid {
    Junction,
    Pump { h0: f64, a: f64, on: bool },
    Valve { cv: f64, opening: f64 },
}

fn mid_strategy() -> impl Strategy<Value = Mid> {
    prop_oneof![
        Just(Mid::Junction),
        (10.0..60.0f64, 1e2..1e4f64, any::<bool>()).prop_map(|(h0, a, on)| Mid::Pump { h0, a, on }),
        // opening ≥ 0.05 keeps the chain conducting (closed-valve floating is
        // covered by a dedicated unit test, not here).
        (1e-4..5e-3f64, 0.05..1.0f64).prop_map(|(cv, opening)| Mid::Valve { cv, opening }),
    ]
}

/// Factor applied to a generated valve coefficient in GAS service only.
///
/// Reachability, not realism, and it was MEASURED rather than guessed. A gas
/// valve chokes only once it takes `x_choke` of its own inlet pressure — 8–84%
/// across this file's `x_T` and slate ranges — which cannot happen while the
/// generated pipes carry most of the drop. At the liquid coefficient range
/// `α_valve` lands at 48..1.2e5 against an `α_pipe` of 1e3..5e6, and the
/// measured choked fraction was **0 out of 185 converged gas chains**: the
/// whole arm was running on the degenerate `Y → 1` tail where the gas fold is
/// the liquid fold, testing nothing this milestone added.
///
/// A 50× smaller coefficient puts `α_valve` at ~3e5..3e9, so the valve
/// dominates and the plateau is reached. The LIQUID range is deliberately
/// untouched, so every case that existed before the gas arm keeps its meaning
/// and `simple_agrees_on_a_healthy_fraction` keeps its measured threshold.
const GAS_CV_SCALE: f64 = 0.02;

/// The valve coefficient this fluid should see. One function so the chain and
/// tree generators cannot drift apart on it.
fn valve_cv(cv: f64, fluid: &Fluid) -> f64 {
    if fluid.is_gas() {
        cv * GAS_CV_SCALE
    } else {
        cv
    }
}

/// A pump on a gas is a weak pressure source, NOT a compressor model: its
/// rise is `ρ·g·H`, so at 1.2 kg/m³ a 60 m curve is ~700 Pa. It stays in the
/// gas generator because it is a legal graph the solver must not choke on, and
/// this note is here so a later reader does not read its presence as a claim
/// that compression is supported — it is not (DESIGN §3a defers it).
fn mid_node(m: &Mid, i: usize, fluid: &Fluid) -> Node {
    let kind = match *m {
        Mid::Junction => NodeKind::Junction,
        Mid::Pump { h0, a, on } => NodeKind::Pump {
            h0: Meter(h0),
            a,
            on,
        },
        Mid::Valve { cv, opening } => NodeKind::Valve {
            cv_max: valve_cv(cv, fluid),
            opening,
            x_t: fluid.x_t,
        },
    };
    Node {
        name: format!("mid{i}"),
        kind,
        heat_input: Watt(0.0),
    }
}

/// Build a Source → mids* → Sink chain and its ordered edge ids.
fn build_chain(
    mids: &[Mid],
    pipes: &[(f64, f64, f64, f64)],
    p_src: f64,
    p_snk: f64,
    fluid: &Fluid,
) -> (PlantGraph, Vec<refinery_core::graph::EdgeId>) {
    let mut g = PlantGraph::new();
    let mut chain = vec![g.add_node(source(p_src, fluid))];
    for (i, m) in mids.iter().enumerate() {
        chain.push(g.add_node(mid_node(m, i, fluid)));
    }
    chain.push(g.add_node(sink(p_snk, fluid)));

    let mut edges = Vec::new();
    for i in 0..chain.len() - 1 {
        edges.push(g.add_pipe(
            chain[i],
            chain[i + 1],
            pipe(pipes[i], &format!("pipe{i}"), fluid),
        ));
    }
    (g, edges)
}

// ---------------------------------------------------------------------------
// TREE generator: hub-biased random tree with branching junctions.
// ---------------------------------------------------------------------------

/// Max non-root primary nodes ⇒ up to `MAX_K + 1` primary nodes before any
/// device subdivision.
const MAX_K: usize = 8;
const MAX_NODES: usize = MAX_K + 1;
/// Parents are drawn from the first `HUB_SPAN` node indices, so a handful of
/// nodes accumulate children and become genuine 3+ degree hubs — branching is
/// then a STRUCTURAL property of the generator, present even for small k where
/// shrinking lands, not a large-k-only accident. `strategy_actually_branches`
/// asserts this holds for the real strategy.
const HUB_SPAN: usize = 3;

/// A device optionally spliced into a tree edge. `None` leaves the edge a plain
/// pipe; the others subdivide it into pipe → device → pipe (F6: 1 in / 1 out).
#[derive(Debug, Clone)]
enum MidDevice {
    None,
    Pump { h0: f64, a: f64, on: bool },
    Valve { cv: f64, opening: f64 },
}

fn device_strategy() -> impl Strategy<Value = MidDevice> {
    prop_oneof![
        3 => Just(MidDevice::None),
        1 => (10.0..60.0f64, 1e2..1e4f64, any::<bool>())
            .prop_map(|(h0, a, on)| MidDevice::Pump { h0, a, on }),
        1 => (1e-4..5e-3f64, 0.05..1.0f64)
            .prop_map(|(cv, opening)| MidDevice::Valve { cv, opening }),
    ]
}

fn fixed_node(is_source: bool, p: f64, i: usize, fluid: &Fluid) -> Node {
    let kind = if is_source {
        NodeKind::Source {
            pressure: Pascal(p),
            temperature: T_AMBIENT,
            composition: fluid.composition.clone(),
        }
    } else {
        NodeKind::Sink {
            pressure: Pascal(p),
            temperature: T_AMBIENT,
            composition: fluid.composition.clone(),
        }
    };
    Node {
        name: format!("fix{i}"),
        kind,
        heat_input: Watt(0.0),
    }
}

fn is_free(k: &NodeKind) -> bool {
    matches!(
        k,
        NodeKind::Junction | NodeKind::Pump { .. } | NodeKind::Valve { .. }
    )
}

/// All four generated inputs. Every vec has a FIXED, generous length indexed by
/// position (advisor #3: no k-tied lengths / `prop_flat_map`); only `raw_parents`
/// carries the node count, and topology is derived from it by modulo + degree so
/// any shrunk sample is still a valid tree.
type TreeInputs = (
    Vec<usize>,                // raw_parents: length = node count − 1; carries size
    Vec<(bool, f64)>,          // fixed_specs[i]: (is_source, pressure) if node i is a leaf
    Vec<MidDevice>,            // devices[j-1]: optional device on edge into child j
    Vec<(f64, f64, f64, f64)>, // pipes: primary edge j-1, subdivided half MAX_K+j-1
    Fluid,                     // the whole tree's fluid — plant-level, never per-node
);

fn tree_inputs_strategy() -> impl Strategy<Value = TreeInputs> {
    (
        prop::collection::vec(0..1000usize, 1..=MAX_K),
        prop::collection::vec((any::<bool>(), 1.0e5..8.0e5f64), MAX_NODES..=MAX_NODES),
        prop::collection::vec(device_strategy(), MAX_K..=MAX_K),
        prop::collection::vec(pipe_strategy(), (2 * MAX_K)..=(2 * MAX_K)),
        fluid_strategy(),
    )
}

/// Build a hub-biased random tree. Child `j` (1..=k) attaches to parent
/// `raw_parents[j-1] % min(j, HUB_SPAN)`, so the first `HUB_SPAN` nodes become
/// hubs. Degree-1 nodes are leaves ⇒ fixed Source/Sink (pressure reference);
/// higher-degree nodes are Junctions. A selected edge is subdivided by a device.
fn build_tree(inputs: &TreeInputs) -> PlantGraph {
    let (raw_parents, fixed_specs, devices, pipes, fluid) = inputs;
    let k = raw_parents.len();
    let n_nodes = k + 1;

    // Parent + degree of every primary node.
    let mut parent = vec![0usize; n_nodes];
    let mut degree = vec![0usize; n_nodes];
    for j in 1..=k {
        let span = j.min(HUB_SPAN);
        let p = raw_parents[j - 1] % span;
        parent[j] = p;
        degree[j] += 1;
        degree[p] += 1;
    }

    // Leaves (degree 1) pin pressure as fixed Source/Sink; interiors are
    // Junctions. A tree with ≥2 nodes always has ≥2 leaves, so the graph is
    // always anchored.
    let mut g = PlantGraph::new();
    let mut ids = Vec::with_capacity(n_nodes);
    for (i, &deg) in degree.iter().enumerate().take(n_nodes) {
        let node = if deg == 1 {
            let (is_source, p) = fixed_specs[i];
            fixed_node(is_source, p, i, fluid)
        } else {
            Node {
                name: format!("jn{i}"),
                kind: NodeKind::Junction,
                heat_input: Watt(0.0),
            }
        };
        ids.push(g.add_node(node));
    }

    // Edges parent → child, oriented so a spliced device folds into its OUTLET
    // edge (device → child), matching the solver's fold-at-source convention.
    for j in 1..=k {
        let src = ids[parent[j]];
        let dst = ids[j];
        match &devices[j - 1] {
            MidDevice::None => {
                g.add_pipe(src, dst, pipe(pipes[j - 1], &format!("e{j}"), fluid));
            }
            dev => {
                let kind = match dev {
                    MidDevice::Pump { h0, a, on } => NodeKind::Pump {
                        h0: Meter(*h0),
                        a: *a,
                        on: *on,
                    },
                    MidDevice::Valve { cv, opening } => NodeKind::Valve {
                        cv_max: valve_cv(*cv, fluid),
                        opening: *opening,
                        x_t: fluid.x_t,
                    },
                    MidDevice::None => unreachable!("matched above"),
                };
                let mid = g.add_node(Node {
                    name: format!("dev{j}"),
                    kind,
                    heat_input: Watt(0.0),
                });
                g.add_pipe(src, mid, pipe(pipes[j - 1], &format!("e{j}a"), fluid));
                g.add_pipe(
                    mid,
                    dst,
                    pipe(pipes[MAX_K + j - 1], &format!("e{j}b"), fluid),
                );
            }
        }
    }
    g
}

/// Signed mass imbalance at a node from the RETURNED edge flows:
/// Σ(incoming ṁ) − Σ(outgoing ṁ). Should be ~0 at every zero-volume free node.
fn node_imbalance(
    g: &PlantGraph,
    sol: &HydraulicSolution,
    nid: refinery_core::graph::NodeId,
) -> f64 {
    let mut bal = 0.0;
    for (e, _other, incoming) in g.incident(nid) {
        let f = sol.edge_mass_flow[&e];
        bal += if incoming { f } else { -f };
    }
    bal
}

// ---------------------------------------------------------------------------
// Meta-test (advisor #1): the tree strategy must actually produce branching
// nodes, or it adds nothing over the chain test. Sample the REAL strategy with
// a deterministic runner and assert 3+ degree hubs appear at a healthy rate.
// ---------------------------------------------------------------------------

#[test]
fn strategy_actually_branches() {
    const SAMPLES: usize = 300;
    let mut runner = TestRunner::deterministic();
    let strat = tree_inputs_strategy();

    let mut max_seen = 0usize;
    let mut branched = 0usize;
    for _ in 0..SAMPLES {
        let inputs = strat
            .new_tree(&mut runner)
            .expect("strategy produces a value")
            .current();
        let g = build_tree(&inputs);
        let md = g.node_ids().map(|n| g.incident(n).len()).max().unwrap_or(0);
        max_seen = max_seen.max(md);
        if md >= 3 {
            branched += 1;
        }
    }

    assert!(
        max_seen >= 3,
        "tree generator never produced a 3+ degree node in {SAMPLES} samples \
         (it degenerated to chains — the whole test adds nothing over build_chain)"
    );
    // Branching must be COMMON, not a one-in-300 fluke, or shrinking will strip
    // it away in practice. Empirically ≈60% branch; 20% is a safe floor.
    assert!(
        branched * 5 >= SAMPLES,
        "branching too rare: only {branched}/{SAMPLES} samples had a 3+ degree node"
    );
}

// ---------------------------------------------------------------------------
// Non-vacuous guard for the I5 breadth tests: on the CHAIN generator (1-in/1-out
// nodes, well-conditioned far more often than stiff random trees), Simple must
// converge AND agree with Newton on a healthy fraction of the cases where Newton
// converges. Without this, `chain_fidelity_agreement` could pass while silently
// skipping every case (Simple always diverging), verifying nothing.
// ---------------------------------------------------------------------------

#[test]
fn simple_agrees_on_a_healthy_fraction() {
    const SAMPLES: usize = 300;
    let mut runner = TestRunner::deterministic();
    let strat = (
        prop::collection::vec(mid_strategy(), 0..5usize),
        prop::collection::vec(pipe_strategy(), 6usize..7),
        1.0e5..8.0e5f64,
        1.0e5..8.0e5f64,
    );
    let slate = Slate::water_only();

    let mut newton_ok = 0usize;
    let mut agreed = 0usize;
    for _ in 0..SAMPLES {
        let (mids, pipes, p_src, p_snk) = strat
            .new_tree(&mut runner)
            .expect("strategy produces a value")
            .current();
        let (g, _) = build_chain(&mids, &pipes, p_src, p_snk, &Fluid::liquid());
        let n = match NewtonFlowSolver::default().solve(
            &g,
            &slate,
            &Default::default(),
            Seconds(0.1),
        ) {
            Ok(n) if n.diagnostics.converged => n,
            _ => continue,
        };
        newton_ok += 1;
        let s = match SimpleFlowSolver::default().solve(
            &g,
            &slate,
            &Default::default(),
            Seconds(0.1),
        ) {
            Ok(s) if s.diagnostics.converged => s,
            _ => continue,
        };
        // Both converged: require flow agreement within 5% on non-tiny edges.
        let throughput = n
            .edge_mass_flow
            .values()
            .fold(0.0f64, |m, &f| m.max(f.abs()));
        let floor = 1e-6 + 1e-3 * throughput;
        let ok = n.edge_mass_flow.iter().all(|(eid, &fa)| {
            let fb = s.edge_mass_flow[eid];
            let scale = fa.abs().max(fb.abs());
            scale < floor || (fa - fb).abs() / scale <= 0.05
        });
        if ok {
            agreed += 1;
        }
    }

    assert!(newton_ok > 0, "Newton converged on no chain samples");
    // Empirically Simple converges+agrees on ≈100% of Newton-converged chains
    // (295/296 at time of writing); 50% is a safe floor that still fails loudly
    // if Simple silently stops converging.
    assert!(
        agreed * 2 >= newton_ok,
        "Simple agreed with Newton on only {agreed}/{newton_ok} converged chains \
         (I5 breadth would be vacuously skipping)"
    );
}

// ---------------------------------------------------------------------------
// Non-vacuity guard for the GAS arm, and the one measurement M5.4's fork 6
// could not make on a fixed plant.
//
// Three ways this arm could be silently worthless, each measured rather than
// argued:
//   1. gas plants never generated (a `prop_oneof` weight typo);
//   2. generated but never CHOKED — every valve sitting in the `Y → 1` tail
//      where the gas fold IS the liquid fold, so `fold_gas_valve`'s clamp,
//      plateau and inner solve are never touched;
//   3. generated, choked, and never SOLVED — Newton diverging on all of them,
//      which I3 accepts as legal and would therefore hide.
//
// (3) is also fork 6's open question turned into a number. Choking enters the
// solve as a frozen `α` recompiled each iteration with no `dα/dp` in the
// Jacobian, and the note deferred a true-derivative branch on the evidence of
// ONE plant (`gas_valve.toml`: 8 Newton iterations cold, 0 warm). A generated
// population is where a limit cycle would show up instead.
//
// MEASURED, and the numbers are the point of the test as much as the
// assertions are: 185/400 gas, Newton converging on 184/185 at a worst 43
// iterations against a `max_iter` of 50, Simple on 175/185, 101/184 carrying
// a valve at all and 33/101 of those reaching a choked one. Trees are counted
// separately (189/400 gas, 53/189 choked) because a spliced device there faces
// two pipes rather than one, so the chain's ratio does not carry over.
//
// **43 out of 50 reads alarming until it is given a control, which is why the
// liquid arm is measured alongside it: liquid's worst is 47.** The expensive
// cases are stiff random chains, and they are stiff whatever flows through
// them — the frozen `α` is not what costs the iterations. Fork 6's deferral
// therefore stands, now on a population rather than on one plant, and the
// honest caveat is about the CAP rather than about gas: a random chain of
// either fluid can come within a few iterations of `max_iter`.
// ---------------------------------------------------------------------------

/// Did this valve edge reach its choked plateau at the converged solution?
///
/// Measured on the flow, which needs no `α_eff` and therefore no cancellation:
/// `Q_gas(s)` is capped at `Y·√(x_choke·p₁/α_liquid)` with `Y = 2/3`, and it
/// attains that cap exactly when `s/p₁ ≥ x_choke`. So "is the flow at its
/// plateau" and "is the valve choked" are the same question, and the first one
/// is answerable from quantities the solution already carries.
///
/// This is a REACHABILITY measurement, not a correctness gate, which is why
/// calling the production element functions here is legitimate: the question
/// is whether the generator visits the branch, not whether the branch is right.
fn valve_edge_is_choked(
    graph: &PlantGraph,
    sol: &HydraulicSolution,
    fluid: &Fluid,
    eid: refinery_core::graph::EdgeId,
) -> bool {
    let (src, tgt) = graph.endpoints(eid);
    let (cv_max, opening, x_t) = match graph.node(src).kind {
        // Fold-at-source: a device folds into the edge LEAVING it, so only an
        // edge whose `src` is the valve carries one.
        NodeKind::Valve {
            cv_max,
            opening,
            x_t: Some(x_t),
        } => (cv_max, opening, x_t),
        _ => return false,
    };
    let pressures: std::collections::BTreeMap<_, _> = sol
        .node_pressure
        .iter()
        .map(|(n, p)| (*n, p.value()))
        .collect();
    let Ok(compiled) = refinery_solvers::network::compile_edge(
        graph,
        eid,
        &fluid.slate,
        &Default::default(),
        &pressures,
    ) else {
        return false;
    };
    let p_up = pressures[&src]
        .max(pressures[&tgt])
        .max(refinery_solvers::network::RHO_EVAL_P_FLOOR);
    let alpha_valve =
        (compiled.rho / refinery_solvers::network::RHO_WATER_REF) / (cv_max * opening).powi(2);
    let comp = &fluid.composition;
    let gamma = comp.mixture_cp(&fluid.slate).value() / comp.mixture_cv(&fluid.slate).value();
    let x_choke = refinery_solvers::elements::specific_heat_ratio_factor(gamma) * x_t;
    let plateau = (2.0 / 3.0) * (x_choke * p_up / alpha_valve).sqrt();
    let q = sol.edge_mass_flow[&eid].abs() / compiled.rho;
    // The 1e-4 is a MEASURED floor, not a fudge. A choked edge does not report
    // its plateau exactly: `edge_flows` inverts the branch through
    // `smooth_signed_sqrt(dp − β, eps_dp)`, whose O(eps/Δp) regularization
    // leaves the flow short by ~3e-6 relative at these drops. The two
    // populations are nonetheless cleanly separated — choked edges measure
    // q/plateau ≥ 0.999997 and the closest unchoked one 0.954 — so this
    // threshold has ~30x margin below the noise and ~460x above the gap.
    q >= plateau * (1.0 - 1e-4)
}

#[test]
fn the_gas_arm_generates_chokes_and_solves() {
    const SAMPLES: usize = 400;
    let mut runner = TestRunner::deterministic();
    let strat = (
        prop::collection::vec(mid_strategy(), 1..5usize),
        prop::collection::vec(pipe_strategy(), 6usize..7),
        1.0e5..8.0e5f64,
        1.0e5..8.0e5f64,
        fluid_strategy(),
    );

    let (mut gas, mut newton_ok, mut simple_ok, mut choked) = (0usize, 0usize, 0usize, 0usize);
    let mut with_valve = 0usize;
    let mut worst_iterations = 0u32;
    let mut liquid_worst = 0u32;
    for _ in 0..SAMPLES {
        let (mids, pipes, p_src, p_snk, fluid) = strat
            .new_tree(&mut runner)
            .expect("strategy produces a value")
            .current();
        if !fluid.is_gas() {
            // Liquid runs as the CONTROL. Without it, a worst-case gas
            // iteration count is a number with nothing to compare against:
            // a stiff random chain is expensive to solve whatever is flowing
            // through it, and only the difference isolates the frozen `α`.
            let (g, _) = build_chain(&mids, &pipes, p_src, p_snk, &fluid);
            if let Ok(n) = NewtonFlowSolver::default().solve(
                &g,
                &fluid.slate,
                &Default::default(),
                Seconds(0.1),
            ) {
                if n.diagnostics.converged {
                    liquid_worst = liquid_worst.max(n.diagnostics.iterations);
                }
            }
            continue;
        }
        gas += 1;
        let (g, edges) = build_chain(&mids, &pipes, p_src, p_snk, &fluid);
        let n = match NewtonFlowSolver::default().solve(
            &g,
            &fluid.slate,
            &Default::default(),
            Seconds(0.1),
        ) {
            Ok(n) if n.diagnostics.converged => n,
            _ => continue,
        };
        newton_ok += 1;
        worst_iterations = worst_iterations.max(n.diagnostics.iterations);
        // A chain whose mids are all junctions and pumps has no valve to
        // choke, so it is not evidence either way — the choked fraction is
        // taken over the chains that actually carry one.
        if mids.iter().any(|m| matches!(m, Mid::Valve { .. })) {
            with_valve += 1;
            if edges
                .iter()
                .any(|&e| valve_edge_is_choked(&g, &n, &fluid, e))
            {
                choked += 1;
            }
        }
        if let Ok(s) =
            SimpleFlowSolver::default().solve(&g, &fluid.slate, &Default::default(), Seconds(0.1))
        {
            if s.diagnostics.converged {
                simple_ok += 1;
            }
        }
    }

    println!(
        "gas chains: {gas}/{SAMPLES}; newton converged {newton_ok}/{gas} \
         (worst {worst_iterations} iterations); simple converged {simple_ok}/{gas}; \
         carrying a valve {with_valve}/{newton_ok}; at least one valve choked \
         {choked}/{with_valve}; worst liquid iterations {liquid_worst} (control)"
    );

    // The TREE generator needs its own count, not an inference from the chain's.
    // It splices a device by SUBDIVIDING an edge, so a valve there faces two
    // pipes instead of one and the resistance ratio `GAS_CV_SCALE` was sized
    // against does not carry over. If trees never choke, the one thing the tree
    // generator exists for — `edge_flows` cross-checked against `assemble` at a
    // 3+ degree hub — never sees a choked branch, which is the 0/185 failure one
    // level down.
    let tree_strat = tree_inputs_strategy();
    let (mut gas_trees, mut choked_trees) = (0usize, 0usize);
    for _ in 0..SAMPLES {
        let inputs = tree_strat
            .new_tree(&mut runner)
            .expect("strategy produces a value")
            .current();
        if !inputs.4.is_gas() {
            continue;
        }
        gas_trees += 1;
        let g = build_tree(&inputs);
        let Ok(n) = NewtonFlowSolver::default().solve(
            &g,
            &inputs.4.slate,
            &Default::default(),
            Seconds(0.1),
        ) else {
            continue;
        };
        if n.diagnostics.converged
            && g.edge_ids()
                .any(|e| valve_edge_is_choked(&g, &n, &inputs.4, e))
        {
            choked_trees += 1;
        }
    }
    println!("gas trees: {gas_trees}/{SAMPLES}; with a choked valve {choked_trees}/{gas_trees}");

    // (1) Gas must be a substantial share, not a rounding error. The strategy
    // is 1:1, so ~50%; 25% is a floor that fails loudly on a weight typo.
    assert!(
        gas * 4 >= SAMPLES,
        "only {gas}/{SAMPLES} generated plants were gas — the gas arm is barely sampled"
    );
    // (3) Newton must crack most of them. A frozen-`α` limit cycle would show
    // as a collapse here, and I3 would silently accept it as SolverDiverged.
    assert!(
        newton_ok * 2 >= gas,
        "Newton converged on only {newton_ok}/{gas} gas chains — frozen-alpha \
         convergence has degraded and DESIGN §3a fork 6's deferral needs revisiting"
    );
    // (2) The one that matters most: choking must actually happen. Without it
    // every gas assertion in this file runs on the degenerate `Y → 1` branch,
    // where the gas fold and the liquid fold are the same function.
    assert!(
        choked * 5 >= with_valve,
        "only {choked}/{with_valve} converged gas chains carrying a valve reached a \
         choked one — the generator has drifted off the branch the gas arm exists \
         to reach"
    );
    // The same floor for trees, measured separately at 53/189 (28%). A tree
    // that never chokes would leave the branching cross-check — the reason the
    // tree generator exists at all — running only on unchoked branches.
    assert!(
        choked_trees * 10 >= gas_trees,
        "only {choked_trees}/{gas_trees} gas trees reached a choked valve — the \
         branching cross-check never sees a choked branch"
    );
}

// ---------------------------------------------------------------------------
// Property tests.
// ---------------------------------------------------------------------------

proptest! {
    #![proptest_config(ProptestConfig::with_cases(400))]

    /// CHAIN — I2 + I3 + I1: the solver terminates as Ok(converged, all-finite)
    /// or Err(SolverDiverged); and when it converges, a series chain conserves
    /// mass — every edge carries the same flow (no accumulation at the
    /// zero-volume interior nodes), and no pressure is non-finite.
    #[test]
    fn chain_conserves_or_diverges(
        mids in prop::collection::vec(mid_strategy(), 0..5usize),
        raw_pipes in prop::collection::vec(pipe_strategy(), 6usize..7),
        p_src in 1.0e5..8.0e5f64,
        p_snk in 1.0e5..8.0e5f64,
        fluid in fluid_strategy(),
    ) {
        let (g, edges) = build_chain(&mids, &raw_pipes, p_src, p_snk, &fluid);
        let mut solver = NewtonFlowSolver::default();

        match solver.solve(&g, &fluid.slate, &Default::default(), Seconds(0.1)) {
            Ok(sol) => {
                prop_assert!(sol.diagnostics.converged, "Ok must mean converged");
                prop_assert!(all_finite(&sol), "no NaN/Inf may escape a solve");
                // NOTE: absolute pressure is NOT asserted non-negative here. An
                // over-driven pump (e.g. equal end pressures + low resistance)
                // has a genuine solution with sub-zero absolute suction — that
                // is real cavitation, which M1 does not model (no vapor-pressure
                // floor). The pure hydraulic solve returning it is correct; a
                // pressure floor belongs to a later cavitation milestone.
                // I1: series chain ⇒ all edge flows equal to convergence tol.
                let flows: Vec<f64> = edges.iter().map(|e| sol.edge_mass_flow[e]).collect();
                let throughput = flows.iter().fold(0.0f64, |m, f| m.max(f.abs()));
                let (lo, hi) = flows.iter().fold((f64::MAX, f64::MIN), |(lo, hi), &f| {
                    (lo.min(f), hi.max(f))
                });
                if !flows.is_empty() {
                    prop_assert!(
                        (hi - lo) <= 1e-5 + 1e-6 * throughput,
                        "mass imbalance across chain: spread {} (throughput {throughput})",
                        hi - lo
                    );
                }
            }
            Err(SimError::SolverDiverged { .. }) => { /* acceptable per I3 */ }
            Err(other) => prop_assert!(false, "unexpected error: {other}"),
        }
    }

    /// CHAIN — I4: two fresh solvers on the same network produce bit-identical
    /// results.
    #[test]
    fn chain_solve_is_deterministic(
        mids in prop::collection::vec(mid_strategy(), 0..5usize),
        raw_pipes in prop::collection::vec(pipe_strategy(), 6usize..7),
        p_src in 1.0e5..8.0e5f64,
        p_snk in 1.0e5..8.0e5f64,
        fluid in fluid_strategy(),
    ) {
        let (g, _) = build_chain(&mids, &raw_pipes, p_src, p_snk, &fluid);
        let a = NewtonFlowSolver::default().solve(&g, &fluid.slate, &Default::default(), Seconds(0.1));
        let b = NewtonFlowSolver::default().solve(&g, &fluid.slate, &Default::default(), Seconds(0.1));
        assert_same_solution(a, b)?;
    }

    /// TREE — I2 + I3 + I1 over BRANCHING topologies: the solver terminates
    /// legally, and on convergence every zero-volume free node (Junction / Pump /
    /// Valve, including 3+ degree hubs the chain never builds) balances mass:
    /// Σ incoming ṁ = Σ outgoing ṁ. This exercises `assemble`'s Σ-over-3+-edges
    /// residual AND cross-checks that `edge_flows` (a separate function) agrees
    /// with it. See the module doc on what this does and does not verify.
    #[test]
    fn tree_conserves_or_diverges(inputs in tree_inputs_strategy()) {
        let g = build_tree(&inputs);
        let mut solver = NewtonFlowSolver::default();

        match solver.solve(&g, &inputs.4.slate, &Default::default(), Seconds(0.1)) {
            Ok(sol) => {
                prop_assert!(sol.diagnostics.converged, "Ok must mean converged");
                prop_assert!(all_finite(&sol), "no NaN/Inf may escape a solve");
                let throughput = sol
                    .edge_mass_flow
                    .values()
                    .fold(0.0f64, |m, &f| m.max(f.abs()));
                for nid in g.node_ids() {
                    if is_free(&g.node(nid).kind) {
                        let bal = node_imbalance(&g, &sol, nid);
                        prop_assert!(
                            bal.abs() <= 1e-5 + 1e-6 * throughput,
                            "mass imbalance {bal} at {nid:?} (throughput {throughput})"
                        );
                    }
                }
            }
            Err(SimError::SolverDiverged { .. }) => { /* acceptable per I3 */ }
            Err(other) => prop_assert!(false, "unexpected error: {other}"),
        }
    }

    /// TREE — I4: determinism holds with device nodes interleaved into a
    /// branching graph (BTreeMap ordering + no wall-clock/RNG/HashMap).
    #[test]
    fn tree_solve_is_deterministic(inputs in tree_inputs_strategy()) {
        let g = build_tree(&inputs);
        let slate = &inputs.4.slate;
        let a = NewtonFlowSolver::default().solve(&g, slate, &Default::default(), Seconds(0.1));
        let b = NewtonFlowSolver::default().solve(&g, slate, &Default::default(), Seconds(0.1));
        assert_same_solution(a, b)?;
    }

    /// CHAIN — I5 breadth: where BOTH solvers converge, their steady flows agree
    /// within 5%. A chain Simple cannot crack is skipped (legal per I3).
    #[test]
    fn chain_fidelity_agreement(
        mids in prop::collection::vec(mid_strategy(), 0..5usize),
        raw_pipes in prop::collection::vec(pipe_strategy(), 6usize..7),
        p_src in 1.0e5..8.0e5f64,
        p_snk in 1.0e5..8.0e5f64,
        fluid in fluid_strategy(),
    ) {
        let (g, _) = build_chain(&mids, &raw_pipes, p_src, p_snk, &fluid);
        let newton = NewtonFlowSolver::default().solve(&g, &fluid.slate, &Default::default(), Seconds(0.1));
        let simple = SimpleFlowSolver::default().solve(&g, &fluid.slate, &Default::default(), Seconds(0.1));
        assert_fidelity_agreement(newton, simple)?;
    }

    /// TREE — I5 breadth over branching topologies (same both-converged rule).
    #[test]
    fn tree_fidelity_agreement(inputs in tree_inputs_strategy()) {
        let g = build_tree(&inputs);
        let slate = &inputs.4.slate;
        let newton = NewtonFlowSolver::default().solve(&g, slate, &Default::default(), Seconds(0.1));
        let simple = SimpleFlowSolver::default().solve(&g, slate, &Default::default(), Seconds(0.1));
        assert_fidelity_agreement(newton, simple)?;
    }

    /// I4 for the Simple solver: two fresh instances (empty warm-start) on the
    /// same tree produce byte-identical results, or both diverge.
    #[test]
    fn tree_simple_is_deterministic(inputs in tree_inputs_strategy()) {
        let g = build_tree(&inputs);
        let slate = &inputs.4.slate;
        let a = SimpleFlowSolver::default().solve(&g, slate, &Default::default(), Seconds(0.1));
        let b = SimpleFlowSolver::default().solve(&g, slate, &Default::default(), Seconds(0.1));
        assert_same_solution(a, b)?;
    }
}

/// I5 comparison: if BOTH solvers return Ok(converged), every non-negligible
/// edge flow must agree within 5% (the I5 contract). Otherwise skip — a network
/// only one solver cracks is legal (I3). An absolute floor (0.1% of Newton
/// throughput + 1e-6 kg/s) ignores near-zero edges, where relative error is
/// meaningless.
fn assert_fidelity_agreement(
    newton: Result<HydraulicSolution, SimError>,
    simple: Result<HydraulicSolution, SimError>,
) -> Result<(), TestCaseError> {
    let (n, s) = match (newton, simple) {
        (Ok(n), Ok(s)) if n.diagnostics.converged && s.diagnostics.converged => (n, s),
        _ => return Ok(()), // at least one did not converge → skip
    };
    let throughput = n
        .edge_mass_flow
        .values()
        .fold(0.0f64, |m, &f| m.max(f.abs()));
    let floor = 1e-6 + 1e-3 * throughput;
    for (eid, &fa) in &n.edge_mass_flow {
        let fb = s.edge_mass_flow[eid];
        let scale = fa.abs().max(fb.abs());
        if scale < floor {
            continue;
        }
        let rel = (fa - fb).abs() / scale;
        prop_assert!(
            rel <= 0.05,
            "fidelity flow mismatch at {:?}: newton={fa}, simple={fb} (rel {rel})",
            eid
        );
    }
    Ok(())
}

/// Two solves of the same network must agree bit-for-bit, or both diverge.
fn assert_same_solution(
    a: Result<HydraulicSolution, SimError>,
    b: Result<HydraulicSolution, SimError>,
) -> Result<(), TestCaseError> {
    match (a, b) {
        (Ok(sa), Ok(sb)) => {
            prop_assert_eq!(sa.edge_mass_flow, sb.edge_mass_flow);
            prop_assert_eq!(
                format!("{:?}", sa.node_pressure),
                format!("{:?}", sb.node_pressure)
            );
            prop_assert_eq!(sa.diagnostics.iterations, sb.diagnostics.iterations);
        }
        (Err(_), Err(_)) => { /* both diverge identically */ }
        _ => prop_assert!(false, "determinism: one solve converged, the other did not"),
    }
    Ok(())
}
