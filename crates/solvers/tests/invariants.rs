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
//!   * CHAIN — Source → (Junction | Pump | Valve | ReliefValve)* → Sink joined
//!     by pipes. Every node is 1-in/1-out, so it is a targeted
//!     reverse-flow-through-device path (randomized end pressures exercise both
//!     flow directions, hence the combined-branch's shifted-oddness reverse
//!     path). It does NOT branch.
//!
//!   * TREE — a hub-biased random tree: interior Junctions with 3+ incident
//!     edges (parents drawn from the first `HUB_SPAN` nodes so branching is
//!     structural, not a full-size-only fluke that shrinking erases — see
//!     `strategy_actually_branches`, which is the guard that this generator
//!     earns its keep). Pumps/valves/PSVs are inserted by SUBDIVIDING an edge,
//!     which gives the device exactly one inlet + one outlet edge (F6 by
//!     construction). Leaves are fixed Source/Sink at random pressures.
//!     A tree may additionally carry RELIEF SPURS — see below.
//!
//! FLOATING SUBNETWORKS ARE NOW GENERATED, and that is a change to the plant
//! model rather than one more `prop_oneof` arm. Every device before the PSV had
//! a conductance fixed for the whole solve, so "every open branch conducts ⇒ the
//! whole graph is anchored" held by construction and this file delegated
//! floating entirely to `newton_reference.rs`'s hand cases. A PSV's conductance
//! is a function of the pressure ITERATE, so it does not.
//!
//! Note the distinction the spur exists for: DEAD END is not FLOATING. A PSV
//! spliced into a chain or a tree edge never floats anything — cut either at one
//! edge and both components still contain a fixed leaf. Only a spur reaches it:
//! `parent → pipe → PSV → pipe → end`, where `end` is either a flare (a fixed
//! low-pressure Sink — the load-bearing case, a PSV that genuinely relieves and
//! in gas service can choke) or a blocked-in DEAD LEG (a free Junction, which
//! has no conducting path to any anchor while the PSV is shut).
//!
//! **A KNOWN DEFECT lived in that second case until M8.0, and these generators
//! are what measured it.** `network::prepare` computed the anchored set ONCE
//! from the seed compile, which is exact for a constant-conductance element and
//! stale for a PSV, both ways round. It is now an ACTIVE SET
//! (`network::solve_with_active_anchoring`, DESIGN §3c): the classification is
//! re-asked between passes until it stops changing. The two hand-checkable
//! plants that pinned the defect now assert the fix, below.
//!
//! A THIRD legal termination came with it, and the tree and chain gates admit it
//! by name: a plant whose classification will not settle is refused rather than
//! answered by iteration parity. Its rate is measured in
//! `the_relief_arm_lifts_relieves_and_floats`, floored so the path is reached
//! and capped so refusing cannot pass for solving.
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
//! Floating-subnetwork pinning by a CLOSED VALVE — an opening an operator set,
//! constant for the whole solve — keeps its hand-checkable cases in
//! `newton_reference.rs` (`floating_subnetwork_with_pump_reports_zero_flow`,
//! `closed_valve_*`). What the generators here add is the case those cannot
//! express: an opening the SOLVE determines, so that whether a subnetwork floats
//! is a property of the answer and not of the graph.
//!
//! I5 HAS A NEW BOUNDARY, and it is asserted rather than skipped. A PSV in
//! REVERSE flow makes the network genuinely multi-rooted: its spring senses its
//! own inlet flange, so in reverse the pressure that opens it is the one the
//! flow arrives at — opening it RAISES its own sensed pressure, positive
//! feedback, and shut-with-no-flow and open-with-flow are both exact roots.
//! (Relieving forward the feedback is negative and the root is unique.) So the
//! two fidelities may legitimately land on different answers, and
//! `chain_fidelity_agreement` responds by PROVING both are roots rather than
//! declining to compare — a skip would have masked any real Simple bug that
//! happened to produce reverse flow through a PSV.

use proptest::prelude::*;
use proptest::strategy::{Strategy, ValueTree};
use proptest::test_runner::TestRunner;
use refinery_core::components::{Composition, PseudoComponent, Slate};
use refinery_core::error::SimError;
use refinery_core::graph::{LeakRole, Node, NodeId, NodeKind, Pipe, PlantGraph};
use refinery_core::stream::Stream;
use refinery_core::traits::{FlowSolver, HydraulicSolution};
use refinery_core::units::*;
use refinery_solvers::elements::isentropic_critical_drop_ratio;
use refinery_solvers::{NewtonFlowSolver, SimpleFlowSolver};
use std::collections::BTreeMap;

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
                cp_shape: None,
                phase: refinery_core::components::Phase::Gas,
            },
            PseudoComponent {
                name: "propane".into(),
                tb: Kelvin(231.05),
                molar_mass: KgPerMol(0.044096),
                density: None,
                cp: JPerKgK(1670.0),
                cp_shape: None,
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
        leak: LeakRole::None,
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

/// A leak orifice edge (M6.1): zero geometry, an `Orifice` role carrying the
/// commanded area, and the plant's own fluid.
///
/// Zero length and diameter are what the loader writes and what `compile_edge`
/// never reads — it returns on the `Orifice` role before touching either. They
/// are a tripwire rather than a value: if that early return were removed,
/// `pipe_resistance` on zeros is non-finite and the edge fails loudly instead of
/// quietly acquiring a second resistance in series with the hole.
fn leak_pipe(bore: f64, name: &str, fluid: &Fluid) -> Pipe {
    Pipe {
        leak: LeakRole::Orifice {
            area: SquareMeter(std::f64::consts::PI * bore * bore / 4.0),
        },
        ..pipe((0.0, 0.0, 0.02, 0.0), name, fluid)
    }
}

/// Bore [m] of the hole punched in one junction, or `None` for an intact node.
///
/// **One in four, and the range is sized from the resistances rather than
/// picked.** A leak that cannot compete with the pipes around it is generated
/// and tests nothing — the failure mode this file has now recorded twice (a
/// generated arm born vacuous). At the middle of `pipe_strategy`'s range a pipe
/// contributes `α ≈ 8e6`, while an orifice contributes `α = ρ/(2·Cd²·A²)`, so a
/// 0.15 m bore gives `α ≈ 4e6` (the leak dominates), 0.05 m gives `α ≈ 3.5e8`
/// (a trickle) and 0.01 m gives `α ≈ 2e11` (effectively intact). The range
/// therefore spans dominant to negligible, and
/// `the_leak_arm_conducts_and_is_refused_both_ways` measures how many samples
/// land where the balance gate can actually see the difference.
///
/// A leak is generated WITHOUT regard to phase, deliberately. Half of
/// `fluid_strategy`'s plants are gas. Until M37 a hole in one was refused by
/// `compile_edge`'s second door; since M37 it is the isentropic nozzle law
/// (docs/DESIGN.md §41), and the same draws now reach the balance gate in gas —
/// counted, choked ones among them, in
/// `the_leak_arm_conducts_and_is_refused_both_ways`.
fn leak_strategy() -> impl Strategy<Value = Option<f64>> {
    prop_oneof![
        3 => Just(None),
        1 => (0.01..0.15f64).prop_map(Some),
    ]
}

/// The refusal a LEAK makes legal.
///
/// A back-feed refusal is an ASSERTION that fired: the plant went below
/// atmospheric with a hole open, and `finalize` refused to draw an arbitrary
/// composition into it rather than reporting a plausible number (DESIGN §3b). It
/// is not a divergence, so it may not be swallowed by the `SolverDiverged` arm;
/// it is counted in the meta-test below so it cannot quietly become the *only*
/// thing the leak arm produces. (The gas refusal that stood beside it until M37
/// is gone: a gas hole is now sized, docs/DESIGN.md §41.)
fn is_legal_leak_refusal(e: &SimError) -> bool {
    matches!(e, SimError::Numerical(m) if m.contains("back-feeds"))
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
    Relief { cv: f64, set: f64, band: f64 },
}

fn mid_strategy() -> impl Strategy<Value = Mid> {
    prop_oneof![
        Just(Mid::Junction),
        (10.0..60.0f64, 1e2..1e4f64, any::<bool>()).prop_map(|(h0, a, on)| Mid::Pump { h0, a, on }),
        // opening ≥ 0.05 keeps the chain conducting (closed-valve floating is
        // covered by a dedicated unit test, not here).
        (1e-4..5e-3f64, 0.05..1.0f64).prop_map(|(cv, opening)| Mid::Valve { cv, opening }),
        relief_strategy().prop_map(|(cv, set, band)| Mid::Relief { cv, set, band }),
    ]
}

/// `(cv_max, set_pressure, accumulation)` for a generated PSV.
///
/// The set pressure spans the INTERIOR of the end-pressure range
/// (`1.0e5..8.0e5`) rather than sitting under it or over it, and that is the one
/// tuning decision in this arm: it is what makes shut / partially lifted / fully
/// lifted all common. A band of 0.2–1.5 bar against that range keeps the
/// partial-lift window wide enough to land in — a 1 mbar band would be a step in
/// all but name and the smoothstep would go untested — while staying inside the
/// 10–21% of set pressure real accumulation allows (API 520).
///
/// Unlike a plain valve there is no `opening` to generate: that is the whole
/// point of the device, and where it comes from — the node's OWN pressure, read
/// inside the solve — is what the arm exists to exercise.
fn relief_strategy() -> impl Strategy<Value = (f64, f64, f64)> {
    (1e-4..5e-3f64, 1.5e5..7.5e5f64, 0.2e5..1.5e5f64)
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
        Mid::Relief { cv, set, band } => NodeKind::ReliefValve {
            cv_max: valve_cv(cv, fluid),
            set_pressure: Pascal(set),
            accumulation: Pascal(band),
            x_t: fluid.x_t,
            blowdown: None,
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
    Relief { cv: f64, set: f64, band: f64 },
}

fn device_strategy() -> impl Strategy<Value = MidDevice> {
    prop_oneof![
        3 => Just(MidDevice::None),
        1 => (10.0..60.0f64, 1e2..1e4f64, any::<bool>())
            .prop_map(|(h0, a, on)| MidDevice::Pump { h0, a, on }),
        1 => (1e-4..5e-3f64, 0.05..1.0f64)
            .prop_map(|(cv, opening)| MidDevice::Valve { cv, opening }),
        1 => relief_strategy().prop_map(|(cv, set, band)| MidDevice::Relief { cv, set, band }),
    ]
}

/// Where a relief SPUR discharges — and the reason the spur exists at all.
///
/// A PSV spliced INTO a chain or a tree edge, like the arms above, never floats
/// anything: cut a chain or a tree at one edge and each of the two components
/// still contains a fixed leaf, so both stay anchored. Dead end is not the same
/// property as floating, and only a spur — a branch hanging off the network,
/// which neither generator could previously build — reaches the second one.
#[derive(Debug, Clone)]
enum SpurEnd {
    /// A flare header: a FIXED low-pressure sink, and the load-bearing case. A
    /// PSV here actually relieves — real flow, into a hub that must still
    /// balance, and in gas service a branch that can choke. This is
    /// `relief_blowdown.toml`'s geometry with the numbers generated.
    Flare(f64),
    /// A blocked-in dead leg: a FREE junction, no reservoir behind it. Carries
    /// zero flow whether the PSV is shut or open (nothing downstream to take
    /// any), so it has no physics content and is not pretended to have one. Its
    /// single purpose: while the PSV is shut its outlet edge does not conduct,
    /// so the terminal has no conducting path to ANY anchor and `prepare` must
    /// pin it as floating. That is the path this file's header used to delegate
    /// entirely to `newton_reference.rs`'s hand-built cases.
    DeadLeg,
}

/// A relief branch hung off a primary tree node: `parent → pipe → PSV → pipe →
/// end`. It carries its own two pipes rather than indexing a shared vec, which
/// keeps `TreeInputs` at six fields and every length fixed (shrink-safe).
#[derive(Debug, Clone)]
struct Spur {
    cv: f64,
    set: f64,
    band: f64,
    end: SpurEnd,
    inlet: (f64, f64, f64, f64),
    outlet: (f64, f64, f64, f64),
}

/// One in four primary nodes gets a spur, so a typical tree carries one or two
/// and a fair number carry none — the no-spur tree must stay common, since it is
/// every pre-existing case. The flare pressure sits BELOW the primary tree's
/// range (`1.0e5..8.0e5`), because a flare header at receiver pressure would
/// take nothing and the relieving case would be generated but never reached.
fn spur_strategy() -> impl Strategy<Value = Option<Spur>> {
    prop_oneof![
        3 => Just(None),
        1 => (
            relief_strategy(),
            prop_oneof![
                1 => (0.9e5..1.6e5f64).prop_map(SpurEnd::Flare),
                1 => Just(SpurEnd::DeadLeg),
            ],
            pipe_strategy(),
            pipe_strategy(),
        )
            .prop_map(|((cv, set, band), end, inlet, outlet)| Some(Spur {
                cv,
                set,
                band,
                end,
                inlet,
                outlet,
            })),
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
        NodeKind::Junction
            | NodeKind::Pump { .. }
            | NodeKind::Valve { .. }
            // A relief valve is free (it pins no pressure) and it is checked
            // like every other free node. Two of them are worth naming: a PSV
            // node is a DEAD END whenever it is shut, and a floating dead-leg
            // terminal balances trivially because all its edges report zero —
            // which is the correct answer and is asserted, not skipped.
            | NodeKind::ReliefValve { .. }
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
    Vec<Option<Spur>>,         // spurs[i]: a relief branch hung off primary node i
    Vec<Option<f64>>,          // leaks[i]: orifice bore [m] punched in junction i
);

fn tree_inputs_strategy() -> impl Strategy<Value = TreeInputs> {
    (
        prop::collection::vec(0..1000usize, 1..=MAX_K),
        prop::collection::vec((any::<bool>(), 1.0e5..8.0e5f64), MAX_NODES..=MAX_NODES),
        prop::collection::vec(device_strategy(), MAX_K..=MAX_K),
        prop::collection::vec(pipe_strategy(), (2 * MAX_K)..=(2 * MAX_K)),
        fluid_strategy(),
        prop::collection::vec(spur_strategy(), MAX_NODES..=MAX_NODES),
        prop::collection::vec(leak_strategy(), MAX_NODES..=MAX_NODES),
    )
}

/// Build a hub-biased random tree. Child `j` (1..=k) attaches to parent
/// `raw_parents[j-1] % min(j, HUB_SPAN)`, so the first `HUB_SPAN` nodes become
/// hubs. Degree-1 nodes are leaves ⇒ fixed Source/Sink (pressure reference);
/// higher-degree nodes are Junctions. A selected edge is subdivided by a device.
fn build_tree(inputs: &TreeInputs) -> PlantGraph {
    let (raw_parents, fixed_specs, devices, pipes, fluid, spurs, leaks) = inputs;
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
                    MidDevice::Relief { cv, set, band } => NodeKind::ReliefValve {
                        cv_max: valve_cv(*cv, fluid),
                        set_pressure: Pascal(*set),
                        accumulation: Pascal(*band),
                        x_t: fluid.x_t,
                        blowdown: None,
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

    // Relief spurs, hung off the finished tree rather than woven into it. The
    // order matters for what stays true: every primary node's kind was already
    // decided by its PRIMARY degree, so a spur off a degree-1 node leaves that
    // node the fixed Source/Sink it was and simply gives it a second edge (legal
    // — only devices are degree-constrained). A spurless tree is therefore built
    // exactly as before, which is what keeps every pre-existing case meaningful.
    for (i, spur) in spurs.iter().enumerate().take(n_nodes) {
        let Some(spur) = spur else { continue };
        let psv = g.add_node(Node {
            name: format!("psv{i}"),
            kind: NodeKind::ReliefValve {
                cv_max: valve_cv(spur.cv, fluid),
                set_pressure: Pascal(spur.set),
                accumulation: Pascal(spur.band),
                x_t: fluid.x_t,
                blowdown: None,
            },
            heat_input: Watt(0.0),
        });
        let end = match spur.end {
            SpurEnd::Flare(p) => g.add_node(Node {
                name: format!("flare{i}"),
                kind: NodeKind::Sink {
                    pressure: Pascal(p),
                    temperature: T_AMBIENT,
                    composition: fluid.composition.clone(),
                },
                heat_input: Watt(0.0),
            }),
            SpurEnd::DeadLeg => g.add_node(Node {
                name: format!("deadleg{i}"),
                kind: NodeKind::Junction,
                heat_input: Watt(0.0),
            }),
        };
        // Oriented parent → PSV → end, so the PSV folds into its OUTLET edge and
        // its own node pressure is the one the spring senses — the same
        // fold-at-source convention every other device here follows.
        g.add_pipe(ids[i], psv, pipe(spur.inlet, &format!("s{i}a"), fluid));
        g.add_pipe(psv, end, pipe(spur.outlet, &format!("s{i}b"), fluid));
    }

    // LEAKS, last, and hung only off JUNCTIONS. Three choices here are load
    // bearing:
    //
    // - **Only junctions.** That is where the loader puts one — a declared leak
    //   splits its pipe and hangs the orifice off the new midpoint junction — and
    //   it is also the only place a leak has anything to say about I1. A hole in
    //   a leaf would run reservoir → Atmosphere, both pressures pinned, and
    //   contribute to no free node's balance at all: generated, conducting, and
    //   testing nothing.
    // - **One shared Atmosphere, created only if some leak exists.** A tree with
    //   no leak is then EXACTLY the tree it was before this arm, which is what
    //   keeps every pre-existing case meaning what it meant. The same reasoning
    //   the spur arm above is built on.
    // - **After the spurs.** A leak adds an edge to its junction, and the spur
    //   loop reads primary node kinds; ordering it last means neither arm can
    //   change what the other sees.
    let mut vent: Option<refinery_core::graph::NodeId> = None;
    for (i, leak) in leaks.iter().enumerate().take(n_nodes) {
        let Some(bore) = leak else { continue };
        if !matches!(g.node(ids[i]).kind, NodeKind::Junction) {
            continue;
        }
        let air = match vent {
            Some(existing) => existing,
            None => {
                let created = g.add_node(Node {
                    name: "atmosphere".into(),
                    kind: NodeKind::Atmosphere,
                    heat_input: Watt(0.0),
                });
                vent = Some(created);
                created
            }
        };
        // Junction → Atmosphere, so positive graph direction is OUTWARD and a
        // negative flow is unambiguously the back-feed `finalize` refuses.
        g.add_pipe(ids[i], air, leak_pipe(*bore, &format!("leak{i}"), fluid));
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
        let mut inputs = strat
            .new_tree(&mut runner)
            .expect("strategy produces a value")
            .current();
        // STRIP THE SPURS before measuring, or this guard passes for the wrong
        // reason. A spur adds an edge to its primary parent, so a degree-2
        // interior node carrying one would count as a 3+ degree "hub" — and the
        // whole purpose of this number is that the TREE strategy branches, i.e.
        // that it earns its keep over `build_chain`. A relief branch is not
        // branching in that sense. Stripping them keeps the measurement's
        // historical meaning (≈60% branch) comparable across this slice.
        inputs.5.iter_mut().for_each(|s| *s = None);
        // And the LEAKS, for the same reason and one more. A leak also adds an
        // edge to its junction, so it would inflate this count exactly as a spur
        // would; and it is not branching in the sense this number exists to
        // measure. Stripping keeps the ≈60% historical figure comparable across
        // both slices.
        inputs.6.iter_mut().for_each(|l| *l = None);
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
    let mut dead = 0usize;
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
        // A chain carrying a SHUT PSV is dead end to end — every flow is zero,
        // so every edge falls under the floor below and `ok` would be trivially
        // true. Counting those as agreement is exactly the vacuity this test
        // exists to detect, one level down, so they are excluded from BOTH
        // counters and reported separately.
        let throughput = n
            .edge_mass_flow
            .values()
            .fold(0.0f64, |m, &f| m.max(f.abs()));
        if throughput <= 1e-9 {
            dead += 1;
            continue;
        }
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

    println!(
        "simple-vs-newton on chains: newton converged with flow {newton_ok}/{SAMPLES}; \
         agreed {agreed}; excluded as dead (a shut PSV zeroes the chain) {dead}"
    );
    assert!(newton_ok > 0, "Newton converged on no chain samples");
    // The PSV arm must not have eaten the population this guard measures.
    assert!(
        newton_ok * 4 >= SAMPLES,
        "only {newton_ok}/{SAMPLES} chains carried any flow at all ({dead} were dead) — \
         the relief arm has crowded out the live chains this guard is about"
    );
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
    let pressures: std::collections::BTreeMap<_, _> = sol
        .node_pressure
        .iter()
        .map(|(n, p)| (*n, p.value()))
        .collect();
    let (cv_max, opening, x_t) = match graph.node(src).kind {
        // Fold-at-source: a device folds into the edge LEAVING it, so only an
        // edge whose `src` is the valve carries one.
        NodeKind::Valve {
            cv_max,
            opening,
            x_t: Some(x_t),
        } => (cv_max, opening, x_t),
        // A PSV chokes on exactly the same law; its opening is just read from
        // the plant instead of set by an operator.
        NodeKind::ReliefValve {
            cv_max,
            set_pressure,
            accumulation,
            x_t: Some(x_t),
            ..
        } => (
            cv_max,
            refinery_solvers::elements::relief_opening(
                pressures[&src],
                set_pressure.value(),
                accumulation.value(),
            ),
            x_t,
        ),
        // A check valve chokes on the same law too (M31); its opening is read
        // off the forward drive across its branch, net of the static head.
        NodeKind::CheckValve {
            cv_max,
            full_open,
            x_t: Some(x_t),
        } => {
            let Ok(c) = refinery_solvers::network::compile_edge(
                graph,
                eid,
                &fluid.slate,
                &Default::default(),
                &pressures,
            ) else {
                return false;
            };
            let drive = pressures[&src] - pressures[&tgt] - c.branch.beta;
            (
                cv_max,
                refinery_solvers::elements::check_opening(drive, full_open.value()),
                x_t,
            )
        }
        _ => return false,
    };
    // A SHUT valve must be rejected before the plateau is computed, or it counts
    // as choked and the measurement inflates in the flattering direction:
    // `α_valve = ρ_rel/0² = +∞` gives `plateau = 0` against a flow of `0`, and
    // `0 ≥ 0` is true. Only a PSV can reach this — a generated `Valve` has
    // `opening ≥ 0.05` by construction — which is precisely why the guard had to
    // arrive with this arm.
    if opening < refinery_solvers::network::OPEN_EPS {
        return false;
    }
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
    //
    // TWO-SIDED since M31: a flow ABOVE its plateau is not choked, it is a valve
    // that never folded at all (the incompressible law overshoots the cap). The
    // one-sided test counted that fault as a choke, and the gas check-valve arm
    // was blind to it (docs/DESIGN.md §34, mutation 2). On the correct engine the
    // flow never exceeds the plateau, so no recorded count moved.
    q >= plateau * (1.0 - 1e-4) && q <= plateau * (1.0 + 1e-4)
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
    let (mut gas_trees, mut choked_trees, mut choked_spurs) = (0usize, 0usize, 0usize);
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
        if !n.diagnostics.converged {
            continue;
        }
        // SPUR chokes are counted apart from SPLICED-device chokes, for the same
        // reason trees are counted apart from chains one paragraph up: this
        // counter's job is to protect the branching `edge_flows`-vs-`assemble`
        // cross-check, and a PSV choking on a dead-end relief branch is not that
        // check. Folding the two together inflated it from 53/189 to 83/179 —
        // the guard would then have been satisfied by the arm that has nothing
        // to do with what it guards.
        let mut spliced = false;
        let mut spur = false;
        for e in g.edge_ids() {
            if !valve_edge_is_choked(&g, &n, &inputs.4, e) {
                continue;
            }
            // `build_tree` owns every name here: `dev{j}` is spliced into a
            // primary edge, `psv{i}` hangs off a spur.
            if g.node(g.endpoints(e).0).name.starts_with("psv") {
                spur = true;
            } else {
                spliced = true;
            }
        }
        choked_trees += usize::from(spliced);
        choked_spurs += usize::from(spur);
    }
    println!(
        "gas trees: {gas_trees}/{SAMPLES}; with a choked SPLICED valve {choked_trees}/{gas_trees}; \
         with a choked SPUR psv {choked_spurs}/{gas_trees}"
    );

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
            // The THIRD legal termination, new with M8.0. A plant whose
            // anchoring will not settle has two self-consistent answers
            // (chatter) or none yet; the solver refuses rather than picking one
            // by iteration parity, and refusing is exactly as legal as
            // diverging. Named by TYPE deliberately — accepting it by matching a
            // substring of a message would also accept an unrelated numerical
            // failure that happened to be worded like one. The RATE is measured
            // rather than left to this matcher: see
            // `the_relief_arm_lifts_relieves_and_floats`.
            Err(SimError::AnchoringUnsettled { .. }) => { /* acceptable per I3, M8.0 */ }
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
            // A leak's back-feed refusal is a legal outcome but NOT a divergence,
            // so it is admitted by name rather than folded into the arm above —
            // and the solved rate is floored in
            // `the_leak_arm_conducts_and_is_refused_both_ways`, so a change that
            // made every leaky tree refuse would fail there instead of quietly
            // emptying this gate.
            Err(ref other) if is_legal_leak_refusal(other) => {}
            // The THIRD legal termination, new with M8.0 — see the chain arm
            // above for why it is admitted by TYPE rather than by message, and
            // where its rate is measured.
            Err(SimError::AnchoringUnsettled { .. }) => { /* acceptable per I3, M8.0 */ }
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
        assert_fidelity_agreement(&g, &fluid, newton, simple, true)?;
    }

    /// TREE — I5 breadth over branching topologies (same both-converged rule).
    #[test]
    fn tree_fidelity_agreement(inputs in tree_inputs_strategy()) {
        let g = build_tree(&inputs);
        let fluid = &inputs.4;
        let newton = NewtonFlowSolver::default().solve(&g, &fluid.slate, &Default::default(), Seconds(0.1));
        let simple = SimpleFlowSolver::default().solve(&g, &fluid.slate, &Default::default(), Seconds(0.1));
        // `prove_roots: false` — a tree can carry the frozen-anchoring defect,
        // which makes a re-derived anchored set legitimately disagree with the
        // one the solve used. See `assert_fidelity_agreement`.
        assert_fidelity_agreement(&g, fluid, newton, simple, false)?;
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

/// The `eps_dp` both solvers carry in `Default` (`newton_flow.rs`,
/// `simple_flow.rs`). Recomputing a residual at any OTHER value measures the
/// difference between two regularizations instead of the residual, and the
/// number is not small: `smooth_signed_sqrt` deviates by O(eps/Δp), so on the
/// 44.7 Pa drop of the very case that motivated this helper, 1e-6 against this
/// 1.0 reports a 1.1% imbalance on a root that is exact to 7e-7 kg/s.
const SOLVER_EPS_DP: f64 = 1.0;

/// Node pressures as plain f64, the form every `network` entry point takes.
fn pressures_of(sol: &HydraulicSolution) -> BTreeMap<NodeId, f64> {
    sol.node_pressure
        .iter()
        .map(|(n, p)| (*n, p.value()))
        .collect()
}

/// Every relief valve's opening under a given pressure assignment — the
/// signature that says WHICH branch of the pressure-actuated characteristic a
/// solution sits on.
fn relief_openings(graph: &PlantGraph, pressures: &BTreeMap<NodeId, f64>) -> BTreeMap<NodeId, f64> {
    let mut out = BTreeMap::new();
    for nid in graph.node_ids() {
        if let NodeKind::ReliefValve {
            set_pressure,
            accumulation,
            ..
        } = graph.node(nid).kind
        {
            out.insert(
                nid,
                refinery_solvers::elements::relief_opening(
                    pressures[&nid],
                    set_pressure.value(),
                    accumulation.value(),
                ),
            );
        }
    }
    out
}

/// Is this pressure assignment an exact root of the network? Returns
/// `(worst |node imbalance| [kg/s], throughput [kg/s])`.
///
/// Judged FROM SCRATCH: nothing is read back from the solution except the
/// pressures themselves. The edges are recompiled at them, the anchored set is
/// re-derived from those edges, and the flows come out of `edge_flows`. That
/// independence is the whole point — it is what turns "the two fidelities
/// disagree, so skip the comparison" into "this plant has two roots, and here is
/// the proof", and a genuine solver bug cannot satisfy the second one.
///
/// `anchors` is the fixed set: these generators build no capacitive node, so
/// `!is_free` is exactly Source ∪ Sink (a Vessel would have to be added here).
fn worst_recomputed_imbalance(
    graph: &PlantGraph,
    fluid: &Fluid,
    pressures: &BTreeMap<NodeId, f64>,
) -> Result<(f64, f64), SimError> {
    let compiled = refinery_solvers::network::compile_edges(
        graph,
        &fluid.slate,
        &Default::default(),
        pressures,
    )?;
    let anchors: std::collections::BTreeSet<NodeId> = graph
        .node_ids()
        .filter(|&n| !is_free(&graph.node(n).kind))
        .collect();
    let anchored = refinery_solvers::network::anchored_set(graph, &compiled, &anchors);
    let res = refinery_solvers::network::edge_flows(
        graph,
        &compiled,
        pressures,
        &anchored,
        SOLVER_EPS_DP,
    );
    let mut worst = 0.0f64;
    for nid in graph.node_ids() {
        if !is_free(&graph.node(nid).kind) {
            continue;
        }
        let mut bal = 0.0;
        for (e, _other, incoming) in graph.incident(nid) {
            bal += if incoming {
                res.mass_flow[&e]
            } else {
                -res.mass_flow[&e]
            };
        }
        worst = worst.max(bal.abs());
    }
    Ok((worst, res.throughput))
}

/// I5 comparison: if BOTH solvers return Ok(converged), every non-negligible
/// edge flow must agree within 5% (the I5 contract). Otherwise skip — a network
/// only one solver cracks is legal (I3). An absolute floor (0.1% of Newton
/// throughput + 1e-6 kg/s) ignores near-zero edges, where relative error is
/// meaningless.
///
/// **A disagreement is not automatically a failure any more, and the escape is
/// an ASSERTION rather than a skip.** A PSV in reverse flow leaves the network
/// genuinely multi-rooted (see the module header), so the two fidelities can
/// each be exactly right and differ. Declining to compare there would have
/// weakened I5 by exactly the amount needed to hide a real Simple bug that
/// happened to produce reverse flow through a PSV. Instead a disagreement must
/// clear two bars: the two solutions put some PSV on DIFFERENT branches of its
/// characteristic (the only mechanism in this model that can multiply roots),
/// and — where `prove_roots` — each is independently verified to be an exact
/// root. Neither is satisfiable by a solver that is simply wrong.
///
/// `prove_roots` is false for TREES, and the reason is a defect rather than a
/// principle: `worst_recomputed_imbalance` re-derives the anchored set at the
/// solution, which is precisely what `network::prepare` does NOT do, so on a
/// tree carrying the anchoring defect M8.0 fixed (see the tests below)
/// the proof would fail for a reason that has nothing to do with multiplicity.
/// Chains cannot reach that: their free nodes lose their anchor only when a
/// whole segment is sealed off between two shut PSVs, and a sealed segment parks
/// at a pressure that keeps those PSVs shut, so seed and solution agree.
fn assert_fidelity_agreement(
    graph: &PlantGraph,
    fluid: &Fluid,
    newton: Result<HydraulicSolution, SimError>,
    simple: Result<HydraulicSolution, SimError>,
    prove_roots: bool,
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
    let mut mismatch: Option<(refinery_core::graph::EdgeId, f64, f64, f64)> = None;
    for (eid, &fa) in &n.edge_mass_flow {
        let fb = s.edge_mass_flow[eid];
        let scale = fa.abs().max(fb.abs());
        if scale < floor {
            continue;
        }
        let rel = (fa - fb).abs() / scale;
        if rel > 0.05 && mismatch.is_none_or(|(_, _, _, w)| rel > w) {
            mismatch = Some((*eid, fa, fb, rel));
        }
    }
    let Some((eid, fa, fb, rel)) = mismatch else {
        return Ok(());
    };

    let (pn, ps) = (pressures_of(&n), pressures_of(&s));
    let (open_n, open_s) = (relief_openings(graph, &pn), relief_openings(graph, &ps));
    let split = open_n.iter().any(|(nid, a)| (a - open_s[nid]).abs() > 1e-6);
    prop_assert!(
        split,
        "fidelity flow mismatch at {eid:?}: newton={fa}, simple={fb} (rel {rel}) — and NOT \
         explained by relief-valve multiplicity: every PSV (if any) sits on the same branch \
         of its characteristic in both solutions, so the two solvers disagree about a network \
         with a unique steady state"
    );
    if !prove_roots {
        return Ok(());
    }
    for (label, pressures) in [("newton", &pn), ("simple", &ps)] {
        let (worst, tp) = worst_recomputed_imbalance(graph, fluid, pressures)
            .map_err(|e| TestCaseError::fail(format!("recompiling {label}'s solution: {e}")))?;
        // 10x the loosest convergence criterion either solver grades itself
        // against (Simple: 1e-8 + 1e-6·throughput).
        prop_assert!(
            worst <= 1e-7 + 1e-5 * tp,
            "the two fidelities disagree at {eid:?} (newton={fa}, simple={fb}, rel {rel}) and \
             put a PSV on different branches, but {label}'s answer is NOT a root: worst \
             recomputed node imbalance {worst:.3e} kg/s against a throughput of {tp:.3e}. \
             Multiplicity would make BOTH exact; one of them is simply wrong"
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

// ---------------------------------------------------------------------------
// Non-vacuity for the RELIEF arm — the same three questions the gas arm asks,
// re-asked for a device whose opening the SOLVE determines.
//
// A PSV can be generated and still test nothing, in more ways than a valve can:
//   1. never generated at all (a `prop_oneof` weight typo);
//   2. generated but always on ONE branch of its characteristic — all shut (the
//      chain is then dead end to end and every gate passes on zeros) or all
//      fully lifted (the smoothstep's interior, the only part of it that is not
//      a constant, then goes untouched);
//   3. spurs generated but never RELIEVING — a flare branch that never carries
//      flow is a plain dead end, and the load-bearing case is gone;
//   4. dead legs generated but never FLOATING, which would leave the whole
//      point of the spur — a subnetwork whose anchoring the ANSWER decides —
//      unreached;
//   5. the configuration where the spring's own inlet reading is
//      DISTINGUISHABLE from the upwind one never occurring. That last is not an
//      abstract worry: it is the single configuration separating the shipped
//      model from the most plausible mutation of it, so a catch resting on it
//      must not rest on luck. It is now the reachability floor under an
//      ASSERTION — a PSV shut at its own flange must carry nothing — rather than
//      a number reported next to one. It was the latter first, and the mutation
//      run is what exposed the difference: `pressures[&src]` → `upwind` in
//      `compile_edge` was caught by nothing in the workspace while this counter
//      stood at well over its floor, because every conservation and determinism
//      invariant holds just as well for a wrongly-lifted valve.
//
// Two numbers here are DEFERRAL EVIDENCE rather than health checks, and they are
// bounded so the deferral cannot quietly worsen: Newton's divergence rate on
// spur trees, and Simple's convergence rate on them. See docs/ROADMAP.md M5.4
// and the anchoring tests below.
// ---------------------------------------------------------------------------

/// How far below its set pressure a PSV's flange must sit before the arm will
/// hold the shipped model to "shut, therefore carrying nothing" [Pa].
///
/// Not a tolerance on the physics but on the REPORTING path: `edge_flows` uses
/// the branch compiled at the last Newton iterate, so a flange within
/// convergence distance of the set pressure may have been compiled on the other
/// side of it. 1 kPa is AT MOST 5% of the accumulation band — the bound is taken
/// against the narrowest band `relief_strategy` draws, and it spans 0.2–1.5 bar,
/// so most samples see well under that. Wide enough to be robust, narrow enough
/// that the discriminating population survives it: 21 configurations do, against
/// the floor of 10 below. Both numbers measured, neither inferred.
const SPRING_MARGIN: f64 = 1.0e3;

/// The outlet edge a device at `nid` folds into (fold-at-source), and its far
/// end. `None` for a node with no outgoing edge.
fn outlet_of(graph: &PlantGraph, nid: NodeId) -> Option<(refinery_core::graph::EdgeId, NodeId)> {
    graph
        .incident(nid)
        .into_iter()
        .find(|(_, _, incoming)| !incoming)
        .map(|(e, other, _)| (e, other))
}

#[test]
fn the_relief_arm_lifts_relieves_and_floats() {
    const SAMPLES: usize = 400;
    let mut runner = TestRunner::deterministic();

    // ---- chains -----------------------------------------------------------
    let chain_strat = (
        prop::collection::vec(mid_strategy(), 1..5usize),
        prop::collection::vec(pipe_strategy(), 6usize..7),
        1.0e5..8.0e5f64,
        1.0e5..8.0e5f64,
        fluid_strategy(),
    );
    let (mut psv_chains, mut shut, mut partial, mut full) = (0usize, 0usize, 0usize, 0usize);
    let (mut discriminating, mut reverse_open) = (0usize, 0usize);
    // M8.0's new termination, counted on chains as well as trees: a chain can
    // carry a relief mid-run, and the active-set loop refuses a plant whose
    // classification will not settle rather than picking one of its answers.
    let (mut chain_cycled, mut chain_capped) = (0usize, 0usize);
    for _ in 0..SAMPLES {
        let (mids, pipes, p_src, p_snk, fluid) = chain_strat
            .new_tree(&mut runner)
            .expect("strategy produces a value")
            .current();
        if !mids.iter().any(|m| matches!(m, Mid::Relief { .. })) {
            continue;
        }
        psv_chains += 1;
        let (g, _) = build_chain(&mids, &pipes, p_src, p_snk, &fluid);
        let outcome =
            NewtonFlowSolver::default().solve(&g, &fluid.slate, &Default::default(), Seconds(0.1));
        if let Err(SimError::AnchoringUnsettled { cycled, .. }) = &outcome {
            if *cycled {
                chain_cycled += 1;
            } else {
                chain_capped += 1;
            }
        }
        let Ok(sol) = outcome else {
            continue;
        };
        if !sol.diagnostics.converged {
            continue;
        }
        let pressures = pressures_of(&sol);
        for (nid, opening) in relief_openings(&g, &pressures) {
            if opening < refinery_solvers::network::OPEN_EPS {
                shut += 1;
            } else if opening > 1.0 - 1e-9 {
                full += 1;
            } else {
                partial += 1;
            }
            let NodeKind::ReliefValve {
                set_pressure,
                accumulation,
                ..
            } = g.node(nid).kind
            else {
                unreachable!("relief_openings yields only relief valves")
            };
            let Some((eid, other)) = outlet_of(&g, nid) else {
                continue;
            };
            // The configuration that tells the SPRING's reading apart from the
            // UPWIND one: this PSV's own flange is below set (so the shipped
            // model holds it shut) while the far end of its outlet edge is high
            // enough that a model reading the upwind end would lift it WIDE.
            //
            // Both sides carry a margin, and neither is cosmetic. The flange
            // needs one because `edge_flows` reports flows from the branch
            // compiled at the LAST iterate rather than re-compiled at the
            // converged pressures: a sample whose flange lands a hair below set
            // could have been compiled a hair above it, and the shipped model
            // would then legitimately report a whisker of flow. The far end
            // needs one so the mutation's opening is substantial rather than
            // infinitesimal — a discriminating configuration that separates the
            // two readings by 1e-9 of opening is not one a gate can stand on.
            let shut_at_its_own_flange = pressures[&nid] <= set_pressure.value() - SPRING_MARGIN;
            let upwind_reading_would_lift_it = refinery_solvers::elements::relief_opening(
                pressures[&other],
                set_pressure.value(),
                accumulation.value(),
            ) > 0.1;
            if shut_at_its_own_flange && upwind_reading_would_lift_it {
                discriminating += 1;
                // THE GATE, and the reason this is no longer merely counted.
                // `discriminating` was a reachability floor: it proved the
                // generator VISITS the one configuration separating the shipped
                // spring from its most plausible mutation, and then asserted
                // nothing about the answer there. Mutating `pressures[&src]` to
                // `upwind` in `compile_edge` was caught by ZERO gates in the
                // whole workspace — the conservation and determinism invariants
                // all hold for a wrongly-lifted valve, because a wrong opening
                // still yields a perfectly conservative solve. Reachability is
                // not sensitivity ([[a-counter-is-not-a-gate]]).
                //
                // What makes this non-tautological: it reads the SOLVED FLOW,
                // not the opening. Re-deriving the opening from `pressures[&nid]`
                // and asserting it is zero would only restate `relief_opening`'s
                // own algebra. A shut valve has zero conductance, so the edge it
                // folds into must carry NOTHING; the upwind reading lifts it
                // against a strictly positive Δp (the far end is above set and
                // the flange below it) and this flow becomes nonzero.
                //
                // And this is why FINDING 1's non-uniqueness does not destabilize
                // it. The configuration selected here IS the multi-root geometry —
                // driven backwards, shut-and-dead and open-with-reverse-flow can
                // both be self-consistent — but the predicate reads the CONVERGED
                // state, so it only ever asserts about the root the solve actually
                // reported. A solve that landed on the open root converges with
                // its flange ABOVE set and is excluded rather than failed. What is
                // asserted is internal consistency of one reported answer, not a
                // choice between two ([[prove-the-exception-dont-skip-it]]).
                let carried = sol.edge_mass_flow[&eid];
                assert!(
                    carried.abs() < 1e-12,
                    "a PSV whose own inlet flange is at {:.0} Pa, below its {:.0} Pa set \
                     pressure, must be SHUT and its outlet edge must carry nothing — it \
                     carries {carried:.6e} kg/s. The far end of that edge is at {:.0} Pa, \
                     which is exactly the pressure a model reading the UPWIND end instead \
                     of the valve's own flange would have opened on",
                    pressures[&nid],
                    set_pressure.value(),
                    pressures[&other]
                );
            }
            if opening >= refinery_solvers::network::OPEN_EPS && sol.edge_mass_flow[&eid] < 0.0 {
                reverse_open += 1;
            }
        }
    }
    println!(
        "psv chains {psv_chains}/{SAMPLES}; openings shut {shut} / partial {partial} / full \
         {full}; spring-vs-upwind discriminating configurations {discriminating}; open psv \
         passing REVERSE flow {reverse_open}"
    );

    // ---- trees ------------------------------------------------------------
    let tree_strat = tree_inputs_strategy();
    let (mut spur_trees, mut relieving, mut floating_legs) = (0usize, 0usize, 0usize);
    let (mut newton_diverged, mut simple_ok) = (0usize, 0usize);
    let (mut tree_cycled, mut tree_capped) = (0usize, 0usize);
    let mut newton_hit_the_cap = 0usize;
    // Read from the solver rather than restated, so the split below cannot drift
    // out of step with the cap it is measuring against.
    let newton_max_iter = NewtonFlowSolver::default().max_iter;
    for _ in 0..SAMPLES {
        let mut inputs = tree_strat
            .new_tree(&mut runner)
            .expect("strategy produces a value")
            .current();
        let n_primary = inputs.0.len() + 1;
        if !inputs.5.iter().take(n_primary).any(|s| s.is_some()) {
            continue;
        }
        // STRIP THE LEAKS. This test measures the RELIEF arm's reachability, and
        // a leak can only interfere: on a gas plant it makes `prepare` refuse
        // outright (the incompressible-orifice door), and on a liquid one it
        // draws flow away from the spur and perturbs the very openings being
        // counted. Neither is a fact about the relief arm. The leak arm has its
        // own reachability test and measures itself there.
        inputs.6.iter_mut().for_each(|l| *l = None);
        spur_trees += 1;
        let g = build_tree(&inputs);
        let fluid = &inputs.4;

        // Which nodes the SOLVER treats as floating — read from `prepare`, the
        // very call both solvers make, rather than recomputed by this test.
        let prep = refinery_solvers::network::prepare(
            &g,
            &fluid.slate,
            &Default::default(),
            &Default::default(),
        )
        .expect("a generated tree compiles");
        floating_legs += g
            .node_ids()
            .filter(|nid| g.node(*nid).name.starts_with("deadleg") && !prep.anchored.contains(nid))
            .count();

        match NewtonFlowSolver::default().solve(&g, &fluid.slate, &Default::default(), Seconds(0.1))
        {
            Ok(sol) if sol.diagnostics.converged => {
                for nid in g.node_ids() {
                    if !g.node(nid).name.starts_with("psv") {
                        continue;
                    }
                    let Some((eid, other)) = outlet_of(&g, nid) else {
                        continue;
                    };
                    if g.node(other).name.starts_with("flare")
                        && sol.edge_mass_flow[&eid].abs() > 1e-9
                    {
                        relieving += 1;
                    }
                }
            }
            // Split the failures by MECHANISM rather than counting them together.
            // A singular Jacobian leaves a residual row and column identically
            // zero, so Newton gives up almost immediately; running out of
            // iterations on an ordinary stiff plant is a different event that
            // happens to produce the same `Err`. Worth separating because the
            // liquid/gas control in this file measures a worst case of 48 against
            // `max_iter = 50` — a two-iteration margin — so "diverged" cannot be
            // read as "hit the singular Jacobian" without checking. Since M8.0
            // the split is also what shows WHERE the fix landed: the fast
            // failures went from ~42 to ~9 while the cap-exhausting ones did not
            // move, which is the signature of a well-posedness fix rather than
            // of a solver that got faster.
            Err(SimError::SolverDiverged { iterations, .. }) => {
                newton_diverged += 1;
                if iterations >= newton_max_iter {
                    newton_hit_the_cap += 1;
                }
            }
            // NOT counted as a divergence: a plant whose anchoring will not
            // settle is a different event from a solve that could not find the
            // root of one, and folding them together is what would let the fix
            // look like it had merely moved the failures around.
            Err(SimError::AnchoringUnsettled { cycled, .. }) => {
                if cycled {
                    tree_cycled += 1;
                } else {
                    tree_capped += 1;
                }
            }
            _ => newton_diverged += 1,
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
        "spur trees {spur_trees}/{SAMPLES}; flare spurs RELIEVING {relieving}; dead legs FLOATING \
         at the seed {floating_legs}; newton diverged {newton_diverged}/{spur_trees}, of which \
         {newton_hit_the_cap} exhausted the {newton_max_iter}-iteration cap; simple converged \
         {simple_ok}/{spur_trees}; anchoring UNSETTLED on trees {tree_cycled} cycled + \
         {tree_capped} capped, on chains {chain_cycled} cycled + {chain_capped} capped \
         (of {psv_chains} relief-bearing chains)"
    );

    // (1) The arm is sampled at all.
    assert!(
        psv_chains * 4 >= SAMPLES,
        "only {psv_chains}/{SAMPLES} chains carried a relief valve"
    );
    // (2) All THREE branches of the characteristic are visited. `partial` is the
    // one that matters most — it is the only region where the smoothstep is not
    // a constant, so a linear ramp, a step, or a clamp bug can show only there.
    assert!(
        shut > 0 && partial > 0 && full > 0,
        "the relief characteristic is not fully exercised: shut {shut}, partial {partial}, full \
         {full} — a zero in any bucket means one branch of the smoothstep is untested"
    );
    assert!(
        partial * 10 >= shut + partial + full,
        "only {partial} of {} generated openings are PARTIAL lifts — the interior of the \
         smoothstep, the only part of it that is not a constant, is barely sampled",
        shut + partial + full
    );
    // (5) The mutation-discriminating configuration must be COMMON — this is the
    // reachability floor under the zero-flow GATE asserted above, not a statistic
    // beside it. A catch that rests on one lucky sample is not a catch, and a
    // floor with no assertion behind it was the hole this number used to have.
    //
    // It is therefore load-bearing in a way it was not before: the gate above is
    // the ONLY thing in the workspace that catches `pressures[&src]` → `upwind`
    // (measured: 1 gate with this arm, 0 without), and it runs only on the
    // samples this floor counts. A generator drift that halves the population
    // would disarm that catch rather than merely thin a statistic. Left at 10
    // against a measured 20 rather than raised to hug the measurement, so an
    // honest change in the generators is not fought by a tripwire — but the
    // consequence is written down here rather than left to be rediscovered.
    assert!(
        discriminating >= 10,
        "only {discriminating} generated configurations put a PSV's own flange below its set \
         pressure while the far end of its outlet edge sits above it — the one configuration \
         where sensing the flange and sensing the upwind end give different answers. The \
         shut-therefore-carrying-nothing gate above runs only on these, so at this rate the \
         arm cannot claim to pin WHICH pressure a PSV senses"
    );
    // (3) The load-bearing spur actually relieves.
    assert!(
        relieving >= 20,
        "only {relieving} flare spurs carried any relief flow — the spur arm has degenerated into \
         a dead end and the case it exists for is gone"
    );
    // (4) And the dead leg actually floats.
    assert!(
        floating_legs >= 20,
        "only {floating_legs} dead legs were floating — the floating-subnetwork path this slice \
         added is not being reached"
    );
    // These two numbers bounded a DEFERRAL until M8.0 and now measure a FIX, so
    // both are re-measured and tightened. Left at the deferral's loose bounds
    // they would have become counters with nothing behind them
    // ([[a-counter-is-not-a-gate]], which this file has already been caught by).
    //
    // Measured on the same generators, before and after the active-set loop:
    // Newton failed 74/305 and now fails 42/305, of which 33 exhausted the
    // iteration cap rather than hitting a singular Jacobian — so the fast,
    // structurally-broken failures went from ~42 to ~9. Simple converged 55/305
    // and now converges 57/305. The remaining `newton_diverged` is dominated by
    // ordinary stiffness, which is why the cap split is still reported: a rise
    // here means one of two different things and the split says which.
    //
    // Tightened again by M26.1 (docs/DESIGN.md §30), for the same reason. The
    // "ordinary stiffness" was mostly not: it was a PSV partly open with its
    // opening frozen into Newton's Jacobian, so every step overshot. Measured
    // immediately before M26.1: 39/305 failing, 28 at the cap. After: 5/305,
    // none at the cap. Left at `* 6` (50) the bound would pass with the fix
    // removed (39), which is the counter-with-nothing-behind-it this file keeps
    // being caught by; at `* 20` (15) removing it fails, and so does putting
    // the term in the wrong column (55).
    assert!(
        newton_diverged * 20 <= spur_trees,
        "Newton fails on {newton_diverged}/{spur_trees} spur trees ({newton_hit_the_cap} of \
         them by exhausting the {newton_max_iter}-iteration cap), against 5 (none at the cap) \
         measured when M26.1 gave the Jacobian a relief valve's opening slope, and 39 (28 at \
         the cap) before it. A rise in the cap share is that slope's signature (DESIGN §30); \
         a rise in the rest is the anchoring loop"
    );
    assert!(
        newton_hit_the_cap * 100 <= spur_trees,
        "{newton_hit_the_cap}/{spur_trees} spur trees exhausted Newton's \
         {newton_max_iter}-iteration cap, against none after M26.1 and 28 before it — a PSV \
         in its band whose opening slope is missing from the Jacobian crawls exactly like \
         this (DESIGN §30)"
    );
    assert!(
        simple_ok * 7 >= spur_trees,
        "Simple converged on only {simple_ok}/{spur_trees} spur trees, against 57 measured \
         when the active-set loop landed — `tree_fidelity_agreement` has no non-vacuity \
         guard of its own, so this is where a collapse would be seen"
    );
    // M8.0's new termination, CAPPED. Until M45 it was floored as well, for
    // reachability: measured, the generators reached the cycle 6 times and the
    // cap twice on trees, plus 2 cycles on chains, and a floor kept the refusal
    // paths exercised by a real solve rather than only by the stub-driven unit
    // tests.
    //
    // M45.0 (docs/DESIGN.md §50) took that floor's population away, and was
    // right to: every cycle these generators reached was a relief opening into a
    // stretch with one way in, which carries nothing whichever way the relief
    // stands — a dead end's tie, now answered (`dead_end_tie`). Measured on
    // today's generators: 4 cycled + 1 capped on trees and 1 cycled on chains
    // with the tie removed; 0 + 1 and 0 + 0 with it. So the bound is now a
    // CEILING on all four that the fix's removal fails, the M26.1 precedent
    // above. Reachability moved to fixed plants: the cycle by a real solve in
    // `dead_end_disc.rs`'s `a_dead_end_below_vacuum_is_refused` (until M46 also
    // in the two-reliefs chain, which M46 answers — unchanged counts here, 0 + 1
    // and 0 + 0 on both sides of it), the cap by its stub plus the one tree that
    // still reaches it here.
    let refused = tree_cycled + tree_capped + chain_cycled + chain_capped;
    assert!(
        refused <= 2,
        "the anchoring loop refused {tree_cycled} cycling + {tree_capped} capped spur trees \
         and {chain_cycled} + {chain_capped} chains, against 0 + 1 and 0 + 0 measured when \
         M45.0 answered a dead end's tie and 4 + 1 and 1 + 0 without it — a relief into a \
         blocked-in stretch carries nothing and is not chatter (DESIGN §50)"
    );
}

// ---------------------------------------------------------------------------
// THE ANCHORING ACTIVE SET (M8.0) — the two plants that used to be
// `known_defect_frozen_anchoring_*`, now asserting the fix, plus three gates on
// the loop's control flow itself.
//
// `network::prepare` derived the anchored set ONCE, from the seed compile. Every
// element before the PSV had a conductance that was constant for the whole
// solve, which made that exact; a PSV's conductance is a function of the
// pressure ITERATE, so the classification could be stale in either direction.
// `network::solve_with_active_anchoring` re-asks it between passes (DESIGN §3c).
//
// The first two tests were written as characterization tests, MEANT to fail when
// the defect was fixed, with their own docstrings saying what they should then
// assert. That is what they now assert, verbatim from those docstrings.
// ---------------------------------------------------------------------------

/// Seed-OPEN, converged-SHUT — the half that used to lose the whole solve on
/// both fidelities.
///
/// A(8 bar) —thin, long→ J —fat, short→ B(1 bar), with a blocked-in relief leg
/// off J. The cold seed is the mean of the fixed pressures, 4.5 bar, above the
/// 4.0 bar set — so at the seed the PSV conducts and the dead-leg terminal is
/// anchored. At the answer J sits near 1 bar and the PSV is shut, which under
/// the frozen set left the terminal's residual row and column identically zero
/// and the Jacobian singular.
///
/// What it must do now is what the old test's failure message specified: A feeds
/// B, the relief stays shut, and the sealed leg's pressure is indeterminate —
/// so it is parked, and its edges carry nothing.
#[test]
fn a_relief_that_shuts_on_the_way_to_the_answer_no_longer_defeats_the_solve() {
    let fluid = Fluid::liquid();
    let mut g = PlantGraph::new();
    let a = g.add_node(source(8.0e5, &fluid));
    let j = g.add_node(Node {
        name: "j".into(),
        kind: NodeKind::Junction,
        heat_input: Watt(0.0),
    });
    let b = g.add_node(sink(1.0e5, &fluid));
    let psv = g.add_node(Node {
        name: "psv".into(),
        kind: NodeKind::ReliefValve {
            cv_max: 1e-3,
            set_pressure: Pascal(4.0e5),
            accumulation: Pascal(0.5e5),
            x_t: None,
            blowdown: None,
        },
        heat_input: Watt(0.0),
    });
    let leg = g.add_node(Node {
        name: "deadleg".into(),
        kind: NodeKind::Junction,
        heat_input: Watt(0.0),
    });
    let e_aj = g.add_pipe(a, j, pipe((50.0, 0.05, 0.05, 0.0), "a_j", &fluid));
    let e_jb = g.add_pipe(j, b, pipe((1.0, 0.30, 0.01, 0.0), "j_b", &fluid));
    let e_jpsv = g.add_pipe(j, psv, pipe((5.0, 0.06, 0.02, 0.0), "j_psv", &fluid));
    let e_psvleg = g.add_pipe(psv, leg, pipe((5.0, 0.06, 0.02, 0.0), "psv_leg", &fluid));

    // The premise, asserted rather than assumed: the SEED really does classify
    // the terminal as anchored, so this is the seed-open/converged-shut case and
    // not some other plant that happens to solve.
    let prep = refinery_solvers::network::prepare(
        &g,
        &fluid.slate,
        &Default::default(),
        &Default::default(),
    )
    .expect("the plant compiles");
    approx::assert_relative_eq!(prep.pressures[&j], 4.5e5, max_relative = 1e-12);
    assert!(
        prep.anchored.contains(&leg),
        "premise failed: the dead leg must be ANCHORED at the seed for this to be the \
         seed-open/converged-shut case"
    );

    // Simple needs a raised sweep cap on this plant, and that is NOT this
    // defect. Measured: it converges in 7002 sweeps against a default of 5000 —
    // ordinary Gauss-Seidel stiffness on a fat/thin resistance ratio, which is
    // M5.4's FINDING 3 and is cured by sweeping longer. What the frozen
    // classification did was categorically different and no cap could cure it:
    // the leg's diagonal was exactly zero, so the node-wise step was non-finite
    // and the reported residual was `inf`. That contrast is asserted below.
    let mut simple = SimpleFlowSolver::default();
    simple.max_iter = 20_000;
    // Each fidelity's own convergence criterion, read off the solver rather than
    // restated, because the zeros asserted below are SOLVED zeros and the bar for
    // them is whatever that solver promised.
    let newton = NewtonFlowSolver::default();
    let tolerances = [
        (newton.tol_abs_kg_s, newton.tol_rel),
        (simple.tol_abs_kg_s, simple.tol_rel),
    ];
    for ((name, result), (tol_abs, tol_rel)) in [
        (
            "newton",
            NewtonFlowSolver::default().solve(&g, &fluid.slate, &Default::default(), Seconds(0.1)),
        ),
        (
            "simple",
            simple.solve(&g, &fluid.slate, &Default::default(), Seconds(0.1)),
        ),
    ]
    .into_iter()
    .zip(tolerances)
    {
        let sol = result.unwrap_or_else(|e| {
            panic!(
                "{name} failed a plant the active-set loop exists to solve — A feeds B, the \
                 relief stays shut, and only the CLASSIFICATION had to move: {e}"
            )
        });
        assert!(
            sol.diagnostics.converged,
            "{name} returned Ok but not converged"
        );

        // A feeds B, as one stream: the spur takes nothing, so the two live
        // edges carry the same flow. Asserted against each other rather than
        // against a hand-computed number — the magnitude is `newton_reference`'s
        // job, and what is at stake here is that the plant solves AT ALL.
        let (m_aj, m_jb) = (sol.edge_mass_flow[&e_aj], sol.edge_mass_flow[&e_jb]);
        assert!(m_aj > 0.0, "{name}: A must feed J, got {m_aj:.6e} kg/s");
        approx::assert_relative_eq!(m_aj, m_jb, max_relative = 1e-6);

        // The relief stays shut — and the spur's two edges are zero for two
        // DIFFERENT reasons, which is worth keeping apart because it sets two
        // different bars.
        //
        // The far edge touches a floating node, so it is inert: `edge_flows`
        // reports a structural zero and machine epsilon is the right bar.
        //
        // The near edge has both ends anchored, so it is live, and it carries
        // nothing only because the PSV behind it is a dead end — a fact the
        // SOLVE has to discover, to its own convergence criterion and no
        // further. Since M9.2 that criterion is the PSV node's OWN: the far edge
        // being inert leaves the node exactly one live edge, so the node's mass
        // residual IS this flow, and `grade_nodes` accepted it only if
        //
        //     |near| < tol_abs + tol_rel·|near|   ⟺   |near| < tol_abs/(1 − tol_rel)
        //
        // which is derived from the criterion rather than chosen. Before M9.2
        // the bar here was `tol_abs + tol_rel · max|ṁ|` over the WHOLE plant —
        // `1.04e-5` kg/s on the game fidelity, set by a trunk carrying 10 kg/s
        // that this spur has nothing to do with.
        //
        // **This tightening is a consistency edit and is measured NOT to
        // discriminate on this plant**, which is why it is recorded here rather
        // than counted as M9.2's gate: the accepted flow is `0.0` exactly on
        // Newton and `2.31e-10` on Simple, identical before and after the
        // change, so both cleared even the old, looser bar by a wide margin.
        // The measurement that does discriminate is
        // `a_valve_node_balances_against_its_own_flow_not_the_plants` in
        // `crates/scenarios/tests/control_reference.rs`, and it discriminates
        // because it watches the valve node while the valve is still
        // CONDUCTING. A shut branch is the easy case (M9.1's lesson, again).
        let bar = tol_abs / (1.0 - tol_rel);
        let near = sol.edge_mass_flow[&e_jpsv];
        assert!(
            near.abs() < bar,
            "{name}: the shut relief's inlet edge is the dead-end node's only live \
             edge, so its flow IS that node's mass imbalance and the solve promised \
             it under {bar:.3e} kg/s. It carries {near:.3e}"
        );
        approx::assert_relative_eq!(sol.edge_mass_flow[&e_psvleg], 0.0, epsilon = 1e-12);

        // And the answer really is on the SHUT side of the spring, which is what
        // makes the seed's classification wrong rather than merely different.
        let opening = refinery_solvers::elements::relief_opening(
            sol.node_pressure[&psv].value(),
            4.0e5,
            0.5e5,
        );
        assert!(
            opening <= refinery_solvers::network::OPEN_EPS,
            "{name}: the PSV must end SHUT for this to be the case it is named after \
             (opening {opening:.3e} at {:.0} Pa against a 4.0 bar set)",
            sol.node_pressure[&psv].value()
        );

        // The sealed leg's pressure is INDETERMINATE — a dead end behind a shut
        // valve has no pressure of its own — so it is parked at the floating
        // convention's value rather than invented.
        approx::assert_relative_eq!(
            sol.node_pressure[&leg].value(),
            P_ATM.value(),
            max_relative = 1e-12
        );
    }

    // The discriminating fact about Simple, separated from the sweep count it
    // happens to need. Whatever its default cap is, this plant must no longer be
    // able to produce an INFINITE residual: that value meant a zero diagonal —
    // an anchored node with no conducting edge left — and it is the signature of
    // the classification being stale rather than of a solve being slow. Written
    // to accept convergence too, so a future rise in the default cap improves
    // this test's subject rather than breaking it.
    match SimpleFlowSolver::default().solve(&g, &fluid.slate, &Default::default(), Seconds(0.1)) {
        Ok(sol) => assert!(sol.diagnostics.converged),
        Err(SimError::SolverDiverged { residual, .. }) => assert!(
            residual.is_finite(),
            "Simple reported an INFINITE residual, which is the zero-diagonal signature \
             of an anchored node whose edges all stopped conducting — the very state the \
             active-set loop exists to prevent"
        ),
        Err(other) => panic!("unexpected error from Simple: {other}"),
    }
}

/// Seed-SHUT, converged-OPEN — the half that used to converge with one
/// determinate pressure reported as atmospheric.
///
/// The same skeleton with the resistances swapped so J settles HIGH, and a set
/// pressure above the 4.5 bar seed. At the seed the PSV is shut and the terminal
/// has no conducting path to any anchor; at the answer the PSV is open, which
/// makes the terminal's pressure perfectly determinate — a dead end carries no
/// flow, so with no friction drop and no elevation it sits at exactly its
/// neighbour's. That is now what is reported.
#[test]
fn a_leg_behind_a_relief_that_opens_reports_its_neighbours_pressure() {
    let fluid = Fluid::liquid();
    let mut g = PlantGraph::new();
    let a = g.add_node(source(8.0e5, &fluid));
    let j = g.add_node(Node {
        name: "j".into(),
        kind: NodeKind::Junction,
        heat_input: Watt(0.0),
    });
    let b = g.add_node(sink(1.0e5, &fluid));
    let psv = g.add_node(Node {
        name: "psv".into(),
        kind: NodeKind::ReliefValve {
            cv_max: 1e-3,
            set_pressure: Pascal(5.0e5),
            accumulation: Pascal(0.5e5),
            x_t: None,
            blowdown: None,
        },
        heat_input: Watt(0.0),
    });
    let leg = g.add_node(Node {
        name: "deadleg".into(),
        kind: NodeKind::Junction,
        heat_input: Watt(0.0),
    });
    // Fat and short from A, thin and long to B ⇒ J settles near A's 8 bar.
    g.add_pipe(a, j, pipe((1.0, 0.30, 0.01, 0.0), "a_j", &fluid));
    g.add_pipe(j, b, pipe((50.0, 0.05, 0.05, 0.0), "j_b", &fluid));
    g.add_pipe(j, psv, pipe((5.0, 0.06, 0.02, 0.0), "j_psv", &fluid));
    // Level, so a dead end's correct pressure is exactly its neighbour's: with
    // no flow there is no friction drop, and with no elevation there is no head.
    let e_psvleg = g.add_pipe(psv, leg, pipe((5.0, 0.06, 0.02, 0.0), "psv_leg", &fluid));

    let prep = refinery_solvers::network::prepare(
        &g,
        &fluid.slate,
        &Default::default(),
        &Default::default(),
    )
    .expect("the plant compiles");
    assert!(
        !prep.anchored.contains(&leg),
        "premise failed: the dead leg must be FLOATING at the seed for this to be the \
         seed-shut/converged-open case"
    );

    let sol = NewtonFlowSolver::default()
        .solve(&g, &fluid.slate, &Default::default(), Seconds(0.1))
        .expect("this half always converged — it was the reported pressure that was wrong");
    assert!(sol.diagnostics.converged);

    let p_psv = sol.node_pressure[&psv].value();
    let opening = refinery_solvers::elements::relief_opening(p_psv, 5.0e5, 0.5e5);
    assert!(
        opening > refinery_solvers::network::OPEN_EPS,
        "premise failed: the PSV must end OPEN (inlet {p_psv:.0} Pa against a 5.0 bar set)"
    );
    // The leg carries no flow, which is correct and was never the defect — but
    // it is now zero for a DIFFERENT reason, and the tolerance has to follow.
    // While the leg floated, its edge was inert and reported a structural zero;
    // now that the leg is an anchored unknown, the zero is SOLVED — the node's
    // mass balance is driven to the solver's own convergence criterion and no
    // further. So the bar is that criterion, read off the solver rather than
    // guessed, not machine epsilon. Asserting 1e-12 here would be asserting
    // something the solve never promised.
    // Since M9.2 that criterion is the leg node's OWN, and this edge is its only
    // one, so the node's mass residual IS this flow and the bar it was graded
    // against collapses to `tol_abs/(1 - tol_rel)` - the same derivation as the
    // seed-open/converged-shut half above, and non-discriminating here for the
    // same measured reason.
    let solver = NewtonFlowSolver::default();
    let tol = solver.tol_abs_kg_s / (1.0 - solver.tol_rel);
    let carried = sol.edge_mass_flow[&e_psvleg];
    assert!(
        carried.abs() < tol,
        "the dead leg carries {carried:.3e} kg/s against the {tol:.3e} kg/s the solve \
         promised its own node - a dead end must carry nothing"
    );

    // THE FIX. Two assertions, because "equals its neighbour" and "is no longer
    // parked" are different claims and the second is the one that used to fail:
    // a leg parked at P_ATM would satisfy neither, but a leg that happened to
    // solve near atmospheric would satisfy the first alone.
    let p_leg = sol.node_pressure[&leg].value();
    approx::assert_relative_eq!(p_leg, p_psv, max_relative = 1e-6);
    assert!(
        (p_leg - P_ATM.value()).abs() > 1.0e5,
        "the leg is back at atmospheric ({p_leg:.0} Pa) behind an OPEN relief — the \
         classification has stopped following the answer"
    );
}

// --- the loop's own control flow, driven by a stub pass ---------------------
//
// These three run `solve_with_active_anchoring` directly with a `pass` that
// returns pressures of the test's choosing. That is what makes the loop's three
// exits separable at all: a real plant reaches whichever exit its physics
// reaches, so a gate built on one could not distinguish "settled" from "cycled"
// from "gave up", and two of those exits are error paths a plant in this repo
// may never take.

/// The stub pass: report success at whatever pressures the test dictates.
fn stub_pass(
    graph: &PlantGraph,
    fluid: &Fluid,
    prep: refinery_solvers::network::Prepared,
    dictate: impl Fn(&mut BTreeMap<NodeId, f64>),
) -> refinery_solvers::network::AnchorPass {
    let mut pressures = prep.pressures;
    dictate(&mut pressures);
    let edges = refinery_solvers::network::edge_flows(
        graph,
        &prep.compiled,
        &pressures,
        &prep.anchored,
        1.0,
    );
    let result = refinery_solvers::network::finalize(graph, &pressures, edges, 0, 0.0);
    let _ = fluid;
    refinery_solvers::network::AnchorPass { result, pressures }
}

/// A plant with nothing pressure-actuated in it runs EXACTLY ONE pass.
///
/// This is the mechanism behind the regression anchor, asserted rather than
/// argued: every scenario without a relief valve must be bit-identical to what
/// it was before the loop existed, and the reason is that its classification is
/// already a fixed point, so the loop returns pass one verbatim. Counting the
/// passes is the only way to see that from outside — the ANSWER looks the same
/// either way, which is exactly the point.
#[test]
fn an_ordinary_plant_runs_one_anchoring_pass() {
    let fluid = Fluid::liquid();
    let mut g = PlantGraph::new();
    let a = g.add_node(source(8.0e5, &fluid));
    let j = g.add_node(Node {
        name: "j".into(),
        kind: NodeKind::Junction,
        heat_input: Watt(0.0),
    });
    let b = g.add_node(sink(1.0e5, &fluid));
    g.add_pipe(a, j, pipe((10.0, 0.10, 0.02, 0.0), "a_j", &fluid));
    g.add_pipe(j, b, pipe((10.0, 0.10, 0.02, 0.0), "j_b", &fluid));

    let mut warm = BTreeMap::new();
    let mut passes = 0usize;
    let out = refinery_solvers::network::solve_with_active_anchoring(
        &g,
        &fluid.slate,
        &Default::default(),
        &mut warm,
        Seconds(1.0),
        |prep| {
            passes += 1;
            stub_pass(&g, &fluid, prep, |_| {})
        },
    );
    assert!(out.is_ok(), "an ordinary plant must solve: {:?}", out.err());
    assert_eq!(
        passes, 1,
        "a plant with no pressure-actuated element must settle on its FIRST \
         classification — if it re-passes, every pre-M8.0 golden is at risk"
    );
    // The warm start is committed by the driver, from the pass that was accepted.
    assert!(
        warm.contains_key(&j),
        "the driver must commit the warm start for a converged solve"
    );
}

/// A classification that ALTERNATES is caught as a cycle, not waited out.
///
/// The stub drives both reliefs open on the first pass and shut on the second,
/// so the second pass's recomputed set is one already seen. Physically this is a
/// relief whose own discharge re-seats it — chatter, which needs element state
/// and is deferred (DESIGN §3a) — and the honest answer is to say so rather than
/// return whichever of the two self-consistent states the parity landed on
/// ([[prove-the-exception-dont-skip-it]]).
///
/// **The stretch is a RELAY, not a dead leg** (M45.0, docs/DESIGN.md §50). Until
/// M45 this stub drove one relief into a blocked-in leg, and that is not
/// chatter: with one way into the leg nothing can flow whichever way the relief
/// stands, so the two classifications are one answer, and the loop now answers
/// it (`dead_end_tie`). Chatter needs a flow the alternation switches, so the
/// leg here leaves through a second relief to its own sink.
#[test]
fn an_alternating_classification_is_reported_as_a_cycle() {
    let fluid = Fluid::liquid();
    let (g, psv, relay) = relay_plant(&fluid, 5.0e5);

    let mut warm = BTreeMap::new();
    let mut passes = 0usize;
    let out = refinery_solvers::network::solve_with_active_anchoring(
        &g,
        &fluid.slate,
        &Default::default(),
        &mut warm,
        Seconds(1.0),
        |prep| {
            passes += 1;
            let lift = passes == 1;
            stub_pass(&g, &fluid, prep, |p| {
                // Above the set + accumulation band, or well below it — both
                // reliefs together, so the stretch has two ways across it.
                p.insert(psv, if lift { 6.0e5 } else { 2.0e5 });
                p.insert(relay, if lift { 6.0e5 } else { 2.0e5 });
            })
        },
    );
    let Err(SimError::AnchoringUnsettled { cycled, detail }) = out else {
        panic!("an alternating classification must be refused, got {out:?}");
    };
    assert!(
        cycled,
        "the cycle exit must be FLAGGED as a cycle rather than as a give-up — the two \
         are different plant states and only one of them names a deferral: {detail}"
    );
    assert!(
        detail.contains("psv"),
        "the diagnostic must name the element that is chattering: {detail}"
    );
    // Caught by REPETITION, so it stops at the repeat rather than burning the cap.
    assert_eq!(
        passes,
        2,
        "the cycle must be detected the moment a classification repeats, not after \
         {} passes",
        refinery_solvers::network::MAX_ANCHOR_PASSES
    );
    assert!(
        warm.is_empty(),
        "a solve that ends in Err must not leave a warm start behind"
    );
}

/// A classification that keeps producing NEW sets hits the cap, and says so in
/// different words from the cycle.
///
/// Four independent dead legs give sixteen possible classifications, and the
/// stub walks the low bits of the pass counter through eight of them without
/// ever repeating — so the cycle detector cannot fire and the only exit left is
/// the cap. This is the backstop, and without a stub it would be unreachable:
/// no plant in this repo is known to walk that many.
#[test]
fn a_classification_that_never_repeats_hits_the_cap() {
    let fluid = Fluid::liquid();
    let mut g = PlantGraph::new();
    let a = g.add_node(source(8.0e5, &fluid));
    let j = g.add_node(Node {
        name: "j".into(),
        kind: NodeKind::Junction,
        heat_input: Watt(0.0),
    });
    let b = g.add_node(sink(1.0e5, &fluid));
    g.add_pipe(a, j, pipe((10.0, 0.10, 0.02, 0.0), "a_j", &fluid));
    g.add_pipe(j, b, pipe((10.0, 0.10, 0.02, 0.0), "j_b", &fluid));
    // Four spurs, each with a set pressure ABOVE the 4.5 bar cold seed, so the
    // seed classification is "all shut" and the stub's first move is new.
    let mut psvs = Vec::new();
    for i in 0..4 {
        let psv = g.add_node(Node {
            name: format!("psv{i}"),
            kind: NodeKind::ReliefValve {
                cv_max: 1e-3,
                set_pressure: Pascal(6.0e5),
                accumulation: Pascal(0.5e5),
                x_t: None,
                blowdown: None,
            },
            heat_input: Watt(0.0),
        });
        let leg = g.add_node(Node {
            name: format!("deadleg{i}"),
            kind: NodeKind::Junction,
            heat_input: Watt(0.0),
        });
        g.add_pipe(j, psv, pipe((5.0, 0.06, 0.02, 0.0), "j_psv", &fluid));
        g.add_pipe(psv, leg, pipe((5.0, 0.06, 0.02, 0.0), "psv_leg", &fluid));
        psvs.push(psv);
    }

    let mut warm = BTreeMap::new();
    let mut passes = 0usize;
    let out = refinery_solvers::network::solve_with_active_anchoring(
        &g,
        &fluid.slate,
        &Default::default(),
        &mut warm,
        Seconds(1.0),
        |prep| {
            passes += 1;
            let bits = passes;
            let psvs = psvs.clone();
            stub_pass(&g, &fluid, prep, move |p| {
                for (i, &psv) in psvs.iter().enumerate() {
                    // Bit set ⇒ this relief lifts ⇒ its leg is anchored.
                    let lifted = bits & (1 << i) != 0;
                    p.insert(psv, if lifted { 8.0e5 } else { 2.0e5 });
                }
            })
        },
    );
    let Err(SimError::AnchoringUnsettled { cycled, detail }) = out else {
        panic!("a classification that never settles must be refused, got {out:?}");
    };
    assert!(
        !cycled,
        "the cap exit must be distinguishable from the cycle exit — a plant still \
         moving is not a plant alternating between two answers: {detail}"
    );
    assert_eq!(
        passes,
        refinery_solvers::network::MAX_ANCHOR_PASSES,
        "the cap must be what stops it, which means every pass must have been run"
    );
}

/// A pass that ends NON-FINITE is returned as-is, never reclassified.
///
/// The fourth exit from the loop, and the only one with no plant behind it: the
/// guard exists because `Simple` has a path that fails *because* its pressure
/// map went non-finite, and a classification derived from NaN is an artefact of
/// the NaN rather than an answer — an arbitrary retry is worse than an honest
/// failure (DESIGN §3c, fork 2b's guard).
///
/// It gets a stub for the same reason the cycle and the cap do, and one reason
/// more: it was found UNCOVERED by this slice's own mutation pass. Removing the
/// guard left the whole suite green, so nothing in this repo reached it. The
/// seed here classifies the relief OPEN (2 bar set against the 4.5 bar cold
/// seed), so the NaN is what shuts it — without the guard the loop would take
/// that flip seriously and re-pass under a set that NaN invented.
#[test]
fn a_pass_that_ends_non_finite_is_not_reclassified() {
    let fluid = Fluid::liquid();
    let (g, psv, _leg) = spur_plant(&fluid, 2.0e5);

    let mut warm = BTreeMap::new();
    let mut passes = 0usize;
    let out = refinery_solvers::network::solve_with_active_anchoring(
        &g,
        &fluid.slate,
        &Default::default(),
        &mut warm,
        Seconds(1.0),
        |prep| {
            passes += 1;
            stub_pass(&g, &fluid, prep, |p| {
                p.insert(psv, f64::NAN);
            })
        },
    );
    assert!(
        matches!(out, Err(SimError::NonFiniteState { .. })),
        "the pass's OWN failure must survive — replacing it with an anchoring \
         refusal would report the loop's confusion instead of the solver's: {out:?}"
    );
    assert_eq!(
        passes, 1,
        "a pass whose pressures went non-finite must not be reclassified: the set \
         a NaN implies is an artefact of the NaN, and re-passing under it spends \
         the cap turning an honest failure into a made-up one"
    );
    assert!(
        warm.is_empty(),
        "a failed pass must leave no warm start behind, non-finite least of all"
    );
}

/// A(8 bar) → J → B(1 bar) with one blocked-in relief spur off J, at the given
/// set pressure. Shared by the loop's control-flow gates, which care about the
/// classification rather than about any particular resistance.
fn spur_plant(fluid: &Fluid, set_pressure: f64) -> (PlantGraph, NodeId, NodeId) {
    let mut g = PlantGraph::new();
    let a = g.add_node(source(8.0e5, fluid));
    let j = g.add_node(Node {
        name: "j".into(),
        kind: NodeKind::Junction,
        heat_input: Watt(0.0),
    });
    let b = g.add_node(sink(1.0e5, fluid));
    let psv = g.add_node(Node {
        name: "psv".into(),
        kind: NodeKind::ReliefValve {
            cv_max: 1e-3,
            set_pressure: Pascal(set_pressure),
            accumulation: Pascal(0.5e5),
            x_t: None,
            blowdown: None,
        },
        heat_input: Watt(0.0),
    });
    let leg = g.add_node(Node {
        name: "deadleg".into(),
        kind: NodeKind::Junction,
        heat_input: Watt(0.0),
    });
    g.add_pipe(a, j, pipe((10.0, 0.10, 0.02, 0.0), "a_j", fluid));
    g.add_pipe(j, b, pipe((10.0, 0.10, 0.02, 0.0), "j_b", fluid));
    g.add_pipe(j, psv, pipe((5.0, 0.06, 0.02, 0.0), "j_psv", fluid));
    g.add_pipe(psv, leg, pipe((5.0, 0.06, 0.02, 0.0), "psv_leg", fluid));
    (g, psv, leg)
}

/// `spur_plant` with the leg carried on through a SECOND relief (`relay`, set at
/// 4 bar) to its own 1 bar sink: a stretch with two ways across it, so an
/// alternation of both reliefs switches a flow through it (M45.0). Both set
/// points sit above the 3.33 bar cold seed (the mean of the three pinned
/// pressures), so the stretch floats on the first classification. Returns the
/// graph, the first relief and the relay.
fn relay_plant(fluid: &Fluid, set_pressure: f64) -> (PlantGraph, NodeId, NodeId) {
    let (mut g, psv, leg) = spur_plant(fluid, set_pressure);
    let relay = g.add_node(Node {
        name: "relay".into(),
        kind: NodeKind::ReliefValve {
            cv_max: 1e-3,
            set_pressure: Pascal(4.0e5),
            accumulation: Pascal(0.5e5),
            x_t: None,
            blowdown: None,
        },
        heat_input: Watt(0.0),
    });
    let out = g.add_node(sink(1.0e5, fluid));
    g.add_pipe(leg, relay, pipe((5.0, 0.06, 0.02, 0.0), "leg_relay", fluid));
    g.add_pipe(relay, out, pipe((5.0, 0.06, 0.02, 0.0), "relay_out", fluid));
    (g, psv, relay)
}

/// **Two reliefs in series have one answer, and Newton gives it** (M46,
/// docs/DESIGN.md §51; was the pinned defect A20). A 6.99 bar source, a relief
/// set at 6.42 bar, one set at 3.23 bar, a 1 bar sink — the case
/// `chain_fidelity_agreement`'s generator found once M45.0 existed. Both reliefs
/// open, 68.354 kg/s, on both fidelities.
///
/// Newton's road to it: the first pass, everything anchored from the cold seed,
/// fails; the second, with the second relief's node floating, converges and
/// points straight back at the first's classification. Until M46 that repeat
/// was refused as chatter, though the classification it repeats had never
/// converged. It is now re-run once, from the second pass's answer, and
/// converges.
///
/// **It is also the one fixed plant that pins `dead_end_tie`'s re-check.** The
/// second relief's node is a stretch with ONE conducting edge in the compile its
/// filled classification came from (the second relief was shut there, parked
/// low), so it passes the count; filled to the first relief's pressure, the
/// second relief lifts, and only the re-check sees it. Without it Newton
/// "answers" zero flow through a chain the other solver runs at 68 kg/s.
#[test]
fn two_reliefs_in_series_answer_on_both_fidelities() {
    let fluid = Fluid::liquid();
    let (g, edges) = two_reliefs_in_series(&fluid);

    let newton = NewtonFlowSolver::default()
        .solve(&g, &fluid.slate, &Default::default(), Seconds(0.1))
        .expect("Newton answers the chain: the repeat it reaches is not chatter");
    let simple = SimpleFlowSolver::default()
        .solve(&g, &fluid.slate, &Default::default(), Seconds(0.1))
        .expect("the game solver answers the chain");
    for (name, solution) in [("newton", &newton), ("simple", &simple)] {
        for &e in &edges {
            let flow = solution.edge_mass_flow[&e];
            assert!(
                (flow - 68.354).abs() < 1e-3,
                "{name}: both reliefs open, one flow through the chain: {flow} kg/s"
            );
        }
    }
}

/// **A re-run that fails is refused as before, at once** (M46, docs/DESIGN.md
/// §51). Newton's road through `two_reliefs_in_series_answer_on_both_fidelities`,
/// scripted, with the re-run failing too: the filled pass fails, the floating
/// pass converges and points back at it, and the re-run — the third pass —
/// fails again. The refusal is the cycle the loop postponed, not a walk on to
/// the cap from whatever the failed re-run's iterate implies.
///
/// The guard on the other side — a repeat onto a classification that HAS
/// converged is refused without a re-run — is
/// `an_alternating_classification_is_reported_as_a_cycle`'s two-pass count.
#[test]
fn a_retry_that_fails_is_refused_as_the_cycle_it_postponed() {
    let fluid = Fluid::liquid();
    let (g, _edges) = two_reliefs_in_series(&fluid);
    let named = |name: &str| g.node_ids().find(|&n| g.node(n).name == name).unwrap();
    let (first, second) = (named("mid0"), named("mid1"));

    let mut warm = BTreeMap::new();
    let mut passes = 0usize;
    let out = refinery_solvers::network::solve_with_active_anchoring(
        &g,
        &fluid.slate,
        &Default::default(),
        &mut warm,
        Seconds(0.1),
        |prep| {
            passes += 1;
            let filled = prep.anchored.contains(&second);
            let mut pass = stub_pass(&g, &fluid, prep, |p| {
                if filled {
                    // Where Newton's cold pass gave up: both reliefs shut.
                    p.insert(first, 5.5e5);
                    p.insert(second, 2.5e5);
                } else {
                    // No flow past the first relief: it stands at the source.
                    p.insert(first, 698767.3083954572);
                }
            });
            if filled {
                pass.result = Err(SimError::SolverDiverged {
                    iterations: 2,
                    residual: 309.0,
                    residual_history: vec![437.0, 309.0],
                });
            }
            pass
        },
    );
    let Err(SimError::AnchoringUnsettled { cycled, detail }) = out else {
        panic!("a retry that fails must be refused, got {out:?}");
    };
    assert!(cycled, "refused as the cycle it postponed: {detail}");
    assert!(
        detail.contains("mid1"),
        "the diagnostic names the second relief: {detail}"
    );
    assert_eq!(
        passes, 3,
        "filled (failed), floating (converged), ONE retry (failed) — then refused"
    );
    assert!(
        warm.is_empty(),
        "a solve that ends in Err must not leave a warm start behind"
    );
}

/// The A20 chain: a 6.99 bar source, a relief set at 6.42 bar, one set at
/// 3.23 bar, a 1 bar sink; the reliefs are `mid0` and `mid1`.
fn two_reliefs_in_series(fluid: &Fluid) -> (PlantGraph, Vec<refinery_core::graph::EdgeId>) {
    let mids = [
        Mid::Relief {
            cv: 0.002297131857065713,
            set: 642073.520100982,
            band: 20000.0,
        },
        Mid::Relief {
            cv: 0.0001,
            set: 322711.0173797172,
            band: 20000.0,
        },
    ];
    let pipes = [
        (26.85771911202901, 0.16941785515279675, 0.01, 0.0),
        (1.0, 0.25118774211238937, 0.01, 0.0),
        (1.0, 0.05, 0.01, 0.0),
    ];
    build_chain(&mids, &pipes, 698767.3083954572, 1.0e5, fluid)
}

/// **A dead end's tie in gas stands on an exact root** (docs/DEFERRED.md A22,
/// found by M46's mutation runs; the head fixed in M47). Gas, a 5.90 bar
/// source, a relief set at 5.25 bar, a junction down a 3.8 m drop, a relief set
/// at 6.85 bar, a 7.67 bar sink: the drive is BACKWARDS.
///
/// Two answers stand, and both are kept. The second relief senses only its own
/// inlet: held shut, the stretch between the reliefs fills from the first
/// relief to 5.90 bar, under its set, so it stays shut — Newton's answer,
/// through §50's dead-end tie, zero flow everywhere. Held open, the sink pushes
/// the stretch to 7.48 bar, over its set, so it stays open — the game solver's,
/// 0.2708 kg/s backwards. That is the reverse-flow multiplicity this file's
/// header already allows. Since M48.1 (docs/DESIGN.md §53) a relief keeps the
/// state it last stood in, shut on the first solve, so both solvers now answer
/// zero flow here; the game solver's open root is still an exact root, and
/// `a_relief_stays_as_it_was_where_a22_has_two_answers` reaches it by history.
///
/// What was wrong was Newton's: the stood stretch read the 3.8 m drop's static
/// head from the compile its PARKED pass came from, and in gas that head moves
/// with the pressure. Recompiled where it stood, the drop was 0.043 Pa off and
/// carried 3.0e-5 kg/s against a throughput of zero, so the agreement gate's
/// root proof failed and any random run that drew this chain failed with it.
#[test]
fn a_dead_end_tie_in_gas_stands_on_an_exact_root() {
    let fluid = a22_fluid();
    let (g, edges) = a22_chain(A22_SOURCE, A22_SINK, &fluid);
    let newton = NewtonFlowSolver::default()
        .solve(&g, &fluid.slate, &Default::default(), Seconds(0.1))
        .expect("Newton answers through the dead end's tie");
    let simple = SimpleFlowSolver::default()
        .solve(&g, &fluid.slate, &Default::default(), Seconds(0.1))
        .expect("the game solver answers");
    // Tick 1 remembers every relief shut (M48.1, docs/DESIGN.md §53): the game
    // solver's own first answer — the second relief held open, 0.2708 kg/s
    // backwards — is an exact root too, and it is set aside for the held-shut
    // root its history picks.
    for &e in &edges {
        assert_eq!(
            newton.edge_mass_flow[&e], 0.0,
            "Newton: the second relief held shut, nothing flows — if this moved, \
             the tie changed (update A22 and this gate)"
        );
        assert!(
            simple.edge_mass_flow[&e].abs() < 1e-9,
            "game solver: started shut, it stays shut: {}",
            simple.edge_mass_flow[&e]
        );
    }
    // The agreement gate's own root bound, at Newton's throughput of zero.
    let (imbalance, throughput) =
        worst_recomputed_imbalance(&g, &fluid, &pressures_of(&newton)).expect("compiles");
    assert!(
        imbalance <= 1e-7 + 1e-5 * throughput,
        "Newton's stood stretch is a root where it stands: 3.0e-5 kg/s on the          parked pass's gas head, read {imbalance:.3e}"
    );
    if let Err(e) = assert_fidelity_agreement(&g, &fluid, Ok(newton), Ok(simple), true) {
        panic!("the random chain arm accepts this pair: {e}");
    }
}

/// A22's slate and chain (M46's mutation runs drew it; M47, M48.1).
fn a22_fluid() -> Fluid {
    Fluid::gas(0.1412898282423541, 0.5111970104338283)
}

const A22_SOURCE: f64 = 589569.6291973268;
const A22_SINK: f64 = 766611.3079391625;

/// The chain at the given boundary pressures [Pa]. Built in the same order every
/// time, so its `NodeId`s match across calls and a solver's warm start carries.
fn a22_chain(
    p_src: f64,
    p_snk: f64,
    fluid: &Fluid,
) -> (PlantGraph, Vec<refinery_core::graph::EdgeId>) {
    let mids = [
        Mid::Relief {
            cv: 0.0005465476441746615,
            set: 525151.5155196126,
            band: 20000.0,
        },
        Mid::Junction,
        Mid::Relief {
            cv: 0.0030206220252378935,
            set: 684889.851198137,
            band: 20000.0,
        },
    ];
    let pipes = [
        (13.269368171285288, 0.15471275506689028, 0.01, 0.0),
        (
            1.0,
            0.2143683747163805,
            0.03700775446932792,
            -3.8175495888055213,
        ),
        (1.0, 0.05, 0.01, 0.0),
        (44.52142561490201, 0.05, 0.01, 0.0),
        (1.0, 0.05, 0.01, 0.0),
        (1.0, 0.05, 0.01, 0.0),
    ];
    build_chain(&mids, &pipes, p_src, p_snk, fluid)
}

/// **A relief stays as it was where A22 has two answers** (M48.1, docs/DESIGN.md
/// §53), on both fidelities. Five solves at a first pair of boundary pressures,
/// then five at A22's, on the same solver: after the chain ran forward through
/// both reliefs (source 9 bar), the second stays open and A22 flows 0.2708 kg/s
/// backwards; after everything sat shut (source 4 bar, sink 1 bar), it stays
/// shut and nothing flows. Before M48.1 Newton returned to zero flow after
/// running open, and the game solver's cold answer was the open one.
#[test]
fn a_relief_stays_as_it_was_where_a22_has_two_answers() {
    let fluid = a22_fluid();
    for (history, first, open) in [
        ("ran open", (9.0e5, A22_SINK), true),
        ("sat shut", (4.0e5, 1.0e5), false),
    ] {
        let solvers: [(&str, Box<dyn FlowSolver>); 2] = [
            ("newton", Box::new(NewtonFlowSolver::default())),
            ("simple", Box::new(SimpleFlowSolver::default())),
        ];
        for (name, mut solver) in solvers {
            let (g, _) = a22_chain(first.0, first.1, &fluid);
            for _ in 0..5 {
                solver
                    .solve(&g, &fluid.slate, &Default::default(), Seconds(0.1))
                    .unwrap_or_else(|e| panic!("{name}, {history}: {e}"));
            }
            let (g, edges) = a22_chain(A22_SOURCE, A22_SINK, &fluid);
            for tick in 0..5 {
                let flow = solver
                    .solve(&g, &fluid.slate, &Default::default(), Seconds(0.1))
                    .unwrap_or_else(|e| panic!("{name}, {history}, A22 {tick}: {e}"))
                    .edge_mass_flow[&edges[0]];
                if open {
                    assert!(
                        (flow + 0.2708).abs() < 1e-3,
                        "{name}, {history}, A22 tick {tick}: held open, 0.2708 kg/s \
                         backwards, read {flow}"
                    );
                } else {
                    assert!(
                        flow.abs() < 1e-9,
                        "{name}, {history}, A22 tick {tick}: held shut, read {flow}"
                    );
                }
            }
        }
    }
}

// --- the second active set: starved tanks (M24, DESIGN §28 fork 3) -----------
//
// The same stub-pass pattern as the anchoring exits above, for the same reason:
// a real plant reaches whichever exit its physics reaches. A single tank's
// in-tick recovery can only be the rounding boundary (every solve starts
// all-wet, and a starved tank supplies less than the wet pass drew, so its
// pressure falls), and the union exit needs two tanks moving against each other.
// Neither is something a scenario can be steered into.

/// A water tank of 1 m² holding `mass` kg: its bottom pressure is
/// `P_ATM + g·m` Pa.
fn tank_node(name: &str, mass: f64, fluid: &Fluid) -> Node {
    Node {
        name: name.into(),
        kind: NodeKind::Tank(refinery_core::graph::TankState {
            area: SquareMeter(1.0),
            height: Meter(10.0),
            mass: Kg(mass),
            temperature: T_AMBIENT,
            composition: fluid.composition.clone(),
            ambient_ua: WattPerKelvin::ZERO,
        }),
        heat_input: Watt(0.0),
    }
}

fn wet_pressure(g: &PlantGraph, fluid: &Fluid, tank: NodeId) -> f64 {
    match &g.node(tank).kind {
        NodeKind::Tank(t) => t.bottom_pressure(&fluid.slate).value(),
        _ => unreachable!(),
    }
}

/// Tanks `names` (with their masses) → one junction `j` → a 1 bar sink. The
/// stub dictates `j`, and each starved tank, per pass.
fn tanks_to_junction(fluid: &Fluid, tanks: &[(&str, f64)]) -> (PlantGraph, Vec<NodeId>, NodeId) {
    let mut g = PlantGraph::new();
    let ids: Vec<NodeId> = tanks
        .iter()
        .map(|(name, mass)| g.add_node(tank_node(name, *mass, fluid)))
        .collect();
    let j = g.add_node(Node {
        name: "j".into(),
        kind: NodeKind::Junction,
        heat_input: Watt(0.0),
    });
    let b = g.add_node(sink(1.0e5, fluid));
    for &t in &ids {
        g.add_pipe(t, j, pipe((10.0, 0.10, 0.02, 0.0), "t_j", fluid));
    }
    g.add_pipe(j, b, pipe((10.0, 0.10, 0.02, 0.0), "j_b", fluid));
    (g, ids, j)
}

/// Net pressure-driven outflow of `node` in a solution [kg/s].
fn net_out(g: &PlantGraph, sol: &HydraulicSolution, node: NodeId) -> f64 {
    g.incident(node)
        .into_iter()
        .map(|(e, _, incoming)| {
            let f = sol.edge_mass_flow[&e];
            if incoming {
                -f
            } else {
                f
            }
        })
        .sum()
}

/// A tank the wet pass draws past empty is re-solved STARVED, and that pass is
/// accepted: its report carries the supply `m/dt` (one expression, shared with
/// the classification) and the tank's own residual, and the warm start is
/// committed from IT — the starved pass is the one the tank is free in.
#[test]
fn a_tank_drawn_past_empty_is_starved_and_accepted() {
    let fluid = Fluid::liquid();
    let (g, tanks, j) = tanks_to_junction(&fluid, &[("t", 1.0)]);
    let t = tanks[0];
    let wet = wet_pressure(&g, &fluid, t);
    let mut warm = BTreeMap::new();
    let mut passes = 0usize;
    let out = refinery_solvers::network::solve_with_active_anchoring(
        &g,
        &fluid.slate,
        &Default::default(),
        &mut warm,
        Seconds(0.5),
        |prep| {
            passes += 1;
            let starving = passes > 1;
            stub_pass(&g, &fluid, prep, |p| {
                // Pass 1 pulls the junction far below the tank; pass 2 finds the
                // starved tank below its wet pressure, as a real starved solve
                // does.
                p.insert(j, 0.5e5);
                if starving {
                    p.insert(t, wet - 1.0e3);
                }
            })
        },
    );
    let sol = out.expect("a starved solve is an answer");
    assert_eq!(
        passes, 2,
        "one wet pass finds the overdraw, one starved pass is kept"
    );
    let report = sol.starved.get(&t).expect("the tank is reported starved");
    assert_eq!(
        report.supply.value(),
        1.0 / 0.5,
        "the supply is m/dt, dt = 0.5"
    );
    assert_eq!(
        report.residual.value(),
        report.supply.value() - net_out(&g, &sol, t),
        "the residual is the tank's OWN: supply minus its solved net outflow"
    );
    assert_eq!(
        warm.get(&t),
        Some(&(wet - 1.0e3)),
        "the warm start is committed from the accepted (starved) pass"
    );
}

/// A repeat that differs only in the starved set is ACCEPTED as the starved
/// pass (§28 fork 3). The starved pass lands above the wet pressure — the
/// boundary inside the solver's tolerance — so it proposes going wet again,
/// which is the classification pass 1 already ran. Refusing it would fail
/// `tank_overfill_trip` past tick 12 000; accepting the WET pass would create
/// the overdraw the loop exists to remove.
#[test]
fn a_starvation_only_repeat_is_accepted_as_the_starved_pass() {
    let fluid = Fluid::liquid();
    let (g, tanks, j) = tanks_to_junction(&fluid, &[("t", 1.0)]);
    let t = tanks[0];
    let wet = wet_pressure(&g, &fluid, t);
    let mut warm = BTreeMap::new();
    let mut passes = 0usize;
    let out = refinery_solvers::network::solve_with_active_anchoring(
        &g,
        &fluid.slate,
        &Default::default(),
        &mut warm,
        Seconds(1.0),
        |prep| {
            passes += 1;
            let starving = passes > 1;
            stub_pass(&g, &fluid, prep, |p| {
                p.insert(j, 0.5e5);
                if starving {
                    p.insert(t, wet + 1.0);
                }
            })
        },
    );
    let sol = out.expect("a starvation-only repeat is accepted, not refused");
    assert_eq!(passes, 2, "the repeat is caught the moment it surfaces");
    assert!(
        sol.starved.contains_key(&t),
        "the kept pass is the STARVED one"
    );
}

/// A starved tank RECOVERS inside a solve when another tank's starving changes
/// what the network does to it. Two tanks: pass 1 over-draws only `a`; pass 2
/// (a starved) pushes `a` above its wet pressure and over-draws `c`; pass 3 (c
/// starved) is settled. The kept set is `{c}` — never recovering would keep
/// `{a, c}` and starve a tank the network is pushing into.
#[test]
fn a_starved_tank_recovers_inside_a_solve() {
    let fluid = Fluid::liquid();
    let (g, tanks, j) = tanks_to_junction(&fluid, &[("a", 1.0), ("c", 1.0e3)]);
    let (a, c) = (tanks[0], tanks[1]);
    let (wet_a, wet_c) = (wet_pressure(&g, &fluid, a), wet_pressure(&g, &fluid, c));
    let mut warm = BTreeMap::new();
    let mut passes = 0usize;
    let out = refinery_solvers::network::solve_with_active_anchoring(
        &g,
        &fluid.slate,
        &Default::default(),
        &mut warm,
        Seconds(1.0),
        |prep| {
            passes += 1;
            let pass = passes;
            stub_pass(&g, &fluid, prep, move |p| match pass {
                // Only `a` (1 kg) is over-drawn: `c` holds a tonne, and ~60 kg/s
                // leaves it at this pull.
                1 => {
                    p.insert(j, wet_a - 5.0e4);
                }
                // `a` is pushed into; `c` is drawn at 1e9 Pa of head — some
                // 8 000 kg/s, far past its tonne over one tick.
                2 => {
                    p.insert(a, wet_a + 1.0e5);
                    p.insert(j, -1.0e9);
                }
                // `c` starved and below its wet pressure; `a` barely drawn.
                _ => {
                    p.insert(c, wet_c - 1.0e3);
                    p.insert(j, wet_a - 1.0e-3);
                }
            })
        },
    );
    let sol = out.expect("the solve settles");
    assert_eq!(passes, 3);
    assert_eq!(
        sol.starved.keys().copied().collect::<Vec<_>>(),
        vec![c],
        "a recovered and c starved"
    );
}

/// A starvation-only repeat that surfaces on the LESS starved of its two passes
/// runs the union once more with recovery frozen, rather than keeping the pass
/// that would over-draw. Pass 1 over-draws both; pass 2 ({a, c}) pushes `a` up
/// so it would recover; pass 3 ({c}) over-draws `a` again, proposing {a, c},
/// already run — and {c} does not contain it. Keeping pass 3 would let `a`
/// over-draw; pass 4 runs {a, c}, where `a` may not recover, and is kept.
#[test]
fn a_repeat_on_the_less_starved_pass_runs_the_union() {
    let fluid = Fluid::liquid();
    let (g, tanks, j) = tanks_to_junction(&fluid, &[("a", 1.0), ("c", 1.0)]);
    let (a, c) = (tanks[0], tanks[1]);
    let wet_a = wet_pressure(&g, &fluid, a);
    let wet_c = wet_pressure(&g, &fluid, c);
    let mut warm = BTreeMap::new();
    let mut passes = 0usize;
    let out = refinery_solvers::network::solve_with_active_anchoring(
        &g,
        &fluid.slate,
        &Default::default(),
        &mut warm,
        Seconds(1.0),
        |prep| {
            passes += 1;
            let pass = passes;
            stub_pass(&g, &fluid, prep, move |p| match pass {
                1 | 3 => {
                    p.insert(j, 0.5e5);
                    if pass == 3 {
                        p.insert(c, wet_c - 1.0e3);
                    }
                }
                _ => {
                    p.insert(a, wet_a + 1.0e5);
                    p.insert(c, wet_c - 1.0e3);
                    p.insert(j, 0.5e5);
                }
            })
        },
    );
    let sol = out.expect("the union is an answer");
    assert_eq!(passes, 4, "the union costs exactly one more pass");
    assert_eq!(
        sol.starved.keys().copied().collect::<Vec<_>>(),
        vec![a, c],
        "the kept pass is the union, where neither tank can over-draw"
    );
}

/// **Gate 10: a starved tank is not an anchor.** A tank whose only neighbour is
/// a dead-end junction is the plant's only pressure reference while it is wet.
/// Starved, its equation is a constant supply and says nothing about its own
/// pressure, so the subnetwork FLOATS: every edge carries zero and the tank
/// keeps everything it held (its residual is its whole supply). Counting it as
/// an anchor would ask a real solver for a supply with nowhere to go — and here
/// would carry the wet pass's pull straight into the answer.
#[test]
fn a_starved_tank_with_no_other_reference_floats() {
    let fluid = Fluid::liquid();
    let mut g = PlantGraph::new();
    let t = g.add_node(tank_node("t", 1.0, &fluid));
    let j = g.add_node(Node {
        name: "j".into(),
        kind: NodeKind::Junction,
        heat_input: Watt(0.0),
    });
    g.add_pipe(t, j, pipe((10.0, 0.10, 0.02, 0.0), "t_j", &fluid));

    // The classification itself, on the real code: starved, `t` is free and
    // not among the anchors.
    let supplies: BTreeMap<NodeId, f64> = [(t, 1.0)].into_iter().collect();
    let classes = refinery_solvers::network::classify(&g, &fluid.slate, &supplies);
    assert!(classes.free.contains(&t), "a starved tank is a free node");
    assert!(
        refinery_solvers::network::base_anchors(&classes).is_empty(),
        "a starved tank is not an anchor"
    );

    let mut warm = BTreeMap::new();
    let mut passes = 0usize;
    let out = refinery_solvers::network::solve_with_active_anchoring(
        &g,
        &fluid.slate,
        &Default::default(),
        &mut warm,
        Seconds(1.0),
        |prep| {
            passes += 1;
            let wet = passes == 1;
            stub_pass(&g, &fluid, prep, |p| {
                // The wet pass pulls the junction down (a stub's licence: a real
                // dead end carries nothing); the starved pass dictates nothing.
                if wet {
                    p.insert(j, 0.5e5);
                }
            })
        },
    );
    let sol = out.expect("a floating starved tank is an answer, not a failure");
    assert_eq!(passes, 2);
    assert!(
        sol.edge_mass_flow.values().all(|&f| f == 0.0),
        "a floating subnetwork carries nothing: {:?}",
        sol.edge_mass_flow
    );
    let report = sol.starved[&t];
    assert_eq!(
        report.residual, report.supply,
        "delivering nothing, the tank keeps its whole supply"
    );
}

// ---------------------------------------------------------------------------
// Non-vacuity for the LEAK arm (M6.1) — and the extra question a leak forces
// that the gas and relief arms did not.
//
// A leak can be generated and still test nothing, in the ways those arms
// already record: never generated (a weight typo), or generated so small that
// its orifice is effectively closed and every balance passes on a plant that is
// hydraulically intact. It adds a third: a leak has two REFUSAL paths, and if
// they consumed every sample the arm would look busy while never once solving a
// plant with a hole in it.
//
// What makes this test a gate rather than a tally is `discriminating`. Counting
// leaks that conduct proves the GENERATOR works; it does not prove the balance
// assertion in `tree_conserves_or_diverges` would notice a broken one. So each
// conducting leak is checked against the exact tolerance that gate uses, twice:
// the junction must balance WITH the leak edge counted, and must FAIL to balance
// without it. A sample passing both is one where dropping the leak from
// `edge_flows` — the whole failure mode M6.0 found the old `leak_area` in —
// would turn `tree_conserves_or_diverges` red.
// ---------------------------------------------------------------------------

#[test]
fn the_leak_arm_conducts_and_is_refused_both_ways() {
    const SAMPLES: usize = 400;
    let mut runner = TestRunner::deterministic();
    let strat = tree_inputs_strategy();

    let (mut leaky_trees, mut back_fed) = (0usize, 0usize);
    // The gas half (M37): holes on gas plants that discriminate, and how many of
    // those vent CHOKED — the regime M6 refused for.
    let (mut gas_discriminating, mut gas_choked) = (0usize, 0usize);
    let (mut diverged, mut solved) = (0usize, 0usize);
    let (mut leaks_seen, mut conducting, mut discriminating) = (0usize, 0usize, 0usize);

    for _ in 0..SAMPLES {
        let inputs = strat
            .new_tree(&mut runner)
            .expect("strategy produces a value")
            .current();
        let g = build_tree(&inputs);
        let fluid = &inputs.4;
        let holes: Vec<_> = g
            .edge_ids()
            .filter(|e| matches!(g.pipe(*e).leak, LeakRole::Orifice { .. }))
            .collect();
        if holes.is_empty() {
            continue;
        }
        leaky_trees += 1;

        match NewtonFlowSolver::default().solve(&g, &fluid.slate, &Default::default(), Seconds(0.1))
        {
            Err(SimError::Numerical(m)) if m.contains("back-feeds") => back_fed += 1,
            Ok(sol) if sol.diagnostics.converged => {
                solved += 1;
                let throughput = sol
                    .edge_mass_flow
                    .values()
                    .fold(0.0f64, |m, &f| m.max(f.abs()));
                // The very tolerance `tree_conserves_or_diverges` applies. Read
                // from one place so this test cannot certify a discrimination
                // the gate would not actually make.
                let tol = 1e-5 + 1e-6 * throughput;
                for hole in holes {
                    leaks_seen += 1;
                    let flow = sol.edge_mass_flow[&hole];
                    if flow.abs() <= 0.01 * throughput || flow.abs() <= 1e-6 {
                        continue; // a trickle: generated, but nothing to see
                    }
                    conducting += 1;
                    // The junction the hole hangs off. The leak edge leaves it,
                    // so `node_imbalance` books it as `−flow`; dropping the edge
                    // adds that back.
                    let (junction, _) = g.endpoints(hole);
                    let with = node_imbalance(&g, &sol, junction);
                    let without = with + flow;
                    if with.abs() <= tol && without.abs() > tol {
                        discriminating += 1;
                        if fluid.is_gas() {
                            gas_discriminating += 1;
                            // The hole vents from its junction: its drop ratio
                            // against the choke for this gas.
                            let p = sol.node_pressure[&junction].value();
                            let x = (p - P_ATM.value()) / p;
                            let comp = &g.pipe(hole).stream.composition;
                            let gamma = comp.mixture_cp(&fluid.slate).value()
                                / comp.mixture_cv(&fluid.slate).value();
                            if flow > 0.0 && x > isentropic_critical_drop_ratio(gamma) {
                                gas_choked += 1;
                            }
                        }
                    }
                }
            }
            _ => diverged += 1,
        }
    }

    println!(
        "leaky trees {leaky_trees}/{SAMPLES}: solved {solved}, refused back-feed {back_fed}, \
         diverged {diverged}; of {leaks_seen} holes on solved plants {conducting} conduct >1% \
         of throughput, {discriminating} of them decisively (the balance gate fails without the \
         leak edge), {gas_discriminating} of those in gas and {gas_choked} of those choked"
    );

    // (1) The arm is sampled at all.
    assert!(
        leaky_trees * 4 >= SAMPLES,
        "only {leaky_trees}/{SAMPLES} trees carried a leak"
    );
    // (2) The refusals do not eat the arm. A leak that is ALWAYS refused is a
    // leak the mass balance never sees, which is precisely the state M6.0 found
    // the feature in — present, reachable-looking, and never exercised.
    //
    // Set at a quarter. It was measured at 36% while every gas leak was refused
    // (a ceiling of ~50%); since M37 sizes a gas hole, it is 143 of 152. The
    // floor was not raised with it: a quarter still catches the arm hollowing
    // out, and the gas half has its own floors below.
    assert!(
        solved * 4 >= leaky_trees,
        "only {solved}/{leaky_trees} leaky trees actually SOLVED ({back_fed} refused for \
         back-feed, {diverged} diverged) — the arm is generating holes nothing ever flows \
         through"
    );
    // (3) The GAS hole reaches the balance gate, and does so CHOKED (M37,
    // docs/DESIGN.md §41). Until M37 every one of these was refused by
    // `compile_edge`'s second door; this is that door's replacement — the
    // generated population is where a gas hole whose mass went unbooked, or a
    // choked branch the solvers could not carry, would show. Measured 93 gas
    // holes that discriminate and 78 of them choked (of 400 samples); floored at
    // about half of each, for the reason (2) gives. One gas tree in the measured
    // population diverges with its hole open and solves with it sealed (one
    // liquid tree does the same) — a legal outcome under I3, recorded against
    // docs/DEFERRED.md A18 rather than floored on.
    assert!(
        gas_discriminating >= 45 && gas_choked >= 40,
        "only {gas_discriminating} gas holes reach the balance gate decisively and \
         {gas_choked} of them choked — the gas half of the leak arm has hollowed out"
    );
    // `back_fed` is REPORTED and deliberately not floored. It is 3 of 400 — the
    // generator does reach a junction below `P_ATM` (a relief spur's flare may
    // sit at 0.9e5, under atmospheric, and pull one there), so the path is
    // demonstrably live, but a floor on a number that small would be a tripwire
    // on luck rather than a gate on behaviour. What actually pins that refusal is
    // `leak_reference::a_leak_below_atmospheric_is_refused`, a hand-built plant
    // whose junction pressure is MEASURED sub-atmospheric before the leak is
    // opened. This line exists so a reader knows the generated population is not
    // where that claim rests.
    //
    // (4) THE GATE, not a statistic. Each of these samples is one where the
    // balance assertion in `tree_conserves_or_diverges` holds with the leak's
    // flow counted and BREAKS without it — so at this rate that assertion really
    // is what stands between the repo and a leak edge whose mass goes unbooked.
    assert!(
        discriminating >= 20,
        "only {discriminating} generated leaks are large enough that the mass-balance gate \
         would notice them going unbooked (of {conducting} that conduct at all). Below this \
         the leak arm proves the GENERATOR works and proves nothing about the gate"
    );
}

// ---------------------------------------------------------------------------
// The CHECK VALVE arm (M30, docs/DESIGN.md §33).
//
// Its own test rather than a new `Mid` variant: widening `mid_strategy` would
// reshuffle every chain the existing gates draw and move their recorded
// reachability counts. The GAS half (M31, docs/DESIGN.md §34) is a second test
// over the same chain builder, for the same reason: the liquid test's draws
// and counts stay exactly as M30 recorded them.
// ---------------------------------------------------------------------------

/// One generated disc chain: `Source → before* → DISC → after* → Sink`, the end
/// pressures drawn independently so the drive across the disc runs backwards
/// about as often as forwards, and a pump among the mids can push either way.
#[allow(clippy::type_complexity)]
fn disc_chain_strategy() -> impl Strategy<
    Value = (
        Vec<Mid>,
        Vec<Mid>,
        (f64, f64),
        Vec<(f64, f64, f64, f64)>,
        f64,
        f64,
    ),
> {
    (
        prop::collection::vec(mid_strategy(), 0..3usize),
        prop::collection::vec(mid_strategy(), 0..3usize),
        (1e-4..5e-3f64, 0.1e5..1.0e5f64),
        prop::collection::vec(pipe_strategy(), 7usize..8),
        1.0e5..8.0e5f64,
        1.0e5..8.0e5f64,
    )
}

/// Builds the chain and returns it with its ordered edges and the disc's id.
fn build_disc_chain(
    before: &[Mid],
    after: &[Mid],
    (cv, full_open): (f64, f64),
    pipes: &[(f64, f64, f64, f64)],
    p_src: f64,
    p_snk: f64,
    fluid: &Fluid,
) -> (PlantGraph, Vec<refinery_core::graph::EdgeId>, NodeId) {
    let mut g = PlantGraph::new();
    let mut chain = vec![g.add_node(source(p_src, fluid))];
    for (i, m) in before.iter().enumerate() {
        chain.push(g.add_node(mid_node(m, i, fluid)));
    }
    let disc = g.add_node(Node {
        name: "disc".into(),
        kind: NodeKind::CheckValve {
            cv_max: valve_cv(cv, fluid),
            full_open: Pascal(full_open),
            x_t: fluid.x_t,
        },
        heat_input: Watt(0.0),
    });
    chain.push(disc);
    for (i, m) in after.iter().enumerate() {
        chain.push(g.add_node(mid_node(m, before.len() + i, fluid)));
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
    (g, edges, disc)
}

/// **A disc in a random liquid chain**, on both fidelities: the solve terminates
/// legally (I3) with nothing non-finite (I2); a converged series chain carries one
/// flow (I1); the disc's own outlet never carries a backward flow; and wherever the
/// drive across its branch, net of the outlet pipe's static head, is not forward,
/// it carries EXACTLY zero. Counted, with floors, so the arm cannot pass on a
/// generator that never shuts the disc, never lifts it fully, or never leaves it
/// inside its band.
#[test]
fn the_check_valve_arm_shuts_lifts_and_conserves() {
    // Liquid: the fluid is fixed, so the runner draws exactly what M30 recorded.
    let counts = run_disc_arm(&mut |_| Fluid::liquid());
    eprintln!(
        "check-valve chains: converged newton {}/{DISC_SAMPLES}, simple {}/{DISC_SAMPLES}; disc \
         on newton shut {} (with the sink above the source {}) / inside its band {} / full \
         lift {}",
        counts.converged[0],
        counts.converged[1],
        counts.shut,
        counts.backward_drive_shut,
        counts.partial,
        counts.full
    );
    counts.assert_floors();
}

/// **The same arm in gas service** (M31, docs/DESIGN.md §34, ledger row E24):
/// the disc's valve folds through the ISA compressible law, with `x_T` and the
/// two-cut slate drawn as the gas arm draws them and the coefficient scaled by
/// `GAS_CV_SCALE` so the valve, not the pipes, takes the drop. The same
/// invariants and the same floors, and one more: the disc must reach its
/// choked plateau in a share of the samples, or the arm runs on the `Y → 1`
/// tail where the gas fold is the liquid one and proves nothing about it.
#[test]
fn the_check_valve_arm_in_gas_service_shuts_lifts_chokes_and_conserves() {
    let gas = (0.05..0.95f64, 0.1..0.9f64).prop_map(|(w, x_t)| Fluid::gas(w, x_t));
    let counts = run_disc_arm(&mut |runner| {
        gas.new_tree(runner)
            .expect("strategy produces a value")
            .current()
    });
    eprintln!(
        "gas check-valve chains: converged newton {}/{DISC_SAMPLES}, simple {}/{DISC_SAMPLES}; \
         disc on newton shut {} (with the sink above the source {}) / inside its band {} / \
         full lift {} / choked {} (inside its band {})",
        counts.converged[0],
        counts.converged[1],
        counts.shut,
        counts.backward_drive_shut,
        counts.partial,
        counts.full,
        counts.choked,
        counts.choked_in_band
    );
    counts.assert_floors();
    assert!(
        counts.choked >= DISC_SAMPLES / 40,
        "the gas disc must reach its choked plateau: {} of {DISC_SAMPLES}",
        counts.choked
    );
}

const DISC_SAMPLES: usize = 400;

/// What one run of the disc arm reached, counted on Newton's answers.
struct DiscCounts {
    converged: [usize; 2],
    shut: usize,
    partial: usize,
    full: usize,
    backward_drive_shut: usize,
    choked: usize,
    /// Choked while the disc is still inside its band.
    choked_in_band: usize,
}

impl DiscCounts {
    fn assert_floors(&self) {
        let (shut, partial, full) = (self.shut, self.partial, self.full);
        assert!(
            shut >= DISC_SAMPLES / 8 && partial >= DISC_SAMPLES / 40 && full >= DISC_SAMPLES / 8,
            "the arm must reach all three states: shut {shut}, in band {partial}, full {full}"
        );
        assert!(
            self.converged[0] * 4 >= DISC_SAMPLES * 3 && self.converged[1] * 2 >= DISC_SAMPLES,
            "converged newton {} simple {} of {DISC_SAMPLES}",
            self.converged[0],
            self.converged[1]
        );
    }
}

/// A disc in a random chain of `next_fluid`'s fluid, on both fidelities: the
/// solve terminates legally (I3) with nothing non-finite (I2); a converged
/// series chain carries one flow (I1); the disc's own outlet never carries a
/// backward flow; and wherever the drive across its branch, net of the outlet
/// pipe's static head, is not forward, it carries EXACTLY zero. Counted, with
/// floors, so the arm cannot pass on a generator that never shuts the disc,
/// never lifts it fully, or never leaves it inside its band.
fn run_disc_arm(next_fluid: &mut dyn FnMut(&mut TestRunner) -> Fluid) -> DiscCounts {
    let mut runner = TestRunner::deterministic();
    let strat = disc_chain_strategy();
    let mut counts = DiscCounts {
        converged: [0; 2],
        shut: 0,
        partial: 0,
        full: 0,
        backward_drive_shut: 0,
        choked: 0,
        choked_in_band: 0,
    };
    for _ in 0..DISC_SAMPLES {
        let (before, after, disc_spec, pipes, p_src, p_snk) = strat
            .new_tree(&mut runner)
            .expect("strategy produces a value")
            .current();
        let fluid = next_fluid(&mut runner);
        let (g, edges, disc) =
            build_disc_chain(&before, &after, disc_spec, &pipes, p_src, p_snk, &fluid);
        let outlet = outlet_of(&g, disc).expect("the disc has an outlet").0;
        for (k, which) in ["newton", "simple"].into_iter().enumerate() {
            let outcome = if k == 0 {
                NewtonFlowSolver::default().solve(
                    &g,
                    &fluid.slate,
                    &Default::default(),
                    Seconds(0.1),
                )
            } else {
                SimpleFlowSolver::default().solve(
                    &g,
                    &fluid.slate,
                    &Default::default(),
                    Seconds(0.1),
                )
            };
            let sol = match outcome {
                Ok(sol) => sol,
                Err(SimError::SolverDiverged { .. }) | Err(SimError::AnchoringUnsettled { .. }) => {
                    continue
                }
                Err(other) => panic!("{which}: unexpected error: {other}"),
            };
            assert!(
                sol.diagnostics.converged && all_finite(&sol),
                "{which}: I2/I3"
            );
            counts.converged[k] += 1;

            // I1 on a series chain: one flow, to the fidelity's own tolerance.
            let (tol_abs, tol_rel) = if k == 0 {
                let s = NewtonFlowSolver::default();
                (s.tol_abs_kg_s, s.tol_rel)
            } else {
                let s = SimpleFlowSolver::default();
                (s.tol_abs_kg_s, s.tol_rel)
            };
            let flows: Vec<f64> = edges.iter().map(|e| sol.edge_mass_flow[e]).collect();
            let throughput = flows.iter().fold(0.0f64, |m, f| m.max(f.abs()));
            let (lo, hi) = flows
                .iter()
                .fold((f64::MAX, f64::MIN), |(lo, hi), &f| (lo.min(f), hi.max(f)));
            assert!(
                hi - lo <= 10.0 * (tol_abs + tol_rel * throughput),
                "{which}: one flow along the chain, spread {} at throughput {throughput}",
                hi - lo
            );

            // The disc itself, read against its OWN branch at the answer.
            let pressures = pressures_of(&sol);
            let c = refinery_solvers::network::compile_edge(
                &g,
                outlet,
                &fluid.slate,
                &Default::default(),
                &pressures,
            )
            .expect("the disc's edge compiles at the answer");
            let drive = pressures[&c.src] - pressures[&c.tgt] - c.branch.beta;
            let flow = sol.edge_mass_flow[&outlet];
            assert!(
                flow >= 0.0,
                "{which}: the disc passed {flow} kg/s backwards"
            );
            if drive <= 0.0 {
                assert_eq!(
                    flow, 0.0,
                    "{which}: a disc with drive {drive} Pa carries nothing"
                );
            }
            if k == 0 {
                let NodeKind::CheckValve { full_open, .. } = g.node(disc).kind else {
                    unreachable!("the disc is a check valve")
                };
                if drive <= 0.0 {
                    counts.shut += 1;
                    if p_snk > p_src {
                        counts.backward_drive_shut += 1;
                    }
                } else if drive < full_open.value() {
                    counts.partial += 1;
                } else {
                    counts.full += 1;
                }
                if valve_edge_is_choked(&g, &sol, &fluid, outlet) {
                    counts.choked += 1;
                    if drive < full_open.value() {
                        counts.choked_in_band += 1;
                    }
                }
            }
        }
    }
    counts
}
