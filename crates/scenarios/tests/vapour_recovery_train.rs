//! M15.1: the second recovery stage (docs/DESIGN.md §17).
//!
//! The plant under test is `scenarios/crude_column_recovery_train.toml`:
//! `crude_column_recovery.toml` with `vent_to = "polish_drum"` on the drum and
//! a second identical drum behind it. Nothing in `core`, `solvers` or the
//! loader changed to make it run (§17 fork 2), so what these gates defend is
//! that a chain is a real second user of machinery M14.1 built for one hop —
//! and, for gate 3, that the machinery does anything at all.
//!
//! **The measurement that shaped gate 3, before any of it was written.**
//! `PlantGraph::holdup_evaluation_order` returns *exactly* `node_ids()` on this
//! file, on `crude_column_recovery.toml`, and on every other plant in the
//! corpus. Depth does not change that; DECLARATION ORDER does. So a reorder
//! gate has to build the reordered plant itself and prove the sort's answer
//! differs from node order before it can prove the answers agree — and at depth
//! 2 it can prove something a depth-1 plant cannot ask for, that the constraint
//! is TRANSITIVE and no single transposition of node order satisfies it. That
//! last claim is asserted by brute force below rather than by inspection, with
//! the depth-1 plant beside it as the control that a single swap DOES fix.
//!
//! **What is NOT a gate here.** "Stage 2's mass gain equals what stage 1
//! re-vented minus what stage 2 re-vented" at hop 2 alone would be the holdup's
//! own mass update restated (M7.4b). Gate 5 closes the chain end to end: what
//! the two product tanks published against what the two drums did with it, with
//! the interior hop cancelling.

use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::units::T_AMBIENT;
use refinery_core::Engine;

const TRAIN: &str = include_str!("../../../scenarios/crude_column_recovery_train.toml");
/// One stage, and the file this one is meant to be diffed against.
const ONE_STAGE: &str = include_str!("../../../scenarios/crude_column_recovery.toml");

/// Stage 1 first receives at tick 1 208, first re-vents at 2 752, and stage 2
/// does not reach its own bubble point until 3 834 — 64% of this run. Every gate
/// here samples at the end, and a shorter one would measure a plant with an
/// idle second stage.
const TICKS: u64 = 6_000;
const DT: f64 = 0.1;

const NAPHTHA_VENT: &str = "naphtha_tank__boiloff_vent";
const DISTILLATE_VENT: &str = "distillate_tank__boiloff_vent";
const BOTTOMS_VENT: &str = "bottoms_tank__boiloff_vent";
const STAGE_1_VENT: &str = "recovery_drum__boiloff_vent";
const STAGE_2_VENT: &str = "polish_drum__boiloff_vent";
const STAGE_1: &str = "recovery_drum";
const STAGE_2: &str = "polish_drum";

/// Both drums start at 0.5 m × 8 m² of light naphtha at 680 kg/m³, the files'
/// own declaration.
const DRUM_START_KG: f64 = 0.5 * 8.0 * 680.0;

/// The file below its header comment.
///
/// Same reason as M14.1's: this file's header quotes its own table names and
/// keys as part of explaining the diff, so a whole-document string replacement
/// rewrites the COMMENT first and produces a document that no longer parses.
fn body(src: &str) -> &str {
    &src[src.find("[meta]").expect("every scenario declares [meta]")..]
}

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("a shipped scenario must parse");
    refinery_scenarios::build_engine(&file)
        .unwrap_or_else(|e| panic!("a shipped scenario must build: {e}"))
}

fn node<'a>(snapshot: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    snapshot
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("the plant declares a node '{name}'"))
}

fn edge<'a>(snapshot: &'a Snapshot, name: &str) -> Option<&'a EdgeSnapshot> {
    snapshot.edges.iter().find(|e| e.name == name)
}

fn tank(snapshot: &Snapshot, name: &str) -> (f64, Vec<f64>, f64) {
    match &node(snapshot, name).kind {
        NodeKind::Tank(t) => (
            t.mass.value(),
            t.composition.fractions().to_vec(),
            t.temperature.value(),
        ),
        other => panic!("node '{name}' is a {other:?}, not a tank"),
    }
}

/// Every tank's state, off the graph, in node order. Gate 3's comparison.
fn tank_states(engine: &Engine) -> Vec<(String, f64, f64, Vec<f64>)> {
    engine
        .graph
        .node_ids()
        .filter_map(|id| match &engine.graph.node(id).kind {
            NodeKind::Tank(t) => Some((
                engine.graph.node(id).name.clone(),
                t.mass.value(),
                t.temperature.value(),
                t.composition.fractions().to_vec(),
            )),
            _ => None,
        })
        .collect()
}

// ---------------------------------------------------------------------------
// The two balance probes, carried over from M14.1 unchanged
// ---------------------------------------------------------------------------
//
// Both are depth-agnostic as written, and that is worth stating rather than
// rediscovering: `tank_boundary_power` classifies a vent by whether its
// RECEIVER is a tank, so the tank → drum hop and the drum → drum hop are both
// interior and only the drum → atmosphere hop crosses the surface. Nothing in
// either function counts hops.

/// The holdups' total enthalpy above the datum [J]. `m·c̄p(x)·(T − T_REF)`
/// telescopes exactly through the tank update, so this is the control volume's
/// internal energy rather than an approximation of it.
fn holdup_energy(engine: &Engine) -> f64 {
    use refinery_core::energy::T_REF;
    engine
        .graph
        .node_ids()
        .filter_map(|id| match &engine.graph.node(id).kind {
            NodeKind::Tank(t) => Some(
                t.mass.value()
                    * t.composition.mixture_cp(&engine.slate).value()
                    * (t.temperature.value() - T_REF.value()),
            ),
            _ => None,
        })
        .sum()
}

/// Net power INTO the control volume "every tank on this plant" [W].
///
/// The surface is drawn around the holdups so that the column's own duties stay
/// outside it — M7.4b's rule, since a reboiler duty defined to close a balance
/// cannot then be audited by it.
fn tank_boundary_power(engine: &Engine) -> f64 {
    use refinery_core::energy::{stream_enthalpy_flux, T_REF};
    let is_tank = |id| matches!(engine.graph.node(id).kind, NodeKind::Tank(_));
    let mut power = 0.0;
    for eid in engine.graph.edge_ids() {
        let pipe = engine.graph.pipe(eid);
        let stream = &pipe.stream;
        let (from, to) = engine.graph.endpoints(eid);
        if let Some(emitter) = pipe.leak.boiloff_vent_emitter() {
            let receiver = if from == emitter { to } else { from };
            if !is_tank(receiver) {
                let cp = stream.composition.mixture_cp(&engine.slate);
                power -= stream_enthalpy_flux(stream, cp).value().abs();
            }
            continue;
        }
        for (end, incoming) in [(from, false), (to, true)] {
            if !is_tank(end) {
                continue;
            }
            let into = if incoming {
                stream.mass_flow.value()
            } else {
                -stream.mass_flow.value()
            };
            let cp = stream.composition.mixture_cp(&engine.slate);
            power += into * cp.value() * (stream.temperature.value() - T_REF.value());
        }
    }
    power
}

/// `Σ UA·(T_AMB − T)` over the tanks, at their START-of-tick temperatures —
/// which is the state `energy::heat_load` reads. Sampling after the tick would
/// charge both condensers at the wrong temperature, an O(`dt`) error that looks
/// exactly like truncation and is really a measurement bug.
fn tank_ambient_power(engine: &Engine) -> f64 {
    engine
        .graph
        .node_ids()
        .filter_map(|id| match &engine.graph.node(id).kind {
            NodeKind::Tank(t) => {
                Some(t.ambient_ua.value() * (T_AMBIENT.value() - t.temperature.value()))
            }
            _ => None,
        })
        .sum()
}

/// One run, sampling every tick.
///
/// **At tick resolution, not at the snapshot interval, and that is not a
/// detail.** M15.0's first draft integrated these same rates over 10-tick spans
/// and got 42.52% where the answer is 42.4316% — and then cited the agreement
/// with M14.1's own coarsely-integrated figure as corroboration. Two numbers
/// computed the same wrong way agree.
struct Run {
    /// Σ ṁ·dt through each named vent [kg], in the order asked for.
    vented: Vec<f64>,
    /// The first tick on which each named vent published a nonzero flow, or 0.
    first_tick: Vec<u64>,
    /// Σ (enthalpy arriving)·dt and its latent half [J], at hop 1 and hop 2.
    arriving: [(f64, f64); 2],
    /// `ΔU` over the tanks, and what the boundary says it should have been [J].
    energy: (f64, f64),
    final_snapshot: Snapshot,
}

fn run(src: &str, vents: &[&str]) -> Run {
    let mut engine = build(src);
    let start = holdup_energy(&engine);
    let mut vented = vec![0.0; vents.len()];
    let mut first_tick = vec![0u64; vents.len()];
    let mut arriving = [(0.0, 0.0); 2];
    let mut expected = 0.0;
    let flow_of = |e: &Engine, name: &str| {
        e.graph
            .edge_ids()
            .find(|eid| e.graph.pipe(*eid).name == name)
    };
    for t in 1..=TICKS {
        // Before the tick, because that is the state `heat_load` reads.
        expected += tank_ambient_power(&engine) * DT;
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
        expected += tank_boundary_power(&engine) * DT;
        // Off the GRAPH, not off a snapshot: `Engine::snapshot` allocates the
        // whole published document, and taking one on every tick of every run
        // in this file is the dev-profile cost CI actually pays.
        for (i, name) in vents.iter().enumerate() {
            if let Some(eid) = flow_of(&engine, name) {
                let m = engine.graph.pipe(eid).stream.mass_flow.value();
                vented[i] += m * DT;
                if m > 0.0 && first_tick[i] == 0 {
                    first_tick[i] = t;
                }
            }
        }
        for (hop, feeders) in [
            (0usize, &[NAPHTHA_VENT, DISTILLATE_VENT][..]),
            (1, &[STAGE_1_VENT]),
        ] {
            for name in feeders {
                let Some(eid) = flow_of(&engine, name) else {
                    continue;
                };
                let stream = &engine.graph.pipe(eid).stream;
                let cp = stream.composition.mixture_cp(&engine.slate);
                arriving[hop].0 +=
                    refinery_core::energy::stream_enthalpy_flux(stream, cp).value() * DT;
                arriving[hop].1 +=
                    stream.mass_flow.value() * stream.latent.map_or(0.0, |l| l.value()) * DT;
            }
        }
    }
    Run {
        vented,
        first_tick,
        arriving,
        energy: (holdup_energy(&engine) - start, expected),
        final_snapshot: engine.snapshot(),
    }
}

fn train_run() -> Run {
    run(
        body(TRAIN),
        &[
            NAPHTHA_VENT,
            DISTILLATE_VENT,
            BOTTOMS_VENT,
            STAGE_1_VENT,
            STAGE_2_VENT,
        ],
    )
}

// ---------------------------------------------------------------------------
// Gate 1 — the train recovers more, and the emitting half does not notice
// ---------------------------------------------------------------------------

/// **Recovery alone cannot be the whole claim, which is why gate 2 exists**, and
/// this test measures the reason rather than citing it: one drum with
/// `UA = 1e5` recovers essentially everything too (M14.1), so a gate on mass is
/// passed by a plant with no second stage in it at all. That alternative is run
/// here as gate 1's own vacuity control.
///
/// **The emitting half is the second assertion and the more surprising one.**
/// The two product tanks vent 6 709.70 kg whether their vapour ends at an
/// atmosphere, at one drum, or at two — M14.1's finding, one hop further out.
/// It is NOT bit-exact and does not need to be: a plant with one more node is a
/// different plant at the last few bits (M14.1 measured that with an inert
/// `spare_tank` as its control). What is new here is the SIZE of it — 2.7e-9 and
/// 3.7e-9 relative against M14.1's 8.7e-11 and 1.2e-10, thirty times larger for
/// one more node — so the bound is 1e-8 and is a float-noise bound, not a
/// physics one.
#[test]
fn the_train_recovers_more_and_the_emitting_tanks_do_not_notice() {
    let train = train_run();
    let one_stage = run(
        body(ONE_STAGE),
        &[NAPHTHA_VENT, DISTILLATE_VENT, BOTTOMS_VENT, STAGE_1_VENT],
    );

    // Reachability first, because every ratio below is 0/0 without it.
    assert!(train.first_tick[0] > 0, "the naphtha tank must boil");
    assert!(train.first_tick[1] > 0, "the distillate tank must boil");
    assert_eq!(
        train.first_tick[2], 0,
        "the bottoms tank is this file's control and must never boil"
    );
    assert!(
        train.first_tick[3] > 0,
        "stage 1 must re-vent, or stage 2 is fed nothing and this plant is one stage"
    );
    assert!(
        train.first_tick[4] > 0,
        "stage 2 must re-vent within the shipped run, or its interior arm is unreachable \
         and the fifth dead gate in this project's record"
    );
    assert!(
        train.first_tick[4] > train.first_tick[3],
        "stage 2 cannot vent before stage 1 has sent it anything: {} against {}",
        train.first_tick[4],
        train.first_tick[3]
    );

    let arrived = train.vented[0] + train.vented[1];
    let train_recovery = (arrived - train.vented[4]) / arrived;
    let one_stage_recovery =
        (one_stage.vented[0] + one_stage.vented[1] - one_stage.vented[3]) / arrived;
    println!(
        "gate 1: {arrived:.4} kg emitted; train recovers {:.6}%, one stage {:.6}%",
        100.0 * train_recovery,
        100.0 * one_stage_recovery
    );
    assert!(
        train_recovery > one_stage_recovery + 0.30,
        "the second stage must recover materially more: {train_recovery:.6} against \
         {one_stage_recovery:.6}"
    );
    assert!(
        train_recovery < 0.99,
        "and must still be interior over the shipped run, not {train_recovery:.6} — a \
         saturated train has no interior arm to gate"
    );

    // The vacuity control for a mass-only reading of this milestone: ONE drum,
    // with the condenser M14.1 measured at 100%, matches the train on mass.
    let flooded = run(
        &body(ONE_STAGE).replace(
            "ambient_exchange_ua_w_per_k = 35000.0",
            "ambient_exchange_ua_w_per_k = 1.0e5",
        ),
        &[NAPHTHA_VENT, DISTILLATE_VENT, BOTTOMS_VENT, STAGE_1_VENT],
    );
    let flooded_recovery = (flooded.vented[0] + flooded.vented[1] - flooded.vented[3])
        / (flooded.vented[0] + flooded.vented[1]);
    println!(
        "gate 1: one drum at UA = 1e5 recovers {:.6}%",
        100.0 * flooded_recovery
    );
    assert!(
        flooded_recovery > train_recovery,
        "a single big condenser must beat the train on MASS ({flooded_recovery:.6} \
         against {train_recovery:.6}), or gate 2 is defending nothing and a recovery \
         number would have been enough"
    );

    for (i, name) in [(0, "naphtha"), (1, "distillate")] {
        let relative = (train.vented[i] - one_stage.vented[i]).abs() / one_stage.vented[i];
        println!("gate 1: {name} tank {relative:.4e} relative against one stage");
        assert!(
            relative < 1e-8,
            "the {name} tank must be indifferent to how far downstream its vapour goes: \
             {:.10e} kg against {:.10e} kg, {relative:.3e} relative",
            train.vented[i],
            one_stage.vented[i]
        );
    }

    // And one hop further out again: stage 1 on the train must reproduce the
    // single-drum plant's drum, because what it RECEIVES is unchanged and where
    // its own vapour goes is nothing to do with it.
    let (m_train, _, t_train) = tank(&train.final_snapshot, STAGE_1);
    let (m_one, _, t_one) = tank(&one_stage.final_snapshot, STAGE_1);
    let mass_rel = (m_train - m_one).abs() / m_one;
    println!("gate 1: stage 1 vs the one-stage drum, mass {mass_rel:.4e} relative");
    assert!(
        mass_rel < 1e-8 && (t_train - t_one).abs() / t_one < 1e-8,
        "stage 1 must be the same drum as the one-stage plant's: {m_train:.4} kg at \
         {t_train:.4} K against {m_one:.4} kg at {t_one:.4} K"
    );
}

// ---------------------------------------------------------------------------
// Gate 2 — two drums are two PRODUCTS, which a bigger condenser is not
// ---------------------------------------------------------------------------

/// **The gate that makes §17 fork 1 a plant rather than a knob.** One drum at a
/// high `UA` condenses the whole stream into ONE inventory at one temperature;
/// a train FRACTIONATES, because stage 2 receives stage 1's own equilibrium
/// vapour rather than the tanks'. Measured at tick 6 000: stage 1 is a 386.22 K
/// pot holding 26.14% light naphtha, stage 2 a 354.19 K one holding 95.44% —
/// 3.65× apart.
///
/// **Two states at one instant, not two runs** — M12.1's finding, now for the
/// fourth time. A trajectory comparison is passed by anything that moves the
/// answer; only a composition read off two inventories at one tick says the
/// train did the thing a train is for.
///
/// **The control is the arriving stream.** Stage 2 must be richer than what
/// arrives at it, or it is a bucket rather than a separator, and richer than
/// stage 1's own liquid, or the two drums are one drum written twice.
#[test]
fn the_two_drums_hold_different_products_at_one_instant() {
    let mut engine = build(body(TRAIN));
    for t in 1..=TICKS {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
    }
    let s = engine.snapshot();

    let (_, x1, t1) = tank(&s, STAGE_1);
    let (_, x2, t2) = tank(&s, STAGE_2);
    let arriving = edge(&s, STAGE_1_VENT).expect("stage 1's vent");
    assert!(
        arriving.stream.mass_flow.value() > 0.0,
        "stage 1 must be feeding stage 2 at the sampled tick, or the comparison below \
         names a stream that is not flowing"
    );
    let y1 = arriving.stream.composition.fractions();
    println!(
        "gate 2: stage 1 x[light] = {:.4} at {t1:.2} K, stage 2 x[light] = {:.4} at \
         {t2:.2} K, arriving y[light] = {:.4}",
        x1[0], x2[0], y1[0]
    );

    assert!(
        x2[0] > 3.0 * x1[0],
        "the two drums must hold different products: stage 2 at {:.5} light cut against \
         stage 1's {:.5}",
        x2[0],
        x1[0]
    );
    assert!(
        x2[0] > y1[0],
        "and stage 2 must be RICHER than the vapour arriving at it ({:.5} against \
         {:.5}) — a holdup that merely accumulated its inflow would match it",
        x2[0],
        y1[0]
    );
    assert!(
        t1 - t2 > 25.0,
        "the stages must sit at different temperatures: {t1:.2} K and {t2:.2} K"
    );
    let heaviest = x1.len() - 1;
    assert!(
        x2[heaviest] < 0.01 * x1[heaviest],
        "and stage 2 must be far leaner in the heaviest cut: {:.3e} against {:.3e}",
        x2[heaviest],
        x1[heaviest]
    );
}

// ---------------------------------------------------------------------------
// Gate 3 — the evaluation order, with the control M14.1's could not have
// ---------------------------------------------------------------------------

/// The plant with both drums declared ABOVE the tanks that feed them.
fn declared_receivers_first(src: &str, first_receiver: &str) -> String {
    let src = body(src);
    let drum = src.find(first_receiver).expect("the first drum's block");
    let pipes = src.find("[[pipes]]").expect("the pipe table");
    let naphtha = src.find("[nodes.naphtha_tank]").expect("the naphtha tank");
    format!(
        "{}{}{}{}",
        &src[..naphtha],
        &src[drum..pipes],
        &src[naphtha..drum],
        &src[pipes..]
    )
}

/// (emitter, receiver) for every tank → tank vent, as positions in `order`.
fn violations(engine: &Engine, order: &[refinery_core::graph::NodeId]) -> usize {
    let position = |id| order.iter().position(|n| *n == id).expect("a node");
    let mut bad = 0;
    for eid in engine.graph.edge_ids() {
        let Some(emitter) = engine.graph.pipe(eid).leak.boiloff_vent_emitter() else {
            continue;
        };
        let (from, to) = engine.graph.endpoints(eid);
        let receiver = if from == emitter { to } else { from };
        if !matches!(engine.graph.node(receiver).kind, NodeKind::Tank(_)) {
            continue;
        }
        if position(emitter) > position(receiver) {
            bad += 1;
        }
    }
    bad
}

/// **The gate B18 exists for, and its first half is the whole point.**
///
/// `PlantGraph::holdup_evaluation_order` returns exactly `node_ids()` on every
/// plant in the shipped corpus, this one included — so a reorder gate written
/// against a natural file proves nothing, and M14.1's compared node IDs rather
/// than the sort's own output. This one asserts, in code:
///
/// 1. On the file as shipped the sort agrees with node order — stated, so that
///    "the sort does something" cannot be read into a passing test.
/// 2. On a file declaring both drums first, node order VIOLATES the constraint
///    and the sort does not.
/// 3. **No single transposition of node order satisfies it either.** That is
///    what depth 2 buys and depth 1 cannot ask for, and the depth-1 plant is
///    run beside it as the control: on `crude_column_recovery.toml` reordered
///    the same way, one swap is enough.
/// 4. The reordered plant reproduces the shipped one BIT for BIT, on every
///    tank, on every tick — not merely at the end.
#[test]
fn the_evaluation_order_is_transitive_and_the_answer_does_not_depend_on_it() {
    let natural = build(body(TRAIN));
    let ids: Vec<_> = natural.graph.node_ids().collect();
    let order = natural
        .graph
        .holdup_evaluation_order()
        .expect("the shipped plant has an order");
    assert_eq!(
        order, ids,
        "the file as SHIPPED must be one the sort leaves alone — if this ever fails, \
         the coverage argument below has changed and §17 finding (i) is stale"
    );

    let reordered_src = declared_receivers_first(TRAIN, "[nodes.recovery_drum]");
    let reordered = build(&reordered_src);
    let r_ids: Vec<_> = reordered.graph.node_ids().collect();
    let r_order = reordered
        .graph
        .holdup_evaluation_order()
        .expect("the reordered plant has an order");
    let name = |e: &Engine, id| e.graph.node(id).name.clone();
    println!(
        "gate 3: reordered node_ids {:?}",
        r_ids
            .iter()
            .map(|i| name(&reordered, *i))
            .collect::<Vec<_>>()
    );
    println!(
        "gate 3: reordered eval     {:?}",
        r_order
            .iter()
            .map(|i| name(&reordered, *i))
            .collect::<Vec<_>>()
    );
    let broken = violations(&reordered, &r_ids);
    assert!(
        broken > 0,
        "the reordered file must actually put a receiver before its feeders, or this \
         gate measures nothing"
    );
    assert_eq!(
        violations(&reordered, &r_order),
        0,
        "and the sort must fix every one of them"
    );

    // (3) The transitive claim, by brute force over all transpositions.
    let single_swap_fixes = |e: &Engine, ids: &[refinery_core::graph::NodeId]| {
        for i in 0..ids.len() {
            for j in (i + 1)..ids.len() {
                let mut candidate = ids.to_vec();
                candidate.swap(i, j);
                if violations(e, &candidate) == 0 {
                    return true;
                }
            }
        }
        false
    };
    assert!(
        !single_swap_fixes(&reordered, &r_ids),
        "at depth 2 the constraint must be TRANSITIVE: no single transposition of node \
         order may satisfy it, or the reorder gate is passed by any implementation that \
         merely pushes receivers to the end"
    );

    // The control, and without it (3) is a claim about arithmetic rather than
    // about depth: the depth-1 plant reordered the same way IS fixed by one swap.
    let depth_one = build(&declared_receivers_first(
        ONE_STAGE,
        "[nodes.recovery_drum]",
    ));
    let d_ids: Vec<_> = depth_one.graph.node_ids().collect();
    assert!(
        violations(&depth_one, &d_ids) > 0,
        "the depth-1 control must be reordered too, or it proves nothing"
    );
    assert!(
        single_swap_fixes(&depth_one, &d_ids),
        "the depth-1 plant must be fixable by ONE swap — that is the coverage a chain \
         adds, and if this fails the claim above is about something else"
    );

    // (4) Bit-identical, every tank, every tick.
    let mut a = build(body(TRAIN));
    let mut b = build(&reordered_src);
    for t in 1..=TICKS {
        a.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        b.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        let (mut sa, mut sb) = (tank_states(&a), tank_states(&b));
        sa.sort_by(|p, q| p.0.cmp(&q.0));
        sb.sort_by(|p, q| p.0.cmp(&q.0));
        assert_eq!(
            sa, sb,
            "declaration order must not change any holdup's state, and it did at tick {t}"
        );
    }
}

// ---------------------------------------------------------------------------
// Gate 4 — a three-holdup cycle, refused at depth 2
// ---------------------------------------------------------------------------

/// The depth-2 form of M14.1's cycle refusal: a → b → c → a, which no pairwise
/// check can see. The refusal has two independently-defended callers — the
/// loader and `Engine::tick` — and M14.1 measured that removing it from one
/// leaves the other firing; this is the scenario half, and
/// `crates/solvers/tests/vapour_recovery_contract.rs` carries the hand-built
/// graph.
#[test]
fn a_three_holdup_vent_cycle_is_refused() {
    let cyclic = body(TRAIN).replace(
        "[nodes.polish_drum]\ntype = \"tank\"",
        "[nodes.polish_drum]\ntype = \"tank\"\nvent_to = \"naphtha_tank\"",
    );
    assert!(
        cyclic.contains("vent_to = \"naphtha_tank\""),
        "the edit must have applied, or this test refuses the wrong plant"
    );
    let file = refinery_scenarios::load_str(&cyclic).expect("the file must still parse");
    let message = match refinery_scenarios::build_engine(&file) {
        Ok(_) => panic!("a three-holdup vent cycle must be refused at load"),
        Err(e) => e.to_string(),
    };
    assert!(
        message.contains("form a cycle"),
        "the cycle must be named: got {message}"
    );
    for member in ["naphtha_tank", STAGE_1, STAGE_2] {
        assert!(
            message.contains(member),
            "every member of the cycle must be named, and '{member}' is missing from: \
             {message}"
        );
    }
}

// ---------------------------------------------------------------------------
// Gate 5 — mass closes across the CHAIN, with nothing in flight at either hop
// ---------------------------------------------------------------------------

/// **Both sides are independent, which keeps this out of M7.4b's trap.** The
/// left is what the two product tanks put on their vent edges over the run; the
/// right is what left the far end of the chain plus what the two drums kept.
/// The interior hop appears on neither side: stage 1's vent is an outflow of
/// one drum and an inflow of the other, and it cancels — which is exactly the
/// property a wrong evaluation order breaks, by parking `ṁ_v·dt` in flight at
/// that hop on every tick.
///
/// **This is the gate whose tolerance is the argument for the sort existing.**
/// M14.1 measured the cost of the wrong order at one hop as 1.69 kg against
/// 6 709.7, a systematic 2.5e-4; at two hops there are two places for it to
/// happen. The bound below is 1e-12, eight orders under that, and it is float
/// summation over 6 000 terms rather than physics — both sides are rectangle
/// sums of the same per-tick rates.
#[test]
fn mass_closes_across_both_hops_with_nothing_in_flight() {
    let train = train_run();
    let arrived = train.vented[0] + train.vented[1];
    let left = train.vented[4];
    let (m1, _, _) = tank(&train.final_snapshot, STAGE_1);
    let (m2, _, _) = tank(&train.final_snapshot, STAGE_2);
    let kept = (m1 - DRUM_START_KG) + (m2 - DRUM_START_KG);

    assert!(
        train.vented[3] > 0.0,
        "the interior hop must have carried something, or it cancels trivially"
    );
    let residual = (arrived - left - kept).abs();
    println!(
        "gate 5: {arrived:.4} kg in, {left:.4} kg out, {kept:.4} kg kept — residual \
         {residual:.6e} kg ({:.4e} relative)",
        residual / arrived
    );
    assert!(
        residual / arrived < 1e-12,
        "the chain's books must close: {arrived:.4} kg arrived, {left:.4} kg reached the \
         atmosphere, {kept:.4} kg stayed in the two drums — residual {residual:.6e} kg"
    );

    // Each hop separately, so a failure says WHICH one leaks. Neither of these
    // is independent of the holdup update on its own — that is the header's
    // point — but together with the chain balance above they localise a fault.
    let hop1 = (train.vented[0] + train.vented[1] - train.vented[3] - (m1 - DRUM_START_KG)).abs();
    let hop2 = (train.vented[3] - train.vented[4] - (m2 - DRUM_START_KG)).abs();
    println!("gate 5: hop 1 residual {hop1:.6e} kg, hop 2 residual {hop2:.6e} kg");
    assert!(
        hop1 / arrived < 1e-12 && hop2 / train.vented[3] < 1e-12,
        "each hop must close on its own: {hop1:.6e} kg at hop 1, {hop2:.6e} kg at hop 2"
    );
}

// ---------------------------------------------------------------------------
// Gate 6 — energy closes across the chain, with the latent term at BOTH hops
// ---------------------------------------------------------------------------

/// **I6b's shape, on a plant where the vapour is received twice.** `ΔU` over the
/// five tanks equals the enthalpy that crossed their surface plus the heat the
/// two condensers removed. Both vent hops are interior and contribute nothing —
/// but only if each receiver is credited `c̄p·(T − T_REF) + λ` rather than the
/// sensible half, which is B16's defect mirrored onto a receiving end.
///
/// **The second hop is a term the first hop's gate cannot see, and this
/// measures how big it is.** Over the run stage 1 receives 4.121041e9 J, 42.51%
/// of it latent; stage 2 receives 1.933335e9 J, **58.07%** latent — a larger
/// share, because what stage 1 re-vents is lighter and closer to its own bubble
/// point than what the product tanks emitted. Dropping `λ` at hop 2 alone —
/// scoped so hop 1 keeps its credit — takes the residual from 1.5435e-13 to
/// **2.1754e-2**, a hole of 1.12e9 J which is hop 2's own latent arrival to
/// three figures. It fires this gate and gate 1 — on gate 1's CONTROL, not its
/// recovery comparison: denied the arriving latent heat the polisher is credited
/// less energy, runs COLDER and stops re-venting at all, so "stage 2 must re-vent
/// within the shipped run" is what panics. Nothing else in the workspace fires:
/// no M14.1 gate sees it, and gate 5 is blind because no mass moved.
///
/// The bound is M14.1's `1e-9`, and the residual is 1.5435e-13. See
/// `the_energy_residual_does_not_fall_like_a_truncation_term` in
/// `vapour_recovery_reference.rs` for why that bound is not sized against an
/// engine error: the balance telescopes exactly and what is left is the gate's
/// own summation floor.
#[test]
fn the_energy_books_close_across_both_hops() {
    let train = train_run();
    let (delta, expected) = train.energy;
    let relative = (delta - expected).abs() / delta.abs().max(expected.abs());
    // Printed rather than only formatted on failure: a figure a passing test
    // never emits is a figure nobody measured (M14.1's own finding).
    println!("gate 6: dU = {delta:.6e} J, boundary = {expected:.6e} J, rel = {relative:.4e}");
    assert!(
        relative < 1e-9,
        "ΔU over the five tanks = {delta:.6e} J against {expected:.6e} J of boundary \
         enthalpy plus condenser duty: relative miss {relative:.4e}"
    );

    for (hop, (total, latent)) in train.arriving.iter().enumerate() {
        let share = latent / total;
        println!(
            "gate 6: hop {} receives {total:.6e} J, of which latent {latent:.6e} = {:.2}%",
            hop + 1,
            100.0 * share
        );
        assert!(
            *total > 0.0 && share > 0.30,
            "hop {} carries {:.2}% latent ({latent:.6e} J of {total:.6e} J) — too small \
             for this gate to be about it",
            hop + 1,
            100.0 * share
        );
    }
    assert!(
        train.arriving[1].1 / train.arriving[1].0 > train.arriving[0].1 / train.arriving[0].0,
        "the SECOND hop must be the more latent-heavy of the two, or the first hop's own \
         gate would have covered it: {:.4} against {:.4}",
        train.arriving[1].1 / train.arriving[1].0,
        train.arriving[0].1 / train.arriving[0].0
    );
}

// ---------------------------------------------------------------------------
// Gate 7 — the rename reaches the tank, and stops at the loader
// ---------------------------------------------------------------------------

/// **The wire-form half is what defends §17's own riskiest prediction**, and it
/// is asserted on the serialized bytes for M10.1's reason: a Rust match on a
/// field passes under any serde name.
///
/// The prediction is "all eighteen existing plants byte-identical". The rename
/// is byte-neutral because it stops at the LOADER — not because no published
/// field bears the name, which is what §17 fork 4's cost list says and which is
/// false. `NodeSnapshot::kind` serializes `TankState`, so `ambient_ua` IS a
/// published name; renaming the core field would have moved every plant with a
/// tank in it at once. This gate pins both ends: the new TOML key reaches the
/// graph unscaled, and the published document still spells the field the way
/// every frontend and every baseline already reads it.
///
/// The retired spelling's refusal lives beside the other loader refusals, in
/// `crates/scenarios/src/lib.rs`, with the counterfactual that shows an
/// unrecognised key is silently DROPPED.
#[test]
fn the_renamed_key_reaches_the_drum_and_the_published_name_is_unchanged() {
    let mut engine = build(body(TRAIN));
    for drum in [STAGE_1, STAGE_2] {
        let id = engine.graph.find_node(drum).expect("both drums");
        match &engine.graph.node(id).kind {
            NodeKind::Tank(t) => assert_eq!(
                t.ambient_ua.value(),
                35_000.0,
                "35 000 W/K under the new key must be 35 000 W/K on '{drum}', unscaled"
            ),
            other => panic!("'{drum}' is a {other:?}"),
        }
    }

    engine.tick().expect("one tick");
    let json = serde_json::to_string(&engine.snapshot()).expect("a snapshot serializes");
    assert!(
        json.contains("\"ambient_ua\":35000.0"),
        "the PUBLISHED field is still `ambient_ua` — renaming it would move every plant \
         with a tank in it, and the eighteen-plant prediction with them"
    );
    assert!(
        !json.contains("ambient_exchange_ua"),
        "and the scenario-facing spelling must not have leaked into the snapshot"
    );
}
