//! M14.1: the recovered vapour (docs/DESIGN.md §16).
//!
//! The gates the design note named before anything was built, plus the fork-5
//! refusals. The plant under test is `scenarios/crude_column_recovery.toml`:
//! `crude_column_boiloff.toml` with a cooled `recovery_drum` and
//! `vent_to = "recovery_drum"` on the two tanks that boil.
//!
//! **Gate 3's reference is not the one the note names, and the plant is what
//! says so.** §16 asks that "the drum's contents are richer in the light cut
//! than the emitting tank's". On this plant that inequality points the WRONG
//! WAY: the drum takes vapour from two tanks, the heavier of them carries 2.3×
//! the flow, and the drum's light fraction (0.2614 at tick 6 000) sits well
//! BELOW `naphtha_tank`'s own liquid (0.4971). The reference the gate needs is
//! the flow-weighted mix of ALL the emitting liquids — 0.1523 — against which
//! the drum is 1.72× richer. Written the note's way the gate would have been
//! falsified by the CORRECT engine.
//!
//! **What is NOT a gate here, for M7.4b's reason.** "The drum's mass gain equals
//! what arrived minus what it re-vented" is the holdup's own mass update
//! restated. Gate 1 below has two independent sides: what the EMITTING tanks
//! published, against what the RECEIVING tank did with it.

use refinery_core::snapshot::{EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::units::T_AMBIENT;
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/crude_column_recovery.toml");
/// The same plant with every vent still ending at the atmosphere — gate 1's
/// control, and the file this one is meant to be diffed against.
const CONTROL: &str = include_str!("../../../scenarios/crude_column_boiloff.toml");

/// The naphtha tank first boils at tick 1 210, the distillate tank at 2 380, and
/// the drum first re-vents at 2 760 — so a run has to pass 2 760 before either
/// arm of gate 4 exists, and every gate here samples at the end.
const TICKS: u64 = 6_000;
const DT: f64 = 0.1;

const NAPHTHA_VENT: &str = "naphtha_tank__boiloff_vent";
const DISTILLATE_VENT: &str = "distillate_tank__boiloff_vent";
const BOTTOMS_VENT: &str = "bottoms_tank__boiloff_vent";
const DRUM_VENT: &str = "recovery_drum__boiloff_vent";
const DRUM: &str = "recovery_drum";

/// The file below its header comment.
///
/// Every edit in this module is a string replacement, and the demo's header
/// quotes its own table names and keys — `[nodes.recovery_drum]`,
/// `vent_to = "recovery_drum"` — as part of explaining the diff against
/// `crude_column_boiloff.toml`. Editing the whole file rewrites the COMMENT
/// first and produces a document that no longer parses, which is how three of
/// these tests failed before they ever reached the engine.
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
        refinery_core::graph::NodeKind::Tank(t) => (
            t.mass.value(),
            t.composition.fractions().to_vec(),
            t.temperature.value(),
        ),
        other => panic!("node '{name}' is a {other:?}, not a tank"),
    }
}

/// The holdups' total enthalpy above the datum [J].
///
/// `m·c̄p(x)·(T − T_REF)` telescopes EXACTLY through the tank update — the engine
/// integrates that same expression and then divides it back out to get `T` — so
/// this is the control volume's internal energy and not an approximation of it.
fn holdup_energy(engine: &Engine) -> f64 {
    use refinery_core::energy::T_REF;
    use refinery_core::graph::NodeKind;
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

/// Net power INTO the control volume "every tank on this plant" [W], from the
/// streams the last tick published.
///
/// **The control volume is the TANKS, not the plant, and that choice is what
/// makes this gate about M14.** A plant-wide balance on this file would have to
/// carry the furnace duty and the column's condenser and reboiler duties — and
/// M7.4b established that the reboiler duty is *defined* as the condenser duty
/// plus the column's own external balance, so the column's contribution closes
/// by construction and audits nothing (M13.1 recorded the same thing about its
/// gate 1). Drawing the surface around the holdups instead puts the column
/// outside it, where its duties are somebody else's arithmetic, and leaves
/// exactly the terms this milestone added.
///
/// **A vent between two tanks is INTERIOR and contributes nothing**, which is
/// the thing being measured: the emitting tank loses the vapour's enthalpy
/// through its flash (a state change, inside `holdup_energy`) and the receiving
/// tank gains it through `stream_enthalpy_flux`. The two cancel only if the
/// receiver is credited `c̄p·(T − T_REF) + λ`. A receiver calling
/// `enthalpy_flux` instead drops `λ` and this balance misses by the whole
/// arriving latent term.
///
/// **A vent ending anywhere else CROSSES the surface** and is subtracted: the
/// drum's own vent to atmosphere, and `bottoms_tank`'s, which carries nothing.
fn tank_boundary_power(engine: &Engine) -> f64 {
    use refinery_core::energy::{stream_enthalpy_flux, T_REF};
    use refinery_core::graph::NodeKind;
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
        // An ordinary edge contributes at whichever end is a tank. The draws
        // are the only such edges on this plant, and each has exactly one.
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

/// `Σ UA·(T_AMB − T)` over the tanks, at their START-of-tick temperatures.
///
/// The engine's `energy::heat_load` reads the temperature the tank had when the
/// tick began, so a probe sampling after the tick would be charging the
/// condenser at the wrong temperature — an O(`dt`) error that looks exactly like
/// a truncation term and is really a measurement bug.
fn tank_ambient_power(engine: &Engine) -> f64 {
    use refinery_core::graph::NodeKind;
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
struct Run {
    /// Σ ṁ·dt through each named vent [kg], in the order asked for.
    vented: Vec<f64>,
    /// Each named vent published a nonzero flow on at least one tick.
    reached: Vec<bool>,
    /// Σ (enthalpy arriving at the drum)·dt [J], and its latent half.
    arriving_energy: (f64, f64),
    /// `ΔU` over the tanks, and what the boundary says it should have been [J].
    energy: (f64, f64),
    final_snapshot: Snapshot,
}

fn run(src: &str, vents: &[&str]) -> Run {
    let mut engine = build(src);
    let start = holdup_energy(&engine);
    let mut vented = vec![0.0; vents.len()];
    let mut reached = vec![false; vents.len()];
    let mut arriving = (0.0, 0.0);
    let mut expected = 0.0;
    for t in 1..=TICKS {
        // Before the tick, because that is the state `heat_load` reads.
        expected += tank_ambient_power(&engine) * DT;
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
        expected += tank_boundary_power(&engine) * DT;
        // Off the GRAPH, not off a snapshot: `Engine::snapshot` allocates the
        // whole published document, and taking one on every tick of every run in
        // this file is the dev-profile cost CI actually pays. The vent's stream
        // is the same object the snapshot would copy.
        for (i, name) in vents.iter().enumerate() {
            if let Some(eid) = engine
                .graph
                .edge_ids()
                .find(|e| engine.graph.pipe(*e).name == *name)
            {
                let m = engine.graph.pipe(eid).stream.mass_flow.value();
                vented[i] += m * DT;
                reached[i] |= m > 0.0;
            }
        }
        for name in [NAPHTHA_VENT, DISTILLATE_VENT] {
            let Some(eid) = engine
                .graph
                .edge_ids()
                .find(|e| engine.graph.pipe(*e).name == name)
            else {
                continue;
            };
            let stream = &engine.graph.pipe(eid).stream;
            let cp = stream.composition.mixture_cp(&engine.slate);
            arriving.0 += refinery_core::energy::stream_enthalpy_flux(stream, cp).value() * DT;
            arriving.1 += stream.mass_flow.value() * stream.latent.map_or(0.0, |l| l.value()) * DT;
        }
    }
    Run {
        vented,
        reached,
        arriving_energy: arriving,
        energy: (holdup_energy(&engine) - start, expected),
        final_snapshot: engine.snapshot(),
    }
}

// ---------------------------------------------------------------------------
// Gate 1 — mass: what the emitters published is what the receiver did with it
// ---------------------------------------------------------------------------

/// **The two sides are independent, which is what keeps this out of M7.4b's
/// trap.** The left is what the two EMITTING tanks put on their vent edges over
/// the run; the right is what the RECEIVING drum re-vented plus what it kept.
/// Neither is computed from the other — the emitters' rates come from their own
/// flashes and the drum's inventory from a Euler integration whose only input is
/// the edge.
///
/// **Four reachability counters, because 0 = 0 + 0 is true.** Both source vents
/// must have carried vapour, `bottoms_tank`'s must not have (it is the file's
/// control for the default destination), and the drum's own must have — without
/// the last one the gate is passed by a perfect condenser, which deletes fork
/// 1's whole argument.
///
/// **The control is the plant this one was made from, and it is NOT bit-exact —
/// measured, with its own control.** `crude_column_boiloff.toml` runs the same
/// two tanks into the same flashes, and their vented masses agree to 8.7e-11
/// and 1.2e-10 relative rather than exactly. That is not the routing: adding an
/// inert `spare_tank` to `crude_column_boiloff.toml` — nothing piped to it, no
/// `vent_to` anywhere on the plant — moves the same two figures by 7.1e-11 and
/// 9.8e-10. **A plant with one more node is a different plant at the last few
/// bits**, and the mechanism is not pinned here (M9.1's rule: do not fit one).
/// What the comparison does say is that the EMITTING half of the engine is
/// indifferent to where its vapour goes.
#[test]
fn what_the_two_tanks_vent_is_what_the_drum_receives() {
    let demo = run(
        DEMO,
        &[NAPHTHA_VENT, DISTILLATE_VENT, BOTTOMS_VENT, DRUM_VENT],
    );
    assert!(demo.reached[0], "the naphtha tank must boil");
    assert!(demo.reached[1], "the distillate tank must boil");
    assert!(
        !demo.reached[2],
        "the bottoms tank is this file's control and must never boil"
    );
    assert!(
        demo.reached[3],
        "the drum must re-vent, or gate 4 has no upper arm and fork 1 is untested"
    );

    let arrived = demo.vented[0] + demo.vented[1];
    let revented = demo.vented[3];
    let (drum_mass, _, _) = tank(&demo.final_snapshot, DRUM);
    // 0.5 m × 8 m² of light naphtha at 680 kg/m³, the file's declared start.
    let accumulated = drum_mass - 0.5 * 8.0 * 680.0;

    // The bound is float summation over 6 000 terms, not physics: both sides are
    // rectangle sums of the SAME per-tick rate, so nothing is being
    // approximated. Twelve orders under the quantity itself.
    let residual = (arrived - revented - accumulated).abs();
    assert!(
        residual / arrived < 1e-12,
        "the drum's books must close: {arrived:.4} kg arrived, {revented:.4} kg left again, \
         {accumulated:.4} kg stayed — residual {residual:.6e} kg"
    );

    let control = run(CONTROL, &[NAPHTHA_VENT, DISTILLATE_VENT, BOTTOMS_VENT]);
    assert!(
        edge(&control.final_snapshot, DRUM_VENT).is_none(),
        "the control plant has no drum"
    );
    for (i, name) in [(0, "naphtha"), (1, "distillate")] {
        let relative = (demo.vented[i] - control.vented[i]).abs() / control.vented[i];
        assert!(
            relative < 1e-9,
            "the {name} tank must be indifferent to where its vapour goes: \
             {:.10e} kg against {:.10e} kg, {relative:.3e} relative",
            demo.vented[i],
            control.vented[i]
        );
    }
}

// ---------------------------------------------------------------------------
// Gate 2 — energy: the holdups' books close with the drum in the loop
// ---------------------------------------------------------------------------

/// **The mirror of I6b, on a plant where the vapour is received rather than
/// thrown away.** `ΔU` over the four tanks equals the enthalpy that crossed
/// their surface plus the heat the drum's condenser removed. See
/// `tank_boundary_power` for why the surface is drawn there and why a vent
/// between two tanks contributes nothing to it.
///
/// **The counterfactual is measured, not asserted.** Over the run the drum
/// receives **4.121041e9 J**, of which **1.751830e9 J — 42.51% — is the latent
/// term**. A receiver calling `enthalpy_flux` instead of `stream_enthalpy_flux`
/// drops exactly that, which is B16's defect mirrored. Measured under the
/// mutation, this gate then misses by **3.35e-2 relative**, against a bound of
/// `1e-9` and a passing residual of 8.51e-14: **twelve orders**.
///
/// **The tolerance is derived by RUNNING the `dt` discriminator, not by
/// reasoning about it** — see `the_energy_residual_does_not_fall_like_a_truncation_term`,
/// which is `#[ignore]`d beside this gate. The measured residual is
/// **8.5104e-14** at the shipped step, and 6.3446e-14 and 2.4341e-13 at half and
/// a quarter of it. It does NOT fall the way a truncation term must (a
/// first-order one would reach a quarter of the coarse value; this reaches
/// 2.86× it), so the bound is not sized against an engine error. It also does
/// not rise cleanly like `1/dt` the way M13.1's did — it dips first — which is
/// what the summation floor looks like rather than a clean cancellation law, and
/// saying so is part of the measurement. `1e-9` is four orders above the
/// residual and twelve below the mutation.
#[test]
fn the_holdups_energy_books_close_with_the_drum_in_the_loop() {
    let demo = run(
        DEMO,
        &[NAPHTHA_VENT, DISTILLATE_VENT, BOTTOMS_VENT, DRUM_VENT],
    );
    let (delta, expected) = demo.energy;
    let scale = delta.abs().max(expected.abs());
    let relative = (delta - expected).abs() / scale;
    // Printed rather than only formatted on failure, because the tolerance
    // paragraph above quotes these numbers and a figure a passing test never
    // emits is a figure nobody measured. `cargo test -- --nocapture`.
    println!("gate 2: dU = {delta:.6e} J, boundary = {expected:.6e} J, relative = {relative:.4e}");
    assert!(
        relative < 1e-9,
        "ΔU over the tanks = {delta:.6e} J against {expected:.6e} J of boundary enthalpy \
         plus condenser duty: relative miss {relative:.4e}"
    );

    // The control on the control: the arriving latent term has to be a large
    // share of the arriving enthalpy, or "the balance closes" is a claim about
    // a term that was barely there.
    let (total, latent) = demo.arriving_energy;
    println!(
        "gate 2: arriving enthalpy {total:.6e} J, of which latent {latent:.6e} J = {:.2}%",
        100.0 * latent / total
    );
    assert!(
        latent / total > 0.30,
        "the latent share of the enthalpy arriving at the drum is {:.2}% \
         ({latent:.6e} J of {total:.6e} J) — too small for this gate to be about it",
        100.0 * latent / total
    );
}

/// **The derivation behind gate 2's `1e-9`, run rather than reasoned.**
///
/// `#[ignore]` because it is three full runs of a cascade plant at successively
/// finer steps and it defends a CONSTANT rather than a behaviour — the same
/// reason M9.3b declined to gate an iteration count. Run it with
/// `cargo test --release -p refinery-scenarios --test vapour_recovery_reference
/// -- --ignored --nocapture` when the tolerance is questioned.
///
/// **What it discriminates.** Gate 2's residual could be either of two things,
/// and they move in opposite directions as the step shrinks. Euler truncation
/// FALLS with `dt` — a first-order term is four times smaller at `dt/4`, a
/// second-order one sixteen. Float cancellation in the gate's own `ΔU` does not
/// fall at all, and M13.1 measured its own gate rising like `1/dt`
/// (4.58e-12 → 7.0e-12 → 1.40e-11).
///
/// **Measured here, and it is a WEAKER answer than M13.1's** — which is worth
/// stating rather than rounding into the same sentence. Over the same 600 s of
/// plant time:
///
/// | step | ticks | relative residual |
/// |---|---:|---:|
/// | `dt` = 0.1 s | 6 000 | 8.5104e-14 |
/// | `dt/2` | 12 000 | 6.3446e-14 |
/// | `dt/4` | 24 000 | 2.4341e-13 |
///
/// It is **not monotone**: it dips and then rises, ending 2.86× above where it
/// started. So `1/dt` is not what this is, and neither is truncation — a
/// first-order term would have arrived at a quarter of the coarse value and this
/// is nearly three times it. What it looks like is the summation floor: both
/// sides are ~5e10 J accumulated over 6 000 to 24 000 terms, they agree to about
/// a hundred ULP, and how the last bits land is not a smooth function of the
/// step. The DISCRIMINATION still holds, and it is the only thing the tolerance
/// needs: gate 2's bound is not sized against an engine error, because there
/// is no engine error there to size it against. The balance telescopes exactly.
///
/// The tick count scales with the step so all three runs cover the same 600 s of
/// plant time, or the comparison is between different trajectories.
#[test]
#[ignore]
fn the_energy_residual_does_not_fall_like_a_truncation_term() {
    let mut residuals = Vec::new();
    for (label, dt, ticks) in [
        ("dt", "0.1", 6_000u64),
        ("dt/2", "0.05", 12_000),
        ("dt/4", "0.025", 24_000),
    ] {
        let src = body(DEMO).replace("dt = 0.1", &format!("dt = {dt}"));
        let mut engine = build(&src);
        let start = holdup_energy(&engine);
        let step: f64 = dt.parse().expect("a step");
        let mut expected = 0.0;
        for t in 1..=ticks {
            expected += tank_ambient_power(&engine) * step;
            engine
                .tick()
                .unwrap_or_else(|e| panic!("{label}: tick {t}: {e}"));
            expected += tank_boundary_power(&engine) * step;
        }
        let delta = holdup_energy(&engine) - start;
        let relative = (delta - expected).abs() / delta.abs().max(expected.abs());
        println!("{label:5} ({ticks} ticks): relative residual {relative:.4e}");
        assert!(
            relative < 1e-9,
            "{label}: the balance must close at every step size, not {relative:.4e}"
        );
        residuals.push(relative);
    }
    // The one thing the tolerance rests on. A first-order truncation term would
    // land at a QUARTER of the coarse-step value at `dt/4` and a second-order
    // one at a sixteenth; the bar is half, which leaves a factor of two of slack
    // against the first-order prediction and is nowhere near the measured 2.86×.
    // Deliberately not an assertion that it RISES: it does not do so monotonically
    // and a gate saying it did would be fitted to two of three points.
    let (coarse, fine) = (residuals[0], residuals[2]);
    assert!(
        fine > coarse / 2.0,
        "the residual falls like a truncation term ({coarse:.4e} at dt, {fine:.4e} at \
         dt/4), so gate 2's bound is sized against the engine rather than against the \
         gate's own summation and the tolerance paragraph is wrong"
    );
}

// ---------------------------------------------------------------------------
// Gate 3 — the drum fills at the VAPOUR's composition, and fractionates
// ---------------------------------------------------------------------------

/// **The gate for §16's mutation 2, and M12.1's finding is why it compares two
/// states at ONE instant rather than two runs.** A receiver resolving its inflow
/// through `edge_composition_at` would read the upwind end of the vent — the
/// emitting tank — and fill the drum with LIQUID `x`. The drum would still gain
/// the right mass, still close gate 1, and still re-vent; only its contents
/// would be wrong.
///
/// The reference is the flow-weighted mix of the emitting tanks' liquids, not
/// one tank's — see this file's header, where the note's own wording is
/// falsified by the correct engine. Measured at tick 6 000: the drum holds
/// 0.2614 light naphtha against 0.1523 for the liquid mix (1.72×) and 0.0018
/// residue against 0.0784 (44× leaner).
///
/// **The second half is the fractionation itself.** The drum re-vents at
/// `y_B = K·x_B`, its OWN equilibrium, and not the mixture that arrived: 0.8333
/// light naphtha leaving against 0.2760 arriving. A drum that merely passed its
/// inflow through would publish what came in.
#[test]
fn the_drum_fills_at_the_vapour_composition_and_re_vents_at_its_own() {
    let mut engine = build(DEMO);
    for t in 1..=TICKS {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
    }
    let snapshot = engine.snapshot();

    let naphtha_vent = edge(&snapshot, NAPHTHA_VENT).expect("the naphtha vent");
    let distillate_vent = edge(&snapshot, DISTILLATE_VENT).expect("the distillate vent");
    let (a, b) = (
        naphtha_vent.stream.mass_flow.value(),
        distillate_vent.stream.mass_flow.value(),
    );
    assert!(
        a > 0.0 && b > 0.0,
        "both tanks must be boiling at the sampled tick, or the weights below name nothing"
    );

    let (_, x_naphtha, _) = tank(&snapshot, "naphtha_tank");
    let (_, x_distillate, _) = tank(&snapshot, "distillate_tank");
    let (_, x_drum, _) = tank(&snapshot, DRUM);
    let mix = |p: &[f64], q: &[f64]| -> Vec<f64> {
        p.iter()
            .zip(q)
            .map(|(u, v)| (a * u + b * v) / (a + b))
            .collect()
    };
    let liquid_mix = mix(&x_naphtha, &x_distillate);
    let vapour_mix = mix(
        naphtha_vent.stream.composition.fractions(),
        distillate_vent.stream.composition.fractions(),
    );

    // The control first, M9.3b's shape: the two references must actually
    // differ, or "the drum matches one of them" is satisfied by both.
    assert!(
        (vapour_mix[0] - liquid_mix[0]).abs() > 0.05,
        "the vapour and liquid references must differ in the light cut: {:.5} against {:.5}",
        vapour_mix[0],
        liquid_mix[0]
    );

    assert!(
        x_drum[0] > 1.5 * liquid_mix[0],
        "the drum must be richer in the light cut than the emitting LIQUIDS: \
         {:.5} against {:.5}",
        x_drum[0],
        liquid_mix[0]
    );
    let heaviest = x_drum.len() - 1;
    assert!(
        x_drum[heaviest] < 0.1 * liquid_mix[heaviest],
        "and leaner in the heaviest: {:.5} against {:.5}",
        x_drum[heaviest],
        liquid_mix[heaviest]
    );

    let y_drum = edge(&snapshot, DRUM_VENT)
        .expect("the drum's own vent")
        .stream
        .composition
        .fractions()
        .to_vec();
    assert!(
        y_drum[0] > x_drum[0] + 0.2,
        "the drum's vapour must be richer in the light cut than its own liquid: \
         {:.5} against {:.5}",
        y_drum[0],
        x_drum[0]
    );
    assert!(
        (y_drum[0] - vapour_mix[0]).abs() > 0.2,
        "and it must not be the mixture that arrived: {:.5} against {:.5}",
        y_drum[0],
        vapour_mix[0]
    );
}

// ---------------------------------------------------------------------------
// Gate 4 — recovery is interior, and the condenser is what moves it
// ---------------------------------------------------------------------------

fn with_drum_ua(ua: &str) -> String {
    body(DEMO).replace(
        "ambient_exchange_ua_w_per_k = 35000.0",
        &format!("ambient_exchange_ua_w_per_k = {ua}"),
    )
}

/// The fraction of the arriving mass the drum kept, over a whole run.
fn recovered_fraction(src: &str) -> f64 {
    let r = run(
        src,
        &[NAPHTHA_VENT, DISTILLATE_VENT, BOTTOMS_VENT, DRUM_VENT],
    );
    let arrived = r.vented[0] + r.vented[1];
    assert!(arrived > 0.0, "the source tanks must boil");
    (arrived - r.vented[3]) / arrived
}

/// **Two-sided, because one-sided is passed by both failures.** A drum that
/// condensed everything recovers 100%; a drum that condensed nothing recovers
/// 0%. Either would pass "recovery > 0" or "recovery < 1" alone.
///
/// **The knob is shown to MOVE THE ANSWER**, which is M7.1's rule — mutate a
/// knob before trusting a plant to cover it, because `smearing_k` is what
/// happens when nobody does. Measured over the run: `UA = 0` recovers 20.72%,
/// the shipped 3.5e4 W/K recovers 42.52%, and 1e5 W/K recovers 100.00%.
///
/// **Fork 4's stated mechanism is FALSE, and this is where the measurement says
/// so.** §16 argues for a heat sink on the grounds that "a drum with no cooling
/// reaches its bubble point and re-vents everything: at steady state it recovers
/// nothing and the demo is dead". It does not. An uncooled drum boils the LIGHT
/// material back off and keeps the heavy, so its own bubble point climbs as it
/// goes: by tick 6 000 it is a 435.2 K pot holding 3 183.9 kg that has stopped
/// re-venting altogether — its instantaneous retention is 100%, and its run
/// recovery is 20.72% of a stream it has fractionated the wrong way round. The
/// case for the condenser is therefore not "otherwise nothing is recovered" but
/// "otherwise half as much is recovered, and what is kept is the bottom of the
/// barrel".
///
/// **And the recovered fraction is NOT MONOTONE in `UA` at the low end**:
/// 20.72% at 0, 17.35% at 5e3, 23.75% at 1.5e4, 42.52% at 3.5e4, 100% at 1e5.
/// Two mechanisms compete — cooling retains mass, and what is retained moves the
/// drum's own bubble point — so a gate asserting monotonicity would be fitted to
/// wherever it happened to sample. This one asserts the two ENDS, which is what
/// fork 4 is actually about.
#[test]
fn the_drums_recovery_is_interior_and_the_condenser_is_what_moves_it() {
    let shipped = recovered_fraction(body(DEMO));
    assert!(
        shipped > 0.05 && shipped < 0.95,
        "the shipped plant must recover an interior fraction over the run, not {shipped:.4}"
    );

    let uncooled = recovered_fraction(&with_drum_ua("0.0"));
    let flooded = recovered_fraction(&with_drum_ua("1.0e5"));
    assert!(
        uncooled < shipped - 0.1,
        "the condenser must be worth something: {uncooled:.4} uncooled against \
         {shipped:.4} at the shipped UA"
    );
    assert!(
        flooded > 0.99,
        "enough cooling must condense essentially everything, not {flooded:.4}"
    );

    // The instantaneous arm at the shipped UA, which is the state a frontend
    // would sample: the drum is holding about half of what is arriving.
    let mut engine = build(body(DEMO));
    for t in 1..=TICKS {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
    }
    let s = engine.snapshot();
    let arriving = edge(&s, NAPHTHA_VENT)
        .expect("vent")
        .stream
        .mass_flow
        .value()
        + edge(&s, DISTILLATE_VENT)
            .expect("vent")
            .stream
            .mass_flow
            .value();
    let leaving = edge(&s, DRUM_VENT).expect("vent").stream.mass_flow.value();
    let held = (arriving - leaving) / arriving;
    assert!(
        held > 0.05 && held < 0.95,
        "and an interior fraction at the last tick, not {held:.4}"
    );
}

// ---------------------------------------------------------------------------
// Gate 6 — the refusals of fork 5, each with its own reason
// ---------------------------------------------------------------------------

fn refusal(src: &str) -> String {
    let file = refinery_scenarios::load_str(src).expect("the file must still parse");
    match refinery_scenarios::build_engine(&file) {
        Ok(_) => panic!("this plant must be refused at load"),
        Err(e) => e.to_string(),
    }
}

/// Every kind `NodeKind` has, each refused for its own reason rather than by
/// falling off the end of a list. M11 fork 4 is the precedent: a trigger naming
/// four node kinds from memory was falsified by the first slice that enumerated
/// the type.
#[test]
fn a_vent_may_end_only_at_an_atmosphere_or_a_tank() {
    // Naming an atmosphere explicitly is the DEFAULT written out, and is legal.
    // The loader builds `boiloff_atmosphere` only when a plant has none, so the
    // file has to declare one before it can name it.
    let explicit = body(DEMO)
        .replace(
            "[nodes.recovery_drum]",
            "[nodes.sky]\ntype = \"atmosphere\"\n\n[nodes.recovery_drum]",
        )
        .replacen("vent_to = \"recovery_drum\"", "vent_to = \"sky\"", 1);
    let file = refinery_scenarios::load_str(&explicit).expect("parses");
    refinery_scenarios::build_engine(&file)
        .expect("naming an atmosphere explicitly is the default, not an error");

    for (replacement, needle) in [
        ("vent_to = \"column\"", "which is a column"),
        ("vent_to = \"preheater\"", "which is a zero-volume node"),
        ("vent_to = \"crude_source\"", "which is a source"),
        ("vent_to = \"nowhere\"", "which is not a node in this plant"),
        ("vent_to = \"naphtha_tank\"", "which is itself"),
    ] {
        let src = body(DEMO).replacen("vent_to = \"recovery_drum\"", replacement, 1);
        let message = refusal(&src);
        assert!(
            message.contains(needle),
            "the refusal for `{replacement}` must say why: got {message}"
        );
    }
}

/// A `vessel` and a `sink` each need a plant that has one, so they get their own
/// case rather than a `vent_to` edit on a file with neither.
///
/// **The vessel case needs a plant of its own, and finding that out is the
/// point.** A `vessel` must hold a GAS-phase composition — §3a's own refusal,
/// which fires when the node is constructed, one pass before the vents are
/// built. The demo's slate is five liquid cuts, so a vessel cannot be declared
/// on it at all and the first draft of this test was refused for the wrong
/// reason with the right exit code. The fixture below carries a gas cut so that
/// the vessel is a legal node and fork 5's refusal is the one that fires.
#[test]
fn a_vent_may_not_end_at_a_vessel_or_a_sink() {
    const VESSEL_PLANT: &str = r#"
# No pipes: the refusal fires in `build_boiloff_vents`, which runs before
# `validate_topology` would have an opinion about an unconnected plant.
pipes = []

[meta]
name = "vent_to_a_vessel"
description = "A tank told to vent into a gas holdup."

[simulation]
dt = 0.1

[fidelity]
flow = "newton"
thermo = "trouton"
boiloff = "flash"

[[components]]
name = "naphtha"
tb_c = 80.0
molar_mass_kg_per_mol = 0.100
density_kg_per_m3 = 680.0
cp_j_per_kg_k = 2200.0

[[components]]
name = "vapour"
tb_c = -40.0
molar_mass_kg_per_mol = 0.030
cp_j_per_kg_k = 2000.0
phase = "gas"

[nodes.holdup]
type = "tank"
area_m2 = 8.0
height_m = 12.0
initial_level_m = 0.5
temperature_c = 40.0
composition = { naphtha = 1.0 }
vent_to = "receiver"

[nodes.receiver]
type = "vessel"
volume_m3 = 5.0
pressure_bar = 2.0
temperature_c = 40.0
composition = { vapour = 1.0 }
"#;
    let message = refusal(VESSEL_PLANT);
    assert!(
        message.contains("a gas holdup whose state is a pressure"),
        "got {message}"
    );

    let with_sink = body(DEMO).replace(
        "[nodes.recovery_drum]",
        "[nodes.flare]
type = \"sink\"
pressure_bar = 1.0
         composition = { light_naphtha = 1.0 }

[nodes.recovery_drum]",
    );
    let message =
        refusal(&with_sink.replacen("vent_to = \"recovery_drum\"", "vent_to = \"flare\"", 1));
    assert!(
        message.contains("a sink, whose composition is DECLARED"),
        "got {message}"
    );
}

/// **The cycle, with its reachability SHOWN rather than assumed** — §16 fork 5
/// asks for exactly that, because this project has four gates on record that had
/// no power over their own subject.
///
/// Two lines: point the drum's vent back at the naphtha tank that feeds it. The
/// refusal comes from `PlantGraph::holdup_evaluation_order`, which is the
/// function the engine calls every tick to decide which holdup to update first —
/// so a cycle is refused because no such order exists, and not because a rule
/// somewhere says cycles are bad.
#[test]
fn boil_off_vents_may_not_form_a_cycle() {
    let cyclic = body(DEMO).replace(
        "ambient_exchange_ua_w_per_k = 35000.0",
        "ambient_exchange_ua_w_per_k = 35000.0\nvent_to = \"naphtha_tank\"",
    );
    let message = refusal(&cyclic);
    assert!(
        message.contains("form a cycle"),
        "the cycle must be named: got {message}"
    );
    assert!(
        message.contains("naphtha_tank") && message.contains("recovery_drum"),
        "and both members named: got {message}"
    );
}

/// `vent_to` on a plant that builds no vents at all. Refused rather than
/// ignored, for `PuncturePipe`'s reason (M6.0): a key nothing reads is a file
/// that looks configured and is not.
#[test]
fn vent_to_is_refused_on_a_plant_that_never_boils() {
    let src = body(DEMO).replace("boiloff = \"flash\"", "boiloff = \"none\"");
    let message = refusal(&src);
    assert!(
        message.contains("selects a model that never boils"),
        "got {message}"
    );
}

/// **File order does not decide the answer**, which is what
/// `PlantGraph::holdup_evaluation_order` buys. Moving the drum's block above the
/// tanks that vent into it reverses their node ids, and a loop iterating
/// `node_ids()` would then update the drum BEFORE its feeders — parking one
/// tick of vapour in flight on every tick, a systematic `ṁ_v·dt` that gate 1's
/// tolerance would have to be widened to swallow. The two runs must agree
/// exactly, not nearly.
#[test]
fn moving_the_drum_up_the_file_changes_no_number() {
    let src = body(DEMO);
    let drum = src.find("[nodes.recovery_drum]").expect("the drum block");
    let pipes = src.find("[[pipes]]").expect("the pipe table");
    let naphtha = src.find("[nodes.naphtha_tank]").expect("the naphtha tank");
    let reordered = format!(
        "{}{}{}{}",
        &src[..naphtha],
        &src[drum..pipes],
        &src[naphtha..drum],
        &src[pipes..]
    );

    let mut a = build(src);
    let mut b = build(&reordered);
    let id_of = |e: &Engine, name: &str| {
        e.graph
            .node_ids()
            .position(|id| e.graph.node(id).name == name)
            .expect("the drum")
    };
    assert_ne!(
        id_of(&a, DRUM),
        id_of(&b, DRUM),
        "the reordered file must actually move the drum's node id"
    );

    for t in 1..=TICKS {
        a.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        b.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
    }
    let (mass_a, x_a, t_a) = tank(&a.snapshot(), DRUM);
    let (mass_b, x_b, t_b) = tank(&b.snapshot(), DRUM);
    assert_eq!(
        mass_a, mass_b,
        "the drum's inventory must not depend on file order"
    );
    assert_eq!(t_a, t_b, "nor its temperature");
    assert_eq!(x_a, x_b, "nor its composition");
}
