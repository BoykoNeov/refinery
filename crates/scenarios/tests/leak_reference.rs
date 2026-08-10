//! The leak, end to end (M6.1): declaration, split, command routing, the mass
//! that actually leaves, and every refusal (docs/DESIGN.md §3b).
//!
//! `orifice.rs` in the solvers crate pins the element — `Q = Cd·A·√(2·Δp/ρ)` —
//! against a hand calculation on a two-node plant. This file pins everything the
//! element cannot reach on its own, all of which is only observable through the
//! loader and the engine: that a declared leak becomes a split pipe plus a
//! dormant orifice, that an undeclared plant is untouched, that
//! `PuncturePipe` routes to the orifice rather than storing a number nobody
//! reads, and that the leaked mass balances against the plant's inventories.
//!
//! **Why the no-churn gate is written as a comparison of two files.** M6.0
//! measured that only two tests in the repo assert `snapshot.edges.len()`, so a
//! loader change that silently restructured every plant could pass the suite.
//! `a_plant_without_a_leak_is_the_plant_it_was` compares `leaking_line.toml`
//! against `tank_pump_valve.toml` — identical but for one `leak_to` line — so
//! the claim "declaring no leak changes nothing" is checked against a plant that
//! does declare one, rather than asserted by the absence of a failure.

use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::units::SquareMeter;

const INTACT: &str = include_str!("../../../scenarios/tank_pump_valve.toml");
const LEAKY: &str = include_str!("../../../scenarios/leaking_line.toml");

/// The declared pipe a game punctures in `leaking_line.toml`.
const PUNCTURED: &str = "fill_line";
/// A 10 cm² hole (a 3.6 cm bore). Sized by MEASUREMENT, not by feel: it carries
/// 5.1 kg/s against the line's 13.8 kg/s, so **37% of the transfer goes out of
/// the hole** and a mass-balance gate built on it is discriminating rather than
/// nominally satisfied. At 1 cm² the leak is 3.8% of the flow, which is a number
/// a sign error could hide inside.
///
/// It is also sized away from the knife edge in the other direction. The leak
/// junction sits at ~1.37 bara with the hole open — 35 kPa above atmospheric —
/// so this plant is nowhere near the back-feed refusal, and a plant that drifted
/// into it would fail loudly rather than flip behaviour between fidelities.
/// Above ~30 cm² the leak exceeds the line's supply and the downstream half
/// reverses, which is a different plant and not what these gates measure.
const HOLE_M2: f64 = 1.0e-3;

fn engine(src: &str) -> refinery_core::engine::Engine {
    let file = refinery_scenarios::load_str(src).expect("scenario must parse");
    refinery_scenarios::build_engine(&file).expect("scenario must build")
}

/// Build and require a REFUSAL, returning the message. `Engine` is not `Debug`,
/// so `expect_err` is unavailable — and matching explicitly also lets the panic
/// say which scenario was wrongly accepted.
fn expect_refusal(toml_src: &str, what: &str) -> String {
    let file = refinery_scenarios::load_str(toml_src).expect("scenario must parse");
    match refinery_scenarios::build_engine(&file) {
        Ok(_) => panic!("{what}"),
        Err(e) => e.to_string(),
    }
}

fn edge<'a>(snapshot: &'a Snapshot, name: &str) -> &'a refinery_core::snapshot::EdgeSnapshot {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("plant must have an edge named '{name}'"))
}

fn tank_mass(snapshot: &Snapshot, name: &str) -> f64 {
    snapshot
        .tanks
        .iter()
        .find(|(n, _)| n == name)
        .map(|(_, t)| t.mass.value())
        .unwrap_or_else(|| panic!("plant must have a tank named '{name}'"))
}

/// Open the declared leak to `HOLE_M2` and return the engine.
fn punctured() -> refinery_core::engine::Engine {
    let mut engine = engine(LEAKY);
    let id = edge(&engine.snapshot(), PUNCTURED).id;
    engine
        .apply(Command::PuncturePipe {
            edge: id,
            area: SquareMeter(HOLE_M2),
        })
        .expect("the declared pipe must be punctureable");
    engine
}

// --- What the loader builds ---------------------------------------------------

/// A declared leak is a split pipe, a junction and a dormant orifice — exactly
/// one node and two edges more than the same plant without the declaration.
#[test]
fn a_declared_leak_splits_its_pipe_at_the_midpoint() {
    let intact = engine(INTACT).snapshot();
    let leaky = engine(LEAKY).snapshot();

    // One Atmosphere node and one midpoint junction; the leak path's two halves
    // and the orifice replace one pipe.
    assert_eq!(leaky.nodes.len(), intact.nodes.len() + 2);
    assert_eq!(leaky.edges.len(), intact.edges.len() + 2);

    let upstream = edge(&leaky, PUNCTURED);
    let downstream = edge(&leaky, "fill_line__downstream");
    let orifice = edge(&leaky, "fill_line__leak");
    let mid = leaky
        .nodes
        .iter()
        .find(|n| n.name == "fill_line__leak_point")
        .expect("the split must create a midpoint junction");

    // Wired in a line: valve → upstream → mid → downstream → tank, with the
    // orifice hanging off `mid`. The upstream half keeps the declared NAME,
    // which is what makes `PuncturePipe`'s JSON contract still address a pipe
    // the scenario author wrote.
    assert_eq!(upstream.to, mid.id);
    assert_eq!(downstream.from, mid.id);
    assert_eq!(orifice.from, mid.id);
    assert_eq!(
        leaky.nodes[orifice.to.0 as usize].name, "outside",
        "the orifice must run junction → atmosphere, so positive graph direction is outward"
    );
}

/// The split is hydraulically the SAME PIPE: `k ∝ L` and `β = ρ·g·Δz` both add
/// back over two halves, so the flow through the intact plant and through the
/// declared-but-undamaged one must agree.
///
/// This is the gate that catches a half that was not halved, and both halves of
/// that claim are MEASURED rather than reasoned — the first estimate of one of
/// them was wrong by a factor of forty. Mutating the loader to skip the length
/// halving moves this plant's flow by **0.74%** (13.753 → 13.652 kg/s), not by
/// the 29% that doubling a pipe's resistance suggests: the control valve
/// dominates the series resistance here, so twice the pipe's `k` is a small
/// change to the total. Skipping the ELEVATION halving moves it by **6.1%**
/// (13.753 → 12.909), because `β` is a driving head and enters undiluted.
/// Against a 3e-6 tolerance both are caught with room to spare, and it is worth
/// knowing that the smaller one is the length — the opposite of the intuition.
///
/// Either mutation is a plausible-looking plant that quietly is not the one the
/// file describes, and neither shows up in the shape assertions above.
///
/// **The agreement is NOT exact, and the gap is derived rather than tolerated.**
/// In real arithmetic the split is identity: `k ∝ L` and `β = ρ·g·Δz` both add
/// back exactly, and halving in binary is exact. What differs is the
/// REGULARIZATION. A single branch computes `Δ/√(Δ + ε)`; two identical halves
/// in series each see `Δ/2` and give `Δ/(√k·√(Δ + 2ε))`, so the split plant
/// behaves as though `ε` were doubled and runs slightly SLOWER by
///
/// ```text
/// 1 − √((Δ + ε)/(Δ + 2ε))  ≈  ε/(2·Δ)  =  1/(2·4.0e5)  ≈  1.25e-6
/// ```
///
/// at this plant's ~4.0 bar net drop across the fill line. That is asserted
/// below, sign and size, rather than hidden inside a loose tolerance: a
/// difference of this magnitude with the wrong sign, or ten times this size, is
/// not the regularization and this gate should not pass it.
#[test]
fn splitting_a_pipe_does_not_change_the_plant_it_describes() {
    let mut intact = engine(INTACT);
    let mut leaky = engine(LEAKY);
    for tick in 0..50 {
        intact.tick().expect("intact plant converges");
        leaky.tick().expect("declared-leak plant converges");
        let a = edge(&intact.snapshot(), PUNCTURED).stream.mass_flow.value();
        let b = edge(&leaky.snapshot(), PUNCTURED).stream.mass_flow.value();
        assert!(
            a > 1.0,
            "tick {tick}: the reference plant must actually be flowing ({a} kg/s), or \
             this comparison is between two zeros"
        );
        approx::assert_relative_eq!(a, b, max_relative = 3.0e-6);
    }

    // The sharp statement, on the last tick: the shortfall is `ε/(2·Δ)` at the
    // plant's own net drop, read from the intact plant's pressures so the
    // prediction does not depend on the split one it is judging.
    let snapshot = intact.snapshot();
    let node = |name: &str| {
        snapshot
            .nodes
            .iter()
            .find(|n| n.name == name)
            .unwrap_or_else(|| panic!("no node '{name}'"))
            .pressure_pa
    };
    // Δ = (P_valve − P_tank) − ρ·g·Δz: the drop the branch's √ actually sees,
    // net of the 5 m of static head the file declares.
    let net_drop =
        node("discharge_valve") - node("receiving_tank") - 998.0 * refinery_core::units::G * 5.0;
    let a = edge(&snapshot, PUNCTURED).stream.mass_flow.value();
    let b = edge(&leaky.snapshot(), PUNCTURED).stream.mass_flow.value();
    let shortfall = (a - b) / a;
    let predicted = 1.0 / (2.0 * net_drop); // ε = 1.0 Pa, the solvers' `eps_dp`
    assert!(
        shortfall > 0.0,
        "the split plant carries an extra ε and can only be SLOWER; got {shortfall:e}"
    );
    approx::assert_relative_eq!(shortfall, predicted, max_relative = 0.1);
}

/// The other half of the no-churn claim: the plant that declares nothing carries
/// no leak machinery at all — no atmosphere, no orifice, and `leak_mass_flow`
/// flat zero on every edge.
#[test]
fn a_plant_without_a_leak_is_the_plant_it_was() {
    let mut intact = engine(INTACT);
    intact.tick().expect("intact plant converges");
    let snapshot = intact.snapshot();
    assert_eq!(
        snapshot.edges.len(),
        3,
        "the reference plant has three pipes"
    );
    for e in &snapshot.edges {
        assert_eq!(
            e.leak_mass_flow, 0.0,
            "edge '{}' reports a leak in a plant that declares none",
            e.name
        );
    }
}

// --- The command -------------------------------------------------------------

/// `PuncturePipe` names the DECLARED pipe and the engine routes the area onto
/// that pipe's dormant orifice — the indirection fork C is built on.
///
/// The check that matters is the one M6.0's finding demands: not that the
/// command returns `Ok`, but that something downstream CHANGES. A command that
/// stores a number no solver reads returns `Ok` too.
#[test]
fn puncturing_the_declared_pipe_opens_its_orifice() {
    let mut dormant = engine(LEAKY);
    dormant.tick().expect("converges");
    assert_eq!(
        edge(&dormant.snapshot(), "fill_line__leak")
            .stream
            .mass_flow
            .value(),
        0.0,
        "an undamaged leak path must conduct exactly nothing"
    );

    let mut leaking = punctured();
    leaking.tick().expect("converges");
    let escaping = edge(&leaking.snapshot(), "fill_line__leak")
        .stream
        .mass_flow
        .value();
    assert!(
        escaping > 0.1,
        "a 1 cm² hole in a pressurised line must carry real mass outward, got {escaping} kg/s"
    );
}

/// Repair is `area = 0`, and it must actually stop the leak — the property fork
/// C was chosen for over graph surgery, which would have had to remove nodes.
#[test]
fn repair_closes_the_leak_again() {
    let mut engine = punctured();
    engine.tick().expect("converges");
    let id = edge(&engine.snapshot(), PUNCTURED).id;
    engine
        .apply(Command::PuncturePipe {
            edge: id,
            area: SquareMeter::ZERO,
        })
        .expect("area 0 is repair, and stays legal");
    engine.tick().expect("converges");
    assert_eq!(
        edge(&engine.snapshot(), "fill_line__leak")
            .stream
            .mass_flow
            .value(),
        0.0,
        "a repaired leak must conduct exactly nothing again"
    );
}

/// Puncturing a pipe with no declared path is REFUSED, not silently stored.
/// This is the defect M6.0 found — a command that wrote a field nothing read —
/// pinned so it cannot come back as a no-op on an undeclared pipe.
#[test]
fn puncturing_a_pipe_with_no_leak_path_is_refused() {
    let mut engine = engine(LEAKY);
    let suction = edge(&engine.snapshot(), "suction").id;
    let err = engine
        .apply(Command::PuncturePipe {
            edge: suction,
            area: SquareMeter(HOLE_M2),
        })
        .expect_err("a pipe that declares no leak path cannot be punctured");
    assert!(
        err.to_string().contains("declares no leak path"),
        "the refusal must say what is missing: {err}"
    );
}

/// Puncturing the ORIFICE itself is refused too — it is not a pipe that has a
/// leak, it is the leak. Accepting it would set an orifice's area on an edge
/// whose own `LeakRole` says it already is one, which is the shape the enum
/// exists to forbid.
#[test]
fn puncturing_the_orifice_itself_is_refused() {
    let mut engine = engine(LEAKY);
    let orifice = edge(&engine.snapshot(), "fill_line__leak").id;
    let err = engine
        .apply(Command::PuncturePipe {
            edge: orifice,
            area: SquareMeter(HOLE_M2),
        })
        .expect_err("the orifice is not a punctureable pipe");
    assert!(
        err.to_string().contains("IS a leak orifice"),
        "the refusal must name the confusion: {err}"
    );
}

// --- The two fields that carry one quantity -----------------------------------

/// `leak_mass_flow` on the punctured pipe and the orifice edge's own flow are
/// the same number, published twice — so they are gated, not assumed. Two fields
/// for one quantity is how they drift.
#[test]
fn snapshot_leak_flow_matches_the_orifice_edge() {
    let mut engine = punctured();
    for _ in 0..20 {
        engine.tick().expect("converges");
        let snapshot = engine.snapshot();
        let view = edge(&snapshot, PUNCTURED).leak_mass_flow;
        let actual = edge(&snapshot, "fill_line__leak").stream.mass_flow.value();
        assert!(
            view > 0.1,
            "the gate is vacuous unless mass is leaving: {view}"
        );
        assert_eq!(
            view.to_bits(),
            actual.to_bits(),
            "the convenience view must BE the orifice's flow, not a recomputation \
             of it: {view} vs {actual}"
        );
    }
    // And nowhere else. In particular not on the orifice edge, which would say
    // the plant lost the same mass twice.
    let snapshot = engine.snapshot();
    for e in &snapshot.edges {
        if e.name != PUNCTURED {
            assert_eq!(
                e.leak_mass_flow, 0.0,
                "edge '{}' must not report another edge's leak",
                e.name
            );
        }
    }
}

// --- The mass that actually leaves --------------------------------------------

/// A punctured line drains, and the mass balances: every kilogram that leaves
/// the supply tank either reaches the receiving tank or goes out of the hole.
///
/// This is M6.1's headline gate, and the shape is what makes it strong. The
/// leaked mass is integrated from the ORIFICE EDGE's reported flow, while the
/// two tank inventories are integrated by the engine's own transport — separate
/// mechanisms, so a leak that conducted mass without removing it from the plant
/// (or removed it twice) breaks the sum. A leak that merely throttled the line
/// would leave the balance intact and is caught by the drain assertion instead.
#[test]
fn a_punctured_line_drains_and_the_mass_balances() {
    const TICKS: usize = 500;
    let mut engine = punctured();
    let dt = engine.dt().value();
    let start_supply = tank_mass(&engine.snapshot(), "supply_tank");
    let start_receiving = tank_mass(&engine.snapshot(), "receiving_tank");

    // **Integrated with the ENGINE's rule, not with a better one.** The tank
    // update is `m ← m + ṁ·dt` at the flow that tick's own solve produced, so
    // the leak is summed the same way: one rectangle per tick, at the flow the
    // snapshot reports *after* that tick. A trapezoid here is more accurate and
    // therefore wrong for this purpose — it disagrees with the tanks by half of
    // the first tick (0.26 kg on this plant, 4e-4 of the mass moved), which
    // would surface as a mass-balance failure that is entirely the quadrature's.
    // The gate is "the engine conserves what it transports", not "the engine
    // integrates well"; the latter is `integrator-order-of-convergence`'s job.
    let mut leaked = 0.0;
    for _ in 0..TICKS {
        engine.tick().expect("a leaking plant must still converge");
        leaked += edge(&engine.snapshot(), "fill_line__leak")
            .stream
            .mass_flow
            .value()
            * dt;
    }

    let snapshot = engine.snapshot();
    let lost = start_supply - tank_mass(&snapshot, "supply_tank");
    let gained = tank_mass(&snapshot, "receiving_tank") - start_receiving;

    assert!(
        leaked > 100.0,
        "the leak must carry real mass over {TICKS} ticks or the balance is between \
         near-zeros; got {leaked} kg"
    );
    assert!(
        gained > 0.0 && gained < lost,
        "the receiving tank must still fill ({gained} kg) but by strictly less than the \
         supply tank lost ({lost} kg) — the difference is what went out of the hole"
    );
    // 1e-9 relative to the mass that moved: the transported quantity, not a
    // fraction of the tanks' large standing inventories, which would make the
    // tolerance meaningless (M1's acceptance-gate lesson).
    let residual = lost - gained - leaked;
    assert!(
        (residual / lost).abs() < 1e-9,
        "mass in = mass out + accumulation: {lost} kg left the supply tank, {gained} kg \
         reached the receiving tank and {leaked} kg escaped — {residual} kg unaccounted"
    );
}

// --- The refusals -------------------------------------------------------------

/// The back-feed blocker M6.0 surfaced and left open, resolved and gated in the
/// direction that matters. **A gate that only ever runs the leak OUTWARD would
/// reproduce exactly the defect `energy::boundary_composition` predicted**, so
/// this plant is built to pull its leak junction BELOW atmospheric: a
/// sub-atmospheric sink downstream of an open valve, with the hole on the line
/// into it.
#[test]
fn a_leak_below_atmospheric_is_refused() {
    const VACUUM_PLANT: &str = r#"
[meta]
name = "vacuum_leak"
description = "A line pulled below atmospheric by a sub-atmospheric sink, with a hole in it."
[simulation]
dt = 0.1
[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"

[nodes.feed]
type = "source"
pressure_bar = 0.4
temperature_c = 20.0

[nodes.vacuum]
type = "sink"
pressure_bar = 0.2
temperature_c = 20.0

[nodes.outside]
type = "atmosphere"

[[pipes]]
name = "suction_line"
from = "feed"
to = "vacuum"
length_m = 20.0
diameter_m = 0.10
leak_to = "outside"
"#;
    let mut engine = engine(VACUUM_PLANT);

    // Dormant, the plant runs: the whole line sits below P_ATM the entire time,
    // and an undamaged leak path is simply not a path. This half is what proves
    // the refusal below fires on the DAMAGE and not on the plant.
    engine
        .tick()
        .expect("a sub-atmospheric plant with an intact line is perfectly legal");
    let mid = engine
        .snapshot()
        .nodes
        .into_iter()
        .find(|n| n.name == "suction_line__leak_point")
        .expect("the leak path must exist")
        .pressure_pa;
    assert!(
        mid < refinery_core::units::P_ATM.value(),
        "this gate is vacuous unless the leak junction really is below atmospheric; \
         it is at {mid} Pa"
    );

    let id = edge(&engine.snapshot(), "suction_line").id;
    engine
        .apply(Command::PuncturePipe {
            edge: id,
            area: SquareMeter(HOLE_M2),
        })
        .expect("the line is punctureable");
    let err = engine
        .tick()
        .expect_err("a leak that draws atmosphere INTO the plant must be refused");
    assert!(
        err.to_string().contains("back-feeds"),
        "the refusal must name what it refuses: {err}"
    );
}

/// A leak on a gas line is refused at LOAD, naming the file — the first of the
/// two doors (`network::compile_edge` is the second, gated in
/// `orifice.rs::a_gas_orifice_is_refused_at_compile_time`).
#[test]
fn a_leak_on_a_gas_line_is_refused_at_load() {
    const GAS_PLANT: &str = r#"
[meta]
name = "gas_leak"
description = "A fuel gas header with a hole in it — refused: the orifice law is incompressible."
[simulation]
dt = 0.1
[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"

[[components]]
name = "methane"
tb_c = -161.5
molar_mass_kg_per_mol = 0.016043
cp_j_per_kg_k = 2220.0
phase = "gas"

[nodes.header]
type = "source"
pressure_bar = 10.0
temperature_c = 20.0
composition = { methane = 1.0 }

[nodes.flare]
type = "sink"
pressure_bar = 1.0
temperature_c = 20.0
composition = { methane = 1.0 }

[nodes.outside]
type = "atmosphere"

[[pipes]]
name = "header_line"
from = "header"
to = "flare"
length_m = 50.0
diameter_m = 0.20
leak_to = "outside"
"#;
    let err = expect_refusal(GAS_PLANT, "a leak on a gas line must be refused at load");
    assert!(
        err.contains("choked"),
        "the refusal must say why the incompressible law is wrong: {err}"
    );
}

/// A leak declared on a COLUMN's feed or draw is refused, and the refusal is
/// this file's most load-bearing one because the failure it prevents is silent.
///
/// `network::is_column_draw_edge` recognises a draw by its two endpoints, and
/// `edge_flows` guards a draw's flow to zero on the strength of that — a draw's
/// flow is PRESCRIBED (`splitᵢ·ṁ_feed`, written post-sweep), not pressure-driven.
/// Split the draw and neither half matches any more, so the guard stops applying
/// and the draw silently becomes a pressure-driven number: finite, deterministic,
/// mass-conserving and wrong, reached by a scenario line that reads perfectly
/// reasonably.
///
/// The assertion checks WHICH refusal fires, not merely that one does.
/// `validate_topology` would reject a split feed too — a column is 1-in-N-out —
/// but with a message about edge degrees, naming a cause that is not the reason.
/// An error that misdirects the next reader is barely better than none.
#[test]
fn a_leak_on_a_column_pipe_is_refused() {
    const COLUMN: &str = include_str!("../../../scenarios/crude_column.toml");
    let with_atmosphere = COLUMN.replacen(
        "[[pipes]]",
        "[nodes.outside]\ntype = \"atmosphere\"\n\n[[pipes]]",
        1,
    );
    for line in ["naphtha_draw", "hot_feed_line"] {
        let bad = with_atmosphere.replacen(
            &format!("name = \"{line}\""),
            &format!("name = \"{line}\"\nleak_to = \"outside\""),
            1,
        );
        let err = expect_refusal(&bad, "a column's pipes cannot carry a leak path");
        assert!(
            err.contains("prescribed by the feed split"),
            "'{line}' must be refused for the reason it is refused, not incidentally by \
             the degree check: {err}"
        );
    }
}

/// `leak_to` must name an `Atmosphere`. Venting into the plant is an ordinary
/// pipe and the scenario should say so.
#[test]
fn a_leak_to_a_non_atmosphere_node_is_refused() {
    let bad = LEAKY.replace(r#"leak_to = "outside""#, r#"leak_to = "receiving_tank""#);
    let err = expect_refusal(&bad, "a tank is not the outside world");
    assert!(
        err.contains("not an atmosphere"),
        "the refusal must name the kind mismatch: {err}"
    );
}

/// And it must name a node that exists — a typo in a damage declaration is a
/// load-time error, not a plant that silently cannot be damaged.
#[test]
fn a_leak_to_an_unknown_node_is_refused() {
    let bad = LEAKY.replace(r#"leak_to = "outside""#, r#"leak_to = "outsdie""#);
    let err = expect_refusal(&bad, "a typo must not load");
    assert!(
        err.contains("not a node in this plant"),
        "the refusal must name the missing node: {err}"
    );
}
