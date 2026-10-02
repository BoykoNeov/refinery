//! M31: a check valve in GAS service — `docs/DEFERRED.md` row E24, built as
//! `docs/DESIGN.md` §34 specifies.
//!
//! The element under test: M30's disc, whose opening is read off the forward
//! drive across its branch, with the valve that opening sets folded through the
//! ISA compressible law exactly as a `Valve`'s is (`network::fold_gas_service`,
//! one owner for all three valve kinds).
//!
//! Gates, one claim each:
//!
//! 1. **The disc keeps a receiver out of its header**: on the demo, the disc's
//!    outlet never carries a backward flow and carries exactly zero until the
//!    receiver has blown down below what the header can push, on both
//!    fidelities, where the twin with a plain valve in its place runs backwards
//!    into the header. The two then settle to one answer.
//! 2. **At full lift it IS a plain gas valve, bit for bit, and the fold is
//!    really in it**: the disc's branch equals a `Valve`'s at opening 1 with the
//!    same `kv` and `x_t`, on a RISING tail (so the static head is not a few
//!    pascals), at a drop where the gas law matters and at the choke —
//!    and differs from the incompressible branch there, which is the control
//!    that catches a disc that skipped the fold.
//! 3. **The opening's share of the slope is right in gas**, unchoked and choked:
//!    the share `ṁ·k` against the true share (the disc's centred difference less
//!    a plain valve's at the same fixed opening), so the fold's own frozen-`x`
//!    error — which every gas valve carries — is subtracted out rather than
//!    folded into the tolerance. Shut and carrying exactly nothing backwards.
//! 4. **Every refusal, each on its own message.**
//! 5. **The demo never comes near the choke**, measured, so the file's claim
//!    that the fold is exercised elsewhere is a number rather than a hope.
//! 6. **A disc publishes `x_t` only in gas service**, on the serialized bytes.

use refinery_core::energy::NodeStates;
use refinery_core::graph::{EdgeId, NodeId, NodeKind, PlantGraph};
use refinery_core::Engine;
use refinery_scenarios::{build_engine, load_str};
use refinery_solvers::elements::{
    check_opening, pipe_resistance, specific_heat_ratio_factor, QuadraticBranch,
};
use refinery_solvers::network::{compile_edge, CompiledEdge, RHO_WATER_REF};
use refinery_solvers::NewtonFlowSolver;
use std::collections::BTreeMap;

const DEMO: &str = include_str!("../../../scenarios/gas_receiver_check_valve.toml");

/// The demo's check valve, as declared.
const CHECK_DECL: &str = "type = \"check_valve\"\nkv = 12.0\nfull_open_bar = 0.1\nx_t = 0.72";
/// The same body as a plain valve held fully open: the twin.
const PLAIN_DECL: &str = "type = \"valve\"\nkv = 12.0\nopening = 1.0\nx_t = 0.72";

const FIDELITIES: [&str; 2] = ["newton", "simple"];

fn swap(src: &str, from: &str, to: &str) -> String {
    assert!(
        src.contains(from),
        "the fixture's substitution must land: `{from}` is not in the plant"
    );
    src.replacen(from, to, 1)
}

fn on(fidelity: &str, src: &str) -> String {
    swap(src, "flow = \"newton\"", &format!("flow = \"{fidelity}\""))
}

fn build(src: &str) -> Engine {
    build_engine(&load_str(src).expect("the plant must parse"))
        .unwrap_or_else(|e| panic!("the plant must build: {e}"))
}

fn refusal(what: &str, src: &str) -> String {
    match load_str(src) {
        Err(e) => e.to_string(),
        Ok(file) => match build_engine(&file) {
            Ok(_) => panic!("{what}: this plant should not have loaded"),
            Err(e) => e.to_string(),
        },
    }
}

fn node(graph: &PlantGraph, name: &str) -> NodeId {
    graph
        .find_node(name)
        .unwrap_or_else(|| panic!("plant must declare a '{name}' node"))
}

fn pipe(graph: &PlantGraph, name: &str) -> EdgeId {
    graph
        .edge_ids()
        .find(|e| graph.pipe(*e).name == name)
        .unwrap_or_else(|| panic!("plant must declare a '{name}' pipe"))
}

fn edge_flow(engine: &Engine, name: &str) -> f64 {
    engine
        .snapshot()
        .edges
        .iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("the plant declares a pipe '{name}'"))
        .stream
        .mass_flow
        .value()
}

fn pressure(engine: &Engine, name: &str) -> f64 {
    engine
        .snapshot()
        .nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("the plant declares a node '{name}'"))
        .pressure_pa
}

// ------------------------------------------------------------------ gate 1

const TICKS: u64 = 6_000;
/// The last tick on which the disc is shut, measured.
const LAST_SHUT: u64 = 300;

struct Run {
    flow: Vec<f64>,
    receiver: Vec<f64>,
}

fn run(label: &str, src: &str) -> Run {
    let mut engine = build(src);
    let mut out = Run {
        flow: Vec::new(),
        receiver: Vec::new(),
    };
    for t in 1..=TICKS {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("{label}: the plant must run: tick {t}: {e}"));
        out.flow.push(edge_flow(&engine, "check_outlet"));
        out.receiver.push(pressure(&engine, "receiver"));
    }
    out
}

#[test]
fn the_disc_keeps_the_receiver_out_of_its_header() {
    for fidelity in FIDELITIES {
        let disc = run(fidelity, &on(fidelity, DEMO));
        let twin = run(fidelity, &on(fidelity, &swap(DEMO, CHECK_DECL, PLAIN_DECL)));
        let at = |v: &[f64], t: u64| v[(t - 1) as usize];

        // Never backwards, and exactly nothing while the receiver is above what
        // the header can push.
        for (i, &f) in disc.flow.iter().enumerate() {
            let t = i as u64 + 1;
            assert!(
                f >= 0.0,
                "{fidelity}: the disc passed {f} kg/s backwards at tick {t}"
            );
            if t <= LAST_SHUT {
                assert_eq!(f, 0.0, "{fidelity}: the disc is shut at tick {t}");
            } else {
                assert!(f > 0.0, "{fidelity}: the disc has reopened at tick {t}");
            }
        }

        // The twin runs backwards into the header, and the receiver it leaves is
        // lower for it.
        let twin_worst = twin.flow.iter().copied().fold(f64::INFINITY, f64::min);
        let backwards = twin.flow.iter().filter(|f| **f < 0.0).count();
        eprintln!(
            "MEASURE {fidelity}: twin worst {twin_worst} kg/s over {backwards} ticks; receiver at \
             100 disc {} twin {}; at {TICKS} disc {} twin {}",
            at(&disc.receiver, 100),
            at(&twin.receiver, 100),
            at(&disc.receiver, TICKS),
            at(&twin.receiver, TICKS)
        );
        assert!(
            twin_worst < -0.4 && backwards > 150,
            "{fidelity}: the twin blows the receiver back into the header, worst {twin_worst} \
             kg/s over {backwards} ticks"
        );
        assert!(
            at(&disc.receiver, 100) - at(&twin.receiver, 100) > 2.0e5,
            "{fidelity}: the disc keeps the receiver's gas while the twin loses it"
        );

        // One answer at the settle: at full lift the disc is that valve.
        let gap = |t: u64| (at(&disc.receiver, t) - at(&twin.receiver, t)).abs();
        assert!(
            gap(TICKS) < 1e-4 * at(&twin.receiver, TICKS) && gap(TICKS) < 0.1 * gap(3_000),
            "{fidelity}: the two settle together, gap {} Pa at {TICKS} and {} Pa at 3000",
            gap(TICKS),
            gap(3_000)
        );
    }
}

// ------------------------------------------------------------------ gate 2

/// A disc between a pinned header and a sink, gas, a short wide tail so the
/// valve takes the drop and can be driven to its choke.
const DISC: &str = r#"
[meta]
name = "gas_check_slope"
[simulation]
dt = 0.1
[fidelity]
flow = "newton"
thermo = "constant"
reactions = "none"

[[components]]
name = "fuel_gas"
phase = "gas"
tb_c = -161.5
molar_mass_kg_per_mol = 0.016043
cp_j_per_kg_k = 2220.0

[nodes.header]
type = "source"
pressure_bar = 20.0
temperature_c = 20.0

[nodes.disc]
type = "check_valve"
kv = 12.0
full_open_bar = 0.1
x_t = 0.72

[nodes.drain]
type = "sink"
pressure_bar = 2.0
temperature_c = 20.0

[[pipes]]
name = "riser"
from = "header"
to = "disc"
length_m = 5.0
diameter_m = 0.05

[[pipes]]
name = "tail"
from = "disc"
to = "drain"
length_m = 1.0
diameter_m = 0.05
"#;

const DISC_DECL: &str = "type = \"check_valve\"\nkv = 12.0\nfull_open_bar = 0.1\nx_t = 0.72";

/// The fixture's tail compiled with the disc at `p_disc` and the drain at
/// `p_drain`; the edge and its raw drop.
fn tail(engine: &Engine, p_disc: f64, p_drain: f64) -> (CompiledEdge, f64) {
    let graph = &engine.graph;
    let mut p: BTreeMap<NodeId, f64> = graph.node_ids().map(|n| (n, p_drain)).collect();
    p.insert(node(graph, "header"), p_disc + 1.0e5);
    p.insert(node(graph, "disc"), p_disc);
    let c = compile_edge(
        graph,
        pipe(graph, "tail"),
        &engine.slate,
        &NodeStates::default(),
        &p,
    )
    .expect("the tail compiles");
    (c, p_disc - p_drain)
}

fn tail_flow(engine: &Engine, p_disc: f64, p_drain: f64) -> f64 {
    let eps = NewtonFlowSolver::default().eps_dp;
    let (c, dp) = tail(engine, p_disc, p_drain);
    c.rho * c.branch.flow(dp, eps)
}

/// The same fixture with a plain valve at `opening` where the disc is.
fn plain_at(src: &str, decl: &str, opening: f64, x_t: &str) -> Engine {
    let kv = decl
        .lines()
        .find_map(|l| l.strip_prefix("kv = "))
        .expect("the declaration has a kv");
    build(&swap(
        src,
        decl,
        &format!("type = \"valve\"\nkv = {kv}\nopening = {opening:?}\nx_t = {x_t}"),
    ))
}

/// The tail's branch with the fold left out: the incompressible law on the
/// gas's own density, the fault gate 2 must be able to see.
fn incompressible(engine: &Engine, c: &CompiledEdge) -> QuadraticBranch {
    let graph = &engine.graph;
    let tail = graph.pipe(pipe(graph, "tail"));
    let NodeKind::CheckValve { cv_max, .. } = graph.node(node(graph, "disc")).kind else {
        panic!("the fixture's disc is a check valve")
    };
    let k = pipe_resistance(
        tail.friction_factor,
        tail.length.value(),
        tail.diameter.value(),
        c.rho,
    );
    QuadraticBranch::pipe(k, c.branch.beta).in_series(QuadraticBranch::valve(
        cv_max,
        1.0,
        c.rho / RHO_WATER_REF,
    ))
}

#[test]
fn at_full_lift_it_is_a_plain_gas_valve_and_the_fold_is_in_it() {
    // The tail RISES, so the branch carries a static head and the fold's own
    // `|dp − β|` is not the disc's drive `dp − β` read twice: handing the fold
    // the drive instead of the drop is invisible on a level gas line, where
    // `β` is a few pascals (M31's mutation 6, uncaught until this rise).
    let risen = swap(
        DISC,
        "length_m = 1.0
diameter_m = 0.05
",
        "length_m = 1.0
diameter_m = 0.05
elevation_change_m = 30.0
",
    );
    let engine = build(&risen);
    let plain = plain_at(&risen, DISC_DECL, 1.0, "0.72");
    let eps = NewtonFlowSolver::default().eps_dp;

    // (p_disc, p_drain): a 3 bar drop off 20 bar, where `Y` is visibly below
    // one, and a drop past the choke.
    for (label, p_disc, p_drain) in [("unchoked", 20.0e5, 17.0e5), ("choked", 10.0e5, 2.0e5)] {
        let (disc, dp) = tail(&engine, p_disc, p_drain);
        let (valve, _) = tail(&plain, p_disc, p_drain);
        assert_eq!(disc.check_opening_log_slope, 0.0, "{label}: full lift");
        assert!(
            disc.branch.beta > 1.0e3,
            "{label}: the tail's static head is not a rounding term, β = {} Pa",
            disc.branch.beta
        );
        assert_eq!(
            disc.branch.alpha.to_bits(),
            valve.branch.alpha.to_bits(),
            "{label}: the disc's branch is a plain gas valve's at opening 1"
        );
        assert_eq!(disc.branch.beta.to_bits(), valve.branch.beta.to_bits());
        assert_eq!(
            disc.conductance(dp, eps).to_bits(),
            (disc.rho * disc.branch.flow_ddp(dp, eps)).to_bits(),
            "{label}: no opening share above the band"
        );

        // The control: the incompressible branch is a different number, so a
        // disc that skipped the fold could not pass the identity above.
        let liquid = incompressible(&engine, &disc);
        let gas_flow = disc.rho * disc.branch.flow(dp, eps);
        let liquid_flow = disc.rho * liquid.flow(dp, eps);
        eprintln!(
            "MEASURE {label}: gas {gas_flow:e} kg/s against incompressible {liquid_flow:e} \
             (ratio {})",
            gas_flow / liquid_flow
        );
        assert!(
            gas_flow < (1.0 - 1e-3) * liquid_flow,
            "{label}: the fold is in the disc's branch, {gas_flow:e} against {liquid_flow:e}"
        );
    }

    // The choked point really is choked: the flow does not rise as the drain
    // falls further, where the incompressible law's keeps rising. The plateau's
    // bound is the `eps_dp` regularisation's (measured 3.7e-8 relative, the
    // O(eps/Δp) shortfall `invariants.rs`'s choke detector records), not zero.
    let (disc_choked, dp_choked) = tail(&engine, 10.0e5, 2.0e5);
    let (_, dp_deeper) = tail(&engine, 10.0e5, 1.5e5);
    let choked = tail_flow(&engine, 10.0e5, 2.0e5);
    let deeper = tail_flow(&engine, 10.0e5, 1.5e5);
    let liquid = incompressible(&engine, &disc_choked);
    let liquid_rise = liquid.flow(dp_deeper, eps) / liquid.flow(dp_choked, eps) - 1.0;
    eprintln!(
        "MEASURE plateau: gas rise {:e}, incompressible rise {liquid_rise:e}",
        deeper / choked - 1.0
    );
    assert!(
        ((deeper - choked) / choked).abs() < 1e-6 && liquid_rise > 1e-2,
        "past the choke the disc's flow is a plateau: {choked:e} then {deeper:e}, where the          incompressible law's rises by {liquid_rise:e}"
    );
}

// ------------------------------------------------------------------ gate 3

/// Gate 2's fixture driven to its choke INSIDE the band: a low-pressure line,
/// a small `x_t` and a wide band, so the valve reaches `x_choke` of its inlet
/// before the disc reaches full lift. The random arm visits this corner once
/// in 400 samples, which is why it has a fixture.
fn low_pressure() -> (String, &'static str) {
    let decl = "type = \"check_valve\"\nkv = 12.0\nfull_open_bar = 0.5\nx_t = 0.2";
    let src = swap(
        &swap(DISC, DISC_DECL, decl),
        "pressure_bar = 2.0",
        "pressure_bar = 0.5",
    );
    (
        swap(&src, "pressure_bar = 20.0", "pressure_bar = 1.2"),
        decl,
    )
}

/// The opening's share of the slope against the true share, at `fraction` of
/// the band below `p_disc`. Returns `(model, true, centred disc, choked)`.
fn opening_share(
    src: &str,
    decl: &str,
    x_t: &str,
    p_disc: f64,
    full_open: f64,
    fraction: f64,
) -> (f64, f64, f64, bool) {
    let engine = build(src);
    let eps = NewtonFlowSolver::default().eps_dp;
    let drive = fraction * full_open;
    let p_drain = p_disc - drive;
    let (disc, dp) = tail(&engine, p_disc, p_drain);
    let op = check_opening(drive, full_open);
    let plain = plain_at(src, decl, op, x_t);
    let (valve, _) = tail(&plain, p_disc, p_drain);

    // Inside the band the disc IS the plain valve at its own opening.
    assert_eq!(
        disc.branch.alpha.to_bits(),
        valve.branch.alpha.to_bits(),
        "at {fraction} of the band the disc's branch is a valve's at opening {op}"
    );
    assert!(disc.check_opening_log_slope > 0.0, "the share is live");

    // Moving the DRAIN moves the drive and leaves the upwind state alone, so
    // the density and the fold's `p_up` hold still and the centred difference
    // sees only the drop.
    let h = 1e-3 * full_open;
    let centred = |e: &Engine| {
        (tail_flow(e, p_disc, p_drain - h) - tail_flow(e, p_disc, p_drain + h)) / (2.0 * h)
    };
    let centred_disc = centred(&engine);
    let centred_plain = centred(&plain);
    let frozen = disc.rho * disc.branch.flow_ddp(dp, eps);
    let model = disc.conductance(dp, eps) - frozen;
    let truth = centred_disc - centred_plain;
    // Choked: the plain valve's flow has stopped answering the drain while its
    // frozen slope still says it should — the plateau, seen as a slope.
    let choked = centred_plain.abs() < 1e-3 * frozen;
    eprintln!(
        "MEASURE p_disc {p_disc} band {fraction}: share model {model:e} true {truth:e} \
         (relative {:e}); centred disc {centred_disc:e} plain {centred_plain:e} frozen {frozen:e} \
         (the fold's own frozen-x error {:e}); choked {choked}",
        (model - truth).abs() / truth,
        (frozen - centred_plain) / centred_plain.abs().max(f64::MIN_POSITIVE)
    );
    (model, truth, centred_disc, choked)
}

#[test]
fn the_opening_share_is_right_in_gas_unchoked_and_choked() {
    // Unchoked: the demo's 20 bar, where the gas valve is nearly its liquid self.
    for fraction in [0.2, 0.5, 0.8] {
        let (model, truth, centred, choked) =
            opening_share(DISC, DISC_DECL, "0.72", 20.0e5, 0.1e5, fraction);
        assert!(!choked, "at 20 bar a 0.1 bar band does not choke");
        assert!(
            (model - truth).abs() < 1e-3 * truth,
            "unchoked, {fraction} of the band: share {model:e} against {truth:e}"
        );
        assert!(
            truth > 0.5 * centred,
            "the share is most of the slope inside the band: {truth:e} of {centred:e}"
        );
    }

    // Choked inside the band.
    let (src, decl) = low_pressure();
    let mut reached = 0;
    for fraction in [0.5, 0.8] {
        let (model, truth, _, choked) = opening_share(&src, decl, "0.2", 1.2e5, 0.5e5, fraction);
        reached += usize::from(choked);
        assert!(
            (model - truth).abs() < 2e-2 * truth,
            "choked, {fraction} of the band: share {model:e} against {truth:e}"
        );
    }
    assert!(
        reached > 0,
        "the low-pressure fixture reaches its choke inside the band"
    );
}

#[test]
fn a_shut_gas_disc_carries_nothing_backwards() {
    let engine = build(DISC);
    let eps = NewtonFlowSolver::default().eps_dp;
    for (p_disc, p_drain) in [(18.0e5, 20.0e5), (2.0e5, 10.0e5), (5.0e5, 5.0e5)] {
        let (c, dp) = tail(&engine, p_disc, p_drain);
        assert!(!c.conducts, "a gas disc with no forward drive is shut");
        assert_eq!(c.check_opening_log_slope, 0.0);
        assert_eq!(c.rho * c.branch.flow(dp, eps), 0.0, "and passes nothing");
    }
}

// ------------------------------------------------------------------ gate 4

#[test]
fn every_gas_refusal_names_its_own_reason() {
    let liquid_demo = include_str!("../../../scenarios/tank_level_fill_check_valve.toml");
    let shaped = include_str!("../../../scenarios/fired_gas_drum.toml");
    let shaped_with_disc = swap(
        &swap(
            shaped,
            "[nodes.surge_drum]",
            "[nodes.disc]\ntype = \"check_valve\"\nkv = 12.0\nfull_open_bar = 0.1\nx_t = \
             0.72\n\n[nodes.surge_drum]",
        ),
        "from = \"heater\"\nto = \"surge_drum\"",
        "from = \"heater\"\nto = \"disc\"\nlength_m = 1.0\ndiameter_m = 0.05\n\n[[pipes]]\nname \
         = \"disc_outlet\"\nfrom = \"disc\"\nto = \"surge_drum\"",
    );
    let cases: Vec<(&str, String, &str)> = vec![
        (
            "a gas disc with no x_t",
            swap(DEMO, "\nx_t = 0.72\n\n# Charged", "\n\n# Charged"),
            "'make_up_check' is in gas service and must declare `x_t`",
        ),
        (
            "a liquid disc declaring x_t",
            swap(
                liquid_demo,
                "full_open_bar = 0.015",
                "full_open_bar = 0.015\nx_t = 0.72",
            ),
            "is in liquid service and declares `x_t = 0.72`",
        ),
        (
            "an x_t above one",
            swap(
                DEMO,
                CHECK_DECL,
                &CHECK_DECL.replace("x_t = 0.72", "x_t = 1.2"),
            ),
            "'make_up_check' has x_t = 1.2, outside (0, 1)",
        ),
        (
            "an x_t of zero",
            swap(
                DEMO,
                CHECK_DECL,
                &CHECK_DECL.replace("x_t = 0.72", "x_t = 0.0"),
            ),
            "'make_up_check' has x_t = 0, outside (0, 1)",
        ),
        (
            "a shaped heat capacity with a gas disc",
            shaped_with_disc,
            "heat_capacity = \"linear\" with valve 'disc' declaring x_t",
        ),
    ];
    for (what, src, needle) in cases {
        let message = refusal(what, &src);
        assert!(
            message.contains(needle),
            "{what}: refused for its own reason, expected `{needle}` in: {message}"
        );
    }
}

// ------------------------------------------------------------------ gate 5

#[test]
fn the_demo_never_comes_near_the_choke() {
    let mut engine = build(DEMO);
    let outlet = pipe(&engine.graph, "check_outlet");
    let pipe_def = engine.graph.pipe(outlet).clone();
    let comp = &pipe_def.stream.composition;
    let gamma = comp.mixture_cp(&engine.slate).value() / comp.mixture_cv(&engine.slate).value();
    let x_choke = specific_heat_ratio_factor(gamma) * 0.72;
    let eps = NewtonFlowSolver::default().eps_dp;
    let mut worst: f64 = 0.0;
    for t in 1..=TICKS {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        let snap = engine.snapshot();
        let p = |n: NodeId| snap.nodes[n.0 as usize].pressure_pa;
        let (src, tgt) = engine.graph.endpoints(outlet);
        let pressures: BTreeMap<NodeId, f64> = engine.graph.node_ids().map(|n| (n, p(n))).collect();
        let c = compile_edge(
            &engine.graph,
            outlet,
            &engine.slate,
            &NodeStates::default(),
            &pressures,
        )
        .expect("the disc's edge compiles");
        let drive = p(src) - p(tgt) - c.branch.beta;
        if drive <= 0.0 {
            continue;
        }
        // The valve's own share of the drive: the whole, less the spool's
        // friction at the flow the branch carries.
        let q = c.branch.flow(p(src) - p(tgt), eps);
        let k = pipe_resistance(
            pipe_def.friction_factor,
            pipe_def.length.value(),
            pipe_def.diameter.value(),
            c.rho,
        );
        let own = drive - k * q * q;
        worst = worst.max(own / p(src) / x_choke);
    }
    eprintln!("MEASURE the disc's own x/x_choke peaks at {worst}");
    assert!(
        worst < 0.1,
        "the demo stays on the Y → 1 tail, x/x_choke ≤ {worst}"
    );
}

// ------------------------------------------------------------------ gate 6

/// The kind's `x_t` on the wire: absent on a liquid disc, so M30's demo
/// publishes the bytes it always did, and present on a gas one. Asserted on the
/// serialized snapshot, because a Rust match on the field passes whatever the
/// serde attribute says (M31's mutation 5, uncaught until this gate).
#[test]
fn a_disc_publishes_x_t_only_in_gas_service() {
    let disc_kind = |src: &str, name: &str| {
        let engine = build(src);
        let json = serde_json::to_value(engine.snapshot()).expect("a snapshot serializes");
        json["nodes"]
            .as_array()
            .expect("nodes")
            .iter()
            .find(|n| n["name"] == name)
            .unwrap_or_else(|| panic!("the plant declares '{name}'"))["kind"]
            .clone()
    };
    let liquid = disc_kind(
        include_str!("../../../scenarios/tank_level_fill_check_valve.toml"),
        "discharge_check",
    );
    assert_eq!(liquid["type"], "check_valve");
    assert!(
        liquid.get("x_t").is_none(),
        "a liquid disc publishes no x_t key: {liquid}"
    );
    let gas = disc_kind(DEMO, "make_up_check");
    assert_eq!(gas["x_t"], 0.72, "a gas disc publishes its x_t: {gas}");
}
