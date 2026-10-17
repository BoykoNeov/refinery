//! M62.6b — a valve on an all-vapour stream chokes (docs/DESIGN.md §67.19):
//! Leung's ω jet, the burst hole's law (M62.4d), on the valve's own share of
//! its branch's drop. The fold's algebra is gated in `elements.rs`; here a
//! steam supply through a valve into a sink at falling pressure, through the
//! loader and both solvers: the flow rises to the choke and holds there, and
//! the choked flow is the law's at the state the solve read.

use refinery_core::graph::NodeKind;
use refinery_core::snapshot::Snapshot;
use refinery_core::Engine;
use refinery_solvers::elements::QuadraticBranch;
use refinery_solvers::network::RHO_WATER_REF;

/// A 10 bar, 400 °C steam supply through a valve into a sink at
/// `sink_bar`, on short wide pipes so the valve takes the drop.
fn steam_line(solver: &str, sink_bar: f64) -> String {
    format!(
        r#"
[meta]
name = "steam_valve"
description = "M62.6b gate: a steam valve that chokes."

[simulation]
dt = 0.1

[[components]]
name = "water"
tb_c = 100.0
molar_mass_kg_per_mol = 0.018015
density_kg_per_m3 = 998.0
cp_j_per_kg_k = 4184.0
dh_vap_j_per_mol = 40650.0
cp_vapour_j_per_kg_k = 2000.0

[fidelity]
flow = "{solver}"
thermo = "trouton"
line_flash = "equilibrium"

[nodes.steam_supply]
type = "source"
pressure_bar = 10.0
temperature_c = 400.0
composition = {{ water = 1.0 }}

[nodes.steam_valve]
type = "valve"
kv = 20.0
opening = 0.5

[nodes.header]
type = "sink"
pressure_bar = {sink_bar}
composition = {{ water = 1.0 }}

[[pipes]]
name = "supply_line"
from = "steam_supply"
to = "steam_valve"
length_m = 2.0
diameter_m = 0.3

[[pipes]]
name = "steam_line"
from = "steam_valve"
to = "header"
length_m = 2.0
diameter_m = 0.3
"#
    )
}

fn settled(solver: &str, sink_bar: f64) -> Engine {
    let src = steam_line(solver, sink_bar);
    let mut engine =
        refinery_scenarios::build_engine(&refinery_scenarios::load_str(&src).unwrap()).unwrap();
    for t in 1..=60 {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("{solver} at {sink_bar} bar: tick {t}: {e}"));
        assert!(engine.snapshot().solver.converged, "{solver}: tick {t}");
    }
    engine
}

fn flow(s: &Snapshot) -> f64 {
    s.edges
        .iter()
        .find(|e| e.name == "steam_line")
        .unwrap()
        .stream
        .mass_flow
        .value()
}

/// **The flow rises to the choke and holds there**: past a drop of about
/// `1 − e^{−1/2}` of the valve's inlet pressure (the ω law's critical ratio for
/// steam, all vapour), lowering the sink adds no steam — where the liquid law
/// the valve had before M62.6b went on rising as `√Δp`.
#[test]
fn a_steam_valve_chokes_and_holds_its_flow() {
    for solver in ["newton", "simple"] {
        // 10 bar in: unchoked at 9, 8 and 7 bar out, choked from 6 bar down.
        let flows: Vec<f64> = [9.0, 8.0, 7.0, 6.0, 3.0, 1.5]
            .iter()
            .map(|sink| flow(&settled(solver, *sink).snapshot()))
            .collect();
        for pair in flows[..3].windows(2) {
            assert!(
                pair[1] > pair[0] * 1.01,
                "{solver}: unchoked, a lower sink must pass more: {flows:?}"
            );
        }
        let choked = flows[3];
        for f in &flows[4..] {
            assert!(
                (f - choked).abs() < 1e-5 * choked,
                "{solver}: choked, the flow holds: {flows:?}"
            );
        }
    }
}

/// **The choked flow is the ω law's at the state the solve read**: the valve's
/// liquid branch `α_v` (its Kv, its opening, the inlet's density) and the
/// choked jet, `ṁ = ρ₀·e^{−1/2}·√(P₀/(2·α_v))` — `ω = 1` for steam all vapour,
/// whatever share of the drop the pipe takes once the valve is past its choke.
/// The inlet's density is the ideal gas's, as the line flash's vapour is.
#[test]
fn the_choked_steam_flow_is_the_omega_laws() {
    for solver in ["newton", "simple"] {
        let engine = settled(solver, 1.5);
        let s = engine.snapshot();
        let valve = s.nodes.iter().find(|n| n.name == "steam_valve").unwrap();
        let (p0, t0) = (valve.pressure_pa, valve.temperature_k);
        let rho0 = p0 * 0.018015 / (8.314_462_618 * t0);
        let (cv, opening) = match &valve.kind {
            NodeKind::Valve {
                cv_max, opening, ..
            } => (*cv_max, *opening),
            _ => panic!("the steam valve is a valve"),
        };
        let alpha = QuadraticBranch::valve(cv, opening, rho0 / RHO_WATER_REF).alpha;
        let want = rho0 * (-0.5f64).exp() * (p0 / (2.0 * alpha)).sqrt();
        let got = flow(&s);
        assert!(
            (got / want - 1.0).abs() < 1e-6,
            "{solver}: {got} kg/s of steam, the ω law's {want}"
        );
    }
}

/// **A check valve chokes on steam too**: the valve kinds share the one fold
/// (`fold_gas_service`), so a check valve whose disc is full open holds its
/// flow past the choke as a control valve does. (A relief valve shares it as
/// well, and is refused on a plant with the line flash, §58.)
#[test]
fn a_check_valve_chokes_on_steam() {
    let valve = "[nodes.steam_valve]
type = \"valve\"
kv = 20.0
opening = 0.5
";
    let check = "[nodes.steam_valve]
type = \"check_valve\"
kv = 10.0
full_open_bar = 0.1
";
    let flows: Vec<f64> = [3.0, 1.5]
        .iter()
        .map(|sink| {
            let src = steam_line("newton", *sink).replace(valve, check);
            assert!(src.contains(check), "the fixture's valve was replaced");
            let mut engine =
                refinery_scenarios::build_engine(&refinery_scenarios::load_str(&src).unwrap())
                    .unwrap();
            for t in 1..=60 {
                engine
                    .tick()
                    .unwrap_or_else(|e| panic!("at {sink} bar: tick {t}: {e}"));
            }
            flow(&engine.snapshot())
        })
        .collect();
    assert!(flows[0] > 0.05, "{flows:?}");
    assert!(
        (flows[1] - flows[0]).abs() < 1e-5 * flows[0],
        "choked, the flow holds: {flows:?}"
    );
}

/// **A move across the choke settles in a handful of iterations** (M62.6c,
/// docs/DESIGN.md §67.20): the header dropped from 9 bar (unchoked) to 2
/// (choked) in one command. Choked, the valve's flow no longer moves with the
/// header, where its frozen branch says it moves as the √-law's; Newton reads
/// the jet's own slope (`ChokeSlope`), and the game solver's node step too.
#[test]
fn a_move_across_the_choke_settles_quickly() {
    for solver in ["newton", "simple"] {
        let mut engine = settled(solver, 9.0);
        let header = engine.graph.find_node("header").unwrap();
        engine
            .apply(refinery_core::snapshot::Command::SetReservoirPressure {
                node: header,
                pressure: refinery_core::units::Pascal(2.0e5),
            })
            .unwrap();
        let mut worst = 0;
        for t in 1..=20 {
            engine
                .tick()
                .unwrap_or_else(|e| panic!("{solver}: tick {t}: {e}"));
            let s = engine.snapshot();
            assert!(s.solver.converged, "{solver}: tick {t}");
            worst = worst.max(s.solver.iterations);
        }
        let choked = flow(&settled(solver, 1.5).snapshot());
        let got = flow(&engine.snapshot());
        assert!(
            (got - choked).abs() < 1e-5 * choked,
            "{solver}: {got} against {choked}"
        );
    }
}

/// The steam line with a junction past the valve and a long line on to the
/// header: the valve's downstream end an unknown of the solve.
fn steam_line_to_junction(solver: &str, sink_bar: f64) -> String {
    steam_line(solver, sink_bar)
        .replace(
            "[[pipes]]\nname = \"steam_line\"\nfrom = \"steam_valve\"\nto = \"header\"\nlength_m = 2.0\ndiameter_m = 0.3\n",
            "[nodes.manifold]\ntype = \"junction\"\n\n[[pipes]]\nname = \"steam_line\"\nfrom = \"steam_valve\"\nto = \"manifold\"\nlength_m = 2.0\ndiameter_m = 0.3\n\n[[pipes]]\nname = \"header_line\"\nfrom = \"manifold\"\nto = \"header\"\nlength_m = 200.0\ndiameter_m = 0.08\n",
        )
}

/// **The choking valve's slope is in the solve** (M62.6c, docs/DESIGN.md
/// §67.20): the valve into a manifold, a long line on to the header, the
/// header dropped from 9 bar to 1.2 in one command. Choked, the valve's flow
/// stops moving with the manifold's pressure, where its frozen branch says it
/// moves as the √-law's. Measured on the first tick after the move: 5 Newton
/// iterations and 5 game-solver sweeps with the jet's slope, 8 and 7 without.
#[test]
fn the_choking_valves_slope_shortens_the_solve() {
    for solver in ["newton", "simple"] {
        let src = steam_line_to_junction(solver, 9.0);
        assert!(src.contains("manifold"), "the fixture's line was replaced");
        let mut engine =
            refinery_scenarios::build_engine(&refinery_scenarios::load_str(&src).unwrap()).unwrap();
        for _ in 0..30 {
            engine.tick().unwrap();
        }
        let header = engine.graph.find_node("header").unwrap();
        engine
            .apply(refinery_core::snapshot::Command::SetReservoirPressure {
                node: header,
                pressure: refinery_core::units::Pascal(1.2e5),
            })
            .unwrap();
        let mut worst = 0;
        for t in 1..=30 {
            engine
                .tick()
                .unwrap_or_else(|e| panic!("{solver}: tick {t}: {e}"));
            let s = engine.snapshot();
            assert!(s.solver.converged, "{solver}: tick {t}");
            worst = worst.max(s.solver.iterations);
        }
        assert!(worst <= 6, "{solver}: {worst} iterations a tick");
    }
}
