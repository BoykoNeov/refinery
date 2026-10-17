//! M62.6b — a valve on an all-vapour stream chokes (docs/DESIGN.md §67.19);
//! M63.3 — as an ideal gas (§68.3, ledger row B67): the ISA gas law on the
//! valve's declared `x_t`, an ideal nozzle without one, and a hole the
//! isentropic nozzle a gas hole is. Until M63.3 all three were Leung's ω jet
//! on two isenthalpic flashes, an isothermal gas, 9 % under the nozzle. The
//! folds' algebra is gated in `elements.rs`; here a steam supply through a
//! valve into a sink at falling pressure, through the loader and both solvers:
//! the flow rises to the choke and holds there, and the choked flow is the
//! law's at the state the solve read.

use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::units::SquareMeter;
use refinery_core::Engine;
use refinery_solvers::elements::{QuadraticBranch, ORIFICE_CD};
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

/// Steam's heat-capacity ratio on the slate: `c_p,v/(c_p,v − R/M)`, water
/// declaring 2 000 J/(kg·K) — 1.30.
fn steam_gamma() -> f64 {
    2000.0 / (2000.0 - 8.314_462_618 / 0.018015)
}

/// The isentropic flow function at the choke, `ψ* = γ/(γ−1)·(r^{2/γ} −
/// r^{(γ+1)/γ})`, `r = (2/(γ+1))^{γ/(γ−1)}` — written here from the textbook,
/// not from `elements.rs`.
fn psi_choked(gamma: f64) -> f64 {
    let r = (2.0 / (gamma + 1.0)).powf(gamma / (gamma - 1.0));
    gamma / (gamma - 1.0) * (r.powf(2.0 / gamma) - r.powf((gamma + 1.0) / gamma))
}

/// The valve's inlet state and liquid branch at the solved state: `(ρ₀, P₀,
/// α_v, x_t)`, the inlet's density the ideal gas's, as the line flash's
/// vapour is.
fn valve_inlet(s: &Snapshot) -> (f64, f64, f64, Option<f64>) {
    let valve = s.nodes.iter().find(|n| n.name == "steam_valve").unwrap();
    let (p0, t0) = (valve.pressure_pa, valve.temperature_k);
    let rho0 = p0 * 0.018015 / (8.314_462_618 * t0);
    let (cv, opening, x_t) = match &valve.kind {
        NodeKind::Valve {
            cv_max,
            opening,
            x_t,
        } => (*cv_max, *opening, *x_t),
        _ => panic!("the steam valve is a valve"),
    };
    let alpha = QuadraticBranch::valve(cv, opening, rho0 / RHO_WATER_REF).alpha;
    (rho0, p0, alpha, x_t)
}

/// The fixture with the valve declaring `x_t`.
fn with_x_t(src: &str, x_t: f64) -> String {
    let out = src.replace("opening = 0.5\n", &format!("opening = 0.5\nx_t = {x_t}\n"));
    assert!(out.contains("x_t"), "the valve declares x_t");
    out
}

fn settled_src(src: &str) -> Engine {
    let mut engine =
        refinery_scenarios::build_engine(&refinery_scenarios::load_str(src).unwrap()).unwrap();
    for t in 1..=60 {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        assert!(engine.snapshot().solver.converged, "tick {t}");
    }
    engine
}

/// **The flow rises to the choke and holds there**: past a drop of
/// `1 − (2/(γ+1))^{γ/(γ−1)}` = 45.4 % of the valve's inlet pressure (an ideal
/// nozzle on steam, `γ = 1.30`), lowering the sink adds no steam — where the
/// liquid law the valve had before M62.6b went on rising as `√Δp`.
#[test]
fn a_steam_valve_chokes_and_holds_its_flow() {
    for solver in ["newton", "simple"] {
        // 10 bar in: unchoked at 9, 8, 7 and 6 bar out, choked from 4 bar down.
        let flows: Vec<f64> = [9.0, 8.0, 7.0, 6.0, 4.0, 3.0, 1.5]
            .iter()
            .map(|sink| flow(&settled(solver, *sink).snapshot()))
            .collect();
        for pair in flows[..4].windows(2) {
            assert!(
                pair[1] > pair[0] * 1.01,
                "{solver}: unchoked, a lower sink must pass more: {flows:?}"
            );
        }
        let choked = flows[4];
        for f in &flows[5..] {
            assert!(
                (f - choked).abs() < 1e-5 * choked,
                "{solver}: choked, the flow holds: {flows:?}"
            );
        }
    }
}

/// **Without `x_t` the choked flow is an ideal nozzle's at the state the solve
/// read** (Saint-Venant–Wantzel): `ṁ = ρ₀·√(ψ*·P₀/α_v)` — the liquid law
/// `ρ₀·√(Δp/α_v)` scaled by `√(ψ/x)` at the choke — whatever share of the drop
/// the pipe takes once the valve is past it. M62.6b's ω law passed
/// `ρ₀·e^{−1/2}·√(P₀/(2α_v))`, 9 % less.
#[test]
fn the_choked_steam_flow_is_the_nozzles() {
    for solver in ["newton", "simple"] {
        let s = settled(solver, 1.5).snapshot();
        let (rho0, p0, alpha, x_t) = valve_inlet(&s);
        assert_eq!(x_t, None);
        let want = rho0 * (psi_choked(steam_gamma()) * p0 / alpha).sqrt();
        let got = flow(&s);
        assert!(
            (got / want - 1.0).abs() < 1e-6,
            "{solver}: {got} kg/s of steam, the nozzle's {want}"
        );
        let omega = rho0 * (-0.5f64).exp() * (p0 / (2.0 * alpha)).sqrt();
        assert!(
            got / omega - 1.0 > 0.09,
            "{solver}: the nozzle {got} against the old ω law's {omega}"
        );
    }
}

/// **On its declared `x_t` the valve is the ISA gas law** (IEC 60534-2-1, the
/// law a gas valve runs, M5.4): choked past `x = F_k·x_T` of its inlet
/// pressure, `F_k = γ/1.4`, at `ṁ = ρ₀·(2/3)·√(F_k·x_T·P₀/α_v)`. A globe
/// valve's `x_T = 0.72` chokes at a drop of 67 %, LATER than the nozzle's
/// 45 %, and passes some 15 % more: its pressure recovery.
#[test]
fn a_steam_valve_on_its_x_t_is_the_isa_gas_law() {
    for solver in ["newton", "simple"] {
        let flows: Vec<f64> = [5.0, 4.0, 2.5, 1.5]
            .iter()
            .map(|sink| {
                let src = with_x_t(&steam_line(solver, *sink), 0.72);
                flow(&settled_src(&src).snapshot())
            })
            .collect();
        assert!(
            flows[1] > flows[0] * 1.01,
            "{solver}: at a 60 % drop the globe valve is not yet choked: {flows:?}"
        );
        assert!(
            (flows[3] - flows[2]).abs() < 1e-5 * flows[2],
            "{solver}: choked, the flow holds: {flows:?}"
        );
        let s = settled_src(&with_x_t(&steam_line(solver, 1.5), 0.72)).snapshot();
        let (rho0, p0, alpha, x_t) = valve_inlet(&s);
        let x_choke = steam_gamma() / 1.4 * x_t.unwrap();
        let want = rho0 * (2.0 / 3.0) * (x_choke * p0 / alpha).sqrt();
        let got = flow(&s);
        assert!(
            (got / want - 1.0).abs() < 1e-6,
            "{solver}: {got} kg/s of steam, the ISA law's {want}"
        );
        let nozzle = flow(&settled(solver, 1.5).snapshot());
        assert!(
            got / nozzle - 1.0 > 0.10,
            "{solver}: the globe valve {got} against the nozzle's {nozzle}"
        );
    }
}

/// **A hole in a steam line is the isentropic nozzle a gas hole is** (M37's
/// law, docs/DESIGN.md §41; M63.3): punctured, the 10 bar steam leaks to the
/// atmosphere at `ṁ = C_d·A·√(2·ρ₀·P₀·ψ*)` at the hole's upwind state — the
/// boiling jet's ω law took steam as an isothermal gas, 9 % short.
#[test]
fn a_hole_in_a_steam_line_is_the_isentropic_nozzle() {
    const HOLE: f64 = 1.0e-4; // m²
    let line = "name = \"supply_line\"\nfrom = \"steam_supply\"\nto = \"steam_valve\"\n\
                length_m = 2.0\ndiameter_m = 0.3\n";
    let src = steam_line("newton", 1.5).replace(
        line,
        &format!("{line}leak_to = \"outside\"\n\n[nodes.outside]\ntype = \"atmosphere\"\n"),
    );
    assert!(src.contains("leak_to"), "the fixture's line was replaced");
    let mut engine = settled_src(&src);
    let pipe = engine
        .graph
        .edge_ids()
        .find(|&e| engine.graph.pipe(e).name == "supply_line")
        .unwrap();
    engine
        .apply(Command::PuncturePipe {
            edge: pipe,
            area: SquareMeter(HOLE),
        })
        .expect("a hole in a steam line is admitted");
    for t in 1..=60 {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        assert!(engine.snapshot().solver.converged, "tick {t}");
    }
    let s = engine.snapshot();
    let point = s
        .nodes
        .iter()
        .find(|n| n.name == "supply_line__leak_point")
        .unwrap();
    let (p0, t0) = (point.pressure_pa, point.temperature_k);
    let rho0 = p0 * 0.018015 / (8.314_462_618 * t0);
    let want = ORIFICE_CD * HOLE * (2.0 * rho0 * p0 * psi_choked(steam_gamma())).sqrt();
    let got = s
        .edges
        .iter()
        .find(|e| e.name == "supply_line__leak")
        .unwrap()
        .stream
        .mass_flow
        .value();
    assert!(
        (got / want - 1.0).abs() < 1e-6,
        "{got} kg/s through the hole, the nozzle's {want}"
    );
}

/// **On a plant that boils, `x_t` is read on vapour alone** (M63.3): the line
/// flash admits no gas-phase member, so no valve there is in gas service, and
/// a valve declaring `x_t` whose stream arrives liquid — the supply's water at
/// 50 °C, far below its boiling point — passes exactly what the same valve
/// without it does, the liquid law, bit for bit.
#[test]
fn a_valve_reads_its_x_t_on_vapour_alone() {
    let cold = |src: String| {
        let out = src.replace("temperature_c = 400.0", "temperature_c = 50.0");
        assert!(
            out.contains("temperature_c = 50.0"),
            "the supply was cooled"
        );
        out
    };
    for solver in ["newton", "simple"] {
        let plain = settled_src(&cold(steam_line(solver, 3.0))).snapshot();
        let declared = settled_src(&cold(with_x_t(&steam_line(solver, 3.0), 0.72))).snapshot();
        assert!(flow(&plain) > 1.0, "{solver}: the water flows");
        assert_eq!(
            flow(&declared).to_bits(),
            flow(&plain).to_bits(),
            "{solver}: x_t moved a liquid valve"
        );
    }
}

/// **A check valve chokes on steam too**: the valve kinds share the one fold
/// (`fold_valve`, M63.3), so a check valve whose disc is full open holds its
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
