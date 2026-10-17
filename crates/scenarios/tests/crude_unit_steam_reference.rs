//! M62.6d — the wet crude unit with stripping steam (`scenarios/crude_unit_steam.toml`,
//! docs/DESIGN.md §67.21): a 10 bar, 400 °C supply through a valve that chokes
//! on the all-vapour steam, into the column's bottom stage. The column's steam
//! equations are gated in `reference/steam_column.rs`, the valve's choke in
//! `steam_valve_reference.rs`; these gates hold the shipped plant to what it is
//! for.

use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::Engine;
use refinery_solvers::elements::QuadraticBranch;
use refinery_solvers::network::RHO_WATER_REF;

const PLANT: &str = include_str!("../../../scenarios/crude_unit_steam.toml");
const WET: &str = include_str!("../../../scenarios/crude_unit_wet.toml");
const PRODUCTS: [&str; 5] = ["naphtha", "kerosene", "diesel", "gas_oil", "residue"];

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the plant parses");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant builds: {e}"))
}

fn run(engine: &mut Engine, ticks: u32) {
    for t in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
        assert!(
            engine.snapshot().solver.converged,
            "tick {t} did not converge"
        );
    }
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn edge<'a>(s: &'a Snapshot, name: &str) -> &'a EdgeSnapshot {
    s.edges.iter().find(|e| e.name == name).unwrap()
}

/// **The steam rate is the choked valve's, and it leaves by the water draw.**
/// Half open, the valve passes the ω law's choked flow at the state the solve
/// read (`ω = 1`, `ṁ = ρ₀·e^{−1/2}·√(P₀/(2α_v))`) — 0.853 kg/s, a real column's
/// 8–10 lb of steam per barrel of residue. The water draw carries the crude's
/// water and the steam's; no product carries any; the column sends out what
/// both lines bring; and the bottom stands colder than the wet plant's.
#[test]
fn the_steam_is_the_choked_valves_and_leaves_by_the_water_draw() {
    let mut engine = build(PLANT);
    run(&mut engine, 100);
    let s = engine.snapshot();
    let water = engine.slate.free_water().unwrap();

    let valve = node(&s, "steam_valve");
    let (p0, t0) = (valve.pressure_pa, valve.temperature_k);
    let rho0 = p0 * 0.018015 / (8.314_462_618 * t0);
    let (cv, opening) = match &valve.kind {
        NodeKind::Valve {
            cv_max, opening, ..
        } => (*cv_max, *opening),
        _ => panic!("the steam valve is a valve"),
    };
    let alpha = QuadraticBranch::valve(cv, opening, rho0 / RHO_WATER_REF).alpha;
    let choked = rho0 * (-0.5f64).exp() * (p0 / (2.0 * alpha)).sqrt();
    let steam = edge(&s, "stripping_steam").stream.mass_flow.value();
    assert!(
        (steam / choked - 1.0).abs() < 1e-6,
        "{steam} kg/s of steam, the choked valve's {choked}"
    );
    assert!((0.80..0.90).contains(&steam), "{steam} kg/s");

    let feed = edge(&s, "transfer_line__downstream");
    let crude_water = feed.stream.mass_flow.value() * feed.stream.composition.fractions()[water];
    let draw = edge(&s, "water_draw").stream.mass_flow.value();
    assert!(
        (draw - crude_water - steam).abs() < 1e-9 * draw,
        "the water draw takes {draw} kg/s of the crude's {crude_water} and the steam's {steam}"
    );
    let mut out = draw;
    for product in PRODUCTS {
        let suction = edge(&s, &format!("{product}_suction"));
        assert_eq!(
            suction.stream.composition.fractions()[water],
            0.0,
            "{product} carries water"
        );
        out += suction.stream.mass_flow.value();
    }
    let into = feed.stream.mass_flow.value() + steam;
    assert!((out - into).abs() < 1e-9 * into, "{out} out of {into}");

    let mut wet = build(WET);
    run(&mut wet, 100);
    let bottom = |s: &Snapshot| {
        *node(s, "column")
            .column_operation
            .as_ref()
            .unwrap()
            .stage_temperatures_k
            .last()
            .unwrap()
    };
    let (with, without) = (bottom(&s), bottom(&wet.snapshot()));
    assert!(
        with < without - 4.0,
        "the steamed bottom at {with} K, the wet plant's at {without} K"
    );
}

/// **The column's first law closes on the plant's own books, the steam's
/// enthalpy among them**, every tick of the first hundred.
#[test]
fn the_steamed_crude_unit_closes_its_first_law() {
    let mut engine = build(PLANT);
    let pumparounds: f64 = match &node(&engine.snapshot(), "column").kind {
        NodeKind::Column {
            cascade: Some(spec),
            ..
        } => spec.pumparounds.iter().map(|p| p.duty.value()).sum(),
        _ => panic!("the crude unit's column has a cascade"),
    };
    let flux = |engine: &Engine, e: &EdgeSnapshot| {
        engine
            .enthalpy()
            .stream_enthalpy_flux(&engine.slate, &e.stream)
            .unwrap()
            .value()
    };
    let mut worst = 0.0f64;
    for _ in 0..100 {
        run(&mut engine, 1);
        let s = engine.snapshot();
        let arriving = flux(&engine, edge(&s, "transfer_line__downstream"))
            + flux(&engine, edge(&s, "stripping_steam"));
        let mut leaving = flux(&engine, edge(&s, "water_draw"));
        for product in PRODUCTS {
            leaving += flux(&engine, edge(&s, &format!("{product}_suction")));
        }
        let duty = node(&s, "column").column_duty.unwrap();
        let books = arriving + duty.reboiler_w - duty.condenser_w - pumparounds - leaving;
        let gross =
            arriving.abs() + duty.reboiler_w + duty.condenser_w + pumparounds + leaving.abs();
        worst = worst.max(books.abs() / gross);
    }
    assert!(worst <= 1e-12, "the column's first law: worst {worst:e}");
}

/// **Both fidelities agree** on the shipped plant: the charge, the steam and
/// the water draw.
#[test]
fn slow_the_steamed_crude_unit_runs_on_both_fidelities() {
    let mut answers = Vec::new();
    for flow in ["newton", "simple"] {
        let mut engine = build(&PLANT.replace("flow = \"newton\"", &format!("flow = \"{flow}\"")));
        run(&mut engine, 150);
        let s = engine.snapshot();
        answers.push((
            edge(&s, "crude_line").stream.mass_flow.value(),
            edge(&s, "stripping_steam").stream.mass_flow.value(),
            edge(&s, "water_draw").stream.mass_flow.value(),
        ));
    }
    let (a, b) = (answers[0], answers[1]);
    assert!(
        (a.0 - b.0).abs() < 1e-6 * a.0
            && (a.1 - b.1).abs() < 1e-6 * a.1
            && (a.2 - b.2).abs() < 1e-6 * a.2,
        "newton {a:?}, simple {b:?}"
    );
}
