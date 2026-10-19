//! M64.3 — the steamed crude unit stripped by its steam alone
//! (`scenarios/crude_unit_stripped.toml`, docs/DESIGN.md §69.4, §69.9): the
//! reboiler held at zero, the furnace carrying its heat, the charge held on a
//! flow loop. The held column's equations are gated in
//! `refinery-solvers`' `reference/held_reboiler.rs`, the key's way through the
//! loader in `held_reboiler_reference.rs`; these gates hold the shipped plant
//! to the numbers it was built on, and its column's fallback — the
//! continuation, run mid-run when the last tick's profile cannot follow a move
//! — to a move that forces it.

use refinery_core::graph::{ControlledValue, LoopId, NodeKind};
use refinery_core::snapshot::{Command, EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::units::{KgPerSec, Watt};
use refinery_core::Engine;

const PLANT: &str = include_str!("../../../scenarios/crude_unit_stripped.toml");
const PRODUCTS: [&str; 5] = ["naphtha", "kerosene", "diesel", "gas_oil", "residue"];
/// The charge loop, the plant's only one.
const CHARGE_LOOP: LoopId = LoopId(0);

fn build(flow: &str) -> Engine {
    let src = PLANT.replace("flow = \"newton\"", &format!("flow = \"{flow}\""));
    let file = refinery_scenarios::load_str(&src).expect("the plant parses");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant builds: {e}"))
}

fn run(engine: &mut Engine, ticks: u32) {
    for _ in 0..ticks {
        let tick = engine.snapshot().tick + 1;
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: tick {tick}: {e}"));
        assert!(
            engine.snapshot().solver.converged,
            "tick {tick} did not converge"
        );
    }
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn edge<'a>(s: &'a Snapshot, name: &str) -> &'a EdgeSnapshot {
    s.edges.iter().find(|e| e.name == name).unwrap()
}

/// The column's state as the gates read it: the bottom [°C], the reflux
/// ratio, the feed's vapour share (molar) and the charge [kg/s].
fn state(s: &Snapshot) -> (f64, f64, f64, f64) {
    let operation = node(s, "column").column_operation.clone().unwrap();
    (
        operation.stage_temperatures_k.last().unwrap() - 273.15,
        operation.reflux_ratio,
        operation.feed_vapour_fraction,
        edge(s, "crude_line").stream.mass_flow.value(),
    )
}

/// The coil's temperature and its tubes' failure limit [°C].
fn coil(s: &Snapshot) -> (f64, f64) {
    match &node(s, "furnace").kind {
        NodeKind::Furnace { coil, tubes, .. } => (
            coil.temperature.value() - 273.15,
            tubes.failure_temperature.value() - 273.15,
        ),
        _ => panic!("the furnace is a furnace"),
    }
}

/// **The plant starts where it stays.** Its coil declared at the 436.88 °C
/// it settles at under 58 MW, its feed valve at the opening its loop settles
/// at, every tick of the first hundred holds the measured state (§69.9): the
/// reboiler published as exactly zero, the bottom at 363.63 °C — a real
/// atmospheric bottom's 340–370 °C — on a reflux of 3.460, the feed flashing
/// 0.888 of its moles (about 70 % of its mass, a real flash zone's; M64.0
/// finding 4's conversion), the charge at the loop's 112.8 kg/s, the coil
/// within 0.05 K of its start and far under its limit.
fn starts_where_it_stays(flow: &str) {
    let mut engine = build(flow);
    for _ in 0..100 {
        run(&mut engine, 1);
        let s = engine.snapshot();
        let tick = s.tick;
        assert_eq!(node(&s, "column").column_duty.unwrap().reboiler_w, 0.0);
        let (bottom, reflux, flash, charge) = state(&s);
        assert!(
            (bottom - 363.63).abs() < 0.05,
            "tick {tick}: bottom {bottom} °C"
        );
        assert!(
            (reflux - 3.460).abs() < 1.0e-3,
            "tick {tick}: reflux {reflux}"
        );
        assert!((flash - 0.888).abs() < 1.0e-3, "tick {tick}: flash {flash}");
        assert!(
            (charge - 112.8).abs() < 0.01,
            "tick {tick}: charge {charge} kg/s"
        );
        let (metal, limit) = coil(&s);
        assert!(
            (metal - 436.88).abs() < 0.05,
            "tick {tick}: coil {metal} °C"
        );
        assert!(
            metal < limit - 100.0,
            "tick {tick}: coil {metal} °C, limit {limit}"
        );
    }
}

#[test]
fn the_stripped_crude_unit_starts_where_it_stays_newton() {
    starts_where_it_stays("newton");
}

#[test]
fn slow_the_stripped_crude_unit_starts_where_it_stays_simple() {
    starts_where_it_stays("simple");
}

/// **The column's first law closes on the plant's own books** with nothing
/// from a reboiler or a pumparound in them: the crude's enthalpy and the
/// steam's in, the condenser's duty and the products' and the water draw's
/// enthalpy out, every tick of the first hundred. To Newton's tolerance, not
/// to rounding as on the steamed plant (1e-12): there the reboiler's duty is
/// the closure, computed from the balance; here it is held at zero and the
/// bottom stage's balance is an equation solved to 8.9e-8 of the feed's latent
/// heat flow (§69.7) — under 3 W on its 32 MW. Measured 1.6e-9 of the gross.
#[test]
fn the_stripped_crude_unit_closes_its_first_law() {
    let mut engine = build("newton");
    let pumparounds: f64 = match &node(&engine.snapshot(), "column").kind {
        NodeKind::Column {
            cascade: Some(spec),
            ..
        } => spec.pumparounds.iter().map(|p| p.duty.value()).sum(),
        _ => panic!("the crude unit's column has a cascade"),
    };
    assert_eq!(pumparounds, 0.0);
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
        assert_eq!(duty.reboiler_w, 0.0);
        let books = arriving - duty.condenser_w - leaving;
        assert!(books.abs() < 3.0, "the column's first law: {books} W");
        let gross = arriving.abs() + duty.condenser_w + leaving.abs();
        worst = worst.max(books.abs() / gross);
    }
    assert!(worst <= 1e-8, "the column's first law: worst {worst:e}");
}

/// **A charge step the last tick cannot follow is caught by the fallback**
/// (M64.2's debt, docs/DESIGN.md §69.9). Settled, the loop's setpoint moved
/// 112.8 → 140 kg/s steps the valve on its proportional action, and the
/// charge jumps to 127.8 kg/s in one tick. The column's warm start from the
/// last tick fails there (measured: on every pass of that tick); the
/// continuation — the same column with its reboiler, walked to zero — starts
/// it, and the plant runs on: the reboiler at zero, the bottom 358.80 °C on a
/// reflux of 3.267, and the next ten ticks warm from it. Its gas oil is cut
/// back (M67.1, §72.1), to 95 % of its share on the step and 99.6 % ten ticks
/// on: before M67 the column ran that tray under the minimum liquid, at
/// 359.95 °C and 3.262. (The loop then
/// swings the charge back to 125.5 kg/s by tick 150 and climbs to its setpoint
/// on the integral's 20 s or so, 137.7 at tick 700 — measured, not gated: the
/// fast tier's budget.)
fn a_charge_step_is_caught_by_the_fallback(flow: &str) {
    let mut engine = build(flow);
    run(&mut engine, 100);
    engine
        .apply(Command::SetSetpoint {
            loop_id: CHARGE_LOOP,
            value: ControlledValue::Flow {
                kg_per_s: KgPerSec(140.0),
            },
        })
        .unwrap();
    run(&mut engine, 1);
    let s = engine.snapshot();
    assert_eq!(node(&s, "column").column_duty.unwrap().reboiler_w, 0.0);
    let (bottom, reflux, _, charge) = state(&s);
    assert!((charge - 127.80).abs() < 0.01, "charge {charge} kg/s");
    assert!((bottom - 358.80).abs() < 0.01, "bottom {bottom} °C");
    assert!((reflux - 3.267).abs() < 1.0e-3, "reflux {reflux}");
    let cut = &node(&s, "column").column_draws_cut_back;
    assert_eq!(cut.len(), 1, "{cut:?}");
    assert_eq!(cut[0].draw, 4, "the gas-oil draw");
    assert!((cut[0].of_declared - 0.954).abs() < 0.005, "{cut:?}");
    run(&mut engine, 10);
    assert_eq!(
        node(&engine.snapshot(), "column")
            .column_duty
            .unwrap()
            .reboiler_w,
        0.0
    );
}

#[test]
fn a_charge_step_is_caught_by_the_fallback_newton() {
    a_charge_step_is_caught_by_the_fallback("newton");
}

#[test]
fn slow_a_charge_step_is_caught_by_the_fallback_simple() {
    a_charge_step_is_caught_by_the_fallback("simple");
}

/// **A pumparound that takes the diesel tray's liquid cuts the diesel back**
/// (M67.1, docs/DESIGN.md §72.1): 30 MW of pumparound on stage 7 — refused
/// until M67 — runs, its diesel draw at 24 % of its share, the bottom near
/// 318 °C.
#[test]
fn a_pumparound_that_takes_the_diesel_trays_liquid_cuts_it_back() {
    let mut engine = build("newton");
    run(&mut engine, 10);
    let column = node(&engine.snapshot(), "column").id;
    engine
        .apply(Command::SetPumparoundDuty {
            node: column,
            pumparound: 2,
            duty: Watt(30.0e6),
        })
        .unwrap();
    engine
        .tick()
        .unwrap_or_else(|e| panic!("30 MW on stage 7: {e}"));
    let s = engine.snapshot();
    let cut = &node(&s, "column").column_draws_cut_back;
    assert_eq!(cut.len(), 1, "{cut:?}");
    assert_eq!(cut[0].draw, 3, "the diesel draw");
    assert!((cut[0].of_declared - 0.245).abs() < 0.005, "{cut:?}");
    let (bottom, _, _, _) = state(&s);
    assert!((bottom - 317.72).abs() < 0.01, "bottom {bottom} °C");
}

/// **Past its edge the column is refused by name, and the fallback says
/// where it stopped** (§69.9): 40 MW of pumparound on stage 7 takes the trays'
/// liquid faster than cutting the draws back gives it, and the continuation
/// the warm start's failure sends it to stops short of zero.
#[test]
fn a_pumparound_past_its_edge_is_refused_by_name() {
    let mut engine = build("newton");
    run(&mut engine, 10);
    let column = node(&engine.snapshot(), "column").id;
    engine
        .apply(Command::SetPumparoundDuty {
            node: column,
            pumparound: 2,
            duty: Watt(40.0e6),
        })
        .unwrap();
    let e = engine
        .tick()
        .expect_err("40 MW on stage 7 is past the column's edge")
        .to_string();
    for part in [
        "starts from the same column with its reboiler",
        "stopped at step",
        "run dry",
        "40.0 MW on stage 7",
    ] {
        assert!(e.contains(part), "missing '{part}' in: {e}");
    }
}

/// **A furnace and a steam rate under the old floor run, the gas oil cut
/// back** (M67.1, docs/DESIGN.md §72.1): the furnace at 45 MW and the steam
/// valve at 0.05 from tick 1 000 — a halt some 800 ticks later until M67, the
/// gas-oil tray run dry (M67.0's finding 6). Now the plant runs on, the
/// reboiler at zero, its gas oil at 43 % of its share 3 000 ticks on (and
/// falling with the coil: 35 % at 6 000), the only draw cut back. Its
/// ticks cost what the coil's drift costs with nothing cut (A29).
#[test]
fn slow_a_furnace_and_steam_under_the_old_floor_run_cut_back() {
    let mut engine = build("newton");
    run(&mut engine, 1_000);
    let s = engine.snapshot();
    let (furnace, valve) = (node(&s, "furnace").id, node(&s, "steam_valve").id);
    engine
        .apply(Command::SetFurnaceDuty {
            node: furnace,
            duty: Watt(45.0e6),
        })
        .unwrap();
    engine
        .apply(Command::SetValveOpening {
            node: valve,
            opening: 0.05,
        })
        .unwrap();
    run(&mut engine, 3_000);
    let s = engine.snapshot();
    assert_eq!(node(&s, "column").column_duty.unwrap().reboiler_w, 0.0);
    let cut = &node(&s, "column").column_draws_cut_back;
    assert_eq!(cut.len(), 1, "{cut:?}");
    assert_eq!(cut[0].draw, 4, "the gas-oil draw");
    assert!((cut[0].of_declared - 0.430).abs() < 0.005, "{cut:?}");
}
