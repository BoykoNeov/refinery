//! M23.1: the brim spills (docs/DESIGN.md §27).
//!
//! Every tank owns an overflow edge the loader builds, `<tank>__overflow`, to
//! the plant's first `Atmosphere` (or to a new `overflow_atmosphere`). At the
//! END of each tick, after the boil-off, whatever liquid stands above
//! `ρ(x)·A·H` leaves through it, so the level never reads above the shell.
//!
//! The gates, numbered as §27 names them (gate 10, the trip demo's
//! counterfactual, lives in `trip_demo.rs` beside the plant it is about):
//!
//! 1. The brim tie is exact, on a composition where a LEVEL comparison would
//!    spill a rounding error forever.
//! 2. At the brim the spill is the net inflow (the demo), and the capacity is
//!    the END-of-tick composition's (a mixing fixture).
//! 3. Each tank's mass books close, tick by tick, with the spill counted.
//! 4. The spilled stream is the tank's own liquid, and its energy is counted.
//! 5. The spill is a RATE: `dt = 0.5` against `dt = 1`.
//! 6. Ownership and order: two tanks on one declared atmosphere, and a tank
//!    that boils and spills in the same tick.
//! 7. A spill that stops reads zero at once.
//! 8. The load-time refusals, each by its own message, and `PuncturePipe`.
//! 9. The edge ids are where the bytes claim says.
//!
//! And one the note did not specify: a tank over its brim that owns no
//! overflow edge is an `Err`, not a quiet overfill.

use refinery_core::engine::Engine;
use refinery_core::graph::{LeakRole, NodeKind, TankState};
use refinery_core::snapshot::Command;
use refinery_core::stream::Stream;
use refinery_core::units::{KgPerSec, SquareMeter};
use refinery_scenarios::{build_engine, load_str};

const DEMO: &str = include_str!("../../../scenarios/tank_overflow.toml");
const BOILOFF_PLANT: &str = include_str!("../../../scenarios/crude_column_boiloff.toml");

/// The demo's receiving tank, and its overflow edge.
const RECEIVER: &str = "receiving_tank";
const SPILL: &str = "receiving_tank__overflow";
/// The tick the demo first spills, on both fidelities (§27 fork 7).
const FIRST_SPILL: u64 = 2859;

fn build(src: &str) -> Engine {
    build_engine(&load_str(src).expect("the plant must parse"))
        .unwrap_or_else(|e| panic!("the plant must build: {e}"))
}

fn on(src: &str, flow: &str) -> String {
    let from = r#"flow = "newton""#;
    assert!(src.contains(from), "the plant declares the newton fidelity");
    src.replace(from, &format!(r#"flow = "{flow}""#))
}

fn swap(src: &str, from: &str, to: &str) -> String {
    assert!(src.contains(from), "the substitution must land: `{from}`");
    src.replacen(from, to, 1)
}

fn tick(engine: &mut Engine) {
    engine
        .tick()
        .unwrap_or_else(|e| panic!("tick {}: {e}", engine.snapshot().tick + 1));
}

fn tank(engine: &Engine, name: &str) -> TankState {
    let id = engine
        .graph
        .find_node(name)
        .unwrap_or_else(|| panic!("no node '{name}'"));
    match &engine.graph.node(id).kind {
        NodeKind::Tank(t) => t.clone(),
        other => panic!("'{name}' is a tank, not {other:?}"),
    }
}

fn stream(engine: &Engine, name: &str) -> Stream {
    let id = engine
        .graph
        .edge_ids()
        .find(|e| engine.graph.pipe(*e).name == name)
        .unwrap_or_else(|| panic!("no edge '{name}'"));
    engine.graph.pipe(id).stream.clone()
}

fn flow(engine: &Engine, name: &str) -> f64 {
    stream(engine, name).mass_flow.value()
}

// --- Gate 1 -------------------------------------------------------------------

/// A water tank 5 m² by 6 m, declared exactly full: the one-ULP case. Found by
/// search, and asserted below rather than trusted: its declared-full LEVEL reads
/// 6.000000000000001 m, so `level() > height` spills a rounding error on every
/// tick, while `mass > capacity` ties exactly because both sides are
/// `TankState::mass_at_level` in one association.
const FULL_TANK: &str = r#"
[meta]
name = "full_tank"
description = "A tank declared exactly at its brim, fed through a valve."

[simulation]
dt = 1.0

[fidelity]
flow = "newton"

[nodes.feed]
type = "source"
pressure_bar = 3.0
temperature_c = 20.0

[nodes.feed_valve]
type = "valve"
kv = 20.0
opening = 0.0

[nodes.full_tank]
type = "tank"
area_m2 = 5.0
height_m = 6.0
initial_level_m = 6.0
temperature_c = 20.0

[[pipes]]
name = "feed_line"
from = "feed"
to = "feed_valve"
length_m = 5.0
diameter_m = 0.05

[[pipes]]
name = "tank_line"
from = "feed_valve"
to = "full_tank"
length_m = 5.0
diameter_m = 0.05
"#;

/// **Gate 1. The brim tie is exact.** Declared full with nothing arriving (the
/// feed valve shut, so the tank's only edge is the valve's OUTLET, which a shut
/// valve holds at exactly zero), it spills exactly `0.0` on every tick, on both
/// fidelities. Opened, it spills from tick 1 and ends each tick at its capacity
/// bit for bit.
#[test]
fn a_tank_declared_full_spills_nothing_and_a_fed_one_spills_from_tick_one() {
    for fidelity in ["newton", "simple"] {
        let mut engine = build(&on(FULL_TANK, fidelity));
        let at_load = tank(&engine, "full_tank");
        // The control. Without it the gate has nothing a level comparison
        // would get wrong, and mutation 1 would pass it.
        assert!(
            at_load.level(&engine.slate).value() > at_load.height.value(),
            "the control: this tank's declared-full level must read above its height, \
             got {} m in {} m",
            at_load.level(&engine.slate).value(),
            at_load.height.value()
        );
        assert_eq!(
            at_load.mass.value().to_bits(),
            at_load.capacity(&engine.slate).value().to_bits(),
            "declared full holds exactly its capacity, by construction"
        );
        for t in 1..=50 {
            tick(&mut engine);
            assert_eq!(flow(&engine, "tank_line"), 0.0, "{fidelity}, tick {t}");
            assert_eq!(
                flow(&engine, "full_tank__overflow"),
                0.0,
                "{fidelity}, tick {t}: a tank exactly at its brim spills nothing"
            );
            assert_eq!(
                tank(&engine, "full_tank").mass.value().to_bits(),
                at_load.mass.value().to_bits(),
                "{fidelity}, tick {t}"
            );
        }

        let mut fed = build(&on(
            &swap(FULL_TANK, "opening = 0.0", "opening = 1.0"),
            fidelity,
        ));
        for t in 1..=50 {
            tick(&mut fed);
            let inflow = flow(&fed, "tank_line");
            let spill = flow(&fed, "full_tank__overflow");
            assert!(inflow > 0.0, "{fidelity}, tick {t}: the feed runs");
            assert!(spill > 0.0, "{fidelity}, tick {t}: and the tank spills it");
            let state = tank(&fed, "full_tank");
            assert_eq!(
                state.mass.value().to_bits(),
                state.capacity(&fed.slate).value().to_bits(),
                "{fidelity}, tick {t}: a spilling tank ends the tick at its capacity"
            );
            // One liquid, full at both ends of the tick, so everything that
            // arrived left: to a few ULP of the inventory over `dt`.
            let bound = 8.0 * f64::EPSILON * state.mass.value();
            assert!(
                (spill - inflow).abs() <= bound,
                "{fidelity}, tick {t}: spilled {spill} kg/s of {inflow} kg/s arriving"
            );
        }
    }
}

// --- Gate 2 -------------------------------------------------------------------

/// **Gate 2. At the brim, the spill is the net inflow.** On the demo, from the
/// tick after the first spill, the receiving tank holds its capacity bit for bit
/// and spills fill minus drain, to a few ULP of its inventory over `dt`. Probe:
/// 1.82e-12 kg/s, 3.0e-13 relative. The first spilling tick only spills the
/// part of that tick's inflow that stood above the brim, so it is excluded.
#[test]
fn at_the_brim_the_demo_spills_its_net_inflow() {
    for fidelity in ["newton", "simple"] {
        let mut engine = build(&on(DEMO, fidelity));
        let dt = 1.0;
        let mut first = None;
        for t in 1..=3_200u64 {
            tick(&mut engine);
            let spill = flow(&engine, SPILL);
            if spill > 0.0 && first.is_none() {
                first = Some(t);
                continue;
            }
            if first.is_none() {
                assert_eq!(
                    spill, 0.0,
                    "{fidelity}, tick {t}: an idle overflow reads zero"
                );
                continue;
            }
            let state = tank(&engine, RECEIVER);
            assert_eq!(
                state.mass.value().to_bits(),
                state.capacity(&engine.slate).value().to_bits(),
                "{fidelity}, tick {t}"
            );
            let net = flow(&engine, "fill_line") - flow(&engine, "drain_line");
            let bound = 8.0 * f64::EPSILON * state.mass.value() / dt;
            assert!(
                (spill - net).abs() <= bound,
                "{fidelity}, tick {t}: spilled {spill}, net inflow {net}, bound {bound}"
            );
        }
        assert_eq!(first, Some(FIRST_SPILL), "{fidelity}");
    }
}

/// A tank at its brim holding diesel, fed kerosene: its contents lighten every
/// tick, so its capacity in KILOGRAMS falls, and which composition the capacity
/// is taken at is visible.
const MIXING: &str = r#"
[meta]
name = "lightening_tank"
description = "A full tank of diesel fed kerosene: its capacity in kg falls as it lightens."

[simulation]
dt = 1.0

[fidelity]
flow = "newton"

[[components]]
name = "kerosene"
tb_c = 200.0
molar_mass_kg_per_mol = 0.170
density_kg_per_m3 = 800.0
cp_j_per_kg_k = 2000.0

[[components]]
name = "diesel"
tb_c = 300.0
molar_mass_kg_per_mol = 0.230
density_kg_per_m3 = 850.0
cp_j_per_kg_k = 1900.0

[nodes.feed]
type = "source"
pressure_bar = 3.0
temperature_c = 20.0
composition = { kerosene = 1.0 }

[nodes.feed_valve]
type = "valve"
kv = 20.0
opening = 1.0

[nodes.buffer]
type = "tank"
area_m2 = 2.0
height_m = 3.0
initial_level_m = 3.0
temperature_c = 20.0
composition = { diesel = 1.0 }

[[pipes]]
name = "feed_line"
from = "feed"
to = "feed_valve"
length_m = 5.0
diameter_m = 0.05

[[pipes]]
name = "tank_line"
from = "feed_valve"
to = "buffer"
length_m = 5.0
diameter_m = 0.05
"#;

/// **Gate 2's mixing case.** The capacity is `ρ(x_end)·A·H`, bit for bit, and it
/// differs from the capacity at the start-of-tick composition on every tick —
/// the control, without which mutation 3 (capacity at the START-of-tick
/// composition) would pass.
#[test]
fn the_capacity_is_the_end_of_tick_compositions() {
    for fidelity in ["newton", "simple"] {
        let mut engine = build(&on(MIXING, fidelity));
        for t in 1..=200 {
            let before = tank(&engine, "buffer");
            tick(&mut engine);
            let after = tank(&engine, "buffer");
            assert!(
                flow(&engine, "buffer__overflow") > 0.0,
                "{fidelity}, tick {t}"
            );
            let end = after.capacity(&engine.slate).value();
            let start = TankState {
                composition: before.composition.clone(),
                ..after.clone()
            }
            .capacity(&engine.slate)
            .value();
            assert_eq!(
                after.mass.value().to_bits(),
                end.to_bits(),
                "{fidelity}, tick {t}"
            );
            assert!(
                end < start,
                "{fidelity}, tick {t}: the control — lightening lowers the capacity \
                 ({end} kg against {start} kg at the start-of-tick composition)"
            );

            // Gate 4's composition half, on the one plant whose compositions
            // differ: the demo is water only, so there every composition is
            // `[1.0]` and a spill at the INFLOW's composition (mutation 2), or
            // one the composition pass overwrote with the upwind START-of-tick
            // one, reads the same bytes. Here all three differ.
            let spilled = stream(&engine, "buffer__overflow").composition;
            assert_eq!(
                spilled.fractions(),
                after.composition.fractions(),
                "{fidelity}, tick {t}: the spill is the tank's end-of-tick liquid"
            );
            assert_ne!(
                spilled.fractions(),
                before.composition.fractions(),
                "{fidelity}, tick {t}: the control — the tank's liquid changed this tick"
            );
            assert_ne!(
                spilled.fractions(),
                stream(&engine, "tank_line").composition.fractions(),
                "{fidelity}, tick {t}: the control — the tank is not its inflow"
            );
        }
    }
}

// --- Gate 3 -------------------------------------------------------------------

/// **Gate 3. Each tank's mass books close, tick by tick, with the spill
/// counted**, on the demo: `Δm = (Σ in − Σ out − spill)·dt` to a few ULP of the
/// gross traffic. Probe: 1.1e-16 relative.
///
/// **The control is part of the gate**: no tank starves inside the window, so
/// the books are the spill's and not M24's dry-tank path. **A plant-wide sum is
/// NOT this gate**: it misses by the solvers' own node imbalance at the pump and
/// valve (6.3e-5 kg on Newton, 3.7e-3 kg on the game solver over the run), which
/// measures the solver, not the spill.
#[test]
fn each_tanks_mass_books_close_with_the_spill_counted() {
    for fidelity in ["newton", "simple"] {
        let mut engine = build(&on(DEMO, fidelity));
        let dt = 1.0;
        let mut spilling = 0;
        for t in 1..=4_000u64 {
            let supply_old = tank(&engine, "supply_tank").mass.value();
            let receiver_old = tank(&engine, RECEIVER).mass.value();
            tick(&mut engine);
            assert!(
                engine.last_solution().unwrap().starved.is_empty(),
                "{fidelity}, tick {t}: the control — no tank starves in the window"
            );
            let suction = flow(&engine, "suction");
            let fill = flow(&engine, "fill_line");
            let drain = flow(&engine, "drain_line");
            let spill = flow(&engine, SPILL);
            assert_eq!(flow(&engine, "supply_tank__overflow"), 0.0);
            if spill > 0.0 {
                spilling += 1;
            }

            let supply_new = tank(&engine, "supply_tank").mass.value();
            let receiver_new = tank(&engine, RECEIVER).mass.value();
            for (name, old, new, into, out) in [
                ("supply_tank", supply_old, supply_new, 0.0, suction),
                (RECEIVER, receiver_old, receiver_new, fill, drain + spill),
            ] {
                let residual = (new - old) - (into - out) * dt;
                let scale = old + (into + out) * dt;
                let relative = residual.abs() / scale;
                assert!(
                    relative <= 4.0 * f64::EPSILON,
                    "{fidelity}, tick {t}, {name}: residual {residual} kg on {scale} kg"
                );
            }
        }
        assert!(
            spilling > 1_000,
            "{fidelity}: the books saw {spilling} spilling ticks"
        );
    }
}

// --- Gate 4 -------------------------------------------------------------------

/// **Gate 4. The spilled stream is the tank's liquid, and its energy is
/// counted.** On every spilling tick the overflow edge carries the tank's
/// END-of-tick temperature and composition bit for bit, and no `latent`. And the
/// receiving tank's own energy balance closes with the spill booked through
/// `stream_enthalpy_flux`:
///
///   `E_new − E_old = (h_fill·ṁ_fill − h_tank,old·ṁ_drain − h_spill·ṁ_spill)·dt`
///
/// with the drain debited at the tank's START-of-tick state (the upwind rule)
/// and the fill at its arriving temperature, which is its published outlet.
///
/// **The tolerance is round-off in the gate's own `ΔE`**: a difference of two
/// stocks of about 2.5e9 J (29 940 kg of water about 20 K above the 273.15 K
/// datum), whose absolute rounding is fixed, against per-tick fluxes of about
/// 1e6 J. So the residual is graded against the STOCK, and the bound is a few
/// ULP of it. Dropping the spill's term misses by its ~5e5 J per tick, about
/// 1e12 ULP. See `ENERGY_BOUND_ULPS`.
#[test]
fn the_spill_is_the_tanks_own_liquid_and_its_energy_is_counted() {
    for fidelity in ["newton", "simple"] {
        let mut engine = build(&on(DEMO, fidelity));
        let dt = 1.0;
        let mut spilling_ticks = 0;
        for _ in 1..=3_400u64 {
            let old = tank(&engine, RECEIVER);
            tick(&mut engine);
            let new = tank(&engine, RECEIVER);
            let spill = stream(&engine, SPILL);
            if spill.mass_flow.value() == 0.0 {
                continue;
            }
            spilling_ticks += 1;
            assert_eq!(
                spill.temperature.value().to_bits(),
                new.temperature.value().to_bits()
            );
            assert_eq!(spill.composition.fractions(), new.composition.fractions());
            assert!(spill.latent.is_none(), "a spill is liquid");

            let model = engine.enthalpy();
            let slate = &engine.slate;
            let stock = |t: &TankState| {
                model
                    .enthalpy_stock(slate, &t.composition, t.mass, t.temperature)
                    .unwrap()
            };
            let fill = stream(&engine, "fill_line");
            let drain = flow(&engine, "drain_line");
            let into = model.stream_enthalpy_flux(slate, &fill).unwrap().value();
            let out = model
                .enthalpy_flux(slate, &old.composition, KgPerSec(drain), old.temperature)
                .unwrap()
                .value()
                + model.stream_enthalpy_flux(slate, &spill).unwrap().value();
            let residual = (stock(&new) - stock(&old)) - (into - out) * dt;
            let ulps = residual.abs() / (f64::EPSILON * stock(&old));
            assert!(
                ulps <= ENERGY_BOUND_ULPS,
                "{fidelity}: residual {residual} J is {ulps} ULP of the {} J stock",
                stock(&old)
            );
        }
        assert!(spilling_ticks > 500, "{fidelity}: the gate saw the spill");
    }
}

/// Measured worst on the demo over its 542 spilling ticks in this window: 7.33
/// ULP of the stock on Newton, 7.20 on the game solver. The bound is about
/// twice that, and eleven orders below the dropped-term case.
const ENERGY_BOUND_ULPS: f64 = 16.0;

// --- Gate 5 -------------------------------------------------------------------

/// **Gate 5. The spill is a RATE.** The demo at `dt = 0.5` for 12 000 ticks
/// against `dt = 1` for 6 000: the two spill rates at `t = 6 000 s` agree to
/// 1e-4 relative (probe: 8.7e-6). A per-tick total would differ by 2×. **This is
/// the only gate that sees mutation 7, because the demo runs at `dt = 1`**,
/// where a mass per tick and a mass per second are the same number.
#[test]
fn the_spill_is_a_rate_not_a_mass_per_tick() {
    let mut whole = build(DEMO);
    for _ in 0..6_000 {
        tick(&mut whole);
    }
    let half_step = swap(DEMO, "dt = 1.0 ", "dt = 0.5 ");
    let mut half = build(&half_step);
    for _ in 0..12_000 {
        tick(&mut half);
    }
    let (a, b) = (flow(&whole, SPILL), flow(&half, SPILL));
    assert!(a > 5.0, "the demo is spilling at t = 6 000 s: {a}");
    assert!(
        ((a - b) / a).abs() < 1.0e-4,
        "{a} kg/s at dt = 1 against {b} kg/s at dt = 0.5"
    );
}

// --- Gate 6 -------------------------------------------------------------------

/// Two tanks and one DECLARED atmosphere that ordinary pipes also reach. Tank
/// `filling` is fed faster than it drains and spills; tank `draining` only
/// drains. Both overflows land on `air`, and so do both drain pipes.
fn two_tanks(first: &str, second: &str) -> String {
    let tank = |name: &str| match name {
        "filling" => r#"
[nodes.filling]
type = "tank"
area_m2 = 1.0
height_m = 2.0
initial_level_m = 1.9
temperature_c = 20.0
"#
        .to_string(),
        _ => r#"
[nodes.draining]
type = "tank"
area_m2 = 1.0
height_m = 2.0
initial_level_m = 1.5
temperature_c = 20.0
"#
        .to_string(),
    };
    format!(
        r#"
[meta]
name = "two_tanks_one_air"
description = "Two tanks on one declared atmosphere; one spills."

[simulation]
dt = 1.0

[fidelity]
flow = "newton"

[nodes.feed]
type = "source"
pressure_bar = 3.0
temperature_c = 20.0

[nodes.feed_valve]
type = "valve"
kv = 20.0
opening = 1.0
{}{}
[nodes.air]
type = "atmosphere"

[[pipes]]
name = "feed_line"
from = "feed"
to = "feed_valve"
length_m = 5.0
diameter_m = 0.05

[[pipes]]
name = "fill_line"
from = "feed_valve"
to = "filling"
length_m = 5.0
diameter_m = 0.05

[[pipes]]
name = "filling_drain"
from = "filling"
to = "air"
length_m = 5.0
diameter_m = 0.02

[[pipes]]
name = "draining_drain"
from = "draining"
to = "air"
length_m = 5.0
diameter_m = 0.02
"#,
        tank(first),
        tank(second)
    )
}

/// **Gate 6. Ownership and order.** Each overflow carries only its own tank's
/// spill, the other reads exactly `0.0`, and the atmosphere's own pass writes
/// nothing over either — asserted through the spilling tank's own mass books,
/// which a zero written over its spill would break. Run in both declaration
/// orders, so whichever overflow the atmosphere meets first, one arrangement
/// puts the spilling tank's there.
#[test]
fn each_overflow_carries_only_its_own_tanks_spill() {
    for (first, second) in [("filling", "draining"), ("draining", "filling")] {
        for fidelity in ["newton", "simple"] {
            let mut engine = build(&on(&two_tanks(first, second), fidelity));
            assert!(
                engine.graph.find_node("overflow_atmosphere").is_none(),
                "the declared atmosphere is reused"
            );
            let mut spilled = 0;
            for t in 1..=400 {
                let old = tank(&engine, "filling").mass.value();
                tick(&mut engine);
                let spill = flow(&engine, "filling__overflow");
                assert_eq!(
                    flow(&engine, "draining__overflow"),
                    0.0,
                    "{first} first, {fidelity}, tick {t}: the other tank spills nothing"
                );
                assert!(flow(&engine, "draining_drain") > 0.0);
                let new = tank(&engine, "filling").mass.value();
                let residual = (new - old)
                    - (flow(&engine, "fill_line") - flow(&engine, "filling_drain") - spill);
                assert!(
                    residual.abs() <= 4.0 * f64::EPSILON * (old + 20.0),
                    "{first} first, {fidelity}, tick {t}: the spilling tank's books miss \
                     by {residual} kg"
                );
                if spill > 0.0 {
                    spilled += 1;
                }
            }
            assert!(
                spilled > 100,
                "{first} first, {fidelity}: spilled on {spilled} ticks"
            );
        }
    }
}

/// A tank declared full of naphtha at 40 °C, fed naphtha at 150 °C: it heats to
/// its bubble point, and from tick 202 it boils AND spills on the same tick.
const BOILING_AT_THE_BRIM: &str = r#"
[meta]
name = "boiling_at_the_brim"
description = "A tank declared full, fed hot naphtha: it boils and spills."

[simulation]
dt = 0.1

[fidelity]
flow = "newton"
thermo = "trouton"
boiloff = "flash"

[[components]]
name = "light_naphtha"
tb_c = 80.0
molar_mass_kg_per_mol = 0.100
density_kg_per_m3 = 680.0
cp_j_per_kg_k = 2200.0

[[components]]
name = "heavy_naphtha"
tb_c = 150.0
molar_mass_kg_per_mol = 0.130
density_kg_per_m3 = 750.0
cp_j_per_kg_k = 2100.0

[nodes.hot_source]
type = "source"
pressure_bar = 3.0
temperature_c = 150.0
composition = { light_naphtha = 0.5, heavy_naphtha = 0.5 }

[nodes.feed_valve]
type = "valve"
kv = 40.0
opening = 1.0

[nodes.product_tank]
type = "tank"
area_m2 = 0.5
height_m = 1.0
initial_level_m = 1.0
temperature_c = 40.0
composition = { light_naphtha = 0.5, heavy_naphtha = 0.5 }

[[pipes]]
name = "feed_line"
from = "hot_source"
to = "feed_valve"
length_m = 10.0
diameter_m = 0.10

[[pipes]]
name = "tank_line"
from = "feed_valve"
to = "product_tank"
length_m = 10.0
diameter_m = 0.10
"#;

/// **Gate 6's second half: the spill comes AFTER the boil-off.** On a tank that
/// boils and spills in the same tick, both edges are written, and the tank ends
/// each spilling tick exactly at its capacity — which is what "after" means.
/// Spilling first would leave it below the brim by the mass that then boiled.
/// The vent lands on the loader's `boiloff_atmosphere`, and so does the
/// overflow: one atmosphere, not two.
#[test]
fn a_boiling_tank_at_its_brim_vents_and_spills_and_ends_exactly_full() {
    let mut engine = build(BOILING_AT_THE_BRIM);
    assert!(engine.graph.find_node("overflow_atmosphere").is_none());
    let mut both = 0;
    for t in 1..=600u64 {
        tick(&mut engine);
        let vent = flow(&engine, "product_tank__boiloff_vent");
        let spill = flow(&engine, "product_tank__overflow");
        assert!(spill > 0.0, "tick {t}: the feed outruns the flash");
        let state = tank(&engine, "product_tank");
        assert_eq!(
            state.mass.value().to_bits(),
            state.capacity(&engine.slate).value().to_bits(),
            "tick {t}: vent {vent} kg/s, spill {spill} kg/s"
        );
        if vent > 0.0 {
            both += 1;
            assert!(stream(&engine, "product_tank__boiloff_vent")
                .latent
                .is_some());
            assert!(stream(&engine, "product_tank__overflow").latent.is_none());
        }
    }
    assert!(both > 300, "the tank boiled while spilling on {both} ticks");
}

// --- Gate 7 -------------------------------------------------------------------

/// **Gate 7. A spill that stops reads zero at once.** The demo spilling, its
/// discharge valve shut by command: the tank falls below its brim on the next
/// tick and its overflow reads exactly `0.0` — not the last tick's rate.
#[test]
fn a_spill_that_stops_reads_zero_on_the_next_tick() {
    for fidelity in ["newton", "simple"] {
        let mut engine = build(&on(DEMO, fidelity));
        for _ in 0..FIRST_SPILL + 50 {
            tick(&mut engine);
        }
        assert!(flow(&engine, SPILL) > 5.0, "{fidelity}: spilling");
        let valve = engine.graph.find_node("discharge_valve").unwrap();
        engine
            .apply(Command::SetValveOpening {
                node: valve,
                opening: 0.0,
            })
            .unwrap();
        tick(&mut engine);
        assert_eq!(flow(&engine, "fill_line"), 0.0, "{fidelity}");
        assert_eq!(
            flow(&engine, SPILL),
            0.0,
            "{fidelity}: the overflow keeps no stale rate"
        );
        let state = tank(&engine, RECEIVER);
        assert!(state.mass.value() < state.capacity(&engine.slate).value());
    }
}

// --- Gate 8 -------------------------------------------------------------------

/// **Gate 8. The load-time refusals**, each asserting a distinctive substring of
/// its OWN message, so a refusal deleted cannot be covered for by a neighbour.
#[test]
fn every_malformed_tank_or_overflow_name_is_refused_for_its_own_reason() {
    let tank_block = "area_m2 = 5.0\nheight_m = 6.0\ninitial_level_m = 6.0\n";
    let cases: Vec<(String, &str)> = vec![
        (
            swap(FULL_TANK, "area_m2 = 5.0", "area_m2 = 0.0"),
            "has area_m2 = 0",
        ),
        (
            swap(FULL_TANK, "area_m2 = 5.0", "area_m2 = -5.0"),
            "has area_m2 = -5",
        ),
        (
            swap(FULL_TANK, "area_m2 = 5.0", "area_m2 = nan"),
            "has area_m2 = NaN",
        ),
        (
            swap(FULL_TANK, "height_m = 6.0", "height_m = 0.0"),
            "has height_m = 0",
        ),
        (
            swap(FULL_TANK, "height_m = 6.0", "height_m = -6.0"),
            "has height_m = -6",
        ),
        (
            swap(FULL_TANK, "height_m = 6.0", "height_m = inf"),
            "has height_m = inf",
        ),
        (
            swap(FULL_TANK, "initial_level_m = 6.0", "initial_level_m = -1.0"),
            "initial_level_m = -1: a level must be",
        ),
        (
            swap(FULL_TANK, "initial_level_m = 6.0", "initial_level_m = nan"),
            "initial_level_m = NaN: a level must be",
        ),
        (
            swap(
                FULL_TANK,
                tank_block,
                "area_m2 = 5.0\nheight_m = 6.0\ninitial_level_m = 6.5\n",
            ),
            "starts over its own brim",
        ),
        (
            format!(
                "{FULL_TANK}\n[[pipes]]\nname = \"full_tank__overflow\"\nfrom = \"full_tank\"\n\
                 to = \"feed_valve\"\nlength_m = 1.0\ndiameter_m = 0.05\n"
            ),
            "already has a pipe by that name. A frontend finds a tank's spill",
        ),
        (
            swap(
                FULL_TANK,
                "[nodes.feed_valve]",
                "[nodes.overflow_atmosphere]\ntype = \"sink\"\npressure_bar = 1.0\n\n\
                 [nodes.feed_valve]",
            ),
            "overflow atmosphere would be named 'overflow_atmosphere'",
        ),
    ];
    for (plant, expected) in cases {
        let refused = load_str(&plant)
            .map_err(|e| e.to_string())
            .and_then(|file| build_engine(&file).map(|_| ()).map_err(|e| e.to_string()));
        match refused {
            Ok(()) => panic!("this plant should not have loaded (expected: {expected})"),
            Err(message) => assert!(
                message.contains(expected),
                "expected `{expected}` in: {message}"
            ),
        }
    }

    // Admitted at the boundary: declared exactly full is gate 1's plant.
    build(FULL_TANK);
}

/// Beside the sweep: `PuncturePipe` on an overflow edge is refused by its own
/// message — it has no area a command could set.
#[test]
fn puncturing_an_overflow_is_refused() {
    let mut engine = build(DEMO);
    let edge = engine
        .graph
        .edge_ids()
        .find(|e| engine.graph.pipe(*e).name == SPILL)
        .unwrap();
    let error = engine
        .apply(Command::PuncturePipe {
            edge,
            area: SquareMeter(0.01),
        })
        .expect_err("an overflow is not punctureable")
        .to_string();
    assert!(
        error.contains("is a tank's overflow, not a pipe"),
        "{error}"
    );
}

// --- Gate 9 -------------------------------------------------------------------

/// **Gate 9. The edge ids are where the bytes claim says.** On
/// `crude_column_boiloff` the three vents come straight after the declared pipes
/// and the overflows are the last edges in the graph, one per tank in node order,
/// all on the vents' own atmosphere. (The vents kept their pre-M23 ids 5, 6, 7
/// until M37, whose burn-out hole splits the furnace's outlet pipe in the
/// declared-pipe loop and so adds two edges ahead of them: 7, 8, 9 since,
/// docs/DESIGN.md §42. The order is the claim, and it is unchanged.) Built before the vents they would take 5–7 and
/// renumber the vents — five boil-off plants' published bytes (§27 premise 2).
/// A bytes claim gets a gate, and CI commits no baseline.
#[test]
fn the_overflows_are_built_after_the_vents() {
    let engine = build(BOILOFF_PLANT);
    let snapshot = engine.snapshot();
    let ids: Vec<(u32, &str)> = snapshot
        .edges
        .iter()
        .map(|e| (e.id.0, e.name.as_str()))
        .collect();
    assert_eq!(
        &ids[7..],
        [
            (7, "naphtha_tank__boiloff_vent"),
            (8, "distillate_tank__boiloff_vent"),
            (9, "bottoms_tank__boiloff_vent"),
            (10, "naphtha_tank__overflow"),
            (11, "distillate_tank__overflow"),
            (12, "bottoms_tank__overflow"),
        ]
    );
    let air = engine.graph.find_node("boiloff_atmosphere").unwrap();
    for eid in engine.graph.edge_ids() {
        if let LeakRole::Overflow { owner } = engine.graph.pipe(eid).leak {
            assert_eq!(engine.graph.endpoints(eid), (owner, air));
        }
    }
}

// --- Not in the note ----------------------------------------------------------

/// **A tank over its brim that owns no overflow edge is an `Err`.** §27 left it
/// unspecified; the vent's precedent (a boil-off with nowhere to go) decides it.
/// Dropping the excess would be mass leaving by no accounted path, and keeping it
/// is the level above the shell M23 removes. Reached by handing the demo's
/// overflow to another owner, which is what a graph built by hand without one
/// looks like to the tank.
#[test]
fn a_tank_over_its_brim_with_no_overflow_of_its_own_is_refused() {
    let mut engine = build(DEMO);
    let edge = engine
        .graph
        .edge_ids()
        .find(|e| engine.graph.pipe(*e).name == SPILL)
        .unwrap();
    let elsewhere = engine.graph.find_node("supply_tank").unwrap();
    // The supply tank now "owns" two overflows; it finds its own first.
    engine.graph.pipe_mut(edge).leak = LeakRole::Overflow { owner: elsewhere };
    let mut error = None;
    for _ in 0..FIRST_SPILL {
        if let Err(e) = engine.tick() {
            error = Some(e.to_string());
            break;
        }
    }
    let error = error.expect("the tank reaches its brim and has nowhere to spill");
    assert_eq!(
        engine.snapshot().tick,
        FIRST_SPILL - 1,
        "it fails on the spilling tick"
    );
    assert!(error.contains("filled past its brim"), "{error}");
    assert!(error.contains(RECEIVER), "{error}");
}
