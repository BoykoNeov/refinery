//! M63.4 — a column flooded with water (docs/DESIGN.md §68.4, ledger row C13):
//! a crude tank with free water at its bottom drains downhill into a wet
//! column. The tank's outlet draws its water first (M62.3), which the column
//! takes whole by its water draw (§67.17 rule 10); on the tick the water runs
//! out the column is fed the last water and the first oil together, and from
//! the next on oil. That mixed tick's water share is whatever the tank had
//! left — measured on this plant: 14 % with 4 % water in the tank, which the
//! column separates; 34–67 % a few kilograms of water later, which its solve
//! refuses. Refused at least `FLOODED_WATER_SHARE` water, the column passes the
//! feed through for the tick, its water to the water draw and its oil to the
//! bottoms, and fractionates the oil from the next tick on.

use refinery_core::snapshot::{EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::Engine;

/// The plant, its tank holding `water` of its mass as free water.
fn slug_column(solver: &str, water: f64) -> String {
    let diesel = 0.32 - (water - 0.04);
    format!(
        r#"[meta]
name = "slug_column"
description = "M63.4 gate: a crude tank with free water at its bottom drains downhill into a wet column."

[simulation]
dt = 0.5

[fidelity]
flow = "newton"
thermo = "trouton"
boiloff = "flash"
line_flash = "equilibrium"
separation = "heat_balance"

[[components]]
name = "naphtha"
tb_c = 120.0
molar_mass_kg_per_mol = 0.11
density_kg_per_m3 = 720.0
cp_j_per_kg_k = 2150.0

[[components]]
name = "kerosene"
tb_c = 200.0
molar_mass_kg_per_mol = 0.170
density_kg_per_m3 = 800.0
cp_j_per_kg_k = 2000.0

[[components]]
name = "diesel"
tb_c = 280.0
molar_mass_kg_per_mol = 0.230
density_kg_per_m3 = 850.0
cp_j_per_kg_k = 1950.0

[[components]]
name = "water"
tb_c = 100.0
molar_mass_kg_per_mol = 0.018015
density_kg_per_m3 = 998.0
cp_j_per_kg_k = 4184.0
dh_vap_j_per_mol = 40650.0
cp_vapour_j_per_kg_k = 2000.0
free_water = true

[nodes.crude_tank]
type = "tank"
area_m2 = 3.0
height_m = 10.0
initial_level_m = 8.0
temperature_c = 30.0
composition = {{ naphtha = 0.32, kerosene = 0.32, diesel = {diesel}, water = {water} }}

[nodes.feed_valve]
type = "valve"
kv = 20.0
opening = 0.5

[nodes.column]
type = "column"
pressure_bar = 1.01325
draws = [
    {{ outlet = "naphtha_tank", stage = 0, draw_ratio = 0.3 }},
    {{ outlet = "sour_water_tank", stage = 0, phase = "water" }},
    {{ outlet = "bottoms_tank", stage = 6 }},
]

[nodes.column.cascade]
stages = 6
feed_stage = 3
reflux_ratio = 2.0

[nodes.naphtha_tank]
type = "tank"
area_m2 = 10.0
height_m = 10.0
initial_level_m = 0.5
temperature_c = 40.0
composition = {{ naphtha = 1.0 }}

[nodes.sour_water_tank]
type = "tank"
area_m2 = 10.0
height_m = 10.0
initial_level_m = 0.5
temperature_c = 40.0
composition = {{ water = 1.0 }}

[nodes.bottoms_tank]
type = "tank"
area_m2 = 10.0
height_m = 10.0
initial_level_m = 0.5
temperature_c = 40.0
composition = {{ kerosene = 0.5, diesel = 0.5 }}

[[pipes]]
name = "tank_outlet"
from = "crude_tank"
to = "feed_valve"
length_m = 5.0
diameter_m = 0.10
elevation_change_m = -20.0

[[pipes]]
name = "feed_line"
from = "feed_valve"
to = "column"
length_m = 10.0
diameter_m = 0.10

[[pipes]]
name = "naphtha_line"
from = "column"
to = "naphtha_tank"
length_m = 10.0
diameter_m = 0.10

[[pipes]]
name = "water_draw"
from = "column"
to = "sour_water_tank"
length_m = 10.0
diameter_m = 0.10

[[pipes]]
name = "bottoms_line"
from = "column"
to = "bottoms_tank"
length_m = 10.0
diameter_m = 0.10
"#
    )
    .replace("flow = \"newton\"", &format!("flow = \"{solver}\""))
}

/// A rounding trace of water is no free water (M62.2, docs/DESIGN.md §67.10).
const TRACE: f64 = 1e-9;

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn edge<'a>(s: &'a Snapshot, name: &str) -> &'a EdgeSnapshot {
    s.edges.iter().find(|e| e.name == name).unwrap()
}

/// What the column was fed each tick, and how it answered: `(water share,
/// flooded reason, fractionated)` for ticks 1..=`ticks`, the run checked to
/// converge every tick.
fn run(solver: &str, water: f64, ticks: u32) -> Vec<(f64, Option<String>, bool)> {
    let src = slug_column(solver, water);
    let mut engine: Engine =
        refinery_scenarios::build_engine(&refinery_scenarios::load_str(&src).unwrap()).unwrap();
    let mut out = Vec::new();
    for t in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("{solver}, {water}: tick {t}: {e}"));
        let s = engine.snapshot();
        assert!(s.solver.converged, "{solver}, {water}: tick {t}");
        let feed = &edge(&s, "feed_line").stream;
        let share = feed.composition.fractions()[3];
        let column = node(&s, "column");
        out.push((
            share,
            column.column_flooded.clone(),
            column.column_operation.is_some(),
        ));
        if share > TRACE && share < 1.0 && column.column_flooded.is_some() {
            // The flooded tick: the water draw takes the feed's water, the
            // bottoms its oil, the naphtha nothing.
            let flow = feed.mass_flow.value();
            let water_draw = edge(&s, "water_draw").stream.mass_flow.value();
            let bottoms = edge(&s, "bottoms_line").stream.mass_flow.value();
            let naphtha = edge(&s, "naphtha_line").stream.mass_flow.value();
            assert!(
                (water_draw - share * flow).abs() < 1e-6 * flow,
                "{solver}: the water draw {water_draw} of {} kg/s of water",
                share * flow
            );
            assert!(
                (bottoms - (1.0 - share) * flow).abs() < 1e-6 * flow,
                "{solver}: the bottoms {bottoms} of {} kg/s of oil",
                (1.0 - share) * flow
            );
            assert_eq!(naphtha, 0.0, "{solver}: the naphtha draw");
        }
    }
    out
}

/// The tick the tank's water runs out, and what the column did on it and on
/// the ticks around it.
fn the_slugs_end(ticks: &[(f64, Option<String>, bool)]) -> usize {
    let at: Vec<usize> = (0..ticks.len())
        .filter(|&i| ticks[i].0 > TRACE && ticks[i].0 < 1.0)
        .collect();
    assert_eq!(at.len(), 1, "one mixed tick: {at:?}");
    let end = at[0];
    assert!(end > 10, "the water comes first");
    for (share, flooded, _) in &ticks[..end] {
        assert_eq!(*share, 1.0);
        assert_eq!(
            flooded, &None,
            "an all-water feed is rule 10's, not a flood"
        );
    }
    for (share, flooded, fractionated) in &ticks[end + 1..] {
        assert!(*share <= TRACE, "{share} water after the slug");
        assert_eq!(flooded, &None);
        assert!(fractionated, "the oil after the slug is fractionated");
    }
    end
}

/// **The slug's last tick floods the column, and the run goes on** — 51 % of
/// that tick's feed is water: the column's solve refuses it, so it passes the
/// feed through, its refusal kept in the snapshot; the tick before took
/// water, the ticks after oil, fractionated from a cold start.
#[test]
fn a_water_slug_running_out_floods_the_column_for_one_tick() {
    let ticks = run("newton", 0.04004, 400);
    let end = the_slugs_end(&ticks);
    let (share, flooded, fractionated) = &ticks[end];
    assert!(
        (0.4..0.6).contains(share),
        "the mixed tick is {share} water"
    );
    let reason = flooded.as_deref().expect("the column flooded");
    assert!(reason.contains("heat-balance cascade"), "{reason}");
    assert!(!fractionated);
}

/// **A slug that ends dry enough is separated** — with 4 % water in the tank
/// the mixed tick is 14 % water: under the share, and the column separates it.
#[test]
fn a_slug_that_ends_dry_enough_is_separated() {
    let ticks = run("newton", 0.04, 400);
    let end = the_slugs_end(&ticks);
    let (share, flooded, fractionated) = &ticks[end];
    assert!(*share < 0.15, "the mixed tick is {share} water");
    assert_eq!(flooded, &None);
    assert!(fractionated);
}

/// **Both fidelities flood the same tick.**
#[test]
fn slow_the_slug_floods_the_column_on_the_game_solver() {
    let ticks = run("simple", 0.04004, 400);
    let end = the_slugs_end(&ticks);
    assert!(ticks[end].1.is_some(), "the game solver's column flooded");
}
