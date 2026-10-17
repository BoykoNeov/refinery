//! M62.4 — heat on a boiling stream (docs/DESIGN.md §67.4): on a plant that
//! selects `line_flash = "equilibrium"` a furnace and a cooler settle what
//! passes by the line flash from their inflow's enthalpy and their heat, and a
//! column separates its feed whole — the latent heat the transfer line carries
//! included — and sends each draw out as a liquid at its own stage.
//!
//! The flash itself is held to a hand calculation in
//! `crates/solvers/tests/reference/line_flash.rs`; these gates hold what each
//! unit hands it and what it does with the answer. The plants are inline: the
//! crude unit (`scenarios/crude_unit.toml`) under the line flash, its transfer
//! line widened to 40 cm (8 cm chokes on its vapour, M62.0 finding 10), its
//! tubes set never to burst; and the flashing rundown's naphtha through a
//! cooler.

use refinery_core::components::{Composition, Slate};
use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::traits::{EnthalpyModel, LineFlashModel};
use refinery_core::units::{Kelvin, Pascal, SquareMeter};
use refinery_core::Engine;
use refinery_solvers::line_flash::EquilibriumLineFlash;
use refinery_solvers::TroutonThermo;

const CRUDE_UNIT: &str = include_str!("../../../scenarios/crude_unit.toml");

/// The crude unit as a plant that boils (M62.0 finding 10).
fn crude_unit_flashing() -> String {
    let src = CRUDE_UNIT
        .replace(
            "boiloff = \"none\"",
            "boiloff = \"flash\"\nline_flash = \"equilibrium\"",
        )
        .replace("tube_failure_c = 550.0", "tube_failure_c = 1951.1")
        .replace(
            "name = \"transfer_line\"\nfrom = \"furnace\"\nto = \"column\"\nlength_m = 15.0\ndiameter_m = 0.08",
            "name = \"transfer_line\"\nfrom = \"furnace\"\nto = \"column\"\nlength_m = 15.0\ndiameter_m = 0.40",
        );
    assert!(src.contains("diameter_m = 0.40") && src.contains("line_flash"));
    src
}

/// A cooler removing `duty_mw` [MW] from what passes.
fn cooler(duty_mw: f64) -> String {
    format!(
        "type = \"cooler\"
duty_mw = {duty_mw}"
    )
}

/// A small furnace firing `duty_mw` [MW], its tubes set never to burst.
fn furnace(duty_mw: f64) -> String {
    format!(
        "type = \"furnace\"
duty_mw = {duty_mw}
coil_heat_capacity_mj_per_k = 1.0
         coil_ua_kw_per_k = 30.0
coil_temperature_c = 120.0
flame_temperature_c = 1951.1
         tube_failure_c = 1951.1
tube_rupture_area_cm2 = 1.0
fluid_heating_value_mj_per_kg = 44.0"
    )
}

/// The flashing rundown's naphtha, boiled through a control valve, then through
/// `unit` (a node definition's keys), to a destination at atmospheric pressure.
fn rundown_through(solver: &str, unit: &str) -> String {
    format!(
        r#"
[meta]
name = "cooled_rundown"

[simulation]
dt = 0.5

[fidelity]
flow = "{solver}"
thermo = "trouton"
line_flash = "equilibrium"

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

[nodes.rundown]
type = "source"
pressure_bar = 3.0
temperature_c = 115.0
composition = {{ light_naphtha = 0.7, heavy_naphtha = 0.3 }}

[nodes.rundown_valve]
type = "valve"
kv = 40.0
opening = 0.5

[nodes.valve_outlet]
type = "junction"

[nodes.unit]
{unit}

[nodes.product]
type = "sink"
pressure_bar = 1.01325
temperature_c = 40.0
composition = {{ light_naphtha = 0.5, heavy_naphtha = 0.5 }}

[[pipes]]
name = "rundown_line"
from = "rundown"
to = "rundown_valve"
length_m = 30.0
diameter_m = 0.08

[[pipes]]
name = "valve_spool"
from = "rundown_valve"
to = "valve_outlet"
length_m = 2.0
diameter_m = 0.08

[[pipes]]
name = "flash_line"
from = "valve_outlet"
to = "unit"
length_m = 40.0
diameter_m = 0.15

[[pipes]]
name = "cooled_line"
from = "unit"
to = "product"
length_m = 20.0
diameter_m = 0.15
"#
    )
}

fn build(src: &str) -> Result<Engine, String> {
    let file = refinery_scenarios::load_str(src).map_err(|e| e.to_string())?;
    refinery_scenarios::build_engine(&file).map_err(|e| e.to_string())
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

/// `ṁ·(h(T) + λ)` [W], the enthalpy a stream carries, its latent heat included.
fn total_flux(engine: &Engine, e: &EdgeSnapshot) -> f64 {
    engine
        .enthalpy()
        .stream_enthalpy_flux(&engine.slate, &e.stream)
        .unwrap()
        .value()
}

/// The line flash's state for `specific_enthalpy` [J/kg] of `composition` at
/// `pressure`, from first principles: the liquid-equivalent temperature on the
/// engine's datum, settled by an independent instance of the model.
fn flashed(
    slate: &Slate,
    enthalpy: &dyn EnthalpyModel,
    composition: &Composition,
    specific_enthalpy: f64,
    pressure: Pascal,
) -> refinery_core::traits::Settled {
    let liquid_equivalent = enthalpy
        .temperature_from_enthalpy(slate, composition, specific_enthalpy, 1.0)
        .unwrap();
    EquilibriumLineFlash
        .settle(
            slate,
            composition,
            liquid_equivalent,
            pressure,
            &TroutonThermo::new(),
            enthalpy,
        )
        .unwrap()
}

fn coil_temperature(s: &Snapshot, name: &str) -> f64 {
    match &node(s, name).kind {
        NodeKind::Furnace { coil, .. } => coil.temperature.value(),
        other => panic!("not a furnace: {other:?}"),
    }
}

/// **A furnace's outlet is the isenthalpic flash of what its coil gave its
/// inflow, at its own pressure** — and the crude boils in its tubes. The coil's
/// first law gives the fluid `Q − C·ΔT_c/dt − flue` (§37); added to the
/// inflow's enthalpy and flashed at the furnace's solved pressure by an
/// independent instance of the model, it is the furnace's published state: its
/// temperature, its vapour share, and the latent heat its outlet carries.
///
/// The counterfactual is the liquid-equivalent temperature itself — what the
/// furnace published before M62.4 (M62.0 finding 10: 341 against 284 °C).
#[test]
fn a_boiling_furnace_leaves_at_the_flash_of_what_its_coil_gave() {
    // The crude boils in the tubes; the naphtha arrives boiling.
    furnace_gate(
        &crude_unit_flashing(),
        "furnace",
        "crude_line",
        "transfer_line",
        false,
    );
    furnace_gate(
        &rundown_through("newton", &furnace(0.3)),
        "unit",
        "flash_line",
        "cooled_line",
        true,
    );
}

fn furnace_gate(
    src: &str,
    furnace_name: &str,
    inflow_name: &str,
    outlet_name: &str,
    inflow_boils: bool,
) {
    let mut engine = build(src).unwrap();
    run(&mut engine, 200);
    let before = engine.snapshot();
    run(&mut engine, 1);
    let now = engine.snapshot();
    let slate = engine.slate.clone();
    let enthalpy = engine.enthalpy();
    let (duty, capacity) = match &node(&now, furnace_name).kind {
        NodeKind::Furnace { duty, coil, .. } => (duty.value(), coil.heat_capacity.value()),
        _ => unreachable!(),
    };
    let dt = engine.dt().value();
    let flue = node(&now, furnace_name).flue_loss_w.unwrap();
    let to_fluid = duty
        - capacity
            * (coil_temperature(&now, furnace_name) - coil_temperature(&before, furnace_name))
            / dt
        - flue;
    let inflow = edge(&now, inflow_name);
    assert_eq!(
        inflow.stream.latent.is_some(),
        inflow_boils,
        "{furnace_name}: the inflow's vapour is not what the case is for"
    );
    let mass_flow = inflow.stream.mass_flow.value();
    let arriving = total_flux(&engine, inflow);
    let composition = &inflow.stream.composition;
    let pressure = Pascal(node(&now, furnace_name).pressure_pa);
    let expected = flashed(
        &slate,
        enthalpy,
        composition,
        (arriving + to_fluid) / mass_flow,
        pressure,
    );

    let furnace = node(&now, furnace_name);
    let outlet = edge(&now, outlet_name);
    let share = expected
        .vapour
        .expect("the crude must boil in the furnace's tubes");
    assert!(
        (furnace.temperature_k - expected.temperature.value()).abs() < 1e-9 * furnace.temperature_k,
        "the furnace stands at {} K, the flash of what its coil gave at {} K",
        furnace.temperature_k,
        expected.temperature.value()
    );
    let published = furnace.vapour_fraction.unwrap();
    assert!(
        (published - share.mass_fraction).abs() < 1e-9,
        "vapour share {published} against {}",
        share.mass_fraction
    );
    let latent = outlet.stream.latent.unwrap().value();
    assert!(
        (latent - share.latent.value()).abs() < 1e-9 * latent,
        "the outlet carries {latent} J/kg of latent heat against {}",
        share.latent.value()
    );
    // The first law at the furnace, from the published states alone.
    let leaving = mass_flow
        * (enthalpy
            .specific_enthalpy(&slate, composition, Kelvin(furnace.temperature_k))
            .unwrap()
            .value()
            + latent);
    assert!(
        (leaving - arriving - to_fluid).abs() < 1e-9 * leaving.abs(),
        "the fluid left with {leaving} W of the {arriving} + {to_fluid} W it had"
    );
    // Measured at tick 201: the crude unit's furnace at 291.8 °C against its
    // 350.1 °C liquid-equivalent, 43 % vapour; the rundown's at 94.9 °C against
    // 131.4 °C, 26 % vapour from the 14 % that arrived.
    assert!(
        share.liquid_equivalent.value() - furnace.temperature_k > 10.0,
        "the boiling stream must stand well below its liquid-equivalent temperature \
         ({} against {} K), or the gate cannot tell a flash from none",
        furnace.temperature_k,
        share.liquid_equivalent.value()
    );
}

/// **A cooler on a boiling stream condenses part of it**: its outlet is the
/// isenthalpic flash of what arrived, latent heat included, less its duty, at
/// its own pressure — on both fidelities. Its vapour share falls but stays
/// above zero at this duty, and its counterfactual (the duty taken off the
/// sensible heat alone, the vapour carried through) would leave it hotter than
/// it arrived would allow.
#[test]
fn a_cooler_condenses_part_of_a_boiling_stream() {
    let mut answers = Vec::new();
    for solver in ["newton", "simple"] {
        let mut engine = build(&rundown_through(solver, &cooler(0.2))).unwrap();
        run(&mut engine, 200);
        let s = engine.snapshot();
        let slate = engine.slate.clone();
        let inflow = edge(&s, "flash_line");
        let mass_flow = inflow.stream.mass_flow.value();
        let arriving_share = inflow
            .stream
            .vapour_fraction
            .expect("the line boils after its valve");
        let arriving = total_flux(&engine, inflow);
        let cooler = node(&s, "unit");
        let expected = flashed(
            &slate,
            engine.enthalpy(),
            &inflow.stream.composition,
            (arriving - 0.2e6) / mass_flow,
            Pascal(cooler.pressure_pa),
        );
        assert!(
            (cooler.temperature_k - expected.temperature.value()).abs()
                < 1e-9 * cooler.temperature_k,
            "{solver}: the cooler stands at {} K, the flash at {} K",
            cooler.temperature_k,
            expected.temperature.value()
        );
        let share = expected
            .vapour
            .expect("this duty condenses only part of the vapour");
        let published = cooler.vapour_fraction.unwrap();
        assert!(
            (published - share.mass_fraction).abs() < 1e-9,
            "{solver}: {published}"
        );
        assert!(
            published < 0.8 * arriving_share,
            "{solver}: the cooler must condense a real part of what arrived: {published} of \
             {arriving_share}"
        );
        let out = edge(&s, "cooled_line");
        assert!(
            (out.stream.latent.unwrap().value() - share.latent.value()).abs()
                < 1e-9 * share.latent.value(),
            "{solver}: the cooled line carries the cooler's latent heat"
        );
        answers.push((mass_flow, cooler.temperature_k, published));
    }
    let (newton, simple) = (answers[0], answers[1]);
    assert!(
        (newton.0 - simple.0).abs() < 1e-6 * newton.0
            && (newton.1 - simple.1).abs() < 1e-6
            && (newton.2 - simple.2).abs() < 1e-6,
        "the fidelities disagree: {newton:?} against {simple:?}"
    );
}

/// **A column on a boiling plant takes the feed's latent heat and draws
/// liquids.** Its published temperature is the feed's liquid-equivalent, what
/// its separation is handed; every draw leaves at its stage with no vapour of
/// its own (M62.0 finding 10: before M62.4 every draw carried the feed's 43 %);
/// and the column's first law closes on the published streams and duties — the
/// feed's enthalpy with its latent heat, the reboiler in, the condenser and the
/// pumparounds out, the draws' enthalpy — every tick. Without the transfer
/// line's latent heat the books miss by that heat, which the counterfactual
/// shows is far larger than the residual.
#[test]
fn a_column_on_a_boiling_plant_takes_its_feed_whole_and_draws_liquids() {
    let mut engine = build(&crude_unit_flashing()).unwrap();
    let slate = engine.slate.clone();
    let pumparounds: f64 = match &node(&engine.snapshot(), "column").kind {
        NodeKind::Column {
            cascade: Some(spec),
            ..
        } => spec.pumparounds.iter().map(|p| p.duty.value()).sum(),
        _ => panic!("the crude unit's column has a cascade"),
    };
    let draws = [
        "naphtha_draw",
        "kerosene_draw",
        "diesel_draw",
        "gas_oil_draw",
        "residue_draw",
    ];
    let (mut worst, mut worst_without) = (0.0f64, f64::INFINITY);
    for t in 1..=300 {
        run(&mut engine, 1);
        let s = engine.snapshot();
        let feed = edge(&s, "transfer_line__downstream");
        let latent = feed.stream.latent.expect("the transfer line boils").value();
        let mass_flow = feed.stream.mass_flow.value();
        let arriving = total_flux(&engine, feed);
        let column = node(&s, "column");
        // The column stands at its feed's liquid-equivalent temperature.
        let liquid_equivalent = engine
            .enthalpy()
            .temperature_from_enthalpy(&slate, &feed.stream.composition, arriving / mass_flow, 1.0)
            .unwrap();
        assert!(
            (column.temperature_k - liquid_equivalent.value()).abs() < 1e-9 * column.temperature_k,
            "tick {t}: the column stands at {} K, its feed's liquid-equivalent at {} K",
            column.temperature_k,
            liquid_equivalent.value()
        );
        assert!(
            column.vapour_fraction.is_none(),
            "tick {t}: a column holds no line vapour"
        );
        let mut leaving = 0.0;
        for name in draws {
            let draw = edge(&s, name);
            assert!(
                draw.stream.vapour_fraction.is_none() && draw.stream.latent.is_none(),
                "tick {t}: draw '{name}' carries vapour"
            );
            leaving += total_flux(&engine, draw);
        }
        let duty = column
            .column_duty
            .expect("the heat-balance column reports its duties");
        let books = arriving + duty.reboiler_w - duty.condenser_w - pumparounds - leaving;
        let gross =
            arriving.abs() + duty.reboiler_w + duty.condenser_w + pumparounds + leaving.abs();
        worst = worst.max(books.abs() / gross);
        worst_without = worst_without.min((books - mass_flow * latent).abs() / gross);
    }
    // Measured: 1.7e-16 relative, and 4.2e-2 without the latent heat.
    assert!(
        worst <= 1e-12,
        "the column's first law: worst residual {worst:e} relative"
    );
    assert!(
        worst_without > 1e-2,
        "without the transfer line's latent heat the books still close ({worst_without:e})"
    );
}

/// **What a boiling plant still refuses, by name** (§67.4): a furnace whose
/// tubes can burst — the hole would be a two-phase jet, which chokes — and a
/// column on a slate with free water, which its stages do not balance yet
/// (M62.5). The furnace's outlet carries its dormant burn-out hole, so a
/// puncture of it is refused too; a repair (area zero) changes nothing and is
/// taken.
#[test]
fn a_boiling_plant_refuses_a_bursting_furnace_a_wet_column_and_a_puncture() {
    let bursting =
        crude_unit_flashing().replace("tube_failure_c = 1951.1", "tube_failure_c = 550.0");
    let wet = crude_unit_flashing().replace(
        "[[components]]\nname = \"residue\"",
        "[[components]]\nname = \"water\"\ntb_c = 100.0\nmolar_mass_kg_per_mol = 0.018015\n\
         density_kg_per_m3 = 998.0\ncp_j_per_kg_k = 4186.0\ndh_vap_j_per_mol = 40650.0\n\
         cp_vapour_j_per_kg_k = 2000.0\nfree_water = true\n\n[[components]]\nname = \"residue\"",
    );
    assert!(wet.contains("free_water"));
    for (src, expected) in [(bursting, "tubes can burst"), (wet, "free_water")] {
        let err = match build(&src) {
            Ok(_) => panic!("must refuse ({expected})"),
            Err(e) => e,
        };
        assert!(
            err.contains("line_flash") && err.contains(expected),
            "expected a line-flash refusal naming {expected}: {err}"
        );
    }

    let mut engine = build(&crude_unit_flashing()).unwrap();
    let transfer_line = edge(&engine.snapshot(), "transfer_line").id;
    let err = engine
        .apply(Command::PuncturePipe {
            edge: transfer_line,
            area: SquareMeter(1e-4),
        })
        .expect_err("a hole in a boiling line is refused");
    assert!(err.to_string().contains("two-phase jet"), "{err}");
    engine
        .apply(Command::PuncturePipe {
            edge: transfer_line,
            area: SquareMeter(0.0),
        })
        .expect("a repair changes nothing and is taken");
}
