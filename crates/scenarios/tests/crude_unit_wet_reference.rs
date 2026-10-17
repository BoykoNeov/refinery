//! M62.5 — water through the column (`scenarios/crude_unit_wet.toml`,
//! docs/DESIGN.md §67.17): the boiling crude unit with 0.3 % water in its
//! crude. The column's wet equations are gated against an independent solve in
//! `refinery-solvers`' `reference/wet_column.rs`; these gates hold the shipped
//! plant to what it is for, and the loader to its refusals.

use refinery_core::components::Composition;
use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::traits::ThermoModel;
use refinery_core::units::{Kelvin, Pascal};
use refinery_core::Engine;
use refinery_solvers::{bubble_temperature, MoleFractions, TroutonThermo};

const PLANT: &str = include_str!("../../../scenarios/crude_unit_wet.toml");
const WATER_DRAW: &str = "    { outlet = \"sour_water_tank\", stage = 0, phase = \"water\" },\n";

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the plant parses");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant builds: {e}"))
}

fn refusal(src: &str) -> String {
    let file = refinery_scenarios::load_str(src).expect("the plant parses");
    match refinery_scenarios::build_engine(&file) {
        Ok(_) => panic!("the plant must be refused"),
        Err(e) => e.to_string(),
    }
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

/// **The crude's water rides up the column and leaves by its own draw.**
/// Every kilogram the crude brings leaves the condenser by the water draw — no
/// tray is cold enough to hold free water, so none reaches the bottoms; the
/// naphtha carries none. The draw leaves at the WET bubble point of the
/// naphtha over its water (computed here independently from the drawn
/// naphtha), and every tray from the top down to the feed stands hotter than
/// water's dew point at its share of the vapour. Measured at 6 000 ticks:
/// 112.83 kg/s of crude, 0.3385 kg/s of water at 93.72 °C, the top tray at
/// 166.2 °C.
#[test]
fn the_crude_unit_decants_its_crudes_water_at_the_condenser() {
    let mut engine = build(PLANT);
    run(&mut engine, 100);
    let s = engine.snapshot();
    let slate = engine.slate.clone();
    let water = slate.free_water().expect("the slate has its water");
    let crude = edge(&s, "crude_line");
    // The charge, 1.4 % vapour through the feed valve: the liquid law at its
    // inlet's mixture density. A valve chokes only an ALL-vapour stream in
    // M62.6b (docs/DESIGN.md §67.19); part-liquid choking is M62.6c's, and
    // re-pins this.
    assert!(
        (crude.stream.mass_flow.value() / 112.8327 - 1.0).abs() < 1e-4,
        "the charge is {} kg/s",
        crude.stream.mass_flow.value()
    );
    let crude_water = crude.stream.mass_flow.value() * crude.stream.composition.fractions()[water];
    let draw = edge(&s, "water_draw");
    assert!(
        (draw.stream.mass_flow.value() - crude_water).abs() < 1e-6 * crude_water,
        "the water draw takes {} kg/s of the crude's {crude_water}",
        draw.stream.mass_flow.value()
    );
    assert_eq!(draw.stream.composition.fractions()[water], 1.0);
    let naphtha = edge(&s, "naphtha_suction");
    assert_eq!(naphtha.stream.composition.fractions()[water], 0.0);
    for product in ["kerosene", "diesel", "gas_oil", "residue"] {
        let suction = edge(&s, &format!("{product}_suction"));
        assert_eq!(
            suction.stream.composition.fractions()[water],
            0.0,
            "{product} carries water"
        );
    }

    // The condenser: the naphtha's oil over the water decanted with it.
    let column = node(&s, "column");
    let pressure = Pascal(column.pressure_pa);
    let thermo = TroutonThermo::new();
    let mut masses: Vec<f64> = naphtha
        .stream
        .composition
        .fractions()
        .iter()
        .map(|x| x * naphtha.stream.mass_flow.value())
        .collect();
    masses[water] += draw.stream.mass_flow.value();
    let overhead =
        MoleFractions::from_mass(&Composition::from_weights(&masses).unwrap(), &slate).unwrap();
    let wet_bubble =
        bubble_temperature(&slate, &thermo, &overhead, pressure, "the overhead").unwrap();
    assert!(
        (draw.stream.temperature.value() - wet_bubble.value()).abs() < 1e-6,
        "the water leaves at {} K, the overhead's wet bubble point is {} K",
        draw.stream.temperature.value(),
        wet_bubble.value()
    );
    assert_eq!(naphtha.stream.temperature, draw.stream.temperature);

    // Every tray above the feed holds the crude's water as vapour, under its
    // saturation: y_w = W/(V + W) < K_w(T).
    let operation = column.column_operation.as_ref().unwrap();
    let water_moles = draw.stream.mass_flow.value() / slate.get(water).molar_mass.value();
    let feed_stage = 8;
    for j in 0..feed_stage {
        let oil = operation.stage_vapour_mol_s[j];
        let share = water_moles / (oil + water_moles);
        let k_water = thermo
            .k_value(
                &slate,
                water,
                Kelvin(operation.stage_temperatures_k[j]),
                pressure,
            )
            .unwrap();
        assert!(
            share < k_water,
            "stage {}: water is {share} of the vapour, saturation {k_water}",
            j + 1
        );
    }

    // The decanted water reaches its tank liquid.
    let vent = edge(&s, "sour_water_tank__boiloff_vent");
    assert_eq!(vent.stream.mass_flow.value(), 0.0, "the water tank vents");
}

/// **The column's first law closes on the plant's own books**, water and all
/// (M62.4a's gate, `heat_on_boiling_reference.rs`, on the wet plant): the
/// feed's enthalpy as the engine carries it — the transfer line's latent heat
/// included, its water's at Kirchhoff's λ(T) — the reboiler in, the condenser
/// and the pumparounds out, and every draw's enthalpy, the water draw's among
/// them, each from the engine's enthalpy model on the published stream. The
/// counterfactuals: without the water draw's enthalpy, and without the feed's
/// latent heat, the books miss by far more than the residual.
#[test]
fn the_wet_column_closes_its_first_law_on_the_plants_books() {
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
    let (mut worst, mut without_water, mut without_latent) = (0.0f64, f64::INFINITY, f64::INFINITY);
    for _ in 0..100 {
        run(&mut engine, 1);
        let s = engine.snapshot();
        let feed = edge(&s, "transfer_line__downstream");
        let arriving = flux(&engine, feed);
        let latent = feed.stream.mass_flow.value() * feed.stream.latent.unwrap().value();
        let water = flux(&engine, edge(&s, "water_draw"));
        let mut leaving = water;
        for product in ["naphtha", "kerosene", "diesel", "gas_oil", "residue"] {
            leaving += flux(&engine, edge(&s, &format!("{product}_suction")));
        }
        let duty = node(&s, "column").column_duty.unwrap();
        let books = arriving + duty.reboiler_w - duty.condenser_w - pumparounds - leaving;
        let gross =
            arriving.abs() + duty.reboiler_w + duty.condenser_w + pumparounds + leaving.abs();
        worst = worst.max(books.abs() / gross);
        without_water = without_water.min((books + water).abs() / gross);
        without_latent = without_latent.min((books - latent).abs() / gross);
    }
    // Measured: 1.8e-16 relative; 3.9e-4 without the water draw, 4.2e-2
    // without the feed's latent heat.
    assert!(
        worst <= 1e-12,
        "the column's first law: worst residual {worst:e}"
    );
    assert!(
        without_water > 1e-5 && without_latent > 1e-2,
        "the books close without the water draw ({without_water:e}) or the feed's latent \
         heat ({without_latent:e})"
    );
}

/// **Both fidelities agree** on the shipped plant.
#[test]
fn slow_the_wet_crude_unit_runs_on_both_fidelities() {
    let mut answers = Vec::new();
    for flow in ["newton", "simple"] {
        let mut engine = build(&PLANT.replace("flow = \"newton\"", &format!("flow = \"{flow}\"")));
        run(&mut engine, 150);
        let s = engine.snapshot();
        answers.push((
            edge(&s, "crude_line").stream.mass_flow.value(),
            edge(&s, "water_draw").stream.mass_flow.value(),
            edge(&s, "water_draw").stream.temperature.value(),
        ));
    }
    let (a, b) = (answers[0], answers[1]);
    assert!(
        (a.0 - b.0).abs() < 1e-6 * a.0
            && (a.1 - b.1).abs() < 1e-6 * a.1
            && (a.2 - b.2).abs() < 1e-4,
        "newton {a:?}, simple {b:?}"
    );
}

/// **The water draw's refusals, by name** (§67.17 rule 9): a wet slate's
/// column declares one; a dry slate's may not; only the heat-balance cascade
/// carries a water balance; it is listed second, after the distillate, and
/// declares no ratio.
#[test]
fn the_water_draw_is_required_where_there_is_water_and_refused_elsewhere() {
    let missing = refusal(&PLANT.replace(WATER_DRAW, ""));
    assert!(missing.contains("has no water draw"), "{missing}");

    let dry = refusal(&PLANT.replace("free_water = true\n", ""));
    assert!(dry.contains("no component declares free_water"), "{dry}");

    let cascade =
        refusal(&PLANT.replace("separation = \"heat_balance\"", "separation = \"cascade\""));
    // Its own refusal, not the pumparounds' (which also names heat_balance).
    assert!(
        cascade.contains("would dissolve it in the oil as one more cut"),
        "{cascade}"
    );

    let rated = refusal(&PLANT.replace(
        "phase = \"water\" }",
        "phase = \"water\", draw_ratio = 0.01 }",
    ));
    assert!(rated.contains("declares no draw_ratio"), "{rated}");

    let third = refusal(&PLANT.replace(WATER_DRAW, "").replace(
        "    { outlet = \"diesel_pump\"",
        &format!("{WATER_DRAW}    {{ outlet = \"diesel_pump\""),
    ));
    assert!(third.contains("listed second"), "{third}");
}
