//! M62.2 — the wet rundown (`scenarios/wet_rundown.toml`, docs/DESIGN.md
//! §67.2): the flashing rundown's naphtha with 3 % free water by mass, let down
//! through a control valve into a vented tank. Free water stands in the line
//! after the valve beside the boiling oil, and in the tank, which sits on its
//! wet bubble point.
//!
//! The free-water flash itself is held to a hand calculation in
//! `crates/solvers/tests/reference/free_water.rs`. These gates hold the plant:
//! the line three-phase after the valve on that hand calculation, the tank on
//! its wet bubble point with water standing in it, the external energy and
//! mass books closing every tick, the two fidelities agreeing, and `free_water`
//! refused wherever nothing would read it.

use refinery_core::components::Slate;
use refinery_core::energy::T_REF;
use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{NodeSnapshot, Snapshot};
use refinery_core::traits::EnthalpyModel;
use refinery_core::units::P_ATM;
use refinery_core::Engine;
use refinery_solvers::{bubble_temperature, MoleFractions, TroutonThermo};

const PLANT: &str = include_str!("../../../scenarios/wet_rundown.toml");
const DT: f64 = 0.5;

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the scenario must parse");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant must build: {e}"))
}

fn on_solver(solver: &str) -> String {
    PLANT.replace("flow = \"newton\"", &format!("flow = \"{solver}\""))
}

fn tick(engine: &mut Engine, t: u64) {
    engine
        .tick()
        .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
    assert!(
        engine.snapshot().solver.converged,
        "tick {t} did not converge"
    );
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn refusal(source: &str) -> String {
    match refinery_scenarios::load_str(source) {
        Ok(file) => match refinery_scenarios::build_engine(&file) {
            Ok(_) => panic!("the plant must be refused"),
            Err(e) => e.to_string(),
        },
        Err(e) => e.to_string(),
    }
}

/// **Three phases in the line, on the hand calculation.** At 2 000 ticks the
/// junction after the valve stands at 85.71 °C and 1.504 bar, 12.77 % of the
/// mass vapour (water on IAPWS-IF97's saturation line since M63.2, §68.2) —
/// and the independent script, flashing the supply's composition at that same
/// temperature and pressure, finds it in the three-phase state, 1.67 % of the
/// mass free water beside the oil liquid, and the vapour share 0.127 735 012 0. The junction's `q` is the ENERGY's
/// (`settle_full`), the script's the flash's: on a mixture they meet at the
/// root, and they do here to the script's printed digits.
///
/// And the tank stands on its WET bubble point at atmospheric pressure, with
/// free water in it: free water acts at any amount, so the oil boils where its
/// vapour pressure and water's add up to the atmosphere — 74.41 °C (74.13 before M63.2), more than
/// 10 K below where the same oil boils dry. Since M62.3 the water is a layer
/// its outlet draws first (docs/DESIGN.md §67.3), so the tank holds less than
/// one tick's arriving water — about 0.002 % of its mass, where M62.2's mixed
/// tank held 1.6 % — and still stands on the same bubble point.
#[test]
fn the_line_stands_three_phase_after_the_valve_on_the_hand_calculation() {
    let mut engine = build(PLANT);
    for t in 1..=2000 {
        tick(&mut engine, t);
    }
    let s = engine.snapshot();
    let junction = node(&s, "valve_outlet");
    assert!(
        (junction.temperature_k / 358.855_459_081_289_5 - 1.0).abs() < 1e-9
            && (junction.pressure_pa / 150_410.346_041_178_88 - 1.0).abs() < 1e-9,
        "the junction settled at {} K, {} Pa — not the state the script was run at",
        junction.temperature_k,
        junction.pressure_pa
    );
    let q = junction.vapour_fraction.expect("the line boils");
    assert!(
        (q - 0.127_735_012_0).abs() < 1e-10,
        "the junction carries {q} of its mass as vapour, the script 0.127 735 012 0"
    );

    let slate = engine.slate.clone();
    let NodeKind::Tank(tank) = &node(&s, "rundown_tank").kind else {
        panic!("the tank is a tank");
    };
    let water = tank.composition.fractions()[2] * tank.mass.value();
    let x = MoleFractions::from_mass(&tank.composition, &slate).unwrap();
    assert!(
        x.standing_water(&slate).is_some(),
        "the tank holds {water} kg of water: none stands"
    );
    let line = &s
        .edges
        .iter()
        .find(|e| e.name == "flash_line")
        .unwrap()
        .stream;
    let arriving = line.mass_flow.value() * line.composition.fractions()[2] * DT;
    assert!(
        water < arriving,
        "the tank holds {water} kg of water, more than the {arriving} kg one tick brings:          its outlet does not draw the water layer first"
    );
    let thermo = TroutonThermo::new();
    let wet = bubble_temperature(&slate, &thermo, &x, P_ATM, "the tank").unwrap();
    assert!(
        (tank.temperature.value() - wet.value()).abs() < 1e-6,
        "the tank stands at {} K, its wet bubble point is {} K",
        tank.temperature.value(),
        wet.value()
    );
    // The same oil dry boils well above where the tank stands.
    let mut oil = x.fractions().to_vec();
    oil[2] = 0.0;
    let dry = bubble_temperature(
        &slate,
        &thermo,
        &MoleFractions::from_amounts(&oil).unwrap(),
        P_ATM,
        "the dry oil",
    )
    .unwrap();
    assert!(
        dry.value() - wet.value() > 10.0,
        "dry {} K against wet {} K: the water does not lower the boil",
        dry.value(),
        wet.value()
    );
}

/// Every holdup's internal energy [J] on the published state.
fn holdup_energy(s: &Snapshot, slate: &Slate) -> f64 {
    s.nodes
        .iter()
        .filter_map(|n| match &n.kind {
            NodeKind::Tank(t) => Some(
                t.mass.value()
                    * t.composition.mixture_cp(slate).value()
                    * (t.temperature.value() - T_REF.value()),
            ),
            _ => None,
        })
        .sum()
}

/// Every holdup's mass [kg] of one component.
fn holdup_component(s: &Snapshot, c: usize) -> f64 {
    s.nodes
        .iter()
        .filter_map(|n| match &n.kind {
            NodeKind::Tank(t) => Some(t.mass.value() * t.composition.fractions()[c]),
            _ => None,
        })
        .sum()
}

/// Net power [W] and per-component mass rates [kg/s] across the plant's
/// boundary, INTO it, from published state, and the gross enthalpy flow [W] the
/// residual is graded against — `flashing_rundown_reference.rs`'s books, with
/// the mass kept per component so the water's own books are seen.
fn boundary(
    s: &Snapshot,
    slate: &Slate,
    enthalpy: &dyn EnthalpyModel,
    latent: bool,
) -> (f64, Vec<f64>, f64) {
    let kind_of = |id| &s.nodes.iter().find(|n| n.id == id).unwrap().kind;
    let outside = |k: &NodeKind| {
        matches!(
            k,
            NodeKind::Source { .. } | NodeKind::Sink { .. } | NodeKind::Atmosphere
        )
    };
    let (mut power, mut mass, mut gross) = (0.0, vec![0.0; slate.len()], 0.0);
    for e in &s.edges {
        let flux = if latent {
            enthalpy
                .stream_enthalpy_flux(slate, &e.stream)
                .unwrap()
                .value()
        } else {
            enthalpy
                .enthalpy_flux(
                    slate,
                    &e.stream.composition,
                    e.stream.mass_flow,
                    e.stream.temperature,
                )
                .unwrap()
                .value()
        };
        let flows = e
            .stream
            .composition
            .fractions()
            .iter()
            .map(|w| w * e.stream.mass_flow.value());
        match (outside(kind_of(e.from)), outside(kind_of(e.to))) {
            (true, false) => {
                power += flux;
                mass.iter_mut().zip(flows).for_each(|(m, f)| *m += f);
            }
            (false, true) => {
                power += e.dissipation_w - flux;
                mass.iter_mut().zip(flows).for_each(|(m, f)| *m -= f);
            }
            (false, false) => power += e.dissipation_w,
            (true, true) => panic!("an edge with both ends outside the plant"),
        }
        gross += flux.abs();
    }
    (power, mass, gross)
}

/// **The plant's external energy books, and every component's mass books,
/// close on every tick** — water's among them: what the supply feeds is what
/// the tank holds plus what leaves by the product line and the vent, the vent
/// carrying the water the wet boil-off sized at the vapour's own latent heat.
/// The counterfactual is the energy sum with the latent terms left out, which
/// must not close.
#[test]
fn the_energy_and_every_components_books_close() {
    let mut engine = build(PLANT);
    let slate = engine.slate.clone();
    let mut previous = engine.snapshot();
    let (mut worst, mut worst_without, mut worst_mass) = (0.0f64, 0.0f64, 0.0f64);
    for t in 1..=1500 {
        tick(&mut engine, t);
        let now = engine.snapshot();
        let accumulation = (holdup_energy(&now, &slate) - holdup_energy(&previous, &slate)) / DT;
        let (power, mass_in, gross) = boundary(&now, &slate, engine.enthalpy(), true);
        let (power_sensible, _, _) = boundary(&now, &slate, engine.enthalpy(), false);
        worst = worst.max((accumulation - power).abs() / gross);
        worst_without = worst_without.max((accumulation - power_sensible).abs() / gross);
        let throughput: f64 = now
            .edges
            .iter()
            .map(|e| e.stream.mass_flow.value().abs())
            .sum();
        for (c, inflow) in mass_in.iter().enumerate() {
            let rate = (holdup_component(&now, c) - holdup_component(&previous, c)) / DT;
            worst_mass = worst_mass.max((rate - inflow).abs() / throughput);
        }
        previous = now;
    }
    assert!(
        worst <= 1e-8,
        "energy books: worst residual {worst:e} relative"
    );
    assert!(
        worst_mass <= 1e-8,
        "mass books: worst residual {worst_mass:e} relative"
    );
    assert!(
        worst_without > 1e-2,
        "without the latent terms the books still close ({worst_without:e}): the plant is \
         not carrying latent heat across its boundary and the gate proves nothing"
    );
}

/// **Both fidelities land on one wet plant**, on its way to steady state and
/// settled.
#[test]
fn both_fidelities_agree_on_the_wet_rundown() {
    let run = |solver: &str| {
        let mut engine = build(&on_solver(solver));
        let mut readings = Vec::new();
        for t in 1..=2000 {
            tick(&mut engine, t);
            if t == 300 || t == 2000 {
                let s = engine.snapshot();
                let NodeKind::Tank(tank) = &node(&s, "rundown_tank").kind else {
                    panic!("the tank is a tank");
                };
                readings.push([
                    node(&s, "valve_outlet").vapour_fraction.unwrap_or(0.0),
                    node(&s, "valve_outlet").temperature_k,
                    tank.temperature.value(),
                    tank.composition.fractions()[2],
                    tank.mass.value(),
                ]);
            }
        }
        readings
    };
    let (newton, simple) = (run("newton"), run("simple"));
    for (n, g) in newton.iter().zip(&simple) {
        for (a, b) in n.iter().zip(g) {
            assert!(
                (a - b).abs() <= 1e-6 * a.abs().max(1e-3),
                "newton {n:?} against simple {g:?}"
            );
        }
    }
}

/// `free_water` is refused wherever nothing would read it: on a plant without
/// the line flash (which a gas member never passes either), and twice on one slate;
/// and on a member that is not water by its declared numbers (M63.2).
#[test]
fn free_water_is_refused_where_nothing_reads_it() {
    let no_flash = PLANT.replace("line_flash = \"equilibrium\"\n", "");
    let e = refusal(&no_flash);
    assert!(
        e.contains("'water' declares free_water") && e.contains("line_flash"),
        "{e}"
    );

    // A gas member never reaches a refusal of its own: the line flash refuses
    // any gas component, and without it the key is refused above.

    let twice = PLANT.replace(
        "[nodes.rundown]",
        "[[components]]\nname = \"brine\"\ntb_c = 100.0\nmolar_mass_kg_per_mol = 0.018\n\
         density_kg_per_m3 = 1030.0\ncp_j_per_kg_k = 4000.0\nfree_water = true\n\n\
         [nodes.rundown]",
    );
    let e = refusal(&twice);
    assert!(
        e.contains("'water' and 'brine' both declare free_water"),
        "{e}"
    );

    // Free water is water (M63.2, docs/DESIGN.md §68.2): its vapour pressure is
    // the steam tables' curve, so a member boiling or weighing as another
    // fluid is refused by name.
    for (key, other) in [
        (
            "tb_c = 100.0
molar_mass_kg_per_mol = 0.018
",
            "tb_c = 150.0
molar_mass_kg_per_mol = 0.018
",
        ),
        (
            "tb_c = 100.0
molar_mass_kg_per_mol = 0.018
",
            "tb_c = 100.0
molar_mass_kg_per_mol = 0.030
",
        ),
    ] {
        let not_water = PLANT.replace(key, other);
        assert_ne!(not_water, PLANT);
        let e = refusal(&not_water);
        assert!(
            e.contains("'water' declares free_water with") && e.contains("§68.2"),
            "{e}"
        );
    }
}
