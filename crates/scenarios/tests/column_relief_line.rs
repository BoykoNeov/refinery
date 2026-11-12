//! M75.3.2 — a column's relief valve on the map, its line to a flare
//! (docs/DESIGN.md §80.6): the holdup plant (`crude_unit_holdup.toml`,
//! untouched) with its steel, the M75.1 `condenser_limit`, and its `relief`
//! a `relief_valve` node — one pipe into it from the column, one out of it
//! to a flare — declared inline (`column_relief::relief_plant`).

use refinery_core::graph::NodeKind;
use refinery_core::snapshot::Snapshot;
use refinery_core::units::Kelvin;

use crate::column_empty::{engine_of, holdup, refused, run, state, Books, FLARE_PIPE};
use crate::column_relief::{
    book_valve, relief_plant, vent, with_relief, DESIGN, FLARE, FLARE_SINK, INLET_PIPE, P_ATM,
    VALVE,
};

/// The relief declared as the fixtures do: by the valve's name.
fn by_name() -> String {
    format!("valve = \"{VALVE}\"")
}

/// **A relief line is refused where it is no line**, each by name: a valve
/// that names no node, or a node that is no relief valve; a pop valve; a
/// valve with no `x_t` or one outside (0, 1]; a set above the column's
/// design pressure or at the pressure it runs at; full lift above 110 % of
/// the design pressure (gauge, API RP-520); a line through a junction (a
/// leak path on its pipe), or that ends anywhere but a sink or the
/// atmosphere; a flare at or above the set; a plant with the line flash (which holds no relief valve at all); the
/// M75.3.1 form, its numbers on the column.
#[test]
fn a_relief_line_is_refused_where_it_is_no_line() {
    let book = book_valve(4000.0);
    let says = |src: String, words: &[&str]| {
        let e = refused(&src);
        for word in words {
            assert!(e.contains(word), "{word}: {e}");
        }
    };
    let set_line = format!("set_pressure_bar = {:?}", DESIGN / 1e5);
    let acc_line = format!("accumulation_bar = {:?}", 0.10 * (DESIGN - P_ATM) / 1e5);
    says(
        relief_plant("valve = \"nowhere\"", &book, FLARE_SINK, ""),
        &["'nowhere'", "no node"],
    );
    // A plain valve where the relief should be.
    says(
        relief_plant(
            "valve = \"column_psv\"",
            "kv = 4000.0\nopening = 1.0\nx_t = 0.7",
            FLARE_SINK,
            "",
        )
        .replace("type = \"relief_valve\"", "type = \"valve\""),
        &["not a `relief_valve`"],
    );
    says(
        relief_plant(
            &by_name(),
            &format!("{book}\nblowdown_bar = 0.2"),
            FLARE_SINK,
            "",
        ),
        &["blowdown_bar", "C32"],
    );
    says(
        relief_plant(&by_name(), &book.replace("\nx_t = 0.7", ""), FLARE_SINK, ""),
        &["no x_t"],
    );
    says(
        relief_plant(
            &by_name(),
            &book.replace("x_t = 0.7", "x_t = 1.5"),
            FLARE_SINK,
            "",
        ),
        &["x_t = 1.5"],
    );
    says(
        relief_plant(
            &by_name(),
            &book.replace(&set_line, "set_pressure_bar = 3.6"),
            FLARE_SINK,
            "",
        ),
        &["above the column's design pressure"],
    );
    says(
        relief_plant(
            &by_name(),
            &book.replace(&set_line, "set_pressure_bar = 1.8"),
            FLARE_SINK,
            "",
        ),
        &["would vent at rest"],
    );
    says(
        relief_plant(
            &by_name(),
            &book.replace(&acc_line, "accumulation_bar = 0.3"),
            FLARE_SINK,
            "",
        ),
        &["110 %"],
    );
    // A leak path on the line's pipe in: a junction between.
    says(
        relief_plant(
            &by_name(),
            &book,
            FLARE_SINK,
            "\n[nodes.outside]\ntype = \"atmosphere\"\n",
        )
        .replace(
            "length_m = 3.0\ndiameter_m = 0.9\n",
            "length_m = 3.0\ndiameter_m = 0.9\nleak_to = \"outside\"\n",
        ),
        &["nothing between"],
    );
    // The line into a tank.
    says(
        relief_plant(
            &by_name(),
            &book,
            "type = \"tank\"\narea_m2 = 30.0\nheight_m = 30.0\ninitial_level_m = 0.5\n\
             temperature_c = 40.0\ncomposition = { light_naphtha = 1.0 }",
            "",
        ),
        &["a flare is a `sink` or the atmosphere"],
    );
    says(
        relief_plant(&by_name(), &book, &FLARE_SINK.replace("1.01325", "3.6"), ""),
        &["not below its set"],
    );
    says(
        // With the coil in cells the line flash asks for (§68.1).
        relief_plant(&by_name(), &book, FLARE_SINK, "")
            .replace(
                "boiloff = \"none\"",
                "boiloff = \"flash\"\nline_flash = \"equilibrium\"",
            )
            .replace(
                "fluid_heating_value_mj_per_kg = 42.686\n",
                "fluid_heating_value_mj_per_kg = 42.686\ncoil_cells = 8\ncoil_length_m = 800.0\n\
                 coil_diameter_m = 0.24\n",
            ),
        &["line_flash", "'column_psv', a relief valve"],
    );
    // The M75.3.1 form: the valve's numbers on the column.
    says(
        relief_plant("kv = 4000.0, x_t = 0.7", &book, FLARE_SINK, ""),
        &["kv"],
    );
}

/// **The flare may be the atmosphere**: its pressure is atmospheric, as a
/// sink's is the sink's.
#[test]
fn the_flare_may_be_the_atmosphere() {
    let engine = engine_of(&relief_plant(
        &by_name(),
        &book_valve(4000.0),
        "type = \"atmosphere\"",
        "",
    ))
    .unwrap();
    let relief = holdup(&engine).condenser.unwrap().relief.unwrap();
    assert_eq!(relief.flare_pressure.value(), P_ATM);
}

/// One pipe's stream by name.
fn pipe(snapshot: &Snapshot, name: &str) -> refinery_core::stream::Stream {
    snapshot
        .edges
        .iter()
        .find(|e| e.name == name)
        .unwrap()
        .stream
        .clone()
}

/// **The line carries the vent**, the coolant lost (the governing case,
/// Kister §9.3; its temperature the test's own hand, as M75.3.1's story):
/// on every tick both pipes carry the column's vent to the bit — its flow,
/// composition and temperature — and the latent heat that makes each pipe's
/// whole enthalpy the heat the column booked leaving by it; `None` on a tick
/// that vents nothing. The valve stands at the column's pressure, reports no
/// cavitation, and the books close every tick counting the vent off the
/// flare's pipe; the relief lifts.
#[test]
fn slow_the_line_carries_the_vent() {
    let mut engine = engine_of(&with_relief(4000.0)).unwrap();
    run(&mut engine, 100);
    let books = Books::of(&engine);
    let coolant = holdup(&engine).condenser.unwrap().coolant;
    let set_coolant = |engine: &mut refinery_core::engine::Engine, t: Kelvin| {
        let node = engine.graph.find_node("column").unwrap();
        if let NodeKind::Column {
            cascade: Some(spec),
            ..
        } = &mut engine.graph.node_mut(node).kind
        {
            spec.holdup
                .as_mut()
                .unwrap()
                .condenser
                .as_mut()
                .unwrap()
                .coolant = t;
        }
    };
    set_coolant(&mut engine, Kelvin(2000.0));
    let (mut vented_ticks, mut most, mut worst_mass, mut worst_heat) =
        (0, 0.0_f64, 0.0_f64, 0.0_f64);
    for tick in 0..1200 {
        if tick == 900 {
            set_coolant(&mut engine, coolant);
        }
        let before = state(&engine);
        engine.tick().unwrap_or_else(|e| panic!("tick {tick}: {e}"));
        let (m, h) = books.close(&engine, &before);
        worst_mass = worst_mass.max(m.abs());
        worst_heat = worst_heat.max(h.abs());
        let s = engine.snapshot();
        let read = vent(&s);
        for name in [INLET_PIPE, FLARE_PIPE] {
            let stream = pipe(&s, name);
            assert_eq!(
                stream.mass_flow.value(),
                read.flow_kg_s,
                "tick {tick}: {name}"
            );
            assert_eq!(stream.vapour_fraction, None, "tick {tick}: {name}");
            // Idle, the line past the valve stands as an idle line does —
            // its valve has nothing in to take a state from.
            if read.flow_kg_s > 0.0 {
                let off = (stream.temperature.value() - read.temperature_k).abs();
                if name == INLET_PIPE {
                    assert_eq!(off, 0.0, "tick {tick}: {name}");
                } else {
                    assert!(off < 1e-9, "tick {tick}: {name} {off} K");
                }
                // The pipe in to the bit; past the valve, its one inflow's
                // mix renormalised, to the last bit or so.
                let apart = stream
                    .composition
                    .fractions()
                    .iter()
                    .zip(read.composition.fractions())
                    .map(|(a, b)| (a - b).abs())
                    .fold(0.0, f64::max);
                if name == INLET_PIPE {
                    assert_eq!(stream.composition, read.composition, "tick {tick}");
                } else {
                    assert!(apart < 1e-12, "tick {tick}: {name} {apart}");
                }
                let latent = stream
                    .latent
                    .expect("a vapour carries its latent heat")
                    .value();
                let cp: f64 = stream
                    .composition
                    .fractions()
                    .iter()
                    .enumerate()
                    .map(|(c, w)| w * engine.slate.get(c).cp.value())
                    .sum();
                let flux = read.flow_kg_s * (cp * (read.temperature_k - 273.15) + latent);
                assert!(
                    (flux - read.heat_w).abs() <= 1e-9 * read.heat_w.abs(),
                    "tick {tick}: {name} carries {flux} W, the column booked {}",
                    read.heat_w
                );
                assert!(latent > 1.0e5, "tick {tick}: {name} λ {latent}");
            } else {
                assert_eq!(stream.latent, None, "tick {tick}: {name}");
            }
        }
        if read.flow_kg_s > 0.0 {
            vented_ticks += 1;
        }
        most = most.max(read.flow_kg_s);
        let node = |name: &str| s.nodes.iter().find(|n| n.name == name).unwrap();
        assert_eq!(
            node(VALVE).pressure_pa,
            node("column").pressure_pa,
            "tick {tick}"
        );
        assert!(node(VALVE).cavitation.is_none(), "tick {tick}");
        assert_eq!(node(FLARE).pressure_pa, P_ATM, "tick {tick}");
    }
    println!(
        "M75_3_2 the line: {vented_ticks} ticks vented, at most {most:.2} kg/s; books {worst_mass:.2e} \
         kg/s {worst_heat:.2} W"
    );
    assert!(vented_ticks > 100, "{vented_ticks}");
    assert!(most > 1.0, "{most}");
    assert!(worst_mass < 1.0e-8, "{worst_mass}");
    assert!(worst_heat < 20.0, "{worst_heat}");
}
