//! A component's own latent heat and vapour heat capacity, through the loader
//! and a running plant (M62.1, docs/DESIGN.md §67.1).
//!
//! The thermodynamics is gated against steam tables in
//! `crates/solvers/tests/reference/water.rs`; this file gates the plant file:
//! the keys load and reach the line flash, and every place they would be a
//! number nothing reads refuses them by name.

use refinery_scenarios::{build_engine, load_str};

/// A steam supply at 10 bar and 400 °C through a valve to a 1.8 bar
/// destination: superheated, all vapour, on a flashing plant.
const STEAM_LINE: &str = r#"
[meta]
name = "steam_line"
description = "M62.1 gate: a superheated steam supply through a valve."

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
flow = "newton"
thermo = "trouton"
line_flash = "equilibrium"

[nodes.steam_supply]
type = "source"
pressure_bar = 10.0
temperature_c = 400.0
composition = { water = 1.0 }

[nodes.steam_valve]
type = "valve"
kv = 20.0
opening = 0.5

[nodes.column_bottom]
type = "sink"
pressure_bar = 1.8
composition = { water = 1.0 }

[[pipes]]
name = "steam_header"
from = "steam_supply"
to = "steam_valve"
length_m = 30.0
diameter_m = 0.1

[[pipes]]
name = "steam_line"
from = "steam_valve"
to = "column_bottom"
length_m = 20.0
diameter_m = 0.1
"#;

fn refusal(source: &str) -> String {
    match load_str(source) {
        Err(e) => e.to_string(),
        Ok(file) => match build_engine(&file) {
            Err(e) => e.to_string(),
            Ok(_) => panic!("the plant loaded, where it must be refused"),
        },
    }
}

#[test]
fn a_steam_line_carries_the_steam_tables_heat_through_its_valve() {
    // On the engine's 0 °C saturated-liquid datum a stream's enthalpy is
    // `cp·(T − T_REF) + latent`; a valve is isenthalpic, so the steam reaching
    // the destination carries the supply's 3 275 kJ/kg (IAPWS-IF97: 3 263.9 at
    // 10 bar and 400 °C).
    for solver in ["newton", "simple"] {
        let src = STEAM_LINE.replace("flow = \"newton\"", &format!("flow = \"{solver}\""));
        let mut engine = build_engine(&load_str(&src).unwrap()).unwrap();
        for _ in 0..200 {
            engine.tick().unwrap();
        }
        let snapshot = engine.snapshot();
        let line = snapshot
            .edges
            .iter()
            .find(|e| e.name == "steam_line")
            .unwrap();
        let stream = &line.stream;
        assert!(stream.mass_flow.value() > 0.1, "{solver}: no steam flows");
        assert_eq!(
            stream.vapour_fraction,
            Some(1.0),
            "{solver}: not all vapour"
        );
        let latent = stream
            .latent
            .expect("a vapour stream carries its latent heat");
        let h = 4184.0 * (stream.temperature.value() - 273.15) + latent.value();
        assert!(
            (h / 3263.9e3 - 1.0).abs() < 0.005,
            "{solver}: {h} J/kg arrives, against the table's 3 263.9 kJ/kg"
        );
    }
}

#[test]
fn the_keys_are_refused_wherever_nothing_would_read_them() {
    // Under a model that reads no latent heat of its own.
    let constant_alpha = STEAM_LINE
        .replace("thermo = \"trouton\"", "thermo = \"constant_alpha\"")
        .replace("line_flash = \"equilibrium\"", "");
    let e = refusal(&constant_alpha);
    assert!(
        e.contains("dh_vap_j_per_mol") && e.contains("trouton"),
        "{e}"
    );
    let only_vapour_cp = constant_alpha.replace("dh_vap_j_per_mol = 40650.0\n", "");
    let e = refusal(&only_vapour_cp);
    assert!(
        e.contains("cp_vapour_j_per_kg_k") && e.contains("trouton"),
        "{e}"
    );

    // On a gas, which never boils here — on a plant without the line flash,
    // which refuses any gas component first, and with cold water so its supply
    // is not boiling there.
    let cold = STEAM_LINE
        .replace("line_flash = \"equilibrium\"", "")
        .replace("temperature_c = 400.0", "temperature_c = 20.0");
    for key in [
        "dh_vap_j_per_mol = 40650.0",
        "cp_vapour_j_per_kg_k = 2000.0",
    ] {
        let gas = format!(
            "{cold}\n[[components]]\nname = \"hydrogen\"\nphase = \"gas\"\ntb_c = \
             -252.87\nmolar_mass_kg_per_mol = 0.002016\ncp_j_per_kg_k = 14300.0\n{key}\n"
        );
        let e = refusal(&gas);
        assert!(e.contains("gas component 'hydrogen'"), "{key}: {e}");
    }

    // Beside a cp shape.
    let shaped = STEAM_LINE
        .replace(
            "thermo = \"trouton\"",
            "thermo = \"trouton\"\nheat_capacity = \"linear\"",
        )
        .replace(
            "cp_j_per_kg_k = 4184.0\n",
            "cp_shape_anchor_c = 25.0\ncp_shape_at_anchor_j_per_kg_k = 4180.0\n\
         cp_shape_slope_j_per_kg_k2 = 0.1\n",
        );
    let e = refusal(&shaped);
    assert!(e.contains("cp_vapour_j_per_kg_k beside a cp shape"), "{e}");

    // A latent heat or a capacity must be a positive magnitude.
    for (from, to) in [
        ("dh_vap_j_per_mol = 40650.0", "dh_vap_j_per_mol = 0.0"),
        (
            "cp_vapour_j_per_kg_k = 2000.0",
            "cp_vapour_j_per_kg_k = -1.0",
        ),
    ] {
        let e = refusal(&STEAM_LINE.replace(from, to));
        assert!(e.contains("non-positive"), "{to}: {e}");
    }
}
