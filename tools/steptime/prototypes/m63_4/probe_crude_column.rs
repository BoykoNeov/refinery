//! M63.4 probe (docs/DESIGN.md §68.4; see FINDINGS.md): the crude unit's own column, its oil
//! held at the shipped wet plant's settled rate, water added on top; each share
//! solved cold and warm (seeded from the 0.3 % solve).

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::graph::{CascadeSpec, ColumnDraw, NodeId, Pumparound};
use refinery_core::traits::{ColumnPass, Separation, SeparationModel};
use refinery_core::units::{JPerKgK, JPerMol, Kelvin, KgPerM3, KgPerMol, KgPerSec, Pascal, Watt};
use refinery_solvers::{HeatBalanceCascade, TroutonThermo};

fn slate() -> Slate {
    let cut = |name: &str, tb_c: f64, molar_mass: f64, density: f64, cp: f64| PseudoComponent {
        name: name.into(),
        tb: Kelvin(tb_c + 273.15),
        molar_mass: KgPerMol(molar_mass),
        density: Some(KgPerM3(density)),
        cp: JPerKgK(cp),
        cp_shape: None,
        phase: Phase::Liquid,
        dh_vap: None,
        cp_vapour: None,
        free_water: false,
    };
    Slate::new(vec![
        cut("light_naphtha", 70.0, 0.095, 680.0, 2200.0),
        cut("heavy_naphtha", 150.0, 0.13, 750.0, 2100.0),
        cut("kerosene", 220.0, 0.17, 800.0, 2050.0),
        cut("diesel", 300.0, 0.22, 850.0, 2000.0),
        cut("gas_oil", 380.0, 0.3, 900.0, 1950.0),
        cut("residue", 560.0, 0.55, 980.0, 1850.0),
        PseudoComponent {
            name: "water".into(),
            tb: Kelvin(373.15),
            molar_mass: KgPerMol(0.018015),
            density: Some(KgPerM3(998.0)),
            cp: JPerKgK(4184.0),
            cp_shape: None,
            phase: Phase::Liquid,
            dh_vap: Some(JPerMol(40_650.0)),
            cp_vapour: Some(JPerKgK(2000.0)),
            free_water: true,
        },
    ])
    .unwrap()
}

fn show(r: &Result<Separation, refinery_core::error::SimError>) -> String {
    match r {
        Ok(s) => format!(
            "ok R={:.2} top={:.1} bot={:.1} reb={:.1}MW",
            s.reflux_ratio.unwrap_or(f64::NAN),
            s.profile
                .as_ref()
                .map_or(f64::NAN, |p| p.temperatures[0].value() - 273.15),
            s.profile
                .as_ref()
                .map_or(f64::NAN, |p| p.temperatures.last().unwrap().value() - 273.15),
            s.reboiler_duty.map_or(f64::NAN, |d| d.value() / 1e6),
        ),
        Err(e) => format!("ERR {}", e.to_string().chars().take(120).collect::<String>()),
    }
}

#[test]
fn c13_probe_crude() {
    let slate = slate();
    let draws = vec![
        ColumnDraw::by_stage(NodeId(1), 0, Some(0.22)),
        ColumnDraw::water_draw(NodeId(2)),
        ColumnDraw::by_stage(NodeId(3), 3, Some(0.16)),
        ColumnDraw::by_stage(NodeId(4), 5, Some(0.2)),
        ColumnDraw::by_stage(NodeId(5), 7, Some(0.14)),
        ColumnDraw::by_stage(NodeId(6), 10, None),
    ];
    let pumparound = |stage: u32, mw: f64| Pumparound {
        stage,
        duty: Watt(mw * 1e6),
    };
    let spec = CascadeSpec {
        stages: 10,
        feed_stage: 8,
        reflux_ratio: 4.0,
        pumparounds: vec![pumparound(4, 15.0), pumparound(6, 15.0), pumparound(7, 10.0)],
        steam: None,
    };
    let oil = [0.07976, 0.13958, 0.15952, 0.1994, 0.13958, 0.27916];
    let oil_total: f64 = oil.iter().sum();
    let oil_flow = 112.83266 * 0.997;
    let base_t = 624.0538;
    let thermo = TroutonThermo::new();
    // The 0.3 % solve, the warm seed.
    let feed_at = |water: f64| {
        let mut weights: Vec<f64> = oil.iter().map(|x| x / oil_total * (1.0 - water)).collect();
        weights.push(water);
        Composition::from_weights(&weights).unwrap()
    };
    for (label, keep_flux) in [("fixed T", false), ("fixed enthalpy", true)] {
        eprintln!("== {label}");
        let base_feed = feed_at(0.003);
        let base_flow = oil_flow / 0.997;
        let base_flux = base_flow * base_feed.mixture_cp(&slate).value() * (base_t - 273.15);
        let base = HeatBalanceCascade::new()
            .separate(
                &ColumnPass {
                    slate: &slate,
                    draws: &draws,
                    smearing: Kelvin(0.0),
                    pressure: Pascal(1.8e5),
                    feed: &base_feed,
                    feed_flow: KgPerSec(base_flow),
                    temperature: Kelvin(base_t),
                    cascade: Some(&spec),
                    seed: None,
                    steam: None,
                },
                &thermo,
            )
            .expect("the 0.3 % column solves cold");
        let seed = base.profile.clone();
        for water in [0.003, 0.01, 0.02, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.6, 0.8, 0.99] {
            let feed = feed_at(water);
            let flow = oil_flow / (1.0 - water);
            let t = if keep_flux {
                273.15 + base_flux / (flow * feed.mixture_cp(&slate).value())
            } else {
                base_t
            };
            let pass = |seed| ColumnPass {
                slate: &slate,
                draws: &draws,
                smearing: Kelvin(0.0),
                pressure: Pascal(1.8e5),
                feed: &feed,
                feed_flow: KgPerSec(flow),
                temperature: Kelvin(t),
                cascade: Some(&spec),
                seed,
                steam: None,
            };
            let cold = HeatBalanceCascade::new().separate(&pass(None), &thermo);
            let warm = HeatBalanceCascade::new().separate(&pass(seed.as_ref()), &thermo);
            eprintln!(
                "  water {water:>5} T={:.1}C: cold {} | warm {}",
                t - 273.15,
                show(&cold),
                show(&warm)
            );
        }
    }
}
