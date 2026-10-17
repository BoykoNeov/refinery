//! M63.4 probe (docs/DESIGN.md §68.4; see FINDINGS.md): M62.5's test column fed a feed
//! that is more and more water, cold-started and warm-started.

use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
use refinery_core::graph::{CascadeSpec, ColumnDraw, NodeId};
use refinery_core::traits::{ColumnPass, SeparationModel};
use refinery_core::units::{JPerKgK, JPerMol, Kelvin, KgPerM3, KgPerMol, KgPerSec, Pascal};
use refinery_solvers::{HeatBalanceCascade, TroutonThermo};

fn slate() -> Slate {
    let cut = |name: &str, tb_c: f64, molar_mass: f64, cp: f64| PseudoComponent {
        name: name.into(),
        tb: Kelvin(tb_c + 273.15),
        molar_mass: KgPerMol(molar_mass),
        density: Some(KgPerM3(800.0)),
        cp: JPerKgK(cp),
        cp_shape: None,
        phase: Phase::Liquid,
        dh_vap: None,
        cp_vapour: None,
        free_water: false,
    };
    Slate::new(vec![
        cut("light", 70.0, 0.095, 2200.0),
        cut("naphtha", 150.0, 0.13, 2100.0),
        cut("kerosene", 220.0, 0.17, 2050.0),
        cut("diesel", 300.0, 0.22, 2000.0),
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

#[test]
fn c13_probe() {
    let slate = slate();
    let draws = vec![
        ColumnDraw::by_stage(NodeId(1), 0, Some(0.3)),
        ColumnDraw::water_draw(NodeId(2)),
        ColumnDraw::by_stage(NodeId(3), 3, Some(0.25)),
        ColumnDraw::by_stage(NodeId(4), 8, None),
    ];
    let spec = CascadeSpec {
        stages: 8,
        feed_stage: 5,
        reflux_ratio: 2.0,
        pumparounds: Vec::new(),
        steam: None,
    };
    let oil = [0.2, 0.3, 0.3, 0.2];
    for (pressure, feed_c) in [(1.8e5, 230.0), (1.8e5, 300.0), (1.01325e5, 200.0)] {
        eprintln!("== {pressure} Pa, feed {feed_c} °C");
        let mut seed = None;
        for water in [
            0.01, 0.1, 0.14, 0.16, 0.18, 0.2, 0.3, 0.5, 0.9, 0.99, 0.9999, 1.0,
        ] {
            let mut weights: Vec<f64> = oil.iter().map(|x| x * (1.0 - water)).collect();
            weights.push(water);
            let feed = Composition::from_weights(&weights).unwrap();
            let pass = |seed| ColumnPass {
                slate: &slate,
                draws: &draws,
                smearing: Kelvin(0.0),
                pressure: Pascal(pressure),
                feed: &feed,
                feed_flow: KgPerSec(10.0),
                temperature: Kelvin(feed_c + 273.15),
                cascade: Some(&spec),
                seed,
                steam: None,
            };
            let cold = HeatBalanceCascade::new().separate(&pass(None), &TroutonThermo::new());
            let warm = HeatBalanceCascade::new().separate(&pass(seed.as_ref()), &TroutonThermo::new());
            let show = |r: &Result<refinery_core::traits::Separation, refinery_core::error::SimError>| match r {
                Ok(s) => format!(
                    "ok R={:.3} top={:.1} bot={:.1} reb={:.2}MW",
                    s.reflux_ratio.unwrap_or(f64::NAN),
                    s.profile.as_ref().map_or(f64::NAN, |p| p.temperatures[0].value() - 273.15),
                    s.profile.as_ref().map_or(f64::NAN, |p| p.temperatures.last().unwrap().value() - 273.15),
                    s.reboiler_duty.map_or(f64::NAN, |d| d.value() / 1e6),
                ),
                Err(e) => format!("ERR {}", e.to_string().chars().take(160).collect::<String>()),
            };
            eprintln!("  water {water:>7}: cold {} | warm {}", show(&cold), show(&warm));
            if let Ok(s) = &warm {
                seed = s.profile.clone();
            } else if let Ok(s) = &cold {
                seed = s.profile.clone();
            }
        }
    }
}
