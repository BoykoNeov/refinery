//! M53.0 — a supply that is partly vapour, and a line that boils as its pressure
//! falls (docs/DESIGN.md §58; ledger row B46 and B3's flashing-line clause).
//!
//! An inline plant — a hot naphtha supply at 2.4 bar, a 30 m pipe, a control
//! valve, a 20 m pipe, a destination at 1.2 bar — with `[fidelity] line_flash =
//! "equilibrium"`. The flash itself is held to a hand calculation in
//! `crates/solvers/tests/reference/line_flash.rs`; these gates hold the PLANT:
//! the supply's vapour share where it stands, the stream re-boiling and cooling
//! across the valve, every move across the bubble pressure landing on the cold
//! answer on both fidelities, the refusal lifted only where modelled, and
//! everything a flashing plant may not hold refused at load by name.

use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot};
use refinery_core::units::Kelvin;
use refinery_core::Engine;

/// The probe plant of ROADMAP M53, at a supply temperature and an opening.
fn plant(solver: &str, line_flash: &str, celsius: f64, opening: f64) -> String {
    format!(
        r#"
[meta]
name = "flashing_rundown_probe"
description = "A hot naphtha supply let down through a control valve."

[simulation]
dt = 0.1

[fidelity]
flow = "{solver}"
thermo = "trouton"
line_flash = "{line_flash}"

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
pressure_bar = 2.4
temperature_c = {celsius}
composition = {{ light_naphtha = 0.7, heavy_naphtha = 0.3 }}

[nodes.rundown_valve]
type = "valve"
kv = 40.0
opening = {opening}

[nodes.receiver]
type = "sink"
pressure_bar = 1.2
temperature_c = 40.0
composition = {{ light_naphtha = 0.7, heavy_naphtha = 0.3 }}

[[pipes]]
name = "upstream"
from = "rundown"
to = "rundown_valve"
length_m = 30.0
diameter_m = 0.08

[[pipes]]
name = "downstream"
from = "rundown_valve"
to = "receiver"
length_m = 20.0
diameter_m = 0.10
"#
    )
}

fn build(src: &str) -> Result<Engine, String> {
    let file = refinery_scenarios::load_str(src).map_err(|e| e.to_string())?;
    refinery_scenarios::build_engine(&file).map_err(|e| e.to_string())
}

/// Run `ticks`, returning the worst Newton/sweep count a tick took.
fn run(engine: &mut Engine, ticks: u64) -> u32 {
    let mut worst = 0;
    for t in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("the plant must run: tick {t}: {e}"));
        let solver = &engine.snapshot().solver;
        assert!(solver.converged, "tick {t} did not converge");
        worst = worst.max(solver.iterations);
    }
    worst
}

fn node<'a>(snapshot: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    snapshot.nodes.iter().find(|n| n.name == name).unwrap()
}

fn edge<'a>(snapshot: &'a Snapshot, name: &str) -> &'a refinery_core::stream::Stream {
    &snapshot
        .edges
        .iter()
        .find(|e| e.name == name)
        .unwrap()
        .stream
}

/// **A supply past its bubble point flows as a mixture, and re-boils at the
/// valve.** 122 °C at 2.4 bar is 10.78% vapour by mass where it stands — the
/// reference's hand calculation — and the edge it feeds carries that share and
/// its latent heat, `q·λ`. Across the valve the pressure falls, more of it boils
/// and it cools; and the mixture is so light that the wide-open valve passes a
/// quarter of what it passes cold.
#[test]
fn a_supply_past_boiling_flows_as_a_mixture_and_reboils_at_the_valve() {
    let mut hot = build(&plant("newton", "equilibrium", 122.0, 1.0)).unwrap();
    run(&mut hot, 20);
    let s = hot.snapshot();
    let supply = node(&s, "rundown");
    let q_supply = supply
        .vapour_fraction
        .expect("the supply boils where it stands");
    assert!(
        (q_supply - 0.107_835_961_205_522).abs() < 1e-9,
        "the reference's hand calculation: {q_supply}"
    );
    let upstream = edge(&s, "upstream");
    assert_eq!(upstream.vapour_fraction, Some(q_supply));
    // λ_vapour = 309 078.508 J/kg (the reference).
    let latent = upstream
        .latent
        .expect("a two-phase stream carries its latent heat");
    assert!((latent.value() - q_supply * 309_078.508_029_7).abs() < 1e-3);

    let valve = node(&s, "rundown_valve");
    let q_valve = valve.vapour_fraction.expect("the valve re-boils it");
    assert!(
        q_valve > q_supply,
        "{q_valve} after the valve, {q_supply} before"
    );
    assert!(
        valve.temperature_k < 122.0 + 273.15,
        "it cools as it boils: {} K",
        valve.temperature_k
    );
    assert_eq!(edge(&s, "downstream").vapour_fraction, Some(q_valve));

    let mut cold = build(&plant("newton", "equilibrium", 110.0, 1.0)).unwrap();
    run(&mut cold, 20);
    let (hot_flow, cold_flow) = (
        upstream.mass_flow.value(),
        edge(&cold.snapshot(), "upstream").mass_flow.value(),
    );
    assert!(
        hot_flow < 0.3 * cold_flow,
        "{hot_flow} kg/s boiling against {cold_flow} kg/s liquid"
    );
    assert_eq!(node(&cold.snapshot(), "rundown").vapour_fraction, None);
}

/// **Every move across the bubble pressure lands on the cold answer, on both
/// fidelities** (§58 fork 3). Ordered moves between four supply temperatures —
/// liquid, just liquid, boiling a little, boiling hard — and two openings,
/// each sent mid-run by command, each landing on a cold start declared at the
/// new state. Without the density slope Newton took its 50-iteration cap and the
/// game solver cycled across the bubble pressure; without fresh edges the game
/// solver cycled on a step from 110 °C. Measured worst here: 13 iterations a
/// tick on either fidelity.
#[test]
fn every_move_across_the_bubble_pressure_lands_on_the_cold_answer() {
    let temperatures = [110.0, 120.0, 121.2, 125.0];
    let openings = [0.05, 1.0];
    for solver in ["newton", "simple"] {
        let mut cold = Vec::new();
        for &t in &temperatures {
            for &o in &openings {
                let mut engine = build(&plant(solver, "equilibrium", t, o)).unwrap();
                run(&mut engine, 20);
                cold.push((
                    (t, o),
                    edge(&engine.snapshot(), "upstream").mass_flow.value(),
                ));
            }
        }
        let mut worst_iterations = 0;
        let mut worst_miss = 0.0f64;
        for &((t, o), _) in &cold {
            for &((t2, o2), target) in &cold {
                if (t, o) == (t2, o2) {
                    continue;
                }
                let mut engine = build(&plant(solver, "equilibrium", t, o)).unwrap();
                run(&mut engine, 10);
                let supply = engine.graph.find_node("rundown").unwrap();
                let valve = engine.graph.find_node("rundown_valve").unwrap();
                engine
                    .apply(Command::SetSourceTemperature {
                        node: supply,
                        temperature: Kelvin(t2 + 273.15),
                    })
                    .unwrap();
                engine
                    .apply(Command::SetValveOpening {
                        node: valve,
                        opening: o2,
                    })
                    .unwrap();
                worst_iterations = worst_iterations.max(run(&mut engine, 10));
                let landed = edge(&engine.snapshot(), "upstream").mass_flow.value();
                worst_miss = worst_miss.max(((landed - target) / target).abs());
            }
        }
        assert!(
            worst_miss < 1e-6,
            "{solver}: a move missed its cold answer by {worst_miss:e}"
        );
        assert!(
            worst_iterations <= 13,
            "{solver}: a tick took {worst_iterations} iterations"
        );
    }
}

/// **A step from liquid to boiling settles on the game solver** — the case the
/// built engine's probe traced (§58 fork 3): a supply stepped from 110 °C to
/// 125 °C at half open cycled across the valve's bubble pressure (1.857 bar
/// liquid, 1.701 bar at 208 kg/m³, 2 497 times each) while the node step judged
/// densities frozen at the top of its sweep. Read fresh, it lands.
#[test]
fn a_step_from_liquid_to_boiling_settles_on_the_game_solver() {
    let target = {
        let mut cold = build(&plant("simple", "equilibrium", 125.0, 0.5)).unwrap();
        run(&mut cold, 20);
        edge(&cold.snapshot(), "upstream").mass_flow.value()
    };
    let mut engine = build(&plant("simple", "equilibrium", 110.0, 0.5)).unwrap();
    run(&mut engine, 10);
    let supply = engine.graph.find_node("rundown").unwrap();
    engine
        .apply(Command::SetSourceTemperature {
            node: supply,
            temperature: Kelvin(125.0 + 273.15),
        })
        .unwrap();
    run(&mut engine, 10);
    let landed = edge(&engine.snapshot(), "upstream").mass_flow.value();
    assert!(
        ((landed - target) / target).abs() < 1e-6,
        "{landed} against {target}"
    );
}

/// **The refusal is lifted only where it is modelled** (§58 fork 1, the user's
/// DECISION). The same supply at 125 °C is refused by the loader on a plant that
/// carries liquid only and accepted on a flashing one; a command warming a
/// running supply past boiling likewise.
#[test]
fn a_boiling_supply_is_refused_only_where_the_line_does_not_flash() {
    let Err(refused) = build(&plant("newton", "none", 125.0, 0.5)) else {
        panic!("a plant carrying liquid only must refuse a boiling supply");
    };
    assert!(refused.contains("is boiling"), "{refused}");
    assert!(
        refused.contains("line_flash"),
        "the refusal names the cure: {refused}"
    );
    build(&plant("newton", "equilibrium", 125.0, 0.5)).expect("a flashing plant feeds it");

    for (line_flash, accepted) in [("none", false), ("equilibrium", true)] {
        let mut engine = build(&plant("newton", line_flash, 110.0, 0.5)).unwrap();
        run(&mut engine, 5);
        let supply = engine.graph.find_node("rundown").unwrap();
        let warmed = engine.apply(Command::SetSourceTemperature {
            node: supply,
            temperature: Kelvin(125.0 + 273.15),
        });
        assert_eq!(warmed.is_ok(), accepted, "{line_flash}: {warmed:?}");
    }
}

/// **A flashing plant holds only what the flash models** (§58 fork 5), each
/// refused at load by name.
#[test]
fn a_flashing_plant_holds_only_what_the_flash_models() {
    let base = plant("newton", "equilibrium", 110.0, 0.5);
    let cases = [
        (
            base.replace("thermo = \"trouton\"", "thermo = \"constant\""),
            "thermo = \"constant\"",
        ),
        // A pump is admitted since M54 (§59); one declaring M50's suction key
        // is not, because on a flashing plant the key would change nothing.
        (
            base.replace(
                "[nodes.rundown_valve]\ntype = \"valve\"\nkv = 40.0",
                "[nodes.rundown_valve]\ntype = \"pump\"\nh0_m = 40.0\na = 800.0\non = true\nnpsh_required_m = 3.0\n#",
            ),
            "npsh_required_m",
        ),
        // A check valve is admitted since M54.1 (§59.1); a relief valve is not.
        (
            base.replace(
                "[nodes.rundown_valve]\ntype = \"valve\"\nkv = 40.0",
                "[nodes.rundown_valve]\ntype = \"relief_valve\"\nkv = 40.0\nset_pressure_bar = 2.0\naccumulation_bar = 0.2\n#",
            ),
            "a relief valve",
        ),
        (
            base.replace(
                "length_m = 30.0\ndiameter_m = 0.08",
                "length_m = 30.0\ndiameter_m = 0.08\nleak_to = \"outside\"\n\n[nodes.outside]\ntype = \"atmosphere\"",
            ),
            "leak path",
        ),
    ];
    for (src, expected) in cases {
        let err = match build(&src) {
            Ok(_) => panic!("must refuse ({expected})"),
            Err(e) => e,
        };
        assert!(
            err.contains("line_flash") && err.contains(expected),
            "expected a line-flash refusal naming {expected}: {err}"
        );
    }
}

/// **A plant without the model publishes no vapour**: the same plant on
/// `"none"`, its valve's outlet below the liquid's bubble pressure, carries
/// liquid and writes no key it did not write before.
#[test]
fn a_plant_without_the_model_publishes_no_vapour() {
    let mut engine = build(&plant("newton", "none", 120.0, 1.0)).unwrap();
    run(&mut engine, 20);
    let s = engine.snapshot();
    assert!(s.nodes.iter().all(|n| n.vapour_fraction.is_none()));
    assert!(s.edges.iter().all(|e| e.stream.vapour_fraction.is_none()));
    let wire = serde_json::to_string(&s).unwrap();
    assert!(!wire.contains("vapour_fraction"), "{wire}");
}

/// **Deterministic**: two runs of the boiling plant, snapshot for snapshot.
#[test]
fn the_flashing_line_is_deterministic() {
    let trace = |solver: &str| {
        let mut engine = build(&plant(solver, "equilibrium", 121.2, 0.5)).unwrap();
        (0..30)
            .map(|_| {
                engine.tick().unwrap();
                serde_json::to_string(&engine.snapshot()).unwrap()
            })
            .collect::<Vec<_>>()
    };
    for solver in ["newton", "simple"] {
        assert_eq!(trace(solver), trace(solver), "{solver}");
    }
}
