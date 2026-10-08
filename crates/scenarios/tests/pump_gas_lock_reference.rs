//! M54.3 — the gas-lock demo (`scenarios/pump_gas_lock.toml`, docs/DESIGN.md §59):
//! M50's pump plant with the line flash and a check valve on the discharge.
//!
//! The law, the solvers, the disc and the lock are gated on inline plants
//! (`pump_two_phase_reference.rs`, `flashing_check_valve_reference.rs`,
//! `gas_lock_reference.rs`). These gates hold the SHIPPED plant: the story its
//! file tells, beat by beat on both fidelities, and its energy and mass books
//! closing on every tick of it.

use refinery_core::components::Slate;
use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, NodeSnapshot, Snapshot};
use refinery_core::traits::EnthalpyModel;
use refinery_core::units::{Kelvin, Pascal};
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/pump_gas_lock.toml");

fn build(solver: &str) -> Engine {
    let src = DEMO.replace("flow = \"newton\"", &format!("flow = \"{solver}\""));
    let file = refinery_scenarios::load_str(&src).expect("the scenario must parse");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant must build: {e}"))
}

fn node<'a>(s: &'a Snapshot, name: &str) -> &'a NodeSnapshot {
    s.nodes.iter().find(|n| n.name == name).unwrap()
}

fn flow(s: &Snapshot) -> f64 {
    s.edges
        .iter()
        .find(|e| e.name == "suction_line")
        .unwrap()
        .stream
        .mass_flow
        .value()
}

fn locked(engine: &Engine) -> bool {
    let pump = engine.graph.find_node("feed_pump").unwrap();
    match engine.graph.node(pump).kind {
        NodeKind::Pump { gas_locked, .. } => gas_locked,
        _ => unreachable!(),
    }
}

/// The gas collected in the pump's eye, as a share of what locks it (M55.1).
fn pocket(engine: &Engine) -> f64 {
    let pump = engine.graph.find_node("feed_pump").unwrap();
    match engine.graph.node(pump).kind {
        NodeKind::Pump { gas_pocket, .. } => gas_pocket,
        _ => unreachable!(),
    }
}

/// The story's commands, in the order the file's header tells it.
enum Beat {
    Supply(f64),
    Destination(f64),
    Pump(bool),
    Vent,
}

fn apply(engine: &mut Engine, beat: &Beat) -> Result<(), String> {
    let id = |name: &str| engine.graph.find_node(name).unwrap();
    let command = match *beat {
        Beat::Supply(celsius) => Command::SetSourceTemperature {
            node: id("rundown_source"),
            temperature: Kelvin(celsius + 273.15),
        },
        Beat::Destination(bar) => Command::SetReservoirPressure {
            node: id("unit_feed"),
            pressure: Pascal(bar * 1e5),
        },
        Beat::Pump(on) => Command::SetPumpOn {
            node: id("feed_pump"),
            on,
        },
        Beat::Vent => Command::VentPump {
            node: id("feed_pump"),
        },
    };
    engine.apply(command).map_err(|e| e.to_string())
}

/// Run `ticks` ticks; the worst iterations a tick.
fn run(engine: &mut Engine, ticks: u32, label: &str) -> u32 {
    let mut worst = 0;
    for t in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("{label}: tick {t} failed: {e}"));
        let s = engine.snapshot();
        assert!(s.solver.converged, "{label}: tick {t}");
        worst = worst.max(s.solver.iterations);
    }
    worst
}

/// **The file's story, beat by beat, on both fidelities.** At 100 °C the pump
/// runs on liquid; at 110 °C its suction offers 7–16.5% vapour by volume and it
/// has lost part of its push; a one-second surge to 120 °C fills its gas pocket a
/// third of the way and it recovers as the pocket drains; held at 120 °C it locks
/// once the pocket is full, 3 s on; cooled back to 100 °C it stays
/// dead; a vent is refused while it runs; stopped, vented and started it runs as
/// it did at first; into 3 bar it still pushes; at 125 °C it locks again and the
/// check valve holds the line shut instead of letting the destination run back.
#[test]
fn the_files_story_on_both_fidelities() {
    for solver in ["newton", "simple"] {
        let mut engine = build(solver);
        let mut worst = run(&mut engine, 20, solver);
        let s = engine.snapshot();
        let liquid = flow(&s);
        assert_eq!(
            node(&s, "feed_pump").pump_two_phase,
            None,
            "{solver}: vapour at 100 °C"
        );
        assert!(
            (14.5..15.5).contains(&liquid),
            "{solver}: {liquid} kg/s at 100 °C"
        );

        apply(&mut engine, &Beat::Supply(110.0)).unwrap();
        worst = worst.max(run(&mut engine, 20, solver));
        let s = engine.snapshot();
        let pump = node(&s, "feed_pump")
            .pump_two_phase
            .expect("vapour at 110 °C");
        assert!(
            (0.07..0.165).contains(&pump.void_fraction),
            "{solver}: {pump:?}"
        );
        assert!(
            pump.head_multiplier > 0.1 && pump.head_multiplier < 0.9,
            "{solver}: {pump:?}"
        );
        assert!(!locked(&engine), "{solver}: locked at 110 °C");

        // A one-second surge to 120 °C: the suction offers over a third of its
        // volume in vapour, the pump has no push, and its pocket fills a third
        // of the way (M55.1, 3 s to fill). Back at 110 °C it drains at the same
        // rate, the push it takes fading as it goes, and the pump is where it was.
        let fall = flow(&engine.snapshot());
        apply(&mut engine, &Beat::Supply(120.0)).unwrap();
        worst = worst.max(run(&mut engine, 10, solver));
        assert!(!locked(&engine), "{solver}: a one-second surge locked it");
        let surged = pocket(&engine);
        assert!(
            (surged - 1.0 / 3.0).abs() < 1e-9,
            "{solver}: pocket {surged} after a second past the lock point"
        );
        apply(&mut engine, &Beat::Supply(110.0)).unwrap();
        worst = worst.max(run(&mut engine, 2, solver));
        let recovering = flow(&engine.snapshot());
        assert!(
            pocket(&engine) > 0.0 && recovering < fall,
            "{solver}: {recovering} kg/s with a part-full pocket, {fall} without"
        );
        worst = worst.max(run(&mut engine, 18, solver));
        assert_eq!(pocket(&engine), 0.0, "{solver}: the pocket did not drain");
        let after = flow(&engine.snapshot());
        assert!(
            (after - fall).abs() <= 1e-6 * fall,
            "{solver}: {after} kg/s after the surge, {fall} before it"
        );

        // Held at 120 °C the pocket fills in 3 s and the pump locks.
        apply(&mut engine, &Beat::Supply(120.0)).unwrap();
        worst = worst.max(run(&mut engine, 25, solver));
        assert!(
            !locked(&engine),
            "{solver}: locked before its pocket filled"
        );
        worst = worst.max(run(&mut engine, 15, solver));
        assert!(locked(&engine), "{solver}: not locked after 4 s at 120 °C");

        apply(&mut engine, &Beat::Supply(100.0)).unwrap();
        worst = worst.max(run(&mut engine, 20, solver));
        assert!(locked(&engine), "{solver}: cooling cleared the lock");
        let dead = flow(&engine.snapshot());
        assert!(
            dead < 0.6 * liquid,
            "{solver}: locked {dead} kg/s against {liquid}"
        );

        let refused = apply(&mut engine, &Beat::Vent).expect_err("vented while running");
        assert!(refused.contains("still running"), "{refused}");
        apply(&mut engine, &Beat::Pump(false)).unwrap();
        worst = worst.max(run(&mut engine, 10, solver));
        apply(&mut engine, &Beat::Vent).expect("a stopped, locked pump vents");
        apply(&mut engine, &Beat::Pump(true)).unwrap();
        worst = worst.max(run(&mut engine, 20, solver));
        let back = flow(&engine.snapshot());
        assert!(
            (back - liquid).abs() <= 1e-6 * liquid,
            "{solver}: vented and restarted at {back} kg/s, first {liquid}"
        );

        apply(&mut engine, &Beat::Destination(3.0)).unwrap();
        worst = worst.max(run(&mut engine, 20, solver));
        let into_3_bar = flow(&engine.snapshot());
        assert!(into_3_bar > 5.0, "{solver}: {into_3_bar} kg/s into 3 bar");
        assert!(!locked(&engine));

        apply(&mut engine, &Beat::Supply(125.0)).unwrap();
        worst = worst.max(run(&mut engine, 40, solver));
        assert!(locked(&engine), "{solver}: not locked at 125 °C");
        let shut = flow(&engine.snapshot());
        assert!(shut.abs() < 1e-6, "{solver}: {shut} kg/s past a shut disc");

        // Measured worst a tick over the story: Newton 11, the game solver 19
        // (21 before M55).
        let cap = if solver == "newton" { 14 } else { 26 };
        assert!(
            worst <= cap,
            "{solver}: {worst} iterations a tick, cap {cap}"
        );
    }
}

/// Net power [W] and mass rate [kg/s] across the plant's boundary, INTO it, and
/// the gross enthalpy flow [W] the residual is graded against — the books of
/// `flashing_rundown_reference.rs`: an edge with one end outside carries
/// `ṁ·(h + latent)`, an interior edge only its friction heat, and an edge leaving
/// the plant both (its published outlet temperature holds the heat its own
/// friction made). This plant holds nothing, so the books close to zero.
fn boundary(
    s: &Snapshot,
    slate: &Slate,
    enthalpy: &dyn EnthalpyModel,
    latent: bool,
) -> (f64, f64, f64) {
    let kind_of = |id| &s.nodes.iter().find(|n| n.id == id).unwrap().kind;
    let outside = |k: &NodeKind| matches!(k, NodeKind::Source { .. } | NodeKind::Sink { .. });
    let (mut power, mut mass, mut gross) = (0.0, 0.0, 0.0);
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
        match (outside(kind_of(e.from)), outside(kind_of(e.to))) {
            (true, false) => {
                power += flux;
                mass += e.stream.mass_flow.value();
            }
            (false, true) => {
                power += e.dissipation_w - flux;
                mass -= e.stream.mass_flow.value();
            }
            (false, false) => power += e.dissipation_w,
            (true, true) => panic!("an edge with both ends outside the plant"),
        }
        gross += flux.abs();
    }
    (power, mass, gross)
}

/// **The plant's energy and mass books close on every tick of the story** —
/// liquid, the pump on the table's fall, a surge its pocket survives, locked,
/// stopped, vented, pushing into 3 bar, and locked behind a shut disc — to
/// round-off of the gross enthalpy crossing the boundary (measured: at most
/// 2.2e-10 of the energy and 2.0e-10 of the mass; 1e-14 or less on liquid). The counterfactual is the same sum with the latent
/// terms left out, which must not close: from 110 °C the line past the valve
/// boils, and at 125 °C the supply does.
#[test]
fn the_books_close_through_the_whole_story() {
    let mut engine = build("newton");
    let slate = engine.slate.clone();
    let story: [(u64, Beat); 11] = [
        (20, Beat::Supply(110.0)),
        (40, Beat::Supply(120.0)),
        (50, Beat::Supply(110.0)),
        (70, Beat::Supply(120.0)),
        (110, Beat::Supply(100.0)),
        (130, Beat::Pump(false)),
        (140, Beat::Vent),
        (141, Beat::Pump(true)),
        (160, Beat::Destination(3.0)),
        (180, Beat::Supply(125.0)),
        (230, Beat::Supply(125.0)),
    ];
    let (mut worst, mut worst_without, mut worst_mass) = (0.0f64, 0.0f64, 0.0f64);
    let (mut scale, mut flow_scale) = (0.0f64, 0.0f64);
    for t in 1..=230u64 {
        engine.tick().unwrap_or_else(|e| panic!("tick {t}: {e}"));
        let s = engine.snapshot();
        let (power, mass, gross) = boundary(&s, &slate, engine.enthalpy(), true);
        let (power_sensible, _, _) = boundary(&s, &slate, engine.enthalpy(), false);
        let throughput: f64 = s
            .edges
            .iter()
            .map(|e| e.stream.mass_flow.value().abs())
            .sum();
        // Graded against the PLANT's scale — the largest gross enthalpy flow and
        // throughput the story has reached — not the tick's own: behind the shut
        // disc the line carries 1e-10 kg/s and 40 µW, where a 24 µW round-off
        // reads as "61%" of nothing.
        scale = scale.max(gross);
        flow_scale = flow_scale.max(throughput);
        worst = worst.max(power.abs() / scale);
        worst_without = worst_without.max(power_sensible.abs() / scale);
        worst_mass = worst_mass.max(mass.abs() / flow_scale);
        for (at, beat) in &story {
            if *at == t {
                apply(&mut engine, beat).unwrap();
            }
        }
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
        worst_without > 1e-3,
        "without the latent terms the books still close ({worst_without:e}): nothing \
         two-phase crossed the boundary and the gate proves nothing"
    );
}
