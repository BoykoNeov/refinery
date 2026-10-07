//! M53.1 — the flashing rundown (`scenarios/flashing_rundown.toml`,
//! docs/DESIGN.md §58): hot naphtha let down through a control valve, boiling
//! in the line after it, into a vented tank whose boil-off carries the vapour
//! away; and, by command, a supply warmed past its own boiling point.
//!
//! The flash itself is held to a hand calculation in
//! `crates/solvers/tests/reference/line_flash.rs`. These gates hold the plant:
//! the line boiling where the pressure says it must, the junction's published
//! state carrying the enthalpy that arrived, the plant's external energy and
//! mass books closing every tick — through a boiling supply and a tank run dry —
//! friction booked on the liquid's share of a boiling pipe, and the two
//! fidelities agreeing.

use refinery_core::components::Slate;
use refinery_core::energy::T_REF;
use refinery_core::graph::NodeKind;
use refinery_core::snapshot::{Command, EdgeSnapshot, NodeSnapshot, Snapshot};
use refinery_core::traits::EnthalpyModel;
use refinery_core::units::Kelvin;
use refinery_core::Engine;

const DEMO: &str = include_str!("../../../scenarios/flashing_rundown.toml");
const DT: f64 = 0.5;

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the scenario must parse");
    refinery_scenarios::build_engine(&file).unwrap_or_else(|e| panic!("the plant must build: {e}"))
}

fn on_solver(solver: &str) -> String {
    DEMO.replace("flow = \"newton\"", &format!("flow = \"{solver}\""))
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

fn edge<'a>(s: &'a Snapshot, name: &str) -> &'a EdgeSnapshot {
    s.edges.iter().find(|e| e.name == name).unwrap()
}

fn warm_supply(engine: &mut Engine, celsius: f64) {
    let supply = engine.graph.find_node("rundown").unwrap();
    engine
        .apply(Command::SetSourceTemperature {
            node: supply,
            temperature: Kelvin(celsius + 273.15),
        })
        .expect("a flashing plant takes a boiling supply");
}

/// **The line boils after the valve, and nowhere before it.** The valve's inlet
/// is liquid (2.93 bar against a bubble pressure of 2.08); just past it, at about
/// 1.40 bar, a tenth of the mass is vapour and the stream has cooled from 115 °C
/// to about 101 °C. And the junction's published state carries exactly the
/// enthalpy that arrived: `h(T_j) + q·λ = h(T_in)` for its one liquid inflow,
/// whose published temperature already holds the friction upstream of it.
#[test]
fn the_line_boils_after_the_valve_and_keeps_its_enthalpy() {
    let mut engine = build(DEMO);
    for t in 1..=600 {
        tick(&mut engine, t);
    }
    let s = engine.snapshot();
    assert_eq!(node(&s, "rundown").vapour_fraction, None);
    assert_eq!(node(&s, "rundown_valve").vapour_fraction, None);
    let junction = node(&s, "valve_outlet");
    let q = junction
        .vapour_fraction
        .expect("the line boils past the valve");
    assert!((0.09..0.11).contains(&q), "{q}");
    assert!(
        junction.temperature_k < 115.0 + 273.15 - 10.0,
        "it cools as it boils: {} K",
        junction.temperature_k
    );
    assert_eq!(edge(&s, "flash_line").stream.vapour_fraction, Some(q));

    let arriving = &edge(&s, "valve_spool").stream;
    assert_eq!(arriving.vapour_fraction, None);
    let enthalpy = engine.enthalpy();
    let slate = &engine.slate;
    let h_in = enthalpy
        .specific_enthalpy(slate, &arriving.composition, arriving.temperature)
        .unwrap()
        .value();
    let latent = edge(&s, "flash_line").stream.latent.unwrap().value();
    let h_out = enthalpy
        .specific_enthalpy(slate, &arriving.composition, Kelvin(junction.temperature_k))
        .unwrap()
        .value()
        + latent;
    assert!(
        (h_out - h_in).abs() <= 1e-9 * h_in.abs(),
        "the junction holds {h_out} J/kg of the {h_in} J/kg that arrived"
    );
}

/// Every holdup's internal energy [J] on the published state: a tank's
/// `m·c̄p·(T − T_REF)` (a liquid's `cv` is its `cp`).
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

fn holdup_mass(s: &Snapshot) -> f64 {
    s.nodes
        .iter()
        .filter_map(|n| match &n.kind {
            NodeKind::Tank(t) => Some(t.mass.value()),
            _ => None,
        })
        .sum()
}

/// Net power [W] and mass rate [kg/s] across the plant's boundary, INTO it, from
/// published state, and the GROSS enthalpy flow [W] the residual is graded
/// against: an edge with one end outside carries `ṁ·(h + latent)` in or out, an
/// interior edge contributes only its friction heat (`boiloff_reference.rs`'s
/// books; this plant has no heat loads) — and so does an edge LEAVING the plant,
/// whose published outlet temperature holds the heat its own friction made
/// inside the boundary. The boil-off plants never needed that last term: none of
/// their outgoing edges has friction. Measured here without it, the books miss
/// by exactly the product line's 291 W.
fn boundary(
    s: &Snapshot,
    slate: &Slate,
    enthalpy: &dyn EnthalpyModel,
    latent: bool,
) -> (f64, f64, f64) {
    let kind_of = |id| &s.nodes.iter().find(|n| n.id == id).unwrap().kind;
    let outside = |k: &NodeKind| {
        matches!(
            k,
            NodeKind::Source { .. } | NodeKind::Sink { .. } | NodeKind::Atmosphere
        )
    };
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
            // Leaving the plant, at its published OUTLET temperature, which
            // holds the heat its own friction made inside the boundary: that
            // heat is a source here, and it leaves in the flux.
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

/// **The plant's external energy and mass books close on every tick** — the
/// line boiling into the tank, then the supply warmed to 135 °C so that it boils
/// where it stands, the tank draining and running dry. Arriving latent heat has
/// ONE route into a tank's books (its balance; its boil-off vents what it boils),
/// and this is the gate that holds it to that: drop the route and the tank's
/// energy falls short of what crossed the boundary. The counterfactual is the
/// same sum with the latent terms left out, which must not close.
#[test]
fn the_energy_and_mass_books_close_through_a_boiling_supply_and_a_dry_tank() {
    let mut engine = build(DEMO);
    let slate = engine.slate.clone();
    let mut previous = engine.snapshot();
    let (mut worst, mut worst_without, mut worst_mass) = (0.0f64, 0.0f64, 0.0f64);
    let mut ran_dry = false;
    for t in 1..=1500 {
        if t == 300 {
            warm_supply(&mut engine, 135.0);
        }
        tick(&mut engine, t);
        let now = engine.snapshot();
        let accumulation = (holdup_energy(&now, &slate) - holdup_energy(&previous, &slate)) / DT;
        let (power, mass_in, gross) = boundary(&now, &slate, engine.enthalpy(), true);
        let (power_sensible, _, _) = boundary(&now, &slate, engine.enthalpy(), false);
        // Graded against the GROSS enthalpy crossing the boundary: the net
        // accumulation is near zero at the steady level, and round-off scales
        // with the terms summed, not with what is left of them.
        worst = worst.max((accumulation - power).abs() / gross);
        worst_without = worst_without.max((accumulation - power_sensible).abs() / gross);
        let mass_rate = (holdup_mass(&now) - holdup_mass(&previous)) / DT;
        let throughput: f64 = now
            .edges
            .iter()
            .map(|e| e.stream.mass_flow.value().abs())
            .sum();
        worst_mass = worst_mass.max((mass_rate - mass_in).abs() / throughput);
        ran_dry |= now.tanks.iter().any(|(_, tank)| tank.mass.value() < 1.0);
        previous = now;
    }
    assert!(
        ran_dry,
        "the hot supply must drain the tank, or the dry path is untested"
    );
    // Measured: 4.6e-10 and 6.9e-10 (release, 3 000 ticks) — round-off.
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

/// **The settled plant against a hand calculation of the whole of it.** At steady
/// state the tank is one equilibrium stage at atmospheric pressure: its liquid
/// `x` on its bubble point, its vent `y = K·x`, the light cut's balance fixing the
/// vented share `V`, and the energy balance fixing `T` —
///
/// ```text
///   H_in = h(115 °C) + Φ/ṁ = 249 786.19 J/kg   (Φ: the 1 352.05 W of friction
///                                                upstream of the tank)
///   H_in = V·(h_y(T) + λ) + (1 − V)·h_x(T)
/// ```
///
/// solved by bisection in a few lines of Python outside the repo (ROADMAP M53):
/// **V = 0.179 301, T = 90.136 60 °C**, with `λ` as the tank's boil-off sizes it
/// (M12: mass-weighted over the LIQUID's fractions, 302 227 J/kg). With the
/// VAPOUR's own `λ`, as the line flash takes it, a single flash gives
/// V = 0.175 586 — the two conventions meeting in one tank, ledger row B51.
///
/// The engine lands +1.9e-5 and −2.8 mK off, and that is the boil-off's own
/// timestep: it parks the tank on the bubble point of the liquid it had BEFORE
/// the vapour left, so the published liquid sits just below its own bubble point.
/// Measured at half the step: +1.1e-5 and −1.4 mK — first order, converging on
/// the hand calculation. The tank starts at the settled state here, so 600 ticks
/// suffice (the settled numbers move by 2e-7 after that).
#[test]
fn the_settled_plant_lands_on_a_hand_calculation_of_the_whole() {
    let settled = DEMO.replace(
        "initial_level_m = 4.5
temperature_c = 88.0
composition = { light_naphtha = 0.7, heavy_naphtha = 0.3 }",
        "initial_level_m = 4.755
temperature_c = 90.137
composition = { light_naphtha = 0.6488, heavy_naphtha = 0.3512 }",
    );
    assert_ne!(
        settled, DEMO,
        "the tank's initial state must have been replaced"
    );
    let mut engine = build(&settled);
    for t in 1..=600 {
        tick(&mut engine, t);
    }
    let s = engine.snapshot();
    let fed = edge(&s, "rundown_line").stream.mass_flow.value();
    let vented = edge(&s, "rundown_tank__boiloff_vent")
        .stream
        .mass_flow
        .value();
    let share = vented / fed;
    let tank = s.tanks[0].1.temperature.value() - 273.15;
    assert!((share - 0.179_301_4).abs() < 2.5e-5, "vented share {share}");
    assert!((tank - 90.136_60).abs() < 3.5e-3, "tank at {tank} °C");
    assert!(
        (share - 0.175_586).abs() > 3e-3,
        "the tank vents by its own latent heat, not the vapour's: {share}"
    );
}

/// **Friction heats the liquid's share of a boiling pipe, not its whole
/// volume** (§58 fork 6). On `flash_line` — no elevation, so its whole drop is
/// friction — the booked heat over `ΔP·ṁ·(1 − q)` is `1/ρ_l`, and the liquid's
/// density is bounded by its two cuts' (680 and 750 kg/m³) whatever its
/// composition. Booked on the whole flowing volume it would be `1/(ρ·(1 − q))`,
/// with `ρ` the mixture's 44 kg/m³ — sixteen times more.
#[test]
fn friction_heats_the_liquid_share_of_a_boiling_pipe_only() {
    let mut engine = build(DEMO);
    for t in 1..=600 {
        tick(&mut engine, t);
    }
    let s = engine.snapshot();
    let line = edge(&s, "flash_line");
    let q = line.stream.vapour_fraction.expect("the line boils");
    let drop = node(&s, "valve_outlet").pressure_pa - node(&s, "rundown_tank").pressure_pa;
    let per_liquid_volume = line.dissipation_w / (drop * line.stream.mass_flow.value() * (1.0 - q));
    assert!(
        (1.0 / 750.0..=1.0 / 680.0).contains(&per_liquid_volume),
        "friction heat per unit liquid volume {per_liquid_volume:e} m³/kg: not 1/ρ_liquid"
    );
}

/// **Both fidelities land on one plant**, settled and after the supply boils.
#[test]
fn both_fidelities_agree_on_the_flashing_rundown() {
    let run = |solver: &str| {
        let mut engine = build(&on_solver(solver));
        let mut readings = Vec::new();
        for t in 1..=900 {
            if t == 600 {
                warm_supply(&mut engine, 135.0);
            }
            tick(&mut engine, t);
            if t == 600 || t == 900 {
                let s = engine.snapshot();
                readings.push((
                    edge(&s, "rundown_line").stream.mass_flow.value(),
                    edge(&s, "rundown_tank__boiloff_vent")
                        .stream
                        .mass_flow
                        .value(),
                    node(&s, "valve_outlet").vapour_fraction.unwrap_or(0.0),
                    node(&s, "valve_outlet").temperature_k,
                ));
            }
        }
        readings
    };
    let (newton, simple) = (run("newton"), run("simple"));
    for (n, g) in newton.iter().zip(&simple) {
        let pairs = [(n.0, g.0), (n.1, g.1), (n.2, g.2), (n.3, g.3)];
        for (a, b) in pairs {
            assert!(
                (a - b).abs() <= 1e-6 * a.abs().max(1e-3),
                "newton {n:?} against simple {g:?}"
            );
        }
    }
}

/// **A supply warmed past its own boiling point boils where it stands** — the
/// ledger row B46 this milestone was opened for. At 135 °C and 3.0 bar it is
/// 34.75% vapour by mass, the independent hand calculation's
/// 0.347 526 116 816 403; the line's flow falls by more than half, and what
/// leaves the tank after it runs dry is still boiling.
#[test]
fn a_supply_warmed_past_boiling_boils_where_it_stands() {
    let mut engine = build(DEMO);
    for t in 1..=300 {
        tick(&mut engine, t);
    }
    let cold_flow = edge(&engine.snapshot(), "rundown_line")
        .stream
        .mass_flow
        .value();
    warm_supply(&mut engine, 135.0);
    for t in 301..=1500 {
        tick(&mut engine, t);
    }
    let s = engine.snapshot();
    let q = node(&s, "rundown")
        .vapour_fraction
        .expect("the supply boils");
    assert!((q - 0.347_526_116_816_403).abs() < 1e-9, "{q}");
    assert_eq!(edge(&s, "rundown_line").stream.vapour_fraction, Some(q));
    let hot_flow = edge(&s, "rundown_line").stream.mass_flow.value();
    assert!(
        hot_flow < 0.5 * cold_flow,
        "{hot_flow} kg/s against {cold_flow}"
    );
    // The tank has run dry and passes its inflow straight through: settled at
    // its own pressure, as a junction is — not handed on as liquid holding the
    // latent heat as superheat (184 °C, measured before the fix). Read on the
    // tank's OWN outlet: the outlet valve downstream re-flashes either way.
    assert!(
        s.tanks.iter().all(|(_, tank)| tank.mass.value() < 1.0),
        "the tank must be dry"
    );
    let drained = &edge(&s, "tank_outlet").stream;
    assert!(
        drained.vapour_fraction.is_some(),
        "the dry tank passes the boiling stream on as what it is"
    );
    assert!(
        drained.temperature.value() < 120.0 + 273.15,
        "{} K",
        drained.temperature.value()
    );
}
