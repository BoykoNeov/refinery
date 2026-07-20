//! The reactor, as wired: `scenarios/fcc_reactor.toml` end to end (M4.1).
//!
//! `core::energy`'s unit tests already prove the reactor's two duties and the
//! imposed setpoint against hand-built flows, and `solvers::reactor`'s tests
//! prove `SimpleLookup`'s per-lump conversion and its mass-conserving
//! normalization in isolation. Re-deriving either here would add nothing.
//!
//! What is NOT covered anywhere else, and is what this file exists for:
//!   1. the `reactions = "lookup"` selection and the `t_set_c`/`tau_s` loader
//!      arm reaching a real `NodeKind::Reactor`,
//!   2. the reaction running INSIDE a real hydraulic solve + transport sweep, so
//!      the product edge downstream carries the cracked composition,
//!   3. the TOTAL-MASS gate: the reactor changes composition by chemistry
//!      (gasoil → gasoline + gas + coke) yet conserves total mass — the mass in
//!      equals the mass out DESPITE the per-component books not balancing, which
//!      is the whole reason a reactor is the first unit to break I7.
//!
//! The gate is not vacuous: it asserts the composition genuinely CHANGED (a
//! reaction that did nothing would trivially conserve mass) in the same breath
//! as asserting the total is unmoved.
//!
//! Lives in `scenarios/` for the same reason as `furnace_reference.rs`: it needs
//! the TOML and its loader, and `scenarios` depends on `solvers`.

use refinery_core::snapshot::{EdgeSnapshot, NodeSnapshot};

const SCENARIO: &str = include_str!("../../../scenarios/fcc_reactor.toml");

/// Slate order in the file: gas, gasoline, gasoil, coke.
const GAS: usize = 0;
const GASOLINE: usize = 1;
const GASOIL: usize = 2;
const COKE: usize = 3;

/// The `t_set_c = 520.0` the TOML declares, in SI (K). Written here as an
/// independent constant: if the loader's °C→K offset is dropped or wrong, the
/// reactor's outlet temperature misses this.
const T_SET_K: f64 = 520.0 + 273.15;

/// The FCC placeholder yields for one unit of gasoil, in slate order
/// (gas, gasoline, gasoil, coke) — the table `SimpleLookup::fcc_demo` carries.
/// Pinned here as the reference the wired plant must reproduce; a change to the
/// demo table is meant to fail this.
const CRACKED: [f64; 4] = [0.15, 0.55, 0.25, 0.05];

/// Ticks to run before reading. The plant has no inventory, so the hydraulic
/// solve is at steady state from tick 1; a handful confirms it stays there.
const TICKS: u64 = 20;

/// Composition and flow come from products and O(1) divisions of doubles, so
/// relative error is a few ulp. 1e-9 on ~1-magnitude fractions is far below any
/// modelling defect and far above float noise.
const TOL: f64 = 1e-9;

fn edge<'a>(snap: &'a [EdgeSnapshot], name: &str) -> &'a EdgeSnapshot {
    snap.iter()
        .find(|e| e.name == name)
        .unwrap_or_else(|| panic!("scenario must define edge '{name}'"))
}

fn node<'a>(snap: &'a [NodeSnapshot], name: &str) -> &'a NodeSnapshot {
    snap.iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("scenario must define node '{name}'"))
}

#[test]
fn the_reactor_cracks_gasoil_yet_conserves_total_mass() {
    let file = refinery_scenarios::load_str(SCENARIO).expect("the fcc scenario must parse");
    let mut engine = refinery_scenarios::build_engine(&file).expect("the fcc plant must build");
    for _ in 0..TICKS {
        engine.tick().expect("the fcc plant must tick");
    }
    let snap = engine.snapshot();

    let feed = &edge(&snap.edges, "feed_line").stream;
    let product = &edge(&snap.edges, "product_line").stream;

    // The feed is pure gasoil, unchanged upstream of the reactor.
    let feed_frac = feed.composition.fractions();
    assert!(
        (feed_frac[GASOIL] - 1.0).abs() < TOL && feed_frac[GASOLINE] < TOL,
        "the feed edge must carry pure gasoil, got {feed_frac:?}"
    );

    // The product edge carries the CRACKED slate — the reaction actually ran.
    let prod_frac = product.composition.fractions();
    assert!(
        prod_frac[GASOLINE] > 0.5 && prod_frac[GASOIL] < 0.3,
        "the reaction must have cracked gasoil into gasoline (product {prod_frac:?})"
    );
    for (i, &expected) in CRACKED.iter().enumerate() {
        assert!(
            (prod_frac[i] - expected).abs() < TOL,
            "product fraction {i} = {} must match the demo table {expected}",
            prod_frac[i]
        );
    }
    // The reaction is total-mass-neutral: the product distribution sums to 1.
    let sum: f64 = prod_frac.iter().sum();
    assert!(
        (sum - 1.0).abs() < TOL,
        "product composition must sum to 1 (mass neutrality), got {sum}"
    );

    // THE GATE: mass in = mass out through the reactor, despite the composition
    // change above. Both edges point away from their upwind end, so both flows
    // are positive and must be equal.
    let (m_in, m_out) = (feed.mass_flow.value(), product.mass_flow.value());
    assert!(
        m_in > 0.0,
        "the plant must actually be flowing, got {m_in} kg/s"
    );
    assert!(
        (m_in - m_out).abs() < TOL * m_in.max(1.0),
        "total mass must be conserved through the reactor: {m_in} kg/s in vs {m_out} kg/s out"
    );

    // Per-COMPONENT mass is NOT conserved — the point of a reactor. Gasoil mass
    // in exceeds gasoil mass out; the difference reappears as other lumps.
    let gasoil_in = m_in * feed_frac[GASOIL];
    let gasoil_out = m_out * prod_frac[GASOIL];
    assert!(
        gasoil_out < gasoil_in - TOL,
        "gasoil mass must FALL across the reactor ({gasoil_in} → {gasoil_out} kg/s): a reactor \
         breaks per-component conservation by design"
    );
    let _ = (GAS, COKE); // slate positions documented above; asserted via sum.

    // The setpoint is imposed on the reactor's outlet, not mixed from the feed.
    let riser = node(&snap.nodes, "riser");
    assert!(
        (riser.temperature_k - T_SET_K).abs() < 1e-6,
        "the reactor must hold t_set = {T_SET_K} K, got {}",
        riser.temperature_k
    );
}
