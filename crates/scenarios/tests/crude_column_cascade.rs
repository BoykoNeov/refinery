//! M7.4c: the two crude-column demos, as shipped, and against each other.
//!
//! `scenarios/crude_column.toml` and `scenarios/crude_column_cascade.toml` are
//! the same plant on the two separation fidelities. The cascade's algebra is
//! gated in isolation in `refinery-solvers/tests/reference/cascade.rs` and the
//! cascade AS WIRED in `cascade_column.rs`; what neither of those can see, and
//! this file does, is the pair:
//!
//! * a **side draw** off a real tray, which no other wired test has — every
//!   cascade fixture in `cascade_column.rs` is a two-draw column, so the middle
//!   arm of `CascadePlan`'s draw loop was reached by unit tests only;
//! * the demo's **feed design**, which M7.4b made a precondition rather than a
//!   preference: the file has to deliver its column a saturated liquid, through a
//!   furnace, across a first-tick flow transient, for a long run;
//! * the two files' **draw flows agreeing exactly** while everything else about
//!   the products differs, which is what makes the pair a comparison rather than
//!   two demos that happen to share a slate;
//! * and the cut-point column's **smearing ramp**, which M7.1 measured as set in
//!   every demo file in the repo and changing no number in any of them.
//!
//! `Engine` is not `Debug` (boxed solver traits), so failures are unwrapped by
//! hand rather than with `expect`/`expect_err` on the engine itself.

use refinery_core::engine::Engine;
use refinery_core::graph::NodeKind;
use refinery_scenarios::{build_engine, load_str};

const SPLITTER: &str = include_str!("../../../scenarios/crude_column.toml");
const CASCADE: &str = include_str!("../../../scenarios/crude_column_cascade.toml");

/// The slate order both files declare, lightest first.
const LIGHT_NAPHTHA: usize = 0;
const HEAVY_NAPHTHA: usize = 1;
const KEROSENE: usize = 2;
const DIESEL: usize = 3;
const RESIDUE: usize = 4;

fn build(src: &str) -> Engine {
    build_engine(&load_str(src).expect("a shipped scenario must parse"))
        .unwrap_or_else(|e| panic!("a shipped scenario must build: {e}"))
}

/// Run `ticks` ticks, or panic naming the one that failed. Both demos are steady
/// from tick 2 on, so any tick past the first is the steady state.
fn run(src: &str, ticks: u32) -> Engine {
    let mut engine = build(src);
    for tick in 1..=ticks {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick} of a shipped demo must converge: {e}"));
    }
    engine
}

fn edge(engine: &Engine, name: &str) -> refinery_core::stream::Stream {
    let eid = engine
        .graph
        .edge_ids()
        .find(|e| engine.graph.pipe(*e).name == name)
        .unwrap_or_else(|| panic!("edge '{name}' should exist"));
    engine.graph.pipe(eid).stream.clone()
}

fn flow(engine: &Engine, name: &str) -> f64 {
    edge(engine, name).mass_flow.value()
}

fn fractions(engine: &Engine, name: &str) -> Vec<f64> {
    edge(engine, name).composition.fractions().to_vec()
}

fn temperature(engine: &Engine, name: &str) -> f64 {
    edge(engine, name).temperature.value()
}

const DRAWS: [&str; 3] = ["naphtha_draw", "distillate_draw", "bottoms_draw"];

// ---------------------------------------------------------------------------
// The pair, side by side.
// ---------------------------------------------------------------------------

/// **The demo's whole claim, as an assertion.** The two files put the SAME three
/// mass rates into the same three tanks and separate them completely differently.
///
/// The rates agreeing is not a coincidence and is not free: the cascade's
/// `draw_ratio`s are set to the mass yields the splitter next door produces, and
/// the two plants are hydraulically identical (same source pressure, same pipe
/// geometry, and a stream's density comes from its composition rather than its
/// temperature, so the cascade's cooler feed does not move the flow). That makes
/// the rates a CONTROL: with them pinned, every difference below is the
/// separation model and nothing else.
///
/// What differs, and each of these fails on a different kind of mistake:
///
/// * **The splitter's bands are sharp** (up to the ramp). Its bottoms is pure
///   residue and its distillate carries none, because a cut either falls in a
///   boiling range or it does not. The cascade's bottoms carries 69% residue and
///   30% diesel, and its distillate carries 11% residue — every cut appears in
///   more than one product, which is what an equilibrium stage does and a band
///   cannot.
/// * **The splitter's draws are isothermal.** All three leave at the one feed
///   temperature, because a boiling range has no trays; the cascade's leave at
///   387.6 / 505.3 / 632.0 K, strictly ordered and straddling the feed.
/// * **Only one of them has equipment.** The cascade reports a condenser and a
///   reboiler duty; the splitter reports none at all — `None`, not zero (M7.4b
///   correction 5).
#[test]
fn the_two_demos_split_the_same_rates_into_different_products() {
    let splitter = run(SPLITTER, 200);
    let cascade = run(CASCADE, 200);

    // The control: identical feed, identical draw rates.
    approx::assert_relative_eq!(
        flow(&splitter, "hot_feed_line"),
        flow(&cascade, "hot_feed_line"),
        max_relative = 1e-9
    );
    for draw in DRAWS {
        let a = flow(&splitter, draw);
        let b = flow(&cascade, draw);
        assert!(a > 1.0, "'{draw}' must carry real flow, got {a} kg/s");
        approx::assert_relative_eq!(a, b, max_relative = 1e-9);
    }

    // A cut that the splitter puts in exactly one draw, the cascade spreads.
    // Residue is the sharpest case: 100% of it leaves in the bottoms next door.
    approx::assert_relative_eq!(
        fractions(&splitter, "bottoms_draw")[RESIDUE],
        1.0,
        max_relative = 1e-12
    );
    assert_eq!(
        fractions(&splitter, "distillate_draw")[RESIDUE],
        0.0,
        "a boiling-range band above the residue's cut point cannot contain any"
    );
    let cascade_bottoms = fractions(&cascade, "bottoms_draw");
    let cascade_distillate = fractions(&cascade, "distillate_draw");
    assert!(
        cascade_bottoms[RESIDUE] < 0.9 && cascade_bottoms[DIESEL] > 0.1,
        "an equilibrium bottoms is not a pure cut: got {cascade_bottoms:?}"
    );
    assert!(
        cascade_distillate[RESIDUE] > 0.01,
        "a side draw five stages down must carry some of the heaviest cut — an \
         equilibrium stage separates by volatility, not by a boundary in tb; got \
         {cascade_distillate:?}"
    );

    // Isothermal draws next door; a real profile here.
    // The column is fed by the feed line's DOWNSTREAM half: since M37 every
    // furnace outlet is split for its burn-out hole (docs/DESIGN.md §42), and the
    // declared name ends at the hole's junction, half a pipe of friction short.
    let feed = temperature(&splitter, "hot_feed_line__downstream");
    for draw in DRAWS {
        approx::assert_relative_eq!(temperature(&splitter, draw), feed, max_relative = 1e-12);
    }
    let profile: Vec<f64> = DRAWS.iter().map(|d| temperature(&cascade, d)).collect();
    let cascade_feed = temperature(&cascade, "hot_feed_line__downstream");
    assert!(
        profile[0] < profile[1] && profile[1] < profile[2],
        "a cascade's draws are strictly ordered top to bottom, got {profile:?}"
    );
    assert!(
        profile[0] < cascade_feed && profile[2] > cascade_feed,
        "a saturated-liquid feed sits strictly inside the column's own profile: got \
         {profile:?} against a feed at {cascade_feed} K"
    );

    // Equipment on one fidelity only.
    let splitter_json = serde_json::to_string(&splitter.snapshot()).unwrap();
    assert!(
        !splitter_json.contains("column_duty"),
        "a cut-point column has no condenser or reboiler to report on"
    );
    let column = cascade.graph.find_node("column").unwrap();
    let duties = &cascade.node_states().column_separation[&column];
    let condenser = duties.condenser_duty.expect("the cascade computes duties");
    let reboiler = duties.reboiler_duty.expect("the cascade computes duties");
    assert!(
        condenser.value() > 1.0e6 && reboiler.value() > condenser.value(),
        "separating 193 kg/s of crude costs megawatts, and the reboiler carries the \
         condenser's load plus the column's external sensible balance; got \
         {condenser:?} and {reboiler:?}"
    );
}

// ---------------------------------------------------------------------------
// The splitter demo's smearing ramp — M7.1's measurement, closed.
// ---------------------------------------------------------------------------

/// **`smearing_k` finally changes a number, and this is the hand calculation.**
///
/// M7.1 measured that `smearing_k` is set in every demo file in this repo and
/// exercised by exactly one test — the unit test that moved with the code — since
/// no component's boiling point landed inside a ramp anywhere. M7.4c moves
/// `crude_column.toml`'s first cut from 185 °C to 155 °C for that reason, and this
/// gate is what stops it drifting back out.
///
/// The ramp is linear and centred on the cut, full width `smearing_k`, so the
/// mass fraction of a cut boiling at `tb` landing ABOVE a cut at `T_cut` is
/// `0.5 + (tb − T_cut)/s`, clamped. With `tb` = 150 °C, `T_cut` = 155 °C and
/// `s` = 25 K that is `0.5 − 0.2` = **0.3**, so the heavy naphtha goes 70/30
/// overhead/down and appears in TWO draws. Everything below follows from that one
/// number and the feed's declared 0.12/0.18/0.25/0.25/0.20:
///
/// ```text
/// naphtha    = 0.12 + 0.7·0.18 = 0.246      of which heavy = 0.126/0.246 = 0.51220
/// distillate = 0.3·0.18 + 0.50 = 0.554      of which heavy = 0.054/0.554 = 0.09747
/// bottoms    = 0.20
/// ```
///
/// A sharp splitter — smearing disabled, or a cut back outside every ramp — gives
/// 0.30 / 0.50 / 0.20 with the heavy naphtha wholly overhead, which fails every
/// line here. The declared feed composition is used rather than one read back off
/// the engine, so the split rule appears on one side of the comparison only.
#[test]
fn the_cut_point_demo_smears_a_component_across_two_draws() {
    let engine = run(SPLITTER, 200);
    let feed = flow(&engine, "hot_feed_line");

    for (draw, expected) in DRAWS.iter().zip([0.246, 0.554, 0.200]) {
        approx::assert_relative_eq!(flow(&engine, draw) / feed, expected, max_relative = 1e-9);
    }

    let naphtha = fractions(&engine, "naphtha_draw");
    let distillate = fractions(&engine, "distillate_draw");
    approx::assert_relative_eq!(naphtha[HEAVY_NAPHTHA], 0.126 / 0.246, max_relative = 1e-9);
    approx::assert_relative_eq!(
        distillate[HEAVY_NAPHTHA],
        0.054 / 0.554,
        max_relative = 1e-9
    );
    // The other four cuts stay sharp — this is a ramp across ONE boundary, not a
    // general blurring of the split, and a smearing width applied to every cut
    // would fail here while passing the two lines above.
    assert_eq!(
        (naphtha[KEROSENE], naphtha[DIESEL], naphtha[RESIDUE]),
        (0.0, 0.0, 0.0),
        "no cut but the heavy naphtha boils inside a ramp, so the naphtha draw is \
         otherwise a clean band: {naphtha:?}"
    );
    assert_eq!(
        distillate[LIGHT_NAPHTHA], 0.0,
        "the light naphtha boils 75 K below the first cut and cannot reach the \
         distillate: {distillate:?}"
    );
}

// ---------------------------------------------------------------------------
// The cascade demo's feed design.
// ---------------------------------------------------------------------------

/// **The design input the file's header claims, measured.**
///
/// The cascade admits a saturated-liquid feed only and refuses anything more than
/// 1% of the feed off-phase — about ±1.18 K here. Two separate things have to
/// stay inside that window and only one of them is the number in the file:
///
/// 1. **The steady state**, which is what `duty_mw` was derived for. It must land
///    on the bubble point rather than merely inside the window, because the window
///    is what absorbs everything else.
/// 2. **The first tick**, which runs at a different flow. A pipe's transport
///    density comes from its STORED composition, which on tick 1 is the
///    composition it was born with rather than the crude it is about to carry — so
///    the feed rate steps from 176.478 to 192.685 kg/s between ticks 1 and 2, and
///    a 9.2% flow step lands directly on the furnace's `Q/(ṁ·c̄p)` rise. That is
///    why the header calls the small heater a design decision: at this duty tick 1
///    sits 0.53 K above saturation, and a 30 K preheat would put it 2.7 K above and
///    the plant would refuse its own first tick.
///
/// Measured over 400 ticks rather than one, per `a-cached-classification-expires`:
/// a plant admissible at `t = 0` has not been shown to stay admissible. It is the
/// same reasoning `the_cascade_column_stays_admissible_for_a_long_run` applies to
/// the M7.3 fixture, on a plant with a furnace in the loop.
///
/// **This test caught nothing in M7.4c's mutation pass, and that is what it should
/// do.** It is not a gate on the model — it is a gate on the FILE, and the faults
/// it exists for are edits to `crude_column_cascade.toml` (a different duty, a
/// different source temperature, a different column pressure), which no mutation
/// of the engine can produce. Recorded rather than deleted, on M7.4b's precedent
/// for the two tests there that said the same thing in advance.
#[test]
fn the_cascade_demo_feeds_its_column_a_saturated_liquid_from_the_first_tick() {
    let mut engine = build(CASCADE);
    let column = engine.graph.find_node("column").unwrap();
    let pressure = match &engine.graph.node(column).kind {
        NodeKind::Column { pressure, .. } => *pressure,
        other => panic!("expected a column, got {other:?}"),
    };
    // The feed composition is the source's, declared in the file and unchanged by
    // anything upstream — the furnace is a pass-through.
    let feed_mix =
        refinery_core::components::Composition::from_weights(&[0.12, 0.18, 0.25, 0.25, 0.20])
            .unwrap();
    let bubble = bubble_point_of(&engine.slate, &feed_mix, pressure);

    let mut first = f64::NAN;
    let mut steady = f64::NAN;
    let mut worst = 0.0f64;
    for tick in 1..=400 {
        engine
            .tick()
            .unwrap_or_else(|e| panic!("tick {tick} must converge, not refuse the feed: {e}"));
        let offset = temperature(&engine, "hot_feed_line__downstream") - bubble;
        if tick == 1 {
            first = offset;
        }
        if tick == 2 {
            steady = offset;
        }
        worst = worst.max(offset.abs());
    }

    // The steady state is ON the bubble point, not merely inside the window.
    assert!(
        steady.abs() < 0.01,
        "the derived duty must land the steady feed on its bubble point ({bubble} K), got \
         {steady} K off. `duty_mw` is Q = ṁ·c̄p·ΔT at 192.685 kg/s and needs re-deriving \
         against whatever this plant now produces."
    );
    // The first tick lands there too (M55.0, docs/DESIGN.md §60.0). Until M55 it
    // flowed its lines at `Stream::stagnant`'s placeholder liquid — the slate's
    // first component — so the tick-1 flow was lower, the furnace's rise higher,
    // and the feed +0.53 K superheated: the one tick that used the ±1.18 K
    // window. Measured since: −0.002 K, the steady offset's size.
    assert!(
        first.abs() < 0.01,
        "the first tick put the feed {first} K off saturation: a liquid line flowing \
         its first tick at a placeholder composition again?"
    );
    assert!(worst < 0.01, "the feed left its bubble point by {worst} K");
}

/// The bubble point of a MASS composition at `pressure` [K], by bisection on
/// `Σ Kᵢ(T)·xᵢ = 1` over mole fractions — a second implementation, using only the
/// published `ThermoModel::k_value`, for `cascade_column.rs`'s reason: the
/// cascade's own routine is private, and a gate that called it would be reading
/// the answer back.
fn bubble_point_of(
    slate: &refinery_core::components::Slate,
    mass: &refinery_core::components::Composition,
    pressure: refinery_core::units::Pascal,
) -> f64 {
    use refinery_core::traits::ThermoModel;
    use refinery_core::units::Kelvin;
    use refinery_solvers::{MoleFractions, TroutonThermo};

    let thermo = TroutonThermo::new();
    let moles = MoleFractions::from_mass(mass, slate).expect("the feed is a valid mix");
    let sum_kx = |t: f64| -> f64 {
        moles
            .fractions()
            .iter()
            .enumerate()
            .map(|(c, x)| {
                x * thermo
                    .k_value(slate, c, Kelvin(t), pressure)
                    .expect("K at T > 0")
            })
            .sum()
    };
    let (mut low, mut high) = (200.0_f64, 1200.0_f64);
    assert!(
        sum_kx(low) < 1.0 && sum_kx(high) > 1.0,
        "the bracket must straddle the bubble point"
    );
    for _ in 0..200 {
        let mid = 0.5 * (low + high);
        if sum_kx(mid) < 1.0 {
            low = mid;
        } else {
            high = mid;
        }
    }
    0.5 * (low + high)
}

// ---------------------------------------------------------------------------
// The side draw, and determinism.
// ---------------------------------------------------------------------------

/// **The middle arm of the draw loop, on a wired plant.** Every cascade fixture
/// in `cascade_column.rs` is a two-draw column — a distillate at the condenser and
/// a bottoms at the reboiler — so `DrawLocation::Stage`, the arm that reads a
/// liquid draw off an interior tray, is reached there only through the bottoms.
/// This demo's middle product leaves stage 5 of 8.
///
/// What pins that it really is a tray draw rather than a relabelled bottoms: its
/// temperature is strictly between the other two and equals the bubble point of
/// the composition IT carries, recomputed here from the published `k_value`. An
/// off-by-one in the one-based-file to zero-based-profile translation leaves the
/// draw ordered and away from the feed and fails this.
#[test]
fn the_cascade_demos_middle_product_leaves_a_real_tray() {
    let engine = run(CASCADE, 200);
    let column = engine.graph.find_node("column").unwrap();
    let pressure = match &engine.graph.node(column).kind {
        NodeKind::Column {
            pressure, draws, ..
        } => {
            assert_eq!(draws.len(), 3, "the demo column has three draws");
            assert_eq!(draws[1].stage, Some(5), "the side draw is at stage 5");
            *pressure
        }
        other => panic!("expected a column, got {other:?}"),
    };

    for draw in DRAWS {
        let carried =
            refinery_core::components::Composition::from_weights(&fractions(&engine, draw))
                .expect("a draw carries a valid composition");
        let saturated = bubble_point_of(&engine.slate, &carried, pressure);
        approx::assert_relative_eq!(temperature(&engine, draw), saturated, max_relative = 1e-6);
    }
}

/// Determinism (I4) for the cascade demo — the counterpart of
/// `column_reference.rs`'s `the_demo_column_plant_reruns_bit_identically`, and it
/// covers strictly more: a cascade adds a fixed-point iteration whose stopping
/// point depends on a chain of bubble-point bisections and `nc` Thomas sweeps, so
/// "the answer is deterministic" is a claim about the solve rather than about the
/// data structures the splitter version pins.
///
/// **Caught nothing in M7.4c's mutation pass**, as a determinism gate should: a
/// mutation changes what the engine computes, and both runs here compute it the
/// same wrong way. The fault it exists for is a `HashMap` iteration or a
/// wall-clock read reaching the tick loop, which is a shape no numerical mutation
/// has. Said here rather than left to be rediscovered.
#[test]
fn the_cascade_demo_reruns_bit_identically() {
    let go = || -> Vec<Vec<u8>> {
        let mut engine = build(CASCADE);
        (0..50)
            .map(|tick| {
                engine
                    .tick()
                    .unwrap_or_else(|e| panic!("tick {tick} must converge: {e}"));
                serde_json::to_vec(&engine.snapshot()).expect("a snapshot must serialize")
            })
            .collect()
    };
    let first = go();
    let second = go();
    assert_eq!(first.len(), 50, "a run must capture one snapshot per tick");
    for (i, (a, b)) in first.iter().zip(&second).enumerate() {
        assert!(
            a == b,
            "the cascade demo diverged at tick {}:\n  first:  {}\n  second: {}",
            i + 1,
            String::from_utf8_lossy(a),
            String::from_utf8_lossy(b)
        );
    }
}
