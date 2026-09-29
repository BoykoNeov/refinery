//! The game fidelity's stiff-pair stall and its fix (M21.1, docs/DESIGN.md §25,
//! ledger row A3).
//!
//! **The mechanism.** A vessel and the zero-volume node across a very conductive
//! line — a PSV's inlet, a vent valve's line — can only move TOGETHER in a
//! node-by-node Gauss–Seidel sweep, and the pair's common error decays by about
//! `c/(g + c)` per sweep: `g` the line's conductance, `c = C/dt` the vessel's
//! accumulation slope. Predicted 0.98850 against 0.988502 measured on
//! `relief_blowdown`. `g` is large from geometry on a line that flows, and from
//! the square root's regularisation on a dead end (a shut PSV) — and a dead end's
//! slope at zero flow still grows with the line's size. `dt` enters through `c`
//! alone, so a longer tick makes the pair slower in proportion.
//!
//! **The fix** is an additive correction in `SimpleFlowSolver`: after each sweep,
//! groups of unknowns are shifted by one common amount, a scalar Newton step on
//! the group's net imbalance, over a hierarchy of heavy-edge pairs. Newton is
//! untouched.
//!
//! Every gate here runs a plant under BOTH fidelities side by side and compares
//! them at every snapshot, not only the last: the claim is that the game solver
//! now finds Newton's answer on plants it used to stall on, not merely that it
//! returns `Ok`. The shapes are the ones one edit to a shipped file reaches, which
//! is how far away A3's trigger turned out to be:
//!
//! | gate | plant | before M21.1 (Simple) |
//! |---|---|---|
//! | 1 | `relief_blowdown`, inlet 5 m × 60 mm / 2 m × 100 mm / 1 m × 150 mm | 920 / fails tick 41 / fails tick 1 |
//! | 2 | `relief_blowdown` at `dt = 1.0` | fails tick 1 |
//! | 3 | `vessel_pressure_control`, vent line 2 m × 100 mm, `dt = 1.0` | fails tick 1 |
//! | 4 | `relief_twin_vessels`, and its 2 m × 100 mm `two_vessel` sibling | fails tick 1 / fails tick 47 |

use refinery_scenarios::ScenarioFile;

const RELIEF: &str = include_str!("../../../scenarios/relief_blowdown.toml");
const VENT: &str = include_str!("../../../scenarios/vessel_pressure_control.toml");
const TWIN: &str = include_str!("../../../scenarios/relief_twin_vessels.toml");

/// The agreement bound on node pressure between the fidelities, relative, at
/// every snapshot. The bound `relief_valve_reference.rs`'s old sweep gate used.
/// Measured worst across these gates: 4.72e-5, at the PSV node on tick 13 of the
/// `dt = 1.0` relief plant, inside the valve's accumulation band. The shipped
/// `dt = 0.1` relief plant already sits at 1.83e-6 before this change.
const PRESSURE_AGREEMENT: f64 = 1e-4;

fn load(src: &str) -> ScenarioFile {
    refinery_scenarios::load_str(src).expect("scenario parses")
}

/// Set one declared pipe's geometry on a loaded file.
fn resize(file: &mut ScenarioFile, pipe: &str, length_m: f64, diameter_m: f64) {
    let def = file
        .pipes
        .iter_mut()
        .find(|p| p.name == pipe)
        .unwrap_or_else(|| panic!("plant must declare a '{pipe}' pipe"));
    def.length_m = length_m;
    def.diameter_m = diameter_m;
}

/// What running one plant under both fidelities side by side gives: each one's
/// worst iteration count, and the worst relative disagreement between their node
/// pressures over EVERY snapshot, with where and when it happened.
struct BothFidelities {
    newton_worst: u32,
    simple_worst: u32,
    worst_pressure_disagreement: f64,
    at: String,
}

fn run_both(file: &mut ScenarioFile, ticks: usize) -> BothFidelities {
    let mut engine_under = |flow: &str| {
        file.fidelity.flow = flow.to_string();
        refinery_scenarios::build_engine(file).expect("builds")
    };
    let mut newton = engine_under("newton");
    let mut simple = engine_under("simple");
    let mut out = BothFidelities {
        newton_worst: 0,
        simple_worst: 0,
        worst_pressure_disagreement: 0.0,
        at: String::new(),
    };
    for tick in 1..=ticks {
        newton
            .tick()
            .unwrap_or_else(|e| panic!("newton tick {tick}: {e}"));
        simple
            .tick()
            .unwrap_or_else(|e| panic!("simple tick {tick}: {e}"));
        let (n, s) = (newton.snapshot(), simple.snapshot());
        out.newton_worst = out.newton_worst.max(n.solver.iterations);
        out.simple_worst = out.simple_worst.max(s.solver.iterations);
        for (a, b) in n.nodes.iter().zip(&s.nodes) {
            assert_eq!(a.id, b.id, "the two snapshots must list nodes in one order");
            let rel = (a.pressure_pa - b.pressure_pa).abs() / a.pressure_pa.abs();
            if rel > out.worst_pressure_disagreement {
                out.worst_pressure_disagreement = rel;
                out.at = format!("{} at tick {tick}", a.name);
            }
        }
    }
    out
}

fn assert_agrees(run: &BothFidelities, what: &str) {
    assert!(
        run.worst_pressure_disagreement < PRESSURE_AGREEMENT,
        "{what}: the fidelities disagree by {:.3e} on {}, against {PRESSURE_AGREEMENT:e}",
        run.worst_pressure_disagreement,
        run.at
    );
}

/// The game solver stays within a small multiple of Newton's own worst count on
/// the same plant. A ratio rather than a ceiling, because a bare ceiling is the
/// fitted margin `relief_valve_reference.rs`'s old `< 2500` gate was.
fn assert_near_newton(run: &BothFidelities, what: &str) {
    assert!(
        run.simple_worst < 5 * run.newton_worst,
        "{what}: the Simple fidelity took {} sweeps against Newton's {} — the group \
         correction is not removing the stiff pair's slow mode",
        run.simple_worst,
        run.newton_worst
    );
}

/// **Gate 1 — the count no longer tracks the stiffness.** This is the old `< 2500`
/// sweep gate from `relief_valve_reference.rs`, re-premised.
///
/// That gate existed because the first demo geometry, 2 m × 100 mm, stalled the
/// Simple fidelity at its cap, and the fix then was to re-size the line to
/// 5 m × 60 mm. It defended a margin on one geometry. The solver is fixed now, so
/// the gate runs three inlet lines spanning more than an order of magnitude of
/// `g/c` and asserts the claim itself: the sweep count is FLAT across them
/// (largest at most 3× the smallest), each is within 5× of Newton's, and every
/// snapshot agrees with Newton. Measured: 8, 8 and 7 sweeps against Newton's 10,
/// 8 and 8.
#[test]
fn the_simple_sweep_count_is_flat_in_the_relief_lines_stiffness() {
    let mut worsts = Vec::new();
    for (length_m, diameter_m) in [(5.0, 0.06), (2.0, 0.10), (1.0, 0.15)] {
        let mut file = load(RELIEF);
        resize(&mut file, "psv_inlet", length_m, diameter_m);
        let run = run_both(&mut file, 1500);
        let what = format!("relief inlet {length_m} m × {diameter_m} m");
        assert!(
            run.newton_worst < 25,
            "{what}: Newton must crack this plant easily; took {} iterations",
            run.newton_worst
        );
        assert_near_newton(&run, &what);
        assert_agrees(&run, &what);
        worsts.push(run.simple_worst);
    }
    let lo = *worsts.iter().min().expect("three runs");
    let hi = *worsts.iter().max().expect("three runs");
    assert!(
        hi <= 3 * lo,
        "the Simple sweep count must not track the inlet line's stiffness: {worsts:?} \
         across 5 m × 60 mm, 2 m × 100 mm and 1 m × 150 mm"
    );
}

/// **Gate 2 — the PSV's accumulation band at `dt = 1.0`.**
///
/// Ten times the tick makes `c` ten times smaller and the pair ten times slower;
/// before M21.1 the Simple fidelity failed here at tick 1. It is also where a
/// group step graded on FROZEN coefficients cycles: 20–21 bar is the PSV's band,
/// where its opening is a steep function of its own inlet pressure, and a frozen
/// compile holds that opening fixed while a 38 kPa group shift moves the inlet —
/// 20.180 ↔ 20.562 bar with period two from tick 14. That is why the group
/// trial recompiles its boundary edges, and this gate is what defends it.
#[test]
fn the_relief_band_at_a_one_second_tick_agrees_with_newton() {
    let mut file = load(RELIEF);
    file.simulation.dt = 1.0;
    let run = run_both(&mut file, 6000);
    assert_agrees(&run, "relief_blowdown at dt = 1.0");
}

/// **Gate 3 — the conducting shape.** A vent line that FLOWS, not a dead end:
/// `vessel_pressure_control` with M10.1's first-draft 2 m × 100 mm vent line at
/// `dt = 1.0`, which failed at tick 1. Here `g` comes from geometry (the line
/// carries 0.17–0.50 kg/s across a 13–67 Pa drop) rather than from the square
/// root's regularisation, so this says the fix is not specific to a shut valve.
#[test]
fn a_wide_conducting_vent_at_a_one_second_tick_agrees_with_newton() {
    let mut file = load(VENT);
    resize(&mut file, "vent_line", 2.0, 0.10);
    file.simulation.dt = 1.0;
    let run = run_both(&mut file, 6000);
    assert_agrees(&run, "vessel_pressure_control, 2 m × 100 mm vent, dt = 1.0");
}

/// **Gate 4 — two stiff pairs in one connected set.** Two slow modes, and one
/// common shift for the whole set removes only their sum, which is why the
/// groups are a hierarchy of pairs.
///
/// Two plants, in two tests (the second is
/// `two_vessel_separates_the_groupings_by_sweep_count`, so it runs even when this
/// one fails first), because they separate the two groupings in two different
/// ways:
/// - the shipped `relief_twin_vessels` (1 m × 150 mm relief lines): the old
///   solver and the one-shift-per-set shortcut both FAIL at tick 1, so CI's
///   corpus also defends this. Measured: 6 sweeps against Newton's 8.
/// - `two_vessel`, the same plant at 2 m × 100 mm: the old solver failed at tick
///   47, while the shortcut SURVIVES at 4 472 sweeps — under the cap, so only a
///   sweep-count assertion sees it. The hierarchy takes 8.
#[test]
fn two_relieving_vessels_in_one_plant_agree_with_newton() {
    let mut shipped = load(TWIN);
    let run = run_both(&mut shipped, 6000);
    assert_near_newton(&run, "relief_twin_vessels");
    assert_agrees(&run, "relief_twin_vessels");
}

/// Gate 4's second plant, in a test of its own so that it runs even when the
/// shipped twin fails first. Under one-group-per-set it must fail on the
/// sweep-count assertion (4 472 against Newton's 8), not on a divergence: this
/// is the plant that tells the two groupings apart by COST, where the shipped
/// twin tells them apart by failure.
#[test]
fn two_vessel_separates_the_groupings_by_sweep_count() {
    let mut two_vessel = load(TWIN);
    resize(&mut two_vessel, "psv1_inlet", 2.0, 0.10);
    resize(&mut two_vessel, "psv2_inlet", 2.0, 0.10);
    let run = run_both(&mut two_vessel, 6000);
    assert_near_newton(&run, "two_vessel (relief lines 2 m × 100 mm)");
    assert_agrees(&run, "two_vessel (relief lines 2 m × 100 mm)");
}

/// Measurement, not a gate: every number the gates above quote.
/// `cargo test --release -p refinery-scenarios --test stiff_pair_reference -- \
/// --ignored --nocapture`
#[test]
#[ignore = "measurement, not a gate"]
fn measure_the_stiff_pair_gates() {
    let report = |what: &str, run: BothFidelities| {
        println!(
            "{what:<52} simple worst {:>3}  newton worst {:>3}  pressure disagreement {:.3e} ({})",
            run.simple_worst, run.newton_worst, run.worst_pressure_disagreement, run.at
        );
    };
    for (length_m, diameter_m) in [(5.0, 0.06), (2.0, 0.10), (1.0, 0.15)] {
        let mut file = load(RELIEF);
        resize(&mut file, "psv_inlet", length_m, diameter_m);
        report(
            &format!("gate 1: relief inlet {length_m} m x {diameter_m} m, 1500 ticks"),
            run_both(&mut file, 1500),
        );
    }
    let mut file = load(RELIEF);
    file.simulation.dt = 1.0;
    report(
        "gate 2: relief_blowdown at dt = 1.0",
        run_both(&mut file, 6000),
    );
    let mut file = load(VENT);
    resize(&mut file, "vent_line", 2.0, 0.10);
    file.simulation.dt = 1.0;
    report(
        "gate 3: vent 2 m x 100 mm at dt = 1.0",
        run_both(&mut file, 6000),
    );
    report(
        "gate 4: relief_twin_vessels",
        run_both(&mut load(TWIN), 6000),
    );
    let mut file = load(TWIN);
    resize(&mut file, "psv1_inlet", 2.0, 0.10);
    resize(&mut file, "psv2_inlet", 2.0, 0.10);
    report(
        "gate 4: two_vessel (2 m x 100 mm)",
        run_both(&mut file, 6000),
    );
}
