//! M57.2: a catalyst bed with hydrocracking chemistry finds its own steady
//! states — the cool one, the hot one, the edge between them, and the history
//! that picks which (docs/DESIGN.md §62.4, §62.9).
//!
//! The hand calculation is the bed's steady first law with one heat capacity on
//! every lump (the shipped slate's): `c̄p·(T − T_in) = q(T, τ)`, `q` the heat a
//! kilogram of feed releases cracking at the bed's temperature over `τ`. `q` is
//! the chemistry's own `react` — its exactness is gated in
//! `solvers/tests/reference/hydrocracking.rs` — so what is checked here is the
//! bed: that the engine's long runs land on the roots of that balance, found by
//! bisection, and on the RIGHT root for the bed's history.
//!
//! `T_in` and `τ` are read off the snapshot: the feed is warmed a tenth of a
//! kelvin by friction on its way in, and `τ` is the solved flow's.
//!
//! Every run but the shipped files' uses the calm file with its bed made 60
//! times lighter (0.5 MJ/K, `C/W` ≈ 3 s), so a debug build reaches each steady
//! state in seconds. The heat capacity sets how fast a bed gets somewhere, not
//! where it can rest.

use refinery_core::engine::Engine;
use refinery_core::graph::{NodeId, NodeKind};
use refinery_core::snapshot::{Command, Snapshot};
use refinery_core::traits::ReactionModel;
use refinery_core::units::{Kelvin, Seconds};
use refinery_solvers::Hydrocracking;

const CALM: &str = include_str!("../../../scenarios/hydrocracker_bed.toml");
const RUNAWAY: &str = include_str!("../../../scenarios/hydrocracker_runaway.toml");

/// Every lump's heat capacity in the shipped slate [J/(kg·K)].
const CP: f64 = 3200.0;
const KELVIN: f64 = 273.15;

/// The calm file with a light bed loaded at `bed_c` [°C] and fed at `feed_c`.
fn fixture(bed_c: f64, feed_c: f64) -> String {
    let src = CALM
        .replace(
            "bed_heat_capacity_mj_per_k = 30.0",
            "bed_heat_capacity_mj_per_k = 0.5",
        )
        .replace(
            "bed_temperature_c = 374.1",
            &format!("bed_temperature_c = {bed_c}"),
        )
        .replace(
            "temperature_c = 365.0",
            &format!("temperature_c = {feed_c}"),
        );
    assert!(src.contains("= 0.5\n") && src.contains(&format!("bed_temperature_c = {bed_c}")));
    src
}

fn build(src: &str) -> Engine {
    let file = refinery_scenarios::load_str(src).expect("the hydrocracker plant must load");
    refinery_scenarios::build_engine(&file).expect("the hydrocracker plant must build")
}

fn run(engine: &mut Engine, ticks: u32) {
    for _ in 0..ticks {
        engine.tick().expect("the hydrocracker plant must run");
    }
}

fn node_id(snap: &Snapshot, name: &str) -> NodeId {
    snap.nodes
        .iter()
        .find(|n| n.name == name)
        .unwrap_or_else(|| panic!("the plant defines '{name}'"))
        .id
}

/// The bed's temperature [K].
fn bed(snap: &Snapshot) -> f64 {
    let node = snap.nodes.iter().find(|n| n.name == "bed").expect("bed");
    match &node.kind {
        NodeKind::BedReactor { bed } => bed.temperature.value(),
        other => panic!("'bed' must be a bed reactor, got {other:?}"),
    }
}

/// What the bed sees: its inlet temperature [K] and residence time [s].
fn bed_inputs(snap: &Snapshot) -> (f64, f64) {
    let inlet = snap
        .edges
        .iter()
        .find(|e| e.name == "valve_line")
        .expect("valve_line")
        .stream
        .temperature
        .value();
    let node = snap.nodes.iter().find(|n| n.name == "bed").expect("bed");
    let tau = node.residence_time_s.expect("a flowing bed reports its τ");
    (inlet, tau)
}

/// The chemistry on the shipped slate.
struct Balance {
    model: Hydrocracking,
    slate: refinery_core::components::Slate,
    feed: refinery_core::components::Composition,
}

impl Balance {
    /// The shipped slate's four lumps, in its order. The chemistry reads their
    /// names only; the balance's one heat capacity is [`CP`].
    fn new() -> Self {
        use refinery_core::components::{Composition, Phase, PseudoComponent, Slate};
        use refinery_core::units::{JPerKgK, KgPerM3, KgPerMol};
        let slate = Slate::new(
            ["vgo", "distillate", "naphtha", "gas"]
                .iter()
                .map(|name| PseudoComponent {
                    name: (*name).into(),
                    tb: Kelvin(500.0),
                    molar_mass: KgPerMol(0.2),
                    density: Some(KgPerM3(800.0)),
                    phase: Phase::Liquid,
                    cp: JPerKgK(CP),
                    cp_shape: None,
                })
                .collect(),
        )
        .expect("slate");
        let model = Hydrocracking::song_2017(&slate).expect("model");
        let feed = Composition::pure(slate.len(), 0);
        Self { model, slate, feed }
    }

    /// `q(T, τ)` [J/kg of feed]: what cracking at `T` releases.
    fn released(&self, t: f64, tau: f64) -> f64 {
        -self
            .model
            .react(&self.feed, Kelvin(t), Seconds(tau), &self.slate)
            .expect("react")
            .dh_rxn
            .value()
    }

    /// `T − q(T)/c̄p`: the feed temperature at which `T` is a steady bed.
    fn feed_for(&self, t: f64, tau: f64) -> f64 {
        t - self.released(t, tau) / CP
    }

    /// The steady bed in `[lo, hi]` for a feed at `inlet`, by bisection on
    /// `c̄p·(T − T_in) − q(T)`, which must change sign across the bracket.
    fn root(&self, inlet: f64, tau: f64, mut lo: f64, mut hi: f64) -> f64 {
        let g = |t: f64| self.feed_for(t, tau) - inlet;
        assert!(
            g(lo) * g(hi) < 0.0,
            "no steady bed in [{lo}, {hi}] K for a {inlet} K feed"
        );
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if g(mid) * g(lo) > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }

    /// The hottest feed with a cool steady bed [K], and the bed temperature it
    /// gives: the local maximum of `T − q(T)/c̄p` below the middle branch, by a
    /// scan at 1 mK.
    fn ignition(&self, tau: f64) -> (f64, f64) {
        let (mut feed, mut at) = (f64::NEG_INFINITY, 0.0);
        let mut t = 620.0;
        while t < 720.0 {
            let f = self.feed_for(t, tau);
            if f > feed {
                (feed, at) = (f, t);
            }
            t += 1e-3;
        }
        (feed, at)
    }

    fn ignition_feed(&self, tau: f64) -> f64 {
        self.ignition(tau).0
    }

    /// The cool steady bed: between the feed and where the cool branch ends.
    fn cool(&self, inlet: f64, tau: f64) -> f64 {
        self.root(inlet, tau, inlet - 1.0, self.ignition(tau).1)
    }

    /// The hot steady bed: past the middle branch (850 K) and below the full
    /// 375 K rise over any feed used here.
    fn hot(&self, inlet: f64, tau: f64) -> f64 {
        self.root(inlet, tau, 850.0, 1100.0)
    }

    fn steady(&self, inlet: f64, tau: f64, hot: bool) -> f64 {
        if hot {
            self.hot(inlet, tau)
        } else {
            self.cool(inlet, tau)
        }
    }
}

#[test]
fn a_calm_bed_settles_on_the_cool_root_of_its_balance() {
    // Loaded at the feed's temperature, the bed warms onto its cool root.
    let mut engine = build(&fixture(365.0, 365.0));
    run(&mut engine, 2_000);
    let snap = engine.snapshot();
    let (inlet, tau) = bed_inputs(&snap);
    let want = Balance::new().cool(inlet, tau);
    assert!(
        (bed(&snap) - want).abs() < 1e-6,
        "the bed is at {} K; the balance's cool root is {want} K",
        bed(&snap)
    );
    // §62.9: about 9 K above its feed.
    assert!((8.0..10.0).contains(&(want - inlet)), "{} K", want - inlet);
}

#[test]
fn a_hot_bed_settles_on_the_hot_root_of_the_same_balance() {
    // The same feed, the bed loaded hot: history picks the other branch.
    let mut engine = build(&fixture(740.0, 365.0));
    run(&mut engine, 2_000);
    let snap = engine.snapshot();
    let (inlet, tau) = bed_inputs(&snap);
    let want = Balance::new().hot(inlet, tau);
    assert!(
        (bed(&snap) - want).abs() < 1e-6,
        "the bed is at {} K; the balance's hot root is {want} K",
        bed(&snap)
    );
    // Nearly everything cracked to gas: close to the 375 K full rise.
    assert!(
        (370.0..=375.0).contains(&(want - inlet)),
        "{} K",
        want - inlet
    );
}

/// The cool branch ends at a feed temperature the balance names; a feed 1.5 K
/// below it keeps the bed cool, 1.5 K above it runs the bed away.
#[test]
fn the_bed_ignites_where_the_cool_branch_ends() {
    let balance = Balance::new();
    let mut calm = build(&fixture(374.1, 365.0));
    run(&mut calm, 10);
    let snap = calm.snapshot();
    let (inlet, tau) = bed_inputs(&snap);
    // The friction heat between the source and the bed, to aim the source.
    let warming = inlet - (365.0 + KELVIN);
    let ignition = balance.ignition_feed(tau);
    // §62.9: about 10 K above the calm day's feed.
    assert!(
        (9.5..11.0).contains(&(ignition - inlet)),
        "the cool branch ends {} K above the calm feed",
        ignition - inlet
    );

    for (offset, runs_away) in [(-1.5, false), (1.5, true)] {
        let mut engine = build(&fixture(374.1, 365.0));
        let source = node_id(&engine.snapshot(), "vgo_feed");
        engine
            .apply(Command::SetSourceTemperature {
                node: source,
                temperature: Kelvin(ignition + offset - warming),
            })
            .expect("the source takes a new temperature");
        run(&mut engine, 6_000);
        let snap = engine.snapshot();
        let (inlet, tau) = bed_inputs(&snap);
        let want = balance.steady(inlet, tau, runs_away);
        assert!(
            (bed(&snap) - want).abs() < 1e-4,
            "feed {offset:+} K from ignition: the bed is at {} K, the {} root is {want} K",
            bed(&snap),
            if runs_away { "hot" } else { "cool" }
        );
    }
}

/// Hysteresis: a bed that ran away stays hot when the feed is brought back
/// below where it ignited — down to 330 °C here, 45 K under the ignition feed.
#[test]
fn a_hot_bed_stays_hot_on_a_feed_that_could_not_have_lit_it() {
    let balance = Balance::new();
    let mut engine = build(&fixture(740.0, 330.0));
    run(&mut engine, 2_000);
    let snap = engine.snapshot();
    let (inlet, tau) = bed_inputs(&snap);
    assert!(inlet < balance.ignition_feed(tau) - 40.0);
    let want = balance.hot(inlet, tau);
    assert!(
        (bed(&snap) - want).abs() < 1e-6,
        "the bed is at {} K; the hot root on a 330 °C feed is {want} K",
        bed(&snap)
    );
    // And the same feed has a cool root a fresh bed would sit on.
    let cool = balance.cool(inlet, tau);
    assert!(cool < inlet + 5.0, "{cool} K");
}

/// The feed-cut runaway: halving the flow doubles `τ` (§62.2), so each
/// kilogram releases more heat while half the flow carries it away. A cut to
/// about 80% holds on a new, warmer cool root; a cut to about half runs away.
#[test]
fn cutting_the_feed_runs_the_calm_bed_away() {
    let balance = Balance::new();
    for (opening, runs_away) in [(0.8, false), (0.5, true)] {
        let mut engine = build(&fixture(374.1, 365.0));
        let valve = node_id(&engine.snapshot(), "feed_valve");
        run(&mut engine, 1); // τ is reported once the sweep has run
        let tau_before = bed_inputs(&engine.snapshot()).1;
        engine
            .apply(Command::SetValveOpening {
                node: valve,
                opening,
            })
            .expect("the valve takes a new opening");
        run(&mut engine, 6_000);
        let snap = engine.snapshot();
        let (inlet, tau) = bed_inputs(&snap);
        assert!(tau > 1.2 * tau_before, "τ must follow the cut: {tau} s");
        let want = balance.steady(inlet, tau, runs_away);
        assert!(
            (bed(&snap) - want).abs() < 1e-4,
            "opening {opening}: the bed is at {} K, the {} root is {want} K",
            bed(&snap),
            if runs_away { "hot" } else { "cool" }
        );
    }
}

/// The shipped calm file starts on its cool root and stays there.
#[test]
fn the_calm_file_holds_its_cool_root() {
    let mut engine = build(CALM);
    run(&mut engine, 6_000);
    let snap = engine.snapshot();
    let (inlet, tau) = bed_inputs(&snap);
    let want = Balance::new().cool(inlet, tau);
    assert!(
        (bed(&snap) - want).abs() < 0.01,
        "the calm file's bed is at {} K; its cool root is {want} K",
        bed(&snap)
    );
}

/// The shipped runaway file runs away by itself: no cool root exists at its
/// feed, the bed creeps for minutes and then climbs, as its header says.
#[test]
fn the_runaway_file_runs_away_by_itself() {
    let mut engine = build(RUNAWAY);
    let balance = Balance::new();
    run(&mut engine, 1);
    let (inlet, tau) = bed_inputs(&engine.snapshot());
    assert!(
        inlet > balance.ignition_feed(tau) + 5.0,
        "the runaway file's feed must be past the edge"
    );
    let mut minutes = Vec::new();
    for _ in 0..30 {
        run(&mut engine, 600);
        minutes.push(bed(&engine.snapshot()) - KELVIN);
    }
    // The header's claims: about 400 °C at 6 min and 450 °C at 16, past 600 °C
    // at 22 and 700 °C at 25, settling near 760 °C (minute n is index n − 1).
    assert!(
        (395.0..405.0).contains(&minutes[5]),
        "6 min: {}",
        minutes[5]
    );
    assert!(minutes[14] < 450.0 && minutes[16] > 450.0, "{minutes:?}");
    assert!(minutes[21] > 600.0 && minutes[24] > 700.0, "{minutes:?}");
    assert!(
        (740.0..761.0).contains(&minutes[29]),
        "30 min: {}",
        minutes[29]
    );
    assert!(
        minutes.windows(2).all(|w| w[1] > w[0]),
        "it only climbs: {minutes:?}"
    );
}

/// The held reactor admits the hydrocracking chemistry too (it is exact at any
/// `τ`): the products at its set temperature and fixed `tau_s` are the model's.
#[test]
fn a_held_reactor_runs_hydrocracking_at_its_set_temperature() {
    let src = CALM.replace(
        "type = \"bed_reactor\"\nbed_heat_capacity_mj_per_k = 30.0\ncatalyst_volume_m3 = 20.0\nbed_temperature_c = 374.1",
        "type = \"reactor\"\nt_set_c = 390.0\ntau_s = 3600.0",
    );
    assert!(src.contains("t_set_c"));
    let mut engine = build(&src);
    run(&mut engine, 5);
    let snap = engine.snapshot();
    let product = snap
        .edges
        .iter()
        .find(|e| e.name == "product_line")
        .expect("product_line");
    let balance = Balance::new();
    let want = balance
        .model
        .react(
            &balance.feed,
            Kelvin(663.15),
            Seconds(3600.0),
            &balance.slate,
        )
        .expect("react")
        .products;
    for (got, want) in product
        .stream
        .composition
        .fractions()
        .iter()
        .zip(want.fractions())
    {
        assert!((got - want).abs() < 1e-12, "{got} vs {want}");
    }
    assert!(product.stream.temperature.value() > 663.0);
}
