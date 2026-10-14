//! Gates for the reactor screen's scripted timelines (M57.5, `demo/reactor.gd`,
//! docs/DESIGN.md §62.13).
//!
//! The scene is demonstrated, not gated: no `cargo test` can say a bed looks
//! like it is running away. What IS checkable is the story its `--auto` runs
//! tell. These tests send the scene's commands through the same [`Session`] the
//! gdext binding forwards to, at the same ticks as `AUTO_CALM`, `AUTO_TRIP`,
//! `AUTO_QUENCH` and `AUTO_WALL` (M58), and pin what each beat is there to
//! show. A plant edit that renames a node the scene resolves, or an engine
//! change that stops the calm bed staying hot after its feed comes back, fails
//! here instead of quietly changing the demo.
//!
//! **The command text is the scene's own**, byte for byte as its recorded runs
//! printed it (Godot's `JSON.stringify` sorts keys and writes `14000000.0`), so
//! the ids-must-be-integers cast `_do` makes is checked too.
//!
//! The plants are the shipped `hydrocracker_bed.toml`, `hydrocracker_trip.toml`,
//! `hydrocracker_quench.toml` and `hydrocracker_wall_failure.toml`; their
//! physics is gated in `crates/scenarios/tests/` (`hydrocracker_reference.rs`,
//! `bed_trip_reference.rs`, `quench_reference.rs`, `bed_wall_reference.rs`).
//! These tests own only the timelines.

use refinery_godot_ext::bridge::Session;
use serde_json::Value;

fn scenario_src(name: &str) -> String {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("reading {}: {e}", path.display()))
}

/// A loaded session plus the ids of the beds the scene draws.
struct Screen {
    session: Session,
    beds: Vec<usize>,
}

impl Screen {
    /// `nodes` is every node name the scene's `PLANTS` entry resolves, `beds`
    /// the beds among them, in the scene's order.
    fn load(scenario: &str, nodes: &[&str], beds: &[&str]) -> Self {
        let mut session = Session::default();
        assert_eq!(session.load_scenario(&scenario_src(scenario)), "null");
        // `_load` halts on any of these missing; resolve them the same way.
        for node in nodes.iter().chain(beds) {
            assert!(session.node_id(node) >= 0, "{scenario} has no node {node}");
        }
        let beds = beds.iter().map(|b| session.node_id(b) as usize).collect();
        Screen { session, beds }
    }

    /// Tick until `tick_index() == tick`, failing on any engine error — the
    /// halt the scene would show.
    fn run_to(&mut self, tick: i64) {
        while self.session.tick_index() < tick {
            self.tick();
        }
    }

    fn tick(&mut self) {
        assert_eq!(
            self.session.tick(),
            "null",
            "tick {}",
            self.session.tick_index() + 1
        );
    }

    /// Tick until the plant's first trip latches; the tick it latched on.
    fn run_to_trip(&mut self, limit: i64) -> i64 {
        while self.session.tick_index() < limit {
            self.tick();
            if self.trip_status() == "tripped" {
                return self.session.tick_index();
            }
        }
        panic!("no trip by tick {limit}");
    }

    fn snap(&self) -> Value {
        serde_json::from_str(&self.session.snapshot_json()).unwrap()
    }

    fn send(&mut self, json: &str) {
        assert_eq!(self.session.apply_command_json(json), "null", "{json}");
    }

    /// The engine's refusal message, as the scene prints it after `REFUSED:`.
    fn refused(&mut self, json: &str) -> String {
        let out: Value = serde_json::from_str(&self.session.apply_command_json(json)).unwrap();
        out["message"]
            .as_str()
            .unwrap_or_else(|| panic!("{json} was admitted"))
            .to_string()
    }

    /// Bed `i`'s temperature [°C], `kind.bed.temperature`.
    fn bed_c(&self, i: usize) -> f64 {
        self.snap()["nodes"][self.beds[i]]["kind"]["bed"]["temperature"]
            .as_f64()
            .unwrap()
            - 273.15
    }

    /// The oil entering bed `i` [°C]: the stream on the edge that ends at it.
    fn inlet_c(&self, i: usize) -> f64 {
        let snap = self.snap();
        let into = snap["edges"]
            .as_array()
            .unwrap()
            .iter()
            .find(|e| e["to"].as_u64() == Some(self.beds[i] as u64))
            .unwrap()
            .clone();
        into["stream"]["temperature"].as_f64().unwrap() - 273.15
    }

    /// The `vgo` share of bed `i`'s outflow — the scene's "uncracked".
    fn uncracked(&self, i: usize) -> f64 {
        let snap = self.snap();
        let out = snap["edges"]
            .as_array()
            .unwrap()
            .iter()
            .find(|e| e["from"].as_u64() == Some(self.beds[i] as u64))
            .unwrap()
            .clone();
        out["stream"]["composition"]["mass_fractions"][0]
            .as_f64()
            .unwrap()
    }

    /// Whether bed `i`'s wall has failed, `kind.wall.state` (M58).
    fn wall_failed(&self, i: usize) -> bool {
        self.snap()["nodes"][self.beds[i]]["kind"]["wall"]["state"]["status"] == "failed"
    }

    /// The fire bed `i`'s failed wall feeds [W], `wall_fire_w` (M58).
    fn wall_fire_w(&self, i: usize) -> f64 {
        self.snap()["nodes"][self.beds[i]]["wall_fire_w"]
            .as_f64()
            .unwrap()
    }

    /// Tick until bed `i`'s wall fails; the tick it failed on.
    fn run_to_failure(&mut self, i: usize, limit: i64) -> i64 {
        while self.session.tick_index() < limit {
            self.tick();
            if self.wall_failed(i) {
                return self.session.tick_index();
            }
        }
        panic!("no wall failure by tick {limit}");
    }

    fn trip_status(&self) -> String {
        self.snap()["trips"][0]["state"]["status"]
            .as_str()
            .unwrap()
            .to_string()
    }

    fn mode(&self, loop_id: usize) -> String {
        self.snap()["controls"][loop_id]["mode"]
            .as_str()
            .unwrap()
            .to_string()
    }

    fn duty_w(&self, node: usize) -> f64 {
        self.snap()["nodes"][node]["kind"]["duty"].as_f64().unwrap()
    }
}

/// A refused reset reads as a person reads it on screen: the bed's reading and
/// the limit in °C, not the engine's debug text (the fix M57.5 made).
fn assert_readable_refusal(message: &str) {
    assert!(message.contains("condition still holds"), "{message}");
    assert!(
        message.contains("the temperature is 40")
            && message.contains("at or above its limit of 400.0 °C"),
        "{message}"
    );
    assert!(!message.contains("Kelvin("), "{message}");
}

// The scene's command text, as its recorded runs printed it.
const FEED_385_C: &str = r#"{"cmd":"set_source_temperature","node":0,"temperature":658.15}"#;
const FEED_365_C: &str = r#"{"cmd":"set_source_temperature","node":0,"temperature":638.15}"#;
const FEED_220_C: &str = r#"{"cmd":"set_source_temperature","node":0,"temperature":493.15}"#;
const FEED_200_C: &str = r#"{"cmd":"set_source_temperature","node":0,"temperature":473.15}"#;
const PATCH: &str = r#"{"area":0.0,"cmd":"puncture_pipe","edge":2}"#;
const REPAIR: &str = r#"{"cmd":"repair_bed_wall","node":2}"#;
const RESET: &str = r#"{"cmd":"reset_trip","trip_id":0}"#;
const FIRE_14_MW: &str = r#"{"cmd":"set_furnace_duty","duty":14000000.0,"node":2}"#;
const BED_LOOP_MANUAL: &str = r#"{"cmd":"set_controller_mode","loop_id":0,"mode":"manual"}"#;
const QUENCH_MINIMUM: &str =
    r#"{"cmd":"set_setpoint","loop_id":1,"value":{"kg_per_s":0.25,"variable":"flow"}}"#;
const BED_LOOP_AUTO: &str = r#"{"cmd":"set_controller_mode","loop_id":0,"mode":"auto"}"#;
const PREHEAT_LOOP_AUTO: &str = r#"{"cmd":"set_controller_mode","loop_id":2,"mode":"auto"}"#;

/// `AUTO_CALM`: the feed warmed 20 K runs the bed away; brought back, the bed
/// stays hot; only a 220 °C feed brings it down, and then the calm day returns.
#[test]
fn the_calm_bed_timeline_runs_away_and_stays_hot() {
    let mut screen = Screen::load(
        "hydrocracker_bed.toml",
        &["vgo_feed", "feed_valve", "product"],
        &["bed"],
    );
    // The constants above carry an id the scene looks up by name at run time.
    assert_eq!(screen.snap()["nodes"][0]["name"], "vgo_feed");

    // The calm day: 9 K over a 365 °C feed.
    screen.run_to(600);
    assert!(
        (screen.bed_c(0) - 374.10).abs() < 0.01,
        "{}",
        screen.bed_c(0)
    );
    screen.send(FEED_385_C);

    // The creep, then the runaway: nothing uncracked by 21 minutes.
    screen.run_to(3600);
    assert!(
        (390.0..400.0).contains(&screen.bed_c(0)),
        "{}",
        screen.bed_c(0)
    );
    screen.run_to(12600);
    assert!(screen.bed_c(0) > 500.0, "{}", screen.bed_c(0));
    assert!(screen.uncracked(0) < 1e-3);

    // Since M58: at 550 °C the wall fails, 21 min 36 s in, and the leak burns.
    assert_eq!(screen.run_to_failure(0, 15000), 12964);
    screen.run_to(12970);
    assert!(screen.wall_fire_w(0) > 50e6, "{}", screen.wall_fire_w(0));

    // 25 min: the feed back to the calm day's 365 °C. Ten minutes on, the bed
    // is HOTTER, not cooler: the hot state holds on the calm day's feed.
    screen.run_to(15000);
    let back_at = screen.bed_c(0);
    assert!(back_at > 650.0, "{back_at}");
    screen.send(FEED_365_C);
    screen.run_to(21000);
    assert!(
        (screen.inlet_c(0) - 365.1).abs() < 0.05,
        "{}",
        screen.inlet_c(0)
    );
    assert!(
        screen.bed_c(0) > back_at && screen.bed_c(0) > 730.0,
        "{}",
        screen.bed_c(0)
    );

    // 35 min: the hole patched, which puts the fire out, and a 220 °C feed. It
    // holds above 550 °C ten minutes on, then falls back onto the cool state.
    screen.send(PATCH);
    screen.send(FEED_220_C);
    screen.run_to(27000);
    assert!(
        (screen.inlet_c(0) - 220.1).abs() < 0.05,
        "{}",
        screen.inlet_c(0)
    );
    assert_eq!(screen.wall_fire_w(0), 0.0, "a patched hole burns nothing");
    assert!(screen.bed_c(0) > 570.0, "{}", screen.bed_c(0));
    screen.run_to(33000);
    assert!(screen.bed_c(0) < 420.0, "{}", screen.bed_c(0));

    // 55 min: the wall repaired, the calm day's feed again, and the calm day's
    // bed.
    screen.send(REPAIR);
    screen.send(FEED_365_C);
    screen.run_to(33001);
    assert!(!screen.wall_failed(0));
    screen.run_to(48000);
    assert!(
        (374.0..375.5).contains(&screen.bed_c(0)),
        "{}",
        screen.bed_c(0)
    );
    assert!(screen.uncracked(0) > 0.93);
}

/// `AUTO_WALL` (M58): the bed loaded mid-creep fails its wall at 550 °C and the
/// leak burns; a repair is refused while the hole is open and again while the
/// bed is past its limit; patched and fed 200 °C oil, it is admitted at 20 min,
/// and back on the calm day's feed the bed settles on its calm state.
#[test]
fn the_wall_timeline_fails_patches_chills_and_repairs() {
    let mut screen = Screen::load(
        "hydrocracker_wall_failure.toml",
        &["vgo_feed", "feed_valve", "product"],
        &["bed"],
    );
    assert_eq!(screen.snap()["nodes"][0]["name"], "vgo_feed");
    assert_eq!(screen.snap()["nodes"][2]["name"], "bed");
    assert_eq!(screen.snap()["edges"][2]["name"], "product_line");

    // §63's figure: the wall fails at 4 min 35 s, and the leak burns.
    assert_eq!(screen.run_to_failure(0, 6000), 2750);
    screen.run_to(2760);
    assert!(
        (50e6..60e6).contains(&screen.wall_fire_w(0)),
        "{}",
        screen.wall_fire_w(0)
    );

    // 10 min: refused (the hole), patched, refused (the heat), chilled.
    screen.run_to(6000);
    let says = screen.refused(REPAIR);
    assert!(
        says.contains("still leaks") && says.contains("'product_line'"),
        "{says}"
    );
    screen.send(PATCH);
    let says = screen.refused(REPAIR);
    assert!(
        says.contains("at or past its wall's 550.0 °C limit") && says.contains("colder feed"),
        "{says}"
    );
    assert!(!says.contains("Kelvin("), "{says}");
    screen.send(FEED_200_C);

    // 20 min: under its limit, the repair is admitted.
    screen.run_to(12000);
    assert!(screen.bed_c(0) < 550.0, "{}", screen.bed_c(0));
    assert!(screen.wall_failed(0));
    screen.send(REPAIR);
    screen.run_to(12001);
    assert!(!screen.wall_failed(0));

    // 25 min: the calm day's feed, and by 55 min the calm day's bed.
    screen.run_to(15000);
    screen.send(FEED_365_C);
    screen.run_to(33000);
    assert!(
        (373.5..374.5).contains(&screen.bed_c(0)),
        "{}",
        screen.bed_c(0)
    );
    assert!(!screen.wall_failed(0));
}

/// `AUTO_TRIP`: the trip cuts the fire at 400 °C; a reset inside the condition
/// is refused in words; one after it relights nothing; relit lower by hand, the
/// bed settles under its limit.
#[test]
fn the_trip_timeline_cuts_the_fire_and_relights_by_hand() {
    let mut screen = Screen::load(
        "hydrocracker_trip.toml",
        &["vgo_feed", "feed_valve", "preheater", "product"],
        &["bed"],
    );
    assert_eq!(screen.snap()["nodes"][2]["name"], "preheater");
    assert_eq!(screen.snap()["trips"][0]["name"], "bed_high");

    // DESIGN §62.10's figure: the trip fires at 6 min 10 s.
    assert_eq!(screen.run_to_trip(6000), 3698);
    assert_eq!(screen.duty_w(2), 0.0);

    // 3800: the bed turned at about 400.4 °C and is still past its limit.
    screen.run_to(3800);
    assert!(screen.bed_c(0) >= 400.0, "{}", screen.bed_c(0));
    assert_readable_refusal(&screen.refused(RESET));

    // 7200: admitted, and the preheater stays dark.
    screen.run_to(7200);
    screen.send(RESET);
    screen.run_to(7201);
    assert_eq!(screen.trip_status(), "armed");
    assert_eq!(screen.duty_w(2), 0.0, "the reset relights nothing");

    // 7800: relit by hand at 14 MW; the bed settles near 386 °C, armed.
    screen.run_to(7800);
    screen.send(FIRE_14_MW);
    screen.run_to(36000);
    assert!(
        (385.7..386.0).contains(&screen.bed_c(0)),
        "{}",
        screen.bed_c(0)
    );
    assert_eq!(screen.trip_status(), "armed");
}

/// `AUTO_QUENCH`: the loop holds the second bed; taken over by hand with the
/// quench cut, the bed creeps into the trip; reset after it cools and handed
/// back, the loops relight the preheater and hold the bed at 380 °C again.
#[test]
fn the_quench_timeline_loses_the_bed_by_hand_and_wins_it_back() {
    let mut screen = Screen::load(
        "hydrocracker_quench.toml",
        &[
            "vgo_feed",
            "feed_valve",
            "preheater",
            "mix",
            "product_cooler",
            "product_drum",
            "product",
        ],
        &["bed1", "bed2"],
    );
    let snap = screen.snap();
    assert_eq!(snap["controls"][0]["name"], "bed2_temperature");
    assert_eq!(snap["controls"][1]["name"], "quench_flow");
    assert_eq!(snap["controls"][2]["name"], "preheat_outlet");
    assert!(screen.session.edge_id("quench_line") >= 0);

    // 1200: the cascade has caught the second bed.
    screen.run_to(1200);
    assert!((screen.bed_c(1) - 380.0).abs() < 0.5, "{}", screen.bed_c(1));
    assert_eq!(screen.mode(0), "auto");
    screen.send(BED_LOOP_MANUAL);
    screen.send(QUENCH_MINIMUM);

    // The creep into the trip, which cuts the preheater and puts its loop in
    // MANUAL; the bed loop stays where the person left it.
    let tripped_at = screen.run_to_trip(12000);
    assert_eq!(tripped_at, 8717);
    assert_eq!(screen.mode(2), "manual");
    assert_eq!(screen.mode(0), "manual");

    // 9000: still past its limit.
    screen.run_to(9000);
    assert_readable_refusal(&screen.refused(RESET));

    // 11000: reset, both loops handed back.
    screen.run_to(11000);
    screen.send(RESET);
    screen.send(BED_LOOP_AUTO);
    screen.send(PREHEAT_LOOP_AUTO);

    // 24000: the preheater relit under its loop, the bed held again, the quench
    // cold and the trip armed.
    screen.run_to(24000);
    assert!(screen.duty_w(2) > 1.0e7, "{}", screen.duty_w(2));
    assert!((screen.bed_c(1) - 380.0).abs() < 0.3, "{}", screen.bed_c(1));
    assert_eq!(screen.mode(0), "auto");
    assert_eq!(screen.mode(2), "auto");
    assert_eq!(screen.trip_status(), "armed");
    let quench = &screen.snap()["controls"][0]["quench"];
    assert!(quench["quench_k"].as_f64().unwrap() < quench["joins_k"].as_f64().unwrap());
}
