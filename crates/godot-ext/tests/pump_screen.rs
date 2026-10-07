//! Gates for the pump screen's scripted timelines (M51, `demo/pump.gd`).
//!
//! The scene is demonstrated, not gated — no `cargo test` can say a pump looks
//! starved. What IS checkable is the story its `--auto` runs tell: these tests
//! send the scene's commands through the same [`Session`] the gdext binding
//! forwards to, at the same ticks as `AUTO_LIMIT` and `AUTO_BOILING`, and pin
//! what each beat is there to show. A plant edit that renames a node the scene
//! resolves, or a solver change that stops a throttle mid-run from giving the
//! pump its head back, fails here instead of quietly changing the demo.
//!
//! **The command text is the scene's own**, byte for byte as its recorded run
//! printed it (Godot's `JSON.stringify` sorts keys and writes `0.2`, `1.0`,
//! `false`), so the ids-must-be-integers cast `_do` makes is checked too.
//!
//! The plants are the shipped `pump_cavitation_flow_limit.toml` (M50) and
//! `cavitating_pump.toml` (M11); their physics is gated in
//! `crates/scenarios/tests/pump_cavitation_reference.rs` (DESIGN §55, §56).
//! These tests own only the timelines.

use refinery_godot_ext::bridge::Session;
use serde_json::Value;

fn scenario_src(name: &str) -> String {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("reading {}: {e}", path.display()))
}

/// A loaded session plus the ids the scene resolves at load.
struct Screen {
    session: Session,
    pump: usize,
    suction_pipe: usize,
}

impl Screen {
    /// `names` is the scene's `PLANTS` entry: source, pump, valve, destination,
    /// then the suction pipe.
    fn load(scenario: &str, names: [&str; 5]) -> Self {
        let mut session = Session::default();
        assert_eq!(session.load_scenario(&scenario_src(scenario)), "null");
        // `_load` halts on any of these missing; resolve them the same way.
        for node in &names[..4] {
            assert!(session.node_id(node) >= 0, "{scenario} has no node {node}");
        }
        assert!(
            session.edge_id(names[4]) >= 0,
            "{scenario} has no pipe {}",
            names[4]
        );
        let pump = session.node_id(names[1]) as usize;
        let suction_pipe = session.edge_id(names[4]) as usize;
        Screen {
            session,
            pump,
            suction_pipe,
        }
    }

    /// Tick until `tick_index() == tick`, failing on any engine error — the
    /// halt the scene would show.
    fn run_to(&mut self, tick: i64) {
        while self.session.tick_index() < tick {
            assert_eq!(
                self.session.tick(),
                "null",
                "tick {}",
                self.session.tick_index() + 1
            );
        }
    }

    fn snap(&self) -> Value {
        serde_json::from_str(&self.session.snapshot_json()).unwrap()
    }

    fn send(&mut self, json: &str) {
        assert_eq!(self.session.apply_command_json(json), "null", "{json}");
    }

    fn pump(&self) -> Value {
        self.snap()["nodes"][self.pump].clone()
    }

    fn flow(&self) -> f64 {
        self.snap()["edges"][self.suction_pipe]["stream"]["mass_flow"]
            .as_f64()
            .unwrap()
    }

    /// `pump_suction.head_fraction`, or `None` where the snapshot leaves the
    /// report out — what the scene draws as `--`.
    fn push(&self) -> Option<f64> {
        self.pump()
            .get("pump_suction")
            .map(|s| s["head_fraction"].as_f64().unwrap())
    }

    fn lamp(&self) -> bool {
        self.pump()["cavitation"]["cavitating"].as_bool().unwrap()
    }
}

// The scene's command text, as its recorded runs printed it.
const LIMIT_VALVE: [(f64, &str); 6] = [
    (0.2, r#"{"cmd":"set_valve_opening","node":2,"opening":0.2}"#),
    (0.3, r#"{"cmd":"set_valve_opening","node":2,"opening":0.3}"#),
    (0.4, r#"{"cmd":"set_valve_opening","node":2,"opening":0.4}"#),
    (0.6, r#"{"cmd":"set_valve_opening","node":2,"opening":0.6}"#),
    (0.8, r#"{"cmd":"set_valve_opening","node":2,"opening":0.8}"#),
    (1.0, r#"{"cmd":"set_valve_opening","node":2,"opening":1.0}"#),
];
const LIMIT_STOP: &str = r#"{"cmd":"set_pump_on","node":1,"on":false}"#;
const LIMIT_START: &str = r#"{"cmd":"set_pump_on","node":1,"on":true}"#;
const BOILING_VALVE: [(f64, &str); 5] = [
    (0.2, r#"{"cmd":"set_valve_opening","node":3,"opening":0.2}"#),
    (0.4, r#"{"cmd":"set_valve_opening","node":3,"opening":0.4}"#),
    (0.6, r#"{"cmd":"set_valve_opening","node":3,"opening":0.6}"#),
    (0.8, r#"{"cmd":"set_valve_opening","node":3,"opening":0.8}"#),
    (1.0, r#"{"cmd":"set_valve_opening","node":3,"opening":1.0}"#),
];
const BOILING_STOP: &str = r#"{"cmd":"set_pump_on","node":2,"on":false}"#;
const BOILING_START: &str = r#"{"cmd":"set_pump_on","node":2,"on":true}"#;

fn valve(table: &[(f64, &'static str)], opening: f64) -> &'static str {
    table
        .iter()
        .find(|(o, _)| *o == opening)
        .map(|(_, json)| *json)
        .unwrap()
}

/// `AUTO_LIMIT`: the pump at a quarter of its head on the file's opening, its
/// whole head back on a throttle, the flow levelling off while the head goes
/// as the valve opens, the throttle again from wide open, and a stop that
/// shows how little the pump was adding.
#[test]
fn the_limit_plant_timeline_tells_its_story() {
    let mut screen = Screen::load(
        "pump_cavitation_flow_limit.toml",
        [
            "rundown_source",
            "feed_pump",
            "discharge_valve",
            "unit_feed",
            "suction_line",
        ],
    );
    // The constants above carry ids the scene looks up by name at run time; a
    // plant edit that moves one should fail here, not as a puzzling refusal.
    let snap = screen.snap();
    assert_eq!(snap["nodes"][1]["name"], "feed_pump");
    assert_eq!(snap["nodes"][2]["name"], "discharge_valve");

    // Tick 1 runs the whole curve and reports no push (§55 fork 4); tick 2 has
    // the liquid's bubble pressure and does.
    screen.run_to(1);
    assert!(screen.push().is_none());
    screen.run_to(20);
    let push = screen.push().unwrap();
    assert!(
        (0.25..0.27).contains(&push),
        "push {push} on the file's 0.6"
    );
    assert!((screen.flow() - 10.96).abs() < 0.01, "{}", screen.flow());
    // The lamp reads "not boiling" while the pump loses three quarters of its
    // head: the bulk liquid is above its bubble pressure, the eye is not.
    assert!(!screen.lamp());

    // 20: throttle to 0.2 on a RUNNING pump — the move Newton gave up on
    // before M51 (ledger row A23). The whole head comes back.
    screen.send(valve(&LIMIT_VALVE, 0.2));
    screen.run_to(39);
    assert!(screen.push().unwrap() > 0.999);
    assert!((screen.flow() - 6.839).abs() < 0.001, "{}", screen.flow());

    // 40..120: open a step at a time. The flow rises; the head falls.
    let mut readings = vec![(0.2, screen.flow(), screen.push().unwrap())];
    for (tick, opening) in [(40, 0.3), (60, 0.4), (80, 0.6), (100, 0.8), (120, 1.0)] {
        screen.send(valve(&LIMIT_VALVE, opening));
        screen.run_to(tick + 19);
        readings.push((opening, screen.flow(), screen.push().unwrap()));
    }
    for pair in readings.windows(2) {
        let ((a, flow_a, push_a), (b, flow_b, push_b)) = (pair[0], pair[1]);
        assert!(flow_b > flow_a, "{a} -> {b}: flow {flow_a} -> {flow_b}");
        assert!(push_b < push_a, "{a} -> {b}: push {push_a} -> {push_b}");
    }
    // The flat end: from 0.4 to wide open buys under 1 kg/s while the pump
    // goes from 61% of its head to under 6%.
    let (at_04, at_10) = (readings[2], readings[5]);
    assert!(at_10.1 - at_04.1 < 1.0, "{at_04:?} -> {at_10:?}");
    assert!(at_04.2 > 0.6 && at_10.2 < 0.06, "{at_04:?} -> {at_10:?}");

    // 140: throttle from wide open — the head comes back again.
    screen.send(valve(&LIMIT_VALVE, 0.2));
    screen.run_to(159);
    assert!(screen.push().unwrap() > 0.999);
    assert!((screen.flow() - 6.839).abs() < 0.001, "{}", screen.flow());

    // 160: back to 0.6 and stop the pump. The supply alone still drives
    // 8.18 kg/s through it, and a stopped pump reports no push.
    screen.send(valve(&LIMIT_VALVE, 0.6));
    screen.send(LIMIT_STOP);
    screen.run_to(179);
    assert!(screen.push().is_none());
    assert!((screen.flow() - 8.18).abs() < 0.01, "{}", screen.flow());

    // 180: restart. The pump is back where it started.
    screen.send(LIMIT_START);
    screen.run_to(199);
    let push = screen.push().unwrap();
    assert!(
        (0.25..0.27).contains(&push),
        "push {push} after the restart"
    );
    assert!((screen.flow() - 10.96).abs() < 0.01, "{}", screen.flow());
}

/// `AUTO_BOILING`: M11's pump boils all run and nothing happens to it. The
/// flow follows the valve all the way; stopped, the line runs backwards.
#[test]
fn the_boiling_plant_timeline_tells_its_story() {
    let mut screen = Screen::load(
        "cavitating_pump.toml",
        [
            "rundown_source",
            "suction",
            "discharge_valve",
            "unit_feed",
            "lift_line",
        ],
    );
    let snap = screen.snap();
    assert_eq!(snap["nodes"][2]["name"], "suction");
    assert_eq!(snap["nodes"][3]["name"], "discharge_valve");

    screen.run_to(19);
    let at_06 = screen.flow();
    assert!(screen.lamp());
    assert!(screen.push().is_none(), "no suction model, no push report");

    // 20..80: the flow follows the valve — about in proportion, unlike the
    // M50 pump's flat end — with the lamp lit throughout.
    let mut flows = vec![];
    for (tick, opening) in [(20, 0.2), (40, 0.4), (60, 0.8), (80, 1.0)] {
        screen.send(valve(&BOILING_VALVE, opening));
        screen.run_to(tick + 19);
        assert!(screen.lamp(), "{opening}: the lamp stays lit");
        assert!(screen.push().is_none());
        flows.push(screen.flow());
    }
    assert!(flows.windows(2).all(|w| w[1] > w[0]), "{flows:?}");
    assert!(
        flows[3] > 1.5 * at_06,
        "wide open {} against {at_06}",
        flows[3]
    );

    // 100: stop it. The destination drives the line backwards.
    screen.send(BOILING_STOP);
    screen.run_to(119);
    assert!(screen.flow() < 0.0, "{}", screen.flow());

    // 120: restart on the file's opening — back to where it began.
    screen.send(BOILING_START);
    screen.send(valve(&BOILING_VALVE, 0.6));
    screen.run_to(139);
    assert!((screen.flow() - at_06).abs() < 1e-6 * at_06);
}
