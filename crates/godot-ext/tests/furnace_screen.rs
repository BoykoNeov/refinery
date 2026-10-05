//! Gates for the furnace screen's scripted timelines (M39, `demo/furnace.gd`).
//!
//! The scene itself is demonstrated, not gated — no `cargo test` can say a coil
//! looks hot. What IS checkable is the story its `--auto` runs tell: these tests
//! send the scene's commands through the same [`Session`] the gdext binding
//! forwards to, at the same ticks as `AUTO_TRIP` and `AUTO_BURNOUT`, and pin
//! what each one is there to show. A change that moves a trip by a tick, makes a
//! refusal an acceptance, or renames a node the scene resolves fails here instead
//! of quietly turning the demo into a different story.
//!
//! **The command text is the scene's own**, byte for byte as its recorded run
//! printed it (Godot's `JSON.stringify` sorts keys and writes `0.0` for a float),
//! so the ids-must-be-integers cast `_do` makes is checked too: a float id would
//! be refused by serde before it reached the engine.
//!
//! The plants are the shipped `furnace_coil_trip.toml` and `furnace_burnout.toml`;
//! their physics is gated in `crates/scenarios/tests` (DESIGN §39, §42). These
//! tests own only the timelines.

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
    heater: usize,
}

impl Screen {
    fn load(scenario: &str, feed: &str, destination: &str) -> Self {
        let mut session = Session::default();
        assert_eq!(session.load_scenario(&scenario_src(scenario)), "null");
        // `_load` halts on any of these missing; resolve them the same way.
        for node in ["heater", feed, destination] {
            assert!(session.node_id(node) >= 0, "{scenario} has no node {node}");
        }
        assert!(
            session.edge_id("heated_line") >= 0,
            "{scenario} has no heated_line"
        );
        let heater = session.node_id("heater") as usize;
        Screen { session, heater }
    }

    /// Tick until `tick_index() == tick`, failing on any engine error.
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

    fn send(&mut self, json: &str) -> String {
        self.session.apply_command_json(json)
    }

    fn heater(&self) -> Value {
        self.snap()["nodes"][self.heater].clone()
    }

    fn duty_w(&self) -> f64 {
        self.heater()["kind"]["duty"].as_f64().unwrap()
    }

    fn coil_k(&self) -> f64 {
        self.heater()["kind"]["coil"]["temperature"]
            .as_f64()
            .unwrap()
    }

    fn trip(&self, index: usize) -> Value {
        self.snap()["trips"][index]["state"].clone()
    }

    fn loop_mode(&self) -> String {
        self.snap()["controls"][0]["mode"]
            .as_str()
            .unwrap()
            .to_owned()
    }
}

fn tripped(at_tick: u64) -> Value {
    serde_json::json!({"status": "tripped", "at_tick": at_tick})
}

fn tripped_by_hand(at_tick: u64) -> Value {
    serde_json::json!({"status": "tripped", "at_tick": at_tick, "by_hand": true})
}

fn armed() -> Value {
    serde_json::json!({"status": "armed"})
}

// The scene's command text, as its recorded run printed it.
const RESET_SKIN: &str = r#"{"cmd":"reset_trip","trip_id":0}"#;
const RESET_OUTLET: &str = r#"{"cmd":"reset_trip","trip_id":1}"#;
const PRESS_SKIN: &str = r#"{"cmd":"manual_trip","trip_id":0}"#;
const PRESS_OUTLET: &str = r#"{"cmd":"manual_trip","trip_id":1}"#;
const AUTO: &str = r#"{"cmd":"set_controller_mode","loop_id":0,"mode":"auto"}"#;
const SETPOINT_50: &str =
    r#"{"cmd":"set_setpoint","loop_id":0,"value":{"k":323.15,"variable":"temperature"}}"#;
const PATCH: &str = r#"{"area":0.0,"cmd":"puncture_pipe","edge":2}"#;
const DUTY_HALF_MW: &str = r#"{"cmd":"set_furnace_duty","duty":500000.0,"node":2}"#;
const NEW_TUBES: &str = r#"{"cmd":"replace_tubes","node":2}"#;

/// `AUTO_TRIP`: the trip fires by itself, fires again on the same target, holds
/// on a lower one, and the button cuts a healthy plant that a reset does not
/// relight.
#[test]
fn the_trip_plant_timeline_tells_its_story() {
    let mut screen = Screen::load("furnace_coil_trip.toml", "cool_feed", "hold_tank");
    // The constants above carry ids the scene looks up by name at run time; a
    // plant edit that moves one should fail here, not as a puzzling refusal.
    let snap = screen.snap();
    assert_eq!(snap["trips"][0]["name"], "tube_skin_high");
    assert_eq!(snap["trips"][1]["name"], "outlet_high");
    assert_eq!(snap["controls"][0]["name"], "outlet_temperature");
    // The scene finds its loop by this, the heater's outlet (M41).
    let watched = snap["controls"][0]["watches"]["node"].as_u64().unwrap() as usize;
    assert_eq!(snap["nodes"][watched]["name"], "heater");

    // By itself, at tick 75, with the outlet trip still armed.
    screen.run_to(74);
    assert_eq!(screen.trip(0), armed());
    screen.run_to(75);
    assert_eq!(screen.trip(0), tripped(75));
    assert_eq!(screen.trip(1), armed());
    assert_eq!(screen.duty_w(), 0.0);
    assert_eq!(screen.loop_mode(), "manual");

    // 150: reset, AUTO on the same 60 °C target — the fouled coil trips again.
    screen.run_to(150);
    assert_eq!(screen.send(RESET_SKIN), "null");
    assert_eq!(screen.send(AUTO), "null");
    screen.run_to(237);
    assert_eq!(screen.trip(0), armed());
    screen.run_to(238);
    assert_eq!(screen.trip(0), tripped(238));

    // 500: reset, target 50 °C, AUTO — and the coil stays under its trip.
    screen.run_to(500);
    assert_eq!(screen.send(RESET_SKIN), "null");
    assert_eq!(screen.send(SETPOINT_50), "null");
    assert_eq!(screen.send(AUTO), "null");
    let skin_limit_k = screen.snap()["trips"][0]["limit"]["k"].as_f64().unwrap();
    let mut hottest_k = f64::NEG_INFINITY;
    while screen.session.tick_index() < 900 {
        screen.run_to(screen.session.tick_index() + 1);
        hottest_k = hottest_k.max(screen.coil_k());
    }
    assert!(hottest_k < skin_limit_k - 5.0, "coil reached {hottest_k} K");
    assert_eq!(screen.trip(0), armed());
    assert_eq!(screen.trip(1), armed());
    assert_eq!(screen.loop_mode(), "auto");
    assert!(screen.duty_w() > 0.0);

    // 900: the emergency stop presses both armed trips; the state records 901.
    assert_eq!(screen.send(PRESS_SKIN), "null");
    assert_eq!(screen.send(PRESS_OUTLET), "null");
    assert_eq!(screen.duty_w(), 0.0, "the cut lands at the command");
    screen.run_to(901);
    assert_eq!(screen.trip(0), tripped_by_hand(901));
    assert_eq!(screen.trip(1), tripped_by_hand(901));

    // 901: a healthy plant resets at once — and stays dark until 950's AUTO.
    assert_eq!(screen.send(RESET_SKIN), "null");
    assert_eq!(screen.send(RESET_OUTLET), "null");
    screen.run_to(950);
    assert_eq!(screen.trip(0), armed());
    assert_eq!(screen.trip(1), armed());
    assert_eq!(screen.duty_w(), 0.0, "a reset does not relight the furnace");
    assert_eq!(screen.loop_mode(), "manual");
    assert_eq!(screen.send(AUTO), "null");
    screen.run_to(1000);
    assert!(screen.duty_w() > 0.0);
}

/// `AUTO_AUTORESET` (M40): the self-resetting tube trip cuts and relights the
/// fouled heater on its own, a lower target ends the cycle, and the emergency
/// stop is NOT undone by the trip — neither by itself nor by a person's reset.
#[test]
fn the_autoreset_timeline_tells_its_story() {
    let mut screen = Screen::load("furnace_coil_trip_autoreset.toml", "cool_feed", "hold_tank");
    let snap = screen.snap();
    assert_eq!(snap["trips"][0]["name"], "tube_skin_high");
    assert_eq!(snap["trips"][1]["name"], "outlet_high");
    assert_eq!(snap["controls"][0]["name"], "outlet_temperature");
    // The scene finds its loop by this, the heater's outlet (M41).
    let watched = snap["controls"][0]["watches"]["node"].as_u64().unwrap() as usize;
    assert_eq!(snap["nodes"][watched]["name"], "heater");
    assert_eq!(
        snap["trips"][0]["reset"],
        serde_json::json!({"mode": "auto", "reset_at": {"variable": "temperature", "k": 353.15}})
    );

    // Cuts at 75, relights by itself at 128 with no command sent, cuts at 214.
    screen.run_to(75);
    assert_eq!(screen.trip(0), tripped(75));
    assert_eq!(screen.loop_mode(), "manual");
    screen.run_to(127);
    assert_eq!(screen.trip(0), tripped(75));
    screen.run_to(128);
    assert_eq!(screen.trip(0), armed());
    assert_eq!(screen.loop_mode(), "auto", "the trip handed the loop back");
    // Bumpless: on its first tick the loop holds the zero it took over from,
    // and ramps from there.
    assert_eq!(screen.duty_w(), 0.0);
    screen.run_to(130);
    assert!(screen.duty_w() > 0.0);
    screen.run_to(214);
    assert_eq!(screen.trip(0), tripped(214));
    screen.run_to(267);
    assert_eq!(screen.trip(0), armed());

    // 300: target 50 °C, in AUTO — the cycle stops, the coil stays under 100 °C.
    screen.run_to(300);
    assert_eq!(screen.send(SETPOINT_50), "null");
    let skin_limit_k = screen.snap()["trips"][0]["limit"]["k"].as_f64().unwrap();
    let mut hottest_k = f64::NEG_INFINITY;
    while screen.session.tick_index() < 900 {
        screen.run_to(screen.session.tick_index() + 1);
        hottest_k = hottest_k.max(screen.coil_k());
    }
    assert!(hottest_k < skin_limit_k, "coil reached {hottest_k} K");
    assert_eq!(screen.trip(0), armed());
    assert_eq!(screen.loop_mode(), "auto");

    // 900: the emergency stop. The coil falls far under the 80 °C reset point,
    // and the pressed trip waits for a person all the same.
    assert_eq!(screen.send(PRESS_SKIN), "null");
    assert_eq!(screen.send(PRESS_OUTLET), "null");
    screen.run_to(1000);
    assert!(screen.coil_k() < 353.15, "coil {} K", screen.coil_k());
    assert_eq!(screen.trip(0), tripped_by_hand(901));
    assert_eq!(screen.trip(1), tripped_by_hand(901));

    // 1000: a person resets both — and the furnace stays dark: a pressed trip
    // never restarts anything (DESIGN §45).
    assert_eq!(screen.send(RESET_SKIN), "null");
    assert_eq!(screen.send(RESET_OUTLET), "null");
    screen.run_to(1100);
    assert_eq!(screen.trip(0), armed());
    assert_eq!(screen.duty_w(), 0.0);
    assert_eq!(screen.loop_mode(), "manual");
}

/// `AUTO_BURNOUT`: the tubes burst by themselves, new tubes are refused on a
/// coil still past its limit, and accepted once it has cooled.
#[test]
fn the_burnout_timeline_tells_its_story() {
    let mut screen = Screen::load("furnace_burnout.toml", "charge", "product");
    let pipe = screen.session.edge_id("heated_line") as usize;
    // PATCH, DUTY_HALF_MW and NEW_TUBES carry these ids; see the trip test.
    assert_eq!(pipe, 2, "heated_line moved: update PATCH");
    assert_eq!(
        screen.heater, 2,
        "heater moved: update the duty and tubes commands"
    );
    let tubes = |s: &Screen| s.heater()["kind"]["tubes"]["state"].clone();
    let leak = |s: &Screen| s.snap()["edges"][pipe]["leak_mass_flow"].as_f64().unwrap();
    let fire = |s: &Screen| s.heater()["tube_fire_w"].as_f64().unwrap();

    screen.run_to(1144);
    assert_eq!(tubes(&screen), serde_json::json!({"status": "intact"}));
    assert_eq!(leak(&screen), 0.0);
    screen.run_to(1145);
    assert_eq!(
        tubes(&screen),
        serde_json::json!({"status": "failed", "at_tick": 1145})
    );
    assert!(leak(&screen) > 0.0);
    assert!(fire(&screen) > 0.0);

    // 1300: patch, cut to 0.5 MW, ask for tubes — refused, coil far too hot.
    screen.run_to(1300);
    assert_eq!(screen.send(PATCH), "null");
    assert_eq!(screen.send(DUTY_HALF_MW), "null");
    let refusal: Value = serde_json::from_str(&screen.send(NEW_TUBES)).unwrap();
    assert_eq!(refusal["code"], "invalid_command");
    assert!(
        refusal["message"]
            .as_str()
            .unwrap()
            .contains("Let the coil cool first"),
        "{refusal}"
    );
    screen.run_to(1301);
    assert_eq!(leak(&screen), 0.0, "the patch stops the leak");
    assert_eq!(fire(&screen), 0.0, "and puts the fire out");

    // 2500: cooled under the limit — new tubes go in.
    screen.run_to(2500);
    let failure_k = screen.heater()["kind"]["tubes"]["failure_temperature"]
        .as_f64()
        .unwrap();
    assert!(screen.coil_k() < failure_k);
    assert_eq!(screen.send(NEW_TUBES), "null");
    screen.run_to(2501);
    assert_eq!(tubes(&screen), serde_json::json!({"status": "intact"}));
}
