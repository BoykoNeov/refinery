//! Gates for the column screen's scripted timelines (M60.4, `demo/column.gd`).
//!
//! As `furnace_screen.rs` does for the furnace screen: the scene is
//! demonstrated, not gated, but the story its `--auto` runs tell is — these
//! tests send the scene's own command text through the same [`Session`] the
//! gdext binding forwards to, at the ticks of its `TIMELINES`, and pin what each
//! step is there to show. The plants' physics is gated in
//! `crates/scenarios/tests` (`crude_unit.rs`, `heat_balance_column.rs`); these
//! tests own only the timelines.
//!
//! **The command text is the scene's**: Godot's `JSON.stringify` sorts the keys
//! and writes every number as a float, ids cast to int by the scene.

use refinery_godot_ext::bridge::Session;
use serde_json::Value;

fn scenario_src(name: &str) -> String {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("reading {}: {e}", path.display()))
}

/// A loaded session and the ids the scene resolves at load.
struct Screen {
    session: Session,
    furnace: i64,
    column: i64,
    feed_pipe: i64,
}

impl Screen {
    /// `_load`'s resolution: a name missing from the plant halts the scene.
    fn load(scenario: &str, furnace: &str, feed_pipe: &str, draws: &[&str]) -> Self {
        let mut session = Session::default();
        assert_eq!(session.load_scenario(&scenario_src(scenario)), "null");
        let furnace = session.node_id(furnace);
        let column = session.node_id("column");
        let feed_pipe = session.edge_id(feed_pipe);
        assert!(
            furnace >= 0 && column >= 0 && feed_pipe >= 0,
            "{scenario}: names"
        );
        for draw in draws {
            assert!(session.edge_id(draw) >= 0, "{scenario} has no draw {draw}");
        }
        Screen {
            session,
            furnace,
            column,
            feed_pipe,
        }
    }

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

    fn send(&mut self, json: &str) {
        assert_eq!(self.session.apply_command_json(json), "null", "{json}");
    }

    fn snapshot(&self) -> Value {
        serde_json::from_str(&self.session.snapshot_json()).unwrap()
    }

    fn column_node(&self) -> Value {
        self.snapshot()["nodes"][self.column as usize].clone()
    }

    /// (feed °C, flash share, reflux ran, condenser MW, reboiler MW)
    fn reading(&self) -> (f64, f64, f64, f64, f64) {
        let snapshot = self.snapshot();
        let column = &snapshot["nodes"][self.column as usize];
        let feed = snapshot["edges"]
            .as_array()
            .unwrap()
            .iter()
            .find(|e| e["id"].as_i64() == Some(self.feed_pipe))
            .unwrap()["stream"]["temperature"]
            .as_f64()
            .unwrap();
        let op = &column["column_operation"];
        (
            feed - 273.15,
            op["feed_vapour_fraction"].as_f64().unwrap(),
            op["reflux_ratio"].as_f64().unwrap(),
            column["column_duty"]["condenser_w"].as_f64().unwrap() / 1e6,
            column["column_duty"]["reboiler_w"].as_f64().unwrap() / 1e6,
        )
    }
}

/// **The crude unit** (`AUTO_CRUDE`): the furnace up to 30 MW at 600 heats the
/// feed and flashes more of it; the reflux down to 1.5 at 1 500 (where M60's
/// solve halted; Newton runs it, DESIGN §66.7) and the pumparounds to 150 % at
/// 2 400 are run as set; the column publishes ten stage temperatures rising
/// down the column.
#[test]
fn slow_the_crude_unit_timeline() {
    let mut screen = Screen::load(
        "crude_unit.toml",
        "furnace",
        "transfer_line",
        &[
            "naphtha_draw",
            "kerosene_draw",
            "diesel_draw",
            "gas_oil_draw",
            "residue_draw",
        ],
    );
    screen.run_to(600);
    let (feed, flash, reflux, _, _) = screen.reading();
    assert!((350.0..362.0).contains(&feed), "{feed}");
    assert_eq!(reflux, 4.0);
    let stages: Vec<f64> = screen.column_node()["column_operation"]["stage_temperatures_k"]
        .as_array()
        .unwrap()
        .iter()
        .map(|t| t.as_f64().unwrap())
        .collect();
    assert_eq!(stages.len(), 10);
    assert!(stages.windows(2).all(|w| w[1] > w[0]), "{stages:?}");

    let furnace = screen.furnace;
    screen.send(&format!(
        r#"{{"cmd":"set_furnace_duty","duty":30000000.0,"node":{furnace}}}"#
    ));
    screen.run_to(1500);
    let (hotter, more, _, _, _) = screen.reading();
    assert!(hotter > feed + 10.0, "{hotter} after {feed}");
    assert!(more > flash, "{more} after {flash}");

    let column = screen.column;
    screen.send(&format!(
        r#"{{"cmd":"set_reflux_ratio","node":{column},"reflux_ratio":1.5}}"#
    ));
    screen.run_to(2400);
    assert_eq!(screen.reading().2, 1.5);

    for (i, mw) in [15.0, 15.0, 10.0].iter().enumerate() {
        screen.send(&format!(
            r#"{{"cmd":"set_pumparound_duty","duty":{:.1},"node":{column},"pumparound":{i}}}"#,
            mw * 1.5e6
        ));
    }
    screen.run_to(3300);
    assert_eq!(screen.reading().2, 1.5);
}

/// **The hot-feed column past its limit** (`AUTO_HOT`): the furnace to 70 MW
/// at 600 and to 110 MW at 1 500 runs the reflux up past its setting of 2;
/// the setting dropped to 0 at 2 400 changes nothing — the column runs at what
/// the floor needs.
#[test]
fn the_hot_feed_timeline() {
    let mut screen = Screen::load(
        "crude_column_hot_feed.toml",
        "preheater",
        "hot_feed_line__downstream",
        &["naphtha_draw", "distillate_draw", "bottoms_draw"],
    );
    let (furnace, column) = (screen.furnace, screen.column);
    screen.run_to(600);
    assert_eq!(screen.reading().2, 2.0);
    screen.send(&format!(
        r#"{{"cmd":"set_furnace_duty","duty":70000000.0,"node":{furnace}}}"#
    ));
    screen.run_to(1500);
    let at_70 = screen.reading();
    assert!(at_70.2 > 2.0, "{at_70:?}");
    screen.send(&format!(
        r#"{{"cmd":"set_furnace_duty","duty":110000000.0,"node":{furnace}}}"#
    ));
    screen.run_to(2400);
    let at_110 = screen.reading();
    assert!(at_110.2 > at_70.2 + 1.0, "{at_110:?} after {at_70:?}");
    assert!(at_110.3 > at_70.3 + 10.0, "the condenser carries the heat");
    screen.send(&format!(
        r#"{{"cmd":"set_reflux_ratio","node":{column},"reflux_ratio":0.0}}"#
    ));
    screen.run_to(3000);
    let floor = screen.reading();
    assert!(
        (floor.2 - at_110.2).abs() < 1e-3 * at_110.2,
        "set to 0, ran at {} where the floor needs {}",
        floor.2,
        at_110.2
    );
}

/// **The overheated column** (`AUTO_OVERHEATED`): running above its setting of
/// 2 from the start; set to 8 at 600, above what its floor needs, it runs as
/// set.
#[test]
fn the_overheated_timeline() {
    let mut screen = Screen::load(
        "crude_column_overheated.toml",
        "preheater",
        "hot_feed_line__downstream",
        &["naphtha_draw", "distillate_draw", "bottoms_draw"],
    );
    let column = screen.column;
    screen.run_to(600);
    assert!(screen.reading().2 > 3.0, "{:?}", screen.reading());
    screen.send(&format!(
        r#"{{"cmd":"set_reflux_ratio","node":{column},"reflux_ratio":8.0}}"#
    ));
    screen.run_to(1200);
    assert_eq!(screen.reading().2, 8.0);
}

/// **The stripped crude unit** (`AUTO_STRIPPED`, M64.4): its reboiler held at
/// zero, the reflux a result. The steam valve wide open at 600 strips the
/// bottom cooler; the pumparounds to 8 MW each at 1 500 take heat off the trays
/// and the reflux and the condenser fall; the furnace to 65 MW at 2 400 flashes
/// more of the feed and the reflux rises again. The reboiler stays at zero
/// throughout.
#[test]
fn slow_the_stripped_timeline() {
    let mut screen = Screen::load(
        "crude_unit_stripped.toml",
        "furnace",
        "transfer_line",
        &[
            "naphtha_suction",
            "water_draw",
            "kerosene_suction",
            "diesel_suction",
            "gas_oil_suction",
            "residue_suction",
        ],
    );
    let steam_valve = screen.session.node_id("steam_valve");
    let steam_pipe = screen.session.edge_id("stripping_steam");
    assert!(steam_valve >= 0 && steam_pipe >= 0, "the steam's names");
    // (bottom stage °C, steam kg/s)
    let stripping = |screen: &Screen| {
        let snapshot = screen.snapshot();
        let stages = snapshot["nodes"][screen.column as usize]["column_operation"]
            ["stage_temperatures_k"]
            .as_array()
            .unwrap()
            .clone();
        let steam = snapshot["edges"]
            .as_array()
            .unwrap()
            .iter()
            .find(|e| e["id"].as_i64() == Some(steam_pipe))
            .unwrap()["stream"]["mass_flow"]
            .as_f64()
            .unwrap();
        (stages.last().unwrap().as_f64().unwrap() - 273.15, steam)
    };
    let (furnace, column) = (screen.furnace, screen.column);

    screen.run_to(600);
    let start = screen.reading();
    assert_eq!(start.4, 0.0, "the reboiler held at zero");
    assert!((start.2 - 3.460).abs() < 1e-3, "{start:?}");
    let (bottom, steam) = stripping(&screen);
    assert!((bottom - 363.63).abs() < 0.05, "{bottom}");

    screen.send(&format!(
        r#"{{"cmd":"set_valve_opening","node":{steam_valve},"opening":1.0}}"#
    ));
    screen.run_to(1500);
    let (stripped, more_steam) = stripping(&screen);
    assert!(more_steam > 2.0 * steam, "{more_steam} after {steam}");
    assert!(stripped < bottom - 8.0, "{stripped} after {bottom}");
    let steamed = screen.reading();
    assert_eq!(steamed.4, 0.0);

    for i in 0..3 {
        screen.send(&format!(
            r#"{{"cmd":"set_pumparound_duty","duty":8000000.0,"node":{column},"pumparound":{i}}}"#
        ));
    }
    screen.run_to(2400);
    let cooled = screen.reading();
    assert!(cooled.2 < steamed.2 - 1.5, "{cooled:?} after {steamed:?}");
    assert!(cooled.3 < steamed.3 - 15.0, "the condenser carries less");
    assert_eq!(cooled.4, 0.0);

    screen.send(&format!(
        r#"{{"cmd":"set_furnace_duty","duty":65000000.0,"node":{furnace}}}"#
    ));
    screen.run_to(3600);
    let fired = screen.reading();
    assert!(
        fired.1 > cooled.1 && fired.0 > cooled.0 + 5.0,
        "{fired:?} after {cooled:?}"
    );
    assert!(
        fired.2 > cooled.2 + 0.1,
        "the reflux rises: {fired:?} after {cooled:?}"
    );
    assert_eq!(fired.4, 0.0);
}

/// **The cold crude unit's start-up** (`AUTO_COLD` and `WALKS`, M66.4): the
/// furnace to 58 MW at 10; the pumparounds off at 500; from 500 to 2 390, every
/// 10 ticks, the reboiler key — held at the duty it runs at less 2.5 MW, the
/// scene's own arithmetic (its 1 005-tick furnace press is the screen's to
/// hold, while the reboiler is held, and sends nothing). The reboiler never
/// rises; the column declines 1 534 ticks of the walk, the feed still too cool
/// for the step asked, and lands on zero on tick 2 321; on 2 700 it is held
/// there, no decline, the feed 368.1 °C and the reflux 3.070.
#[test]
fn slow_the_cold_start_up_timeline() {
    let mut screen = Screen::load(
        "crude_unit_cold.toml",
        "furnace",
        "transfer_line",
        &[
            "naphtha_suction",
            "water_draw",
            "kerosene_suction",
            "diesel_suction",
            "gas_oil_suction",
            "residue_suction",
        ],
    );
    let (furnace, column) = (screen.furnace, screen.column);
    let declined = |screen: &Screen| !screen.column_node()["column_reboiler_declined"].is_null();

    screen.run_to(10);
    let cold = screen.reading();
    assert!((cold.4 - 87.945).abs() < 0.01 && cold.2 == 4.0, "{cold:?}");
    screen.send(&format!(
        r#"{{"cmd":"set_furnace_duty","duty":58000000.0,"node":{furnace}}}"#
    ));
    screen.run_to(500);
    for i in 0..3 {
        screen.send(&format!(
            r#"{{"cmd":"set_pumparound_duty","duty":0.0,"node":{column},"pumparound":{i}}}"#
        ));
    }
    let mut last = f64::INFINITY;
    let mut zero_at = None;
    let mut declined_ticks = 0;
    for tick in 500..=2_700i64 {
        screen.run_to(tick);
        let running = screen.reading().4;
        assert!(running <= last, "tick {tick}: {running} MW after {last} MW");
        last = running;
        if declined(&screen) {
            declined_ticks += 1;
        }
        if running == 0.0 && zero_at.is_none() {
            zero_at = Some(tick);
        }
        if tick <= 2_390 && (tick - 500) % 10 == 0 && running > 0.0 {
            let target = (running - 2.5).max(0.0) * 1.0e6;
            screen.send(&format!(
                r#"{{"cmd":"set_reboiler_duty","duty":{target:?},"node":{column}}}"#
            ));
        }
    }
    // A tick after `crude_unit_cold_reference.rs`'s 2 320: the scene sends each
    // command after its tick, that test before it.
    assert_eq!(zero_at, Some(2_321), "the tick it reached zero");
    assert_eq!(declined_ticks, 1_534);
    let end = screen.reading();
    assert_eq!(end.4, 0.0, "{end:?}");
    assert!(!declined(&screen));
    assert!(
        (end.2 - 3.070).abs() < 1e-3 && (end.0 - 368.1).abs() < 0.1,
        "{end:?}"
    );
}
