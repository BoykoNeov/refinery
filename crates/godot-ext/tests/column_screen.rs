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
/// more of the feed and the reflux rises again; the pumparounds to 12 MW each at
/// 3 300 run (M67.4; the scene's 3 400 furnace press is held by its region and
/// sends nothing). The reboiler stays at zero throughout.
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
    screen.run_to(3300);
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

    for i in 0..3 {
        screen.send(&format!(
            r#"{{"cmd":"set_pumparound_duty","duty":12000000.0,"node":{column},"pumparound":{i}}}"#
        ));
    }
    screen.run_to(3600);
    assert_eq!(screen.reading().4, 0.0);
}

/// **The cold crude unit's start-up** (`AUTO_COLD` and `WALKS`, M66.4): the
/// furnace to 58 MW at 10; the pumparounds off at 500; from 500 to 2 390, every
/// 10 ticks, the reboiler key — held at the duty it runs at less 2.5 MW, the
/// scene's own arithmetic (its 1 005-tick furnace press is the screen's to
/// hold, while the reboiler is held, and sends nothing). The reboiler never
/// rises and the column declines none of the walk: since M67 (§72.1) it cuts
/// its gas oil back where the feed is still too cool for the step, and lands
/// on zero on tick 821 (M66: 2 321, after 1 534 declined ticks); on 2 700 it
/// is held there, the feed 368.1 °C, the reflux 3.088, the gas oil back to
/// 81 % of its share.
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
    assert_eq!(zero_at, Some(821), "the tick it reached zero");
    assert_eq!(declined_ticks, 0);
    let end = screen.reading();
    assert_eq!(end.4, 0.0, "{end:?}");
    assert!(!declined(&screen));
    assert!(
        (end.2 - 3.088).abs() < 1e-3 && (end.0 - 368.1).abs() < 0.1,
        "{end:?}"
    );
    let cut = &screen.column_node()["column_draws_cut_back"];
    assert_eq!(cut[0]["draw"], 4, "{cut}");
    let share = cut[0]["of_declared"].as_f64().unwrap();
    assert!((share - 0.811).abs() < 0.005, "the gas oil at {share}");
}

/// The crude unit settled at its file's state, then its three keys sent at
/// once (the region's one-jump route), run `ticks` on: whether it ran.
fn crude_jump_runs(furnace: f64, reflux: f64, pumparounds: f64, ticks: i64) -> bool {
    let mut screen = Screen::load(
        "crude_unit.toml",
        "furnace",
        "transfer_line",
        &["naphtha_draw"],
    );
    screen.run_to(1000);
    let (furnace_id, column) = (screen.furnace, screen.column);
    screen.send(&format!(
        r#"{{"cmd":"set_furnace_duty","duty":{:?},"node":{furnace_id}}}"#,
        furnace * 1.0e6
    ));
    screen.send(&format!(
        r#"{{"cmd":"set_reflux_ratio","node":{column},"reflux_ratio":{reflux:?}}}"#
    ));
    for (i, mw) in [15.0, 15.0, 10.0].iter().enumerate() {
        screen.send(&format!(
            r#"{{"cmd":"set_pumparound_duty","duty":{:?},"node":{column},"pumparound":{i}}}"#,
            mw * pumparounds * 1.0e6
        ));
    }
    (0..ticks).all(|_| screen.session.tick() == "null")
}

/// **The crude unit's region is the column's** (M67.3, DESIGN §72.8): the
/// table the scene holds its keys by holds the file's own setting, and on
/// either side of its edge the column does what the table says — at 20 MW and
/// a reflux of 1, the pumparounds at 50 % run and at 25 % do not; at 0 MW and
/// a reflux of 1.5, at 25 % they run and off they do not. A column that moves
/// its edge fails this, and the region is measured again.
#[test]
fn slow_the_crude_units_region_is_the_columns() {
    let region = region("crude");
    assert!(
        region_allows(&region, [20.0, 4.0, 1.0]),
        "the file's setting"
    );
    for (furnace, reflux, inside, outside) in [(20.0, 1.0, 0.5, 0.25), (0.0, 1.5, 0.25, 0.0)] {
        assert!(region_allows(&region, [furnace, reflux, inside]));
        assert!(!region_allows(&region, [furnace, reflux, outside]));
        assert!(
            crude_jump_runs(furnace, reflux, inside, 300),
            "{furnace} MW, reflux {reflux}, pumparounds {inside}"
        );
        assert!(
            !crude_jump_runs(furnace, reflux, outside, 300),
            "{furnace} MW, reflux {reflux}, pumparounds {outside}"
        );
    }
}

/// A plant's measured region as the scene reads it (`demo/column_region_*.json`).
fn region(plant: &str) -> Value {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join(format!("../../demo/column_region_{plant}.json"));
    serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap()
}

/// Whether a region allows its three keys at `values` (in its `keys` order),
/// all on its grid.
fn region_allows(region: &Value, values: [f64; 3]) -> bool {
    let keys = region["keys"].as_array().unwrap();
    let at: Vec<usize> = keys
        .iter()
        .zip(values)
        .map(|(key, value)| {
            region["grid"][key.as_str().unwrap()]
                .as_array()
                .unwrap()
                .iter()
                .position(|x| (x.as_f64().unwrap() - value).abs() < 1e-6)
                .unwrap_or_else(|| panic!("{key} {value} is not on the grid"))
        })
        .collect();
    region["allowed"][at[0]][at[1]].as_str().unwrap().as_bytes()[at[2]] == b'1'
}

/// The stripped crude unit started from its file with its furnace at
/// `furnace` MW and its coil declared where that furnace settles it (the
/// region's own start, M67.4), then the steam valve and every pumparound sent
/// at once on tick 20 (the region's one-jump route), run `ticks` on: whether
/// it ran.
fn stripped_jump_runs(furnace: f64, coil_c: f64, steam: f64, pumparound_mw: f64) -> bool {
    let src = scenario_src("crude_unit_stripped.toml")
        .replacen("duty_mw = 58.0\n", &format!("duty_mw = {furnace:?}\n"), 1)
        .replacen(
            "coil_temperature_c = 436.88\n",
            &format!("coil_temperature_c = {coil_c:?}\n"),
            1,
        );
    let mut session = Session::default();
    assert_eq!(session.load_scenario(&src), "null");
    let (column, valve) = (session.node_id("column"), session.node_id("steam_valve"));
    for _ in 0..20 {
        assert_eq!(session.tick(), "null", "the settled start");
    }
    let mut send = |json: String| assert_eq!(session.apply_command_json(&json), "null");
    send(format!(
        r#"{{"cmd":"set_valve_opening","node":{valve},"opening":{steam:?}}}"#
    ));
    for i in 0..3 {
        send(format!(
            r#"{{"cmd":"set_pumparound_duty","duty":{:?},"node":{column},"pumparound":{i}}}"#,
            pumparound_mw * 1.0e6
        ));
    }
    (0..100).all(|_| session.tick() == "null")
}

/// **The stripped crude unit's region is the column's** (M67.4, DESIGN §72.9):
/// the table allows the file's setting (58 MW, between its grid's 57.5 and
/// 60, the steam valve at 40 %, no pumparounds), and holds the coil its
/// furnace settles at; on either side of its edge the column does what the
/// table says — at 45 MW with the steam wide open, 11 MW of pumparound runs
/// and 12 does not. A column that moves its edge fails this, and the region
/// is measured again.
#[test]
fn slow_the_stripped_units_region_is_the_columns() {
    let region = region("stripped");
    assert!(region_allows(&region, [57.5, 0.4, 0.0]));
    assert!(region_allows(&region, [60.0, 0.4, 0.0]));
    let furnaces = region["grid"]["furnace"].as_array().unwrap();
    let at_45 = furnaces
        .iter()
        .position(|f| f.as_f64() == Some(45.0))
        .unwrap();
    let coil = region["coil_c"][at_45].as_f64().unwrap();
    assert!(region_allows(&region, [45.0, 1.0, 11.0]));
    assert!(!region_allows(&region, [45.0, 1.0, 12.0]));
    assert!(stripped_jump_runs(45.0, coil, 1.0, 11.0), "11 MW at 45 MW");
    assert!(!stripped_jump_runs(45.0, coil, 1.0, 12.0), "12 MW at 45 MW");
}
