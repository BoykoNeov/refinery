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
/// furnace to 58 MW at 10; at 500 the pumparounds off and the reboiler held
/// (the screen holds its 400-tick reboiler press, and its first press to turn
/// them off, with the reboiler free, §72.12; it sends 50 %, the reboiler, then
/// off — the same commands before tick 501); from 500 to 2 390, every 10
/// ticks, the reboiler key — held at the duty it runs at less 2.5 MW, the
/// scene's own arithmetic (its 1 005-tick steam press is the screen's to
/// hold, while the reboiler is held, and sends nothing; M67.5). The reboiler never
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

/// Whether a region allows its keys at `values` (in its `keys` order), all on
/// its grid: `allowed` nests one list per key but the last, a string (three
/// keys, or the loop plant's four, M72.6).
fn region_allows<const N: usize>(region: &Value, values: [f64; N]) -> bool {
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
    assert_eq!(at.len(), keys.len(), "one value per key");
    let (last, outer) = at.split_last().unwrap();
    let row = outer.iter().fold(&region["allowed"], |node, &i| &node[i]);
    row.as_str().unwrap().as_bytes()[*last] == b'1'
}

/// A steam-stripped plant started from its file with its furnace at
/// `furnace` MW and its coil declared where that furnace settles it (the
/// regions' own start, M67.4), then the steam valve and the pumparounds
/// (`pumparound_mw`, one per pumparound) sent at once on tick 20 (the regions'
/// one-jump route), run 100 ticks on: whether it ran. `file`'s furnace is its
/// first `duty_mw = ` line, its coil its `coil_temperature_c`.
fn settled_jump_runs(
    file: &str,
    (furnace, coil_c): (f64, f64),
    steam: f64,
    pumparound_mw: [f64; 3],
) -> bool {
    let src: String = scenario_src(file)
        .lines()
        .scan((false, false), |done, line| {
            let out = if !done.0 && line.starts_with("duty_mw = ") {
                done.0 = true;
                format!("duty_mw = {furnace:?}")
            } else if !done.1 && line.starts_with("coil_temperature_c = ") {
                done.1 = true;
                format!("coil_temperature_c = {coil_c:?}")
            } else {
                line.to_string()
            };
            Some(out + "\n")
        })
        .collect();
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
    for (i, mw) in pumparound_mw.iter().enumerate() {
        send(format!(
            r#"{{"cmd":"set_pumparound_duty","duty":{:?},"node":{column},"pumparound":{i}}}"#,
            mw * 1.0e6
        ));
    }
    (0..100).all(|_| session.tick() == "null")
}

/// A region's settled coil at a grid furnace [°C].
fn region_coil(region: &Value, furnace: f64) -> f64 {
    let at = region["grid"]["furnace"]
        .as_array()
        .unwrap()
        .iter()
        .position(|f| f.as_f64() == Some(furnace))
        .unwrap();
    region["coil_c"][at].as_f64().unwrap()
}

/// **The stripped crude unit's region is the column's** (M67.4, DESIGN §72.9):
/// the table allows the file's setting (58 MW, between its grid's 57.5 and
/// 60, the steam valve at 40 %, no pumparounds), and holds the coil its
/// furnace settles at; on either side of its edge the column does what the
/// table says, at four points along its pumparound ceiling — at 45 MW with
/// the steam wide open, 11 MW of pumparound runs and 12 does not; at 40 MW,
/// 47.5 and 52.5, with the steam at 100 %, 40 % and 10 %, likewise a megawatt
/// apart (each outside one fails within a few ticks of the move, so a 100-tick
/// run judges it). A column that moves its edge fails this, and the region is
/// measured again.
#[test]
fn slow_the_stripped_units_region_is_the_columns() {
    let region = region("stripped");
    assert!(region_allows(&region, [57.5, 0.4, 0.0]));
    assert!(region_allows(&region, [60.0, 0.4, 0.0]));
    let file = "crude_unit_stripped.toml";
    for (furnace, steam, inside, outside) in [
        (45.0, 1.0, 11.0, 12.0),
        (40.0, 1.0, 10.0, 11.0),
        (47.5, 0.4, 11.0, 12.0),
        (52.5, 0.1, 11.0, 12.0),
    ] {
        assert!(region_allows(&region, [furnace, steam, inside]));
        assert!(!region_allows(&region, [furnace, steam, outside]));
        let settled = (furnace, region_coil(&region, furnace));
        assert!(
            settled_jump_runs(file, settled, steam, [inside; 3]),
            "{inside} MW at {furnace} MW, steam {steam}"
        );
        assert!(
            !settled_jump_runs(file, settled, steam, [outside; 3]),
            "{outside} MW at {furnace} MW, steam {steam}"
        );
    }
}

/// **The cold crude unit's region, its reboiler free, is the column's**
/// (M67.5, DESIGN §72.10): the table allows the file's setting (20 MW, the
/// steam valve at 40 %, the pumparounds at their 15/15/10 MW), and at its edge
/// — the hottest furnace with the pumparounds off — the column does what the
/// table says: at 65 MW and 40 % steam, half the pumparounds runs and none does
/// not. (A quarter runs here too, but fails some 8 400 ticks on at full steam,
/// so the region closes it at every steam setting, §72.12.) At 62.5 MW with
/// the steam wide open, a quarter runs and none fails at the move. The rest of
/// its edge, no pumparounds from 50 MW, fails only some 7 500 ticks on, too
/// late for a test (C19).
#[test]
fn slow_the_cold_units_region_is_the_columns() {
    let region = region("cold");
    assert!(region_allows(&region, [20.0, 0.4, 1.0]));
    assert!(region_allows(&region, [65.0, 0.4, 0.5]));
    assert!(!region_allows(&region, [65.0, 0.4, 0.25]));
    assert!(!region_allows(&region, [65.0, 0.4, 0.0]));
    let at_65 = (65.0, region_coil(&region, 65.0));
    let file = "crude_unit_cold.toml";
    assert!(
        settled_jump_runs(file, at_65, 0.4, [7.5, 7.5, 5.0]),
        "half the pumparounds at 65 MW"
    );
    assert!(
        !settled_jump_runs(file, at_65, 0.4, [0.0; 3]),
        "no pumparounds at 65 MW"
    );
    assert!(region_allows(&region, [62.5, 1.0, 0.25]));
    assert!(!region_allows(&region, [62.5, 1.0, 0.0]));
    let at_62 = (62.5, region_coil(&region, 62.5));
    assert!(
        settled_jump_runs(file, at_62, 1.0, [3.75, 3.75, 2.5]),
        "a quarter of the pumparounds at 62.5 MW"
    );
    assert!(
        !settled_jump_runs(file, at_62, 1.0, [0.0; 3]),
        "no pumparounds at 62.5 MW"
    );
}

/// The loop plant's file flows (kg/s) and return temperatures (°C), as the
/// scene reads them from the column at load (`crude_unit_loops.toml`).
const LOOP_FLOWS: [f64; 3] = [85.0, 95.0, 65.0];
const LOOP_RETURNS_C: [f64; 3] = [150.0, 215.0, 270.0];

impl Screen {
    /// The loop plant (M72.6): the scene's names for it.
    fn loops() -> Self {
        Screen::load(
            "crude_unit_loops.toml",
            "furnace",
            "transfer_line",
            &[
                "naphtha_draw",
                "kerosene_draw",
                "diesel_draw",
                "gas_oil_draw",
                "residue_draw",
            ],
        )
    }

    /// The tray plant (M73.4): the loop plant with its trays sized, the same
    /// names.
    fn trays() -> Self {
        Screen::load(
            "crude_unit_trays.toml",
            "furnace",
            "transfer_line",
            &[
                "naphtha_draw",
                "kerosene_draw",
                "diesel_draw",
                "gas_oil_draw",
                "residue_draw",
            ],
        )
    }

    /// The trays the column names flooding and weeping (stages, top down);
    /// empty while none does.
    fn trays_named(&self) -> (Vec<u64>, Vec<u64>) {
        let op = self.column_node()["column_operation"].clone();
        let named = |key: &str| -> Vec<u64> {
            op[key]
                .as_array()
                .map(|l| l.iter().map(|x| x.as_u64().unwrap()).collect())
                .unwrap_or_default()
        };
        assert_eq!(
            op["trays"].as_array().map(Vec::len),
            Some(12),
            "a tray per stage above the reboiler"
        );
        (named("trays_flooding"), named("trays_weeping"))
    }

    /// `_set_loops_flow`: every loop at `share` of its file's flow.
    fn send_loops_flow(&mut self, share: f64) {
        let column = self.column;
        for (i, flow) in LOOP_FLOWS.iter().enumerate() {
            self.send(&format!(
                r#"{{"cmd":"set_loop_flow","flow":{:?},"node":{column},"pumparound_loop":{i}}}"#,
                flow * share
            ));
        }
    }

    /// `_set_loops_return`: every loop returning `kelvin` from its file's.
    fn send_loops_return(&mut self, kelvin: f64) {
        let column = self.column;
        for (i, celsius) in LOOP_RETURNS_C.iter().enumerate() {
            self.send(&format!(
                r#"{{"cmd":"set_loop_return_temperature","node":{column},"pumparound_loop":{i},"temperature":{:?}}}"#,
                celsius + 273.15 + kelvin
            ));
        }
    }

    /// Each loop's cooler duty (MW) and whether it runs at its capacity.
    fn loop_duties(&self) -> Vec<(f64, bool)> {
        self.column_node()["column_operation"]["loops"]
            .as_array()
            .unwrap()
            .iter()
            .map(|run| {
                (
                    run["duty_w"].as_f64().unwrap() / 1e6,
                    run["at_capacity"].as_bool().unwrap_or(false),
                )
            })
            .collect()
    }
}

/// **The loop plant** (`AUTO_LOOPS`, M72.6): the furnace to 30 MW at 600; the
/// loops' flow to 150 % at 1 500 takes more heat off the trays; to 200 % at
/// 2 400 runs two coolers at their capacity; their return 40 K warmer at 3 300
/// takes less off and frees them; the reflux to 1.5 at 3 600 runs as set; the
/// return key pressed to +60 K at 3 900 is past the table's edge, which the
/// reflux at 2.0 clears — the screen holds it, and the column runs on.
#[test]
fn slow_the_loop_plant_timeline() {
    let mut screen = Screen::loops();
    screen.run_to(600);
    assert_eq!(screen.reading().2, 4.0);
    let total = |duties: &[(f64, bool)]| duties.iter().map(|d| d.0).sum::<f64>();
    let at_file = screen.loop_duties();
    assert!(at_file.iter().all(|d| !d.1), "{at_file:?}");

    let furnace = screen.furnace;
    screen.send(&format!(
        r#"{{"cmd":"set_furnace_duty","duty":30000000.0,"node":{furnace}}}"#
    ));
    screen.run_to(1500);
    let before = total(&screen.loop_duties());

    screen.send_loops_flow(1.5);
    screen.run_to(2400);
    let faster = total(&screen.loop_duties());
    assert!(faster > before + 5.0, "{faster} MW after {before}");

    screen.send_loops_flow(2.0);
    screen.run_to(3300);
    let full = screen.loop_duties();
    assert_eq!(full.iter().filter(|d| d.1).count(), 2, "{full:?}");

    screen.send_loops_return(40.0);
    screen.run_to(3600);
    let warmer = screen.loop_duties();
    assert!(warmer.iter().all(|d| !d.1), "{warmer:?}");
    assert!(
        total(&warmer) < total(&full) - 5.0,
        "{warmer:?} after {full:?}"
    );

    let column = screen.column;
    screen.send(&format!(
        r#"{{"cmd":"set_reflux_ratio","node":{column},"reflux_ratio":1.5}}"#
    ));
    screen.run_to(3900);
    assert_eq!(screen.reading().2, 1.5);

    let region = region("loops");
    assert!(region_allows(&region, [30.0, 1.5, 2.0, 40.0]));
    assert!(!region_allows(&region, [30.0, 1.5, 2.0, 60.0]));
    assert!(region_allows(&region, [30.0, 2.0, 2.0, 60.0]));
    screen.run_to(4200);
    assert_eq!(screen.reading().2, 1.5);
}

/// The loop plant settled at its file's state, then its four keys sent at once
/// (the region's one-jump route), run `ticks` on: whether it ran.
fn loops_jump_runs(furnace: f64, reflux: f64, flow: f64, ret: f64, ticks: i64) -> bool {
    loop_plant_jump_runs(Screen::loops(), [furnace, reflux, flow, ret], ticks)
}

/// A loop plant (`Screen::loops` or `::trays`) settled at its file's state,
/// then its four keys — furnace MW, reflux, loops' flow share, return K — sent
/// at once (the regions' one-jump route), run `ticks` on: whether it ran.
fn loop_plant_jump_runs(
    mut screen: Screen,
    [furnace, reflux, flow, ret]: [f64; 4],
    ticks: i64,
) -> bool {
    screen.run_to(1000);
    let (furnace_id, column) = (screen.furnace, screen.column);
    screen.send(&format!(
        r#"{{"cmd":"set_furnace_duty","duty":{:?},"node":{furnace_id}}}"#,
        furnace * 1.0e6
    ));
    screen.send(&format!(
        r#"{{"cmd":"set_reflux_ratio","node":{column},"reflux_ratio":{reflux:?}}}"#
    ));
    screen.send_loops_flow(flow);
    screen.send_loops_return(ret);
    (0..ticks).all(|_| screen.session.tick() == "null")
}

/// **The loop plant's region is the column's** (M72.6, DESIGN §77.15): the
/// table holds the file's own setting, and on either side of its edge the
/// column does what the table says — at 20 MW and a reflux of 1, the loops at
/// half their flow run returning as declared and not 20 K warmer; at 0 MW and
/// a reflux of 2, at a quarter of their flow they run 20 K warmer and not
/// 40 K. A failing jump fails on its first tick, so a short run decides it.
#[test]
fn slow_the_loop_units_region_is_the_columns() {
    let region = region("loops");
    assert!(
        region_allows(&region, [20.0, 4.0, 1.0, 0.0]),
        "the file's setting"
    );
    for (furnace, reflux, flow, inside, outside) in
        [(20.0, 1.0, 0.5, 0.0, 20.0), (0.0, 2.0, 0.25, 20.0, 40.0)]
    {
        assert!(region_allows(&region, [furnace, reflux, flow, inside]));
        assert!(!region_allows(&region, [furnace, reflux, flow, outside]));
        assert!(
            loops_jump_runs(furnace, reflux, flow, inside, 100),
            "{furnace} MW, reflux {reflux}, flow {flow}, return {inside:+} K"
        );
        assert!(
            !loops_jump_runs(furnace, reflux, flow, outside, 100),
            "{furnace} MW, reflux {reflux}, flow {flow}, return {outside:+} K"
        );
    }
}

/// **The tray plant** (`AUTO_TRAYS`, M73.4, DESIGN §78.11): the loop plant on
/// sized trays, the furnace at the file's 20 MW. The reflux down to 1.5 at 600
/// and the middle trays weep; up to 6 at 1 500 and the top four flood; to 10 at
/// 2 400 and all twelve flood while the column runs as set, the pressure over
/// it less than the drops it reads (capped at flood, §78.9); back to 2 at
/// 3 300, every tray inside its window. The loops' flow to 200 % at 3 600 and
/// their return 20 K colder at 3 900 flood the trays their liquid runs down;
/// the reflux to 1 at 4 200 and the top two weep while 9 and 10 still flood;
/// their return to +40 K at 4 500 and every tray weeps; the return key pressed
/// to +60 K at 4 800 is past the table's edge, which the reflux at 2.0 clears —
/// the screen holds it, and the column runs on.
#[test]
fn slow_the_tray_plant_timeline() {
    let mut screen = Screen::trays();
    let none: Vec<u64> = vec![];
    let all: Vec<u64> = (1..=12).collect();
    let column = screen.column;
    let reflux = |screen: &mut Screen, ratio: f64| {
        screen.send(&format!(
            r#"{{"cmd":"set_reflux_ratio","node":{column},"reflux_ratio":{ratio:?}}}"#
        ))
    };
    screen.run_to(600);
    assert_eq!(
        screen.trays_named(),
        (none.clone(), none.clone()),
        "the file's"
    );

    reflux(&mut screen, 1.5);
    screen.run_to(1500);
    assert_eq!(screen.trays_named(), (none.clone(), (3..=8).collect()));

    reflux(&mut screen, 6.0);
    screen.run_to(2400);
    assert_eq!(screen.trays_named(), ((1..=4).collect(), none.clone()));

    reflux(&mut screen, 10.0);
    screen.run_to(3300);
    assert_eq!(screen.trays_named(), (all.clone(), none.clone()));
    assert_eq!(screen.reading().2, 10.0, "flooded, the column runs as set");
    let trays = screen.column_node()["column_operation"]["trays"].clone();
    let trays = trays.as_array().unwrap();
    let span =
        trays[11]["pressure_pa"].as_f64().unwrap() - trays[0]["pressure_pa"].as_f64().unwrap();
    let read: f64 = trays[..11]
        .iter()
        .map(|t| t["pressure_drop_pa"].as_f64().unwrap())
        .sum();
    assert!(span < read, "capped: {span} Pa over {read} Pa read");

    reflux(&mut screen, 2.0);
    screen.run_to(3600);
    assert_eq!(screen.trays_named(), (none.clone(), none.clone()));

    screen.send_loops_flow(2.0);
    screen.run_to(3900);
    assert_eq!(screen.trays_named(), ((9..=11).collect(), none.clone()));

    screen.send_loops_return(-20.0);
    screen.run_to(4200);
    assert_eq!(screen.trays_named(), ((9..=12).collect(), none.clone()));

    reflux(&mut screen, 1.0);
    screen.run_to(4500);
    assert_eq!(
        screen.trays_named(),
        ((9..=10).collect(), (1..=2).collect())
    );

    screen.send_loops_return(40.0);
    screen.run_to(4800);
    assert_eq!(screen.trays_named(), (none.clone(), all.clone()));

    let region = region("trays");
    assert!(region_allows(&region, [20.0, 1.0, 2.0, 40.0]));
    assert!(!region_allows(&region, [20.0, 1.0, 2.0, 60.0]));
    assert!(region_allows(&region, [20.0, 2.0, 2.0, 60.0]));
    assert!(!region_allows(&region, [20.0, 1.5, 2.0, 60.0]));
    screen.run_to(5100);
    assert_eq!(screen.reading().2, 1.0);
}

/// **The tray plant's region is the column's** (M73.4, DESIGN §78.11): the
/// table holds the file's own setting, and on either side of its edge the
/// column does what the table says — at 20 MW and a reflux of 2, the loops
/// returning 60 K warm run at 125 % of their flow and not at 100 % (the loop
/// plant needs 150 % there: the one place below 25 MW the trays moved the
/// edge); at 0 MW and a reflux of 1, at half their flow they run returning as
/// declared and not 20 K warmer. A failing jump fails on its first tick
/// (every one measured did), so a short run decides it.
#[test]
fn slow_the_tray_units_region_is_the_columns() {
    let trays = region("trays");
    assert!(
        region_allows(&trays, [20.0, 4.0, 1.0, 0.0]),
        "the file's setting"
    );
    for (inside, outside) in [
        ([20.0, 2.0, 1.25, 60.0], [20.0, 2.0, 1.0, 60.0]),
        ([0.0, 1.0, 0.5, 0.0], [0.0, 1.0, 0.5, 20.0]),
    ] {
        assert!(region_allows(&trays, inside), "{inside:?}");
        assert!(!region_allows(&trays, outside), "{outside:?}");
        assert!(
            loop_plant_jump_runs(Screen::trays(), inside, 100),
            "{inside:?} runs"
        );
        assert!(
            !loop_plant_jump_runs(Screen::trays(), outside, 100),
            "{outside:?} does not"
        );
    }
    let loops = region("loops");
    assert!(
        !region_allows(&loops, [20.0, 2.0, 1.25, 60.0]),
        "the loop plant's edge"
    );
}
