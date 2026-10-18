# M60.4's demonstrated criterion: a screen for the column fed hot from its
# furnace (docs/DESIGN.md §65).
#
# The heat-balance cascade (M60.1), the reflux that rises past the column's
# limit (M60.2), and the pumparounds of a realistic crude unit (M60.3) ran only
# in tests and the CLI. This scene draws them, on any of three shipped plants:
#
#   crude       scenarios/crude_unit.toml — a crude unit: a furnace firing a
#               270 °C crude to 355 °C, a ten-stage column with naphtha,
#               kerosene, diesel and gas oil drawn above the feed and three
#               pumparounds. Its keys stop at the range measured safe (§65.9),
#               widened when M61 gave the column a Newton solve (§66.7).
#   hot         scenarios/crude_column_hot_feed.toml — a light crude fed 100 K
#               past its bubble point; fire the furnace through the column's
#               limit and watch the reflux rise past its setting.
#   overheated  scenarios/crude_column_overheated.toml — that column already
#               past its limit, running above the reflux it is set to.
#   stripped    scenarios/crude_unit_stripped.toml — the crude unit with no
#               reboiler (M64, §69): its bottom stripped by steam alone, the
#               furnace carrying the heat, the reflux a result. Its keys are
#               the furnace, the steam valve and the pumparounds (the user's
#               decision 2, §69); the reflux keys say why they do nothing.
#   cold        scenarios/crude_unit_cold.toml — that crude unit before its
#               start-up (M66, §71): the furnace low, the column on its
#               reboiler. Start it: fire the furnace, take the pumparounds off,
#               walk the reboiler down from the duty it runs at (T/G), free it
#               again (Y). A step the feed is still too cool for is declined by
#               the column, which keeps the duty it ran at and says so.
#
# Same rule as every screen: this script computes nothing physical. Every
# number it draws is a snapshot field — the stage temperatures and vapour, the
# feed's flash, the reflux the column RAN at, the duties — and every refusal it
# shows is the engine's own message. The one rule it adds is the screen's own:
# its keys stop at each plant's safe range (the user's decision, §65.9), and it
# says so when a key would go past it. Display units only: kelvin to °C, watts
# to MW (CLAUDE.md rule 4).
#
# Interactive:  godot --path . res://demo/column.tscn
#               (keys listed on screen and in _input; 1 / 2 / 3 / 4 / 5 switch plants)
# Recorded:     godot --headless --path . res://demo/column.tscn --quit-after 20000 -- --auto [--plant=hot]
#               ...runs that plant's scripted timeline (TIMELINES) and prints a
#               `t=` line at a fixed interval plus one line per command.
# Screenshots:  add --shots=<dir> to a WINDOWED --auto run to save a PNG at each
#               scripted event.
extends Node2D

## The plants, the names on each the scene draws, and the range its keys stay
## inside. A name missing from a plant is a load-time halt, not a guess.
## `draws` lists each product's pipe and label TOP-DOWN, the order the
## column's own `draws` list them.
const PLANTS := {
	"crude":
	{
		"scenario": "res://scenarios/crude_unit.toml",
		"title": "A crude unit: furnace, column, three side draws and three pumparounds",
		"furnace": "furnace",
		"feed_pipe": "transfer_line",
		"draws":
		[
			["naphtha_draw", "naphtha"],
			["kerosene_draw", "kerosene"],
			["diesel_draw", "diesel"],
			["gas_oil_draw", "gas oil"],
			["residue_draw", "residue"],
		],
		"ticks_per_frame": 8,
		# The range measured warm from the running plant under Newton (M61.2,
		# §66.7): every corner of it — the furnace 0 or 40 MW, the reflux 1.5 or
		# 15, the pumparounds 50 % or 400 % — runs in one step. Below it a low
		# reflux with low pumparounds runs the gas-oil tray dry.
		"furnace_mw": [0.0, 40.0],
		"reflux": [1.5, 15.0],
		"pumparounds": [0.5, 4.0],
	},
	"hot":
	{
		"scenario": "res://scenarios/crude_column_hot_feed.toml",
		"title": "A light crude fed hot: fire the furnace through the column's limit",
		"furnace": "preheater",
		"feed_pipe": "hot_feed_line",
		"draws":
		[
			["naphtha_draw", "naphtha"],
			["distillate_draw", "distillate"],
			["bottoms_draw", "bottoms"],
		],
		"ticks_per_frame": 8,
		# The coil stays under its tube limit to about 110 MW (§65.8); past the
		# column's limit the reflux rises, so any setting runs.
		"furnace_mw": [0.0, 110.0],
		"reflux": [0.0, 8.0],
	},
	"overheated":
	{
		"scenario": "res://scenarios/crude_column_overheated.toml",
		"title": "The same column past its limit: it runs above the reflux it is set to",
		"furnace": "preheater",
		"feed_pipe": "hot_feed_line",
		"draws":
		[
			["naphtha_draw", "naphtha"],
			["distillate_draw", "distillate"],
			["bottoms_draw", "bottoms"],
		],
		"ticks_per_frame": 8,
		"furnace_mw": [0.0, 110.0],
		"reflux": [0.0, 8.0],
	},
	"stripped":
	{
		"scenario": "res://scenarios/crude_unit_stripped.toml",
		"title": "A crude unit stripped by steam: no reboiler, the reflux a result",
		"furnace": "furnace",
		"feed_pipe": "transfer_line",
		"steam_valve": "steam_valve",
		"steam_pipe": "stripping_steam",
		"draws":
		[
			["naphtha_suction", "naphtha"],
			["water_draw", "sour water"],
			["kerosene_suction", "kerosene"],
			["diesel_suction", "diesel"],
			["gas_oil_suction", "gas oil"],
			["residue_suction", "residue"],
		],
		# Measured on the screen (M64.4, §69.10): 5 ms a tick settled, 10–12
		# over its first 600 ticks and 24–26 while the coil drifts after a
		# furnace step (ledger A29); three ticks keep a settled frame near 15 ms.
		"ticks_per_frame": 3,
		# The range measured warm from the running plant (M64.4, §69.10): every
		# corner — the furnace 52.5 or 65 MW, the steam valve 10 % or 100 %,
		# every pumparound 0 or 8 MW — runs, reached in one step from the
		# running plant or from the opposite corner. Past it: the furnace at
		# 50 MW with the steam at 10 % (the bottom stage, as the coil cools),
		# the pumparounds at 10 MW with the steam at 10 % (at once). The top is
		# the user's (§69.10): at 65 MW at least about 76 % of the feed boils
		# off by mass, near a real flash zone's 60–75 %; at 80 MW (which runs)
		# at least 91 %.
		"furnace_mw": [52.5, 65.0],
		"steam": [0.1, 1.0],
		"pumparound_mw": [0.0, 8.0],
		"range_ref": "§69.10",
	},
	"cold":
	{
		"scenario": "res://scenarios/crude_unit_cold.toml",
		"title": "A crude unit before its start-up: hand the reboiler down as the feed heats",
		"furnace": "furnace",
		"feed_pipe": "transfer_line",
		"steam_valve": "steam_valve",
		"steam_pipe": "stripping_steam",
		"draws":
		[
			["naphtha_suction", "naphtha"],
			["water_draw", "sour water"],
			["kerosene_suction", "kerosene"],
			["diesel_suction", "diesel"],
			["gas_oil_suction", "gas oil"],
			["residue_suction", "residue"],
		],
		# Not measured on the screen yet: the machine was under other load when
		# this plant was added (M66.4, §71.11). The stripped plant's three.
		"ticks_per_frame": 3,
		# The reboiler on keys (T/G, Y): a hold the column cannot run at yet is
		# declined (M66.2). While it is held the keys that could raise the
		# column's edge — the furnace down, the steam down, the pumparounds —
		# are the screen's to hold (§71.11); free, the box below.
		"reboiler": true,
		"furnace_mw": [20.0, 65.0],
		"steam": [0.1, 1.0],
		"pumparounds": [0.0, 1.0],
		"range_ref": "§71.11",
	},
}

## Scripted timelines for the recorded runs, in ticks. Each is replayed with the
## same command text by `crates/godot-ext/tests/column_screen.rs`, which pins
## what it is there to show.
##
## crude: the furnace up to 30 MW at 600 heats the feed and flashes more of it;
## the reflux down to 1.5 at 1 500 (where M60's solve halted, §66.7) and the
## pumparounds to 150 % at 2 400 run as set; at 3 000 the reflux key is pressed
## below 1.5 and the screen holds it — the edge of the safe range.
const AUTO_CRUDE := {
	600: ["furnace_30"],
	1500: ["reflux_1.5"],
	2400: ["pumparounds_1.5"],
	3000: ["reflux_down"],
	3300: ["quit"],
}
## hot: the furnace to 70 MW at 600 and to 110 MW at 1 500 runs the reflux up
## past its setting of 2 and the condenser with it; the setting dropped to 0 at
## 2 400 changes nothing — the column runs at what its vapour floor needs.
const AUTO_HOT := {
	600: ["furnace_70"],
	1500: ["furnace_110"],
	2400: ["reflux_0"],
	3000: ["quit"],
}
## overheated: running above its setting from the start; set to 8 at 600, above
## what the floor needs, it runs as set.
const AUTO_OVERHEATED := {
	600: ["reflux_8"],
	1200: ["quit"],
}
## stripped: the steam valve wide open at 600 strips the bottom cooler; the
## pumparounds to 8 MW each at 1 500 take heat off the trays and the reflux
## falls with the condenser's load; the furnace to 65 MW at 2 400 flashes more
## of the feed and the reflux rises again; at 3 300 the pumparound key is
## pressed past 8 MW and the screen holds it — the edge of the safe range.
const AUTO_STRIPPED := {
	600: ["steam_1.0"],
	1500: ["pumparounds_8mw"],
	2400: ["furnace_65"],
	3300: ["pumparounds_up"],
	3600: ["quit"],
}
## cold: the start-up (M66, §71.10). The furnace to 58 MW at 10; the
## pumparounds off at 500; from 500 the reboiler key pressed every 10 ticks
## (`WALKS`), each press the duty it runs at less 2.5 MW — the column declining
## the steps the feed is still too cool for, landing on zero near 2 320; at
## 1 000 the furnace key pressed down while the reboiler is held, and the
## screen holds it.
const AUTO_COLD := {
	10: ["furnace_58"],
	500: ["pumparounds_0"],
	1000: ["furnace_down"],
	2700: ["quit"],
}
## The reboiler key pressed every `[2]` ticks from `[0]` to `[1]` on a timeline.
const WALKS := {"cold": [500, 2390, 10]}
const TIMELINES := {
	"crude": AUTO_CRUDE, "hot": AUTO_HOT, "overheated": AUTO_OVERHEATED, "stripped": AUTO_STRIPPED, "cold": AUTO_COLD
}
const AUTO_SHOT_TICKS := {
	"crude": [600, 1500, 2400, 3000],
	"hot": [600, 1500, 2400, 3000],
	"overheated": [600, 1200],
	"stripped": [600, 1500, 2400, 3300],
	"cold": [500, 1000, 2000, 2400],
}
const PRINT_EVERY := 300

## Step sizes for the keys.
const DUTY_STEP_MW := 2.5
const REFLUX_STEP := 0.5
const PUMPAROUND_STEP := 0.25
## On a plant whose pumparounds start at zero the keys step every one by
## this much [MW], not by a share of a duty the file declares.
const PUMPAROUND_STEP_MW := 1.0
const STEAM_STEP := 0.05
const MAX_TICKS_PER_FRAME := 64

@onready var sim: RefinerySim = $Sim

var plant_key := "crude"
var furnace_id := -1
var column_id := -1
var feed_pipe_id := -1
## The steam valve and its pipe into the bottom stage, -1 on a plant without.
var steam_valve_id := -1
var steam_pipe_id := -1
var draw_ids: Array = []
## The pumparounds' duties as the file declares them [W]: the 100 % the keys
## scale from.
var design_pumparounds: Array = []
var snapshot: Dictionary = {}
var halted := ""
var auto_run := false
var shots_dir := ""
var paused := false
var finished := false
var shooting := false
var ticks_per_frame := 1
var message := ""
var message_bad := false
## The reflux the column ran at last frame, to print a line when it parts from
## (or rejoins) its setting.
var seen_raised := false
## The wall time the last PRINT_EVERY ticks took [µs], tick and snapshot both:
## the screen's own cost, printed on the `t=` line.
var block_usec := 0


func _ready() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg == "--auto":
			auto_run = true
		elif arg.begins_with("--plant="):
			plant_key = arg.trim_prefix("--plant=")
		elif arg.begins_with("--shots="):
			shots_dir = arg.trim_prefix("--shots=")
	if not PLANTS.has(plant_key):
		_halt("no plant '%s' (crude | hot | overheated | stripped | cold)" % plant_key)
		return
	_load(plant_key)


func _load(key: String) -> void:
	plant_key = key
	halted = ""
	finished = false
	snapshot = {}
	seen_raised = false
	block_usec = 0
	var plant: Dictionary = PLANTS[key]
	ticks_per_frame = plant["ticks_per_frame"]

	var err = JSON.parse_string(sim.load_scenario(plant["scenario"]))
	if err != null:
		_halt("load failed: %s" % err)
		return

	furnace_id = sim.node_id(plant["furnace"])
	column_id = sim.node_id("column")
	feed_pipe_id = sim.edge_id(plant["feed_pipe"])
	draw_ids = []
	for draw in plant["draws"]:
		draw_ids.append(sim.edge_id(draw[0]))
	steam_valve_id = sim.node_id(plant["steam_valve"]) if plant.has("steam_valve") else -1
	steam_pipe_id = sim.edge_id(plant["steam_pipe"]) if plant.has("steam_pipe") else -1
	var steam_missing := plant.has("steam_valve") and (steam_valve_id < 0 or steam_pipe_id < 0)
	if furnace_id < 0 or column_id < 0 or feed_pipe_id < 0 or draw_ids.has(-1) or steam_missing:
		_halt("this scenario does not have the names the scene expects")
		return

	snapshot = JSON.parse_string(sim.snapshot_json())
	design_pumparounds = []
	for pumparound in _pumparounds():
		design_pumparounds.append(float(pumparound["duty"]))
	_say("loaded %s" % plant["scenario"], false)
	print("column: loaded %s" % plant["scenario"])
	queue_redraw()


func _physics_process(_delta: float) -> void:
	if halted != "" or paused or finished or shooting:
		return
	for i in ticks_per_frame:
		if not _step():
			return
	queue_redraw()


## One engine tick, then the scripted timeline for it. False when the scene must
## stop ticking this frame.
func _step() -> bool:
	var started := Time.get_ticks_usec()
	var err = JSON.parse_string(sim.tick())
	if err != null:
		# The engine's own reason — past the safe range it names the settings
		# the column could not settle at (§65.9).
		_halt("tick %d: %s" % [sim.tick_index(), err["message"]])
		return false
	snapshot = JSON.parse_string(sim.snapshot_json())
	block_usec += Time.get_ticks_usec() - started
	var tick := sim.tick_index()
	_report_events(tick)

	if not auto_run:
		return true
	if tick % PRINT_EVERY == 0:
		print("%s  [%.1f ms/tick]" % [_readout(tick), block_usec / 1000.0 / PRINT_EVERY])
		block_usec = 0
	var timeline: Dictionary = TIMELINES[plant_key]
	if timeline.has(tick):
		for action in timeline[tick]:
			if action == "quit":
				finished = true
				get_tree().quit()
				return false
			_do(action)
	if WALKS.has(plant_key):
		var walk: Array = WALKS[plant_key]
		if tick >= walk[0] and tick <= walk[1] and (tick - walk[0]) % walk[2] == 0 and _mw("reboiler_w") > 0.0:
			_do("reboiler_down")
	if shots_dir != "" and tick in AUTO_SHOT_TICKS[plant_key]:
		_shoot(tick)
		return false
	return true


## A line when the column starts or stops running above its reflux setting.
## A held column has no setting to part from: its reflux is a result.
func _report_events(tick: int) -> void:
	var raised := _raised()
	if raised != seen_raised:
		print(
			"column: t=%d  reflux %s (set %.2f, ran %.3f)"
			% [tick, "RAISED above its setting" if raised else "back at its setting", _reflux_set(), _reflux_ran()]
		)
	seen_raised = raised


func _shoot(tick: int) -> void:
	shooting = true
	queue_redraw()
	await RenderingServer.frame_post_draw
	var path := "%s/column_%s_t%04d.png" % [shots_dir, plant_key, tick]
	var image := get_viewport().get_texture().get_image()
	if image != null:
		image.save_png(path)
		print("column: saved %s" % path)
	shooting = false


func _input(event: InputEvent) -> void:
	if not (event is InputEventKey and event.pressed and not event.echo):
		return
	match event.keycode:
		KEY_1:
			_load("crude")
		KEY_2:
			_load("hot")
		KEY_3:
			_load("overheated")
		KEY_4:
			_load("stripped")
		KEY_5:
			_load("cold")
		KEY_SPACE:
			paused = not paused
		KEY_BRACKETLEFT:
			ticks_per_frame = maxi(1, ticks_per_frame / 2)
		KEY_BRACKETRIGHT:
			ticks_per_frame = mini(MAX_TICKS_PER_FRAME, ticks_per_frame * 2)
		KEY_UP:
			_do("furnace_up")
		KEY_DOWN:
			_do("furnace_down")
		KEY_W:
			_do("reflux_up")
		KEY_S:
			_do("reflux_down")
		KEY_E:
			_do("pumparounds_up")
		KEY_D:
			_do("pumparounds_down")
		KEY_R:
			_do("steam_up")
		KEY_F:
			_do("steam_down")
		KEY_T:
			_do("reboiler_up")
		KEY_G:
			_do("reboiler_down")
		KEY_Y:
			_do("reboiler_free")
	queue_redraw()


# --------------------------------------------------------------- commands

## Every action the keys and the timelines share. A key that would leave the
## plant's safe range is held at its edge and says so; nothing is sent.
func _do(action: String) -> void:
	var plant: Dictionary = PLANTS[plant_key]
	match action:
		"furnace_up", "furnace_down":
			var step := DUTY_STEP_MW if action == "furnace_up" else -DUTY_STEP_MW
			_set_furnace_mw(_duty_w() / 1.0e6 + step)
		"furnace_30":
			_set_furnace_mw(30.0)
		"furnace_58":
			_set_furnace_mw(58.0)
		"furnace_70":
			_set_furnace_mw(70.0)
		"furnace_65":
			_set_furnace_mw(65.0)
		"furnace_110":
			_set_furnace_mw(110.0)
		"reflux_up", "reflux_down":
			if not plant.has("reflux"):
				if _held():
					_say("the reflux is a result on this column: its reboiler is held at %.1f MW, so the heat the feed brings sets it" % _mw("reboiler_w"), true)
				else:
					_say("the reflux stays at its declared %.1f on this plant: its keys' range is not measured (DESIGN %s)" % [_reflux_set(), plant["range_ref"]], true)
				return
			var step := REFLUX_STEP if action == "reflux_up" else -REFLUX_STEP
			_set_reflux(_reflux_set() + step)
		"reflux_0":
			_set_reflux(0.0)
		"reflux_1.5":
			_set_reflux(1.5)
		"reflux_2":
			_set_reflux(2.0)
		"reflux_8":
			_set_reflux(8.0)
		"pumparounds_up", "pumparounds_down":
			var up := action == "pumparounds_up"
			if plant.has("pumparound_mw"):
				_set_pumparound_mw(_pumparound_mw() + (PUMPAROUND_STEP_MW if up else -PUMPAROUND_STEP_MW))
			else:
				_set_pumparounds(_pumparound_scale() + (PUMPAROUND_STEP if up else -PUMPAROUND_STEP))
		"pumparounds_1.5":
			_set_pumparounds(1.5)
		"pumparounds_0":
			_set_pumparounds(0.0)
		"pumparounds_8mw":
			_set_pumparound_mw(8.0)
		"steam_up", "steam_down":
			if steam_valve_id < 0:
				_say("this column has no steam valve", true)
				return
			_set_steam(snappedf(_steam_opening() + (STEAM_STEP if action == "steam_up" else -STEAM_STEP), STEAM_STEP))
		"steam_1.0":
			_set_steam(1.0)
		"reboiler_up", "reboiler_down":
			if not plant.has("reboiler"):
				_say("this plant's reboiler is not on a key", true)
				return
			# From the duty it RUNS at, so a walk follows the feed (§71.3).
			var running := _mw("reboiler_w")
			var target := running + DUTY_STEP_MW if action == "reboiler_up" else maxf(0.0, running - DUTY_STEP_MW)
			if action == "reboiler_down" and running <= 0.0:
				_hold("reboiler", "0.0 MW")
				return
			_send("hold the reboiler at %.1f MW" % target, {"cmd": "set_reboiler_duty", "node": column_id, "duty": target * 1.0e6})
		"reboiler_free":
			if not plant.has("reboiler"):
				_say("this plant's reboiler is not on a key", true)
				return
			_send("free the reboiler: it closes the column at the reflux of %.1f" % _reflux_set(), {"cmd": "set_reboiler_duty", "node": column_id, "duty": null})


## On a plant whose reboiler is on a key, while it is held: the keys that could
## raise the column's edge are held by the screen (§71.11) — the column runs on
## its edge through the walk, and a decline protects only the reboiler's own
## commands. True when `what` was held.
func _held_back(what: String, raises_edge: bool) -> bool:
	if not (PLANTS[plant_key].has("reboiler") and _held() and raises_edge):
		return false
	var text := "%s stays where it is while the reboiler is held: the column runs on its edge, and this move could take the feed's heat below it — free the reboiler first (Y; DESIGN %s)" % [what, PLANTS[plant_key]["range_ref"]]
	_say(text, true)
	if auto_run:
		print("column: t=%d  HELD: %s" % [sim.tick_index(), text])
	return true


func _set_furnace_mw(megawatts: float) -> void:
	if _held_back("the furnace", megawatts < _duty_w() / 1.0e6):
		return
	var limits: Array = PLANTS[plant_key]["furnace_mw"]
	var held := clampf(megawatts, limits[0], limits[1])
	if not is_equal_approx(held, megawatts) and is_equal_approx(held, _duty_w() / 1.0e6):
		_hold("furnace", "%.1f MW" % held)
		return
	_send("fire the furnace at %.1f MW" % held, {"cmd": "set_furnace_duty", "node": furnace_id, "duty": held * 1.0e6})


func _set_reflux(ratio: float) -> void:
	var limits: Array = PLANTS[plant_key]["reflux"]
	var held := clampf(ratio, limits[0], limits[1])
	if not is_equal_approx(held, ratio) and is_equal_approx(held, _reflux_set()):
		_hold("reflux", "%.1f" % held)
		return
	_send("set the reflux ratio to %.1f" % held, {"cmd": "set_reflux_ratio", "node": column_id, "reflux_ratio": held})


## Every pumparound to `scale` times the duty its file declares.
func _set_pumparounds(scale: float) -> void:
	if design_pumparounds.is_empty():
		_say("this column has no pumparounds", true)
		return
	if _held_back("the pumparounds", not is_equal_approx(scale, _pumparound_scale())):
		return
	var limits: Array = PLANTS[plant_key]["pumparounds"]
	var held := clampf(scale, limits[0], limits[1])
	if not is_equal_approx(held, scale) and is_equal_approx(held, _pumparound_scale()):
		_hold("pumparounds", "%.0f %%" % (held * 100.0))
		return
	for i in design_pumparounds.size():
		_send(
			"pumparound on stage %d to %.1f MW" % [int(_pumparounds()[i]["stage"]), float(design_pumparounds[i]) * held / 1.0e6],
			{"cmd": "set_pumparound_duty", "node": column_id, "pumparound": i, "duty": float(design_pumparounds[i]) * held}
		)


## Every pumparound to `megawatts`, on a plant whose keys step them together
## in MW (its file declares them at zero).
func _set_pumparound_mw(megawatts: float) -> void:
	var limits: Array = PLANTS[plant_key]["pumparound_mw"]
	var held := clampf(megawatts, limits[0], limits[1])
	if not is_equal_approx(held, megawatts) and is_equal_approx(held, _pumparound_mw()):
		_hold("pumparounds", "%.1f MW each" % held)
		return
	for i in _pumparounds().size():
		_send(
			"pumparound on stage %d to %.1f MW" % [int(_pumparounds()[i]["stage"]), held],
			{"cmd": "set_pumparound_duty", "node": column_id, "pumparound": i, "duty": held * 1.0e6}
		)


## The stripping steam's valve to `opening` (0–1).
func _set_steam(opening: float) -> void:
	if _held_back("the steam valve", opening < _steam_opening()):
		return
	var limits: Array = PLANTS[plant_key]["steam"]
	var held := clampf(opening, limits[0], limits[1])
	if not is_equal_approx(held, opening) and is_equal_approx(held, _steam_opening()):
		_hold("steam valve", "%.0f %%" % (held * 100.0))
		return
	_send("open the steam valve to %.0f %%" % (held * 100.0), {"cmd": "set_valve_opening", "node": steam_valve_id, "opening": held})


## The screen's own refusal: the key is at the edge of the safe range.
func _hold(what: String, at: String) -> void:
	var ref: String = PLANTS[plant_key].get("range_ref", "§65.9, §66.7")
	var text := "%s stays at %s — the edge of this plant's safe range (DESIGN %s)" % [what, at, ref]
	_say(text, true)
	if auto_run:
		print("column: t=%d  HELD: %s" % [sim.tick_index(), text])


## Send one command and put the engine's answer on screen.
func _send(what: String, command: Dictionary) -> void:
	var text := JSON.stringify(command)
	var err = JSON.parse_string(sim.apply_command(text))
	if err == null:
		_say(what, false)
		if auto_run:
			print("column: t=%d  %s  %s" % [sim.tick_index(), what, text])
	else:
		_say("%s: REFUSED — %s" % [what, err["message"]], true)
		if auto_run:
			print("column: t=%d  %s  REFUSED: %s" % [sim.tick_index(), what, err["message"]])


func _say(text: String, bad: bool) -> void:
	message = text
	message_bad = bad


# ---------------------------------------------------------------- reading

func _node(id: int) -> Dictionary:
	return snapshot["nodes"][id]


func _edge(id: int) -> Dictionary:
	return snapshot["edges"][id]


func _column() -> Dictionary:
	return _node(column_id)


func _cascade() -> Dictionary:
	return _column()["kind"]["cascade"]


func _pumparounds() -> Array:
	return _cascade().get("pumparounds", [])


func _pumparound_scale() -> float:
	if design_pumparounds.is_empty():
		return 1.0
	return float(_pumparounds()[0]["duty"]) / float(design_pumparounds[0])


## The first pumparound's duty [MW]: the keys that step them in MW keep every
## one at the same duty.
func _pumparound_mw() -> float:
	if _pumparounds().is_empty():
		return 0.0
	return float(_pumparounds()[0]["duty"]) / 1.0e6


## Whether the column holds its reboiler (`reboiler_duty_mw`, M64): its
## reflux is then a result, and its declared `reflux_ratio` only a seed.
func _held() -> bool:
	return _cascade().has("reboiler_duty")


func _held_mw() -> float:
	return float(_cascade().get("reboiler_duty", 0.0)) / 1.0e6


## The hold the column declined this tick (M66.2): the duty asked for and the
## column's own reason, or empty while it runs at the duty it holds.
func _declined() -> Dictionary:
	return _column().get("column_reboiler_declined", {})


## Whether the column runs above the reflux it is set to (never on a held one).
func _raised() -> bool:
	return not _held() and _reflux_ran() > _reflux_set() + 1e-9


func _steam_opening() -> float:
	return float(_node(steam_valve_id)["kind"]["opening"])


func _steam_kg_s() -> float:
	return float(_edge(steam_pipe_id)["stream"]["mass_flow"])


func _steam_k() -> float:
	return float(_edge(steam_pipe_id)["stream"]["temperature"])


func _operation() -> Dictionary:
	# Absent before the first tick and while the column is idle.
	return _column().get("column_operation", {})


func _duties() -> Dictionary:
	return _column().get("column_duty", {})


func _duty_w() -> float:
	return float(_node(furnace_id)["kind"]["duty"])


func _coil_k() -> float:
	return float(_node(furnace_id)["kind"]["coil"]["temperature"])


func _failure_k() -> float:
	return float(_node(furnace_id)["kind"]["tubes"]["failure_temperature"])


func _reflux_set() -> float:
	return float(_cascade()["reflux_ratio"])


func _reflux_ran() -> float:
	return float(_operation().get("reflux_ratio", _reflux_set()))


func _flash() -> float:
	return float(_operation().get("feed_vapour_fraction", 0.0))


func _feed_k() -> float:
	return float(_edge(feed_pipe_id)["stream"]["temperature"])


func _feed_kg_s() -> float:
	return float(_edge(feed_pipe_id)["stream"]["mass_flow"])


func _stage_temperatures() -> Array:
	return _operation().get("stage_temperatures_k", [])


func _stage_vapour() -> Array:
	return _operation().get("stage_vapour_mol_s", [])


func _c(kelvin: float) -> String:
	return "%.1f °C" % (kelvin - 273.15)


func _mw(field: String) -> float:
	return float(_duties().get(field, 0.0)) / 1.0e6


func _readout(tick: int) -> String:
	var pumparounds := "none"
	if PLANTS[plant_key].has("pumparound_mw"):
		pumparounds = "%.1f MW each" % _pumparound_mw()
	elif not design_pumparounds.is_empty():
		pumparounds = "%.0f%%" % (_pumparound_scale() * 100.0)
	var line := (
		"t=%5d  furnace=%6.2f MW  feed=%6.1f C  flash=%5.1f%%  reflux %s  condenser=%6.2f MW  reboiler=%6.2f MW  pumparounds=%s"
		% [
			tick,
			_duty_w() / 1.0e6,
			_feed_k() - 273.15,
			_flash() * 100.0,
			"ran=%.3f (a result)" % _reflux_ran() if _held() else "set=%.2f ran=%.3f" % [_reflux_set(), _reflux_ran()],
			_mw("condenser_w"),
			_mw("reboiler_w"),
			pumparounds,
		]
	)
	if steam_valve_id >= 0:
		var stages := _stage_temperatures()
		line += "  steam=%.0f%% %.3f kg/s  bottom=%.1f C" % [
			_steam_opening() * 100.0, _steam_kg_s(), float(stages[-1]) - 273.15 if not stages.is_empty() else 0.0
		]
	if PLANTS[plant_key].has("reboiler"):
		if not _held():
			line += "  reboiler free"
		elif _declined().is_empty():
			line += "  reboiler held"
		else:
			line += "  reboiler DECLINED %.2f MW" % (float(_declined()["asked_w"]) / 1.0e6)
	return line


# ---------------------------------------------------------------- drawing

const FURNACE_RECT := Rect2(40, 330, 150, 170)
const COLUMN_RECT := Rect2(330, 130, 100, 380)
const PRODUCT_X := 520.0
const BAR_WIDTH := 140.0
const PANEL_X := 800.0

const BACKGROUND := Color(0.09, 0.10, 0.12)
const SHELL := Color(0.42, 0.45, 0.50)
const PIPE := Color(0.55, 0.58, 0.62)
const COOL := Color(0.25, 0.50, 0.85)
const HOT := Color(0.90, 0.30, 0.15)
const FLAME := Color(1.0, 0.55, 0.10)
const VAPOUR := Color(0.85, 0.85, 0.95, 0.55)
const COOLER := Color(0.40, 0.75, 0.95)
const INK := Color(0.88, 0.90, 0.93)
const DIM := Color(0.60, 0.63, 0.68)
const GOOD := Color(0.35, 0.80, 0.45)
const BAD := Color(0.95, 0.30, 0.25)
const WARN := Color(0.95, 0.78, 0.30)
## One colour per cut of the slate, lightest first.
const CUT_COLORS := [
	Color(0.95, 0.90, 0.45),
	Color(0.95, 0.70, 0.30),
	Color(0.55, 0.85, 0.55),
	Color(0.40, 0.70, 0.95),
	Color(0.65, 0.50, 0.90),
	Color(0.55, 0.40, 0.35),
	# Water, the seventh member on a wet plant's slate (M62).
	Color(0.30, 0.85, 0.85),
]
const STEAM := Color(0.92, 0.92, 0.98)
## The stage colour scale [K]: cool at the top of a crude column, hot at its foot.
const STAGE_COLD_K := 373.15
const STAGE_HOT_K := 873.15


func _draw() -> void:
	draw_rect(Rect2(Vector2.ZERO, get_viewport_rect().size), BACKGROUND)
	if halted != "" and snapshot.is_empty():
		_text(Vector2(30, 40), "HALTED — %s" % halted, BAD)
		return
	if snapshot.is_empty():
		return

	_text(Vector2(30, 34), PLANTS[plant_key]["title"], INK, 20)
	_text(
		Vector2(30, 60),
		"t = %d s    %d tick(s) per frame%s" % [sim.tick_index(), ticks_per_frame, "    PAUSED" if paused else ""],
		DIM
	)

	_draw_furnace()
	_draw_column()
	_draw_products()
	_draw_panel()

	draw_multiline_string(
		ThemeDB.fallback_font, Vector2(30, 556), message, HORIZONTAL_ALIGNMENT_LEFT,
		get_viewport_rect().size.x - 60, 15, 2, BAD if message_bad else GOOD
	)
	var keys := "Up/Down furnace   W/S reflux   E/D pumparounds   Space pause   [ ] speed"
	if PLANTS[plant_key].has("reboiler"):
		keys = "Up/Down furnace   R/F steam   E/D pumparounds   T/G reboiler   Y free it   Space pause   [ ] speed"
	elif steam_valve_id >= 0:
		keys = "Up/Down furnace   R/F steam   E/D pumparounds   Space pause   [ ] speed"
	_text(Vector2(30, 600), keys, DIM)
	_text(Vector2(30, 624), "1 crude unit   2 hot feed   3 overheated   4 stripped by steam   5 cold start-up", DIM)
	if halted != "":
		draw_multiline_string(
			ThemeDB.fallback_font, Vector2(30, 90), "HALTED — %s" % halted, HORIZONTAL_ALIGNMENT_LEFT,
			get_viewport_rect().size.x - 60, 15, 3, BAD
		)


## The furnace: flames sized by its duty, the coil coloured toward its tube limit.
func _draw_furnace() -> void:
	draw_rect(FURNACE_RECT, Color(0.13, 0.12, 0.12))
	draw_rect(FURNACE_RECT, SHELL, false, 3.0)
	var mw := _duty_w() / 1.0e6
	if mw > 0.0:
		var height := minf(FURNACE_RECT.size.y - 30, 15.0 + sqrt(mw) * 14.0)
		var base := FURNACE_RECT.position + Vector2(20, FURNACE_RECT.size.y)
		var step := (FURNACE_RECT.size.x - 40) / 4.0
		for i in 4:
			var x := base.x + step * (i + 0.5)
			draw_colored_polygon(
				PackedVector2Array([Vector2(x - step * 0.4, base.y), Vector2(x, base.y - height), Vector2(x + step * 0.4, base.y)]),
				FLAME
			)
	var coil := _coil_k()
	var coil_color := COOL.lerp(HOT, clampf((coil - 293.15) / (_failure_k() - 293.15), 0.0, 1.0))
	for row in 4:
		var y := FURNACE_RECT.position.y + 25 + row * 16.0
		draw_line(Vector2(FURNACE_RECT.position.x + 20, y), Vector2(FURNACE_RECT.end.x - 20, y), coil_color, 4.0)
	_text(FURNACE_RECT.position + Vector2(0, -26), "furnace  %.1f MW" % mw, FLAME if mw > 0.0 else DIM)
	_text(FURNACE_RECT.position + Vector2(0, -8), "coil %s (tubes fail %s)" % [_c(coil), _c(_failure_k())], DIM, 13)

	# The transfer line, into the feed stage.
	var feed_y := _stage_y(int(_cascade()["feed_stage"]) - 1)
	var out := Vector2(FURNACE_RECT.end.x, FURNACE_RECT.position.y + 40)
	draw_polyline(PackedVector2Array([out, Vector2(260, out.y), Vector2(260, feed_y), Vector2(COLUMN_RECT.position.x, feed_y)]), PIPE, 6.0)
	_text(FURNACE_RECT.position + Vector2(0, FURNACE_RECT.size.y + 20), "feed %s, %.0f kg/s" % [_c(_feed_k()), _feed_kg_s()], INK, 14)
	_text(FURNACE_RECT.position + Vector2(0, FURNACE_RECT.size.y + 38), "%.0f %% flashes in the column" % (_flash() * 100.0), WARN if _flash() > 0.0 else DIM, 13)


func _stage_count() -> int:
	return int(_cascade()["stages"])


## The centre of stage `j` (zero-based from the top) on the column.
func _stage_y(j: int) -> float:
	var pitch := COLUMN_RECT.size.y / _stage_count()
	return COLUMN_RECT.position.y + (j + 0.5) * pitch


## The column: one band per stage coloured by its temperature, the vapour each
## stage sends up as a bar beside it, the condenser and reflux above, the
## reboiler below, and each pumparound as a cooler on its stage.
func _draw_column() -> void:
	var temperatures := _stage_temperatures()
	var vapour := _stage_vapour()
	var stages := _stage_count()
	var pitch := COLUMN_RECT.size.y / stages
	var feed_molar := 1.0
	for v in vapour:
		feed_molar = maxf(feed_molar, float(v))
	for j in stages:
		var band := Rect2(COLUMN_RECT.position + Vector2(0, j * pitch), Vector2(COLUMN_RECT.size.x, pitch))
		var color := Color(0.18, 0.19, 0.22)
		if j < temperatures.size():
			color = COOL.lerp(HOT, clampf((float(temperatures[j]) - STAGE_COLD_K) / (STAGE_HOT_K - STAGE_COLD_K), 0.0, 1.0))
		draw_rect(band, color)
		if j < temperatures.size():
			draw_rect(Rect2(band.position + Vector2(18, pitch * 0.5 - 9), Vector2(64, 16)), Color(0.05, 0.05, 0.07, 0.6))
			_text(band.position + Vector2(22, pitch * 0.5 + 4), _c(float(temperatures[j])), INK, 12)
		draw_line(band.position, band.position + Vector2(band.size.x, 0), SHELL, 1.0)
		if j < vapour.size():
			var width := 40.0 * float(vapour[j]) / feed_molar
			draw_rect(Rect2(Vector2(COLUMN_RECT.position.x - 6 - width, band.position.y + pitch * 0.3), Vector2(width, pitch * 0.4)), VAPOUR)
	draw_rect(COLUMN_RECT, SHELL, false, 3.0)
	_text(COLUMN_RECT.position + Vector2(-64, COLUMN_RECT.size.y + 14), "vapour up", DIM, 12)

	# Condenser and reflux.
	var top := COLUMN_RECT.position + Vector2(COLUMN_RECT.size.x * 0.5, 0)
	var condenser := top + Vector2(0, -40)
	draw_line(top, condenser, PIPE, 4.0)
	draw_circle(condenser, 14, COOLER)
	var raised := _raised()
	_text(condenser + Vector2(-250, -4), "condenser %.1f MW" % _mw("condenser_w"), INK, 14)
	var reflux_label := "reflux %.2f%s" % [_reflux_ran(), "  RAISED (set %.1f)" % _reflux_set() if raised else ""]
	if _held():
		reflux_label = "reflux %.2f, a result" % _reflux_ran()
	_text(condenser + Vector2(-250, 14), reflux_label, WARN if raised else DIM, 14)

	var foot := COLUMN_RECT.position + Vector2(COLUMN_RECT.size.x * 0.5, COLUMN_RECT.size.y)
	if steam_pipe_id >= 0:
		# The stripping steam, into the bottom stage from the left.
		var bottom_y := _stage_y(_stage_count() - 1)
		var from := Vector2(FURNACE_RECT.end.x + 6, bottom_y)
		draw_line(from, Vector2(COLUMN_RECT.position.x, bottom_y), STEAM, 3.0)
		_text(from + Vector2(0, -22), "steam %.2f kg/s" % _steam_kg_s(), STEAM, 13)
		_text(from + Vector2(0, -7), "%s, valve %.0f %%" % [_c(_steam_k()), _steam_opening() * 100.0], DIM, 11)
	# A column held at zero has no reboiler to draw: its bottom stage's
	# balance is held at zero (the panel says so). One held above zero —
	# partway through a start-up — still has one, at the duty it holds.
	if not _held() or _mw("reboiler_w") > 0.0:
		var reboiler := Rect2(foot + Vector2(-30, 10), Vector2(60, 26))
		draw_line(foot, reboiler.position + Vector2(30, 0), PIPE, 4.0)
		draw_rect(reboiler, Color(0.35, 0.18, 0.10))
		draw_rect(reboiler, SHELL, false, 2.0)
		_text(reboiler.position + Vector2(-60, 46), "reboiler %.1f MW%s" % [_mw("reboiler_w"), "  held" if _held() else ""], INK, 14)

	# Pumparounds, left of the column at their stages.
	for pumparound in _pumparounds():
		var y := _stage_y(int(pumparound["stage"]) - 1)
		var at := Vector2(COLUMN_RECT.end.x + 2, y)
		draw_line(at, at + Vector2(10, 0), COOLER, 3.0)
		draw_circle(at + Vector2(16, 0), 6, COOLER)
		_text(at + Vector2(26, 16), "%.1f MW off" % (float(pumparound["duty"]) / 1.0e6), COOLER, 11)


## Each product: its pipe from its stage, its flow and temperature, and its
## composition as a bar of the slate's cuts.
func _draw_products() -> void:
	var plant: Dictionary = PLANTS[plant_key]
	var draws: Array = _column()["kind"]["draws"]
	var slate: Array = snapshot["slate"]
	for i in draw_ids.size():
		var edge := _edge(draw_ids[i])
		var stage := int(draws[i]["stage"])
		var y: float
		if stage == 0 and draws[i].get("water", false):
			# The decanter's free water, above the naphtha drawn beside it.
			y = COLUMN_RECT.position.y - 72
		elif stage == 0:
			y = COLUMN_RECT.position.y - 40
		elif stage >= _stage_count():
			y = COLUMN_RECT.end.y + 23
		else:
			y = _stage_y(stage - 1)
		var from := Vector2(COLUMN_RECT.end.x + (14 if stage == 0 else 0), y)
		draw_line(from, Vector2(PRODUCT_X - 8, y), PIPE, 3.0)
		var fractions: Array = edge["stream"]["composition"]["mass_fractions"]
		var x := PRODUCT_X
		for c in fractions.size():
			var width := BAR_WIDTH * float(fractions[c])
			draw_rect(Rect2(Vector2(x, y - 8), Vector2(width, 16)), CUT_COLORS[c % CUT_COLORS.size()])
			x += width
		draw_rect(Rect2(Vector2(PRODUCT_X, y - 8), Vector2(BAR_WIDTH, 16)), SHELL, false, 1.0)
		_text(Vector2(PRODUCT_X + BAR_WIDTH + 8, y - 2), plant["draws"][i][1], INK, 13)
		_text(
			Vector2(PRODUCT_X + BAR_WIDTH + 8, y + 12),
			"%.1f kg/s  %s" % [float(edge["stream"]["mass_flow"]), _c(float(edge["stream"]["temperature"]))],
			DIM,
			12
		)
	# The legend: the slate's cuts, lightest first, three to a row.
	for c in slate.size():
		var at := Vector2(PANEL_X + (c % 3) * 112, 84 + (c / 3) * 16)
		draw_rect(Rect2(at, Vector2(12, 12)), CUT_COLORS[c % CUT_COLORS.size()])
		_text(at + Vector2(16, 11), str(slate[c]["name"]).replace("_", " "), DIM, 11)


## The right-hand panel: the column's books.
func _draw_panel() -> void:
	# Below the slate's legend, three cuts to a row.
	var y := 108.0 + ceili(snapshot["slate"].size() / 3.0) * 16.0
	_text(Vector2(PANEL_X, y), "COLUMN", DIM)
	y += 24
	_text(Vector2(PANEL_X, y), "feed            %s" % _c(_feed_k()), INK)
	y += 22
	_text(Vector2(PANEL_X, y), "flashes          %5.1f %%" % (_flash() * 100.0), WARN if _flash() > 0.0 else INK)
	y += 22
	var raised := _raised()
	if _held():
		_text(Vector2(PANEL_X, y), "reflux ran      %6.3f  (a result)" % _reflux_ran(), INK)
		y += 18
		if _mw("reboiler_w") > 0.0:
			_text(Vector2(PANEL_X + 12, y), "reboiler held: the heat sets the reflux", DIM, 13)
		else:
			_text(Vector2(PANEL_X + 12, y), "no reboiler: the feed's heat sets it", DIM, 13)
	else:
		_text(Vector2(PANEL_X, y), "reflux set      %6.2f" % _reflux_set(), INK)
		y += 22
		_text(Vector2(PANEL_X, y), "reflux ran      %6.3f" % _reflux_ran(), WARN if raised else INK)
	if raised:
		y += 18
		_text(Vector2(PANEL_X + 12, y), "the feed brings more heat than", WARN, 13)
		y += 16
		_text(Vector2(PANEL_X + 12, y), "the setting carries out: it rose", WARN, 13)
	y += 22
	_text(Vector2(PANEL_X, y), "condenser      %6.2f MW" % _mw("condenser_w"), INK)
	y += 22
	if _held():
		_text(Vector2(PANEL_X, y), "reboiler       %6.2f MW  held" % _mw("reboiler_w"), INK)
	else:
		_text(Vector2(PANEL_X, y), "reboiler       %6.2f MW  free" % _mw("reboiler_w") if PLANTS[plant_key].has("reboiler") else "reboiler       %6.2f MW" % _mw("reboiler_w"), INK)
	y += 22
	# A hold the column declined (M66.2): it keeps the duty it ran at.
	if not _declined().is_empty():
		_text(Vector2(PANEL_X + 12, y - 4), "asked %.2f MW: DECLINED, kept the duty it ran at" % (float(_declined()["asked_w"]) / 1.0e6), WARN, 13)
		y += 18
	if steam_valve_id >= 0:
		_text(Vector2(PANEL_X, y), "steam          %6.3f kg/s (%.0f %%)" % [_steam_kg_s(), _steam_opening() * 100.0], STEAM)
		y += 22
		var stages := _stage_temperatures()
		if not stages.is_empty():
			_text(Vector2(PANEL_X, y), "bottom          %s" % _c(float(stages[-1])), INK)
			y += 22
	var total := 0.0
	for pumparound in _pumparounds():
		total += float(pumparound["duty"])
	if PLANTS[plant_key].has("pumparound_mw"):
		_text(Vector2(PANEL_X, y), "pumparounds    %6.2f MW (%.1f each)" % [total / 1.0e6, _pumparound_mw()], COOLER)
		y += 22
	elif not _pumparounds().is_empty():
		_text(Vector2(PANEL_X, y), "pumparounds    %6.2f MW (%.0f %%)" % [total / 1.0e6, _pumparound_scale() * 100.0], COOLER)
		y += 22

	y += 18
	_text(Vector2(PANEL_X, y), "FURNACE", DIM)
	y += 24
	_text(Vector2(PANEL_X, y), "fired          %6.2f MW" % (_duty_w() / 1.0e6), FLAME if _duty_w() > 0.0 else INK)
	y += 22
	_text(Vector2(PANEL_X, y), "up the stack   %6.2f MW" % (float(_node(furnace_id).get("flue_loss_w", 0.0)) / 1.0e6), INK)
	y += 22
	_text(Vector2(PANEL_X, y), "coil           %s" % _c(_coil_k()), INK)

	y += 40
	_text(Vector2(PANEL_X, y), "SAFE RANGE OF THE KEYS", DIM)
	y += 22
	var plant: Dictionary = PLANTS[plant_key]
	var low: float = plant["furnace_mw"][0]
	_text(Vector2(PANEL_X, y), ("furnace  %.0f–%.0f MW" if low == roundf(low) else "furnace  %.1f–%.0f MW") % [low, plant["furnace_mw"][1]], DIM, 14)
	if plant.has("reflux"):
		y += 18
		_text(Vector2(PANEL_X, y), "reflux   %.1f–%.1f" % [plant["reflux"][0], plant["reflux"][1]], DIM, 14)
	if plant.has("steam"):
		y += 18
		_text(Vector2(PANEL_X, y), "steam valve  %.0f–%.0f %%" % [plant["steam"][0] * 100.0, plant["steam"][1] * 100.0], DIM, 14)
	if plant.has("pumparound_mw"):
		y += 18
		_text(Vector2(PANEL_X, y), "pumparounds  %.0f–%.0f MW each" % [plant["pumparound_mw"][0], plant["pumparound_mw"][1]], DIM, 14)
	if plant.has("pumparounds"):
		y += 18
		_text(
			Vector2(PANEL_X, y),
			"pumparounds  %.0f–%.0f %%" % [plant["pumparounds"][0] * 100.0, plant["pumparounds"][1] * 100.0],
			DIM,
			14
		)
	if plant.has("reboiler"):
		y += 18
		_text(Vector2(PANEL_X, y), "reboiler held: furnace and steam up only,", DIM, 14)
		y += 18
		_text(Vector2(PANEL_X, y), "  the pumparounds where they are", DIM, 14)


func _text(at: Vector2, text: String, color: Color, size: int = 16) -> void:
	draw_string(ThemeDB.fallback_font, at, text, HORIZONTAL_ALIGNMENT_LEFT, -1, size, color)


func _halt(reason: String) -> void:
	halted = reason
	push_error("column: %s" % reason)
	print("column: HALTED — %s" % reason)
