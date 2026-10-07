# M51's demonstrated criterion: a screen for M50, what cavitation does to a pump.
#
# M50 made a pump lose push as its suction nears the liquid's boiling pressure;
# it ran only in tests and the CLI. This scene draws it, on either of two
# shipped plants:
#
#   limit    scenarios/pump_cavitation_flow_limit.toml (M50) — hot naphtha drawn
#            through a long, narrow suction line. Opening the valve buys almost
#            no flow: the pump answers more opening with less push.
#   boiling  scenarios/cavitating_pump.toml (M11) — the same naphtha lifted 25 m
#            to a pump with no suction model. Its suction is below the boiling
#            pressure the whole run and it keeps its whole push: the "before".
#
# Same rule as plant.gd and furnace.gd: this script computes nothing physical.
# Every number it draws is a snapshot field, every marker is a value the
# snapshot carries (the boiling pressure, the pump's NPSH3), and the only
# arithmetic is display units — pascals to bar, fractions to percent.
#
# The plants have no tanks, so each settles on the tick after a change and a
# screen of current readings would sit still. The TRAIL is the screen's memory:
# one dot per valve opening visited, at the flow and push the engine reported
# there (the latest tick at that opening, so tick 1's whole-curve guess is
# overwritten by tick 2). Drawing it is plotting reported numbers, not a curve.
#
# M52 adds the operator's real cure: the supply's pressure and temperature, and
# the destination's pressure, by command. The trail keys each dot by the supply
# and destination it was read under; dots from earlier conditions stay, faded,
# so cooling the supply shows as the whole curve moving.
#
# Interactive:  godot --path . res://demo/pump.tscn
#               (keys listed on screen and in _input; 1 / 2 switch plants)
# Recorded:     godot --headless --path . res://demo/pump.tscn --quit-after 20000 -- --auto [--plant=boiling]
#               ...runs that plant's scripted timeline (TIMELINES) and prints a
#               `t=` line at a fixed interval plus one line per command. Those
#               lines are the observation in ROADMAP.md's M51 section.
# Screenshots:  add --shots=<dir> to a WINDOWED --auto run (headless renders
#               nothing) to save a PNG at each scripted tick in AUTO_SHOT_TICKS.
extends Node2D

## The two plants, and the names on each the scene draws. A name missing from a
## plant is a load-time halt, not a guess.
const PLANTS := {
	"limit":
	{
		"scenario": "res://scenarios/pump_cavitation_flow_limit.toml",
		"title": "Hot naphtha pump on a long suction line",
		"source": "rundown_source",
		"suction_pipe": "suction_line",
		"pump": "feed_pump",
		"valve": "discharge_valve",
		"destination": "unit_feed",
	},
	"boiling":
	{
		"scenario": "res://scenarios/cavitating_pump.toml",
		"title": "The M11 pump: boiling, at full push",
		"source": "rundown_source",
		"suction_pipe": "lift_line",
		"pump": "suction",
		"valve": "discharge_valve",
		"destination": "unit_feed",
	},
}

## Scripted timelines for the recorded runs, in ticks. Each tick is one the
## bridge test `crates/godot-ext/tests/pump_screen.rs` replays with the same
## commands and pins the outcome of.
##
## limit: from the file's 0.6 the valve is throttled to 0.2 (whole push back),
## then opened a step at a time to 1.0 — the flow rises 6.8 -> 11.3 kg/s while
## the push falls to 5%. Throttled back to 0.2 the push returns (a move Newton
## gave up on until M51). At 160 the pump is stopped on the file's 0.6: the
## supply still drives 8.2 kg/s through it, so the pump was adding under 3 kg/s.
## Restarted at 180.
##
## M52, from 200 on the file's 0.6: the supply cooled to 90 °C gives the pump
## nearly its whole push back (16.3 kg/s); warmed back to 110 °C it is where it
## was; the supply raised to 3.0 bar gives back two thirds (15.8 kg/s); back to
## 2.4 bar, then heated to 125 °C — REFUSED, the supply itself would boil; the
## destination raised to 2.0 bar cuts the flow by back-pressure.
const AUTO_LIMIT := {
	20: ["valve_0.2"],
	40: ["valve_0.3"],
	60: ["valve_0.4"],
	80: ["valve_0.6"],
	100: ["valve_0.8"],
	120: ["valve_1.0"],
	140: ["valve_0.2"],
	160: ["valve_0.6", "pump_off"],
	180: ["pump_on"],
	200: ["supply_c_90"],
	220: ["supply_c_110"],
	240: ["supply_bar_3.0"],
	260: ["supply_bar_2.4", "supply_c_125"],
	280: ["destination_bar_2.0"],
	300: ["quit"],
}
## boiling: the same sweep on the M11 pump. The flow follows the valve all the
## way (3.7 -> 17.1 kg/s) with the boiling lamp lit throughout: nothing happens
## to a pump with no suction model. Stopped at 100, the destination drives the
## flow backwards.
const AUTO_BOILING := {
	20: ["valve_0.2"],
	40: ["valve_0.4"],
	60: ["valve_0.8"],
	80: ["valve_1.0"],
	100: ["pump_off"],
	120: ["pump_on", "valve_0.6"],
	140: ["quit"],
}
const TIMELINES := {"limit": AUTO_LIMIT, "boiling": AUTO_BOILING}
const AUTO_SHOT_TICKS := {
	"limit": [1, 39, 79, 139, 159, 179, 199, 219, 259, 299],
	"boiling": [19, 79, 119],
}
## Print a `t=` line every this many ticks in a recorded run.
const PRINT_EVERY := 10

## The valve keys' step, as a fraction of full opening.
const VALVE_STEP := 0.05
## The supply and destination keys' steps: 0.1 bar [Pa] and 5 K.
const PRESSURE_STEP_PA := 1.0e4
const TEMPERATURE_STEP_K := 5.0
const MAX_TICKS_PER_FRAME := 64

@onready var sim: RefinerySim = $Sim

var plant_key := "limit"
var source_id := -1
var suction_pipe_id := -1
var pump_id := -1
var valve_id := -1
var destination_id := -1
var snapshot: Dictionary = {}
## What the screen has seen: "<opening %>|<on|off>" -> {opening, flow, head}.
## `head` is null where the snapshot reported no push (pump stopped, first tick,
## a pump with no suction model).
var trail: Dictionary = {}
var halted := ""
var auto_run := false
var shots_dir := ""
var paused := false
var finished := false
var shooting := false
var ticks_per_frame := 1
var message := ""
var message_bad := false
var seen_lamp := ""
## A command has moved the supply or the destination from the file's values;
## from then on the recorded lines carry them (M52).
var moved_conditions := false


func _ready() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg == "--auto":
			auto_run = true
		elif arg.begins_with("--plant="):
			plant_key = arg.trim_prefix("--plant=")
		elif arg.begins_with("--shots="):
			shots_dir = arg.trim_prefix("--shots=")
	if not PLANTS.has(plant_key):
		_halt("no plant '%s' (limit | boiling)" % plant_key)
		return
	_load(plant_key)


func _load(key: String) -> void:
	plant_key = key
	halted = ""
	finished = false
	snapshot = {}
	trail = {}
	seen_lamp = ""
	moved_conditions = false
	var plant: Dictionary = PLANTS[key]

	var err = JSON.parse_string(sim.load_scenario(plant["scenario"]))
	if err != null:
		_halt("load failed: %s" % err)
		return

	source_id = sim.node_id(plant["source"])
	pump_id = sim.node_id(plant["pump"])
	valve_id = sim.node_id(plant["valve"])
	destination_id = sim.node_id(plant["destination"])
	suction_pipe_id = sim.edge_id(plant["suction_pipe"])
	if source_id < 0 or pump_id < 0 or valve_id < 0 or destination_id < 0 or suction_pipe_id < 0:
		_halt("this scenario does not have the names the scene expects")
		return

	snapshot = JSON.parse_string(sim.snapshot_json())
	_say("loaded %s" % plant["scenario"], false)
	print("pump: loaded %s" % plant["scenario"])
	queue_redraw()


func _physics_process(_delta: float) -> void:
	if halted != "" or paused or finished or shooting:
		return
	for i in ticks_per_frame:
		if not _step():
			return
	queue_redraw()


## One engine tick, then the scripted timeline for it. False when the scene
## must stop ticking this frame (a halt, a screenshot, the end of the run).
func _step() -> bool:
	var err = JSON.parse_string(sim.tick())
	if err != null:
		_halt("tick %d: %s" % [sim.tick_index(), err["message"]])
		return false
	snapshot = JSON.parse_string(sim.snapshot_json())
	var tick := sim.tick_index()
	_remember()
	_report_events(tick)

	if not auto_run:
		return true
	if tick % PRINT_EVERY == 0 or TIMELINES[plant_key].has(tick):
		print(_readout(tick))
	var timeline: Dictionary = TIMELINES[plant_key]
	if timeline.has(tick):
		for action in timeline[tick]:
			if action == "quit":
				finished = true
				get_tree().quit()
				return false
			_do(action)
	if shots_dir != "" and tick in AUTO_SHOT_TICKS[plant_key]:
		_shoot(tick)
		return false
	return true


## Put this tick's reading on the trail, replacing what was there for the same
## opening, pump state and supply/destination conditions.
func _remember() -> void:
	var key := "%s|%d|%s" % [_conditions(), roundi(_opening() * 100.0), "on" if _pump_on() else "off"]
	trail[key] = {
		"opening": _opening(),
		"flow": _flow(),
		"head": _head_fraction(),
		"on": _pump_on(),
		"conditions": _conditions(),
	}


## The supply and destination a reading was taken under, as the trail keys it.
func _conditions() -> String:
	return "%.2f|%.1f|%.2f" % [_source_pa() / 1.0e5, _source_k() - 273.15, _destination_pa() / 1.0e5]


## Print a line when the boiling lamp changes — the engine's account, not ours.
func _report_events(tick: int) -> void:
	var lamp := _lamp_label()
	if seen_lamp != "" and lamp != seen_lamp:
		print("pump: t=%d  boiling lamp -> %s" % [tick, lamp])
	seen_lamp = lamp


## Ticking holds until the frame showing `tick` has been drawn and saved.
func _shoot(tick: int) -> void:
	shooting = true
	queue_redraw()
	await RenderingServer.frame_post_draw
	var path := "%s/pump_%s_t%04d.png" % [shots_dir, plant_key, tick]
	var image := get_viewport().get_texture().get_image()
	if image != null:
		image.save_png(path)
		print("pump: saved %s" % path)
	shooting = false


func _input(event: InputEvent) -> void:
	if not (event is InputEventKey and event.pressed and not event.echo):
		return
	match event.keycode:
		KEY_1:
			_load("limit")
		KEY_2:
			_load("boiling")
		KEY_SPACE:
			paused = not paused
		KEY_BRACKETLEFT:
			ticks_per_frame = maxi(1, ticks_per_frame / 2)
		KEY_BRACKETRIGHT:
			ticks_per_frame = mini(MAX_TICKS_PER_FRAME, ticks_per_frame * 2)
		KEY_UP:
			_do("valve_up")
		KEY_DOWN:
			_do("valve_down")
		KEY_K:
			_do("toggle_pump")
		KEY_W:
			_do("supply_up")
		KEY_S:
			_do("supply_down")
		KEY_E:
			_do("warmer")
		KEY_D:
			_do("cooler")
		KEY_R:
			_do("destination_up")
		KEY_F:
			_do("destination_down")
		KEY_C:
			trail = {}
			_say("trail cleared", false)
	queue_redraw()


# --------------------------------------------------------------- commands

## Every action the keys and the timelines share. Each sends the contract's own
## JSON; ids are cast to int, or serde would refuse `0.0` for an id.
func _do(action: String) -> void:
	if action.begins_with("valve_") and action.trim_prefix("valve_").is_valid_float():
		_set_opening(float(action.trim_prefix("valve_")))
		return
	# The scripted forms: a value in display units, converted at this boundary.
	if action.begins_with("supply_bar_"):
		_set_pressure(source_id, "source", float(action.trim_prefix("supply_bar_")) * 1.0e5)
		return
	if action.begins_with("supply_c_"):
		_set_supply_temperature(float(action.trim_prefix("supply_c_")) + 273.15)
		return
	if action.begins_with("destination_bar_"):
		_set_pressure(destination_id, "destination", float(action.trim_prefix("destination_bar_")) * 1.0e5)
		return
	match action:
		# Each step is snapped to the step, so a run of key presses lands on
		# round values. The engine refuses what it must (a boiling supply, a
		# pressure at or below zero) and the screen shows its reason.
		"supply_up", "supply_down":
			var step := PRESSURE_STEP_PA if action == "supply_up" else -PRESSURE_STEP_PA
			_set_pressure(source_id, "source", snappedf(_source_pa() + step, PRESSURE_STEP_PA))
		"destination_up", "destination_down":
			var step := PRESSURE_STEP_PA if action == "destination_up" else -PRESSURE_STEP_PA
			_set_pressure(destination_id, "destination", snappedf(_destination_pa() + step, PRESSURE_STEP_PA))
		"warmer", "cooler":
			var step := TEMPERATURE_STEP_K if action == "warmer" else -TEMPERATURE_STEP_K
			# Snapped in °C, so 110 °C steps to 105 and 115.
			_set_supply_temperature(snappedf(_source_k() - 273.15 + step, TEMPERATURE_STEP_K) + 273.15)
		"valve_up", "valve_down":
			var step := VALVE_STEP if action == "valve_up" else -VALVE_STEP
			# Snapped to the step so the trail's dots line up; clamped to the
			# valve's own range, which the engine would refuse past.
			_set_opening(clampf(snappedf(_opening() + step, VALVE_STEP), 0.0, 1.0))
		"pump_on", "pump_off", "toggle_pump":
			var on := action == "pump_on" or (action == "toggle_pump" and not _pump_on())
			_send(
				"%s %s" % ["start" if on else "stop", PLANTS[plant_key]["pump"]],
				{"cmd": "set_pump_on", "node": int(pump_id), "on": on}
			)


## `which` is the PLANTS key naming the node ("source" or "destination").
func _set_pressure(id: int, which: String, pressure_pa: float) -> void:
	var sent := _send(
		"%s to %.2f bar" % [PLANTS[plant_key][which], pressure_pa / 1.0e5],
		{"cmd": "set_reservoir_pressure", "node": int(id), "pressure": pressure_pa}
	)
	moved_conditions = moved_conditions or sent


func _set_supply_temperature(temperature_k: float) -> void:
	var sent := _send(
		"%s to %.0f °C" % [PLANTS[plant_key]["source"], temperature_k - 273.15],
		{"cmd": "set_source_temperature", "node": int(source_id), "temperature": temperature_k}
	)
	moved_conditions = moved_conditions or sent


func _set_opening(opening: float) -> void:
	_send(
		"%s to %.0f%% open" % [PLANTS[plant_key]["valve"], opening * 100.0],
		{"cmd": "set_valve_opening", "node": int(valve_id), "opening": opening}
	)


## Send one command and put the engine's answer on screen — its own refusal
## message when it says no. True when the engine took it.
func _send(what: String, command: Dictionary) -> bool:
	var text := JSON.stringify(command)
	var err = JSON.parse_string(sim.apply_command(text))
	if err == null:
		_say(what, false)
		if auto_run:
			print("pump: t=%d  %s  %s" % [sim.tick_index(), what, text])
		# A command moves the kind at once; read it back so the screen shows
		# the new value before the next tick.
		snapshot = JSON.parse_string(sim.snapshot_json())
		return true
	_say("%s: REFUSED — %s" % [what, err["message"]], true)
	if auto_run:
		print("pump: t=%d  %s  REFUSED: %s" % [sim.tick_index(), what, err["message"]])
	return false


func _say(text: String, bad: bool) -> void:
	message = text
	message_bad = bad


# ---------------------------------------------------------------- reading

## Field lookups only. `cavitation` is absent where thermo has no boiling
## pressure, `pump_suction` wherever the pump is stopped, has no
## `npsh_required_m`, or has not yet been handed its liquid's boiling pressure
## (tick 1) — hence `get` and the nulls.

func _node(id: int) -> Dictionary:
	return snapshot["nodes"][id]


func _pump_kind() -> Dictionary:
	return _node(pump_id)["kind"]


func _pump_on() -> bool:
	return bool(_pump_kind()["on"])


func _opening() -> float:
	return float(_node(valve_id)["kind"]["opening"])


## Through the suction line [kg/s], signed by its declared direction.
func _flow() -> float:
	return float(snapshot["edges"][suction_pipe_id]["stream"]["mass_flow"])


## The pump's suction pressure [Pa]: the pump node's own pressure.
func _suction_pa() -> float:
	return float(_node(pump_id)["pressure_pa"])


## The supply's and destination's pressures [Pa] and the supply's temperature
## [K], off their kinds: what the file or the last command set, real before the
## first tick.
func _source_pa() -> float:
	return float(_node(source_id)["kind"]["pressure"])


func _source_k() -> float:
	return float(_node(source_id)["kind"]["temperature"])


func _destination_pa() -> float:
	return float(_node(destination_id)["kind"]["pressure"])


## What the engine says about the supply boiling where it stands (M52):
## "measured" with the pressure its liquid boils below, "gas", or
## "cannot_tell"; null before the first tick.
func _supply_boiling() -> Variant:
	return _node(source_id).get("supply_boiling")


func _supply_boiling_label() -> String:
	var check = _supply_boiling()
	if check == null:
		return "--"
	if check["check"] == "measured":
		return "boils below %s" % _bar(float(check["bubble_pressure_pa"]))
	if check["check"] == "gas":
		return "a gas: nothing to boil"
	return "this plant cannot check for boiling"


## The liquid's boiling (bubble) pressure at the suction [Pa]; NAN where the
## plant's thermo has none.
func _bubble_pa() -> float:
	var cavitation = _node(pump_id).get("cavitation")
	return NAN if cavitation == null else float(cavitation["bubble_pressure_pa"])


## M11's lamp: the bulk liquid at the suction is below its boiling pressure.
func _lamp() -> Variant:
	var cavitation = _node(pump_id).get("cavitation")
	return null if cavitation == null else bool(cavitation["cavitating"])


## The share of its curve the pump delivers (M50), or null.
func _head_fraction() -> Variant:
	var suction = _node(pump_id).get("pump_suction")
	return null if suction == null else float(suction["head_fraction"])


func _npsh_available_m() -> Variant:
	var suction = _node(pump_id).get("pump_suction")
	return null if suction == null else float(suction["npsh_available_m"])


## The pump's declared NPSH3 [m] — the margin at which it has lost 3% of its
## push — or null on a pump with no suction model.
func _npsh_required_m() -> Variant:
	var suction = _pump_kind().get("suction")
	return null if suction == null else float(suction["npsh_required"])


func _lamp_label() -> String:
	var lamp = _lamp()
	if lamp == null:
		return "none"
	return "BOILING" if lamp else "no"


## Why there is no push reading, in words.
func _no_push_reason() -> String:
	if not _pump_on():
		return "pump stopped"
	if _npsh_required_m() == null:
		return "no suction model: whole push, boiling or not"
	return "not yet measured (first tick)"


func _bar(pa: float) -> String:
	return "--" if is_nan(pa) else "%.3f bar" % (pa / 1.0e5)


func _percent(value) -> String:
	return "--" if value == null else "%.0f%%" % (float(value) * 100.0)


func _readout(tick: int) -> String:
	var head = _head_fraction()
	var margin = _npsh_available_m()
	var line := (
		"t=%4d  valve=%3.0f%%  pump=%s  flow=%6.2f kg/s  suction=%s  boils at %s  margin=%s  push=%s  lamp=%s"
		% [
			tick,
			_opening() * 100.0,
			"on " if _pump_on() else "off",
			_flow(),
			_bar(_suction_pa()),
			_bar(_bubble_pa()),
			"--" if margin == null else "%.2f m" % float(margin),
			_percent(head) if head != null else "-- (%s)" % _no_push_reason(),
			_lamp_label(),
		]
	)
	# The supply and destination, once a command has moved either (M52) — so
	# every line up to then reads as it did in M51.
	if moved_conditions:
		line += (
			"  supply=%s %.0f C (%s)  destination=%s"
			% [_bar(_source_pa()), _source_k() - 273.15, _supply_boiling_label(), _bar(_destination_pa())]
		)
	return line


# ---------------------------------------------------------------- drawing

const SOURCE_POS := Vector2(70, 170)
const PUMP_POS := Vector2(350, 170)
const PUMP_RADIUS := 34.0
const VALVE_POS := Vector2(510, 170)
const SINK_POS := Vector2(670, 170)
const GAUGE_RECT := Rect2(60, 300, 24, 200)
const PLOT_RECT := Rect2(360, 300, 340, 200)
const PANEL_X := 790.0

const BACKGROUND := Color(0.09, 0.10, 0.12)
const SHELL := Color(0.42, 0.45, 0.50)
const PIPE := Color(0.55, 0.58, 0.62)
const LIQUID := Color(0.20, 0.45, 0.75)
const VAPOUR := Color(0.85, 0.90, 0.95)
const PUSH := Color(0.95, 0.60, 0.20)
const INK := Color(0.88, 0.90, 0.93)
const DIM := Color(0.60, 0.63, 0.68)
const GRID := Color(0.20, 0.22, 0.25)
const GOOD := Color(0.35, 0.80, 0.45)
const BAD := Color(0.95, 0.30, 0.25)
const MARK := Color(0.95, 0.78, 0.30)


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
		"t = %.1f s    %d tick(s) per frame%s" % [float(snapshot["sim_time"]), ticks_per_frame, "    PAUSED" if paused else ""],
		DIM
	)

	_draw_line()
	_draw_pump()
	_draw_gauge()
	_draw_plot()
	_draw_panel()

	draw_multiline_string(
		ThemeDB.fallback_font, Vector2(30, 556), message, HORIZONTAL_ALIGNMENT_LEFT,
		get_viewport_rect().size.x - 60, 15, 2, BAD if message_bad else GOOD
	)
	_text(
		Vector2(30, 600),
		"Up/Down valve   K pump on/off   W/S supply pressure   E/D supply temperature   R/F destination pressure",
		DIM
	)
	_text(
		Vector2(30, 624),
		"C clear trail   Space pause   [ ] speed   1 long suction line (M50)   2 boiling pump, no suction model (M11)",
		DIM
	)
	if halted != "":
		_text(Vector2(30, 90), "HALTED — %s" % halted, BAD)


## Source, suction line, pump, valve and destination, with the flow on the
## suction line. A flow running backwards is drawn as such.
func _draw_line() -> void:
	var plant: Dictionary = PLANTS[plant_key]
	draw_polyline(PackedVector2Array([SOURCE_POS, PUMP_POS - Vector2(PUMP_RADIUS, 0)]), PIPE, 6.0)
	draw_polyline(PackedVector2Array([PUMP_POS + Vector2(PUMP_RADIUS, 0), SINK_POS]), PIPE, 6.0)

	draw_rect(Rect2(SOURCE_POS - Vector2(30, 40), Vector2(40, 80)), LIQUID)
	draw_rect(Rect2(SOURCE_POS - Vector2(30, 40), Vector2(40, 80)), SHELL, false, 3.0)
	_text(SOURCE_POS + Vector2(-34, 64), plant["source"], DIM, 14)
	_text(SOURCE_POS + Vector2(-34, 82), _bar(_source_pa()), DIM, 14)
	_text(SOURCE_POS + Vector2(-34, 100), "%.0f °C" % (_source_k() - 273.15), DIM, 14)

	var flow := _flow()
	var arrow := "->" if flow >= 0.0 else "<- BACKWARDS"
	_text(Vector2(SOURCE_POS.x + 30, SOURCE_POS.y - 34), plant["suction_pipe"], DIM, 14)
	_text(Vector2(SOURCE_POS.x + 30, SOURCE_POS.y - 14), "%.2f kg/s %s" % [flow, arrow], BAD if flow < 0.0 else INK, 14)

	# The valve: a bow-tie, its opening written under it.
	var half := 16.0
	draw_colored_polygon(
		PackedVector2Array(
			[
				VALVE_POS + Vector2(-half, -half),
				VALVE_POS,
				VALVE_POS + Vector2(-half, half),
			]
		),
		SHELL
	)
	draw_colored_polygon(
		PackedVector2Array(
			[
				VALVE_POS + Vector2(half, -half),
				VALVE_POS,
				VALVE_POS + Vector2(half, half),
			]
		),
		SHELL
	)
	_text(VALVE_POS + Vector2(-40, 40), plant["valve"], DIM, 14)
	_text(VALVE_POS + Vector2(-40, 58), "%.0f%% open" % (_opening() * 100.0), INK, 16)

	draw_rect(Rect2(SINK_POS - Vector2(0, 30), Vector2(50, 60)), SHELL, false, 3.0)
	_text(SINK_POS + Vector2(-6, 50), plant["destination"], DIM, 14)
	_text(SINK_POS + Vector2(-6, 68), _bar(_destination_pa()), DIM, 14)


## The pump, filled by the share of its push it still delivers, with vapour at
## its eye in proportion to the push it has lost. On a pump with no suction
## model the lamp alone decides the bubbles, and the fill stays whole.
func _draw_pump() -> void:
	var head = _head_fraction()
	var on := _pump_on()
	var fill := GOOD
	if not on:
		fill = SHELL
	elif head != null:
		fill = BAD.lerp(GOOD, float(head))
	draw_circle(PUMP_POS, PUMP_RADIUS, fill)
	draw_arc(PUMP_POS, PUMP_RADIUS, 0.0, TAU, 48, INK, 2.0)

	var lost := 0.0
	if on and head != null:
		lost = 1.0 - float(head)
	elif on and _lamp() == true:
		lost = 1.0
	for i in roundi(lost * 10.0):
		var angle := TAU * i / 10.0
		draw_circle(PUMP_POS + Vector2(cos(angle), sin(angle)) * 16.0, 5.0, VAPOUR)

	_text(PUMP_POS + Vector2(-40, -PUMP_RADIUS - 30), PLANTS[plant_key]["pump"], INK)
	var label := "STOPPED" if not on else ("push %s" % _percent(head) if head != null else "whole push")
	_text(PUMP_POS + Vector2(-40, PUMP_RADIUS + 26), label, fill if on else BAD, 18)


## The suction pressure, against the liquid's boiling pressure and the supply's.
func _draw_gauge() -> void:
	var rect := GAUGE_RECT
	var high := maxf(_source_pa(), _suction_pa()) * 1.15
	draw_rect(rect, Color(0.13, 0.14, 0.16))
	var fraction := clampf(_suction_pa() / high, 0.0, 1.0)
	var below: bool = not is_nan(_bubble_pa()) and _suction_pa() < _bubble_pa()
	draw_rect(
		Rect2(rect.position + Vector2(0, rect.size.y * (1.0 - fraction)), Vector2(rect.size.x, rect.size.y * fraction)),
		BAD if below else LIQUID
	)
	draw_rect(rect, SHELL, false, 2.0)
	_marker(rect, high, _source_pa(), "supply %s" % _bar(_source_pa()), DIM)
	if not is_nan(_bubble_pa()):
		_marker(rect, high, _bubble_pa(), "boils at %s" % _bar(_bubble_pa()), MARK)
	_text(rect.position + Vector2(-6, -12), "pump suction", DIM, 14)
	_text(rect.position + Vector2(-6, rect.size.y + 22), _bar(_suction_pa()), BAD if below else INK)


func _marker(rect: Rect2, high: float, pa: float, label: String, color: Color) -> void:
	var y := rect.end.y - rect.size.y * clampf(pa / high, 0.0, 1.0)
	draw_line(Vector2(rect.position.x - 4, y), Vector2(rect.end.x + 4, y), color, 2.0)
	_text(Vector2(rect.end.x + 8, y + 5), label, color, 14)


## The trail: flow (blue, left scale) and push (orange, right scale) at every
## opening visited. A hollow dot is the pump stopped. The ring is now.
func _draw_plot() -> void:
	var rect := PLOT_RECT
	var flow_low := 0.0
	var flow_high := 5.0
	for point in trail.values():
		flow_low = minf(flow_low, floorf(float(point["flow"]) / 5.0) * 5.0)
		flow_high = maxf(flow_high, ceilf(float(point["flow"]) / 5.0) * 5.0)
	draw_rect(rect, Color(0.12, 0.13, 0.15))
	for i in 5:
		var x := rect.position.x + rect.size.x * i / 4.0
		draw_line(Vector2(x, rect.position.y), Vector2(x, rect.end.y), GRID, 1.0)
		_text(Vector2(x - 12, rect.end.y + 18), "%d%%" % (i * 25), DIM, 12)
	var zero_y := _plot_y(0.0, flow_low, flow_high)
	draw_line(Vector2(rect.position.x, zero_y), Vector2(rect.end.x, zero_y), GRID, 1.0)
	draw_rect(rect, SHELL, false, 2.0)
	_text(Vector2(rect.position.x - 4, rect.position.y - 10), "flow %.0f kg/s" % flow_high, LIQUID, 12)
	_text(Vector2(rect.position.x - 4, rect.end.y + 34), "valve opening", DIM, 12)
	# The legend, beside the axis title.
	var legend := Vector2(rect.position.x + 100, rect.end.y + 30)
	draw_circle(legend, 4.0, LIQUID)
	_text(legend + Vector2(8, 4), "flow", LIQUID, 12)
	draw_circle(legend + Vector2(50, 0), 4.0, PUSH)
	_text(legend + Vector2(58, 4), "push", PUSH, 12)
	draw_arc(legend + Vector2(104, 0), 4.0, 0.0, TAU, 16, LIQUID, 1.5)
	_text(legend + Vector2(112, 4), "pump off", DIM, 12)
	draw_arc(legend + Vector2(178, 0), 7.0, 0.0, TAU, 24, INK, 1.5)
	_text(legend + Vector2(188, 4), "now", DIM, 12)
	_text(Vector2(rect.position.x - 12, rect.end.y + 4), "0" if flow_low == 0.0 else "", LIQUID, 12)
	_text(Vector2(rect.end.x - 70, rect.position.y - 10), "push 100%", PUSH, 12)
	if flow_low < 0.0:
		_text(Vector2(rect.position.x - 22, rect.end.y + 4), "%.0f" % flow_low, LIQUID, 12)

	var now_conditions := _conditions()
	for point in trail.values():
		var x := rect.position.x + rect.size.x * float(point["opening"])
		var flow_at := Vector2(x, _plot_y(float(point["flow"]), flow_low, flow_high))
		# A dot read under other supply or destination conditions is faded: the
		# curve it belongs to is not the one the plant is on now.
		var alpha := 1.0 if point["conditions"] == now_conditions else 0.3
		var liquid := Color(LIQUID, alpha)
		if point["on"]:
			draw_circle(flow_at, 4.0, liquid)
		else:
			draw_arc(flow_at, 4.0, 0.0, TAU, 16, liquid, 1.5)
		if point["head"] != null:
			draw_circle(Vector2(x, rect.end.y - rect.size.y * float(point["head"])), 4.0, Color(PUSH, alpha))

	var now_x := rect.position.x + rect.size.x * _opening()
	draw_arc(Vector2(now_x, _plot_y(_flow(), flow_low, flow_high)), 8.0, 0.0, TAU, 24, INK, 1.5)


func _plot_y(flow: float, low: float, high: float) -> float:
	return PLOT_RECT.end.y - PLOT_RECT.size.y * clampf((flow - low) / (high - low), 0.0, 1.0)


## The right-hand panel: the pump's suction in numbers, and what each means.
func _draw_panel() -> void:
	var y := 110.0
	_text(Vector2(PANEL_X, y), "PUMP", DIM)
	y += 24
	_text(
		Vector2(PANEL_X, y),
		"%-14s%s" % [PLANTS[plant_key]["pump"], "running" if _pump_on() else "STOPPED (K starts it)"],
		GOOD if _pump_on() else BAD
	)
	y += 22
	var head = _head_fraction()
	if head != null:
		_text(Vector2(PANEL_X, y), "push delivered  %s of its curve" % _percent(head), BAD.lerp(GOOD, float(head)))
	else:
		_text(Vector2(PANEL_X, y), "push delivered  -- (%s)" % _no_push_reason(), DIM, 14)
	y += 22
	_text(Vector2(PANEL_X, y), "flow            %.2f kg/s" % _flow(), INK)

	y += 36
	_text(Vector2(PANEL_X, y), "SUPPLY", DIM)
	y += 24
	_text(Vector2(PANEL_X, y), "%s  %s, %.0f °C" % [PLANTS[plant_key]["source"], _bar(_source_pa()), _source_k() - 273.15], INK)
	y += 22
	var check = _supply_boiling()
	var unchecked: bool = check != null and check["check"] == "cannot_tell"
	_text(Vector2(PANEL_X + 12, y), _supply_boiling_label(), BAD if unchecked else MARK, 14)
	y += 20
	_text(Vector2(PANEL_X + 12, y), "destination %s" % _bar(_destination_pa()), DIM, 14)

	y += 36
	_text(Vector2(PANEL_X, y), "SUCTION", DIM)
	y += 24
	_text(Vector2(PANEL_X, y), "pressure        %s" % _bar(_suction_pa()), INK)
	y += 22
	_text(Vector2(PANEL_X, y), "liquid boils at %s" % _bar(_bubble_pa()), MARK)
	y += 22
	var margin = _npsh_available_m()
	var needed = _npsh_required_m()
	if needed == null:
		_text(Vector2(PANEL_X, y), "margin          -- (no suction model)", DIM, 14)
	else:
		_text(
			Vector2(PANEL_X, y),
			"margin          %s" % ("--" if margin == null else "%.2f m" % float(margin)),
			INK if margin == null or float(margin) >= float(needed) else BAD
		)
		y += 20
		_text(Vector2(PANEL_X + 12, y), "loses 3%% of its push at %.2f m (NPSH3)" % float(needed), DIM, 14)

	y += 36
	_text(Vector2(PANEL_X, y), "BOILING LAMP", DIM)
	y += 24
	var lamp = _lamp()
	_text(Vector2(PANEL_X, y), _lamp_label(), DIM if lamp == null else (BAD if lamp else GOOD))
	# Why the lamp and the push can disagree, said where a player would ask.
	var why := PackedStringArray()
	if lamp == true and needed == null:
		why = PackedStringArray(["the liquid boils at the suction, but this", "pump has no suction model: it keeps its", "whole push anyway (the M11 plant)"])
	elif lamp == false and head != null and float(head) < 0.97:
		why = PackedStringArray(["the liquid in the line is not boiling;", "it boils at the pump's eye, where the", "pressure is lower - the push counts that"])
	for line in why:
		y += 18
		_text(Vector2(PANEL_X + 12, y), line, DIM, 14)


func _text(at: Vector2, text: String, color: Color, size: int = 16) -> void:
	draw_string(ThemeDB.fallback_font, at, text, HORIZONTAL_ALIGNMENT_LEFT, -1, size, color)


func _halt(reason: String) -> void:
	halted = reason
	push_error("pump: %s" % reason)
	print("pump: HALTED — %s" % reason)
	queue_redraw()
