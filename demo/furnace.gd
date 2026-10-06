# M39's demonstrated criterion: a screen for the furnace milestones (M34–M38).
#
# The furnace work since M34 — a tube coil with a temperature of its own, the
# flame ceiling, tubes that burst into a fire, trips that cut the fuel and the
# emergency-stop button — ran only in tests and the CLI. This scene draws it, on
# either of two shipped plants:
#
#   trip     scenarios/furnace_coil_trip.toml — a fouled heater holding its outlet
#            at 60 °C while its tubes run hot, and the tube-skin trip that cuts it.
#   burnout  scenarios/furnace_burnout.toml — a fouled heater over-fired on gas
#            oil until its tubes burst and the leak burns in the firebox.
#   autoreset scenarios/furnace_coil_trip_autoreset.toml (M40) — the trip plant
#            with a tube trip that resets itself and relights the furnace, so it
#            cuts and relights on its own until the target is lowered.
#   burst    scenarios/furnace_burst_during_stop.toml (M43) — the self-resetting
#            trip on tubes that burst on the trip's own tick: the trip resets, the
#            furnace stays dark, and the screen says why (the snapshot's
#            `trip_stop`).
#   permissive scenarios/furnace_restart_permissive.toml (M44) — a heater fired by
#            hand behind a feed pump: the tank's overfill trip stops the pump, the
#            starved heater's tube trip cuts it and resets itself, and the heater
#            stays dark while the pump's trip is latched — its start permissive.
#
# Same rule as plant.gd, and the same reason: this script computes nothing
# physical. Every number it draws is a snapshot field, every marker on a gauge is
# a limit the snapshot carries (the tubes' failure temperature, the flame, each
# trip's own limit), and every refusal it shows is the engine's own message. The
# one arithmetic it does is display units — kelvin to °C, watts to MW — which is
# what a frontend boundary is for (CLAUDE.md rule 4).
#
# Interactive:  godot --path . res://demo/furnace.tscn
#               (keys listed on screen and in _input; 1 / 2 switch plants)
# Recorded:     godot --headless --path . res://demo/furnace.tscn --quit-after 20000 -- --auto [--plant=burnout]
#               ...runs that plant's scripted timeline (TIMELINES)
#               and prints a `t=` line at a fixed interval plus one line per event.
#               Those lines are the observation in ROADMAP.md's M39 section.
# Screenshots:  add --shots=<dir> to a WINDOWED --auto run (headless renders
#               nothing) to save a PNG at each scripted event.
extends Node2D

## The three plants, and the names on each the scene draws. A name missing from a
## plant is a load-time halt, not a guess.
const PLANTS := {
	"trip":
	{
		"scenario": "res://scenarios/furnace_coil_trip.toml",
		"title": "Fouled heater with a tube-skin trip",
		"feed": "cool_feed",
		"outlet_pipe": "heated_line",
		"destination": "hold_tank",
		"ticks_per_frame": 1,
	},
	"burnout":
	{
		"scenario": "res://scenarios/furnace_burnout.toml",
		"title": "Over-fired heater: tube burn-out",
		"feed": "charge",
		"outlet_pipe": "heated_line",
		"destination": "product",
		"ticks_per_frame": 4,
	},
	"autoreset":
	{
		"scenario": "res://scenarios/furnace_coil_trip_autoreset.toml",
		"title": "Fouled heater with a self-resetting tube trip",
		"feed": "cool_feed",
		"outlet_pipe": "heated_line",
		"destination": "hold_tank",
		"ticks_per_frame": 1,
	},
	"burst":
	{
		"scenario": "res://scenarios/furnace_burst_during_stop.toml",
		"title": "Tubes burst during a trip: who relights?",
		"feed": "cool_feed",
		"outlet_pipe": "heated_line",
		"destination": "hold_tank",
		"ticks_per_frame": 1,
	},
	"permissive":
	{
		"scenario": "res://scenarios/furnace_restart_permissive.toml",
		"title": "Starved heater: the tube trip waits for the pump",
		"feed": "cool_feed",
		"outlet_pipe": "heated_line",
		"destination": "hold_tank",
		"ticks_per_frame": 1,
		# The one plant with a pump the scene draws and starts (key K).
		"pump": "feed_pump",
	},
}

## Scripted timelines for the recorded runs, in ticks. Each tick is one the
## bridge test `crates/godot-ext/tests/furnace_screen.rs` replays with the same
## commands and pins the outcome of — so a timeline that stops telling its story
## fails `cargo test`, not just this demo.
##
## trip: the trip fires by itself at 75. Reset with the loop back in AUTO at
## the 60 °C target, the fouled coil climbs past 100 °C again and trips at 239 —
## the honest answer, since this coil settles at 117.3 °C on that target. Reset
## with the target at 50 °C instead and the coil settles near 76 °C. The button
## is pressed on that healthy plant at 900, reset at 901, and the loop relit at
## 950: by default a reset does not relight the furnace (docs/DESIGN.md §45).
const AUTO_TRIP := {
	150: ["reset", "auto"],
	500: ["reset", "setpoint_50", "auto"],
	900: ["press"],
	901: ["reset"],
	950: ["auto"],
	1000: ["quit"],
}
## burnout: the tubes burst by themselves at 1 145. At 1 300 the hole is
## patched, the fuel cut to 0.5 MW and new tubes asked for — refused, because
## the coil is still far past its limit. The coil cools under 550 °C near 2 460;
## new tubes go in at 2 500.
const AUTO_BURNOUT := {
	1300: ["patch", "duty_0.5", "new_tubes"],
	2500: ["new_tubes"],
	2600: ["quit"],
}
## autoreset (M40): nobody touches the trip. It cuts at 75, re-arms and hands
## the loop back by itself at 128, cuts again at 214 and relights at 267 — the
## fouled coil cannot hold 60 °C under its limit. At 300 the target goes to
## 50 °C and the cycle stops. The button is pressed at 900; the coil falls far
## under the 80 °C reset point and the pressed trip still waits. A person resets
## both at 1000 and the furnace stays dark: a trip pressed by hand never
## restarts anything.
const AUTO_AUTORESET := {
	300: ["setpoint_50"],
	900: ["press"],
	1000: ["reset"],
	1100: ["quit"],
}
## burst (M43): the tubes burst on tick 75, the tick the trip cuts the fuel. At
## 80 the hole is patched and new tubes fitted while the trip still holds. The
## trip resets itself at 128 and the furnace stays dark: a burst during a stop
## makes its restart a person's (DESIGN §47), and the panel says so. At 200 a
## person relights it on a 50 °C target, which keeps the coil under the trip.
const AUTO_BURST := {
	80: ["patch", "new_tubes"],
	200: ["setpoint_50", "auto"],
	400: ["quit"],
}
## permissive (M44): the tank's overfill trip stops the feed pump at 266; the
## heater, still fired at 3 MW, starves and its tube trip cuts it at 299, then
## resets itself at 321 and leaves it dark — the pump's trip is its start
## permissive (DESIGN §49), and the panel says so. At 400 a person resets the
## pump's trip, starts the pump and relights the heater at 3 MW.
const AUTO_PERMISSIVE := {
	400: ["reset", "pump_on", "duty_3"],
	500: ["quit"],
}
const TIMELINES := {
	"trip": AUTO_TRIP,
	"burnout": AUTO_BURNOUT,
	"autoreset": AUTO_AUTORESET,
	"burst": AUTO_BURST,
	"permissive": AUTO_PERMISSIVE,
}
const AUTO_SHOT_TICKS := {
	"trip": [74, 75, 238, 900, 901, 975],
	"burnout": [1144, 1145, 1300, 2500],
	"autoreset": [75, 128, 214, 300, 901, 1001],
	"burst": [75, 80, 128, 200, 260],
	"permissive": [266, 299, 321, 400, 450],
}
## Print a `t=` line every this many ticks in a recorded run.
const PRINT_EVERY := {"trip": 25, "burnout": 100, "autoreset": 25, "burst": 25, "permissive": 25}

## Step sizes for the keys.
const DUTY_STEP_W := 2.5e5
const SETPOINT_STEP_K := 5.0
const MAX_TICKS_PER_FRAME := 64

@onready var sim: RefinerySim = $Sim

var plant_key := "trip"
var heater_id := -1
## -1 on every plant but the one whose entry names a `pump`.
var pump_id := -1
var feed_id := -1
var destination_id := -1
var outlet_pipe_id := -1
var snapshot: Dictionary = {}
var halted := ""
var auto_run := false
var shots_dir := ""
var paused := false
## A recorded run has reached its end, or is waiting on a screenshot.
var finished := false
var shooting := false
var ticks_per_frame := 1
var message := ""
var message_bad := false
## Last-seen trip, tube and stop states, to print an event line when one changes.
var seen_trip_states: Array = []
var seen_tubes := ""
var seen_stop := ""


func _ready() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg == "--auto":
			auto_run = true
		elif arg.begins_with("--plant="):
			plant_key = arg.trim_prefix("--plant=")
		elif arg.begins_with("--shots="):
			shots_dir = arg.trim_prefix("--shots=")
	if not PLANTS.has(plant_key):
		_halt("no plant '%s' (trip | burnout | autoreset | burst | permissive)" % plant_key)
		return
	_load(plant_key)


func _load(key: String) -> void:
	plant_key = key
	halted = ""
	finished = false
	snapshot = {}
	seen_trip_states = []
	seen_tubes = ""
	seen_stop = ""
	var plant: Dictionary = PLANTS[key]
	ticks_per_frame = plant["ticks_per_frame"]

	var err = JSON.parse_string(sim.load_scenario(plant["scenario"]))
	if err != null:
		_halt("load failed: %s" % err)
		return

	heater_id = sim.node_id("heater")
	feed_id = sim.node_id(plant["feed"])
	destination_id = sim.node_id(plant["destination"])
	outlet_pipe_id = sim.edge_id(plant["outlet_pipe"])
	pump_id = sim.node_id(plant["pump"]) if plant.has("pump") else -1
	if heater_id < 0 or feed_id < 0 or destination_id < 0 or outlet_pipe_id < 0:
		_halt("this scenario does not have the names the scene expects")
		return
	if plant.has("pump") and pump_id < 0:
		_halt("this scenario does not have the names the scene expects")
		return

	snapshot = JSON.parse_string(sim.snapshot_json())
	_say("loaded %s" % plant["scenario"], false)
	print("furnace: loaded %s" % plant["scenario"])
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
		# Same reasoning as plant.gd: ticking through a diverged solve draws a
		# plant nobody solved.
		_halt("tick %d: %s" % [sim.tick_index(), err["message"]])
		return false
	snapshot = JSON.parse_string(sim.snapshot_json())
	var tick := sim.tick_index()
	_report_events(tick)

	if not auto_run:
		return true
	if tick % int(PRINT_EVERY[plant_key]) == 0:
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


## Print a line when a trip latches or resets, or the tubes change state — read
## from the snapshot, so it is the engine's account of what happened.
func _report_events(tick: int) -> void:
	var states: Array = []
	for trip in _trips():
		states.append(_trip_label(trip))
	if seen_trip_states.size() == states.size():
		for i in states.size():
			if states[i] != seen_trip_states[i]:
				print("furnace: t=%d  trip %s -> %s" % [tick, _trips()[i]["name"], states[i]])
	seen_trip_states = states

	var tubes := _tubes_label()
	if seen_tubes != "" and tubes != seen_tubes:
		print("furnace: t=%d  tubes -> %s" % [tick, tubes])
	seen_tubes = tubes

	# The trips' account of the heater's stop (M43). When they let go without
	# relighting it, the message line says so in the engine's reasons: it is the
	# moment a player would otherwise be left guessing.
	var reasons := _bar_lines()
	var stop_line := _stop_label()
	if not reasons.is_empty():
		stop_line += " (%s)" % ", ".join(reasons)
	if seen_stop != "" and stop_line != seen_stop:
		print("furnace: t=%d  heater stop -> %s" % [tick, stop_line])
		var stop = _trip_stop()
		if stop != null and stop["status"] == "not_restarted":
			_say(
				(
					"t=%d: the trips let go and the heater was NOT relit: %s. Relight it by hand: A (loop AUTO) or Up (fuel)"
					% [tick, ", ".join(reasons)]
				),
				true
			)
	seen_stop = stop_line


## Ticking holds until the frame showing `tick` has been drawn and saved.
func _shoot(tick: int) -> void:
	shooting = true
	queue_redraw()
	await RenderingServer.frame_post_draw
	var path := "%s/furnace_%s_t%04d.png" % [shots_dir, plant_key, tick]
	var image := get_viewport().get_texture().get_image()
	if image != null:
		image.save_png(path)
		print("furnace: saved %s" % path)
	shooting = false


func _input(event: InputEvent) -> void:
	if not (event is InputEventKey and event.pressed and not event.echo):
		return
	match event.keycode:
		KEY_1:
			_load("trip")
		KEY_2:
			_load("burnout")
		KEY_3:
			_load("autoreset")
		KEY_4:
			_load("burst")
		KEY_5:
			_load("permissive")
		KEY_SPACE:
			paused = not paused
		KEY_BRACKETLEFT:
			ticks_per_frame = maxi(1, ticks_per_frame / 2)
		KEY_BRACKETRIGHT:
			ticks_per_frame = mini(MAX_TICKS_PER_FRAME, ticks_per_frame * 2)
		KEY_E:
			_do("press")
		KEY_R:
			_do("reset")
		KEY_A:
			_do("toggle_mode")
		KEY_W:
			_do("setpoint_up")
		KEY_S:
			_do("setpoint_down")
		KEY_UP:
			_do("duty_up")
		KEY_DOWN:
			_do("duty_down")
		KEY_P:
			_do("patch")
		KEY_N:
			_do("new_tubes")
		KEY_K:
			_do("toggle_pump")
	queue_redraw()


# --------------------------------------------------------------- commands

## Every action the keys and the timelines share. Each sends the contract's own
## JSON; ids read from the snapshot arrive as floats and are cast to int, or
## serde would refuse `0.0` for an id.
func _do(action: String) -> void:
	match action:
		"press":
			# The emergency stop presses every ARMED trip on the plant.
			var pressed := 0
			for trip in _trips():
				if trip["state"]["status"] == "armed":
					_send("press %s" % trip["name"], {"cmd": "manual_trip", "trip_id": int(trip["id"])})
					pressed += 1
			if pressed == 0:
				_say("emergency stop: no armed trip on this plant to press", true)
		"reset":
			var reset := 0
			for trip in _trips():
				if trip["state"]["status"] == "tripped":
					_send("reset %s" % trip["name"], {"cmd": "reset_trip", "trip_id": int(trip["id"])})
					reset += 1
			if reset == 0:
				_say("reset: no tripped trip on this plant", true)
		"auto":
			_set_mode("auto")
		"toggle_mode":
			var loop = _loop()
			if loop != null:
				_set_mode("manual" if loop["mode"] == "auto" else "auto")
			else:
				_say("this plant has no control loop", true)
		"setpoint_50":
			_set_setpoint_k(273.15 + 50.0)
		"setpoint_up", "setpoint_down":
			var loop = _loop()
			if loop == null:
				_say("this plant has no control loop", true)
				return
			var step := SETPOINT_STEP_K if action == "setpoint_up" else -SETPOINT_STEP_K
			_set_setpoint_k(float(loop["setpoint"]["k"]) + step)
		"duty_up", "duty_down", "duty_0.5", "duty_3":
			var duty := 3.0e6 if action == "duty_3" else 5.0e5
			if action == "duty_up" or action == "duty_down":
				var step := DUTY_STEP_W if action == "duty_up" else -DUTY_STEP_W
				duty = maxf(0.0, _duty_w() + step)
			_send(
				"fire heater at %.2f MW" % (duty / 1.0e6),
				{"cmd": "set_furnace_duty", "node": heater_id, "duty": duty}
			)
		"patch":
			_send("patch %s" % PLANTS[plant_key]["outlet_pipe"], {"cmd": "puncture_pipe", "edge": outlet_pipe_id, "area": 0.0})
		"new_tubes":
			_send("replace the heater's tubes", {"cmd": "replace_tubes", "node": heater_id})
		"pump_on", "toggle_pump":
			if pump_id < 0:
				_say("this plant has no pump on the screen", true)
				return
			var on := action == "pump_on" or not _pump_on()
			_send(
				"%s %s" % ["start" if on else "stop", PLANTS[plant_key]["pump"]],
				{"cmd": "set_pump_on", "node": pump_id, "on": on}
			)


func _set_mode(mode: String) -> void:
	var loop = _loop()
	if loop == null:
		_say("this plant has no control loop", true)
		return
	_send(
		"%s to %s" % [loop["name"], mode.to_upper()],
		{"cmd": "set_controller_mode", "loop_id": int(loop["id"]), "mode": mode}
	)


func _set_setpoint_k(kelvin: float) -> void:
	var loop = _loop()
	if loop == null:
		_say("this plant has no control loop", true)
		return
	_send(
		"%s setpoint to %.1f C" % [loop["name"], kelvin - 273.15],
		{
			"cmd": "set_setpoint",
			"loop_id": int(loop["id"]),
			"value": {"variable": "temperature", "k": kelvin},
		}
	)


## Send one command and put the engine's answer on screen — its own refusal
## message when it says no, which is usually the most useful line on the screen.
func _send(what: String, command: Dictionary) -> void:
	var text := JSON.stringify(command)
	var err = JSON.parse_string(sim.apply_command(text))
	if err == null:
		_say(what, false)
		if auto_run:
			print("furnace: t=%d  %s  %s" % [sim.tick_index(), what, text])
	else:
		_say("%s: REFUSED — %s" % [what, err["message"]], true)
		if auto_run:
			print("furnace: t=%d  %s  REFUSED: %s" % [sim.tick_index(), what, err["message"]])


func _say(text: String, bad: bool) -> void:
	message = text
	message_bad = bad


# ---------------------------------------------------------------- reading

## Field lookups only, as in plant.gd. `trips` and `controls` are left out of the
## snapshot when a plant has none, hence the defaults.

func _node(id: int) -> Dictionary:
	return snapshot["nodes"][id]


func _heater_kind() -> Dictionary:
	return _node(heater_id)["kind"]


func _duty_w() -> float:
	return float(_heater_kind()["duty"])


func _coil_k() -> float:
	return float(_heater_kind()["coil"]["temperature"])


func _flame_k() -> float:
	return float(_heater_kind()["flame_temperature"])


func _failure_k() -> float:
	return float(_heater_kind()["tubes"]["failure_temperature"])


func _tubes_failed() -> bool:
	return _heater_kind()["tubes"]["state"]["status"] == "failed"


## The trips' account of the heater's stop (M43, the snapshot's `trip_stop`):
## null when they have nothing to say — never stopped, handed back, or relit.
func _trip_stop() -> Variant:
	return _node(heater_id).get("trip_stop")


## The engine's reasons, in words. While a trip holds the heater, the last three
## can still go away: new tubes, or the other trips clearing (M44).
const BAR_TEXT := {
	"reset_restarts_nothing": "a trip that held it restarts nothing",
	"pressed_by_hand": "the emergency stop was pressed",
	"tubes_burst_during_stop": "the tubes burst during the stop",
	"tubes_burst": "the tubes are burst: N fits new ones",
	"trip_about_to_fire": "another trip on it is past its limit",
	"permissive_not_clear": "a trip it waits for is not clear (see TRIPS)",
}


## Whether the plant's drawn pump runs; false on a plant without one.
func _pump_on() -> bool:
	return pump_id >= 0 and bool(_node(pump_id)["kind"]["on"])


func _stop_label() -> String:
	var stop = _trip_stop()
	if stop == null:
		return "none"
	if stop["status"] == "held":
		return "HELD by a trip"
	return "NOT RELIT at t=%d" % int(stop["at_tick"])


func _bar_lines() -> PackedStringArray:
	var lines := PackedStringArray()
	var stop = _trip_stop()
	if stop != null:
		for bar in stop["barred_by"]:
			lines.append(BAR_TEXT.get(bar, bar))
	return lines


func _tubes_label() -> String:
	var state: Dictionary = _heater_kind()["tubes"]["state"]
	if state["status"] == "failed":
		return "BURST at t=%d" % int(state["at_tick"])
	return "intact"


## Outlet temperature [K], or NAN before the first solve (a `null` field).
func _outlet_k() -> float:
	var t = _node(heater_id)["temperature_k"]
	return NAN if t == null else float(t)


func _optional_w(field: String) -> float:
	var w = _node(heater_id).get(field)
	return 0.0 if w == null else float(w)


func _outlet_flow() -> float:
	return float(snapshot["edges"][outlet_pipe_id]["stream"]["mass_flow"])


func _leak_kg_s() -> float:
	return float(snapshot["edges"][outlet_pipe_id]["leak_mass_flow"])


func _trips() -> Array:
	return snapshot.get("trips", [])


## The loop on the heater's outlet, found by what it watches (M41), not by its
## place in the list; null when the plant has none.
func _loop() -> Variant:
	for loop in snapshot.get("controls", []):
		var watches: Dictionary = loop["watches"]
		if watches.has("node") and int(watches["node"]) == heater_id and loop["setpoint"]["variable"] == "temperature":
			return loop
	return null


func _trip_label(trip: Dictionary) -> String:
	var state: Dictionary = trip["state"]
	if state["status"] == "armed":
		return "armed"
	var label := "TRIPPED at t=%d" % int(state["at_tick"])
	if state.get("by_hand", false):
		label += " (by hand)"
	return label


func _c(kelvin: float) -> String:
	return "--" if is_nan(kelvin) else "%.1f C" % (kelvin - 273.15)


## Which of this screen's gauges a trip belongs on, from the trip's own
## `watches` (M40): "coil" for the heater's tubes, "outlet" for its outlet
## temperature, "" for anything else. Nothing is assumed from the trip's name.
func _trip_gauge(trip: Dictionary) -> String:
	var watches: Dictionary = trip["watches"]
	if watches.has("coil") and int(watches["coil"]) == heater_id:
		return "coil"
	if watches.has("node") and int(watches["node"]) == heater_id and trip["limit"]["variable"] == "temperature":
		return "outlet"
	return ""


## What a trip watches, in words, from `watches` and its limit's variable.
func _watch_label(trip: Dictionary) -> String:
	var watches: Dictionary = trip["watches"]
	var variable: String = trip["limit"]["variable"]
	if watches.has("coil"):
		return "%s tubes" % _node(int(watches["coil"]))["name"]
	if watches.has("pipe"):
		return "%s %s" % [snapshot["edges"][int(watches["pipe"])]["name"], variable]
	var node := _node(int(watches["node"]))
	if node["kind"]["type"] == "furnace" and variable == "temperature":
		return "%s outlet" % node["name"]
	return "%s %s" % [node["name"], variable]


## A tagged value (a limit or a reading) in its display unit; `--` when absent.
func _value_text(value) -> String:
	if value == null:
		return "--"
	match value["variable"]:
		"temperature":
			return _c(float(value["k"]))
		"level":
			return "%.2f m" % float(value["m"])
		"pressure":
			return "%.2f bar" % (float(value["pa"]) / 1.0e5)
		"flow":
			return "%.2f kg/s" % float(value["kg_per_s"])
	return "?"


func _trip_sign(trip: Dictionary) -> String:
	return ">=" if trip["direction"] == "high" else "<="


## Who resets a trip and what the reset does, from its `reset` field (M40);
## absent means the default, a person's reset that restarts nothing.
func _reset_label(trip: Dictionary) -> String:
	var reset = trip.get("reset")
	if reset == null:
		return "reset by hand, restarts nothing"
	# A restart is the trips' to give only when nothing bars it (M43): the
	# heater's stop line in the FURNACE block says whether anything does.
	if reset["mode"] == "manual_restart":
		return "reset by hand, restarts unless barred%s" % _waits_for(trip)
	var under := "<" if trip["direction"] == "high" else ">"
	return (
		"resets itself %s %s, restarts unless barred%s"
		% [under, _value_text(reset["reset_at"]), _waits_for(trip)]
	)


## A trip's start permissives (M44), by name: the trips its restart waits for.
func _waits_for(trip: Dictionary) -> String:
	var names := PackedStringArray()
	for id in trip.get("restart_permissives", []):
		names.append(_trips()[int(id)]["name"])
	return "" if names.is_empty() else "; waits for %s" % ", ".join(names)


## The level fraction of a tank destination — plant.gd's rule, from the slate.
func _tank_fraction(id: int) -> float:
	var kind: Dictionary = _node(id)["kind"]
	var fractions: Array = kind["composition"]["mass_fractions"]
	var slate: Array = snapshot["slate"]
	var inverse := 0.0
	for i in fractions.size():
		if float(fractions[i]) > 0.0:
			inverse += float(fractions[i]) / float(slate[i]["density_kg_per_m3"])
	var level := float(kind["mass"]) * inverse / float(kind["area"])
	return clampf(level / float(kind["height"]), 0.0, 1.0)


func _readout(tick: int) -> String:
	var trips := ""
	for trip in _trips():
		trips += "  %s=%s" % [trip["name"], _trip_label(trip)]
	var loop = _loop()
	var loop_text := "" if loop == null else "  loop=%s" % loop["mode"]
	if pump_id >= 0:
		loop_text += "  pump=%s" % ("on" if _pump_on() else "off")
	return (
		"t=%5d  duty=%5.2f MW  coil=%7.2f C  outlet=%6.2f C  flow=%6.2f kg/s  leak=%5.3f kg/s  tubes=%s  fire=%5.2f MW%s%s"
		% [
			tick,
			_duty_w() / 1.0e6,
			_coil_k() - 273.15,
			_outlet_k() - 273.15,
			_outlet_flow(),
			_leak_kg_s(),
			_tubes_label(),
			_optional_w("tube_fire_w") / 1.0e6,
			loop_text,
			trips,
		]
	)


# ---------------------------------------------------------------- drawing

const FEED_POS := Vector2(60, 380)
const FURNACE_RECT := Rect2(190, 140, 220, 300)
const GAUGE_RECT := Rect2(440, 140, 26, 300)
const LEAK_POS := Vector2(600, 180)
const DEST_RECT := Rect2(650, 250, 100, 190)
const PANEL_X := 790.0

const BACKGROUND := Color(0.09, 0.10, 0.12)
const SHELL := Color(0.42, 0.45, 0.50)
const PIPE := Color(0.55, 0.58, 0.62)
const LIQUID := Color(0.20, 0.45, 0.75)
const STEEL := Color(0.50, 0.52, 0.56)
const HOT := Color(0.90, 0.30, 0.15)
const WHITE_HOT := Color(1.0, 0.92, 0.65)
const FLAME := Color(1.0, 0.55, 0.10)
const FIRE := Color(1.0, 0.30, 0.05)
const SPRAY := Color(0.45, 0.70, 0.95)
const INK := Color(0.88, 0.90, 0.93)
const DIM := Color(0.60, 0.63, 0.68)
const GOOD := Color(0.35, 0.80, 0.45)
const BAD := Color(0.95, 0.30, 0.25)
## A trip's limit on a gauge while it is armed; BAD once it has tripped.
const TRIP_MARK := Color(0.95, 0.78, 0.30)


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

	_draw_pipes()
	_draw_furnace()
	_draw_gauge()
	_draw_leak()
	_draw_destination()
	_draw_panel()

	draw_multiline_string(
		ThemeDB.fallback_font, Vector2(30, 556), message, HORIZONTAL_ALIGNMENT_LEFT,
		get_viewport_rect().size.x - 60, 15, 2, BAD if message_bad else GOOD
	)
	_text(
		Vector2(30, 600),
		"E emergency stop   R reset trips   A loop auto/manual   W/S setpoint   Up/Down fuel",
		DIM
	)
	_text(
		Vector2(30, 624),
		"P patch   N new tubes   K pump   Space pause   [ ] speed   1 trip   2 burn-out   3 self-reset   4 burst   5 pump trip",
		DIM
	)
	if halted != "":
		_text(Vector2(30, 90), "HALTED — %s" % halted, BAD)


func _draw_pipes() -> void:
	var inlet := FURNACE_RECT.position + Vector2(0, FURNACE_RECT.size.y - 60)
	draw_polyline(PackedVector2Array([FEED_POS, Vector2(FEED_POS.x, inlet.y), inlet]), PIPE, 6.0)
	draw_circle(FEED_POS, 14, PIPE)
	_text(FEED_POS + Vector2(-30, 36), PLANTS[plant_key]["feed"], DIM)
	_text(FEED_POS + Vector2(-30, 56), _c(float(_node(feed_id)["kind"]["temperature"])), DIM)

	var outlet := FURNACE_RECT.position + Vector2(FURNACE_RECT.size.x, 40)
	var to_dest := DEST_RECT.position + Vector2(DEST_RECT.size.x * 0.5, 0)
	draw_polyline(
		PackedVector2Array([outlet, Vector2(to_dest.x, outlet.y), to_dest]), PIPE, 6.0
	)
	_text(Vector2(LEAK_POS.x - 10, outlet.y + 26), "%.2f kg/s" % _outlet_flow(), DIM)
	_text(Vector2(LEAK_POS.x - 10, outlet.y + 46), "outlet %s" % _c(_outlet_k()), INK)
	# Every trip that watches the outlet, under the outlet's own reading (M40).
	var line := 0
	for trip in _trips():
		if _trip_gauge(trip) == "outlet":
			var tripped: bool = trip["state"]["status"] == "tripped"
			_text(
				Vector2(LEAK_POS.x - 10, outlet.y + 66 + line * 18),
				"trip %s %s" % [_trip_sign(trip), _value_text(trip["limit"])],
				BAD if tripped else TRIP_MARK,
				14
			)
			line += 1


## The firebox: burner flames sized by the duty, the coil coloured by its own
## temperature, and — when the tubes have burst — the leak's fire, sized by
## `tube_fire_w`. Every size is read, none is remembered from a command.
func _draw_furnace() -> void:
	draw_rect(FURNACE_RECT, Color(0.13, 0.12, 0.12))
	draw_rect(FURNACE_RECT, SHELL, false, 3.0)
	_text(FURNACE_RECT.position + Vector2(0, FURNACE_RECT.size.y + 24), "heater", INK)
	_text(
		FURNACE_RECT.position + Vector2(0, FURNACE_RECT.size.y + 46),
		"fired %.2f MW" % (_duty_w() / 1.0e6),
		FLAME if _duty_w() > 0.0 else DIM
	)

	var fire_w := _optional_w("tube_fire_w")
	if fire_w > 0.0:
		var height := minf(FURNACE_RECT.size.y - 20, 40.0 + sqrt(fire_w / 1.0e6) * 45.0)
		_flames(FURNACE_RECT.position + Vector2(20, FURNACE_RECT.size.y), FURNACE_RECT.size.x - 40, height, 6, FIRE)
	if _duty_w() > 0.0:
		var height := 20.0 + sqrt(_duty_w() / 1.0e6) * 40.0
		_flames(FURNACE_RECT.position + Vector2(40, FURNACE_RECT.size.y), FURNACE_RECT.size.x - 80, height, 4, FLAME)

	# The coil: a serpentine in the radiant section, steel -> red at the tubes'
	# failure limit -> white-hot toward the flame. Broken in the middle once burst.
	var coil_color := _coil_color()
	var left := FURNACE_RECT.position.x + 30
	var right := FURNACE_RECT.end.x - 30
	var top := FURNACE_RECT.position.y + 40
	var points := PackedVector2Array()
	for row in 7:
		var y := top + row * 22.0
		if row % 2 == 0:
			points.append_array([Vector2(left, y), Vector2(right, y)])
		else:
			points.append_array([Vector2(right, y), Vector2(left, y)])
	var below := FURNACE_RECT.position + Vector2(0, FURNACE_RECT.size.y)
	if _tubes_failed():
		var half := points.size() / 2
		draw_polyline(points.slice(0, half), coil_color, 5.0)
		draw_polyline(points.slice(half + 1), coil_color, 5.0)
		_text(below + Vector2(0, 68), "TUBES BURST", BAD, 18)
	else:
		draw_polyline(points, coil_color, 5.0)
	if fire_w > 0.0:
		_text(below + Vector2(120, 68), "tube fire %.1f MW" % (fire_w / 1.0e6), FIRE)
	# What the trips will do, or did, with the heater (M43): a dark furnace that
	# nothing will relight says so under itself.
	var stop = _trip_stop()
	if stop != null and not stop["barred_by"].is_empty():
		if stop["status"] == "not_restarted":
			_text(below + Vector2(0, 92), "DARK: WAITS FOR A PERSON (A or Up relights)", BAD, 16)
		else:
			_text(below + Vector2(0, 92), "a reset will NOT relight it", TRIP_MARK, 16)


func _coil_color() -> Color:
	var coil := _coil_k()
	var ambient := 293.15
	if coil <= _failure_k():
		return STEEL.lerp(HOT, clampf((coil - ambient) / (_failure_k() - ambient), 0.0, 1.0))
	return HOT.lerp(WHITE_HOT, clampf((coil - _failure_k()) / (_flame_k() - _failure_k()), 0.0, 1.0))


## The coil's thermometer. Its scale tops out a little above the tubes' failure
## limit until the coil passes it, then grows with the coil toward the flame —
## so a trip near 100 °C and a burst at 550 °C are both readable on the trip
## plant, and the burn-out's climb to the flame still fits.
func _draw_gauge() -> void:
	var low := 273.15
	var high := minf(_flame_k(), maxf(_failure_k() * 1.15, _coil_k() * 1.1))
	var rect := GAUGE_RECT
	draw_rect(rect, Color(0.13, 0.14, 0.16))
	var fraction := clampf((_coil_k() - low) / (high - low), 0.0, 1.0)
	draw_rect(
		Rect2(rect.position + Vector2(0, rect.size.y * (1.0 - fraction)), Vector2(rect.size.x, rect.size.y * fraction)),
		_coil_color()
	)
	draw_rect(rect, SHELL, false, 2.0)
	# The burst label moves up a line when a tube trip's label would print over
	# it (M43's plant fails its tubes 0.1 K above its 100 °C trip).
	var burst_dy := 0.0
	for trip in _trips():
		if _trip_gauge(trip) == "coil":
			var gap := (float(trip["limit"]["k"]) - _failure_k()) / (high - low) * rect.size.y
			if absf(gap) < 16.0:
				burst_dy = -16.0
	_marker(rect, low, high, _failure_k(), "burst %s" % _c(_failure_k()), BAD, burst_dy)
	if _flame_k() <= high + 0.5:
		_marker(rect, low, high, _flame_k(), "flame %s" % _c(_flame_k()), FLAME)
	# Every trip that watches the tubes, on the tubes' own scale (M40).
	for trip in _trips():
		if _trip_gauge(trip) == "coil":
			var tripped: bool = trip["state"]["status"] == "tripped"
			# An auto trip's reset point is a dim unlabelled line under its limit
			# (the two sit a few pixels apart on this scale), named in the
			# limit's own label.
			var label := "trip %s" % _c(float(trip["limit"]["k"]))
			var reset = trip.get("reset")
			if reset != null and reset["mode"] == "auto":
				var reset_k := float(reset["reset_at"]["k"])
				_marker(rect, low, high, reset_k, "", DIM)
				label += ", resets %s" % _c(reset_k)
			_marker(rect, low, high, float(trip["limit"]["k"]), label, BAD if tripped else TRIP_MARK)
	_text(rect.position + Vector2(-6, -12), "coil", DIM)
	_text(rect.position + Vector2(-6, rect.size.y + 24), _c(_coil_k()), INK)


func _marker(
	rect: Rect2, low: float, high: float, kelvin: float, label: String, color: Color, label_dy: float = 0.0
) -> void:
	var y := rect.end.y - rect.size.y * clampf((kelvin - low) / (high - low), 0.0, 1.0)
	draw_line(Vector2(rect.position.x - 4, y), Vector2(rect.end.x + 4, y), color, 2.0)
	_text(Vector2(rect.end.x + 8, y + 5 + label_dy), label, color, 14)


## The leak, sized by the reported mass flow. Orange when the tubes have burst
## and the leak is burning, blue for a hole that leaks without lighting.
func _draw_leak() -> void:
	var flow := _leak_kg_s()
	if flow <= 0.0:
		return
	var color := FIRE if _optional_w("tube_fire_w") > 0.0 else SPRAY
	var reach := 24.0 + flow * 30.0
	for i in 7:
		var spread := deg_to_rad(240.0 + i * 10.0)
		draw_line(LEAK_POS, LEAK_POS + Vector2(cos(spread), sin(spread)) * reach, color, 3.0)
	_text(LEAK_POS + Vector2(-24, -reach - 10), "leak %.3f kg/s" % flow, color)


func _draw_destination() -> void:
	var node := _node(destination_id)
	var label: String = PLANTS[plant_key]["destination"]
	if node["kind"]["type"] == "tank":
		var fraction := _tank_fraction(destination_id)
		var warmth := clampf((float(node["kind"]["temperature"]) - 293.15) / 40.0, 0.0, 1.0)
		draw_rect(
			Rect2(
				DEST_RECT.position + Vector2(0, DEST_RECT.size.y * (1.0 - fraction)),
				Vector2(DEST_RECT.size.x, DEST_RECT.size.y * fraction)
			),
			LIQUID.lerp(HOT, warmth)
		)
		draw_rect(DEST_RECT, SHELL, false, 3.0)
		_text(DEST_RECT.position + Vector2(0, DEST_RECT.size.y + 24), label, INK)
		_text(DEST_RECT.position + Vector2(0, DEST_RECT.size.y + 46), _c(float(node["kind"]["temperature"])), INK)
	else:
		var box := Rect2(DEST_RECT.position + Vector2(10, DEST_RECT.size.y - 60), Vector2(80, 60))
		var top := DEST_RECT.position + Vector2(DEST_RECT.size.x * 0.5, 0)
		draw_line(top, Vector2(top.x, box.position.y), PIPE, 6.0)
		draw_rect(box, SHELL, false, 3.0)
		_text(box.position + Vector2(0, box.size.y + 24), label, INK)


## The right-hand panel: the furnace's books, each trip with its reading against
## its limit, and the loop.
func _draw_panel() -> void:
	var y := 120.0
	_text(Vector2(PANEL_X, y), "FURNACE", DIM)
	y += 24
	_text(Vector2(PANEL_X, y), "fired          %6.2f MW" % (_duty_w() / 1.0e6), INK)
	y += 22
	_text(Vector2(PANEL_X, y), "up the stack   %6.2f MW" % (_optional_w("flue_loss_w") / 1.0e6), INK)
	y += 22
	_text(Vector2(PANEL_X, y), "tube fire      %6.2f MW" % (_optional_w("tube_fire_w") / 1.0e6), FIRE if _optional_w("tube_fire_w") > 0.0 else INK)
	y += 22
	_text(Vector2(PANEL_X, y), "tubes          %s" % _tubes_label(), BAD if _tubes_failed() else GOOD)
	# The trips' stop (M43): whether they will relight it, and if not, why.
	y += 22
	var stop = _trip_stop()
	var reasons := _bar_lines()
	var stop_color := DIM
	if stop != null:
		stop_color = TRIP_MARK if reasons.is_empty() else BAD
	_text(Vector2(PANEL_X, y), "trip stop      %s" % _stop_label(), stop_color)
	if stop != null and reasons.is_empty():
		y += 18
		_text(Vector2(PANEL_X + 12, y), "relit when the trips let go", DIM, 14)
	elif not reasons.is_empty():
		y += 18
		_text(Vector2(PANEL_X + 12, y), "a person relights it, because:", BAD, 14)
		for reason in reasons:
			y += 18
			_text(Vector2(PANEL_X + 12, y), "- %s" % reason, BAD, 14)
	if pump_id >= 0:
		y += 22
		_text(
			Vector2(PANEL_X, y),
			"%-15s%s" % [PLANTS[plant_key]["pump"], "running" if _pump_on() else "STOPPED (K starts it)"],
			GOOD if _pump_on() else BAD
		)

	y += 40
	_text(Vector2(PANEL_X, y), "TRIPS", DIM)
	y += 24
	if _trips().is_empty():
		_text(Vector2(PANEL_X, y), "none on this plant", DIM)
		y += 22
	for trip in _trips():
		var tripped: bool = trip["state"]["status"] == "tripped"
		var state: Dictionary = trip["state"]
		_text(Vector2(PANEL_X, y), trip["name"], INK)
		_text(
			Vector2(PANEL_X + 160, y),
			"TRIPPED t=%d" % int(state["at_tick"]) if tripped else "armed",
			BAD if tripped else GOOD
		)
		y += 20
		_text(
			Vector2(PANEL_X + 12, y),
			(
				"%s %s   trips %s %s%s"
				% [
					_watch_label(trip),
					_value_text(trip.get("measurement")),
					_trip_sign(trip),
					_value_text(trip["limit"]),
					"   by hand" if state.get("by_hand", false) else "",
				]
			),
			DIM,
			14
		)
		y += 18
		_text(Vector2(PANEL_X + 12, y), _reset_label(trip), DIM, 14)
		y += 24

	y += 14
	_text(Vector2(PANEL_X, y), "CONTROL LOOP", DIM)
	y += 24
	var loop = _loop()
	if loop == null:
		_text(Vector2(PANEL_X, y), "none — fired by hand", DIM)
		return
	var measured = loop.get("measurement")
	_text(Vector2(PANEL_X, y), "%s   %s" % [loop["name"], str(loop["mode"]).to_upper()], GOOD if loop["mode"] == "auto" else BAD)
	y += 22
	_text(
		Vector2(PANEL_X, y),
		"target %s   reads %s   output %.0f%%"
		% [
			_c(float(loop["setpoint"]["k"])),
			_c(NAN if measured == null else float(measured["k"])),
			float(loop["output"]) * 100.0,
		],
		INK,
		14
	)


func _flames(base_left: Vector2, width: float, height: float, count: int, color: Color) -> void:
	var step := width / count
	for i in count:
		var x := base_left.x + step * (i + 0.5)
		draw_colored_polygon(
			PackedVector2Array(
				[
					Vector2(x - step * 0.4, base_left.y),
					Vector2(x, base_left.y - height),
					Vector2(x + step * 0.4, base_left.y),
				]
			),
			color
		)


func _text(at: Vector2, text: String, color: Color, size: int = 16) -> void:
	draw_string(ThemeDB.fallback_font, at, text, HORIZONTAL_ALIGNMENT_LEFT, -1, size, color)


func _halt(reason: String) -> void:
	halted = reason
	push_error("furnace: %s" % reason)
	print("furnace: HALTED — %s" % reason)
	queue_redraw()
