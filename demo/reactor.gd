# M57.5's demonstrated criterion: a screen for M57, a reactor whose own heat sets
# its temperature (docs/DESIGN.md §62.13).
#
# M57 built a catalyst bed whose cracking releases heat faster the hotter it
# runs, so it has a calm state and a runaway one; a trip that cuts the preheater;
# and a quench that a loop opens as the bed heats. It ran only in tests and the
# CLI. This scene draws it, on any of four shipped plants, each loaded unchanged:
#
#   calm    scenarios/hydrocracker_bed.toml (M57.2) — one bed on a calm day.
#           Warm its feed 20 K and it runs away; bring the feed back and the bed
#           STAYS hot. Only a much colder feed brings it back.
#   trip    scenarios/hydrocracker_trip.toml (M57.3) — the bed behind a fired
#           preheater whose firing has crept up, and the trip that cuts the fire
#           at 400 °C.
#   quench  scenarios/hydrocracker_quench.toml (M57.4) — two beds, the second
#           held at 380 °C by a loop that sets the flow of cooled product mixed in
#           ahead of it, with a backup trip.
#   wall    scenarios/hydrocracker_wall_failure.toml (M58) — the runaway bed
#           loaded mid-creep: at 550 °C its wall fails, the leak sprays out of the
#           product line and burns, and a tenth of that fire heats the bed. Patch
#           the hole (P), chill the feed hard, and repair the wall (M) once the
#           bed is back under its limit — the engine refuses the repair before.
#   hydrogen  scenarios/hydrocracker_hydrogen_wall.toml (M59.5) — a bed with its
#           hydrogen loop and no trip, loaded hot: it eats its loop down, and its
#           wall fails with TWO holes, one on the product line and one on the
#           hydrogen's outlet. The loop empties through its hole. P patches both;
#           stop the oil and the refilled hydrogen sweeps the bed down.
#   h2quench  scenarios/hydrocracker_hydrogen_quench.toml (M59.3) — two beds in
#           one hydrogen loop, the second held at 400 °C by cold hydrogen mixed in
#           ahead of it, and a backup trip that DEPRESSURES the loop (opens the
#           vent, shuts the makeup). D opens or closes the vent by hand.
#
# Since M58 every bed's wall can fail (docs/DESIGN.md §63): the calm plant's
# runaway breaks its wall too, and its story patches and repairs it. Since M59
# a plant may have a hydrogen loop (§64), drawn under the oil line: the
# separator's pressure, the compressor, the gas heater, a pass beside each bed,
# the makeup and the vent, and a hydrogen quench where there is one.
#
# Each plant runs on the solver its own file names — Newton, for all six
# (§62.13 decision 1; ledger row A27).
#
# Same rule as plant.gd, furnace.gd and pump.gd: this script computes nothing
# physical. Every number it draws is a snapshot field, every marker is a limit or
# target the snapshot carries, and every refusal is the engine's own message. The
# only arithmetic is display units — kelvin to °C, watts to MW, fractions to
# percent.
#
# The plants take minutes to hours to tell their stories, so the screen keeps a
# MEMORY: one sample every SAMPLE_TICKS, drawn two ways. The time chart is each
# bed's temperature and the temperature of the oil entering it over the last
# half hour — the creep before a runaway. The trail is the same samples as dots
# at (entering oil, bed): warming the feed and cooling it again traces the jump
# up and the bed staying hot. Neither draws a curve the engine did not report.
#
# Interactive:  godot --path . res://demo/reactor.tscn
#               (keys listed on screen and in _input; 1 to 6 switch plants)
# Recorded:     godot --headless --path . res://demo/reactor.tscn --quit-after 400000 -- --auto [--plant=trip|quench|wall|hydrogen|h2quench]
#               ...runs that plant's scripted timeline (TIMELINES) and prints a
#               `t=` line every PRINT_EVERY ticks plus one line per command and
#               event. Those lines are the observation in ROADMAP.md's M57.5
#               section; `crates/godot-ext/tests/reactor_screen.rs` replays them.
# Screenshots:  add --shots=<dir> to a WINDOWED --auto run (headless renders
#               nothing) to save a PNG at each tick in AUTO_SHOT_TICKS.
extends Node2D

## The four plants, and the names on each the scene draws. A name missing from
## a plant is a load-time halt, not a guess. Optional entries are absent on the
## plants that do not have them.
##
## Speeds are ticks of 0.1 s per 60 Hz frame. The editor loads the extension's
## DEBUG build, where a tick costs about 0.17 ms on the calm bed, 0.27 ms on the
## trip plant, 1.1 ms on the quench plant, 1.7 ms on the hydrogen wall and
## 2.7 ms on the hydrogen quench on a quiet machine (twice that on a busy one),
## and reading the snapshot about half as much again (measured by the recorded
## runs' last line). Each default keeps a
## frame's simulation under about 8 ms; `]` goes faster at the frame rate's cost.
const PLANTS := {
	"calm":
	{
		"scenario": "res://scenarios/hydrocracker_bed.toml",
		"title": "A hydrocracker bed on a calm day — push it, then try to bring it back",
		"feed": "vgo_feed",
		"feed_valve": "feed_valve",
		"beds": ["bed"],
		"product": "product",
		"ticks_per_frame": 16,
		"max_ticks_per_frame": 128,
	},
	"trip":
	{
		"scenario": "res://scenarios/hydrocracker_trip.toml",
		"title": "A bed behind a preheater fired too hard, and the trip that cuts the fire",
		"feed": "vgo_feed",
		"feed_valve": "feed_valve",
		"preheater": "preheater",
		"beds": ["bed"],
		"product": "product",
		"ticks_per_frame": 8,
		"max_ticks_per_frame": 64,
	},
	"quench":
	{
		"scenario": "res://scenarios/hydrocracker_quench.toml",
		"title": "Two beds, and a quench of cooled product that holds the second at 380 °C",
		"feed": "vgo_feed",
		"feed_valve": "feed_valve",
		"preheater": "preheater",
		"beds": ["bed1", "bed2"],
		"mix": "mix",
		"cooler": "product_cooler",
		"drum": "product_drum",
		"quench_pipe": "quench_line",
		"product": "product",
		"ticks_per_frame": 4,
		"max_ticks_per_frame": 32,
	},
	"wall":
	{
		"scenario": "res://scenarios/hydrocracker_wall_failure.toml",
		"title": "A runaway that breaks the reactor wall — patch it, chill the feed, repair it",
		"feed": "vgo_feed",
		"feed_valve": "feed_valve",
		"beds": ["bed"],
		"product": "product",
		"ticks_per_frame": 16,
		"max_ticks_per_frame": 128,
	},
	# The hydrogen plants name their loop too. Each bed's hydrogen pass is read
	# from the snapshot (the `bed_hydrogen` node whose `bed` is it), not named.
	"hydrogen":
	{
		"scenario": "res://scenarios/hydrocracker_hydrogen_wall.toml",
		"title": "A hydrogen bed whose runaway breaks the wall twice over — patch both holes, stop the oil",
		"feed": "vgo_feed",
		"feed_valve": "feed_valve",
		"beds": ["bed"],
		"product": "product",
		"separator": "separator",
		"compressor": "compressor",
		"gas_heater": "gas_heater",
		"gas_cooler": "gas_cooler",
		"makeup_valve": "makeup_valve",
		"vent_valve": "vent_valve",
		"ticks_per_frame": 4,
		"max_ticks_per_frame": 32,
	},
	"h2quench":
	{
		"scenario": "res://scenarios/hydrocracker_hydrogen_quench.toml",
		"title": "Two beds in one hydrogen loop, a cold-hydrogen quench, and a trip that vents the loop",
		"feed": "vgo_feed",
		"feed_valve": "feed_valve",
		"beds": ["bed1", "bed2"],
		"product": "product",
		"quench_pipe": "quench_line",
		"separator": "separator",
		"compressor": "compressor",
		"gas_heater": "gas_heater",
		"gas_cooler": "gas_cooler",
		"gas_mix": "mix",
		"makeup_valve": "makeup_valve",
		"vent_valve": "vent_valve",
		"ticks_per_frame": 2,
		"max_ticks_per_frame": 16,
	},
}

## Scripted timelines for the recorded runs, in ticks of 0.1 s. Each beat is one
## the bridge test `crates/godot-ext/tests/reactor_screen.rs` replays with the
## same commands and pins the outcome of.
##
## calm: at 1 min the feed is warmed from 365 to 385 °C. The bed creeps, then
## runs away; at 550 °C, 21 min 36 s in, its wall fails and the leak burns (M58).
## At 25 min the feed goes back to 365 °C and the bed STAYS hot. At 35 min the
## hole is patched and the feed cooled to 220 °C: the bed holds above 550 °C for
## a quarter of an hour, then falls. At 55 min the wall is repaired and the feed
## goes back to 365 °C, and the bed returns to its calm state.
const AUTO_CALM := {
	600: ["feed_c_385"],
	15000: ["feed_c_365"],
	21000: ["patch", "feed_c_220"],
	33000: ["repair", "feed_c_365"],
	48000: ["quit"],
}
## trip: from the file's start the bed creeps; the trip fires at 400 °C and cuts
## the preheater. A reset 10 s later is REFUSED — the bed is still past its
## limit. At 12 min the reset is admitted and relights nothing; at 13 min the
## preheater is relit by hand at 14 MW, below the 16.86 MW that ran it away,
## and the bed settles near 386 °C (within 0.1 K of it by 60 min).
const AUTO_TRIP := {
	3800: ["reset"],
	7200: ["reset"],
	7800: ["duty_mw_14"],
	36000: ["quit"],
}
## quench: the loop catches the second bed at 380 °C. At 2 min it is put in
## MANUAL and the quench cut by hand to its 0.25 kg/s minimum: the bed creeps
## into the trip, which cuts the preheater. A reset at 15 min is refused; at
## 18 min 20 s it is admitted and both loops are put back in AUTO. The preheater
## relights under its loop and the quench holds the bed at 380 °C again.
const AUTO_QUENCH := {
	1200: ["manual:bed2_temperature", "target_kgs:quench_flow:0.25"],
	9000: ["reset"],
	11000: ["reset", "auto:bed2_temperature", "auto:preheat_outlet"],
	24000: ["quit"],
}
## wall: the bed creeps from 450 °C; its wall fails at 550 °C, 4 min 35 s in, and
## the leak burns at about 55 MW. At 10 min a repair is REFUSED (the hole is
## open); the hole is patched; a repair is REFUSED again (the bed is past its
## limit); the feed is chilled to 200 °C. At 20 min, the bed under 550 °C, the
## repair is admitted. At 25 min the feed goes back to 365 °C, and the bed
## settles on the calm day's 374 °C.
const AUTO_WALL := {
	6000: ["repair", "patch", "repair", "feed_c_200"],
	12000: ["repair"],
	15000: ["feed_c_365"],
	33000: ["quit"],
}
## hydrogen (M59.5): the bed, loaded at 460 °C, eats its loop down from 150 bar;
## its wall fails at 550 °C, about 9 min in, opening both holes, and the loop
## empties through the hydrogen one. At 15 min a repair is REFUSED (the oil's
## hole); P patches BOTH holes; a repair is REFUSED again (the heat); the oil is
## stopped. The makeup refills the loop and the hydrogen sweeps the bed down: at
## 25 min the repair is admitted. At 40 min, the bed near 410 °C, the gas heater's
## target and the feed go to 385 °C and the oil is opened again; the bed settles
## near 404 °C.
const AUTO_HYDROGEN := {
	9000: ["repair", "patch", "repair", "valve_pct_0"],
	15000: ["repair"],
	24000: ["target_c:gas_heater_outlet:385", "feed_c_385", "valve_pct_100"],
	36000: ["quit"],
}
## h2quench (M59.5): the loops hold the second bed at 400 °C. At 2 min its loop
## is put in MANUAL and the quench cut by hand to 0.1 kg/s: the bed creeps into
## the trip, which vents the loop and shuts the makeup. At 15 min a reset and
## closing the vent by hand are both REFUSED — the bed is past its limit and the
## trip holds the vent. At 22 min the reset is admitted, the vent closed by hand,
## and both loops put back in AUTO: the makeup refills the loop and the quench
## holds the bed at 400 °C again.
const AUTO_H2QUENCH := {
	1200: ["manual:bed2_temperature", "target_kgs:quench_flow:0.1"],
	9000: ["reset", "vent_close"],
	13200: ["reset", "vent_close", "auto:separator_pressure", "auto:bed2_temperature"],
	36000: ["quit"],
}
const TIMELINES := {
	"calm": AUTO_CALM,
	"trip": AUTO_TRIP,
	"quench": AUTO_QUENCH,
	"wall": AUTO_WALL,
	"hydrogen": AUTO_HYDROGEN,
	"h2quench": AUTO_H2QUENCH,
}
const AUTO_SHOT_TICKS := {
	"calm": [600, 6000, 12000, 15000, 21000, 30000, 33000, 47999],
	"trip": [3600, 3800, 6000, 7200, 12000, 35999],
	"quench": [1200, 6000, 8800, 9000, 11000, 15000, 23999],
	"wall": [2700, 2760, 6000, 9000, 12000, 15000, 32999],
	"hydrogen": [3000, 5400, 7200, 9000, 12000, 15000, 24000, 35999],
	"h2quench": [1200, 5600, 9000, 13200, 18000, 35999],
}
## Print a `t=` line every this many ticks (1 minute) in a recorded run.
const PRINT_EVERY := 600
## One chart sample every this many ticks (10 s of plant time).
const SAMPLE_TICKS := 100
## The time chart's span, in samples: 30 minutes.
const CHART_SAMPLES := 180
## The trail's memory, in samples: 2 hours.
const TRAIL_SAMPLES := 720

## Key steps: 5 K on a temperature, 5% on a valve, 0.5 MW of firing, 5 MW of
## cooling; a loop target by 5 K, 0.25 kg/s or 0.5 m, by its own variable.
const TEMPERATURE_STEP_K := 5.0
const VALVE_STEP := 0.05
const FIRING_STEP_W := 5.0e5
const COOLING_STEP_W := 5.0e6
const FLOW_STEP_KG_S := 0.25
const LEVEL_STEP_M := 0.5

@onready var sim: RefinerySim = $Sim

var plant_key := "calm"
var feed_id := -1
var feed_valve_id := -1
var preheater_id := -1
var mix_id := -1
var cooler_id := -1
var drum_id := -1
var product_id := -1
var quench_pipe_id := -1
## The hydrogen loop, on the plants that have one; -1 elsewhere.
var separator_id := -1
var compressor_id := -1
var gas_heater_id := -1
var gas_cooler_id := -1
var gas_mix_id := -1
var makeup_id := -1
var vent_id := -1
## The edge out of the compressor: its flow and the pressure it delivers to.
var compressor_out_edge := -1
## Per bed, in the plant's order: node id, the edge into it, the edge out of it.
var bed_ids: Array[int] = []
var bed_in_edges: Array[int] = []
var bed_out_edges: Array[int] = []
## Per bed: the pipes a failed wall's holes sit on, the oil's first (M58, and
## the hydrogen's since M59.4) — what P patches, and where a leak is reported.
var bed_hole_pipes: Array = []
## Per bed: its hydrogen pass and the edge into it; -1 on a bed without one.
var pass_ids: Array[int] = []
var pass_in_edges: Array[int] = []
## Where each bed's vessel was last drawn [px]: its pass is drawn under it.
var bed_xs: Array[float] = []
## Where "vgo" sits in the slate: the uncracked share of a bed's outflow.
var vgo_index := -1
var snapshot: Dictionary = {}
## Each: {"tick": int, "beds": [[bed_k, inlet_k], ...]}. Newest last.
var samples: Array = []
var selected_loop := 0
var halted := ""
var auto_run := false
var shots_dir := ""
var paused := false
var finished := false
var shooting := false
var ticks_per_frame := 1
var max_ticks_per_frame := 64
var message := ""
var message_bad := false
## Last-seen trip states, loop modes and quench readings, for event lines.
var seen_trips: Array = []
var seen_modes: Array = []
var seen_cold := ""
var seen_walls: Array = []
## Wall time [µs] of the engine's ticks and of reading its snapshot, printed at
## the end of a recorded run: what sets each plant's speed (§62.13).
var tick_usec_total := 0
var tick_usec_worst := 0
var read_usec_total := 0
var read_usec_worst := 0
var ticks_timed := 0
var reads_timed := 0


func _ready() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg == "--auto":
			auto_run = true
		elif arg.begins_with("--plant="):
			plant_key = arg.trim_prefix("--plant=")
		elif arg.begins_with("--shots="):
			shots_dir = arg.trim_prefix("--shots=")
	if DisplayServer.get_name() != "headless":
		get_window().size = Vector2i(1440, 810)
	if not PLANTS.has(plant_key):
		_halt("no plant '%s' (calm | trip | quench | wall | hydrogen | h2quench)" % plant_key)
		return
	_load(plant_key)


func _load(key: String) -> void:
	plant_key = key
	halted = ""
	finished = false
	snapshot = {}
	samples = []
	selected_loop = 0
	seen_trips = []
	seen_modes = []
	seen_cold = ""
	seen_walls = []
	var plant: Dictionary = PLANTS[key]
	ticks_per_frame = plant["ticks_per_frame"]
	max_ticks_per_frame = plant["max_ticks_per_frame"]

	var err = JSON.parse_string(sim.load_scenario(plant["scenario"]))
	if err != null:
		_halt("load failed: %s" % err)
		return

	feed_id = sim.node_id(plant["feed"])
	feed_valve_id = sim.node_id(plant["feed_valve"])
	product_id = sim.node_id(plant["product"])
	preheater_id = sim.node_id(plant["preheater"]) if plant.has("preheater") else -1
	mix_id = sim.node_id(plant["mix"]) if plant.has("mix") else -1
	cooler_id = sim.node_id(plant["cooler"]) if plant.has("cooler") else -1
	drum_id = sim.node_id(plant["drum"]) if plant.has("drum") else -1
	quench_pipe_id = sim.edge_id(plant["quench_pipe"]) if plant.has("quench_pipe") else -1
	separator_id = sim.node_id(plant["separator"]) if plant.has("separator") else -1
	compressor_id = sim.node_id(plant["compressor"]) if plant.has("compressor") else -1
	gas_heater_id = sim.node_id(plant["gas_heater"]) if plant.has("gas_heater") else -1
	gas_cooler_id = sim.node_id(plant["gas_cooler"]) if plant.has("gas_cooler") else -1
	gas_mix_id = sim.node_id(plant["gas_mix"]) if plant.has("gas_mix") else -1
	makeup_id = sim.node_id(plant["makeup_valve"]) if plant.has("makeup_valve") else -1
	vent_id = sim.node_id(plant["vent_valve"]) if plant.has("vent_valve") else -1
	var missing := feed_id < 0 or feed_valve_id < 0 or product_id < 0
	for optional in [
		["preheater", preheater_id], ["mix", mix_id], ["cooler", cooler_id], ["drum", drum_id],
		["quench_pipe", quench_pipe_id], ["separator", separator_id], ["compressor", compressor_id],
		["gas_heater", gas_heater_id], ["gas_cooler", gas_cooler_id], ["gas_mix", gas_mix_id],
		["makeup_valve", makeup_id], ["vent_valve", vent_id],
	]:
		if plant.has(optional[0]) and int(optional[1]) < 0:
			missing = true
	bed_ids = []
	for name in plant["beds"]:
		var id := sim.node_id(name)
		missing = missing or id < 0
		bed_ids.append(id)
	if missing:
		_halt("this scenario does not have the names the scene expects")
		return

	snapshot = JSON.parse_string(sim.snapshot_json())
	# A bed's inlet and outlet are the edges that end and start at it — read from
	# the snapshot's own wiring, so a leak split ahead of a bed is followed.
	bed_in_edges = []
	bed_out_edges = []
	for id in bed_ids:
		var into := -1
		var out := -1
		for edge in snapshot["edges"]:
			if int(edge["to"]) == id:
				into = int(edge["id"])
			if int(edge["from"]) == id:
				out = int(edge["id"])
		if into < 0 or out < 0 or _node(id)["kind"]["type"] != "bed_reactor":
			_halt("'%s' is not a bed with one line in and one out" % _node(id)["name"])
			return
		bed_in_edges.append(into)
		bed_out_edges.append(out)
	# Each hole a failed wall opens is an orifice edge hanging off a junction the
	# loader split into its pipe (§3b); `puncture_pipe` names the pipe, which is
	# the edge ending at that junction. Read from the wall's own list — the oil's
	# hole, then the hydrogen's (M59.4) — so P patches every hole the repair asks
	# for.
	bed_hole_pipes = []
	for id in bed_ids:
		var wall: Dictionary = _node(id)["kind"]["wall"]
		var holes: Array = [wall.get("hole")]
		if wall.has("hydrogen_leak"):
			holes.append(wall["hydrogen_leak"].get("hole"))
		var pipes: Array[int] = []
		for hole in holes:
			var pipe := -1
			if hole != null:
				var junction := int(_edge(int(hole))["from"])
				for edge in snapshot["edges"]:
					if int(edge["to"]) == junction:
						pipe = int(edge["id"])
			if pipe < 0:
				_halt("%s's wall has a hole on no pipe the scene can find" % _node(id)["name"])
				return
			pipes.append(pipe)
		bed_hole_pipes.append(pipes)
	# Each bed's hydrogen pass: the `bed_hydrogen` node that names it.
	pass_ids = []
	pass_in_edges = []
	for id in bed_ids:
		var pass_id := -1
		var pass_in := -1
		for node in snapshot["nodes"]:
			if node["kind"]["type"] == "bed_hydrogen" and int(node["kind"]["bed"]) == id:
				pass_id = int(node["id"])
		for edge in snapshot["edges"]:
			if pass_id >= 0 and int(edge["to"]) == pass_id:
				pass_in = int(edge["id"])
		pass_ids.append(pass_id)
		pass_in_edges.append(pass_in)
	if separator_id >= 0 and pass_ids.has(-1):
		_halt("this plant names a hydrogen loop, but a bed has no hydrogen pass")
		return
	compressor_out_edge = -1
	for edge in snapshot["edges"]:
		if compressor_id >= 0 and int(edge["from"]) == compressor_id:
			compressor_out_edge = int(edge["id"])
	vgo_index = -1
	for i in snapshot["slate"].size():
		if snapshot["slate"][i]["name"] == "vgo":
			vgo_index = i
	if vgo_index < 0:
		_halt("this scenario has no 'vgo' component")
		return

	_say("loaded %s" % plant["scenario"], false)
	print("reactor: loaded %s" % plant["scenario"])
	queue_redraw()


func _physics_process(_delta: float) -> void:
	if halted != "" or paused or finished or shooting:
		return
	for i in ticks_per_frame:
		if not _step(i == ticks_per_frame - 1):
			break
	queue_redraw()


## One engine tick, then the scripted timeline for it. False when the scene
## must stop ticking this frame (a halt, a screenshot, the end of the run).
##
## Reading the snapshot costs about what a tick does, so an interactive frame
## reads it only on its `last` tick and on a chart sample; events then show at
## the frame they are seen in. A recorded run reads every tick, so its event
## lines carry the tick they happened on.
func _step(last: bool) -> bool:
	var start := Time.get_ticks_usec()
	var err = JSON.parse_string(sim.tick())
	var spent := Time.get_ticks_usec() - start
	tick_usec_total += spent
	tick_usec_worst = maxi(tick_usec_worst, spent)
	ticks_timed += 1
	if err != null:
		_halt("tick %d: %s" % [sim.tick_index(), err["message"]])
		return false
	var tick := sim.tick_index()
	if not (auto_run or last or tick % SAMPLE_TICKS == 0):
		return true
	start = Time.get_ticks_usec()
	snapshot = JSON.parse_string(sim.snapshot_json())
	spent = Time.get_ticks_usec() - start
	read_usec_total += spent
	read_usec_worst = maxi(read_usec_worst, spent)
	reads_timed += 1
	if tick % SAMPLE_TICKS == 0:
		_sample(tick)
	_report_events(tick)

	if not auto_run:
		return true
	if tick % PRINT_EVERY == 0:
		print(_readout(tick))
	var timeline: Dictionary = TIMELINES[plant_key]
	if timeline.has(tick):
		for action in timeline[tick]:
			if action == "quit":
				print(
					(
						"reactor: %d ticks: a tick %.3f ms on average, worst %.3f ms; a snapshot read %.3f ms on average, worst %.3f ms"
						% [
							ticks_timed,
							tick_usec_total / 1000.0 / maxi(1, ticks_timed),
							tick_usec_worst / 1000.0,
							read_usec_total / 1000.0 / maxi(1, reads_timed),
							read_usec_worst / 1000.0,
						]
					)
				)
				finished = true
				get_tree().quit()
				return false
			_do(action)
	if shots_dir != "" and tick in AUTO_SHOT_TICKS[plant_key]:
		_shoot(tick)
		return false
	return true


## Keep this tick's bed and inlet temperatures for both charts.
func _sample(tick: int) -> void:
	var beds: Array = []
	for i in bed_ids.size():
		beds.append([_bed_k(i), _inlet_k(i)])
	samples.append({"tick": tick, "beds": beds})
	if samples.size() > TRAIL_SAMPLES:
		samples.pop_front()


## Print a line when a trip latches or resets, a loop changes mode, or a quench
## stops (or starts again) being colder than what it joins — the engine's
## account, read from the snapshot.
func _report_events(tick: int) -> void:
	var trips: Array = []
	for trip in _trips():
		trips.append(_trip_label(trip))
	if seen_trips.size() == trips.size():
		for i in trips.size():
			if trips[i] != seen_trips[i]:
				_event(tick, "trip %s -> %s" % [_trips()[i]["name"], trips[i]], false)
	seen_trips = trips

	var modes: Array = []
	for loop in _loops():
		modes.append(loop["mode"])
	if seen_modes.size() == modes.size():
		for i in modes.size():
			if modes[i] != seen_modes[i]:
				_event(tick, "loop %s -> %s" % [_loops()[i]["name"], str(modes[i]).to_upper()], false)
	seen_modes = modes

	var cold := ""
	for loop in _loops():
		if loop.has("quench"):
			cold += "%s:%s " % [loop["name"], "cold" if _quench_cold(loop) else "NOT cold"]
	if seen_cold != "" and cold != seen_cold:
		_event(tick, "quench -> %s" % cold.strip_edges(), true)
	seen_cold = cold

	var walls: Array = []
	for i in bed_ids.size():
		walls.append(_wall_failed(i))
	if seen_walls.size() == walls.size():
		for i in walls.size():
			if walls[i] != seen_walls[i]:
				var name: String = _node(bed_ids[i])["name"]
				_event(tick, "%s wall -> %s" % [name, "FAILED" if walls[i] else "repaired"], walls[i])
	seen_walls = walls


func _event(tick: int, text: String, bad: bool) -> void:
	if auto_run:
		print("reactor: t=%d  %s" % [tick, text])
	if bad:
		_say("t=%d: %s" % [tick, text], true)


## Ticking holds until the frame showing `tick` has been drawn and saved.
func _shoot(tick: int) -> void:
	shooting = true
	queue_redraw()
	await RenderingServer.frame_post_draw
	var path := "%s/reactor_%s_t%05d.png" % [shots_dir, plant_key, tick]
	var image := get_viewport().get_texture().get_image()
	if image != null:
		image.save_png(path)
		print("reactor: saved %s" % path)
	shooting = false


func _input(event: InputEvent) -> void:
	if not (event is InputEventKey and event.pressed and not event.echo):
		return
	match event.keycode:
		KEY_1:
			_load("calm")
		KEY_2:
			_load("trip")
		KEY_3:
			_load("quench")
		KEY_4:
			_load("wall")
		KEY_5:
			_load("hydrogen")
		KEY_6:
			_load("h2quench")
		KEY_D:
			_do("vent_toggle")
		KEY_SPACE:
			paused = not paused
		KEY_BRACKETLEFT:
			ticks_per_frame = maxi(1, ticks_per_frame / 2)
		KEY_BRACKETRIGHT:
			ticks_per_frame = mini(max_ticks_per_frame, ticks_per_frame * 2)
		KEY_W:
			_do("feed_warmer")
		KEY_S:
			_do("feed_cooler")
		KEY_UP:
			_do("valve_up")
		KEY_DOWN:
			_do("valve_down")
		KEY_F:
			_do("firing_up")
		KEY_V:
			_do("firing_down")
		KEY_C:
			_do("cooling_up")
		KEY_X:
			_do("cooling_down")
		KEY_TAB:
			var count := _loops().size()
			if count > 0:
				selected_loop = (selected_loop + 1) % count
		KEY_A:
			_do("toggle_mode")
		KEY_Q:
			_do("target_up")
		KEY_Z:
			_do("target_down")
		KEY_E:
			_do("press")
		KEY_R:
			_do("reset")
		KEY_P:
			_do("patch")
		KEY_M:
			_do("repair")
		KEY_T:
			samples = []
			_say("charts cleared", false)
	queue_redraw()


# --------------------------------------------------------------- commands

## Every action the keys and the timelines share. Each sends the contract's own
## JSON; ids read from the snapshot arrive as floats and are cast to int, or
## serde would refuse `0.0` for an id.
func _do(action: String) -> void:
	# The scripted forms: a value in display units, converted at this boundary.
	if action.begins_with("feed_c_"):
		_set_feed_k(float(action.trim_prefix("feed_c_")) + 273.15)
		return
	if action.begins_with("duty_mw_"):
		_set_firing_w(float(action.trim_prefix("duty_mw_")) * 1.0e6)
		return
	if action.begins_with("manual:") or action.begins_with("auto:"):
		var parts := action.split(":")
		_set_mode(_loop_named(parts[1]), parts[0])
		return
	if action.begins_with("target_kgs:"):
		var parts := action.split(":")
		_set_target(_loop_named(parts[1]), float(parts[2]))
		return
	if action.begins_with("target_c:"):
		var parts := action.split(":")
		_set_target(_loop_named(parts[1]), float(parts[2]) + 273.15)
		return
	if action.begins_with("valve_pct_"):
		_set_feed_valve(float(action.trim_prefix("valve_pct_")) / 100.0)
		return
	match action:
		# Each key step snaps to its step, so a run of presses lands on round
		# values; the engine refuses what it must and the screen shows why.
		"feed_warmer", "feed_cooler":
			var step := TEMPERATURE_STEP_K if action == "feed_warmer" else -TEMPERATURE_STEP_K
			_set_feed_k(snappedf(_feed_k() - 273.15 + step, TEMPERATURE_STEP_K) + 273.15)
		"valve_up", "valve_down":
			var step := VALVE_STEP if action == "valve_up" else -VALVE_STEP
			_set_feed_valve(clampf(snappedf(_valve_opening() + step, VALVE_STEP), 0.0, 1.0))
		"vent_toggle", "vent_open", "vent_close":
			# Depressuring by hand (M59.5): the loop's vent fully open, or shut.
			# A trip holding the vent refuses it, in the engine's words.
			if vent_id < 0:
				_say("this plant has no hydrogen vent", true)
				return
			var open := action == "vent_open" or (action == "vent_toggle" and _opening(vent_id) < 0.5)
			_send(
				"%s %s" % [PLANTS[plant_key]["vent_valve"], "opened (depressuring)" if open else "closed"],
				{"cmd": "set_valve_opening", "node": int(vent_id), "opening": 1.0 if open else 0.0}
			)
		"firing_up", "firing_down":
			if preheater_id < 0:
				_say("this plant has no preheater", true)
				return
			var step := FIRING_STEP_W if action == "firing_up" else -FIRING_STEP_W
			_set_firing_w(maxf(0.0, snappedf(_duty_w(preheater_id) + step, FIRING_STEP_W)))
		"cooling_up", "cooling_down":
			if cooler_id < 0:
				_say("this plant has no product cooler", true)
				return
			var step := COOLING_STEP_W if action == "cooling_up" else -COOLING_STEP_W
			var duty := maxf(0.0, snappedf(_duty_w(cooler_id) + step, COOLING_STEP_W))
			_send(
				"%s removes %.0f MW" % [PLANTS[plant_key]["cooler"], duty / 1.0e6],
				{"cmd": "set_cooler_duty", "node": int(cooler_id), "duty": duty}
			)
		"toggle_mode":
			var loop = _selected()
			if loop != null:
				_set_mode(loop, "manual" if loop["mode"] == "auto" else "auto")
		"target_up", "target_down":
			var loop = _selected()
			if loop != null:
				_step_target(loop, 1.0 if action == "target_up" else -1.0)
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
		"patch":
			# Patch EVERY hole in every failed wall — the oil's on its outlet pipe
			# (M58, docs/DESIGN.md §63) and, on a bed whose hydrogen leaks too,
			# the hydrogen's on its pass's outlet (M59.4, §64.13): each pipe at
			# area zero. The fire goes out with the last of them, and the repair
			# is refused while any one still leaks.
			var patched := 0
			for i in bed_ids.size():
				if _wall_failed(i):
					for pipe in bed_hole_pipes[i]:
						_send(
							"patch %s" % _edge(pipe)["name"],
							{"cmd": "puncture_pipe", "edge": int(pipe), "area": 0.0}
						)
					patched += 1
			if patched == 0:
				_say("patch: no failed wall on this plant", true)
		"repair":
			# Repair every failed wall; the engine refuses one still leaking or
			# still at its limit, in its own words.
			var repaired := 0
			for i in bed_ids.size():
				if _wall_failed(i):
					_send(
						"repair %s's wall" % _node(bed_ids[i])["name"],
						{"cmd": "repair_bed_wall", "node": int(bed_ids[i])}
					)
					repaired += 1
			if repaired == 0:
				_say("repair: no failed wall on this plant", true)


func _set_feed_k(kelvin: float) -> void:
	_send(
		"%s to %.0f C" % [PLANTS[plant_key]["feed"], kelvin - 273.15],
		{"cmd": "set_source_temperature", "node": int(feed_id), "temperature": kelvin}
	)


func _set_feed_valve(opening: float) -> void:
	_send(
		"%s to %d%%" % [PLANTS[plant_key]["feed_valve"], roundi(opening * 100.0)],
		{"cmd": "set_valve_opening", "node": int(feed_valve_id), "opening": opening}
	)


func _set_firing_w(duty: float) -> void:
	_send(
		"fire %s at %.2f MW" % [PLANTS[plant_key]["preheater"], duty / 1.0e6],
		{"cmd": "set_furnace_duty", "node": int(preheater_id), "duty": duty}
	)


func _set_mode(loop, mode: String) -> void:
	if loop == null:
		_say("this plant has no such loop", true)
		return
	_send(
		"%s to %s" % [loop["name"], mode.to_upper()],
		{"cmd": "set_controller_mode", "loop_id": int(loop["id"]), "mode": mode}
	)


## A loop's target moved one key step, in the loop's own variable.
func _step_target(loop: Dictionary, sign: float) -> void:
	var setpoint: Dictionary = loop["setpoint"]
	match setpoint["variable"]:
		"temperature":
			var c := snappedf(float(setpoint["k"]) - 273.15 + sign * TEMPERATURE_STEP_K, TEMPERATURE_STEP_K)
			_set_target(loop, c + 273.15)
		"flow":
			_set_target(loop, snappedf(float(setpoint["kg_per_s"]) + sign * FLOW_STEP_KG_S, FLOW_STEP_KG_S))
		"level":
			_set_target(loop, snappedf(float(setpoint["m"]) + sign * LEVEL_STEP_M, LEVEL_STEP_M))
		_:
			_say("%s: no key step for a %s target" % [loop["name"], setpoint["variable"]], true)


## Send a loop's target, `value` in the loop's own SI unit (K, kg/s or m).
func _set_target(loop, value: float) -> void:
	if loop == null:
		_say("this plant has no such loop", true)
		return
	var variable: String = loop["setpoint"]["variable"]
	var tagged := {"variable": variable}
	var shown := ""
	match variable:
		"temperature":
			tagged["k"] = value
			shown = _c(value)
		"flow":
			tagged["kg_per_s"] = value
			shown = "%.2f kg/s" % value
		"level":
			tagged["m"] = value
			shown = "%.2f m" % value
		_:
			_say("%s: no target form for a %s loop" % [loop["name"], variable], true)
			return
	_send(
		"%s target to %s" % [loop["name"], shown],
		{"cmd": "set_setpoint", "loop_id": int(loop["id"]), "value": tagged}
	)


## Send one command and put the engine's answer on screen — its own refusal
## message when it says no.
func _send(what: String, command: Dictionary) -> void:
	var text := JSON.stringify(command)
	var err = JSON.parse_string(sim.apply_command(text))
	if err == null:
		_say(what, false)
		if auto_run:
			print("reactor: t=%d  %s  %s" % [sim.tick_index(), what, text])
	else:
		_say("%s: REFUSED — %s" % [what, err["message"]], true)
		if auto_run:
			print("reactor: t=%d  %s  REFUSED: %s" % [sim.tick_index(), what, err["message"]])


func _say(text: String, bad: bool) -> void:
	message = text
	message_bad = bad


# ---------------------------------------------------------------- reading

## Field lookups only. `trips` and `controls` are left out of the snapshot when
## a plant has none, hence the defaults.

func _node(id: int) -> Dictionary:
	return snapshot["nodes"][id]


func _edge(id: int) -> Dictionary:
	return snapshot["edges"][id]


func _solved() -> bool:
	return sim.tick_index() > 0


func _bed_k(i: int) -> float:
	return float(_node(bed_ids[i])["kind"]["bed"]["temperature"])


## The oil entering bed `i`: the stream on its inlet edge. Unknown before the
## first solve, when the edges carry no solved stream.
func _inlet_k(i: int) -> float:
	return float(_edge(bed_in_edges[i])["stream"]["temperature"]) if _solved() else NAN


## The heat bed `i`'s reaction released over the last tick [W]; NAN (unknown)
## before the first tick, when the snapshot leaves it out.
func _reaction_w(i: int) -> float:
	var value = _node(bed_ids[i]).get("reaction_heat_w")
	return NAN if value == null else float(value)


## Bed `i`'s wall: failed or intact, its limit [K], and the fire its leak feeds
## [W] (all of it; `fire_share` of it heats the bed). The fire is NAN before the
## first tick, when the snapshot leaves it out (M58, docs/DESIGN.md §63).
func _wall_failed(i: int) -> bool:
	return _node(bed_ids[i])["kind"]["wall"]["state"]["status"] == "failed"


func _wall_limit_k(i: int) -> float:
	return float(_node(bed_ids[i])["kind"]["wall"]["failure_temperature"])


func _wall_fire_w(i: int) -> float:
	var value = _node(bed_ids[i]).get("wall_fire_w")
	return NAN if value == null else float(value)


func _wall_share(i: int) -> float:
	return float(_node(bed_ids[i])["kind"]["wall"]["fire_share"])


func _residence_s(i: int) -> float:
	var value = _node(bed_ids[i]).get("residence_time_s")
	return NAN if value == null else float(value)


## The share of bed `i`'s outflow still uncracked feed: the `vgo` mass fraction
## on its outlet edge.
func _uncracked(i: int) -> float:
	if not _solved():
		return NAN
	return float(_edge(bed_out_edges[i])["stream"]["composition"]["mass_fractions"][vgo_index])


func _feed_k() -> float:
	return float(_node(feed_id)["kind"]["temperature"])


func _feed_flow() -> float:
	return float(_edge(bed_in_edges[0])["stream"]["mass_flow"]) if _solved() else NAN


func _valve_opening() -> float:
	return _opening(feed_valve_id)


func _opening(valve: int) -> float:
	return float(_node(valve)["kind"]["opening"])


## A node's solved pressure [Pa]; NAN before the first solve. Read from the
## NODE: a pipe's stream carries no solved pressure.
func _pressure_pa(id: int) -> float:
	var value = _node(id).get("pressure_pa")
	return NAN if value == null else float(value)


## The mass flow along an edge [kg/s], signed as the edge runs; NAN before the
## first solve.
func _flow(edge: int) -> float:
	return float(_edge(edge)["stream"]["mass_flow"]) if _solved() else NAN


## What leaks out of a pipe's hole [kg/s], reported on the pipe itself.
func _leak(pipe: int) -> float:
	var value = _edge(pipe).get("leak_mass_flow")
	return 0.0 if value == null else float(value)


func _bar(pa: float) -> String:
	return "--" if is_nan(pa) else "%.2f bar" % (pa / 1.0e5)


func _duty_w(id: int) -> float:
	return float(_node(id)["kind"]["duty"])


## A furnace's outlet temperature, solved; unknown before the first tick.
func _node_k(id: int) -> float:
	var value = _node(id).get("temperature_k")
	return NAN if value == null else float(value)


func _quench_flow() -> float:
	return float(_edge(quench_pipe_id)["stream"]["mass_flow"]) if _solved() else NAN


func _trips() -> Array:
	return snapshot.get("trips", [])


func _loops() -> Array:
	return snapshot.get("controls", [])


func _selected() -> Variant:
	var loops := _loops()
	if loops.is_empty():
		_say("this plant has no control loop", true)
		return null
	return loops[selected_loop % loops.size()]


func _loop_named(name: String) -> Variant:
	for loop in _loops():
		if loop["name"] == name:
			return loop
	return null


## A quench loop's check (§62.11): the quench arriving colder than the stream it
## joins. Both temperatures are the snapshot's; the comparison is the reading
## its doc names.
func _quench_cold(loop: Dictionary) -> bool:
	var quench: Dictionary = loop["quench"]
	return float(quench["quench_k"]) < float(quench["joins_k"])


func _trip_label(trip: Dictionary) -> String:
	var state: Dictionary = trip["state"]
	if state["status"] == "armed":
		return "armed"
	var label := "TRIPPED at t=%d" % int(state["at_tick"])
	if state.get("by_hand", false):
		label += " (by hand)"
	return label


## The trips and loops that watch bed `i`'s temperature, for its markers.
func _bed_trips(i: int) -> Array:
	var found: Array = []
	for trip in _trips():
		var watches: Dictionary = trip["watches"]
		if watches.has("node") and int(watches["node"]) == bed_ids[i] and trip["limit"]["variable"] == "temperature":
			found.append(trip)
	return found


func _bed_loops(i: int) -> Array:
	var found: Array = []
	for loop in _loops():
		var watches: Dictionary = loop["watches"]
		if watches.has("node") and int(watches["node"]) == bed_ids[i] and loop["setpoint"]["variable"] == "temperature":
			found.append(loop)
	return found


func _c(kelvin: float) -> String:
	return "--" if is_nan(kelvin) else "%.1f C" % (kelvin - 273.15)


func _mw(watts: float) -> String:
	return "--" if is_nan(watts) else "%.2f MW" % (watts / 1.0e6)


func _pct(fraction: float) -> String:
	return "--" if is_nan(fraction) else "%.1f%%" % (fraction * 100.0)


## A tagged value (a limit, target or reading) in its display unit.
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


## The level fraction of a tank — plant.gd's rule, from the slate.
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
	var line := "t=%6d (%5.1f min)  feed=%6.1f C  valve=%3d%%" % [tick, tick / 600.0, _feed_k() - 273.15, roundi(_valve_opening() * 100.0)]
	if preheater_id >= 0:
		line += "  fired=%5.2f MW  preheated=%6.1f C" % [_duty_w(preheater_id) / 1.0e6, _node_k(preheater_id) - 273.15]
	for i in bed_ids.size():
		line += (
			"  | %s in=%6.1f C bed=%7.2f C heat=%6.2f MW uncracked=%5.1f%%"
			% [
				_node(bed_ids[i])["name"],
				_inlet_k(i) - 273.15,
				_bed_k(i) - 273.15,
				_reaction_w(i) / 1.0e6,
				_uncracked(i) * 100.0,
			]
		)
		line += " wall=%s fire=%6.2f MW" % ["FAILED" if _wall_failed(i) else "intact", _wall_fire_w(i) / 1.0e6]
	if quench_pipe_id >= 0:
		line += "  | quench=%5.2f kg/s" % _quench_flow()
	if separator_id >= 0:
		line += (
			"  | loop=%7.2f bar makeup=%3d%% vent=%3d%%"
			% [_pressure_pa(separator_id) / 1.0e5, roundi(_opening(makeup_id) * 100.0), roundi(_opening(vent_id) * 100.0)]
		)
		for i in bed_ids.size():
			line += "  %s=%7.2f bar %5.2f kg/s" % [_node(pass_ids[i])["name"], _pressure_pa(pass_ids[i]) / 1.0e5, _flow(pass_in_edges[i])]
	for i in bed_ids.size():
		if _wall_failed(i):
			line += "  leaks:"
			for pipe in bed_hole_pipes[i]:
				line += " %s=%.3f kg/s" % [_edge(pipe)["name"], _leak(pipe)]
	for loop in _loops():
		line += "  %s=%s" % [loop["name"], loop["mode"]]
	for trip in _trips():
		line += "  %s=%s" % [trip["name"], _trip_label(trip)]
	return line


# ---------------------------------------------------------------- drawing

const LINE_Y := 200.0
const LINE_X0 := 60.0
const LINE_X1 := 900.0
const VESSEL := Vector2(64, 180)
## The vessels' fill scale [K]: a calm bed sits low, a runaway one near the top.
const VESSEL_LOW_K := 473.15
const VESSEL_HIGH_K := 1073.15
const CHART_RECT := Rect2(60, 420, 520, 250)
const TRAIL_RECT := Rect2(660, 420, 280, 250)
## On a plant with a hydrogen loop the loop is drawn where the charts' top was,
## and the charts are shorter.
const CHART_RECT_GAS := Rect2(60, 545, 520, 135)
const TRAIL_RECT_GAS := Rect2(660, 545, 280, 135)
## The hydrogen loop's rows: the gas line, the quench under it, the return.
const GAS_Y := 446.0
const GAS_QUENCH_Y := GAS_Y + 24.0
const GAS_RETURN_Y := GAS_Y + 64.0
const GAS_SEPARATOR_X := 150.0
const GAS_COMPRESSOR_X := 240.0
const GAS_HEATER_X := 355.0
const PANEL_X := 990.0

const BACKGROUND := Color(0.09, 0.10, 0.12)
const SHELL := Color(0.42, 0.45, 0.50)
const PIPE := Color(0.55, 0.58, 0.62)
const LIQUID := Color(0.20, 0.45, 0.75)
const HOT := Color(0.90, 0.30, 0.15)
const WHITE_HOT := Color(1.0, 0.92, 0.65)
const FLAME := Color(1.0, 0.55, 0.10)
const INK := Color(0.88, 0.90, 0.93)
const DIM := Color(0.60, 0.63, 0.68)
const GRID := Color(0.20, 0.22, 0.25)
const GOOD := Color(0.35, 0.80, 0.45)
const BAD := Color(0.95, 0.30, 0.25)
const TRIP_MARK := Color(0.95, 0.78, 0.30)
const TARGET_MARK := Color(0.45, 0.85, 0.95)
## One colour per bed, on the vessels' labels and both charts.
const BED_COLORS := [Color(1.0, 0.62, 0.25), Color(0.62, 0.55, 1.0)]
const HYDROGEN := Color(0.55, 0.85, 0.72)


func _draw() -> void:
	draw_rect(Rect2(Vector2.ZERO, get_viewport_rect().size), BACKGROUND)
	if halted != "" and snapshot.is_empty():
		_text(Vector2(30, 40), "HALTED — %s" % halted, BAD)
		return
	if snapshot.is_empty():
		return

	_text(Vector2(30, 34), PLANTS[plant_key]["title"], INK, 20)
	var tick := sim.tick_index()
	_text(
		Vector2(30, 60),
		(
			"t = %d min %02d s    %d tick(s) of 0.1 s per frame%s"
			% [tick / 600, (tick / 10) % 60, ticks_per_frame, "    PAUSED" if paused else ""]
		),
		DIM
	)

	_draw_line()
	if separator_id >= 0:
		_draw_gas_loop()
	_draw_chart()
	_draw_trail()
	_draw_panel()

	draw_multiline_string(
		ThemeDB.fallback_font, Vector2(30, 712), message, HORIZONTAL_ALIGNMENT_LEFT,
		get_viewport_rect().size.x - 60, 15, 2, BAD if message_bad else GOOD
	)
	_text(
		Vector2(30, 768),
		"W/S feed temperature   Up/Down feed valve   F/V preheater firing   C/X product cooler   D hydrogen vent   Tab pick loop   A auto/manual   Q/Z loop target",
		DIM, 14
	)
	_text(
		Vector2(30, 790),
		"E emergency stop   R reset trips   P patch holes   M repair wall   T clear charts   Space pause   [ ] speed   1 calm  2 trip  3 quench  4 wall  5 hydrogen  6 h2 quench",
		DIM, 14
	)
	if halted != "":
		_text(Vector2(30, 90), "HALTED — %s" % halted, BAD)


## The process line, left to right, one station per piece of equipment the plant
## names: feed, valve, preheater, the beds (a mix between two), the cooler and
## drum, the product. The quench returns underneath, from the drum to the mix.
func _draw_line() -> void:
	var stations: Array = [["feed", feed_id], ["valve", feed_valve_id]]
	if preheater_id >= 0:
		stations.append(["preheater", preheater_id])
	for i in bed_ids.size():
		if i > 0 and mix_id >= 0:
			stations.append(["mix", mix_id])
		stations.append(["bed", i])
	if cooler_id >= 0:
		stations.append(["cooler", cooler_id])
	if drum_id >= 0:
		stations.append(["drum", drum_id])
	stations.append(["product", product_id])

	var step := (LINE_X1 - LINE_X0) / (stations.size() - 1)
	draw_line(Vector2(LINE_X0, LINE_Y), Vector2(LINE_X1, LINE_Y), PIPE, 5.0)
	bed_xs = []
	bed_xs.resize(bed_ids.size())
	var mix_x := -1.0
	var drum_x := -1.0
	for s in stations.size():
		var x := LINE_X0 + step * s
		match stations[s][0]:
			"feed":
				draw_circle(Vector2(x, LINE_Y), 16.0, _temperature_color(_feed_k()))
				_label_under(x, LINE_Y + 30, ["feed", _c(_feed_k()), "%.1f kg/s" % _feed_flow() if _solved() else "-- kg/s"])
			"valve":
				_draw_valve(Vector2(x, LINE_Y))
				_label_under(x, LINE_Y + 30, ["valve", "%d%% open" % roundi(_valve_opening() * 100.0)])
			"preheater":
				_draw_preheater(x)
			"mix":
				mix_x = x
				draw_circle(Vector2(x, LINE_Y), 7.0, PIPE)
			"bed":
				bed_xs[int(stations[s][1])] = x
				_draw_bed(x, int(stations[s][1]))
			"cooler":
				var box := Rect2(x - 22, LINE_Y - 22, 44, 44)
				draw_rect(box, Color(0.13, 0.20, 0.28))
				draw_rect(box, LIQUID, false, 2.0)
				_label_under(x, LINE_Y + 40, ["cooler", "-%.0f MW" % (_duty_w(cooler_id) / 1.0e6)])
			"drum":
				drum_x = x
				_draw_drum(x)
			"product":
				draw_polygon(
					PackedVector2Array([Vector2(x - 12, LINE_Y - 14), Vector2(x + 12, LINE_Y), Vector2(x - 12, LINE_Y + 14)]),
					PackedColorArray([PIPE])
				)
				_label_under(x, LINE_Y + 30, ["product"])
	if mix_x >= 0.0 and drum_x >= 0.0:
		_draw_quench(mix_x, drum_x)


func _draw_valve(at: Vector2) -> void:
	# A bow tie, as two triangles meeting at the seat (one crossed polygon
	# does not triangulate).
	draw_colored_polygon(PackedVector2Array([at + Vector2(-14, -12), at, at + Vector2(-14, 12)]), SHELL)
	draw_colored_polygon(PackedVector2Array([at + Vector2(14, -12), at, at + Vector2(14, 12)]), SHELL)


func _draw_preheater(x: float) -> void:
	var box := Rect2(x - 34, LINE_Y - 70, 68, 110)
	draw_rect(box, Color(0.14, 0.12, 0.11))
	draw_rect(box, SHELL, false, 3.0)
	var duty := _duty_w(preheater_id)
	# Flames sized by the firing the snapshot reports, against the heater's
	# own loop range where it has one; a dark firebox when it is out.
	if duty > 0.0:
		var height := clampf(duty / 3.0e7, 0.08, 1.0) * 60.0
		for k in 4:
			var fx := box.position.x + 8 + k * 15
			draw_colored_polygon(
				PackedVector2Array([Vector2(fx, box.end.y - 6), Vector2(fx + 6, box.end.y - 6 - height), Vector2(fx + 12, box.end.y - 6)]),
				FLAME
			)
	_label_under(x, LINE_Y + 60, ["preheater", "fired %.2f MW" % (duty / 1.0e6), "out %s" % _c(_node_k(preheater_id))])


## A bed: a vessel filled to its temperature on a fixed scale and coloured by
## it, with its trips' limits and its loops' targets across it, and its books
## underneath.
func _draw_bed(x: float, i: int) -> void:
	var rect := Rect2(Vector2(x - VESSEL.x * 0.5, LINE_Y - VESSEL.y * 0.5), VESSEL)
	draw_rect(rect, Color(0.13, 0.14, 0.16))
	var kelvin := _bed_k(i)
	var fraction := clampf((kelvin - VESSEL_LOW_K) / (VESSEL_HIGH_K - VESSEL_LOW_K), 0.0, 1.0)
	draw_rect(
		Rect2(rect.position + Vector2(0, rect.size.y * (1.0 - fraction)), Vector2(rect.size.x, rect.size.y * fraction)),
		_temperature_color(kelvin)
	)
	draw_rect(rect, SHELL, false, 3.0)
	for loop in _bed_loops(i):
		var target := float(loop["setpoint"]["k"])
		var y := _vessel_y(rect, target)
		draw_line(Vector2(rect.position.x - 6, y), Vector2(rect.end.x, y), TARGET_MARK, 2.0)
		_text(Vector2(rect.position.x - 50, y + 4), "%.0f" % (target - 273.15), TARGET_MARK, 12)
	# The wall's limit, and on a failed wall the crack, the spray out of the
	# outlet line and its fire, sized by the power the snapshot reports (M58).
	var wall_y := _vessel_y(rect, _wall_limit_k(i))
	var failed := _wall_failed(i)
	draw_dashed_line(Vector2(rect.position.x, wall_y), Vector2(rect.end.x, wall_y), BAD if failed else DIM, 2.0, 6.0)
	_text(Vector2(rect.position.x - 58, wall_y + 4), "wall %.0f" % (_wall_limit_k(i) - 273.15), BAD if failed else DIM, 12)
	if failed:
		draw_polyline(
			PackedVector2Array([
				rect.position + Vector2(rect.size.x * 0.55, 0),
				rect.position + Vector2(rect.size.x * 0.35, rect.size.y * 0.3),
				rect.position + Vector2(rect.size.x * 0.6, rect.size.y * 0.5),
				rect.position + Vector2(rect.size.x * 0.4, rect.size.y * 0.75),
			]),
			BAD, 3.0
		)
		var fire := _wall_fire_w(i)
		var spray_at := Vector2(rect.end.x + 30, LINE_Y)
		if fire > 0.0:
			var height := clampf(fire / 6.0e7, 0.15, 1.0) * 90.0
			for k in 3:
				var fx := spray_at.x - 12 + k * 12
				draw_colored_polygon(
					PackedVector2Array([
						Vector2(fx - 7, LINE_Y - 4),
						Vector2(fx, LINE_Y - 4 - height * (0.7 + 0.15 * k)),
						Vector2(fx + 7, LINE_Y - 4),
					]),
					FLAME
				)
			_text(spray_at + Vector2(24, -46), "fire %s" % _mw(fire), FLAME, 13)
			_text(spray_at + Vector2(24, -30), "%s into bed" % _mw(fire * _wall_share(i)), FLAME, 12)
		_text(Vector2(rect.position.x - 6, rect.position.y - 26), "WALL FAILED", BAD, 15)
	for trip in _bed_trips(i):
		var limit := float(trip["limit"]["k"])
		var y := _vessel_y(rect, limit)
		var tripped: bool = trip["state"]["status"] == "tripped"
		draw_line(Vector2(rect.position.x, y), Vector2(rect.end.x + 6, y), BAD if tripped else TRIP_MARK, 2.0)
		_text(Vector2(rect.end.x + 8, y + 4), "trip %.0f" % (limit - 273.15), BAD if tripped else TRIP_MARK, 12)
	var name: String = _node(bed_ids[i])["name"]
	_text(Vector2(rect.position.x, rect.position.y - 8), name, BED_COLORS[i % BED_COLORS.size()], 15)
	_label_under(
		x,
		rect.end.y + 20,
		[
			_c(kelvin),
			"in %s" % _c(_inlet_k(i)),
			"heat %s" % _mw(_reaction_w(i)),
			"uncracked %s" % _pct(_uncracked(i)),
			"%s in bed" % ("--" if is_nan(_residence_s(i)) else "%.0f s" % _residence_s(i)),
		]
	)


func _vessel_y(rect: Rect2, kelvin: float) -> float:
	return rect.end.y - rect.size.y * clampf((kelvin - VESSEL_LOW_K) / (VESSEL_HIGH_K - VESSEL_LOW_K), 0.0, 1.0)


func _draw_drum(x: float) -> void:
	var rect := Rect2(x - 30, LINE_Y - 50, 60, 100)
	var fraction := _tank_fraction(drum_id)
	var kelvin := float(_node(drum_id)["kind"]["temperature"])
	draw_rect(
		Rect2(rect.position + Vector2(0, rect.size.y * (1.0 - fraction)), Vector2(rect.size.x, rect.size.y * fraction)),
		_temperature_color(kelvin)
	)
	draw_rect(rect, SHELL, false, 3.0)
	_text(Vector2(rect.position.x - 4, rect.position.y - 8), "drum", DIM, 14)
	_label_under(x, rect.end.y + 18, [_c(kelvin)])


## The quench, from the drum back along the bottom to the mix, labelled with the
## flow it carries and the two temperatures the loop's check compares.
func _draw_quench(mix_x: float, drum_x: float) -> void:
	# Under the beds' readings, above the charts.
	var low := LINE_Y + 178.0
	var color := LIQUID
	var loop = _quench_loop()
	var cold := true
	if loop != null:
		cold = _quench_cold(loop)
		color = LIQUID if cold else BAD
	draw_polyline(
		PackedVector2Array([Vector2(drum_x, LINE_Y + 50), Vector2(drum_x, low), Vector2(mix_x, low), Vector2(mix_x, LINE_Y)]),
		color, 3.0
	)
	# Labelled beside the drum's leg, clear of the beds' readings.
	var at := Vector2(drum_x + 10, LINE_Y + 110)
	_text(at, "quench %s" % ("%.2f kg/s" % _quench_flow() if _solved() else "-- kg/s"), color, 14)
	if loop != null:
		var quench: Dictionary = loop["quench"]
		_text(at + Vector2(0, 18), "at %s" % _c(float(quench["quench_k"])), color, 13)
		_text(at + Vector2(0, 35), "joins %s%s" % [_c(float(quench["joins_k"])), "" if cold else "  NOT COLD"], color, 13)


## The hydrogen loop (M59.5), under the oil line: the separator and its
## pressure, the compressor, the gas heater, each bed's pass under its bed, the
## gas cooler and the return; the makeup and the vent on the separator; a
## hydrogen quench from the compressor's discharge to the mix between the
## passes, where the plant has one; and a failed wall's hydrogen leak at its
## pass. Every number is a snapshot field.
func _draw_gas_loop() -> void:
	var cooler_x := float(bed_xs[bed_xs.size() - 1]) + 125.0
	var line := PackedVector2Array([
		Vector2(GAS_SEPARATOR_X, GAS_Y), Vector2(cooler_x, GAS_Y), Vector2(cooler_x, GAS_RETURN_Y),
		Vector2(GAS_SEPARATOR_X, GAS_RETURN_Y), Vector2(GAS_SEPARATOR_X, GAS_Y),
	])
	draw_polyline(line, HYDROGEN.darkened(0.3), 3.0)

	# The separator: the loop's pressure, which the makeup holds and the vent
	# and every leak lower.
	var sep := Rect2(GAS_SEPARATOR_X - 18, GAS_Y - 30, 36, 60)
	draw_rect(sep, Color(0.12, 0.17, 0.16))
	draw_rect(sep, SHELL, false, 3.0)
	_label_over(GAS_SEPARATOR_X, GAS_Y - 54, ["separator", _bar(_pressure_pa(separator_id))], HYDROGEN)
	# Makeup in and vent out, on stubs to the left.
	var vent_open := _opening(vent_id)
	draw_line(Vector2(40, GAS_Y - 14), Vector2(sep.position.x, GAS_Y - 14), HYDROGEN.darkened(0.3), 2.0)
	draw_line(Vector2(40, GAS_Y + 14), Vector2(sep.position.x, GAS_Y + 14), HYDROGEN.darkened(0.3), 2.0)
	_draw_small_valve(Vector2(85, GAS_Y - 14), BAD if vent_open > 0.0 else SHELL)
	_draw_small_valve(Vector2(85, GAS_Y + 14), SHELL)
	_text(Vector2(26, GAS_Y - 26), "vent %d%%" % roundi(vent_open * 100.0), BAD if vent_open > 0.0 else DIM, 13)
	_text(Vector2(26, GAS_Y + 36), "makeup %d%%" % roundi(_opening(makeup_id) * 100.0), DIM, 13)

	# The compressor, and the pressure it delivers.
	var on: bool = _node(compressor_id)["kind"].get("on", true)
	draw_circle(Vector2(GAS_COMPRESSOR_X, GAS_Y), 15.0, Color(0.14, 0.16, 0.18))
	draw_arc(Vector2(GAS_COMPRESSOR_X, GAS_Y), 15.0, 0.0, TAU, 24, SHELL, 3.0)
	draw_colored_polygon(
		PackedVector2Array([
			Vector2(GAS_COMPRESSOR_X - 7, GAS_Y - 9), Vector2(GAS_COMPRESSOR_X + 9, GAS_Y), Vector2(GAS_COMPRESSOR_X - 7, GAS_Y + 9)
		]),
		HYDROGEN if on else DIM
	)
	var delivers := NAN
	if compressor_out_edge >= 0:
		delivers = _pressure_pa(int(_edge(compressor_out_edge)["to"]))
	_label_over(
		GAS_COMPRESSOR_X, GAS_Y - 54,
		["compressor" if on else "compressor OFF", "%.2f kg/s" % _flow(compressor_out_edge) if _solved() else "-- kg/s", "to %s" % _bar(delivers)],
		HYDROGEN
	)

	# The gas heater.
	var heater := Rect2(GAS_HEATER_X - 22, GAS_Y - 14, 44, 28)
	draw_rect(heater, Color(0.14, 0.12, 0.11))
	draw_rect(heater, SHELL, false, 2.0)
	if _duty_w(gas_heater_id) > 0.0:
		draw_rect(Rect2(heater.position + Vector2(4, 18), Vector2(36, 6)), FLAME)
	_label_over(GAS_HEATER_X, GAS_Y - 54, ["gas heater", "fired %s" % _mw(_duty_w(gas_heater_id)), "out %s" % _c(_node_k(gas_heater_id))], HYDROGEN)

	# Each pass under its bed, coloured by its temperature (the bed's own).
	for i in bed_ids.size():
		var x := float(bed_xs[i])
		var box := Rect2(x - 28, GAS_Y - 11, 56, 22)
		draw_rect(box, _temperature_color(_node_k(pass_ids[i])))
		draw_rect(box, SHELL, false, 2.0)
		_label_over(
			x, GAS_Y - 54,
			[
				"%s %s" % [_node(pass_ids[i])["name"], _bar(_pressure_pa(pass_ids[i]))],
				"hydrogen %s" % ("%.2f kg/s" % _flow(pass_in_edges[i]) if _solved() else "--"),
			],
			BED_COLORS[i % BED_COLORS.size()]
		)
		# A failed wall's hydrogen hole, on the pass's outlet: what leaks.
		if _wall_failed(i) and bed_hole_pipes[i].size() > 1:
			var leak := _leak(int(bed_hole_pipes[i][1]))
			var at := Vector2(x + 44, GAS_Y)
			if leak > 0.0:
				var height := clampf(leak / 0.3, 0.15, 1.0) * 34.0
				for k in 2:
					var fx := at.x - 5 + k * 10
					draw_colored_polygon(
						PackedVector2Array([Vector2(fx - 5, GAS_Y - 3), Vector2(fx, GAS_Y - 3 - height * (0.8 + 0.2 * k)), Vector2(fx + 5, GAS_Y - 3)]),
						FLAME
					)
			_text(Vector2(at.x - 6, GAS_Y + 32), "H2 leak %.3f kg/s" % leak, FLAME if leak > 0.0 else DIM, 12)

	# The gas cooler, on the way back to the separator.
	var cooler := Rect2(cooler_x - 18, GAS_Y - 14, 36, 28)
	draw_rect(cooler, Color(0.13, 0.20, 0.28))
	draw_rect(cooler, LIQUID, false, 2.0)
	_label_over(cooler_x, GAS_Y - 54, ["gas cooler", "out %s" % _c(_node_k(gas_cooler_id))], HYDROGEN)

	# The hydrogen quench: from the compressor's discharge to the mix ahead of
	# the second pass, coloured by the bed loop's cold check.
	if gas_mix_id >= 0 and quench_pipe_id >= 0 and bed_xs.size() > 1:
		var tee_x := GAS_COMPRESSOR_X + 50.0
		var mix_x := (float(bed_xs[0]) + float(bed_xs[1])) * 0.5
		var color := HYDROGEN
		var loop = _quench_loop()
		var cold := true
		if loop != null:
			cold = _quench_cold(loop)
			color = LIQUID if cold else BAD
		draw_polyline(
			PackedVector2Array([Vector2(tee_x, GAS_Y), Vector2(tee_x, GAS_QUENCH_Y), Vector2(mix_x, GAS_QUENCH_Y), Vector2(mix_x, GAS_Y)]),
			color, 2.5
		)
		draw_circle(Vector2(tee_x, GAS_Y), 4.0, PIPE)
		draw_circle(Vector2(mix_x, GAS_Y), 6.0, PIPE)
		var text := "quench %s" % ("%.2f kg/s" % _quench_flow() if _solved() else "-- kg/s")
		if loop != null:
			var quench: Dictionary = loop["quench"]
			text += "  at %s, joins %s%s" % [_c(float(quench["quench_k"])), _c(float(quench["joins_k"])), "" if cold else "  NOT COLD"]
		_text(Vector2(tee_x + 34, GAS_QUENCH_Y + 16), text, color, 12)


func _draw_small_valve(at: Vector2, color: Color) -> void:
	draw_colored_polygon(PackedVector2Array([at + Vector2(-9, -7), at, at + Vector2(-9, 7)]), color)
	draw_colored_polygon(PackedVector2Array([at + Vector2(9, -7), at, at + Vector2(9, 7)]), color)


## Lines of text centred on `x`, the first one's baseline at `y`: labels drawn
## above a station, the first line in `color`.
func _label_over(x: float, y: float, lines: Array, color: Color) -> void:
	for n in lines.size():
		var text := str(lines[n])
		var width := ThemeDB.fallback_font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, 13).x
		_text(Vector2(x - width * 0.5, y + n * 16), text, color if n == 0 else DIM, 13)


## The bed loop that carries a quench reading, if the plant has one.
func _quench_loop() -> Variant:
	for loop in _loops():
		if loop.has("quench"):
			return loop
	return null


## The time chart: each bed's temperature (bold) and the oil entering it (thin)
## over the last CHART_SAMPLES samples, with the bed's trip limits and loop
## targets across it. The vertical scale fits what is drawn.
func _draw_chart() -> void:
	var rect := CHART_RECT if separator_id < 0 else CHART_RECT_GAS
	draw_rect(rect, Color(0.11, 0.12, 0.14))
	_text(rect.position + Vector2(0, -10), "temperature, last 30 minutes (bold: bed, thin: oil entering it)", DIM, 14)
	var shown := samples.slice(maxi(0, samples.size() - CHART_SAMPLES))
	var marks: Array = []
	for i in bed_ids.size():
		for trip in _bed_trips(i):
			marks.append([float(trip["limit"]["k"]), TRIP_MARK, "trip"])
		for loop in _bed_loops(i):
			marks.append([float(loop["setpoint"]["k"]), TARGET_MARK, "target"])
	var low := INF
	var high := -INF
	for sample in shown:
		for pair in sample["beds"]:
			for k in pair:
				if not is_nan(k):
					low = minf(low, k)
					high = maxf(high, k)
	for mark in marks:
		low = minf(low, mark[0])
		high = maxf(high, mark[0])
	if shown.is_empty() or low > high:
		draw_rect(rect, SHELL, false, 1.0)
		_text(rect.position + Vector2(12, 28), "waiting for the first sample", DIM, 14)
		return
	var scale := _nice_range(low, high)
	_grid_y(rect, scale)
	for mark in marks:
		var y := _y_of(rect, scale, mark[0])
		draw_line(Vector2(rect.position.x, y), Vector2(rect.end.x, y), mark[1], 1.0)
		_text(Vector2(rect.end.x - 54, y - 4), "%s %.0f" % [mark[2], mark[0] - 273.15], mark[1], 12)
	var last_tick := int(samples[-1]["tick"])
	var span := float(CHART_SAMPLES * SAMPLE_TICKS)
	for i in bed_ids.size():
		var bed_line := PackedVector2Array()
		var inlet_line := PackedVector2Array()
		for sample in shown:
			var x := rect.end.x - rect.size.x * (last_tick - int(sample["tick"])) / span
			var pair: Array = sample["beds"][i]
			bed_line.append(Vector2(x, _y_of(rect, scale, pair[0])))
			if not is_nan(pair[1]):
				inlet_line.append(Vector2(x, _y_of(rect, scale, pair[1])))
		var color: Color = BED_COLORS[i % BED_COLORS.size()]
		if inlet_line.size() > 1:
			draw_polyline(inlet_line, color.darkened(0.35), 1.0)
		if bed_line.size() > 1:
			draw_polyline(bed_line, color, 2.5)
	draw_rect(rect, SHELL, false, 1.0)
	_text(Vector2(rect.position.x, rect.end.y + 16), "-30 min", DIM, 12)
	_text(Vector2(rect.end.x - 26, rect.end.y + 16), "now", DIM, 12)


## The trail: every sample as a dot at (oil entering the bed, bed), older dots
## fainter, the newest ringed. No curve is drawn that the engine did not report.
func _draw_trail() -> void:
	var rect := TRAIL_RECT if separator_id < 0 else TRAIL_RECT_GAS
	draw_rect(rect, Color(0.11, 0.12, 0.14))
	_text(rect.position + Vector2(0, -10), "trail: bed against oil entering it", DIM, 14)
	var x_low := INF
	var x_high := -INF
	var y_low := INF
	var y_high := -INF
	for sample in samples:
		for pair in sample["beds"]:
			if is_nan(pair[1]):
				continue
			y_low = minf(y_low, pair[0])
			y_high = maxf(y_high, pair[0])
			x_low = minf(x_low, pair[1])
			x_high = maxf(x_high, pair[1])
	if x_low > x_high:
		draw_rect(rect, SHELL, false, 1.0)
		return
	var xs := _nice_range(x_low, x_high)
	var ys := _nice_range(y_low, y_high)
	_grid_y(rect, ys)
	_text(Vector2(rect.position.x, rect.end.y + 16), "%.0f" % (xs[0] - 273.15), DIM, 12)
	_text(Vector2(rect.end.x - 26, rect.end.y + 16), "%.0f" % (xs[1] - 273.15), DIM, 12)
	_text(Vector2(rect.position.x + rect.size.x * 0.5 - 60, rect.end.y + 16), "oil entering, C", DIM, 12)
	var count := samples.size()
	for n in count:
		var age := float(count - 1 - n) / float(maxi(1, count - 1))
		for i in bed_ids.size():
			var pair: Array = samples[n]["beds"][i]
			if is_nan(pair[1]):
				continue
			var at := Vector2(
				rect.position.x + rect.size.x * (pair[1] - xs[0]) / (xs[1] - xs[0]),
				_y_of(rect, ys, pair[0])
			)
			var color: Color = BED_COLORS[i % BED_COLORS.size()]
			color.a = lerpf(1.0, 0.18, age)
			draw_circle(at, 2.5, color)
			if n == count - 1:
				draw_arc(at, 6.0, 0.0, TAU, 16, BED_COLORS[i % BED_COLORS.size()], 1.5)
	draw_rect(rect, SHELL, false, 1.0)


## A range [K] around `low`..`high`, widened to whole 25-degree steps, at least
## 50 degrees tall so a settled line is not drawn as noise.
func _nice_range(low: float, high: float) -> Array:
	var step := 25.0
	var a := floorf((low - 273.15) / step) * step
	var b := ceilf((high - 273.15) / step) * step
	if b - a < 50.0:
		var middle := (a + b) * 0.5
		a = floorf((middle - 25.0) / step) * step
		b = a + 50.0
	return [a + 273.15, b + 273.15]


func _y_of(rect: Rect2, scale: Array, kelvin: float) -> float:
	return rect.end.y - rect.size.y * clampf((kelvin - scale[0]) / (scale[1] - scale[0]), 0.0, 1.0)


## Horizontal grid lines with their °C, about five to a chart.
func _grid_y(rect: Rect2, scale: Array) -> void:
	var span: float = scale[1] - scale[0]
	var step := 25.0
	while span / step > 6.0:
		step *= 2.0
	var c := ceilf((scale[0] - 273.15) / step) * step
	while c + 273.15 <= scale[1] + 0.01:
		var y := _y_of(rect, scale, c + 273.15)
		draw_line(Vector2(rect.position.x, y), Vector2(rect.end.x, y), GRID, 1.0)
		_text(Vector2(rect.position.x - 40, y + 4), "%.0f" % c, DIM, 12)
		c += step


## The right-hand panel: the feed, the heater and quench, each loop's faceplate
## and each trip against its limit.
func _draw_panel() -> void:
	var y := 100.0
	_text(Vector2(PANEL_X, y), "LOOPS (Tab picks, A auto/manual, Q/Z target)", DIM, 14)
	y += 22
	var loops := _loops()
	if loops.is_empty():
		_text(Vector2(PANEL_X, y), "none on this plant", DIM, 14)
		y += 20
	for n in loops.size():
		var loop: Dictionary = loops[n]
		var picked := n == selected_loop % loops.size()
		var auto: bool = loop["mode"] == "auto"
		_text(
			Vector2(PANEL_X, y),
			"%s %s  %s" % [">" if picked else " ", loop["name"], "AUTO" if auto else "MANUAL"],
			INK if picked else DIM,
			15
		)
		y += 18
		var line := "    target %s  saw %s  out %.0f%%" % [
			_value_text(loop["setpoint"]), _value_text(loop.get("measurement")), float(loop["output"]) * 100.0
		]
		if loop.has("drives"):
			line += "  -> sets %s" % loops[int(loop["drives"])]["name"]
		_text(Vector2(PANEL_X, y), line, DIM, 13)
		y += 17
		if loop.has("quench"):
			var cold := _quench_cold(loop)
			var state := "quench is colder: more quench cools the bed" if cold else "quench is NOT colder: more would not cool it"
			if not cold and not auto:
				state += " - FROZEN, put back in AUTO by hand once it is cold"
			_text(Vector2(PANEL_X, y), "    " + state, GOOD if cold else BAD, 13)
			y += 17
		y += 4

	y += 10
	_text(Vector2(PANEL_X, y), "TRIPS (E press all armed, R reset)", DIM, 14)
	y += 22
	var trips := _trips()
	if trips.is_empty():
		_text(Vector2(PANEL_X, y), "none on this plant", DIM, 14)
		y += 20
	for trip in trips:
		var tripped: bool = trip["state"]["status"] == "tripped"
		var watches: Dictionary = trip["watches"]
		var watched := ""
		if watches.has("node"):
			watched = _node(int(watches["node"]))["name"]
		elif watches.has("pipe"):
			watched = _edge(int(watches["pipe"]))["name"]
		_text(Vector2(PANEL_X, y), "  %s  %s" % [trip["name"], _trip_label(trip)], BAD if tripped else INK, 15)
		y += 18
		_text(
			Vector2(PANEL_X, y),
			"    %s %s %s %s; %s" % [
				watched,
				trip["limit"]["variable"],
				">=" if trip["direction"] == "high" else "<=",
				_value_text(trip["limit"]),
				_reset_label(trip),
			],
			DIM, 13
		)
		y += 21


## Who resets a trip and what the reset does, from its `reset` field (M40), as
## furnace.gd words it; absent means the default.
func _reset_label(trip: Dictionary) -> String:
	var reset = trip.get("reset")
	if reset == null or reset["mode"] == "manual":
		return "reset by hand, restarts nothing"
	if reset["mode"] == "manual_restart":
		return "reset by hand, restarts unless barred"
	return "resets itself at %s, restarts unless barred" % _value_text(reset["reset_at"])


func _label_under(x: float, y: float, lines: Array) -> void:
	for n in lines.size():
		var text := str(lines[n])
		var width := ThemeDB.fallback_font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, 13).x
		_text(Vector2(x - width * 0.5, y + n * 16), text, INK if n == 0 else DIM, 13)


## A display colour for a temperature: blue when cool, red through orange as a
## bed heats, white-hot near the top of the vessels' scale.
func _temperature_color(kelvin: float) -> Color:
	if is_nan(kelvin):
		return SHELL
	var t := clampf((kelvin - VESSEL_LOW_K) / (VESSEL_HIGH_K - VESSEL_LOW_K), 0.0, 1.0)
	if t < 0.4:
		return LIQUID.lerp(HOT, t / 0.4)
	return HOT.lerp(WHITE_HOT, (t - 0.4) / 0.6)


func _text(at: Vector2, text: String, color: Color, size: int = 16) -> void:
	draw_string(ThemeDB.fallback_font, at, text, HORIZONTAL_ALIGNMENT_LEFT, -1, size, color)


func _halt(reason: String) -> void:
	halted = reason
	push_error("reactor: %s" % reason)
	print("reactor: HALTED — %s" % reason)
	queue_redraw()
