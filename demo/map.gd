# A map of ANY plant: every shipped scenario drawn from its own snapshot.
#
# The five screens beside this one are each drawn by hand for a few plants —
# 23 of the 71 that ship. This scene takes the other road: it knows no plant.
# It reads the nodes and pipes a snapshot lists, lays them out left to right
# in the order the liquid meets them, and draws each by its kind. So every
# plant in `scenarios/` can be watched running, and a new plant file appears
# here without this file changing.
#
# It is a first cut, on purpose (docs/plans/plant-map-screen.md has the rest):
# the layout is computed rather than drawn by a person, and a column or a
# reactor is one box — their insides are what the hand-drawn screens are for.
# The mouse INSPECTS (M78, slice 1 of the plan): over a node or a pipe, the
# panel at the right lists every number the snapshot gives it, walked
# generically, so a field the engine adds appears without this file changing.
#
# And it has HANDS (M79, slice 2): a click on a valve or a furnace selects it
# and Up/Down move it, a click on a pump starts or stops it, a click on a pipe
# with P held makes a hole in it or mends the one the map made. With the mouse
# over nothing the panel lists the plant's control loops and trips, and keys
# work them. Nothing is checked here first: every command goes to the engine
# and its answer — its own refusal, in its own words — is the message line. A
# tick the engine cannot solve puts the plant back to just before the last
# command it accepted and undoes that command (M76), as the pump screen does.
#
# A valve or a furnace a control loop holds in AUTO refuses Up/Down: the loop
# would overwrite the move on the next tick. M takes it by hand (M80): the
# snapshot names the node each loop moves (`actuates`), so M puts THAT loop in
# MANUAL, and M again gives the valve back to it in AUTO. The line under the
# headings says what the keys do to what is picked, and the panel over a node
# or a pipe says what a click on it does.
#
# And a person can fix its LAYOUT (M81, slice 3): Ctrl+drag moves a node,
# Ctrl+S keeps every node's place in `demo/layout/<plant>.json` (name → [x, y]
# on the page), and the next load draws each node that file names there —
# the computed place for the rest. A file naming a node the plant does not
# have, or a place off the page, is named on the message line, and `--auto`
# counts that plant as stopped, so a layout cannot go quietly stale.
#
# And it is HOME (M84, slice 4): the plant runs in demo/running.gd, not in
# this scene, so Enter opens the screen that draws the plant running up close
# — the column screen on a crude unit — on the same plant as it stands, and
# Enter there comes back here; neither starts it again. A plant no other
# screen draws says so on the message line. The menu's map row opens on the
# plant running too; Left/Right, as before, load the next plant afresh.
#
# And its pipes READ (M85): each runs at right angles, the way a plant's do,
# routed round the other nodes and their names and not along another pipe
# (demo/route.gd), at load and when a dragged node is let go. Where two must
# cross, the one on top — the forward line over a recycle — cuts a gap in the
# other with its dark border; where a pipe joins a node, a flange marks the
# spot; its arrow and number sit on its longest run. A column or a reactor is
# drawn as tall as what is piped to it, so each draw meets it level.
#
# Same rule as every screen: nothing physical is computed here. A pipe's
# width is its reported flow, its colour its reported temperature; a tank's
# fill is the level the plant.gd demo derives from snapshot fields only. The
# panel converts units for reading (Pa to bar, K to °C, W to MW) and nothing
# more; a field whose unit this file does not know is shown as it came.
#
# Interactive:  godot --path . res://demo/map.tscn      (or the menu's last row)
#               Left/Right plant   Space pause   [ ] speed   Esc menu   mouse: inspect (wheel scrolls)
#               click a valve/furnace: Up/Down, M take it from its loop or give it back
#               click a pump: on/off   P+click a pipe: hole/mend
#               Tab loop   A auto/manual   Q/Z target   R reset tripped trips
#               Ctrl+drag a node: move it   Ctrl+S keep the layout (demo/layout/)
#               Enter: the screen that draws this plant up close (M84)
# Recorded:     godot --headless --path . res://demo/map.tscn -- --auto
#               ...loads every plant in turn, runs it 200 ticks, prints one
#               line each and a count: a smoke test of the map on all of them
#               (and builds the panel of every node and pipe, so a field shape
#               the panel cannot read shows up as a script error).
# Screenshot:   add --plant=<file stem> --shot=<file.png> to a windowed run;
#               --hover=<node or pipe name> points at it as the mouse would,
#               --scroll=<lines> scrolls its panel as the wheel would,
#               --pick=<valve or furnace name> clicks it as the mouse would.
# Hands:        godot --headless --path . res://demo/map.tscn -- --hands --plant=<stem>
#               ...runs 20 ticks (--hands-at=<tick>: that many), then works the first valve, pump and furnace,
#               a pipe or two, the loops and trips, and the first valve or furnace a
#               loop holds (Up, M, Up, M) through the same calls
#               the mouse and keys make, printing every answer; exits nonzero
#               if the plant stopped. Add --break-at=<tick> to set the first
#               supply to -270 °C at that tick (the pump screen's own break:
#               on pump_gas_lock the next tick cannot be solved) and see the
#               map recover. With --shot=<file.png> on a windowed run, the
#               page is saved at tick 200 after the tour.
# Layout:       --drag=<node>@<x>,<y> (repeatable) moves a node as Ctrl+drag
#               would, --save-layout then saves as Ctrl+S would, and
#               --layout-dir=<dir> reads and writes the layout files there
#               instead of demo/layout (a test that leaves the shipped ones be).
# Hand-over:    --plant=<stem> --enter=<tick>,<tick>,… presses Enter at each of
#               those ticks (M84): every screen reads it, so a run goes from
#               the map to the unit's screen and back, each printing the tick
#               it took the plant over at. Add --quit-after to end it.
extends Node2D

const Screen := preload("res://demo/screen.gd")
const Route := preload("res://demo/route.gd")

const SCENARIOS := "res://scenarios"
const MAX_TICKS_PER_FRAME := 64
const AUTO_TICKS := 200

const BACKGROUND := Screen.BACKGROUND
const SHELL := Screen.SHELL
const BODY := Color(0.15, 0.17, 0.20)
const INK := Screen.INK
const DIM := Screen.DIM
const COOL := Screen.COOL
const HOT := Screen.HOT
const FLAME := Screen.FLAME
const GOOD := Screen.GOOD
const BAD := Screen.BAD
const WARN := Screen.WARN
const IDLE := Color(0.30, 0.32, 0.36)

## The temperatures a pipe's colour runs between [K]: 20 °C to 450 °C.
const COLD_K := 293.15
const HOT_K := 723.15

## Where a pipe may be routed (M85): the page between the headings and the
## message line, left of the panel's backing.
const ROUTED := Rect2(24, 96, 836, 474)
## A tower (column, reactor) reaches this far above its highest neighbour and
## below its lowest [px], and no lower than TOWER_FLOOR (its name under it).
const TOWER_OVERHANG := 14.0
const TOWER_FLOOR := 532.0
## The dark border each pipe is drawn with, either side [px]: where two cross,
## the one on top cuts a gap that wide in the one under it.
const CASING := 2.5

## More nodes than this in one column of the layout and it is drawn as two.
const CROWDED := 5

## The page the plant is laid out on, inside the headings and the legend, left
## of the panel: a node's name runs ~55 px either side of it, so the right-most
## column of nodes stops that far short of the panel's backing (PANEL_X - 10).
## A crowded right-most column steps left, not right (`_lay_out`).
const AREA := Rect2(70, 110, 735, 440)

## Where each plant's hand-placed layout is kept (M81): `<plant stem>.json`,
## node name → [x, y] on the page. Beside the map, never in the plant file.
const LAYOUTS := "res://demo/layout"
## Where a node may be dragged or placed by a layout file: below the headings,
## above the message line, and as far right as AREA reaches (a name runs ~55 px
## either side of its node; right of that is the panel).
const PLACEABLE := Rect2(30, 100, 785, 460)

## The inspection panel: its left edge at the base width [px], its width, and
## the line height of its text.
const PANEL_X := 880.0
const PANEL_WIDTH := 255.0
## How far each nested level of the panel is set in [px].
const PANEL_INDENT := 9.0
const PANEL_LINE := 14.0
const PANEL_TOP := 104.0
## Deeper than this many nested names (`a.b.c.d`) and a field is left out: a
## column's holdup keeps its whole working state under `kind`.
const PANEL_DEPTH := 4
## A list longer than this is left out (a column's stage profiles).
const PANEL_LIST := 6

## Units the panel converts, by a field's last name. A name ending `_pa`, `_k`,
## `_w`, `_kg_s`, `_s` or `_m` says its unit itself (`_unit`); these are the
## ones that do not, each as its Rust type declares it (crates/core/src).
const PASCAL_FIELDS := ["pressure", "set_pressure", "design_pressure", "flare_pressure", "reference_pressure", "accumulation", "ceiling", "bubble_pressure", "full_open", "amount", "column_pressure"]
const KELVIN_FIELDS := ["temperature", "t_set", "coolant", "water_inlet", "return_temperature", "failure_temperature", "flame_temperature", "upper_cut", "drum_temperature"]
const WATT_FIELDS := ["duty", "reboiler_duty"]
const FLOW_FIELDS := ["flow", "mass_flow", "water_flow", "leak_mass_flow"]
const PERCENT_FIELDS := ["opening"]
const SECOND_FIELDS := ["tau"]
const METRE_FIELDS := ["h0", "height", "npsh_required", "diameter", "spacing", "weir_height"]

## One Up/Down on a valve: 5 % of its travel, onto that grid.
const VALVE_STEP := 0.05
## One Up/Down on a furnace: this share of its firing, and at least this [W].
const DUTY_SHARE := 0.05
const DUTY_STEP_MIN_W := 5.0e5
## The hole P+click makes in a pipe [m²]; a mend is area 0.
const HOLE_M2 := 0.001
## One Q/Z on a loop's target, by its variable: a temperature [K], a level
## [m], a pressure [Pa]; a flow moves this share, and at least this [kg/s].
const TEMPERATURE_STEP_K := 5.0
const LEVEL_STEP_M := 0.25
const PRESSURE_STEP_PA := 1.0e4
const FLOW_SHARE := 0.05
const FLOW_STEP_MIN_KG_S := 0.25
## `--hands`: the ticks run before the tour, and between its steps.
const HANDS_TICKS := 20
const HANDS_PAUSE := 10

## The one running plant (M84, demo/running.gd), and its simulator.
@onready var running: Node = Screen.running(get_tree())
@onready var sim: RefinerySim = running.sim
## The tick the plant stood at when this screen loaded or took it over.
var started_tick := 0

var plants: PackedStringArray = []
var plant_index := 0
var snapshot: Dictionary = {}
var halted := ""
var paused := false
var ticks_per_frame := 1
var auto_run := false
var shot_path := ""
## Node id → where it is drawn. Absent for a node the map hides.
var places: Dictionary = {}
## Pipe id → its corners, from its upstream node to its downstream one (M85,
## demo/route.gd): routed at load and when a dragged node is let go. Absent for
## a pipe with an end the map hides.
var routes: Dictionary = {}
## Pipe id → true for a pipe back to a node at or left of where it starts (a
## recycle, a reflux, a pumparound's draw): drawn first, so the forward pipes
## cross OVER them.
var backward: Dictionary = {}
## The largest flow on the map this tick [kg/s]: pipe widths are shares of it.
var largest_flow := 1.0
## Pipe id → how far its dashes have moved along it [px] (M83). Moved each
## frame the plant runs, by that pipe's flow then, so a paused or stopped plant
## holds them still and a screenshot taken at a tick is the same every run.
var dash_travel: Dictionary = {}
## What the mouse is over: {"node": id} or {"edge": id}, or {} for nothing. An
## id, never the entry itself: the snapshot is replaced every tick.
var hovered: Dictionary = {}
## `--hover=<name>`: what to point at once the plant is laid out; the mouse is
## then ignored, so a screenshot shows what was asked.
var hover_name := ""
## `--scroll=<lines>`: the panel scrolled that far, as the wheel would.
var hover_scroll := 0
## `--pick=<name>`: the valve or furnace to click once the plant is laid out.
var pick_name := ""
## How many of the panel's lines the mouse wheel has scrolled past.
var panel_scroll := 0
## What a click picked for Up/Down: {"node": id} of a valve or a furnace, or {}.
## Apart from `hovered`, so it stays when the mouse moves on.
var selected: Dictionary = {}
## Which of the snapshot's `controls` Tab has picked (modulo how many).
var selected_loop := 0
## The message line: the engine's last answer, and its colour.
var message := ""
var message_color := DIM
# What each command was in words (`running.said`), and the punctures the
# engine took (`running.holes`: [command text, edge id, area]), are kept with
# the plant (M84), so they outlast a visit to another screen. The snapshot
# reports a hole's flow but not its size, and a hole with no pressure across
# it flows nothing, so P+click asks `running.holes` whether to mend.
## `--hands`: the recorded tour instead of the mouse; `--break-at=<tick>`.
var hands_run := false
var break_at := -1
## `--hands-at=<tick>`: the tick the tour starts at (a trip that fires later).
var hands_at := HANDS_TICKS
## The folder the layout files are read from and written to; `--layout-dir=`
## moves it (a test that must not touch the shipped files).
var layout_dir := LAYOUTS
## How many nodes the plant's layout file placed, and what was wrong with it
## (a name the plant does not have, a place off the page): `--auto` fails on
## any, so a renamed node cannot leave a layout file quietly stale.
var hand_placed := 0
var layout_problems: Array = []
## The node Ctrl+drag is moving (-1: none), and where the mouse took it from.
var dragging := -1
var drag_offset := Vector2.ZERO
## A node was moved since the layout file was last read or written.
var layout_moved := false
## `--drag=<name>@<x>,<y>` (repeatable) and `--save-layout`: moves and a save
## made through the same calls as Ctrl+drag and Ctrl+S, once, at the first load.
var drags: Array = []
var save_layout := false


func _ready() -> void:
	Screen.fit(get_window())
	for file in DirAccess.get_files_at(SCENARIOS):
		if file.ends_with(".toml"):
			plants.append(file.trim_suffix(".toml"))
	plants.sort()
	var options := Screen.arguments()
	auto_run = options["auto"]
	var asked: String = options.get("plant", Screen.chosen_plant(get_tree(), ""))
	shot_path = options["shot"]
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--hover="):
			hover_name = arg.trim_prefix("--hover=")
		elif arg.begins_with("--scroll="):
			hover_scroll = int(arg.trim_prefix("--scroll="))
		elif arg.begins_with("--pick="):
			pick_name = arg.trim_prefix("--pick=")
		elif arg == "--hands":
			hands_run = true
		elif arg.begins_with("--hands-at="):
			hands_run = true
			hands_at = int(arg.trim_prefix("--hands-at="))
		elif arg.begins_with("--break-at="):
			break_at = int(arg.trim_prefix("--break-at="))
		elif arg.begins_with("--layout-dir="):
			layout_dir = arg.trim_prefix("--layout-dir=").trim_suffix("/")
		elif arg.begins_with("--drag="):
			drags.append(arg.trim_prefix("--drag="))
		elif arg == "--save-layout":
			save_layout = true
	if plants.is_empty():
		_halt("no plant files in %s" % SCENARIOS)
		return
	if asked != "" and plants.has(asked):
		plant_index = plants.find(asked)
	# Enter on another screen, or the menu's row with no plant named: the plant
	# running, as it stands (M84). A run naming --plant= starts it afresh, and
	# a recorded run's options are its first screen's, not this one's.
	var chosen_running := Screen.chosen_running(get_tree())
	var stem: String = running.scenario.get_file().trim_suffix(".toml")
	if running.scenario == "%s/%s.toml" % [SCENARIOS, stem] and plants.has(stem) and (chosen_running or (asked == "" and not auto_run)):
		auto_run = false
		hands_run = false
		shot_path = ""
		hover_name = ""
		pick_name = ""
		drags = []
		save_layout = false
		_load(plants.find(stem), false)
		return
	if auto_run:
		_run_all()
		return
	_load(plant_index)
	if hands_run:
		_hands()
		if shot_path == "":
			get_tree().quit(1 if halted != "" else 0)


## Load plant `index` fresh, or (`fresh` false) take over the plant running,
## which is that plant (M84): laid out as it stands, nothing loaded or ticked.
func _load(index: int, fresh: bool = true) -> void:
	plant_index = posmod(index, plants.size())
	halted = ""
	snapshot = {}
	places = {}
	routes = {}
	backward = {}
	hovered = {}
	panel_scroll = 0
	selected = {}
	selected_loop = 0
	message = ""
	dragging = -1
	dash_travel = {}
	if fresh:
		var err = JSON.parse_string(running.load_scenario("%s/%s.toml" % [SCENARIOS, plants[plant_index]]))
		if err != null:
			_halt("load failed: %s" % err["message"])
			return
		started_tick = sim.tick_index()
		if not _step():
			return
	else:
		snapshot = JSON.parse_string(sim.snapshot_json())
		_say("took over %s as it stands, t = %.1f s" % [plants[plant_index], float(snapshot["sim_time"])], DIM)
		print("map: took over %s at tick %d (%d hole(s) on record)" % [plants[plant_index], sim.tick_index(), running.holes.size()])
	started_tick = sim.tick_index()
	_lay_out()
	if not layout_problems.is_empty():
		_say("layout %s: %s" % [_layout_path(), "; ".join(layout_problems)], BAD)
		if not auto_run:
			print("map: %s: LAYOUT — %s" % [plants[plant_index], "; ".join(layout_problems)])
	elif hand_placed > 0 and not auto_run:
		print("map: %s: %d node(s) placed by %s" % [plants[plant_index], hand_placed, _layout_path()])
	_scripted_layout()
	if hover_name != "":
		_hover_named(hover_name)
		panel_scroll = hover_scroll
	if pick_name != "":
		for node in snapshot["nodes"]:
			if str(node["name"]) == pick_name:
				_click({"node": int(node["id"])}, false)
	queue_redraw()


## One engine tick. False when it could not be solved: the plant was then put
## back (`_recover`) or, with nothing to undo, stopped.
func _step() -> bool:
	var err = JSON.parse_string(sim.tick())
	if err != null:
		_recover(err)
		return false
	snapshot = JSON.parse_string(sim.snapshot_json())
	if hands_run and sim.tick_index() == break_at:
		_break()
	if Screen.enter_due(started_tick, sim.tick_index()):
		_closer()
		return false
	if Screen.escape_due(started_tick, sim.tick_index()):
		Screen.to_menu(get_tree())
		return false
	return true


## A tick could not be solved. The plant is put back as it stood just before
## the last command the engine accepted, and that command is undone — however
## many good ticks ran since (M76; pump.gd's `_recover`). With no command left
## to undo the plant itself cannot be solved, and the map stops as before.
func _recover(err: Dictionary) -> void:
	var failed_after := float(snapshot.get("sim_time", 0.0))
	var failed_tick := sim.tick_index() + 1
	var answer = JSON.parse_string(sim.recover())
	if answer.has("code"):
		_halt("tick %d: %s" % [failed_tick, err["message"]])
		return
	snapshot = JSON.parse_string(sim.snapshot_json())
	var undone := []
	for text in answer["dropped"]:
		undone.append(running.said.get(text, text))
		_forget_hole(text)
	_say(
		"the plant could not be solved after t = %.1f s — put back to t = %.1f s, undone: %s\n%s"
			% [failed_after, float(snapshot["sim_time"]), ", ".join(undone), err["message"]],
		BAD
	)
	print("map: RECOVERED — tick %d failed; put back to tick %d, undone: %s; %s" % [failed_tick, int(answer["rewound_to"]), ", ".join(undone), err["message"]])
	queue_redraw()


func _physics_process(delta: float) -> void:
	if auto_run or halted != "" or paused or snapshot.is_empty():
		return
	for i in ticks_per_frame:
		if not _step():
			return
	_move_dashes(delta)
	queue_redraw()
	if shot_path != "" and sim.tick_index() >= AUTO_TICKS:
		var path := shot_path
		shot_path = ""
		_shoot(path)


func _shoot(path: String) -> void:
	paused = true
	queue_redraw()
	await Screen.shoot(self, "map", path)
	get_tree().quit()


## `--auto`: every plant loaded, laid out and run, one line each.
func _run_all() -> void:
	var failed := 0
	for i in plants.size():
		_load(i)
		var ticks := 1
		while halted == "" and ticks < AUTO_TICKS and _step():
			ticks += 1
		if halted != "" or not layout_problems.is_empty():
			# A layout file naming a node the plant lacks fails too: stale.
			failed += 1
		if not snapshot.is_empty():
			# Every node's and pipe's panel, built and thrown away: a field
			# shape the walker cannot read fails here, on every plant.
			for node in snapshot["nodes"]:
				_inspect_lines({"node": int(node["id"])})
			for edge in snapshot["edges"]:
				_inspect_lines({"edge": int(edge["id"])})
		var placed := ", %d by hand" % hand_placed if hand_placed > 0 else ""
		var verdict := "ok" if halted == "" else halted
		if not layout_problems.is_empty():
			verdict = "LAYOUT: " + "; ".join(layout_problems)
		print("map: %-36s %3d nodes drawn%s, %3d pipes, %s" % [plants[i], places.size(), placed, snapshot.get("edges", []).size(), verdict])
	print("map: %d plants, %d stopped" % [plants.size(), failed])
	get_tree().quit(1 if failed > 0 else 0)


func _input(event: InputEvent) -> void:
	if event is InputEventMouseMotion and dragging >= 0:
		# A node on the move follows the mouse; nothing is hovered meanwhile.
		_drag_to(get_local_mouse_position())
		queue_redraw()
		return
	if event is InputEventMouseButton and not event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
		if dragging >= 0:
			_drag_end()
			queue_redraw()
		return
	if event is InputEventMouseMotion and hover_name == "" and not places.is_empty():
		# The local position, not the event's: the page is stretched to the
		# window, and `places` are page coordinates. Over the panel itself
		# the choice stays, so the wheel can scroll it.
		var mouse := get_local_mouse_position()
		if _panel_rect(get_viewport_rect().size).has_point(mouse):
			return
		var over := _hit(mouse)
		if over != hovered:
			hovered = over
			panel_scroll = 0
			queue_redraw()
		return
	if event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
		# A click on the plant (not on the panel) is a hand on what it hits.
		# With Ctrl held it is a node picked up to move instead, and never a
		# click: a click on a pump would start or stop it.
		var mouse := get_local_mouse_position()
		if places.is_empty() or _panel_rect(get_viewport_rect().size).has_point(mouse):
			return
		if event.ctrl_pressed:
			var under := _hit(mouse)
			if under.has("node"):
				_drag_start(int(under["node"]), mouse)
			return
		if halted == "":
			_click(_hit(mouse), Input.is_key_pressed(KEY_P))
			queue_redraw()
		return
	if event is InputEventMouseButton and event.pressed:
		# The wheel scrolls the panel: a column reports more than fits, and
		# so may a plant's loops and trips.
		if event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			panel_scroll += 3
		elif event.button_index == MOUSE_BUTTON_WHEEL_UP:
			panel_scroll = maxi(0, panel_scroll - 3)
		queue_redraw()
		return
	if not (event is InputEventKey and event.pressed and not event.echo):
		return
	match event.keycode:
		KEY_ENTER, KEY_KP_ENTER:
			_closer()
			return
		KEY_ESCAPE:
			Screen.to_menu(get_tree())
			return
		KEY_F11:
			Screen.toggle_fullscreen(get_window())
		KEY_RIGHT, KEY_PAGEDOWN:
			_load(plant_index + 1)
		KEY_LEFT, KEY_PAGEUP:
			_load(plant_index - 1)
		KEY_SPACE:
			paused = not paused
		KEY_BRACKETLEFT:
			ticks_per_frame = maxi(1, ticks_per_frame / 2)
		KEY_BRACKETRIGHT:
			ticks_per_frame = mini(MAX_TICKS_PER_FRAME, ticks_per_frame * 2)
		KEY_UP:
			_act("up")
		KEY_DOWN:
			_act("down")
		KEY_TAB:
			_act("next_loop")
		KEY_A:
			_act("loop_mode")
		KEY_Q:
			_act("target_up")
		KEY_Z:
			_act("target_down")
		KEY_R:
			_act("reset")
		KEY_M:
			_act("hand")
		KEY_S:
			if event.ctrl_pressed and not snapshot.is_empty():
				_save_layout()
	queue_redraw()


# ------------------------------------------------------------------ hands
#
# The mouse and the keys call these; so does `--hands`, so the recorded tour
# goes through exactly what a person's clicks do. Nothing is checked before it
# is sent: the engine refuses, in its own words, what it will not do.

## A click on `target` ({"node": id}, {"edge": id} or {} for nothing), with P
## held or not.
func _click(target: Dictionary, puncture: bool) -> void:
	if halted != "":
		return
	if target.has("edge"):
		var edge := _entry("edges", int(target["edge"]))
		if edge.is_empty():
			return
		# The loader splits a punctureable pipe at a junction it names
		# `<pipe>__leak_point`; the upstream half keeps the pipe's name and is
		# the one `puncture_pipe` takes. A click on the downstream half means
		# the same pipe.
		var start := _entry("nodes", int(edge["from"]))
		if str(start.get("name", "")).ends_with("__leak_point"):
			for other in snapshot["edges"]:
				if int(other["to"]) == int(start["id"]):
					edge = other
					break
		var label := str(edge["name"]).replace("_", " ")
		if not puncture:
			_say("%s: hold P and click to make a hole in it, or mend the one the map made" % label, DIM)
			return
		var id := int(edge["id"])
		var area := 0.0 if _holed(edge) else HOLE_M2
		var what := "mend %s" % label if area == 0.0 else "hole in %s, %s m²" % [label, _number(area)]
		var command := {"cmd": "puncture_pipe", "edge": id, "area": area}
		if _send(what, command):
			running.holes.append([JSON.stringify(command), id, area])
		return
	if not target.has("node"):
		selected = {}
		return
	var node := _entry("nodes", int(target["node"]))
	if node.is_empty():
		return
	if str(node["name"]).ends_with("__leak_point"):
		# The junction the loader puts in the middle of a punctureable pipe
		# (drawn as a dot, no name): a click on it is a click on that pipe.
		for edge in snapshot["edges"]:
			if int(edge["to"]) == int(node["id"]):
				_click({"edge": int(edge["id"])}, puncture)
				return
	var label := str(node["name"]).replace("_", " ")
	var type := _kind(node)
	match type:
		"valve", "furnace", "cooler":
			selected = {"node": int(node["id"])}
			# The loop that moves it is picked too, so A and Q/Z work the loop
			# M takes it from.
			var owner := _loop_on(int(node["id"]))
			if owner.is_empty():
				_say("%s picked — Up/Down move it" % label, INK)
				return
			selected_loop = (snapshot["controls"] as Array).find(owner)
			if owner["mode"] == "auto":
				_say("%s picked — loop %s holds it in AUTO: M takes it by hand, Q/Z move the loop's target" % [label, _label(owner)], INK)
			else:
				_say("%s picked — loop %s is in MANUAL, so it is yours: Up/Down move it, M gives it back" % [label, _label(owner)], INK)
		"pump":
			selected = {}
			var on := not bool(node["kind"].get("on", true))
			_send("%s %s" % ["start" if on else "stop", label], {"cmd": "set_pump_on", "node": int(node["id"]), "on": on})
		_:
			selected = {}
			_say("%s: the map has no hand on a %s (its own screen may)" % [label, type.replace("_", " ")], DIM)


## One key's action: "up"/"down" on what is picked, "next_loop", "loop_mode",
## "target_up"/"target_down" on the picked loop, "reset" every tripped trip.
func _act(action: String) -> void:
	if halted != "" or snapshot.is_empty():
		return
	match action:
		"up", "down":
			var direction := 1.0 if action == "up" else -1.0
			var node := _entry("nodes", int(selected.get("node", -1)))
			if node.is_empty():
				_say("click a valve, a furnace or a cooler first: Up/Down move what is picked", DIM)
				return
			var label := str(node["name"]).replace("_", " ")
			var kind: Dictionary = node["kind"]
			if _kind(node) == "valve":
				# On the 5 % grid, and sent as it falls: past 0 or 100 % the
				# engine says why not.
				var opening := snappedf(float(kind.get("opening", 0.0)) + direction * VALVE_STEP, VALVE_STEP)
				_send("%s to %.0f %% open" % [label, opening * 100.0], {"cmd": "set_valve_opening", "node": int(node["id"]), "opening": opening})
			else:
				var duty := float(kind.get("duty", 0.0))
				var next := duty + direction * maxf(duty * DUTY_SHARE, DUTY_STEP_MIN_W)
				# A step down that would cross nothing lands on it, so the
				# furnace can be put out; one from nothing is sent as it is.
				if duty > 0.0 and next < 0.0:
					next = 0.0
				if _kind(node) == "cooler":
					_send("cool %s by %.2f MW" % [label, next / 1.0e6], {"cmd": "set_cooler_duty", "node": int(node["id"]), "duty": next})
				else:
					_send("fire %s at %.2f MW" % [label, next / 1.0e6], {"cmd": "set_furnace_duty", "node": int(node["id"]), "duty": next})
		"next_loop":
			var loops: Array = snapshot.get("controls", [])
			if loops.is_empty():
				_say("this plant has no control loops", DIM)
				return
			selected_loop = (selected_loop + 1) % loops.size()
			_say("loop %s picked — A auto/manual, Q/Z its target" % str(loops[selected_loop]["name"]).replace("_", " "), INK)
		"loop_mode":
			var loop := _picked_loop()
			if loop.is_empty():
				return
			var mode := "manual" if loop["mode"] == "auto" else "auto"
			_send("%s to %s" % [str(loop["name"]).replace("_", " "), mode.to_upper()], {"cmd": "set_controller_mode", "loop_id": int(loop["id"]), "mode": mode})
		"target_up", "target_down":
			var loop := _picked_loop()
			if loop.is_empty():
				return
			_step_target(loop, 1.0 if action == "target_up" else -1.0)
		"hand":
			# M: the loop that moves what is picked to MANUAL, or back to AUTO.
			var node := _entry("nodes", int(selected.get("node", -1)))
			if node.is_empty():
				_say("click a valve, a furnace or a cooler first: M takes it from the loop that holds it, or gives it back", DIM)
				return
			var label := str(node["name"]).replace("_", " ")
			var owner := _loop_on(int(node["id"]))
			if owner.is_empty():
				_say("%s: no control loop moves it, so Up/Down are yours already" % label, DIM)
				return
			if owner["mode"] == "auto":
				_send("%s taken by hand: loop %s to MANUAL" % [label, _label(owner)], {"cmd": "set_controller_mode", "loop_id": int(owner["id"]), "mode": "manual"})
			else:
				_send("%s given back: loop %s to AUTO" % [label, _label(owner)], {"cmd": "set_controller_mode", "loop_id": int(owner["id"]), "mode": "auto"})
		"reset":
			var reset := 0
			for trip in snapshot.get("trips", []):
				if trip["state"]["status"] == "tripped":
					_send("reset %s" % str(trip["name"]).replace("_", " "), {"cmd": "reset_trip", "trip_id": int(trip["id"])})
					reset += 1
			if reset == 0:
				_say("reset: no trip on this plant has tripped", DIM)


## The control loop that moves node `id` — the one whose `actuates` names it
## (M80) — or {}. The engine refuses two loops on one actuator, so it is one.
func _loop_on(id: int) -> Dictionary:
	for loop in snapshot.get("controls", []):
		if int(loop.get("actuates", -1)) == id:
			return loop
	return {}


## A loop's or a node's name, as the map writes it.
func _label(entry: Dictionary) -> String:
	return str(entry["name"]).replace("_", " ")


## The line under the headings: what is picked and what each key does to it
## now, from this tick's snapshot (the loop's mode as the engine reports it).
## [text, colour].
func _keys_line() -> Array:
	var node := _entry("nodes", int(selected.get("node", -1)))
	if node.is_empty():
		return ["click a valve, a furnace or a cooler to pick it   ·   point at anything to see what a click on it does", DIM]
	var kind: Dictionary = node["kind"]
	var now := ""
	var step := ""
	if _kind(node) == "valve":
		now = "%s %% open" % _number(float(kind.get("opening", 0.0)) * 100.0)
		step = "Up/Down open or close it 5 %"
	else:
		now = "%.2f MW" % (float(kind.get("duty", 0.0)) / 1.0e6)
		step = "Up/Down move its %s 5 %% (at least 0.5 MW)" % ("cooling" if _kind(node) == "cooler" else "firing")
	var head := "PICKED %s, %s" % [_label(node), now]
	# A fired trip may hold what is picked whatever its loop's mode: the
	# engine says so if it does; this line says one has fired.
	for trip in snapshot.get("trips", []):
		if trip["state"]["status"] == "tripped":
			head = "(a trip has fired: R resets it)   " + head
			break
	var owner := _loop_on(int(node["id"]))
	if owner.is_empty():
		return ["%s — %s   (no loop moves it)" % [head, step], WARN]
	if owner["mode"] == "auto":
		# A cascade's inner loop: while the outer one is in AUTO it writes the
		# target, and the engine refuses Q/Z on it.
		var targets := "Q/Z move the target"
		for loop in snapshot.get("controls", []):
			if int(loop.get("drives", -1)) == int(owner["id"]) and loop["mode"] == "auto":
				targets = "the target is set by loop %s (Tab to it)" % _label(loop)
		return ["%s — loop %s holds it in AUTO, target %s:   M take it by hand   %s" % [head, _label(owner), _value_text(owner["setpoint"]), targets], WARN]
	return ["%s — yours (loop %s in MANUAL):   %s   M give it back to the loop" % [head, _label(owner), step], WARN]


## The loop Tab has picked, or {} (said on the message line) when there is none.
func _picked_loop() -> Dictionary:
	var loops: Array = snapshot.get("controls", [])
	if loops.is_empty():
		_say("this plant has no control loops", DIM)
		return {}
	return loops[selected_loop % loops.size()]


## A loop's target moved one key step, in the loop's own variable, and sent
## tagged with it.
func _step_target(loop: Dictionary, direction: float) -> void:
	var setpoint: Dictionary = loop["setpoint"]
	var label := str(loop["name"]).replace("_", " ")
	var tagged := {"variable": setpoint["variable"]}
	match setpoint["variable"]:
		"temperature":
			# Snapped in °C, so 362 °C steps to 365 and 360.
			tagged["k"] = snappedf(float(setpoint["k"]) - 273.15 + direction * TEMPERATURE_STEP_K, TEMPERATURE_STEP_K) + 273.15
		"level":
			tagged["m"] = snappedf(float(setpoint["m"]) + direction * LEVEL_STEP_M, LEVEL_STEP_M)
		"pressure":
			tagged["pa"] = snappedf(float(setpoint["pa"]) + direction * PRESSURE_STEP_PA, PRESSURE_STEP_PA)
		"flow":
			var flow := float(setpoint["kg_per_s"])
			tagged["kg_per_s"] = flow + direction * maxf(absf(flow) * FLOW_SHARE, FLOW_STEP_MIN_KG_S)
		_:
			_say("%s: the map has no step for a %s target" % [label, setpoint["variable"]], BAD)
			return
	_send("%s target to %s" % [label, _value_text(tagged)], {"cmd": "set_setpoint", "loop_id": int(loop["id"]), "value": tagged})


## Whether P+click mends `edge` rather than holing it: the map's own last
## puncture of it left a hole, or something is leaking out of it now.
func _holed(edge: Dictionary) -> bool:
	var id := int(edge["id"])
	var holes: Array = running.holes
	for i in range(holes.size() - 1, -1, -1):
		if int(holes[i][1]) == id:
			return float(holes[i][2]) > 0.0
	return float(edge.get("leak_mass_flow", 0.0)) > 0.0


## A recovery undid the puncture sent as `text`: the latest such is forgotten.
func _forget_hole(text: String) -> void:
	var holes: Array = running.holes
	for i in range(holes.size() - 1, -1, -1):
		if holes[i][0] == text:
			holes.remove_at(i)
			return


## Send one command and put the engine's answer on the message line — its own
## refusal when it says no (pump.gd's `_send`). True when the engine took it.
func _send(what: String, command: Dictionary) -> bool:
	var text := JSON.stringify(command)
	var err = JSON.parse_string(sim.apply_command(text))
	if err == null:
		running.said[text] = what
		_say(what, GOOD)
		# A command moves the kind at once; read it back so the map shows the
		# new value before the next tick.
		snapshot = JSON.parse_string(sim.snapshot_json())
		return true
	_say("%s: REFUSED — %s" % [what, err["message"]], BAD)
	return false


func _say(text: String, color: Color) -> void:
	message = text
	message_color = color
	if hands_run:
		print("map: t=%d  %s%s" % [sim.tick_index(), "" if color != BAD else "[red] ", text.replace("\n", " / ")])


## `--hands`: 20 ticks, then the first valve, pump and furnace on the plant, a
## pipe that cannot be holed and one that can, and the loops and trips, each
## through `_click` and `_act`, with a few ticks between; then on to
## `--break-at` if one was given, and 20 ticks more.
func _hands() -> void:
	if halted != "":
		return
	_run_ticks(hands_at - sim.tick_index())
	print("map: hands on %s at tick %d" % [plants[plant_index], sim.tick_index()])
	var first := {}
	for node in snapshot["nodes"]:
		var type := _kind(node)
		if type in ["valve", "pump", "furnace"] and not first.has(type) and places.has(int(node["id"])):
			first[type] = {"node": int(node["id"])}
	if first.has("pump"):
		_click(first["pump"], false)
		_run_ticks(HANDS_PAUSE)
		_click(first["pump"], false)
	if first.has("furnace"):
		_click(first["furnace"], false)
		_act("up")
		_act("down")
	# A pipe the scenario made punctureable keeps its label on its upstream
	# half, which ends at the junction the loader names `<pipe>__leak_point`.
	var holeable := {}
	var plain := {}
	var downstream := {}
	for edge in snapshot["edges"]:
		if _pipe_path(edge).is_empty():
			continue
		var holes_here := str(_entry("nodes", int(edge["to"])).get("name", "")).ends_with("__leak_point")
		if holes_here and holeable.is_empty():
			holeable = {"edge": int(edge["id"])}
		elif not holes_here and plain.is_empty():
			plain = {"edge": int(edge["id"])}
		if str(_entry("nodes", int(edge["from"])).get("name", "")).ends_with("__leak_point"):
			downstream = {"edge": int(edge["id"])}
	if not plain.is_empty():
		_click(plain, false)
		_click(plain, true)
	if not holeable.is_empty():
		_click(holeable, true)
		_run_ticks(HANDS_PAUSE)
		_click(holeable, true)
	if not downstream.is_empty():
		# The pipe's downstream half holes and mends the same pipe; so does a
		# click at the pipe's middle, where the mouse finds the junction the
		# loader put there — sent through `_hit`, as the mouse's would be.
		_click(downstream, true)
		_click(downstream, true)
		var middle: Vector2 = places[int(_entry("edges", int(downstream["edge"]))["from"])]
		_click(_hit(middle), true)
		_click(_hit(middle), true)
	var loops: Array = snapshot.get("controls", [])
	if loops.size() > 1:
		_act("next_loop")
	_act("loop_mode")
	_act("loop_mode")
	_act("target_up")
	_act("target_down")
	_act("reset")
	if first.has("valve"):
		_click(first["valve"], false)
		_act("up")
		_act("down")
		_act("down")
	# The first valve, furnace or cooler a loop holds: Up (refused while the
	# loop is in AUTO), M takes it by hand, Up, M gives it back. Every loop
	# names what it moves (`actuates`) or the loop it sets (`drives`); a
	# library built before M80 names neither, and the tour says so and stops.
	var held := -1
	for loop in snapshot.get("controls", []):
		if not loop.has("actuates") and not loop.has("drives"):
			_halt("loop '%s' names nothing it moves: the Godot library predates M80 — rebuild it" % loop["name"])
			return
		if held < 0 and loop.has("actuates") and places.has(int(loop["actuates"])):
			held = int(loop["actuates"])
	if held >= 0:
		_click({"node": held}, false)
		print("map: keys line: %s" % _keys_line()[0])
		_act("up")
		_act("hand")
		print("map: keys line: %s" % _keys_line()[0])
		_act("up")
		_act("hand")
	if break_at > sim.tick_index():
		_run_ticks(break_at - sim.tick_index())
	_run_ticks(HANDS_TICKS)
	print("map: hands done at tick %d, %s" % [sim.tick_index(), "ok" if halted == "" else halted])


func _run_ticks(count: int) -> void:
	for i in maxi(0, count):
		if halted != "":
			return
		_step()


## `--break-at`: the first supply set to -270 °C, through `_send` like any
## command — pump.gd's break, which the gas-locked pump cannot survive.
func _break() -> void:
	for node in snapshot["nodes"]:
		if _kind(node) == "source":
			_send("%s to -270 °C" % str(node["name"]).replace("_", " "), {"cmd": "set_source_temperature", "node": int(node["id"]), "temperature": 3.15})
			return


# ----------------------------------------------------------------- layout

func _kind(node: Dictionary) -> String:
	return str(node["kind"]["type"])


## The atmosphere is where leaks, vents and overflows end; drawn as a node it
## would gather a line from every punctureable pipe. The map shows what
## reaches it at the pipe's own end instead (`_draw_pipe`).
func _hidden(node: Dictionary) -> bool:
	return _kind(node) == "atmosphere"


## Columns of nodes, left to right by how many pipes the liquid crosses to
## reach each from a supply; top to bottom within a column by where each one's
## upstream neighbours sit, so pipes cross as little as a single pass manages.
## Fixed at load from the pipes' declared directions: a flow that reverses
## does not move the drawing.
func _lay_out() -> void:
	var nodes: Array = snapshot["nodes"]
	var edges: Array = snapshot["edges"]
	var downstream := {}
	var upstream := {}
	for node in nodes:
		downstream[int(node["id"])] = []
		upstream[int(node["id"])] = []
	for edge in edges:
		downstream[int(edge["from"])].append(int(edge["to"]))
		upstream[int(edge["to"])].append(int(edge["from"]))

	# Depth by breadth-first search from every node nothing feeds (a plant's
	# supplies). Shortest distance, so a recycle loop cannot run it away.
	var depth := {}
	var queue: Array = []
	for node in nodes:
		var id := int(node["id"])
		if upstream[id].is_empty() and not _hidden(node):
			depth[id] = 0
			queue.append(id)
	if queue.is_empty() and not nodes.is_empty():
		depth[int(nodes[0]["id"])] = 0
		queue.append(int(nodes[0]["id"]))
	var head := 0
	while head < queue.size():
		var id: int = queue[head]
		head += 1
		for next in downstream[id]:
			if not depth.has(next):
				depth[next] = depth[id] + 1
				queue.append(next)
	# What no supply reaches (a closed loop's far side): beside its neighbour.
	for pass_index in nodes.size():
		var placed_any := false
		for node in nodes:
			var id := int(node["id"])
			if depth.has(id):
				continue
			for next in downstream[id]:
				if depth.has(next):
					depth[id] = maxi(0, depth[next] - 1)
					placed_any = true
					break
		if not placed_any:
			break

	var columns := {}
	var deepest := 0
	for node in nodes:
		var id := int(node["id"])
		if _hidden(node):
			continue
		var d: int = depth.get(id, 0)
		deepest = maxi(deepest, d)
		if not columns.has(d):
			columns[d] = []
		columns[d].append(id)

	places = {}
	var fractions := {}
	for d in deepest + 1:
		var column: Array = columns.get(d, [])
		# Order by the mean height of the neighbours already placed upstream.
		var keyed: Array = []
		for id in column:
			var total := 0.0
			var count := 0
			for from in upstream[id]:
				if fractions.has(from):
					total += fractions[from]
					count += 1
			keyed.append([total / count if count > 0 else 0.5, id])
		keyed.sort()
		# A crowded column (a crude unit's five product tanks beside its three
		# pumps) is drawn as two, every other node a little to the right, so
		# each has twice the height to itself.
		var step := AREA.size.x / deepest if deepest > 0 else AREA.size.x
		var crowded := keyed.size() > CROWDED
		for i in keyed.size():
			var id: int = keyed[i][1]
			var fraction := (i + 1.0) / (keyed.size() + 1.0)
			fractions[id] = fraction
			var x := AREA.position.x + (AREA.size.x * d / deepest if deepest > 0 else AREA.size.x * 0.5)
			if crowded and i % 2 == 1:
				# The right-most column steps left: right of it is the panel.
				# (No shipped plant has a crowded right-most column, 2026-11-15:
				# a guard for the plant files to come.)
				x += minf(96.0, step * 0.45) * (-1.0 if d == deepest and deepest > 0 else 1.0)
			# Level with each other where they can be: a pipe between two
			# nodes at one height runs straight (M85; the half-step stagger
			# that kept a slanted line off the names would be a jog in it now).
			places[id] = Vector2(x, AREA.position.y + AREA.size.y * fraction)
	# Last, so the computed order of the nodes it does not name is unchanged.
	_apply_layout()
	_route_pipes()


# ------------------------------------------------------- a layout by hand
#
# Computed layout will never suit every plant (docs/plans/plant-map-screen.md,
# slice 3). A person drags a node with Ctrl held and Ctrl+S keeps every drawn
# node's place in `demo/layout/<plant>.json`; the next load puts each node the
# file names there, and the computed place stands for any it does not (a node
# a plant file adds later). Where a node is drawn changes nothing else.

func _layout_path() -> String:
	return "%s/%s.json" % [layout_dir, plants[plant_index]]


## The plant's layout file, if it has one, over the computed places. Anything
## wrong with it is a problem named, and the rest of it still applied.
func _apply_layout() -> void:
	hand_placed = 0
	layout_problems = []
	layout_moved = false
	var path := _layout_path()
	if not FileAccess.file_exists(path):
		return
	var file = JSON.parse_string(FileAccess.get_file_as_string(path))
	if not file is Dictionary:
		layout_problems.append("not a JSON object of node name → [x, y]")
		return
	var ids := {}
	for node in snapshot["nodes"]:
		var name := str(node["name"])
		if ids.has(name):
			layout_problems.append("two nodes are named '%s'" % name)
		ids[name] = int(node["id"])
	for name in file:
		var at = file[name]
		if not ids.has(name):
			layout_problems.append("no node is named '%s'" % name)
			continue
		if not places.has(ids[name]):
			layout_problems.append("'%s' is not drawn on the map" % name)
			continue
		if not (at is Array and at.size() == 2 and typeof(at[0]) in [TYPE_INT, TYPE_FLOAT] and typeof(at[1]) in [TYPE_INT, TYPE_FLOAT]):
			layout_problems.append("'%s' is not at [x, y]" % name)
			continue
		var point := Vector2(float(at[0]), float(at[1]))
		if point != _placeable(point):
			layout_problems.append("'%s' at (%d, %d) is off the page" % [name, point.x, point.y])
		places[ids[name]] = _placeable(point)
		hand_placed += 1


## `point`, moved onto the part of the page a node may sit on.
func _placeable(point: Vector2) -> Vector2:
	return point.clamp(PLACEABLE.position, PLACEABLE.end)


## Ctrl+press on a node: it follows the mouse until the button is let go.
func _drag_start(id: int, mouse: Vector2) -> void:
	dragging = id
	drag_offset = places[id] - mouse


func _drag_to(mouse: Vector2) -> void:
	if dragging < 0:
		return
	places[dragging] = _placeable(mouse + drag_offset)
	layout_moved = true


func _drag_end() -> void:
	if dragging < 0:
		return
	var node := _entry("nodes", dragging)
	dragging = -1
	_route_pipes()
	_say("moved %s — Ctrl+S keeps the layout (%s); Left/Right drops it" % [node.get("name", "?"), _layout_path()], WARN)


## Ctrl+S: every drawn node's place, in whole pixels and sorted by name, so a
## file edited by hand and one saved here read alike. Run from the project
## folder the file lands in `demo/layout/`; where that cannot be written (an
## exported game) the message line says so and nothing changes.
func _save_layout() -> void:
	var names: Array = []
	var at := {}
	for node in snapshot.get("nodes", []):
		var id := int(node["id"])
		if places.has(id):
			names.append(str(node["name"]))
			at[str(node["name"])] = places[id]
	names.sort()
	# One node a line, `"name": [x, y]`, so the file reads (and is edited) as
	# a list of places; JSON.stringify would spread each pair over four lines.
	var rows: Array = []
	for name in names:
		rows.append("\t%s: [%d, %d]" % [JSON.stringify(name), roundi(at[name].x), roundi(at[name].y)])
	var path := _layout_path()
	DirAccess.make_dir_recursive_absolute(layout_dir)
	var file := FileAccess.open(path, FileAccess.WRITE)
	if file == null:
		_say("the layout could not be written to %s: %s" % [path, error_string(FileAccess.get_open_error())], BAD)
		print("map: layout NOT saved — %s: %s" % [path, error_string(FileAccess.get_open_error())])
		return
	file.store_string("{\n%s\n}\n" % ",\n".join(rows))
	file.close()
	layout_moved = false
	hand_placed = names.size()
	layout_problems = []
	_say("layout saved: %d nodes in %s" % [names.size(), ProjectSettings.globalize_path(path)], GOOD)
	print("map: layout saved — %d nodes in %s" % [names.size(), path])


## `--drag=<name>@<x>,<y>` and `--save-layout`, through the calls the mouse and
## Ctrl+S make; once, so Left/Right later does not repeat them on another plant.
func _scripted_layout() -> void:
	for drag in drags:
		var parts: PackedStringArray = str(drag).split("@")
		var xy: PackedStringArray = parts[1].split(",") if parts.size() == 2 else PackedStringArray()
		var id := -1
		for node in snapshot["nodes"]:
			if parts.size() == 2 and str(node["name"]) == parts[0] and places.has(int(node["id"])):
				id = int(node["id"])
		if id < 0 or xy.size() != 2:
			print("map: --drag=%s: not <drawn node name>@<x>,<y>" % drag)
			continue
		_drag_start(id, places[id])
		_drag_to(Vector2(float(xy[0]), float(xy[1])))
		print("map: dragged %s to (%d, %d)" % [parts[0], places[id].x, places[id].y])
		_drag_end()
	drags = []
	if save_layout:
		save_layout = false
		_save_layout()


# ---------------------------------------------------------------- drawing

func _draw() -> void:
	var size := get_viewport_rect().size
	draw_rect(Rect2(Vector2.ZERO, size), BACKGROUND)
	if plants.is_empty():
		_text(Vector2(30, 40), "HALTED — %s" % halted, BAD)
		return
	_text(Vector2(30, 34), plants[plant_index].replace("_", " "), INK, 20)
	_text(
		Vector2(30, 60),
		"plant %d of %d    t = %.1f s    %d tick(s) per frame%s%s" % [
			plant_index + 1, plants.size(), float(snapshot.get("sim_time", 0.0)), ticks_per_frame, "    PAUSED" if paused else "",
			"    layout moved, not kept (Ctrl+S)" if layout_moved else ""
		],
		DIM
	)
	if halted != "":
		draw_multiline_string(ThemeDB.fallback_font, Vector2(30, 86), "HALTED — %s" % halted, HORIZONTAL_ALIGNMENT_LEFT, size.x - 60, 14, 4, BAD)
	elif not snapshot.is_empty():
		# What is picked and what the keys do to it, left of the panel.
		var keys := _keys_line()
		var room := _panel_rect(size).position.x - 50.0
		_text(Vector2(30, 86), keys[0], keys[1], Screen.fitting_size(keys[0], room, 13, 9))
	if snapshot.is_empty():
		_draw_legend(size)
		return

	largest_flow = _largest_flow()
	# The pipes back first, the forward ones over them: where two cross, the
	# one on top cuts a gap in the other with its dark border (`_draw_pipe`).
	for edge in snapshot["edges"]:
		if backward.has(int(edge["id"])):
			_draw_pipe(edge)
	for edge in snapshot["edges"]:
		if not backward.has(int(edge["id"])):
			_draw_pipe(edge)
	for node in snapshot["nodes"]:
		if places.has(int(node["id"])):
			_draw_node(node, places[int(node["id"])])
			if selected.get("node", -1) == int(node["id"]):
				# What Up/Down move, in a warmer colour; the hover's outline
				# then sits just inside it, so both read at once and neither
				# reaches the name under the node.
				draw_rect(_box(int(node["id"])).grow(5.0), WARN, false, 2.0)
			if hovered.get("node", -1) == int(node["id"]):
				var grow := 2.0 if selected.get("node", -1) == int(node["id"]) else 5.0
				draw_rect(_box(int(node["id"])).grow(grow), INK, false, 1.5)
	_draw_panel(size)
	_draw_legend(size)


## The pipes' key at the top right of the headings, the message line and the
## keys at the bottom. The message line is the engine's last answer: green
## taken, red refused (two lines at most: a recovery's says what it undid).
func _draw_legend(size: Vector2) -> void:
	var x := size.x - 560.0
	_text(Vector2(x, 60), "pipes: width = flow, colour = temperature", DIM, 13)
	for i in 60:
		draw_rect(Rect2(Vector2(x + 300 + i * 3.0, 50), Vector2(3, 10)), COOL.lerp(HOT, i / 59.0))
	_text(Vector2(x + 272, 60), "20", DIM, 12)
	_text(Vector2(x + 486, 60), "450 °C", DIM, 12)
	if message != "":
		draw_multiline_string(ThemeDB.fallback_font, Vector2(30, size.y - 60.0), message, HORIZONTAL_ALIGNMENT_LEFT, size.x - 60.0, 13, 2, message_color)
	# Two lines: the plant's controls, then the keys that move between plants
	# and screens — what keeps the plant running and what starts one over.
	var controls := "click a valve/furnace/cooler: Up/Down, M by hand   click a pump: on/off   P+click a pipe: hole/mend   Tab A Q Z loops   R reset trips   Ctrl+drag/Ctrl+S layout"
	var moves := "Space pause   [ ] speed   Left/Right start another plant over"
	Screen.legend(self, Vector2(30, size.y - 30.0), [controls, moves], DIM, 13, 16.0, size.x - 60.0, "Enter closer look, same plant")


## The largest flow on the map this tick [kg/s], never zero.
func _largest_flow() -> float:
	var largest := 1.0e-9
	for edge in snapshot["edges"]:
		largest = maxf(largest, absf(float(edge["stream"]["mass_flow"])))
	return largest


## Every running pipe's dashes moved on by `seconds` at the speed its share of
## the largest flow sets, the way it flows now (M83, Screen.dash_speed).
func _move_dashes(seconds: float) -> void:
	var largest := _largest_flow()
	for edge in snapshot["edges"]:
		var flow := float(edge["stream"]["mass_flow"])
		if absf(flow) > 1.0e-6:
			var id := int(edge["id"])
			dash_travel[id] = fposmod(float(dash_travel.get(id, 0.0)) + Screen.dash_speed(flow / largest) * seconds, Screen.DASH_PERIOD)


func _warmth(kelvin: float) -> Color:
	return COOL.lerp(HOT, clampf((kelvin - COLD_K) / (HOT_K - COLD_K), 0.0, 1.0))


## A pipe: a line from its upstream node to its downstream one, as wide as its
## share of the map's largest flow, coloured by its stream's temperature, an
## arrowhead the way it flows NOW. One that ends at the atmosphere is a short
## spray at its upstream node, drawn only while something leaves through it.
func _draw_pipe(edge: Dictionary) -> void:
	var from := int(edge["from"])
	var to := int(edge["to"])
	var flow := float(edge["stream"]["mass_flow"])
	var running := absf(flow) > 1.0e-6
	if not places.has(from) or not places.has(to):
		var at: Vector2 = places.get(from, places.get(to, Vector2.INF))
		if at == Vector2.INF or not running:
			return
		for i in 5:
			var angle := deg_to_rad(-70.0 - i * 10.0)
			draw_line(at, at + Vector2(cos(angle), sin(angle)) * 26.0, WARN, 2.0)
		_text(at + Vector2(-30, -34), "%.2f kg/s out" % absf(flow), WARN, 11)
		return
	var color := _warmth(float(edge["stream"]["temperature"])) if running else IDLE
	var width := 1.5 + 6.5 * sqrt(absf(flow) / largest_flow) if running else 1.5
	var path := _pipe_path(edge)
	# Drawn on into each node, level with its middle, so the pipe meets the
	# shape drawn over it rather than stopping at the box round it.
	var drawn := path.duplicate()
	if path.size() >= 2:
		drawn[0] = _into(path[0], path[1], places[from])
		drawn[drawn.size() - 1] = _into(path[path.size() - 1], path[path.size() - 2], places[to])
	if hovered.get("edge", -1) == int(edge["id"]):
		draw_polyline(drawn, INK, width + 2.0 * CASING + 4.0)
		for corner in range(1, drawn.size() - 1):
			var half := Vector2.ONE * (width * 0.5 + CASING + 2.0)
			draw_rect(Rect2(drawn[corner] - half, half * 2.0), INK)
	Screen.pipe(self, drawn, color, width, float(dash_travel.get(int(edge["id"]), 0.0)) if running else NAN, CASING)
	# A flange where it joins equipment; a junction is a tee in the pipe itself.
	if path.size() >= 2:
		if _kind(_entry("nodes", from)) != "junction":
			_flange(path[0], path[1], color, width)
		if _kind(_entry("nodes", to)) != "junction":
			_flange(path[path.size() - 1], path[path.size() - 2], color, width)
	# On the middle of the pipe's longest run: always on its drawn line (M85).
	var middle := Route.middle(path)
	var mid: Vector2 = middle[0]
	if running and middle[1] != Vector2.ZERO:
		var along: Vector2 = middle[1] * (1.0 if flow > 0.0 else -1.0)
		var across := Vector2(-along.y, along.x)
		draw_colored_polygon(PackedVector2Array([mid + along * 9.0, mid - along * 5.0 + across * 6.0, mid - along * 5.0 - across * 6.0]), color)
		# The number above a level run, right of an upright one.
		var number := "%.1f" % absf(flow)
		var offset := Vector2(-18, -9) if absf(along.x) > 0.5 else Vector2(8, 4)
		_text(mid + offset, number, DIM, 10)
	if float(edge.get("leak_mass_flow", 0.0)) > 0.0:
		for i in 5:
			var angle := deg_to_rad(60.0 + i * 15.0)
			draw_line(mid, mid + Vector2(cos(angle), sin(angle)) * 22.0, WARN, 2.0)
		_text(mid + Vector2(-26, 38), "leak %.2f kg/s" % float(edge["leak_mass_flow"]), WARN, 11)


## Where a pipe between two drawn nodes runs (M85): its routed corners, at
## right angles, round the other nodes and their names (`_route_pipes`). While
## a node is dragged, its own pipes are a quick sketch until it is let go.
## Empty for a pipe with an end the map hides. Drawing and the mouse both use
## this, so a pipe is found where it is seen.
func _pipe_path(edge: Dictionary) -> PackedVector2Array:
	var from := int(edge["from"])
	var to := int(edge["to"])
	if not places.has(from) or not places.has(to):
		return PackedVector2Array()
	if dragging >= 0 and (from == dragging or to == dragging) or not routes.has(int(edge["id"])):
		return Route.sketch(_box(from), _box(to))
	return routes[int(edge["id"])]


## A pipe's end `port` (on its node's box, the pipe running on to `next`)
## carried back into the node, as far as level with the node's middle `centre`.
func _into(port: Vector2, next: Vector2, centre: Vector2) -> Vector2:
	var out := (next - port).normalized()
	return port - out * maxf(0.0, (port - centre).dot(out))


## Where a pipe joins its node (M85): a flange, a short bar across the pipe
## just outside the box, a little wider than the pipe and outlined dark, so
## each end shows which node it is on and where on it.
func _flange(port: Vector2, next: Vector2, color: Color, width: float) -> void:
	var out := (next - port).normalized()
	if out == Vector2.ZERO:
		return
	var across := Vector2(-out.y, out.x)
	var half_length := width * 0.5 + 4.0
	var at := port + out * 3.0
	var bar := PackedVector2Array([at - across * half_length, at + across * half_length])
	draw_line(bar[0] - across, bar[1] + across, BACKGROUND, 5.0)
	draw_line(bar[0], bar[1], color.lightened(0.35), 3.0)


## The box a drawn node takes on the page [px]: its kind's size round its
## place — but a column or a reactor (M85) runs from above its highest
## neighbour to below its lowest, as a real tower stands taller than what is
## piped to it, so each draw and return can meet it level, in a straight pipe.
func _box(id: int) -> Rect2:
	var node := _entry("nodes", id)
	var extent := _extent(_kind(node))
	var box := Rect2(places[id] - extent, extent * 2.0)
	if not _kind(node) in ["column", "reactor"]:
		return box
	var top := box.position.y
	var bottom := box.end.y
	for edge in snapshot.get("edges", []):
		var other := -1
		if int(edge["from"]) == id:
			other = int(edge["to"])
		elif int(edge["to"]) == id:
			other = int(edge["from"])
		if other >= 0 and other != id and places.has(other):
			top = minf(top, places[other].y - TOWER_OVERHANG)
			bottom = maxf(bottom, places[other].y + TOWER_OVERHANG)
	top = maxf(top, PLACEABLE.position.y)
	bottom = minf(bottom, TOWER_FLOOR)
	return Rect2(box.position.x, top, box.size.x, maxf(bottom - top, extent.y * 2.0))


## How far under a node's place its name is written [px] (its reading 13 px
## under that): under the box, however tall a tower has grown.
func _label_y(node: Dictionary) -> float:
	var type := _kind(node)
	if type in ["column", "reactor"]:
		var id := int(node["id"])
		return _box(id).end.y - places[id].y + 16.0
	return _label_offset(type)


## Every drawn pipe routed (demo/route.gd), the forward ones first, the
## shortest first among them, so a plant's main line gets the straight runs
## and the recycles take the lanes round the outside.
func _route_pipes() -> void:
	routes = {}
	backward = {}
	var boxes := {}
	var labels: Array = []
	for node in snapshot.get("nodes", []):
		var id := int(node["id"])
		if not places.has(id):
			continue
		boxes[id] = _box(id)
		labels.append_array(_label_boxes(node, places[id]))
	var keyed: Array = []
	for edge in snapshot.get("edges", []):
		var from := int(edge["from"])
		var to := int(edge["to"])
		if not boxes.has(from) or not boxes.has(to) or from == to:
			continue
		var a: Vector2 = places[from]
		var b: Vector2 = places[to]
		var back := b.x <= a.x + 1.0
		if back:
			backward[int(edge["id"])] = true
		keyed.append([1 if back else 0, absf(b.x - a.x) + absf(b.y - a.y), int(edge["id"]), from, to])
	keyed.sort()
	var pipes: Array = keyed.map(func(k): return [k[2], k[3], k[4]])
	routes = Route.route(boxes, labels, pipes, ROUTED)


## Where a node's name and its reading are written under it: the boxes a pipe
## keeps off where it can, each [node id, Rect2]. The reading's width is a guess (it changes as the
## plant runs); the name's is measured.
func _label_boxes(node: Dictionary, at: Vector2) -> Array:
	var name := str(node["name"])
	if _kind(node) == "junction" and name.ends_with("__leak_point"):
		return []
	var font := ThemeDB.fallback_font
	var y := at.y + _label_y(node)
	var name_width := font.get_string_size(name.replace("_", " "), HORIZONTAL_ALIGNMENT_LEFT, -1, 11).x
	var reading_width := 100.0
	var id := int(node["id"])
	return [
		[id, Rect2(at.x - name_width * 0.5 - 2.0, y - 11.0, name_width + 4.0, 14.0)],
		[id, Rect2(at.x - reading_width * 0.5, y + 3.0, reading_width, 13.0)],
	]


## How far under a node of `type` its name is written [px]; its reading 13 px
## under that. A tower's depends on how tall it is drawn (`_label_y`).
func _label_offset(type: String) -> float:
	match type:
		"tank", "vessel":
			return 34.0
		"valve", "relief_valve", "check_valve":
			return 24.0
		"junction":
			return 18.0
		"furnace":
			return 38.0
	return 30.0


## Half the width and height a node of `type` is drawn [px]: what the mouse
## finds it by, and the outline drawn round it while it is inspected.
func _extent(type: String) -> Vector2:
	match type:
		"source", "sink", "pump":
			return Vector2(12, 12)
		"tank", "vessel":
			return Vector2(17, 20)
		"valve", "relief_valve", "check_valve":
			return Vector2(12, 9)
		"junction":
			return Vector2(6, 6)
		"furnace":
			return Vector2(22, 22)
		"cooler", "water_cooler", "heat_exchanger", "coil_cell":
			return Vector2(13, 13)
		"column", "reactor":
			return Vector2(18, 60)
	return Vector2(14, 14)


## What is drawn at `point`: a node first (the nearest whose shape holds it),
## then the pipe nearest it within a few pixels of its line; {} for neither.
func _hit(point: Vector2) -> Dictionary:
	var best := {}
	var best_distance := INF
	for node in snapshot.get("nodes", []):
		var id := int(node["id"])
		if not places.has(id):
			continue
		var box := _box(id)
		var distance := point.distance_to(box.get_center())
		if box.grow(0.5).has_point(point) and distance < best_distance:
			best = {"node": id}
			best_distance = distance
	if not best.is_empty():
		return best
	for edge in snapshot.get("edges", []):
		var path := _pipe_path(edge)
		for i in range(1, path.size()):
			var distance := point.distance_to(Geometry2D.get_closest_point_to_segment(point, path[i - 1], path[i]))
			if distance <= 6.0 and distance < best_distance:
				best = {"edge": int(edge["id"])}
				best_distance = distance
	return best


## `--hover=<name>`: the named node's place, or the middle of the named
## pipe's longest run, sent through `_hit` as the mouse's would be.
func _hover_named(wanted: String) -> void:
	for node in snapshot["nodes"]:
		if str(node["name"]) == wanted and places.has(int(node["id"])):
			hovered = _hit(places[int(node["id"])])
			return
	for edge in snapshot["edges"]:
		var path := _pipe_path(edge)
		if str(edge["name"]) == wanted and path.size() >= 2:
			hovered = _hit(Route.middle(path)[0])
			return
	print("map: --hover=%s names no drawn node or pipe" % wanted)


## A node, by its kind. Its name under it, and the one or two numbers that
## kind is watched for.
func _draw_node(node: Dictionary, at: Vector2) -> void:
	var kind: Dictionary = node["kind"]
	var type := _kind(node)
	var label := str(node["name"])
	var reading := "%.2f bar  %.0f °C" % [float(node["pressure_pa"]) / 1.0e5, float(node["temperature_k"]) - 273.15]
	var label_y := _label_y(node)
	# Where a fire's flames stand: over the node (a tower's top, not its middle).
	var top_y := at.y - label_y
	match type:
		"source", "sink":
			draw_circle(at, 11, COOL if type == "source" else BODY)
			draw_arc(at, 11, 0.0, TAU, 32, SHELL, 2.0)
		"tank", "vessel":
			var rect := Rect2(at - Vector2(17, 20), Vector2(34, 40))
			draw_rect(rect, BODY)
			var level := _tank_fraction(node)
			if level > 0.0:
				draw_rect(Rect2(rect.position + Vector2(0, rect.size.y * (1.0 - level)), Vector2(rect.size.x, rect.size.y * level)), _warmth(float(node["temperature_k"])))
			draw_rect(rect, SHELL, false, 2.0)
			if type == "tank":
				reading = "%.0f %%  %.0f °C" % [level * 100.0, float(node["temperature_k"]) - 273.15]
		"pump":
			var on := bool(kind.get("on", true))
			draw_circle(at, 12, GOOD.darkened(0.4) if on else BODY)
			draw_arc(at, 12, 0.0, TAU, 32, GOOD if on else SHELL, 2.0)
			draw_colored_polygon(PackedVector2Array([at + Vector2(7, 0), at + Vector2(-4, -6), at + Vector2(-4, 6)]), INK if on else DIM)
			reading = "on" if on else "OFF"
			if bool(node.get("cavitation", {}).get("cavitating", false)) and on:
				reading = "on, cavitating"
		"valve", "relief_valve", "check_valve":
			var opening := float(kind.get("opening", 1.0))
			var color := SHELL if opening > 0.0 else BAD.darkened(0.3)
			draw_colored_polygon(PackedVector2Array([at, at + Vector2(-11, -8), at + Vector2(-11, 8)]), color)
			draw_colored_polygon(PackedVector2Array([at, at + Vector2(11, -8), at + Vector2(11, 8)]), color)
			reading = "%.0f %% open" % (opening * 100.0) if type == "valve" else type.replace("_", " ")
		"junction":
			draw_circle(at, 3.5, SHELL)
			# A punctureable pipe's midpoint is plumbing, not equipment.
			if label.ends_with("__leak_point"):
				return
		"furnace":
			var rect := Rect2(at - Vector2(22, 22), Vector2(44, 44))
			draw_rect(rect, BODY)
			draw_rect(rect, SHELL, false, 2.0)
			var duty := float(kind.get("duty", 0.0))
			if duty > 0.0:
				for i in 3:
					var x := at.x - 12.0 + i * 12.0
					draw_colored_polygon(PackedVector2Array([Vector2(x - 5, rect.end.y - 2), Vector2(x, rect.end.y - 24), Vector2(x + 5, rect.end.y - 2)]), FLAME)
			reading = "%.1f MW  out %.0f °C" % [duty / 1.0e6, float(node["temperature_k"]) - 273.15]
		"cooler", "water_cooler", "heat_exchanger", "coil_cell":
			draw_circle(at, 13, BODY)
			draw_arc(at, 13, 0.0, TAU, 32, COOL, 2.0)
			draw_line(at + Vector2(-9, 5), at + Vector2(-3, -5), COOL, 2.0)
			draw_line(at + Vector2(-3, -5), at + Vector2(3, 5), COOL, 2.0)
			draw_line(at + Vector2(3, 5), at + Vector2(9, -5), COOL, 2.0)
			if kind.has("duty"):
				reading = "%.1f MW off  %.0f °C" % [float(kind["duty"]) / 1.0e6, float(node["temperature_k"]) - 273.15]
		"column", "reactor":
			var rect := _box(int(node["id"]))
			top_y = rect.position.y - 16.0
			draw_rect(rect, _warmth(float(node["temperature_k"])).darkened(0.35))
			if type == "column":
				# A tray line every ~20 px, however tall it is drawn.
				var trays := maxi(5, roundi(rect.size.y / 20.0) - 1)
				for i in trays:
					var y := rect.position.y + rect.size.y * (i + 1) / (trays + 1.0)
					draw_line(Vector2(rect.position.x + 4, y), Vector2(rect.end.x - 4, y), SHELL, 1.0)
			draw_rect(rect, SHELL, false, 2.5)
		_:
			draw_rect(Rect2(at - Vector2(14, 14), Vector2(28, 28)), BODY)
			draw_rect(Rect2(at - Vector2(14, 14), Vector2(28, 28)), SHELL, false, 2.0)
			reading = type.replace("_", " ")
	if float(node.get("heat_input_w", 0.0)) > 0.0:
		# A fire on the node: the engine's own heat source (CLAUDE.md, damage).
		for i in 3:
			var x := at.x - 14.0 + i * 14.0
			draw_colored_polygon(PackedVector2Array([Vector2(x - 7, top_y + 6), Vector2(x, top_y - 18), Vector2(x + 7, top_y + 6)]), FLAME)
	_centred(at + Vector2(0, label_y), label.replace("_", " "), INK, 11)
	_centred(at + Vector2(0, label_y + 13), reading, DIM, 10)


## A tank's fill, 0–1: its mass over the volume its liquid takes, the density
## blended over the slate by mass fraction (volumes add) — demo/plant.gd's
## rule, every operand a snapshot field. A vessel reports no height: empty.
func _tank_fraction(node: Dictionary) -> float:
	var kind: Dictionary = node["kind"]
	if not (kind.has("mass") and kind.has("area") and kind.has("height") and kind.has("composition")):
		return 0.0
	var fractions: Array = kind["composition"]["mass_fractions"]
	var slate: Array = snapshot["slate"]
	var specific_volume := 0.0
	for i in mini(fractions.size(), slate.size()):
		var density := float(slate[i].get("density_kg_per_m3", 0.0))
		if density > 0.0:
			specific_volume += float(fractions[i]) / density
	var volume := float(kind["area"]) * float(kind["height"])
	if specific_volume <= 0.0 or volume <= 0.0:
		return 0.0
	return clampf(float(kind["mass"]) * specific_volume / volume, 0.0, 1.0)


# ------------------------------------------------------------------ panel

## The panel at the right: what the mouse is over, every number the snapshot
## gives it — or, over nothing, the plant's loops and trips — as many lines as
## fit above the legend and then how many did not.
func _draw_panel(size: Vector2) -> void:
	var left := _panel_rect(size).position.x + 10.0
	var bottom := size.y - 66.0
	draw_rect(_panel_rect(size), BODY.darkened(0.25))
	var lines := _inspect_lines(hovered) if not hovered.is_empty() else []
	if lines.is_empty():
		lines = _overview_lines()
	# The title stays; the wheel scrolls the rest (`panel_scroll`), a line
	# saying how many are above and below where any are hidden.
	var room := int((bottom - PANEL_TOP) / PANEL_LINE)
	var body := lines.size() - 1
	var first := 1
	if body > room:
		first = 1 + clampi(panel_scroll, 0, body - room + 1)
	panel_scroll = first - 1
	var rows: Array = [lines[0]]
	if first > 1:
		rows.append(["… %d above" % (first - 1), "", DIM, 0])
	var space := room - (rows.size() - 1)
	var last := mini(lines.size(), first + space)
	if last < lines.size():
		last -= 1
	rows.append_array(lines.slice(first, last))
	if last < lines.size():
		rows.append(["… %d more (mouse wheel)" % (lines.size() - last), "", DIM, 0])
	var font := ThemeDB.fallback_font
	for i in rows.size():
		var line: Array = rows[i]
		var y := PANEL_TOP + i * PANEL_LINE
		var title := str(line[0])
		var value := str(line[1])
		var color: Color = line[2]
		var x := left + PANEL_INDENT * int(line[3])
		if value == "":
			var heading := 14 if i == 0 else 10
			draw_string(font, Vector2(x, y), title, HORIZONTAL_ALIGNMENT_LEFT, -1, Screen.fitting_size(title, left + PANEL_WIDTH - x, heading, 9), color)
			continue
		var value_width := font.get_string_size(value, HORIZONTAL_ALIGNMENT_LEFT, -1, 11).x
		draw_string(font, Vector2(left + PANEL_WIDTH - value_width, y), value, HORIZONTAL_ALIGNMENT_LEFT, -1, 11, color)
		draw_string(font, Vector2(x, y), _shortened(title, left + PANEL_WIDTH - value_width - 10.0 - x, 10), HORIZONTAL_ALIGNMENT_LEFT, -1, 10, DIM)


## The panel's backing on a page of `size`: at PANEL_X on the base page, at
## the right edge of a wider one.
func _panel_rect(size: Vector2) -> Rect2:
	var left := maxf(PANEL_X, size.x - PANEL_WIDTH - 12.0)
	var bottom := size.y - 66.0
	return Rect2(left - 10.0, PANEL_TOP - 20.0, PANEL_WIDTH + 18.0, bottom - PANEL_TOP + 26.0)


## `text`, its end cut off until it is no wider than `width` at `size`.
func _shortened(text: String, width: float, size: int) -> String:
	var font := ThemeDB.fallback_font
	var cut := text
	while cut.length() > 1 and font.get_string_size(cut, HORIZONTAL_ALIGNMENT_LEFT, -1, size).x > width:
		cut = cut.substr(0, cut.length() - 2) + "…"
	return cut


## The panel's lines for `target` ({"node": id} or {"edge": id}), each
## [name, value, colour, indent], a heading where the value is "". Empty when
## the target is not in this tick's snapshot.
func _inspect_lines(target: Dictionary) -> Array:
	var lines: Array = []
	if target.has("node"):
		var node := _entry("nodes", int(target["node"]))
		if node.is_empty():
			return lines
		lines.append([str(node["name"]).replace("_", " "), "", INK, 0])
		lines.append(_click_line(target))
		var owner := _loop_on(int(node["id"]))
		if not owner.is_empty():
			lines.append(["moved by loop", "%s (%s)" % [_label(owner), str(owner["mode"]).to_upper()], INK, 0])
		lines.append(["kind", _kind(node).replace("_", " "), INK, 0])
		var readings := {}
		for key in node:
			if key not in ["id", "name", "kind"]:
				readings[key] = node[key]
		lines.append(["READINGS", "", DIM, 0])
		_walk(readings, 1, lines)
		lines.append(["SETTINGS AND STATE", "", DIM, 0])
		_walk(node["kind"], 1, lines)
	elif target.has("edge"):
		var edge := _entry("edges", int(target["edge"]))
		if edge.is_empty():
			return lines
		lines.append([str(edge["name"]).replace("_", " "), "", INK, 0])
		lines.append(_click_line(target))
		lines.append(["pipe from", str(_entry("nodes", int(edge["from"])).get("name", "?")).replace("_", " "), INK, 0])
		lines.append(["to", str(_entry("nodes", int(edge["to"])).get("name", "?")).replace("_", " "), INK, 0])
		lines.append(["READINGS", "", DIM, 0])
		# A stream's `pressure` is left out: every pipe reports atmospheric
		# there (DEFERRED F10); the nodes at its ends hold the real ones.
		var stream := {}
		for key in edge["stream"]:
			if key != "pressure":
				stream[key] = edge["stream"][key]
		_walk(stream, 1, lines)
		var rest := {}
		for key in edge:
			if key not in ["id", "name", "from", "to", "stream"]:
				rest[key] = edge[key]
		_walk(rest, 1, lines)
	return lines


## The panel's line saying what a click on `target` does, a heading.
func _click_line(target: Dictionary) -> Array:
	if target.has("edge"):
		var edge := _entry("edges", int(target["edge"]))
		# A pipe the plant lets leak is split at a `__leak_point` junction.
		for end in ["from", "to"]:
			if str(_entry("nodes", int(edge[end])).get("name", "")).ends_with("__leak_point"):
				return ["P+click makes a hole in it; P+click again mends it", "", WARN, 0]
		return ["no hand on this pipe (it has no leak path)", "", DIM, 0]
	var node := _entry("nodes", int(target["node"]))
	if str(node["name"]).ends_with("__leak_point"):
		return ["the middle of a pipe: P+click holes or mends it", "", WARN, 0]
	match _kind(node):
		"valve", "furnace", "cooler":
			return ["click picks it: Up/Down move it, M by hand", "", WARN, 0]
		"pump":
			return ["click starts or stops it", "", WARN, 0]
	return ["no hand on it from the map", "", DIM, 0]


## The panel over nothing: what is picked, then every control loop (target,
## what it last measured, its output) and every trip (what it watches, its
## limit, its state), the snapshot's `controls` and `trips` as
## demo/reactor.gd lists them; the keys that work them in the headings.
func _overview_lines() -> Array:
	var lines: Array = [["the plant's loops and trips", "", INK, 0]]
	lines.append(["point at a node or a pipe to read it", "", DIM, 0])
	var picked := _entry("nodes", int(selected.get("node", -1)))
	if not picked.is_empty():
		lines.append(["picked (Up/Down, M)", str(picked["name"]).replace("_", " "), WARN, 0])
	lines.append(["LOOPS — Tab picks, A auto/manual, Q/Z target", "", DIM, 0])
	var loops: Array = snapshot.get("controls", [])
	if loops.is_empty():
		lines.append(["none on this plant", "", DIM, 1])
	for n in loops.size():
		var loop: Dictionary = loops[n]
		var mine := n == selected_loop % loops.size()
		lines.append([("> " if mine else "") + str(loop["name"]).replace("_", " "), str(loop["mode"]).to_upper(), WARN if mine else INK, 0])
		lines.append(["target", _value_text(loop["setpoint"]), INK, 1])
		lines.append(["measured", _value_text(loop.get("measurement")), INK, 1])
		lines.append(["output", "%s %%" % _number(float(loop["output"]) * 100.0), INK, 1])
		if loop.has("actuates"):
			lines.append(["moves", str(_entry("nodes", int(loop["actuates"])).get("name", "?")).replace("_", " "), INK, 1])
		if loop.has("drives"):
			lines.append(["sets the target of", str(loops[int(loop["drives"])]["name"]).replace("_", " "), INK, 1])
	lines.append(["TRIPS — R resets the tripped", "", DIM, 0])
	var trips: Array = snapshot.get("trips", [])
	if trips.is_empty():
		lines.append(["none on this plant", "", DIM, 1])
	for trip in trips:
		var state: Dictionary = trip["state"]
		var tripped: bool = state["status"] == "tripped"
		var label := "armed"
		if tripped:
			label = "TRIPPED at tick %d%s" % [int(state["at_tick"]), " by hand" if state.get("by_hand", false) else ""]
		lines.append([str(trip["name"]).replace("_", " "), label, BAD if tripped else INK, 0])
		lines.append(["watches", _watched(trip["watches"]), INK, 1])
		lines.append(["trips at", "%s %s" % ["≥" if trip["direction"] == "high" else "≤", _value_text(trip["limit"])], INK, 1])
		lines.append(["measured", _value_text(trip.get("measurement")), INK, 1])
	return lines


## A loop's or a trip's `watches` — {"node": i}, {"pipe": i} or {"coil": i} —
## as the name it indexes.
func _watched(watches: Dictionary) -> String:
	if watches.has("node"):
		return str(_entry("nodes", int(watches["node"])).get("name", "?")).replace("_", " ")
	if watches.has("pipe"):
		return str(_entry("edges", int(watches["pipe"])).get("name", "?")).replace("_", " ")
	if watches.has("coil"):
		return "coil %d" % (int(watches["coil"]) + 1)
	return "?"


## A tagged value (a setpoint, a measurement, a limit) in its own unit; null
## — a loop with nothing to act on yet — is "unknown", never zero.
func _value_text(value) -> String:
	if value == null:
		return "unknown"
	match str(value.get("variable", "")):
		"temperature":
			return "%s °C" % _number(float(value["k"]) - 273.15)
		"level":
			return "%s m" % _number(float(value["m"]))
		"pressure":
			return "%s bar" % _number(float(value["pa"]) / 1.0e5)
		"flow":
			return "%s kg/s" % _number(float(value["kg_per_s"]))
	return str(value)


## This tick's entry in `snapshot[list]` with this id, or {}.
func _entry(list: String, id: int) -> Dictionary:
	for entry in snapshot.get(list, []):
		if int(entry["id"]) == id:
			return entry
	return {}


## Every scalar under `fields`, depth first, set in by `depth`; a dictionary
## or a list of them is a heading with its fields under it. A composition is
## its three largest cuts; a list longer than PANEL_LIST, and anything nested
## deeper than PANEL_DEPTH, are left out; `type` is serde's tag.
func _walk(fields: Dictionary, depth: int, lines: Array) -> void:
	for key in fields:
		if key == "type":
			continue
		var value = fields[key]
		var title := str(key).replace("_", " ")
		if value is Dictionary:
			if value.has("mass_fractions"):
				_composition(value["mass_fractions"], title, depth, lines)
			elif depth < PANEL_DEPTH:
				lines.append([title, "", DIM, depth])
				_walk(value, depth + 1, lines)
		elif value is Array:
			if value.is_empty() or value.size() > PANEL_LIST:
				continue
			if value[0] is Dictionary:
				if depth < PANEL_DEPTH:
					for i in value.size():
						if value[i] is Dictionary:
							lines.append(["%s %d" % [title, i + 1], "", DIM, depth])
							_walk(value[i], depth + 1, lines)
			else:
				var parts := PackedStringArray()
				for item in value:
					parts.append(_value("", item))
				lines.append([title, ", ".join(parts), INK, depth])
		else:
			lines.append([_plain_name(str(key)), _value(str(key), value), INK, depth])


## A field's name without the unit it ends in when the value shows that unit:
## `temperature_k` reads "temperature", its value "… °C".
func _plain_name(key: String) -> String:
	for suffix in ["_kg_s", "_mol_s", "_pa", "_k", "_w", "_s", "_m"]:
		if key.ends_with(suffix) and _unit(key) != "":
			return key.trim_suffix(suffix).replace("_", " ")
	return key.replace("_", " ")


## A composition's three largest cuts by mass, named from the snapshot's slate.
func _composition(fractions: Array, title: String, depth: int, lines: Array) -> void:
	var slate: Array = snapshot.get("slate", [])
	var ranked: Array = []
	for i in fractions.size():
		if float(fractions[i]) > 0.0:
			ranked.append([float(fractions[i]), str(slate[i].get("name", "cut %d" % (i + 1))) if i < slate.size() else "cut %d" % (i + 1)])
	ranked.sort()
	ranked.reverse()
	if ranked.is_empty():
		lines.append([title, "nothing", INK, depth])
		return
	for i in mini(3, ranked.size()):
		lines.append([title if i == 0 else "", "%s  %.1f %%" % [str(ranked[i][1]).replace("_", " "), ranked[i][0] * 100.0], INK, depth])


## A field's value for reading, in the unit its name gives it (`_unit`), or as
## it came when its name gives none.
func _value(key: String, value) -> String:
	if value == null:
		return "none"
	if value is bool:
		return "yes" if value else "no"
	if value is String:
		return value.replace("_", " ")
	var x := float(value)
	match _unit(key):
		"bar":
			return "%s bar" % _number(x / 1.0e5)
		"°C":
			return "%s °C" % _number(x - 273.15)
		"MW":
			return "%s MW" % _number(x / 1.0e6)
		"kg/s":
			return "%s kg/s" % _number(x)
		"%":
			return "%s %%" % _number(x * 100.0)
		"mol/s":
			return "%s mol/s" % _number(x)
		"s":
			return "%s s" % _number(x)
		"m":
			return "%s m" % _number(x)
	return _number(x)


## The unit a field's name gives it; "" for none known. SI in, so the
## conversions in `_value` are the only arithmetic.
func _unit(key: String) -> String:
	if key.ends_with("_pa") or key in PASCAL_FIELDS:
		return "bar"
	if key.ends_with("_k") or key in KELVIN_FIELDS:
		return "°C"
	if key.ends_with("_w") or key in WATT_FIELDS:
		return "MW"
	if key.ends_with("_kg_s") or key in FLOW_FIELDS:
		return "kg/s"
	if key.ends_with("_fraction") or key in PERCENT_FIELDS:
		return "%"
	if key.ends_with("_mol_s"):
		return "mol/s"
	if key.ends_with("_s") or key in SECOND_FIELDS:
		return "s"
	if key.ends_with("_m") or key in METRE_FIELDS:
		return "m"
	return ""


## A number in a few characters: whole numbers whole, very large or very
## small ones as a mantissa and a power of ten (a valve's Cv is ~1e-5).
func _number(x: float) -> String:
	if x == 0.0:
		return "0"
	var size := absf(x)
	if size >= 1.0e6 or size < 1.0e-3:
		var power := floori(log(size) / log(10.0))
		return "%.3fe%d" % [x / pow(10.0, power), power]
	if x == roundf(x):
		return "%d" % int(x)
	if size >= 100.0:
		return "%.1f" % x
	if size >= 1.0:
		return "%.2f" % x
	return "%.4f" % x


func _centred(at: Vector2, text: String, color: Color, size: int) -> void:
	var width := ThemeDB.fallback_font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, size).x
	draw_string(ThemeDB.fallback_font, at - Vector2(width * 0.5, 0), text, HORIZONTAL_ALIGNMENT_LEFT, -1, size, color)


func _text(at: Vector2, text: String, color: Color, size: int = 16) -> void:
	Screen.text(self, at, text, color, size)


func _halt(reason: String) -> void:
	halted = reason
	print("map: HALTED — %s" % reason)
	queue_redraw()


## Enter (M84): the screen that draws the plant running up close, on the same
## plant as it stands — or, when no screen but this one draws it, says so.
func _closer() -> void:
	var scene := Screen.screen_for(running.scenario)
	if scene == "":
		_say("only the map draws %s: no screen shows it closer" % plants[plant_index], WARN)
		print("map: Enter — only the map draws %s" % plants[plant_index])
		queue_redraw()
		return
	Screen.open_running(get_tree(), scene)
