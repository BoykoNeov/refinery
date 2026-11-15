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
# it WATCHES. No keys reach the plant yet, the layout is computed rather than
# drawn by a person, and a column or a reactor is one box — their insides are
# what the hand-drawn screens are for. The mouse INSPECTS (M78, slice 1 of the
# plan): over a node or a pipe, the panel at the right lists every number the
# snapshot gives it, walked generically, so a field the engine adds appears
# without this file changing.
#
# Same rule as every screen: nothing physical is computed here. A pipe's
# width is its reported flow, its colour its reported temperature; a tank's
# fill is the level the plant.gd demo derives from snapshot fields only. The
# panel converts units for reading (Pa to bar, K to °C, W to MW) and nothing
# more; a field whose unit this file does not know is shown as it came.
#
# Interactive:  godot --path . res://demo/map.tscn      (or the menu's last row)
#               Left/Right plant   Space pause   [ ] speed   Esc menu   mouse: inspect (wheel scrolls)
# Recorded:     godot --headless --path . res://demo/map.tscn -- --auto
#               ...loads every plant in turn, runs it 200 ticks, prints one
#               line each and a count: a smoke test of the map on all of them
#               (and builds the panel of every node and pipe, so a field shape
#               the panel cannot read shows up as a script error).
# Screenshot:   add --plant=<file stem> --shot=<file.png> to a windowed run;
#               --hover=<node or pipe name> points at it as the mouse would,
#               --scroll=<lines> scrolls its panel as the wheel would.
extends Node2D

const Screen := preload("res://demo/screen.gd")

const SCENARIOS := "res://scenarios"
const MAX_TICKS_PER_FRAME := 64
const AUTO_TICKS := 200

const BACKGROUND := Color(0.09, 0.10, 0.12)
const SHELL := Color(0.42, 0.45, 0.50)
const BODY := Color(0.15, 0.17, 0.20)
const INK := Color(0.88, 0.90, 0.93)
const DIM := Color(0.60, 0.63, 0.68)
const COOL := Color(0.25, 0.50, 0.85)
const HOT := Color(0.90, 0.30, 0.15)
const FLAME := Color(1.0, 0.55, 0.10)
const GOOD := Color(0.35, 0.80, 0.45)
const BAD := Color(0.95, 0.30, 0.25)
const WARN := Color(0.95, 0.78, 0.30)
const IDLE := Color(0.30, 0.32, 0.36)

## The temperatures a pipe's colour runs between [K]: 20 °C to 450 °C.
const COLD_K := 293.15
const HOT_K := 723.15

## More nodes than this in one column of the layout and it is drawn as two.
const CROWDED := 5

## The page the plant is laid out on, inside the headings and the legend, left
## of the panel: a node's name runs ~55 px either side of it, so the right-most
## column of nodes stops that far short of the panel's backing (PANEL_X - 10).
## A crowded right-most column steps left, not right (`_lay_out`).
const AREA := Rect2(70, 110, 735, 440)

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

@onready var sim: RefinerySim = $Sim

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
## The largest flow on the map this tick [kg/s]: pipe widths are shares of it.
var largest_flow := 1.0
## What the mouse is over: {"node": id} or {"edge": id}, or {} for nothing. An
## id, never the entry itself: the snapshot is replaced every tick.
var hovered: Dictionary = {}
## `--hover=<name>`: what to point at once the plant is laid out; the mouse is
## then ignored, so a screenshot shows what was asked.
var hover_name := ""
## `--scroll=<lines>`: the panel scrolled that far, as the wheel would.
var hover_scroll := 0
## How many of the panel's lines the mouse wheel has scrolled past.
var panel_scroll := 0


func _ready() -> void:
	Screen.fit(get_window())
	for file in DirAccess.get_files_at(SCENARIOS):
		if file.ends_with(".toml"):
			plants.append(file.trim_suffix(".toml"))
	plants.sort()
	var asked := Screen.chosen_plant(get_tree(), "")
	for arg in OS.get_cmdline_user_args():
		if arg == "--auto":
			auto_run = true
		elif arg.begins_with("--plant="):
			asked = arg.trim_prefix("--plant=")
		elif arg.begins_with("--shot="):
			shot_path = arg.trim_prefix("--shot=")
		elif arg.begins_with("--hover="):
			hover_name = arg.trim_prefix("--hover=")
		elif arg.begins_with("--scroll="):
			hover_scroll = int(arg.trim_prefix("--scroll="))
	if plants.is_empty():
		_halt("no plant files in %s" % SCENARIOS)
		return
	if asked != "" and plants.has(asked):
		plant_index = plants.find(asked)
	if auto_run:
		_run_all()
		return
	_load(plant_index)


func _load(index: int) -> void:
	plant_index = posmod(index, plants.size())
	halted = ""
	snapshot = {}
	places = {}
	hovered = {}
	panel_scroll = 0
	var err = JSON.parse_string(sim.load_scenario("%s/%s.toml" % [SCENARIOS, plants[plant_index]]))
	if err != null:
		_halt("load failed: %s" % err["message"])
		return
	if not _step():
		return
	_lay_out()
	if hover_name != "":
		_hover_named(hover_name)
		panel_scroll = hover_scroll
	queue_redraw()


## One engine tick. False when the plant stopped.
func _step() -> bool:
	var err = JSON.parse_string(sim.tick())
	if err != null:
		_halt("tick %d: %s" % [sim.tick_index() + 1, err["message"]])
		return false
	snapshot = JSON.parse_string(sim.snapshot_json())
	return true


func _physics_process(_delta: float) -> void:
	if auto_run or halted != "" or paused or snapshot.is_empty():
		return
	for i in ticks_per_frame:
		if not _step():
			return
	queue_redraw()
	if shot_path != "" and sim.tick_index() >= AUTO_TICKS:
		var path := shot_path
		shot_path = ""
		_shoot(path)


func _shoot(path: String) -> void:
	paused = true
	queue_redraw()
	await RenderingServer.frame_post_draw
	var image := get_viewport().get_texture().get_image()
	if image != null:
		image.save_png(path)
		print("map: saved %s" % path)
	get_tree().quit()


## `--auto`: every plant loaded, laid out and run, one line each.
func _run_all() -> void:
	var failed := 0
	for i in plants.size():
		_load(i)
		var ticks := 1
		while halted == "" and ticks < AUTO_TICKS and _step():
			ticks += 1
		if halted != "":
			failed += 1
		if not snapshot.is_empty():
			# Every node's and pipe's panel, built and thrown away: a field
			# shape the walker cannot read fails here, on every plant.
			for node in snapshot["nodes"]:
				_inspect_lines({"node": int(node["id"])})
			for edge in snapshot["edges"]:
				_inspect_lines({"edge": int(edge["id"])})
		print("map: %-36s %3d nodes drawn, %3d pipes, %s" % [plants[i], places.size(), snapshot.get("edges", []).size(), "ok" if halted == "" else halted])
	print("map: %d plants, %d stopped" % [plants.size(), failed])
	get_tree().quit(1 if failed > 0 else 0)


func _input(event: InputEvent) -> void:
	if event is InputEventMouseMotion and hover_name == "" and not places.is_empty():
		# The local position, not the event's: the page is stretched to the
		# window, and `places` are page coordinates.
		var over := _hit(get_local_mouse_position())
		if over != hovered:
			hovered = over
			panel_scroll = 0
			queue_redraw()
		return
	if event is InputEventMouseButton and event.pressed and not hovered.is_empty():
		# The wheel scrolls the panel: a column reports more than fits.
		if event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			panel_scroll += 3
		elif event.button_index == MOUSE_BUTTON_WHEEL_UP:
			panel_scroll = maxi(0, panel_scroll - 3)
		queue_redraw()
		return
	if not (event is InputEventKey and event.pressed and not event.echo):
		return
	match event.keycode:
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
	queue_redraw()


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
				x += minf(96.0, step * 0.45) * (-1.0 if d == deepest and deepest > 0 else 1.0)
			# Neighbouring columns sit half a step apart in height, so a
			# straight run of pipe does not hide the names under it.
			var stagger := 14.0 if d % 2 == 1 and keyed.size() == 1 else 0.0
			places[id] = Vector2(x, AREA.position.y + AREA.size.y * fraction + stagger)


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
		"plant %d of %d    t = %.1f s    %d tick(s) per frame%s" % [
			plant_index + 1, plants.size(), float(snapshot.get("sim_time", 0.0)), ticks_per_frame, "    PAUSED" if paused else ""
		],
		DIM
	)
	if halted != "":
		draw_multiline_string(ThemeDB.fallback_font, Vector2(30, 86), "HALTED — %s" % halted, HORIZONTAL_ALIGNMENT_LEFT, size.x - 60, 14, 4, BAD)
	if snapshot.is_empty():
		_draw_legend(size)
		return

	largest_flow = 1.0e-9
	for edge in snapshot["edges"]:
		largest_flow = maxf(largest_flow, absf(float(edge["stream"]["mass_flow"])))
	for edge in snapshot["edges"]:
		_draw_pipe(edge)
	for node in snapshot["nodes"]:
		if places.has(int(node["id"])):
			_draw_node(node, places[int(node["id"])])
			if hovered.get("node", -1) == int(node["id"]):
				var extent := _extent(_kind(node)) + Vector2(5, 5)
				draw_rect(Rect2(places[int(node["id"])] - extent, extent * 2.0), INK, false, 1.5)
	_draw_panel(size)
	_draw_legend(size)


func _draw_legend(size: Vector2) -> void:
	var y := size.y - 44.0
	_text(Vector2(30, y), "pipes: width = flow, colour = temperature", DIM, 13)
	for i in 60:
		draw_rect(Rect2(Vector2(330 + i * 3.0, y - 10), Vector2(3, 10)), COOL.lerp(HOT, i / 59.0))
	_text(Vector2(300, y), "20", DIM, 12)
	_text(Vector2(516, y), "450 °C", DIM, 12)
	_text(Vector2(30, size.y - 20.0), "Left/Right plant   Space pause   [ ] speed   F11 full screen   Esc menu   mouse: inspect, wheel scrolls   (this map watches: the plant's keys are on its own screen)", DIM, 13)


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
	var a: Vector2 = places[from]
	var b: Vector2 = places[to]
	var color := _warmth(float(edge["stream"]["temperature"])) if running else IDLE
	var width := 1.5 + 6.5 * sqrt(absf(flow) / largest_flow) if running else 1.5
	var path := _pipe_path(edge)
	if hovered.get("edge", -1) == int(edge["id"]):
		draw_polyline(path, INK, width + 6.0)
	draw_polyline(path, color, width)
	if running:
		var index := path.size() / 2
		var start: Vector2 = path[index - 1]
		var end: Vector2 = path[index]
		var mid := (start + end) * 0.5
		var along := (end - start).normalized() * (1.0 if flow > 0.0 else -1.0)
		var across := Vector2(-along.y, along.x)
		draw_colored_polygon(PackedVector2Array([mid + along * 9.0, mid - along * 5.0 + across * 6.0, mid - along * 5.0 - across * 6.0]), color)
		_text(mid + Vector2(-18, -9), "%.1f" % absf(flow), DIM, 10)
	if float(edge.get("leak_mass_flow", 0.0)) > 0.0:
		var mid := (a + b) * 0.5
		for i in 5:
			var angle := deg_to_rad(60.0 + i * 15.0)
			draw_line(mid, mid + Vector2(cos(angle), sin(angle)) * 22.0, WARN, 2.0)
		_text(mid + Vector2(-26, 38), "leak %.2f kg/s" % float(edge["leak_mass_flow"]), WARN, 11)


## Where a pipe between two drawn nodes runs: straight from its upstream node
## to its downstream one, or — back to a node at or left of where it starts (a
## recycle, a reflux, a pumparound's return) — round underneath, not through.
## Empty for a pipe with an end the map hides. Drawing and the mouse both use
## this, so a pipe is found where it is seen.
func _pipe_path(edge: Dictionary) -> PackedVector2Array:
	var from := int(edge["from"])
	var to := int(edge["to"])
	if not places.has(from) or not places.has(to):
		return PackedVector2Array()
	var a: Vector2 = places[from]
	var b: Vector2 = places[to]
	if b.x <= a.x + 1.0:
		var drop := maxf(a.y, b.y) + 46.0 + 5.0 * (int(edge["id"]) % 5)
		return PackedVector2Array([a, Vector2(a.x, drop), Vector2(b.x, drop), b])
	return PackedVector2Array([a, b])


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
		var offset: Vector2 = (point - places[id]).abs()
		var extent := _extent(_kind(node))
		if offset.x <= extent.x and offset.y <= extent.y and offset.length() < best_distance:
			best = {"node": id}
			best_distance = offset.length()
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


## `--hover=<name>`: the named node's place, or the middle of the named pipe's
## middle run, sent through `_hit` as the mouse's would be.
func _hover_named(wanted: String) -> void:
	for node in snapshot["nodes"]:
		if str(node["name"]) == wanted and places.has(int(node["id"])):
			hovered = _hit(places[int(node["id"])])
			return
	for edge in snapshot["edges"]:
		var path := _pipe_path(edge)
		if str(edge["name"]) == wanted and path.size() >= 2:
			var index := path.size() / 2
			hovered = _hit((path[index - 1] + path[index]) * 0.5)
			return
	print("map: --hover=%s names no drawn node or pipe" % wanted)


## A node, by its kind. Its name under it, and the one or two numbers that
## kind is watched for.
func _draw_node(node: Dictionary, at: Vector2) -> void:
	var kind: Dictionary = node["kind"]
	var type := _kind(node)
	var label := str(node["name"])
	var reading := "%.2f bar  %.0f °C" % [float(node["pressure_pa"]) / 1.0e5, float(node["temperature_k"]) - 273.15]
	var label_y := 30.0
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
			label_y = 34.0
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
			label_y = 24.0
		"junction":
			draw_circle(at, 3.5, SHELL)
			# A punctureable pipe's midpoint is plumbing, not equipment.
			if label.ends_with("__leak_point"):
				return
			label_y = 18.0
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
			label_y = 38.0
		"cooler", "water_cooler", "heat_exchanger", "coil_cell":
			draw_circle(at, 13, BODY)
			draw_arc(at, 13, 0.0, TAU, 32, COOL, 2.0)
			draw_line(at + Vector2(-9, 5), at + Vector2(-3, -5), COOL, 2.0)
			draw_line(at + Vector2(-3, -5), at + Vector2(3, 5), COOL, 2.0)
			draw_line(at + Vector2(3, 5), at + Vector2(9, -5), COOL, 2.0)
			if kind.has("duty"):
				reading = "%.1f MW off  %.0f °C" % [float(kind["duty"]) / 1.0e6, float(node["temperature_k"]) - 273.15]
		"column", "reactor":
			var rect := Rect2(at - Vector2(18, 60), Vector2(36, 120))
			draw_rect(rect, _warmth(float(node["temperature_k"])).darkened(0.35))
			if type == "column":
				for i in 5:
					var y := rect.position.y + rect.size.y * (i + 1) / 6.0
					draw_line(Vector2(rect.position.x + 4, y), Vector2(rect.end.x - 4, y), SHELL, 1.0)
			draw_rect(rect, SHELL, false, 2.5)
			label_y = 76.0
		_:
			draw_rect(Rect2(at - Vector2(14, 14), Vector2(28, 28)), BODY)
			draw_rect(Rect2(at - Vector2(14, 14), Vector2(28, 28)), SHELL, false, 2.0)
			reading = type.replace("_", " ")
	if float(node.get("heat_input_w", 0.0)) > 0.0:
		# A fire on the node: the engine's own heat source (CLAUDE.md, damage).
		for i in 3:
			var x := at.x - 14.0 + i * 14.0
			draw_colored_polygon(PackedVector2Array([Vector2(x - 7, at.y - label_y + 6), Vector2(x, at.y - label_y - 18), Vector2(x + 7, at.y - label_y + 6)]), FLAME)
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
## gives it, as many lines as fit above the legend and then how many did not.
func _draw_panel(size: Vector2) -> void:
	var left := maxf(PANEL_X, size.x - PANEL_WIDTH - 12.0)
	var bottom := size.y - 66.0
	draw_rect(Rect2(left - 10.0, PANEL_TOP - 20.0, PANEL_WIDTH + 18.0, bottom - PANEL_TOP + 26.0), BODY.darkened(0.25))
	var lines := _inspect_lines(hovered) if not hovered.is_empty() else []
	if lines.is_empty():
		var hint := "Point at a node or a pipe to read every number the plant reports for it."
		draw_multiline_string(ThemeDB.fallback_font, Vector2(left, PANEL_TOP), hint, HORIZONTAL_ALIGNMENT_LEFT, PANEL_WIDTH, 12, 3, DIM)
		return
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
		lines.append(["pipe from", str(_entry("nodes", int(edge["from"])).get("name", "?")).replace("_", " "), INK, 0])
		lines.append(["to", str(_entry("nodes", int(edge["to"])).get("name", "?")).replace("_", " "), INK, 0])
		lines.append(["READINGS", "", DIM, 0])
		_walk(edge["stream"], 1, lines)
		var rest := {}
		for key in edge:
			if key not in ["id", "name", "from", "to", "stream"]:
				rest[key] = edge[key]
		_walk(rest, 1, lines)
	return lines


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
	var fitted := Screen.fitting_size(text, get_viewport_rect().size.x - at.x - 8.0, size)
	draw_string(ThemeDB.fallback_font, at, text, HORIZONTAL_ALIGNMENT_LEFT, -1, fitted, color)


func _halt(reason: String) -> void:
	halted = reason
	print("map: HALTED — %s" % reason)
	queue_redraw()
