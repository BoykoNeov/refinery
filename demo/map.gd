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
# what the hand-drawn screens are for.
#
# Same rule as every screen: nothing physical is computed here. A pipe's
# width is its reported flow, its colour its reported temperature; a tank's
# fill is the level the plant.gd demo derives from snapshot fields only.
#
# Interactive:  godot --path . res://demo/map.tscn      (or the menu's last row)
#               Left/Right plant   Space pause   [ ] speed   Esc menu
# Recorded:     godot --headless --path . res://demo/map.tscn -- --auto
#               ...loads every plant in turn, runs it 200 ticks, prints one
#               line each and a count: a smoke test of the map on all of them.
# Screenshot:   add --plant=<file stem> --shot=<file.png> to a windowed run.
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

## The page the plant is laid out on, inside the headings and the legend.
const AREA := Rect2(70, 110, 1012, 440)

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
	var err = JSON.parse_string(sim.load_scenario("%s/%s.toml" % [SCENARIOS, plants[plant_index]]))
	if err != null:
		_halt("load failed: %s" % err["message"])
		return
	if not _step():
		return
	_lay_out()
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
		print("map: %-36s %3d nodes drawn, %3d pipes, %s" % [plants[i], places.size(), snapshot.get("edges", []).size(), "ok" if halted == "" else halted])
	print("map: %d plants, %d stopped" % [plants.size(), failed])
	get_tree().quit(1 if failed > 0 else 0)


func _input(event: InputEvent) -> void:
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
				x += minf(96.0, step * 0.45)
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
	_draw_legend(size)


func _draw_legend(size: Vector2) -> void:
	var y := size.y - 44.0
	_text(Vector2(30, y), "pipes: width = flow, colour = temperature", DIM, 13)
	for i in 60:
		draw_rect(Rect2(Vector2(330 + i * 3.0, y - 10), Vector2(3, 10)), COOL.lerp(HOT, i / 59.0))
	_text(Vector2(300, y), "20", DIM, 12)
	_text(Vector2(516, y), "450 °C", DIM, 12)
	_text(Vector2(30, size.y - 20.0), "Left/Right plant   Space pause   [ ] speed   F11 full screen   Esc menu   (this map watches: the plant's keys are on its own screen)", DIM, 13)


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
	var path := PackedVector2Array([a, b])
	# A pipe back to a node at or left of where it starts (a recycle, a
	# reflux, a pumparound's return) goes round underneath, not through.
	if b.x <= a.x + 1.0:
		var drop := maxf(a.y, b.y) + 46.0 + 5.0 * (int(edge["id"]) % 5)
		path = PackedVector2Array([a, Vector2(a.x, drop), Vector2(b.x, drop), b])
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
