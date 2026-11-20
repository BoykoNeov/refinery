# What every screen shares that is not a plant: the way in from the menu, the
# way back to it, the window, and text that stays inside its space.
#
# Deliberately NOT a `class_name`: a global class is found through the cache
# the editor writes (`.godot/global_script_class_cache.cfg`), and a clone that
# has only ever run `godot --path .` would meet an unknown identifier. Each
# screen preloads this file instead:
#
#   const Screen := preload("res://demo/screen.gd")
#
# Nothing here ticks the simulator (demo/running.gd holds the one running,
# M84), and nothing here is physical.
extends RefCounted

const MENU := "res://demo/menu.tscn"

## The size every screen but the reactor's is laid out for. project.godot's
## stretch settings scale this to whatever the window is, so a screen draws at
## these coordinates on any monitor.
const BASE_SIZE := Vector2i(1152, 648)

const MAP := "res://demo/map.tscn"

## Where the menu leaves the plant it was asked for, for the screen it opens.
const PLANT_META := "refinery_plant"
## Where the menu leaves a save to start from instead (M77).
const SAVE_META := "refinery_save"
## Set by Enter (M84): the screen opened takes over the plant running.
const RUNNING_META := "refinery_running"
## Set by Esc: the scene the menu was opened from, so its "carry on" row for
## that screen is the one chosen and Enter goes straight back.
const FROM_META := "refinery_from"

## The screens, in the order a newcomer should meet them: the newest and
## richest first (the menu lists them so). `script` is read for its PLANTS —
## each plant's `scenario` is how Enter on the map finds the screen that draws
## the plant running; a screen with no table names its one `row`, and its one
## `scenario` when it draws one.
const SCREENS := [
	{
		"name": "Distillation column",
		"about": "a crude unit: furnace, column, side products — from a simple column to one that holds liquid",
		"scene": "res://demo/column.tscn",
		"script": "res://demo/column.gd",
	},
	{
		"name": "Reactor",
		"about": "a hydrocracker bed that makes its own heat: runaways, quenches, a wall that fails",
		"scene": "res://demo/reactor.tscn",
		"script": "res://demo/reactor.gd",
	},
	{
		"name": "Pump",
		"about": "a pump starved at its suction: cavitation, boiling, a gas lock to vent",
		"scene": "res://demo/pump.tscn",
		"script": "res://demo/pump.gd",
	},
	{
		"name": "Furnace",
		"about": "a fired heater and its protection: trips, burst tubes, who relights it",
		"scene": "res://demo/furnace.tscn",
		"script": "res://demo/furnace.gd",
	},
	{
		"name": "Leaking line",
		"about": "the first demo: puncture a pipe, light a fire, repair both",
		"scene": "res://demo/plant.tscn",
		"script": "",
		"row": "A pump, a valve and a line between two tanks",
		"scenario": "res://scenarios/leaking_line.toml",
	},
	{
		"name": "Plant map",
		"about": "any of the shipped plants, drawn from its own data — click its valves, pumps, furnaces and pipes",
		"scene": MAP,
		"script": "",
		"row": "The plant running, or any plant in scenarios/ (Left/Right)",
	},
]

## The quick save: F5 writes it, F9 and the menu's Continue read it (M77).
## `user://` is Godot's folder for this game under the user's profile.
const QUICK_SAVE := "user://saves/quick.json"

## Every screen's palette (M82). A screen names the ones it draws with, under
## its own name where it has one: `const INK := Screen.INK`,
## `const MARK := Screen.WARN`.
const BACKGROUND := Color(0.09, 0.10, 0.12)
const SHELL := Color(0.42, 0.45, 0.50)
const PIPE := Color(0.55, 0.58, 0.62)
const LIQUID := Color(0.20, 0.45, 0.75)
const COOL := Color(0.25, 0.50, 0.85)
const HOT := Color(0.90, 0.30, 0.15)
const FLAME := Color(1.0, 0.55, 0.10)
const INK := Color(0.88, 0.90, 0.93)
const DIM := Color(0.60, 0.63, 0.68)
const GRID := Color(0.20, 0.22, 0.25)
const GOOD := Color(0.35, 0.80, 0.45)
const BAD := Color(0.95, 0.30, 0.25)
## A mark, a warning, a fired trip.
const WARN := Color(0.95, 0.78, 0.30)
## Behind a chart, a shade off the background.
const CHART_BACKGROUND := Color(0.11, 0.12, 0.14)

## A pipe's dashes (M83, `pipe`): drawn this long [px] every this far along
## it, moving this many px a second on a pipe carrying the screen's largest
## flow, and at least the slow speed on any pipe that flows at all.
const DASH_LENGTH := 7.0
const DASH_PERIOD := 22.0
const DASH_FAST := 60.0
const DASH_SLOW := 10.0


## The plant the menu asked for, or `otherwise`. Read once: switching plants
## inside the screen, then coming back through the menu, starts clean.
static func chosen_plant(tree: SceneTree, otherwise: String) -> String:
	if not tree.root.has_meta(PLANT_META):
		return otherwise
	var key := str(tree.root.get_meta(PLANT_META))
	tree.root.remove_meta(PLANT_META)
	return key if key != "" else otherwise


## Open `scene` on `plant` ("" for the screen's own default).
static func open(tree: SceneTree, scene: String, plant: String) -> void:
	tree.root.set_meta(PLANT_META, plant)
	tree.change_scene_to_file(scene)


## Open `scene` on the save at `path` (M77): the screen loads it in place of a
## plant.
static func open_save(tree: SceneTree, scene: String, path: String) -> void:
	tree.root.set_meta(SAVE_META, path)
	tree.change_scene_to_file(scene)


## The save the menu asked a screen to start from, or "". Read once.
static func chosen_save(tree: SceneTree) -> String:
	if not tree.root.has_meta(SAVE_META):
		return ""
	var path := str(tree.root.get_meta(SAVE_META))
	tree.root.remove_meta(SAVE_META)
	return path


## Write a save's text to `path`, making its folder: "" when written, else
## why not. Written beside it first and then moved over it, so a failed write
## never leaves half a save where the last whole one was.
static func write_save(path: String, text: String) -> String:
	var made := DirAccess.make_dir_recursive_absolute(path.get_base_dir())
	if made != OK:
		return "could not make %s (error %d)" % [path.get_base_dir(), made]
	var partial := path + ".partial"
	var file := FileAccess.open(partial, FileAccess.WRITE)
	if file == null:
		return "could not write %s (error %d)" % [partial, FileAccess.get_open_error()]
	file.store_string(text)
	file.close()
	var moved := DirAccess.rename_absolute(partial, path)
	if moved != OK:
		return "could not move %s over %s (error %d)" % [partial, path, moved]
	return ""


## A save's text and what its screen noted in it, or {} when there is no save
## at `path` or it is not one a screen wrote: {text, tick, note}.
static func read_save(path: String) -> Dictionary:
	if not FileAccess.file_exists(path):
		return {}
	var text := FileAccess.get_file_as_string(path)
	var save = JSON.parse_string(text)
	if not (save is Dictionary) or not (save.get("frontend") is Dictionary):
		return {}
	return {"text": text, "tick": int(save.get("tick", 0)), "note": save["frontend"]}


## Esc: the menu, the plant running held still behind it. The menu's first
## rows carry on with it; every row naming a plant starts that plant over.
static func to_menu(tree: SceneTree) -> void:
	if tree.current_scene != null:
		tree.root.set_meta(FROM_META, tree.current_scene.scene_file_path)
	tree.change_scene_to_file(MENU)


## The scene Esc left for the menu, or "". Read once.
static func chosen_from(tree: SceneTree) -> String:
	if not tree.root.has_meta(FROM_META):
		return ""
	var path := str(tree.root.get_meta(FROM_META))
	tree.root.remove_meta(FROM_META)
	return path


## The one running plant (M84, demo/running.gd): its `sim`, its `scenario`.
static func running(tree: SceneTree) -> Node:
	return tree.root.get_node("Running")


## Open `scene` on the plant running, as it stands (Enter, M84).
static func open_running(tree: SceneTree, scene: String) -> void:
	tree.root.set_meta(RUNNING_META, true)
	tree.change_scene_to_file(scene)


## Whether the screen was opened to take over the plant running. Read once.
static func chosen_running(tree: SceneTree) -> bool:
	if not tree.root.has_meta(RUNNING_META):
		return false
	tree.root.remove_meta(RUNNING_META)
	return true


## A screen's PLANTS table, key → its dictionary, in the file's order; {} for
## a screen with none.
static func plants_of(script_path: String) -> Dictionary:
	if script_path == "":
		return {}
	var script: Script = load(script_path)
	if script == null:
		return {}
	return script.get_script_constant_map().get("PLANTS", {})


## The key in `plants` (a screen's PLANTS) whose file is `scenario`, or "".
## The file itself, never the names in it: the crude units share their names.
static func key_for(plants: Dictionary, scenario: String) -> String:
	for key in plants:
		if str(plants[key].get("scenario", "")) == scenario:
			return key
	return ""


## The scene that draws the plant in `scenario` up close, or "" when only the
## map does.
static func screen_for(scenario: String) -> String:
	for screen in SCREENS:
		if screen.get("scenario", "") == scenario and scenario != "":
			return screen["scene"]
		if key_for(plants_of(screen["script"]), scenario) != "":
			return screen["scene"]
	return ""


## `--enter=<tick>,<tick>,…` (M84): a screen presses Enter itself when the
## plant reaches one of these ticks after the screen took it — the hand-over
## between the map and a unit's screen, run without a person.
static func enter_due(started_tick: int, tick: int) -> bool:
	return _due("--enter=", started_tick, tick)


## `--escape=<tick>,<tick>,…`: a screen presses Esc itself at these ticks, as
## `enter_due` presses Enter; the menu it opens then presses Enter on the row
## chosen for it (carry on, on the screen just left) — or, with `--shot=`,
## saves its picture and quits. The round trip through the menu, run without
## a person.
static func escape_due(started_tick: int, tick: int) -> bool:
	return _due("--escape=", started_tick, tick)


static func _due(option: String, started_tick: int, tick: int) -> bool:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with(option):
			for at in arg.trim_prefix(option).split(","):
				if int(at) == tick and tick > started_tick:
					return true
	return false


## Whether this run was given `--escape=` (the menu's cue to press Enter).
static func escaping() -> bool:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--escape="):
			return true
	return false


## Lay the screen out for `size` logical pixels, whatever the window is. A
## windowed window smaller than that is grown to it (never shrunk, never moved
## out of fullscreen); headless runs have no window to size.
static func fit(window: Window, size: Vector2i = BASE_SIZE) -> void:
	window.content_scale_size = size
	if DisplayServer.get_name() == "headless":
		return
	if window.mode == Window.MODE_WINDOWED and (window.size.x < size.x or window.size.y < size.y):
		window.size = size


static func toggle_fullscreen(window: Window) -> void:
	if window.mode == Window.MODE_FULLSCREEN or window.mode == Window.MODE_EXCLUSIVE_FULLSCREEN:
		window.mode = Window.MODE_WINDOWED
	else:
		window.mode = Window.MODE_FULLSCREEN


## The largest font size, from `size` down to `smallest`, at which `text` is
## no wider than `width`. A line that is too long is drawn smaller rather than
## over its neighbour or off the window.
static func fitting_size(text: String, width: float, size: int, smallest: int = 10) -> int:
	var font := ThemeDB.fallback_font
	var fitted := size
	while fitted > smallest and font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, fitted).x > width:
		fitted -= 1
	return fitted


## A line of text on `canvas`, its baseline starting at `at`. One too long for
## its space — `width` where something is drawn beside it, else up to `margin`
## short of the window's right edge — is drawn smaller rather than over its
## neighbour or off the screen.
static func text(canvas: CanvasItem, at: Vector2, line: String, color: Color, size: int = 16, width: float = -1.0, margin: float = 8.0) -> void:
	var room := width if width > 0.0 else canvas.get_viewport_rect().size.x - at.x - margin
	canvas.draw_string(ThemeDB.fallback_font, at, line, HORIZONTAL_ALIGNMENT_LEFT, -1, fitting_size(line, room, size), color)


## The key legend at the bottom of a screen: `lines` drawn `step` apart from
## `at` down, each fitted to `width` as `text` fits it, with `enter` (what
## Enter does: on a unit's screen, back to the map on the same plant, M84)
## and "Esc menu" closing the last — so a screen adds a key in one place and every screen has
## the way out.
static func legend(canvas: CanvasItem, at: Vector2, lines: Array, color: Color, size: int = 16, step: float = 24.0, width: float = -1.0, enter: String = "Enter map, same plant") -> void:
	for i in lines.size():
		var line: String = lines[i]
		if i == lines.size() - 1:
			line += "   %s   Esc menu" % enter
		text(canvas, at + Vector2(0, step * i), line, color, size, width)


## The options every screen reads after `--`: {auto, shots, shot}, and "plant"
## only when `--plant=` was given (an empty one is kept, for the screen to
## refuse). `--shots=` is a folder a timeline's frames go to, `--shot=` one
## file. A screen's own options are its own to read.
static func arguments() -> Dictionary:
	var found := {"auto": false, "shots": "", "shot": ""}
	for arg in OS.get_cmdline_user_args():
		if arg == "--auto":
			found["auto"] = true
		elif arg.begins_with("--plant="):
			found["plant"] = arg.trim_prefix("--plant=")
		elif arg.begins_with("--shots="):
			found["shots"] = arg.trim_prefix("--shots=")
		elif arg.begins_with("--shot="):
			found["shot"] = arg.trim_prefix("--shot=")
	return found


## Save the window as a PNG at `path` once the frame being drawn is done, and
## print "<who>: saved <path>". A coroutine: `await Screen.shoot(...)`, or the
## caller runs on before the frame is saved.
static func shoot(canvas: CanvasItem, who: String, path: String) -> void:
	await RenderingServer.frame_post_draw
	var image := canvas.get_viewport().get_texture().get_image()
	if image != null:
		image.save_png(path)
		print("%s: saved %s" % [who, path])


## A range around `low`..`high` (M83, the reactor's chart's own rule): widened
## to whole `step`s counted from `offset` (273.15 to count degrees Celsius
## from kelvin), at least `least` tall so a settled line is not drawn as noise.
static func nice_range(low: float, high: float, step: float, least: float, offset: float = 0.0) -> Array:
	var a := floorf((low - offset) / step) * step
	var b := ceilf((high - offset) / step) * step
	if b - a < least:
		var middle := (a + b) * 0.5
		a = floorf((middle - least * 0.5) / step) * step
		b = a + least
	return [a + offset, b + offset]


## Where `value` falls on a chart's `rect` drawn over `scale` ([low, high]):
## low at the bottom edge, high at the top, kept inside.
static func chart_y(rect: Rect2, scale: Array, value: float) -> float:
	return rect.end.y - rect.size.y * clampf((value - scale[0]) / (scale[1] - scale[0]), 0.0, 1.0)


## Lines across `rect` at whole steps of `scale`, about five to a chart, each
## labelled left of it in `format` (counted from `offset`, as `nice_range`).
static func grid(canvas: CanvasItem, rect: Rect2, scale: Array, step: float, offset: float = 0.0, format: String = "%.0f") -> void:
	var span: float = scale[1] - scale[0]
	while span / step > 6.0:
		step *= 2.0
	var c := ceilf((scale[0] - offset) / step) * step
	while c + offset <= scale[1] + 0.01:
		var y := chart_y(rect, scale, c + offset)
		canvas.draw_line(Vector2(rect.position.x, y), Vector2(rect.end.x, y), GRID, 1.0)
		text(canvas, Vector2(rect.position.x - 40, y + 4), format % c, DIM, 12)
		c += step


## A time chart (M83, lifted from the reactor's): `title` over its panel, the
## grid, `marks` across it, each `series` as a line, the frame, and `ends`
## under its two ends ([left, right], e.g. ["-30 min", "now"]).
##   series  [[points, color, width], …] drawn in that order; points are
##           [tick, value] pairs, a NaN value left out; a tick `now_tick -
##           span_ticks` sits at the left edge, `now_tick` at the right.
##   marks   [[value, color, label], …], a limit or a target: a line, its
##           label at the right end.
##   axis    {step, least, offset, format} as `nice_range` and `grid` take
##           them; with "range": [low, high] the scale is that, not fitted.
## The vertical scale fits the series and the marks. With nothing to draw,
## the panel says `waiting` instead.
static func chart(canvas: CanvasItem, rect: Rect2, title: String, series: Array, marks: Array, axis: Dictionary, now_tick: int, span_ticks: float, ends: Array, waiting: String = "waiting for the first sample") -> void:
	canvas.draw_rect(rect, CHART_BACKGROUND)
	text(canvas, rect.position + Vector2(0, -10), title, DIM, 14)
	var points := 0
	var low := INF
	var high := -INF
	for line in series:
		for point in line[0]:
			points += 1
			if not is_nan(point[1]):
				low = minf(low, point[1])
				high = maxf(high, point[1])
	for mark in marks:
		low = minf(low, mark[0])
		high = maxf(high, mark[0])
	if points == 0 or low > high:
		canvas.draw_rect(rect, SHELL, false, 1.0)
		text(canvas, rect.position + Vector2(12, 28), waiting, DIM, 14)
		return
	var offset: float = axis.get("offset", 0.0)
	var scale: Array = axis["range"] if axis.has("range") else nice_range(low, high, axis["step"], axis["least"], offset)
	grid(canvas, rect, scale, axis["step"], offset, axis.get("format", "%.0f"))
	for mark in marks:
		var y := chart_y(rect, scale, mark[0])
		canvas.draw_line(Vector2(rect.position.x, y), Vector2(rect.end.x, y), mark[1], 1.0)
		text(canvas, Vector2(rect.end.x - 54, y - 4), mark[2], mark[1], 12)
	for line in series:
		var drawn := PackedVector2Array()
		for point in line[0]:
			if not is_nan(point[1]):
				var x: float = rect.end.x - rect.size.x * (now_tick - int(point[0])) / span_ticks
				drawn.append(Vector2(x, chart_y(rect, scale, point[1])))
		if drawn.size() > 1:
			canvas.draw_polyline(drawn, line[1], line[2])
	canvas.draw_rect(rect, SHELL, false, 1.0)
	text(canvas, Vector2(rect.position.x, rect.end.y + 16), ends[0], DIM, 12)
	text(canvas, Vector2(rect.end.x - 26, rect.end.y + 16), ends[1], DIM, 12)


## How fast a pipe's dashes move [px/s] (M83): by `share`, its flow's share of
## the screen's largest, from DASH_SLOW to DASH_FAST as its square root (as
## the map's pipe widths grow); negative, flowing back, they move back.
static func dash_speed(share: float) -> float:
	if share == 0.0:
		return 0.0
	return signf(share) * lerpf(DASH_SLOW, DASH_FAST, sqrt(minf(absf(share), 1.0)))


## A pipe (M83): `points`, from its upstream end, as one line and — unless
## `travel` is NaN, a pipe not flowing — short dashes along it, lighter than
## the line, moved `travel` px on from the upstream end. A screen that adds
## `dash_speed` × the frame's seconds to `travel` each frame the plant runs
## makes them run the way the flow does. A dash is carried round a corner,
## not cut at it.
static func pipe(canvas: CanvasItem, points: PackedVector2Array, color: Color, width: float, travel: float = NAN) -> void:
	canvas.draw_polyline(points, color, width)
	if is_nan(travel) or points.size() < 2:
		return
	# How far along the pipe each of its points is [px].
	var lengths := PackedFloat32Array([0.0])
	for i in range(1, points.size()):
		lengths.append(lengths[i - 1] + points[i - 1].distance_to(points[i]))
	var total: float = lengths[lengths.size() - 1]
	var dash_color := color.lightened(0.55)
	var dash_width := maxf(1.0, width * 0.5)
	var start := fposmod(travel, DASH_PERIOD) - DASH_PERIOD
	while start < total:
		var a := maxf(start, 0.0)
		var b := minf(start + DASH_LENGTH, total)
		if b > a:
			var dash := PackedVector2Array([_along(points, lengths, a)])
			for i in points.size():
				if lengths[i] > a and lengths[i] < b:
					dash.append(points[i])
			dash.append(_along(points, lengths, b))
			canvas.draw_polyline(dash, dash_color, dash_width)
		start += DASH_PERIOD


## The point `distance` px along the line through `points`, `lengths` how far
## along each point is.
static func _along(points: PackedVector2Array, lengths: PackedFloat32Array, distance: float) -> Vector2:
	for i in range(1, points.size()):
		if distance <= lengths[i] or i == points.size() - 1:
			var segment := lengths[i] - lengths[i - 1]
			var share := 0.0 if segment <= 0.0 else (distance - lengths[i - 1]) / segment
			return points[i - 1].lerp(points[i], share)
	return points[0]
