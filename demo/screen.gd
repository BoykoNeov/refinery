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
# Nothing here touches the simulator, and nothing here is physical.
extends RefCounted

const MENU := "res://demo/menu.tscn"

## The size every screen but the reactor's is laid out for. project.godot's
## stretch settings scale this to whatever the window is, so a screen draws at
## these coordinates on any monitor.
const BASE_SIZE := Vector2i(1152, 648)

## Where the menu leaves the plant it was asked for, for the screen it opens.
const PLANT_META := "refinery_plant"
## Where the menu leaves a save to start from instead (M77).
const SAVE_META := "refinery_save"

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


static func to_menu(tree: SceneTree) -> void:
	tree.change_scene_to_file(MENU)


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
## `at` down, each fitted to `width` as `text` fits it, with "Esc menu" closing
## the last — so a screen adds a key in one place and every screen has the way
## out.
static func legend(canvas: CanvasItem, at: Vector2, lines: Array, color: Color, size: int = 16, step: float = 24.0, width: float = -1.0) -> void:
	for i in lines.size():
		var line: String = lines[i]
		if i == lines.size() - 1:
			line += "   Esc menu"
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
