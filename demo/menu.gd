# The way in: every screen and every plant each can open, on one page.
#
# Until this scene the choice was a console menu in Refinery.cmd (screens
# only) and a row of number keys inside each screen (its plants). This lists
# both, read from the screens themselves: each screen's `PLANTS` table is the
# source of its rows here, so a plant added to a screen appears without this
# file changing. Esc in any screen comes back here.
#
# M77: when a quick save exists (F5 on the pump screen), a Continue row heads
# the list and opens its screen on it.
#
# Computes nothing and loads no simulator: it only chooses a scene.
#
#   godot --path .                     this menu (the project's main scene)
#   godot --headless --path . -- --auto
#       ...still the M6.2 demo: `--auto` has no meaning for a menu, and that
#       command predates it, so it is passed on to the leaking line.
extends Node2D

const Screen := preload("res://demo/screen.gd")

## The screens, in the order a newcomer should meet them: the newest and
## richest first. `script` is read for its PLANTS; a screen with no table
## (the leaking line's one plant, the map of all of them) names its one `row`.
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
	},
	{
		"name": "Plant map",
		"about": "any of the shipped plants, drawn from its own data — click its valves, pumps, furnaces and pipes",
		"scene": "res://demo/map.tscn",
		"script": "",
		"row": "Every plant in scenarios/, one after another (Left/Right)",
	},
]

const BACKGROUND := Screen.BACKGROUND
const INK := Screen.INK
const DIM := Screen.DIM
const ACCENT := Screen.FLAME
const BAND := Color(0.16, 0.18, 0.22)

const LEFT := 40.0
const TOP := 66.0
const ROW := 17.0
const HEADING := 21.0
## The lowest a row may sit: past it the list is drawn closer together, so a
## Continue row (M77) does not push the last screen off the window.
const BOTTOM := 638.0

## One entry per line the menu can open: {scene, plant, label, y}.
var rows: Array = []
## Each screen's heading: {name, about, y}.
var headings: Array = []
var selected := 0
## The height of a row as drawn: ROW, or less when the list is squeezed.
var row_step := ROW


func _ready() -> void:
	var options := Screen.arguments()
	if options["auto"]:
		get_tree().change_scene_to_file.call_deferred("res://demo/plant.tscn")
		return
	Screen.fit(get_window())
	if options["shot"] != "":
		_shoot.call_deferred(options["shot"])
	var y := TOP
	var save: Dictionary = Screen.read_save(Screen.QUICK_SAVE)
	if not save.is_empty() and save["note"].has("scene"):
		y += HEADING
		headings.append({"name": "Continue", "about": "where you left off (F5 saves, on the pump screen)", "y": y})
		y += 4.0 + ROW
		rows.append({"scene": str(save["note"]["scene"]), "plant": "", "save": Screen.QUICK_SAVE, "label": _save_label(save), "y": y})
	for screen in SCREENS:
		y += HEADING
		headings.append({"name": screen["name"], "about": screen["about"], "y": y})
		y += 4.0
		var plants := _plants_of(screen["script"])
		if plants.is_empty():
			y += ROW
			rows.append({"scene": screen["scene"], "plant": "", "label": screen["row"], "y": y})
		for key in plants:
			y += ROW
			rows.append({"scene": screen["scene"], "plant": key, "label": str(plants[key].get("title", key)), "y": y})
	if y > BOTTOM:
		var squeeze := (BOTTOM - TOP) / (y - TOP)
		row_step = ROW * squeeze
		for entry in rows + headings:
			entry["y"] = TOP + (float(entry["y"]) - TOP) * squeeze
	queue_redraw()


## `--shot=<file>` on a windowed run: save the menu as a PNG and quit (the
## screens' `--shots=` for a page with no timeline).
func _shoot(path: String) -> void:
	queue_redraw()
	await RenderingServer.frame_post_draw
	await Screen.shoot(self, "menu", path)
	get_tree().quit()


## A screen's PLANTS table, key → its dictionary, in the file's order.
func _plants_of(script_path: String) -> Dictionary:
	if script_path == "":
		return {}
	var script: Script = load(script_path)
	if script == null:
		return {}
	return script.get_script_constant_map().get("PLANTS", {})


func _input(event: InputEvent) -> void:
	if rows.is_empty():
		return
	if event is InputEventMouseMotion:
		var hovered := _row_at(event.position)
		if hovered >= 0 and hovered != selected:
			selected = hovered
			queue_redraw()
	elif event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
		var clicked := _row_at(event.position)
		if clicked >= 0:
			selected = clicked
			_open()
	elif event is InputEventKey and event.pressed:
		match event.keycode:
			KEY_UP:
				selected = posmod(selected - 1, rows.size())
			KEY_DOWN:
				selected = posmod(selected + 1, rows.size())
			KEY_HOME:
				selected = 0
			KEY_END:
				selected = rows.size() - 1
			KEY_ENTER, KEY_KP_ENTER, KEY_SPACE:
				if not event.echo:
					_open()
					return
			KEY_F11:
				if not event.echo:
					Screen.toggle_fullscreen(get_window())
			KEY_ESCAPE:
				if not event.echo:
					get_tree().quit()
					return
		queue_redraw()


## The quick save's row: its plant, by the title its screen gives it, and when.
func _save_label(save: Dictionary) -> String:
	var note: Dictionary = save["note"]
	var plant := str(note.get("plant", ""))
	var script_path := str(note.get("scene", "")).trim_suffix(".tscn") + ".gd"
	var plants := _plants_of(script_path) if ResourceLoader.exists(script_path) else {}
	var title := str(plants[plant].get("title", plant)) if plants.has(plant) else plant
	return "Quick save: %s, %d ticks in" % [title, save["tick"]]


func _row_at(at: Vector2) -> int:
	for i in rows.size():
		var y: float = rows[i]["y"]
		if at.y > y - row_step + 4.0 and at.y <= y + 4.0 and at.x >= LEFT - 10.0:
			return i
	return -1


func _open() -> void:
	var row: Dictionary = rows[selected]
	if row.has("save"):
		Screen.open_save(get_tree(), row["scene"], row["save"])
		return
	Screen.open(get_tree(), row["scene"], row["plant"])


func _draw() -> void:
	var size := get_viewport_rect().size
	draw_rect(Rect2(Vector2.ZERO, size), BACKGROUND)
	_text(Vector2(LEFT, 44), "Refinery", INK, 28)
	_text(Vector2(LEFT + 150, 44), "pick a plant to run", DIM, 16)
	_text(Vector2(LEFT, 68), "Up / Down or the mouse to choose    Enter to open    Esc inside a screen comes back here    F11 full screen", DIM, 13)

	for heading in headings:
		var y: float = heading["y"]
		_text(Vector2(LEFT, y), heading["name"], ACCENT, 17)
		var name_width := ThemeDB.fallback_font.get_string_size(heading["name"], HORIZONTAL_ALIGNMENT_LEFT, -1, 17).x
		_text(Vector2(LEFT + name_width + 16, y), heading["about"], DIM, 13)

	for i in rows.size():
		var row: Dictionary = rows[i]
		var y: float = row["y"]
		var chosen := i == selected
		if chosen:
			draw_rect(Rect2(Vector2(LEFT - 10, y - row_step + 4.0), Vector2(size.x - 2 * (LEFT - 10), row_step)), BAND)
			draw_rect(Rect2(Vector2(LEFT - 10, y - row_step + 4.0), Vector2(3, row_step)), ACCENT)
		_text(Vector2(LEFT + 14, y), row["label"], INK if chosen else DIM, 14)


func _text(at: Vector2, text: String, color: Color, size: int = 16) -> void:
	Screen.text(self, at, text, color, size, -1.0, 16.0)
