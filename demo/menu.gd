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
# Computes nothing and loads no plant: it only chooses a scene. The plant
# running (M84) is held still while the menu is up, and the menu says plainly
# which rows keep it and which start over:
#   - "The plant running" heads the list when a plant is running: one row for
#     its own screen, one for the map; both carry on with it as it stands.
#     The row for the screen Esc came from is the one chosen, so Esc then
#     Enter is a round trip.
#   - The map's own row carries on with it too (Left/Right there start over).
#   - Every row naming a plant, and the quick save, start over; the running
#     plant's own row says so beside it.
#
#   godot --path .                     this menu (the project's main scene)
#   godot --headless --path . -- --auto
#       ...still the M6.2 demo: `--auto` has no meaning for a menu, and that
#       command predates it, so it is passed on to the leaking line.
extends Node2D

const Screen := preload("res://demo/screen.gd")

## The screens and their order: screen.gd's SCREENS (M84: Enter on the map
## reads the same list).
const SCREENS := Screen.SCREENS

const BACKGROUND := Screen.BACKGROUND
const INK := Screen.INK
const DIM := Screen.DIM
const ACCENT := Screen.FLAME
const BAND := Color(0.16, 0.18, 0.22)
const GOOD := Screen.GOOD
const WARN := Screen.WARN

const LEFT := 40.0
## Under the keys' line, two lines for the chosen row's plant's description
## (M87), then the list.
const TOP := 112.0
const ABOUT_Y := 82.0
const ROW := 17.0
const HEADING := 21.0
## The lowest a row may sit: past it the list is drawn closer together, so a
## Continue row (M77) does not push the last screen off the window.
const BOTTOM := 638.0

## One entry per line the menu can open: {scene, plant, label, y}, with
## "running": true on a row that carries on with the plant running and a
## "note" drawn beside a row (what it does to the plant running).
var rows: Array = []
## Each screen's heading: {name, about, y}.
var headings: Array = []
var selected := 0
## Whether a plant is running behind the menu (M84): the menu then says which
## rows keep it.
var is_running := false
## Beside the row that names the plant running: choosing it loses the run.
const RESTARTS := "starts this plant over — your changes are lost"
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
	elif Screen.escaping() and Screen.running(get_tree()).scenario != "":
		# A recorded round trip (`--escape=`): Enter on the row chosen.
		_open.call_deferred()
	var came_from := Screen.chosen_from(get_tree())
	var running_scenario: String = Screen.running(get_tree()).scenario
	is_running = running_scenario != ""
	var y := TOP
	if is_running:
		var title := _title_of(running_scenario)
		var at := _running_time()
		var screen_scene := Screen.screen_for(running_scenario)
		y += HEADING
		headings.append({"name": "The plant running", "about": "%s, t = %s" % [title, at], "y": y})
		y += 4.0
		if screen_scene != "":
			y += ROW
			rows.append({"scene": screen_scene, "plant": "", "running": true, "label": "Carry on, on the %s screen" % _screen_name(screen_scene), "y": y, "scenario": running_scenario})
		y += ROW
		rows.append({"scene": Screen.MAP, "plant": "", "running": true, "label": "Carry on, on the plant map", "y": y, "scenario": running_scenario})
		selected = 0
		for i in rows.size():
			if rows[i]["scene"] == came_from:
				selected = i
	var save: Dictionary = Screen.read_save(Screen.QUICK_SAVE)
	if not save.is_empty() and save["note"].has("scene"):
		y += HEADING
		var about := "back to the moment F5 was pressed on the pump screen"
		if is_running:
			about += " — the plant running is dropped"
		headings.append({"name": "Quick save", "about": about, "y": y})
		y += 4.0 + ROW
		rows.append({"scene": str(save["note"]["scene"]), "plant": "", "save": Screen.QUICK_SAVE, "label": _save_label(save), "y": y})
	for screen in SCREENS:
		y += HEADING
		headings.append({"name": screen["name"], "about": screen["about"], "y": y})
		y += 4.0
		var plants := _plants_of(screen["script"])
		if plants.is_empty():
			y += ROW
			var row := {"scene": screen["scene"], "plant": "", "label": screen["row"], "y": y, "scenario": str(screen.get("scenario", ""))}
			if screen["scene"] == Screen.MAP and is_running:
				# The map opens on the plant running (M84): no plant named.
				row["label"] = "The plant running, as you left it; Left/Right there start another plant over"
				row["running"] = true
				row["scenario"] = running_scenario
			elif screen["scene"] == Screen.MAP:
				row["label"] = "Any plant in scenarios/ (Left/Right there to choose)"
			elif is_running and screen.get("scenario", "") == running_scenario:
				row["note"] = RESTARTS
			rows.append(row)
		for key in plants:
			y += ROW
			var row := {"scene": screen["scene"], "plant": key, "label": str(plants[key].get("title", key)), "y": y, "scenario": str(plants[key].get("scenario", ""))}
			if is_running and str(plants[key].get("scenario", "")) == running_scenario:
				row["note"] = RESTARTS
			rows.append(row)
	# `--select=<plant>` (M87, for a screenshot): the first row running that
	# plant's file is the one chosen.
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--select="):
			var wanted := "/%s.toml" % arg.trim_prefix("--select=")
			for i in rows.size():
				if str(rows[i].get("scenario", "")).ends_with(wanted):
					selected = i
					break
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


func _plants_of(script_path: String) -> Dictionary:
	return Screen.plants_of(script_path)


## The plant in `scenario` by the name the menu lists it under: its screen's
## title for it, the leaking line's row, else the file's stem.
func _title_of(scenario: String) -> String:
	for screen in SCREENS:
		if screen.get("scenario", "") == scenario:
			return str(screen["row"])
		var plants := _plants_of(screen["script"])
		var key := Screen.key_for(plants, scenario)
		if key != "":
			return str(plants[key].get("title", key))
	return scenario.get_file().trim_suffix(".toml")


## The menu's heading for the screen `scene`, lower case: "distillation column".
func _screen_name(scene: String) -> String:
	for screen in SCREENS:
		if screen["scene"] == scene:
			return str(screen["name"]).to_lower()
	return scene.get_file().get_basename()


## How far the plant running has got, in plant time: "1234 s".
func _running_time() -> String:
	var snapshot = JSON.parse_string(Screen.running(get_tree()).sim.snapshot_json())
	if not (snapshot is Dictionary):
		return "?"
	return "%.0f s" % float(snapshot.get("sim_time", 0.0))


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
	if row.get("running", false):
		Screen.open_running(get_tree(), row["scene"])
		return
	Screen.open(get_tree(), row["scene"], row["plant"])


func _draw() -> void:
	var size := get_viewport_rect().size
	draw_rect(Rect2(Vector2.ZERO, size), BACKGROUND)
	_text(Vector2(LEFT, 40), "Refinery", INK, 28)
	if is_running:
		_text(Vector2(LEFT + 150, 40), "green rows carry on with the plant running (it waits while you are here); every other row starts its plant from the beginning", DIM, 15)
	else:
		_text(Vector2(LEFT + 150, 40), "pick a plant to run", DIM, 16)
	_text(Vector2(LEFT, 62), "Up / Down or the mouse to choose    Enter to open    Esc inside a screen comes back here    F11 full screen", DIM, 13)
	# The chosen row's plant, in its file's words (M87): two lines at most.
	var about := Screen.description(str(rows[selected].get("scenario", ""))) if not rows.is_empty() and str(rows[selected].get("scenario", "")) != "" else ""
	if about != "":
		draw_multiline_string(ThemeDB.fallback_font, Vector2(LEFT, ABOUT_Y), about, HORIZONTAL_ALIGNMENT_LEFT, size.x - 2.0 * LEFT, 14, 2, INK)

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
		var label_color := INK if chosen else DIM
		if row.get("running", false):
			label_color = GOOD if chosen else GOOD.darkened(0.25)
		_text(Vector2(LEFT + 14, y), row["label"], label_color, 14)
		if row.has("note"):
			var label_width := ThemeDB.fallback_font.get_string_size(row["label"], HORIZONTAL_ALIGNMENT_LEFT, -1, 14).x
			_text(Vector2(LEFT + 14 + label_width + 16, y), "← " + row["note"], WARN, 13)


func _text(at: Vector2, text: String, color: Color, size: int = 16) -> void:
	Screen.text(self, at, text, color, size, -1.0, 16.0)
