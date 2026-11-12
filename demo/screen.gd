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
