# The plant that is running, one for every screen (M84,
# docs/plans/plant-map-screen.md slice 4).
#
# Until M84 each screen owned its own simulator, so going from the map to the
# column screen started the plant again. Now this node owns the one simulator,
# above the scenes (an autoload: project.godot, [autoload] Running), and a
# screen opened with Enter takes the plant over as it stands instead of
# loading it again: the map is home, a unit's screen a closer look at the
# same running plant.
#
# A screen reaches it by path, `get_node("/root/Running")`, not by its global
# name: like screen.gd's rule on `class_name`, nothing here depends on a cache
# the editor writes.
#
# Beside the simulator it keeps what the screens noted about this plant that
# the simulator does not keep itself, so a screen that takes the plant over
# reads it too: what each command was in words, the holes the map made, and
# the plant as it was loaded (the column's design duties are read from it).
# Every fresh load forgets all three.
#
# Nothing here is physical, and nothing here ticks: whichever screen is shown
# runs the plant; the menu holds it still.
extends Node

## The one simulator.
var sim: RefinerySim
## The plant file running, as the screens name it (`res://scenarios/<stem>.toml`);
## "" before the first load, or after one that failed.
var scenario := ""
## Every command text the engine took → what the screen that sent it called
## it, so a recovery on any screen can say what it undid (M76).
var said: Dictionary = {}
## The punctures the map made, in order: [command text, edge id, area m²]
## (map.gd's `holes`: the snapshot reports a hole's flow, not its size).
var holes: Array = []
## The plant's snapshot as it was loaded, before its first tick: what a screen
## that takes the plant over later reads its design from, so a setting moved
## since is not mistaken for the design. {} after a save is loaded.
var as_loaded: Dictionary = {}


func _init() -> void:
	sim = RefinerySim.new()
	sim.name = "Sim"
	add_child(sim)


## Load the plant at `path` fresh, forgetting everything noted about the last:
## the simulator's own answer (JSON `null`, or its error).
func load_scenario(path: String) -> String:
	_forget()
	var answer: String = sim.load_scenario(path)
	if JSON.parse_string(answer) == null:
		scenario = path
		as_loaded = JSON.parse_string(sim.snapshot_json())
	return answer


## Load a save (M77) of the plant at `path`: the simulator's own answer. The
## words its commands were sent with come from the save's note (`said`).
func load_save(text: String, path: String, said_then: Dictionary) -> String:
	var answer: String = sim.load_save(text)
	if JSON.parse_string(answer) == null:
		_forget()
		scenario = path
		said.merge(said_then)
	return answer


func _forget() -> void:
	scenario = ""
	said.clear()
	holes.clear()
	as_loaded = {}
