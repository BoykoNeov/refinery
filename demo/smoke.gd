# Proves the GDExtension is loadable and its API answers, with no scene and no
# rendering involved:
#
#   godot --headless --path . --script res://demo/smoke.gd
#
# A missing or mismatched library makes RefinerySim an unknown identifier here,
# which is the failure this script exists to make loud and immediate. See the
# header of refinery.gdextension for the two setup steps a fresh clone needs.
extends SceneTree


func _initialize() -> void:
	var sim := RefinerySim.new()

	# Every call answers before a scenario is loaded — the contract
	# bridge::Session holds, checked here from the Godot side.
	print("smoke: unloaded tick_index=", sim.tick_index(), " is_loaded=", sim.is_loaded())
	print("smoke: unloaded tick=", sim.tick())

	var err = JSON.parse_string(sim.load_scenario("res://scenarios/leaking_line.toml"))
	if err != null:
		print("smoke: FAILED to load: ", err)
		quit(1)
		return

	var pipe := sim.edge_id("fill_line")
	sim.apply_command(
		JSON.stringify({"cmd": "puncture_pipe", "edge": pipe, "area": 0.001})
	)
	for i in 100:
		sim.tick()

	var snapshot = JSON.parse_string(sim.snapshot_json())
	print(
		"smoke: tick=",
		sim.tick_index(),
		" nodes=",
		snapshot["nodes"].size(),
		" leak=",
		snapshot["edges"][pipe]["leak_mass_flow"],
		" kg/s"
	)
	sim.free()
	quit()
