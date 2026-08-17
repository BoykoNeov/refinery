# M6.2's demonstrated acceptance criterion (DESIGN §8): a scene that runs the
# engine, reads its snapshots, and draws the two things the damage model can do.
#
# "Demonstrated", not gated, and the distinction is the point: no `cargo test`
# can assert that a tank looks like a tank. What IS checkable is that the
# numbers on screen came out of the engine, so this script computes nothing
# physical — every quantity it draws is a field it read from a snapshot.
#
# Interactive:  open the project and press P / F / R (see _input).
# Recorded:     godot --headless --path . -- --auto
#               ...which runs the scripted sequence below and prints a `t=` line
#               every 50 ticks. Those lines are the observation in ROADMAP.md.
#
# Time runs ~6x faster than real: one engine tick (0.1 s of plant time) per
# physics frame (1/60 s).
extends Node2D

const SCENARIO := "res://scenarios/leaking_line.toml"
## 10 cm². The hole `leak_reference` sizes so that ~37% of the transfer goes
## out of it — chosen there to be visible rather than nominal, and reused here
## for the same reason.
const HOLE_M2 := 0.001
## 5 MW onto the receiving tank.
const FIRE_W := 5.0e6

## Scripted timeline for the recorded run, in ticks.
const AUTO_PUNCTURE := 100
const AUTO_IGNITE := 200
const AUTO_REPAIR := 300
const AUTO_END := 350

@onready var sim: RefinerySim = $Sim

var supply_id := -1
var receiving_id := -1
var fill_line_id := -1
var snapshot: Dictionary = {}
var halted := ""
var auto_run := false
## Largest tank mass seen anywhere this run — see the note on _tank_fraction.
var peak_mass := 0.0


func _ready() -> void:
	auto_run = "--auto" in OS.get_cmdline_user_args()

	var err = JSON.parse_string(sim.load_scenario(SCENARIO))
	if err != null:
		_halt("load failed: %s" % err)
		return

	supply_id = sim.node_id("supply_tank")
	receiving_id = sim.node_id("receiving_tank")
	fill_line_id = sim.edge_id("fill_line")
	if supply_id < 0 or receiving_id < 0 or fill_line_id < 0:
		_halt("this scenario does not have the names the scene expects")
		return

	print("plant: loaded %s" % SCENARIO)
	print("plant: keys — P puncture, F fire, R repair and extinguish")


func _physics_process(_delta: float) -> void:
	if halted != "":
		return

	var err = JSON.parse_string(sim.tick())
	if err != null:
		# A diverged solve is not necessarily terminal, but a scene that keeps
		# ticking through one shows a plant nobody solved. Stop and say so.
		_halt("tick %d: %s" % [sim.tick_index(), err["message"]])
		return

	snapshot = JSON.parse_string(sim.snapshot_json())
	queue_redraw()

	var tick := sim.tick_index()
	if tick % 50 == 0:
		print(_readout(tick))

	if auto_run:
		match tick:
			AUTO_PUNCTURE:
				print("plant: puncturing fill_line, %s m^2" % HOLE_M2)
				_puncture(HOLE_M2)
			AUTO_IGNITE:
				print("plant: fire on receiving_tank, %s W" % FIRE_W)
				_ignite(FIRE_W)
			AUTO_REPAIR:
				print("plant: repaired and extinguished")
				_puncture(0.0)
				_ignite(0.0)
			AUTO_END:
				get_tree().quit()


func _input(event: InputEvent) -> void:
	if not (event is InputEventKey and event.pressed and not event.echo):
		return
	match event.keycode:
		KEY_P:
			_puncture(HOLE_M2)
		KEY_F:
			_ignite(FIRE_W)
		KEY_R:
			_puncture(0.0)
			_ignite(0.0)


# --------------------------------------------------------------- commands

func _puncture(area: float) -> void:
	_send({"cmd": "puncture_pipe", "edge": fill_line_id, "area": area})


func _ignite(power: float) -> void:
	_send({"cmd": "set_heat_input", "node": receiving_id, "power": power})


func _send(command: Dictionary) -> void:
	var err = JSON.parse_string(sim.apply_command(JSON.stringify(command)))
	if err != null:
		push_error("refused: %s" % err["message"])


# ---------------------------------------------------------------- reading

## Every getter below is a field lookup. Nothing here computes physics — if a
## number is not in the snapshot, this scene does not draw it.

func _node(id: int) -> Dictionary:
	return snapshot["nodes"][id]


func _mass(id: int) -> float:
	return _node(id)["kind"]["mass"]


func _temperature(id: int) -> float:
	return _node(id)["kind"]["temperature"]


func _fire_w(id: int) -> float:
	return _node(id)["heat_input_w"]


func _leak_kg_s() -> float:
	return snapshot["edges"][fill_line_id]["leak_mass_flow"]


## How tall to draw a tank's bar, 0..1.
##
## **This is mass on a shared scale, NOT a fill level, and the difference is a
## gap rather than a style choice.** The snapshot cannot answer "how full is
## it": a tank reports mass [kg], area [m²] and height [m], and turning that
## into a level needs the fluid's density — which lives in the slate, and the
## slate is not in the snapshot (docs/ROADMAP.md M6.2, the deferral).
## Hardcoding water's 998 kg/m³ here would draw a confident level that is wrong
## for every other plant: the invented-data failure this repo has three notes
## about, in a frontend. The kg figure beside each bar is the absolute truth.
##
## The scale is shared across tanks and is the largest mass seen anywhere this
## run, so the bars are comparable with each other. A per-tank scale was tried
## first and is worse than useless: a tank that is FILLING is always at its own
## maximum, so it draws as permanently full.
func _tank_fraction(id: int) -> float:
	if peak_mass <= 0.0:
		return 0.0
	return clampf(_mass(id) / peak_mass, 0.0, 1.0)


## One pass over the tanks the scene draws, before any of them is drawn — so
## both bars use the same scale within a frame.
func _rescale() -> void:
	for id in [supply_id, receiving_id]:
		peak_mass = max(peak_mass, _mass(id))


func _readout(tick: int) -> String:
	return (
		"t=%4d  supply=%9.1f kg  receiving=%9.1f kg  T=%7.3f K  leak=%6.3f kg/s  fire=%5.2f MW"
		% [
			tick,
			_mass(supply_id),
			_mass(receiving_id),
			_temperature(receiving_id),
			_leak_kg_s(),
			_fire_w(receiving_id) / 1.0e6,
		]
	)


# ---------------------------------------------------------------- drawing

const SUPPLY_RECT := Rect2(70, 150, 150, 320)
## Bars are mass on a SHARED scale, not fill level — see _tank_fraction.
const RECEIVING_RECT := Rect2(790, 150, 150, 320)
const PUMP_POS := Vector2(300, 470)
const VALVE_POS := Vector2(430, 470)
const LEAK_POS := Vector2(620, 300)

const SHELL := Color(0.42, 0.45, 0.50)
const LIQUID := Color(0.20, 0.45, 0.75)
const HOT := Color(0.90, 0.30, 0.15)
const PIPE := Color(0.55, 0.58, 0.62)
const FLAME := Color(1.0, 0.55, 0.10)
const SPRAY := Color(0.45, 0.70, 0.95)
const INK := Color(0.88, 0.90, 0.93)


func _draw() -> void:
	draw_rect(Rect2(Vector2.ZERO, get_viewport_rect().size), Color(0.09, 0.10, 0.12))

	if halted != "":
		_text(Vector2(30, 40), "HALTED — %s" % halted, HOT)
		return
	if snapshot.is_empty():
		return

	_rescale()
	_draw_line_run()
	_draw_tank(SUPPLY_RECT, supply_id, "supply_tank")
	_draw_tank(RECEIVING_RECT, receiving_id, "receiving_tank")
	_draw_leak()
	_draw_fire()

	_text(Vector2(30, 40), _readout(sim.tick_index()), INK)
	_text(Vector2(30, 64), "P puncture    F fire    R repair + extinguish", PIPE)


func _draw_tank(rect: Rect2, id: int, label: String) -> void:
	var fraction := _tank_fraction(id)
	var fill := Rect2(
		rect.position + Vector2(0, rect.size.y * (1.0 - fraction)),
		Vector2(rect.size.x, rect.size.y * fraction)
	)
	# Tint toward red with temperature: 20 C is cold, 60 C is the far end.
	var warmth := clampf((_temperature(id) - 293.15) / 40.0, 0.0, 1.0)
	draw_rect(fill, LIQUID.lerp(HOT, warmth))
	draw_rect(rect, SHELL, false, 3.0)
	# All labels BELOW the shell: above it is where the flames go.
	var below := rect.position + Vector2(0, rect.size.y)
	_text(below + Vector2(0, 26), label, INK)
	_text(below + Vector2(0, 48), "%.0f kg" % _mass(id), INK)
	_text(below + Vector2(0, 70), "%.2f K" % _temperature(id), INK)


func _draw_line_run() -> void:
	var path := PackedVector2Array(
		[
			SUPPLY_RECT.position + Vector2(SUPPLY_RECT.size.x, SUPPLY_RECT.size.y),
			Vector2(SUPPLY_RECT.position.x + SUPPLY_RECT.size.x, PUMP_POS.y),
			PUMP_POS,
			VALVE_POS,
			LEAK_POS,
			RECEIVING_RECT.position + Vector2(0, RECEIVING_RECT.size.y * 0.5),
		]
	)
	draw_polyline(path, PIPE, 6.0)
	draw_circle(PUMP_POS, 16, PIPE)
	_text(PUMP_POS + Vector2(-24, 40), "pump", PIPE)
	draw_rect(Rect2(VALVE_POS - Vector2(12, 12), Vector2(24, 24)), PIPE)
	_text(VALVE_POS + Vector2(-24, 40), "valve", PIPE)


## The leak: a spray whose length is the reported mass flow. Drawn only when
## the engine says mass is leaving, so an unpunctured plant shows nothing and a
## repaired one stops immediately.
func _draw_leak() -> void:
	var flow := _leak_kg_s()
	if flow <= 0.0:
		return
	var reach := 20.0 + flow * 24.0
	for i in 7:
		var spread := deg_to_rad(60.0 + i * 10.0)
		draw_line(LEAK_POS, LEAK_POS + Vector2(cos(spread), sin(spread)) * reach, SPRAY, 3.0)
	_text(LEAK_POS + Vector2(-30, -20), "%.2f kg/s" % flow, SPRAY)


## The fire: read from `heat_input_w`, which is the engine's own answer to "is
## this node on fire?" — NOT from this scene's memory of having sent the
## command. That is the difference between drawing the plant and drawing your
## own past actions; see the M6.2 note in ROADMAP.md.
func _draw_fire() -> void:
	var power := _fire_w(receiving_id)
	if power <= 0.0:
		return
	var height := 30.0 + sqrt(power / 1.0e6) * 26.0
	var base := RECEIVING_RECT.position + Vector2(RECEIVING_RECT.size.x * 0.5, 0)
	for i in 3:
		var offset := Vector2((i - 1) * 26.0, 0)
		draw_colored_polygon(
			PackedVector2Array(
				[
					base + offset + Vector2(-16, 0),
					base + offset + Vector2(0, -height),
					base + offset + Vector2(16, 0),
				]
			),
			FLAME
		)
	_text(base + Vector2(-40, -height - 12), "%.1f MW" % (power / 1.0e6), FLAME)


func _text(at: Vector2, text: String, color: Color) -> void:
	draw_string(ThemeDB.fallback_font, at, text, HORIZONTAL_ALIGNMENT_LEFT, -1, 16, color)


func _halt(reason: String) -> void:
	halted = reason
	push_error("plant: %s" % reason)
	print("plant: HALTED — %s" % reason)
	queue_redraw()
