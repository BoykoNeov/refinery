# Where the map's pipes run (M85): at right angles, the way a plant's pipes
# do, round the nodes and their names, and not along one another.
#
# The map (demo/map.gd) hands this the box each drawn node takes, the box each
# node's name and reading take, and its pipes in the order to route them; it
# gets back each pipe's corners. Nothing here knows a plant: a box is a box.
#
# How a pipe is found. It leaves its upstream node from one SIDE of the box
# and enters its downstream one at a side, each through a short straight stub.
# Between the two stubs it runs one of a few shapes: straight, one corner, or
# three straight runs through a "channel" (an upright run at some x, or a level
# run at some y, tried every GRID px across the page). Every candidate gets a
# cost, and the cheapest is kept:
#   length, and a fixed price per bend          — short and plain beats long
#   through a node's box                         — almost never
#   through a node's name or reading             — avoided where it can be
#   along a pipe routed before it, on top of it  — far dearer than a detour
#   close beside one                             — a little, so lanes spread
#   across one                                   — moderate: crossings happen
# Candidates are sorted by their length-and-bends price and checked in that
# order; once that price alone is dearer than the best found, nothing later
# can win and the search stops.
#
# Two passes. The first finds which side each pipe leaves and enters by, each
# port at the height (or across-position) of the node at the other end, so a
# pipe between two nodes level with each other runs dead straight. The second
# spreads the pipes that share a side along it, in the order their other ends
# sit, never closer than PORT_GAP, and routes again on those ports.
#
# Deterministic: the same boxes and pipes give the same corners, every load
# (the candidates are generated and compared in a fixed order, ties kept by
# who came first).
#
# CHANGING THIS FILE? The hand layouts in demo/layout/ were tuned against it
# as it stands (M85): nodes nudged until their pipes ran straight, and a few
# px either way could turn a straight run into a hop. No test sees an ugly
# pipe — `map --auto` only checks every plant loads and runs. After any change
# here, screenshot every plant (the `--shot=` line in CLAUDE.md) and look.
extends RefCounted

## A port's stub: how far a pipe runs straight out of a side before it may turn [px].
const STUB := 12.0
## The spacing of the channels a pipe may run through [px].
const GRID := 10.0
## How far beyond its two ends a pipe may swing round [px].
const SWING := 80.0
## How far a pipe keeps clear of a box that is not one of its ends [px].
const MARGIN := 6.0
## Two pipes sharing a side are at least this far apart [px], when it has room.
const PORT_GAP := 10.0
## A port keeps this far in from its side's corners [px].
const PORT_INSET := 4.0
## Parallel pipes closer than LANE are on top of each other; closer than NEAR, crowded [px].
const LANE := 7.0
const NEAR := 14.0
## A step across this wide or more bends halfway along, else at the far end [px].
const WIDE_STEP := 30.0

## The prices (cost units; a px of pipe is LENGTH of them).
const LENGTH := 0.1
const BEND := 6.0
const THROUGH_NODE := 500.0
const THROUGH_LABEL := 40.0
const THROUGH_OWN_LABEL := 8.0  # its own end's name: out of a node's bottom it must
const ON_TOP := 2.0  # per px run on top of another pipe
const BESIDE := 0.3  # per px run crowded beside one
const CROSSING := 15.0

## The four sides, by the way out of them.
const SIDES := [Vector2(1, 0), Vector2(0, 1), Vector2(-1, 0), Vector2(0, -1)]
## How many times every pipe is taken up and laid again round all the others.
const REROUND := 1


## Every pipe's corners, from its upstream end to its downstream one.
## `boxes`: node id → Rect2 it is drawn in. `labels`: [node id, Rect2] for
## each name and reading a pipe should keep off. `pipes`: [pipe id, from id, to id], in the order
## to route them (the first routed gets first choice of the lanes). `area`:
## where a pipe may run. Returns pipe id → PackedVector2Array [px].
static func route(boxes: Dictionary, labels: Array, pipes: Array, area: Rect2) -> Dictionary:
	# Pass 1: each end's side, its port at the far node's position, each pipe
	# alone (the pipes sharing a side all start from one spot until they are
	# spread, so meeting one another here means nothing yet).
	var sides := {}
	for pipe in pipes:
		var from_box: Rect2 = boxes[pipe[1]]
		var to_box: Rect2 = boxes[pipe[2]]
		var best := _search(pipe, boxes, labels, [], area, _ports(from_box, to_box, -1), _ports(to_box, from_box, -1))
		sides[pipe[0]] = [best[1], best[2]]

	# Pass 2: the ports spread along their sides, each pipe routed round the
	# ones before it.
	var ports := _spread_ports(boxes, pipes, sides)
	var paths := {}
	for pipe in pipes:
		paths[pipe[0]] = _search(pipe, boxes, labels, paths.values(), area, [[ports["%d/0" % pipe[0]], sides[pipe[0]][0]]], [[ports["%d/1" % pipe[0]], sides[pipe[0]][1]]])[0]

	# Then round again, REROUND times: each pipe taken up and laid again round
	# ALL the others, any side free (its own spread port on the side it has,
	# the far node's position on the rest), so a pipe routed early is not stuck
	# with a choice a later one made dear; and the ports spread again for the
	# sides that come out of it.
	for round_index in REROUND:
		var moved := false
		for pipe in pipes:
			var others := _others(pipes, paths, pipe)
			# A pipe that already meets nothing stays as it is.
			if _clash_cost(paths[pipe[0]], _world(pipe, boxes, labels, others), INF) == 0.0:
				continue
			var starts := _ports(boxes[pipe[1]], boxes[pipe[2]], -1)
			var ends := _ports(boxes[pipe[2]], boxes[pipe[1]], -1)
			starts[sides[pipe[0]][0]][0] = ports["%d/0" % pipe[0]]
			ends[sides[pipe[0]][1]][0] = ports["%d/1" % pipe[0]]
			var best := _search(pipe, boxes, labels, others, area, starts, ends)
			paths[pipe[0]] = best[0]
			if best[1] != sides[pipe[0]][0] or best[2] != sides[pipe[0]][1]:
				sides[pipe[0]] = [best[1], best[2]]
				moved = true
		# No side changed: every port is where it was, nothing to lay again.
		if not moved:
			break
		ports = _spread_ports(boxes, pipes, sides)
		for pipe in pipes:
			paths[pipe[0]] = _search(pipe, boxes, labels, _others(pipes, paths, pipe), area, [[ports["%d/0" % pipe[0]], sides[pipe[0]][0]]], [[ports["%d/1" % pipe[0]], sides[pipe[0]][1]]])[0]
	return paths


## Every pipe's path but `pipe`'s own.
static func _others(pipes: Array, paths: Dictionary, pipe: Array) -> Array:
	var out: Array = []
	for other in pipes:
		if other[0] != pipe[0]:
			out.append(paths[other[0]])
	return out


## Each pipe end's port ("<pipe id>/<0 upstream, 1 downstream>" → point): the
## pipes that share a node's side spread along it in the order their far ends
## sit, each as near where it would like as PORT_GAP allows.
static func _spread_ports(boxes: Dictionary, pipes: Array, sides: Dictionary) -> Dictionary:
	var shared := {}
	for pipe in pipes:
		for end in 2:
			var node: int = pipe[1 + end]
			var other: int = pipe[2 - end]
			var side: int = sides[pipe[0]][end]
			var key := "%d/%d" % [node, side]
			if not shared.has(key):
				shared[key] = []
			shared[key].append([_across(boxes[other].get_center(), side), pipe[0], end, _wanted(boxes[node], boxes[other], side)])
	var ports := {}
	for key in shared:
		var parts: PackedStringArray = str(key).split("/")
		var box: Rect2 = boxes[int(parts[0])]
		var side := int(parts[1])
		var wanted: Array = shared[key]
		wanted.sort()
		var low := (box.position.y if side % 2 == 0 else box.position.x) + PORT_INSET
		var high := (box.end.y if side % 2 == 0 else box.end.x) - PORT_INSET
		var places := _spread(wanted.map(func(w): return w[3]), low, high)
		for i in wanted.size():
			ports["%d/%d" % [wanted[i][1], wanted[i][2]]] = _port(box, side, places[i])
	return ports


## A quick path while a node is dragged (routing every frame would stall it):
## out of the side facing the other end, one upright run halfway, in.
static func sketch(from_box: Rect2, to_box: Rect2) -> PackedVector2Array:
	var a := from_box.get_center()
	var b := to_box.get_center()
	var horizontal := absf(b.x - a.x) >= absf(b.y - a.y)
	if horizontal:
		var mid := (a.x + b.x) * 0.5
		return _simplified(PackedVector2Array([a, Vector2(mid, a.y), Vector2(mid, b.y), b]))
	var mid := (a.y + b.y) * 0.5
	return _simplified(PackedVector2Array([a, Vector2(a.x, mid), Vector2(b.x, mid), b]))


## The middle of `path`'s longest straight run (the first, of equals), and the
## way the path runs there (a unit vector; zero for a path with no length).
## Where the map puts a pipe's arrow, its number and a leak's spray: always on
## the drawn line, never on a short step of it.
static func middle(path: PackedVector2Array) -> Array:
	var best := -1
	var longest := 0.0
	for i in range(1, path.size()):
		var length := path[i - 1].distance_to(path[i])
		if length > longest:
			longest = length
			best = i
	if best < 0:
		return [path[0] if path.size() > 0 else Vector2.ZERO, Vector2.ZERO]
	return [(path[best - 1] + path[best]) * 0.5, (path[best] - path[best - 1]) / longest]


# ------------------------------------------------------------------ inside

## The ports a pipe end may use: [point, side] for each of the four sides
## (`side` < 0) or the one given, each at the far box's position, kept on the side.
static func _ports(box: Rect2, far: Rect2, side: int) -> Array:
	var out: Array = []
	for s in 4:
		if side >= 0 and s != side:
			continue
		out.append([_port(box, s, _wanted(box, far, s)), s])
	return out


## Where on `box`'s `side` a pipe to `far` would like its port. Where the two
## boxes face each other along that side, the middle of that stretch — the far
## end works out the same point, so the pipe runs straight; else as near the
## far box's middle as the side allows.
static func _wanted(box: Rect2, far: Rect2, side: int) -> float:
	var low := (box.position.y if side % 2 == 0 else box.position.x) + PORT_INSET
	var high := (box.end.y if side % 2 == 0 else box.end.x) - PORT_INSET
	var far_low := (far.position.y if side % 2 == 0 else far.position.x) + PORT_INSET
	var far_high := (far.end.y if side % 2 == 0 else far.end.x) - PORT_INSET
	if maxf(low, far_low) <= minf(high, far_high):
		return (maxf(low, far_low) + minf(high, far_high)) * 0.5
	return clampf(_across(far.get_center(), side), low, high)


## Where along a side a point sits: its y for the left and right sides, its x
## for the top and bottom.
static func _across(point: Vector2, side: int) -> float:
	return point.y if side % 2 == 0 else point.x


## The point on `box`'s `side` at `along` (a y on the left and right sides, an x
## on the top and bottom).
static func _port(box: Rect2, side: int, along: float) -> Vector2:
	match side:
		0:
			return Vector2(box.end.x, along)
		1:
			return Vector2(along, box.end.y)
		2:
			return Vector2(box.position.x, along)
	return Vector2(along, box.position.y)


## `wanted` positions (sorted) moved apart to at least PORT_GAP, inside
## [low, high]; evenly spread when the side is too short for that.
static func _spread(wanted: Array, low: float, high: float) -> Array:
	var count := wanted.size()
	if count == 1:
		return wanted
	if (count - 1) * PORT_GAP > high - low:
		var even: Array = []
		for i in count:
			even.append(lerpf(low, high, i / (count - 1.0)))
		return even
	var placed: Array = wanted.duplicate()
	for i in range(1, count):
		placed[i] = maxf(placed[i], placed[i - 1] + PORT_GAP)
	placed[count - 1] = minf(placed[count - 1], high)
	for i in range(count - 2, -1, -1):
		placed[i] = minf(placed[i], placed[i + 1] - PORT_GAP)
	return placed


## The cheapest path for `pipe` from any of `starts` to any of `ends` ([point,
## side] each). Returns [path, start side, end side].
static func _search(pipe: Array, boxes: Dictionary, labels: Array, routed: Array, area: Rect2, starts: Array, ends: Array) -> Array:
	var candidates: Array = []
	var order := 0
	for start in starts:
		var p: Vector2 = start[0]
		var q: Vector2 = p + SIDES[start[1]] * STUB
		for end in ends:
			var t: Vector2 = end[0]
			var r: Vector2 = t + SIDES[end[1]] * STUB
			# A Z costs the same wherever its middle run is. A wide step, halfway
			# is tried first so it wins the tie; a small step (a few px between
			# two nearly level nodes) reads better as a bend at the far end,
			# not a kink halfway along.
			var shapes: Array = [[]]
			if absf(q.y - r.y) >= WIDE_STEP:
				shapes.append([Vector2((q.x + r.x) * 0.5, q.y), Vector2((q.x + r.x) * 0.5, r.y)])
			if absf(q.x - r.x) >= WIDE_STEP:
				shapes.append([Vector2(q.x, (q.y + r.y) * 0.5), Vector2(r.x, (q.y + r.y) * 0.5)])
			shapes.append([Vector2(r.x, q.y)])
			shapes.append([Vector2(q.x, r.y)])
			for x in _channels(q.x, r.x, area.position.x, area.end.x):
				shapes.append([Vector2(x, q.y), Vector2(x, r.y)])
			for y in _channels(q.y, r.y, area.position.y, area.end.y):
				shapes.append([Vector2(q.x, y), Vector2(r.x, y)])
			for shape in shapes:
				if shape.is_empty() and q.x != r.x and q.y != r.y:
					continue
				var points: Array = [p, q]
				points.append_array(shape)
				points.append(r)
				points.append(t)
				# Priced from its corners; the path itself built only for the
				# few that get as far as their clashes being counted.
				var plain := _measure(points, area)
				if plain < INF:
					candidates.append([plain, order, points, start[1], end[1]])
				order += 1
	candidates.sort()
	var world := _world(pipe, boxes, labels, routed)
	var best: Array = []
	var best_cost := INF
	for candidate in candidates:
		if candidate[0] >= best_cost:
			break
		var path := _simplified(PackedVector2Array(candidate[2]))
		var cost: float = candidate[0] + _clash_cost(path, world, best_cost - candidate[0])
		if cost < best_cost:
			best_cost = cost
			best = [path, candidate[3], candidate[4]]
	if best.is_empty():
		# Nothing valid (both ends off the page): straight, as the map once drew.
		var a: Rect2 = boxes[pipe[1]]
		var b: Rect2 = boxes[pipe[2]]
		return [PackedVector2Array([a.get_center(), b.get_center()]), 0, 2]
	return best


## The channel positions between and around `a` and `b`, every GRID px, on the
## page; plus the two ends and their middle.
static func _channels(a: float, b: float, low: float, high: float) -> Array:
	var out: Array = [a, b, (a + b) * 0.5]
	var first := ceilf(maxf(minf(a, b) - SWING, low) / GRID) * GRID
	var last := minf(maxf(a, b) + SWING, high)
	var x := first
	while x <= last:
		out.append(x)
		x += GRID
	return out


## `points` without repeats and without corners that do not turn.
static func _simplified(points: PackedVector2Array) -> PackedVector2Array:
	var out := PackedVector2Array()
	for point in points:
		if out.size() > 0 and out[out.size() - 1].is_equal_approx(point):
			continue
		if out.size() >= 2:
			var a: Vector2 = out[out.size() - 2]
			var b: Vector2 = out[out.size() - 1]
			if (is_equal_approx(a.x, b.x) and is_equal_approx(b.x, point.x)) or (is_equal_approx(a.y, b.y) and is_equal_approx(b.y, point.y)):
				out[out.size() - 1] = point
				continue
		out.append(point)
	return out


## The price of length and bends of the path through `points` (each run level
## or upright); INF for one off the page or doubling back on itself (out of a
## stub and straight back past the node, or into the far node the wrong way).
static func _measure(points: Array, area: Rect2) -> float:
	var page := area.grow(1.0)
	var length := 0.0
	var bends := 0
	var heading := Vector2.ZERO
	for i in points.size():
		var point: Vector2 = points[i]
		if not page.has_point(point):
			return INF
		if i == 0:
			continue
		var run: Vector2 = point - points[i - 1]
		var run_length := absf(run.x) + absf(run.y)
		if run_length < 0.001:
			continue
		var way := run / run_length
		if heading != Vector2.ZERO:
			var turn := heading.dot(way)
			if turn < -0.5:
				return INF
			if turn < 0.5:
				bends += 1
		heading = way
		length += run_length
	return length * LENGTH + bends * BEND


## What a path for `pipe` can run into, as flat lists of numbers, worked out
## once per search (the candidates it prices run to thousands):
##   "boxes":  x0, y0, x1, y1 of every other node, grown by MARGIN
##   "from", "to": its own two ends' boxes, [just inside, grown by MARGIN] —
##             the run out of one and the run into the other may only not go
##             through the inside (the ports sit on the edge); any other run
##             keeps clear of them too, not skirting along an edge
##   "labels": x0, y0, x1, y1, price of every name and reading
##   "runs":   level (1) or upright (0), the fixed coordinate, low, high — of
##             every run of the pipes already laid
static func _world(pipe: Array, boxes: Dictionary, labels: Array, routed: Array) -> Dictionary:
	var others := PackedFloat32Array()
	for id in boxes:
		if id == pipe[1] or id == pipe[2]:
			continue
		var box: Rect2 = boxes[id].grow(MARGIN)
		others.append_array([box.position.x, box.position.y, box.end.x, box.end.y])
	var marks := PackedFloat32Array()
	for label in labels:
		var box: Rect2 = label[1]
		marks.append_array([box.position.x, box.position.y, box.end.x, box.end.y, THROUGH_OWN_LABEL if label[0] == pipe[1] or label[0] == pipe[2] else THROUGH_LABEL])
	var runs := PackedFloat32Array()
	for other in routed:
		for j in range(1, other.size()):
			var c: Vector2 = other[j - 1]
			var d: Vector2 = other[j]
			if is_equal_approx(c.y, d.y):
				runs.append_array([1.0, c.y, minf(c.x, d.x), maxf(c.x, d.x)])
			else:
				runs.append_array([0.0, c.x, minf(c.y, d.y), maxf(c.y, d.y)])
	var ends: Array = []
	for id in [pipe[1], pipe[2]]:
		var box: Rect2 = boxes[id]
		ends.append([box.grow(-1.0), box.grow(MARGIN)])
	return {"boxes": others, "from": ends[0], "to": ends[1], "labels": marks, "runs": runs}


## What the path runs through and along (`world`, from `_world`); stops adding
## once past `enough`.
static func _clash_cost(path: PackedVector2Array, world: Dictionary, enough: float) -> float:
	var others: PackedFloat32Array = world["boxes"]
	var marks: PackedFloat32Array = world["labels"]
	var runs: PackedFloat32Array = world["runs"]
	var from_end: Array = world["from"]
	var to_end: Array = world["to"]
	var cost := 0.0
	var last := path.size() - 1
	for i in range(1, path.size()):
		var a: Vector2 = path[i - 1]
		var b: Vector2 = path[i]
		var x0 := minf(a.x, b.x)
		var x1 := maxf(a.x, b.x)
		var y0 := minf(a.y, b.y)
		var y1 := maxf(a.y, b.y)
		var k := 0
		while k < others.size():
			if x0 < others[k + 2] and x1 > others[k] and y0 < others[k + 3] and y1 > others[k + 1]:
				cost += THROUGH_NODE
			k += 4
		for end in 2:
			var box: Rect2 = (from_end[0] if i == 1 else from_end[1]) if end == 0 else (to_end[0] if i == last else to_end[1])
			if x0 < box.end.x and x1 > box.position.x and y0 < box.end.y and y1 > box.position.y:
				cost += THROUGH_NODE
		k = 0
		while k < marks.size():
			if x0 < marks[k + 2] and x1 > marks[k] and y0 < marks[k + 3] and y1 > marks[k + 1]:
				cost += marks[k + 4]
			k += 5
		# Meeting the pipes already laid: parallel, on top of one or crowding
		# it along their overlap; across, each passing the other's line
		# strictly inside its length.
		var level := is_equal_approx(a.y, b.y)
		var fixed := a.y if level else a.x
		var low := x0 if level else y0
		var high := x1 if level else y1
		k = 0
		while k < runs.size():
			if (runs[k] == 1.0) == level:
				var gap := absf(fixed - runs[k + 1])
				if gap < NEAR:
					var overlap := minf(high, runs[k + 3]) - maxf(low, runs[k + 2])
					if overlap > 0.0:
						cost += overlap * (ON_TOP if gap < LANE else BESIDE)
			elif runs[k + 1] > low and runs[k + 1] < high and fixed > runs[k + 2] and fixed < runs[k + 3]:
				cost += CROSSING
			k += 4
		if cost >= enough:
			return cost
	return cost
