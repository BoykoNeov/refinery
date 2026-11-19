# Plan: the plant map, from a picture to a place to play

## What exists

`demo/map.gd` (2026-11-12) draws any shipped plant from its snapshot: nodes
laid out left to right by how many pipes the liquid crosses from a supply,
each drawn by its kind, pipes as wide as their flow and coloured by their
temperature. `godot --headless --path . res://demo/map.tscn -- --auto` loads
all 71 plants and exits nonzero if one stops. It watches only.

Read its header comment and `_lay_out` before changing anything. Its rule,
like every screen's: **it computes nothing physical**; every number drawn is
a snapshot field.

Known weak spots, seen in screenshots:

- A column's products and pumparound pumps share one layout column and their
  pipes cross (`crude_unit_loops`). Readable, not pretty.
- A column and a reactor are one box each.
- Kinds the map has no shape for are a square with the kind's name
  (`reactor` beds, compressors appear this way).
- Pipes into one node all end at its centre.

## Slice 1 — hover and inspect (no decision) — DONE 2026-11-15 (M78)

- [x] Mouse over a node or a pipe: outline it and show a panel at the right
      edge (x ≥ 870, the page is 1152 wide) with its name, kind and every
      scalar field the snapshot gives it — pressure in bar, temperature in
      °C, flows in kg/s, duties in MW. Walk the node's dictionary
      generically (skip arrays longer than 6 and nested compositions; show
      a composition as the three largest cuts by name from `snapshot.slate`).
- [x] Shrink `AREA` to leave the panel its width. Re-run `--auto` and four
      screenshots (`--plant=crude_unit_loops`, `hydrocracker_hydrogen_quench`,
      `tank_level_fill_control`, `fcc_plant`) and LOOK at them.

**As built** (`demo/map.gd`):
- A unit is converted only where it is known: a name ending `_pa`, `_k`,
  `_w`, `_kg_s`, `_s`, `_m`, or a bare name whose Rust type was checked
  (`pressure`, `duty`, `t_set`, `tau`, …, the lists at the top of the file).
  Any other field is shown as it came — a wrong unit is worse than none.
- Nested fields are indented under a heading, not dotted names; nesting
  deeper than four names is left out (a column's holdup keeps its whole
  working state under `kind`).
- **Added**: the mouse wheel scrolls the panel. The overfilling column
  reports ~200 lines; without scrolling most were unreachable.
- **Added**: `--hover=<name>` and `--scroll=<lines>` for recorded
  screenshots. They send the item's drawn position through the same hit
  test the mouse uses. `--auto` also builds every node's and pipe's panel on
  all 72 plants (its printed lines are unchanged).
- **`AREA` 1012 → 735 px wide**: 640 first, and the hydrocracker's
  names collided. A crowded right-most column would step its every second
  node LEFT, not right into the panel. No shipped plant has one (checked
  2026-11-15), so this is a guard for plant files to come and has not been
  seen on screen. The panel draws from x = 870, its text from 880.
- With the mouse over the panel, the item stays selected, so the wheel works.
- A pipe's `stream.pressure` is left out: every pipe reports atmospheric
  pressure there (DEFERRED F10).

## Slice 2 — hands (no decision) — DONE 2026-11-19 (M79)

The commands are the engine's own JSON (`crates/core/src/snapshot.rs`,
`enum Command`; examples in every `demo/*.gd` `_send`).

- [x] Click a **valve**: select it; Up/Down move its opening by 5 %
      (`set_valve_opening`). Click a **pump**: toggle it (`set_pump_on`).
      Click a **furnace**: Up/Down its duty by 5 % of its current value, at
      least 0.5 MW (`set_furnace_duty`). Click a **pipe** with P held:
      puncture it 0.001 m² / repair it (`puncture_pipe`, area 0 repairs).
- [x] Every refusal is shown in the engine's own words on the message line
      (bottom, red), every acceptance in green — as the other screens do.
      Do not pre-check anything in the script: send, and show what came back.
- [x] If `failed-tick-recovery.md` slice 1 has landed, a failed tick calls
      `recover()`; if not, it halts as now.
- [x] Add the control loops and trips as two lists in the panel when nothing
      is hovered (the snapshot's `controls` and `trips`, the way
      `demo/reactor.gd` `_draw` lists them): Tab picks a loop, A toggles
      auto/manual, Q/Z move its target; R resets trips.

**As built** (`demo/map.gd`, commit f015b43):
- A picked valve or furnace has a yellow outline; the hover's sits inside it.
  A valve steps on the 5 % grid, sent as it falls (105 % is the engine's to
  refuse). A furnace step down that would cross zero lands on zero, so it
  can be put out.
- **P+click decides hole or mend from the map's own record** of the holes it
  made: the snapshot reports a hole's flow, never its size, and a hole with
  no pressure across it flows nothing. A leaking pipe the map did not hole
  is mended too. A recovery forgets the holes it undid.
- **A click on a punctureable pipe's downstream half means the same pipe**:
  the loader splits it at `<pipe>__leak_point`, and only the upstream half
  takes `puncture_pipe`; without this the visible half answered "declares
  no leak path".
- The message line is two lines at the bottom (a recovery says what it
  undid and why); the pipes' key moved up beside the headings.
- Q/Z steps: temperature 5 °C, level 0.25 m, pressure 0.1 bar, flow 5 %
  (at least 0.25 kg/s); the panel's lists show each loop's target, last
  measurement, output and cascade link, each trip's watched point, limit,
  measurement and state.
- **Added**: `--hands [--hands-at=<tick>] [--break-at=<tick>] --plant=<stem>`
  tours a plant through the same calls the mouse and keys make and prints
  every answer; `--break-at` sets the first supply to −270 °C (the pump
  screen's break) to see the map recover.
- **Seen**: on most plants with loops the first Up/Down is refused — the
  valve or furnace is a loop's actuator in AUTO; A (or Q/Z) is the way. The
  snapshot does not say which loop drives which actuator, so the map cannot
  offer "take this one over" on the picked item without an engine field.

## Slice 3 — a layout a person can fix (no decision)

Computed layout will never be right for every plant. Give each plant an
optional sidecar: `demo/layout/<plant>.json`, `{"node_name": [x, y], …}` in
page coordinates. **Not** in the scenario file (those never change:
CLAUDE.md, Conventions).

- [ ] `_lay_out` uses the sidecar's position for any node it names and the
      computed one for the rest.
- [ ] In the map: hold Ctrl and drag a node to move it; Ctrl+S writes the
      sidecar (only when run from the project folder, i.e.
      `OS.has_feature("editor")` or the path is writable; otherwise say so).
- [ ] Hand-place the eight column-screen plants and the six reactor plants.
      Commit the sidecars. Screenshots before/after in the commit message.

## Slice 4 — one running plant, many screens (ASK FIRST)

Today each scene owns its own `RefinerySim` and loads its own file, so going
from the map to the column screen restarts the plant. The game in
`docs/GAME.md` §3.5 wants the map as home and a unit's screen as a closer
look at the **same running plant**.

That means the simulator lives above the scenes (an autoload, or the scenes
become children of one root that owns it). It changes every screen's
`_ready` and `_load`. **Ask the user before starting**, offering: (a) one
shared simulator, the hand-drawn screens open on whatever is running and
refuse a plant they cannot draw; (b) keep them separate, the map stays the
only general screen. Recommend (a) after `screen-kit.md` has landed, not
before — it touches the same lines.

## Later, not planned here

Moving dashes on pipes, an alarm strip, trends: `docs/GAME.md` §3, and they
belong in the shared kit (`screen-kit.md`) so every screen gets them.

## Verification for every slice

```
godot --headless --path . res://demo/map.tscn -- --auto        # 71 plants, 0 stopped
godot --path . res://demo/map.tscn --quit-after 2000 -- --plant=<stem> --shot=W:/temp/claude/<task>/<stem>.png
```

No Rust changes in slices 1–3, so no cargo gates beyond `cargo fmt --check`
being untouched; say so in the commit.
