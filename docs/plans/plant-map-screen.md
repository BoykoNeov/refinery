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

## Slice 1 — hover and inspect (no decision)

- [ ] Mouse over a node or a pipe: outline it and show a panel at the right
      edge (x ≥ 870, the page is 1152 wide) with its name, kind and every
      scalar field the snapshot gives it — pressure in bar, temperature in
      °C, flows in kg/s, duties in MW. Walk the node's dictionary
      generically (skip arrays longer than 6 and nested compositions; show
      a composition as the three largest cuts by name from `snapshot.slate`).
- [ ] Shrink `AREA` to leave the panel its width. Re-run `--auto` and four
      screenshots (`--plant=crude_unit_loops`, `hydrocracker_hydrogen_quench`,
      `tank_level_fill_control`, `fcc_plant`) and LOOK at them.

## Slice 2 — hands (no decision)

The commands are the engine's own JSON (`crates/core/src/snapshot.rs`,
`enum Command`; examples in every `demo/*.gd` `_send`).

- [ ] Click a **valve**: select it; Up/Down move its opening by 5 %
      (`set_valve_opening`). Click a **pump**: toggle it (`set_pump_on`).
      Click a **furnace**: Up/Down its duty by 5 % of its current value, at
      least 0.5 MW (`set_furnace_duty`). Click a **pipe** with P held:
      puncture it 0.001 m² / repair it (`puncture_pipe`, area 0 repairs).
- [ ] Every refusal is shown in the engine's own words on the message line
      (bottom, red), every acceptance in green — as the other screens do.
      Do not pre-check anything in the script: send, and show what came back.
- [ ] If `failed-tick-recovery.md` slice 1 has landed, a failed tick calls
      `recover()`; if not, it halts as now.
- [ ] Add the control loops and trips as two lists in the panel when nothing
      is hovered (the snapshot's `controls` and `trips`, the way
      `demo/reactor.gd` `_draw` lists them): Tab picks a loop, A toggles
      auto/manual, Q/Z move its target; R resets trips.

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
