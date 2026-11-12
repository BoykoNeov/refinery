# Plan: one kit under every screen

## The problem

`demo/column.gd` (2 100 lines), `reactor.gd` (1 800), `furnace.gd` (1 100),
`pump.gd` (1 000), `plant.gd`, `map.gd` and `menu.gd` each carry their own
copy of: the colour constants, `_text`, `_halt`, the argument parsing
(`--auto`, `--plant=`, `--shots=`), the load → tick → snapshot → timeline →
screenshot loop, pause and speed keys, the message line, the key legend.
A fix to one (text running off the window, fixed in `column.gd` only on
2026-11-12) does not reach the others.

`demo/screen.gd` exists and holds the first shared pieces (menu round trip,
window fitting, `fitting_size`). This plan grows it.

## Rules

- **Behaviour must not change.** Each screen's `--auto` run prints the same
  lines before and after, to the character. That is the test: record each
  screen's `--auto` output for every plant before starting
  (CLAUDE.md lists the commands), and diff after every step.
- The Rust tests `crates/godot-ext/tests/{column,furnace,pump,reactor}_screen.rs`
  replay the screens' timelines from Rust; they do not read the `.gd` files,
  so they neither help nor hinder. `column_screen.rs` reads
  `demo/column_region_*.json` — do not move those.
- `screen.gd` stays a preloaded script with static functions and no
  `class_name` (its header says why).
- One screen per commit. Start with the smallest (`pump.gd`), end with
  `column.gd`.

## Steps

- [ ] **0. Record.** For every screen and plant: the headless `--auto`
      output to `W:\temp\claude\screen-kit\before\<screen>_<plant>.txt`, and
      a windowed `--shots` run. Strip the `[x.x ms/tick]` timings before
      diffing (they vary run to run).
- [ ] **1. Palette.** Move the colour constants every screen shares
      (`BACKGROUND`, `SHELL`, `PIPE`, `COOL`, `HOT`, `FLAME`, `INK`, `DIM`,
      `GOOD`, `BAD`, `WARN`) into `screen.gd` as constants; each screen
      keeps `const INK := Screen.INK` lines so its drawing code is untouched.
      Check first that the values really are equal across screens
      (`grep -n '^const \(INK\|DIM\|…\) ' demo/*.gd`); where one differs,
      keep that screen's own and note it.
- [ ] **2. Text.** One `Screen.text(canvas, at, text, color, size, width)`
      that shrinks to fit (what `column.gd`'s `_text` does now). Every
      screen's `_text` becomes a one-line call to it. Look at every
      screenshot: nothing should move, and anything that ran off the edge
      before now fits.
- [ ] **3. Arguments.** `Screen.arguments() -> Dictionary` returning
      `{auto, plant, shots}`; each `_ready` uses it.
- [ ] **4. Screenshot.** `Screen.shoot(node, path)` (the
      `await RenderingServer.frame_post_draw` / `save_png` block, four
      copies today).
- [ ] **5. Legends.** `Screen.legend(canvas, lines, max_width)` drawing the
      one or two key-legend lines at the bottom, fitted, with "Esc menu"
      appended — so a new key is added in one place per screen and never
      overlaps a side panel.
- [ ] **6. Stop here** and report the line counts before and after. The
      tick loop and timelines differ enough between screens (walks,
      trails, charts) that sharing them is a redesign, not a tidy-up; do
      not attempt it under this plan.

## Then, new shared visuals (each its own commit, each LOOKED at)

- [ ] **Flow dashes.** `Screen.pipe(canvas, points, flow_share, color,
      phase)`: the polyline plus short dashes that advance along it by
      `phase` (seconds of wall time × a speed set by the flow's share). Use
      it in `map.gd` first.
- [ ] **A trend chart.** Lift the reactor screen's chart
      (`demo/reactor.gd`, the "temperature, last 30 minutes" block and its
      `samples` array) into `Screen.chart(canvas, rect, series, …)`; the
      reactor uses it unchanged; give the column screen's holdup plant a
      chart of its drum and base levels (it is the plant whose answers take
      minutes).

## Done when

The six scripts share one palette, one text function, one argument parser,
one screenshot function and one legend; every `--auto` output is unchanged;
and the next screen someone writes starts from the kit.
