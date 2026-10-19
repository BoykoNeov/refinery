# M70.1 — the measurements behind DESIGN §75.3

Every timing and answer move in §75.3 came from these, on 2026-10-19, a
machine loaded by other work (read ratios, not milliseconds).

- `driver/` — `m70drv`, M70.0's `m70_0_step` without the probe's log: a plant
  held from its file, its furnace stepped (`<hold> <MW> <after>`), or `COLD=1`
  the cold start-up of `crude_unit_cold_reference.rs`, or `HELD=1` no command
  at all. A CSV row per tick (wall µs, re-solves, the hardest solve's
  iterations, its residual) in `OUT/<label>.csv`; `SNAP=<tick,…>` adds those
  ticks' snapshots as JSON lines. `Cargo.toml` points at the repo's crates; a
  copy pointed at a worktree of the commit before (0335994 for §75.3) is the
  "head" build.
- `time.sh` + `fastest.py` — the stripped plant 58 → 65 MW and the cold
  start-up, head and new alternated, three rounds; fastest of three per tick,
  medians by window.
- `held.sh` — six plants held 2 000 ticks, head and new alternated.
- `split.sh` + `split.py` — the held boiling and stripped plants by window
  (100–1 000, 1 000–2 000), with iterations and residuals: the numbers that
  showed the extra step bought nothing on the crude units (residual ~2e-12
  kg/s before it) and led to `BALANCED_BAR`.
- `answers.py` — two CLIs' `run` output compared field by field (the moves
  held from their files); the key-press moves used the driver's `SNAP=` at
  the ticks listed in §75.3 and the same `walk`.

The scripts name `W:/temp/claude/m70_1/` (the drivers' builds, `target_new`,
`target_head`, `target_cli_head`, and the output folders), removed at M70's
close. To re-run: two copies of `driver/` (one per engine), each built
`--release` into its own target folder, and the paths in the scripts set to
them.
