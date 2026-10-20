# M71.2 — the stripped and cold tables re-checked on the M71.1 engine

DESIGN §76.3. The engine of commit 891be64 (M71.1: a column's draws guessed,
a failed guess dropped), in a clean worktree, through `../m67_3/m67_3_grid.rs`
and `../m67_3/run.sh`, 4 runs at a time, below-normal priority, on M69.3's
specs (`../m69_3/*.specs`, unchanged). "Old" is M70.2's committed output for
the same spec (`../m70_2/*.out`, the M70.1 engine); `compare.py` matches them
spec by spec, verdict and failing tick.

**Why again** (the user's decision): the draws are guessed where the tables
live — floods, cut-back draws, the cold warm-up — a key press moves answers
up to 1.6e-5 against M70.1 (§76.2), and the fallback could let a setting that
failed by a guess run.

| batch | what | wall | old | new |
|---|---|---|---|---|
| 1 | stripped edge, inside | 5 min (both sides) | 149 / 149 run | 149 / 149 run |
| 1 | stripped edge, outside | | 1 / 122 run | 1 / 122 run |
| 1 | cold edge, inside | 9 min (both sides) | 160 / 160 run | 160 / 160 run |
| 1 | cold edge, outside | | 134 / 140 run | 134 / 140 run |
| 2 | cold 10 000-tick rows of §72.12 (`cold_long`) | 5 min (with the presses and coils) | 12 / 12 run | 12 / 12 run |
| 2 | cold held presses (`cold_held`) | | 12 / 23 run | 12 / 23 run |
| 2 | the stripped coils freed 15 000 ticks (`str_coils`) | | 384.3, 413.9 °C | 384.3, 413.9 °C |
| 2 | every allowed cold 10 000-tick row (`cold_rows`) | 12 min | 84 / 84 run | 84 / 84 run |

**Every spec's verdict is the old one, and every failing spec fails on the same
tick** (`compare.txt`: no line differs). About 31 minutes of machine time, the
batches 14 and 17 minutes. No setting outside an edge flipped to running.

Both tables stand as committed; `column_screen.rs` unchanged.

## To re-run

The scripts name `W:/temp/claude/refinery-m71-2` (a worktree of 891be64) and
the output folder `W:/temp/claude/m71-2`, both removed at M71's close. Rebuild
as `../m70_2/FINDINGS.md` says, at those paths; then `band.sh stripped`,
`band.sh cold`, `rest.sh`, `rows.sh`.
