# M70.2 — the stripped and cold tables re-checked on the M70.1 engine

DESIGN §75.4. The engine of commit 4389813 (M70.1), in a clean worktree,
through `../m67_3/m67_3_grid.rs` and `../m67_3/run.sh`, 4 runs at a time,
below-normal priority, on M69.3's own specs (`../m69_3/*.specs`, unchanged).
"Old" is M69.3's committed output for the same spec (`../m69_3/*.out`, the
M69.2 engine); `compare.py` matches them spec by spec, verdict and failing tick.

**Why again**: held from their files M70.1 moves the crude units at most
4.4e-9, but after a key press up to 1.6e-5 (the cold start-up's residue
cooler, 10 Pa of 6.4 bar) and 4.7e-6 (the stripped plant after 58 → 45 MW) —
larger than the 6.5e-7 M69.3's edge check covered (§75.3). The user's decision,
asked with those numbers.

| batch | what | wall | old | new |
|---|---|---|---|---|
| 1 | stripped edge, inside | 7 min (both sides) | 149 / 149 run | 149 / 149 run |
| 1 | stripped edge, outside | | 1 / 122 run | 1 / 122 run |
| 1 | cold edge, inside | 11 min (both sides) | 160 / 160 run | 160 / 160 run |
| 1 | cold edge, outside | | 134 / 140 run | 134 / 140 run |
| 2 | cold 10 000-tick rows of §72.12 (`cold_long`) | 9 min (with the presses) | 12 / 12 run | 12 / 12 run |
| 2 | cold held presses (`cold_held`, the 23 that set `held_limits`) | | 12 / 23 run | 12 / 23 run |
| 2 | every allowed cold 10 000-tick row (`cold_rows`) | 35 min | 84 / 84 run | 84 / 84 run |
| — | the stripped coils freed 15 000 ticks (`str_coils`) | a few min | 384.3, 413.9 °C | 384.3, 413.9 °C |

**Every spec's verdict is the old one, and every failing spec fails on the same
tick** (`compare.txt`: no line differs). About 65 minutes of machine time, the
two batches 18 and 44 minutes. The first `str_coils` run used `rest.sh`'s
`free` (no `=1`), on which the driver panics — M69.3's later `longer.sh` form was
re-run by hand; `str_coils.out` is that run.

Nothing on the screen moves: both tables stand as committed, `column_screen.rs`
unchanged.

## To re-run

The scripts name `W:/temp/claude/refinery-m70-2` (a worktree of 4389813) and
the output folder `W:/temp/claude/m70-2`, both removed at M70's close. To
rebuild: a worktree of the commit at that path; `../m67_3/m67_3_grid.rs` copied
into its `crates/godot-ext/tests/` and registered at the end of that folder's
`suite.rs` (`#[path = "m67_3_grid.rs"] mod m67_3_grid;`); `cargo test --release
-p refinery-godot-ext --test suite --no-run`; `../m69_3/*.specs` copied to the
output folder; then `band.sh stripped`, `band.sh cold`, `rest.sh`, `rows.sh`.
