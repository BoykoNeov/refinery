# M69.3 — the stripped and cold tables re-checked on the predicted engine

DESIGN §74.6. The engine of commit 0abbca7 (M69.2), in a clean worktree, through
`../m67_3/m67_3_grid.rs` and `../m67_3/run.sh`, 4 runs at a time, below-normal
priority. "Old" is commit 9f66416 (M69.1: the memo, bit for bit the engine the
tables were measured on).

**An edge check, not a full re-run.** The full grids cost about 2 hours on 12
threads (§72.9, §72.10, §72.12); on 4, past 3. The predictor moves answers at
most 6.5e-7, so only a cell already on an edge can flip. `../m67_3/edges.py`
lists each committed region's band: every allowed cell with a disallowed
neighbour (inside) and every disallowed one with an allowed neighbour
(outside), each as one jump from its furnace's settled file, 20 ticks, the
move, 1 500 ticks (§72.12's second-round length).

1. **The stripped plant** (`band.sh stripped`, 11 minutes): inside 149 of 149
   run; outside 121 of 122 fail. The one that runs, 47.5 MW, steam 80 %,
   pumparounds 12 MW, runs on the old engine too, to every printed digit
   (`old_one.out`): a route closed it, not the predictor. Unchanged.

2. **The cold plant free** (`band.sh cold`, 28 minutes): inside 160 of 160 run;
   outside 6 of 140 fail at the move. The other 134 are the late edge — no
   pumparounds from 50 MW, a quarter at 65 MW — closed in §72.12 for failing
   7 400–8 400 ticks on, which 1 500 ticks cannot judge.

3. **The late edge is gone** (`rest.sh`, `diff.sh`, `longer.sh`): the 10 000-tick
   rows of §72.12 at three steam settings (no pumparounds at 47.5 and 50 MW, a
   quarter at 62.5 and 65 MW) all run on the new engine, and three of them run
   20 000 ticks to the same reading as at 10 000. On the old engine the same
   specs fail as recorded: 7 440 ticks (50 MW, full steam, none), 8 416 (65 MW,
   full steam, a quarter — §72.12's number exactly), 7 581 (52.5 MW, 40 %,
   none).

4. **The held cold plant** (`cold_held.specs`, 23 of §72.10's 63 presses, the
   ones that set `held_limits`): the steam down fails exactly where recorded
   (one step at 650 and 750; four through 850; seven through 1 000; all run at
   1 500); the furnace's eight steps run; the pumparounds to 50 % run; to 100 %
   fail on the press at 1 000 and 2 000. One differs: the pumparounds to 100 %
   at 550 now run 2 700 ticks (old: fail 1 228 ticks on, `diff_held_*.out`).
   The limit is the same at its other ticks, so it stands.

5. **No allowed long row fails** (`rows.sh`, `cold_rows.out`, 19 minutes):
   finding 3 could be a failure moved rather than gone, so every 10 000-tick
   row the table allows was run — no pumparounds at 20–45 MW, a quarter at
   20–60 MW (62.5 in finding 3), steam 5, 40 and 100 %: 84 of 84 run.

6. **The coils** (`str_coils.out`, the stripped plant freed 15 000 ticks): 384.3
   °C at 40 MW and 413.9 at 50, `str_coils.txt`'s values.

**Decided** (the user, 2026-10-19, asked with these numbers): keep both tables
as committed. The cold one now holds back ~134 settings that run, the safe
side; recorded in C19 so a story that needs them can reopen them.

## To re-run

The scripts name `W:/temp/claude/refinery-m69-3` (a worktree of 0abbca7),
`W:/temp/claude/refinery-m69-3-old` (of 9f66416) and the output folder
`W:/temp/claude/m69-3`, all removed at M69's close. To rebuild: a worktree of
the commit at that path; `../m67_3/m67_3_grid.rs` copied into its
`crates/godot-ext/tests/` and registered at the end of that folder's
`suite.rs` (`#[path = "m67_3_grid.rs"] mod m67_3_grid;`); `cargo test --release
-p refinery-godot-ext --test suite --no-run`; the specs here copied to the
output folder.
