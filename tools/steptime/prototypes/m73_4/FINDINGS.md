# M73.4 — the tray plant's keys as a measured region (DESIGN §78.11)

The column screen's key 7 plays `scenarios/crude_unit_trays.toml`: the loop
plant (`crude_unit_loops.toml`) on six sections of four-pass sieve trays, each
stage at its own pressure (M73.1–M73.3). Its keys are the loop plant's, so its
table is measured on M72.6's grid by M72.6's routes (`../m72_6/FINDINGS.md`),
on the shipped engine (HEAD 919157e), with one change the user chose.

## The cost, and the user's choice (2026-10-22)

A sample of 56 jumps and 28 walks (random, seed 7) ran 3.6 times the loop
plant's on the jumps and 2 times on the walks — every tick re-solves the trays'
pressures until they settle (§78.9). Key 6's routes whole came to about 3 ¼ h
by the sample, possibly 6 h (key 6 ran twice its own sample). Offered: the
full routes, shorter waits after each run's last move, or the furnace in
10 MW steps. **Chosen: shorter waits** — each run settles **1 000 ticks** after
its last move, not 3 000 (`RUN=1000`); the walks' 3 000 ticks after the
furnace, which waits on the coil, unchanged; the 20 000-tick check on the
edge kept.

## The files

- `specs_trays.py jumps|walks [furnaces]` — `../m72_6/specs_loops.py`, the same
  grid and routes.
- `m73_4_grid.rs` — `../m72_6/m72_6_grid.rs` with the plant `trays` and each
  run's trays read: the stages flooding and weeping, the highest share of
  flood and downcomer backup, the lowest weep margin, the pressure from the
  top tray to the last. Put it in `crates/godot-ext/tests/`, register it in
  `suite.rs`, build the release test binary, run with `run.sh`.
- `region.py trays <jumps.out,…> <walks.out,…> <out.json>` — M72.6's builder,
  several batch files per route. Wrote `demo/column_region_trays.json`.
- `analyse.py` — the failures by kind, the table against the loop plant's,
  the edge as M72.6's table, the trays along the reflux and over the region.
- `edge.py`, `long.specs`, `long.out` — the 20 000-tick check.

## Machine time

Six batches on 14 threads, one after another, each under an hour: jumps by
furnace 0–10, 15–25, 30–40 MW (14.8, 17.5, 27.4 minutes), walks the same
(27.1, 25.0, 26.3) — 2 h 18 min in all; the sample 1 minute, the long check
1 minute. The raw output (~15 MB) is not kept; the counts are below.

## Measured

- **Jumps**: 16 218 of 17 010 ran. The 792 failures are the same 88 (reflux,
  flow, return) settings at all nine furnace duties (the loop plant: 89), every
  one on the jump's own tick — 731 the cascade not converging, 61 no step
  improving it. **The trays' pressure re-solve never gave up** (its 25-solve
  limit, §78.9) on any run, jump or walk.
- **Walks**: 5 994, 495 failed (the loop plant: 514) on 273 settings (284) —
  231 no step improving, 150 not converging, 114 a start that left the
  physical region (a liquid rate negative, the halved-pumparound retries
  failing too). 19 failed where the jump ran, all at 25–40 MW (26).
- **Allowed**: 16 199, all reachable (the loop plant: 16 183).

**The trays did not move the edge inward.** Every setting the loop plant's
table allows, this one allows too, and 16 more:
- a reflux of 2 with the loops returning 60 K warm runs at 125 % of their
  flow, where the loop plant needs 150 % — at every furnace but 25 and 30 MW
  (7 settings);
- at 25 MW the loops stopped at a reflux of 2.5, at every return but the
  file's (6; the loop plant refuses all seven there by its walks);
- at 30 MW: reflux 0.5 at 75 % and +0 K, 1.5 at 75 % and +40 K, 2 at 150 %
  and +60 K.

The edge, 0–20 MW (the same table at each; the least flow allowed, "none" no
flow on the grid runs; from 2.5 everything runs):

| reflux | −60 K | −40 K | −20 K | +0 K | +20 K | +40 K | +60 K |
|---|---|---|---|---|---|---|---|
| 0.5 | 50 % | 50 % | 50 % | 75 % | 125 % | none | none |
| 1 | 25 % | 50 % | 50 % | 50 % | 75 % | 150 % | none |
| 1.5 | 25 % | 25 % | 25 % | 50 % | 50 % | 75 % | none |
| 2 | 25 % | 25 % | 25 % | 25 % | 25 % | 50 % | **125 %** |

— the loop plant's but the bold one (150 %). From 25 MW a step more in
places, as on the loop plant; with the loops stopped a reflux of 2.5 refused
at 25–35 MW at +0 K, and 3 at 40 MW.

**The trays**, at the file's loops (100 %, +0 K), the same at 0, 20 and
40 MW to a tray: every tray weeping at a reflux of 0.5 (the holes 0.53–0.57
of their weep point at the least) and 1 (at 0 MW trays 1–11); trays 3–8 at 1.5 (at 40 MW 11 and
12 too); none from 2 (the least margin 1.00) to 5; tray 2 flooding at 5.5,
trays 1–4 at 6 (at 0 MW 11 and 12 too), every tray from 7 — and the column
runs on to 15, every tray at up to 256 % of jet flood, its downcomers at
352 % of their limit. The pressure from the top tray to the last 74 mbar at
a reflux of 0.5, 120 at the file's 4, 170 at 15, capped (§78.9).

Over the allowed settings, every tray inside its window on **1 542 of
16 199**. At a reflux of 0.5 and 1 some tray weeps on every one; from 5.5
some tray floods on every one. Between, the loops decide: run fast and cold
they flood the trays their liquid runs down (9–12) from a reflux of 1 (at 20
MW, 1 and 200 % below +0 K: 9 and 10 flooding, 1 and 2 weeping, at once);
run warm they leave the trays starved (at 2, 200 % and +60 K: 3–12 weeping).

**The long check** (`edge.py`, `long.specs`, `long.out`): at 0 and 40 MW,
17 allowed settings each on the edge (a refused neighbour on some key), and
the screen timeline's two (20 MW, reflux 1, 200 %, +40 K; reflux 2, 200 %,
+60 K, which frees its hold), jumped to and run 20 000 ticks: all 36 ran.

## The screen

Headless through its timeline (`--auto --plant=trays`), the machine idle:
1.1–1.6 ms a tick, the snapshot's parse included, so six ticks a frame keep
it near 9–10 ms.

## What was not measured

- Each run settles 1 000 ticks after its last move, not 3 000 (the user's
  choice): a setting that fails between 1 000 and 3 000 ticks would read
  allowed. Every measured failure on a jump came on its own tick, and the 36
  edge settings ran 20 000.
- As M72.6: the return walked only after the flow or the reflux; two key
  orders; the loops apart; the long check by the jump route only; a press
  faster than 300 ticks.
- The trays' readings are the jumps' (the walks record them too, unread).
