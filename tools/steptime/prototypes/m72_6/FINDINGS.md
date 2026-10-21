# M72.6 — the loop plant's keys as a measured region (DESIGN §77.15)

The column screen's key 6 plays `scenarios/crude_unit_loops.toml`. Its keys are
held by a table measured here, by C19's method (M67.3, DESIGN §72.8), on the
shipped engine (HEAD 36e36aa).

## The grid (the user's choice, 2026-10-21: "Together, 4-key table")

- furnace 0–40 MW in 5 MW steps (9);
- reflux 0.5–15 in 0.5 (30);
- the loops' flow 0–200 % of the file's 85 / 95 / 65 kg/s in 25 % steps (9),
  all three loops together;
- their return temperatures −60 to +60 K from the file's 150 / 215 / 270 °C
  in 20 K steps (7), together.

17 010 settings. The file's: 20 MW, reflux 4, 100 %, +0 K.

## The routes

`specs_loops.py jumps|walks [furnaces]` writes them; `m72_6_grid.rs` (put it
in `crates/godot-ext/tests/`, register it in `suite.rs`) runs them; `run.sh`
runs the release test binary at below-normal priority.

1. **One jump** to every setting: the file's state settled at tick 1 000, all
   four keys sent on one tick, then 3 000 ticks.
2. **Reflux first**: the furnace at 1 000, 3 000 ticks; the reflux in screen
   steps from 4, the flow in steps from 100 %, then the return in steps from
   +0 to −60 and to +60 (two walks per reflux and flow), a press every 300
   ticks, then 3 000 ticks.
3. **Loops first**: the flow, then the return, then the reflux down to 0.5
   and up to 15.

`region.py loops <jumps.out> <walks.out> <out.json>` writes
`demo/column_region_loops.json`: allowed where the jump ran and no walk failed
on reaching it, kept where reachable from the file's by single steps.

## Machine time

The first jump batch was stopped 3.4 minutes in (773 runs kept), when its
estimate came to about 75 minutes, and the rest was split. Five batches on 14
threads, each under an hour (from the output files' creation and last-write
times): jumps 773 + 8 118 + 8 119 (3.4, 38.5 and 38.8 minutes), walks 3 042 +
2 952 (53.8 and 56.8 minutes — the second, 36 minutes by the first's pace,
came within 3 minutes of the hour; why it ran slower was not traced); the
long check 1 minute. About 3 h 10 min in all, against the 1 h 40 min first
estimated from a sample. The raw output (~6 MB) is not kept; the counts are below.

## Measured

- **Jumps**: 16 209 of 17 010 ran. Every one of the 801 failures came on the
  jump's own tick (the cascade did not converge), and the same 89 (reflux,
  flow, return) settings fail at every furnace duty: the column fails before
  the furnace has changed the feed.
- **Walks**: 5 994, 514 failed, on 284 settings. 26 of those ran by the jump,
  so their edge is the route's — all at 25–40 MW. Walk failures per furnace:
  71, 71, 71, 72, 73 at 0–20 MW; 62, 45, 25, 24 at 25–40.
- **Allowed**: 16 183, all reachable.

The edge, 0–20 MW (the same table at each): a low reflux needs the loops
running, and more of them the warmer they return.

| reflux | −60 K | −40 K | −20 K | +0 K | +20 K | +40 K | +60 K |
|---|---|---|---|---|---|---|---|
| 0.5 | 50 % | 50 % | 50 % | 75 % | 125 % | none | none |
| 1 | 25 % | 50 % | 50 % | 50 % | 75 % | 150 % | none |
| 1.5 | 25 % | 25 % | 25 % | 50 % | 50 % | 75 % | none |
| 2 | 25 % | 25 % | 25 % | 25 % | 25 % | 50 % | 150 % |

(the least flow allowed; "none": no flow on the grid runs). From 2.5 every
setting runs.

From 25 MW the walks take a step more in places (the 26 settings):
- reflux 0.5: +0 K needs 100 % from 30 MW; −20 K needs 75 % at 25 MW;
- reflux 1: +0 K needs 75 % and +20 K 100 %, from 30 MW;
- reflux 1.5: +40 K needs 100 % at 25 and 30 MW; +20 K 75 % at 30 MW;
- reflux 2: +40 K needs 75 % and +60 K 175 %, at 30 MW;
- the loops stopped: a reflux of 2.5 is refused at 25 MW at every return, at
  30–35 MW at +0 K (and at 30 MW the loops at 50 % and +60 K); at 40 MW a reflux of 3 is
  refused at +0 K while 2.5 is allowed.

The edge is not monotone in the furnace or the reflux, and the table keeps
what was measured (as M67.3's did at 37.5–40 MW).

**The long check** (`edge.py` picks, `long.specs`, `long.out`): at 0 and
40 MW, 17 allowed settings each on the edge (a refused neighbour on some key),
and the screen timeline's two (30 MW, reflux 1.5, 200 %, +40 K; reflux 2,
+60 K), jumped to and run 20 000 ticks: all 36 ran.

## What was not measured

- The return walked only after the flow (loops-first) or after the reflux and
  the flow (reflux-first); no walk moves the return first.
- Two key orders; the furnace walked only first.
- The three loops apart: the keys move them together.
- The long check by the jump route only.
- A press faster than 300 ticks.
