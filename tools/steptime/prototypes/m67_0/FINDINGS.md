# M67.0 probe — a side draw cut back where its tray runs short (C17), and the column screen's keys as a region (C12) (2026-10-18)

Throwaway, applied to `f13d8a7` (M66 closed) in a detached worktree:

- `probe_cutback.patch` — a trial of ledger C17 in the heat-balance cascade,
  behind `M67_LIQUID_MIN` (unset: the shipped engine, to the bit), and
  counters (`crates/solvers/src/probe.rs`, printed by the CLI's `corpus` as
  `M67 <plant> …` lines on stderr):
  - `ColumnEquations` gains a set of **cut-back draws**. For each, the mass
    ratio row `W_k = U_k·M̄ − r_k·F` gives way to `L_j = L_min`: the liquid
    leaving the draw's tray is held at the minimum and the draw takes what is
    left. The bottoms is `1 − Σ` of the rest. The published split is each
    draw's ran ratio.
  - `newton_with_water` becomes a wrapper around the shipped wet-set solve
    (renamed `newton_water_set`). Its first attempt is the shipped solve from
    the start as given, with no draw cut back, so a column that never reaches
    the minimum is untouched to the bit. Then:
    - A converged solve with a tray under the minimum cuts that draw back.
    - A failure whose last iterate runs a draw's tray dry (≤ max(L_min, 1 %
      of F)) cuts that draw back.
    - A start outside the physical region is cut back top-down.
    - A cut-back draw whose ran share would pass its declared one is
      released.
    - The retry starts from the last converged profile.
    - If no set of cut-back draws holds, the plain solve's answer stands, and
      only that one, so nothing that ran before can fail.
  - `cut_start`, the start's draws cut back top-down. The balance is the
    solve's own: the top vapour is `(R + 1)·D` at the commanded reflux (a held
    column's own), and each cut leaves `L_min` on every tray down to the next
    draw, never above the draw's declared rate.
  - The cut-back set is carried from one tick to the next. In the probe this
    is a per-thread side channel (`LAST_SHORT`); in a slice it is a field on
    the carried profile, beside its free water.
- `m67_0_grid.rs` — the driver (`crates/godot-ext/tests/`, registered in
  `suite.rs`). It drives a plant through the bridge's `Session` with the
  column screen's own command text. A run is a list of moves (`key=value`
  joined by `+`, moves by `>`, `@N` the ticks to the next move). Keys:
  - `furnace` MW, `reflux`, `steam`;
  - `pa`, a share of the file's 15/15/10 MW on the crude and cold plants, MW
    on every pumparound on the stripped plant;
  - `reb` (MW, held), `free`, and `walk=step` (the screen's start-up walk,
    `column.gd` WALKS).

  It prints whether the run held. If it ran, it prints the settled reading,
  the cut-back counters, and the end state: each side draw's share of its
  declared rate, and the lowest liquid under a draw.
- `run.sh` — runs the suite binary at below-normal priority. `TARGET` names
  the build folder and `OUT` the output folder; both are required.

Run (Windows, release, below-normal priority):

```
PLANT=crude|stripped|cold [WARM=1000] [GAP=3000] [RUN=6000] [JOBS=8] \
  [M67_LIQUID_MIN=0.05] [M67_TRACE=1] SPECS='furnace=20+reflux=1+pa=0.5;…' \
  TARGET=<build folder> OUT=<output folder> sh run.sh <label>
M67_LIQUID_MIN=0.05 target/release/refinery corpus scenarios --ticks 6000 \
  --jobs 14 --solver newton|simple --baseline B/<fidelity>.json
```

`L_min` below is a share of the feed's moles. Every crude-plant grid run moves
every key in one step at tick 1 000, then runs 6 000 ticks. "Walks" step the
screen's keys (reflux 0.5, pumparounds 0.25) every 300 ticks.

## Results

1. **The trial switched off is the shipped engine.** All 67 plants on both
   fidelities are byte-identical to the rolling baseline.

2. **No shipped plant comes near a dry draw tray except the stripped one.**
   The least liquid under any side draw over the whole corpus run is:

   | plant | least liquid under a draw |
   |---|---|
   | `crude_unit_stripped` | 6.55 % of the feed (stage 7, gas oil) |
   | `crude_column_overheated` | 38.8 % |
   | every other column | over 60 % |

   No solve in the corpus fails.

3. **At `L_min` 2 % or 5 % every plant is byte-identical on both
   fidelities.** At 10 % only `crude_unit_stripped` moves: its gas oil is cut
   back to 84.6 % of its declared rate on 6 631 of 6 642 solves, with one
   extra solve in the run. That holds once the cut-back set is carried from
   tick to tick (result 7). An earlier build that trimmed every start moved
   the stripped plant in the last bits even at 2 %. The first attempt must be
   the shipped solve.

4. **The crude unit (key 1): 55 of the old box's 85 failures run.** On the
   furnace 0–40 MW × reflux 0.5–15 × pumparounds 0–4 grid (400 settings),
   nothing that ran fails and 30 still fail. What is cut back is the cost, as
   end-state shares of the declared rate:
   - Reflux 0.5 with pumparounds 0.75–4: the kerosene runs at 45–46 %.
   - Reflux 0.5 with pumparounds 0.5: the kerosene at 47 % and the diesel at
     74 %.
   - Reflux 1 with pumparounds 0.5: the diesel at 78 %.
   - Reflux 1.5 with pumparounds 0.25: the gas oil at 51–65 %.
   - Reflux 2.5 with no pumparounds: the gas oil at 30–42 % (77 % at 30 MW).
   - At furnace 40 MW the vapour floor (§66.3) carries the reflux to
     1.7–3.8, and nothing is cut.

   The 30 still failing are the corner where the column makes too little
   liquid above the feed:
   - reflux 0.5 or 1 with pumparounds 0–0.25;
   - reflux 1.5–2 with none;
   - one at reflux 1 with pumparounds 0.5.

   With its draws cut back there, the column holds every tray wet but cannot
   close the top stage's energy balance (`did not converge in 50 iterations`,
   the worst equation stage 1's energy). This is not a dry tray.

5. **The crude unit's edge depends on the path.** Walking into the failing
   corner in screen steps, in both orders (reflux first, pumparounds first),
   28 of 62 walks run.
   - The same end setting can run by one route and fail by the other. Furnace
     40, reflux 1, pumparounds 0.25 fails reflux-first at pumparounds 0.5. It
     runs pumparounds-first, at a reflux of 2.76 against a commanded 1 (the
     vapour floor).
   - About half the one-step failures run when walked into, and a fast player
     is nearer the one-step case.
   - At the extreme (furnace 30, reflux 0.5, pumparounds 0.25) two walks ran
     on an earlier build by cutting a draw to 0.000 of its declared rate, at
     3–4 solves a tick. On the final build they fail, so the result there
     depends on the solve's route.

6. **The stripped crude unit (key 4): 81 of 83 settings run, against 30 of
   83 before.** The steam floor is gone: every furnace 42.5–55 MW × steam
   0.05–1.0 setting runs, and the joint furnace × steam × pumparound moves
   run too. What is cut back is the gas oil, as end-state shares:

   | furnace | steam 0.05 | steam 1.0 |
   |---|---|---|
   | 42.5 MW | 22 % | 67 % |
   | 45 MW | 35 % | 77 % |
   | 50 MW | 58 % | 97 % |

   Nothing is cut at furnace 52.5 MW with steam 1.0, or at 55 MW with steam
   0.6–1.0. Pumparounds of 10–11 MW cut two or three draws, to as little as
   22 %, at a reflux of 0.31–0.61. Still failing: pumparounds 12 MW at
   furnace 45 or 47.5 MW (`did not converge`, 400–1 000 ticks after the
   move).

7. **The cost.** Before the carry, a column at an edge solved twice every
   tick: the plain solve failed at 18–50 Newton iterations, then the cut-back
   one held (`extra 6000` in 6 000 ticks). With the cut-back set carried, one
   solve a tick: `extra 1` in the run. Crude grid runs average 5.21 s with
   nothing cut, 5.34 s with a draw cut back at the end, and 5.30 s on the
   shipped engine (12 runs in parallel, so a ratio, not a frame time).

8. **The cold plant (key 5).**
   - **The start-up moves.** The screen's story (furnace 58 MW at 10, the
     pumparounds off at 500, the reboiler walked down 2.5 MW every 10 ticks
     from 500) already runs the gas-oil tray to 0.0000 of the feed on the
     shipped engine, and converges there. With `L_min` 5 % that draw is cut
     back, down to 0.000 of its declared rate at the worst. The story still
     ends held at zero, at a reflux of 3.088 against 3.070 and a bottom of
     348.1 °C against 352.3 °C. It moves at any `L_min > 0`. The corpus plant
     `crude_unit_cold` (no commands) does not move. The screen's replay
     (`column_screen.rs`) does.
   - **Freed, it has no edge inside the keys' box.** After the story the
     reboiler is freed and the keys are set in one step: furnace 20–65 MW ×
     steam 0.1/0.4/1.0 × pumparounds 0/0.5/1. All 90 settings run on both
     engines and settle identically (reflux 4.0–4.82). Nothing is cut at the
     end, and the least liquid under a draw is 26.8 % of the feed.

## What a slice must settle (the user's forks)

1. `L_min`. At 2 % or 5 % the corpus holds, and 10 % moves the stripped plant.
2. The cold start-up story moves at any `L_min`, because the shipped story
   runs a tray dry.
3. How a cut-back product shows. The snapshot needs it, as the screen cannot
   tell otherwise; the precedent is `column_reboiler_declined`.
4. A draw cut to zero: a product that stops, or the edge.
5. The screen's limits: a table that cannot be exact (result 5), measured
   conservatively over walks and one-step moves, or something else.

The user's answers (2026-10-18), DESIGN §72: `L_min` 5 %; the cold start-up may
move; a cut-back product named on screen, one cut to nothing a product that
stops; cautious tables, a setting off-limits if any measured route to it failed.
