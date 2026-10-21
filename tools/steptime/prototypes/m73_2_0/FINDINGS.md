# M73.2.0 — the pressure feedback probed, 2026-10-21

M73's decision 3 (DESIGN §78 intro): each stage solves at its own pressure,
the top at the column's declared 1.8 bar, each stage below higher by the trays
above it. Open for this probe (§78.1): the drops from the last tick or solved
with the stages, and what the drop fed back is past flood. No engine code
changed.

## The probe

- `probe_pressure.patch` (on c5d4e14) + `probe.rs` (`crates/solvers/src/`) +
  `m73_2_0_probe.rs` (`crates/scenarios/tests/`, a module of `suite.rs`; the
  patch adds the line). A thread-local stage-pressure profile read at every
  site that took the column's one pressure: `stage_k_values_profile`,
  `bubble_profile` (the fixed point and the post-solve K), Newton's K in
  `ColumnEquations::evaluate`, the wet bubble profile, the feed's seed and
  flash (at the feed stage's pressure), each tray's vapour density (at the
  stage below's). The condenser stays at the top pressure. Off, every site
  reads the column's pressure as shipped.
- Modes (`probe.rs`): **off** (one pressure; the drops only read), **lag**
  (this tick's stages at the drops the LAST tick left — `end_tick` after each
  `Engine::tick`, so a lag of one tick, not one pass), **solve** (within the
  pass: solve, read the drops, solve again at them until they move < 0.5 Pa
  — where a solve with the pressures inside Newton would land, not its cost).
  Rules past flood: **raw** (the correlation's drop as it reads) and **cap**
  (the drop with the vapour cut to exactly 100 % of jet flood by bisection,
  the liquid as it is — the flow parameter moves with the vapour, so not a
  plain scaling).
- The plant: M73.1's fixture, the loop plant with its five sections inline
  (`tray_hydraulics.rs`'s 6.1 / 7.4 / 8.3 / 8.8 / 8.3 m) — **sized at one
  pressure**; M73.3 sizes the new plant's.
- Runs: E1 from cold at the file's settings, 600 ticks, each mode, one thread;
  E1b a cold start straight at the settled drops; E2 the reflux line 0.5–15
  (30 settings), each on a fresh engine, 300 ticks at 4 then the jump, 300
  ticks more, five modes, 14 threads (10 s); E3 the same jumps to 1, 3.5, 4.5,
  6, 8, 15 on ONE thread for the wall column. `line.py`, `lagdiff.py` →
  `analysis_line.txt`, `analysis_lag.txt`, `analysis_timing.txt`. Output to
  `$M73_OUT` (default `W:\temp\claude\m73_2_0\out`, not kept; re-run in under
  a minute).
- **The check that every site was found**: after each solve,
  `|Σ K(T_j, P_j)·x_j − 1|` at each stage's own pressure ≤ 8.4e-8 on every
  tick of every run, and ≥ 4.3e-2 at the column's one pressure — every K was
  taken at its stage's pressure.

## Findings

1. **At the file's settings the products barely move; the bottom is 5.5 K
   hotter.** The bottom 586.19 → 591.68 °C (§78.0's estimate 5–7 K), the top
   +0.03 K; the purities (each draw's own cut, mol %) naphtha 96.48 → 96.46,
   kerosene 81.29 → 81.23, diesel 77.95 → 77.85, gas oil 75.80 → 75.64,
   residue 96.60 → 96.54 — at most 0.16 points; the reboiler 85.15 →
   86.57 MW (+1.4 MW), the condenser unchanged; the feed's flash share
   0.663 → 0.654 (it flashes 100 mbar higher). The column's drop 123.1 mbar
   at one pressure, 122.1 fed back.
2. **Along the reflux line the shift grows with the drop** (`analysis_line.
   txt`, settled 300 ticks after the jump): the bottom +3.7 K at a reflux of
   0.5 to +15.4 K at 15 raw (the drop 78 → 360 mbar); **capped, +7.5 K at
   most** (the drop levels off at 150–170 mbar once the trays pass flood, from
   about 6). The purities move at most 0.52 points raw, 0.23 capped.
   Weeping is where it was at one pressure (every tray at 0.5–1, seven at
   1.5, one at 2–2.5).
3. **Every mode ran every setting.** No solve refused anywhere on 0.5–15, at
   one pressure or fed back, lagged or solved, raw or capped.
4. **Lag and solve land on the same column; the lag is wrong for a few
   ticks after a key moves** (`analysis_lag.txt`): on the jump tick the
   lagged column runs at the OLD drops — up to 238 mbar and 9.9 K at the
   bottom off on a jump to 15 raw, 47 mbar and 2.0 K capped, 44 mbar and
   2.1 K going down to 0.5 — and its error then shrinks about 15 times a
   tick, the solve's re-solving spread over ticks: at 15 raw 238 → 15.8 →
   0.8 → 0.0 mbar, within 1 mbar by the third tick; capped within 0.3 mbar
   by the next. No ringing. **After every load** (no history)
   the lagged column's first tick is at one pressure and its second 5.5 K
   hotter at the bottom. The plant itself drifts 40–150 ticks after a jump
   (the loops' coolers, the coil) in every mode, the solved one included.
5. **Solved within the tick, the drops settle fast**: 3–5 solves on a tick
   a key moves (each re-solve cuts the drops' error 10–15 times: 23.8 kPa
   off → 0.5 Pa in five at 15), ONE on a steady tick (the last tick's drops
   are the first guess). The answer does not hang on that guess: from cold,
   started from no drops (9 solves) and from the settled ones (E1b, 6), the
   first tick lands within 0.11 Pa a tray and 0.0003 K at the bottom — inside
   the 0.5 Pa tolerance — so the seed contract `separation_is_start_
   insensitive` gates still holds (gated in M73.2).
   Coupled this weakly, putting the pressures inside Newton (one unknown per
   stage, about a third more Jacobian columns) buys nothing the re-solve
   does not.
6. **The cost is small** (`analysis_timing.txt`, one thread, release): a tick
   the reflux moves 1.2–1.9 ms at one pressure, 2.4–5.3 ms solved, the lag's
   1.2–3.6 ms spread over two ticks; a steady tick 0.5–0.6 ms in every mode;
   the cold first tick 14 ms → 22 ms (9 solves over its 3 passes, against 3).
   A 60 fps frame is 16.7 ms.
7. **The cold start needs no hand-over.** Solved, the first solve is at one
   pressure and the next at its drops — the hand-over happens inside the tick
   and the 13-tray plant starts as it does today (its seed retry included).
   Started straight at the settled profile (E1b, no hand-over), it starts too,
   in 6 solves.
8. **What "refuse past flood" would take from the screen.** A dry column's
   refusal is an `Err` from `separate` (`flooded_pass_through` only answers a
   feed that is mostly WATER); the tick fails and the column screen halts
   (`column.gd` `_halt`). A screen that does not want to halt must stop the
   key first — its measured region (`column_region_loops.json`, M72.6), with
   the other keys at the file's, would end the reflux at 5.0 (refusing on jet
   flood only: one tray at 5.5) or 4.5 (on jet flood or downcomer backup: one
   tray past its downcomer limit at 5.0) against 0.5–15 today — 20 or 21 of
   the line's 30 settings gone, against
   decision 4 (the keys keep their full range, the player sees the trays
   flood). The downcomer end leans on the multipass layout §78.5 names
   unchecked (C21).
9. **Capped, a flooded column still says it floods**: the cap holds only the
   drop fed back; the tray's reading (flood fraction to 2.6, backup to 3.0)
   and the lists are the raw ones.

## Not probed

- A wet or steamed column (the patch reaches the wet sites; the plant is dry).
- Draw and loop lines starting at their own stage's pressure (§78.1's
  boundary; the network side, 40–110 mbar over the top on the loop stages).
- Tray sizes chosen at the fed-back pressures (M73.3).
