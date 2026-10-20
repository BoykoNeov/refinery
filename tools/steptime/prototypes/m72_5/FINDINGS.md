# M72.5.0 — the key-press hitch (C11), probed

`probe.patch` on d31ddd1 (throwaway, not for commit): counters on every column
solve (`crates/solvers/src/probe.rs`), three switches, and the driver
`tools/steptime/src/bin/m72_5_count.rs`.

- `PROBE_JAC=fd|broyden|chord<k>`: the Jacobian each Newton step is taken on.
  `fd` is today's (forward differences every iteration). `broyden`: forward
  differences on iteration 1 and on a refresh, Broyden's "good" rank-1 update
  in between (scaled by the unknowns' sizes); refreshed after a step that
  needed halving or cut the residual by less than half (`PROBE_RATIO`), and a
  reused Jacobian with no descent or a singular one is refreshed and the
  iteration retried. `chord3`: one forward-difference Jacobian for up to three
  iterations, same refresh rules.
- `PROBE_ORDER=1`: the order fix below. `PROBE_DETECT=1`: the order fix's
  divergence detector (it also runs the floor-first attempt and reports
  `PROBE_DIVERGE` wherever that attempt would have converged and held).

The story (`m72_5_count`): 1 000 ticks, then key presses each followed by 300
ticks: reflux 4 → 2 → 4, furnace 20 → 35 → 20 MW, reflux 4 → 6 → 4; on the
loop plant also loop 1's flow 85 → 120 → 85 kg/s, loop 2's return 215 → 180 →
215 °C, loop 3's flow 65 → 40 → 65 kg/s. Plants: the shipped `crude_unit`
(10 trays, 23 unknowns) and `crude_unit_loops` (13 trays, 32). `run.sh` runs
it at below-normal priority, `compare.py` sets the kept snapshots of two runs
side by side. Every run's output is in `runs/`.

## Findings

1. **The hitch is a failed solve on the wrong set, not the Jacobian.** After the
   reflux is LOWERED, `newton` sees the warm start running above the declared
   reflux and tries the vapour floor first (§66.3: "a start that ran above its
   declared reflux tries the floor first"). The start ran above it because the
   player lowered it, not because the floor bound, and the floor solve has no
   answer: it grinds to "found no step that improves it" (crude unit, 15
   iterations, 600 evaluations) or "did not converge in 50 iterations" (loop
   plant, 1 715 evaluations), the step scale near zero from iteration 5. Then
   the declared-reflux solve, from the same start, converges in 3–4 iterations.

   | tick after reflux 4 → 2 | evaluations | of which the failed floor solve |
   |---|---|---|
   | crude unit | 700 | 603 |
   | loop plant | 1 848 | 1 715 |

   Raising the reflux, furnace steps and every loop key: 1–4 iterations, 25–133
   evaluations; no failed solve anywhere on them.

2. **The order fix**: the floor first only where the start's lowest vapour is
   at the floor (within 1e-6), which is what a floor-bound column's warm start
   looks like; otherwise the declared reflux first, and today's floor-first
   attempt after it wherever the declared solve did not return a column
   standing above the floor (failed, or below the floor). Every answer today's
   order returns, this one returns, unless the floor-first attempt converges
   and holds while the declared solve ALSO returns a standing column —
   two answers at two refluxes, which a reflux that only raises the vapour
   does not have. **Byte-identical**: both plants' snapshots through the whole
   story (`compare.py fd fd_order`), the corpus 68 of 68 on both flow solvers
   against the rolling baseline (`runs/order_*.txt`), the full test suite
   (1 230 passed, the column screen's table gates among them) with the
   detector on — `PROBE_DIVERGE` never printed.

   | | today | order fix |
   |---|---|---|
   | crude unit, worst tick after tick 1 | 700 evaluations | 97 |
   | loop plant, worst tick after tick 1 | 1 848 | 133 |
   | loop plant, tick 1 (cold) | 2 708 | 904 |

   The cold start gains too: its warm-up ends above the declared reflux, and
   each hand-over solve paid the failed floor attempt.

3. **Broyden on top** halves what is left: worst ticks 55 / 72 evaluations, the
   loop plant's cold start 468. **It moves every heat-balance plant**: every
   snapshot of both plants differs from today's (130–250 numbers each), the
   largest relative difference 6e-8 the tick after a key press and ~1e-11
   settled — inside the solve's tolerance, but a moved fingerprint everywhere
   and the column screen's three tables (C19) to re-measure. A steady tick is
   one Newton step on a forward-difference Jacobian either way, so it saves
   nothing there. Chord (k = 3) sits between: worst 76 / 103, same moves.

4. **The "free Jacobian" bound** (an analytic Jacobian at its best): the
   order-fixed key press on the loop plant is 4 iterations; with the Jacobian
   free it would cost the start, 4 line-search evaluations and a few
   evaluations' worth each for the derivatives — ~15–20 evaluations against
   Broyden's 72. Not worth an analytic Jacobian's size for ~1 ms.

5. **Timing** (release, below-normal priority, two runs agreeing to the counts;
   the machine shared, so walls are indicative):

   | | today | order fix | order fix + Broyden |
   |---|---|---|---|
   | crude unit, reflux 4 → 2 | 5.6–12 ms | 1.3–1.8 ms | 0.7–1.1 ms |
   | loop plant, reflux 4 → 2 | 23–35 ms | 2.3–2.7 ms | 0.9–1.6 ms |
   | loop plant, cold tick 1 | 50–72 ms | 20–25 ms | 13–18 ms |
   | loop plant, steady median | 0.74–0.88 ms | 0.63–0.86 ms | 0.61–0.87 ms |

   Against the 2.5 ms budget: with the order fix, every key press on both
   plants is at or under it; the loop plant's reflux press at its edge.
