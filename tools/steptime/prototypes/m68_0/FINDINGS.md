# M68.0 — what the column's Newton costs, per tick (C11), 2026-10-19

The probe: `probe_count.patch` (on 58ffdc1) counts, per tick, the column
solves (`separate`, `ColumnEquations::solve`), Newton's iterations, and its
evaluations by caller (the start, the Jacobian's columns, the line search),
and splits the wall time inside them into shares. The driver
(`tools/steptime/src/bin/m68_0_count.rs` in the patch) plays nine column
plants through a story — `crude_unit` and `crude_unit_stripped` with key
presses, `crude_unit_cold` its start-up (`crude_unit_cold_reference.rs`'s),
the rest 2 000 ticks held — on Newton (the column solve does not depend on
the hydraulic solver). Output: `count.txt` (all nine), `count_screen_plants.txt`
(the three screen plants again, with a per-tick dump).

**The machine was loaded by other work throughout** (about a third of 16
cores busy; the second run's ordinary ticks ran 1.4 times the first run's, with
single ticks of hundreds of ms on the stripped plant that are the machine, not
the plant). So the numbers that decide are the COUNTS, which are exact and
deterministic; a time is the count times the cost of one evaluation, about
11 µs on the crude unit (the first run: evaluations 66.5 % of a 0.426 ms tick
over 25.6 evaluations). Shares, not milliseconds, are read from the timer.

## Findings

1. **The dry column plants: the column is the tick, and a steady tick is one
   Newton step.** `crude_unit` 2 794 of 2 799 ticks take exactly one iteration,
   25 evaluations (one start, 23 Jacobian columns, one line-search trial);
   the column is ~90 % of a 0.43 ms tick, the Jacobian ~63 %. The hot-fed and
   overheated columns converge without a step on half their ticks (median one
   evaluation, ~10 a tick on average, 0.19–0.25 ms a tick).

2. **A big key press on the crude unit is one tick of ~700–760 evaluations —
   about 8 ms, one hitch, then back to 25.** The reflux 4 → 2 and the furnace
   20 → 35 MW (with the reflux and pumparounds back) take 19–21 iterations over
   two solves; a pumparound or reflux step that stays near the running state,
   97–121 evaluations (1–1.5 ms). Five such ticks in 2 799.

3. **The wet, steamed, boiling, stripped and cold plants: the column is NOT the
   tick.** Held, the column is 4–10 % of their wall; their ticks are 2.7–15 ms
   on this machine, and what fills them is the coil re-solving through its
   drift (A29). Their column runs 1.6–4.8 times a tick because each of the
   engine's re-solves re-runs it: the stripped plant after a pumparound step
   holds 5 solves × 4 iterations = 105 evaluations every tick (~1.2 ms), the
   cold start-up 100–180 (~1.1–2 ms), 32 ticks over 150, none over 300.

4. **Inside one evaluation** (crude unit, share of the tick): the condenser's
   bubble point 20 %, the Thomas sweeps 17 %, the enthalpies 10 %, the
   K-values 9 %, the normalisation 5 %, `draw_states` with I7's residual 2 %;
   the dense solve 1 %. On the dry plants 20–35 % of the tick is the column
   OUTSIDE Newton (`separate` 87–91 % against Newton's 47–69 %).

5. **What a bit-identical Jacobian could save is bounded at ~10 %** of the
   crude unit's tick: a temperature column changes one stage's K-values and a
   flow column none (on a dry feed), and `draw_states` with I7's residual does
   not feed the residuals — but the condenser's bubble point, the sweeps and the
   enthalpies all move with every column. Not built: the bound is below what
   it would change. (A trimmed evaluation must keep every check that can fail
   after the residuals are built, or the backward-difference fallback picks a
   different side and the bits move.)

6. **Anything that changes Newton's path moves every column plant's
   fingerprint** — a Jacobian reused across ticks or within a solve, Broyden's
   update, another difference step — and with it C19: the crude, stripped and
   cold plants' screen tables re-measured (within an hour each). Reused across
   steps, the steady crude unit tick would fall from 25 evaluations to a few,
   and the key-press tick by several times; on the coil plants it would take
   ~1 ms off ticks of 3–15.

## So

C11's trigger — "a column whose solve is the frame's cost" — is met only by
the crude unit's big key press, one ~8 ms tick. Everywhere else the column is
inside the budget, or a small share of a tick the coil already overruns. A29
(the coil's drift) is the frame's cost on every plant that misses it, and its
cure takes the column's repeat solves with it; C11 is re-measured by this
probe after A29.
