# M70.0 — the stripped crude unit after a furnace step (A29's leftover), 2026-10-19

Asked as: "the natural next milestone is the stripped plant's slowdown after a
furnace step. The open fixes are fewer pipe sections in the furnace or a
smoothed density, and both would slightly move some plants' results — do it."
Probed first, both fixes and what the probe found instead.

**The probe**: `probe_m70.patch` (on 77a3982) logs, at every density check of
a tick (`energy::swept_density_change`), the largest relative move and the
node that made it (`crates/core/src/probe.rs`), and adds three switches:
`M70_SMOOTH=<q0>` (the line flash's density reads its vapour share through a
smoothstep over `[0, q0]`, `q0·x³(3 − 2x)`, `x = q/q0`, C¹ at both ends),
`M70_COMP=1` (the M69.2 predictor moves each zero-volume node's composition
on too, `2·now − before` clamped at zero and renormalised), `M70_QUAD=1` (the
predictor follows a parabola through the last three ticks, `3·now − 3·before +
before₂`, from the second tick after a command; a straight line before that).
The driver `m70_0_step` (in the patch): a plant held, its furnace stepped, a
CSV row per tick (wall, re-solves, each check's move and node) and snapshots at
the step, +60, +600 and the end; `COLD=1` plays the cold start-up instead
(M68.0's). `run.sh` / `run_godot.sh` run it in the release / screen (`godot`
profile) build; `summary.py` reads the CSVs.

The machine was shared with other work; each number is one run, and the
release and screen builds agree to a few per cent.

## Findings

1. **The second solve after a furnace step is not the coil's.** On the
   shipped engine, from the tenth tick after 58 → 65 MW the first density
   check's largest move is the residue pump's (5e-7, falling to 1e-7 over
   three minutes): the column's bottoms drift in composition as the feed
   heats, and the M69.2 predictor moves temperatures, not compositions. Every
   tick re-solves once for it. Before the step the plant solves once a tick
   (the largest move 3e-11, the gas-oil pump).

2. **Smoothing the density at the boiling onset does nothing here**
   (`M70_SMOOTH=0.05`: 5.1 ms, 2.0 solves, as shipped). The onset cell is not
   where the move is.

3. **The composition predicted, the coil is next**: the residue pump's move
   gone, the coil's middle cells (3–5) set the first check at 5e-8 falling to
   1.3e-8 over three minutes — the curvature of its drift, which a straight
   line misses. Still 2.0 solves a tick, 5.1 ms.

4. **A parabola through three ticks closes it**: with the composition and the
   curvature predicted, the first check is ~1e-10 from the 14th tick after the
   step, 100 times under `SWEPT_DENSITY_TOL`, and the plant solves once a
   tick, as it does settled.

5. **The parabola alone does nothing** (`M70_QUAD=1` without `M70_COMP`, run
   after the table on a machine loaded by other work — shipped re-run at the
   same time 9.1 ms against the table's 5.1, so its wall is not read): 2.02
   solves a tick over the 600 ticks after the step and 2.00 after, as shipped.
   The residue pump's composition sets the re-solve until it is predicted, and
   only then does the coil's curvature show. Both or neither.

6. **Four coil cells instead of eight halve no solves**: the residue pump still
   re-solves every tick. Each solve is cheaper: 3.3 against 5.1 ms after the
   step, 1.85 against 2.45 settled. It moves the physics (the outlet +0.3 to
   +0.6 K, the coil metal −2.2 K, the column's flash zone +0.3 to +0.7 K), and
   the file's starting coil temperature is eight cells' settled one, so the
   four-cell plant drifts from its first tick (3 000 ticks held here).

## The table (the stripped plant, 58 → 65 MW, ms a tick)

```text
                                settled   after: 60 ticks   600 ticks   then   solves a tick   over 2.5 ms in 600
release
  shipped                        2.45         5.10            5.09      5.07       2.0              600
  smoothed density (q0 = 5 %)      —          5.17            5.09      5.08       2.0              600
  composition predicted            —          5.09            5.10      5.10       2.0              600
  + the parabola                 2.53         2.75            2.74      2.73       1.0              598
  four cells                     1.85         3.37            3.30      3.32       2.0              600
  four cells + the parabola      1.93         1.96            1.84      1.86       1.0               25
screen build (godot profile)
  shipped                        2.49         5.11            5.11      5.10       2.0              600
  + the parabola                 2.51         2.74            2.71      2.73       1.0              599
  four cells                     1.84         3.39            3.29      3.31       2.0              600
  four cells + the parabola      1.84         1.88            1.83      1.83       1.0               21
```

Other steps, release, the parabola against shipped: 58 → 45 MW 2.80 against
5.22 ms (600 ticks), 1.06 against 2.07 solves; 58 → 75 MW 2.72 against 7.11,
1.06 against 2.53.

**The cold start-up** (2 400 ticks, its reboiler commanded every 10 ticks from
tick 500): shipped 5.8 ms over ticks 100–700 and 7.75 after; the composition
alone the same; with the parabola 2.8 and 4.1 ms, 1.21 and 1.04 solves.

**The answers move within the solvers' tolerance** with the parabola (the
stripped plant, 58 → 65 MW, against shipped at four ticks): at most 4.0e-7
relative (the residue cooler's pressure), no field over 1e-6.
