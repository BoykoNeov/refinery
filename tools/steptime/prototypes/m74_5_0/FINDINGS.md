# M74.5.0 — stages that stop boiling, part A of the start from empty's probe (2026-11-10)

The record is DESIGN §79.9. After M74.4 the user said yes to the start from
empty's probe, which the user's 2026-11-09 decision ties to the feed cut "by
stages that stop boiling" (C26). Split in two (the advisor's advice): **part
A**, a stage that stops boiling, measured here on the running holdup plant;
**part B**, the empty column, put to the user as questions first. No engine
code changed.

## The prototype (`stop_boiling.patch`, on 055b190's `column_step.rs`)

Three changes, each switchable by `M74_5_0_VARIANT` (probe only):

1. **Complementarity**: each stage boils (`Σ K·x' = 1`, `V ≥ 0`) or does not
   (`V = 0`, `Σ K·x' ≤ 1`, its liquid below its bubble point) — the stage's
   first residual `max(Σ K·x' − 1, −V/500 mol/s)`, a min-function
   (semismooth Newton; Hintermüller, Ito & Kunisch, SIAM J. Optim. 13 (2002)
   865). A boiling stage's residual is today's to the bit. A stage not
   boiling has its temperature from its energy balance alone — the heat its
   liquid stores is in since M74.2.
2. **Projection**: a vapour a Newton step takes through zero lands on zero,
   the rest of the step whole (today: the whole step shrunk to 0.9 of the
   distance to zero, so a vapour never reaches it). A zero vapour is no
   longer refused by the evaluation.
3. **The sum-of-squares fallback** (`l2`): a step that finds no descent in
   the largest residual (today's line search, which would refuse the tick)
   is searched again on the sum of squares (the semismooth Newton merit; Qi &
   Sun, Math. Program. 58 (1993) 353). Only where the tick would stop.

Variants: `damping` = 2 alone, `complementarity` = 1 alone, `both` = 1 + 2,
`l2` = 1 + 2 + 3, `fb` = 2 with 1 as Fischer–Burmeister (Fischer,
Optimization 24 (1992) 269) in place of the min-function.

The tray hydraulics at zero vapour were read first (`trays.rs`): no division
by the vapour anywhere — `sieve_tray` reads no vapour as no flood and full
weeping, the dry-plate head zero, the vapour density from `y = K·x`.

## The driver (`m74_5_0_cold.rs`)

M74.4's (`../m74_4/`) with `PLANT=valved` (the holdup plant with a valve on
its crude line, as `column_holdup.rs`'s `fixture_valved`), the key `valve`,
each run's stages that sent up no vapour (under 1e-3 mol/s) counted, and
`TRACE=N`. Built twice from worktrees of 055b190: patched and unpatched.
`compare.py` sets a batch on one against the other.

## Findings

1. **Neither change alone does it; both do.** The one-tick feed cut and the
   reboiler held at 0 MW in the worst corner: refused on `damping` alone and on
   `complementarity` alone, as unpatched; both run on `both`
   (`variant_*.txt`).
2. **The one-tick feed cut (C26) runs, and no stage stops boiling** (the least
   vapour 43 mol/s, stage 3) — for as long as the furnace's tubes last (9.3
   minutes firing; 20 minutes, the whole run, with it off — finding 8). The unpatched refusal ("stage 3's vapour
   0.000") was Newton's PATH through a stage with no vapour, not the answer:
   the converged first tick with no feed has every stage boiling. The user's
   2026-11-09 decision stands on a different reason: the machinery is needed,
   but the cut is not a column whose trays stop boiling.
3. **The 80 MW reboiler floor (§79.8) was real**: the 70 held runs that set
   it, rebuilt from §79.8's table (the counts match: unpatched 31 of 70 run,
   24 of 48 + 5 of 16 + 2 of 6), all 70 run patched (`holds_*.txt`). Every
   run the unpatched engine refused has, patched, stages that stop boiling;
   every run it ran has none. Which stop: the top four under the loops cold
   and full (1–4, from 0.1 to 20 minutes with some reboiler; at 0 MW to the
   end), the base and the stage above it
   at a reboiler of 0 MW (12–13, to the end — nothing heats them). Every run
   with some reboiler ends with every stage boiling.
4. **The feed cut, the set (`cut_*.txt`)**: unpatched 0 of 7 run, `both` 4
   of 7: the cut; the cut and the feed back after 5 minutes; the cut at a
   40 MW reboiler; the cut at 0 MW with the loops cold and full and then the
   feed and 86 MW back — up to 8 stages stop boiling, every one boiling again
   2.3 minutes after the feed and heat return (7.3 minutes cold in all). Two refused:
   - **the cut with the loops and the reflux stopped**: the trays' pressure
     drops still move 0.5 Pa after 25 solves, 11 s on (§79.6's other wall —
     the pressure re-solve, not a stage that stops boiling);
   - **the cut at 20 MW, the feed back in one tick**: Newton stalls on the
     tick stage 3 starts boiling again (0.01 MW short, no descent).
5. **The sum-of-squares fallback cures the stall** (`sw_l2.txt`): that run
   goes through, stage 3 boiling again 9 ticks after the feed returns; the
   Fischer–Burmeister function does not (`sw_fb.txt`). The fallback acts only
   on a tick that would otherwise be refused, so every tick that converges
   today is unchanged by construction.
6. **Every shipped plant byte-identical** on `both`, 70 of 70 on Newton and
   70 of 70 on the simple solver, against the rolling baseline (the holdup
   plant included: no stage there stops boiling).
7. **Below the floor, at random** (`below_*.txt`, `l2`): 200 single moves,
   the reboiler held 0–80 MW, the other four keys anywhere in the box, 20
   minutes after — all 200 run; 5 have stages that stop boiling for a time
   (always among the top four), none at the end. The slowest tick 71 ms
   (14 runs at once on 16 cores; §79.8's slowest 168 ms).
8. **The cut runs only 9.3 minutes with the furnace firing** (a side agent's
   catch: the set above stopped at 8.3 minutes, just short). At 5 568 ticks
   after the cut the engine refuses "edge 'transfer_line__downstream' leaves
   column … names no draw" — the transfer line running backwards out of the
   column, refused on purpose. **The cause is the furnace**: firing its
   20 MW into a line with no flow (exactly 0 kg/s throughout), its coil
   reaches 549.8 °C the tick before — its tubes fail at 550 °C (M39's
   burn-out), the burst a leak, and the column drains back towards it. Not
   the column's pressure: a check valve between the feed valve and the
   furnace changes nothing (`check_valved_check.txt`, 5 560 ticks), and
   the time is the same at 20 or 86 MW of reboiler. **With the furnace off
   the cut runs the full 20 minutes** (`furnace_off.txt`), at 86 MW (the
   column draining, the naphtha 2.7 kg/s at the end) and at 20 MW (stages
   1–4 not boiling, nothing over the top). What a burst tube with a shut
   feed should do — the column's liquid back down the transfer line to the
   hole, or a check valve at the column — is for the user.

## What it costs

About 25 minutes of machine time in all (the 70 held hours patched and
unpatched ~10 min, the 200 random ~6 min, the corpus twice, the variants).

## Part B — the empty column, questions first

Read off the code (the advisor's list, DESIGN §79.9): the level loops'
outputs are multiples of the first tick's settled flows (the design point —
it can still be solved at load as a calculation, the state then set empty);
the reflux is R times the distillate, so a dry drum sends none and Kister's
total-reflux start (§12.8 pp. 329–330) is not expressible; an empty stage has
no state (no moles, no composition: "ran out of liquid"); and what the
column is fed while it fills. Kister §12.1 (pp. 313–314): pressure, heating,
feed, heating and cooling sources, then rates. M74.0's hand estimate: 60 t to
fill (10 minutes of feed), 36 GJ to warm the liquid and ~12 GJ the steel
(~77 t estimated).
