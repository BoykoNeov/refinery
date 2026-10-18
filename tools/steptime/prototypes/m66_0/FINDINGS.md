# M66.0 probe — the stripped crude unit started from a cold plant (2026-10-18)

Throwaway, applied to `3f1557d` (M65 closed) in a detached worktree:

- `probe_reboiler.patch` — `Command::SetReboilerDuty { node, duty: Option<Watt> }`
  (hold the reboiler at a duty, or free it, while the plant runs; no checks),
  its referent in the bridge, and counters in the heat-balance cascade: warm
  starts that failed, continuations (M64.2), cold starts, halved starts (M65.1).
- `m66_0_probe.rs` — the driver (`crates/scenarios/tests/`, registered in
  `suite.rs`). It builds the plant from `crude_unit_stripped.toml` with its
  reboiler FREE (no `reboiler_duty_mw`), its furnace at FURNACE MW, its coil at
  COIL °C, its pumparounds at PA, its steam valve at STEAM; EVENTS are timed
  commands, RAMPS linear walks, COMPARE a twin engine held at its closure.

Run (Windows, release, below-normal priority):

```
[FURNACE=20] [COIL=322.05] [PA=15/15/10] [STEAM=0.4] [FLOW=simple] [HELD=1] \
[SOURCE_T=270] [TICKS=..] [REPORT=..] [EVENTS=tick:kind:value,..] \
[RAMPS=from:to:kind:target,..] [COMPARE=tick] [RECOVER=free] \
  target/release/deps/suite-*.exe m66_0_probe --nocapture
```

"The cold plant" below is the stripped plant's geometry at the steamed plant's
own state: the furnace at 20 MW, its coil at 322 °C, the reboiler free at the
declared reflux 4.0, the pumparounds at the steamed plant's 15/15/10 MW, the
steam at 0.867 kg/s, the charge on its loop at 112.8 kg/s.

## Results

1. **The cold plant runs on its reboiler.** Tick 1 40–50 ms (seven cold starts
   over the tick's sweeps); the loop brings the charge from 131 kg/s at the
   declared opening to 112.80 by tick 1 500; settled: feed 289.4 °C, flash
   0.664, reboiler 88.0 MW, bottom 580.2 °C, reflux 4.0 — the steamed plant.
   With the steam shut the free column runs too (bottom 586.6 °C, reboiler
   99.0 MW on tick 1), and opening the valve to 0.4 mid-run is followed warm.

2. **Holding the reboiler at its own closure is bumpless**, and freeing it again.
   At tick 1 500 a twin engine holds its reboiler at the published closure
   (88.018024533 MW): the next tick agrees with the free engine to 6e-6 in the
   reflux, 2.6e-5 K on every stage, 7e-5 MW in the duty (the free one drifting
   with the coil, the held one not); no warm start fails, no continuation runs.
   Freed again, the next tick is the free engine's to 4.5e-11 MW (34 ms, more
   Newton iterations; no fallback).

3. **The coil does not care about the column.** Furnace 20 → 58 MW at tick 1,
   the reboiler free: the coil 322 → 391 °C by 150 s, 419 by 300 s, 434 by
   600 s, 436.88 °C settled by about 1 500 s — the stripped plant's own coil to
   the digit, its feed 387.55 °C. The free column's reboiler falls 88 → 59 MW
   (its reflux pinned at 4.0).

4. **The hand-over in one step lands where the stripped plant stands.** On the
   hot plant (coil 436.88 °C), the reboiler held at zero and the pumparounds
   taken off on the same tick: that tick's warm Newton, from the free column at
   reflux 4.0, lands straight on the held column — reflux 3.4604, bottom
   363.633 °C, condenser 52.217 MW — with no continuation. With the pumparounds
   kept (15/15/10) it is refused (stage 3 dry; the walk stops at step 7 of 10,
   17.7 MW); with the steam shut it is refused (the bottom stage's bubble point,
   step 10 of 10).

5. **The edge in feed temperature.** Furnace 20 → 58 MW at tick 1, the step to
   zero (pumparounds off) at tick T: refused at 150 s (feed 346.8 °C) and 200 s
   (357.5 °C), the gas-oil tray (stage 7) named dry; runs at 250 s (365.5 °C),
   300 s, 350 s, and follows the coil up from there.

6. **The edge at every duty** (steam 0.867 kg/s, charge 112.8 kg/s; each plant
   at a furnace duty's settled coil, the pumparounds walked 40 → 0 MW over
   100 s, then the held duty walked from the closure to zero over 600 s; the
   duty at which it was refused, stage 7 dry every time):

   ```
   furnace MW        20     30     40     46     50
   feed °C           289.4  314.7  340.4  356.1  366.7
   lowest duty MW    28.5   20.3   12.0   5.6    0 (runs: bottom 351.4, R 3.04)
   ```

   Roughly 0.37–0.5 MW of reboiler per kelvin the feed lacks under ~362 °C.
   **What the edge is**: every refusal names the gas-oil tray, stage 7, "the
   draws above it taking all the liquid" — its draw a fixed 0.14 share of the
   feed. Too little heat below the feed, and the column cannot make the gas oil
   that share asks for. A real unit would cut its gas-oil draw back and run on
   with that product short; there is no command for a draw's share, so the
   model refuses. Any rule for a hand-over too early stands in for that.
   **The steam moves it**: on a 340 °C feed the lowest duty is 14.8 MW at
   0.22 kg/s of steam, 12.0 at 0.87, 7.3 at 2.15.

7. **Ramps.** Furnace 20 → 58 MW at tick 1; from tick 2 the reboiler held at its
   closure and walked to zero, the pumparounds 40 → 0 MW alongside, over:
   150 s — refused at 133 s, the duty 11.2 MW on a 342 °C feed (the ramp
   outran the coil); 300 s — runs; 600 s — runs. The bottom stays hot until the
   last few MW (581 → 470 °C), then drops to ~355 °C as the reboiler reaches zero.

8. **A refused hand-over need not end the run.** The tick that refuses does not
   advance the engine's clock; freeing the reboiler and ticking again runs, the
   column back on its reboiler (the charge loop has run twice for that tick: a
   refused tick is not rolled back). A refused tick costs 18.6–157 ms (the warm
   attempt and the continuation; 157 ms on the 0.22 kg/s steam case). Today every frontend halts on a refusal (the
   column screen's `_halt`).

9. **Freeing a held zero** (the stripped plant at tick 200): the next tick is the
   free column at the floor — reflux 4.39 (the 20 % vapour floor above the
   declared 4.0), bottom 540.7 °C, reboiler 21.0 MW, no fallback.

10. **Settled, the start-up IS the stripped plant.** Furnace 20 → 58 MW at tick 1,
    the step to zero at 250 s, run to 2 400 s: bottom 363.634 °C, reflux 3.4604,
    condenser 52.217 MW, flash 0.8884, charge 112.800 kg/s, coil 436.88 °C —
    `crude_unit_stripped`'s numbers, on Newton and on the game solver to every
    printed digit. One steady state.

11. **Cost.** Newton: tick 1 ~90 ms with the furnace step, 22–24 ms a tick while
    the coil drifts (the first ~1 500 s), 4 ms settled. The game solver: tick 1
    190 ms, 63–91 ms a tick over the first ~900 s of drift, 11–12 settled
    (ledger A29). A full start-up story is ~15 000–24 000 ticks: a slow_ test.

12. **A start from ambient is out of reach**, and for two reasons. With the
    furnace off and the crude at 200, 100 or 30 °C (the preheat train's 270 °C
    lowered), the free column asks its reboiler for 160–330 MW, the charge loop
    loses the charge (195–311 kg/s), and at 200 and 30 °C the network's solve
    diverges in the coil (a line flash at 1e12–1e23 Pa). Not chased: the plant's
    valve and supply are sized for a boiling coil.

## After the forks: a hold the column declines (the user's decision)

The probe solver's fallback (in `probe_reboiler.patch`, `separate`): the
driver hands the solver the last tick's published reboiler duty (the RUNNING
duty); when the column refuses a held duty that differs from it, it is solved
again held at the running duty, and that answer stands. Switches: `CMD_CONT`
(the continuation tried at the asked-for duty first), `RUN_CONT` (the
continuation allowed when the running duty's warm start fails), `MEMO` (a pass
that declined makes the tick's later passes go straight to the running duty).

13. **A step to zero too early is caught.** Furnace 20 → 58 MW at tick 1, held at
    zero (pumparounds off) at 150 s: the column stays at its running 70.28 MW,
    every pass declining, until the feed reaches ~362 °C (≈ 240 s), and then lands
    on zero by itself; never refused. The reflux climbs 7.60 → 7.97 meanwhile
    (a duty held while the feed heats). Cost a held-back tick: **58–70 ms
    warm-only** against ~20, **160–204 ms with the continuation at the asked-for
    duty** (`CMD_CONT`): not used. The zero lands the same tick either way: no
    step just above the edge was reachable only through the continuation.

14. **The key's walk, the running duty less a step.** Pumparounds off at 50 s;
    from then every 10 ticks the hold set at the running duty less 2.5 MW: the
    running duty tracks the edge as the feed heats (18.3 MW at a 323.6 °C feed,
    13.3 at 340.2, 5.8 at 357.6; finding 6's 12.0 at 340, 5.6 at 356). Warm only,
    at 3.34 MW on a 359.7 °C feed the running duty's own warm start failed (its
    starting point outside the physical region) and the tick was refused.

15. **With the running duty rescued by the continuation (`RUN_CONT`) and one
    decline a tick (`MEMO`)** the same walk reaches zero at ~245 s and runs on
    (bottom 358.9 °C by 400 s, climbing to the stripped plant's): the rescue ran
    4 times in ~1 700 held-back ticks; one failed warm solve a tick; **40–54 ms
    a held-back tick against 23–29** in the same run.

## How much of a start-up needs the column's holdup (C2)

The column is quasi-steady: each tick it is the steady column for that tick's
feed. Everything the probe's start-up shows is set by things that DO have
time — the coil's heat capacity (a ~300 s time constant), the charge loop —
and the order the commands come in. What it cannot show without holdup is the
part of a real start-up before the column has a profile: filling the column
with cold oil, circulating it, the trays loading as the feed heats, products
off-spec to slop until the column lines out, and a column that limps on with a
dry tray rather than halting. A start-up from the steamed plant's warm state
(finding 1) needs none of that; a start-up from ambient (finding 12) needs all
of it, and C2 with it.
