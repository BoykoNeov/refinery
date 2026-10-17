# M64.0 probe — steam stripping on the steamed crude unit (2026-10-17)

Throwaway, applied to `58fd374` (M63 closed) in a detached worktree:

- `probe_held.patch` — `Specification::Held(duty)` in the column Newton (the
  reboiler stage's own energy balance an equation with its duty held, water and
  steam terms included; the reflux an unknown; no vapour floor anywhere); the
  heat-balance cascade solving under it whenever the probe holds a duty; a cold
  start by continuation (`probe_continuation`); rounding at zero clamped in the
  published reboiler closure. Plus `probe.rs` (the switch: `crates/solvers/src/`)
  and `m64_walk.rs` (the walk: `crates/scenarios/tests/`, registered in
  `suite.rs`).
- `crude_unit_stripped_probe.toml` — `crude_unit_steam.toml` with its furnace at
  68 MW, its coil starting at 488 °C and its pumparounds at zero.

Run (Linux, 4 cores, release, `nice`):

```
[PLANT=<abs path>] [PA_SHARE=0.6] [DQ=5] [STEP_TICKS=300] [SETTLE=3000] \
[OPENINGS=a,b,..] [FURNACE_WALK=a,b,..] [BASE_FURNACE=MW] [COLD=MW PA=a,b,c] \
cargo test --release -p refinery-scenarios --test suite m64_walk -- --nocapture
```

Timing on this machine reads about 2.4 × the Windows box's: the shipped
`crude_unit_steam` at steady state is 2.9 ms a tick here against 2.41 ms there
(DESIGN §67.21), with tick 1 at 55–62 ms.

## Results

1. **The reboiler walks to zero, warm.** From `crude_unit_steam.toml` at tick
   600 (reboiler 87.96 MW, furnace 20, pumparounds 15/15/10 MW), in 5 MW steps of
   300 ticks, each step taking 60 % of the reboiler's drop off the pumparounds and
   the rest onto the furnace: every step converges. Pumparounds reach zero at a
   reboiler of 18 MW; the furnace ends at 68 MW. The published closure equals
   the held duty to the printed digit at every step (82.96 against 82.96), and
   at zero rounds to −2.7e-7 W — which `column_duties` refuses as negative
   (clamped in the probe: M64.1 must decide how a held zero is published).

   ```
   reboiler MW   87.96  58    28    18    8     3     0     0 settled (t18000)
   furnace MW    20     32    44    50    60    65    68    68
   bottom °C     580.2  574.4 553.5 528.6 464.2 402.8 359.4 383.1
   reflux        4.00   3.45  3.33  3.25  3.09  3.18  3.36  4.81
   feed flash q  0.664  0.713 0.791 0.817 0.849 0.865 0.880 0.981
   charge kg/s   112.83 110.70 106.79 105.21 103.04 101.78 100.56 93.64
   coil °C       322    339   370   383   401   412   423   488
   ```

   The bottom at "0" (359 °C) is not yet settled: the coil takes ~1 200 s of sim
   time to come up, the feed heats, more of it flashes, and the settled bottom
   is 383 °C.

2. **The bottom against the steam**, reboiler zero, furnace 68 MW, each opening
   600 ticks (the coil still creeping, so read the trend more than the digit):

   ```
   valve opening   0.1    0.25   0.4    0.5    0.75   1.0
   steam kg/s      0.217  0.542  0.867  1.083  1.620  2.151
   bottom °C       396.1  387.6  382.5  378.0  371.2  366.1
   reflux          4.37   4.53   4.75   4.71   4.87   5.00
   ```

   M62.0's probe on `crude_unit.toml` read 390.6 / 378.8 / 370.5 / 361.4 at 0.1 /
   0.5 / 1 / 2 kg/s: the same stripping, 5–8 K hotter here (a wetter, more
   flashed feed). A real atmospheric bottom reads about 340–370 °C.

3. **The furnace's range**, reboiler zero, steam 0.867 kg/s, each step 1 500
   ticks: 80 MW runs (coil 512 °C, feed 471 °C, flash 1.000, charge 91.4 kg/s);
   68 → 40 MW runs, the bottom 383 → 349 °C, the coil 488 → 405 °C; **at 35 MW
   the gas-oil tray (stage 7) runs dry**, and the refusal names it. The coil
   stays under its 550 °C limit everywhere the column runs.

4. **The charge falls as the furnace fires.** No loop holds the charge: the
   feed valve is wide open and the rate is what the hydraulics pass. A coil
   that boils harder drops more pressure, so 20 → 68 MW takes the charge 112.8 →
   93.6 kg/s, and the heat per kilogram rises with it: the feed flashes 0.98 of
   its moles at 68 MW and all of them at 80. A real unit holds its charge on a
   flow loop and flashes about 60–75 % at the flash zone, its residue's heavy
   end never vaporising. Here the column still runs — the stripping section
   below the feed carries 8–18 mol/s of oil vapour over the steam — but the
   furnace number that gives a realistic bottom gives an unrealistic flash.

5. **The cold start.** On the shipped file's own start (coil 322 °C, furnace
   moved to 68 MW at tick 0) M61's warm-up refuses on tick 1 — a negative
   vapour into stage 8 — and the continuation (a base column keeping its
   reboiler and the shipped 15/15/10 MW pumparounds, then 10 Newton solves
   walking the pumparounds and the held duty to the plant's) fails at its
   9th step, 8 MW: at tick 1 the coil has not heated anything, the feed
   arrives near 290 °C, and a 290 °C feed cannot run without its reboiler —
   no start-up method fixes that. **On a plant that starts hot**
   (`crude_unit_stripped_probe.toml`, coil at its settled 488 °C) the
   continuation starts it: base reboiler 41.8 MW, 10 steps, about 15 ms each
   time it runs (nine times within tick 1, the engine's sweeps); **tick 1
   183–186 ms** against the shipped plant's 55–62; it lands on the warm walk's
   answer — bottom 383.0 °C, reflux 4.809, against 383.1 and 4.810.

6. **Cost.** The held solve itself costs what the declared one does: held at
   the plant's own 87.96 MW, 5.70 ms a tick against 5.54 unheld over the same
   ticks. What costs is the coil's drift (ledger A29): the SHIPPED plant, its
   reboiler kept, its furnace stepped 20 → 30 MW, runs 14.7 ms a tick while its
   coil drifts and 8.6 ms twenty minutes later, against 2.9 ms at rest. The
   stripped probe plant started at its settled state runs 12.0 / 9.6 / 7.0 ms
   over ticks 1–3 000 / 3 000–6 000 / 6 000–9 000, its coil creeping 488.0 →
   488.4 °C: a 68 MW coil boiling 98 % of its charge settles slower than a 20 MW
   one, and A29 re-solves every tick it moves.

7. **C14 does not bite.** The pumparounds are at zero at the end of the walk;
   the charge falls 17 % and the column runs. C14 (fixed pumparounds against a
   falling charge on a wetter crude) stays out of M64.
