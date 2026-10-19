# M70.3 trace — the settled stripped plant's 4–6 %, 2026-10-20

Asked as "Trace the 4–6 %" (DESIGN §75.3 left it "not traced"). DESIGN §75.6.

**Timing could not see it.** The settled stripped plant, held 2 500 ticks
from its file, the engine before M70.1 (0335994) against after (1abbc0b),
seven rounds alternated on a machine loaded by other work: ticks 1 500–2 500
0.99 (median ratio), 1 000–1 500 1.02, 100–1 000 1.04, single rounds 0.57–1.85.

**Counting could.** `trace_on_*.patch` (throwaway, never committed to the
engine) counts the line flash's flashes and questions (`settle_full`,
`settle_remembered`), per instance, and logs each question the flow solver's
`DensityMemo` could not answer; `driver_main.rs` is the M70.1 driver reading
them. Counts are the same in every round.

| line-flash flashes a tick, the stripped plant held | 100–1 000 | 1 000–2 500 |
|---|---|---|
| before M70.1 | 209.5 | 206.7 |
| M70.1 | 228.3 (+9 %) | 214.0 (+3.5 %) |
| M70.1, no composition guess | 211.1 | 207 |
| M70.1, no parabola | 214.0 | 214.0 |
| M70.1, compositions moved under 1e-12 not guessed | 228.2 | 213.7 |

- 100–1 000: the parabola's 1.09 solves a tick (§75.3), on top of what follows.
- 1 000–2 500: **the composition guess, at the four draw pumps.** The engine's
  own instance asks 127 questions a tick either way; the flow solver's 101–113
  before (it varies), 113 every tick after. All 12 extra are at
  `kerosene_pump`, `diesel_pump`, `gas_oil_pump` and `residue_pump`, three
  each (`misses_tick1003_*.csv`, tick 1 003): the solve asks each pump's
  density at its pressure and the Jacobian's two neighbours (±100 Pa) for the
  pipe INTO it (`*_suction`, read from the column's draw, `column_separation`,
  not guessed) and again for the pipe OUT (`*_discharge`, read from the pump's
  own state, guessed since M70.1). Before, the two were the same bits and the
  memo answered the second; now the guess moves the pump's composition by
  ~2e-9 of a fraction and they differ.
- Not rounding: guessing only compositions that moved over 1e-12 changes
  nothing (213.7).

**The cause is C20's**: a column's draws are neither guessed nor checked, so
the pump just below each draw is guessed alone. Guessing the draws with the
column's feed would close both — and move answers again.
