# M65.0 probe — the wet crude unit on a wetter crude (2026-10-18)

Ledger row C14. Throwaway, against `6fc2274` (M64 closed), Windows box,
release, below-normal priority.

- `make_wetter.py <w…>` — `scenarios/crude_unit_wet.toml` with `w` % water by
  mass in its crude, the oil scaled to `1 − w` (M63.4's recipe). Writes
  `wet_<w>.toml` into the probe's temp folder (paths inside the scripts).
- `make_pa.py <w> <s…>` — that plant with all three pumparounds at `s` times the
  shipped 15/15/10 MW (`wet_<w>_pa<s>.toml`).
- `make_loop.py <w> <opening>` — that plant with its charge on the stripped
  plant's flow loop (Kv 400, setpoint 112.8 kg/s).
- `sweep_cold.sh` — tick 1 at each `(w, s)`, printing where it runs.
- `summ.py <snapshots…>` — one line per snapshot (charge, transfer line, flash,
  duties, coil, bottom).
- `probe_walk.rs` — started cold inside the range, the pumparounds walked warm
  (register it in `crates/scenarios/tests/suite.rs` with
  `#[path = "../../../tools/steptime/prototypes/m65_0/probe_walk.rs"] mod
  m65_probe_walk;`, run `--test suite m65_probe -- --nocapture`).
- `probe_start.patch` — on `crates/solvers/src/heat_balance.rs`: a print of the
  feed's latent heat flow; `M65_WARMUP_STOP` (the cold warm-up stops at its last
  physical iterate rather than refusing); `M65_CONT` (an unheld column whose
  cold start refuses starts again from its pumparounds halved, up to four
  times, and walks them back to its own in `CONTINUATION_STEPS` Newton solves).

## Results

1. **Reproduced.** 0.3 % (shipped) and 1 % run; 2, 3, 5 and 10 % halt on tick 1,
   the cold warm-up refusing "the draws above the reboiler carry …". The charge
   falls as the supply boils harder through the fixed valve:

   ```
   water %          0.3    1     2     3     5     10
   charge kg/s     112.8  91.1  76.1  67.2  56.6  43.7   (tick 1)
   ```

2. **The cold start's range is narrow** (tick 1, pumparounds as a share of the
   shipped 40 MW; 0.05 steps):

   ```
   water %    runs at
   0.3        0.40 – 1.30
   1          0.40 – 1.10
   2          0.35, 0.45 – 0.95
   3          0.45 – 0.85
   5          0.50 – 0.75
   10         0.55 – 0.65
   12–15      0.50/0.55 – 0.60
   20         none (see 8)
   ```

3. **Warm, there is no range.** Started cold inside it (0.7 at 2 %, 0.6 at 3, 5
   and 10 %), run 1000 ticks, then walked: up by 0.1 (4 MW) every 300 ticks to
   3.0 (120 MW), and down by 0.05 to zero. Every step ran at every water share.
   **C14 is the cold start, not the column**: not the water, not the charge,
   not the pumparounds.

4. **No multiple of the feed's latent heat flow lands in every range.** The
   feed's latent heat flow (M64.2's base is 1.5 times it) on tick 1's last pass
   is 16.2, 14.2, 11.7 and 8.6 MW at 2, 3, 5 and 10 %; the ranges in those
   units are 0.86–2.34, 1.27–2.39, 1.71–2.56 and 2.56–3.03. The low edge sits
   near 20 MW whatever the feed; the high one falls with it. A feed-derived
   base (or "scale the pumparounds with the feed", the ledger's cure) does not
   fit: proportional to the charge it misses 10 % (0.39 against 0.55–0.65).

5. **The warm-up stopped at its last physical iterate** (`M65_WARMUP_STOP`)
   removes the LOW edge: tick 1 runs from 0.2 at every share to 20 %. The high
   edge stays: Newton from that iterate refuses its starting point.

6. **The pumparounds halved, then walked back** (`M65_CONT`, without the stop)
   starts every share 0.3–20 % at the shipped 40 MW (from half) and at 80 MW
   (from a quarter). Zero pumparounds is not a base that always starts (it
   refuses at 0.3, 2 and 5 %). Only on a refused cold start: the shipped 0.3 %
   and 1 % plants never walk. Tick 1: 108–114 ms (five passes walk) against the
   shipped plant's 95–102.

   Run 12 000 ticks at the shipped 40 MW, both fidelities, identical to every
   printed digit (Newton 121–137 s, game solver 233–271 s for the run):

   ```
   water %                   2       5       10
   charge kg/s              75.1    55.4    42.5
   transfer line, vapour    59 %    73 %    89 %   by mass (tick 12 000)
   column feed flash        0.825   0.922   0.981  molar
   coil °C  t6000/t12000    338/340 358/363 380/394   (limit 550)
   reboiler MW              64.6    50.9    41.8
   bottom °C                587     588     588
   ```

   2 % has settled; 5 % and 10 % still creep (the furnace's 20 MW on a charge
   less than half the shipped). 10 % flashes nearly all its feed.

7. **The charge on a flow loop does not hold it.** The stripped plant's loop on
   a Kv 400 valve: 2 % passes 107.3 kg/s at 0.6 open; fully open, 5 % passes
   97.5 and 10 % 75.2 (and these start at the shipped 40 MW without help). A
   two-phase supply chokes the valve; the loop cannot reach 112.8 past ~3 %.

8. **M63.4's flood rule halts the crude unit.** At 15 % and 20 % without the
   fix, the column refuses, the water share passes 15 % and the column floods —
   then the pumped naphtha line refuses its 0 kg/s ("arrives at 0.81 bar against
   the store's 1.05 bar"): a draw line's pump cannot hold an empty line against
   its store (§67.13). M63.4 measured the rule on a tank-fed column with no
   pumped draws. With the fix (6) the column separates at 15 and 20 % on tick 1
   instead (not run long). A gap of its own, for the ledger.

9. **The shipped 0.3 % plant has the same narrow cold range** (16–52 MW). It
   touches C12 (the screen's keys) and C15 (a cold plant) without being built
   for them.
