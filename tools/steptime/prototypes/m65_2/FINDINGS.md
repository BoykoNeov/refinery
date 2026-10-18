# M65.2 probe — the flooded crude unit's pumped lines (2026-10-18)

Docs: `docs/DESIGN.md` §70.2. Plants: `../m65_0/make_wetter.py <w>` (the wet
crude unit with `w` % water). Release, Windows box, on M65.1 (`d24dc71`).

`probe_flood.patch` (on `crates/solvers/src/network.rs` and
`heat_balance.rs`) holds three throwaway rules behind switches:

- `M65_HOLD` — an empty draw's line reads its pipe's stored stream, not the
  separation's draw;
- `M65_IDLE` — a draw line carrying nothing is not refused when it cannot
  reach its store;
- `M65_BUBBLE` — the flooded oil to the bottoms at its own bubble point at the
  column's pressure, the empty draws that liquid, the condenser the difference.

## Results

1. **Where the crude unit floods, after M65.1**: it separates to 40 % water
   and floods from 50 % (M63.4's finding 2, on the plant this time). Before
   M65.2 the flooded tick halts on the naphtha line ("cannot deliver its 0.0000
   kg/s … arrives at 0.81 bar against the store's 1.05").
2. **`M65_HOLD` and `M65_IDLE` both fail one line on**: the RESIDUE line, its
   draw the whole oil at the feed's liquid-equivalent ~650 K, a boiling stream
   at 1.8 bar — 0.58 bar against 1.06 on tick 1 (HOLD); −164 537 bar at
   11.04 kg/s on tick 2 (IDLE).
3. **`M65_BUBBLE` runs**: 50 % and 80 % water flood every tick for 600 ticks,
   the oil out at 155.6 °C, the condenser 4–11 MW more.
4. **The built rule (a flash zone, DESIGN §70.2)** at first failed at 50 % on
   tick 172 and at 60 % on tick 1: once every drop of oil flashed, the empty
   lines stood full of the overhead's composition at the FLASH temperature, a
   vapour at the column's pressure (the march read 15.6 kg/m³). Standing full
   of the condensed overhead at its bubble point, every share runs (1 000
   ticks each):

   ```
   water %       2     10    15    20     30     40     50     60     80
   column       sep   sep   sep   flood  flood  flood  flood  flood  flood
   charge kg/s  75.7  43.4  36.9  32.7   27.5   24.2   21.9   20.2   17.7
   overhead     -     -     -     21.3   17.6   14.5   11.0   8.1    3.5   kg/s of oil
   bottoms      -     -     -     4.8    1.7    0      0      0      0     kg/s
   ```

5. **The halvings on a flooded feed cost half a second a tick**: 50 % water,
   20 ticks in 10.5 s with M65.1's halvings, 0.4 s without. Hence decision 4 of
   §70.2: a feed wet enough to flood is not retried.
