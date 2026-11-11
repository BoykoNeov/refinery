# M74.6.1 — the user's decisions on M74.6.0, and the steel's guess checked (2026-11-11)

The record is DESIGN §79.15. M74.6.0 put four questions to the user (total
reflux with the feed on, the steel declared or derived, the draining after the
third branch, and — from a side note — whether the steel's guessed mass should
be rerun at half and double). The user's answer: **"do what is closer to
reality"**. This applies it to each, with the evidence, and runs the checks the
advisor asked for. **No engine code changed**; `prototype.patch` is
M74.6.0's plus one switch, `M74_6_1_STEEL_SCALE` (every stage's steel mass
times it, default 1), on dc03fce.

## The sources

- **Kister, *Distillation Operation* §12.8 (pp. 329–331)**: a total-reflux
  start-up runs the column, reboiler and condenser on a charge whose
  composition is like the feed's, then admits the feed. It is "least attractive
  when the ratio of reflux to feed is low (in such cases, most of the
  stabilizing can only be performed after feed is introduced)" — a crude
  unit's case. §12.8 also: columns are operated on total reflux during feed
  interruptions (minutes to days).
- **Kister §12.1 (pp. 313–314)**: the start-up order — heating and cooling,
  then the feed, then the heating and cooling sources, then the rates.
- **Ludwig Vol. 1, eq. 11-55 (book p. 328)**: a vessel's weight from its shell,
  `480·π·D·t/12·(L + h + R)·1.10` [lb; D ft, t in] — the shell term as here
  (steel's density times the shell's area times its thickness).
- **The deck's 60 kg/m²**: no source found in the library (searched every text
  book for tray weights, gauges and deck thickness). A 10-gauge (3.4 mm) carbon
  steel deck alone is 27 kg/m²; supports, beams, downcomers and weirs add to
  it. Read as a game value, checked by the scale runs below.

## Runs (`run.sh`, all at below-normal priority, about 15 minutes of machine time)

| run | switches | specs |
|---|---|---|
| `start_x{0.5,1,2}` | empty, steel × 0.5 / 1 / 2 | `tr=0` an hour; total reflux 20 min then normal |
| `interrupt`, `interrupt_nosteel` | the valved holdup plant at design, steel on / off | feed and furnace shut with total reflux, the reboiler at the file's 86.2 MW or held at 40 MW, 30 min; the same then the feed and furnace back, 30 min |
| `holds_drain_steel_x{0.5,2}` | draining and steel × 0.5 / 2 | M74.5.0's 70 held hours (`../m74_5_0/holds_specs.txt`) |

`settle.py` reads the traces (`settle.txt`); on M74.6.0's trace it gives
M74.6.0's numbers back (21 min, 613 °C, 4 / 6 / 15 min).

## Findings

1. **The rebuild reproduces M74.6.0**: steel × 1, the same 1779 ticks with a
   stage empty, the same products to the digit.
2. **The start from empty does not hang on the guess** (`start_x*`):

   | steel | stages empty, the last at | drum 10 % | side products | base 10 % | top four settled | base peak |
   |---|---|---|---|---|---|---|
   | × 0.5 | 1.5 min | 3 min | 5 min | 16 min | 21 min | 613 °C |
   | × 1 | 3.0 min | 4 min | 6 min | 15 min | 21 min | 613 °C |
   | × 2 | 5.2 min | 6 min | 6 min | 4 min | 20 min | 613 °C |

   It runs at all three. Heavier steel keeps the trays empty longer (more heat
   to put in before they hold liquid) and fills the base sooner (the vapour
   condensing on the cold steel runs down); the products settle at the same
   time, set by the base's level loop and the products' own loops. Total
   reflux 20 minutes then normal: settled from 49–50 minutes at every scale.
3. **The draining's 65 of 70 does not hang on the guess**
   (`holds_drain_steel_x*`): 65 of 70 at × 0.5, × 1 (M74.6.0) and × 2, the
   same 5 failing (the base at 0 MW with the reflux at 15, stage 13's balance
   short, M74.5 §79.12's stall), only when they fail moving. The least any
   tray held: 0.5 kg at × 0.5, 1.7 kg at × 1, 2.0 kg at × 2 — **no tray
   reaches exactly nothing at any scale**, so M74.6.0's finding 7 stands: a wet
   tray emptying within a run is not shown, and the draining still waits on
   the third branch.
4. **The feed interruption on total reflux runs** (`interrupt*`): the column
   full and at design, the feed and furnace shut, the distillate and side
   draws shut:
   - steel on, the reboiler left at 86.2 MW: runs the 30 minutes, the drum
     held at 0.50, the base falling to 0.36 (its bottoms closing on its level),
     trays 9 and 12 flooding (the reboiler's vapour with no feed to heat), the
     base peaking at 608 °C;
   - steel on, the reboiler held at 40 MW: runs, nothing floods, trays 1–10
     and 12 weep, the base peaking at 565 °C;
   - the feed and furnace back after 30 minutes, total reflux off: the side
     products from 31 minutes, the four top products settled from 34 — four
     minutes after the feed (Kister: on feed introduction "the products
     usually stay on-spec");
   - **steel off** (the shipped step), the reboiler at 86.2 MW: refused 54 s
     in, the heat-balance cascade's trays' pressure drops not settling; held
     at 40 MW it runs as with the steel.

## The decisions, applied ("closer to reality")

1. **Total reflux is a product switch, not a feed switch**: the distillate and
   the side draws shut, the drum's level on the reflux, the bottoms left on
   the base's level; the feed is the operator's valve as now. A start from
   empty keeps the crude coming in (a crude unit cannot stabilise without its
   feed, Kister p. 331); held too long with the full feed, the column floods
   (M74.6.0 finding 6) — a real operator's mistake, left to happen. With the
   feed shut it is the feed-interruption mode (finding 4). This reverses
   M74.6.0's suggestion (the feed off on total reflux).
2. **The steel derived from the trays' geometry, with an optional per-plant
   override** of its mass; off unless a plant asks for it, on a new plant file
   (it moves any plant that holds liquid). The guess is not load-bearing for the
   start or the held runs (findings 2–3).
3. **The draining after the third branch** (a stage that boils dry): finding
   3 confirms no tray empties within the held runs at any scale.

## Not checked

- The start from empty on the simple solver; the first tick after load still
  publishing the settled solve (M74.6.0's list).
- **Only the column starts empty**: the crude arrives at its full 270 °C
  preheat and the furnace at its design state from the first tick. A real crude
  unit warms its heater slowly (hours, not 21 minutes) — outside M74.
- Why the shipped step refuses the feed interruption at full reboiler without
  steel (finding 4) — the pressure-drop cascade, not the holdup step's Newton.
