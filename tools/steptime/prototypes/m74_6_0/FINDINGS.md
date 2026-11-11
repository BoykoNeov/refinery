# M74.6.0 — the start from empty, its probe (2026-11-11)

The record is DESIGN §79.14. After M74.5 the user said yes to part B of the
start from empty's probe (§79.9's decisions 2–4: a tray with no vapour draining
through its holes, each tray's steel a heat store, a total-reflux start). Done
in the order the advisor set: the steel first (it is what makes an empty stage
solvable), the empty stage's branches, C29's held-back draining as the first
gate, then the start. **No engine code changed**; the prototype is
`prototype.patch` (on 6d3ad27, M74.5's `draining.patch` included), every
change behind a switch.

## The prototype (`prototype.patch`)

Switches (`crates/solvers/src/probe.rs`, environment, probe only):

- `M74_6_0_STEEL=1` — **each stage's steel a heat store**, lumped at the
  stage's temperature: `m·cp·(T' − T)/dt` added to the heat its liquid stores.
  The mass from the trays' geometry (M74.0's game estimate): a 15 mm shell over
  the tray spacing and a deck at 60 kg/m²; the base a 15 mm shell around its
  volume and a head; carbon steel at 500 J/(kg·K). On the holdup plant 59 t on
  the trays, 13 t at the base: 36 MJ/K in all (stage 11, the feed's, 2.1 MJ/K).
- `M74_6_0_DRAIN=1` — C29's draining (`../m74_5/draining.patch`); off,
  nothing weeps.
- `M74_6_0_EMPTY=1` — **the start from empty**: the first tick's settled solve
  kept for the level loops' references and as each stage's label, the state then
  emptied, every stage's steel and the drum at 20 °C.
- `set_total_reflux(bool)`, per thread — **the total-reflux start**: the
  distillate and the side draws shut, all the drum's level loop asks sent back
  as reflux (`(R + 1)` times its output, so the drum's outflow is unchanged
  when the switch goes back); the bottoms left on the base's level, the drum's
  overflow past full still leaving as distillate.

Always on, each only where a tick is refused today (so the corpus at rest is
untouched — checked, below):

1. **A stage or the drum holding nothing** reads the last liquid it held as a
   label (no moles); a Thomas row with nothing held, coming or going holds
   none; an empty drum carries its temperature; `finish` keeps the label.
2. **A tray's slope** read over 1 kg when it holds none.
3. **The liquid going down capped** so a tray ends the tick at exactly nothing
   rather than below it ("a tray gives what it has", within the tick).
4. **A start the state cannot reach** (a stage emptied past nothing) restarts
   Newton from the state's temperatures and no vapour.
5. **The Jacobian's difference** on an empty stage's vapour (at its bound,
   where any vapour empties it past nothing) is allowed a shortfall of twice the
   difference, read as empty. Found on the first empty start: without it every
   step from empty was refused "stage 1 ran out of liquid".
6. **The bottoms pump loses its suction** below the base's low level, as the
   reboiler's tubes uncover (scheme A's fixed bottoms would otherwise draw an
   empty base).

## The driver (`m74_6_0_empty.rs`)

M74.5.0's, with the key `tr` (total reflux 1/0), each stage's liquid [t] and
the products in the trace, and each run's stages holding exactly 0 kg counted
(ticks, most at once, which, last tick, least any tray held). Put in
`crates/godot-ext/tests/`, registered in `suite.rs`, `refinery-solvers` a
dev-dependency (`set_total_reflux`).

## Findings

1. **Every shipped plant byte-identical with every switch off**, 70 of 70 on
   Newton and 70 of 70 on the simple solver, on the final code
   (`corpus_final_*.txt`); `refinery-solvers --lib` 176 of 176,
   `column_holdup::` 26 of 26, `column_screen::` 14 of 14, the weeping reference
   case 5 of 5. **The steel on moves only `crude_unit_holdup`**
   (`corpus_steel_*.txt`, both fidelities) — any plant holding liquid would
   move, so the steel is a per-plant declaration, off unless asked for.
2. **Without steel an empty stage cannot be solved** (`start_nosteel.txt`):
   the start from empty refused on its first step (stage 11, the feed's, short
   69 MW after one Newton step) — with no liquid and nothing flowing, nothing
   in a stage's energy balance depends on its temperature.
3. **The column starts from empty and lines out** (`start_steel*.txt`, the
   holdup plant, the crude at its file's 104 kg/s and 270 °C preheat, the
   reboiler held at the file's 86.7 MW, scheme B), an hour each:
   - the hot feed lands on stage 11, part flashes; the vapour condenses on the
     cold steel above, warming the trays one by one from the feed up — the top
     tray reached about 3½ minutes in; **up to 12 stages hold exactly nothing at
     once**, the last empty 3.0 minutes in, every stage boiling from 3.1 minutes;
   - **the drum 10 % full at 4 minutes, the side products from 6, the base 10 %
     at 15** (its reboiler cut by the base's level until then); **the four top
     products within 2 % of their end from 21 minutes**;
   - **at 60 minutes the bottoms and the base's level still swing** (the base
     0.48–0.59 over the last 20 minutes, the bottoms 31.5 kg/s against the
     running column's 29.0): the base's level loop, slow on a 115 m³ base;
   - against the running column (`design.txt`): every tray's liquid and
     temperature within a few per cent, the top four products to the digit;
   - **the base overshoots by about 20 K**: 613 °C at its peak against the
     running column's 592 °C (the residue heavy; the plant's reboiler, not a
     crude unit's stripping steam);
   - M74.0's hand estimate set against it: 10 minutes to fill (the base 10 %
     at 15, the four top products at 21), 7–9 minutes to warm at the reboiler
     (the top reached at 3½; the steel's heat in from the feed and the vapour,
     not only the reboiler's);
   - the slowest tick 76 ms (four runs at once).
4. **The reboiler shut for the first 10 minutes, then 86.7 MW**: runs, the
   base 10 % full at 5 minutes, the four top products settled from 34 minutes,
   the base's peak 595 °C.
5. **Total reflux for 20 minutes, then the normal loops**: runs; the side
   products from 21 minutes, the four top products settled from 50.
6. **Total reflux for the whole hour, the crude still coming in, does not hold**
   (`start_steel.txt`, `tr=1`): the drum and the base full, trays 1–11 flooding,
   the condenser at 114 MW, the drum's overflow leaving as 15 kg/s of naphtha,
   87 kg/s down the bottoms — with the side draws shut everything the feed
   brings goes up or down. Kister's total reflux has no feed (§12.8): see the
   decisions.
7. **The draining's gate (C29), measured, and what it does not show**:
   - the 70 held runs (M74.4's corners, an hour each): draining alone 49 of 70
     (`holds_drain.txt`, before change 5 above — the stage-3 refusals partly
     the Jacobian's; read as neither way), **steel alone 70 of 70**
     (`holds2_steel.txt`), **draining and steel 65 of 70**
     (`holds2_drain_steel.txt`);
   - **in none of the 140 steel runs did a tray reach 0 kg** — the least held
     1.7 kg: the steel keeps stage 3 boiling, so the tray over it does not dump
     and stage 3 does not empty (unpatched, stage 3 stopped boiling and emptied).
     The steel cured the refusals; **a wet tray emptying to exactly nothing
     within a run is not shown** (change 3 untested);
   - the 5 left are one kind, **the base at 0 MW with the reflux at 15** (stage
     13's balance short by under 0.001 MW, Newton without descent after 2–8
     steps): the draining's (steel alone runs all 70) — M74.5 §79.12's stall.
8. **Shut down with the draining, the trays go dry and the step stalls**
   (`shut_*.txt`, the valved holdup plant, feed, furnace and reboiler shut at
   once): the column cools on its stored heat, the trays drain to under 50 kg
   each over 15 minutes (never exactly 0 — the weeping slows as a tray empties),
   and at 16 minutes Newton stalls on stage 11's bubble point (−1.5·10⁻⁴) as
   stage 12 stops boiling over trays holding grams (`shut_fine_trace.txt`); the
   sum-of-squares fallback does not rescue it. Without the draining the same
   shut-down runs (the trays held at their weirs, 960 kg the least). **The
   candidate, for the building slice**: a third branch for a stage that boils
   dry — its vapour capped at what arrives (`V ≤ held/dt + arriving`, the
   complementarity a mid-function on `[0, available]`), its temperature then
   its energy balance's.

## Not checked

- **The start from empty on the simple solver** — the driver runs the plant on
  its file's solver.
- **The first tick after load still publishes the settled solve** (the design
  point, one tick of 0.1 s); the building slice steps it from empty.
- **The steel-alone batch's slowest tick, 391 ms**, measured while the starts
  ran beside it (the draining-and-steel batch's, alone, 94 ms).
- The furnace, the transfer line and every other unit start at their design
  state: only the column starts empty.

## What it costs

About 40 minutes of machine time: the four batches of 70 held hours (~5 min
each on 12–14 cores), the corpus six times, the starts and shut-downs
(~3 min), the builds.

## Decisions for the user

1. **Total reflux with the crude coming in** (finding 6): the feed off during
   total reflux (and so the furnace, C28), the feed cut back, or the mode as
   prototyped (the bottoms on the base's level, the drum overflowing to the
   naphtha) — which ran 20 minutes then the normal loops (finding 5).
2. **The steel's mass**: declared in the plant file, or derived from the trays'
   geometry as here; either way off unless a plant asks for it, on a new plant
   file (finding 1; it moves which stages stop boiling in M74.5's key-8 corners).
3. **The draining (C29)** cannot ship on this evidence: the empty tray it needs
   is not shown within a run (finding 7) and the dry tray it makes stalls the
   step (finding 8) — the third branch first.
