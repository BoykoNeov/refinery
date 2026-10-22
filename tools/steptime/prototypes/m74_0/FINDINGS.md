# M74.0 — a column that holds liquid (C2): the probe, 2026-10-22

Asked after M73 as "what is next"; the user chose 1, column holdup (C2), and
for its scope "start from empty too": the running column that takes time to
answer a key, then the start from an empty column. No engine code changed.

## The probe

- `probe_holdup.patch` (on d381fd1, `crates/solvers/src/heat_balance.rs` and
  `trays.rs`): after each converged heat-balance solve, every stage's
  temperature, pressure, liquid mole fractions, K-values, vapour, liquid in /
  down / drawn and loop circulation, the reflux and the distillate's
  composition; and per tray its active and downcomer areas, weir, crest over
  the weir, downcomer backup, liquid density and load. Kept per thread as
  JSON, on only with `PROBE_HOLDUP` set; `probe_last_holdup()` reads it.
- `probe_driver.patch` + `m74_0_holdup.rs`: M73.4's grid driver
  (`tools/steptime/prototypes/m73_4/`), printing each run's column on the
  tick before its first move (`BEFORE`) and at its end (`AFTER`). Put
  `m74_0_holdup.rs` in `crates/godot-ext/tests/`.
- Run: the tray plant (`crude_unit_trays.toml`) at the file's settings, 3 000
  ticks, one key moved, 20 000 ticks more: the reflux 4 → 2, 3, 5 (and 4, a
  control), the furnace 20 → 15, 25 MW, the loops' flow ×0.5, ×1.5, their
  return temperatures ±30 K. Ten runs, 19 s (`steps.out`, `steps_results.txt`).
- `analyze.py <steps.out> [base s] [drum s] [deck scale]`: per component, the
  composition model with holdup, `H_j·dx_j/dt = in − out`, linear, the
  AFTER state's flows and K-values frozen; one state per stage and one for
  the reflux drum (a total condenser: the drum's liquid is the reflux and the
  distillate). The feed is the AFTER state's steady residual per stage — it
  lands on stage 11 alone (the feed tray), which checks the flows read.
  The slowest time constant per component, and the products' response from
  the BEFORE compositions, exact and by implicit Euler at dt = 0.1 s, the
  time to 90 % and 99 % of the move.
- Holdups: a tray holds its clear liquid on the deck, `ρL·A_active·(h_w +
  h_ow)` — Sinnott's head, the one M73's pressure drop uses — plus its
  downcomer's backup, `ρL·A_dc·h_b`. The base and the reflux drum hold
  5 minutes of the liquid leaving them (Ludwig Vol. 1, 4th ed., "Rules of
  thumb: summary", p. xix, towers rule 12: "Reflux drums usually are
  horizontal, with a liquid holdup of 5 min half-full"; p. xxiv, vessels rule
  5: 5 min for reflux drums, 5–10 min for a product feeding another tower).
- `startup.py <steps.out>`: hand estimates for a start from empty, and the
  heat a running column's liquid stores as its temperatures move.

## Findings

1. **What the column holds** (`analysis_*.txt`, first table): each tray
   2.0–4.8 t (deck 1.0–2.6 t, downcomer 0.9–3.3 t), 14–23 s of its liquid
   load; the twelve trays 44.8 t. The base 8.7 t and the reflux drum 34.2 t
   at 5 minutes each (the drum carries the reflux too: at a reflux of 4 it
   passes five times the distillate). 87.7 t in all, 14 minutes of feed.
2. **How slowly the products answer a key** (`analysis_base300_drum300.txt`):
   the side products move first — 90 % of the way in 1–4 minutes after a
   reflux or loop-return move, 99 % in 20–50 — and the top product last: 90 %
   in 23–43 minutes, 99 % in about 1–2 hours. A move of the loops' flow is
   slower on the kerosene draw (90 % in 18–20 minutes), the tray the top
   loop draws from too. The slowest time constant is
   16 minutes at a reflux of 2, 27 at 4 and 32 at 5, on the light cuts.
3. **The reflux drum sets the top product's pace.** The drum's liquid leaves
   the column only as distillate, a fifth of what passes through it at a reflux
   of 4, so its 34 t turn over in about 25 minutes. With 1-minute drum
   and base (`analysis_base60_drum60.txt`) the slowest time constant falls to
   12–14 minutes, set by the trays and the loops' recirculation; at 10 minutes
   (`analysis_base600_drum600.txt`) it is 41–61. Half the deck liquid
   (`analysis_deck_half.txt`, an aerated deck holds less clear liquid than
   Sinnott's head) moves the slowest time constant by about 2 %, because the
   drum dominates. **So the drum and base sizes are the choice that matters**;
   the tray holdup comes out of M73's geometry and barely matters.
4. **The furnace moves no product** on this reboiled column (under 0.1 points;
   the reboiler makes up the difference, as M73.0 found for the trays).
5. **The settled column is today's.** The model with holdup settles on the
   AFTER state to 2e-6 or better in mole fraction (to 2e-11 on the furnace
   steps): holdup changes the path, not where it ends. So the shipped plants'
   steady answers, and the screen's measured tables, carry over — a check, not
   an assumption, for the building slices.
6. **The tick is fine-grained enough.** Implicit Euler at 0.1 s against the
   exact response: at most 1.1e-4 in mole fraction, against time constants
   of 4–60 minutes. The holdup term `H/dt` outweighs a tray's flows about a
   hundredfold, so each tick's composition step is small and well
   conditioned.
7. **Heat stored in the liquid can be left out of a running column.** As a
   key moves the stage temperatures (up to 12 K at a reflux of 2), the trays'
   and base's liquid stores or gives up 150–560 MJ — under 1 MW spread over
   10 minutes, against 45–110 MW of condenser and reboiler duty. Leaving it
   out keeps each stage's energy balance as it is today.
8. **The cost, estimated**: the probe's ticks ran 0.65 ms each at the file's
   settings and 0.8 ms on the furnace steps, where the column re-solves a
   slowly drifting feed for many minutes — the nearest thing today to a
   column whose compositions drift every tick. Measured when built.
9. **A start from empty, by hand** (`startup.py`, `startup.txt`):
   - filling the column's 88 t at the 104 kg/s feed: 14 minutes (the trays
     alone 7);
   - warming it from 20 °C: the liquid about 37 GJ, the steel (77 t: a 15 mm
     shell on the file's diameters, decks at 60 kg/m², a game estimate) about
     12 GJ — 41 minutes at the furnace's 20 MW, 8–9 minutes at the reboiler's
     87–107 MW. Real start-ups are paced by their operators, and slower.
   - **the order** (Kister, *Distillation Operation*, 1990, §12.1 pp. 313–314;
     §12.8 pp. 329–330): pressure up, heat, feed, the heating and cooling
     on, up to rate; "one of the most powerful, and often one of the least
     troublesome" starts is on **total reflux**: the column, reboiler,
     condenser and reflux system stabilized with no product drawn, the
     levels set, then the feed introduced and the product flows begun —
     "the products usually stay on-spec".
10. **What a start from empty needs that a running column does not**, so
    what the second part asks of the engine:
    - **the column holds mass**: feed in and products out differ while it
      fills, so the draws follow the levels (the base's, the drum's) rather
      than shares of the feed — today every draw is a share of this tick's
      feed, and the column node passes exactly what it takes (§7);
    - **trays that fill and drain**: a tray passes liquid down only over its
      weir (Francis), an empty tray passes none, a draw on a dry tray draws
      nothing;
    - **stages that are not boiling**: a cold stage holds liquid below its
      bubble point, or nothing; vapour condenses on the cold steel and
      liquid above it; the column warms from the bottom up. Today's solve
      puts every stage at its bubble point, so the heat stored in the
      liquid and the steel is the whole story here (finding 9), where on a
      running column it is negligible (finding 7);
    - **a place for off-spec product**: until the products settle they go
      to slop (Kister §12.2, Fig. 12.1), or are held back on total reflux.
    None of this is probed numerically here: it needs a prototype stage
    model of its own, a probe of its own.

## Not checked

- **The frozen K-values.** Each response is linear, with the AFTER state's
  flows and K-values held while the compositions move. A real stage's
  bubble point and K-values move with its composition, so the times are an
  estimate of the scale (minutes to an hour), not a prediction to the minute.
  The building slice measures the real response.
- **The base's and drum's sizes** are rules of thumb; the plant has no drum or
  base geometry to read.
