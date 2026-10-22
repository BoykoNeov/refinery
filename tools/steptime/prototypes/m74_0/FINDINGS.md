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
- `analyze.py <steps.out> [base s] [drum s] [deck scale] [total|distillate]`:
  per component, the composition model with holdup, `H_j·dx_j/dt = in −
  out`, linear, the AFTER state's flows and K-values frozen; one state per
  stage and one for the reflux drum (a total condenser: the drum's liquid is
  the reflux and the distillate). The feed is the AFTER state's steady
  residual per stage — it lands on stage 11 alone (the feed tray), which
  checks the flows read. The slowest time constant per component, and the
  products' response from the BEFORE compositions, exact and by implicit
  Euler at dt = 0.1 s, the time to 90 % and 99 % of the move.
- Holdups, **sized once on the file's settings** (the control run's BEFORE)
  **and held as masses for every run**: real trays, base and drum are a fixed
  size. (A first pass sized them on each run's own flows, so the drum shrank
  at a low reflux and grew at a high one; the advisor caught it.) A tray holds
  its clear liquid on the deck, `ρL·A_active·(h_w + h_ow)` — Sinnott's head,
  the one M73's pressure drop uses — plus its downcomer's backup,
  `ρL·A_dc·h_b`. The base holds 5 minutes of the bottoms. The reflux drum
  holds 5 minutes of either the distillate or the reflux and distillate it
  passes — the two readings of the rules:
  - Ludwig Vol. 1, 4th ed., "Rules of thumb: summary", p. xix, towers rule 12:
    "Reflux drums usually are horizontal, with a liquid holdup of 5 min
    half-full" (no flow named); p. xxiv, vessels rule 5.
  - Kister, *Distillation Operation* (1990), §15.14, Table 15.2 p. 482: the
    surge between the low and high level alarms, for tower reflux, 5 minutes
    of the product (or 2 of the reflux plus product, the author's own
    preference); p. 483: below the low-level alarm, 2 minutes of the
    product. At its normal level, mid-band, the drum holds about 4.5 minutes
    of the distillate on the first, 2 of distillate plus 1 of reflux and
    distillate on the second: 6–10 t here. And: "Unnecessarily large reflux
    drum residence times should be avoided. These … incur an excessive
    composition lag that interferes with column control."
- `startup.py <steps.out>`: hand estimates for a start from empty (the drum
  at Kister's 5 minutes of distillate), and the heat a running column's
  liquid stores as its temperatures move.

## Findings

1. **What the column holds** (`analysis_*.txt`, first table): each tray
   2.0–4.8 t (deck 1.0–2.6 t, downcomer 0.9–3.3 t), 14–23 s of its liquid
   load; the twelve trays 44.8 t; the base 8.7 t. The drum 6.8 t on 5
   minutes of the distillate (Kister), 34.2 t on 5 minutes of all it passes
   (at a reflux of 4 it passes five times the distillate). 60 t in all on
   Kister's drum, 10 minutes of feed.
2. **How slowly the products answer a key** — the drum on Kister's basis
   (`analysis_drum5_distillate.txt`): the side products first, 90 % of the
   way in 1–4 minutes after a reflux or loop-return move; the top product 90 %
   in 8–15 minutes, 99 % in 20–42; the bottoms 90 % in 1–6. A move of the
   loops' flow is slower on the kerosene draw (90 % in 14–17 minutes), the
   tray the top loop draws from too. The slowest time constant 13 minutes at
   every setting, set by the trays and the loops' recirculation.
3. **The reflux drum sets the top product's pace.** Its liquid leaves the
   column only as distillate, so a drum sized on all it passes (34 t,
   `analysis_drum5_total.txt`) turns over in about 25 minutes: the top
   product 90 % in 27–40 minutes, 99 % in 1–1.6 hours, the slowest time
   constant 26.5 minutes. A 1-minute drum on all it passes is Kister's drum
   again (6.8 t, `analysis_drum1_total.txt`, the base 1 minute: the same 13
   minutes). Ten minutes of the distillate (`analysis_drum10_distillate.txt`,
   13.7 t, base 17.4 t): the top product 90 % in 16–18 minutes. Half the
   deck liquid (`analysis_deck_half.txt`, an aerated deck holds less clear
   liquid than Sinnott's head) moves the slowest time constant by about 2 %
   with the large drum. **So the drum's size is the choice that matters**;
   the trays' holdup comes out of M73's geometry and matters less.
4. **The furnace moves no product** on this reboiled column (under 0.1 points;
   the reboiler makes up the difference, as M73.0 found for the trays).
5. **The settled column is today's.** The model with holdup settles on the
   AFTER state to 2e-6 or better in mole fraction (to 2e-11 on the furnace
   steps): holdup changes the path, not where it ends. So the shipped plants'
   steady answers carry over — a check, not an assumption, for the building
   slices. The screen's measured tables carry over in their end states, but
   measuring them on a plant with holdup costs about ten times the ticks:
   "1 000 ticks after the last move" no longer means settled (8 000–24 000
   do), and the path between can differ — a draw cut back while the
   compositions lag.
6. **The tick is fine-grained enough.** Implicit Euler at 0.1 s against the
   exact response: at most 1.1e-4 in mole fraction, against time constants
   of 4–27 minutes. The holdup term `H/dt` outweighs a tray's flows about a
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
   - filling the column's 60 t at the 104 kg/s feed: 10 minutes (the trays
     alone 7);
   - warming it from 20 °C: the liquid about 36 GJ, the steel (77 t: a 15 mm
     shell on the file's diameters, decks at 60 kg/m², a game estimate) about
     12 GJ — 40 minutes at the furnace's 20 MW, 7–9 minutes at the reboiler's
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
