# M75.0 — an overfull column's knock-on effects, its probe (2026-11-11)

The record is DESIGN §80.0. Asked after M74: the user chose C30. **No engine
code changed**; the driver is `m75_0.rs` (M74.6.2's, with `PBAR` putting the
column's declared pressure in the holdup plant's file), run on f4c0811.

## Read first (Kister, *Distillation Operation*, 1990)

- **§13.3, p. 345** — "With an all-liquid top product, an overflowing
  accumulator will usually back up liquid into the condenser, flooding some
  tubes. This will reduce condensation and prevent further rise in liquid
  level. Often, column pressure will rise and the relief valve may possibly
  lift." The drum side, self-limiting: less condensed, less arriving.
- **§13.2, pp. 343–345** — a base above the reboiler return: vapour slugging
  through the deep liquid, trays lifted off their supports, the bottom trays
  flooded; high-level alarms and trips. The base side, its own physics.
- **§17.2, pp. 526–531** — pressure control on a total condenser with liquid
  product is most often a **flooded condenser** (§17.2.2, Fig. 17.5): part of
  the condenser is kept flooded, its condensate's flow varying the flooded
  area; less free area condenses less and the pressure rises. A drum past full
  floods it beyond the controller's range. Fig. 17.5b, the flooded reflux drum,
  runs the drum full on purpose.

## Measured

1. **The operator's mistake of M74.6.0** (`overfill.txt`): the holdup plant
   with its steel, total reflux switched on at 100 s, the feed left on, an
   hour. The drum full **26 minutes** in, the base **36 minutes**; trays 1–11
   flooding; the top's vapour 1 208 → ~3 000 mol/s, the top 163 → 102–136 °C
   (the lights stay in the column). **The condenser asked**: 44.8 MW running;
   53.6 MW one minute in (1.2×), 82 at six minutes (1.8×), 100 at sixteen,
   115 at the hour (2.6×). The slop and `"full"` alarms as §79.18.
2. **The column at other pressures** (`p*.txt`): the holdup plant declared at
   1.8 (its file), 2.2, 2.8, 3.5 and 4.5 bar, ten minutes each — each settles
   (its held reboiler the duty its first tick settled on: 86.7, 94.6, 104.6,
   114.0, 125.0 MW). The top 163 / 172 / 183 / 194 / 207 °C, the base
   591–672 °C; the condenser 44.8 → 43.4 MW; the products within 4 %; the feed
   unchanged (its pump at 30 bar); tray 11 flooding at 4.5 bar. These are
   plants settled at a pressure, not a running column whose pressure climbs.

## By hand

- **How fast the pressure moves**: vapour not condensed, `ΔP/Δt = ṅ·R·T/V`.
  With 30 MW of 82 left uncondensed (~900 mol/s at ~30 kJ/mol) into an
  overhead volume of ~100 m³ (the column's top, its vapour line, the drum's
  vapour space — a guess) at ~410 K: **~0.3 bar a second**. The pressure
  settles within seconds of the condenser's capacity changing, so per tick it
  is an algebraic balance (the condenser condensing what arrives), not a slow
  state — the drum's level is the slow one.
- **A condenser capped near its design**: at 44.8 MW across 163 − 35 °C
  (cooling at 35 °C, a game value), `UA` ≈ 0.35 MW/K; with a fifth more area
  (a game margin) 0.42 MW/K. Six minutes into the mistake the top is at
  136 °C and the column asks 82 MW: condensing it needs the top at ~230 °C —
  past the 4.5-bar plant's 207 °C. **On this story the condenser's limit, not
  the full drum, is the first act**: the pressure would climb within minutes,
  and likely lift a relief valve, long before the drum fills. The rising
  pressure itself boils up less at a held reboiler, so the estimate overstates
  — the probe's prototype would say by how much.
- **Relief**: the engine's relief valves sit between pipes
  (`relief_twin_vessels.toml`); a column has no vapour outlet a relief line
  could hang from. A relief line is a new edge (a new plant file only) or a
  reading on the column, as the slop is.

## Not checked

- A running column whose pressure changes (no command moves a column's
  pressure): the prototype's first job.
- The base side's physics (§13.2): its own slice.
- The simple solver.

## What it costs

About 3 minutes of machine time.

## For the user

1. The drum side first (the condenser and the pressure), or the base side (the
   bottom trays flooded, Kister §13.2)?
2. The slop line against the back-up: since M74.6 step 3 the excess leaves at
   once, so the drum never stays overfull. Slop as an operator's drain (open
   or shut), the back-up replacing it, or slop only past a higher limit?
3. The condenser's capacity: declared per plant (area, cooling temperature)
   or derived from its design duty with a margin?
4. Pressure control: the flooded condenser (Kister §17.2.2) — normally holding
   the declared pressure, losing it when its free area cannot condense what
   arrives — or the pressure simply floating?
5. Relief: a real relief line to a flare in the new plant file, or a reading
   on the column (as the slop)?
