# M75.3.0 — relief on the column, its probe (2026-11-11)

The record is DESIGN §80.4. **No engine code changed**: `relief.patch` (on
ca87edb) puts a relief valve on the column's top inside its pressure solve,
its settings and readings on the ticking thread
(`heat_balance::probe`); `m75_3_probe.rs` is the driver (register it in
`crates/scenarios/tests/suite.rs`, run `--test suite m75_3_probe::
--nocapture`, env `KV`, `S0_TICKS`, `TR_TICKS`, `GAMMA`, `XT`, `BACK`);
`ring.py` reads a trace every tick for ringing. The traces here are thinned
(every 10th tick; `s0_sizing.csv` every 50th) from the per-tick runs.

## Read first

- **Kister, *Distillation Operation* (1990), ch. 9.** §9.5 p. 242, Fig. 9.3
  p. 243: a column's relief at its top, upstream of the condenser
  ("location A"); §9.3 pp. 233–234: the relieving pressure the set plus the
  allowed overpressure, often 10–20 % above set; loss of coolant "frequently
  sets the column relief requirements" — the total incoming vapour; §9.4
  pp. 236–240: credits rarely taken (no operator action, no controller
  action; with fired reboilers "temperature pinches rarely (if at all)
  occur", p. 240).
- **Ludwig Vol. 1B (5th ed.), ch. 9.** Fig. 9.7A p. 1593 (API RP-520): set
  at the MAWP (design) for a single valve; a single valve's maximum
  relieving pressure 110 % of the MAWP (gauge); blowdown typical, the
  closing pressure ~93 %. p. 1590: the definitions (set, overpressure,
  accumulation, blowdown). §9.18.1 p. 1632: the set at the MAWP or lower,
  the relieving pressure at 110 % of set. pp. 1618–1619: cooling water
  failure — the total vapours entering; reflux failure at the top of a
  column — the total overhead vapour; Fig. 9.17, a safety valve on the
  column protecting condenser and receiver. p. 1629: a safety-relief valve
  in vapour service "will open only enough to allow the pressure to drop
  below the set pressure", following the pressure down. p. 1836: API
  orifice letters, T = 16 774 mm².

## The patch

- The vent `n_v(P)` [mol/s]: the engine's own laws — `relief_opening`
  (the smoothstep from set to set + accumulation) and `gas_valve_flow`
  (ISA-75.01, choked at `F_k·x_T`), the vapour ideal at last tick's top
  temperature with the drum liquid's molar mass (a total condenser's
  vapour). Set 3.52 bar (the derived design pressure), accumulation 10 % of
  the gauge set (0.25 bar: full lift 3.77 bar), to 1.013 bar, γ 1.1,
  `x_T` 0.7.
- Taken off the top vapour before the drum and the condenser, at each
  trial pressure of the tick's solve: the drum's balance, its row of the
  component sweep and the condenser's duty all on `V₀ − n_v`; the vent
  carries `n_v·K₀·x₁` and `n_v·H₀` — the sweep's own fractions, so the books
  close exactly. The solve's upper end the full lift + 2 bands (in place of
  the design pressure).
- Pop: `Blowdown::after`'s latch from last tick's pressure, full lift while
  lifted, reseat 7 % of the gauge set below it. Tick-late: the vent at last
  tick's pressure. "Coolant lost": the condenser's capacity times zero.

## Measured (the holdup plant with its steel, `margin = 0.05`)

1. **The governing case, the coolant lost** (`s0_sizing`, Kv 100 000 — far
   too big, holding the pressure just above set): the pressure 1.8 → 3.52
   bar in 57 s, held at 3.535–3.545; the column answers its pressure, the
   top's vapour falling from ~1 200 mol/s to 150–250 before the relief and
   the vent peaking at **75 kg/s** two minutes in (the drum's loop and the
   reflux settling), then easing to 32 kg/s at 12 minutes (top 230 °C, M
   0.113 kg/mol). Refused at 755 s: see 4.
2. **Sized by the book rule** (all the top vapour at normal running,
   ~1 208 mol/s, ~137 kg/s): Kv ≈ 5 090 at full lift (by hand, the same
   law); API RP-520's critical-flow area ≈ 0.114 m² (Kd 0.975, γ 1.1) —
   about seven T orifices. **Kv 5 100, the coolant lost** (`s1_lost`): the
   pressure 3.64 bar at most, inside the band, the valve at most 49 % open;
   **total reflux** (`s3_tr`): the relief lifts at 349 s and holds 3.58
   bar, 18–26 kg/s. Every tick converging (≤ 8 trials), no sign flip in
   the pressure tick to tick; books 2.4e-10 kg/s and 7.7 W at worst.
3. **Tick-late changes nothing that matters** (`s5`): within 98 Pa of the
   in-tick solve over 5 minutes, no ringing — the relief's slope (~0.1–1
   kW/Pa of latent heat) is small beside the column's own answer (~93
   kW/Pa, §80.2). The in-tick vent stays for the books' sake (it is no
   dearer).
4. **The vent can ask a little more than the column sends up** — while the
   pressure falls, the overhead's own vapour (a pressure here, not a stock,
   §80.2) — at most 0.4 % with the gradual valve; the patch books it out of
   the drum, and a drum with nothing else coming or going went negative and
   the tick was refused (`s0` at 755 s). The build caps the vent at the top's
   vapour.
5. **Pop fails** (`s2`, `s4`, `s6`, refused at 88 s / 494 s / 88 s): at
   full lift the valve vents ~1 190 mol/s against ~940 sent up — 27 % more,
   out of the drum — and the pressure falls ~5 kPa a tick to the reseat
   (3.345 bar); shut, the vent drops from 128 kg/s to nothing in a tick and
   the next tick's step does not converge. A pop valve on a column needs
   the overhead's vapour as a stock (and a reseat the step can follow).
6. **Without relief nothing moves** (the patch's no-relief path is the
   shipped one; not re-run as a gate here — the build's).

## The network

The relief line is a draw line: its flow the column's (the vent), marched
from the column to the flare, the network guarding it (`edge_flows`), as
the product lines. The relief's node gives the element and its numbers
(`cv_max`, set, accumulation, `x_T`); a pop latch (`run_relief_latches`)
would read that node's marched pressure, not the column's — moot with the
gradual valve. A marched line has carried liquid only; a vapour line's
march (density, friction) is the build's to check.

## Put to the user

The valve's size (the book rule, Kv ≈ 5 100, or the model's own peak,
≈ 2 800); gradual against pop; past full lift (held at a ceiling with an
alarm, or refused).
