# M74.6.2.0 — a stage that boils dry, its probe (2026-11-11)

The record is DESIGN §79.19. M74.6's part 2 (the third branch, then C29's
draining) probed before building, in the advisor's order. **No engine code
changed**; the prototype is `prototype.patch` (on 469efbc: M74.5's
`draining.patch` merged, M74.6.0's cap on the liquid going down, every change
below behind an environment switch), the driver `m74_6_2_dry.rs` (M74.6.0's,
`make_driver.py`: the steel put in the file with `STEEL=1`, total reflux the
shipped command).

## Switches (probe only)

- `M74_DRY=1` — **the third branch**: a stage's end-of-tick holdup a residual,
  not a refusal (the sweep reads it clamped at nothing); each stage's first
  residual `max(−V/V_s, min(bubble, max(H'/(dt·V_s), Σx' − 1)))` — today's
  `max(bubble, −V/V_s)` to the bit wherever `H'` is not near nothing.
- `M74_RESCUE=pdas` — on a tick today's Newton refuses only: a primal-dual
  active set (each stage held not boiling / boiling / dry, solved, re-chosen by
  sign: not boiling but over its bubble point → boiling; boiling with V < 0 →
  not boiling; boiling past empty → dry; dry with Σx' > 1 → boiling), the free
  residual checked at the end.
- `M74_EXPLICIT=1` — in that rescue, the trays' liquid not answering the
  tick's vapour (the slope off).
- `M74_SLOPE_CAP=c` — every tick, a tray's `slope·dt` capped at `c`.
- `M74_DIAG`, `M74_NDIAG` — traces.

## Measured

The valved holdup plant with its steel, feed, furnace and reboiler shut at
once, the draining on (M74.6.0 finding 8's shut-down), 60 minutes asked:

| switches | runs to | refused on |
|---|---|---|
| none (`shut.txt`) | 7.8 min | the base stopping boiling (V 0.05 mol/s), its energy 0.001 MW short; trays ≥ 94 kg |
| `DRY` + `RESCUE` (`pdas.txt`) | 16.6 min | the base passes (one rescue); then stage 12 cycles: held not boiling it sits 0.007 K over its bubble point, held boiling it needs V12 ≈ −0.011 mol/s (`forced_boiling_trace.txt`) — no branch consistent; trays 1–3 mol |
| `DRY` + `RESCUE` + `EXPLICIT` (`expl.txt`) | 17.0 min | 4 rescues pass 16.6 min, trays boil dry (stages 7, 8, 10, 11); then stage 11 cycles boiling ↔ dry (boiling it ends below empty, dry its vapour under its dew point) |
| `SLOPE_CAP=0.02` (`cap_shut.txt`) | 17.0 min | the 7.8-minute base stall gone with no rescue; stage 11 runs out of liquid — the third branch's case |
| `SLOPE_CAP=0.02` + `DRY` (`capdry_shut.txt`) | 17.0 min | stage 3's bubble point (−5e-7), stage 4 not boiling |

The 70 held hours (M74.5.0's `holds_specs.txt`, the holdup plant with steel
and draining): **65 of 70 in every variant** (`holds_plain.txt`,
`holds_rescue.txt`, `holds_cap.txt`) — the same 5 (the base at 0 MW, the
reflux at 15, stage 12's vapour near nothing), the rescue moving when they
fail (one from 15 to 38 minutes in), not whether.

1. **The bubble row does not go flat near dry** (the advisor's first worry,
   measured): `∂S/∂T` 0.012–0.019 per K on trays holding 1–3 mol, a third of
   that at 0.02 mol (`near_dry_jacobian.txt`).
2. **The stalls are ticks with no consistent branch**, at the scale of mK and
   10⁻⁴ mol/s, on trays holding grams. A cause found: a tray's slope (`dL/dM`,
   backward Euler within the tick) grows without bound as the tray empties —
   `slope·dt` 0.005 on a running tray, 0.1–0.4 on the near-empty, 1.03 at
   0.12 mol — so the tick's vapour comes back down as liquid in the same tick.
   Capping it moves the shut-down's first refusal from 7.8 to 17 minutes, but
   not the 5 held runs.
3. **The dry formula's dew guard is inert in the free residual** (the
   advisor's correction): on a dry tray passing no liquid the sweep pins
   `ΣK·x' ≈ 1`, so `min(bubble, ·)` reads ≈ 0 whatever `Σx' − 1` is — a cold
   dry tray passing vapour through would be accepted. Only the rescue's own
   sign test caught it. The dry state needs its own row, not a branch under
   the bubble point's; not shippable as prototyped.
4. **Every fix buys ~30 s of a dying column** (vapour 1–5 mol/s, trays holding
   grams), then a new two-branch cycle. The approach does not converge.
5. **The dry state and the draining are one piece to test**: without the
   draining no run reaches a dry tray; the dry state alone is testable only on
   a built state.

## Not checked

- The corpus with any switch on (the dry formula must leave every plant, the
  empty start above all, byte-identical before it ships).
- A cold empty stage with vapour arriving, Newton started from the
  pass-through guess (the dew guard's test, finding 3).
- Sub-stepping a refused tick (several shorter steps) — not prototyped.

## What it costs

About 40 minutes of machine time: the shut-downs (~1 min each), the 70 held
hours four times (~5–9 min each on 7–12 cores), the builds.

## After the user's first answer: sub-stepping a refused tick

The user chose (2026-11-11) to try smaller steps first, and to defer if they
did not carry the 5 held runs. `M74_SUB=m` (`heat_balance::sub_steps`): a tick
the step refuses run again as `m` steps of `dt/m`, each refused one split
again (to depth 2), the products and duties their mean, the state the last's.
With `m` = 10:

| switches | runs to | refused on |
|---|---|---|
| `SUB` (`sub_shut.txt`) | 17.0 min | stage 11 runs out of liquid in the short steps |
| `SUB` + `DRY` (`subdry_shut.txt`) | 17.2 min | the trays' pressure drops not settling (0.6 Pa after 25 solves, §78.9's loop) |

The 70 held hours (`holds_sub.txt`): **65 of 70**, the same 5, later (two
the step's base stall even in the short steps; three now the pressure drops'
loop, 0.57–0.62 Pa). Smaller steps do not carry them. **Decided, the user's
standing answer: the dry state and the draining deferred (C29), M74 closed.**

## For the user (before the first answer)

The third branch as specified does not carry the draining's shut-down or its 5
held runs; the regime left is a column with grams on its trays. Choices:
rework the tray liquid into the tick's unknowns, sub-step a refused tick, a
minimum liquid film (a modelling choice), or defer the dry state and the
draining (C29) and close M74 without them.
