# M72.0 — a pumparound whose liquid goes somewhere (C9), 2026-10-20

Asked after M71 as "what is next"; the user chose the column group (the
pumparound loop, then flooding and weeping, then a column with holdup), the
loop first. DESIGN §77.0.

**The probe**: `probe_circuits.patch` (on 4b09fc2, `crates/solvers/src/`) makes
each declared pumparound a liquid circuit inside the heat-balance cascade —
drawn off stage `d`, returned to `d − 1` with the composition of `d` — chosen by
environment variables:

- `PROBE_PA_FLOW_KG_S=85,95,65` — each pumparound's circulation, in declared
  order (none set: every pumparound the side cooler it is today).
- `PROBE_PA_SHIFT` — `down` (returned on the declared stage, drawn from the one
  below), `up` (drawn from the declared stage, returned one above), or
  `pairs:3-2,5-4,7-6` (one-based draw-return pairs, in declared order).
- `PROBE_PA_TRET_C=150,215,270` — the return temperature each loop's cooler
  holds (none: the declared duty sets it).
- `PROBE_START=duty` — a cold start on the declared duties first, then handed to
  the held return temperatures over `CONTINUATION_STEPS` solves.
- `PROBE_WALK=flow|both` — the halvings scale the loop flows (and the cooling).
- `PROBE_PRINT=1` — each converged Newton solve's profile on stderr.

The patch also raises `SHARP_SEED_ITERATIONS` 10 → 30 (finding 8). A loop's
molar flow is a Newton unknown appended after the free water, `P·M̄(x_d) = ṁ`,
started every solve from the feed's molar mass (so two to three steps a tick).

`probe_tests.patch` adds two throwaway tests to
`crates/scenarios/tests/crude_unit.rs`: `probe_m72_cut_back_with_loops` (the
reflux walked down after 1 000 ticks) and `probe_m72_timing` (ignored; the
steady median of 300 ticks, then the ticks after a reflux 4 → 2, on
`PROBE_SCENARIO`). `cu13_side.toml` and `cu13_loops.toml` are the crude unit on
13 trays (draws on 4, 7, 10, the feed on 11), its pumparounds on 3, 6, 9 and on
4, 7, 10.

## Findings

1. **Loads.** The crude unit, loops 3→2, 5→4, 7→6 at 500 / 430 / 215 mol/s:
   liquid down (kg/s) on stages 1–9 `97 230 130 259 152 243 160 208 288`
   against the side cooler's `98 106 88 126 107 152 158 208 288` — the trays
   inside a loop 1.6–2.2 times.
2. **Purity** at the draw trays (liquid mole fraction of the product's own cut):
   side cooler kerosene 0.724, diesel 0.679, gas oil 0.666; loops off the
   product trays (3→2, 5→4, 7→6) 0.694, 0.646, 0.642; loops returned ONTO the
   kerosene and diesel trays (4→3, 6→5, 7→6) 0.684, 0.585, 0.560.
3. **Thirteen trays**: side coolers 0.832, 0.800, 0.772 (stages 4, 7, 10); loops
   4→3, 7→6, 10→9 on duties 0.812, 0.778, 0.755.
4. **Costless and closed**: `heat_balance::` lib tests with
   `PROBE_PA_SHIFT=up PROBE_PA_FLOW_MOL_S=40` (the molar version of the patch)
   — 17 pass, both closure tests among them; the two that fail are the cut-back
   tests, whose tray no longer runs short. 3 000 ticks of the crude unit 1.04 /
   1.06 s without / with loops.
5. **kg/s** converges: three steps every tick (restarted from the feed's molar
   mass), on both duty and held-return forms.
6. **Held return** at 150 / 215 / 270 °C: duties 14.21, 14.90, 10.93 MW. Duty
   held at 15 MW with 100 mol/s: return −124.9 °C, solved.
7. **Cold start, 13 trays, held return**: refused (stage 11's vapour −2 194
   mol/s); the cooling walked as a share, or the flows, from 1/16: refused
   (stage 10's liquid −109 mol/s). On duties first then handed over
   (`PROBE_START=duty`): starts; 309 ticks of 2 steps, two of 3–4; duties 15.06,
   17.05, 12.79 MW at the held returns.
8. **Taller cold starts**: 11, 12 and 13 trays with side coolers all refuse
   ("the draws above the reboiler carry 475–488 mol/s away from a feed of 472.5")
   with `SHARP_SEED_ITERATIONS` 10; all start with 30.
9. **A short tray**: reflux walked down after 1 000 ticks — side coolers cut the
   kerosene back at 0.6, 0.4, 0.3, 0.2 (58, 32, 18, 4 % of its share) and halt at
   0.1; with loops nothing is cut back at 0.3–0.6 and 0.2 halts ("a liquid rate
   left the physical region").
10. **Timing** (release, two runs agreeing to ±0.05 ms steady, ±0.2 ms key
    press): steady median / the tick after reflux 4 → 2 —
    shipped crude unit 0.52 / 8.3 ms; 10 trays, held-return loops 1.44 / 11.5;
    13 trays, side coolers 0.70 / 30.4; 13 trays, held-return loops 1.34 / 24.8.
