---
name: m3-composition-weighting
description: "Composition blends by mass alone, temperature by mass·cp — sharing the weight vector conserves mass while corrupting fractions"
metadata: 
  node_type: memory
  type: project
  originSessionId: e47ecd2b-a1c3-4c0c-82f0-07843f692f3a
  modified: 2026-07-20T13:45:09.973Z
---

Composition and temperature ride the **same** `resolve_node_temperatures`
sweep (never build a parallel one — the Kahn ordering, exchanger pair merge and
recycle rejection are already correct), but they **must not share a weight
vector**:

- temperature/enthalpy mixes weighted by `mass_flow × cp`
- composition mixes weighted by `mass_flow` **alone**

They are independent: neither result feeds the other, both read the same
inflow edges. Compute both in one pass, with separate weights.

**Why:** reusing the enthalpy weights for composition is the natural mistake
and is nearly invisible — it **conserves total mass** and passes energy
conservation, corrupting only the fractions. That makes it the discriminating
mutation for this box, and the proof that the per-component invariant (I7) is
not a tautology — the same role the post-tick-mass Euler slip played for I6 in
M2.1 (see [[euler-truncation-tolerance]]).

**How to apply:** lead M3.1's falsification with the `mass·cp`-weighting
mutation; I7 and the blend reference must fail on it while I1 and the energy
gates stay green. Mirror `mix_inflows`' zero-inflow rule exactly (fall back to
the node's previous value — `Composition::from_weights` errors on all-zero, so
blending through it is impossible), fold multi-inflow blends in deterministic
edge order, and blend a tank's inventory on **start-of-tick** mass.

Consequence to watch: once composition moves, a tank's mixture density is
time-varying, so anything deriving level from mass must use the current
composition's density — the echo of the M3.1 loader bug where tank mass was
computed at water's density. See [[m3-column-solver-crux]].
