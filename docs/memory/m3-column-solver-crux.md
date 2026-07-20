---
name: m3-column-solver-crux
description: "M3 sliced 2026-07-20; the simple column needs prescribed flow, which a purely pressure-driven solver cannot express"
metadata: 
  node_type: memory
  type: project
  originSessionId: e47ecd2b-a1c3-4c0c-82f0-07843f692f3a
  modified: 2026-07-20T13:44:58.967Z
---

M3 (pseudo-component crude + simple column) is sliced into **M3.1** (slate +
composition transport) and **M3.2** (the fixed-cut-point column). Unlike M2,
the halves are hard in *different* ways and neither blocks the other.

**The crux, found before any M3.2 code:** `solvers/network.rs` and
`elements.rs` are entirely pressure-driven — every edge flow is
`QuadraticBranch::flow(dp)`, and there is no prescribed-flow or divider branch
anywhere. A column is one-in-three-out where the split ratio comes from feed
*composition*, not from the draws' hydraulic resistances. Those two facts
cannot both hold alongside per-component mass conservation unless the draw
flows are **prescribed**. So M3.2 is a **solver-extension** milestone, not an
additive-unit one like M2's furnace/cooler — every M2 unit was a hydraulic
pass-through.

**Why:** this reframes M3.2's cost. Budgeting it as "one more unit model"
would be wrong by a large factor, and the fixed-flow branch has its own
well-posedness risk (an element with zero dP-derivative can leave the Jacobian
singular and needs a pressure anchor elsewhere).

**How to apply:** settle A-vs-B in a written DESIGN note before code — (A) cut
points fixed, draw rates follow, needs the fixed-flow branch; (B) draw valves
fixed, cut points emergent, no solver change but the reference number becomes
hydraulics-dependent. DESIGN §5 says cut points are the fixed thing, arguing
for A. Land the fixed-flow branch *before* the column that sits on it. The
column's reference must pin a **per-draw composition** number: the mutation
this unit invites (cut boundary off by one, smearing disabled) conserves total
mass exactly, so a mass-balance gate cannot falsify it. See
[[falsifiability-as-scoping-criterion]] and [[m3-composition-weighting]].
