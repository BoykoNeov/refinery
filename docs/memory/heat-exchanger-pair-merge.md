---
name: heat-exchanger-pair-merge
description: "HeatExchanger landed 2026-07-20 (M2.2) — pair-merged sweep vertex, and the C_min-on-side-A vacuity falsification caught"
metadata: 
  node_type: memory
  type: project
  originSessionId: 80f2d27b-ca8c-4bfa-8518-c010323eccb0
  modified: 2026-07-20T09:38:31.600Z
---

`HeatExchanger` closed the third M2.2 box on 2026-07-20 (`0d5b20b`, design
note `ea13c8e`). ΔT-effectiveness fidelity: one signed
`Q = ε·C_min·(T_a_in − T_b_in)`, ε stored on the pair (a coupling list on
`PlantGraph`), `NodeKind::HeatExchanger` carrying only side identity.

Two things worth keeping that the code alone does not say:

**The sweep is no longer per-node.** An exchanger side's outlet depends on an
inlet that is not one of its own inflow edges, so `resolve_node_temperatures`
groups zero-volume nodes into *vertices* keyed by a leader id, and a pair is one
vertex. Any future unit that couples two streams must join that grouping rather
than add a second mechanism. Self-dependencies are counted and never released on
purpose — that is what routes a side-feeds-partner plant into the M2.1 recycle
rejection. See [[newton-solver-advisor-brief]] for the sweep's origin.

**Falsification found a vacuity in the reference PLANT, not the test.** The
per-side-effectiveness mutation passed `heat_recovery.toml` because the C_min
stream sat on side B, where the wrong formula coincides with the right answer.
Fixed by making the hot (small) stream side A, with the test asserting that
premise. Generalizes: when a reference case has a *choice of which side gets the
special role*, the choice itself is load-bearing and belongs in an assertion.
Same lesson as [[kv-handcalc-reference]] — a gate can be green for a reason
unrelated to what it claims to check.

Remaining in M2.2: ambient heat exchange (tanks and pipes, driving force
`T_ambient − T_node`, must heat AND cool with no second path), and pump work /
valve throttling if it earns its keep (~0.02 K).
