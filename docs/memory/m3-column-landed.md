---
name: m3-column-landed
description: "M3.2 column landed 2026-07-20; the draw-flow write is post-sweep (fresh-feed consistency), and a guard test can be masked by a later stage"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: b1d00244-3fae-4c5c-9223-608ad2c1e2e2
  modified: 2026-07-20T16:33:48.933Z
---

The M3.2 fixed cut-point column (`NodeKind::Column`) landed, closing M3. Two
method lessons worth carrying, both found while building against the DESIGN note:

**1. Where a prescribed quantity is written is forced by a consistency law, not
by convenience.** The note framed the draw flow as *computed in*
`network::edge_flows`. It cannot be. The split needs the feed composition; the
feed a zero-volume column consumes this tick is the sweep's **resolved** (fresh)
one — you can't make the intake stale without breaking the sweep. Per-component
balance at the column, `Σᵢ ṁ_drawᵢ·comp_i,c = ṁ_feed·f_feed_fresh,c`, holds only
if the flow-split and the composition-split come from the **same** feed. Since
the fresh feed comp exists only after the sweep, the draw flow is finalized in
`Engine::tick` post-sweep; `edge_flows` only **guards** (reports draws as 0).
`energy::column_separation` is the single owner, called for both flows and
compositions so they cannot diverge. Splitting the *stored* (stale) feed comp
unbalances every component on any feed transient — worst on tick 0.

**Why:** "well-posed / converges / conserves total mass" is never the test —
[[well-posed-is-not-correct]]. The location that looks like a code-organization
choice was fixed by mass conservation.

**2. A guard test can be vacuous because a LATER stage masks the fault.** The
obvious silent-bogus-draw-flow gate (make a draw pipe absurdly restrictive,
assert the draw doesn't move) passes *even with the `edge_flows` guard removed* —
the post-sweep override rewrites the out-flowing draw anyway. That test pins the
override, not the guard. The guard's real hazard is the **wrong-sign** case: a
product tank filled above the column pressure whose ungated pressure-driven draw
back-feeds into the sweep and pollutes the feed mix. Falsifying against the
*masking* stage, not the surface symptom, is what found the real gate
(`a_full_product_tank_does_not_pollute_the_split`).

**How to apply:** when a fault is corrected downstream of where the gate reads,
the gate is vacuous — mutate and confirm it fails. The moving-feed transient gate
(`per_component_mass_survives_a_moving_feed_composition`) is likewise the ONLY one
that catches the stale-split bug; every fixed-feed reference stays green, the
coverage-hole point of [[unfalsifiable-is-a-claim-about-coverage]]. See also
[[m3-column-solver-crux]] and [[m3-composition-weighting]].
