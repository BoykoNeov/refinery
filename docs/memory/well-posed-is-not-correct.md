---
name: well-posed-is-not-correct
description: "M3.2 column note (2026-07-20) — a solve can converge, conserve mass and rerun bit-identically while being frozen-wrong; check what the fixed point MEANS"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 4c0dd15e-6430-466b-ae1b-38b628ee790d
  modified: 2026-07-20T15:29:19.816Z
---

When judging a proposed formulation, "does it converge / is the Jacobian
nonsingular / does mass balance" is not the question. The M3.2 column's naive
form (draws prescribed from the *previous* tick's feed, column node free) solves
`ṁ_feed(P_C) = Σsᵢ = ṁ_feed_prev` — one monotone equation in one unknown. It
converges, conserves every component, and reruns bit-identically. It also
freezes the feed at its initial value forever: close an upstream valve and the
column's pressure slides to absorb it while the flow never moves.

**Why:** every automated gate the project owns would have passed that plant.
Well-posedness is a property of the algebra; correctness is a property of what
the fixed point *means*. The roadmap had pre-registered the risk as "the
Jacobian may be singular" — the real failure was a perfectly conditioned system
solving the wrong equation.

**How to apply:** when a formulation prescribes a quantity, write out the
equation the solve actually lands on and ask what it says physically. Then
perturb the plant on paper (close a valve, drain a tank) and check the answer
moves. Relatedly: a conservation gate is green *by construction* for any
splitter-shaped unit, so it has no discriminating power there — see
[[unfalsifiable-is-a-claim-about-coverage]] and
[[falsifiability-as-scoping-criterion]].
