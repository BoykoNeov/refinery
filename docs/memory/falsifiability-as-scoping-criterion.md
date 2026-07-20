---
name: falsifiability-as-scoping-criterion
description: "M2 closed 2026-07-20; a feature whose gate cannot be falsified is not ready to build — use it to scope, not just to test"
metadata: 
  node_type: memory
  type: project
  originSessionId: 04831589-4a42-414b-a823-f76581376aa4
  modified: 2026-07-20T13:26:07.366Z
---

M2 (Heat) closed 2026-07-20. Its last box — pump work / valve throttling heat —
was **deferred to M5 rather than built**, and the deciding argument was not
"too small to matter" but "too small to *gate*": the effect is ~0.02 K, below
the 1e-3 K Euler tolerance the ambient tests already carry, so a reference test
for it could not be made to fail for the right reason.

**Why:** this project's whole test discipline is falsify-before-trust (see
[[kv-handcalc-reference]], [[euler-truncation-tolerance]],
[[pipe-ambient-transform]]). That discipline turns out to be a *scoping* tool
as well as a testing one. If you cannot construct a mutation whose failure the
gate would catch, the feature ships untested no matter how much test code
surrounds it — so the honest move is to defer it to a fidelity level where the
quantity is large enough to pin, and record it as a deliberate limitation in
DESIGN rather than as an omission.

**How to apply:** before building a small-magnitude physical term, ask what
tolerance its gate would carry and whether a realistic error would exceed it.
If not, defer with the reason written down and name the milestone where the
quantity becomes measurable. Deferring is not dropping — the item lives in the
target milestone's box list, not only in prose.

Also from this close: `cargo test` does not exercise the CLI, and the ROADMAP's
milestone criteria include a *runnable demo*. Verify it with a real
`cargo run -p refinery-cli` on a scenario that shows the milestone's physics
before declaring a milestone done.
