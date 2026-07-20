---
name: unfalsifiable-is-a-claim-about-coverage
description: "\"No gate can falsify this\" is a claim about the test surface, not the change — check for a hole where the change lives before deferring"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: e47ecd2b-a1c3-4c0c-82f0-07843f692f3a
  modified: 2026-07-20T15:07:24.328Z
---

Deferring a change because no existing gate can turn red on it
([[falsifiability-as-scoping-criterion]]) is right, but the premise needs its
own check: **"unfalsifiable" is a statement about the test surface, not about
the change.**

M3.1 deferred taking a stream's `cp` from the resolved upwind node rather than
the pipe's stored (one-tick-stale) composition. The argument was airtight —
composition transport is entirely `cp`-free, and the regression anchor is
one-component, where every `cp` is equal by construction. It was also wrong,
because the surface had a hole exactly where the change lived: **every
multi-component test was isothermal and every thermal test was one-component.**
Writing the one case that crosses both axes — a hot cut of one `cp` entering a
tank of another — failed immediately, and not at either candidate answer. The
lag was not the bounded transient the deferral assumed; the wrong enthalpy was
booked on the *first* tick.

**Why:** two orthogonal axes each tested alone will always look like they cover
the plane. A deferral resting on that illusion ships untested code with a
written justification attached, which is worse than shipping it plainly — the
note discourages the next reader from looking.

**How to apply:** before accepting "no gate can falsify this", name the two
axes the change sits between and ask whether any single test varies both. If
none does, the honest options are to write that test or to say the coverage is
absent — not to conclude the change is unobservable. Corollary from the same
review: when a feature exists for one specific case (a `Sink`'s composition
exists for reverse flow), gate that case *deterministically*; leaving it to a
proptest generator to reach by chance is the wrong coverage for it. See
[[m3-composition-weighting]].
