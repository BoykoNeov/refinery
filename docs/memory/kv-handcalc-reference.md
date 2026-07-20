---
name: kv-handcalc-reference
description: "Kv/hand-calc reference test landed 2026-07-17, closing M1; falsify numerical gates with SUBTLE mutations — a violent one catches for the wrong reason"
metadata: 
  node_type: memory
  type: project
  originSessionId: ebf49fac-4ef0-441a-b955-48debc623fd2
---

`scenarios/tests/kv_reference.rs` closed the last M1 gap (2026-07-17): the
`Kv → cv_si` conversion and `tank_pump_valve`'s flow magnitude now have an
analytic anchor. Reference plant steady flow at initial levels (8.0 m / 1.0 m)
is **13.753287 kg/s** — the roadmap's long-standing "~13.7 kg/s" estimate was
right.

**The tautology trap, worth re-reading before writing any reference test here.**
A hand calc that reuses the code's own formula to compute its "expected" value
proves nothing — the bug sits on both sides and the test is green. Concretely:
the reference's valve coefficient must be derived from the *Kv definition*
(IEC 60534-2-1: Kv m³/h of water at 1 bar, SG 1 ⇒ `cv_si = (Kv/3600)/√1e5`),
never by calling `kv_to_cv_si`, which is the thing under test. Advisor flagged
this as the one thing that would silently defeat the whole exercise.

**Two tiers, different strength — don't conflate them.** The Kv-definition test
is *truly* independent: its expected value comes from the published standard,
not from anything in the workspace. The network hand calc necessarily mirrors
`QuadraticBranch`'s series algebra — that is the inherent ceiling of any network
hand calc, it catches wrong constants/signs/folds/unit slips but not an error in
the model's *formulation*. Accept the ceiling; don't chase a "more independent"
network check that doesn't exist.

**Falsify with a SUBTLE mutation — a violent one catches for the wrong reason.**
The generalizable lesson from this session. First falsification attempt broke
`kv_to_cv_si` by dropping the √ entirely (÷1e5 not ÷√1e5). *Everything* went
red — but only because cv_si fell 316x, valve resistance rose 1e5x, and Newton
simply diverged. That's an accidental catch: it proves nothing about whether the
tests cover Kv, and it would have wrongly "refuted" the roadmap's claim that a
wrong conversion stays green. The honest mutation was a plausible bar-vs-atm
slip (`√101325` for `√1e5`, +0.66%), which leaves the network well-conditioned:
every other M1 test stayed green (m1_acceptance, fidelity_agreement,
newton_reference, invariants — 26 tests) and only kv_reference failed. **A
mutation that breaks conditioning tests the solver, not the assertion.**

Test location deviates from CLAUDE.md's `solvers/tests/reference/` rule on
purpose: `kv_to_cv_si` is private to `scenarios` and only observable through the
loader, so the test must live in that crate (noted in the file so it reads as
deliberate). `newton_reference.rs` already doesn't follow that rule either.

Useful mechanics: `cv_max` on `NodeKind::Valve` is the **wide-open** coefficient
— the loader stores it unscaled and linear trim (`cv_eff = cv_si·opening`) is
applied at solve time in `elements.rs`. Scaling it in the loader too would
square the trim. Solver `eps_dp = 1.0` Pa stiffens each branch by ~1 Pa against
~411 kPa driving head ⇒ ~4e-6 relative shift in Q, which is why 1e-3 tolerance
is right and why the exact Kv identity is asserted inline rather than through
`valve_flow` (whose regularization would blur it).

Related: [[m1-acceptance-gate]], [[simple-flow-solver-and-network-extraction]],
[[newton-solver-advisor-brief]], [[commit-and-push-cadence]].
