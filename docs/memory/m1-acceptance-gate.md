---
name: m1-acceptance-gate
description: "M1 acceptance test landed 2026-07-17 (mass tolerance absolute-by-measurement, gates falsified before trusted); M1 now CLOSED — hand-calc landed, see kv-handcalc-reference"
metadata: 
  node_type: memory
  type: project
  originSessionId: 5939d134-8bbf-4d1a-81fd-381180287785
---

The M1 acceptance criteria used to be only *observed* in manual demo runs —
nothing failed when they stopped holding. `crates/scenarios/tests/m1_acceptance.rs`
now enforces them at the engine level (convergence, mass balance,
bit-identical reruns). Commit `c414a63`.

**M1 is now CLOSED** — the hand-calc gap this file used to track was filled by
`scenarios/tests/kv_reference.rs`; see [[kv-handcalc-reference]]. The gap was
real, and measured: a 0.66% Kv conversion error passes *every* gate in this
file (converges, conserves mass, reruns bit-identically) plus
`fidelity_agreement`, because two solvers agree on the same wrong number.

I initially ticked that roadmap box anyway while simultaneously filing the
same gap as "still open" in this file; advisor caught the contradiction.
**Self-consistency gates (conservation, convergence, determinism) say nothing
about whether magnitudes are right** — don't let green ones imply a milestone
is done.

**The mass tolerance is absolute (1e-8 kg), decided by measurement, not
assumption.** Advisor initially argued for *relative*, reasoning that absolute
1e-8 kg against ~180,000 kg would be tighter than fp cancellation allows. I
measured instead: worst drift over 1000 ticks is **1.7e-10 kg**, ~57x inside
the budget, and advisor reversed on the evidence. There is no catastrophic
cancellation because the two tanks exchange *matched* increments — the same
solved flow leaves one and enters the other — so error stays at the ulp
random-walk floor. A relative 1e-8 would be ~1.8e-3 kg: seven orders of
magnitude above real drift, green even with a visible leak. **Don't loosen
this to relative** without re-measuring; the reasoning is in the test's
`MASS_TOLERANCE_KG` comment.

Reference-plant numbers worth knowing (`tank_pump_valve`): worst case **9**
Newton iterations of the 50 budget; total inventory **179,640 kg** (180 m³ ×
998 kg/m³ from `Composition::mixture_density`, not a hardcoded 1000); steady
flow ~13.7 kg/s. The plant is *closed* — two tanks, no Source/Sink/Atmosphere
— which is what makes total mass a conserved quantity to assert against.

**Falsify a gate before trusting it.** Each assertion here was proven to bite:
a 1 ng/tick leak injected into the engine's tank integration fails mass
balance at tick 6; a lowered budget fails convergence at tick 1; a shifted
comparison fails the rerun check. A green test that was never seen red is not
evidence of anything.

**Also still open (deliberately, not forgotten):** the determinism test does
in-process reruns, which is the roadmap's literal spec ("bit-identical
reruns"). A *stored* golden snapshot file would additionally catch unintended
physics changes across commits, but churns on any intentional solver change —
offered to the user as a follow-up, not built. The Source/Sink/Atmosphere/
Junction node paths are also unpinned at the scenario boundary; the reference
plant never exercises them.

Related: [[simple-flow-solver-and-network-extraction]],
[[newton-solver-advisor-brief]], [[commit-and-push-cadence]].
