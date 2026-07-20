# Memory index — refinery

- [Commit and push cadence](commit-and-push-cadence.md) — commit + push at end of session, docs/memory update, or work batch
- [Newton solver implemented](newton-solver-advisor-brief.md) — NewtonFlowSolver done (M1): folded combined-branch, Armijo line search, F2 floating, faer single-threaded
- [SimpleFlowSolver + network.rs](simple-flow-solver-and-network-extraction.md) — Simple solver done (M1): conductance-scaled Gauss–Seidel, shared network compilation, I5 tests, build_engine done
- [M1 acceptance gate](m1-acceptance-gate.md) — acceptance test landed 2026-07-17 (mass tolerance absolute-by-measurement, gates falsified before trusted); M1 now closed
- [Kv hand-calc reference](kv-handcalc-reference.md) — closed M1 2026-07-17; the tautology trap, and why falsification needs a *subtle* mutation (a violent one catches for the wrong reason)
- [HeatExchanger + pair merge](heat-exchanger-pair-merge.md) — landed 2026-07-20 (M2.2); the sweep is vertex-based now, and why C_min must sit on side A
- [Euler truncation tolerance](euler-truncation-tolerance.md) — tank ambient exchange landed 2026-07-20; derive a truncation-sized tolerance, never assert against the integrator's own formula
- [Falsifiability as a scoping criterion](falsifiability-as-scoping-criterion.md) — M2 closed 2026-07-20; if a feature's gate can't be falsified, defer it rather than ship it untested
- [Pipe ambient transform](pipe-ambient-transform.md) — pipe ambient exchange landed 2026-07-20 (M2.2 closed); separate readers need separate mutations, and a gate can be vacuous by reading a field that never moves
- [M3 column solver crux](m3-column-solver-crux.md) — M3 sliced 2026-07-20; the simple column needs prescribed flow, which a pressure-driven solver can't express
- [Composition weighting](m3-composition-weighting.md) — composition blends by mass alone, temperature by mass·cp; sharing weights conserves mass while corrupting fractions
- ["Unfalsifiable" is about coverage](unfalsifiable-is-a-claim-about-coverage.md) — M3.1 composition transport landed 2026-07-20; a deferral that died when the missing test was written
- [Well-posed ≠ correct](well-posed-is-not-correct.md) — M3.2 column note 2026-07-20; a solve can converge, conserve mass and rerun bit-identically while frozen-wrong
- [M3 column landed](m3-column-landed.md) — M3 closed 2026-07-20; the draw-flow write is post-sweep (fresh-feed consistency), and a guard test can be vacuous when a later stage masks the fault
- [M4 reactor crux](m4-reactor-crux.md) — M4 opened 2026-07-20; crux is conservation + the sensible-only datum (not kinetics), isothermal-ROT avoids the adiabatic fixed point, energy gate is a two-duty DIFFERENCE to dodge a tautology
