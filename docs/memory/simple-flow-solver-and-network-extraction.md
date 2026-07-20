---
name: simple-flow-solver-and-network-extraction
description: SimpleFlowSolver implemented (M1) + shared solvers/network.rs extracted; I5 agreement tests landed; what remains for M1
metadata: 
  node_type: memory
  type: project
  originSessionId: f0fe7bf2-1842-4280-9bbd-f26520f3af58
---

As of 2026-07-14, `SimpleFlowSolver::solve` (crates/solvers/src/simple_flow.rs)
is **implemented** and the shared network compilation was extracted. Advisor was
consulted before writing.

**Shared `solvers/src/network.rs`** — the single source of truth for the fidelity
seam. Both solvers now call: `validate_degrees`, `compile_edges`/`CompiledEdge`,
`fixed_pressure`, `classify` (fixed/free/anchored + deterministic cold seed),
`anchored_set`, `edge_flows`, `finalize` (NaN-scan). Newton was repointed at it
and re-verified **bit-identical green** before Simple was written. Newton-only
bits (Jacobian `assemble`, `solve_linear`, Armijo line search) stayed in
newton_flow.rs. See [[newton-solver-advisor-brief]] for the element physics.

**SimpleFlowSolver = conductance-scaled nonlinear Gauss–Seidel** (NOT the doc's
original fixed-`beta` sketch — that diverges on stiff branches). Per free node:
`ΔP_n = ω · imbalance_n / Σ_e g_e`, `g_e = ρ·branch.flow_ddp ≥ 0`, swept in
ascending id order in place until max node imbalance < tol. This is diagonal
(Jacobi) preconditioning of the same weighted-Laplacian Newton assembles →
scale-invariant across the wide pipe/valve conductance spread. Solves the SAME
fixed point as Newton; only difference is the looser stop tol (`tol_rel=1e-6`).
Iterates to convergence INSIDE `solve()` (returns a converged HydraulicSolution,
not a per-tick transient), warm-started from prior pressures, ignores `dt`.
Non-convergence in `max_iter` (5000) sweeps → `Err(SolverDiverged)`, never NaN.
Defaults: `omega=1.0, max_iter=5000, tol_abs=1e-8, tol_rel=1e-6, eps_dp=1.0`.
**Watch-item:** omega=1.0 is undamped; if a future cold first tick of a long run
diverges, lower it — no under-relaxation was needed on any test network.

**I5 agreement tests landed** (was the stubbed correctness oracle):
- `tests/fidelity_agreement.rs` — 6 curated well-posed cases (guaranteed
  convergence, non-vacuous), incl. the `tank_pump_valve` reference topology.
- `tests/invariants.rs` — random chain/tree cross-check (compare flows only when
  BOTH converge, absolute flow floor, 5% bound) + `tree_simple_is_deterministic`
  + `simple_agrees_on_a_healthy_fraction` meta-guard (Simple agrees on ≈100% —
  295/296 — of Newton-converged chains; 50% floor). All green, clippy+fmt clean.

**`scenarios::build_engine` is now DONE** (2026-07-14, later session). Steps 2–3
implemented in crates/scenarios/src/lib.rs: nodes instantiated in IndexMap file
order (deterministic ids), pipes resolved via `graph.find_node`, SI conversion at
the boundary (`bar_to_pa`, `c_to_k`, `kv_to_cv_si = Kv/(3600·√1e5)`, tank mass =
ρ·A·h with ρ from `Composition::mixture_density`, NOT hardcoded 998).
`validate_topology`: reuses solver's `network::validate_degrees` (remapped to a
`Scenario` err) for the (1-in,1-out) pump/valve invariant + a FRESH union-find
connectivity check over ALL pipes (not `anchored_set`, which walks conducting
edges — a t=0-closed valve must not sever the net at load) requiring ≥1
pressure-fixing node per component. Happy-path test
`build_engine_wires_and_runs_the_reference_plant` builds the reference, ticks 50×
(all Ok), asserts receiving_tank fills — proves wiring + flow *direction* +
fold-at-source. NOT yet pinned: conversion *magnitude* (correct-by-derivation
only) and the Source/Sink/Atmosphere/Junction node paths (reference plant never
exercises them) — that's the still-open "hand-calc reference (compare steady flow
to analytic value)" roadmap item. Full workspace green, clippy+fmt clean.

**Updated 2026-07-17:** the CLI and the acceptance/determinism tests are all
done — the CLI turned out to be already implemented, the roadmap checkboxes
had merely fallen behind the code. **Verify against the code before trusting a
checkbox** here or in ROADMAP.md, in either direction: they have been stale
both ways. **M1 is NOT closed** — the "hand-calc reference (compare steady
flow to analytic value)" item is real and outstanding; it is what would pin
the `kv_to_cv_si` magnitude this file flags as unpinned above. See
[[m1-acceptance-gate]].

**Committed + pushed** (2026-07-14): two commits on main — `26b67df`
feat(solvers) (the Simple-solver/network.rs batch, previously never committed)
and `d51f976` feat(scenarios) (build_engine). Each verified green
(tests+clippy+fmt) independently before push.

Related: [[newton-solver-advisor-brief]], [[commit-and-push-cadence]].
