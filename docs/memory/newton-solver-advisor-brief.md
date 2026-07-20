---
name: newton-solver-advisor-brief
description: NewtonFlowSolver is implemented (M1) — the pump/valve pressure-jump formulation and its load-bearing decisions
metadata:
  node_type: memory
  type: project
  originSessionId: dd36de1c-db69-468c-8457-fce7ea89fd73
---

`NewtonFlowSolver::solve` (crates/solvers/src/newton_flow.rs) is **implemented**
as of 2026-07-14. The advisor was consulted first (comments archived at
`M:\claud_projects\temp\refinery-newton-advisor\newton-flow-advisor-comments.md`)
and settled the open pump/valve pressure-jump question.

**Decision: folded combined-branch (option b).** Every M1 element is affine in
`Q·|Q|` (`dp = α·Q|Q| + β`), so a pipe and the device folded into its end
compose in closed form (`Σα, Σβ`) and invert to `Q(dp)` with no inner solve —
both costs the original brief feared evaporate. Implemented as
`elements::QuadraticBranch` (pipe/valve/pump ctors + `in_series` + `flow`/
`flow_ddp`), shared with the future SimpleFlowSolver.

Key implementation facts (so they aren't relitigated):
- Free nodes = Junction/Pump/Valve; the pump/valve unknown is its **inlet/
  suction** port; the device rise folds into its OUTLET edge. Fixed = Source/
  Sink/Atmosphere/Tank.
- Jacobian `J = −L` (weighted graph Laplacian), symmetric neg-def; faer dense
  LU. **faer pinned to single-threaded** (`set_global_parallelism(None)`) for
  determinism (rule 3).
- Line search needs the **Armijo** sufficient-decrease condition on `½‖R‖₂²`,
  NOT "any decrease" — the √-law's symmetric overshoot passes "any decrease"
  at t=1 and stalls. This was a real bug caught in testing.
- Convergence: `‖R‖_∞ < tol_abs + tol_rel·throughput` (relative, per DESIGN §3).
- F2 floating subnetworks (closed valve severs downstream): unanchored free
  nodes are pinned + excluded; an edge is "active" only if BOTH endpoints are
  anchored, which forces a dead pump's edge to exactly 0. Openings < 1e-6 snap
  to closed.
- `SimError::SolverDiverged` now carries `residual_history` (DESIGN §3, F8).
- M1 has **no cavitation floor** — over-driven pumps yield genuine sub-zero
  absolute pressure (documented in DESIGN §3); frontends read it as cavitation.

Tests: elements unit tests, `tests/newton_reference.rs` (hand calcs, tee
conservation, floating-pump, closed valve, degree error, tank head),
`tests/invariants.rs` proptests (converge-or-Err, finiteness, determinism)
over TWO generators: a linear chain (reverse-flow-through-device path) and a
hub-biased random tree (branching junctions; devices spliced by edge
subdivision for F6; `strategy_actually_branches` asserts 3+ degree hubs
really appear). Per-node balance there is near-tautological (cross-checks
`edge_flows` vs `assemble`, not a correctness oracle) — real random-net
correctness is **I5** (Newton vs SimpleFlowSolver), now landed — see
[[simple-flow-solver-and-network-extraction]] (the shared helpers here moved to
`solvers/network.rs`).

Related: [[simple-flow-solver-and-network-extraction]], [[commit-and-push-cadence]].
