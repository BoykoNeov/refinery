---
name: pipe-ambient-transform
description: "Pipe ambient exchange landed 2026-07-20 (M2.2); separate readers need separate mutations, and a test can be vacuous by reading the wrong field"
metadata: 
  node_type: memory
  type: project
  originSessionId: 052ad1e6-906d-4c85-8722-5c1dd42bcfe4
  modified: 2026-07-20T13:18:45.294Z
---

Pipe ambient heat exchange landed 2026-07-20, closing M2.2's last substantive
box. `energy::pipe_outlet_temperature` is the analytic transform;
`energy::edge_temperature_at` is the single owner of "which END of this edge
does a reader mean". See [[heat-exchanger-pair-merge]] and
[[euler-truncation-tolerance]] for the two boxes before it.

Two lessons worth more than the feature:

**Separate readers need separate mutations.** The sweep (`inflow_totals`) and
the tank loop in `Engine::tick` both consume the transform, but through
independent call sites. Mutating the sweep to read the raw upwind temperature
did NOT fail the tank tests — the tank path never runs that code. A test
asserting the tank path was therefore *unfalsified* until a tank-loop-specific
mutation was written. Counting mutations is not the same as covering readers:
enumerate the call sites, then check each has a mutation that reaches it.

**A test can be vacuous by reading a field that never moves.** The tank gate
first read the snapshot's node temperature. For a tank that is the sweep's
*start-of-tick boundary value* — it does not change within the tick — so the
test reported 373.15 K under every mutation and passed against the very bug it
was written for. It only discriminated once pointed at the tank's own
integrated `TankState::temperature`. When a gate passes under its own mutation,
suspect the *reading* before the logic.

**Why:** Both failures pass a green suite and look like coverage. This project's
whole discipline is "falsified before trusted" (see [[kv-handcalc-reference]]),
and both of these defeat it while appearing to satisfy it.

**How to apply:** For any value with more than one consumer, write one mutation
per consumer and confirm each fails a *different* named test. Before trusting a
gate, confirm the field it reads actually changes within the window under test.
Prefer a cross-check derived independently of the implementation — here the
enthalpy drop was checked against `UA·LMTD`, algebraically identical to the
exponential but not a readback of it.

Also: `git restore` reverts to HEAD, not to "before my last edit". Mutation
testing on uncommitted work must back the files up somewhere else first —
restoring from git destroyed a session's uncommitted implementation once.
