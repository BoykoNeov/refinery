# Roadmap

Each milestone ends with: all tests green, clippy clean, a runnable CLI demo,
and DESIGN.md updated. Do not start Mn+1 before Mn's acceptance criteria pass.

## M1 — Flow network core (CURRENT)
Water only. Units: Source, Sink, Atmosphere, Tank, Pump, Valve, Junction.
NetworkFlowSolver (Newton) + SimpleFlowSolver behind the same trait.
- [x] `core`: units newtypes, Stream, PlantGraph, Engine tick skeleton,
      Snapshot/Command, solver traits
- [x] `scenarios`: TOML loader + EngineBuilder (`build_engine`: node/pipe
      instantiation with SI conversion + load-time topology validation);
      `tank_pump_valve.toml`
- [x] `solvers`: element characteristics (pipe/valve/pump), Newton solver
      with damping, SimpleFlowSolver (shared network compilation in
      `solvers/network.rs`; conductance-scaled Gauss–Seidel)
- [x] `cli`: run scenario N ticks, JSON snapshot output, `--solver` override
- [ ] Tests: hand-calc reference (pump fills tank through valve, compare
      steady flow to analytic value). NOT DONE — and it is the last M1 gap.
      `newton_reference.rs` pins pipe/pump/tank/junction in isolation, but no
      test drives a Kv-derived valve (both valve cases hardcode `cv_max`, and
      one only checks a *closed* valve blocks flow). So the scenario-boundary
      `kv_to_cv_si = Kv/(3600·√1e5)` conversion and `tank_pump_valve`'s
      ~13.7 kg/s steady flow are correct-by-derivation only, never checked
      against an analytic value. Nothing else covers this: a wrong conversion
      conserves mass, converges, and reruns bit-identically — all green — and
      two solvers can agree on the same wrong number.
- [x] Tests: proptest mass conservation on random networks
      (`solvers/tests/invariants.rs`, I1 chain + tree)
- [x] Tests: golden determinism test (bit-identical reruns)
      (`scenarios/tests/m1_acceptance.rs`)
- [x] Acceptance: 1000-tick run of tank_pump_valve converges every tick in
      <50 Newton iterations; mass balance error <1e-8; both solvers produce
      qualitatively matching steady states.
      Enforced by `scenarios/tests/m1_acceptance.rs` (convergence, mass
      balance, bit-identical reruns) + `solvers/tests/fidelity_agreement.rs`
      (newton ↔ simple). Measured on the reference plant: 9 Newton iterations
      worst case, 1.7e-10 kg worst mass drift. Note these criteria check that
      the plant is *self-consistent*, not that its magnitudes are *right* —
      the hand-calc box above is what pins that, so M1 is not closed until it
      lands.

## M2 — Heat
Temperature transport in streams, tank thermal inventory, HeatExchanger and
Furnace units, heat loss to ambient. Energy-balance property tests.

## M3 — Pseudo-component crude + simple column
Component slates in scenarios, composition transport, mixture properties,
fixed-cut-point column (simple fidelity). Demo: crude source → furnace →
column → three product tanks.

## M4 — Reactor
FCC 4-lump complex reactor + lookup-table simple reactor behind
ReactionModel. Reference test against published lump yields.

## M5 — Gas & pressure realism (scoped design note first)
Compressible/two-phase approximations where needed (column overheads, flare).
May be simplified or deferred — decide with a written note in docs/.

## M6 — Godot frontend + damage
godot-ext adapter, minimal scene reading snapshots; leak/fire commands
(already supported by the graph model) get game-side visualization.
Complex column (stage cascade) can proceed in parallel here if desired.
