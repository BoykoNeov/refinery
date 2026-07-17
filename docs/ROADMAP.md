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
- [x] Tests: hand-calc reference (pump fills tank through valve, compare
      steady flow to analytic value) — `scenarios/tests/kv_reference.rs`.
      Two levels, pinning different things:
      1. The `Kv → cv_si` conversion against the **published definition**
         (IEC 60534-2-1 / ISA-75.01: a Kv valve passes Kv m³/h of water at
         1 bar, SG 1). This is the only *truly* independent anchor in M1 —
         its expected value comes from the standard, not from any formula in
         the workspace.
      2. `tank_pump_valve`'s flow at its initial levels (8.0 m / 1.0 m)
         against an independently derived **13.753287 kg/s**, with the valve
         coefficient in the reference derived from the Kv definition rather
         than from `kv_to_cv_si` — reusing the code's own conversion would
         hide a bug in it on both sides. Both fidelities are pinned to the
         analytic number, not merely to each other.
      Lives in `scenarios/` rather than `solvers/tests/reference/` because
      `kv_to_cv_si` is private to that crate and only observable through the
      loader (deliberate deviation, noted in the file).
      **Falsified before trusted:** a plausible bar-vs-atm slip (`√101325`
      for `√1e5`, a +0.66% error) leaves *every* other M1 test green — it
      converges, conserves mass, reruns bit-identically, and both solvers
      agree on the same wrong number — and only `kv_reference` fails. The gap
      this box described was real, and is now closed.
      (Caveat worth keeping: the network hand-calc in (2) necessarily mirrors
      `QuadraticBranch`'s series algebra — the inherent ceiling of any network
      hand calc. It catches wrong constants, sign/fold errors and unit slips,
      not an error in the model's formulation. (1) is what has no such
      ceiling.)
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
      the hand-calc box above is what pins that, and it has now landed
      (`scenarios/tests/kv_reference.rs`).

**M1 acceptance criteria are met.** All boxes ticked; `cargo test --workspace`,
`cargo clippy --workspace --all-targets -- -D warnings` and `cargo fmt --check`
are green. M2 may begin.

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
