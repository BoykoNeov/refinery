# Refinery Simulator — Project Guide

Headless, deterministic oil-refinery simulation engine in Rust. Science-based
approximations (pseudo-components, lumped kinetics, network hydraulics).
Consumed by multiple frontends: Godot 4 game (via GDExtension), CLI batch
runner, and future dashboards. Fidelity is swappable per physical concern
(simple ↔ complex) via trait implementations selected in scenario config —
never via `if simple_mode` branches.

## Workspace layout

```
crates/
  core/       # Types, units, plant graph, engine loop, solver TRAITS, snapshots.
              # ZERO heavy deps. Never imports Godot, never imports solvers.
  solvers/    # Trait IMPLEMENTATIONS: flow solvers, thermo, kinetics.
              # Depends on core + faer. Simple and complex variants live here.
  scenarios/  # Plant definition format (TOML), loader, engine builder.
  cli/        # Headless runner: load scenario, run N ticks, emit JSON snapshots.
  godot-ext/  # GDExtension adapter. The ONLY crate that knows Godot exists.
docs/         # DESIGN.md (architecture + physics), ROADMAP.md (milestones)
scenarios/    # *.toml plant definitions (start with tank_pump_valve.toml)
```

## Hard architectural rules

1. **`core` is sacred.** No Godot types, no solver implementations, no I/O
   beyond serde. If a change to `core` is needed to satisfy a frontend,
   the frontend adapter is wrong.
2. **Fidelity = trait impl selection.** `FlowSolver`, `ThermoModel`,
   `ReactionModel` are traits in `core`; implementations live in `solvers`;
   scenario TOML picks which. Adding `if fidelity == Simple` inside shared
   code is a bug.
3. **Determinism is non-negotiable.**
   - Fixed timestep. Same scenario + same seed ⇒ bit-identical snapshots.
   - NEVER iterate a `HashMap`/`HashSet` in simulation code. Use `Vec`,
     `BTreeMap`, or `IndexMap`. Clippy is configured to help; don't fight it.
   - No parallelism in the tick loop (float reduction order). Revisit only
     with benchmarks and a determinism plan.
   - No wall-clock time, no thread randomness inside `core`/`solvers`.
     Randomness (if ever needed) comes from a seeded RNG in engine state.
4. **SI units internally, everywhere.** Kelvin, Pascal, kg/s, m³, J.
   Unit newtypes from `core::units` are mandatory in public APIs — a bare
   `f64` crossing a crate boundary is a code-review failure. Convert to
   display units only at frontend boundaries.
5. **No panics in the engine.** `core` and `solvers` return
   `Result<_, SimError>`. `unwrap`/`expect` allowed only in tests and `cli`.
   A diverging solver is an `Err` with diagnostics, not a crash and not a NaN
   silently propagating — check for NaN/Inf after every solve.
6. **Frontends consume snapshots.** `Snapshot` is plain serde data. Frontends
   never reach into engine internals. Commands go in through
   `Engine::apply(Command)`, state comes out through `Engine::snapshot()`.

## Physics/numerics decisions (already made — don't relitigate casually)

- **Quasi-steady hydraulics:** pressures/flows re-solved to steady state each
  tick (Newton on the network); only slow states integrate over time
  (inventories, temperatures, compositions). Pressure waves are out of scope.
- **Crude = pseudo-components:** ~10–30 boiling-point cuts with Tb, MW,
  density. No molecular chemistry.
- **Reactions = lumped kinetics:** e.g. 4-lump FCC model, simple ODEs
  integrated with fixed-step RK4 inside a unit's tick.
- **Network solve:** node pressures unknowns, branch flows from element
  characteristics (valve Cv, pump curve, pipe resistance), Newton–Raphson with
  faer for the linear solves. Incompressible liquid first; gas handling is a
  later, explicit milestone.
- **Damage model (game):** a leak is an extra edge to an `Atmosphere` sink;
  a fire is a heat source on a node. Damage never needs special-case physics.

## Commands

```
cargo build --workspace
cargo test  --workspace              # must pass before any commit
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all
cargo run -p refinery-cli -- run scenarios/tank_pump_valve.toml --ticks 1000
cargo test -p refinery-solvers --release -- proptest   # slow property tests
```

`godot-ext` is in the default workspace as of M6.2, but only its **bridge**
half — the pure-Rust translation layer, which has no Godot dependency and is
covered by `cargo test --workspace`. The gdext binding lives behind an
off-by-default feature, because that is where the toolchain requirement
actually is:

```
# needs Godot >= 4.7. The --target-dir is REQUIRED, not tidiness: see below.
cargo build -p refinery-godot-ext --features godot --target-dir target/godot
cargo clippy -p refinery-godot-ext --features godot --all-targets -- -D warnings
```

That clippy line is not optional politeness: with the feature off, the
workspace lint pass does not see one line of the binding. Run it (and record
that you did) whenever the binding changes.

The separate `--target-dir` exists because the crate is `cdylib` + `rlib`, so
`cargo test --workspace` rebuilds the same `.dll` with the feature OFF and
overwrites the one Godot loads. The symptom is `GDExtension entry point
'gdext_rust_init' not found`, which does not sound like what it is.

Running the Godot frontend needs one more step that nothing warns about — the
editor must be opened once so it writes `.godot/extension_list.cfg`, which is
what a *running* project loads extensions from. See the header comment in
`refinery.gdextension`; skipping it produces a GDScript parse error naming
nothing relevant.

```
godot --headless --path . --editor --quit    # once, after cloning; exits nonzero
godot --headless --path . -- --auto          # the M6.2 demo; ends itself at t=350
```

The editor step's nonzero exit is a shutdown crash that happens with this
extension removed too — ignore it, the file it writes is what matters.

## Testing philosophy

- **Invariant property tests (proptest) are the backbone.** For any randomly
  generated valid network: mass in = mass out + accumulation (per component);
  no negative inventories, pressures, or temperatures; solver either converges
  or returns Err — never NaN.
- **Reference cases:** each unit model gets at least one test against a hand
  calculation or published example (cite the source in a comment). Keep them
  in `crates/solvers/tests/reference/`.
- **Golden snapshot tests:** known scenario + N ticks ⇒ stored snapshot,
  compared with `approx` tolerances (exact for determinism checks).
- When fixing a solver bug, first add a failing test reproducing it.

## Conventions

- Rust 2021, `rustfmt` defaults. Descriptive names over abbreviations
  (`inlet_pressure`, not `p_in`) except inside tight math kernels where the
  paper's notation may be mirrored — then cite the paper.
- Every physical equation in code gets a comment naming the law/source
  (e.g. `// Valve flow: Q = Cv * sqrt(dP / SG), ISA-75.01`).
- Errors: `thiserror` in libraries, `anyhow` only in `cli`.
- Public APIs documented with `///` including units of every quantity.
- Keep PR-sized changes: one unit model, one solver improvement, or one
  refactor at a time. Update `docs/DESIGN.md` when interfaces change.

## Git & commits

- Conventional Commits (`feat:`, `fix:`, `chore:`, `docs:`, `refactor:`,
  `test:`). Scope optional (e.g. `feat(solvers): ...`).
- Every commit must build and pass `cargo test`, `cargo clippy -D warnings`,
  and `cargo fmt --check`. No red commits on the default branch.
- `Cargo.lock` is committed (determinism: pinned dependency versions).

## Current milestone

See `docs/ROADMAP.md`. Work only on the current milestone unless asked.
M1–M6 are closed: flow network, heat, crude + simple column, reactor, gas and
pressure realism, and damage + the Godot frontend. **M7 (complex column, stage
cascade) is scoped**: the design note is landed (DESIGN §5, "Complex column
(M7)") and M7.1–M7.4 are open boxes in ROADMAP. The note's verdicts are
decisions, not results — expect building it to correct them, as every earlier
milestone's note was corrected.
