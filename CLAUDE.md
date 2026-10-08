# Refinery Simulator — Project Guide

Headless, deterministic oil-refinery simulation engine in Rust. Science-based
approximations (pseudo-components, lumped kinetics, network hydraulics).
Frontends: Godot 4 game (GDExtension), CLI batch runner, future dashboards.
Fidelity is swappable per physical concern (simple ↔ complex) via trait
implementations selected in scenario config — never via `if simple_mode`.

## Workspace layout

```
crates/
  core/       # Types, units, plant graph, engine loop, solver TRAITS, snapshots.
              # ZERO heavy deps. Never imports Godot, never imports solvers.
  solvers/    # Trait IMPLEMENTATIONS (flow, thermo, kinetics). core + faer.
  scenarios/  # TOML plant format: schema.rs (document), build.rs (→ Engine),
              # validate.rs (load-time refusals). API re-exported from lib.rs.
  cli/        # `run` one scenario to JSON snapshots; `corpus` runs every
              # scenario: iterations, wall time, per-plant fingerprint.
  godot-ext/  # GDExtension adapter. The ONLY crate that knows Godot exists.
docs/         # DESIGN.md (architecture + physics), ROADMAP.md (milestones),
              # DEFERRED.md (open hurdles, un-defer triggers — read before
              # scoping a slice), MILESTONES.md (close-out reports, newest first)
scenarios/    # *.toml plant definitions
.github/      # CI: fmt, clippy, test, godot-feature lint, corpus
```

## Hard architectural rules

1. **`core` is sacred.** No Godot types, no solver implementations, no I/O
   beyond serde. If `core` must change to satisfy a frontend, the adapter is wrong.
2. **Fidelity = trait impl selection.** `FlowSolver`, `ThermoModel`,
   `ReactionModel` are traits in `core`; impls live in `solvers`; scenario TOML
   picks which. `if fidelity == Simple` in shared code is a bug.
3. **Determinism is non-negotiable.**
   - Fixed timestep. Same scenario + same seed ⇒ bit-identical snapshots.
   - NEVER iterate a `HashMap`/`HashSet` in simulation code. Use `Vec`,
     `BTreeMap`, or `IndexMap`.
   - No parallelism in the tick loop (float reduction order). Revisit only
     with benchmarks and a determinism plan.
   - No wall-clock time or thread randomness in `core`/`solvers`; randomness,
     if ever needed, comes from a seeded RNG in engine state.
4. **SI units internally, everywhere.** Kelvin, Pascal, kg/s, m³, J. Unit
   newtypes from `core::units` are mandatory in public APIs — a bare `f64`
   crossing a crate boundary is a review failure. Display units only at
   frontend boundaries.
5. **No panics in the engine.** `core` and `solvers` return
   `Result<_, SimError>`. `unwrap`/`expect` only in tests and `cli`. A diverging
   solver is an `Err` with diagnostics — check for NaN/Inf after every solve.
6. **Frontends consume snapshots.** `Snapshot` is plain serde data. Commands in
   through `Engine::apply(Command)`, state out through `Engine::snapshot()`.

## Physics/numerics decisions (made — don't relitigate casually)

- **Quasi-steady hydraulics:** pressures/flows re-solved to steady state each
  tick (Newton on the network); only slow states integrate (inventories,
  temperatures, compositions). No pressure waves.
- **Crude = pseudo-components:** ~10–30 boiling-point cuts (Tb, MW, density).
- **Reactions = lumped kinetics** (e.g. 4-lump FCC), fixed-step RK4 per tick.
- **Network solve:** node pressures unknowns, branch flows from element
  characteristics (valve Cv, pump curve, pipe resistance), Newton–Raphson, faer.
- **Damage:** a leak is an extra edge to an `Atmosphere` sink; a fire is a heat
  source on a node. No special-case physics.

## Commands

```
cargo build --workspace
cargo test  --workspace                       # must pass before any commit
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all
cargo test -p refinery-solvers --release      # slow property tests
cargo run -p refinery-cli -- run scenarios/<file>.toml --ticks 6000
#   (relief_twin_vessels.toml is the M21 demo: add --solver simple)

# Corpus: every scenario's worst iterations/tick, wall time, snapshot fingerprint.
# --out before a change, --baseline after = "byte-identical" as an exit code.
# Release (wall time is a column). Baselines match by plant NAME only, so keep
# one baseline per fidelity — mixing them reads "moved" and fails.
cargo run --release -p refinery-cli -- corpus scenarios --ticks 6000 [--solver simple]
cargo run --release -p refinery-cli -- corpus scenarios --ticks 6000 --out before.json
cargo run --release -p refinery-cli -- corpus scenarios --ticks 6000 --baseline before.json
```

CI runs the four gate commands, the godot-feature clippy, the release property
tests and the corpus under both fidelities. No baseline is committed, so CI only
asserts every plant runs; the before/after comparison is a per-slice manual step.

**Godot.** `godot-ext` builds by default as its pure-Rust bridge only; the gdext
binding is behind the `godot` feature (needs Godot ≥ 4.7):

```
cargo build  -p refinery-godot-ext --features godot --target-dir target/godot
cargo clippy -p refinery-godot-ext --features godot --all-targets -- -D warnings
```

- Run that clippy (and record it) whenever the binding changes — the workspace
  lint never sees the binding.
- `--target-dir` is REQUIRED: the crate is `cdylib`+`rlib`, so `cargo test
  --workspace` would overwrite Godot's `.dll` with a feature-off build. Symptom:
  `GDExtension entry point 'gdext_rust_init' not found`.
- After cloning, open the editor once so it writes `.godot/extension_list.cfg`
  (else: a GDScript parse error naming nothing relevant). Its nonzero exit is a
  harmless shutdown crash.

```
godot --headless --path . --editor --quit    # once, after cloning
godot --headless --path . -- --auto          # the M6.2 demo; ends at t=350
godot --path . res://demo/furnace.tscn       # the M39 furnace screen (keys on screen)
godot --headless --path . res://demo/furnace.tscn --quit-after 20000 -- --auto [--plant=burnout]
godot --path . res://demo/pump.tscn          # the M51 pump screen (keys on screen)
godot --headless --path . res://demo/pump.tscn --quit-after 20000 -- --auto [--plant=boiling|gaslock]
```

## Testing

- **Invariant property tests (proptest) are the backbone.** For any random
  valid network: mass in = mass out + accumulation (per component); no negative
  inventories, pressures or temperatures; solver converges or returns Err —
  never NaN.
- **Reference cases:** each unit model gets ≥ 1 test against a hand calculation
  or published example (cite the source). Keep them in
  `crates/solvers/tests/reference/`.
- **Golden snapshots:** scenario + N ticks ⇒ stored snapshot, `approx`
  tolerances (exact for determinism checks).
- Fixing a solver bug: first add a failing test reproducing it.

## Conventions

- Rust 2021, `rustfmt` defaults. Descriptive names (`inlet_pressure`, not
  `p_in`) except in tight math kernels mirroring a paper — then cite it.
- Every physical equation gets a comment naming its law/source
  (e.g. `// Valve flow: Q = Cv * sqrt(dP / SG), ISA-75.01`).
- `thiserror` in libraries, `anyhow` only in `cli`.
- Public APIs get `///` docs with units of every quantity.
- PR-sized changes: one unit model, solver improvement or refactor at a time.
  Update `docs/DESIGN.md` when interfaces change.
- **Never add a loop or trip to an existing scenario file** — the existing
  files are the regression anchor (it would move their corpus fingerprint).
  Each slice ships a NEW file or an inline test fixture.

## Git & commits

- Conventional Commits (`feat:`, `fix:`, `chore:`, `docs:`, `refactor:`,
  `test:`), scope optional.
- Every commit builds and passes `cargo test`, `cargo clippy -D warnings`,
  `cargo fmt --check`. `Cargo.lock` is committed.

## Current milestone

Work only on the current milestone unless asked. **Current: M56, OPEN** — the
frame budget M55 overran. M56.0 remembers the line flash's densities inside each
flow solver (`network::DensityMemo`, no answer moved); M56.1 (the user's
decision) finds the line flash's two roots by superlinear searches (two boiling
plants moved ≤ 7.6e-10). The flashing rundown is back inside the 2.5 ms budget;
the gas-lock story's locked beat is not (worst 130–174 ms on the game solver,
19 iterations every tick of a plant standing still) — M56.2 is that stall
(A24 narrowed, A25). **Latest closed: M55 (2026-10-08).** Every close-out report is at the top of `docs/MILESTONES.md`; read the relevant one
before touching that milestone's code. When a milestone closes, write its box at
the top of `docs/MILESTONES.md` and update only the line above.

`docs/ROADMAP.md` and `docs/DESIGN.md` are huge (~0.5 MB / ~1.1 MB): never read
them whole. `grep -n '^## M' docs/ROADMAP.md` and read only the section needed.
