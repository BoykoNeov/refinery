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
  scenarios/  # Plant definition format (TOML), loader, engine builder:
              # schema.rs (the document), build.rs (document → Engine),
              # validate.rs (the load-time refusals). API re-exported from lib.rs.
  cli/        # Headless runner: `run` one scenario to JSON snapshots;
              # `corpus` runs every scenario and reports iterations, wall time
              # and a per-plant fingerprint (the "runs byte-identical" check).
  godot-ext/  # GDExtension adapter. The ONLY crate that knows Godot exists.
docs/         # DESIGN.md (architecture + physics), ROADMAP.md (milestones),
              # DEFERRED.md (every open hurdle, its un-defer trigger, and the
              # measured distance from it — read it before scoping a slice),
              # MILESTONES.md (every earlier milestone's close-out report)
scenarios/    # *.toml plant definitions (start with tank_pump_valve.toml)
.github/      # CI: fmt, clippy, test, the godot-feature lint, and the corpus
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
cargo run -p refinery-cli -- run scenarios/tank_level_control.toml --ticks 6000   # the M8.4 loop demo
cargo run -p refinery-cli -- run scenarios/vessel_pressure_control.toml --ticks 6000  # the M10.1 pressure loop
cargo run -p refinery-cli -- run scenarios/crude_column_boiloff.toml --ticks 6000      # the M12.1 boiling tanks
cargo run -p refinery-cli -- run scenarios/crude_column_recovery.toml --ticks 6000     # the M14.1 recovery drum
cargo run -p refinery-cli -- run scenarios/fired_gas_drum.toml --ticks 6000        # the M16.2 shaped heat capacity
cargo run -p refinery-cli -- run scenarios/tank_temperature_control.toml --ticks 6000 # the M17.1 temperature loop
cargo run -p refinery-cli -- run scenarios/tank_temperature_heating.toml --ticks 6000 # the M18.1 reverse-acting loop
cargo run -p refinery-cli -- run scenarios/furnace_outlet_control.toml --ticks 6000   # the M19.1 furnace-outlet loop
cargo run -p refinery-cli -- run scenarios/tank_flow_control.toml --ticks 6000        # the M20.1 flow loop
cargo run -p refinery-cli -- run scenarios/relief_twin_vessels.toml --ticks 6000 --solver simple  # the M21.1 stiff pairs
cargo run -p refinery-cli -- run scenarios/tank_overfill_trip.toml --ticks 6000       # the M22.1 overfill trip
cargo run -p refinery-cli -- run scenarios/tank_runs_dry.toml --ticks 6000            # the M24.1 dry tank
cargo run -p refinery-cli -- run scenarios/tank_overflow.toml --ticks 6000            # the M23.1 spill
cargo run -p refinery-cli -- run scenarios/furnace_cascade_control.toml --ticks 6000  # the M25.1 cascade
cargo run -p refinery-cli -- run scenarios/tank_level_fill_control.toml --ticks 6000  # the M29 fill-valve loop
cargo run -p refinery-cli -- run scenarios/tank_level_fill_check_valve.toml --ticks 6000  # the M30 check valve
cargo run -p refinery-cli -- run scenarios/gas_receiver_check_valve.toml --ticks 6000      # the M31 gas check valve
cargo run -p refinery-cli -- run scenarios/tank_overheat_trip.toml --ticks 6000            # the M32 fuel cut
cargo run -p refinery-cli -- run scenarios/furnace_low_flow_trip.toml --ticks 6000         # the M33 low-flow trip
cargo test -p refinery-solvers --release              # slow property tests

# The corpus: every shipped scenario, worst solver iterations per tick, wall
# time of the tick loop, and a fingerprint over every snapshot. `--out` before
# a change and `--baseline` after is the "runs byte-identical" claim as an
# exit code. Release, because wall time is one of its columns. A plant that
# fails to load or fails a tick exits nonzero on its own, baseline or not.
# Baseline rows are matched by plant NAME alone, so a baseline recorded under
# one fidelity and compared against a run under the other now reads "moved" and
# fails, where it used to read "new" and pass. Keep a baseline per fidelity.
cargo run --release -p refinery-cli -- corpus scenarios --ticks 6000
cargo run --release -p refinery-cli -- corpus scenarios --ticks 6000 --solver simple
cargo run --release -p refinery-cli -- corpus scenarios --ticks 6000 --out before.json
cargo run --release -p refinery-cli -- corpus scenarios --ticks 6000 --baseline before.json
```

CI (`.github/workflows/ci.yml`) runs the four gate commands, the godot-feature
clippy, the release property tests, and the corpus under both fidelities on
every push and pull request. A red main is now a red check, not a memory. What
CI does NOT check is that the numbers are unchanged: no baseline file is
committed, so its corpus steps assert only that every plant still runs. The
before/after comparison stays a per-slice measurement made by hand.

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

The full close-out report of every earlier milestone (M1–M33), with what each
one measured and corrected, is in `docs/MILESTONES.md`. It is not loaded
automatically: read the relevant box there before touching that milestone's code.
When a milestone closes, its box goes here and the previous one moves there.

**M34 is CLOSED (2026-10-03): a furnace's tube coil — `docs/DEFERRED.md` row B39
struck, E10 narrowed to coolers, new row B40.** Taken on a DECISION (the user's):
every furnace gets one, knowing it moves thirteen plants. One slice: DESIGN §37.
- **`FurnaceCoil`** on `NodeKind::Furnace`: heat capacity, metal-to-process `UA`,
  and a temperature that is a STATE (written back at the end of every tick). Duty
  and fire go into the metal; the fluid takes `G·(T_c − T_in)`,
  `G = W·(1 − e^(−UA/W))`. Exact step (never divides by `G`), and the fluid gets
  `Q − C·ΔT_c/dt` from the STORED change, so the first law closes at the furnace.
- **Three required keys**, no defaults: `coil_heat_capacity_mj_per_k`,
  `coil_ua_kw_per_k`, `coil_temperature_c`. Shipped coils: 1 MJ/K per rated MW,
  `UA` = 2 × the load capacity rate, loaded at the steady coil on TICK 2's flow.
- **A furnace is never `held`**: with no flow the fluid reads the coil. Outlet
  loops act on a stagnant coil; a stall no longer opens a furnace cascade; the
  held-outlet rule survives for COOLERS and is gated there. Outlet trips stay
  refused, each unit for its own reason (M35 builds the furnace's).
- **21 plants byte-identical, 13 moved**; only `fired_gas_drum`'s iterations moved
  (4 571 → 13 074 Newton). Steady states unchanged. M32's trip now clears at tick
  1 294, not inside its tripping tick; M33's twin reads 419 °C at 5 000, not 1 521.
- **The coil reached E19's trigger** (a cascade furnace at full fire dips a hair
  off its limit for 1–3 ticks); closed by a saturation latch, DESIGN §38.

**Exactly nine of the thirty-four files in `scenarios/` declare a `[[controls]]`
table** — `tank_level_control.toml` (M8.4, a level),
`vessel_pressure_control.toml` (M10.1, a pressure),
`tank_temperature_control.toml` (M17.1, a temperature),
`tank_temperature_heating.toml` (M18.1, a reverse-acting temperature),
`furnace_outlet_control.toml` (M19.1, a furnace's own outlet),
`tank_flow_control.toml` (M20.1, a valve's own flow) and
`furnace_cascade_control.toml` (M25.1, a cascade: two loops),
`tank_level_fill_control.toml` (M29, a level on its FILL valve) and
`tank_level_fill_check_valve.toml` (M30, the same loop behind a check valve,
with a trip). **The
other twenty-five
were written before M8 (thirteen of them) or after it without a loop, and ARE
the regression anchor** (three, `tank_overfill_trip.toml`,
`tank_overheat_trip.toml` and `furnace_low_flow_trip.toml`, carry a `[[trips]]` table instead, one, `tank_runs_dry.toml`, runs a tank dry, one,
`tank_overflow.toml`, spills, and one, `gas_receiver_check_valve.toml`, puts a
check valve in gas service); adding a loop to one of them
would move its snapshot, which is why each regulation slice ships a NEW file
rather than wiring one into an existing plant. Every other plant that carries a
loop is an inline test fixture for the same reason.
