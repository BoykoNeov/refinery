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

**M8 is open, and its scope is regulation** — control loops, so a plant holds
itself somewhere instead of being held by whoever is sending commands. It opened
with a defect rather than a feature: **M8.0 landed 2026-08-26**, the anchoring
active-set loop (DESIGN §3c), which un-defers M5's FINDING 2 — `network::prepare`
used to freeze the anchored set at the seed compile, so a relief valve whose
`conducts` depends on the pressure *iterate* could be classified stale in either
direction. **M8.1 landed 2026-08-26**: the regulation design note, DESIGN §10,
seven forks argued before any code. The three worth knowing before touching this
milestone — a control loop lives *beside* the graph (`PlantGraph::controls`), not
as a node and not as a field on the actuator it writes; the algorithm is a trait
but the project's **first per-instance seam**, so "rule 2 says trait" had to be
argued rather than inherited, and its impls own STATE where every earlier seam's
are pure; and the loop runs at the TOP of the tick on the *previous* tick's
state, because reading this tick's solve and writing an actuator is an algebraic
loop.

**M8.2 landed 2026-08-26** — the seam itself: `PlantGraph::controls`, the
`Controller` trait (the project's first `Vec<Box<dyn _>>` and its first seam whose
impls own state), `ProportionalController`, the `[[controls]]` table, and the two
loop commands. Five details of the note were corrected while building; three
matter before touching M8.3.

**`initial_output` is deliberately NOT in M8.2, and the reflex that wants it is a
trap.** A proportional controller with no bias shuts its valve completely at
setpoint, which makes `u = u_b + K·e` look obviously right. It is not: fork 5
defines `initial_output` as the loop's *memory* and a P loop has none, and M8.3's
gate reads the P loop's steady-state offset as a signal — a bias makes that offset
a function of how well the bias was chosen instead. So the algorithm is
`u = clamp(K·e, 0, 1)`, the offset is large and honest, and the key is not in the
`[[controls]]` struct at all.

Two more: the tuning key is **`gain_per_m`**, not the note's bare `gain`, by fork
4's own argument about `setpoint_m` (a gain is `1/m` on a level loop and `1/Pa` on
a pressure loop). And **two refusals the note names have no reachable path today**
— both directions of "the setpoint's variable disagrees with the loop's", which one
`ControlledValue` variant makes unrepresentable — so they are recorded in comments
naming their own expiry rather than shipped as guards nothing reaches.

A control loop can now slam a valve shut between two ticks, and **a branch driven
to zero flow in ONE tick stalls the Newton solver**. That is NOT the loop's defect:
`Command::SetValveOpening` writing the identical endpoint fails identically, which
is the control that settles it. Reached gradually the same endpoint converges. It
is pinned by `a_branch_shut_in_one_tick_stalls_the_solver_whoever_shuts_it`, which
is written to fail when the solver is fixed, and it belongs to a `newton_flow`
slice. Practically: a level loop needs a gain gentle enough not to clamp to zero
in one step, or a setpoint step small enough not to.

No file in `scenarios/` declares a `[[controls]]` table — the two plants that carry
a loop are inline test fixtures, because the thirteen shipped files ARE the
regression anchor (26 runs byte-identical across this slice). The wired demo that
regulates is M8.4's. `docs/ROADMAP.md` M8.3–M8.5 are the remaining slices.

M1–M7 are closed: flow network, heat, crude + simple column, reactor, gas and
pressure realism, damage + the Godot frontend, and the complex column. **M7 closed
2026-08-18** — the design note (DESIGN §5, "Complex column (M7)"), the
`SeparationModel` seam (M7.1), `ThermoModel::k_value` with `TroutonThermo` and the
Rachford–Rice flash (M7.2), `StageCascade` with `CascadeSpec` and stage-located
`ColumnDraw`s (M7.3), and M7.4's three slices: tray temperatures through
`edge_temperature_at`'s column arm (a), `dh_vap` plus both duties plus the
saturated-liquid feed guard (b), and I7's cascade arm plus
`scenarios/crude_column_cascade.toml` (c).
The note's verdicts are decisions, not results — M7.1 corrected its call shape,
M7.2 its signature and gate structure, M7.3 found that the constant-α model
M7.2 shipped *for* the cascade could not have driven one stage of it, M7.4a
falsified its own gate's justification with a mutation, and **M7.4b and M7.4c each
found that a gate their own box specified cannot exist** — both because the
reboiler duty is *defined* to close the balance those gates would have checked it
against. The rule that came out of it: a quantity defined to close a balance can
never be gated by that balance, so check an invariant's two sides are computed by
independent paths before writing it.

The two column demos are a PAIR and are meant to be diffed:
`crude_column.toml` (cut-point) and `crude_column_cascade.toml` (cascade) run the
same crude into the same three tanks at the same three rates, and differ only in
what the separation model does with it. `crude_column.toml`'s first cut sits at
155 °C so the heavy naphtha boils inside its smearing ramp — that is deliberate
and is the only exercise `smearing_k` gets on a wired plant.

A cascade column is specified by ratios only: `reflux_ratio` (molar, internal)
plus one `draw_ratio` per draw (a **mass** fraction of the feed, with the bottoms
left over). An absolute product rate in kg/s is inadmissible — it re-runs the
failure that killed M3.2's prescribed-draw column. `N` counts the reboiler and
excludes the total condenser.

The two separation fidelities carry mutually exclusive config, enforced at load in
both directions: `up_to_c` + `smearing_k` are the splitter's, `stage` +
`draw_ratio` + `phase` + `[cascade]` are the cascade's. Adding a knob to one means
refusing it on the other.

Draw temperatures are real tray temperatures and are **read**:
`energy::column_draw_at` is the single owner of "which draw is this edge" for both
composition and temperature, so the two fields of a draw always come from the same
draw. A cascade column is therefore not enthalpy-neutral, and M7.4b's duties are
what close its external energy books.

**The two duties are not symmetric, and treating them as if they were is the
mistake to avoid.** The condenser duty is its own exact envelope; the reboiler
duty is *defined* as the condenser duty plus the column's external sensible
balance. That asymmetry is forced: constant molar overflow leaves every interior
stage with an energy residual, so a locally-exact reboiler duty would disagree with
the column's own balance by several times the quantity that balance measures. Two
consequences to hold on to — the reboiler duty carries the formulation's error and
the condenser duty does not, and **"the difference equals the external balance" is
a tautology, not a gate** (DESIGN §5, M7.4b correction 1).

Both duties are `Option<Watt>`: `None` from the cut-point splitter, which has no
such equipment, and `Some` from the cascade — including `Some(ZERO)` for an idle
column, which is an answer rather than an absence. `NodeSnapshot::column_duty`
carries that outward and is skipped when `None`.

A cascade's feed must be a **saturated liquid** and this is now refused rather than
assumed, in both directions, outside a window of `ε·Δh_vap/c̄p` at `ε = 1%` — about
±1.2 K on the M7.3 slate, wider on a heavier one. Any new cascade scenario has to
be built with its feed on the mix's bubble point at the column's pressure; that is
a design input for the file, not something to tune afterwards.
