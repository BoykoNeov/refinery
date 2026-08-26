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
cargo run -p refinery-cli -- run scenarios/tank_level_control.toml --ticks 6000   # the M8.4 loop demo
cargo test -p refinery-solvers --release              # slow property tests
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

**M9 is OPEN, and its scope is solver robustness.** It opened the way M8 did —
with a defect the previous milestone reached and deliberately did not fix. Slices
are scoped one at a time, because what the next one should be depends on what the
last one measured. Two have landed and M9.2 is unscoped.

**Both slices are about the same step**, and reading M9.1 without M9.0 will not
work — M9.1's whole argument is the closed form M9.0 derived, applied to a solver
that had no line search at all.

**M9.0 landed 2026-08-26** — the shut-in stall. The design note is DESIGN §11.
Five things to know before touching the hydraulic solver.

**The mechanism M8 recorded for this defect was false in both of its clauses, and
that is the thing to internalise.** M8 said an unbounded `dQ/dΔP` made the step
enormous and the line search cut it back. Measured: the line search accepted the
full step on all fifty iterations and never halved anything; the shut valve's
conductance is exactly ZERO rather than unbounded; and the branch drop changed
sign every iteration while the residual fell anyway. **A monotone residual history
says nothing about the iterate's path**, because the merit is even in the error
and the error is not — M8 inferred "crawling, not oscillating" from exactly that
and was wrong.

**The comment above the line search was right the whole time; the constant under
it was not.** It said Armijo "rejects [the near-symmetric overshoot] and forces
t ≤ ½". `ARMIJO_C = 1e-4` did not. **A mis-sized constant under a correct comment
is worse than a wrong comment**, because the comment is what stops the next reader
from checking.

**The fix is `ARMIJO_C: 1e-4 → 5e-2` and the arithmetic behind it is a relation
between three constants.** A shut valve orphans a node with exactly one live edge
(F6), so the failing solve is scalar; a full Newton step on `x/√(|x|+ε)` lands on
the MIRROR of the drop, `2ε` nearer the root, while a HALF step lands within `ε`
of the root from any drop at all. A solve therefore stalls iff
`2·eps_dp·max_iter < |Δp₀| ≲ eps_dp/ARMIJO_C`, which is empty iff
`ARMIJO_C ≥ 1/(2·max_iter)`. **`eps_dp` cancels — shrinking the regularisation is
the reflex and is inert.** `armijo_c_closes_the_shut_in_stall_window` asserts the
relation and fires on either half of it, so **lowering `max_iter` reopens the
window** and is a code-review failure without raising `ARMIJO_C` with it.

**Stricter Armijo made the solver FASTER, which is the reverse of the standing
objection.** Worst-case iterations per pass across 6 000 ticks of all fourteen
shipped scenarios fell 11 → 10 against a cap of 50, because rejecting the full
step forces the near-exact half step. Ten of fourteen scenarios stay
byte-identical; four move by at most `7.6e-11` relative on any physical quantity.
**From here, "runs byte-identical" means post-M9.0 identical for those four**
(`fcc_plant`, `knockout_drum`, `leaking_line`, `tank_level_control`).

**Two gates changed shape, and one of them kept a catch nobody was defending.**
`a_branch_shut_in_one_tick_stalls_...` became
`a_branch_shut_in_one_tick_converges_whoever_shuts_it` and asserts the shut
branch's ENDPOINT — zero flow, and the dead leg sitting at the tank's bottom
pressure — because `Ok(())` is passed by any line search that accepts anything.
Righting it kept M8.4's accidental catch of "the control pass moved below the
solve" (now 4.16 kg/s through a branch it says is shut), which closes the tick-order
gap M8.4 recorded as open. **Re-run an upside-down test's catches after turning it
right way up.** The demo's gain gate was re-premised rather than inverted: `0.4`
still clamps, and now recovers.

**M9.1 landed 2026-08-26** — the same step on the OTHER solver. The design note
is DESIGN §11's M9.1 half. Eight things to know.

**The shut valve is not the subject; it is where the defect stops finishing.**
`SimpleFlowSolver` has no step-rejection criterion of any kind — it applies its
full node-wise step unconditionally — so it takes the worst-available step on
EVERY valve node of EVERY plant. On the M8.2 fixture with the drain 20% open and
nothing shut anywhere, it takes 189 sweeps against Newton's 8, and the node still
moving at the end is the FEED valve on the other side of the plant, which takes
exactly 189 whatever the drain does. **"The shut-in stall on the other fidelity"
was the wrong frame, and the probe's own first row said so.**

**Half the mechanism is M9.0's closed form and half is new.** On a shut valve the
drop alternates sign every sweep and falls by exactly `2.0000 Pa`, from
`106 790.90 Pa` — predicted 53 395 sweeps, and raising the cap converges at
**53 411**. On a CONDUCTING valve the same overshoot contracts geometrically
instead, at a rate roughly proportional to the valve's conductance. One mechanism
whose contraction factor reaches 1 as the valve shuts, so **"continuous, not a
cliff" is that limit, not the drop growing** — the drop moves 2.5% across the
whole column and is not what separates the cases.

**The stall window here is unbounded above, and that is a difference in kind.**
With nothing rejecting anything it is `(2·eps_dp·max_iter, ∞)`, so **no `max_iter`
closes it**. M9.0's fork 2 was rejected on cost; here it is not available. And it
is reachable structurally: the cold seed is the mean of the pinned pressures, so
free nodes start ~200 kPa from their roots before anyone touches a valve.

**Damping and the line search are the SAME remedy — measured, both ways.**
Lowering `omega` is the reflex and makes the corpus worst case 3.5× worse
(`relief_blowdown` 868 → 3 035 at `ω = 0.5`, because that plant converges on its
vessel's `−C/dt` term, not on branch conductance). And with the line search in at
`ω = 0.5`, the corpus reproduces the no-line-search `ω = 0.5` numbers EXACTLY,
because the half step already passes its own test. `omega` stays `pub` with a
default of `1.0` and a doc comment that no longer advertises damping as the cure
for stiffness.

**§11's own deferred fork was built here and is inert on the case it was written
for.** The sign-reversal trust region is a big corpus win and reproduces the
shut-in divergence bit for bit, because the mirror step DOES shrink the imbalance
by `2ε` — so "reverses *and* does not shrink" never fires. Not an un-defer trigger
for Newton, but evidence against the rule.

**`MAX_HALVINGS = 8` is load-bearing and the closed form does NOT say so.** §11's
form says a half step lands within `eps_dp` of the root from any drop, which reads
as "one halving is enough" — predicted inert, and `MAX_HALVINGS = 1` DIVERGES
M8.0's anchoring plant at 20 000 sweeps. The form describes a **dead leg**, where
F6 leaves one live edge and the mirror is exact; a second live edge moves the root
off the mirror and the half step can be rejected too. **The closed form's reach is
narrower than the fix's**, and the comment that said otherwise was corrected.
Also from the mutation pass: a trial evaluation that is not the step's own
function **freezes** the residual (identical to the last digit for 5 000 sweeps)
rather than slowing it, and is caught by two M5-era cross-fidelity tests and by
neither new gate. The reject-all branch is **uncaught and recorded as such** — it
fires 3 281 times across the corpus, every site at `|imbalance| ≤ 3.4e-13 kg/s`.

**"No shipped scenario runs this solver" is measured, and the first probe lied.**
0 of 14 fire a `panic!` at `SimpleFlowSolver::solve`, with `leaking_line` forced
to `simple` as the control that does. The first run said 14 of 14: `panic!` at the
top of a function makes the rest unreachable, rustc **echoes that source line** in
the diagnostic, and `cargo run` replays cached warnings every invocation. Build
once, then run the binary, and require `panicked at` beside the marker.

**`ARMIJO_C = 5e-2` here is Newton's number and deliberately NOT Newton's
constant, and one M9.0 gate cannot be mirrored.** The two tests are the same test
(the factor of two is the square of the merit, not tuning), but Newton's margin is
five times a bound tied to *its* cap of 50; at 5 000 that bound is `1e-4`, and
`1e-4` leaves a valve 1% open crawling for 1 239 sweeps. Re-swept here: the knee
is between `1e-3` and `1e-2`, and the cost lands entirely on `relief_blowdown`
(6% at `5e-2`, 53% at `2e-1`). The same arithmetic kills a mirrored
`armijo_c_closes_the_shut_in_stall_window` — it would clear by 500× and pass at
almost any constant. **So the shut-in gate alone does not pin this fix**; the
throttled-valve sweep budget beside it is what fails at the slack constant, and
`a_branch_shut_in_one_tick_converges_whoever_shuts_it` now runs on BOTH
fidelities.

**M8 is CLOSED (2026-08-26), and its scope was regulation** — control loops, so a
plant holds itself somewhere instead of being held by whoever is sending
commands. Its six slices are summarized below; M9 is unscoped. It opened
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
a function of how well the bias was chosen instead. So `ProportionalController` is
`u = clamp(K·e, 0, 1)`, the offset is large and honest, and the key belongs to the
PI loop alone.

Two more: the tuning key is **`gain_per_m`**, not the note's bare `gain`, by fork
4's own argument about `setpoint_m` (a gain is `1/m` on a level loop and `1/Pa` on
a pressure loop). And **two refusals the note names have no reachable path today**
— both directions of "the setpoint's variable disagrees with the loop's", which one
`ControlledValue` variant makes unrepresentable — so they are recorded in comments
naming their own expiry rather than shipped as guards nothing reaches.

**M8.3 landed 2026-08-26** — `PiController`, the anti-windup clamp, MANUAL→AUTO
transfer and `initial_output`. Four things to know before M8.4.

**The loop's memory is stored in OUTPUT units, and that is what makes fork 4's
"same arithmetic" claim keepable.** `u = clamp(K·e + b, 0, 1)`, where `b` is the
share of the valve position the integral term owns — not `∫e dt`. Inverting for
"what memory makes the next output be `u`" is then `b = u − K·e`, one private
`back_calculate`, and all three writers of a loop's memory go through it: the
anti-windup clamp, the MANUAL→AUTO seed, and the load-time seed from
`initial_output`. With the textbook state those would have been three formulas.
The integral is also accumulated AFTER the output is computed (explicit Euler),
which is what makes the first update after a seed return the seeded position.

**MANUAL→AUTO reads the measurement FRESH, not `last_measurement`.** Commands are
applied between ticks, so the state at transfer time IS what the next control pass
will measure, and seeding against it makes the transfer exact (measured deviation
zero, against a derived few-ULP bound). The stale read is the reflex, and it was
run as a mutation: it steps the valve by 4.66e-5 — small enough that any bound
picked to "look tight" would have passed it, which is the argument for deriving
the tolerance rather than choosing one.

**Gate 3 as DESIGN §10 specifies it does not discriminate.** "The P loop with an
offset, the PI loop without one" fails as a test because a proportional loop's
offset is `e = u/K`: a big enough gain passes it with no integral term anywhere.
What a P loop *cannot* do is move its output while holding its level. So the gate
asserts that identity — the P half's level move equals its own valve travel over
the gain (0.477229 m measured against 0.477230 m forced), while the PI half moved
its valve further and its level by 0.0013 m.

`initial_output` is required on `algorithm = "pi"`, refused on `"p"` with its own
reason (it names a memory that controller does not have, not an unknown key), and
`"pid"` is now what the unknown-algorithm refusal is tested with.

**M8.4 landed 2026-08-26** — the wired demo, the mutation pass, and a measured
coverage record. Four things to know.

**`scenarios/tank_level_control.toml` is the first shipped file with a
`[[controls]]` table**, and it is meant to be diffed against
`tank_pump_valve.toml`. **A level loop must actuate a DRAIN, and that is forced,
not chosen**: the error is `measurement − setpoint` and the output is
`clamp(K·e + b, 0, 1)`, so a rising level OPENS the actuator — on a fill valve
that is runaway. Every level loop written after this one inherits that. The gain
bound is the plant's own and both sides are run: at the settled operating point
the drain sits at ~0.376, so a +1 m setpoint step subtracts `gain_per_m` in one
tick — 0.25 and 0.35 absorb it, 0.4 hits exactly 0. (M8.4 measured that 0.4
also stalled the solver; **M9.0 fixed that**, so 0.4 now clamps, recovers and
parks on the stepped setpoint. The bound is a tuning bound now, not a stability
one.) The file ships 0.25. M8.2's 0.05/0.1 pair belongs to M8.2's plant and does not
transfer.

**A PI loop cannot slam its actuator at STARTUP at any gain**, which is the
reverse of the worry the roadmap box was written with: the memory is seeded by
`b = u − K·e`, so the first output is the declared `initial_output` whatever `K`
is (gains 0.25 to 20 all survive). The startup step exists only if a file
declares the valve's `opening` and `initial_output` apart. The clamp bound is
reachable only through a setpoint move, so it takes a command to measure and
cannot be read off a CLI run.

**All seven named mutations have now been run, and four of the seven predictions
were wrong.** The two "predicted uncaught" edits were both caught — but read the
mechanisms in DESIGN §10 before trusting either word. "The loop runs after the
solve" is caught only by the test written to FAIL when the solver is fixed, so
**no gate asserts the tick order and that gap is recorded, not filled** — until
M9.0, which righted that test rather than deleting it and thereby kept the catch
as a state assertion.
"`initial_output` ignored" is caught by M8.4's own new startup gate and by the
refusal sweep (the key's range check lives inside `seed_from_output`, so an edit
that stops calling it stops validating too). And "back-calculation dropped on
MANUAL→AUTO" fires the transfer gate alone while gate 4 stays green, falsifying
the table and confirming M8.3's revision of it.

**What the demo does NOT cover was measured, not assumed** — a `panic!` compiled
into each site, the shipped file run for 6 000 ticks, with the load-time seed as
the control that must fire. Not reached: the anti-windup arm (the output never
leaves `[0.194, 0.384]`), `ProportionalController` (no shipped file selects it),
the `Manual` arm of the tick pass, the engine's range backstop. And because the
CLI issues no commands, **the MANUAL→AUTO transfer has no wired exercise at
all** — it is covered by fixtures only.

A control loop can now slam a valve shut between two ticks, and M8.4 found that
**a branch driven to zero flow in ONE tick stalled the Newton solver**. That was
never the loop's defect — `Command::SetValveOpening` writing the identical
endpoint failed identically — and **M9.0 fixed it in the solver** (see the M9 box
below). A level loop no longer needs a gain gentle enough to avoid clamping; it
still wants one, for tuning reasons.

**Exactly one file in `scenarios/` declares a `[[controls]]` table** —
`tank_level_control.toml`, M8.4's. The other thirteen were written before M8 and
ARE the regression anchor; adding a loop to one of them would move its snapshot.
Two more plants that carry a loop are inline test fixtures for the same reason.

**M8.5 landed 2026-08-26, and M8 is closed** — `Snapshot::slate`, so a frontend
can turn a tank's mass into a fill level. Four things to know.

**The regression anchor moved by exactly one key, and it moved on purpose.**
`slate` is the first field on `Snapshot` with neither `serde(default)` nor
`skip_serializing_if`, so every scenario's JSON changed. That is not an oversight
copied from the wrong pattern — it is the discriminating argument: `controls: []`
and `column_duty: None` are *true statements* about a plant, while an empty slate
is impossible (`Slate::new` refuses one), so a `default` would let an old document
deserialize into a snapshot claiming the plant has no components. Measured rather
than predicted: strip `"slate":[…],` from each of the 28 after-runs and all 28
reproduce their before-file byte for byte. **From here, "runs byte-identical"
means post-M8.5 identical.**

**A tank's pressure cannot gate its density, and that is algebra, not a gap.**
The obvious independent check on a published density is the tank's hydrostatic
head — but `P − P_ATM = ρ·g·h = ρ·g·(m/ρA) = m·g/A`, so the density cancels
exactly and a snapshot shipping `cp` in the density slot would move both sides
identically. Every other candidate cancels the same way: a density is observable
only through a *volume*, and the only volume a scenario declares is
`initial_level_m`. So the load-time level is the **single** anchor outside the
code, it exists only at tick 0, and that is what the real gate reconstructs. The
impossibility is kept as an assertion rather than dropped, because it is the
first thing the next person will reach for.

**A snapshot's tank pressure and its tank mass are one Euler step apart** — 0.67
Pa, 8.6e-6 relative, found by that assertion failing. The tick is solve →
transport → unit dynamics, so the pressure came from the mass at the *start* of
the tick. Nothing is wrong; a frontend drawing a level reads mass, the fresh one.

**A tank's component densities are never `null`, so the scene needs no fallback.**
The `Option` exists because a slate may carry gas cuts; it is unreachable down
the fill path because the loader refuses a gas-phase tank and a gas holdup is a
`vessel`, whose state is a pressure. Swept across all fourteen shipped files.
The wired demo (`leaking_line.toml`) is water-only, so **it cannot exercise
mixing at all** — that is covered on `crude_column.toml`, whose naphtha tank
becomes a real two-component mixture as it fills.

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
