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
              # measured distance from it — read it before scoping a slice)
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

**M11 is CLOSED (2026-09-06), and its scope was the cavitation criterion.**
M11.0 wrote the note (DESIGN §13, seven forks) and M11.1 built it: a third method
on `ThermoModel`, a per-tick criterion in `Engine::tick`, `NodeSnapshot::cavitation`,
`scenarios/cavitating_pump.toml` and twelve gates. It is **the first milestone
taken on a decision rather than on a measurement** — neither of B1's trigger
clauses had fired, and clause (b), "a frontend needs to display cavitation", is a
decision that was made rather than an event that arrived. What licensed it: §3
told frontends for ten milestones to read a *wrong* number as the signal, and the
M10 close-out's correction left them with nothing.

**The row's own noun was wrong and the milestone ships neither of the things it
names.** B1 says "floor", §3 promised a "clamp"; §13 fork 1 rejects the clamp
because the vapour it would account for is mass in a phase the state vector lacks
— which *is* `docs/DEFERRED.md` B3. M11 ships a **criterion and a signal**, the
hydraulics are untouched, and a cavitating pump still delivers full head. That
disagreement is deliberate and is now its own ledger row (B9).

**The finding that re-premised the row: B1's 1.865× was a number no engine
configuration could produce.** `heat_recovery` declares `thermo = "constant"`,
whose `k_value` is an `Err`; the number came from the M10 close-out's standalone
script. **Fourteen of the fifteen pre-M11 plants select `constant`**, so under
B1's original four-kind node list ("pump, valve, junction or exchanger") the
engine-computable set across the whole corpus was **EMPTY** — every node of those
kinds sits on a plant whose model refuses, and the one plant that can answer
(`crude_column_cascade`) has none of them. Its only flow-path node is a
**furnace**, so §13 fork 4 enumerates all fourteen `NodeKind` variants and the
ledger's trigger now names six kinds. **A distance is a property of the engine,
not of the plant.**

Five things the next milestone inherits. **`None` never means "healthy"** — it
means there is no criterion here (wrong node kind, a gas, a model that cannot
answer, or before the first tick), and fourteen plants emit nothing at all; a
frontend must render that as *unknown*. **The error VARIANT is load-bearing**:
`SimError::Scenario` from a thermo model means "this fidelity cannot answer" and
reports nothing, every other variant fails the tick — three of the workspace's six
`ThermoModel` impls are test stubs and each had to be told which it meant. **An
exclusion cannot be gated on the verdict**: an excluded node publishes nothing, so
the only way to show the exclusion is doing work is to evaluate the criterion
independently and find it would have fired — and the plant has to actually be in
the state, which `crude_column_cascade`'s naphtha tank is not until **tick 1 826**.
**The demo carries no holdup at all**, because the obvious shape (a hot rundown
tank feeding a pump) would have shipped a plant sitting in B3's state. And
**`thermo = "trouton"` now changes numbers on a plant with no column** — M11 is
the first consumer that makes the key matter there, which was measured before it
was relied on (the fidelity switch alone moves nothing: 175 197.988 Pa either
way).

**From here, "runs byte-identical" means post-M11 identical.** Fourteen of the
fifteen pre-M11 plants are unchanged on both fidelities; `crude_column_cascade`
carries exactly one extra key on exactly one node (`preheater`), verified by
stripping the key and reproducing the before-file byte for byte. No solver
iteration count moved. `scenarios/` now holds **sixteen** files.

**M10 is CLOSED (2026-09-06), and its scope was the second controlled variable.**
Pressure is built (M10.0 + M10.1) and pressure is all it built — temperature and
flow were explicitly not committed to and stay deferred as ledger row E1b. It is
the first milestone chosen from `docs/DEFERRED.md` rather than handed over as a
defect. E1 — pressure control — was the row; the milestone was scoped one step
wider on purpose, because everything the note argues is machinery the *second*
variable pays for and the third and fourth inherit. **That argument was tested
and came back stronger than stated, so a third variable has nothing left to prove
about the seam**, which is why the milestone closes here rather than continuing.

**The asymmetry between the two variables left behind is recorded, not acted on.**
A temperature really is absent before the first tick — resolved by the tick, not
stored on the graph — so a temperature loop *would* owe the tick-0 rule pressure
turned out not to owe. A flow lives on an EDGE, and `measure` takes a node, so
flow control is a signature change rather than a new match arm. **Neither has a
plant asking**, which is the sentence that keeps twenty other ledger rows
deferred.

**The close-out's own work was `docs/DEFERRED.md` row B1 — the cavitation floor —
and it falsified the sentence the row rested on.** DESIGN §3 told frontends to
read *negative absolute node pressure* as "cavitating". A liquid boils below its
**vapour** pressure, which is positive, so §3's marker fires **late by exactly
that vapour pressure**. Measured on the reference plant with one number changed
(the pump mounted above its tank): cavitation begins at **17.44 m** of suction
lift and §3's marker only fires at **18.02 m** — a band in which the plant is
boiling and reported healthy. It is narrow only because cold water boils at
5 640.6 Pa; on light naphtha at 445.75 K the same marker would be **nine bar**
late. §3's *reachability* claim was true and is now measured: 19 m of lift gives
−9 522.9 Pa with the solver converging and 9.8 kg/s flowing. §3 is corrected:
**the engine does not detect cavitation and has no signal for it**, and a
negative pressure is a symptom after the fact, not a criterion. **The first draft
of this write-up said the marker "can never fire", which was an overstatement
caught by running the probe instead of reasoning about it.** The row's old distance ("the lowest pressure is 100 000 Pa, so nothing
is near a vapour pressure") compared a pressure against **zero** while concluding
something about a vapour pressure it never evaluated — and its 100 000 Pa was a
*declared sink*, not a solved state. Against each node's own bubble point the
tightest solved margin is **1.865×**. B1's trigger is now written: a solved
pressure in the **hydraulic path** (pump, valve, junction, exchanger — not a
holdup) falling below that node's bubble pressure, or a frontend needing to
display cavitation. **The node-kind clause is load-bearing**: without it the
trigger fires immediately on two product tanks, and a tank above its bubble point
is B3's two-phase holdup, not cavitation. **Clause (a)'s reachability was measured,
not assumed** — one edit to one shipped file reaches it — because a trigger no
plant can reach would have been the fifth dead gate in this project's record.

**The sweep moved a different row, and that is the sharper finding.** B3 (phase in
the state vector) said "no shipped plant asks" and guards three paths at load.
There is a **fourth path no refusal names, because it does not exist at load**: a
column draws at real tray temperatures, the product tank has no cooler, and the
tank stores a liquid the model's own correlation says is boiling.
`crude_column`'s `naphtha_tank` sits at **0.30×** its own bubble pressure,
sustained and worsening as the draw heats it (395 K → 432 K over the run), and
reports as liquid throughout. **A load-time refusal cannot catch a state that
emerges during the run.** Whether the fix is a product cooler in two files or
phase in the state vector is left open in the row.

**Three of four categories in that sweep were exclusions, and saying so is part
of the result.** Ten of fifteen plants have a node below its bubble pressure:
five declare gas-phase cuts (a liquid bubble-point test is meaningless there),
two are columns (which are *at* their bubble point by definition), two are the
FCC plants (B3 already). Publishing the headline without the exclusions would
have moved two rows on evidence that does not exist.

**M10.0 landed 2026-09-06** — the design note, DESIGN §12, six forks and five
gates, no code. Four things to know before the building slice.

**The reason §10 gives for deferring pressure is FALSE, and it is the third
recurrence of that exact error.** Fork 3 says a pressure is *solved* and lives in
`last_solution`/`NodeStates`, empty before tick 1, so a pressure loop has no
measurement at tick 0. A `Vessel`'s pressure is `m/C` with `m` on the graph —
**stored**, and exactly the declared figure, because `build.rs` computes the
initial mass as `P · capacitance(slate)` through the same method `pressure`
divides by. The measurement path already exists, `measure`'s signature does not
change, and the promised tick-0 rule is not owed. The claim is true only of a
*junction's* pressure. §3a fork 4 made the same class of error, and §10 fork 3
corrected M8.1's version of it **one paragraph before committing it again** about
the variable it was deferring.

**The ledger's own distance for E1 was wrong: `relief_blowdown` has NO ordinary
valve** — source, vessel, PSV, sink — and the PSV is refused as an actuator by
name. It lacks an actuator, not a variant, and adding one would move a regression
anchor. The demo is a new file, as M8.4's was. Row corrected.

**The schema's prediction `setpoint_pa` is wrong; the keys are `setpoint_bar` and
`gain_per_bar`.** Every pressure a scenario declares is in bar (six keys across
four node kinds, no `_pa`). The trap that comes with it, named in advance: the controller's
arithmetic is in Pascals, so the setpoint AND the gain both convert at the same
site, and converting one without the other is a factor of 100 000 no type
catches — a gain is a bare `f64` all the way in.

**M8.4's "a level loop must actuate a drain" is too narrow.** What the sign
convention forces is that the actuator is an **outlet of the measured holdup** —
a drain for a level, a vent for a vessel. The demo vents to flare, so the
direction question never arises; throttling the make-up instead is reverse
acting, refused at load in both controllers by design, and is deferred with its
own trigger (DEFERRED E7) rather than smuggled in as a negative gain. The demo
also carries **no PSV** (a shut relief valve is the dead-end shape behind A3) and
its vent must be sized to sit interior at steady state, or the milestone repeats
M8.4's coverage gap where the wired loop never reached its own saturation arm.

**M10.1 landed 2026-09-06** — the measured variable, the vessel arm, the two
config keys and the demo plant. The design note is DESIGN §12, "Corrections from
building it". Seven things to know.

**The seam held, and it held wider than M8 claimed it would.** M8.2 built the
control machinery with one variable in it and asserted it was variable-agnostic.
Not one line of `Engine::run_control_loops` changed — and neither did the
`Controller` trait, either controller implementation, `ControlLoop`, `ControlMode`,
`ControlSnapshot` or `Snapshot`. The whole milestone is two enum arms, four match
arms, two scenario keys and a demo. This project's record is that about half its
predictions are wrong; this one was right, and saying so is part of the record.

**What did NOT hold is one level down: `ControlledValue::error` became unsound and
fork 6 did not name it.** Its doc read "both arguments are the same type by
construction, so a level measurement cannot be differenced against a pressure
setpoint" — **a property of there being ONE variant, not of the type.** With two,
`error(Pressure{5e5}, Level{4.0})` returns a plausible `499996.0`, metres
subtracted from Pascals. It now returns `NaN`, which the engine's existing
finite-output check turns into a diagnosed error. The wider lesson: **a safety
argument resting on a type having one inhabitant expires silently, because the
code does not change.** It also showed the `SetSetpoint` variable guard is
load-bearing rather than cosmetic — it and the tick pass's `setpoint.variable()`
are what keep `error` sound.

**The setpoint's `×1e5` and the gain's `÷1e5` are resolved at ONE site, above the
algorithm match, and that placement is the fix rather than tidiness.** Both `"p"`
and `"pi"` need a gain, so following the existing per-arm shape would have put the
conversion at two sites. Measured: converting one without the other fires no
solver, mass or energy test — the loop stays stable, merely mistuned by five
orders — but it does move the settled operating point, so the demo gates catch it
too. The note's "gate 3 and nothing else" was wrong in the "nothing else" clause.

**Fork 5's reason for expecting a cheap plant is FALSE, and it corrects ledger row
A3.** The fork predicted the demo would avoid `relief_blowdown`'s 920
game-fidelity sweeps because a controlled vent conducts, so its node is not a dead
end. The vent conducts 0.4987 kg/s and the first draft still took **741 sweeps**.
With only the vent line's geometry changed: 2 m × 0.10 m → 741, 5 m × 0.06 m → 34,
the shipped 10 m × 0.05 m → 13. **Dead-endedness is not the mechanism; the
branch's conductance against the vessel's capacitance is.** The shipped geometry
was chosen on gas velocity (20.8 m/s against the placeholder's oversized 5.2), not
to fix this — the sweep count is the consequence, recorded.

**A byte-identity baseline has NO power over the file the slice adds, and that is
how the sharpest mutation escaped.** Giving the new `ControlledValue` variant the
same serde tag as the old one moves **zero** corpus rows on both fidelities and
passes the entire test suite, because the only plant whose bytes change is the one
that is new in the same slice and has no baseline row. The demo then reports
`{"variable":"level","pa":2000000.0}`. Tagging the *existing* variant is caught,
and that is the edit §12's prose describes while its mutation table lists the
other. A wire-form gate now closes it, asserted on the serialized bytes because a
Rust match on `ControlledValue::Pressure { .. }` passes under any tag.

**An identity carried across from another node kind is a hypothesis about that
kind's state vector.** M8.5 measured a tank's pressure and mass "one Euler step
apart"; gate 2 carried that across and FAILED. On a vessel the exact identity is
on the MASS, because `C = V·M̄/(R·T)` is itself a function of a state that moved —
the receiver heats as it fills and the pressures miss by 653.6 Pa, 4.078e-4
relative, **which is exactly `ΔT/T`**. Dividing the temperature out restores it to
1.2e-7. A tank's capacitance analogue is geometry; a vessel's is a state.

**The false sentence survived in a third file, and the write-up was citing its
absence from the diff as a virtue.** `traits.rs`'s `Controller` doc still said the
two arguments "are the same type by construction, so the difference
`ControlledValue::error` takes is always dimensionally honest" — the expired claim
again, on the page the next implementer reads. Corrected after the fact. The
gdext binding was also built and linted behind its feature (`--features godot
--target-dir target/godot`), because a new enum variant is exactly what breaks a
feature-gated exhaustive match and the workspace lint sees none of that crate;
both are clean.

**One of the five specified mutations is not expressible**, and the demo's
counterfactual came out differently from M8.4's. "`measure` reads the solved
pressure" cannot be written — `last_solution` is private to `Engine` and `measure`
takes `&self` on the graph — so gate 1's `NaN` half defends a fault the module
boundary already prevents (fourth time in this project a specified gate had no
power over its own subject). And a parked pressure loop does not run away the way
a parked level loop did: a vent's flow rises with the vessel's own pressure, a far
stiffer feedback than `ρgh`, so it **settles at the wrong number** — 25.197 bar
against the loop's 20.000. The demo's gain bound is also two-sided, unlike M8.4's:
the vent settles interior at 0.558224, so `0.5582` reaches 0 on a one-bar step up
and `0.4418` reaches 1 on a step down. **From here, "runs byte-identical" is
unchanged — all fourteen pre-M10 plants are identical on both fidelities.**

**M9 is CLOSED (2026-09-06), and its scope was solver robustness.** It opened the
way M8 did — with a defect the previous milestone reached and deliberately did not
fix — and slices were scoped one at a time, because what the next one should be
depended on what the last one measured. Six boxes landed: three about the
hydraulic solvers (M9.0, M9.1, M9.2), one that turned the milestone's own
hand-measurement into the `corpus` command (M9.3), and two that spent it (M9.3a,
M9.3b). **Nothing in `docs/DEFERRED.md` was past its trigger** when M9 closed. **That is no longer true: B3 (two phases in the state vector) went past on 2026-09-06** — see its row and the ledger's own summary.

Three things carry forward past the milestone. **The scope moved three times** —
which step to take (M9.0/M9.1), when the solver may stop (M9.2), what an iteration
costs and how many there are (M9.3) — and only the first was foreseeable.
**Every box found its predecessor's write-up wrong** (M8's mechanism false in both
clauses, M9.1's first draft fitted to one divergence, M9.3a's table wrong in every
cell, M9.3b's deferral naming the wrong half), so a write-up composed from memory
or from a plausible mechanism is a hypothesis with the formatting of a result —
run the mutation the sentence implies. And **the next milestone is chosen from a
table**: `refinery corpus` plus `docs/DEFERRED.md`, every open hurdle with the
argument that deferred it, its un-defer trigger, and the measured distance.

**M9.3a landed 2026-09-04** — the bubble-point root finder. The design note is
DESIGN §5, "How fast is fast enough" and "What M9.3a changed". Five things to
know.

**The trigger A1 was sitting past is now written, and it is a FRAME budget, not
a corpus total.** The Godot binding leaves ticking to the scene, which calls
`tick()` from `_physics_process` — 16.7 ms at 60 Hz — and a plant may carry
several columns, so one cascade column gets ~2.5 ms per tick. Per *column*,
because the corpus total is an artefact of which files ship. And read as a ratio
**inside one session**: this machine drifted 1.7× slower in a day, so the probe's
own 12.4 s is not comparable to a number measured later. Now 1.10 ms per tick.

**The probe's own proposed remedy would have broken the solve, and the reason
generalises.** It said "a bisection that stops at the tolerance the caller can
see"; DEFERRED A2 said 22 steps. But the cascade's outer convergence test
*differences two bubble-point outputs*, so the root finder's resolution is a
**noise floor on the test that grades it** — it must stay far below that
tolerance, not meet it. Resolution stayed at the float spacing; speed came from
the method (regula falsi, Illinois weighting, Brent's two-step safeguard, on
`ln Σ K·x`). 60 fixed steps → **15 evaluations**, against bisection's 55.

**The logarithm is the whole speed-up and the safeguard is not — the reverse of
what the first write-up claimed.** Swept independently: with the transform every
safeguard variant costs 15–16, without it none costs less than 38. That first
table was written from memory and **every cell was wrong**, caught by running the
double-revert mutation (predicted 28, measured 38). The safeguard buys the
worst-case bound behind `BUBBLE_POINT_MAX_EVALUATIONS`; the gate deliberately
does not defend it, because a bound tight enough to fire on 16 would be fitted to
one composition. Also: the secant's bracket guard is a **negated conjunction** so
a NaN falls to bisection — the "obvious" rewrite reads identically and poisons
the bracket.

**The expected ~7× was 2.60×, fully attributed rather than shrugged at.** Region
timers in both versions: bubble points 14 288 → 4 686 ms, i.e. **3.05×** not the
4× that 60 → 15 predicts, because each evaluation now costs ~31% more (an `ln`
plus secant arithmetic against a bare midpoint); the rest is the unchanged 14%
floor of the K-profile and Thomas sweeps. `flash.rs`'s bisection is **cleared by
measurement** — once per solve, not once per outer iteration.

**The number that did NOT move scopes M9.3b: 200 000 outer iterations over 6 000
ticks, 33.3 per tick, identical before and after.** This slice made each
iteration cheaper; the warm start makes them fewer, and multiplies all three
regions rather than one. Bubble points are still 77% of the loop.

Two measurement habits from this slice. Wall time was taken **A/B/A/B in one
session with an unrelated plant as a control**, because the machine's drift
exceeds the effect on any single pair. And the movement bound was taken over
**all 600 snapshots, not the final one** — the worst deviation is a transient at
tick 270 (1.31e-14) and the settled value is 3.94e-15, so the endpoint alone
understates it 3.3×. Exactly one of fourteen plants moves, on both fidelities.
**From here, "runs byte-identical" means post-M9.3a identical for
`crude_column_cascade` under both `newton` and `simple`.**

**M9.0 and M9.1 are about the same step**, and reading M9.1 without M9.0 will not
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
is DESIGN §11's M9.1 half. Nine things to know.

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

**the closed form does not reach as far as the constants it justifies.** §11's form
says a half step lands within `eps_dp` of the root from any drop, so
`MAX_HALVINGS: 8 → 1` was predicted inert. It DIVERGES M8.0's anchoring plant at
20 000 sweeps. **The first draft of that finding fitted a mechanism to that one
divergence — which is what M8 did — so it was bisected instead:** `2` passes both
the test and the whole workspace, `3` changes nothing. One node needs `t = ¼`, and
**why it is outside the form is NOT measured** (the dead-leg reading — F6 leaves
one live edge so the mirror is exact — is a candidate recorded as a candidate).
So `8` is six halvings of margin and is *not* load-bearing; it is inherited from
`newton_flow`, unjustified there too. Cutting it to `2` would fit a constant to
today's fourteen plants. Also from the pass: a trial evaluation that is not the
step's own function **freezes** the residual (identical to the last digit for
5 000 sweeps) rather than slowing it, caught by two M5-era cross-fidelity tests
and by neither new gate. The reject-all branch is **uncaught**, and the bisection
bounds it — no node needs a step below `¼`, so halvings 3–8 are never the accepted
one, and its 3 281 sites all sit at `|imbalance| ≤ 3.4e-13 kg/s`.

**Lowering `omega` is a CORRECTNESS result, not the cost result the fork argues.**
At `ω = 0.5` the shut-in fixture returns `Ok` with `3.77e-6` kg/s through a shut
branch — inside the solver's own `tol_abs + tol_rel·throughput` and outside the
gate's `1e-6`. A wrong endpoint reported as converged, on the plant the slice
exists for. The 3.5× sweep table is the weaker half of the case for `ω = 1.0`.

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

**M9.2 landed 2026-08-31** — not the step this time but the STOPPING RULE. The
design note is DESIGN §11's M9.2 section. Six things to know.

**DESIGN §3 has specified "relative mass-imbalance per node" since M1 and the code
never did it.** Both fidelities compared the worst node imbalance against
`tol_abs + tol_rel × the largest flow anywhere in the plant`, so a spur carrying
10 g/s was graded against a 10 kg/s trunk. The fix is one shared function,
`network::grade_nodes`, whose scale is `max |ṁ|` over that node's OWN incident
active edges. Both solvers now stop on one rule structurally, not by convention.
**This framing — code catching up with a written spec — is what licensed changing
a settled decision** instead of re-litigating a constant.

**This is the mechanism behind M9.1's `ω = 0.5` finding.** M9.1 recorded `Ok`
returned with `3.77e-6` kg/s through a shut branch, inside the solver's own
tolerance. The damping was the path; the stopping rule is why the endpoint was
accepted.

**The shipped corpus is not the reachability argument — the fixture is.** Over 500
ticks of all fourteen plants only two exceed the per-node bar at all (2.30× and
1.015×, one node each). On the shut-in fixture at shipped settings the game
fidelity's worst is **127×**. Reach for the fixture when the corpus says "barely".

**The shut valve is the EASY case, for the second slice running.** The dead leg
behind the shut valve cannot discriminate — `1.6e-9` kg/s before AND after. What
discriminates is the valve node while the valve still CONDUCTS, and the observable
is an identity, not a bound: a valve holds no volume, so its two edges must carry
the same flow and their difference IS that node's residual. Worst ratio of miss to
the solver's own promise: newton **1.81 → 0.97**, simple **2.58 → 0.39**, both
failing on the old rule. A ratio of 0.97 is a pass, not a near miss — the bar is
the promise rather than a chosen constant, and near-1 says the criterion BINDS
there. Two dead-end assertions in `tests/invariants.rs` were tightened the same
way, and **predicting both from measuring one was wrong**: one is inert (accepted
flow identical before and after), the other fires under the mutation at `5.97e-8`
kg/s against a `1.00e-8` promise. Two assertions of the same shape on two plants
are two measurements.

**The mutation pass has one uncaught edit and it is fork 1's own.** Reverting to
the plant-wide scale is caught twice; grading only the last node is caught by 12
tests across four binaries. But swapping `Σ` for `max` as the local scale — the
alternative fork 1 rejects — passes EVERYTHING, because `Σ = 2·max` at a two-edge
node so the bar doubles and the valve gate's 0.97 becomes 0.48, and the solve
never spends the extra slack. **Fork 1 rests entirely on the inequality
`max_incident ≤ throughput` — the new rule is never looser than the old — and no
test defends it.** Left open deliberately.

**The gate this slice set out to write was a vessel, and measuring killed it.** A
vessel's accumulation term is deliberately OUTSIDE the scale (fork 2), so a vessel
gate would assert against the one quantity the criterion does not grade. Third
time in this project a specified gate had no power over its own subject.

**Cost is one iteration on one plant, and the `Err` fear was measured away.**
Twelve of fourteen scenarios stay byte-identical over 6 000 ticks; the two that
move do so by ≤ `8e-8` relative on any physical quantity. Worst iterations move
only on `tank_level_control`, 3 → 4. Because proptest generates spurs and dead
legs — the population whose bar collapsed to `tol_abs` — the reachability counts
were compared either side and are identical (chains 238/300, gas 202/205 Newton
and 187/205 Simple, psv chains 196/400). **From here, "runs byte-identical" means
post-M9.2 identical for `relief_blowdown` and `tank_level_control`.**

**M9.3b landed 2026-09-04** — the warm start, and M9.3 closes with it. The design
note is DESIGN §5, "The warm start (M9.3b)". Six things to know.

**Fork 5's word "profile" names the wrong half, and that is the finding.** A stage
cascade iterates stage TEMPERATURES and stage LIQUID COMPOSITIONS at once.
`stage_t` is what reads as "the profile" — it is what `with_seed_offset` perturbs
and what the convergence diagnostic named — and seeding it alone is nearly inert:
38.0 → 35.0 outer iterations per solve, 8%, which vanishes into wall-clock noise.
Seeding BOTH gives **1.006**. The outer convergence test is a CONJUNCTION, and the
composition profile was the binding criterion all along. **The obvious reading
would have shipped the 8% and written the warm start up as measured and
disappointing** — it was caught only because the A/B came back inside the noise
band and the iteration count was measured to find out why.

**The type is the measurement.** `CascadeProfile` holds both halves behind ONE
`Option`, so "temperatures without compositions" — the configuration just
falsified — is unrepresentable. Same shape and argument as the two duties.

**It needed no engine state, and that settled the seam fork.** The previous tick's
`NodeStates` is already an argument to the sweep and its separations are already
keyed by node, so the profile rides `Separation` out and `ColumnPass` back in.
`SeparationModel::separate` keeps `&self` and its "a function of `pass` alone"
contract stays LITERALLY true, because the history is an argument rather than
state on the model. `&mut self` and interior mutability both falsify that sentence
and both put per-column state on the single `Box<dyn SeparationModel>` the engine
holds for every column, which would cross-seed two columns on one plant.

**1.006 iterations per solve needed an adversarial gate, not a celebration** —
converging on the first pass is what a right seed looks like AND what a criterion
that stopped binding looks like. Over 6 000 solves: 38 once (tick 1, before a
profile exists), 1 for 5 998 ticks, one zero-flow return. So it binds when the
seed is ABSENT; what no shipped plant reaches is a seed present and WRONG, and
that is the gate — a light feed's converged profile handed to a heavy feed, which
must return the heavy feed's own cold answer. **Two controls are asserted first**
(the profiles must differ by > 1 K somewhere, the distillates by > 1000× the
tolerance), because without them the gate is passed by a solver that ignores its
seed and equally by one that ignores its feed.

**The fixed-point worry was real, measured, and did not happen.** A warm start
moves the answer by the outer tolerance rather than the ULP — ~8 orders more than
M9.3a — and the cascade feeds tanks that integrate. Over 600 snapshots on both
fidelities: worst move on any quantity above 1e-3 is **7.6e-07**, temperatures
1.7e-08, duties 1.5e-08, pressures at the ULP, tank masses **bit-identical** (a
draw's rate is a mass ratio of the feed, so an inventory never depended on the
profile). The decisive number is that the drift is **flat across all ten deciles**,
first equal to last — a tolerance-ball reseat, not accumulation. The full warm
start is also CLOSER to cold than the partial one, because a better seed converges
nearer the true fixed point.

**One mutation is uncaught, deliberately.** Reverting the liquid half of the seed
fails NOTHING: a half-warm start is still correct, only slow, and no test measures
a cascade's iteration count. (Breaking the convergence test is caught by seven
tests; dropping the seed's shape check by exactly its own gate.) A gate for it
would have to assert a cost rather than a correctness, which is how a fitted test
gets written. The defence is the type: undoing the fix means deleting a struct
field, not forgetting a line. **Result: 4 828.9 → 235.7 ms newton (20.5×) and
4 760.9 → 223.8 ms simple (21.3×) per 6 000 ticks, paired in one session; 37.8×
fewer outer iterations, which is the machine-independent number. From here, "runs
byte-identical" means post-M9.3b identical for `crude_column_cascade` under both
fidelities.**

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

**Exactly two of the sixteen files in `scenarios/` declare a `[[controls]]`
table** — `tank_level_control.toml` (M8.4, a level) and
`vessel_pressure_control.toml` (M10.1, a pressure). **The other thirteen were
written before M8 and ARE the regression anchor**; adding a loop to one of them
would move its snapshot, which is why each regulation slice ships a NEW file
rather than wiring one into an existing plant. Every other plant that carries a
loop is an inline test fixture for the same reason.

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
