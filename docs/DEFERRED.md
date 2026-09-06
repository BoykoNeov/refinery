# The open-hurdle ledger

Every design note in `DESIGN.md` ends with a list headed "deferred, with what
un-defers each". There are nine of those lists, spread over five sections, and
nothing collects them — so scoping a slice has meant re-reading all nine to find
out which trigger the corpus is nearest to. This file is the collection. It adds
one column the notes cannot carry: **the measured distance from each trigger, as
of a date**, taken with `refinery corpus` where a number exists.

Three rules for keeping it honest:

- **An entry here points at the paragraph that argued it.** The argument stays in
  `DESIGN.md`; this file never re-argues a deferral, it only says where the plant
  stands relative to it. If the two disagree, the note is right and this file is
  stale.
- **A deferral without a written trigger is a gap, and is marked as one** rather
  than given a trigger invented here. Three of them are below.
- **When a slice takes an item, strike it with the slice's name** rather than
  deleting the row. The record of what was deferred and for how long is part of
  what the ledger is for.

Distances measured 2026-09-02 on the fourteen shipped scenarios, 6 000 ticks,
release build, unless a row says otherwise. **`scenarios/` holds fifteen files as
of M10.1**; rows whose distance predates it say so, and the two that M10.1
re-measured (A3, E1) carry the new number. **B1 and B3 were re-measured
2026-09-06** at the M10 close-out, on all fifteen and both fidelities, and both
of their old distances were wrong — B1's because it compared a pressure against
zero rather than against a vapour pressure, B3's because the state it says
nothing asks for arises during a run rather than at load, where its three
refusals look.

## A. Solver numerics — M9's subject

| # | item | argued in | un-defers when | distance, measured |
|---|---|---|---|---|
| ~~A1~~ | ~~**The stage cascade solves cold on every tick.**~~ **CLOSED by M9.3b**, and the deferral named the wrong half. Fork 5 says "seed from the previous tick's profile", and "profile" reads as the stage TEMPERATURES — which is what `with_seed_offset` perturbs. Seeding those alone is 38.0 → 35.0 outer iterations per solve, 8%, inside wall-clock noise. The cascade iterates the stage LIQUID COMPOSITIONS too, its convergence test is a conjunction over both, and the composition profile was the binding criterion; seeding both gives **1.006** iterations per solve. | §5, "The warm start (M9.3b)" | — | 4 828.9 → **235.7 ms** per 6 000 ticks (20.5×, paired in one session); 86% → **26%** of the corpus's wall time; **0.039 ms per tick** against the 2.5 ms frame budget M9.3a wrote, 64× inside it. |
| ~~A2~~ | ~~**The bubble-point bisection runs 60 steps whatever the tolerance asks.**~~ **CLOSED by M9.3a**, and its stated remedy was wrong in a way worth keeping. The row proposed cutting the step count to the ~22 that meets the outer convergence test. That would have broken the solve: the outer test *differences two bubble-point outputs* (`|T − T'|/T'` against a tolerance as tight as `1e-12`), so the root finder's resolution is a **noise floor on the test that grades it** and must stay far below the tolerance, not meet it. Speed had to come from the method, not the tolerance — and the row's other estimate, "a bracketed Newton or secant would take ~8", measured **15**. | `cascade.rs`, `bubble_point` | — | Resolution kept at the float spacing; evaluations 60 → 15; bubble points 14 288 → 4 686 ms per 6 000 ticks. |
| A13 | `CascadeProfile::liquid` is `Vec<Vec<f64>>` in `core`. The newtype that owns the normalisation invariant, `solvers::molar::MoleFractions`, lives in `solvers`, and `core` must not depend on it — so a warm-start seed crosses the boundary as bare `f64`. Argued rather than silent: rule 4 is about quantities whose UNIT a reader could get wrong, a mole fraction has none, and `core` never interprets the field (it stores what the model returned and hands it back). | `traits.rs`, `CascadeProfile`; rule 4 | A second consumer in `core` that must READ the field, or any other seam needing mole fractions across the boundary — at which point `MoleFractions` moves down into `core` and both use it. | One field, one producer, one consumer, both in `solvers`. Shape is checked at the seed site, so a malformed profile is ignored rather than trusted. |
| A3 | `relief_blowdown` on `SimpleFlowSolver` takes 900-odd sweeps because the receiver's Gauss–Seidel diagonal is dominated by a fat branch — a preconditioning problem, not an overshoot. **The row used to say the branch had to be a DEAD END (a normally-shut PSV) and M10.1 measured that it does not.** Its demo plant has no dead end anywhere — the controlled vent conducts 0.4987 kg/s at steady state — and with a 2 m × 0.10 m vent line it took **741 sweeps**; narrowing the same conducting branch to 10 m × 0.05 m took it to 13. The flow the branch carries is incidental; its conductance against the vessel's capacitance is the whole of it. Shut-ness is one way to get there, not the mechanism. | §11, M9.1, "Deferred"; corrected in §12, M10.1 | A plant of that shape reaches the sweep cap (5 000). | **920** of 5 000; the gate in `relief_valve_reference.rs` asserts < 2 500. 5.4× under the trigger and unchanged since M9.1. `vessel_pressure_control`, the same vessel with a conducting vent, sits at **13**. |
| A4 | `tol_rel` on each fidelity (`1e-8` Newton, `1e-6` Simple) was chosen against a plant-wide throughput and now multiplies a node's own traffic; neither has been re-swept against the new meaning. | §11, M9.2, "Deferred" | A plant needs a tolerance argued from its own numbers rather than inherited. | No shipped plant does. Worst Newton iterations per tick across the corpus: 10 of 50 (`relief_blowdown`); worst Simple sweeps other than A3: 16 (`fcc_plant`). |
| A5 | A node with exactly one live edge can only ever satisfy `tol_abs`; correct for a dead end, wrong for a terminal consumer whose single edge carries real flow. | §11, M9.2, "Deferred" | A scenario adds a terminal consumer. | None shipped, per M9.2's note; the single-edge nodes it measured were all dead legs. |
| A6 | Fork 1 of M9.2 rests on `max_incident ≤ throughput` and no test defends it: swapping `Σ` for `max` as the local scale passes everything. Left open deliberately — closing it needs a fixture built to stop exactly on a tolerance, which the project has called a fitted test. | §11, M9.2, "The mutation pass" | Deliberately none. | Not a distance; a recorded gap. |
| A7 | One node on M8.0's anchoring plant needs `t = ¼` and the closed form says `t = ½` reaches the root from any drop. `MAX_HALVINGS: 8` is six halvings of margin; `2` passes today's fourteen plants and would be a fitted constant. **Why** the node is outside the form is not measured (the two-live-edge reading is a candidate). | §11, M9.1, "The mutation pass" | A plant needs a step below `¼`, or the mechanism is measured. | No node in the corpus takes a halving deeper than 2; the reject-all branch fires 3 281 times, all at `\|imbalance\| ≤ 3.4e-13` kg/s. |
| A8 | Below `2·eps_dp·max_iter` Newton still walks toward a shut branch's root in 2 Pa steps: a 90 Pa drop spends 45 iterations doing what one half step would. | §11, M9.0, "Deferred" | Same code fork 4 would replace; un-defers with fork 4. | Worst pass in the corpus is 10 iterations; the crawl is not what any shipped plant is spending them on. |
| A9 | **Fork 4, the sign-reversal trust region.** Scale-free and exactly right on the mirror step, deferred because a legitimate solve does reverse a branch's drop. M9.1 built it node-wise on the other fidelity and it was **inert** on the shut-in — the mirror step does shrink the imbalance, by `2ε`. | §11, M9.0 fork 4; §11, M9.1 fork 5 | A Newton stall that survives fork 1 (a correctly rejected full step whose half step is still not enough). | None found. Evidence from M9.1 is against the rule, not for it. |
| A10 | The Simple solver's cold seed is the mean of the pinned pressures, so free nodes start ~200 kPa from their roots on tick 0. | §11, M9.1 fork 4 | Only alongside a reason other than the shut-in, which it cannot reach. | Tick-0 cost only; no shipped plant Errs on it. |
| A11 | `NewtonFlowSolver::max_iter` and `SimpleFlowSolver::omega` are `pub`, and a caller setting `max_iter: 10` or `omega: 0.5` reopens a window or accepts a wrong endpoint. No scenario file can set either. | §11, M9.0 "Deferred"; §11, M9.1 "Deferred" | The solver's numerics become scenario config, at which point both become load-time refusals. | Nothing in `crates/scenarios/src` names either field. Asserted against `Default` by unit tests. |
| A12 | Dense LU in `NewtonFlowSolver`; "sparse later if needed". | §3 | A plant big enough for the dense solve to be the cost. | The largest shipped plant has 7 nodes; the corpus's cost is A1, not linear algebra. |

## B. Hydraulics and gas — §3 and §3a

| # | item | argued in | un-defers when | distance, measured |
|---|---|---|---|---|
| B1 | **No cavitation floor — and the noun is wrong.** An over-driven pump can produce a genuine solution with a suction pressure below the fluid's vapour pressure, and the engine has no signal for it. **The row has now been re-premised twice.** The M10 close-out corrected "frontends read sub-zero absolute pressure as cavitating" — zero is not the threshold, a liquid boils below its *vapour* pressure, so §3's marker fires **late by exactly that vapour pressure** (0.58 m of suction lift on cold water; **nine bar** on light naphtha at 445.75 K). **M11.0 corrected the noun**: DESIGN §13 fork 1 rejects the clamp this row and §3 both promise, because the vapour a clamp would account for is mass in a phase the state vector does not have — which is B3. What M11 builds is a **criterion and a signal**, and it stops where B3 begins. | §3, corrected 2026-09-06; **DESIGN §13 (M11.0)** | **Trigger written 2026-09-06, node list corrected 2026-09-06.** Either of: **(a)** a *solved* pressure at a node in the hydraulic path falling below that node's own bubble pressure; or **(b)** a frontend needing to *display* cavitation. **The node list is six kinds, not four**: pump, valve, relief valve, junction, exchanger **and furnace/cooler** — the first draft omitted the last two, and a fired heater's outlet is exactly where a refiner expects a liquid to boil. Holdups stay excluded (that is B3), as do declared boundaries, columns (at their bubble point by definition) and reactors (an imposed outlet temperature). **Clause (b) fired as a DECISION, not a measurement, and M11 is that decision.** | **Re-measured 2026-09-06 with the ENGINE's instrument, and the previous distance could not be produced by a run.** 1.865× was `heat_recovery`'s `hx_hot` — and that plant declares `thermo = "constant"`, whose `k_value` is an `Err`, so the number came from the close-out's standalone script rather than from the model. **Fourteen of fifteen plants are in that position**, so under the trigger's *original* four-kind list the engine-computable set across the whole corpus is **EMPTY**. The one plant that can answer is `crude_column_cascade`; its only flow-path node is a **furnace** (`preheater`), at **1.899×** over 6 000 ticks (spread 1.899–1.921, so static like `hx_hot` was). Its column sits at 0.988–1.000× (excluded by definition) and its `naphtha_tank` at 0.940× (B3's). Reachability is unchanged and re-measured with the engine's instrument — raising `tank_pump_valve`'s suction line crosses at **17.44 m** and reaches **−9 522.94 Pa** at 19 m with the solver converging — but it takes **two** edits, not the one this row used to claim: the elevation *and* the fidelity line. The second moves no number (175 197.988 Pa either way). |
| B2 | Fixed friction factor per pipe; Colebrook/Haaland "later". | §3; `elements.rs` `pipe_resistance` | **No trigger written.** | No shipped pipe declares `friction`; all fourteen plants run the schema default of 0.02 on every pipe. A plant whose Reynolds number crosses a regime inside a run is where a fixed `f` becomes distinguishable from a correlation. |
| B3 | Two phases in one `Stream`: a flashing feed line, a partial condenser, a vapour side draw. Phase is absent from the state vector, so this changes `Stream`, `Composition` and every reader — a milestone, not a slice. M7 refuses all three at load. **There is a fourth path in that no refusal names**, found 2026-09-06 by B1's sweep: a two-phase *holdup*, arriving not from declared config but from a **draw temperature**. A cascade column draws at real tray temperatures (M7.4a), the product tank has no cooler, and the tank then stores a liquid the model's own vapour-pressure correlation says is boiling. A refusal cannot catch it, because nothing in the file is wrong at load — the state emerges during the run. | §3a, "What is deferred"; §5 M7 fork 0; the fourth path is new | A plant needs one of the three — **or holds an inventory below its own bubble point**, which two already do. | **"No shipped plant asks" was false and is corrected.** Measured over 6 000 ticks: `crude_column`'s `naphtha_tank` sits at **0.30×** its own bubble pressure, **sustained and worsening** (P/P_bub 0.464 at tick 1 000 → 0.312 at 3 000 → 0.304 at 6 000, as the draw heats the tank 395 K → 432 K); `crude_column_cascade`'s same tank hovers on the line, worst **0.940×**. Both report as liquid with no signal. **Which fix is right is open and this row does not decide it**: a product cooler in the two files is a scenario defect and cheap; phase in the state vector is the model defect and is this row. The other three load-time refusals still have no plant asking. |
| B4 | Real-gas `Z`, gas `cp(T)`. | §3a | A case near the critical point. | Ideal gas throughout; additive when needed. |
| B5 | Pump efficiency heating (~0.03 K). | §3a; M2 close | A scenario with real pump curves and efficiencies, where a wrong `η` is distinguishable from a right one. | `η` has one possible value in the repo. |
| B6 | PSV hysteresis and chatter — needs element state. | §3a fork 5 | A relief case where reseat pressure matters. | The relief valve is a characteristic, not a controller (fork 5's verdict). |
| B7 | Air ingress through a leak, and therefore combustion. Fire stays a heat source on a node. | §3b, "What M6 does not attempt" | A slate carrying air and a reason to burn it. | `Atmosphere` back-feed is an `Err` (option 3). |
| B8 | Acoustic / pressure-wave dynamics. | §3, §3a | Never; out of scope by design. | — |

## C. Columns — §5, M7

| # | item | argued in | un-defers when |
|---|---|---|---|
| C1 | Tray hydraulics: pressure drop per tray, weeping, flooding. | §5, "Deferred from M7" | A stage needs a vapour *density* — fork 1's boundary. |
| C2 | Column holdup and tray dynamics; the cascade is quasi-steady per tick. | same | A startup or composition-front transient needs watching rather than stepping over. |
| C3 | Non-ideal K (activity coefficients). | same | A slate carrying a polar component. |
| C4 | Murphree efficiency per tray. | same | A case can tell 20 real trays from 14 ideal ones. |
| C5 | Feed quality `q`; the cascade refuses a feed off its bubble point outside `ε·λ̄/c̄p`. | same; M7.4b | A plant preheats its feed past the bubble point on purpose. |

## D. Reactors — §5, M4

| # | item | argued in | un-defers when |
|---|---|---|---|
| D1 | Adiabatic / emergent outlet temperature (the coupled ODE, fork 3). | §5, "Simple reactor", "Deferred" | A case where the reactor's own heat release sets its temperature. |
| D2 | Flow-dependent residence time `τ = V·ρ/ṁ`. | same | A reactor whose throughput moves during a run. |
| D3 | A 1-in-2-out reactor separating coke at the outlet (reintroduces the draw-flow machinery). | same | Coke needs to leave by a second outlet rather than ride to a downstream column. |
| D4 | The catalyst regenerator loop that physically supplies the endothermic duty. | same | The duty must be sourced, not reported. |
| D5 | Coke-on-catalyst deactivation (Voorhies form), feed-quality dependence of the constants, a second lump slate. | §5, "FCC 4-lump", "Deferred" | A catalyst inventory exists (D4). |

## E. Regulation — §10

| # | item | argued in | un-defers when | distance |
|---|---|---|---|---|
| ~~E1~~ | ~~**Pressure control.**~~ **CLOSED by M10.1**, 2026-09-06; the note is §12. Three things this row got wrong, all measured. It said a `Vessel`'s pressure needs a measurement path, and a vessel's pressure is **stored** (`m/C`, `m` on the graph, exactly the declared figure from load) so the path already existed and `measure`'s signature never changed. It said `relief_blowdown` lacks only the variant, when that plant has **no ordinary valve at all** and the PSV is refused as an actuator by name. And the schema predicted the key would be `setpoint_pa`, where every pressure the format declares is in bar — the keys are `setpoint_bar` and `gain_per_bar`, with the ×1e5 and ÷1e5 written at one site because converting one without the other is invisible to every convergence and conservation test in the workspace. | §12 | — | Demo is `scenarios/vessel_pressure_control.toml`, a new file: adding an actuator to `relief_blowdown` would have moved a regression anchor. All fourteen pre-M10 plants byte-identical, both fidelities. |
| E1b | **Temperature and flow control.** What is left of E1 once pressure moves out. Each needs its own measurement path argued. | §10, "Deferred"; §12 | Per variable. A temperature really is a `NodeStates` quantity and absent before the first tick; a flow lives on an EDGE, which nothing in `measure`'s signature can name. | Neither has a plant asking. `measure(&self, slate, node, variable)` takes a node, so flow control is a signature change and not a new match arm. |
| E7 | **Reverse action** — a loop whose actuator is an INLET of the measured holdup. The sign convention (`error = measurement − setpoint`, `u = clamp(K·e + b, 0, 1)`) forces the actuator to be an OUTLET; M8.4 recorded this as "a level loop must actuate a drain" and §12 generalised it to "an outlet of the measured holdup" — a drain for a tank, a vent for a vessel. A negative gain is refused at load in both controllers, deliberately: reverse action needs its own declaration, not a sign. | §12 fork 4; `control.rs` on both controllers | A plant whose only actuator is upstream of what it measures. | Open since M8.2 and never recorded until M10.0. Both shipped loops actuate an outlet. **Measured as a mutation in M10.1**: rewiring the demo's vent into the make-up line is caught by four demo gates — the plant runs, converges and conserves mass, and simply never reaches its setpoint. |
| E2 | Cascaded loops. | same | An inner loop fast enough to be worth separating; needs an execution-order rule stronger than declaration order. | One loop shipped. |
| E3 | Derivative action. | same | A loop oscillatory enough for damping to be distinguishable from a lower gain. | Neither shipped loop oscillates: the level loop's output never leaves `[0.194, 0.384]` and the pressure loop's never leaves `[0.3000, 0.5882]`. |
| E4 | Split-range, override, feedforward — "more than one writer of one actuator", refused at load. | same | Together, with a defined arbitration. | — |
| E5 | Actuator dynamics and deadband. | same | A loop's performance depends on them; at 1:600 tick-to-integral-time it does not. | — |
| E6 | Interlocks and trips — a discrete layer. | same | A safety case needing a plant to shut *itself* down. | — |

## F. Frontends — §8

| # | item | argued in | un-defers when |
|---|---|---|---|
| F1 | Out-of-range ids in a `Command` are validated in `bridge`, not `core`; `core` still panics on one. | §8, "The translation layer" | A second untrusted-input frontend, or an in-repo caller that can construct an out-of-range id. `core_panics_on_an_out_of_range_id` fires if `core` changes underneath. |

## Reading the ledger

- **A row's stated MECHANISM can be wrong even when its number is right.** A3's
  920 sweeps were never in doubt; its explanation was, and M10.1 falsified it by
  building a plant of the shape the row says is required and finding the cost
  without it. A distance column gets re-measured every milestone and a mechanism
  column gets read and believed, which is the wrong way round.
- **Nothing is past its trigger.** A1 was the only one, and M9.3 closed it in two
  commits: M9.3a wrote the trigger it had been sitting past (a cascade column must
  fit inside a sixth of a Godot physics frame) and closed A2 under it; M9.3b took
  fork 5's licence and closed A1 itself, 64× inside the new trigger.
- **A deferral can name the wrong half of its own subject.** A1 said the cascade
  "solves cold", and fork 5's remedy was to seed "the previous tick's profile".
  Both are true and neither says WHICH profile — and the one they naturally read
  as is worth 8%, while the one they do not name is worth 97%. Read a deferral for
  what it leaves unsaid, not only for what it licenses.
- **A closed row's stated remedy is worth keeping when it was wrong.** A2's was:
  it proposed loosening a tolerance that turns out to be the noise floor of the
  test grading it. Struck rather than deleted, because the next person to see a
  suspiciously tight constant will reach for exactly that.
- **Everything with a numeric trigger is well inside it.** A3 at 5.4× under the
  cap is the nearest, and it has not moved since M9.1. **B1 is second at 1.87×**,
  which is a distance the row did not have until 2026-09-06 and which is far
  nearer than the number it used to carry.
- **One deferral has no trigger at all** (B2); A1's was written by M9.3a and
  **B1's at the M10 close-out**. Writing one is the first job of whichever slice
  takes them, and "when someone wants it" does not count — the note has to say
  what plant or frontend would tell a right answer from a wrong one.
- **A distance is a property of the ENGINE, not of the plant.** B1's 1.865× was
  produced by the M10 close-out's measurement script, which reimplemented Raoult
  over Trouton standalone. The plant it names declares `thermo = "constant"`,
  whose `k_value` is an `Err`, so **no engine configuration of that plant can
  produce the number**. A distance that cannot come out of a run is a prediction
  about a configuration nobody ships. This is the second correction to the same
  row and it is a different kind from the first: that one had the wrong
  *quantity*, this one has an *instrument the engine does not have*. Both tells
  were in the row's own text — the first named the quantity it was not measuring,
  the second named a plant whose fidelity line is three lines from its top.
- **A trigger's node list is a claim, and B1's was falsified by the first slice
  that read it.** It named four kinds; the engine-computable set of those four
  across all fifteen plants is empty, and the one node in the corpus that can be
  evaluated is a **furnace**, which the list omits. Corrected to six kinds. A
  four-name list inherited without enumerating the type would have been the fifth
  dead gate in this project's record (M7.4b, M7.4c, M9.2, M10.1).
- **A distance is only as good as the quantity it is measured against, and B1's
  was measured against the wrong one for five milestones.** The row read node
  pressures against **zero**, reported 120 kPa of clearance, and concluded "no
  shipped plant is anywhere near a vapour pressure" — while never evaluating a
  vapour pressure. Against the model's own (`ThermoModel::k_value`, which existed
  from M7.2) the clearance is 1.87×. The tell was available in the row's own
  sentence: it named the quantity it was not measuring. **When a row's distance
  and its stated concern are in different units, the distance is not about the
  concern.**
- **A trigger has to name what kind of node it is about.** B1's first draft would
  have fired immediately, because two product *tanks* are already below their
  bubble point — but a tank above its bubble point is a two-phase holdup (B3),
  not cavitation, which is a hydraulic-path phenomenon. Scoping the trigger to
  the flow path is what keeps B1 and B3 from claiming each other's evidence, and
  the exclusion had to be written down rather than assumed.
- **Most of a scary-looking sweep was not evidence.** Ten of fifteen plants have
  a node below its bubble pressure. Five declare `phase = "gas"` cuts, where a
  liquid bubble-point test is meaningless by construction; two are columns, which
  are *at* their bubble point by definition; two carry an FCC `gas` lump with
  `tb = −40 °C` declared liquid, which is this table's B3 already. **Three of the
  four categories were exclusions**, and a sweep that reports the headline
  without them would have moved two rows on evidence that does not exist.
