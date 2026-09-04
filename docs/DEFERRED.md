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
release build, unless a row says otherwise.

## A. Solver numerics — M9's subject

| # | item | argued in | un-defers when | distance, measured |
|---|---|---|---|---|
| A1 | **The stage cascade solves cold on every tick.** `StageCascade::separate` seeds every stage at the feed's bubble point and iterates to convergence with no memory of the previous tick's profile. Fork 5 forbids *holding* a previous profile and explicitly *allows* seeding from one ("a warm start changes the iteration count, not the fixed point"), and M7.3 recorded that it did not build one. | §5, "Complex column", fork 5; `cascade.rs` `with_seed_offset` doc; §5, "How fast is fast enough" | **A cascade column costs more than a sixth of a physics frame.** Written M9.3a. The Godot binding ticks from `_physics_process`, which is 16.7 ms at the default 60 Hz, and a plant may carry several columns — so the budget for one is ~2.5 ms and the trigger is one column exceeding it on this machine's corpus run. Absolute times are only comparable within one session (this machine drifted 1.7× in a day), so the measurement is: run the corpus, divide `crude_column_cascade`'s tick wall time by the ticks, compare. | **Past the trigger, and the distance halved but did not close.** M9.3a cut the cascade 2.60× — 2.87 → **1.10 ms per tick** measured A/B/A/B against a control plant — by making each outer iteration cheaper. The iteration count it does NOT touch: **200 000 outer iterations over 6 000 ticks, 33.3 per tick, identical before and after**. Bubble points are still **77%** of the loop (was 90.7%). That count is what a warm start attacks, and it multiplies all three regions rather than one. |
| ~~A2~~ | ~~**The bubble-point bisection runs 60 steps whatever the tolerance asks.**~~ **CLOSED by M9.3a**, and its stated remedy was wrong in a way worth keeping. The row proposed cutting the step count to the ~22 that meets the outer convergence test. That would have broken the solve: the outer test *differences two bubble-point outputs* (`|T − T'|/T'` against a tolerance as tight as `1e-12`), so the root finder's resolution is a **noise floor on the test that grades it** and must stay far below the tolerance, not meet it. Speed had to come from the method, not the tolerance — and the row's other estimate, "a bracketed Newton or secant would take ~8", measured **15**. | `cascade.rs`, `bubble_point` | — | Resolution kept at the float spacing; evaluations 60 → 15; bubble points 14 288 → 4 686 ms per 6 000 ticks. |
| A3 | `relief_blowdown` on `SimpleFlowSolver` takes 900-odd sweeps because a normally-shut PSV leaves its valve node a dead end and the receiver's Gauss–Seidel diagonal is dominated by a fat branch carrying nothing — a preconditioning problem, not an overshoot. | §11, M9.1, "Deferred" | A plant of that shape reaches the sweep cap (5 000). | **920** of 5 000; the gate in `relief_valve_reference.rs` asserts < 2 500. 5.4× under the trigger and unchanged since M9.1. |
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
| B1 | **No cavitation floor.** An over-driven pump can produce a genuine solution with sub-zero absolute suction pressure; frontends are told to read that as "cavitating", not as an error. | §3, "No cavitation / vapor-pressure floor in M1" | **No trigger written.** "A later milestone." | Measured: the lowest node pressure in any plant's tick-6 000 snapshot is 100 000 Pa (atmospheric), so no shipped plant is anywhere near a vapour pressure. A frontend that wants to *show* cavitation, or a pump-curve scenario with a real NPSH margin, is the obvious trigger and would need writing down first. |
| B2 | Fixed friction factor per pipe; Colebrook/Haaland "later". | §3; `elements.rs` `pipe_resistance` | **No trigger written.** | No shipped pipe declares `friction`; all fourteen plants run the schema default of 0.02 on every pipe. A plant whose Reynolds number crosses a regime inside a run is where a fixed `f` becomes distinguishable from a correlation. |
| B3 | Two phases in one `Stream`: a flashing feed line, a partial condenser, a vapour side draw. Phase is absent from the state vector, so this changes `Stream`, `Composition` and every reader — a milestone, not a slice. M7 refuses all three at load. | §3a, "What is deferred"; §5 M7 fork 0 | A plant needs one of the three. | Three load-time refusals, each naming the bullet. No shipped plant asks. |
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
| E1 | **Pressure, temperature and flow control.** Fork 1's shape is variable-agnostic; each needs a measurement path and an actuator that exists. | §10, "Deferred" | Per variable; **pressure is the near one** — a `Vessel`'s state IS a pressure and a relief-free vessel has no way to be held anywhere. | `relief_blowdown` already has the vessel and the valve; it lacks only the measurement variant. |
| E2 | Cascaded loops. | same | An inner loop fast enough to be worth separating; needs an execution-order rule stronger than declaration order. | One loop shipped. |
| E3 | Derivative action. | same | A loop oscillatory enough for damping to be distinguishable from a lower gain. | The shipped loop's output never leaves `[0.194, 0.384]`. |
| E4 | Split-range, override, feedforward — "more than one writer of one actuator", refused at load. | same | Together, with a defined arbitration. | — |
| E5 | Actuator dynamics and deadband. | same | A loop's performance depends on them; at 1:600 tick-to-integral-time it does not. | — |
| E6 | Interlocks and trips — a discrete layer. | same | A safety case needing a plant to shut *itself* down. | — |

## F. Frontends — §8

| # | item | argued in | un-defers when |
|---|---|---|---|
| F1 | Out-of-range ids in a `Command` are validated in `bridge`, not `core`; `core` still panics on one. | §8, "The translation layer" | A second untrusted-input frontend, or an in-repo caller that can construct an out-of-range id. `core_panics_on_an_out_of_range_id` fires if `core` changes underneath. |

## Reading the ledger

- **A1 is the only item past its trigger, and it now HAS one.** M9.3a wrote it
  (a cascade column must fit inside a sixth of a Godot physics frame) and then
  closed A2 under it. A1 is still not a defect — no scenario `Err`s and the
  cascade converges on every tick — but it is still the corpus's wall-time
  budget, and fork 5's own sentence is the licence.
- **A closed row's stated remedy is worth keeping when it was wrong.** A2's was:
  it proposed loosening a tolerance that turns out to be the noise floor of the
  test grading it. Struck rather than deleted, because the next person to see a
  suspiciously tight constant will reach for exactly that.
- **Everything with a numeric trigger is well inside it.** A3 at 5.4× under the
  cap is the nearest, and it has not moved since M9.1.
- **Two deferrals have no trigger at all** (B1, B2); A1's was written by M9.3a.
  Writing one is the
  first job of whichever slice takes them, and "when someone wants it" does not
  count — the note has to say what plant or frontend would tell a right answer
  from a wrong one.
