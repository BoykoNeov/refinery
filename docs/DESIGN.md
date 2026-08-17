# Design

## 1. Shape of the system

One deterministic engine crate-family, many frontends.

```
            ┌─────────────────────────────────────────┐
            │                 core                    │
            │  units · Stream · PlantGraph · Engine   │
            │  Snapshot · Command · solver TRAITS     │
            └──────────────▲──────────────────────────┘
                           │ implements
            ┌──────────────┴──────────────┐
            │           solvers           │
            │ SimpleFlowSolver            │
            │ NetworkFlowSolver (Newton)  │
            │ ConstantThermo / CutThermo  │
            │ NoReactions / FccFourLump   │
            └──────────────▲──────────────┘
                           │ selected by
            ┌──────────────┴──────────────┐
            │          scenarios          │  TOML → EngineBuilder
            └───┬───────────────┬─────────┘
                │               │
         ┌──────▼─────┐  ┌──────▼───────┐
         │    cli     │  │  godot-ext   │   (thin shells)
         └────────────┘  └──────────────┘
```

Data flow per tick:

1. Frontend calls `engine.apply(command)*` (valve setpoints, pump on/off,
   damage events).
2. `engine.tick(dt)`:
   a. **Hydraulic solve** (quasi-steady): FlowSolver computes node pressures
      and branch flows for the current network + element states.
   b. **Transport**: move mass/energy/composition along branches for `dt`.
      Energy transport is upwind advection off the solved flow signs (§4a);
      composition transport arrives with the pseudo-component slate in M3.
   c. **Unit dynamics**: each unit integrates its slow states (tank level,
      column temperatures, reactor lump concentrations) with the resolved
      flows as boundary conditions.
   d. **Validation**: NaN/Inf scan, invariant checks in debug builds.
3. Frontend calls `engine.snapshot()` → serde-serializable state.

## 2. Plant graph

- **Nodes** = units: `Source`, `Sink`, `Atmosphere`, `Tank`, `Pump`, `Valve`,
  `Junction`, `Furnace`, `Cooler`, later `HeatExchanger`, `Column`, `Reactor`.
  (Pumps/valves are nodes with one in / one out edge; this keeps *all*
  flow elements uniform for the solver: every edge is a plain pipe.)
- **Edges** = pipes: geometry (L, D, roughness), and the transported
  `Stream` state.
- Stored in `core::graph::PlantGraph` (petgraph `StableDiGraph` wrapped in
  our own type so petgraph never leaks into public APIs).
- IDs: `NodeId(u32)`, `EdgeId(u32)`, stable across ticks, serialized in
  snapshots so frontends can map to scene objects.

## 3. Hydraulics (the hard part — build first)

Quasi-steady network solve each tick. Unknowns: pressure at every internal
node. Equations: mass balance at every internal node, with branch flow given
by the element characteristic between adjacent nodes:

> **Amended by M5.3.** "Quasi-steady" is no longer exactly right, and the
> difference is worth stating where the claim is made rather than only in §3a.
> With a capacitive vessel in the network, one solve is not a steady state — it
> is **one implicit-Euler step of a differential-algebraic system**: an algebraic
> mass balance `Σṁ = 0` at every zero-volume node, and `Σṁ = C·dP/dt` at every
> capacitive one. Pressure *accumulation* is now inside the solve instead of
> absent from the model. Pressure WAVES remain out of scope, unchanged: the
> vessel stores mass against pressure, it does not propagate anything. A network
> with no vessel in it is bit-for-bit the steady solve described below, because
> `C` is simply absent from every node's residual.

- Pipe (turbulent, Darcy–Weisbach): `dP = f(Re) · L/D · ρv²/2`; start with a
  fixed friction factor, add Colebrook/Haaland later.
- Valve (ISA): `Q = Cv_eff(opening) · sqrt(dP/SG)`, smooth near dP=0 to keep
  the Jacobian finite (use `Q ∝ dP/sqrt(|dP|+ε)` regularization).
- Pump: head curve `H(Q) = H0 − aQ²` (quadratic fit; per-pump params).
- Tanks/sources/sinks pin pressure (hydrostatic head for tanks:
  `P = P_top + ρgh(level)`).

Solve with Newton–Raphson, analytic Jacobian, faer dense LU first (networks
are small), sparse later if needed. Convergence: relative mass-imbalance
< 1e-8 per node, max 50 iterations, damped steps (line search: halve until
residual decreases). Non-convergence ⇒ `Err(SimError::SolverDiverged {...})`
with the residual history attached.

Fluids: incompressible liquid through M3. Gas/compressibility is milestone
M5 and gets its own design note before implementation.

No cavitation / vapor-pressure floor in M1: the hydraulic solve is a pure
pressure-flow system, so an over-driven pump (e.g. low downstream resistance)
can produce a genuine solution with sub-zero *absolute* suction pressure. That
is real cavitation the model does not yet represent; a vapor-pressure clamp is
a later milestone. Frontends should treat negative absolute node pressure as a
"cavitating" signal, not a solver error.

**SimpleFlowSolver** (game-fidelity): solves the *same* quasi-steady fixed point
as Newton, but matrix-free — **nonlinear Gauss–Seidel** over node pressures
instead of a global linear solve. Each free node takes a scalar Newton step from
its own mass imbalance, `ΔP_n = ω · imbalance_n / Σ_e g_e` with branch
conductance `g_e = ρ · dQ/d(dP) ≥ 0`, sweeping in ascending id order until the
max node imbalance is below tolerance (looser than Newton's). This is diagonal
(Jacobi) preconditioning of the same weighted-Laplacian system, so it is
scale-invariant across the wide pipe/valve conductance spread — unlike a fixed
pressure-gain constant, which diverges on stiff branches. O(edges) per sweep,
warm-started from the previous tick, so steady state costs a handful of sweeps;
non-convergence in `max_iter` sweeps is `Err(SolverDiverged)`, never a NaN or an
unconverged `Ok`. Node classification, element compilation, and the final
edge-flow/NaN-scan are shared verbatim with the Newton solver (`solvers/network.rs`),
so the two fidelities agree on well-posed networks to well within 5% (I5).

## 3a. Gas & pressure realism (M5) — specified before building

The roadmap opened M5 with a licence the other milestones did not get: "may be
simplified or deferred — decide with a written note". Taking that seriously is
this note's first job, so it begins with a **scoping verdict** rather than a
design, and the design follows only for what survives.

### "Gas" is three separate assumptions, and they are worth pricing apart

Everything before M5 is incompressible liquid. Introducing gas breaks three
*independent* assumptions, and bundling them is what would make this milestone
unbounded:

1. **Density is a per-edge constant for the tick.** `network::compile_edge`
   reads `ρ` once from the stored composition and never revisits it. For a gas
   `ρ = P·M̄/(R·T)`, so `ρ` depends on the very node pressures being solved for.
2. **Pressure is purely algebraic.** Every node either pins a pressure or gets
   one from an instantaneous mass balance (§3). Nothing in the model can *store*
   pressure — but a gas vessel's entire behaviour is that it fills, builds
   pressure, and relieves. A liquid tank fakes this well (its level is a slow
   state and its pressure follows) precisely *because* liquid capacitance is
   enormous; §"Capacitance" below prices that.
3. **Phase is not in the state vector at all.** `Stream` carries one
   `Composition` over one slate, with no vapour fraction, no latent heat, no
   equilibrium. Condensation and vaporisation are not a missing *unit*; they are
   a change to `Stream`/`Composition` and every reader of them.

**Verdict.** M5 buys (2) and a restricted (1); it **defers (3) entirely**. The
reasoning is in "What is deferred" below, but the short form is that (2) is what
gives the milestone its point — pressure that builds and relieves is the
behaviour the game and the damage model need, and it is the one the current
architecture cannot express at all — while (3) is a wide refactor whose absence
can be made *loud* rather than silent (a single mixed-phase guard), which is the
condition this project has consistently required of a deferral.

**A scope correction to the roadmap's own line.** M5's heading names "column
overheads" as a target. It is not reachable and should not be attempted here: a
column overhead is a *condensing* stream, which is assumption (3). What M5 does
reach is the other half of that line — the **flare**, and the vessel that
relieves into it. That is stated as a correction rather than quietly dropped.

### Frictional dissipation into the stream (the M2.2 debt) — build it

§4a defers pump work and valve throttling heat with an explicit re-opening
condition: *measure the rise on a plant where it should be largest, and build it
only if a gate on that number can be made to fail for the right reason.* The
measurement has now been taken on `tank_pump_valve.toml` at its reference state
(ṁ = 13.7532 kg/s), and it re-opens the question:

| term | ΔP [Pa] | ΔT = ΔP/(ρ·cp) [K] |
|---|---|---|
| control valve (Kv 50 at 50% open) | 393 798 | **0.094309** |
| the three pipes' friction | 15 767 | 0.003776 |
| pump curve droop (`α = ρ·g·a`) | 1 487 | 0.000356 |
| **total** | **411 052** | **0.098441** |

The valve term is **94× the 1e-3 K tolerance** the ambient-exchange tests
already carry, on the *existing* reference plant — not on a contrived high-head
service. M2.2's own prediction ("the case for revisiting is throttling, not
pumping") is confirmed with a number, and the deferral's premise no longer
holds.

**The rule, and why it needs no new parameter.** `QuadraticBranch` already
separates the two kinds of pressure term, and the split is exactly the physical
one: **`α` is dissipative, `β` is not.** `α·Q|Q|` is friction — pipe, valve
trim, pump droop — and becomes heat in the fluid; `β` is elevation head
(reversible potential) and the pump's pressure jump (shaft work in). So the
dissipated power on a branch is `α·Q|Q|·Q` [W] and the temperature rise across
it is `α·Q|Q|/(ρ·cp)`, with nothing to tune. That is why this lands and pump
*efficiency* heating does not: `(1/η − 1)·ΔP/(ρ·cp)` is ~0.03 K at η = 0.75 here,
which is falsifiable in magnitude, but `η` is a new parameter with exactly one
possible value in this repo — the `cat_oil_ratio` argument from M4.2 — so it has
no gate that could distinguish a right value from a wrong one.

**Where the heat lands: the edge, as a transform.** A device folds into its
outlet edge (fold-at-source), so the branch `α` that includes a valve or pump
already belongs to that edge. Dissipation is therefore one more term in the
per-edge outlet transform that `energy::pipe_outlet_temperature` and
`energy::edge_temperature_at` already own for ambient exchange — the same
structure, the same two readers (`inflow_totals` and the tank loop), and the same
trap waiting if only one of them is updated. A valve node consequently reports
its *inlet* temperature, with its heat appearing on the edge downstream; that is
the same display choice §4a made for a pipe's two ends, and it is stated rather
than fixed.

**The seam.** `core::energy` cannot compute `α·Q|Q|` — `QuadraticBranch` lives in
`solvers`, and re-deriving element physics in `core` would violate rule 2 as
plainly as a fidelity `if`. So the solver reports it: `HydraulicSolution` gains a
per-edge dissipated power, computed where `α` and `Q` are already in hand, and
`core` consumes it exactly as it consumes `edge_mass_flow`. Recomputing `ΔP_fric`
in `core` as `(P_up − P_down) − β` is the tempting alternative and is wrong for
the same reason: `β` is solver-side knowledge, and a `core` that knows it knows
the element physics.

**I6 gains the term rather than excluding it.** Unlike M4's `Δh_rxn`, which sits
outside the sensible-only frame, dissipation *is* sensible heat and is exactly
computable, so the energy-conservation proptest adds `Σ_edges` dissipation to its
`Q` budget. Excluding it would be widening an invariant to tolerate a term the
model knows precisely.

**The cost, named up front: this is the first deliberate break of the
bit-identical regression anchor.** M3.1 and M4 both held it. Dissipation cannot:
it has no free parameter, so there is no honest default that switches it off, and
every plant with a valve now warms. `scenarios/tests/isothermal_plant.rs` — the
flat line M2 built specifically to trap spurious offsets — *must* change, and the
goldens must be re-recorded. The alternative considered and rejected was a
`dissipation = false` scenario flag: it would preserve the anchor by making a
physics term optional, which is a fidelity `if` wearing a config file's clothes.
The replacement is strictly stronger than what it retires: the same plant, the
same trap, asserted against a *predicted* 0.094309 K instead of against zero.

#### Corrections from building it (M5.1, landed)

The note above got the seam, the rule and the cost right. Building it corrected
it on four points, each recorded rather than quietly absorbed.

**1. The transform is COUPLED, not one more term added on.** The note says
dissipation is "one more term in the per-edge outlet transform", which is true of
where it lives and wrong about how it composes. Heat released a fraction `ξ`
along a pipe has only `(1 − ξ)` of the pipe left to leak back out to ambient
through, so it does not arrive in full. Integrating
`ṁ·cp·dT/dx = ua'·(T_AMBIENT − T) + φ'` gives, with `C = |ṁ|·cp` and `β = UA/C`,

```text
T_out = T_AMBIENT + (T_in − T_AMBIENT)·e^{−β} + (Φ/C)·ψ(β),   ψ(β) = (1 − e^{−β})/β
```

Adding `Φ/C` on top of the pure exponential over-credits it by `(Φ/C)(1 − ψ)`,
which is **first order in `β`** — hence exactly zero wherever `UA = 0`, which is
every scenario shipped today, and wrong the moment one is not. `ψ` is evaluated
through `expm1` because small `β` is the ordinary case and `1 − e^{−β}` cancels
there. That mutation (`ψ → 1`) fails exactly one gate in the workspace and no
others, which is what makes the coupling a tested claim rather than a nicety.

**2. `Φ` is assumed UNIFORM along the edge, and that is a limitation the seam
forces.** A valve's dissipation is really concentrated at its trim, which
fold-at-source puts at the edge's *inlet*, where the fluid then has the whole
pipe to shed it: `(Φ/C)·e^{−β}` rather than `(Φ/C)·ψ(β)`, a difference of
`≈ (Φ/C)·β/2`. It cannot be resolved as specified — the solver reports ONE `Φ`
per edge, so `core` cannot separate device friction from pipe-wall friction
without a second seam field. Zero at `UA = 0`; stated rather than discovered.

**3. The pipe-ambient LMTD identity GENERALIZES rather than breaking.** The
obvious patch — `ṁ·cp·ΔT == UA·LMTD − Φ` — is not an identity: with a frictional
source the profile decays toward `T* = T_AMBIENT + Φ/UA`, not toward ambient, so
the log-mean of the endpoint `(T − T_AMBIENT)` values is no longer the integral
mean. Taken about `T*` instead it is exact again, because the `−Φ` from the
source and the `+Φ` from integrating `UA·(T − T_AMBIENT)` over the offset cancel:

```text
ṁ·cp·(T_in − T_out) = UA · logmean(T_in − T*, T_out − T*)
```

`pipe_ambient_reference.rs` still checks it at 1e-12 relative. Relaxing the old
form's tolerance to admit `Φ` would have kept the test green while it stopped
discriminating — the failure mode M4.2's envelope note warns about, in a
different costume.

**4. I6 needed more than the budget line above.** Adding `Σ_e Φ_e` is necessary
and not sufficient: the boundary flux also had to move to the temperature at
which fluid crosses the **reservoir's own** boundary, selected by FLOW SIGN. A
reservoir that is upwind supplies its own temperature and the pipe's friction is
picked up afterwards, inside the control volume; reading the edge's outlet there
credits that friction twice. This was invisible before M5.1 for exactly the
reason M2.2's tank loop was: an edge with `UA = 0` was ISOTHERMAL, so both ends
were the same number and either read was right. Same lesson, second arrival.

**5. The blast radius was 18 gates across 8 files, not the one the roadmap
named.** Every absolute-temperature assertion downstream of a flowing pipe moved.
The fix that kept them sharp is worth stating as a pattern: an exact identity was
re-stated **across the unit** — from what ARRIVES on the inlet edge to what the
unit's NODE resolves to — which leaves both frictional terms outside the claim
and keeps it exact, instead of widening a tolerance to swallow them. A furnace's
first law, a cooler's signed first law, the exchanger's `ε·C_min` relation and its
energy closure are all stated that way now. Only claims that are really about
DIRECTION ("the inlet must not carry the duty") became bounds, and those still
discriminate by two orders of magnitude.

**One decision the note did not anticipate: a column draw reports `Φ = 0`.** Its
flow is prescribed (`splitᵢ·ṁ_feed`), not pressure-driven, so `α·Q|Q|` is not its
pressure drop and booking it would invent heat — the same argument that makes
`edge_flows` refuse to report a draw's flow at all (§5). That is what keeps
"draws leave at the feed temperature" an exact equality rather than three
different temperatures, and it is gated there.

**One guard, and an honest account of it.** `finalize` scans `Φ` for finiteness
and sign separately from the flows. The finiteness half follows the existing
convention — rule 5's "nothing non-finite escapes a solve", the same reason the
pressures and flows are scanned — and it is not redundant with them, because `Φ`
is a CUBE of the flow and can overflow to `+∞` while the flow itself stays
finite. **The sign half is cover held in advance and is unreachable today**,
stated in the manner `checked_temperature` states its own tank arm rather than
claimed to have earned its place: `Φ = α·|Q|³` and `α < 0` would make
`conducts` false, which zeroes the flow before `Φ` is ever formed — so no input,
including an unguarded negative pump curve coefficient, reaches it. It is there
for the fidelity that gives an element a genuinely signed characteristic, and
this note says so rather than letting a future reader infer it is falsifiable.

### Fork 1 — how does gas-ness enter the model?

**Phase is a property of the pseudo-component, not of the stream.** A component
is declared `gas` or `liquid` (defaulting to `liquid`, so every existing slate is
untouched) and its density law follows: `ρ = P·M̄/(R·T)` for gas,
`PseudoComponent::density` for liquid. `molar_mass` already exists on the
component and has had no reader until now; this is what it was reserved for. For
a mixture, `1/M̄ = Σ(wᵢ/Mᵢ)` — mass-fraction weighting of the reciprocal, the
correct rule, and the exact analogue of the existing liquid `mixture_density`.

The alternative — phase as a *stream* state (vapour fraction) — is assumption (3)
and is deferred. What makes that deferral safe rather than silent is a single
guard: **a composition mixing gas-phase and liquid-phase components is refused**,
and, because streams mix at runtime wherever two lines join, the check is
topological and done at load time — *every connected component of the plant graph
is all-gas or all-liquid*. A model with no phase equilibrium must not be handed a
two-phase mixture and quietly volume-average it.

Two consequences, both stated as limitations rather than discovered later:

- The FCC slate is **unchanged**: its "light gases" lump stays a liquid-phase
  pseudo-component, because declaring it a gas would make `fcc_plant.toml`
  illegal under the rule above. That is precisely the thing two-phase would fix,
  and it keeps M4's goldens bit-identical.
- A gas system is therefore a **separate sub-plant** in M5 — vessel, header,
  relief line, flare — not a gas stream threaded through a liquid plant.
- A flare needs **no new node kind**: it is an ordinary `Sink` at `P_ATM` with a
  gas composition and a temperature, all of which `Sink` already carries. The
  stack, the flame and the smoke are presentation (M6).

**The circularity, and the blast radius.** `ρ` now depends on `P`, which is the
unknown. The resolution is the one §3 and M3.1 already use for staleness, tightened
one notch: the transport density on an edge is evaluated at its **upwind node's
current pressure iterate**, recomputed each solver iteration rather than once per
solve. This is frozen-coefficient Newton — the `dρ/dP` term is omitted from the
Jacobian, so convergence is linear rather than quadratic near the root, but the
fixed point it converges to is the consistent one, which a previous-*tick* density
would not be (a blowing-down vessel changes density fast enough that a one-tick
lag is a defect, not a lag — the M3.2 lesson). Structurally this moves
`compile_edges` inside the iteration loop in `network.rs`, shared by both
fidelities as everything in that file is. For an all-liquid network the recompile
is a pure function of unchanged inputs, so it reproduces bit-identical numbers and
the M1–M4 goldens survive this slice (they do not survive dissipation — see
above).

Blast radius of the signature change, confirmed rather than assumed: outside
`components.rs` itself there are exactly **three** callers of `mixture_density`
— `network::fixed_pressure` (tank hydrostatic), `network::compile_edge`
(transport), and the scenario loader's tank-mass computation. The cost line the
scoping verdict rests on is genuinely small.

#### Corrections from building it (M5.2, landed)

The verdict above survived intact — phase on the component, the topological
guard, upwind density recompiled per iteration, the FCC slate untouched, a flare
as an ordinary `Sink`. Six things it got wrong or did not see, each stated where
it matters rather than folded into the paragraph it corrects.

**The blast-radius count was of the wrong function.** `mixture_density` did not
change signature at all: the dispatch is a NEW method, `Composition::density_at
(slate, P, T)`, which branches on the composition's phase and calls
`mixture_density` for the liquid arm. Only `compile_edge` moved to it, so
`mixture_density` is now *by name* the liquid rule with two callers — the tank's
hydrostatic head and the loader's tank mass. That is a better outcome than the
one costed, and it is what makes the next correction possible.

**Both remaining callers are TANKS, and the connected-component guard does not
protect them.** A tank of gas is single-phase, so a component containing only it
and a gas sink is perfectly legal under the topological rule — and yet `ρ·A·h`
and `ρgh` are liquid-LEVEL quantities that name nothing for a gas, both reading a
stored density a gas component does not have. The refusal therefore sits at the
tank, one layer earlier than fork 1 put it: **a tank's composition must be
liquid**. A gas holdup is fork 2's capacitive vessel, whose state is pressure and
not level, which is why this costs the milestone nothing. With that guard in
place, `PseudoComponent::density` becomes `Option<KgPerM3>` — absent for a gas,
and *refused* at the loader if declared, on the same grounds that keep a "gas Cv"
out of M5.4: a number nothing reads is how an author comes to believe the model
uses something it does not.

**A pipe's stored composition needed a phase-correct seed, and the note did not
see it.** The seed is written at LOAD, before any transport has run, and it was
`Composition::pure(slate_len, 0)` from M3.1 onward. On exactly the slates this
guard exists to permit — one liquid sub-plant, one gas sub-plant, one slate —
component 0 is a liquid, so every gas line would compile its first solve at
~998 kg/m³ instead of ~6.6: not stale, wrong by a factor of 150, and silent,
because two reservoirs and a pipe converge happily on it. The seed is now the
first component of the pipe's own connected-component phase, which for an
all-liquid slate IS index 0 — so the fix is invisible by construction on every
pre-M5.2 scenario, and needs a mixed-slate two-sub-plant case to be falsifiable
at all. That case is now in the test file.

**"Move `compile_edges` inside the loop" was not the whole structural change.**
`classify` used to compute the anchored set, which needs the compiled edges,
which now need a pressure to evaluate `ρ(P,T)` at, which is the cold seed
`classify` produces — a cycle. It is broken by splitting the prologue into
classify → seed → compile → anchor → re-pin floating, owned by one shared
`network::prepare` so neither fidelity can get the order wrong. The anchored set
is still computed ONCE, from the seed compile, so it cannot flap mid-solve;
`conducts` is a sign-of-`α` test and cannot differ between iterates anyway.

**That last clause was true when it was written and M5.4's relief valve made it
false.** It holds for every element whose conductance is fixed for the duration
of a solve — a valve at an operator setpoint, a pipe, a pump. A PSV's opening is
a function of the pressure **iterate**, so the sign of its `α` genuinely can
differ between iterates, and a set computed once from the seed compile can be
STALE. Both directions are reachable from the I5 generators, and neither is
exotic:

- **seed-open, converged-shut — the solve fails outright, on both fidelities.**
  A dead-leg terminal behind the PSV enters the solve as an unknown because the
  seed says its edge conducts; at the answer the PSV is shut, the edge is inert,
  and that node's residual row and column are identically zero — a singular
  Jacobian. Newton diverges, Simple reports an infinite residual. Measured on
  83 of 306 generated spur trees.
- **seed-shut, converged-open — the benign half.** Every flow is right and one
  reported PRESSURE is wrong: the terminal stays parked at `P_ATM` though behind
  an open PSV it is perfectly determinate (a dead end carries no flow, so it
  sits at its neighbour's less the static head).

Deliberately NOT fixed in the slice that found it. The fix is either an outer
loop over the classification or a per-iteration anchored set whose dimension
changes mid-solve — and the freeze above was a considered choice against exactly
that flapping, so reversing it is a design decision with its own slice and its
own measured regression anchor, not a test-only patch. What holds the line
meanwhile: `known_defect_frozen_anchoring_*` in `crates/solvers/tests/
invariants.rs` pin both halves as characterization tests written to FAIL when the
defect is fixed, at which point their assertions become the description of the
fix; and the generated rates are floored in the arm so the deferral cannot
quietly worsen.

**The iterate needs a pressure floor that the converged answer does not.** A
Newton trial can overshoot to a non-positive pressure on its way to the root,
where `ρ = P·M̄/(R·T) ≤ 0` makes the pipe resistance non-positive and
`compile_edge` returns `Err` — turning a *transient* into a failed solve. The
evaluation pressure is floored at `RHO_EVAL_P_FLOOR = 1 Pa`, which keeps the
coefficient finite and tiny so the Armijo line search rejects the step by the
same mechanism that handles every other bad one. It is a regularisation of the
iterate, exactly the role `eps_dp` plays for `√dp`, and no converged physical
solution comes near it.

**Upwind is selected by PRESSURE, not by flow direction, and that is
well-conditioned rather than merely convenient.** Direction is not available when
the coefficient is compiled — it is what the compiled branch computes — so the
higher-pressure endpoint is the density's node. The argument that makes it safe
is that the two are only ambiguous when the endpoint pressures are close, and
then the two candidate densities are close too: the choice cannot matter much
exactly where it is hardest to make. It is exact whenever `β` is small against
the branch drop, which is every gas line (elevation head `ρ·g·Δz` at ~6 kg/m³ is
~1e-3 of a liquid's). The one case that breaks it is a PUMP folded into a gas
edge, where `β = ρ·g·h0` can put the higher pressure downstream. It is **not
guarded**: a compressor is not a pump and no M5 plant has one, so the guard would
be unfalsifiable — the same test that killed it here is the one that kept the
tank guard.

**Measured, since the note predicted the shape and not the size.**
Frozen-coefficient Newton costs 8–9 iterations cold on the reference gas plant
and 0–1 warm-started at steady state, against a 50 cap; the two fidelities land
within 1e-8 of each other. Linear convergence is free at this scale.

Two things about *testing* a gas plant that were not obvious in advance, both of
which changed what the reference plant is:

- **Two reservoirs and a pipe never enter the Newton loop.** With both endpoints
  pinned, `n == 0` and the solve returns directly — so the obvious reference
  plant would have exercised the density law and none of the frozen-coefficient
  machinery this slice is about. `gas_line.toml` therefore has a free tee between
  two pipes of different diameter, which also makes "each edge takes its own
  upwind" a claim with a consequence (6.58 vs 6.10 kg/m³, 3.5% on the flow).
- **Friction heating is not a footnote in gas service.** M5.1's dissipation puts
  130 kW into 0.9 kg/s of gas on this plant — an 80 K rise, which feeds straight
  back into `ρ`. The reference therefore solves the FRESHLY BUILT graph, where
  the stream is still at the seed temperature the file declares, rather than
  ticking to a steady state whose density depends on a thermal history.

And one coverage note in the shape of M3.1's: **the wired plant cannot carry the
whole density law.** It runs at exactly `T_AMBIENT` and on a pure single
component, so a `T` dropped from `P·M̄/(R·T)` and a mean molar mass computed as
`Σ(wᵢ·Mᵢ)` instead of `1/Σ(wᵢ/Mᵢ)` are both invisible to it — the first because
the temperature is the one the slip hardcodes, the second because the two rules
coincide on a pure cut. Both are pinned by `core::components` unit tests instead,
and mutation confirms each fails there and nowhere else.

#### Corrected after landing: the density was evaluated at the wrong END of the edge

Found while scoping M5.3, fixed ahead of it in its own commit, because it is an
M5.2 defect rather than a cost of the vessel.

`compile_edge` read `pipe.stream.temperature`, which §4a defines as the edge's
**OUTLET** — the inlet transformed by ambient exchange *and by the edge's own
frictional dissipation*. So the transport density described the gas that had
already crossed the pipe, not the gas entering it. In gas service that is not a
footnote: expanding an ideal gas across a branch dissipates `Δp/ρ` per kilogram,
i.e. `ΔT/T = (γ−1)/γ`, ~20% for a light gas at any pressure ratio worth
simulating. Measured on `gas_line`: the relief line stores **375.0 K** while its
upwind tee sits at **297.3 K**.

What makes it a defect and not another accepted staleness is that **the offset
contains no `dt`**. `Φ` and `ṁ` are both instantaneous, so `Φ/(ṁ·cp)` is the same
at any step size — it is a different steady model, which no tolerance can be
derived around and no order-of-convergence gate would diagnose, because it does
not shrink toward zero. Fork 2's rate gate depends on the identity
`ρ_edge = m_vessel/V`, which holds exactly only when the edge's temperature is the
vessel's own.

The fix: the temperature comes from the upwind NODE via
`energy::boundary_temperature` where the node has one. Composition needed no
corresponding change — step 3b writes each stream's composition as its upwind
node's, since a pipe trades heat and never mass, so the stored copy already *is*
the upwind value.

**Stated limitation, with its measured size.** A zero-volume upwind node
(junction, valve, pump, exchanger side) has no temperature of its own: its value
is the sweep's mix, which does not exist when the solve opens the tick and is not
stored on the graph. Those edges keep the stored-outlet fallback and keep the
error. Un-defers when the flow solver gains the previous tick's resolved node
states — a `FlowSolver` signature change, deferred to the slice that first needs
it, which is M5.4 (vessel → valve → relief line → flare puts a *valve* upwind of
the line the milestone reads).

Blast radius, measured rather than predicted: the seven pre-M5.2 scenarios × two
fidelities × 200 ticks are byte-identical (14/14) — a liquid's `density_at`
ignores `T`. `gas_line` moves by **+0.10%** on the flow, and the smallness is
itself the limitation above talking: the edge that was 4 K wrong had an inertial
upwind node and was corrected, while the edge that is 78 K wrong has the tee
upwind and was not.

### Fork 2 — capacitance: how a vessel's pressure enters the solve

This is the central design decision of M5. A gas vessel stores mass, and its
pressure is `P = m·R·T/(V·M̄)`. Two ways to couple that to the hydraulic solve:

- **(A) Explicit — the vessel is a `Tank` with an ideal-gas pressure law.**
  `fixed_pressure` returns the pressure implied by the *start-of-tick* inventory;
  the solve treats it as a pinned reservoir; the inventory integrates afterwards.
  Zero solver machinery. It is exactly what `Tank::bottom_pressure` already does,
  with `ρgh` swapped for the gas law.
- **(B) Semi-implicit — the vessel is a free node carrying a capacitance term.**
  Its residual is `R_n = Σ_e ṁ_e(P) − (m(P) − mⁿ)/dt` with `m(P) = P·V·M̄/(R·T)`,
  so the accumulation term is `C·(P − Pⁿ)/dt` with **`C = V·M̄/(R·T)` [kg/Pa]**.

**(A) is rejected, and the reason is a stability bound that is reachable at
ordinary plant scale.** Linearising (A) about a steady state, with
`g = −dΣṁ/dP ≥ 0` the total conductance of the vessel's branches [kg/(s·Pa)],
one step is `δmⁿ⁺¹ = δmⁿ·(1 − dt·g/C)`, which is stable only while

```
dt · g / C  <  2
```

Everything turns on the size of `C`, and liquid and gas are five orders apart:

| body | capacitance `C = dm/dP` | `dt·g/C` at its own reference state |
|---|---|---|
| `tank_pump_valve`'s supply tank (`A/g` = 20/9.81) | **2.04 kg/Pa** | 8.2e-7 |
| 1 m³ knock-out drum, light gas (M̄ 0.03), 300 K | **1.20e-5 kg/Pa** | **4.2** |

*(Measured by building it: **8.35** on `knockout_drum.toml`. The row below costs a
single branch; a real knock-out drum has an inlet and an outlet, and `g` sums
over a vessel's branches. The verdict is stronger on the plant than in the
table.)*

The drum's row uses a 20 kg/s line at 0.2 bar drop (`g ≈ ṁ/2ΔP = 5e-4`) and
`dt = 0.1 s` — a small vessel on a fat low-pressure line, which is an
*unremarkable* piece of plant, not a contrived one. It is a factor of two outside
the bound and would oscillate and diverge. The tank is six orders *inside* it.

That table is the whole argument, and it also explains why M1 was right to do the
simple thing: the explicit treatment was never justified, it was merely never
loaded. This is the same reachability test §4a used to decide that the tank's
ambient term needed a comment rather than a guard (`UA·dt/(m·cp) ~ 1e-6`) — the
criterion is not "can this be made to break" but "does plausible input break it",
and here plausible input does. A guard would be worse than useless: the regime it
would refuse is a small vessel relieving quickly, which is exactly the scenario
the milestone exists to simulate.

**(B) also earns three things beyond stability**, which is what makes it a design
rather than a workaround:

- **`C` is exact, not a linearisation.** `m(P)` is *linear* in `P` at fixed `T`
  and `M̄`, so `C = V·M̄/(R·T)` is the whole relation, and the Jacobian entry is
  exact even though the density coefficient elsewhere is frozen.
- **It unifies the three node classes rather than adding a fourth.** `C → 0` is
  the zero-volume junction (`Σṁ = 0`); `C → ∞` is the infinite reservoir (`P`
  immovable). A capacitive vessel is the continuum between them, and the
  diagonal `C/dt` it adds to the Jacobian strictly *improves* conditioning.
- **Capacitance is an anchor.** `network::anchored_set` currently calls a free
  node with no conducting path to a pinned node "floating" and zeroes its edges.
  A capacitive node needs no such path — its own equation determines its pressure
  — so it counts as an anchor, and a closed gas system with no fixed node at all
  becomes well-posed for the first time.

`FlowSolver::solve` already takes `dt`, so the trait does not change. The
`SimpleFlowSolver` inherits the term with no new concept: its per-node update
`ΔP = ω·imbalance/Σg` gains `C/dt` in the denominator and the accumulation term
in the numerator, which is the same diagonal preconditioning it already performs.
Both fidelities keep solving the same fixed point, so I5 stands.

**What "quasi-steady" now means, stated so §3 is not quietly falsified.** With a
capacitive node in it, one solve is no longer a steady state — it is one
*implicit-Euler step of a differential-algebraic system*: algebraic mass balance
at every zero-volume node, `C·dP/dt` at every capacitive one. Pressure waves
remain out of scope; what changes is that pressure *accumulation* is now inside
the solve instead of absent from the model.

### Fork 3 — internal energy: a gas vessel needs `cv`, and gets it free

The tank energy balance uses `u ≈ h = cp·(T − T_REF)`, which is the
incompressible approximation. It is wrong for a gas holdup by a factor of `γ`,
and the error is not academic — it is the difference between a vessel that cools
as it blows down and one that does not.

For an ideal gas the correction costs no new parameter: `cv = cp − R/M̄`. So the
rule is `u = cv·(T − T_REF)` with **`cv = cp` for liquid components and
`cp − R/M̄` for gas ones**

*(Corrected by building it: `u = cv·(T − T_REF)` is **wrong** — it is
datum-inconsistent with `h = cp·(T − T_REF)` and would suppress the very cooling
this fork is about, by a factor of 15. The shipped rule is `u = cv·T − cp·T_REF`;
see "Corrections from building it (M5.3, landed)" below. The `cv` rule itself,
and the paragraph that follows, are unaffected.)* — phase-conditional, which is legitimate (it is a
phase branch in a property law, not a fidelity branch), and bit-identical for
every existing all-liquid scenario. Note the branch must be phase-conditional:
applying `cp − R/M̄` to water would shift its `cv` by 11% and change every
golden in the workspace.

The payoff is that blowdown cooling then **emerges** instead of being modelled:
with `d(m·u)/dt = ṁ_out·h_out` and the stream leaving at the vessel's own
temperature, `m·cv·dT/dt = −ṁ·(cp − cv)·T`, whose first integral is
`T/Tᵢ = (m/mᵢ)^(γ−1)`. That relation is *path-independent* — it does not contain
`t`, the resistance, or the downstream pressure — which is what makes it a
reference gate rather than a readback of the integrator.

#### Corrections from building it (M5.3, landed)

Forks 2 and 3 survived in outline — capacitance in the shared residual,
capacitance as an anchor, `cv` phase-conditional, the vessel inertial like a
tank, blowdown cooling emergent. Four things they got wrong or did not see.

**Fork 3's `u = cv·(T − T_REF)` is wrong, and it would have killed fork 3's own
first integral.** The rule has to be consistent with the enthalpy every stream
crossing the boundary already carries, `h = cp·(T − T_REF)`, and thermodynamics
fixes their difference at `h − u = P/ρ = (R/M̄)·T`. The stated rule gives
`(R/M̄)·(T − T_REF)` instead. The correct form is

```
u = cv·T − cp·T_REF
```

which is `cp·(T − T_REF)` for a liquid (`cv = cp`) — so every existing tank
balance is bit-identical — and is what makes `m·cv·dT/dt = −ṁ·(R/M̄)·T` come out.
Note fork 3 *derived* that ODE with `T` and then wrote the rule with
`(T − T_REF)`, contradicting itself in the same paragraph. The size of the slip:
The size of the slip:
on the reference blowdown the inconsistent datum predicts a **5.5 K** drop where
the consistent one predicts **80.3 K** over the run. Both are finite, smooth,
mass-conserving and reproducible — the `well-posed ≠ correct` failure mode
again, and the reason gate (ii) is a first integral and not a plausibility check.
The offset is invisible whenever a holdup's mass is constant, which is why no
tank ever noticed and why only a vessel could surface it.

**The Simple solver was measuring its residual against stale coefficients.** It
compiled the density coefficients at the start of each sweep and then measured
the post-sweep residual — and shipped the resulting flows — with those same
pre-sweep coefficients. Harmless while every free node was warm-started at a
pressure it barely moved from; a capacitive vessel moves ~400 Pa **every tick by
design**, so the sweep converges in one pass and the coefficient is never
refreshed. That put the two fidelities 2.0e-4 apart on the blowdown, about four
orders above the residual either solver reported. A convergence flag can be
perfectly honest about a fixed point nobody solved. The compile moved after the
sweep; all-liquid networks are unaffected because `density_at` ignores its
arguments there.

**"Capacitance is an anchor" is two changes, not one, and they are in different
crates.** `network::anchored_set` had to seed its walk from the capacitive nodes
as well as the fixed ones — but the scenario loader would have refused the plant
before the solver ever saw it, because `validate_topology` demanded a
*pressure-fixing* node per connected component. The load-time predicate is
therefore a distinct one (`provides_pressure_reference`) from
`network::fixed_pressure`, which still answers `None` for a vessel. Two names,
two meanings; collapsing them would either refuse closed gas systems or pin a
vessel's pressure.

**The stability verdict is stronger on the plant than in the table.** Fork 2
costs `dt·g/C = 4.2` for the drum, from a single 20 kg/s branch. The reference
plant wires it to two — a drum with an inlet and an outlet, which is what a
knock-out drum is — and `g` is the sum over its branches, so the measured figure
at the plant's own converged state is **8.35**. The rejected explicit scheme's
amplification factor `|1 − dt·g/C|` is 7.35 per tick there.

One thing the note predicted and the build confirmed, worth recording because it
was the load-bearing assumption of gate (iii): `m_new = C·P_solved` holds
identically, so the discharge edge's `ρ = P·M̄/(R·T)` IS the vessel's `m/V`
exactly. That is what makes the closed-form blowdown an anchor rather than a
readback — and it only holds because a gas edge takes its temperature from the
upwind NODE (see the M5.2 correction above); with the pipe's own outlet
temperature it is off by `(γ−1)/γ` at every step size.

### Fork 4 — choked flow versus the C¹ invariant

`elements.rs` commits in its header to characteristics that are C¹-smooth through
zero, because the Newton Jacobian must stay finite; that is why the regularised
`x/√(|x|+ε)` exists at all. Choking is a **cap**, and a cap is a kink. "Add
choking later" would silently break that contract, so this note picks rather than
postpones.

**Choking ships, with a smoothed transition, because relief cannot be honest
without it.** A PSV discharging 10 bar to a flare at 1 bar is at a pressure ratio
of 0.1 — deeply choked — and an incompressible orifice law applied there
overpredicts the relieving rate substantially and monotonically. Since the point
of M5.4 is *what happens during a relief event*, a flow law valid only above a
pressure ratio of ~0.7 would be wrong exactly where it is read. The alternative
(refuse below the critical ratio with an `Err`) fails on its own terms: it turns
the milestone's headline scenario into a solver error.

The form is the published one — IEC 60534-2-1 / ISA-75.01 gas sizing, the same
standard M1's `Kv` anchor came from: `ṁ = Cv·Y·√(x·P₁·ρ₁)` with the pressure-drop
ratio `x = ΔP/P₁`, the expansion factor `Y = 1 − x/(3·F_k·x_T)` and `x` clamped at
the choke point `F_k·x_T` (where `Y = 2/3`). The clamp is replaced by a smoothstep
blend over a narrow band in `x`, so `Y(x)` and `dY/dx` are both continuous across
the choke — and the *continuity of the Jacobian entry across the choke point* is
itself a gate, since it is the invariant being risked. The band width is a
numerical parameter and must be shown not to move the answer: a gate at two band
widths an order apart, agreeing to well inside the reference tolerance.

**The parameter count, because this is where a published anchor can quietly stop
being one.** The equation brings three coefficients, and they are not alike:

- **`Cv` is not new.** The standard uses the *same* valve flow coefficient for
  liquid and gas sizing — different equation, one coefficient — so the gas form
  reuses `cv_si` exactly as loaded from metric `Kv` today, and degenerates to the
  existing liquid branch at `Y = 1`. This is the load-bearing line that keeps the
  parameter count at one, and it is why a "gas Cv" field must not appear.
- **`F_k = γ/1.40` is derived**, from the slate's own `cp` and `M̄` via
  `γ = cp/(cp − R/M̄)`. Nothing to declare and nothing to invent.
- **`x_T` is genuinely new, per-valve, and manufacturer-tabulated**, and it is
  the one that could turn this gate into M4.2's envelope circularity: hardcode a
  single invented `x_T` and *both* the ISA gate and the choked-plateau gate pass
  for whatever value was chosen. That is precisely the argument that defers pump
  `η` here, and it has to be answered rather than borrowed against.

**`x_T` is therefore a declared field** on the valve/relief node — required in
gas service, with no default, since a silent default is the invented value in
disguise — and the reference plant carries a value **cited from IEC 60534-2-1's
own table of typical `x_T` by valve style**, so the number the gate reads is one
the workspace did not make up. The anti-circularity move is the same one M5.2
uses for pressure: the reference case runs at **two different `x_T` values**, so
what is pinned is the *dependence* and not one coincidence.

The residual ceiling, stated rather than glossed: this anchors the **equation**,
given inputs the scenario declares. It does not validate that any particular
valve's `x_T` is right — nothing in this workspace could, and a citation is the
honest substitute for a measurement.

### Fork 5 — the relief valve is a characteristic, not a controller

A PSV is the first element in this project whose opening depends on the plant
state, which is one short step from a controls subsystem (level control, pressure
control, cascades, tuning, integral windup). M5 does not take that step. **The
relief valve is a pure element characteristic**: its opening is a smooth,
memoryless function of its own upstream pressure — closed below the set pressure,
ramping to full over the accumulation band above it — evaluated inside the solve
alongside every other branch characteristic. No state, no tuning constants, no
tick history, and it lives in `elements.rs` with everything else.

This is a deliberate scope boundary, and it is also the physically honest model
at this fidelity: a spring-loaded PSV really is a pressure-actuated area, not a
controller. What it gives up is stated: no blowdown hysteresis (a real PSV
recloses below its set pressure), and no chatter dynamics. Both need state, and
state is what turns an element into a controller.

### Fork 6 — how the choked law enters `QuadraticBranch` (M5.4, before building)

Forks 4 and 5 settle *which* equations M5.4 ships. Neither settles the question
that decides how much of the workspace moves: **the ISA gas law is not affine in
`Q·|Q|`, and `QuadraticBranch` is.** Every branch in this project is
`dp = α·Q|Q| + β`, which is what makes the fold-at-source convention compose in
closed form and what both fidelities and the Jacobian are built on. A law that
depends on the *absolute* upstream pressure — and that goes flat above the choke
point — is not of that form. This fork settles how the two meet, and it settles
reverse flow, which fork 4 does not mention at all.

**The algebra first, because it is better than it looks.** The standard's mass
form is `W = C·N₆·Y·√(x·p₁·ρ₁)` with `x = Δp/p₁`. Since `x·p₁ = Δp` identically,
that is `W = C·Y·√(Δp·ρ₁·ρ_ref)` in coherent SI. The repo's liquid valve is
`Q = cv_si·opening·√(dp/ρ_rel)` with `ρ_rel = ρ/998`, so
`ṁ = ρ·Q = cv_eff·√(ρ_ref·ρ·dp)` — **the same expression**. Fork 4's "degenerates
to the liquid branch at `Y = 1`" is therefore literally true, not approximately,
and the one-coefficient claim is arithmetic rather than an aspiration. The first
gate of this slice is that identity: at `Y = 1` the gas branch reproduces
`QuadraticBranch::valve` bit for bit.

**The clamp is inside the square root, not only in `Y`.** The sizing variable is
`x_s = min(x, F_k·x_T)`, substituted into `√(x_s·p₁·ρ₁)` *as well as* into
`Y = 1 − x_s/(3·F_k·x_T)`. Scaling `α` by `1/Y²` alone gets `Y` right, leaves
`√Δp` inside, and therefore has **no plateau at all** — a model that reads as
choked (`Y` bottoms out at 2/3) and is not. Only fork 4's plateau gate would
notice. This is why the clamp cannot be an afterthought in the implementation.

**The verdict: a frozen effective `α`, with a bounded inner scalar solve for the
valve's own share of the branch drop.** Written out, the gas valve is exactly the
liquid one with

```text
α_eff = α_liquid · r / Y² ,   r = x/x_s ,   Y = 1 − x_s/(3·F_k·x_T)
```

so it stays a `QuadraticBranch` and nothing downstream — series composition, the
Jacobian, `edge_flows`' `Φ = α·Q|Q|·Q`, the Simple sweep, `finalize` — learns that
gas exists. Unchoked, `r = 1` and this is the `1/Y²` scaling; choked, `x_s` is
pinned at `F_k·x_T` and `r` grows in proportion to `x`, which *is* the plateau
expressed as a coefficient. Writing `r = max(1, x/x_c)` rather than `x/min(x,x_c)`
also disposes of the `x → 0` division: `r` is identically 1 there.

The coefficient is **frozen at the iterate and recompiled every iteration**,
which is the pattern M5.2 already blessed and measured for `ρ(P,T)`: `dY/dx` and
`dr/dx` are omitted from the Jacobian, so convergence near the root is linear
rather than quadratic, and the fixed point reached is the *consistent* one. Both
fidelities inherit it because both go through `compile_edges`.

**What is genuinely new is the inner solve, and it is new because the fold is.**
`x` is `Δp_valve/p₁` — the valve's OWN drop — while a compiled edge only knows
the drop across the whole folded branch (pipe ∘ valve). Attributing the entire
branch drop to the valve was considered and rejected: it applies a valve's
`x_T` to a pipe's friction, and it would contaminate the one gate in this slice
that is supposed to be an independent published anchor. Taking `x` from the
*unchoked* flow instead — one predictor pass, no iteration — is worse, and
quietly so: at outer convergence the pressures stop moving, so the coefficient
stops moving too, and the solver settles on a fixed point that is **not** the ISA
solution. That is a silent wrong number of exactly the kind this workspace
refuses.

So `compile_edge` solves, for the valve's share `s = Δp_valve ∈ [0, D]` with
`D = dp − β` the folded branch's shifted drop,

```text
g(s) = s + α_pipe·ṁ(s)²/ρ²  =  D ,     ṁ(s) = cv_eff·Y·√(ρ_ref·ρ·x_s·p₁)
```

`g` is continuous, `g(0) = 0` and `g(D) ≥ D`, and it is strictly increasing —
the `s` term alone is, and `ṁ(s)` is non-decreasing once the choke is smoothed
— so the root exists, is unique, and **bisection is unconditionally robust**.
That is the property that makes an inner solve acceptable here: it is scalar,
bracketed, monotone, deterministic and local to one edge, not a nested Newton
whose failure would have to be reported. It costs the closed-form inverse
`newton_flow`'s header advertises, on gas valve edges only; every other edge in
the project keeps it.

**Reverse flow, which fork 4 does not mention.** `Y = 1 − x/(3·F_k·x_T)` with
`x < 0` gives `Y > 1` — an expansion factor that *increases* the flow, which is
unphysical, and unbounded as the flow reverses harder. `compile_edge` already
names the higher-pressure endpoint the upwind one, so the resolution is to
evaluate `x` from `|D|` against the **upwind** node's pressure, leaving the
branch odd about `β` exactly as `elements.rs`'s shifted-oddness tests assert. A
gas valve therefore chokes symmetrically in both directions. For a control valve
that is right. **For a PSV it is a limitation, and it is stated here rather than
discovered**: a real relief valve does not pass reverse flow at all, and this one
does — the same register as fork 5's "no blowdown hysteresis, no chatter", and
un-defers with the same element state those need.

**`x_T` and `F_k` must not invent a second notion of "gas service".** `cv_si` is
one field for both services, so something has to decide when a valve needs an
`x_T`. That decision already exists: M5.2's single-phase connected-component
analysis. The loader reuses it. Building a second, independent test for "this
valve is in gas service" gives two answers that can disagree, and the failure
mode is a plant that loads with no `x_T` and silently runs the liquid branch on
gas. `F_k = γ/1.40` with `γ = cp/(cp − R/M̄)` likewise reuses M5.3's
phase-conditional `cv`, so it is composition-dependent and evaluated per edge at
the upwind composition, consistent with `ρ`.

**The upwind TEMPERATURE deferral un-defers here, and it is a prerequisite rather
than a tidy-up.** `network::compile_edge` states the limitation and names this
slice as the trigger: a zero-volume upwind node has no temperature of its own, so
the edge compiles its density at the *pipe's stored outlet* temperature — on
`gas_line` at steady state, 375.0 K instead of 297.3 K, a 21% density error. M5.4
is where that stops being tolerable, and the reason is one sentence: **`ρ₁` enters
the ISA equation under a square root, so a 21% density error is ~10% on `ṁ` — and
an ISA reference gate cannot be an independent published anchor while the density
it reads is 21% wrong.** The fix is a `FlowSolver::solve` signature change
carrying the previous tick's resolved `NodeStates`, which `Engine` already holds
and already passes to `resolve_node_states`; it is threading, not new state.

Note what changes about the *kind* of error this leaves. The stored-outlet
fallback is a fixed offset with no `dt` in it — a different steady model, which is
the M3.1 `cp` lesson's test for a defect rather than a staleness. The previous
tick's resolved temperature is an honest staleness: it shrinks with the step, and
it is the same lag §3 already accepts for the tank levels feeding a quasi-steady
solve. The order of preference is therefore `boundary_temperature` (a node with
its own temperature is current, not lagged), then the previous tick's resolved
value, then the stored outlet — which is only reachable on tick 0, before any
sweep has run.

**This slice ships in three commits**, for the reason CLAUDE.md gives and the
reason `separate-the-exposed-defect-from-the-feature` gives: (a) the upwind
temperature un-deferral, which is independent of choking and carries its own
regression cost; (b) choking — `Y`, the smoothed clamp, `x_T`, `F_k`, and gates
(i)–(iii) of fork 4's list; (c) the relief valve and the demo. The relief valve is
its own `NodeKind` rather than a flag on `Valve`, on the `Cooler`-versus-negative-
`Furnace` precedent: the intent belongs in the name, not in the sign or the
presence of a number in a TOML file.

**Correction to fork 4's own claim about its anchor, made before writing the
gate rather than after.** Fork 4 and the per-slice test list below both say the
ISA gate is "an independent published anchor, exactly as `kv_reference` anchors
M1 — its expected value comes from the standard, not from any formula in the
workspace." **That is an overclaim, and it does not survive contact with the
implementation.** `kv_reference` earns its status because the `Kv` *definition*
is a physical statement — a `Kv` valve passes `Kv` m³/h of water at 1 bar, SG 1 —
from which the test derives `cv_si` by a route the workspace does not contain.
There is no analogous non-formula statement behind `Y = 1 − x/(3·F_k·x_T)`. A
gate that computes an expected `ṁ` from that expression is checking
transcription, units and algebra: `kv_reference`'s *network hand calc* ceiling,
not its `Kv`-definition ceiling.

What does carry content independent of the implementation, and is what the slice
should lean on:

- **the `Y → 1` limit** — the gas branch reproduces `QuadraticBranch::valve`. Two
  code paths agreeing, not either one read back;
- **the plateau** — `dṁ/dP_downstream = 0` below the critical ratio, a *property*
  no unchoked law has, asserted at the folded branch and not only at the valve;
- **`Y = 2/3` exactly at and beyond the choke point** — a specific number, and
  independent of whatever `x_T` the scenario declares;
- **`F_k = γ/1.40` derived from the slate** through `Composition::mixture_cv`,
  rather than declared anywhere.

The intermediate-`x` magnitude is a restatement of the formula and is labelled as
one. `x_T`'s own provenance is a smaller matter than it looks: it is a *declared
input*, the two-`x_T` design pins the dependence rather than the value, and
nothing here is calibrated to it — so a secondary citation, marked as secondary,
is the honest treatment. The high-stakes half is the **equation form**, which
until now rested on this document alone. It has been checked against sources
outside the repo (`Y = 1 − x/(3·F_k·x_T)`, `F_k = γ/1.40`, choke at `x = F_k·x_T`
where `Y = 2/3`); the standard itself was **not** read, and the gate says so.

#### Corrections from building it (M5.4, landed)

Three things the fork above got wrong, and one it could not have known.

**The smoothstep is not needed, and fork 4's premise was false.** Fork 4 shipped
choking on the reasoning that "a choke cap is a kink" which would break
`elements.rs`'s C¹ contract. It is not a kink for this `Y`: the standard's
expansion factor is constructed so the sizing curve meets the plateau with zero
slope — `d(Y·√x_s)/dx → −√x_c/(3x_c) + (2/3)/(2√x_c) = 0` — and the frozen
coefficient inherits it, `dα_eff/dx = 2.25·α/x_c` on *both* sides. Measured, the
one-sided limits agree to round-off. So the exact clamp is already C¹ for the
flow AND for the assembled Jacobian entry, which is what fork 4 was protecting; a
blend would be a fabricated parameter that biases the answer and buys nothing.
`CHOKE_BLEND = 0`, the parameter survives so the claim stays falsifiable, and only
the second derivative jumps — which Newton does not need.

**The escalation trigger was not met.** Frozen `α` converges in 8 iterations cold
and 0 warm on the choked reference plant, 10 worst case on the relief demo, so
the branch type carrying `flow(dp, p_up)` with a true derivative stays deferred —
on the measurement this fork asked for, not on the argument.

**`P₁` is the upwind node's, and no forward-flowing plant can see otherwise.**
Taking it from the edge's `src` — which the fold-at-source convention puts right
next to the valve — passed the entire suite. It is right only while the flow runs
forward. A reversed plant, where the two differ tenfold, is what gates it.

**And one the fork could not have known, because it is about the Simple
fidelity rather than the element.** A normally-shut PSV leaves its valve node a
DEAD END: its outlet branch does not conduct, so the vessel's Gauss–Seidel
diagonal is dominated by a fat inlet branch carrying no net flow, and each sweep
moves the vessel by almost nothing. Newton is immune — it solves the linear
system exactly. This is a property of relief plants generally, not of one file:
any normally-shut branch on a low-resistance line will do it. The demo's inlet
line is sized against it (and is the more realistic geometry for it, per API
520's limit on PSV inlet drop), and the general statement belongs here: **the
Simple fidelity's stiffness limit is reached by a dead-ended branch, not only by
a wide conductance spread between flowing ones.**

**What would escalate this verdict, stated as a measurement rather than an
argument.** Frozen `α_eff` overstates the branch conductance on a choked valve —
the truth is `dṁ/d(dp) = 0` and the frozen form reports `ṁ/(2·(dp − β))` — which
*understates* the Newton step and is therefore slow rather than unstable. If the
PSV plant converges in tens of iterations, that is the answer and the linear rate
is documented the way M5.2's was. If it limit-cycles or hits the cap, that is the
justification for a branch type carrying `flow(dp, p_up)` with a true derivative,
and it lands in its own commit with the measured iteration counts attached. The
same measurement covers fork 5's hazard: a relief valve's opening rises with its
own upstream pressure, which is positive feedback on flow within a single solve.

### What the tests must pin, per slice

The order matters as much as the content: each gate must pin one thing, and each
must be shown to fail under a mutation aimed at it and stay green under the
others.

**Dissipation.** (i) The absolute hand calc: the valve's steady-state rise of
**0.094309 K** on `tank_pump_valve`, at round-off tolerance. (ii) The
mechanical-energy closure — `Σ dissipation = pump β − elevation β − net ΔP across
the plant` — which is derived independently of the term being tested and closes
here to **3 Pa in 411 052** (the regularisation floor). (iii) I6 extended with the
dissipation budget. Mutations that must each fail their own gate and no other:
crediting the *total* branch drop instead of `α·Q|Q|` (elevation booked as heat —
12% high, fails (i) and (ii)); dropping the pump's own `α`; writing the heat to
the edge's inlet instead of its outlet (the pipe-ambient trap, reachable only
through the tank-loop reader).

**Gas density.** A gas source → pipe → sink plant with the flow pinned against
`ṁ = √(2DA²ρP/(fL))` computed from `ρ = P·M̄/(R·T)` by hand, and a second point at
a different source pressure so the *P-dependence* is pinned and not just one
number. The mixed-phase load-time refusal gets its own case. A one-component
water regression run confirms the per-iteration recompile is bit-identical.

*(Corrected by building it: that plant has **no free node**, so the Newton loop
never runs on it and the frozen-coefficient recompile would go untested. The
shipped reference is source → tee → sink. See "Corrections from building it
(M5.2, landed)" under fork 1, which also records the two rules — the `T`
dependence and the reciprocal `M̄` — that no wired plant of this shape can carry.)*

That hand calc carries one `ρ`, the upwind node's, while a real gas expands along
the edge — so it pins the **upwind-density convention as implemented** and is not
a physics anchor, which is worth saying before someone reads it as one. The
physical error it accepts grows with the pressure ratio across the edge, in the
same direction and for the same reason as an unchoked orifice law's. That is what
M5.4 is for, and it is why the two slices are ordered this way rather than merged.

**Capacitance.** Three gates, because one cannot cover it:

- **The design-decision gate.** The 1 m³ drum of the table above — the case the
  rejected explicit scheme diverges on — run to a converged, bounded steady
  state. This is the gate that falsifies fork 2's verdict, and it is the one most
  easily left out.
- **The thermodynamic first integral.** `T/Tᵢ = (m/mᵢ)^(γ−1)` on a physically
  ordinary blowdown to atmosphere. Path-independent, so it pins the `cv` balance
  and the gas law without pinning the time integration.
- **The time coupling.** The first integral above has no `t` in it, so something
  must pin the capacitance-orifice *rate*. Discharging to a **zero-pressure
  sink** makes the ODE elementary: with `ṁ = √(2DA²ρP/(fL))`, `ρ = m/V` and
  `P = Pᵢ(m/mᵢ)^γ`, mass follows `dm/dt = −A₀·m^((1+γ)/2)`, whose solution is
  `m/mᵢ = (1 + ½(γ−1)·A₀·mᵢ^((γ−1)/2)·t)^(−2/(γ−1))`, with `P = Pᵢ(m/mᵢ)^γ`.
  Its ceiling is named rather than glossed, in the manner M4.2's envelope gate
  names its own: a vacuum sink drives the incompressible orifice law far outside
  where it is physical, so this gate pins the **numerics** of the
  capacitance/energy coupling, *not* the fidelity of the flow law. The flow law's
  physical anchor is the ISA gate below. The tolerance must be **derived from
  implicit Euler's O(dt) truncation, not tuned**, and backed by an
  order-of-convergence gate (halving `dt` halves the error) — the M4.2 lesson,
  and the only gate that catches a degraded scheme that still converges.
  **The fixture is unconfirmed and must be checked before it is relied on**: a
  `Sink` at 0 Pa may be refused at load, and it certainly drags `classify`'s cold
  seed (the mean of the fixed pressures) toward zero on a plant whose vessel
  starts at 10 bar, which is a bad start rather than a wrong answer but is worth
  knowing about first. If it does not load cleanly, the fallback is a small
  finite `P₀` with the closed form re-derived — isothermally it is the elementary
  arctan form, `arctan√((P−P₀)/P₀)` linear in `t` — or, failing that, numerical
  quadrature of the same ODE inside the test. What must not happen is the
  milestone's only *rate* gate quietly depending on a fixture nobody confirmed.

**Choking and relief.** (i) The IEC 60534-2-1 gas sizing equation — *and see the
correction under fork 6: this is NOT an independent published anchor in
`kv_reference`'s sense, because the expansion factor has no non-formula
definition to derive an expected value from. It is the network-hand-calc
ceiling. The gates that carry independent content are the `Y → 1` degeneracy, the
plateau, `Y = 2/3` at the choke, and `F_k` derived from the slate.* (ii) The
choked *plateau*: below the critical ratio, `dṁ/dP_downstream = 0`, a property no
unchoked law has and which a Y-factor implemented without the clamp would fail.
(iii) Jacobian continuity across the choke point, and band-width insensitivity.
(iv) The relief valve's memorylessness — the same upstream pressure gives the
same opening regardless of how it was approached, which fails the moment anyone
adds hysteresis without saying so.

### What is deferred, and what would un-defer it

- **Two-phase flow, flash and condensation** — assumption (3). The reason is
  **scope, not unfalsifiability**: a Tb-derived Raoult/Antoine flash *could* be
  pinned against a published binary, so the M4 "invented parameters with no gate"
  argument does not apply here and must not be borrowed. What defers it is that
  phase is absent from the state vector, so it changes `Stream`, `Composition`
  and every reader of them — a milestone, not a slice. It is made safe by the
  single-phase connected-component guard, so the model refuses two-phase input
  loudly instead of averaging it. Un-defers when a plant needs a condensing
  overhead or a flashing feed, i.e. the complex column (M6+).
- **Real-gas compressibility (Z), and gas `cp(T)`.** Ideal gas is adequate well
  away from the critical point, and both are additive to the density law once a
  case needs them.
- **Pump efficiency heating.** ~0.03 K here and falsifiable in magnitude, but `η`
  is a parameter with one possible value in this repo. Un-defers with a scenario
  carrying real pump curves and efficiencies, where a wrong `η` would be
  distinguishable from a right one.
- **PSV hysteresis and chatter** — needs element state; see fork 5.
- **Acoustic / pressure-wave dynamics** — out of scope since §3 and unchanged.

## 3b. Damage: the leak (M6) — specified before building

M6's ROADMAP stub said the damage commands were "already supported by the graph
model". Half of that is true, and the false half is the reason this section
exists.

### The finding: `PuncturePipe` is a command that does nothing

**Fire is real.** `Command::SetHeatInput` reaches the energy balance at
`energy.rs:1369` and `energy.rs:1496`, stacking on a unit's own duty rather than
replacing it, exactly as `snapshot.rs:26` promises.

**The leak is not.** Three independent checks, each of which alone would be
suggestive and which together are conclusive:

1. `Pipe::leak_area` (`graph.rs:451`) is **written** at `engine.rs:119` and
   **read by no solver**. Every other occurrence in the workspace is a test or
   loader fixture initialising it to `ZERO`.
2. `EdgeSnapshot::leak_mass_flow` is the literal `0.0`, carrying the comment
   *"populated when leak paths land"* (`engine.rs:690`).
3. **No test anywhere constructs `Command::PuncturePipe`.** The enum variant and
   the engine's match arm are its only two occurrences in the repo.

And there is no TOML surface for it either: `scenarios/src/lib.rs:380` hardcodes
`leak_area: ZERO`, so neither a command nor a config file can reach a nonzero
value. The field is unreachable, unread and unreported.

This is the failure mode this project has now recorded twice — a stored number
that nothing consumes is how an author comes to believe the model has a feature
it does not have (M5.2), and a fixture that pins a field to its identity value
disables the code path meant to be under test (M5.2 again). Here it went one
step further than either: the *command* existed, so the feature looked shipped
from the frontend contract inward.

**A related gap, found while checking the first.** `NodeKind::Atmosphere` is
expressible in TOML (`NodeDef::Atmosphere`, `scenarios/src/lib.rs:615`) and is
handled in four places — `P_ATM` (`network.rs:163`), `T_AMBIENT`
(`energy.rs:773`), a first-component composition (`energy.rs:742`), and the
column-draw outlet whitelist (`scenarios/src/lib.rs:1070`, which is *not* a
degree rule; `validate_degrees` leaves an Atmosphere unconstrained). **No
scenario and no test in the repo builds one.** So the leak's destination is
itself an untested degenerate, and M6's first gate will be exercising those arms
for the first time.

### Fork A — a scalar sink inside the pipe's own equation. **Rejected.**

The shape `Pipe::leak_area` currently implies: the pipe keeps its two endpoints
and loses mass somewhere along its length. It is rejected on evidence rather
than taste. `network::edge_flows` returns **one** `ṁ` per edge, and every
I-series mass balance sums exactly those. A mid-pipe sink makes one edge deliver
different mass at its two ends, which is not a local change: it propagates into
the mass balance, energy transport, composition transport, and dissipation —
`Φ = α·Q|Q|·Q` is computed from that single `q` at `network.rs:696`, and there
would no longer be a single `q` to compute it from. It also contradicts what
`graph.rs:8` and `snapshot.rs:21` already promise in prose ("a leak adds an edge
to an `Atmosphere` node"). The field's shape is the outlier; the comments are
the design.

### Fork B — graph surgery at puncture time. **Rejected, for the frontend.**

Split the punctured pipe into two, insert a junction, hang an orifice edge from
it to `Atmosphere`. The solver would not notice: `network::prepare` is called
per tick from inside both fidelities (`newton_flow.rs:96`, `simple_flow.rs:88`),
so a topology change *between* ticks is recompiled cleanly, and `PlantGraph`
wraps a `StableDiGraph` (`graph.rs:496`), so existing `NodeId`s and `EdgeId`s
survive an insertion.

It is rejected because the **snapshot changes shape mid-run**. Frontends consume
snapshots (rule 6), and a node list that grows during play is a contract problem
for every consumer, not just Godot. Repair has no story either: undoing damage
would mean removing nodes, and a `StableDiGraph` that keeps indices stable
across insertions does not promise anything as pleasant across removals.

### Fork C — a dormant leak path, decided at load. **Chosen.**

A leak is an **edge to an `Atmosphere` node, created when the scenario is
loaded, and inert until damaged**. Its conductance comes from an orifice area;
zero area means it does not conduct, and `network::compile_edge`'s existing
`conducts = alpha.is_finite() && alpha > 0.0` test already expresses exactly
that. The topology is fixed for the run, so the snapshot's shape is fixed too,
and repair is `area = 0` rather than graph surgery.

**The frozen-anchoring interaction is checked and clean, not open.** M5 left a
live defect: `network::prepare` freezes the anchored set at the seed compile,
which is stale for a PSV because its opening depends on the pressure iterate.
The discriminating question for a leak is the same one: *does the leak element's
`conducts` ever depend on the pressure iterate?* It does not — a leak's `alpha`
is a function of its **area**, which is a commanded quantity constant across a
solve. Even if the orifice reuses M5.4's choked law, so that `alpha` is
recompiled each iteration, `conducts` is sign-of-alpha and cannot flip. The seed
classification is therefore exact for a leak, and M6 does not drag the
frozen-anchoring un-defer along with it. **This must be re-checked, not
inherited, if a leak ever becomes pressure-actuated.**

#### Where the leak edge attaches — and why neither endpoint works

An edge to `Atmosphere` needs an **origin node**, and `PuncturePipe { edge, .. }`
names a *pipe*, which is an edge between two nodes. Fork B answered this by
splitting at the midpoint. Fork C must answer it too, and the two obvious
answers are both wrong:

- **They are not equivalent physically.** Hang the orifice on the upstream
  endpoint and it sees upstream pressure; on the downstream endpoint,
  downstream pressure. A real mid-pipe puncture sees neither. On a long line
  with real frictional `dP` that is the difference between a plausible leak rate
  and a wrong one.
- **Both are often illegal.** `network::validate_degrees` requires **exactly one
  inlet and one outlet** for `Pump`, `Valve`, `ReliefValve`, `Furnace`,
  `Cooler`, `Reactor` and `HeatExchanger` (`network.rs:126–144`). A leak edge
  leaving any of them makes `n_out = 2` and fails the check. A pipe's upstream
  endpoint is a pump or a valve constantly — `tank_pump_valve.toml` is nothing
  but — so an endpoint rule would forbid leaks on exactly the lines a game most
  wants to puncture, and would do it as a load-time `Err`.

**Resolution: split at LOAD, not at puncture.** A declared leak path splits its
pipe into two halves, joined by a `Junction`, with a dormant orifice edge from
that junction to `Atmosphere`. This is fork B's geometry with fork C's timing,
and the timing is what does the work: the split happens before tick 0, so the
topology and the snapshot's shape are fixed for the entire run, and the junction
is a node kind `validate_degrees` places no constraint on. It also gives the
leak the right pressure — the midpoint's — for free.

`PuncturePipe { edge }` therefore names the **original** pipe as the scenario
declared it, and the loader keeps the mapping from that name to the leak edge it
created. The command's JSON shape is untouched (see below).

#### Which pipes get one — and the real discriminator

Two candidates: auto-create a dormant leak path for **every** pipe at load, or
create one only where the **TOML declares** it.

The cost that matters is **not** per-tick CPU. `Snapshot` carries one
`EdgeSnapshot` per edge, and this repo's reference tests compare snapshot
contents; under the split-at-load resolution above, auto-create would also
*double every pipe* and add a junction per pipe, changing node counts, pipe
lengths and edge identity across every existing scenario. Measured rather than
predicted, as M5.1 requires: **exactly two tests assert `snapshot.edges.len()`**
(`furnace_reference.rs:237`, `isothermal_plant.rs:118`), and the determinism
gate is rerun-versus-rerun (`m1_acceptance.rs:160`) so it is immune. The churn is
therefore small in test count but large in *meaning* — every reference plant
would become a different plant.

That is what settles it: C's "fixed snapshot shape" advantage holds **within a
run** but not **across this change**, and the declared-in-TOML variant has zero
churn because no existing scenario declares a leak. The scenario author naming
where the plant may be damaged is a modest price, and a game that wants
puncture-anywhere can declare a path on every pipe in its own scenario file
without imposing that on the reference plants.

**M6.1 confirms this by construction** — a scenario that declares a leak and one
that does not, with the second's snapshot unchanged from today's.

### The blocker: `Atmosphere` back-feeds, and its composition is arbitrary

`energy.rs:733–737` gives an `Atmosphere` node the **first slate component** as
its composition, and says so honestly: nothing draws mass out of an atmosphere,
because leak edges run *into* it, so no gate can falsify the choice. The comment
names its own expiry — *"a decision the milestone that back-feeds from a leak
will have to make properly"*. **M6 is that milestone.**

A leak edge is pressure-driven like any other. Whenever the plant side falls
below `P_ATM`, the edge back-feeds, and that regime is reachable in this repo
today — `capacitive_vessel_reference.rs:310` blows a vessel down toward vacuum.
The arbitrary first component would then flow *into* the plant: mass-conserving,
finite, deterministic, and wrong — DESIGN §5's silent hazard, in the one place
the code already warned it would appear.

**This blocked shipping the leak until M6.1 chose among four, each with a gate
that runs in the back-feed direction. They are listed with their real prices —
one of which is not what it first looks like — and the choice is recorded after
the list:**

1. **Pin `Atmosphere` to a real air composition on the slate.** The honest
   model, and the only one under which air ingress means anything. It requires
   every slate that owns an Atmosphere to name an air-like component, and the
   FCC slate — `gas`, `gasoline`, `gasoil`, `coke` (`fcc_plant.toml`) — has
   none, so this is a slate change, not a code change.
2. **Make the leak a one-way orifice** — refuse reverse flow. **This is the
   option that looked cheap and is not.** A branch that conducts one way and not
   the other is a check valve: a derivative discontinuity at `dp = 0`, on an
   edge Newton iterates through, which is precisely the hazard §3a fork 4 exists
   for and spent a fork resolving for the choked law. Worse, it makes `conducts`
   depend on the **sign of the pressure iterate** — which voids the
   frozen-anchoring exemption argued two subsections above. That exemption was
   stated for an *area-driven* `alpha` and carries an explicit re-check trigger;
   **this option fires it.** Choosing it means smoothstep treatment plus a
   `known_defect_frozen_anchoring`-shaped problem, i.e. dragging in the M5
   deferral M6 was otherwise clear of.
3. **`Err` on sub-atmospheric back-feed through a leak.** Now the cheap one, by
   elimination. No C¹ break — the edge conducts symmetrically and the solve is
   unchanged; the refusal happens after, on a converged answer. Consistent with
   rule 5 ("a wrong answer is an `Err`, not a quiet number") and with §5's
   silent-hazard doctrine. The cost is a game that blows a leaking vessel down
   toward vacuum gets a hard error rather than a plausible picture.
4. **Back-feed the plant-side composition** — the leak breathes back what it
   just released, taking the edge's composition from the plant side in both
   directions. No arbitrary component, no C¹ break, and no error. But it is an
   invented rule with no source, and it is wrong in the one way that matters:
   air drawn into a hydrocarbon line is the interesting event, and this models
   it as nothing happening.

Whichever is picked, **a gate that only ever runs the leak in the *outward*
direction reproduces exactly the defect `energy.rs:735` predicted.** The gate
must drive the plant below `P_ATM` with a leak open.

#### Chosen (M6.1): option 3, `Err` on back-feed

The refusal lives in `network::finalize` — the one epilogue *both* fidelities
already go through, so the compiler enforces that neither can drift from it,
the same reasoning that puts `accumulation` and `heat_load` in single homes.

**It needs no tolerance, and that is a consequence of the element rather than a
choice.** An orifice branch is built with `beta = 0`, and
`QuadraticBranch::flow`'s sign is `sign(dp − beta)`, so a leak edge carries mass
inward **iff** its junction is strictly below `P_ATM` — the physical condition
itself, with no band of near-zero flows to argue about. That is why the early
return in `compile_edge` builds the orifice ALONE: give it an elevation head, or
compose it in series with a pipe, and the refusal keeps compiling while quietly
firing at the wrong pressure.

**The note's own reachability example expired while M6.1 was building on it.**
§3b argued sub-atmospheric was reachable by citing
`capacitive_vessel_reference`'s blowdown toward vacuum — but that plant is GAS,
and M6.1 refuses gas leaks (below), so the cited evidence no longer supports the
claim it was cited for. A liquid replacement (a sub-atmospheric sink pulling a
junction under `P_ATM`) was **built and its junction pressure read** before this
paragraph was written; the gate asserts that pressure is sub-atmospheric before
opening the hole, so it cannot go vacuous later. It also runs the plant with the
leak DORMANT first, which is what distinguishes "the refusal fires on the
damage" from "the refusal fires on this plant".

`energy::boundary_composition`'s `Atmosphere` arm keeps its arbitrary value and
does **not** become an `Err` or an `unreachable!()`. It is called for every
inertial node on every tick, including in a plant whose declared leak path is
dormant and where nothing back-feeds at all, so a panic there would be a rule-5
violation reachable by the *undamaged* case. The value is now unreachable by
construction rather than merely untested, and the comment says so.

### The split changes the answer — by the regularization (M6.1, measured)

**"Declared-in-TOML wins at zero churn" is true of every scenario in this repo
and false of any scenario that adds a declaration**, and the difference is worth
stating because the argument above does not imply it.

The split is exact in real arithmetic: `k ∝ L` and `β = ρ·g·Δz` both add back
over two halves, and halving is exact in binary. What does *not* compose is the
**regularization**. A single branch computes `Δ/√(Δ + ε)`; two identical halves
in series each see `Δ/2` and give `Δ/(√k·√(Δ + 2ε))`, so a split plant behaves
as though `ε` were doubled and runs slightly slower:

```text
1 − √((Δ + ε)/(Δ + 2ε))  ≈  ε/(2·Δ)  =  1.25e-6   at leaking_line's ~4.0 bar
```

Measured 1.216e-6 after 50 ticks. So **declaring a leak moves that pipe's flow
in the seventh significant figure even while the leak is dormant** — far below
any fidelity this model claims, and far above bit-identity, which is the part
that matters: a golden snapshot taken before a declaration will not reproduce
after one. `splitting_a_pipe_does_not_change_the_plant_it_describes` asserts the
gap's sign and size against the derivation rather than hiding it in a tolerance.

### Two hazards this note did not identify, found while building

**A leak on a COLUMN's feed or draw — refused at load.**
`network::is_column_draw_edge` recognises a draw by its two endpoints, and
`edge_flows` guards a draw's flow to zero on the strength of it, because a
draw's flow is *prescribed* (`splitᵢ·ṁ_feed`, written post-sweep) and not
pressure-driven at all. Split that edge and neither half matches any more, so
the guard silently stops applying and the draw becomes a pressure-driven number
— §5's silent hazard, reached by a scenario line that reads perfectly
reasonably. The feed is refused alongside it: `validate_degrees` would reject a
split feed anyway (a column is 1-in-N-out), but with a message about edge
degrees that names the wrong cause, so the gate checks *which* refusal fires.

**A leak on a GAS line — refused, at two doors.** The orifice law shipped here
is incompressible, and a hole venting a pressurised gas line to atmosphere is
choked over essentially its whole useful range (~0.53 of absolute inlet pressure
for a diatomic gas), so Torricelli would overpredict the escape rate — finite,
deterministic and wrong, in the one number a damage model exists to report.
This is M5.4's refusal of the incompressible law on a compressible fluid,
recurring one unit along. Two doors, per the `require_gas_valve_x_t` precedent:
the loader names the scenario file, and `compile_edge` catches a plant built
directly — which is not hypothetical, since M6.1 added a leak arm to the
invariant generators (they build a `PlantGraph` and never call `build_engine`)
and the compile-time door now fires on 81 of 400 generated samples. It un-defers
with an orifice `x_T` and a published anchor to size it against.

**`ORIFICE_CD = 0.61` is a bracket, not a transcription.** Every standard
treatment of a sharp-edged orifice lands in 0.60–0.62, and it is quoted at that
confidence rather than to three figures from a table this repo has read: a hole
punched by damage has no defined edge geometry, so more precision would be false
precision about the damage rather than about the arithmetic. It is deliberately
not a scenario parameter — the leak rate is linear in `Cd`, so nothing in the
model can discriminate 0.60 from 0.62, and a knob no gate can settle invites
tuning the plant through it. Un-defers with a plant needing two leaks of
different geometry.

### Two contract decisions, stated once

- **`Command::PuncturePipe { edge, area }` keeps its exact JSON shape.**
  `Command` is `#[serde(tag = "cmd")]` (`snapshot.rs:11`) and therefore a
  frontend contract. Under fork C, `edge` names the **pipe** and the engine
  routes the area onto that pipe's dormant leak path. Renaming it to
  `SetLeakArea` would break the contract to describe an implementation detail;
  "puncture pipe" still names the physical act correctly.
- **`leak_mass_flow` is reported on the punctured pipe, not on the leak edge.**
  It is a translation decision, and the pipe is the thing a frontend draws a
  spray from. The leak edge remains in the snapshot as an ordinary edge with its
  own flow; the field on the pipe is the convenience view. **Two fields carrying
  one quantity is how they drift**, so their agreement is a one-line gate, not
  an assumption — and under the split-at-load resolution the pipe in question is
  the upstream half, which is where a frontend that still draws the original
  pipe will look.

### What M6 does not attempt here

Back-feeding **composition** from the atmosphere into the plant is out of scope
under all three options above — option 1 makes it well-defined, options 2 and 3
make it impossible. Air ingress as a modelled phenomenon (and therefore
combustion) un-defers with a slate carrying air and a reason to burn it. Fire
remains a heat source on a node, as §2 has said since M1.

## 4. Streams and pseudo-components

```
Stream { mass_flow: KgPerSec, temperature: Kelvin, pressure: Pascal,
         composition: Composition }
Composition = normalized mass fractions over the scenario's component slate
PseudoComponent { name, tb: Kelvin, mw: KgPerMol, density: KgPerM3, cp: JPerKgK }
```

The slate (list of pseudo-components) is defined per scenario; water-only
scenarios have a slate of one. Crude slates: 10–30 TBP cuts. Mixture
properties are mass-fraction-weighted (adequate at this fidelity; document
exceptions where they matter, e.g. mixture density uses volume-fraction
weighting).

## 4a. Energy transport (M2)

The temperature field is **mixed**, and naming the two kinds of node is what
makes it tractable. The distinction is *thermal inertia*, not unit type:

- **Inertial nodes** — `Tank`, `Source`, `Sink`, `Atmosphere`. They carry a
  temperature. A tank integrates it as a slow state (§1 step c); the reservoirs
  hold it fixed. Within a tick, all four are *boundary conditions*, read at
  their start-of-tick value.
- **Zero-volume nodes** — `Junction`, `Pump`, `Valve`, `Furnace`, `Cooler`, and
  each side of a `HeatExchanger`. No inventory,
  so temperature is not a state at all: it is **algebraic**, the instantaneous
  enthalpy-weighted mix of the inflows,
  `T = T_REF + Σ(ṁ_in·cp_in·(T_in − T_REF) + Q) / Σ(ṁ_in·cp_in)` — the first
  law for a point with no accumulation.

**The `Furnace` is entirely that formula's `Q` term** (M2.2). It adds no
physics: a fired heater's tube inventory is negligible against its duty, so its
outlet is algebraic, `T_out = T_in + Q/(ṁ·cp)`, and it is hydraulically a
pass-through at this fidelity — the tube-side pressure drop belongs to the
connecting pipes' resistance rather than to a device characteristic. Its duty
is the heat delivered *to the process fluid*; combustion efficiency and a
firing-rate model are a later fidelity step.

`duty` is a field of `NodeKind::Furnace`, deliberately **not** stored in
`Node::heat_input`. That field is the damage model's hook, and the two sum in
`energy::heat_load`: a fire on a furnace must *add* to its duty, not overwrite
the operator's setpoint. Sharing storage would make a plant run **colder**
during a fire — a wrong answer that looks entirely plausible in a snapshot.
Like `Pump`/`Valve`, a furnace is restricted to one inlet and one outlet edge,
but for a different reason: theirs is the hydraulic fold-at-source convention
(F6), the furnace's is that "the stream through it" only names something when
there is one process stream. Nothing numerical forces it — the mixing formula
would average N inlets happily — so a branched furnace is rejected because the
author meant something the model does not represent.

**The `Cooler` is the same `Q` term with the other sign** (M2.2) — structurally
the furnace's mirror in every respect, and it exists as a separate unit rather
than as a furnace with a negative duty. Both store `duty` as a non-negative
**magnitude**; direction is a property of the *unit*, applied in the one place
that owns the convention, `energy::heat_load`. The alternative — one unit, signed
duty — puts the physics in the sign of a number in a TOML file, where a typo
turns a heater into a chiller and the plant still runs. With two units the intent
is in the name, a negative duty means nothing, and both the loader and the
`Set*Duty` commands reject it. The payoff comes free at `heat_load`'s sum: a fire
on a cooler *fights* the cooling instead of replacing it, exactly as it *adds* to
a furnace's duty.

**Over-cooling is an error, not a clamp.** A duty exceeding the sensible heat its
stream carries above 0 K drives the mix below absolute zero. The result is
*finite*, so no NaN/Inf check sees it, and it would propagate downstream as an
ordinary temperature — measured at **−508 K** on `cooler_chiller.toml` at 200 MW
with the check removed. The check is stated generally rather than as a
cooler-specific case: a negative absolute temperature is broken whatever produced
it. Clamping to 0 K was rejected — it reports a plausible number instead of the
one that was asked for, the silent wrong answer this project treats as worse than
a failure.

`energy::checked_temperature` is the single owner of that rule, because there is
more than one path that computes a temperature and **a cooler is not the only
lever that can drive one sub-zero**. Zero-volume nodes mix their inflows in
`mix_inflows`; tanks integrate their thermal inventory in the engine's unit-dynamics
step, on a path the mixing guard never watched. A large enough net heat *sink* on
a tank integrates the same finite, forbidden number. Both paths route through the
one checker so they cannot drift apart on what "impossible" means, and a new heat
term arrives already guarded rather than reopening the hole on whichever path is
newest.

Nothing reaches the tank path *today*: a tank carries no duty, mixing cannot fall
below its coldest inflow, and `Command::SetHeatInput` refuses a negative fire (see
below). That guard is therefore cover held in advance, for the **ambient exchange**
that will put a signed `Q` straight onto the tank balance — the one heat term that
is legitimately signed, because a vessel warmer than its surroundings loses heat
and a colder one gains it. Writing the check before that term rather than after it
is the whole point of consolidating the two sites.

**A fire only heats.** `Command::SetHeatInput` refuses a negative power for the
same reason `Set*Duty` refuses a negative magnitude: `heat_input` is the damage
model's hook, and there is no damage that chills a unit. Allowing one would be a
second, undeclared way to spend heat, bypassing the `Cooler` the model added for
that job. Zero stays legal — it is "the fire is out". Every *net* heat sink is
thus the property of a unit that declares itself one, ambient exchange included.

The tank guard sits *inside* the minimum-thermal-mass branch: a nearly-empty tank
has no meaningful temperature and holds its last valid one, so it has no computed
value to check and must not trip the guard.

An edge's stream temperature is its **upwind** node's temperature, selected by
the sign of the solved flow (donor-cell). Flow sign, never edge direction: the
hydraulic solver produces reverse flows routinely, and upwinding off the graph's
arrows would silently transport heat the wrong way.

Zero-volume nodes must therefore be resolved upstream-first — a topological sort
over *this tick's flow directions*, not the graph's edge directions. Kahn sweep,
lowest node id first, so the order is a function of the graph alone (rule 3).

**Why an acyclic sweep is sufficient, not a shortcut.** The ordering fails to
exist only if a recycle passes through zero-volume nodes *exclusively*. Any tank
or reservoir in the loop breaks it: its temperature is a start-of-tick constant,
so the dependency chain terminates there. A zero-volume-only recycle is a
genuine simultaneous system (every temperature defined in terms of the others);
it is rejected with `SimError::Numerical` naming the stuck nodes, rather than
resolved in some arbitrary order that would look plausible and be wrong. No
scenario in the workspace builds one. If a real plant needs it — a recycle loop
with no vessel anywhere in it — the fix is a linear solve over the loop, and it
should arrive with the scenario that motivates it, not before.

**Enthalpy datum.** `h = cp·(T − T_REF)`, `T_REF = 273.15 K`. Every flux in the
engine goes through `energy::enthalpy_flux`, so the datum cancels exactly as
long as mass balances. It is deliberately non-zero: a 0 K datum makes `h = cp·T`
and would hide any path that dropped the reference entirely.

**Accuracy is inherited from the hydraulics.** Enthalpy cancels at a junction
only as exactly as *mass* balances there, and the flow solver stops at a finite
residual (Newton: `1e-8 + 1e-8·throughput` kg/s). That ε leaves the junction
carrying `cp·ε·(T − T_REF)` W of unbalanced enthalpy, so the relative energy
error settles at ≈ ε/ṁ ≈ 1e-8 — measured at 9.4e-9. Energy conservation cannot
be made tighter than the mass conservation it rides on; tightening I6 means
tightening the flow solver first.

**The `HeatExchanger` is the first unit that does not fit the per-node sweep**
(M2.2). Every heat term before it — a fire, a furnace's duty, a cooler's — is
imposed on one node from outside, so `heat_load` can answer "how much heat
enters this node" by looking at that node alone. An exchanger's heat comes from
*another stream*, and how much there is depends on that stream's temperature.
It is the first place two nodes' temperatures are computed together.

**Representation: two nodes, hydraulically independent, thermally coupled.**
Each side is an ordinary zero-volume 1-in/1-out pass-through, exactly like a
furnace, and **the flow solver is unchanged and does not know the two are
paired** — no heat-carrying edge, no four-port node. Both alternatives fight the
model's central premise that an edge is a pipe carrying mass; a thermal link
carries none, and giving one node four ports would make "the stream through it"
ambiguous everywhere that phrase is currently load-bearing.

Duty at ΔT-effectiveness fidelity, one *signed* quantity computed once:

```text
C_a = ṁ_a·cp_a,  C_b = ṁ_b·cp_b,  C_min = min(C_a, C_b)
Q   = ε·C_min·(T_a_in − T_b_in)
T_a_out = T_a_in − Q/C_a        T_b_out = T_b_in + Q/C_b
```

Three properties of that form are load-bearing, and each is a bug if broken:

- **Energy conserves by construction.** One `Q`, subtracted from one side and
  added to the other, for *any* ε and any capacity rates: `−Q + Q = 0`
  identically. Two independently computed effectiveness terms would be the
  natural-looking alternative and would break I6 for a reason no reference test
  would localize. Same discipline as `heat_load`: one owner of the sign.
- **Neither side is "the hot one".** The labels are for humans reading TOML; the
  physics is in the sign of `(T_a_in − T_b_in)`, so a side fed hotter than its
  partner simply reverses `Q` with no second code path. This is the direct
  analogue of ambient exchange's `T_ambient − T_node`, and it is reachable:
  the `Cooler` above can chill a stream below the one it later meets.
- **`C_min`, not `C_max` or either side's own C.** With `C_min` and `ε ≤ 1`, the
  outlet temperatures cannot cross — the second law holds automatically rather
  than needing a check. Using `C_max` lets the cold outlet exceed the hot inlet.
  This is invisible when `C_a = C_b`, which is why the reference case deliberately
  gives the two sides **unequal** capacity rates.

**The sweep resolves a pair as one vertex.** Both outlets depend only on the two
*inlets*, so there is no simultaneous solve inside an exchanger — but side A
reads `T_b_in`, which is **not one of A's inflow edges**. A per-node Kahn sweep
would mark A ready as soon as A's own inflows cleared and mix it against a stale
partner inlet, which converges, serializes and looks entirely plausible. So each
pair is merged into a single sweep vertex: ready when the *union* of both sides'
zero-volume upstream dependencies clears, then both sides resolved together.

That merge also gets the pathological case right for free. If one side's outlet
feeds the other's inlet through zero-volume nodes only, the merged vertex can
never become ready and the existing zero-volume-recycle rejection fires — which
is correct, because that plant *is* a genuine simultaneous system, exactly the
kind the sweep declines to guess at.

**ε is a property of the pair, not of either node**, so it is stored once, in a
coupling list on `PlantGraph`, rather than duplicated into both `NodeKind`s where
the two copies could disagree. The node kind carries only the side's *identity*
(what makes `is_zero_volume` and `boundary_temperature` recognize it). This is
the same instinct that made `Furnace` and `Cooler` separate units instead of one
signed duty: put the invariant somewhere it cannot be violated, rather than in a
convention someone has to remember.

ε ∈ (0, 1] is validated at the **loader, which is its only entry point**. This
is a deliberate departure from the `Furnace`/`Cooler` precedent of guarding both
loader and command, and the difference is what the quantity IS: a duty is an
operator setpoint that moves during a run, so `Set*Duty` exists and needs its own
guard, whereas ε is fixed hardware — surface area and geometry — that nothing
changes mid-run. Adding a `Set*Effectiveness` command purely to have a second
place to validate would be dead code with no caller, and a guard on a path
nothing takes cannot be falsified. If a fouling model ever makes ε time-varying,
it arrives with its own entry point and its own guard.

No NTU, no LMTD, and no counter- versus co-current distinction at this fidelity:
a constant ε from the scenario file is what "simple before complex" means here,
and the geometry-dependent models are a later trait implementation, not a
refinement to bolt onto this one.

**Ambient exchange is not "heat loss", and the naming matters** (M2.2). The
driving force is `T_ambient − T_body`, so ONE signed term must heat a body
colder than ambient and cool one hotter:

```text
Q_ambient = UA·(T_ambient − T_body)        [W, signed]
```

A one-directional "loss" would be wrong for a chilled tank on a warm day — and
with the `Cooler` and the `HeatExchanger` in place, that is now a reachable
plant state rather than a hypothetical. The term therefore lives in ONE function
whose sign falls out of the subtraction, consumed by every body that has an
ambient boundary. No `if colder` branch, no second code path, and no sign
convention of its own to remember — the same move that made `heat_load` the sole
owner of the duty sign.

`UA` [W/K] lumps the overall heat transfer coefficient with the exposed area,
because at this fidelity nothing distinguishes them: no geometry, no wind, no
insulation model, no radiation. It defaults to ZERO — a perfectly insulated
body — so every existing scenario is bit-identical after the change, and
`isothermal_plant.rs` keeps testing exactly what it tested before. A default
that silently started leaking heat would make that flat line a lie.

**Tanks and pipes are NOT one change, and are deliberately separate boxes.**
The single ROADMAP line covering both hid a real asymmetry:

- **A tank is additive.** Its temperature is already an integrated state with a
  `Q` term in `d(m·u)/dt = Σ ṁ·h + Q`; ambient exchange is one more contribution
  to that `Q`, guarded on arrival by `checked_temperature` — which §4a already
  records as cover placed in advance for exactly this term. Nothing structural
  moves.
- **A pipe is not.** Today an edge's stream temperature IS its upwind node's
  temperature, carried along unchanged; that identity is the core of the M2.1
  upwind sweep. A pipe exchanging heat with ambient has an outlet that differs
  from its inlet, which the model has no way to express — it needs a per-edge
  TRANSFORM:

  ```text
  T_out = T_ambient + (T_in − T_ambient)·exp(−UA/(ṁ·cp))
  ```

  the analytic solution for plug flow along a pipe, not an Euler step, so it
  stays stable and correct at any `UA/(ṁ·cp)` instead of overshooting past
  ambient on a long tick. It gets its own box and its own note — see
  **Ambient exchange for pipes** below, which corrects this bullet's original
  claim that the transform is a step running *between* the sweep and transport.

Tanks land first, alone, because they are testable alone: heat a cold tank,
cool a hot one, and check both against `T(t) = T_amb + (T₀ − T_amb)·exp(−UA·t/(m·cp))`
in the constant-mass limit.

**Tanks are done** (the pipe box remains open). The term lives in
`energy::ambient_exchange` and is summed by `heat_load`, so the engine's tank
integration receives it without a line changed there. One thing the note above
did not anticipate: the exponential is the *continuous* solution, and the tank's
`Q` is integrated by explicit Euler like every other slow state, so the discrete
result is `(1−α)^N` with `α = UA·dt/(m·cp)` — close to `exp(−αN)` but not equal.
That gap is Euler truncation, and it sets the reference test's tolerance
(1e-3 K at one time constant, against 1e-9 for a single exact step). It also
bounds stability: the term oscillates and diverges once `α > 2`. For a tank that
is unreachable — `α` is ~1e-6 at refinery scale — so it is documented in
`Engine::tick` rather than guarded. For a PIPE it is not unreachable at all,
since `ṁ·cp` can be small, which is the second and independent reason that box
insists on the analytic transform rather than an Euler step.

### Ambient exchange for pipes (M2.2) — specified before building

**The change is not "one more heat term". It is that an edge stops being
isothermal.** Every heat term so far has been imposed on a *node*; this one acts
along an *edge*, and that breaks an identity the whole M2.1 transport rests on:
an edge's temperature IS its upwind node's, carried along unchanged. With a `UA`
on a pipe, the enthalpy that *leaves* the upstream node is at the inlet
temperature and the enthalpy that *arrives* downstream is at the outlet
temperature, and the difference is the heat the pipe traded with ambient. An
edge therefore has TWO temperatures, and the single `Stream::temperature` field
can no longer answer "what temperature does this edge carry" without first
asking "at which end".

The transform is the analytic plug-flow solution:

```text
T_out = T_AMBIENT + (T_in − T_AMBIENT)·exp(−UA/(|ṁ|·cp))
```

Three details of that form are load-bearing:

- **`|ṁ|`, not signed `ṁ`.** The sign of the flow already did its job upstream —
  it selected which end is the inlet (donor-cell). The denominator is a capacity
  *rate*, a magnitude. Feeding a signed `ṁ` flips the exponent on every reverse
  flow, turning decay into growth: the pipe would run *away* from ambient, and
  it would do so only on reversed edges, which the reference plants do not have.
- **Analytic, not Euler.** `exp` cannot cross ambient however large the exponent
  gets; a `1 − UA/(ṁ·cp)` step overshoots past it and then diverges. The tank
  box tolerated explicit Euler because `α = UA·dt/(m·cp)` is ~1e-6 at refinery
  scale and the instability is unreachable. Here the denominator is `ṁ·cp`, a
  *flow* rather than an inventory, and a nearly-closed valve drives it toward
  zero — so the unstable regime is not merely reachable, it is one throttle away.
- **`UA` defaults to 0**, exactly as it does for tanks: `exp(0) = 1`, the
  transform is the identity, and every existing scenario stays bit-identical.
  `isothermal_plant.rs` keeps meaning what it meant.

**Zero flow is a guard, not a limit.** At `ṁ = 0` with `UA = 0` the exponent is
`0/0 → NaN`, and a NaN temperature is *not* caught by the mixing guard — it
would reach step 4's finiteness check as a bare "non-finite" with no diagnostic
naming the pipe. This is not hypothetical: a closed valve produces zero-flow
edges today (`newton_reference.rs::closed_valve_blocks_flow_both_ends_anchored`).
Below a flow threshold the transform returns `T_in` unchanged, which is also the
physically right answer at this fidelity — a stagnant pipe carries no enthalpy
either way, the same reasoning that lets transport pick `from` arbitrarily at
exactly zero flow. It is the same limitation as heat into a stagnant zero-volume
node: with no throughput there is no stream to carry the heat, and modelling it
honestly needs pipe-wall thermal mass, which is a fidelity step.

**The sweep does not get harder, and this is worth stating because the bullet
above originally implied it would.** The transform needs only that edge's own
`ṁ` and `UA`, both known before the sweep starts — it introduces no dependency
on any temperature the sweep has not already resolved. The topological order is
therefore untouched. What changes is a single *value*: `inflow_totals` reads its
upstream node's temperature and must instead read the transformed edge outlet.
"Interleaved into the sweep" is accurate; "a step between the sweep and
transport" was not, because a pipe's outlet is an input to the downstream node's
mix and so cannot run after the mixing that consumes it.

**One owner: "the temperature entering node N from edge E".** The transform is
read from two places — `energy::inflow_totals` (the sweep's mixing) and the tank
integration in `Engine::tick` step 3 — and both go through one function that
finds the upwind end and applies the transform only when `N` is the *downstream*
end. Applying it ad hoc at each site is the same mistake the `HeatExchanger`
note rejected in its "two independently computed effectiveness terms" form: two
copies of a rule that must agree, with nothing forcing them to. With `UA = 0`
the helper returns the same value at both ends, which is why the change is
bit-identical by construction rather than by measurement.

**This retires a property the tank loop currently documents.** That loop needs
"no in/out branch" today because an outflow edge is upwind of the tank and so
already carries the tank's own temperature, making the signed flux subtract
exactly the enthalpy that leaves. That was never a fact about tanks — it was a
*consequence* of edges being isothermal, and it does not survive them. Routed
through the helper it becomes true again for the right reason: at the tank's
outflow edge the tank IS the upwind end, so no transform applies and the flux is
unchanged; at an inflow edge the tank is downstream and gets the transformed
value. The comment must say which of those it is relying on.

`Stream::temperature` is then consumed by nothing in the engine but transport's
own write — only tests and the snapshot read it — so which end it reports
becomes a deliberate *display* choice rather than a correctness one. It reports
the **outlet**: that is what the downstream node receives, and it is the only one
of the two that a snapshot cannot reconstruct from the upwind node's temperature.

**I6 needs no new term, and deliberately does not get one.** The pipe's ambient
`Q` sits on no node, so a UA-bearing pipe would open a hole in the telescoping
sum that I6 checks. It does not, because the proptest generators leave pipe `UA`
at its default 0 — every generated edge stays isothermal and the enthalpy
cancels exactly as before. This is the same containment that keeps furnaces,
coolers and exchangers out of those generators, but for a better reason than the
furnace's: nothing is *wrong* with a UA-pipe in an energy balance, it simply
needs a term the invariant does not currently carry. Folding a fixed UA-pipe
into an I6-style case is the cheap way to close that later; it is not this box.
What this box does instead is pin the term with reference cases, **one of which
asserts the energy closes** — inlet-minus-outlet enthalpy equals the pipe's
ambient `Q` — so "the heat goes somewhere accounted for" is tested rather than
asserted in prose.

**The `HeatExchanger` composes for free**, and saying so explicitly is cheaper
than rediscovering it. An exchanger side is an ordinary zero-volume node
resolved through `inflow_totals`, so it reads its inlet as a transformed edge
temperature without any change to the coupling. The pair merge is unaffected:
the feeding edge's upwind node still resolves before the pair vertex becomes
ready, because the transform added no dependency.

**Tests, and what would falsify them.** The discriminating case — the analogue
of the exchanger's pair-merge ordering test — is a pipe with `UA` **between two
zero-volume nodes**, where mixing the downstream node against the raw upwind
node temperature instead of the transformed outlet gives a visibly different
answer. That is the gate that proves the transform is inside the sweep rather
than after it; a pipe feeding a tank cannot prove it, because the tank's
integration would mask the ordering. Alongside it: a single-pipe reference
against the closed-form exponential, at **round-off tolerance (~1e-9), not the
tank box's 1e-3** — the contrast is the point, since this transform is analytic
and has no truncation error to budget for; an overshoot case at large
`UA/(|ṁ|·cp)` where an Euler step would cross ambient and the analytic form
provably cannot; the zero-flow/closed-valve case reaching a finite temperature;
and the energy-closure case above.

Each must be falsified before it is trusted, per the boxes above: at minimum,
signed `ṁ` in place of `|ṁ|` (should fail only on a reversed-flow case — if
nothing fails, no test covers reverse flow and one is missing), the raw-upwind
mutation (should fail the two-zero-volume-nodes gate and *only* it), and
removing the zero-flow guard (should fail with a NaN, and if the plant instead
runs, the guard is being reached by some other path and the test is vacuous).

**Known limitations at this fidelity** (each deliberate, none accidental):

- ~~**No pump work or valve throttling heat.**~~ **RETIRED — built in M5.1.**
  Kept here rather than deleted, because the deferral→measurement→build loop is
  the point. At M2 both dissipated into the stream in reality while a
  pass-through device copied its inlet temperature to its outlet; the reference
  pump's rise is ~0.02 K, far below the model's accuracy and below any tolerance
  a reference test could be falsified against, so building it in M2 would have
  meant a feature with no gate that earns its place. The deferral named a
  MEASUREMENT as its re-opening condition, and that measurement — taken on
  `tank_pump_valve` itself in §3a — put the *valve* term at **0.094309 K**, 94×
  the 1e-3 K tolerance the ambient tests carry, on the reference plant rather
  than a contrived one. The bullet was right about the pump and wrong about the
  plant: throttling was already large enough here.
  Friction now lands as `Φ = α·Q|Q|·Q` on each edge's outlet transform (§3a). A
  pass-through device consequently still reports its INLET temperature — its
  heat appears on the edge leaving it, by fold-at-source — which is a display
  choice, not the old omission. Pump *efficiency* heating stays deferred, for a
  different reason (§3a): `η` is a parameter with one possible value in this
  repo, so no gate could tell a right value from a wrong one.
- **Heat into a zero-volume node with no throughput is dropped.** It has no
  thermal mass to store it and no stream to carry it away. A fire against
  stagnant inventory belongs on a `Tank`; this is the one case where the engine
  does not conserve energy, and it is a gap in the model rather than slack the
  invariant tests are widened to tolerate.
- **An empty tank holds its last temperature.** `T = T_REF + E/(m·cp)` is
  singular at `m = 0`, and the Euler mass update can overshoot into the clamp,
  at which point mass and energy have both stopped being conserved and the ratio
  is meaningless rather than merely imprecise.
- **A furnace or cooler with no throughput drops its duty**, by the stagnant-node
  rule above. Firing a heater with no flow through it is a real and dangerous
  operating state (tube damage), and the model is silent on it rather than
  wrong about it — representing it needs tube metal as a thermal mass, which is
  a fidelity step, not a bug fix. This is also why neither unit appears in the
  proptest generators: a stagnant one would "violate" energy conservation for a
  documented model-gap reason rather than a defect.
- **A cooler has no coolant-temperature floor.** Its duty is fixed, so it will
  cool a stream past any coolant temperature, past ambient, and (absent the
  guard above) past 0 K. Only the last is detectable without modelling a coolant.
  Cooling to a realistic approach temperature is the `HeatExchanger`'s job — a
  fidelity step, not a missing bound here. That unit has now landed, so a plant
  that needs the floor can have it; the `Cooler` keeps its fixed duty on purpose.
- **An exchanger side with no throughput transfers nothing**, by the same
  stagnant-node rule. That one is physics rather than a gap: with no flow there
  is no capacity rate to transfer against, and the running side passes straight
  through. Exchangers are absent from the I6 proptest generators for a
  different reason than furnaces and coolers, worth stating so it is not
  mistaken for a conservation gap: a both-sides-flowing exchanger conserves
  energy BY CONSTRUCTION and would strengthen I6 rather than strain it. What
  keeps it out is generator complexity — random valid networks would have to
  emit paired sides and a coupling table — so this is a cost decision, not a
  model limitation. A fixed exchanger in an I6-style case is the cheap way in
  if it is ever wanted.
- **The exchanger is ΔT-effectiveness only** — no NTU, no LMTD, no
  co-/counter-current distinction, and ε is a constant rather than a function of
  flow. Consequently the model has no opinion on outlet ORDERING: a cold outlet
  above the hot outlet is ordinary counter-current behaviour and is not flagged.
  What ε ≤ 1 with `C_min` does guarantee is the bound that matters — neither
  stream passes the other's inlet.
- **No heat loss to ambient** yet — the rest of M2 (see ROADMAP).

**`ThermoModel` is still a reserved slot.** Transport uses constant-property
`cp` off `Composition` (ideal mass-fraction mixing), which is exactly what the
trait's doc prescribes: extend when a consumer actually needs a property, not
before. Constant-cp water does not. It takes over when T-dependent or non-ideal
properties arrive — at which point `Composition`'s `mixture_*` helpers delegate
to it, and the symmetry premise of the mixing reference test needs rechecking
(equal-temperature legs stop carrying equal flows once density varies with T).

## 5. Reactions and separation

- **Reactor (complex):** lumped kinetics, e.g. FCC 4-lump
  (gasoil → gasoline → gas + coke), Arrhenius rates, RK4 fixed substeps
  inside the unit tick. Source: Weekman & Nace-style lumping; cite the exact
  parameter set used in the code.
- **Reactor (simple):** fixed conversion table per (T-band, feed cut) —
  a lookup, no ODEs.
- **Column (complex):** stage-by-stage flash cascade at quasi-steady state,
  solved per tick (later milestone; needs K-values — start with Raoult +
  Antoine per pseudo-component derived from Tb).
- **Column (simple):** fixed cut-point splitter: assigns each
  pseudo-component to a draw by boiling range, with a smearing parameter for
  imperfect separation.

### Simple column (M3.2) — specified before building

The roadmap opened M3.2 with a fork — **(A)** cut points fixed and draw rates
following, versus **(B)** draw valves fixed and cut points emergent — and with a
flagged risk: (A) appeared to need a *prescribed-flow branch*
(`flow = setpoint`, `dflow/dp = 0`) in a solve that is entirely pressure-driven,
which could leave the Jacobian singular. **The verdict is (A), and the risk turns
out to be avoidable rather than merely survivable: the prescribed-flow branch is
not needed at all.** The reasoning below is the point of this note; the shape it
lands on is a consequence.

**Why (B) loses, in one line.** Per-draw composition would depend on the
hydraulic solve, so the reference number this unit needs (see "What the tests
must pin") becomes hydraulics-dependent and effectively unpinnable. That is
decisive on its own.

**Well-posedness — the dichotomy that kills the naive (A).** Suppose each draw
is prescribed from the *previous* tick's feed, `sᵢ = yieldᵢ · ṁ_feed_prev`, as
the roadmap sketched. There are only two ways to treat the column node, and both
fail:

- **Column free (zero-volume, pressure an unknown).** Its residual is
  `R_C = ṁ_feed(P_C) − Σᵢ sᵢ = 0`. Since `Σᵢ yieldᵢ = 1`, that reads
  `ṁ_feed_now = ṁ_feed_prev`. One equation, one unknown, and `ṁ_feed` is
  monotone in `P_C`, so it *converges* — to a plant in which the feed is frozen
  at its initial value forever. Close a valve upstream, drain the supply tank:
  `P_C` slides to absorb it and the feed never moves. A converging, mass-
  conserving, deterministic, thoroughly wrong plant.
- **Column fixed-pressure (zero-volume).** There is then no mass-balance
  equation at the column at all, so `ṁ_feed_now − Σᵢ sᵢ` is created or destroyed
  inside a zero-volume node. Per-component conservation breaks and I7 fails.

The lag is not a "limitation to state"; it is the defect. What both branches
show is that the *total* through a column must stay hydraulically determined,
and only the *split* may be composition-determined.

**The design.** Pin the column's pressure and take the split from the
**current** solve:

- The column is a **fixed-pressure, zero-volume** node — `NodeKind::Column`
  carries an operating `pressure`, and `network::fixed_pressure` returns it,
  exactly as `Source`/`Sink`/`Tank` do. This is not a modelling fudge to dodge
  the solver: real columns run on pressure control, and the overhead pressure is
  the operator setpoint that sets the whole cut structure.
- The feed edge is an **ordinary pressure-driven edge** into that fixed node.
  Nothing about it is new, and the plant therefore acts on the column: a
  throttled feed valve or a draining supply tank lowers `ṁ_feed`, which is
  precisely what the frozen-feed branch above could not do.
- Draw flows are computed **after** the hydraulic solve as
  `ṁ_drawᵢ = splitᵢ · ṁ_feed_now`, with `Σᵢ splitᵢ = 1` by construction.
  The column is therefore mass-neutral **identically, every tick** — no holdup,
  no lag, and no residual to leak. (M3.1's composition staleness in the
  hydraulic density is unchanged and unrelated.)

**Why this needs no new solver machinery, and where the real change is.** Draw edges
run column (fixed) → product tank (fixed). `assemble` only accumulates residual
and conductance for endpoints present in `idx`, which holds *free* nodes only,
so a fixed→fixed edge contributes nothing to the Jacobian. The singular
zero-derivative branch the roadmap feared never enters the system — the risk is
not solved, it does not arise.

The change lands one layer out, in `network::edge_flows`, which would otherwise
evaluate a draw edge as `ρ·branch.flow(dp)` and report a *pressure-driven*
number for an edge whose flow is prescribed. That is the hazard to guard, and it
is a silent one: the bogus flow is finite, deterministic, and mass-conserving at
both endpoints (both are infinite reservoirs), so nothing downstream complains.

**Correction from building it — `edge_flows` only GUARDS; the authoritative
draw flow is written post-sweep.** This box first read as if `edge_flows` would
*compute* `splitᵢ · ṁ_feed`. It cannot, and the reason is per-component
conservation, not convenience. The split needs the feed composition; the feed
composition a column consumes this tick is the sweep's *resolved* one (the feed
edge delivers `ṁ_feed · f_feed_fresh` into the column — you cannot make the
intake stale without breaking the sweep); and per-component balance at the
fixed, zero-volume column,

```
Σᵢ ṁ_drawᵢ · comp_i,c = ṁ_feed · f_feed_fresh,c ,
```

holds *only if the flow-split and the composition-split are built from the same
feed composition*. Since the intake is fresh and the fresh feed composition does
not exist until after the sweep, the draw flow cannot be finalized inside the
solve. Splitting `edge_flows`'s *stored* (one-tick-stale) composition — the
tempting literal reading — unbalances every component on any feed transient
(worst on tick 0, where the stored value is the stagnant seed). So the split
runs in **one** place, `energy::column_separation`, called twice from the same
resolved feed: by `edge_composition_at` for the draw *compositions*, and by
`Engine::tick` (after the sweep, before transport) for the draw *flows*.

`edge_flows` therefore only **guards**: it reports every draw edge as zero. That
kills the bogus pressure-driven magnitude, and — the part the original framing
missed — it also stops the *wrong-sign* case. When a product tank fills above
the column pressure, the ungated `ρ·branch.flow(dp)` runs the draw backwards
(tank → column); fed to the sweep that spurious inflow pollutes the column's
feed mix, so the very split that prescribes the draws is taken from a
contaminated feed. Zeroing the draw for the sweep is what makes a draw insensitive
to its product tank's back-pressure (a stated limitation below) actually hold.
The consequence for `edge_flows`: under this arrangement it needs no slate and no
composition, only the topology telling a draw edge from an ordinary one.

This refines the verdict without touching it: (A) still wins, the column is still
fixed-pressure zero-volume, still needs no prescribed-flow branch, still
mass-neutral every tick. Only the *location* of the draw-flow write moved — out
of the solve and into the post-sweep engine step. And "I7 is green by
construction" (below) becomes a property to *hit*, not a freebie: it is true only
under this split/composition consistency, which a fixed-feed reference cannot
check (there stale = fresh). The moving-feed per-component gate
(`per_component_mass_survives_a_moving_feed_composition`) is what earns it.

To be precise about "no solver change": `network::fixed_pressure`, `classify`
and `validate_degrees` each gain a `Column` arm — the last because the column is
the **first 1-in-N-out unit**, where every existing device is 1-in-1-out, so
that function's degree rule cannot simply be extended to it. What is *not*
needed is the new machinery: no prescribed-flow branch type, no Jacobian change,
no change to `assemble`. The algorithmic changes are the `edge_flows` guard and
the post-sweep engine step above, and that step carries one ordering obligation
worth stating rather than rediscovering in code: **the feed flow must be summed
before the draws that scale off it** — trivially satisfied where it lives now
(the engine reads the resolved feed edge, then writes every draw), but it was the
buried assumption when the write was imagined inside `edge_flows`'s edge-id loop.

Likewise, "reject a free node on a draw line at load time" is a topology
*trace* of the column's outlet edges, not a field lookup. Cheap, but it is real
validation work and belongs in the loader alongside the existing checks.

**The restriction this buys, stated rather than assumed.** The "no Jacobian
change" claim holds only while every draw edge has fixed nodes at both ends. A
free node on a draw line — an operator valve on the kerosene draw, which a game
plainly wants — puts a prescribed edge back into the system. That case is not
automatically ill-posed: a free node fed by one prescribed edge and drained by
one pressure-driven edge has residual `sᵢ − ṁ(P)`, one monotone equation in one
unknown. The general statement is: **the Jacobian stays nonsingular iff every
free node retains at least one pressure-driven edge to an anchored node**, which
is the existing `anchored_set` reachability with prescribed edges excluded from
`conducts`. M3.2 ships the fixed→fixed case and **rejects a free node on a draw
line at load time**, because a guard whose failure mode cannot be exercised by a
scenario in the repo is a guard that cannot be falsified. Lifting the
restriction is a later, tested step, not a silent capability.

**Pressure-fixing is necessary but not sufficient for a draw outlet.** The guard
cannot simply be "the outlet pins a pressure": `fixed_pressure` is `Some` for
five kinds, and two of them — `Source` and `Column` — are pressure-fixing yet
wrong outlets that *run*. A draw into a `Source` vanishes the product into an
infinite supply, mass "conserved" at the boundary; a draw into another `Column`
chains them, and the post-sweep two-pass draw write reads the upstream draw edge
(guarded to zero in the solve) before the downstream column's own write lands, so
the second column silently sees a zero feed. Both are refused at load: a draw
outlet is restricted to the three product-store kinds (`Tank`/`Sink`/
`Atmosphere`) explicitly, not to the pressure-fixing set. Chaining columns is a
later, tested extension, refused now rather than run half-working — the same move
as the valve-on-a-draw case above.

**Cut assignment and smearing.** Separation acts on the **feed**, not on any
stored inventory. Each draw `i` owns a boiling-point band from the column's
ordered cut points; component `c` with normal boiling point `Tb_c` gets a weight
`w_ic` per draw, and the split and per-draw composition are

```
splitᵢ      = Σ_c f_feed,c · w_ic          (mass fraction of feed to draw i)
comp_i,c    = f_feed,c · w_ic / splitᵢ     (draw i's composition)
```

`w_ic` is a ramp of width `smearing` [K] across each cut point rather than a
step, so a component boiling near a boundary lands partly in each adjacent draw.
**`Σᵢ w_ic = 1` is enforced by normalization, not hoped for**: that identity is
exactly what makes `Σᵢ splitᵢ = 1` and hence per-component conservation true by
construction rather than by numerical luck. `smearing = 0` is a sharp splitter
and is the degenerate case worth keeping representable.

Determinism: cut points are listed in file order and each names its outlet by
name, matching M3.1's slate convention (positional vectors, name-addressed
files).

**Transport.** A column is the first node whose outlets do **not** all carry the
node's own composition, so `energy::edge_composition_at` — already the single
owner of "which end of this edge am I" for the upwind rule — becomes the single
owner of "which *draw* am I" as well, returning the cut composition when the
upwind node is a column. Additive, and it keeps the rule in one place, the same
reason `edge_temperature_at` exists.

**Energy, and the identity that actually carries it.** Draws leave at the
**feed temperature**, but that alone does *not* make the enthalpy books balance
by inspection: the draws have different compositions and therefore different
heat capacities, so they do not share a specific enthalpy and `Σᵢ ṁᵢ·h` cannot
be written with one `h` factored out. What conserves energy is the same
normalization that conserves mass:

```
Σᵢ splitᵢ·cpᵢ = Σᵢ splitᵢ·(Σ_c comp_i,c·cp_c)
              = Σ_c f_feed,c·cp_c·(Σᵢ w_ic)
              = cp_feed                       since Σᵢ w_ic = 1
```

That is M3.1's linearity identity again (`Σ_c (Σ_s m_s·f_sc)·cp_c = Σ_s m_s·cp_s`,
already a test rather than a claim), applied to a split instead of a blend. So
`Σᵢ w_ic = 1` is load-bearing **twice over** — it is what makes per-component
mass conservation and the energy balance both true by construction, and its
violation is exactly M3.1's "conserves total mass, corrupts the fractions"
signature. I6 stays green because of this identity, not because the draws are
isothermal.

A real column's draws sit at their tray temperatures; representing that needs
the reboiler/condenser duties and a tray cascade, which is the complex column.

**Three things this fidelity does not model, deliberately.** **Reverse flow on
the feed** is rejected as `SimError::Numerical`: "the feed splits by boiling
range" names nothing when the feed runs backwards, and silently splitting a
negative flow would produce negative draws. Zero feed is *not* an error — all
draws are zero and each carries the feed composition. Draws leave at the **feed
temperature**, per the paragraph above. And a draw is **insensitive to
downstream back-pressure**: `ṁ_drawᵢ = splitᵢ · ṁ_feed` discards the draw edge's
`P_col − P_tank` entirely, so a product tank near full does not throttle or
reverse its draw — the column keeps pushing. That is inherent to (A) rather than
an oversight, and it is reachable in a game, so it is a stated limitation and
not merely an assertion buried in a test.

**What the tests must pin, and why the obvious gate is worthless here.** A
splitter conserves every component identically — in = out, by the normalization
above — so **I7 is green by construction and cannot falsify this unit at all.**
Neither can a total-mass gate. A cut boundary off by one component, or smearing
silently disabled, moves no mass balance anywhere. The reference must therefore
assert a **per-draw composition vector** against a hand calculation. Zero-volume
is what makes that hand calculation clean: each draw's composition is a pure
function of the feed composition, with no holdup history in it.

(This is also why "a tank with split outlets" was rejected as an implementation:
a holdup mixes to a single composition and its outlets carry *that*, so it
separates nothing — and it would make the reference number depend on tick
history.)

### Simple reactor (M4) — specified before building

The roadmap opens M4 with a `ReactionModel` trait that is a bare stub and an
engine slot (`reactions`) reserved but unused. A reactor is the first unit that
changes composition by **chemistry** — it destroys some pseudo-components and
creates others — which puts it at odds with the two conservation invariants M3
built (I7 per-component mass, I6 energy), both of which assume every interior
node conserves each component and its sensible enthalpy. A reactor conserves
**total** mass but not per-component mass, and it moves chemical energy the
engine's datum does not track. Those two collisions, not the kinetics, are the
crux, so this gets a written note before any code — the same discipline the
column got.

There are three forks to settle. Two are decisive on inspection; the third is
the well-posedness question, and it is the one that shapes the unit.

**Fork 1 — vocabulary: are the kinetic lumps slate members, or a layer above
the slate?** The FCC 4-lump model speaks in four lumps (gasoil → gasoline →
gas + coke); everything else in the engine — transport, tanks, the column, I7 —
speaks the engine-wide pseudo-component **slate** (a positional vector, canonical
order, §4). Two ways to reconcile them:

- **(V-slate) the lumps ARE slate members.** A scenario that runs an FCC reactor
  puts the four lumps in its slate as ordinary pseudo-components, each with its
  own `Tb`, `MW`, `density`, `cp`. The reaction is then a transformation on the
  composition vector, reading and writing those components by index, resolved
  from names at load exactly as column draws and exchanger sides already are.
- **(V-map) a Tb-band ↔ lump mapping.** Aggregate the slate's cuts into lumps by
  boiling range, react the lumps, then redistribute each lump's mass change back
  across its member cuts.

**(V-slate) wins, and (V-map) is not close.** The redistribution (V-map) needs is
*underdetermined*: when a kilogram of the gasoil lump cracks, which member cuts
of the gasoline lump receive the product, and in what proportion, is not
information the 4-lump model carries — it would be a second invented model bolted
under the first, with its own unfalsifiable parameters. (V-slate) has no such
freedom: the reaction network names exactly the components it moves mass between.
Its cost, stated rather than hidden: **coke and light gas are awkward
pseudo-components.** Coke does not boil; it is given a defensibly high `Tb` so a
downstream column routes it to the bottoms/residue draw, and its `density`/`cp`
are nominal. That is a modelling compromise, but it is a *local* one (four
parameter rows in a slate) rather than a whole redistribution model, and it keeps
one vocabulary across the entire plant.

**Fork 2 — energy: this is a co-equal crux, not a footnote.** The engine's energy
datum is **sensible-only**: `h = cp·(T − T_ref)`, no formation enthalpy (§4a).
A reaction is invisible to that datum in two distinct ways, and both must be
handled:

1. **The cp shift at constant T.** `Σ_c m_c·cp_c` changes when the composition
   changes, even with the temperature held fixed, because the lumps have
   different heat capacities. So a reactor cannot be "sensible-enthalpy neutral";
   the sensible books move across it *by construction*, and something must own
   that move.
2. **The heat of reaction.** Cracking is endothermic. The energy absorbed is
   formation-enthalpy change, which the sensible datum does not carry at all, so
   it must enter as an explicit parameter `Δh_rxn` (specific, at `T_ref`) that
   the `ReactionModel` returns alongside the product composition. If it is not
   modelled it has no gate, and a feature with no falsifiable gate is one M-work
   has consistently refused to ship — so either `Δh_rxn` is modelled and pinned,
   or the reactor's energy behaviour is a lie by omission.

Two *distinct* duties fall out of this, and conflating them makes the energy gate
vacuous — the trap worth naming before an implementer walks into it:

- **Emergent sensible duty** = `ṁ·[cp_out·(T_set − T_ref) − cp_feed·(T_feed −
  T_ref)]`. This is what the engine's sensible books see, and it closes **by
  construction** because the reactor imposes `T_set` — exactly as a furnace's
  sensible books close because it imposes its duty. The reactor is the furnace
  inverted: duty emergent, temperature fixed. Pinning this quantity gates the
  **cp-shift** (using `cp_feed` at both ends fails it), and nothing more.
- **Reported physical duty** = emergent sensible duty **+ `ṁ·Δh_rxn`**. This is
  the external heat a regenerator/operator must supply to hold `T_set` against
  the endotherm, and it is the **only** quantity that gates `Δh_rxn`. It is a
  reported diagnostic: under the isothermal verdict below, **`Δh_rxn` never feeds
  the forward outlet temperature** — a reader looking for where `Δh_rxn` changes a
  downstream `T` will correctly find nothing, because feeding it back *is* the
  adiabatic case (Fork 3) this milestone defers. Modelling it explicitly is what
  gives the endotherm a falsifiable home despite the sensible-only datum.

The invariant consequence mirrors the mass story: **I6 excludes reactors.** Not
because I6 structurally cannot span one — it already carries a furnace's `Q` in
its boundary accounting — but because the reactor's reported duty carries a
`Δh_rxn` term that lives *outside* I6's sensible-only frame, so folding it in
would mean teaching I6 about formation enthalpy it otherwise never touches.
The reactor's own energy gate owns that instead.

**Fork 3 — the well-posedness verdict: isothermal at a riser-outlet-temperature
(ROT) setpoint, not adiabatic.** This is the fork that decides whether the unit
is explicit or implicit, and it is the direct analog of M3.2's "columns run on
pressure control."

- **(T-iso) the reactor holds a fixed outlet temperature `T_set`.** Reaction
  extent is then a **pure function of the known `T_set`**, the feed composition,
  and the residence time — no inner solve. The duty required to hold `T_set` is
  *emergent* (computed, reported), exactly inverting the furnace, whose duty is
  an input and whose ΔT is emergent.
- **(T-adia) the reactor is adiabatic; outlet T emerges from the feed enthalpy
  plus the heat of reaction.** With T-dependent kinetics (Arrhenius), extent
  depends on T and T depends on extent, so the outlet is an **implicit fixed
  point solved inside a zero-volume node** — the same shape of ill-posedness the
  column note dissected, and worse, because it couples the composition ODE to a
  temperature ODE along the residence coordinate.

**(T-iso) wins for M4, and it is not a fudge.** Riser outlet temperature *is* the
FCC operator's primary handle — the catalyst circulation rate is trimmed to hold
it — so pinning `T_set` is the same modelling move, and the same justification, as
pinning the column's pressure. It also keeps the unit's reference **constructible**:
the roadmap requires a test against *published lump yields at a known temperature*,
and (T-iso) evaluates the kinetics at exactly that known temperature, whereas
(T-adia) drags a temperature trajectory along the residence coordinate into the
reference number. Adiabatic / emergent-T is deferred — it is where the coupled
`dC/dτ` + `dT/dτ` ODE earns its place, a later fidelity step, not M4's.

**The design that falls out.** With those three settled, the reactor is
structurally simple — **hydraulically it is a furnace**:

- `NodeKind::Reactor` is a **zero-volume, 1-in-1-out** node carrying only config:
  the outlet setpoint `T_set` and the residence time `τ` (a fixed design
  parameter — flow-dependent `τ = V·ρ/ṁ` is deferred, since it would make the
  published-yield reference `ṁ`-dependent). The reacting components, the rate
  constants, and `Δh_rxn` live in the **`ReactionModel`**, not the node — fidelity
  is trait-impl selection (CLAUDE.md rule 2), and the node carries parameters the
  way a valve carries its `Cv` while the algorithm lives in the solver.
- **No new solver machinery.** Total-mass-neutral and 1-in-1-out means outlet flow
  = feed flow, both edges ordinary pressure-driven, and the flow solver's existing
  zero-volume node mass balance already forces `ṁ_out = ṁ_in` — it never learns a
  reaction happened. `classify`, `fixed_pressure` and `validate_degrees` each gain
  a `Reactor` arm **identical to `Furnace`'s** (zero-volume, pins no pressure,
  one inlet + one outlet). No prescribed-flow branch, no Jacobian change, and —
  unlike the column — **no post-sweep draw-flow prescription**, because there is
  one outlet and its flow is hydraulically determined.
- **The reaction runs inside the composition sweep, and this is the one real
  plumbing change.** A node downstream of a reactor must see the *product*
  composition within the same sweep, so the transform cannot be deferred to a
  post-sweep step (that would feed the downstream node the pre-reaction feed).
  Therefore `energy::resolve_node_states` gains a `&dyn ReactionModel` parameter
  (the trait lives in `core`, so this is not a `core → solvers` dependency — the
  engine passes its `reactions` object down). When the Kahn sweep reaches a
  reactor, its inflows are already resolved: mix them to the **feed**, call
  `reactions.react(feed, T_set, τ, slate)` to get the **product** composition and
  `Δh_rxn`, and store the *product* as the reactor's resolved composition and
  `T_set` as its resolved temperature. Note the reactor is the first **T-overriding
  zero-volume node**: it is *swept*, so `boundary_temperature`/`boundary_composition`
  must both return `None` for it (returning `Some` T trips the sweep's
  inertial-vs-swept partition guard), yet it carries **two** temperatures at once —
  the feed (mixed inflows, needed for the emergent-duty calc) and the outlet
  (`T_set`, imposed on everything downstream). A furnace, by contrast, mixes its
  inflows for T and merely adds duty; the reactor discards the mixed feed T for its
  outlet. Because the reactor has a **single outlet**
  that carries a **uniform** product composition, the existing upwind rule
  delivers it to every outlet edge with **no change to `edge_composition_at`** —
  the reactor is genuinely simpler than the column here, which needed
  `edge_composition_at` special-casing only because its N draws differ.
- **`react()` is called once per reactor per tick.** RK4 over `τ` is not cheap;
  computing the product once in the sweep and letting the upwind rule distribute
  it (rather than re-integrating per outlet edge, as a naive `edge_composition_at`
  hook would) is deliberate.

**The `ReactionModel` contract.** One method — a pure function

```
react(feed: &Composition, temperature: Kelvin, tau: Seconds, slate: &Slate)
    -> Result<Reaction { products: Composition, dh_rxn: JPerKg }, SimError>
```

*As built (M4.1), this refines the sketch above in three ways, none changing the
physics:* `τ` is threaded now (the lookup fidelity ignores it, but `FourLump`
integrates `dC/dτ` over it — putting it in the signature now spares a trait
churn one milestone later); the two returns are a named `Reaction` struct rather
than a bare tuple; and `Δh_rxn` is a `JPerKg` newtype with an explicit **sign
convention — positive = endothermic (heat absorbed)** — so the reported duty
`= sensible + ṁ·Δh_rxn` and FCC cracking carries a positive value. The two
emergent duties are returned to the engine on `NodeStates.reactor_duty`.

Testable in isolation exactly like `energy::column_separation`, with impls in
`solvers/`:

- **`NoReactions`** — identity composition, `Δh_rxn = 0`. The default, so every
  pre-M4 scenario stays bit-identical (the regression anchor M3 also relied on).
- **`SimpleLookup`** — fixed conversion table per `(T-band, feed lump)`; no ODE.
  Establishes all the plumbing and every gate — mass redistribution, the emergent
  duty, the composition transform through the sweep — *without* the kinetics.
- **`FourLump`** — Weekman/Lee-style FCC 4-lump Arrhenius kinetics, integrated
  with **fixed-count** RK4 substeps over `τ` (fixed count, not adaptive:
  determinism, CLAUDE.md rule 3). Cite the exact parameter set in the code.

Mass-conservation is the reaction network's own property: for kinetics the rate
matrix has columns summing to zero (mass created in products equals mass
destroyed in reactants), and the lookup table's rows are renormalized to `Σ = 1`.
This is the reactor's load-bearing normalization — the exact analog of the
column's `Σᵢ w_ic = 1` — and its violation is M3.1's "conserves total mass,
corrupts the fractions" signature applied to a reaction instead of a blend.

**What the tests must pin, and why the obvious gate is worthless here.** As with
the column, a total-mass gate cannot falsify this unit — the reaction is total-mass
neutral by construction — and I7 excludes it. The gates that earn their place:

- **A per-lump yield reference** against published 4-lump numbers at a stated
  `T_set` and `τ` (`solvers/tests/reference/`), the roadmap's required anchor.
  This is what a wrong rate constant, a transposed stoichiometry, or a dropped
  RK4 substep fails.
- **The reactor total-mass gate** — `Σ_c` out = `Σ_c` in across the unit — which a
  rate matrix whose columns do *not* sum to zero fails, while every per-component
  and energy check that cannot see a uniform mass leak stays green.
- **The reactor energy gate** — the two duties above, pinned separately so each
  falsifies its own term. The *reported* duty must sit exactly `ṁ·Δh_rxn` above
  the *sensible-only* baseline `ṁ·[cp_out·(T_set − T_ref) − cp_feed·(T_feed −
  T_ref)]`, so a wrong or dropped `Δh_rxn` fails it and nothing else. The gate is
  the **difference**, not the reported duty against its own formula: computing the
  duty by that formula and asserting it equals the formula is a tautology (a
  well-posed, self-consistent gate that pins nothing). Dropping the cp-shift
  (using `cp_feed` at both ends) fails the sensible baseline itself.
- **The `NoReactions` regression anchor** — every M1/M2/M3 golden bit-identical,
  the same one-component-slate guarantee M3 leaned on.

**Slice: simple first.** The `SimpleLookup` reactor lands the whole
redistribution + `Δh_rxn` + sweep-transform machinery and all four gate *shapes*
without the ODE; `FourLump` is then additive kinetics behind the same trait,
pinned by the published-yield reference. This is the M2/M3 slicing rationale
again: put the milestone's structural difficulty in the first slice and make the
second slice a fidelity swap.

**Deferred, stated rather than omitted.** Adiabatic / emergent-T (the coupled ODE,
Fork 3); flow-dependent residence time `τ = V·ρ/ṁ`; a 1-in-2-out reactor that
separates coke at the outlet (that reintroduces the column's draw-flow machinery —
coke rides the single outlet and a downstream column routes it); and the catalyst
regenerator loop that physically supplies the endothermic duty (the emergent duty
is reported, not sourced from a coupled regenerator).

### FCC 4-lump kinetics (M4.2) — what building it settled

The additive fidelity swap the slice above promised: `FourLump` implements
`ReactionModel` beside `SimpleLookup`, selected by `reactions = "fcc"`, and
nothing above the trait changed. `NodeKind::Reactor`, the sweep, the two duties
and the loader arm are all M4.1's, untouched. What follows is what only building
the kinetics could decide.

**The network, and a typo in the source.** Gas oil → gasoline (`k₁`), → light
gases (`k₂`), → coke (`k₃`); gasoline → light gases (`k₄`), → coke (`k₅`). Gas
oil cracking is **second order** in its mass fraction, gasoline cracking **first
order** — stated by Olufemi, Latinwo & Olukayode, *Riser Reactor Simulation in a
Fluid Catalytic Cracking Unit*, Chem. & Process Eng. Research 7 (2013) 12–21, and
independently by Bunny et al., RJPBCS 6(4) (2015) 1269. Olufemi's *printed*
eqs. (15)–(16) put the gasoline paths at `x₃²`, contradicting that same paper's
stated assumption; the code follows the assumption, notes the discrepancy, and
`first_order_gasoline_cracking_matches_the_closed_form` exists so that the typo
cannot be reintroduced silently.

**Catalyst decay is a reparametrization, and that is why the model is checkable.**
`φ(t) = exp(−α·t)` (Weekman) depends only on time, so `θ(τ) = ∫₀^τ φ =
(1 − e^{−ατ})/α` turns every rate law into its decay-free form in θ. Second-order
gas oil then has the closed form `1/y₁ = 1 + Kθ` with each product taking `kᵢ/K`
of the converted feed, and a pure-gasoline feed decays as `e^{−(k₄+k₅)θ}`. Those
are hand calculations, derived from the rate law rather than read back from the
integrator, and they are what pins the stoichiometry.

**The units crux — and the `core` change that was NOT made.** Published FCC rate
constants are almost never in units that accept a 3-second residence time: they
are per unit **catalyst mass** (riser models multiply by holdup and catalyst
density) or are quoted against *space time* in hours. Dropping such a constant
into `τ = 3 s` gives a conversion wrong by decades that still converges, still
conserves mass, and still reruns bit-identically — this milestone's version of the
`√101325`-for-`√1e5` slip M1's `kv_reference` was built for. The convention is
therefore stated once, in the module: **`k₁…k₃` in (mass fraction·s)⁻¹, `k₄`,`k₅`
in s⁻¹, against `tau` in seconds, with catalyst loading folded in** at the
reference plant's catalyst-to-oil ratio. A `cat_oil_ratio` field on
`NodeKind::Reactor` was considered and rejected: the node deliberately models no
catalyst inventory (the regenerator loop is deferred above), so the field would
have exactly one value in every scenario, and a parameter with one value has no
gate that could falsify it.

**The anchor is an ENVELOPE, and that is a limitation with a name.** The roadmap
asked for a reference against published lump yields at a stated `T_set`/`τ`. Every
source that tabulates `k₁…k₅` for this network proved unreachable behind a
paywall, and transcribing constants from a search summary is precisely the
silent-wrong-number failure this workspace refuses. So the constants are
**calibrated, not transcribed**, and the published anchor is the industrial riser
data reproduced by Olufemi et al. Tables 1–4 from Ali & Rohani (1997): outlet
795–808 K, COR 5.43–7.20, gasoline 41.78–46.90 wt%, coke 5.34–5.83 wt%, 79 wt%
conversion. The calibrated set lands at gasoline 43.8, coke 5.70, conversion 79.0
and light gases 29.6 wt% at 800 K / 3 s.

**And the envelope gate's ceiling has to be named, because it is easy to
overclaim.** The constants were *fitted* to that envelope and the gate then checks
they land in it — one anchor used twice. It is therefore a **regression lock on
the calibration, not an independent validation of the parameter set**: it fails
loudly on a fault introduced later (a `tau` unit slip, a dropped catalyst-loading
fold — decades out of band) and says nothing about whether these five constants
are the right five. That is the same ceiling `kv_reference` names for M1's network
hand calc, and it is why upgrading to a point match against a tabulated set
(Lee et al. 1989; Ahari et al. 2008) is worth doing when either paper becomes
reachable: it converts the gate from a regression lock into an independent check.
The closed-form gates are unaffected — they are told the parameters, so they carry
no such circularity.

**`Δh_rxn` from per-lump formation enthalpies**, `Σᵢ (y_out,i − y_in,i)·h_f,i`.
Path-independent (a state function, which is what enthalpy of formation means),
identically zero when nothing reacts, and mass-consistent given `Σy = 1` at both
ends. The datum is free — only differences matter — so gas oil is 0. Formation
enthalpies for "light gases" and "coke" are not published quantities: these are
chosen numbers whose SUM is published (a few hundred kJ/kg of feed for FCC
cracking, Sadeghbeigi), and the code says so rather than presenting them as
sourced.

**The gate that had to be invented.** The closed forms pass at the shipped 64 RK4
substeps with orders of magnitude to spare, so they cannot tell a correct RK4 from
a degraded one — a dropped stage still converges, just slower. The discriminating
gate is the **order of convergence**: halving the step must cut the error ~16×.
Measuring it also exposed that the naive step range is wrong — below 32 substeps
this problem is not asymptotic (the error changes sign between 8 and 16 steps and
the ratio reads 245, meaning nothing), so the gate runs at 32/64/128 where the
measured ratios are 14.7 and 15.5. Truncation at the shipped count is 5.6e-9,
which is what makes the closed-form tolerance a derived number instead of a tuned
one.

**Falsified before trusted**, eight mutations, each caught by the set that should
catch it: rate constants a decade low fails the envelope and the wired plant
**while every closed-form gate stays green** — the two gate kinds discriminating
exactly as claimed; first-order gas oil; second-order gasoline (the paper's typo);
**one shared activation energy, which fails the Arrhenius gate and nothing else**;
mis-set RK4 stage weights, caught by the order gate; a flipped decay sign, which
makes the catalyst gain activity; formation enthalpies signed backwards, which
fails only the two energy gates and leaves every composition gate green; and a
rate-matrix column that does not sum to zero, which the pre-normalization mass
check turns into an `Err` rather than a plausible composition. Falsification also
found a **vacuity in a test of this file's own**: asserting that the products sum
to 1 proves nothing, because `Composition::from_weights` normalizes — it now
asserts on a slate carrying an inert, where the reacting lumps' combined share is
a genuinely free quantity.

**Deferred, on top of the reactor note's list.** Coke-on-catalyst deactivation
(the Voorhies/Bunny form `φ = (B+1)/(B + exp(A·C_coke))`, which couples decay to
the coke the pass itself makes and so needs a catalyst inventory this reactor does
not have); feed-quality dependence of the constants; and any second lump slate —
`FourLump` names its four lumps and refuses a slate without them.

## 6. Time

- Engine fixed timestep, default `dt = 0.1 s` (config per scenario).
  Frontends may call `tick` faster/slower than real time.
- Slow states integrate explicitly (Euler for inventories, RK4 inside
  reactors). If a unit needs smaller steps, it substeps internally —
  the global tick never changes at runtime.

## 7. Snapshots and commands

- `Snapshot`: tick index, sim time, per-node state (levels, temperatures,
  unit-specific extras as tagged enums), per-edge stream state, solver
  diagnostics (iterations, residual). Serde: JSON for humans, bincode later
  if profiling demands.
- `Command`: `SetValveOpening{node, opening}`, `SetPumpOn{node, on}`,
  `PuncturePipe{edge, area}`, `SetHeatInput{node, power}` (the fire — this is
  what the older sketch called `IgniteNode`), `SetFurnaceDuty{node, duty}`,
  `SetCoolerDuty{node, duty}` — applied between ticks, validated, invalid
  commands return Err without mutating. `#[serde(tag = "cmd")]`, so the variant
  names and field names are a frontend contract: see §3b before renaming one.

**Every command must have a reported consequence, and the fire did not.**
Found while building the M6.2 scene. `Command::SetHeatInput` worked — the
energy balance read `node.heat_input` and the temperature responded — but no
snapshot field carried it, so a frontend could infer a fire from a rising
temperature or remember having sent the command, and could not read the
engine's own answer. `EdgeSnapshot::leak_mass_flow` exists for exactly the
analogous question about the leak, which is what makes the asymmetry an
oversight rather than a decision. It is also M6.0's defect with the direction
reversed: there, a field nothing consumed; here, a field nothing reported. A
scene drawing flames from its own memory keeps drawing them after a reload or
a refused command — a picture of what the frontend did, not of what the engine
holds. `NodeSnapshot::heat_input_w` closes it.

**The trap in closing it.** `energy::heat_load(node)` — the function every
consumer of "how much heat enters this node" already calls — returns the fire
PLUS the node's own unit term: a furnace's duty, a cooler's negative duty, a
tank's ambient exchange. Reporting that sum is the obvious implementation and
would show every furnace in every scenario as on fire. The field is
`node.heat_input`, the damage hook alone; operating setpoints stay on `kind`
where they already are, and a fire on a furnace *stacks* rather than replacing
it. Gated in `scenarios/tests/fire_reporting.rs`, whose ambient arm builds its
plant inline because **no scenario in the repo sets a nonzero tank
`ambient_ua_w_per_k`** — an arm written against the existing files would report
zero for the right reason and pass for the wrong one.

## 8. Godot integration (M6)

`godot-ext` (gdext crate) exposes a `RefinerySim` node: `load_scenario(path)`,
`tick_in_physics_process`, `get_snapshot() -> Dictionary` (or typed accessors
for hot paths), `send_command(...)`. Sync single-threaded first; move the
engine to its own thread behind a snapshot channel only if profiling shows
tick time threatening the frame budget.

**What is gateable here and what is only demonstrated.** M6 bundles two unlike
things, and conflating them is how a milestone comes to believe it is tested.
The adapter's **translation layer** — command JSON → `Command`, `Snapshot` →
JSON or `Dictionary`, `SimError` → a signal — is ordinary Rust and is
unit-tested like anything else, including the round-trips that a typo in a
`#[serde]` tag would break. A **scene** is not gateable by this repo's
standards: no `cargo test` can assert that a tank looks like a tank. The scene
is therefore a **demonstrated** acceptance criterion, in the sense
`relief_blowdown.toml` was for M5 — a named thing that is run and observed, with
the observation written down. Stating this up front is cheaper than discovering
at the end of M6 that half the milestone has no gate and pretending otherwise.

### The translation layer (M6.2) — what building it settled

**The crate splits along the feature, not along the file.** `godot-ext` now
holds two things: `bridge`, plain Rust with no Godot types, and (later) the
gdext `RefinerySim` node behind `--features godot`, off by default. The reason
is not tidiness. gdext's `GString`/`Dictionary`/`Variant` need a live Godot
runtime, so anything written in terms of them cannot be exercised by
`cargo test` at all — godot-rust's own suite runs inside the engine. A
translation layer expressed in Godot types would therefore be exactly the
"half the milestone has no gate" outcome the paragraph above exists to
prevent. Everything with a decision in it lives on the pure side; the binding
is marshalling with no branches.

**The crate joined the default workspace.** It was excluded because it needed
a Godot toolchain; that requirement now belongs to the `godot` feature, not to
the crate, and a gate that only runs under a bespoke `--manifest-path`
invocation is a gate nobody invokes. `cargo test --workspace` and
`cargo clippy --workspace --all-targets` now cover it.

**The trust boundary is here, and it is load-bearing.** `Engine::apply`
indexes its graph directly (`core/src/graph.rs:552-563`), so an out-of-range
`NodeId`/`EdgeId` **panics** — measured, not inferred. Two project rules point
opposite ways: rule 5 says the engine never panics, rule 1 says a `core` change
needed to satisfy a frontend means the adapter is wrong. **Reachability breaks
the tie.** Every in-repo caller takes its ids from a snapshot and is in-range
by construction, and the CLI constructs no `Command` at all; untrusted external
input is the only path in. So the ids are validated in `bridge`, against the
set it read from the engine, and `core` is untouched. `Referent`'s
wildcard-free match on `Command` is what makes that guard survive a new command
variant — the crate stops building until the variant declares what it
addresses. **Un-defers** into a `core` fix if a second untrusted-input frontend
appears, or if any in-repo caller gains the ability to construct an
out-of-range id; `core_panics_on_an_out_of_range_id` is a characterization test
that fires if `core` changes underneath the guard.

**No second command format.** `Command` addresses nodes and edges by numeric
id and that JSON shape is a contract (§7, §3b), so the bridge does not add a
name-addressed variant of it — two wire formats for one action is how they
drift. It exposes `node_id(name)` / `edge_id(name)` instead: a scene resolves
once at startup and sends the contract's own JSON thereafter. Name uniqueness
is a *requirement* of that lookup, enforced by refusing to load a plant that
breaks it; the sweep over `scenarios/` is evidence it is not onerous, not what
makes it true. Under M6.1's split-at-load, a declared punctureable pipe's name
resolves to the **upstream half** — the edge `PuncturePipe` addresses and the
one carrying `leak_mass_flow` — with `<name>__downstream` and `<name>__leak`
reachable by their own names.

**Pre-tick, three snapshot fields serialize as `null`, and the bridge emits
them.** `node.pressure_pa`, `node.temperature_k` and `edge.dissipation_w` are
NaN until a solve has happened, and `serde_json` writes NaN as `null`.
Substituting a number would invent data the solver has not produced — the
failure mode this repo has three notes about — so the engine's JSON is passed
through unchanged. The gate pins the *field set*, not "the round trip fails":
a round-trip assertion would keep passing if a different field started
emitting null. Two consequences a scene author will otherwise report as bugs:
pre-tick JSON does not deserialize back into a `Snapshot`, and pre-tick
`nodes[i].temperature_k` is `null` while `nodes[i].kind.temperature` is a real
number — the first is *solved*, the second is *stored*.

**`serde_json` needed `float_roundtrip`, and nothing before now could have
found it.** The default float parser is fast, not exact: it can land 1–2 ULP
from the value that was written, so `parse(write(x)) != x`. The bridge's
round-trip gate failed on exactly two edge floats out of ~90. The existing
determinism gate never saw it because it compares serialized *strings* between
reruns and never parses one back — a class of defect that "compare the output
text" cannot reach. The feature is set at `[workspace.dependencies]`, because
cargo unifies features per build and a per-crate setting would make the
behaviour depend on which crates are in the build.

## 9. Error handling & diagnostics

`SimError` (thiserror): `SolverDiverged`, `NonFiniteState{location}`,
`InvalidCommand`, `ScenarioError`. Engine keeps a ring buffer of recent solver
diagnostics included in snapshots — frontends can show "solver stress" and
tests can assert convergence quality, not just results.
