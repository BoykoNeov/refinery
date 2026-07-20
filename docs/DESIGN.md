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

- **No pump work or valve throttling heat.** Both dissipate into the stream in
  reality; a pass-through device currently copies its inlet temperature to its
  outlet. The reference pump's rise is ~0.02 K — far below the model's accuracy,
  and below any tolerance a reference test could be falsified against, so
  building it in M2 would have meant a feature with no gate that earns its
  place. Deferred to M5 (ROADMAP), where ΔP-driven throttling in a high-head
  service and real enthalpy make the quantity large enough to be worth pinning.
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
- `Command`: `SetValveOpening{id, frac}`, `SetPumpOn{id, bool}`,
  `PuncturePipe{edge, area}`, `IgniteNode{id}`, `SetFeed{...}` — applied
  between ticks, validated, invalid commands return Err without mutating.

## 8. Godot integration (M6)

`godot-ext` (gdext crate) exposes a `RefinerySim` node: `load_scenario(path)`,
`tick_in_physics_process`, `get_snapshot() -> Dictionary` (or typed accessors
for hot paths), `send_command(...)`. Sync single-threaded first; move the
engine to its own thread behind a snapshot channel only if profiling shows
tick time threatening the frame budget.

## 9. Error handling & diagnostics

`SimError` (thiserror): `SolverDiverged`, `NonFiniteState{location}`,
`InvalidCommand`, `ScenarioError`. Engine keeps a ring buffer of recent solver
diagnostics included in snapshots — frontends can show "solver stress" and
tests can assert convergence quality, not just results.
