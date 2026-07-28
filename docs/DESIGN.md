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
and light gases 29.6 wt% at 800 K / 3 s. What that anchor buys is the decade-scale
fault above; what it cannot buy is a percent-level check on any individual rate
constant. Both halves are stated in the test file rather than implied. Upgrading
to a point match against a tabulated set (Lee et al. 1989; Ahari et al. 2008)
remains available and needs only the paper.

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
