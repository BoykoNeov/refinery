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

"Per node" was written here at M1 and was not implemented until **M9.2** (§11),
which is why that slice reads as the code catching up with this paragraph rather
than as a retuning. Both fidelities now stop on one shared rule,
`network::grade_nodes`, and the scale is the node's own incident traffic.

Fluids: incompressible liquid through M3. Gas/compressibility is milestone
M5 and gets its own design note before implementation.

No cavitation / vapor-pressure floor in M1: the hydraulic solve is a pure
pressure-flow system, so an over-driven pump (e.g. low downstream resistance)
can produce a genuine solution with sub-zero *absolute* suction pressure. That
is real cavitation the model does not yet represent; a vapor-pressure clamp is
a later milestone.

**Corrected 2026-09-06 (M10 close-out).** This paragraph used to end "Frontends
should treat negative absolute node pressure as a 'cavitating' signal, not a
solver error." **The reachability claim above is true and is now measured; the
threshold in that last sentence is wrong, and it is wrong by exactly the fluid's
vapour pressure.** A liquid boils below its own vapour pressure, which is
positive, so a plant is already cavitating while its absolute pressure is still
above zero and the signal is silent.

**Measured on the reference plant with one number changed** — `elevation_change_m`
on `tank_pump_valve`'s suction line, i.e. the pump mounted above its tank:

| suction lift | pump-node pressure | state |
|---|---|---|
| 17 m | 9 904.7 Pa | liquid, 1.76× its bubble point |
| **17.44 m** | **5 640.6 Pa** | **cavitation begins** |
| 18 m | 190.2 Pa | boiling; the signal below is still silent |
| **18.02 m** | **0 Pa** | **the signal this paragraph named finally fires** |
| 19 m | −9 522.9 Pa | solver converges, 9.8 kg/s, deeply wrong |

So the marker is not unreachable and not useless — it is **late**, by a margin
equal to the vapour pressure of whatever is in the line. For cold water that is
5 640.6 Pa, which is the 0.58 m band above. **For anything hot or light it is not
a band, it is the whole plant**: light naphtha at 445.75 K boils at 913 281 Pa,
so the same marker would be nine bar late on a crude column's own fluid.

The distinction matters because the model **already contains** the thermodynamics
that decides it — `ThermoModel::k_value` with `TroutonThermo`, since M7.2 — so
the right criterion was available and was never used. Same shape as this
section's "relative mass-imbalance per node", specified at M1 and not implemented
until M9.2.

The honest statement, until a floor exists: **the engine does not detect
cavitation and has no signal for it**, and a negative pressure is a symptom that
arrives after the fact rather than a criterion. `docs/DEFERRED.md` B1 carries the
trigger and the measured distance — no shipped plant is past it, the tightest
solved margin in the corpus being **1.865×** (`heat_recovery`'s exchanger).

**Superseded by §13 (M11), and that 1.865× comes with a caveat the close-out did
not have.** The criterion is specified there — a node's own **bubble pressure**,
from `ThermoModel::bubble_pressure`, reported per node in the snapshot as a
verdict plus the number behind it. It is a *signal*, deliberately not the "clamp"
this paragraph promised: §13 fork 1 rejects the clamp, because the vapour a clamp
would account for is mass in a phase the state vector does not have (ledger row
B3). The caveat is that `heat_recovery` declares `thermo = "constant"`, whose
`k_value` is an `Err`, so **no engine configuration of that plant can produce
1.865×** — it is a number from the close-out's measurement script. Fourteen of
the fifteen shipped plants are in that position.

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

> **This deferral was spent by M8.0 and the paragraph above is history.** The
> outer loop over the classification is `network::solve_with_active_anchoring`
> (DESIGN §3c); both `known_defect_frozen_anchoring_*` pins were turned the right
> way up and now assert the fix, under the heading "the two plants that used to
> be `known_defect_frozen_anchoring_*`" in `invariants.rs`. Nothing here still
> holds a line.

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
  loudly instead of averaging it. ~~Un-defers when a plant needs a condensing
  overhead or a flashing feed, i.e. the complex column (M6+).~~ **Trigger
  narrowed by M7.0** (§5, "Complex column", fork 0): a condensing overhead does
  not need a two-phase *stream*, because under a total condenser the vapour never
  crosses a pipe — vapour-ness inside a cascade is a per-stage flow variable, not
  a component property. What remains deferred is the set that genuinely puts two
  phases in a `Stream`: a **flashing feed line**, a **partial condenser** (vapour
  distillate), and a **vapour side draw**. M7 refuses all three at load rather
  than approximating them, and each refusal message names this bullet.
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

## 3c. The anchoring active-set loop (M8.0) — specified before building

§3a leaves one **live defect** rather than an absent feature: `network::prepare`
derives the anchored set ONCE, from the seed compile, so a plant whose anchoring
the ANSWER decides is classified from a pressure that is not the answer. That is
exact for every element whose `conducts` is constant through a solve, and the
relief valve is the first element for which it is not. Both directions are
reachable and both are pinned by hand-checkable plants
(`known_defect_frozen_anchoring_*`): seed-open/converged-shut **fails the solve
outright on both fidelities** (a zero residual row and column, hence a singular
Jacobian), and seed-shut/converged-open converges with one determinate pressure
reported as atmospheric.

This note settles how the classification stops being frozen. It is the un-defer
§3a asked for — "a design decision with its own slice and its own measured
regression anchor" — and the shape below is that decision.

### Fork 1 — reclassify every iteration. **Rejected.**

The anchored set decides which free nodes are UNKNOWNS, so recomputing it inside
the loop changes the dimension of the system mid-solve. Three costs, the first of
which is disqualifying on its own:

1. **The line search stops meaning anything.** Newton's Armijo test compares
   `φ = ½‖R‖₂²` at the trial against `φ` at the incumbent. Those two numbers are
   sums over different index sets the moment the unknown set changes, so
   "sufficient decrease" is being asserted between quantities that are not
   comparable. The same objection applies to Simple's residual: `converged_at`
   would be reading a max over a set that moves under it.
2. It reinstates exactly the flapping the freeze was chosen to prevent, with no
   mechanism to stop a two-iteration cycle — the iterate is still moving, so
   there is no fixed point to detect.
3. Determinism survives it, but reproducing a failure means reproducing the
   dimension history, which is not in any diagnostic this workspace carries.

### Fork 2 — an outer loop over the classification. **Chosen.**

An *active-set* iteration, the standard shape for a problem whose unknown set is
part of its own answer. One pass is exactly today's solve. After it, the edges
are recompiled at the pressures that pass ended on and the anchored set is
recomputed from THOSE; if it differs from the set the pass ran under, the pass is
repeated under the new one.

Two properties this buys, and they are why it is preferred to any local repair of
the singular row:

- **Pass 1 is bit-for-bit the current solver.** A plant whose classification is
  already a fixed point — every plant with no relief valve in it — exits after
  one pass having computed exactly what it computes today. The regression anchor
  is therefore structural rather than tuned, which is the strongest form it can
  take. It is still measured (below), because "structural" has been wrong here
  before.
- **The dimension is constant WITHIN a pass**, so both fidelities' convergence
  tests keep comparing like with like, and the freeze's original justification is
  preserved rather than overturned. The freeze was right about what it forbade;
  it was applied at the wrong scope.

#### Fork 2a — reclassify only after a CONVERGED pass. **Rejected.**

It fixes the benign half and leaves the fatal half untouched: the whole point of
seed-open/converged-shut is that the pass does not converge, so a rule that fires
only on convergence never sees it. The 83-of-306 divergence rate is the half a
user actually meets, and a fix addressing only the cosmetic half is not worth the
slice.

#### Fork 2b — reclassify from the last accepted iterate, converged or not. **Chosen.**

Both of Newton's give-up paths (`dp` non-finite, and line-search exhaustion) and
both of Simple's (a non-finite step, the iteration cap) return WITHOUT writing a
bad value into the pressure map — the trial is discarded and the incumbent is
what remains. So a failed pass still leaves a finite, meaningful iterate, and on
the seed-open/converged-shut plant that iterate is exactly the one that reveals
the relief has shut. Reclassifying from it turns the failure into information.

**One guard, because "finite" is asserted above rather than guaranteed:** Simple
has one path (`pressures.values().any(|p| !p.is_finite())`) that fails *because*
the map went non-finite. A pass whose final pressures are not all finite is not
reclassified — its error is returned as-is. A classification derived from NaN
would be arbitrary, and an arbitrary retry is worse than an honest failure.

### The termination contract — a cycle is DETECTED, not waited out

The loop can fail to settle, and the geometry that does it is already in the
generator population. A relief spur that **relieves** (as opposed to a dead leg,
which cannot move anything because it carries no flow) is a feedback loop between
the classification and the answer: with the spur inert the junction sits above
set pressure, so the spur is classified open; with the spur active enough escapes
to the flare that the junction falls below set, so it is classified shut; and
pass 3 repeats pass 1.

Terminating on a bare pass cap and returning the last pass's answer would pick
between two self-consistent states **by iteration parity**, which is precisely
the failure `prove-the-exception-dont-skip-it` was written about. So the loop
keeps the classifications it has run, in order, and:

- **fixed point** (the recomputed set equals the set just used) ⇒ return that
  pass's result verbatim, `Ok` or `Err`;
- **repeat** (the recomputed set is one already seen) ⇒ terminate immediately
  with an `Err` naming the nodes that changed;
- **cap** (still producing new sets after `MAX_ANCHOR_PASSES`) ⇒ terminate with
  an `Err` saying so.

The last two are different events — a cycle is a plant with two self-consistent
answers, a cap is a plant still moving — and they are distinguished by a FLAG
rather than by their wording, because a caller has to be able to tell them from
an ordinary numerical failure (correction 1: this paragraph originally chose
`SimError::Numerical` and no new variant, and building it falsified the reason).
A repeat only counts as a cycle when the pass that produced it CONVERGED
(correction 2).

**What the non-settling case IS, physically.** It is valve chatter: a relief
whose own discharge is what makes it re-seat. §3a already defers "PSV hysteresis
and chatter (needs element state)", and this is the same plant state arriving
through the solver instead of through the element. So the honest answer is an
`Err` that names the oscillating node and points at that deferral, not a
tie-break dressed as an answer. How often generated spur trees land in it is a
MEASUREMENT this slice owes: nonzero is the deferral's new evidence, and zero
means the class is unreached and must be reported as unreached rather than as
covered.

### Where the loop lives — shared, and the reason is the warm start

Both fidelities are defeated by the same plant, so both need the fix; the
question is one driver or two copies. It is one, in `network.rs`, for the reason
`prepare`'s own doc comment gives ("so neither solver can get it wrong") plus one
that is specific to this change:

`warm_start` is solver state written **only on convergence**. Under a loop that
rule is no longer well defined by itself — pass 1 can converge under a
classification the loop then REJECTS, and writing the warm start there would leak
a rejected classification's pressures into the next tick, a cross-tick coupling
that does not exist today. The rule becomes "the driver commits the warm start
once, from the final accepted pass, and only when that pass converged". That is
one rule about the loop, not two rules about two solvers, and it must not be
written twice and left to drift.

Shape: the driver owns `prepare`, the pass loop, the reclassification and the
warm-start commit; each solver contributes one *pass* — its existing body, minus
the `prepare` call and minus the warm-start write, returning its result **and**
its final pressures. `prepare` gains an anchored-set override for passes after
the first, so a later pass runs under the set the previous pass's answer implies
rather than under the one its own seed implies.

Each pass after the first is seeded from the previous pass's pressures. That is
continuation rather than restart, and it is a PATH not an answer — uniqueness
makes the seed unable to move the fixed point, the same argument already recorded
for `Pⁿ`-versus-cold-mean in `prepare`. It is expected to be un-catchable by any
gate, and is listed as such below rather than left looking covered.

### What must not change, stated as a prediction that can be wrong

Two of the thirteen scenarios contain a relief valve (`gas_line.toml`,
`relief_blowdown.toml`). So:

- the **other eleven must be byte-identical** over a long run. Not "the suite is
  green" — a specific eleven, and if one of them moves, the loop has a bug rather
  than a legitimate effect;
- those two MAY move, and if they do the pass count is what explains it. Any
  scenario needing more than one pass without a relief valve in it falsifies the
  claim that `conducts` is pressure-independent for every other element, which is
  the claim this whole regression anchor rests on.

The two numbers that bounded the deferral — Newton's divergence rate and Simple's
convergence rate on generated spur trees — were floors under a defect. Once the
defect is fixed they must be **re-measured and tightened**, or they become
exactly the vacuous counters this repo has already shipped twice
(`a-counter-is-not-a-gate`). `floating_legs` reads the seed-time set directly and
is unaffected.

### The mutations this slice owes, named before building

Two are predicted NOT to be caught, and saying so in advance is the point:

- reclassify only after a converged pass (fork 2a) — must fail the seed-open
  plant's new assertion and nothing else, since that is the mutation which earns
  2b over 2a;
- cycle detection removed / cap of 1 — must fail the seed-open plant;
- seed each pass from the cold seed instead of the previous pass's pressures —
  **predicted uncaught**, being a path and not an answer;
- commit the warm start from every pass rather than the final one — **predicted
  uncaught by any single-tick test**, and therefore a gap to fill with a
  multi-tick gate or to record as uncovered.

Measured at the end of this section: three of these predictions were wrong,
including both "predicted uncaught" ones.

### Corrections from building it (M8.0, landed)

Six, of which two change the shape above rather than its verdict. The chosen fork
survived: an outer loop, first pass unchanged, reclassifying from the last
accepted iterate whether or not it converged.

**1. The error surface needed a new variant after all, and the reason given
against one was wrong twice over.** The note argued for `SimError::Numerical` on
the grounds that widening the type would spend a frontend change on a diagnostic
string. Measured, the Godot bridge cost exactly ONE line — its match is
exhaustive on purpose, precisely so a new variant has to be given a code. And the
real objection is not cost at all: a CALLER has to discriminate this outcome.
I3's termination invariant must accept "the classification did not settle" as a
legal ending, exactly as it accepts divergence, while still rejecting an ordinary
numerical failure — and doing that by matching a substring of a human-readable
message is not something to gate on. Hence
`AnchoringUnsettled { cycled, detail }`, with the flag rather than the prose
carrying the distinction.

**2. A repeat is a CYCLE only when the pass that produced it converged.** The
termination contract above says a repeated classification is terminal. Measured
before the qualifier existed: **13 of 21 repeats followed a pass that FAILED**. A
failed pass ends on a mid-flight iterate, so the classification it implies is a
guess rather than an answer, and repeating a guess is not evidence of two
self-consistent states — which is what the note claimed the cycle exit meant.
With the qualifier the loop keeps iterating in that case and the cap catches
genuine non-termination. On generated spur trees this moves 10 cycles + 0 caps to
**6 cycles + 2 caps**, with 2 plants reaching an ordinary divergence instead. Two
things follow: `cycled: true` now means what the note said it meant, and the cap
branch stopped being a stub-only path.

**3. `warm_start` was doing two jobs, and multi-pass is what separated them.**
Seeding a free node and parking a floating one both read the same map, which is
indistinguishable while there is one pass. They are not the same thing. Seeding an
anchored unknown is a PATH to an answer, so the nearest available start wins and
the previous pass's pressures are right. A floating node's value is what gets
REPORTED for a pressure the plant does not determine — and a previous pass's
mid-solve iterate is a fine path and a meaningless report. Found by a test rather
than by reading: the seed-open plant's sealed leg came back at 100502.69 Pa, which
is pass 1's iterate for a node pass 2 had decided was indeterminate. `prepare_anchored`
now takes the continuation seed separately, and the floating pin keeps reading the
tick's own warm start — the pre-M8.0 convention untouched.

**4. There are now two kinds of zero on a dead-end spur, and they carry different
tolerances.** While a leg floats, its edge is inert and `edge_flows` reports a
STRUCTURAL zero, exact to the bit. Once the leg is an anchored unknown, the same
zero is SOLVED — the node's mass balance is driven to the solver's own convergence
criterion and no further. Two assertions written at machine epsilon had to become
assertions against that criterion, read off the solver rather than restated (and
it is Simple, at `tol_rel = 1e-6` against Newton's `1e-8`, where the difference
shows). This is not a loosening: machine epsilon was asserting something no solve
ever promised.

**5. The regression anchor came out stronger than predicted, and the reason is
worth more than the result.** The prediction was eleven scenarios byte-identical
and two possibly moved. Measured over 300 ticks on BOTH fidelities: all thirteen
byte-identical, and every solve of every tick settles at pass one. A relief valve
changing state does not necessarily change the anchored SET — both shipped relief
scenarios discharge to a fixed node, which anchors whatever the valve does. What
moves the classification is a relief that ISOLATES a subnetwork, and no scenario
in this repo has one.

So **no wired demo exercises this fix.** It is exercised by the generators (152
floating dead legs per 400 samples) and by two hand-built plants. That is recorded
here rather than left to be discovered, because the inverse mistake — trusting a
demo file to cover a knob it never moves — is one this repo has already made
(`a-hand-written-scenario-can-be-vacuous`).

**6. Simple's failure on the seed-open plant changed KIND rather than
disappearing, and the test asserts the difference.** It now converges on that
plant in 7002 sweeps against a default cap of 5000, so at its defaults it still
gives up — but that is ordinary Gauss–Seidel stiffness on a fat/thin resistance
ratio (M5.4 FINDING 3), curable by sweeping longer. The frozen classification's
failure was categorically different and no cap could cure it: the leg's diagonal
was exactly zero, the node-wise step was non-finite, and the reported residual was
`inf`. The gate is therefore that this plant can no longer produce an infinite
residual, which is the signature, rather than that Simple converges in some
particular number of sweeps.

### What the mutations measured (the pass, run after landing)

Eight edits, seven caught. The three wrong predictions are worth more than the
seven right ones, and one of them is wrong about a mechanism rather than about a
test.

**The continuation seed is not "a path, not an answer" — this loop promoted it.**
Re-seeding each pass from the tick's warm start instead of from the previous
pass's pressures was predicted uncaught, on the uniqueness argument recorded in
`prepare` for `Pⁿ`-versus-cold-mean: the fixed point does not depend on where the
iteration starts. That argument is about ONE pass, and it stays true. What the
active-set loop adds is a second question the seed decides: a pass ends somewhere,
the classification is recomputed from where it ends, and the loop terminates on
the classification. So the seed selects which classification the next pass runs
under — an answer-shaped role, not a path-shaped one. Measured: the seed-shut
plant, which converged before this loop existed and converges under it, comes back
`AnchoringUnsettled { cycled: true }` when each pass is re-seeded cold, and
generated spur-tree refusals go from 6 cycles + 2 caps to 39 + 9. **The
continuation seed is what makes the classification sequence contract**, and the
note recorded it as a free choice.

**A single-tick gate can catch a cross-tick leak if it asserts the STATE rather
than the consequence.** Committing the warm start from every converged pass was
predicted uncatchable inside one tick, and the prediction was reasoning about
effects — a leaked pressure only shows up when the next tick reads it. The cycle
gate catches it anyway, because it asserts that a solve ending in `Err` leaves the
warm-start map empty. No multi-tick gate is owed.

**Two edits bundled into one prediction behave differently.** "Cycle detection
removed / cap of 1" was one line predicting one catch. A cap of 1 is the pre-M8.0
solver, so it fails the seed-open plant and four more; removing cycle detection
does not touch that plant and is caught only by the gates written for the cycle
exit itself. A prediction that names two edits and one outcome cannot be falsified
by either of them separately, which is the failure mode to avoid next time.

**The one uncaught mutation is fork 2b's own guard, and it is now gated.**
Removing the non-finite check — reclassifying from a NaN iterate — left the whole
workspace green: nothing in the repo reaches the branch. It is Simple's failure
path, and correction 6 above is why nothing reaches it any more (the plant that
used to produce an infinite residual no longer does). Rather than record a
defensive branch as unverified, it gets the same treatment as the loop's other
three exits — a stub gate, `a_pass_that_ends_non_finite_is_not_reclassified`,
which NaNs a relief whose seed classification is OPEN so that the NaN is what
shuts it. With the guard: one pass, and the pass's own `NonFiniteState` survives.
Without it: two passes, under a classification the NaN invented.

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
  solved per tick (M7; needs K-values — start with Raoult + a vapour-pressure
  law per pseudo-component derived from Tb). Scoped in "Complex column (M7)"
  below; the Antoine form named here is refined there to a Clausius–Clapeyron /
  Trouton one, because Antoine constants are data this slate does not carry.
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

### Complex column (M7) — specified before building

The simple column is a splitter: cut points in, mass fractions out, no vapour,
no trays, no duties. The complex column is the fidelity swap named since §5's
opening line — "stage-by-stage flash cascade at quasi-steady state, solved per
tick, needs K-values". This note settles what that costs *before* any cascade
code, because five of the six forks below change the shape of `core`, not just
the arithmetic inside a solver.

The verdicts, up front: the two phases live **inside the unit** and the plant
graph never sees one; the K-value fills the `ThermoModel` stub and the cascade
gets a new `SeparationModel`; the column is specified by **ratios only**; the
per-component balance becomes **tolerance-bounded for the first time in this
workspace**; a non-converged cascade is an **`Err`**, not a held profile; and
latent heat stays internal, which buys a scope boundary that must be refused at
load rather than half-supported.

#### Fork 0 — the deferral names this milestone, and its premise was too broad

§3a says of two-phase flow, flash and condensation: *"Un-defers when a plant
needs a condensing overhead or a flashing feed, i.e. the complex column (M6+)."*
ROADMAP repeats it. A note that quietly narrows a deferral whose stated trigger
is *this milestone* leaves a future reader holding a promise nobody kept, so the
premise gets argued rather than stepped around.

The premise is that a condensing overhead implies a two-phase **stream**. Under
a total condenser it does not. Phase in this model is a property of the
*component* (`PseudoComponent::phase`), and vapour-ness inside a cascade is not
a component property at all — it is a per-stage **flow variable**, `V_j` and
`y_j`, which exists only between tray `j` and tray `j+1`. Nothing on the plant
graph is ever two-phase: the feed edge carries liquid, every draw edge carries
liquid, and the vapour that separates them lives and dies between the reboiler
and the condenser without crossing a `Pipe`.

So the deferral does not die here; it **narrows**, and its trigger is rewritten
to the cases that genuinely put two phases in a `Stream`: a flashing feed *line*
(vapour appearing in a pipe upstream of the column), a **partial** condenser
(vapour distillate), and a vapour side draw. Those still change `Stream`,
`Composition` and every reader of them, exactly as §3a says — and this milestone
refuses all three at load (below) rather than approximating them. What M7 does
retire is the *reason* the trigger named the complex column: the belief that a
stage cascade cannot be built without them.

#### Fork 1 — where the two phases live. **Internal, and structurally so.**

- **(a) Internal to the unit.** The cascade is state private to the separation
  model. The plant graph keeps `NodeKind::Column` exactly as M3.2 built it:
  fixed-pressure, zero-volume, one feed in, N draws out, draw edges fixed→fixed,
  draw flows written post-sweep. Every hydraulic argument in the M3.2 note above
  survives untouched — no Jacobian change, no prescribed-flow branch, no new
  solver machinery. The complex column is **a different `SeparationModel`, not a
  different plant unit.**
- **(b) Phase in the state vector.** A vapour fraction on `Stream`, `Composition`
  carrying both phases, every reader updated. This is §3a's deferred milestone,
  and it is what the three refused cases above would need.

**(a) wins, and the reason to write it down is the hazard it hides.** The
tempting implementation of an internal vapour stream is a `Composition` — the
type already exists, already normalizes, already blends. It is the wrong
carrier, and it fails *silently*: on the all-liquid slate a crude column has,
`Composition::phase()` sees only `Phase::Liquid` components and returns
`Ok(Liquid)` with no error, after which `density_at` hands back the stored
**liquid** density and `mixture_cv` the liquid `cp`. Not a refusal — a finite,
deterministic, plausible, wrong number, which is the failure shape this
workspace keeps catching (`a-command-can-be-a-no-op`, and the gas-density clamp
M5.4 found). The single-phase connected-component guard cannot help: it guards
the *graph*, and this vapour is not on the graph.

The fix is structural rather than a comment. Cascade-internal state is a
**solvers-local type in molar units** — stage molar flows `L_j`, `V_j` and mole
fractions `x_j`, `y_j` — not a `core::Composition`. Then handing an internal
vapour to a liquid property function is not a mistake to remember; it does not
typecheck. Mass-fraction `Composition` appears only at the unit's boundary,
where the draws are handed back to the sweep.

That choice is affordable only because of what the cascade **reads**, which is
worth enumerating rather than discovering:

| needs | from | have it? |
|---|---|---|
| `K_c(T, P)` | vapour-pressure law over `tb` | fork 2 |
| molar mass (mass ⇄ mole) | `PseudoComponent::molar_mass` | yes |
| liquid `cp` (tray enthalpy) | `PseudoComponent::cp` | yes |
| `Δh_vap` (duties only) | derived from `tb`, M7.4 | fork 5 |
| **vapour density** | — | **not needed** |
| **vapour `cv`** | — | **not needed** |

The last two rows are the load-bearing ones: they are the only reasons the
internal vapour would need `core`'s property functions, and they are needed only
by **tray hydraulics** — pressure drop per tray, weeping, flooding. Deferring
tray hydraulics (uniform pressure across the column, which is what "fixed
operating pressure" already asserts) is therefore not a convenience; it is the
condition under which fork 1's verdict is safe. If a later slice wants flooding,
it must revisit **this fork**, not just add a correlation.

Mass ⇄ mole conversion at the boundary deserves its own line, because M4.2's
real crux turned out to be units rather than the ODE. Compositions in this
workspace are **mass** fractions; vapour–liquid equilibrium is **molar**. The
conversion `n_c ∝ w_c / M_c` runs at exactly two points (feed in, draws out) and
is a gate of its own (below), not a step trusted to review.

#### Fork 2 — which seam owns the K-value. **`ThermoModel`, as reserved.**

`traits.rs` carries `ThermoModel` with no methods and a docstring saying to
extend it "when a consumer actually needs a property, not before". A K-value is
a thermophysical property of a component at `(T, P)` — not a separation policy —
so it is that consumer, and putting it anywhere else would create two seams both
claiming the same physics. The cascade itself (how many stages, which is the
feed stage, what a reflux ratio means) is separation policy and gets a new
`SeparationModel` in `core`, implemented in `solvers`, selected by
`[fidelity] separation = "cut_point" | "cascade"` alongside the existing `flow` /
`thermo` / `reactions` strings.

Two consequences follow from the `ReactionModel` precedent and should be built
in from the start rather than churned later:

- **The trait signature is designed for the complex fidelity now.**
  `ReactionModel::react` carries `tau`, which the lookup fidelity ignores
  entirely, precisely so M4.2's swap needed no trait churn. `SeparationModel`
  does the same: it takes the feed *flow*, *temperature* and the column's
  pressure — all three ignored by the cut-point splitter, all three load-bearing
  for the cascade — and returns per-draw split, composition, **temperature** and
  the column's duties. The simple impl returns the feed temperature for every
  draw and zero duties, which is exactly what it means today.
- **The threading is already proven.** `reactions` reaches the sweep as
  `&dyn ReactionModel` through `energy::resolve_node_states`; `SeparationModel`
  follows the same path to the two call sites that matter —
  `energy::edge_composition_at` (four callers) and the post-sweep draw write in
  `Engine::tick`. `column_separation` has exactly those two callers today, which
  is what makes the seam a small, bit-identical move rather than a refactor.

Per-column *equipment* (stage count, feed stage, reflux ratio, condenser type)
is per-node config, not per-engine, so it lands on `NodeKind::Column` — and the
"authoritative number nothing reads" problem it raises already has this
workspace's answer twice over. `PseudoComponent::density` is required iff the
component is liquid; a gas valve's `x_T` is required iff it is in gas service
(`require_gas_valve_x_t`). The same **declared-iff-used** loader correspondence
applies here: cascade fields required iff `separation = "cascade"` and refused
otherwise, and `ColumnDraw::upper_cut` inverted — required by the splitter,
refused by the cascade, which locates a draw by **stage** instead. Neither
fidelity can then carry a field the other silently ignores.

#### Fork 3 — what specifies the column. **Ratios only, and this is M3.2's argument again.**

A column with fixed pressure, a known feed, N stages and a known feed stage has
two remaining degrees of freedom. The textbook pairs are `(R, D)` — reflux ratio
and distillate **rate** — or `(R, V)` — reflux and boilup rate. **Any spec
containing an absolute flow is inadmissible here, and the reason is already
written above.** M3.2 killed the column whose draws were prescribed from the
previous tick's feed: the total through a column must stay *hydraulically*
determined, and only the *split* may be composition-determined. A specified
`D = 3.0 kg/s` re-runs that failure exactly — it either freezes the feed (free
column) or creates mass in a zero-volume node (fixed-pressure column), and it
does it while converging, conserving and rerunning bit-identically.

So the admissible specification set is **dimensionless**: reflux ratio `R = L/D`
(molar, internal) and distillate-to-feed ratio `D/F` (**mass**, at the boundary —
correction 1), plus one `S_i/F` per side draw. Bottoms is
then `1 − D/F − Σ S_i/F` and never specified. `Σ splitᵢ = 1` survives as an
identity of the *specification* rather than of the arithmetic, which is what
keeps the column mass-neutral every tick under a feed that moves — the property
the M3.2 note calls out as needing a moving-feed gate to earn.

This also settles what an operator command can touch: `R` and `D/F`, which are
real control-room handles, and never a product rate in kg/s.

#### Fork 4 — I7 stops being free, for the first time in this workspace

M3.2's note states that a splitter conserves every component identically, so I7
is **green by construction and has no discriminating power over a column at
all**. That claim is fidelity-specific and it expires here. A converged cascade
balances per-component only to its convergence tolerance; an unconverged one
does not balance at all. The options:

- **Force it exact** by assigning the last draw the residual (`bottoms = feed −
  Σ others`, per component). Exact by construction, and it can hand back a
  **negative** mass fraction when the cascade is off — trading a visible
  tolerance for an invisible corruption in the one product a refinery cares
  least about checking. Rejected.
- **Bound it.** Take the cascade's compositions as computed, normalize each draw
  to `Σ_c = 1`, take the draw flows from the ratio spec (so *total* mass stays
  exact — see correction 1, which supplies the mechanism this line originally
  only asserted), and make the per-component residual a **gated `Err`** with the
  residual in the message.

**Bound it, and the bound is not free to choose.** I7's own tolerance is
`COMPONENT_MASS_TOLERANCE_KG = 1e-6` kg over a 0.1 s tick — 1e-5 kg/s — chosen
absolute for the M1 acceptance-gate reason, with the stated rationale that
"per-component balance can never be tighter than the total mass balance it
partitions". The cascade's convergence tolerance is therefore **derived from
that number and the column's feed rate**, not picked to make a test pass. And
the residual must be a *gate*, not a diagnostic: a reported number nobody
asserts on is the shape `a-counter-is-not-a-gate` records, so the threshold
needs a test that reaches it deliberately (a cascade capped at too few
iterations must `Err`, and must be shown to).

Note also what this un-blocks: I7's generator builds only
Source/Junction/Tank/Sink today, so no I-series invariant has ever reached a
column of either fidelity. Extending it to reach a cascade is a real arm with
real discriminating power — and per `a-generated-arm-can-be-born-vacuous`, it
needs a reachability count before it is believed.

#### Fork 5 — a solver inside the tick loop can fail. **`Err`, and warm start is not state.**

The simple column cannot fail; a Newton cascade can. Rule 5 says a diverging
solver returns an `Err` with diagnostics, and the flow solver already does
exactly that (I3: divergence is legal). The alternative — hold the previous
tick's converged profile and carry on — is worse than it looks: it makes the
column's output depend on tick history, which destroys the property M3.2 leans
on ("zero-volume is what makes that hand calculation clean — no tick history in
it") and turns every reference gate into a run-length-dependent number.

`Err` it is. What is *not* forbidden is seeding the cascade from the previous
tick's profile: a **warm start changes the iteration count, not the fixed
point**. That distinction is the whole of it — a converged answer is the same
answer within tolerance from any admissible start, so the reference gate must be
run cold and additionally shown to be start-insensitive (same answer within
tolerance from a perturbed seed). Determinism is unaffected either way: the warm
start is itself deterministic.

#### How fast is fast enough (M9.3a) — the trigger fork 5 never wrote

Fork 5 licensed the warm start and named no condition for building it, which is
how `DEFERRED.md` A1 came to sit past a trigger that did not exist. M9.3a wrote
one, and the number comes from the only frontend with a clock.

The Godot binding does not tick itself — `RefinerySim` deliberately leaves that
to the scene, which calls `tick()` from `_physics_process`. That is **16.7 ms at
the default 60 Hz**, and it is a whole-frame budget the simulation shares with
everything else the game does. A refinery may plausibly carry several columns,
so the budget for **one** cascade column is about **2.5 ms per tick** — a sixth
of a frame. That is the trigger: one column costing more than that is over
budget, whatever the corpus's total says.

Two things make this a usable trigger rather than a slogan. It is **per column,
not per corpus** — the corpus runs one plant at a time and its total is an
artefact of which files happen to be shipped, while a frame budget is a property
of the frontend. And it is measured as a **ratio within one session**: this
machine drifted 1.7× slower in a single day (the same code measured 10 268 ms in
the morning and 17 228 ms in the afternoon), so an absolute millisecond figure
copied from an old note means nothing. Run the corpus, divide the plant's tick
wall time by the ticks, and put an unrelated plant beside it as a control.

Against that bar, at the time of writing: **1.10 ms per tick**, down from 2.87.
Inside the trigger — but the margin is 2.3×, not an order of magnitude, and the
cost scales with the stage count and the slate size, both of which a scenario
file chooses freely. A1 stays open with the trigger now attached to it.

#### What M9.3a changed, and the one number it did not touch

The bubble point is the cascade's inner loop: one per stage per outer iteration.
It ran a **fixed 60 bisection steps** regardless of the answer. It now runs
regula falsi under Illinois weighting with Brent's two-step bisection safeguard,
on `ln Σ K·x` rather than `Σ K·x − 1`, and stops when the bracket closes to the
float spacing — **15 evaluations** on the reference fixture.

**The logarithm is the whole of the speed-up, and the safeguard is not.** Both
knobs were swept independently: with the transform every safeguard variant costs
15 or 16 evaluations, and without it none costs less than 38. A K-value is
exponential in temperature, so `Σ K·x` spans thirty-eight orders of magnitude
across the `[50, 2000]` K bracket on the shipped slate — the case regula falsi
is famous for crawling on, and taking its logarithm is what makes the secant
productive. The safeguard is kept for the **worst-case bound** behind
`BUBBLE_POINT_MAX_EVALUATIONS`, which is a guarantee about a slate nobody has run
yet, not a number on this fixture. Anyone tuning it for speed is tuning the wrong
knob, and the code says so.

**The resolution could not be loosened, and the deferral that said it could was
wrong in an instructive way.** A2 proposed cutting the step count to the ~22 that
meets the cascade's own `1e-6` relative convergence test. But that test
*differences two bubble-point outputs*, so the root finder's resolution is a
**noise floor on the test that grades it** — it must stay far below the
tolerance, not meet it. Speed had to come from the method.

**What did not move is the outer iteration count: 200 000 over 6 000 ticks,
33.3 per tick, identical before and after.** That is the measurement that keeps
A1 open and points at the warm start. M9.3a made each iteration cheaper — bubble
points fell from 90.7% of the loop to 77.3%, 14 288 ms to 4 686 ms — and a warm
start is the change that makes the iterations *fewer*, which multiplies all three
regions rather than one. Attribution after the change: bubble points 77.3%,
K-value profile 6.6%, Thomas sweeps 7.3%, the rest 8.8%. `flash.rs`'s own
bisection does not appear: it runs once per solve, for the saturated-liquid feed
guard, not once per outer iteration.

Cost of the change, since fork 5 requires the fixed point to be unmoved: exactly
one of fourteen plants moves against the corpus baseline, on both fidelities, by
at most **1.31e-14 relative** on any physical quantity across 600 snapshots.
Measuring only the final snapshot would have understated that 3.3× — the worst
deviation is a transient at tick 270, not the settled value. From here, "runs
byte-identical" means post-M9.3a identical for `crude_column_cascade` under both
`newton` and `simple`.

#### The warm start (M9.3b) — and the half of the profile fork 5 does not name

Fork 5 licenses seeding "from the previous tick's profile" and M9.3a's
measurement said what to spend it on: the cascade's outer iteration count did not
move, 33 to 38 per tick depending on the run, every tick re-deriving an answer
that had barely changed. This is the slice that spends it.

**The word "profile" hides the finding.** A stage cascade iterates two profiles at
once — the stage temperatures and the stage liquid compositions — and `stage_t`
is the one that reads as "the profile": it is what `with_seed_offset` perturbs,
what the convergence message names, and what a reader of fork 5 would reach for.
Seeding it alone is nearly inert. Measured on `crude_column_cascade` over 6 000
ticks:

```text
                                      outer iterations   per solve   wall
  cold (M9.3a)                                 227 962        38.0   ~8 081 ms
  warm TEMPERATURES only                       209 968        35.0   ~7 894 ms
  warm temperatures AND LIQUID                   6 036       1.006     ~408 ms
```

An 8% saving that vanishes into the noise of a wall-clock measurement, against a
97% one. The reason is that the outer test is a **conjunction** over
`profile_change` (the change in stage liquid compositions), `temperature_change`
and the per-component residual. Seed the temperatures and leave `liquid` at
`vec![feed.fractions(); stages]` and the composition profile still walks in from
cold every tick — it was the binding criterion all along, and the temperatures
were riding along behind it.

**So `CascadeProfile` is one struct behind one `Option`, and that is the
measurement expressed as a type.** Two independent `Option` fields would make
"temperatures without compositions" representable, and that state is not a
partial warm start — it is the configuration just falsified. The precedent is
`condenser_duty` and `reboiler_duty`, which travel together for the same kind of
reason: a model that knows one knows both.

**It needed no engine state, which decided the fork.** `resolve_node_states`
already receives the previous tick's `NodeStates`, and its `column_separation` is
already a `BTreeMap<NodeId, Separation>`. So the seed is one lookup, the profile
rides `Separation` out and `ColumnPass` back in, and `SeparationModel::separate`
keeps `&self` and its "a function of `pass` alone" contract — literally, because
the history is an argument rather than state on the model. The alternatives
(`&mut self`, or interior mutability) both falsify that sentence and both put
per-column state on an object the engine holds ONE of for every column on the
plant, which would cross-seed two columns on the same tick.

**1.006 iterations per solve is the number that needed an adversarial gate, not
a celebration.** Converging on the first pass is what a right seed looks like and
also what a criterion that stopped binding looks like. The distribution over
6 000 solves is 38 once (tick 1, before any profile exists), 1 for 5 998 ticks,
and one zero-flow early return — so the criterion demonstrably binds when the
seed is ABSENT. What no shipped plant produces at steady state is a seed that is
present and WRONG, so that is what the gate builds:
`a_warm_start_from_the_wrong_profile_still_lands_on_the_cold_answer` solves a
light feed cold, hands its converged profile to a heavy feed, and requires the
heavy feed's own cold answer back. Two controls are asserted BEFORE the
comparison, because without them the gate is passed by a solver that ignores its
seed and equally by one that ignores its feed: the two profiles must differ by
more than 1 K on some stage, and the two distillates by more than a thousand
times the converged tolerance.

**A seed of the wrong shape is ignored rather than reshaped**, and that has its
own gate. It is contracted a hint; the only way to get a wrong-shaped one is a
plant edited under a live node id; and a padded or truncated profile is a start
no column ever had, which is worse than the feed's own bubble point.

**Cost, and it is not what the fixed-point worry predicted.** The concern going
in was that a warm start moves the answer by the outer tolerance rather than by
the ULP — roughly eight orders more than M9.3a — and that a cascade feeding tanks
would let that accumulate. Measured over 600 snapshots on both fidelities: the
worst move on any quantity above 1e-3 is **7.6e-07**, temperatures move 1.7e-08
(1.1e-5 K on 632 K), duties 1.5e-08, pressures at the ULP, and tank masses are
**bit-identical** — a draw's rate is a mass ratio of the feed, so an inventory
does not depend on the temperature profile at all. The decisive number is that
the drift is **flat across all ten deciles**, first equal to last, on both
fidelities: a tolerance-ball reseat, not accumulation. The full warm start is
also CLOSER to the cold answer than the temperature-only one (7.6e-07 against
1.35e-06), because a better seed converges nearer the true fixed point — the
more aggressive change is the more faithful one.

**One sentence in the non-convergence diagnostic had to be re-premised, not
deleted.** It said "the profile is not held over from a previous tick", which is
now false as written and sits in the message a reader gets when a solve fails.
The distinction it was reaching for survives: a previous profile may SEED a solve
and is never its ANSWER, and a solve that fails publishes no profile at all, so
the next tick starts cold. The message says that instead.

**One mutation is uncaught, deliberately, and it is this slice's own finding.**
Reverting the liquid half of the seed — the 8%-inert configuration — fails
NOTHING in the workspace, because a half-warm start is still correct, merely
slow, and no test measures a cascade's iteration count. Breaking the convergence
test is caught by seven tests including the new warm-start gate, and dropping the
seed's shape check is caught by exactly the gate written for it; but the edit
that would quietly undo 97% of the speed-up is invisible to the suite. Recorded
rather than papered over: a gate for it would have to assert an iteration count,
which is a cost measurement rather than a correctness one, and this project has
three times called a bound fitted to today's plants a fitted test. The defence is
the type — `CascadeProfile` makes the half-warm start unrepresentable, so the
edit has to delete a struct field rather than forget a line.

#### Energy — latent heat cancels, and that is what buys the scope boundary

Every enthalpy in this workspace is **sensible-only** against a shared datum, and
M4's lesson (`datum-consistency-in-a-holdup-balance`) is that mixing datums hides
until a first integral catches it. A column vaporizes and condenses, so the
question is whether latent heat forces a datum change.

It does not, under one condition. With a **total** condenser and all-liquid
draws, every kilogram vaporized inside the column is condensed inside the
column. The internal latent flows cancel identically, and the external balance
is purely sensible:

```
Q_reb − Q_cond = Σᵢ ṁ_drawᵢ·h(T_i) − ṁ_feed·h(T_feed)      (sensible, one datum)
```

The individual duties are *not* sensible — `Q_reb ≈ V̄·Δh_vap` is mostly latent,
and it is the number a game cares about (fuel, cooling water). They are
**emergent diagnostics**, exactly as `energy::reactor_duty` is emergent rather
than a configured field, and per M4's lesson the energy gate is the **difference
of the two duties**, not either one alone.

The condition is the scope boundary, and it is refused at load rather than
half-supported — the same move M3.2 made for a valve on a draw line and for
chained columns:

- **partial condenser** (vapour distillate) — refused;
- **vapour side draw** — refused;
- **flashing feed line** — refused by the existing single-phase
  connected-component guard, unchanged;
- **subcooled reflux / superheated feed** — allowed, they are sensible terms.

Each refusal names fork 0's narrowed deferral in its message, so the trigger and
the guard cannot drift apart.

One further `core` change follows: the draws now leave at **their tray
temperatures**, not the feed temperature. M3.2's note already flags this as the
gap ("a real column's draws sit at their tray temperatures; representing that
needs the reboiler/condenser duties and a tray cascade, which is the complex
column"). `edge_temperature_at` today reads one resolved temperature for the
upwind node and returns it unchanged at the upwind end — so it gains a column
arm, mirroring exactly what `edge_composition_at` already does for "which draw
am I". The rule stays in one place, which is why that function exists.

#### What the tests must pin — three families, and conflating them proves neither

The strongest gates here are derivable from first principles, so none of them
needs a transcription from a paywalled table (`published-anchor-envelope`).

1. **The cascade algebra**, through a relative volatility supplied by the test.
   At **total reflux** a binary with constant `α` over `N` stages must satisfy
   **Fenske** exactly: `(x_D/(1−x_D))·((1−x_B)/x_B) = α^N`. A single stage must
   reproduce a hand-computed **Rachford–Rice** flash. `α = 1` must produce no
   separation at any `N` — the null gate. All three are green under a *wrong
   vapour-pressure correlation*, which is the point of separating them.
2. **The K-value correlation** on its own. Two parts, and they must not be one
   test: the **exact** identities (`K = 1` at `T = tb` and reference pressure;
   `K` monotone in `T`; a heavier cut has the lower `K` at fixed `T`), and the
   **magnitude**, which — since the slate carries only `tb` and a
   Clausius–Clapeyron/Trouton form derives `Δh_vap` from it — can only be an
   **envelope** against a vapour pressure actually read, labelled a regression
   lock rather than validation, per the ceiling M4.2 already set.
3. **The unit as wired**: the loader building a cascade column, a real solve
   handing it a feed flow the file never states, `Σ draws = feed`, the duty
   *difference* against the sensible balance, per-draw temperatures ordered
   top-to-bottom, and every refusal above actually refusing.

Plus the two that are neither: the **mass ⇄ mole round trip** at the boundary
(fork 1), and the **non-convergence `Err`** (fork 4), each reachable on purpose.

And the seam's own gate is a *regression anchor*, not a reference: moving
`column_separation` behind `SeparationModel` must leave every existing golden
**bit-identical**, which is the standard this workspace has held since M3.1's
1-component anchor.

#### Corrections to this note, before a line of cascade code

Every earlier note in this file was corrected by building it. Six of these were
found by review of the note itself, which is cheaper, so they are recorded the
same way rather than folded in silently. **No verdict moves; two of them change
what M7.1 builds.**

**1. Fork 3 asserted an exactness it did not supply a mechanism for — and the
hole is at the mass ⇄ mole boundary fork 1 flags.** "Total mass stays exact" is
only true once the *basis* of `D/F` is stated. A reflux ratio is conventionally
**molar**, and if `D/F` were molar too, then `D_mass = F_mass·(D/F)·(M̄_D/M̄_F)` —
the mass split would depend on the distillate composition still being solved
for, so total mass would close only *at convergence*, silently inheriting fork
4's tolerance instead of being exact. **`D/F` and every `S_i/F` are MASS ratios**,
declared at the boundary where SI mass is the workspace's basis (rule 4); bottoms
is `1 − D/F − Σ S_i/F` by subtraction, so the splits sum to 1 by construction and
`Σ ṁ_draw = ṁ_feed` exactly, from the same defensive normalization M3.2 already
names. `R` stays molar and stays **internal** — it never crosses the trait
boundary. The cascade's own molar distillate rate is then not an input but part
of its fixed point, which is a real inner coupling and is stated here rather than
discovered in M7.3.

The asymmetry that makes this safe is worth keeping next to fork 4's verdict:
normalizing *total* splits is benign, because every split is ≥ 0 and it is
exactly what `column_separation` already does defensively — whereas forcing
*per-component* exactness by residual assignment can go negative, which is why
fork 4 rejected it. Same instinct, opposite answer, for a reason.

**2. Fork 2's trait signature could not reach a K-value — and `thermo` reaches
nothing today.** The signature specified there (feed flow, temperature, pressure
in; split, composition, temperature, duties out) gives the cascade no way to call
the `ThermoModel` that fork 2 just put the K-value on. By fork 2's own `tau`
argument that parameter belongs in the signature at **M7.1**, or M7.3 churns the
trait — the outcome fork 2 claims to avoid. So `SeparationModel` takes
`&dyn ThermoModel`, which the cut-point splitter ignores exactly as it ignores
the feed flow.

That is a **threading** change and not only a signature one, which is the part
worth checking before M7.1 starts: `Engine` holds `thermo: Box<dyn ThermoModel>`
carrying `#[allow(dead_code)]` and it has **zero call sites** in the workspace —
the slot has been reserved and unread since M1. `reactions` was reserved the same
way and is threaded to the sweep, so the path is proven; but M7.1 is the commit
that first makes `thermo` a live dependency, and it must say so.

**3. The Fenske gate has an off-by-one hiding in a convention.** `α^N` is exact
at total reflux only once `N` is defined, and the textbook `N_min` **counts the
reboiler as an equilibrium stage and does not count a total condenser** — so a
column of `N` trays plus a reboiler gives the exponent `N+1`. A gate called
"exact, derivable" that passes or fails for an off-by-one is worse than no gate,
so **`N` in this workspace counts equilibrium stages including the reboiler and
excluding the total condenser**, stated here, asserted in the test, and named in
the `NodeKind::Column` field doc.

**4. Two gates in family 2 must stay apart, and here is the sharp reason.**
"Separate the correlation gate from the cascade gate" was argued from coverage;
the stronger form is that `K = 1` at `(tb, P_ref)` is exact **and independent of
the Trouton constant** — a Clausius–Clapeyron form integrates *from* that anchor,
so the identity holds for any value of it. Same for the ordering property: under
`Psat = P_ref·exp[(C/R)(1 − tb/T)]`, `K` is monotone decreasing in `tb` for every
`C > 0`. So the exact identities cannot detect a wrong `C` **even in principle**,
and only the envelope can. One test covering both would not merely be weak; it
would be structurally incapable of the catch.

**5. "Superheated feed — allowed, they are sensible terms" is too glib under
constant molar overflow.** Feed quality `q` sets the internal flows
(`L' = L + q·F`), so a subcooled or partly-vaporized feed changes the *cascade*,
not just an enthalpy term. **M7.3 takes a saturated-liquid feed only** and
refuses the rest at load; `q` as a parameter is deferred with the rest of the
list below. The tension a reader will trip on, resolved in one line: a
*superheated* feed flashes at the feed stage, which is inside the unit and
therefore fine under fork 1 — what is refused is a feed line carrying two phases
in a `Pipe`, which is on the graph.

**6. The Trouton framing overclaimed, in exactly the shape this repo hunts.**
M7.2 said `Δh_vap` from `tb` is "derived from data the slate already carries, not
invented parameters". Half true: `tb` is slate data, but **Trouton's constant is
one empirical fitted number and is not**. The gate structure already concedes it
(exact identities plus an envelope for magnitude), so the framing must too. The
honest claim is narrower and still enough to distinguish this from M4's refused
case: **one constant, carrying a published envelope and an exact identity that
does not depend on it** — where M4's refusal was of parameters with *no* gate of
either kind.

**And one committed claim that survived checking.** "No I-series invariant has
ever reached a column of either fidelity" was written from `composition_transport.rs`
alone and generalized. It is true: `Column` appears nowhere in
`crates/solvers/tests/` at all — not in `invariants.rs`, the gas-valve arm, or
any other. Fork 4's framing stands.

#### Corrections from building it (M7.1, landed)

The seam is in: `SeparationModel` in `core::traits`, `CutPointSplitter` in
`solvers::separation`, `[fidelity] separation = "cut_point"` in the loader, and
every one of the twelve scenarios byte-identical over 300 ticks. Four things the
note had wrong or unstated, and one measurement that changes what M7.4 owes.

**1. The MODEL reaches one call site; the RESULT reaches the two consumers.**
Fork 2 said `SeparationModel` "follows the same path to the two call sites"
(`edge_composition_at` and the post-sweep draw write). That is the wrong shape,
and the deciding argument is not cost — it is that **a per-edge call has nowhere
to put the duties**. `Separation` carries condenser and reboiler duties; a model
invoked from inside `edge_composition_at` computes them once per draw edge per
reader and stores them zero times. The `ReactionModel` precedent says the same
thing for a second reason ("`react()` is called once per reactor per tick …
rather than re-integrating per outlet edge, as a naive `edge_composition_at` hook
would"), which at the cut-point fidelity merely wastes work but at M7.3 would
re-solve a stage cascade four times a tick.

So `separate` runs **once per column, in the composition sweep**, exactly where
`react` runs, and the result is stored in `NodeStates::column_separation`
alongside `reactor_duty` — which the note already called "the one extensive
quantity resolved on this sweep", and which is now the general shape rather than
the reactor's exception. Both consumers read it.

Two consequences worth stating. The trait object threads to **one** function; what
threads to the readers is a plain data map (`edge_composition_at`, `stream_cp_at`,
`edge_temperature_at`, `inflow_totals`, `mix_inflows`, `exchange_pair`,
`reactor_duty` each gained one parameter), which is strictly less plumbing than
passing `&dyn SeparationModel` plus the temperature map into the composition mix.
And M3.2's central worry — the flow split and the composition split must come
from the same pass or per-component mass fails at the column — stops being a
discipline two call sites keep and becomes **structural**: there is no longer
anything to recompute.

**2. Making `thermo` live was the cheap half.** Correction 2 flagged the threading
as "the part worth checking before M7.1 starts", and it was: the slot had zero call
sites since M1. But handing `&dyn ThermoModel` down to one function is three lines.
The real work of this slice was the map above, which the note did not anticipate at
all because it had the call shape wrong.

**3. `Separation::draws` is checked against the column's draw list, not indexed
into.** The two are parallel *by contract*, and a trait contract kept by an impl in
another crate is exactly what rule 5 says not to trust with a `[]`. A model
returning a short list now errors by name in both readers — and because the only
implementation in the workspace can never violate that contract, both guards were
**unreachable by every existing test**. `solvers/tests/separation_contract.rs`
breaks the contract on purpose with a stub returning one draw for a two-draw
column, and each guard was verified by removing it and watching its test fail. A
guard cited in a design note as satisfying rule 5 and never once run is the
"a counter is not a gate" shape.

**3a. Which readers actually traverse the column arm, stated because the
threading suggests more than is reachable.** Seven functions gained the
separations parameter, but the arm inside `edge_composition_at` fires only where a
draw edge is read: `Engine::tick`'s transport loop and stream publish, and the
tank inflow loop (`stream_cp_at` / `edge_temperature_at` / `edge_composition_at`
on a product tank's inflow). It is **unreachable** through `inflow_totals`,
`mix_inflows` and `exchange_pair`, because those resolve *zero-volume* nodes and
the loader refuses a free node on a draw line — a draw's outlet must be a product
store, so a draw edge is never an inflow to a swept node. That threading is
therefore defensive: correct, compiled, and dormant until the day a draw is
allowed to feed a junction. Stating it beats implying the anchor covered it.

**4. The reverse-feed refusal stays in `Engine::tick`, and `ColumnPass::feed_flow`
is `0` rather than negative under it.** The sweep's inflow sum is the column's feed
(draw edges are guarded to zero in the solve), and a column running backwards has
no inflow at all. Keeping the guard where M3.2 put it preserves the error *and* its
timing exactly; the splitter never reads the flow, so nothing changes today. **M7.3
must revisit this**: a cascade WOULD solve on the zero and fail with a worse message
before the guard that names the cause is ever reached.

**5. The measurement: `smearing` is exercised by exactly ONE test in this
workspace, and it is the one that moved.** Shifting the ramp's centre by `1e-7`
(`+ 0.5` → `+ 0.5000001`) leaves all twelve scenarios **byte-identical** over 300
ticks and the entire suite green — except
`smearing_splits_a_boundary_cut_between_adjacent_draws`, the unit test that
travelled from `core::energy` to `solvers::separation` with the code.

The reason is arithmetic, not luck: **no component's boiling point lands inside a
ramp in any of the three column plants this repo has** — checked one by one, not
inferred from the mutation. `crude_column.toml` cuts at 458.15 K and 613.15 K with
`smearing_k = 25.0` (so ramps of 445.65–470.65 K and 600.65–625.65 K) over a slate
boiling at 353/423/493/573/673 K. `fcc_plant.toml` cuts at 293.15 K and 523.15 K
with the same smearing (ramps 280.65–305.65 K and 510.65–535.65 K) over a slate
boiling at 233.15/373.15/673.15/1173.15 K. The `column_reference` plant has the
same shape: cuts at 150/250 °C over components boiling at 100/200/300 °C, every
one at least 50 K clear of a 12.5 K half-width. Every weight is clamped to 0 or 1, so **the demo column is a sharp splitter
and its `smearing_k = 25.0` currently changes no number in any output.** This is
"a generated arm can be born vacuous" applied to a hand-written scenario.

It does not weaken this slice's gate — the anchor is sharply discriminating on the
*split* (perturbing one draw's split by `1e-7` relative moves `crude_column`'s
output and leaves the column-free scenarios alone, which is what proves the anchor
is not vacuous) — but it does mean the ramp's only coverage is a unit test, and
that **M7.4's demo scenario should place at least one cut inside a ramp**, where
the two fidelities are also most worth comparing.

*(Closed in M7.4c, and by moving THIS file's cut rather than by adding one to the
new demo — a cascade column has no smearing to exercise, so a new file could not
have satisfied it. `crude_column.toml`'s first cut moved from 185 °C to 155 °C, so
the heavy naphtha at 150 °C sits half a ramp width below it and goes 70/30. The
measurement above is what makes the closure checkable: disabling smearing was
caught by that one unit test and nothing else, and is now caught by two wired demo
gates as well.)*

**And one thing found next door, recorded rather than fixed here.**
`[fidelity] thermo` is parsed into a `String` that the loader never matches on:
`build_engine` hardcodes `ConstantThermo`, so `thermo = "nonsense"` loads happily
today. Every other fidelity string is validated with a list of valid names. M7.2
needs that arm to exist anyway (it selects a K-value fidelity), so it is recorded
against that box instead of widening this commit. *(Fixed in M7.2, on its own
commit — see correction 2 below.)*

#### Corrections from building it (M7.2, landed)

The K-value is on `ThermoModel`, the flash is in `solvers`, the mole boundary is
a type of its own, and the ignored `thermo` string is fixed. Three things the
note had wrong or unstated, and two measurements — one of which says something
the note's own correction 4 did not go far enough on.

**1. The K-value names its component by INDEX, not by `&PseudoComponent`.** The
ROADMAP wrote `k_value(component, T, P)`, which reads as the component itself,
and for a correlation that is fine — Trouton needs only `tb`. The constraint
comes from the *other* implementation. `ConstantAlphaThermo` carries one K per
slate position and identifies a component by **position**, exactly as
`SimpleLookup::fcc_demo(&slate)` resolves its lumps at construction; handed a
`&PseudoComponent` it would have to look itself up by name on every call, per
component per stage per tick. So the signature is
`k_value(&Slate, usize, Kelvin, Pascal) -> Result<f64, SimError>`. This is the
one part of the slice that would have been expensive to change later, because it
is in `core` and correction 2 above is literally a note about churning this trait.

**2. The loader arm and the fidelity it selects are two different slices, and
only one of them belongs here.** M7.2's box said to give `[fidelity] thermo` a
match arm "to select a K-value fidelity". Building it split the line in half. The
arm itself is a **defect** — `thermo = "nonsense"` loaded a working plant — and
landed on its own, before any new thermo existed. Making `"trouton"` *selectable*
is a different act, and it is deferred to M7.3: nothing reads a K-value until the
cascade does, so a scenario setting `thermo = "trouton"` today would change no
number in any plant. That is precisely the vacuous knob M7.1 measured on
`smearing_k` one section above, and adding a second one in the commit that
records the first would be hard to defend.

M7.3 therefore owes two things at that arm, not one: `"trouton"`, and the
**load-time refusal of `separation = "cascade"` with `thermo = "constant"`**.
Until then `ConstantThermo::k_value` returning `Err` is the only guard, and a
unit test is the only thing that reaches it.

**3. `ConstantThermo::k_value` refuses rather than returning 1.** Worth writing
down because `K = 1` is the tempting answer and it is the workspace's recurring
failure shape: finite, deterministic, plausible, wrong. A column running on it
would separate nothing and read as a physics result rather than a missing model.

**4. The measurement that extends correction 4: the envelope is blind too, over
most of its range.** Correction 4 established that the exact identities cannot
detect a wrong Trouton constant *even in principle*, so the envelope carries the
whole load. Building the envelope showed the load is not carried evenly. Near the
boiling point the Clausius–Clapeyron anchor pins `Psat` to `P_atm` **whatever the
constant is**: at 342.69 K a constant wrong by ±30% lands at ratios of 1.007 and
0.992 against the tabulation — inside any band the correct constant could pass.
All the discriminating power lives at the **cold end** of the tabulated range,
where the exponent has room to diverge (0.595 and 2.049 at 286.18 K).

So the gate's sample range is load-bearing, not stylistic, and the reference test
asserts *both* directions: that a wrong constant escapes the band at the cold end
**and** that it hides near `tb`. A later slice narrowing the samples toward the
boiling point — the obvious "simplification", since that is where the model is
most accurate — would leave a green test that catches nothing.

**5. The size of what one fitted constant costs, on the fluids that matter.**
Correction 6 conceded that Trouton's constant is not slate data without saying
how wrong it makes things. Measured against NIST's Antoine coefficients for
n-hexane over their whole stated validity range: **within about 10%**, worst at
the cold end, exact at the anchor by construction. On water it is about **60%
high at 50 °C** — Trouton's rule is poorest for hydrogen-bonding fluids, whose
true `Δs_vap` is nearer 109 than 88 J/(mol·K). A crude slate is hydrocarbons, so
the hydrocarbon number is the relevant one; the water number is recorded because
the workspace's one-component default slate *is* water, and a future reader
flashing it should know what they are holding.

**5a. Bit-identity is structural here, not measured — and that is worth saying,
because measuring it is this workspace's habit.** M7.1 earned its claim by
running all twelve scenarios for 300 ticks and comparing bytes. M7.2 does not
need that: `build_engine` still hands `Box::new(ConstantThermo)` to every file,
the loader arm only adds a refusal that all twelve files pass, and everything
else is a new module no running plant reaches. There is no path from any scenario
to a K-value, so there is nothing for the arithmetic to differ about. When M7.3
makes `"trouton"` selectable, that stops being true and the measurement comes
back.

**6. The mass ⇄ mole round trip is necessary and not sufficient, measured.** The
note lists it as a gate of its own. It cannot catch the bug it exists for:
writing `n ∝ w·M` instead of `w/M` in **both** directions round-trips exactly.
Both mutations were run — one-sided fails the round trip, two-sided passes it and
fails only a hand-computed mole-fraction vector. On a 50/50 mass mixture of
components 2.75× apart in molar mass, the wrong rule returns *precisely the other
component's answer*: the two rules swap the mixture end for end, and both sum to
1. The hand calc is the gate; the round trip catches a one-sided slip and a lost
component, which is worth having and is not the same claim.

#### Corrections from building it (M7.3, landed)

The cascade is in: `StageCascade` in `solvers`, `CascadeSpec` and the stage-located
`ColumnDraw` on `NodeKind::Column`, `thermo = "trouton"` and
`separation = "cascade"` in the loader with the declared-iff-used correspondence
in both directions, and all five gates. **No verdict moved.** Six things the note
had wrong or unstated, three of them measurements.

**1. `ConstantAlphaThermo` could not do the job it was written for — and the note
says so in its own words.** M7.2 shipped it "so a separation gate can be written
against cascade *algebra* with the correlation held out of it", and this section
names that gate as family 1. Building family 1 falsified the claim. A stage's
temperature IS its bubble point, `Σ_c K_c(T)·x_c = 1`, and with `T`-independent
K-values that equation has **no root at all**: the left side does not depend on
`T`. The M7.2 type could not have driven one stage.

The fix is the narrowest one that keeps the claim true: an optional
`K_c(T) = k_c·(T/T_ref)^n`. Relative volatility is `α_ij = k_i/k_j` for **any**
`n`, because the scaling is shared by every component and cancels — so Fenske's
`α^N` stays exact and no correlation enters. `n = 0` is bit-for-bit the shipped
behaviour, which is why the M7.2 flash gates still compare with `assert_eq!`.

The alternative considered and rejected: have the cascade *detect* a flat model
and hold some deterministic convention, as `flash_isothermal` does for the
all-`K = 1` case. It does not transfer. The flash's degenerate case is
recognisable from the K vector alone; a missing bubble point is not — a detector
cannot tell "K does not depend on `T`" from "the root is outside my bracket", and
the second case is finite, deterministic, plausible and wrong. **A stage whose
bubble-point equation has no root in the bracket is an `Err` naming the model**,
and the flat model reaching that arm is a gate of its own.

**2. Fenske cannot be evaluated AT total reflux under fork 3's own
specification, so the gate is a limit plus a bound.** Total reflux is `D = 0`,
`R = ∞`, `F = 0` — not expressible by ratios, and not a well-posed
boundary-value problem either (the profile is then determined only up to scale),
which is why the textbook states Fenske as a *limiting* relation. The gate the
ROADMAP calls "exact, derivable" therefore lands as two claims:

- **`ratio ≤ α^N` at every `N` and every `R`, exactly.** Non-asymptotic, and it is
  the half that polices correction 3's stage convention: a cascade that
  *under*-counted would be claiming `α^(N+1)` and would breach the bound by a
  whole factor of `α`.
- **First order in `1/R`.** At finite reflux `y_{j+1} = (R·x_j + x_D)/(R+1)`, which
  differs from the total-reflux `y_{j+1} = x_j` by `O(1/R)`, so doubling `R` must
  halve the error. Gating the error *ratio* is sharper than any single tolerance
  at one large `R`, and it catches the *over*-counted convention, where the limit
  would be `α^(N−1)`, the error would stall near `1 − 1/α`, and the halving ratios
  would go to 1 instead of 2.

**Measured before the assertion was written**, per
`integrator-order-of-convergence`. Sweeping `R` from 40 to 20480 at
`N ∈ {2, 3, 5, 10}`, the halving ratio reaches 2 in different places: `N = 2` is
asymptotic from about `R = 160`, `N = 5` only from about `R = 320`, and **`N = 10`
is still at 1.95 at `R = 20480`** — the compounding over stages pushes the
asymptotic window out faster than `N` grows. So the order gate runs on
`N ∈ {2, 3, 5}` over `R ∈ [320, 5120]`, and the tall column is asserted on the
bound and the direction only, said out loud in the test rather than quietly
dropped.

**3. The stiff case is a PINCH, and it is not the coupling correction 1
introduced.** Successive substitution converges linearly here, and the iteration
cap had to be measured rather than guessed. A 10-stage, `α = 4`, `R = 5` binary
at `D/F = 0.35` converges in **20** passes; the same column at `D/F = 0.5` needs
about **1300**. The reason is physical: at that ratio the distillate is asked for
*exactly* the light component the feed contains, so the specification sits on a
pinch and both products approach purity. The obvious suspect was correction 1's
`M̄_D` coupling — and it is not: repeating the measurement on a slate whose two
cuts have **equal molar mass** gives the same two numbers, 20 and 1300. The cap is
set above the stiff case.

**4. The bottoms-rate guard is not about reflux, and it exists only because
`D/F` is a mass ratio.** The first attempt to reach it used a large reflux ratio,
on the reasoning that a hard-boiling column runs its reboiler dry. The algebra
refutes that: `V − L = D` telescopes through the column, so `B = F − D − Σ S` and
**no reflux ratio can dry the reboiler**. Nor can a converged split, since every
kilogram out came from a kilogram in and so the moles out cannot exceed the moles
in. What reaches the guard is an *iterate* outside the physical region, and only
because the draw ratios are MASS ratios: a distillate rich in light cuts can have
a mean molar mass small enough that half the feed's mass is more than all of its
moles. Found by search after the first version of its test failed to fire, which
is `a-void-mutation-looks-like-a-catch` in the shape of a guard rather than a
mutation.

**5. M7.1's correction 4 is discharged in `solvers`, not in `core` — and the
discharge is a CONVENTION, not a refusal.** That correction predicted a cascade
would "solve on the zero and fail with a worse message before the guard that names
the cause is ever reached", and it is exactly right: `separate` is called *inside*
`resolve_node_states`, while `Engine::tick`'s reverse-feed refusal fires after the
sweep, and at zero feed every internal molar flow is zero and the first stage row
is singular.

The first fix was to refuse a non-positive feed, and it was wrong in a way worth
recording. `column_feed_flow` reports a zero for **two** different states — a
column running backwards, and a column with nothing flowing at all — and only the
first is a fault. Refusing both kills a tick the moment an operator shuts a feed
valve, and it makes the two separation fidelities **disagree about which plants are
legal**: the cut-point splitter has always handled an idle column, since a fraction
of nothing is nothing. That is the hazard `ConstantAlphaThermo::k_value`'s own
comment names ("a model that silently accepts a state its sibling refuses…"), met
in the other direction.

So an idle cascade returns its declared splits, with the feed composition and
temperature as inert placeholders — nothing is carried anywhere at zero flow, which
is the same move the splitter makes for a draw whose band catches no component and
the same "decided, not discovered" move the flash makes for its all-`K = 1` case.
The genuinely reversed case then falls through to the engine's own guard, which is
precisely what correction 4 wanted reached. A negative or non-finite feed is still
refused; it cannot come from the sweep, so the only caller who can produce one is a
hand-built pass. The geometry is validated **before** the flow is looked at, so
which plants are legal never depends on how much is going through them. Nothing in
`core` moved — the same reachability argument
`reachability-decides-which-rule-wins` records, applied to a message rather than a
panic.

**6. The per-component residual had to be made *visibly* binding, or fork 4's
"gate, not a counter" would have been unearned.** The convergence test is a
conjunction of three criteria, so a failure message reporting only the residual
reads identically whether the residual was the binding constraint or merely along
for the ride — and a test asserting the message mentions a residual would pass
either way. The message names **each unmet criterion** instead, and the gate
asserts the residual is among them.

Two smaller things, recorded because they are decisions rather than
consequences. The draws report their **real tray temperatures** now rather than
waiting for M7.4: the cascade solves the profile anyway (a K-value needs a
temperature), so filling `DrawSeparation::temperature` costs nothing and returning
the feed temperature would be a number this model knows to be wrong — M7.4 is left
with only the reader. And both **duties stay `Watt::ZERO`**, because `Q_reb` is
mostly latent and `Δh_vap` is M7.4's box; unlike the splitter's zero that is a gap
rather than the honest answer, and the two are documented differently so nobody
reads across.

**And the measurement 5a promised.** M7.2 argued its bit-identity was structural
rather than measured, and said the measurement comes back when `"trouton"` becomes
selectable. It came back: all twelve scenarios, 300 ticks, every snapshot,
**byte-identical** against `5b1a71b`. The new `Option` fields on `ColumnDraw` and
`NodeKind::Column` carry `skip_serializing_if`, so a cut-point column's serialized
shape is unchanged too — which was the reason to add fields rather than replace
`upper_cut` with a sum type.

**What the mutation pass found.** Seven mutations, each verified to compile.
Reading K at the wrong stage, folding the condenser with `V` instead of `L`,
treating `D/F` as molar, taking the distillate as stage 1's *liquid* rather than
its condensed vapour, and letting a draw not leave its stage were each caught by
five or six gates. Loosening the residual bound by 10⁴ was caught by exactly one —
the wired plant's own per-component balance, which computes the check itself
rather than trusting the solver's number. The seventh was **not caught, and must
not be**: seeding the profile at the feed's resolved temperature instead of its
bubble point is a different *start*, and fork 5's whole claim is that the start
does not change the fixed point. That mutation surviving is the start-insensitivity
gate working.

#### Corrections from building it (M7.4a, landed)

The reader is in: `edge_temperature_at` has its column arm, and a draw leaves at
the temperature its separation gave it. **The note's shape survived exactly** —
"a column arm, mirroring exactly what `edge_composition_at` already does for
'which draw am I'" is what got built. Three things it did not say, one of them a
claim this slice made and then falsified.

**1. The mirror is one lookup, not two parallel arms.** The note says the
temperature arm mirrors the composition arm, which reads as *write a second arm of
the same shape*. Building it that way would have been the "two independently
computed effectiveness terms" mistake `edge_temperature_at`'s own docstring
rejects, one level down: two copies of "which draw is this edge" with nothing
forcing them to agree, and a disagreement would hand a draw its own composition at
a **different draw's temperature** — a state no plant can be in. So
`column_draw_at` returns the whole `DrawSeparation` and both readers take the
field they need out of it. The arms mirror because they are the same lookup, not
because they were written alike.

**2. The arm goes at the INLET resolution, not in the upwind early return.** The
obvious place is the `node == upwind` branch — that is the branch that means "the
fluid leaves the node at the node's own temperature", and a draw is exactly the
exception to it. Putting it there is wrong: the *downstream* branch then starts
`pipe_outlet_temperature` from the column's mixed feed temperature, transforming
an inlet the fluid never had. It is invisible in this workspace, where every
scenario's `ambient_ua` is 0 and the transform is the identity — the latent-bug
class that same docstring already warns about for tanks. The gate therefore puts a
live `ambient_ua` on a draw pipe, which is the only one in the repo.

**3. The claim that justified that gate was itself wrong, and the mutation said
so.** The gate was written believing a misplaced arm would slip past the
tray-temperature test entirely. It does not: `Engine::tick` stores a pipe's
temperature from its **downstream** end, so the early return is not the branch
that test reads, and the misplacement fails both. What the ambient gate actually
earns is narrower and still worth having — it is the only place the arm and the
transform are shown to **compose**, rather than the arm resolving a number the
transform then drops. Recorded rather than deleted, per
`a-void-mutation-looks-like-a-catch`: the reasoning is why the test exists, and a
future reader who deletes it as redundant should see what it does and does not
cover.

**The measurement that matters most is the negative one.** All twelve scenarios
are byte-identical over 300 ticks, which was expected — `CutPointSplitter` sets
`temperature: pass.temperature`, the same `f64` the sweep stored in the node map,
so the cut-point path is unchanged by construction. Confirmed rather than
asserted, and then falsified: a 1e-7 relative nudge to that field moves
`crude_column` and `fcc_plant` and leaves the other ten alone. **The same nudge on
the pre-change tree moves nothing at all — 0 of 12.** That is the difference
between "the field exists" and "the field is read", measured rather than argued,
and it is the cleanest available proof that this slice wired something.

**What the mutation pass found.** Four mutations, each verified to compile: the arm
absent; the arm misplaced to the early return; every stage draw reporting the top
tray (the off-by-one class, since the file's `stage` is one-based with `0` meaning
the condenser while the profile is zero-based); and the distillate's temperature
taken at stage 1's *liquid* rather than the vapour a total condenser condenses. All
four caught.

**And the bubble-point identity was never the sole catcher — which is worth saying
because the first draft of this paragraph claimed the opposite.** That identity is
the slice's centrepiece gate: each draw's temperature equals the bubble point of
the composition it carries, recomputed in the test from the published `k_value`
instead of by calling the cascade's private routine, and it covers both draw
locations at once because a distillate off a total condenser and a liquid off a
tray are both saturated liquids. The claim written first was that it *is* the gate
catching the off-by-one. It is not: that mutation also fails the wired plant's own
split-and-balance test, and the fourth mutation — chosen specifically to be
invisible to every balance, since it moves one temperature and no mass — was caught
by the M7.3 flash reduction as well.

So the identity's value is **independence, not unique coverage**: it is a second
implementation of the saturated-liquid contract that agrees with the cascade's own
to 1e-6, and a wrong profile has to fool two unrelated derivations rather than one.
That is a real thing to hold, and it is a smaller thing than "this gate is what
catches X". Recorded as measured, per `a-conjunctive-gate-hides-which-criterion-
bound` — a gate that fires alongside others has not been shown to bind, and saying
otherwise is the same error this section's correction 3 already caught once.

**Left open on purpose, and named because nothing goes red.** A column was
enthalpy-neutral by construction while every draw shared one temperature:
`Σ ṁᵢ·cpᵢ = ṁ·cp_feed`, since `cp` is linear in composition and per-component mass
is conserved. Differing tray temperatures end that, and the residual **is** the
net reboiler-minus-condenser duty M7.4b adds. Between the two slices the column's
external energy books do not close and no gate reaches the gap: I6's generator
builds only Source/Junction/Tank/Sink, and no scenario file selects the cascade
until M7.4c.

**A precondition violation surfaced with it, and it is not the same kind of open
box.** The wired M7.3 fixture feeds its column at 120 °C, which is *above* the
bubble point of its own 50/50 mix at 1.5 bar — the feed is superheated relative to
the column. Constant molar overflow assumes a **saturated-liquid** feed
(correction 5 to the note), and nothing enforces it, so the wired fixture has
always been off-model. This does not retract M7.3: Fenske, the null case and the
flash reduction all run on hand-built passes in `solvers`, not on this plant. But
"M7.4b will notice" understates it — an unclosed balance is a missing feature,
while this is the formulation being fed something it does not admit.

**M7.4b owes the runtime guard, not a tolerance in the docs.** `separate` returns
`Err` when the feed is off its bubble point by more than a derived bound, and the
fixture's feed moves onto its bubble point in the same slice so the wired gates
measure an admissible column. The bound is derived from the enthalpy error the
superheat represents, not picked to pass.

That refusal makes the cascade reject plants the splitter accepts, and the
asymmetry is deliberate — which is worth stating next to correction 5, where the
opposite call was made. An **idle** column is a state the cascade *can* answer (a
fraction of nothing is nothing), so refusing it would have made fidelity change
legality for no reason. A superheated feed is a state it *cannot* answer, and the
rule for that is already in this workspace: `ConstantThermo::k_value` errs rather
than returning a plausible number. Refuse what you cannot answer; never refuse what
you can.

#### Corrections from building it (M7.4b, landed)

The duties are in, `Δh_vap` is on `ThermoModel`, and a feed off its bubble point is
refused. **The note's shape for this half did not survive**, and the correction is
the largest one M7 has taken: the gate this box was specified with cannot exist.

**1. Two locally-exact duty envelopes are not available, and the note asked for
them.** The box says the gate is "the **duty difference** against the sensible
external balance (M4's two-duty lesson)". That reads as: compute each duty from
its own envelope, then check that their difference is the external balance. The
condenser end works — one stream in, one stream out, same composition, both
endpoints on the profile. The reboiler end does not, and neither does any interior
stage, because **constant molar overflow fixes `L` and `V` instead of solving each
stage's energy balance**. Every interior stage therefore carries an energy
residual, and those residuals accumulate down the column rather than cancelling.

Measured on the reference binary (`N = 6`, `R = 2`, `D/F = 0.35`), which is the
part worth having rather than the argument:

| | W |
|---|---|
| `Q_cond`, its own envelope (exact) | 3.141e6 |
| `Q_reb`, closing the external balance | 3.232e6 |
| the external sensible balance — their difference | 9.1e4 |
| `Q_reb` from its OWN full envelope | 3.681e6 |
| the formulation's inconsistency | 4.5e5 |

A locally-exact pair would have reported this column absorbing **0.54 MW** net
when its streams carry **0.09 MW** — wrong by 4.9× the quantity the balance
measures, and wrong in the direction that reads as a plant creating energy. That
is a worse diagnostic than a coarse one, so the pair is built the other way round:

```text
Q_cond = V·λ̄(y₁) + V·c̄p(y₁)·(T₁ − T_cond)          exact, local
Q_reb  = Q_cond + [Σᵢ ṁᵢ·cpᵢ·(Tᵢ − T_REF) − ṁ_F·cp_F·(T_F − T_REF)]
```

The condenser is determined by physics; **the reboiler is the duty that closes the
column's external energy balance**, which is the property a plant-level balance
needs and a locally-exact pair would not have. The cost is stated rather than
hidden: the reboiler duty carries the formulation's error and the condenser duty
does not.

**And that is why the box's own gate had to be replaced.** `Q_reb − Q_cond` is now
the external sensible balance *by construction*, so asserting that it equals the
external sensible balance is exactly the tautology M4's two-duty lesson warns
about, one level up — computing a quantity by a formula and asserting it satisfies
the formula. What replaced it, in order of how much physics each carries:

- **`α = 1`.** Give every cut the same K and everything collapses: one uniform
  temperature, so stage 1's dew point *is* the distillate's bubble point and the
  condenser's sensible term vanishes; every draw at that temperature with the
  feed's composition, so the external balance is identically zero and the two
  duties must be **equal**. What is left is `Q_reb = Q_cond = V·λ̄(z)` with
  `V = (R+1)·(D/F)·ṁ_F/M̄(z)` — every factor from the fixture, nothing read back
  off the solver. It pins the latent basis, the boilup, the mass ⇄ mole conversion
  in `D`, and the equality, in one hand calculation.
- **The condenser envelope in the general case**, reconstructed from the public
  return plus the published `k_value`: `T₁` as the dew point of the distillate by
  the test's own bisection, `T_cond` as its bubble point, `V` from the reflux
  ratio and the declared mass ratio. This is the gate with the general-case
  physics in it.
- **An envelope on the reboiler**: `V·min_c λ_c ≤ Q_reb ≤ V·max_c λ_c`. Loose, and
  it is the only thing available for the duty that closes the balance — but it
  catches the errors that actually threaten a duty (a boilup taken as `R·D`, a
  latent heat applied per kilogram, a duty counted twice).
- **The difference, as a WIRING gate**, against the enthalpy the engine's own
  edges carry. Labelled as such in its own docstring: it exercises the draw write,
  M7.4a's column arm and `cp` mixing end to end, and it proves nothing about
  whether either duty is right.

**2. The telescoping term is not what dominates the inconsistency — the interior
SENSIBLE terms are.** The argument above was first written as "the unequal molar
latent heats accumulate": `V·[λ̄(y₁) − λ̄(y_N)]`, which on this fixture is 1.2e5 W.
The measured inconsistency is 4.5e5 W, four times that. The latent spread is real
and is part of it; the larger part is the tray-to-tray sensible enthalpy CMO
drops. Recorded because the verdict survived on a reason that was only a quarter
right, and a future slice that revisits this (feed quality `q`, or an energy
balance per stage) needs the true dominant term rather than the plausible one.

**3. One coincidence, caught before it became a gate.** `Q_reb` happens to sit
within **0.2%** of `V·λ̄(y_N)` on the reference fixture — the reboiler's own
sensible term nearly cancels the CMO inconsistency. A tight assertion against
that number was written, passed, and was then removed: it is a property of these
particular numbers, not of the model, and gating it would have been fitting to a
cancellation. `a-cached-classification-expires` in miniature — a green assertion
whose reason expires with the fixture.

**4. `Δh_vap` on `ThermoModel` brought a new SHAPE of gate with it, and it is the
first identity here that can see the Trouton constant.** `TroutonThermo::k_value`
integrates Clausius–Clapeyron with `Δh_vap` held constant — that is the only
reason it has a closed form — so the model is *already committed* to a latent
heat, and the vapour pressure's own slope recovers it:

```text
d ln K / d(1/T) = −Δh_vap / R          (at fixed P)
```

`ln K` is exactly linear in `1/T` for this form, so a two-point secant is not an
approximation of the derivative; it **is** the derivative, and the check is to
machine precision. Correction 4 to the note said the exact identities are
"structurally incapable" of policing the empirical constant, and that stays true
of the three M7.2 shipped — but this one is different in kind: the quantity it
pins is `C·tb`, so it moves with `C`. It still does not catch a wrong `C` (both
sides move together — that is the envelope's job); what it catches is **the two
halves of one model disagreeing**, which is the failure that would let a cascade
solve its profile on one latent heat and its duties on another. That failure had
no gate at all before this slice, and it is a plausible one: `Δh_vap` is *linear*
in `C` where `K` buries it in an exponent, so a duty is the most `C`-sensitive
number this model produces.

**5. A duty this model cannot compute is a refusal, and the two zeros had to stop
sharing a type.** Through M7.3 both fidelities returned `Watt::ZERO`, documented
as meaning different things — the splitter's a gap, the cascade's a placeholder.
Once the cascade computes real numbers, keeping the splitter's zero would make it
the only duty a cut-point column ever reports, and a frontend sizing cooling water
off `0 W` is the finite-deterministic-plausible-wrong shape this workspace keeps
catching. So `Separation`'s duties are `Option<Watt>`: `None` from the splitter
(there is no such equipment to report on), `Some` from the cascade — including
`Some(ZERO)` for an **idle** column, because a condenser with nothing to do really
does have zero duty and that is an answer. `NodeSnapshot::column_duty` carries the
same distinction outward with `skip_serializing_if`, which is what keeps the
twelve scenarios byte-identical: they are, over 300 ticks, every snapshot.

This is `Command::SetHeatInput`'s lesson run in reverse. There, a consequence
nothing reported. Here, the risk was a report nothing computed.

**6. The saturated-liquid feed guard, and the fixture that had never satisfied
it.** M7.4a found the wired fixture feeding its column 34 K above the bubble point
of its own mix — 28% of the feed off-phase, a plant constant molar overflow does
not admit. `separate` now refuses it, **two-sided**: a subcooled feed condenses
extra reflux (`q > 1`) exactly as a superheated one flashes, and refusing only the
first would let half the violation through silently.

The bound is derived, and then measured. A liquid `ΔT` off its bubble point
carries `c̄p·ΔT` J/mol of excess enthalpy, which flashes `c̄p·ΔT/λ̄` of the feed, so
the admissible window is `ΔT_max = ε·λ̄/c̄p` — **not a constant in Kelvin**: about
±1.2 K on the M7.3 slate and wider on a heavier one, which is gated by running the
same offset against doubled latent heats and getting opposite verdicts. The
refusal quotes the bubble point, the window and the off-phase percentage, so an
author can act on it rather than guess.

**And calibrating `ε` turned up the sharpest argument for the guard existing at
all.** The first version of that measurement ran the fixture at both edges of its
window and asserted the draw compositions moved by less than `ε`. That assertion
could not fail. Trace the feed temperature through `separate` and it reaches three
places — this guard, the idle placeholder, and the feed-enthalpy term of the
duties. It does **not** reach the seed (that is the feed's *bubble point*, not its
resolved temperature), the flow profile, the stage balances or the K-values,
because there is no `q` in this formulation. The compositions are therefore
bit-identical across the whole window, and `moved < 2ε` was asserting `0 < 0.02` —
the same vacuous shape correction 7 below records for M7.4a's control, found the
same way and in the same slice.

The fix makes the gate stronger and the reasoning better. Compositions and the
condenser duty are now asserted **exactly equal**; the reboiler duty is the one
thing that moves, by exactly `−ṁ_F·cp_F·ΔT`, asserted against that closed form;
and `ε` is calibrated against the resulting shift, `ε·(F/V)·(λ̄(z)/λ̄(y₁))` ≈ `0.93ε`.
What the vacuity revealed is the real justification: **a feed off its bubble point
does not make this cascade produce a slightly wrong answer — it makes it produce
the same answer to a different question.** No amount of running the model can
surface that, which is precisely why the precondition has to be checked rather
than observed. It also means M7.3's anchors did not move when the reference
fixtures went from a fixed `T_REF` to each feed's own bubble point: Fenske, the
null case and the flash reduction are numerically untouched. What had been
off-model since M7.3 was the enthalpy bookkeeping — draw temperatures, and now
duties — never the separation.

The asymmetry against M7.3's correction 5 is deliberate and both halves are now
built: an **idle** column is a state this model can answer, so it is not refused;
a feed off its bubble point is one it cannot, so it is. Refuse what you cannot
answer; never refuse what you can.

**7. The blast radius was in the REASONING of two landed gates, not their
numbers.** Moving the fixture's feed onto its bubble point moved
`each_draw_leaves_at_its_own_tray_temperature`'s justification out from under it:
that test asserted both draws sit clear of the column's mixed feed temperature and
explained it by the feed being superheated. With a saturated feed the draws
**straddle** it — 345.98 K, feed 359.40 K, 387.89 K — which is a property of
columns rather than of a fixture, so the assertion was rewritten as the straddle
rather than retuned. M7.4a predicted this in as many words; it is recorded here
because the prediction was about the numbers and the real work was the reason.

And a fault that had been there since M7.4a surfaced with it. That slice's ambient
-transform test carried a "control" comparing the outlet against the midpoint
between the tray and the feed — but the feed was hotter than the tray, so
`outlet < tray` already implied it, and **it could not have failed for any `UA`**.
Exactly `a-conjunctive-gate-hides-which-criterion-bound`: a second assertion that
fires with the first and was never shown to bind. It is a counterfactual now —
recover the pipe's own decay from the result and apply it to the inlet a misplaced
arm would have used, which really can fail when the `UA` is large enough to swamp
the difference. The `UA` also went from 40 to 10 000 W/K: at 40 the workspace's
only live ambient transform moved its stream by **0.019 K**, which is a thin thing
for the only instance of it to be.

**What the mutation pass found.** Nine mutations, each verified to compile before
its result was believed (`a-void-mutation-looks-like-a-catch`), run with
`--no-fail-fast` so a catch in one binary could not hide the rest
(`mutation-harness-needs-no-fail-fast`). **All nine caught**, and the pattern of
*which* gate caught what is the part worth keeping:

- The **condenser envelope** caught five of the nine, and was the SOLE catcher of
  two: reading a total condenser as pure latent heat, and taking its latent heat
  at the bottoms composition instead of the distillate's. Nothing else in the
  workspace sees either. That is genuine unique coverage, stated because M7.4a's
  equivalent paragraph had to retract exactly this claim about a different gate.
- The **`α = 1` hand calculation** caught the latent basis (per kilogram instead
  of per mole) and the boilup (`R·D` instead of `(R+1)·D`) — and was the sole
  catcher of neither, since the condenser envelope sees both too. It is held for
  the same reason M7.4a holds its bubble-point identity: independence, not unique
  coverage. It is the only gate here with no solver output in it at all.
- The **two feed-guard mutations** — refusing superheat only, and a window fixed
  in Kelvin rather than `ε·λ̄/c̄p` — were each caught by exactly one gate, their
  own. Both of those gates exist because a one-sided guard and a magic constant
  are the two ways this box could have been half-built.
- Factoring the mean molar mass out of `Σ x·M·cp` was caught by the condenser
  envelope and by the feed window's own measurement — the second because a molar
  heat capacity is what converts a temperature offset into a phase error. (That
  window test was rewritten after the pass, once its composition assertion turned
  out to be vacuous; the mutation was re-run against the replacement and is still
  caught by both.)
- `Δh_vap = C·T` instead of `C·tb` was caught by both new thermo identities,
  including the Clausius–Clapeyron slope, which is the identity added *for* this
  class.

**And two of the new tests caught nothing, which is what they said they would.**
`the_duty_difference_is_the_external_sensible_balance` and
`the_reported_duties_bracket_the_enthalpy_the_plants_own_edges_carry` fired on no
mutation. Both docstrings say in advance that they are not duty gates — one is an
identity true by construction, the other a wiring check — so the empty result
confirms the labelling rather than exposing dead tests. Recorded rather than
quietly deleted: they cover the draw write, the M7.4a column arm and `cp` mixing
end to end, which no mutation in this pass touched.

#### Corrections from building it (M7.4c, landed)

The I-series reaches a column for the first time, both demos ship, and the largest
correction is the same shape as M7.4b's.

**1. The energy arm this milestone left owing cannot exist either.** The M7.4 box
warned that an energy invariant reaching a cascade column "will NOT close unless it
reads `NodeStates::column_separation` and counts that difference as a node heat
term". That is true, and it is also the whole of it: `StageCascade::duties`
*defines* `Q_reb = Q_cond + (Σ draw flux − feed flux)` in the engine's own datum,
so an invariant that adds `Q_reb − Q_cond` at the column node is adding exactly the
gap it is measuring. It closes identically, for any duties whatsoever — two duties
both wrong by the same amount included. The deterministic version already exists
(`the_reported_duties_bracket_the_enthalpy_the_plants_own_edges_carry`) and already
says in its own docstring that it proves nothing about either duty.

So M7.4c ships I7's cascade arm and NOT an I6 one, and records the verdict rather
than writing the test (`falsifiability-as-scoping-criterion`). What un-defers it is
a formulation where the reboiler duty is computed locally — an energy balance per
stage — which is the same condition correction 2 of M7.4b names for revisiting the
CMO inconsistency. Not a coincidence: one formulation choice produces both.

**The generalizable form, because this is now twice: a quantity DEFINED to close a
balance can never be gated by that balance.** Before writing an invariant, check
that its two sides are computed by independent paths. That is
`a-specified-gate-can-be-impossible` turned from a retrospective into a procedure.

**2. A cascade scenario's admissible parameter region is a property of the SOLVER
PATH, not of the physics, and it has to be swept for.** The obvious demo — the
splitter's own 0.30 / 0.50 / 0.20 yields at `R = 2` — is refused: "the draws above
the reboiler are carrying 1.026e3 mol/s away from a feed of 1.004e3 mol/s". The
refusal is right and its message already explains how mass ratios and a molar
constraint can disagree, but the part worth recording is that it is about an
ITERATE. Moles out equal moles in at every converged split, identically, because
per-component mass does; what fails is that successive substitution passes through
profiles whose distillate is lighter than the answer, and at a high enough reflux
one of those overshoots.

The consequence for anyone writing another cascade file: the working region cannot
be read off the specification. Sweeping `(D/F, S/F, R)` on this plant gives
0.30 / 0.50 failing at every `R > 1`, 0.28 / 0.50 failing at `R = 4`, and
0.246 / 0.554 — the shipped yields — solving at `R = 1, 2, 3` and failing at
`R = 4`. The demo ships at `R = 2` with that margin written into the file, because a
scenario one parameter step from a refusal is a scenario that refuses itself after
the next edit.

**3. The first tick is a different plant, and a per-tick precondition is what made
that matter.** A pipe's transport density comes from its STORED composition (§3a
fork 6, and M3.1's surviving deferral), which on tick 1 is the composition the
stream was born with rather than the one it is about to carry. On the crude demo
that is a step from 176.478 to 192.685 kg/s between ticks 1 and 2 — 9.2%, and it
never recurs.

It had never mattered before, because no gate in this workspace asserted anything
about a first tick that a 9% flow difference could break. M7.4b's saturated-liquid
guard runs every tick, so it does. The furnace's rise is `Q/(ṁ·c̄p)` and inherits
the whole step, which turns the demo's duty from a free parameter into a
constrained one: the rise must be small enough that 9.2% of it fits inside the
window while the steady state sits on the bubble point. At the shipped 5.5 K trim
tick 1 sits 0.53 K above saturation against ±1.18 K; at `crude_column.toml`'s own
30 K preheat it would sit 2.7 K above and refuse its own first tick. That is why
the cascade demo's heater is small and its source hot, which otherwise reads as an
arbitrary difference between two files meant to be the same plant.

**4. An attribution falsified before it was believed, and the control was the
thing that was wrong.** The cascade arm's I7 budget admits a second term for the
cascade's own convergence residual, and the first justification offered for it was
a comparison against the column-free tee plant. That comparison shows **1.7e-7 kg
against the cascade's 3.4e-7 kg** — within a factor of two, because Newton stops at
`1e-8 + 1e-8·throughput` kg/s and the tee plant moves ~170 kg/s. Two unrelated
mechanisms landing on the same order, so the comparison establishes nothing while
reading exactly like confirmation.

The attribution that holds is a measurement on the cascade plant itself: every node
it builds is pressure-anchored, so the hydraulic solve has no unknowns, and the
solver's reported residual measures exactly 0. That leaves the cascade's
convergence as the only candidate for the 3.4e-7. The lesson is narrower than
"measure rather than infer" — it is that **a control on a different fixture can
agree with you for a reason that has nothing to do with your claim**, and the fix
was to measure the term being excluded rather than to compare against a plant that
excludes it.

**5. The residual criterion fork 4 insisted on is a backstop, not the binding
constraint.** Removing `residual <= COMPONENT_RESIDUAL_KG_PER_S` from the
convergence conjunction changes **no test's verdict anywhere in the workspace** —
the profile and temperature criteria are strictly tighter on every plant any test
builds, and they stop the solve first. Loosening those while keeping the residual
still leaves the I7 arm green; loosening those *and* dropping the residual makes it
fire. So the criterion does bound the quantity the arm measures, and it has never
been the thing that stopped a solve. Fork 4's demand that it be a gate rather than
a counter is satisfied — M7.3's capped-iteration test reaches it deliberately — but
"reachable on purpose" and "binding in practice" are different claims and this note
had been treating them as one.

#### Deferred from M7, with what would un-defer each

- **Tray hydraulics** — pressure drop per tray, weeping, flooding. Un-defers the
  moment a stage needs a vapour *density*, which is fork 1's verdict boundary,
  not an additive correlation.
- **Partial condenser, vapour side draw, flashing feed** — fork 0's narrowed
  two-phase deferral. Un-defers with phase in the state vector.
- **Column holdup and tray dynamics** — the cascade is quasi-steady per tick, the
  same assumption §3 makes for hydraulics. Un-defers if a startup or a
  composition-front transient needs to be *watched* rather than stepped over.
- **Non-ideal K (activity coefficients)** — Raoult is adequate for hydrocarbon
  cuts, which are the only thing this slate describes. Un-defers with a slate
  carrying a polar component, where the error is distinguishable.
- **Efficiency (Murphree) per tray** — a real column's stages are not
  equilibrium stages. Additive to the cascade once a case can tell 20 real trays
  from 14 ideal ones; deferring it keeps `N` meaning one thing.
- **Feed quality `q`** (correction 5) — the cascade takes a saturated-liquid feed
  and **refuses the rest at runtime** as of M7.4b, two-sided, outside a window of
  `ε·λ̄/c̄p`. (The note said "at load", which was never possible: a column's feed
  temperature is solved, not declared, so nothing at load time knows it.)
  Un-defers when a plant preheats its feed past the bubble point on purpose, which
  is the case where `q` is distinguishable from the assumption rather than a
  second name for it — and the guard is what makes that case visible instead of
  silently mis-solved.

## 6. Time

- Engine fixed timestep, default `dt = 0.1 s` (config per scenario).
  Frontends may call `tick` faster/slower than real time.
- Slow states integrate explicitly (Euler for inventories, RK4 inside
  reactors). If a unit needs smaller steps, it substeps internally —
  the global tick never changes at runtime.

## 7. Snapshots and commands

- `Snapshot`: tick index, sim time, the component slate (name + density, so a
  frontend can interpret the `mass_fractions` that index it — M8.5), per-node
  state (levels, temperatures, unit-specific extras as tagged enums), per-edge
  stream state, solver diagnostics (iterations, residual). Serde: JSON for
  humans, bincode later if profiling demands.
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

**The other half of the same rule: a frontend must be able to DERIVE what it
draws.** `Snapshot::slate` (M8.5) closes M6.2's deferral. `heat_input_w` above
is about a command with no reported consequence; this is about a reported
consequence with no interpretation. A tank reported mass [kg], area [m²] and
height [m], which is everything except the density that turns them into
`h = m/(ρ·A)` — so the M6.2 scene drew mass on a scale shared between its two
tanks and printed kg, both faithful, neither answering "how full is it". The
component *names* were missing on the same grounds: a composition crossed as a
bare `mass_fractions` array indexing a list the frontend could not see.

**The fix is the slate, deliberately not a `level_m` field**, and the two are
not close calls. `TankState::level` already exists in `core` and a level field
would have been three lines. But the deferral names three triggers — an
absolute fill fraction, a per-component readout, a component name — and a level
serves one; the slate serves all three and several nobody has asked for yet. It
is also the right *kind* of data: the slate is an input a scenario declared, so
publishing it invents nothing, whereas each derived field added to `Snapshot` is
a quantity a frontend must then trust the engine to keep in step with the state
beside it. `ComponentSnapshot` carries `name` and `density_kg_per_m3` only —
`tb`, `molar_mass` and `cp` are inputs to models that run inside the engine, and
a frontend reading them could only recompute what the engine already reports.

**It is the first field on `Snapshot` with neither `default` nor
`skip_serializing_if`, and the asymmetry is the argument.** `controls: []` and
`column_duty: None` are true statements about a plant — it has no loops, that
node is not a column — so absence is the honest encoding and byte-identity for
the older scenarios comes free. An empty slate is not a statement: `Slate::new`
refuses one, so every engine that exists has at least one component, and a
`default` would let a pre-M8.5 document deserialize into a snapshot whose slate
claims there are none. So this field moved every scenario's bytes, which is what
made the measurement below worth doing.

**A tank's component densities are never `null`, and that is enforced upstream.**
The `Option` exists because a *slate* may carry gas cuts, whose density is
`P·M̄/(R·T)` rather than a constant. It cannot be reached down the fill-level
path: the loader refuses a tank whose composition is gas-phase, and the other
holdup kind, `Vessel`, has a pressure for a state and no level to draw. So the
scene divides by it with no fallback branch, and
`no_tank_anywhere_holds_a_component_without_a_density` is what would notice if
that guard were relaxed.

## 8. Godot integration (M6)

`godot-ext` (gdext crate) exposes a `RefinerySim` node: `load_scenario(path)`,
`tick_in_physics_process`, `get_snapshot() -> Dictionary` (or typed accessors
for hot paths), `send_command(...)`. Sync single-threaded first; move the
engine to its own thread behind a snapshot channel only if profiling shows
tick time threatening the frame budget.

**Two corrections to that sketch, made when the binding was built (M6.2).**

1. **`get_snapshot()` returns a JSON string, not a `Dictionary`.** Building a
   `Dictionary` means a recursive JSON→`Variant` converter, which is code
   written in Godot types — precisely the untestable half this section's own
   rule pushes work *out of*. The sketch would have moved the largest piece of
   translation logic in the adapter to the side `cargo test` cannot reach. It
   crosses as a `GString` and GDScript calls `JSON.parse_string()`. Typed
   accessors for hot paths remain available if profiling ever asks for them;
   nothing measured says it does.
2. **The node does not tick itself.** `tick_in_physics_process` as a property
   of the node makes pausing, single-stepping and running faster than the
   frame rate into engine concerns. The scene calls `tick()` from its own
   `_physics_process`, which is the same arrangement with the decision left
   where it belongs.

**Fallible calls return the JSON `null` on success and `{code, message}` on
failure**, so one `JSON.parse_string` serves both and a scene branches on
truthiness. Name lookups return `-1` rather than an error object: ids are
`u32` so no real id collides with it, and resolving a name is a startup step a
scene either got right or must fix in its own source.

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

### The binding (M6.2, second commit) — what building it settled

**`Session` exists so the binding contains no decisions.** A Godot node is
constructed before it is told which scenario to run, so *something* has to
answer "what happens if you tick before loading?". Answering it in the binding
would put a decision on the side `cargo test` cannot reach — the exact failure
the pure/gdext split exists to prevent, arriving through the back door. So
`bridge::Session` is a `Bridge` that may not exist yet, with every method
total: plain Rust in, plain Rust out, every failure mode carrying a code. The
binding is then one forwarding line per method, and "this file contains no
decisions" is a claim a reader can check by reading it. Its gates are the ones
that cover the binding's behaviour, because there is nothing else in it.

**A failed load leaves the running plant untouched** — the same rule a refused
`Command` follows. A typo in a scenario path must not destroy a running game,
and half-swapping a plant would be worse than either outcome.

**There is no mutation evidence for the binding, and that is the honest
report.** No `cargo test` binary can construct a `GString`, so there is
nothing to mutate. What makes the absence acceptable is the paragraph above:
the module is branch-free marshalling, so the claim being made about it is
verifiable by reading rather than by running. `cargo clippy --workspace` does
not see it either (the feature is off), so
`cargo clippy -p refinery-godot-ext --features godot --all-targets -D warnings`
is run by hand and recorded in the roadmap when the binding changes.

**The version pin is three numbers that must agree**: `api-4-7` in
`crates/godot-ext/Cargo.toml`, `compatibility_minimum = 4.7` in
`refinery.gdextension`, and `config/features` in `project.godot`. Godot 4.7 is
what the extension was built and demonstrated against; the 4.3 `project.godot`
used to declare was the version it was created under, tested against nothing.
**Consequence, stated rather than discovered: this project now requires Godot
>= 4.7.** A mismatch between the three fails silently — the library simply
does not load, with no message naming the cause.

**`cargo test --workspace` silently breaks the game, and the fix is a build
directory rather than a warning.** The crate is `cdylib` + `rlib`, so the
mandated pre-commit test run rebuilds the very `.dll` Godot loads — with the
`godot` feature off, producing a library whose entry point does not exist.
Godot then reports `GDExtension entry point 'gdext_rust_init' not found`,
which reads like a broken build and is really "your last cargo command
overwrote it". Every commit cycle would reproduce it. The featured build goes
to `--target-dir target/godot` instead, which nothing else writes to, so the
failure mode is removed rather than documented. Found by running the smoke
script after a test run, not by reasoning about it.

**The setup step that is not in any of those files.** Outside the editor,
Godot loads extensions from `.godot/extension_list.cfg`, which the *editor*
writes; it does not scan for `*.gdextension` at runtime. `.godot/` is
gitignored, so a fresh clone that builds the library and runs the project gets
`Identifier "RefinerySim" not declared` — a GDScript parse error naming
nothing relevant. Opening the project in the editor once fixes it. Written
into `refinery.gdextension`'s header, because that is the file someone reads
when the extension does not load.

**`serde_json` needed `float_roundtrip`, and nothing before now could have
found it.** The default float parser is fast, not exact: it can land 1–2 ULP
from the value that was written, so `parse(write(x)) != x`. The bridge's
round-trip gate failed on exactly two edge floats out of ~90. The existing
determinism gate never saw it because it compares serialized *strings* between
reruns and never parses one back — a class of defect that "compare the output
text" cannot reach. The feature is set at `[workspace.dependencies]`, because
cargo unifies features per build and a per-crate setting would make the
behaviour depend on which crates are in the build.

### The fill level (M8.5) — what building it settled

The scene half of M6.2's deferral. `_tank_fraction` was mass over the largest
mass seen anywhere that run; it is now `h/H` with `h = m/(ρ·A)` and `ρ` blended
from `Snapshot::slate` by the reciprocal rule. `peak_mass` and the `_rescale`
pass that maintained it are gone, and each tank now prints its level in metres
and its fill percentage beside the kg it already printed. The scene still
computes no physics in the sense that mattered in M6.2 — every operand is a
snapshot field, and the blend is the same arithmetic `core::components` applies
to the same numbers.

**The independent second side does not exist, and the reason is worth keeping.**
The natural way to gate a published density is against something the solver
derived from it, and a tank pins `P = P_ATM + ρ·g·h` — so `(P − P_ATM)/(ρ·g)`
looks like a solver-side level to check the snapshot-side one against. It is
not. Substituting `h = m/(ρ·A)` cancels the density exactly:

```text
P − P_ATM = ρ·g·h = ρ·g·m/(ρ·A) = m·g/A
```

A tank's hydrostatic pressure is mass over area and carries **no density
information at all**; a snapshot shipping `cp` in the density slot moves both
sides by the same factor and they agree. The same cancellation kills every other
candidate — mass balance, holdup, transport — because a density is observable
only through a *volume*, and the only volume any scenario declares is
`initial_level_m`. So the load-time level is the single anchor outside the code,
it exists only at tick 0, and the gate that reconstructs it there is this
slice's real one. What it pins is the WIRING — right field, right order, nothing
dropped; the mixing rule itself is `components.rs`'s and is unit-tested there.
This is the fourth time in this project a specified or reflexive gate turned out
to have one side computed from the other, and the first where the answer was to
*state* the impossibility as an assertion rather than drop it:
`a_tanks_pressure_carries_no_density_to_gate_one` is kept precisely because it
is the first thing the next person will reach for.

**A tank's reported pressure and its reported mass are one Euler step apart.**
Found by that assertion failing at 8.6e-6 relative — 0.67 Pa on
`leaking_line`'s supply tank. The tick runs solve → transport → unit dynamics
(§1), so the pressure in a snapshot was computed from the mass at the *start* of
the tick while the mass in the same snapshot is what the integration left. The
gate compares against tick 0's mass and is then exact, and asserts the fresh
comparison *fails*, so the offset is recorded rather than absorbed into a
tolerance. Nothing is wrong with either number; a frontend drawing a level reads
mass, which is the fresh one.

**What the wiring mutations measured.** Three edits at the emission site, each
verified to compile and run against the workspace: the slate emitted **sorted by
name** (3 gates fire), **`cp` in the density slot** (5), and **the first
component dropped** (7).

The order mutation is the interesting one, twice over. A five-cut slate sorted
by name moves the naphtha tank's density from 680 to 850 kg/m³ but leaves
kerosene — centre of the list, third alphabetically — exactly where it was, so a
gate run only on the distillate tank would have been vacuous under both it and a
reversal; the gate's table names which tank discriminates which mutation instead
of asserting that all three move. And the **bridge** gate, the only one that
runs on the plant the scene actually draws, does not catch it at all:
`leaking_line.toml` carries one component and sorting a one-element list is the
identity. The wired demo is blind to the most likely wiring error in the feature
it exists to demonstrate, which is a fact about the demo rather than about the
code, and is why the crude plant carries the gates that matter.

The only gate no mutation fires is `a_tanks_pressure_carries_no_density_to_gate
_one` — which is the point of it: it documents an impossibility rather than
guarding a behaviour.

## 9. Error handling & diagnostics

`SimError` (thiserror): `SolverDiverged`, `NonFiniteState{location}`,
`InvalidCommand`, `ScenarioError`. Engine keeps a ring buffer of recent solver
diagnostics included in snapshots — frontends can show "solver stress" and
tests can assert convergence quality, not just results.

## 10. Regulation — control loops (M8) — specified before building

### The premise, and the deferral that named it

Nothing in this simulator regulates anything. Every `Command` variant is a
direct manual write of a number a human chose: a valve opening, a pump's
on/off, a furnace duty. A tank fills until the mass clamp catches it; a level
that drifts drifts forever unless something outside the engine notices and
sends another command. Thirteen scenarios, and not one setpoint among them.

§3a fork 5 is where this was last refused, and it drew the boundary precisely:
a relief valve is "a pure element characteristic … no state, no tuning
constants, no tick history", and **"state is what turns an element into a
controller."** M8 crosses that line deliberately, and this note is mostly about
what the state costs.

### Fork 0 — what "regulation" is scoped to in the first building slice

One loop type: **a tank's level, actuating a valve on its outlet.**

Both halves already exist and are already load-bearing. `TankState::level(ρ)`
is what `bottom_pressure` reads, so the measurement is not new; `Valve::opening`
is a settable, range-validated field, so the actuator is not new either. The
slice therefore adds **no physics and no solver machinery** — its entire content
is the loop, its state, its command surface, and its gates. That is deliberate:
a slice introducing a new measurement *and* a new actuator alongside the control
machinery could not tell a controller bug from a measurement bug.

### Fork 1 — where a loop lives on the model

- **(a) A new `NodeKind`. Rejected.** A controller conducts nothing.
  `validate_degrees`, `anchored_set`, `compile_edges` and `NodeSnapshot` all
  assume a node is a hydraulic object; a node with no incident edges is a
  floating dead leg to §3c and a NaN pressure to every snapshot reader. The
  graph would carry a thing whose only property is that every graph algorithm
  must skip it.
- **(b) A field on the actuator node. Rejected.** A loop names a measurement
  and an actuator *independently*, and in the only case this slice builds they
  are different nodes — hanging the loop off either end makes the other end the
  arbitrary one. Worse, the actuator field is precisely what the loop WRITES;
  storing the writer inside the written struct makes "who owns this opening"
  unanswerable at the exact point `apply` has to answer it (fork 4).
- **(c) Chosen: an ordered list beside the graph**, `PlantGraph::controls:
  Vec<ControlLoop>`. Each entry names its measurement node, its measured
  variable, its actuator node, its algorithm and tuning, its mode, its setpoint
  and its state. Declaration order is execution order; a `Vec`, never a map
  (rule 3). A `LoopId` indexes it the way `NodeId` indexes the nodes, because
  the command surface has to name one from outside.

### Fork 2 — is the algorithm a trait? Rule 2 does not settle this by itself

The reflex is: proportional versus integral is a fidelity, fidelity is
trait-impl selection (rule 2), therefore a `Controller` trait. **That reflex
skips the step that matters here**, so it gets argued rather than inherited.

Every seam this project has built — `FlowSolver`, `ThermoModel`,
`ReactionModel`, `SeparationModel` — is an engine-wide **singleton**: one box,
chosen once, by one string in `[fidelity]`. A control algorithm is not that
shape. One plant can reasonably want a proportional loop on one tank and an
integral loop on another, and `[fidelity]` has no way to say so — its keys are
per-engine by construction.

Two candidate shapes, differing in arity rather than in principle:

- **(a) An enum on the loop, matched inside the update. Rejected.** That match
  is `if fidelity == Simple` wearing a different hat, sitting inside the one
  function every loop calls. It is the shape rule 2 names as a bug.
- **(b) Chosen: a trait, boxed per loop.** `Controller` in `core::traits`,
  impls in `solvers`, selected per `[[controls]]` entry rather than in
  `[fidelity]`. This is the project's **first `Vec<Box<dyn _>>` seam** and the
  first fidelity choice made per instance instead of per engine. Rule 2 is
  honored, not bent: what changes is how many there are.

**One asymmetry has to be said out loud, because it is genuinely new.** Every
existing seam's impls are pure functions of their arguments. A `Controller`
impl **owns state** — the integral term — which makes the box part of the
engine's inventory rather than part of its configuration. That is exactly the
property §3a fork 5 named as the thing that turns an element into a controller,
and it is why fork 5's reasoning does not carry over: a PSV could stay an
element because it had none.

### Fork 3 — when in the tick, and what the loop is allowed to see

A loop reading *this* tick's solved state and writing an actuator opening would
change a solver input after the solve, requiring a re-solve, whose answer would
change the input again. That is an algebraic loop, and it is the same shape §3c
rejected for per-iteration reclassification: an input that moves mid-solve makes
the solver's own comparisons quantities over different problems. **Rejected.**

**Chosen: the loop runs at the top of the tick, before the hydraulic solve, on
the state standing at the start of it.** One `dt` of measurement lag, and two
independent justifications, neither of them convenience:

- It is what a real plant does. A DCS samples on a scan and acts on the previous
  sample; sampled control with one scan of lag **is** the physical system, not
  an approximation of it.
- It is the staleness §3 already accepts everywhere else: the quasi-steady solve
  is already driven by tank levels integrated at the end of the previous tick,
  which `network.rs` says of its transport density and `engine.rs` of its draws.

**Where the measurement is read differs by variable, and this note originally
got it wrong for the one variable the first slice builds.** The two kinds are
already distinguished elsewhere in this file, in `heat_input_w`'s words: a
*stored* quantity, like a tank's temperature, versus a *solved* one.

- A **level is stored.** `TankState.mass` lives on the graph and is real from
  load — the loader computes it from `initial_level_m` — so a level loop reads
  the graph and has a genuine measurement at tick 0. It does **not** read
  `NodeStates`, which carries temperature, composition, reactor duties and
  column separations and no inventory at all.
- A **pressure or a temperature is solved**, and lives in `last_solution` /
  `NodeStates`, both of which are empty before the first tick. A loop on either
  has no measurement at tick 0 and needs a stated rule for that tick when those
  variables un-defer — which is one more reason they are deferred separately
  rather than "for free once the seam exists".

So the earlier claim that "tick 0 has no previous state, therefore the initial
output must be declared" is **false for a level loop** and is not the reason
`initial_output` exists. The reason is fork 5's: the loop's memory is an initial
condition, and one declared number is what keeps it from being a silent zero.
The tick-0 measurement question is real, but it belongs to the deferred
variables, not to this one.

### Fork 4 — the command surface, and what "manual" now means

Today `Command::SetValveOpening` writes `opening` and nothing contests it. With
a loop in AUTO on that valve, the write survives until the top of the next tick
and is then silently overwritten — **a command that appears to work and does
not**, which is the failure `apply` already refuses by name for the relief
valve. It gets the same treatment and its own reason string.

- `SetControllerMode { loop_id, mode }` — `Auto | Manual`.
- `SetSetpoint { loop_id, value }` — range- and finiteness-checked like every
  other command argument.
- In `Manual`, the existing `SetValveOpening` drives the actuator, unchanged.
  In `Auto` it is refused.

**Transfer between modes is bumpless in both directions, and that is a decision
rather than a nicety.** AUTO→MANUAL is free — the actuator already holds the
loop's last output. MANUAL→AUTO is not: an integral term that kept accumulating
(or that sat at zero) makes the output jump the instant the loop takes over. The
fix is to back-calculate the integral from the actuator's current position at
the moment of transfer, which is **the same arithmetic anti-windup needs**
(fork 6, gate 4). Building it once, in the slice that ships the integral, is
cheaper than deferring it — which is why this note does not defer it.

**§7's rule lands directly on this slice, and is why the snapshot surface is
specified here rather than discovered later:** *every command must have a
reported consequence.* A `SetSetpoint` writing a field no snapshot reports is
M6.0's `PuncturePipe` with the direction reversed — the defect §7 already
documents. So:

```text
Snapshot        { …, controls: Vec<ControlSnapshot> }   // skipped when empty
ControlSnapshot { id, name, mode, setpoint, measurement, output }
```

`measurement` is the value **the controller acted on**, not a re-read of the
current state. Those differ by one tick (fork 3), and reporting the fresh one
would make a lagging loop look instantaneous — hiding the lag from precisely the
person debugging it.

**`setpoint` and `measurement` cannot be bare `f64`s, and the field list above
would have made them so.** Every quantity on `NodeSnapshot` carries its unit in
its own name — `pressure_pa`, `temperature_k`, `heat_input_w`, `condenser_w` —
because a snapshot is plain serde data with no newtypes to carry it. A loop's
setpoint has no such name available: it is metres today and Pascals the moment
pressure control un-defers, so `setpoint_m` would be a lie on half the loops and
a bare `setpoint` would be a number whose unit depends on a *sibling field*,
which is the failure rule 4 exists to prevent at a crate boundary. §7 already
supplies the answer in its own words — "unit-specific extras as tagged enums":

```text
ControlledValue = Level { m: f64 } | Pressure { pa: f64 } | …   // #[serde(tag = "variable")]
ControlSnapshot { id, name, mode, setpoint: ControlledValue, measurement: ControlledValue, output }
```

One consequence is worth taking deliberately rather than discovering: the
setpoint and the measurement are then the **same type**, so a loop cannot report
a setpoint in one variable against a measurement in another. That is
`column_draw_at`'s rule from M7.4 — one owner for two fields that must agree —
applied to the pair a reader is most likely to subtract.

Inside `core` the same quantity is a unit newtype (`Meter`, `Pascal`) carried by
the same enum, so rule 4 holds on the way in as well as on the way out. And at
the TOML boundary the key carries the unit the way every other key does
(`area_m2`, `initial_level_m`, `temperature_c`): `setpoint_m` for a level loop,
and the loader refuses a key that does not match the loop's `variable` — the
same two-directional refusal the two separation fidelities already enforce, so
no second notion of "which unit is this" can exist to disagree.

`output` stays a bare fraction: an actuator position is dimensionless in
`[0, 1]` and is already validated as such by `SetValveOpening`. It gains a unit
question only when an actuator that is not a valve un-defers.

Two traps, both already paid for elsewhere in this file:

- The list is `skip_serializing_if = "Vec::is_empty"`, so the thirteen existing
  scenarios stay byte-identical — the move `ColumnDraw` and `column_duty` both
  made.
- **No controller field on `NodeSnapshot`.** A per-node `Option` reporting "no
  loop here" on every node of every plant is the inverse of `column_duty`'s
  argument: absent where there is nothing to report, and the report lives with
  the loop that owns it.

### Fork 5 — controller state is an initial condition, not an implementation detail

A PI loop's integral term is stored state in exactly the sense a tank's mass is:
carried across ticks, and the next answer depends on it. Rule 3 says same
scenario ⇒ bit-identical, and M8.0 has just finished paying for the belief that
a carried-over number is "a path, not an answer" — `prepare`'s continuation seed
turned out to decide where a pass terminates.

So the scenario declares the loop's memory, and declares **one** number rather
than two:

```toml
[[controls]]
name            = "level_control"
measurement     = { node = "supply_tank", variable = "level" }
actuator        = "discharge_valve"
algorithm       = "pi"        # "p" | "pi"
setpoint_m      = 6.0         # the unit is in the KEY, and must match `variable`
mode            = "auto"      # "auto" | "manual"
initial_output  = 0.5         # actuator position at t = 0
gain            = 0.4
integral_time_s = 120.0       # PI only; refused on "p"
```

`initial_output` is the declared quantity; the integral term is **derived** from
it at load, by the same back-calculation MANUAL→AUTO uses. So there is exactly
one way a loop's memory can be initialized and no silent zero anywhere. `gain`
and `integral_time_s` have no defaults, for the reason `x_T` has none (§3a fork
6): a silent default is an invented value in disguise, and every gate would then
pass for whatever was chosen.

Refused at load, in both directions — the pattern the two separation fidelities
established:

- `integral_time_s` present with `algorithm = "p"`, or absent with `"pi"`.
- Two loops naming the same actuator. Split-range and override control are real
  and are deferred below; until then two writers of one opening is an ambiguity
  with no defined resolution order, which is worse than a refusal.
- `variable = "level"` on a node that is not a `Tank` — a level names nothing on
  a vessel whose state IS pressure (§3a fork 2 says so in those words).
- An `actuator` that is not a `Valve`, and a `ReliefValve` with its own reason:
  its opening is not settable by anything (§3a fork 5).

### Fork 6 — the gates, named before building, and the vacuity each one closes

**"The level sat at the setpoint" is not a gate.** A tank draining through a
fixed valve self-regulates: `bottom_pressure` rises with level, so outflow rises
with level, and the thing finds an equilibrium with no controller anywhere in
sight. A single steady-state assertion passes on the plant with the loop
*removed*, which is `a-control-can-be-implied-by-its-assertion` one milestone
later.

Four gates, each named with the mutation it is predicted to be the one to catch:

1. **The loop-off counterfactual.** Same plant, same disturbance, loop parked in
   `manual`. The level must leave the band the AUTO run holds. Written first,
   and it is the control rather than the test.
2. **Setpoint tracking.** Step the setpoint mid-run; the level must move to the
   new value. This proves the output is a function of the setpoint, which the
   steady-state assertion cannot — a self-regulating tank's equilibrium is a
   function of the valve position alone.
3. **Disturbance rejection.** `Command::PuncturePipe` is a step increase in
   outflow and needs no new *engine* machinery — but it is not free at the
   scenario level: `apply` refuses a puncture on a pipe whose file declares no
   `leak_to`, deliberately (§3b fork C), so the gate's plant has to declare the
   leak path at load. The loop must return the level toward setpoint; the P loop
   must do so with a **measurable offset** and the PI loop without one. That
   pair is what proves the integral term does the thing its name claims —
   neither half proves it alone.
4. **Saturation and windup**, on a plant built to saturate: inflow exceeding the
   outlet's flow at *full* opening, so level rises while the valve is pinned and
   the integral accumulates against an actuator that cannot answer. Cut the
   inflow and an unclamped integral holds the valve open well past the setpoint
   — the undershoot is the signature. **The plant has to be built for this**,
   stated here because an anti-windup branch nothing reaches is the vacuous
   counter this file has shipped twice (`a-counter-is-not-a-gate`).

Plus the regression anchor, in M8.0's shape and for M8.0's reason: **all thirteen
existing scenarios byte-identical on both fidelities**, because none declares a
loop. Anything that moves is the seam leaking into plants that never asked for
it.

### Corrections from building it (M8.2, landed)

The seam, `ProportionalController`, the command surface and the `[[controls]]`
table landed as fork 1 through fork 4 specify. Five things the note got wrong or
left unsaid, and one finding that belongs to a different subsystem.

**Correction 1 — `gain` cannot be a bare key, for `setpoint_m`'s own reason.**
Fork 5's TOML sample writes `gain = 0.4`, and fork 4 spends a whole correction
establishing that a loop's setpoint key must carry its unit because the quantity
is "metres today and Pascals the moment pressure control un-defers". A gain is
`1/m` on a level loop and `1/Pa` on a pressure loop, so a bare `gain` is a number
whose unit depends on a **sibling key** — which is the exact failure that
correction names, applied to the other half of the same entry. The key is
`gain_per_m`, and it is required-per-variable exactly as `setpoint_m` is.

**Correction 2 — `initial_output` is not in this slice, and putting it here would
have cost the next slice its gate.** The reflex while building was that a P
controller needs a bias (`u = u_b + K·e`), because without one a level loop shuts
its valve completely at setpoint. That reflex is wrong twice over. It is wrong
about the quantity: fork 5 defines `initial_output` as *the loop's memory*, from
which the integral is derived, and a proportional controller has no memory for it
to be the initial condition **of**. And it is wrong about the consequence: gate 3
reads the P loop's steady-state offset as the discriminating half of a pair, and
with a manual-reset bias that offset becomes a function of how well the bias was
chosen rather than of the missing integral. So `ProportionalController` is
`u = clamp(K·e, 0, 1)` with nothing else in it, the offset is large and honest
(**+0.74 m** at a 4 m setpoint on the gate plant), and the key is not in the
`[[controls]]` struct at all — `deny_unknown_fields` refuses a file that writes
it, and M8.3 adds it once, with one meaning. The alternative — the key landing
here as "bias" and changing meaning in M8.3 to "integral seed" — is worse than
either.

**Correction 3 — `MeasuredVariable` is derived from the setpoint, not stored
beside it.** The note names both types and the reflex is a field for each. Two
fields with one invariant is the shape M7.4's `column_draw_at` rule exists to
prevent, so `ControlLoop` stores `setpoint: ControlledValue` alone and
`ControlledValue::variable()` answers "what does this loop measure". The loader
still parses a `variable` key, because a file must say which unit its setpoint key
carries before the setpoint can be built — but it is consumed there rather than
stored twice.

**Correction 4 — two of fork 4's and fork 5's refusals have no reachable path
today, and are recorded rather than shipped as guards.** Both are the "setpoint
key disagrees with `variable`" refusal, in its two directions. With one
`ControlledValue` variant a mismatched setpoint is *unrepresentable* — `SetSetpoint`
can carry nothing else — and a mismatched TOML key is refused by
`deny_unknown_fields` as unknown rather than as mismatched. A refusal path nothing
can reach is a coverage claim that cannot be checked
([[a-counter-is-not-a-gate]]), so neither is written. Both become required the day
a second variable lands, and the types carry comments that say so
([[a-comment-that-names-its-own-expiry]]). The same applies to the mirror of
fork 5's tuning refusal: `integral_time_s` *present* on `"p"` is refused with its
own message, while `integral_time_s` *absent* on `"pi"` is not, because `"pi"` is
not a selectable algorithm until M8.3 and the unknown-algorithm arm is what a file
writing it already hits.

**Correction 5 — `PlantGraph` loses `Clone`, and a tank's level gains an owner.**
The derive had no call site in the workspace and could not survive a
`Box<dyn Controller>`: cloning a graph would fork a loop's memory into two engines
that then diverge, which is the opposite of what rule 3 wants from a copied plant.
`Debug` survives, which is why `Controller` carries it as a supertrait. Separately,
`network::fixed_pressure` used to compute a tank's density inline — harmless while
the solver's head was the only consumer of a level, and not harmless once a
controller *acts* on the same level. `TankState::density`/`level`/`bottom_pressure`
now take the slate, so there is one definition of where the liquid surface is.

### The finding this slice reached and did not fix

> **Superseded by M9.0 (DESIGN §11), and its stated mechanism is false.** The
> stall is real and was fixed by one constant; the explanation below — an
> unbounded `dQ/dΔP` and a line search cutting the step back — describes neither
> the conductances nor the code that ran. What actually happens is that Newton on
> the regularised square-root law overshoots to the *mirror* of the branch drop,
> which `ARMIJO_C = 1e-4` was too small to reject. Kept as written because the
> corrections below are only readable against it.

**A branch driven to zero flow in ONE tick stalls the hydraulic solver, and a hand
command does it exactly as a controller does.** M8.2 makes something newly
reachable — before it, a valve opening moved only when a person sent a command,
and now a loop can slam one shut between two ticks. A setpoint step large enough
to clamp `K·e` to zero produces `SolverDiverged` on the very next tick.

The control is what settles where this belongs: the identical endpoint, written by
`Command::SetValveOpening` on the same plant with the loop parked in MANUAL, fails
the same way. So the seam **reached** a defect rather than introducing one, and
fixing it inside the control loop — a rate limit — would hide it rather than mend
it. Measured, on the gate plant after 8 000 ticks:

- The residual falls **monotonically**, by about 0.36% per iteration:
  `4.158 → 3.344` over the 50-iteration cap. Newton is crawling, not oscillating
  and not stuck. `ΔP = α·Q|Q|` has an unbounded `dQ/dΔP` as `Q → 0`, so the step
  from a warm start carrying 3.3 kg/s is enormous and the line search cuts it back
  to nearly nothing.
- The **same endpoint reached gradually converges** (20 ticks of `0.20 → 0.00`),
  and so does a cold start already at the shut state. It is the jump that fails,
  not the state.
- `gain_per_m = 0.05`, which never fully shuts the valve, survives the same step;
  so does the `simple` flow solver; so does any target opening at or above 0.01.

Pinned by `a_branch_shut_in_one_tick_stalls_the_solver_whoever_shuts_it`, which is
written to FAIL when the solver is fixed and carries the assertion it should then
make. It belongs to a `newton_flow` slice — warm-start handling or step damping —
not to M8.

### Corrections from building it (M8.3, landed)

`PiController`, the anti-windup clamp, MANUAL→AUTO transfer and `initial_output`
landed as fork 4 and fork 5 specify. Six things the note got wrong or left unsaid,
and one of them is a gate.

**Correction 1 — "the same arithmetic" is true only for one choice of state, and
the note does not name it.** Fork 4 asserts that the anti-windup clamp and the
MANUAL→AUTO back-calculation are the same arithmetic and are therefore built once.
That is not a property of PI control; it is a property of how the integral term is
*stored*. With the textbook state — `∫e dt`, multiplied by `K/T_i` where it is
used — the clamp and the transfer are two different formulas over two different
quantities, and "built once" would have been a claim the code could not keep.

So the memory is held **in output units**: `u = clamp(K·e + b, 0, 1)`, and `b` is
the share of the actuator position the integral term is responsible for. Inverting
that for "what memory makes the next output be `u`" is `b = u − K·e`, one line,
and all three writers of a loop's memory go through it — the clamp, the transfer,
and the load-time seed from `initial_output`. The representation was chosen *by*
fork 4's claim rather than the claim being checked against a representation chosen
for other reasons.

**Correction 2 — the integral is evaluated on errors already accumulated, and the
order is load-bearing rather than stylistic.** `update` computes its output from
the `b` standing at the top of the tick and adds this tick's error afterwards
(explicit Euler, the engine's own rule). The alternative — accumulate, then
compute — differs by one integration step, which is invisible in every steady
state and is exactly what makes the transfer inexact: the first `update` after a
seed would return `u + (K/T_i)·e·dt` instead of `u`. The bumpless-transfer gate
asserts a few ULP, and it can only do so because of this ordering.

**Correction 3 — the transfer reads the measurement FRESH, and the reflex is to
reuse `last_measurement`.** Fork 3 establishes that a loop acts on one-tick-old
state, and `ControlLoop::last_measurement` is that state, stored and reported. The
reflex while building was that a transfer should therefore seed against it. It
should not. Commands are applied *between* ticks, so the state standing when
`SetControllerMode` runs is precisely the state the next control pass will
measure: seeding against a fresh read makes the next output equal the actuator's
current position identically (measured deviation: exactly zero), while seeding
against `last_measurement` computes the seed against a different error than it is
spent against, and the transfer is bumpless only to the extent the plant had
stopped moving. The staleness fork 3 wants is in what the controller *acts on*,
not in what a transfer is *calibrated against*.

**Correction 4 — `Controller::seed_from_output` has no default body, deliberately.**
A stateless impl needs an empty one, and the obvious economy is a default that
does nothing. That default would mean an impl with memory inherits silence by
forgetting — the one failure the method exists to prevent. `ProportionalController`
writes its no-op out, with the reason it is a no-op.

**Correction 5 — `initial_output` is accepted on a loop declared `mode =
"manual"`, and the refusal list could be read as forbidding it.** The number does
seed real state at load, the loop's first `set_controller_mode` re-seeds it from
the actuator anyway, and refusing it would mean a plant that starts in MANUAL and
goes to AUTO on tick 1 must omit a key it is then required to have. It is
accepted; the reasoning is recorded because someone reading fork 5's refusals will
ask.

**Correction 6 — gate 3 as the note specifies it does not discriminate, and this
is the correction that matters.** Fork 6 says: "the P loop must do so with a
measurable offset and the PI loop without one. That pair is what proves the
integral term does the thing its name claims." The first half is fine. The second
half is not a discriminating test: a proportional loop's steady-state offset is
`e = u/K`, so a large enough gain drives the offset toward zero and passes a
"returned to setpoint" assertion with no integral term anywhere in the code. The
gate would have measured the gain.

What a proportional loop **cannot** do is move its output while holding its level,
because `u = K·e` makes those the same statement. So the gate asserts the
identity rather than the outcome: the P half's level move must equal its own valve
travel divided by the gain (measured 0.477229 m against 0.477230 m, agreeing to
1.2e-6 m), and the PI half must move its valve at least as far (0.260950) while
its level does not move (0.001340 m, against the 0.521900 m that travel would
force on a proportional loop). This is `a-control-can-be-implied-by-its-assertion`
answered by making the assertion an identity of the algorithm, and it is the third
time in two milestones that a gate specified in advance had to be rebuilt because
one side of it was not independent of the other.

**And one measured constraint the note could not have known.** Gate 4 removes the
load by cutting the inflow, and cutting it to zero drives a branch to zero flow in
one tick — the M8.2 stall, unrelated to control, which would have failed the gate
for a reason that has nothing to do with windup. The cut is to a quarter of the
feed valve's opening, which removes far more load than the gate needs and leaves
the solver a problem it can solve.

**The mutation both new gates were checked against**, since a docstring that
claims a gate catches something is a claim rather than a hope. `if (0.0..=1.0)
.contains(&unclamped)` → `if true`, i.e. accumulate while the actuator is pinned:
checked to compile, run, and restored from a single pre-mutation snapshot. The
loop then holds the drain wide open a metre below setpoint while it spends the
surplus it accumulated — deepest level **2.7379 m against 3.7042 m** with the
clamp, ending at **4.6213 m against 3.9304 m** on the far side of the overshoot
that pays for it. Both of gate 4's assertions fire.

**Its prediction was right, and that is worth saying because the last one was
not.** The table above predicts "gate 4 alone" for this edit, and gate 4 alone is
what fired. M8.2's early edit — "gain applied to the measurement instead of the
error" — was predicted to fail gates 2 and 3 and in fact failed three, including
one the table does not mention. Two of the seven named edits are now run: this
one, predicted correctly, and that one, predicted incompletely.

**A second edit was run that the table does NOT name**, and it is counted
separately for that reason. The transfer gate's failure message names a cause, a
named cause is a claim, so the cause was applied: seeding the transfer from
`last_measurement` instead of a fresh read. It is caught, by that gate alone — and
its number is the argument for the derived tolerance rather than a comfortable
one. The stale seed steps the valve by **4.66e-5**, which any bound chosen to
"look tight" would have passed. Only a bound derived from the two roundings in
`b = u − K·e` followed by `K·e + b` is small enough to see it.

So the tally M8.4 inherits is **five of the seven named edits remaining**, plus
one unnamed edit already run. The table itself needs one revision there: it
predicts that dropping the back-calculation on MANUAL→AUTO is caught by "gate 4,
through the same clamp arithmetic", and that prediction was written before the
transfer gate existed. It should now be caught by the transfer gate first, which
makes it a prediction M8.4 can falsify rather than a stale one to quietly fix.


**One consequence for this note's own deferral list.** "Actuator dynamics (stroke
time, rate limits) … un-defer when a loop's measured performance depends on them,
which at `dt = 0.1 s` against a 120 s integral time it does not" is right about
performance and incomplete about *reachability*: a loop's ability to run at all can
depend on the actuator's rate, through the solver rather than through the control.
The deferral stands — the fault is the solver's, and the manual write proves it —
but its stated reason is now known not to be the whole test.

### What the regression anchor measured

All thirteen shipped scenarios, both flow fidelities, 300 ticks: **26 runs, every
one exiting zero, every one byte-identical** to the same run on the tree before
this slice. (A *pre-M8.5* measurement: `Snapshot::slate` later added one key to
every snapshot in the workspace, so reproducing this number requires a pre-M8.5
tree — §7 and ROADMAP M8.5 carry that measurement.) The mechanism is `skip_serializing_if = "Vec::is_empty"` plus the
empty-list early exit at the top of `run_control_loops`, so a plant that declares
no loop does not execute one line of the seam. M8.0's shape and M8.0's reason.

### The mutations this slice owes, named before building

Predictions, which is what makes them falsifiable — M8.0 got three of four wrong.
**Left verbatim: the outcomes are in "The mutation pass, against the
predictions" below, and editing a prediction after running it destroys the only
thing it was for.** (This one got four of seven wrong.)

| the edit | predicted catch |
|---|---|
| the integral never accumulates (PI degraded to P) | gate 3's offset pair, and nothing else |
| the anti-windup clamp removed | gate 4 alone |
| the loop runs AFTER the solve instead of before | **predicted uncaught** — a one-tick shift on a slow loop |
| gain applied to the measurement instead of the error | gates 2 and 3; deliberately subtler than a sign flip, which would fail everything for the wrong reason (M1's lesson) |
| `initial_output` ignored, integral seeded at zero | **predicted uncaught by any steady-state gate** — it lives in the transient |
| the AUTO refusal of `SetValveOpening` removed | the refusal gate alone |
| back-calculation dropped on MANUAL→AUTO | gate 4, through the same clamp arithmetic |

Two of the seven are predicted uncaught. If either is caught, this note was
wrong about what its gates measure; if either survives, it names a gap the slice
must fill or record.

### The demo, and what it can and cannot slam (M8.4, landed)

`scenarios/tank_level_control.toml` is the first file in `scenarios/` to declare
a `[[controls]]` table: M1's reference plant with its receiving tank held at
4.0 m by a PI loop on a drain valve, meant to be diffed against
`tank_pump_valve.toml`. Four things it settled that the roadmap box could not
have.

**A level loop has to actuate a DRAIN, and that is forced by the sign
convention rather than chosen.** `ControlledValue::error` is
`measurement − setpoint` and an output is `clamp(K·e + b, 0, 1)`, so a rising
level OPENS the actuator. On a fill valve that is runaway; only on a drain is it
regulation. So the demo keeps M1's `discharge_valve` fixed at 0.5 and adds a
controlled drain — which is why the diff against `tank_pump_valve.toml` is two
extra nodes rather than a table and nothing else. This is the first place the
convention chosen in fork 1 constrains a *plant*, and it will constrain every
level loop written after it.

**The gain bound is the plant's, and both sides of it are run.** At the demo's
settled operating point the drain sits at ~0.376, so a setpoint step of +1 m
subtracts `gain_per_m × 1` from the output in one tick: `0.25` and `0.35` absorb
it, and `0.4` reaches exactly `0.0`, shuts the branch in one tick and diverges
the Newton solver (residual ~1.1e1) — M8.2's finding, the solver's defect and
not the loop's. The file ships `0.25`. M8.2's gate plant read `0.05` survives
and `0.1` does not; that number belongs to that plant and does not transfer, and
the roadmap box's instruction to read the bound here is why both were measured
rather than one inherited.

**A PI demo cannot slam its actuator at startup at any gain, which is the
reverse of the worry the box was written with.** The memory is seeded by
`b = u − K·e` against the error standing at load, so the first `update` returns
the declared `initial_output` whatever `K` is — gains from 0.25 to 20 were run on
the shipped file and every one survives. The startup step the box feared exists
only if the file declares `level_valve.opening` and `initial_output` *apart*,
and the demo declares them equal for exactly that reason. The bound above is
reachable only through a setpoint move, which is why it takes a command to
measure and cannot be read off a CLI run.

**`dt = 1.0 s` is the only such value in `scenarios/` and is not an accuracy
shortcut.** A level loop on this tank settles in ~2 500 s; at the repo's usual
`0.1 s` the demo would be 60 000 ticks. The same run at `dt = 0.5` over twice
the ticks ends at 3.991993 m / 0.366078 against 3.991994 m / 0.366080 — the
answer is the step size's to 1e-6. **The endpoint is the weaker half of that
check**, because a settled state is where the derivatives are smallest and the
step size matters least; the informative comparison is the overshoot peak, which
is 4.079400 m against 4.079582 m — agreement to 1.8e-4 m at the one place in the
run where the level is actually moving fast.

The demo's own trajectory carries the discrimination gate 3 had to be rebuilt to
get: over its last 2 000 ticks the load is still falling (the supply tank is
draining), the loop tracks it by CLOSING the drain by 6.778e-3 — and the level
*rises* by 1.584e-3. `Δlevel = Δu / K` would force a FALL of 2.711e-2. The wrong
sign, not merely a smaller number than the identity predicts.

### The mutation pass, against the predictions (M8.4, landed)

All seven named edits have now been run, each verified to compile and each
applied to a source restored from one pre-mutation snapshot, with the whole
workspace run `--no-fail-fast` so a catch set cannot be truncated at the first
failing binary. **Four of the seven predictions were wrong**, which is close to
M8.0's three-of-four and is the reason the table is written before building
rather than after.

| the edit | predicted | what actually fired |
|---|---|---|
| gain applied to the measurement (run in M8.2) | gates 2 and 3 | three gates, one the table does not mention — **incomplete** |
| the anti-windup clamp removed (run in M8.3) | gate 4 alone | gate 4 alone — **right** |
| the integral never accumulates | gate 3's offset pair, **and nothing else** | four: gates 3 and 4, and both of the demo's — **wrong about "nothing else"** |
| the loop runs AFTER the solve | **uncaught** | one: `a_branch_shut_in_one_tick_stalls_the_solver_whoever_shuts_it` — **falsified** (and see M9.0: the catch SURVIVED that test being turned right way up) |
| `initial_output` ignored, integral seeded at zero | **uncaught by any steady-state gate** | two: the demo's startup gate and fork 5's refusal sweep — **falsified** |
| the AUTO refusal of `SetValveOpening` removed | the refusal gate alone | the refusal gate alone — **right** |
| back-calculation dropped on MANUAL→AUTO | gate 4, through the clamp arithmetic | the transfer gate alone; gate 4 silent — **the table wrong, M8.3's revision right** |

Three of those need their mechanism stated, because the bare word "caught" would
be misleading in each.

**The loop running after the solve is caught by a test that pins a DEFECT, not
by a gate of the seam.** `a_branch_shut_in_one_tick_stalls_the_solver_whoever_
shuts_it` steps a setpoint and asserts that the very next `tick` returns `Err`.
With the control pass moved below the solve, that tick solves with the valve
still where it was, converges, and the assertion written to fail when the solver
is FIXED fails instead — its message reads "SOLVER FIXED: a controller can now
shut a branch in one tick". So the prediction was right about the mechanism (a
one-tick shift, invisible in every settled number) and wrong about the
consequence, and it was a test kept deliberately upside-down that saw it. The
gap the prediction named is real and is still open: **no gate asserts the tick
ORDER**, and the one that noticed is a test whose whole purpose is to be deleted
when the solver is fixed. That is recorded rather than filled, because a gate
for it would have to assert something about a one-tick shift on a plant slow
enough that nothing else can see it.

> **M9.0 closed this, and not by writing the gate this paragraph asks for.** The
> solver was fixed and the pin was turned the right way up rather than deleted;
> it now asserts the shut branch's ENDPOINT, and the reordering leaves `4.16 kg/s`
> running through a branch it says is shut. A state assertion catches what an
> expected failure caught, and keeps catching it across the next solver change.
> The lesson stands the other way round from how it was written: an upside-down
> test can be carrying a real gate, so re-run its catches after righting it
> (DESIGN §11).

**`initial_output` ignored is caught twice, and only one of the two is about
what the edit is named for.** The demo's startup gate sees it directly — the
first output comes back `0` instead of `0.2`, a step of −0.2 against a derived
bound of `f64::EPSILON / 2`. The other catch is fork 5's refusal sweep, and it
fires because the range check on `initial_output` lives *inside*
`seed_from_output`: an edit that stops calling the seed also stops validating
the key, so `initial_output = 1.4` loads. That is a faithful consequence of
ignoring the key, but a narrower edit — keep the validation, drop only the seed
— would leave the demo's gate alone, and the demo's gate did not exist when the
prediction was written. **The note's prediction was correct about the gates it
had**; it is M8.4's own transient gate that falsifies it, which is the slice
filling the gap the note asked it to fill or record.

**Dropping the back-calculation on MANUAL→AUTO steps the valve by 3.83e-1** and
is caught by the transfer gate alone. Gate 4 — the table's prediction, written
before the transfer gate existed — stays green. M8.3 flagged that prediction as
stale and asked for it to be falsified rather than quietly fixed; it now has
been.

### What the demo does NOT cover, measured

M8.0's precedent is a slice whose own fix no wired scenario exercised, and
`a-hand-written-scenario-can-be-vacuous` is a demo file whose knob changed no
number. So the demo's coverage is measured the way both of those should have
been: a `panic!` is compiled into each site and the shipped file is run for its
documented 6 000 ticks. **`seed_from_output` is the control and MUST fire** —
without it, "nothing fired" cannot be told apart from a probe that never reached
the binary.

| site | reached by the demo's 6 000 ticks? |
|---|---|
| `PiController::seed_from_output` (the load-time seed) | **yes** — the control, and it panics |
| the anti-windup arm of `PiController::update` | no |
| `ProportionalController::update` | no |
| the `ControlMode::Manual` arm of the tick's control pass | no |
| the engine's range backstop on a controller's output | no |

So the demo exercises the loader's `[[controls]]` path, the tick order, the PI
algorithm's ordinary arm and the snapshot's `controls` array — and **nothing
else in the seam**. The anti-windup branch is not merely untaken by luck: the
run's output stays inside `[0.194, 0.384]` for all 6 000 ticks, so `unclamped`
never leaves `[0, 1]` and the branch cannot be entered. `ProportionalController`
is reached by no file in `scenarios/` at all.

Three further gaps need no probe, because they are facts about the runner rather
than about the plant: the CLI issues **no commands**, so `SetControllerMode`,
`SetSetpoint` and the AUTO refusal of `SetValveOpening` are unreachable from any
shipped scenario, and the MANUAL→AUTO transfer — the piece of M8.3 that cost the
most to get exact — has no wired exercise whatever. All of them are covered by
`control_reference.rs` and `level_control_demo.rs` on fixtures and by hand.

**What this does not license.** None of these is a defect and none un-defers
anything; the point of measuring is that "the repo has a demo that regulates" is
now a claim with a stated extent. A second wired plant — a P loop, or one whose
actuator saturates — would close the first two rows, and is not written here
because M8's remaining slice is the snapshot's, not another plant's.

### Deferred, with what un-defers each

- **Pressure, temperature and flow control.** Fork 1's shape is
  variable-agnostic; each needs a measurement path and an actuator that exists.
  Un-defers per variable, and pressure is the near one — a `Vessel`'s state IS a
  pressure, and a relief-free vessel currently has no way to be held anywhere.
- **Cascaded loops** — a loop whose setpoint is another loop's output.
  Un-defers when a plant has an inner loop fast enough to be worth separating;
  it needs an execution-order rule stronger than declaration order.
- **Derivative action.** Deferred because a D term on a measurement this project
  reports with one tick of lag and no noise model is tuning theatre — it would
  move numbers no gate could interpret. Un-defers with a plant whose loop is
  oscillatory enough for damping to be distinguishable from a lower gain.
- **Split-range, override, feedforward.** All three are "more than one writer of
  one actuator", which fork 5 refuses at load. They un-defer together, with a
  defined arbitration.
- **Actuator dynamics** (stroke time, rate limits) and **deadband.** Un-defer
  when a loop's measured performance depends on them, which at `dt = 0.1 s`
  against a 120 s integral time it does not. **The numbers are the note's, not a
  shipped plant's**: M8.4's demo is the only wired loop and runs `dt = 1.0 s`
  against a 600 s integral time, which is the same 1:600 ratio, so the reasoning
  survives on the plant that exists. The reason it was *incomplete* is above —
  reachability, not performance.
- **Interlocks and trips** — a discrete layer, not a regulating one. Un-defers
  with a safety case needing a plant to shut *itself* down.
- **The slate on the snapshot** (M6.2's deferral) stays exactly where it is, and
  this slice does **not** trigger it. A level controller reads
  `TankState::level(ρ)` inside the engine, which already holds the slate; the
  deferral's stated trigger is a *frontend* needing an absolute fill fraction, a
  per-component readout, or a component name. Satisfying one of three triggers
  by a route that never leaves the engine is not the un-defer condition, and
  treating it as one would spend a written decision without paying for it.

## 11. Solver robustness (M9) — specified before building

M9 opens the way M8 did, with a defect the previous milestone reached and did not
fix: **a branch driven to zero flow in one tick stalls the Newton hydraulic
solver.** M8 left it pinned by a characterization test and named a `newton_flow`
slice as its owner. This is that slice, and the first thing it did was falsify
the mechanism M8 recorded for it.

### The recorded mechanism is false, in both of its clauses

DESIGN §10 and the pin's own docstring say this:

> The residual falls monotonically, by about 0.36% per iteration: `4.158 → 3.344`
> over the 50-iteration cap. Newton is crawling, not oscillating and not stuck.
> `ΔP = α·Q|Q|` has an unbounded `dQ/dΔP` as `Q → 0`, so the step from a warm
> start carrying 3.3 kg/s is enormous and the line search cuts it back to nearly
> nothing.

Instrumenting the failing solve — every iteration's step, accepted `t`, per-edge
conductance, and the orphaned node's branch drop — refutes both halves:

- **The line search cuts nothing.** `accepted = true` at `t = 1` on all 50
  iterations, in both halves of the pin. No halving ever happens, so "the line
  search cuts it back" describes code that does not run.
- **No conductance blows up.** The shut valve's `alpha` is `+∞`, so its
  conductance is exactly **zero** — the opposite of unbounded. The one conducting
  edge into the orphaned node is an ordinary pipe whose conductance grows mildly
  across the failing solve, `7.400e-3 → 9.154e-3`.
- **The iterate oscillates.** The branch drop runs `+281.825, −279.839, +277.853,
  −275.868, …, −188.625, +186.646, −184.667` Pa — it changes sign every
  iteration. The residual falls monotonically anyway, because the residual is a
  function of `|drop|`. So the pin's own inference — "monotone, therefore
  crawling rather than oscillating" — does not follow, and is the second thing
  this slice has to rewrite.

The general lesson is one this project has paid for before: a monotone residual
history says nothing about the iterate's path, because the merit is even in the
error and the error is not.

### The real mechanism: Newton on the regularised square root is a shrinking 2-cycle

F6 (`validate_degrees`) forces a valve node to have exactly one inlet edge and one
outlet edge, so shutting a valve **always** leaves a node with exactly one
conducting edge — a dead leg with one unknown pressure. That reduces the failing
solve to a scalar problem, which can be done in closed form.

Let `x = Δp − β` be that branch's driving drop, `c` its conductance and

```
f(x)  = x / sqrt(|x| + ε)              (elements::smooth_signed_sqrt, ε = eps_dp = 1 Pa)
f'(x) = (|x|/2 + ε) / (|x| + ε)^{3/2}
```

so the node residual is `R = c·f(x)`, with its root at `x = 0`. The Newton step in
`x` is `f/f' = x(|x| + ε)/(|x|/2 + ε)`, and therefore

```
x₁ = x − f/f'   = −x · (|x|/2) / (|x|/2 + ε)      (full step, t = 1)
x₁ = x − ½·f/f' =  x · ε / (|x| + 2ε)             (half step, t = ½)
```

Two facts fall straight out of that pair, and between them they are the whole
defect and the whole fix:

- **The full step is the mirror image, shrunk by `2ε`.** `|x₁| = |x|·|x|/(|x| +
  2ε)`, so `|x| − |x₁| = |x|·2ε/(|x| + 2ε) → 2ε` for `|x| ≫ ε`. The iterate walks
  toward the root at **two pascals per iteration**, forever, whatever the plant.
  Measured decrements on the pin: `1.99, 1.99, 1.98, …, 1.98` against a predicted
  `2.0`.
- **The half step is nearly exact, and is bounded independently of the start.**
  `|x₁| = |x|·ε/(|x| + 2ε) < ε`. One `t = ½` step lands within a pascal of the
  root from *any* drop, however large.

So the two candidate steps are not "big and small". They are "the worst step
available" and "the answer". Everything below is about which one gets taken.

### Why the line search takes the wrong one

Merit is `φ = ½‖R‖₂²`, and on this scalar problem `φ ∝ f(x)²`. For `|x| ≫ ε`,
`f(x)² ≈ |x|`, so the full step's merit ratio is

```
φ₁/φ₀ ≈ |x₁|/|x₀| ≈ 1 − 2ε/|x₀|
```

The Armijo test at `t = 1` is `φ₁ ≤ (1 − 2·ARMIJO_C)·φ₀`, so the mirror step is

```
ACCEPTED   iff   |x₀| ≲ ε / ARMIJO_C
```

At the shipped `ε = 1 Pa` and `ARMIJO_C = 1e-4` that threshold is **10 000 Pa**.
The pin's plant starts the failing solve at 282 Pa, far below it, so the mirror
step is accepted every time and the solve crawls at 2 Pa per iteration:
282/2 = 141 iterations needed against a `max_iter` of 50. The by-hand half starts
at 114.6 Pa, needs 57, and misses the cap by seven.

**The comment above the line search already states the correct intent** — "merely
requiring *any* decrease would accept the √-law's near-symmetric overshoot (t=1)
and stall; Armijo rejects it and forces t ≤ ½". That sentence is right. The
constant underneath it does not implement it. The defect is a mis-sized constant
under a correct comment, which is worse than a wrong comment, because the comment
is what stops the next reader from checking.

### The stall window, and the relation between three constants

A solve stalls exactly when the mirror step is accepted *and* the resulting 2 Pa
crawl cannot finish inside the cap:

```
2·ε·max_iter   <   |x₀|   ≲   ε / ARMIJO_C
```

Three consequences, and the middle one kills the reflex fix:

1. At the shipped constants the window is `(100 Pa, 10 000 Pa]` — wide, and
   sitting squarely where ordinary plants live.
2. **`ε` cancels.** It scales both bounds identically, so changing `eps_dp`
   cannot open or close the window. "The regularisation is the problem, shrink
   it" is the obvious move and it is inert.
3. The window is empty iff **`ARMIJO_C ≥ 1/(2·max_iter)`**, which is `1e-2` at
   the default `max_iter = 50`. That is a relation between two constants that
   have never had anything to do with each other, and it is the thing this slice
   has to write down somewhere a reader will find it.

**Verified by prediction rather than by fitting.** At `ARMIJO_C = 3e-3` the
relation predicts a window of `(100 Pa, 333 Pa]` — a *non-monotone* signature,
converging at both ends of a drop sweep and stalling in the middle, which a
merely-improved solver cannot fake. Sweeping the dead-leg pipe diameter (the drop
falls roughly as `d⁻⁵` at nearly constant flow) produced exactly that: `0.08`
converged, `0.09` converged, `0.10`/`0.11`/`0.12` stalled, `0.13`/`0.14`/`0.16`
converged.

### Reachability: it is a drop window, not a plant shape

Three topologies were built and all three reproduce it, so nothing about the
pin's plant is special:

- **scalar** — the controlled valve straight off the tank (the pin's own plant);
- **chain** — a junction inserted between tank and valve (`4.1577 → 3.3442`
  against the scalar's `4.1579 → 3.3445`);
- **branching** — a tee with two valves shut in the same tick, which converges at
  the shipped diameters and stalls once the legs are narrowed to `d = 0.075`.

That last one is the trap this slice nearly fell into. The tee converging first
time reads as "branching cures it, the defect needs an isolated dead leg" — a
topological conclusion, and false. Narrowing the legs brought the stall straight
back. What decides is `|x₀|`, and a plant lands inside or outside the window for
reasons that have nothing to do with its shape.

Combined with F6, this is common rather than exotic: every shut valve in every
plant orphans a node with exactly one live edge, and whether that plant survives
being shut in one tick was decided by an undocumented pressure band.

### The forks

**Fork 1 — raise `ARMIJO_C` so the derived relation holds. CHOSEN.**
One constant, no new code path, and it makes the line search do what its own
comment already claims. `ARMIJO_C: 1e-4 → 5e-2`.

Not `1e-2`, though `1e-2` is what the relation demands: at `1e-2` the two bounds
merely *touch*, and the derivation is leading order in `ε/|x|`, so the neglected
terms decide the boundary. Measured rather than assumed — at `1e-2` a surviving
stall band was found at `d = 0.124` (residual stuck at 0.6604) and `d = 0.126`
(0.0328). `5e-2` is a factor of five of margin on the relation, and it closed
every sample taken.

The standing objection to a stricter sufficient-decrease demand is that it
rejects legitimate steps and turns a slow solve into a reported divergence. On
this residual it does the opposite, and the closed form above says why: rejecting
`t = 1` forces `t = ½`, and `t = ½` lands within `ε` of the root. **Measured
worst-case Newton iterations per pass, over 6 000 ticks of all fourteen shipped
scenarios**, against a cap of 50:

| | worst pass, `1e-4` | worst pass, `5e-2` |
|---|---|---|
| `tank_level_control` | **11** | 9 |
| `knockout_drum` | 10 | 7 |
| `relief_blowdown` | 10 | **10** |
| `leaking_line` | 9 | 8 |
| `tank_pump_valve` | 9 | 9 |
| `gas_line`, `gas_valve` | 8 | 8 |
| `fcc_plant` | 6 | 7 |
| `heat_recovery` | 5 | 5 |
| `crude_column`, `crude_column_cascade` | 4 | 4 |
| `fcc_reactor` | 3 | 3 |
| `cooler_chiller`, `furnace_heater` | 0 | 0 |

The worst case across the corpus **falls**, 11 → 10, and one scenario rises by a
single iteration. Both leave a factor of five under the cap. "It has margin" is a
measurement here, not an assertion.

**Fork 2 — raise `max_iter` instead.** The relation closes from below as well as
from above, so `max_iter ≥ 1/(2·ARMIJO_C) = 5 000` also empties the window.
Rejected: every iteration is a dense LU, so this funds the crawl rather than
removing it, and it makes a genuinely divergent plant take a hundred times longer
to say so. It is recorded because it proves the window is a *relation* and not a
property of either constant alone.

**Fork 3 — shrink `eps_dp`.** Inert, per the cancellation above. Recorded because
it is the reflex: the regularisation looks like the culprit and is not.

**Fork 4 — a trust region in branch-drop space**, refusing any step that reverses
the sign of a branch's `Δp − β`. Scale-free, constant-free, and exactly right
here: the mirror step *is* a sign reversal, and the half step is not. Deferred,
not rejected — a legitimate solve does reverse a branch's drop (a PSV in reverse
flow multi-roots the network, M5.4; reverse flow through a tee leg is ordinary),
so the rule would have to be "reverses *and* does not shrink", which is a new
criterion with its own blast radius across the generated-network arm. **Un-defers
if a plant is found whose stall survives fork 1** — that is, one where the mirror
step is correctly rejected and the halved step is still not enough.

**Fork 5 — quadratic-interpolation backtracking**, choosing `t` by fitting the
merit rather than halving. Rejected as redundant: the existing backtrack already
tries `t = ½` second, and `t = ½` is within `ε` of the root. What was missing was
never a better `t`; it was a criterion for rejecting `t = 1`.

### What the gate has to assert, and what it must not

The pin becomes a convergence gate, and the obvious rewrite — "both halves now
return `Ok`" — is refused for the reason M3.2 recorded: a solve can converge,
conserve mass and rerun bit-identically while frozen wrong. A line search that
accepted anything at all would also return `Ok` here.

So the gate asserts the **endpoint**, through an identity that does not depend on
how much the tank drained on the way there: with the branch shut, the orphaned
valve node carries no flow, so its pressure must equal its neighbour's less the
static head — on this plant, exactly the tank's bottom pressure. That is checked
on both routes to the state (the loop's setpoint step and a hand
`SetValveOpening`), and against the third route M8 recorded as already
converging, the gradual `0.20 → 0.00` over 20 ticks. Three independent paths to
one identity.

The **sweep is the other half of the gate and needs its own control.** 171
diameters across `d ∈ [0.060, 0.400]` all converge at `5e-2` — a result that is
worthless alone, because a sweep that never fails cannot distinguish a fixed
solver from a vacuous probe. Run at `1e-4` the identical sweep stalls on 24 of
171, a contiguous band `d ∈ [0.060, 0.106]`, whose upper edge is where the drop
falls below `2·ε·max_iter = 100 Pa` — and the last stalling sample sits at a
residual of `5.5e-5`, i.e. the crawl nearly finished inside the cap. The band's
location is predicted by the relation, not fitted to it.

### The gate's two assertions, each proven on its own

Three edits were compiled, run and restored from one pre-mutation snapshot.

| the edit | what fired |
|---|---|
| `ARMIJO_C` back to `1e-4` | all three: the coupling unit test, the convergence gate, the demo's gain gate |
| `tol_abs_kg_s: 1e-8 → 1e-3` — a solve that returns `Ok` while stopping short | the convergence gate, on flow (`2.6e-4 kg/s`) and, with that assertion relaxed, on pressure (`1.1e-3 Pa`) |
| the control pass moved BELOW the solve | the convergence gate alone, on flow — `4.16 kg/s` through a branch it says is shut |

The second is the one that justifies the gate's shape: it returns `Ok`, so an
`is_ok()` rewrite would have passed it. Each of the two assertions was checked to
fire alone, by relaxing the other — the flow bound is hit first otherwise, and a
gate whose second assertion has never been reached is a gate with one assertion.

### It closes a gap M8.4 recorded as open, and that was not the aim

M8.4's mutation pass predicted that "the loop runs AFTER the solve" would go
**uncaught**, and found that it was caught — by the stall pin, which asserted the
next tick returns `Err` and therefore fired when the reordering made it converge.
M8.4 was careful about what that meant: the catch came from a test written to be
*deleted* when the solver was fixed, so it recorded the gap as still open —
**"no gate asserts the tick ORDER"** — rather than claiming it filled.

Fixing the solver was exactly the event that was supposed to lose that catch.
Instead the rewritten gate keeps it and improves it: with the control pass below
the solve, the tick solves with the valve still open, and the dead-leg assertion
sees `4.16 kg/s` running through a branch the plant says is shut. That is a
*state* being asserted rather than a failure being expected, so it survives the
next solver change too, and `the_reported_measurement_is_the_one_the_controller_
acted_on` still does not fire on this edit — the tick-order gap is closed by the
gate that used to be upside-down, not by the gate that claims the subject.

The general form is worth keeping: **an upside-down test can be carrying a real
gate, and the way to find out is to re-run the mutations it caught after turning
it the right way up.** Deleting it and trusting the named gates would have
reopened a gap that M8.4 paid to discover.

### Blast radius, measured

All fourteen shipped scenarios declare `flow = "newton"`, but **twelve of them
actually execute the changed comparison and two structurally cannot.**
`cooler_chiller` and `furnace_heater` converge at iteration zero on every pass of
every tick — the seed already satisfies the tolerance, so the loop body holding
the Armijo test never runs. Their byte-identity is guaranteed by construction and
is evidence of nothing, which is worth saying out loud: counting them as passes
would be the degenerate-fixture mistake this project has made before, where a
reference plant with no free node never ran the solver loop at all.

Over 6 000 ticks: **ten of fourteen are byte-identical — eight of the twelve that
could have moved**; `fcc_plant`, `knockout_drum`, `leaking_line` and
`tank_level_control` move. The largest
*relative* move in the whole corpus is `3.96` — on `solver.residual`, a
diagnostic whose target is zero and whose two values are `9.6e-14` and `4.8e-13`,
so a relative comparison there measures nothing. Excluding that block, the worst
move on any physical quantity is `7.6e-11` relative (`4.7e-14` absolute) on an
`fcc_plant` mass fraction of 0.06%, and the worst on any bulk quantity — a mass,
a temperature, a dissipation — is `3.3e-13` relative. Nothing physical moves.

The whole workspace suite passes except the two tests written to fail when this
is fixed, and the generated-network property arm passes.

### Deferred, with what un-defers each

- **Fork 4's trust region**, above. Un-defers on a stall that survives fork 1.
- **The crawl itself, on the accepted side.** Below `2·ε·max_iter` the solver
  still walks in 2 Pa steps; it now always finishes, but a plant whose drop is
  90 Pa spends 45 iterations doing what one halved step would do. Harmless at the
  measured worst case of 10, and it is the same code fork 4 would replace.
- **`max_iter` is a `pub` field.** A caller constructing `NewtonFlowSolver` with
  `max_iter: 10` reopens the window at `ARMIJO_C = 5e-2`, and nothing refuses it.
  No scenario file can set it — nothing in `crates/scenarios/src` mentions
  `max_iter`, `tol_abs_kg_s` or `eps_dp` — so this is a code-level invariant, and
  it is written on the field's own doc comment and asserted against the default
  by a unit test. Un-defers if the solver's numerics ever become scenario config,
  at which point it must become a load-time refusal.

### M9.1 — the mirror step on the game fidelity — specified before building

M9.1 was scoped by a probe sent to ask a binary question: M9.0 fixed the shut-in
stall in `NewtonFlowSolver`, all fourteen shipped scenarios declare
`flow = "newton"`, so does the same defect live in `SimpleFlowSolver`? The answer
is yes, and the measurement that produced it moved the subject.

**The shut-in is not the defect. It is where the defect stops finishing.**
`SimpleFlowSolver` has no step-rejection criterion of any kind — it applies
`P += ω · imbalance/g_sum` unconditionally — so it takes the step M9.0 proved is
the worst one available on *every* valve node of *every* plant, always. The
shut-in is the limit where that step makes no progress at all.

#### The probe's question, and why the answer changed the subject

M8.2's control fixture, tick 0, `dt = 0.5 s`, the solver called directly with the
drain valve's opening varied. **The `[[controls]]` table was deleted from the
fixture for this measurement**, so nothing here is the control loop's: the counts
are identical with the loop present, which is what makes the defect the solver's.

| drain valve | `SimpleFlowSolver` | `NewtonFlowSolver` |
|---|---|---|
| `0.20` (ordinary, open) | 189 | 8 |
| `0.05` | 923 | 8 |
| `0.01` | 3874 | 8 |
| shut | **diverged at the 5 000 cap**, residual 77.2 kg/s | 8 |

The first row is the finding. A valve at 20% open is an ordinary operating point
with nothing shut anywhere, and it costs 189 sweeps against Newton's 8. Tracing
which node is still moving late in the sweep says why: at `0.20` **the binding
node is not the drain valve at all**. It is `feed_valve` — a fully open valve on a
fixed 5 bar header, on the opposite side of the plant — and it takes **189 sweeps
in all four runs, identically**, because the drain's opening has nothing to do
with it. The drain valve only becomes the binding node once throttled below about
`0.05`.

So "the shut-in stall on the other fidelity" is the wrong frame. The right one is
that this solver overshoots on ordinary plants routinely, and the shut valve is
the tail where the overshoot stops converging.

#### The mechanism, measured on the failing solve

M9.0's own lesson is that a recorded mechanism can be false in both of its
clauses, so this one was instrumented rather than inherited: every sweep's node
pressures, on the failing solve and on the three converging ones.

**On the shut valve it is exactly the closed form.** The dead leg's branch drop
`x = p_tank − p_valve` changes sign every single sweep and its magnitude falls by
**2.0000 Pa per sweep, unvarying across all 5 000 of them**, starting from
`|x₀| = 106 790.90 Pa`. §11's algebra predicts `|x| − |x₁| = |x|·2ε/(|x| + 2ε)`,
which at `ε = eps_dp = 1 Pa` is `1.99996`.

That is a prediction, so it was checked as one rather than fitted: the crawl needs
`106 790.90 / 2 = 53 395` sweeps, so raising `max_iter` to 100 000 should converge
at about that count. **Measured: 53 411**, 0.03% out.

**On a conducting valve it is the same overshoot with a contraction factor.** The
sign still alternates every sweep, but the magnitude contracts geometrically
instead of by a constant, because the second, conducting branch damps the
reflection. Net decrement per sign-pair, as a fraction of `|x|`:

| drain valve | contraction per pair |
|---|---|
| `0.20` | `1.10e-2` |
| `0.05` | `3.05e-3` |
| `0.01` | `6.57e-4` |

— roughly proportional to the valve's own conductance. **So it is one mechanism
across the whole column, and the shut valve is the limit where the contraction
factor reaches exactly 1 and only the additive `2ε` is left.** "Continuous, not a
cliff" is that limit being approached continuously; it is *not*, as first
supposed, the branch drop growing as the valve closes. The drop barely moves
(`109 529 → 106 791 Pa` across the four runs) and is not what separates them.

#### The window is unbounded above, and that is a difference in kind

§11 derives Newton's stall window as

```text
2·eps_dp·max_iter   <   |Δp₀|   ≲   eps_dp / ARMIJO_C
```

with an upper bound that exists **because** Armijo eventually rejects the mirror
step. `SimpleFlowSolver` rejects nothing, so there is no upper bound:

```text
2·eps_dp·max_iter   <   |Δp₀|   <   ∞          (10 kPa, ∞) as shipped
```

Two consequences, and the first is the whole argument for this slice:

1. **No value of `max_iter` closes this window.** §11's fork 2 — fund the crawl
   with a bigger cap — was rejected there on cost, because every Newton iteration
   is a dense LU. Here it is rejected on principle: raising the cap moves the
   lower bound and the window stays infinite. A rejection criterion is not one of
   several ways to fix this; it is the only one.
2. **It is reachable structurally rather than by an unlucky plant.** The cold
   seed in `network::classify` is the mean of the pinned pressures, so on this
   fixture the free nodes start ~200 kPa from their roots — twenty times the
   window's lower bound — before anybody touches a valve. Any plant whose
   reservoirs span a few bar starts inside the window on tick 0.

#### The forks

**Fork 1 — a per-node sufficient-decrease test in the sweep. CHOSEN.**
Require `|R_after| ≤ (1 − c·t)·|R_before|` on the node's own scalar imbalance,
halving `t` until it holds. It is the node-wise analogue of what M9.0 already
made Newton do, it needs no new state, and by the closed form it cannot cost more
than one halving on the case it exists for: `t = ½` lands within `ε` of the root
from any drop.

**Fork 2 — raise `max_iter`.** Rejected on principle, per the unbounded window
above. Recorded because it is the move that works on the other fidelity.

**Fork 3 — lower the default `ω`.** Rejected, and the measurement that kills it
also explains it. Worst sweeps in any tick over 500 ticks of all fourteen shipped
scenarios, forced onto this fidelity:

| scenario | `ω = 1.0` | `0.9` | `0.75` | `0.5` |
|---|---|---|---|---|
| `relief_blowdown` | 868 | 1079 | 1520 | **3035** |
| `gas_valve` | 459 | 31 | 18 | 22 |
| `tank_level_control` | 78 | 22 | 16 | 30 |
| `fcc_plant` | 16 | 20 | 28 | 51 |
| `leaking_line` | 18 | 15 | 12 | 30 |
| `crude_column`, `crude_column_cascade` | 3 | 7 | 10 | 20 |

The corpus worst case gets 3.5× worse, because `relief_blowdown`'s convergence is
driven by its vessel's own `−C/dt` accumulation term rather than by branch
conductance, and damping that funds nothing. A global `ω` trades one plant's
stall for another's cost.

**And the decisive measurement is that damping and the line search are the same
remedy.** With fork 1 in place at `c = 5e-2` and `ω = 0.5`, the corpus reproduces
the *no-line-search* `ω = 0.5` column exactly — `relief_blowdown` 3035,
`fcc_plant` 51, `crude_column` 20 — because at `ω = ½` the line search never
fires: the half step already passes its own test. Fork 3 is fork 1 applied
unconditionally to every node of every plant, and fork 1 is fork 3 charged for
only where it is needed. `ω` therefore survives as a field, its default stays
`1.0`, and its doc comment has to stop advertising itself as the remedy for
stiffness.

**Fork 4 — a better cold seed.** Rejected as unable to reach the case. A valve
shut between two ticks moves its node's root by the whole branch drop in one tick,
so no seeding policy can anticipate it; and `network::prepare`'s own comment
already establishes that the seed is a path rather than an answer. It would
shorten the crawl on tick 0 and leave every later one.

**Fork 5 — port §11's own fork 4, the sign-reversal trust region, node-wise.**
Built and **measured, and it does not fix the defect.** "Reject a step that
reverses the node's imbalance without shrinking it" is a large corpus win —
`gas_valve` 459 → 7, `tank_level_control` 78 → 12 — and on the shut-in it
reproduces the divergence **bit for bit**: the same 5 000 sweeps, the same
`7.719e1` residual. The reason is the qualifier §11 wrote into the rule itself:
the mirror step *does* shrink the imbalance, by `2ε` worth, so "reverses *and*
does not shrink" is never satisfied on exactly the case the rule was written for.

This does not fire §11's un-defer trigger for Newton — that trigger is "a stall
that survives fork 1", and none has been found. What it does is supply evidence
against the deferred rule from the other fidelity: it is attractive, it is
scale-free, and on the one case it was designed for it is inert.

#### The constant is Newton's number and is deliberately not Newton's constant

The two tests are the same test. Newton compares `φ_t ≤ (1 − 2c·t)·φ` on
`φ = ½‖R‖₂²`; the per-node form compares `|R_t| ≤ (1 − c·t)·|R|` on a scalar
residual. Since `φ ∝ R²` and `(1 − ct)² = 1 − 2ct + O(c²t²)`, **the factor of two
is the square and not a tuning choice**, and `c` means the same thing on both
sides.

The *justification* for `5e-2` does not transfer, though, and this is the trap.
Newton's bare relation bound is `1/(2·max_iter) = 1e-2` at its cap of 50; here
`max_iter = 5000` makes the same bound `1e-4`, five hundred times slacker. So the
number was re-derived by measurement on this solver:

| `c` | drain `0.20` | `0.05` | `0.01` | shut | `relief_blowdown` worst |
|---|---|---|---|---|---|
| none | 189 | 923 | 3874 | **diverged** | 868 |
| `1e-4` | 84 | 258 | 1239 | 7 | — |
| `1e-3` | 84 | 7 | 8 | 7 | 774 |
| `1e-2` | **7** | 7 | 8 | 7 | 847 |
| `5e-2` | 7 | 7 | 8 | 7 | **920** |
| `2e-1` | 7 | 7 | 7 | 7 | 1325 |

Three things to read off it:

- **The relation's own bound closes the shut-in and leaves the crawl.** At
  `1e-4` the fully shut valve converges in 7 sweeps and a valve 1% open still
  takes 1239. The relation is about the dead leg, where progress is additive; the
  near-shut nodes converge geometrically and need a test strict enough to reject
  the mirror on a *conducting* node. This is the same shape M9.0 found at its own
  bare bound, for a different reason — which is the argument for measuring the
  margin on each solver rather than porting it.
- **The knee is between `1e-3` and `1e-2`**, and above `1e-2` the fixture is
  saturated at 7–8 sweeps whatever the opening.
- **The cost lands on one plant and rises monotonically with strictness.**
  `relief_blowdown` is the corpus worst either way; `5e-2` costs it 6% and `2e-1`
  costs it 53%.

`5e-2` is five times the measured knee and nearly free. That it is also Newton's
number is a coincidence of two independent measurements, and the two are
deliberately kept as **separate constants in separate files, each naming the
other**: a shared one would let a future re-tuning of Newton's margin — which is
tied to *its* `max_iter` of 50 — silently move a solver whose bound is five
hundred times slacker. Rule 2's independent implementations, applied to a number.

#### What the gate has to assert, and the one M9.0 wrote that cannot be mirrored

The endpoint, not `Ok(())` — M9.0's rule, and it binds harder here, because the
table above contains a value (`1e-4`) at which the shut-in converges and the
defect is still there. A gate that only watches the shut valve would pass it.

So the gate is two assertions on one fixture:

- **the shut branch's endpoint**, the identity M9.0 already gates on Newton — no
  flow through the shut branch, and the dead leg sitting at the tank's bottom
  pressure. It is fidelity-independent by construction, which is the point: both
  solvers must land on the same state, so this is an arm on the existing gate
  rather than a new one.
- **a sweep budget on the throttled fixture**, at 1% open. This is the vacuity
  control for the first: it is what fails at `c = 1e-4`, where the endpoint
  assertion passes.

**M9.0's constant-relation unit test cannot be mirrored here, and shipping a copy
of it would be worse than shipping nothing.** With fork 1 in place this solver's
window takes Newton's form, `(2·eps_dp·max_iter, eps_dp/c]`, empty iff
`c ≥ 1/(2·max_iter)`. At `max_iter = 5000` that is `1e-4`, which `5e-2` clears by
a factor of five hundred — so the assertion would pass at almost any constant a
later reader chose, and would read as coverage while providing none. The thing
that actually needs gating on this fidelity is that a rejection criterion exists
at all, and the endpoint gate is what asserts it.

#### The branch where no step is acceptable, and how often it is reached

If none of the halvings satisfies the test, the node is left where it is and the
sweep moves on — a well-defined Gauss–Seidel choice (a node whose own scalar step
cannot improve its own balance is one for its neighbours to move) and the honest
one, since the alternative is applying a step the criterion just rejected.

That branch is reached **3 281 times across 500 ticks of all fourteen scenarios**,
which sounds like a hot path and is not: **every one of those sites has
`|imbalance| ≤ 3.4e-13 kg/s`**, five orders of magnitude below the solver's own
`tol_abs_kg_s = 1e-8`. It is the floating-point noise floor on an already-converged
node, where `(1 − c·t)·|R|` is unreachable by rounding and the step being dropped
is a no-op. Recorded because "it fires 3 281 times" is exactly the kind of number
that would otherwise be read as a defect by whoever finds it next.

#### The cost, in wall time rather than in iterations

The scoping probe refused to claim this solver was slow from iteration counts
alone, on the grounds that a sweep is `O(edges)` with no linear algebra while a
Newton iteration is a dense LU. The same refusal applies to the fix: the line
search costs up to nine extra imbalance evaluations per node per sweep, and
iteration counts cannot say whether the sweeps it saves pay for them.

Release build, 500 ticks, three runs each, best of three, in milliseconds:

| scenario | before | after |
|---|---|---|
| `relief_blowdown` | 91.2 | 96.3 |
| `gas_valve` | 4.4 | 3.9 |
| `tank_level_control` | 5.8 | 5.8 |
| `leaking_line` | 5.1 | 5.8 |

`relief_blowdown`'s +6% in sweeps does not clear run-to-run spread, and no
scenario moves outside it. **The evaluations are paid for by the sweeps they
save**; the claim is "unchanged at this resolution", not "faster".

#### Deferred, with what un-defers each

- **`ω` is a `pub` field and no scenario file can set it** — the same shape as
  §11's `max_iter` deferral, and the same disposition: a code-level invariant
  written on the field's own doc comment, saying that the line search now owns
  what damping used to be for and that lowering `ω` is *measured* to cost.
  Un-defers if the solver's numerics ever become scenario config.
- **`relief_blowdown`'s 868 sweeps are a different mechanism and this slice does
  not touch them** (868 → 920). `relief_valve_reference.rs` records it: a
  normally-shut PSV leaves its valve node a dead end, so the receiver's
  Gauss–Seidel diagonal is dominated by a fat inlet branch carrying no net flow.
  That is a preconditioning problem, not an overshoot, and the measurement above
  separates them — the fix that takes `gas_valve` from 459 to 7 leaves this plant
  where it was. Un-defers if a plant of that shape reaches the cap.
- **The cold seed**, fork 4. Un-defers only alongside a reason other than this
  one, since it cannot reach the shut-in case at all.

#### The mutation pass, and the two predictions it falsified

Six edits, each restored from a single pre-mutation snapshot with the mtime
moved, each verified to have applied *and* compiled, `cargo test --workspace
--no-fail-fast` so no binary truncates the catch set.

| # | edit | predicted | fired |
|---|---|---|---|
| 1 | `ARMIJO_C: 5e-2 → 1e-4` | throttled gate alone | **throttled gate alone** |
| 2 | accept every step unconditionally | both new gates | **both new gates** |
| 3 | apply `full/256` where the search rejects | uncaught | **uncaught** |
| 4 | trial imbalance drops the capacitance term | — | `both_fidelities_agree_on_a_capacitive_plant`, `both_fidelities_settle_the_relief_…` |
| 5 | `MAX_HALVINGS: 8 → 1` | uncaught | **`a_relief_that_shuts_on_the_way_to_the_answer_…`** (passes at 2) |
| 6 | default `ω: 1.0 → 0.5` | — | shut-in gate (`simple` arm), `both_fidelities_settle_the_relief_…` |

**Mutation 6 is a correctness result, and fork 3 above argues it as a cost one.**
At `ω = 0.5` the shut-in fixture's solve returns `Ok` and leaves `3.77e-6` kg/s
through a branch that is shut. That sits inside this solver's own convergence
criterion — `tol_abs + tol_rel·throughput` is about `4e-6` at that plant's rate —
and outside the `1e-6` the endpoint gate allows. So the damping does not merely
buy fewer correct answers per second; it buys a **wrong endpoint reported as
converged**, on the one plant the slice was written for. The 3.5× sweep table is
the weaker half of the argument for leaving `ω` at 1.0, and the field's own doc
comment now leads with this instead.

**Mutation 1 is the one this note stakes a claim on in print, and it holds.**
The bare bound the stall relation licenses at this cap converges the shut branch
and leaves a valve 1% open crawling, so the endpoint gate stays green and the
sweep budget beside it is what fails. The two gates divide the work the way the
note says they do — measured, not asserted.

**Mutation 5 falsifies something this note implies, and the correction is worth
more than the mutation.** §11's closed form says a HALF step lands within `ε` of
the root *from any drop*, which reads as "one halving is enough" and makes this
edit inert — the prediction, and it is wrong. `MAX_HALVINGS = 1` diverges M8.0's
anchoring plant at 20 000 sweeps, residual `5.397e1`. **The closed form's reach is
narrower than the constant it was being used to justify.**

**How much narrower was bisected rather than argued**, because the first draft of
this paragraph did exactly what M8 did: fitted a mechanism to one divergence.
Measured on the failing test, then on the whole workspace:

| `MAX_HALVINGS` | `a_relief_that_shuts_…` | `cargo test --workspace` |
|---|---|---|
| 1 | diverges, 20 000 sweeps, residual `5.397e1` | — |
| 2 | passes | **0 failing** |
| 3 | passes | 0 failing |

So one node, on one plant, needs `t = ¼`. **Why it is outside the closed form is
not measured.** The form was derived on a dead leg — F6 leaves the orphaned node
exactly one live edge, so the mirror is exact — and the natural reading is that a
second live edge shifts the root off the mirror. That is a candidate fitted to a
single data point, not a result, and nothing in this slice tests it. It is
recorded as a candidate for that reason.

Two consequences worth stating plainly. **`8` is six halvings of margin over
anything the corpus needs**, so it is *not* load-bearing; it is inherited from
`newton_flow`, where it has never been justified either. And cutting it to the
measured `2` would be fitting a constant to today's fourteen plants — the same
move fork 3 was rejected for.

**Mutation 4 fails in a shape worth recognising: the residual FREEZES.**
`0.02701386453465011` for all 5 000 sweeps, identical to the last digit. A line
search whose trial evaluation is not the same function the step was derived from
does not converge slowly — every `t` is rejected, `step = 0`, and the node never
moves again. Both catches are pre-existing M5-era cross-fidelity tests; **neither
new gate sees it**, because both watch a valve and this breaks a vessel.

**Mutation 3 is uncaught, and the depth bisection makes the gap quantitative.**
Nothing gates the branch where no `t` is acceptable. A gate would need a plant on
which that branch fires at an imbalance big enough to matter, and the corpus says
none exists: 3 281 reject sites across all fourteen scenarios, every one at
`|imbalance| ≤ 3.384e-13 kg/s` — five orders below `tol_abs_kg_s`. The bisection
above says the same thing from the other side: no node anywhere needs a `t` below
`¼`, so halvings three through eight are never the accepted step and everything
reaching the bottom of the loop was already at the noise floor. Not "untested, and
no gate is possible", but **unreachable at a load-bearing imbalance, with the
margin measured at six halvings.**

#### Reachability: no shipped scenario runs this file

The corpus numbers above and the "blast radius is nil" claim both rest on all
fourteen shipped files declaring `flow = "newton"`. Grep proves the text; a
`panic!` at `SimpleFlowSolver::solve` proves the call. All fourteen at default
fidelity: **0 of 14 fire**, with `leaking_line` forced to `simple` as the control
that must, and does.

**The first run of that probe reported 14 of 14, and the false positive is the
part to remember.** `panic!` at the top of a function makes the rest of it
unreachable, so rustc emits a diagnostic that **echoes the panic's own source
line** — and `cargo run` replays cached build warnings on every invocation. The
probe was matching the compiler, not the program. Build once, then invoke the
binary, and require `panicked at` alongside the marker.

### M9.2 — the stopping rule, and the scale that belonged to another node

M9.0 and M9.1 were both about which step the solver takes. This slice is about
when it is allowed to stop, which is a different question and the one neither
touched.

#### The note and the code disagreed, and the note was the one that was right

DESIGN §3 has said since M1:

> Convergence: relative mass-imbalance **< 1e-8 per node**, max 50 iterations.

The code said something else, and had since M1 in `newton_flow` and since the
Simple sweep was written:

```text
max_n |R_n|   <   tol_abs + tol_rel · max_e |ṁ_e|
```

Per node in neither factor. `throughput = max_e |ṁ_e|` ranges over the WHOLE
network (`network::edge_flows`), so **the error budget granted to any one node
was set by the largest pipe anywhere in the plant** — a quantity with no relation
to the equation being graded. A mass balance at a node is a sum of that node's own
incident flows; nothing about a 10 kg/s trunk says how exactly a spur carrying
10 g/s must balance.

This is not an argument that a constant was too loose. It is the code catching up
with a specification that was already written, and that framing is what licenses
changing a settled decision (M5.4's rule: verify the premise, not just the
verdict). The rule now lives in one place, `network::grade_nodes`, which both
fidelities call — so "the two solvers stop on the same criterion" is structural
rather than a convention two files have to keep.

#### What the old rule granted, and where it bound

`tol_rel` is `1e-8` on Newton and `1e-6` on Simple. On a plant moving 4 kg/s the
Simple solver therefore allowed EVERY node an imbalance of 4e-6 kg/s, including a
node whose own branches carried nothing. That is the mechanism behind M9.1's
`ω = 0.5` finding — `Ok` returned with `3.77e-6` kg/s through a branch the plant
says is shut, inside the solver's own tolerance and outside the gate's `1e-6`.
The `ω` was the path; **the criterion is why the endpoint was accepted**.

**Where the numbers in this section come from**, since they cannot be reproduced
from the shipped code: a temporary `network::probe_report`, called at each
solver's ACCEPTING iterate from `newton_flow` and `simple_flow`, gated on a
`REFINERY_PROBE` environment variable, printing per solve the accepted residual,
the plant-wide throughput, and four readings of the same criterion (`max` and `Σ`
local scales, each with and without accumulation). It was deleted before this
slice was committed, as M8.4's panic-probe was. Every ratio below is
`accepted |R_n|` over the bar named in its column heading; reinstating the probe
from this description reproduces them.

Measured across 500 ticks of all fourteen shipped scenarios, both fidelities
(accepted residual as a fraction of the bar, worst per plant):

- The rule BINDS — the accepted residual sits at 0.64–0.9997 of the bar — on
  `gas_line`, `gas_valve`, `knockout_drum`, `relief_blowdown` and
  `tank_level_control` at the shipped fidelity. It was not a vestigial check.
- But the shipped corpus barely exercises the defect. Reading the same tolerance
  per node, only two plants exceed it at all: `tank_level_control` at 2.30× and
  `relief_blowdown` at 1.015×, one node each.

**So the corpus is not the reachability argument. The fixture is.** On
`a_branch_shut_in_one_tick_converges_whoever_shuts_it` — the plant M9.0 and M9.1
were both spent on — at the shipped `ω = 1.0`, over 24 022 solves per fidelity:

| fidelity | solves with ≥1 node over its own bar | worst ratio |
|---|---|---|
| newton | 2 177 (9.1%) | 3.39 |
| simple | 159 (0.7%) | **127.3** |

The same node both times: the dead leg behind the shutting valve. Simple's worst
is `|R| = 3.13e-6` kg/s against a local flow scale of `1.46e-2` kg/s, while the
plant-wide reading says 0.74 of tolerance — comfortably "converged".

#### The forks

**Fork 1 — the scale is `max` over the node's own incident active edges.**
Rejected: `Σ_e |ṁ_e|`, the textbook scaled residual. The deciding property is not
a measurement: `max_incident ≤ throughput` at every node of every plant, so the
new criterion is **never looser than the one it replaces**, anywhere. `Σ` is
looser at any node with more than one live edge, which would let some plants stop
EARLIER than they do today — the wrong direction for a slice that exists because
the rule is too slack. The corpus shows `Σ` hiding the very violation it would be
introduced to expose: on `relief_blowdown` the `Σ` reading is 0.771 and the `max`
reading is 1.015.

**Fork 2 — accumulation stays out of the scale.** `assemble` already documented
the exclusion for `throughput`: the convergence scale is the network's mass flow,
and a vessel's accumulation is measured against that, not added to it. A local
scale SHARPENS that reason rather than inheriting it. Including `|−C·ΔP/dt|` would
hand `relief_blowdown` a bar twice as loose (0.9999 → 0.505 on the Simple sweep)
on the one node whose convergence M9.1 measured as driven by that very term — a
quantity grading itself, which this project has been burned by twice (M7.4b,
M7.4c). The consequence is stated rather than discovered: on a dead leg every
incident flow is ~0, so the bar collapses to `tol_abs = 1e-8`.

**Fork 3 — the reported residual is unchanged.** `SolveDiagnostics.residual`
stays `‖R‖_∞` in kg/s: it is a diagnostic, and the worst absolute node imbalance
is what a diagnostic in kg/s should say. Reporting a scaled residual would change
what every snapshot's `solver.residual` means, for a gain nobody asked for.

**Fork 4 — `tol_rel` is not re-tuned.** The shape of the criterion changed, not
the constant. `1e-8` and `1e-6` now mean what they always read as — a relative
error on the node's own flow — so re-fitting them here would be tuning to today's
fourteen plants on top of a change whose whole argument is that it needs no
plant-specific evidence.

#### The gate: the shut valve is the wrong place to look, for the second time

The obvious gate is the dead leg behind the shut valve, and it does not work.
With the valve shut, that branch's accepted flow is `1.6e-9` kg/s — three orders
inside `assert_dead_leg`'s bound before the change and after it — so no bar drawn
there discriminates. This is M9.1's lesson arriving again on the same fixture:
**a shut branch is the EASY case.**

What discriminates is the valve node while the valve is still CONDUCTING, and the
observable there needs no tolerance of its own. `drain_line` feeds the valve node
and `rundown_line` leaves it; a `Valve` holds no volume, so the two edges must
carry the same flow, and **the difference between them IS that node's mass
residual**. The solver's own promise is therefore the bar, read off the solver
rather than restated. Walking the valve down in twenty steps, worst ratio of miss
to promise over the walk:

| | before M9.2 | after |
|---|---|---|
| newton | **1.81** (step 4) | 0.97 (step 9) |
| simple | **2.58** (step 6) | 0.39 (step 7) |

Both fidelities fail on the old rule and pass on the new. At its worst step the
game fidelity misses by `2.06e-6` kg/s across a node carrying `0.787` kg/s —
accepted only because the tank feed elsewhere in the plant is larger. **A ratio of
0.97 in the "after" column is a pass, not a near miss**: the bar is the promise
rather than a chosen constant, and a ratio near 1 says the criterion BINDS at that
node, so the gate watches a live constraint (M7.4c's "reachable is not binding").

**The candidate gate this slice set out to write was a vessel, and measuring
killed it.** The plan was to watch a capacitive vessel rather than a valve, on the
reasoning that M9.1's frozen-residual mutation was caught only by M5-era
cross-fidelity tests. But a vessel node is precisely where fork 2 decides NOT to
tighten: its accumulation term is excluded from the scale by argument, so a vessel
gate would have been asserting against the one quantity the criterion deliberately
does not grade. The two vessel plants move by at most `8e-8` relative. The right
place to look turned out to be a valve node, and to be the identity across it
rather than the flow through it.

**Two assertions elsewhere were tightened the same way, and only ONE of them is a
consistency edit — which was got wrong first.** Both sites in `tests/invariants.rs`
grade a dead-end node whose single live edge makes its residual exactly that
edge's flow, so the bar derives to `tol_abs/(1 − tol_rel)` rather than being
chosen. The seed-open/converged-shut half really is inert: its accepted flow is
`0.0` exactly on Newton and `2.31e-10` on Simple, **identical before and after the
change**. The seed-shut/converged-open half is not: under the pre-M9.2 rule its
leg carries `5.97e-8` kg/s against the `1.00e-8` its own node promises, six times
over, and the mutation pass fires it.

**The error was predicting both from measuring one.** Two assertions with the same
shape, on two plants, are two measurements — and the second plant's larger trunk
is exactly what buys the extra slack. So M9.2 has two gates rather than one, and
the second was found by mutating rather than by the reasoning that wrote it.

#### The mutation pass: three edits, and the fork nothing defends

Each mutation restored from one snapshot taken before the pass, with the mtime
moved, and each was checked to have actually compiled (M5.4's rules).

**1. The pre-M9.2 rule, re-expressed inside the new shape** — grade every node
against the largest scale in the plant rather than its own. **Caught, twice**:
`a_valve_node_balances_against_its_own_flow_not_the_plants` at 1.81× on the Newton
half, and `a_leg_behind_a_relief_that_opens_reports_its_neighbours_pressure` at
`5.97e-8` kg/s against a `1.00e-8` promise. The second is the one that was
predicted inert, and finding it is what corrected the write-up above.

**2. Only the last node graded** — `converged = …` in place of
`converged = converged && …`. **Caught broadly**: 12 tests across four binaries,
including both fidelity-agreement gates, the M1 acceptance run's closed-plant mass
balance, and the punctured-line leak reference. Nothing subtle to record; a
stopping rule that reads one node is a different program.

**3. `Σ` in place of `max` for the local scale — UNCAUGHT.** Every test in
`refinery-solvers` and `refinery-scenarios` passes with fork 1's rejected
alternative in place. **This is the honest state of fork 1: it rests on the
inequality `max_incident ≤ throughput` — an argument that the new rule is never
looser than the old — and nothing in the suite defends the choice between `max`
and `Σ`.**

The mechanism, because "uncaught" without one is not a finding. `Σ = 2·max` at any
node with two live edges, which the valve node in gate 1 is, so the mutation
doubles that node's bar and the worst ratio falls from 0.97 to about 0.48. The
gate restates `max` independently rather than reading the solver's scale, so it
*could* fire — but only if the solve actually spent the extra slack, and it does
not: Newton and the sweep both overshoot the looser requirement on the next
iterate anyway. Everywhere else the two readings differ by less than the margin
any endpoint gate leaves.

Closing it would need a gate on a multi-edge node whose solve stops exactly at the
bar — which is a fixture built to sit on a tolerance, and the project has called
that a fitted test before. It is left open deliberately, and it is the reason
fork 1 is argued from an inequality rather than from a measurement: **the
inequality is the only thing holding that fork up.**

#### Blast radius and cost, measured

Twelve of the fourteen shipped scenarios run byte-identical over 6 000 ticks. The
two that move are exactly the two the corpus probe predicted — `relief_blowdown`
and `tank_level_control` — and they move on physical quantities by at most
`8e-8` relative (worst: an edge dissipation). `tank_level_control`'s worst
accepted imbalance tightens from `1.42e-7` to `1.07e-7` kg/s, which is the
intended direction. **From here, "runs byte-identical" means post-M9.2 identical
for those two**, on top of the post-M9.0 baseline for `fcc_plant`,
`knockout_drum`, `leaking_line` and `tank_level_control`.

The cost is one iteration on one plant. Worst iterations per tick over 6 000 ticks
move on `tank_level_control` alone, 3 → 4; every other scenario is unchanged, and
`relief_blowdown` still peaks at the 920 sweeps M9.1 recorded. **The fear that a
tighter bar would push randomly generated plants into `Err` was measured and did
not happen**: the reachability harnesses in `tests/invariants.rs` report identical
convergence counts either side of the change — chains 238/300, gas 202/205 Newton
and 187/205 Simple, psv chains 196/400, gas trees 200/400. That mattered because
proptest generates spurs and dead legs, which is exactly the population whose bar
collapsed to `tol_abs`, and fourteen curated plants moving by `8e-8` says nothing
about it.

#### Deferred, with what un-defers each

- **`tol_rel` per fidelity is still two unrelated constants.** `1e-8` and `1e-6`
  were chosen when they multiplied a plant-wide throughput; they now multiply a
  node's own traffic, which is a different quantity, and neither has been re-swept
  against the new meaning. Un-defers when a plant needs a tolerance argued from
  its own numbers rather than inherited.
- **A node with exactly one live edge can only ever satisfy `tol_abs`.** The
  relative term is multiplied by a scale that its own residual bounds, so
  `tol_rel` is inert there and the absolute floor does all the work. That is
  correct for a dead end and would be wrong for a node whose single edge carries
  real flow — a terminal consumer, say. No shipped plant has one; a scenario that
  adds one un-defers this.

## 12. The second controlled variable — pressure (M10) — specified before building

### What M10 is, and why it is a milestone rather than a ledger row

M9 closed with nothing in `docs/DEFERRED.md` past its trigger, so the next
milestone is chosen from that table rather than handed over as a defect. E1 is
the nearest row: §10 fork 1's loop shape is variable-agnostic, and each variable
un-defers on its own once it has a measurement path and an actuator that exists.

**The milestone is "the second controlled variable", and pressure is the first
one it builds.** That framing is argued rather than assumed, because the obvious
alternative — call it "pressure control", one row, one slice — would make
temperature and flow two more unrelated rows later, each re-deriving the same
questions about where a measurement comes from and what bounds a setpoint. The
whole content of this note is machinery that the *second* variable pays for and
the third and fourth inherit: a `MeasuredVariable` with more than one arm, a
`ControlledValue` with more than one variant, a setpoint key chosen by the
variable, and the two cross-variable refusals that only become expressible once
two variables exist. M8 built the seam; M10 is what proves the seam was
variable-agnostic, and it can only prove that by adding a variable to it.

What M10 does **not** commit to is temperature and flow. Whether they follow in
this milestone is a decision for after pressure is measured, in M9's habit of
scoping one slice at a time — and both look worse than pressure on exactly the
criterion below: a temperature is a `NodeStates` quantity and genuinely solved,
and a flow lives on an edge, which nothing in `measure`'s signature can name.

### The premise E1 states is false, and this is its third recurrence

§10 fork 3 says where a measurement comes from differs by variable, and splits
the world with `heat_input_w`'s own words — a *stored* quantity versus a *solved*
one. It then says:

> A **pressure or a temperature is solved**, and lives in `last_solution` /
> `NodeStates`, both of which are empty before the first tick. A loop on either
> has no measurement at tick 0 and needs a stated rule for that tick when those
> variables un-defer.

**That is false for the one node kind E1 names.** A `Vessel`'s pressure is
`VesselState::pressure(slate) = m/C`, and `m` lives on the graph. It is real from
load and it is *exactly* the declared figure: `build.rs` computes the initial
mass as `P_declared · capacitance(slate)` through the same `capacitance` method
`pressure` divides by, precisely so that round trip is exact. So a vessel-pressure
loop reads the graph, has a genuine measurement at tick 0, and needs no stated
rule for that tick. `measure`'s signature does not change and fork 3's promised
tick-0 rule is not owed.

The claim is true of a **junction's** pressure, which exists only in
`last_solution` — and that is where the deferral survives, scoped to the node
kinds it is actually true of rather than to the variable.

**Third time.** §3a fork 4 specified a smoothstep across the choke on the premise
that a clamp is a kink; the premise was false and the machinery was dropped. §10
fork 3 itself corrected M8.1's "tick 0 has no previous state" for a *level*, and
recorded the generalisation — before arguing from a quantity's absence, check
whether it is stored or solved. It then made the same error in the next
paragraph, about the variable it was deferring. A premise checked for the case in
front of you is not checked for the case you are deferring, and a deferral's
stated reason is exactly the sentence nobody re-reads.

### The ledger's distance for E1 is wrong: the plant has no actuator

E1's distance column says `relief_blowdown` "already has the vessel and the
valve, and lacks only the measurement variant". It has four nodes — a source, the
vessel, a `relief_valve` and a sink — and **no ordinary valve at all**. The PSV
is refused as an actuator by name and for its own reason (its opening is a
memoryless function of its own inlet pressure, so a controller writing it would
be overwritten inside the same tick). The plant lacks an actuator, not a variant.

Adding one would also move a regression anchor: thirteen of the fourteen shipped
files were written before M8 and *are* the anchor, which is why M8.4 shipped a
new file rather than adding a loop to `tank_pump_valve.toml`. M10's demo is a new
file for the same reason. The row is corrected in the ledger.

### Fork 1 — which node kinds can answer for a pressure

`measure` is the single owner of where a measurement comes from, and its
`(variable, kind)` match is where this is settled. Four answers, and the three
refusals each carry their own reason because each is a different mistake:

- **`Vessel` — yes.** Stored, exact from load, per above.
- **`Tank` — refused, and the reason is measured.** A tank's pressure is
  `P_atm + ρ·g·h = P_atm + m·g/A` (M8.5): the density cancels exactly, so a tank
  pressure loop is a *level loop in a worse unit*, with a setpoint the author
  would have to convert by hand and a gain in the wrong reciprocal unit. The
  refusal names `variable = "level"` as the thing that was meant.
- **`Junction` — refused, and this is where fork 3's claim is true.** A junction
  holds nothing; its pressure is an unknown of the solve and does not exist
  before the first tick. Refused with the tick-0 rule named as the missing piece,
  so the refusal doubles as the deferral's own trigger.
- **Everything else — refused as a catch-all.** A source's and a sink's pressures
  are pinned by declaration, which makes them boundary conditions rather than
  states; regulating one is regulating the scenario file. The catch-all says so.

**Rejected: a `pressure()` method on `Node`, dispatching internally.** It would
make "which nodes can be pressure-controlled" a property of a helper rather than
of `measure`'s match, and the three refusals above are three different sentences
that a single `Option<Pascal>` cannot carry.

### Fork 2 — what bounds a pressure setpoint

`check_setpoint` bounds a level by the measured tank's own geometry — a setpoint
above the tank's height is unreachable, so the loop sits pinned at saturation and
reads as a tuning problem. **A vessel has no geometric analogue.**
`P = m·R·T/(V·M̄)` is unbounded above; there is no height to exceed.

The reflex is to reconstruct reachability from the plant — refuse a setpoint
above every source pressure or below every sink pressure. `check_setpoint` takes
`&self` on the graph, so it is *possible*. **Rejected.** It is a network
traversal masquerading as a range check, it is wrong the moment a plant has a
compressor or a second source, and it would refuse legitimate plants for a reason
the author cannot act on. The level arm's bound is cheap because a tank's height
is a declared number on the measured node itself; nothing on a vessel plays that
role.

**Chosen: finite and strictly positive, with a comment saying why the level's
bound has no analogue.** A pressure setpoint of zero is not "drain it" the way a
level setpoint of zero is — it is a vacuum the ideal-gas relation cannot reach at
finite mass — so `0` is refused here where it is legal there, and the asymmetry
is stated at both arms.

### Fork 3 — the key's unit, and the schema's own prediction is wrong

`ControlDef::setpoint_m` predicts, in a comment written at M8.2: "when pressure
control un-defers, its key is `setpoint_pa`". **Follow the format instead of the
prediction.** Every pressure a scenario file declares is in bar — `pressure_bar`
on source, sink, vessel and column, `set_pressure_bar` and `accumulation_bar` on
the PSV, six pressure keys across four node kinds and no `_pa` anywhere —
converted once at the loader by `bar_to_pa`. A `setpoint_pa` would be the only pressure in the format not in bar,
which is the failure fork 4 exists to prevent (a number whose unit a reader has
to infer from its neighbours), inverted. **The keys are `setpoint_bar` and
`gain_per_bar`**, and this note corrects its own prediction the way M8.2
corrected the note's bare `gain` to `gain_per_m`.

**The trap that comes with it, named here because it is silent.**
`ControlledValue::magnitude` returns SI, so a controller's arithmetic is in
Pascals: the error is in Pa and the gain must therefore be *per Pa*. Both the
setpoint and the gain need converting at load, at the same site, and converting
one and not the other is a factor of 100 000 that no type catches — the gain is a
bare `f64` all the way into `ProportionalController::new`. The two conversions
are written as one pair with a comment naming the other, and the gate below
measures the loop's output against a hand-computed `K·e` rather than trusting
either.

A consequence taken deliberately: the scenario key and the snapshot field carry
different units for this variable (`setpoint_bar` in,
`{"variable":"pressure","pa":…}` out) where the level loop's agree. That matches
the rest of the format (`pressure_bar` in, `pressure_pa` out) rather than the
level loop, and rule 4 holds on both sides — the unit is in the key on the way in
and in the type on the way out.

### Fork 4 — the direction of action, and what a vent valve buys

`ControlledValue::error` is `measurement − setpoint` and the output is
`clamp(K·e + b, 0, 1)`, so **a rising measurement opens the actuator**. M8.4
recorded this as "a level loop must actuate a drain", and read as a rule about
levels it is too narrow: what is forced is that **the actuator must be an outlet
of the measured holdup**. A drain is the level case; for a vessel it is a vent.

So the loop this milestone builds vents the vessel — pressure rises, the vent
opens, the vessel blows down toward its setpoint — and the direction question
never arises. The alternative wiring, throttling the *make-up* into the vessel,
is reverse acting and is **not expressible today**: a negative gain is refused at
load in both controllers, deliberately, with the note that a reverse-acting loop
needs its own declaration rather than a sign. **M10 does not build reverse
action.** It is a separate row with its own trigger, and building it beside the
first pressure loop would blur which of the two the demo's numbers are evidence
for.

What this milestone *does* owe is generalising M8.4's sentence in the code that
carries it, since "drain" is now one of two words for the same constraint.

### Fork 5 — the demo plant, and whether it carries a PSV

A new file, for the anchor reason above, and meant to be diffed against
`relief_blowdown.toml` the way `tank_level_control.toml` is meant to be diffed
against `tank_pump_valve.toml`: the same receiver on the same make-up line, with
the PSV replaced by a controlled vent. The pair then says exactly what regulation
is — one plant is held by a spring, the other by a loop.

Three design inputs for the file, each of which will otherwise be discovered:

- **The vent valve is in gas service, so it must declare `x_t`.** Checked against
  `require_gas_valve_x_t` rather than copied from `relief_blowdown`: the rule
  reads the phase M5.2's topological analysis assigns to that NODE, and both
  valve kinds share it because both reach the same compressible law. A gas-only
  slate makes every node gas, so the vent is in gas service by the plant's
  composition and not by anyone's choice, and the loader refuses the file on the
  first load without the key. `0.72` with `gas_valve.toml`'s citation, as
  `relief_blowdown` already carries. The mirror direction bites too: the key is
  refused on a liquid valve, so this is a design input the file cannot dodge by
  omitting.
- **The valve must sit interior at steady state**, off both limits, or the
  milestone repeats M8.4's recorded coverage gap — that demo's output never left
  `[0.194, 0.384]`, so the anti-windup arm was never reached by a wired run.
  Sizing the vent so the settled opening is mid-range is a design input like the
  cascade's saturated-liquid feed, not something to tune afterwards.
- **No PSV on the demo plant.** A normally-shut PSV leaves its valve node a dead
  end, which is the shape that puts `relief_blowdown` at 920 sweeps of 5 000 on
  the game fidelity and is A3's whole subject; a second plant of that shape would
  move a deferral's distance as a side effect of a milestone about something
  else. The controlled vent conducts at steady state, so its node is not a dead
  end. **Predicted, not assumed** — the corpus is run on both fidelities and the
  new plant's sweep count recorded either way.

### Fork 6 — the two cross-variable refusals become reachable for the first time

`setpoint_m` on a pressure loop, and `setpoint_bar` on a level loop. Neither is
covered today: `deny_unknown_fields` refuses a key that belongs to no algorithm
and no variable, and until now the *other* variable's key did not exist, so
"belongs to another variable" was not a state the format could reach. By the
project's own rule — a refusal of something the format cannot express is not a
refusal — these are new work rather than existing coverage, and they are two
refusals, not one, because they are two different files with two different
mistakes in them.

The same becomes true of `gain_per_m` versus `gain_per_bar`, and of the setpoint
variable a `Command::SetSetpoint` carries: `ControlledValue`'s own doc records
that with one variant "the setpoint's variable disagrees with the loop's" is
unrepresentable and there is deliberately no guard, and that **the moment a
second variant lands that refusal becomes required**. M10 is that moment. The
comment names its own expiry; this slice pays it.

### Fork 9 — the boil-off is a FIDELITY KEY, not a behaviour

Raised by the user on 2026-09-06, after the note above was written and before any
code: *some physics models should be toggleable and swappable, especially where
there is a big difference in CPU cost or where they contradict reality in
different ways and neither is provably better.* That is rule 2, and it applies to
this term more sharply than to anything the project has put behind a key so far.

- **(a) unconditional** — every plant whose thermo model can answer gets the
  term.
- **(b) a fifth `[fidelity]` key**, `boiloff = "none" | "flash"`, selecting an
  implementation the way `reactions` selects a `ReactionModel`.
- **(c) a per-node key** on the tank that boils.

**Verdict: (b).**

**The two models are wrong in different ways and neither is a refinement of the
other, which is the actual reason for the key.** `"none"` says a product tank is
a liquid store whose contents never boil however hot the column runs — wrong,
and wrong *conservatively*, in that it moves no mass and invents no stream.
`"flash"` says a quarter of the naphtha product leaves as vapour at `y = K·x`
through a vent to `Atmosphere` with nothing downstream of it — also wrong,
because a real unit condenses that vapour and recovers it, which is exactly what
B12 and B13 defer. A plant is not more correct for choosing either one; it is
making a different statement about what is being modelled. That is a fidelity
choice in this project's sense, not a bug fix with a flag on it.

**It also resolves the question M12.0 left open, and resolves it without weakening
the physics.** The note asked whether ≈24% of a naphtha product boiling away was
an acceptable movement of a pre-M8 regression anchor. Under (b) the anchor
declares `"none"` and does not move at all, the term is exercised in full on a
one-line twin of it (fork 8), and the corpus carries both answers side by side.
The fallback the note named — a quieter term — is not taken, and must not be:
tuning a model down to protect a baseline is how a physics engine acquires a
constant nobody can justify.

**(c) is rejected.** A phase behaviour is a property of the *model*, not of one
vessel; per-node selection would let a single plant carry two thermodynamics with
no argument for which node gets which. Every existing per-node key is geometry,
duty or a setpoint — never model selection — and the one per-instance seam in
the workspace, `Controller`, was argued as an exception in §10 rather than
inherited. Nothing here asks for a second exception.

**Four traps, one from each key that already exists, every one of them something
that actually went wrong.** They are listed here because a fifth key written
without reading `schema.rs`'s own comments will step in at least one:

1. **Default `"none"`, argued the M5.2 way.** Not "so old files still parse" but
   *that is what a pre-M12 file means*. The distinction is the one `separation`'s
   default already draws, and it is forced here anyway: fourteen of sixteen
   plants declare `thermo = "constant"`, which cannot answer.
2. **Refuse `boiloff = "flash"` with `thermo = "constant"` at load**, at the same
   site and in the same shape as the existing `separation = "cascade"` +
   `thermo = "constant"` refusal in `build_engine`. Without it the pairing loads
   happily and fails at tick 1 with a solver error naming a K-value, which is
   §9's "an error must name what the author can fix" failing.
3. **Refuse an unknown value, and write the test that exercises the refusal.**
   `thermo` was parsed and then *ignored* from M1 to M7.2, so `thermo =
   "nonsense"` loaded a working plant — and it was found by wiring a neighbour
   key beside it, not by a test, because nothing reached the value so nothing
   could fail on it. A key whose value is read only when some other key is set is
   born in exactly that state.
4. **The key must change a number on a plant that ships, on the day it lands.**
   `trouton` was held back for a whole milestone rather than shipped as a knob
   nothing could discriminate, citing `smearing_k` — set in every demo file,
   changing no number — as the anti-pattern. This key clears the bar the day it
   lands: 0% against ~24% of a product stream, on two files differing by one
   line.

### Fork 9's other half: what does NOT become a key, and why

The user's criterion has two clauses — a large CPU difference, *or* two models
wrong in different ways with no clear winner. **This term qualifies under the
second clause and not the first, and measuring that is what keeps the argument
honest.**

**Measured, because it is the user's own criterion and it was a guess.** A
bubble-point root find on the shipped five-cut slate, through `TroutonThermo`,
timed over 200 000 solves per composition in a release build:

| tank composition | `T_bub` | ns per solve |
|---|---|---:|
| naphtha (0.5299 / 0.4685 / 0.0016) | 368.21 K | **2 559** |
| distillate | 503.84 K | **3 034** |
| bottoms | 607.73 K | **2 381** |

**Against the budget that matters, this is nothing: ~0.016% of a 60 Hz frame per
boiling tank** (A1's yardstick is the 16.7 ms `_physics_process` call the Godot
binding ticks from). Against the plant's own tick it is not nothing — measured
in the same session, `crude_column_cascade` costs 355.9 ms per 6 000 ticks, i.e.
**59.3 µs per tick**, so three boiling tanks would add ~13%; on a cheap plant
like `cooler_chiller` (45.3 ms, **7.6 µs per tick**) a single tank is ~34%.

**Those two percentages are not in tension, and stating the ratio once is what
stops this being reopened.** 34% of `cooler_chiller`'s tick is 2.6 µs. It is a
large share of a small number, and the number it has to fit inside is 16.7 ms.
A game plant is far more likely to be a handful of tanks than a cascade, so the
34% is the figure a future reader will reach for — and thirty tanks on one plant
would still be 78 µs, half a percent of a frame.

**So the CPU clause does not carry this key and the disagreement clause does**,
and getting that the wrong way round would have mattered. "A root find per tank
per tick" *sounds* like the expensive-model half of the user's criterion; it is
two and a half microseconds. A key defended on a cost of 0.016% of the budget is
a key defended on nothing, and the moment the term were made cheaper the argument
for the key would evaporate — whereas the argument that actually holds it up
(the two models make different claims about the plant, and B12/B13 say the more
expensive one is *also* incomplete) does not depend on speed at all.

The same criterion, applied to the other candidates in the engine, says *not
yet* rather than *no*: a real-gas equation of state and a temperature-dependent
`cp` are both genuinely "wrong in a different way", both are absent as seams, and
**neither has a shipped plant whose numbers would differ.** Adding a key nothing
can tell apart is `smearing_k` again, so they go to the ledger with "a plant that
discriminates the two answers" as the trigger (`docs/DEFERRED.md` B14 and B15)
rather than into this milestone.

### The gates, named before building, and the vacuity each one closes

1. **The tick-0 measurement.** At load, before any tick, the loop's reported
   measurement is the vessel's declared pressure and the *snapshot's* pressure
   for the same node is `NaN` — the solve has not run. **The two sides are
   independent**: one is `ControlSnapshot::measurement`, read from the graph's
   stored mass; the other is `NodeSnapshot::pressure_pa`, read from
   `last_solution`, which is `None`. This is the gate that asserts the finding
   fork 3 got wrong, and it is worth stating that the *equality* half alone would
   be near-tautological — mass is built from pressure through the same
   capacitance `pressure` divides by, so it is a round trip and could only catch
   an asymmetric fault (M7.2's rule). The `NaN` half is what discriminates.
2. **The one-Euler-step offset, read during the transient.** While the vessel is
   still moving, the loop's measurement is the *start*-of-tick pressure and the
   snapshot's is the solved end-of-tick one; they differ by one step, exactly as
   M8.5 measured for a tank (0.67 Pa, 8.6e-6 relative). **The gate must be read
   during the transient**, because at steady state the difference vanishes and
   the assertion is vacuous — a reachability requirement on the fixture, stated
   before it is written.
3. **The unit pairing.** The loop's first output is compared against a
   hand-computed `K·e` with the error in Pascals, which fails if the setpoint is
   converted and the gain is not, or vice versa. A round trip through the loader
   would not.
4. **P versus PI transfers unchanged from M8.3** — the P loop's steady-state
   offset equals its own actuator travel over the gain, which a high-gain P loop
   cannot fake. Nothing about it is level-specific; running it on the pressure
   plant is what shows that.
5. **The refusal sweep**: each of fork 1's three refusals, fork 2's two bounds,
   and fork 6's two cross-variable keys, each asserted on its own message rather
   than on "load fails".

Two controls asserted first, in M9.3b's habit: the vessel's pressure must
actually move during the run (otherwise every gate above is passed by a plant
that sits still), and the vent valve's opening must leave its initial value
(otherwise they are passed by a loop that writes nothing).

### What must not change, stated as a prediction that can be wrong

**Adding a `ControlledValue` variant leaves every shipped scenario
byte-identical, `tank_level_control.toml` included.** The enum is
`#[serde(tag = "variable")]`, so an unused variant contributes nothing to the
wire form of the used one, and no pre-M10 file can select the new variable. The
prediction stops being true the moment anyone touches the serde representation of
the existing variant — which is exactly the edit this states in advance so it
cannot be made quietly. Checked with `corpus --baseline` on both fidelities, not
argued.

### The mutations this slice owes, named before building

- **The gain converted, the setpoint not** (and its mirror). Predicted caught by
  gate 3 and by nothing else — the loop would still be stable, just tuned by a
  factor of 100 000, which every convergence and conservation test tolerates.
- **`measure` reads the solved pressure instead of the stored one.** Predicted
  caught by gate 1's `NaN` half at tick 0 and by gate 2 during the transient.
- **`Tank` accepted for `variable = "pressure"`.** Predicted caught by the
  refusal sweep only, which is the point: nothing physical goes wrong, the loop
  merely controls a level in Pascals.
- **The vent wired to the make-up line instead** (reverse action by rewiring
  rather than by sign). Predicted caught by the control that the pressure moves
  toward setpoint, and predicted *not* caught by any convergence gate.
- **The second `ControlledValue` variant given the same serde tag.** Predicted
  caught by the byte-identity prediction above.

As always the predictions are the point of writing them down, and this project's
record is that roughly half of them are wrong.

### Deferred, with what un-defers each

- **Reverse action** — a loop whose actuator is an inlet of the measured holdup.
  Needs its own declaration, never a negative gain. Un-defers with a plant whose
  only actuator is upstream of what it measures.
- **Junction pressure control**, and with it fork 3's genuine tick-0 rule. A
  junction's pressure is solved and absent before the first tick; the refusal in
  fork 1 names this as the missing piece.
- **Temperature and flow.** A temperature is a `NodeStates` quantity and really
  is solved; a flow lives on an edge, and `measure` names a node. Each needs its
  own measurement path argued, which is the shape this note has now walked once.
- **Actuators other than a valve opening.** Pump speed and duty are still refused
  by name, unchanged from M8.

### Corrections from building it (M10.1, landed)

Nine things the note got wrong or left unsaid, in the order they bit.

#### The one prediction that held exactly: `run_control_loops` is unchanged

M8.2 built the control seam with one variable in it and claimed it was
variable-agnostic. **It was.** The tick pass already read

```rust
self.graph.measure(&self.slate, control.measurement_node, control.setpoint.variable())
```

and that line reads a vessel's pressure without knowing it did anything new. Not
one line of `Engine::run_control_loops` changed in this milestone, and neither
did `Controller`, `ProportionalController`, `PiController`, `ControlLoop`,
`ControlMode`, `ControlSnapshot` or `Snapshot`. The whole of M10.1 is two new
enum arms, four new match arms, two scenario keys and a demo.

That is worth stating plainly because this project's record with predictions is
that about half are wrong, and the temptation on the other half is to say nothing.
The seam held; the thing that did NOT hold is one level down, in the type the seam
carries.

#### `ControlledValue::error` became unsound, and fork 6 did not name it

Fork 6 lists the refusals a second variant makes reachable: the two cross-variable
scenario keys, and the `Command::SetSetpoint` guard `ControlledValue`'s own doc
promised. It does not name the method that actually does the arithmetic:

```rust
pub fn error(measurement: Self, setpoint: Self) -> f64 {
    measurement.magnitude() - setpoint.magnitude()
}
```

whose doc ended "both arguments are the same type by construction, so a level
measurement cannot be differenced against a pressure setpoint". **That sentence
was a property of there being one variant, not of the type**, and with two,
`error(Pressure { pa: 5e5 }, Level { m: 4.0 })` returns a plausible `499996.0`
— metres subtracted from Pascals, silently.

The fix is a `NaN` on a mismatch, which the engine's existing
`!output.is_finite()` check turns into a diagnosed `SimError::Numerical` naming
the loop, one call later. But the more useful part is what it revealed about the
`SetSetpoint` guard: that guard is not a cosmetic refusal about faceplates. It and
the tick pass's `setpoint.variable()` are **the two things that keep `error`
sound**, and the `NaN` is only the backstop behind them. Fork 6 asked for the
guard as a courtesy to the reader; it is load-bearing.

The general form, and it is the third time this project has hit it: **a safety
argument that rests on a type having one inhabitant expires when the type gets a
second one, and it expires silently, because the code does not change.** M8.2's
own doc caught this for the command surface and named its expiry. Nobody wrote the
same note over `error`, four lines below.

#### The gain resolution had to be hoisted, and the note's "one site" is why

Fork 3's whole defence against the factor of 100 000 is that the setpoint's `×1e5`
and the gain's `÷1e5` are written as one pair a reader sees together. `build_controls`
fetched the gain **inside each algorithm arm** — once under `"p"` and once under
`"pi"` — so following the existing shape would have put the conversion at two
sites and reopened the trap for whoever adds the third algorithm. Both are now
resolved above `match def.algorithm.as_str()`, and the arms consume the result.

This is a case where the safe edit was the one that changed MORE code than the
feature needed. The minimal diff was two more `require_keyed` calls.

#### Fork 5's reason for expecting a cheap plant is FALSE, and it corrects A3

Fork 5 predicted the demo would avoid `relief_blowdown`'s 920 game-fidelity sweeps
because "the controlled vent conducts at steady state, so its node is not a dead
end". The vent does conduct — 0.4987 kg/s at the settled state, a real flow, no
dead end anywhere in the plant. **The first draft of the file, with a 2 m × 0.10 m
vent line, took 741 sweeps.**

So dead-endedness is not what drives that cost. Measured, on the same plant with
only the vent line's geometry changed:

| vent line | worst sweeps, game fidelity | per-decile worst |
|---|---|---|
| 2 m × 0.10 m | **741** | 741, 298, 297, 286, 269, 249, 228, 206, 184, 161 |
| 5 m × 0.06 m (`psv_inlet`'s) | 34 | 34, 14, 14, 13, 13, 12, 11, 10, 9, 8 |
| 10 m × 0.05 m (shipped) | **13** | 13, 6, 6, 6, 5, 5, 4, 4, 4, 3 |

The mechanism is the one `psv_inlet`'s own comment describes — the vessel's
Gauss–Seidel diagonal dominated by a fat branch, so each sweep moves the vessel by
almost nothing — but **the branch does not have to be carrying nothing** for it to
happen. `relief_blowdown`'s comment says "a branch carrying no net flow"; the flow
is incidental and the conductance is the whole of it. Ledger row A3 states the
dead end as the mechanism and is corrected.

Two further readings, because the profile above is the interesting part. The
sweep count **falls monotonically as the vent opens** (741 → 161 across the run,
while the opening goes 0.30 → 0.5582), so it is not monotone in total path
conductance: narrowing the pipe helps AND opening the valve helps. And the
geometry that shipped was **not chosen to fix this**. It was chosen on gas
velocity — 0.4987 kg/s at 12.214 kg/m³ through 0.05 m is 20.8 m/s, an ordinary gas
line, where the placeholder 0.10 m gave 5.2 m/s and was simply oversized. The
sweep count is the consequence, recorded; had the two disagreed, the velocity
would still have won and the cost would have been reported.

#### Gate 2's "one Euler step apart" is too simple on a vessel, and the first draft failed

M8.5 measured a tank's snapshot pressure and its mass one Euler step apart, 0.67
Pa and 8.6e-6 relative, and gate 2 was specified by carrying that reading across.
Written out, the vessel version looked like an exact identity: the solve closes
`C·(P − Pⁿ)/dt = Σṁ` and the integrator advances the mass with the same flows, so
`mⁿ⁺¹ = C·P_solved` and therefore `mⁿ⁺¹/C = P_solved`.

**The mass half is exact. The pressure half is not, because `C` is itself a
function of a state that moved.** `C = V·M̄/(R·T)`, the receiver heats as it fills,
and `m/C` is re-evaluated at the new temperature. Measured at tick 101: the loop
reads 1 603 637.7305 Pa against the tick-100 solve's 1 602 984.0865 Pa, **653.6441
Pa apart, 4.078e-4 relative** — and `ΔT/T` over that tick is 0.1308 K on 320.8531
K, which is **4.077e-4**. The residual is the temperature term to four figures.

So the gate divides it out. `P/T` is proportional to the mass alone, and the
assertion is that the loop's `P/T` at tick N+1 is the solve's `P/T` at tick N —
which holds to 1.2e-7, against the 1.2e-3 a `measure` reading the solved pressure
would produce. Four orders of separation, where the undivided form had none.

The habit this is an instance of: **an identity carried across from another node
kind is a hypothesis about that kind's state vector.** A tank's capacitance
analogue is its area, which is geometry; a vessel's is a function of temperature,
which is a state. The two look the same in the balance equation and are not.

#### One of the five specified mutations is NOT EXPRESSIBLE

The note asks for "`measure` reads the solved pressure instead of the stored one",
predicted caught by gate 1's `NaN` half. **That edit cannot be written.**
`last_solution` is a private field on `Engine`; `PlantGraph::measure` takes `&self`
on the *graph*, which holds nodes, couplings and control loops and has no path to a
solve at all. The fault gate 1 defends is prevented by the module boundary, not by
the gate.

That is a real result rather than a technicality, and it cuts both ways. Gate 1's
`NaN` half is genuinely weaker than the note claims — it cannot fail for the reason
it was written for. What it still does is pin the *observable*: if anyone ever
threads solved state into `measure` (by widening the signature, which is the only
way in), the tick-0 assertion is what stops it landing quietly. The nearest
expressible substitute was run instead — `measure` returning a `NaN` pressure — and
its result is in the table below.

This is the fourth time in this project a specified gate or mutation turned out to
have no power over its own subject (M7.4b's duty gate, M7.4c's cascade balance,
M9.2's vessel gate, and now this). The common shape: **the note reasons about what
the code should check, and does not check what the code can reach.**

#### The demo's counterfactual came out differently from M8.4's, and the difference is physical

M8.4's parked level loop ran to the tank's roof, and that gate could assert
divergence. A parked pressure loop does not diverge: a vent's flow rises with the
vessel's own pressure, which is a far stiffer feedback than a tank's `ρgh`, so the
manual run **settles** — at 25.197 bar, against the loop's 20.000. So the gate
asserts a wrong equilibrium rather than a runaway: five bar apart, with the auto
run holding its setpoint to six figures.

Worth keeping because the reflex is to reuse the previous demo's assertion. "The
parked plant runs away" is a property of how weakly that particular plant
self-regulates, not of what a control loop is for.

#### The gain bound is two-sided here, and the bound is not the clamp

M8.4's level demo had one gain bound: its drain settled at 0.376 and its setpoint
was only ever stepped up, so only the lower clamp was reachable. This plant's vent
settles **interior at 0.558224**, so a one-bar setpoint step moves the output by
`gain_per_bar` in either direction and there are two bounds — `0.5582` reaches 0 on
a step up, `0.4418` reaches 1 on a step down. The shipped `0.10` has a factor of
four of margin either way, and both neighbours are run, because a margin claim with
only the passing side measured is not a margin claim.

The second correction is smaller and cost a test run. **A gain of exactly the
settled opening lands the output ON zero without engaging the clamp** — measured
`3.4e-10`, not `0.0`, because the loop is still that far from its own fixed point
after 20 000 ticks. The bound and the clamp are two claims: the arithmetic is
asserted at the bound, and the clamp ten percent past it, where it engages exactly.

#### M8.2's unknown-variable test case expired, exactly as its algorithm twin did

`a_control_table_is_refused_where_fork_5_says_it_must_be` used
`variable = "pressure"` as its stand-in for "a variable the loader does not know".
It now names a variable the loader knows and refuses for a measured reason of its
own, so the case moved to `"temperature"` — the same move M8.3 made when `"pi"`
stopped being a good stand-in for an unknown algorithm and the case became `"pid"`.

Two of these in three milestones is a pattern worth naming: **a refusal test
written against "the next thing that does not exist yet" has a shelf life, and the
slice that implements that thing is the one that owes the replacement.** The test
does not fail loudly on its own terms — it failed here only because the new
refusal's message says something different.

### The mutation pass, against the predictions (M10.1, landed)

Eight edits — the five the note named, plus the two guards this slice added, plus
the split of the fifth into the two directions it turned out to be. **Three of the
six predictions were wrong**, which is this project's usual rate.

| # | edit | predicted | measured |
|---|---|---|---|
| 1a | the setpoint's `×1e5` dropped, the gain's `÷1e5` kept | gate 3, and nothing else | **caught by 7**, gate 3 among them |
| 1b | the gain's `÷1e5` dropped, the setpoint's kept | gate 3, and nothing else | **caught by 7**, gate 3 among them |
| 2 | `measure` reads the solved pressure | gate 1's `NaN` half, and gate 2 | **not expressible** — see below. The nearest substitute (a `NaN` measurement) is caught by 12 |
| 3 | `Tank` accepted for `variable = "pressure"` | the refusal sweep only | **caught by exactly 1** — the refusal sweep |
| 4 | the vent rewired as an INLET of the receiver | the demo's control that the pressure moves; no convergence gate | **caught by 4**, all of them demo gates; no convergence or conservation test fired |
| 5a | the NEW variant tagged `"level"` | the byte-identity check | ***UNCAUGHT* by everything**, until this slice added a gate for it |
| 5b | the EXISTING variant tagged `"pressure"` | (the prose, not the table) | caught by the corpus, both fidelities, `tank_level_control` moved |
| 6 | the `SetSetpoint` variable guard dropped | gate 5d | caught by exactly 1 — gate 5d |
| 7 | `error` subtracts across variables again | gate 5d's backstop half | caught by exactly 1 — gate 5d |

**Prediction 1's "and nothing else" was wrong in both directions, and the reason
is worth keeping.** The note argued that a factor of 100 000 in the tuning "would
still be stable, just tuned by a factor of 100 000, which every convergence and
conservation test tolerates". That half is exactly right — no solver, mass or
energy test fires. What the note did not account for is that the demo gates pin
the *settled operating point*, and a loop mistuned by five orders does not settle
where the file's header says. So gate 3 is not the only thing standing between
this project and the trap; it is the only thing that **names** it, which is a
different and smaller claim than the note made.

**Prediction 5 was wrong because a regression anchor cannot see the file the slice
adds.** Split into its two directions, one is caught and one was invisible:

- tagging the **existing** level variant `"pressure"` moves `tank_level_control`
  and the corpus exits nonzero on both fidelities. That is what the note's prose
  describes — "the moment anyone touches the serde representation of the existing
  variant".
- tagging the **new** pressure variant `"level"` moves **nothing**: zero rows on
  both fidelities, and the whole test suite green. The baseline was recorded
  before the slice, so the only plant whose bytes change is the one that is new in
  the same slice and therefore has no baseline row. The demo then reports
  `{"variable":"level","pa":2000000.0}` and a frontend draws a pressure faceplate
  labelled as a level.

The mutation table listed the second and the prose described the first, and they
are not the same edit. **The general rule: a byte-identity baseline protects the
old files and has no power over the new one, so anything a slice ADDS needs an
assertion of its own.** `the_demo_reports_its_variable_on_the_wire_as_pressure`
is that assertion, and it had to be written against the serialized bytes — a Rust
match on `ControlledValue::Pressure { .. }` passes under any tag.

**Predictions 3, 6 and 7 held exactly**, each caught by exactly one test and that
test the one named. 6 and 7 are caught by the *same* test, which is deliberate:
gate 5d asserts the refusal and the `NaN` backstop as two assertions in one place,
because they are two halves of one claim — that nothing subtracts metres from
Pascals — and separating them would suggest either could stand alone.

## 13. The cavitation criterion (M11) — specified before building

### What M11 is, and why its own row's name is wrong

`docs/DEFERRED.md` B1 is called **"no cavitation floor"** and §3 above promises
"a vapor-pressure clamp is a later milestone". This note ships **neither a floor
nor a clamp**. It ships a *criterion* — the model's own answer to "is this liquid
boiling?" — and a *signal* that carries the answer to a frontend. Fork 1 rejects
the clamp on its merits, so the row's noun is corrected in the ledger rather than
honoured in the code.

The distinction is the whole scope boundary. A clamp changes what the plant
*does*; a criterion changes what the engine *says*. What cavitation does to a
pump — head degradation, the vapour that forms and then collapses — is mass in a
phase this state vector does not have, which is ledger row B3 and a milestone of
its own. M11 stops exactly where B3 begins, and the snapshot says so.

### The trigger is not past, and the reason is sharper than "not yet"

B1's distance column says **1.865×**, at `heat_recovery`'s `hx_hot`. That number
is real, reproduces to six figures, and **no engine configuration of that plant
can produce it.** `heat_recovery` declares `thermo = "constant"`, and
`ConstantThermo::k_value` is an `Err` by design — it has no phase equilibrium at
all. The M10 close-out measured the corpus with a *standalone script* that
reimplemented Raoult over Trouton, and the script's instrument is not the
engine's.

**Fourteen of the fifteen shipped plants select `thermo = "constant"`.** The one
that does not is `crude_column_cascade`. So the set of nodes at which *the engine*
can evaluate a bubble pressure today is not the corpus — it is one plant.

Measured, 6 000 ticks, every tick, through the model's own public API
(`TroutonThermo::k_value`, so `Psat = P·K` exactly):

| plant | node | kind | margin `P/P_bub` | engine can evaluate? |
|---|---|---|---|---|
| `crude_column_cascade` | `preheater` | **furnace** | **1.899** (spread 1.899–1.921) | **yes** |
| `crude_column_cascade` | `column` | column | 0.988–1.000 | yes, and meaningless (below) |
| `crude_column_cascade` | `naphtha_tank` | tank | 0.940 | yes, and it is B3's |
| `crude_column` | `preheater` | furnace | 1.258 | no — `constant` |
| `heat_recovery` | `hx_hot` | exchanger | 1.865 | no — `constant` |
| every liquid pump/valve/junction | | | ≥ 21.4 | no — all `constant` |

**Under the trigger's own words — "a pump, valve, junction or exchanger" — the
engine-computable set across all fifteen plants is EMPTY.** Every node of those
four kinds lives on a plant whose thermo model refuses to answer, and the one
plant that can answer has none of those four kinds in it. Its only flow-path node
is a **furnace**, which the trigger does not name.

**This is a fourth variant of a pattern the ledger already tracks.** The M10
close-out corrected B1's distance because it had been measured *against the wrong
quantity* — a pressure against zero, concluding about a vapour pressure. This
correction is different in kind: the quantity is right and the **instrument is one
the engine does not have**. A distance measured with a script says what the
physics would say if the engine could ask; it does not say what the engine can
report. The tell was available in the old row too — it cites `heat_recovery` by
name, and `heat_recovery`'s fidelity line is three lines from the top of its file.

Reading for the ledger: **a distance is a property of the engine, not of the
plant.** If the number cannot be produced by a run, it is a prediction about a
configuration nobody ships.

### The licence this milestone is taken under, stated plainly

Neither trigger clause has fired. Clause (a) — a solved hydraulic-path pressure
below its own bubble point — is not merely unreached, it is *unmeasurable* on
fourteen plants. Clause (b) — a frontend needing to display cavitation — is a
**decision**, and this milestone is that decision being made rather than a
measurement arriving.

Two things make it a decision worth making now, and they are stated so a later
reader can disagree with the reasoning rather than guess it:

1. **§3's frontend contract is actively wrong, and this is the only ledger row
   whose subject is a wrong number a frontend is asked to interpret.** Until the
   M10 close-out, §3 told frontends to read negative absolute pressure as
   cavitating. A frontend that implemented exactly what was written displays
   cavitation late by the fluid's vapour pressure — 0.58 m of suction lift on
   cold water, **nine bar** on light naphtha at 445.75 K. The close-out corrected
   the paragraph to say the engine has no signal; that is honest, and it leaves a
   frontend with nothing.
2. **The thermodynamics has been in the engine since M7.2 and has exactly one
   consumer.** `ThermoModel::k_value` is read by the stage cascade and by nothing
   else, which is why `thermo = "trouton"` on a plant with no column is a knob
   that changes no number — `schema.rs` says so in a comment and deliberately
   does not refuse the pairing. **This milestone is the first consumer that makes
   `thermo` matter on a plant with no column**, and that is measured, not
   asserted: switching `tank_pump_valve` to `"trouton"` leaves its pump-node
   pressure at 175 197.988 Pa, identical to the last digit.

What this note does **not** claim is that a plant asked. None did.

### Fork 1 — a signal, a refusal, or a clamp

Three shapes for "the engine knows this liquid is boiling", and the choice
determines the milestone's size.

**(a) A clamp — floor the node pressure at the bubble pressure inside the solve.
Rejected.** It is what §3 promised and it is wrong at this fidelity. A clamped
node no longer satisfies `Σṁ = 0`: the mass that fails to balance is vapour, and
vapour at a node the state vector calls liquid is exactly B3. The model would
report a converged, conserving solve that conserves nothing — a stronger version
of the failure `well-posed ≠ correct` already names. It is also a kink in the
residual, and §3a fork 4 spent a whole fork establishing that this network keeps
its characteristics C¹.

**(b) An `Err` — refuse the tick. Rejected on two grounds.** Rule 5's `Err` is
for a solve that *failed*; this solve succeeded, and the answer it produced is a
genuine root of the equations it was given. And it converts an operating state a
game must render — an over-driven pump, a suction line someone throttled — into a
dead simulation. §3b's damage doctrine is the precedent: a leak is an edge and a
fire is a heat source, because damage must be *representable*, not fatal.

**(c) A signal — evaluate the criterion and report it. Chosen.** The engine says
what its own thermodynamics says, the hydraulics are untouched, and the frontend
gets the sentence §3 could not give it. The cost is honesty about what is not
modelled: a cavitating pump in M11 still delivers its full head, so the snapshot
says the plant is boiling while the flow says it is not. That is a *known*
disagreement with a name and a ledger row, which is better than the current
state, where there is no disagreement because there is no signal.

### Fork 2 — what the criterion is, and one property of it worth keeping

`P_node < P_bub(T_node, x_node)`, where `P_bub` is the **mixture bubble
pressure**: the pressure at which the first bubble forms out of a liquid of that
composition at that temperature.

```text
P_bub = Σ_c x_c · Psat_c(T)          Raoult, x MOLE fractions
```

**Rejected: "any single component is above its own vapour pressure"**
(`max_c Psat_c(T) > P`). That fires on the lightest cut alone and would call a
crude boiling whenever its light naphtha would boil neat. A mixture boils when
the *sum* of its partial pressures reaches the ambient pressure; the light cut's
contribution is weighted by how much of it there is. For a pure fluid the two are
the same number, which is why a water-only fixture cannot tell them apart — see
the gates.

**A property worth recording, because the cascade's experience predicts the
opposite: the bubble PRESSURE is explicit, where the bubble TEMPERATURE is a root
find.** `cascade.rs::bubble_point` solves `Σ K_c(T)·x_c = 1` for `T`, and M9.3a
spent a slice getting it from 60 evaluations to 15, with a
`BUBBLE_POINT_MAX_EVALUATIONS` bound and an argument about resolution being a
noise floor on the test that grades it. None of that applies here. `P_bub` is a
weighted sum of closed forms — no iteration, no cap, no tolerance, and no failure
mode other than the ones `k_value` already guards. **A criterion whose cost is
one `exp` per component per node is a criterion that can run every tick**, which
is what fork 6 turns on.

### Fork 3 — where the number comes from: a new trait method

**Chosen: `ThermoModel::bubble_pressure(&self, slate, composition, temperature)
-> Result<Pascal, SimError>`**, a third method on the trait that has carried two
since M7.2.

**Rejected: computing `P·Σ x_c·K_c(T, P)` in `core` from the existing
`k_value`.** It is algebraically identical *and only for a Raoult-form model*:
the `P` cancels because `K = Psat/P`, which is a property of `TroutonThermo`, not
of the trait. Writing that cancellation into `core` puts a model assumption in
`core` as plainly as a fidelity `if` would (rule 2), and it would be silently
wrong for any future `K` that is not inversely proportional to pressure. It also
needs **mole** fractions, and ledger row A13 records why `MoleFractions` lives in
`solvers` and must not be dragged down into `core`.

The three implementations, and each refusal has its own reason:

- **`TroutonThermo` — the closed form above.** It converts the mass composition
  it is handed to mole fractions at its own boundary, which is §5 fork 1's
  convention: a `Composition` is mass fractions everywhere in this workspace, and
  molar is a model's internal business.
- **`ConstantThermo` — `Err`.** It has no phase equilibrium, exactly as its
  `k_value` does not. Rule 5: never a plausible number.
- **`ConstantAlphaThermo` — `Err`, and the reason is sharper than "it was handed
  its numbers".** Its `K` is *independent of pressure*, so `P·Σ x·K` depends on
  which `P` you evaluate it at: there is no bubble pressure to return, not merely
  none it was told. That sentence belongs in the code, because the obvious
  reading — "it has K-values, so it can answer" — is wrong.

The signature takes a `&Composition` rather than a node or a slate index, so the
method knows nothing about graphs and stays a pure property lookup, the shape
`k_value` and `dh_vap` already have.

### Fork 4 — which node kinds are subject, enumerated

The trigger names four kinds. `NodeKind` has **fourteen** variants, and the
section above measured what inheriting a four-name list would cost: the one node
in the corpus the engine can actually evaluate is of a kind the list omits. So
every variant gets a verdict and a reason.

**Subject to the criterion — the zero-volume hydraulic path:**

- **`Pump`** — the failure mode §3's paragraph is about. Suction-side boiling is
  the canonical case.
- **`Valve`, `ReliefValve`** — flashing across a throttle is the second canonical
  case; the pressure recovers downstream, the vapour does not.
- **`Junction`** — a header or tee; its pressure is a pure unknown of the solve.
- **`HeatExchanger`** — the corpus's tightest script-measured margin (1.865×).
- **`Furnace`, `Cooler` — and these are the ones the trigger's list drops.** A
  fired heater's outlet is precisely where a refiner expects a liquid to boil;
  that is what a heater is for, and vaporising in the *tubes* rather than
  downstream is a real and expensive failure. They are also the only flow-path
  kind the engine can evaluate anywhere in the shipped corpus, so excluding them
  by inheriting a four-name list would leave the criterion with **no reachable
  node at all** — the fifth dead gate this project has written. The trigger is
  corrected in the ledger to name six kinds.

**Excluded, holdups — this is the load-bearing exclusion:**

- **`Tank`, `Vessel`.** A holdup below its bubble point is a **two-phase
  inventory**, which is B3, not cavitation. The distinction is not pedantry: it
  is what stops the two rows claiming each other's evidence, and it is already
  measured — `crude_column_cascade`'s `naphtha_tank` sits at **0.940×** and
  `crude_column`'s at **0.30×**, so a criterion without the exclusion fires on
  two shipped plants and reports them as cavitating pumps.

**Excluded, declared boundaries:**

- **`Source`, `Sink`, `Atmosphere`.** Their pressures are typed into the file,
  not solved. Grading one grades the author's arithmetic, and the old B1 distance
  is the cautionary case: its 100 000 Pa was a declared sink.

**Excluded, and each for its own physics:**

- **`Column`.** A column is **at** its bubble point by definition — that is what
  a column is. Measured over 6 000 ticks: `crude_column_cascade`'s sits between
  **0.988 and 1.000**, straddling the criterion for the whole run. A signal there
  reports the model working.
- **`Reactor`.** It *imposes* its outlet temperature (`t_set`), so the
  temperature the criterion would read is a setpoint rather than a resolved
  state; and the FCC slate declares a `gas` lump with `tb = −40 °C` as a liquid,
  which makes a liquid bubble-point test meaningless there (measured: 0.0029×).
  That second half is B3's, and it is why the exclusion is stated rather than
  left to the phase check below.

**And one cross-cutting exclusion that is not a node kind: a gas composition.**
A vapour does not cavitate; it is already vapour. `Composition::phase` is the
existing owner of that question and answers `Gas`, `Liquid`, or `Err` on a
mixture — so the criterion asks it rather than inventing a second notion of
phase, exactly as M5.2's density dispatch does. Without it, **four plants carry a
gas-phase subject node** — a tee, a control valve, a PSV and a vent valve — and
every one of them reads between 0.012× and 0.017×, so the signal would be noise
on every gas plant in the corpus. A mixed-phase composition is already an `Err`
everywhere else in the engine and stays one here.

**The exclusion has no reachable subject today, and that is stated rather than
discovered later**: all four of those plants declare `thermo = "constant"`, so
the model refuses before the phase check is reached. Its gate therefore needs a
fixture that selects `"trouton"` on a gas slate — which is the shape of a dead
gate, and is why the fixture is named in the mutation list rather than assumed to
fall out of the corpus.

### Fork 5 — what the snapshot carries, and the difference between "no" and "cannot tell"

**Chosen: `NodeSnapshot::cavitation: Option<CavitationSnapshot>`**, skipped when
`None`, carrying both the verdict and the number it was made from:

```rust
pub struct CavitationSnapshot {
    /// Bubble pressure of this node's liquid at its resolved temperature [Pa].
    pub bubble_pressure_pa: f64,
    /// `pressure_pa < bubble_pressure_pa` — the engine SAYING SO.
    pub cavitating: bool,
}
```

**`None` means "there is no criterion here", never "healthy".** That is
`column_duty`'s shape and `column_duty`'s argument: a cut-point column reports no
duties because its fidelity has no such equipment, and a node reports no
cavitation state when it is the wrong kind, when its fluid is a gas, or when the
plant's thermo model cannot answer. Reporting `cavitating: false` in those cases
would be `heat_input_w`'s lesson pointed the other way — not a field nothing
reports, but a field reporting what nothing computed. **Fourteen of fifteen
plants will emit nothing at all**, and a frontend must render that as "unknown",
not as "fine".

**Both fields, not one, and the reason for each.** The `bool` is the deliverable:
clause (b) asks for the engine to say so rather than for a frontend to infer it,
and inference is what §3 got wrong. The number is what makes the verdict
auditable and what a margin gauge draws — a frontend holding only a `bool` cannot
show a plant getting closer. Two fields carrying one relationship is how they
drift, so their agreement is a gate, the way `leak_mass_flow`'s agreement with
its orifice edge is.

**Rejected: a bare margin ratio `P/P_bub`.** It hides a division by a bubble
pressure that can underflow toward zero for a heavy residue — the corpus already
contains one at 213 Pa, giving a ratio of 630 — and it makes the verdict a
comparison against 1.0 that every frontend re-implements, which is the current
situation with a different constant.

**The regression anchor moves by exactly one key on one node.** Only
`crude_column_cascade` selects `trouton`, and only its `preheater` survives fork
4's exclusions, so it is the single node in the corpus that gains a key. The
other fourteen plants are byte-identical for free. Verified M8.5's way — strip
the key from the after-run and reproduce the before-file byte for byte — rather
than predicted.

### Fork 6 — evaluated in the tick, not in the snapshot

`Engine::snapshot` takes `&self` and returns a `Snapshot`, not a `Result`. A
criterion evaluated there would have to swallow the `Err` a thermo model returns,
which rule 5 forbids and which would turn "this model cannot answer" into
"healthy". So the evaluation happens in `tick`, where both halves are in hand —
`solution.node_pressure` from step 1, and the resolved temperature and
composition from step 2b — and the snapshot reports what the tick stored.

**Every tick, not on demand.** A frontend samples snapshots (the CLI writes one
every N ticks; the Godot scene reads one per physics frame), and a criterion
evaluated only when someone looks cannot see a transient between two looks. Fork
2's closed form is what makes this affordable: one `exp` per component per
subject node, against a tick that already runs a Newton network solve and, on the
one plant that can answer, a stage cascade.

**Stored beside `last_solution`, not inside `NodeStates`.** `NodeStates` is
produced by `resolve_node_states` and consumed by the *next* tick's solver as
`previous_states`; it has one owner and one contract. This is a diagnostic over
the *pair* (solution, states), computed after both exist, and feeding it back
into the solver's input would be a coupling nobody asked for. `reactor_duty` is
the counter-precedent worth naming: it lives in `NodeStates` because the sweep is
the only place its inputs meet. Here the sweep is not.

**Nothing in the forward solve reads it.** Same as `column_duty` and
`reactor_duty`: an emergent diagnostic. That is what makes M11 unable to break a
determinism or conservation gate — and also what makes those gates unable to
defend it.

### Fork 7 — the demo plant, and why it is not water

A new file, on M8.4's and M10.1's precedent: the thirteen pre-M8 plants *are* the
regression anchor, and adding a signal to one would move it.

**B1's reachability fixture takes two edits, not the one the row claims.**
Raising `tank_pump_valve`'s suction line crosses the bubble point at 17.44 m —
re-measured here with the engine's own instrument, reproducing the close-out's
table to six figures (17 m → 9 904.66 Pa; 19 m → −9 522.94 Pa, solver converged,
10.0 kg/s flowing). But that plant declares `thermo = "constant"`, so the second
edit is the fidelity line, and the row's "one number in one shipped file" is
wrong in the same way its distance was. **The fidelity edit moves no number**
(measured: 175 197.988 Pa either way), which is what makes it admissible in a
demo at all.

**The demo runs a light hydrocarbon, not water, and this is a design input.**
`TroutonThermo`'s own doc comment says the correlation is poorest for associating
fluids — it overstates water's vapour pressure by about 60% at 50 °C — and is
good to roughly 10% over the reference hydrocarbon's tabulated range. A demo
whose entire subject is a vapour pressure should run on the fluid class the
correlation is honest about. It is also the better demonstration: on cold water
§3's old marker is 0.58 m of lift late; on a light naphtha it is **nine bar**
late, which is more than the operating range of the plant carrying it.

**Two shape requirements, both from measured coverage gaps.** The cavitating node
must sit **interior** to the regime for most of the run, not touch it at the last
tick — M8.4's wired loop never reached its own saturation arm, and M10.1's vent
had to be sized to sit interior at steady state. And the same file must carry a
**healthy** subject node, so the firing and non-firing arms are covered by one
plant and a frontend has something to draw both states from.

### The gates, named before building, and the vacuity each one closes

1. **The anchor: a pure component at its normal boiling point has
   `P_bub = P_ATM` exactly.** `K = 1` at `(tb, P_ATM)` is the identity the
   correlation is integrated from, so this is exact and independent of Trouton's
   constant. It closes an arithmetic error in the Raoult sum. **It is
   structurally incapable of seeing a wrong constant** (§5 correction 4), which
   is why it is not the only gate.
2. **The envelope half reuses `tests/reference/vapour_pressure.rs`.** That file
   already carries a published envelope, run at a deliberately wrong constant. A
   second envelope invented here would be a second thing to keep right.
3. **Mass-versus-mole, on a slate where they differ materially.** The crude slate
   spans molar masses 0.100 to 0.400 kg/mol, so mass and mole fractions are far
   apart; a water-only plant cannot catch the swap at all
   (`degenerate-fixture-disables-the-code-path`). This is M4.2's crux — units,
   not the algorithm — and it is the mutation most likely to produce a plausible
   wrong number.
4. **The holdup exclusion, on the plant already in the state.**
   `crude_column_cascade`'s `naphtha_tank` sits at 0.940× and must report **no
   signal**, while the same run's `preheater` at 1.899× reports one. This is the
   gate defending the clause the ledger calls load-bearing, and it is asserted on
   a plant that is already below its bubble point rather than on a fixture built
   to be.
5. **The late-marker gate — the whole content of §3's correction, asserted.** The
   demo plant, at an operating point inside the band, must report
   `cavitating: true` while its absolute pressure is still **positive**. On the
   reference plant that band is `[17.44 m, 18.02 m]` of lift; on the hydrocarbon
   demo it is bar-wide. Without this the milestone is a refactor of a number
   nobody checks.
6. **Cannot-answer reports `None`, not `false`.** Both refusing models, asserted
   at the snapshot level, because the mistake this prevents is not an `Err`
   escaping — it is an `Err` being turned into a clean bill of health.
7. **The wire form, asserted on the serialized bytes.** M10.1's lesson: a
   byte-identity baseline has no power over the file the slice *adds*, so the
   demo's own signal must be asserted on its JSON. A Rust match on
   `Some(CavitationSnapshot { cavitating: true, .. })` passes under any serde
   tag.

### What must not change, stated as a prediction that can be wrong

- **Fourteen of fifteen plants byte-identical on both fidelities**, and
  `crude_column_cascade` different by exactly one key on exactly one node.
- **No solver iteration count moves anywhere.** The criterion is downstream of
  the solve and feeds nothing back.
- **Wall time on `crude_column_cascade` unchanged within noise**, measured A/B in
  one session with an unrelated plant as a control — that plant is the only one
  with a wall-time history (M9.3a, M9.3b), so it is the one someone will ask
  about.
- **`measure`, `Controller`, every `FlowSolver` and `network.rs` untouched.**
  M10.1's equivalent prediction held exactly and saying so was part of the
  record; this one is broader and correspondingly more likely to be wrong.

### The mutations this slice owes, named before building

1. **Mass fractions where mole are needed** → predicted caught by gate 3, and by
   nothing else.
2. **The node-kind filter widened to include `Tank`** → gate 4.
3. **The node-kind filter narrowed to the trigger's four names** (dropping
   `Furnace`/`Cooler`) → predicted caught by gate 4's `preheater` half alone,
   because that is the only subject node in the corpus.
4. **`bubble_pressure` returning `Ok(0.0)` instead of `Err`** on a model with no
   equilibrium → gate 6; without it, every node of every plant reports healthy
   against a bubble pressure of zero.
5. **The comparison made against `P_ATM`, or against the upstream node's
   pressure, instead of this node's own** → gate 5.
6. **The gas-phase exclusion dropped** → predicted caught only by a **fixture**:
   the four gas plants that would report `cavitating: true` at 0.012–0.017× all
   declare `thermo = "constant"` and refuse before the phase check, so the
   shipped corpus cannot catch this edit. The fixture selects `"trouton"` on a
   gas slate.
7. **`<` widened to `<=`** → predicted **uncaught**, and said in advance: exact
   equality on a float pressure is measure-zero, and no fixture can be built to
   land on it without being fitted to the arithmetic.

### Deferred, with what un-defers each

- **The consequence of cavitation** — head degradation, an NPSH curve, the vapour
  itself. This is B3 plus a pump model, and the snapshot's honest disagreement
  (boiling, yet delivering full head) is what un-defers it: a frontend that must
  show a pump *losing* flow rather than a warning lamp.
- **Latching a transient.** The criterion is evaluated every tick but reported
  only when sampled, so an event between two snapshots is invisible. A latch is
  state and needs a reset command, which is a discrete layer (E6). Un-defers when
  a plant is measured to cavitate between two sampled snapshots.
- **A load-time refusal for "this plant wants the signal and its thermo cannot
  give it".** Deliberately absent: `thermo = "trouton"` on a plant with no column
  is a legal and now-meaningful choice, and its absence is a silent `None` rather
  than an error, because nothing in a scenario file *asks* for cavitation
  reporting. Un-defers if a frontend needs the signal on a plant whose author did
  not choose the model that provides it.
- **The other thirteen plants' fidelity lines.** Switching them to `"trouton"`
  would give the criterion twelve more subject nodes and move no physics — but it
  would move the regression anchor of every one of them, for a signal nothing
  reads. Un-defers with a frontend that reads it.

#### Corrections from building it (M11.1, landed)

The note's seven forks all survived; three of them were **underspecified in a way
that only shows up when something has to compile**, and one gate had to be
rewritten because the form the note gave it could not fail.

##### 1. The note never said what happens to the OTHER kind of error

Fork 3 says a model with no vapour–liquid equilibrium returns an `Err`, and fork
5 says the snapshot reports `None` there. Neither says what the engine does with
an `Err` that means something *else* — a non-finite temperature, a composition
the slate cannot interpret. Swallowing every error would turn a genuine fault
into a silent "no criterion", which is the failure fork 5 spends a paragraph
forbidding one level up.

**The variant carries it.** `ConstantThermo` already refused `k_value` with
`SimError::Scenario`, and `TroutonThermo`'s state guard already used
`Numerical`/`NonFiniteState`, so the distinction existed before this slice needed
it. The tick pass reads `Scenario` as "this configuration cannot answer" and
reports nothing; every other variant propagates and fails the tick. That is
`SimError::AnchoringUnsettled`'s own precedent — a caller that must accept one
outcome needs to tell it apart from an ordinary failure, and a substring of a
human-readable message is not something to gate on.

It has a consequence worth recording: **the test stub in `core`'s own
`energy.rs` was refusing with `Numerical`**, and left alone it would have made a
sweep test fail a tick rather than report nothing. Three of the six
`ThermoModel` implementations in this workspace are test stubs, and adding a
method to a trait means deciding what each of them *means*, not just what makes
them compile.

##### 2. Fork 5 was silent on tick 0, and the tempting answer is the forbidden one

`pressure_pa` is NaN before the first solve and `NodeStates` is empty, so there
is nothing to compare and nothing to evaluate at. The reflex is to report
`Some` with a NaN bubble pressure, matching `pressure_pa`'s convention — and a
NaN bubble pressure is a number no model produced, which is exactly what rule 5
forbids handing out. **`None` before the first tick**, decided and written on the
field rather than discovered. `dissipation_w` reports NaN there and this reports
absence, and the difference is that one of them is a *quantity that exists and
is not yet known* while the other is *a verdict nobody has made*.

##### 3. Gate 4 was specified in a form that cannot fail, and its run length was wrong

The note calls gate 4 "the strongest gate available" because it is asserted on a
plant already in the state, and then specifies the assertion as "the tank must
report **no signal**". Two things were wrong with that.

**The assertion has to be on the `Option`, not the verdict.**
`cavitating == false` is what a *broken* exclusion reports for a tank that is not
boiling — so a gate in that form passes whether the node-kind clause works or
not. Only `is_none()` distinguishes "excluded" from "included and healthy". That
is the `None`-versus-`false` distinction fork 5 argues for a whole paragraph, and
the gate that depends on it did not name it.

**And the plant is not in the state when the note assumed it was.** The M10
close-out reported `crude_column_cascade`'s naphtha tank at 0.940× "hovering on
the line"; measured tick by tick, it is **above** its bubble point for the first
1 826 ticks and only then crosses — 3.99× at tick 1, 1.69× at tick 400, 0.957×
at tick 2500. A gate running 400 ticks would have been asserting an exclusion on
a tank that was not boiling, which is the same vacuity in a second dress. The
gate runs 2 500 ticks and **asserts the control first**: it computes the tank's
bubble pressure itself and refuses to proceed unless the margin is below 1.

The control has to be computed rather than read, and that is a general property
of an exclusion: **an excluded node publishes nothing, so there is no way to show
the exclusion is doing work except to evaluate the criterion independently and
find that it would have fired.**

##### 4. The demo has no holdup anywhere, which fork 7 did not require

Fork 7 asks for a light hydrocarbon, an interior operating point and a healthy
node in the same file. Building it added a fourth constraint that the note should
have derived: **a hot naphtha in a vented tank is B3's two-phase inventory**, and
a demo built the obvious way — a rundown tank feeding a pump — would have shipped
a plant sitting in the state the exclusion exists to keep out of this milestone.
`cavitating_pump.toml` is a source, a junction, a pump, a valve and a sink, so
its only sub-bubble-point node is on the flow path. It is also steady from tick 1,
which is what makes "interior for the whole run" a property of the plant rather
than of the run length.

The mechanism is the one §3's paragraph describes and B1's fixture uses: the pump
is 25 m above its supply. The numbers are chosen against the correlation rather
than tuned — at 110 °C the mixture's bubble pressure is 1.829 bar, the header
sits at 3.0 bar (1.64×), the pump at 1.27 bar (0.70×) and the discharge valve at
3.95 bar (2.16×).

##### 5. What the anchor gate can and cannot buy, stated where it is used

Gate 1 (`P_bub = P_ATM` for a pure cut at its own `tb`) is exact for any Trouton
constant, and therefore — DESIGN §5 correction 4 again — structurally blind to a
wrong one. The note says to reuse `tests/reference/vapour_pressure.rs` for the
magnitude half, and building it made the *mechanism* of that reuse explicit: the
tie is an **exactness identity**, that a pure liquid's bubble pressure IS that
component's vapour pressure, asserted to `1e-15` across the whole validity range
and at three different constants. Only because the two are the same number to the
ULP does the existing envelope grade the new method rather than something near
it. The envelope is then restated on `bubble_pressure` itself, so a reader asking
"is the number the cavitation signal compares against any good?" finds the answer
under that name.

##### 6. What did not change, measured

Every prediction in "What must not change" held. **Fourteen of the fifteen
pre-M11 plants are byte-identical on both fidelities**, and
`crude_column_cascade` moved — by exactly one key on exactly one node, verified
M8.5's way rather than predicted: 60 snapshots, 60 `cavitation` keys, and
stripping them reproduces the before-file byte for byte (276 626 bytes either
side). **No solver iteration count moved anywhere**, on either fidelity. Wall
time on the cascade is unchanged within noise — 320.3 ms before against 330.5 ms
after, paired A/B/A/B in one session, against a control-plant spread of ±15% in
the same runs — which is what a criterion costing five `exp` calls per tick
should look like beside a stage cascade. `measure`, `Controller`, both
`FlowSolver`s and `network.rs` are untouched.

##### 7. The mutation pass, against the predictions

All eight edits compiled, and **the six the note predicted caught were caught,
each by the gate the note named and by nothing that was not expected**:

| edit | caught by |
|---|---|
| mass fractions where mole are needed | the `solvers` unit test and the demo's hand calc — the two written for it |
| the node-kind filter widened to holdups | **gate 4 alone** |
| the filter narrowed to B1's four names | **gate 4's furnace half alone** |
| `bubble_pressure` answering `Ok(0.0)` instead of refusing | the `ConstantThermo` unit test and the whole-plant silence gate |
| the comparison made against `P_ATM` | three gates: the drift check, the wire form, and the cascade's furnace |
| the gas-phase exclusion dropped | its fixture gate alone — **the shipped corpus could not have caught it**, because every gas plant refuses one step earlier |

Two edits changed nothing, and only one of them was predicted.

**`<` widened to `<=` is uncaught, as the note said in advance.** Exact equality
between a solved pressure and a bubble pressure is measure-zero, and a fixture
built to land on it would be fitted to the arithmetic rather than to the physics.

**Swallowing every error variant is uncaught, and the note could not have
predicted it** — the decision it breaks (correction 1) was made while building,
after the note was written. That is the mutation pass doing its actual job:
finding the guard that no test defends because no *loadable* plant can reach it.
The scenario format selects a thermo model by name and both selectable models
either answer or refuse with `Scenario`, so the "every other variant fails the
tick" arm was a branch that compiled, was cited in a design note as satisfying
rule 5, and had never once run. `solvers/tests/cavitation_contract.rs` closes it
with a stub pair — one refusing with `Numerical`, one with `Scenario`, on the same
hand-built plant — which is `separation_contract.rs`'s shape and exists for
exactly the same reason.

**The general form, and it is the third time this project has hit it**: a
decision made *while building* has no gate unless someone writes one, because the
gate list was drawn up against the note. Read the diff for decisions the note does
not contain, and mutate those too.

## 14. The two-phase holdup (M12) — specified before building

### What fired, and what this milestone is not

`docs/DEFERRED.md` B3 — "two phases in one `Stream`" — went past its trigger on
2026-09-06 and is the only row in the ledger on the wrong side of its own number.
It names **four** paths into a two-phase state, and it is worth being exact about
which one fired, because the other three decide this milestone's size:

1. a **flashing feed line** — refused at load,
2. a **partial condenser** — refused at load,
3. a **vapour side draw** — refused at load,
4. a **two-phase holdup** — *not* refused, because nothing in the file is wrong
   at load. It arrives during the run, from a draw temperature.

Only (4) fired. `crude_column_cascade`'s `naphtha_tank` sits below its own bubble
pressure — on the plant as shipped, on its own declared `thermo = "trouton"`,
with no edit to any file — for 4 176 of 6 000 ticks, worst margin 0.940× at tick
3 580, first crossing at tick 1 825. The other three paths still have no plant
asking, and each is a `Stream` change: a vapour fraction on `Stream`, a
`Composition` carrying two phases, and every reader of both.

**So M12 does not close B3, and this note says so before it argues anything.**
It takes the fourth path. The three stream paths keep the row. This is M11's
shape repeated deliberately: that milestone's row was called "no cavitation
floor", it shipped neither a floor nor §3's clamp, and the gap between what it
shipped and what its row named became ledger row B9. **A milestone that closes
part of a row must say which part, or the row's noun goes stale** — which the
ledger has now recorded three separate times against B1 and B3.

### The finding that shapes the whole note: the holdup does not need a two-phase state vector

The reflex reading of B3 is that a boiling tank must *store* two phases — a
vapour fraction on `TankState`, a liquid inventory and a vapour inventory, and a
pressure to hold the vapour at. That reading is what makes B3 "a milestone, not a
slice", and it is what the approved scope named.

It is also wrong for a tank, for a reason written into the type ten milestones
ago. `NodeKind::Tank` is documented as **vented** — "gas blanket pressure =
P_ATM" — and `NodeKind::Vessel` is documented as gas-only, with the sentence
"the two kinds partition the holdups by phase". A vapour that forms in a vented
tank **leaves**. There is nothing to store, and storing it would destroy the
partition that has held since M5.

That gives M12 a shape M11 could not have: the vapour is not mass in a phase the
state vector lacks, which is exactly why §13 fork 1 rejected a pressure clamp.
It is mass that **exits by an accounted path**, which is M6's leak-to-`Atmosphere`
doctrine applied to a holdup instead of a pipe.

**Consequence for scope, stated at the top rather than discovered at fork 5:**
this milestone is *narrower* than "phase in the state vector". `Stream` does not
change. `Composition` does not change. The ~50 files that read them do not
change. What changes is one branch of `Engine::tick`'s step 3, one new edge kind,
and the load-time rules around it.

### Fork 1 — what the engine does when a holdup crosses its bubble point

Four shapes, and the choice is the milestone.

**(a) Nothing but report it — M11's signal, extended to holdups. Rejected here,
and the reason is not the one that rejected the clamp.** M11's criterion is a
diagnostic because the alternative was unsound; here the alternative is sound.
Reporting alone would leave the shipped cascade's naphtha tank heating past its
boiling point indefinitely with a lamp lit, which is not a plant state — it is the
absence of a term. B9 exists because a *pump* cannot be corrected without phase;
a *tank* can.

**(b) An `Err` — refuse the tick. Rejected, on §13 fork 1's grounds and one
more.** The solve succeeded and the state is a genuine root. And it would turn
4 176 ticks of a shipped regression anchor into a dead run, which converts a
modelling gap into a broken corpus.

**(c) Store the vapour — a two-phase `TankState`. Rejected.** It needs a pressure
state a vented tank does not have, it destroys the `Tank`/`Vessel` phase
partition, and it is the expensive reading of B3 that the venting argument above
shows a tank does not need. It is what a *pressurised* holdup would need, which
is why it stays on the row rather than being deleted.

**(d) Boil it off — vaporise the superheat and vent the vapour. Chosen.** The
tank holds at its bubble temperature, the vapour leaves through a vent, and mass
and energy both balance because nothing is unaccounted. This is the physical
behaviour of an atmospheric tank filled with liquid above its boiling point, and
it is a term the engine is already shaped to carry.

### Fork 2 — the vapour leaves at `y = K·x`, not at `x`

**The trap first, because it is silent.** Decrementing the tank's mass at the
tank's own composition conserves mass exactly, passes I1 and I7, and **never
changes the tank's composition** — so the light cut that is doing the boiling
stays in the tank forever. That is the opposite of what boiling does, and no
conservation test can see it, because it conserves.

What leaves is the equilibrium vapour: `y_c = K_c(T, P) · x_c`, mole fractions,
normalised. `ThermoModel::k_value` has returned exactly this since M7.2. The
mass ⇄ mole conversion happens at the unit's boundary, which is §5 fork 1's rule
and M4.2's "the real crux was UNITS, not the ODE".

**A boil-off is a flash, not a decrement**, and the one-line test of whether an
implementation understands that is whether the tank's composition moves.

### Fork 3 — how much boils: enthalpy-limited, not rate-limited

Two formulations.

**(a) A rate law** — a mass-transfer coefficient times the superheat. Rejected:
it needs a constant nobody has, and a number nothing anchors is the
authoritative-looking-figure failure this workspace refuses (the same argument
that made `PseudoComponent::density` an `Option` in M5.2).

**(b) An enthalpy constraint — chosen.** The tank cannot be above its bubble
temperature; the mass that boils is the mass whose latent heat absorbs the
excess:

```text
f = cp̄ · (T − T_bub(P_node, x)) / Δh̄_vap        [fraction of the inventory]
```

with the tank left at `T_bub`. No new constant: `cp̄` is
`Composition::mixture_cp`, `Δh̄_vap` is `ThermoModel::dh_vap` mass-weighted, and
`T_bub` is fork 5's root find. This is the same move `energy` already makes for a
tank's ambient exchange — a signed term derived from state, with no tuning knob.

**The flash fraction is clamped to 1, and the second table above is why that is a
specification rather than defensive coding.** Three tanks on the flipped FCC plant
receive a stream whose `f_in` exceeds 1 — the column draws at 800.4 K into a tank
whose own contents boil at 374.4 K, so **more than all of what arrives would have
to flash**. The constraint has no solution there: there is not enough mass to
absorb the arriving enthalpy at the bubble point, and the honest answer is that
everything arriving boils off and the tank does not fill. The temperature is left
at `T_bub` on whatever inventory remains, which is the "hold the last valid value"
rule the existing `MIN_THERMAL_MASS_KG` branch already applies to a nearly-empty
tank.

**The clamp is about the arriving stream, not the standing inventory**, and that
distinction is this note's own correction: the first draft argued it from the
uncorrected engine's accumulated superheat, which the boil-off term prevents from
ever existing. **A specification defended by a number the fixed engine cannot
produce** is the instrument error this ledger has now recorded three times against
B1 and B3, and it was one draft away from being made a fourth — inside the note
that cites the rule.

### Fork 4 — where the vapour goes: a vent edge, not a bare decrement

**The bare decrement breaks the backbone tests, and that is the argument.** I1 is
"Σ inflow = Σ outflow + Δ inventory per tick". Mass that leaves an inventory with
no outflow edge fails it — or, worse, is absorbed by a tolerance and passes.

So the vapour leaves by an **edge to `Atmosphere`**, and I1 counts it with no
change to I1. Two precedents make this cheap:

- **`leak_to`** (M6.0) already builds a tank/pipe → `Atmosphere` edge and already
  refuses a target that is not an atmosphere.
- **A prescribed-flow edge already exists.** A column draw's flow is not
  pressure-driven: `network::edge_flows` recognises it by topology, guards it to
  **zero**, and `Engine::tick` writes the authoritative value post-sweep. A vent's
  flow is set by an enthalpy balance, not by `ρ·branch.flow(dp)`, so it is the
  same case — and the comment that guards the draw names exactly the hazard a
  pressure-driven vent would have: "a finite, deterministic, mass-conserving
  *wrong* number that nothing downstream flags".

The vent flow is written in step 3, where the boil-off is computed, rather than
step 2b where a draw is written — a draw needs the feed composition, a vent needs
the holdup update that precedes it.

### Fork 5 — the bubble TEMPERATURE, which is a root find

§13 fork 2 recorded that the bubble *pressure* is a weighted sum of closed forms
while the bubble *temperature* is a root find, and cited it as the reason a
per-tick criterion is affordable. M12 needs the expensive one: the tank's
pressure is given and its temperature is the unknown.

That root find exists — `cascade.rs::bubble_point`, M9.3a's regula-falsi/Illinois
solver on `ln Σ K·x`, 15 evaluations with a `BUBBLE_POINT_MAX_EVALUATIONS` bound.
It is **private to `cascade.rs` and takes mole fractions.** Three options:

- **(a) A fourth `ThermoModel` method.** Rejected: it is not a property lookup,
  it is a solve *over* a property lookup, and putting an iteration behind a trait
  that has three closed forms invites an implementation with a different bracket.
- **(b) Duplicate the root find in `core`.** Rejected outright — `core` gets no
  solver, and a second bracket is the "two notions of the same thing" failure.
- **(c) Promote `cascade::bubble_point` to a shared `solvers` helper — chosen.**
  It is already the workspace's one answer to this question, it is already
  measured and bounded, and the mass ⇄ mole conversion at its edge is the
  boundary §5 fork 1 already draws. The engine reaches it the way it reaches
  every other solver: through a trait object, so the seam question is which one.

**This is the note's weakest fork and it is flagged as such**: it is the only one
where the chosen option moves existing code rather than adding beside it, and
M9.3a's constants were tuned against the cascade's compositions rather than a
tank's. The building slice re-measures the evaluation count on a tank before
trusting the bound.

### Fork 6 — where in the tick

Inside step 3's `NodeKind::Tank` branch, after `mass_new` and the temperature are
computed and before the finiteness check in step 4. Not step 5, where M11's
criterion lives: that is a diagnostic evaluated on finished state, and this is a
slow state's own dynamics.

The ordering matters for one reason worth naming: the tank must be allowed to
*reach* the superheated state within the tick and be corrected in the same tick,
not the next one. A one-tick lag here is M3.2's lesson (a lag that does not
vanish as `dt → 0` is a defect, not a lag) — the boil-off is an algebraic
constraint, so it must be applied at the same time as the state it constrains.

**Amended by fork 9: the placement stays, the arithmetic does not.** With the
term behind a selectable model, everything fork 3 specifies — the flash
fraction, its clamp at 1, the `y = K·x` composition — belongs to the impl, and
`Engine::tick`'s `Tank` branch keeps only two things: *when* to ask (here, this
tick) and *what to do with the answer* (fork 4's vent edge). That division is
what rule 2 means by "fidelity is trait impl selection": the branch must contain
no arithmetic that differs between `"none"` and `"flash"`, or the key is an
`if simple_mode` in disguise.

### Fork 7 — which plants can evaluate this at all

**Fourteen of the sixteen shipped plants declare `thermo = "constant"`, whose
`k_value`, `dh_vap` and `bubble_pressure` are all `Err` by design.** A boil-off
needs all three. So as shipped this term is reachable on exactly **two** plants,
and only one of them holds a boiling liquid.

This is the constraint that killed B1's distance twice and B3's once — *a
distance is a property of the engine, not of the plant* — and it is checked here
before the gates are written rather than after. The consequence: the `Scenario`
`Err` arm M11 established is inherited exactly. A model that cannot answer means
**no boil-off term**, not a boil-off of zero, and fourteen plants must stay
byte-identical for that reason and not by accident.

**Measured before the forks below were settled**, over 6 000 ticks of every
liquid tank on the three plants whose thermo model can answer, through the
engine's own `TroutonThermo`. `f` here is the share of the inventory whose latent
heat would absorb the tank's *standing* superheat — measured on the UNCORRECTED
engine, where nothing removes it. **It is accumulated drift, not a rate, and the
second table below is what fork 3 actually rests on.** Reading this one as a
per-tick quantity is the error this note made in its first draft and corrected
before anything was built on it:

| plant | tank | first boils | ticks boiling | worst superheat | worst `f` |
|---|---|---|---|---|---|
| `crude_column_cascade` (**as shipped**) | `naphtha_tank` | 1 825 | 4 176 / 6 000 | 2.31 K | **0.0166** |
| `crude_column_cascade` | `distillate_tank`, `bottoms_tank` | — | 0 | — | 0 |
| `crude_column` (flipped) | `naphtha_tank` | 324 | 5 677 / 6 000 | 52.3 K | **0.376** |
| `fcc_plant` (flipped) | `gasoline_tank` | 290 | 5 711 / 6 000 | 300.4 K | **2.013** |
| `fcc_plant` (flipped) | `gas_drum` | 1 | 6 000 / 6 000 | 495.3 K | **1.811** |
| `fcc_plant` (flipped) | `bottoms_tank` | — | 0 | — | 0 |

**The quantity fork 3 needs is a different one, and measuring it separately
changed two of this note's claims.** Under the boil-off the tank never reaches
678 K — it parks at its bubble point and thereafter carries only the superheat one
tick's inflow adds. So what decides both the cost and the clamp is the flash
fraction of the **arriving stream**,

```text
f_in = cp̄ · (T_in − T_bub) / Δh̄_vap
```

which is `1` when everything arriving boils and the tank cannot fill. Measured the
same way, with `T_in` the enthalpy-weighted inflow temperature:

| plant | tank | `T_in` | `T_bub` | `f_in` | inventory share per tick |
|---|---|---|---|---|---|
| `crude_column_cascade` (**as shipped**) | `naphtha_tank` | 387.6 K | 354.3 K | **0.236** | 4.1e-4 |
| `crude_column_cascade` | `distillate_tank` | 505.3 K | 491.2 K | 0.117 | 2.8e-4 |
| `crude_column_cascade` | `bottoms_tank` | 632.0 K | 623.5 K | 0.097 | 2.8e-5 |
| `crude_column` (flipped) | `naphtha_tank` | 445.8 K | 354.3 K | 0.648 | 1.1e-3 |
| `fcc_plant` (flipped) | `gasoline_tank` | 800.4 K | 374.4 K | **2.854** | 1.4e-3 |
| `fcc_plant` (flipped) | `gas_drum` | 800.4 K | 233.6 K | **2.072** | 2.1e-3 |
| `fcc_plant` (flipped) | `bottoms_tank` | 800.4 K | 675.9 K | **1.261** | 3.8e-4 |

**Correction 1 — "gentle" was true of the wrong quantity, and the two readings
point opposite ways.** Per tick the term is genuinely small everywhere: the worst
inventory share is 2.1e-3, so nothing here is stiff and no integrator argument is
needed. But the shipped cascade's naphtha tank settles at `f_in = 0.236` — **a
quarter of the naphtha product boils off** once the tank reaches its bubble point
at tick 1 825. That is a large plant-level correction to a pre-M8 regression
anchor, and the first draft of this note called it small on the strength of the
1.7% drift figure. The term is cheap to integrate and expensive in what it says
about the plant; those are different sentences and only one of them was measured.

**Correction 2 — `f ≥ 1` is reachable, but not for the reason the first draft
gave.** It is not that a stored inventory holds more heat than its own latent heat
can absorb; under the correction no inventory ever gets there, because the term
prevents the superheat from accumulating. It is that **three** tanks on the
flipped FCC plant receive a stream so far above their own bubble point that more
than all of it flashes — the column draws at 800.4 K into a tank whose contents
boil at 374.4 K. Such a tank cannot fill at all. The clamp is therefore about the
arriving stream, the case is reachable on three tanks rather than two, and gate 6
asserts against an inflow rather than an inventory.

**Correction 3 — the term is not as selective as the first table suggested.**
The first measurement found only one of the cascade's three tanks boiling, and
this note called the other two a control. They are not: their inflow is already
above their own bubble points (`f_in` of 0.117 and 0.097) and they have simply not
heated there within 6 000 ticks. **A tank that is not boiling yet is not a
control**, and a gate that uses one as its negative case is asserting on the run
length rather than on the physics.

### Fork 8 — the demo plant, and a gate that may not be writable on the shipped one (amended by fork 9)

Every regulation and criterion milestone since M8.4 has shipped a NEW file rather
than wiring the feature into an existing plant, because thirteen of the sixteen
scenarios are the regression anchor. M12 has a sharper reason: **the plant that
fired the trigger may be unable to gate the milestone's central fork.**

`crude_column_cascade`'s `naphtha_tank` is **declared** `composition =
{ light_naphtha = 1.0 }` — a pure component. For a pure fluid `y = K·x` and `x`
are the same vector, so fork 2's entire distinction — a flash versus a decrement —
would be invisible on it, exactly as §13's water-only fixture could not tell a
mixture bubble point from a single component's vapour pressure.

**Measured rather than assumed, and it comes out the other way.** The stage-0
draw is not pure light naphtha: by tick 6 000 the tank holds **0.5299 light /
0.4685 heavy / 0.0016 kerosene**, and it is already a real mixture long before it
starts boiling at tick 1 825. So the shipped plant *can* carry the milestone's
central gate. `crude_column`'s naphtha tank is 0.5325 / 0.4675, the same story.

**But the declaration is the trap.** A gate written against the file's own
`light_naphtha = 1.0` would be asserting on a composition the tank holds only at
tick 0, and a short-running gate would sample exactly that. Gate 1 must assert
that the tank is a mixture as a **control** before it asserts anything about the
flash — the same shape M9.3b's warm-start gate needed, where two controls came
before the assertion because without them the gate is passed by a solver that
ignores its seed.

The demo file is still new rather than wired into an existing plant, for the
thirteen-anchor reason above — but the shipped cascade is now the *regression*
case AND a usable second exercise, not a plant that cannot see the feature.

**Amended by fork 9, and the amendment makes the demo almost free.** With the
term selectable, `crude_column_cascade.toml` keeps `boiloff = "none"` and stays
byte-identical, and the demo is that same file with **one line changed**. That is
deliberately the M7 pattern: `crude_column.toml` and `crude_column_cascade.toml`
already ship as a pair meant to be diffed, identical in feed, rates and tanks and
differing only in what the separation model does with them. A third file in that
family — same plant, `boiloff = "flash"` — makes the two answers directly
comparable in one `corpus` run, which is the only honest way to present a fork
where **neither model is provably better** (fork 9). It also means every gate
below runs on a plant whose *only* difference from a shipped anchor is the key
under test, so a gate that fires is pointing at the term and not at the plant.

### The gates, named before building, and the vacuity each one closes

1. **The tank's composition MOVES, and toward the heavy end.** The gate for fork
   2, and the only one that separates a flash from a decrement. A decrement at
   `x` conserves mass, passes I1 and I7, and leaves the fractions untouched — so
   every conservation gate in the workspace is passed by the broken version.
   Needs a multi-component boiling tank (fork 8). Two controls asserted first,
   M9.3b's shape: the tank must actually be boiling, and its inflow must not
   itself be driving the composition the same way.
2. **Mass balances with the vent counted, and FAILS without it.** I1 on a plant
   with an active boil-off, plus the counterfactual — delete the vent edge and
   the balance must break. Without the second half this gate is passed by an
   engine that never boils anything.
3. **The tank parks AT its bubble temperature.** `T ≤ T_bub(P_node, x) + ε` for
   every tick after the first crossing, with `ε` derived from the Euler step
   rather than chosen (M2's truncation-tolerance rule).
4. **NOT a gate: "the mass removed times the latent heat equals the excess
   enthalpy".** That is the definition of the mass removed. It is the M7.4b /
   M7.4c trap — a quantity defined to close a balance cannot be gated by that
   balance — and it is written down here so the building slice does not
   rediscover it for the fifth time. What replaces it is gate 3, whose two sides
   (a temperature trajectory and a thermodynamic property) are computed by
   independent paths.
5. **ALL SIXTEEN shipped plants byte-identical, on both fidelities** — raised
   from fourteen by fork 9, because the two plants that *can* evaluate the term
   now decline it by declaring `boiloff = "none"`. Verified M8.5's way — strip
   the new key and reproduce the before-file byte for byte — rather than
   predicted. **This gate got stronger and easier at the same time, which is
   worth distrusting**: an all-identical corpus is also what a key that is parsed
   and ignored produces (fork 9, trap 3), so gate 5 is only meaningful beside
   gate 7.
6. **The everything-flashes case terminates honestly.** Where `f_in ≥ 1` the tank
   cannot fill: it must not reach a negative mass, a NaN, or a sub-zero
   temperature, and the mass balance must still close over the tick. Asserted
   against an **inflow**, not an inventory — the inventory version of this gate
   defends a state the correction prevents from ever arising, which would be the
   fifth specified gate in this project with no power over its own subject.
7. **The two files differ, and differ in the predicted direction.** Added by fork
   9. The demo and its one-line twin must disagree on the naphtha tank's mass,
   temperature and composition, by roughly the measured `f_in = 0.236` rather
   than by any non-zero amount. This is the gate that makes gate 5's silence mean
   "declined" rather than "ignored", and it is the only thing standing between
   this key and the `thermo = "nonsense"` defect.

   **Its control has to be asserted first, and it is not the obvious one.** "The
   two files differ" is *also* satisfied by a demo that differs for some
   unrelated reason — a mistyped draw ratio, a different tank geometry — and
   such a gate would be green while proving nothing about the key, which is
   `smearing_k`'s shape with extra steps. So the gate must first assert that the
   two TOML documents are identical **except for the `boiloff` line**, textually,
   before it asserts anything about the numbers. That is the same
   control-before-assertion discipline gate 1 uses for the tank's mixture and
   M9.3b's warm-start gate uses for its two profiles, and here it is what makes
   the difference attributable to the key rather than to the plant.

### The mutations this slice owes, named before building

| edit | prediction |
|---|---|
| vapour leaves at `x` instead of `y = K·x` | caught by **gate 1 alone**, and only on a multi-component tank — every conservation test passes |
| the vent edge dropped, mass decremented in place | caught by **I1** and gate 2 |
| the temperature clamped to `T_bub` with no mass removed | energy destroyed rather than carried out; predicted caught by gate 2, NOT by gate 3 |
| the boil-off applied one tick late | predicted caught by gate 3 only if `ε` is derived; a chosen tolerance passes it (M8.3's 4.66e-5 lesson) |
| a `Scenario` `Err` read as "boil off zero" rather than "no term" | predicted **inert**, and that is worth recording: unlike M11's snapshot key, where `false` and absent differ, a zero term and no term are the same arithmetic |
| `f` clamped to 1 removed | the `f ≥ 1` case; predicted caught by gate 6 |
| `boiloff` parsed and then ignored (the impl always `"none"`) | **the M1–M7.2 `thermo` defect, reproduced deliberately.** Predicted caught by gate 7 and by NOTHING else — gate 5 passes, every conservation test passes, and the corpus is all-identical, which is the point |
| the default flipped to `"flash"` | predicted caught by gate 5 on two plants and by no other gate; the other fourteen cannot form the term, so a wrong default is invisible on 87% of the corpus |
| the `"flash"` + `thermo = "constant"` refusal dropped | predicted **not** caught by any gate above — it needs its own refusal test, because no shipped file declares that pairing. Named here so the building slice does not discover it by shipping it |
| an unknown `boiloff` value accepted and treated as `"none"` | same shape; needs its own refusal test. `"pid"` is what M8.3's unknown-algorithm refusal is tested with, and this wants the same |

### What must not change, stated as a prediction that can be wrong

- **REWRITTEN BY FORK 9. All sixteen shipped plants are byte-identical on both
  fidelities**, not fourteen. The first version of this section predicted that
  `crude_column_cascade` would move by roughly a quarter of its naphtha draw and
  called it "the largest anchor movement any milestone in this project has
  taken". With the term behind `boiloff`, the anchor declares `"none"` and does
  not move at all. **The prediction was not wrong about the physics — it was
  wrong about who bears it**, and the movement now lives entirely on the new demo
  file, where it is the feature rather than a cost.
- **The movement itself must still appear, on the demo.** ~24% of the naphtha
  draw leaving as vapour from tick 1 825 is what gate 7 asserts. A slice that
  ships an all-identical corpus AND a demo that matches its twin has built
  nothing, however green it is.
- **The other two cascade tanks are heading the same way and are NOT controls.**
  Their inflow is already above their own bubble points; they have simply not
  heated there within 6 000 ticks. A longer run moves them too — on the demo;
  on the anchor they are frozen by the key.
- **`cavitating_pump` is untouched.** Its only sub-bubble-point node is a pump,
  and M11's node-kind exclusion and this milestone's holdup scope are disjoint by
  construction — which is worth asserting, because the two features now both read
  a bubble point and a reader could reasonably expect one to subsume the other.
- **No solver iteration count moves on any plant that does not boil**, and the
  root find is per boiling tank per tick, not per node.

### Deferred, with what un-defers each

- **The three stream paths of B3** — a flashing feed line, a partial condenser, a
  vapour side draw. All three still refused at load, none with a plant asking.
  **The row stays open and is partially struck**, with M12's name against the
  holdup clause only.
- **A pressurised two-phase holdup.** Fork 1(c). A vented tank has nowhere to
  keep vapour; a `Vessel` holding a boiling liquid would. Un-defers when a plant
  needs a holdup that is neither all-liquid nor all-gas *and* cannot vent.
- **Condensation — the reverse term.** M12 vaporises and never condenses: a
  subcooled vapour arriving at a holdup is the mirror case and is not built.
  Un-defers with a plant whose holdup receives a vapour it must keep.
- **What the vented vapour does after it leaves.** It goes to `Atmosphere` and
  stops being modelled, exactly as a leak does. Routing it to a flare or a vapour
  recovery unit is a topology the format cannot express today.
- **A selectable equation of state, and a selectable `cp(T)`.** Fork 9's other
  half. Both are "wrong in a different way" in the user's sense and neither has a
  shipped plant that can tell the two answers apart, which is this project's bar
  for a fidelity key. `docs/DEFERRED.md` B14 and B15, each with that plant as its
  trigger.
