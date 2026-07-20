---
name: m4-reactor-crux
description: "M4 reactor: crux is conservation + the sensible-only datum (not kinetics); energy gate is a two-duty DIFFERENCE. M4.1 (NodeKind::Reactor + SimpleLookup) LANDED 2026-07-20"
metadata: 
  node_type: memory
  type: project
  originSessionId: cc96fbf9-f6e0-4aab-99a6-725d6dc6dbd6
  modified: 2026-07-20T19:28:33.439Z
---

M4 (reactor) opened 2026-07-20 with a design-note-first slice (DESIGN §5
"Simple reactor (M4)", ROADMAP M4.0/M4.1/M4.2). The crux is **not** the FCC
kinetics — it is that a reactor is the first unit to change composition by
chemistry, colliding with BOTH M3 invariants: total-mass-neutral but not
per-component (vs I7), and it moves chemical energy the sensible-only datum does
not track (vs I6).

Three forks settled:
- **Lumps ARE slate members** (resolved by name), not a Tb-band↔lump mapping —
  the within-lump redistribution a mapping needs is underdetermined. Cost: coke
  is an awkward pseudo-component (high Tb so a column routes it to bottoms).
- **Isothermal at a ROT setpoint**, not adiabatic — the "columns run on pressure
  control" move again ([[m3-column-solver-crux]]). Holding T_set makes extent a
  pure function of a known T (no inner solve); adiabatic couples dC/dτ+dT/dτ into
  a fixed point inside a zero-volume node — deferred.
- Structural payoff: **the reactor is hydraulically a furnace** (zero-volume,
  1-in-1-out, mass-neutral) → no new solver machinery; the one real change is
  `resolve_node_states` gains `&dyn ReactionModel` and reacts INSIDE the sweep
  (so downstream sees product); single uniform outlet leaves `edge_composition_at`
  untouched (unlike the column's N draws).

**The energy-gate trap (kept because it recurs).** A first-draft gate that pins
"the sensible discontinuity" against `ṁ·[Δ(cp·(T−Tref)) + Δh_rxn]` reduces to
`0 = ṁ·Δh_rxn` (the discontinuity IS the first term), forcing Δh_rxn=0. Fix: TWO
distinct duties. **Emergent sensible duty** `ṁ·Δ(cp·(T−Tref))` closes by
construction (reactor imposes T_set like a furnace imposes duty), gates the
cp-shift. **Reported physical duty** = sensible + `ṁ·Δh_rxn`, a diagnostic that
gates Δh_rxn and NEVER feeds forward outlet T. The gate is the DIFFERENCE, not
reported-duty vs its own formula ([[well-posed-is-not-correct]] tautology).

**M4.1 LANDED 2026-07-20** (`NodeKind::Reactor { t_set, tau }` + `SimpleLookup`),
four commits on `m4-reactor`. Durable lessons from the build:
- **The reactor is a T-OVERRIDING zero-volume node** — the first whose two
  intensive fields diverge. It returns `None` from BOTH `boundary_temperature`
  and `boundary_composition` (swept, not inertial: `Some` T trips the partition
  guard), yet its composition is the feed mix REACTED and its T is IMPOSED
  (t_set), not mixed. Handled in the sweep's single-node branch: `inflow_totals`
  FIRST (need T_in + ṁ for the emergent duty), then overwrite composition with
  products and T with t_set. t_set holds even with ṁ=0 (a config setpoint, not a
  derived value) — cleaner than a furnace's "hold previous".
- **The energy gate is a CORE unit test with an inline mock `ReactionModel`** — no
  scenario/solver needed. Mutation-verified BOTH terms fall independently:
  feed-cp-for-product-cp breaks the sensible assert; a dropped Δh breaks the
  difference ([[kv-handcalc-reference]] falsification discipline).
- **The reactor total-mass gate is near-tautological — attribute it correctly.**
  Hydraulic continuity at the free zero-volume node forces m_out==m_in REGARDLESS
  of the reaction, and `Composition::from_weights` normalizes products to Σ=1
  unconditionally. So "mass in == out" guards the SOLVER's continuity, not the
  reaction's mass-neutrality; the real guard is row renormalization (Σ=1),
  covered by SimpleLookup's own test. (Unlike the column's N prescribed draws,
  which genuinely could fail to sum — [[m3-column-landed]].) The integration
  test earns its keep on composition-changed / setpoint / wiring asserts.
- **I6/I7 exclude reactors — STATE it at the generator, don't rely on the proptest
  passing** ([[unfalsifiable-is-a-claim-about-coverage]]). A comment at each
  `build_plant`/`build_thermal_plant`: I7 excludes because a reactor breaks
  per-component mass by construction; I6 because ṁ·Δh_rxn sits outside its
  sensible-only frame.
- **Trait shape refined from the roadmap sketch, advisor-endorsed:**
  `react(feed, T, tau, slate) -> Reaction { products, dh_rxn }` — tau threaded now
  (M4.2 kinetics integrate over it; saves a churn), named struct not a tuple,
  `dh_rxn: JPerKg` with sign convention **positive = endothermic** (so reported =
  sensible + ṁ·Δh_rxn and FCC cracking is positive). Duties returned on
  `NodeStates.reactor_duty` (NOT the snapshot — no speculative surface).

Next: M4.2 — `FourLump` Arrhenius kinetics (fixed-count RK4 over τ, determinism),
Δh_rxn from formation enthalpies, reference against published lump yields; the
crude→furnace→reactor→column→tanks demo.
