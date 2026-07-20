---
name: m4-reactor-crux
description: "M4 reactor design note landed 2026-07-20; the crux is conservation + the sensible-only energy datum, not the kinetics, and the energy gate is a two-duty DIFFERENCE"
metadata: 
  node_type: memory
  type: project
  originSessionId: cc96fbf9-f6e0-4aab-99a6-725d6dc6dbd6
  modified: 2026-07-20T18:31:23.931Z
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

**The trap the advisor caught, worth keeping.** The first draft's energy gate
pinned "the sensible discontinuity across the unit" against
`ṁ·[Δ(cp·(T−Tref)) + Δh_rxn]` — but the sensible discontinuity IS the first term,
so the gate reduced to `0 = ṁ·Δh_rxn`, forcing Δh_rxn=0 and failing every real
reactor. Fix: TWO distinct duties. **Emergent sensible duty** `ṁ·Δ(cp·(T−Tref))`
closes by construction (reactor imposes T_set like a furnace imposes duty),
gates the cp-shift. **Reported physical duty** = sensible + `ṁ·Δh_rxn` is a
diagnostic that gates Δh_rxn and NEVER feeds forward outlet T (feeding it back is
the adiabatic case). The gate is the DIFFERENCE (reported sits exactly ṁ·Δh_rxn
above the sensible baseline) — pinning reported-duty against its own formula is
the [[well-posed-is-not-correct]] tautology. Under isothermal, Δh_rxn is
diagnostic-only.

**Why:** the reactor's difficulty was mislocated twice — first onto the kinetics
(it's the conservation collisions), then the energy gate was drafted vacuous. The
sensible-only datum makes reaction heat invisible, so it needs an explicit
parameter with a gate that discriminates rather than restates.

**How to apply:** when M4.1 lands, the energy gate must assert the DIFFERENCE
(reported − sensible = ṁ·Δh_rxn), and the reactor must return `None` from both
`boundary_temperature`/`boundary_composition` (it's swept, T-overriding) — a
`Some` T trips the sweep partition guard. I6/I7 exclude reactors (their networks
contain none); the reactor gets its own total-mass and energy gates.
