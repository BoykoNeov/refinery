---
name: euler-truncation-tolerance
description: "Tank ambient exchange landed 2026-07-20 (M2.2); when a test's tolerance is integrator truncation rather than round-off, derive it — and never assert against the integrator's own formula"
metadata: 
  node_type: memory
  type: project
  originSessionId: b9be0288-bf78-41f2-8be5-c8123ea5d306
  modified: 2026-07-20T10:19:24.015Z
---

Tank ambient heat exchange landed 2026-07-20, closing the M2.2 tank box
(`Q = UA·(T_ambient − T_tank)` via `energy::ambient_exchange`, summed by
`heat_load`, picked up unchanged by the engine's tank loop). The pipe box is
still open and deliberately separate.

The non-obvious, reusable part is **what a test's tolerance is made of**. Every
earlier tank reference case ran at 1e-9 because `Q` was constant: explicit Euler
is then EXACT, and the only error is float round-off. Ambient exchange breaks
that — `Q` depends on `T`, so the engine produces `(1−α)^N` where physics has
`exp(−αN)`, with `α = UA·dt/(m·cp)`. That gap is *truncation*, and copying the
familiar 1e-9 would fail a correct implementation.

**Why:** the trap is that both available fixes are wrong. Loosening the number
until it passes hides how much error is admitted; asserting against `(1−α)^N`
instead makes it tight again but tests nothing — that formula IS the integrator,
so it agrees with any Euler implementation of any wrong `Q`. Same shape as the
tautology trap in [[kv-handcalc-reference]].

**How to apply:** derive the tolerance in closed form and write the derivation
into the test (here `α`=1e-4, `N`=10 000 ⇒ `e⁻¹·5e-5` ⇒ ~3.7e-4 K, so 1e-3).
Then falsify to prove the looseness costs nothing — a 0.1% error in the term
still failed. Prefer splitting: a single-tick anchor where the step is exact
keeps a round-off-tight statement of the term's magnitude, and the multi-tick
case carries the integrator's error alone, so the two failure modes never blur.

Two things this leaves for the pipe box: `α > 2` diverges — irrelevant for a
tank (~1e-6, documented in `Engine::tick`, not guarded) but NOT for a pipe,
where `ṁ·cp` can be small, which is the second reason it needs the analytic
plug-flow transform. And `thermal_plant_conserves_energy` will genuinely fail if
a plant ever feeds it a `UA > 0` tank: ambient is an open-system term. Let it
fail loudly rather than widening it in advance. See also
[[heat-exchanger-pair-merge]].
