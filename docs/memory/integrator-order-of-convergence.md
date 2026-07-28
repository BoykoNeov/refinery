---
name: integrator-order-of-convergence
description: "A closed-form gate cannot catch a degraded integrator — the order-of-convergence ratio can; measure the asymptotic range before writing it"
metadata:
  type: feedback
---

M4.2's RK4 kinetics (2026-07-28). An analytic solution to the ODE is a strong
gate for the RATE LAW, and a weak one for the INTEGRATOR: at 64 substeps over
3 s the truncation error is 5.6e-9, so a dropped or mis-weighted RK4 stage still
lands inside any tolerance a physical gate could carry. The closed form passes
either way.

What discriminates is the **order of convergence**: halving the step must cut the
error ~2⁴ = 16. A degraded method reads ~4 (second order) or ~2 (first order),
and no correct implementation can fake it.

**Why:** this is the same shape as [[euler-truncation-tolerance]] — never assert
against the integrator's own arithmetic, assert against a property the correct
integrator has and a wrong one does not. Convergence order is that property.

**How to apply:**
- Expose the substep count (a builder arg) so a test can MEASURE the error at
  several step sizes. It is an integrator setting, not a model parameter, and
  exposing it is what makes the tolerance derived instead of tuned.
- **Measure the asymptotic range first.** The naive `error(4)/error(8)` read 245
  here and meant nothing — the error changes sign between 8 and 16 steps, so
  those points are not asymptotic. The gate runs at 32/64/128 (ratios 14.7, 15.5).
- Assert the shipped count's error is far below the closed-form tolerance in the
  SAME test, so the tolerance's provenance is visible where it is used.
- Corollary for tests generally: a post-normalization sum is a vacuous assertion.
  `Composition::from_weights` normalizes, so "products sum to 1" is true whatever
  the kinetics did — the live guard is the PRE-normalization check, and it is
  falsified by the reaction returning `Err`, not by the sum.
