//! Property tests — the backbone of solver trust (CLAUDE.md → Testing).
//!
//! Invariants, for ANY randomly generated valid network:
//!   I1. Mass conservation: Σ inflow = Σ outflow + Δ inventory per tick,
//!       to 1e-8 relative (Newton) / documented looser bound (Simple).
//!   I2. No negative inventories, pressures below absolute zero, or NaN/Inf
//!       anywhere in the snapshot, ever.
//!   I3. Solver terminates: Ok(converged) or Err(SolverDiverged). Never
//!       Ok with non-finite values.
//!   I4. Determinism: same scenario + same commands ⇒ byte-identical
//!       serialized snapshots across two fresh engine instances.
//!   I5. Fidelity agreement: Newton and Simple steady states match within
//!       5% on flows for well-posed networks (no near-zero-dP pathologies).
//!
//! Generator plan (implement in a `strategies` module here):
//!   - Random tree of 1..8 tanks/junctions joined by pipes, with 1..3
//!     sources and sinks, pumps/valves inserted on random branches,
//!     parameters drawn from physically sane ranges (documented per unit).
//!   - Shrinking must stay valid: generate parameters first, topology last.

use proptest::prelude::*;

proptest! {
    // Placeholder keeping the harness compiling until M1 solver lands.
    // Replace with I1..I5 as the first act of M1 test work.
    #[test]
    fn harness_smoke(x in 0.0f64..1.0) {
        prop_assert!(x.is_finite());
    }
}
