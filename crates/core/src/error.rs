//! Engine error type. The engine never panics and never lets NaN escape:
//! bad numerics become `Err` with diagnostics.

use thiserror::Error;

#[derive(Debug, Error)]
pub enum SimError {
    #[error("hydraulic solver diverged after {iterations} iterations (residual {residual:.3e})")]
    SolverDiverged {
        iterations: u32,
        /// Final ‖R‖_∞ (kg/s).
        residual: f64,
        /// Per-iteration ‖R‖_∞ history for diagnostics (DESIGN §3).
        residual_history: Vec<f64>,
    },

    #[error("non-finite value produced at {location}")]
    NonFiniteState { location: String },

    #[error("invalid command: {0}")]
    InvalidCommand(String),

    #[error("scenario error: {0}")]
    Scenario(String),

    #[error("numerical error: {0}")]
    Numerical(String),

    /// The anchoring active set did not reach a fixed point within one solve
    /// (M8.0, docs/DESIGN.md §3c). A LEGAL termination, like `SolverDiverged`:
    /// the plant has no single answer to report, and the solver says so rather
    /// than picking one.
    ///
    /// `cycled` separates the two ways that happens, and they are genuinely
    /// different plant states — which is why this is a variant rather than a
    /// `Numerical` carrying a sentence. `true` is a classification the solve has
    /// already seen: two self-consistent answers, physically a relief whose own
    /// discharge re-seats it (chatter, deferred with element state — §3a).
    /// `false` is a solve still producing new classifications at the pass cap.
    /// Callers that must accept this outcome — the I3 termination invariant does
    /// — need to tell it apart from an ordinary numerical failure, and a
    /// substring of a human-readable message is not something to gate on.
    #[error("the anchoring classification did not settle: {detail}")]
    AnchoringUnsettled { cycled: bool, detail: String },
}
