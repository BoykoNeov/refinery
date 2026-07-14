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
}
