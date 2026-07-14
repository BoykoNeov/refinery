//! refinery-solvers: trait implementations at both fidelity levels.
//!
//! | Trait          | Simple (game)        | Complex (research)      |
//! |----------------|----------------------|-------------------------|
//! | FlowSolver     | SimpleFlowSolver     | NewtonFlowSolver        |
//! | ThermoModel    | ConstantThermo (M2)  | CutThermo (M2/M3)       |
//! | ReactionModel  | LookupReactor (M4)   | FccFourLump (M4)        |
//!
//! Selection happens in refinery-scenarios from TOML config; nothing in
//! here or in core branches on a fidelity flag.

pub mod elements;
pub mod newton_flow;
pub mod simple_flow;

pub use newton_flow::NewtonFlowSolver;
pub use simple_flow::SimpleFlowSolver;

use refinery_core::traits::{ReactionModel, ThermoModel};

/// M1 placeholder: constant-property water; Composition's ideal-mixing
/// helpers carry properties until M2.
pub struct ConstantThermo;
impl ThermoModel for ConstantThermo {
    fn name(&self) -> &'static str {
        "constant"
    }
}

/// M1–M3 placeholder: no chemistry.
pub struct NoReactions;
impl ReactionModel for NoReactions {
    fn name(&self) -> &'static str {
        "none"
    }
}
