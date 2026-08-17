//! refinery-solvers: trait implementations at both fidelity levels.
//!
//! | Trait           | Simple (game)        | Complex (research)      |
//! |-----------------|----------------------|-------------------------|
//! | FlowSolver      | SimpleFlowSolver     | NewtonFlowSolver        |
//! | ThermoModel     | ConstantThermo (M2)  | CutThermo (M2/M3)       |
//! | ReactionModel   | SimpleLookup (M4.1)  | FourLump (M4.2)         |
//! | SeparationModel | CutPointSplitter     | StageCascade (M7.3)     |
//!
//! Selection happens in refinery-scenarios from TOML config; nothing in
//! here or in core branches on a fidelity flag.

pub mod elements;
pub mod four_lump;
pub mod network;
pub mod newton_flow;
pub mod reactor;
pub mod separation;
pub mod simple_flow;

pub use four_lump::{FourLump, FourLumpParams};
pub use newton_flow::NewtonFlowSolver;
pub use reactor::SimpleLookup;
pub use separation::CutPointSplitter;
pub use simple_flow::SimpleFlowSolver;

use refinery_core::components::{Composition, Slate};
use refinery_core::error::SimError;
use refinery_core::traits::{Reaction, ReactionModel, ThermoModel};
use refinery_core::units::{JPerKg, Kelvin, Seconds};

/// M1 placeholder: constant-property water; Composition's ideal-mixing
/// helpers carry properties until M2.
pub struct ConstantThermo;
impl ThermoModel for ConstantThermo {
    fn name(&self) -> &'static str {
        "constant"
    }
}

/// Identity reaction: the feed passes through unchanged with zero heat of
/// reaction. The default for every network with no reactor, and the regression
/// anchor that keeps all pre-M4 goldens bit-identical (`react` is never even
/// called without a `Reactor` node present).
pub struct NoReactions;
impl ReactionModel for NoReactions {
    fn name(&self) -> &'static str {
        "none"
    }
    fn react(
        &self,
        feed: &Composition,
        _temperature: Kelvin,
        _tau: Seconds,
        _slate: &Slate,
    ) -> Result<Reaction, SimError> {
        Ok(Reaction {
            products: feed.clone(),
            dh_rxn: JPerKg::ZERO,
        })
    }
}
