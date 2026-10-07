//! refinery-solvers: trait implementations at both fidelity levels.
//!
//! | Trait           | Simple (game)        | Complex (research)      |
//! |-----------------|----------------------|-------------------------|
//! | FlowSolver      | SimpleFlowSolver     | NewtonFlowSolver        |
//! | ThermoModel     | ConstantThermo       | TroutonThermo (M7.2)    |
//! | ReactionModel   | SimpleLookup (M4.1)  | FourLump (M4.2)         |
//! | SeparationModel | CutPointSplitter     | StageCascade (M7.3)     |
//! | BoilOffModel    | NoBoilOff (M12.1)    | FlashBoilOff (M12.1)    |
//! | Controller      | ProportionalController (M8.2) | PiController (M8.3) |
//!
//! The last row is the odd one and says so here rather than in a comment nobody
//! reads: a `Controller` is chosen PER LOOP, not once per engine, so it is not
//! selected by a `[fidelity]` key at all — a plant declares one per `[[controls]]`
//! entry (docs/DESIGN.md §10 fork 2). It is also the first row whose impls own
//! state.
//!
//! Selection happens in refinery-scenarios from TOML config; nothing in
//! here or in core branches on a fidelity flag.
//!
//! The thermo row said `CutThermo (M2/M3)` until M7.2. No such type was ever
//! written and no milestone ever owed one — the trait had no methods, so there
//! was nothing for a second implementation to differ about. It is named here
//! because a table that promises a type is a claim like any other.

pub mod boiloff;
pub mod bubble;
pub mod cascade;
pub mod control;
pub mod elements;
pub mod enthalpy;
pub mod flash;
pub mod four_lump;
pub mod line_flash;
pub mod molar;
pub mod network;
pub mod newton_flow;
pub mod reactor;
pub mod separation;
pub mod simple_flow;
pub mod thermo;

pub use boiloff::{FlashBoilOff, NoBoilOff};
pub use bubble::bubble_temperature;
pub use cascade::StageCascade;
pub use control::{PiController, ProportionalController};
pub use enthalpy::{ConstantEnthalpy, LinearCpEnthalpy};
pub use flash::{flash_isothermal, FlashResult};
pub use four_lump::{FourLump, FourLumpParams};
pub use line_flash::{EquilibriumLineFlash, NoLineFlash};
pub use molar::MoleFractions;
pub use newton_flow::NewtonFlowSolver;
pub use reactor::SimpleLookup;
pub use separation::CutPointSplitter;
pub use simple_flow::SimpleFlowSolver;
pub use thermo::{ConstantAlphaThermo, TroutonThermo};

use refinery_core::components::{Composition, Slate};
use refinery_core::error::SimError;
use refinery_core::traits::{Reaction, ReactionModel, ThermoModel};
use refinery_core::units::{JPerKg, JPerMol, Kelvin, Pascal, Seconds};

/// M1 placeholder: constant-property water; Composition's ideal-mixing
/// helpers carry properties.
///
/// The default and the only fidelity any scenario in this repo selects. It has
/// **no phase equilibrium**, so `k_value` is an `Err` and not a number — see the
/// method.
pub struct ConstantThermo;
impl ThermoModel for ConstantThermo {
    fn name(&self) -> &'static str {
        "constant"
    }

    /// Refused, deliberately.
    ///
    /// The tempting answer is `K = 1`, and it is the exact failure shape this
    /// workspace keeps catching: finite, deterministic, plausible, and wrong —
    /// a column that runs and separates nothing, which reads as a physics result
    /// rather than a missing model. `ConstantThermo` is constant-property water;
    /// it has no vapour pressure to divide by a system pressure.
    ///
    /// The pairing this protects (`separation = "cascade"` with
    /// `thermo = "constant"`) becomes a LOAD-time refusal in M7.3, when
    /// `cascade` first becomes selectable. Until then this arm is the only
    /// guard, and a unit test is the only thing that reaches it.
    fn k_value(
        &self,
        _slate: &Slate,
        _component: usize,
        _temperature: Kelvin,
        _pressure: Pascal,
    ) -> Result<f64, SimError> {
        Err(SimError::Scenario(
            "the 'constant' thermo fidelity has no vapour-liquid equilibrium, so it has \
             no K-value; select thermo = \"trouton\" for a model that does"
                .into(),
        ))
    }

    /// Refused, for `k_value`'s reason exactly.
    ///
    /// A model with no vapour phase has no heat of vaporization either, and the
    /// tempting answer here is worse than `K = 1`: a zero latent heat makes a
    /// column's condenser duty come out as pure sensible desuperheating —
    /// small, finite, and off by an order of magnitude, which is the number a
    /// game would size a cooling-water pump from.
    fn dh_vap(
        &self,
        _slate: &Slate,
        _component: usize,
        _temperature: Kelvin,
    ) -> Result<JPerMol, SimError> {
        Err(SimError::Scenario(
            "the 'constant' thermo fidelity has no vapour phase, so it has no heat of \
             vaporization; select thermo = \"trouton\" for a model that does"
                .into(),
        ))
    }

    /// Refused, and this is the arm fourteen of the fifteen shipped plants take.
    ///
    /// `SimError::Scenario` rather than `Numerical`, and the variant is
    /// load-bearing: the engine's cavitation pass reads this one as "no
    /// criterion at this node" and reports nothing, while any other variant
    /// fails the tick (docs/DESIGN.md §13). A plant on this fidelity is not
    /// broken — it simply has no thermodynamics to be graded against, and
    /// `cavitating: false` would be a clean bill of health nothing computed.
    fn bubble_pressure(
        &self,
        _slate: &Slate,
        _composition: &Composition,
        _temperature: Kelvin,
    ) -> Result<Pascal, SimError> {
        Err(SimError::Scenario(
            "the 'constant' thermo fidelity has no vapour-liquid equilibrium, so it has \
             no bubble pressure; select thermo = \"trouton\" for a model that does"
                .into(),
        ))
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

#[cfg(test)]
mod constant_thermo_tests {
    use super::*;
    use refinery_core::components::Slate;
    use refinery_core::units::Kelvin;

    /// The fidelity fourteen of the fifteen shipped plants select has no bubble
    /// pressure, and refuses with the variant that means "this configuration
    /// cannot answer".
    ///
    /// **The variant is the assertion.** `SimError::Scenario` is what the
    /// engine's cavitation pass reads as "no criterion at this node"; any other
    /// variant fails the tick. A refusal that returned `Numerical` here would
    /// stop every `thermo = "constant"` plant in the corpus from ticking
    /// (docs/DESIGN.md §13, "Corrections from building it").
    #[test]
    fn the_constant_fidelity_refuses_a_bubble_pressure_as_a_scenario_error() {
        let slate = Slate::water_only();
        let err = ConstantThermo
            .bubble_pressure(&slate, &Composition::pure(slate.len(), 0), Kelvin(300.0))
            .expect_err("constant-property water has no vapour-liquid equilibrium");
        assert!(
            matches!(err, SimError::Scenario(_)),
            "must refuse as `Scenario`, the variant the engine reads as 'no criterion': {err}"
        );
    }
}
