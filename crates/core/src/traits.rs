//! Solver traits — the fidelity seam.
//!
//! `core` defines WHAT must be computed; `refinery-solvers` provides HOW,
//! in simple (game) and complex (research) variants. Scenario config picks
//! implementations at engine build time. Shared engine code must never
//! branch on fidelity.

use crate::components::{Composition, Slate};
use crate::error::SimError;
use crate::graph::{ColumnDraw, EdgeId, NodeId, PlantGraph};
use crate::units::{JPerKg, Kelvin, KgPerSec, Pascal, Seconds, Watt};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Result of one hydraulic solve: node pressures + branch mass flows.
/// BTreeMap for deterministic iteration and stable serialization.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HydraulicSolution {
    pub node_pressure: BTreeMap<NodeId, Pascal>,
    /// Positive = flow in edge direction (source → target), kg/s.
    pub edge_mass_flow: BTreeMap<EdgeId, f64>,
    /// Power friction dissipates into the stream on each edge [W], always ≥ 0.
    ///
    /// **The solver reports this because `core` must not compute it.** The rule
    /// is `Φ = α·Q|Q|·Q`, and the split it rests on — `α` is dissipative, `β`
    /// (elevation head, pump jump) is not — is a fact about `QuadraticBranch`,
    /// which lives in `solvers`. Re-deriving `ΔP_fric` here as
    /// `(P_up − P_down) − β` would put element physics in `core` as plainly as a
    /// fidelity `if` would (CLAUDE.md rule 2, docs/DESIGN.md §3a). So it crosses
    /// the seam as data, and `core` consumes it exactly as it consumes
    /// `edge_mass_flow`.
    ///
    /// A device folds into its outlet edge (fold-at-source), so a valve's or a
    /// pump's own friction appears on the edge LEAVING it, not on the node.
    pub edge_dissipation: BTreeMap<EdgeId, Watt>,
    pub diagnostics: SolveDiagnostics,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SolveDiagnostics {
    pub iterations: u32,
    pub residual: f64,
    pub converged: bool,
}

/// Computes the quasi-steady pressure/flow field for the current network
/// state. Must be pure w.r.t. the graph (read-only); the engine applies
/// the solution to streams afterwards.
pub trait FlowSolver: Send {
    /// `previous_states` is the PREVIOUS tick's resolved node temperatures and
    /// compositions — empty on the first tick, before any sweep has run.
    ///
    /// It exists for one reason (docs/DESIGN.md §3a fork 6): a gas edge's
    /// transport density `ρ = P·M̄/(R·T)` is evaluated at its upwind node's
    /// state, and a ZERO-VOLUME upwind node (junction, valve, exchanger side)
    /// has no temperature of its own to read. Before M5.4 such an edge fell back
    /// to the pipe's stored OUTLET temperature — its inlet plus whatever ambient
    /// exchange and frictional dissipation the pipe added — which on `gas_line`
    /// is 375.0 K against the tee's real 297.3 K, a 21% density error containing
    /// no `dt`. That is a different steady model, not a staleness; the previous
    /// tick's resolved value is an honest staleness that shrinks with the step,
    /// and is the same lag §3 already accepts for the tank levels feeding a
    /// quasi-steady solve.
    ///
    /// A solve must never MUTATE anything from it: like `graph`, it is read-only
    /// input, and the engine overwrites it wholesale after the sweep.
    fn solve(
        &mut self,
        graph: &PlantGraph,
        slate: &Slate,
        previous_states: &crate::energy::NodeStates,
        dt: Seconds,
    ) -> Result<HydraulicSolution, SimError>;

    /// Human-readable identifier for snapshots/logs (e.g. "newton-network").
    fn name(&self) -> &'static str;
}

/// Physical property provider. M1 uses constant-property water; M2+ uses
/// composition/temperature-dependent models. Kept minimal on purpose —
/// extend when a consumer actually needs a property, not before.
///
/// **M7.1 gave it a consumer; M7.2 gives it its first method.**
/// `SeparationModel::separate` takes `&dyn ThermoModel`, so the slot reserved on
/// `Engine` since M1 is finally read and reaches `solvers` — and `k_value` is
/// the property that consumer actually needs, which is the condition the
/// paragraph above sets (DESIGN §5, fork 2).
pub trait ThermoModel: Send {
    fn name(&self) -> &'static str;

    /// The vapour–liquid equilibrium ratio of one component at `(T, P)`:
    /// `K_c = y_c / x_c`, dimensionless.
    ///
    /// **`x` and `y` are MOLE fractions**, which is the whole reason fork 1
    /// makes the cascade's internal state molar and converts at the unit's
    /// boundary. A `Composition` in this workspace is mass fractions, and
    /// handing one to this number without converting is the slip M4.2's real
    /// crux (units, not the ODE) says to expect.
    ///
    /// A component is named by its **index into `slate`**, not by a
    /// `&PseudoComponent`, because that is the identity a per-component
    /// parameter vector is resolved against — `ConstantAlphaThermo` carries one
    /// K per slate position, the same shape `SimpleLookup::fcc_demo(&slate)`
    /// resolves at construction. A reference to the component alone would force
    /// every such model to look itself up by name on the hot path.
    ///
    /// The anchor every **correlation** shares: `K = 1` at `T = tb` and
    /// `P = P_ATM`, because `tb` is the NORMAL boiling point. That identity is
    /// what makes the property testable without a published table — and, per
    /// DESIGN §5 correction 4, it is *structurally incapable* of detecting a
    /// wrong empirical constant in the magnitude of `K` away from that anchor,
    /// which is why a correlation needs a separate envelope gate.
    ///
    /// It is not a trait invariant, and one implementation opts out on purpose:
    /// a model that returns K-values **supplied by a test** (`ConstantAlphaThermo`)
    /// exists precisely so a separation gate can be written against algebra with
    /// no correlation in it, so requiring it to honour a boiling-point anchor
    /// would defeat what it is for. A model that claims to compute `K` from
    /// physical properties is expected to hit the anchor; a model that is handed
    /// its numbers is not.
    ///
    /// # Errors
    /// `SimError` if the state is outside what the model can evaluate — a
    /// non-positive or non-finite `T` or `P`, a component index off the end of
    /// the slate, or a fidelity that has no vapour–liquid equilibrium at all
    /// (`ConstantThermo`, which is every scenario before M7.2). Rule 5: a model
    /// that cannot answer says so, and never returns a plausible number.
    fn k_value(
        &self,
        slate: &Slate,
        component: usize,
        temperature: Kelvin,
        pressure: Pascal,
    ) -> Result<f64, SimError>;

    // Density/cp currently live on Composition (ideal mixing). This trait
    // takes over when non-ideal or T-dependent behavior arrives, at which
    // point Composition's mixture_* helpers delegate here.
}

/// One draw's separation result: the fraction of the feed mass it takes, the
/// composition that fraction carries, and the temperature it leaves at.
#[derive(Debug, Clone)]
pub struct DrawSeparation {
    /// The mass fraction of the feed that leaves by this draw. `Σᵢ splitᵢ = 1`,
    /// which is what makes a column mass-neutral every tick with no holdup.
    pub split: f64,
    /// This draw's composition (mass fractions, `Σ = 1`).
    pub composition: Composition,
    /// The temperature this draw leaves at [K].
    ///
    /// The cut-point splitter returns the feed temperature it was handed, which
    /// is exactly what the engine does today — a column is a swept zero-volume
    /// node and every draw reads it as their upwind end. A cascade's draws leave
    /// at their **tray** temperatures instead, which differ per draw.
    ///
    /// **Nothing reads this field yet** (M7.1 changes no physics): the value is
    /// carried so that M7.4 can wire `energy::edge_temperature_at`'s column arm
    /// without churning the trait. Until then the splitter's copy and the sweep's
    /// mixed value are the same number by construction, and
    /// `a_draw_leaves_at_the_feed_temperature_it_was_handed` is the test that
    /// pins that agreement.
    pub temperature: Kelvin,
}

/// The outcome of one column pass: every draw's split, plus the column's two
/// heat duties.
#[derive(Debug, Clone)]
pub struct Separation {
    /// One entry per `ColumnDraw`, in the same order.
    pub draws: Vec<DrawSeparation>,
    /// Heat REMOVED at the condenser [W], a non-negative magnitude.
    ///
    /// The `Furnace`/`Cooler` convention, not a signed duty: which way a named
    /// piece of equipment moves heat is a property of the equipment, so storing
    /// it signed would make a condenser that heats representable. Both duties are
    /// emergent DIAGNOSTICS like `energy::ReactorDuty` — nothing in the forward
    /// solve is driven by them — and the energy gate M7.4 owes is their
    /// DIFFERENCE against the sensible external balance, never either alone
    /// (M4's two-duty lesson, DESIGN §5).
    pub condenser_duty: Watt,
    /// Heat ADDED at the reboiler [W], a non-negative magnitude. See
    /// `condenser_duty`.
    pub reboiler_duty: Watt,
}

/// Everything one column pass is a function of: the equipment, and the feed
/// state that reaches it this tick.
///
/// A struct rather than eight positional arguments, and that is a scoping
/// decision as much as a readability one: M7.3's cascade config (stage count,
/// feed stage, reflux ratio, `D/F`) lands here as further fields under the
/// declared-iff-used correspondence, and an impl that ignores them needs no
/// edit — the same trait-churn argument correction 2 makes about `thermo`.
pub struct ColumnPass<'a> {
    /// The canonical component slate; `feed`'s fractions index into it.
    pub slate: &'a Slate,
    /// The column's draws in ascending boiling-point order. The returned
    /// `Separation::draws` is parallel to this.
    pub draws: &'a [ColumnDraw],
    /// Ramp width across each cut point [K] (`NodeKind::Column::smearing`).
    pub smearing: Kelvin,
    /// The column's pinned operating pressure [Pa]. Ignored by the splitter,
    /// load-bearing for a cascade's K-values.
    pub pressure: Pascal,
    /// The feed composition resolved by this tick's sweep — never a holdup:
    /// a holdup mixes to one composition and separates nothing (DESIGN §5).
    pub feed: &'a Composition,
    /// Total mass entering the column this tick [kg/s]. Ignored by the splitter
    /// (a fraction is a fraction), load-bearing for a cascade's internal flows.
    ///
    /// This is the sweep's inflow sum, so a column running BACKWARDS reports
    /// `0` here rather than a negative number — `Engine::tick` owns the
    /// reverse-feed refusal and fires it after the sweep, with the diagnostic
    /// that names the cause. Harmless today because the splitter never reads
    /// this; M7.3 must revisit it, because a cascade WOULD solve on the zero
    /// and fail with a worse message before that guard is reached.
    pub feed_flow: KgPerSec,
    /// The feed's resolved temperature [K].
    pub temperature: Kelvin,
}

/// How a column divides its feed among its draws — the separation seam.
///
/// The fidelity split this trait exists for: `CutPointSplitter` (M3.2's boiling
/// -range splitter, moved here verbatim in M7.1) and the M7.3 stage cascade are
/// two implementations of one contract, selected by
/// `[fidelity] separation` in scenario TOML. `NodeKind::Column` is unchanged
/// between them — **the complex column is a different `SeparationModel`, not a
/// different plant unit** (DESIGN §5, fork 1).
///
/// **Called once per column per tick**, from the composition sweep, and the
/// result is stored in `energy::NodeStates::column_separation` for its two
/// consumers: `energy::edge_composition_at` (which draw carries what) and
/// `Engine::tick`'s post-sweep draw write (how much each draw carries). This is
/// the `ReactionModel` precedent exactly — `react` runs once per reactor rather
/// than once per outlet edge — and it is what keeps the flow split and the
/// composition split derived from the SAME pass, which per-component
/// conservation at a zero-volume column requires.
pub trait SeparationModel: Send {
    fn name(&self) -> &'static str;

    /// Split one column's feed among its draws.
    ///
    /// Pure: a function of `pass` alone, with no tick history. That is what keeps
    /// a column's reference a clean hand calculation, and it is a contract, not an
    /// implementation note — a cascade may WARM-START from a previous profile
    /// (that changes the iteration count) but must never let one change the
    /// answer (DESIGN §5, fork 5).
    ///
    /// `thermo` is unused by the cut-point splitter, which separates on the
    /// slate's boiling points alone; the cascade reads K-values off it (M7.2).
    ///
    /// # Errors
    /// `SimError` if the split cannot be produced — an invalid draw composition,
    /// or (from M7.3) a cascade that does not converge. A non-converged solve is
    /// an `Err`, never a held previous profile.
    fn separate(
        &self,
        pass: &ColumnPass<'_>,
        thermo: &dyn ThermoModel,
    ) -> Result<Separation, SimError>;
}

/// The outcome of one reactor pass: the product composition and its heat of
/// reaction.
#[derive(Debug, Clone)]
pub struct Reaction {
    /// Product mass-fraction composition (Σ = 1). TOTAL mass is conserved — the
    /// reaction moves mass BETWEEN components, which is exactly what makes a
    /// reactor the first unit to break per-component conservation (I7).
    pub products: Composition,
    /// Specific heat of reaction [J per kg of feed]; positive = endothermic
    /// (heat absorbed). The reactor's reported physical duty adds `ṁ·Δh_rxn`
    /// to the emergent sensible duty (`energy::reactor_duty`).
    pub dh_rxn: JPerKg,
}

/// Chemical conversion inside reactor units. Implementations: `NoReactions`
/// (identity), `SimpleLookup` (conversion table), FCC 4-lump kinetics (M4.2).
///
/// The engine applies `react` INSIDE the energy sweep, so a downstream node
/// sees the product composition the same tick (DESIGN §5).
pub trait ReactionModel: Send {
    fn name(&self) -> &'static str;

    /// Convert one reactor pass's FEED into products at the reactor's held
    /// temperature `temperature` over residence time `tau`.
    ///
    /// Isothermal at a ROT setpoint (DESIGN §5): `temperature` is imposed by the
    /// reactor, so the extent is a pure function of a KNOWN temperature with no
    /// inner temperature solve. `tau` is unused by the lookup fidelity but is
    /// load-bearing for the M4.2 kinetics, which integrate `dC/dτ` over it — it
    /// is in the signature now so that additive swap needs no trait churn.
    ///
    /// `NoReactions` returns the feed unchanged with `Δh_rxn = 0`, so a network
    /// with no reactor node — every scenario before M4 — stays bit-identical.
    ///
    /// # Errors
    /// `SimError` if the model cannot produce a valid product composition (e.g.
    /// a table whose row does not normalize, or a feed lump it does not know).
    fn react(
        &self,
        feed: &Composition,
        temperature: Kelvin,
        tau: Seconds,
        slate: &Slate,
    ) -> Result<Reaction, SimError>;
}
