//! Solver traits — the fidelity seam.
//!
//! `core` defines WHAT must be computed; `refinery-solvers` provides HOW,
//! in simple (game) and complex (research) variants. Scenario config picks
//! implementations at engine build time. Shared engine code must never
//! branch on fidelity.

use crate::components::{Composition, Slate};
use crate::error::SimError;
use crate::graph::{CascadeSpec, ColumnDraw, ControlledValue, EdgeId, NodeId, PlantGraph};
use crate::units::{JPerKg, JPerKgK, JPerMol, Kelvin, Kg, KgPerSec, Pascal, Seconds, Watt};
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

/// The specific ENTHALPY of a mixture, and everything derived from it — the
/// heat-capacity seam (M16.2, docs/DESIGN.md §20).
///
/// **What this trait owns is `h(T)`, not `cp`** (§20 fork 1). Both capacities are
/// derived from it: the spot value is `dh/dT`, the mean over an interval is the
/// difference quotient `(h(T₂) − h(T₁))/(T₂ − T₁)`. The alternative — supply
/// `cp(T)` and let each consumer integrate — is refused for the reason
/// `mixture_cv` and `cp − R/M̄` are gated against each other in `components.rs`:
/// two expressions of one quantity drift, and here the thing that would drift is
/// the datum `energy::T_REF` that §4a's reference cancellation rests on. With one
/// function that IS the integral, a consumer cannot integrate it from the wrong
/// end; with a capacity and a convention, sixteen call sites can each get it
/// wrong independently.
///
/// **Why the flux, the stock, the mix and both inversions are methods here rather
/// than free functions in `energy` taking an `h`.** They are algebraically
/// `ṁ·h`, `m·h`, `T(Σṁh/Σṁ)` and `T(U/m)` — but algebra is not bits.
/// `enthalpy_flux` was `(ṁ·cp)·(T − T_REF)` for fifteen milestones, and
/// regrouping it to `ṁ·(cp·(T − T_REF))` — with no shape, no seam and no physics
/// change whatever — moves **11 of the 19** shipped plants on each fidelity, and
/// not the same 11 (measured, M16.2). Fork 4 promises the constant model
/// reproduces today's arithmetic *bit for bit*; the only way to keep that promise
/// is to let the model own the grouping. `ConstantEnthalpy` therefore holds the
/// pre-M16 expressions verbatim, and a gate asserts the two forms agree to a few
/// ULP so the override cannot hide a real disagreement.
///
/// **Pure, and a function of its arguments alone**, like every seam in this file
/// except `Controller`: the engine holds ONE model for the whole plant, so state
/// on `self` would cross-seed two holdups.
///
/// **`T_REF` is not a parameter.** Every method integrates from
/// `energy::T_REF` and an implementation that picks its own datum is wrong, not
/// configurable — which is what gate 5 asserts against the constant explicitly
/// rather than relying on a zero reference to make it moot.
pub trait EnthalpyModel: Send {
    fn name(&self) -> &'static str;

    /// Specific enthalpy `h(T) = ∫_{T_REF}^{T} cp(τ) dτ` [J/kg], on the
    /// saturated-liquid datum M13 settled (`energy::T_REF`).
    ///
    /// # Errors
    /// `SimError::Numerical` when the mixture has no evaluable shape at this
    /// state — never a NaN (rule 5).
    fn specific_enthalpy(
        &self,
        slate: &Slate,
        composition: &Composition,
        temperature: Kelvin,
    ) -> Result<JPerKg, SimError>;

    /// The enthalpy flux `ṁ·h(T)` [W] carried by a mass flow.
    ///
    /// Sign follows `mass_flow`, exactly as the free function it replaces did:
    /// the caller passes flow *into* the node it is accounting for, so an outflow
    /// subtracts. **The SENSIBLE term alone** — a caller holding a whole
    /// `Stream` wants `stream_enthalpy_flux` below, which adds `λ`.
    ///
    /// # Errors
    /// As `specific_enthalpy`.
    fn enthalpy_flux(
        &self,
        slate: &Slate,
        composition: &Composition,
        mass_flow: KgPerSec,
        temperature: Kelvin,
    ) -> Result<Watt, SimError>;

    /// The TOTAL enthalpy flux a stream carries, `ṁ·(h(T) + λ)` [W].
    ///
    /// The single owner of "a stream's specific enthalpy on this engine's datum",
    /// and the reason `Stream::latent` is a field on the stream rather than a
    /// lookup keyed by edge id (§15 fork 1): a consumer closing an energy balance
    /// calls this and never has to know which edges carry vapour.
    ///
    /// **Provided, not required, and that is a measurement rather than a
    /// convenience.** The pre-M16 expression was already `ṁ·(cp·(T − T_REF) + λ)`
    /// — the `ṁ·h` grouping this trait wants — so composing it out of
    /// `specific_enthalpy` reproduces it bit for bit with no override, which the
    /// flux and the stock above cannot do.
    ///
    /// # Errors
    /// As `specific_enthalpy`.
    fn stream_enthalpy_flux(
        &self,
        slate: &Slate,
        stream: &crate::stream::Stream,
    ) -> Result<Watt, SimError> {
        let h = self
            .specific_enthalpy(slate, &stream.composition, stream.temperature)?
            .value();
        let latent = stream.latent.map_or(0.0, |l| l.value());
        Ok(Watt(stream.mass_flow.value() * (h + latent)))
    }

    /// A holdup's enthalpy STOCK `m·h(T)` [J].
    ///
    /// The same expression as the flux with a mass in place of a mass rate, and a
    /// method of its own rather than a units lie at the call site: an inventory is
    /// not a flow, and `engine.rs`'s tank branch is the second site in the engine
    /// that holds the datum (§20's consumer (a)).
    ///
    /// # Errors
    /// As `specific_enthalpy`.
    fn enthalpy_stock(
        &self,
        slate: &Slate,
        composition: &Composition,
        mass: Kg,
        temperature: Kelvin,
    ) -> Result<f64, SimError>;

    /// The mean capacity over `[t1, t2]`: `(h(t2) − h(t1))/(t2 − t1)`
    /// [J/(kg·K)].
    ///
    /// **At `t2 == t1` this returns the SPOT value `cp(t1)`**, which is the
    /// continuous limit and therefore a right answer rather than a convenient one
    /// — the same standing as `pipe_outlet_temperature`'s zero-flow return of its
    /// inlet (§20 fork 7). Exact equality, not a threshold, for that function's
    /// own stated reason: a threshold is a magic number that would also flatten
    /// legitimately small intervals. The state is reachable rather than
    /// hypothetical — a tank sitting at ambient, a draw at its own tray
    /// temperature, a stagnant edge.
    ///
    /// # Errors
    /// As `specific_enthalpy`.
    fn mean_cp(
        &self,
        slate: &Slate,
        composition: &Composition,
        t1: Kelvin,
        t2: Kelvin,
    ) -> Result<JPerKgK, SimError>;

    /// The SPOT capacity `cp(T) = dh/dT` [J/(kg·K)].
    ///
    /// The third consumer, and the one this project keeps conflating with the
    /// second (§20's enumeration): `γ = cp/cv` is a ratio at ONE temperature, and
    /// so is the derivative an inverter needs. A mean over an interval is a
    /// different number, and answering one question with the other is the error
    /// mutation 1 exists to catch.
    ///
    /// # Errors
    /// As `specific_enthalpy`.
    fn spot_cp(
        &self,
        slate: &Slate,
        composition: &Composition,
        temperature: Kelvin,
    ) -> Result<JPerKgK, SimError>;

    /// Specific internal energy `u(T) = h(T) − (R/M̄)·T` [J/kg], on the same
    /// datum.
    ///
    /// **Well formed under any shape, which is §20's correction to §18.** The
    /// offset `R/M̄` is a property of the mixture's molar mass and is temperature
    /// independent (`Composition::gas_constant_offset`), so `cv(T) = cp(T) − R/M̄`
    /// holds exactly whatever `cp` does. With `cp` flat this evaluates to M5.3's
    /// `cv·T − cp·T_REF` — and, in `ConstantEnthalpy`, to those very bits.
    ///
    /// # Errors
    /// As `specific_enthalpy`.
    fn specific_internal_energy(
        &self,
        slate: &Slate,
        composition: &Composition,
        temperature: Kelvin,
    ) -> Result<JPerKg, SimError>;

    /// Invert the enthalpy stock: the temperature [K] a holdup of `mass` ends a
    /// step at, holding `energy` joules on the enthalpy datum.
    ///
    /// **Takes `(energy, mass)` rather than a pre-divided `h`**, and that is the
    /// signature doing the work rather than the body: the constant model needs
    /// `T_REF + energy/(mass·cp)` — one division by a product — to reproduce the
    /// pre-M16 tank branch exactly, and a caller that had already divided could
    /// not give it back.
    ///
    /// # Errors
    /// `SimError::Numerical` when no temperature answers — a mixture whose shape
    /// has no root at this energy, which is `h` leaving the range its declared
    /// shape is monotone over.
    fn temperature_from_enthalpy(
        &self,
        slate: &Slate,
        composition: &Composition,
        energy: f64,
        mass: f64,
    ) -> Result<Kelvin, SimError>;

    /// Invert `specific_internal_energy` for a holdup: the temperature [K] a
    /// vessel of `mass` ends a step at, holding `energy` joules of INTERNAL
    /// energy.
    ///
    /// Kept beside its forward form for the reason M5.3 gave: a balance must not
    /// integrate on one datum and read back on another.
    ///
    /// # Errors
    /// As `temperature_from_enthalpy`.
    fn temperature_from_internal_energy(
        &self,
        slate: &Slate,
        composition: &Composition,
        energy: f64,
        mass: f64,
    ) -> Result<Kelvin, SimError>;

    /// The temperature [K] a zero-volume node's inflows mix to.
    ///
    /// **Three sums arrive and the two models divide different pairs**, which is
    /// why this is a method rather than a division at the call site. The mixed
    /// specific enthalpy is `Σṁh/Σṁ` and the answer is its inverse; the constant
    /// model instead keeps `T_REF + Σṁh/Σṁcp`, the expression `mix_inflows` has
    /// evaluated since M2.1. The two are the same number and not the same bits.
    ///
    /// # Errors
    /// As `temperature_from_enthalpy`.
    fn mix_temperature(
        &self,
        slate: &Slate,
        composition: &Composition,
        totals: InflowEnthalpy,
    ) -> Result<Kelvin, SimError>;
}

/// The three sums a zero-volume node's inflows produce, carried together because
/// `EnthalpyModel::mix_temperature`'s two implementations divide different pairs
/// of them (see that method).
///
/// `capacity_rate` is also read on its own, by the exchanger, to size `C_min` —
/// the second reader §20's consumer enumeration does not name.
#[derive(Debug, Clone, Copy)]
pub struct InflowEnthalpy {
    /// `Σ ṁ·h` [W].
    pub enthalpy_rate: f64,
    /// `Σ ṁ` [kg/s].
    pub mass_rate: f64,
    /// `Σ ṁ·cp` [W/K], each term at its own inlet's SPOT capacity.
    pub capacity_rate: f64,
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

    /// The heat of vaporization of one component [J/mol] — the latent heat a
    /// condenser removes and a reboiler supplies.
    ///
    /// **Per MOLE, and `temperature` is here for a fidelity that does not exist
    /// yet.** The molar basis is fork 1's mass ⇄ mole boundary (a duty multiplies
    /// a molar flow); the parameter is the `ReactionModel::react`/`tau`
    /// precedent, so a Watson-style `Δh_vap(T)` is an additive swap rather than
    /// trait churn.
    ///
    /// **The identity that binds this to `k_value`, and it is not optional for a
    /// correlation.** A model whose `k_value` integrates Clausius–Clapeyron is
    /// *already committed* to a `Δh_vap`, because that constant is what it
    /// integrated:
    ///
    /// ```text
    /// d ln K / d(1/T) = −Δh_vap / R          (at fixed P)
    /// ```
    ///
    /// So `dh_vap` returning anything else would make one model contradict
    /// itself, and a cascade would compute its profile on one latent heat and its
    /// duties on another. This is the gate M7.4b adds, and unlike the `K = 1` at
    /// `(tb, P_ATM)` anchor it **does** see an empirical constant — which is what
    /// DESIGN §5 correction 4 says the identities could not do.
    ///
    /// As with `k_value`, a model handed its numbers by a test
    /// (`ConstantAlphaThermo`) is not held to the identity: it has no correlation
    /// to be consistent with. It is held to something narrower — it must `Err`
    /// rather than invent a latent heat it was never given.
    ///
    /// # Errors
    /// `SimError` as `k_value`: a non-positive or non-finite `T`, a component
    /// index off the end of the slate, or a fidelity with no latent heat at all
    /// (`ConstantThermo`). Rule 5: never a plausible number.
    fn dh_vap(
        &self,
        slate: &Slate,
        component: usize,
        temperature: Kelvin,
    ) -> Result<JPerMol, SimError>;

    /// The **bubble pressure** of a liquid mixture at `temperature` [Pa]: the
    /// pressure at which the first bubble comes out of a liquid of this
    /// composition, and therefore the pressure below which that liquid is
    /// boiling (docs/DESIGN.md §13).
    ///
    /// `composition` is MASS fractions, like every `Composition` in this
    /// workspace; a model needing mole fractions converts at its own boundary
    /// (§5 fork 1). Nothing here is a graph quantity — this is a property
    /// lookup, the shape `k_value` and `dh_vap` already have.
    ///
    /// **Why a method rather than `P·Σ x_c·K_c(T, P)` at the call site.** That
    /// identity is exact — and exact only for a model whose `K` is inversely
    /// proportional to pressure, which is Raoult's law and a property of
    /// `TroutonThermo` rather than of this trait. Writing it into `core` would
    /// put a model assumption in `core` as plainly as a fidelity `if` would
    /// (CLAUDE.md rule 2), and it would silently return the wrong number for any
    /// future `K` that is not `Psat/P`. It also needs mole fractions, which
    /// `docs/DEFERRED.md` A13 keeps in `solvers`.
    ///
    /// **A closed form, unlike the bubble TEMPERATURE.** `Σ K_c(T)·x_c = 1`
    /// solved for `T` is a root find that cost M9.3a a whole slice; solved for
    /// `P` it is a weighted sum. That is what makes a per-tick criterion
    /// affordable (§13 fork 2).
    ///
    /// # Errors
    /// Two kinds, and a caller must tell them apart (§13, "Corrections from
    /// building it"):
    /// - **`SimError::Scenario` — this fidelity has no vapour–liquid
    ///   equilibrium**, so there is no bubble pressure to give. A legitimate
    ///   configuration: every plant on `thermo = "constant"` is in it. The
    ///   engine reads it as "no criterion at this node" and reports nothing,
    ///   exactly as it reports no `column_duty` for a fidelity with no
    ///   condenser.
    /// - **Every other variant — a genuine fault** (a non-positive or
    ///   non-finite temperature, a composition the slate cannot interpret).
    ///   It propagates and fails the tick, per rule 5.
    ///
    /// This is the same distinction `SimError::AnchoringUnsettled` exists for,
    /// and it is a variant rather than a substring for the same reason.
    ///
    /// A model that is HANDED its K-values refuses too, and for a sharper
    /// reason than "it has no correlation": if `K` does not depend on pressure
    /// then `P·Σ x·K` depends on which `P` it is evaluated at, so there is no
    /// bubble pressure to return.
    fn bubble_pressure(
        &self,
        slate: &Slate,
        composition: &Composition,
        temperature: Kelvin,
    ) -> Result<Pascal, SimError>;

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
    /// **Read since M7.4a**, by `energy::edge_temperature_at`'s column arm — the
    /// field was carried unread from M7.1 so that arm needed no trait churn. On
    /// the splitter path the value and the sweep's mixed temperature are still
    /// the same number by construction, which is why wiring the reader left every
    /// scenario byte-identical; `a_draw_leaves_at_the_feed_temperature_it_was_
    /// handed` is what pins that agreement.
    ///
    /// One lookup serves this field and `composition` both
    /// (`energy::column_draw_at`), so a draw can never be handed its own
    /// composition at another draw's temperature.
    pub temperature: Kelvin,
}

/// One column's converged iterate, kept so a LATER tick can start from it.
///
/// # Both halves, or neither — and that is a measurement, not symmetry
///
/// A stage cascade iterates two profiles at once: the stage temperatures and the
/// stage liquid compositions. Its convergence test is a CONJUNCTION over both,
/// so seeding one and not the other leaves the unseeded half walking in from
/// cold every tick and the solve is barely faster. M9.3b measured exactly that
/// on `crude_column_cascade` over 6 000 ticks: seeding the temperatures alone
/// took 38.0 → 35.0 outer iterations per solve, an 8% saving that vanished into
/// the noise of a wall-clock measurement. Seeding both took it to **1.006**.
///
/// That is why this is one struct behind one `Option` rather than two `Option`
/// fields on [`Separation`]. Two options would make "temperatures without
/// compositions" representable, and that state is not a partial warm start — it
/// is the configuration measurement has already falsified. Same argument, and
/// same shape, as `condenser_duty` and `reboiler_duty`: a model that knows one
/// knows both.
///
/// # It is a hint, and nothing may read it as an answer
///
/// Fork 5 permits a warm start because it changes the iteration count and not
/// the fixed point. Nothing downstream consumes this: it is absent from
/// `Snapshot`, no frontend sees it, and its only reader is the next tick's
/// [`ColumnPass::seed`]. A model must return the same answer within tolerance
/// whether it gets one, gets a wrong one, or gets none.
#[derive(Debug, Clone)]
pub struct CascadeProfile {
    /// Stage temperatures [K], one per stage, in the model's own stage order.
    pub temperatures: Vec<Kelvin>,
    /// Stage liquid MOLE fractions: one row per stage, each row indexed by the
    /// slate's component order and summing to one.
    ///
    /// **A bare `f64` here is deliberate and is the one place this file argues
    /// for one.** Rule 4 makes an un-newtyped `f64` crossing a crate boundary a
    /// review failure, and the rule is about physical quantities whose unit a
    /// reader could get wrong. A mole fraction has no unit, `solvers` already has
    /// the newtype that owns the invariant (`molar::MoleFractions`, which is
    /// where `from_amounts` enforces normalisation), and `core` must not depend
    /// on `solvers` to name it. Moving that type down into `core` would be the
    /// alternative; it is a wider change than this slice, and it buys nothing
    /// here, because `core` never interprets this field — it stores the value the
    /// model returned and hands the same value back. See `docs/DEFERRED.md`.
    pub liquid: Vec<Vec<f64>>,
}

/// The outcome of one column pass: every draw's split, plus the column's two
/// heat duties.
#[derive(Debug, Clone)]
pub struct Separation {
    /// One entry per `ColumnDraw`, in the same order.
    pub draws: Vec<DrawSeparation>,
    /// Heat REMOVED at the condenser [W], a non-negative magnitude, or `None`
    /// from a fidelity that has no condenser to speak of.
    ///
    /// The `Furnace`/`Cooler` convention, not a signed duty: which way a named
    /// piece of equipment moves heat is a property of the equipment, so storing
    /// it signed would make a condenser that heats representable. Both duties are
    /// emergent DIAGNOSTICS like `energy::ReactorDuty` — nothing in the forward
    /// solve is driven by them.
    ///
    /// **`Option` rather than a zero, and M7.4b is when that stopped being a
    /// comment.** Through M7.3 both fidelities returned `Watt::ZERO` and the two
    /// zeros meant different things: for `CutPointSplitter` it is a gap (a
    /// boiling-range split has no trays, no boilup, nothing to compute a duty
    /// from), for the cascade it was "M7.4 has not landed yet". Once the cascade
    /// computes real duties, keeping the splitter's zero would publish a number
    /// no model produced — and a frontend sizing cooling water off a reported
    /// `0 W` is the finite-deterministic-plausible-wrong shape this workspace
    /// keeps catching. `None` says "this fidelity does not answer that", which
    /// is what `NodeSnapshot::column_duty` then does not serialize.
    ///
    /// A cascade with nothing flowing reports `Some(ZERO)`, not `None`: an idle
    /// column really does have zero duty, and that is an answer.
    pub condenser_duty: Option<Watt>,
    /// Heat ADDED at the reboiler [W], a non-negative magnitude, or `None`. See
    /// `condenser_duty`.
    ///
    /// The two are always `Some` together or `None` together — a model that
    /// knows one knows both, since M7.4b derives this one from the other plus
    /// the column's external sensible balance (`StageCascade::duties`).
    pub reboiler_duty: Option<Watt>,
    /// This pass's converged iterate, for the next tick to start from, or `None`
    /// from a fidelity that iterates nothing (the cut-point splitter) and from a
    /// pass that did not converge to anything worth reusing. See
    /// [`CascadeProfile`], which argues why both of its halves travel together.
    ///
    /// A reader who wants a tray temperature wants `DrawSeparation::temperature`
    /// — a draw's real temperature, which is published. This is an iterate.
    ///
    /// It rides `Separation` rather than engine state because the previous tick's
    /// `NodeStates` is already threaded into the sweep that makes this one, so
    /// the warm start costs no new state and gets per-column keying from the
    /// `BTreeMap` the results already live in.
    pub profile: Option<CascadeProfile>,
}

/// Everything one column pass is a function of: the equipment, and the feed
/// state that reaches it this tick.
///
/// A struct rather than eight positional arguments, and that is a scoping
/// decision as much as a readability one: M7.3's cascade config landed here as a
/// further field (`cascade`) under the declared-iff-used correspondence, and the
/// splitter needed no edit to ignore it — the same trait-churn argument
/// correction 2 makes about `thermo`. `D/F` did NOT land here: it is per-draw,
/// so it rides `ColumnDraw::draw_ratio` beside the `upper_cut` it replaces.
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
    /// The column's equilibrium-stage equipment, present iff the cascade fidelity
    /// is selected (`NodeKind::Column::cascade`). Ignored by the splitter; the
    /// cascade refuses a column that has none.
    pub cascade: Option<&'a CascadeSpec>,
    /// The previous tick's converged iterate for THIS column, when there is one —
    /// `Separation::profile` from the last pass, or `None` on the first tick,
    /// after a load, or from a fidelity that publishes none.
    ///
    /// **The seam stays pure because the history arrives HERE.** `separate` is
    /// contracted a function of `pass` alone, and a warm start is tick history,
    /// so the history is made an argument rather than hidden in the model. Three
    /// things follow that would not hold if the model held it: the engine-wide
    /// `Box<dyn SeparationModel>` singleton stays stateless while two columns on
    /// one plant keep separate profiles; a test can hand `separate` a deliberately
    /// wrong seed with no stateful model to build; and the contract sentence above
    /// `separate` stays literally true.
    ///
    /// An implementation MUST treat this as a hint it is free to ignore — wrong
    /// length, wrong plant, absurd values — and must return the same fixed point
    /// within tolerance whatever it holds. `separation_is_start_insensitive`
    /// gates exactly that.
    pub seed: Option<&'a CascadeProfile>,
}

/// What one tick's boil-off removes from a holdup: the vapour that formed, and
/// the temperature the liquid is left at.
///
/// A MASS and a composition rather than a rate, because the constraint that
/// produces it is an enthalpy balance over the tick rather than a rate law
/// (docs/DESIGN.md §14 fork 3). The engine divides by `dt` at the one place a
/// rate is needed — the vent edge's `mass_flow`.
#[derive(Debug, Clone)]
pub struct BoilOff {
    /// Mass vaporised over this tick [kg]. Never negative, never more than the
    /// holdup it came from.
    pub vapour_mass: Kg,
    /// What left, as MASS fractions — the equilibrium vapour `y_c = K_c·x_c`
    /// normalised, converted back at the model's own mass ⇄ mole boundary.
    ///
    /// **This field is the whole fork.** Returning the holdup's own composition
    /// here conserves mass exactly, passes I1 and I7, and leaves the tank's
    /// fractions untouched for ever — a decrement, not a flash, and no
    /// conservation test in this workspace can see the difference (§14 fork 2).
    pub vapour: Composition,
    /// What the liquid is left at [K]: its bubble temperature at the holdup's
    /// own pressure. The enthalpy constraint is stated *as* "the tank cannot be
    /// above this", so the temperature is an output of the same solve that
    /// sized `vapour_mass` and not a second answer to be derived from it.
    pub liquid_temperature: Kelvin,
    /// Latent heat per kilogram of `vapour` [J/kg] — the `Δh̄_vap` this flash
    /// was sized against, not a number to be re-derived from it.
    ///
    /// The argument is `liquid_temperature`'s, one field up, and it is stronger
    /// here: there are at least three plausible ways to recompute this
    /// downstream — at the holdup's temperature instead of its bubble point,
    /// mole-weighted instead of mass-weighted, or over the VAPOUR's fractions
    /// instead of the liquid's — and all three produce a finite, smooth,
    /// plausible number that leaves a residual reading as a bug in whoever is
    /// closing the books. The model divided by this number to size
    /// `vapour_mass`; the books have to multiply by the same one.
    ///
    /// **Specific rather than total, and that is what removes an arm.** Where a
    /// per-component cap cuts the vaporisation short of what the enthalpy
    /// constraint asked for, less mass boils and the liquid is left above its
    /// bubble point — and the energy carried is still `vapour_mass ·
    /// latent_heat` (docs/DESIGN.md §15 fork 5). One formula, both branches.
    ///
    /// Over the liquid's mass fractions at the bubble point, because that is
    /// what the flash fraction `c̄p·ΔT/Δh̄_vap` was divided by. Weighting by the
    /// vapour's fractions would be more defensible thermodynamically and would
    /// stop the balance closing: the invariant checks bookkeeping consistency,
    /// not thermodynamic virtue, and if the two ever disagree the fix belongs
    /// in the model rather than in the term the vent carries.
    pub latent_heat: JPerKg,
}

/// Whether a liquid holdup above its bubble point boils off, and how much —
/// the M12 fidelity seam (docs/DESIGN.md §14, `[fidelity] boiloff`).
///
/// **A key rather than a behaviour, and the argument is the user's own** (§14
/// fork 9): two models that are wrong in different ways, with neither a
/// refinement of the other. `NoBoilOff` says a product tank is a liquid store
/// whose contents never boil however hot the column runs — wrong, and wrong
/// conservatively, in that it moves no mass and invents no stream.
/// `FlashBoilOff` says the superheat leaves as vapour at `y = K·x` through a
/// vent to `Atmosphere` with nothing downstream of it — also wrong, because a
/// real unit condenses that vapour and recovers it (`docs/DEFERRED.md` B12,
/// B13). A plant is not more correct for choosing one; it is making a different
/// statement about what is being modelled.
///
/// **`Ok(None)` is the ordinary answer and means "no term here".** A model that
/// cannot evaluate a K-value, a holdup that is not boiling, and the `"none"`
/// fidelity all take it, and the engine leaves the holdup untouched — no vapour,
/// no vent flow, no temperature write. That is deliberately the same arithmetic
/// as a boil-off of zero: unlike `NodeSnapshot::cavitation`, where absent and
/// `false` are different claims, here there is nothing for a frontend to
/// misread.
///
/// **Pure, and a function of its arguments alone**, like every seam in this file
/// except `Controller`: the engine holds ONE model for every holdup on the
/// plant, so state on `self` would cross-seed two tanks.
pub trait BoilOffModel: Send {
    fn name(&self) -> &'static str;

    /// How much of `mass` boils off this tick, given the holdup's END-of-tick
    /// state.
    ///
    /// `composition` is MASS fractions, `pressure` is the holdup's own pressure
    /// (a vented tank's gas blanket is at `P_ATM`; it is passed rather than
    /// assumed so the caller owns the question), and `thermo` is the engine's
    /// thermodynamics — reached as an argument for the same reason
    /// `SeparationModel::separate` takes one: a K-value is a property of the
    /// fluid, not of this seam. `enthalpy` arrives on the same argument (M16.2):
    /// the flash fraction is an ENTHALPY RATIO, and under §20 the engine has one
    /// owner of what an enthalpy is. `enthalpy` arrives on the same argument (M16.2):
    /// the flash fraction is an ENTHALPY RATIO, and under §20 the engine has one
    /// owner of what an enthalpy is.
    ///
    /// **What makes an implementation of this sound, stated because one model
    /// satisfying it is not the same as the trait requiring it** (the M10.1
    /// lesson: a safety argument resting on there being ONE inhabitant expires
    /// silently, because the code does not change). A boil-off model asks the
    /// thermo model two questions that must agree — is this liquid below its
    /// bubble PRESSURE at its own temperature, and what TEMPERATURE does it boil
    /// at under its own pressure. `FlashBoilOff` uses the first to gate the
    /// second, and treats a disagreement as "nothing boils this tick" rather
    /// than as a fault, which is right only for a `ThermoModel` whose
    /// `bubble_pressure` and `k_value` come from ONE correlation.
    /// `TroutonThermo` does; three of this workspace's six implementations are
    /// test stubs that answer only some of the three methods. A model that
    /// derives the two answers independently can make a boil-off model swallow a
    /// real inconsistency, and the place to catch that is here, in the model,
    /// not in `Engine::tick`.
    ///
    /// # Errors
    /// `SimError` when the model can evaluate the term and it comes out
    /// impossible — a non-finite flash fraction, a vapour composition that does
    /// not normalise, a bubble temperature with no root in the model's bracket.
    /// A model that simply has no vapour–liquid equilibrium returns `Ok(None)`
    /// instead: "this fidelity cannot answer" is not a fault, it is fourteen of
    /// the sixteen shipped plants (§14 fork 7).
    #[allow(clippy::too_many_arguments)] // one argument per seam this model reads
    fn boil_off(
        &self,
        slate: &Slate,
        composition: &Composition,
        mass: Kg,
        temperature: Kelvin,
        pressure: Pascal,
        thermo: &dyn ThermoModel,
        enthalpy: &dyn EnthalpyModel,
    ) -> Result<Option<BoilOff>, SimError>;
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
    /// Pure: a function of `pass` alone, with no tick history of its own. That is
    /// what keeps a column's reference a clean hand calculation, and it is a
    /// contract, not an implementation note — a cascade may WARM-START from a
    /// previous profile (that changes the iteration count) but must never let one
    /// change the answer (DESIGN §5, fork 5).
    ///
    /// M9.3b built that warm start and the sentence above still holds literally,
    /// which was the reason for building it this way: the previous profile
    /// arrives as `ColumnPass::seed` and leaves as `Separation::profile`, so it is
    /// part of `pass` rather than state on `self`. `&self` is therefore still the
    /// right receiver, and a model that stashed a profile internally would be
    /// wrong twice over — it would falsify this sentence, and the engine holds
    /// ONE model for every column on the plant.
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

/// One control algorithm — the regulation seam (docs/DESIGN.md §10 fork 2).
///
/// **The project's first per-instance seam.** `FlowSolver`, `ThermoModel`,
/// `ReactionModel` and `SeparationModel` are engine-wide singletons, each chosen
/// once by one string in `[fidelity]`. A control algorithm is not that shape: one
/// plant can reasonably want a proportional loop on one tank and an integral loop
/// on another, and `[fidelity]`'s keys are per-engine by construction. So this is
/// selected per `[[controls]]` entry and boxed on the loop. Rule 2 is honoured
/// and it is the seam's ARITY that changed — the alternative, an enum matched
/// inside one shared `update`, is `if fidelity == Simple` wearing a hat.
///
/// **And it is the first seam whose impls own STATE.** Every other trait in this
/// file is a pure function of its arguments; a PI controller's integral term is
/// carried across ticks and the next answer depends on it, which is why `update`
/// takes `&mut self` and why §3a fork 5's "state is what turns an element into a
/// controller" is the line M8 crosses on purpose. `Debug` is a supertrait so
/// `PlantGraph` keeps its derive with a `Box<dyn Controller>` inside it.
pub trait Controller: Send + std::fmt::Debug {
    /// Human-readable identifier for snapshots/logs (e.g. "proportional").
    fn name(&self) -> &'static str;

    /// Compute the actuator position this loop should hold for the next tick.
    ///
    /// `measurement` is the plant state standing at the TOP of the tick — one
    /// `dt` older than the solve that follows it, because reading this tick's
    /// solve and writing an actuator is an algebraic loop (fork 3). Both it and
    /// `setpoint` carry their unit in their type.
    ///
    /// **What used to stand here — "and are the same type by construction, so the
    /// difference `ControlledValue::error` takes is always dimensionally honest"
    /// — expired in M10** (docs/DESIGN.md §12). That was a property of
    /// `ControlledValue` having ONE variant, not of the type: with a second, the
    /// two arguments can be a pressure and a level, and the subtraction would be
    /// Pascals minus metres. What actually keeps them matched is the engine —
    /// `run_control_loops` measures `setpoint.variable()`, and
    /// `Command::SetSetpoint` refuses a value whose variable disagrees with the
    /// loop's — with `error`'s own cross-variable `NaN` as the backstop behind
    /// them. **An impl may assume the pair matches; it may not assume the TYPE is
    /// what guarantees it**, and neither guard may be deleted as redundant.
    ///
    /// **The error term is `ControlledValue::error(measurement, setpoint)` and no
    /// implementation may compute its own.** That function owns the sign
    /// convention (positive = above setpoint), and an impl differencing the two
    /// itself would be free to disagree with the value a snapshot reader
    /// reconstructs from the same two reported numbers.
    ///
    /// The return is a **dimensionless** actuator position in `[0, 1]`, which is
    /// what `Command::SetValveOpening` already validates a valve opening to be.
    /// **The unit question this sentence used to defer arrived in M17** (docs/DESIGN.md
    /// §21): a cooler's actuated quantity is a duty in watts, and the answer is that
    /// the position stays a fraction — of the LOOP's declared `max_duty` — mapped
    /// to watts by `PlantGraph::set_actuator_position`, never by an impl. Clamping to that interval is the implementation's job, because
    /// saturation is exactly what M8.3's anti-windup has to know about; the engine
    /// re-checks the range and refuses rather than trusting it.
    ///
    /// `dt` is the fixed timestep. Unused by `ProportionalController`, and in the
    /// signature now so that the integral term needs no trait churn — the
    /// `ReactionModel::tau` precedent.
    ///
    /// # Errors
    /// `SimError` if the algorithm cannot produce a position — a non-finite
    /// measurement or setpoint, or (from M8.3) state that has gone non-finite.
    /// Rule 5: a controller that cannot answer says so, and never returns a
    /// plausible number for a valve to be driven to.
    fn update(
        &mut self,
        measurement: ControlledValue,
        setpoint: ControlledValue,
        dt: Seconds,
    ) -> Result<f64, SimError>;

    /// Set this controller's memory so that its NEXT `update` returns `output`.
    ///
    /// Two callers, one arithmetic, and that is the whole reason this is on the
    /// trait rather than inside one impl (docs/DESIGN.md §10 fork 4):
    ///
    /// - **Load.** `initial_output` is the loop's declared memory (fork 5), and
    ///   the integral term is *derived* from it here rather than declared beside
    ///   it — so there is exactly one way a loop's memory can be initialised and
    ///   no silent zero anywhere.
    /// - **MANUAL→AUTO.** The actuator holds whatever a human left it at, and a
    ///   loop taking over must not step it. Seeding from that position is what
    ///   makes the transfer bumpless, and it is the same back-calculation the
    ///   anti-windup clamp performs when it refuses to accumulate — which is why
    ///   fork 4 does not defer bumpless transfer to a slice after the integral.
    ///
    /// `measurement` and `setpoint` are the pair the next `update` will see, so
    /// the caller must read the measurement at the moment of transfer rather than
    /// reuse the loop's one-tick-old `last_measurement` — otherwise the seed is
    /// computed against a different error than it is spent against, and the
    /// transfer is bumpless only to the extent the level stopped moving.
    ///
    /// A stateless controller implements this as an explicit no-op. There is
    /// deliberately **no default body**: an impl with memory that forgot to seed
    /// it would inherit a silent nothing, which is the one failure this method
    /// exists to prevent.
    ///
    /// # Errors
    /// `SimError` if `output` is not a finite fraction in `[0, 1]`, or if the
    /// error term is non-finite. Rule 5: a controller that cannot seed its memory
    /// says so rather than carrying a NaN into the next tick.
    fn seed_from_output(
        &mut self,
        output: f64,
        measurement: ControlledValue,
        setpoint: ControlledValue,
    ) -> Result<(), SimError>;
}
